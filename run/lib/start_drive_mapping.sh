#!/bin/bash
# start_drive_mapping.sh - start everything one mapping drive needs on the Jetson, in one command,
#                          so that none of it depends on a network connection staying up.
#
# ONE OF THREE DIVERGED COPIES - read this first.
#   start_drive_mapping.sh   the live drive script (drives 3-10, Sept 2026): mapping a new area
#   start_drive_localise.sh  the same script plus a LOCALISE=1 mode (26 Sept 2026): find the robot on a saved map
#   start_drive_slam.sh      the same script plus a SLAM=1 mode (27 Sept 2026): continue a saved map
# The two variants were made by copying the live script and adding their own `if [ "$LOCALISE" = 1 ]` /
# `if [ "$SLAM" = 1 ]` blocks; small fixes made to the live script afterwards may be missing from a variant
# (each was re-staged by hand and checked by a checksum against the live file). THEY SHOULD BE MERGED INTO ONE
# SCRIPT WITH THREE MODES, but a merge cannot be tested without the robot and camera, so all three are shipped
# as they ran. The checksum self-check that compared a variant against the live file is not in this layout
# and was removed. Paths come from run/lib/paths.sh; run/mapping.sh, run/localise.sh and run/slam_session.sh
# are the intended entry points.
#
# WHY EVERYTHING IS DETACHED
#   Drive 1's programs ran in terminals opened over the network. The Jetson's
#   wifi card fell off mid-drive, nobody could reach the mapping to stop it,
#   and the Jetson later lost power with it still running - the map was never
#   shut down properly. Everything here is started DETACHED (setsid nohup):
#   it keeps running if every connection to the Jetson drops, and the
#   automatic stop (auto_stop_mapping.py) ends the mapping properly by itself.
#
#   *Plain terms: once this has been run, the Jetson can be left alone. It
#   maps the drive, and when told "park" it waits for the robot to settle on
#   the mark and switches the map off properly by itself.*
#
# WHAT IT STARTS, in order, checking each before the next
#   1. roscore                    (if none is running)
#   2. the ZED X camera, NEURAL   ALWAYS a fresh one: a camera left running is stopped first
#   3. the mapping                (record_mapping_run.launch, log fed to the monitor)
#   3b. the live map page         ($LIVE_MAP_URL - instead of RViz)
#   3e. the camera guard          (camera_guard.py: restarts a dead camera, raises $RECORDS_DIR/<run>/ALERT)
#   4. the progress line          (jobs page $JOBS_PAGE_URL)
#   3d. SVO=1 only: the camera recording ($REC/$RUN.svo2) + its guard (svo_guard.sh)
#   4b. the live timelapse recorder (drive_media_recorder.py -> $REC/media/)
#   5. the stop-on-"park" helper  (closes the map only when told "park")
#
# WHAT IT REFUSES
#   - a run id whose database already exists (never overwrites a drive)
#   - /use_sim_time true on the live master (it silently freezes a real run)
#   gate check to stop the recording [ we will move the recording files, free up the space
#   before the next drive ]", ENGINEERING_NOTES.md rule 15 as amended). It prints the free space for the
#   record; the space is made before each drive (drive 1's map reached 4.9 GB in 17 min).
#
#   usage:  start_drive.sh s2_static_02
#
# camera blended by an EKF, an extended Kalman filter). WITHOUT FUSION=1 THIS SCRIPT
# DOES EXACTLY WHAT IT DID BEFORE: every fusion step sits inside
# `if [ "${FUSION:-0}" = 1 ]`. With it (DESIGN.md section 6.1), in this order, the
# robot standing still throughout: 1a the bridge (below);
# 1b the blend (fused_odometry.launch: EKF + input conditioner); 1c check that only
# the blend publishes odom -> base_link; 3 the mapping with vo_publish_tf:=false (the
# camera tracker stops publishing that link); 3a the same check plus map -> odom from
# the map; 3c the fusion recording ($REC/fusion.bag); 6 a ready gate that waits, without
# giving up, for the gyroscope's false turn to be KEPT (a fresh status line, n >= 160
# readings) before the robot may move - only an override file it names lets a drive go
# ahead without it, and then it never prints "can now be left alone".
#   1a. the bridge receiver (robot_bridge_recv.py, from $TOOLS): it connects to the
#       robot's sender on port 8111 and republishes the robot's wheels and gyroscope
#       on this Jetson as /robot/*, re-timed to this Jetson's clock. REFUSES the drive
#       if the link is not UP within 30 s - there is NO silent camera-only fallback,
#       because drive 3 must not repeat drives 1-2's camera-only settings.
#       Robot address(es): ROBOT_ADDRS (comma list, tried in order), default = the
#       host part of LIDAR_PEER, else $ROBOT_WIFI_ADDR (the robot's WiFi address as of
#   The stop-on-"park" helper is told --fusion: after the map has closed it runs
#   stop_fusion.sh (recording, then blend, then bridge), or, if that is not deployed,
#   stops the bridge itself. Process numbers: $REC/fusion_pids (BRIDGE_PID=... lines).
#     FUSION=1 $TOOLS_DIR/start_drive.sh s2_static_03
#
# (SVO2) for the whole drive so its pictures can be replayed offline through the tracker. WITHOUT
# SVO=1 NOTHING CHANGES except that fusion.bag also records /rtabmap/odom_info_lite and the left/right
# camera_info. SVO_MODE picks the wrapper's compression (general/svo_compression, read at camera
# STARTUP only) and must be 1-4: 2 = H265 lossy (default), 1 = H264 lossy, 3/4 = H264/H265 "lossless"
# (really near-lossless). 0 (true lossless PNG) is refused: 64 MB/s measured = ~115 GB per 30 min.
# svo_guard.sh stops the recording when THIS drive's rtabmap exits or free space falls under
# SVO_FLOOR_MB (2500); stop_fusion.sh stops it a second time after the map closes.
# AFTER EVERY SVO=1 DRIVE: bash $TOOLS_DIR/restore_camera_mode.sh  (camera back to mode 0
# for the camera depth study, ENGINEERING_NOTES.md rule 16).
#   FUSION=1 SVO=1 $TOOLS_DIR/start_drive.sh s2_static_04
#
#   touch $WORK_DIR/PARK_<run>   -> waits for 45 s standing still (the map
#   needs a moment on the mark to recognise the start), then closes properly.
#   Standing still on its own never ends a drive.
# repository folders and addresses (REPO_ROOT, RECORDS_DIR, WORK_DIR, JOBS_DIR, TOOLS_DIR, *_ADDR): run/lib/paths.sh
. "$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)/paths.sh"
set -u
RUN="${1:?usage: start_drive.sh <run_id>}"
DB="$WORK_DIR/$RUN.db"
REC="$RECORDS_DIR/$RUN"
TOOLS="$TOOLS_DIR"
CAM_TOPIC=/zedx_front/zed_node/rgb/camera_info

step() { echo; echo "== $*"; }
die() { echo "   REFUSED: $*" >&2; exit 1; }

set +u      # ROS's setup scripts read unset variables
# The same environment an interactive shell on the Jetson gets: ROS, the
# built workspace ($CATKIN_WS, whose src links into the repo), ROS_IP, CUDA.
source "$SIDEWALK_ENV"
set -u

[ -e "$DB" ] && die "$DB already exists - pick the next run id rather than overwrite a drive"
# the camera guard (step 3e) - without it a camera crash goes unnoticed, as on drive 5
[ -r "$TOOLS/camera_guard.py" ] || die "$TOOLS/camera_guard.py is missing - deploy it first (camera_crash_recovery_2026-09-26/DEPLOY_NOTES.md)"
SVO_MODE="${SVO_MODE:-2}"
if [ "${SVO:-0}" = 1 ]; then
    case "$SVO_MODE" in
        1|2|3|4) ;;
        0) die "SVO_MODE=0 (true lossless) writes ~64 MB/s = ~115 GB per 30 min - it cannot fit on this disk" ;;
        *) die "SVO_MODE must be 1, 2, 3 or 4 (got '$SVO_MODE')" ;;
    esac
    [ -e "$REC/$RUN.svo2" ] && die "$REC/$RUN.svo2 already exists (a restart mid-drive?) - never record over it; see DEPLOY_NOTES.md 'restart mid-drive'"
    for f in svo_guard.sh restore_camera_mode.sh; do
        [ -r "$TOOLS/$f" ] || die "SVO=1: $TOOLS/$f is missing - deploy it first"
    done
    echo "   SVO=1: the camera's own recording, compression mode $SVO_MODE, to $REC/$RUN.svo2"
fi
# free space: printed for the record, never a reason to refuse (no storage gate, rule 15 amended)
FREE_MB=$(df -BM --output=avail "$HOME" | tail -1 | tr -dc 0-9)
echo "   Jetson disk: ${FREE_MB} MB free (a drive uses about 5 GB, or about 5.3 GB with FUSION=1)"
if [ "${FUSION:-0}" = 1 ]; then
    # ======================= FUSION=1 (drive 3 onward) ================================
    #   FUSION=1 $TOOLS_DIR/start_drive.sh s2_static_03
    # The robot's position is decided by an EKF (extended Kalman filter: a program that
    # blends several sensors into one best position, trusting each by how noisy it is)
    # fed by the robot's wheels, the robot's gyroscope and the camera, instead of by the
    # camera alone. The 2D map is still drawn from the camera's depth pictures; only WHO
    # says where the robot is changes. Every FUSION line of this script sits inside an
    # `if [ "${FUSION:-0}" = 1 ]` block, so without FUSION=1 it runs exactly as before.
    # The robot must stand still from here until step 6 says READY.
    # There is NO silent camera-only fallback: if a fusion piece fails, this refuses.
    for f in "$TOOLS/tf_one_parent_check.py" "$TOOLS/stop_fusion.sh"; do
        [ -r "$f" ] || die "FUSION=1: $f is missing - copy it there from this repository (tools/drive/, tools/robot_side/ or catkin_ws/src/sidewalk_slam/scripts/) first"
    done
    SSLAM=$(rospack find sidewalk_slam 2>/dev/null)
    [ -x "$SSLAM/scripts/ekf_input_conditioner.py" ] && [ -r "$SSLAM/launch/fused_odometry.launch" ] \
        && [ -r "$SSLAM/config/ekf_fused.yaml" ] \
        || die "FUSION=1: the blend is not installed in $SSLAM (ekf_input_conditioner.py, fused_odometry.launch, ekf_fused.yaml)"
    # tf_ok JSON [extra args]: the one-parent check PASSES and every checked link has only the
    # expected publisher (odom -> base_link: /ekf_fused; map -> odom: /rtabmap/rtabmap).
    tf_ok() {
        local js="$1"; shift
        python3 "$TOOLS/tf_one_parent_check.py" --seconds 5 --expect-parent base_link=odom \
            --require-frame base_link --json "$js" "$@" > "${js%.json}.txt" 2>&1 || return 1
        python3 - "$js" <<'EOF'
import json, sys
d = json.load(open(sys.argv[1]))
want = {("odom", "base_link"): ["/ekf_fused"], ("map", "odom"): ["/rtabmap/rtabmap"]}
bad = ["%s -> %s published by %s" % (e["parent"], e["child"], e["publishers"]) for e in d["edges"]
       if (e["parent"], e["child"]) in want and e["publishers"] != want[(e["parent"], e["child"])]]
for b in bad:
    print("   wrong publisher: " + b)
sys.exit(1 if bad else 0)
EOF
    }
    echo "   FUSION=1: wheels + gyroscope + camera blended by an EKF (blend in $SSLAM)"
fi
mkdir -p "$REC" "$WORK_DIR"

step "1. roscore"
if timeout 5 rostopic list >/dev/null 2>&1; then
    echo "   already running"
else
    setsid nohup roscore > "$REC/roscore.log" 2>&1 < /dev/null &
    for i in $(seq 1 20); do timeout 3 rostopic list >/dev/null 2>&1 && break; sleep 1; done
    timeout 5 rostopic list >/dev/null 2>&1 || die "roscore did not come up - see $REC/roscore.log"
    echo "   started"
fi
ST=$(rosparam get /use_sim_time 2>/dev/null || echo unset)
[ "$ST" = "true" ] && die "/use_sim_time is true on the live master - a real run would freeze"
echo "   /use_sim_time: $ST (must not be true)"
if [ "${FUSION:-0}" = 1 ]; then
    step "1a. FUSION: the bridge from the robot (wheels + gyroscope -> /robot/*)"
    RECV="$TOOLS/robot_bridge_recv.py"
    [ -f "$RECV" ] || die "$RECV is missing - copy tools/drive/robot_bridge_recv.py from this repository there"
    PEER_HOST=$(echo "${LIDAR_PEER:-}" | sed -E 's#^[a-z]+://##; s#[:/].*$##')
    ROBOT_ADDRS="${ROBOT_ADDRS:-${PEER_HOST:-$ROBOT_WIFI_ADDR}}"
    BPROG="$JOBS_DIR/${RUN}_bridge.progress"
    T_RECV=$(date +%s)
    setsid nohup python3 "$RECV" __name:=robot_bridge_recv \
        _robot:="$ROBOT_ADDRS" _port:=8111 _run:="$RUN" \
        > "$REC/bridge_recv.log" 2>&1 < /dev/null &
    RECV_PID=$!
    # same file and format stop_fusion.sh reads (it is sourced: NAME_PID=<number> lines)
    echo "BRIDGE_PID=$RECV_PID" >> "$REC/fusion_pids"
    UP=""
    for i in $(seq 1 30); do
        if [ -f "$BPROG" ] && [ "$(stat -c %Y "$BPROG")" -ge "$T_RECV" ] \
                && head -1 "$BPROG" | grep -q '^BRIDGE UP'; then UP=1; break; fi
        kill -0 "$RECV_PID" 2>/dev/null || break
        sleep 1
    done
    if [ -z "$UP" ]; then
        echo "   last bridge line: $(head -1 "$BPROG" 2>/dev/null)"
        kill -TERM "$RECV_PID" 2>/dev/null
        die "the robot's sender is not answering on ${ROBOT_ADDRS}:8111 - run robot_side.sh start on the robot first (or set ROBOT_ADDRS to the robot's current address). Starting without FUSION=1 would make this a camera-only drive."
    fi
    echo "   $(head -1 "$BPROG")"
    echo "   receiver pid $RECV_PID; clock log $JOBS_DIR/${RUN}_bridge_clock.csv"
fi
if [ "${FUSION:-0}" = 1 ]; then
    # (after the bridge is UP: the blend needs the wheels to start publishing)
    step "1b. the blend - EKF + input conditioner (fused_odometry.launch)"
    # the ready gate (step 6) believes only a conditioner status line written after this moment
    T_FUSED=$(date +%s)
    # learning starts now, not at step 6: a push during steps 1b-5 would go into the kept value
    echo "   HANDS OFF WHILE PARKED: nobody touches, pushes or lifts the robot while it is parked - now and at every stop (a slow turn with the wheels at zero is learned as gyroscope offset)"
    echo "   stand still from now until step 6 says READY"
    setsid nohup roslaunch sidewalk_slam fused_odometry.launch run:="$RUN" \
        > "$REC/fused_odometry.log" 2>&1 < /dev/null &
    FUSED_PID=$!
    echo "FUSED_PID=$FUSED_PID" >> "$REC/fusion_pids"
    if ! timeout 15 rostopic echo -n1 /fused/odometry > /dev/null 2>&1; then
        bash "$TOOLS/stop_fusion.sh" "$RUN" > "$REC/stop_fusion.log" 2>&1
        die "/fused/odometry silent after 15 s - see $REC/fused_odometry.log"
    fi
    echo "   publishing /fused/odometry"

    step "1c. pre-flight: ONE program says where the robot is, and it is the blend"
    if ! tf_ok "$REC/tf_check_1.json"; then
        bash "$TOOLS/stop_fusion.sh" "$RUN" > "$REC/stop_fusion.log" 2>&1
        die "odom -> base_link check failed - see $REC/tf_check_1.txt (two publishers, or not /ekf_fused)"
    fi
    echo "   PASS: odom -> base_link published by /ekf_fused only"
fi

step "2. camera (ZED X, NEURAL depth) - a FRESH camera for every drive"
# (exit -11, a segmentation fault) 12 minutes in; the earlier crash at 00:37 also came late in a long
# camera session. So no camera session spans two drives: any camera still running is stopped here,
# by process number, and a new one is started. Process checks read /proc/<pid>/comm and whole
# arguments, never "contains" (ENGINEERING_NOTES.md rule 8). comm is cut to 15 letters: "zed_wrapper_nod".
cam_launches() {   # roslaunch processes with zedx_front.launch as a whole argument (comm "roslaunch",
    local d A x c   # or "python3" with argv[1] = .../roslaunch when started through python3)
    for d in /proc/[0-9]*; do
        c=$(cat "$d/comm" 2>/dev/null)
        [ "$c" = roslaunch ] || [ "$c" = python3 ] || continue
        mapfile -d '' -t A < "$d/cmdline" 2>/dev/null || continue
        [ "$c" = python3 ] && { [ "${#A[@]}" -ge 2 ] && [ "$(basename -- "${A[1]}")" = roslaunch ] || continue; }
        for x in "${A[@]:1}"; do [ "$(basename -- "$x")" = zedx_front.launch ] && { echo "${d#/proc/}"; break; }; done
    done
}
cam_nodes() {      # the ZED driver itself: short name AND full program name
    local d A
    for d in /proc/[0-9]*; do
        [ "$(cat "$d/comm" 2>/dev/null)" = zed_wrapper_nod ] || continue
        mapfile -d '' -t A < "$d/cmdline" 2>/dev/null || continue
        [ "${#A[@]}" -ge 1 ] && [ "$(basename -- "${A[0]}")" = zed_wrapper_node ] && echo "${d#/proc/}"
    done
}
OLDL=$(cam_launches | tr '\n' ' '); OLDN=$(cam_nodes | tr '\n' ' ')
if [ -n "$OLDL$OLDN" ]; then
    echo "   a camera is already running (roslaunch: ${OLDL:-none}; driver: ${OLDN:-none})"
    # earlier drive, or the depth study's (rule 16). Three signs, any one refuses; CAMERA_FORCE_RESTART=1 overrides.
    if [ "${CAMERA_FORCE_RESTART:-0}" != 1 ]; then
        # (a) a camera recording file (.svo2) anywhere in the home folder written in the last 10 s
        GROW=$(find "$HOME" -maxdepth 5 -name '*.svo2' -newermt "@$(( $(date +%s) - 10 ))" 2>/dev/null | head -3)
        [ -n "$GROW" ] && die "a camera recording is being written right now: $GROW - stop it first (svo_guard.sh --stop <run>), or, if it is truly abandoned, run again with CAMERA_FORCE_RESTART=1"
        # (b) the camera's own diagnostics: "SVO Recording" reads NOT ACTIVE when idle (restore_camera_mode.sh)
        DIAG=$(timeout 6 rostopic echo /diagnostics 2>/dev/null | grep -A1 'key: "SVO Recording"' | grep 'value:' | head -1)
        if [ -n "$DIAG" ] && ! echo "$DIAG" | grep -q '"NOT ACTIVE"'; then
            die "the camera reports an SVO recording in progress ($DIAG) - stop it first, or run again with CAMERA_FORCE_RESTART=1"
        fi
        # (c) the camera was started in a recording mode by ANOTHER run and never restored
        MODE_LINE=$(cat "$WORK_DIR/camera_svo_mode.txt" 2>/dev/null)
        if echo "$MODE_LINE" | grep -q "(NOT default) - camera started by " \
                && ! echo "$MODE_LINE" | grep -q "camera started by $RUN;"; then
            die "camera_svo_mode.txt says the running camera belongs to another run: '$MODE_LINE' - finish that run (restore_camera_mode.sh), or run again with CAMERA_FORCE_RESTART=1"
        fi
    else
        echo "   CAMERA_FORCE_RESTART=1: the recording checks are skipped"
    fi
    echo "   no recording in progress - stopping it for a fresh one"
    for p in $OLDL; do kill -INT "$p" 2>/dev/null; done       # roslaunch's own clean shutdown
    for i in $(seq 1 30); do [ -z "$(cam_launches)$(cam_nodes)" ] && break; sleep 1; done
    for sig in TERM KILL; do
        [ -z "$(cam_launches)$(cam_nodes)" ] && break
        for p in $(cam_launches) $(cam_nodes); do echo "   still up after SIGINT: pid $p - SIG$sig"; kill -"$sig" "$p" 2>/dev/null; done
        sleep 5
    done
    [ -z "$(cam_launches)$(cam_nodes)" ] || die "the old camera would not stop (pids $(cam_launches) $(cam_nodes))"
    echo "   old camera stopped"
    sleep 3    # let the camera services release the camera before it is opened again
fi
# svo_compression:= needs the launch argument added in DEPLOY_NOTES.md (empty = the yaml's 0)
SVOARG=""
if [ "${SVO:-0}" = 1 ]; then
    SVOARG="svo_compression:=$SVO_MODE"
    # state file read by restore_camera_mode.sh and by anyone about to record for the camera depth study
    echo "$(TZ=America/Toronto date '+%F %T %Z') mode $SVO_MODE (NOT default) - camera started by $RUN; run restore_camera_mode.sh after the drive" > "$WORK_DIR/camera_svo_mode.txt"
fi
# camera_guard.py restarts the camera with exactly this command line and environment (it reads
# them from this roslaunch's /proc entry), so change them here only
#   map ~5 camera positions/s vs 7-12): the camera tracker engine, default GEN_2. TRACKER=GEN_1 selects the older engine.
#   Crash backtrace (libSegFault) and the camera guard stay on for both.
TRACKER="${TRACKER:-GEN_2}"
case "$TRACKER" in GEN_1|GEN_2) ;; *) echo "TRACKER must be GEN_1 or GEN_2" >&2; exit 2;; esac
#  - pos_tracking_mode:=GEN_1: the camera's own position tracker on its older engine; Stereolabs made GEN_1
#    the default for SDK 4.2.x "as a workaround for the random crash issue caused by GEN_2". Needs the
#    pos_tracking_mode launch argument in sidewalk_perception/zedx_front.launch (deploy that FIRST).
#  - LD_PRELOAD libSegFault: if the driver still crashes, camera.log gets a backtrace naming the library.
#    The camera guard copies this environment on a restart, so restarts keep it too.
CAM_LOG_START=$(( $(wc -l < "$REC/camera.log" 2>/dev/null || echo 0) + 1 ))
DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority \
LD_PRELOAD=/lib/aarch64-linux-gnu/libSegFault.so SEGFAULT_SIGNALS="segv abrt bus ill fpe" setsid nohup \
    roslaunch sidewalk_perception zedx_front.launch depth_mode:=NEURAL pos_tracking_mode:=$TRACKER $SVOARG \
    >> "$REC/camera.log" 2>&1 < /dev/null &
CAM_LAUNCH_PID=$!
echo -n "   waiting for images "
for i in $(seq 1 18); do
    timeout 5 rostopic echo -n1 "$CAM_TOPIC" >/dev/null 2>&1 && break
    echo -n "."
done
echo
timeout 8 rostopic echo -n1 "$CAM_TOPIC" >/dev/null 2>&1 \
    || die "no camera images after 90 s - see $REC/camera.log"
echo "   publishing (camera roslaunch pid $CAM_LAUNCH_PID, driver pid $(cam_nodes | tr '\n' ' '))"
# refuse unless the running camera really uses the chosen tracker engine $TRACKER (both the parameter and the driver's own line)
PTM=$(rosparam get /zedx_front/zed_node/pos_tracking/pos_tracking_mode 2>/dev/null || echo unset)
[ "$PTM" = "$TRACKER" ] || die "camera tracker engine parameter is '$PTM', not $TRACKER - is the new zedx_front.launch deployed? (camera_segfault_2026-09-26/DEPLOY.md)"
tail -n +"$CAM_LOG_START" "$REC/camera.log" | grep -aqE "Positional tracking mode[[:space:]]+-> GEN ${TRACKER#GEN_}" \
    || die "camera.log does not show 'Positional tracking mode -> GEN ${TRACKER#GEN_}' for this camera start - see $REC/camera.log"
echo "   camera tracker engine: $TRACKER (parameter and driver log agree)"
if [ "${SVO:-0}" = 1 ]; then
    # the wrapper reads its compression ONCE, at camera start (zed_wrapper_nodelet.cpp:1178):
    # a camera already running with another mode must be restarted, not reconfigured
    CM=$(rosparam get /zedx_front/zed_node/general/svo_compression 2>/dev/null || echo unset)
    [ "$CM" = "$SVO_MODE" ] || die "the running camera records with svo_compression=$CM, not $SVO_MODE. Restart it first (DEPLOY_NOTES.md, 'camera restart'), then run this again"
    echo "   camera compression setting: $CM (as asked)"
fi

step "3. mapping"
if [ "${FUSION:-0}" = 1 ]; then
    # the same command as below, plus vo_publish_tf:=false: the camera tracker stops publishing
    # odom -> base_link, because the blend (step 1b) owns it. KEEP THE TWO COMMANDS IN STEP.
    setsid nohup roslaunch sidewalk_slam record_mapping_run.launch \
        run_id:="$RUN" database_path:="$DB" rtabmap_log:="$REC/mapping.log" \
        ground_truth:=false use_sim_time:=false wait_imu_to_init:=true \
        force_3dof:=true rviz:=false vo_publish_tf:=false \
        > "$REC/mapping.log" 2>&1 < /dev/null &
else
setsid nohup roslaunch sidewalk_slam record_mapping_run.launch \
    run_id:="$RUN" database_path:="$DB" rtabmap_log:="$REC/mapping.log" \
    ground_truth:=false use_sim_time:=false wait_imu_to_init:=true \
    force_3dof:=true rviz:=false \
    > "$REC/mapping.log" 2>&1 < /dev/null &
fi
RT=""
for i in $(seq 1 45); do
    RT=$(ps -eo pid=,comm= | awk '$2 == "rtabmap" {print $1}')
    [ -n "$RT" ] && break
    sleep 2
done
[ -n "$RT" ] || die "the mapping did not start within 90 s - see $REC/mapping.log"
echo "   running (pid $RT), writing $DB"
setsid nohup bash -c 'sudo -n tegrastats --interval 1000 --logfile "$1" & T=$!; while kill -0 "$2" 2>/dev/null; do sleep 5; done; sudo -n pkill -P "$T" -x tegrastats' _ "$REC/tegrastats.log" "$RT" > /dev/null 2>&1 < /dev/null &   # power/temperature log for this drive, stops when its mapping (pid $RT) ends (2026-09-26: drive 7 had none)
if [ "${FUSION:-0}" = 1 ]; then
    step "3a. pre-flight: the blend owns odom -> base_link, the map owns map -> odom"
    OK=0
    for i in 1 2 3; do
        sleep 5
        tf_ok "$REC/tf_check_2.json" --expect-parent odom=map && { OK=1; break; }
    done
    if [ "$OK" != 1 ]; then
        # close the map PROPERLY (the existing close-now switch, read by auto_stop_mapping.py),
        # then the blend after it
        touch "$WORK_DIR/STOP_$RUN"
        setsid nohup python3 "$TOOLS/auto_stop_mapping.py" --run "$RUN" --fusion > "$REC/autostop.log" 2>&1 < /dev/null &
        die "position-tree check failed after the mapping started - see $REC/tf_check_2.txt. The map is being closed (STOP_$RUN)."
    fi
    echo "   PASS: odom -> base_link by /ekf_fused only; map -> odom by /rtabmap/rtabmap only"
fi

step "3b. the live map in a browser - $LIVE_MAP_URL (camera | LiDAR)"
# live_map_server.py (August): one small PNG of the growing map, refreshed every
# shows the robot's live LiDAR view beside it (lidar_live_map.py on the robot,
# fetched through this Jetson). Restarted with each drive so the LiDAR panel
# points at the robot's current address.
# The robot's WiFi address comes from DHCP and CAN change on a restart (it moved
#   LIDAR_PEER=http://<robot address>:8095 bash start_drive.sh RUN
PEER="${LIDAR_PEER:-http://$ROBOT_WIFI_ADDR:8095}"
for d in /proc/[0-9]*; do
    mapfile -d '' -t A < "$d/cmdline" 2>/dev/null || continue
    [ "${#A[@]}" -ge 2 ] && [ "$(basename -- "${A[0]}")" = python3 ] \
        && [ "$(basename -- "${A[1]}")" = live_map_server.py ] && kill -TERM "${d#/proc/}"
done
sleep 2
setsid nohup rosrun sidewalk_slam live_map_server.py _port:=8095 _peer:="$PEER" \
    > "$RECORDS_DIR/live_map_server.log" 2>&1 < /dev/null &
sleep 4
if ss -ltn | grep -q ':8095 '; then
    echo "   serving; LiDAR panel from $PEER"
    CODE=$(curl -s -m 3 -o /dev/null -w "%{http_code}" "$PEER/stats.json" || true)
    if [ "$CODE" = 200 ]; then
        echo "   LiDAR view reachable"
    else
        echo "   !! LiDAR panel NOT reachable at $PEER (answer: ${CODE:-none}). The camera map is"
        echo "      unaffected. If robot_side.sh start already ran, the robot's address has probably"
        echo "      changed: on the robot run  ip -4 -brief addr show wlan0  and restart with"
        echo "      LIDAR_PEER=http://<that address>:8095"
    fi
else
    echo "   !! did not start - see $RECORDS_DIR/live_map_server.log (the drive is unaffected)"
fi

if [ "${FUSION:-0}" = 1 ]; then
    step "3c. the fusion recording (Jetson internal disk, ~0.25 GB per 20 min)"
    # DESIGN.md section 10, Tier 1: enough to re-run the blend offline at full camera rate.
    setsid nohup rosrun rosbag record -O "$REC/fusion.bag" \
        /rtabmap/odom /robot/wheel_odom /robot/imu /robot/imu_raw /robot/ekf_odom /robot_bridge/status \
        /ekf_in/wheel_odom /ekf_in/imu /ekf_in/vo_odom /fused/odometry /tf /tf_static /diagnostics \
        /zedx_front/zed_node/imu/data /rtabmap/odom_info_lite \
        /zedx_front/zed_node/left/camera_info /zedx_front/zed_node/right/camera_info \
        > "$REC/fusion_bag.log" 2>&1 < /dev/null &
    RECORD_PID=$!
    echo "RECORD_PID=$RECORD_PID" >> "$REC/fusion_pids"
    sleep 5
    S1=$(stat -c %s "$REC/fusion.bag.active" 2>/dev/null || echo 0)
    sleep 10
    S2=$(stat -c %s "$REC/fusion.bag.active" 2>/dev/null || echo 0)
    if [ "$S2" -gt "$S1" ] && [ "$(cat /proc/$RECORD_PID/comm 2>/dev/null)" = record ]; then
        echo "   recording (pid $RECORD_PID), growing: $S1 -> $S2 bytes in 10 s"
    else
        echo "   !! fusion.bag is NOT growing ($S1 -> $S2 bytes; pid $RECORD_PID is '$(cat /proc/$RECORD_PID/comm 2>/dev/null)')"
        echo "      the map is unaffected, but the blend cannot be re-run offline - see $REC/fusion_bag.log"
    fi
fi
if [ "${SVO:-0}" = 1 ]; then
    step "3d. the camera recording (SVO2, Jetson internal disk) + its guard"
    # With /rtabmap/odom_info_lite and camera_info in fusion.bag (step 3c) this lets drive 4's tracker be
    # re-run offline (camera fix 3). The wrapper FALLS BACK SILENTLY through H265 -> H264 -> LOSSLESS if
    # a mode is refused (zed_wrapper_nodelet.cpp:4672-4693); LOSSLESS would fill the disk in ~1.5 min,
    # so the reply is read and anything but the asked mode is stopped at once.
    SVOF="$REC/$RUN.svo2"
    G="$TOOLS/svo_guard.sh"
    WANT=$(case "$SVO_MODE" in 1) echo H264;; 2) echo H265;; 3) echo H264_LOSSLESS;; 4) echo H265_LOSSLESS;; esac)
    ANS=$(timeout 15 rosservice call /zedx_front/zed_node/start_svo_recording "svo_filename: '$SVOF'" 2>&1 | tr '\n' ' ')
    echo "   answer: $ANS"
    if ! echo "$ANS" | grep -q "Recording started ($WANT)"; then
        bash "$G" --stop "$RUN" > "$REC/svo_stop_at_start.log" 2>&1
        echo "   !! NOT recording as $WANT - stopped (see $REC/svo_stop_at_start.log). The map is unaffected; this drive has NO camera recording"
    else
        T_G=$(date +%s)
        # the guard watches THIS drive's mapping process (RT, step 3), not any rtabmap
        setsid nohup nice -n 19 bash "$G" "$RUN" "$RT" > "$REC/svo_guard.log" 2>&1 < /dev/null &
        GPID=$!
        S1=$(stat -c %s "$SVOF" 2>/dev/null || echo 0); sleep 12
        S2=$(stat -c %s "$SVOF" 2>/dev/null || echo 0)
        GP="$JOBS_DIR/${RUN}_svo.progress"
        # guard alive = its process number still runs bash with svo_guard.sh as the script (whole names)
        GOK=""
        if [ -r "/proc/$GPID/cmdline" ]; then
            mapfile -d '' -t GA < "/proc/$GPID/cmdline"
            [ "${#GA[@]}" -ge 2 ] && [ "$(basename -- "${GA[1]}")" = svo_guard.sh ] && GOK=1
        fi
        if [ -n "$GOK" ] && [ -f "$GP" ] && [ "$(stat -c %Y "$GP")" -ge "$T_G" ]; then
            echo "   growing: $(( (S2 - S1) / 1000000 )) MB in 12 s; guard pid $GPID watching mapping pid $RT; jobs page line ${RUN}_svo"
        else
            # no working guard = nothing would stop the recording: stop it now (checked), keep the map
            # kill only if that number is still our guard (a number can be reused by another program)
            [ -n "$GOK" ] && kill -TERM "$GPID" 2>/dev/null
            bash "$G" --stop "$RUN" > "$REC/svo_stop_at_start.log" 2>&1
            echo "   !! the guard is not running or wrote no progress line - camera recording STOPPED (see $REC/svo_guard.log). The map is unaffected"
        fi
    fi
fi
step "3e. the camera guard - restarts a dead camera and raises the alarm"
# pictures arriving? If not: $RECORDS_DIR/$RUN/ALERT (one line, the watcher's phone notification),
# "!! CAMERA DOWN" on the jobs page and the live map page, and a restart with the same command
# (at most 5). It stops by itself when THIS drive's mapping (pid $RT) exits. NOT niced: a camera it
# restarts inherits its priority.
setsid nohup python3 "$TOOLS/camera_guard.py" --run "$RUN" --rtabmap-pid "$RT" \
    --launch-pid "$CAM_LAUNCH_PID" --svo "${SVO:-0}" \
    > "$REC/camera_guard.out" 2>&1 < /dev/null &
CG_PID=$!
echo "CAMGUARD_PID=$CG_PID" >> "$REC/fusion_pids"   # for the record; stop_fusion.sh does not need it
CGP="$JOBS_DIR/${RUN}_camera.progress"
CGOK=""
for i in $(seq 1 15); do
    sleep 1
    [ -f "$CGP" ] && grep -q ' ok, pictures' "$CGP" && { CGOK=1; break; }
done
if [ -n "$CGOK" ]; then
    echo "   $(head -1 "$CGP")"
    echo "   guard pid $CG_PID; jobs page line ${RUN}_camera; alert file $REC/ALERT"
else
    echo "   !! the camera guard is NOT watching: $(head -1 "$CGP" 2>/dev/null || echo 'no line') - see $REC/camera_guard.log"
    echo "      the drive goes on, but a camera crash would again go unnoticed"
fi

step "4. progress line for the jobs page"
setsid nohup python3 "$TOOLS_DIR/slam_progress.py" \
    --run "$RUN" --expect-minutes 20 > "$REC/progress.log" 2>&1 < /dev/null &
echo "   started"

step "4b. live timelapse - the live map page's pictures every 3 s (rule 22)"
# drive_media_recorder.py: saves camera map + LiDAR panel from :8095 into $REC/media/, stops by
# itself when the drive closes (the stop-on-"park" helper says "complete", or says "FAILED" and
# the mapping has ended; or no rtabmap for 120 s), then builds $REC/media/${RUN}_timelapse.mp4
# (a video file). Lowest processor and disk priority; it only asks the page for its pictures.
# Stop it early, safely (checks the process number is really the recorder first):
#   python3 "$TOOLS/drive_media_recorder.py" --run "$RUN" --stop
MEDIA_REC="$TOOLS/drive_media_recorder.py"
if [ -r "$MEDIA_REC" ]; then
    setsid nohup nice -n 19 python3 "$MEDIA_REC" --run "$RUN" \
        > "$REC/media_recorder.out" 2>&1 < /dev/null &
    sleep 1   # only to show its first line; the drive never waits on the recorder
    echo "   $(head -1 "$JOBS_DIR/${RUN}_media.progress" 2>/dev/null || echo "!! no ${RUN}_media line yet - see $REC/media_recorder.out")"
else
    echo "   !! $MEDIA_REC is missing - NO timelapse will be recorded for this drive (the map is unaffected)"
fi

step "5. stop-on-\"park\" helper (never stops on stillness alone)"
if [ "${FUSION:-0}" = 1 ]; then
    # --fusion: after the map has closed, also stop what $REC/fusion_pids lists (stop_fusion.sh)
    setsid nohup python3 "$TOOLS/auto_stop_mapping.py" --run "$RUN" --fusion \
        > "$REC/autostop.log" 2>&1 < /dev/null &
else
setsid nohup python3 "$TOOLS/auto_stop_mapping.py" --run "$RUN" \
    > "$REC/autostop.log" 2>&1 < /dev/null &
fi
sleep 3
tail -1 "$REC/autostop.log" | sed 's/^/   /'
if [ "${FUSION:-0}" = 1 ]; then
    echo "   after the map closes, auto_stop_mapping.py --fusion stops the fusion recording, the blend"
    echo "   and the bridge, in that order (by hand, if ever needed:  bash $TOOLS/stop_fusion.sh $RUN)"

    step "6. ready gate - DO NOT MOVE THE ROBOT until this says READY"
    echo "   HANDS OFF WHILE PARKED: nobody touches, pushes or lifts the robot while it is parked - now and at every stop (a slow turn with the wheels at zero is learned as gyroscope offset)"
    # the conditioner measures the gyroscope's false turn while the wheels read exactly zero.
    # READY only once it has KEPT a value - the one that stays in use after the robot moves - in
    # a FRESH status file. Its running value (the "bias" field) is NOT enough: until kept, a WiFi
    # began (T_FUSED, step 1b) and at most 30 s ago (the conditioner writes every 10 s), so a line
    # left by an earlier attempt, or by a conditioner that has died, can never pass; (2) the kept
    # value needs n >= 160 = 0.8 x 10 Hz x 20 s, the fewest readings the conditioner can ever keep
    # whatever the gyroscope's rate (a fixed n >= 390 meant 20 s only at 19.5 Hz); (3) the gate
    # never gives up: without READY it says DO NOT DRIVE and keeps waiting until READY, or until
    # the operator creates the override file it names; only READY leads to "left alone" below.
    # kept_gate LINE: prints "<offset> <n>" from the line's "kept <offset>deg/min(n=<n>)" field and
    # succeeds only if that field is there, well formed, and n >= 160.
    kept_gate() {
        local v
        v=$(printf '%s\n' "$1" | sed -n 's/.* kept \([-+][0-9]\{1,3\}\.[0-9]\{2\}\)deg\/min(n=\([0-9]\{1,7\}\))\( .*\)*$/\1 \2/p' | head -1)
        [ -n "$v" ] || return 1
        echo "$v"
        [ "${v#* }" -ge 160 ]
    }
    # status_fresh FILE SINCE: prints the file's age in seconds ("no file" if absent); succeeds only
    # if it was written at or after SINCE (epoch seconds) and at most 30 s ago.
    status_fresh() {
        local m now
        m=$(stat -c %Y "$1" 2>/dev/null) || { echo "no file"; return 1; }
        now=$(date +%s)
        echo "$((now - m))"
        [ "$m" -ge "$2" ] && [ $((now - m)) -le 30 ]
    }
    # fusion_ready FILE SINCE: READY = a fresh status file (status_fresh) whose kept field passes
    # kept_gate; prints kept_gate's "<offset> <n>".
    fusion_ready() {
        status_fresh "$1" "$2" > /dev/null || return 1
        kept_gate "$(head -1 "$1" 2>/dev/null)"
    }
    CP="$JOBS_DIR/${RUN}_ekf_inputs.progress"
    OVR="$WORK_DIR/DRIVE_WITHOUT_OFFSET_$RUN"
    T_GATE=$(date +%s)
    KV=""
    KEPT_OK=""
    W=0
    while :; do
        if KV=$(fusion_ready "$CP" "$T_FUSED"); then KEPT_OK=1; break; fi
        # the override counts only if created (or touched) after this gate began
        if [ -f "$OVR" ] && [ "$(stat -c %Y "$OVR" 2>/dev/null || echo 0)" -ge "$T_GATE" ]; then break; fi
        if [ "$W" -eq 40 ]; then
            echo "   $(head -1 "$CP" 2>/dev/null || echo 'no conditioner status line yet')"
            echo "   status file age: $(status_fresh "$CP" "$T_FUSED") s (READY needs one written in the last 30 s, after this start)"
            # the reason: a fresh file with no kept value, or a file too old to believe (round-3 review)
            if status_fresh "$CP" "$T_FUSED" > /dev/null; then
                WHY="no gyroscope offset KEPT yet"
            else
                WHY="the status file is not fresh ($(status_fresh "$CP" "$T_FUSED")) - the conditioner may have stopped, or cannot write $JOBS_DIR; see $REC/fused_odometry.log"
            fi
            echo "   ################################################################################"
            echo "   ###  DO NOT DRIVE.  NOT READY after 40 s.                                   ###"
            echo "   ###  why: $WHY"
            echo "   ###  Keep the robot still, hands off. This script keeps waiting for READY.  ###"
            echo "   ###  To drive anyway WITHOUT the gyroscope correction (the blend's heading  ###"
            echo "   ###  then drifts about 5 deg/min), create this override file:              ###"
            echo "   ###     touch $OVR"
            echo "   ################################################################################"
        elif [ "$W" -gt 40 ] && [ $(( (W - 40) % 30 )) -eq 0 ]; then
            echo "   DO NOT DRIVE - still NOT READY after ${W} s (status file age $(status_fresh "$CP" "$T_FUSED") s; override: touch $OVR)"
            echo "      $(head -1 "$CP" 2>/dev/null || echo 'no conditioner status line yet')"
        fi
        W=$((W + 1))
        sleep 1
    done
    B="${KV% *}"
    N="${KV#* }"
    echo "   $(cat "$CP" 2>/dev/null || echo 'no conditioner status line yet')"
    if [ -n "$KEPT_OK" ]; then
        echo "   READY: gyroscope offset ${B} deg/min KEPT over n=$N readings. The robot may move."
        awk -v b="${B:-0}" 'BEGIN {exit !(b < -1 || b > 1)}' \
            && echo "   !! the offset is outside +-1 deg/min: the gyro re-zero (robot_side.sh) probably did not take"
        echo "   a 30 s stop every ~5 minutes of driving lets the blend re-measure the gyroscope"
    else
        # NOT READY and the operator created the override: go ahead, but never say "left alone"
        echo "$(date '+%F %T %Z') OVERRIDE $OVR - driving without a kept gyroscope offset; status: $(head -1 "$CP" 2>/dev/null)" >> "$REC/gate_override.txt"
        echo "   !! OVERRIDE ($OVR): NOT READY, and the drive goes ahead WITHOUT a kept gyroscope offset."
        echo "      The blend's heading drifts about 5 deg/min until a stop of 20 s (hands off) is kept."
        echo "      Logged in $REC/gate_override.txt"
        echo
        echo "== $RUN is mapping WITHOUT a kept gyroscope offset (override). Stop for 20 s, hands off, as soon as you can."
        echo "   when the robot is on the mark, say 'park'; that runs:  touch $WORK_DIR/PARK_$RUN"
        echo "   (something wrong - close now, no hold:              touch $WORK_DIR/STOP_$RUN)"
        echo "   after it stops, the check prints in:  $REC/autostop.log"
        echo "   FUSION: bridge line on the jobs page as ${RUN}_bridge ($JOBS_PAGE_URL);"
        echo "           it stops by itself after the map has closed (see $REC/fusion_pids)"
        exit 0
    fi
fi

echo
echo "== $RUN is mapping. The Jetson can now be left alone."
echo "   when the robot is on the mark, say 'park'; that runs:  touch $WORK_DIR/PARK_$RUN"
echo "   (something wrong - close now, no hold:              touch $WORK_DIR/STOP_$RUN)"
echo "   after it stops, the check prints in:  $REC/autostop.log"
if [ "${FUSION:-0}" = 1 ]; then
    echo "   FUSION: bridge line on the jobs page as ${RUN}_bridge ($JOBS_PAGE_URL);"
    echo "           it stops by itself after the map has closed (see $REC/fusion_pids)"
fi
