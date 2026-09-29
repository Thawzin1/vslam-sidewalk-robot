#!/bin/bash
# replay_lidar.sh - turn a recorded drive's LiDAR packets into a LiDAR map, offline.
#
# Runs AFTER a drive, ON THE ROBOT, from its card, on an ISOLATED ROS master
# (port 11312) so it cannot disturb the live one on 11311 that drives the
# sensors and the wheels. Record once, replay many: the same bag gives the same
# input to every configuration, so any difference in the output is caused by
# the configuration. Nothing of the colleague's is modified (ENGINEERING_NOTES.md rule 18).
#
# TWO MODES
#   colleague   (default, unchanged) the colleague's own pipeline:
#               self_navigation 3dreplay_pipeline.launch sensor:=helios.
#               Its map takes the wheel+IMU estimate /odometry/filtered as its
#               path and made ZERO loop closures on drives 1 and 2 - kept for
#               comparison, not as the reference.
#               -> <card>/slam_series2/<run>/  rtab_helios.db  lidar.tum  wheel.tum
#                  lidar_map.png/.npz  replay.log  products.log
#
#   reference   OUR lidar_reference.launch (tools/ on the card): the colleague's
#               decoder, restamp and body filter unchanged, then LiDAR scan-matching
#               odometry (wheel+IMU used only as the first guess of each 0.1 s step)
#               and a map that closes loops. One folder per VARIANT, written once:
#               -> <card>/slam_series2/<run>/lidar_ref_<variant>/
#                  rtab_lidar_ref.db   the map (stays on the card; cloud backup)
#                  lidar_ref.tum       its CORRECTED path, one pose per node (col 9 = node id)
#                  lidar_ref_odom.tum  the LiDAR odometry alone, before loop closures (~10 Hz)
#                  closures.tsv        every accepted loop closure and how far it pulled
#                  odom_health.json    scans seen / lost, timing, log warnings counted
#                  lidar_ref_map.png/.npz   its walls, drawn from the corrected positions
#                  provenance.txt, params.args, params_used.yaml, rtabmap_params_robot.txt,
#                  bag_tf_check.txt, stamp_probe.csv/.log, scan_counts.json,
#                  odom_lidar_ref.bag, replay.log, play.log, products.log
#               The variant's values are tools/lidar_ref_params/<variant>.args: one
#               name:=value per line, overriding a default of lidar_reference.launch.
#               v01.args holds no values: v01 IS the launch file's defaults.
#
# WHAT THE REFERENCE IS: an independent estimate, not ground truth (ENGINEERING_NOTES.md
# rule 19). Say "agreement with an independent LiDAR estimate", never "error".
#
# TRAPS THIS EXISTS TO AVOID
#   1. /use_sim_time must be true on the replay master, or RTAB-Map receives
#      It is set ONLY on the isolated master: a leftover `true` on the live
#      master silently freezes a real run.
#   2. The map must be CLOSED before anything reads it - its corrected path is
#      written only at shutdown. So the mapping program is interrupted by PID
#      (exact name `rtabmap` AND running on the 11312 master, never a substring
#      search) and waited for with NO deadline - a half-closed map is worse than
#      a slow one. The progress line keeps saying how long it has been waiting.
#   3. It never deletes and never overwrites. An existing map refuses the run. A
#      variant folder left by an attempt that failed before its map existed is
#      MOVED aside (renamed .failed-<time>), never removed. reference only (added
#      check, trap 8), the mapping program has already created its map file, so
#      the folder is renamed .failed-<time> as soon as everything is closed. A
#      retry of the same variant then starts clean (ENGINEERING_NOTES.md rule 14: re-runs
#      need no one to tidy up first), and the failed attempt stays for diagnosis.
#   4. reference only: the bag's /tf is played UNFILTERED - the body filter
#      needs the wheel links from it and the scan matching needs odom->base_link
#      from it. Our nodes add one NEW frame (odom_lidar_ref) above odom and
#      nothing else; bag_tf_check.py confirms on THIS bag, before launch, that
#      nothing in it is already odom's parent.
#   5. reference only: roslaunch silently IGNORES an argument name it does not
#      declare, so a misspelt setting would vanish without a word (tested
#   6. A replay writes a map whose final size is not known when it starts, so it
#      reserves space like a recording (ENGINEERING_NOTES.md rule 15), not like a transfer.
#      (Grid/RayTracing: mark every square a laser line of sight crosses as free
#      floor). With Grid/3D=true that needs RTAB-Map built WITH OctoMap (the 3D
#      map library); without it RTAB-Map prints one warning and silently skips it
#      (LocalGridMaker.cpp:518-558, 0.21.10). A variant that sets
#      map_grid_ray_tracing:=true is refused unless "rtabmap --version" says
#      "With OctoMap: true".
#      direction each small patch of wall faces) - without them it silently drops
#      point-to-plane matching and corridor handling (ENGINEERING_NOTES.md section 2.8). The
#      launch file used to switch them off by accident (see its icp_odometry
#      comment). The bag is not played until the odometry's own start-up line in
#      replay.log reads "IcpOdometry: scan_normal_k = N" with N > 0. rosconsole
#      (ROS's C++ logging) is set to write each line at once
#      (ROSCONSOLE_STDOUT_LINE_BUFFERED=1), or the line could sit in a buffer.
#      where the floor is - Grid/MaxGroundHeight, a height in base_link (the
#      robot's body frame), set about 0.10 m above the floor. That value was
#      worked out from the Husky's description, not from a drive. A variant that
#      sets map_grid_ground_is_obstacle:=false with a non-zero
#      map_grid_max_ground_height is refused unless THIS bag's /tf_static puts
#      the floor (base_footprint) at (max_ground_height - 0.10) +- 0.03 m.
#      If the bag has no base_footprint, the floor can be confirmed another way
#      (a slice of the parked scans) and given by hand: FLOOR_Z_BY_HAND=-0.13
#      (metres in base_link); that value and the fact it was typed are written
#      to provenance.txt.
#      Plain terms: v02 is told "the floor is 13 cm below the robot's body
#      centre". Before it runs, the recording itself must agree.
#
# CPU: runs after the drive, not during it. Reference mode runs at nice 10 so
# the robot's own driving stack (it is the ROS master for the wheels) always
# wins the processor; odom_health.json says whether the replay kept up.
#
#   usage:  ./replay_lidar.sh s2_static_01                        (colleague, as before)
#           ./replay_lidar.sh s2_static_02 reference v01           (our reference)
#           RATE=0.5 ./replay_lidar.sh s2_static_02 reference v01b (slower, if scans were dropped)
# repository folders and addresses (REPO_ROOT, RECORDS_DIR, WORK_DIR, JOBS_DIR, TOOLS_DIR, *_ADDR): run/lib/paths.sh
. "$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)/../../run/lib/paths.sh"
set -u
RUN="${1:?usage: replay_lidar.sh <run_id> [colleague | reference <variant>]}"
MODE="${2:-colleague}"
VARIANT="${3:-}"
D="${LIDAR_CARD_DIR:-/media/administrator/USB Drive/slam_series2}"   # override ONLY for testing
T="$D/tools"
BAG="$D/${RUN}_lidar.bag"
RATE="${RATE:-1.0}"
NEED_GB=4          # free space demanded before a map is written (rule 15: like a recording)
FLOOR_MARGIN=0.10  # trap 9: Grid/MaxGroundHeight sits this far above the floor (REPORT.md section 4.3)
FLOOR_TOL=0.03     # trap 9: how far the recorded floor may be from where the values file assumes it

case "$MODE" in
colleague)
    OUT="$D/$RUN"; DB="$OUT/rtab_helios.db"
    PROG="$JOBS_DIR/${RUN}_lidar_replay.progress"; TAG="${RUN}_LIDAR_REPLAY" ;;
reference)
    [[ "$VARIANT" =~ ^[A-Za-z0-9_]+$ ]] || { echo "reference needs a variant name of letters, digits, _ (e.g. v01)" >&2; exit 2; }
    OUT="$D/$RUN/lidar_ref_$VARIANT"; DB="$OUT/rtab_lidar_ref.db"
    ARGF="$T/lidar_ref_params/$VARIANT.args"; LAUNCHF="$T/lidar_reference.launch"
    PROG="$JOBS_DIR/${RUN}_lidar_ref_${VARIANT}.progress"; TAG="${RUN}_LIDAR_REF_${VARIANT}" ;;
*)  echo "unknown mode '$MODE' (colleague | reference <variant>)" >&2; exit 2 ;;
esac
mkdir -p "$JOBS_DIR"
say() { echo "$1" > "$PROG.tmp" && mv "$PROG.tmp" "$PROG"; echo "$(TZ=America/Toronto date +%H:%M:%S) $1"; }
fail() { say "$TAG  FAILED - $1"; exit 1; }

# ROS's own setup scripts read unset variables, so strict mode is paused for
# them - with it on they abort (ROS_DISTRO: unbound variable) the moment this
set +u
source /opt/ros/noetic/setup.bash
source "$CATKIN_WS/devel/setup.bash" 2>/dev/null || true
set -u

[ -s "$BAG" ] || fail "no recording at $BAG"
[ -e "$DB" ] && fail "$DB already exists; not overwriting it"
FREE=$(df -BG --output=avail "$D" 2>/dev/null | tail -1 | tr -dc 0-9)
[ -n "$FREE" ] || fail "cannot read the card's free space - is it mounted?"
[ "$FREE" -ge "$NEED_GB" ] || fail "only ${FREE} GB free on the card; a replay's map needs ${NEED_GB} GB reserved"

export ROS_MASTER_URI=http://localhost:11312
# A process of THIS replay master: exact name AND the 11312 master (never a substring).
replay_pid() {
    local p
    for p in $(ps -eo pid=,comm= | awk -v n="$1" '$2 == n {print $1}'); do
        if tr '\0' '\n' < "/proc/$p/environ" 2>/dev/null | grep -qx 'ROS_MASTER_URI=http://localhost:11312'; then
            echo "$p"
        fi
    done
}
[ -z "$(replay_pid rtabmap)$(replay_pid icp_odometry)" ] \
    || fail "another replay's mapping is still running on the 11312 master; wait for it to finish"

ARGS=()
GUESS=odom
RAYTRACE=false
GROUND_IS_OBSTACLE=true     # the launch file's defaults (lidar_reference.launch, grid step G1 arguments)
MAX_GROUND_H=0
FLOOR_EXPECT=""
if [ "$MODE" = reference ]; then
    [ -s "$LAUNCHF" ] || fail "no $LAUNCHF"
    [ -e "$ARGF" ] || fail "no values file $ARGF (v01.args may be empty, but must exist)"
    mapfile -t ARGS < <(sed -e 's/#.*//' -e 's/^[[:space:]]*//' -e 's/[[:space:]]*$//' "$ARGF" | grep -v '^$')
    for kv in "${ARGS[@]+"${ARGS[@]}"}"; do
        k="${kv%%:=*}"
        [ "$k" != "$kv" ] || fail "'$kv' in $ARGF is not name:=value"
        case "$k" in db_dir|db_name) fail "'$k' is set by this script, not by a values file" ;; esac
        grep -qF "<arg name=\"$k\"" "$LAUNCHF" || fail "'$k' in $ARGF is not a setting of lidar_reference.launch"
        [ "$k" = guess_frame_id ] && GUESS="${kv#*:=}"
        [ "$k" = map_grid_ray_tracing ] && RAYTRACE="${kv#*:=}"
        [ "$k" = map_grid_ground_is_obstacle ] && GROUND_IS_OBSTACLE="${kv#*:=}"
        [ "$k" = map_grid_max_ground_height ] && MAX_GROUND_H="${kv#*:=}"
    done
    [ -n "$GUESS" ] || fail "guess_frame_id is empty in $ARGF: base_link would get two parents (see lidar_reference.launch)"
    # trap 9: the floor height this variant assumes, to be confirmed against the bag before launch
    FLOOR_EXPECT=""
    if [ "$GROUND_IS_OBSTACLE" = false ] \
       && awk -v h="$MAX_GROUND_H" 'BEGIN{exit !(h+0 != 0)}' 2>/dev/null; then
        FLOOR_EXPECT=$(awk -v h="$MAX_GROUND_H" -v m="$FLOOR_MARGIN" 'BEGIN{printf "%.3f", h - m}')
    fi
    if [ -n "${FLOOR_Z_BY_HAND:-}" ]; then
        [ -n "$FLOOR_EXPECT" ] || fail "FLOOR_Z_BY_HAND is set, but variant $VARIANT does not place the floor (no ground height to check)"
        awk -v z="$FLOOR_Z_BY_HAND" 'BEGIN{exit !(z ~ /^-?[0-9]*\.?[0-9]+$/)}' \
            || fail "FLOOR_Z_BY_HAND='$FLOOR_Z_BY_HAND' is not a number of metres (e.g. -0.13)"
        awk -v z="$FLOOR_Z_BY_HAND" -v e="$FLOOR_EXPECT" -v t="$FLOOR_TOL" 'BEGIN{d=z-e; if(d<0)d=-d; exit !(d<=t+1e-9)}' \
            || fail "the floor given by hand (FLOOR_Z_BY_HAND=$FLOOR_Z_BY_HAND m) is not within ${FLOOR_TOL} m of where $ARGF puts it ($FLOOR_EXPECT m): change map_grid_max_ground_height to (floor + $FLOOR_MARGIN) and write down why"
    fi
    # a folder holding ANY result is refused; one left by an attempt that failed before
    # producing anything is moved aside (renamed), never removed
    for f in rtab_lidar_ref.db lidar_ref.tum odom_lidar_ref.bag odom_lidar_ref.bag.active; do
        [ -e "$OUT/$f" ] && fail "$OUT/$f already exists; not overwriting it"
    done
    if [ -e "$OUT" ]; then
        mv "$OUT" "$OUT.failed-$(TZ=America/Toronto date +%Y%m%d-%H%M%S)" || fail "cannot move the old $OUT aside"
    fi
fi
mkdir -p "$OUT"

if [ "$MODE" = reference ]; then
    say "$TAG  0/1 s  checks before launch: build, bag frames  0s"
    # (a) the robot's own build must have what the launch file pins (Icp/Strategy=1, Optimizer/Strategy=2)
    # draw on (a script over ssh, no DISPLAY) it ABORTS before printing anything - measured on the
    # Jetson: exit 134, "qt.qpa.xcb: could not connect to display", 0 lines - and the check below
    # would then refuse a good build. "offscreen" tells Qt to draw nowhere; measured: 52 "With" lines, exit 0.
    QT_QPA_PLATFORM=offscreen rtabmap --version > "$OUT/rtabmap_version_robot.txt" 2>&1
    QT_QPA_PLATFORM=offscreen rtabmap --params  > "$OUT/rtabmap_params_robot.txt" 2>&1     # provenance, ENGINEERING_NOTES.md section 4 rule 8
    grep -Eq "With libpointmatcher: *true" "$OUT/rtabmap_version_robot.txt" \
        || fail "this RTAB-Map build has no libpointmatcher (Icp/Strategy=1 would silently fall back); see rtabmap_version_robot.txt"
    grep -Eq "With GTSAM: *true" "$OUT/rtabmap_version_robot.txt" \
        || fail "this RTAB-Map build has no GTSAM (Optimizer/Strategy=2); see rtabmap_version_robot.txt"
    if [ "$RAYTRACE" = true ]; then     # trap 7
        grep -Eq "With OctoMap: *true" "$OUT/rtabmap_version_robot.txt" \
            || fail "variant $VARIANT turns on ray tracing, but this RTAB-Map build has no OctoMap (it would be silently skipped); see rtabmap_version_robot.txt"
    fi
    # (b) this drive's /tf must accept our one new frame without a conflict (trap 4), and, for a
    #     variant that places the floor, must put the floor where the variant assumes it (trap 9)
    FLOOR_ARGS=()
    if [ -n "$FLOOR_EXPECT" ] && [ -z "${FLOOR_Z_BY_HAND:-}" ]; then
        FLOOR_ARGS=(--expect-floor "$FLOOR_EXPECT" --floor-tol "$FLOOR_TOL")
    fi
    python3 "$T/bag_tf_check.py" "$BAG" --guess-frame "$GUESS" "${FLOOR_ARGS[@]+"${FLOOR_ARGS[@]}"}" > "$OUT/bag_tf_check.txt" 2>&1 \
        || fail "the bag's frames do not allow this replay - see $OUT/bag_tf_check.txt"
fi

# ---- provenance (ENGINEERING_NOTES.md section 4 rule 8), written before anything runs
{
  echo "run=$RUN mode=$MODE variant=$VARIANT rate=$RATE"
  echo "started $(TZ=America/Toronto date '+%Y-%m-%d %H:%M:%S %Z') on $(hostname)"
  echo "bag $BAG $(stat -c %s "$BAG") bytes"
  dpkg -l | awk '/ros-noetic-rtabmap(-odom|-slam|-util|-sync)? /{print $2, $3}'
  md5sum "$0" "$T/lidar_reference.launch" "${ARGF:-/dev/null}" "$T/lidar_ref_products.py" \
         "$T/bag_tf_check.py" "$T/scan_counter.py" "$T/stamp_lag_probe.py" 2>/dev/null
  RC="$(rospack find rslidar_sdk 2>/dev/null)/config/replay_config.yaml"
  md5sum "$RC" 2>/dev/null; grep -n "use_lidar_clock\|ts_first_point\|split_angle" "$RC" 2>/dev/null
  SN="$(rospack find self_navigation 2>/dev/null)"
  md5sum "$SN/launch/helios_body_filter.launch" "$SN/scripts/helios_restamp.py" "$SN/scripts/box_crop.py" \
         "$SN/launch/3dreplay_pipeline.launch" "$SN/launch/rtabmap_3d.launch" 2>/dev/null
  [ "$MODE" = reference ] && head -3 "$OUT/rtabmap_version_robot.txt"
  if [ "$MODE" = reference ] && [ -n "$FLOOR_EXPECT" ]; then
      if [ -n "${FLOOR_Z_BY_HAND:-}" ]; then
          echo "floor: GIVEN BY HAND FLOOR_Z_BY_HAND=$FLOOR_Z_BY_HAND m in base_link (variant assumes $FLOOR_EXPECT +- $FLOOR_TOL m); not read from the bag"
      else
          echo "floor: confirmed from the bag's /tf_static (variant assumes $FLOOR_EXPECT +- $FLOOR_TOL m; see bag_tf_check.txt)"
      fi
  fi
} > "$OUT/provenance.txt" 2>&1
[ "$MODE" = reference ] && cp "$ARGF" "$OUT/params.args"

CORE=""
if ! timeout 5 rostopic list >/dev/null 2>&1; then
    setsid nohup roscore -p 11312 > "$OUT/replay_core.log" 2>&1 < /dev/null &
    CORE=$!
    sleep 6
fi
rosparam set /use_sim_time true
[ "$(rosparam get /use_sim_time)" = "true" ] || fail "could not set sim time on 11312"

if [ "$MODE" = colleague ]; then
    setsid nohup roslaunch self_navigation 3dreplay_pipeline.launch \
        sensor:=helios db_dir:="$OUT" > "$OUT/replay.log" 2>&1 < /dev/null &
else
    # trap 8: every C++ node writes each log line at once, so the start-up check below can read it
    ROSCONSOLE_STDOUT_LINE_BUFFERED=1 setsid nohup nice -n 10 roslaunch "$LAUNCHF" db_dir:="$OUT" "${ARGS[@]+"${ARGS[@]}"}" \
        > "$OUT/replay.log" 2>&1 < /dev/null &
fi
LAUNCH=$!
# From here on a failure must not leave the replay running: close the map by SIGINT
# (no deadline - trap 2) and stop this master's nodes, then report.
abort() {
    say "$TAG  FAILED - $1 (closing what was started)"
    local p
    for p in $(replay_pid rtabmap); do
        kill -INT "$p" 2>/dev/null
        while [ -d "/proc/$p" ] && ! grep -q '^State:.*Z' "/proc/$p/status" 2>/dev/null; do sleep 2; done
    done
    rosnode kill -a > /dev/null 2>&1
    kill -INT "$LAUNCH" 2>/dev/null
    # wait (at most 60 s) for roslaunch to finish closing its nodes: the odometry recorder
    # renames odom_lidar_ref.bag.active by its old path, so the folder must not move under it
    for _ in $(seq 1 30); do
        { [ -d "/proc/$LAUNCH" ] && ! grep -q '^State:.*Z' "/proc/$LAUNCH/status" 2>/dev/null; } || break
        sleep 2
    done
    [ -n "$CORE" ] && kill -INT "$CORE" 2>/dev/null
    local where=""
    if [ "$MODE" = reference ]; then
        # trap 3: rename (never remove) this attempt's folder so a retry of the same variant starts
        # clean. Reference mode ONLY: in colleague mode $OUT is the whole run folder.
        local moved="$OUT.failed-$(TZ=America/Toronto date +%Y%m%d-%H%M%S)"
        if mv "$OUT" "$moved" 2>/dev/null; then
            where="; this attempt kept in $moved - re-running the same variant starts clean"
        else
            where="; could not rename $OUT - move it aside before re-running this variant"
        fi
    fi
    say "$TAG  FAILED - $1$where"
    exit 1
}
sleep 14

RT=$(replay_pid rtabmap)
[ -n "$RT" ] || abort "the LiDAR mapping did not start; see $OUT/replay.log"
PROBE=""; COUNTER=""
if [ "$MODE" = reference ]; then
    [ -n "$(replay_pid icp_odometry)" ] || abort "the LiDAR odometry did not start; see $OUT/replay.log"
    # trap 8: the odometry's own start-up line must show normals switched on. Matched after ": " so a
    # different node's line (e.g. "RGBDIcpOdometry: ...") can never be taken for it.
    NK=""
    for _ in $(seq 1 30); do
        NK=$(grep -aoE ': IcpOdometry: scan_normal_k +=+ *[0-9]+' "$OUT/replay.log" | tail -1 | grep -oE '[0-9]+$')
        [ -n "$NK" ] && break
        sleep 2
    done
    [ -n "$NK" ] || abort "could not confirm the odometry computes normals: no 'IcpOdometry: scan_normal_k = N' line in replay.log after 60 s"
    [ "$NK" -gt 0 ] || abort "the odometry started with scan_normal_k = $NK: no normals, so no point-to-plane matching (see lidar_reference.launch, icp_odometry comment)"
    echo "odometry normals from $NK neighbours (IcpOdometry start-up line)" >> "$OUT/provenance.txt"
    rosparam dump "$OUT/params_used.yaml" /lidar_ref 2>/dev/null   # what the nodes actually received
    # stage counts (decoded / after the body filter / odometry) and the timing probe (first ~600 scans)
    nice -n 10 python3 "$T/scan_counter.py" _out:="$OUT/scan_counts.json" > "$OUT/scan_counter.log" 2>&1 &
    COUNTER=$!
    nice -n 10 python3 "$T/stamp_lag_probe.py" _csv:="$OUT/stamp_probe.csv" > "$OUT/stamp_probe.log" 2>&1 &
    PROBE=$!
    sleep 3
fi

DUR=$(rosbag info -y -k duration "$BAG" 2>/dev/null | cut -d. -f1); DUR=${DUR:-1200}
( T0=$(date +%s)
  while :; do
      E=$(( $(date +%s) - T0 ))
      C=$(awk -v e="$E" -v r="$RATE" -v d="$DUR" 'BEGIN{c=int(e*r); if(c>d-1)c=d-1; print c}')
      X=""
      [ -d "/proc/$RT" ] || X="  !! THE MAPPING PROCESS (pid $RT) HAS DIED"
      [ -s "$OUT/scan_counts.json" ] && [ -z "$X" ] && X=$(python3 -c "import json;d=json.load(open('$OUT/scan_counts.json'));print('  scans: %d decoded, %d to odometry' % (d.get('/helios/points',0), d.get('/lidar_ref/odom',0)))" 2>/dev/null)
      say "$TAG  ${C}/${DUR} s  replaying at ${RATE}x  ${E}s${X}" > /dev/null
      sleep 15
  done ) &
TICKER=$!
if [ "$MODE" = colleague ]; then
    rosbag play "$BAG" --clock -r "$RATE" > "$OUT/play.log" 2>&1
else
    # explicit input list; /tf and /tf_static are KEPT (trap 4). The Ouster packets are not needed.
    nice -n 10 rosbag play "$BAG" --clock -r "$RATE" \
        --topics /helios/packets /tf /tf_static /odometry/filtered /imu/data > "$OUT/play.log" 2>&1
fi
sleep 10            # let the last scans through the pipeline
kill "$TICKER" 2>/dev/null
[ -n "$PROBE" ] && kill -INT "$PROBE" 2>/dev/null
[ -n "$COUNTER" ] && kill -INT "$COUNTER" 2>/dev/null

# ---- close the map: interrupt by PID, then wait with NO deadline (trap 2)
DIED=""
[ -d "/proc/$RT" ] || DIED="the mapping process had already died during the replay (see replay.log); "
kill -INT "$RT" 2>/dev/null
W0=$(date +%s); NEXT=0
while [ -d "/proc/$RT" ] && ! grep -q '^State:.*Z' "/proc/$RT/status" 2>/dev/null; do
    W=$(( $(date +%s) - W0 ))
    if [ "$W" -ge "$NEXT" ]; then
        NEXT=$(( W + 30 ))
        if [ "$W" -lt 1800 ]; then
            say "$TAG  ${DUR}/${DUR} s  closing the LiDAR map (pid $RT)  ${W}s" > /dev/null
        else
            say "$TAG  ${DUR}/${DUR} s  STILL closing the LiDAR map after $(( W / 60 )) min (pid $RT) - NOT killed, waiting  ${W}s" > /dev/null
        fi
    fi
    sleep 2
done
echo "$(TZ=America/Toronto date +%H:%M:%S) LiDAR map closed after $(( $(date +%s) - W0 )) s"
rosnode kill -a > /dev/null 2>&1        # 11312 only (exported above); also closes odom_lidar_ref.bag
kill -INT "$LAUNCH" 2>/dev/null
sleep 5
[ -n "$CORE" ] && kill -INT "$CORE" 2>/dev/null

if [ "$MODE" = colleague ]; then
    say "$TAG  map closed - making the reference products"
    {
      python3 "$T/check_closed.py" "$OUT/rtab_helios.db" --lidar
      python3 "$T/db_corrected_tum.py" "$OUT/rtab_helios.db" "$OUT/lidar.tum" --label "LiDAR (Helios), $RUN"
      python3 "$T/bag_to_tum.py" "$BAG" --topic /odometry/filtered --out "$OUT/wheel.tum"
      python3 "$T/render_map.py" "$OUT/rtab_helios.db" "$OUT" --poses "$OUT/lidar.tum" \
          --zband 0.15 2.0 --cell 0.05 --npz "$OUT/lidar_map.npz" --name lidar_map \
          --title "$RUN - the LiDAR map (walls 0.15-2.0 m up), corrected positions"
    } > "$OUT/products.log" 2>&1
    if [ -s "$OUT/lidar.tum" ] && [ -s "$OUT/wheel.tum" ] && [ -s "$OUT/lidar_map.npz" ]; then
        N=$(grep -vc '^#' "$OUT/lidar.tum")
        CL=$(python3 -c "import json;print(json.load(open('$OUT/lidar.tum.meta.json')).get('loop_closures','?'))")
        say "$TAG  complete  ${N} LiDAR nodes, ${CL} loop closures"
        exit 0
    fi
    fail "${DIED}products missing; see $OUT/products.log"
fi

# ---- reference products
say "$TAG  map closed - making the reference products"
for _ in $(seq 1 30); do [ -e "$OUT/odom_lidar_ref.bag.active" ] || break; sleep 2; done   # up to 60 s for the recorder to close its bag
WHEEL="$D/$RUN/wheel.tum"       # the wheel path does not depend on the variant: made once, reused
( T0=$(date +%s); while :; do sleep 20; say "$TAG  ${DUR}/${DUR} s  making the reference products  $(( $(date +%s) - T0 ))s" > /dev/null; done ) &
TICKER=$!
{
  python3 "$T/check_closed.py" "$DB" --lidar
  python3 "$T/db_corrected_tum.py" "$DB" "$OUT/lidar_ref.tum" --label "LiDAR reference $VARIANT, $RUN"
  [ -s "$WHEEL" ] || python3 "$T/bag_to_tum.py" "$BAG" --topic /odometry/filtered --out "$WHEEL"
  if [ -e "$OUT/odom_lidar_ref.bag.active" ]; then
      echo "WARNING odom_lidar_ref.bag was not closed; not reindexed automatically."
      echo "  to recover it (writes a new file, removes nothing): rosbag reindex '$OUT/odom_lidar_ref.bag.active'"
  fi
  python3 "$T/lidar_ref_products.py" --db "$DB" --odom-bag "$OUT/odom_lidar_ref.bag" \
      --log "$OUT/replay.log" --out "$OUT" --expected-scans $(( DUR * 10 )) --counts "$OUT/scan_counts.json"
  python3 "$T/render_map.py" "$DB" "$OUT" --poses "$OUT/lidar_ref.tum" \
      --zband 0.15 2.0 --cell 0.05 --npz "$OUT/lidar_ref_map.npz" --name lidar_ref_map \
      --title "$RUN - LiDAR reference $VARIANT (walls 0.15-2.0 m up), corrected positions"
} > "$OUT/products.log" 2>&1
kill "$TICKER" 2>/dev/null

if grep -q "CLOSED PROPERLY" "$OUT/products.log" && [ -s "$OUT/lidar_ref.tum" ] && [ -s "$OUT/lidar_ref_odom.tum" ] \
   && [ -s "$OUT/odom_health.json" ] && [ -e "$OUT/closures.tsv" ] && [ -s "$OUT/lidar_ref_map.npz" ]; then
    S=$(python3 - "$OUT" <<'PY'
import json, sys
o = sys.argv[1]
m = json.load(open(o + "/lidar_ref.tum.meta.json")); h = json.load(open(o + "/odom_health.json"))
print("%d nodes, %d loop closures, end gap %.2f m, %d/%d scans to odometry, %.1f%% lost"
      % (m.get("poses", 0), h.get("closures", 0), m.get("end_gap_corrected_m", float("nan")),
         h.get("scans_processed", 0), h.get("expected_scans", 0), 100 * (h.get("lost_fraction") or 0)))
PY
)
    say "$TAG  complete  $S"
else
    fail "${DIED}products missing or the map did not close properly; see $OUT/products.log"
fi
