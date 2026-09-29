#!/bin/bash
# replay.sh - re-run a recorded mapping drive from its recordings, on the Jetson, with NO robot.
#
# WHAT IT DOES
#   Takes the two recordings a drive leaves behind - fusion.bag (the robot's wheels and gyroscope)
#   and <run>.svo2 (the ZED camera's own recording) - and runs them through the same programs the
#   live drive used (the blend of wheels + gyroscope + camera, then RTAB-Map), producing a NEW map
#   database and a summary of its numbers. Nothing here touches the original drive's files.
#
#   Data flow (details and the reasons in docs/REPLAY.md):
#     fusion.bag --rosbag play (paused, /clock)--> wheels, gyroscope --+
#     <run>.svo2 --ZED wrapper (file playback, real-time pacing)-------+--> blend (EKF) --> RTAB-Map --> <out>/<run>_replay.db
#
# HOW TO RUN
#   replay.sh <run> [--depth-mode MODE] [--rate 1.0] [--max-s S] [--out DIR] [--svo PATH] [--bag PATH]
#                   [--self-calib true|false]
#     <run>          the drive's name, e.g. s2_fusion_T5b
#     --depth-mode   the camera's depth mode (default NEURAL, what the drives used)
#     --rate         playback speed; only 1.0 is supported (docs/REPLAY.md, "Why the rate is fixed")
#     --max-s        stop after this many seconds of playback (default: the end of the camera recording)
#     --out          output folder (default $WORK_DIR/replay/<run>)
#     --svo, --bag   the recordings, if not at $RECORDS_DIR/<run>/<run>.svo2 and .../fusion.bag
#     --self-calib   false (default): freeze the lens geometry so two replays agree; true: as live
#     --sync-margin-s  the bag is placed this far AHEAD of the camera recording's first frame (default 3),
#                    so that after the first picture the starter can measure the offset and hold the bag
#                    for exactly that long (replay_starter.py's header). The first margin + 2 s of the
#                    recording play before the fine sync, so start recordings with the robot parked.
#
# INPUTS   $RECORDS_DIR/<run>/fusion.bag and $RECORDS_DIR/<run>/<run>.svo2 (or --bag/--svo)
# OUTPUTS  in the output folder:
#   <run>_replay.db          the new map          replay_summary.json   the numbers (rule 20: both gaps)
#   camera.tum               tracking alone       camera_corrected.tum  the map's corrected positions
#   replay_outputs.bag       /fused/odometry, /rtabmap/odom, /tf, camera + robot IMU, /clock (for scoring)
#   replay_launch.log        every program's screen output   clock_drift.csv   the sync evidence
#   svo_index.csv(.json)     every frame's capture time      unpaused.json     when the bag was resumed
#   A progress line is kept at $JOBS_DIR/<run>_replay.progress for the jobs page.
#
# ENVIRONMENT (all optional)
#   REPO_ROOT     the repository (default: $SIDEWALK_REPO, else the repository this script is in) - its catkin workspace
#   RECORDS_DIR   where run records live (default ~/.run_records)
#   WORK_DIR      where maps and the tools live (default ~/slam_series2; tools in $WORK_DIR/tools)
#   JOBS_DIR      progress lines for the jobs page (default ~/jobs)
#   REPLAY_ROS_PORT  the private ROS master's port (default 11399) - never the live master's 11311
#
# SAFETY
#   - Runs on its own ROS master, so a live drive or another replay on this machine is untouched.
#   - Refuses to overwrite an existing output database.
#   - Stops programs by the process numbers it started, never by searching names (ENGINEERING_NOTES.md rule 8).
#   - Closes RTAB-Map first and waits for the database to finish writing, then the rest.
set -euo pipefail

# ------------------------------------------------------------------ settings and arguments
HERE="$(cd "$(dirname "$0")" && pwd)"
DEV_ROOT="$(cd "$HERE/.." && pwd)"
REPO_ROOT="${REPO_ROOT:-${SIDEWALK_REPO:-$DEV_ROOT}}"   # default: the repository this script is in
RECORDS_DIR="${RECORDS_DIR:-$HOME/.run_records}"
WORK_DIR="${WORK_DIR:-$HOME/slam_series2}"
JOBS_DIR="${JOBS_DIR:-$HOME/jobs}"
TOOLS_DIR="${TOOLS_DIR:-$WORK_DIR/tools}"
PORT="${REPLAY_ROS_PORT:-11399}"
# the launch file and helper nodes: this folder's copies, or the package's once they are merged in
SCRIPTS="${REPLAY_SCRIPTS_DIR:-$DEV_ROOT/catkin_ws/src/sidewalk_slam/scripts}"
LAUNCH="${REPLAY_LAUNCH_FILE:-$DEV_ROOT/catkin_ws/src/sidewalk_slam/launch/replay_drive.launch}"
CAM_LAUNCH="${REPLAY_CAMERA_LAUNCH_FILE:-$DEV_ROOT/catkin_ws/src/sidewalk_slam/launch/replay_camera.launch}"

usage() { sed -n '2,40p' "$0" | sed 's/^# \{0,1\}//'; }
[ $# -ge 1 ] || { usage; exit 2; }
case "$1" in -h|--help) usage; exit 0;; esac
RUN="$1"; shift
DEPTH_MODE=NEURAL; RATE=1.0; MAX_S=0; OUT=""; SVO=""; BAG=""; SELF_CALIB=false; SYNC_MARGIN=3.0
while [ $# -gt 0 ]; do
    case "$1" in
        --depth-mode) DEPTH_MODE="$2"; shift 2;;
        --rate)       RATE="$2"; shift 2;;
        --max-s)      MAX_S="$2"; shift 2;;
        --out)        OUT="$2"; shift 2;;
        --svo)        SVO="$2"; shift 2;;
        --bag)        BAG="$2"; shift 2;;
        --self-calib) SELF_CALIB="$2"; shift 2;;
        --sync-margin-s) SYNC_MARGIN="$2"; shift 2;;
        *) echo "unknown argument: $1" >&2; usage; exit 2;;
    esac
done
[ "$RATE" = "1.0" ] || [ "$RATE" = "1" ] || {
    echo "only --rate 1.0 is supported: the ZED SDK paces the camera recording by the wall clock" >&2
    echo "(svo_real_time_mode), so a slower bag would fall out of step. See docs/REPLAY.md." >&2; exit 2; }
SVO="${SVO:-$RECORDS_DIR/$RUN/$RUN.svo2}"
BAG="${BAG:-$RECORDS_DIR/$RUN/fusion.bag}"
OUT="${OUT:-$WORK_DIR/replay/$RUN}"
DB="$OUT/${RUN}_replay.db"

STARTED_PIDS=()      # every background process this script starts, stopped again on any error
say() { echo "$(date '+%F %T') $*"; }
die() {
    say "ERROR: $*" >&2
    for p in "${STARTED_PIDS[@]}"; do kill -INT "$p" 2>/dev/null || true; done
    [ "${#STARTED_PIDS[@]}" -gt 0 ] && sleep 5
    for p in "${STARTED_PIDS[@]}"; do kill -TERM "$p" 2>/dev/null || true; done
    exit 1
}
progress() { mkdir -p "$JOBS_DIR"; echo "REPLAY $RUN $*" > "$JOBS_DIR/${RUN}_replay.progress.tmp"
             mv "$JOBS_DIR/${RUN}_replay.progress.tmp" "$JOBS_DIR/${RUN}_replay.progress"; }

# ------------------------------------------------------------------ 1. the inputs
missing=0
for f in "$BAG" "$SVO"; do
    [ -r "$f" ] || { say "missing: $f"; missing=1; }
done
if [ "$missing" = 1 ]; then
    cat <<MSG
The recordings are not on this machine. They are in the Autonomous Service Robot
Teams folder (ask Prof. Moein Mehrtash for access), under series2_raw/$RUN/$RUN/.
Download that folder on any computer, copy it to the Jetson over the lab network, e.g.

    rsync -s -av --partial $RUN/ <user>@<jetson>:~/replay_inputs/slam/$RUN/

and run this command again with --svo <that folder>/$RUN.svo2 --bag <that folder>/fusion.bag
(or copy the folder to $RECORDS_DIR/$RUN/, where they are looked for by default).
See data/README.md.
MSG
    exit 1
fi
[ -e "$DB" ] && die "$DB exists - this never overwrites a map; choose another --out"
mkdir -p "$OUT"

# ------------------------------------------------------------------ 2. the ROS environment, private master
set +u
if [ -r "$HOME/.sidewalk_env.sh" ]; then source "$HOME/.sidewalk_env.sh"
else
    source /opt/ros/noetic/setup.bash
    [ -r "$REPO_ROOT/catkin_ws/devel/setup.bash" ] && source "$REPO_ROOT/catkin_ws/devel/setup.bash"
    [ -r "$HOME/catkin_ws/devel/setup.bash" ] && source "$HOME/catkin_ws/devel/setup.bash"
fi
set -u
export ROS_MASTER_URI="http://localhost:$PORT"
# the ZED wrapper wants a display session (the SDK's graphics libraries); use the machine's if present
export DISPLAY="${DISPLAY:-:0}"
[ -z "${XAUTHORITY:-}" ] && [ -r "/run/user/$(id -u)/gdm/Xauthority" ] && export XAUTHORITY="/run/user/$(id -u)/gdm/Xauthority"
[ -r "$TOOLS_DIR/db_to_tum.py" ] || die "tools not found in $TOOLS_DIR (set TOOLS_DIR or WORK_DIR)"
if rosparam list > /dev/null 2>&1; then die "a ROS master already answers on port $PORT - is another replay running? (REPLAY_ROS_PORT picks another port)"; fi

# ------------------------------------------------------------------ 3. when does the camera recording start?
say "replay of $RUN"
say "  bag  $BAG"
say "  svo  $SVO"
say "  out  $OUT"
INDEX="$OUT/svo_index.csv"
if [ ! -s "$INDEX.json" ]; then
    say "reading every frame's capture time from the camera recording (once; about 65 frames/s)"
    progress "indexing the camera recording"
    nice -n 19 python3 "$SCRIPTS/replay_svo_index.py" "$SVO" "$INDEX" > "$OUT/svo_index.log" 2>&1 \
        || die "could not read the SVO - see $OUT/svo_index.log"
fi
SVO_T0=$(python3 -c "import json,sys; print(json.load(open(sys.argv[1]))['first_ns']/1e9)" "$INDEX.json")
SVO_DUR=$(python3 -c "import json,sys; print(json.load(open(sys.argv[1]))['duration_s'])" "$INDEX.json")
SVO_FRAMES=$(python3 -c "import json,sys; print(json.load(open(sys.argv[1]))['frames'])" "$INDEX.json")
BAG_T0=$(rosbag info --yaml -k start "$BAG")
BAG_T1=$(rosbag info --yaml -k end "$BAG")
OFFSET=$(python3 -c "print(max(0.0, round($SVO_T0 - $BAG_T0 + $SYNC_MARGIN, 3)))")
BAG_POSITION=$(python3 -c "print($BAG_T0 + $OFFSET)")
say "  camera recording: $SVO_FRAMES frames, ${SVO_DUR} s, starts $(python3 -c "print(round($SVO_T0 - $BAG_T0, 3))") s into the bag (bag placed at ${OFFSET} s, sync margin ${SYNC_MARGIN} s)"
python3 -c "import sys; sys.exit(0 if $SVO_T0 >= $BAG_T0 - 0.5 and $SVO_T0 <= $BAG_T1 else 1)" \
    || die "the camera recording starts outside the bag's time span (svo $SVO_T0, bag $BAG_T0..$BAG_T1) - are these from the same drive?"

# ------------------------------------------------------------------ 4. start everything
cleanup_on_error() {
    say "stopping after an error"
    for p in "${STARTED_PIDS[@]}"; do kill -INT "$p" 2>/dev/null || true; done
    sleep 5
    for p in "${STARTED_PIDS[@]}"; do kill -TERM "$p" 2>/dev/null || true; done
}
trap cleanup_on_error ERR

say "private ROS master on port $PORT"
roscore -p "$PORT" > "$OUT/roscore.log" 2>&1 < /dev/null &
ROSCORE_PID=$!; STARTED_PIDS+=("$ROSCORE_PID")
for i in $(seq 1 30); do rosparam list > /dev/null 2>&1 && break; sleep 1; done
rosparam list > /dev/null 2>&1 || die "roscore did not come up - see $OUT/roscore.log"
# before any node starts, so no node ever sees wall time
rosparam set /use_sim_time true

T_WALL0=$(date +%s)
say "starting the bag player (paused), the blend and the mapping ($LAUNCH)"
progress "0/${SVO_DUR%.*} s  0s  starting"
nice -n 19 roslaunch "$LAUNCH" run:="$RUN" bag:="$BAG" bag_start_offset:="$OFFSET" \
    database_path:="$DB" out_dir:="$OUT" rate:="$RATE" \
    > "$OUT/replay_launch.log" 2>&1 < /dev/null &
LAUNCH_PID=$!; STARTED_PIDS+=("$LAUNCH_PID")

# child_pid NAME: the process number of the launch's direct child whose program name is NAME
# (whole name from /proc, never a substring search); empty if none
child_pid() { ps -o pid=,comm= --ppid "$LAUNCH_PID" 2>/dev/null | awk -v n="$1" '$2 == n {print $1; exit}'; }
cam_pid()   { ps -o pid=,comm= --ppid "$CAM_LAUNCH_PID" 2>/dev/null | awk '$2 == "zed_wrapper_nod" {print $1; exit}'; }

for i in $(seq 1 60); do rosservice list 2>/dev/null | grep -qx /replay_bag/pause_playback && break; sleep 1; done
rosservice list 2>/dev/null | grep -qx /replay_bag/pause_playback || die "the bag player did not start - see $OUT/replay_launch.log"
# the mapping must be subscribed before the first picture, or the first pictures are simply lost
for i in $(seq 1 60); do rosnode list 2>/dev/null | grep -qx /rtabmap/rtabmap && break; sleep 1; done
rosnode list 2>/dev/null | grep -qx /rtabmap/rtabmap || die "RTAB-Map did not start - see $OUT/replay_launch.log"
sleep 3
say "starting the camera from its recording ($CAM_LAUNCH); its own launch, see the file's header"
nice -n 19 roslaunch "$CAM_LAUNCH" svo:="$SVO" depth_mode:="$DEPTH_MODE" self_calib:="$SELF_CALIB" \
    > "$OUT/camera.log" 2>&1 < /dev/null &
CAM_LAUNCH_PID=$!; STARTED_PIDS+=("$CAM_LAUNCH_PID")

nice -n 19 python3 "$SCRIPTS/replay_watch.py" --run "$RUN" --out-dir "$OUT" --jobs-dir "$JOBS_DIR" \
    --svo-start "$SVO_T0" --total-s "$SVO_DUR" > "$OUT/replay_watch.log" 2>&1 < /dev/null &
WATCH_PID=$!; STARTED_PIDS+=("$WATCH_PID")
rm -f "$OUT/unpaused.json"
python3 "$SCRIPTS/replay_starter.py" --marker "$OUT/unpaused.json" --bag-position "$BAG_POSITION" \
    > "$OUT/replay_starter.log" 2>&1 < /dev/null &
STARTER_PID=$!; STARTED_PIDS+=("$STARTER_PID")

say "waiting for the camera to open its recording (the depth model takes 10-30 s to load), then the fine sync (about 5 s)"
for i in $(seq 1 300); do
    [ -s "$OUT/unpaused.json" ] && break
    [ -d "/proc/$CAM_LAUNCH_PID" ] || die "the camera launch ended - see $OUT/camera.log"
    sleep 1
done
[ -s "$OUT/unpaused.json" ] || die "no camera picture within 300 s - see $OUT/replay_launch.log and $OUT/replay_starter.log"
T_PLAY0=$(date +%s)
say "bag resumed and brought into step: $(python3 -c "import json,sys; d=json.load(open(sys.argv[1])); print('offset before %s s, held %s s, after %s s' % (d['offset_before_s'], d['pause_applied_s'], d['offset_after_s']))" "$OUT/unpaused.json")"

# ------------------------------------------------------------------ 5. watch until one recording ends
# The camera node does not stop at the end of its file on this build (see replay_watch.py's header),
# so the watcher writes svo_end.json when the recording has been played out.
RT_PID=""
for i in $(seq 1 30); do RT_PID="$(child_pid rtabmap)"; [ -n "$RT_PID" ] && break; sleep 1; done
[ -n "$RT_PID" ] || die "RTAB-Map is not running - see $OUT/replay_launch.log"
say "RTAB-Map pid $RT_PID, camera pid $(cam_pid), bag player pid $(child_pid play)"
cat > "$OUT/pids.txt" <<PIDS
ROS_MASTER_URI=$ROS_MASTER_URI
ROSCORE_PID=$ROSCORE_PID
LAUNCH_PID=$LAUNCH_PID
CAMERA_LAUNCH_PID=$CAM_LAUNCH_PID
RTABMAP_PID=$RT_PID
WATCH_PID=$WATCH_PID
PIDS
WHY=""
while :; do
    sleep 5
    [ -s "$OUT/svo_end.json" ]            && { WHY="the camera recording ended ($(python3 -c "import json,sys; print(json.load(open(sys.argv[1]))['why'])" "$OUT/svo_end.json"))"; break; }
    [ -z "$(cam_pid)" ]                   && { WHY="the camera node exited"; break; }
    [ -z "$(child_pid play)" ]            && { WHY="the bag ended"; break; }
    [ -d "/proc/$RT_PID" ]                || { WHY="RTAB-Map exited on its own"; break; }
    if [ "$MAX_S" -gt 0 ] && [ $(( $(date +%s) - T_PLAY0 )) -ge "$MAX_S" ]; then WHY="--max-s $MAX_S reached"; break; fi
done
T_STOP=$(date +%s)
say "stopping: $WHY (after $((T_STOP - T_PLAY0)) s of playback)"
trap - ERR

# ------------------------------------------------------------------ 6. shut down in the right order
# First stop the inputs so nothing new arrives, give the mapping a moment to finish what it holds,
# then interrupt RTAB-Map ITSELF and wait - with no deadline - for its database to close. Only then
# interrupt the launch (roslaunch's own 15 s limit would kill a map still writing; auto_stop_mapping.py
# stops the live drives the same way).
progress "closing the map"
p="$(child_pid play)"; [ -n "$p" ] && kill -INT "$p" 2>/dev/null || true
kill -INT "$CAM_LAUNCH_PID" 2>/dev/null || true
sleep 5
if [ -d "/proc/$RT_PID" ]; then
    kill -INT "$RT_PID" 2>/dev/null || true
    n=0
    while [ -d "/proc/$RT_PID" ]; do sleep 1; n=$((n+1)); [ $((n % 30)) = 0 ] && say "  database still closing (${n}s)"; done
    say "RTAB-Map closed its database after ${n}s"
fi
kill -INT "$LAUNCH_PID" 2>/dev/null || true
for i in $(seq 1 90); do [ -d "/proc/$LAUNCH_PID" ] || [ -d "/proc/$CAM_LAUNCH_PID" ] || break; sleep 1; done
for p in "$LAUNCH_PID" "$CAM_LAUNCH_PID"; do
    [ -d "/proc/$p" ] && { say "launch $p still up after 90 s - terminating it"; kill -TERM "$p" 2>/dev/null || true; }
done
kill -INT "$WATCH_PID" 2>/dev/null || true
kill -INT "$STARTER_PID" 2>/dev/null || true
sleep 2
kill -INT "$ROSCORE_PID" 2>/dev/null || true
for i in $(seq 1 20); do [ -d "/proc/$ROSCORE_PID" ] || break; sleep 1; done
WALL_S=$(( $(date +%s) - T_WALL0 ))
say "all stopped (${WALL_S}s wall time)"

# ------------------------------------------------------------------ 7. the numbers
[ -s "$DB" ] || die "no database was written at $DB - see $OUT/replay_launch.log"
# run_monitor.py (started by record_mapping_run.launch) writes $OUT/monitor.csv: odometry resets, closures
python3 - "$OUT/extra.json" "$WHY" "$OFFSET" "$DEPTH_MODE" "$SELF_CALIB" "$SVO" "$BAG" "$((T_STOP - T_PLAY0))" "$SYNC_MARGIN" <<'PY'
import json, sys
a = sys.argv
json.dump({"stopped_because": a[2], "bag_start_offset_s": float(a[3]), "depth_mode": a[4], "self_calib": a[5],
           "svo": a[6], "bag": a[7], "played_wall_s": int(a[8]), "sync_margin_s": float(a[9])}, open(a[1], "w"), indent=1)
PY
nice -n 19 python3 "$SCRIPTS/replay_summarize.py" --run "$RUN" --db "$DB" --out-dir "$OUT" --tools-dir "$TOOLS_DIR" \
    --svo-index "$INDEX" --wall-s "$WALL_S" --monitor-csv "$OUT/monitor.csv" --extra "$OUT/extra.json" | tee "$OUT/replay_summary.txt"
progress "DONE $(python3 -c "import json,sys; d=json.load(open(sys.argv[1])); print('%s nodes, %s closures, gaps %s / %s m' % (d['nodes'], d['loop_closures'], d['tracking_gap_m'], d['corrected_gap_m']))" "$OUT/replay_summary.json")"
say "done - $OUT/replay_summary.json"
