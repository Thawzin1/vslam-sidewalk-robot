#!/usr/bin/env bash
#
# sim_run.sh - execute ONE simulated mapping run end to end (was tools/p1_run.sh, the Phase-1 campaign runner).
#   Called by run/simulate.sh, which picks the world and route; it can also be called directly:
#
#   run/lib/sim_run.sh <RUN_ID> <ARM>
#     ARM = A | AY | B0 | B1     (which simulated sensor feeds the map; B1 = stereo camera with image noise)
#
# Inputs: WORLD_LAUNCH, ROUTE, SPAWN (environment, see below). Outputs: <records>/<RUN_ID>/ (logs, checklist.txt,
# frame_yield.json, timelapse) and the map database under <work>/sim_maps/<date>/.
# It stops ONLY the processes it started itself (by process number); it never kills other ROS or Gazebo programs
# by name, and it refuses to start while a Gazebo server or roslaunch is already running.
# *Plain terms: one scripted drive in the simulator, recorded like a real one.*
#
# One run = clean sweep → sim → record stack → frame-yield sampler →
# map timelapse → scripted route → teardown → timelapse video → checklist.
# Everything is written under $RECORDS_DIR/<RUN_ID>/ plus the standard
# trajectory/db locations; the checklist lands in checklist.txt there.
#
# The arm definitions are THE experiment: change them only as a new,
# labelled arm (one variable per rung).
#
# WHICH WORLD IS DRIVEN
#   The arm chooses the SENSOR configuration. Three environment variables
#   choose the PLACE, so the same proven sequence can drive the outdoor
#   world or either indoor office layout without forking this script:
#
#     WORLD_LAUNCH  launch file inside the sidewalk_sim package
#     ROUTE         route file for route_player.py. A bare name (no slash)
#                   is resolved inside sidewalk_sim's routes/ directory; a
#                   path with a slash in it is used exactly as written.
#     SPAWN         extra launch arguments giving the starting pose, passed
#                   through verbatim. Set it to the empty string to inherit
#                   whatever spawn pose the world's own launch file
#                   declares — which is the right answer for the indoor
#                   worlds, whose launch files already place the robot
#                   somewhere sensible for that floor plan.
#
#   All three default to the outdoor values, so every existing invocation
#   keeps working exactly as before when none of them is set.
#
# USAGE
#   Outdoor world (unchanged — this is the form every validated Phase-1 run
#   used, and it still needs no environment variables at all):
#
#     ./p1_run.sh p1_B1_r6 B1
#
#   Second office layout, the fifteen-room building with the closed-loop
#   corridor. Its launch file already spawns the robot in the entrance
#   foyer facing into the building, so hand the spawn pose back to it:
#
#     WORLD_LAUNCH=husky_office2_gazebo.launch \
#     ROUTE=office2_full.yaml \
#     SPAWN="" \
#     ./p1_run.sh p1_office2_r1 B1
#
#   First office layout, the four-room corridor floor. There is NO
#   per-world wrapper launch file for it: husky_real_gazebo.launch already
#   defaults its `world` argument to office.world, so that launch file IS
#   the first office layout, and its own spawn defaults (origin, facing
#   +X) are the ones to inherit:
#
#     WORLD_LAUNCH=husky_real_gazebo.launch \
#     ROUTE=office1_full.yaml \
#     SPAWN="" \
#     ./p1_run.sh p1_office1_r1 B1
#
# No `set -u`: ROS's own setup scripts reference unset variables
# (1.ros_distro.sh dies on ROS_DISTRO) and would abort this script at the
# source lines. The ${n:?} guards below cover the arguments instead.

# repository folders and addresses (REPO_ROOT, RECORDS_DIR, WORK_DIR, JOBS_DIR, TOOLS_DIR, *_ADDR): run/lib/paths.sh
. "$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)/paths.sh"
RUN_ID="${1:?usage: p1_run.sh RUN_ID ARM}"
ARM="${2:?usage: p1_run.sh RUN_ID ARM}"

# ------------------------------------------------------------ world / route
# The outdoor world stays the default for all three, so an invocation that
# sets none of them behaves identically to every run recorded so far.
WORLD_LAUNCH="${WORLD_LAUNCH:-husky_mcity_gazebo.launch}"
# `${SPAWN-...}` uses the hyphen WITHOUT a colon on purpose: that form
# substitutes the default only when SPAWN is completely unset, so a caller
# who explicitly writes SPAWN="" gets an empty string through to the launch
# line and the world's own launch file supplies the starting pose. The
# colon form would have overwritten that deliberate empty value with the
# outdoor pose — which, in a 24 by 18 metre office, is outside the building.
SPAWN="${SPAWN-spawn_x:=0.0 spawn_y:=-60.5 spawn_z:=0.3 spawn_yaw:=3.1416}"

# ---------------------------------------------------------------- arm matrix
case "$ARM" in
  A)   SIM_FLAGS="";                                          SLAM_FLAGS="";                              YIELD_FLAGS="--rgbd" ;;
  AY)  SIM_FLAGS="";                                          SLAM_FLAGS="";                              YIELD_FLAGS="--rgbd" ;;
       # AY's single lever is applied to rtabmap_zedx.yaml BEFORE calling this
       # script (its own labelled config state) — the runner itself is identical.
  B0)  SIM_FLAGS="zedx_stereo:=true zedx_noise_stddev:=0.0";  SLAM_FLAGS="stereo:=true approx_sync:=false"; YIELD_FLAGS="" ;;
  B1)  SIM_FLAGS="zedx_stereo:=true";                         SLAM_FLAGS="stereo:=true approx_sync:=false"; YIELD_FLAGS="" ;;
  *) echo "unknown arm: $ARM" >&2; exit 2 ;;
esac

# Gazebo's server renders the camera sensors even with gui:=false, so it
# needs a real display and its authority file or the camera silently creates
# nothing at all. FLAGGED, deliberately left as-is: both values below are
# hardcoded, and docs/OPERATIONS.md ("Remote GUI access to the Jetson")
# states that the display number must be RE-DERIVED after every fresh login
# and must not be assumed to be :1 — the login manager starts a new server
# per session rather than reusing the greeter's. These two lines have held
# across every run so far and are not being changed on speculation, but if a
# run aborts at the CAMERA SILENT check below, re-derive both values by the
# procedure in that document (read the Xorg process's own -auth argument;
# cross-check the newest socket in /tmp/.X11-unix) and correct them here.
export DISPLAY="${SIM_DISPLAY:-${DISPLAY:-:1}}"
export XAUTHORITY="${SIM_XAUTHORITY:-${XAUTHORITY:-/run/user/1000/gdm/Xauthority}}"
source /opt/ros/noetic/setup.bash
source "$CATKIN_WS/devel/setup.bash"
[ -r "$SIDEWALK_ENV" ] && source "$SIDEWALK_ENV"

# Resolved only now, because a bare route name is looked up with rospack and
# rospack needs the package path the source lines above just set. A name
# containing a slash is taken as a literal path and left alone, so an
# absolute path or one relative to the working directory both still work.
ROUTE="${ROUTE:-mcity_loop.yaml}"
case "$ROUTE" in
  */*) ;;
  *)   ROUTE="$(rospack find sidewalk_sim)/routes/$ROUTE" ;;
esac
[ -f "$ROUTE" ] || { echo "[p1_run $RUN_ID] NO SUCH ROUTE FILE: $ROUTE" >&2; exit 7; }

REC="$RECORDS_DIR/$RUN_ID"
mkdir -p "$REC"
DB_DIR="$WORK_DIR/sim_maps/$(date +%F)"
mkdir -p "$DB_DIR"                       # SQLite will NOT create parents (SOLVED)
LOG="$REC/rtabmap.log"

say() { echo "[p1_run $RUN_ID] $*"; }

# nothing_running: refuse to start if a Gazebo server or a roslaunch is already up. This script does not kill
# other people's processes - the old runner did, by name, and could stop a real drive by mistake.
nothing_running() {
  local left
  left=$(ps -eo comm= | awk '$1=="roslaunch" || $1=="gzserver" || $1=="gzclient" || $1=="rtabmap"' | wc -l)
  [ "$left" = "0" ] || { say "REFUSED: $left roslaunch/gzserver/gzclient/rtabmap process(es) already running - stop them first (by pid)"; exit 3; }
}
STARTED_PIDS=()
# stop_started: SIGINT (clean shutdown) every process this script started, wait, then SIGKILL what is left - by pid only.
stop_started() {
  local p
  for p in "${STARTED_PIDS[@]}"; do kill -2 "$p" 2>/dev/null; done
  sleep 8
  for p in "${STARTED_PIDS[@]}"; do kill -0 "$p" 2>/dev/null && kill -9 "$p" 2>/dev/null; done
  sleep 2
}
progress() { echo "SIM $RUN_ID $1  $(( $(date +%s) - T0 ))s" > "$JOBS_DIR/sim_${RUN_ID}.progress"; }
T0=$(date +%s)

say "=== pre-flight ==="
nothing_running
progress "1/7 pre-flight"

say "=== sim up (arm $ARM, world $WORLD_LAUNCH) ==="
# $SPAWN is intentionally unquoted: it is a list of launch arguments, and
# when it is empty it must vanish entirely so the launch file's own spawn
# defaults apply. Quoting it would pass one empty argument to roslaunch.
nohup roslaunch sidewalk_sim "$WORLD_LAUNCH" gui:=false rviz:=false \
  zedx:=true $SIM_FLAGS $SPAWN > "$REC/gazebo.log" 2>&1 < /dev/null &
SIM_PID=$!
STARTED_PIDS+=("$SIM_PID")
progress "2/7 simulator starting"
sleep 45
CAM_TOPIC="/zedx_front/zed_node/left/camera_info"
[ "$ARM" = "A" ] || [ "$ARM" = "AY" ] && CAM_TOPIC="/zedx_front/zed_node/rgb/camera_info"
if ! timeout 15 rostopic hz -w 10 "$CAM_TOPIC" 2>&1 | grep -q "average rate"; then
  say "CAMERA SILENT — aborting"; progress "FAILED camera silent"; stop_started; exit 4
fi

say "=== record stack ==="
nohup roslaunch sidewalk_slam record_mapping_run.launch run_id:="$RUN_ID" \
  database_path:="$DB_DIR/$RUN_ID.db" rtabmap_log:="$LOG" \
  monitor_out:="$REC/monitor.csv" $SLAM_FLAGS rviz:=false \
  > "$LOG" 2>&1 < /dev/null &
REC_PID=$!
STARTED_PIDS+=("$REC_PID")
progress "3/7 mapping started"
sleep 25
if grep -qi fatal "$LOG"; then say "FATAL in rtabmap log — aborting"; progress "FAILED rtabmap fatal"; stop_started; exit 5; fi
for node in /rtabmap/rtabmap /ground_truth_odom /run_monitor; do
  rosnode list 2>/dev/null | grep -q "$node" || { say "MISSING NODE $node — aborting"; progress "FAILED missing $node"; stop_started; exit 6; }
done

say "=== samplers ==="
nohup rosrun sidewalk_evaluation frame_yield.py --mode run $YIELD_FLAGS \
  --out "$REC/frame_yield.json" > "$REC/yield.log" 2>&1 < /dev/null &
YIELD_PID=$!
STARTED_PIDS+=("$YIELD_PID")
nohup rosrun sidewalk_slam map_manager.py timelapse --name "$RUN_ID" \
  > "$REC/timelapse.log" 2>&1 < /dev/null &
TL_PID=$!
STARTED_PIDS+=("$TL_PID")
nohup "$REPO_ROOT/tools/sim/make_run_video.sh" --capture "$RUN_ID" \
  > "$REC/overhead.log" 2>&1 < /dev/null &
OVERHEAD_PID=$!
STARTED_PIDS+=("$OVERHEAD_PID")
progress "4/7 samplers up"

say "=== route ($ROUTE) ==="
progress "5/7 driving the route"
rosrun sidewalk_sim route_player.py \
  --route "$ROUTE" \
  > "$REC/route.log" 2>&1
ROUTE_RC=$?
say "route exit $ROUTE_RC"
sleep 10   # let the last frames settle into the map

say "=== teardown ==="
kill -TERM "$YIELD_PID" 2>/dev/null; sleep 3      # SIGTERM → writes the JSON
kill -9 "$TL_PID" 2>/dev/null
kill -2 "$OVERHEAD_PID" 2>/dev/null; sleep 3      # SIGINT → AVI gets indexed;
                                                    # -9 here leaves it unplayable
kill -2 "$REC_PID" 2>/dev/null; sleep 8            # SIGINT → db + sidecars flush
kill -2 "$SIM_PID" 2>/dev/null; sleep 6
stop_started
progress "6/7 teardown done"

say "=== timelapse video ==="
rosrun sidewalk_slam map_manager.py timelapse-video --name "$RUN_ID" \
  >> "$REC/timelapse.log" 2>&1 || say "timelapse-video failed (non-fatal)"

say "=== checklist ==="
progress "7/7 checklist"
{
  echo "run_id: $RUN_ID   arm: $ARM   date: $(date -Is)"
  # Which place was driven, recorded alongside the numbers it produced. Once
  # this harness can drive more than one world, a checklist that names only
  # the arm no longer identifies the run (ENGINEERING_NOTES.md section 4: every run writes its
  # provenance, every number traces back to a run).
  echo "world_launch: $WORLD_LAUNCH"
  echo "route: $ROUTE"
  echo "spawn: ${SPAWN:-(launch file defaults)}"
  echo "route_exit: $ROUTE_RC"
  echo "resets: $(grep -c 'Odometry automatically reset' "$LOG")"
  echo "quality_lines: $(grep -c 'Odom: quality' "$LOG")"
  echo "sigma_pinned_1e-4: $(grep -c 'std dev=0.000100m' "$LOG")"
  echo "lc_reject_events: $(grep -c 'Rejecting all added loop closures' "$LOG")"
  echo "fatal: $(grep -ci fatal "$LOG")"
  echo "backend_exceptions: $(grep -ciE 'IndeterminantLinearSystem|very huge|diverging' "$LOG")"
  echo "cannot_be_used_distinct: $(grep -h 'cannot be used' "$LOG" | sort -u | wc -l)"
  echo "monitor_tail: $(tail -1 "$REC/monitor_STATUS.txt" 2>/dev/null)"
  echo "yield_json: $(cat "$REC/frame_yield.json" 2>/dev/null | tr -d '\n' | head -c 400)"
  META="$REPO_ROOT/logs/sidewalk_slam/trajectories/$RUN_ID/rtabmap_trajectory.tum.meta.json"
  echo "est_mean_rate_hz: $(python3 -c "import json;print(json.load(open('$META'))['mean_rate_hz'])" 2>/dev/null)"
  echo "db_size: $(ls -la "$DB_DIR/$RUN_ID.db" 2>/dev/null | awk '{print $5}')"
} | tee "$REC/checklist.txt"

progress "DONE 7/7"
say "=== DONE ==="
