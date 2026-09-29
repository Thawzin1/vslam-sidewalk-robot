#!/bin/bash
# go_nav.sh - open the gate and send a route's goals (real run). Jetson only. Run ONLY when the user is beside
# the robot, holding the autonomy button (R2), and has said "go".
#
#   usage:  go_nav.sh <run> A|B        (A = long corridor out and back; B = the loop, only after A passed)
#
# Removes the pause file (the Jetson-side gate), then runs nav_goals.py in the background: it records the planned
# routes and the driven path into $REC/nav_<route>/ and writes the jobs-page line <run>_nav.
# repository folders and addresses (REPO_ROOT, RECORDS_DIR, WORK_DIR, JOBS_DIR, TOOLS_DIR, *_ADDR): run/lib/paths.sh
. "$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)/../paths.sh"
set -u
RUN="${1:?usage: go_nav.sh <run> A|B}"
ROUTE="${2:?usage: go_nav.sh <run> A|B}"
HERE="$(cd "$(dirname "$0")" && pwd)"
NAVPKG="$REPO_ROOT/catkin_ws/src/sidewalk_navigation"   # launch/, config/, maps/, goals/, scripts/
REC="$RECORDS_DIR/$RUN"
GOALS="$NAVPKG/goals/goals_d10_route_${ROUTE}.yaml"
[ -r "$GOALS" ] || { echo "no $GOALS"; exit 1; }
source /opt/ros/noetic/setup.bash
grep -q "NAV_SENDER_PID" "$REC/nav_pids" 2>/dev/null || { echo "start_nav.sh $RUN has not run"; exit 1; }
OUT="$REC/nav_${ROUTE}"
[ -e "$OUT/goals_result.json" ] && OUT="${OUT}_$(date +%H%M%S)"
rm -f "$WORK_DIR/NAV_PAUSE"
setsid nohup nice -n 5 python3 "$NAVPKG/scripts/nav_goals.py" --run "$RUN" --goals "$GOALS" --out "$OUT" --fixes --dwell 10 \
    > "$REC/nav_goals_${ROUTE}.log" 2>&1 < /dev/null &
echo "NAV_GOALS_PID=$!" >> "$REC/nav_pids"
echo "route $ROUTE started (pid $!): recording to $OUT; jobs-page line ${RUN}_nav"
echo "pause: touch $WORK_DIR/NAV_PAUSE    stop everything: bash $HERE/stop_nav.sh $RUN"
