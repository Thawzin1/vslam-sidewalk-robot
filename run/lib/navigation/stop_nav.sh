#!/bin/bash
# stop_nav.sh - stop the navigation layer (real run), robot-safe order. Jetson only. Leaves the localisation,
# camera and wheel bridge running (stop those with stop_localise.sh afterwards, as for a localisation session).
#
#   usage:  stop_nav.sh <run>
#
# Order: 1 pause file (the gate shuts: the robot is sent "stop" within 0.05 s), 2 cancel all goals,
# 3 stop nav_goals.py (its files are flushed every second), 4 stop the sender (its last message is "stop";
# the robot side stops on silence anyway), 5 stop nav_run.launch. Stops ONLY the PIDs in $REC/nav_pids,
# never by name (ENGINEERING_NOTES.md rule 8).
# repository folders and addresses (REPO_ROOT, RECORDS_DIR, WORK_DIR, JOBS_DIR, TOOLS_DIR, *_ADDR): run/lib/paths.sh
. "$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)/../paths.sh"
set -u
RUN="${1:?usage: stop_nav.sh <run>}"
REC="$RECORDS_DIR/$RUN"
source /opt/ros/noetic/setup.bash
touch "$WORK_DIR/NAV_PAUSE"
echo "1. gate shut (NAV_PAUSE)"
timeout 5 rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID '{}' > /dev/null 2>&1 && echo "2. goals cancelled"
[ -f "$REC/nav_pids" ] || { echo "no $REC/nav_pids - nothing else to stop"; exit 0; }
pid_of() { grep "^$1=" "$REC/nav_pids" | tail -1 | cut -d= -f2; }
stop_pid() {   # stop_pid NAME PID [group]
    local n="$1" p="$2" g="${3:-}"
    [ -n "$p" ] || return 0
    ps -o pid= -p "$p" > /dev/null 2>&1 || { echo "   $n ($p) already gone"; return 0; }
    if [ -n "$g" ]; then kill -INT -- "-$p" 2>/dev/null; else kill -INT "$p" 2>/dev/null; fi
    for i in $(seq 1 15); do ps -o pid= -p "$p" > /dev/null 2>&1 || break; sleep 1; done
    if ps -o pid= -p "$p" > /dev/null 2>&1; then
        echo "   $n ($p) did not stop on INT - KILL"
        if [ -n "$g" ]; then kill -KILL -- "-$p" 2>/dev/null; else kill -KILL "$p" 2>/dev/null; fi
    else
        echo "   $n ($p) stopped"
    fi
}
echo "3. goal runner";  for p in $(grep "^NAV_GOALS_PID=" "$REC/nav_pids" | cut -d= -f2); do stop_pid nav_goals "$p"; done
echo "4. command sender"; stop_pid nav_cmd_sender "$(pid_of NAV_SENDER_PID)"
echo "5. navigation launch"; stop_pid nav_run.launch "$(pid_of NAV_LAUNCH_PID)" group
mv "$REC/nav_pids" "$REC/nav_pids.stopped_$(date +%H%M%S)"
echo "done. Localisation still running: stop it with stop_localise.sh $RUN when the robot is parked."
echo "Robot side (only if it was started): stop nav_cmd_recv.py by its pid (DEPLOY_NOTES.md step R5)."
