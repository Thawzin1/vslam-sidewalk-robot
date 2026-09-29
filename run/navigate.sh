#!/bin/bash
# navigate.sh - drive a route of goals on the saved map, on top of a RUNNING localisation.
#
#   usage:  run/navigate.sh <run> <route A|B|B2|B2home|B2rest>   start the navigation layer, wait for you
#                                                                to hold the autonomy button, then send the goals
#           run/navigate.sh start <run>       only the layer (map server, camera obstacles, move_base, command link)
#           run/navigate.sh go <run> <route>  send a route's goals (the button must be held)
#           run/navigate.sh pause <run>       shut the gate: the robot is told "stop" at once
#           run/navigate.sh stop <run>        stop the navigation layer (the localisation keeps running)
#
# Order for a real run: robot: run/robot_side.sh start <run>; robot: nav_cmd_recv.py (the command receiver,
# started by hand, see catkin_ws/src/sidewalk_navigation/doc/DEPLOY_NOTES.md); Jetson: run/localise.sh <run>
# with --kidnap-prior "0 0 0" (the robot IS on the start mark); then this. Routes are goal lists in
# catkin_ws/src/sidewalk_navigation/goals/. Wheel commands reach the robot only through the command link.
# *Plain terms: the robot knows where it is; now it is asked to go somewhere by itself.*
set -euo pipefail
LIB="$(cd "$(dirname "$(readlink -f "$0")")/lib" && pwd)"
. "$LIB/paths.sh"
NAV="$LIB/navigation"
usage() { sed -n '2,15p' "$0" | sed 's/^# \{0,1\}//'; exit "${1:-0}"; }
[ $# -ge 1 ] || usage 1
case "$1" in
    -h|--help) usage;;
    start) exec bash "$NAV/start_nav.sh" "${2:?run}";;
    go)    exec bash "$NAV/go_nav.sh" "${2:?run}" "${3:?route}";;
    pause) touch "$WORK_DIR/NAV_PAUSE"; echo "gate shut (NAV_PAUSE): the robot is told stop; resume with: run/navigate.sh go ${2:-<run>} <route>"; exit 0;;
    stop)  exec bash "$NAV/stop_nav.sh" "${2:?run}";;
esac
RUN="$1"; ROUTE="${2:?usage: run/navigate.sh <run> <route>}"
[ -r "$REPO_ROOT/catkin_ws/src/sidewalk_navigation/goals/goals_d10_route_${ROUTE}.yaml" ] || { echo "no route $ROUTE (see catkin_ws/src/sidewalk_navigation/goals/)" >&2; exit 1; }
bash "$NAV/start_nav.sh" "$RUN"
echo
echo "== The gate is SHUT. Stand beside the robot, HOLD the autonomy button, then press Enter to send route $ROUTE."
echo "   (Ctrl-C leaves the layer running with the gate shut; run/navigate.sh stop $RUN ends it.)"
read -r _
exec bash "$NAV/go_nav.sh" "$RUN" "$ROUTE"
