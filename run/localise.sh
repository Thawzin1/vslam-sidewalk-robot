#!/bin/bash
# localise.sh - find the robot on a saved map (localisation mode: the map is loaded, nothing is added).
#
#   usage:  run/localise.sh <run> [map.db] [--bench] [--kidnap-prior "x y yaw"|none] [--first-mark NAME]
#           run/localise.sh next <run> <k> <mark>     the robot is parked on a new mark: start attempt k there
#           run/localise.sh stop <run> [--keep-camera] end the session and check the master map is untouched
#
# The saved map (MAP_DB, default <work>/localise/s2_static_10_map.db) is the MASTER: RTAB-Map only ever
# opens a working copy, and stop checks the master's checksum afterwards. --bench = camera only, no
# robot (desk test). The "kidnap prior" puts a deliberately wrong start guess in the copy (default
# "40 40 90": 40 m away, so only a real recognition can place the robot); "none" keeps the map's
# saved last position. Before this, on the robot: run/robot_side.sh start <run>.
# *Plain terms: does the robot know where it is on last week's map, from nothing?*
set -euo pipefail
LIB="$(cd "$(dirname "$(readlink -f "$0")")/lib" && pwd)"
. "$LIB/paths.sh"
usage() { sed -n '2,13p' "$0" | sed 's/^# \{0,1\}//'; exit "${1:-0}"; }
[ $# -ge 1 ] || usage 1
case "$1" in
    -h|--help) usage;;
    next) shift; exec bash "$LIB/localise/next_start.sh" "$@";;
    stop) shift; exec bash "$LIB/localise/stop_localise.sh" "$@";;
esac
RUN="$1"; shift
export LOCALISE=1 LOC_TOOLS="$LIB/localise" FUSION=1
export MAP_DB="${MAP_DB:-$WORK_DIR/localise/s2_static_10_map.db}"
while [ $# -gt 0 ]; do
    case "$1" in
        --bench) unset FUSION; echo "bench: camera only (no robot link, no blend)";;
        --kidnap-prior) [ "$2" = none ] && export KIDNAP_PRIOR="" || export KIDNAP_PRIOR="$2"; shift;;
        --first-mark) export FIRST_MARK="$2"; shift;;
        -h|--help) usage;;
        -*) echo "unknown option: $1" >&2; usage 1;;
        *) export MAP_DB="$1";;
    esac; shift
done
if [ "${FUSION:-0}" = 1 ] && [ -z "${ROBOT_ADDRS:-}" ]; then
    export ROBOT_ADDRS="${ROBOT_USB_ADDR:+$ROBOT_USB_ADDR,}${ROBOT_WIFI_ADDR}"
    [ -n "$ROBOT_ADDRS" ] || { echo "no robot address: set ROBOT_WIFI_ADDR / ROBOT_USB_ADDR in env.sh (or use --bench)" >&2; exit 1; }
fi
[ -n "${LIDAR_PEER:-}" ] || [ -z "${ROBOT_WIFI_ADDR}" ] || export LIDAR_PEER="http://$ROBOT_WIFI_ADDR:8095"
echo "== localisation session $RUN on $MAP_DB"
echo "   watch: $JOBS_DIR/${RUN}_localise.progress (jobs page $JOBS_PAGE_URL)"
echo "   next mark:  run/localise.sh next $RUN <k> <mark>     end:  run/localise.sh stop $RUN"
exec bash "$LIB/start_drive_localise.sh" "$RUN"
