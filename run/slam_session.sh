#!/bin/bash
# slam_session.sh - continue a saved map: load it, keep mapping (new area joins the old map).
#
#   usage:  run/slam_session.sh <run> [map.db] [--bench] [--mask] [--no-svo] [--loop-thr 0.08]
#                                     [--no-live-obstacles]
#           run/slam_session.sh stop <run> [--now]    after "park" closed the map: the checks and report
#
# A COPY of the saved map (default <work>/localise/s2_static_10_map.db) becomes <work>/<run>.db and
# RTAB-Map adds today's drive to it. --mask blanks people out of the depth before the map (the camera's
# people detector must have been optimised once on this machine). The camera is recorded (SVO) unless
# --no-svo. End the drive with run/park.sh <run>, then run the stop checks.
# *Plain terms: last week's map plus today's corridor, joined where the camera recognises the join.*
set -euo pipefail
LIB="$(cd "$(dirname "$(readlink -f "$0")")/lib" && pwd)"
. "$LIB/paths.sh"
usage() { sed -n '2,12p' "$0" | sed 's/^# \{0,1\}//'; exit "${1:-0}"; }
[ $# -ge 1 ] || usage 1
case "$1" in
    -h|--help) usage;;
    stop) shift; exec bash "$LIB/slam_session/stop_slam.sh" "$@";;
esac
RUN="$1"; shift
export SLAM=1 SLAM_TOOLS="$LIB/slam_session" FUSION=1 SVO=1 LIVE_OBSTACLES=1
export MAP_DB="${MAP_DB:-$WORK_DIR/localise/s2_static_10_map.db}"
while [ $# -gt 0 ]; do
    case "$1" in
        --bench) unset FUSION; echo "bench: camera only (no robot link, no blend)";;
        --mask) export MASK=1;;
        --no-svo) SVO=0;;
        --loop-thr) export LOOP_THR="$2"; shift;;
        --no-live-obstacles) LIVE_OBSTACLES=0;;
        -h|--help) usage;;
        -*) echo "unknown option: $1" >&2; usage 1;;
        *) export MAP_DB="$1";;
    esac; shift
done
export SVO
if [ "${FUSION:-0}" = 1 ] && [ -z "${ROBOT_ADDRS:-}" ]; then
    export ROBOT_ADDRS="${ROBOT_USB_ADDR:+$ROBOT_USB_ADDR,}${ROBOT_WIFI_ADDR}"
    [ -n "$ROBOT_ADDRS" ] || { echo "no robot address: set ROBOT_WIFI_ADDR / ROBOT_USB_ADDR in env.sh (or use --bench)" >&2; exit 1; }
fi
[ -n "${LIDAR_PEER:-}" ] || [ -z "${ROBOT_WIFI_ADDR}" ] || export LIDAR_PEER="http://$ROBOT_WIFI_ADDR:8095"
REC="$RECORDS_DIR/$RUN"; mkdir -p "$REC"
if [ "$LIVE_OBSTACLES" = 1 ]; then
    # the camera's obstacle points for the live map page (people and objects appear and disappear live);
    # waits for the camera's depth, then runs in its own session; stop_slam.sh stops it by its pid
    ( for i in $(seq 1 150); do
          set +u; source /opt/ros/noetic/setup.bash; set -u
          rostopic list 2>/dev/null | grep -qx /zedx_front/zed_node/depth/depth_registered && break; sleep 2; done
      exec roslaunch --wait "$LIB/slam_session/live_obstacles.launch" here:="$REPO_ROOT/catkin_ws/src/sidewalk_navigation" \
          > "$REC/live_obstacles.log" 2>&1 ) < /dev/null &
    echo $! > "$REC/live_obstacles.pid"; disown
fi
echo "== SLAM session $RUN continues $MAP_DB  (MASK=${MASK:-0} SVO=$SVO)"
echo "   watch: $JOBS_DIR/${RUN}_slam.progress (jobs page $JOBS_PAGE_URL)"
echo "   end: run/park.sh $RUN, then  run/slam_session.sh stop $RUN"
exec bash "$LIB/start_drive_slam.sh" "$RUN"
