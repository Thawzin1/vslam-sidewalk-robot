#!/bin/bash
# mapping.sh - start a mapping drive: camera, wheel/gyro blend, RTAB-Map, live map page, guards, timelapse.
#
#   usage:  run/mapping.sh <run> [--svo] [--no-fusion] [--tracker GEN_1|GEN_2]
#                                 [--robot-addr ADDR[,ADDR]] [--lidar-peer URL]
#   end it: run/park.sh <run>       (only when the robot is parked on its start mark)
#
# What it does: calls run/lib/start_drive_mapping.sh, the drive script used for drives 3-10 (Sept 2026).
# Everything it starts is detached (keeps running if your connection drops). By default the robot's
# wheels and gyroscope are blended with the camera (FUSION); --no-fusion is camera only.
# Before this, on the robot:  run/robot_side.sh start <run>   (LiDAR recording + wheel bridge sender).
#
# *Plain terms: one command starts the whole drive; "park" ends it properly.*
set -euo pipefail
LIB="$(cd "$(dirname "$(readlink -f "$0")")/lib" && pwd)"
. "$LIB/paths.sh"
usage() { sed -n '2,13p' "$0" | sed 's/^# \{0,1\}//'; exit "${1:-0}"; }
[ $# -ge 1 ] || usage 1
case "$1" in -h|--help) usage;; esac
RUN="$1"; shift
export FUSION=1 SVO=0
while [ $# -gt 0 ]; do
    case "$1" in
        --svo) SVO=1;;
        --no-fusion) FUSION=0;;
        --tracker) export TRACKER="$2"; shift;;
        --robot-addr) export ROBOT_ADDRS="$2"; shift;;
        --lidar-peer) export LIDAR_PEER="$2"; shift;;
        -h|--help) usage;;
        *) echo "unknown option: $1" >&2; usage 1;;
    esac; shift
done
export SVO
# the robot's addresses come from env.sh (paths.sh reads it); the drive script needs at least one
if [ "$FUSION" = 1 ] && [ -z "${ROBOT_ADDRS:-}" ]; then
    export ROBOT_ADDRS="${ROBOT_USB_ADDR:+$ROBOT_USB_ADDR,}${ROBOT_WIFI_ADDR}"
    [ -n "$ROBOT_ADDRS" ] || { echo "no robot address: set ROBOT_WIFI_ADDR / ROBOT_USB_ADDR in env.sh, or pass --robot-addr" >&2; exit 1; }
fi
[ -n "${LIDAR_PEER:-}" ] || [ -z "${ROBOT_WIFI_ADDR}" ] || export LIDAR_PEER="http://$ROBOT_WIFI_ADDR:8095"
echo "== mapping drive $RUN  (FUSION=$FUSION SVO=$SVO)"
echo "   progress lines: $JOBS_DIR/${RUN}*.progress (jobs page $JOBS_PAGE_URL)"
echo "   to end the drive when the robot is on its mark:  run/park.sh $RUN"
exec bash "$LIB/start_drive_mapping.sh" "$RUN"
