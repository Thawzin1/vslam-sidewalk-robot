#!/bin/bash
# park.sh - end a drive properly: tell the stop helper the robot is parked.
#
#   usage:  run/park.sh <run>          the map closes after a 45 s hold standing still (so the map
#                                      can recognise the start mark), then everything else stops
#           run/park.sh <run> --now    something is wrong: close the map at once, no hold
#
# It only creates a small flag file; the stop helper started by the drive (auto_stop_mapping.py)
# does the closing and writes its verdict to <records>/<run>/autostop.log.
# *Plain terms: "park" is the only thing that ends a drive - standing still never does.*
set -euo pipefail
LIB="$(cd "$(dirname "$(readlink -f "$0")")/lib" && pwd)"
. "$LIB/paths.sh"
[ $# -ge 1 ] && [ "$1" != "-h" ] && [ "$1" != "--help" ] || { sed -n '2,10p' "$0" | sed 's/^# \{0,1\}//'; exit 1; }
RUN="$1"
if [ "${2:-}" = "--now" ]; then
    touch "$WORK_DIR/STOP_$RUN"; echo "STOP_$RUN written: the map closes now (no hold)"
else
    touch "$WORK_DIR/PARK_$RUN"; echo "PARK_$RUN written: keep the robot still for 45 s; the map then closes"
fi
echo "verdict will appear in: $RECORDS_DIR/$RUN/autostop.log"
echo "after an --svo drive, put the camera back to its default recording mode:  bash $TOOLS_DIR/restore_camera_mode.sh"
