#!/bin/bash
# lidar_reference.sh - make the independent LiDAR estimate for a drive, on the Jetson, from the robot's recording.
#
#   usage:  run/lidar_reference.sh <run> [--variant f3dof|colleague] [--no-pull] [--link robot-usb|robot]
#                                        [--validate DIR] [--rate 0.5]
#
# Steps: (1) copy the drive's LiDAR recording off the robot's card (checksummed; skipped with --no-pull or
# when already here); (2) replay it through the colleague's LiDAR mapping pipeline with one change,
# Reg/Force3DoF=true ("f3dof": the map may slide and turn but not tip), producing lidar.tum, wheel.tum,
# lidar_map.png and facts under <replays>/<run>/lidar_ref_<variant>/; (3) optionally compare with a replay
# made on the robot (--validate <its folder>). Needs the colleague's stack built in ~/lidar_slam_ws
# (docs/OPERATIONS.md, "LiDAR reference"). The result is "agreement with an independent LiDAR estimate",
# never ground truth. Progress: <jobs>/<run>_lidar_ref_<variant>_jetson.progress.
# *Plain terms: a second opinion on the path, from a different sensor.*
set -euo pipefail
LIB="$(cd "$(dirname "$(readlink -f "$0")")/lib" && pwd)"
. "$LIB/paths.sh"
LR="$LIB/lidar_reference"
usage() { sed -n '2,14p' "$0" | sed 's/^# \{0,1\}//'; exit "${1:-0}"; }
[ $# -ge 1 ] || usage 1
case "$1" in -h|--help) usage;; esac
RUN="$1"; shift
VARIANT=f3dof; PULL=1; LINK=robot-usb; VALIDATE=""
while [ $# -gt 0 ]; do
    case "$1" in
        --variant) VARIANT="$2"; shift;;
        --no-pull) PULL=0;;
        --link) LINK="$2"; shift;;
        --validate) VALIDATE="$2"; shift;;
        --rate) export RATE="$2"; shift;;
        -h|--help) usage;;
        *) echo "unknown option: $1" >&2; usage 1;;
    esac; shift
done
if [ "$PULL" = 1 ]; then
    echo "== 1. the LiDAR recording, robot card -> $LIDAR_BAG_DIR"
    bash "$LR/pull_bag_from_robot.sh" "$RUN" "$LINK"
fi
echo "== 2. replay ($VARIANT) -> $LIDAR_REPLAY_DIR/$RUN/lidar_ref_$VARIANT/"
bash "$LR/replay_colleague_variant_jetson.sh" "$RUN" wrap "$VARIANT"
if [ -n "$VALIDATE" ]; then
    echo "== 3. compare with the robot's own replay"
    python3 "$LR/validate_against_robot.py" --jetson "$LIDAR_REPLAY_DIR/$RUN/lidar_ref_$VARIANT" --robot "$VALIDATE"
fi
echo "done. Put the products into the results pack with:  run/evaluate.sh $RUN --lidar"
