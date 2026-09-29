#!/bin/bash
# simulate.sh - the no-hardware path: drive a simulated Husky with a simulated ZED X through a world and map it.
#
#   usage:  run/simulate.sh <world> <route> [run] [--arm A|AY|B0|B1]
#     world:  mcity | office1 | office2 | indoor | sidewalk      (Gazebo worlds in catkin_ws/src/sidewalk_sim/worlds)
#     route:  a file name in catkin_ws/src/sidewalk_sim/routes (mcity_loop.yaml, office1_full.yaml,
#             office2_full.yaml, indoor_loop.yaml, indoor_shuttle.yaml ...) or a path to your own
#     run:    a name for the record folder (default sim_<world>_<time>)
#     arm:    which simulated sensor feeds the map: B1 (default) = stereo camera with realistic image noise,
#             B0 = stereo without noise, A/AY = depth-image camera
#
# Needs Gazebo Classic 11 and a display (the simulated cameras render on the GPU even without a window).
# Writes <records>/<run>/ (logs, checklist.txt, frame yield, timelapse) and the map database under
# <work>/sim_maps/<date>/. Progress: <jobs>/sim_<run>.progress. Stops only the processes it started.
# *Plain terms: the whole mapping pipeline on a computer alone, no robot or camera needed.*
set -euo pipefail
LIB="$(cd "$(dirname "$(readlink -f "$0")")/lib" && pwd)"
usage() { sed -n '2,16p' "$0" | sed 's/^# \{0,1\}//'; exit "${1:-0}"; }
[ $# -ge 2 ] || usage 1
case "$1" in -h|--help) usage;; esac
WORLD="$1"; ROUTE="$2"; shift 2
RUN=""; ARM=B1
while [ $# -gt 0 ]; do
    case "$1" in
        --arm) ARM="$2"; shift;;
        -h|--help) usage;;
        -*) echo "unknown option: $1" >&2; usage 1;;
        *) RUN="$1";;
    esac; shift
done
[ -n "$RUN" ] || RUN="sim_${WORLD}_$(date +%m%d_%H%M)"
# which launch file and starting pose belong to each world (the office/indoor launch files place the robot themselves)
case "$WORLD" in
    mcity)    export WORLD_LAUNCH=husky_mcity_gazebo.launch;;
    office1)  export WORLD_LAUNCH=husky_real_gazebo.launch SPAWN="";;
    office2)  export WORLD_LAUNCH=husky_office2_gazebo.launch SPAWN="";;
    indoor)   export WORLD_LAUNCH=husky_real_gazebo.launch SPAWN="world:=indoor.world";;
    sidewalk) export WORLD_LAUNCH=husky_sidewalk_gazebo.launch SPAWN="";;
    *) echo "unknown world: $WORLD" >&2; usage 1;;
esac
export ROUTE
exec bash "$LIB/sim_run.sh" "$RUN" "$ARM"
