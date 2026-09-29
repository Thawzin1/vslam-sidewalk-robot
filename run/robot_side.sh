#!/bin/bash
# robot_side.sh - the robot computer's half of a drive: LiDAR recording, gyro re-zero, live LiDAR view, wheel bridge.
#
#   usage (ON THE ROBOT's computer):  run/robot_side.sh start <run> | stop <run> | storage - | selftest -
#   usage (on the Jetson):            the same line is printed as an ssh command for you to run on the robot
#
# start: re-zero the gyroscope while the wheels are still, start the LiDAR packet recording (record_lidar.sh,
#        to the robot's card), the live LiDAR view for the browser, and the bridge sender that copies the
#        wheels and gyroscope to the Jetson; stop: close the recording properly, then the view, then the sender
#        (run it AFTER "park" has closed the map); storage: the read-only disk reporter for the storage page.
# The tools live in tools/robot_side/ and expect the colleague's self_navigation package on the robot.
# *Plain terms: the robot records what its own sensors saw, and shares its wheel readings with the Jetson.*
set -euo pipefail
LIB="$(cd "$(dirname "$(readlink -f "$0")")/lib" && pwd)"
. "$LIB/paths.sh"
[ $# -ge 2 ] && [ "$1" != "-h" ] && [ "$1" != "--help" ] || { sed -n '2,12p' "$0" | sed 's/^# \{0,1\}//'; exit 1; }
if rospack find self_navigation >/dev/null 2>&1 || [ -d "/media/administrator/USB Drive/slam_series2" ]; then
    exec bash "$REPO_ROOT/tools/robot_side/robot_side.sh" "$@"
fi
echo "This must run on the robot's computer. On the robot (ask the project owner for the ssh alias):"
echo "    bash <this repository on the robot>/run/robot_side.sh $*"
echo "or from the Jetson:  ssh ${ROBOT_SSH:-robot} 'bash <repo on the robot>/run/robot_side.sh $*'"
exit 2
