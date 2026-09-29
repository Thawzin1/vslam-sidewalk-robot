#!/bin/bash
# start_nav.sh - add the navigation layer to a RUNNING localisation (real robot). Jetson only.
#
#   usage:  start_nav.sh <run>                 e.g. start_nav.sh s2_nav_01
#
# BEFORE this (DEPLOY_NOTES.md / PROTOCOL.md), in this order:
#   robot:  robot_side.sh start <run>                         (LiDAR recording + wheel bridge sender)
#   robot:  nav_cmd_recv.py (ONLY after the user's approval)   (the command receiver, USB-C wire only)
#   Jetson: MAP_DB=$WORK_DIR/localise/s2_static_10_map.db KIDNAP_PRIOR="0 0 0" \
#           ROBOT_ADDRS=$ROBOT_USB_ADDR,<robot WiFi address> \
#           (waits for READY; KIDNAP_PRIOR "0 0 0" = "you are on the start mark, facing east")
#
# This script then: refuses unless the localisation is really up (map -> odom from RTAB-Map, odom -> base_link
# from the blend, fresh camera depth, /use_sim_time not true); starts nav_run.launch (map server, camera
# obstacles, move_base); reads back every navigation setting move_base actually loaded (rule 4); starts
# nav_cmd_sender.py and waits for the robot's receiver to answer. It does NOT send any goal: go_nav.sh does,
# when the user is holding the autonomy button and says "go".
# Process numbers go to $REC/nav_pids (NAME=pid); stop_nav.sh stops exactly those.
# repository folders and addresses (REPO_ROOT, RECORDS_DIR, WORK_DIR, JOBS_DIR, TOOLS_DIR, *_ADDR): run/lib/paths.sh
. "$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)/../paths.sh"
set -u
RUN="${1:?usage: start_nav.sh <run>}"
HERE="$(cd "$(dirname "$0")" && pwd)"
NAVPKG="$REPO_ROOT/catkin_ws/src/sidewalk_navigation"   # launch/, config/, maps/, goals/, scripts/
REC="$RECORDS_DIR/$RUN"
ROBOT_CMD_ADDRS="${ROBOT_CMD_ADDRS:-$ROBOT_USB_ADDR}"
PAUSE="$WORK_DIR/NAV_PAUSE"
die() { echo "!! REFUSED: $*" >&2; exit 1; }
step() { echo; echo "== $(TZ=America/Toronto date +%H:%M:%S) $*"; }
source /opt/ros/noetic/setup.bash
[ -f "$CATKIN_WS/devel/setup.bash" ] && source "$CATKIN_WS/devel/setup.bash"
[ -d "$REC" ] || die "$REC does not exist - start the localisation first (start_localise.sh $RUN)"

step "1. is the localisation really up?"
timeout 5 rostopic list > /dev/null 2>&1 || die "no ROS master on $ROS_MASTER_URI"
[ "$(rosparam get /use_sim_time 2>/dev/null || echo unset)" = "true" ] && die "/use_sim_time is true"
timeout 8 rostopic echo -n1 /fused/odometry/header > /dev/null 2>&1 || die "/fused/odometry silent (the blend is not running)"
timeout 8 rostopic echo -n1 /zedx_front/zed_node/depth/camera_info/header > /dev/null 2>&1 || die "camera depth silent"
timeout 15 python3 -c "import rospy, tf2_ros
rospy.init_node('nav_tf_check', anonymous=True)
b = tf2_ros.Buffer(); l = tf2_ros.TransformListener(b)
t = b.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(8)).transform.translation
print('map -> base_link: x %.2f y %.2f' % (t.x, t.y))" > "$REC/nav_tf_check.txt" 2>&1 \
    || die "no map -> base_link transform (RTAB-Map localisation not publishing) - $REC/nav_tf_check.txt"
echo "   $(cat "$REC/nav_tf_check.txt") (the robot should be near 0, 0: the start mark)"
grep -q "Localization mode" "$REC/mapping.log" 2>/dev/null || echo "   (warning: 'Localization mode' not found in $REC/mapping.log - check it)"
echo "   ok: blend, camera depth, map -> base_link"
python3 "$NAVPKG/scripts/check_goals.py" "$NAVPKG/maps/d10_nav.yaml" "$NAVPKG/goals/goals_d10_route_A.yaml" "$NAVPKG/goals/goals_d10_route_B.yaml" \
    | sed 's/^/   /' || die "a goal failed check_goals.py"

step "2. navigation layer (map server, camera obstacles, move_base)"
touch "$PAUSE"     # the gate stays SHUT until go_nav.sh removes this
setsid nohup roslaunch "$NAVPKG/launch/nav_run.launch" here:="$NAVPKG" > "$REC/nav_launch.log" 2>&1 < /dev/null &
echo "NAV_LAUNCH_PID=$!" >> "$REC/nav_pids"
for i in $(seq 1 60); do
    timeout 5 rosservice list 2>/dev/null | grep -qx /move_base/DWAPlannerROS/set_parameters && break
    sleep 2
done
timeout 5 rosservice list 2>/dev/null | grep -qx /move_base/DWAPlannerROS/set_parameters \
    || die "move_base did not come up - see $REC/nav_launch.log (stop with stop_nav.sh $RUN)"
timeout 10 rostopic echo -n1 /nav/obstacles_cloud/header > /dev/null 2>&1 || die "no camera obstacle points on /nav/obstacles_cloud"
echo "   move_base up; camera obstacle points flowing"

step "3. read back what move_base actually loaded (ENGINEERING_NOTES.md rule 4)"
rosparam get /move_base > "$REC/nav_params_move_base.yaml" 2>&1
: > "$REC/nav_dynparam_readback.txt"
for ns in DWAPlannerROS GlobalPlanner global_costmap/obstacle_layer local_costmap/obstacle_layer \
          global_costmap/inflation_layer local_costmap/inflation_layer; do
    echo "== /move_base/$ns" >> "$REC/nav_dynparam_readback.txt"
    timeout 20 rosrun dynamic_reconfigure dynparam get "/move_base/$ns" >> "$REC/nav_dynparam_readback.txt" 2>&1
done
grep -o "'max_vel_x': [0-9.]*\|'max_vel_theta': [0-9.]*" "$REC/nav_dynparam_readback.txt" | sed 's/^/   /'

step "4. the command link to the robot (nav_cmd_sender.py -> ${ROBOT_CMD_ADDRS}:8112)"
setsid nohup python3 "$NAVPKG/scripts/nav_cmd_sender.py" __name:=nav_cmd_sender _run:="$RUN" _robot:="$ROBOT_CMD_ADDRS" \
    _pause_file:="$PAUSE" > "$REC/navcmd_sender.log" 2>&1 < /dev/null &
echo "NAV_SENDER_PID=$!" >> "$REC/nav_pids"
for i in $(seq 1 15); do grep -q "CONNECTED" "$JOBS_DIR/${RUN}_navcmd_events.log" 2>/dev/null && break; sleep 2; done
if grep -q "CONNECTED" "$JOBS_DIR/${RUN}_navcmd_events.log" 2>/dev/null; then
    echo "   $(grep CONNECTED "$JOBS_DIR/${RUN}_navcmd_events.log" | tail -1)"
else
    echo "   !! the robot's receiver is not answering on ${ROBOT_CMD_ADDRS}:8112 - the sender keeps trying."
    echo "      Is it started on the robot (DEPLOY_NOTES.md step R3)? Is the USB-C wire up (ping $ROBOT_USB_ADDR)?"
fi
echo
echo "== READY (the gate is SHUT: $PAUSE exists). Jobs page: ${RUN}_navcmd, ${RUN}_localise ($JOBS_PAGE_URL)"
echo "   When the user holds the autonomy button and says 'go':  bash $HERE/go_nav.sh $RUN A"
echo "   Stop at any time:  touch $PAUSE   (resume: go_nav.sh)   |   end:  bash $HERE/stop_nav.sh $RUN"
