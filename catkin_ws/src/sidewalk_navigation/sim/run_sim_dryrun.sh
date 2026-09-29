#!/bin/bash
# run_sim_dryrun.sh - the navigation dry run in Gazebo, start to finish, then stops EVERYTHING it started.
#
#   usage:  run_sim_dryrun.sh <out_dir> [routeA|routeB|both] [safety|nosafety]
#
# Runs on the Jetson, on its OWN ROS master (port 11411) and Gazebo master (port 11445), so the live master
# on 11311 (drives, localisation, the live map page) is never touched. Headless Gazebo; cameras render on the
# Jetson's display session :0 (gzserver needs a display to render the depth camera - ENGINEERING_NOTES.md section 5).
# Every process is started in its own session (setsid) and its process-group id is written to
# <out>/pgids; stop_all kills exactly those groups - never by name (ENGINEERING_NOTES.md rule 8).
# NOTE: do NOT use ~/sidewalk_sim/demo_sim_jetson.sh for this: it runs `pkill -f rosmaster` and
# `pkill -f gzserver`, which would kill the LIVE master on 11311 as well.
set -u
OUT="${1:?usage: run_sim_dryrun.sh <out_dir> [routeA|routeB|both] [safety|nosafety]}"
ROUTES="${2:-both}"
SAFETY="${3:-safety}"
HERE="$(cd "$(dirname "$0")/.." && pwd)"
mkdir -p "$OUT"
OUT="$(cd "$OUT" && pwd)"
CTL="$OUT/ctl"; mkdir -p "$CTL"
PG="$OUT/pgids"; : > "$PG"
PROG="$JOBS_DIR/nav_sim_dryrun.progress"
T0=$(date +%s)
prog() { echo "NAV_SIM $1  $(( $(date +%s) - T0 ))s" > "$PROG"; echo "[$(TZ=America/Toronto date +%H:%M:%S)] $1"; }
[ -e "$WORK_DIR/HELPERS_STOP" ] && { echo "HELPERS_STOP exists - not starting"; exit 3; }

source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11411 ROS_IP=127.0.0.1 ROS_HOSTNAME=127.0.0.1
export GAZEBO_MASTER_URI=http://127.0.0.1:11445 GAZEBO_MODEL_DATABASE_URI=""
export DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority
export HUSKY_REALSENSE_ENABLED=1 HUSKY_REALSENSE_PARENT=base_link \
       HUSKY_REALSENSE_XYZ="0.071 0.02 0.563" HUSKY_REALSENSE_RPY="0 0.0565 0"
export ROS_LOG_DIR="$OUT/roslog"
# sim-only turn calibration (sim_husky_extras.yaml explains); SIM_TURN_CAL=0 keeps Clearpath's 1.875
[ "${SIM_TURN_CAL:-1}" = 1 ] && export HUSKY_CONFIG_EXTRAS="$HERE/sim/sim_husky_extras.yaml"

start() {   # start NAME CMD... : own session, nice 19, pgid recorded
    local name="$1"; shift
    setsid nohup nice -n 19 "$@" > "$OUT/$name.log" 2>&1 < /dev/null &
    local pid=$!
    echo "$name $pid" >> "$PG"
    echo "$pid"
}
stop_all() {
    prog "stopping everything this run started"
    tac "$PG" | while read -r name pid; do
        kill -INT -- "-$pid" 2>/dev/null
    done
    for i in $(seq 1 20); do
        alive=0
        while read -r name pid; do
            ps -o pid= -g "$pid" > /dev/null 2>&1 && alive=1
        done < "$PG"
        [ $alive = 0 ] && break
        sleep 1
    done
    while read -r name pid; do
        left=$(ps -o pid= -g "$pid" 2>/dev/null | tr '\n' ' ')
        [ -n "$left" ] && { echo "   $name group $pid still has: $left - SIGKILL"; kill -KILL -- "-$pid" 2>/dev/null; }
    done < "$PG"
    # the sender restarted by safety_tests.py lives in its own session: its pid file is authoritative
    [ -f "$OUT/sender.pid" ] && kill -INT "$(cat "$OUT/sender.pid")" 2>/dev/null
    sleep 2
    [ -f "$OUT/sender.pid" ] && kill -KILL "$(cat "$OUT/sender.pid")" 2>/dev/null
    echo "   stopped. Left on port 11411/11445: $(ss -ltnp 2>/dev/null | grep -cE ':(11411|11445) ') listeners"
}
trap stop_all EXIT

prog "0/6 roslaunch (Gazebo + Husky + map + move_base)"
LPID=$(start launch roslaunch -p 11411 "$HERE/sim/sim_nav.launch" here:="$HERE")
sleep 8
start localiser python3 "$HERE/sim/sim_localiser.py" 2.0 0.05 1.0 > /dev/null
for i in $(seq 1 120); do
    [ -e "$WORK_DIR/HELPERS_STOP" ] && { echo "HELPERS_STOP appeared"; exit 3; }
    timeout 5 rosservice list 2>/dev/null | grep -qx /move_base/DWAPlannerROS/set_parameters && \
        timeout 5 rostopic echo -n1 /nav/obstacles_cloud/header > /dev/null 2>&1 && break
    sleep 2
done
timeout 5 rosservice list 2>/dev/null | grep -qx /move_base/DWAPlannerROS/set_parameters || { prog "FAILED: move_base never came up"; exit 1; }
prog "1/6 sim up; turn-on-the-spot probe, then the command link (robot side, joystick stand-in, Jetson side)"
if [ "${PROBE:-0}" = 1 ]; then   # leaves the robot turned: only for a probe-only run
    nice -n 19 python3 "$HERE/sim/turn_probe.py" "$OUT" > "$OUT/turn_probe.log" 2>&1
    tail -1 "$OUT/turn_probe.log"
fi
start recv python3 "$HERE/nav_cmd_recv.py" __name:=nav_cmd_recv _run:=sim_dry _bind:=127.0.0.1 \
    _allow_sim_time:=true _deadman_button:=7 _jobs_dir:="$OUT" > /dev/null
start simjoy python3 "$HERE/sim/sim_joy.py" "$CTL" 7 > /dev/null
SENDER_CMD="cd '$HERE' || exit 1; export ROS_MASTER_URI=$ROS_MASTER_URI ROS_IP=127.0.0.1; nice -n 19 python3 nav_cmd_sender.py __name:=nav_cmd_sender _run:=sim_dry _robot:=127.0.0.1 _odom_topic:=/odometry/filtered _allow_sim_time:=true _pause_file:='$CTL/NAV_PAUSE' _jobs_dir:='$JOBS_DIR' >> '$OUT/sender.log' 2>&1 < /dev/null & echo \$! > '$OUT/sender.pid'"
echo "$SENDER_CMD" > "$OUT/sender_restart_cmd.txt"
setsid bash -c "$SENDER_CMD"
sleep 4
grep -q "CONNECTED" "$OUT/sender.log" || { prog "FAILED: sender did not connect to the receiver"; exit 1; }

prog "2/6 reading back the settings move_base actually loaded (rule 4)"
rosparam get /move_base > "$OUT/params_move_base.yaml" 2>&1
for ns in DWAPlannerROS GlobalPlanner global_costmap/obstacle_layer local_costmap/obstacle_layer \
          global_costmap/inflation_layer local_costmap/inflation_layer global_costmap/static_layer; do
    echo "== /move_base/$ns" >> "$OUT/dynparam_readback.txt"
    timeout 20 rosrun dynamic_reconfigure dynparam get "/move_base/$ns" >> "$OUT/dynparam_readback.txt" 2>&1
done
rosparam get /nav_obstacles > "$OUT/params_obstacles.yaml" 2>&1
rosparam get /twist_mux > "$OUT/params_twist_mux.yaml" 2>&1
rosparam get /husky_velocity_controller > "$OUT/params_husky_velocity_controller.yaml" 2>&1

run_route() {   # run_route NAME GOALS
    prog "$3 route $1 (goals: $2)"
    nice -n 19 python3 "$HERE/nav_goals.py" --run "sim_$1" --goals "$2" --out "$OUT/$1" \
        --truth-model husky --dwell 3 --jobs-dir "$JOBS_DIR" > "$OUT/goals_$1.log" 2>&1
    tail -2 "$OUT/goals_$1.log"
}
case "$ROUTES" in
    routeA) run_route A "$HERE/goals_d10_route_A.yaml" "3/6" ;;
    routeB) run_route B "$HERE/goals_d10_route_B.yaml" "3/6" ;;
    routeB2) run_route B2 "$HERE/goals_d10_route_B2.yaml" "3/6" ;;
    both)   run_route A "$HERE/goals_d10_route_A.yaml" "3/6"
            run_route B "$HERE/goals_d10_route_B.yaml" "4/6" ;;
esac
if [ "$SAFETY" = safety ]; then
    prog "5/6 safety stop tests T1-T7"
    nice -n 19 python3 "$HERE/sim/safety_tests.py" "$CTL" "$OUT/safety" "$OUT/sender.pid" \
        "$OUT/sender_restart_cmd.txt" > "$OUT/safety_tests.log" 2>&1
    tail -3 "$OUT/safety_tests.log"
fi
prog "6/6 done - stopping"
