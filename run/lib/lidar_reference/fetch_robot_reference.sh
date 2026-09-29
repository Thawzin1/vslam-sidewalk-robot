#!/bin/bash
# fetch_robot_reference.sh - READ-ONLY look at the robot, to close the two gaps the Jetson copy could not close
# while the robot was off (it changes nothing on the robot; ssh + cat/md5sum/sqlite read only):
#   R1  the robot's own rslidar_sdk/config/replay_config.yaml -> robot_copy/replay_config.yaml.robot
#       (expected md5 cb9ca662ae093bf9763782b3da43ef5d, from drive 10's provenance), and whether its settings
#       equal robot_replay_config.yaml (the reconstruction the Jetson has been using)
#   R2  md5 of every colleague file the replay uses, robot vs Jetson copy (self_navigation launches + scripts,
#       mcm07_husky URDF + body-filter settings, the rslidar_sdk source tree), and the robot's package versions
#   R3  the parameters the robot's map program actually ran with for <run>'s f3dof replay (the map's Info table)
#       -> validation_<run>/robot_params_db.txt, for validate_against_robot.py --robot-params
#
#   usage: ./fetch_robot_reference.sh [run=s2_static_10] [robot-usb|robot]
set -u
RUN="${1:-s2_static_10}"
HOST="${2:-robot-usb}"
HERE="$(cd "$(dirname "$0")" && pwd)"
V="$HERE/validation_$RUN"; mkdir -p "$V"
REP="$HERE/robot_check_$(TZ=America/Toronto date +%Y-%m-%d_%H%M).txt"
RW="/home/administrator/catkin_ws/src"
JW="$HOME/lidar_slam_ws"
CARD="/media/administrator/USB Drive/slam_series2"
timeout 15 ssh "$HOST" true 2>/dev/null || { echo "the robot does not answer on '$HOST'"; exit 1; }
{
echo "fetch_robot_reference.sh $RUN via $HOST, $(TZ=America/Toronto date '+%Y-%m-%d %H:%M %Z')"
echo "== R1 decoder config"
scp -q "$HOST:$RW/rslidar_sdk/config/replay_config.yaml" "$HERE/robot_copy/replay_config.yaml.robot" \
    && echo "robot copy: $(md5sum < "$HERE/robot_copy/replay_config.yaml.robot" | cut -c1-32) (expected cb9ca662ae093bf9763782b3da43ef5d)"
python3 - "$HERE/robot_copy/replay_config.yaml.robot" "$HERE/robot_replay_config.yaml" <<'PY'
import sys, yaml
a, b = (yaml.safe_load(open(p)) for p in sys.argv[1:3])
print("settings equal (comments ignored): %s" % (a == b))
if a != b:
    print("robot:", a); print("jetson:", b)
    print("ACTION: copy robot_copy/replay_config.yaml.robot over robot_replay_config.yaml (keep a header line) and re-run")
PY
echo "== R2 colleague files, robot vs Jetson copy"
for f in matt_self_navigation/launch/3dreplay_pipeline.launch matt_self_navigation/launch/rtabmap_3d.launch \
         matt_self_navigation/launch/helios_body_filter.launch matt_self_navigation/scripts/helios_restamp.py \
         matt_self_navigation/scripts/box_crop.py mcm07_husky/config/husky_full.urdf \
         mcm07_husky/config/robot_body_filter_helios.yaml; do
    r=$(ssh "$HOST" "md5sum '$RW/$f' 2>/dev/null" | cut -c1-32)
    case "$f" in matt_self_navigation/*|mcm07_husky/*) j=$(md5sum < "$JW/nobuild/$f" | cut -c1-32) ;; esac
    [ "$r" = "$j" ] && s=same || s=DIFFERENT
    echo "$s  $f  robot $r  jetson $j"
done
# rslidar_sdk: hash of every source file (build products and the generated cmake file left out)
tree_hash='find . -type f \( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" -o -name "*.c" -o -name CMakeLists.txt -o -name package.xml \) ! -path "./build*" | LC_ALL=C sort | xargs md5sum | md5sum | cut -c1-32'
r=$(ssh "$HOST" "cd '$RW/rslidar_sdk' && $tree_hash")
j=$(cd "$JW/src/rslidar_sdk" && eval "$tree_hash")
[ "$r" = "$j" ] && s=same || s=DIFFERENT
echo "$s  rslidar_sdk source tree  robot $r  jetson $j"
echo "== robot package versions"
ssh "$HOST" "dpkg -l | awk '/ros-noetic-(rtabmap-slam|sensor-filters|robot-body-filter) /{print \$2, \$3}'"
echo "== R3 the robot's map parameters for $RUN f3dof"
ssh "$HOST" "python3 -c \"
import sqlite3
c = sqlite3.connect('file:$CARD/$RUN/lidar_ref_f3dof/rtab_helios.db?mode=ro', uri=True)
rows = [r[0] for r in c.execute('SELECT parameters FROM Info') if r[0]]
for kv in sorted(set(p for r in rows for p in r.split(';') if p)):
    print(kv.replace(':', ' = ', 1))
\"" > "$V/robot_params_db.txt" 2> "$V/robot_params_db.err"
echo "$(wc -l < "$V/robot_params_db.txt") parameters -> validation_$RUN/robot_params_db.txt $(cat "$V/robot_params_db.err")"
[ -s "$V/robot_params_db.err" ] || rm -f "$V/robot_params_db.err"
} 2>&1 | tee "$REP"
echo "written: $REP"
