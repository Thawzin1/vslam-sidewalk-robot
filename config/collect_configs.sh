#!/bin/bash
# collect_configs.sh - refresh config/: a documented MIRROR of every parameter file the workflows use.
#
#   usage:  config/collect_configs.sh          (run from anywhere; prints what it copied)
#
# The ROS packages keep the copies roslaunch reads (the source of truth); this folder holds a flat,
# read-only mirror so a reader can see every setting in one place next to CONFIG_GUIDE.md.
# Edit the package copy, then run this again. Files copied here are named <package>__<file>.yaml.
# *Plain terms: one folder with every knob, for reading; the real knobs stay in the packages.*
set -euo pipefail
HERE="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"; R="$(cd "$HERE/.." && pwd)"
copy() { cp -p "$R/$1" "$HERE/$2"; printf '  %-55s <- %s\n' "$2" "$1"; }
echo "config/ mirror refreshed from:"
copy catkin_ws/src/sidewalk_slam/config/rtabmap_zedx.yaml            sidewalk_slam__rtabmap_zedx.yaml
copy catkin_ws/src/sidewalk_slam/config/rtabmap_localization.yaml    sidewalk_slam__rtabmap_localization.yaml
copy catkin_ws/src/sidewalk_slam/config/ekf_fused.yaml               sidewalk_slam__ekf_fused.yaml
copy catkin_ws/src/sidewalk_slam/config/slam_runner.yaml             sidewalk_slam__slam_runner.yaml
copy catkin_ws/src/sidewalk_perception/config/zedx_front.yaml        sidewalk_perception__zedx_front.yaml
copy catkin_ws/src/sidewalk_bringup/config/robot_frames.yaml         sidewalk_bringup__robot_frames.yaml
for f in costmap_common costmap_global costmap_local dwa global_planner move_base obstacles; do
  copy catkin_ws/src/sidewalk_navigation/config/$f.yaml              sidewalk_navigation__$f.yaml; done
copy run/lib/slam_session/zedx_front_od.yaml                         slam_session__zedx_front_od.yaml
copy run/lib/slam_session/zedx_front_od_replay.yaml                  slam_session__zedx_front_od_replay.yaml
copy run/lib/lidar_reference/robot_replay_config.yaml                lidar_reference__robot_replay_config.yaml
copy run/lib/localise/localise_run.launch                            localise__localise_run.launch
copy run/lib/slam_session/slam_run.launch                            slam_session__slam_run.launch
echo "done: $(ls "$HERE" | grep -c '__') files; meanings in config/CONFIG_GUIDE.md"
