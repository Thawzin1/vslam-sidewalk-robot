cmake_minimum_required(VERSION 3.0.2)
project(sidewalk_slam)

# ---------------------------------------------------------------------------
# This package is pure Python. Nothing is compiled here, so CMake exists only
# to declare the catkin dependency graph and to install the scripts, launch
# files and configuration into the devel/install space where `rosrun` and
# `roslaunch` can find them.
#
# In particular this file does NOT try to build or find rtabmap_ros. That is
# the job of scripts/build_rtabmap_ros.py, which builds it once, by hand,
# against the RTAB-Map core already installed in /usr/local. Adding
# find_package(rtabmap_ros) here would make this package unbuildable on any
# machine that has not been through that step - including a computer used
# only to write documentation.
# ---------------------------------------------------------------------------
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
  rosbag
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    tf2_ros
    tf2_geometry_msgs
    rosbag
)

include_directories(${catkin_INCLUDE_DIRS})

# ---------------------------------------------------------------------------
# Executable Python nodes and command-line tools.
#
# slam_common.py is installed alongside them because every other script imports
# it as a sibling module. catkin_install_python rewrites its shebang, which is
# harmless for a module that is never run directly (it does have a __main__
# block - an environment probe - which is useful to be able to run).
# ---------------------------------------------------------------------------
catkin_install_python(PROGRAMS
  scripts/slam_common.py
  scripts/build_rtabmap_ros.py
  scripts/make_orbslam3_config.py
  scripts/run_orbslam3.py
  scripts/trajectory_recorder.py
  scripts/slam_runner.py
  scripts/validate_slam_run.py
  scripts/map_manager.py
  scripts/run_monitor.py
  # here, so an install-space-only deployment would not have them - and
  # verify_run.py in particular is named by every run procedure as the step
  # that proves a recording is intact.
  scripts/grid_sample.py
  scripts/map_to_image.py
  scripts/verify_run.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# ---------------------------------------------------------------------------
# Non-code assets.
#
# config/ must be installed: make_orbslam3_config.py locates its templates
# relative to its own file, and slam_runner.py loads config/slam_runner.yaml the
# same way. If config/ is missing from the install space, both fail with a
# specific "template not found" message rather than silently using defaults.
# ---------------------------------------------------------------------------
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(FILES explain.yaml README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  OPTIONAL
)
