cmake_minimum_required(VERSION 3.0.2)
project(sidewalk_perception)

# ---------------------------------------------------------------------------
# This package is currently pure Python. There is nothing to compile, so the
# CMake file exists only to (a) declare the catkin dependency graph and
# (b) install the scripts, launch files and config into the devel/install
# space so that `rosrun` and `roslaunch` can find them.
#
# We still list the C++ dependencies in find_package: catkin uses them to
# order the workspace build, and the depth-validity node is a likely future
# C++ rewrite (see package.xml).
# ---------------------------------------------------------------------------
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2_ros
  cv_bridge
  image_transport
  message_filters
  rosbag
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    sensor_msgs
    std_msgs
    geometry_msgs
    tf2_ros
    cv_bridge
    image_transport
    message_filters
    rosbag
)

include_directories(${catkin_INCLUDE_DIRS})

# ---------------------------------------------------------------------------
# Executable Python nodes.
#
# perception_common.py is installed alongside them because the nodes import it
# as a sibling module. catkin_install_python rewrites the shebang, which is
# harmless for a module that is never executed directly.
# ---------------------------------------------------------------------------
catkin_install_python(PROGRAMS
  scripts/sensor_validator.py
  scripts/frame_audit.py
  scripts/svo_manager.py
  scripts/make_manifest.py
  scripts/obstacle_segmenter.py
  scripts/perception_common.py
  # tree but absent here, so an install-space-only deployment would not have
  # them — while zedx_front.launch and robot_frames.yaml both instruct the
  # operator to run the first two by name.
  scripts/measure_camera_tilt.py
  scripts/check_gravity.py
  scripts/cloud_decimator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# ---------------------------------------------------------------------------
# Non-code assets.
#
# explain.yaml is installed too, so that a deployed (install-space-only) robot
# still carries its own documentation source. That is a deliberate policy for
# this project: the machine in the field should be able to explain itself.
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
