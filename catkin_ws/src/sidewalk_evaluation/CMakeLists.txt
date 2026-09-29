cmake_minimum_required(VERSION 3.0.2)
project(sidewalk_evaluation)

## Pure Python package: analysis scripts, launch files and configuration.
## Nothing is compiled. CMake exists here only to register the package with
## catkin, to install the Python entry points with their executable bit intact,
## and to place the launch/config assets where roslaunch can resolve them.

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  cv_bridge
  message_filters
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    tf2_ros
    cv_bridge
    message_filters
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Every script is installed into the same destination directory. That matters:
## the analysis scripts import each other as plain sibling modules
## (`from traj_metrics import ...`), which only resolves if they land together.
catkin_install_python(PROGRAMS
  scripts/eval_common.py
  scripts/traj_metrics.py
  scripts/trajectory_recorder.py
  scripts/evaluate_trajectory.py
  scripts/depth_benchmark/depth_characterization.py
  scripts/system_monitor.py
  scripts/compare_slam.py
  scripts/generate_report.py
  scripts/nav_run.py
  scripts/ingest_navigation_run.py
  scripts/depth_benchmark/target_extent.py
  # from the source tree but were never installed, so an install-space-only
  # deployment would not have had them. bench_panels.py is imported by both as
  # a plain sibling module, so all three must land in the same directory or
  # the import fails - see the note at the top of this file.
  scripts/depth_benchmark/bench_panels.py
  scripts/depth_benchmark/bench_report.py
  scripts/depth_benchmark/panel_depth_stats.py
  # sphere_centroid.py is Nicolas's detector, copied byte-for-byte from
  # matt_self_navigation so the camera and the LiDARs are measured by the same
  # code. sphere_centroid_selftest.py imports it as a plain sibling module, so
  # per the note at the top of this file the two must land together.
  scripts/depth_benchmark/sphere_centroid.py
  scripts/depth_benchmark/sphere_centroid_selftest.py
  scripts/depth_benchmark/roi_viz.py
  scripts/depth_benchmark/camera_pose_from_floor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Assets. OPTIONAL keeps the build from failing if a directory is empty during
## early development.
foreach(dir launch config)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir}
    OPTIONAL)
endforeach()
