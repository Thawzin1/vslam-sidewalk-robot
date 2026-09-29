cmake_minimum_required(VERSION 3.0.2)
project(sidewalk_bringup)

## This package is pure Python plus launch/config/URDF assets. Nothing is
## compiled, so CMake exists only to register the package with catkin and to
## install the scripts and resources into the devel/install space.

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  xacro
)

## Export src/sidewalk_bringup as an importable Python package.
##
## This is what lets every other package in the project write
##     from sidewalk_bringup.run_logger import RunLogger
## and have it work both from a source workspace and after `catkin_make install`.
##
## Without it, siblings have to reach into this package's scripts/ directory by
## absolute path — which works from source and then silently fails once
## installed, because catkin places scripts under lib/<pkg>/ and not on the
## Python path. The symptom is subtle: logging degrades to a fallback stub and
## the project-wide RUNLOG.md quietly stops being updated.
catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs tf2_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Registering the Python nodes here is what makes them runnable via `rosrun`
## and ensures the executable bit survives installation.
catkin_install_python(PROGRAMS
  scripts/hello_ros.py
  scripts/preflight_check.py
  scripts/run_logger.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Assets. Installing these means launch files resolve correctly whether the
## workspace was built with catkin_make or installed to a system location.
foreach(dir launch config urdf)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir}
    OPTIONAL)
endforeach()
