cmake_minimum_required(VERSION 3.0.2)
project(sidewalk_navigation)

## Pure Python package: launch files, parameter files, maps, goal routes and scripts.
## Nothing is compiled; CMake registers the package, installs the scripts with their
## executable bit, and places the assets where roslaunch can find them.
find_package(catkin REQUIRED COMPONENTS rospy std_msgs geometry_msgs nav_msgs sensor_msgs actionlib_msgs tf2_ros)

catkin_package(CATKIN_DEPENDS rospy std_msgs geometry_msgs nav_msgs sensor_msgs actionlib_msgs tf2_ros)

catkin_install_python(PROGRAMS
  scripts/nav_goals.py          # sends a route's goals, records the plans and the driven path
  scripts/nav_cmd_sender.py     # Jetson side of the guarded wheel-command link
  scripts/nav_cmd_recv.py       # ROBOT side of the link (runs on the robot's computer)
  scripts/score_nav.py          # scores a navigation run from its record folder
  scripts/check_goals.py        # refuses goals that fall on or next to a wall
  scripts/make_local_walls.py   # the local-costmap wall map from the saved map minus the driven outline
  scripts/thin_walls_check.py   # checks the wall map against the drive's own path
  maps/export_nav_map.py        # exports the 2D map from a saved RTAB-Map database
  sim/sim_localiser.py sim/sim_joy.py sim/safety_tests.py sim/turn_probe.py sim/grid_to_world.py sim/plot_safety.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir launch config maps goals sim doc)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir}
    OPTIONAL)
endforeach()
