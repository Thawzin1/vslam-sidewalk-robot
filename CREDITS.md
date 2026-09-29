# Credits

This project stands on other people's work. Where it is used unchanged it is credited here and not copied.

## Software this repository builds on
- **RTAB-Map** (Mathieu Labbé and François Michaud, Université de Sherbrooke) - the SLAM (simultaneous
  localisation and mapping) system at the centre of everything here, version 0.21.13 with its ROS 1 wrapper
  `rtabmap_ros`. Labbé, M., Michaud, F. (2019). RTAB-Map as an open-source lidar and visual SLAM library for
  large-scale and long-term online operation. Journal of Field Robotics, 36(2), 416-446.
  https://github.com/introlab/rtabmap
- **zed-ros-wrapper** and the **ZED SDK** (Stereolabs) - the ZED X camera driver, depth, positional tracking
  and the people detector. https://github.com/stereolabs/zed-ros-wrapper
- **robot_localization** (Tom Moore, Charles River Analytics / Locus Robotics) - the extended Kalman filter
  (a program that blends several sensors into one best position) used for the wheels + gyroscope + camera
  blend. Moore, T., Stouch, D. (2014). A Generalized Extended Kalman Filter Implementation for the Robot
  Operating System. IAS-13. https://github.com/cra-ros-pkg/robot_localization
- **move_base / DWA local planner / global_planner / costmap_2d** (the ROS navigation stack, Eitan
  Marder-Eppstein et al.). Marder-Eppstein, E., et al. (2010). The Office Marathon: Robust Navigation in an
  Indoor Office Environment. ICRA. https://github.com/ros-planning/navigation
- **Clearpath Husky packages** (`husky_description`, `husky_gazebo`, `husky_control`, Clearpath Robotics) -
  the robot model, its simulation and the real robot's base software. https://github.com/husky/husky
- **evo** (Michael Grupp) - trajectory evaluation (pinned to 1.31.1 for Python 3.8).
  https://github.com/MichaelGrupp/evo
- **Gazebo Classic 11**, **ROS Noetic**, **OpenCV 4.2**, **g2o**, **GTSAM**, **libpointmatcher**,
  **NumPy / SciPy / Matplotlib**.

## Work by colleagues (used, not included)
- **The robot's LiDAR SLAM stack, `self_navigation`** (Matt Jing, the lab colleague whose package runs on the
  robot's own computer): `rtabmap_3d.launch`, the dual-LiDAR packet recorder, the replay pipeline
  (`3dreplay_pipeline.launch`), the body filter, restamping and deskewing. It is the source of every
  "independent LiDAR estimate" in `results/`, used unchanged (one parameter, `Reg/Force3DoF=true`, added in a
  wrapper launch file for the replays). It is **not part of this repository**; it lives on the robot and in
  the lab's own repositories. Also his `mcm07_husky` and `ouster_description` configuration packages.
- **Nicolas Polga** - worked jointly with the author on the ZED X vs LiDAR camera depth study; the sphere-centroid detector in
  `catkin_ws/src/sidewalk_evaluation/scripts/depth_benchmark/sphere_centroid.py` is his code, copied
  byte-for-byte so the camera and the LiDARs were measured by the same method.

## People
- **Prof. Moein Mehrtash** (McMaster University) - supervisor.
- **Mitacs Globalink 2026** - the internship programme that funded the work.
- **Anthropic's Claude Code** - used as a programming and writing assistant throughout (see the AI-use
  statement in README.md). All experiments, checks and decisions were made by the author.
