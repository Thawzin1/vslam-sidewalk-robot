# VSLAM-Based Navigation for Autonomous Sidewalk Robots

Mitacs Globalink 2026 · McMaster University · Supervisor: Prof. Moein Mehrtash · Author: Thaw Zin

A stereo camera, a wheeled robot and an embedded computer: can the robot build a map of a building it has
never seen, find itself on that map a week later from nothing, extend the map on a later visit, and drive
to goals on it by itself? This repository holds the software, the configuration, the run procedures, the
measured results of a four-month project that answered each of those questions
on a real robot.

## What was achieved (numbers from `results/`)

| capability | run | result |
|---|---|---|
| Mapping a whole floor with a stereo camera blended with the robot's wheels and gyroscope | drive 10, `results/series2_static/s2_static_10` | about 252 m driven (length of the corrected camera map's path); the corrected map came back to within **0.03 m** of its start after **199 loop closures** (the map recognising places it had seen before) |
| Agreement with an independent LiDAR estimate of the same drive | drive 10 | **0.69 m median** (1.15 m 95th percentile), rigid alignment, no resizing |
| Localisation on the saved map from a deliberately wrong start guess | `s2_loc_04` | **0 of 575** position fixes in a wrong place (none more than 1.5 m from what the wheels said) |
| Continuing the saved map on a later visit (a SLAM session) | `s2_slam_01` | the new drive joined the old map and added **73 m²** of new floor; the old map unchanged |
| Autonomous navigation on the saved map | `s2_nav_03` | **4 of 5** goals reached on its own; one 1.5 m doorway stretch needed a straight-line command; no contact |

Every number above traces to a `RESULTS.md` in the named folder. The LiDAR figure is written as *agreement
with an independent estimate*, not as an error against ground truth: the LiDAR path is another SLAM
system's answer (range noise 15-30 mm, and the LiDAR-to-camera mounting offset was never measured).
See `results/README.md` for every run.

## Hardware

Clearpath Husky A200 (a four-wheel, skid-steer research robot with its own onboard computer that drives the
wheels), a Stereolabs ZED X stereo camera (0.120 m between its two lenses, 1920 x 1200 pixels, 15 frames per
second in practice) on a 3D-printed mount, and an NVIDIA Jetson AGX Orin that runs the camera and the SLAM.
The Jetson and camera are powered from a portable battery, independent of the robot. Two LiDARs on the robot
(Ouster and RoboSense Helios) are recorded by a colleague's software and used only as the yardstick.
Details, the power chain and the known hardware traps: `docs/HARDWARE.md`.

## Software stack (fixed versions - see `docs/HARDWARE.md` before "upgrading" anything)

| component | version |
|---|---|
| Platform | Jetson AGX Orin, L4T R35.4.1 / JetPack 5.1, Ubuntu 20.04.6, 12 cores, 29 GB |
| Middleware | ROS 1 Noetic (end of life), Python 3.8 |
| SLAM | RTAB-Map 0.21.13 (`rtabmap_ros`); back-ends g2o, GTSAM (no Ceres) |
| Sensor blend | `robot_localization` EKF (an extended Kalman filter, blending wheels + gyroscope + camera) |
| Camera driver | ZED SDK 4.x, `zed-ros-wrapper` |
| Navigation | `move_base` with the DWA local planner and `global_planner` |
| Simulator | Gazebo Classic 11.15.1 |
| Vision library | OpenCV 4.2.0 without the contrib modules (SIFT/SURF unavailable) |
| Evaluation | `evo` 1.31.1, NumPy/SciPy/Matplotlib |

## Repository map

```
run/            one launcher per job: mapping.sh, park.sh, localise.sh, slam_session.sh, navigate.sh,
                lidar_reference.sh, evaluate.sh, simulate.sh, robot_side.sh, replay.sh (stub); run/lib/ holds
                paths.sh (every folder and address), the three drive scripts and the workflow helpers
catkin_ws/src/  ROS packages: sidewalk_slam (RTAB-Map launch/config, the blend, the live map page),
                sidewalk_perception (camera), sidewalk_bringup (robot model, mount, storage page),
                sidewalk_navigation (September navigation), sidewalk_sim (worlds, routes),
                sidewalk_evaluation (trajectory metrics; the camera depth study under scripts/depth_benchmark/)
tools/          helpers: drive/ (camera guard, auto stop, bridge receiver ...), db/ (map-database readers),
                results_pack/, live_site/ (the website the Jetson serves), wifi/, robot_side/, sim/, jobs_dashboard.py
config/         a mirror of every parameter file + CONFIG_GUIDE.md (plain meaning of each setting)
docs/           OPERATIONS.md (procedures, the launchers), SOLVED.md and DO_NOT_REPEAT.md (what worked and what
                failed, with the reasons), ENGINEERING_NOTES.md (stack, traps, working rules), HARDWARE.md,
                INSTALLED.md, REPLAY.md, worksheet/ (the Information Worksheet for whoever continues the work)
results/        per run: RESULTS.md, numbers (JSON), trajectories (.tum), charts and maps (small files only)
data/           DATA_INDEX.md: index of the Teams folder that holds the raw recordings and videos;
                list_folder.py prints a local copy of it the same way (data/README.md)
```

## Ten-minute start

```bash
git clone https://github.com/Thawzin1/vslam-sidewalk-robot.git && cd vslam-sidewalk-robot
cp env.example.sh env.sh                 # fill in your folders and (if you have the robot) its addresses
source /opt/ros/noetic/setup.bash
(cd catkin_ws && catkin_make)            # needs rtabmap_ros, robot_localization, move_base, husky_* installed
source catkin_ws/devel/setup.bash
run/simulate.sh --help                   # the no-hardware path: a simulated Husky + ZED X in a Gazebo world
run/simulate.sh office2 office2_full.yaml my_first_sim
run/replay.sh --help                     # the from-recordings path (a stub until the replay build lands)
python3 tools/jobs_dashboard.py &        # the jobs page: every long job's progress line, port 8096
```

With the robot: `docs/OPERATIONS.md`, section "The run/ launchers", in this order: `robot_side.sh start`,
`mapping.sh`, `park.sh`, `robot_side.sh stop`, `lidar_reference.sh`, `evaluate.sh`. Read `docs/SOLVED.md` and
`docs/DO_NOT_REPEAT.md` before changing a parameter: most obvious changes were tried.

## Data

Raw recordings (camera `.svo2` files, RTAB-Map `.db` databases, `.bag` recordings, LiDAR recordings) and
the timelapse and fly-through videos are not in this repository. They are all in the **Autonomous Service Robot**
Microsoft Teams folder (ask Prof. Moein Mehrtash, McMaster University, for access). `data/DATA_INDEX.md` lists
every file per run, with its size and what it contains, by its path inside that folder. To replay a recording,
download it from Teams and copy it to the Jetson over the lab network (`data/README.md`, `docs/REPLAY.md`).

## How to cite

See `CITATION.cff` (GitHub shows a "Cite this repository" button from it). RTAB-Map should be cited as
Labbé and Michaud (2019), Journal of Field Robotics 36(2).

## Credits and licence

`CREDITS.md` lists the software this work builds on (RTAB-Map, the ZED ROS wrapper, robot_localization, the
ROS navigation stack, the Clearpath Husky packages, evo) and the colleagues' work it used unchanged: the
lab's LiDAR SLAM stack on the robot (the reference trajectories; not included here) and the joint camera
depth study. Code and documents in this repository are released under the MIT licence (`LICENSE`).

## Statement on the use of AI tools

Anthropic's Claude Code was used throughout as a programming and writing assistant: it helped write the
software, the analysis scripts and the documentation in this repository. The experiments, the physical
work with the robot, the checks of every result against its source files, and the decisions about what to
run and what to claim were made by the author. Where a number appears, the file it came from is named.
