# DATA_INDEX.md - every recording and result of the SLAM project, and what each file holds

**Where these files are:** the **Autonomous Service Robot** Microsoft Teams folder (a shared online folder of the lab's team). Ask Prof. Moein Mehrtash (McMaster University) for access. Every path below is relative to the top of that folder.

**Listed on 29 Sept 2026.** The robot card folder is missing 27 of 354 files (the card failed while being copied); the camera depth study's station recordings (.svo2) are added separately.

*Plain terms: this page lists, run by run, the raw files (recordings, map databases, logs) in the Teams folder, what each file contains, and which results pack (the small folder of numbers and figures in this repository) was scored from it. Nothing here is a copy of the data - it is the map of where the data is. To use a recording on the Jetson, download it from Teams on any computer and copy it to the Jetson over the lab network (see `data/README.md`).*

## How to read the file names

| file | what it is |
|---|---|
| `fusion.bag` | the Jetson's recording of every position stream (ROS bag: a timed log of messages). Topics: /rtabmap/odom /robot/wheel_odom /robot/imu /robot/imu_raw /robot/ekf_odom /robot_bridge/status /ekf_in/wheel_odom /ekf_in/imu /ekf_in/vo_odom /fused/odometry /tf /tf_static /diagnostics /zedx_front/zed_node/imu/data /rtabmap/odom_info_lite /zedx_front/zed_node/left/camera_info /zedx_front/zed_node/right/camera_info (+ /rtabmap/info and /rtabmap/localization_pose in localisation runs; + /zedx_front/zed_node/obj_det/objects in SLAM sessions with the people mask) |
| `<run>.svo2` | ZED X camera recording: stereo images + the camera's IMU. Depth is NOT stored; it is recomputed at playback in whichever depth mode you choose |
| `<run>.db` | RTAB-Map database: every map snapshot (a picture with the position it was taken from), the links between them, and - if the map was closed properly - the corrected positions (`Admin.opt_poses`) |
| `<run>_lidar.bag` | the robot's LiDAR recording, raw packets (not clouds). Topics: /helios/packets /ouster/lidar_packets /ouster/imu_packets /tf /tf_static /odometry/filtered /imu/data /husky_velocity_controller/odom /imu/data_raw /helios_pcl/scan /ouster_pcl/scan |
| `<run>_gyro_zero.json` | the robot gyroscope's standing-still bias measured before the drive |
| `media/` | live-map timelapse frames (and the mp4) recorded during the run |
| `lidar/` (under series2_raw/<run>/) | the robot's LiDAR files once moved off the card: bag, gyro file, replay outputs |

## Top-level folders

| folder | holds |
|---|---|
| `series2_raw/` | Jetson run records of the September walkway series: <run>/<run>.db (the RTAB-Map database), <run>/<run>/ (the run folder: fusion.bag, .svo2 camera recording, logs, media/ timelapse frames), <run>/lidar/ (the robot's LiDAR bag once moved). |
| `robot_card_slam_series2/` | The robot's own 128 GB card, folder slam_series2: every <run>_lidar.bag, <run>_gyro_zero.json, replay outputs (<run>/ folders, *_replay.out) and our robot-side tools (tools/). 327 of the card's 354 files (107.1 GiB); the other 27 were lost when the card failed while being copied. |
| `paper_zedx_vs_lidar/` | The camera depth study data: how precisely the ZED X camera measures distance, checked against LiDAR (laser scanner) measurements, station by station. CSV results, logs and the study's tools. |
| `series1_august_lab/` | August lab-room work: logs_real_indoor, jetson_run_records_rescue, Week_7, jetson_sidewalk128/{rtabmap_maps,archive} (the old card's databases). |
| `slam_research_01_runs/` | The results packs (numbers, figures, timelapse videos, 3D clouds) - the same folders as results/<series>/<run>/ in the repository, with the large files. |
| `freeze_bench_2026-09-26/` | Parked camera-freeze benches of 26 Sept (timing.bag per condition): the evidence that freezes grow with Jetson uptime and that GEN_1 freezes more. |

## series2_raw/

*Jetson run records of the September walkway series: <run>/<run>.db (the RTAB-Map database), <run>/<run>/ (the run folder: fusion.bag, .svo2 camera recording, logs, media/ timelapse frames), <run>/lidar/ (the robot's LiDAR bag once moved).*

20903 files, 124.09 GB in all.

### s2_fusion_T4b  (23 files, 196.33 MB)

| | |
|---|---|
| date (Hamilton time) | 25 Sep 2026, 03:58-04:14 |
| what | **parked rehearsal** - robot on one spot, turns in place; lens covered; forced WiFi cut |
| why it was driven | prove the wheel+gyroscope+camera blend before drive 3 |
| scored in | `results/series2_static/s2_fusion_T4b/RESULTS.md` |

| file | size | what it contains |
|---|---|---|
| `s2_fusion_T4b/s2_fusion_T4b/fusion.bag` | 193.86 MB | the Jetson's recording of every position stream during the run (ROS bag). Topics: /rtabmap/odom /robot/wheel_odom /robot/imu /robot/imu_raw /robot/ekf_odom /robot_bridge/status /ekf_in/wheel_odom /ekf_in/imu /ekf_in/vo_odom /fused/odometry /tf /tf_static /diagnostics /zedx_front/zed_node/imu/data /rtabmap/odom_info_lite /zedx_front/zed_node/left/camera_info /zedx_front/zed_node/right/camera_info (+ /rtabmap/info and /rtabmap/localization_pose in localisation runs; + /zedx_front/zed_node/obj_det/objects in SLAM sessions with the people mask) |
| `s2_fusion_T4b/s2_fusion_T4b/mapping.log` | 2.26 MB | RTAB-Map's own log for the run (loop closures accepted/rejected, saves, warnings) |
| `s2_fusion_T4b/s2_fusion_T4b/monitor.csv` | 35 KB | the live mapping monitor's per-second counters and its last status line |
| `s2_fusion_T4b/s2_fusion_T4b/fused_odometry.log` | 9 KB | the blend (EKF) and its input conditioner's log, incl. the kept gyroscope offset |
| `s2_fusion_T4b/s2_fusion_T4b/autostop.log` | 6 KB | the stop-on-'park' helper's log |
| `s2_fusion_T4b/s2_fusion_T4b/bridge_recv.log` | 4 KB | the robot-to-Jetson link receiver's log (delivered share of the robot's readings) |
| `s2_fusion_T4b/s2_fusion_T4b/forced_drop.sh` | 2 KB | one of our tools/scripts (copy kept with the data) |
| `s2_fusion_T4b/s2_fusion_T4b/monitor_STATUS.txt` | 134 B | the live mapping monitor's per-second counters and its last status line |
| + 15 small record files (logs, json, csv, txt, pids) | 162 KB | run records: logs, checks, progress lines |

### s2_fusion_T5b  (998 files, 8.36 GB)

| | |
|---|---|
| date (Hamilton time) | 25-26 Sep 2026, ~23:49-00:07 |
| what | **parked rehearsal** - T5b: link drop A1/A2, ZED gyroscope in the blend; 16.5 min |
| why it was driven | rehearse drive 4; found 10 camera freezes in 16.5 min |
| scored in | `- (see PROGRESS.html 26 Sep 00:54)` |

| file | size | what it contains |
|---|---|---|
| `s2_fusion_T5b/s2_fusion_T5b.db` | 3.26 GB | RTAB-Map database: every map snapshot (picture + position), the links between them, and the corrected positions if closed properly |
| `s2_fusion_T5b/s2_fusion_T5b/fusion.bag` | 2.67 GB | the Jetson's recording of every position stream during the run (ROS bag). Topics: /rtabmap/odom /robot/wheel_odom /robot/imu /robot/imu_raw /robot/ekf_odom /robot_bridge/status /ekf_in/wheel_odom /ekf_in/imu /ekf_in/vo_odom /fused/odometry /tf /tf_static /diagnostics /zedx_front/zed_node/imu/data /rtabmap/odom_info_lite /zedx_front/zed_node/left/camera_info /zedx_front/zed_node/right/camera_info (+ /rtabmap/info and /rtabmap/localization_pose in localisation runs; + /zedx_front/zed_node/obj_det/objects in SLAM sessions with the people mask) |
| `s2_fusion_T5b/s2_fusion_T5b_b/s2_fusion_T5b_b.svo2` | 1.95 GB | ZED X camera recording (stereo images + camera IMU); depth can be recomputed offline in any depth mode |
| `s2_fusion_T5b/s2_fusion_T5b/s2_fusion_T5b.svo2` | 444.86 MB | ZED X camera recording (stereo images + camera IMU); depth can be recomputed offline in any depth mode |
| `s2_fusion_T5b/s2_fusion_T5b/mapping.log` | 1.52 MB | RTAB-Map's own log for the run (loop closures accepted/rejected, saves, warnings) |
| `s2_fusion_T5b/s2_fusion_T5b/media/s2_fusion_T5b_timelapse.mp4` | 655 KB | timelapse video of the live map page, recorded during the run |
| `s2_fusion_T5b/s2_fusion_T5b/monitor.csv` | 36 KB | the live mapping monitor's per-second counters and its last status line |
| `s2_fusion_T5b/s2_fusion_T5b/camera.log` | 24 KB | the ZED camera program's log |
| `s2_fusion_T5b/s2_fusion_T5b/fused_odometry.log` | 8 KB | the blend (EKF) and its input conditioner's log, incl. the kept gyroscope offset |
| `s2_fusion_T5b/s2_fusion_T5b/autostop.log` | 7 KB | the stop-on-'park' helper's log |
| `s2_fusion_T5b/s2_fusion_T5b/bridge_recv.log` | 2 KB | the robot-to-Jetson link receiver's log (delivered share of the robot's readings) |
| `s2_fusion_T5b/s2_fusion_T5b/monitor_STATUS.txt` | 137 B | the live mapping monitor's per-second counters and its last status line |
| `.../media/` 972 timelapse frames (png) + index/log | 49.94 MB | the live map page, one picture every 3 s, recorded during the run (the mp4 above is made from them) |
| + 14 small record files (logs, json, csv, txt, pids) | 9 KB | run records: logs, checks, progress lines |

### s2_fusion_T5c  (825 files, 3.03 GB)

| | |
|---|---|
| date (Hamilton time) | 26 Sep 2026, ~00:26-00:41 |
| what | **parked rehearsal** - T5c: 16-bit depth in the map; 9.7 min |
| why it was driven | does 16-bit depth cut the freezes? (2 in 9.7 min); camera crashed while recording |
| scored in | `- (docs/SOLVED.md '16-bit depth cuts the camera freezes')` |

| file | size | what it contains |
|---|---|---|
| `s2_fusion_T5c/s2_fusion_T5c/s2_fusion_T5c.svo2` | 1.61 GB | ZED X camera recording (stereo images + camera IMU); depth can be recomputed offline in any depth mode |
| `s2_fusion_T5c/s2_fusion_T5c.db` | 924.75 MB | RTAB-Map database: every map snapshot (picture + position), the links between them, and the corrected positions if closed properly |
| `s2_fusion_T5c/s2_fusion_T5c/fusion.bag` | 470.30 MB | the Jetson's recording of every position stream during the run (ROS bag). Topics: /rtabmap/odom /robot/wheel_odom /robot/imu /robot/imu_raw /robot/ekf_odom /robot_bridge/status /ekf_in/wheel_odom /ekf_in/imu /ekf_in/vo_odom /fused/odometry /tf /tf_static /diagnostics /zedx_front/zed_node/imu/data /rtabmap/odom_info_lite /zedx_front/zed_node/left/camera_info /zedx_front/zed_node/right/camera_info (+ /rtabmap/info and /rtabmap/localization_pose in localisation runs; + /zedx_front/zed_node/obj_det/objects in SLAM sessions with the people mask) |
| `s2_fusion_T5c/s2_fusion_T5c/mapping.log` | 1.56 MB | RTAB-Map's own log for the run (loop closures accepted/rejected, saves, warnings) |
| `s2_fusion_T5c/s2_fusion_T5c/media/s2_fusion_T5c_timelapse.mp4` | 937 KB | timelapse video of the live map page, recorded during the run |
| `s2_fusion_T5c/s2_fusion_T5c/monitor.csv` | 32 KB | the live mapping monitor's per-second counters and its last status line |
| `s2_fusion_T5c/s2_fusion_T5c/autostop.log` | 6 KB | the stop-on-'park' helper's log |
| `s2_fusion_T5c/s2_fusion_T5c/fused_odometry.log` | 5 KB | the blend (EKF) and its input conditioner's log, incl. the kept gyroscope offset |
| `s2_fusion_T5c/s2_fusion_T5c/bridge_recv.log` | 3 KB | the robot-to-Jetson link receiver's log (delivered share of the robot's readings) |
| `s2_fusion_T5c/s2_fusion_T5c/monitor_STATUS.txt` | 138 B | the live mapping monitor's per-second counters and its last status line |
| `.../media/` 805 timelapse frames (png) + index/log | 60.09 MB | the live map page, one picture every 3 s, recorded during the run (the mp4 above is made from them) |
| + 10 small record files (logs, json, csv, txt, pids) | 9 KB | run records: logs, checks, progress lines |

### s2_loc_01  (514 files, 1.30 GB)

| | |
|---|---|
| date (Hamilton time) | 26 Sep 2026, 05:03-05:10 |
| what | **localisation session 1** - drive 4's map, mapping off; lost at stop 1 |
| why it was driven | first localisation attempt; robot battery not charged enough |
| scored in | `- (03_methods/localisation_demo_2026-09-26/RESULTS.md, session 1)` |

| file | size | what it contains |
|---|---|---|
| `s2_loc_01/s2_loc_01/fusion.bag` | 1.27 GB | the Jetson's recording of every position stream during the run (ROS bag). Topics: /rtabmap/odom /robot/wheel_odom /robot/imu /robot/imu_raw /robot/ekf_odom /robot_bridge/status /ekf_in/wheel_odom /ekf_in/imu /ekf_in/vo_odom /fused/odometry /tf /tf_static /diagnostics /zedx_front/zed_node/imu/data /rtabmap/odom_info_lite /zedx_front/zed_node/left/camera_info /zedx_front/zed_node/right/camera_info (+ /rtabmap/info and /rtabmap/localization_pose in localisation runs; + /zedx_front/zed_node/obj_det/objects in SLAM sessions with the people mask) |
| `s2_loc_01/s2_loc_01/mapping.log` | 455 KB | RTAB-Map's own log for the run (loop closures accepted/rejected, saves, warnings) |
| `s2_loc_01/s2_loc_01/media/s2_loc_01_timelapse.mp4` | 140 KB | timelapse video of the live map page, recorded during the run |
| `s2_loc_01/s2_loc_01/camera.log` | 21 KB | the ZED camera program's log |
| `s2_loc_01/s2_loc_01/monitor.csv` | 16 KB | the live mapping monitor's per-second counters and its last status line |
| `s2_loc_01/s2_loc_01/localise_track.csv` | 9 KB | localisation watcher outputs: fixes, events and summary |
| `s2_loc_01/s2_loc_01/fused_odometry.log` | 6 KB | the blend (EKF) and its input conditioner's log, incl. the kept gyroscope offset |
| `s2_loc_01/s2_loc_01/bridge_recv.log` | 2 KB | the robot-to-Jetson link receiver's log (delivered share of the robot's readings) |
| `s2_loc_01/s2_loc_01/camera_guard.log` | 573 B | the camera guard: restarts, outages (start, end, seconds, cause, outcome) |
| `s2_loc_01/s2_loc_01/camera_guard.out` | 573 B | the camera guard: restarts, outages (start, end, seconds, cause, outcome) |
| `s2_loc_01/s2_loc_01/localise_summary.json` | 540 B | localisation watcher outputs: fixes, events and summary |
| `s2_loc_01/s2_loc_01/camera_guard.json` | 285 B | the camera guard: restarts, outages (start, end, seconds, cause, outcome) |
| `s2_loc_01/s2_loc_01/localise_events.csv` | 170 B | localisation watcher outputs: fixes, events and summary |
| `s2_loc_01/s2_loc_01/monitor_STATUS.txt` | 132 B | the live mapping monitor's per-second counters and its last status line |
| `.../media/` 488 timelapse frames (png) + index/log | 28.33 MB | the live map page, one picture every 3 s, recorded during the run (the mp4 above is made from them) |
| + 12 small record files (logs, json, csv, txt, pids) | 10 KB | run records: logs, checks, progress lines |

### s2_loc_02  (1202 files, 8.98 GB)

| | |
|---|---|
| date (Hamilton time) | 26 Sep 2026, 06:16-06:34 |
| what | **localisation session 2** - drive 4's map; 5 starts from the saved start guess |
| why it was driven | does the robot find itself on a saved map? (1 of 5; 4 shown 4-10 m wrong) |
| scored in | `results/series2_static/s2_loc_02/RESULTS.md` |

| file | size | what it contains |
|---|---|---|
| `s2_loc_02/raw_from_robot/s2_loc_02_lidar.bag` | 5.59 GB | the robot's LiDAR recording: raw laser packets, not expanded clouds. Topics: /helios/packets /ouster/lidar_packets /ouster/imu_packets /tf /tf_static /odometry/filtered /imu/data /husky_velocity_controller/odom /imu/data_raw /helios_pcl/scan /ouster_pcl/scan |
| `s2_loc_02/s2_loc_02/fusion.bag` | 2.71 GB | the Jetson's recording of every position stream during the run (ROS bag). Topics: /rtabmap/odom /robot/wheel_odom /robot/imu /robot/imu_raw /robot/ekf_odom /robot_bridge/status /ekf_in/wheel_odom /ekf_in/imu /ekf_in/vo_odom /fused/odometry /tf /tf_static /diagnostics /zedx_front/zed_node/imu/data /rtabmap/odom_info_lite /zedx_front/zed_node/left/camera_info /zedx_front/zed_node/right/camera_info (+ /rtabmap/info and /rtabmap/localization_pose in localisation runs; + /zedx_front/zed_node/obj_det/objects in SLAM sessions with the people mask) |
| `s2_loc_02/lidar/s2_loc_02/rtab_helios.db` | 220.40 MB | the LiDAR map database written by the colleague's replay pipeline (RTAB-Map, LiDAR mode) |
| `s2_loc_02/lidar/s2_loc_02.partial-20260926/rtab_helios.db` | 209.46 MB | the LiDAR map database written by the colleague's replay pipeline (RTAB-Map, LiDAR mode) |
| `s2_loc_02/lidar/s2_loc_02/play.log` | 85.27 MB | a small record file (numbers, log or provenance) |
| `s2_loc_02/lidar/s2_loc_02.partial-20260926/play.log` | 84.21 MB | a small record file (numbers, log or provenance) |
| `s2_loc_02/lidar/s2_loc_02/wheel.tum` | 5.53 MB | the robot's own wheel+gyroscope path (/odometry/filtered) in TUM format |
| `s2_loc_02/s2_loc_02/mapping.log` | 1.71 MB | RTAB-Map's own log for the run (loop closures accepted/rejected, saves, warnings) |
| `s2_loc_02/s2_loc_02/media/s2_loc_02_timelapse.mp4` | 543 KB | timelapse video of the live map page, recorded during the run |
| `s2_loc_02/lidar/s2_loc_02/lidar_map.png` | 68 KB | the LiDAR map's walls (cells 0.15-2.0 m above the base) as a picture / array |
| `s2_loc_02/s2_loc_02/monitor.csv` | 41 KB | the live mapping monitor's per-second counters and its last status line |
| `s2_loc_02/s2_loc_02/localise_track.csv` | 27 KB | localisation watcher outputs: fixes, events and summary |
| `s2_loc_02/lidar/s2_loc_02/lidar.tum` | 22 KB | the LiDAR estimate's corrected path (TUM format: time x y z qx qy qz qw), one line per map node |
| `s2_loc_02/s2_loc_02/camera.log` | 21 KB | the ZED camera program's log |
| `s2_loc_02/lidar/s2_loc_02/lidar_map.npz` | 17 KB | the LiDAR map's walls (cells 0.15-2.0 m above the base) as a picture / array |
| `s2_loc_02/s2_loc_02/fused_odometry.log` | 10 KB | the blend (EKF) and its input conditioner's log, incl. the kept gyroscope offset |
| `s2_loc_02/s2_loc_02/localise_events.csv` | 6 KB | localisation watcher outputs: fixes, events and summary |
| `s2_loc_02/s2_loc_02/bridge_recv.log` | 3 KB | the robot-to-Jetson link receiver's log (delivered share of the robot's readings) |
| `s2_loc_02/s2_loc_02/localise_summary.json` | 3 KB | localisation watcher outputs: fixes, events and summary |
| `s2_loc_02/s2_loc_02/camera_guard.log` | 573 B | the camera guard: restarts, outages (start, end, seconds, cause, outcome) |
| `s2_loc_02/s2_loc_02/camera_guard.out` | 573 B | the camera guard: restarts, outages (start, end, seconds, cause, outcome) |
| `s2_loc_02/s2_loc_02/camera_guard.json` | 284 B | the camera guard: restarts, outages (start, end, seconds, cause, outcome) |
| `s2_loc_02/lidar/s2_loc_02_gyro_zero.json` | 174 B | the robot gyroscope's standing-still bias measured before the drive |
| `s2_loc_02/s2_loc_02/monitor_STATUS.txt` | 135 B | the live mapping monitor's per-second counters and its last status line |
| `.../media/` 1157 timelapse frames (png) + index/log | 87.90 MB | the live map page, one picture every 3 s, recorded during the run (the mp4 above is made from them) |
| + 21 small record files (logs, json, csv, txt, pids) | 1.13 MB | run records: logs, checks, progress lines |

### s2_loc_03  (1460 files, 3.47 GB)

| | |
|---|---|
| date (Hamilton time) | 26 Sep 2026, 10:41-11:06 |
| what | **localisation session 3** - drive 4's map; 5 starts, each told it is 40 m off the map |
| why it was driven | off-map start guess + LoopThr 0.08: finds itself at 4 of 5 starts, never shows a wrong place |
| scored in | `results/series2_static/s2_loc_03/RESULTS.md` |

| file | size | what it contains |
|---|---|---|
| `s2_loc_03/s2_loc_03/fusion.bag` | 2.98 GB | the Jetson's recording of every position stream during the run (ROS bag). Topics: /rtabmap/odom /robot/wheel_odom /robot/imu /robot/imu_raw /robot/ekf_odom /robot_bridge/status /ekf_in/wheel_odom /ekf_in/imu /ekf_in/vo_odom /fused/odometry /tf /tf_static /diagnostics /zedx_front/zed_node/imu/data /rtabmap/odom_info_lite /zedx_front/zed_node/left/camera_info /zedx_front/zed_node/right/camera_info (+ /rtabmap/info and /rtabmap/localization_pose in localisation runs; + /zedx_front/zed_node/obj_det/objects in SLAM sessions with the people mask) |
| `s2_loc_03/lidar/s2_loc_03/rtab_helios.db` | 220.29 MB | the LiDAR map database written by the colleague's replay pipeline (RTAB-Map, LiDAR mode) |
| `s2_loc_03/lidar/s2_loc_03/play.log` | 150.43 MB | a small record file (numbers, log or provenance) |
| `s2_loc_03/lidar/s2_loc_03/wheel.tum` | 8.34 MB | the robot's own wheel+gyroscope path (/odometry/filtered) in TUM format |
| `s2_loc_03/s2_loc_03/mapping.log` | 2.56 MB | RTAB-Map's own log for the run (loop closures accepted/rejected, saves, warnings) |
| `s2_loc_03/s2_loc_03/media/s2_loc_03_timelapse.mp4` | 762 KB | timelapse video of the live map page, recorded during the run |
| `s2_loc_03/lidar/s2_loc_03/lidar_map.png` | 70 KB | the LiDAR map's walls (cells 0.15-2.0 m above the base) as a picture / array |
| `s2_loc_03/s2_loc_03/monitor.csv` | 47 KB | the live mapping monitor's per-second counters and its last status line |
| `s2_loc_03/s2_loc_03/localise_track.csv` | 40 KB | localisation watcher outputs: fixes, events and summary |
| `s2_loc_03/lidar/s2_loc_03/lidar.tum` | 22 KB | the LiDAR estimate's corrected path (TUM format: time x y z qx qy qz qw), one line per map node |
| `s2_loc_03/s2_loc_03/camera.log` | 21 KB | the ZED camera program's log |
| `s2_loc_03/lidar/s2_loc_03/lidar_map.npz` | 20 KB | the LiDAR map's walls (cells 0.15-2.0 m above the base) as a picture / array |
| `s2_loc_03/s2_loc_03/fused_odometry.log` | 15 KB | the blend (EKF) and its input conditioner's log, incl. the kept gyroscope offset |
| `s2_loc_03/s2_loc_03/localise_events.csv` | 11 KB | localisation watcher outputs: fixes, events and summary |
| `s2_loc_03/s2_loc_03/localise_summary.json` | 3 KB | localisation watcher outputs: fixes, events and summary |
| `s2_loc_03/s2_loc_03/bridge_recv.log` | 2 KB | the robot-to-Jetson link receiver's log (delivered share of the robot's readings) |
| `s2_loc_03/s2_loc_03/camera_guard.log` | 573 B | the camera guard: restarts, outages (start, end, seconds, cause, outcome) |
| `s2_loc_03/s2_loc_03/camera_guard.out` | 573 B | the camera guard: restarts, outages (start, end, seconds, cause, outcome) |
| `s2_loc_03/s2_loc_03/camera_guard.json` | 285 B | the camera guard: restarts, outages (start, end, seconds, cause, outcome) |
| `s2_loc_03/lidar/s2_loc_03_gyro_zero.json` | 174 B | the robot gyroscope's standing-still bias measured before the drive |
| `s2_loc_03/s2_loc_03/monitor_STATUS.txt` | 135 B | the live mapping monitor's per-second counters and its last status line |
| `.../media/` 1421 timelapse frames (png) + index/log | 118.29 MB | the live map page, one picture every 3 s, recorded during the run (the mp4 above is made from them) |
| + 18 small record files (logs, json, csv, txt, pids) | 437 KB | run records: logs, checks, progress lines |

### s2_loc_04  (1751 files, 13.36 GB)

| | |
|---|---|
| date (Hamilton time) | 27 Sep 2026, 07:43-08:08 |
| what | **whole-floor localisation** - drive 10's map, mapping off, told it starts 40 m outside the building; the floor driven once |
| why it was driven | pass lines W1-W4: 0 of 575 fixes wrong; first fix 212 s |
| scored in | `results/series2_static/s2_loc_04/RESULTS.md` |

| file | size | what it contains |
|---|---|---|
| `s2_loc_04/raw_from_robot/s2_loc_04_lidar.bag` | 9.25 GB | the robot's LiDAR recording: raw laser packets, not expanded clouds. Topics: /helios/packets /ouster/lidar_packets /ouster/imu_packets /tf /tf_static /odometry/filtered /imu/data /husky_velocity_controller/odom /imu/data_raw /helios_pcl/scan /ouster_pcl/scan |
| `s2_loc_04/s2_loc_04/fusion.bag` | 2.91 GB | the Jetson's recording of every position stream during the run (ROS bag). Topics: /rtabmap/odom /robot/wheel_odom /robot/imu /robot/imu_raw /robot/ekf_odom /robot_bridge/status /ekf_in/wheel_odom /ekf_in/imu /ekf_in/vo_odom /fused/odometry /tf /tf_static /diagnostics /zedx_front/zed_node/imu/data /rtabmap/odom_info_lite /zedx_front/zed_node/left/camera_info /zedx_front/zed_node/right/camera_info (+ /rtabmap/info and /rtabmap/localization_pose in localisation runs; + /zedx_front/zed_node/obj_det/objects in SLAM sessions with the people mask) |
| `s2_loc_04/lidar_replay_jetson/done/lidar_ref_f3dof/rtab_helios.db` | 749.39 MB | the LiDAR map database written by the colleague's replay pipeline (RTAB-Map, LiDAR mode) |
| `s2_loc_04/lidar_replay_jetson/done/lidar_ref_f3dof/play.log` | 175.07 MB | a small record file (numbers, log or provenance) |
| `s2_loc_04/lidar_replay_jetson/done/lidar_ref_f3dof/wheel.tum` | 9.09 MB | the robot's own wheel+gyroscope path (/odometry/filtered) in TUM format |
| `s2_loc_04/s2_loc_04/mapping.log` | 3.37 MB | RTAB-Map's own log for the run (loop closures accepted/rejected, saves, warnings) |
| `s2_loc_04/s2_loc_04/media/s2_loc_04_timelapse.mp4` | 1.48 MB | timelapse video of the live map page, recorded during the run |
| `s2_loc_04/s2_loc_04/tegrastats.log` | 834 KB | NVIDIA's once-a-second processor/GPU/memory/power log of the Jetson |
| `s2_loc_04/lidar_replay_jetson/done/lidar_ref_f3dof/lidar_map.png` | 166 KB | the LiDAR map's walls (cells 0.15-2.0 m above the base) as a picture / array |
| `s2_loc_04/lidar_replay_jetson/done/lidar_ref_f3dof/lidar.tum` | 75 KB | the LiDAR estimate's corrected path (TUM format: time x y z qx qy qz qw), one line per map node |
| `s2_loc_04/s2_loc_04/localise_events.csv` | 71 KB | localisation watcher outputs: fixes, events and summary |
| `s2_loc_04/s2_loc_04/monitor.csv` | 68 KB | the live mapping monitor's per-second counters and its last status line |
| `s2_loc_04/lidar_replay_jetson/done/lidar_ref_f3dof/lidar_map.npz` | 61 KB | the LiDAR map's walls (cells 0.15-2.0 m above the base) as a picture / array |
| `s2_loc_04/s2_loc_04/localise_track.csv` | 53 KB | localisation watcher outputs: fixes, events and summary |
| `s2_loc_04/s2_loc_04/camera.log` | 21 KB | the ZED camera program's log |
| `s2_loc_04/s2_loc_04/fused_odometry.log` | 13 KB | the blend (EKF) and its input conditioner's log, incl. the kept gyroscope offset |
| `s2_loc_04/s2_loc_04/bridge_recv.log` | 10 KB | the robot-to-Jetson link receiver's log (delivered share of the robot's readings) |
| `s2_loc_04/s2_loc_04/localise_summary.json` | 1 KB | localisation watcher outputs: fixes, events and summary |
| `s2_loc_04/s2_loc_04/camera_guard.out` | 598 B | the camera guard: restarts, outages (start, end, seconds, cause, outcome) |
| `s2_loc_04/s2_loc_04/camera_guard.log` | 598 B | the camera guard: restarts, outages (start, end, seconds, cause, outcome) |
| `s2_loc_04/s2_loc_04/camera_guard.json` | 283 B | the camera guard: restarts, outages (start, end, seconds, cause, outcome) |
| `s2_loc_04/raw_from_robot/s2_loc_04_gyro_zero.json` | 174 B | the robot gyroscope's standing-still bias measured before the drive |
| `s2_loc_04/s2_loc_04/monitor_STATUS.txt` | 140 B | the live mapping monitor's per-second counters and its last status line |
| `.../media/` 1708 timelapse frames (png) + index/log | 286.49 MB | the live map page, one picture every 3 s, recorded during the run (the mp4 above is made from them) |
| + 20 small record files (logs, json, csv, txt, pids) | 500 KB | run records: logs, checks, progress lines |

### s2_nav_01  (1772 files, 3.04 GB)

| | |
|---|---|
| date (Hamilton time) | 27 Sep 2026, ~16:14-16:46 |
| what | **navigation run 1** - route A (corridor out and back) 2 of 2 goals; route B stopped after a wall-post touch |
| why it was driven | first autonomous driving on drive 10's map |
| scored in | `results/navigation/s2_nav_01/ (nav_score.json, USER_NOTES.txt)` |

| file | size | what it contains |
|---|---|---|
| `s2_nav_01/fusion.bag` | 2.79 GB | the Jetson's recording of every position stream during the run (ROS bag). Topics: /rtabmap/odom /robot/wheel_odom /robot/imu /robot/imu_raw /robot/ekf_odom /robot_bridge/status /ekf_in/wheel_odom /ekf_in/imu /ekf_in/vo_odom /fused/odometry /tf /tf_static /diagnostics /zedx_front/zed_node/imu/data /rtabmap/odom_info_lite /zedx_front/zed_node/left/camera_info /zedx_front/zed_node/right/camera_info (+ /rtabmap/info and /rtabmap/localization_pose in localisation runs; + /zedx_front/zed_node/obj_det/objects in SLAM sessions with the people mask) |
| `s2_nav_01/mapping.log` | 2.31 MB | RTAB-Map's own log for the run (loop closures accepted/rejected, saves, warnings) |
| `s2_nav_01/nav_B/plans.jsonl` | 2.30 MB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_01/nav_B_part2/user_photo_rear_at_corner.jpg` | 1.39 MB | a figure or photo |
| `s2_nav_01/nav_B_part2/user_photo_overview_at_corner.jpg` | 1.39 MB | a figure or photo |
| `s2_nav_01/nav_A_try1_range4m/plans.jsonl` | 1.20 MB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_01/nav_B_try1_r2_not_held/plans.jsonl` | 1.15 MB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_01/media/s2_nav_01_timelapse.mp4` | 1.14 MB | timelapse video of the live map page, recorded during the run |
| `s2_nav_01/tegrastats.log` | 807 KB | NVIDIA's once-a-second processor/GPU/memory/power log of the Jetson |
| `s2_nav_01/nav_B_part2/plans.jsonl` | 529 KB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_01/camera.log` | 413 KB | the ZED camera program's log |
| `s2_nav_01/nav_A/plans.jsonl` | 262 KB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_01/nav_B/cmd.csv` | 119 KB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_01/nav_A_try1_range4m/cmd.csv` | 103 KB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_01/nav_launch.log` | 95 KB | navigation launch logs and the move_base settings actually loaded |
| `s2_nav_01/nav_B/robot_status.jsonl` | 91 KB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_01/nav_A_try1_range4m/robot_status.jsonl` | 84 KB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_01/nav_B_try1_r2_not_held/cmd.csv` | 68 KB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_01/nav_A/cmd.csv` | 68 KB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_01/monitor.csv` | 66 KB | the live mapping monitor's per-second counters and its last status line |
| `s2_nav_01/nav_B/odom.csv` | 66 KB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_01/nav_B/actual.csv` | 66 KB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_01/nav_A_try1_range4m/actual.csv` | 57 KB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_01/nav_A_try1_range4m/odom.csv` | 57 KB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_01/nav_B_try1_r2_not_held/robot_status.jsonl` | 55 KB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_01/nav_A/robot_status.jsonl` | 50 KB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_01/localise_track.csv` | 50 KB | localisation watcher outputs: fixes, events and summary |
| `s2_nav_01/nav_B_try1_r2_not_held/actual.csv` | 36 KB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_01/nav_B_try1_r2_not_held/odom.csv` | 36 KB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_01/nav_A/odom.csv` | 35 KB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_01/nav_A/actual.csv` | 35 KB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_01/nav_B_part2/cmd.csv` | 29 KB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_01/localise_events.csv` | 29 KB | localisation watcher outputs: fixes, events and summary |
| `s2_nav_01/nav_B_part2/robot_status.jsonl` | 24 KB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_01/nav_B/costmap_frames/frame_008.png` | 18 KB | a figure or photo |
| `s2_nav_01/nav_B/costmap_frames/frame_006.png` | 18 KB | a figure or photo |
| `s2_nav_01/nav_B/costmap_frames/frame_025.png` | 18 KB | a figure or photo |
| `s2_nav_01/nav_B/costmap_frames/frame_022.png` | 18 KB | a figure or photo |
| `s2_nav_01/nav_B/costmap_frames/frame_023.png` | 17 KB | a figure or photo |
| `s2_nav_01/nav_B/costmap_frames/frame_005.png` | 17 KB | a figure or photo |
| ... 108 more files of this kind | | |
| `.../media/` 1614 timelapse frames (png) + index/log | 245.65 MB | the live map page, one picture every 3 s, recorded during the run (the mp4 above is made from them) |
| + 10 small record files (logs, json, csv, txt, pids) | 28 KB | run records: logs, checks, progress lines |

### s2_nav_02  (1061 files, 1.77 GB)

| | |
|---|---|
| date (Hamilton time) | 27 Sep 2026, ~17:31-17:53 |
| what | **navigation run 2** - route B2 around the loop: 4 of 5 goals, no contact |
| why it was driven | with the local walls + clearance guard fixes after run 1 |
| scored in | `results/navigation/s2_nav_02/ (goals_result.json)` |

| file | size | what it contains |
|---|---|---|
| `s2_nav_02/fusion.bag` | 1.62 GB | the Jetson's recording of every position stream during the run (ROS bag). Topics: /rtabmap/odom /robot/wheel_odom /robot/imu /robot/imu_raw /robot/ekf_odom /robot_bridge/status /ekf_in/wheel_odom /ekf_in/imu /ekf_in/vo_odom /fused/odometry /tf /tf_static /diagnostics /zedx_front/zed_node/imu/data /rtabmap/odom_info_lite /zedx_front/zed_node/left/camera_info /zedx_front/zed_node/right/camera_info (+ /rtabmap/info and /rtabmap/localization_pose in localisation runs; + /zedx_front/zed_node/obj_det/objects in SLAM sessions with the people mask) |
| `s2_nav_02/nav_B2/plans.jsonl` | 2.80 MB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_02/mapping.log` | 1.73 MB | RTAB-Map's own log for the run (loop closures accepted/rejected, saves, warnings) |
| `s2_nav_02/nav_B2_part2/plans.jsonl` | 1.29 MB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_02/media/s2_nav_02_timelapse.mp4` | 896 KB | timelapse video of the live map page, recorded during the run |
| `s2_nav_02/nav_B2_part3/plans.jsonl` | 570 KB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_02/tegrastats.log` | 514 KB | NVIDIA's once-a-second processor/GPU/memory/power log of the Jetson |
| `s2_nav_02/nav_B2/cmd.csv` | 173 KB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_02/nav_B2/robot_status.jsonl` | 131 KB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_02/nav_B2/actual.csv` | 95 KB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_02/nav_B2/odom.csv` | 93 KB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_02/nav_B2_part2/cmd.csv` | 78 KB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_02/nav_B2_part2/robot_status.jsonl` | 60 KB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_02/nav_launch.log` | 51 KB | navigation launch logs and the move_base settings actually loaded |
| `s2_nav_02/nav_B2_part2/odom.csv` | 43 KB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_02/nav_B2_part2/actual.csv` | 43 KB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_02/monitor.csv` | 41 KB | the live mapping monitor's per-second counters and its last status line |
| `s2_nav_02/localise_track.csv` | 31 KB | localisation watcher outputs: fixes, events and summary |
| `s2_nav_02/localise_events.csv` | 22 KB | localisation watcher outputs: fixes, events and summary |
| `s2_nav_02/camera.log` | 21 KB | the ZED camera program's log |
| `s2_nav_02/nav_B2_part3/cmd.csv` | 17 KB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_02/nav_B2_part3/robot_status.jsonl` | 13 KB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_02/nav_B2_part3/actual.csv` | 9 KB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_02/nav_B2_part3/odom.csv` | 9 KB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_02/fused_odometry.log` | 7 KB | the blend (EKF) and its input conditioner's log, incl. the kept gyroscope offset |
| `s2_nav_02/nav_params_move_base.yaml` | 6 KB | navigation launch logs and the move_base settings actually loaded |
| `s2_nav_02/nav_dynparam_readback.txt` | 4 KB | navigation launch logs and the move_base settings actually loaded |
| `s2_nav_02/navcmd_sender.log` | 3 KB | navigation launch logs and the move_base settings actually loaded |
| `s2_nav_02/bridge_recv.log` | 2 KB | the robot-to-Jetson link receiver's log (delivered share of the robot's readings) |
| `s2_nav_02/fusion_bag.log` | 1 KB | navigation launch logs and the move_base settings actually loaded |
| `s2_nav_02/nav_B2/fixes.csv` | 1 KB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_02/tf_check_2.txt` | 1 KB | navigation launch logs and the move_base settings actually loaded |
| `s2_nav_02/nav_B2_part2/fixes.csv` | 1 KB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_02/nav_B2/goals_result.json` | 953 B | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_02/nav_B2_part2/goals_result.json` | 942 B | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_02/localise_summary.json` | 769 B | localisation watcher outputs: fixes, events and summary |
| `s2_nav_02/camera_guard.log` | 598 B | the camera guard: restarts, outages (start, end, seconds, cause, outcome) |
| `s2_nav_02/camera_guard.out` | 598 B | the camera guard: restarts, outages (start, end, seconds, cause, outcome) |
| `s2_nav_02/nav_goals_B2.log` | 539 B | navigation launch logs and the move_base settings actually loaded |
| `s2_nav_02/nav_goals_B2_part2.log` | 526 B | navigation launch logs and the move_base settings actually loaded |
| ... 9 more files of this kind | | |
| `.../media/` 1005 timelapse frames (png) + index/log | 150.17 MB | the live map page, one picture every 3 s, recorded during the run (the mp4 above is made from them) |
| + 7 small record files (logs, json, csv, txt, pids) | 3 KB | run records: logs, checks, progress lines |

### s2_nav_03  (1024 files, 1.47 GB)

| | |
|---|---|
| date (Hamilton time) | 27 Sep 2026, 18:53-19:03 |
| what | **navigation run 3** - route B2 full round trip: 4 of 5 goals on its own, 1.5 m straight through the doorway by command |
| why it was driven | the final navigation result |
| scored in | `results/navigation/s2_nav_03/RESULTS.md` |

| file | size | what it contains |
|---|---|---|
| `s2_nav_03/s2_nav_03/fusion.bag` | 1.32 GB | the Jetson's recording of every position stream during the run (ROS bag). Topics: /rtabmap/odom /robot/wheel_odom /robot/imu /robot/imu_raw /robot/ekf_odom /robot_bridge/status /ekf_in/wheel_odom /ekf_in/imu /ekf_in/vo_odom /fused/odometry /tf /tf_static /diagnostics /zedx_front/zed_node/imu/data /rtabmap/odom_info_lite /zedx_front/zed_node/left/camera_info /zedx_front/zed_node/right/camera_info (+ /rtabmap/info and /rtabmap/localization_pose in localisation runs; + /zedx_front/zed_node/obj_det/objects in SLAM sessions with the people mask) |
| `s2_nav_03/s2_nav_03/nav_B2/plans.jsonl` | 1.73 MB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_03/s2_nav_03/mapping.log` | 1.46 MB | RTAB-Map's own log for the run (loop closures accepted/rejected, saves, warnings) |
| `s2_nav_03/s2_nav_03/media/s2_nav_03_timelapse.mp4` | 846 KB | timelapse video of the live map page, recorded during the run |
| `s2_nav_03/s2_nav_03/nav_B2home/plans.jsonl` | 694 KB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_03/s2_nav_03/tegrastats.log` | 467 KB | NVIDIA's once-a-second processor/GPU/memory/power log of the Jetson |
| `s2_nav_03/s2_nav_03/nav_B2rest/plans.jsonl` | 262 KB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_03/s2_nav_03/nav_B2/cmd.csv` | 131 KB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_03/s2_nav_03/nav_B2/robot_status.jsonl` | 97 KB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_03/s2_nav_03/nav_B2home/cmd.csv` | 91 KB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_03/s2_nav_03/nav_B2/actual.csv` | 71 KB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_03/s2_nav_03/nav_B2/odom.csv` | 71 KB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_03/s2_nav_03/nav_B2home/robot_status.jsonl` | 67 KB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_03/s2_nav_03/nav_launch.log` | 52 KB | navigation launch logs and the move_base settings actually loaded |
| `s2_nav_03/s2_nav_03/nav_B2home/odom.csv` | 47 KB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_03/s2_nav_03/nav_B2home/actual.csv` | 47 KB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_03/s2_nav_03/monitor.csv` | 37 KB | the live mapping monitor's per-second counters and its last status line |
| `s2_nav_03/s2_nav_03/localise_track.csv` | 28 KB | localisation watcher outputs: fixes, events and summary |
| `s2_nav_03/s2_nav_03/localise_events.csv` | 23 KB | localisation watcher outputs: fixes, events and summary |
| `s2_nav_03/s2_nav_03/camera.log` | 21 KB | the ZED camera program's log |
| `s2_nav_03/s2_nav_03/nav_B2rest/cmd.csv` | 12 KB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_03/s2_nav_03/nav_B2rest/robot_status.jsonl` | 11 KB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_03/s2_nav_03/nav_B2rest/odom.csv` | 8 KB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_03/s2_nav_03/nav_B2rest/actual.csv` | 8 KB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_03/s2_nav_03/fused_odometry.log` | 7 KB | the blend (EKF) and its input conditioner's log, incl. the kept gyroscope offset |
| `s2_nav_03/s2_nav_03/nav_params_move_base.yaml` | 6 KB | navigation launch logs and the move_base settings actually loaded |
| `s2_nav_03/s2_nav_03/navcmd_sender.log` | 5 KB | navigation launch logs and the move_base settings actually loaded |
| `s2_nav_03/s2_nav_03/nav_dynparam_readback.txt` | 4 KB | navigation launch logs and the move_base settings actually loaded |
| `s2_nav_03/s2_nav_03/bridge_recv.log` | 2 KB | the robot-to-Jetson link receiver's log (delivered share of the robot's readings) |
| `s2_nav_03/s2_nav_03/nav_B2/fixes.csv` | 2 KB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_03/s2_nav_03/nav_B2home/fixes.csv` | 2 KB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_03/s2_nav_03/fusion_bag.log` | 1 KB | navigation launch logs and the move_base settings actually loaded |
| `s2_nav_03/s2_nav_03/nav_B2/goals_result.json` | 1 KB | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_03/s2_nav_03/tf_check_2.txt` | 1 KB | navigation launch logs and the move_base settings actually loaded |
| `s2_nav_03/s2_nav_03/nav_goals_B2.log` | 876 B | navigation launch logs and the move_base settings actually loaded |
| `s2_nav_03/s2_nav_03/localise_summary.json` | 771 B | localisation watcher outputs: fixes, events and summary |
| `s2_nav_03/s2_nav_03/nav_B2rest/goals_result.json` | 686 B | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_03/s2_nav_03/nav_B2home/goals_result.json` | 681 B | navigation records per route: commands, robot position, plans, fixes, goal results |
| `s2_nav_03/s2_nav_03/camera_guard.out` | 598 B | the camera guard: restarts, outages (start, end, seconds, cause, outcome) |
| `s2_nav_03/s2_nav_03/camera_guard.log` | 598 B | the camera guard: restarts, outages (start, end, seconds, cause, outcome) |
| ... 11 more files of this kind | | |
| `.../media/` 966 timelapse frames (png) + index/log | 144.68 MB | the live map page, one picture every 3 s, recorded during the run (the mp4 above is made from them) |
| + 7 small record files (logs, json, csv, txt, pids) | 3 KB | run records: logs, checks, progress lines |

### s2_scale_01  (688 files, 1.73 GB)

| | |
|---|---|
| date (Hamilton time) | 26 Sep 2026, 13:10-13:22 |
| what | **taped 15.00 m distance test** - 3 round trips between two taped marks |
| why it was driven | which ruler is honest? camera 14.90 m (-0.7 %), wheels 15.94 m (+6.2 %) |
| scored in | `results/series2_static/s2_scale_01/RESULTS.md` |

| file | size | what it contains |
|---|---|---|
| `s2_scale_01/s2_scale_01.db` | 1.31 GB | RTAB-Map database: every map snapshot (picture + position), the links between them, and the corrected positions if closed properly |
| `s2_scale_01/s2_scale_01/fusion.bag` | 393.53 MB | the Jetson's recording of every position stream during the run (ROS bag). Topics: /rtabmap/odom /robot/wheel_odom /robot/imu /robot/imu_raw /robot/ekf_odom /robot_bridge/status /ekf_in/wheel_odom /ekf_in/imu /ekf_in/vo_odom /fused/odometry /tf /tf_static /diagnostics /zedx_front/zed_node/imu/data /rtabmap/odom_info_lite /zedx_front/zed_node/left/camera_info /zedx_front/zed_node/right/camera_info (+ /rtabmap/info and /rtabmap/localization_pose in localisation runs; + /zedx_front/zed_node/obj_det/objects in SLAM sessions with the people mask) |
| `s2_scale_01/s2_scale_01/mapping.log` | 1.42 MB | RTAB-Map's own log for the run (loop closures accepted/rejected, saves, warnings) |
| `s2_scale_01/s2_scale_01/media/s2_scale_01_timelapse.mp4` | 551 KB | timelapse video of the live map page, recorded during the run |
| `s2_scale_01/s2_scale_01/monitor.csv` | 25 KB | the live mapping monitor's per-second counters and its last status line |
| `s2_scale_01/s2_scale_01/camera.log` | 21 KB | the ZED camera program's log |
| `s2_scale_01/s2_scale_01/fused_odometry.log` | 5 KB | the blend (EKF) and its input conditioner's log, incl. the kept gyroscope offset |
| `s2_scale_01/s2_scale_01/autostop.log` | 5 KB | the stop-on-'park' helper's log |
| `s2_scale_01/s2_scale_01/bridge_recv.log` | 2 KB | the robot-to-Jetson link receiver's log (delivered share of the robot's readings) |
| `s2_scale_01/s2_scale_01/camera_guard.out` | 604 B | the camera guard: restarts, outages (start, end, seconds, cause, outcome) |
| `s2_scale_01/s2_scale_01/camera_guard.log` | 604 B | the camera guard: restarts, outages (start, end, seconds, cause, outcome) |
| `s2_scale_01/s2_scale_01/camera_guard.json` | 289 B | the camera guard: restarts, outages (start, end, seconds, cause, outcome) |
| `s2_scale_01/s2_scale_01/monitor_STATUS.txt` | 135 B | the live mapping monitor's per-second counters and its last status line |
| `.../media/` 666 timelapse frames (png) + index/log | 35.99 MB | the live map page, one picture every 3 s, recorded during the run (the mp4 above is made from them) |
| + 9 small record files (logs, json, csv, txt, pids) | 6 KB | run records: logs, checks, progress lines |

### s2_slam_01  (1487 files, 16.56 GB)

| | |
|---|---|
| date (Hamilton time) | 27 Sep 2026, 13:25-13:52 |
| what | **SLAM session** - continued drive 10's map; 156 m; 73 m2 of new floor; people mask test |
| why it was driven | full SLAM on an existing map (S1-S5) and the people-blanking replay |
| scored in | `results/series2_slam/s2_slam_01/RESULTS.md` |

| file | size | what it contains |
|---|---|---|
| `s2_slam_01/replays/s2_slam_01_replay_on.db` | 5.52 GB | RTAB-Map database: every map snapshot (picture + position), the links between them, and the corrected positions if closed properly |
| `s2_slam_01/replays/s2_slam_01_replay_off.db` | 5.38 GB | RTAB-Map database: every map snapshot (picture + position), the links between them, and the corrected positions if closed properly |
| `s2_slam_01/s2_slam_01.svo2` | 4.20 GB | ZED X camera recording (stereo images + camera IMU); depth can be recomputed offline in any depth mode |
| `s2_slam_01/fusion.bag` | 1.21 GB | the Jetson's recording of every position stream during the run (ROS bag). Topics: /rtabmap/odom /robot/wheel_odom /robot/imu /robot/imu_raw /robot/ekf_odom /robot_bridge/status /ekf_in/wheel_odom /ekf_in/imu /ekf_in/vo_odom /fused/odometry /tf /tf_static /diagnostics /zedx_front/zed_node/imu/data /rtabmap/odom_info_lite /zedx_front/zed_node/left/camera_info /zedx_front/zed_node/right/camera_info (+ /rtabmap/info and /rtabmap/localization_pose in localisation runs; + /zedx_front/zed_node/obj_det/objects in SLAM sessions with the people mask) |
| `s2_slam_01/mapping.log` | 3.67 MB | RTAB-Map's own log for the run (loop closures accepted/rejected, saves, warnings) |
| `s2_slam_01/media/s2_slam_01_timelapse.mp4` | 1.64 MB | timelapse video of the live map page, recorded during the run |
| `s2_slam_01/tegrastats.log` | 781 KB | NVIDIA's once-a-second processor/GPU/memory/power log of the Jetson |
| `s2_slam_01/monitor.csv` | 66 KB | the live mapping monitor's per-second counters and its last status line |
| `s2_slam_01/camera.log` | 22 KB | the ZED camera program's log |
| `s2_slam_01/fused_odometry.log` | 10 KB | the blend (EKF) and its input conditioner's log, incl. the kept gyroscope offset |
| `s2_slam_01/autostop.log` | 9 KB | the stop-on-'park' helper's log |
| `s2_slam_01/bridge_recv.log` | 2 KB | the robot-to-Jetson link receiver's log (delivered share of the robot's readings) |
| `s2_slam_01/camera_guard.log` | 728 B | the camera guard: restarts, outages (start, end, seconds, cause, outcome) |
| `s2_slam_01/camera_guard.out` | 728 B | the camera guard: restarts, outages (start, end, seconds, cause, outcome) |
| `s2_slam_01/camera_guard.json` | 285 B | the camera guard: restarts, outages (start, end, seconds, cause, outcome) |
| `s2_slam_01/monitor_STATUS.txt` | 140 B | the live mapping monitor's per-second counters and its last status line |
| `.../media/` 1449 timelapse frames (png) + index/log | 245.51 MB | the live map page, one picture every 3 s, recorded during the run (the mp4 above is made from them) |
| + 22 small record files (logs, json, csv, txt, pids) | 72 KB | run records: logs, checks, progress lines |

### s2_static_01  (13 files, 4.74 GB)

| | |
|---|---|
| date (Hamilton time) | 24 Sep 2026, 01:09-03:30 |
| what | **walkway drive 1 (pilot)** - camera only; 240 m of corridors, 17 min |
| why it was driven | first walkway drive; found 59 tracking losses at blank walls (39.6 m tracking gap) |
| scored in | `DRIVE1_DIAGNOSIS.md` (project records; no pack) |

| file | size | what it contains |
|---|---|---|
| `s2_static_01/raw_from_robot/s2_static_01_lidar.bag` | 4.08 GB | the robot's LiDAR recording: raw laser packets, not expanded clouds. Topics: /helios/packets /ouster/lidar_packets /ouster/imu_packets /tf /tf_static /odometry/filtered /imu/data /husky_velocity_controller/odom /imu/data_raw /helios_pcl/scan /ouster_pcl/scan |
| `s2_static_01/raw_from_robot/s2_static_01/rtab_helios.db` | 611.05 MB | the LiDAR map database written by the colleague's replay pipeline (RTAB-Map, LiDAR mode) |
| `s2_static_01/raw_from_robot/s2_static_01/play.log` | 56.19 MB | a small record file (numbers, log or provenance) |
| `s2_static_01/raw_from_robot/s2_static_01/wheel.tum` | 4.19 MB | the robot's own wheel+gyroscope path (/odometry/filtered) in TUM format |
| `s2_static_01/raw_from_robot/s2_static_01/lidar_map.png` | 241 KB | the LiDAR map's walls (cells 0.15-2.0 m above the base) as a picture / array |
| `s2_static_01/raw_from_robot/s2_static_01/lidar_map.npz` | 106 KB | the LiDAR map's walls (cells 0.15-2.0 m above the base) as a picture / array |
| `s2_static_01/raw_from_robot/s2_static_01/lidar.tum` | 61 KB | the LiDAR estimate's corrected path (TUM format: time x y z qx qy qz qw), one line per map node |
| + 6 small record files (logs, json, csv, txt, pids) | 648 KB | run records: logs, checks, progress lines |

### s2_static_02  (13 files, 7.07 GB)

| | |
|---|---|
| date (Hamilton time) | 24 Sep 2026, 03:54-04:17 |
| what | **walkway drive 2** - camera only, slower, same-direction re-passes; 277 m, 22 min |
| why it was driven | test the rewritten driving procedure; first map shut down properly on 'park' |
| scored in | `- (numbers in docs/worksheet/index.html, section 1.2)` |

| file | size | what it contains |
|---|---|---|
| `s2_static_02/raw_from_robot/s2_static_02_lidar.bag` | 6.24 GB | the robot's LiDAR recording: raw laser packets, not expanded clouds. Topics: /helios/packets /ouster/lidar_packets /ouster/imu_packets /tf /tf_static /odometry/filtered /imu/data /husky_velocity_controller/odom /imu/data_raw /helios_pcl/scan /ouster_pcl/scan |
| `s2_static_02/raw_from_robot/s2_static_02/rtab_helios.db` | 746.56 MB | the LiDAR map database written by the colleague's replay pipeline (RTAB-Map, LiDAR mode) |
| `s2_static_02/raw_from_robot/s2_static_02/play.log` | 93.15 MB | a small record file (numbers, log or provenance) |
| `s2_static_02/raw_from_robot/s2_static_02/wheel.tum` | 6.41 MB | the robot's own wheel+gyroscope path (/odometry/filtered) in TUM format |
| `s2_static_02/raw_from_robot/s2_static_02/lidar_map.png` | 260 KB | the LiDAR map's walls (cells 0.15-2.0 m above the base) as a picture / array |
| `s2_static_02/raw_from_robot/s2_static_02/lidar_map.npz` | 118 KB | the LiDAR map's walls (cells 0.15-2.0 m above the base) as a picture / array |
| `s2_static_02/raw_from_robot/s2_static_02/lidar.tum` | 75 KB | the LiDAR estimate's corrected path (TUM format: time x y z qx qy qz qw), one line per map node |
| + 6 small record files (logs, json, csv, txt, pids) | 330 KB | run records: logs, checks, progress lines |

### s2_static_03  (21 files, 350.48 MB)

| | |
|---|---|
| date (Hamilton time) | 25 Sep 2026, 04:30-04:58 |
| what | **walkway drive 3** - first fused drive (camera + wheels + gyroscope); 140 m, 28 min |
| why it was driven | does the blend stop the camera's tracking losses from throwing away distance? |
| scored in | `results/series2_static/s2_static_03/RESULTS.md` |

| file | size | what it contains |
|---|---|---|
| `s2_static_03/s2_static_03/fusion.bag` | 347.01 MB | the Jetson's recording of every position stream during the run (ROS bag). Topics: /rtabmap/odom /robot/wheel_odom /robot/imu /robot/imu_raw /robot/ekf_odom /robot_bridge/status /ekf_in/wheel_odom /ekf_in/imu /ekf_in/vo_odom /fused/odometry /tf /tf_static /diagnostics /zedx_front/zed_node/imu/data /rtabmap/odom_info_lite /zedx_front/zed_node/left/camera_info /zedx_front/zed_node/right/camera_info (+ /rtabmap/info and /rtabmap/localization_pose in localisation runs; + /zedx_front/zed_node/obj_det/objects in SLAM sessions with the people mask) |
| `s2_static_03/s2_static_03/mapping.log` | 3.10 MB | RTAB-Map's own log for the run (loop closures accepted/rejected, saves, warnings) |
| `s2_static_03/s2_static_03/monitor.csv` | 65 KB | the live mapping monitor's per-second counters and its last status line |
| `s2_static_03/s2_static_03/turns.log` | 22 KB | the live turn watcher: robot vs camera vs blend heading per turn |
| `s2_static_03/s2_static_03/fused_odometry.log` | 12 KB | the blend (EKF) and its input conditioner's log, incl. the kept gyroscope offset |
| `s2_static_03/s2_static_03/autostop.log` | 10 KB | the stop-on-'park' helper's log |
| `s2_static_03/s2_static_03/bridge_recv.log` | 3 KB | the robot-to-Jetson link receiver's log (delivered share of the robot's readings) |
| `s2_static_03/s2_static_03/monitor_STATUS.txt` | 140 B | the live mapping monitor's per-second counters and its last status line |
| + 13 small record files (logs, json, csv, txt, pids) | 270 KB | run records: logs, checks, progress lines |

### s2_static_04  (658 files, 5.36 GB)

| | |
|---|---|
| date (Hamilton time) | 26 Sep 2026, 00:44-00:53 |
| what | **walkway drive 4** - the reference route; 87 m, 10 min |
| why it was driven | drive 3's fixes together (16-bit depth, parked-drop guard, camera gyroscope); became the route later drives repeat |
| scored in | `results/series2_static/s2_static_04/RESULTS.md` |

| file | size | what it contains |
|---|---|---|
| `s2_static_04/raw_from_robot/s2_static_04_lidar.bag` | 3.24 GB | the robot's LiDAR recording: raw laser packets, not expanded clouds. Topics: /helios/packets /ouster/lidar_packets /ouster/imu_packets /tf /tf_static /odometry/filtered /imu/data /husky_velocity_controller/odom /imu/data_raw /helios_pcl/scan /ouster_pcl/scan |
| `s2_static_04/s2_static_04.db` | 1.10 GB | RTAB-Map database: every map snapshot (picture + position), the links between them, and the corrected positions if closed properly |
| `s2_static_04/s2_static_04/fusion.bag` | 667.53 MB | the Jetson's recording of every position stream during the run (ROS bag). Topics: /rtabmap/odom /robot/wheel_odom /robot/imu /robot/imu_raw /robot/ekf_odom /robot_bridge/status /ekf_in/wheel_odom /ekf_in/imu /ekf_in/vo_odom /fused/odometry /tf /tf_static /diagnostics /zedx_front/zed_node/imu/data /rtabmap/odom_info_lite /zedx_front/zed_node/left/camera_info /zedx_front/zed_node/right/camera_info (+ /rtabmap/info and /rtabmap/localization_pose in localisation runs; + /zedx_front/zed_node/obj_det/objects in SLAM sessions with the people mask) |
| `s2_static_04/raw_from_robot/s2_static_04/rtab_helios.db` | 272.08 MB | the LiDAR map database written by the colleague's replay pipeline (RTAB-Map, LiDAR mode) |
| `s2_static_04/raw_from_robot/s2_static_04/play.log` | 53.77 MB | a small record file (numbers, log or provenance) |
| `s2_static_04/raw_from_robot/s2_static_04/wheel.tum` | 3.22 MB | the robot's own wheel+gyroscope path (/odometry/filtered) in TUM format |
| `s2_static_04/s2_static_04/mapping.log` | 1.44 MB | RTAB-Map's own log for the run (loop closures accepted/rejected, saves, warnings) |
| `s2_static_04/s2_static_04/media/s2_static_04_timelapse.mp4` | 613 KB | timelapse video of the live map page, recorded during the run |
| `s2_static_04/raw_from_robot/s2_static_04/carving_test_2026-09-26_0146/carving_before_after.png` | 107 KB | a figure or photo |
| `s2_static_04/raw_from_robot/s2_static_04/carving_test_2026-09-26_0146/C/lidar_map.png` | 81 KB | the LiDAR map's walls (cells 0.15-2.0 m above the base) as a picture / array |
| `s2_static_04/raw_from_robot/s2_static_04/carving_test_2026-09-26_0146/B/lidar_map.png` | 75 KB | the LiDAR map's walls (cells 0.15-2.0 m above the base) as a picture / array |
| `s2_static_04/raw_from_robot/s2_static_04/carving_test_2026-09-26_0146/A/lidar_map.png` | 74 KB | the LiDAR map's walls (cells 0.15-2.0 m above the base) as a picture / array |
| `s2_static_04/raw_from_robot/s2_static_04/lidar.tum` | 27 KB | the LiDAR estimate's corrected path (TUM format: time x y z qx qy qz qw), one line per map node |
| `s2_static_04/s2_static_04/monitor.csv` | 24 KB | the live mapping monitor's per-second counters and its last status line |
| `s2_static_04/raw_from_robot/s2_static_04/carving_test_2026-09-26_0146/C/lidar_map.npz` | 15 KB | the LiDAR map's walls (cells 0.15-2.0 m above the base) as a picture / array |
| `s2_static_04/raw_from_robot/s2_static_04/carving_test_2026-09-26_0146/B/lidar_map.npz` | 15 KB | the LiDAR map's walls (cells 0.15-2.0 m above the base) as a picture / array |
| `s2_static_04/s2_static_04/camera.log` | 15 KB | the ZED camera program's log |
| `s2_static_04/raw_from_robot/s2_static_04/carving_test_2026-09-26_0146/A/lidar_map.npz` | 15 KB | the LiDAR map's walls (cells 0.15-2.0 m above the base) as a picture / array |
| `s2_static_04/s2_static_04/fused_odometry.log` | 6 KB | the blend (EKF) and its input conditioner's log, incl. the kept gyroscope offset |
| `s2_static_04/raw_from_robot/s2_static_04/carving_diag_2026-09-26_0151/diag_cells.py` | 5 KB | one of our tools/scripts (copy kept with the data) |
| `s2_static_04/s2_static_04/autostop.log` | 4 KB | the stop-on-'park' helper's log |
| `s2_static_04/s2_static_04/bridge_recv.log` | 2 KB | the robot-to-Jetson link receiver's log (delivered share of the robot's readings) |
| `s2_static_04/raw_from_robot/s2_static_04_gyro_zero.json` | 174 B | the robot gyroscope's standing-still bias measured before the drive |
| `s2_static_04/s2_static_04/monitor_STATUS.txt` | 138 B | the live mapping monitor's per-second counters and its last status line |
| `s2_static_04/raw_from_robot/s2_static_04/lidar_map.png` | 0 B | the LiDAR map's walls (cells 0.15-2.0 m above the base) as a picture / array |
| `s2_static_04/raw_from_robot/s2_static_04/lidar_map.npz` | 0 B | the LiDAR map's walls (cells 0.15-2.0 m above the base) as a picture / array |
| `.../media/` 603 timelapse frames (png) + index/log | 40.74 MB | the live map page, one picture every 3 s, recorded during the run (the mp4 above is made from them) |
| + 29 small record files (logs, json, csv, txt, pids) | 607 KB | run records: logs, checks, progress lines |

### s2_static_05  (1461 files, 2.59 GB)

| | |
|---|---|
| date (Hamilton time) | 26 Sep 2026, 02:14-02:35 |
| what | **walkway drive 5** - longer loop; 61 m mapped before the camera crashed at 11.2 min |
| why it was driven | disk-save waits off; a person behind the robot for the LiDAR clearing test; camera program reused from drive 4 and crashed |
| scored in | `results/series2_static/s2_static_05/RESULTS.md` |

| file | size | what it contains |
|---|---|---|
| `s2_static_05/s2_static_05.db` | 1.42 GB | RTAB-Map database: every map snapshot (picture + position), the links between them, and the corrected positions if closed properly |
| `s2_static_05/s2_static_05/fusion.bag` | 1.00 GB | the Jetson's recording of every position stream during the run (ROS bag). Topics: /rtabmap/odom /robot/wheel_odom /robot/imu /robot/imu_raw /robot/ekf_odom /robot_bridge/status /ekf_in/wheel_odom /ekf_in/imu /ekf_in/vo_odom /fused/odometry /tf /tf_static /diagnostics /zedx_front/zed_node/imu/data /rtabmap/odom_info_lite /zedx_front/zed_node/left/camera_info /zedx_front/zed_node/right/camera_info (+ /rtabmap/info and /rtabmap/localization_pose in localisation runs; + /zedx_front/zed_node/obj_det/objects in SLAM sessions with the people mask) |
| `s2_static_05/s2_static_05/mapping.log` | 1.52 MB | RTAB-Map's own log for the run (loop closures accepted/rejected, saves, warnings) |
| `s2_static_05/s2_static_05/media/s2_static_05_timelapse.mp4` | 1.38 MB | timelapse video of the live map page, recorded during the run |
| `s2_static_05/s2_static_05/monitor.csv` | 57 KB | the live mapping monitor's per-second counters and its last status line |
| `s2_static_05/s2_static_05/autostop.log` | 9 KB | the stop-on-'park' helper's log |
| `s2_static_05/s2_static_05/fused_odometry.log` | 8 KB | the blend (EKF) and its input conditioner's log, incl. the kept gyroscope offset |
| `s2_static_05/s2_static_05/bridge_recv.log` | 3 KB | the robot-to-Jetson link receiver's log (delivered share of the robot's readings) |
| `s2_static_05/s2_static_05/monitor_STATUS.txt` | 136 B | the live mapping monitor's per-second counters and its last status line |
| `.../media/` 1442 timelapse frames (png) + index/log | 163.49 MB | the live map page, one picture every 3 s, recorded during the run (the mp4 above is made from them) |
| + 10 small record files (logs, json, csv, txt, pids) | 9 KB | run records: logs, checks, progress lines |

### s2_static_06  (1479 files, 11.83 GB)

| | |
|---|---|
| date (Hamilton time) | 26 Sep 2026, 04:20-04:45 |
| what | **walkway drive 6** - new walkway added; 238 m, 25 min |
| why it was driven | first drive with the camera guard (restart in 12.7 s); LiDAR map tilted -> Force3DoF replay |
| scored in | `results/series2_static/s2_static_06/RESULTS.md` |

| file | size | what it contains |
|---|---|---|
| `s2_static_06/raw_from_robot/s2_static_06_lidar.bag` | 7.33 GB | the robot's LiDAR recording: raw laser packets, not expanded clouds. Topics: /helios/packets /ouster/lidar_packets /ouster/imu_packets /tf /tf_static /odometry/filtered /imu/data /husky_velocity_controller/odom /imu/data_raw /helios_pcl/scan /ouster_pcl/scan |
| `s2_static_06/s2_static_06.db` | 2.80 GB | RTAB-Map database: every map snapshot (picture + position), the links between them, and the corrected positions if closed properly |
| `s2_static_06/s2_static_06/fusion.bag` | 1.46 GB | the Jetson's recording of every position stream during the run (ROS bag). Topics: /rtabmap/odom /robot/wheel_odom /robot/imu /robot/imu_raw /robot/ekf_odom /robot_bridge/status /ekf_in/wheel_odom /ekf_in/imu /ekf_in/vo_odom /fused/odometry /tf /tf_static /diagnostics /zedx_front/zed_node/imu/data /rtabmap/odom_info_lite /zedx_front/zed_node/left/camera_info /zedx_front/zed_node/right/camera_info (+ /rtabmap/info and /rtabmap/localization_pose in localisation runs; + /zedx_front/zed_node/obj_det/objects in SLAM sessions with the people mask) |
| `s2_static_06/s2_static_06/mapping.log` | 3.14 MB | RTAB-Map's own log for the run (loop closures accepted/rejected, saves, warnings) |
| `s2_static_06/s2_static_06/media/s2_static_06_timelapse.mp4` | 2.25 MB | timelapse video of the live map page, recorded during the run |
| `s2_static_06/lidar/s2_static_06/lidar_map.png` | 169 KB | the LiDAR map's walls (cells 0.15-2.0 m above the base) as a picture / array |
| `s2_static_06/lidar/s2_static_06/lidar.tum` | 80 KB | the LiDAR estimate's corrected path (TUM format: time x y z qx qy qz qw), one line per map node |
| `s2_static_06/lidar/s2_static_06/lidar_map.npz` | 63 KB | the LiDAR map's walls (cells 0.15-2.0 m above the base) as a picture / array |
| `s2_static_06/s2_static_06/monitor.csv` | 58 KB | the live mapping monitor's per-second counters and its last status line |
| `s2_static_06/s2_static_06/camera.log` | 43 KB | the ZED camera program's log |
| `s2_static_06/s2_static_06/bridge_recv.log` | 11 KB | the robot-to-Jetson link receiver's log (delivered share of the robot's readings) |
| `s2_static_06/s2_static_06/fused_odometry.log` | 9 KB | the blend (EKF) and its input conditioner's log, incl. the kept gyroscope offset |
| `s2_static_06/s2_static_06/autostop.log` | 9 KB | the stop-on-'park' helper's log |
| `s2_static_06/s2_static_06/camera_guard.out` | 1 KB | the camera guard: restarts, outages (start, end, seconds, cause, outcome) |
| `s2_static_06/s2_static_06/camera_guard.log` | 1 KB | the camera guard: restarts, outages (start, end, seconds, cause, outcome) |
| `s2_static_06/s2_static_06/camera_guard.json` | 394 B | the camera guard: restarts, outages (start, end, seconds, cause, outcome) |
| `s2_static_06/s2_static_06/camera_outages.csv` | 216 B | the camera guard: restarts, outages (start, end, seconds, cause, outcome) |
| `s2_static_06/s2_static_06/monitor_STATUS.txt` | 139 B | the live mapping monitor's per-second counters and its last status line |
| `.../media/` 1441 timelapse frames (png) + index/log | 234.72 MB | the live map page, one picture every 3 s, recorded during the run (the mp4 above is made from them) |
| + 20 small record files (logs, json, csv, txt, pids) | 1.25 MB | run records: logs, checks, progress lines |

### s2_static_07  (804 files, 2.61 GB)

| | |
|---|---|
| date (Hamilton time) | 26 Sep 2026, 13:28-13:40 |
| what | **walkway drive 7** - drive 4's corridors a bit further, camera tracker GEN_1; 107 m, 14 min |
| why it was driven | does the older tracker stop the crashes? (no crash, but 16 freezes and only 30 closures) |
| scored in | `results/series2_static/s2_static_07/RESULTS.md` |

| file | size | what it contains |
|---|---|---|
| `s2_static_07/s2_static_07.db` | 1.50 GB | RTAB-Map database: every map snapshot (picture + position), the links between them, and the corrected positions if closed properly |
| `s2_static_07/s2_static_07/fusion.bag` | 1.03 GB | the Jetson's recording of every position stream during the run (ROS bag). Topics: /rtabmap/odom /robot/wheel_odom /robot/imu /robot/imu_raw /robot/ekf_odom /robot_bridge/status /ekf_in/wheel_odom /ekf_in/imu /ekf_in/vo_odom /fused/odometry /tf /tf_static /diagnostics /zedx_front/zed_node/imu/data /rtabmap/odom_info_lite /zedx_front/zed_node/left/camera_info /zedx_front/zed_node/right/camera_info (+ /rtabmap/info and /rtabmap/localization_pose in localisation runs; + /zedx_front/zed_node/obj_det/objects in SLAM sessions with the people mask) |
| `s2_static_07/s2_static_07/mapping.log` | 1.71 MB | RTAB-Map's own log for the run (loop closures accepted/rejected, saves, warnings) |
| `s2_static_07/s2_static_07/media/s2_static_07_timelapse.mp4` | 1024 KB | timelapse video of the live map page, recorded during the run |
| `s2_static_07/s2_static_07/monitor.csv` | 28 KB | the live mapping monitor's per-second counters and its last status line |
| `s2_static_07/s2_static_07/camera.log` | 21 KB | the ZED camera program's log |
| `s2_static_07/s2_static_07/fused_odometry.log` | 7 KB | the blend (EKF) and its input conditioner's log, incl. the kept gyroscope offset |
| `s2_static_07/s2_static_07/autostop.log` | 5 KB | the stop-on-'park' helper's log |
| `s2_static_07/s2_static_07/bridge_recv.log` | 3 KB | the robot-to-Jetson link receiver's log (delivered share of the robot's readings) |
| `s2_static_07/s2_static_07/camera_guard.out` | 605 B | the camera guard: restarts, outages (start, end, seconds, cause, outcome) |
| `s2_static_07/s2_static_07/camera_guard.log` | 605 B | the camera guard: restarts, outages (start, end, seconds, cause, outcome) |
| `s2_static_07/s2_static_07/camera_guard.json` | 290 B | the camera guard: restarts, outages (start, end, seconds, cause, outcome) |
| `s2_static_07/s2_static_07/monitor_STATUS.txt` | 134 B | the live mapping monitor's per-second counters and its last status line |
| `.../media/` 782 timelapse frames (png) + index/log | 81.16 MB | the live map page, one picture every 3 s, recorded during the run (the mp4 above is made from them) |
| + 9 small record files (logs, json, csv, txt, pids) | 9 KB | run records: logs, checks, progress lines |

### s2_static_08  (826 files, 2.74 GB)

| | |
|---|---|
| date (Hamilton time) | 26 Sep 2026, 15:48-16:02 |
| what | **walkway drive 8** - drive 4's route repeated, GEN_2; 87 m, 14 min |
| why it was driven | the 'same result every time' test: 0.10 m home, 0.26 m from the LiDAR as on drive 4 |
| scored in | `results/series2_static/s2_static_08/RESULTS.md` |

| file | size | what it contains |
|---|---|---|
| `s2_static_08/s2_static_08.db` | 1.48 GB | RTAB-Map database: every map snapshot (picture + position), the links between them, and the corrected positions if closed properly |
| `s2_static_08/s2_static_08/fusion.bag` | 1.19 GB | the Jetson's recording of every position stream during the run (ROS bag). Topics: /rtabmap/odom /robot/wheel_odom /robot/imu /robot/imu_raw /robot/ekf_odom /robot_bridge/status /ekf_in/wheel_odom /ekf_in/imu /ekf_in/vo_odom /fused/odometry /tf /tf_static /diagnostics /zedx_front/zed_node/imu/data /rtabmap/odom_info_lite /zedx_front/zed_node/left/camera_info /zedx_front/zed_node/right/camera_info (+ /rtabmap/info and /rtabmap/localization_pose in localisation runs; + /zedx_front/zed_node/obj_det/objects in SLAM sessions with the people mask) |
| `s2_static_08/s2_static_08/mapping.log` | 1.75 MB | RTAB-Map's own log for the run (loop closures accepted/rejected, saves, warnings) |
| `s2_static_08/s2_static_08/media/s2_static_08_timelapse.mp4` | 996 KB | timelapse video of the live map page, recorded during the run |
| `s2_static_08/s2_static_08/tegrastats.log` | 412 KB | NVIDIA's once-a-second processor/GPU/memory/power log of the Jetson |
| `s2_static_08/s2_static_08/camera.log` | 217 KB | the ZED camera program's log |
| `s2_static_08/s2_static_08/monitor.csv` | 29 KB | the live mapping monitor's per-second counters and its last status line |
| `s2_static_08/s2_static_08/fused_odometry.log` | 6 KB | the blend (EKF) and its input conditioner's log, incl. the kept gyroscope offset |
| `s2_static_08/s2_static_08/autostop.log` | 6 KB | the stop-on-'park' helper's log |
| `s2_static_08/s2_static_08/bridge_recv.log` | 2 KB | the robot-to-Jetson link receiver's log (delivered share of the robot's readings) |
| `s2_static_08/s2_static_08/camera_guard.out` | 1 KB | the camera guard: restarts, outages (start, end, seconds, cause, outcome) |
| `s2_static_08/s2_static_08/camera_guard.log` | 1 KB | the camera guard: restarts, outages (start, end, seconds, cause, outcome) |
| `s2_static_08/s2_static_08/camera_guard.json` | 422 B | the camera guard: restarts, outages (start, end, seconds, cause, outcome) |
| `s2_static_08/s2_static_08/camera_outages.csv` | 241 B | the camera guard: restarts, outages (start, end, seconds, cause, outcome) |
| `s2_static_08/s2_static_08/monitor_STATUS.txt` | 137 B | the live mapping monitor's per-second counters and its last status line |
| `.../media/` 801 timelapse frames (png) + index/log | 64.88 MB | the live map page, one picture every 3 s, recorded during the run (the mp4 above is made from them) |
| + 10 small record files (logs, json, csv, txt, pids) | 7 KB | run records: logs, checks, progress lines |

### s2_static_09  (1070 files, 10.05 GB)

| | |
|---|---|
| date (Hamilton time) | 26 Sep 2026, 20:52-21:21 |
| what | **walkway drive 9** - the first longer route, fresh Jetson restart; 252 m by its map nodes (tracking alone), 29 min; map NOT closed (WiFi lost) |
| why it was driven | 3x drive 4's distance; found the 5-minute WiFi roaming timer |
| scored in | `results/series2_static/s2_static_09/RESULTS.md` |

| file | size | what it contains |
|---|---|---|
| `s2_static_09/s2_static_09.db` | 2.75 GB | RTAB-Map database: every map snapshot (picture + position), the links between them, and the corrected positions if closed properly |
| `s2_static_09/s2_static_09.db` | 2.75 GB | RTAB-Map database: every map snapshot (picture + position), the links between them, and the corrected positions if closed properly |
| `s2_static_09/s2_static_09/fusion.bag.active` | 2.23 GB | the Jetson's recording of every position stream during the run (ROS bag). Topics: /rtabmap/odom /robot/wheel_odom /robot/imu /robot/imu_raw /robot/ekf_odom /robot_bridge/status /ekf_in/wheel_odom /ekf_in/imu /ekf_in/vo_odom /fused/odometry /tf /tf_static /diagnostics /zedx_front/zed_node/imu/data /rtabmap/odom_info_lite /zedx_front/zed_node/left/camera_info /zedx_front/zed_node/right/camera_info (+ /rtabmap/info and /rtabmap/localization_pose in localisation runs; + /zedx_front/zed_node/obj_det/objects in SLAM sessions with the people mask) |
| `s2_static_09/s2_static_09/fusion.bag.active` | 2.23 GB | the Jetson's recording of every position stream during the run (ROS bag). Topics: /rtabmap/odom /robot/wheel_odom /robot/imu /robot/imu_raw /robot/ekf_odom /robot_bridge/status /ekf_in/wheel_odom /ekf_in/imu /ekf_in/vo_odom /fused/odometry /tf /tf_static /diagnostics /zedx_front/zed_node/imu/data /rtabmap/odom_info_lite /zedx_front/zed_node/left/camera_info /zedx_front/zed_node/right/camera_info (+ /rtabmap/info and /rtabmap/localization_pose in localisation runs; + /zedx_front/zed_node/obj_det/objects in SLAM sessions with the people mask) |
| `s2_static_09/s2_static_09/mapping.log` | 3.73 MB | RTAB-Map's own log for the run (loop closures accepted/rejected, saves, warnings) |
| `s2_static_09/s2_static_09/tegrastats.log` | 860 KB | NVIDIA's once-a-second processor/GPU/memory/power log of the Jetson |
| `s2_static_09/s2_static_09/monitor.csv` | 71 KB | the live mapping monitor's per-second counters and its last status line |
| `s2_static_09/s2_static_09/camera.log` | 37 KB | the ZED camera program's log |
| `s2_static_09/s2_static_09/autostop.log` | 9 KB | the stop-on-'park' helper's log |
| `s2_static_09/s2_static_09/bridge_recv.log` | 3 KB | the robot-to-Jetson link receiver's log (delivered share of the robot's readings) |
| `s2_static_09/s2_static_09/camera_guard.log` | 1 KB | the camera guard: restarts, outages (start, end, seconds, cause, outcome) |
| `s2_static_09/s2_static_09/camera_guard.out` | 1 KB | the camera guard: restarts, outages (start, end, seconds, cause, outcome) |
| `s2_static_09/s2_static_09/camera_guard.json` | 413 B | the camera guard: restarts, outages (start, end, seconds, cause, outcome) |
| `s2_static_09/s2_static_09/camera_outages.csv` | 275 B | the camera guard: restarts, outages (start, end, seconds, cause, outcome) |
| `s2_static_09/s2_static_09/monitor_STATUS.txt` | 140 B | the live mapping monitor's per-second counters and its last status line |
| `s2_static_09/s2_static_09/fused_odometry.log` | 0 B | the blend (EKF) and its input conditioner's log, incl. the kept gyroscope offset |
| `.../media/` 1043 timelapse frames (png) + index/log | 86.48 MB | the live map page, one picture every 3 s, recorded during the run (the mp4 above is made from them) |
| + 11 small record files (logs, json, csv, txt, pids) | 14 KB | run records: logs, checks, progress lines |

### s2_static_10  (1671 files, 6.21 GB)

| | |
|---|---|
| date (Hamilton time) | 27 Sep 2026, 02:02-02:31 |
| what | **walkway drive 10** - the WHOLE second floor; 258 m by its map nodes (tracking alone; 252 m corrected), 29 min; closed properly |
| why it was driven | the map everything after it uses (localisation, SLAM session, navigation) |
| scored in | `results/series2_static/s2_static_10/RESULTS.md` |

| file | size | what it contains |
|---|---|---|
| `s2_static_10/s2_static_10.db` | 3.25 GB | RTAB-Map database: every map snapshot (picture + position), the links between them, and the corrected positions if closed properly |
| `s2_static_10/s2_static_10/fusion.bag` | 1.69 GB | the Jetson's recording of every position stream during the run (ROS bag). Topics: /rtabmap/odom /robot/wheel_odom /robot/imu /robot/imu_raw /robot/ekf_odom /robot_bridge/status /ekf_in/wheel_odom /ekf_in/imu /ekf_in/vo_odom /fused/odometry /tf /tf_static /diagnostics /zedx_front/zed_node/imu/data /rtabmap/odom_info_lite /zedx_front/zed_node/left/camera_info /zedx_front/zed_node/right/camera_info (+ /rtabmap/info and /rtabmap/localization_pose in localisation runs; + /zedx_front/zed_node/obj_det/objects in SLAM sessions with the people mask) |
| `s2_static_10/lidar_replay_jetson/done/lidar_ref_f3dof/rtab_helios.db` | 828.09 MB | the LiDAR map database written by the colleague's replay pipeline (RTAB-Map, LiDAR mode) |
| `s2_static_10/lidar_replay_jetson/done/lidar_ref_f3dof/play.log` | 163.34 MB | a small record file (numbers, log or provenance) |
| `s2_static_10/lidar_replay_jetson/done/lidar_ref_f3dof/wheel.tum` | 8.41 MB | the robot's own wheel+gyroscope path (/odometry/filtered) in TUM format |
| `s2_static_10/s2_static_10/mapping.log` | 3.70 MB | RTAB-Map's own log for the run (loop closures accepted/rejected, saves, warnings) |
| `s2_static_10/s2_static_10/media/s2_static_10_timelapse.mp4` | 2.54 MB | timelapse video of the live map page, recorded during the run |
| `s2_static_10/s2_static_10/tegrastats.log` | 867 KB | NVIDIA's once-a-second processor/GPU/memory/power log of the Jetson |
| `s2_static_10/lidar_replay_jetson/done/lidar_ref_f3dof/lidar_map.png` | 174 KB | the LiDAR map's walls (cells 0.15-2.0 m above the base) as a picture / array |
| `s2_static_10/lidar_replay_jetson/done/lidar_ref_f3dof/lidar.tum` | 82 KB | the LiDAR estimate's corrected path (TUM format: time x y z qx qy qz qw), one line per map node |
| `s2_static_10/s2_static_10/monitor.csv` | 71 KB | the live mapping monitor's per-second counters and its last status line |
| `s2_static_10/lidar_replay_jetson/done/lidar_ref_f3dof/lidar_map.npz` | 59 KB | the LiDAR map's walls (cells 0.15-2.0 m above the base) as a picture / array |
| `s2_static_10/s2_static_10/camera.log` | 21 KB | the ZED camera program's log |
| `s2_static_10/s2_static_10/fused_odometry.log` | 15 KB | the blend (EKF) and its input conditioner's log, incl. the kept gyroscope offset |
| `s2_static_10/s2_static_10/bridge_recv.log` | 14 KB | the robot-to-Jetson link receiver's log (delivered share of the robot's readings) |
| `s2_static_10/s2_static_10/autostop.log` | 10 KB | the stop-on-'park' helper's log |
| `s2_static_10/s2_static_10/camera_guard.out` | 593 B | the camera guard: restarts, outages (start, end, seconds, cause, outcome) |
| `s2_static_10/s2_static_10/camera_guard.log` | 593 B | the camera guard: restarts, outages (start, end, seconds, cause, outcome) |
| `s2_static_10/s2_static_10/camera_guard.json` | 284 B | the camera guard: restarts, outages (start, end, seconds, cause, outcome) |
| `s2_static_10/s2_static_10/monitor_STATUS.txt` | 140 B | the live mapping monitor's per-second counters and its last status line |
| `.../media/` 1632 timelapse frames (png) + index/log | 292.14 MB | the live map page, one picture every 3 s, recorded during the run (the mp4 above is made from them) |
| + 19 small record files (logs, json, csv, txt, pids) | 528 KB | run records: logs, checks, progress lines |

### test_copies  (82 files, 7.28 GB)

| file | size | what it contains |
|---|---|---|
| `test_copies/raw_from_robot/s2_static_02/E1.db` | 746.71 MB | RTAB-Map database: every map snapshot (picture + position), the links between them, and the corrected positions if closed properly |
| `test_copies/raw_from_robot/s2_static_02/Edebias_input.db` | 746.56 MB | RTAB-Map database: every map snapshot (picture + position), the links between them, and the corrected positions if closed properly |
| `test_copies/raw_from_robot/s2_static_02/Ectl_input.db` | 746.56 MB | RTAB-Map database: every map snapshot (picture + position), the links between them, and the corrected positions if closed properly |
| `test_copies/raw_from_robot/s2_static_02/input.db` | 746.56 MB | RTAB-Map database: every map snapshot (picture + position), the links between them, and the corrected positions if closed properly |
| `test_copies/raw_from_robot/s2_static_02/Ectl.db` | 745.96 MB | RTAB-Map database: every map snapshot (picture + position), the links between them, and the corrected positions if closed properly |
| `test_copies/raw_from_robot/s2_static_02/E1b.db` | 745.96 MB | RTAB-Map database: every map snapshot (picture + position), the links between them, and the corrected positions if closed properly |
| `test_copies/raw_from_robot/s2_static_02/E0r.db` | 745.96 MB | RTAB-Map database: every map snapshot (picture + position), the links between them, and the corrected positions if closed properly |
| `test_copies/raw_from_robot/s2_static_02/E0.db` | 745.96 MB | RTAB-Map database: every map snapshot (picture + position), the links between them, and the corrected positions if closed properly |
| `test_copies/raw_from_robot/s2_static_02/E5.db` | 738.00 MB | RTAB-Map database: every map snapshot (picture + position), the links between them, and the corrected positions if closed properly |
| `test_copies/raw_from_robot/s2_static_02/Edebias.db` | 737.08 MB | RTAB-Map database: every map snapshot (picture + position), the links between them, and the corrected positions if closed properly |
| `test_copies/raw_from_robot/s2_static_02/wheel_debiased.tum` | 6.39 MB | a path in TUM format (time x y z qx qy qz qw) |
| `test_copies/raw_from_robot/s2_static_02/camera_corrected.tum` | 50 KB | the camera map's path after the map's corrections (TUM format) |
| + 70 small record files (logs, json, csv, txt, pids) | 2.27 MB | run records: logs, checks, progress lines |

## robot_card_slam_series2/

*The robot's own 128 GB card, folder slam_series2: every <run>_lidar.bag, <run>_gyro_zero.json, replay outputs (<run>/ folders, *_replay.out) and our robot-side tools (tools/). 327 of the card's 354 files (107.1 GiB); the other 27 were lost when the card failed while being copied.*

327 files, 107.11 GB in all.

### other (tests, old scripts)  (50 files, 293.80 MB)

| file | size | what it contains |
|---|---|---|
| `bad_clock_20260926/s2_static_08_lidar.bag` | 218.87 MB | the robot's LiDAR recording: raw laser packets, not expanded clouds. Topics: /helios/packets /ouster/lidar_packets /ouster/imu_packets /tf /tf_static /odometry/filtered /imu/data /husky_velocity_controller/odom /imu/data_raw /helios_pcl/scan /ouster_pcl/scan |
| `chain_test.bag` | 60.24 MB | a ROS bag recording |
| `rosip_test.bag` | 13.61 MB | a ROS bag recording |
| `maps/rtab_helios.db` | 760 KB | the LiDAR map database written by the colleague's replay pipeline (RTAB-Map, LiDAR mode) |
| `tools_carving_test/render_map.py` | 22 KB | one of our tools/scripts (copy kept with the data) |
| `live_view_tests/20260924_073203/map.png` | 18 KB | a figure or photo |
| `live_view_tests/20260924_202516/map.png` | 17 KB | a figure or photo |
| `tools_carving_test/compare_carving_v2.py` | 17 KB | one of our tools/scripts (copy kept with the data) |
| `tools_carving_test/lidar_carve.py` | 14 KB | one of our tools/scripts (copy kept with the data) |
| `tools_carving_test/compare_carving.py` | 11 KB | one of our tools/scripts (copy kept with the data) |
| `tools_carving_test/robot_carving_test.sh` | 8 KB | one of our tools/scripts (copy kept with the data) |
| `replay_lidar.sh` | 4 KB | one of our tools/scripts (copy kept with the data) |
| `record_lidar.sh` | 2 KB | one of our tools/scripts (copy kept with the data) |
| `bad_clock_20260926/s2_static_08_gyro_zero.json` | 174 B | the robot gyroscope's standing-still bias measured before the drive |
| + 36 small record files (logs, json, csv, txt, pids) | 237 KB | run records: logs, checks, progress lines |

### s2_fusion_T5  (2 files, 6.60 GB)

| | |
|---|---|
| date (Hamilton time) | 25 Sep 2026, ~23:00 |
| what | **parked rehearsal** - T5: killed by a Jetson hard reset at 23:39 |
| why it was driven | rehearse drive 4's new receiver and link-drop guard |
| scored in | `- (see PROGRESS.html 26 Sep 00:54)` |

| file | size | what it contains |
|---|---|---|
| `s2_fusion_T5_lidar.bag` | 6.60 GB | the robot's LiDAR recording: raw laser packets, not expanded clouds. Topics: /helios/packets /ouster/lidar_packets /ouster/imu_packets /tf /tf_static /odometry/filtered /imu/data /husky_velocity_controller/odom /imu/data_raw /helios_pcl/scan /ouster_pcl/scan |
| `s2_fusion_T5_gyro_zero.json` | 174 B | the robot gyroscope's standing-still bias measured before the drive |

### s2_fusion_T5b  (2 files, 5.13 GB)

| | |
|---|---|
| date (Hamilton time) | 25-26 Sep 2026, ~23:49-00:07 |
| what | **parked rehearsal** - T5b: link drop A1/A2, ZED gyroscope in the blend; 16.5 min |
| why it was driven | rehearse drive 4; found 10 camera freezes in 16.5 min |
| scored in | `- (see PROGRESS.html 26 Sep 00:54)` |

| file | size | what it contains |
|---|---|---|
| `s2_fusion_T5b_lidar.bag` | 5.13 GB | the robot's LiDAR recording: raw laser packets, not expanded clouds. Topics: /helios/packets /ouster/lidar_packets /ouster/imu_packets /tf /tf_static /odometry/filtered /imu/data /husky_velocity_controller/odom /imu/data_raw /helios_pcl/scan /ouster_pcl/scan |
| `s2_fusion_T5b_gyro_zero.json` | 174 B | the robot gyroscope's standing-still bias measured before the drive |

### s2_fusion_T5c  (2 files, 4.17 GB)

| | |
|---|---|
| date (Hamilton time) | 26 Sep 2026, ~00:26-00:41 |
| what | **parked rehearsal** - T5c: 16-bit depth in the map; 9.7 min |
| why it was driven | does 16-bit depth cut the freezes? (2 in 9.7 min); camera crashed while recording |
| scored in | `- (docs/SOLVED.md '16-bit depth cuts the camera freezes')` |

| file | size | what it contains |
|---|---|---|
| `s2_fusion_T5c_lidar.bag` | 4.17 GB | the robot's LiDAR recording: raw laser packets, not expanded clouds. Topics: /helios/packets /ouster/lidar_packets /ouster/imu_packets /tf /tf_static /odometry/filtered /imu/data /husky_velocity_controller/odom /imu/data_raw /helios_pcl/scan /ouster_pcl/scan |
| `s2_fusion_T5c_gyro_zero.json` | 174 B | the robot gyroscope's standing-still bias measured before the drive |

### s2_loc_01  (2 files, 2.63 GB)

| | |
|---|---|
| date (Hamilton time) | 26 Sep 2026, 05:03-05:10 |
| what | **localisation session 1** - drive 4's map, mapping off; lost at stop 1 |
| why it was driven | first localisation attempt; robot battery not charged enough |
| scored in | `- (03_methods/localisation_demo_2026-09-26/RESULTS.md, session 1)` |

| file | size | what it contains |
|---|---|---|
| `s2_loc_01_lidar.bag` | 2.63 GB | the robot's LiDAR recording: raw laser packets, not expanded clouds. Topics: /helios/packets /ouster/lidar_packets /ouster/imu_packets /tf /tf_static /odometry/filtered /imu/data /husky_velocity_controller/odom /imu/data_raw /helios_pcl/scan /ouster_pcl/scan |
| `s2_loc_01_gyro_zero.json` | 174 B | the robot gyroscope's standing-still bias measured before the drive |

### s2_loc_02  (20 files, 606.09 MB)

| | |
|---|---|
| date (Hamilton time) | 26 Sep 2026, 06:16-06:34 |
| what | **localisation session 2** - drive 4's map; 5 starts from the saved start guess |
| why it was driven | does the robot find itself on a saved map? (1 of 5; 4 shown 4-10 m wrong) |
| scored in | `results/series2_static/s2_loc_02/RESULTS.md` |

| file | size | what it contains |
|---|---|---|
| `s2_loc_02/rtab_helios.db` | 220.40 MB | the LiDAR map database written by the colleague's replay pipeline (RTAB-Map, LiDAR mode) |
| `s2_loc_02.partial-20260926/rtab_helios.db` | 209.46 MB | the LiDAR map database written by the colleague's replay pipeline (RTAB-Map, LiDAR mode) |
| `s2_loc_02/play.log` | 85.27 MB | a small record file (numbers, log or provenance) |
| `s2_loc_02.partial-20260926/play.log` | 84.21 MB | a small record file (numbers, log or provenance) |
| `s2_loc_02/wheel.tum` | 5.53 MB | the robot's own wheel+gyroscope path (/odometry/filtered) in TUM format |
| `s2_loc_02/lidar_map.png` | 68 KB | the LiDAR map's walls (cells 0.15-2.0 m above the base) as a picture / array |
| `s2_loc_02/lidar.tum` | 22 KB | the LiDAR estimate's corrected path (TUM format: time x y z qx qy qz qw), one line per map node |
| `s2_loc_02/lidar_map.npz` | 17 KB | the LiDAR map's walls (cells 0.15-2.0 m above the base) as a picture / array |
| `s2_loc_02_replay.out` | 188 B | the robot-side LiDAR replay's screen output (one line: which files it produced) |
| `s2_loc_02_gyro_zero.json` | 174 B | the robot gyroscope's standing-still bias measured before the drive |
| + 10 small record files (logs, json, csv, txt, pids) | 1.12 MB | run records: logs, checks, progress lines |

### s2_loc_03  (16 files, 8.82 GB)

| | |
|---|---|
| date (Hamilton time) | 26 Sep 2026, 10:41-11:06 |
| what | **localisation session 3** - drive 4's map; 5 starts, each told it is 40 m off the map |
| why it was driven | off-map start guess + LoopThr 0.08: finds itself at 4 of 5 starts, never shows a wrong place |
| scored in | `results/series2_static/s2_loc_03/RESULTS.md` |

| file | size | what it contains |
|---|---|---|
| `s2_loc_03_lidar.bag` | 8.45 GB | the robot's LiDAR recording: raw laser packets, not expanded clouds. Topics: /helios/packets /ouster/lidar_packets /ouster/imu_packets /tf /tf_static /odometry/filtered /imu/data /husky_velocity_controller/odom /imu/data_raw /helios_pcl/scan /ouster_pcl/scan |
| `s2_loc_03/rtab_helios.db` | 220.29 MB | the LiDAR map database written by the colleague's replay pipeline (RTAB-Map, LiDAR mode) |
| `s2_loc_03/play.log` | 150.43 MB | a small record file (numbers, log or provenance) |
| `s2_loc_03/wheel.tum` | 8.34 MB | the robot's own wheel+gyroscope path (/odometry/filtered) in TUM format |
| `s2_loc_03/lidar_map.png` | 70 KB | the LiDAR map's walls (cells 0.15-2.0 m above the base) as a picture / array |
| `s2_loc_03/lidar.tum` | 22 KB | the LiDAR estimate's corrected path (TUM format: time x y z qx qy qz qw), one line per map node |
| `s2_loc_03/lidar_map.npz` | 20 KB | the LiDAR map's walls (cells 0.15-2.0 m above the base) as a picture / array |
| `s2_loc_03_replay.out` | 189 B | the robot-side LiDAR replay's screen output (one line: which files it produced) |
| `s2_loc_03_gyro_zero.json` | 174 B | the robot gyroscope's standing-still bias measured before the drive |
| + 7 small record files (logs, json, csv, txt, pids) | 431 KB | run records: logs, checks, progress lines |

### s2_loc_04  (8 files, 889.02 MB)

| | |
|---|---|
| date (Hamilton time) | 27 Sep 2026, 07:43-08:08 |
| what | **whole-floor localisation** - drive 10's map, mapping off, told it starts 40 m outside the building; the floor driven once |
| why it was driven | pass lines W1-W4: 0 of 575 fixes wrong; first fix 212 s |
| scored in | `results/series2_static/s2_loc_04/RESULTS.md` |

| file | size | what it contains |
|---|---|---|
| `s2_loc_04/lidar_ref_f3dof/rtab_helios.db` | 742.93 MB | the LiDAR map database written by the colleague's replay pipeline (RTAB-Map, LiDAR mode) |
| `s2_loc_04/lidar_ref_f3dof/play.log` | 144.56 MB | a small record file (numbers, log or provenance) |
| `s2_loc_04_gyro_zero.json` | 174 B | the robot gyroscope's standing-still bias measured before the drive |
| `s2_loc_04_f3dof_replay.out` | 0 B | the robot-side LiDAR replay's screen output (one line: which files it produced) |
| + 4 small record files (logs, json, csv, txt, pids) | 1.53 MB | run records: logs, checks, progress lines |

### s2_nav_01  (2 files, 8.63 GB)

| | |
|---|---|
| date (Hamilton time) | 27 Sep 2026, ~16:14-16:46 |
| what | **navigation run 1** - route A (corridor out and back) 2 of 2 goals; route B stopped after a wall-post touch |
| why it was driven | first autonomous driving on drive 10's map |
| scored in | `results/navigation/s2_nav_01/ (nav_score.json, USER_NOTES.txt)` |

| file | size | what it contains |
|---|---|---|
| `s2_nav_01_lidar.bag` | 8.63 GB | the robot's LiDAR recording: raw laser packets, not expanded clouds. Topics: /helios/packets /ouster/lidar_packets /ouster/imu_packets /tf /tf_static /odometry/filtered /imu/data /husky_velocity_controller/odom /imu/data_raw /helios_pcl/scan /ouster_pcl/scan |
| `s2_nav_01_gyro_zero.json` | 174 B | the robot gyroscope's standing-still bias measured before the drive |

### s2_nav_02  (2 files, 5.87 GB)

| | |
|---|---|
| date (Hamilton time) | 27 Sep 2026, ~17:31-17:53 |
| what | **navigation run 2** - route B2 around the loop: 4 of 5 goals, no contact |
| why it was driven | with the local walls + clearance guard fixes after run 1 |
| scored in | `results/navigation/s2_nav_02/ (goals_result.json)` |

| file | size | what it contains |
|---|---|---|
| `s2_nav_02_lidar.bag` | 5.87 GB | the robot's LiDAR recording: raw laser packets, not expanded clouds. Topics: /helios/packets /ouster/lidar_packets /ouster/imu_packets /tf /tf_static /odometry/filtered /imu/data /husky_velocity_controller/odom /imu/data_raw /helios_pcl/scan /ouster_pcl/scan |
| `s2_nav_02_gyro_zero.json` | 174 B | the robot gyroscope's standing-still bias measured before the drive |

### s2_nav_03  (2 files, 5.51 GB)

| | |
|---|---|
| date (Hamilton time) | 27 Sep 2026, 18:53-19:03 |
| what | **navigation run 3** - route B2 full round trip: 4 of 5 goals on its own, 1.5 m straight through the doorway by command |
| why it was driven | the final navigation result |
| scored in | `results/navigation/s2_nav_03/RESULTS.md` |

| file | size | what it contains |
|---|---|---|
| `s2_nav_03_lidar.bag` | 5.51 GB | the robot's LiDAR recording: raw laser packets, not expanded clouds. Topics: /helios/packets /ouster/lidar_packets /ouster/imu_packets /tf /tf_static /odometry/filtered /imu/data /husky_velocity_controller/odom /imu/data_raw /helios_pcl/scan /ouster_pcl/scan |
| `s2_nav_03_gyro_zero.json` | 174 B | the robot gyroscope's standing-still bias measured before the drive |

### s2_scale_01  (2 files, 3.54 GB)

| | |
|---|---|
| date (Hamilton time) | 26 Sep 2026, 13:10-13:22 |
| what | **taped 15.00 m distance test** - 3 round trips between two taped marks |
| why it was driven | which ruler is honest? camera 14.90 m (-0.7 %), wheels 15.94 m (+6.2 %) |
| scored in | `results/series2_static/s2_scale_01/RESULTS.md` |

| file | size | what it contains |
|---|---|---|
| `s2_scale_01_lidar.bag` | 3.54 GB | the robot's LiDAR recording: raw laser packets, not expanded clouds. Topics: /helios/packets /ouster/lidar_packets /ouster/imu_packets /tf /tf_static /odometry/filtered /imu/data /husky_velocity_controller/odom /imu/data_raw /helios_pcl/scan /ouster_pcl/scan |
| `s2_scale_01_gyro_zero.json` | 174 B | the robot gyroscope's standing-still bias measured before the drive |

### s2_slam_01  (2 files, 8.32 GB)

| | |
|---|---|
| date (Hamilton time) | 27 Sep 2026, 13:25-13:52 |
| what | **SLAM session** - continued drive 10's map; 156 m; 73 m2 of new floor; people mask test |
| why it was driven | full SLAM on an existing map (S1-S5) and the people-blanking replay |
| scored in | `results/series2_slam/s2_slam_01/RESULTS.md` |

| file | size | what it contains |
|---|---|---|
| `s2_slam_01_lidar.bag` | 8.32 GB | the robot's LiDAR recording: raw laser packets, not expanded clouds. Topics: /helios/packets /ouster/lidar_packets /ouster/imu_packets /tf /tf_static /odometry/filtered /imu/data /husky_velocity_controller/odom /imu/data_raw /helios_pcl/scan /ouster_pcl/scan |
| `s2_slam_01_gyro_zero.json` | 174 B | the robot gyroscope's standing-still bias measured before the drive |

### s2_static_03  (34 files, 8.87 GB)

| | |
|---|---|
| date (Hamilton time) | 25 Sep 2026, 04:30-04:58 |
| what | **walkway drive 3** - first fused drive (camera + wheels + gyroscope); 140 m, 28 min |
| why it was driven | does the blend stop the camera's tracking losses from throwing away distance? |
| scored in | `results/series2_static/s2_static_03/RESULTS.md` |

| file | size | what it contains |
|---|---|---|
| `s2_static_03_lidar.bag` | 8.20 GB | the robot's LiDAR recording: raw laser packets, not expanded clouds. Topics: /helios/packets /ouster/lidar_packets /ouster/imu_packets /tf /tf_static /odometry/filtered /imu/data /husky_velocity_controller/odom /imu/data_raw /helios_pcl/scan /ouster_pcl/scan |
| `s2_static_03/rtab_helios.db` | 564.14 MB | the LiDAR map database written by the colleague's replay pipeline (RTAB-Map, LiDAR mode) |
| `s2_static_03/play.log` | 120.04 MB | a small record file (numbers, log or provenance) |
| `s2_static_03/wheel.tum` | 8.04 MB | the robot's own wheel+gyroscope path (/odometry/filtered) in TUM format |
| `s2_static_03/carving_test_2026-09-26_0145/carving_before_after.png` | 127 KB | a figure or photo |
| `s2_static_03/lidar_map.png` | 114 KB | the LiDAR map's walls (cells 0.15-2.0 m above the base) as a picture / array |
| `s2_static_03/carving_test_2026-09-26_0145/B/lidar_map.png` | 107 KB | the LiDAR map's walls (cells 0.15-2.0 m above the base) as a picture / array |
| `s2_static_03/lidar.tum` | 56 KB | the LiDAR estimate's corrected path (TUM format: time x y z qx qy qz qw), one line per map node |
| `s2_static_03/c3/lidar.tum` | 56 KB | the LiDAR estimate's corrected path (TUM format: time x y z qx qy qz qw), one line per map node |
| `s2_static_03/carving_test_2026-09-26_0145/B/lidar_map.npz` | 36 KB | the LiDAR map's walls (cells 0.15-2.0 m above the base) as a picture / array |
| `s2_static_03/lidar_map.npz` | 36 KB | the LiDAR map's walls (cells 0.15-2.0 m above the base) as a picture / array |
| `s2_static_03/c3/lidar_map.npz` | 36 KB | the LiDAR map's walls (cells 0.15-2.0 m above the base) as a picture / array |
| `s2_static_03/carving_diag_2026-09-26_0151/diag_cells.py` | 5 KB | one of our tools/scripts (copy kept with the data) |
| `s2_static_03_gyro_zero.json` | 174 B | the robot gyroscope's standing-still bias measured before the drive |
| + 20 small record files (logs, json, csv, txt, pids) | 1.57 MB | run records: logs, checks, progress lines |

### s2_static_05  (24 files, 7.80 GB)

| | |
|---|---|
| date (Hamilton time) | 26 Sep 2026, 02:14-02:35 |
| what | **walkway drive 5** - longer loop; 61 m mapped before the camera crashed at 11.2 min |
| why it was driven | disk-save waits off; a person behind the robot for the LiDAR clearing test; camera program reused from drive 4 and crashed |
| scored in | `results/series2_static/s2_static_05/RESULTS.md` |

| file | size | what it contains |
|---|---|---|
| `s2_static_05_lidar.bag` | 7.06 GB | the robot's LiDAR recording: raw laser packets, not expanded clouds. Topics: /helios/packets /ouster/lidar_packets /ouster/imu_packets /tf /tf_static /odometry/filtered /imu/data /husky_velocity_controller/odom /imu/data_raw /helios_pcl/scan /ouster_pcl/scan |
| `s2_static_05/rtab_helios.db` | 626.16 MB | the LiDAR map database written by the colleague's replay pipeline (RTAB-Map, LiDAR mode) |
| `s2_static_05/play.log` | 122.42 MB | a small record file (numbers, log or provenance) |
| `s2_static_05/wheel.tum` | 6.96 MB | the robot's own wheel+gyroscope path (/odometry/filtered) in TUM format |
| `s2_static_05/carving_test_2026-09-26_0307/carving_before_after.png` | 131 KB | a figure or photo |
| `s2_static_05/lidar_map.png` | 129 KB | the LiDAR map's walls (cells 0.15-2.0 m above the base) as a picture / array |
| `s2_static_05/lidar.tum` | 62 KB | the LiDAR estimate's corrected path (TUM format: time x y z qx qy qz qw), one line per map node |
| `s2_static_05/lidar_map.npz` | 43 KB | the LiDAR map's walls (cells 0.15-2.0 m above the base) as a picture / array |
| `s2_static_05_gyro_zero.json` | 174 B | the robot gyroscope's standing-still bias measured before the drive |
| + 15 small record files (logs, json, csv, txt, pids) | 538 KB | run records: logs, checks, progress lines |

### s2_static_06  (31 files, 927.62 MB)

| | |
|---|---|
| date (Hamilton time) | 26 Sep 2026, 04:20-04:45 |
| what | **walkway drive 6** - new walkway added; 238 m, 25 min |
| why it was driven | first drive with the camera guard (restart in 12.7 s); LiDAR map tilted -> Force3DoF replay |
| scored in | `results/series2_static/s2_static_06/RESULTS.md` |

| file | size | what it contains |
|---|---|---|
| `s2_static_06/rtab_helios.db` | 799.24 MB | the LiDAR map database written by the colleague's replay pipeline (RTAB-Map, LiDAR mode) |
| `s2_static_06/play.log` | 117.99 MB | a small record file (numbers, log or provenance) |
| `s2_static_06/wheel.tum` | 7.33 MB | the robot's own wheel+gyroscope path (/odometry/filtered) in TUM format |
| `s2_static_06/lidar_map.png` | 169 KB | the LiDAR map's walls (cells 0.15-2.0 m above the base) as a picture / array |
| `s2_static_06/lidar_ref_f3dof/lidar_map.png` | 149 KB | the LiDAR map's walls (cells 0.15-2.0 m above the base) as a picture / array |
| `s2_static_06/lidar.tum` | 80 KB | the LiDAR estimate's corrected path (TUM format: time x y z qx qy qz qw), one line per map node |
| `s2_static_06/lidar_ref_f3dof/lidar.tum` | 79 KB | the LiDAR estimate's corrected path (TUM format: time x y z qx qy qz qw), one line per map node |
| `s2_static_06/lidar_map.npz` | 63 KB | the LiDAR map's walls (cells 0.15-2.0 m above the base) as a picture / array |
| `s2_static_06/lidar_ref_f3dof/lidar_map.npz` | 53 KB | the LiDAR map's walls (cells 0.15-2.0 m above the base) as a picture / array |
| `s2_static_06_f3dof_replay.out` | 202 B | the robot-side LiDAR replay's screen output (one line: which files it produced) |
| `s2_static_06_gyro_zero.json` | 174 B | the robot gyroscope's standing-still bias measured before the drive |
| + 20 small record files (logs, json, csv, txt, pids) | 2.49 MB | run records: logs, checks, progress lines |

### s2_static_07  (16 files, 4.88 GB)

| | |
|---|---|
| date (Hamilton time) | 26 Sep 2026, 13:28-13:40 |
| what | **walkway drive 7** - drive 4's corridors a bit further, camera tracker GEN_1; 107 m, 14 min |
| why it was driven | does the older tracker stop the crashes? (no crash, but 16 freezes and only 30 closures) |
| scored in | `results/series2_static/s2_static_07/RESULTS.md` |

| file | size | what it contains |
|---|---|---|
| `s2_static_07_lidar.bag` | 4.28 GB | the robot's LiDAR recording: raw laser packets, not expanded clouds. Topics: /helios/packets /ouster/lidar_packets /ouster/imu_packets /tf /tf_static /odometry/filtered /imu/data /husky_velocity_controller/odom /imu/data_raw /helios_pcl/scan /ouster_pcl/scan |
| `s2_static_07/rtab_helios.db` | 530.26 MB | the LiDAR map database written by the colleague's replay pipeline (RTAB-Map, LiDAR mode) |
| `s2_static_07/play.log` | 71.99 MB | a small record file (numbers, log or provenance) |
| `s2_static_07/wheel.tum` | 4.20 MB | the robot's own wheel+gyroscope path (/odometry/filtered) in TUM format |
| `s2_static_07/lidar_map.png` | 125 KB | the LiDAR map's walls (cells 0.15-2.0 m above the base) as a picture / array |
| `s2_static_07/lidar.tum` | 53 KB | the LiDAR estimate's corrected path (TUM format: time x y z qx qy qz qw), one line per map node |
| `s2_static_07/lidar_map.npz` | 33 KB | the LiDAR map's walls (cells 0.15-2.0 m above the base) as a picture / array |
| `s2_static_07_replay.out` | 194 B | the robot-side LiDAR replay's screen output (one line: which files it produced) |
| `s2_static_07_gyro_zero.json` | 174 B | the robot gyroscope's standing-still bias measured before the drive |
| + 7 small record files (logs, json, csv, txt, pids) | 313 KB | run records: logs, checks, progress lines |

### s2_static_08  (33 files, 4.84 GB)

| | |
|---|---|
| date (Hamilton time) | 26 Sep 2026, 15:48-16:02 |
| what | **walkway drive 8** - drive 4's route repeated, GEN_2; 87 m, 14 min |
| why it was driven | the 'same result every time' test: 0.10 m home, 0.26 m from the LiDAR as on drive 4 |
| scored in | `results/series2_static/s2_static_08/RESULTS.md` |

| file | size | what it contains |
|---|---|---|
| `s2_static_08_lidar.bag` | 4.24 GB | the robot's LiDAR recording: raw laser packets, not expanded clouds. Topics: /helios/packets /ouster/lidar_packets /ouster/imu_packets /tf /tf_static /odometry/filtered /imu/data /husky_velocity_controller/odom /imu/data_raw /helios_pcl/scan /ouster_pcl/scan |
| `s2_static_08/rtab_helios.db` | 469.09 MB | the LiDAR map database written by the colleague's replay pipeline (RTAB-Map, LiDAR mode) |
| `s2_static_08/play.log` | 70.96 MB | a small record file (numbers, log or provenance) |
| `s2_static_08/lidar_ref_f3dof/play.log` | 66.29 MB | a small record file (numbers, log or provenance) |
| `s2_static_08/wheel.tum` | 4.19 MB | the robot's own wheel+gyroscope path (/odometry/filtered) in TUM format |
| `s2_static_08/lidar_ref_f3dof/wheel.tum` | 4.19 MB | the robot's own wheel+gyroscope path (/odometry/filtered) in TUM format |
| `s2_static_08/lidar_map.png` | 118 KB | the LiDAR map's walls (cells 0.15-2.0 m above the base) as a picture / array |
| `s2_static_08/lidar_ref_f3dof/lidar_map.png` | 71 KB | the LiDAR map's walls (cells 0.15-2.0 m above the base) as a picture / array |
| `s2_static_08/lidar_map.npz` | 49 KB | the LiDAR map's walls (cells 0.15-2.0 m above the base) as a picture / array |
| `s2_static_08/lidar.tum` | 47 KB | the LiDAR estimate's corrected path (TUM format: time x y z qx qy qz qw), one line per map node |
| `s2_static_08/lidar_ref_f3dof/lidar.tum` | 46 KB | the LiDAR estimate's corrected path (TUM format: time x y z qx qy qz qw), one line per map node |
| `s2_static_08/lidar_ref_f3dof/lidar_map.npz` | 24 KB | the LiDAR map's walls (cells 0.15-2.0 m above the base) as a picture / array |
| `s2_static_08_f3dof_replay.out` | 201 B | the robot-side LiDAR replay's screen output (one line: which files it produced) |
| `s2_static_08_replay.out` | 195 B | the robot-side LiDAR replay's screen output (one line: which files it produced) |
| `s2_static_08_gyro_zero.json` | 174 B | the robot gyroscope's standing-still bias measured before the drive |
| + 18 small record files (logs, json, csv, txt, pids) | 923 KB | run records: logs, checks, progress lines |

### s2_static_09  (16 files, 9.36 GB)

| | |
|---|---|
| date (Hamilton time) | 26 Sep 2026, 20:52-21:21 |
| what | **walkway drive 9** - the first longer route, fresh Jetson restart; 252 m by its map nodes (tracking alone), 29 min; map NOT closed (WiFi lost) |
| why it was driven | 3x drive 4's distance; found the 5-minute WiFi roaming timer |
| scored in | `results/series2_static/s2_static_09/RESULTS.md` |

| file | size | what it contains |
|---|---|---|
| `s2_static_09_lidar.bag` | 9.20 GB | the robot's LiDAR recording: raw laser packets, not expanded clouds. Topics: /helios/packets /ouster/lidar_packets /ouster/imu_packets /tf /tf_static /odometry/filtered /imu/data /husky_velocity_controller/odom /imu/data_raw /helios_pcl/scan /ouster_pcl/scan |
| `s2_static_09/lidar_ref_f3dof/play.log` | 156.90 MB | a small record file (numbers, log or provenance) |
| `s2_static_09/lidar_ref_f3dof/wheel.tum` | 8.96 MB | the robot's own wheel+gyroscope path (/odometry/filtered) in TUM format |
| `s2_static_09/lidar_ref_f3dof/lidar_map.png` | 138 KB | the LiDAR map's walls (cells 0.15-2.0 m above the base) as a picture / array |
| `s2_static_09/lidar_ref_f3dof/lidar.tum` | 100 KB | the LiDAR estimate's corrected path (TUM format: time x y z qx qy qz qw), one line per map node |
| `s2_static_09/lidar_ref_f3dof/lidar_map.npz` | 48 KB | the LiDAR map's walls (cells 0.15-2.0 m above the base) as a picture / array |
| `s2_static_09_f3dof_replay.out` | 203 B | the robot-side LiDAR replay's screen output (one line: which files it produced) |
| `s2_static_09_gyro_zero.json` | 174 B | the robot gyroscope's standing-still bias measured before the drive |
| + 8 small record files (logs, json, csv, txt, pids) | 702 KB | run records: logs, checks, progress lines |

### s2_static_10  (17 files, 9.48 GB)

| | |
|---|---|
| date (Hamilton time) | 27 Sep 2026, 02:02-02:31 |
| what | **walkway drive 10** - the WHOLE second floor; 258 m by its map nodes (tracking alone; 252 m corrected), 29 min; closed properly |
| why it was driven | the map everything after it uses (localisation, SLAM session, navigation) |
| scored in | `results/series2_static/s2_static_10/RESULTS.md` |

| file | size | what it contains |
|---|---|---|
| `s2_static_10_lidar.bag` | 8.54 GB | the robot's LiDAR recording: raw laser packets, not expanded clouds. Topics: /helios/packets /ouster/lidar_packets /ouster/imu_packets /tf /tf_static /odometry/filtered /imu/data /husky_velocity_controller/odom /imu/data_raw /helios_pcl/scan /ouster_pcl/scan |
| `s2_static_10/lidar_ref_f3dof/rtab_helios.db` | 815.39 MB | the LiDAR map database written by the colleague's replay pipeline (RTAB-Map, LiDAR mode) |
| `s2_static_10/lidar_ref_f3dof/play.log` | 133.25 MB | a small record file (numbers, log or provenance) |
| `s2_static_10/lidar_ref_f3dof/wheel.tum` | 8.41 MB | the robot's own wheel+gyroscope path (/odometry/filtered) in TUM format |
| `s2_static_10/lidar_ref_f3dof/lidar_map.png` | 169 KB | the LiDAR map's walls (cells 0.15-2.0 m above the base) as a picture / array |
| `s2_static_10/lidar_ref_f3dof/lidar.tum` | 81 KB | the LiDAR estimate's corrected path (TUM format: time x y z qx qy qz qw), one line per map node |
| `s2_static_10/lidar_ref_f3dof/lidar_map.npz` | 61 KB | the LiDAR map's walls (cells 0.15-2.0 m above the base) as a picture / array |
| `s2_static_10_f3dof_replay.out` | 201 B | the robot-side LiDAR replay's screen output (one line: which files it produced) |
| `s2_static_10_gyro_zero.json` | 174 B | the robot gyroscope's standing-still bias measured before the drive |
| + 8 small record files (logs, json, csv, txt, pids) | 1.49 MB | run records: logs, checks, progress lines |

### tools  (44 files, 516 KB)

| file | size | what it contains |
|---|---|---|
| `tools/lidar_live_map.py` | 35 KB | one of our tools/scripts (copy kept with the data) |
| `tools/odom_debias.py` | 33 KB | one of our tools/scripts (copy kept with the data) |
| `tools/replay_colleague_variant.sh` | 26 KB | one of our tools/scripts (copy kept with the data) |
| `tools/replay_lidar.sh` | 25 KB | one of our tools/scripts (copy kept with the data) |
| `tools/robot_storage_agent.py` | 22 KB | one of our tools/scripts (copy kept with the data) |
| `tools/render_map.py` | 22 KB | one of our tools/scripts (copy kept with the data) |
| `tools/robot_bridge_send.py` | 21 KB | one of our tools/scripts (copy kept with the data) |
| `tools/db_rewrite_poses.py` | 20 KB | one of our tools/scripts (copy kept with the data) |
| `tools/nav_cmd_recv.py` | 17 KB | one of our tools/scripts (copy kept with the data) |
| `tools/lidar_carve.py` | 14 KB | one of our tools/scripts (copy kept with the data) |
| `tools/cpu_sampler.py` | 13 KB | one of our tools/scripts (copy kept with the data) |
| `tools/compare_carving.py` | 11 KB | one of our tools/scripts (copy kept with the data) |
| `tools/gyro_rezero_test.py` | 10 KB | one of our tools/scripts (copy kept with the data) |
| `tools/robot_side.sh` | 9 KB | one of our tools/scripts (copy kept with the data) |
| `tools/stageA_closure_check.py` | 8 KB | one of our tools/scripts (copy kept with the data) |
| `tools/tf_one_parent_check.py` | 8 KB | one of our tools/scripts (copy kept with the data) |
| `tools/wifi_roam_helper.py` | 8 KB | one of our tools/scripts (copy kept with the data) |
| `tools/optimize_graph_se2.py` | 7 KB | one of our tools/scripts (copy kept with the data) |
| `tools/live_view_stationary_test.sh` | 6 KB | one of our tools/scripts (copy kept with the data) |
| `tools/db_corrected_tum.py` | 6 KB | one of our tools/scripts (copy kept with the data) |
| `tools/bag_to_tum.py` | 5 KB | one of our tools/scripts (copy kept with the data) |
| `tools/helios_channel_check.py` | 5 KB | one of our tools/scripts (copy kept with the data) |
| `tools/stageA_metrics.py` | 5 KB | one of our tools/scripts (copy kept with the data) |
| `tools/stageA_robot.sh` | 5 KB | one of our tools/scripts (copy kept with the data) |
| `tools/card_pattern_test.py` | 4 KB | one of our tools/scripts (copy kept with the data) |
| `tools/check_closed.py` | 4 KB | one of our tools/scripts (copy kept with the data) |
| `tools/c3_robot.sh` | 4 KB | one of our tools/scripts (copy kept with the data) |
| `tools/list_lidar_links.py` | 4 KB | one of our tools/scripts (copy kept with the data) |
| `tools/record_lidar.sh` | 3 KB | one of our tools/scripts (copy kept with the data) |
| `tools/robot_battery_reader.py` | 3 KB | one of our tools/scripts (copy kept with the data) |
| `tools/gyro_zero_now.py` | 3 KB | one of our tools/scripts (copy kept with the data) |
| `tools/live_gap_check.py` | 3 KB | one of our tools/scripts (copy kept with the data) |
| `tools/colleague_variants/f3dof.launch` | 1 KB | one of our tools/scripts (copy kept with the data) |
| `tools/install_wifi_roam_helper.sh` | 1003 B | one of our tools/scripts (copy kept with the data) |
| + 10 small record files (logs, json, csv, txt, pids) | 146 KB | run records: logs, checks, progress lines |

## paper_zedx_vs_lidar/

*The camera depth study data: how precisely the ZED X camera measures distance, checked against LiDAR (laser scanner) measurements, station by station. CSV results, logs and the study's tools.*

835 files, 280.00 MB in all.

### (top level)  (9 files, 260 KB)

| file | size | what it contains |
|---|---|---|
| + 9 small record files (logs, json, csv, txt, pids) | 260 KB | run records: logs, checks, progress lines |

### 00_references  (6 files, 11.57 MB)

| file | size | what it contains |
|---|---|---|
| `00_references/sensors-22-07146.pdf` | 10.19 MB | - |
| + 5 small record files (logs, json, csv, txt, pids) | 1.38 MB | run records: logs, checks, progress lines |

### 01_nicolas_code  (11 files, 9 KB)

| file | size | what it contains |
|---|---|---|
| + 11 small record files (logs, json, csv, txt, pids) | 9 KB | run records: logs, checks, progress lines |

### 02_zedx_pipeline  (127 files, 1.60 MB)

| file | size | what it contains |
|---|---|---|
| `02_zedx_pipeline/campaign_dashboard.py` | 188 KB | one of our tools/scripts (copy kept with the data) |
| `02_zedx_pipeline/station_panel.py` | 30 KB | one of our tools/scripts (copy kept with the data) |
| `02_zedx_pipeline/jobs_dashboard.py` | 28 KB | one of our tools/scripts (copy kept with the data) |
| `02_zedx_pipeline/s4_detect.py` | 25 KB | one of our tools/scripts (copy kept with the data) |
| `02_zedx_pipeline/make_setup_illustrations.py` | 19 KB | one of our tools/scripts (copy kept with the data) |
| `02_zedx_pipeline/s2_panel.py` | 18 KB | one of our tools/scripts (copy kept with the data) |
| `02_zedx_pipeline/s4_autopilot.sh` | 17 KB | one of our tools/scripts (copy kept with the data) |
| `02_zedx_pipeline/s2_boxcheck_compare.py` | 17 KB | one of our tools/scripts (copy kept with the data) |
| `02_zedx_pipeline/pass_record.sh` | 17 KB | one of our tools/scripts (copy kept with the data) |
| `02_zedx_pipeline/s3_laser.sh` | 16 KB | one of our tools/scripts (copy kept with the data) |
| `02_zedx_pipeline/s3_cam_locate.py` | 15 KB | one of our tools/scripts (copy kept with the data) |
| `02_zedx_pipeline/s3_sphere_replay.py` | 14 KB | one of our tools/scripts (copy kept with the data) |
| `02_zedx_pipeline/s3_compare.py` | 14 KB | one of our tools/scripts (copy kept with the data) |
| `02_zedx_pipeline/s2_pull.sh` | 13 KB | one of our tools/scripts (copy kept with the data) |
| `02_zedx_pipeline/offline_campaign_4m_v2.sh` | 13 KB | one of our tools/scripts (copy kept with the data) |
| `02_zedx_pipeline/s3_results_md.py` | 13 KB | one of our tools/scripts (copy kept with the data) |
| `02_zedx_pipeline/s4_results_md.py` | 11 KB | one of our tools/scripts (copy kept with the data) |
| `02_zedx_pipeline/offline_campaign_4m.sh` | 11 KB | one of our tools/scripts (copy kept with the data) |
| `02_zedx_pipeline/campaign_stage.sh` | 11 KB | one of our tools/scripts (copy kept with the data) |
| `02_zedx_pipeline/campaign_worker_test.sh` | 11 KB | one of our tools/scripts (copy kept with the data) |
| `02_zedx_pipeline/mainonly_progress_publisher.py` | 11 KB | one of our tools/scripts (copy kept with the data) |
| `02_zedx_pipeline/s3_deskew.py` | 10 KB | one of our tools/scripts (copy kept with the data) |
| `02_zedx_pipeline/station_svo_v2.sh` | 10 KB | one of our tools/scripts (copy kept with the data) |
| `02_zedx_pipeline/s2_replay_vs_live.py` | 10 KB | one of our tools/scripts (copy kept with the data) |
| `02_zedx_pipeline/s3_find_target.py` | 10 KB | one of our tools/scripts (copy kept with the data) |
| `02_zedx_pipeline/s2_station_stage.sh` | 9 KB | one of our tools/scripts (copy kept with the data) |
| `02_zedx_pipeline/calib_dashboard.py` | 9 KB | one of our tools/scripts (copy kept with the data) |
| `02_zedx_pipeline/s4_laser.py` | 9 KB | one of our tools/scripts (copy kept with the data) |
| `02_zedx_pipeline/s2_station_sweep.sh` | 9 KB | one of our tools/scripts (copy kept with the data) |
| `02_zedx_pipeline/s2_report.py` | 9 KB | one of our tools/scripts (copy kept with the data) |
| `02_zedx_pipeline/campaign_selftest.sh` | 9 KB | one of our tools/scripts (copy kept with the data) |
| `02_zedx_pipeline/s4_camera.py` | 8 KB | one of our tools/scripts (copy kept with the data) |
| `02_zedx_pipeline/s3_camera.sh` | 8 KB | one of our tools/scripts (copy kept with the data) |
| `02_zedx_pipeline/campaign_runner.py` | 8 KB | one of our tools/scripts (copy kept with the data) |
| `02_zedx_pipeline/station_svo_v1_as_ran_6m.sh` | 8 KB | one of our tools/scripts (copy kept with the data) |
| `02_zedx_pipeline/s4_regate.py` | 7 KB | one of our tools/scripts (copy kept with the data) |
| `02_zedx_pipeline/s4_campaign.sh` | 7 KB | one of our tools/scripts (copy kept with the data) |
| `02_zedx_pipeline/s2_3m_stage.sh` | 7 KB | one of our tools/scripts (copy kept with the data) |
| `02_zedx_pipeline/swap_to_v2.sh` | 7 KB | one of our tools/scripts (copy kept with the data) |
| `02_zedx_pipeline/s2_camera.sh` | 7 KB | one of our tools/scripts (copy kept with the data) |
| ... 61 more files of this kind | | |
| + 26 small record files (logs, json, csv, txt, pids) | 752 KB | run records: logs, checks, progress lines |

### 03_data  (497 files, 232.45 MB)

| file | size | what it contains |
|---|---|---|
| `03_data/_jetson_station_figures/w3_4.0m_NEURAL_pinned/w3_4.0m_NEURAL_pinned_raw.npz` | 51.15 MB | - |
| `03_data/_jetson_station_figures/w3_4.0m_NEURAL/w3_4.0m_NEURAL_raw.npz` | 50.37 MB | - |
| `03_data/_jetson_station_figures/w3_3.0m_NEURAL_fromSVO/w3_3.0m_NEURAL_fromSVO_raw.npz` | 41.72 MB | - |
| `03_data/detector_noise/frame40_views.npz` | 9.16 MB | - |
| `03_data/_jetson_station_figures/w3_4.0m_NEURAL/w3_4.0m_NEURAL_timelapse.npz` | 5.40 MB | - |
| `03_data/_jetson_cowork_media/frames/s4_walk_RECEDE_1_f00064.png` | 2.15 MB | a figure or photo |
| `03_data/_jetson_cowork_media/frames/s4_walk_RECEDE_2_f00060.png` | 1.95 MB | a figure or photo |
| `03_data/_jetson_cowork_media/frames/s4_walk_RECEDE_3_f00090.png` | 1.93 MB | a figure or photo |
| `03_data/_jetson_cowork_media/frames/s4_walk_RECEDE_3_f00454.png` | 1.93 MB | a figure or photo |
| `03_data/_jetson_cowork_media/frames/s4_walk_APPR_3_take2_f00525.png` | 1.93 MB | a figure or photo |
| `03_data/_jetson_cowork_media/frames/s4_walk_APPR_1_f00058.png` | 1.88 MB | a figure or photo |
| `03_data/_jetson_cowork_media/frames/s4_walk_RECEDE_1_f00544.png` | 1.87 MB | a figure or photo |
| `03_data/_jetson_cowork_media/frames/s4_walk_APPR_3_take2_f00061.png` | 1.87 MB | a figure or photo |
| `03_data/_jetson_cowork_media/frames/s4_walk_RECEDE_2_f00515.png` | 1.87 MB | a figure or photo |
| `03_data/_jetson_cowork_media/frames/s4_walk_RECEDE_1_f00320.png` | 1.87 MB | a figure or photo |
| `03_data/_jetson_cowork_media/frames/s4_walk_RECEDE_3_f00771.png` | 1.87 MB | a figure or photo |
| `03_data/_jetson_cowork_media/frames/s4_walk_RECEDE_2_f00303.png` | 1.87 MB | a figure or photo |
| `03_data/_jetson_cowork_media/frames/s4_walk_APPR_2_f00335.png` | 1.86 MB | a figure or photo |
| `03_data/_jetson_cowork_media/frames/s4_walk_APPR_3_take2_f00309.png` | 1.86 MB | a figure or photo |
| `03_data/_jetson_cowork_media/frames/s4_empty_scene_f00425.png` | 1.86 MB | a figure or photo |
| `03_data/_jetson_cowork_media/frames/s4_empty_scene_f00250.png` | 1.86 MB | a figure or photo |
| `03_data/_jetson_cowork_media/frames/s4_empty_scene_f00050.png` | 1.86 MB | a figure or photo |
| `03_data/_jetson_cowork_media/frames/s4_walk_APPR_2_f00570.png` | 1.86 MB | a figure or photo |
| `03_data/_jetson_cowork_media/frames/s4_walk_APPR_1_f00498.png` | 1.86 MB | a figure or photo |
| `03_data/_jetson_cowork_media/frames/s4_walk_APPR_2_f00067.png` | 1.86 MB | a figure or photo |
| `03_data/_jetson_cowork_media/frames/s4_walk_APPR_1_f00293.png` | 1.86 MB | a figure or photo |
| `03_data/_jetson_station_figures/w3_3.0m_NEURAL_fromSVO/w3_3.0m_NEURAL_fromSVO_left.png` | 536 KB | a figure or photo |
| `03_data/_jetson_station_figures/w3_4.0m_NEURAL/w3_4.0m_NEURAL_right.png` | 529 KB | a figure or photo |
| `03_data/_jetson_station_figures/w3_4.0m_NEURAL/w3_4.0m_NEURAL_left.png` | 528 KB | a figure or photo |
| `03_data/_jetson_station_figures/w3_4.0m_NEURAL_pinned/w3_4.0m_NEURAL_pinned_right.png` | 525 KB | a figure or photo |
| `03_data/_jetson_station_figures/w3_4.0m_NEURAL_pinned/w3_4.0m_NEURAL_pinned_left.png` | 523 KB | a figure or photo |
| `03_data/_jetson_station_figures/w3_4.0m_NEURAL_pinned/w3_4.0m_NEURAL_pinned_cloud_box.png` | 324 KB | a figure or photo |
| `03_data/_jetson_station_figures/w3_4.0m_NEURAL/w3_4.0m_NEURAL_cloud_box.png` | 322 KB | a figure or photo |
| `03_data/_jetson_station_figures/w3_3.0m_NEURAL_fromSVO/w3_3.0m_NEURAL_fromSVO_cloud_box.png` | 285 KB | a figure or photo |
| `03_data/_jetson_station_figures/w3_4.0m_NEURAL/w3_4.0m_NEURAL_cloud_full.png` | 146 KB | a figure or photo |
| `03_data/_jetson_station_figures/w3_3.0m_NEURAL_fromSVO/w3_3.0m_NEURAL_fromSVO_cloud_full.png` | 142 KB | a figure or photo |
| `03_data/_jetson_station_figures/w3_4.0m_NEURAL_pinned/w3_4.0m_NEURAL_pinned_cloud_full.png` | 141 KB | a figure or photo |
| `03_data/_jetson_scenario2_3m/w3_3.0m_setup.jpg` | 140 KB | a figure or photo |
| `03_data/_jetson_station_figures/w3_4.0m_NEURAL/w3_4.0m_NEURAL_confidence.png` | 107 KB | a figure or photo |
| `03_data/_jetson_station_figures/w3_3.0m_NEURAL_fromSVO/w3_3.0m_NEURAL_fromSVO_confidence.png` | 106 KB | a figure or photo |
| ... 30 more files of this kind | | |
| + 427 small record files (logs, json, csv, txt, pids) | 29.65 MB | run records: logs, checks, progress lines |

### 04_analysis  (19 files, 154 KB)

| file | size | what it contains |
|---|---|---|
| + 19 small record files (logs, json, csv, txt, pids) | 154 KB | run records: logs, checks, progress lines |

### 05_figures  (57 files, 8.15 MB)

| file | size | what it contains |
|---|---|---|
| `05_figures/scenes/walkway1_5m_lossless.png` | 1006 KB | a figure or photo |
| `05_figures/scenes/walkway1_7m_lossless.png` | 986 KB | a figure or photo |
| `05_figures/stations/w3_3m_vs_4m_CAMERA.png` | 861 KB | a figure or photo |
| `05_figures/fig_camera_views.png` | 828 KB | a figure or photo |
| `05_figures/scenes/walkway3_4m.png` | 523 KB | a figure or photo |
| `05_figures/fig_apex_surface.png` | 409 KB | a figure or photo |
| `05_figures/fig_detector_noise.png` | 282 KB | a figure or photo |
| `05_figures/stations/w3_4.0m_LIDAR_CONTACT_SHEET.png` | 282 KB | a figure or photo |
| `05_figures/fig_lands_where_5m.png` | 274 KB | a figure or photo |
| `05_figures/fig_per_mode_term.png` | 184 KB | a figure or photo |
| `05_figures/fig_precision_vs_accuracy.png` | 182 KB | a figure or photo |
| `05_figures/fig_camera_views.jpg` | 181 KB | a figure or photo |
| `05_figures/fig_depth_sweep_8.0m.png` | 179 KB | a figure or photo |
| `05_figures/fig_offline_5m_modes.png` | 179 KB | a figure or photo |
| `05_figures/fig_depth_sweep_7.0m.png` | 179 KB | a figure or photo |
| `05_figures/fig_depth_sweep_5.0m.png` | 177 KB | a figure or photo |
| `05_figures/fig_station_geometry.png` | 174 KB | a figure or photo |
| `05_figures/fig_s3_motion.png` | 171 KB | a figure or photo |
| `05_figures/fig_apex_surface.jpg` | 162 KB | a figure or photo |
| `05_figures/explainers/all_stations_scatter.png` | 107 KB | a figure or photo |
| `05_figures/explainers/w3_4m_lasers_scatter.png` | 100 KB | a figure or photo |
| `05_figures/fig_s3_reach_s3_0.85ms_run1.png` | 98 KB | a figure or photo |
| `05_figures/explainers/what_rms3d_means.png` | 98 KB | a figure or photo |
| `05_figures/fig_s3_reach_s3_0.85ms_run2.png` | 96 KB | a figure or photo |
| `05_figures/fig_s3_reach_s3_0.40ms_run2.png` | 95 KB | a figure or photo |
| `05_figures/fig_s3_reach_s3_0.40ms_run1.png` | 95 KB | a figure or photo |
| `05_figures/stations/w3_4.0m_RING_STRUCTURE.png` | 64 KB | a figure or photo |
| `05_figures/build_station_5m_page.py` | 31 KB | one of our tools/scripts (copy kept with the data) |
| `05_figures/build_offline_5m_report.py` | 27 KB | one of our tools/scripts (copy kept with the data) |
| `05_figures/fig_s3_reach.py` | 10 KB | one of our tools/scripts (copy kept with the data) |
| `05_figures/plot_detector_noise.py` | 10 KB | one of our tools/scripts (copy kept with the data) |
| `05_figures/station_modes_table.py` | 10 KB | one of our tools/scripts (copy kept with the data) |
| `05_figures/analyse_closeout.py` | 9 KB | one of our tools/scripts (copy kept with the data) |
| `05_figures/fig_s3_motion.py` | 9 KB | one of our tools/scripts (copy kept with the data) |
| `05_figures/plot_lands_where.py` | 8 KB | one of our tools/scripts (copy kept with the data) |
| `05_figures/plot_per_mode_term.py` | 8 KB | one of our tools/scripts (copy kept with the data) |
| `05_figures/render_burst_rate_drift.py` | 8 KB | one of our tools/scripts (copy kept with the data) |
| `05_figures/render_timelapse.py` | 8 KB | one of our tools/scripts (copy kept with the data) |
| `05_figures/plot_station_geometry.py` | 6 KB | one of our tools/scripts (copy kept with the data) |
| `05_figures/plot_apex_surface.py` | 6 KB | one of our tools/scripts (copy kept with the data) |
| ... 9 more files of this kind | | |
| + 8 small record files (logs, json, csv, txt, pids) | 181 KB | run records: logs, checks, progress lines |

### 08_knowledge_base  (10 files, 277 KB)

| file | size | what it contains |
|---|---|---|
| + 10 small record files (logs, json, csv, txt, pids) | 277 KB | run records: logs, checks, progress lines |

### 09_web_monitor  (19 files, 113 KB)

| file | size | what it contains |
|---|---|---|
| `09_web_monitor/sync_page.py` | 6 KB | one of our tools/scripts (copy kept with the data) |
| + 18 small record files (logs, json, csv, txt, pids) | 107 KB | run records: logs, checks, progress lines |

*Four more folders of the study (06, 07, 10, 11; about 25 MB) hold its write-up material and are not listed here.*

### catkin_ws  (1 files, 8 KB)

| file | size | what it contains |
|---|---|---|
| `catkin_ws/src/sidewalk_evaluation/scripts/svo_camera_pose.py` | 8 KB | one of our tools/scripts (copy kept with the data) |

## series1_august_lab/

*August lab-room work: logs_real_indoor, jetson_run_records_rescue, Week_7, jetson_sidewalk128/{rtabmap_maps,archive} (the old card's databases).*

1999 files, 48.32 GB in all.

### jetson_sidewalk128  (1999 files, 48.32 GB)

| file | size | what it contains |
|---|---|---|
| `jetson_sidewalk128/archive/rescued_data/lab_map_01.db` | 8.54 GB | RTAB-Map database: every map snapshot (picture + position), the links between them, and the corrected positions if closed properly |
| `jetson_sidewalk128/rtabmap_maps/2026-08-26/lab_map_05.db` | 6.83 GB | RTAB-Map database: every map snapshot (picture + position), the links between them, and the corrected positions if closed properly |
| `jetson_sidewalk128/rtabmap_maps/2026-09-01/lab_map_09b.db` | 6.33 GB | RTAB-Map database: every map snapshot (picture + position), the links between them, and the corrected positions if closed properly |
| `jetson_sidewalk128/rtabmap_maps/2026-09-01/lab_map_09a.db` | 6.31 GB | RTAB-Map database: every map snapshot (picture + position), the links between them, and the corrected positions if closed properly |
| `jetson_sidewalk128/rtabmap_maps/2026-09-01/lab_map_08.db` | 5.41 GB | RTAB-Map database: every map snapshot (picture + position), the links between them, and the corrected positions if closed properly |
| `jetson_sidewalk128/rtabmap_maps/2026-09-01/lab_map_09d.db` | 4.63 GB | RTAB-Map database: every map snapshot (picture + position), the links between them, and the corrected positions if closed properly |
| `jetson_sidewalk128/archive/rescued_data/office1_explore_03.db` | 3.66 GB | RTAB-Map database: every map snapshot (picture + position), the links between them, and the corrected positions if closed properly |
| `jetson_sidewalk128/rtabmap_maps/2026-09-01/lab_map_09c.db` | 3.47 GB | RTAB-Map database: every map snapshot (picture + position), the links between them, and the corrected positions if closed properly |
| `jetson_sidewalk128/rtabmap_maps/2026-08-30/bench_r09n5_raytrace_off.db` | 1.71 GB | RTAB-Map database: every map snapshot (picture + position), the links between them, and the corrected positions if closed properly |
| `jetson_sidewalk128/archive/rescued_data/lab_map_01_repair.orig.bag` | 88.56 MB | a ROS bag recording |
| `jetson_sidewalk128/archive/rescued_data/lab_map_01.bag` | 88.56 MB | a ROS bag recording |
| `jetson_sidewalk128/rtabmap_maps/2026-08-26/lab_map_05_3d_cloud.ply` | 46.53 MB | - |
| `jetson_sidewalk128/rtabmap_maps/2026-08-26/lab_map_06_3d_cloud.ply` | 42.89 MB | - |
| `jetson_sidewalk128/archive/rescued_data/lab_map_01_repair.bag` | 15.22 MB | a ROS bag recording |
| `jetson_sidewalk128/archive/rescued_data/lab_map_01_depth/1787532659.717926.png` | 1.98 MB | a figure or photo |
| `jetson_sidewalk128/archive/rescued_data/lab_map_01_depth/1787532639.217818.png` | 1.96 MB | a figure or photo |
| `jetson_sidewalk128/archive/rescued_data/lab_map_01_depth/1787532629.884442.png` | 1.94 MB | a figure or photo |
| `jetson_sidewalk128/archive/rescued_data/lab_map_01_depth/1787533091.120213.png` | 1.93 MB | a figure or photo |
| `jetson_sidewalk128/archive/rescued_data/lab_map_01_depth/1787532675.818001.png` | 1.91 MB | a figure or photo |
| `jetson_sidewalk128/archive/rescued_data/lab_map_01_depth/1787533122.353737.png` | 1.91 MB | a figure or photo |
| `jetson_sidewalk128/archive/rescued_data/lab_map_01_depth/1787532637.184476.png` | 1.91 MB | a figure or photo |
| `jetson_sidewalk128/archive/rescued_data/lab_map_01_depth/1787533119.920365.png` | 1.90 MB | a figure or photo |
| `jetson_sidewalk128/archive/rescued_data/lab_map_01_depth/1787533088.087010.png` | 1.90 MB | a figure or photo |
| `jetson_sidewalk128/archive/rescued_data/lab_map_01_depth/1787532627.751113.png` | 1.90 MB | a figure or photo |
| `jetson_sidewalk128/archive/rescued_data/lab_map_01_depth/1787532628.884433.png` | 1.89 MB | a figure or photo |
| `jetson_sidewalk128/archive/rescued_data/lab_map_01_depth/1787533108.420309.png` | 1.89 MB | a figure or photo |
| `jetson_sidewalk128/archive/rescued_data/lab_map_01_depth/1787532638.184552.png` | 1.88 MB | a figure or photo |
| `jetson_sidewalk128/archive/rescued_data/lab_map_01_depth/1787532640.284502.png` | 1.88 MB | a figure or photo |
| `jetson_sidewalk128/archive/rescued_data/lab_map_01_depth/1787533112.887008.png` | 1.88 MB | a figure or photo |
| `jetson_sidewalk128/archive/rescued_data/lab_map_01_depth/1787532630.984448.png` | 1.87 MB | a figure or photo |
| `jetson_sidewalk128/archive/rescued_data/lab_map_01_depth/1787532641.417828.png` | 1.87 MB | a figure or photo |
| `jetson_sidewalk128/archive/rescued_data/lab_map_01_depth/1787532623.451078.png` | 1.87 MB | a figure or photo |
| `jetson_sidewalk128/archive/rescued_data/lab_map_01_depth/1787532626.651093.png` | 1.87 MB | a figure or photo |
| `jetson_sidewalk128/archive/rescued_data/lab_map_01_depth/1787532624.451085.png` | 1.86 MB | a figure or photo |
| `jetson_sidewalk128/archive/rescued_data/lab_map_01_depth/1787533109.653659.png` | 1.86 MB | a figure or photo |
| `jetson_sidewalk128/archive/rescued_data/lab_map_01_depth/1787532632.051112.png` | 1.85 MB | a figure or photo |
| `jetson_sidewalk128/archive/rescued_data/lab_map_01_depth/1787533114.053691.png` | 1.85 MB | a figure or photo |
| `jetson_sidewalk128/archive/rescued_data/lab_map_01_depth/1787532663.917947.png` | 1.85 MB | a figure or photo |
| `jetson_sidewalk128/archive/rescued_data/lab_map_01_depth/1787532625.517746.png` | 1.85 MB | a figure or photo |
| `jetson_sidewalk128/archive/rescued_data/lab_map_01_depth/1787533101.186932.png` | 1.84 MB | a figure or photo |
| ... 1292 more files of this kind | | |
| + 667 small record files (logs, json, csv, txt, pids) | 555 KB | run records: logs, checks, progress lines |

## slam_research_01_runs/

*The results packs (numbers, figures, timelapse videos, 3D clouds) - the same folders as results/<series>/<run>/ in the repository, with the large files.*

Not itemised in the 29 Sept listing. It holds the same folders as `results/<series>/<run>/` in this repository, plus the large files kept out of the repository (timelapse videos, 3D point clouds).

## freeze_bench_2026-09-26/

*Parked camera-freeze benches of 26 Sept (timing.bag per condition): the evidence that freezes grow with Jetson uptime and that GEN_1 freezes more.*

15 files, 2.95 GB in all.

### freeze_ab  (4 files, 241.37 MB)

| file | size | what it contains |
|---|---|---|
| `freeze_ab/R1_GEN2/timing.bag` | 63.31 MB | camera timing bag of a parked freeze bench (camera picture stamps) |
| `freeze_ab/R4_GEN2/timing.bag` | 60.46 MB | camera timing bag of a parked freeze bench (camera picture stamps) |
| `freeze_ab/R2_GEN1/timing.bag` | 59.92 MB | camera timing bag of a parked freeze bench (camera picture stamps) |
| `freeze_ab/R3_GEN1/timing.bag` | 57.68 MB | camera timing bag of a parked freeze bench (camera picture stamps) |

### freeze_cause  (11 files, 2.71 GB)

| file | size | what it contains |
|---|---|---|
| `freeze_cause/L1_lights_on/timing.bag` | 1.17 GB | camera timing bag of a parked freeze bench (camera picture stamps) |
| `freeze_cause/R1_after_reboot/timing.bag` | 1.04 GB | camera timing bag of a parked freeze bench (camera picture stamps) |
| `freeze_cause/B2_baseline/timing.bag` | 61.25 MB | camera timing bag of a parked freeze bench (camera picture stamps) |
| `freeze_cause/S2_no_live_sender/timing.bag` | 60.75 MB | camera timing bag of a parked freeze bench (camera picture stamps) |
| `freeze_cause/S3_no_storage_sense/timing.bag` | 60.52 MB | camera timing bag of a parked freeze bench (camera picture stamps) |
| `freeze_cause/S4_no_page_servers/timing.bag` | 60.38 MB | camera timing bag of a parked freeze bench (camera picture stamps) |
| `freeze_cause/S3b_no_storage_sense/timing.bag` | 60.31 MB | camera timing bag of a parked freeze bench (camera picture stamps) |
| `freeze_cause/B1_baseline/timing.bag` | 60.21 MB | camera timing bag of a parked freeze bench (camera picture stamps) |
| `freeze_cause/S1_no_preload/timing.bag` | 59.89 MB | camera timing bag of a parked freeze bench (camera picture stamps) |
| `freeze_cause/S5a_fresh_services/timing.bag` | 56.38 MB | camera timing bag of a parked freeze bench (camera picture stamps) |
| `freeze_cause/S5b_fresh_services/timing.bag` | 39.55 MB | camera timing bag of a parked freeze bench (camera picture stamps) |

## Camera depth-study station recordings (.svo2) - added separately

The bench-test camera recordings of the camera depth study (the `.svo2` files from the sphere stations) are not in the 29 Sept listing; they are added to the Teams folder separately, under `paper_zedx_vs_lidar/`. What to expect:

| station / set | files (pattern) | notes |
|---|---|---|
| walkway-3 station 3 m | `w3_3.0m_*.svo2` | the last copy was removed 2026-09-19: CSVs/logs only in git |
| walkway-3 station 4 m | `w3_4.0m_*.svo2` (+ `w3_4.0m_provenance.txt`) | the 4 m sphere-centre figures come from 9 Sept |
| walkway-3 station 5 m | `w3_5.0m_burst01.svo2` ... (several ~80 s bursts) | lossless PNG compression, 1920x1200, SDK 4.2.5 |
| walkway-3 station 7 m | `w3_7.0m_*.svo2` | |
| walkway-3 station 8 m | `w3_8.0m_*.svo2` (incl. the 1400 mm wide-box pass) | |
| scenario 3 (drive-at-the-frame, 0.40 and 0.85 m/s) | `s3_0.40ms_run*` / `s3_0.85ms_run*` recordings + laser bags under `bags/` | 21 GB with scenario 4 (`03_data/scenario3/`) |
| scenario 4 | `03_data/scenario4/` recordings + `bags/` | as above |

Each `.svo2` holds the stereo pictures and the camera IMU; depth is recomputed offline in any depth mode (`ZED_SVO_Editor -inf <file>` prints the real compression mode).

## Total listed

282.73 GB across the folders listed above.

## Other contents of robot_card_slam_series2/ (last known list of the robot card)

*Last known contents of the card (29 Sept 2026): `<run>_lidar.bag` + `<run>_gyro_zero.json` for s2_fusion_T5/T5b/T5c, s2_loc_01/02/03/04, s2_nav_01/02/03, s2_scale_01, s2_slam_01, s2_static_03/05/06/07/08/09/10; replay folders `s2_loc_02/ s2_loc_03/ s2_loc_04/ s2_static_03/ s2_static_05/ s2_static_06/ s2_static_07/ s2_static_08/ s2_static_09/ s2_static_10/`; `*_replay.out` / `*_f3dof_replay.out`; `tools/`, `tools_carving_test/`, `live_view_tests/`, `gyro_tests/`, `maps/`, `bad_clock_20260926/`; test bags `chain_test.bag`, `rosip_test.bag`; `record_lidar.sh`, `replay_lidar.sh`.*

*The run descriptions (what / why it was driven / results pack) were written by hand from each run's RESULTS.md. To list a local copy of the Teams folder in the same table form: `python3 data/list_folder.py <folder>` (see `data/README.md`).*
