# Drive 6 (s2_static_06) - results pack

*26 Sept 2026, Hamilton time: live recorder started 04:20:44, "ready" about a minute later, camera program down 04:24:49.5-04:25:02.2 (12.7 s, restarted by the camera guard), "park" 04:44:45, map closed properly 04:45:32. The fourth fused drive (fused = the camera blended with the robot's wheels and gyroscope (turn-rate sensor) by an EKF (extended Kalman filter, a standard way of mixing sensors)), and the longest, with a new walkway added to the route. Made on the Jetson on 26 Sept by `results_packs_2026-09-26/` (project records) (pack_drive6_heavy.sh, analyze_run.py, turns_replay.py, extract_timing.py, tegra_peaks.py, crash_split_drive6.py, compare_lidar.py, this file's writer `write_results_drive6.py`). Every number names the file it comes from; files are in this folder unless a path is given. Nothing here is ground truth (ENGINEERING_NOTES.md rule 19). N = 1.*

## 0. The camera outage (read first)

- `camera_outages.csv` (camera guard): camera program **gone at 04:24:49.5, back at 04:25:02.2: 12.7 s**; outcome: "restarted (1); camera positions back after 0.4 s".
- The camera tracker (`/rtabmap/odom`, fusion.bag) was silent longest 12.87 s near the outage (04:24:48 to 04:25:01; crash_split.json).
- *Plain terms: unlike drive 5, the new camera guard restarted the camera program within 13 s and the map carried on. The robot moved 0.0 m (its own wheels + gyroscope) while the camera was down, i.e. it was standing still, so no stretch of the route is missing from the camera map.*

| | start to outage | during the outage | back to end | source |
|---|---|---|---|---|
| distance, robot's own wheels + gyroscope (/robot/ekf_odom) | 38.2 m | 0.0 m | 200.2 m | crash_split.json |
| distance, blend (/fused/odometry) | 37.7 m | 0.0 m | 203.2 m | crash_split.json |
| map snapshots (nodes) | 264 | 2 | 1048 | database Node table |

*Distances: positions every 0.5 s, straight steps summed (crash_split.json); the start used is the recording's first message, because drive 6's "ready" moment is not in a timed line.*

## 1. How far from home at the end - both kinds (rule 20)

*Plain terms: "tracking alone" = where each source's own step-by-step tracking put the robot, never corrected; "corrected" = where the map puts it after its loop closures (the map recognising a place seen before and pulling itself straight). In a fused drive the map's nodes carry the BLEND's position (drive 4 RESULTS.md section 1 explains why), and the camera's tracking restarts from the blend each time it gives up.*

*Words used below: TF = ROS's shared table of where each part of the robot is; Node.pose = each map snapshot's tracking-alone position; Admin.opt_poses = the corrected positions RTAB-Map saves when it shuts down properly; fusion.bag = the drive's recording of every sensor stream; quick_check = SQLite's own database-integrity check; tegrastats = NVIDIA's once-a-second load/power/temperature log; ffmpeg = a standard video program; rtabmap-export settings: depth up to 6 m, every 4th pixel, points merged into 3 cm cubes.*

| kind | source | end-to-start gap | heading gap | path length | file |
|---|---|---|---|---|---|
| tracking alone | map nodes (Node.pose = the blend's position at each node), node 1 to node 1314 | **1.847 m** | - | 238.14 m | facts.json, camera.tum |
| tracking alone | camera tracking, every message (/rtabmap/odom, lost stretches left out; re-seeded by the blend at each restart) | 1.135 m | -2.40 deg | 279.9 m | numbers.json (fusion.bag) |
| tracking alone | blend (/fused/odometry) | 1.845 m | -2.93 deg | 269.4 m | numbers.json |
| tracking alone | robot's own wheels + gyroscope (/robot/ekf_odom) | 3.380 m | -18.50 deg | 242.7 m | numbers.json |
| **corrected** | camera map's saved graph, Admin.opt_poses, node 1 to node 1260 (830 graph nodes), 3D | **0.7391 m** | - | - | camera_corrected.tum.meta.json |
| corrected | every camera node placed by its graph node's correction, node 1 to node 1314, floor plane | 0.742 m | - | 230.23 m | map_corrected_facts.json |
| corrected | **LiDAR map** (robot, colleague's self_navigation rtabmap_3d.launch), floor plane | **2.138 m** | - | 235.8 m | lidar_map_facts.json |
| corrected | LiDAR map, 3D (height included) | 5.0783 m | - | - | lidar.tum.meta.json |

**Loop closures beside the corrected gaps: camera map 135** (23 recognised-again + 112 nearby re-matches; database Link table types 1 and 2, counted once per pair; `closed_check.txt` gives 135); the live monitor's last line said 130 accepted (`~/.run_records/s2_static_06/monitor_STATUS.txt`) - the database, read after the map closed, is the complete count. **LiDAR map 59** (lidar.tum.meta.json).

- Closures joining a later node straight to node 1 (the start): 0. None, so the corrected gap is not simply a repeat of an end-to-start closure.
- Closures touching a node made before the camera outage (04:24:49): **0 of 135** (crash_split.json). *Plain terms: after the camera came back, the map never re-recognised any place from the first part of the drive, including the start - so nothing tied the end of the drive back to its beginning.*
- Last camera loop closure: newer node at 04:44:27 (Hamilton); the park was 04:44:45.
- **The LiDAR map's 3D gap (5.08 m) is much larger than its floor gap (2.14 m)**: [INFERENCE] the difference is mostly height (sqrt(3D^2 - floor^2) = 4.6 m), i.e. the LiDAR map's height drifted; the floor-plane number is the one to compare with the camera's. Not investigated further here.

## 2. Every turn: robot vs camera vs blend

The live turn watcher's rule replayed after the drive over fusion.bag by `turns_replay.py` (unchanged; its control on drive 3 is in drive 4 RESULTS.md section 2). Output: turns_replayed.log -> turns.csv. *95th percentile = the value 95 % of turns stay under. The robot's own wheels + gyroscope is the comparison for short turns (not ground truth).*

| summary (65 turns) | median | 95th percentile | largest | within 1 deg |
|---|---|---|---|---|
| abs(blend - robot) | 0.30 deg | 1.68 deg | 2.8 deg | 55 of 65 |
| abs(camera - robot) | 4.35 deg | 20.64 deg | 58.9 deg | 4 of 65 |

Turns with no camera value (camera lost for the whole turn): 1. Figure: `turns_difference.png`.

| turn | ended | direction | s | robot deg | camera deg | blend deg | camera lost | blend - robot | camera - robot |
|---|---|---|---|---|---|---|---|---|---|
| 1 | 04:22:26 | left | 3.9 | 86.1 | 46.8 | 86.9 | 71 % | 0.8 | -39.3 |
| 2 | 04:22:31 | left | 1.6 | 37.9 | 96.8 | 38.7 | 61 % | 0.8 | 58.9 |
| 3 | 04:22:34 | left | 0.9 | 18.4 | 24.1 | 19.3 | 0 % | 0.9 | 5.7 |
| 4 | 04:23:34 | left | 5.9 | 73.4 | 72.0 | 73.5 | 0 % | 0.1 | -1.4 |
| 5 | 04:23:54 | left | 2.1 | 0.6 | 2.1 | 0.4 | 0 % | -0.2 | 1.5 |
| 6 | 04:24:46 | left | 2.9 | 63.1 | 69.4 | 61.2 | 0 % | -1.9 | 6.3 |
| 7 | 04:24:49 | left | 1.5 | 40.7 | 55.3 | 41.3 | 0 % | 0.6 | 14.6 |
| 8 | 04:24:54 | left | 0.6 | 9.9 | None | 10.7 | 100 % | 0.8 | None |
| 9 | 04:25:18 | right | 0.5 | -6.4 | -10.6 | -6.9 | 0 % | -0.5 | -4.2 |
| 10 | 04:25:22 | right | 0.3 | -2.9 | -8.2 | -2.9 | 0 % | 0.1 | -5.3 |
| 11 | 04:25:25 | right | 0.6 | -10.8 | -15.2 | -11.2 | 0 % | -0.4 | -4.4 |
| 12 | 04:25:29 | right | 0.1 | -0.2 | -1.8 | 0.0 | 0 % | 0.2 | -1.6 |
| 13 | 04:25:33 | right | 0.5 | -4.7 | -7.1 | -4.0 | 0 % | 0.7 | -2.4 |
| 14 | 04:25:36 | right | 0.2 | -0.1 | -4.8 | -0.0 | 0 % | 0.1 | -4.7 |
| 15 | 04:25:58 | left | 0.5 | 4.6 | 10.5 | 4.7 | 0 % | 0.0 | 5.9 |
| 16 | 04:26:57 | right | 0.2 | -0.4 | -5.5 | -0.6 | 0 % | -0.2 | -5.1 |
| 17 | 04:27:01 | right | 0.1 | 0.3 | -4.0 | 0.6 | 0 % | 0.2 | -4.3 |
| 18 | 04:27:06 | left | 1.2 | 3.7 | 4.8 | 3.8 | 0 % | 0.1 | 1.1 |
| 19 | 04:27:09 | left | 0.2 | 0.5 | 10.2 | 0.1 | 0 % | -0.4 | 9.7 |
| 20 | 04:27:14 | left | 0.4 | 7.6 | 17.0 | 7.5 | 0 % | -0.1 | 9.4 |
| 21 | 04:27:23 | left | 0.6 | 16.9 | 19.5 | 17.6 | 81 % | 0.7 | 2.6 |
| 22 | 04:27:25 | left | 0.2 | 0.6 | 9.1 | 1.9 | 0 % | 1.2 | 8.5 |
| 23 | 04:28:06 | left | 0.3 | 4.7 | 11.9 | 6.9 | 0 % | 2.2 | 7.2 |
| 24 | 04:29:38 | right | 12.3 | -106.9 | -115.5 | -107.2 | 14 % | -0.3 | -8.6 |
| 25 | 04:29:43 | right | 2.3 | -21.7 | -26.1 | -22.5 | 14 % | -0.7 | -4.4 |
| 26 | 04:29:46 | right | 1.5 | -23.7 | -27.3 | -24.4 | 0 % | -0.8 | -3.6 |
| 27 | 04:30:21 | left | 0.7 | 8.9 | 11.4 | 8.4 | 0 % | -0.5 | 2.5 |
| 28 | 04:30:25 | left | 0.9 | 11.5 | 15.5 | 11.6 | 0 % | 0.1 | 4.0 |
| 29 | 04:30:28 | left | 0.2 | 0.8 | 5.0 | 0.5 | 0 % | -0.3 | 4.2 |
| 30 | 04:30:36 | left | 4.5 | 19.5 | 24.2 | 19.8 | 0 % | 0.3 | 4.7 |
| 31 | 04:31:15 | right | 3.0 | -84.0 | -89.9 | -85.0 | 0 % | -1.0 | -5.9 |
| 32 | 04:31:40 | left | 0.3 | 1.6 | 3.6 | 1.9 | 0 % | 0.3 | 2.0 |
| 33 | 04:32:18 | left | 1.7 | 64.9 | 44.0 | 66.6 | 53 % | 1.7 | -20.9 |
| 34 | 04:32:20 | left | 0.6 | 13.4 | 53.1 | 13.6 | 56 % | 0.2 | 39.7 |
| 35 | 04:32:26 | left | 4.1 | 62.0 | 68.7 | 62.0 | 0 % | -0.0 | 6.7 |
| 36 | 04:33:11 | left | 1.8 | 20.5 | 22.6 | 20.4 | 0 % | -0.2 | 2.1 |
| 37 | 04:33:23 | left | 3.3 | 60.0 | 62.6 | 59.7 | 0 % | -0.2 | 2.6 |
| 38 | 04:34:10 | left | 2.3 | 76.8 | 84.5 | 77.4 | 0 % | 0.6 | 7.7 |
| 39 | 04:34:45 | right | 6.4 | -85.0 | -86.2 | -85.3 | 0 % | -0.2 | -1.2 |
| 40 | 04:34:54 | right | 2.1 | -37.1 | -37.5 | -34.3 | 0 % | 2.8 | -0.4 |
| 41 | 04:35:02 | right | 3.9 | -19.0 | -20.2 | -19.1 | 0 % | -0.1 | -1.2 |
| 42 | 04:35:07 | right | 1.4 | -4.3 | -7.4 | -4.8 | 0 % | -0.5 | -3.1 |
| 43 | 04:35:34 | left | 8.0 | 77.0 | 57.8 | 77.2 | 8 % | 0.3 | -19.2 |
| 44 | 04:37:31 | left | 0.4 | 5.7 | 11.7 | 6.9 | 0 % | 1.2 | 6.0 |
| 45 | 04:37:34 | left | 0.4 | 5.3 | 11.0 | 6.1 | 0 % | 0.8 | 5.7 |
| 46 | 04:37:37 | left | 0.3 | 2.0 | 5.6 | 2.0 | 0 % | -0.0 | 3.6 |
| 47 | 04:37:42 | left | 3.9 | 32.7 | 36.3 | 32.9 | 0 % | 0.2 | 3.6 |
| 48 | 04:38:23 | left | 5.1 | 81.9 | 84.0 | 81.8 | 1 % | -0.0 | 2.1 |
| 49 | 04:38:56 | right | 2.7 | -79.6 | -84.2 | -79.7 | 0 % | -0.1 | -4.6 |
| 50 | 04:39:08 | right | 4.1 | -72.4 | -77.2 | -72.8 | 0 % | -0.4 | -4.8 |
| 51 | 04:39:35 | left | 2.7 | 75.7 | 80.0 | 75.4 | 0 % | -0.3 | 4.3 |
| 52 | 04:39:42 | right | 2.5 | 3.7 | 1.9 | 3.5 | 0 % | -0.2 | -1.8 |
| 53 | 04:39:51 | right | 2.3 | -1.0 | -3.6 | -1.5 | 0 % | -0.5 | -2.6 |
| 54 | 04:40:23 | left | 3.9 | 79.9 | 84.6 | 79.9 | 0 % | 0.1 | 4.7 |
| 55 | 04:40:31 | left | 4.7 | 89.9 | 91.1 | 89.1 | 0 % | -0.8 | 1.2 |
| 56 | 04:40:40 | left | 3.7 | 81.6 | 82.3 | 82.8 | 16 % | 1.3 | 0.7 |
| 57 | 04:41:00 | left | 14.4 | 351.3 | 354.8 | 351.9 | 5 % | 0.5 | 3.5 |
| 58 | 04:42:02 | left | 3.7 | 5.6 | 5.4 | 4.0 | 0 % | -1.6 | -0.2 |
| 59 | 04:42:10 | right | 0.2 | -0.5 | -7.2 | -2.1 | 0 % | -1.6 | -6.7 |
| 60 | 04:42:25 | left | 3.9 | 81.8 | 81.2 | 81.1 | 22 % | -0.7 | -0.6 |
| 61 | 04:43:09 | left | 3.9 | 79.1 | 86.5 | 79.4 | 0 % | 0.3 | 7.4 |
| 62 | 04:43:20 | left | 3.2 | 65.5 | 75.4 | 65.2 | 0 % | -0.2 | 9.9 |
| 63 | 04:43:35 | left | 0.5 | 3.9 | 5.0 | 4.1 | 0 % | 0.2 | 1.1 |
| 64 | 04:44:06 | right | 4.1 | -79.6 | -86.5 | -81.2 | 0 % | -1.6 | -6.9 |
| 65 | 04:44:15 | left | 3.5 | -4.5 | -0.3 | -4.3 | 0 % | 0.2 | 4.2 |

## 3. The WiFi link to the robot (the bridge)

*Source: `~/.run_records/s2_static_06/bridge_recv.log` (Jetson), its final counters; numbers.json.*

- **Link down: 14 times**, at 04:28:58, 04:29:32, 04:31:04, 04:33:39, 04:35:33, 04:35:49, 04:36:58, 04:37:04, 04:41:19, 04:41:25, 04:41:29, 04:41:34, 04:42:28, 04:44:20 (times from `bridge_recv.log` itself, whose clock is Hamilton time; numbers.json's `down_lines_hamilton` shifts them by 4 h because analyze_run.py assumes UTC - as noted in drive 5 RESULTS.md section 3). Connections: 15 in 1 receiver session(s).
- Robot-data silences over 1.5 s that did NOT have the freeze signature (freezes_pass_check.json, `other_wheel_silences_over_1p5s`, first number = seconds): 16, longest 64.6 s.

| stream | robot read | robot sent | published on the Jetson | % of sent | missed while down | too late (stale) | lost |
|---|---|---|---|---|---|---|---|
| /husky_velocity_controller/odom | 15549 | 14127 | 13753 | 97.35 | 1491 | 222 | 25 |
| /imu/data | 30292 | 27525 | 26801 | 97.37 | 2889 | 442 | 47 |
| /imu/data_raw | 30291 | 27541 | 26813 | 97.36 | 2887 | 438 | 41 |
| /odometry/filtered | 77759 | 70645 | 68773 | 97.35 | 7460 | 1119 | 119 |

## 4. Camera tracking: losses, restarts, freezes

- **Lost** (fusion.bag, `/rtabmap/odom`: all-zero pose or covariance 9999): **23 stretches, 29.3 s in total, longest 5.9 s, 1.5 % of messages** (numbers.json).
- **Restarts from TF** (`mapping.log`, "Odometry automatically reset to latest odometry pose available from TF" - the camera's tracking gave up and restarted from the blend's position): **42**, in 11 bursts: 04:22:24-04:22:30 (25); 04:27:22 (1); 04:29:36-04:29:40 (3); 04:32:17-04:32:19 (2); 04:33:49 (1); 04:35:27 (1); 04:38:17 (1); 04:40:38 (1); 04:40:58 (1); 04:41:36-04:41:41 (5); 04:42:23 (1). The mapping monitor's last line says resets=42 (`monitor_STATUS.txt`).
- **Freezes** (pass_check.py from `03_methods/jetson_stalls_drive3_2026-09-25/`, on this drive's recording -> freezes_pass_check.json): camera tracker silences > 1.5 s: **4** (3.00 s, 12.68 s, 1.57 s, 2.67 s); 'ZED Diagnostic' pauses > 2 s: 8 (3.62 s, 3.32 s, 6.19 s, 3.24 s, 2.39 s, 3.65 s, 2.32 s, 2.38 s); database saves >= 5.5 s: 0; control, camera gyroscope longest gap 10.443 s; map updates parsed 1270 of 1314 nodes. Verdict as printed: **VOID (control failed: the recorder or the whole Jetson stalled)** (pass line = zero silences).
  The 12.7 s silence is the camera outage itself (section 0), not a freeze. The check's control (the camera's own gyroscope, which must never pause more than 0.1 s) failed with a 10.44 s gap from 04:24:48 to 04:24:59 (timing.npz, receive times of /zedx_front/zed_node/imu/data) - inside the outage 04:24:49-04:25:02, so VOID here reflects the camera crash; [INFERENCE] not a Jetson-wide stall (the robot data kept arriving then). Outside the outage: 3 tracker silences > 1.5 s.
- Database integrity (`db_check.txt`, SQLite quick_check, read-only): `PASS ~/slam_series2/s2_static_06.db: quick_check=['ok'], 1314 map nodes, 1.3 s`

## 5. Blend heading while standing still (reported, not a pass line)

stops 178 (464 s still); signed sum **-2.774 deg**; sum of each stop's net change 4.436 deg; largest single stop -0.380 deg (numbers.json). Rule: robot wheel odometry speed and turn rate both zero, below 1e-6 - the log holds 1e-16 float noise, not 0.0 (nearest wheel message within 0.3 s).

## 6. Processor, power and temperature

*Source: `~/rehearsal_t5/tegrastats_drive6.log` (tegrastats, once a second), window 04:20:44 to 04:45:35 Hamilton, read by tegra_peaks.py -> power_temperature.json.*

| quantity | peak | when | median |
|---|---|---|---|
| total power | **36.5 W** | 04:25:05 | 30.0 W |
| hottest point on the chip (tj) | **61.6 C** | 04:41:07 | - |
| graphics processor temperature | 55.8 C | 04:41:22 | - |
| graphics processor load | 99 % | 04:23:36 | 23 % |
| processor load, mean of the 12 cores | 97.7 % | 04:24:59 | 40.8 % |
| memory free, lowest | 48.0 % (15.5 GB used) | 04:42:38 | - |

- tegrastats skips over 2 s: 04:25:44 (3 s), 04:26:15 (3 s), 04:42:24 (3 s); samples 1440.
- RTAB-Map, time to process each map snapshot (database Statistics table): **median 577.3 ms, 95th percentile 796.4 ms, largest 5786.2 ms over 1314 nodes**.

## 7. Agreement with the LiDAR estimate

*The reference is the robot's LiDAR map (a colleague's work: self_navigation `rtabmap_3d.launch`, replayed on the robot after the drive): the robot's wheels + gyroscope corrected by LiDAR loop closures. **An independent estimate, not ground truth** (ENGINEERING_NOTES.md rule 19): LiDAR range noise 15-30 mm, the two LiDARs agree to about 5 mm, and the LiDAR-to-camera mounting offset has never been measured. It shares the robot's wheels + gyroscope with the blend, so for the blend it is not fully independent. Alignment: SE(3) (rotation + shift), never scale (ENGINEERING_NOTES.md section 4 rule 4).*

- LiDAR map: 914 positions, 235.8 m, **59 LiDAR loop closures**; its own end gap 2.138 m on the floor (section 1).
- **The LiDAR estimate FAILED its own check this drive** (agreement.json: "FAILED ITS OWN CHECK: ends 2.14 m from its start although the robot parked on the start mark, with 59 LiDAR loop closures - the differences below measure the reference's drift as much as the camera's"). Its input, the robot's own wheels + gyroscope, ended 3.38 m / -18.5 deg from its start (section 1), and by eye the LiDAR map's walls look doubled in `04_figures/camera_vs_lidar_2026-09-26/drive6_camera_vs_lidar.png`. So a camera-to-LiDAR distance here is disagreement between two estimates that both drifted, not the camera's error. [UNVERIFIED] that the drift sits on the new walkway specifically (reported by the main session; not located here).

| path compared | kind (rule 20) | matched | median | 95th percentile | largest | best-fit scale (NOT applied) | file |
|---|---|---|---|---|---|---|---|
| camera map, corrected (camera_corrected.tum) | map's corrected path | 99.9 % (829) | **0.580 m** | **1.483 m** | 2.434 m | 1.0683 | agreement.json |
| map nodes (= blend), tracking alone (camera.tum) | tracking alone | 95.8 % (1259) | **0.620 m** | **2.243 m** | 2.297 m | 1.0132 | agreement.json |

- Clock shift: estimated -0.04 s, applied 0.0 s (compare_lidar.py applies one only if it improves the fit by > 0.05 m and 20 %).

## 8. Figures, video and 3D map in this folder

| file | what it shows |
|---|---|
| map.png | camera floor map, map nodes at their tracking-alone positions (Node.pose = the blend's) |
| map_corrected.png | camera floor map after the loop closures (Admin.opt_poses) - the map RTAB-Map believes in |
| trajectory.png | where each source put the robot, two panels (rule 20): tracking alone (robot solid; camera and blend dashed; a cross marks the camera outage) and corrected (camera map dashed; LiDAR estimate solid) |
| turns_difference.png | per turn: camera minus robot and blend minus robot, degrees |
| lidar_comparison.png | camera paths over the LiDAR map (LiDAR solid, camera dashed) and the distance to the LiDAR estimate along the route |
| lidar_map.png | the LiDAR's own floor map |
| timelapse.mp4 | **recorded live during the drive**: the live map page (camera map left, LiDAR view right) every 3 s, 496 frames at 10 a second (`timelapse_live_index.csv`; `~/.run_records/s2_static_06/media/recorder.log`: missed 0, write errors 0) |
| timelapse_25/50/75/100.png | stills at 25/50/75/100 % of the timelapse (video frames 124, 248, 372, 495, counted from 0; extracted with ffmpeg) |
| map_3d/drive6_map_3d.ply, map_3d/d6_3d.png | the camera map exported to 3D like drive 4's: `rtabmap-export --cloud --max_range 6 --decimation 4 --voxel 0.03` on a copy of the database (export.log); d6_3d.png = two views (render.py) |

Other files: camera.tum / camera_corrected.tum (+ .meta.json); paths_2hz.csv; turns.csv, turns_replayed.log; numbers.json; crash_split.json; freezes_pass_check.json; power_temperature.json; db_check.txt; closed_check.txt; lidar.tum, wheel.tum, lidar_map.npz, lidar_reference_dense.tum, lidar_comparison/.

## 9. Still open

- N = 1 at these settings: no spread can be quoted (ENGINEERING_NOTES.md section 4).
- Why the LiDAR estimate and the robot's own wheels + gyroscope drifted this drive (3.38 m / -18.5 deg for the robot) is not established; that it happened on the new walkway is [UNVERIFIED] here.
- The camera crash's root cause (third exit of the camera program in two nights) is unknown; the guard now restarts it.
- The camera's path is not independent of the blend in a fused drive (section 1).
