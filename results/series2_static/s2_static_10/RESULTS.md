# Drive 10 (s2_static_10) - results pack: the whole 2nd floor

*27 Sept 2026, Hamilton time: mapping 02:02:42 (camera guard 'watching') to 02:31:15 (autostop closing the map); "park" received 02:30:31, still 60 s, **map closed properly** (`closed_check.txt`: Admin.opt_poses saved). Camera tracker GEN_2, depth NEURAL (`~/.run_records/s2_static_10/camera.log`); FUSION=1 (the camera blended with the robot's wheels and gyroscope (turn-rate sensor) by an EKF (extended Kalman filter, a standard way of mixing sensors)). Pre-drive, as reported by the drive session (not re-measured here): Jetson freshly rebooted (up about 8 min at the start); robot IMU (its motion sensor) tilted 1.8 deg (read from /imu/data at 02:00); gyroscope offset +0.43 deg/min at the start_drive ready check; WiFi roaming helper v2 on both the Jetson and the robot. Made on the Jetson on 27 Sept by `results_packs_2026-09-26/` (project records) (pack_drive10_heavy.sh, analyze_run.py, turns_replay.py, extract_timing.py, freeze_timing_drive7.py, tegra_peaks.py, bridge_freshness_drive10.py; LiDAR half: pack_drive10_lidar.sh, compare_lidar.py, size_fit_agreement.py, robot_wheels_from_robot_bag.py, `03_methods/lidar_drift_2026-09-26/score_f3dof.py`, compare_drives_4_8_9_10.py, deck_figure_drive10.py; this file's writer `write_results_drive10.py`). Every number names the file it comes from; files are in this folder unless a path is given. Nothing here is ground truth (ENGINEERING_NOTES.md rule 19). N = 1.*

**The LiDAR half was added later on 27 Sept.** The first LiDAR replay died with the robot's battery at about 02:40; it was re-run on the robot (Force3DoF, the same method as drive 9) and its products pulled into `lidar_ref_f3dof/` (sha256 checked on arrival). Its `provenance.txt` says "started 2024-06-17 16:30:54 EDT on cpr-a200-0985": the robot had just booted and its clock had not locked yet when the replay started. The positions themselves carry the recording's own time stamps: `lidar.tum` runs 02:00:46 to 02:30:15 Hamilton (947 positions), inside the robot's own recording (`wheel.tum`, 02:00:44 to 02:31:35) and around the camera map's 02:02:10 to 02:31:14; the robot's wheels show it standing still (0.00 m moved before 02:02:27, under 0.02 m of jitter after 02:30:15). So the products are valid; only that one provenance line carries the wrong date.

*Words used below: node = one saved map snapshot; Node.pose = each node's tracking-alone position; Admin.opt_poses = the corrected positions RTAB-Map saves when it shuts down properly; loop closure = the map recognising a place seen before and pulling itself straight; fusion.bag = the drive's recording of every sensor stream; blend = /fused/odometry, the EKF's mix of camera + wheels + gyroscope; tegrastats = NVIDIA's once-a-second load/power/temperature log; quick_check = SQLite's own database-integrity check.*

## 0. At a glance

| item | drive 9 | drive 10 | evidence |
|---|---|---|---|
| duration (map nodes, first to last) | 1734 s | **1744 s** | facts.json `duration_s` |
| distance, camera map corrected | 246.6 m | **251.5 m** | map_corrected_facts.json `path_m_nodes` |
| map snapshots (nodes) / in the corrected graph | 1334 / 893 | **1558 / 860** | facts.json, camera_corrected.tum.meta.json |
| loop closures (pairs) | 305 | **199** (23 recognised-again + 176 nearby) | database Link table types 1, 2 |
| loop closures per 10 m (corrected distance) | 12.4 | **7.9** | the two rows above |
| end of drive | no park, NOT closed | **park, closed properly** | autostop.log, closed_check.txt |
| corrected positions | re-optimised by our script | **RTAB-Map's own (Admin.opt_poses)** | camera_corrected.tum.meta.json `method` |
| camera program | 1 hang (28.3 s) | **0 crashes, 0 hangs** (guard restarts 0) | camera_guard.json |
| camera restarts from the blend's position | 101 | **180** | mapping.log |
| camera tracker silences > 1.5 s | 7 (6 outside the hang) | **1** | freezes_pass_check.json |
| blend without fresh robot data (> 0.5 s) | - (not measured) | **196.8 s = 11.5 % of the drive** | bridge_freshness.json |
| LiDAR yardstick | Force3DoF replay | **Force3DoF replay**, passes its own check (76 LiDAR closures, ends 0.04 m from its start) | agreement.json |
| **camera map vs LiDAR estimate, median / 95th percentile (SE(3), no resizing)** | 0.42 / 0.65 m | **0.69 / 1.15 m** | agreement.json (section 7) |
| best-fit size factor (not applied) / size-fitted median (for information) | 1.052 / 0.13 m | **1.072 / 0.14 m** | agreement.json, size_fit.json (section 7a) |

## 1. How far from home at the end - both kinds (rule 20)

*Plain terms: "tracking alone" = where each source's own step-by-step tracking put the robot, never corrected; "corrected" = where the map puts it after its loop closures. In a fused drive the map's nodes carry the BLEND's position, and the camera's tracking restarts from the blend each time it gives up. The drive ended on its start mark by eye, so a perfect estimate shows about 0; tape-mark parking is only good to about 0.2-0.5 m.*

| kind | source | end-to-start gap | heading gap | path length | file |
|---|---|---|---|---|---|
| tracking alone | map nodes (Node.pose = the blend's position at each node), node 1 to node 1558 | **2.110 m** | - | 257.92 m | facts.json, camera.tum |
| tracking alone | camera tracking, every message (/rtabmap/odom, lost stretches left out; re-seeded by the blend at each restart) | 2.081 m | -4.02 deg | 294.2 m | numbers.json (fusion.bag) |
| tracking alone | blend (/fused/odometry) | 2.111 m | -1.50 deg | 293.6 m | numbers.json |
| tracking alone | robot's wheels + gyroscope as the Jetson received it over WiFi (/robot/ekf_odom, 02:02:27-02:31:20) | 2.027 m | -14.92 deg | 264.5 m | numbers.json (WiFi gaps: section 3) |
| tracking alone | robot's own wheels + gyroscope, **the robot's own recording** (/odometry/filtered, 02:02:27-02:31:35, start to parked) | **2.027 m** | **-15.01 deg** | 265.5 m | robot_wheels_from_robot_bag.json |
| **corrected** | camera map's graph, node 1 to node 1500 (860 graph nodes), RTAB-Map's own Admin.opt_poses, 3D | **0.0327 m** | - | - | camera_corrected.tum.meta.json |
| corrected | every camera node placed by its graph node's correction, node 1 to node 1558, floor plane | 0.032 m | - | 251.51 m | map_corrected_facts.json |
| **corrected** | **LiDAR estimate (Force3DoF replay)**, floor plane | **0.044 m** | - | 263.3 m | lidar.tum, lidar_map_facts.json |
| corrected | LiDAR estimate (Force3DoF replay), 3D | 0.0438 m | - | - | lidar.tum.meta.json |

**Loop closures beside the corrected gaps: camera map 199** (23 recognised-again + 176 nearby re-matches; database Link table types 1 and 2, counted once per pair; camera_corrected.tum.meta.json gives 199). The live monitor's last line: `t=1753s  est=(0.004, -0.032)  pos_err=n/a (no ground truth)  quality=162  resets=180  ACCEPTED_LC=196 (23 global + 173 prox)  rejected_LC=0` (`monitor_STATUS.txt`). Rejected in `mapping.log`: 14. **LiDAR closures: 76** (lidar.tum.meta.json).

- **Closures joining a later node straight to node 1 (the start): 3** (1-489 nearby re-match at 02:10:41, 1-1491 nearby re-match at 02:30:02, 1-1499 nearby re-match at 02:30:14); **2 of them from the last 2 minutes of the drive** (node(s) 1491, 1499, i.e. end-to-start; both in the last 15 s of driving). So the corrected gap largely repeats those end-to-start closures (rule 20): it says the map joined its end to its start, not how far the tracking drifted - that is the tracking-alone rows (about 2.1 m).
- Where the closures fell (newer node's time, the drive cut into four equal quarters): 0 / 37 / 35 / 127. First 02:10:22, last 02:30:15.
  *Plain terms: no closure at all in the first quarter of the drive (until 02:09:26) - [INFERENCE] the first part of the route was new ground, so there was nothing yet to recognise; most closures came in the last quarter, when the robot came back through mapped corridors.*
- **The robot's own wheels + gyroscope ended 2.03 m and -15.0 deg off**, so its heading drifted by about 15 deg over the drive; the LiDAR estimate, built on those same wheels + gyroscope, came home to 0.04 m after its 76 LiDAR loop closures. The Jetson's copy of that stream (over WiFi) gives 2.027 m / -14.92 deg, the same drift: it is the wheels + gyroscope themselves, not the WiFi. *Plain terms: the robot's own sense of direction was off by about 15 degrees by the end; the LiDAR map corrected it by recognising places.*
- 860 of 1558 nodes are in the corrected graph; the rest stay in the database's Node table but out of the map graph (RTAB-Map's memory management).

## 2. Every turn: robot vs camera vs blend

The live turn watcher's rule replayed after the drive over fusion.bag by `turns_replay.py` (unchanged). Output: turns_replayed.log -> turns.csv. *95th percentile = the value 95 % of turns stay under. The robot's wheels + gyroscope, as received over WiFi, is the comparison for short turns (not ground truth); a turn during a WiFi gap has a less reliable robot value (section 3).*

| summary (29 turns) | median | 95th percentile | largest | within 1 deg |
|---|---|---|---|---|
| abs(blend - robot) | 0.20 deg | 1.36 deg | 1.9 deg | 24 of 29 |
| abs(camera - robot) | 3.40 deg | 25.28 deg | 53.1 deg | 3 of 29 |

Turns with no camera value (camera lost for the whole turn): 1. Figure: `turns_difference.png`.

| turn | ended | direction | s | robot deg | camera deg | blend deg | camera lost | blend - robot | camera - robot |
|---|---|---|---|---|---|---|---|---|---|
| 1 | 02:05:28 | right | 0.2 | -0.4 | -2.1 | -0.5 | 0 % | -0.0 | -1.7 |
| 2 | 02:05:33 | right | 2.1 | -10.2 | -11.8 | -10.0 | 0 % | 0.2 | -1.6 |
| 3 | 02:05:41 | left | 5.1 | 9.8 | 12.7 | 9.5 | 48 % | -0.3 | 2.9 |
| 4 | 02:06:08 | left | 24.9 | 531.1 | 535.7 | 531.4 | 23 % | 0.2 | 4.6 |
| 5 | 02:07:56 | left | 3.9 | 83.5 | 90.5 | 84.9 | 0 % | 1.4 | 7.0 |
| 6 | 02:09:17 | left | 29.2 | 501.9 | 506.6 | 503.2 | 13 % | 1.3 | 4.7 |
| 7 | 02:12:00 | left | 20.2 | 266.5 | 233.1 | 267.1 | 12 % | 0.7 | -33.4 |
| 8 | 02:12:02 | left | 0.4 | 1.8 | None | 1.7 | 100 % | -0.1 | None |
| 9 | 02:12:08 | left | 3.5 | 72.1 | 125.2 | 72.6 | 26 % | 0.5 | 53.1 |
| 10 | 02:13:05 | left | 9.2 | 172.5 | 176.9 | 172.7 | 45 % | 0.2 | 4.4 |
| 11 | 02:15:58 | right | 4.9 | -81.1 | -86.3 | -82.1 | 0 % | -0.9 | -5.2 |
| 12 | 02:16:08 | right | 1.9 | -0.9 | -2.0 | -0.9 | 0 % | -0.0 | -1.1 |
| 13 | 02:16:15 | left | 3.4 | 4.5 | 8.4 | 4.4 | 0 % | -0.0 | 3.9 |
| 14 | 02:17:07 | right | 3.6 | -79.7 | -85.2 | -80.9 | 0 % | -1.2 | -5.5 |
| 15 | 02:17:39 | left | 4.2 | 8.0 | 10.9 | 7.4 | 0 % | -0.6 | 2.9 |
| 16 | 02:17:41 | right | 0.9 | -7.9 | -10.4 | -7.8 | 0 % | 0.1 | -2.5 |
| 17 | 02:17:58 | right | 13.7 | 97.0 | 93.9 | 96.4 | 0 % | -0.6 | -3.1 |
| 18 | 02:18:15 | right | 2.3 | -1.5 | -3.8 | -1.4 | 0 % | 0.1 | -2.3 |
| 19 | 02:19:12 | left | 4.1 | 83.7 | 86.0 | 83.7 | 0 % | -0.0 | 2.3 |
| 20 | 02:20:42 | left | 3.0 | 75.9 | 80.2 | 76.4 | 0 % | 0.4 | 4.3 |
| 21 | 02:23:30 | left | 5.0 | 81.4 | 91.6 | 80.5 | 15 % | -0.9 | 10.2 |
| 22 | 02:24:36 | right | 0.4 | -2.5 | -2.7 | -0.6 | 0 % | 1.9 | -0.2 |
| 23 | 02:24:49 | left | 7.5 | 3.6 | 4.1 | 2.3 | 0 % | -1.3 | 0.5 |
| 24 | 02:24:58 | left | 5.9 | 75.5 | 82.3 | 75.7 | 0 % | 0.2 | 6.8 |
| 25 | 02:26:45 | right | 10.1 | -175.7 | -177.5 | -175.8 | 48 % | -0.1 | -1.8 |
| 26 | 02:27:29 | right | 9.0 | -51.8 | -56.8 | -52.0 | 0 % | -0.2 | -5.0 |
| 27 | 02:27:42 | right | 7.5 | -4.5 | -7.8 | -5.2 | 0 % | -0.7 | -3.3 |
| 28 | 02:29:59 | left | 7.0 | 166.6 | 167.4 | 166.4 | 9 % | -0.2 | 0.8 |
| 29 | 02:30:12 | right | 0.1 | 0.3 | -3.2 | 0.1 | 0 % | -0.1 | -3.5 |

## 3. The WiFi link to the robot (the bridge) - how long the blend ran without fresh robot data

*Sources: `~/.run_records/s2_static_10/bridge_recv.log` (its clock is UTC, converted here), its FINAL counters; `bridge_freshness.json` / `bridge_freshness.png` (bridge_freshness_drive10.py over fusion.bag). Rule: the blend's EKF ignores a robot input older than 0.5 s (`sensor_timeout: 0.5`, catkin_ws/src/sidewalk_slam/config/ekf_fused.yaml:49), so a moment counts as "no fresh robot data" when the newest robot message the blend received was more than 0.5 s old. Window 02:02:42-02:31:15 (1713 s).*

*Plain terms: the robot sends its wheel readings 10 times a second and its gyroscope 20 times a second over WiFi. When nothing arrived for half a second, the blend was steering on the camera alone.*

| robot input to the blend | no fresh data (> 0.5 s old) | stretches | > 2 s old | > 5 s old | longest stretch |
|---|---|---|---|---|---|
| wheels (/ekf_in/wheel_odom) | **200.6 s (11.7 %)** | 48 | 158.1 s (9.2 %) | 110.7 s (6.5 %) | 25.8 s |
| gyroscope (/ekf_in/imu) | **224.9 s (13.1 %)** | 59 | 170.9 s (10.0 %) | 114.8 s (6.7 %) | 26.4 s |
| neither wheels nor gyroscope | **196.8 s (11.5 %)** | 46 | 154.6 s (9.0 %) | 104.4 s (6.1 %) | 25.8 s |

- Longest stretches with neither wheels nor gyroscope fresh: 02:23:35 (25.8 s); 02:13:53 (17.2 s); 02:16:36 (17.1 s); 02:25:57 (16.0 s); 02:18:43 (15.0 s); 02:09:57 (13.7 s); 02:14:36 (12.5 s); 02:23:10 (10.0 s).
- Wheel messages received per 10 s window: median 10.0 a second, lowest 0.0 a second; 9 of 172 windows under 2 a second, 6 under 1 a second (nominal 10).
- **Link down (no bytes for 1.5 s): 19 time(s)**, at 02:05:00, 02:08:37, 02:08:47, 02:09:50, 02:09:58, 02:11:20, 02:11:32, 02:11:47, 02:12:18, 02:13:55, 02:14:47, 02:16:39, 02:17:52, 02:18:46, 02:23:11, 02:23:36, 02:24:45, 02:25:58, 02:27:22 Hamilton; connections 20 (bridge_recv.log FINAL `connects`).

| stream | robot read | robot sent | published on the Jetson | % of sent | missed while down | too late (stale, > 300 ms) | lost |
|---|---|---|---|---|---|---|---|
| /husky_velocity_controller/odom | 18145 | 16026 | 15411 | 96.2 % | 1884 | 403 | 158 |
| /imu/data | 35252 | 31113 | 29952 | 96.3 % | 3653 | 764 | 323 |
| /imu/data_raw | 35252 | 31142 | 29966 | 96.2 % | 3628 | 770 | 328 |
| /odometry/filtered | 90757 | 80172 | 77122 | 96.2 % | 9345 | 2001 | 851 |

*"missed while down" = readings the robot made while the link was down; "stale" = arrived but more than 300 ms old, so thrown away (the bridge receiver's rule). The robot's own copy of all these readings is in its own recording, unaffected by WiFi.*
- Where it happened: `bridge_freshness.png` (right panel, the blend's own tracking-alone path, red where it had no fresh robot data). Read off that figure: the gaps sit on the far corridors of the loop (the bottom corridor and the right-hand corridor as drawn), hardly any on the corridor past the start - consistent with the WiFi flickering far from the lab [INFERENCE: no WiFi signal strength was logged along the route].

## 4. Camera tracking: losses, restarts, freezes, crashes

- **Camera program: 0 crashes, 0 hangs**: camera guard ended with restarts 0, state "ended" (`~/.run_records/s2_static_10/camera_guard.json`).
- **Lost** (fusion.bag, `/rtabmap/odom`: all-zero pose or covariance 9999 - the tracker's "I am lost" value): **15 stretches, 58.4 s in total, longest 12.6 s, 3.1 % of messages** (numbers.json).
- **Restarts from TF** (`mapping.log`, "Odometry automatically reset to latest odometry pose available from TF" - the camera's tracking gave up and restarted from the blend's position): **180**, in 10 burst(s) (restarts within 5 s of each other grouped). Largest bursts: 02:05:40-02:05:44 (49); 02:26:37-02:26:42 (36); 02:05:58-02:06:01 (27); 02:12:58-02:13:01 (14); 02:11:18-02:11:29 (13). The monitor's last line says resets=180 (`monitor_STATUS.txt`). Drive 9: 101 over 1734 s; drive 10: 180 over 1744 s.
  All bursts: 02:05:40-02:05:44 (49); 02:05:58-02:06:01 (27); 02:08:42-02:08:46 (12); 02:09:04-02:09:08 (7); 02:11:18-02:11:29 (13); 02:11:58-02:12:05 (8); 02:12:58-02:13:01 (14); 02:23:21-02:23:25 (13); 02:26:37-02:26:42 (36); 02:29:56 (1).
- **Freezes** (pass_check.py from `03_methods/jetson_stalls_drive3_2026-09-25/` -> freezes_pass_check.json): camera tracker silences > 1.5 s: **1** (2.33 s); 'ZED Diagnostic' pauses > 2 s: 1 (3.33 s); database saves >= 5.5 s: 0; control, camera gyroscope longest gap 0.034 s; map updates parsed 1506 of 1558 nodes. Verdict as printed: **FAIL** (pass line = zero silences).
- Where the tracker silences come from (`freeze_timing_drive7.py` -> freeze_timing.json): 1 silences; 1 overlap a 'ZED Diagnostic' pause of the camera program's picture loop; 0 overlap a database save of 1.5 s or more.
  - 02:07:22, 2.33 s: diagnostic pause yes, database save none, restart from TF within 5 s after: no
- Silent feature-detector downgrade (`"cannot be used"` in mapping.log, ENGINEERING_NOTES.md section 2.6): 0 lines.
- Database integrity (`db_check.txt`, quick_check, read-only): `PASS ~/slam_series2/s2_static_10.db: quick_check=['ok'], 1558 map nodes, 1.5 s`

## 5. Blend heading while standing still (reported, not a pass line)

stops 60 (678 s still); signed sum **-3.907 deg**; sum of each stop's net change 6.189 deg; largest single stop -4.078 deg (numbers.json). Rule: robot wheel odometry speed and turn rate both zero, below 1e-6 - the log holds 1e-16 float noise, not 0.0 (nearest wheel message within 0.3 s). *Stillness is judged from the robot's wheel stream as received over WiFi; during WiFi gaps (section 3) a stop may be missed.*

## 6. Processor, power and temperature

*Source: `~/.run_records/s2_static_10/tegrastats.log` (once a second; its clock is UTC, converted), window 02:02:42 to 02:31:15 Hamilton, read by tegra_peaks.py -> power_temperature.json. Drive 9's figures: its pack's power_temperature.json.*

| quantity | drive 10 peak | when | drive 10 median | drive 9 peak / median |
|---|---|---|---|---|
| total power (four named supply rails) | **32.8 W** | 02:26:56 | 29.5 W | 35.7 / 29.9 W |
| hottest point on the chip (tj) | **62.2 C** | 02:29:29 | - | 62.1 C / - |
| processor temperature | 62.2 C | 02:29:29 | - | 62.2 C / - |
| graphics processor temperature | 55.9 C | 02:11:31 | - | 55.9 C / - |
| graphics processor load | 98 % | 02:26:35 | 22 % | 99 / 22 % |
| processor load, mean of the 12 cores | 50.2 % | 02:22:43 | 39.8 % | 98.5 / 40.6 % |
| memory free, lowest | 55.3 % (13.3 GB used) | 02:31:01 | - | 54.1 % / - |

- tegrastats skips over 2 s: 02:07:25 (3 s); samples 1666.
- *Plain terms: the memory guard (earlyoom) kills the mapping program if free memory falls under 15 %; the lowest here is shown above.*
- RTAB-Map, time to process each map snapshot (database Statistics table, read-only): **median 544.2 ms, 95th percentile 750.9 ms, largest 5836.5 ms over 1558 nodes** (numbers.json). Drive 9: 599.7 / 838.5 / 5851.5 ms over 1334 nodes.

## 7. Agreement with the LiDAR estimate

*The reference is the **LiDAR estimate (Force3DoF replay)**: the robot's LiDAR mapping (a colleague's work: self_navigation `rtabmap_3d.launch`) replayed on the robot after the drive with ONE parameter changed, `Reg/Force3DoF=true` (the map is held flat on the floor, so a wrongly-read gravity direction cannot tip it; `03_methods/lidar_drift_2026-09-26/FINDINGS.md` section 11). No colleague-mode replay exists for drive 10. It is the robot's wheels + gyroscope corrected by LiDAR loop closures. **An independent estimate, not ground truth** (ENGINEERING_NOTES.md rule 19): LiDAR range noise 15-30 mm, the two LiDARs agree to about 5 mm, and the LiDAR-to-camera mounting offset has never been measured. It shares the robot's wheels + gyroscope with the blend, so for the blend it is not fully independent. Alignment: SE(3) (rotation + shift), never resizing (ENGINEERING_NOTES.md section 4 rule 4).*

*Plain terms: "agreement" = at each moment, how far apart the camera map and the LiDAR map put the robot, after turning and sliding one path once to fit the other. Median = the typical distance; 95th percentile = the distance 95 % of moments stay under.*

- LiDAR estimate (Force3DoF replay): 947 positions, 263.3 m, **76 LiDAR loop closures** (drive 9: 379); its own end gap 0.044 m on the floor, 0.044 m in 3D (section 1). Its own check (agreement.json, compare_lidar.py: ends within 0.5 m of its start AND at least one LiDAR closure): "passes its own check: ends 0.04 m from its start, 76 LiDAR loop closures".
- **Flatness check of the LiDAR estimate** (`03_methods/lidar_drift_2026-09-26/score_f3dof.py` -> lidar_flatness_check.json): largest tilt 0.0 deg, height range 0.00 m -> **PASS** (pass line: tilt under 3 deg and height range under 0.3 m). *Under Force3DoF flatness is built in, so passing is expected; the evidence that this yardstick is sound is its closures and its end gap.* Its own quick comparison there (node positions, 2D fit, a different method from the table below): vs camera map 0.68 / 1.14 m, vs robot's wheels + gyroscope 0.77 / 1.17 m (median / 95th percentile).

| path compared | kind (rule 20) | matched | median | 95th percentile | largest | best-fit size factor (NOT applied) | drive 9 median / 95th pct | file |
|---|---|---|---|---|---|---|---|---|
| camera map, corrected (camera_corrected.tum) | map's corrected path | 100.0 % (860) | **0.685 m** | **1.151 m** | 1.242 m | 1.0717 | 0.419 / 0.648 m | agreement.json |
| map nodes (= blend), tracking alone (camera.tum) | tracking alone | 96.3 % (1500) | **0.718 m** | **1.228 m** | 1.233 m | 1.0312 | 0.278 / 0.577 m | agreement.json |

- Clock shift: estimated +0.26 s, applied 0.0 s (compare_lidar.py applies one only if it improves the fit by > 0.05 m and 20 %).
- Where the distance grows (`lidar_comparison.png`, lower right): the camera map's distance from the LiDAR estimate rises and falls in a regular pattern, largest at the far corners of the loop and smallest near the start - the pattern a SIZE difference makes after a rotation + shift fit, rather than a bend in one place [INFERENCE from the figure; section 7a measures it].

### 7a. For information, size fitted (NOT the result)

*Why this is here: a taped 15.00 m test (`01_runs/series2_static/s2_scale_01/RESULTS.md`) found the robot's wheels read about 6 % long and the camera within 1 %. The LiDAR estimate takes its size from the wheels, so part of the SE(3) distance above is the LiDAR estimate being too large, not the camera map's shape. Below, the same comparison is repeated allowing one size change (Sim(3)). For information only: the camera's size is real and is not to be fitted away (ENGINEERING_NOTES.md section 4 rule 4).* Made by `size_fit_agreement.py` -> size_fit.json, same reference, same 50 ms pairing, same fitting code as compare_lidar.py; its rigid (SE(3)) run reproduces agreement.json exactly (control: passed).

| path compared | pairs | SE(3) median / 95th pct (the result) | size fitted: median / 95th pct | size factor applied to the camera path | drive 9 size fitted |
|---|---|---|---|---|---|
| camera map, corrected | 860 | 0.685 / 1.151 m | 0.140 / 0.294 m (for information, size fitted) | 1.0717 | 0.126 / 0.281 m at 1.0515 |
| map nodes (= blend), tracking alone | 1500 | 0.718 / 1.228 m | 0.650 / 1.106 m (for information, size fitted) | 1.0312 | 0.132 / 0.403 m at 1.0274 |

*Plain terms: a size factor of 1.072 = the corrected camera path would have to be stretched by 7.2 % to fit the LiDAR estimate best. The tape test says the robot's wheels + gyroscope read 1.064 times the true distance and the camera 0.993 times, and the corrected camera map 0.989 times (legs spread about +-1 %), so the size difference expected from the tape alone is about 1.064 / 0.993 = 1.071 to 1.064 / 0.989 = 1.076. Drive 10's 1.0717 is consistent with that, well inside the one-drive spread of the size factor (0.015, drives 3-5; compare_drives_4_8_9_10.json `spreads.scale`). [INFERENCE] so most of the 0.69 m median is the LiDAR estimate's size (from the wheels), not the camera map's shape: allowing for size, the camera map sits a median 0.14 m from the LiDAR estimate, about the same as drive 9's 0.13 m.*

### 7b. The side-by-side picture: camera map and LiDAR map side by side

`d10_camera_vs_lidar_side_by_side.png` (deck_figure_drive10.py): left, the camera map from above (the 3D point cloud, points 0.15-2.0 m above the floor, 2503352 of 4517220 points, 5 cm cells); middle, the LiDAR estimate's map (Force3DoF replay, walls 0.15-2.0 m up) - both in the LiDAR map's frame at the same scale, the camera map moved by rotation + shift only (fit residual 0.0000 m, rotation -21.53 deg); right, the last frame the live page recorded of the LiDAR view during the drive (`000568.png`), i.e. the colleague's default settings with the tilt from the robot's IMU, in its own framing and scale.

- **What it shows (by eye, not measured):** the live default-settings LiDAR view has doubled and fanned-out walls, as seen during the drive; the Force3DoF replay of the same recording has single, straight walls; the camera map's walls are single too. [INFERENCE] the doubling in the live view is the tilt from the IMU (drive 9's investigation, FINDINGS.md section 11), not the LiDAR itself - holding the map flat removes it. The camera map is visibly smaller than the LiDAR map (its dashed path sits inside the solid one at the far corners): that is the size factor of section 7a.

## 8. Distances - from the camera and from the wheels

*Each source's own path length. "2 Hz" = positions every 0.5 s, straight steps summed (paths_2hz.csv). The tape test (s2_scale_01) found the robot's wheels read about 6 % long and the camera within 1 %.*

| source | what it is | path length | drive 9 | file |
|---|---|---|---|---|
| **camera map, corrected** | camera, after its loop closures (every node) | **251.5 m** | 246.6 m | map_corrected_facts.json |
| map nodes, tracking alone | the blend's position at each node | 257.9 m | 251.9 m | facts.json |
| camera tracking, 2 Hz (re-seeded by the blend at restarts) | - | 264.0 m | - | paths_2hz.csv |
| blend, 2 Hz | - | 259.9 m | - | paths_2hz.csv |
| robot's wheels + gyroscope as received over WiFi, 2 Hz (gaps bridged by straight steps) | - | 262.9 m | - | paths_2hz.csv |
| **robot's own wheels + gyroscope, 2 Hz, the robot's own recording** | wheels (what the LiDAR map is built on), start to parked | **263.4 m** | 255.1 m | robot_wheels_from_robot_bag.json |
| robot's own wheels + gyroscope, every message, the robot's own recording | same, straight steps between every message | 265.5 m | 256.9 m | robot_wheels_from_robot_bag.json |
| LiDAR estimate (Force3DoF replay) | wheels + gyroscope corrected by LiDAR closures (its map nodes) | 263.3 m | 255.0 m | lidar_map_facts.json |
| robot's wheels + gyroscope x 0.940 (for information: the tape's correction) | wheels (robot's own recording, 2 Hz), resized by the taped test | 247.6 m | 239.8 m | computed here |

**Distance to quote for drive 10: about 252 m by the camera map** (corrected; the wheels say 263 m, which the tape shows reads about 6 % long - resized by the tape, 248 m).

## 9. Drives 4 / 8 / 9 / 10 - did the whole-floor drive keep the quality of the short drive and of drive 9?

*Made by `compare_drives_4_8_9_10.py` -> compare_drives_4_8_9_10.md / .json, shared_route.json, shared_route.png (a copy of drive 9's compare_drives_4_8_9.py, which is left as it was). Drive 4 = the short reference drive (85 m), drive 9 = the first longer drive (247 m), drive 8 = drive 4's route repeated (for information). Drive 10's route lined up on drive 9's needed a constraint: the floor's corridors are nearly symmetric about the start, and an unconstrained fit turned it by about -90 deg; the heading is taken from both drives' clear alignment on drive 4 (shared_route.json `pairs_note_9_vs_10`).*

*Plain terms for the verdict lines: each row states how much one drive's number is expected to vary by chance ("spread"); "agrees" = the two drives differ by less than that; "differs by N times" = N times more (3 or more is a real difference, 1-3 is a hint).*

**In plain words.** Drive 10's camera map came home to 0.03 m with 199 loop closures (3 of them joining the start, two in the last 15 s of driving, 02:30:02 and 02:30:14), and its LiDAR estimate to 0.04 m with 76 closures, although the robot's own wheels + gyroscope that the LiDAR estimate is built on ended 2.03 m / -15 deg off. **Against the LiDAR estimate, drive 10's camera map sits a median 0.69 m away (95th percentile 1.15 m) over its whole 252 m route - more than drive 9 (0.42 / 0.65 m over 247 m) and drive 4 (0.26 / 0.49 m over 85 m): median differs by 2.31 times, 95th percentile differs by 1.32 times against drive 9, median differs by 3.70 times, 95th percentile differs by 1.73 times against drive 4.** [INFERENCE] Most of that is size, not shape: its best-fit size factor is 1.072 (drive 9 1.052, drive 4 1.063; the tape test predicts about 1.07 because the robot's wheels, which give the LiDAR estimate its size, read about 6 % long; the size difference is measured, blaming it on the wheels is inferred), and with one size change allowed (for information only - the camera's size is real) drive 10 reads 0.14 / 0.29 m against drive 9's 0.13 / 0.28 m and drive 4's 0.09 / 0.19 m. On the route shared with drive 4 (82 m of drive 10's route), drive 10 reads 0.48 / 0.75 m against drive 4's 0.26 / 0.49 m on the same part - median 2.0 times, 95th percentile 0.7 times the combined spread. On the route shared with drive 9 (130 m), drive 10 reads 0.66 / 1.09 m against drive 9's 0.43 / 0.66 m - median 2.0 times, 95th percentile 1.1 times the combined spread.

**Verdict, drive 10 against drive 4 (computed): of 17 judged rows about the camera, the blend, the agreement with the LiDAR estimate and the mapping program, 8 agree within the combined uncertainty, 3 differ by 1-3 times it, and 6 differ by more than 3 times it: camera map loop closures per 10 m of its route (8.0 times); start-to-end gap, map nodes tracking alone (= blend) (7.3 times); start-to-end gap, camera tracking alone (0 closures) (6.5 times); agreement with the LiDAR estimate, ON THE ROUTE SHARED WITH DRIVE 4, map nodes (= blend) tracking alone: median (SE(3)) (6.4 times); camera tracking restarts from the blend, per 10 min (5.4 times); agreement with the LiDAR estimate, camera map corrected, whole drive: median (SE(3)) (3.7 times).** Counted separately: the robot's own wheels + gyroscope (0 / 0 / 2 agree / 1-3 times / over 3 times) and the LiDAR estimate itself (1 / 0 / 1).

**Verdict, drive 10 against drive 9 (computed): of 17 judged rows about the camera, the blend, the agreement with the LiDAR estimate and the mapping program, 6 agree within the combined uncertainty, 6 differ by 1-3 times it, and 5 differ by more than 3 times it: start-to-end gap, camera tracking alone (0 closures) (5.5 times); camera map loop closures per 10 m of its route (4.9 times); camera tracking restarts from the blend, per 10 min (4.7 times); start-to-end gap, map nodes tracking alone (= blend) (4.5 times); agreement with the LiDAR estimate, ON THE ROUTE SHARED BY DRIVES 9 AND 10, map nodes (= blend) tracking alone: median (SE(3)) (4.2 times).** Counted separately: the robot's own wheels + gyroscope (0 / 1 / 1 agree / 1-3 times / over 3 times) and the LiDAR estimate itself (1 / 0 / 1).

*How to read this: two drives never give identical numbers. For each row the spread one drive is expected to have is stated, with where it comes from (column "from"); the two are combined as sqrt(s_a^2 + s_b^2). "Agrees within X" = the difference is smaller than that; "differs by N times" = N times larger (3 or more is a real difference). N = 1 drive each; the distance spreads are working figures. "Shared" rows use only the part both drives covered; "whole drive" rows are per 10 min or per 10 m where the number grows with length. Drive 8's column is for information (its yardstick: its own Force3DoF LiDAR replay). The drive 9 vs drive 4 column repeats s2_static_09/compare_drives_4_8_9.md with the same method (a check that nothing moved).*

**Shared route - how it was defined.** camera map corrected path (camera_corrected.tum; drive 9: graph re-optimised by optimize_graph_se2.py; drive 10: Admin.opt_poses), filled in every 5 cm; the later drive lined up on the earlier one by 2D rotation + shift (ICP, pairs < 1.0 m, best of 36 starting headings; drive 10 on drive 9: headings within 30 deg of the one implied by both drives' alignment on drive 4); a point is on the shared route if within 0.5 m of the other drive's route; direction of travel not tested. Drive 10 lined up on drive 4 by -0.8 deg and 0.21 m (start then 0.21 m from drive 4's): **82.5 of its 250.1 m within 0.5 m of drive 4's route; drive 4: 84.7 of 84.7 m (100 %) within 0.5 m of drive 10's.** Drive 10 lined up on drive 9 by +2.0 deg and 0.41 m (start then 0.41 m from drive 9's): **130.0 of its 250.1 m within 0.5 m of drive 9's route; drive 9: 196.4 of 244.8 m (80 %).** Sensitivity (drive 10's length shared with drive 9 at R = 0.3 / 0.5 / 0.75 / 1.0 m): 123.9 m / 130.0 m / 132.8 m / 135.0 m. Figure: `shared_route.png`. Control: the whole-drive figures recomputed here reproduce every agreement.json (passed), and drive 9's shared-route figures against drive 4 reproduce compare_drives_4_8_9.json (passed).

Drive 10 off the route shared with drive 4: camera map corrected vs LiDAR estimate median 0.74 m, 95th percentile 1.17 m; off the route shared with drive 9: 0.70 / 1.19 m.

| row | group | scope | drive 4 | drive 8 (info) | drive 9 | drive 10 | spread per drive | from | drive 10 vs drive 4 | drive 10 vs drive 9 | drive 9 vs drive 4 | note |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| route: shared length with drive 4 (drive's own / drive 4's, within 0.5 m) | context | route | - | 75.2 of 83.0 m / 83.2 of 84.7 m | 98.2 of 244.8 m / 79.1 of 84.7 m | 82.5 of 250.1 m / 84.7 of 84.7 m | - | - | - | - | - | not judged: defines the comparison (shared_route.json) |
| route: drive 10 shared with drive 9 (drive 10's / drive 9's, within 0.5 m) | context | route | - | - | - | 130.0 of 250.1 m / 196.4 of 244.8 m | - | - | - | - | - | not judged: defines the drive 10 vs drive 9 comparison |
| distance driven - camera map corrected / robot's own wheels + gyroscope / LiDAR estimate | context | whole drive | 85.1 m / 90.1 m / 88.8 m | 83.5 m / 88.6 m / 87.3 m | 246.6 m / 256.9 m / 255.0 m | 251.5 m / 265.5 m / 263.3 m | - | - | - | - | - | not judged: a description of the route; the wheels read about 6 % long (tape test) |
| drive duration | context | whole drive | 619 s | 850 s | 1734 s | 1744 s | - | - | - | - | - | not judged: description |
| LiDAR yardstick used | context | whole drive | colleague-mode LiDAR replay | Force3DoF LiDAR replay (for information) | Force3DoF LiDAR replay | Force3DoF LiDAR replay | - | - | - | - | - | not judged: which LiDAR estimate each drive is compared with |
| camera tracker setting / start | context | whole drive | GEN_2, camera freshly started | GEN_2 | GEN_2, fresh Jetson boot | GEN_2, fresh Jetson boot | - | - | - | - | - | not judged: setting |
| start-to-end gap, camera tracking alone (0 closures) | camera and blend | whole drive | 0.25 m | 1.53 m | 0.53 m | 2.08 m | 0.20 m | A | **differs** by 6.47 times (difference 1.83 m, combined 0.28 m) | **differs** by 5.47 times (difference 1.55 m, combined 0.28 m) | differs by 1.00 times (difference 0.28 m, combined 0.28 m) |  |
| start-to-end gap, map nodes tracking alone (= blend) | camera and blend | whole drive | 0.06 m | 0.60 m | 0.83 m | 2.11 m | 0.20 m | A | **differs** by 7.26 times (difference 2.05 m, combined 0.28 m) | **differs** by 4.54 times (difference 1.28 m, combined 0.28 m) | differs by 2.72 times (difference 0.77 m, combined 0.28 m) |  |
| start-to-end gap, robot's own wheels + gyroscope, tracking alone | robot's wheels + gyroscope | whole drive | 0.27 m | 0.98 m | 0.69 m | 2.03 m | 0.20 m | A | **differs** by 6.23 times (difference 1.76 m, combined 0.28 m) | **differs** by 4.71 times (difference 1.33 m, combined 0.28 m) | differs by 1.51 times (difference 0.43 m, combined 0.28 m) |  |
| heading gap, robot's own wheels + gyroscope | robot's wheels + gyroscope | whole drive | +2.3 deg | +8.4 deg | -3.6 deg | -15.0 deg | 3.0 deg | B | **differs** by 4.08 times (difference 17.3 deg, combined 4.2 deg) | **differs** by 2.69 times (difference 11.4 deg, combined 4.2 deg) | differs by 1.39 times (difference 5.9 deg, combined 4.2 deg) |  |
| start-to-end gap, **camera map corrected** (floor) | camera and blend | whole drive | 0.05 m | 0.10 m | 0.01 m | 0.03 m | 0.20 m | A | **agrees** within 0.28 m (difference 0.01 m) | **agrees** within 0.28 m (difference 0.02 m) | agrees within 0.28 m (difference 0.03 m) | closures beside it (rule 20): drive 4 181 (3 join the start); drive 8 145 (0 join the start); drive 9 305 (0 join the start); drive 10 199 (3 join the start); drive 9's corrected path is the graph RE-OPTIMISED (database not closed) |
| camera map loop closures per 10 m of its route | camera and blend | whole drive | 21.3 | 17.4 | 12.4 | 7.9 | 0.6 | C | **differs** by 7.96 times (difference 13.4, combined 1.7) | **differs** by 4.93 times (difference 4.5, combined 0.9) | differs by 5.14 times (difference 8.9, combined 1.7) | counts: 181 / 145 / 305 / 199 |
| start-to-end gap, **LiDAR estimate corrected** (floor) | LiDAR map | whole drive | 0.03 m | 0.02 m | 0.01 m | 0.04 m | 0.20 m | A | **agrees** within 0.28 m (difference 0.01 m) | **agrees** within 0.28 m (difference 0.03 m) | agrees within 0.28 m (difference 0.02 m) | LiDAR closures 75 / 87 / 379 / 76 |
| LiDAR estimate loop closures per 10 m | LiDAR map | whole drive | 8.4 | 10.0 | 14.9 | 2.9 | 0.3 | C | **differs** by 5.40 times (difference 5.6, combined 1.0) | **differs** by 14.39 times (difference 12.0, combined 0.8) | differs by 5.18 times (difference 6.4, combined 1.2) | drive 4's LiDAR estimate is the colleague-mode replay, drives 8-10 the Force3DoF replay |
| LiDAR yardstick passes its own check | LiDAR map | whole drive | yes | yes | yes | yes | - | - | - | - | - | not judged: a pass/fail check |
| camera freezes (tracker silent > 1.5 s), per 10 min | camera and blend | whole drive | 1.0 | 10.6 | 2.1 | 0.3 | 0.3 | D | **agrees** within 1.0 (difference 0.6) | **differs** by 1.89 times (difference 1.7, combined 0.9) | agrees within 1.3 (difference 1.1) | counts: 1 / 15 / 6 / 1; drive 8's crash outage and drive 9's hang outage are not counted as freezes |
| camera tracking restarts from the blend, per 10 min | camera and blend | whole drive | 25.2 | 7.1 | 34.9 | 61.9 | 4.6 | D | **differs** by 5.43 times (difference 36.7, combined 6.8) | **differs** by 4.67 times (difference 27.0, combined 5.8) | differs by 1.61 times (difference 9.8, combined 6.0) | counts: 26 / 10 / 101 / 180 |
| camera tracking lost, seconds per 10 min | camera and blend | whole drive | 25.4 s | 10.1 s | 21.0 s | 20.1 s | 6.9 s | E | **agrees** within 9.8 s (difference 5.3 s) | **agrees** within 9.8 s (difference 0.9 s) | agrees within 9.8 s (difference 4.3 s) | counts only time the tracker SAID it was lost |
| camera program outages | context | whole drive | none | 1, recovered: 15:52:57, back after 12.7 s (camera guard); mapping carried on | 0 crashes; 1 camera HANG, recovered: 21:12:29, back after 28.3 s (camera guard); mapping carried on | none (camera guard: 0 restarts) | - | - | - | - | - | not judged: one event is not a rate |
| turns: abs(camera - robot), median of all turns | camera and blend | whole drive | 2.3 deg | 3.2 deg | 2.6 deg | 3.4 deg | 1.0 deg | F | **agrees** within 1.5 deg (difference 1.1 deg) | **agrees** within 1.5 deg (difference 0.8 deg) | agrees within 1.5 deg (difference 0.3 deg) | turns: 16 / 26 / 34 / 29 |
| turns: abs(blend - robot), median of all turns | camera and blend | whole drive | 0.2 deg | 0.2 deg | 0.3 deg | 0.2 deg | 0.2 deg | F | **agrees** within 0.2 deg (difference 0.0 deg) | **agrees** within 0.2 deg (difference 0.1 deg) | agrees within 0.2 deg (difference 0.1 deg) |  |
| **agreement with the LiDAR estimate, ON THE ROUTE SHARED WITH DRIVE 4**, camera map corrected: median (SE(3)) | agreement with the LiDAR estimate | shared with drive 4 | 0.26 m | 0.25 m | 0.43 m | 0.48 m | 0.08 m | G | **differs** by 1.96 times (difference 0.23 m, combined 0.12 m) | - | - | drive 4's figure is over its part shared with drive 10 (with drive 9: 0.24 m, drive 8: 0.26 m); pairs 283 / 425 / 383 / 241 |
| **agreement with the LiDAR estimate, ON THE ROUTE SHARED WITH DRIVE 4**, camera map corrected: 95th percentile (SE(3)) | agreement with the LiDAR estimate | shared with drive 4 | 0.49 m | 0.47 m | 0.61 m | 0.75 m | 0.27 m | H | **agrees** within 0.38 m (difference 0.26 m) | - | - |  |
| **agreement with the LiDAR estimate, ON THE ROUTE SHARED BY DRIVES 9 AND 10**, camera map corrected: median (SE(3)) | agreement with the LiDAR estimate | shared by 9 and 10 | - | - | 0.43 m | 0.66 m | 0.08 m | G | - | **differs** by 2.01 times (difference 0.23 m, combined 0.12 m) | - | pairs 729 / 447 |
| **agreement with the LiDAR estimate, ON THE ROUTE SHARED BY DRIVES 9 AND 10**, camera map corrected: 95th percentile (SE(3)) | agreement with the LiDAR estimate | shared by 9 and 10 | - | - | 0.66 m | 1.09 m | 0.27 m | H | - | **differs** by 1.12 times (difference 0.43 m, combined 0.38 m) | - |  |
| **agreement with the LiDAR estimate, ON THE ROUTE SHARED WITH DRIVE 4**, map nodes (= blend) tracking alone: median (SE(3)) | agreement with the LiDAR estimate | shared with drive 4 | 0.12 m | 0.14 m | 0.28 m | 0.85 m | 0.08 m | G | **differs** by 6.36 times (difference 0.73 m, combined 0.12 m) | - | - | drive 4's figure is over its part shared with drive 10 (with drive 9: 0.09 m, drive 8: 0.11 m); pairs 483 / 602 / 566 / 703 |
| **agreement with the LiDAR estimate, ON THE ROUTE SHARED WITH DRIVE 4**, map nodes (= blend) tracking alone: 95th percentile (SE(3)) | agreement with the LiDAR estimate | shared with drive 4 | 0.24 m | 0.39 m | 0.36 m | 1.23 m | 0.27 m | H | **differs** by 2.59 times (difference 0.99 m, combined 0.38 m) | - | - |  |
| **agreement with the LiDAR estimate, ON THE ROUTE SHARED BY DRIVES 9 AND 10**, map nodes (= blend) tracking alone: median (SE(3)) | agreement with the LiDAR estimate | shared by 9 and 10 | - | - | 0.27 m | 0.76 m | 0.08 m | G | - | **differs** by 4.18 times (difference 0.48 m, combined 0.12 m) | - | pairs 963 / 927 |
| **agreement with the LiDAR estimate, ON THE ROUTE SHARED BY DRIVES 9 AND 10**, map nodes (= blend) tracking alone: 95th percentile (SE(3)) | agreement with the LiDAR estimate | shared by 9 and 10 | - | - | 0.40 m | 1.23 m | 0.27 m | H | - | **differs** by 2.17 times (difference 0.83 m, combined 0.38 m) | - |  |
| agreement with the LiDAR estimate, camera map corrected, whole drive: median (SE(3)) | agreement with the LiDAR estimate | whole drive | 0.26 m | 0.26 m | 0.42 m | 0.69 m | 0.08 m | G | **differs** by 3.70 times (difference 0.43 m, combined 0.12 m) | **differs** by 2.31 times (difference 0.27 m, combined 0.12 m) | differs by 1.39 times (difference 0.16 m, combined 0.12 m) | whole-drive lengths 85 / 83 / 247 / 252 m |
| agreement with the LiDAR estimate, camera map corrected, whole drive: 95th percentile (SE(3)) | agreement with the LiDAR estimate | whole drive | 0.49 m | 0.51 m | 0.65 m | 1.15 m | 0.27 m | H | **differs** by 1.73 times (difference 0.66 m, combined 0.38 m) | **differs** by 1.32 times (difference 0.50 m, combined 0.38 m) | agrees within 0.38 m (difference 0.16 m) | whole-drive lengths 85 / 83 / 247 / 252 m |
| best-fit size factor, camera map corrected vs LiDAR estimate (not applied) | agreement with the LiDAR estimate | whole drive | 1.063 | 1.062 | 1.052 | 1.072 | 0.015 | I | **agrees** within 0.021 (difference 0.009) | **agrees** within 0.021 (difference 0.020) | agrees within 0.021 (difference 0.012) |  |
| for information, size fitted (Sim(3)): camera map corrected, whole drive, median | for information | whole drive | 0.09 m | 0.08 m | 0.13 m | 0.14 m | 0.08 m | J | **agrees** within 0.12 m (difference 0.05 m) | **agrees** within 0.12 m (difference 0.01 m) | agrees within 0.12 m (difference 0.03 m) |  |
| for information, size fitted (Sim(3)): camera map corrected, whole drive, 95th percentile | for information | whole drive | 0.19 m | 0.18 m | 0.28 m | 0.29 m | 0.27 m | K | **agrees** within 0.38 m (difference 0.10 m) | **agrees** within 0.38 m (difference 0.01 m) | agrees within 0.38 m (difference 0.09 m) |  |
| RTAB-Map time per map snapshot, median | processor | whole drive | 510 ms | 515 ms | 600 ms | 544 ms | 49 ms | L | **agrees** within 70 ms (difference 34 ms) | **agrees** within 70 ms (difference 56 ms) | differs by 1.29 times (difference 90 ms, combined 70 ms) |  |
| total power, median | processor | whole drive | 29.8 W | 29.6 W | 29.9 W | 29.5 W | - | - | - | - | - | not judged: the drive-to-drive spread of drives 4, 5 (camera-alive part) and 6 (0.3 W) comes from 3 drives and is too small to trust as a yardstick |
| hottest point on the chip (tj), peak | processor | whole drive | 61.6 C | 61.5 C | 62.1 C | 62.2 C | - | - | - | - | - | not judged: the drive-to-drive spread of drives 4, 5 (camera-alive part) and 6 (0.1 C) comes from 3 drives and is too small to trust as a yardstick |
| processor load, mean of 12 cores, median | processor | whole drive | 41.2 % | 39.3 % | 40.6 % | 39.8 % | - | - | - | - | - | not judged: the drive-to-drive spread of drives 4, 5 (camera-alive part) and 6 (0.2 %) comes from 3 drives and is too small to trust as a yardstick |

Spread sources (column "from"):
- **A** = where the robot parked: s2_scale_01 found stops at one mark up to 0.19 m apart even with a wheel stop (its RESULTS.md); drives 4, 8, 9 and 10 ended on the start mark by eye (tape-mark parking is only +-0.2-0.5 m)
- **B** = parking heading by eye, working figure (not measured)
- **C** = counting spread, the square root of each count (Poisson, the least spread a count of chance events has), scaled
- **D** = counting spread, the square root of each count (Poisson, the least spread a count of chance events has) (at least 1), scaled
- **E** = drive-to-drive spread (sample standard deviation) of lost seconds per 10 min, drives 3, 4, 5, 6
- **F** = drive-to-drive spread (sample standard deviation) of the per-turn median, drives 4, 5, 6
- **G** = drive-to-drive spread (sample standard deviation) of the camera map's whole-drive agreement on drives 3, 4, 5, whose LiDAR estimate passed its own check; different routes, so it may overstate the spread for a repeat
- **H** = drive-to-drive spread (sample standard deviation) of the camera map's whole-drive agreement (95th percentile) on drives 3, 4, 5, whose LiDAR estimate passed its own check; different routes, so it may overstate the spread for a repeat
- **I** = drive-to-drive spread (sample standard deviation) of drives 3, 4, 5 (valid LiDAR estimates)
- **J** = drive-to-drive spread (sample standard deviation) of the camera map's whole-drive agreement on drives 3, 4, 5, whose LiDAR estimate passed its own check; different routes, so it may overstate the spread for a repeat (borrowed)
- **K** = drive-to-drive spread (sample standard deviation) of the camera map's whole-drive agreement (95th percentile) on drives 3, 4, 5, whose LiDAR estimate passed its own check; different routes, so it may overstate the spread for a repeat (borrowed)
- **L** = drive-to-drive spread (sample standard deviation) of drives 4, 5, 6 (numbers.json processor_rtabmap)

Caveats: (1) the 0.20 m for tracking-alone gaps is only the parking floor; tracking drift varies far more from drive to drive and grows with route length, so "differs" on those rows between a 85 m and a 250 m drive is expected and weak. (2) The agreement spreads come from whole drives on different routes (drives 3-5); the shared-route rows borrow them. (3) The shared-route agreement uses the whole-drive SE(3) alignment. (4) Drive 4's yardstick is the colleague-mode LiDAR replay, drives 9 and 10 the Force3DoF replay: one parameter apart (FINDINGS.md section 11). (5) Drive 9's corrected camera path is the re-optimised graph (its database was never closed); drive 10's is RTAB-Map's own saved result. (6) Counts use the Poisson floor, the least spread a count can have. (7) The size factor is the LiDAR estimate's size as much as the camera's: the LiDAR estimate takes its size from the robot's wheels, which the tape test found about 6 % long; the size-fitted rows are for information, never the result (ENGINEERING_NOTES.md section 4 rule 4).

## 10. Figures, video and 3D map in this folder

| file | what it shows |
|---|---|
| map.png | camera floor map, map nodes at their tracking-alone positions (Node.pose = the blend's) |
| map_corrected.png | camera floor map after the loop closures (RTAB-Map's own Admin.opt_poses) |
| trajectory.png | where each source put the robot, two panels (rule 20): tracking alone (robot's wheels + gyroscope as received over WiFi solid; camera and blend dashed) and corrected (camera map dashed, LiDAR estimate solid) |
| lidar_comparison.png | the LiDAR map with the camera's paths drawn on it, and how far apart they are along the route (compare_lidar.py) |
| d10_camera_vs_lidar_side_by_side.png | **the side-by-side picture** (section 7b): camera map and LiDAR map from above, same scale, plus the last live LiDAR frame (default settings) |
| shared_route.png | the parts of the route drive 10 shared with drives 4 and 9 (section 9) |
| lidar_map.png | the LiDAR estimate's map (Force3DoF replay) with its path |
| turns_difference.png | per turn: camera minus robot and blend minus robot, degrees |
| bridge_freshness.png | robot messages reaching the blend over time, and where on the route the blend had no fresh robot data |
| timelapse.mp4 | **recorded live during the drive** (the live map page every 3 s: camera map left, LiDAR view right), built by the recorder at park: 568 frames at 10 a second (`timelapse_live_index.csv`, `~/.run_records/s2_static_10/media_recorder.out`) |
| timelapse_25/50/75/100.png | stills at 25/50/75/100 % of the timelapse (ffmpeg) |
| map_3d/ | 3D point cloud of the camera map (`drive10_map_3d.ply`), `d10_3d.png` (two 3D views), `d10_topdown.png` (from above, robot path dashed); points placed at RTAB-Map's own saved corrected positions (`rtabmap-export --opt 2`, identical to camera_corrected.tum on all 860 nodes); `map_3d/README.txt` |

Other files: camera.tum / camera_corrected.tum (+ .meta.json); lidar.tum (+ .meta.json), wheel.tum (+ .meta.json), lidar_reference_dense.tum, lidar_map.npz, lidar_map_facts.json, lidar_comparison/ (evaluate_trajectory.py outputs), agreement.json, size_fit.json, lidar_flatness_check.json, robot_wheels_from_robot_bag.json, compare_drives_4_8_9_10.md/.json, shared_route.json, d10_camera_vs_lidar_side_by_side.json; lidar_ref_f3dof/ (the replay's products as pulled from the robot, with provenance.txt and logs); paths_2hz.csv; turns.csv, turns_replayed.log; numbers.json; facts.json, map_corrected_facts.json; freezes_pass_check.json; freeze_timing.json; power_temperature.json; bridge_freshness.json; db_check.txt; closed_check.txt.

## 11. Still open

- **Size**: the LiDAR estimate is 7.2 % larger than the camera map (the camera map 6.7 % smaller; section 7a); the tape test attributes this to the wheels, but the LiDAR-to-camera mounting offset has never been measured, and no drive has a measured true length. A taped long leg on this floor would settle which map has the right size.
- **LiDAR estimate: 76 closures over 263 m** (drive 9: 379 over 255 m) - fewer per metre; it still passes its own check (0.04 m end gap).
- **WiFi**: the blend ran 11.5 % of the drive without fresh robot data (section 3); the turn comparison and the stillness check use the robot stream as received, so they are weaker where it was missing.
- **180 camera restarts from the blend's position** (drive 9: 101): the camera's own tracking gave up often; the map still closed 199 loops.
- N = 1: one drive of the whole floor, not five repeats (ENGINEERING_NOTES.md section 4 rule 2).
- The camera's path is not independent of the blend in a fused drive (section 1).
