# Drive 9 (s2_static_09) - results pack: the first longer drive

*26 Sept 2026, Hamilton time: live recorder started 20:52:40, autostop armed 20:52:41, camera HUNG 21:12:29.2-21:12:57.5 (restarted by the camera guard); the robot's data stopped reaching the Jetson at the 21:15:59 WiFi change, nothing got through from about 21:17:29, and the Jetson was power-cycled (after 21:21:06, its last log line), so **"park" never came and the map was NOT closed properly** (the autostop's last line, 21:21:06: "waiting for 'park'"). Fresh Jetson boot before the drive; camera tracker GEN_2; FUSION=1 (the camera blended with the robot's wheels and gyroscope (turn-rate sensor) by an EKF (extended Kalman filter, a standard way of mixing sensors)); gyroscope re-zeroed to 0.14 deg/min and robot IMU (its motion sensor) tilted 1.8 deg at the start (both as reported by the drive session's pre-drive checks, not re-measured here). Route: drive 4's route plus new corridors (98.2 of its 244.8 m within 0.5 m of drive 4's route, shared_route.json). Made on the Jetson on the night of 26 Sept by `results_packs_2026-09-26/` (project records) (pack_drive9_heavy.sh, analyze_run.py, turns_replay.py, extract_timing.py, crash_split_drive9.py, timelapse_pipe_drive9.py, tegra_peaks.py, freeze_timing_drive7.py, compare_lidar.py, size_fit_agreement.py, compare_drives_4_8_9.py, this file's writer `write_results_drive9.py`). Every number names the file it comes from; files are in this folder unless a path is given. Nothing here is ground truth (ENGINEERING_NOTES.md rule 19). N = 1.*

*Words used below: node = one saved map snapshot; TF = ROS's shared table of where each part of the robot is; Node.pose = each node's tracking-alone position; Admin.opt_poses = the corrected positions RTAB-Map saves when it shuts down properly (drive 9 has none); re-optimised = the same correction recomputed from the database's own links by `tools/db/optimize_graph_se2.py`; fusion.bag = the drive's recording of every sensor stream (drive 9's was never closed: fusion.bag.active, read with its index rebuilt in memory, file untouched); SE(3) = lining two paths up by rotation and shift only; Sim(3) = the same but also allowing a size change; Force3DoF = the LiDAR map replayed with the robot held flat on the floor (no tipping); tegrastats = NVIDIA's once-a-second load/power/temperature log; quick_check = SQLite's own database-integrity check; ffmpeg = a standard video program.*

## 0. What was different from drive 4, and what went wrong (read first)

| item | drive 4 | drive 9 | evidence |
|---|---|---|---|
| camera tracker | GEN_2 | GEN 2 | `~/.run_records/s2_static_09/camera.log` "Positional tracking mode -> ..." |
| duration | 619 s | 1734 s | facts.json `duration_s` |
| route | the reference route | drive 4's route plus new corridors; LiDAR estimate 255 m against drive 4's 89 m | lidar_map_facts.json |
| camera program | no crash | **0 crashes; 1 HANG, recovered in 28.3 s** (camera guard restarts 1) | `camera_outages.csv`, `camera_guard.json` |
| end of drive | "park", map closed properly | **no "park"; Jetson power-cycled; map NOT closed** (integrity check passed) | autostop.log, closed_check.txt, db_check.txt |
| corrected camera positions | RTAB-Map's own (Admin.opt_poses) | **re-optimised** by optimize_graph_se2.py (Admin.opt_poses empty) | camera_corrected.tum.meta.json |
| LiDAR yardstick | the colleague-mode replay | **the Force3DoF replay** (no colleague-mode replay exists for drive 9) | lidar_ref_f3dof/, FINDINGS.md section 11 |

**The camera hang.** `camera_outages.csv` (camera guard): outage logged from **21:12:29.2** (the guard's start time, after 16 s with no pictures; the tracker was already silent from 21:12:13, below), camera restarted and outage closed at **21:12:57.5: 28.3 s** logged; cause "no pictures for 16 s although the camera process runs"; outcome "restarted (1); camera positions back after 0.4 s but LOST (null position)". The system journal of that boot shows nvargus-daemon (NVIDIA's camera service) reporting **"InsufficientMemory"** at 21:12:44 (`network_and_camera_events_journal.txt`). *Plain terms: the camera program did not die; the camera service underneath it could not get memory for new pictures, so the program sat waiting, and the guard restarted it.* The Jetson's own memory was not short (lowest free 54.1 % over the whole drive, section 6), so [INFERENCE] the shortage is in the camera service's own picture-buffer memory, not the system's, and not in our code. The camera tracker (`/rtabmap/odom`) was silent longest 43.06 s near the hang (21:12:13 to 21:12:56; crash_split.json).

| | start to hang | during the hang | back to end | source |
|---|---|---|---|---|
| distance, robot's own wheels + gyroscope (/robot/ekf_odom; ends 21:15:59, when its data stopped reaching the Jetson) | 181.1 m | 6.2 m | 42.2 m | crash_split.json |
| distance, blend (/fused/odometry) | 178.7 m | 6.2 m | 69.9 m | crash_split.json |
| map snapshots (nodes) | 1006 | 1 | 327 | database Node table |

*Distances: positions every 0.5 s, straight steps summed (crash_split.json); start = the recording's first message, end = each stream's last message (no park). Plain terms: the robot moved 6.2 m (its own wheels + gyroscope) while the camera was down, so that stretch has no camera pictures in the map - the blend carried the position across it on wheels + gyroscope.*

- Closures touching a node made before the hang: **280 of 305**; closures whose newer node came after the camera was back: 127 (crash_split.json). *Plain terms: the map recognised places from before the hang after the camera came back, so the two halves of the map are tied together by closures.*

**The network loss and the power cycle.** From the same journal (`network_and_camera_events_journal.txt`, Hamilton time first):

    20:55:59 1790470559.133111 jetson wpa_supplicant[481]: wlan0: CTRL-EVENT-CONNECTED - Connection to f4:bd:9e:a1:e7:02 completed [id=0 id_str=]
    21:00:59 1790470859.290101 jetson wpa_supplicant[481]: wlan0: CTRL-EVENT-CONNECTED - Connection to f4:bd:9e:a1:e7:0d completed [id=0 id_str=]
    21:10:59 1790471459.470523 jetson wpa_supplicant[481]: wlan0: CTRL-EVENT-CONNECTED - Connection to f4:bd:9e:a1:c1:8d completed [id=0 id_str=]
    21:12:44 1790471564.532133 jetson nvargus-daemon[2435]: SCF: Error InsufficientMemory:  (propagating from src/services/gl/EGLStreamProducer.cpp, function allocateAndRegis
    21:12:44 1790471564.532133 jetson nvargus-daemon[2435]: SCF: Error InsufficientMemory:  (propagating from src/services/gl/EGLStreamProducer.cpp, function getBuffer(), lin
    21:15:59 1790471759.701413 jetson wpa_supplicant[481]: wlan0: CTRL-EVENT-CONNECTED - Connection to f4:bd:9e:a1:c1:82 completed [id=0 id_str=]
    21:17:29 1790471849.162538 jetson [remote-access service]: relay connection timed out (log line shortened)
    21:18:11 1790471891.535302 jetson [remote-access service]: retrying the name look-up for its relay (log line shortened)

*Plain terms: the Jetson's WiFi changed access point 4 times during the drive, each on a 5-minute mark (20:55:59, 21:00:59, 21:10:59, 21:15:59). **At the 21:15:59 change the robot's data stopped reaching the Jetson for good**: the last robot message in the Jetson's recording is at 21:15:59 (the bridge log's last link-down: 21:16:00, section 3), and from about 21:17:29 nothing got through at all (the remote-access service used at the time to reach the Jetson lost its relay; name look-ups timed out). The camera kept running on the Jetson: the last camera-tracker and blend messages readable in the unclosed recording are at 21:20:56 / 21:20:56 (crash_split.json; the recording's last unfinished chunk, a few seconds, is unreadable). The robot was then still about 17 m from its start; by its own recording (wheel.tum, made on the robot and unaffected) it stopped moving at 21:18:42, back at the start. So **from 21:15:59 to parking the blend ran on the camera alone** (no wheels, no robot gyroscope), and the map's last graph node is at 21:18:45. The user then power-cycled the Jetson after 21:21:06 (the autostop's last line).* The route was completed; what was lost is the proper shutdown and the robot's data on the Jetson from 21:15:59 to parking.

## 1. How far from home at the end - both kinds (rule 20)

*Plain terms: "tracking alone" = where each source's own step-by-step tracking put the robot, never corrected; "corrected" = where the map puts it after its loop closures (the map recognising a place seen before and pulling itself straight). In a fused drive the map's nodes carry the BLEND's position (drive 4 RESULTS.md section 1 explains why), and the camera's tracking restarts from the blend each time it gives up. The drive ended on its start mark by eye, so a perfect estimate shows about 0; the real end was not taped (and tape-mark parking is only good to about 0.2-0.5 m).*

**Drive 9's database was never closed, so it holds no saved corrected positions (Admin.opt_poses is empty, `closed_check.txt`). The corrected camera positions below were RE-OPTIMISED from the database's own links by `tools/db/optimize_graph_se2.py`** (Gauss-Newton on the floor-plane pose graph, link types 0-4, first node held fixed; it reproduced RTAB-Map's own saved result to 0.35 mm on series-1 run 8, the check in its header). They are therefore the same kind of number as drives 4 and 8's corrected gaps, but computed by our script, not by RTAB-Map at shutdown.

| kind | source | end-to-start gap | heading gap | path length | file |
|---|---|---|---|---|---|
| tracking alone | map nodes (Node.pose = the blend's position at each node), node 1 to node 1334 | **0.827 m** | - | 251.88 m | facts.json, camera.tum |
| tracking alone | map nodes, same first and last graph nodes as the corrected row (node 1 to 1201) | 0.799 m | - | - | camera_corrected.tum.meta.json |
| tracking alone | camera tracking, every message (/rtabmap/odom, lost stretches left out; re-seeded by the blend at each restart) | 0.533 m | +6.19 deg | 280.5 m | numbers.json (fusion.bag) |
| tracking alone | blend (/fused/odometry; from 21:15:59 to the end on the camera alone - the robot's data had stopped) | 0.817 m | -0.19 deg | 289.1 m | numbers.json |
| tracking alone | **robot's own wheels + gyroscope, the robot's own recording** (/odometry/filtered in wheel.tum), 20:52:25 to parked 21:24:09 | **0.694 m** | -3.60 deg | 256.9 m | robot_wheels_from_robot_bag.json |
| NOT end-to-start | robot's own wheels + gyroscope as the Jetson received it (/robot/ekf_odom), ends 21:15:59, mid-route | 17.251 m | -2.33 deg | 231.1 m | numbers.json (control: the robot's own recording over the same window gives 17.229 m) |
| **corrected (re-optimised)** | camera map's graph, node 1 to node 1201 (893 graph nodes), floor plane | **0.0136 m** | - | - | camera_corrected.tum.meta.json |
| corrected (re-optimised) | every camera node placed by its graph node's correction, node 1 to node 1334, floor plane | 0.039 m | - | 246.62 m | map_corrected_facts.json |
| corrected | **LiDAR estimate (Force3DoF replay)**, floor plane | **0.011 m** | - | 255.0 m | lidar_map_facts.json |
| corrected | LiDAR estimate (Force3DoF replay), 3D | 0.0106 m | - | - | lidar.tum.meta.json |

**Loop closures beside the corrected gaps: camera map 305** (18 recognised-again + 287 nearby re-matches; database Link table types 1 and 2, counted once per pair; camera_corrected.tum.meta.json gives 305); the live monitor's last line said 239 accepted (`~/.run_records/s2_static_09/monitor_STATUS.txt`). Rejected in `mapping.log`: 1. **LiDAR estimate 379** (lidar.tum.meta.json).

- **Closures joining a later node straight to node 1 (the start): 0**. So nothing ties the end straight to the start: the corrected gap is not a repeat of an end-to-start closure.
- Where the closures fell (newer node's time, the drive cut into four equal quarters): 29 / 53 / 117 / 106. First 20:56:54, last 21:18:45.
- 893 of 1334 nodes are in the graph (drive 4: 284 of 540). The other 441 are still in the database's Node table with weight -9 (RTAB-Map keeps them but leaves them out of the map graph); 133 of them come after the last graph node 1201 (21:18:45). The last node of all was made at 21:21:03. Every-node floor gap (table above): 0.039 m.

## 2. Every turn: robot vs camera vs blend

The live turn watcher's rule replayed after the drive over fusion.bag by `turns_replay.py` (unchanged; its control on drive 3 is in drive 4 RESULTS.md section 2). Output: turns_replayed.log -> turns.csv. *95th percentile = the value 95 % of turns stay under. The robot's own wheels + gyroscope is the comparison for short turns (not ground truth).*

| summary (34 turns) | median | 95th percentile | largest | within 1 deg |
|---|---|---|---|---|
| abs(blend - robot) | 0.30 deg | 0.83 deg | 1.3 deg | 33 of 34 |
| abs(camera - robot) | 2.60 deg | 27.82 deg | 44.9 deg | 0 of 34 |

Turns with no camera value (camera lost for the whole turn): 1. Figure: `turns_difference.png`. The turns stop at 21:15:59: the turn rule needs the robot's stream, which stopped reaching the Jetson then (section 0), so the turns from then to parking (21:18:42) are not in this table.

| turn | ended | direction | s | robot deg | camera deg | blend deg | camera lost | blend - robot | camera - robot |
|---|---|---|---|---|---|---|---|---|---|
| 1 | 20:54:43 | right | 6.8 | -89.8 | -92.2 | -89.8 | 12 % | 0.0 | -2.4 |
| 2 | 20:55:58 | right | 0.3 | -0.1 | -2.4 | -0.1 | 0 % | -0.0 | -2.3 |
| 3 | 20:56:13 | left | 5.0 | 81.8 | 92.8 | 81.8 | 18 % | 0.0 | 11.0 |
| 4 | 20:56:23 | left | 4.6 | 89.2 | 91.9 | 89.2 | 0 % | 0.0 | 2.7 |
| 5 | 20:56:33 | left | 4.7 | 86.5 | 91.7 | 86.1 | 0 % | -0.4 | 5.2 |
| 6 | 20:56:44 | left | 5.1 | 74.1 | 41.9 | 74.2 | 55 % | 0.1 | -32.2 |
| 7 | 20:57:54 | left | 7.3 | 86.4 | 88.0 | 86.5 | 9 % | 0.0 | 1.6 |
| 8 | 20:58:22 | right | 0.3 | -0.6 | -2.4 | -0.9 | 0 % | -0.3 | -1.8 |
| 9 | 20:58:57 | left | 5.7 | 89.9 | 45.0 | 90.4 | 52 % | 0.5 | -44.9 |
| 10 | 20:59:12 | left | 3.6 | 47.3 | 64.9 | 47.6 | 10 % | 0.3 | 17.6 |
| 11 | 20:59:17 | left | 3.3 | 25.3 | 26.7 | 25.5 | 0 % | 0.2 | 1.4 |
| 12 | 21:00:25 | left | 6.1 | 81.7 | 83.5 | 81.3 | 0 % | -0.4 | 1.8 |
| 13 | 21:01:53 | left | 5.2 | 76.2 | 83.7 | 76.6 | 1 % | 0.4 | 7.5 |
| 14 | 21:02:48 | left | 3.2 | 83.2 | 58.3 | 83.7 | 48 % | 0.5 | -24.9 |
| 15 | 21:02:58 | left | 6.2 | 84.6 | 104.9 | 84.4 | 25 % | -0.2 | 20.3 |
| 16 | 21:04:29 | right | 2.9 | -20.5 | -25.4 | -21.2 | 0 % | -0.7 | -4.9 |
| 17 | 21:04:48 | right | 17.0 | -146.0 | -143.2 | -146.0 | 10 % | 0.0 | 2.8 |
| 18 | 21:05:29 | left | 7.0 | 76.0 | 77.9 | 76.5 | 0 % | 0.5 | 1.9 |
| 19 | 21:05:42 | left | 0.7 | 0.8 | 2.8 | 0.6 | 0 % | -0.2 | 2.0 |
| 20 | 21:06:30 | right | 3.1 | -54.0 | -56.4 | -53.7 | 0 % | 0.4 | -2.4 |
| 21 | 21:06:35 | right | 2.7 | -25.3 | -27.6 | -25.2 | 0 % | 0.1 | -2.3 |
| 22 | 21:08:32 | left | 0.3 | 1.8 | 4.9 | 1.9 | 0 % | 0.1 | 3.1 |
| 23 | 21:08:55 | right | 7.1 | -90.4 | -91.5 | -90.9 | 10 % | -0.5 | -1.1 |
| 24 | 21:09:48 | left | 4.8 | 72.3 | 76.4 | 72.8 | 12 % | 0.5 | 4.1 |
| 25 | 21:10:45 | left | 10.4 | 173.2 | 175.7 | 174.0 | 29 % | 0.8 | 2.5 |
| 26 | 21:12:14 | right | 8.8 | -76.2 | -77.6 | -76.4 | 0 % | -0.3 | -1.4 |
| 27 | 21:12:28 | right | 8.6 | -83.1 | None | -82.7 | 100 % | 0.4 | None |
| 28 | 21:13:11 | left | 2.9 | 72.6 | 77.1 | 73.9 | 0 % | 1.3 | 4.5 |
| 29 | 21:13:26 | left | 4.8 | 1.5 | 2.7 | 1.6 | 0 % | 0.1 | 1.2 |
| 30 | 21:14:03 | right | 3.4 | -71.8 | -74.0 | -70.9 | 0 % | 0.9 | -2.2 |
| 31 | 21:14:06 | right | 0.2 | -0.3 | -2.0 | -0.4 | 0 % | -0.1 | -1.7 |
| 32 | 21:14:14 | right | 5.0 | -83.7 | -86.3 | -84.4 | 12 % | -0.7 | -2.6 |
| 33 | 21:15:08 | right | 4.8 | -80.0 | -84.7 | -80.2 | 37 % | -0.1 | -4.7 |
| 34 | 21:15:54 | left | 10.7 | 173.2 | 168.1 | 173.5 | 7 % | 0.2 | -5.1 |

## 3. The WiFi link to the robot (the bridge)

*Source: `~/.run_records/s2_static_09/bridge_recv.log` (Jetson), its final counters; numbers.json (`down_lines_hamilton` = the log's UTC clock minus 4 h).*

- **Link down: 3 time(s)**, at 21:02:31, 21:06:19, 21:16:00 Hamilton. Connections: 0 in 0 receiver session(s).
- Robot-data silences over 1.5 s that did NOT have the freeze signature (freezes_pass_check.json, `other_wheel_silences_over_1p5s`): 2, longest 5.3 s.
- **No per-stream counters**: the bridge receiver writes its final counters when it stops, and it never stopped cleanly (power cycle). The link-down times above are from its running log.
- Parked-drop guard (`fused_odometry.log`): 0 line(s).

| stream | robot read | robot sent | published on the Jetson | % of sent | missed while down | too late (stale) | lost |
|---|---|---|---|---|---|---|---|

## 4. Camera tracking: losses, restarts, freezes, crashes

- **Camera program: 0 crashes, 1 hang, recovered** (section 0): camera guard ended with restarts 1, state "ok".
- **Lost** (fusion.bag, `/rtabmap/odom`: all-zero pose or covariance 9999 - covariance = the tracker's own stated uncertainty; 9999 is its "I am lost" value): **46 stretches, 60.8 s in total, longest 10.0 s, 3.1 % of messages** (numbers.json). This counts only time the tracker SAID it was lost: the hang's 43 s of silence, when no messages came at all, is NOT in it.
- **Restarts from TF** (`mapping.log`, "Odometry automatically reset to latest odometry pose available from TF" - the camera's tracking gave up and restarted from the blend's position): **101**, in 14 burst(s): 20:54:39 (1); 20:56:02-20:56:08 (38); 20:56:42-20:56:49 (24); 20:57:17 (1); 20:57:50 (1); 20:58:55-20:59:07 (9); 21:02:47-21:02:53 (12); 21:04:39-21:04:41 (2); 21:09:40-21:09:43 (2); 21:10:38-21:10:41 (4); 21:12:57 (1); 21:14:11 (1); 21:14:54-21:15:04 (4); 21:15:50 (1). The mapping monitor's last line says resets=101 (`monitor_STATUS.txt`).
- **Freezes** (pass_check.py from `03_methods/jetson_stalls_drive3_2026-09-25/`, on this drive's recording -> freezes_pass_check.json): camera tracker silences > 1.5 s: 7 in all (5.27 s, 1.97 s, 2.83 s, 2.17 s, 1.53 s, 2.77 s, 42.97 s); **outside the hang: 6** (the hang's silence is the longest, around the guard's 28.3 s outage); 'ZED Diagnostic' pauses > 2 s: 9 (6.21 s, 2.56 s, 2.91 s, 2.16 s, 2.36 s, 3.37 s, 34.26 s, 6.22 s, 3.13 s); database saves >= 5.5 s: 0; control, camera gyroscope longest gap 25.272 s; map updates parsed 1291 of 1334 nodes. Verdict as printed: **VOID (control failed: the recorder or the whole Jetson stalled)** (pass line = zero silences; the hang is one of the silences).
  The check printed VOID because its control (the camera's own gyroscope, which must never pause more than 0.1 s) paused 25.3 s; [INFERENCE] that pause is the camera hang itself (the gyroscope comes from the camera program), as on drives 6 and 8.
- **Where the tracker silences come from** (`freeze_timing_drive7.py` -> freeze_timing.json, same gap rule): 7 silences; 7 overlap a 'ZED Diagnostic' pause of the camera program's picture loop; 0 overlap a database save of 1.5 s or more (84 such saves, longest 4.76 s). The hang is one of the 7.
- For comparison: drive 4 (GEN_2, camera freshly started) 1 freeze(s) in 619 s; drive 8 (GEN_2, Jetson up for hours) 15 outside its outage in 850 s (each pack's freezes_pass_check.json). Drive 9 ran on a fresh Jetson boot, so a low count was expected (Jetson-uptime finding, 26 Sept).
- Database integrity (`db_check.txt`, quick_check, read-only): `PASS ~/slam_series2/s2_static_09.db: quick_check=['ok'], 1334 map nodes, 1.1 s`

## 5. Blend heading while standing still (reported, not a pass line)

*Covers only up to 21:15:59: stillness is judged from the robot's wheel stream, which stopped reaching the Jetson then (section 0).* 
stops 78 (251 s still); signed sum **+0.089 deg**; sum of each stop's net change 1.418 deg; largest single stop -0.181 deg (numbers.json). Rule: robot wheel odometry speed and turn rate both zero, below 1e-6 - the log holds 1e-16 float noise, not 0.0 (nearest wheel message within 0.3 s).

## 6. Processor, power and temperature

*Source: `~/.run_records/s2_static_09/tegrastats.log` (tegrastats, once a second; its clock is UTC, converted), window 20:52:25 to 21:21:00 Hamilton, read by tegra_peaks.py -> power_temperature.json. Drive 4's figures: its pack's power_temperature.json (a separate tegrastats log).*

| quantity | drive 9 peak | when | drive 9 median | drive 4 peak / median |
|---|---|---|---|---|
| total power (four named supply rails) | **35.7 W** | 21:13:01 | 29.9 W | 36.6 / 29.8 W |
| hottest point on the chip (tj) | **62.1 C** | 21:08:47 | - | 61.6 C / - |
| processor temperature | 62.2 C | 21:13:01 | - | 61.6 C / - |
| graphics processor temperature | 55.9 C | 21:10:40 | - | 55.7 C / - |
| graphics processor load | 99 % | 21:02:01 | 22 % | 99 / 23 % |
| processor load, mean of the 12 cores | 98.5 % | 21:12:55 | 40.6 % | 99.9 / 41.2 % |
| memory free, lowest | 54.1 % (13.7 GB used) | 21:20:57 | - | 66.2 % / - |

- tegrastats skips over 2 s (the log itself missing seconds): 20:58:19 (3 s), 21:05:38 (3 s); samples 1665 over 1715 s.
- *Plain terms: the processor peak near 100 % at the start is the camera and mapping programs starting up; the median is the working load. The memory guard (earlyoom) kills the mapping program if free memory falls under 15 %; the lowest here is shown above.*
- RTAB-Map, time to process each map snapshot (database Statistics table, read-only): **median 599.7 ms, 95th percentile 838.5 ms, largest 5851.5 ms over 1334 nodes** (numbers.json). Drive 4: 510.2 / 787.8 / 1852.7 ms over 540 nodes.

## 7. Agreement with the LiDAR estimate

*The reference is the **LiDAR estimate (Force3DoF replay)**: the robot's LiDAR mapping (a colleague's work: self_navigation `rtabmap_3d.launch`) replayed on the robot after the drive with ONE parameter changed, `Reg/Force3DoF=true` (the map is held flat on the floor, so a wrongly-read gravity direction cannot tip it; `03_methods/lidar_drift_2026-09-26/FINDINGS.md` section 11). No colleague-mode replay exists for drive 9. It is the robot's wheels + gyroscope corrected by LiDAR loop closures. **An independent estimate, not ground truth** (ENGINEERING_NOTES.md rule 19): LiDAR range noise 15-30 mm, the two LiDARs agree to about 5 mm, and the LiDAR-to-camera mounting offset has never been measured. It shares the robot's wheels + gyroscope with the blend, so for the blend it is not fully independent. Alignment: SE(3) (rotation + shift), never scale (ENGINEERING_NOTES.md section 4 rule 4).*

- LiDAR estimate (Force3DoF replay): 1180 positions, 255.0 m, **379 LiDAR loop closures** (drive 4, colleague mode: 75); its own end gap 0.011 m on the floor, 0.011 m in 3D (section 1). Its own check (agreement.json, compare_lidar.py: ends within 0.5 m of its start AND at least one LiDAR closure): "passes its own check: ends 0.01 m from its start, 379 LiDAR loop closures".
- **Flatness check of the LiDAR estimate** (`03_methods/lidar_drift_2026-09-26/score_f3dof.py` -> lidar_flatness_check.json): largest tilt 0.0 deg, height range 0.00 m, floor / 3D end gap 0.01 / 0.01 m -> **PASS** (pass line: tilt under 3 deg and height range under 0.3 m). *Plain terms: a flat floor must give a flat map; drives 6 and 8's colleague-mode maps failed this by tens of degrees. Under Force3DoF flatness is built in, so passing it is expected; the evidence that this yardstick is sound is its closures and its end gap.* Its own quick comparison there (node positions, 2D fit, a different method from the table below): vs camera map 0.42 / 0.65 m, vs robot's wheels + gyroscope 0.14 / 0.39 m (median / 95th percentile).

| path compared | kind (rule 20) | matched | median | 95th percentile | largest | best-fit scale (NOT applied) | file |
|---|---|---|---|---|---|---|---|
| camera map, corrected (camera_corrected.tum) | map's corrected path | 99.9 % (892) | **0.419 m** | **0.648 m** | 0.848 m | 1.0515 | agreement.json |
| map nodes (= blend), tracking alone (camera.tum) | tracking alone | 90.0 % (1200) | **0.278 m** | **0.577 m** | 0.983 m | 1.0274 | agreement.json |

- Clock shift: estimated +0.29 s, applied 0.0 s (compare_lidar.py applies one only if it improves the fit by > 0.05 m and 20 %).
- **For information, drive 8 against its own Force3DoF replay** (same compare_lidar.py method, `s2_static_08/lidar_ref_f3dof/compare_same_method/`; drive 8's colleague-mode LiDAR map failed its own check): the LiDAR estimate passes its own check: ends 0.02 m from its start, 87 LiDAR loop closures; camera map corrected **median 0.260 m, 95th percentile 0.508 m** (473 pairs); size fitted, for information: median 0.078 m at size factor 1.0616.

### 7a. For information, size fitted (NOT the result)

*Why this is here: a taped 15.00 m test (`01_runs/series2_static/s2_scale_01/RESULTS.md`, same day) found the robot's wheels read about 6 % long and the camera within 1 %. The LiDAR estimate takes its size from the wheels, so part of the SE(3) distance above is the LiDAR estimate being too large, not the camera map's shape. Below, the same comparison is repeated allowing one size change (Sim(3)). For information only: the camera's size is real and is not to be fitted away (ENGINEERING_NOTES.md section 4 rule 4).* Made by `size_fit_agreement.py` -> size_fit.json, same reference, same 50 ms pairing, same fitting code as compare_lidar.py; its rigid (SE(3)) run reproduces agreement.json exactly (control: passed).

| path compared | pairs | SE(3) median / 95th pct (the result) | size fitted: median / 95th pct | size factor applied to the camera path |
|---|---|---|---|---|
| camera map, corrected | 892 | 0.419 / 0.648 m | 0.126 / 0.281 m (for information, size fitted) | 1.0515 |
| map nodes (= blend), tracking alone | 1200 | 0.278 / 0.577 m | 0.132 / 0.403 m (for information, size fitted) | 1.0274 |

*Plain terms: a size factor of 1.052 = the corrected camera path would have to be stretched by 5.2 % to fit the LiDAR estimate best. The tape test says the robot's wheels + gyroscope read 1.064 times the true distance and the camera 0.993 times, so the size difference expected from the tape alone is 1.064 / 0.993 = 1.071. [INFERENCE] Drive 9's 1.052 against that: within 1.4 times the one-drive spread of the size factor (0.015, drives 3-5) of that, so [INFERENCE] most of the size difference is the reference's, not the camera's.*

## 8. Distances - from the camera and from the wheels

*Each source's own path length for the whole drive. The wheels read long (tape: robot wheels + gyroscope 6.4 % long, wheels alone 6.2 %, s2_scale_01 legs.json, legs 1, 2, 3, 6); the camera read within 1 %. "2 Hz" = positions every 0.5 s, straight steps summed (paths_2hz.csv).*

| source | what it is | path length | drive 4 | file |
|---|---|---|---|---|
| **camera map, corrected** | camera, after its loop closures (every node) | **246.6 m** | 85.1 m | map_corrected_facts.json |
| camera tracking, 2 Hz | camera's own tracking (re-seeded by the blend at restarts); to 21:20:56, including standing after parking | 257.6 m | - | paths_2hz.csv |
| map nodes, tracking alone | the blend's position at each node | 251.9 m | 87.2 m | facts.json |
| blend, 2 Hz | camera + wheels + gyroscopes | 254.8 m | - | paths_2hz.csv |
| **robot's own wheels + gyroscope, 2 Hz, the robot's own recording** | wheels (what the LiDAR map is built on), start to parked | **255.1 m** | - | robot_wheels_from_robot_bag.json |
| robot's own wheels + gyroscope, 2 Hz, as the Jetson received it | ends 21:15:59 (section 0), NOT the whole drive | 229.5 m | - | paths_2hz.csv |
| LiDAR estimate (Force3DoF replay) | wheels + gyroscope corrected by LiDAR closures | 255.0 m | 88.8 m | lidar_map_facts.json |
| robot's wheels + gyroscope x 0.940 (for information: the tape's correction) | wheels (robot's own recording), resized by the taped test | 239.8 m | - | computed here |

**Distance to quote for drive 9: about 247 m by the camera map** (the wheels say 255 m, which the tape shows reads about 6 % long).

## 9. Drives 4 / 8 / 9 - did the longer drive keep drive 4's quality?

*Drive 9 against drive 4 on the part of the route both covered, judged against stated uncertainty (ENGINEERING_NOTES.md section 4 rule 0); drive 8 beside them for information, scored against its own Force3DoF replay. Made by compare_drives_4_8_9.py.*

**In plain words: partly. It came home as well as drive 4, but its typical (median) distance from the LiDAR estimate is larger on every agreement row: 1.4-1.7 times the combined spread - more than the spread, less than the 3 times that would make it a clear difference. Its 95th percentiles sit inside the band, but that band is wide (0.38 m, from only 3 drives), so that test says little.** On the part of the route both drives covered (98 m of drive 9's 245 m lies within 0.5 m of drive 4's route, covering 93 % of drive 4's), drive 9's corrected camera map sits a median 0.43 m (95th percentile 0.61 m) from its LiDAR estimate, against drive 4's 0.24 / 0.46 m on the same part - median 1.7 times, 95th percentile 0.4 times the combined spread. Over its whole 247 m route drive 9 reads 0.42 / 0.65 m (drive 4, whole 85 m: 0.26 / 0.49 m) - median differs by 1.39 times, 95th percentile agrees within 0.38 m. Drive 9's camera map came home to 0.01 m with 305 loop closures (0 joining the start) against drive 4's 0.05 m with 181 (3); its LiDAR estimate to 0.01 m with 379 closures. For information, drive 8 against its own no-tipping LiDAR replay: 0.26 / 0.51 m. With the alignment refitted on the shared part alone, drive 9 reads 0.21 / 0.45 m against drive 4's 0.22 / 0.48 m - for information only: a subset fit always lowers the distance, and more so for drive 9 (43 % of its pairs refitted against drive 4's 86 %); [INFERENCE] consistent with the single alignment being spread over a 3x longer route, but a genuinely bent longer map is equally possible (off the shared route drive 9 reads 0.38 m, lower than on it).

**Verdict, drive 9 against drive 4 (computed): of 17 judged rows about the camera, the blend, the agreement with the LiDAR estimate and the mapping program, 9 agree within the combined uncertainty, 7 differ by 1-3 times it, and 1 differ by more than 3 times it: camera map loop closures per 10 m of its route (5.1 times).** Counted separately: the robot's own wheels + gyroscope (0 / 2 / 0 agree / 1-3 times / over 3 times) and the LiDAR estimate itself (1 / 0 / 1).

*How to read this: two drives never give identical numbers. For each row the spread one drive is expected to have is stated, with where it comes from (column "from"); the two are combined as sqrt(s4^2 + s9^2). "Agrees within X" = the difference is smaller than that; "differs by N times" = N times larger (3 or more is a real difference). N = 1 drive each; the distance spreads are working figures. "Shared route" rows use only the part both drives covered; "whole drive" rows are per 10 min or per 10 m where the number grows with length. Drive 8's column is for information (its yardstick: its own Force3DoF LiDAR replay).*

**Shared route - how it was defined.** camera map corrected path (camera_corrected.tum; drive 9: graph re-optimised by optimize_graph_se2.py), filled in every 5 cm; drives 8 and 9 lined up on drive 4 by 2D rotation + shift (ICP, pairs < 1.0 m); a point is on the shared route if within 0.5 m of the other drive's route; direction of travel not tested. Drive 9 lined up on drive 4 by -7.1 deg and 0.75 m; its start then sits 0.75 m from drive 4's. **Drive 9: 98.2 of its 244.8 m within 0.5 m of drive 4's route; drive 4: 79.1 of 84.7 m (93 %) within 0.5 m of drive 9's.** On the shared part drive 9's route runs a median 0.08 m (95th percentile 0.34 m) from drive 4's. (Both drives started on the same mark by eye; the 0.75 m start offset after lining up comes with the rotation, which is fitted to the whole shared route, not to the start.) Sensitivity (drive 9's shared length): 0.3 m: 91.0 m, 0.5 m: 98.2 m, 0.75 m: 102.4 m, 1.0 m: 104.6 m. Figure: `shared_route.png`.

Drive 9 off the shared route (the new corridors): camera map corrected vs LiDAR estimate median 0.38 m, 95th percentile 0.70 m.

| row | group | scope | drive 4 | drive 8 (info) | drive 9 | spread per drive | from | drive 9 vs drive 4 | drive 8 vs drive 4 |
|---|---|---|---|---|---|---|---|---|---|
| route: shared length (drive's own / drive 4's, within 0.5 m) | context | route | - | 75.2 of 83.0 m / 83.2 of 84.7 m | 98.2 of 244.8 m / 79.1 of 84.7 m | - | - | not judged: defines the comparison (shared_route.json) | - |
| distance driven - camera map corrected / robot's own wheels + gyroscope / LiDAR estimate | context | whole drive | 85.1 m / 90.1 m / 88.8 m | 83.5 m / 88.6 m / 87.3 m | 246.6 m / 256.9 m / 255.0 m | - | - | not judged: a description of the route; the wheels read about 6 % long (tape test) | - |
| drive duration | context | whole drive | 619 s | 850 s | 1734 s | - | - | not judged: description | - |
| LiDAR yardstick used | context | whole drive | colleague-mode LiDAR replay | Force3DoF LiDAR replay (for information) | Force3DoF LiDAR replay | - | - | not judged: which LiDAR estimate each drive is compared with | - |
| camera tracker setting / start | context | whole drive | GEN_2, camera freshly started | GEN_2 | GEN_2, fresh Jetson boot | - | - | not judged: setting | - |
| start-to-end gap, camera tracking alone (0 closures) | camera and blend | whole drive | 0.25 m | 1.53 m | 0.53 m | 0.20 m | A | **differs** by 1.00 times (difference 0.28 m, combined 0.28 m) | differs by 4.52 times (difference 1.28 m, combined 0.28 m) |
| start-to-end gap, map nodes tracking alone (= blend) | camera and blend | whole drive | 0.06 m | 0.60 m | 0.83 m | 0.20 m | A | **differs** by 2.72 times (difference 0.77 m, combined 0.28 m) | differs by 1.92 times (difference 0.54 m, combined 0.28 m) |
| start-to-end gap, robot's own wheels + gyroscope, tracking alone | robot's wheels + gyroscope | whole drive | 0.27 m | 0.98 m | 0.69 m | 0.20 m | A | **differs** by 1.51 times (difference 0.43 m, combined 0.28 m) | differs by 2.52 times (difference 0.71 m, combined 0.28 m) |
| heading gap, robot's own wheels + gyroscope | robot's wheels + gyroscope | whole drive | +2.3 deg | +8.4 deg | -3.6 deg | 3.0 deg | B | **differs** by 1.39 times (difference 5.9 deg, combined 4.2 deg) | differs by 1.44 times (difference 6.1 deg, combined 4.2 deg) |
| start-to-end gap, **camera map corrected** (floor) | camera and blend | whole drive | 0.05 m | 0.10 m | 0.01 m | 0.20 m | A | **agrees** within 0.28 m (difference 0.03 m) - closures beside it (rule 20): drive 4 181 (3 join the start), drive 8 145 (0), drive 9 305 (0); drive 9's corrected path is the graph RE-OPTIMISED by optimize_graph_se2.py (database not closed) | agrees within 0.28 m (difference 0.05 m) |
| camera map loop closures per 10 m of its route | camera and blend | whole drive | 21.3 | 17.4 | 12.4 | 0.7 | C | **differs** by 5.14 times (difference 8.9, combined 1.7) - counts: 181 / 145 / 305; drive 9 includes 147 m of new corridors driven once, where a map cannot yet recognise anything, so fewer per metre is expected [INFERENCE] | differs by 1.82 times (difference 3.9, combined 2.1) |
| start-to-end gap, **LiDAR estimate corrected** (floor) | LiDAR map | whole drive | 0.03 m | 0.02 m | 0.01 m | 0.20 m | A | **agrees** within 0.28 m (difference 0.02 m) - LiDAR closures 75 / 87 / 379 | agrees within 0.28 m (difference 0.01 m) |
| LiDAR estimate loop closures per 10 m | LiDAR map | whole drive | 8.4 | 10.0 | 14.9 | 0.8 | C | **differs** by 5.18 times (difference 6.4, combined 1.2) - drive 4's LiDAR estimate is the colleague-mode replay, drive 9's the Force3DoF replay: not the same replay mode (drive 8 in Force3DoF: 87 closures) | differs by 1.05 times (difference 1.5, combined 1.4) |
| LiDAR yardstick passes its own check | LiDAR map | whole drive | yes | yes | yes | - | - | not judged: a pass/fail check | - |
| camera freezes (tracker silent > 1.5 s), per 10 min | camera and blend | whole drive | 1.0 | 10.6 | 2.1 | 0.8 | D | **agrees** within 1.3 (difference 1.1) - counts: 1 / 15 / 6; drive 8's crash outage and drive 9's hang outage are not counted as freezes | differs by 3.32 times (difference 9.6, combined 2.9) |
| camera tracking restarts from the blend, per 10 min | camera and blend | whole drive | 25.2 | 7.1 | 34.9 | 3.5 | D | **differs** by 1.61 times (difference 9.8, combined 6.0) - counts: 26 / 10 / 101 | differs by 3.34 times (difference 18.1, combined 5.4) |
| camera tracking lost, seconds per 10 min | camera and blend | whole drive | 25.4 s | 10.1 s | 21.0 s | 6.9 s | E | **agrees** within 9.8 s (difference 4.3 s) - counts only time the tracker SAID it was lost; silences with no messages (drive 9's 43 s hang, drive 8's outage) are not in it | differs by 1.57 times (difference 15.3 s, combined 9.8 s) |
| camera program outages | context | whole drive | none | 1, recovered: 15:52:57, back after 12.7 s (camera guard); mapping carried on | 0 crashes; 1 camera HANG, recovered: 21:12:29, back after 28.3 s (camera guard); mapping carried on | - | - | not judged: one event is not a rate | - |
| turns: abs(camera - robot), median of all turns | camera and blend | whole drive | 2.3 deg | 3.2 deg | 2.6 deg | 1.0 deg | F | **agrees** within 1.5 deg (difference 0.3 deg) - turns: 16 / 26 / 34 | agrees within 1.5 deg (difference 0.9 deg) |
| turns: abs(blend - robot), median of all turns | camera and blend | whole drive | 0.2 deg | 0.2 deg | 0.3 deg | 0.2 deg | F | **agrees** within 0.2 deg (difference 0.1 deg) | agrees within 0.2 deg (difference 0.0 deg) |
| **agreement with the LiDAR estimate, ON THE SHARED ROUTE**, camera map corrected: median (SE(3)) | agreement with the LiDAR estimate | shared route | 0.24 m | 0.25 m | 0.43 m | 0.08 m | G | **differs** by 1.69 times (difference 0.20 m, combined 0.12 m) - drive 4's figure is over its part shared with drive 9 (with drive 8: 0.26 m); pairs 244 / 425 / 383 | agrees within 0.12 m (difference 0.01 m) |
| **agreement with the LiDAR estimate, ON THE SHARED ROUTE**, camera map corrected: 95th percentile (SE(3)) | agreement with the LiDAR estimate | shared route | 0.46 m | 0.47 m | 0.61 m | 0.27 m | H | **agrees** within 0.38 m (difference 0.16 m) | agrees within 0.38 m (difference 0.01 m) |
| **agreement with the LiDAR estimate, ON THE SHARED ROUTE**, map nodes (= blend) tracking alone: median (SE(3)) | agreement with the LiDAR estimate | shared route | 0.09 m | 0.14 m | 0.28 m | 0.08 m | G | **differs** by 1.64 times (difference 0.19 m, combined 0.12 m) - drive 4's figure is over its part shared with drive 9 (with drive 8: 0.11 m); pairs 419 / 602 / 566 | agrees within 0.12 m (difference 0.05 m) |
| **agreement with the LiDAR estimate, ON THE SHARED ROUTE**, map nodes (= blend) tracking alone: 95th percentile (SE(3)) | agreement with the LiDAR estimate | shared route | 0.23 m | 0.39 m | 0.36 m | 0.27 m | H | **agrees** within 0.38 m (difference 0.13 m) | agrees within 0.38 m (difference 0.17 m) |
| for information, agreement with the LiDAR estimate ON THE SHARED ROUTE, camera map corrected, alignment REFITTED on the shared part only: median | for information | shared route | 0.22 m | 0.24 m | 0.21 m | 0.08 m | I | **agrees** within 0.12 m (difference 0.01 m) - NOT a fair comparison: a fit to a subset always lowers the leftover distance, and drive 9 refits on 383 of its 892 pairs (43 %) against drive 4's 244 of 283 (86 %), so drive 9 gains more. [INFERENCE] consistent with the whole-route alignment being spread over a 3x longer route, but equally with the longer map being genuinely bent | agrees within 0.12 m (difference 0.02 m) |
| for information, same, 95th percentile | for information | shared route | 0.48 m | 0.47 m | 0.45 m | 0.27 m | J | **agrees** within 0.38 m (difference 0.03 m) | agrees within 0.38 m (difference 0.01 m) |
| agreement with the LiDAR estimate, camera map corrected, whole drive: median (SE(3)) | agreement with the LiDAR estimate | whole drive | 0.26 m | 0.26 m | 0.42 m | 0.08 m | G | **differs** by 1.39 times (difference 0.16 m, combined 0.12 m) - drive 9's whole drive is 247 m against drive 4's 85 m | agrees within 0.12 m (difference 0.00 m) |
| agreement with the LiDAR estimate, camera map corrected, whole drive: 95th percentile (SE(3)) | agreement with the LiDAR estimate | whole drive | 0.49 m | 0.51 m | 0.65 m | 0.27 m | G | **agrees** within 0.38 m (difference 0.16 m) - drive 9's whole drive is 247 m against drive 4's 85 m | agrees within 0.38 m (difference 0.02 m) |
| for information, size fitted (Sim(3)): camera map corrected, whole drive, median | for information | whole drive | 0.09 m | 0.08 m | 0.13 m | 0.08 m | I | **agrees** within 0.12 m (difference 0.03 m) | agrees within 0.12 m (difference 0.02 m) |
| best-fit size factor, camera map corrected vs LiDAR estimate (not applied) | agreement with the LiDAR estimate | whole drive | 1.063 | 1.062 | 1.052 | 0.015 | K | **agrees** within 0.021 (difference 0.012) | agrees within 0.021 (difference 0.001) |
| RTAB-Map time per map snapshot, median | processor | whole drive | 510 ms | 515 ms | 600 ms | 49 ms | L | **differs** by 1.29 times (difference 90 ms, combined 70 ms) | agrees within 70 ms (difference 5 ms) |
| total power, median | processor | whole drive | 29.8 W | 29.6 W | 29.9 W | - | - | not judged: the drive-to-drive spread of drives 4, 5 (camera-alive part) and 6 (0.3 W) comes from 3 drives and is too small to trust as a yardstick | - |
| hottest point on the chip (tj), peak | processor | whole drive | 61.6 C | 61.5 C | 62.1 C | - | - | not judged: the drive-to-drive spread of drives 4, 5 (camera-alive part) and 6 (0.1 C) comes from 3 drives and is too small to trust as a yardstick | - |
| processor load, mean of 12 cores, median | processor | whole drive | 41.2 % | 39.3 % | 40.6 % | - | - | not judged: the drive-to-drive spread of drives 4, 5 (camera-alive part) and 6 (0.2 %) comes from 3 drives and is too small to trust as a yardstick | - |

Spread sources (column "from"):
- **A** = where the robot parked: s2_scale_01 found stops at one mark up to 0.19 m apart even with a wheel stop (its RESULTS.md); drives 4, 8 and 9 ended on the start mark by eye (tape-mark parking is only +-0.2-0.5 m)
- **B** = parking heading by eye, working figure (not measured)
- **C** = counting spread, the square root of each count (Poisson, the least spread a count of chance events has), scaled
- **D** = counting spread, the square root of each count (Poisson, the least spread a count of chance events has) (at least 1), scaled
- **E** = drive-to-drive spread (sample standard deviation) of lost seconds per 10 min, drives 3, 4, 5, 6
- **F** = drive-to-drive spread (sample standard deviation) of the per-turn median, drives 4, 5, 6
- **G** = drive-to-drive spread (sample standard deviation) of the camera map's whole-drive agreement on drives 3, 4, 5, whose LiDAR estimate passed its own check; different routes, so it may overstate the spread for a repeat
- **H** = drive-to-drive spread (sample standard deviation) of the camera map's whole-drive agreement (95th percentile) on drives 3, 4, 5, whose LiDAR estimate passed its own check; different routes, so it may overstate the spread for a repeat
- **I** = drive-to-drive spread (sample standard deviation) of the camera map's whole-drive agreement on drives 3, 4, 5, whose LiDAR estimate passed its own check; different routes, so it may overstate the spread for a repeat (borrowed)
- **J** = drive-to-drive spread (sample standard deviation) of the camera map's whole-drive agreement (95th percentile) on drives 3, 4, 5, whose LiDAR estimate passed its own check; different routes, so it may overstate the spread for a repeat (borrowed)
- **K** = drive-to-drive spread (sample standard deviation) of drives 3, 4, 5 (valid LiDAR estimates)
- **L** = drive-to-drive spread (sample standard deviation) of drives 4, 5, 6 (numbers.json processor_rtabmap)

Caveats: (1) the 0.20 m for tracking-alone gaps is only the parking floor; tracking drift varies far more from drive to drive, and grows with route length, so drive 9's (3x longer) "differs" on those rows is expected and weak. (2) The agreement spreads come from whole drives on different routes; the shared-route rows borrow them. (3) The shared-route agreement uses the whole-drive SE(3) alignment. (4) Drive 4's yardstick is the colleague-mode LiDAR replay, drive 9's the Force3DoF replay: one parameter apart (FINDINGS.md section 11); drive 4's IMU never tipped, so the two modes are expected to agree there [INFERENCE - drive 4 was not replayed in Force3DoF]. (5) Drive 9's corrected camera path is the re-optimised graph (validated to 0.35 mm where a saved result exists), not RTAB-Map's own saved result, because the database was never closed. (6) Counts use the Poisson floor, the least spread a count can have.

## 10. Figures, video and 3D map in this folder

| file | what it shows |
|---|---|
| map.png | camera floor map, map nodes at their tracking-alone positions (Node.pose = the blend's) |
| map_corrected.png | camera floor map after the loop closures (graph RE-OPTIMISED by optimize_graph_se2.py - the database was not closed) |
| trajectory.png | where each source put the robot, two panels (rule 20): tracking alone (robot's wheels + gyroscope solid - the Jetson's copy, which ends 21:15:59 mid-route; its whole-drive figures are in section 1 from the robot's own recording; camera and blend dashed) and corrected (camera map, re-optimised graph, dashed; LiDAR estimate, Force3DoF replay, solid); the camera hang marked |
| turns_difference.png | per turn: camera minus robot and blend minus robot, degrees |
| lidar_comparison.png | camera paths over the LiDAR map (LiDAR solid, camera dashed) and the distance to the LiDAR estimate along the route (SE(3)) |
| lidar_map.png | the LiDAR estimate's own floor map (Force3DoF replay) |
| shared_route.png | drive 9's and drive 8's routes lined up on drive 4's (camera maps corrected, dashed); the shared part drawn heavy, the rest black |
| `04_figures/camera_vs_lidar_2026-09-26/drive9_camera_vs_lidar.png` | camera map vs LiDAR estimate map, same scale (chart figure) |
| timelapse.mp4 | **frames recorded live during the drive** (the live map page every 3 s: camera map left, LiDAR view right), 571 frames (`timelapse_live_index.csv`). The recorder builds its video at "park", which never came, so the video was built afterwards from those saved frames by `timelapse_pipe_drive9.py` (the recorder's own layout and captions) |
| timelapse_25/50/75/100.png | stills at 25/50/75/100 % of the timelapse (video frames 142, 285, 428, 570, counted from 0; extracted with ffmpeg) |
| (no 3D map) | 3D export SKIPPED: only  4417 MB free on the Jetson at 22:33 EDT (rule: > 6 GB needed; rtabmap-export needs a writable copy of the 2.8 GB database) |

Other files: camera.tum / camera_corrected.tum (+ .meta.json); paths_2hz.csv; turns.csv, turns_replayed.log; numbers.json; crash_split.json; freezes_pass_check.json; freeze_timing.json; power_temperature.json; closure_span.json; db_check.txt; closed_check.txt; size_fit.json; shared_route.json; compare_drives_4_8_9.json/.md; lidar.tum, wheel.tum, lidar_map.npz, lidar_reference_dense.tum, lidar_comparison/ (copied from lidar_ref_f3dof/, whose products.log / provenance.txt describe the robot's replay); lidar_flatness_check.json; network_and_camera_events_journal.txt.

## 11. Still open

- N = 1 per drive: drives 4, 8 and 9 are one drive each on overlapping routes, not five repeats (ENGINEERING_NOTES.md section 4 rule 2).
- **The map was not closed properly.** The corrected camera positions are our re-optimisation (section 1); RTAB-Map's own at-shutdown result does not exist for this drive. Park must be reached before the Jetson is touched; the WiFi roaming every 5 minutes (section 0) is the likely thing to fix before the next long drive [INFERENCE].
- **The camera hang** (nvargus-daemon InsufficientMemory, section 0) is new: drives 5, 6 and 8 were crashes. Its cause is open; the lowest free memory during the drive is in section 6.
- **Drive 4's yardstick is the colleague-mode replay, drive 9's the Force3DoF replay** - one parameter apart. Drive 4's IMU never tipped, so the two modes should agree there [INFERENCE]; replaying drive 4 in Force3DoF would settle it.
- The distance spreads used in section 9 are working figures (parking by eye), not a measured repeatability.
- 3D export: 3D export SKIPPED: only  4417 MB free on the Jetson at 22:33 EDT (rule: > 6 GB needed; rtabmap-export needs a writable copy of the 2.8 GB database).
- The camera's path is not independent of the blend in a fused drive (section 1).

## 10. 3D and 2D maps (made 27 Sept ~01:40 Hamilton, `map_3d/`)
`drive9_map_3d.ply` (3.57 M points, 3 cm voxel grid, 110 MB, in the Autonomous Service Robot Teams folder, not in git), `d9_3d.png`
(two 3D views), `d9_topdown.png` (points below 2 m from above, robot path dashed). Same method as drive 6
(`rtabmap-export --cloud --poses --max_range 6 --decimation 4 --voxel 0.03` on a copy of the database). Points sit at the
**map's corrected positions**: the export re-optimises the graph from the database's own links; they agree with
`camera_corrected.tum` (optimize_graph_se2.py) to a median 2.9 mm on the floor plane (max 7.3 mm, 892 nodes; the last
node 1201 is not in the export's graph). Details: `map_3d/README.txt`.
