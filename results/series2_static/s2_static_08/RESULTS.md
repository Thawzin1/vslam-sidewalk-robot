# Drive 8 (s2_static_08) - results pack: drive 4's route, repeated

*26 Sept 2026, Hamilton time: live recorder started 15:48:31, autostop armed 15:48:32, camera program down 15:52:57.2-15:53:09.9 (restarted by the camera guard), "park" 16:01:35, map closed properly 16:02:15. User-driven over drive 4's route as closely as practical: the long corridor out to about 11 m and back, turning round, twice; the side corridor, reversed in and forward out (planned about 6.5 m as on drive 4; measured 11.1 m in from the start, camera map corrected, shared_route.json); the long corridor once more, out forward and back in reverse; park. Fused (the camera blended with the robot's wheels and gyroscope (turn-rate sensor) by an EKF (extended Kalman filter, a standard way of mixing sensors)), camera tracker GEN_2 as on drive 4 (GEN_1 / GEN_2 = the ZED camera's two built-in position-tracking methods, a camera setting). Made on the Jetson on 26 Sept by `results_packs_2026-09-26/` (project records) (pack_drive8_heavy.sh, analyze_run.py, turns_replay.py, extract_timing.py, crash_split_drive8.py, tegra_peaks.py, freeze_timing_drive7.py, compare_lidar.py, size_fit_agreement.py, compare_drives_4_7_8.py, this file's writer `write_results_drive8.py`). Every number names the file it comes from; files are in this folder unless a path is given. Nothing here is ground truth (ENGINEERING_NOTES.md rule 19). N = 1.*

*Words used below: node = one saved map snapshot; TF = ROS's shared table of where each part of the robot is; Node.pose = each node's tracking-alone position; Admin.opt_poses = the corrected positions RTAB-Map saves when it shuts down properly; fusion.bag = the drive's recording of every sensor stream; SE(3) = lining two paths up by rotation and shift only; Sim(3) = the same but also allowing a size change; tegrastats = NVIDIA's once-a-second load/power/temperature log; quick_check = SQLite's own database-integrity check; ffmpeg = a standard video program.*

## 0. What was different from drive 4, and the camera outage (read first)

| item | drive 4 | drive 8 | evidence |
|---|---|---|---|
| camera tracker | GEN_2 | GEN 2 (reverted after drive 7's GEN_1 freezes) | `~/.run_records/s2_static_08/camera.log` "Positional tracking mode -> ..." |
| duration | 619 s | 850 s | facts.json `duration_s` |
| camera program crashes | none | **1, recovered in 12.7 s** (camera guard) | `camera_outages.csv`, `camera_guard.json` restarts 1 |
| route | the reference route | the same route by the user's driving; 75.2 of 83.0 m within 0.5 m of drive 4's route | shared_route.json, section 9 |
| tegrastats | a separate log (`~/rehearsal_t5/`) | **in the run records for the first time** (`tegrastats.log`) | section 6 |

**The camera outage.** `camera_outages.csv` (camera guard): camera program **gone at 15:52:57.2, back at 15:53:09.9: 12.7 s**; cause "camera process gone"; outcome "restarted (1); camera positions back after 0.4 s but LOST (null position)". The camera tracker (`/rtabmap/odom`, fusion.bag) was silent longest 13.13 s near the outage (15:52:56 to 15:53:09; crash_split.json, from the recording's receive times; section 4's 13.05 s is the same silence measured by pass_check.py from extract_timing.py's timing file - a different script and time column, not checked further). **This is the first crash with a libSegFault backtrace** (a crash report printed by the system library that catches the fault): the fault is inside the ZED SDK (`libsl_zed.so`), under `sl::Camera::grab()` called from the ROS wrapper's device-poll thread, at a `pthread_mutex_lock` (device-poll thread = the wrapper's loop that asks the camera for each new picture; pthread_mutex_lock = taking a lock that stops two parts of a program changing the same memory at once) (`03_methods/camera_segfault_2026-09-26/RESULTS.md` section 9; [INFERENCE] there: a lock on memory that is no longer valid; not in our code). N = 1 backtrace.

| | start to outage | during the outage | back to end | source |
|---|---|---|---|---|
| distance, robot's own wheels + gyroscope (/robot/ekf_odom) | 26.7 m | 1.5 m | 59.0 m | crash_split.json |
| distance, blend (/fused/odometry) | 26.5 m | 1.5 m | 59.0 m | crash_split.json |
| map snapshots (nodes) | 261 | 1 | 456 | database Node table |

*Distances: positions every 0.5 s, straight steps summed (crash_split.json); start = the recording's first message. Plain terms: the robot moved 1.5 m (its own wheels + gyroscope) while the camera was down, so that stretch has no camera pictures in the map - the blend carried the position across it on wheels + gyroscope.*

- Closures touching a node made before the outage: **54 of 145**; closures whose newer node came after the camera was back: 145 (crash_split.json). *Plain terms: the map recognised places from before the crash after the camera came back, so the two halves of the map are tied together by closures.*

## 1. How far from home at the end - both kinds (rule 20)

*Plain terms: "tracking alone" = where each source's own step-by-step tracking put the robot, never corrected; "corrected" = where the map puts it after its loop closures (the map recognising a place seen before and pulling itself straight). In a fused drive the map's nodes carry the BLEND's position (drive 4 RESULTS.md section 1 explains why), and the camera's tracking restarts from the blend each time it gives up. The drive ended on its start mark by eye, so a perfect estimate shows about 0; the real end was not taped.*

| kind | source | end-to-start gap | heading gap | path length | file |
|---|---|---|---|---|---|
| tracking alone | map nodes (Node.pose = the blend's position at each node), node 1 to node 718 | **0.601 m** | - | 86.51 m | facts.json, camera.tum |
| tracking alone | camera tracking, every message (/rtabmap/odom, lost stretches left out; re-seeded by the blend at each restart) | 1.529 m | -10.59 deg | 102.9 m | numbers.json (fusion.bag) |
| tracking alone | blend (/fused/odometry) | 0.604 m | -6.47 deg | 92.0 m | numbers.json |
| tracking alone | robot's own wheels + gyroscope (/robot/ekf_odom) | 0.979 m | +8.44 deg | 88.6 m | numbers.json |
| **corrected** | camera map's saved graph, Admin.opt_poses, node 1 to node 663 (475 graph nodes), 3D | **0.1015 m** | - | - | camera_corrected.tum.meta.json |
| corrected | every camera node placed by its graph node's correction, node 1 to node 718, floor plane | 0.110 m | - | 83.45 m | map_corrected_facts.json |
| corrected | **LiDAR map** (robot, colleague's self_navigation rtabmap_3d.launch), floor plane | **4.727 m** | - | 82.5 m | lidar_map_facts.json |
| corrected | LiDAR map, 3D (height included) | 8.7462 m | - | - | lidar.tum.meta.json |

**Loop closures beside the corrected gaps: camera map 145** (16 recognised-again + 129 nearby re-matches; database Link table types 1 and 2, counted once per pair; `closed_check.txt` / camera_corrected.tum.meta.json gives 145); the live monitor's last line said 111 accepted (`~/.run_records/s2_static_08/monitor_STATUS.txt`) - the database, read after the map closed, is the complete count. Rejected in `mapping.log`: 2. **LiDAR map 18** (lidar.tum.meta.json).

- **Closures joining a later node straight to node 1 (the start): 0**. So nothing ties the end straight to the start: the corrected gap is not a repeat of an end-to-start closure.
- Where the closures fell (newer node's time, the drive cut into four equal quarters): 0 / 4 / 29 / 112. First 15:54:58, last 16:01:08; "park" 16:01:35.
- 475 of 718 nodes are in the saved graph (drive 4: 284 of 540). The other 243 are still in the database's Node table with weight -9 (RTAB-Map keeps them but leaves them out of the map graph); 55 of them come after node 663 (16:01:08), 27 s before "park" (whole seconds). Every-node floor gap (next row of the table above): 0.110 m.

## 2. Every turn: robot vs camera vs blend

The live turn watcher's rule replayed after the drive over fusion.bag by `turns_replay.py` (unchanged; its control on drive 3 is in drive 4 RESULTS.md section 2). Output: turns_replayed.log -> turns.csv. *95th percentile = the value 95 % of turns stay under. The robot's own wheels + gyroscope is the comparison for short turns (not ground truth).*

| summary (26 turns) | median | 95th percentile | largest | within 1 deg |
|---|---|---|---|---|
| abs(blend - robot) | 0.25 deg | 1.05 deg | 2.4 deg | 24 of 26 |
| abs(camera - robot) | 3.20 deg | 35.77 deg | 55.0 deg | 2 of 26 |

Turns with no camera value (camera lost for the whole turn): 0. Figure: `turns_difference.png`.

| turn | ended | direction | s | robot deg | camera deg | blend deg | camera lost | blend - robot | camera - robot |
|---|---|---|---|---|---|---|---|---|---|
| 1 | 15:50:45 | right | 1.8 | -33.2 | -36.6 | -32.7 | 0 % | 0.5 | -3.4 |
| 2 | 15:50:48 | right | 1.6 | -29.7 | -24.2 | -29.3 | 38 % | 0.4 | 5.5 |
| 3 | 15:50:53 | right | 2.2 | -41.8 | -50.2 | -40.7 | 61 % | 1.1 | -8.4 |
| 4 | 15:50:57 | right | 2.2 | -41.2 | -41.3 | -41.1 | 26 % | 0.1 | -0.1 |
| 5 | 15:50:59 | right | 0.3 | -0.3 | -4.2 | -0.5 | 0 % | -0.2 | -3.9 |
| 6 | 15:51:11 | left | 0.1 | -0.2 | 3.4 | -0.2 | 0 % | -0.1 | 3.6 |
| 7 | 15:51:27 | right | 0.4 | -3.9 | -8.7 | -3.1 | 0 % | 0.8 | -4.8 |
| 8 | 15:52:20 | right | 10.3 | -90.5 | -93.1 | -90.6 | 15 % | -0.0 | -2.6 |
| 9 | 15:55:45 | right | 2.6 | -31.6 | -34.0 | -32.0 | 0 % | -0.4 | -2.4 |
| 10 | 15:55:52 | right | 2.1 | -16.4 | -18.1 | -16.6 | 0 % | -0.1 | -1.7 |
| 11 | 15:55:57 | right | 1.0 | -16.0 | -19.0 | -15.8 | 0 % | 0.3 | -3.0 |
| 12 | 15:56:02 | right | 0.9 | -8.7 | -9.9 | -8.7 | 0 % | -0.0 | -1.2 |
| 13 | 15:57:38 | left | 5.6 | 90.0 | 45.1 | 89.7 | 17 % | -0.3 | -44.9 |
| 14 | 15:57:42 | left | 0.8 | 14.8 | 69.8 | 15.8 | 27 % | 0.9 | 55.0 |
| 15 | 15:57:46 | left | 0.7 | 15.0 | 14.3 | 12.6 | 0 % | -2.4 | -0.7 |
| 16 | 15:57:49 | left | 0.2 | -0.3 | 4.5 | -0.0 | 0 % | 0.3 | 4.8 |
| 17 | 15:57:59 | left | 0.1 | -0.4 | 1.6 | -0.3 | 0 % | 0.1 | 2.0 |
| 18 | 15:58:31 | left | 0.2 | -0.3 | 5.4 | -0.4 | 0 % | -0.1 | 5.7 |
| 19 | 15:58:46 | left | 0.2 | -0.2 | 1.9 | -0.0 | 0 % | 0.2 | 2.1 |
| 20 | 15:59:09 | left | 1.3 | 12.9 | 15.0 | 12.7 | 0 % | -0.2 | 2.1 |
| 21 | 15:59:21 | right | 1.7 | -18.4 | -21.9 | -18.3 | 0 % | 0.0 | -3.5 |
| 22 | 15:59:38 | right | 5.7 | -86.7 | -90.1 | -87.3 | 0 % | -0.6 | -3.4 |
| 23 | 15:59:47 | left | 0.5 | 1.6 | 3.7 | 1.5 | 0 % | -0.1 | 2.1 |
| 24 | 16:00:48 | right | 5.3 | -75.1 | -76.8 | -74.9 | 0 % | 0.2 | -1.7 |
| 25 | 16:00:56 | right | 1.3 | -10.0 | -14.7 | -9.5 | 0 % | 0.5 | -4.7 |
| 26 | 16:01:01 | left | 2.4 | -4.1 | -2.5 | -3.5 | 0 % | 0.6 | 1.6 |

## 3. The WiFi link to the robot (the bridge)

*Source: `~/.run_records/s2_static_08/bridge_recv.log` (Jetson), its final counters; numbers.json (`down_lines_hamilton` = the log's UTC clock minus 4 h).*

- **Link down: 0 time(s)**. Connections: 1 in 1 receiver session(s).
- Robot-data silences over 1.5 s that did NOT have the freeze signature (freezes_pass_check.json, `other_wheel_silences_over_1p5s`): 0, longest 0.0 s.
- Parked-drop guard (`fused_odometry.log`): 4 line(s): 15:53:30 DROP GUARD: wheels and gyroscope silent for >= 0.3 s while PARKED - publishing "not moving; 15:53:36 DROP GUARD: wheels and gyroscope silent for >= 0.3 s while PARKED - publishing "not moving; 15:53:30 DROP GUARD ended after 0.75 s: the wheels came back (held throughout); 15:53:36 DROP GUARD ended after 0.01 s: the gyroscope came back (held throughout).

| stream | robot read | robot sent | published on the Jetson | % of sent | missed while down | too late (stale) | lost |
|---|---|---|---|---|---|---|---|
| /husky_velocity_controller/odom | 8999 | 8890 | 8823 | 99.25 | 0 | 72 | 0 |
| /imu/data | 17739 | 17524 | 17391 | 99.24 | 0 | 144 | 0 |
| /imu/data_raw | 17739 | 17524 | 17391 | 99.24 | 0 | 144 | 0 |
| /odometry/filtered | 45002 | 44459 | 44117 | 99.23 | 0 | 369 | 0 |

## 4. Camera tracking: losses, restarts, freezes, crashes

- **Camera program crashes: 1, recovered** (section 0): camera guard ended with restarts 1, state "ended".
- **Lost** (fusion.bag, `/rtabmap/odom`: all-zero pose or covariance 9999 - covariance = the tracker's own stated uncertainty; 9999 is its "I am lost" value): **9 stretches, 14.3 s in total, longest 4.0 s, 1.2 % of messages** (numbers.json; the outage is inside these).
- **Restarts from TF** (`mapping.log`, "Odometry automatically reset to latest odometry pose available from TF" - the camera's tracking gave up and restarted from the blend's position): **10**, in 5 burst(s): 15:50:48-15:50:55 (4); 15:52:19 (1); 15:53:10 (1); 15:57:05 (1); 15:57:38-15:57:40 (3). The mapping monitor's last line says resets=10 (`monitor_STATUS.txt`).
- **Freezes** (pass_check.py from `03_methods/jetson_stalls_drive3_2026-09-25/`, on this drive's recording -> freezes_pass_check.json): camera tracker silences > 1.5 s: 16 in all (5.07 s, 11.27 s, 1.53 s, 3.77 s, 4.40 s, 13.05 s, 5.93 s, 4.23 s, 3.50 s, 4.13 s, 6.63 s, 2.67 s, 6.03 s, 2.37 s, 7.03 s, 3.03 s); **outside the outage: 15**; 'ZED Diagnostic' pauses > 2 s: 17 (5.34 s, 11.76 s, 4.66 s, 5.30 s, 3.51 s, 6.42 s, 3.32 s, 6.19 s, 4.37 s, 4.12 s, 4.72 s, 7.38 s, 2.99 s, 6.66 s, 2.63 s, 7.98 s, 3.24 s); database saves >= 5.5 s: 0; control, camera gyroscope longest gap 10.772 s; map updates parsed 696 of 718 nodes. Verdict as printed: **VOID (control failed: the recorder or the whole Jetson stalled)** (pass line = zero silences).
  The check printed VOID because its control (the camera's own gyroscope, which must never pause more than 0.1 s) paused 10.8 s; [INFERENCE] that pause is the camera outage itself (the gyroscope comes from the camera program), as on drive 6.
- **Where the tracker silences come from** (`freeze_timing_drive7.py` -> freeze_timing.json, same gap rule): 16 silences; 15 overlap a 'ZED Diagnostic' pause of the camera program's picture loop; 1 overlap a database save of 1.5 s or more (12 such saves, longest 3.78 s). The outage is one of the 16.
- For comparison (chart.csv): drive 4 (GEN_2) 1 freeze in 619 s; drive 7 (GEN_1) 16 in 824 s.
- Database integrity (`db_check.txt`, quick_check, read-only): `PASS ~/slam_series2/s2_static_08.db: quick_check=['ok'], 718 map nodes, 0.7 s`

## 5. Blend heading while standing still (reported, not a pass line)

stops 72 (252 s still); signed sum **-0.554 deg**; sum of each stop's net change 1.392 deg; largest single stop -0.137 deg (numbers.json). Rule: robot wheel odometry speed and turn rate both zero, below 1e-6 - the log holds 1e-16 float noise, not 0.0 (nearest wheel message within 0.3 s).

## 6. Processor, power and temperature (new for this drive's run records)

*Source: `~/.run_records/s2_static_08/tegrastats.log` (tegrastats, once a second; its clock is UTC, converted), window 15:47:57 to 16:02:15 Hamilton, read by tegra_peaks.py -> power_temperature.json. Drive 4's figures: its pack's power_temperature.json (a separate tegrastats log).*

| quantity | drive 8 peak | when | drive 8 median | drive 4 peak / median |
|---|---|---|---|---|
| total power (four named supply rails) | **36.3 W** | 15:53:14 | 29.6 W | 36.6 / 29.8 W |
| hottest point on the chip (tj) | **61.5 C** | 16:00:51 | - | 61.6 C / - |
| processor temperature | 61.5 C | 16:00:51 | - | 61.6 C / - |
| graphics processor temperature | 54.9 C | 16:02:07 | - | 55.7 C / - |
| graphics processor load | 99 % | 15:49:51 | 22 % | 99 / 23 % |
| processor load, mean of the 12 cores | 100.0 % | 15:48:00 | 39.3 % | 99.9 / 41.2 % |
| memory free, lowest | 49.6 % (15.1 GB used) | 16:01:38 | - | 66.2 % / - |

- tegrastats skips over 2 s (the log itself missing seconds): 15:54:32 (5 s), 15:55:26 (4 s), 15:56:47 (4 s), 15:58:41 (7 s), 15:59:51 (3 s); samples 814 over 858 s.
- *Plain terms: the processor peak near 100 % at the start is the camera and mapping programs starting up; the median is the working load. The memory guard (earlyoom) kills the mapping program if free memory falls under 15 %; the lowest here is shown above.*
- RTAB-Map, time to process each map snapshot (database Statistics table, read-only): **median 515.3 ms, 95th percentile 716.0 ms, largest 3070.1 ms over 718 nodes** (numbers.json). Drive 4: 510.2 / 787.8 / 1852.7 ms over 540 nodes.

## 7. Agreement with the LiDAR estimate

*The reference is the robot's LiDAR map (a colleague's work: self_navigation `rtabmap_3d.launch`, replayed on the robot after the drive): the robot's wheels + gyroscope corrected by LiDAR loop closures. **An independent estimate, not ground truth** (ENGINEERING_NOTES.md rule 19): LiDAR range noise 15-30 mm, the two LiDARs agree to about 5 mm, and the LiDAR-to-camera mounting offset has never been measured. It shares the robot's wheels + gyroscope with the blend, so for the blend it is not fully independent. Alignment: SE(3) (rotation + shift), never scale (ENGINEERING_NOTES.md section 4 rule 4).*

- LiDAR map: 543 positions, 82.5 m, **18 LiDAR loop closures** (drive 4: 75); its own end gap 4.727 m on the floor, 8.746 m in 3D (section 1). Its own check (agreement.json, compare_lidar.py: ends within 0.5 m of its start AND at least one LiDAR closure): "FAILED ITS OWN CHECK: ends 4.73 m from its start although the robot parked on the start mark, with 18 LiDAR loop closures - the differences below measure the reference's drift as much as the camera's".
- **The LiDAR estimate FAILED its own check for drive 8, so it is not a yardstick here**: the numbers below compare two estimates.

| path compared | kind (rule 20) | matched | median | 95th percentile | largest | best-fit scale (NOT applied) | file |
|---|---|---|---|---|---|---|---|
| camera map, corrected (camera_corrected.tum) | map's corrected path | 99.8 % (474) | **1.971 m** | **3.085 m** | 3.420 m | 1.0131 | agreement.json |
| map nodes (= blend), tracking alone (camera.tum) | tracking alone | 92.2 % (662) | **1.818 m** | **3.031 m** | 3.212 m | 0.9143 | agreement.json |

- Clock shift: estimated -0.43 s, applied 0.0 s (compare_lidar.py applies one only if it improves the fit by > 0.05 m and 20 %).

### 7a. For information, size fitted (NOT the result)

*Why this is here: a taped 15.00 m test (`01_runs/series2_static/s2_scale_01/RESULTS.md`, same day) found the robot's wheels read about 6 % long and the camera within 1 %. The LiDAR estimate takes its size from the wheels, so part of the SE(3) distance above is the LiDAR estimate being too large, not the camera map's shape. Below, the same comparison is repeated allowing one size change (Sim(3)). For information only: the camera's size is real and is not to be fitted away (ENGINEERING_NOTES.md section 4 rule 4).* Made by `size_fit_agreement.py` -> size_fit.json, same reference, same 50 ms pairing, same fitting code as compare_lidar.py; its rigid (SE(3)) run reproduces agreement.json exactly (control: passed).

| path compared | pairs | SE(3) median / 95th pct (the result) | size fitted: median / 95th pct | size factor applied to the camera path |
|---|---|---|---|---|
| camera map, corrected | 474 | 1.971 / 3.085 m | 1.924 / 3.044 m (for information, size fitted) | 1.0131 |
| map nodes (= blend), tracking alone | 662 | 1.818 / 3.031 m | 2.185 / 3.287 m (for information, size fitted) | 0.9143 |

*Plain terms: a size factor of 1.013 = the corrected camera path would have to be stretched by 1.3 % to fit the LiDAR estimate best. The tape test says the robot's wheels + gyroscope read 1.064 times the true distance and the camera 0.993 times, so the size difference expected from the tape alone is 1.064 / 0.993 = 1.071. [INFERENCE] Drive 8's 1.013 against that: smaller than the tape alone predicts, by 0.058. With the LiDAR estimate failing its own check this drive, its size is no longer just the wheels' size, so this factor cannot be read as the camera's size - unlike drives 4 (1.063) and 7.*

## 8. Distances - from the camera and from the wheels

*Each source's own path length for the whole drive. The wheels read long (tape: robot wheels + gyroscope 6.4 % long, wheels alone 6.2 %, s2_scale_01 legs.json, legs 1, 2, 3, 6); the camera read within 1 %. "2 Hz" = positions every 0.5 s, straight steps summed (paths_2hz.csv).*

| source | what it is | path length | drive 4 | file |
|---|---|---|---|---|
| **camera map, corrected** | camera, after its loop closures (every node) | **83.5 m** | 85.1 m | map_corrected_facts.json |
| camera tracking, 2 Hz | camera's own tracking (re-seeded by the blend at restarts) | 88.4 m | - | paths_2hz.csv |
| map nodes, tracking alone | the blend's position at each node | 86.5 m | 87.2 m | facts.json |
| blend, 2 Hz | camera + wheels + gyroscopes | 87.0 m | - | paths_2hz.csv |
| **robot's own wheels + gyroscope, 2 Hz** | wheels (what the LiDAR map is built on) | **87.4 m** | - | paths_2hz.csv |
| LiDAR map | wheels + gyroscope corrected by LiDAR closures | 82.5 m | 88.8 m | lidar_map_facts.json |
| robot's wheels + gyroscope x 0.940 (for information: the tape's correction) | wheels, resized by the taped test | 82.2 m | - | computed here |

**Distance to quote for drive 8: about 83 m by the camera map** (the wheels say 87 m, which the tape shows reads about 6 % long).

## 9. Drive 4 vs drive 8 (drive 7 beside them) - did drive 8 repeat drive 4?

**In plain words: partly.** What repeated: drive 8 was driven along drive 4's line (75 of its 83 m within 0.5 m of drive 4's route; on that part a median 0.06 m apart), and its **camera map, corrected**, came home to 0.10 m with 145 loop closures, none joining the start, against drive 4's 0.05 m with 181 closures, 3 of them joining the start - within the parking uncertainty. The turns agree too. What did not repeat: the camera map made fewer closures (17.4 per 10 m against 21.3, 1.8 times the counting spread); the camera's tracking alone ended 1.53 m out against 0.25 m (drive 8 includes the camera crash, which drive 4 did not have); the camera froze 15 times against 1 on the same GEN_2 setting; and the LiDAR map made 18 closures against 75 and ended 4.73 m from its start, failing its own check, so drive 8 has no LiDAR yardstick of its own.

**Verdict, drive 8 against drive 4, camera side (computed): of 13 judged rows about the camera, the blend and the mapping program, 5 agree within the combined uncertainty, 6 differ by 1-3 times it, and 2 differ by more than 3 times it: start-to-end gap, camera tracking alone (0 closures) (4.5 times); camera freezes (tracker silent > 1.5 s), per 10 min (3.3 times).** Counted separately, not in that tally: the robot's own wheels + gyroscope (0 / 2 / 0 agree / 1-3 times / over 3 times - they are not the camera); the LiDAR map (0 / 0 / 2 - one LiDAR failure seen twice); and the 8 agreement-with-the-LiDAR-estimate rows, shown but not counted for drive 8 because its LiDAR estimate failed its own check (it ends 4.73 m from its start), so they measure that failure, not the camera. Drive 7 against drive 4 on the same camera-side rows (13 of them - drive 7 has no processor log): 5 agree, 4 differ by 1-3 times, 4 by more than 3 times; drive 7's LiDAR estimate also failed its own check, so its LiDAR-agreement verdicts in the table are for reference only.

*How to read this: two drives never give identical numbers. For each row the spread one drive is expected to have is stated, with where it comes from (column "from"); the two are combined as sqrt(s4^2 + s8^2). "Agrees within X" = the difference is smaller than that; "differs by N times" = it is N times larger (3 or more is a real difference). The distance spreads are working figures, not a measured repeatability - N = 1 drive each. Scope: "shared route" rows use only the part of each route that the two drives both covered (definition below); "whole drive" rows are normalised per 10 min or per 10 m where the number grows with length.*

**Shared route - how it was defined.** camera map corrected path (camera_corrected.tum), filled in every 5 cm; drives 7 and 8 lined up on drive 4 by 2D rotation + shift (ICP, pairs < 1.0 m); a point is on the shared route if within 0.5 m of the other drive's route; direction of travel not tested. Drive 8 lined up on drive 4 by -3.4 deg and 0.40 m; after that its start sits 0.40 m from drive 4's. **Drive 8: 75.2 of its 83.0 m within 0.5 m of drive 4's route; drive 4: 83.1 of 84.7 m within 0.5 m of drive 8's.** On the shared part, drive 8's route runs a median 0.06 m (95th percentile 0.30 m) from drive 4's. Sensitivity (drive 8's shared length): 0.3 m: 71.4 m, 0.5 m: 75.2 m, 0.75 m: 75.9 m, 1.0 m: 76.4 m. The unshared part is the far end of the side corridor: drive 8 went 11.1 m down it from the start against drive 4's 6.5 m (drive 7: 14.8 m; camera maps corrected, drive 4's map frame) - further in than the planned about 6.5 m. Drive 7 for comparison: 73.1 of 103.2 m shared. Route lengths here (saved-graph nodes, filled in every 5 cm) differ slightly from the every-node path lengths used in the per-10 m row and in section 8 (drive 8: 83.0 against 83.5 m). Figure: `shared_route.png`.

| row | group | scope | drive 4 | drive 7 | drive 8 | spread per drive | from | drive 8 vs drive 4 | drive 7 vs drive 4 |
|---|---|---|---|---|---|---|---|---|---|
| route: shared length (drive's own / drive 4's, within 0.5 m) | context | route | - | 73.1 of 103.2 m / 82.5 of 84.7 m | 75.2 of 83.0 m / 83.1 of 84.7 m | - | - | not judged: defines the comparison (shared_route.json) | - |
| distance driven - camera map corrected / robot's own wheels + gyroscope | context | whole drive | 85.1 m / 90.1 m | 103.4 m / 108.7 m | 83.5 m / 88.6 m | - | - | not judged: a description of the route; the wheels read about 6 % long (tape test) | - |
| drive duration | context | whole drive | 619 s | 824 s | 850 s | - | - | not judged: description | - |
| camera map route vs drive 4's LiDAR path, sideways distance on the shared route: median / 95th pct | context | shared route | 0.02 / 0.10 m (its own) | - | 0.06 / 0.27 m | - | - | not judged: shows drive 8 was driven along the same line; NOT an independent check of the map's shape - only points already within 0.5 m of drive 4's camera route are kept (so the 95th percentile cannot much exceed 0.5 m) and drive 4's camera route lies 0.02 m from its own LiDAR path; path-to-path, says nothing about distance along the route (shared_route.json) | - |
| camera tracker setting | context | whole drive | GEN_2 | GEN_1 | GEN_2 | - | - | not judged: the one setting that was changed between drives | - |
| start-to-end gap, camera tracking alone (0 closures) | camera and blend | whole drive | 0.25 m | 0.60 m | 1.53 m | 0.20 m | A | **differs** by 4.5 times (difference 1.28 m, combined 0.28 m) | differs by 1.2 times (difference 0.35 m, combined 0.28 m) |
| start-to-end gap, map nodes tracking alone (= blend) | camera and blend | whole drive | 0.06 m | 0.99 m | 0.60 m | 0.20 m | A | **differs** by 1.9 times (difference 0.54 m, combined 0.28 m) | differs by 3.3 times (difference 0.94 m, combined 0.28 m) |
| start-to-end gap, robot's own wheels + gyroscope, tracking alone | robot's wheels + gyroscope | whole drive | 0.27 m | 0.12 m | 0.98 m | 0.20 m | A | **differs** by 2.5 times (difference 0.71 m, combined 0.28 m) | agrees within 0.28 m (difference 0.14 m) |
| heading gap, robot's own wheels + gyroscope | robot's wheels + gyroscope | whole drive | +2.3 deg | +2.7 deg | +8.4 deg | 3.0 deg | B | **differs** by 1.4 times (difference 6.1 deg, combined 4.2 deg) | agrees within 4.2 deg (difference 0.4 deg) |
| start-to-end gap, **camera map corrected** (floor) | camera and blend | whole drive | 0.05 m | 0.17 m | 0.10 m | 0.20 m | A | **agrees** within 0.28 m (difference 0.05 m) - closures beside it (rule 20): drive 4 181 (3 join the start), drive 8 145 (0 join the start) | agrees within 0.28 m (difference 0.12 m) |
| start-to-end gap, camera map corrected, 3D | camera and blend | whole drive | 0.05 m | 0.17 m | 0.10 m | 0.20 m | A | **agrees** within 0.28 m (difference 0.05 m) - same closures as the floor row: drive 4 181 (3 join the start), drive 8 145 (0) | agrees within 0.28 m (difference 0.12 m) |
| camera map loop closures (count) | camera and blend | whole drive | 181 | 30 | 145 | 12 | C | **differs** by 2.0 times (difference 36, combined 18) | differs by 10.4 times (difference 151, combined 15) |
| camera map loop closures per 10 m of its route | camera and blend | whole drive | 21.3 | 2.9 | 17.4 | 1.4 | D | **differs** by 1.8 times (difference 3.9, combined 2.1) | differs by 11.0 times (difference 18.4, combined 1.7) |
| closures joining the end to the start | camera and blend | whole drive | 3 | 0 | 0 | 1 | E | **differs** by 1.5 times (difference 3, combined 2) | differs by 1.5 times (difference 3, combined 2) |
| start-to-end gap, **LiDAR map corrected** (floor) | LiDAR map | whole drive | 0.03 m | 0.16 m | 4.73 m | 0.20 m | A | **differs** by 16.6 times (difference 4.70 m, combined 0.28 m) | agrees within 0.28 m (difference 0.13 m) |
| LiDAR map loop closures (count) | LiDAR map | whole drive | 75 | 0 | 18 | 4 | E | **differs** by 5.9 times (difference 57, combined 10) | differs by 8.6 times (difference 75, combined 9) |
| LiDAR estimate passes its own check | context | whole drive | yes | NO | NO | - | - | not judged: a pass/fail check | - |
| camera freezes (tracker silent > 1.5 s), per 10 min | camera and blend | whole drive | 1.0 | 11.6 | 10.6 | 2.7 | F | **differs** by 3.3 times (difference 9.6, combined 2.9) - counts: 1 / 16 / 15; drive 8's camera outage is not counted as a freeze | differs by 3.5 times (difference 10.7, combined 3.1) |
| camera tracking restarts from the blend (count) | camera and blend | whole drive | 26 | 9 | 10 | 3 | C | **differs** by 2.7 times (difference 16, combined 6) | differs by 2.9 times (difference 17, combined 6) |
| camera tracking lost, seconds per 10 min | camera and blend | whole drive | 25.4 s | 10.8 s | 10.1 s | 6.9 s | G | **differs** by 1.6 times (difference 15.3 s, combined 9.8 s) | differs by 1.5 times (difference 14.5 s, combined 9.8 s) |
| camera program crashes | context | whole drive | none | none (camera guard: 0 restarts) | 1, recovered: 15:52:57, back after 12.7 s (camera guard); mapping carried on | - | - | not judged: one event is not a rate; see RESULTS section 0 | - |
| turns: abs(camera - robot), median of all turns | camera and blend | whole drive | 2.3 deg | 2.7 deg | 3.2 deg | 1.0 deg | H | **agrees** within 1.5 deg (difference 0.9 deg) - turns: 16 / 13 / 26 | agrees within 1.5 deg (difference 0.4 deg) |
| turns: abs(blend - robot), median of all turns | camera and blend | whole drive | 0.2 deg | 0.2 deg | 0.2 deg | 0.2 deg | H | **agrees** within 0.2 deg (difference 0.0 deg) | agrees within 0.2 deg (difference 0.0 deg) |
| **agreement with the LiDAR estimate, ON THE SHARED ROUTE**, camera map corrected: median (SE(3)) | agreement with the LiDAR estimate | shared route | 0.26 m | 0.35 m | 2.02 m | 0.08 m | I | **differs** by 15.3 times (difference 1.77 m, combined 0.12 m) - drive 4's figure is over its part shared with drive 8 (with drive 7: 0.26 m); pairs 271 / 430 / 426. - drive 8's LiDAR estimate FAILED its own check (ends 4.73 m from its start), so this row is NOT counted in the verdict | agrees within 0.12 m (difference 0.10 m) (drive 7's LiDAR estimate failed its own check: reference only) |
| **agreement with the LiDAR estimate, ON THE SHARED ROUTE**, camera map corrected: 95th percentile (SE(3)) | agreement with the LiDAR estimate | shared route | 0.47 m | 0.76 m | 3.11 m | 0.27 m | I | **differs** by 6.9 times (difference 2.64 m, combined 0.38 m) - drive 8's LiDAR estimate FAILED its own check (ends 4.73 m from its start), so this row is NOT counted in the verdict | agrees within 0.38 m (difference 0.29 m) (drive 7's LiDAR estimate failed its own check: reference only) |
| **agreement with the LiDAR estimate, ON THE SHARED ROUTE**, map nodes (= blend) tracking alone: median (SE(3)) | agreement with the LiDAR estimate | shared route | 0.11 m | 0.38 m | 1.90 m | 0.08 m | I | **differs** by 15.5 times (difference 1.79 m, combined 0.12 m) - drive 4's figure is over its part shared with drive 8 (with drive 7: 0.12 m); pairs 466 / 515 / 603. - drive 8's LiDAR estimate FAILED its own check (ends 4.73 m from its start), so this row is NOT counted in the verdict | differs by 2.3 times (difference 0.27 m, combined 0.12 m) (drive 7's LiDAR estimate failed its own check: reference only) |
| **agreement with the LiDAR estimate, ON THE SHARED ROUTE**, map nodes (= blend) tracking alone: 95th percentile (SE(3)) | agreement with the LiDAR estimate | shared route | 0.24 m | 0.60 m | 3.03 m | 0.27 m | I | **differs** by 7.3 times (difference 2.79 m, combined 0.38 m) - drive 8's LiDAR estimate FAILED its own check (ends 4.73 m from its start), so this row is NOT counted in the verdict | agrees within 0.38 m (difference 0.37 m) (drive 7's LiDAR estimate failed its own check: reference only) |
| agreement with the LiDAR estimate, camera map corrected, whole drive: median (SE(3)) | agreement with the LiDAR estimate | whole drive | 0.26 m | 0.45 m | 1.97 m | 0.08 m | I | **differs** by 14.9 times (difference 1.71 m, combined 0.12 m) - drive 8's LiDAR estimate FAILED its own check (ends 4.73 m from its start), so this row is NOT counted in the verdict | differs by 1.6 times (difference 0.19 m, combined 0.12 m) (drive 7's LiDAR estimate failed its own check: reference only) |
| agreement with the LiDAR estimate, camera map corrected, whole drive: 95th percentile (SE(3)) | agreement with the LiDAR estimate | whole drive | 0.49 m | 0.88 m | 3.08 m | 0.27 m | I | **differs** by 6.8 times (difference 2.59 m, combined 0.38 m) - drive 8's LiDAR estimate FAILED its own check (ends 4.73 m from its start), so this row is NOT counted in the verdict | differs by 1.0 times (difference 0.39 m, combined 0.38 m) (drive 7's LiDAR estimate failed its own check: reference only) |
| for information, size fitted (Sim(3)): camera map corrected, whole drive, median | agreement with the LiDAR estimate | whole drive | 0.09 m | 0.16 m | 1.92 m | 0.08 m | J | **differs** by 15.9 times (difference 1.83 m, combined 0.12 m) | agrees within 0.12 m (difference 0.07 m) (drive 7's LiDAR estimate failed its own check: reference only) |
| best-fit size factor, camera map corrected vs LiDAR estimate (not applied) | agreement with the LiDAR estimate | whole drive | 1.063 | 1.078 | 1.013 | 0.015 | K | **differs** by 2.4 times (difference 0.050, combined 0.021) | agrees within 0.021 (difference 0.015) (drive 7's LiDAR estimate failed its own check: reference only) |
| RTAB-Map time per map snapshot, median | processor | whole drive | 510 ms | 525 ms | 515 ms | 49 ms | L | **agrees** within 70 ms (difference 5 ms) | agrees within 70 ms (difference 15 ms) |
| total power, median | processor | whole drive | 29.8 W | - (no tegrastats log) | 29.6 W | - | - | not judged: the drive-to-drive spread of drives 4, 5 (camera-alive part) and 6 (0.3 W) comes from 3 drives and is too small to trust as a yardstick; drive 8's window also starts about 34 s before its live recorder (start-up load included) | - |
| hottest point on the chip (tj), peak | processor | whole drive | 61.6 C | - (no tegrastats log) | 61.5 C | - | - | not judged: the drive-to-drive spread of drives 4, 5 (camera-alive part) and 6 (0.1 C) comes from 3 drives and is too small to trust as a yardstick; drive 8's window also starts about 34 s before its live recorder (start-up load included) | - |
| processor load, mean of 12 cores, median | processor | whole drive | 41.2 % | - (no tegrastats log) | 39.3 % | - | - | not judged: the drive-to-drive spread of drives 4, 5 (camera-alive part) and 6 (0.2 %) comes from 3 drives and is too small to trust as a yardstick; drive 8's window also starts about 34 s before its live recorder (start-up load included) | - |

Spread sources (column "from"):
- **A** = where the robot parked: s2_scale_01 found stops at one mark up to 0.19 m apart even with a wheel stop (its RESULTS.md, uncertainty); drives 4, 7 and 8 parked on the start mark by eye
- **B** = parking heading by eye, working figure (not measured)
- **C** = counting spread, the square root of each count (Poisson, the least spread a count of chance events has)
- **D** = counting spread, the square root of each count (Poisson, the least spread a count of chance events has), scaled
- **E** = counting spread, the square root of each count (Poisson, the least spread a count of chance events has) (at least 1)
- **F** = counting spread, the square root of each count (Poisson, the least spread a count of chance events has) (at least 1), scaled
- **G** = drive-to-drive spread (sample standard deviation) of lost seconds per 10 min, drives 3, 4, 5, 6
- **H** = drive-to-drive spread (sample standard deviation) of the per-turn median, drives 4, 5, 6
- **I** = drive-to-drive spread (sample standard deviation) of the camera map's whole-drive agreement on drives 3, 4, 5, the drives whose LiDAR estimate passed its own check; different routes, so it may overstate the spread for a repeat
- **J** = drive-to-drive spread (sample standard deviation) of the camera map's whole-drive agreement on drives 3, 4, 5, the drives whose LiDAR estimate passed its own check; different routes, so it may overstate the spread for a repeat (borrowed)
- **K** = drive-to-drive spread (sample standard deviation) of drives 3, 4, 5 (valid LiDAR estimates)
- **L** = drive-to-drive spread (sample standard deviation) of drives 4, 5, 6 (numbers.json processor_rtabmap)

Caveats: (1) the 0.20 m for tracking-alone gaps is only the parking floor; how much tracking drift varies from drive to drive is not known and is surely larger, so "differs" on those rows is weaker than it reads. (2) The agreement spreads come from whole drives on different routes; the shared-route rows borrow them, and the map-nodes (= blend) rows borrow the camera map's spread although the blend's own drive-to-drive spread is far larger (drives 3, 4, 5: 1.84 / 0.12 / 0.12 m median), so drive 7's "differs" on those rows is inflated. (3) The shared-route agreement uses the whole-drive SE(3) alignment, not one refitted on the shared part. (4) Counts use the Poisson floor, the least spread a count can have; real drive-to-drive spread of counts is likely larger.

## 10. Figures, video and 3D map in this folder

| file | what it shows |
|---|---|
| map.png | camera floor map, map nodes at their tracking-alone positions (Node.pose = the blend's) |
| map_corrected.png | camera floor map after the loop closures (Admin.opt_poses) - the map RTAB-Map believes in |
| trajectory.png | where each source put the robot, two panels (rule 20): tracking alone (robot solid; camera and blend dashed) and corrected (camera map dashed; LiDAR estimate solid); the camera outage marked |
| turns_difference.png | per turn: camera minus robot and blend minus robot, degrees |
| lidar_comparison.png | camera paths over the LiDAR map (LiDAR solid, camera dashed) and the distance to the LiDAR estimate along the route (SE(3)) |
| lidar_map.png | the LiDAR's own floor map |
| shared_route.png | drive 8's and drive 7's routes lined up on drive 4's (camera maps corrected, dashed); the shared part drawn heavy, the rest black |
| `04_figures/camera_vs_lidar_2026-09-26/drive8_camera_vs_lidar.png` | camera map vs LiDAR map, same scale (chart figure) |
| timelapse.mp4 | **recorded live during the drive**: the live map page (camera map left, LiDAR view right), 266 frames (`timelapse_live_index.csv`; recorder.log last line: `2026-09-26 16:02:43 EDT build: ~/.run_records/s2_static_08/media/s2_static_08_timelapse.mp4 - 266 frames at`) |
| timelapse_25/50/75/100.png | stills at 25/50/75/100 % of the timelapse (video frames 66, 133, 199, 265, counted from 0; extracted with ffmpeg) |
| map_3d/drive8_map_3d.ply, map_3d/d8_3d.png | the camera map exported to 3D like drive 4's: `rtabmap-export --cloud --max_range 6 --decimation 4 --voxel 0.03` (depth up to 6 m, every 4th pixel, points merged into 3 cm cubes) on a copy of the database (export.log); d8_3d.png = two views (render.py) |

Other files: camera.tum / camera_corrected.tum (+ .meta.json); paths_2hz.csv; turns.csv, turns_replayed.log; numbers.json; crash_split.json; freezes_pass_check.json; freeze_timing.json; power_temperature.json; closure_span.json; db_check.txt; closed_check.txt; size_fit.json; shared_route.json; compare_drives_4_7_8.json/.md; lidar.tum, wheel.tum, lidar_map.npz, lidar_reference_dense.tum, lidar_comparison/; products.log, provenance.txt (the robot's replay).

## 11. Still open

- N = 1 per drive: drive 4 vs drive 8 is one repeat, not five (ENGINEERING_NOTES.md section 4 rule 2). A spread of repeat drives needs at least three more on this route.
- The camera crash's root cause is inside the ZED SDK (backtrace, section 0); the guard recovers it but the fault remains.
- The distance spreads used in section 9 are working figures (parking by eye), not a measured repeatability.
- **Why the LiDAR map failed** (ends 4.73 m from its start with 18 closures, while its own input, the robot's wheels + gyroscope, ended 0.98 m out) is not established. [INFERENCE] some LiDAR closures were wrong: by eye its walls are smeared and rotated (`lidar_map.png`, `04_figures/camera_vs_lidar_2026-09-26/drive8_camera_vs_lidar.png`). Check the replay on the robot when it is back from charging (not contacted for this pack).
- **Camera freezes on GEN_2** (15 outside the outage, section 4) contradict drive 7's reading that the freezes go with GEN_1; the picture-loop pauses happen on both settings. Their cause is open.
- The camera map closed no loops in the first quarter of the drive and most (112 of 145) in the last quarter (section 1); drive 4's database was no longer on the Jetson, so the same split could not be made for it here.
- The camera's path is not independent of the blend in a fused drive (section 1).
