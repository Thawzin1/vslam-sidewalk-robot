# Drive 5 (s2_static_05) - results pack

*26 Sept 2026, Hamilton time: live recorder started 02:13:59, "ready" 02:14:12, **camera program crashed 02:25:26**, "park" 02:34:48, map closed properly 02:37:55 (by the park helper's 3-minute timeout). The third fused drive (fused = the camera blended with the robot's wheels and gyroscope (turn-rate sensor) by an EKF (extended Kalman filter, a standard way of mixing sensors)). Made on the Jetson on 26 Sept by `results_packs_2026-09-26/` (project records) (pack_drive5_heavy.sh, analyze_run.py, turns_replay.py, extract_timing.py, tegra_peaks.py, crash_split_drive5.py, compare_lidar.py, this file's writer `write_results_drive5.py`). Every number names the file it comes from; files are in this folder unless a path is given. Nothing here is ground truth (ENGINEERING_NOTES.md rule 19). N = 1.*

**What was different from drive 4** (each checked in the run's own files):

| change | plain meaning | evidence |
|---|---|---|
| confirmed saves off: `DbSqlite3/Synchronous=0` (drive 4: 1) | the map program no longer waits for the disk to confirm each save; meant to remove the camera freezes (section 4). Risk: a damaged database if the Jetson hangs or loses power mid-save | the database's Info table; `catkin_ws/src/sidewalk_slam/config/rtabmap_zedx.yaml` lines 715-724 (changed 26 Sept 01:40, after drive 4); `03_methods/jetson_stalls_drive3_2026-09-25/FINDINGS.md` line 116 |
| a person stood behind the robot at the start | the LiDAR clearing test (does the LiDAR map erase someone who walked away?) - section 8 | `~/.run_records/s2_static_05/staged_person.txt` |
| the camera program was NOT restarted | drive 5 reused the camera program still running from drive 4 (started about 00:43); it crashed 1 h 41 min after its start | `~/jobs/s2_static_05_start_drive.out` step 2: "already publishing"; `docs/DO_NOT_REPEAT.md`, "Drive 5" |
| 16-bit depth, camera gyroscope in the blend, parked-drop guard | unchanged from drive 4 | `Mem/SaveDepth16Format:true`; `fused_odometry.log` |

## 0. The camera crash - read this before any number below

*Plain terms: 11 minutes into the drive the camera program died. The robot kept driving for another 9 minutes on its wheels and gyroscope alone, but the map stopped growing at that moment, because the map is built from camera pictures.*

- **What happened:** `zed_wrapper_node` (the camera program) crashed with a segmentation fault (a memory-access error; exit code -11) at **02:25:26.2** (`~/.ros/log/56b222ca-b95d-11f1-b68f-48e7da41463d/roslaunch-jetson-118172.log` line 63; the same session's screen log is `~/.run_records/s2_static_04/camera.log` lines 196-200, because the camera session was started for drive 4). Marked REQUIRED in its launch file, so its whole launch shut down. No camera recording (SVO) was on. The last camera messages the recorder got: gyroscope 02:25:25.5, tracker 02:25:25.5, the camera program's own status (ZED Diagnostic) 02:25:25.1 (`crash_split.json`, from fusion.bag).
- **Nobody was told for 9 minutes:** the drive's progress line kept saying "running"; the only sign was the park helper's "NO camera positions for N s" (`autostop.log`). The fix (fresh camera every drive, a guard that restarts it, a CAMERA DOWN warning) is in `03_methods/camera_crash_recovery_2026-09-26/`. Root cause of the crash: unknown.

| | with the camera (ready 02:14:12 to crash) | without the camera (crash to "park" 02:34:48) | source |
|---|---|---|---|
| minutes | **11.2** | **9.4** (12.4 to the map closing) | staged_person.txt, roslaunch log |
| distance, robot's own wheels + gyroscope (/robot/ekf_odom) | **61.9 m** | **138.9 m** | crash_split.json |
| distance, blend (/fused/odometry) | 62.0 m | 134.7 m - **blend only**: after the crash the blend had wheels + robot gyroscope and nothing from the camera (its gyroscope died with it) | crash_split.json |
| map snapshots (nodes) added | **688** (node 1 at 02:13:27 to node 688 at 02:25:25) | **0** | database Node table |
| map updates in `mapping.log` | - | **0** | mapping.log |
| live-map timelapse frames (one every 3 s; the recorder started at 02:13:59, 13 s before "ready", so this column counts from the recorder's start, not from "ready") | 229 (frames 1-229) | 251: frame 230 (02:25:26) is the first after the crash and still shows the camera's last picture; from frame 231 (02:25:29) on, 250 frames say "NO DATA for N s - the odometry node may have stopped" | media/index.csv |

*Distances: positions every 0.5 s, straight steps summed, the same for every source (`crash_split.json`). Section 1's path lengths (204.8 m robot, 203.2 m blend, numbers.json) are longer than these (200.8 m, 196.7 m) only because they sum every message at full rate, so small side-to-side wobble adds up; neither is wrong, but only same-method figures may be compared (docs/SOLVED.md). The robot's own estimate recorded on the robot (`wheel.tum`, from its LiDAR recording) gives the same 61.9 m / 138.9 m - it is the same stream, recorded twice.*

**What the map contains after 02:25:** nothing. The map program only adds a snapshot when a camera picture arrives, so the camera map covers the first 31 % of the distance (61.9 of 200.8 m by the robot's own estimate) and stops where the robot was at 02:25:25. The 138.9 m driven after that are in the blend, the robot's own estimate and the LiDAR map, but not in the camera map. The map was still closed properly: `closed_check.txt` (CLOSED PROPERLY - safe to power off.).

## 1. How far from home at the end - both kinds (rule 20)

*Plain terms: "tracking alone" = where each source's own step-by-step tracking (odometry) put the robot, never corrected; "corrected" = where the map puts it after its loop closures (the map recognising a place it has seen before and pulling itself straight). A "node" is one saved map snapshot; the "graph" is the nodes and the links between them. Admin.opt_poses = the corrected node positions RTAB-Map saves when it shuts down properly; Node.pose = each node's tracking-alone position. The drive was planned to end on its start mark; the real end position was not measured with a tape.*

**In a fused drive the map's nodes carry the BLEND's position, not the camera's** (drive 4 RESULTS.md section 1: the map reads odom->base_link from TF (ROS's shared table of where each part of the robot is), and that comes from the blend; `~/.run_records/s2_static_05/tf_check_2.txt`). Each camera restart (section 4) re-started the camera's tracking from the blend's position, so "camera tracking" is not an independent camera-only path.

**The camera map's gaps are NOT end-to-start gaps in this drive.** Its last node is where the robot was at the crash, mid-route. Node 1 to node 688 measures how far the robot was from its start at 02:25:25, not how well the map came home: the robot's own wheels + gyroscope put it **8.41 m** from the start at 02:25:25.5 (paths_2hz.csv, the 2-per-second sample nearest node 688), close to the map nodes' 7.906 m tracking alone and 7.968 m corrected below. The two do not start at the same moment: the paths in paths_2hz.csv start at the recording's first message (02:13:45), node 1 at 02:13:27, so a few centimetres between them mean nothing. Only the blend and the robot's own estimate reached the end of the drive, so only their gaps are end-to-start gaps.

| kind | source | covers | gap from its first to its last position | heading gap | path length | file |
|---|---|---|---|---|---|---|
| tracking alone | map nodes (Node.pose = the blend's position at each node), node 1 to node 688 | to the crash | 7.906 m (NOT end-to-start) | - | 60.98 m | facts.json, camera.tum |
| tracking alone | camera tracking (/rtabmap/odom, lost stretches left out; re-seeded by the blend at each restart) | 02:13:45-02:25:25, to the crash | 8.279 m (NOT end-to-start) | +89.17 deg | 86.2 m | numbers.json (fusion.bag) |
| tracking alone | blend (/fused/odometry) - camera in it only to 02:25:26 | 02:13:45-02:37:52, whole drive | **3.453 m** | -3.32 deg | 203.2 m | numbers.json (fusion.bag) |
| tracking alone | robot's own wheels + gyroscope (/robot/ekf_odom) | 02:13:45-02:37:52, whole drive | **0.058 m** | +3.87 deg | 204.8 m | numbers.json (fusion.bag) |
| **corrected** | map's saved graph, Admin.opt_poses, node 1 to node 688 (280 graph nodes) | to the crash | 7.9676 m (NOT end-to-start) | - | - | camera_corrected.tum.meta.json |
| corrected | every node placed by its graph node's correction, node 1 to node 688 | to the crash | 7.968 m (NOT end-to-start) | - | 60.08 m | map_corrected_facts.json |

**Loop closures in the camera map: 72** (10 recognised-again + 62 nearby re-matches; database Link table, types 1 and 2; `closed_check.txt` gives the same total), all made before 02:25:25. None joins a later node to node 1 (the start) - the drive never came back to its start while the camera was alive. The live mapping monitor's last line reads 78 accepted (`monitor_STATUS.txt`); the database, read after the map closed, holds 72 and is the complete count.

**End-to-start, tracking alone, whole drive: the blend ended 3.453 m / -3.32 deg from its start; the robot's own wheels + gyroscope 0.058 m / +3.87 deg.** The blend had no camera for its last 134.7 m, so this is mostly a wheels + gyroscope result - yet it ended 3.4 m further from the start than the robot's own estimate from the same wheels + gyroscope. The largest single-turn difference after the crash is turn 49 (blend 86.1 deg against the robot's 76.7 deg, section 2), ended 02:29:43 - before the first link drop (02:29:59, section 3), so not caused by it. [INFERENCE] The blend's own settings (its gyroscope offset, the camera gyroscope input going silent) are the likely place to look; not investigated here. Drive 4 for scale: blend 0.064 m / -1.1 deg, robot 0.266 m / +2.3 deg over 90.1 m (drive 4 numbers.json) - a different length and a camera present to the end, so not like-for-like; reported only.

*Drive 3's pass line ("after its own corrections the map comes home within 0.3 m of its start mark", NEXT_PLAN_2026-09-25.md item 3) cannot be applied: the camera map never came home. It is not a pass and not a fail.*

## 2. Every turn: robot vs camera vs blend

*Plain terms: for each turn, how many degrees each source says the robot turned. The robot's own wheels + gyroscope is the comparison for short turns (not ground truth). "95th percentile" = the value 95 % of turns stay under.*

The live turn watcher did not run during drive 5 (no turns.log in `~/.run_records/s2_static_05/`). Its rule was replayed over the drive's recording by `turns_replay.py`, unchanged, as for drive 4 (control on drive 3: 52 of 52 turns found, heading agreement median 0.0 deg - `control_drive3_turns_replayed.log`). Output: `turns_replayed.log` -> turns.csv.

| summary | median | 95th percentile | largest | within 1 deg |
|---|---|---|---|---|
| abs(blend - robot), 32 turns with the camera alive | 0.40 deg | 0.84 deg | 1.3 deg | 31 of 32 |
| abs(camera - robot), same turns | 3.50 deg | 14.80 deg | 26.8 deg | 2 of 31 |
| abs(blend - robot), 34 turns after the crash (blend = wheels + robot gyroscope only) | 0.65 deg | 1.70 deg | 9.4 deg | 25 of 34 |

Drive 4 for scale (its RESULTS.md section 2, 16 turns): blend within 1 deg on 16 of 16, largest 0.5 deg; camera median 2.3, largest 21.8 deg. Figure: `turns_difference.png` (after the crash there is no camera point).

*"started turning" = the side the wheels were turning when the turn began; "net" = the sign of the robot's own heading change (left = positive degrees). They differ on: turn 9; turn 16; turn 26; turn 32; turn 33; turn 36; turn 42; turn 46; turn 54; turn 55; turn 60; turn 64; turn 66.*

| turn | ended | camera alive? | started turning | net | s | robot deg | camera deg | blend deg | camera lost | blend - robot | camera - robot |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 02:17:26 | yes | left | left | 2.2 | 31.8 | 33.9 | 32.6 | 0 % | 0.8 | 2.1 |
| 2 | 02:17:29 | yes | left | left | 1.4 | 32.5 | 17.1 | 31.9 | 82 % | -0.5 | -15.4 |
| 3 | 02:17:32 | yes | left | left | 0.9 | 21.9 | - | 21.5 | 100 % | -0.5 | - |
| 4 | 02:17:39 | yes | left | left | 0.4 | 3.5 | 7.1 | 3.1 | 0 % | -0.4 | 3.6 |
| 5 | 02:17:43 | yes | left | left | 0.4 | 2.0 | 5.3 | 1.9 | 14 % | -0.1 | 3.3 |
| 6 | 02:17:47 | yes | left | left | 0.9 | 15.4 | 18.4 | 15.8 | 0 % | 0.3 | 3.0 |
| 7 | 02:17:50 | yes | left | left | 0.5 | 5.8 | 10.0 | 6.1 | 0 % | 0.3 | 4.2 |
| 8 | 02:18:01 | yes | right | right | 0.2 | 0.0 | -4.0 | -0.5 | 0 % | -0.6 | -4.0 |
| 9 | 02:18:32 | yes | left | right | 0.1 | -0.3 | 2.5 | -0.3 | 0 % | -0.0 | 2.8 |
| 10 | 02:18:38 | yes | left | left | 0.3 | 3.0 | 6.5 | 2.9 | 0 % | -0.0 | 3.5 |
| 11 | 02:18:59 | yes | left | left | 2.6 | 46.7 | 49.9 | 46.2 | 0 % | -0.4 | 3.2 |
| 12 | 02:19:34 | yes | left | left | 0.6 | 8.0 | 10.9 | 7.9 | 0 % | -0.1 | 2.9 |
| 13 | 02:19:45 | yes | left | left | 2.0 | 44.1 | 43.7 | 44.2 | 29 % | 0.1 | -0.4 |
| 14 | 02:19:50 | yes | left | left | 1.9 | 25.7 | 28.3 | 25.8 | 0 % | 0.1 | 2.6 |
| 15 | 02:19:58 | yes | left | left | 6.3 | 88.8 | 90.6 | 88.4 | 0 % | -0.4 | 1.8 |
| 16 | 02:20:05 | yes | left | right | 0.2 | -0.3 | 2.4 | -0.4 | 0 % | -0.1 | 2.7 |
| 17 | 02:20:10 | yes | left | left | 2.3 | 50.4 | 55.3 | 51.7 | 0 % | 1.3 | 4.9 |
| 18 | 02:20:17 | yes | left | left | 1.5 | 17.0 | 20.3 | 16.9 | 0 % | -0.1 | 3.3 |
| 19 | 02:20:22 | yes | left | left | 2.4 | 60.1 | 64.2 | 59.4 | 0 % | -0.7 | 4.1 |
| 20 | 02:20:25 | yes | left | left | 1.3 | 21.3 | 24.9 | 21.8 | 0 % | 0.5 | 3.6 |
| 21 | 02:20:35 | yes | left | left | 4.3 | 72.0 | 71.4 | 72.1 | 14 % | 0.1 | -0.6 |
| 22 | 02:20:55 | yes | left | left | 0.7 | 8.4 | 12.4 | 9.2 | 0 % | 0.8 | 4.0 |
| 23 | 02:21:08 | yes | right | right | 6.1 | -8.1 | -11.3 | -7.4 | 0 % | 0.7 | -3.2 |
| 24 | 02:21:38 | yes | left | left | 3.5 | 85.2 | 71.0 | 86.0 | 34 % | 0.8 | -14.2 |
| 25 | 02:21:44 | yes | left | left | 4.0 | 83.0 | 109.8 | 83.6 | 12 % | 0.6 | 26.8 |
| 26 | 02:22:09 | yes | right | left | 0.2 | 0.4 | -2.7 | 0.3 | 0 % | -0.0 | -3.1 |
| 27 | 02:22:22 | yes | left | left | 4.0 | 0.7 | 4.3 | 0.6 | 0 % | -0.0 | 3.6 |
| 28 | 02:22:42 | yes | left | left | 3.6 | 68.3 | 73.1 | 68.8 | 0 % | 0.4 | 4.8 |
| 29 | 02:22:51 | yes | left | left | 1.0 | 4.2 | 5.7 | 3.7 | 0 % | -0.5 | 1.5 |
| 30 | 02:23:52 | yes | left | left | 3.0 | 88.3 | 78.4 | 87.6 | 22 % | -0.7 | -9.9 |
| 31 | 02:24:01 | yes | left | left | 2.4 | 79.6 | 83.6 | 80.5 | 0 % | 0.9 | 4.0 |
| 32 | 02:25:23 | yes | right | left | 13.1 | 13.4 | 6.9 | 13.6 | 0 % | 0.2 | -6.5 |
| 33 | 02:25:33 | **no** | right | left | 4.5 | 3.0 | - | 2.8 | 100 % | -0.2 | - |
| 34 | 02:26:00 | **no** | right | right | 2.6 | -48.6 | - | -48.3 | 100 % | 0.3 | - |
| 35 | 02:26:04 | **no** | right | right | 2.0 | -35.3 | - | -36.8 | 100 % | -1.5 | - |
| 36 | 02:26:05 | **no** | left | right | 0.1 | -0.2 | - | -0.2 | 100 % | 0.0 | - |
| 37 | 02:26:45 | **no** | left | left | 2.2 | 70.2 | - | 70.7 | 100 % | 0.5 | - |
| 38 | 02:26:48 | **no** | left | left | 0.8 | 17.4 | - | 18.3 | 100 % | 1.0 | - |
| 39 | 02:26:51 | **no** | left | left | 0.9 | 19.3 | - | 20.4 | 100 % | 1.1 | - |
| 40 | 02:26:56 | **no** | left | left | 2.0 | 19.0 | - | 20.2 | 100 % | 1.2 | - |
| 41 | 02:27:40 | **no** | left | left | 4.1 | 73.0 | - | 73.6 | 100 % | 0.7 | - |
| 42 | 02:28:18 | **no** | right | left | 3.3 | 5.8 | - | 5.7 | 100 % | -0.1 | - |
| 43 | 02:28:34 | **no** | right | right | 4.0 | -81.2 | - | -81.1 | 100 % | 0.1 | - |
| 44 | 02:28:54 | **no** | left | left | 3.1 | 59.7 | - | 60.6 | 100 % | 0.9 | - |
| 45 | 02:29:07 | **no** | right | right | 2.9 | -54.3 | - | -55.2 | 100 % | -0.9 | - |
| 46 | 02:29:24 | **no** | right | left | 14.6 | 4.4 | - | 4.0 | 100 % | -0.4 | - |
| 47 | 02:29:28 | **no** | left | left | 3.0 | 73.4 | - | 73.9 | 100 % | 0.5 | - |
| 48 | 02:29:32 | **no** | left | left | 0.4 | 1.6 | - | 1.6 | 100 % | 0.1 | - |
| 49 | 02:29:43 | **no** | left | left | 9.3 | 76.7 | - | 86.1 | 100 % | 9.4 | - |
| 50 | 02:30:25 | **no** | left | left | 3.5 | 90.3 | - | 91.1 | 100 % | 0.7 | - |
| 51 | 02:30:28 | **no** | left | left | 0.5 | 9.3 | - | 9.4 | 100 % | 0.1 | - |
| 52 | 02:30:51 | **no** | left | left | 16.7 | 52.7 | - | 53.3 | 100 % | 0.6 | - |
| 53 | 02:31:38 | **no** | right | right | 11.5 | -169.1 | - | -170.2 | 100 % | -1.1 | - |
| 54 | 02:31:45 | **no** | left | right | 4.7 | -5.5 | - | -5.4 | 100 % | 0.1 | - |
| 55 | 02:32:24 | **no** | right | left | 30.9 | 0.2 | - | -0.6 | 100 % | -0.8 | - |
| 56 | 02:32:32 | **no** | right | right | 4.1 | -78.7 | - | -80.3 | 100 % | -1.7 | - |
| 57 | 02:32:37 | **no** | right | right | 2.5 | -36.8 | - | -38.5 | 100 % | -1.7 | - |
| 58 | 02:32:41 | **no** | right | right | 1.3 | -5.3 | - | -5.5 | 100 % | -0.2 | - |
| 59 | 02:32:47 | **no** | right | right | 0.8 | -8.5 | - | -9.8 | 100 % | -1.2 | - |
| 60 | 02:33:08 | **no** | right | left | 4.8 | 0.3 | - | 0.5 | 100 % | 0.2 | - |
| 61 | 02:33:11 | **no** | right | right | 0.6 | -12.9 | - | -14.2 | 100 % | -1.3 | - |
| 62 | 02:33:16 | **no** | right | right | 2.5 | -35.2 | - | -35.3 | 100 % | -0.0 | - |
| 63 | 02:33:24 | **no** | right | right | 4.9 | -14.5 | - | -14.9 | 100 % | -0.4 | - |
| 64 | 02:33:39 | **no** | right | left | 10.8 | 1.2 | - | 0.5 | 100 % | -0.7 | - |
| 65 | 02:34:05 | **no** | right | right | 7.0 | -85.3 | - | -86.0 | 100 % | -0.7 | - |
| 66 | 02:34:11 | **no** | right | left | 4.6 | 10.2 | - | 9.6 | 100 % | -0.6 | - |

## 3. The WiFi link to the robot (the bridge)

*Plain terms: the robot's wheel and gyroscope readings travel to the Jetson over WiFi. Source: `~/.run_records/s2_static_05/bridge_recv.log` (Jetson), its FINAL counters; numbers.json. The link is independent of the camera, so this covers the whole drive.*

- **Link down: 2 times** (02:29:59 STATE DOWN no bytes for 1.5 s; 02:30:32 STATE DOWN no bytes for 1.5 s; back up: 02:30:02 STATE UP RELOCK_QUICK; 02:30:33 STATE UP RELOCK_QUICK), both after the camera crash. **Connections: 3 in 1 receiver session(s)**, the session 1482.9 s long. *(bridge_recv.log's own clock is Hamilton time; numbers.json's `down_lines_hamilton` shifts these by 4 h because analyze_run.py assumes the log is in UTC - use the log's times.)*
- "Stale" = arrived too late to use and was not published (the receiver's own age limit). "Robot read" minus "robot sent" is readings taken while no receiver was connected ([INFERENCE] from `robot_bridge_send.py` `on_msg`, as in drive 4) - not checked against the robot's own log: the robot is off, charging.

| stream (session 1) | robot read | robot sent | published on the Jetson | % of sent | % of what the robot read | missed while down | too late (stale) | lost |
|---|---|---|---|---|---|---|---|---|
| /husky_velocity_controller/odom | 14845 | 14748 | 14654 | 99.36 | 98.71 | 71 | 85 | 0 |
| /imu/data | 28990 | 28803 | 28614 | 99.34 | 98.70 | 139 | 166 | 0 |
| /imu/data_raw | 28990 | 28803 | 28614 | 99.34 | 98.70 | 139 | 166 | 0 |
| /odometry/filtered | 74255 | 73772 | 73298 | 99.36 | 98.71 | 356 | 423 | 0 |

Drive 4 for scale: 0 drops, 1 connection, nothing lost (drive 4 RESULTS.md section 3).

**Parked-drop guard** (`~/.run_records/s2_static_05/fused_odometry.log`): it fires when wheels and gyroscope go quiet for 0.3 s while the robot is parked:

- **event 1**, 02:21:20 (before the crash):
    - 02:21:20.7 DROP GUARD: wheels and gyroscope silent for >= 0.3 s while PARKED - publishing "not moving" and keeping camera frames out until the camera shows real motion or the robot link is back
    - 02:21:21.5 DROP GUARD ended after 0.82 s: the gyroscope came back (held throughout)
- **event 2**, 02:22:35 (before the crash):
    - 02:22:35.9 DROP GUARD: wheels and gyroscope silent for >= 0.3 s while PARKED - publishing "not moving" and keeping camera frames out until the camera shows real motion or the robot link is back
    - 02:22:36.7 DROP GUARD ended after 0.80 s: the gyroscope came back (held throughout)
- **event 3**, 02:23:08 (before the crash):
    - 02:23:08.7 DROP GUARD: wheels and gyroscope silent for >= 0.3 s while PARKED - publishing "not moving" and keeping camera frames out until the camera shows real motion or the robot link is back
    - 02:23:09.5 DROP GUARD ended after 0.79 s: the wheels came back (held throughout)

The blend's input counters at the end of the drive (`~/jobs/s2_static_05_ekf_inputs.progress`): `EKF_INPUTS wheel 10.0Hz imu 19.3Hz vo 0.0Hz vo_dropped 0.0% bias -0.14deg/min(n=565,measuring) kept -0.14deg/min(n=565) still 211s hold 12.2s guard 25/36/0 refused_imu 0 nonfinite 0 offset_refused 0 dropped 1 zed 0.0Hz zoff +0.02deg/min(n=2513) zed_notf 0 hold_yaw_to_zed 25  1470s` - "zed 0.0Hz" is the camera gyroscope, dead since the crash.

## 4. Camera tracking: losses, restarts, freezes (camera-alive part only)

- **Lost** (fusion.bag, `/rtabmap/odom`: RTAB-Map's lost signal = a null pose (all zeros) or its own uncertainty figure (covariance) of 9999): **9 lost stretches, 13.3 s in total, longest 4.6 s, 1.6 % of messages** (numbers.json; the tracker sent nothing after the crash, so these are all before 02:25:26).
- **Restarts from TF** (`mapping.log`, "Odometry automatically reset to latest odometry pose available from TF" - the camera's tracking gave up and restarted from the blend's position): **12**, in 5 bursts: 02:17:28-02:17:32 (5); 02:19:45 (1); 02:20:32 (1); 02:21:37-02:21:39 (3); 02:23:47-02:23:49 (2). The mapping monitor's last line says 12 (`monitor_STATUS.txt`).

**Freezes, with confirmed saves off (the first drive with `DbSqlite3/Synchronous=0`).** A freeze = the camera program stops producing pictures while the map saves to disk (drive 3's fault; `03_methods/jetson_stalls_drive3_2026-09-25/FINDINGS.md`). **Pass line for this change, written before the drive: "0 camera freezes > 1 s (pass_check.py), database PASS"** (`~/slam_series2/tools/DEPLOYED_2026-09-26_sync0_drive5.txt` line 9, Jetson, 01:40). pass_check.py's own thresholds are 1.5 s and 2.0 s, so the 1 s count is made here from the same timing file and both are shown. Window: the camera-alive part only, 02:13:45 to 02:25:25 (11.7 min) - the pass check takes its window from the camera gyroscope, which stopped with the crash, so nothing after 02:25:26 is counted. Output: `freezes_pass_check.json`.

| check | drive 5 (11.7 min, confirmed saves off) | where | drive 4 (10 min, confirmed saves on) |
|---|---|---|---|
| **camera tracker silences > 1 s** (`/rtabmap/odom` header stamps) - the pass line | **2** (1.80 s, 1.60 s) | from 02:14:09, from 02:15:09 | 1 (2.93 s) - counted at > 1.5 s, drive 4's own threshold: drive 4's recording was no longer on the Jetson, so it was not recounted at 1 s |
| camera picture-loop pauses > 2 s ('ZED Diagnostic', sent only from inside the camera program's picture loop, normally once every 1.02 s - so a 1 s threshold cannot be used on it; pass_check.py P4) | **1** (2.77 s) | from 02:14:08 | 1 (3.8 s) |
| tracker silences > 1.5 s (pass_check.py P1) | 2 | - | 1 |
| **longest database save** (`mapping.log`, "Maps update") | **0.86 s** at 02:23:00 | - | 4.45 s |
| database saves >= 1.0 s / >= 1.5 s / >= 5.5 s | 0 / 0 / 0 | - | not counted / 8 / 0 (drive 4 RESULTS.md section 4) |
| control: camera gyroscope longest gap (must be < 0.1 s for the check to count) | 0.028 s | - | - |
| robot-data silences with the "inbound held" signature (P2) | 0 | - | 0 |
| log coverage (map updates parsed / nodes) | 667 / 688 = 96.9 % | - | - |

**Verdict against the pass line: freezes FAIL; database PASS** (`db_check.txt`: `PASS ~/slam_series2/s2_static_05.db: quick_check=['ok'], 688 map nodes, 0.6 s` - the main session's run at 02:37, repeated read-only for this pack; `db_check.py` = the database's own integrity check, SQLite `PRAGMA quick_check`). **Overall: FAIL.** pass_check.py's own verdict: **FAIL**.
 Drive 4 figures are from drive 4 RESULTS.md section 4. N = 1; the change did not remove the freezes on this drive.

**Where the freezes fell:** tracker silent 1.80 s from 02:14:09; tracker silent 1.60 s from 02:15:09. The largest database save within 5 s of each: 0.01 s, 0.01 s - all well under 1 s, and the longest save of the whole camera-alive part was 0.86 s (drive 4: 4.45 s). [INFERENCE] So confirmed saves off did what it was meant to do to the saves, but these freezes are not the drive-3 save-freeze: something else paused the camera pipeline, both times in the first 1.5 min of the drive (the first before "ready" at 02:14:12). tegrastats also skipped 3 s at 02:14:12 (power_temperature.json), which overlaps the first. Cause not established.

## 5. Blend heading while standing still (reported, not a pass line)

*Whole drive; after the crash the blend had no camera.*

| | drive 5 (numbers.json) | drive 4 (its RESULTS.md section 5) |
|---|---|---|
| stops | 129 (704 s still) | 39 (276 s still) |
| signed sum | **-0.909 deg** | +0.085 deg |
| total (sum of each stop's net change) | 2.823 deg | 0.663 deg |
| largest single stop | -0.196 deg | -0.048 deg |

Rule: robot wheel odometry speed and turn rate both zero, below 1e-6 - the log holds 1e-16 float noise, not 0.0 (nearest wheel message within 0.3 s).

## 6. Processor, power and temperature

*Source: `~/rehearsal_t5/tegrastats_drive5.log` (Jetson; tegrastats = NVIDIA's once-a-second load/power/temperature log, clock in UTC), read by `tegra_peaks.py`. Two windows: camera alive (02:13:59-02:25:26, power_temperature_camera_alive.json) and the whole drive (02:13:59-02:37:55, power_temperature.json). Total power = the four named supply rails.*

| quantity | camera alive: peak (when) | camera alive: median | whole drive: median | drive 4 peak |
|---|---|---|---|---|
| total power | **32.1 W** (02:19:36) | 29.4 W | 9.6 W | 36.6 W |
| hottest point on the chip (tj) | **61.7 C** (02:21:20) | - | - | 61.6 C |
| graphics processor temperature | 55.8 C (02:21:22) | - | - | - |
| graphics processor load | 98 % (02:20:10) | 22 % | 0 % | - |
| processor load, mean of the 12 cores | 48.7 % (02:14:21) | 41.2 % | 7.7 % | - |
| memory free, lowest | 55.6 % (13.3 GB used, 02:24:38) | - | - | 66.2 % |

- The whole-drive medians fall to near idle because the camera, its depth processing and the map stopped at 02:25:26 - the load did not cause the crash as far as these figures show: no peak falls near 02:25, and memory was never below 55.6 % free.
- No reset of the Jetson (tegrastats ran through: 1392 samples; skips over 2 s: 02:14:12 (3 s)).
- RTAB-Map, time to process each map snapshot (database Statistics table, read-only): **median 481.4 ms, 95th percentile 642.3 ms, largest 1818.2 ms over 688 nodes** (drive 4's are in its RESULTS.md section 6).

## 7. Agreement with the LiDAR estimate (camera-alive part only)

*The reference is the robot's LiDAR map (a colleague's work: self_navigation `rtabmap_3d.launch`, replayed on the robot after the drive, parked): the robot's wheels + gyroscope corrected by LiDAR loop closures. **An independent estimate, not ground truth** (ENGINEERING_NOTES.md rule 19): LiDAR range noise 15-30 mm, the two LiDARs agree to about 5 mm, and the LiDAR-to-camera mounting offset has never been measured. It shares the robot's wheels + gyroscope with the blend, so for the blend it is not fully independent. Made by `03_methods/results_packs_2026-09-26/compare_lidar.py`.*

- LiDAR map (whole drive, the camera crash does not affect it): 714 positions, 200.67 m, **69 LiDAR loop closures**. Its own end-to-start gap: 0.6295 m in 3D (`lidar.tum.meta.json`, from Admin.opt_poses) and 0.34 m on the floor plane (`lidar_map_facts.json`).
- **Only the camera-alive part is compared**: camera.tum and camera_corrected.tum end at node 688 (02:25:25), so every matched moment is before the crash. The LiDAR map's 138.9 m after the crash have nothing to compare against.

| path compared | kind (rule 20) | matched | median | 95th percentile | largest | best-fit scale | file |
|---|---|---|---|---|---|---|---|
| camera map, corrected (camera_corrected.tum) | map's corrected path | 100.0 % (280) | **0.244 m** | **0.588 m** | 0.665 m | 1.0556 | agreement.json |
| map nodes (= blend), tracking alone (camera.tum) | tracking alone | 100.0 % (688) | **0.123 m** | **0.471 m** | 0.491 m | 1.0294 | agreement.json |

- **Best-fit scale** = the size change that would line the path up best with the LiDAR estimate if scaling were allowed; it is NOT applied (stereo has real metric scale; ENGINEERING_NOTES.md section 4 rule 4). Corrected camera map +5.6 %, the blend's node path +2.9 % (drive 4: +6.3 % / +2.9 %).
- **Clock shift:** the comparison's own check found the paths fit best with the camera shifted +0.08 s (fit 0.321 m, against 0.322 m with no shift); **no shift was applied** (applied: 0.0 s) - compare_lidar.py applies one only when it improves the fit by more than 0.05 m and 20 % (`~/results_packs_work/s2_static_05/compare_lidar.out`).

*Plain terms: at each moment, how far apart the camera map's position and the LiDAR estimate are, after lining the two paths up once (rotation and shift, no scaling).* Drive 4 for scale: corrected median 0.26 m, 95th percentile 0.49 m; map nodes tracking alone 0.12 / 0.24 m (drive 4 agreement.json). Drive 5 compares 11 min against drive 4's 10, so the lengths are similar; N = 1 each.

Figures: `lidar_comparison.png` (LiDAR solid, camera map dashed, map nodes dotted; tracking alone and corrected on separate panels); `lidar_map.png` - the LiDAR floor map of the whole drive.

## 8. The staged person (LiDAR clearing test)

A person stood 1.5 m behind the robot at the start for 30 s, then left (`staged_person.txt`), to test whether the LiDAR clearing ("carving": erasing wall squares that later laser beams pass straight through) removes someone who walked away, without eating real walls. Scored once on the robot against rules registered before the drive (`03_methods/lidar_carving_2026-09-25/RESULTS.md`, last section, "Drive 5 score"): **N1 FAIL** - real walls kept a solid core on 96.1 % of check lines, the line was 98 %; **N4 FAIL** - 7.4 % of the person's squares remained (5 of 68), the line was 5 %; N2 PASS; **N3 UNCHECKED** - 15 spots where a wall band vanished still need to be looked up in the camera pictures, and only spots passed before the camera crash can be. Live carving stays off. Figure: `lidar_carving_score.png` (a copy of `robot_test_round2/drive5_score/carving_v2.png`).

## 9. Figures and video in this folder

| file | what it shows |
|---|---|
| map.png | camera floor map, map nodes at their tracking-alone positions (Node.pose = the blend's), up to the crash |
| map_corrected.png | camera floor map after the loop closures (Admin.opt_poses), up to the crash |
| trajectory.png | where each source put the robot, two panels (rule 20): left = tracking alone (robot's wheels + gyroscope solid; camera tracking and blend dashed - the camera line stops at the crash; a cross on each path marks where it was at the crash), right = corrected (camera map dashed, to the crash; LiDAR estimate solid, whole drive) |
| turns_difference.png | per turn: camera minus robot AND blend minus robot, in degrees; a vertical line between turns 32 and 33 marks the crash, and later turns are boxed "camera dead" |
| lidar_comparison.png | the camera paths over the LiDAR map (map nodes dotted, drawn thicker for drive 5 so they show), and the distance to the LiDAR estimate along the route (camera-alive part), tracking alone and corrected on separate panels |
| lidar_map.png | the LiDAR's own floor map, whole drive |
| lidar_carving_score.png | the LiDAR clearing test's score (section 8) |
| timelapse.mp4 | **recorded live during the drive** (not rebuilt): the live map page (camera map left, LiDAR view right) saved every 3 s, 480 frames at 10 per second = 48 s of video for 24 min; from frame 231 on the camera side says "NO DATA" (the crash) while the LiDAR side keeps going (`timelapse_live_index.csv`; `media/recorder.log`: missed 0, write errors 0) |
| timelapse_crash.png | frame 229 (02:25:23), the last frame saved before the crash |
| timelapse_25/50/75/100.png | stills at 25/50/75/100 % of the timelapse (50 % onwards are after the crash) |

Other files: camera.tum / camera_corrected.tum (+ .meta.json); paths_2hz.csv; turns.csv, turns_replayed.log; numbers.json; crash_split.json; freezes_pass_check.json; power_temperature*.json; closed_check.txt; lidar.tum, wheel.tum, lidar_map.npz, provenance.txt (LiDAR products, made on the robot).

## 10. Still open

- The camera crash's root cause (second exit -11 in one night; the first followed recording start/stops). Fix in place: `03_methods/camera_crash_recovery_2026-09-26/`.
- The camera map covers only the first 61.9 m of 200.8 m; drive 5 cannot give a camera end-to-start gap.
- Freezes with confirmed saves off: one drive, 11.7 camera minutes. More drives are needed before a rate can be quoted.
- N = 1 at these settings: no spread can be quoted (ENGINEERING_NOTES.md section 4).
