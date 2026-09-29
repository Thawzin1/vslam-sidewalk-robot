# Drive 4 (s2_static_04) - results pack

*26 Sept 2026, Hamilton time: ready about 00:43, driving 00:44-00:53, "park" 00:53:37, map closed properly 00:54:32.
The second fused drive (fused = the camera blended with the robot's wheels and gyroscope (turn-rate sensor) by an EKF
(extended Kalman filter, a standard way of mixing sensors)). Made on the Jetson on 26 Sept by
`results_packs_2026-09-26/` (project records) (pack_drive4_heavy.sh, analyze_run.py, turns_replay.py,
extract_timing.py, tegra_peaks.py, this file's writer `write_results_drive4.py`). Every number names the file it comes
from; files are in this folder unless a path is given. Nothing here is ground truth (ENGINEERING_NOTES.md rule 19). N = 1.*

**What was different from drive 3** (each checked in the run's own files):

| change | plain meaning | evidence |
|---|---|---|
| 16-bit depth saved in the map | depth pictures stored at half the size, to shorten database saves | `Mem/SaveDepth16Format:true` in the database's Info table |
| new robot-link receiver + parked-drop guard | if the robot's data stops while parked, the blend holds still instead of following the camera | `bridge_recv.log`; 2 drop-guard events in `fused_odometry.log` (section 3) |
| the camera's own gyroscope in the blend | a second turn-rate sensor, inside the camera | `~/jobs/s2_static_04_ekf_inputs.progress`: zed 44.5 Hz, zed_notf 0 (frame found on /tf_static) |
| no camera recording (no SVO file - SVO is the ZED camera's own video-recording format) | in rehearsal T5c the camera program logged "Error saving frame to SVO" at 00:27:52 and crashed (segfault, exit -11) at 00:37:20, after its recording had been started and stopped 4 times in one camera session; separately, the Jetson hard-reset twice that night (21:44 and 23:39, reset reason SYS_RESET_N, cause unknown) | `docs/DO_NOT_REPEAT.md`, "Night of 25-26 Sept (Hamilton): camera recording, receiver restart, unexplained Jetson resets"; `media/` holds only the live-map timelapse |
| camera program freshly started | - | `camera.log` (00:43 Hamilton) |

## 1. How far from home at the end - both kinds (rule 20)

*Plain terms: "tracking alone" = where each source's own step-by-step tracking (odometry) put the robot, never
corrected; "corrected" = where the map puts it after its loop closures (the map recognising a place it has seen
before and pulling itself straight). A "node" is one saved map snapshot; the "graph" is the nodes and the links
between them. Admin.opt_poses = the corrected node positions RTAB-Map saves in its database when it shuts down properly;
Node.pose = each node's tracking-alone position. The drive was planned to end on its start mark, so a perfect estimate shows about 0; the real end
position was not measured with a tape.*

**Read this first - in a fused drive the map's nodes carry the BLEND's position, not the camera's.** The map program
reads odom->base_link (where the robot's body is in the step-by-step tracking frame) from TF (ROS's shared table of
where each part of the robot is; it subscribes to no odometry topic when odom_frame_id is set;
`catkin_ws/src/sidewalk_slam/config/ekf_fused.yaml` lines 35-37), and that TF comes from the blend (`/ekf_fused`,
`~/.run_records/s2_static_04/tf_check_2.txt`). And each time the camera's tracking restarted (section 4), it restarted from
that same TF position, i.e. from the blend. So "camera, tracking alone" below is the camera's tracking between
restarts, re-seeded by the blend 26 times - not an independent camera-only path.

| kind | source | end-to-start gap | heading gap | path length | file |
|---|---|---|---|---|---|
| tracking alone | map nodes (Node.pose = the blend's position at each node), node 1 to node 540 | **0.057 m** | - | 87.17 m | facts.json, camera.tum |
| tracking alone | camera tracking, every message (/rtabmap/odom, lost stretches left out; re-seeded by the blend at each restart) | 0.250 m | +2.51 deg | 99.4 m | numbers.json (fusion.bag) |
| tracking alone | blend (/fused/odometry) | 0.064 m | -1.08 deg | 93.2 m | numbers.json (fusion.bag) |
| tracking alone | robot's own wheels + gyroscope (/robot/ekf_odom) | 0.266 m | +2.31 deg | 90.1 m | numbers.json (fusion.bag) |
| **corrected** | map's saved graph, Admin.opt_poses, node 1 to node 503 (284 graph nodes) | **0.0478 m** | - | - | camera_corrected.tum.meta.json |
| corrected | every node placed by its graph node's correction, node 1 to node 540 | 0.080 m | - | 85.11 m | map_corrected_facts.json |

**Loop closures beside the corrected gap: 181** (14 recognised-again + 167 nearby re-matches; counted one way in the database's Link table, types 1 and 2; `closed_check.txt` gives the same total). **3 of them join a later node straight to node 1 (the start)**: node 197 -> node 1 at 00:47:46, node 478 -> node 1 at 00:53:11, node 483 -> node 1 at 00:53:20 - all 3 are nearby re-matches (type 2), none recognised-again (type 1). So the corrected gap partly repeats those closures rather than measuring drift independently.
The corrected gap of 0.0478 m is the straight-line distance in 3D, height included (db_corrected_tum.py); on the floor plane alone it is 0.0478 m (camera_corrected.tum, first and last line) - the same, because the corrected path has no height change between its first and last node.
The live mapping monitor counted 128 accepted closures at its last line (t = 627 s, `monitor_STATUS.txt`); the database, read after the map closed, holds 181. The database is the complete count.

The saved graph ends at node 503; the 37 nodes after it were the robot parked: over them the map's tracking-alone position moved net 0.033 m and was never more than 0.033 m from node 503 (camera.tum).

**On tracking alone the blend ended 0.064 m / -1.08 deg from its start; the robot's own wheels + gyroscope 0.266 m / +2.31 deg.** Drive 3 was 5.597 m / -44.8 deg for the blend against 0.280 m / +1.3 deg for the robot (drive 3 RESULTS.md section 1). Drive 4 was shorter (90 m by the robot against 145 m) and had no link drop, so this is not a like-for-like comparison; it is reported only.

*Drive 3's pass line applied the same way (NEXT_PLAN_2026-09-25.md item 3, "after its own corrections the map comes home within 0.3 m of its start mark"; drive 4 has no gap pass line of its own in DRIVE4_PLAN_2026-09-25.md): 0.0478 m - within it, with the caveat above.*

## 2. Every turn: robot vs camera vs blend

*Plain terms: for each turn, how many degrees each source says the robot turned. The robot's own wheels + gyroscope is the comparison for short turns (not ground truth).*

**The live turn watcher did not run during drive 4** (no turns.log in `~/.run_records/s2_static_04/`). Its rule was replayed after the drive over the drive's recording (fusion.bag) by `turns_replay.py`, unchanged: a turn = the wheels' turn rate above 0.10 rad/s for 0.3 s, ending after 1.0 s below 0.05 rad/s. *Control:* the same replay over drive 3's recording found **52 turns, as the live watcher did**; per turn, the robot's heading agreed with the live log to median 0.0 deg (largest 2.0) and blend minus robot to median 0.15 deg (largest 1.9) - `03_methods/results_packs_2026-09-26/control_drive3_turns_replayed.log`. Output: `turns_replayed.log` -> turns.csv.

*95th percentile = the value 95 % of turns stay under.*

| summary (16 turns) | median | 95th percentile | largest | within 1 deg |
|---|---|---|---|---|
| abs(blend - robot) | 0.20 deg | 0.43 deg | 0.5 deg | 16 of 16 |
| abs(camera - robot) | 2.30 deg | 19.10 deg | 21.8 deg | 3 of 16 |

Drive 3 for scale (its RESULTS.md section 2, 52 turns): blend within 1 deg on 41 of 52, largest 25.2 deg (the link drop).
Figure: `turns_difference.png`.

*"started turning" = the side the wheels were turning when the turn began (the watcher's rule reads the turn rate's sign at the start). It is not always the net direction: turn 8 started right, net left (robot 99.7 deg); turn 11 started left, net right (robot -1.8 deg); turn 15 started left, net right (robot -10.9 deg). "net" = the sign of the robot's own heading change over the turn (left = positive degrees).*

| turn | ended | started turning | net | s | robot deg | camera deg | blend deg | camera lost | blend - robot | camera - robot |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 00:46:22 | right | right | 6.3 | -86.2 | -73.8 | -86.2 | 26 % | 0.0 | 12.4 |
| 2 | 00:46:39 | right | right | 4.5 | -83.8 | -82.7 | -84.2 | 19 % | -0.3 | 1.1 |
| 3 | 00:46:48 | left | left | 2.5 | 17.3 | 18.3 | 17.2 | 0 % | -0.1 | 1.0 |
| 4 | 00:46:53 | right | right | 0.5 | -4.5 | -7.0 | -4.2 | 0 % | 0.3 | -2.5 |
| 5 | 00:46:56 | right | right | 0.5 | -1.0 | -3.1 | -1.4 | 0 % | -0.4 | -2.1 |
| 6 | 00:47:38 | left | left | 6.8 | 89.2 | 91.7 | 88.8 | 0 % | -0.4 | 2.5 |
| 7 | 00:47:49 | left | left | 6.0 | 87.5 | 89.5 | 87.6 | 0 % | 0.1 | 2.0 |
| 8 | 00:48:26 | right | left | 17.7 | 99.7 | 77.9 | 99.9 | 10 % | 0.2 | -21.8 |
| 9 | 00:48:36 | left | left | 3.2 | 41.0 | 59.2 | 41.1 | 84 % | 0.1 | 18.2 |
| 10 | 00:48:42 | left | left | 2.3 | 28.7 | 30.3 | 28.8 | 0 % | 0.0 | 1.6 |
| 11 | 00:49:13 | left | right | 13.0 | -1.8 | 1.5 | -2.1 | 0 % | -0.2 | 3.3 |
| 12 | 00:49:25 | right | right | 6.2 | -87.6 | -89.0 | -87.9 | 0 % | -0.2 | -1.4 |
| 13 | 00:50:34 | right | right | 2.9 | -77.6 | -78.3 | -78.1 | 20 % | -0.5 | -0.7 |
| 14 | 00:52:45 | right | right | 22.4 | -10.6 | -11.5 | -10.5 | 0 % | 0.1 | -0.9 |
| 15 | 00:52:54 | left | right | 3.3 | -10.9 | -6.5 | -10.6 | 0 % | 0.3 | 4.4 |
| 16 | 00:53:01 | left | left | 0.2 | 1.3 | 7.2 | 0.9 | 0 % | -0.4 | 5.9 |

## 3. The WiFi link to the robot (the bridge)

*Plain terms: the robot's wheel and gyroscope readings travel to the Jetson over WiFi. Source: `~/.run_records/s2_static_04/bridge_recv.log` (Jetson), its FINAL counters; numbers.json.*

- **Link down: 0 times. Reconnects: 1 connection in one receiver session of 665.8 s.** Nothing lost in transit (lost = 0 on every stream); the sender's own drop counter is 0 on every stream.
- **The robot read more than it sent**: /husky_velocity_controller/odom 53 (5.3 s at 10.1 per second); /imu/data 101 (5.2 s at 19.5 per second); /imu/data_raw 101 (5.2 s at 19.5 per second); /odometry/filtered 262 (5.2 s at 50.3 per second). [INFERENCE] `tools/robot_side/robot_bridge_send.py` (`on_msg`, lines 190-203) counts every reading as read, but queues it for sending only while a receiver is connected ("nobody to send to" - the reading is neither sent nor counted as dropped). The shortfall is the same length of time on every stream, about 5.2 s, which fits readings taken while no receiver was connected (before the Jetson connected at 00:43:26 or after the last send). Not checked against the sender's own log: the robot did not answer ssh (about 01:20 Hamilton).
- "Stale" = arrived too late to use and was not published (the receiver's own age limit).

| stream | robot read | robot sent | published on the Jetson | % of sent | % of what the robot read | missed while down | too late (stale) |
|---|---|---|---|---|---|---|---|
| /husky_velocity_controller/odom | 6700 | 6647 | 6588 | 99.11 | 98.33 | 0 | 56 |
| /imu/data | 12966 | 12865 | 12749 | 99.10 | 98.33 | 0 | 110 |
| /imu/data_raw | 12966 | 12865 | 12749 | 99.10 | 98.33 | 0 | 110 |
| /odometry/filtered | 33505 | 33243 | 32946 | 99.11 | 98.33 | 0 | 282 |

Drive 3 for scale: 1 drop of about 39 s; 97.9 % of sent published; 350 stale on the wheel stream (drive 3 RESULTS.md section 3).

**Parked-drop guard** (`~/.run_records/s2_static_04/fused_odometry.log`): it fired when wheels and gyroscope went quiet for 0.3 s while the robot was parked:

- **event 1**, 00:49:54:
    - 00:49:54.6 DROP GUARD: wheels and gyroscope silent for >= 0.3 s while PARKED - publishing "not moving" and keeping camera frames out until the camera shows real motion or the robot link is back
    - 00:49:55.4 DROP GUARD ended after 0.76 s: the gyroscope came back (held throughout)
- **event 2**, 00:53:22:
    - 00:53:22.9 DROP GUARD: wheels and gyroscope silent for >= 0.3 s while PARKED - publishing "not moving" and keeping camera frames out until the camera shows real motion or the robot link is back
    - 00:53:23.2 DROP GUARD released after 0.27 s: the camera shows real motion (median 0.135 m/s, -0.005 rad/s over 4 frames) - the camera leads until the robot link is back
    - 00:53:23.7 DROP GUARD ended after 0.72 s: the wheels came back (released earlier)

*Plain terms: twice the robot's data paused for under a second while parked; the blend held still, and the second time let the camera lead once the camera showed real motion.*

## 4. Camera tracking: losses, restarts, freezes

- **Lost** (fusion.bag, `/rtabmap/odom`: RTAB-Map's lost signal = a null pose (all zeros) or its own uncertainty figure (covariance) of 9999): **11 lost stretches, 26.2 s in total, longest 10.5 s, 3.8 % of messages** (numbers.json).
- **Restarts from TF** (`mapping.log`, "Odometry automatically reset to latest odometry pose available from TF" - the camera's tracking gave up and restarted from the blend's position): **26**, in 3 bursts: 00:46:21-00:46:35 (11); 00:48:25-00:48:36 (14); 00:50:31 (1). The mapping monitor's reset count agrees (26, `monitor_STATUS.txt`).
- Turns with the camera lost for at least half the turn: turn 9 (turns.csv); blend minus robot on them: 0.1 deg.

**Freezes** (drive 3's fault: the camera program stops producing pictures while the map saves to disk; `03_methods/jetson_stalls_drive3_2026-09-25/FINDINGS.md`). Checked with that work's own pass check (`pass_check.py`, round 2) on this drive's recording -> `freezes_pass_check.json`:

| check | drive 4 | where | drive 3 (28 min) | T5b (16.5 min, 32-bit depth) | T5c (9.7 min, 16-bit depth) |
|---|---|---|---|---|---|
| camera tracker silences > 1.5 s (`/rtabmap/odom` header stamps) | **1** (2.93 s) | from 00:46:58 | 9 | 10 | 2 |
| 'ZED Diagnostic' pauses > 2 s (the camera program's own picture loop) | **1** (3.80 s) | from 00:46:58 | 9 | 5 | - |
| database saves >= 5.5 s | **0** | - | 11 | 7 | - |
| database saves >= 1.5 s | 8; longest 4.45 s at 00:46:57 | - | 176 | - | - |
| control: camera gyroscope longest gap (must be < 0.1 s for the check to count) | 0.036 s | - | 0.031 s | 0.043 s | - |
| robot-data silences with the "inbound held" signature | 0 | - | 9 | 0 | - |
| log coverage (map updates parsed / nodes) | 517 / 540 = 95.7 % | - | 96.5 % | 96.8 % | - |

Verdict as the pass check prints it: **FAIL** (its pass line is zero silences). Rate: 1 freeze in 10.1 min of recording (drive 3: 9 in 28 min; T5b: 10 in 16.5 min; T5c: 2 in 9.7 min - earlier figures from the main session and T5b_SCORE.md). The one freeze starts during the longest database save (4.45 s, 00:46:57) - the same pattern as drive 3 - and tegrastats also skipped 3 s at 00:47:02 (power_temperature.json). With N = 1 per setting and so few events, no rate difference between 16-bit and 32-bit depth can be claimed.

## 5. Blend heading while standing still (reported, not a pass line)

| | drive 4 (numbers.json) | drive 3 (its RESULTS.md section 5) |
|---|---|---|
| stops | 39 (276 s still) | 167 (962 s still) |
| signed sum | **+0.085 deg** | +0.531 deg |
| total (sum of each stop's net change) | 0.663 deg | 6.117 deg |
| largest single stop | -0.048 deg | +0.628 deg |

Rule: robot wheel odometry speed and turn rate both zero, below 1e-6 - the log holds 1e-16 float noise, not 0.0 (nearest wheel message within 0.3 s).

## 6. Processor, power and temperature

*Source: `~/rehearsal_t5/tegrastats_drive4.log` (Jetson; tegrastats = NVIDIA's once-a-second load/power/temperature log, clock in UTC), window 00:43:26 to 00:54:32 Hamilton, read by `tegra_peaks.py` -> power_temperature.json. Total power = the four named supply rails; the same code reproduces T5b's score (35.4 W at 23:51:02).*

| quantity | peak | when | median | T5b (for scale) |
|---|---|---|---|---|
| total power | **36.6 W** | 00:44:21 | 29.8 W | 35.4 W peak, 29.4 W median |
| hottest point on the chip (tj) | **61.6 C** | 00:53:41 | - | 62.8 C |
| graphics processor temperature | 55.7 C | 00:53:34 | - | 55.7 C |
| graphics processor load | 99 % | 00:53:28 | 23 % | 97 % peak |
| processor load, mean of the 12 cores | 99.9 % | 00:44:03 | 41.2 % | - |
| memory free, lowest | 66.2 % (10.1 GB used) | 00:54:21 | - | 69.4 % |

- No reset of the Jetson during the drive (tegrastats ran through; 645 samples in 666 s, one 3 s skip at the freeze).
- The 12-core peak at 00:44:03 is in the first minute, as the programs started and the drive began.
- RTAB-Map, time to process each map snapshot (database Statistics table, read-only): **median 510.2 ms, 95th percentile 787.8 ms, largest 1852.7 ms over 540 nodes** (drive 3: 667.6 / 922.4 / 3353.3 ms). The fusion programs' own processor share was not sampled this drive (no cpu_jetson.json).

## 7. Agreement with the LiDAR estimate

*The reference is the robot's LiDAR map (a colleague's work: self_navigation `rtabmap_3d.launch`, replayed on the robot after the drive, parked): the robot's wheels + gyroscope corrected by LiDAR loop closures. **An independent estimate, not ground truth** (ENGINEERING_NOTES.md rule 19): LiDAR range noise 15-30 mm, the two LiDARs agree to about 5 mm, and the LiDAR-to-camera mounting offset has never been measured. It shares the robot's wheels + gyroscope with the blend, so for the blend it is not fully independent. Made by `03_methods/results_packs_2026-09-26/compare_lidar.py`, a copy of `tools/db/compare_lidar.py` changed only in its figure (labels, split panels); the numbers are identical.*

- LiDAR map: 314 positions, 88.83 m, 75 LiDAR loop closures. Its own end-to-start gap, two ways from the same corrected positions: **0.0469 m in 3D** (height included; `lidar.tum.meta.json`, db_corrected_tum.py, from Admin.opt_poses) and **0.032 m on the floor plane** (`lidar_map_facts.json`; compare_lidar.py's own check prints it rounded as "passes its own check: ends 0.03 m from its start, 75 LiDAR loop closures").

| path compared | kind (rule 20) | matched | median | 95th percentile | largest | best-fit scale | file |
|---|---|---|---|---|---|---|---|
| camera map, corrected (camera_corrected.tum) | map's corrected path | 99.6 % (283) | **0.259 m** | **0.491 m** | 0.548 m | 1.0631 | agreement.json |
| map nodes (= blend), tracking alone (camera.tum) | tracking alone | 89.4 % (483) | **0.117 m** | **0.239 m** | 0.272 m | 1.0289 | agreement.json |

- **Best-fit scale** = the size change that would line the path up best with the LiDAR estimate if scaling were allowed; it is NOT applied (stereo has real metric scale; ENGINEERING_NOTES.md section 4 rule 4). The corrected camera map comes out 6.3 % off in size, the blend's node path 2.9 %. [INFERENCE] A size mismatch makes the distance grow with distance from the start and shrink on the way back - the regular rise-and-fall (sawtooth) along each corridor leg in `lidar_comparison.png` fits that; it was not tested by re-running with scaling allowed.
- **Clock shift:** the comparison's own check found the paths fit best with the camera shifted +0.22 s (fit 0.288 m), against no shift; **no shift was applied** (applied: 0.0 s). compare_lidar.py flags a shift only when it improves the fit by more than 0.05 m and 20 %; here the fit improves from 0.295 m to 0.288 m (printed by compare_lidar.py), below that.

*Plain terms: at each moment, how far apart the camera map's position and the LiDAR estimate are, after lining the two paths up once (rotation and shift, no scaling). "camera.tum" here is the map nodes' tracking-alone positions, which in this fused drive are the blend's (section 1).* Drive 3 for scale: corrected median 0.39 m, 95th percentile 1.00 m; tracking alone 1.84 / 3.12 m (drive 3 agreement.json).

Figure: `lidar_comparison.png` (LiDAR solid, camera map dashed, map nodes dotted; tracking alone and corrected on separate panels); `lidar_map.png` - the LiDAR floor map.

## 8. Figures and video in this folder

| file | what it shows |
|---|---|
| map.png | floor map, map nodes at their tracking-alone positions (Node.pose = the blend's) |
| map_corrected.png | floor map after the loop closures (Admin.opt_poses) - the map RTAB-Map believes in |
| trajectory.png | where each source put the robot, two panels (rule 20): left = tracking alone (robot's wheels + gyroscope solid; camera tracking and blend dashed), right = corrected (camera map dashed; LiDAR estimate solid) |
| turns_difference.png | per turn: camera minus robot AND blend minus robot, in degrees |
| lidar_comparison.png | left: the paths over the LiDAR map; right: the distance to the LiDAR estimate along the route, tracking alone (top) and corrected (bottom) on separate panels |
| lidar_map.png | the LiDAR's own floor map |
| timelapse.mp4 | **recorded live during the drive** (not rebuilt): the live map page (camera map left, LiDAR view right) saved every 3 s, 200 frames played at 10 per second = 20 s of video for 10 min of driving (`timelapse_live_index.csv` = one line per frame, with the tracking and correction status shown at that moment; `~/.run_records/s2_static_04/media/recorder.log`: missed 0, write errors 0) |
| timelapse_25/50/75/100.png | stills from that live timelapse (the last is the moment the map closed: its "NO DATA" line is the shutdown) |

Other files: camera.tum / camera_corrected.tum (+ .meta.json); paths_2hz.csv; turns.csv, turns_replayed.log; numbers.json; freezes_pass_check.json; power_temperature.json; closed_check.txt (map shut down properly: corrected positions, visual dictionary and both-way links saved).

## 9. Still open

- N = 1 at these settings: no spread can be quoted (ENGINEERING_NOTES.md section 4).
- One camera freeze remains (section 4): 16-bit depth did not remove it; whether it lowered the rate cannot be said from one drive.
- The live turn watcher did not run; section 2 is its rule replayed from the recording (with a control on drive 3).
- The camera's path is not independent of the blend in a fused drive (section 1); a camera-only path would need the camera's own tracking replayed with no TF restarts.
