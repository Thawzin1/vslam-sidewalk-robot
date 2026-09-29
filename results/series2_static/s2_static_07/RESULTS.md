# Drive 7 (s2_static_07) - results pack

*26 Sept 2026, Hamilton time: live recorder started 13:27:56, autostop armed 13:28:02, "park" 13:40:26, map closed properly 13:41:15. User-driven, over drive 4's two corridors. The fifth fused drive (fused = the camera blended with the robot's wheels and gyroscope (turn-rate sensor) by an EKF (extended Kalman filter, a standard way of mixing sensors)) and the first mapping drive with the camera's own tracker set to GEN_1 (drive 4: GEN_2). Made on the Jetson on 26 Sept by `results_packs_2026-09-26/` (project records) (pack_drive7_heavy.sh, analyze_run.py, turns_replay.py, extract_timing.py, compare_lidar.py, size_fit_agreement.py, compare_drive4_drive7.py, this file's writer `write_results_drive7.py`). Every number names the file it comes from; files are in this folder unless a path is given. Nothing here is ground truth (ENGINEERING_NOTES.md rule 19). N = 1.*

*Words used below: node = one saved map snapshot; TF = ROS's shared table of where each part of the robot is; Node.pose = each node's tracking-alone position; Admin.opt_poses = the corrected positions RTAB-Map saves when it shuts down properly; fusion.bag = the drive's recording of every sensor stream; SE(3) = lining two paths up by rotation and shift only; Sim(3) = the same but also allowing a size change; ffmpeg = a standard video program.*

## 0. What was different from drive 4 (read first)

| change | plain meaning | evidence |
|---|---|---|
| camera tracker GEN_1 (drive 4: GEN_2) | the ZED camera's built-in position tracker, changed to stop the camera program crashing | `~/.run_records/s2_static_07/camera.log` "Positional tracking mode -> GEN 1"; drive 4's camera.log "GEN 2" |
| **longer route** | drive 7 went much further down the second corridor: about 14-15 m against drive 4's about 6.5 m (map nodes, tracking alone), and ran 824 s against 619 s | camera.tum here and in s2_static_04; facts.json `duration_s` |
| one WiFi link drop | the robot's data stopped for a few seconds once (section 3); drive 4 had none | `bridge_recv.log` |
| camera guard watching | restarts the camera program if it dies; it did not need to (restarts 0) | `camera_guard.json` |
| mapping settings | none changed: the database's saved RTAB-Map settings are identical to drive 6's (drive 4's database is not on the Jetson to compare directly) | Info table, s2_static_06.db vs s2_static_07.db, read-only |

*[INFERENCE] What GEN_1 can and cannot have changed:* the map and the blend both use RTAB-Map's own camera tracking (`/rtabmap/odom`), not the ZED's built-in tracker's positions, and the ZED publishes no TF (`sidewalk_perception/config/zedx_front.yaml` line 173 `publish_tf: false`; the blend's inputs are wheels, two gyroscopes and `/rtabmap/odom`, `sidewalk_slam/config/ekf_fused.yaml` lines 129-216). So GEN_1's positions do not enter the map. **But the tracker runs inside the camera program's picture loop, and that loop paused 16 times this drive (section 4)** - that is a path by which GEN_1 can change the map: pictures stop, tracking stops, the tracker restarts from the blend.

## 1. How far from home at the end - both kinds (rule 20)

*Plain terms: "tracking alone" = where each source's own step-by-step tracking put the robot, never corrected; "corrected" = where the map puts it after its loop closures (the map recognising a place seen before and pulling itself straight). In a fused drive the map's nodes carry the BLEND's position (drive 4 RESULTS.md section 1 explains why), and the camera's tracking restarts from the blend each time it gives up. The drive ended on its start mark by eye, so a perfect estimate shows about 0; the real end was not taped.*

| kind | source | end-to-start gap | heading gap | path length | file |
|---|---|---|---|---|---|
| tracking alone | map nodes (Node.pose = the blend's position at each node), node 1 to node 729 | **0.995 m** | - | 107.09 m | facts.json, camera.tum |
| tracking alone | camera tracking, every message (/rtabmap/odom, lost stretches left out; re-seeded by the blend at each restart) | 0.599 m | +11.26 deg | 119.5 m | numbers.json (fusion.bag) |
| tracking alone | blend (/fused/odometry) | 0.995 m | +0.37 deg | 114.0 m | numbers.json |
| tracking alone | robot's own wheels + gyroscope (/robot/ekf_odom) | 0.122 m | +2.67 deg | 108.7 m | numbers.json |
| **corrected** | camera map's saved graph, Admin.opt_poses, node 1 to node 678 (582 graph nodes), 3D | **0.1696 m** | - | - | camera_corrected.tum.meta.json |
| corrected | every camera node placed by its graph node's correction, node 1 to node 729, floor plane | 0.174 m | - | 103.43 m | map_corrected_facts.json |
| corrected (0 LiDAR closures - nothing corrected: this is the robot's wheels + gyroscope) | **LiDAR map** (robot, colleague's self_navigation rtabmap_3d.launch), floor plane | **0.158 m** | - | 108.0 m | lidar_map_facts.json |
| corrected (0 LiDAR closures - nothing corrected: this is the robot's wheels + gyroscope) | LiDAR map, 3D (height included) | 0.8226 m | - | - | lidar.tum.meta.json |

**Loop closures beside the corrected gaps: camera map 30** (4 recognised-again + 26 nearby re-matches; database Link table types 1 and 2, counted once per pair; `closed_check.txt` gives 30); the live monitor's last line said 33 accepted (`~/.run_records/s2_static_07/monitor_STATUS.txt`) - the database, read after the map closed, is the complete count. Rejected in `mapping.log`: 0. **LiDAR map 0** (lidar.tum.meta.json).

- **Closures joining a later node straight to node 1 (the start): 0.** So nothing ties the end of the drive to its start: the camera map's corrected gap is the blend's tracking plus the one burst of closures below, not a closure repeating itself (rule 20).
- **All 30 camera closures fall in 46.6 s**, 13:37:06 to 13:37:52 (newer node's time), joining older nodes 177-221 to newer nodes 518-552. None in the 153 s from the last closure to "park". *Plain terms: the camera map recognised one stretch of the route once, and nowhere else - drive 4's 181 closures were spread over the whole drive.*
- 582 of 729 nodes are in the saved graph (drive 4: 284 of 540). The other 147 are still in the database's Node table with weight -9 (RTAB-Map keeps them but leaves them out of the map graph); 51 of them are the nodes after node 678, i.e. after 13:40:07, while the robot was parking ("park" at 13:40:26). Why the others were left out is not established. So the corrected gap's end point, node 678, is 13:40:07 - 18 s before "park"; the every-node floor gap to node 729 (0.174 m, next row of the table above) shows this makes little difference.

## 2. Every turn: robot vs camera vs blend

The live turn watcher's rule replayed after the drive over fusion.bag by `turns_replay.py` (unchanged; its control on drive 3 is in drive 4 RESULTS.md section 2). Output: turns_replayed.log -> turns.csv. *95th percentile = the value 95 % of turns stay under. The robot's own wheels + gyroscope is the comparison for short turns (not ground truth).*

| summary (13 turns) | median | 95th percentile | largest | within 1 deg |
|---|---|---|---|---|
| abs(blend - robot) | 0.20 deg | 0.94 deg | 1.3 deg | 12 of 13 |
| abs(camera - robot) | 2.70 deg | 52.54 deg | 115.3 deg | 0 of 13 |

Turns with no camera value (camera lost for the whole turn): 0. Figure: `turns_difference.png`.

| turn | ended | direction | s | robot deg | camera deg | blend deg | camera lost | blend - robot | camera - robot |
|---|---|---|---|---|---|---|---|---|---|
| 1 | 13:29:59 | left | 13.8 | 172.2 | 56.9 | 172.2 | 35 % | -0.1 | -115.3 |
| 2 | 13:31:13 | left | 6.6 | 82.1 | 83.8 | 82.3 | 0 % | 0.2 | 1.7 |
| 3 | 13:32:52 | right | 7.1 | -86.1 | -87.2 | -86.7 | 20 % | -0.6 | -1.1 |
| 4 | 13:33:03 | right | 3.2 | -75.2 | -81.7 | -75.8 | 0 % | -0.6 | -6.5 |
| 5 | 13:34:32 | right | 8.9 | -52.3 | -55.0 | -52.4 | 0 % | -0.1 | -2.7 |
| 6 | 13:34:45 | right | 2.1 | -8.6 | -10.6 | -8.6 | 0 % | -0.0 | -2.0 |
| 7 | 13:35:32 | left | 4.4 | 21.0 | 28.8 | 20.6 | 0 % | -0.3 | 7.8 |
| 8 | 13:35:42 | right | 4.6 | -16.9 | -19.2 | -17.0 | 0 % | -0.0 | -2.3 |
| 9 | 13:36:24 | right | 23.8 | -170.3 | -159.6 | -171.0 | 9 % | -0.7 | 10.7 |
| 10 | 13:37:33 | left | 5.4 | 84.4 | 86.5 | 84.4 | 0 % | -0.0 | 2.1 |
| 11 | 13:38:52 | left | 9.9 | 168.5 | 175.8 | 168.6 | 0 % | 0.1 | 7.3 |
| 12 | 13:39:56 | right | 4.2 | -61.3 | -65.6 | -62.6 | 0 % | -1.3 | -4.3 |
| 13 | 13:40:00 | right | 1.8 | -21.0 | -23.3 | -20.9 | 0 % | 0.2 | -2.3 |

## 3. The WiFi link to the robot (the bridge)

*Source: `~/.run_records/s2_static_07/bridge_recv.log` (Jetson), its final counters; numbers.json. Drive 7's bridge and autostop logs print times on the Jetson's UTC clock (drive 6's printed Hamilton time), so here numbers.json's `down_lines_hamilton` (UTC minus 4 h) is right.*

- **Link down: 1 time(s)**, at 13:32:35 Hamilton. Connections: 2 in 1 receiver session(s).
- Robot-data silences over 1.5 s that did NOT have the freeze signature (freezes_pass_check.json, `other_wheel_silences_over_1p5s`, first number = seconds): 1, longest 5.2 s.
- Parked-drop guard (`fused_odometry.log`): 2 line(s): 13:33:04 DROP GUARD: wheels and gyroscope silent for >= 0.3 s while PARKED - publishing "not moving; 13:33:05 DROP GUARD ended after 0.77 s: the wheels came back (held throughout).

| stream | robot read | robot sent | published on the Jetson | % of sent | missed while down | too late (stale) | lost |
|---|---|---|---|---|---|---|---|
| /husky_velocity_controller/odom | 8900 | 8767 | 8699 | 99.22 | 50 | 91 | 0 |
| /imu/data | 17279 | 17022 | 16892 | 99.24 | 98 | 175 | 0 |
| /imu/data_raw | 17279 | 17022 | 16892 | 99.24 | 98 | 175 | 0 |
| /odometry/filtered | 44501 | 43840 | 43502 | 99.23 | 250 | 451 | 0 |

## 4. Camera tracking: losses, restarts, freezes, crashes

- **Camera program crashes: none.** The camera guard (`camera_guard.json`) ended with restarts 0, state "ended"; `camera_guard.log` shows one camera session from 13:27:55 to the drive's end.
- **Lost** (fusion.bag, `/rtabmap/odom`: all-zero pose or covariance 9999): **5 stretches, 14.9 s in total, longest 10.3 s, 0.9 % of messages** (numbers.json).
- **Restarts from TF** (`mapping.log`, "Odometry automatically reset to latest odometry pose available from TF" - the camera's tracking gave up and restarted from the blend's position): **9**, in 3 burst(s): 13:29:51-13:30:00 (5); 13:32:45-13:32:46 (2); 13:36:12-13:36:14 (2). The mapping monitor's last line says resets=9 (`monitor_STATUS.txt`).
- **Freezes** (pass_check.py from `03_methods/jetson_stalls_drive3_2026-09-25/`, on this drive's recording -> freezes_pass_check.json): camera tracker silences > 1.5 s: **16** (3.43 s, 3.23 s, 6.30 s, 2.83 s, 5.77 s, 2.70 s, 3.77 s, 3.87 s, 5.67 s, 5.43 s, 2.70 s, 2.30 s, 3.83 s, 2.03 s, 2.47 s, 5.73 s); 'ZED Diagnostic' pauses > 2 s: 16 (4.05 s, 4.03 s, 7.09 s, 3.78 s, 6.63 s, 3.42 s, 4.43 s, 4.00 s, 5.80 s, 5.64 s, 3.56 s, 3.23 s, 4.25 s, 3.05 s, 3.03 s, 6.50 s); database saves >= 5.5 s: 0; control, camera gyroscope longest gap 0.037 s; map updates parsed 707 of 729 nodes. Verdict as printed: **FAIL** (pass line = zero silences).
- **Where the freezes come from** (`freeze_timing_drive7.py` -> freeze_timing.json, same gap rule as pass_check.py): **all 16 overlap a pause of the camera program's own picture loop ('ZED Diagnostic' > 2 s); only 3 overlap a database save of 1.5 s or more** (12 such saves, longest 4.88 s). Drive 3's freezes were the reverse (during long saves). *Plain terms: this time the camera program itself stopped delivering pictures, while the map was not busy saving.*
- **Control on the other GEN_1 recording** (s2_scale_01, same afternoon, 11.3 min, same check -> freeze_control_s2_scale_01.json): 9 camera tracker silences > 1.5 s and 9 picture-loop pauses > 2 s, 0 database saves >= 5.5 s. Drives 4 and 6 (GEN 2 in their camera.log) had 1 (10.1 min) and 3 outside drive 6's outage; drive 5 (GEN_2 by default: it ran before the tracker option existed, commit ddca8cc, [INFERENCE]) had 2 (chart.csv). Drive 3's 9 are not counted here: they had a different signature (camera and robot data silent together, during long database saves; its FINDINGS.md). [INFERENCE] The freezes go with GEN_1 (2 GEN_1 recordings: 16 and 9; 3 GEN_2 drives: 1-3). Only 2 against 3 recordings, from different sessions, so this is an association to test (one GEN_1 / GEN_2 pair back to back), not a proven cause.
- Database integrity (`db_check.txt`, SQLite quick_check (its own integrity check), read-only): `PASS ~/slam_series2/s2_static_07.db: quick_check=['ok'], 729 map nodes, 0.7 s`

## 5. Blend heading while standing still (reported, not a pass line)

stops 26 (144 s still); signed sum **-0.003 deg**; sum of each stop's net change 0.460 deg; largest single stop -0.069 deg (numbers.json). Rule: robot wheel odometry speed and turn rate both zero, below 1e-6 - the log holds 1e-16 float noise, not 0.0 (nearest wheel message within 0.3 s).

## 6. Processor load

- **No tegrastats log (NVIDIA's once-a-second load/power/temperature log) was captured for drive 7**: `~/rehearsal_t5/` holds drives 4-6 and the localisation runs only, and no tegrastats process was running when the pack was made. So there is no power, temperature, processor or graphics-processor figure for this drive.
- RTAB-Map, time to process each map snapshot (database Statistics table, read-only): **median 525.0 ms, 95th percentile 652.9 ms, largest 3046.4 ms over 729 nodes** (numbers.json). Drive 4: 510.2 / 787.8 / 1852.7 ms over 540 nodes.

## 7. Agreement with the LiDAR estimate

*The reference is the robot's LiDAR map (a colleague's work: self_navigation `rtabmap_3d.launch`, replayed on the robot after the drive): the robot's wheels + gyroscope corrected by LiDAR loop closures - of which drive 7 had none (below). **An independent estimate, not ground truth** (ENGINEERING_NOTES.md rule 19): LiDAR range noise 15-30 mm, the two LiDARs agree to about 5 mm, and the LiDAR-to-camera mounting offset has never been measured. It shares the robot's wheels + gyroscope with the blend, so for the blend it is not fully independent. Alignment: SE(3) (rotation + shift), never scale (ENGINEERING_NOTES.md section 4 rule 4).*

- LiDAR map: 617 positions, 108.0 m, **0 LiDAR loop closures**; its own end gap 0.158 m on the floor, 0.823 m in 3D (section 1). Its own check (agreement.json): "FAILED ITS OWN CHECK: ends 0.16 m from its start although the robot parked on the start mark, with 0 LiDAR loop closures - the differences below measure the reference's drift as much as the camera's".
- **Can the LiDAR estimate be used as the yardstick for drive 7? No - labelled like drive 6.** compare_lidar.py passes a reference only if it ends within 0.5 m of its start AND has at least one LiDAR loop closure. Drive 7's ends 0.16 m from its start on the floor, which is within 0.5 m, but it has **0 LiDAR closures** (drive 4: 75), so it failed on the second condition only (the check's printed sentence names both numbers but not which condition failed). With no closures, the "LiDAR estimate" here is the robot's own wheels + gyroscope with nothing correcting it - the wheels the tape test shows read about 6 % long. So for drive 7 the numbers below compare the camera with the robot's uncorrected wheels + gyroscope, not with a LiDAR-corrected estimate, and for the blend they are not independent at all. By eye, the LiDAR map's corridor walls look doubled or smeared in `04_figures/camera_vs_lidar_2026-09-26/drive7_camera_vs_lidar.png` and `lidar_map.png`, which fits a map with nothing correcting it (not measured).
- Why the LiDAR map closed no loops is **not established**. The replay used the same scripts as drive 4 (provenance.txt: identical checksums; mode `colleague`, rate 1.0) and its database closed properly (products.log). The replay log's warning "Missing visual features ... Transform cannot be estimated" (462 lines here) also appears in drive 4's replay (298) and drive 6's (688), both of which did close loops, so it does not explain the zero; the "no image subscription" warning is normal for this LiDAR-only replay. Checked on the robot read-only before it went to charge; nothing further could be checked there.

| path compared | kind (rule 20) | matched | median | 95th percentile | largest | best-fit scale (NOT applied) | file |
|---|---|---|---|---|---|---|---|
| camera map, corrected (camera_corrected.tum) | map's corrected path | 100.0 % (582) | **0.445 m** | **0.879 m** | 1.051 m | 1.0778 | agreement.json |
| map nodes (= blend), tracking alone (camera.tum) | tracking alone | 93.0 % (678) | **0.381 m** | **0.750 m** | 0.867 m | 1.0275 | agreement.json |

- Clock shift: estimated +0.60 s, applied 0.0 s (compare_lidar.py applies one only if it improves the fit by > 0.05 m and 20 %).

### 7a. For information, size fitted (NOT the result)

*Why this is here: a taped 15.00 m test (`01_runs/series2_static/s2_scale_01/RESULTS.md`, same day) found the robot's wheels read about 6 % long and the camera within 1 %. The LiDAR estimate takes its size from the wheels, so part of the SE(3) distance above is the LiDAR estimate being too large, not the camera map's shape. Below, the same comparison is repeated allowing one size change (Sim(3)). This is for information only: the camera's size is real and is not to be fitted away (ENGINEERING_NOTES.md section 4 rule 4).* Made by `size_fit_agreement.py` -> size_fit.json, same reference, same 50 ms pairing and the same fitting code as compare_lidar.py; its rigid (SE(3)) run reproduces agreement.json exactly (control: passed).

| path compared | pairs | SE(3) median / 95th pct (the result) | size fitted: median / 95th pct | size factor applied to the camera path |
|---|---|---|---|---|
| camera map, corrected | 582 | 0.445 / 0.879 m | 0.161 / 0.361 m (for information, size fitted) | 1.0778 |
| map nodes (= blend), tracking alone | 678 | 0.381 / 0.750 m | 0.377 / 0.660 m (for information, size fitted) | 1.0275 |

*Plain terms: a size factor of 1.078 = the corrected camera path would have to be stretched by 7.8 % to fit the LiDAR estimate best. The tape test says the robot's wheels + gyroscope read 1.064 times the true distance and the camera 0.993 times, so the size difference expected from the tape alone is 1.064 / 0.993 = 1.071. [INFERENCE] Drive 7's 1.078 is close to that, so most of the size difference is the reference's, not the camera's. [INFERENCE] The blend's factor (1.028) is smaller because the blend itself carries part of the wheels' length (s2_scale_01: blend about 2.7 % long).*

## 8. Distances - from the camera and from the wheels

*Each source's own path length for the whole drive. The wheels read long (tape: robot wheels + gyroscope 6.4 % long, wheels alone 6.2 %, s2_scale_01 legs.json, legs 1, 2, 3, 6); the camera read within 1 %. "2 Hz" = positions every 0.5 s, straight steps summed (paths_2hz.csv), which leaves out most of the small back-and-forth that summing every message adds.*

| source | what it is | path length | file |
|---|---|---|---|
| **camera map, corrected** | camera, after its loop closures (every node) | **103.4 m** | map_corrected_facts.json |
| camera tracking, 2 Hz | camera's own tracking (re-seeded by the blend at restarts) | 107.7 m | paths_2hz.csv |
| map nodes, tracking alone | the blend's position at each node | 107.1 m | facts.json |
| blend, 2 Hz | camera + wheels + gyroscopes | 107.5 m | paths_2hz.csv |
| **robot's own wheels + gyroscope, 2 Hz** | wheels (what the LiDAR map is built on) | **107.8 m** | paths_2hz.csv |
| LiDAR map | wheels + gyroscope, NOT corrected (0 LiDAR closures) | 108.0 m | lidar_map_facts.json |
| robot's wheels + gyroscope x 0.940 (for information: the tape's correction) | wheels, resized by the taped test | 101.3 m | computed here |

**Distance to quote for drive 7: about 103 m by the camera map** (the wheels say 108 m, which the tape shows reads about 6 % long).

## 9. Drive 4 vs drive 7 - did drive 7 repeat drive 4?

**In plain words: partly.** Drive 7 came home about as well as drive 4 - the camera map's corrected gap 0.17 m against 0.05 m agrees within the parking uncertainty, and the size difference against the LiDAR estimate is the same (1.078 against 1.063). But it did **not** repeat how drive 4 got there: the camera map made 30 loop closures against 181 (none joining the start), the camera froze 16 times against 1, the blend's own tracking ended 0.99 m out against 0.06 m, and the LiDAR map made 0 closures against 75, so drive 7 has no LiDAR yardstick. Drive 7 also went further down the second corridor, so the two drives are not the same route.

**Verdict (computed): of 20 judged rows, 9 agree within the combined uncertainty, 7 differ by 1-3 times it, and 4 differ by more than 3 times it: camera map loop closures (count) (10.4 times); LiDAR map loop closures (count) (8.7 times); camera freezes (tracker silent > 1.5 s) (3.6 times); start-to-end gap, map nodes tracking alone (= blend) (3.3 times).**

*Plain terms: two drives never give identical numbers. For each row, the spread one drive is expected to have is stated, with where it comes from; the two are combined as sqrt(s4^2 + s7^2). "Agrees within X" = the difference is smaller than that; "differs by N times" = it is N times larger (3 or more is a real difference). The spreads for distances are working figures, not a measured repeatability - N = 1 drive each. Drive 4 and drive 7 share the two corridors but not the whole route (drive 7 went about twice as far down the second one), so this is a like-for-like comparison only for the kinds of numbers that do not grow with route length.*

| row | drive 4 | drive 7 | spread per drive (4 / 7) | from | agrees? |
|---|---|---|---|---|---|
| distance driven - robot's own wheels + gyroscope / LiDAR map path | 90 m / 89 m | 109 m / 108 m | - | - | not judged: the route differs (drive 7 went further down the second corridor); both read about 6 % long (tape test) |
| distance driven - camera map corrected / wheels + gyroscope at 2 Hz (camera is the honest ruler, tape test) | 85.1 m / 89.0 m | 103.4 m / 107.8 m | - | - | not judged: route differs |
| start-to-end gap, camera tracking alone (0 closures) | 0.25 m | 0.60 m | 0.20 / 0.20 m | A | **differs** by 1.2 times the combined uncertainty (difference 0.35 m, combined 0.28 m) |
| start-to-end gap, map nodes tracking alone (= blend) | 0.06 m | 0.99 m | 0.20 / 0.20 m | A | **differs** by 3.3 times the combined uncertainty (difference 0.94 m, combined 0.28 m) |
| start-to-end gap, robot's own wheels + gyroscope, tracking alone | 0.27 m | 0.12 m | 0.20 / 0.20 m | A | **agrees** within 0.28 m (difference 0.14 m) |
| heading gap, robot's own wheels + gyroscope | +2.3 deg | +2.7 deg | 3.0 / 3.0 deg | B | **agrees** within 4.2 deg (difference 0.4 deg) |
| start-to-end gap, **camera map corrected** (floor) | 0.05 m | 0.17 m | 0.20 / 0.20 m | A | **agrees** within 0.28 m (difference 0.12 m) - but beside it: drive 4 181 closures (3 join the start), drive 7 30 (0 join the start); drive 7's gap is not propped up by an end-to-start closure, drive 4's partly is (rule 20) |
| camera map loop closures (count) | 181 | 30 | 13 / 5 | C | **differs** by 10.4 times the combined uncertainty (difference 151, combined 15) - drive 7's is below every earlier drive (320, 181, 72, 135) |
| start-to-end gap, **LiDAR map corrected** (floor) | 0.03 m | 0.16 m | 0.20 / 0.20 m | A | **agrees** within 0.28 m (difference 0.13 m) - drive 7's LiDAR map had 0 closures, so its gap is the uncorrected wheels + gyroscope |
| LiDAR map loop closures (count) | 75 | 0 | 9 / 0 | C | **differs** by 8.7 times the combined uncertainty (difference 75, combined 9) |
| LiDAR estimate passes its own check | yes | NO | - | - | not judged: a pass/fail check |
| same gaps in 3D: camera map | 0.05 m | 0.17 m | 0.20 / 0.20 m | A | **agrees** within 0.28 m (difference 0.12 m) |
| same gaps in 3D: LiDAR map | 0.05 m | 0.82 m | 0.20 / 0.20 m | A | **differs** by 2.7 times the combined uncertainty (difference 0.78 m, combined 0.28 m) |
| camera freezes (tracker silent > 1.5 s) | 1 | 16 | 1 / 4 | C | **differs** by 3.6 times the combined uncertainty (difference 15, combined 4) - drive 7 ran 1.3 times as long; per 10 min: 1.0 against 11.6 |
| camera tracking restarts from the blend | 26 | 9 | 5 / 3 | C | **differs** by 2.9 times the combined uncertainty (difference 17, combined 6) |
| camera tracking lost, seconds in total | 26.2 s | 14.9 s | 18.1 / 18.1 s | D | **agrees** within 25.6 s (difference 11.3 s) |
| camera program crashes | none | none (camera guard: 0 restarts) | - | - | not judged: same: none on either |
| **agreement with the LiDAR estimate**, camera map corrected: median (SE(3)) | 0.26 m | 0.45 m | 0.08 / 0.08 m | E | **differs** by 1.6 times the combined uncertainty (difference 0.19 m, combined 0.12 m) - drive 7's LiDAR estimate failed its own check (0 LiDAR closures), so its side is the camera against the robot's uncorrected wheels + gyroscope |
| **agreement with the LiDAR estimate**, camera map corrected: 95th percentile (SE(3)) | 0.49 m | 0.88 m | 0.27 / 0.27 m | E | **differs** by 1.0 times the combined uncertainty (difference 0.39 m, combined 0.38 m) - drive 7's LiDAR estimate failed its own check (0 LiDAR closures), so its side is the camera against the robot's uncorrected wheels + gyroscope |
| agreement, map nodes (= blend) tracking alone: median | 0.12 m | 0.38 m | 0.08 / 0.08 m | F | **differs** by 2.3 times the combined uncertainty (difference 0.26 m, combined 0.12 m) - drive 7's LiDAR estimate failed its own check (0 LiDAR closures), so its side is the camera against the robot's uncorrected wheels + gyroscope |
| agreement, map nodes (= blend) tracking alone: 95th percentile | 0.24 m | 0.75 m | 0.27 / 0.27 m | F | **differs** by 1.3 times the combined uncertainty (difference 0.51 m, combined 0.38 m) - drive 7's LiDAR estimate failed its own check (0 LiDAR closures), so its side is the camera against the robot's uncorrected wheels + gyroscope |
| for information, size fitted: camera map corrected median | 0.09 m | 0.16 m | 0.08 / 0.08 m | G | **agrees** within 0.12 m (difference 0.07 m) - drive 7's LiDAR estimate failed its own check (0 LiDAR closures), so its side is the camera against the robot's uncorrected wheels + gyroscope |
| for information, size fitted: camera map corrected 95th percentile | 0.19 m | 0.36 m | 0.27 / 0.27 m | G | **agrees** within 0.38 m (difference 0.17 m) - drive 7's LiDAR estimate failed its own check (0 LiDAR closures), so its side is the camera against the robot's uncorrected wheels + gyroscope |
| best-fit size factor, camera map corrected vs LiDAR estimate (not applied) | 1.063 | 1.078 | 0.015 / 0.015 | H | **agrees** within 0.021 (difference 0.015) |

Spread sources (column "from"):
- **A** = where the robot parked: s2_scale_01 found stops at one mark up to 0.19 m apart even with a wheel stop (its RESULTS.md, uncertainty), and drives 4 and 7 parked on the start mark by eye
- **B** = parking heading by eye, working figure (not measured)
- **C** = counting spread, the square root of each count (Poisson, the least spread a count of chance events has)
- **D** = drive-to-drive spread (sample standard deviation) of drives 3-6
- **E** = drive-to-drive spread (sample standard deviation) of the camera map's agreement on drives 3, 4, 5, the drives whose LiDAR estimate passed its own check; different routes, so it may overstate the spread for a repeat
- **F** = drive-to-drive spread (sample standard deviation) of the camera map's agreement on drives 3, 4, 5, the drives whose LiDAR estimate passed its own check; different routes, so it may overstate the spread for a repeat (borrowed from the camera map's rows)
- **G** = drive-to-drive spread (sample standard deviation) of the camera map's agreement on drives 3, 4, 5, the drives whose LiDAR estimate passed its own check; different routes, so it may overstate the spread for a repeat (borrowed)
- **H** = drive-to-drive spread (sample standard deviation) of drives 3, 4, 5 (valid LiDAR estimates)

Caveats on the spreads: (1) the 0.20 m for tracking-alone gaps is only the measurement floor (where the robot parked); how much tracking drift varies from drive to drive is not known and is surely larger, so "differs" on those rows is weaker than it reads. (2) The blend's agreement rows borrow the camera map's spread; the blend's own values on drives 3-5 (1.84 / 0.12 / 0.12 m median) vary far more because of drive 3's link drop, so their "differs" verdicts rest on that borrowing.

## 10. Figures, video and 3D map in this folder

| file | what it shows |
|---|---|
| map.png | camera floor map, map nodes at their tracking-alone positions (Node.pose = the blend's) |
| map_corrected.png | camera floor map after the loop closures (Admin.opt_poses) - the map RTAB-Map believes in |
| trajectory.png | where each source put the robot, two panels (rule 20): tracking alone (robot solid; camera and blend dashed) and corrected (camera map dashed; LiDAR estimate solid) |
| turns_difference.png | per turn: camera minus robot and blend minus robot, degrees |
| lidar_comparison.png | camera paths over the LiDAR map (LiDAR solid, camera dashed) and the distance to the LiDAR estimate along the route (SE(3)) |
| lidar_map.png | the LiDAR's own floor map |
| timelapse.mp4 | **recorded live during the drive**: the live map page (camera map left, LiDAR view right) every 3 s, 260 frames at 10 a second (`timelapse_live_index.csv`; `~/.run_records/s2_static_07/media/recorder.log`: missed 0, write errors 0) |
| timelapse_25/50/75/100.png | stills at 25/50/75/100 % of the timelapse (video frames 65, 130, 195, 259, counted from 0; extracted with ffmpeg) |
| map_3d/drive7_map_3d.ply, map_3d/d7_3d.png | the camera map exported to 3D like drive 4's: `rtabmap-export --cloud --max_range 6 --decimation 4 --voxel 0.03` (depth up to 6 m, every 4th pixel, points merged into 3 cm cubes) on a copy of the database (export.log); d7_3d.png = two views (render.py) |

Other files: camera.tum / camera_corrected.tum (+ .meta.json); paths_2hz.csv; turns.csv, turns_replayed.log; numbers.json; freezes_pass_check.json; db_check.txt; closed_check.txt; size_fit.json; lidar.tum, wheel.tum, lidar_map.npz, lidar_reference_dense.tum, lidar_comparison/; products.log, provenance.txt (the robot's replay).

## 11. Still open

- N = 1 at these settings: no spread can be quoted (ENGINEERING_NOTES.md section 4). Drive 4 vs 7 is one pair, not a repeat on the same route.
- Why the camera map closed loops over only one 46.6 s stretch is not established. [INFERENCE] candidates: the 16 camera freezes (pictures missing where a revisit would have been recognised), the longer second-corridor leg, the driving line, lighting or people in the scene.
- Whether GEN_1 causes the picture-loop pauses (section 4): test with one GEN_1 and one GEN_2 recording back to back, standing and driving, before the next mapping drive.
- No tegrastats log for this drive (section 6): start one with every drive.
- The camera's path is not independent of the blend in a fused drive (section 1).
