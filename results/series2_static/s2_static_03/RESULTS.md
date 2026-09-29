# Drive 3 (s2_static_03) - results pack

*25 Sept 2026, 04:30-04:58 Hamilton time. The first fused drive (fused = the camera blended with the robot's
wheels and gyroscope (turn-rate sensor) by an EKF (extended Kalman filter, a standard way of mixing sensors)).
Made on the Jetson on 25 Sept by `results_packs_2026-09-25/` (project records) (analyze_run.py,
pack_drive3_heavy.sh, this file's writer). Every number names the file it comes from; all files are in this
folder unless a path is given. Nothing here is ground truth (ENGINEERING_NOTES.md rule 19). No LiDAR estimate exists
for this drive yet: it is made on the robot (stage A), which was off and charging.*

## 1. How far from home at the end - both kinds (rule 20)

*Plain terms: "tracking alone" = where each source's own step-by-step tracking (odometry) put the robot,
never corrected; "corrected" = where the map puts it after its loop closures (the map recognising a place
it has seen before and pulling itself straight). A "node" is one saved map snapshot (a picture plus the position
it was taken from); the "graph" is the set of nodes and the links between them. Node.pose = each node's position
from tracking alone; Admin.opt_poses = the corrected positions RTAB-Map saved when it shut down properly - both are
tables inside the map database. The drive was planned to end on its start mark
(NEXT_PLAN_2026-09-25.md item 3), so a perfect estimate would show about 0; the real end position was not
measured with a tape.*

| kind | source | end-to-start gap | heading gap | path length | file |
|---|---|---|---|---|---|
| tracking alone | camera map nodes (Node.pose), node 1 to node 1128 | **5.599 m** | - | 140.17 m | facts.json, camera.tum |
| tracking alone | camera tracking, every message (/rtabmap/odom, lost stretches left out) | 5.640 m | -41.04 deg | 187.9 m | numbers.json (fusion.bag) |
| tracking alone | blend (/fused/odometry) | 5.597 m | -44.77 deg | 154.2 m | numbers.json (fusion.bag) |
| tracking alone | robot's own wheels + gyroscope (/robot/ekf_odom) | 0.280 m | +1.34 deg | 144.8 m | numbers.json (fusion.bag) |
| **corrected** | map's saved graph, Admin.opt_poses, node 1 to node 858 (446 graph nodes) | **0.0552 m** | - | - | camera_corrected.tum.meta.json |
| corrected | every node placed by its graph node's correction, node 1 to node 1128 | 0.053 m | - | 137.01 m | map_corrected_facts.json |

**On tracking alone the blend did not end near the robot's own estimate.** The blend ended 5.597 m / -44.8 deg from its start; the robot's own wheels + gyroscope ended 0.280 m / +1.3 deg from theirs - the blend's position gap is about 20 times the robot's. Blend minus robot at the end: **-46.1 deg** of heading. Of that, the 52 logged turns add up to **-33.4 deg** (turns.csv, sum of blend - robot), of which turn 1 during the link drop (section 3) is -25.2 deg and the other 51 turns -8.2 deg; the remaining **-12.7 deg built up outside the logged turns** (while driving straight or standing). So the link drop explains about half of the blend's heading gap, not all of it. Reported only, not a pass line; the robot's estimate is a comparison, not ground truth.

The blend's position gap is within 0.05 m of the camera's; why the two agree so closely was not checked here.

**Loop closures beside the corrected gap: 320** (20 recognised-again + 300 nearby re-matches; closed_check.txt, and counted in the database's Link table (the table of connections between nodes), types 1 and 2 (1 = recognised a place seen long before, 2 = re-matched a nearby node)). **Two of them join the end straight to the start** (nearby re-matches node 857 -> node 1 and node 660 -> node 1, read from the Link table on 25 Sept), so the corrected gap partly repeats those closures rather than measuring drift independently.
The saved graph ends at node 858 (04:52:01); nodes after it were the robot parked: over them the camera's tracking moved **net 0.015 m** and was never more than 0.018 m from node 858 (small jitters add up to 0.321 m of summed steps; camera.tum).

**Registered pass line (NEXT_PLAN_2026-09-25.md, item 3): "after its own corrections the map comes home within 0.3 m of its start mark" - 0.0552 m: PASS**, with the caveat above.
The other two drive-3 pass lines there (each corridor drawn once; every tracking-lost moment carried through on the live page) are judged on the live page and the two-visit check; they are **not scored in this pack**.

## 2. Every turn: robot vs camera vs blend (live turn watcher)

*Plain terms: for each turn, how many degrees each source says the robot turned. The robot's own wheels + gyroscope is the comparison for short turns (not ground truth). Source: `~/.run_records/s2_static_03/turns.log` (Jetson), copied as turns.csv; the log's clock is UTC, the times below are Hamilton.*

*95th percentile = the value 95 % of turns stay under.*

| summary (52 turns) | median | 95th percentile | largest | within 1 deg |
|---|---|---|---|---|
| abs(blend - robot) | 0.50 deg | 2.42 deg | 25.2 deg | 41 of 52 |
| abs(camera - robot) | 3.40 deg | 20.90 deg | 63.7 deg | 3 of 49 (3 turns: camera had no value) |

Figure: `turns_difference.png`. The largest blend miss, turn 1 (-25.2 deg), is the link drop in section 3.

| turn | ended | dir | s | robot deg | camera deg | blend deg | camera lost | blend - robot | camera - robot |
|---|---|---|---|---|---|---|---|---|---|
| 1 | 04:37:25 | left | 43.2 | 172.4 | 159.6 | 147.2 | 11 % | -25.2 | -12.8 |
| 2 | 04:38:57 | right | 0.2 | -0.3 | -6.2 | -2.5 | 0 % | -2.2 | -5.9 |
| 3 | 04:39:01 | left | 0.2 | -0.1 | 3.4 | 0.7 | 0 % | 0.8 | 3.5 |
| 4 | 04:39:32 | left | 5.1 | 76.0 | 78.6 | 75.8 | 0 % | -0.2 | 2.6 |
| 5 | 04:40:42 | right | 5.0 | -83.0 | -84.0 | -84.2 | 18 % | -1.2 | -1.0 |
| 6 | 04:40:50 | right | 2.6 | -48.4 | -51.7 | -48.9 | 0 % | -0.5 | -3.3 |
| 7 | 04:40:58 | right | 2.6 | -34.2 | -36.1 | -35.3 | 0 % | -1.0 | -1.9 |
| 8 | 04:41:54 | right | 4.1 | -57.7 | -60.4 | -58.3 | 0 % | -0.6 | -2.7 |
| 9 | 04:41:57 | right | 0.8 | -10.6 | -13.9 | -11.6 | 0 % | -1.0 | -3.3 |
| 10 | 04:42:53 | right | 8.8 | -96.2 | -73.1 | -96.5 | 18 % | -0.3 | 23.1 |
| 11 | 04:43:02 | right | 5.9 | -82.8 | -84.9 | -82.6 | 22 % | 0.2 | -2.1 |
| 12 | 04:43:51 | right | 7.4 | 0.8 | -3.7 | 0.7 | 0 % | -0.0 | -4.5 |
| 13 | 04:43:58 | left | 3.7 | 61.2 | 65.9 | 63.4 | 3 % | 2.1 | 4.7 |
| 14 | 04:44:13 | left | 0.6 | 16.7 | 22.8 | 18.9 | 0 % | 2.2 | 6.1 |
| 15 | 04:44:20 | right | 1.5 | -7.6 | -9.6 | -7.8 | 0 % | -0.2 | -2.0 |
| 16 | 04:45:08 | left | 2.5 | 38.0 | 44.8 | 39.4 | 0 % | 1.4 | 6.8 |
| 17 | 04:45:11 | left | 0.7 | 14.8 | 18.3 | 15.8 | 0 % | 1.0 | 3.5 |
| 18 | 04:45:13 | left | 0.5 | 7.9 | 11.4 | 9.1 | 0 % | 1.2 | 3.5 |
| 19 | 04:45:16 | left | 0.2 | 0.4 | 2.8 | 0.1 | 0 % | -0.3 | 2.4 |
| 20 | 04:45:21 | left | 2.5 | 25.4 | 35.5 | 26.3 | 0 % | 0.9 | 10.1 |
| 21 | 04:45:23 | left | 0.3 | 1.2 | 2.7 | 1.4 | 0 % | 0.2 | 1.5 |
| 22 | 04:45:29 | left | 3.2 | 32.1 | 35.6 | 32.2 | 0 % | 0.2 | 3.5 |
| 23 | 04:46:19 | right | 1.6 | -26.7 | -44.3 | -26.7 | 0 % | 0.1 | -17.6 |
| 24 | 04:46:22 | right | 0.9 | -12.7 | -16.7 | -13.3 | 0 % | -0.5 | -4.0 |
| 25 | 04:46:25 | right | 1.2 | -20.6 | -22.2 | -21.4 | 0 % | -0.8 | -1.6 |
| 26 | 04:47:22 | right | 1.3 | -25.1 | -29.6 | -26.1 | 0 % | -1.0 | -4.5 |
| 27 | 04:47:25 | right | 1.1 | -20.3 | -24.2 | -20.5 | 0 % | -0.2 | -3.9 |
| 28 | 04:47:28 | right | 0.8 | -12.8 | -8.7 | -13.8 | 69 % | -1.0 | 4.1 |
| 29 | 04:47:32 | right | 0.4 | -4.2 | - | -5.0 | 100 % | -0.9 | - |
| 30 | 04:47:55 | right | 1.2 | -15.1 | - | -14.9 | 100 % | 0.2 | - |
| 31 | 04:47:57 | right | 0.6 | -9.8 | -73.5 | -10.2 | 79 % | -0.4 | -63.7 |
| 32 | 04:47:59 | right | 0.3 | -2.4 | -4.7 | -2.3 | 0 % | 0.1 | -2.3 |
| 33 | 04:48:04 | right | 3.6 | -27.7 | -31.1 | -27.9 | 0 % | -0.2 | -3.4 |
| 34 | 04:48:13 | right | 0.2 | 0.1 | -1.5 | 0.2 | 0 % | 0.1 | -1.6 |
| 35 | 04:48:59 | left | 4.2 | 63.3 | 64.5 | 64.0 | 0 % | 0.8 | 1.2 |
| 36 | 04:49:01 | left | 0.2 | 0.0 | 2.9 | -0.0 | 0 % | -0.1 | 2.9 |
| 37 | 04:49:05 | left | 0.2 | 1.0 | 7.4 | 1.0 | 0 % | -0.0 | 6.4 |
| 38 | 04:49:07 | left | 0.3 | 1.1 | 5.9 | 2.3 | 0 % | 1.2 | 4.8 |
| 39 | 04:49:13 | left | 3.3 | 37.9 | 38.0 | 38.0 | 18 % | 0.2 | 0.1 |
| 40 | 04:49:19 | left | 3.8 | 26.9 | 29.2 | 26.6 | 0 % | -0.2 | 2.3 |
| 41 | 04:49:28 | right | 0.8 | -9.5 | -10.6 | -9.2 | 0 % | 0.3 | -1.1 |
| 42 | 04:50:22 | right | 1.7 | -45.1 | -49.0 | -45.6 | 0 % | -0.5 | -3.9 |
| 43 | 04:50:25 | right | 0.5 | -6.0 | -6.6 | -7.5 | 0 % | -1.4 | -0.6 |
| 44 | 04:50:28 | right | 0.4 | -9.1 | - | -12.5 | 100 % | -3.3 | - |
| 45 | 04:50:32 | right | 2.3 | -20.4 | -66.6 | -23.1 | 67 % | -2.7 | -46.2 |
| 46 | 04:50:36 | right | 1.7 | -20.7 | -23.9 | -20.9 | 0 % | -0.2 | -3.2 |
| 47 | 04:50:44 | right | 2.7 | 0.3 | -3.3 | 0.2 | 0 % | -0.1 | -3.6 |
| 48 | 04:51:12 | left | 4.3 | 71.6 | 72.7 | 72.1 | 0 % | 0.5 | 1.1 |
| 49 | 04:51:20 | left | 5.6 | 87.9 | 89.2 | 87.4 | 12 % | -0.5 | 1.3 |
| 50 | 04:51:28 | left | 0.5 | 5.1 | 7.1 | 5.4 | 0 % | 0.4 | 2.0 |
| 51 | 04:51:33 | right | 1.3 | -6.0 | -9.3 | -6.1 | 0 % | -0.1 | -3.3 |
| 52 | 04:51:51 | right | 5.0 | 6.6 | 2.8 | 5.9 | 0 % | -0.7 | -3.8 |

## 3. The WiFi link to the robot (the bridge)

*Plain terms: the robot's wheel and gyroscope readings travel to the Jetson over WiFi. Source: `~/.run_records/s2_static_03/bridge_recv.log` (Jetson), its FINAL counters; numbers.json.*

- Link down **1 time**: 04:36:46 STATE DOWN no bytes for 1.5 s; reconnected 04:37:24, clock re-locked 04:37:25 - **about 39 s with no wheel or gyroscope data**, during turn 1 (the log shows a full WiFi re-login to a new access point; DRIVE4_PLAN_2026-09-25.md).
- Connections: 2 (one receiver session, 1710.9 s).

| stream | robot read | robot sent | published on the Jetson | % of sent | % of what the robot read | missed while down | too late (stale) |
|---|---|---|---|---|---|---|---|
| /husky_velocity_controller/odom | 17297 | 16662 | 16318 | 97.94 | 94.34 | 400 | 350 |
| /imu/data | 33731 | 32489 | 31829 | 97.97 | 94.36 | 779 | 674 |
| /imu/data_raw | 33731 | 32489 | 31829 | 97.97 | 94.36 | 779 | 674 |
| /odometry/filtered | 86501 | 83318 | 81625 | 97.97 | 94.36 | 1998 | 1729 |

## 4. Camera tracking losses

- From the fusion recording (fusion.bag, the drive's recording of the camera, blend and robot data streams; stream `/rtabmap/odom`, lost = RTAB-Map's "lost" signal: a null pose (all zeros) or a covariance (its own uncertainty figure) of 9999): **21 lost stretches, 56.4 s in total, longest 30.2 s, 3.3 % of messages** (numbers.json).
- The mapping monitor counted **55 odometry resets** (times the camera's tracking gave up and restarted from its last good position) by the end (`~/.run_records/s2_static_03/monitor_STATUS.txt`, Jetson).
- Turns with the camera lost for at least half the turn: turns 28-31 and 44-45 (turns.csv); the blend stayed within 3.3 deg of the robot on each.

## 5. Blend heading while standing still (reported, not a pass line)

*Plain terms: when the wheels read zero the robot is not turning, so any heading change in the blend is its own error. Asked for by the user's decision on T2 line 3 (results_conditioner_round3.md section 9).*

| | drive 3 (this pack, numbers.json) | drive-2 replay (results_conditioner_round3.md section 9) |
|---|---|---|
| stops | 167 (962 s still) | 30 |
| signed sum | **+0.531 deg** | -0.74 deg |
| total | 6.117 deg (sum of each stop's net change) | 1.467 deg |
| largest single stop | +0.628 deg | - |

Rule: robot wheel odometry speed and turn rate both zero, below 1e-6 - the log holds 1e-16 float noise, not 0.0 (nearest wheel message within 0.3 s). **The one-way lean of the replay (negative) does not appear here: the signed sum is small and positive.** The two columns are not like-for-like - drive 3 has 167 stops against 30, and the replay's exact stop definition was not re-derived - so compare signs and sizes, not digits. The sum of every small heading step inside the stops is 69.3 deg: sensor noise that nets out.

## 6. Processor load

- Fusion programs on the Jetson (bridge receiver, input conditioner, EKF), sampled every second (`~/.run_records/s2_static_03/cpu_jetson.json`): **median 0.11 cores, 99th percentile 0.14, largest 1.043** - within cpu_sampler's default limit (median at most 2.0 cores, 99th percentile at most 4.0; set in `03_methods/occupancy_research_2026-09-24/REPORT.md` section 4.3 for the LiDAR pipeline): **PASS**. That limit is not a registered pass line of this drive. (A core = one of the Jetson's 12 processors kept fully busy; 99th percentile = the value the load stayed under 99 % of the time.)
- RTAB-Map, time to process each map snapshot (node), from the database's Statistics table (RTAB-Map's own per-snapshot timing log; read-only): **median 667.6 ms, 95th percentile 922.4 ms, largest 3353.3 ms over 1128 nodes**. The mapping program's own processor share was not sampled during this drive.

## 7. Figures and video in this folder

| file | what it shows |
|---|---|
| map.png | floor map from the camera's tracking alone (Node.pose) |
| map_corrected.png | floor map after the loop closures (Admin.opt_poses) - the map RTAB-Map believes in |
| trajectory.png | where each source put the robot; all dashed (no LiDAR estimate yet, so no solid line) |
| turns_difference.png | per turn: camera and blend minus the robot's heading change |
| timelapse_rebuilt.mp4 | the corrected map growing, 40 s of video for 1686 s of driving (1000 frames at 25 per second); **rebuilt after the drive from the database, not recorded live** - every frame says so |
| timelapse_25/50/75/100.png | stills from the timelapse |

Other files: camera.tum / camera_corrected.tum (+ .meta.json) - the two camera paths; paths_2hz.csv - camera, blend and robot paths at 2 per second; turns.csv; numbers.json; closed_check.txt (the map was shut down properly: corrected positions, visual dictionary and both-way links all saved).

## 8. Still open

- **LiDAR estimate**: stage A on the robot, parked, when it is on; then `compare_lidar.py` for "agreement with the LiDAR estimate" and a solid LiDAR line on the trajectory figure.
- Pass lines "each corridor drawn once" and "tracking losses carried through on the page" - not scored here.
- N = 1: one fused drive. No spread can be quoted yet (ENGINEERING_NOTES.md section 4).
