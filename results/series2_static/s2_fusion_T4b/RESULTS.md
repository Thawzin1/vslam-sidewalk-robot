# Rehearsal T4b (s2_fusion_T4b) - results pack

*25 Sept 2026, 03:58-04:14 Hamilton time. The parked rehearsal before drive 3: the robot stayed on one spot and only turned in place, to test the blend (the EKF (extended Kalman filter) that mixes the camera with the robot's wheels and gyroscope (turn-rate sensor)) with the lens covered, during blind turns, and through a forced cut of the WiFi link. Made on the Jetson by `results_packs_2026-09-25/` (project records). Sources: the run records `~/.run_records/s2_fusion_T4b/` (Jetson); the scoring `t4_score.json` / `t4_score.txt` (copied here, made by `03_methods/fusion_bridge_2026-09-24/t4/t4_score.py` on 25 Sept 04:18). Nothing here is ground truth.*

**No floor map and no timelapse**: this rehearsal's map database is not on the Jetson (searched 25 Sept), and a parked rehearsal draws almost no map. `heading_timeline.png` takes the timelapse's place: it shows the whole rehearsal at once.

## 1. The rehearsal's own pass lines (registered before it, scored by t4_score.py)

| line | verdict | the number |
|---|---|---|
| 1 bridge (link carries the data in time) | **FAIL** | delivered % of sent, per receiver session: odom [99.22, 87.94], data [99.18, 87.86], data_raw [99.18, 87.94], filtered [99.2, 87.77] (limit 95 % each) |
| 2 clocks | PASS | 100 % of 1937 pings bound within 5 ms |
| 3 conditioner and rates | **FAIL** | camera **6.72 frames per second** while parked (pass line 12-15; 10-second windows 4.9-7.5, whole run 10.03) - the only part that failed; wheel 9.86 Hz (9-11) and gyroscope 19.23 Hz (18-21) within their limits, heading bias kept +0.48 deg/min from 739 samples (limit 1.0 from at least 400), 0 gyroscope readings refused. Pass rule read from t4_score.py lines 225-226; t4_score.txt prints the figures but no reason |
| 4 parked 5 min | PASS | blend heading moved at most 0.19 deg, position at most 0.0103 m |
| 5 lens covered, parked | PASS | 73.2 s covered: heading at most 0.14 deg, position at most 0.0028 m |
| 6 turns | PASS | blind turns, blend - robot: 0.04, 0.09, 0.02, 0.39, 0.01 deg |
| 7 forced drop | **FAIL** | data back **6.09 s** after the receiver restart (limit 3 s); blend heading changed **5.623 deg**, position **0.0441 m** while parked |
| 8 one owner per position link | PASS | - |
| 10 dot on the live page | PASS | moved at most 0.0103 m |

*Plain terms: "Hz" = readings per second; "heading bias" = the gyroscope's slow built-in drift, which the conditioner (the program that cleans the robot's readings before the blend uses them) measures while parked and subtracts.*

**What drive 4 covers (DRIVE4_PLAN_2026-09-25.md section A), line by line:**

- **Line 7 - covered** by A1 (data back within 3.0 s after a reconnect) and A2 (replaying this rehearsal's drop: heading change at most 0.3 deg, position at most 0.02 m).
- **Line 1 - partly covered.** Its shortfall is in the receiver session after the forced drop (odom [99.22, 87.94] %, data [99.18, 87.86] %, data_raw [99.18, 87.94] %, filtered [99.2, 87.77] %); 6.09 s with no data in a 68.8 s session is about 9 % of it, consistent with the shortfall [inference, not checked message by message]. A1 speeds up the return of data, but no drive-4 pass line re-measures the delivered share itself.
- **Line 3 - NOT covered, open.** The camera ran at 6.72 frames per second while parked against a pass line of 12-15; nothing in DRIVE4_PLAN_2026-09-25.md addresses the camera rate. The cause was not investigated in this pack.

## 2. Every turn: robot vs camera vs blend

*Turns found by t4_score.py from the wheels (1 s of stillness closes a turn); headings 2 s either side.*

| start | s | camera lost | robot deg | blend deg | camera deg | blend - robot |
|---|---|---|---|---|---|---|
| 04:07:44.880 | 6.9 | 100 % | +88.40 | +88.44 | - | +0.04 |
| 04:07:53.186 | 1.7 | 100 % | +1.46 | +1.55 | - | +0.09 |
| 04:07:56.288 | 0.1 | 100 % | -0.27 | -0.29 | - | -0.02 |
| 04:09:14.376 | 8.3 | 100 % | -87.66 | -87.27 | - | +0.39 |
| 04:09:26.081 | 0.1 | 100 % | +0.03 | +0.04 | - | +0.01 |
| 04:11:25.178 | 8.9 | 0 % | +89.84 | +89.70 | +90.19 | -0.14 |
| 04:11:36.497 | 0.1 | 0 % | -0.07 | -0.06 | -0.03 | +0.01 |
| 04:12:18.078 | 8.4 | 0 % | -92.29 | -92.60 | -92.28 | -0.31 |
| 04:12:27.581 | 0.1 | 0 % | -8.17 | -8.35 | -8.02 | -0.18 |

## 3. Start-to-end gap (tracking alone only - rule 20)

*The robot never left its spot, so every source should end at 0 m and 0 deg. No corrected kind exists: the map database is not on the Jetson, and the map made 0 loop closures (`~/.run_records/s2_fusion_T4b/monitor_STATUS.txt`).*

| source | end-to-start gap | heading gap | path length | file |
|---|---|---|---|---|
| robot's own wheels + gyroscope | 0.003 m | -1.69 deg | 0.2 m | numbers.json (fusion.bag) |
| blend | 0.075 m | -7.23 deg | 3.7 m | numbers.json (fusion.bag) |
| camera tracking (lost stretches left out) | 0.085 m | -4.70 deg | 18.2 m | numbers.json (fusion.bag) |

The blend's -7.2 deg includes the forced drop's 5.6 deg (line 7).

## 4. Link, tracking losses, standing still, processor load

- Link, receiver session 1 (900.9 s): wheel readings published on the Jetson 99.22 % of those the robot sent (`bridge_recv.log` FINAL line; the other three streams within 0.2 of it, numbers.json).
- Link, receiver session 2 (68.8 s): wheel readings published on the Jetson 87.94 % of those the robot sent (`bridge_recv.log` FINAL line; the other three streams within 0.2 of it, numbers.json).
- Camera tracking lost: **4 stretches, 197.7 s in total, longest 103.2 s** (31.1 % of camera messages; lens covered on purpose, then recovering; numbers.json).
- Blend heading while the wheels read zero: 18 stops, 880 s; signed sum **-0.820 deg**, sum of each stop's net change 1.398 deg, largest stop -0.293 deg (numbers.json; the forced drop is not inside a stop, because no wheel readings arrived then).
- Fusion programs' processor load (`cpu_jetson.json`): median 0.11 cores, 99th percentile 0.15, largest 0.44 - within cpu_sampler's default limit (median at most 2.0 cores, 99th percentile at most 4.0; set in `03_methods/occupancy_research_2026-09-24/REPORT.md` section 4.3 for the LiDAR pipeline): **PASS**. That limit is not a registered pass line of this rehearsal. (A core = one of the Jetson's 12 processors kept fully busy; the 99th percentile = the value the load stayed under 99 % of the time.)

## 5. Files in this folder

| file | what |
|---|---|
| heading_timeline.png | heading of robot, blend and camera through the whole rehearsal, events marked |
| trajectory.png | position of each source (centimetres: the robot stayed put); all dashed, no LiDAR line |
| numbers.json, paths_2hz.csv | the numbers above; the three paths at 2 per second |
| t4_score.json / .txt, marks.txt, forced_drop.out | the rehearsal's scoring, its time marks and the drop log (copies) |
