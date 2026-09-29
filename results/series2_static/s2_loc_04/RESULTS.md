# Whole-floor localisation, session s2_loc_04 - results pack

*27 Sept 2026, Hamilton time. Start 2 began 07:43:12, park 08:06:47-08:07:56 (last picture processed 08:07:51). Lab 2nd floor, robot + Jetson. **Localisation** = the robot is given drive 10's saved map of the whole floor (`~/slam_series2/localise/s2_static_10_map.db`, Jetson internal disk; map-building switched off) and must work out where it is on it from what the camera sees. It was told it was 40 m OUTSIDE the building (the off-map starting guess (40, 40)), so every fix had to come from real recognition. The user drove the floor slowly, the same way round as drive 10, and parked on the start mark. Raw recordings: `~/.run_records/s2_loc_04/` (Jetson internal disk). Pass lines registered BEFORE the session: `03_methods/localisation_demo_2026-09-26/PASS_LINES_whole_floor.md` (W1-W4), scored here exactly as written. **N = 1 session: counts, never percentages.** Nothing here is ground truth (ENGINEERING_NOTES.md rule 19). Made on the Jetson by `03_methods/results_packs_2026-09-26/loc04_*.py`, which read only the pack's own small files after one extraction pass over `fusion.bag`.*

*Words used below: **fix** = RTAB-Map (the mapping program) accepting where the robot is on the map, after its 3D check (at least 20 matching 3D points between the new picture and a map picture); **recognised again** = a fix found by recognising the whole place from its picture; **nearby re-match** = a fix found by comparing the picture with map pictures close to where the robot already believed it was; **walkway end** = a corner or the end of a corridor, where the robot turned or reversed; **wheels + gyroscope** = the robot's own position estimate from its wheel turns and its turn-rate sensor (`/odometry/filtered`, as it reached the Jetson over WiFi); **blend** = `/fused/odometry`, the Jetson's mix of camera, wheels and gyroscope; **WiFi gap** = a stretch when the robot's data did not reach the Jetson.*

## 0. Answer first

**W1 FAIL, W2 PASS, W3 PASS, W4 FAIL.**

*Plain terms: switched on and told it was outside the building, the robot took 212 s and 6.3 m of driving to recognise where it was - too slow for the two-minute, three-metre target. Once it had recognised its place, it kept recognising places all round the floor (576 fixes, 23.4 per 10 m, at all 3 walkway-end stops), and not one of its fixes put it in a wrong place by the robot's own wheels. Whether its position was accurate to half a metre waits for the LiDAR comparison.*

## 1. The four registered pass lines

| # | pass line (registered before the session) | measured | verdict |
|---|---|---|---|
| W1 | **finds itself at the start**: first fix within **120 s** of the start and within **3 m** of driving | first fix **07:46:44 = 212.1 s** after start 2, after **6.26 m** by the robot's wheels (6.07 m by the blend, the live watcher's figure). A place recognised again (map node 167, 122 matched 3D points), 0.14 m from drive 10's path | **FAIL** (both halves) |
| W2 | **keeps knowing along the way**: at all but one of the 30 s walkway-end stops, a fix during the stop or in the last 10 m before it | **3 of 3** stops (stop 1 34.1 s at the east end of the top corridor, stop 2 29.9 s at the east end of the top corridor, stop 3 19.2 s at the bottom-right corner); needed: at least 2 of 3 | **PASS** |
| W3 | **no wrong places**: zero fixes more than **1.5 m** from where the robot's own wheels + gyroscope say it moved since the previous fix | **0 of 575** fixes over 1.5 m. 502 checked against the wheels + gyroscope: median 0.022 m, 95th percentile 0.101 m, largest **1.08 m** (08:02:28, after 10.8 m with no fix); 73 fell inside WiFi gaps and were checked against the blend instead: largest 0.069 m | **PASS** |
| W4 | **where it thinks it is, vs the LiDAR**: median <= 0.50 m (SE(3), no scale) | median **0.636 m**, 95th percentile 1.125 m, largest 1.234 m, 1166 matched positions; yardstick valid: True (`w4_agreement.json`) | **FAIL** |

What each line means in plain terms, and what passing or failing says:

- **W1** asks whether the robot realises "I know this place" within two minutes and three metres of being switched on somewhere it was told it isn't. *Failing means the robot needs a longer run-in than that before its position can be used.* For information (not the scored start): **start 1**, on the same mark 4 minutes earlier and also given the off-map guess, was accepted about **2.8 s** after its start with **0.0 m** driven (DIAG, section 6: the moment is inferred from RTAB-Map's rule, because the recording begins 18 s into start 1; the live watcher's 34.8 s is when it first looked). So the same mark gave one start inside W1 and one far outside it. Start 2 was begun at 07:43:12 with a fresh copy of the map; the whole-floor drive belongs to start 2.
- **W2** asks whether it keeps recognising places all round the floor, not only near the start. *Passing means it did at the stops it was asked to make; a miss would mark a part of the floor where it drove on memory alone.*
- **W3** asks whether any fix ever put it in the wrong corridor. *Passing means no fix disagreed with the robot's own wheels by more than 1.5 m; a wrong place would show as a jump the wheels did not make.*
- **W4** asks whether its positions agree with an independent LiDAR estimate about as well as the mapping drives did (drives 4/8: 0.24-0.25 m, drive 9: 0.42 m, drive 10's map: 0.69 m median). *It is the only line that measures accuracy rather than consistency.*

**How the numbers were made, and how sure they are (ENGINEERING_NOTES.md section 4 rule 0)**

- *Fix* = a picture on `/rtabmap/info` whose accepted recognition id or nearby re-match id is set; RTAB-Map sets these only after the 3D check passed. Smallest number of matched 3D points among the 576 fixes: 21 (minimum 20).
- *W1 timing*: 212.1 s is the picture time of the accepted fix on `/rtabmap/info`; the live watcher logged it at 213.1 s (it polls every 2 s). Distance: 6.26 m by the robot's wheels, 6.07 m by the blend. The wheels read about 6 % long (15 m tape test, drive `s2_scale_01`: 15.94 m for 15.00 m), so the true distance is nearer 5.9 m. Either way it is about twice the 3 m line, and 212 s is 1.8 times the 120 s line: no reading error changes the verdict.
- *W2 stops* were found from the wheel speed (still >= 15 s within 3 m (along the path) of a walkway end (travel direction turns >= 60 deg), after driving began, not the park). Inside WiFi gaps, where the wheel data never arrived, stillness was read from the Jetson side instead (the blend moving < 0.02 m over 1 s and the camera's own gyroscope < 2 deg/s); stops 2 and 3 each fall partly in a gap. A *walkway end* is where the direction of travel on the localised path turns by 60 degrees or more (2 m before vs 2 m after): 15 found (table in section 3). *The user was asked for 30 s stops at every walkway end; the robot stood 15 s or more at only 3 of the 15 (the others: 0-11 s), so W2 scores those 3.* 15 s (half the 30 s asked for) is a threshold chosen here, after the session; the stops' own lengths fall into two clear groups (the longest walkway-end stop not counted is 11.4 s, the shortest counted 19.2 s), so any threshold between those two gives the same stops. "10 m before" is by the wheels (about 9.4 m true).
- *W3*: each fix is compared with the previous fix carried forward by the robot's own wheels + gyroscope motion between the two picture times. When a fix's picture time lies inside a WiFi gap the wheels' position at that moment is unknown (only the gap's two ends are), so that fix is checked against the blend instead, and the next fix outside the gap is compared with the last fix before it. The wheels' own error grows with the distance since the previous fix - about 6 % of it from the size error plus the gyroscope's heading drift (about -5 deg a minute when parked, earlier sessions) - so a disagreement is only meaningful against that: the largest, 1.08 m after 10.8 m without a fix, is about 1.7 times the 6 % size error alone (0.65 m) and still under the 1.5 m line; the typical fix is 1-3 m after the previous one, where the disagreement is 0.022 m (median).
- The first fix of start 2 (the 57 m jump from the (40, 40) guess onto the map) has no previous fix and is W1's subject, not W3's. The live watcher called it "suspect jump 57 m" - it is the first fix itself (DIAG section 6).

## 2. Timeline (Hamilton time, 27 Sept 2026)

| time | seconds after start 2 | what happened | source |
|---|---|---|---|
| 07:39:23 | - | start 1 on the start mark, off-map guess (40, 40) | raw_small/starts.csv |
| about 07:39:26 | - | start 1 accepted where it was (about 2.8 s, 0.0 m) | DIAG section 6 (inferred) |
| 07:43:12 | 0 | **start 2**: fresh copy of drive 10's map, off-map guess (40, 40); robot on the start mark facing down the top corridor | raw_small/starts.csv |
| 07:43:13 | 0.4 | the right place already recognised (map node 138, 97 matched points) but HELD - RTAB-Map waits for a second recognition scoring 0.08 | DIAG section 1 |
| 07:43:12-07:44:53 | 0.0-101.1 | standing still at the start (101.1 s) | stops.csv |
| 07:44:57-07:46:10 | 105.4-178.0 | standing still at the start (72.6 s) | stops.csv |
| 07:46:14 | 182.6 | driving began (wheels 1 m from the start) | numbers.json |
| **07:46:44** | **212.1** | **first fix**: map node 167 recognised again, 122 points; the robot jumps from (40, 40) onto the map, 0.14 m from drive 10's path; 6.26 m driven | fixes.csv, numbers.json |
| 07:46:52-07:47:05 | 220.1-233.4 | stood still 13.3 s mid-corridor at x 7.2 m on the top corridor (not a walkway end; the reason is not in the recordings) | stops.csv |
| 07:47:18-07:49:50 | 246.0-398.4 | stood still 152.4 s mid-corridor at x 9.2 m on the top corridor (not a walkway end; the reason is not in the recordings) | stops.csv |
| 07:50:21 | 428.7 | walkway end E1, east end of the top corridor; **W2 stop 1**: 34.1 s (07:50:31-07:51:05); fix there or in the 10 m before | walkway_ends.csv |
| 07:52:06 | 534.1 | walkway end E2, bottom-right corner; fix there or in the 10 m before | walkway_ends.csv |
| 07:53:16 | 603.9 | walkway end E3, west end of the bottom corridor (a dead end); fix there or in the 10 m before | walkway_ends.csv |
| 07:53:35 | 623.3 | walkway end E4, bottom-left corner; fix there or in the 10 m before | walkway_ends.csv |
| 07:54:23 | 670.8 | walkway end E5, start-mark corner; stood 8.4 s (07:54:23-07:54:31); fix there or in the 10 m before | walkway_ends.csv |
| 07:55:58 | 765.9 | walkway end E6, east end of the top corridor; **W2 stop 2**: 29.9 s (07:55:21-07:55:51); fix there or in the 10 m before | walkway_ends.csv |
| 07:56:34 | 802.0 | walkway end E7, north end of the north corridor; stood 11.4 s (07:56:42-07:56:53); fix there or in the 10 m before | walkway_ends.csv |
| 07:57:17 | 845.2 | longest WiFi gap: 32.8 s without the robot's data, starting near map (14.7, 3.8) | robot_data_gaps.csv |
| 07:58:50 | 938.6 | walkway end E8, small bend east of the bottom-right corner; fix there or in the 10 m before | walkway_ends.csv |
| 08:00:17 | 1024.8 | walkway end E9, west end of the bottom corridor (a dead end); stood 10.0 s (08:00:02-08:00:12); fix there or in the 10 m before | walkway_ends.csv |
| 08:01:20 | 1088.3 | walkway end E10, start-mark corner; fix there or in the 10 m before | walkway_ends.csv |
| 08:01:45-08:02:28 | 1113.3-1156.3 | longest stretch without a fix: 10.8 m, 43 s, the left corridor (x = 0) driven south | numbers.json |
| 08:02:27 | 1155.5 | walkway end E11, bottom-left corner; stood 6.2 s (08:02:17-08:02:23); **no fix there or in the 10 m before** (10.25 m since the last) | walkway_ends.csv |
| 08:03:19 | 1207.6 | walkway end E12, bottom-right corner; **W2 stop 3**: 19.2 s (08:03:18-08:03:37); fix there or in the 10 m before | walkway_ends.csv |
| 08:04:54 | 1302.5 | walkway end E13, north end of the north corridor; fix there or in the 10 m before | walkway_ends.csv |
| 08:05:33 | 1341.1 | walkway end E14, east end of the top corridor; fix there or in the 10 m before | walkway_ends.csv |
| 08:06:15 | 1383.6 | walkway end E15, start-mark corner; fix there or in the 10 m before | walkway_ends.csv |
| 08:06:47-08:07:56 | 1415.2-1484.2 | **park** on the start mark: the localiser puts the robot 0.051 m from the start mark (section 4) | stops.csv, numbers.json |
| 08:07:51 | 1479.3 | last picture processed; session stopped | pictures.csv |

## 3. Keeping its place along the way (reported, not judged)

| item | value | file |
|---|---|---|
| time not localised / localised (start 2) | **212.1 s / 1267.2 s**; never lost again after the first fix (0 unlocalised messages after it) | numbers.json |
| distance driven, start 2 | 252.8 m by the wheels (246.5 m after the first fix); 245.8 m by the blend; 234.6 m along the localised path | numbers.json |
| fixes | **576** = 506 nearby re-match + 70 recognised again (69 of these also a nearby re-match) | fixes.csv |
| fixes per 10 m (after the first fix) | **23.4**; the 25 pieces of 10 m: none without a fix, fewest 6, median 24 | numbers.json, fixes_per_10m.png |
| longest stretch without a fix, by distance | **10.76 m, 43.0 s** (08:01:45-08:02:28), from map (0.1, -4.8) to (0.1, -14.4): the left corridor (x = 0) driven south. Drive 10 drove this corridor southbound twice (camera_corrected.tum), so these views are on the map; why no fix came is not shown by the recordings | numbers.json |
| longest stretch without a fix, by time | 153.2 s (07:47:18-07:49:51) but only 0.17 m: the robot was standing still mid-corridor (pictures taken standing still are not added to RTAB-Map's short memory, DIAG section 2) | numbers.json |
| stretches of more than 5 m without a fix | 4 (more than 10 m: 1): 7.9 m from 07:53:04; 6.7 m from 07:58:28; 6.0 m from 07:59:54; 10.8 m from 08:01:45 | numbers.json |
| walkway ends with a fix there or in the 10 m before | **14 of 15** (missed: E11 bottom-left corner, 10.25 m since the last fix) | walkway_ends.csv |
| WiFi dropouts (start 2) | **31** gaps over 0.5 s in the robot's data, **141.0 s** in all, 6 over 5 s, longest 32.8 s from 07:57:17; the live view may have paused then - the scoring uses on-board recordings only | robot_data_gaps.csv, bridge_status.csv |
| processor, power, temperature (07:39:23-08:07:57) | power peak 36.1 W, median 30.3 W; chip 62.8 C; graphics load median 22 %, peak 99 %; free memory never below 48.0 % | power_temperature.json |
| drive 10's map unchanged | checksum `98f71ed1...` the same after the session (re-computed by this pack, 27 Sept 10:01 Hamilton, against raw_small/master.sha256); the last working copy had 1558 nodes = drive 10's map. `P4_map_untouched.txt` says "nodes master=540": that count is drive 4's map, because `stop_localise.sh` line 22 defaults to drive 4's map when not told otherwise - the checksum it passed is drive 10's (suggestion only, nothing changed) | raw_small/P4_map_untouched.txt, raw_small/master.sha256 |

**Every walkway end** (a place where the direction of travel turned by 60 degrees or more; E = end, in driving order):

| end | time | where (drive 10's map, m) | which | stood still there | fixes there | fixes in the 10 m before | a fix? |
|---|---|---|---|---|---|---|---|
| E1 | 07:50:21 | (14.7, 0.1) | east end of the top corridor | 34.1 s (**W2 stop 1**) | 3 | 37 | yes |
| E2 | 07:52:06 | (14.7, -15.0) | bottom-right corner | - | 6 | 19 | yes |
| E3 | 07:53:16 | (-4.3, -14.4) | west end of the bottom corridor (a dead end) | - | 0 | 18 | yes |
| E4 | 07:53:35 | (0.5, -14.5) | bottom-left corner | - | 7 | 6 | yes |
| E5 | 07:54:23 | (0.4, -0.1) | start-mark corner | 8.4 s | 5 | 18 | yes |
| E6 | 07:55:58 | (14.4, 0.2) | east end of the top corridor | 29.9 s (**W2 stop 2**) | 3 | 24 | yes |
| E7 | 07:56:34 | (14.6, 9.8) | north end of the north corridor | 11.4 s | 9 | 30 | yes |
| E8 | 07:58:50 | (15.9, -13.2) | small bend east of the bottom-right corner | - | 3 | 14 | yes |
| E9 | 08:00:17 | (-1.6, -14.3) | west end of the bottom corridor (a dead end) | 10.0 s | 3 | 23 | yes |
| E10 | 08:01:20 | (0.7, 0.1) | start-mark corner | - | 6 | 29 | yes |
| E11 | 08:02:27 | (0.1, -15.1) | bottom-left corner | 6.2 s | 0 | 0 | **no** |
| E12 | 08:03:19 | (14.7, -14.7) | bottom-right corner | 19.2 s (**W2 stop 3**) | 4 | 19 | yes |
| E13 | 08:04:54 | (14.8, 10.0) | north end of the north corridor | - | 6 | 22 | yes |
| E14 | 08:05:33 | (14.4, -0.1) | east end of the top corridor | - | 6 | 27 | yes |
| E15 | 08:06:15 | (0.6, -0.1) | start-mark corner | - | 2 | 24 | yes |

*Plain terms: the robot recognised where it was about every 43 cm on average, in every 10 m piece of the floor, and at 14 of the 15 corners and corridor ends. The miss was a corner reached at the end of its longest stretch of 10.8 m without a fix.*

## 4. Start-to-end gaps, both kinds (ENGINEERING_NOTES.md rule 20)

In localisation the camera map is drive 10's and is not changed (section 3, last row), so there is no new graph to measure. What can be said about the park, with the kind of each number:

| kind | source | park vs start | file |
|---|---|---|---|
| corrected by recognition | the localiser's own position while parked (median of the 69.0 s park), on drive 10's map, against drive 10's first position = the start mark | **0.051 m** ((0.03, -0.04) m, -1.4 deg); fixes: 1 during the park, 35 in the 10 m before it | numbers.json, stops.csv |
| tracking alone | robot's wheels + gyroscope, as received over WiFi, start 2 to the end | 1.69 m, -8.4 deg | numbers.json |
| tracking alone | blend (`/fused/odometry`), start 2 to the end | 0.63 m, -0.4 deg | numbers.json |

*The robot started and parked on a tape mark placed by eye; tape-mark parking is only good to about 0.2-0.5 m, so the 0.051 m says the localiser's park position is consistent with the mark, not that it is accurate to centimetres.*

## 5. Why the first fix took 212 s - diagnosis summary, and the procedure change

From `03_methods/localisation_demo_2026-09-26/whole_floor_diag/DIAG.md` (done before this pack; read-only; labels as there). Also in `docs/SOLVED.md`.

- **No check refused anything.** The camera recognised the right place 0.4 s after start 2 (map node 138, 0.28 m from the start mark, 97 matched points). RTAB-Map's localisation mode never acts on one recognition: it holds it and waits for a SECOND recognition scoring at least **0.08** (`Rtabmap/LoopThr`) while the first is still among the last 10 positions it remembers (`RGBD/MaxOdomCacheSize` 10). [VERIFIED, Rtabmap.cpp 0.21.13]
- **Standing still froze that wait.** The robot stood about 178 s. Pictures taken standing still are not added to the short memory, so the held recognition never aged out; none of the 164 standing-still pictures reached 0.08 (median 0.050, highest 0.069). [VERIFIED]
- **Driving built the score up slowly**: three more correct recognitions were held and dropped in turn (nodes 489, 496, 167); at 212.1 s a picture scored 0.0827 while node 167 was held, and the fix was accepted after 6.1 m.
- **No safe setting would have passed W1 here.** Re-scoring RTAB-Map's own recorded scores (validated: it reproduces the session exactly) shows that with the 0.08 bar the earliest possible fix was 212.1 s whatever the memory size or movement thresholds. Only a lower bar (0.065 or 0.06) or accepting a lone recognition would have passed - and those gave fixes OFF the map in the earlier offline test (2 and 3 of 20 stretches), so they stay rejected.
- **Three watcher misreadings corrected**: start 1 was ALSO an off-map start, accepted at about 2.8 s in RTAB-Map's log (not 34.8 s); the "suspect jump 57 m" is the first fix itself; only 4 of the 109 "pending" recognitions came before the first fix (the other 105 were links held while already localised).

**The procedure change (no RTAB-Map setting changes):** at each start, stand still for at most **60 s**; if RTAB-Map has not accepted a fix by then (the page still says "LOCALISED no"), drive on at once along drive 10's route at the usual slow speed.

- *Why:* it keeps every wrong-place guard (0.08 bar, two recognitions, 20 points, off-map guess) and removes the only part of the wait that could not produce a fix. Every start in sessions 3 and 4 that was accepted while standing was accepted within 60 s (2.8, 12.0, 34.9, 57.2 s).
- *What to expect* [INFERENCE, N = 1]: the time half of W1 passes (a start like this one would have been fixed at about 94 s); the **distance half still fails** for a start like this one (the first picture over 0.08 came after 5.8 m of driving). Passing W1 reliably needs higher scores for the right place without higher scores for wrong places; nothing tested so far does that.
- *To make the next session replayable offline*: record the camera (`SVO=1` in `start_drive.sh`) or keep `Mem/LocalizationDataSaved=true` plus the working copy - this session's pictures were not kept, so no setting that changes the scores can be tested on it.

## 6. Figures and video in this folder

| file | what it shows |
|---|---|
| `floor_fixes.png` | **the summary figure**: drive 10's floor (walls from its cleaned 3D map, seen from above), drive 10's route (the map, solid grey), where the robot thought it was (dashed), every fix coloured by kind (teal = nearby re-match, rust = recognised again, gold star = first fix), every walkway end E1-E15 (rust ring = no fix there or in the 10 m before), the three W2 stops (numbered squares), the stretches of more than 5 m without a fix (rust bands) |
| `timeline.png` | the session against the clock: wheel speed with the stops and WiFi gaps; every fix by kind; metres driven since the last fix against W2's 10 m; the W3 check of every fix against its 1.5 m line |
| `fixes_per_10m.png` | fixes in each 10 m of driving after the first fix |
| `start2_first_fix_diagnosis.png` | copied from `whole_floor_diag/start2_timeline.png`: the first 300 s of start 2 - score against the bar, the held recognitions, 3D-check points, distance against the W1 box |
| `timelapse.mp4` | **recorded live during the session**: the live map page every 3 s, 591 frames at 10 a second = 59.1 s (copied unchanged from `~/.run_records/s2_loc_04/media/s2_loc_04_timelapse.mp4`, md5 2e0a0e6d...; recorder: missed 0, write errors 0). The left panel is drive 10's saved map: it does not grow, the robot's marker moves on it. The last ~2 minutes show "NO DATA" because the recorder waits 120 s after the mapping program stops |
| `timelapse_25/50/75/100.png`, `timelapse_park.png` | stills at 25/50/75/100 % of the video (frames 147, 295, 443, 590 counted from 0: 07:47:48, 07:55:12, 08:02:39, 08:10:01 - the last is after the session, "NO DATA") and at the park (frame 546, 08:07:49) |
| `timelapse_live_index.csv` | the recorder's per-frame index (Hamilton time, page status) |

## 7. W4 - how it was scored

**Status: SCORED 27 Sept 12:30 Hamilton.** The LiDAR Force3DoF replay (the colleague's LiDAR mapping replayed with the single change `Reg/Force3DoF=true`, as for drives 9 and 10) ran ON THE JETSON (`03_methods/lidar_replay_on_jetson_2026-09-27/`; validated against the robot on drive 10 - camera-vs-LiDAR 0.675/1.132 m against the robot's 0.685/1.151 m). The first Jetson attempt was cut at 46 s by the 11:32 WiFi outage and power cycle; the re-run from the same sha256-checked recording finished 12:24. *Plain terms: the LiDAR replay gives an independent second opinion of where the robot drove; W4 asks whether the localiser's positions agree with it to within half a metre (median).*

**Prepared, so W4 needs no raw file from the Jetson run folder:**

- `localised.tum` - the localised camera path: start 2 from its first fix (07:46:44) to the end, 1229 positions, `/rtabmap/localization_pose` (the robot's base in drive 10's map frame), Jetson clock, TUM format. Extracted from `fusion.bag` by `loc04_extract.py` (`extract_meta.json`).
- `loc04_w4.py` (in `03_methods/results_packs_2026-09-26/`) - computes W4 from files alone, with the same method as every drive's LiDAR comparison (`compare_lidar.py`: LiDAR nodes filled in with the robot's 50 Hz wheels, `evaluate_trajectory.py`, reference kind cross_stack, 50 ms matching, SE(3) alignment, no scale). It reports the yardstick's own checks beside the number (ends within 0.5 m of its start with at least one LiDAR loop closure; tilt under 3 deg and height range under 0.3 m); if the yardstick fails them, W4 is "not scorable" (the session-3 precedent). **Known-answer test PASSED** (`w4_selftest.json`): run on drive 10 with drive 10's corrected camera path in place of the localised path, it gives median 0.685 m, 95th percentile 1.151 m, largest 1.242 m, 860 matched - drive 10's `agreement.json` says 0.685 / 1.151 / 1.242 m, 860.
- `raw_small/bridge_clock.csv.gz`, `raw_small/bridge_events.log` - the WiFi bridge's clock measurements (robot vs Jetson clocks: at the end the bridge measured the robot's clock 1.6 ms off the Jetson's and corrected for it - negligible against the 50 ms matching), in case the time match needs checking.

**The exact commands** (robot offline at pack time, 27 Sept ~10:00 Hamilton, so not tried. The replay script reads `/media/administrator/USB Drive/slam_series2/s2_loc_04_lidar.bag` - its own naming rule, line 115; that this file exists on the robot's card is [UNVERIFIED] until the robot is back. The robot is the colleague's computer and the ROS master for the wheels: ask before running anything there, ENGINEERING_NOTES.md rule 11):

```bash
# 1a. on the ROBOT (as for drive 10), from the robot card, the robot parked and not recording:
"/media/administrator/USB Drive/slam_series2/tools/replay_colleague_variant.sh" s2_loc_04 wrap f3dof
#     watch it from the Jetson (state changes, failures, silent death; mirrors ~/jobs for the :8096 page):
"<repo>/03_methods/lidar_drift_2026-09-26/watch_f3dof.sh" s2_loc_04
#     then pull the small products into this pack (Jetson; -s because the path has a space; the map database stays on the card):
rsync -a -s --exclude '*.db' robot:"/media/administrator/USB Drive/slam_series2/s2_loc_04/lidar_ref_f3dof/" "<repo>/results/series2_static/s2_loc_04/lidar_ref_f3dof/"
#     and check the copy (the two lists must match):
ssh robot 'cd "/media/administrator/USB Drive/slam_series2/s2_loc_04/lidar_ref_f3dof" && sha256sum lidar.tum wheel.tum' ; (cd "<repo>/results/series2_static/s2_loc_04/lidar_ref_f3dof" && sha256sum lidar.tum wheel.tum)
# 1b. OR on the JETSON, once the Jetson replay has been checked against the robot's (03_methods/lidar_replay_on_jetson_2026-09-27/):
#     ./pull_bag_from_robot.sh s2_loc_04 && ./replay_colleague_variant_jetson.sh s2_loc_04 wrap f3dof
#     and copy ~/lidar_replays/s2_loc_04/lidar_ref_f3dof/{lidar.tum,lidar.tum.meta.json,wheel.tum,wheel.tum.meta.json,lidar_map.*,provenance.txt,products.log} into the pack's lidar_ref_f3dof/
# 2. score W4 (Jetson, files alone; writes w4_agreement.json and w4/):
cd "<repo>/03_methods/results_packs_2026-09-26" && nice -n 19 python3 loc04_w4.py "<repo>/results/series2_static/s2_loc_04"
# 3. rewrite this RESULTS.md with the W4 row filled in:
nice -n 19 python3 loc04_write_results.py "<repo>/results/series2_static/s2_loc_04"
```

**W4 result** (`w4_agreement.json`): median 0.636 m, 95th percentile 1.125 m, largest 1.234 m over 1166 matched positions (94.9 % of the localised path); yardstick: 863 LiDAR nodes, 102 loop closures, ends 0.05 m from its start, tilt at most 0.0 deg, height range 0.00 m; data's own best clock shift +0.45 s (applied: +0.00 s). Verdict: **FAIL**. Agreement with an independent LiDAR estimate, not error against ground truth.

**Why W4 failed, and what it does and does not mean** (explanation added after scoring; the verdict stands as registered):
- The localiser can only be as right as the map it localises on. Drive 10's own corrected map, compared with its own LiDAR replay the same way, gives median **0.685 m** (`w4_selftest.json`). The localised path gives **0.636 m**: it agrees with the LiDAR as well as the map it uses, and no worse.
- The error is mostly SIZE, as for drive 10: the best size fit is **1.068** (drive 10: 1.072), i.e. the camera map is about 7 % larger than the LiDAR's. With that size fitted, for information only (never the scored number, ENGINEERING_NOTES.md section 4 rule 4), the median falls to **0.20 m**, largest 0.62 m (`evo_ape -as`).
- The pass line 0.50 m was set from drives 4/8/9 before drive 10's map-vs-LiDAR figure was known. It is kept; W4 is a FAIL, caused by the map's scale, not by the localiser losing its place (W3: 0 wrong fixes of 575).
*Plain terms: the robot knew which corridor and where along it it was, but the map it was given is stretched by about 7 %, so every position is off by up to a metre at the far ends. Fix the map's size and the localiser's positions come within about 20 cm.*

## 8. What is in this folder, and which raw files the pack still needs

| file | what |
|---|---|
| `numbers.json` | every number above (W1-W3, report-also) - `loc04_score.py` |
| `localised.tum` | the localised path for W4 (section 7) |
| `w4_selftest.json` | the W4 scorer's known-answer test on drive 10 (section 7) |
| `fixes.csv` | the 576 fixes: time, kind, map ids, matched points, position, W3 disagreement |
| `fixes_w3.csv` | the W3 check of every fix after the first: which fix it was compared with, by what (wheels or blend), how far the robot moved, the disagreement |
| `stops.csv` | every still stretch of 5 s or more, where, how long, walkway end, W2 result |
| `walkway_ends.csv` | the 15 walkway ends and whether a fix came there or in the 10 m before |
| `pictures.csv` | every picture RTAB-Map processed (1553, both starts): fix ids, matched points, score, localised position, uncertainty |
| `localisation_pose_all.csv` | every localisation message (1552, both starts) with its uncertainty |
| `robot_wheels_gyro_10hz.tum` | the robot's wheels + gyroscope as received (10 Hz, both samples kept around every gap) |
| `robot_speed_10hz.csv`, `jetson_side_10hz.csv` | wheel speed + gyroscope; the blend's position + the camera's gyroscope (for stillness inside WiFi gaps) |
| `robot_data_gaps.csv`, `bridge_status.csv` | the WiFi gaps and the bridge's state once a second |
| `extract_meta.json` | what was extracted from `fusion.bag` (size, time, counts) |
| `floor_background.npz` | the floor picture's walls: drive 10's cleaned 3D map, wall-height points per 5 cm cell (from `s2_static_10/map_3d_clean/`) - `loc04_background.py` |
| `power_temperature.json` | processor, power, temperature over the session (from tegrastats.log) |
| `raw_small/` | copies of every small file of `~/.run_records/s2_loc_04/` (watcher summary/events/track, starts, camera guard, monitor, bridge, camera and fusion logs, TF checks, P4 line, master checksum, recorder log) plus `mapping.log.gz` and `tegrastats.log.gz` (gzip, checked: unpacks to the original's sha256) and the bridge clock files from `~/jobs/`. `raw_small/MANIFEST.txt` lists each original with its size and sha256 |
| figures and video | section 6 |

**Raw files the pack still needs: none.** Everything the pack, the figures and W4 need is extracted into this folder. `~/.run_records/s2_loc_04/fusion.bag` (2.9 GB) was read once and is not needed again; the timelapse's single frames (`media/cam/`, `media/lidar/`, `media/frames_*/`) are not kept, by design (the video is here). W4's remaining inputs are NOT on the Jetson: they come from the robot's LiDAR recording (`s2_loc_04_lidar.bag` on the robot's card) via the replay in section 7. The run folder `~/.run_records/s2_loc_04/` can therefore leave the Jetson; its copy is in the Autonomous Service Robot Teams folder (`series2_raw/s2_loc_04/`).

## 9. Still open

- **W4**: waits for the LiDAR Force3DoF replay (section 7).
- W1's distance half: no tested change passes it for a cold start in the long corridor (section 5).
- Why the left corridor (x = 0), driven south, gave no fix for 10.8 m although drive 10 drove it southbound twice: not answerable from this session's recordings (its pictures were not kept).
- The 152 s stop mid-corridor at 07:47:18 is not explained by the recordings.
