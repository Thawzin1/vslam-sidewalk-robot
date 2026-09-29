# Localisation session 3 (s2_loc_03) - results pack

*26 Sept 2026, 10:41-11:06 Hamilton time, lab, robot + Jetson. Localisation = the robot is put down somewhere on a map it built earlier (drive 4's map) and must work out where it is on it, without drawing a new map. This pack is a SUMMARY: the scoring, the diagnosis and every number are in the method folder `RESULTS.md` (project records), section "Session 3" (Jetson internal disk). Nothing is re-derived here; the verdict table below is copied from there exactly. Nothing here is ground truth (ENGINEERING_NOTES.md rule 19). N = 5 starts, 1 session.*

*Words used below: fix = RTAB-Map (the mapping program) accepting where the robot is; LoopThr = `Rtabmap/LoopThr`, the score a recognised place must reach before it is used; wall fit v2 = the registered way of lining this session's LiDAR map up with drive 4's (trimmed, with a stability check over 8 part-fits); K1-K3, P1-P5 = the pass lines written before the session in `03_methods/localisation_demo_2026-09-26/PASS_LINES.md`.*

## Answer first (copied from the method RESULTS)

From a deliberately wrong guess 40 m off the map, RTAB-Map accepted where it was at **3 of 5 starts within 2 minutes** (4 of 5 in all), and it **never showed a wrong place**. That is **short of K1's 4 of 5**. P2, P3, K2 and K3 are **not scorable this session**, because the LiDAR reference failed its own registered check.

*Plain terms: the robot was told "you are somewhere far away" on purpose. It stayed visibly off the map until it recognised where it really was, then jumped to the right place. It did that at 4 of the 5 spots, 3 of them fast enough. Whether its position was accurate to 0.30 m cannot be said, because the LiDAR estimate used as the comparison was itself bent this session.*

## Verdict table (copied exactly from the method RESULTS, section "Session 3")

| # | pass line | session 3 | verdict |
|---|---|---|---|
| 0a | **LiDAR drift, primary check** (wall fit v2: 8 part-fits agree within 0.10 m and 1.0 deg) | largest deviation **0.126 m** (limit 0.10); rotations within 0.83 deg; west half vs east half **0.171 m / 0.56 deg**; random halves within 0.03 m of each other | **FAIL**: the LiDAR reference failed its own check, so it is not used to score this session |
| 0b | LiDAR drift, secondary check (tape repeat visits, limit 0.70 m) | start mark, start vs park: **0.33 m** (also the LiDAR map's own start-to-park gap; 0.40 m after correction in the meta file). Spot A, start 2 vs start 4: **0.30 m** | within 0.70 m. This proves little: parking is only +-0.5 m |
| K1 | at least 4 of 5 starts accepted within 120 s and 3 m | start 1: 34.9 s, 0.0 m. Start 4: 57.2 s, 0.33 m. Start 5: 12.0 s, 0.38 m. Start 2: 380.4 s, 5.04 m (too late). Start 3: none | **FAIL, 3 of 5** |
| K2 | position at the fix within 0.30 m of the LiDAR-derived position | for information only: start 1 0.06 m, start 2 0.25 m; starts 4 and 5 have no LiDAR pose at the fix moment | **not scorable this session** |
| K3 | no accepted fix more than 1.0 m from the LiDAR-derived position | for information only: 83 fixes scored, **0 over 1.0 m**, largest 0.31 m (start 5) | **not scorable this session** |
| P1 | at least 4 of 5 starts with a first recognition event within 120 s and 3 m | starts 1 (84.5 s), 4 (57.5 s), 5 (12.3 s) | **FAIL, 3 of 5** |
| P2 | within 0.30 m of the LiDAR place, read when parked | for information only: start 1 0.09 m, start 2 0.26 m | **not scorable this session** |
| P3 | no recognition event more than 1.0 m from the LiDAR place | for information only: largest 0.31 m | **not scorable this session** |
| P4 | master map unchanged | checksum unchanged, 540 nodes | **PASS** |
| P5 | median processing time under 1.0 s | 1,237 pictures: median **0.682 s**, 95th percentile 0.811 s, 9 over 1.0 s | **PASS** |

*"for information only" rows use the LiDAR estimate that failed its own check: they are agreement with a LiDAR estimate that is not usable as a yardstick this session, never error against ground truth.* Protocol deviations (late start at start 2, 10 cm nudges instead of the registered 2 m drive at starts 2 and 3, one refused first start after the robot's WiFi address changed, camera 0 restarts) and the tape's accuracy (about 20-50 cm) are listed in the method RESULTS.

## Why starts 2 and 3 were not accepted (one line; full text in the method RESULTS)

The camera recognised the right place at all five starts, but RTAB-Map needs a second recognition scoring at least 0.08 while the first is still remembered; starts 2 and 3 peaked at 0.081 (only at 380 s) and 0.076. A lower bar, 0.065, was then tested offline and gave fixes outside the map (2 of 20 stretches), so **0.08 stays** (method RESULTS, "LoopThr 0.065 offline test").

## Claim (copied from the method RESULTS, "What can honestly be claimed")

"In one lab session, starting from a deliberately wrong position 40 m off the map, the robot recognised where it was on a map it had built the day before at 4 of 5 starts, 3 of them within a minute. It never showed a wrong position: until it had recognised a place, it stayed visibly off the map." (N = 1 session, 5 starts.) Accuracy: say only that the accepted positions were within about half a metre of where the robot was parked (the tape, +-0.5 m); no 0.30 m accuracy claim is made.

## Start-to-end gaps (rule 20)

Not applicable to the camera: in localisation the camera map is drive 4's saved map and is not changed (P4 PASS), so there is no new camera graph to measure. The robot's LiDAR map of this session (colleague's `self_navigation` LiDAR mapping replayed on the robot) has 253 positions over 38.8 m with a corrected start-to-park gap of 0.334 m on the floor (`lidar_map_facts.json`), 23 loop closures and 0.398 m in 3D (`lidar.tum.meta.json`). That LiDAR map failed its own drift check (row 0a).

## Files in this folder

| file | what it shows |
|---|---|
| session3_starts.png | copied from `03_methods/localisation_demo_2026-09-26/session3_diag/`: the five starts on drive 4's map - LiDAR-derived path (solid, for information only), the localiser after its fix (dashed), first fixes (dots), provisional taped spots (squares) |
| timelapse.mp4 | **recorded live during the session**: the live map page (camera map left, LiDAR view right) every 3 s, 483 frames at 10 a second = 48.3 s (copied unchanged from `~/.run_records/s2_loc_03/media/s2_loc_03_timelapse.mp4`, Jetson internal disk, md5 dc5af251...; `recorder.log` there: missed 1, write errors 0). The left panel is drive 4's saved map, loaded for localisation: it does not grow, the video shows the robot's marker moving on it. The page's header labels ("recorded live during the drive", "map corrections ... so far") are the drive page's generic labels. The last ~2 minutes show "NO DATA" / "LiDAR view offline" because the recorder waits 120 s after the mapping program stops |
| timelapse_live_index.csv | the recorder's per-frame index (Hamilton time, page status) - `index.csv` from the same media folder |
| timelapse_25/50/75/100.png | stills at 25/50/75/100 % of the timelapse (video frames 120, 241, 362, 482, counted from 0; extracted with ffmpeg): 10:48:19, 10:54:51, 11:01:24, 11:07:46 Hamilton (index csv) |
| lidar.tum, lidar_map.*, wheel.tum, products.log, provenance.txt, replay*.log | the robot's LiDAR replay products (method RESULTS) |
| score_kidnap_fit_v2/ | the scores made by `score_kidnap.py` (method RESULTS) |

Not kept here, by design: the timelapse's single frames (they stay in `~/.run_records/s2_loc_03/media/` on the Jetson).
