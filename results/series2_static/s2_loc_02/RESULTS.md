# Localisation session 2 (s2_loc_02) - results pack

*26 Sept 2026, 06:16-06:34 Hamilton time, lab, robot + Jetson. Localisation = the robot is put down somewhere on a map it built earlier (drive 4's map) and must work out where it is on it, without drawing a new map. This pack is a SUMMARY: the scoring, the diagnosis and every number are in the method folder `RESULTS.md` (project records), section "Session 2", and `session2_diag/` (Jetson internal disk). Nothing is re-derived here; the verdict table below is copied from there exactly. Nothing here is ground truth (ENGINEERING_NOTES.md rule 19). N = 5 starts, 1 session.*

*Words used below: fix = RTAB-Map (the mapping program) accepting where the robot is; held link = a recognition RTAB-Map noted but did not use, waiting for a second one; P1-P5 = the pass lines written before the session in `03_methods/localisation_demo_2026-09-26/PASS_LINES.md`.*

## Answer first (copied from the method RESULTS)

**The robot did NOT find itself from nothing.** At 4 of 5 spots the map showed it 4-10 m from where it was, while the live page said "LOCALISED yes". The camera had in fact recognised the right place at every spot. Each time, RTAB-Map held that recognition back and never used it.

*Plain terms: after each restart the map put the robot on the start mark, wherever it really was. The camera did recognise the right place, but the program waited for a second, stronger recognition that never came, so the wrong position stayed on screen. Only start 4 was corrected, after 5.2 m of driving. This is what led to session 3's "start off the map" change.*

## Verdict table (copied exactly from the method RESULTS, section "Session 2")

| # | pass line | session 2 | verdict |
|---|---|---|---|
| P1 | 4 of 5 first fixes within 120 s and 3 m | recognition events (the registered definition) accepted by RTAB-Map: **start 1 only** (first recognition event: nearby re-match at 57.7 s, 0.09 m; this is the P1 figure. RTAB-Map had in fact already accepted a fix by 19.0 s at the latest: the recording began 18.6 s after the start and its first localisation message, at 19.0 s, already has uncertainty 0.001, not 9999. The watcher logged it at 34.3 s. Start 1 began from a correct guess, so it is the easy control either way). Start 4: 138.8 s, 5.2 m. Starts 2, 3, 5: none | **FAIL, 1 of 5** |
| P2 | within 0.30 m of the LiDAR-derived place | Fixed starts (recognition events): 1 and 4. **Start 1: not scorable.** 5 s after its fix the robot was being nudged, and the nearest LiDAR poses are 10 s before and 75 s after, 0.12 m apart (the registered limit is 0.10 m). At the fix moment itself it is 0.29 m. **Start 4: 0.48 m** (uncertainty 0.06 m, heading -0.7 deg). The scorer reads it at the first parked moment, 48 s after the fix, in the side corridor. At the fix moment it is 0.23 m. *Under the new fit (v2), for information:* start 1 still not scorable; start 4 0.40 m. See "LiDAR check" below | **not scorable in effect**: 1 of 2 fixed starts cannot be scored, and the other fails |
| P3 | no fix more than 1.0 m from the LiDAR place | **RTAB-Map's recognition fixes: 0 of 52 scorable (54 in all) over 1.0 m** (start 1: 39, median 0.36 m, largest 0.40 m; start 4: 13, median 0.22 m, largest 0.29 m). *Under the new fit (v2), for information:* largest 0.14 m (start 1), 0.12 m (start 4). **As the map showed it**, before any fix, the LiDAR confirms the positions were wrong by 5.5 m (start 2), 10.1 m (3), 5.6 m (4) and 4.0 m (5). Those were held cache links, not fixes | **PASS** on fixes (the registered definition). The positions shown before a fix were 4.0-10.1 m wrong |
| P4 | master map unchanged | checksum `8c1c1756...` identical, 540 nodes | **PASS** |
| P5 | median processing time under 1.0 s | 1,009 pictures: median **0.802 s**, 95th percentile 0.977 s, 24 over 1.0 s | **PASS** |

*The LiDAR distances above are agreement with the LiDAR estimate, not error against ground truth. That estimate (colleague's `self_navigation` LiDAR mapping replayed on the robot: 254 positions, 32.52 m, corrected start-to-park gap 0.148 m on the floor, `lidar_map_facts.json`; 1 loop closure, 0.179 m in 3D, `lidar.tum.meta.json`) passed its registered wall fit only just, and the fit is not stable along the corridor - each figure carries up to about 0.29 m of along-corridor ambiguity (method RESULTS, "LiDAR check").*

## What session 2 does and does not show (from the method RESULTS)

- **Does:** once RTAB-Map accepted a fix (start 4), it tracked the robot through about 6 m of driving on the saved map, agreeing with the blend's odometry (the camera + wheels + gyroscope mix) to about 0.3 m.
- **Does not:** that the robot finds itself from nothing - every reload started from a plausible-looking wrong guess (the start mark).
- The scorer run without LiDAR prints "P1 5 of 5"; that output is wrong for this session (it counts held links) and is not used.

## Start-to-end gaps (rule 20)

Not applicable to the camera: in localisation the camera map is drive 4's saved map and is not changed (P4 PASS). The LiDAR map's gap is in the note under the table.

## Files in this folder

| file | what it shows |
|---|---|
| timelapse.mp4 | **recorded live during the session**: the live map page (camera map left, LiDAR view right) every 3 s, 395 frames at 10 a second = 39.5 s. The original is in the Autonomous Service Robot Teams folder (`series2_raw/s2_loc_02/s2_loc_02/media/s2_loc_02_timelapse.mp4`), md5 3245c4e0... identical to it (the cloud `recorder.log`: missed 0, write errors 0). The left panel is drive 4's saved map, loaded for localisation: it does not grow. The page's header labels ("recorded live during the drive", "map corrections ... so far") are the drive page's generic labels. Its last frame falls after the mapping program stopped (LiDAR view HTTP 503 in the index) |
| timelapse_live_index.csv | the recorder's per-frame index (Hamilton time, page status), `index.csv` from the same cloud folder (md5 matches) |
| timelapse_25/50/75/100.png | stills at 25/50/75/100 % of the timelapse (video frames 98, 197, 296, 394, counted from 0; extracted with ffmpeg): 06:23:20, 06:28:17, 06:33:14, 06:38:09 Hamilton (index csv) |
| lidar.tum, lidar_map.*, wheel.tum, products.log, provenance.txt, replay_core.log | the robot's LiDAR replay products (method RESULTS, "LiDAR check") |
| score_lidar/, score_lidar_recognition_events/, score_lidar_recognition_events_fit_v2/ | the LiDAR scores (method RESULTS explains which one is registered) |

No summary figure exists for session 2 alone; the diagnosis files are in `03_methods/localisation_demo_2026-09-26/session2_diag/` (README.txt lists them). Session 3's pack (`../s2_loc_03/`) has the sessions 2 vs 3 comparison in the method RESULTS. The timelapse's single frames were not downloaded (they stay in cloud storage).
