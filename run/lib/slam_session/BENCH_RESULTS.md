# SLAM session - bench results (27 Sept 2026, 08:59-09:50 Hamilton, Jetson, camera only, robot OFF)

*The robot was off (charging), parked with its camera on the start mark looking down the long corridor. Every test
started the camera itself and stopped it afterwards (the camera was not running before; the camera mode file was
restored byte for byte). N = 1 per condition; 60 s measuring windows. Records: `bench/` in this folder.
Jetson up 7-7.7 h during the tests (camera freezes grow with uptime - SOLVED.md 26 Sept).*

## 1. Mechanics - does the SLAM mode do what DESIGN.md says?

| check | bench 01 (no mask, SVO) | bench 03 (people mask, SVO) | label |
|---|---|---|---|
| master map untouched | sha256 `98f71ed1...` before = after | same | [BENCH] |
| working copy made, byte-compared | 3.49 GB, `cmp` equal | same | [BENCH] |
| old map loaded in MAPPING mode | log `rtabmap: SLAM mode`; `Working Memory = 860, Local map = 860`; 42 s to load | same, 43 s | [BENCH] |
| pinned settings on the parameter server | IncrementalMemory true, InitWMWithAllNodes true, LoopThr 0.08, AggressiveLoopThr 0.05 | same | [BENCH] |
| new session = new map id | map id 1: 116 new rows (2 kept in the graph; 114 discarded standing still, weight -9) | map id 1: 201 new rows (4 kept) | [BENCH] |
| **the join** (first link today -> drive 10) | today 1561 -> drive-10 node 144, recognised again (type 1), **3.6 s after the map loaded** | today 1560 -> drive-10 node 138 (0.28 m from the start mark), type 1, **2.5 s after the map loaded** | [BENCH] |
| where the join put the camera on drive 10's map | map -> odom (0.16, -0.09) m | links imply (-0.01, -0.06) m from drive 10's start (4 links, spread 0.06 m) | [BENCH]; the camera was parked on the start mark |
| live JOINED line | **missed** (watcher v1 had a wrong field name and started 40 s after the join) | `JOINED yes ... old place 0.28 m from the start mark` 1 s after it started (v2: reads the map's working graph) | [BENCH] |
| closed properly on "park" | autostop 5 s; Admin saved after the start; saved graph 862 = 860 old + 2 new -> joined | 4 s; 864 = 860 + 4; links today->drive 10: 2 recognised + 2 nearby | [BENCH] |
| database whole | quick_check ok (1674 nodes) | ok (1759 nodes) | [BENCH] |
| old map intact | all 1558 old ids present; drive-10 nodes moved median 0.003 m | all present; moved median 0.023 m, max 0.037 m | [BENCH] |
| camera recording (SVO=1) | 408 MB, opens (pyzed, 1847 frames) - but **started 21 s after the join** (step 3d comes after the map) | **fixed** (step 2S, before the map): 840 MB, 3608 frames, from 13:26:20 UTC, before the map started | [BENCH] |
| timelapse | 35 frames, video made | video made | [BENCH] |
| live map page | the start script's 4 s check said "did not start"; it was listening on :8095 moments later (same peer as found) | same warning | [BENCH]; pre-existing check timing, not a SLAM change |

**Not observable on this bench:** the working graph holding today's nodes only *before* the join (DESIGN.md 1.2 f):
both joins came within 4 s, before any watcher could look. That step rests on the source reading.

## 2. People detector - cost, measured

| condition | pictures / map rate | GPU busy % mean (95th pct) | CPU busy % all 12 cores | GPU+SOC power, mean | source |
|---|---|---|---|---|---|
| camera only, detector OFF | depth 14.89 Hz | 36.9 (96) | 15.7 | 12.0 W | `bench/od_20260927_0859/` |
| camera only, detector ON (FAST, people) | depth 14.71 Hz; detector 14.73 Hz (every picture) | 37.2 (91) | 18.6 | **14.2 W** | same |
| full stack, no detector, no mask (bench 01) | map tracking (/rtabmap/odom) **6.87 Hz** | 38.5 (90) | 40.8 | 13.2 W | `bench/s2_slam_bench_01/` |
| full stack, detector ON + mask (bench 03 A) | tracking **7.47 Hz**; detector 13.16 Hz | 38.4 (93) | 40.4 | 13.5 W | `bench/s2_slam_bench_03/` |
| full stack, detector OFF, mask passing through (bench 03 B) | tracking **7.04 Hz** | 32.8 (86) | 43.6 | 13.7 W | same |

- **Reading:** the tracking rate is the same with and without the detector + mask (6.87-7.47 Hz; the three windows
  differ by less than the spread between the two windows with no detector). **M1's bench condition is met**: 7.47 Hz
  with the detector is 106 % of the 7.04 Hz without it (>= 80 % required). The detector's cost shows as about
  **+2.2 W** and **+3 CPU points** at camera level; the GPU busy figure (sampled once a second) is too coarse to
  separate conditions. The mask adds **5-28 ms** per picture (worst per 10 s window) and keeps up at 14-15 pictures a
  second. *Plain terms: switching the people detector on did not slow the map down measurably.*
- **Uncertainty:** one 60 s window per condition, parked, lab scene, no people in view during the windows. A 2.8 s
  camera stall occurred in window A (longest gap of camera pictures); with N = 1 it cannot be attributed to the
  detector (stalls also occur without it and grow with uptime; window A followed the stall of section 3).
- **One-time preparation:** the detector had never been optimised on this Jetson. `od_bench.sh` did it: **649 s with
  no pictures** (file `/usr/local/zed/resources/.objects_performance_3.2.model_optimized-...`, 21 MB, written 13:09
  UTC). Restarting the detector afterwards: 2 s. MASK=1 refuses to start if that file is missing.

## 3. The mask - does it blank people, and only people?

| test | result |
|---|---|
| known answer, offline (`depth_person_mask.py --selftest`) | PASS: box (874, 224)-(1027, 977) = the 900-1000 x 300-900 person box + 10 % + 16 px; 115 209 pixels (5.00 %); a chair box and a 30 %-confidence person left alone; half-resolution scaling right |
| **live made-up person** (`fake_person_test.py`, detector off, a Person box published with each picture's own time stamp) | **PASS**: negative control 5 of 5 pictures identical to the camera's; with the box 5 of 5 pictures all-zero inside the box + margin and identical outside; 303 pictures masked in 20 s (every picture) |
| **real person, unplanned** (13:27:45-13:28:20 UTC, bench 03: someone stood **closer than 1 m**, head and shoulders filling the picture) | the detector reported **no person** (0 boxes; only the made-up id 999 appears in `mask_events.csv`), so nothing was masked. The camera tracking lost the picture **52 times in about 20 s** ("not enough features": the person was closer than the 1.0 m minimum depth) and recovered by itself; the camera recording also has two gaps (about 5 s and 7 s) there. Frames were checked by eye and not kept (privacy) |

- **What this means for the session:** the mask works on what the detector boxes, at the full picture rate. A person
  seen only in close-up (under about 1 m, part of the body) is **not** boxed by the FAST model - and at that range the
  map could not use those pixels anyway (no depth under 1 m). The protocol keeps the person events at 2-4 m.
- **Not yet seen:** a real, whole person at 2-4 m being boxed and blanked - the session's P1/P2 are the first test.

## 4. Decision (recorded before the session, PASS_LINES.md)

**Run the session with the people mask ON (`MASK=1`)** and the camera recording ON (`SVO=1`): the measured cost is
within the noise, the chain is verified end to end with a made-up person, and the detector is optimised. The offline
OFF-vs-ON replays of the same recording (D0-D3) still decide whether the mask *helps*; running it live is what lets
the live map show it. If the `_mask` line says DETECTOR STALE for long during the session, the map simply continues
unmasked (M1 then fails; the offline comparison still stands).

## 5. Offline replay path (for D0-D3) - smoke test

`replay_svo_slam.sh s2_slam_bench_03 on 150` (09:35-09:39 Hamilton): the camera wrapper played bench 03's recording
(every frame, `svo_realtime` false), the detector ran on it, the mask ran (2237 pictures, about 13.5 a second, worst
+5 ms), RTAB-Map mapped into a **fresh** copy of drive 10's map: **joined** (today 1559 -> drive-10 node 1499,
recognised again, 0.08 m from the start mark), closed properly on the STOP file after 150 s (4 s), all 5
`slam_closed_check.py` checks PASS, master unchanged, camera stopped afterwards. The close-up person of section 3 was
again **not** boxed (0 people in 2237 pictures) - the same answer as live. Records: `bench/replay_smoke/`.
**So the OFF/ON comparison is ready to run on the session's recording** (two runs, one variable).

## 6. Disk, measured

- The working copy grows about **2.3 MB per new node even while parked** (bench 01: +267 MB for 116 new rows in about
  2.5 min; RTAB-Map keeps discarded pictures, `Mem/NotLinkedNodesKept` true). About 1 node a second -> **about 4 GB per
  30 min** on top of the 3.5 GB copy.
- Camera recording: **3.4 MB/s** parked (bench 01: 408 MB / 120 s); drive 4's measurement says up to 8 MB/s moving ->
  **6-14 GB per 30 min**.
- **Session total: about 14-22 GB.** The Jetson had 12.3 GB free at the start of these tests. Free space first under
  storage sense (rule 24), or run with `SVO=0` (then D0-D3 are not possible).

## 7. What failed and was fixed (all staged files, nothing live)

| what | how it failed | fix | verified |
|---|---|---|---|
| bench 02 | camera never started: the `config_file:=` path contains spaces (folder names with spaces in them); an unquoted string split it and roslaunch refused ("input files do not exist") | a bash array `"${CAMCFG[@]}"` in `start_drive.sh` | bench 03 started the camera with the file |
| JOINED line v1 | wrong message field (`proximityClosureId`; this build's `rtabmap_msgs/Info` calls it `proximityDetectionId`), and started 40 s after a join that came 4 s after the map loaded | v2 reads the map's working graph (`/rtabmap/mapGraph`: state, not events), starts right after step 3S, 0.10 m dead band on distance | bench 03: JOINED seen 1 s after it started, 0.0 m driven while parked |
| camera recording started late | step 3d comes after the map, so the join was not in the recording | SLAM=1 starts it at step 2S, before the map; 3d only checks and starts the guard | bench 03: recording from 13:26:20, map started 13:26:3x |
| S2 scorer negative control | the first made-up wheel path moved only before the first link, so nothing could disagree (it "passed") | redone with motion between the links | 3 of 4 links flagged wrong, as they must be (`bench/s2_slam_bench_03/s2_test/`) |

## 8. Left behind / removed (rule 9)

- Removed: the three bench working copies (3.5-3.9 GB each), bench 01's camera recording, all bench run records under
  `~/.run_records/` and their jobs-page lines, the face frames extracted during the section-3 check.
- Kept (small): `bench/` in this folder (logs, rates, tegrastats windows, checks, timelapse videos, split and S2 test
  outputs), about 2 MB.
- The detector's optimised model file (21 MB, `/usr/local/zed/resources/`) is kept - it is what makes MASK=1 start
  in 2 s instead of 11 minutes.
