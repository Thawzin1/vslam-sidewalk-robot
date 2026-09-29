# s2_slam_01 - SLAM: continuing drive 10's whole-floor map (27 Sept 2026, 13:25-13:52 Hamilton)

*Scored 27 Sept 2026 16:15 Hamilton against the pass lines registered before the drive (`03_methods/slam_session_2026-09-27/PASS_LINES.md`). N = 1 drive.*

**What it is:** SLAM (building the map while using it) on top of drive 10's saved map of the whole 2nd floor: the robot
started on the usual start mark, recognised drive 10's map, and kept mapping - adding new corridors (areas A, B, D, E)
to the same map. Camera (ZED X) + wheels + gyroscope blend, RTAB-Map 0.21.13 in mapping mode on a copy of drive 10's map.
*Plain terms: the robot opens yesterday's floor plan, finds itself on it, and draws in the parts it has not seen before.*

| what | result |
|---|---|
| joined drive 10's map | **yes, at once** (0.4 s after the map started; recognised place 0.08 m from the start mark) |
| driven | 156 m, 1178 new map places (495 kept in the saved graph) |
| links between today and drive 10 | **238** (25 recognised again, 213 nearby re-matches) |
| start-to-end gap, corrected (map's own corrections) | **0.044 m** |
| start-to-end gap, tracking alone (camera + blend, no corrections) | 0.221 m |
| new ground | **73 m2**, 54 m of path more than 1 m from any drive-10 place |
| drive 10's old map moved by today's corrections | median 0.019 m, 95th percentile 0.19 m, largest 0.33 m (860 places) |
| map closed properly; drive 10's master map unchanged | both **PASS** (`closed_check.json`, master sha256) |
| people blanking | **did not run** in the live drive: the filter compared the camera's label "PERSON" with "Person" (0 of 18,980 pictures blanked). Fixed; the comparison is done by replaying the recorded drive with blanking on and off |

Figures: `slam_map.png` (today's path on drive 10's map, new ground red), `old_map_shift.png`, `timelapse.mp4` (live map page).
Sources: `split_facts.json` (session_split.py), `closed_check.json` (slam_closed_check.py). N = 1 drive.

## People blanking - the same pictures replayed twice (27 Sept, 14:09-15:05)

The drive's camera recording replayed into two fresh copies of drive 10's map, camera only (no wheels), people
detector running in both; the only difference is whether people are blanked (`replay_svo_slam.sh`). Figure:
`blanking_comparison.png`.

| | live drive (camera + wheels + gyroscope) | replay, blanking OFF | replay, blanking ON |
|---|---|---|---|
| joined drive 10's map | at once, 0.08 m from the start mark | at once, 0.19 m | at once, 0.28 m |
| people boxes blanked | 0 (label bug) | 0 | **2,944** (13 tracked people, confidence ~90 %, median box 16 % of the picture, all in the first ~7 min) |
| links to drive 10's map | 238 | 230 | 225 |
| drive 10's map places pushed by the run: median / largest | 0.019 / 0.33 m | **0.356 / 0.96 m** | **0.092 / 0.67 m** |
| last map place from the start | 0.044 m | 12.55 m | 12.98 m |

- **Blanking ON pushed drive 10's map about 4x less** than blanking OFF on identical pictures. N = 1 drive: an indication, not a proof.
- The replays' last map place is at the end of area D in both: the reverse back to the start mark was not added
  to their maps with the camera alone [INFERENCE from the figure]. So 12-13 m is not drift of a closed loop, and the
  replays are not used for the map's accuracy; the live drive is.

## Pass lines (registered before the drive)

| line | result | verdict |
|---|---|---|
| **S1 - joins drive 10's map** within 60 s and 5 m | 0.4 s after the map started, 0.0 m driven, recognised place 0.08 m from the start mark | **PASS** |
| **S2 - no wrong joins** (link vs wheels + gyroscope > 1.5 m + 2 %) | 0 wrong of 238 links; disagreement median 0.017 m, 95th percentile 0.12 m, largest 1.24 m (`s2_summary.json`) | **PASS** |
| **S3 - one consistent map after park** | (a) closed properly, sessions joined; (b) drive 10 moved median 0.019 m (line 0.15), 95th pct 0.19 m (line 0.50); (c) start-to-end gap 0.044 m corrected (line 0.5 m) | **PASS** |
| **S4 - agrees with the LiDAR estimate** (median <= 0.70 m) | this session's LiDAR estimate (Force3DoF replay on the Jetson, 699 nodes, 62 closures) ends **0.52 m** from its start, failing its own 0.5 m check, and its map is visibly bent (`lidar_map.png`); median 0.88 m, 95th pct 1.40 m, size fit 1.054 for information only (`lidar_agreement.json`) | **NOT SCORABLE** (session-3 precedent) |
| **S5 - new area really added** (>= 15 m2 from >= 8 m of new path) | 73.4 m2 from 54.4 m of path outside drive 10 | **PASS** |
| D0-D3 - people test lines | not scored yet: need the person's recorded positions and ghost-cell counts in the two replays; the replay comparison above (old map pushed 0.36 vs 0.09 m) is an extra measure, not a registered line | **pending** |

*Plain terms: the robot recognised the old map at once, never glued today's corridors to the wrong place, kept the old
map where it was and added 73 m2 of new floor. The LiDAR's own map from this drive did not pass its own check, so the
camera-vs-LiDAR number is not scored.*
