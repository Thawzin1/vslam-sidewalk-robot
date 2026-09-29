# SLAM session - pass lines (registered 27 Sept 2026, Hamilton, BEFORE the session)

**What is shown:** the robot is given a copy of drive 10's saved whole-floor map and keeps **mapping** on it
(RTAB-Map mapping mode, a new session in the same database, DESIGN.md section 1). It should recognise where it is on
drive 10's map (the **join**), add corridors drive 10 never entered, and leave one consistent map.
*Plain terms: the robot continues yesterday's floor plan instead of starting a blank one - it must find its place on
it, draw the new rooms onto it, and not tear or misplace anything.*

**N = 1 session.** Counts, never percentages. Every figure carries its spread or its uncertainty (ENGINEERING_NOTES.md section 4
rule 0). The LiDAR estimate is a second opinion, not ground truth (rule 19). Run id: `s2_slam_01` (next free:
`s2_slam_02` ...). Settings as DESIGN.md section 1.3 (`Rtabmap/LoopThr` 0.08 after the join; people mask per the
decision recorded at the bottom of this file, BEFORE the session).

**Uncertainties stated in advance:**
- start mark parking by eye: **0.2-0.5 m** and a few degrees ([[tape-mark parking]] memory; never a reference for
  sub-0.5 m lines).
- the robot's wheels + gyroscope: heading drifts about **5 deg/min** even parked (robot-heading-drift memory), so
  wheel-based checks use distance over short stretches, not absolute heading over the session.
- LiDAR estimate: range noise 15-30 mm; the camera-vs-LiDAR mounting offset has never been measured; drive 10's
  camera map already disagreed with its LiDAR estimate by **0.69 m median / 1.15 m 95th percentile** (SE(3); best-fit
  size factor 1.072) - this session continues that map, so it inherits that disagreement.
- grid cells 0.05 m; the new-area metric uses 0.10 m cells and a 1.5 m "same corridor" radius.

| line | pass if | plain meaning | measured from |
|---|---|---|---|
| **S1 - joins drive 10's map** | the first link from a new-session node to a drive-10 node (link type 1 or 2) comes within **60 s** of the session's first node **and** within **5 m** of driving (robot wheels) | it realises "this is drive 10's floor" within a minute of being switched on at the start mark, before it has gone far | `slam_events.csv` (live) and the database Link table (`session_split.py` `join_after_s`); wheels from fusion.bag `/robot/ekf_odom` |
| **S2 - no wrong joins** | **zero** links between new-session nodes and drive-10 nodes that disagree with the robot's own wheels + gyroscope by more than **1.5 m + 2 % of the wheel distance** since the previous accepted link (the first link: since the start mark) | it never glues today's map onto the wrong corridor | database Link table + drive 10's saved positions + fusion.bag wheels (`score_slam.py`, RESULTS_PACK_PLAN.md) |
| **S3 - one consistent map after "park"** | (a) closed properly (`slam_closed_check.py` all PASS, sessions joined in the saved graph); (b) drive 10's nodes moved by today's corrections: median **<= 0.15 m**, 95th percentile **<= 0.50 m**; (c) today's corrected start-to-end gap **<= 0.5 m** (parking uncertainty), reported beside the tracking-alone gap and the closure counts (rule 20) | the old map was straightened where needed, not bent or torn, and the robot ends where it started on the combined map | `session_split.py` (`drive10_shift_m`, `gap_corrected_m`, `gap_tracking_alone_m`, `closures_by_kind`) |
| **S4 - agrees with the LiDAR estimate** | today's corrected path vs this session's LiDAR Force3DoF replay (SE(3) alignment, no resizing): median **<= 0.70 m** | adding a session does not make the map disagree with the LiDAR more than drive 10 already did (0.69 m) | `compare_lidar.py` on today's `camera_corrected.tum` + the robot's replay |
| **S5 - new area really added** | floor within 1.0 m of today's corrected path and more than 1.5 m from every drive-10 node: **>= 15 m2**, from at least **8 m** of path outside drive 10's corridors | at least one side corridor's worth of floor that drive 10 never mapped is now in the map | `session_split.py` (`new_area_m2`, `new_path_m_outside_drive10`) |

Report also (not judged): time and distance to the join; links to drive 10 per 10 m; links within today; longest
stretch with no link to drive 10 (m, s); camera freezes over 1.5 s; tracking losses; WiFi gaps; nodes added; database
size at the end; how many drive-10 nodes moved more than 0.3 m and where (figure).

**What failing would look like:** S1 - "JOINED no" after 60 s standing and 5 m of driving (held recognitions
never over 0.05, or the 3D check refusing: `mapping.log` "Not enough inliers"). S2 - a link whose old node is in
another corridor (the live `!! SUSPECT JOIN` flag, or a wheel disagreement found afterwards). S3 - `NOT CLOSED
PROPERLY`, or drive-10 nodes moved metres (a wrong closure bent the old map). S4 - median over 0.70 m. S5 - under
15 m2 (the side corridor was not driven far enough, or its nodes were discarded).

---

## People and moving objects (scope addition, 27 Sept) - registered with the lines above

**The test:** two person events during the session (PROTOCOL.md): **P1** while the robot is parked at a 30 s stop, a
person walks across the camera's view 2-4 m in front, and back (about 10 s); **P2** while the robot drives a
straight corridor, a person walks ahead of it in view, 2-4 m away, for about 10 m. The camera recording (SVO=1) is
then replayed **twice through the same stack**, into two fresh copies of drive 10's map: people mask OFF and ON.
Same pictures, same settings; the mask is the only difference (ENGINEERING_NOTES.md section 4 rule 5). Replays are camera-only
(no wheels), the same for both.

| line | pass if | plain meaning |
|---|---|---|
| **D0 - the test had a signal** | the detector reported a Person box in **>= 20 pictures** during each of P1 and P2, and the OFF replay shows the person: **>= 20 occupied grid cells or >= 2000 3D points** within 0.5 m of the person's recorded positions | otherwise nothing can be concluded, and D1-D2 are reported as "no result", not as a pass |
| **D1 - no ghost** | ON replay: occupied grid cells and 3D points within 0.5 m of the person's recorded positions are **<= 20 %** of the OFF replay's counts | with the mask, the person leaves (almost) no trace in the map |
| **D2 - tracking not pushed by the person** | during P2 (and P1), the camera tracking's deviation from the robot's wheel + gyroscope motion over the person window (aligned at its start): ON **<=** OFF + 0.05 m, and tracking losses ON **<=** OFF | walking people do not drag the robot's own position estimate |
| **D3 - no wrong closures** | **zero** wrong links (S2 method) in the ON replay, and none of its accepted links joins two pictures in which the same person is boxed | people never cause a false "I have been here" |
| **M1 - it keeps up live** (only if MASK=1 live) | the map's odometry rate with detector + mask is **>= 80 %** of the rate without (bench, BENCH_RESULTS.md), and the `_mask` line says "masking" (detector alive) for all but **<= 60 s** of the session | the people detector does not slow the map enough to hurt it |

Uncertainty: person positions come from the detector's own 3D position (ZED), about +-0.2 m at 3 m [UNVERIFIED for
this model]; hence the 0.5 m radius. Box-level masking also removes the background inside the box, so a small loss of
wall points in the ON replay is expected and not a failure.

---

## Decision recorded before the session (to be filled by the Jetson operator from the bench, then frozen)

- People mask LIVE during the session: **ON (`MASK=1`)**, decided 27 Sept 09:45 Hamilton from the bench
  (BENCH_RESULTS.md section 4): tracking 7.47 Hz with the detector + mask vs 7.04 Hz without; chain verified with a
  made-up person. M1's bench reference rate is **7.04 Hz** (80 % = 5.6 Hz). Camera recording ON (`SVO=1`) for D0-D3.
- `Rtabmap/LoopThr` 0.08 (DESIGN.md 1.3). Changing it after the session has started makes the session not S1-S5.
