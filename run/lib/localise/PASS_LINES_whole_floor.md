# Whole-floor localisation - pass lines (registered 27 Sept 2026 ~04:55 Hamilton, BEFORE the session)

**What is shown:** the robot is given drive 10's saved map of the whole 2nd floor (`~/slam_series2/localise/s2_static_10_map.db`,
a checked copy: quick_check ok, 1558 nodes, corrected positions saved) with map-building switched off, and must work out
where it is from what the camera sees, while it drives the floor.
*Plain terms: the robot already has the floor plan; the test is whether it can tell where on it it is, and keep knowing.*

**The session (user's protocol, 27 Sept):** no taped spots. The robot starts on the usual start mark, facing down the
long corridor as for every drive, and is told it is 40 m OUTSIDE the building (the off-map start used since session 2),
so any fix must come from real recognition. The user then drives the floor **slowly, the same way round as drive 10**
(same turns - the map only knows the views drive 10 saw, ENGINEERING_NOTES.md section 2.13a), stops **30 s still at each end of a
walkway**, turns, and continues. Ends parked on the start mark ("park").
Recorded: the Jetson's localisation log and fusion recording; the robot's LiDAR recording (for a Force3DoF replay after).

**N = 1 session.** Results are counts, never percentages. The LiDAR replay is a second opinion, not ground truth (rule 19).

| line | pass if | plain meaning |
|---|---|---|
| **W1 - finds itself at the start** | first fix (recognised or nearby re-match on `/rtabmap/info`, 3D check passed) within **120 s** of the start and within **3 m** of driving | it realises "I know this place" within two minutes of being switched on somewhere it was told it isn't |
| **W2 - keeps knowing along the way** | at **all but one** of the 30 s walkway-end stops, there is a fix during the stop or in the last 10 m before it | it keeps recognising places all round the floor, not just at the start |
| **W3 - no wrong places** | **zero** fixes that put the robot more than **1.5 m** from where the robot's own wheels + gyroscope say it moved since the previous fix (checked from the robot's own recording) | it never "recognises" the wrong corridor |
| **W4 - where it thinks it is, vs the LiDAR** | localised path vs this session's LiDAR Force3DoF replay (SE(3) alignment, no scale): median **<= 0.50 m** | its position agrees with an independent LiDAR estimate about as well as the mapping drives did (drive 10 map vs LiDAR is being measured now; drives 4/8: 0.24-0.25 m, drive 9: 0.42 m) |

Report also (not judged): time localised vs not, longest stretch without a fix (metres and seconds), fixes per 10 m, where
fixes were missing (drawn on the map), WiFi dropouts (the live view may pause in the far corridors; scoring uses on-board
recordings only).

**Uncertainty stated in advance (section 4 rule 0):** walkway-end stop positions are not taped, so no tape distance is
scored. The LiDAR estimate's own spread: range noise 15-30 mm; its agreement with the camera on mapping drives ranged
0.24-0.43 m median, which is why W4 is set at 0.50 m and not tighter.
