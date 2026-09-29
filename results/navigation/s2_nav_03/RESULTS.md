# Navigation run s2_nav_03 - route B2, full round trip (27 Sept 2026, Hamilton time)

Localisation on drive 10's saved map (localisation mode: the map is only read, never added to), move_base with the
27c safety sender, user beside the robot holding the dead-man button (R2). Start 18:53, home 19:03.

| # | goal | result | time | end, robot's estimate to goal |
|---|---|---|---|---|
| 1 | B1 east junction | **reached on its own** | 74.9 s | 0.16 m |
| 2 | B2 south-east turn | **reached on its own** | 65.2 s | 0.23 m |
| 3 | W2 south corridor, through the double doors | **failed** twice: 0.10 m to the left door edge, the planner found no safe path (`nav_B2`, `nav_B2rest`) | - | - |
| - | manual assist | **driven straight 1.50 m at 0.10 m/s** through the doorway by the Jetson operator publishing forward-only commands through the same safety sender (clearance guard on, user holding R2), 19:00 (`MANUAL_ASSIST.txt`) | 16 s | - |
| 4 | B3 south-west turn | **reached on its own** | 36.8 s | 0.23 m |
| 5 | B4 start mark (home) | **reached on its own** | 93.2 s | 0.19 m |

- **4 of 5 goals reached on their own; one 1.5 m stretch through the doorway needed a straight-line command.** No contact.
- Path following, route planned when each goal was sent vs where the robot drove (its own estimate): median 0.03 m and
  95th percentile 0.24 m on the first two legs; median 0.11 m and 95th percentile 0.37 m on the last two
  (`score_nav_*/nav_score.json`). The planner re-plans every second; against the plan in force the median is under 0.01 m.
- Localisation: 184 position fixes (33 recognised places), no suspect jumps (`localise_summary.json`); on the way home
  a fix at least every 1.23 m.
- The stops on the way home were the dead-man button being released (sender log), not the planner.
- The doorway (x about 11-13.5 m on the south corridor) is the narrowest point of the route: 11.5 cm spare each side of
  the 0.67 m robot. s2_nav_02 got through it; this run stopped 0.10 m from the door edge. **Open limit:** a doorway that
  tight needs either a centring step before it or a slower, straight-only mode through it.
- LiDAR recording on the robot's card: `s2_nav_03_lidar.bag`, 5.9 GB, closed cleanly 19:05. Not scored against the LiDAR.
- Scoring note: for the first two parts the scorer could not match positions to the wheel log in time, so fix spacing is
  given for the way home only (`score_nav.py` now reports this instead of stopping; backup `.before_emptyfix`).
