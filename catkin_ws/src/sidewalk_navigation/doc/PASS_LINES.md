# Navigation demo - pass lines (REGISTERED 27 Sept 2026 ~10:20 Hamilton, BEFORE any real run)

*Dry-run column updated ~10:55 Hamilton with the final settings (sim runs 9 and 10); the lines themselves are unchanged.*

**What is shown:** the Husky, given drive 10's saved map and told it stands on the start mark, drives ITSELF to goal
points on that map - route A: start mark -> east junction (14.3 m) -> back to the start mark - with the planned route
and the driven path both recorded and drawn .
*Plain terms: you give it a point on the floor plan; it works out how to get there and goes, and afterwards we can
show on one picture the route it meant to take and the path it really took.*

**N = 1 run per route.** Counts and distances, never percentages; no claim of repeatability until a route has been
driven five times (ENGINEERING_NOTES.md section 4 rule 2). The robot's own position is its localisation's estimate; the LiDAR
replay is a second opinion, not ground truth (rule 19).

**Scored by** `score_nav.py` from what `nav_goals.py` records (`~/.run_records/<run>/nav_<route>/`), the robot's own status
stream (`robot_status.jsonl`), the localisation fixes (`fixes.csv`, from `/rtabmap/info`), the user's notes (N2), and,
after the robot-side LiDAR replay, the LiDAR estimate (N5).

| line | pass if | plain meaning | dry run (simulator, route A, final settings: runs 9 and 10) |
|---|---|---|---|
| **N1 - reaches each goal** | for EVERY goal: move_base reports SUCCEEDED within the goal's time limit (150 s per leg of ~14 m), and the robot's own estimate ends **<= 0.50 m** from the goal | it gets to each point on the floor plan, in reasonable time, by its own reckoning | run 9: 2 / 2, 85 s and 102 s, truth 0.23 / 0.10 m from the goals; run 10: 2 / 2, 64 s and 64 s, truth 0.18 / 0.31 m |
| **N2 - no bump, no rescue** | **zero** contacts with anything (the user watches), and **zero** times the user had to let go of R2 or press L1 to prevent one | it drives the whole route without touching anything and without the user having to step in | not judgeable in simulation (no user); software part: 0 unplanned stops in both runs |
| **N3 - drove the route it planned** | the robot's estimate vs the route planned when each goal was sent: **median <= 0.15 m, 95th percentile <= 0.40 m**, all legs together | it follows its own plan closely: typically within a hand's width, and never wandering across the corridor | run 9: 0.083 / 0.235 m; run 10: 0.074 / 0.229 m (truth vs plan: 0.087 / 0.215 and 0.085 / 0.285 m) |
| **N4 - stays localised** | (a) a localisation fix (recognition or nearby re-match accepted on `/rtabmap/info`) within the first **10 m** of driving; (b) no stretch longer than **10 m** without a fix; (c) no single correction of the position larger than **1.0 m** | it keeps knowing where it is on the map all the way, and never suddenly decides it is somewhere else | not judgeable (the stand-in corrects every 2 s); largest step 1.0 m (run 9), during turns on the spot where the simulated wheels slip between corrections |

**Reported, judged loosely (second opinion, after the robot-side LiDAR Force3DoF replay):**

| line | pass if | plain meaning |
|---|---|---|
| **N5 - the LiDAR agrees** | the robot's estimated path vs the LiDAR estimate (SE(3) alignment, no scale): median **<= 0.50 m**; and route A's LiDAR start-to-end distance (it returns to the start mark) **<= 0.50 m** | an independent sensor agrees the robot went where it thought, and came back to where it started (W4's line on the whole-floor localisation) |

Also reported, not judged: time per leg, turns on the spot and how long they took, automatic stops with their reasons
(the robot's status stream), fixes per 10 m, the jobs-page lines' timeline, battery at start and end.

**Uncertainty stated in advance (section 4 rule 0).** The estimate carries the localisation's own error: on drive 10's
map the camera map and the LiDAR estimate differ by a median 0.69 m over 252 m, mostly from a 7 % size difference
(RESULTS.md of s2_static_10), so N1 and N3 measure how well the robot follows ITS OWN map, not metres on the floor;
N5 is the check against an independent sensor, whose own spread is 15-30 mm range noise plus an unmeasured
LiDAR-to-camera mounting offset. The start mark itself is only good to 0.2-0.5 m by eye (parking on tape), so no
tape distance is scored.

**Why these numbers.** N1's 0.50 m = twice the planner's own arrival tolerance (0.25 m), allowing one localisation
correction on arrival. N3's lines are about twice the final dry runs' (0.07-0.08 / 0.23 m), because the real
localisation corrects in steps whose size the simulator did not model. N4's 10 m: the whole-floor localisation (s2_loc_04) got a
fix about every 0.4 m once driving, and its cold start needed 6 m - with the start mark given, 10 m is generous.
N4's 1.0 m: W3's "wrong place" line was 1.5 m; a correction of 1 m in a 1.7 m corridor is already enough to steer
the route into a wall, so it is set tighter.

**A route passes only if N1-N4 all pass.** Route B is attempted only after route A passes.
