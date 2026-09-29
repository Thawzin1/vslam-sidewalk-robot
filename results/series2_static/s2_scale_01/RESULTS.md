# s2_scale_01: taped 15.00 m distance test (26 Sept 2026, 13:10-13:22 Hamilton)

**Answer: the camera is the honest ruler, and the wheels are not.** The camera's own tracking read the 15.00 m tape at
**14.90 m (median, -0.7 %)**. The wheels read **15.94 m (+6.2 %)**. The camera/wheels ratio is **0.932**, the same
~6 % gap seen on drives 3-6.

*Plain terms: two floor marks were taped 15.00 m apart and the robot drove between them six times. The camera said
"about 15 m" and the wheels said "about 16 m". Because the tape settles it, the wheels are the ones that read long.*

## What was run
- Two floor marks 15.00 m apart (tape; the user measured them; +-5 mm). A wheel stop was placed at each mark. The robot
  started at mark 1 facing mark 2 and made 3 round trips: forward to mark 2, wait 10 s, reverse (driving backwards) to
  mark 1, wait 10 s.
- The Jetson mapping drive used FUSION=1 and camera tracker GEN_1. The run closed properly (Admin.opt_poses saved).
- Sources: `~/.run_records/s2_scale_01/fusion.bag` (Jetson internal disk) holds `/robot/wheel_odom` (wheels),
  `/robot/ekf_odom` (the robot's wheels + gyroscope, /odometry/filtered), `/fused/odometry` (the blend) and
  `/rtabmap/odom` (the camera's own tracking; 10 empty "lost" messages dropped). The database
  `~/slam_series2/s2_scale_01.db` (Jetson internal disk) was opened read-only (`immutable=1`) and its corrected map
  positions came from `Admin.opt_poses` via `scripts/db_corrected_tum.py`.
- Parked moments are periods of 5 s or more with wheel speed under 0.01 m/s, and **7 were found** (bag seconds):
  0-38, 129-153, 235-251, 326-343, 420-441, 513-541, 615-680. The position at each is the median over the parked
  window, with 1 s trimmed at each end; position scatter while parked is under 13 mm for every source. The corrected map
  has one map point (node) per parked moment, because the map adds no new points while the robot stands still.
- A leg's distance is the straight line between two consecutive parked positions. No path length is summed.

## The six legs (m)

| leg | direction | tape | wheels | robot wheels+gyro | camera | blend | corrected map |
|---|---|---|---|---|---|---|---|
| 1 | forward | 15.00 | 16.02 | 16.04 | 14.94 | 15.66 | 14.96 |
| 2 | reverse | 15.00 | 16.00 | 16.03 | 14.80 | 15.46 | 14.91 |
| 3 | forward | 15.00 | 15.78 | 15.80 | *15.28 (reset)* | 15.36 | 14.75 |
| 4 | reverse | *~14.0 (stop short)* | 14.78 | 14.81 | 13.68 | 14.42 | 13.75 |
| 5 | forward | *~14.0 (stop short)* | 14.70 | 14.72 | 13.82 | 14.33 | 13.73 |
| 6 | reverse | 15.00 | 15.87 | 15.89 | 14.89 | 15.36 | 14.77 |

**Two exclusions, each found in the data and not chosen:**
- **Legs 4 and 5 have no tape reference.** At the 5th parked moment, before leg 5, the robot stood about **1.0 m short
  of mark 1**. Three independent readings agree: the corrected map puts it 1.05 m from the other mark-1 stops, the wheels
  1.04 m and the blend 0.94 m. Every source reads both legs about 1.1-1.2 m short, so this is where the robot stopped,
  not a sensor fault. *To check: did the robot stop before the wheel stop on the second return?* These legs still count
  for the camera/wheels ratio, which does not need the tape.
- **The camera's leg 3 is not the camera's own reading.** Tracking was lost at 17:16:00 and reset at 17:16:01
  (`mapping.log:6121-6173`). After a reset, tracking restarts from the blended position, so the leg mixes camera and
  blend readings. It is excluded for the camera only.

## Against the tape (valid legs only)

| source | legs used | median | spread (min-max) | error vs 15.00 m | within 1 % (0.15 m)? |
|---|---|---|---|---|---|
| wheels | 1, 2, 3, 6 | 15.94 | 15.78-16.02 | **+0.94 m (+6.2 %)** | **no** |
| robot wheels+gyro (/odometry/filtered) | 1, 2, 3, 6 | 15.96 | 15.80-16.04 | +0.96 m (+6.4 %) | no |
| blend | 1, 2, 3, 6 | 15.41 | 15.36-15.66 | +0.41 m (+2.7 %) | no |
| **camera own tracking** | 1, 2, 6 | **14.90** | 14.80-14.94 | **-0.11 m (-0.7 %)** | **yes (median)**; leg 2 alone is -1.4 % |
| corrected camera map | 1, 2, 3, 6 | 14.84 | 14.75-14.96 | -0.16 m (-1.1 %) | just outside |

**Forward vs reverse** (valid legs): wheels 16.02/15.78 forward vs 16.00/15.87 reverse; camera 14.94 forward vs
14.80/14.89 reverse; map 14.96/14.75 forward vs 14.91/14.77 reverse. **No direction effect** can be seen beyond the
leg-to-leg scatter. With only 1-2 legs per direction, this cannot rule out a small one.

**Uncertainty (section 4 rule 0).** Tape +-5 mm. Stop placement was assumed to repeat within 1-2 cm, but the corrected
map shows the three mark-2 stops up to **0.19 m apart** along the corridor and 0.5-0.6 m apart across it, plus the one
~1 m short stop. So each leg carries roughly **+-0.2 m** from where the robot parked. The wheels' +0.94 m excess is
about 5 times that and is a real error. The camera's -0.11 m is inside it. **Camera and corrected map (0.05 m apart)
cannot be told apart by this test.**

## Camera / wheels ratio (does not need the tape)

| ratio | legs | median | spread | drives 3-6 (FINDINGS row 1 / row 2 / row 5) |
|---|---|---|---|---|
| camera / wheels | 1, 2, 4, 5, 6 | **0.932** | 0.925-0.940 | 0.934-0.950 per drive (d3 middle half 0.927-0.957) - **matches** |
| corrected map / wheels | all 6 | 0.933 | 0.930-0.935 | 0.936-0.942 (vs LiDAR map = wheels) - matches |
| blend / wheels | all 6 | 0.974 | 0.966-0.977 | 0.972-0.981 - matches |
| wheels+gyro / wheels | all 6 | 1.001 | 1.001-1.002 | - |

*Plain terms: whatever made the camera map about 6 % smaller than the LiDAR estimate on drives 3-6 shows up here at the
same size. The tape now says it is the wheels, not the camera.*

## Start-to-end gaps (rule 20)
- Tracking alone (database `Node.pose`): **0.171 m**, with one tracking reset (leg 3).
- Corrected map (`Admin.opt_poses`): **0.229 m**, with 472 loop closures in the saved graph and 0 rejected in
  `mapping.log`. On an out-and-back over one line, these closures mostly join repeated passes over the same stretch.
- Neither is the question here. They are recorded only for completeness.

## What this means
- **Summary line:** "A taped 15.00 m test (6 legs, 26 Sept) shows the wheels read about 6 % long and the camera
  within 1 %. The LiDAR estimate takes its size from the wheels, so the ~6 % size difference on drives 3-6 is mostly the
  LiDAR estimate being too large, not the camera map being too small." Any distance that came from wheels or the LiDAR
  estimate (e.g. "drive 6: 243 m", if it came from them) reads about 6 % long. Keep the SE(3) comparison (rigid
  alignment, no size fit; section 4 rule 4), and still write "agreement with the LiDAR estimate" (rule 19).
- **Wheel-size setting (recommendation only; the robot is a colleague's machine, rule 18, not changed):**
  `HUSKY_WHEEL_MULTIPLIER` of about **0.94** (15.00 / 15.94 = 0.941; the valid legs give 0.936-0.950). This is an
  effective wheel radius of about 0.155 m against the nominal 0.1651 m. It feeds `husky_velocity_controller/wheel_radius_multiplier`
  [VERIFIED /opt/ros/noetic/share/husky_control/launch/control.launch:28-30]. [INFERENCE] The same number also turns
  speed commands into wheel speeds, so today the robot probably drives about 6 % slower than commanded, and after the
  change it would match. The colleague should know that before deciding. One floor surface in one session: before
  anyone relies on it, repeat once outdoors on the sidewalk surface, where tyre slip may differ.
- The blend inherits about 40 % of the wheel error (+2.7 %). A wheel fix would bring it close to the tape too.

## Files (made on the Jetson internal disk)
- `fig_legs_vs_tape.png` / `fig_legs.py`: bars per leg against the 15.00 m line and the +-1 % band.
- `legs.json`: every number above. `ext.py` (bag -> odo.npz, 5.8 MB, not kept), `still.py` (parked moments),
  `legs.py` (leg distances), `marks.py` (stop positions along and across the corridor), `corr.tum` (+meta):
  corrected map positions.
- N = 1 session, one surface; 4 valid legs (3 for the camera).
- `timelapse.mp4`: **recorded live during the run**: the live map page (camera map left, LiDAR view right) every 3 s, 221 frames at 10 a second = 22.1 s (copied unchanged from `~/.run_records/s2_scale_01/media/s2_scale_01_timelapse.mp4`, Jetson internal disk, md5 ab162adf...; `recorder.log` there: missed 0, write errors 0). `timelapse_live_index.csv` = that folder's per-frame `index.csv`.
- `timelapse_25/50/75/100.png`: stills at 25/50/75/100 % of the timelapse (video frames 55, 110, 165, 220, counted from 0; extracted with ffmpeg): 13:13:58, 13:16:47, 13:19:33, 13:22:19 Hamilton (index csv). The single frames are not kept here.

**User's account of the short stop (2026-09-26 ~13:50 Hamilton):** asked whether the robot stopped ~1 m short of
mark 1 before leg 5, the driver estimated it at about 0.72 m. The sensors put it at 0.94-1.05 m
(the wheels' 1.04 m is ~0.98 m after their measured +6.2 %). Either way the robot was not at the wheel stop, so legs 4
and 5 stay excluded from the tape comparison; the verdict does not depend on which figure is right.
