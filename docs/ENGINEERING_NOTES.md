# Engineering notes: the stack, the traps, and the working rules

These are the project's hard-won facts, collected during the Mitacs Globalink 2026 internship (project 50124).
Read them before changing a parameter, a launch file or a procedure. Anything marked ✅ was checked in the source code
of the exact versions listed in section 1. Other files in this repository cite these notes as
"ENGINEERING_NOTES.md section N" (the technical sections) or "ENGINEERING_NOTES.md rule N" (the working rules below).

## 0. Working rules

The numbering is kept from the original project notes, so references elsewhere stay valid. Rules that only concerned
the day-to-day running of the internship are left out.

2. Write "section", not the "§" symbol, in text for people.
4. Every parameter, topic or setting is checked on THIS build before it is suggested (`rtabmap --params`, `rostopic list`,
   the actual file) - never from memory.
5. `docs/PROGRESS.html`-style logs are updated when a run completes or the plan moves.
6. Never guess a specification: check the local references and the installed SDK, or look it up.
7. Recordings are verified straight after each run with an integrity check (not only a checksum), and copied off
   the recording machine the same session. The microSD cards used in this project lost data five times.
7a. Every file path in instructions names the machine it is on (your computer, Jetson, robot) - the same relative path can
   exist on several.
8. Any job that outlives one command has a watcher that reports state changes, failures and silent death - not only
   success. Never test or kill jobs with `pgrep -f` / `pkill -f`: they match their own command line. Compare process
   arguments whole, by base name, and kill by PID. Test every process check against a name you know is absent.
9. Delete files no task needs; back up anything that may be needed later and verify the copy first. Never delete the
   only copy of data, anything a running job uses, or someone else's files.
11. Install what the work needs, and record every install in `docs/INSTALLED.md` (what, machine, command, date, why).
   Verify an install by running the tool, not by trusting the installer's log. On the robot's computer, ask first.
13. Every test is watchable from a browser: it writes a one-line `<name>.progress` file (with `N/M` and `<n>s` for a
   progress bar and time left) at least every ~30 s, which the jobs page discovers by itself. Liveness is judged by
   the file's age, never by searching for a process.
14. A result that looks wrong is re-run; say so afterwards.
15. Disk headroom: a transfer of a known size needs 1.2 GB left afterwards; a live recording must reserve its whole
   expected size plus margin, because a recording that runs out of room truncates while still reporting success.
16. The SLAM work and the ZED X vs LiDAR camera paper are separate projects: separate folders, files and commits.
17. Pass `-s` (`--protect-args`) to every rsync against a path with a space in it.
18. The robot's computer holds a lab colleague's LiDAR SLAM stack (`self_navigation`). Use it, credit it, do not
   modify it, and ask before changing anything on that machine. Record LiDAR packets, not expanded clouds
   (a 20-minute run as clouds is about 18 GB).
19. A LiDAR trajectory is an independent estimate, not ground truth. Write "agreement with the LiDAR estimate", never
   "error against ground truth", and state its uncertainty: LiDAR range noise 15-30 mm, the two LiDARs agree to about
   5 mm, and the LiDAR-to-camera mounting offset has never been measured.
20. Every start-to-end gap says which kind it is. A database's `Node.pose` is the camera's tracking alone (loop
   closures never correct it); the corrected positions are `Admin.opt_poses` or a re-optimised graph. Report both,
   never on one axis as one quantity, and give the loop-closure count beside a corrected gap.
21. Status reports: one lead line, one table (`# | step | where | status`), a few bullets.
22. Every drive gets a results pack (numbers, figures, the live-map timelapse recorded during the drive).
23. The Jetson is also the drive computer: its out-of-memory guard kills the mapping program first, so run no heavy
   analysis during a drive, and run analysis under `nice -n 19`.
24. Storage: every finished recording goes to the Autonomous Service Robot Teams folder; a Jetson copy is deleted
   only after the Teams copy has been downloaded again and matches (size and checksum). Keep 13 GB free on the Jetson
   before a drive (the disk-space page on port 8092 shows it).

## 1. The stack — these are fixed, do not suggest upgrading

| Component | Version | Consequence |
|---|---|---|
| Platform | Jetson AGX Orin · L4T R35.4.1 · JetPack 5.1.x · **12 cores** · 29 GB | CUDA 11.4, TensorRT 8.5.2 |
| OS / middleware | Ubuntu 20.04.6 · **ROS 1 Noetic (EOL)** · aarch64 | Python **3.8** is pinned. Anything requiring ≥3.10 is out. |
| Simulator | **Gazebo Classic 11.15.1** (EOL 3 Feb 2025) | No PBR, no motion blur. ROS 2 simulators are not options. |
| SLAM | **RTAB-Map 0.21.13** | Parameter names differ from master. Always check `rtabmap --params`. |
| Vision | OpenCV **4.2.0**, `xfeatures2d=false`, `nonfree=false` | See section 3 — most detectors are unavailable |
| Graph back-ends | g2o ✅ · GTSAM ✅ · Vertigo ✅ · TORO ✅ · **Ceres ✗** | Anything needing Ceres is blocked |
| VIO front-ends | MSCKF / VINS-Fusion / OpenVINS all **false** | Only `Odom/Strategy` 0 (F2M) and 1 (F2F) exist |
| Camera | ZED X stereo, **0.120 m** baseline, 1920x1200, **sensor grid 30 Hz, everything downstream capped at 15 Hz - the SVO recorder included**, GMSL2 | **Corrected again 2026-09-13, and the previous correction's conclusion was WRONG.** This row said "anything fed by the grab loop - the SVO recorder above all - sees 30 Hz". It does not. `zedx_front.yaml` still reads `grab_frame_rate: 30` / `pub_frame_rate: 15`, but the wrapper applies pub_frame_rate to the grab loop **twice**: `zed_wrapper_nodelet.cpp:237` sets `grab_compute_capping_fps = pub_frame_rate` (15.0), and `:3801` sets `ros::Rate loop_rate(mPubFrameRate)` slept at `:4199`. The SVO frame is banked by the SDK **inside `grab`** (`:3999`; the service handler at `:4647-4710` leaves `RecordingParameters.target_framerate` at 0, and `async_image_retrieval` is never set), so the recorder inherits the grab loop's 15 Hz, not the sensor's 30 Hz. **Measured directly 2026-09-13:** a healthy post-restart recording has a modal frame gap of **0.0667 s = 2/30 s in 284 of 313 gaps** - exactly 15 Hz. At 30 Hz the modal gap would be 0.0333 s. **Why the old evidence did not support the old claim:** gaps being whole multiples of 1/30 s only shows the *sensor* captures on a 30 Hz grid, which it does; a 15 Hz consumer taking every second frame off that grid produces gaps that are also multiples of 1/30 s. And a smallest gap of 0.10001 s is 3/30 s = 10 Hz - *slower* than 15 Hz, so it argued against 30 Hz rather than for it. |
| Robot | Clearpath Husky A200 | |

*(Corrected 2026-09-13 — this row read "8 cores". The machine has **12**, verified
three ways on the hardware: `nproc`, `grep -c ^processor /proc/cpuinfo`, and the
count of `/sys/devices/system/cpu/cpu*` entries all return 12. `/proc/device-tree/model`
reads "Jetson AGX Orin Developer Kit". The 8-core figure is the AGX Orin 32 GB
module's spec and does not apply to this board. It matters wherever parallelism is
being reasoned about — the offline replay campaign runs 10 workers, which is
oversubscription against 8 cores but headroom against 12, and that changes what a
load average means.)*

**Never suggest:** ROS 2, gz-sim, Isaac ROS / cuVSLAM (ROS 2 only, JetPack 7.x), Cartographer (no Noetic release), slam_toolbox (needs Ceres), VINS-Fusion, SVO Pro, DSO.

---

## 2. Traps that have already cost time. Do not walk into them again.

### 2.1 ✅ `pointCloudCutoffMax` defaults to 5.0 m and NaNs the depth image
`gazebo_ros_openni_kinect.cpp:106` sets `point_cloud_cutoff_max_ = 5.0`, applied inside **`FillDepthImageHelper`** — not just the point cloud. Every depth pixel beyond 5 m becomes `quiet_NaN`.
**Rule:** always set `<pointCloudCutoffMax>` explicitly, or use `libgazebo_ros_depth_camera.so` which has no far clamp.

### 2.2 ✅ Gazebo silently ignores `<noise>` on a depth sensor
`DepthCameraSensor::Init` re-implements `CameraSensor::Init` and never calls `NoiseFactory::NewNoiseModel`. `grep -i noise gazebo/sensors/DepthCameraSensor.cc` → zero matches. The SDF parses; nothing happens.
**Rule:** noise works on `<sensor type="camera">` and `<sensor type="multicamera">`, never on `type="depth"`. Model the ZED X as **two plain cameras**.
**Also:** camera `<noise><stddev>` is **unitless in [0,1] per channel**. Realistic is `0.002–0.01`. `0.5` ≈ 127 grey levels — nonsense.

### 2.3 ✅ Odometry covariance is CLAMPED, not computed
`Registration.cpp:36`: `COVARIANCE_LINEAR_EPSILON = 1e-8` → σ = **exactly 1e-4 m**. A static double with **no parameter binding**. With `Vis/BundleAdjustment=1` it is scaled down a further 10×.
**Consequence:** in a noise-free sim, a 1 m loop correction reads as a **10,000σ** outlier and `RGBD/OptimizeMaxError=3.0` rejects any closure that deforms *any* link by > 0.3 mm.
**Rule:** do not tune the back-end to work around this. Fix the sensor noise, or floor the covariance by republishing `twist.covariance` (not `pose.covariance` — RTAB-Map prefers twist and takes the running max). **Never write exactly `1.0`** — that value is silently discarded.

### 2.4 ✅ `Odom/ResetCountdown` hides distance
On reset, odometry resumes from the last good pose and RTAB-Map **discards all frames during the outage** (`"Image %d is ignored!"`). The link across the gap **omits every metre travelled**, and carries the epsilon-floor covariance. No mechanism in RTAB-Map notices.
**Rule:** always log reset count and compute true ground-truth displacement during each outage. **Never quote an ATE without resets/100 m beside it.**

### 2.5 ✅ `Grid/Sensor` and `Grid/RangeMax` get overridden when scans are subscribed
`CoreWrapper.cpp:475-495`: if `subscribe_scan` / `subscribe_scan_cloud` / `gen_scan` is true **and you have not set them yourself**, `Grid/Sensor` → 0 (laser) and then `Grid/RangeMax` → 0 (infinite).
**Rule:** set **both** explicitly and identically in every configuration. Never let them be inferred.

### 2.6 ✅ Feature detectors downgrade SILENTLY
`Features2d.cpp:617-695` falls back to GFTT/ORB with a `UWARN`, not an error. On OpenCV 4.2 without contrib, **SIFT is unavailable** (SIFT moved to the main module in 4.4; the guard is `CV_MAJOR==4 && CV_MINOR<=3`).
**Usable `Kp/DetectorStrategy` / `Vis/FeatureType`: {2 ORB, 7 BRISK, 8 GFTT/ORB, 9 KAZE, 10 ORB-OCTREE}.** Default on this build is **8**, not 6.
**Usable `Vis/CorNNType`: {0,1,2,3,5}** (6 SuperGlue needs WITH_PYTHON, 7 GMS needs xfeatures2d).
**Rule:** grep every run log for `"cannot be used"` and mark the trial invalid if present.

### 2.7 ✅ Husky LiDAR env var is `_ENABLED`, not `_ENABLE`
`HUSKY_UST10_ENABLED=1` (Hokuyo, `/front/scan`), `HUSKY_LMS1XX_ENABLED=1` (SICK, mounted rolled 180°), `HUSKY_URDF_EXTRAS` to inject custom sensors without forking `husky_description`.
Sim UST10: 720 samples, 270° FOV, 0.1–30 m, stddev 0.001 m, 50 Hz, ≈ 0.431 m above ground. **Far better than the real UST-10LX (0.06–10 m @ 40 Hz)** — disclose this in any comparison.

### 2.8 ✅ ICP degeneracy detection needs `Icp/Strategy=1`
`RegistrationIcp.cpp:509-517`: with a 2D `LaserScan` and `Icp/Strategy=0` (PCL), point-to-plane is force-disabled, so `icpStructuralComplexity` is **never computed** and degeneracy detection never runs.
**Requires all three:** `Icp/Strategy=1` (libpointmatcher) + `Icp/PointToPlane=true` + normals (`Icp/PointToPlaneK>0`).
Check with `rtabmap --params | grep "Icp/Strategy"`.

### 2.9 ✅ Multi-camera odometry needs OpenGV, and fails SILENTLY for stereo
`util3d_motion_estimation.cpp:365` is `#ifdef RTABMAP_OPENGV`; without it, it logs an error and returns a **null transform**. The friendly guard at `RegistrationVis.cpp:1687` tests `cameraModels.size>1`, which is **empty for stereo** — so multi-stereo skips the warning entirely.
Also `option(RTABMAP_SYNC_MULTI_RGBD ... OFF)` defaults **OFF** in `rtabmap_sync/CMakeLists.txt`.
**Rule:** verify `With OpenGV = YES` in the build log before putting a second camera anywhere near odometry. Prefer the second camera on the `rtabmap` node only (loop closure), with `rgbd_cameras=0` + `rgbdx_sync`.

### 2.10 ✅ `evo` version pin
Releases ≥ 1.32.0 require Python ≥ 3.10 and will not install on Ubuntu 20.04. **Pin `evo==1.31.1`.**
**Already installed on the Jetson — verified 2026-09-23.** Do not "check whether evo is available" again; invoke it.

### 2.11 ✅ `ultralytics_ros` requirement pin is broken
`lap==0.4.0` has **no wheels on PyPI, sdist only**, and fails to build on modern setuptools. Use `lapx` or `lap>=0.5.13`. Also `opencv-python==4.7.0.72` from pip clashes with `cv_bridge`'s system OpenCV 4.2 — install into a venv with `--system-site-packages`.

### 2.13 ✅ Driving a lap in REVERSE kills loop closure — recognition survives, geometry does not
Measured on run 7c, 2026-08-31. Two laps clockwise then one counterclockwise. Closures worked
for the first two laps (167 accepted, highest node 749 of 1,012) and then **stopped completely
for the final 263 nodes / 16.06 m**, even though 98 % of those nodes sat within 0.5 m of
already-mapped ground (median **0.111 m**).

The log says exactly why:
```
Rejected loop closure 421 -> 1003: Not enough inliers 0/20 (matches=17)
Rejected loop closure 421 -> 1006: Not enough inliers 0/20 (matches=33)
```
**Appearance matching found the place every time. Geometric verification got ZERO inliers of
the 20 required, from as many as 33 appearance matches.**

*Plain terms: standing in one spot facing north and facing south are the same place but
opposite views. The recogniser asks "does this look like somewhere I've been" and correctly
says yes. The verifier asks "can I line these two 3D views up" and correctly says no, because
they show different walls.*

**Rules:**
- **Drive every lap in the same direction** unless reversal is the thing being measured.
- **A reversed segment produces no corrections.** Any distance driven reversed is effectively
  unaided odometry, and the drift over it must be reported as such.
- Do not read "many appearance matches" as "closure is close to firing" — the two stages fail
  independently, and `matches=33, inliers=0` is a total geometric failure, not a near miss.
- If reverse-direction closure is ever needed (it will be, on a sidewalk out-and-back), the
  candidates are a second camera facing backwards, or LiDAR, whose scan geometry is
  orientation-independent in a way stereo is not. **Neither is tested here.**

**CONFIRMED by run 8, 2026-09-01, against a prediction registered before the drive.**
Same lab, same settings, three laps, **direction the only variable — no reversal**:

| | run 8 (one direction) | run 7c (2 forward + 1 reversed) |
|---|---|---|
| highest node in a closure | **965 of 973 — 99.2 %** | 749 of 1,012 — **74.0 %** |
| distance left on unaided odometry | **0.48 m** | **16.07 m** |
| loop closures (distinct pairs) | **265** | 167 |

The written criterion was "the highest node in a closure sits in the last ~10 % of the run".
It landed at 99.2 %. **The map was still correcting itself as the robot parked.**

**❌ WITHDRAWN 2026-09-01 — the "discriminator" below does NOT work. Do not use it.**

This section used to say: *the discriminator, for judging any future run, is not the number of
rejections but **zero inliers despite many matches** — recognition succeeding while geometry
fails*, supported by this table:

| | zero-inlier rejections | of those, with 20+ matches |
|---|---|---|
| run 8 | 12 | **3** |
| 7c's reversed lap | 10 | **7** |

**An audit on 2026-09-01 put that claim to independent checkers and it failed on three
counts:**

1. **RUN 7c'S LOG DOES NOT EXIST.** There is no `mapping.log` for run 7c anywhere on this
   machine, and `git log --name-only` over `Week 7/07_run7c` shows one was never committed —
   even though `.gitignore:87` explicitly whitelists run logs as evidence. **Both 7c figures
   above rest on two lines quoted in prose and cannot be recomputed by anyone.**
2. **IT FAILS AGAINST FORWARD-RUN CONTROLS.** The runs that DO have logs were scored the same
   way. Run **9d**, driven forward throughout, scores zero-inlier = 5 with 3 of them at 20+
   matches — **tying run 8's count of 3**. Run **6**, driven forward and the most accurate run
   in the project, scores **2 of 2 = 100 %**, which is *worse* than 7c's reversed lap at 70 %.
   A measure on which the best forward run scores worse than the reversed lap does not
   identify reversal.
3. **The project's own documents disagree** on the count: three say 12 rejections, one says 10.

*Plain terms: the idea was that a rejection with many matches but no inliers means the map
recognised the place and then failed to line it up — the signature of facing the other way.
The idea is still reasonable. What failed is the claim that this number tells a reversed run
from a forward one, because clean forward runs produce it just as often.*

**WHAT STILL STANDS.** Runs **9a (score 0)** and **9b (score 17)** — same route, same night,
same settings, one turned around and one did not. That is the best-controlled pair in the
project and the gap is real. What does not stand is any use of this number on a run that has
no matched partner.

**RULE: do not judge a single run by this count.** Compare only matched pairs, and if a run's
`mapping.log` is missing, the run cannot be scored at all — say so rather than quoting a
figure from prose.

**Caveats that stay attached to this result:** **N=1**. Run 8 drove **91.06 m** against 7c's
**53.71 m**, so the closed-loop *percentages* (0.99 % vs 1.72 %) are not like-for-like — the
absolute errors, 0.904 m and 0.926 m, differ by 22 mm and agree well inside the several
centimetres of parking-by-eye uncertainty. **Only direction was the experimental variable**, so
neither NEURAL nor the database durability change may be credited with any of it.

**LABEL CORRECTION, 2026-09-24:** 0.904 m and 0.926 m are **tracking-alone (odometry) gaps**,
read from the database's `Node.pose`, which loop closures never correct. Run 8's gap **after the
map's corrections is 0.261 m**. The comparison with 7c still holds, because both are the same
kind. But neither may be set beside runs 03–07b, whose gaps are corrected map positions. See
`docs/SOLVED.md`, "The database's `Node.pose` is the camera's raw tracking".

### 2.13a ✅ TURNING AROUND and DRIVING BACKWARD are different things. Only turning hurts.

Measured on **run 9a, 2026-09-01** — lab plus the walkway outside, returning **in reverse
without ever turning the robot**, so the camera faced the same way for the whole return.

**Loop closure did not merely survive the backward leg — that leg closed better than
anything else in the run:**

| leg (cut from the raw wheel odometry, not from memory) | nodes | closures | per node |
|---|---|---|---|
| **the backward return, 53 s / 5.44 m** | 25 | **49** | **1.960** |
| all forward legs | 309 | 41 | 0.133 |

Against run 7c's *reversed* lap: **zero closures over 16 m** of already-mapped ground.

*Plain terms: walking backwards down a corridor, you still face the far end the whole way,
so you see what you saw on the way out. Turning round and walking forward puts you in the
same places looking at completely different things. The map recognises places by what it can
see, so only the second one breaks it. **The wheels' direction is irrelevant; the camera's
heading is everything.***

**Consequence for the project:** a sidewalk out-and-back can come home **in reverse and keep
its map**, with no extra hardware. Slow, but it works.

**Do NOT compare forward against backward legs within one run.** Closure rate depends on
*where* the robot is, not only which way it points — run 8, driven forward throughout, has
forward legs ranging from 0.000 to 1.000 closures per node, because legs over fresh ground
cannot close and legs over mapped ground can. An outbound leg is fresh ground and a return
leg is mapped ground, so a within-run comparison reads **location** as **direction**. Use
`rosrun sidewalk_evaluation segment_run.py --run <id>`, which prints the return-leg figure
and refuses the invalid comparison.

**STATUS updated 2026-09-01: run 9b IS done, and it is now the evidence this rests on.**
9b repeated 9a's route and *turned around* for the return. Same route, same night, same
settings — direction of the camera on the return was the only variable.

| | 9a — returned **in reverse**, never turned | 9b — **turned around** for the return |
|---|---|---|
| rejections with many matches and zero inliers | **0** | **17** |

**This matched pair is what supports section 2.13a, not run 7c.** 7c was a different route on
a different night and its `mapping.log` was never committed, so it cannot be recomputed — see
the withdrawal notice in 2.13. Treat 7c as an anecdote that pointed the way, and 9a-vs-9b as
the measurement.

**Still N=1 per condition.** One pair is not five. Do not quote a percentage difference from
it; quote the counts, and say the route was driven once each.

### 2.12 ✅ Gazebo's built-in `<actor>` is untextured
`walk.dae` has **zero texture maps** — flat solid colours. A COCO detector may not fire on it. `person_standing` / `person_walking` from `osrf/gazebo_models` **are** fully textured (16 texture maps, MakeHuman-derived).

---

## 3. Parameters that DO NOT EXIST — stop suggesting them

| Name | Status |
|---|---|
| `Odom/CovarianceScale` | ❌ Does not exist. Verified by exhaustive grep. |
| `Reg/VarianceFromInliersCount` | ❌ Removed in 0.15.1, no replacement. Warns and is ignored. |
| `Optimizer/Slam2D` | ❌ Renamed to `Reg/Force3DoF`. Deprecated alias only. |
| `Grid/FromDepth` | ❌ Renamed to `Grid/Sensor` (0=laser, 1=depth, 2=both). |
| `odom_tf_linear_variance` / `odom_tf_angular_variance` | ⚠️ Exist, but fire **only** when the incoming covariance is absent or non-positive. The epsilon clamp guarantees positive, so they **never fire**. |
| `RGBD/LoopCovLimited` | ⚠️ Exists, but caps loop info at max odometry info — both ~1e8 here, so **no-op**. |
| `g2o/RobustKernelDelta` | ⚠️ Exists, but referenced only in `optimizeBA`, never `optimize`. **RTAB-Map applies no robust kernel to pose-graph edges.** |
| Any semantic / mask / dynamic-object parameter | ❌ None exists. Use the depth-masking trick (section 5). |
| A `TREE` object class | ❌ Not in ZED SDK, not in COCO. Requires custom training. |

---

## 4. Rules of engagement for this repo

0. **TWO MEASUREMENTS OF THE SAME THING WILL NEVER BE EXACTLY EQUAL. Judge agreement
   against uncertainty, not against zero** (project decision, 2026-08-30).

   A tape reading, a box drawn by hand on an image, and a number computed by the camera are
   three different instruments measuring one quantity. They disagree by construction. The
   question is never "do they match?" — it is **"do they agree to within what these
   instruments can resolve?"**

   *Plain terms: if you measure a table with a ruler and again with a tape and get 1.202 m and
   1.198 m, the table did not change. Reporting that as a 4 mm discrepancy to be explained is
   a mistake; they agree.*

   **How to apply, every time a comparison is made:**
   - **State the uncertainty of each source before comparing.** Rough working figures for this
     project, to be refined when better ones exist: tape to a housing edge **±5 mm**; a box
     drawn by eye on an image **±10 px** (≈ ±20 mm at 2.5 m); the camera's factory calibration
     **~1 %**; a depth reading — use the measured spread, we have it per surface.
   - **Combine them properly.** Two independent sources agree if their difference is less than
     roughly √(σ₁² + σ₂²). Quote that band, do not eyeball it.
   - **Say "agrees to within X" or "disagrees by X, which is N times the combined
     uncertainty."** Never "matches" and never "is wrong" unqualified.
   - **Do not chase a difference smaller than the combined uncertainty.** It is noise, and
     hunting it wastes lab time and invents explanations for nothing.
   - **Do chase one that is several times larger** — that is a real discrepancy. The camera
     centre disagreeing with a back-solved value by 53 px (109 mm at 2.6 m) is far outside any
     reading error and was a genuine defect. The window fractions agreeing to 4 mm across three
     ranges are the same number.
   - **Every reported figure carries its spread**: median with range, or mean ± standard
     deviation, or a quantile. A bare number with no spread is not a measurement.

   **This does not soften the honesty rules.** An unexplained disagreement is still
   unexplained; uncertainty is not a licence to wave one away. It means the threshold for
   calling something a discrepancy is *stated in advance and defensible*, rather than being
   "the digits differ".

1. **Never quote an ATE without stating the reference and its provenance.** A report that cannot prove where truth came from must refuse to print numbers.
2. **Never report N=1.** Minimum N=5, median with (min–max), plus valid/total run count.
3. **Report median and 95th percentile, not RMSE alone.** RMSE is L₂ and saturates on outliers — a handful of tracking discontinuities will dominate it entirely.
4. **Alignment: SE(3) `-a` only. Never `--correct_scale`** — stereo has observable metric scale. Sim(3) is for monocular.
5. **One variable per experiment.** Record once, replay many: the same bag against every algorithm config, so any output difference is caused by the parameter.
6. **Pin explicitly, never infer:** `Grid/Sensor`, `Grid/RangeMax`, `Optimizer/Strategy`, `Kp/DetectorStrategy`. Defaults on this build are compile-time-conditional.
7. **`Optimizer/Robust=true` and `RGBD/OptimizeMaxError` are mutually exclusive.** Never sweep together.
8. **Every run writes provenance:** `git rev-parse HEAD`, `dpkg -l | grep rtabmap`, full `rtabmap --params` dump, world file hash, complete `rosout`.
9. **Every number in the report traces to a run ID.** If it cannot, run the experiment or delete the claim.
10. **Reset the robot to the world origin before every run.** RTAB-Map anchors its map frame wherever the robot starts; a different start silently offsets every error reading.

---

## 5. Known-good patterns

**Stereo camera in Gazebo (gets real noise):** one `<sensor type="multicamera">` with two `<camera>` children **named literally `left`/`right`** (the plugin keys topic suffixes and the right-only baseline off those names) + a single `libgazebo_ros_multicamera.so` plugin with `<hackBaseline>0.120</hackBaseline>`. ✅ Validated live 2026-08-22 (sim ZED X, Phase 1): noise delivered 1.229 grey levels at stddev 0.005; left/right stamps **bit-identical** → exact sync (`approx_sync=false`); right `camera_info` baseline exactly 0.1200 m; odometry σ honestly distributed, zero frames pinned at the 1e-4 clamp. (Two separate `<sensor type="camera">` blocks also honour noise but cannot guarantee identical stamps — use them only as a fallback.) Gazebo cameras are ideal pinhole (`D=0`, `R=I`) so they are **already rectified** — publish to the real wrapper's `left|right/image_rect_color` names, keep `Rtabmap/ImagesAlreadyRectified=true` (pinned in rtabmap_zedx.yaml), skip `image_proc`. Same optical `frame_id` on both `camera_info`. **gzserver renders cameras even with `gui:=false` and needs `DISPLAY`/`XAUTHORITY` exported, or the sensor silently creates nothing** (`Unable to create MultiCameraSensor. Rendering is disabled.`).

**Visual vs LiDAR comparison:** do it inside RTAB-Map via `Reg/Strategy` (`0=Vis, 1=Icp, 2=VisIcp`) and `icp_odometry:=true`, so the graph/optimizer/grid stay identical and only the modality changes. Comparing against gmapping confounds algorithm with modality.

**Dynamic object masking (no RTAB-Map fork needed):** `Mem/DepthAsMask` and `Vis/DepthAsMask` both default **true**, and the mask RTAB-Map uses **is the depth image** — zero-depth pixels get zero features. So: subscribe to YOLO segmentation masks + depth, write 0 into person pixels, republish. Use **segmentation masks, not boxes**. **Mask the SLAM input only** — keep an unmasked depth stream for the costmap.

**Second camera:** feed it to the `rtabmap` node for loop closure, **not** to `stereo_odometry`. Route: `stereo_sync` per camera → `rgbdx_sync` (`rgbd_cameras=2`) → `rtabmap` with `rgbd_cameras=0`. Set `approx_sync_max_interval ≈ 0.015–0.02`. Two ZED X are not hardware-synced. Disable TF on both ZED nodes; let RTAB-Map own odometry. Select by `serial_number`, not `camera_id`.

**Compute:** ZED depth `ULTRA` **while driving**, until YOLO has been measured running alongside `NEURAL`. **`NEURAL` is permitted for stationary and mapping work.** *(Amended 2026-08-30 — this rule previously read "never NEURAL (NEURAL alone ≈ 80 % of the Orin GPU)". That figure does not reproduce. Measured with the full stack — camera, odometry and mapping: NEURAL **~55 % GPU mean, 99 % peak** against ULTRA's **~36 % mean, 94 % peak**; **8.2 Hz against 9.7 Hz**, which is a 15 % cost and not the 45 % first reported here from an unfair camera-alone comparison; **+11 % reaction distance** at 0.6 m/s. Against that, NEURAL is **6–7× more accurate** on low-texture surfaces and its p99 error tail on blank white cardboard falls **546 mm → 46 mm**. An independent Ouster LiDAR agrees with NEURAL's wall width to **4 mm**, so it is not hallucinating geometry. The live objection is that Phase 7 puts YOLO on the same chip and NEURAL leaves ~45 % free where ULTRA leaves ~64 %. **Measure that, do not estimate it.** Evidence: `docs/BENCH_RESULTS_2026-08-30.md`.)* YOLO `yolov8s-seg` at ~10 Hz, not `yolov8m-seg` at 15 Hz — the tracker interpolates. Run everything as nodelets in one manager; two cameras × L+R × 1920×1200 mono8 × 15 Hz ≈ 138 MB/s over TCP-ROS otherwise.

---

## 6. Verify-before-assuming commands

```bash
rtabmap --version                              # build flags: OpenGV, libpointmatcher, GTSAM, nonfree
rtabmap --params > docs/params_baseline.txt    # authoritative for THIS build
rtabmap --params | grep "Icp/Strategy"         # is libpointmatcher present?
dpkg -l | grep gtsam                           # GncOptimizer needs >= 4.1.1
apt-cache policy libceres-dev                  # is Ceres even installable on arm64?
ls /dev/video*                                 # free GMSL2 port for camera 2?
rosbag info <bag> | grep -i ground             # is ground truth recorded?
grep -rn "pointCloudCutoffMax" <urdf/xacro>    # the 5 m NaN clamp
grep -rn "cannot be used" logs/                # silent detector downgrade
python3 -c "from ultralytics import YOLO"      # does the YOLO path work on py3.8?
```

---

## 7. When you are unsure

State it. Mark claims **[VERIFIED — file:line]**, **[INFERENCE]**, or **[UNVERIFIED]**. Do not invent a parameter, node, topic, message or package name — if you cannot verify it exists in this exact version, say so and propose how to check.

**If a change contradicts something in section 2 or section 3, stop and say so before making it.**

---

## 8. Operational procedures live in docs/OPERATIONS.md

Hands-on procedures are in **`docs/OPERATIONS.md`** — read the relevant section
before any hands-on session: remote GUI/VNC access (the DISPLAY/XAUTHORITY
derivation, the ForwardX11 trap), Gazebo crash-vs-hang triage, the
texture-not-colour rule for worlds (with the live ORB feature check), stale
rosparam cleanup, the memory/queue-depth incident and its standing mitigations
(queue_size=5, rviz:=false on recorded runs, earlyoom), editing the ZED X mount
pose, and the step-by-step instruction format used for hands-on work
hands-on work.

The solved/unsolved log (section 0.2) lives at `docs/SOLVED.md` and
`docs/DO_NOT_REPEAT.md`. 
