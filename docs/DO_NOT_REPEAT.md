# DO_NOT_REPEAT.md — things that failed

Format per entry: **what was tried → how it failed → why → what to do instead.**
Read this (and `SOLVED.md`) before proposing any change (ENGINEERING_NOTES.md section 0.2).

---

### Raising `Odom/ResetCountdown` 1 → 10 to "fix" 147 discontinuities (2026-08-20)
- **Tried:** raised the countdown so odometry resets less often.
- **How it failed:** reduced the visible reset *count*, but each surviving reset now follows 10 failed frames instead of 1 — each unmeasured gap is ~10× longer, and RTAB-Map discards every frame during the outage while the spanning link omits the distance travelled (ENGINEERING_NOTES.md section 2.4).
- **Why:** optimised the visible metric, not the one that matters.
- **Instead:** fix the *cause* of tracking loss (features, noise, frame yield — Phase 1); always log reset count AND true ground-truth displacement per outage; never quote ATE without resets/100 m beside it.

### Tuning the back-end around the covariance clamp (2026-08-20/21)
- **Tried:** `RGBD/OptimizeMaxError` 3.0 → 8.0 to accept loop closures the clamp made look like outliers.
- **How it failed:** 8.0 still rejected everything (errors were ~1 m ≈ 10,000σ against the 1e-4 clamp, not 31σ); the underlying confidence was fabricated either way.
- **Why:** the clamp (`Registration.cpp:36`, σ = exactly 1e-4 m, no parameter binding) makes ANY threshold either useless or dangerous; the acceptance test is broken, not the threshold.
- **Instead:** fix the sensor noise (two plain cameras with `<noise stddev=0.005>`, Phase 1.3) or floor the covariance by republishing `twist.covariance` — never exactly 1.0 (silently discarded). ENGINEERING_NOTES.md section 2.3.

### Vertigo robust optimisation as a shortcut (run #5, 2026-08-21)
- **Tried:** `Optimizer/Robust=true` (Vertigo switchable constraints) to let closures in despite the clamp.
- **How it failed:** 2,111 closures accepted, ATE 0.141 → 2.560 m — 18× worse.
- **Why:** Vertigo's switch prior is hard-coded σ=1.0 and is not scale-invariant: with information ~1e8, `s* ≈ 0.01` — every closure, correct ones included, gets switched off; plus `Optimizer/Robust=true` requires `RGBD/OptimizeMaxError=0`, disabling the only sanity check. Suspected numerical ill-conditioning on top (unconfirmed — the run's logs captured stdout only; see SOLVED.md).
- **Instead:** never sweep `Optimizer/Robust` together with `RGBD/OptimizeMaxError` (ENGINEERING_NOTES.md section 4.7); fix the covariance first.

### Comparing runs #4 and #5 to each other (2026-08-21)
- **Tried:** "the map got 18× worse" from run #4 (235 m) vs run #5 (376 m).
- **How it failed:** different trajectories, different lengths, both hand-driven — confounded before the back-end is even considered.
- **Instead:** both headline numbers are retired. One variable per experiment; record once, replay many (ENGINEERING_NOTES.md section 4.5); scripted waypoint driver (Phase 2) before any A/B claim.

### Quoting any number from `mcity_v2_run2b` (2026-08-22)
- **Tried:** ATE RMSE 26.44 m / drift 18.96% / RPE 1.69 m reported from the first outdoor run.
- **How it failed:** the report's reference was labelled with the self-test-only "Synthetic" description; the RMSE is saturated by 12 tracking-loss teleports (~12.5 m each — RPE 1 m and RPE 1 s identical to four decimals = same outliers dominating both); the Sim(3) 0.4648 "calibration" warning is almost certainly teleport-driven (Phase-0 diagnostic settles it).
- **Instead:** the run's TUM files remain valid *diagnostic* data; its headline numbers are unquotable. Re-evaluate under `simulator_ground_truth` with median/p95 and segment diagnostics (Phase 0 Part E).

### Deep message queues as a "safety margin" (2026-08-21)
- **Tried:** `queue_size 30` on the RGBD pipeline (six queues × 16 MB frames).
- **How it failed:** ~2.9 GB of buffered frames; contributed to the Jetson memory livelock; also fed odometry seconds-old frames, which *causes* tracking loss.
- **Instead:** queue_size 5 (see SOLVED.md); on real-time topics, dropping old frames is correct behaviour.

### RViz left running during recorded drives (2026-08-21)
- **Tried:** monitoring a recorded mapping drive with RViz (MapCloud + Camera displays).
- **How it failed:** 11.14 GB RSS → machine-wide livelock, 29 min network outage, trip to the lab.
- **Instead:** `rviz:=false` on every recorded run; watch the monitor CSV/STATUS and the Gazebo window. See SOLVED.md for the full mitigation set.

### Gazebo `<noise>` on the depth sensor (2026-08-20)
- **Tried:** `<camera><noise><stddev>0.5` on the `depth` sensor to give SLAM realistic covariance.
- **How it failed:** delivered temporal noise 0.0002 grey levels, max frame diff exactly 0 — parsed, silently ignored.
- **Why:** `DepthCameraSensor::Init()` never calls the noise factory (ENGINEERING_NOTES.md section 2.2). Also 0.5 would be nonsense anyway — the stddev is unitless in [0,1]; realistic is 0.002–0.01.
- **Instead:** model the ZED X as two plain `<sensor type="camera">` with `<hackBaseline>0.120</hackBaseline>` on the right (Phase 1.3; ENGINEERING_NOTES.md section 5).

### Persistent x11vnc autostart (2026-08-17)
- **Tried:** installing an autostart entry so x11vnc survives reboots.
- **How it failed:** blocked by the permission classifier as a standing/security-relevant change.
- **Instead:** start x11vnc fresh each login per `docs/OPERATIONS.md`; do not retry the autostart.

### `rospy.wait_for_message` for measurements (2026-08-21)
- **Tried:** sampling camera frames via `wait_for_message` at six map locations.
- **How it failed:** byte-identical results everywhere — stale cached frames; nearly produced "Mcity fails everywhere".
- **Instead:** subscribe, discard ≥10 callbacks, measure ≥8 fresh frames, report the median.

### `pkill -f` over SSH (recurring)
- **Tried:** `pkill -f roslaunch` and similar, ~4 times.
- **How it failed:** the pattern matches the SSH command line carrying it — killed the shell mid-script.
- **Instead:** capture PIDs by exact process name first, `kill` by number only.

### "Timelapse froze because grid_map is gated on loop closure" (2026-08-22)
- **Tried (as a hypothesis, not code):** proposed that RTAB-Map only republishes `/rtabmap/grid_map` on graph-optimization/loop-closure events, and since Phase 1 had 0/1144 accepted, the grid never rebuilt past its first snapshot.
- **How it failed:** disproven by direct measurement. A short diagnostic run with only straight driving and zero loop closures still grew the grid normally (26/54 unique frames) once a persistent subscriber was added — loop closure status was irrelevant. The real cause was the *recorder's* one-shot connect/grab/disconnect subscription pattern (see SOLVED.md), not anything about the graph optimizer.
- **Instead:** when a topic looks frozen, test with a persistent subscriber before proposing a mechanism inside RTAB-Map's optimizer — the standing-subscriber requirement is cheaper to rule in/out first and turned out to be the actual answer.

### Pushing `Odom/ImageDecimation` past 4 to close the 95% frame-yield gate (2026-08-23)
- **Tried:** swept `Odom/ImageDecimation` to 6 and 8 (N=3 each, same route/config as the validated A+Y arm — RGB-D, no noise, decimation=4 baseline), on the theory that the already-working 2→4 lever (which raised yield 83.3%→93.2% and *improved* ATE) would keep paying off if pushed further.
- **How it failed:** decimation=6 (N=3 valid: `p1_dec6_r2/r3/r4`) did clear the yield gate — 97.5–98.04% (median 97.84%) — but ATE med regressed to 0.077–0.529 m (median 0.403 m, a 4.6× regression on the 0.087 m baseline), with RMSE (0.108–1.087 m, median 0.905 m) sitting far outside the baseline's own 0.076–0.298 m run-to-run spread in 2 of 3 runs. Resets appeared (0.92–1.84/100 m) where the baseline had zero in every campaign run. Decimation=8 (N=3: `p1_dec8_r1/r2/r3`) failed *both* gates: yield 88.45–91.95% (median 88.89%, worse than the 93.2% baseline it was meant to beat) and ATE med 1.877–2.236 m, with resets 18.7–26.2/100 m.
- **Why:** decimation doesn't only cut compute per processed frame, it also widens the effective baseline between the frames that *are* processed. `frame_yield.json`'s `yield_odom_all_pct` vs `yield_odom_valid_pct` gap shows this directly: at decimation=4 the two are identical every run (0 invalid poses — all loss is frames silently never attempted, the documented compute floor). At decimation=6/8 a real gap opens (20 invalid poses/run at dec=6, up to 283 at dec=8) — genuine tracking loss under this route's in-place 180° turns, not just skipped frames. Past decimation=4, the robustness cost dominates the compute saving. CPU profiling during a decimation=4 drive (`p1_cpu_profile_r1`) also rules out the "just needs more scheduling headroom" theory: system-wide utilization averaged 84.7% of 8 cores (≈1.2 idle on average) and `rgbd_odometry` itself averaged only ~51% of one core — this is not a contention problem, so core-pinning wouldn't have helped either.
- **Instead:** `Odom/ImageDecimation` stays at 4 (restored and verified after the sweep). 93.2% is the practical frame-yield ceiling for this compute budget, detector (`Kp/DetectorStrategy=8`, GFTT/ORB — confirmed CPU-only: this OpenCV 4.2.0 build has zero CUDA modules compiled in, `cudafeatures2d`/`cudaimgproc`/etc. all "Unavailable"), and route motion profile. Full data: `docs/PHASE1_RESULTS.md` addendum (2026-08-23); run artifacts `~/.run_records/p1_dec{6,8}_r*/` and `~/.run_records/p1_cpu_profile_r1/` on the Jetson.

### Reconstructing a timelapse from a saved database with `rtabmap_util/data_player` (2026-08-24)
- **Tried:** three attempts to rebuild the "map growing over time" video for `office1_explore_03` after the original sim run was gone, by replaying the saved database through a fresh RTAB-Map instance (`slam_from_database.launch`: `data_player` → `stereo_sync` → `rtabmap`, scratch output database, `map_manager.py timelapse` sampling `/rtabmap/grid_map` alongside).
- **How it failed, in three distinct ways:**
  1. **v1** — assumed `data_player`'s `rate` meant real time like `rosbag play --rate`. It does not (see SOLVED.md): the ~78-minute drive replayed in under 5 minutes, and the sampler then sat retrying against an already-dead topic for the rest of its `--duration`. Nothing reported either fact.
  2. **v2** — `data_player` and `stereo_odometry` **both** published `odom → base_link` (552 `TF_REPEATED_DATA` warnings). Fixed by `visual_odometry:=false` + `odom_topic:=/odom`, verified down to **0** warnings. This was a real bug, **but not the cause** — the result was byte-identical.
  3. **v3** — same outcome as v2 exactly: `rtabmap-info` on the scratch database reports **279 poses and 0 links**, Working Memory pinned at 1 node, 247 sampled frames with **1 unique checksum**.
- **Actual root cause (measured, 2026-08-24):** `data_player` **does not publish `/clock`**. The topic is advertised but carries **no messages** — `rostopic hz /clock` reports "no new messages", both with the parameter unset and with `_clock:=true` explicitly set (the parameter appears in `rosparam list` and changes nothing). With `/use_sim_time=true` and no clock source, every node's `ros::Time::now()` stays frozen at 0, so TF lookups at the recording's real stamps (315.9 s, 316.9 s, …) can never resolve. RTAB-Map therefore receives no usable odometry, appends each node **unlinked**, and the local map never exceeds one node — so the occupancy grid never grows and every sampled frame is the same near-empty picture.
- **Instead:** **do not reconstruct a timelapse after the fact.** Record it **live, during the drive**, with `map_manager.py timelapse` sampling `/rtabmap/grid_map` while the real mapping session runs — that path is already fixed and proven (2026-08-22, 154/168 genuinely unique frames on a real drive). The replay path is only ever needed when a run's timelapse was not recorded at the time, which is exactly the situation the standing "never start a mapping run bare" rule exists to prevent. If a database replay is genuinely required later, the unsolved prerequisite is a working simulated clock — publish `/clock` from a separate node, or run with `use_sim_time:=false` and confirm TF resolves — and that must be verified **before** any sampler is attached.
- **Guard that worked and should be kept:** the v3 pipeline gates on evidence before building any video — it reads the graph's link count and the unique-frame count and refuses to emit output if the map did not build. v2's failure reached "DONE" with a plausible-looking 198 KB `.avi` of nothing, and was caught only by a manual check. Never let a frozen timelapse reach the user looking finished.

---

## Normalising closed-loop error as "percentage of path driven" (2026-08-25 → refuted 2026-08-26)

- **What was tried:** with runs 3, 4 and 5 giving 0.77 m, 11.47 m and 4.19 m — a 15x spread that read
  as an unreliable system — the errors were divided by the distance driven, giving 2.3 %, 4.9 % and
  5.0 %. Runs 4 and 5 landed nearly identical, which looked like the noise had been explained away.
  It was written up as "the honest form" and the claim was made that **~5 % of path is what this
  camera-only configuration does.**
- **How it failed:** run 6 drove **96.71 m — 15 % further than run 5 — and finished at 0.22 m**,
  0.23 % of path, **19x more accurate than run 5 over a longer drive.** A rule that says error grows
  with distance driven cannot survive that.
- **Why:** distance driven was never the causal variable. RTAB-Map pins its map where the robot
  starts, and the error is the disagreement with that pin. What matters is **how far the robot ever
  gets from it** — near the pin the camera keeps re-recognising mapped places and each recognition
  pulls the estimate back; far from it, drift accumulates with nothing familiar in view to correct
  against. Runs 4 and 5 matching at ~5 % was a coincidence of two runs with similar path-to-radius
  ratios, not a law.
- **Instead:** report the closed-loop error **with the greatest distance from the start beside it**.
  It is already in every run's `monitor.csv` — `sqrt(est_x² + est_y²)` maximised over the run — and
  costs nothing to extract. Ordered by that radius the four runs are monotonic:
  **4.71 m → 0.77 m · 4.87 m → 0.22 m · 10.38 m → 4.19 m · 12.92 m → 11.47 m.**
- **Still unproven, and cheap to settle:** radius is entangled with area covered, revisit rate and
  duration across these four runs. **Drive the same total distance at two different radii** — a tight
  repeated loop against a long excursion — before treating the relationship as established.

## Blaming low visual texture for the closed-loop error (2026-08-25 → refuted 2026-08-26)

- **What was tried:** run 5's feature count collapsed from 220 to 78 six times in twenty minutes,
  once at the exact second the camera's tracker diverged. This was written into `RUN6_PROCEDURE.md`
  as "a strong candidate for what drives the ~5 % of path error", and the drive was steered towards
  visually detailed areas on that basis.
- **How it failed:** run 6's full log (22,224 measurements, the first run to keep one) shows **mean
  230 features with 12.9 % of samples below 100**, degrading through the run — 402 in the first five
  minutes down to 132 by minute 27. **The worst texture conditions of any run measured produced the
  best accuracy of the project.**
- **Why the wrong conclusion looked right:** run 5's numbers came from glancing at a terminal, which
  shows a *window*, not a distribution. Six collapses seen on screen said nothing about how the other
  thousands of samples behaved, because nobody had them.
- **Compounding it:** during run 6 itself, "mean 403 features" was reported live from the watcher's
  rolling window — that was the first five minutes, the best stretch. The full-run mean is 230. **A
  partial window was quoted as if it characterised the whole run.** Any feature statistic quoted
  before the run ends is a window; say which window, or wait.
- **Instead:** feature counts are worth logging and worth watching for a total dropout, but do not
  attribute an accuracy result to them without the whole run's distribution in hand.

## Measuring "how far the robot ranged" from the SLAM estimate itself (2026-08-26, refuted same day)

- **What was tried:** to explain the 0.22 / 0.77 / 4.19 / 11.47 m closed-loop spread, the greatest
  distance each run reached from its starting point was computed as `max(sqrt(est_x² + est_y²))` over
  `monitor.csv` and correlated against the error. The four runs appeared to line up monotonically, and
  it was written up as the explanation, replacing the "percentage of path" rule refuted an hour earlier.
- **How it failed:** the closed-loop error is the **final** value of the *same expression on the same
  column*. Independent variable and dependent variable were two order-statistics of one number series
  — they cannot be independent by construction. Rebuilding run 5's optimised pose graph from its
  database shows the size of the damage: **at the exact instant run 5 reported 10.38 m from the start,
  the solved map places it 0.33 m from the start.** Run 5 never ranged 10 m; it drifted 10 m. Its true
  maximum reach was 6.93 m. (The reconstruction reproduces RTAB-Map's own final answer for run 6 to
  **2 mm**, so it is the reconstruction that is trustworthy, not the live reading.)
- **Two further independent failures, either one fatal on its own:**
  - **Run 4 was never a radius data point.** Already diagnosed a day earlier as duplicate-signboard
    perceptual aliasing. **Its raw visual odometry closed the loop to 1.83 m after 234.56 m — 0.8 %
    drift, the best of any run.** The 11.47 m was injected by false closures. Remove it and the claim
    rests on three points.
  - **The only matched pair inverts it.** Run 3 at 4.71 m → 0.77 m; run 6 at 4.87 m → 0.22 m.
- **A premise asserted alongside it was false.** "Nothing was tuned between runs" — but
  `RUN6_PROCEDURE.md` line 74, and no earlier procedure, told the driver to *prefer visually detailed
  areas*. The best-scoring run was the only one with a changed driving policy. **Check the procedures
  before ever claiming two runs differ in one variable.**
- **The general trap:** almost every quantity in `monitor.csv` is derived from the SLAM estimate. Any
  "explanation" built from two of them risks explaining drift with drift. Before correlating, ask
  which column is independent of the error. **Distance driven is clean** (it equals raw-odometry path
  length; optimisation changes it under 1 %). **Radius is not.**
- **Instead:** set the independent variable with a **tape measure before the robot is switched on**,
  not from the estimate afterwards. And note the deeper problem no re-analysis fixes: across these
  four runs, bounding-box area is **rank-identical** to radius and speed ties it exactly, so the data
  cannot distinguish them at all. **The design, not the analysis, is the limit.**

## Mixing live monitor counters with database link counts in one table (2026-08-26)

- **What was tried:** the loop-closure column of the four-run comparison took runs 3 and 4 (88, 134)
  from the live `run_monitor` counters and runs 5 and 6 (142, 422) from `rtabmap-info`'s database
  link counts.
- **How it failed:** where both sources exist they disagree by **−19 % and +33 %** (run 5: 175 live
  vs 142 database; run 6: 316 live vs 422 database). The column was not internally comparable, and
  any closures-per-metre figure computed across those rows mixed units.
- **Instead:** pick one source for a whole column and state which. They measure different things —
  the live counter reflects what `/rtabmap/info` announced during the run, the database count
  reflects the links surviving in the final optimised graph.

## Trusting an integrity check run immediately after writing the file (2026-08-26)

- **What was tried:** the standing rule "verify every recording's integrity immediately after the
  run". It was followed for runs 5 and 6, and both reported PASS.
- **How it failed:** the check reads the **page cache**, not the card. The file has just been
  written, so it is entirely in memory; SQLite validates the correct in-memory copy while the
  physical write may already be damaged. `lab_map_06.db` passed at 01:07 and failed the same evening
  with no writes in between.
- **Instead:** `sync`, drop the page cache (`sudo sysctl -w vm.drop_caches=3`), *then* check. This
  is now built into `verify_run.py`. A check that could not drop the cache must be reported as
  unproven, never as a pass.
- **Wider consequence:** do not describe a file as "verified intact" anywhere in this project unless
  the cache was dropped. Several past claims in the project records and run write-ups do not meet that
  bar and have been corrected.

## Re-measuring after applying a correction, and calling it verification (2026-08-29)

- **What was tried:** the camera's mounting tilt was measured by fitting a plane to the floor
  (`measure_camera_tilt.py`), the resulting 3.24° was applied as `cam_pitch`, and the fit was re-run.
  It came back at −0.03° with the floor flat, and that was presented as confirming the camera is
  physically tilted.
- **How it failed:** it confirms nothing about *cause*. `base_footprint` is bolted to the robot body,
  not to the world, so the fit constrains the same quantity both times — the **sum** of (camera tilt
  in its mount) + (robot body pitch on its wheels) + (floor-patch slope). Rotating the model by the
  fitted angle and re-fitting **must** return ~0 for any decomposition of that sum, including ones
  where the correction is a brand-new error. An adversarial check simulated five physically distinct
  worlds on the same iso-contour: all five gave −3.24° before and −0.00° after. Identical output,
  opposite correct actions.
- **A second, smaller instance in the same comment:** "atan(226 mm / 4 m) = 3.23° confirms the normal
  independently" — it does not. `rise/range` is identically `−n[0]/n[2] = −tan(pitch)` computed from
  the same fitted normal, with the plane offset cancelling out. One number restated as two witnesses.
- **Instead:** verify with an instrument that shares no failure mode. Here that was gravity —
  `check_gravity.py` reads the ZED X accelerometer, which measures true vertical and owes nothing to
  the transform tree. It separates the three causes by >3° each and settled it in 30 seconds with no
  hardware movement. A re-measurement through the corrected model is a **deployment check** (it
  proves the edit landed with the right sign and magnitude) and should be described only as that.
- **This is the second instance of the same failure mode.** The first was the withdrawn radius
  finding (see "Measuring how far the robot ranged from the SLAM estimate itself", `a565d31`). The
  pattern is: *two statistics computed from one fit, presented as independent agreement.* Before
  claiming corroboration, ask what would have to be true for the second number to differ from the
  first — if the answer is "nothing", it is the same number.

## Taking a datasheet's field of view over the camera's own calibration (2026-08-29)

- **What was tried:** `perception_common.ZEDX` carried `fov_h_deg 80.0` / `fov_v_deg 52.0`, which
  looked like guesses. They were replaced with the Stereolabs datasheet's stated
  "Max. 73 deg (H) x 45 deg (V)", with a long justification arguing that 52 exceeded the stated
  maximum and so had to be wrong at every resolution.
- **How it failed:** the camera's own factory calibration, read live from `camera_info` at
  1920x1200, reports **74.67 x 50.98** rectified and **74.17 x 50.58** raw. The raw stream carries
  non-zero distortion coefficients and still reads ~74 x 51, so it is not a rectification artefact.
  The implied focal length is 3.78 mm (fx 1258.5 px x 3 um pixels), not the datasheet's 4.6 mm.
  **The superseded 52.0 was within one degree of the truth; the "correction" moved it six degrees
  the wrong way**, overstating the forward blind zone by 175 mm (1.442 m against the true 1.268 m)
  — and that inflated figure was reported as a finding.
- **Why:** the datasheet's numbers are not a pinhole specification and are not internally
  consistent — 73 and 45 imply an 80.6 deg diagonal, not the 87 deg printed. They describe a
  product line, loosely, as maxima. They are marketing geometry, not calibration.
- **Instead:** for anything computed from what the camera OUTPUTS — where the ground first
  appears, blind-zone depth, ground sampling per pixel, any projection — read the camera:
      rostopic echo -n1 /zedx_front/zed_node/left/camera_info
      HFOV = 2*atan(width/(2*fx))    VFOV = 2*atan(height/(2*fy))
  Check `left_raw/camera_info` too: if its distortion coefficients are non-zero and it agrees,
  the value is the lens rather than an artefact of rectification. The datasheet stays the right
  source for what cannot be measured this way — IMU noise densities, depth range, mechanical
  dimensions, the baseline.
- **The wider rule, and this is the second time today:** a more authoritative-looking source is
  not automatically a more accurate one. The morning's error was trusting a plane fit that could
  not distinguish three causes; this one was trusting a document over the instrument. **Ask what
  the source actually measured.** A datasheet measured a product family; `camera_info` measured
  this camera.
- **What caught it:** the unconditional docstring check added to `obstacle_segmenter.py` earlier
  the same day. It recomputes every published figure from the live constants and refuses to pass
  when they drift, so changing the field of view failed exactly 4 of 79 checks — the four that
  depend on it — while the five mount-derived ones still passed. A value-gated version of that
  same check had been silently skipping for weeks.

---

## Comparing three boards while one of them was leaning 40 degrees (2026-08-30)

- **What was tried:** two days of "blank surface versus printed surface" measurements — depth
  spread, plane residual, occupancy cells, floor-versus-obstacle classification — treating the
  three boards as differing only in what was printed on them.
- **How it failed:** one board was leaning back about 40 degrees against the table behind it
  while the other two stood at 6-10. Every comparison mixed *surface* with *lean* and could not
  separate them. **Three published claims had to be withdrawn**: "blank white reads 5.5x
  noisier" (it is 1.65x once upright), "the map classifies 55 % of a blank board as floor" (7 %
  once upright — and on a board tipped 40 degrees, calling it ramp-like is arguably correct),
  and "the black board collapses to 1 cell" (does not reproduce in either arrangement).
- **Why nothing caught it:** the depth statistics, the cell counts and the occupancy pictures
  are all consistent with a leaning board. Nothing in them is *wrong*. The lean was simply
  never measured, so it never appeared beside the numbers it was driving.
- **Instead:** `sidewalk_evaluation/scripts/board_geometry.py` runs **before** every measurement
  session and fits a plane to each target, reporting distance, **lean**, flatness and width. It
  prints a warning when the leans differ by more than 8 degrees, and every results row now
  carries the lean it was taken at. A leaning board is still *flat*, so the plane residual
  separates "tipped over" from "badly measured" — two things that look identical in a depth
  histogram and mean opposite things.
- **The general rule:** before comparing several things, measure the property you are assuming
  they share. Not the property under test — the ones you believe are held constant.

## Four analysis artefacts in one night, all the same shape (2026-08-30)

Each was a real measurement, correctly taken, **of the wrong thing**. Recorded together
because the shape matters more than any one of them.

1. **An edge detector that used depth to find the edge of the board whose depth was under
   test.** It rejected readings beyond 3.9 m, so on the board that read badly it skipped the
   real top rows and placed the edge too low — *precisely on the board where it mattered*.
   Third instance of this circularity class in this project.
2. **Measuring the edge with almost no resolving power.** The boards' tops sit 0.29 m above the
   lens, near the horizon, where range barely moves where a thing lands in the picture — about
   **10 pixels per metre**. The bottom edge, 0.70 m *below* the lens, gives **79 px/m**.
   *Rule: for ranging from a boundary, use the edge furthest from eye level.*
3. **Reproducing the grid's noise filter without its voxel step.** Reported 39-68 neighbours
   per point and concluded "density is not the problem". Both described a cloud the map never
   sees. Caught by reading this build's own parameter help text instead of trusting the
   pipeline order from memory.
4. **Judging an organized point cloud after stripping its invalid points.** The ZED publishes
   1200x1920, one point per pixel. Writing it to a PLY with NaNs removed destroyed that
   structure; the survivors were then located through a *guessed* coordinate convention, and
   the tool reported a board largely absent from the cloud. It was not: compared pixel by pixel
   against the depth image on the same frame, **0.00 % dropped**.

**What they share:** every one substituted a derived, interpreted quantity for a direct one.
**The counter:** when two sensors or two stages can be compared *on the same index* — same
pixel, same timestamp, same frame — do that, and no coordinate convention can be got wrong.
`sidewalk_evaluation/scripts/depth_vs_cloud.py` is the worked example.

**Also recorded:** a tool with board names hard-coded to positions kept reporting the black
board's numbers under the white board's label after the two were swapped. Windows are now
passed on the command line and every row records **both** position and content.

## Ctrl+C on a mapping run can leave a database with no map (2026-08-30)

- **What was tried:** stopping a stationary mapping run with `Ctrl+C`, then exporting its point
  cloud with `rtabmap-export --cloud`.
- **How it failed:** *"There are no saved optimized poses in the database"*, then *"The are no
  odometry poses!? Aborting"*. The file is 5.15 GB and opens fine; `rtabmap-info` reports 817
  nodes — but `Nodes size: 111 KB`, 0.00 % of the file. **The images are there and the map is
  not.** This is not corruption; the file passes every integrity check.
- **Instead:** two things. Use `--opt 2` ("use optimized poses already computed in the
  database"), which works on runs that *did* shut down cleanly — `--opt 0` needs raw odometry
  poses these databases do not carry. And do not rely on a database for artefacts you can save
  live: `sidewalk_slam/scripts/save_map_artifacts.py` writes the 2D map (`.pgm`/`.yaml`/`.png`/
  `.npy`), the 3D cloud (`.ply`) and every setting used, from the running node, before anything
  is stopped.

---

## Copying a 9.6 GB database to /tmp made the Jetson unreachable (2026-08-30)

- **What was tried:** extracting the stored 2D occupancy map from each of seven bench
  databases, by replaying each one through an `rtabmap` node in localization mode. The script
  copied each database to `/tmp` first — deliberately, because rtabmap opens databases
  **read-write** and those seven files were the only record of the day's conditions.
- **How it failed:** the copy step never checked the file's SIZE. `bench_fix_normalK40.db`
  was **9.6 GB** (it had grown that large because that run was left going while tooling was
  written). The copy took the internal disk from 16 GB free to **5.9 GB**, and rtabmap then
  tried to load a 9.6 GB map on top of it. The machine did not crash — it **kept answering
  pings at 0.7 ms and kept serving the storage dashboard in 2.8 ms**, while every new SSH
  login timed out. A process already resident in memory was fine; anything that had to fork
  and touch disk stalled. That is swap thrashing, and it looks nothing like a crash.
- **What made it diagnosable:** the storage dashboard. It was already running, so it answered
  instantly and reported both disks while SSH was unusable. Without it there would have been
  no way to see disk state at all. **Keep it running.**
- **What made it recoverable:** nothing was mid-write. No recording, no mapping run, and every
  database lived on the microSD card which the script only ever read. A reboot returned the
  machine with 16 GB free (Ubuntu clears `/tmp` on boot), 27 GB of 29 GB memory free, zero
  stale ROS processes, and the camera services already active.
- **Instead:** the script now refuses any database that would leave under 10 GB free, prints
  the size and the free space in its state line, and deletes each copy immediately rather than
  carrying it forward. And the watcher on it now alarms on **low memory as well as low disk** —
  the previous watcher checked only disk, so it reported "died silently" without ever showing
  the cause.
- **The wider lesson, and it is the second time today:** a guard that protects one thing can
  break another. Copying the database protected the originals and nearly cost the machine.
  **Before adding a safety copy, check what the largest thing you will copy actually is.**
- **Related but different:** the 2026-08-21 incident, where the Jetson left the network
  entirely when the SLAM stack started. That one is still open and the cause was never settled
  between memory and power. **This event is evidence for the memory explanation** — same
  machine, same class of pressure, and here the network stack stayed up while userland did
  not, which power loss cannot produce.

## Overwriting a shell script while it is running (2026-08-31)

- **What was tried:** fixing a small bug in `make_run_report.sh` and `scp`-ing the corrected
  file to the Jetson while an earlier copy of that same script was still executing there.
- **How it failed:** `line 164: syntax error near unexpected token 'done'`. **Bash reads a
  script incrementally as it runs** - it does not load the whole file first. Overwriting the
  file makes bash resume at the same BYTE OFFSET in the new content, which lands mid-statement.
  The script had already produced its 2D map, so that survived; the 3D export never ran, and
  the failure looked like a logic bug in a section that was in fact never reached.
- **Instead:** never write over a script that is executing. Either wait for it, or copy to a
  NEW name (`make_run_report_v2.sh`) and run that. The same applies to any file a running
  process reads lazily.
- **Why it was hard to see:** `bash -n` passed on the corrected file, and the error pointed at
  a line that was syntactically fine in BOTH versions. The mangling exists only in the
  half-old, half-new byte stream bash was actually reading, which never exists on disk.

## Registries lie: four ways to be told something exists when it does not (2026-08-31)

Four failures in one evening, all the same shape - **asking a registry what exists instead of
asking reality**:

| check used | why it lied |
|---|---|
| `rosservice list \| grep /publish_map` | a dead node's service registration lingers on the master; the list said yes and the CALL then failed |
| `rostopic list \| grep /rtabmap/grid_map` | same ghost. Sent a script down its "live node" branch after rtabmap had exited, to wait on a topic with no publisher |
| `ps -eo comm \| grep -cx make_run_report.sh` | **comm truncates process names to 15 characters**; the name is 18, so it could never match and reported a live script as dead |
| `[ -f "$OUT/map.png" ]` | a zero-byte file exists. One was produced when the machine hung mid-write, and passed as a success |

**What to do instead, in each case:**
- a service: **call it**, in a retry loop, and treat the successful call as the readiness signal
- a topic: count its **publishers** (`rostopic info ... | grep -c '^ \*'`), not its listing
- a process: match `ps -eo args` and exclude the matcher itself - never `comm`, never `pgrep -f`
- a file: check its **size** against a floor, never its existence

**The generalisation worth keeping:** proof that something was *requested* or *registered* is
not proof that it *happened*. Every one of these guards looked correct in code review and was
wrong in operation.

## An absent reading is not a reading of zero: four instances in one day (2026-08-31)

The sequel to "registries lie", and a worse one, because these did not fail — they returned a
number, and the number was fiction. **A field nobody fills in looks exactly like a field
measured to be zero.**

| what was read | what came back | what it actually meant | what it nearly cost |
|---|---|---|---|
| `Keypoint/Dictionary_size/` from `/rtabmap/info` | `-1` | **wrong key name.** The real one ends `/words` | reported live as "NEURAL produces zero visual words" — a headline finding, and false |
| column 8 (`quality`) of `monitor.csv` | `-1` | **nothing writes that column.** Tracking is only in the mapping log | a watcher alarmed "tracking has collapsed" twice at a robot that was tracking fine |
| `Word` table of run 7b's database | `0` rows | **the run crashed before flushing the dictionary.** Its `Feature` table holds 346,962 word observations at ~958 per frame | the same false finding, from a second direction, and it survived a week |
| `rejected_lc` in every run's monitor | `0` | still unresolved — either true, or an unwired counter | quoting "0 rejected" as if it were measured |

**The rule:** before reporting any zero, prove the field is written to at all. Find one run,
anywhere, where that field is non-zero. If none exists, the field is not a measurement and the
correct report is **"not recorded"**, never `0`.

**Print `(none)` and `0` differently.** `audit_run_db.py` does this — a missing table prints
`(none)` and says in its own output that this is not a zero. A tool that renders both as `0`
launders an absence into a finding.

**The self-inflicted one is the lesson.** Twenty minutes after diagnosing the `-1` dictionary
error, the replacement watcher was written to read tracking from the CSV column that holds
`-1`, and alarmed on it. Knowing the failure mode did not prevent repeating it inside the hour.

## RTAB-Map stores every graph link TWICE. Counting rows doubles every closure (2026-08-31)

Counting rows in the `Link` table gave run 7c **84** global closures and run 7a **18**. Both are
exactly double. Every link is written as `A→B` *and* `B→A`:

```
type 0 (neighbour)   920 rows, 920 of them mirrored -> 460 real links
type 1 (global)       84 rows,  84 of them mirrored ->  42 real closures
type 2 (proximity)   250 rows, 250 of them mirrored -> 125 real closures
```

**Count distinct UNORDERED pairs**, and verify the mirroring rather than assuming it — if
`mirrored == distinct ordered` the halving is right; if `mirrored == 0` the rows are already
honest. Gravity links (type 9) are self-links and are **not** doubled, so a blanket "divide by
two" is also wrong.

## Live counters count messages, not links (2026-08-31)

`run_monitor.py` increments once per `/rtabmap/info` **message** whose closure id is non-zero.
The same closure is re-reported across consecutive frames, and messages are also dropped. It
gave **22** for run 7a against a true **14**, and **171** for run 7c against a true **167** — so
it errs in both directions and cannot be corrected by a fixed factor.

**Quote closures only from the database's link table**, as distinct unordered pairs, after the
run has closed cleanly. Use the live counter to watch a run, never to report one.

## "The battery died" was the wrong cause of death for run 7b (2026-08-31)

Run 7b was written up as lost to a flat robot. It was not. It died of:

```
DBDriverSqlite3.cpp:4854::addStatisticsQuery() Condition (rc == SQLITE_DONE) not met!
[DB error: database is locked]  ->  terminate called after throwing 'UException'
```

Power was still on — odometry logged for another minute, and the run ended on an operator
interrupt. **Two different failures produce the same symptom from outside: the run just stops.**
Read the log for a FATAL before attributing a stop to hardware. The guard for exactly this is
already in `SOLVED.md` and had not been applied to that run.

## The rejected-closure counter is not wired to anything (2026-08-31)

Every run this project has recorded reports `rejected_lc = 0`. Run 7c's own mapping log holds
**twelve** rejections:

```
Rtabmap.cpp:3067::process() Rejected loop closure 421 -> 1003: Not enough inliers 0/20 (matches=17)
```

So the zero is not a measurement. It was quoted live during run 7c as "167 accepted, 0
rejected", and that framing actively concealed the run's real story — closures were being
proposed and thrown out for the whole final quarter of the drive.

**This is the seventh instance today of an absent value read as zero**, and the one that did
the most damage, because "none rejected" sounds like positive evidence that everything is
healthy.

**Count rejections from the LOG, not the counter:**
```bash
grep -ac "Rejected loop closure" ~/.run_records/<run>/mapping.log
grep -aoE "Rejected loop closure [0-9]+ -> [0-9]+: [^[]*" ~/.run_records/<run>/mapping.log | tail
```
The second form gives the reason as well as the count, and the reason is the useful part —
"Not enough inliers" and "Not enough matches" are different failures with different fixes.

**And the general form, now stated as strongly as it can be:** before reporting any zero, find
one run anywhere in this project where that field is non-zero. If none exists, the field is not
instrumented and the honest report is **"not recorded"**.

## A near-miss is not what "many matches, zero inliers" means (2026-08-31)

`matches=33, inliers=0/20` reads like "close, needs a nudge". It is the opposite: 33 features
matched by appearance and **not one** survived geometric checking. Appearance matching and
geometric verification are separate stages that fail independently, and a high match count with
zero inliers is a *total* geometric failure — usually because the two views are of genuinely
different surfaces, as happens when the same place is approached from the opposite direction.

Tuning the match threshold in response to this would do nothing at all. See ENGINEERING_NOTES.md section
2.13.

## Recording only the first error line, then reading a pattern into the first lines (2026-08-31)

A sweep of 15 RTAB-Map databases saved **one line per file** — SQLite's first complaint. Three
tidy "failure families" appeared in those 15 first-lines, and two of today's runs shared one:

```
Rowid out of order        lab_map_05, bench_r09n5_raytrace_off
2nd reference to page     lab_map_06, nofilter, bench_filter_r05_n3
Child page depth differs  lab_map_07a, lab_map_07c   <-- "a mode no older file shows"
```

**That last claim is false, and the disproof was already in the repo.**
`Week 7/01_run5_first_3D_map/lab_map_05_integrity.txt`, written 27 August:

```
On tree page 959208 cell 45: Rowid 650097 out of order
On tree page 2 cell 25: Child page depth differs      <-- second line
On tree page 2 cell 24: Child page depth differs
```

`lab_map_05` has the "new" mode. It simply is not that file's *first* line. **The families were a
property of the reporting, not of the files**, and a causal story about today's runs was built on
them within minutes.

**Rules:**
- **Keep every error line**, and raise the limit: `PRAGMA integrity_check(100000)`, not
  `quick_check`. The default error limit is 100 and the default output is truncated.
- **A first line is a sample of size one from that file.** Never compare first lines across files
  and call the differences a pattern.
- **Search the repo for a prior full check before theorising.** This one existed and was four
  days old.

## A read faster than the disk means you read memory, not the disk (2026-08-31)

The same sweep printed a wall-clock time per file. Dividing size by time:

```
thirteen files    76.8 - 79.6 MB/s     <- the card's ceiling, measured at 87.7 MB/s
lab_map_07c            124.7 MB/s      <- 1.4x
lab_map_07b            193.5 MB/s      <- 2.2x FASTER THAN THE HARDWARE CAN DELIVER
```

The page cache was dropped **once, at the start**. The two outliers were the two files being
worked on all evening with the audit and salvage tools, which pulled them straight back into
memory. **Their verdicts describe a cached copy, not the bytes on the card** — the exact failure
`SOLVED.md` records as invalidating a "verified" result, repeated by the person who wrote the
guard.

**And it mattered:** `lab_map_07b`'s "ok" was the single datum behind *"the run that crashed is
clean while the two that shut down cleanly are damaged"*, which was the whole causal story. With
that pass invalid, **7 of 15 is a floor, not a count.**

**Rules:**
- **Always divide size by time and compare against the medium's known speed.** It is one line of
  arithmetic and it catches a cached read every time. A per-file rate is now part of the report.
- **Drop the cache before EACH file**, not once per sweep — anything read in between is back in
  memory.
- **`quick_check` passing is weaker than it sounds.** It skips index-content and constraint
  checks that `integrity_check` performs, so "ok" means "no fault of the kind this test looks
  for", never "sound".

## "DAMAGED" is not one thing: severity spans 2 error lines to a file too broken to check (2026-08-31)

The first sweep produced a pass/fail table and every failing file was treated as equivalent. A
full `PRAGMA integrity_check(100000)` on the same 15 files, cache dropped before each, every
verdict validated by read rate:

```
lab_map_06                 ABORTS - "database disk image is malformed"
nofilter                   ABORTS
lab_map_07a                ABORTS
bench_r09n5_raytrace_off   37,445 error lines
lab_map_05                 37,357 error lines
lab_map_07c                 1,100 error lines
bench_filter_r05_n3             2 error lines
```

**Four orders of magnitude, under one word.** A file with two bad page references and a file the
checker refuses to finish reading are not the same finding and must never be tallied together.

**An abort is a RESULT, not an inconclusive.** `integrity_check` giving up means damage beyond
its error budget — that is the worst tier, not a missing measurement.

**And raw line count is not severity either.** An earlier investigation in this project traced
~1,000-2,000 error lines back to only 3-5 genuinely damaged pages; the rest was fallout from
walking a broken tree. The severity measure is the page-to-object mapping, not the tally.

## A repeated count across unrelated files is a fingerprint, not a coincidence (2026-08-31)

```
lab_map_05                 6.83 GB, 26 Aug   398 "2nd reference to page"   2 child-depth   2 index
bench_r09n5_raytrace_off   1.71 GB, 30 Aug   398 "2nd reference to page"   2 child-depth   2 index
lab_map_07c                6.09 GB, 31 Aug   398 "never used"
```

Two files four days and 5 GB apart with damage profiles matching to within 0.2 %, and a third
reporting the same 398 in a different category. **Whatever is damaging these files leaves a
fixed-size footprint.** Not yet explained, and deliberately not guessed - two explanations were
already offered and withdrawn the same evening.

**The check that would name it:** map the damaged page numbers to the tables that own them
(`SELECT name, path, pageno FROM dbstat('main', 1)`), rather than reasoning about page numbers
in the abstract. Runnable at a desk, no robot.

---

## A monitoring script's own bugs look exactly like discoveries

**2026-09-01, run 8.** Five wrong claims were produced in one evening. **None came from the
robot, the camera, or the database.** Every one came from the code watching them, and four
were caught only because a number failed a sanity check that should have been applied first.

| what was claimed | what was true | the defect |
|---|---|---|
| `FATAL-IN-LOG` | nothing wrong | grepped bare `Segmentation`; matched the **parameter name** `Grid/NormalsSegmentation` in the ordinary startup listing |
| "135 total-failure rejections — the reversed-lap signature" | the real loop-closure count was **0** | grepped bare `Not enough inliers 0/20`, which RTAB-Map also prints for **odometry** failures (`OdometryF2M.cpp:626`) |
| "241 odometry resets, concerning" | run 7c, which produced a good map, had **2,092** | alarmed before establishing the known-good baseline |
| "SQLite did not finish closing" | a clean close | `JournalMode=TRUNCATE` truncates the journal to **zero bytes** instead of deleting it; a 0-byte journal is the *success* state |
| "no ROS master running" | the master was up | a non-interactive `ssh` does not source ROS, so `rostopic` was **not found** and its failure read as absence |

### The check that caught the worst one, and should be applied first every time

**Arithmetic.** The report said `rejected=17` and `near-miss=37, total-fail=135`. A subset
cannot be ten times the size of its own set. That inconsistency was visible in the output
before any investigation, and it is what exposed the odometry/closure conflation.

**Before reporting any derived count, check it against its own total.** Parts must sum to no
more than the whole. This costs one line and would have caught the error at the source.

### Rules

1. **Anchor every grep to the full line shape you mean**, never a bare word. `Segmentation`
   matches a parameter name; `\[FATAL\]` and `Segmentation fault` do not. `Not enough inliers`
   appears in two unrelated subsystems; `Rejected loop closure [0-9]+ -> [0-9]+: Not enough
   inliers` appears in one.
2. **Never raise an alarm on a metric until the same metric has been read off a known-good
   run.** The reset count looked alarming at 241 and was a third of normal.
3. **A watcher that cries wolf is worse than no watcher**, because its later true alarms are
   discounted. Fix the pattern, do not raise the threshold.
4. **Check what a mode actually does before testing for its absence.** The clean-close test was
   written to prove the durability fix worked and would have reported failure every single time
   the fix was working correctly.
5. **Do not put a fast-moving value in a change-detection key.** Keying on the closure count
   would have emitted one notification per closure — 167 on run 7c — and got the watcher
   auto-stopped for noise at exactly the moment it was needed.

---

## `audit_run_db.py` counted link rows, and would have doubled run 8's headline number

**2026-09-01.** That RTAB-Map stores every link twice was already documented here after it
nearly put "84 global, 250 proximity" into a report. **The trap was written down and the tool
still had it** — `SELECT type, COUNT(*) FROM Link GROUP BY type`, counting rows.

Caught before run 8 was audited, but only because the tool was re-read while waiting. It would
have reported **334** loop closures instead of **265**.

**A documented trap is not a fixed trap.** When a counting trap is recorded here, grep the
tools for it the same day:

```bash
grep -rn "FROM Link" catkin_ws/src/*/scripts/ tools/
```

**The fix demonstrates rather than assumes.** It prints rows, distinct unordered pairs, and
their ratio per link type, so the doubling is shown per run — which also revealed that gravity
links are **self-links at ratio 1.00**, so a blanket "divide by two" would have been wrong too.
If any closure type ever returns a ratio other than 2.00, the tool now refuses to let the total
be quoted without explanation.

```
type 0 neighbour   rows=1,230  distinct=615  ratio 2.00
type 1 global      rows=   98  distinct= 49  ratio 2.00
type 2 proximity   rows=  432  distinct=216  ratio 2.00
type 9 gravity     rows=  947  distinct=947  ratio 1.00   <- NOT doubled
```

Validated against run 7c, where it reproduces the known 167 exactly.

---

## Whichever terminal starts the ROS master owns it, and everything dies when it closes

**2026-09-01, run 9b.** No standalone `roscore` was running, so the first `roslaunch` —
the camera, terminal 1 — quietly started one. Its log says it plainly:

```
auto-starting new master
ROS_MASTER_URI=http://127.0.0.1:11311
```

Terminal 2's log has no such line; it merely connected. **So terminal 1 owned the master**,
and when it ended the master went with it. The consequences cascaded and none of them
announced themselves as "the master died":

| what was seen | what it actually was |
|---|---|
| the map stopped growing | camera unregistered, mapper receiving nothing |
| `Unable to communicate with master` on `publish_map` | master gone |
| `ConnectionRefusedError` from `save_map_artifacts.py` | master gone — **no grid, no cloud saved** |
| database showed `words=0` | `rtabmap` orphaned, dictionary never flushed |
| a 4.856 m "closed-loop error" | **the map ended 176 s and 11.02 m before the drive did** |

**That last row is the dangerous one.** The error figure looked like a real measurement and
was reported before being checked. It is not a closed loop at all — it is the distance
between where mapping started and where mapping *stopped*, with eleven unmapped metres after
it. It was withdrawn the same session.

### Rules

1. **Start `roscore` in its own terminal first, every run.** Nobody closes it until the run
   is completely finished. Verify with `ps -eo comm= | grep -cx roscore` **before** launching
   anything else — and confirm you started it, not a `roslaunch`.
2. **Grep every camera/mapping log for `auto-starting new master`.** Its presence means that
   terminal owns the master and the run has a single point of failure.
3. **Before quoting a closed-loop error, prove the map covers the whole drive.** Compare the
   last node's timestamp against the last wheel-odometry timestamp. If the wheels kept
   reporting after the map stopped, the figure is not an error measurement.
4. **An orphaned `rtabmap` still holds the dictionary.** Send SIGINT by PID and wait — it
   flushes and closes. `SIGKILL` loses it permanently. Run 9b went from `words=0` to
   `words=279,799` purely by being interrupted rather than killed.

## Long recorders in interactive ssh terminals (2026-09-04)
**Tried:** the 4 m sphere recorder ran in the user's interactive ssh terminal, per the
usual "user runs long-running commands" division of labour.
**How it failed:** the PC-Jetson link blipped; every ssh terminal died at once. The
recorder *appeared* to survive (checked alive minutes after the drop), then the
Jetson's ssh service finished its disconnect timeout and killed the whole session -
recorder included, 70 samples into 200. The delayed kill is the trap: an
immediately-after check reads ALIVE and reassures wrongly.
**Why:** sshd keeps a broken session's processes alive until its keepalive timeout,
then sends the hangup signal. A watcher sees death only minutes after the link drop.
**Instead:** any recorder or job longer than ~10 minutes on the Jetson starts
DETACHED (`setsid ... < /dev/null &`, output to a log file), launched by the Jetson operator over
ssh, with an armed watcher. Interactive terminals are for short commands and viewers
only. The web monitor (also detached) is the user's window into a run - it survived
this exact drop while every terminal died.

## The pgrep -f self-match trap, /proc edition (2026-09-04 - FOURTH hit)
**Tried:** killing a process by scanning /proc/*/cmdline for its script name -
believing this avoided the banned `pkill -f`.
**How it failed:** identically to pkill -f. The scanning shell's own cmdline (the
whole `bash -c '...'` string sent over ssh) contains the script name being grepped,
so the loop killed its own shell mid-scan. One attempt killed the real process then
itself before restarting anything; the next died before producing output at all.
**Why:** any match-by-command-line method self-matches when the matcher's command
line quotes the target. pgrep -f, grep over /proc cmdlines, ps aux | grep - same trap.
**Instead:** match the EXECUTABLE prefix, not the bare name: require the cmdline to
start with `python3 ` before the script name (`case "$c" in python3*script.py*)`).
A `bash -c` wrapper never starts with python3. Better still, record the PID at
launch and kill that.

## Run logs written to /tmp are evidence, and /tmp does not survive a reboot (2026-09-05)
**Tried:** letting the sphere-benchmark recorder write its log to `/tmp/recorder_<id>.log`
on the Jetson, and reading the acceptance rate out of it during the session.
**How it failed:** the Jetson rebooted overnight, `/tmp` was cleared, and the 4 m and
5 m logs went with it. The measurement CSVs were safe - they were copied to three
devices the same session - but the CSV holds only the NUMERATOR of the acceptance
rate (samples accepted). The denominator, frames processed, exists nowhere else.
Both stations' acceptance rates are now permanently unrecoverable.
**Why it matters more than it looks:** acceptance rate had by then been promoted to
a first-class result of this project - it is the metric on which the camera differs
most sharply from the LiDARs, which accept 1000 of 1000. Two of four stations lost
theirs. And the figures had already been quoted in working notes, which is exactly
the situation ENGINEERING_NOTES.md 2.13 forbids: a number that survives only as prose cannot be
recomputed and must not be reported. They were struck from the results document
rather than quoted.
**Instead:** a station is not closed until its recorder log and viewer log are copied
into the data folder alongside the CSV, and travel with it to every copy of the data. Anything that is evidence lives where the evidence lives. If a value is
computed from a log, ALSO write the value into the run's provenance file, so the
conclusion survives even if the log does not.

## The three storage devices differ in write speed by 280x (measured 2026-09-05)
**Tried:** switching raw camera captures to a genuinely lossless format and writing
them, as all previous captures were written, to the USB stick.
**How it would have failed:** a lossless capture needs roughly 15-25 MB/s sustained.
Measured with `dd ... conv=fsync`, 200 MB each:

    Jetson internal disk   153 MB/s     14 GB free
    USB stick (exFAT)      3.7 MB/s    230 GB free
    microSD (ext4)       0.544 MB/s     69 GB free

The USB is 4-7x too slow for a lossless capture; the camera would have stalled or
silently dropped most frames, and the file would have looked plausible until someone
tried to use it. Caught by testing before the first capture, not after three.
**The microSD number is the alarming one.** 544 kB/s is not a working card. It is the
same card that destroyed four files in one day in August and then lost a freshly
created, synced folder tree across a clean power cycle. ENGINEERING_NOTES.md rule 7 still sends
BULK experimental data (rosbags) there: a mapping bag writes at several MB/s, so that
rule is now actively dangerous and needs revisiting before the next dynamic run.
**Instead:** write anything high-rate to the INTERNAL disk first, then copy to the USB
afterwards - a copy that is merely slow is fine, a capture that cannot keep up is not.
Watch the internal disk's 14 GB: a full internal disk is how lab_map_02 was destroyed.
Measure a device before trusting it with a rate; free space says nothing about speed.

---

## Never leave two orchestration scripts alive — the second one silently reconfigures the hardware under the first

**Date:** 2026-09-09 · **Machine:** Jetson internal disk · **Cost:** one void experiment,
and it was the SECOND time

### What was tried

`stab_test.sh` was started to measure whether depth stabilization explains a
collapse in the detector's success rate. It stopped the running recorder,
restarted the camera with `depth_stabilization:=1`, and began recording.

### How it failed

**It produced zero samples, and the setting it was testing read back as 0.**

The restart itself worked perfectly — the node's own log says so:

```
 * Self calibration    -> DISABLED
 * Depth mode          -> NEURAL [4]
 * Depth Stabilization -> 1
```

Thirty seconds later `live_all_modes.sh`, which was **still running**, reached the
end of its NEURAL block, moved on to NEURAL_PLUS, and restarted the camera with
`depth_stabilization:=0`. The test then read a camera it had not configured, in a
depth mode it had not asked for, and recorded nothing.

The `ps` output that shows it:

```
181421 roslaunch sidewalk_perception zedx_front.launch depth_mode:=NEURAL_PLUS \
       auto_exposure:=true self_calib:=false depth_stabilization:=0
```

### Why

Two reasons, and both are worth keeping:

1. **`kill -INT` does not reliably stop these scripts.** They spend nearly all
   their time inside `sleep`, and a bash script in `sleep` does not act on SIGINT
   promptly. The test *did* send SIGINT to `live_all_modes.sh` and it survived.
2. **Nothing checked.** The test assumed that having sent the signal, the other
   script was gone. It never verified.

**This had happened before** — two close-out chains alive at once, fighting over
the camera — and it was never written down. That is the whole reason it happened
again.

### What to do instead

**Every script that reconfigures the camera must first kill any other
orchestrator, with TERM then KILL, and only then proceed:**

```bash
for PAT in live_all_modes.sh run_pinned_200.sh station_camera.sh finish_4m.sh; do
  for P in $(ps -eo pid,args | grep -F "$PAT" | grep -v grep | awk '{print $1}'); do
    kill -TERM $P 2>/dev/null; sleep 2; kill -KILL $P 2>/dev/null
  done
done
sleep 3
```

**`kill -9` is correct here and is not a violation of the never-kill-9 rule.**
That rule is about the **ZED camera node**, which locks the camera until the
Jetson is rebooted if it is killed hard. These are bash orchestration scripts;
killing them hard costs nothing.

**And verify the setting actually took, from the node, before trusting the
result.** `stab_test.sh` did do this and printed `WARNING: stabilization did not
take — the test below is void`, which is the only reason the void run was caught
instead of being published. Every experiment that changes a setting must read it
back from the node and say out loud when it disagrees.

---

## Every safety check written on 2026-09-09 failed OPEN — they could not report a problem

**Date:** 2026-09-09 · **Machine:** Jetson internal disk · **Found by:** an independent
audit that was asked for, not by the checks themselves

### What was tried

Three guards were built to protect a 16 GB recording campaign whose entire value
depends on the files being bit-exact:

1. a pre-flight that recorded 5 s and was supposed to **abort** unless the file
   was genuinely lossless
2. a per-burst warning that was supposed to fire if any recording was not
   lossless
3. a web page tile that displayed each recording's state

### How they failed

**All three reported success unconditionally. None of them could ever have said
no.**

The pre-flight and the per-burst check called a shell function that ran Python
and read its output. **The ZED SDK writes its INFO banner to STDOUT**, so the
function returned banner text where numbers were expected. The log records it
verbatim:

```
03:11:44   test: [2026-09-09 frames, 03:11:43 UTC][ZED][INFO] Logging level INFO KB per frame
03:11:44   PASS - no fallback, and 03:11:43 UTC][ZED][INFO] ... is consistent with true lossless.
```

`[ "${TK%.*}" -lt 2000 ]` then errored on a non-numeric string, the `if`
evaluated false, and the abort was skipped. **An H265 file would have passed.**

The web page was worse, because it was honest-looking:

```python
    recs.append({'name': f, 'gb': round(b / 1e9, 2)})
for r in recs:
    r['ok'] = True          # <- stamped on every file, nothing opened
```

A comment directly above promised a losslessness check. The code never opened a
single recording. A zero-byte, truncated or lossy file rendered green.

**The recordings happened to be genuinely lossless** — verified independently at
2627–2648 KB/frame — so nothing was lost. That is luck, not process.

### Why

**A check that has never been observed to fail has not been tested.** All three
were written, watched to print PASS on a good file, and trusted. None was ever
shown a bad file. Two of them additionally trusted stdout from a library that
prints to stdout.

### What to do instead

1. **Prove a check can fail before trusting it.** Feed it a deliberately bad
   input — a truncated file, a lossy recording, a zero-byte file — and watch it
   refuse. If that has not been done, the check is decoration.
2. **Never parse the stdout of anything that also logs to stdout.** Print one
   line with a unique prefix and grep for it. `~/kbpf.py` does this:
   it emits exactly one `RESULT <frames> <kb> <gb>` line, and callers grep
   `^RESULT`. Its docstring names this incident.
3. **A numeric test on a possibly-non-numeric string falls through in bash.**
   `[ "$x" -lt 2000 ]` with `$x` non-numeric prints an error and evaluates
   false — the same as passing. Check for emptiness and non-numerics explicitly,
   and treat "could not measure" as a distinct state from "good".
4. **Report three states, never two:** good / bad / not measured. Collapsing the
   third into the first is how all of this happened.

### The command that verifies it

```bash
# a check that cannot fail is not a check. Show it a bad file:
head -c 1000000 good.svo2 > /tmp/truncated.svo2
python3 ~/kbpf.py /tmp/truncated.svo2 | grep '^RESULT'   # must NOT look lossless
```

**Related:** the same audit found three stale `burst_after_neural.sh` processes
armed to hijack the camera, missed because the competing-orchestrator guard
enumerated script names rather than asking what else holds the camera. See the
entry above on two orchestration scripts.

---

## `cmp` after a copy verifies the page cache, not the device

**Date:** 2026-09-09 · **Machine:** Jetson internal disk → USB stick · **Cost:** four
files declared "verified byte-for-byte" while most of their contents had never
left RAM

### What was tried

The station script copies each recording to the USB stick and then runs
`cmp -s <source> <destination>`, logging *"copied and verified byte-for-byte"* on
success. It was written specifically so that a copy could not be believed without
proof.

### How it failed

**It passed in 10–14 seconds per file, on files of 0.7–3.0 GB, over a link
measured at 4.7 MB/s.** Three gigabytes cannot cross that link in ten seconds.

At the moment the third file was declared verified, `/proc/meminfo` read:

```
Dirty:           4046816 kB      <- 4.05 GB, and RISING
Writeback:           108 kB
```

Linux accepts writes into the page cache and reports the file at full size
immediately — `ls`, `du` and `stat` all believe it. `cmp` then read **both** files
back out of that same cache. It compared RAM against RAM. A power cut at that
moment would have destroyed data the log said was verified.

Device throughput, read from `/proc/diskstats` rather than inferred, was
**4.74 MB/s** — so the copy needed roughly forty minutes, not the seconds the log
implied.

### Why

`cmp` reads through the normal filesystem path, which is served from cache when
the pages are resident. Nothing about it reaches the storage device. The check
was testing that the kernel could remember what it had just been told.

### What to do instead

Use `verify_stick.py` (in `sidewalk_evaluation/scripts/`), which:

1. `fsync`s the destination so the kernel must write it out,
2. drops that file's cached pages with **`posix_fadvise(fd, 0, 0, POSIX_FADV_DONTNEED)`**
   — this needs no privileges, unlike `/proc/sys/vm/drop_caches`, which needs root,
3. reads it back, which now comes off the device,
4. compares SHA-256 against the source,
5. and reports how much is still unwritten, saying **NOT SAFE YET** if it is not
   near zero.

**And check the rate, not just the result.** A verification that completes faster
than physics allows is the tell:

```bash
# what is actually reaching the stick, as opposed to being promised
R1=$(awk '/sdb /{print $10}' /proc/diskstats); sleep 15
R2=$(awk '/sdb /{print $10}' /proc/diskstats)
python3 -c "print('%.2f MB/s' % (($R2-$R1)*512/15/1e6))"
awk '/^Dirty/{printf \"%.2f GB still in RAM\n\", $2/1e6}' /proc/meminfo
```

### The pattern this belongs to

**This was the FOURTH check in one night that reported success without testing
what it claimed** — after the pre-flight lossless gate, the per-burst lossless
warning, and the web page's green tick. All four were written as safeguards. All
four were structurally incapable of failing.

Worse: this one was written *after* the entry above titled "Every safety check
written on 2026-09-09 failed OPEN". Knowing the rule is not the same as applying
it. **Before trusting any verification, ask what it would take for it to say no,
and then make it say no.**

### "Verified on the stick" was treated as "safe to delete the original" (2026-09-09)
- **What was tried:** after all 14 of the 4 m station's recordings were proven on
  the 250 GB USB stick by SHA-256 read back off the device with the page cache
  dropped, the Jetson operator advised that the Jetson-internal copies were "duplicates" and
  printed the `rm` for the user to run. The user ran it. Hours later the Jetson
  rebooted and the stick came back with **no partition table and ReadOnly true**
  — sole copy, unreachable, on the very medium this file's 2026-08-25 entry
  blames for five previous losses.
- **How it failed:** nothing was corrupted by the deletion itself. The failure
  was leaving the ONLY copy on exFAT-via-FUSE when a second, independent,
  journaled medium (the ext4 microSD, 69 GB free) was plugged in and idle, and
  a second computer was a third option. Verification answers "is the copy faithful?" —
  it says nothing about "will the medium still enumerate tomorrow?"
- **Why it happened:** the subproject rule "verify after moving, then let the
  user delete the original" was read as licence, and the same project's own
  exFAT history — five losses, documented, in this file — was not consulted at
  the moment of the recommendation.
- **What to do instead:** an irreplaceable set exists on **two independent
  media** before any copy is deleted, and **exFAT never counts as one of the
  two**. Concretely for this study: internal ext4 + a second computer, or internal + ext4
  card; the stick is transport, not storage. The deletion advice must name how
  many independent copies will exist AFTER the delete, and if the answer is
  one, the advice is wrong regardless of any checksum.

### Judging "is my data still there?" by `strings` alone (2026-09-09)
- **What was tried:** a data-presence interlock that ran `strings` over the
  first 16 MiB of a failed USB stick and aborted if it found anything.
- **How it failed:** it fired twice on false positives. First on
  `MPTools_20241226_A5S` - the controller's own mass-production firmware, not
  user data. Then on five 8-character runs of random bytes that happened to be
  printable. Both times it announced "REAL DATA FOUND" on a device that has
  no user data at all.
- **Why:** roughly 37 % of byte values are printable ASCII, so an 8-character
  printable run occurs by chance about once per 2,800 bytes. Any large blob of
  random or firmware data generates them in quantity - 19,661 of them here.
- **What to do instead:** three tests together, never `strings` alone.
  (1) match user-file magic numbers, (2) require a run of 4+ alphabetic
  characters before treating a string as text, (3) measure how many *distinct*
  block contents the device returns. That third one is decisive: real files
  gave 0.0000 duplicate fraction in test, this stick gave 0.9922.
  Implemented in `tools/usb_find_files.py`.

## A big buffered write to the slow SD card can livelock and reboot the Jetson (2026-09-10)

**What was tried:** measuring the microSD's write speed with `dd`, to answer
whether it could hold a recording. Several `dd` runs were launched over `ssh`
with a foreground timeout on the LOCAL side.

**How it failed, in two compounding ways:**
1. **A timed-out `ssh` kills the local client, NOT the remote `dd`.** Each
   timeout left its `dd` running on the Jetson. Three ended up writing the same
   card at once — a self-inflicted version of the "start it and hope" trap, and
   `ps -eo comm | grep '^dd$'` showed all three, one at 7+ minutes.
2. **The concurrent writes built ~2.3 GB of dirty pages** aimed at a card that
   drains far slower than they arrive. The dirty writeback starved the system,
   SSH stopped answering, and the Jetson **rebooted** (came back `up 0 min`).
   This is the same memory-livelock class as the office-DB incident.

**Why it recovered cleanly anyway:** nothing being written mattered (a scratch
file, already unlinked), and no station was recording. The camera, its
services and every dataset live elsewhere. It cost time, not data.

**What to do instead:**
- **Never launch a disk benchmark — or any `dd` — through a foreground-timed
  `ssh`.** Run it with `run_in_background: true` and wait for the completion
  notification, or as a script on the target that logs its own result.
- **One writer to a card at a time.** Before starting another, confirm the
  previous is gone with `ps -eo comm | grep '^dd$'` (never `pgrep -f`).
- **Watch `/proc/meminfo` Dirty during any large write to removable flash.**
  Above ~1 GB of dirty aimed at an SD card is the danger zone on this Jetson.
- **To measure card speed safely:** one modest file (≤512 MB), `oflag=direct`
  avoided (pathologically slow on SD — use `conv=fdatasync`), backgrounded,
  and nothing else touching the card.

## Editing a bash script while it is running kills it (2026-09-10)

**What was tried:** patching `record_offline.sh` to fix its measurement function
*while that very script was mid-run*, recording the 4 m block.

**How it failed:** bash does not read a script into memory. It reads
incrementally and tracks its position **by byte offset**. Changing the file
underneath a running instance makes it resume at whatever now sits at that
offset — usually mid-token — and the script dies. It died silently right after
the block, and the 8 bursts were never recorded.

**Nothing was corrupted** — the block was complete and valid, 4,428 frames. A
step was skipped, not damaged. But an hour of station time was lost.

**What to do instead:**
- **Never edit a script that is executing.** Wait for it to finish, or
  `cp` it to a new name and edit the copy.
- The urge to fix a defect the moment you spot it is exactly the trap here. A
  broken measurement in a running script is a *reporting* problem; killing the
  run turns it into a *data* problem.
- Same family as "never add load to a live station" — extend that rule to
  cover the script driving it, not just CPU and I/O.

## The microSD failed too — 61.7 MB/s to 0.72 MB/s (2026-09-10)

**Symptom:** copying the 11.93 GB block to `SIDEWALK128` crawled. Measured over
a clean 60 s window: **0.72 MB/s**, decelerating (2.7 MB/s averaged over the
whole attempt). It had measured **61.7 MB/s** on ext4 on 2026-08-25. rsync gave
up on its own with `[sender] io timeout after 60 seconds`.

**Why this reads as failing hardware, not a fluke:** the deceleration pattern
is what flash does when its fast write cache is exhausted and it falls back to
native speed, or when it is heavily remapping bad blocks. 3.9 GB of dirty pages
piled up in RAM because the device could not drain them.

**This is the SECOND storage device lost in two days** — the 256 GB exFAT stick
died 2026-09-09. Both were removable flash. Treat every remaining removable
device in this project as suspect until re-measured.

**What replaced it:** copying off over a wired network link (SOLVED.md, "Copying a big recording off
the Jetson"), against 4.68 MB/s over WiFi and 0.72 on the card.

## A 390 s block is not a fixed size — it scales with what else is running

Yesterday's 4 m block: **1,136 frames, 3.01 GB**. Tonight's, same 390 s and same
settings: **4,428 frames, 11.93 GB** — nearly 4x.

The difference is that yesterday the live detector was competing for CPU and the
camera managed ~2.9 effective frames/s; tonight nothing else ran and it recorded
at ~11.4. **Planning disk from a previous night's file sizes is planning from a
number that silently depends on system load.** Budget from
`seconds x measured frame rate x KB/frame`, and re-measure the rate on the night.


---

## Driving a long Jetson job from another computer — it dies with the session (2026-09-20)

- **What was tried:** the Scenario 2 sweeps run as a shell script **on another computer**
  that reaches the Jetson with one `ssh` per depth mode. Overnight work was launched
  that way, backgrounded, with `nohup`, and a chain script behind it to run the next
  job when each finished.
- **How it failed:** that computer's session ended at 06:37 and took **everything** with
  it — the chain, the sweep driver, its `ssh`, the replay running on the Jetson at
  that moment, and a 9 GB transfer that was 95 % done. Eight hours later the Jetson
  was idle, the 8 m wide-box sweep was stuck at three modes of five, and the
  transfer sat 502 MB short. Nothing had failed; nothing had been running.
- **Why `nohup` did not save it:** `nohup` only ignores SIGHUP. Session teardown
  takes down the whole process group, and the remote work was a child of a local
  `ssh` that died with it. A job is only as durable as the process holding its pipe.
- **What to do instead:** **long work runs on the machine that does the work, started
  with `setsid`, owned by init.** `~/s2_run_modes.sh` on the Jetson takes
  the box as environment variables and runs the replays itself; your computer launches it
  and then only polls. A computer that goes away now costs nothing. Keep remote
  drivers for interactive runs where somebody is watching.
- **And check that a resumed job can be resumed.** `rsync --partial --inplace` picked
  the transfer up exactly where it stopped and the SHA-256 matched afterwards, so the
  502 MB was the only loss there. Always re-verify the hash after a resume; a resumed
  file that is never re-hashed is a file nobody has checked.

## `set -u` plus `source /opt/ros/noetic/setup.bash` kills the script, silently (2026-09-20)

- **What was tried:** the new Jetson-side runner opened with `set -u` (good practice —
  an unset variable should be fatal, not empty) and then did
  `source /opt/ros/noetic/setup.bash 2>/dev/null` before each replay.
- **How it failed:** the runner logged `--- QUALITY ---` and vanished. No error, no
  traceback, no output at all. The replay never started.
- **Why:** ROS's setup script reads variables that do not exist yet — `PYTHONPATH`,
  `CMAKE_PREFIX_PATH` and others. Under `set -u` an unbound variable is fatal to the
  **current** shell, and `source` does not fork, so it killed the runner. The
  `2>/dev/null` then swallowed the one line that would have explained it. The
  remote drivers run the same line inside an `ssh` command string, where `set -u`
  is not in force, which is why this had never shown up before.
- **What to do instead:** wrap it — `set +u; source /opt/ros/noetic/setup.bash; set -u`.
  And do not put `2>/dev/null` on something whose failure you have not already seen;
  it hides exactly the message you will need.

## A progress poller that outlives its job reports a corpse as healthy (2026-09-20)

- **What was tried:** the runner forked a subshell to rewrite the `.progress` line
  every 20 s from the workers' own counters, and killed it after each mode.
- **How it failed:** when the runner died at the `set -u` bug above, the poller was
  orphaned to init and **kept writing progress with a rising elapsed time**. The
  dashboard showed a job advancing that did not exist. Rule 13 says liveness is judged
  by the file — so a file that lies is worse than no file.
- **What to do instead:** the poller now holds its parent's pid and runs
  `while kill -0 "$MAIN"`, and writes `!! the runner died - this job is NOT running`
  when it stops. A reporter must never be able to outlive the thing it reports on.

## `ps -eo comm` cannot see a python script, and truncates at 15 characters (2026-09-20)

- **What was tried:** checking for the replay with
  `ps -eo comm | awk '$2=="plane_replay_n.py"'`, and for the runner with
  `$3=="s2_run_modes.s"`.
- **How it failed:** both returned nothing while both were plainly running.
- **Why, two separate reasons:** `comm` is the **executable's** name, so a script run
  as `python3 /path/plane_replay_n.py` shows up as `python3` and the script name
  appears nowhere. And `comm` is capped at **15 characters**, so `plane_replay_n.py`
  (17) is stored as `plane_replay_n.` — an exact comparison against the full name can
  never match. `s2_run_modes.sh` is exactly 15 and does fit; the typo'd 14-character
  pattern did not.
- **What to do instead:** use `tools/jetson_running.sh`, which scans
  `/proc/<pid>/cmdline` and compares each argument **whole, by base name** — that is
  what it was written for. It is not just about avoiding `pgrep -f`'s substring bug;
  `ps -eo comm` cannot answer this question at all. And judge a job's liveness by its
  `.progress` file first (rule 13); a process check is the fallback, not the primary.

---

## A search box wide enough to "just let the detector find it" finds NOTHING (2026-09-20)

**What was tried.** Scenario 3 needs to know where the three-sphere frame stands before a
moving box can follow it. The obvious move was to hold the box still and make it cover
everything ahead - 2 to 16.5 m deep, the full width of the walkway - and let Nicolas's
detector pick the frame out, since its three-sphere triangle test is a strong gate that a
wall cannot pass.

**How it failed.** It reported nothing at all. Not a bad fit, not a wrong object - zero
detections on every cloud of a 45 s drive, twice, while the frame was plainly in view.
That is the dangerous kind of failure: an empty result reads as "the sensor cannot see it"
and would have been written up as one.

**Why.** The detector picks its candidate points at random. With the floor and the walls
inside the box there were 4,138 points of which roughly 200 sat on spheres, so a draw
lands on a sphere about one time in twenty - and it must land on FOUR of them at once, in
the same neighbourhood, to propose a sphere at all. 1,500 draws never managed it. Proved
directly: on the SAME cloud with the SAME detector and the same seed, narrowing the box to
626 points around the frame found it immediately, sides 1.087 / 1.243 / 1.232 m against
the expected 1.10 / 1.23 / 1.23.

**What to do instead.** Slide a shallow box outward and stop at the first slice that
passes the triangle test. `02_zedx_pipeline/s3_find_target.py` (laser) and
`s3_cam_locate.py` (camera) both do this. The gate is unchanged; only the haystack shrinks.

**The general lesson, which is bigger than this box.** A random-sampling fitter's
success depends on the RATIO of wanted points to unwanted ones, not on whether the wanted
ones are present. Widening a search region to "be safe" makes it strictly worse, and it
fails silently.

---

## `${VAR:?message}` parses quotes inside the message (2026-09-20)

**What was tried.** A required-variable check with a helpful message:
`CLOCK="${CLOCK:?set CLOCK, the pass's 'clock offset' from its provenance}"`

**How it failed.** `bash -n` reported `unexpected EOF while looking for matching '` at a
line 100 lines further down, inside an unrelated heredoc. The real fault was 100 lines
above it.

**Why.** The message in `${parameter:?word}` is not text - bash applies quote removal to
it, so an apostrophe opens a single-quoted section that swallows the rest of the file.

**What to do instead.** No apostrophes in a `:?` message. The error will not point at the
line that caused it.

---

## A comment between a backslash and its command runs the command without its environment (2026-09-20)

**What was tried.** An environment prefix on one line, an explanation, then the command:

```bash
FOO=1 BAR=2 \
# why the arguments below look odd
python3 thing.py
```

**How it failed.** `thing.py` ran, produced correct output, and silently never saw `FOO`
or `BAR`. In this case that meant a motion-correction node ran and did its job while the
log of what it corrected — the number the experiment existed to produce — was never
written. Nothing errored. `bash -n` is happy. The node's own log file was empty and that
looked like a node that had not started yet.

**Why.** A backslash continues onto **whatever comes next**, and a comment line is not
nothing — it terminates the command. So the first line becomes a bare assignment, which
sets the variables in the *current* shell without exporting them, and the `python3` line
becomes a separate command that inherits nothing.

**What to do instead.** Put the explanation **above** the whole construct, never inside
it. If a line ends in `\`, the next line is part of the command.

**How to catch it.** Read `/proc/<pid>/environ` of the running child and look for the
variable. A missing output file is ambiguous; a missing variable is not.

---

## Pulling results off the Jetson with plain `rsync -a` OVERWRITES newer analysis on your computer with older Jetson copies

**2026-09-23, caught within the hour, nothing lost — but only because git had the good version.**

**What was tried.** Rescuing result files from the Jetson before freeing its disk, with
`rsync -as --exclude=<recordings>` straight into the depth study's `03_data/` tree, on the reasonable
assumption that the Jetson held files the other computer lacked.

**How it failed.** It did hold 273 files the other computer lacked — and it also held **34 files the
other computer already had in a BETTER version**. `rsync -a` decides by size and modification time, not
by which is more finished, so all 34 were replaced with the Jetson's copies.

The damage was invisible in a size or row-count check: **588 rows before, 588 rows after**, same
measurements, same timestamps. What changed was the **header**:

```
committed (other):   ...,passed_gate,why,passed_gate_fa1pct,gate_cells_fa1pct    14 columns
pulled   (Jetson):   ...,passed_gate,why                                          12 columns
```

The two analysis columns were added on the other computer **after** the Jetson wrote its copy. The
Jetson's file was newer by clock and different by size, so it won — while being **older by
content**.

*Plain terms: the machine that wrote a file last is not necessarily the machine that has the
best version of it. A later timestamp can mean "written again earlier in the workflow", not
"improved".*

**Why it was caught.** `git status` showed 34 files as *modified* when a pure rescue should only
ever *add*. **A rescue that modifies an existing file is a contradiction in terms** — that is
the signal to stop and look.

**What to do instead.**
- **Pull into a staging folder**, never straight over an existing tree, and move files in
  deliberately. This is what the SLAM half did (`06_rescued_from_jetson/`) and it had no such
  problem.
- If pulling in place, pass **`-u` / `--update`**, which refuses to overwrite a file that is
  newer on the receiving side — and even that would not have helped here, so:
- **Always `git status` immediately after a rescue pull.** Any `M` line means something was
  overwritten. Diff the header, not just the row count.
- Restore with `git checkout HEAD -- <file>`, which is what recovered all 34 here.

---

## `pkill -f` killed the shell that ran it — the fifth time substring matching has bitten this project

**2026-09-23.** Stopping a local web server started a few minutes earlier:

```bash
pkill -f "http.server 8777" 2>/dev/null; git add ... && git commit ...
```

**How it failed.** `pkill -f` matches against a process's **whole command line** — including
the command line of the shell that is running `pkill` itself, which contains the string
`http.server 8777` because it is right there in the command. The shell killed itself. Exit code
144, and the `git commit` chained after it never ran.

*Plain terms: it asked "kill anything whose name contains this phrase", and the question itself
contained the phrase.*

**ENGINEERING_NOTES.md rule 8 already warns about exactly this** and had recorded four previous hits. This
is the fifth, and the first where the victim was the shell rather than a false "still running"
reading. **Knowing the rule is not the same as applying it** — it was written for process
*checks* and got skipped for a process *kill*, which is the same defect.

**What to do instead.**
- **Do not use `-f` matching at all** for these. Find the listener and kill it by number:

      ss -ltnp | grep ':8777'          # names the pid
      kill <pid>

- Or keep the pid when the job is started (`$!`) and kill that.
- Or, for a throwaway server, let it be — a local HTTP server on a loopback port costs nothing,
  and leaving it is safer than a careless kill.
- **The test that would have caught it:** ask whether the command's own text contains the
  pattern it is searching for. If it does, the command matches itself.

---

## The jobs page can never judge a job whose progress line has no `N/M` counter

**2026-09-23, found while making mapping runs visible (ENGINEERING_NOTES.md rule 13).**

**What was tried.** A `.progress` line in the documented free-text form — no counter, because a
mapping drive has no known total; it ends when the driver stops. The rule says a line without
`N/M` is simply shown as-is, so this should have been fine.

**How it failed.** The file was discovered and served correctly, and the line rendered — but the
job sat in state **`starting` for ever** and could never become `running` or `stalled`.

**Why**, from `jobs_dashboard.py`:

```python
if moved:
    gap = now - s['last_move']
    if s['cur'] is not None or s['moves'] > 0:   # <- never true without a counter
        ...
        s['moves'] += 1
```

`cur` is `None` for a counter-less job, and `moves` starts at 0, so the guard is false on every
poll and `moves` never leaves 0. The state machine then stops at
`elif s['moves'] < MIN_OBS: state = 'starting'`, which it can never pass.

*Plain terms: the page decides a job is alive by watching its counter tick. A job with no
counter never ticks, so the page never gets enough evidence to call it anything — and a job
frozen at "starting" looks the same whether it is running perfectly or died an hour ago.*

**This affects every counter-less job on that page, not only mapping runs.**

**What to do instead — and what was done.** Give the line a counter even when the total is only
nominal. `slam_progress.py` emits elapsed minutes against a planned duration
(`--expect-minutes`, default 20), capped one below the total while the run is alive so the page
can never decide by itself that the drive has finished. Verified on the Jetson: the job now
reports `cur: 2, tot: 20, pct: 10.0` and reaches `running` once the counter has advanced
`MIN_OBS` times.

**Two vocabulary traps in the same file, both confirmed by reading it:**
- `FAIL_RE` matches **`stopped`** — so a watcher ending normally with "stopped" renders as a
  **failure**. Say "complete" for a clean finish.
- `FAIL_RE` also matches **`stalled`**, which is useful: a job that detects its own stall and
  writes that word is shown as an alarm even though the page's own stall detection cannot fire.

**The dashboard itself was NOT modified** — it is shared infrastructure that other campaigns
depend on. The fix went into the job that writes the line.

---

## Using the colleague's LiDAR mapping settings, as they are, as the reference for the camera

**Tried 2026-09-24 on series-2 drives 1 and 2.** The recordings were replayed through
`self_navigation 3dreplay_pipeline.launch sensor:=helios`, unchanged, and its corrected path
was compared with the camera's.

**How it failed.** It made **zero LiDAR loop closures on both drives**: the Link table holds only
neighbour links (type 0) and gravity links (type 9). Its path is therefore the robot's own
wheel+IMU estimate (`/odometry/filtered`, used with `RGBD/NeighborLinkRefining=false`). Both
drives were parked on the start mark, yet this "reference" ended **3.84 m** (drive 1, 254 m) and
**20.57 m** (drive 2, 293 m) from its own start, with corridors drawn two or three times at
different angles. On drive 2 the camera's corrected map (103 closures, 0.08 m) was far better
than the yardstick meant to check it.

**Why.** Those settings trust the wheels for every step and correct only when a scan matches a
place seen before. Corrections are also capped at 3 m (`Icp/MaxTranslation=3`), and the wheels
had drifted further than that before any same-direction revisit. They were written for the
colleague's own purposes, not as an accuracy reference.

**What to do instead.** Build the reference with LiDAR scan matching for the odometry, so the
drift stays small and the corrections needed stay small. **Do NOT simply raise the 3 m cap**: scan
matching cannot line up scans 10-25 m apart, and wide limits invite false matches between
look-alike corridors (research report 2026-09-24, section 1.2). Replay offline from the same
recordings with our own launch file
(`tools/robot_side/`). **And test every
reference against its own start-to-end gap first:** `compare_lidar.py` now refuses to call a
reference usable if it ends more than 0.5 m from its start or made no LiDAR closures.

---

## Passing an EMPTY launch argument (`database_path:=`) to mean "none" (2026-09-24)

**Tried 2026-09-24, 06:43 Hamilton time, first attempt at test T2 of the occupancy research
report (the live page check).** The replay's RTAB-Map was started with `database_path:=`,
meaning "keep this throw-away map in memory, write no file".

**How it failed.** roslaunch **silently ignores** a `name:=value` argument whose value is empty,
so the launch file's default was used instead:
`/media/sidewalk/SIDEWALK128/rtabmap_maps/sidewalk_front.db`, on the Jetson's microSD card. That
card had been logging ext4 checksum errors (its filing system's self-checks failing) since
04:15 that morning. RTAB-Map died at node 282 of 911 with `database disk image is malformed`,
the test was lost, and the file it created is no longer on the card.

**Why.** `rosgraph/names.py:197` (`load_mappings`, which `roslaunch/loader.py:148` uses to read
command-line arguments) keeps a mapping only `if src and dst`. An empty value is not "empty
string", it is "not given". *Plain terms: leaving a setting blank on the roslaunch command line
does not mean "none"; it means "use the usual one".* No warning is printed.

**What to do instead.** Always pass a real path. For a throw-away replay map, use a file in
`/dev/shm` (a folder held in memory, not on any disk, cleared at reboot), refuse to start if it
already exists, and **check the log line `Using database from "<path>"` before replaying**.
`t2_run.sh` (project records) does all three.
Verify with: `grep "Using database from" <rtabmap log>`.

## Using `rostopic info` to prove a stopped program no longer listens (2026-09-24)

**What was tried.** In `t2_lite_real.sh` (the check that the live map page's tracking banner is
fed by the light message), a copy of the page reading the FULL tracking topic was stopped with
SIGTERM, and `rostopic info /rtabmap/odom_info` was then used to prove nothing listened any more.
**How it failed.** Two seconds after the program had gone (checked by PID and start time), the
master (the ROS program that keeps the list of who publishes and who listens) still named it as a
subscriber. **Why.** The page runs with `disable_signals=True`, so SIGTERM ends it without the
sign-off rospy normally sends the master; the master's list keeps the dead entry until someone
cleans it. That list is registrations, not connections. **What to do instead.** Ask the
PUBLISHING node for its live connections: `rosnode info <node>` and read its "Connections:"
section. That is what decides whether RTAB-Map builds a message at all (it counts live links,
`getNumSubscribers()`, `OdometryROS.cpp` line 946). `t2_lite_real.sh` now does this; evidence of
the stale listing is in `t2_page_test/review_round/lite_real/lite.txt`
(project records), the corrected check in `review_round/lite_real_2/lite.txt`.
*Plain terms: the switchboard's phone book still listed a phone that had been unplugged. Ask the
caller who is actually on the line, not the phone book.*

## Writing anything to the Jetson's microSD card (SIDEWALK128) - it is failing (2026-09-24)

**What was tried.** 07:34 Hamilton: streaming the colleague's two LiDAR databases (1.4 GB) from the
robot to `/media/sidewalk/SIDEWALK128/lidar_stageA/` for stage A.

**How it failed.** The card wrote at **0.1-0.4 MB/s** (61.7 MB/s when tested in August). Its
writes averaged about 90 s each; the kernel reported `sync` blocked for over 120 s; even `ls` on
the card hung until the backlog drained. The copy was stopped at 07:43 by exact PID. Reads are still
normal (76.6 MB/s on 64 MB). An earlier test replay the same morning died with "database disk image
is malformed" when RTAB-Map fell back to a file on this card (see the empty-argument entry above).

**Why.** The card's ext4 filing system has been failing its own checksums since at least
**8 September** (`/var/log/syslog`: "bad block bitmap checksum", "Corrupt inode bitmap",
"Filesystem failed CRC" - 29 lines, the latest 05:14 Hamilton on 24 September). This is the card's
sixth incident in this project.

**What to do instead.** Write nothing new to it. Robot data stays on the robot's own 128 GB card
(user's decision, 24 September). The card still holds 49 GB (`rtabmap_maps/` 35 GB of August
series-1 databases, `archive/rescued_data/` 14 GB): the user confirmed (24 September, 18:00) that
it is **already in cloud storage**, so nothing needs copying off: replace the card when convenient. Left over from the stopped copy, safe to
remove once the card is rescued: `lidar_stageA/s2_static_01/rtab_helios.db.part` (a partial copy;
the original is intact on the robot).


**Update 24 September, about 18:30 - the card SILENTLY DISCARDS every write.** Asked to re-use the
card for camera backups, it was tested with `card_pattern_test.py` (project records) (every
4 KB block carries its own number; the checker proved beforehand with three planted faults):
256 MB written at 33 MB/s, fsync returned success, and read back **directly from the card** as
**65,536 of 65,536 blocks of zeros**. This morning's 640 MB copy also reads as zeros; an August
file on the card still reads as real data. Freeing space does not help - deleting is a write,
and it is discarded too. **Replace the card; test any card with `card_pattern_test.py` (direct
read) before trusting it.**

**Confirmed on a second computer, 24 September 20:10-20:20 (the card was put in another computer's
USB reader, Realtek RTS5129):** the card held NOTHING written on 24 September - not even the
folders the Jetson had listed all day (only its memory had them). A 256 MB pattern written from
that computer read back, after its memory copy was dropped (`dd iflag=nocache count=0`, `fincore` 0 B),
as 65,536 of 65,536 blocks of zeros; a text file too; an August file still reads correctly. So it
is the CARD, not the Jetson's slot. It is genuine (a fake fails only past its real size; this one
refuses every new write - the way worn-out genuine cards lock themselves) and its old contents are
in the cloud. (That computer's reader refuses O_DIRECT reads - "Invalid input" - use the nocache
method there.) The robot's own card passed the same test: 1 GB, direct read, 0 wrong.

**Final test, 24 September 22:40-22:48 - a full format does not stick either.** The user formatted
it on that computer (`sudo mkfs.ext4 -F -L SIDEWALK128 -U 21ea9290-... /dev/sdb1`, no errors), the card
reader was powered off and physically unplugged and re-plugged (nothing cached anywhere), and the
card came back with its OLD contents - `archive`, `recordings`, `rtabmap_maps`, 49 GB used, August
dates. mkfs also reported "last mounted ... Jun 17 2024", although that computer had mounted it twice
that day: those superblock updates were discarded too. **The card is write-locked (silently) and a
format does not undo it. Whether the lock can be reversed is NOT known: a temporary write-protect
flag in the card's CSD register could in principle be cleared (native SD slot only); a lock inside
the card's controller cannot. To be read from /sys/block/mmcblk1/device/csd with the card in the
Jetson's slot. Until then, do not use it.**

**Read on 24 September ~23:00 (card back in the Jetson slot, read-only):** CSD
`400e0032db790003b8ab7f800a404000` - TMP_WRITE_PROTECT (bit 12) = 0, PERM_WRITE_PROTECT (bit 13)
= 0; kernel `ro` 0, `force_ro` 0. CID: manufacturer 0x03 / OEM "SD" / product SN128 = **genuine
SanDisk**, made **12/2024**. So the lock is NOT the reversible register flag: the card's own
controller discards writes while reporting itself writable (the way SanDisk cards behave once
they judge their memory unsafe). **Not reversible by any command. Likely under warranty - claim a
replacement.** Keep it out of the Jetson: it auto-mounts read-write and anything "saved" there
is lost. A replacement should be a high-endurance card, tested with
`card_pattern_test.py` plus an unplug/re-plug read-back before it holds anything.

## Verifying a copy by reading it straight back through the computer's memory (2026-09-24)

**What was tried.** `backup_to_card.sh` made its checksum list from the CARD COPY right after
writing, then "read it straight back" to prove the copy - both reads served from the memory
cache, which still held the bytes that were sent.

**How it failed (demonstrated, not guessed).** On the failing card, the same 256 MB file read
normally gave **0 wrong blocks**; read with `dd iflag=direct` it gave **65,536 wrong (zeros)**.
The old script would have printed "all match" over a copy that was entirely zeros.

**What to do instead.** Make the checksum list from the SOURCE, and read the copy back with
O_DIRECT (`dd if=FILE bs=4M iflag=direct | sha256sum`), so the card itself must produce the bytes.
`backup_to_card.sh` does this since 24 September. The same flaw exists anywhere a copy is checked
with `md5sum`/`sha256sum`/`cmp` soon after writing on the same machine - including this morning's
stage-A copy check on the robot (the later stage-A results, read after reboots, show the robot's
card itself is fine).
## Using `dd iflag=direct` to verify a copy on a newer Ubuntu (2026-09-25)

**What was tried.** Checking a copy with `dd if=FILE bs=4M iflag=direct | sha256sum` (read straight
from the disk, bypassing the memory cache) on a computer with a newer Ubuntu.
**How it failed.** Every file came out different, with fingerprint `e3b0c442...` - the SHA-256 of
**nothing**: `dd` printed `IO error: Invalid input` and read zero bytes. That Ubuntu ships **uutils
coreutils** (a Rust rewrite of the standard tools) as `/usr/bin/dd`, and its `iflag=direct` fails on
ext4. The pre-run test missed it because it ran on `/tmp`, a memory filesystem, not the disk the real
copy landed on. It failed safe (nothing was offered for deletion) but verified nothing.
**What to do instead.** Read directly from Python (O_DIRECT with a page-aligned buffer), or drop the
file's cached pages first. The Jetson (Ubuntu 20.04, GNU `dd`) is unaffected.
**The general lesson: test a check on the same kind of disk the real run will use.**

## 2026-09-25 — An unfinished rsync copy was committed, and blocked every push for 12 days

**What was tried:** `git add` of the depth study's `03_data/svo_7m_0913/` folder on 13 Sept 20:26 while a camera
recording was still being copied into it. rsync writes into a hidden temporary file (`.w3_7.0m_block.svo2.jfnBek`,
a dot, the real name, and six random letters) and renames it at the end. The `.svo2` ignore rule did not match
that name, so the 5 GB half-copy was committed. It was deleted in the next commit, 9 minutes later.
**How it failed:** deleting it later does not take it out of the history. GitHub refuses any push that contains a
file over 100 MB, so none of the 215 commits made after 13 Sept could be pushed (found 25 Sept).
**Fix used:** the history was rewritten without that one file (`git filter-branch --index-filter`, in a separate
copy; final files identical, 211 commit IDs changed). Old-to-new IDs: `docs/COMMIT_ID_MAP_2026-09-25.csv`.
**What to do instead:** never `git add` a folder that a copy or recording is still writing into. Before any push,
check for large files: `git rev-list --objects origin/master..master | cut -d' ' -f1 | git cat-file --batch-check='%(objecttype) %(objectsize)' | awk '$1=="blob" && $2>50e6'`
must print nothing. Suggested ignore lines for recording temp files: `*.svo2.*` and `*.bag.*`.

## Substring process match, fifth time - caught by a guard (2026-09-25, 07:30 Hamilton)

**What was tried.** Stopping the robot's storage reporter with `ps -eo pid=,args= | awk "/[r]obot_storage_agent.py/"`
run inside `ssh robot '...'`. **How it failed.** The `[r]` trick stops awk matching itself, but the whole ssh
command line - which contains the name - is a bash process on the robot, so two bash PIDs matched as well.
**Why no harm.** Each PID was killed only if `ps -o comm=` was exactly `python3`. **Do instead.** Select by
`comm` first, then compare the script argument whole (ENGINEERING_NOTES.md rule 8), exactly as `robot_side.sh`'s `pids_of` does.

## A large copy onto the Jetson during replays stalled them (2026-09-25, 07:47-07:53 Hamilton)

**What happened.** Drive 1 and 2's databases (10.7 GB) were copied from another computer over a wired link into
`~/slam_series2/` (landed 07:47:01 and 07:48:59) while the blend line ran timed replays. Every replay stalled
4-6 s (one 13 s); results from that window were discarded and re-run. **Why [INFERENCE - timing match, no
log line proves it]:** a sustained multi-GB write to the Jetson's internal disk starves other disk and
processor work. **Do instead:** no large transfers onto the Jetson while a replay, bench test or drive runs;
schedule copies between jobs, and never during a drive (the guard in the blend also switches on when the
Jetson stalls).

## LiDAR free-space carving with a fixed stop-short distance erases walls (2026-09-25, 08:40 Hamilton)

**What was tried.** Each carving ray stopped a fixed 0.10 m before its hit. **How it failed.** On a made-up room
only 83.2 % of wall squares and 83.6 % of a person-sized post were kept: rays meeting a wall at a shallow
(glancing) angle run along it and clear it. **Do instead:** stop short by 0.10 m / tan(angle between ray and
wall), as `lidar_carve.py` (project records) does.

## Holding the camera path still while parked does not fix drive 3's camera heading (2026-09-25, 08:55 Hamilton)

**What was tried.** Offline on drive 3 (camera restarts re-chained from the camera's own gyroscope), holding the
camera's path still whenever the robot was parked. **How it failed.** 1.55 m / +12.1 deg / best-fit median
0.96 m against the line written before the test (<= 1.5 m / <= 10 deg / <= 0.9 m); it removed only 2.6 of the
14.7 deg left. **Why:** the remaining heading error is the camera tracker's own, spread over the moving parts.
**Do instead:** record drive 4 so the tracker can be replayed (SVO2 + /rtabmap/odom_info_lite) before testing
any tracker setting. Source: `camera_diagnosis_drive3_2026-09-25/` (project records) (round 2).

## LiDAR carving: low obstacles and uncorrected heading drift (2026-09-25, 09:45 Hamilton)

**What was tried.** Carving each 2D direction up to its nearest return in the wall band (0.15-2.0 m).
**How it failed.** Upper laser rings pass over knee-high objects (a 0.35 m box lost 13 % of its squares; kerbs,
planters, bollards on a sidewalk). With 5 deg/min UNCORRECTED heading drift, carving removes the older, misplaced
copies of walls and opened 13 real gaps in 201 wall pieces. **Do instead:** end beams at any return from 8 cm up
and clear no further than the farthest floor return in that direction (round 2, `lidar_carve.py` 99396c7d);
only run carving on positions with the parked heading drift removed. Source:
`RESULTS.md` (project records) (round 2).

## LiDAR carving round 3 lessons (2026-09-25, 10:05 Hamilton)

- **A scan that is not carved must still add its hits.** Skipping a scan's carving (waiting for drift correction,
  or resting after an expensive scan) also skipped its wall hits, so the carving-on picture drew no walls at all
  while carving waited. Count hits always; skip only the see-through votes.
- **A 1 deg heading margin is too small for 1 deg per-scan jitter.** It opened wall gaps in a 20-minute drift run;
  3 deg (3 sigma of the jitter plus drift inside the 60 s window) did not. Cost: nothing within range x tan 3 deg
  of a return is cleared (a person standing against a wall stays).
- **The wall-gap check cannot judge a hole inside a drift-blurred wall band.** After 20 minutes of drift a
  partition becomes a fan of copies; a 1 m hole cut into it merged with carving's own clearing and was not caught
  (test P3). Read the live gap check next to /map.png and /map_nocarve.png, never alone.
Source: `RESULTS.md` (project records) (round 3).

## Self-matching process check, sixth time (2026-09-25, 23:05 Hamilton) - no harm, but the check was useless

**What was tried.** Before deploying the drive-4 files, a refusal check listed every argument of every process
(`ps -eo args`, split into base names) and refused if a name like `start_drive.sh` appeared. **How it failed.**
The deploy command's own bash line contained all those names (in its file list), so the check matched ITSELF
and printed "REFUSED" for all 7 names - and, because `grep && { ...; false; }` inside a loop does not trip
`set -e`, the deploy carried on anyway. Nothing was actually running (confirmed afterwards; the Jetson had just
rebooted), so the deploy was correct by luck. **Do instead:** check `ps -eo pid=,comm=` first and only look at
the script argument of processes whose comm is python3/bash, skipping `$$` and `$PPID`; make a refusal
`exit`/`return` from a subshell, never `false` inside an `&&` list; test the check against an absent name.

## Night of 25-26 Sept (Hamilton): camera recording, receiver restart, unexplained Jetson resets

- **Camera recording started and stopped repeatedly in one camera session -> crash.** The camera program launched for
  rehearsal T5b (23:50) had its SVO recording started/stopped 4 times across T5b and T5c; in T5c it logged "Error saving
  frame to SVO" at the recording start (00:27:52) and crashed with a segfault (exit -11) at 00:37:20. **Do instead:** a
  fresh camera start per drive; no repeated start/stop of the recording within one camera session; drive 4 ran without
  recording. The recording (mode 2) still needs a clean rehearsal on a fresh camera before any drive relies on it.
- **Restarting the robot-link receiver from saved /proc cmdline/environ with `env -i $(tr '\n' ' ' ...)`** broke on an
  environment value (`env: '40788': No such file or directory`), so the planned 20 s drop became 56 s. **Do instead:**
  relaunch with Python `subprocess.Popen(args, env=dict, start_new_session=True)` from the NUL-separated files.
- **Two Jetson hard resets (21:44 and 23:39, reset reason SYS_RESET_N, no shutdown in the log)**: the second ~1 min after a
  camera recording started, in MAXN mode. User says the original adapter and cable were in place. Cause UNKNOWN; a
  tegrastats log now runs during drives (`~/rehearsal_t5/tegrastats_*.log`) to catch power/temperature before a reset.
- **After a reboot the Jetson's ethernet port came up with no link** ("failed to read UPHY GBE mode - default to 10G")
  although the other end showed a link. `sudo ip link set eth0 down; sudo ip link set eth0 up` renegotiated 1 Gbps.

## Drive 5 (26 Sept 2026, Hamilton): camera crashed mid-drive and nobody was told for 9 minutes
- **What was tried:** drive 5 reused the camera program still running from drive 4 (start_drive.sh: "already publishing"),
  and the drive watcher judged health from the progress line only.
- **How it failed:** the camera program (`zed_wrapper_node`) crashed with a segmentation fault (exit -11) at 02:25:26,
  12 min into the drive, after 1 h 41 min running; no SVO recording was on, memory was fine (13.5/30.6 GB). The launch
  marks it REQUIRED, so it shut down. The map kept running with no images; the progress line kept saying "running";
  the only sign was the park helper's "NO camera positions for N s", which nobody watched. 12 of ~21 min were mapped.
  The park helper then closed the map properly by its 3-min timeout (database PASS, 688 nodes).
- **Why:** root cause of the crash unknown (second exit -11 in one night; the first followed SVO start/stops). The
  silence was a watcher that only looked at one line that cannot see the camera.
- **Do instead:** a fresh camera for every drive, a camera guard that restarts it and raises an alert, and the drive
  watcher pushes a phone notification on "NO camera positions" (camera_crash_recovery_2026-09-26/ (project records)).

## Localisation demo session 1 (26 Sept 2026 05:03-05:10 Hamilton): robot not charged enough - session lost at stop 1
- **What was tried:** the robot charged only about 1 h (03:15-04:15), then drove drive 6 (25 min, ~7.4 GB LiDAR) and
  started the localisation session straight after.
- **How it failed:** at 05:09 the robot ignored the controller. Battery 19.46 V, the robot's own diagnostic "Low power";
  e-stop off, no lockout, controller messages arriving at 20 Hz. Only stop 1 of 5 happened (not localised, see
  run/lib/), so the session cannot be scored.
- **Do instead:** charge fully before a lab block that has two parts (drive + demo). **Do NOT judge the robot's
  readiness from `battery_voltage` in `/status`** - the (the 19.46 V / 'Low power' reading was only context for the event above). The user
  judges the charge; the Jetson operator asks "charged enough for drive + demo?" before a two-part block instead of gating on volts.

## Power loggers left running after drives (26 Sept 2026)
- **What was tried:** each drive/rehearsal started `sudo -n tegrastats --logfile ~/rehearsal_t5/tegrastats_<run>.log`
  by hand from the main session, with nothing stopping it.
- **How it failed:** six loggers (T5b, T5c, drive 4, 5, 6, loc01) were still running at 06:10, each still appending to
  its drive's log hours after the drive (logs no longer end with the drive; analysis must cut by time).
- **Do instead:** stop the drive's tegrastats by PID right after the map closes (same step as the robot-side stop),
  or have start_drive.sh start it and stop_fusion.sh stop it.

## Localisation demo session 2 (26 Sept 2026 06:16-06:34 Hamilton): "localised" at 5 of 5 stops, but 4 were in the WRONG place
- **What was tried:** between stops, `next_start.sh` reloaded a fresh copy of drive 4's map to make the robot "forget";
  the watcher counted "localised" from the published uncertainty / odometry-cache links and scored distance to the
  nearest point of drive 4's PATH.
- **How it failed:** after each reload the pose jumped to drive 4's last saved pose (the start mark) wherever the robot
  was, and stayed there: stops at A, B, A, C were shown ~4-11 m wrong (at the start mark), and the robot parked back
  on the start mark was shown 4.3 m away. The user saw it on the live map. The watcher said "localised" anyway, because
  it counted a single recognition link in the odometry cache as a fix. "0.03 m to the map path" was meaningless -
  every place in a corridor is near the path.
- **Why (corrected after checking the session record, 26 Sept ~07:10):** (1) the reload itself puts the robot on the
  map's saved last position (`Admin.opt_last_localization`, Rtabmap.cpp:1345-1370, "Update map correction based on last
  localization saved in database"); nothing was carried across from the wheel/gyroscope blend. (2) The camera DID
  recognise the right places within 1-19 s (links to map nodes 97, 150, 158, 321 = A, B, A-facing-back, C), but
  RTAB-Map never ACCEPTED them (published uncertainty 9999 all through starts 2, 3, 5): it waits for a second
  recognition while the first is still in its 10-picture memory, and after the first link its recognition threshold
  rises from 0.05 (`RGBD/AggressiveLoopThr`) back to 0.15 (`Rtabmap/LoopThr`, Rtabmap.cpp:2140-2156), so the next one
  came only when the first had dropped out (11-17 moving pictures later) - never a pair. Scores at starts 2, 3, 5
  peaked at 0.096-0.103. Start 4 was accepted because, driving back towards the start area, picture 675 scored 0.157
  (over 0.15) while the first link (665) was still remembered. Proximity matching played no part (0 nearby re-matches
  at starts 2, 3, 5). Evidence: `localisation_demo_2026-09-26/session2_diag/hypotheses.csv`.
- **Fix staged (offline-tested, registered before lab use):** off-map guess in each working copy (`set_prior.py`) +
  `Rtabmap/LoopThr` 0.05 in `localise_run.launch`; `RGBD/MaxOdomCacheSize` 20 does NOT help (tested, 0 of 3).
- **Do instead:** score against the TAPED spot's expected map pose (checked with the LiDAR estimate), never distance to
  the path; the reset must not load the saved correction and should start from a deliberately wrong guess, so only a
  real recognition that lands on the taped spot counts.

## USB stick CAM_REC writes corrupt data (26 Sept 2026 07:35 Hamilton)
- **What was tried:** offloading localisation session 1's records (514 files, 1.3 GB) to the 32 GB exFAT stick CAM_REC
  (`/media/sidewalk/CAM_REC`, mounted by hand with `mount -i -t exfat` after fuse-exfat refused it: "unknown entry type 0x89").
- **How it failed:** sha256 of the copy differed from the original on 3 small files; `camera_guard.json` on the stick
  read back as binary garbage with the SAME size and time stamp. The original was NOT deleted (the check came first).
- **Why:** unknown - the stick's filesystem already showed damage on 25 Sept; possibly the stick itself.
- **Do instead:** do not store anything on CAM_REC until it passes `card_pattern_test.py` + unplug/re-plug read-back
  (as for the failing SIDEWALK128 card). Copy data off over the network instead, always verified by sha256.

## Starting a drive with the robot's address written into start_drive.sh (26 Sept 2026, localisation session 3, 10:41 Hamilton)
- **What was tried:** `start_localise.sh s2_loc_03` with the default robot address. `start_drive.sh` takes the host
  from `LIDAR_PEER`, and otherwise falls back to the fixed default <robot-wifi-address>, for both the bridge (`ROBOT_ADDRS`)
  and the live LiDAR panel (`PEER`).
- **How it failed:** the first start was refused with "the robot's sender is not answering". The robot had been off to
  charge, and the lab WiFi's DHCP (the service that hands out addresses) gave it a new address, <robot-wifi-address>.
  The session was restarted with `LIDAR_PEER=http://<robot-wifi-address>:8095` and ran normally.
- **Why:** the address is not fixed; it has changed at least three times (<an earlier robot-wifi-address> -> a new one on 24 Sept,
  -> another on 26 Sept). `~/.ssh/config` (`Host robot`) had already been updated to the newest, so the scripts and
  the ssh config disagreed.
- **Do instead:** before a drive, read the robot's current address from one place and pass it in. For example:
  `ssh robot hostname -I` with the ssh config kept current, or better, the robot's network name if one
  resolves. Then run `LIDAR_PEER=http://<addr>:8095 start_localise.sh <run>`.
- **Proposed change (not made):** `start_drive.sh` should work the address out, from the `Host robot` entry, by asking
  the robot, or from a small `robot_addr.txt` written when the robot's side starts. It should not fall back to a stale
  fixed address. See OPERATIONS.md, "Robot address changes after charging".

## Camera tracker GEN_1 as the drive default (2026-09-26)
- **Tried:** switched the ZED X position tracker to GEN_1 (Stereolabs' fallback for GEN_2's random crashes), deployed 12:57 after a depth check (jitter 0.99x, pass).
- **How it failed:** drive 7 on GEN_1: 16 camera freezes (drive 4 on GEN_2: 1) and 30 loop closures (drive 4: 181). Parked A/B (ABBA, 4 x 10 min): GEN_1 35 freezes vs GEN_2 15, and the map received ~5 camera positions/s on GEN_1 vs 7-12 on GEN_2. The registered "GEN_1 causes freezes" rule (3x and +5) was not met (2.3x), but the halved position rate is a separate cost.
- **Why:** not established; both engines stall in the driver's picture loop, GEN_1 about twice as often.
- **Instead:** GEN_2 by default (decision 15:40), with the libSegFault crash backtrace and the camera guard (crash back in ~13 s). `TRACKER=GEN_1` still selectable. Evidence: `RESULTS.md` (project records) section 8.

## Passing a path from "the project records..." as an unquoted roslaunch argument (27 Sept 2026, SLAM bench 02)
- **Tried:** `CAMCFG="config_file:=$SLAM_TOOLS/zedx_front_od.yaml"` then `roslaunch ... $CAMCFG`.
- **Failed:** the camera never started ("The following input files do not exist: Internship/SLAM, Research/..."); the
  start refused after 90 s with no pictures. The project folder names contain spaces.
- **Instead:** a bash array, `CAMCFG=("config_file:=$P")` and `"${CAMCFG[@]}"` (empty array = no argument).

## Guessing a ROS message field name (27 Sept 2026, slam_watch.py v1)
- **Tried:** `m.proximityClosureId` on `rtabmap_msgs/Info`. On this build the field is `proximityDetectionId`
  (`python3 -c "from rtabmap_msgs.msg import Info; print(Info.__slots__)"`). Every callback died; the live JOINED line
  said "no" while the map had joined. Also: an event-only watcher started after the event misses it - read state
  (`/rtabmap/mapGraph`) instead. **Instead:** print `__slots__` before using a field (ENGINEERING_NOTES.md 0.3 rule 4).

## Navigation (27 Sept 2026, simulator runs 1-10)
- Saved map walls in move_base's LOCAL collision check: drive 10's map narrows the long corridor to ~1.0 m at x 12.4-13.2,
  the robot refused to move. Use the saved map for global planning only; live ZED depth for the local check.
- Keeping camera marks under the robot's footprint: the planner saw its own cell blocked, every goal failed.
- In-place turn minimum 0.2 rad/s (Clearpath default): stalled in the simulator; use 0.3.
- `~/sidewalk_sim/demo_sim_jetson.sh` runs `pkill -f rosmaster` / `pkill -f gzserver` - never use it while anything live runs.

## WiFi roaming helper v2 reassociated on a STRONG link and made an 8 s blip into a 6-minute outage (27 Sept 2026, 11:32 Hamilton)
- **What was tried:** v2 treated any 6 s without a gateway ping reply as "no link" and ran `wpa_cli reassociate`, then retried every ~11 s.
- **How it failed:** the Jetson was at -56 dBm with a LiDAR replay and a robot-card copy running. At 11:31:52 the pings stopped for 8 s and v2 reassociated. The access point rejected it (`CTRL-EVENT-ASSOC-REJECT status_code=1`), and the link never came back. After 405 s of retries the Jetson could not be reached by ssh, so the user power-cycled it at about 11:39. The replay (46 s in) and the drive 3 card copy were lost.
- **Why:** a ping gap with a strong signal means the problem is past the access point, or the Jetson is busy; reassociating cannot fix that and can get rejected.
- **Instead (v3):** on a strong link (>= -67 dBm), wait 30 s before any repair, and double the gap between failed repairs (8 -> 16 -> 32 -> 60 s). Weak-link behaviour is unchanged.

## People mask compared the ZED label to "Person" - the SDK sends "PERSON" (s2_slam_01, 27 Sept 2026)
- **What was tried:** `depth_person_mask.py` kept a box only if `o.label == "Person"`; its self-test built fake objects labelled "Person", so it passed.
- **How it failed:** in the SLAM drive the detector ran (messages every frame, age 0.1 s) but 0 of 18,980 pictures were masked. Live check after the drive: the message reads `label: "PERSON"`, `sublabel: "Person"`, confidence 93.7 at 1.65 m.
- **Why:** the known-answer test used labels typed by hand instead of the real message values.
- **Instead:** match `label` or `sublabel`, ignoring case (`is_person()`); the self-test now uses the SDK's own labels. **Rule: a self-test's fake inputs must copy a real recorded message, not a guess.** s2_slam_01 is therefore the MASK-OFF run; the mask-on result comes from replaying its SVO with the fixed mask.

## CAM_REC USB stick as staging (27 Sept 2026)
- **Tried:** the Jetson's 29 GB exFAT stick CAM_REC as a staging area (mount needs `sudo mount -i -t exfat ...`; the old FUSE exfat 1.3.0 fails with "unknown entry type 0x89").
- **Failed:** 1 GB of random data written with fsync read back with a different sha256; writes ran below 3.4 MB/s.
- **Why:** an unbranded ("VendorCo ProductCode") stick, probably fake-capacity or failing.
- **Instead:** stage on the Jetson internal disk and copy off over the network (size + checksum checked).


## Navigation: tried after route B's front-left corner touched a wall post (27 Sept 2026, 16:36-17:15)
- **Cause (real run s2_nav_01):** the LOCAL (wheel-check) costmap held camera obstacles only; the ZED X is set to ignore
  anything nearer than 1.0 m (min_depth, deliberate) and sees nothing beside its own corners, so a post 0.59 m from the
  robot's centre was invisible while it turned on the spot. Photos: .run_records/s2_nav_01/nav_B_part2/.
- **Kept:** local costmap = drive 10's walls minus the cells drive 10's own outline drove over (maps/d10_nav_local,
  make_local_walls.py); the same cleaning for the planner map (d10_nav_clean); route B2 (turns only at the widest
  points ON drive 10's path; straight line y = -14.62 through the south corridor's 0.9 m stretch); nav_cmd_sender.py
  27b real-time clearance guard (stop < 2 cm, slow below 0.40 m, turn on the spot only if corner radius 0.60 + 5 cm is
  clear); local costmap always_send_full_costmap. Sim run 18: route B2 3 of 5 (SE turn and the narrow stretch passed;
  overshot the SW turn by 1.1 m - the simulator's known turn-slide - leash stopped it safely).
- **Did NOT work:** eroding / skeletonising the saved walls (erased the post); footprint padding 0.10 (sim run 14: "no
  valid control" at the east junction); guard stop at 5 cm (stalled in the 1.0 m pinch, run 15); global inflation 1.0 m
  with cost_scaling 2.0 ("NO PATH ... legal potential was found", runs 11/16) and use_grid_path true (stalled before
  the pinch, run 17). Global planner is back to the settings proven by real route A.
- **Rule 8 slip:** a kill loop matched "run_sim_dryrun.sh.*run12" as a SUBSTRING of every command line and also hit
  its own shell. Use exact argv matching (argv[1] basename, argv[2] exact suffix), as in scratchpad next_sim.sh.
