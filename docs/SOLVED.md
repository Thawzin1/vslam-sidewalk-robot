# SOLVED.md — things that worked

Format per entry: **symptom → cause → fix → the command that verifies it.**
Read this (and `DO_NOT_REPEAT.md`) before proposing any change (ENGINEERING_NOTES.md section 0.2).
Newest entries at the top of each section.

---

## Real ZED X hardware

### The microSD card destroyed a run mid-write, then corrupted a second file at rest (2026-08-24)
- **Symptom:** run `lab_map_02` (the closed-loop accuracy attempt on the real robot) ended with its
  database at 73 KB / "database disk image is malformed" and its recording reverted to **0 bytes**
  (it had been observed at 10.4 MB mid-drive). Looked like a Jetson power cut. It wasn't.
- **Evidence it was the CARD, not power or software:** the mapping program's own log, written to the
  **internal disk**, kept growing for **20 minutes after** the SD-card files' timestamps froze
  (01:12 → log last write 01:32) — the software outlived the storage. Then the independent
  confirmation: `office1_explore_04.db` on the same card verified "ok" on 2026-08-23 and read
  **"database disk image is malformed" on 2026-08-24 with no writes in between** (`PRAGMA
  quick_check`, run by hand both days). A card that corrupts data at rest is failing.
- **Why no kernel log exists:** this Jetson's system journal is memory-only (no persistent journal
  under `/var/log/journal`), so the failure boot's kernel messages did not survive the reboot.
  File-level evidence above carries the diagnosis.
- **What was lost:** run 2's map and recording (the closed-loop measurement — must be re-driven).
- **What was saved:** run 1 in full — `lab_map_01.bag` and `lab_map_01.db` plus
  `office1_explore_03.db` copied to the internal disk at `~/rescued_data/`, each copy
  **checksum-verified** against the original before being trusted.
- **Rules (now in ENGINEERING_NOTES.md section 0.3):** (1) experimental data is written to the **internal disk**
  during a run and moved to the SD card only after a clean close; (2) the SD card is treated as
  failing — nothing irreplaceable lives only there; (3) the project memory's old warning ("keep
  USB/SSD as primary for critical recordings") was right and is now enforced, not advisory.
- **Verify:** `md5sum` original vs copy for every rescued file; `PRAGMA quick_check` on the rescued
  databases from their new location.
- **THIRD file confirmed damaged (same day, later):** run 1's recording `lab_map_01.bag` read
  cleanly with `rosbag info` at 01:04 (27:53, 520,052 messages) but by the evening contained
  corrupted internal blocks (a chunk header claiming 205 MB inside an 88 MB file) — damaged **at
  rest** on the card; the checksum-verified rescue faithfully copied the already-damaged bytes.
  `rosbag reindex` on a work copy recovered the **first 7 min 14 s** (15.2 MB, 132,314 messages,
  including both position-estimate streams and 29 map snapshots); the remaining ~20 minutes are
  lost. The rescued **database** re-verified "ok" from the internal disk — the map itself is fully
  intact, and it stores one camera picture per map position, so the drive-through video and all
  map/trajectory products survive. Lesson sharpened: a checksum proves the copy equals the source —
  it says nothing about whether the source was still good. **Verify content readability (quick_check
  / rosbag info) at rescue time, not just checksums.**

### "Camera not detected" on a working ZED X — two separate traps, neither about the cable (2026-08-23)
- **Symptom:** the ZED X was physically connected and powered, but `ZED_Diagnostic` reported **"Camera not detected — make sure the camera is plugged in or try another USB 3.0 port"**. Then, once that was seen through, `sl.Camera.open()` failed with `(Argus) Error BadParameter ... eglstream/FrameConsumerImpl.cpp` → `CAMERA FAILED TO SETUP`. Easy to read either as a hardware or cabling fault. Neither was.
- **Trap 1 — `ZED_Diagnostic` does not support GMSL2 cameras.** It enumerates USB devices only. Its advice to "try another USB 3.0 port" is nonsense for a ZED X, which is a **GMSL2** camera on a capture card — there is no USB port involved. It will report "not detected" for a perfectly healthy ZED X, forever.
  - **Use instead**, in increasing order of authority:
    ```bash
    dmesg | grep -iE "zedx|max96712|max9295"   # kernel: did the GMSL2 link train?
    python3 -c "import pyzed.sl as sl; print(sl.Camera.get_device_list())"   # SDK truth
    ```
    A healthy camera shows `zedx_probe: Serial Number : <SN>` + `success` in dmesg, and the SDK prints `model: ZED X  state: AVAILABLE`. Both were true here the whole time.
- **Trap 2 — opening a ZED X needs `DISPLAY` and `XAUTHORITY`, even for fully headless capture.** NVIDIA's Argus framework backs the camera with an **EGLStream**, and its `FrameConsumer` needs a valid EGL/display context. Over a plain SSH session with no usable display, `Camera.open()` fails with the `eglstream/FrameConsumerImpl.cpp BadParameter` above — *not* an error that mentions displays at all. Restarting `nvargus-daemon` does **not** help (tried; the `NvPclStartPlatformDrivers: Failed to start module drivers` line in its log is a symptom, not the cause).
  - **Fix — the same two exports already required for GUI apps** (`docs/OPERATIONS.md`, "Remote GUI access"):
    ```bash
    export DISPLAY=:1
    export XAUTHORITY=/run/user/1000/gdm/Xauthority   # re-derive per that section, do not assume
    python3 your_zed_script.py
    ```
    Identical script, identical hardware: **fails without them, works with them.** Verified back to back.
- **Verify (first light, headless, no window):** open at `HD1200`, grab 40 frames, and read back calibration + features. Healthy result on this unit:
  ```
  ZED X · S/N 47637666 · FW 2001 · 1920x1200 @ 30 fps · 40/40 frames
  fx 1258.51 px · baseline 120.13 mm
  ORB features median 1000 (detector cap) · depth coverage 75.4% · range 1.01–34.76 m
  ```
- **Also worth knowing:** only **one** process may hold the camera at a time. A forgotten `ZED_Diagnostic` window keeps a `CameraProvider` open and blocks everything else — check `pgrep -af "ZED_|zed_"` before concluding anything is broken.
- **Rule this generalizes:** the `DISPLAY`/`XAUTHORITY` pair is not only about *where a window appears*. On this Jetson it also gates **camera capture**, because Argus is EGL-backed. Treat those two exports as mandatory for any ZED X work over SSH, GUI or not.

## Measurement & evaluation

### The sim camera was blind below 1 m indoors — two stacked artifacts, 170 resets in one drive (2026-08-23)
- **Symptom:** the first hand-driven office-room-1 exploration (`office1_explore_03`, 1206 m over 78 min) suffered **170 odometry resets = 14.1 per 100 m** against the project's own gate of **< 2** (`docs/PHASE1_RESULTS.md:6`, `reset_gaps.py:112`) — a 7× fail, ~22.7 m of real path silently deleted, more than double the 8.96 m that made `run2b` "catastrophic". Nothing in the pre-drive checks predicted it: the standing feature gate measured 108–1000 ORB keypoints in every room. Phase 1 outdoors never saw this.
- **Cause — TWO independent artifacts, both close-range, both invisible outdoors:**
  1. **`<near>1.0</near>` on the STEREO cameras** (`sidewalk_bringup/urdf/zedx_mount.urdf.xacro`). On a `type="depth"` sensor `<near>` genuinely is the depth blind zone, and the depth branch sets 1.0 to mirror the real camera's `min_depth: 1.0` — *correct there*. But the stereo branch is two plain **colour** cameras, where `<near>` is a **render clip**: geometry closer than it is **not drawn at all**. The value was copied across for apparent consistency and silently does a different thing. Depth in stereo mode comes from disparity inside RTAB-Map, already floored at 0.70 m by `Stereo/MaxDisparity: 128.0` (`rtabmap_zedx.yaml:275`; 750 px × 0.120 m / 128) — so the render clip bought nothing and cost everything. **A real ZED X still SEES a surface at 0.5 m, it just returns no depth for it.** The sim was harsher than the hardware.
  2. **Untiled wall visuals.** Gazebo paints one texture copy per box face, so each 14 m wall showed a single enormously stretched copy with no corners. The floor had been split into a 4×4 grid for exactly this reason years earlier (its own comment says so) and `docs/OPERATIONS.md` states the rule — **the walls were never given the same treatment.**
- **Why the standing feature gate missed both:** it is measured from the spawn point, in open space. Both artifacts only bite within ~1 m of a surface. In a 13×11 m office the robot is inside that zone constantly (doorways, room corners, wall approaches); outdoors it never is.
- **Fix:** stereo `<near>` 1.0 → **0.1** (both lenses; they must match or disparity breaks in the overlap). Depth-branch `<near>` **left at 1.0 — it is correct there**, and both sites now carry comments saying so. Wall visuals split into 116 tiles of ~1.4 × 1.25 m, each segment **keeping its own distinct material** (`OPERATIONS.md`: textured AND distinct at once — tiling fixes the texture half without touching the distinctness half). **All 12 `<collision>` boxes untouched**, so building shape, contacts and routes are unchanged.
- **Verify** — park the robot facing a bare wall head-on and step in, confirming from `/gazebo/get_model_state` that it actually reached each distance (a blocked teleport fakes a collapse):

  | camera→wall | original | near-clip only | + tiled walls |
  |---|---|---|---|
  | 1.50 m | 336 | 310 | **1000** |
  | 1.00 m | **31** | 517 | **1000** |
  | 0.90 m | **0** | **15** | **1000** |
  | 0.60 m | **0** | **0** | **1000** |
  | 0.40 m | **0** | **0** | **1000** |

  Below 1 m the wall's image **contrast collapsed 24 → 7.3**: it was gone from the render, not merely dim. Room sweep after the fix regressed nothing (NE meeting room's worst heading 323 → 785; global minimum 108 → 128, SW office).
- **The diagnostic that separated the two causes:** at the *same* 0.90 m, a bare wall gave **13** features while a 0.6 m prop carrying a full sharp texture copy gave **236–790**. Same camera, same distance, 20–60× apart ⇒ the residual after the near-clip fix was texture *scale*, not visibility.
- **Rules this generalizes:**
  - **`<near>` means two different things depending on sensor type.** On `type="depth"` it is the depth blind zone; on a colour camera it deletes geometry from the image. Never copy the value between them.
  - **A feature-count gate measured only at the spawn point cannot certify an indoor world.** Measure at the *closest range the robot will actually drive*, not just in open space.
  - **Tile every large surface, not just the floor.** The rule existed in `OPERATIONS.md` and had been applied to exactly one surface.

### route_from_drive.py never once ran on the project's own pinned Python 3.8, and didn't parse there (2026-08-23)
- **Symptom:** after six full rounds of build-and-independently-verify work on `route_from_drive.py` (see the entries below), copying the finished file to the Jetson for the first time and running `python3 -m py_compile` there failed outright: `SyntaxError: EOL while scanning string literal` at `build_header()`'s run-id line. The file had never actually run anywhere but the author's own computer.
- **Cause:** every one of the six verification rounds — build, fix, and adversarial reverify agents alike — ran `python3` on the author's own computer, which resolves to Python 3.14. `ENGINEERING_NOTES.md` pins this project to Python **3.8** (`Ubuntu 20.04.6`), confirmed live on the Jetson (`python3 --version` → `3.8.10`). One line used a multi-line f-string whose `{}` expression split a string literal across two physical lines (`f"...: {run_id or '(could not be ' 'inferred...)'}"`, continued on the next line) — legal under 3.12+'s relaxed f-string grammar (PEP 701), a `SyntaxError` on 3.8. No amount of testing on that computer could ever have caught this: the interpreter itself was the wrong one, not the test cases run against it. All six rounds' adversarial rigor (dense rate sweeps, hundreds of synthetic files, mathematical proofs of noise-independence) verified the *logic* thoroughly and correctly, while the *target platform* was never in the loop at all.
- **Fix:** compute the fallback string into a plain variable before the f-string (`run_id_display = run_id or "(could not be inferred — see note below)"`), then reference it in a single-line f-string. Grepped the rest of the file for the same multi-line-f-string-expression shape; this was the only instance.
- **Verified:** `python3 -m py_compile` passes on both the author's computer (3.14) and the Jetson (3.8.10, the real target). Ran a full synthetic end-to-end conversion on the Jetson itself (not just a syntax check) — 2 m forward, in-place 90° turn, 2 m forward — and got back exactly `{forward: 2.00}`, `{turn: 90}`, `{forward: 2.00}`, self-check 0.000 m / 0.0°.
- **Rule this generalizes:** a tool meant to run on a specific pinned interpreter has to actually run there at least once before being called done — a desktop's newer, more permissive Python is not a stand-in for the target, the same way a noise-free simulated camera was never a stand-in for the real one (see the Phase 1 entries below). Verify on the real target, not just a convenient one.

### route_from_drive.py's round-4 turning-point splitter fragmented single pivots at realistic recording rates (2026-08-23, round 5)
- **Symptom:** round 4's `_find_turning_points()` fix (multi-reversal decomposition via `PIVOT_SWING_MIN_DEG`, a fixed-degree pullback threshold on the run's raw sample-by-sample cumulative heading curve) was validated against a synthetic generator whose sample count per trial was capped independent of recording rate. Independent round-5 verification found that holding a genuine single 75° pivot's angle/rate/jitter fixed and raising ONLY the sample count (via recording rate, ~120 → ~1500 samples for the same physical rotation) raised fragmentation from 0% to 65%. Through the real CLI at 50 Hz, one genuine 75° in-place turn was written as four separate primitives (`{turn: 48}{turn: 17}{turn: 24}{turn: -15}`), self-check reporting PASS throughout.
- **Cause:** a FIXED-DEGREE threshold tested against the RAW, un-smoothed cumulative curve is, in effect, one independent noise comparison per raw sample — an order-statistics problem whose false-positive rate grows with how many samples a pivot happens to have been recorded with, for the identical physical motion. No single degree value is simultaneously safe at ~46 samples and ~1500 samples.
- **Fix (three coupled pieces, all needed together — fixing only one reintroduced the same rate-dependence from a different angle, see below):**
  1. `_find_turning_points()` no longer thresholds the raw curve directly. It confirms a reversal via a rate-sign hysteresis test on `_pivot_window_signed_rate()` — a SIGNED, time-windowed (fixed `PIVOT_WINDOW_S`) heading-rate estimate, the same windowing discipline `PIVOT_WINDOW_S`'s classifier already uses successfully — evaluated only at CHECKPOINTS spaced by `PIVOT_TURN_CHECKPOINT_SPACING_S` (`PIVOT_WINDOW_S/2`) seconds of real time, never once per raw sample. A first attempt kept the per-raw-sample cadence and only swapped the criterion to a windowed rate; measured directly, fragmentation at the exact regression case still climbed from 8% (25 Hz) to 73% (150 Hz) — the number of independent test *opportunities* was still tied to sample count, not the noise level of any one test. Time-spaced checkpoints bound the number of opportunities to (duration / spacing), independent of recording rate.
  2. `PIVOT_MAX_GAP_SAMPLES` (a fixed sample count merging nearby raw `is_rotating[]` runs) was ALSO a rate-dependent bug in the same family — it bridges less real time the faster the recording, so classification-noise dropouts inside one genuine pivot increasingly failed to re-merge at high rates (measured: raw-run count for the same physical pivot grew from ~9 at 25 Hz to ~43 at 150 Hz). Converted to `PIVOT_MAX_GAP_S = PIVOT_WINDOW_S * 2/3` (seconds).
  3. The sign-aware merge decision (round 3's doorway-zigzag fix) used each raw run's own literal degree delta to judge direction — unstable for the 1-2-sample fragments the gap-merge now had to reassemble. Replaced with `_pivot_window_signed_rate()` anchored at the gap's own two boundary samples. A narrower window (`PIVOT_MAX_GAP_S`) was tried first and made things WORSE, not better: a windowed rate's absolute noise floor doesn't shrink with a smaller window (fixed by two endpoint samples' own jitter), so dividing by a smaller `dt` inflates noise-to-signal in the resulting rate. Using the full `PIVOT_WINDOW_S` fixed it, re-verified not to blur across two genuine oppositely-signed pivots separated by 0.05-0.4 s.
  4. `_pivot_window_signed_rate()` itself was first written to AVERAGE its forward/backward windows (reasoned as the natural centred estimate); measured directly, this reintroduces the exact dilution problem `PIVOT_WINDOW_S`'s own docstring already diagnoses for the top-level classifier: a middle phase shorter than `PIVOT_WINDOW_S` (e.g. a small phase sandwiched between two longer opposite-direction legs) dilutes BOTH windows, and averaging combines two diluted readings into a still-diluted one. Concrete before/after: `+30/-8/+25` degrees at 20 deg/s, zero jitter — a deterministic case, no noise to blame — went from 2 correctly-recovered events (averaged) to 0 additional splits (magnitude-picking: return whichever of forward/backward has the larger absolute value, since dilution always pulls a window's magnitude toward zero).
- **Verified:** exact regression case (75°, 15°/s, σ=2.0°, 50 Hz) now single-primitive 96-99% across N=150 trials at 25/50/100 Hz, and 97-99% flat across an independent 20-150 Hz sweep (N=100/rate) — no rate trend. Round 4's own original repro (`+15/-20/+14°` + sub-floor settle tail) recovers all 3 phases 94-100% across 20-150 Hz (never below 2/3, never the round-4 zero-event collapse). Full accumulated `.tum` regression bank (409 files, `verify/`+`myverify/`+`verify4/`): 0 crashes, 0 unexpected exits, 0 self-check failures. 175 files showed fewer confirmed events than the pre-round-4 baseline — ALL 175 confined to `verify4/out_search/` and `out_silent_drop/`, a deliberately adversarial micro-phase search using phase durations (0.16-0.4 s) below `PIVOT_WINDOW_S`'s own already-documented resolution floor; zero such regressions anywhere else in the bank, and all 175 still self-check PASS.
- **Known residual limitation (disclosed, not fixed this round):** a turn rate recorded at *exactly* `PIVOT_MIN_ANGULAR_SPEED_DEGPS` (8 deg/s) sustained for many seconds under jitter can still fragment heavily (e.g. 84 raw `is_rotating[]` runs for one continuous 90° rotation at 8 deg/s, σ=2°) — the classification admission test itself sits exactly at its own floor for the whole rotation, which is an `is_rotating[]`-level (round-1, untouched) issue, not this round's `_find_turning_points()` mechanism. Not a realistic human-driving scenario (no one holds an exactly-constant 8.000°/s in-place turn for 10+ seconds); flagged for awareness, not chased further this round.
- **Verify:** `python3 -m py_compile catkin_ws/src/sidewalk_sim/scripts/route_from_drive.py`; the embedded `_selftest_quat_to_yaw()` runs on every invocation.

### route_from_drive.py silently dropped in-place pivots under ~20-25° (worse at faster, equally realistic turn rates) (2026-08-23)
- **Symptom:** independent verification of the new `route_from_drive.py` converter (turns a recorded ground-truth drive into a `route_player.py` route file) found a real in-place pivot as large as 30° could be measured as low as 17.5° or dropped with zero output, no warning — worse the faster the human turned. An end-to-end test with two genuine ±15° pivots produced a route with BOTH pivots completely absent, and the tool's own self-check reported a deceptively small 0.002 m / 4.8° discrepancy only because the two dropped turns happened to nearly cancel at the endpoint.
- **Cause:** `detect_pivots()` classified each sample by looking FORWARD `PIVOT_WINDOW_S=0.3s` and testing whether net displacement/heading change over that window looked like an in-place turn. Near a pivot's trailing edge, the forward window necessarily spills past the end of the rotation into the straight leg that follows, diluting the measured angular speed there below threshold even though the sample is still genuinely rotating — worse for a faster (shorter-duration) pivot, since a larger fraction of the fixed-length window lands on inert straight driving. The confirmed run's own `delta_deg` is summed only over the samples that survive classification, so a diluted trailing edge shrinks the reported run and undercounts the true rotation, or drops it entirely below `PIVOT_MIN_TOTAL_HEADING_DEG=12°`. Raising the threshold or narrowing the window would only move the same dilution problem onto a different (smaller or faster) true pivot, not fix the mechanism.
- **Fix:** classify each sample with TWO windows — the original forward-looking one plus a mirror-image BACKWARD-looking one (`_pivot_window_speeds()` in `route_from_drive.py`) — and count the sample as rotating if EITHER window qualifies. The backward window is diluted at a pivot's LEADING edge instead, so between the two, any sample genuinely inside a pivot at least `PIVOT_WINDOW_S` long always has one dilution-free window looking at it. No constants changed; the fix is entirely which samples survive classification, not the confirmation threshold or the delta_deg arithmetic. (A pivot physically shorter than `PIVOT_WINDOW_S`, i.e. at/under the confirmation floor driven very fast, can still dilute both windows — a genuine resolution limit of any fixed-window classifier, not a regression.)
- **Verified:** a turn-rate × true-angle sweep (15/30/60 deg/s × 8-90°, isolated single pivot between short 0.35 m legs) dropped from up to 12.9° of undercount (worse at faster rates) to under ±0.65° everywhere above the 12° confirmation floor, roughly rate-independent. The two-pivot wraparound end-to-end case now emits both `{turn: 15}` / `{turn: -15}` primitives (previously zero pivots, one lucky-cancellation 5° RDP corner). Re-ran the original synthetic test (forward 3, turn 90, forward 2, turn -45, forward 1.5), a continuously-curving doorway case (must NOT detect a pivot — still 0), and an out-and-back with a dogleg return: all unchanged in primitive count/magnitude, self-check discrepancies equal or better, only the reported pivot *duration* grew closer to the true physical duration.
- **Verify:** `python3 sweep_pivot.py`-style sweep (reproduction script + wraparound/doorway-curve/loop-revisit TUM generators in the verification pass's scratchpad) against `route_from_drive.detect_pivots()`; confirm undercount stays under ~1° across turn rates, and that `{turn: ...}` primitives near ±15° survive for realistic doorway-threading pivots.

### Phase-1 campaign complete: covariance honest, resets gone, yield 93%, ATE now means something (2026-08-22)
- **Result (16 runs, full ladder, all details in `docs/PHASE1_RESULTS.md`):** resets/100m gate PASS (worst 1.89, vs run2b deleting 9 m of path); σ-clamp gate PASS in B1 (0 pinned frames of ~2,300/run, was ~5%); yield gate honest FAIL at 93.2% best (A+Y); ATE ladder attributable per rung.
- **The finding that reframes everything:** arm A's 0.115 m median ATE was the artifact of a noise-free camera; with real noise (B1) the honest figure is 0.474 m med / 0.42% of path — textbook stereo-VO territory, and the number the real robot should resemble. "Accuracy got worse" is the sim getting truthful.
- **Also measured:** the RGB-D `openni_kinect` plugin delivers only ~10 Hz of its configured 15 (the stereo multicamera delivers the full 15.00) — part of every historical yield problem was the camera, not the consumer. `Odom/ImageDecimation` 2→4 lifted odometry 8.3→9.3 Hz (RGBD) and 7.9→12.3 Hz (stereo) with feature headroom to spare.
- **Open question handed forward:** 0 loop closures accepted in ALL arms even with honest covariance — the run-#4 explanation no longer applies; first experiment for the Phase-2 office worlds.
- **Verify:** `python3 /tmp/p1_aggregate.py` on the Jetson; per-run `~/.run_records/p1_*/checklist.txt`.

### Stereo multicamera sensor: real noise, exact stamps, honest covariance (2026-08-22, Phase 1)
- **Symptom:** the depth sensor ignores `<noise>` (rulebook section 2.2), so sim odometry covariance pinned at the 1e-4 clamp and every loop closure was rejected.
- **Fix (validated live, V1–V8):** one `<sensor type="multicamera">` with left/right `<camera>` children (names MUST be literally left/right — the plugin keys topics and baseline off them), gaussian `stddev 0.005`, single `libgazebo_ros_multicamera.so` plugin, `hackBaseline 0.120`, `frameName` = left optical. `zedx_stereo:=true` on the sim launches; `stereo:=true` on the SLAM launches (stereo_sync + stereo_odometry, same RGBDImage bundle downstream).
- **Measured:** delivered temporal noise **1.229 grey levels** (predicted 1.3; the broken depth path measured 0.0002); left/right stamps **70/70 bit-identical** → stereo arms run `approx_sync:=false`; right camera_info baseline exactly **0.1200 m**; odometry std dev **0.0043–0.0050 m honestly distributed, 0 frames pinned at 0.0001**; shakedown route: pos_err 0.052 m over 5 laps, 0 resets.
- **Deviation from section 5's letter** ("two plain cameras") is deliberate: multicamera is the same CameraSensor render path section 2.2 blesses, with guaranteed-identical stamps the two-sensor form cannot give. section 5 amended.
- **Verify:** `python3 /tmp/stereo_probe.py` and `/tmp/noise_probe.py` patterns (in scratchpad); or `grep -c 'std dev=0.000100m' <run log>` → 0 in stereo mode.

### gzserver camera rendering needs DISPLAY even with gui:=false (2026-08-22)
- **Symptom:** headless stereo sim launched fine but published no camera topics; gzserver log: `Unable to create MultiCameraSensor. Rendering is disabled.`
- **Cause:** camera sensors are rendered by gzserver, which needs a GL context; an SSH launch without `DISPLAY`/`XAUTHORITY` exports has none. (Every earlier launch had the exports; the first `gui:=false` attempt dropped them.)
- **Fix:** ALWAYS export `DISPLAY=:1` + `XAUTHORITY=/run/user/1000/gdm/Xauthority` before any Gazebo launch, GUI or not.
- **Bonus:** `gui:=false` headless runs at near real-time factor 1.0 (the old GUI sim ran at 0.26) — recorded campaign runs are ~4× faster than expected.

### Scripted A/B route: IMU heading + encoder distance (2026-08-22, two failed versions first)
- **v1 open-loop failed:** skid-steer arcs under-rotate; robot walked ~2 m wide and wedged against parked car_1.
- **v2 wheel-odom heading failed:** in-place skid turns slip so hard the encoder frame rotates vs the world; "corrected" track wandered 14 m off the road in 3 laps.
- **v3 WORKS:** heading from the platform IMU (`/imu/data`, absolute, slip-immune), distance from wheel encoders, both vision-blind so the A/B stays unconfounded. Shakedown: contained to x[-9.9,0.5] y[-63.5,-60.5], returned to 0.8 m of start after 5 laps.
- **Files:** `sidewalk_sim/scripts/route_player.py` + `sidewalk_sim/routes/mcity_loop.yaml` (out-and-back with alternating-direction 180s to cancel turn-scrub bias).
- **Verify:** the shakedown containment awk in `docs/PHASE1_RESULTS.md`.

### kill -9 on roslaunch orphans its nodes — full sweep before every campaign run (2026-08-22)
- **Symptom:** relaunching after a `kill -9` teardown produced a sim with NO robot_state_publisher and NO /joint_states: the orphaned old nodes and the new ones killed each other in the same-name registration battle; a stuck controller_spawner burned 96% CPU.
- **Fix:** teardown = SIGINT the roslaunches, wait, then kill -9 stragglers BY PID, then verify `ps` clean before relaunching; if the graph is suspect, also kill the sim's rosmaster (11311 — NEVER 11312, the real camera's) for a stale-param-free slate.
- **Verify:** `ps -eo pid,comm | grep -E 'roslaunch|gzserver'` empty between runs.

### TUM fr1/desk control run: install healthy, evaluator validated (2026-08-22)
- **Symptom (question):** are the outdoor failures the pipeline's fault or the world's?
- **Result:** RTAB-Map 0.21.13 with ALL DEFAULTS on TUM fr1/desk → ATE RMSE **0.043 m** (published band ~0.02–0.05 m). The install is healthy; outdoor failures are the sensor/world model.
- **Bonus:** three scorers agree — internal 0.043158, `evo` 0.043370, our `evaluate_trajectory.py` 0.04337 (identical to evo to 5 dp) — our evaluation maths is validated against the standard tool on public data.
- **Verify:** `cat ~/datasets/tum/rgbd_dataset_freiburg1_desk/rtabmap_rmse.txt` (Jetson); full table in `docs/PHASE0_RESULTS.md`.
- **Pin:** `evo==1.31.1` (≥1.32 needs Python ≥3.10; Jetson is 3.8). Installed `--user` on the Jetson.

### run2b's Sim(3) "calibration error" was teleports — intrinsics acquitted (2026-08-22)
- **Symptom:** whole-run Sim(3) on `mcity_v2_run2b` reported factor 0.4648 (estimate 2.15× oversize) with a warning blaming camera intrinsics.
- **Cause:** the estimate's spatial extent was inflated by teleport excursions across 12 tracking gaps, not by scale error.
- **Fix (diagnostic, now standing tooling):** `evaluate_trajectory.py --split-at-gaps` fits Sim(3) per clean segment. Result: segment scales cluster at ~1.00 (0.9984–1.0023 on the well-tracked ones); nothing near 0.465. **Do not touch intrinsics.** Clean segments also show ATE 0.06–0.13 m over 44–88 m — tracking between losses is good; the whole 26 m ATE lives in the gaps.
- **Verify:** rerun the command in `docs/PHASE0_RESULTS.md`; `rtabmap_longest_segment_scale_sim3 = 0.9665`.

### 12.3 m of "error before the robot moved" was Umeyama tail-drag (2026-08-22)
- **Symptom:** run2b's error-vs-time curve started at 12.3 m while the robot was still stationary.
- **Cause:** least-squares alignment over a diverging trajectory drags the fit away from the start to appease the tail (Zhang & Scaramuzza, IROS 2018).
- **Fix:** `--align-window N` fits the alignment on the first N seconds only (errors still measured over the whole run; policy printed in the report). Measured: error at t=0 went **12.371 m → 0.001 m** with a 60 s window.
- **Verify:** `docs/PHASE0_RESULTS.md` Finding-8 table.

### Reset gaps quantified: 13 resets deleted ~9 m of path from run2b (2026-08-22)
- **Symptom:** Finding 1 said reset gaps omit travelled distance from the pose graph; nobody knew how much.
- **Fix (analysis, reusable):** per-reset GT displacement from the 1 Hz monitor CSV — 13 resets, median **0.875 m** per gap, **8.96 m** summed; the robot was moving near full speed through almost every outage.
- **Rule:** no ATE without resets/100 m and summed gap displacement beside it (ENGINEERING_NOTES.md section 2.4).
- **Verify:** `rosrun sidewalk_evaluation reset_gaps.py --run-id <run_id>` (repo-ified from the original /tmp throwaway on 2026-08-22; adds resets/100m and --json).

### Phase-1.1 "5 m depth NaN clamp" — already fixed in this repo (2026-08-22)
- **Symptom (anticipated):** `pointCloudCutoffMax` defaults to 5.0 m in `libgazebo_ros_openni_kinect.so` and NaNs every depth pixel beyond 5 m.
- **Cause:** plugin default — but this repo already overrides it.
- **Fix:** already present: `<pointCloudCutoff>1.0</pointCloudCutoff>` / `<pointCloudCutoffMax>35.0</pointCloudCutoffMax>` (`catkin_ws/src/sidewalk_bringup/urdf/zedx_mount.urdf.xacro:176-177`, both machines), matching the sensor `<clip>` 1.0/35.0. Live measurement (2026-08-21, mcity preflight): finite depth to 34.28 m, 72.1% finite pixels.
- **Verify:** `grep -n pointCloudCutoffMax catkin_ws/src/sidewalk_bringup/urdf/zedx_mount.urdf.xacro`

### Ground truth already recorded at ~136 Hz — the `--rate 20` was a no-op (2026-08-22)
- **Symptom:** Phase 0.2 demanded GT "at ≥ 50 Hz"; launch file said `--rate 20`.
- **Cause:** `trajectory_recorder.py` `record_from_topic` is callback-driven and never reads `--rate`; GT records at the topic's own rate. `/ground_truth/odom` is a passthrough of `/gazebo/model_states` (~136 Hz measured).
- **Fix:** none needed for rate. The dead `--rate` was removed from the truth recorder in `record_mapping_run.launch` with a comment. Real defects fixed instead: duplicate stamps (see next entry) and per-run monitor CSV archiving.
- **Verify:** run2b meta: 109,874 poses / 807 s ≈ 136 Hz; new runs: `mean_rate_hz` field in `ground_truth_trajectory.tum.meta.json`.

### "24941 non-increasing timestamp(s)" warning on the reference (2026-08-22)
- **Symptom:** evaluate_trajectory warned the GT trajectory had ~25k duplicate/out-of-order stamps.
- **Cause:** `ground_truth_odom.py` stamps with `rospy.Time.now()` inside a ~1 kHz `model_states` callback while the sim clock ticks ~100 Hz → many identical stamps.
- **Fix:** monotonic-stamp guard in the publisher callback (skip publish when `now <= last_stamp`) — heals every consumer (recorder, monitor, rosbag) at the source. Rate stays ≥ 50 Hz (sim-clock resolution).
- **Verify:** next recorded run's evaluation prints zero non-increasing-timestamp warnings on the reference.

### Covariance clamp confirmed live (2026-08-22)
- **Symptom:** loop closures always rejected in sim (224/224 office run #4; 301/301 mcity run2b).
- **Cause:** `Registration.cpp:36` clamps VO covariance at σ = exactly 1e-4 m (ENGINEERING_NOTES.md section 2.3). Confirmed on this system: **422 of 7298** odometry frames in the run2b log carry `std dev=0.000100m` — pinned to the clamp, not computed.
- **Fix (rule):** fix the sensor noise (Phase 1.3 two-plain-camera rebuild), never tune the back-end around it. Never write exactly 1.0 into a republished covariance.
- **Verify:** `grep -c "std dev=0.000100m" ~/.rtabmap_mcity_v2_run2b.log` (Jetson) → 422.

### Run-#5 backend-exception greps: zero hits — with a logging caveat (2026-08-22)
- **Symptom:** Part B Finding 3 proposed grepping run-#5 logs for `IndeterminantLinearSystemException` / `very huge` / `diverging`.
- **Result:** 0 hits for all three in both `~/.rtabmap_vertigo*.log` — but those logs contain **zero WARN/ERROR lines at all**, so they captured stdout only and the grep is inconclusive about the backend.
- **Fix (rule):** every future run's log redirect must be `> file 2>&1` so stderr (where ULogger warnings go) is captured.
- **Verify:** `grep -c WARN <new run log>` returns non-zero on any run that prints warnings.

### `"cannot be used"` in logs is the benign detector-fallback notice (2026-08-22)
- **Symptom:** ENGINEERING_NOTES.md section 2.6 says grep every run log for `"cannot be used"` and mark the trial invalid.
- **Result:** appears 14× in 9 office logs, always as `BRIEF, FREAK and DAISY features cannot be used because OpenCV was not built with xfeatures2d module. GFTT/ORB is used instead.` — the known build limitation; the configured detector IS 8 (GFTT/ORB), so nothing silently changed.
- **Rule:** this exact message does not invalidate a trial; any *other* "cannot be used" message does.
- **Verify:** `grep -h "cannot be used" ~/.rtabmap_*.log | sort -u` (Jetson) → one distinct message.

### median + p95 were already computed for every metric (2026-08-22)
- **Symptom:** Phase 0.4 asked for median + 95th on every metric row.
- **Cause:** `_stats()` (`traj_metrics.py:562-578`) already returns n/rmse/mean/median/std/min/max/p95 for every error series; the ATE report row already showed all three.
- **Fix:** surfacing only — RPE distribution table, corrections median/p95, per-segment drift table added to `generate_report.py`; `ate_p95_m` added to headline/metrics.
- **Verify:** any new `report.html` shows median and 95th columns on RPE and corrections tables.

### Recording smoke test PASSED, after finding two of my own sync bugs (2026-08-22)
- **Result:** all three Phase-0 recording checks confirmed on a live 20 s Gazebo session — 0/2464 non-increasing GT timestamps, `meta.json` carries `reference_kind: simulator_ground_truth` + `mean_rate_hz: 117.87`, monitor CSV lands in its per-run directory.
- **Bug 1 found on the way — a fix committed earlier in the day was never actually on the Jetson.** The queue_size 30→5 memory-safety fix (`rtabmap_mapping.launch`, `rtabmap_common.launch`) was edited and verified locally hours earlier, but those two files were never included in that session's `scp` batch — only `record_mapping_run.launch` was. The Jetson ran `queue_size=30` all day, including through the `mcity_v2_run2b` recording used for every Phase-0 diagnostic. It never crashed, so the gap stayed invisible until this smoke test's roslaunch threw `RLException: unused args [queue_size]` (the synced parent now passed an arg the stale child didn't declare).
  - **Rule:** "I edited and verified it" is not "it is on the Jetson." After any multi-file edit, `grep` the specific changed line on the remote machine — don't trust that it was in an earlier sync batch.
- **Bug 2 — `scp` had stripped the executable bit on 2 of 8 synced Python scripts** (`ground_truth_odom.py`, `ingest_navigation_run.py`) earlier that day, and this failed **completely silently**: no error banner, no "process died" line in the roslaunch log (whose interleaved multi-process output is unreliable to grep, per the run-#5 log entry above) — the node just never appeared in `rosnode list`. Fixed with `chmod +x` on both Jetson copies and the source copy.
  - **Rule:** after syncing any script a `<node>` tag invokes directly, verify `ls -la` shows `x` AND cross-check with `rosnode list` on the live system — do not trust grep against this project's log files to prove a node started.

## Infrastructure & machines

### Jetson "crash" that was a memory livelock — RViz 11 GB (2026-08-21)
- **Symptom:** Jetson vanished from the network ~1 min after starting `record_mapping_run.launch`; unreachable 29 min; looked like a crash/power loss.
- **Cause:** RViz held 11.14 GB (unbounded MapCloud + duplicate full-res Camera subscription), rtabmap 5.18 GB; machine thrashed 15 GB zram so hard the network service couldn't send keepalives. `uptime` proved **no reboot** — livelock, not crash.
- **Fix:** recorded drives run `rviz:=false`; `queue_size` 30→5 in the launch chain (≈2.9 GB → ≈0.48 GB of buffered frames, and stale-frame processing eliminated); earlyoom installed (`-m 15 -s 15`, avoids sshd/Xorg/x11vnc and the network service, prefers rviz/gz*/rtabmap/rgbd_*).
- **Verify:** `systemctl is-active earlyoom` → active; `journalctl -u earlyoom -n 5` shows the avoid/prefer regexes; `grep -n 'queue_size' catkin_ws/src/sidewalk_slam/launch/rtabmap_common.launch` → default 5.
- **Diagnostic rule reusable anywhere:** when a remote machine "crashes", check `uptime` first — uptime exceeding the outage means livelock, cause still resident, and `journalctl -b -1` is the wrong place to look.

### Mapping run silently produced nothing — missing dated folder (2026-08-22)
- **Symptom:** run started clean, odometry log healthy, but no map was ever built; monitor STATUS stale.
- **Cause:** `database_path` pointed into `rtabmap_maps/2026-08-22/` which didn't exist; RTAB-Map's SQLite driver does not create parent directories → FATAL at startup, node stayed alive but processed nothing. Odometry (front end) kept logging healthily, masking it.
- **Fix:** `mkdir -p` the dated folder before launch; check the rtabmap log for `FATAL` within the first minute of any run.
- **Verify:** `grep -i fatal <run log>` right after launch.

### Fake 1 TB microSD ("asdfg") (2026-08-21)
- **Symptom:** 1 TB card threw I/O errors, dropped off the bus mid-read, FAT-fs corruption.
- **Cause:** counterfeit flash — controller reports product name `asdfg` and a false 1 TB capacity; writes past real size wrap and destroy data. A reformat cannot fix firmware lies.
- **Fix:** card retired; 1.29 of 1.81 GB rescued to `backup/sdcard-rescue-20260821/` (research folders on it were empty). Replacement guidance: external USB SSD (Samsung T7 / SanDisk Extreme / Crucial X9) over microSD for SQLite workloads; verify any new card with f3 before trust.
- **Verify:** `journalctl -k | grep mmc` on the computer it was plugged into showed `mmcblk0: mmc0:0001 asdfg 1000 GiB`.

### The Jetson's project folder is a plain file mirror, not a git clone — committed fixes can silently never arrive (2026-08-23)
- **Symptom:** RViz opened during the office-room-1 exploration drive with `Global: error`, `RobotModel: error`, `Camera: warn`, `CameraPose (TF): warn`. The fix looked already done — `rtabmap_common.launch` pointing at `slam_live_safe.rviz` instead of the older `slam.rviz` had been committed to git hours earlier (`f896f99`, part of a batch of six hygiene fixes) and this session's own summary of prior work said it was complete.
- **Cause:** `<repo>` on the Jetson has no `.git` directory at all — it is a plain copy of files, kept in sync only by whoever last ran `scp`/`rsync` into it. The six-file commit was made in a separate copy of the repository on another computer; nothing ever pushed those six files to the Jetson afterward. `git log` on that computer and "the fix is committed" said nothing true about what the Jetson would actually run. This is the same bug class as the `route_from_drive.py` Python-3.8 incompatibility found earlier this session (2026-08-23, above) — a second, independent instance of "committed ≠ deployed" in the same day.
- **Fix:** `rsync -avnc` (checksum-based, not mtime-based — mtimes differ across machines for meaningless reasons) scoped to `catkin_ws/src/`, excluding `__pycache__`, found exactly 10 stale files (all from the same commit). Pushed each to its exact destination path individually, re-ran the same checksum diff, confirmed empty output.
- **Verify / reusable check before trusting Jetson behavior:**
  ```bash
  rsync -avnc --exclude='__pycache__' --exclude='*.pyc' \
    catkin_ws/src/ "<jetson>:<repo>/catkin_ws/src/"
  ```
  Empty output = nothing stale. Any lines printed are files whose *content* differs (not just timestamp) — the Jetson will behave differently from what `git log` on the other computer implies until they're pushed.
- **Rule this generalizes:** on this project, "committed" and "deployed" are two different states and only the second one matters once you are standing in front of the actual robot or sim. Run the checksum diff above at the start of any hands-on session, not just when a specific bug is suspected — a silent gap doesn't announce itself, it just makes the next unrelated symptom (here, an RViz panel) look like a fresh bug instead of a sync problem.

### `MemoryLimit` on this Jetson is accepted but NOT enforced (2026-08-21)
- **Symptom:** wanted to reserve memory for SSH; systemd accepted `-p MemoryLimit=300M`.
- **Cause:** cgroup v1 + 15 GB zram: the limit caps RSS but excess pages swap; systemd 245 has no systemd-oomd; `MemoryMin/Low` need cgroup v2. Proven by test: 900 MB allocated and touched inside a 300 MB scope, survived.
- **Fix:** earlyoom with `--avoid`/`--prefer` (see livelock entry) — watches RAM *and* swap.
- **Verify:** the enforcement test above; `stat -fc %T /sys/fs/cgroup` → `tmpfs` (v1).

## Simulation & worlds

### office2.world: 1.8 m doors, not 1.0 m — the brief's own numbers conflicted (2026-08-22)
- **Symptom:** built office2.world (24×18 m, 15-room office building) with every door at the brief's stated 1.00 m clear width. Live in Gazebo the user found doors visibly too tight.
- **Cause:** the brief also separately states 2.0–2.5 m robot clearance "at every corridor/doorway." Checked `sidewalk_navigation/config/costmap_common.yaml` + `README.md`: Husky footprint 0.99×0.67 m, circumscribed radius 0.598 m → it sweeps a **1.196 m diameter circle just turning in place**, already wider than a 1.00 m door. `README.md` flags this footprint as `measured: false` — treat 1.196 m as a floor, not a target.
- **Fix:** every door widened to 1.80 m (~50% margin over the sweep circle, still visibly narrower than the 2.0–2.5 m corridors it opens onto). 3 doors in smaller utility rooms re-centred on their own room's midpoint (not the old 1.0 m centre) to avoid a near-zero wall sliver at one end of an asymmetric room.
- **Verify:** `grep -n "DOOR WIDTH, revised" catkin_ws/src/sidewalk_sim/worlds/office2.world`; re-run the overlap/door/perimeter validator (see the doorway-passage entry below for why point-geometry alone isn't sufficient proof).

### Teleporting a robot into Gazebo does not test whether it can drive through an opening (2026-08-22)
- **Symptom:** needed to prove the 1.8 m office2.world doors were actually passable, not just geometrically open.
- **Cause:** `/gazebo/set_model_state` teleport ignores collision on the way — it can only prove a point is unoccupied at rest, never that a robot's real footprint can sweep through a gap without clipping something.
- **Fix:** drive the robot through with real `/cmd_vel` commands (fixed linear.x for N seconds) and read `/gazebo/get_model_state` before/after (roll/pitch computed from quaternion, position vs. expected end-point) to catch a collision-ejection or tip. A per-second trajectory poll (position+roll+pitch every 1s during the drive, not just before/after) is what actually localizes *where* a stall happens — before/after alone just says pass/fail with no clue why.
- **Verify:** `~/.run_records/doorway_test.py <label> <x> <y> <yaw> <speed> <duration> <expect_x> <expect_y> <tol>` on the Jetson; PASS requires end position within tolerance of the commanded straight-line endpoint AND roll/pitch under ~17°.

### A "clear" test coordinate near furniture still isn't clear of the robot's own body (2026-08-22)
- **Symptom:** two doorway-passage tests failed on the first (and second) attempt — the robot got shoved/tipped by physics almost immediately after teleport or drove only ~1m of a commanded ~5m path before stalling and pitching up to -50°.
- **Cause:** picked test coordinates checked only as a *point* against furniture bounding boxes (e.g. 0.2–0.5 m from a desk/chair/wall edge). The Husky's own footprint (~0.5–0.6 m half-length/half-width, per the unmeasured-footprint figure above) needs real clearance from that point, not zero. One case: a straight-line path ran exactly along a desk cluster's boundary edge (0.5 m nominal clearance) and the robot rode up onto the desk face at that exact x-range, confirmed by the per-second trajectory log. A second case: a start point only 0.5–0.8 m from a wall let the robot's own body clip the wall during the post-teleport settle, before driving even began.
- **Fix:** require ≥0.8–1.0 m clearance from any prop/wall edge for every point along a planned test path (not just start/end), verified by parametrizing the straight-line path and checking each obstacle's bounding box against the full traversed interval — a point 0.5 m from an edge is not automatically safe.
- **Verify:** re-ran with paths re-routed around the whole furniture cluster (not just the original "clear" point) — clean pass, position within 0.22 m of the commanded endpoint, zero tip.

### Mcity mesh scale was wrong by exactly 2× (2026-08-21)
- **Symptom:** kerbs 0.316 m tall (1.9× the Husky's 0.1651 m wheel radius — unmountable walls); town 584×925 m vs real Mcity ~360 m.
- **Cause:** original model.sdf used 0.254 (inch→m) on a mesh not authored in inches.
- **Fix:** scale 0.127 in `models/mcity/model.sdf` (collision AND visual); kerb 0.158 m (US 6-inch standard); every spawn/prop coordinate recomputed and probed against mesh faces (people on `Concrete_Ground` z=0.158, cars on `Road_*` z=0.0 — note the easily-missed `Road_*_Light` material variants cover most streets near spawn).
- **Verify:** `grep -m1 scale catkin_ws/src/sidewalk_sim/models/mcity/model.sdf` → 0.127; robot settles at z = 0.290 = 0.158 + 0.13228.

### Feature-count gate before every drive (2026-08-20, re-validated at new scale 2026-08-21)
- **Symptom:** flat-colour walls → 13 ORB keypoints in 1920×1200 → odometry arithmetic-impossible (needs 20 inliers).
- **Fix:** texture rules in `docs/OPERATIONS.md`; measured 964 decimated / 1000-cap native at the mcity spawn after rescale.
- **Rule:** never start a recorded drive without the ORB count; under ~50 = stop. Use a draining subscriber (discard ≥10 frames), never `wait_for_message` (returned byte-identical stale frames at 6 different locations once).

## Process traps

### `rsync` with several individual source files and one destination directory silently flattens their paths (2026-08-23)
- **Symptom:** pushed 10 stale files to the Jetson (see "plain file mirror" entry above) with `rsync -avc file1 file2 ... "<jetson>:/dest/dir/"`, got a clean "sent N bytes" success line, but re-running the checksum diff immediately after showed the exact same 10 files still stale — as if nothing had happened.
- **Cause:** without `-R`/`--relative`, `rsync` drops every source argument's leading directories and writes it directly into the destination directory by basename only. All 10 files (from six different subdirectories) landed as loose files directly in `catkin_ws/src/` — e.g. `catkin_ws/src/sidewalk_slam/launch/rtabmap_common.launch` was never touched; `catkin_ws/src/rtabmap_common.launch` was created instead. The real target files were untouched, so the diff correctly still showed them stale.
- **Fix:** `scp` each file individually with its full, explicit destination path — no ambiguity about where it lands. (`rsync -R` from a common base directory would also work, but wasn't used here.)
- **Verify:** re-run the same checksum diff (`rsync -avnc ...`) after any multi-file push, not just after a single-file one — a "sent" confirmation only proves bytes moved somewhere, not that they moved to the right place.
- **Note:** the 10 misplaced stray files are harmless clutter (loose files directly under `catkin_ws/src/`, outside any package directory — nothing builds or launches from them) but still needed deleting, which is the user's action per this project's own rule, not something to route around.

### SSH X11 forwarding silently wins over the shared VNC display — a GUI launch "succeeds" on the wrong screen (2026-08-23)
- **Symptom:** `roslaunch sidewalk_sim husky_real_gazebo.launch` (typed in a normal `ssh <jetson>` session) reported success and the process stayed alive and healthy, but nothing appeared in the VNC window — the user connected and saw an empty desktop.
- **Cause:** the connecting computer's `~/.ssh/config` had `ForwardX11 yes` for the Jetson (added so VS Code's integrated terminal would forward too), so *any* plain interactive SSH login already has a working `DISPLAY` pointing back at that computer, no `-X` flag needed. `gzclient` opened there instead of on the Jetson's own display `:1` — confirmed via `cat /proc/<gzclient_pid>/environ | grep DISPLAY` → `DISPLAY=localhost:11.0`, an SSH-tunneled forward, not the Jetson's real X server. This is the general trap already documented in `docs/OPERATIONS.md` ("Remote GUI access to the Jetson"); this is its first observed live occurrence logged with a dated root-cause trace.
- **Fix:** stop the misdisplayed process (`kill -INT <roslaunch pid>`, found via `pstree -p -s <gzserver pid>` — not the gzserver/gzclient children directly), then relaunch either (a) from a terminal opened *inside* the VNC window itself (simplest — no forwarding involved, a native child of that session), or (b) from an SSH session with `DISPLAY`/`XAUTHORITY` explicitly exported first, overriding the SSH-forwarded default. New helper for (b): `tools/vnc_display_env.sh` — `source` it once per login to re-derive and export both correctly without hand-parsing `ps`/`/tmp/.X11-unix/` output every time (it distinguishes sidewalk's session from gdm's greeter by the `-auth` path, since Xorg itself always runs as root regardless of whose session it serves — see the script's own header for why process UID doesn't work).
- **Verify:** `cat /proc/<pid>/environ | tr '\0' '\n' | grep DISPLAY` on any GUI process should read `:1` (or whatever `tools/vnc_display_env.sh` reports), never `localhost:<port>`.

### `pkill -f <pattern>` self-matches the SSH command line (recurring, ~4×)
- **Fix:** capture PIDs first (`ps -eo pid,comm --no-headers | awk '$2=="roslaunch"{print $1}'`), then `kill` by number only. Never `pkill -f` over SSH.

### Stale ROS parameters survive node death
- See `docs/OPERATIONS.md` — check `rosparam list`, delete under BOTH `/rtabmap/rtabmap/` and `/rtabmap/rgbd_odometry/`, then relaunch.

### `rosout` zombies accumulate forever — one per past roslaunch session, never swept (2026-08-23)
- **Symptom:** ~50 leftover `/opt/ros/noetic/lib/rosout/rosout` processes found on the Jetson (oldest from 2026-08-19, ~482 MB combined RSS), one per past `roslaunch` session going back days.
- **Cause:** `sweep()` in `tools/p1_run.sh` SIGINTs roslaunch processes, then greps stragglers on `comm` for `roslaunch|gzserver|gzclient|rtabmap|stereo_|rgbd_|robot_state_pub|spawner|cmd_vel_relay|trajectory_reco` to SIGKILL — `rosout` was never in that pattern, so each session's rosout node survives its parent roslaunch's death (reparented to init) and accumulates forever. Same class of bug as the `map_saver` zombies above (`comm` not caught by the roslaunch/gzserver pattern).
- **Fix:** added `rosout` to the `sweep()` kill pattern in `tools/p1_run.sh` (both copies). `rosmaster` checked too — not similarly affected (only the persistent real-camera master on 11312 plus one fresh sim-side master on 11311 were ever found running; `sweep()` already kills the sim master explicitly by port).
- **Verification caveat:** `pgrep -c -f "lib/rosout/rosout"` self-matches its own command line (the exact `pkill -f` self-match trap below) and overcounted by one. Use `ps -eo pid,comm | awk '$2=="rosout"'` instead. Cleaned up live with a campaign run (`p1_cpu_profile_r1`) in progress — the one rosout belonging to the active session was identified by parent PID and excluded from the kill; the run's full node graph was confirmed intact afterward.
- **Verify:** `ps -eo pid,comm --no-headers | awk '$2=="rosout"' | wc -l` before/after a sweep — count dropped from 50 orphans (+1 live) to 1 (the live session only).

### All 16 Phase-1 timelapse videos were frozen — `/rtabmap/grid_map` needs a standing subscriber (2026-08-22)
- **Symptom:** every one of the 16 Phase-1 campaign timelapses (200–260 `.pgm` snapshots each) was byte-identical from `frame_000000` to the last frame — a near-empty grid frozen for the entire multi-minute run. Looked like a real mapping failure; the user correctly rejected the videos on sight ("failed clearly from the recorded videos").
- **Cause:** `cmd_timelapse` (`map_manager.py`) spawned a fresh `rosrun map_server map_saver` subprocess every interval — each one connects, grabs one message, disconnects within ~1s. Measured live (`grid_watch.py`, a persistent subscriber run alongside): with only that connect/grab/disconnect cycle touching `/rtabmap/grid_map`, RTAB-Map's grid publisher went silent after its first, near-empty publish for the rest of the run. Re-running the identical config with one added **persistent** subscriber alive the whole time produced real growth (26/54 unique frames, 177 KB→272 KB). RTAB-Map's own 3D pose graph and point cloud were never affected — `rtabmap-export` reconstructed real, complete maps from every database the whole time (confirmed via SE(2)-aligned point-cloud plots against ground truth). The bug was in this recorder, not in the SLAM itself.
- **Fix:** `cmd_timelapse` now opens one `rospy.Subscriber` on the target topic for the whole recording (do-nothing callback, just counts publishes) alongside the existing per-interval `map_saver` calls, and logs/reports that publish count so a future silent topic is self-diagnosing instead of silently producing a frozen video. Verified on the real 5-lap `mcity_loop.yaml` route (the exact route + in-place turns present in every frozen run): 168 frames, 154 unique, 272 real grid publishes observed.
- **Verify:** `md5sum <timelapse dir>/*.pgm | awk '{print $1}' | sort -u | wc -l` should be close to the frame count, not `1`. `grep "topic has published" <run>/timelapse.log` — the count should climb throughout the run, not stay at 0–1.
- **Related bug found while fixing this:** eight `map_saver` processes from five different old campaign runs (`p1_A_r2`, `p1_A_r5`, `p1_AY_r1`, `p1_B0_r1/r2/r3`, `p1_B1_r3`, `p1_B1_r5`) were still alive on the Jetson, busy-spinning at 82–86% CPU, for **5–7.5 hours each** — they never got a message, never respected `map_saver`'s or `run_cmd`'s timeout, and were never matched by `p1_run.sh`'s `sweep()` (which lists `roslaunch|gzserver|...` by `comm` and never included `map_saver`). Killed by hand; `sweep()` in `tools/p1_run.sh` now also kills any process with `comm == map_saver`. Check `ps -eo pid,etime,pcpu,comm | grep map_saver` after any future campaign — this can silently eat most of the Jetson's cores for hours between sessions.

### "0/1144 loop closures accepted" was measured wrong — the log never records acceptance (2026-08-22)
- **Symptom:** every Phase-1 database, in every arm, showed 0 loop closures accepted per the campaign's own log-grep checklist (`grep -c "Rejecting all added loop closures"` counted rejections; nothing counted acceptances). Treated as fact and written into `PHASE1_RESULTS.md` and reported to the user as "the sharpest open question."
- **Cause:** RTAB-Map's own console logger only emits `[ WARN]`/`[ERROR]` into the captured `rtabmap.log` — its acceptance announcements log at `[ INFO]` (RTAB-Map's own info level, not ROS's), which never reached the file, while rejections (`Loop closure X->Y rejected!`, `Rejecting all added loop closures...`) are WARN and always did. A log that structurally can only show rejections was read as proof there were zero acceptances.
- **Fix:** read the saved graph directly instead — `rtabmap-info <db>` (already wrapped by `map_manager.py info`) prints `Global graph: N poses and M links` broken down by `GlobalClosure`/`LocalSpaceClosure`/`LocalTimeClosure` counts and average/max length. Checked `p1_B1_r3.db` (one of the "0 accepted" campaign runs): **39 GlobalClosure + 39 LocalSpaceClosure**, both with honest sub-meter average lengths. The closures were there the whole time.
- **Verify:** never trust a log-grep count of "accepted" events for this build — always cross-check `rtabmap-info`'s own `Global graph:` line against the database.
- **Rule this generalizes:** a metric that is structurally one-sided (can only decrement, never increment, or vice versa) will look like proof of the missing direction even when it says nothing about it. Cross-check against a second, structurally different source before reporting a "zero" as a finding.

### Wheel-slip-against-a-kerb: distance-based route completion can't detect being stuck (2026-08-22)
- **Symptom:** an exploratory building-perimeter route (`mcity_building_loop.yaml`, never part of the validated campaign) repeatedly froze at one spot for ~150s at a time, with a burst of ~500-700 tracking resets each time, before `route_player.py`'s own accounting reported the segment "complete" as if nothing had happened.
- **Cause:** `route_player.py`'s `forward()` completion condition is wheel-odometry arc length (`travelled >= dist_m`). When a wheel pushes against a kerb it can't climb, the wheel keeps turning — the encoder keeps accumulating "distance" — while the chassis doesn't actually move. The completion condition was satisfied by wheel rotation that produced zero real displacement. Confirmed live: segment timing matched the *ideal* duration for the *planned* distance almost exactly, while ground truth showed near-zero net movement over the same window.
- **Fix:** added an opt-in `--recover-on-stuck` mode (`route_player.py`, off by default — every validated campaign route is untouched). It watches `/ground_truth/odom` (simulator-only, disclosed as such — a real robot needs a bump/current-spike/IMU-jerk signal instead) purely as a stall trigger: real displacement far below what commanded speed implies over a ~2.5s window means wheel slip, not progress. Recovery is stop → back up 3s → rotate to break contact (sweeping wider on repeated stalls at the same spot) → **re-aim at the original absolute target** (not just keep driving straight in whatever heading the recovery turn left it at — the first version of this did that and sailed the robot off toward a roundabout, well outside the route, before an external safety monitor caught it).
- **Verify:** compare wheel-odom "distance travelled" against ground-truth displacement over any stall window — if wheel-odom keeps counting while ground truth doesn't move, it's slip, not progress, no matter how plausible the elapsed time looks.
- **Still open, not solved by the recovery logic:** one specific curb/median on this exploratory route could not be reliably passed even by a full 360° sweep of recovery attempts (14 attempts, every 45° increment, still stuck) — recovery gets the robot unstuck from *a* stall, but doesn't make an uncrossable kerb crossable. That specific route is not validated and not part of any gate.

### `RGBD/ProximityMaxGraphDepth` unlimited lets proximity closures compare against the most-drifted node available (2026-08-22)
- **Symptom:** even with honest, unpinned covariance (B1), essentially every loop-closure candidate was rejected at the post-optimization graph-error-ratio stage (`RGBD/OptimizeMaxError=3.0`) — the needed correction was consistently 3-9x the camera's own honest confidence.
- **Cause:** `RGBD/ProximityMaxGraphDepth` was `"0"` (unlimited) — proximity search could match the current pose against a node from 4 laps / 70+ m ago, which has accumulated much more drift than a same-viewpoint node from one lap ago, producing a bigger, less honest-confidence-compatible correction.
- **Fix:** capped at `120` (~2 laps, calibrated from `rtabmap-info`'s own node count on a real B1 database: 274 nodes / 5 laps ≈ 55 nodes/lap) in `rtabmap_zedx.yaml`. Does not touch `RGBD/OptimizeMaxError` or `Optimizer/Robust` (both remain off-limits per ENGINEERING_NOTES.md section 4.7). Live-verified twice: first a short live run (26/54 timelapse frames genuinely changed, confirming the mechanism), then a full extended session — **164 closures accepted** (119 global + 45 local-space), first confirmed acceptances in this project's history.
- **Verify:** `rtabmap-info <db>` → `Global graph:` line shows nonzero `GlobalClosure`/`LocalSpaceClosure` counts with sub-meter average lengths.
- ⚠ **Source database deleted 2026-08-24** during a space-reclaim sweep (`2026-08-22_lc/p1_lc_building_run4_manual.db`). The 164-closure figure stands as documented here but **can no longer be re-derived from raw data** — cite it as a recorded result, not a reproducible one. Same applies to office run #5's 951 closures (`office_vertigo_1739.db`, both copies gone) and the 154/168 timelapse-fix proof (`p1_fix_verify`). Office run #4's 690 closures are unaffected — `office_run4.db` survives in `2026-08-20/`. Full audit in the project records.

### The microSD "card failure" was the FILESYSTEM, not the flash — reformatted exFAT → ext4 (2026-08-25)
- **Symptom, over two days:** five files corrupted on the 128 GB microSD card — `lab_map_02.db` killed mid-write, `office1_explore_04.db`, `office1_explore_03.db` and `lab_map_01.bag` rotted **at rest** (hours later, no writes in between), and finally `lab_map_04.db` failed `PRAGMA quick_check` roughly ten hours after passing it cleanly. Copies between the card and the USB drive ran at **1.6 MB/s**. The card was treated as failing hardware for two days.
- **The evidence that pointed elsewhere:** `f3` (the standard counterfeit-card detector) wrote 36 GiB and read **24 GiB back bit-perfect** — `2097152/0/0/0` sectors ok/corrupted/changed/overwritten on every file checked. A single-file test reached **13.09 GiB**. The raw flash was provably healthy, and a raw `dd` read of the card hit **33 MB/s** while a `cp` between two mounted volumes crawled at 1.6 MB/s.
- **Cause:** both removable volumes were **exFAT mounted through FUSE** (a userspace filesystem driver — `lsblk` shows `fuseblk`, and `/sbin/mount.exfat` runs as an ordinary process). exFAT has **no journal**, so an interrupted or misordered write leaves the metadata inconsistent with no way to recover; and routing every read and write through a userspace process both throttles throughput and widens the window in which that inconsistency can occur. The common factor in all five losses and both speed problems was the filesystem stack, not the flash.
- **Fix (user-authorised 2026-08-25, explicitly overriding the never-delete rule):** card reformatted to **ext4** — journaled, kernel-native driver, no userspace layer:
  ```bash
  sudo umount /media/sidewalk/SIDEWALK128
  sudo mkfs.ext4 -F -L SIDEWALK128 -m 0 /dev/mmcblk1p1     # -m 0 reclaims the ~6 GB root reserve
  # then a UUID fstab entry, because exFAT auto-mounted and ext4 will not:
  # UUID=<uuid>  /media/sidewalk/SIDEWALK128  ext4  defaults,noatime,nofail  0  2
  sudo chown -R sidewalk:sidewalk /media/sidewalk/SIDEWALK128
  ```
- **Measured improvement, same hardware, minutes apart:** write **32.6 → 61.7 MB/s** (1.9×), read **33.4 → 87.7 MB/s** (2.6×), usable capacity 117 GB.
- **Two ext4 differences that will bite if forgotten:** (1) exFAT auto-mounted via udisks; **ext4 needs the fstab entry** or the card is simply absent after a reboot (`nofail` is included so the Jetson still boots without it). (2) exFAT presented every file as mode 777 with no real ownership; **ext4 enforces permissions**, so the mount and its directories must be `chown`ed to `sidewalk` or recordings fail with permission errors mid-run.
- **Guard used when formatting, and worth reusing:** the script refused to run unless `findmnt` confirmed the target was the card *and* that it was not the root device — on this Jetson the card is `/dev/mmcblk1p1` and the operating system is `/dev/mmcblk0p1`, one character apart.
- **Verify:** `lsblk -o NAME,FSTYPE,MOUNTPOINT` shows `ext4`, not `exfat`/`fuseblk`. If corruption recurs on ext4, the flash becomes the suspect again — but it is no longer the leading one.

### The ZED X will not open over SSH because X forwarding hijacks `DISPLAY` (2026-08-25)
- **Symptom:** `roslaunch sidewalk_perception zedx_front.launch` failed repeatedly at the start of real indoor run 2, retrying forever with `CAMERA FAILED TO SETUP`. Two *different* errors appeared, minutes apart, which is what made it confusing:
  1. `(Argus) Error InvalidState: Receive thread is not running cannot send` — plus `nvargus-daemon` showing a restart timestamp 7 seconds *before* the failure.
  2. After fixing that, a completely different one: `libEGL warning: DRI3: failed to query the version` followed by `(Argus) Error BadParameter ... FrameConsumerImpl.cpp, function create(), line 44`.
- **Two independent causes, and the first one masked the second:**
  1. `nvargus-daemon` (the Jetson's camera service that Argus talks to) had genuinely crashed. Fixed by `sudo systemctl restart nvargus-daemon` — and `zed_x_daemon` alongside it for the GMSL2 link.
  2. **The real, repeatable one:** an `~/.ssh/config` with `ForwardX11 yes` makes **every** SSH session to the Jetson inherits `DISPLAY=localhost:<N>.0` — measured live as `localhost:14.0`. That display is tunnelled back to the connecting computer and has **no GPU behind it**. The ZED SDK builds its video stream as an EGL frame consumer on the GPU, so creating it against a forwarded display fails — hence `FrameConsumerImpl` + the `libEGL DRI3` warning. Nothing about the camera, the cable, or the SDK was wrong.
- **Fix — connect with X forwarding OFF and point at the Jetson's own screen:**
  ```bash
  ssh -o ForwardX11=no <jetson>
  export DISPLAY=:0
  export XAUTHORITY=/run/user/1000/gdm/Xauthority   # from Xorg's own -auth argument
  roslaunch sidewalk_perception zedx_front.launch
  ```
  Verified working immediately: `ZED connection [LIVE CAMERA with ID 0]: SUCCESS`, `Serial Number: 47637666`, and `/zedx_front/zed_node/rgb/image_rect_color` at **14.75 Hz**.
- **Derive, do not assume, the two values:** `XAUTHORITY` is the `-auth <path>` argument of the running Xorg process (`ps aux | grep "[X]org"`); the display number is the socket in `/tmp/.X11-unix/` (here the only one is `X0`, so `:0`). `tools/vnc_display_env.sh` does this derivation and exports both.
- **The opposite rule applies to RViz.** RViz *must* be run with plain `ssh -X` and no `DISPLAY` override, or its window opens on the Jetson's invisible screen instead of the user's PC. Camera → local display, X forwarding off. RViz → forwarded display, X forwarding on. Different terminals, opposite settings; this is the whole trap in one line.
- **Verify:** `echo $DISPLAY` in the camera's shell must read `:0`, never `localhost:<N>.0`.

### `rtabmap_util/data_player`'s `rate` param is NOT rosbag play's `--rate` — it paces by stored-node throughput, not original elapsed time (2026-08-24)
- **Symptom:** built `slam_from_database.launch` (data_player + rtabmap_common, for reconstructing a timelapse video from `office1_explore_03.db` after the original camera/sim run was gone) assuming `rate:=1.0` meant "real time," matching `rosbag play --rate=1.0`. Launched it expecting the known ~78-minute original drive duration; it silently finished (`data_player` exited cleanly, `required="true"` tore down the whole roslaunch) in well under 5 minutes. A `map_manager.py timelapse` sampler running alongside kept retrying against the now-dead `/rtabmap/grid_map` topic for the rest of its `--duration`, producing nothing but repeated timeouts.
- **Cause:** `data_player` replays the database's **stored nodes**, not the camera's original frame stream. RTAB-Map's own memory management keeps far fewer nodes than raw captured frames (near-duplicate viewpoints are discarded) — this database held only ~276 kept nodes from a ~78-minute, ~2579-image drive. `rate` paces by node throughput (empirically ≈ nodes/second), so `rate=1.0` replayed all 276 nodes in ~276-300s — confirmed independently from TF timestamps jumping ~316s of *recorded* time within the first ~15-20s of *wall-clock* replay (~16-20x speedup, consistent with 276 nodes / ~280s ≈ 1 node/s).
- **Fix:** for a database with N kept nodes (`rtabmap-info <db>` → `Global graph: N poses`) and a target wall-clock duration T seconds, set `rate:=N/T`. No "real time" value exists the way it does for `rosbag play` — there's no original inter-frame timing preserved to be real-time relative to. Documented directly in `slam_from_database.launch`'s `rate` arg doc so this isn't re-guessed.
- **Verify:** watch TF/log timestamps in the first ~15s of a fresh replay — if the recorded time shown has already advanced far more than 15s, the rate is not 1:1.
- **Verify:** `rtabmap-info <db>` → `Global graph:` line shows nonzero `GlobalClosure`/`LocalSpaceClosure` counts with sub-meter average lengths.

---

## Run 5's camera segfault is NOT caused by the ZED tracker diverging (settled 2026-08-26)

- **Symptom:** run 5's camera node died with exit code -11 (segmentation fault). The last thing
  visible on screen before it was `[ZED][ERROR] Positional tracking has diverged - Re-initializing
  odometry`, so the two looked causally linked and an obvious fix presented itself: disable the
  camera's own positional tracking, which this pipeline does not even consume.
- **Cause of the false link:** the ZED wrapper prints almost nothing after start-up, so the last
  message before a crash *always* looks adjacent to it regardless of when it happened. Timeline
  analysis put the divergence **15 minutes 6 seconds** before the crash, with odometry healthy at
  141 features in the final minute.
- **Settled experimentally by run 6:** the same divergence occurred again (`camera.log` line 196,
  00:49:30) and **the camera recovered and ran to the end of a 33-minute session.** Divergence is
  survivable and is not the crash mechanism. Root cause of run 5's segfault remains **undetermined**
  and has not recurred in one subsequent run.
- **Why the "fix" was right to reject:** it does not work (positional tracking restarts for nine
  separate reasons; `depth_stabilization: 1` forces it on — proved live, it restarted 41 s later the
  moment a subscriber appeared), it costs **1.73x noisier depth** (0.0645 m vs 0.0373 m jitter,
  non-overlapping over 9 samples), and it would have broken comparability with runs 3–5. Had it been
  applied, run 6's improvement would have been **credited to it** and a false result entered the study.
- **Verify:** `grep -c "diverged" ~/.run_records/<run>/camera.log` alongside whether the node
  survived. Divergence without death is the expected pattern.

## Keep every run's screen output — it is what turns guesses into answers (2026-08-26)

- **Symptom:** run 5 could not be root-caused at all, because both the camera's and the mapping
  stack's output went to a terminal and were lost when it closed. Two hypotheses (texture drives the
  error; divergence causes the crash) sat unresolved with no data able to settle either.
- **Fix:** append `2>&1 | tee ~/.run_records/<run_id>/{camera,mapping}.log` to the camera and mapping
  launches. Costs nothing, changes no behaviour, needs no parameter change.
- **What it immediately bought on run 6:** 22,224 feature measurements that **refuted** the texture
  hypothesis, and a logged divergence-without-crash that **refuted** the crash hypothesis. Both had
  been shaping decisions; neither survived contact with a full log.
- **Verify:** `wc -l ~/.run_records/<run_id>/camera.log` is nonzero after the run, and
  `grep -c "quality=" .../mapping.log` returns thousands, not tens.
- **Standing rule:** every run tees both logs. This is no longer optional.

## Loop closure is doing both the rescuing and the damage — the raw-vs-corrected split (2026-08-26)

- **Question:** the four real indoor runs gave closed-loop errors spanning 0.22 m to 11.47 m with
  nothing deliberately changed. Which part of the pipeline is responsible?
- **Answer, from the surviving databases:** compare the raw visual odometry (`Node.pose`, before any
  loop closure or optimisation) against the final corrected estimate.

  | | run 4 | run 5 | run 6 |
  |---|---|---|---|
  | raw visual odometry error | **1.83 m** | 9.60 m | 1.82 m |
  | after loop closures | **11.47 m** | 4.19 m | 0.22 m |
  | effect of the map | **6x worse** | 56 % recovered | 88 % recovered |

  **Run 4's odometry was excellent — 0.8 % drift over 234.56 m — and the map ruined it.** Run 5's
  odometry was genuinely bad and the map recovered more than half. The two large errors have
  **different causes**: falsification of recognition (run 4) and starvation of it (run 5).
- **Frame-level evidence** (RTAB-Map's `Statistics` table, zlib-compressed inside the database):
  run 5's "distance since last localization" climbed 0.10 m → **13.39 m with no recognition for
  ~180 s**; one closure with **298 inliers** (run median 166 — genuine, not false) collapsed the
  estimate from 10.32 m to 0.34 m in a single frame. Run 4's global-closure counter **froze at 12**
  and never moved again — it drove home and was never recognised. **Beyond 8 m, runs 4 and 5 fired
  zero global closures.**
- **Verify:** read `Node.pose` (48-byte float32 3x4 blobs) with python3 sqlite3 and sum inter-node
  distances — the total must equal the run's published "distance driven", which confirms it is raw
  odometry and not the corrected estimate.
- **Why this matters more than any single accuracy number:** it locates the problem in *place
  recognition*, not in the visual odometry front end. Tuning odometry would have been wasted effort.

## The integrity check has been reading MEMORY, not the disk — every past "verified" result is suspect (2026-08-26)

- **Symptom:** `lab_map_06.db` reported `PRAGMA quick_check = ok` at 01:07 immediately after its
  mapping run. Re-checked the same evening it **failed** with page-level errors. Nothing wrote to
  the file in between — only read-only queries and exports.
- **First clue, found by accident:** two queries against the same table disagreed.
  `SELECT count(*) FROM Node` returned 1269 while `SELECT weight, count(*) ... GROUP BY weight`
  summed to 19718 on `lab_map_05.db`. **Both cannot be true.** Different query plans were walking
  different, broken paths through the B-tree — the signature of a corrupt index.
- **Cause:** Linux keeps recently written data in the **page cache**. A database checked straight
  after a run is still entirely in memory, so SQLite reads the correct in-memory copy and reports
  `ok` **without ever touching the card**. If the write to the physical card was damaged, an
  immediate check cannot see it.
- **Proof:** `sync` then `sysctl -w vm.drop_caches=3` (buff/cache 18 GB → 401 MB), then re-read.
  Both databases failed from disk. The corruption was there all along.
- **What this re-frames:** the "verified healthy, then corrupted at rest" incidents in
  the project records — `office1_explore_04.db` `ok` on 2026-08-23 and malformed on 2026-08-24
  with no writes between — are most likely **not corruption at rest at all.** Those files were
  probably damaged on disk from the moment they were written, and the immediate check was reading
  memory. The card may be less guilty than the record suggests; the *verification method* was wrong.
- **Fix, applied:** `verify_run.py` now runs `sync` + `sysctl -w vm.drop_caches=3` **before**
  `quick_check`, reports `page cache dropped — the check below reads the PHYSICAL disk`, and marks
  the result **untrustworthy** if it could not drop the cache. It also cross-checks `count(*)`
  against a full table scan and reports `INDEX CORRUPTION` when they disagree — the check that
  caught this.
- **Verify:** run `verify_run.py` and confirm the `page cache dropped` line is PASS. Without it, a
  pass means nothing.
- **Standing rule:** an integrity check that has not dropped the cache is not evidence. Applies to
  every recording from now on.

## The Jetson's boot evidence kept vanishing — the journal was memory-only (2026-08-29)

- **Symptom:** three separate times the Jetson dropped off the network or died mid-run, and each
  time `journalctl -b -1` (the previous boot's log) returned **nothing**. The 2026-08-21 offline
  incident stayed OPEN for eight days purely because its evidence could not be read. The same thing
  happened again on 2026-08-29: the machine was power-cycled at the lab, uptime 9 min,
  `journalctl -b -1` → **0 lines**.
- **Cause:** `systemd-journald` defaults to `Storage=auto`, which means *persist only if
  `/var/log/journal` exists*. On this Jetson it did not, so the journal lived in
  `/run/log/journal` — **tmpfs, wiped on every reboot**. Every diagnosis window closed the moment
  the machine came back.
- **Fix** (run on the **Jetson**, passwordless sudo):
  ```bash
  sudo mkdir -p /var/log/journal
  sudo systemd-tmpfiles --create --prefix /var/log/journal
  sudo systemctl restart systemd-journald
  ```
- **Verify:** `journalctl --disk-usage` must report a real size *"in the file system"* (not
  "in the runtime"), and `ls -d /var/log/journal/*/` must show a machine-id directory.
  Measured after the fix: `/var/log/journal/dbfef1aa0b064bcf9d30ec3ad0886edb/`, 16.0 MB.
- **Cost:** tens of megabytes, capped automatically by journald. The internal disk had 24 GB free.
- **What this unlocks:** the next crash, livelock or network drop is diagnosable from
  `journalctl -b -1`. Note the standing companion rule in `docs/SOLVED.md` — **check `uptime` first**:
  an uptime exceeding the outage means livelock, not a crash, and the cause is still resident.
- **A formatting lesson worth keeping.** The fix was first handed over as
  `ssh <jetson> 'cmd1 && cmd2 && cmd3'` — correct from another computer, but the user was already
  logged into the Jetson and pasted the quoted part, so bash looked for a file literally named
  `sudo mkdir -p ... && ...`. **When the user is already on the target machine, give plain
  unquoted lines, one per block.** The instruction-format rule already says to name which machine;
  it must also match the shell they are actually sitting at.

## A direct cable showed lit port lights but no link: the interfaces were never brought up (2026-08-29)

- **Symptom:** a Cat-6 cable plugged straight from a computer to the Jetson gave nothing. Port LEDs
  lit at both ends, and yet `ping` failed and both kernels reported **`carrier=0`**.
- **Cause:** neither interface had ever been brought up administratively (`operstate=down`), so the
  link never negotiated. **The LEDs light from port power alone and prove nothing about link state.**
  A second cause on the Jetson: two NetworkManager (Ubuntu's network settings service) profiles
  competed for `eth0`, and the one asking for an automatic address (DHCP) won - on a direct cable
  there is no DHCP server, so the port was left unconfigured.
- **Fix:** give the cable a fixed address on both ends with `nmcli con mod <profile> ipv4.method manual
  ipv4.addresses <address>/24 ipv4.never-default yes ipv6.never-default yes connection.autoconnect yes`,
  and switch the competing DHCP profile's autoconnect off.
- **`ipv4.never-default yes` is the load-bearing flag.** Without it, plugging the cable installs a
  default route down a link that reaches only one host - the internet and the lab network break, and
  the symptom looks like "the WiFi stopped working" right after plugging in a cable.
- **Verify:** `cat /sys/class/net/<if>/carrier` on **both** ends. Do not trust the port LEDs, nor
  `/sys/class/net/<if>/speed` - it reports the LAST negotiated value and read `1000` while `carrier` was `0`.

## RViz froze on an X-forwarded full-resolution point cloud (2026-08-29)

- **Symptom:** RViz opened, then stopped repainting; the desktop showed "rviz is not responding"
  repeatedly. It was displaying `/zedx_front/zed_node/point_cloud/cloud_registered`.
- **Cause, measured:** `rostopic bw` on that topic reports **307.76 MB/s — 36.86 MB per message,
  2.3 million points** (1920×1200 organised). Over an X-forwarded connection the client and link
  cannot absorb that. The Jetson was simultaneously at load 7.47 on 8 cores.
- **Fix:** `sidewalk_perception/scripts/cloud_decimator.py` — subscribes on the Jetson, keeps every
  Nth point by striding the raw byte buffer with numpy, republishes to `/zedx_front/cloud_light`.
  **Measured after: 152.65 KB/s, 92 KB per message — roughly 2000× smaller**, and still perfectly
  adequate for judging which plane a surface lies on.
  ```bash
  rosrun sidewalk_perception cloud_decimator.py _stride:=401 _rate:=2.0
  ```
- **Why striding, not a voxel grid:** a voxel filter gives even spatial density and is the correct
  choice for a mapping pipeline, but it is a nodelet needing a manager and costs CPU on a machine
  already saturated. Striding is O(1) per output point and needs only numpy. **This node is a
  diagnostic aid — never put it in a SLAM pipeline**, because uneven density is exactly what a
  mapper must not be fed.
- **The stride default is 401, not 400,** deliberately: a stride sharing a factor with the 1920-pixel
  row width samples vertical bands instead of a lattice.
- **Verify:** `rostopic bw /zedx_front/cloud_light` should read in the hundreds of KB/s, not MB/s.

## `open_rviz.sh` skipped the robot model on every run — a process-name collision (2026-08-29)

- **Symptom:** RViz opened with **every display erroring at once** — Global Status, Husky body,
  Camera frames, the cloud. Cause was a missing fixed frame: `base_footprint` was not in the TF tree
  because the Husky model had never been started.
- **Cause:** the guard was `ps -eo comm --no-headers | grep -qx robot_state_pub`. But
  `zedx_front.launch` starts **its own** `robot_state_publisher` for `zedx_front_description`, and
  **Linux truncates every process name to 15 characters**, so the camera's publisher is also called
  `robot_state_pub`. The guard matched the camera, concluded the Husky model was already running, and
  skipped it. Because the run procedure always starts the camera first, **this fired on every run.**
- **Fix:** test the resource, not the process — `rosparam get /husky_description`, which is what the
  RobotModel display actually reads, with a 15-second wait and an explicit warning if it never
  appears.
- **The general rule this is the fourth instance of:** process names are a lossy proxy. The standing
  rule already forbids `pgrep -f` (self-matching, three prior false readings); this adds that even
  exact `ps -eo comm` matching collides when two packages run the same binary. **Check for the thing
  you need, not for a process you believe provides it.**

## The ZED X was pitched 3.24° nose-down and nothing modelled it (2026-08-29)

- **Symptom:** with the camera's *position* corrected and verified to 0.3 mm, the reconstructed floor
  still climbed away from the robot — **226 mm of false rise over 4 m of range**. Up close it looked
  fine; at the far end of the useful range it was severe. This is the signature of an angular error,
  which is invisible near the robot and grows linearly with distance.
- **Cause:** `cam_roll/cam_pitch/cam_yaw` in `zedx_front.launch` were all `0.0`, and the mount is not
  square. The bracket holds the camera **3.24° nose-down**, and no file said so. This is separate
  from, and was hidden by, the 0.54 m height error fixed earlier the same day.
- **Fix:** `cam_pitch = 0.0565` rad in **four places that must move together** —
  `sidewalk_perception/launch/zedx_front.launch` (`cam_pitch`),
  `sidewalk_bringup/urdf/husky_a200_real.urdf` and `sidewalk_sim/urdf/husky_real.urdf`
  (`zedx_base_joint` rpy), and `sidewalk_bringup/config/robot_frames.yaml` (`camera_front.pitch`).
  Both consumers apply it as a URDF `rpy="roll pitch yaw"`, so they cannot drift in convention.
  They also **do not sum**: the ZED driver's chain hangs off `base_link` directly and does not
  descend from `zedx_base_frame`, so the two `0.0565` values are parallel descriptions of one mount.
- **Verify:**
  ```
  rosrun sidewalk_perception measure_camera_tilt.py _frames:=5   # must be within ±0.3°
  rosrun sidewalk_perception check_gravity.py _samples:=600      # must read ~0°, not ~3.2 or ~6.5
  rosrun tf tf_echo zedx_base_frame zedx_front_base_link         # must be ~[0,0,0] AND ~[0,0,0]
  ```
  Floor slope over 4 m went from **226 mm to 2 mm**; unmodelled pitch from **−3.24° to −0.03°**.

### The part that matters more than the number: re-measuring is NOT verification

- **Re-running the plane fit after applying the correction proves almost nothing about cause.**
  `base_footprint` is bolted to the robot body (`base_link` minus a fixed 0.13228 m), **not to the
  world**. So the fit constrains the same quantity before and after — the *sum* of (camera tilt in
  its mount) + (body pitch on its wheels) + (floor-patch slope). Getting ~0 back is arithmetic
  confirming itself. It does prove the edit reached the live tree with the right **sign** — a flipped
  sign would have read −6.5° — and nothing more.
- **A uniform floor slope is invisible to this test and is NOT the confound.** The robot tilts with
  the floor, the camera tilts with the robot, `base_footprint` tilts with both, and it cancels. The
  earlier docstring naming "the floor is not level" as the confound was wrong; it is corrected.
- **What established cause was gravity.** The ZED X accelerometer at rest measures true vertical,
  which owes nothing to the transform tree — different physics, no shared failure mode with stereo
  triangulation. Rotated into `base_link` through the corrected model it landed **0.90° from
  vertical**. Camera-level-with-sloped-patch predicts ~3.2°; body-pitched-on-its-wheels predicts
  ~6.5°. Both excluded by a wide margin. **`check_gravity.py` exists for this and should be run
  before any mounting angle is believed.**
- **A circular claim was written and committed before being caught.** A comment asserted that
  "atan(0.226/4) = 3.23° confirms the normal independently". It does not: `rise/4` is identically
  `−n[0]/n[2] = −tan(pitch)` from the *same* fitted normal, with the plane offset cancelling. Two
  statistics of one fit are not two witnesses — **the identical failure mode as the withdrawn radius
  finding (`a565d31`)**, reproduced within hours of the document warning about it. The comment now
  explains why it was circular instead of stating the conclusion.

### Still open, deliberately not "fixed"

- **A uniform ~12 mm height residual.** The corrected floor is flat but sits at −0.012/−0.011/−0.010 m
  at 0/2/4 m. Flat-but-offset is a *height* error, not an angle error. **Do not nudge `cam_pos_z` to
  zero it** — `robot_frames.yaml`'s own header forbids adjusting a measured quantity until the output
  looks better, and 12 mm is inside the stacked uncertainty of a tape reading to a housing edge plus
  the datasheet's unresolved 31.8-vs-36.7 mm height ambiguity.
- **Roll stays 0.** The plane fit read +0.24°, gravity read −1.37°. They disagree by more than either
  is worth, so there is no honest figure. **Do not average two methods to make a disagreement go
  away** — they answer different questions.
- **Yaw is unmeasured and unmeasurable by both methods.** A rotation about vertical leaves a
  horizontal plane horizontal and leaves gravity unchanged. It needs a known straight edge, or a
  driven straight line compared against heading.

### Runs 1–6 were all recorded with this wrong

Every mapping run to date placed observations 0.54 m too low **and** untilted. Their trajectories are
still comparable with each other, but **no absolute height or ground-plane claim from runs 1–6 is
sound**, and none should be compared against a run recorded after this fix without saying so.

---

## The ZED X optical specification was written from memory, not read from the datasheet (2026-08-29)

**Symptom.** `perception_common.ZEDX` — the single dict every geometric claim in
`sidewalk_perception` and `sidewalk_slam` is built on — carried a vertical field of view of
**52 degrees**. The ZED X datasheet states the vertical field of view is **at most 45 degrees**.
A value above the manufacturer's stated maximum is not a rounding: it is wrong at every
resolution the camera offers, not merely at some.

**Cause.** The whole optical block had been typed from recollection rather than read off
`Official Resources/ZED X - ZED X Mini - Datasheet.pdf`. Five of its optical fields were wrong
and one of its IMU fields was wrong:

| field | was | datasheet |
|---|---|---|
| `lens_mm` | 4.0 | **4.6** (4 mm is not a lens Stereolabs sells for this body) |
| `fov_h_deg` | 80 | **73** |
| `fov_v_deg` | 52 | **45** — above the stated maximum |
| `fov_d_deg` | 91 | **87** |
| `aperture_f` | 1.8 | **2.0** |
| `imu_rate_hz` | 400 | **200** (400 Hz is the ZED 2's figure) |

Two further keys, `depth_err_at_1m_pct: 0.2` and `depth_err_at_15m_pct: 3.1`, were **invented**.
The datasheet publishes no 1 m figure and no 15 m figure; it publishes "< 0.4% to 2m" and
"< 7% at 20m".

**How far the invented pair spread — scope, not a count.** This entry originally said "six
files", and that undercount was itself a bug: an undercount in SOLVED.md tells the next reader
to stop looking. Do not trust a number here; run the search. As of 2026-08-29 the pair appears
in **21 tracked files across five packages plus `docs/`**:

```bash
git grep -lE "0\.2 ?%,? (error )?(at|to) 1 ?m|3\.1 ?% (at|to) 15 ?m|error_at_1m_pct|error_at_15m_pct|SPEC_NEAR_FRAC|SPEC_FAR_FRAC|0\.2% at 1 m|0\.2 % at 1 m" -- catkin_ws docs
```

What matters more than the total is that **three of those sites are not documentation** — they
are live values a run is scored against, and the first sweep missed all three:

| site | what it is |
|---|---|
| `sidewalk_evaluation/scripts/depth_characterization.py:66-67` | `SPEC_NEAR_FRAC = 0.002`, `SPEC_FAR_FRAC = 0.031` — the **pass/fail envelope** the depth tool judges measurements against |
| `sidewalk_evaluation/scripts/generate_report.py:336` | `lo, hi = 0.002, 0.031` — the same envelope drawn onto the report figure |
| `sidewalk_evaluation/config/depth_targets.yaml:86-87` | `error_at_1m_pct: 0.2`, `error_at_15m_pct: 3.1` |

`docs/guide/zed-sdk-reference.html` was also missed, and it is the file ENGINEERING_NOTES.md names as the
reference to consult for ZED camera facts — a wrong figure there looks authoritative and gets
believed. The remaining sites split into two kinds: ones that already carry a dated correction
note (`sidewalk_slam`, `sidewalk_perception`) and ones that still state the pair as fact
(`sidewalk_multicam`, `sidewalk_navigation/config/costmap_common.yaml`,
`docs/PROJECT_AUDIT.md:410`). **Check the grep output against that split before assuming a file
is clean.**

**Cost.** `obstacle_segmenter.py` computes where the ground first enters view as
`h / tan(pitch + halfFOV)`. An inflated half-FOV tips the bottom of the frame further down than
the lens can see, so the ground appears to arrive sooner than it does. The **near blind zone —
the band of pavement in front of the bumper the robot never observes — was under-reported by
20 cm**, 0.818 m claimed against 1.018 m real. Same direction, same safety number, as the
132 mm camera-height bug fixed hours earlier. Separately, `sensor_validator.py` scores the
achieved IMU rate against `ZEDX["imu_rate_hz"]` with a 5% tolerance, so **a healthy ZED X
delivering its rated 200 Hz would have been scored 50% low and FAILED** — the validator would
have condemned correct hardware on its first run.

**Fix.** Every field re-read from the datasheet PDF and annotated with the section it came from.
The two invented keys renamed to the ranges the manufacturer really quotes
(`depth_err_to_2m_pct`, `depth_err_at_20m_pct`) rather than left in place with corrected values —
a key called `depth_err_at_1m_pct` cannot be made honest. Three published IMU figures the dict
lacked were added (accelerometer noise density 2.3 mg, gyroscope noise density 0.20 dps,
sensitivity error ±0.5%), with the unit caveat stated: the datasheet prints no per-root-hertz
denominator, so the stored unit is the printed unit and no conversion is performed.

**Verify.**
```bash
python3 catkin_ws/src/sidewalk_perception/scripts/obstacle_segmenter.py --geometry
python3 catkin_ws/src/sidewalk_perception/scripts/obstacle_segmenter.py --self-test   # 79/79
pdftotext -layout "Official Resources/ZED X - ZED X Mini - Datasheet.pdf" - | sed -n '150,250p'
```
The self-test parses the docstring's checked-figures block and recomputes it from the live mount
and `ZEDX`, so a stale figure fails the suite by name rather than sitting there looking plausible.

**Standing lesson.** ENGINEERING_NOTES.md rule 6 already said "never guess a specification". This is what
guessing one costs: two independent bugs, months apart, both shrinking the same safety margin,
neither caught by any test, because a plausible number with a confident comment beside it reads
exactly like a verified one. **Read the document. Cite the section. Every time.**

## Stereolabs DOES publish the ZED X IMU noise densities — the claim that it does not was false and was blocking work (2026-08-29)

**Symptom.** `orbslam3_zedx_stereo_inertial.template.yaml`, `sidewalk_slam/explain.yaml` and the
generated `EXPLAIN.html` all asserted that Stereolabs publishes the IMU's ranges and resolution
"but NOT its noise density or bias instability", and concluded that a **3+ hour Allan-variance
run was required** before any stereo-inertial result could be believed. Stereo-inertial work was
parked behind a prerequisite that did not exist.

**Cause.** Nobody looked. The datasheet's "Sensors Specifications" → "Motion Sensors" table lists
`Accelerometer Noise Density 2.3 mg`, `Gyroscope Noise Density 0.20 dps` and
`Sensitivity Error ± 0.5%` **directly beneath the resolution figures those same notes cited**.

**Fix.** The claim corrected in all three places, with the section named. The values are recorded
in `ZEDX` but are **not** substituted into `IMU.NoiseGyro` / `IMU.NoiseAcc`, and the reason is
stated rather than hidden: ORB-SLAM3's fields are strictly per-root-hertz densities, and the
datasheet writes no denominator.

The two readings, both sensors, with the arithmetic. Conversions: 1 mg = 9.80665e-3 m/s², 1 dps
= π/180 = 1.745329e-2 rad/s. The estimates being compared against are
`IMU_NOISE_ACC = 2.0e-3` m/s²/√Hz and `IMU_NOISE_GYRO = 1.7e-4` rad/s/√Hz
(`make_orbslam3_config.IMU_DEFAULTS`).

| reading | accelerometer (2.3 mg) | gyroscope (0.20 dps) |
|---|---|---|
| literal, per √Hz | 2.2555e-2 m/s²/√Hz = **11.3×** the estimate | 3.4907e-3 rad/s/√Hz = **20.5×** the estimate |
| total RMS over bandwidth, ÷√100 Hz | 2.2555e-3 m/s²/√Hz = **1.13×** | 3.4907e-4 rad/s/√Hz = **2.05×** |

**This entry said "within ~1.2x" until it was corrected. That was the accelerometer's figure
standing in for both.** The gyroscope is off by more than a factor of two — a real disagreement
reported as agreement, which is the same failure mode as the invented depth percentages in the
entry above: one flattering number quoted where two were needed. Nor does the bandwidth
assumption rescue it: both figures divide by the same √BW, so the gyroscope stays 20.5/11.3 =
**1.82× further out than the accelerometer at every bandwidth**. Landing the accelerometer
exactly on its estimate needs BW = 127 Hz; landing the gyroscope exactly on its estimate needs
BW = 422 Hz. There is no single bandwidth at which the second reading reproduces both defaults.
The datasheet states no bandwidth at all — the 100 Hz above is the Nyquist of the 200 Hz output
rate and is an **assumption**, flagged as one.

The document does not choose between the readings, so neither do we. **Flagged and not changed:**
if the bandwidth reading is confirmed, `IMU_NOISE_GYRO` should move 1.7e-4 → 3.5e-4, because the
current value is the optimistic one — it tells the optimiser the gyroscope is quieter than its
maker claims. That is a tuning change and waits for the unit question, one variable per
experiment.

**What is still genuinely unpublished:** bias instability and the random-walk terms. The Allan
variance run remains worth doing for those — it is now an **improvement, not a prerequisite**.
A faster answer: ask Stereolabs support which reading of the density figures is intended.

## ORB-SLAM3 was being told the IMU runs at twice its real rate (2026-08-29)

**Symptom.** `make_orbslam3_config.py` writes `IMU.Frequency` into every generated
stereo-inertial settings file from `slam_common.ZEDX_IMU_RATE_HZ`, which was **400.0** for a
sensor the datasheet rates at **200 Hz**. The comment above that constant claimed these values
were "never written into a SLAM configuration directly" — true of the intrinsics, which do come
from live `camera_info`, and quietly extended to a constant it did not cover.

**Cause.** The 400 Hz figure belongs to the ZED 2 / ZED 2i and was carried across.

**Why it is not cosmetic.** [VERIFIED — `ORB_SLAM3/src/Tracking.cc:613-614`, and again at
`1411-1412` for the legacy parser, in the checkout snapshotted at
`backup/jetson-20260720/home/Developer/ORB_SLAM3`]:

```cpp
const float sf = sqrt(mImuFreq);
mpImuCalib = new IMU::Calib(Tbc, Ng*sf, Na*sf, Ngw/sf, Naw/sf);
```

The declared frequency scales all four inertial noise terms, and the halves move in **opposite**
directions. Over-declaring by 2x makes `sf` too large by √2, so the white-noise terms come out
**41% larger** (inertial data under-weighted against vision) while the bias-walk terms come out
**29% smaller** (biases treated as more stable than they are). Silent: no crash, no warning,
just a trajectory whose inertial weighting is wrong in a plausible-looking way.

**Fix.** Constant corrected to 200.0; the misleading comment replaced with a per-constant account
of what is a cross-check and what is actually emitted.

**Consequence for existing artefacts.** No stereo-inertial result has been produced yet, so
nothing published is affected. Any `orbslam3_*_inertial.yaml` on disk from before 2026-08-29
carries `IMU.Frequency: 400` and **must be regenerated, not hand-edited** — regenerating also
re-reads `T_b_c1` from the live TF tree.

**Still open [UNVERIFIED].** The datasheet states what the *part* produces; ORB-SLAM3 wants the
frequency of the *stream it is fed*, and the zed-ros-wrapper sits in between. Settle it with the
camera powered:

```bash
rostopic hz /zedx_front/zed_node/imu/data
```

---

## The occupancy grid's noise filter was configured to delete flat surfaces (2026-08-30)

**Symptom.** Solid cardboard boards standing 3.1 m ahead registered far fewer occupied cells
than their width warrants, and the map filled the gaps with **free** — asserting drivable
space where an obstacle stands. On 2026-08-29 a black board produced 1 cell where ~13 were
expected; on 2026-08-30 the same boards, rearranged, put a different board at 10 cells.

**Cause — the search radius equals the cell size, and the cloud is thinned first.**
Read from this build's own `rtabmap --params` help text (not from memory, not from the
online docs):

- `Grid/PreVoxelFiltering` = **true** — *"Input cloud is downsampled by voxel filter (voxel
  size is Grid/CellSize) before doing segmentation of obstacles and ground"*
- `Grid/CellSize` = 0.05 m, so after that step every surviving point sits on a 5 cm lattice
- `Grid/NoiseFilteringRadius` = 0.05 m — **the same distance**
- `Grid/NoiseFilteringMinNeighbors` = 5
- the radius parameter's help text ends *"Done after segmentation."*

On a 5 cm lattice a point's four side-neighbours sit at exactly 5.00 cm and its four
diagonals at 7.07 cm, so a 5 cm radius admits **at most four**. Five are demanded.
**A perfectly flat, perfectly measured surface fails by one neighbour.**

Measured offline against the live camera, reproducing the pipeline in that order: all three
boards showed a median of 3.0-3.9 neighbours and lost 73-80 % of their points.

**Fix.**

```yaml
Grid/NoiseFilteringRadius: "0.09"      # was 0.05, which equalled Grid/CellSize
Grid/NoiseFilteringMinNeighbors: "5"   # unchanged
```

At 9 cm the diagonals fall inside, so a flat surface reaches eight neighbours and the
threshold of five keeps real headroom, while a genuinely isolated speck still has none.

**Why not simply lower the threshold to 3.**

> ⚠️ **The reason first written here was WRONG, and was corrected on 2026-09-01.** It said
> *"clumps of three or four wrong points sitting together are exactly what a stereo mismatch
> produces, and a threshold of three waves them through."* **It does not wave them through.**
> Work the geometry on the same 5 cm lattice used above, where a 5 cm radius reaches only the
> four side-neighbours and never the diagonals:
>
> - a **three-point** clump — line or corner — gives its best-placed point only **2**
>   neighbours. Threshold 3 **rejects every one of them.**
> - a **four-point** clump needs a point with 3 of its 4 side-neighbours filled, which only
>   the T shape has. Of the **19** four-point arrangements, **15 are rejected outright**, and
>   in the four T orientations only the centre point survives — then stands alone.
>
> The old reason was plausible and never checked. The setting it argued for is still the
> right one; the argument was not.

**The real reason, which is about headroom on a real surface.** Both settings reach almost the
same cell counts (63 against 65), so the choice is not about which finds more. It is about how
much margin each leaves when the surface is imperfect — and every real surface is:

| setting | neighbours a point on a **flat filled** surface has | bar | headroom |
|---|---|---|---|
| radius 0.05, need 5 | 4 (sides only) | 5 | **−1 — a perfect surface already fails** |
| radius 0.05, need 3 | 4 (sides only) | 3 | **+1 — two missing points and it fails** |
| **radius 0.09, need 5** | **8 (sides + diagonals)** | 5 | **+3** |

*Plain terms: at 5 cm the circle is so tight that a flat wall barely qualifies, so the filter
spends its effort deleting wall rather than deleting noise. Widening the circle to 9 cm lets
a real surface answer the question comfortably while an isolated speck still has nobody near
it at all. Lowering the bar instead would keep the tight circle and just make the test easier
for everything, noise included.*

**Verified by — five stationary runs, one parameter changed at a time, boards untouched:**

| condition | white | printed | black |
|---|---|---|---|
| radius 0.05, need 5 (was standing) | 10 | **33** | 44 |
| radius 0.05, need 3 | 14 | **63** | 50 |
| **radius 0.09, need 5** | 18 | **65** | 48 |

```bash
# the check that confirms it, on the Jetson with a map running:
rosparam get /rtabmap/rtabmap/Grid/NoiseFilteringRadius     # expect 0.09
rosrun sidewalk_slam grid_span.py _expect_m:=2.388 _label:=check
# passes when: span is within ~2 cells per board of 2.388 m AND holes = 0
```

**Scope.** This is about the *filter*, not about surface texture. It applies to every flat
surface the robot maps, printed or blank, and was found because the boards made it visible.

---

## Loop closure stops dead on a reversed lap — confirmed, and how to tell

**Date:** 2026-09-01 (run 8) · **Symptom first seen:** run 7c, 2026-08-31

### Symptom

A mapping run stops producing loop closures partway through and never recovers, while
the robot is driving over ground it has already mapped. Run 7c's closures stopped at
node **749 of 1,012** and the final **16.07 m** ran on unaided odometry, with 98 % of
those nodes sitting within 0.5 m of mapped ground (median 0.111 m).

### Cause

**The lap was driven in the opposite direction.** Appearance-based place recognition
succeeds — it correctly says "I have been here". Geometric verification then fails
totally, because the same spot approached from the opposite direction shows different
surfaces. The log states it plainly:

```
Rejected loop closure 421 -> 1006: Not enough inliers 0/20 (matches=33)
```

*Plain terms: standing in one spot facing north and facing south is the same place but
opposite views. The recogniser asks "does this look like somewhere I have been" and
rightly says yes. The verifier asks "can I line these two 3D views up" and rightly says
no.* The two stages fail independently.

### Fix

**Drive every lap in the same direction** unless reversal is the thing being measured.
There is no parameter to change; this is a property of stereo geometry.

### The verification — a prediction registered before the drive

`docs/RUN8_PROCEDURE.md` was written first and committed, stating the pass criterion:
*"the highest node in a closure sits in the last ~10 % of the run."* Run 8 then drove the
same lab, same settings, three laps, **direction the only variable**:

| | run 8 (one direction) | run 7c (2 forward + 1 reversed) |
|---|---|---|
| highest node in a closure | **965 of 973 — 99.2 %** | 749 of 1,012 — 74.0 % |
| left on unaided odometry | **0.48 m** | **16.07 m** |
| loop closures (distinct pairs) | **265** | 167 |
| closed-loop error | 0.904 m over 91.06 m | 0.926 m over 53.71 m |

### The command that verifies it

```bash
# acceptances are DB-ONLY. There is no "Accepted loop closure" line in the log -
# run 7c had 167 acceptances and ZERO matches for that string.
rosrun sidewalk_slam audit_run_db.py --db <run>.db      # distinct pairs, not rows

# THE DISCRIMINATOR: zero inliers DESPITE many matches, not bare rejection count.
grep -aoE "Rejected loop closure [0-9]+ -> [0-9]+: Not enough inliers 0/20 \(matches=[0-9]+\)" \
     mapping.log | grep -oE "matches=[0-9]+" | cut -d= -f2 | awk '$1>=20' | wc -l
# run 8 = 3, run 7c's reversed lap = 7. Bare rejection counts do NOT separate them.
```

### Limits on the claim

**N=1.** Run 8 drove 91.06 m against 7c's 53.71 m, so the closed-loop *percentages* are
not like-for-like; the absolute errors (0.904 m, 0.926 m) differ by 22 mm and agree
within parking-by-eye uncertainty. **Only direction was the experimental variable** — the
NEURAL depth mode and the database durability change get no credit for this result.

### Why it matters beyond the lab

**A sidewalk route is an out-and-back — the way home is a reversed lap by construction.**
If stereo loop closure will not fire on the return leg, that is a design constraint on the
whole project. The untested candidates are a rear-facing second camera, or LiDAR, whose
scan geometry is orientation-independent in a way stereo is not.

---

## A run that gets SLOWER and TIGHTER at the same time is being censored

**Date:** 2026-09-09 · **Machine:** Jetson internal disk · **Applies to:** any
recorder that only writes frames the detector accepted

### Symptom

The 4 m camera run took **148.5 seconds per accepted sample** against the
morning's 29.5, while its base spread fell to **5.04 mm** from the morning's
12.24. Nothing in the CSV, the acceptance rate or the node log said anything was
wrong.

**A trap inside the trap.** The same run also posted RMS3D 4.09 mm against the
morning's 5.41 — apparently the best precision the camera had ever managed — and
this entry originally blamed that on the censoring too. It was checked and that
is wrong: bootstrapping gives 2.80–4.83 for it and 4.28–6.53 for the morning,
**overlapping**, so the two are not distinguishable at all, and artificially
censoring the morning's n=80 moves its RMS3D only about 9 %. **The censoring is
real but it shows up in the gated quantity — the base — not in RMS3D.** Never
rank spreads from n=6 or n=10; bootstrap them and look at whether the ranges
overlap.

### Cause

The frozen detector accepts a frame only if the measured base sits inside
1.040–1.160 m. The measurement had drifted to a median of 1.1518, leaving
**8.2 mm of room**. At that point the gate stops rejecting bad frames and starts
**cutting one tail off an ordinary distribution**. What survives is tighter than
what was measured, and it takes far longer to collect because most frames are
being thrown away.

*Plain terms: measure a hundred people but discard everyone over six foot, and
the survivors look impressively uniform. That uniformity is the discarding.*

### Fix

Report the **acceptance margin** beside every precision figure:

margin = (0.060 − |median base − 1.100|) × 1000, in millimetres

**Below about 2× the run's own spread, the sample is censored and the precision
figure is not the camera's.** `station_view.py` on port 8090 shows this as its
own tile and plots every recent sample against the gate.

Confirmed by giving the measurement room again: at a 23.7 mm margin the spread
returned to **12.5 mm**, matching the morning's uncensored **12.2 mm**. The
population never changed — only how much of it survived.

### The command that verifies it

```bash
# margin and spread together. Either alone is misleading.
python3 - <<'PY'
import csv, statistics as st
r=[{k:float(v) for k,v in x.items()} for x in csv.DictReader(open('<run>_zedx.csv'))]
b=[q['base'] for q in r]; m=st.median(b); sd=st.pstdev(b)
print('median base %.4f  spread %.1f mm  margin %.1f mm  ->  %s'
      % (m, sd*1000, (0.060-abs(m-1.100))*1000,
         'CENSORED - precision not usable' if (0.060-abs(m-1.100)) < 2*sd else 'clear of the gate'))
PY
```

### Why the acceptance rate does not catch it

The recorder reports its rate over frames it *offered*, and a censored run can
still post a high rate over a long enough window. The margin catches it; the
rate does not. Full write-up, with the three-run table, in the depth study's
`GUIDELINES.md` section 30.

### FUSE exFAT refuses a volume the kernel driver reads fine (2026-09-09)
- **Symptom:** `mount /dev/sda1` (the 29 GB CAM_REC stick) failed with
  `FUSE exfat 1.3.0  ERROR: unknown entry type 0x89.` and mounted nothing.
  Looked like yet another corrupted exFAT volume.
- **Cause:** the userspace FUSE exfat driver (this project's documented
  corruption engine, see 2026-08-25 above) also cannot *read* directory
  entry types it does not know. The L4T 5.10 kernel has a native exfat
  driver that handles them, but `mount -t exfat` is intercepted by the
  `/sbin/mount.exfat` FUSE helper before the kernel ever sees it.
- **Fix:** `sudo mount -i -t exfat -o ro <dev> <mountpoint>` — `-i` skips the
  helper so the kernel driver mounts it. Read-only on principle for any
  volume that might hold sole copies.
- **Verify:** `findmnt -n -o FSTYPE <mountpoint>` says `exfat` (kernel), not
  `fuseblk` (FUSE).

### The 256 GB USB stick: controller failure, not filesystem damage (2026-09-09)
- **Symptom:** after a power cycle the stick enumerated at full size but with
  `Write Protect is on`, no partition table, and udisks showing no filesystem.
- **Diagnosis, read-only:** every sampled offset from sector 0 to 100 GiB
  returns the same synthetic counter pattern (`55aa 55aa 0002 0003 ...`) — the
  controller is serving test data, not the NAND contents. Control read of the
  other stick on the same USB path returned a clean exFAT boot sector, so the
  host is exonerated. Survived a full Jetson reboot unchanged. Vendor string
  is the counterfeit-class "VendorCo ProductCode".
- **Host-side resets tried 2026-09-09, all non-destructive, ALL FAILED:**
  a full Jetson reboot, a USB `authorized` toggle (re-enumeration), a
  `USBDEVFS_RESET` ioctl (a real USB-protocol reset — the electrical
  equivalent of a physical reinsert), and a SCSI `delete` + host rescan
  (fresh INQUIRY). Every one returned the same test pattern and
  write-protect. The controller re-locks on every address, so even the
  protocol-level reinsert does nothing — which makes a physical unplug a
  long shot, not a likely fix.
- **The physical power cycle on a different host was tried 2026-09-09 and
  ALSO FAILED.** The stick was moved to another computer, fully unplugged for 30 s,
  and plugged directly into its USB port with no hub in the chain. It
  re-enumerated genuinely (USB devnum 12 → 14, new port 1-3, the UGreen hub
  gone from the bus entirely) and came back **`/sys/block/sdb/ro` = 1**, no
  partition table, no filesystem. That exhausts the list — there is no
  remaining host-side or physical action that has not now been tried.
- **Also established on that computer: the stick is USB 2.0 hardware.** Direct
  connection, no hub, and it still negotiates `speed=480` with `bcdUSB=2.00`
  on the USB 2.0 root hub while the computer's 10 Gbit/s bus sits empty. A
  SuperSpeed-capable device reports `bcdUSB` 2.10 even when falling back to a
  USB 2 link; exactly 2.00 means no SuperSpeed capability at all. Product
  string `USB2.0 F`, SCSI model `Flash Disk 2.0`. **Its ceiling is 480 Mbit/s
  ≈ 60 MB/s theoretical, ~40 MB/s real, on any port on any machine.** The
  UGreen adapter (Genesys GL850, a USB 2.0-only hub) was a co-equal
  bottleneck, not the cause — removing it changed nothing.
- **THE CAUSE, found 2026-09-09: the controller is running
  mass-production (factory) firmware.** A read of the first 16 MiB returns the
  strings `MPTools_20241226_A5S` and `AP_20241226_A4S`, repeating. MPTool is
  the manufacturer's mass-production utility — the software used to program a
  flash controller on the production line. Those strings are the controller's
  own firmware image being exposed as if it were the data area. This single
  fact explains every symptom at once: the synthetic counter pattern is a
  factory test pattern, there is no partition table because the data LUN is
  not the user NAND, and the device refuses normal operation because it is
  not in normal operating mode.
- **`WP=0` at the SCSI level — the device does NOT claim write-protection.**
  `sg_modes` returns `Mode data length=8, medium type=0x01, WP=0`. The
  read-only state is the kernel's, applied at attach, not a live assertion by
  the drive. `blockdev --setrw`, `hdparm -r0` and a direct sysfs write all
  failed to clear it (`/sys/block/sdb/ro` is not writable even as root).
  `sdparm --clear=WP` does not apply: sdparm has no `WP` field acronym, the
  WP bit lives in the mode parameter *header*, not in a mode page.
- **Do not trust a bare `strings` scan as a data-presence test.** The
  interlock in `tools/usb_force_unlock.sh` aborted on these firmware strings,
  reporting "REAL DATA FOUND". It was a FALSE POSITIVE — controller firmware,
  not user files. Any data-presence check on a device in this state must
  match user-file magic (`#ROSBAG`, `SQLite format 3`, filesystem boot
  sectors) and reject known firmware markers.
- **MEASURED CONCLUSION 2026-09-09, `tools/usb_find_files.py`, read-only,
  evidence at `docs/evidence_usb_stick_2026-09-09.txt`:** 2,064 blocks
  examined - the first 64 MiB contiguously plus 2,000 samples of 4 KiB every
  128 MiB spanning the whole 250 GiB. **All 2,000 whole-device samples
  returned the SAME 4 KiB block**, byte-identical (md5 head `202a73f600e9`).
  Only **16 distinct block contents** exist across all 2,064 blocks; 99.22 %
  are duplicates of another block. **Zero file-format magic** (`#ROSBAG`,
  `SQLite format 3`, exFAT/NTFS/FAT boot sectors, MP4, JPEG, PNG) anywhere.
  Of 19,661 printable-ASCII runs, **not one contained a four-letter
  alphabetic run** - real filenames and headers would produce thousands.
  **No user data is reachable through this controller, at any offset.**
- **CRITICAL DISTINCTION - unreachable is not the same as erased.** The
  controller never reads the NAND at all; it answers every address from a
  canned block set. That means the user data may still be physically intact
  on the flash chips. **Chip-off recovery (desoldering the NAND and reading
  it directly) remains the one route that could reach it.** Running MPTool to
  revive the stick would reflash and re-initialise the NAND and **destroy
  that last chance**. The two options are mutually exclusive: revive the
  hardware, or attempt to recover the data. Never both.
- **DECISION 2026-09-09: shelved, untouched.** Not reflashed, not binned - so
  chip-off recovery stays possible. Physical label: `docs/DEAD_USB_STICK_LABEL.txt`.
- **Consequence for recording:** this stick can never be a live recording
  target. The depth study's storage table already measures 6.6 MB/s on real copies;
  the internal disk's 170 MB/s is not reachable through it by a factor of ~4
  even against the theoretical ceiling.
- **The standing rule this reinforces:** never leave a sole copy on removable
  flash, and never on exFAT — see DO_NOT_REPEAT, same date.

---

## Copying a big recording off the Jetson: use a wired link, rsync, and a checksum read off the device (2026-09-10)

**Symptom.** Both removable routes for getting a recording off the Jetson failed within two days: the
USB stick died in its controller (entry above) and the 128 GB microSD card decayed from 61.7 MB/s to
0.72 MB/s mid-copy. Both failed in the flash, not in the link.

**Fix.** Copy over a wired network link instead, with `rsync` over `ssh` and **compression off** (an
SVO is already compressed; `-C` costs processor time on both ends and buys nothing):

```bash
rsync -s -a --info=progress2 -e "ssh -o ForwardX11=no -o Compression=no" <jetson-user>@<jetson>:<file> .
```

Measured: an 11.9 GB SVO in 240 s (~50 MB/s); another copy on the same cable sustained 85 MB/s.
**Take the rate from total bytes divided by wall-clock time at the end,** never from the progress line in
the first seconds (it read 11 MB/s at 40 s). Quote the rate of the transfer you are talking about.

**Verification - the checksum must be forced to read off the device.** A `sha256sum` run right after a
copy can be answered from the page cache (the computer's memory copy of recently used files), which
proves only that the kernel remembers what it just wrote. Hash the copy and, separately, the original
on the Jetson, and compare the two.

**Two details that matter:**
- **`rsync` writes to a hidden temporary name and renames only on success**, so a killed transfer leaves
  a dot-file, never a full-length file under the real name. **`scp` does not** - a killed `scp` leaves a
  short file with the correct name. Use `rsync` for anything valuable.
- **Do not pass `--partial`** on a restart after a suspected-bad transfer: the resumed file inherits the
  damage and still checksums against a source that agrees with it.

---

## The August benchmark databases existed in ONE place, and the place was 95 % full (2026-09-10)

**Found while hunting for disk space on the Jetson, not by an audit.** That is the
uncomfortable part: nothing was looking for this.

`~/bench_2026-08-29/databases/` held two RTAB-Map databases —
`three_panel_static.db` (4.47 GB) and `nofilter.db` (3.08 GB), 7.1 GB together.
**They are the raw evidence behind `docs/BENCH_RESULTS_2026-08-30.md`**, the
NEURAL-versus-ULTRA measurement that ENGINEERING_NOTES.md section 5 quotes for the depth-mode
decision (55 % vs 36 % GPU, 8.2 vs 9.7 Hz, the 546 mm → 46 mm p99 tail on blank
cardboard).

A second copy of `docs/bench_2026-08-29/` on another computer looked like a backup. It was **5.1 MB** — the
HTML viewers, the CSVs and the logs *built from* the databases, and no databases.
**A folder with the same name on two machines is not evidence that the contents
match.** `du -sh` on both ends takes one second and would have caught this any day
in the last twelve.

**Fix:** copied over a wired link at 85 MB/s, verified by independent SHA-256 on each
end, into `docs/bench_2026-08-29/databases/` — which git ignores already
(`.gitignore:28`, `*.db`), so the folder structure now mirrors the Jetson without
putting 7.1 GB into the repository.

```
OK  three_panel_static.db  e74291927408ee299d9e2b55745e8c957c95e5f9b91e7d20ac6b0dcca18b31bb
OK  nofilter.db            7f3a62421a6190ba0196df75b842727e08f473e9bc7011fe6030b95f9c5d533b
```

**The general rule this earns:** the project has now lost, or nearly lost, data on
a microSD card (five separate events), a USB stick (controller death), and — here —
a *working* internal disk that simply nobody had copied off. **Two of those three
were not device failures at all.** Before any disk gets close to full, list what on
it exists nowhere else:

```bash
ssh <jetson-user>@<jetson> 'du -x -h -d1 ~ | sort -h | tail -15'
```

then compare each entry against the other copy **by size, not by name**.

---

## Every SVO recording permanently costs the camera node ~33 MB and 1.4 % of a core (2026-09-10)

**Symptom.** Eight 4 m bursts, 20 seconds each, five minutes apart, nothing
changed between them. They should have held the same number of frames. They did
not:

```
149  141  133  130  121  120  116  114        a 23.5 % fall across 39 minutes
7.45 ------------------------------ 5.70 frames per second
```

**What it is not.** The machine around the node was idle the whole time — CPU
53 °C, GPU 47 °C, 20 GB of memory free, 19 GB of disk free, load average 5.25 on
8 cores. And the bytes per frame were flat at **2644 → 2654 KB**, so the content
being written never changed; only how often it arrived. This is inside the camera
node.

**Cause — and the sampler is what found it.** Sampling `zed_wrapper_nodelet`
every 30 seconds shows memory that does **not** drift upward smoothly. It sits
flat and then jumps:

| memory jump | at | burst that fired |
|---|---|---|
| +20.0 MB | 4.5 min | burst 4 — 4.1 min |
| +26.5 MB | 10.0 min | burst 5 — 9.6 min |
| +21.3 MB | 15.5 min | burst 6 — 15.0 min |
| +18.0 MB | 20.5 min | burst 7 — 20.5 min |

Every jump lands within half a minute of a recording starting, and CPU steps in
lockstep (181 % → 188 % of one core across five recordings). **The cost is paid
per START/STOP OF A RECORDING, not per minute of running.** Mean **33 MB and
1.4 % of a core, per recording, never returned.**

**Why the distinction decides what to do.** A leak against the clock would argue
for shorter sessions. A leak against the *recording count* argues the opposite:

> **Fewer, longer captures are cheaper than many short ones.** The 390 s block
> cost one start/stop. The eight bursts cost eight, and the last burst ran 23 %
> slower than the first because of it.

**Fix — restart the camera node between stations.** It costs about a minute and
starts each station from a clean baseline instead of inheriting every recording
the node has ever made. **SIGINT only — never `kill -9` the ZED node, that locks
the camera until a reboot.**

**What it does NOT damage.** An SVO file stores raw stereo images; depth is
recomputed at replay. A lower frame rate means **fewer samples, not worse ones** —
every frame written is a faithful stereo pair. The flat KB/frame is the evidence.

**Verify it on any future run** — arm the sampler before recording, then look for
steps rather than a slope:

```bash
ssh <jetson-user>@<jetson> 'for i in $(seq 1 120); do printf "%s %s\n" "$(date -u +%H:%M:%S)" "$(ps -o pcpu=,rss= -p <ZED_PID>)" >> ~/zed_rss_sample.log; sleep 30; done'
```

Figure: `05_figures/render_burst_rate_drift.py` → `fig_burst_rate_drift`.
Evidence: `03_data/svo_4m_0910/zed_rss_sample.log`, `record_bursts_4m.log`.

---

## A stale camera node costs DETECTION, not just frame rate — restart before every station (2026-09-10)

**Symptom.** At the 5 m station the frozen detector was finding the frame on only
**8 of 22 looks**, where 4 m had been essentially every look. The frame had been
carefully squared; the laser confirmed it (leg lengths 4 mm apart on a symmetric
frame, and the laser measures true geometry regardless of viewing angle).

**What was tried first, and was wrong.** The whole failure was attributed to frame
geometry — off-centre, turned, leaning — and the user was given three physical
corrections. Two of them were real and worth making. But after the frame was
squared to 0.00 degrees, detection was still only **6 of the last 25 looks (24 %)**.

**The fix was the user's suggestion, not mine: restart the camera node.** Same
frame, untouched, static across both windows (two alignment readings either side
gave offset 237.5 then 239.0 mm, turn 0.00 degrees):

| | detection |
|---|---|
| before restart, last 25 looks | **6 of 25 — 24 %** |
| after restart, 22 fresh looks | **13 of 22 — 59 %** |

Points on the weakest ball rose with it, ~4,600 to ~5,670.

**This is a DIFFERENT failure from the per-recording leak** documented above. That
one steps memory at each SVO start/stop, ~33 MB and 1.4 % of a core each time.
**This node had made no recordings at all** — 1 hour 34 minutes of uptime with
only the box viewer subscribed — and still came back materially better. So there
are two independent ways a camera node degrades, and only one of them is about
recording.

**HONEST LIMIT ON THIS RESULT.** 6-of-25 against 13-of-22 is a small sample
(Fisher exact p is around 0.02, so the difference is probably real, but it is one
pair). And the earlier window, while the frame was static across both alignment
readings, came at the end of a period in which the frame HAD been handled. The
restart is the likely cause, not a proven one. **It is not a controlled
experiment and must not be quoted as a percentage improvement in the depth study.**
What justifies acting on it is that the remedy is a minute long and cannot hurt.

**Rule: restart the camera node before every station**, not only after recordings.
Use `restart_cam_and_aim.sh <dist>`, which copies the live command line verbatim
rather than retyping it — `zedx_front.launch` declares `self_calib` and
`depth_stabilization` with empty defaults and sets each parameter only when the
argument is non-empty, so an argument left off silently reverts to the YAML value.
It then reads all four pinned parameters back and refuses to continue if any moved.

**SIGINT only. Never `kill -9` the ZED node — it locks the camera until reboot.**

---

## The offline replay hung with six workers at 0 % CPU — `multiprocessing` forked after the camera library had started threads

**Symptom.** `svo_replay_n.py` printed its plan line (`53 frames across 1 files ->
7 slices on 6 workers`) and then nothing, for as long as it was left. Six worker
processes existed, each at **0.0 % CPU**, each with an RSS **identical to the
parent's**, and the machine idle: `load average: 0.16`. No error, no traceback,
no timeout. A job that has hung and a job that is merely slow look exactly the
same from outside, which is what made this expensive.

The kernel said it plainly once asked:

```
$ ps -o pid,stat,wchan:24 -p 4665,4709,4710
   4665 Sl   futex_wait_queue_me     <- parent, waiting for results
   4709 S    pipe_read               <- worker, waiting for a job
   4710 S    futex_wait_queue_me     <- worker, waiting for a lock nobody holds
```

**Cause.** `plan()` opens every recording with the ZED SDK to read its frame
count. Opening a camera starts SDK background threads, which take internal
locks. `multiprocessing`'s default start method on Linux is **fork**, and fork
copies the parent's *memory* but only the *calling thread*. The children
therefore inherit those mutexes already locked, by a thread that does not exist
in them. The first worker that touches the library blocks forever.

*Plain terms: the parent handed each child a photocopy of a room in which a door
was locked, but did not copy the person holding the key. Every child then stood
at that door.*

**Fix.** Start the workers with **spawn**, not fork — each is a fresh interpreter
that loads the library itself and inherits no locks. `svo_replay_n.py`:

```python
ctx = mp.get_context('spawn')
with ctx.Pool(a.workers) as pool:
```

Costs one or two seconds per worker at startup, against hours of processing.

**Verifies it.** The log now reaches the workers' own initialisation, which it
never did before — `[Init] Depth mode: NEURAL` appears once per worker — and
`tegrastats` shows the cores actually working:

```
CPU [100%@1728,100%@1728,100%@1728,59%@1728,100%@1728,...]
```

**Two things to carry forward.**
- **Wrap any long detached job in `timeout`.** This one would have sat there
  overnight and reported nothing in the morning. `timeout -s TERM 900 python3 …`
  turns an invisible deadlock into a job that ends and says so.
- **~2.1 GB of RAM per worker.** Six workers took the Jetson from 2.0 GB to
  14.4 GB. Twelve would need about 27 GB of the 30.6 GB fitted — too close, so
  cap the worker count below the core count on any 12-core power mode.

---

## The self-match trap wears a fourth costume: scanning `/proc` by substring

**Symptom.** A reboot-resume guard was written to avoid `pgrep -f`, by reading
`/proc/<pid>/cmdline` directly. Tested against a name that was **certainly not
running**, it answered *running*:

```
running definitely_not_a_real_process_xyz  ->  YES   (there is no such process)
```

**Cause.** It flattened each command line into one string and asked whether that
string *contained* the name. The process doing the searching has the search term
in its own command line, so **it found itself**. That is precisely the `pgrep -f`
defect — the standing rule in ENGINEERING_NOTES.md section 0.3 item 8 — reimplemented by
hand while trying to obey the rule.

*Plain terms: it is asking a room "is anyone here called Bob?" while wearing a
name badge that reads Bob.*

**Why it mattered here.** The guard's job was to decide whether to relaunch the
replay campaign after a power cut. Inverted, it would have launched a **second
campaign writing over the first one's output files** — with nobody watching.

**Fix.** Compare each ARGUMENT whole, by base name, and skip our own process and
our parent outright:

```bash
running(){   # $1 = exact file name to find as one of a process's arguments
  local want="$1" d pid a
  for d in /proc/[0-9]*; do
    pid=${d#/proc/}
    [ "$pid" = "$$" ] && continue
    [ "$pid" = "$PPID" ] && continue
    while IFS= read -r -d '' a; do
      [ -n "$a" ] && [ "${a##*/}" = "$want" ] && return 0
    done < "$d/cmdline" 2>/dev/null
  done
  return 1
}
```

**Verifies it** — `/tmp/guardtest.sh` on the Jetson, sourcing the real function
out of the real script rather than a copy of it:

```
dashboard is running : YES (correct)
campaign is running  : YES (correct)
absent name matched  : NO  (correct)
matched ITSELF       : NO  (correct)
```

**The general rule, which now covers all four sightings:** *any* test that asks
whether a name appears **anywhere in a command line** will match the asker.
Substring is the flaw; `pgrep -f` is only its most common carrier. **Always test
a process check against a name you know is absent** — that single case is what
exposed this one, and a check that only ever gets asked about running things
will pass every test while being permanently broken.

**And a repeat offence worth noting:** the first attempt to run this test used
`$(sed …)` inside a double-quoted `ssh "…"` string, so the command substitution
ran on the LOCAL computer and read a path that only exists on the Jetson. Third time in
this project. **Write the script to a file and `rsync` it. Every time.**

---

## Five of ten replay workers silently replayed nothing — a depth-engine build race

**Symptom.** The NEURAL_PLUS pass of the offline campaign finished with `rc=0`
and reported

```
wrote 511 accepted samples of 935 frames offered (55%)
```

which looks like a slightly disappointing but ordinary result. It was not. The
per-worker counters told a different story: `burst03` showed **0 of 0 frames**,
`burst04` 25 of 118, `burst07` 16 of 109. **342 of the 935 frames were never
replayed at all.**

The pass's own log had said so, five times, among thousands of routine lines:

```
[ZED][ERROR] Deflating optimized model failed
[ZED][ERROR] [ZED] [Depth]  NEURAL CORRUPTED MODEL
[ZED][ERROR] sl::Camera::Open has not been called, no Camera instance running.
 Optimizing neural_depth_3.6 / 0.1%[>            ] 5min 49s est. left
```

**Cause.** The SDK compiles an optimized engine for a depth mode on first use
and caches it in `/usr/local/zed/resources/`. Ten workers started together, all
missed the cache, **one began building the engine and the other nine read the
half-written file** and got `NEURAL CORRUPTED MODEL`. Five never recovered.

*Plain terms: ten people arrive at a locked door, one goes off to cut a key, and
the rest keep trying the lock with the blank.*

**Why it was invisible, which is the worse half.** `worker()` did
`return shard, []` when the camera would not open — **identical to "this slice
found nothing"**. And the summary line divided by the *planned* frame count, not
the processed one, so a run that skipped a third of its input printed a
plausible acceptance rate and exited 0.

**Why it mattered here.** The entire premise of the campaign is that every depth
mode sees **identical frames**. NEURAL_PLUS saw 593 where NEURAL saw 935, and a
different subset at that — so its σ₃D could not be compared with any other
mode's, which is the only thing it was computed for.

**Fix, three parts, all in `svo_replay_n.py`:**
1. **Build the engine once, alone, before the pool exists** — `warm_up()` opens a
   single camera in the requested mode and grabs one frame. Cached: seconds.
   Uncached: minutes, once. Racing: never.
2. **A failed open returns the error**, the run prints `slice N FAILED: …` and a
   closing block saying the run must not be compared with the other modes, and
   **exits 4**.
3. **Totals count frames actually replayed**, and warn explicitly when planned
   and replayed differ.

**Verifies it** — a five-frame smoke test on the Jetson:

```
warming the PERFORMANCE depth engine (single process, so nothing races)
  slice 1 done: 2 of 2 frames accepted (running total 2)
wrote 2 accepted samples of 5 frames ACTUALLY REPLAYED (40.0%)
```

**The general rule this is the third example of in this project:** *a worker that
returns an empty result on failure is indistinguishable from a worker that
honestly found nothing.* Failure must travel back with the result, and any
"N of M" must divide by what was **done**, never by what was **intended**.
Only NEURAL and NEURAL_PLUS build engines; ULTRA, QUALITY and PERFORMANCE are
classic stereo matching with no model, and were never at risk.

---

## A check whose control outran everything it controlled for (2026-09-20)

**Symptom.** The box-flattering check certified the 8 m station's residuals as
"can be quoted as measured". The opposite is true: in the wider box QUALITY's
residual tripled, 22.89 mm to 72.66 mm, and the mode ordering scrambled.

**Cause.** The check compares each mode the 950 mm box was limiting against a
*reference* group - the modes the box was not touching - on the reasoning that
whatever widening does to those is what widening does anyway. The test was
one-sided: `moved more than the reference by 3 sigma`. At 8 m the reference is
QUALITY, which moved **+217.5 %** while every constrained mode moved +8.6 % to
+56.3 %. Nothing could clear a bar that high, so every mode passed by arithmetic
and the script printed a clean bill of health.

*Plain terms: the thing meant to be a flat baseline jumped higher than everything
it was a baseline for. The comparison still ran; it just could no longer say no.*

**Fix.** A control that outruns every mode it controls for is not a control. Both
implementations - `02_zedx_pipeline/s2_boxcheck_compare.py` and the handoff's
`11_cowork_handoff/scripts/build_handoff.py` - now detect that case, report
CANNOT TELL (exit 3 / verdict `cannot_tell` with the reason in the row), and say
what it does establish: that "did the fitted width reach the wall" is not
capturing what constrains these modes. Signed comparisons are printed with their
direction in words, never as a bare negative sigma.

**Verifies it** - run it on a station where the control behaves and one where it
does not:

```bash
python3 02_zedx_pipeline/s2_boxcheck_compare.py \
  --frozen-dir 03_data/scenario2/w3_3.0m_modes \
  --wide-dir   03_data/scenario2/w3_3.0m_boxcheck --wide-mm 1400   # exit 0, verdict issued
python3 02_zedx_pipeline/s2_boxcheck_compare.py \
  --frozen-dir 03_data/scenario2/w3_8.0Bm_modes \
  --wide-dir   03_data/scenario2/w3_8.0Bm_sy140_modes --wide-mm 1400   # exit 3, refuses
```

At 3 m the reference moves +1.1 % to +1.3 % - a real baseline - and the original
verdicts stand unchanged (PERFORMANCE flattered by the box, QUALITY a lower
bound). **A gate tested only against cases it passes looks fine while being
permanently broken**, which is the same lesson as the process-check rule in
ENGINEERING_NOTES.md section 0.3 rule 8.

## Patterns that described last month's filenames (2026-09-20)

**Symptom.** `build_handoff.py` produced a handoff with no trace of the 8 m repeat
station or either wide-box sweep. Nothing errored. The missing work looked
identical to work not yet done.

**Cause, four instances of one habit** - a literal where a pattern belonged:

| what changed | the pattern that missed it |
|---|---|
| a repeat station carries a letter, `w3_8.0Bm` | `[0-9.]+` cannot match `8.0B` |
| a wide sweep has its own folder, `w3_8.0m_sy140_modes` | glob `w3_*m_modes` |
| the box tag moved from after the mode to before it | one fixed filename shape |
| `folder[:-len('_modes')]` | gives `w3_8.0m_sy140`, which does not exist |

**Fix.** One block of patterns at the top of the file, both filename layouts
matched, and the station folder derived by regex rather than by string surgery.
The same `[0-9.]+` bug had already been fixed in `campaign_dashboard.py` the night
before - **when a naming convention gains a suffix, grep the whole tree for the old
pattern**, because it is never in only one file.

**Verifies it.** `python3 11_cowork_handoff/scripts/build_handoff.py` then check
the tables name every station:

```bash
cut -d, -f2 11_cowork_handoff/04_results_machine_readable/walkway3_plane_scenario2_by_depth_mode.csv | sort -u
```

---

## ROS 1 could not read the RoboSense Helios bags at all — no `rslidar_msg` exists for it

**Symptom.** Every Scenario 3 and 4 bag carries `/helios/packets` as
`rslidar_msg/RslidarPacket`, and nothing on the Jetson could decode it. `rslidar_sdk`
would not even build: `catkin_make` failed with "non-catkin packages... Try
catkin_make_isolated".

**Cause.** RoboSense publish `rslidar_msg` only for ROS 2. Every branch of it declares
`ament_cmake` and `rclcpp`; there is no ROS 1 branch to check out. The robot's mirror
carries `rslidar_sdk` but not the message package it depends on.

**Fix.** Write the ROS 1 package by hand -
`02_zedx_pipeline/rslidar_msg/msg/RslidarPacket.msg`:

```
std_msgs/Header header
uint8 is_difop
uint8 is_frame_begin
uint8[] data
```

**The command that verifies it, and why it is conclusive.** ROS computes a message's
checksum from its field types and names, recursively, and refuses a connection when two
ends disagree. So:

```bash
rosmsg md5 rslidar_msg/RslidarPacket        # 4b1cc155a9097c0cb935a7abf46d6eef
rosbag info <bag> | grep -A1 RslidarPacket  # md5: 4b1cc155a9097c0cb935a7abf46d6eef
```

Identical means it is the same message, not a lookalike. **Control:** `rosmsg md5
rslidar_msg/NoSuchPacket` returns "Cannot locate", so a match is not the check quietly
passing everything.

**And the decode itself was verified by invoking, not by trusting the build log**
(ENGINEERING_NOTES.md rule 11): 40 clouds out of 20 s of a real bag, 57,600 points each - exactly
32 rings x 1800 azimuths, the Helios 32's full frame - 99.1 % of them real numbers,
ranges 0.26 to 48.9 m.

**Built in `~/lidar_ws` on the Jetson, deliberately NOT in `catkin_ws`**,
which carries the ZED pipeline and three SLAM builds.

---

## The Helios reports a 2017 clock, and one measured constant fixes the whole pass

**Symptom.** Decoded clouds are stamped 1483229327 (1 January 2017) while the recording
is 1789852247 (19 September 2026). Transforms cannot be looked up at a cloud's timestamp,
so `sphere_centroid.py`'s tracking ROI cannot be placed at all.

**Cause.** The laser's internal clock was never set. `use_lidar_clock: true` reports it
faithfully. Setting it false does not help: the driver then calls
`std::chrono::system_clock::now()` (`rs_driver/.../basic_attr.hpp:269`), the real wall
clock, which `rosbag play --clock` cannot steer.

**Fix, and why it is exact.** Every data packet carries BOTH clocks - the laser's inside
the payload, the robot's in the ROS header the recorder wrote. The Helios MSOP header puts
`RSTimestampUTC` at byte 20: `sec[6]` then `ss[4]`, both big-endian
(`decoder_RSHELIOS.hpp:50-60`, `basic_attr.hpp:63-67`, `parseTimeUTCWithUs`). So:

```python
sec = int.from_bytes(bytes(msg.data[20:26]), 'big')
us  = int.from_bytes(bytes(msg.data[26:30]), 'big')
offset = msg.header.stamp.to_sec() - (sec + us / 1e6)
```

Over all 67,188 data packets of one pass: median 306622920.578160 s, middle half spanning
0.000617 s, full range 0.012939 s. **Six tenths of a millisecond across a 45 s pass** - one
number converts the entire recording.

Implemented in `02_zedx_pipeline/s3_find_target.py` (`packet_clock_offset`).

**What it also buys.** Nicolas's `helios_restamp.py` stamps a cloud with the replay clock
at the moment the cloud ARRIVES, which is after its last packet. Measured against the true
scan start, that is **0.1192 s late** - 48 mm of travel at 0.40 m/s, 101 mm at 0.85 m/s.
Previously invisible; now a number the depth study can carry.

---

## The tracked search box could not hold the target, and it halved the detections

**Symptom.** Nicolas's `dynamic_paired.launch` searches 0.9 m either side of the tracked
point in height. With it, the sphere-frame detector found the frame in 28 of 60 attempts
across a range of distances — and at one distance, 7.79 m, it found it in **0 of 12**
while finding it easily at 9.32 m and 4.72 m either side. A gap like that in the middle of
a working range is the signature of something structural, not of noise.

**Cause, and it is arithmetic rather than a hypothesis.** The frame is a triangle: two
spheres level, the third 1.10 m above them, radius 0.20 m. So

```
centre of the three   = 1.10 / 3     = 0.367 m above the lower pair
top sphere's centre   = 1.100 - 0.367 = 0.733 m above the centre of the three
top sphere's top edge = 0.733 + 0.200 = 0.933 m above the centre of the three
```

A box reaching 0.900 m **cuts the top sphere on every single frame.** The detector needs
ten points on it, and at range there are only a few tens to begin with.

**Fix.** Search 1.10 m either side. That clears the sphere by 0.167 m, which also covers
the 0.093 m a genuinely static point wanders in this frame; 1.00 m clears it by only
0.067 m, which is less than the wander.

**The command that verifies it** — a controlled sweep, 10 seeds per cell, nothing changed
but the box, on saved scans:

```bash
python3 "Research Paper - ZED-X vs LiDAR/02_zedx_pipeline/s3_find_target.py" --help
# the sweep itself is reproduced in 03_data/scenario3/GEOMETRY.txt, "THE FULL BOX GRID"
```

Result: every row at 0.9 scores 24/50 and every row above it 34–46/50, with the step
exactly at the height the geometry predicts. **Width does nothing** — 1.0 and 1.4 agree in
seven of eight pairs — so widening it only adds clutter.

**On a real drive:** 60 detections at 0.9 against 89 at 1.1, same recording, same detector,
same playback rate.

**The box is an argument in his launch file, not a detector setting**, so changing it does
not void the frozen comparison. Every provenance file records the box its run used, and
the 0.9 m run is kept beside the new one as the evidence.

---

## A drive that could not aim its search box, and a page that called it idle (2026-09-21)

**Symptom.** Scenario 3's last drive, `s3_0.85ms_run2`, stopped at 08:32 with
`locator: FAILED 2 14` and sat doing nothing for seven and a half hours. The live page
said *"nothing running on this drive right now"*, which was true, and gave no hint that
anything was wrong.

**Cause, first half — the bar was in the wrong place.** Before a drive can be replayed,
the camera's own view has to be aimed: a shallow search box is slid outward over sampled
frames until the frame is found often enough to say where the camera sits on the robot.
The search runs twice — a wide sweep to find roughly where the frame is, then a second
sweep aimed at what the first found. The code required **three hits in the wide sweep
before it would run the aimed one**. This drive got two. So the sweep that exists
precisely for the case of a hard-to-find frame was the one thing the difficulty prevented
from running. The two hits it did get were good, and agreed with the other three drives
about the mount to within a few centimetres — nothing about the answer was ever in doubt,
only the count.

**Fix.** One hit is enough to *aim* with (`s3_cam_locate.py`: `if len(got) >= 1`), and the
drive samples 30 frames rather than 14. The bar of three still applies to the **answer**,
which is what it was always for. The re-run found the frame in **18 of 30** and the drive
is replaying.

    # the check that proves it: the locator's own last line
    grep -h "^locator:" ~/zedx_vs_lidar_data/s3/camera/*.log

**Cause, second half — the page could not see the stage at all.** A replay writes one
counter file per worker and the dashboard reads those. The search writes no counters, only
the drive's own one-line progress file, so `_s3_camera_job` returned `None` and the panel
fell through to *"nothing running on this drive right now"* — for a drive two hours into
real work, and, on this night, for a drive that had genuinely died. **A message that is
printed for both a healthy state and a dead one carries no information about either.**

**Fix.** `_s3_setup_stage()` reads that progress line, which carries its own `N/M`, and the
panel gives the stage its own line, its own bar and its own time remaining. A line
containing `fail` is reported as a stop, in amber, by name. The page also now lists every
run that has not started, with how long it takes — read from each drive's own chain log,
where the start and end of every depth mode are already recorded.

    # the check: the panel must name a stage, not say "nothing running"
    curl -s http://<jetson-address>:8094/api/state \
      | python3 -c "import json,sys; d=json.load(sys.stdin)['campaign']['scenario3']; \
        print(d['runs_left_n'], [x['stage'] for x in d['drives'] if x['stage']])"

**The general lesson, which is the third time this project has paid for it.** A liveness
check that can only observe one KIND of work reports every other kind as nothing
happening. The cure is the same each time: report what is actually there, and make sure the
quiet state and the dead state cannot look alike.

---

## "Distance driven" is not one number — it depends entirely on how the path was sampled

**2026-09-23, while rebuilding the SLAM results from source files.**

**Symptom.** Recomputing each run's path length from its own trajectory gave figures far
larger than the ones the project had published: run 6 came out at **144.72 m** against a
documented **96.71 m**, run 3 at 69.32 m against 33.83 m. The runs whose figures came from
`poses.csv` instead reproduced **exactly** (run 8: 91.062 vs 91.06).

**Cause.** The two file types are not the same kind of series.
- `rtabmap_trajectory.tum` is recorded from tf at about **10.8 Hz** — run 6 has **21,547**
  poses over 1,995 s.
- `poses.csv` holds **graph nodes** — run 8 has **973** for the whole drive, because a node
  is only created every 0.05 m of travel (`RGBD/LinearUpdate`).

Adding up every step of a 10 Hz series counts the **position noise as travel**. The same run 6
trajectory, summed at different sampling spacings:

| sampling | path |
|---|---|
| every pose (10.8 Hz, 21,547) | **144.72 m** |
| resampled at 0.02 m | 108.32 m |
| resampled at 0.05 m | 99.50 m |
| resampled at 0.10 m | 93.77 m |
| resampled at 0.20 m | 88.43 m |

*Plain terms: the robot drove one distance. Measuring it with a very fine ruler adds in every
wobble and makes the journey look longer; measuring it with a very coarse one cuts the corners
and makes it look shorter. Neither is wrong, but a distance figure means nothing unless it says
which ruler it used.*

**Fix.** `build_slam_db.py` (project records) resamples any high-rate series to the
mapper's own 0.05 m node spacing before summing, and every row records `path_basis`,
`path_sampling_m` and the unsampled value beside the sampled one. **Two path lengths sampled
differently are never compared.**

**What this does NOT affect — and it matters.** The **closed-loop gap** is the straight-line
distance from the first pose to the last, so it does not depend on sampling at all. Recomputed
from source, it reproduced the published figure on **all nine runs that have one**: 03 0.7775
(0.77), 04 11.4711 (11.47), 05 4.1879 (4.19), 06 0.2213 (0.22), 07c 0.9255 (0.926), 08 0.9037
(0.904), 09a 0.4149 (0.415), 09c 1.6779 (1.678), 09d 1.8649 (1.865). **The accuracy numbers are
sound and independently reproducible; only the distance denominators were method-dependent.**

    # the check that proves it
    python3 "build_slam_db.py" (project records) --verbose

## Loop closures are link types 1 and 2 — type 0 is a neighbour and type 9 is gravity

**2026-09-23.** A first attempt at counting closures from `links.csv` returned **1,778** for
run 8, against a documented 265. Counting rows rather than distinct pairs doubles everything
(each link is stored twice, once each way), and the type filter was inverted.

Verified against this project's own runs:

| type | meaning | counts as a closure? |
|---|---|---|
| 0 | neighbour — consecutive nodes | **no** |
| 1 | global loop closure | yes |
| 2 | local / proximity closure | yes |
| 3, 4 | local-time, user | yes |
| 9 | gravity — one per node | **no** |

De-duplicated into unordered pairs and filtered to types 1–4, the counts reproduce the
published figures **exactly**: run 8 = 49 global + 216 proximity = **265**; run 9a = 54 + 136 =
**190**; run 7c = 42 + 125 = **167**. Run 8's `monitor.csv` independently records
`accepted_lc_global = 49`.

    # the check
    python3 -c "import csv,collections; p=collections.defaultdict(set)
    [p[int(r['type'])].add(tuple(sorted((int(r['from_id']),int(r['to_id']))))) for r in csv.DictReader(open('Week 7/08_run8/links.csv'))]
    print({t:len(v) for t,v in sorted(p.items())})"

---

## A LiDAR replay writes an EMPTY database, silently, unless `/use_sim_time` is true

**2026-09-23, at the lab, found on a 12-second test before it could cost a night.**

**Symptom.** Replaying recorded LiDAR packets through the robot's own
`3dreplay_pipeline.launch` produced a database that looked entirely normal — it appeared in the
right place, roslaunch shut down reporting *"all processes finished cleanly"*, and there was no
error at the end. It contained **zero nodes**. `db_to_tum.py` reported
`0 poses written of 0 nodes`.

**Cause.** `rosbag play --clock` publishes the recording's own time on `/clock`, but **nothing
listens to it unless `/use_sim_time` is `true`**. Every node ran on wall clock while the bag's
transforms carried the time they were recorded at, so the log filled with:

```
TFFramesWatchdog (base_link): Frame rear_left_wheel_link is not reachable!
Cause: Lookup would require extrapolation 82.163960736s into the future.
/rtabmap/rtabmap: Did not receive data since 5 seconds!
```

The 82 seconds is exactly how long after the recording the replay was started.

*Plain terms: the recording says "this happened at 23:42". The programs replaying it are looking
at the clock on the wall, which says 23:44, and conclude the data is from the future and cannot
be used. Telling them to read the recording's clock instead fixes it.*

**Fix.** Set it on the **isolated** master before launching the replay:

```bash
export ROS_MASTER_URI=http://localhost:11312
rosparam set /use_sim_time true
```

Same bag, same pipeline, with the flag: the database went from **96 kB and 0 nodes** to
**778 kB and a real map**, and `db_to_tum.py` produced a correctly stamped pose.

**Two things that make this dangerous rather than merely annoying:**
1. **An empty database is not an obvious failure.** It is created, it is the right size to look
   plausible, and every process exits cleanly. The only way to know is to count the nodes.
   **A replay is not finished until something has counted them** — about 96 kB means empty.
2. **`/use_sim_time true` must never reach the live master.** A leftover `true` silently freezes
   a real recording (ENGINEERING_NOTES.md rule 10). The replay therefore runs on its own master on port
   **11312**, and the live one on 11311 is checked afterwards to confirm it is still unset.

    # the check that proves it
    "/media/administrator/USB Drive/slam_series2/replay_lidar.sh" <run_id>
    # then, on any computer with Python:
    python3 "tools/db/db_to_tum.py" <the db> <out.tum>

---

## The database's `Node.pose` is the camera's raw tracking, NOT the corrected map

**2026-09-24, found by the drive-1 diagnosis (`DRIVE1_DIAGNOSIS.md` (project records)).**

**Symptom.** Drive 1 of series 2 showed a **39.6 m** start-to-end gap and a badly twisted map, although the
robot was parked on its start mark and the map had accepted a closure from the last node back
to the first that measured the two only **0.056 m** apart.

**Cause.** An RTAB-Map database stores two sets of positions:

| column | what it is | when written |
|---|---|---|
| `Node.pose` | where the **tracking alone** (visual odometry) put each node | once, at creation — loop closures never change it |
| `Admin.opt_poses` | where the **map** puts each node after loop closures | only on a proper shutdown |

Every tool here that reads a database (`db_to_tum.py`, `render_map.py`, `extract_graph.py`)
reads `Node.pose`. So those maps and gaps are **tracking drift, not the SLAM result**.

*Plain terms: the database keeps the robot's raw diary and, separately, the map's corrected
version. Only the raw diary is guaranteed to be there.*

**This splits the August series in two, and one chart had mixed them:**

| runs | measured from | gap kind |
|---|---|---|
| 03, 04, 05, 06, 07a, 07b | live `map → base_link` recording (`rtabmap_trajectory.tum`) | **corrected** |
| 07c, 08, 09a, 09c, 09d | `poses.csv` from `Node.pose` | **tracking alone** |

Run 8 shows the size of the difference: **0.904 m** tracking alone, **0.261 m** corrected. **The published values are still
correct; what was wrong was calling both kinds the same thing.**

**Fix.**
- `tools/db/optimize_graph_se2.py` rebuilds the corrected positions from the
  Link table, for any database, including one that was not shut down properly. **Validated
  against RTAB-Map's own saved `Admin.opt_poses` on run 8: all 616 nodes, median difference
  0.16 mm, largest 0.35 mm.**
- `build_slam_db.py` now writes `gap_basis`, `odometry_gap_m` and `corrected_gap_m` for every
  run.
- `render_map.py --poses <corrected.tum>` draws the corrected map, and every image states which
  positions it was drawn from.

**Two traps that stay attached:**
1. **A small corrected gap can be meaningless.** When the only long-range closure joins end to
   start, the corrected gap just repeats that closure's measurement (drive 1: 0.056 m, with
   points moved by up to 39.9 m to achieve it). Report the closure count beside it.
2. **Do not use `rtabmap-export --poses` on a database that was not shut down properly.** Its
   links are stored one way only, and the tool silently drops the closure node. It reported
   40.43 m for drive 1.

    # the check that proves it (any computer with Python, run 8's database)
    python3 "tools/db/optimize_graph_se2.py" "Week 7/08_run8/lab_map_08.db" /tmp/r8.tum
    #   -> end_gap_corrected_m 0.2608, end_gap_odometry_same_nodes_m 0.9014

---

## A script with `set -u` dies silently at `source /opt/ros/noetic/setup.bash` when run over ssh

**2026-09-24, twice in one night** (the automatic-stop self-test, then the drive-1 LiDAR replay).

**Symptom.** A script started over ssh (or detached with `setsid nohup`) exits at once, having
created nothing. Its output shows one line:
`/opt/ros/noetic/etc/catkin/profile.d/1.ros_distro.sh: line 3: ROS_DISTRO: unbound variable`.
The same script works when typed into an interactive terminal, which is why it was missed.

**Cause.** ROS's setup scripts read variables that may be unset. An interactive shell has
already sourced them from `.bashrc`, so the variables exist; a non-interactive one has not,
and `set -u` turns the first unset read into a fatal error.

**Fix.** Pause strict mode around the sourcing, in every script that starts ROS:

```bash
set +u
source /opt/ros/noetic/setup.bash
set -u
```

Applied to `replay_lidar.sh`, `record_lidar.sh` and `start_drive.sh` (which sources the
Jetson's own `~/.sidewalk_env.sh`, the same environment an interactive shell gets).

    # the check: start it the way it will really be started
    ssh robot 'setsid nohup bash "<card>/slam_series2/tools/replay_lidar.sh" <run> > ~/jobs/<run>.out 2>&1 < /dev/null &'

---

## The robot's "LiDAR map" is wheel+IMU odometry corrected by LiDAR - say so

**2026-09-24, read from the colleague's `self_navigation/launch/rtabmap_3d.launch`.** It remaps
`odom` to `/odometry/filtered` (the robot's wheel+IMU estimate) and sets
`RGBD/NeighborLinkRefining=false`, so consecutive map positions come straight from the wheels
and IMU; the LiDAR's 3D scans correct the path only where they match a place seen before
(`RGBD/ProximityBySpace=true`, ICP). Its grid uses `Grid/GroundIsObstacle=true`, so the floor is
stored as obstacle: draw it with `render_map.py --zband 0.15 2.0` (floor sits at about −0.13 m).

**Consequence for every comparison:** the reference is "wheel+IMU odometry corrected by N LiDAR
loop closures", with N stated. Between closures it is only as good as the wheels.
`compare_lidar.py` fills the once-a-second LiDAR positions in with the 50 Hz wheel stream, so
that camera positions pair up at the 50 ms tolerance at all. Unfilled, two once-a-second
series match only about 10 % of the time.

---

## Why the robot's wheel+IMU path doubles corridors: its heading drifts ~5 degrees a minute, even parked

**2026-09-24, from series-2 drives 1 and 2** (full report:
`REPORT.md` (project records), section 1.1).

**Symptom.** Maps built on the robot's `/odometry/filtered` draw each corridor two or three times
at different angles. The colleague-config LiDAR maps ended 3.84 m and 20.57 m from their start
after parking on the start mark.

**Cause, measured.** The heading turns clockwise at a steady rate even while the robot stands
still: −2.65 to −3.69 °/min on drive 1 and −4.65 to −5.44 °/min on drive 2, over 7 parked
stretches. That one rate explains 96 % of each drive's total heading error. The turns themselves
agree with the camera to about 1 % wherever the camera kept tracking. Nothing removes the drift:
the compass is off (`use_mag: false`, `mag_updates: false`, `docs/FUSION_INVESTIGATION.md:306-307`),
the Madgwick drift correction runs only in its compass branch, and the UM7 zeroes its gyroscope
only once, at robot boot.

*Plain terms: the robot knows which way it faces only by adding up small turns, and its turn
sensor reports a small false turn all the time that nothing subtracts.*

**Fix (tested offline).** Measure the rate while parked on the start mark, and subtract rate ×
time from the heading. Using only the START parked stretch, as a live correction must, the
end-to-start gap falls from 3.84 to **1.00 m** (drive 1) and from 20.57 to **0.41 m** (drive 2).
The camera's map pieces placed by the corrected path form a clean right-angled floor plan
(`figures/A_pose_source_maps.png`). **So every drive now starts and ends with ≥ 60 s standing
still.** The rate differs between drives (and by up to 1 °/min within a drive), so it is
measured each time and never carried over.

**Not a reference by itself.** It has no loop closure, and mid-drive it can still be metres off.
Scan-matching LiDAR odometry (rank 3 in the report) is the reference route.

    # the check (any computer): parked-rate fit on each drive's wheel path
    python3 "notes/..." (project records)   # see A2_wheel_bias.json

**Update 2026-09-24 morning: re-zeroing the gyro at the source HELPS but does not remove it**
(`results_robot_tests_2026-09-24.md` (project records)).
The UM7 driver's `/imu_um7/reset` service with `zero_gyros=true` only (approved by the user; a
sensor command, no file changed) took the parked drift from **−2.35 to −0.76 °/min**, and it was
still −0.61 °/min 2–4 minutes later. The registered pass line was 0.5 °/min, so it **FAILED as a
fix on its own**: keep the correction. Two things it showed:
- **The boot-time zero is not enough.** Only 4 minutes after power-on the drift was already
  −2.35 °/min (sensor warming 0.5 °C per minute at the time; cause not separated).
- **Re-zero + correction is the likely best use**: the correction then removes ~0.7 °/min
  instead of ~4. Using it before every drive needs the user's approval (the IMU is the colleague's).

    # the check (robot, parked, ~3 min; nothing else running):
    python3 "/media/administrator/USB Drive/slam_series2/tools/gyro_rezero_test.py" \
        "/media/administrator/USB Drive/slam_series2/gyro_tests"            # add --no-reset to only measure

## Stage-A LiDAR re-processing runs on the robot, on copies, and reproduces the robot's own run exactly

**2026-09-24** (`results_stageA_robot_2026-09-24.md` (project records)).
`rtabmap-reprocess` (robot, RTAB-Map 0.21.10) on a checksummed copy of the colleague's
`rtab_helios.db` reproduced the original drive-2 map to 0.1 mm (0 closures, 20.568 m end gap) and
repeated itself exactly (0.0 m over 865 positions); rewriting the stored positions with the raw
wheel path (`db_rewrite_poses.py`) changed nothing (0.04 mm). So one-setting changes on it are
trustworthy. About 4 minutes and 1.1-1.8 cores per step, confined to cores 8-11 at nice 19.

**Two traps hit on the way:**
- A path with a space (`/media/administrator/USB Drive`) passed through an unquoted shell
  variable split into two arguments and silently produced no metrics. Use a bash array.
- The robot's 0.21.10 has **no `-odom_input_guess`** (the Jetson's 0.21.13 has it), so tests that
  recompute scan-matching odometry with the wheel path as a guess (E2-E4) cannot run there as
  designed.

    # the check (robot, parked, nothing recording):
    bash "/media/administrator/USB Drive/slam_series2/tools/stageA_robot.sh" s2_static_02 E0
    # expect: closures 0, end gap 20.5679 m (2D), heading -118.7 deg

## The live map page drew the robot in the wrong place and hid tracking loss

**2026-09-24** (`results_page_fix.md` (project records)).
**Symptom:** on drive 2 the browser page's green dot sat 19-39 m from the robot for the last
8 minutes, jumped to the start mark whenever tracking was lost, and nothing said whether the map
had corrected itself yet. **Cause:** `live_map_server.py` drew `/rtabmap/odom` (the camera's raw
tracking, which loop closures never move) on top of the corrected map, and read nothing else.
**Fix:** the dot now comes from TF `map -> base_link` (latest available, never waiting on the ROS
clock); odometry messages are not read at all; a red TRACKING LOST banner comes from
`/rtabmap/odom_info_lite` field `lost` (the light copy of `/rtabmap/odom_info`, same message
type, without the big feature lists - changed after review, see below); "last map correction N s
ago, M so far" comes from `/rtabmap/info` fields `loopClosureId` / `proximityDetectionId`. Still
permanently subscribed to `/rtabmap/grid_map`.
**Verified by test T2** (drive 2 replayed on a spare master, port 11313). After all 106 closures,
TF `map -> odom` at the next node exactly matched the replay's own `/rtabmap/mapGraph` correction
(106 of 106), so the page's position source carries every correction; the dot is drawn at TF to
within 0.035 m (P1); compared directly with the last pose of the optimised path it agreed within
0.024 m at the 7 closures where that comparison was possible, and within 0.017 m at the end of the
replay. The old page was up to 39.1 m off. Banner shown within 0.47 s and cleared within 0.50 s in
a real browser; all 106 corrections shown, none invented.
*(Corrected after review, 2026-09-24: an earlier version of this entry gave "within 0.033 m after
all 106 closures" as a separate measurement. It is P1's drawing error again, by construction.)*
**Review round, same day** (`results_page_fix.md` section 9): the banner moved to the light topic
(a real `rgbd_odometry` on drive 2's pictures, on this build, publishes it with `lost` filled in;
the full topic cost the page 2.5-2.7 ms more per message); and the picture no longer grows without
limit when TF puts the robot far away - past 10 m outside the mapped area it keeps the map's size
and draws a magenta arrow at the edge (1 km out diagonally: 1236 x 636 pixels and memory within
2 MB of where it started, where the old code would have tried about 5.6 GB for the picture alone).

    # quick check of a page file (Jetson, spare master; ~2 min each; ports 11317/8119 free):
    cd ~/slam_series2/test_copies/t2/review
    T2_MASTER_PORT=11317 T2_PAGE_PORT=8119 bash t2_smoke.sh live_map_server_review.py
    T2_MASTER_PORT=11317 T2_PAGE_PORT=8119 bash t2_lite_real.sh live_map_server_review.py
    # the full replay (~50 min; needs a COPY of drive 2's database, and a NEW output folder -
    # t2_run.sh refuses one that already holds a run). First the user removes the old throw-away
    # map, which it also refuses to reuse:   rm /dev/shm/t2_replay_map.db /dev/shm/t2_replay_map.db-journal
    export T2_OUT=~/slam_series2/test_copies/t2/out_run3
    bash t2_run.sh master && bash t2_run.sh clockproof && bash t2_run.sh start
    #   then, after "replay ended":  bash t2_run.sh fake; bash t2_run.sh stop
    # then on any computer: python3 t2_evaluate.py <out_run3> --p4 <out_run3>/p4.json


## A backup that checks the card itself, not the computer's memory (2026-09-24)

**Symptom.** A card that discards writes looked perfect to `backup_to_card.sh`.
**Cause.** The checksum list was made from the card copy, and the check re-read it, both through
the memory cache. **Fix.** `backup_to_card.sh` (project records) now hashes the SOURCE and
reads every file back with O_DIRECT; the manifest itself is read back directly too. Tested:
healthy folder PASS (3 files, odd sizes, a space in a name); one flipped byte in the copy
FAIL; the Jetson's failing card FAIL ("the card holds something other than what was sent").

    # the check (Jetson): a card must pass this before it holds any backup
    python3 ~/slam_series2/tools/card_pattern_test.py write /media/<card>/pattern_test.bin --mb 1024
    dd if=/media/<card>/pattern_test.bin bs=4M iflag=direct status=none | python3 ~/slam_series2/tools/card_pattern_test.py verify --mb 1024
    # expect: "0 wrong"


## The storage page showed the Jetson's own disk twice, labelled as the card (2026-09-24)

**Symptom.** The old storage page (port 8092) gave the "microSD card" 53.8 GB total and 10.3 GB
free - exactly the internal disk's figures - while the card was not mounted at all.
**Cause.** It tested "does `/media/sidewalk/SIDEWALK128` exist", and when nothing is mounted there
that is an empty folder ON THE INTERNAL DISK, so measuring it measured the internal disk.
**Fix.** Version 2 of `catkin_ws/src/sidewalk_bringup/scripts/storage_monitor.py` decides
"mounted" only from `/proc/mounts` and gives an unmounted place NO figures; it finds the USB stick
by its ID (`/dev/disk/by-uuid/EFFD-FE82`), the retired card by its factory serial (CID), and the
robot's disk and card through a read-only reporter on the robot (`tools/
robot_storage_agent.py`, port 8113, started by `robot_side.sh storage -` and by `start`).
Tests T1-T11 and the numbers: `NOTES.md` (project records).
**Started at boot** by the Jetson's crontab `@reboot ~/s3/s3_boot.sh` -> `start_page
storage_monitor.py` (no arguments), unchanged.

    # the check (Jetson): an unmounted folder must come back with no figures
    python3 ~/storage_monitor.py --once | python3 -c "import json,sys; print([(d['key'], d['state'], d.get('avail')) for d in json.load(sys.stdin)['devices']])"
    # expect the stick 'mounted' with a number only while `findmnt /media/sidewalk/CAM_REC` shows it

## One real-time storage page for all four places (2026-09-24)

`<storage page, port 8092>` (Jetson, `storage_monitor.py` v2, started by the Jetson's crontab
`@reboot ~/s3/s3_boot.sh`) now shows the Jetson's internal disk, the 32 GB USB backup stick
(CAM_REC, found by its disk ID EFFD-FE82), the robot's own disk and the robot's card, each with
free space, fill rate, time to full and a plain status line against the project's real limits
(fused-drive floor 10,547 MB; robot card 8 GB = one drive + margin; robot disk under 3 GB). The
robot's figures come from `robot_storage_agent.py` (read-only, port 8113), started on the robot with
`robot_side.sh storage -`. 11/11 registered tests passed; the robot half answered live at 23:56.

**Bug the old page had:** its "microSD card" entry measured an empty folder when the card was not
mounted, so it showed the Jetson's own disk a second time. "Mounted" is now decided from
`/proc/mounts` only.

    # the check (any computer on the lab network):
    curl -s <storage page, port 8092>api/storage | python3 -m json.tool | head -40
    # and on the robot, if its half says "not running":
    bash "/media/administrator/USB Drive/slam_series2/tools/robot_side.sh" storage -

## The storage page said "all clear" about a robot it could not hear (2026-09-25, version 3)

**Symptom.** An independent check of storage page version 2 (port 8092) found: a switched-off
robot turned the top line into ALL CLEAR "Every disk has room" with the green pip, even when the
robot's card had last been seen too full for a drive; one damaged answer from the robot blanked the
whole page, the Jetson's own disks included; a USB stick pulled out while still attached read "not
plugged in" with no warning; the mount command it showed went through the FUSE add-on driver this
project lost data through; and a Jetson that hung (rather than refused) kept the green pip up.
**Cause.** The robot's last alarms were reset to 0 while it was silent; the robot's answer was
stored before its entries were checked; the stick branch returned "absent" before looking at the
mount list; and the browser only noticed failed requests, never slow ones.
**Fix.** Page version 3 (`catkin_ws/src/sidewalk_bringup/scripts/storage_monitor.py`) and reporter
version 2 (`tools/robot_side/robot_storage_agent.py`): while the robot is silent the top line
reads **"Jetson disks have room; robot not reporting since HH:MM"** (ROBOT SILENT, grey pip), and a
robot disk in trouble when last seen keeps its alarm, greyed with a still dashed frame; every robot
entry is checked before it is accepted; offline only after 3 missed answers in a row; a pulled-out
stick is amber with `sudo umount`; the mount line shown is `sudo mount -i -t exfat -o
uid=1000,gid=1000 <dev> /media/sidewalk/CAM_REC` and a `fuseblk` stick is amber; the browser gives
each request 4 s and shows NO DATA 10 s after the last good answer; the reporter measures each disk
in its own thread and serves the latest result, from `/`. 35 logic tests + browser, layout, swap and
robot checks, pass lines registered first: `NOTES.md` (project records).

    # the check (any computer on the lab network): version 3, and the robot's own name
    curl -s <storage page, port 8092>api/storage | python3 -c "import json,sys; d=json.load(sys.stdin); print(d['version'], d['verdict']['word'], '|', d['robot']['header'], d['robot']['host'])"
    # expect: 3 ... | answered at <robot-wifi-address> · HH:MM <robot name>   (or ROBOT SILENT / "offline", never ALL CLEAR, when the robot is off)

**Storage page 3.1 (2026-09-25 02:23, Jetson 8092, md5 a7aa20b6):** a second check of version 3 found the phone page overflowing in the robot-not-reporting states (up to 928 px of 812), "offline" on a card the answering robot had stopped measuring, program names (`robot_card`, `avail`, `RecursionError`) in rare fault texts, a 30,000-deep answer counted as a missed answer, and one measuring thread for all three Jetson disks (a hung USB stick froze the internal-disk figure a drive depends on) -> fixed by shorter greyed robot boxes and texts plus tighter phone spacing (now 727-781 px), "offline" only for a silent robot, plain words with error names to the log only, deep answers refused as "not understood", and one measuring thread per Jetson disk (internal disk 1.7 s old at most while the stick hung 30 s); P1-P6 passed, robot-only checks not run (robot off), details in `NOTES.md` (project records) - check: `curl -s <storage page, port 8092>api/storage | python3 -c "import json,sys; d=json.load(sys.stdin); print(d['version'], [(x['key'], x.get('age_s')) for x in d['devices'][:3]])"` (expect `3.1` and an age under 3 s for each Jetson disk).


## The blend's gyroscope-offset correction, round 3: held through stops, bad readings skipped, a gate that waits (2026-09-25)

**Symptom.** The robot's gyroscope reads a false slow turn of about −5 deg/min even parked
(`robot-heading-drift`). The conditioner (`catkin_ws/src/sidewalk_slam/scripts/ekf_input_conditioner.py`)
measures it while the wheels read exactly zero and subtracts it before the EKF. Round 2's review
found: a new stop's first seconds replaced the kept value with a noisy one; a not-a-number reading
poisoned the kept value (972 NaN published); nothing was ever kept at 12-15 readings a second; and
`start_drive.sh` could say "left alone" on a stale or unkept status line.
**Fix (round 3).** H1 the kept value holds through later stops until that stop's own 20 s value is
kept; H2 non-finite readings skipped and counted (`nonfinite N`); H3 the reading density is judged
against the stream's own measured rate; H4 step 6 waits (DO NOT DRIVE) until a status line written
after this start and at most 30 s old shows a kept value with n >= 160, or a named override file.
After the review: the hands-off line prints at step 1b, when learning starts.
**Result.** H1-H5 PASS as registered, independently re-checked; the round-2 negative controls fail
every new line. **Known cost, accepted 2026-09-25 03:24:** in the drive-2 replay the
heading change while standing still is one-sided (−0.74 deg against round 2's −0.29 deg; total size
unchanged), so T2 line 3 (end heading, tracking alone) fails as worded, −6.47 / −6.16 against 6 deg.
Drive 3 reports the same figure on the real robot.
**Verify.** `python3 "test_conditioner_round3.py" (project records) --skip-h5`
(H1-H4(c) PASS; H4(d) now differs by exactly the two storage-gate lines the user removed,
see `conditioner_round3/AFTER_REVIEW_start_drive.md`). On the Jetson:
`md5sum ~/catkin_ws/src/sidewalk_slam/scripts/ekf_input_conditioner.py ~/slam_series2/tools/start_drive.sh`
must read `61282b9a...` and `655011b1...`.


## 16-bit depth cuts the camera freezes about 3x (2026-09-26 00:40 Hamilton, rehearsals T5b vs T5c)

**Symptom.** Camera tracker (and camera program) froze 2-4 s during long RTAB-Map database saves (drive 3: 9 in 28 min;
T5b: 10 in 16.5 min, up to 3.6 s). **Cause.** ~6.3 MB written per map node, ~5.8 MB of it 32-bit depth, with saves
that wait for disk confirmation (FINDINGS.md, jetson_stalls_drive3_2026-09-25). **Fix.** `Mem/SaveDepth16Format: "true"`
in `catkin_ws/src/sidewalk_slam/config/rtabmap_zedx.yaml` (Jetson, md5 f0dd40da). **Result.** T5c: 2 freezes of 2.2 s
in 9.7 min (pass line 0, so reduced, not solved). **Verify.** `rosparam get /rtabmap/rtabmap/Mem/SaveDepth16Format`
during a drive -> 'true'; count /rtabmap/odom header gaps > 1.5 s in the run's fusion.bag.

## Long confirmed database saves - cut from 4.45 s to 0.86 s by DbSqlite3/Synchronous=0 (drive 5, 26 Sept 2026) - PARTLY solved
- **Symptom:** the map program's database saves took up to 4.45 s at the end of drive 4 and froze the camera once (2.9 s).
- **Cause:** confirmed saves (Synchronous=1) wait for the disk after each write; the database is on the internal disk now, not the card.
- **Fix:** `DbSqlite3/Synchronous: "0"` in rtabmap_zedx.yaml (JournalMode stays 1), plus `~/slam_series2/tools/db_check.py` after every drive.
- **Verified:** drive 5 longest save **0.86 s** (drive 4: 4.45 s); database check PASS (688 nodes) even after the camera crashed mid-drive.
  `rosparam get /rtabmap/rtabmap/DbSqlite3/Synchronous` -> 0 at drive start.
- **NOT solved:** the camera freezes. Drive 5 still had 2 tracker silences over 1 s (1.80 s, 1.60 s) in its first 1.5 min with no
  long save nearby - a different cause, unknown. The registered pass line (0 freezes > 1 s) FAILED.

## rtabmap_localization.launch always started in MAPPING mode - fixed in the localisation demo's own launch (26 Sept 2026, 03:30 Hamilton)
- **Symptom:** `roslaunch --dump-params launch/rtabmap_localization.launch` shows `/rtabmap/rtabmap/Mem/IncrementalMemory: 'true'`
  (mapping) although the file exists to localise. Never noticed because localisation had never been run in this project.
- **Cause:** `rtabmap_common.launch` ends the rtabmap node with `$(eval 'false' if arg('localization') in ('true','True','1') else 'true')`.
  Inside `$(eval)` roslaunch passes `arg('localization')` as the Python value `True`, not the string, so the test is always false:
  `roslaunch.substitution_args.resolve_args("$(eval arg('localization') in ('true','True','1'))", context={'arg':{'localization':'true'}})`
  -> `False`. The same pattern makes `delete_db_on_start:=true` never delete (the safe direction).
  **The same `arg(...) in ('true','True','1')` pattern is also in** (found by review, NOT edited):
  `orbslam3_baseline.launch:58-59` (`equalize`, `dry_run`: the flags are never passed),
  `slam_from_recording.launch:105` (`loop`: never passed), and `fusion_mapping.launch:162`, which uses
  `not in`, so its guard ALWAYS fires for odom_source=camera_with_hint: `i_understand_tf_ownership:=true`
  can never let it through.
- **Fix (staged, no live file changed):** `run/lib/localise/localise_run.launch` (Jetson) sets
  `/rtabmap/rtabmap/Mem/IncrementalMemory` to `false` again AFTER the include (last setting wins); its start script refuses unless the
  log says `rtabmap: Localization mode`. The one-line fix to rtabmap_common.launch (`str(arg(...)).lower() in ('true','1')`) is the
  user's decision - see that folder's DEPLOY_NOTES.md.
- **Verified:** bench 2026-09-26 03:33 Hamilton - log `rtabmap: Localization mode (Mem/IncrementalMemory=false)`, parameter server `false`,
  master map checksum and node count (540) unchanged after 4 starts. Check: `roslaunch --dump-params "<that folder>/localise_run.launch"
  run_id:=x database_path:=/tmp/x.db | grep IncrementalMemory` -> `'false'`.

## Localisation demo said "not localised" at the start mark, but RTAB-Map had localised (26 Sept 2026, fixed 09:35 Hamilton)
- **Symptom:** session s2_loc_01 (05:03-05:10): the watcher showed "LOCALISED no" for 7 min with the robot parked on the start
  mark; /rtabmap/info showed hypothesis 0.052 at node 77, 0 matches, 0 inliers on every message.
- **Cause (two parts):** (1) RTAB-Map localised on its 2nd picture (05:03:24-25: odometry-cache links 541->1 recognition,
  542->77/503 nearby re-match; published uncertainty 0.023 m, never 9999) - before start 1 was written (05:03:26) and before
  the watcher ran, and the watcher counted only one-message recognition events. (2) A robot parked below
  `RGBD/LinearUpdate` 0.05 m / `RGBD/AngularUpdate` 0.05 rad, already holding a fix, makes RTAB-Map skip recognition entirely
  (Rtabmap.cpp:1969, 0.21.13), so no further event ever came; the 0.052 was one stale value.
  Offline replay of drive 6's start pictures reproduced it (parked: 0 fixes; moving: every picture).
- **Fix (staged, demo folder only):** `localise_watch.py` also counts a fix from `/rtabmap/localization_pose` uncertainty
  (< 9999) and from map links in `/rtabmap/info` `odom_cache`; start 1 accepts evidence from before its line. PROTOCOL.md:
  after "go" at each spot, nudge about 10 cm or turn about 5 deg, then keep still. Settings unchanged.
- **Verified:** `bash "test_watch_replay.sh" (project records) positive`
  (Jetson, private ROS master on port 11412) -> "LOCALISED yes" from the bag's first message, first fix (0.034, -0.035) m;
  `... negative` (synthetic 9999 stream) -> "LOCALISED no" for 35 s, 0 fixes. Details: that folder's RESULTS.md.

## rospy on the robot fails from cron/ssh: "ResourceNotFound: rosgraph" (2026-09-26)
- **Symptom:** a Python ROS reader started by our robot agent (from cron or a bare ssh) died with `ResourceNotFound: rosgraph`.
- **Cause:** the environment had no ROS setup sourced; cron and non-interactive ssh do not read ~/.bashrc.
- **Fix:** `robot_storage_agent.py` v4 sources `/opt/ros/noetic/setup.bash` before starting `robot_battery_reader.py`.
- **Verify (Jetson):** `curl -s http://<robot-wifi-address>:8113/health.json` shows `"battery": {... "state": "reading", "volts": ...}`.

## Camera freezes (ZED driver picture-loop stalls) grow with Jetson uptime (2026-09-26)
- **Symptom:** camera tracking silent > 1.5 s, 1-3 times per drive on drives 4-6, 15-16 on drives 7-8; parked 6 -> 23 per 10 min through the day; not the tracker engine, not lights, not any of the day's added services (each removed in turn).
- **Cause:** time since the Jetson last booted (1-4.5 h: ~1 per 10 min; 15-20 h: 6-23). Mechanism inside the Jetson/ZED stack not identified.
- **Fix:** restart the Jetson before a drive when it has been up more than a few hours (`sudo -n shutdown -r +1`; status pages, sender and storage checks come back by themselves).
- **Verify:** `cd "camera_segfault_2026-09-26" (project records) && bash freeze_cause_run.sh <label> GEN_2 420 && python3 freeze_count.py bench/freeze_cause/<label>` -> after a fresh boot F=0 (R1_after_reboot), before it F=16 (L1_lights_on).

## Localisation: first fix slow although the right place was recognised at once (s2_loc_04, 27 Sept 2026)
- **Symptom:** told it was 40 m outside, the robot's first accepted fix came after 213 s / 6.1 m, although the camera
  recognised the right map node (0.28 m away, 97 matched points) 0.4 s after the start.
- **Cause [VERIFIED, Rtabmap.cpp 0.21.13 + /rtabmap/info]:** in localisation mode a recognition is only accepted when a
  SECOND one reaches `Rtabmap/LoopThr` (0.08) while the first is still in the last `RGBD/MaxOdomCacheSize` (10) positions;
  pictures taken standing still are not added, so standing still the held one never expires and no still picture
  scored 0.08 (median 0.050, max 0.069). Nothing refused anything: 0 inlier or optimisation rejections.
- **Fix (procedure, no parameter change):** after the start stand still at most 60 s; if not localised, drive on along
  the map's route. Lowering the bar (0.065 / 0.06) or cache 0 would pass W1 but gave false off-map fixes before
  (DO_NOT_REPEAT). Evidence and what-if table: `whole_floor_diag/` (project records).
- **Also:** sessions are not replayable (fusion.bag has no pictures): record with SVO=1 or Mem/LocalizationDataSaved=true.

## Continuing a saved map in MAPPING mode (multi-session SLAM) - bench-verified (27 Sept 2026, 09:13-09:39 Hamilton)
- **What:** RTAB-Map 0.21.13 in mapping mode on a COPY of drive 10's closed map (`start_slam.sh`, staged in
  `run/lib/`) loads the 860 graph nodes, starts map id 1, and joins the old map on
  ONE recognition (bar `RGBD/AggressiveLoopThr` 0.05 until joined; no second-opinion rule in mapping mode; nearby
  re-matching cannot reach the old map before the join - Rtabmap.cpp:2146-2157, 2666-2689, 3244; DESIGN.md there).
- **Verified (bench, camera on the start mark, robot off):** joined 2.5 s / 3.6 s after the map loaded (to drive-10 nodes
  138 / 144); closed properly on "park" (saved graph = 860 old + today's); master sha256 unchanged; SVO replay into a
  fresh copy also joined and closed. Check: `bash "$S/stop_slam.sh" <run>` -> 5 PASS + "master map unchanged".
- **Traps found:** `check_closed.py` passes on ANY copy of a closed map (opt_poses already there) - use
  `slam_closed_check.py --after`; `db_to_tum.py` / `db_corrected_tum.py` mix both sessions - use `session_split.py`;
  the database grows ~2.3 MB per node even for discarded standing-still nodes.

## Navigation prep (27 Sept 2026, simulator) - small traps solved
- roslaunch splits `args=` at the spaces in our folder path ("<repo folder>"): use single quotes inside, `args="'...'"`.
- A map->odom publisher must post-date its stamp by 0.1 s, as RTAB-Map does (its tf_tolerance is 0.100), or move_base drops TF.
- `tf_echo -n` does not exist on this build.
Details: catkin_ws/src/sidewalk_navigation/doc/DESIGN.md.
