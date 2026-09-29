# sidewalk_perception

**Phase 2 — camera bringup and sensor validation.**

Brings the Stereolabs ZED X online on the Jetson AGX Orin, and measures whether
the data it produces is trustworthy before any SLAM algorithm is allowed to
consume it.

> **Read [`EXPLAIN.html`](EXPLAIN.html) first.** It explains this package in
> plain language, including the stereo geometry and the error propagation that
> motivate the design. Open it in any browser; it needs no internet.

## The short version

The camera is the easy part. The real deliverable is the validation tooling,
because a camera can drop frames or mis-stamp timestamps with no visible symptom
at all — and this project's central result is a three-way comparison of SLAM
systems on identical input. A faulty stream would degrade all three by different
amounts and look exactly like a genuine algorithmic finding.

## Quick start

```bash
# 1. What does this machine actually have installed? Touches no hardware.
rosrun sidewalk_perception perception_common.py

# 2. Start the camera.
roslaunch sidewalk_perception zedx_front.launch

# 3. Measure it for 15 minutes and get a verdict.
roslaunch sidewalk_perception validate_sensors.launch

# 4. Record a drive, then describe it.
roslaunch sidewalk_perception record_dataset.launch duration:=600
rosrun sidewalk_perception make_manifest.py validate data/recordings/NAME
```

## Exit codes

Every executable here uses the same convention, so a script can tell a failed
measurement apart from a measurement that could not be attempted.

| Code | Meaning |
|---|---|
| 0 | PASS — ran, and met every threshold |
| 1 | FAIL — ran, and the data did not meet spec |
| 2 | UNAVAILABLE — could not run at all, and says specifically why |
| 3 | INTERRUPTED — Ctrl-C; partial results are still written |

Nothing here raises a traceback when hardware is missing. A traceback is a bug,
not a diagnosis.

## Works without hardware

These run at a desk, with no camera, no robot, no ROS master, and no ZED SDK:

- `make_manifest.py` — all subcommands, including HTML generation
- `perception_common.py` — environment probe and depth-uncertainty table

`frame_audit.py --bag <file>` needs no camera, no robot and no `roscore`, but it
does need the `rosbag` Python library to open the file, so it runs on the Jetson
or on any machine with `python3-rosbag` installed — not on a bare computer without ROS. It
says exactly that and exits 2 rather than failing obscurely.

## Notes

- **`zed_wrapper` is deliberately not a declared dependency.** It is built by
  hand from its Noetic branch against the ZED SDK; declaring it would make this
  package fail to configure on any machine without the SDK. Launch files locate
  it at runtime, and `start_driver:=false` skips it entirely.
- **Everything is namespaced per camera** (`zedx_front`). A second, rear-facing
  ZED X is planned; unprefixed frame names would silently corrupt the transform
  tree once it is fitted.
- **Intrinsics are never hardcoded.** They come from live `camera_info`, because
  per-unit factory calibration differs from the datasheet by enough to matter at
  range. The datasheet values are used only as a plausibility check.

Logs go to `logs/sidewalk_perception/`, with one summary line per run in
`logs/RUNLOG.md`.
