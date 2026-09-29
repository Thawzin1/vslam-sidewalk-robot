# sidewalk_slam

**Phase 3-4 — Visual SLAM.** The robot builds a map of a place it has never
seen, works out where it is on that map, and three different methods are made to
compete on exactly the same data.

> **Read [`EXPLAIN.html`](EXPLAIN.html) first.** It is the plain-language
> explanation of what this package does and why, written for readers who have
> never used ROS. This README is only a pointer and a cheat sheet.
>
> Regenerate it with:
> `python3 tools/explain.py catkin_ws/src/sidewalk_slam`

## What is in here

- **RTAB-Map** — the primary mapping and localisation system, fed with ZED X
  depth, rectified colour and the camera's 200 Hz IMU. Produces a 2D occupancy
  grid *and* a 3D point cloud, in one saveable database, with separate mapping
  and localisation modes.
- **Two comparative baselines** — the ZED SDK's own positional tracking, and
  ORB-SLAM3 (stereo or stereo-inertial).
- **A runner** that drives all three over the same recording and emits
  trajectories in one common TUM format for the evaluation package.
- **A validator** that fails a run if the result is physically impossible.

## First five minutes

```bash
# What does this machine actually have installed?
python3 catkin_ws/src/sidewalk_slam/scripts/slam_common.py

# How do I get the missing half of rtabmap_ros? (plans only; --execute to build)
rosrun sidewalk_slam build_rtabmap_ros.py

# What would the full comparison run? (works with no ROS at all)
rosrun sidewalk_slam slam_runner.py --input recording.bag --dry-run
```

## Prerequisites

- The camera stack from `sidewalk_perception` must be running, **or** a rosbag
  or SVO2 file must be playing. Nothing in this package starts the camera —
  that is deliberate, so these launch files also work offline.
- `rtabmap_ros` must be built. Only `rtabmap_msgs` exists on the Jetson today;
  `build_rtabmap_ros.py` explains and plans the rest.
- ORB-SLAM3's settings file must be **generated**, never copied:
  `rosrun sidewalk_slam make_orbslam3_config.py --from-topic --out ~/orbslam3_zedx.yaml`

## Three things that will cost you a day if you skip them

1. **Do not `apt install ros-noetic-rtabmap-ros`.** It installs a second copy of
   the RTAB-Map core library beside the 0.21.10 already in `/usr/local`. It
   links, it runs, and it corrupts a mapping session hours later.
2. **Only one node may publish `map` → `odom`.** Turn the ZED wrapper's
   transform publishing off whenever RTAB-Map is running. Nothing warns you.
3. **Revisit places while mapping.** Loop closure is the only mechanism that
   removes drift. A straight-line route has no loop to close, and the map will
   be pure accumulated error no matter how good the algorithm is.

## Logs

Every executable writes a timestamped log to `logs/sidewalk_slam/` and one
summary line to `logs/RUNLOG.md`. Trajectories go to
`logs/sidewalk_slam/trajectories/<run_id>/`; maps to
`logs/sidewalk_slam/maps/` and `~/rtabmap_maps/`.
