# sidewalk_evaluation — Phase 5

Measures how accurately the robot knows where it is, and what that accuracy
costs in compute and delay. This is the package that turns a working robot into
a research result.

**→ Read [`EXPLAIN.html`](EXPLAIN.html) first.** It is the plain-language
explanation of what this package measures and why, written to be readable by
someone who has never used ROS. Regenerate it with:

```
python3 tools/explain.py catkin_ws/src/sidewalk_evaluation
```

## Start here

```bash
# Verify the metric mathematics itself — no camera, no robot, no ROS needed.
rosrun sidewalk_evaluation evaluate_trajectory.py --self-test

# Find out what this machine is capable of running.
python3 catkin_ws/src/sidewalk_evaluation/scripts/eval_common.py

# Verify the navigation-run ingestion path — also no hardware, no ROS.
rosrun sidewalk_evaluation ingest_navigation_run.py --self-test
```

## Evaluating a navigation run

`sidewalk_navigation`'s waypoint runner writes one directory per run:

```
logs/sidewalk_navigation/runs/<run_id>/
    trajectory.tum      the executed path, TUM format
    run_summary.json    metrics with units, plus which map and which route
```

The schema for those two files is documented in a comment block duplicated
**verbatim** in `sidewalk_navigation/scripts/nav_run_io.py` (the writer) and
`scripts/nav_run.py` (the reader). Change one, change the other, in the same
commit; the reader checks `schema_version` and refuses anything it does not
recognise.

```bash
# What is in this run? Prints route, map, planner, pose count and every schema
# complaint, then exits. Works without numpy.
rosrun sidewalk_evaluation ingest_navigation_run.py --describe --run RUNDIR

# One run: closed-loop gap and pose-graph corrections. ATE and RPE are
# explicitly recorded as NOT computed, because there is nothing to compare to.
rosrun sidewalk_evaluation ingest_navigation_run.py --run RUNDIR

# One run against an independent reference recorded during the SAME drive.
# The only mode that yields a defensible error figure.
rosrun sidewalk_evaluation ingest_navigation_run.py \
    --run RUNDIR \
    --ref logs/sidewalk_evaluation/20260801-101500_record/wheel_odom.tum \
    --reference-kind wheel_odometry \
    --name campus_loop_run1

# Repeatability across repeated drives of the identical route.
rosrun sidewalk_evaluation ingest_navigation_run.py \
    --repeatability --run RUN1 --run RUN2 --run RUN3 --name campus_loop_repeat

# Or fold a navigation run into the normal entry point as one more estimate.
rosrun sidewalk_evaluation evaluate_trajectory.py \
    --nav-run RUNDIR --ref RECORD/wheel_odom.tum --reference-kind wheel_odometry
```

### What a navigation trajectory can and cannot measure

It is the robot's **own estimate**, produced by the same localisation system
that was steering it. If that system drifted, the recorded path drifted with it
and agrees with itself perfectly the whole way.

| Question | Answerable from a navigation run? |
|---|---|
| Does the robot repeat the same route consistently? | Yes — `--repeatability` |
| Does planner A follow a shorter or straighter path than planner B? | Yes, with map, route and localisation held fixed |
| Did the estimate close the loop, and how far did it jump when it did? | Yes — the default mode |
| Is the robot actually where it thinks it is? | **No.** Needs a reference that does not share the estimator. |

Repeatability results are labelled `reference_kind: nav_repeat_run` in every
file that carries them, so the caveat travels with the number.

## What it produces

| Metric | Meaning |
|---|---|
| ATE | how far off the whole path is, after least-squares alignment |
| RPE | how wrong each individual step is, needing no alignment |
| Drift | error as a percentage of distance travelled, and degrees per metre |
| Loop-closure gap | accumulated error revealed by returning to a marked start |
| Corrections | how far the estimate jumped when the map re-optimised |
| Relocalization | tracking losses, recovery rate, and time to recover |
| Latency & load | capture-to-pose delay, CPU, GPU, memory, dropped frames |
| Depth | bias, noise, dropout and flatness at 1, 2, 5, 10 and 15 m |

Everything lands in one self-contained `report.html` with the figures embedded,
so it opens offline on any machine.

## Honesty notes

* **No number in this package has been produced from real data yet, and the
  metric code has never been executed.** It was written and reviewed on a
  development PC that has neither numpy nor ROS, so it has been syntax-checked
  and read, not run. The first command on the Jetson must be the `--self-test`
  above. Until it passes, treat every figure this package emits as unvalidated.
* ATE and RPE are **comparisons**. Without a reference trajectory they do not
  exist, and this package refuses to invent them — use `--loop-only` instead,
  which measures drift from the route returning to a physically marked start.
* Comparing two SLAM stacks against each other measures **disagreement**, not
  accuracy. Both can agree and both be wrong.
* Drift is a random walk. One run is one sample. Repeat every route at least
  three times; `compare_slam.py` reports the spread and declines to highlight a
  winner whose margin is smaller than it.
* A navigation run's trajectory is the robot's **own estimate**, not ground
  truth. Comparing two navigation runs measures **repeatability**; it is not an
  accuracy measurement and must never be presented as one. `nav_run.py` refuses
  the same run given twice and refuses two runs that drove different routes,
  because both produce a confident number that means nothing.
* Repeatability ATE associates poses by **elapsed time since each run's start**,
  so a run driven at a different speed shows an apparent error along an
  identical line. The waypoint arrival scatter in `repeatability.json` needs no
  such association and is the cleaner of the two figures. Read both.

All maths is implemented on numpy so it runs on the Jetson as delivered. The
`evo` toolkit is a useful independent cross-check if you can install it, but
nothing here depends on it.

Logs go to `logs/sidewalk_evaluation/`, with one summary line per run in
`logs/RUNLOG.md`.
