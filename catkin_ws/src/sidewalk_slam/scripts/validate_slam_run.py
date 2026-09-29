#!/usr/bin/env python3
"""
validate_slam_run.py — check that a SLAM run produced something physically
believable, BEFORE anyone spends time analysing it or trusts it on hardware.

WHY THIS EXISTS
    Phase 3-4 says: validate on recorded data before the live robot is trusted.
    This is that gate.

    A SLAM system almost never announces its own failure. It produces a
    trajectory either way. A diverged run looks, on a plot, like a slightly
    strange but plausible path. The characteristic signatures are all numeric:

      * a 4 m jump between two poses 50 ms apart - that is 80 m/s, and this
        robot walks at 1.5 m/s;
      * timestamps that go backwards, which means two estimators are writing to
        the same topic, or the bag was played twice;
      * quaternions that have drifted off the unit sphere;
      * a trajectory that covers 30 s of a 300 s recording, because tracking
        was lost at the first corner and never recovered.

    None of those are visible by eye. All of them are trivial to test for. So
    they are tested for, every run, automatically, and the run is marked FAIL if
    any of them fires.

    This script never fixes anything and never touches hardware. It reads files.

USAGE
    rosrun sidewalk_slam validate_slam_run.py --run-id 20260720-140322_compare
    python3 validate_slam_run.py --trajectory /path/to/one.tum --max-speed 3.0
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from slam_common import (SlamExit, STACKS, die, make_logger, quat_angle_between,  # noqa: E402
                         quat_norm, read_tum, run_cmd, slam_log_dir,
                         trajectory_dir, tum_path)


def parse_args(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--run-id", help="validate every trajectory of one run")
    src.add_argument("--trajectory", help="validate a single TUM file")

    ap.add_argument("--max-speed", type=float, default=3.0,
                    help="largest believable translational speed in m/s. A "
                         "Clearpath-class differential-drive platform on a "
                         "sidewalk does about 1.0-1.5 m/s; 3.0 leaves headroom "
                         "for a genuinely fast run without admitting teleports. "
                         "(default: 3.0)")
    ap.add_argument("--max-angular-rate", type=float, default=4.0,
                    help="largest believable turn rate in rad/s. Spinning in "
                         "place is about 1 rad/s; 4.0 rad/s is 230 deg/s, "
                         "which this platform cannot do. (default: 4.0)")
    ap.add_argument("--min-rate", type=float, default=1.0,
                    help="warn if the trajectory averages fewer poses per "
                         "second than this (default: 1.0)")
    ap.add_argument("--expected-duration", type=float, default=0.0,
                    help="length of the input recording in seconds. If given, "
                         "coverage is checked against it.")
    ap.add_argument("--min-coverage", type=float, default=0.90,
                    help="fraction of --expected-duration a trajectory must "
                         "span (default: 0.90)")
    ap.add_argument("--closed-loop", action="store_true",
                    help="the robot returned to its starting point. Enables a "
                         "loop-closure error check.")
    ap.add_argument("--loop-tolerance", type=float, default=1.0,
                    help="with --closed-loop, warn if start and end differ by "
                         "more than this many metres (default: 1.0)")
    ap.add_argument("--database", default=None,
                    help="RTAB-Map .db to interrogate for loop closure counts")
    return ap.parse_args(argv)


# --------------------------------------------------------------------------- #

class Check:
    """One named test with a verdict and a sentence explaining it."""

    def __init__(self, name, ok, detail, fatal=True):
        self.name = name
        self.ok = ok
        self.detail = detail
        self.fatal = fatal

    def __repr__(self):
        return f"<{'PASS' if self.ok else 'FAIL'} {self.name}>"


def validate_trajectory(log, path, args):
    """Run every check against one TUM file. Returns (checks, stats)."""
    checks, stats = [], {}
    poses, problems = read_tum(path)

    checks.append(Check("file_parses", not problems,
                        f"{len(problems)} unparseable line(s)"
                        + (": " + "; ".join(problems[:5]) if problems else "")))
    checks.append(Check("has_poses", bool(poses),
                        f"{len(poses)} poses"))
    stats["poses"] = len(poses)
    if not poses:
        return checks, stats

    t0, t1 = poses[0][0], poses[-1][0]
    duration = t1 - t0
    stats["duration_s"] = round(duration, 3)
    stats["first_stamp"] = t0
    stats["last_stamp"] = t1

    # --- timestamps -------------------------------------------------------
    backwards = duplicated = 0
    biggest_gap = 0.0
    gap_at = None
    for a, b in zip(poses, poses[1:]):
        dt = b[0] - a[0]
        if dt < 0:
            backwards += 1
        elif dt == 0:
            duplicated += 1
        elif dt > biggest_gap:
            biggest_gap = dt
            gap_at = a[0]
    stats["largest_time_gap_s"] = round(biggest_gap, 4)

    checks.append(Check(
        "timestamps_monotonic", backwards == 0,
        f"{backwards} pose(s) go backwards in time. Two publishers on one "
        f"topic, or a recording replayed twice into the same file."))
    checks.append(Check(
        "timestamps_unique", duplicated == 0,
        f"{duplicated} pose(s) share a timestamp with the previous one. "
        f"Harmless for plotting, fatal for any per-pose error metric.",
        fatal=False))

    if duration > 0:
        rate = (len(poses) - 1) / duration
        stats["mean_rate_hz"] = round(rate, 3)
        checks.append(Check(
            "pose_rate_reasonable", rate >= args.min_rate,
            f"mean {rate:.2f} Hz (threshold {args.min_rate:.2f} Hz). A very low "
            f"rate usually means the estimator was only updating on keyframes, "
            f"or was losing tracking repeatedly.", fatal=False))

    # A gap much larger than the sampling interval is a tracking dropout: the
    # system stopped producing poses and then resumed somewhere else.
    if "mean_rate_hz" in stats and stats["mean_rate_hz"] > 0:
        nominal = 1.0 / stats["mean_rate_hz"]
        checks.append(Check(
            "no_long_dropout", biggest_gap < max(2.0, 20 * nominal),
            f"largest gap {biggest_gap:.2f}s at t={gap_at} (nominal interval "
            f"{nominal:.3f}s). A gap this size means tracking was lost and "
            f"re-acquired - everything after it may be in a different frame.",
            fatal=False))

    # --- geometry ---------------------------------------------------------
    bad_quat = 0
    max_speed = 0.0
    speed_at = None
    max_omega = 0.0
    path_length = 0.0
    nan_count = 0

    for i, p in enumerate(poses):
        t, x, y, z, qx, qy, qz, qw = p
        if not all(math.isfinite(v) for v in p):
            nan_count += 1
            continue
        if abs(quat_norm(qx, qy, qz, qw) - 1.0) > 1e-3:
            bad_quat += 1
        if i == 0:
            continue
        pt, px, py, pz = poses[i - 1][0], poses[i - 1][1], poses[i - 1][2], poses[i - 1][3]
        dt = t - pt
        d = math.sqrt((x - px) ** 2 + (y - py) ** 2 + (z - pz) ** 2)
        path_length += d
        if dt > 1e-6:
            v = d / dt
            if v > max_speed:
                max_speed, speed_at = v, t
            w = quat_angle_between((qx, qy, qz, qw), poses[i - 1][4:8]) / dt
            max_omega = max(max_omega, w)

    stats["path_length_m"] = round(path_length, 3)
    stats["max_speed_mps"] = round(max_speed, 3)
    stats["max_angular_rate_radps"] = round(max_omega, 3)

    checks.append(Check("no_nan_poses", nan_count == 0,
                        f"{nan_count} pose(s) contain NaN or infinity"))
    checks.append(Check("quaternions_unit", bad_quat == 0,
                        f"{bad_quat} quaternion(s) are not unit length. A "
                        f"non-unit quaternion silently scales every rotated "
                        f"vector."))
    checks.append(Check(
        "speed_plausible", max_speed <= args.max_speed,
        f"peak {max_speed:.2f} m/s at t={speed_at} (limit {args.max_speed:.2f} "
        f"m/s). A jump above the platform's top speed is not motion, it is the "
        f"estimator relocalising into the wrong place."))
    checks.append(Check(
        "angular_rate_plausible", max_omega <= args.max_angular_rate,
        f"peak {max_omega:.2f} rad/s (limit {args.max_angular_rate:.2f} rad/s). "
        f"An impossible rotation rate is the attitude equivalent of a teleport.",
        fatal=False))

    # --- coverage ---------------------------------------------------------
    if args.expected_duration > 0:
        coverage = duration / args.expected_duration
        stats["coverage"] = round(coverage, 3)
        checks.append(Check(
            "covers_recording", coverage >= args.min_coverage,
            f"the trajectory spans {duration:.1f}s of a "
            f"{args.expected_duration:.1f}s recording ({coverage * 100:.0f}%). "
            f"Short coverage means the system stopped tracking partway through "
            f"and never recovered."))

    # --- loop closure -----------------------------------------------------
    if args.closed_loop:
        sx, sy, sz = poses[0][1], poses[0][2], poses[0][3]
        ex, ey, ez = poses[-1][1], poses[-1][2], poses[-1][3]
        misclose = math.sqrt((ex - sx) ** 2 + (ey - sy) ** 2 + (ez - sz) ** 2)
        stats["loop_misclosure_m"] = round(misclose, 3)
        rel = (misclose / path_length * 100.0) if path_length > 0 else 0.0
        stats["loop_misclosure_pct_of_path"] = round(rel, 3)
        checks.append(Check(
            "loop_closes", misclose <= args.loop_tolerance,
            f"start and end differ by {misclose:.2f} m after a {path_length:.1f} m "
            f"path ({rel:.2f}% of distance travelled; tolerance "
            f"{args.loop_tolerance:.2f} m). This is the surveying misclosure of "
            f"the run: if the robot really returned to its starting point, this "
            f"number IS the accumulated drift.", fatal=False))

    return checks, stats


def inspect_database(log, db_path):
    """Ask the RTAB-Map CLI how many loop closures the run actually found.

    Loop closures are the whole point of RTAB-Map. A mapping run that found
    zero of them is a run of visual odometry with extra steps, and the resulting
    map has had no drift corrected out of it at all.
    """
    db = Path(db_path).expanduser()
    if not db.exists():
        log.warn(f"RTAB-Map database not found: {db}")
        return None
    rc, out, err = run_cmd(["rtabmap-info", str(db)], timeout=120)
    if rc == -1:
        log.warn("rtabmap-info is not on PATH, so loop closures cannot be "
                 "counted. It ships with the RTAB-Map core install; if it is "
                 "missing, the standalone tools were not installed.")
        return None
    if rc != 0:
        log.warn(f"rtabmap-info failed: {(err or out).strip()[:400]}")
        return None
    summary = {}
    for line in out.splitlines():
        if ":" in line:
            k, _, v = line.partition(":")
            k = k.strip().lower().replace(" ", "_")
            if any(word in k for word in
                   ("loop", "node", "word", "database", "session")):
                summary[k] = v.strip()
    for k, v in summary.items():
        log.info(f"  db {k}: {v}")
    return summary


def report(log, name, checks, stats):
    log.section(f"Trajectory: {name}")
    for k, v in stats.items():
        log.metric(f"{name}.{k}", v)
    failed_fatal, failed_soft = [], []
    for c in checks:
        if c.ok:
            log.info(f"  PASS  {c.name}")
        elif c.fatal:
            failed_fatal.append(c)
            log.error(f"  FAIL  {c.name}: {c.detail}")
        else:
            failed_soft.append(c)
            log.warn(f"  WARN  {c.name}: {c.detail}")
    return failed_fatal, failed_soft


def cross_check(log, per_stack):
    """Compare the stacks against each other for gross inconsistency."""
    usable = {k: v for k, v in per_stack.items() if v.get("poses")}
    if len(usable) < 2:
        log.warn("fewer than two stacks produced a usable trajectory, so no "
                 "cross-comparison is possible.")
        return

    log.section("Cross-stack consistency")
    spans = {k: (v.get("first_stamp"), v.get("last_stamp"))
             for k, v in usable.items()}
    starts = [s for s, _ in spans.values() if s is not None]
    ends = [e for _, e in spans.values() if e is not None]
    if starts and ends:
        spread = max(starts) - min(starts)
        log.metric("start_time_spread", round(spread, 3), "s")
        if spread > 5.0:
            log.warn(f"the stacks start {spread:.1f}s apart. Over the same "
                     f"recording with simulated time they should start within "
                     f"a second or two of each other. A large spread means one "
                     f"stack took much longer to initialise, and it is being "
                     f"compared over a different portion of the route.")

    lengths = {k: v.get("path_length_m", 0.0) for k, v in usable.items()}
    for k, v in lengths.items():
        log.metric(f"{k}.path_length", v, "m")
    vals = [v for v in lengths.values() if v > 0]
    if len(vals) >= 2:
        spread = (max(vals) - min(vals)) / max(vals)
        log.metric("path_length_spread", round(spread * 100, 1), "%")
        if spread > 0.25:
            log.warn(f"the stacks disagree about how far the robot travelled by "
                     f"{spread * 100:.0f}%. Since they saw identical data, one "
                     f"of them has a scale error. For a stereo system that "
                     f"points at a wrong baseline in its configuration - check "
                     f"Camera.bf in the ORB-SLAM3 settings against the value "
                     f"make_orbslam3_config.py measured.")


def main(argv=None):
    args = parse_args(argv)
    run_name = ("validate_" + (args.run_id or Path(args.trajectory).stem))
    with make_logger(run_name) as log:

        targets = []
        if args.trajectory:
            targets.append((Path(args.trajectory).stem,
                            Path(args.trajectory).expanduser()))
        else:
            d = trajectory_dir(args.run_id)
            for stack in STACKS:
                p = tum_path(args.run_id, stack)
                if p.exists():
                    targets.append((stack, p))
            if not targets:
                die(log, f"no trajectory files found in {d}",
                    "Either the run id is wrong, or slam_runner.py never got "
                    f"far enough to write anything. Check "
                    f"{slam_log_dir()} for the runner's own log.")

        all_fatal, all_soft, per_stack = [], [], {}
        for name, path in targets:
            if not path.exists():
                log.error(f"missing trajectory: {path}")
                all_fatal.append(Check(f"{name}.exists", False, "file missing"))
                continue
            checks, stats = validate_trajectory(log, path, args)
            per_stack[name] = stats
            f, s = report(log, name, checks, stats)
            all_fatal.extend(f)
            all_soft.extend(s)

            meta = path.with_suffix(path.suffix + ".meta.json")
            if meta.exists():
                try:
                    m = json.loads(meta.read_text())
                    log.info(f"  provenance: {m.get('source')} "
                             f"(frame {m.get('frame_id') or 'unknown'})")
                except (OSError, ValueError):
                    log.warn(f"  sidecar {meta.name} is unreadable")
            else:
                log.warn(f"  no .meta.json beside {path.name}: this trajectory "
                         f"has no recorded provenance, so it cannot be "
                         f"interpreted six months from now.")

        if args.database:
            log.section("RTAB-Map database")
            inspect_database(log, args.database)

        if len(per_stack) > 1:
            cross_check(log, per_stack)

        log.section("Verdict")
        log.metric("checks_failed_fatal", len(all_fatal))
        log.metric("checks_failed_soft", len(all_soft))

        if all_fatal:
            names = ", ".join(sorted({c.name for c in all_fatal}))
            log.error(f"{len(all_fatal)} fatal check(s) failed: {names}")
            log.error("This run must NOT be used as evidence, and the system "
                      "must NOT be trusted on live hardware until the cause is "
                      "found and a clean recorded run passes.")
            log.summary(f"VALIDATION FAILED - {len(all_fatal)} fatal, "
                        f"{len(all_soft)} advisory", status="FAIL")
            return 1

        if all_soft:
            log.warn(f"{len(all_soft)} advisory check(s) failed. The run is "
                     f"usable but read the warnings before quoting any number "
                     f"from it.")
            log.summary(f"Validation passed with {len(all_soft)} advisory "
                        f"warning(s) across {len(per_stack)} trajectory(ies)",
                        status="WARN")
            return 0

        log.summary(f"Validation passed cleanly for {len(per_stack)} "
                    f"trajectory(ies)", status="OK")
        return 0


if __name__ == "__main__":
    # SlamExit is how every tool in this package reports an already-diagnosed
    # refusal. The reason has been printed and logged by the time it reaches
    # here; all that is left is to hand the shell a non-zero status.
    try:
        sys.exit(main())
    except SlamExit as _exit:
        sys.exit(_exit.code)
