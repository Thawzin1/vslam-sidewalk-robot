#!/usr/bin/env python3
"""
ingest_navigation_run.py — evaluate a run driven by `sidewalk_navigation`.

WHAT THIS CLOSES
    `sidewalk_navigation/scripts/waypoint_runner.py` is the project's main
    navigation measurement instrument: it drives a fixed route repeatably and
    measures every leg. This script is the other end of that data path. It
    reads the run directory the runner writes, checks it against the shared
    schema (see `nav_run.py`), and hands the trajectory to the ATE/RPE
    machinery that already exists in `traj_metrics.py`.

    It implements NO metrics of its own except one — the waypoint arrival
    scatter, explained below, which has no counterpart in `traj_metrics.py`
    because no other trajectory in this project arrives at labelled points.

THE THING TO GET RIGHT
    A navigation run's trajectory is the robot's OWN ESTIMATE, produced by the
    same localisation system that was steering it. It is not ground truth, and
    no amount of processing can make it one. Three modes exist, and they answer
    three different questions:

    --loop-only (default)
        One run, no reference. Measures self-consistency: the closed-loop gap
        and the pose-graph corrections. ATE and RPE are explicitly recorded as
        not computed, because there is nothing to compare against.

    --ref FILE --reference-kind KIND
        One run against an INDEPENDENT reference recorded during the same
        drive — wheel odometry from `trajectory_recorder.py` is the realistic
        one. This is the only mode that yields a defensible error figure, and
        it is only ever as good as the reference. The reference kind is
        recorded verbatim in the output.

    --repeatability
        Two or more runs of the SAME route. Measures how consistently the
        system reproduces its own answer. This is real, useful information and
        it is NOT accuracy: two runs can agree to a centimetre while both being
        two metres from where the robot physically was.

    The script refuses the mistakes that look plausible: the same run given
    twice, two runs of different routes, and using a navigation trajectory as
    a reference for anything labelled as accuracy.

WHY REPEATABILITY IS REPORTED TWO WAYS
    ATE between two runs requires associating their poses in time, and two runs
    happen at different wall-clock times and at different speeds. `--rebase-time`
    (on by default in repeatability mode) rewrites both trajectories' timestamps
    to seconds-since-run-start so they can be associated at all — but that means
    a run driven 10% slower shows an apparent error even if it followed exactly
    the same line. The ATE figure therefore CONFLATES spatial deviation with
    speed differences, and that caveat travels with the number.

    So the waypoint arrival scatter is reported alongside it. Every leg records
    the pose the robot believed it arrived at, labelled `loop2.wp3` — the same
    physical waypoint on the same lap in every run. Comparing those poses needs
    no time association whatsoever, and it is the cleaner repeatability figure
    of the two.

TYPICAL USE
    # Verify this script with synthetic data, no hardware, no ROS:
    ./ingest_navigation_run.py --self-test

    # One run, self-consistency only:
    ./ingest_navigation_run.py --run ~/Mitacs\\ Internship/logs/sidewalk_navigation/runs/20260801-101500_waypoint_demo

    # One run against wheel odometry recorded during the same drive:
    ./ingest_navigation_run.py --run RUNDIR \\
        --ref logs/sidewalk_evaluation/20260801-101500_record/wheel_odom.tum \\
        --reference-kind wheel_odometry

    # Repeatability over three drives of the identical route:
    ./ingest_navigation_run.py --repeatability --run RUN1 --run RUN2 --run RUN3

NOT A ROS NODE
    Offline analysis. Needs numpy; needs no ROS.

    `--help` works with neither, because the numpy import is deferred to call
    time rather than module scope.

    `--self-test` DOES need numpy: it exercises the real `traj_metrics`
    alignment and ATE code, and there is no honest way to test that without
    the library that implements it. Without numpy it exits 3 with a diagnosed
    message rather than a traceback. The stdlib-only half of this data path —
    schema parsing, unit checking and the comparability guards — is covered by
    `nav_run.py --self-test`, which does run on a machine with neither.
"""
from __future__ import annotations

import argparse
import json
import math
import shutil
import sys
from pathlib import Path

from eval_common import (MissingDependency, RunLogger, ensure_results_dir,
                         fmt, load_config, timestamp_slug, unquote, write_json)
import nav_run

DEFAULT_CONFIG = Path(__file__).resolve().parent.parent / "config" / "evaluation.yaml"

# Reference kinds that are NOT allowed to be paired with a navigation run's own
# trajectory as the reference. Two runs of the same route are compared with
# nav_run.REFERENCE_KIND and nothing else, because every other value here makes
# an accuracy claim that the data cannot support. Derived from the canonical
# list in eval_common (plus synthetic_ground_truth, which also makes an
# accuracy-shaped claim a nav run cannot back) so the two cannot drift apart.
from eval_common import VERIFIED_TRUTH_KINDS  # noqa: E402

ACCURACY_KINDS = tuple(VERIFIED_TRUTH_KINDS) + ("synthetic_ground_truth",)


def _import_metrics():
    """Import traj_metrics. Raises MissingDependency if numpy is absent.

    Deferred to call time rather than module scope, so that `--help` and
    argument validation still work on a machine without numpy — which is the
    authoring PC, where this file is edited. `main()` catches the exception and
    turns it into a diagnosed exit, so no traceback ever reaches the user.
    """
    import traj_metrics as TM
    return TM


# --------------------------------------------------------------------------- #
# Turning a navigation run into something traj_metrics can consume
# --------------------------------------------------------------------------- #

def load_trajectory(TM, run, log, rebase_time=False):
    """Load one run's TUM file as a TM.Trajectory, or return None.

    With `rebase_time`, timestamps become seconds since that run's first pose.
    That is what makes two runs recorded hours apart associable at all; the
    caller is responsible for saying so in the report, because it silently
    changes what the resulting ATE means.
    """
    if run.trajectory is None:
        log.error(
            "Run %s recorded no trajectory (%s), so nothing about its path can "
            "be measured. Its per-leg metrics are still readable from "
            "run_summary.json."
            % (run.run_id, run.doc.get("trajectory_unavailable_reason")
               or "no reason recorded"))
        return None
    try:
        traj = TM.load_tum(run.trajectory, name=run.label)
    except (FileNotFoundError, ValueError) as exc:
        log.error("Run %s: %s" % (run.run_id, exc))
        return None
    if rebase_time and len(traj):
        traj = TM.Trajectory(traj.stamps - traj.stamps[0], traj.xyz, traj.quat,
                             traj.name)
    return traj


def run_provenance(run):
    """The provenance block copied into every result file this script writes.

    Results outlive the directory they were computed from. A metrics file that
    cannot say which map and which route produced it is a number without an
    experiment attached to it.
    """
    return {
        "source": "sidewalk_navigation.waypoint_runner",
        "run_id": run.run_id,
        "run_name": run.run_name,
        "run_dir": str(run.dir),
        "status": run.status,
        "abort_reason": run.abort_reason,
        "started_unix_s": run.started_unix_s,
        "duration_s": run.duration_s,
        "map_database": run.map_path,
        "map_sha1_16": run.map_signature,
        "route_sha1_16": run.route_signature,
        "route_waypoints": run.waypoint_names,
        "local_planner": run.local_planner,
        "navigation_metrics": run.doc.get("metrics"),
        "schema_issues": run.issues,
    }


# --------------------------------------------------------------------------- #
# Waypoint arrival scatter
#
# The one metric defined here rather than in traj_metrics.py, because it is the
# only one that needs labelled arrival points and no other trajectory in this
# project has any. It is deliberately simple: means and spreads of a handful of
# 2D points. Anything more elaborate belongs in traj_metrics.py.
# --------------------------------------------------------------------------- #

def waypoint_scatter(np, runs):
    """Spread of the believed arrival pose at each waypoint, across runs.

    Needs no temporal association and no trajectory alignment: the leg label
    (`loop2.wp3`) identifies the same physical waypoint on the same lap in
    every run of the same route.

    What it measures, precisely: how consistently the system STOPS AT THE SAME
    PLACE IN ITS OWN COORDINATES. Because every run is expressed in the same
    map's frame, a shared drift affecting all runs equally is invisible here —
    which is the whole point, and also the limit. This is repeatability. It is
    not accuracy.
    """
    per_run = [(run.label, run.arrival_poses()) for run in runs]
    usable = [(label, poses) for label, poses in per_run if poses]
    if len(usable) < 2:
        return {
            "available": False,
            "reason": ("fewer than two runs recorded per-waypoint arrival "
                       "poses. Older runs, and runs where TF was never "
                       "available, do not carry them."),
        }

    common = set(usable[0][1])
    for _label, poses in usable[1:]:
        common &= set(poses)
    if not common:
        return {
            "available": False,
            "reason": ("these runs share no waypoint labels, so no waypoint "
                       "can be compared. That normally means they are not "
                       "runs of the same route, or ran different loop counts."),
        }

    per_waypoint = {}
    all_radial, all_pairwise = [], []
    for label in sorted(common):
        pts = np.asarray([[poses[label][0], poses[label][1]]
                          for _l, poses in usable], dtype=float)
        yaws = np.asarray([poses[label][2] for _l, poses in usable], dtype=float)
        centroid = pts.mean(axis=0)
        radial = np.linalg.norm(pts - centroid, axis=1)
        pairwise = 0.0
        for i in range(len(pts)):
            for j in range(i + 1, len(pts)):
                pairwise = max(pairwise, float(np.linalg.norm(pts[i] - pts[j])))
        # Wrap the yaw spread through the circular mean, so a cluster straddling
        # +pi/-pi does not report a spread of 360 degrees.
        mean_yaw = math.atan2(float(np.mean(np.sin(yaws))),
                              float(np.mean(np.cos(yaws))))
        dyaw = np.degrees(np.arctan2(np.sin(yaws - mean_yaw),
                                     np.cos(yaws - mean_yaw)))
        per_waypoint[label] = {
            "n_runs": int(len(pts)),
            "mean_x_m": float(centroid[0]),
            "mean_y_m": float(centroid[1]),
            "max_radial_m": float(radial.max()),
            "rms_radial_m": float(np.sqrt(np.mean(radial ** 2))),
            "max_pairwise_m": pairwise,
            "max_abs_yaw_dev_deg": float(np.max(np.abs(dyaw))),
        }
        all_radial.append(float(np.sqrt(np.mean(radial ** 2))))
        all_pairwise.append(pairwise)

    return {
        "available": True,
        "n_runs": len(usable),
        "runs": [label for label, _ in usable],
        "n_waypoints_common": len(common),
        "per_waypoint": per_waypoint,
        "mean_rms_radial_m": float(np.mean(all_radial)),
        "max_pairwise_m": float(max(all_pairwise)),
        "units": {"positions": "m", "angles": "deg"},
        "interpretation": (
            "Spread of the pose each run BELIEVED it arrived at, in the map "
            "frame, with no temporal association and no trajectory alignment. "
            "It measures repeatability of the combined localisation-plus-"
            "planner system. Drift shared by every run cancels out and is "
            "invisible here, so this is not, and cannot be made into, an "
            "accuracy figure."),
    }


# --------------------------------------------------------------------------- #
# Modes
# --------------------------------------------------------------------------- #

def eval_self_consistency(TM, ET, run, traj, cfg, log):
    """One run, no reference: closed-loop gap and pose-graph corrections."""
    report = ET.evaluate_loop_only(traj, cfg, log)
    report["navigation_run"] = run_provenance(run)
    report["mode"] = "nav_self_consistency"
    report["unavailable"]["ate"] = (
        "requires a reference trajectory. A navigation run's own trajectory "
        "cannot serve as its own reference: it is the localisation system's "
        "estimate compared with itself, which is zero by construction.")
    return report


def eval_against_reference(TM, run, traj, ref, reference_kind, cfg, log,
                           align_mode, max_diff, time_offset, rpe_deltas,
                           seg_lengths, max_speed):
    """One run against an independent reference. The only accuracy mode."""
    report = TM.evaluate(
        ref, traj,
        align_mode=align_mode,
        max_time_diff=max_diff,
        time_offset=time_offset,
        rpe_deltas=rpe_deltas,
        segment_lengths=seg_lengths,
        max_speed_mps=max_speed,
        reference_kind=reference_kind)
    report["navigation_run"] = run_provenance(run)
    report["mode"] = "nav_vs_reference"
    if reference_kind not in ACCURACY_KINDS:
        report.setdefault("warnings", []).append(
            "The reference kind is '%s'. That is a legitimate reference, but "
            "it has its own error, so the figures below bound the DISAGREEMENT "
            "between the two, not the error of either one alone."
            % reference_kind)
    return report


def eval_repeatability(TM, baseline_run, baseline_traj, run, traj, cfg, log,
                       align_mode, max_diff, rpe_deltas, seg_lengths,
                       max_speed, rebased):
    """One run against another run of the same route."""
    report = TM.evaluate(
        baseline_traj, traj,
        align_mode=align_mode,
        max_time_diff=max_diff,
        rpe_deltas=rpe_deltas,
        segment_lengths=seg_lengths,
        max_speed_mps=max_speed,
        reference_kind=nav_run.REFERENCE_KIND)
    report["navigation_run"] = run_provenance(run)
    report["baseline_run"] = run_provenance(baseline_run)
    report["mode"] = "nav_repeatability"
    report["reference_kind_meaning"] = nav_run.REFERENCE_KIND_MEANING
    report.setdefault("warnings", []).append(
        "REPEATABILITY, NOT ACCURACY. The reference here is another run of the "
        "same route by the same localisation system. A small number means the "
        "system is consistent; it says nothing about whether it is correct.")
    if rebased:
        report.setdefault("warnings", []).append(
            "Both trajectories had their timestamps rebased to seconds since "
            "their own start, because two runs on different days share no "
            "clock. Poses are therefore associated by ELAPSED TIME, so a run "
            "driven at a different speed shows an apparent error even along an "
            "identical line. Read the waypoint arrival scatter in "
            "repeatability.json alongside this: it needs no association and "
            "is not affected.")
    return report


# --------------------------------------------------------------------------- #
# Self-test
# --------------------------------------------------------------------------- #

def run_self_test(log) -> int:
    """Exercise the whole ingestion path on synthetic data with no hardware.

    Two synthetic navigation runs are written to a temporary directory: one
    exact circuit, and one the same circuit with a known perturbation. The test
    checks that the ingest path reads them, that the repeatability figure
    responds to the perturbation, and — most importantly — that the guards
    refuse the comparisons that would produce meaningless numbers.
    """
    import tempfile

    TM = _import_metrics()
    np = TM.np
    log.section("Self-test: synthetic navigation runs")

    checks = []

    def check(desc, ok, detail=""):
        checks.append((desc, bool(ok), detail))
        (log.info if ok else log.error)(
            "[%s] %s  (%s)" % ("PASS" if ok else "FAIL", desc, detail))

    def write_run(root, run_id, offset_m, route_sig="ROUTE_SELFTEST"):
        """A synthetic run directory: circuit trajectory plus a summary."""
        d = root / run_id
        d.mkdir(parents=True, exist_ok=True)
        # Exactly one revolution: duration = circumference / speed. A partial
        # or multiple loop would still exercise the code, but the closed-loop
        # gap it produced would be a large number that means nothing, and a
        # self-test whose output has to be explained away is a bad self-test.
        radius, speed = 6.0, 1.0
        traj = TM.synthetic_loop(duration_s=2.0 * math.pi * radius / speed,
                                 rate_hz=10.0, radius=radius, speed_mps=speed,
                                 name=run_id)
        xyz = traj.xyz + np.array([offset_m, 0.0, 0.0])
        TM.save_tum(d / "trajectory.tum",
                    TM.Trajectory(traj.stamps, xyz, traj.quat, run_id))
        doc = nav_run.synthetic_doc(run_id=run_id, route_sig=route_sig,
                                     offset=offset_m)
        doc["trajectory"] = {
            "file": "trajectory.tum",
            "meta_file": "trajectory.tum.meta.json",
            "format": "TUM",
            "columns": "timestamp tx ty tz qx qy qz qw",
            "quaternion_order": "xyzw_scalar_last",
            "length_unit": "m",
            "time_unit": "s_unix_ros_clock",
            "frame_id": "map",
            "child_frame_id": "base_link",
            "pose_source": "synthetic",
            "sample_rate_hz": 10.0,
            "n_poses": len(traj),
            "n_rejected": 0,
            "n_duplicate_stamps_skipped": 0,
            "first_stamp_unix_s": float(traj.stamps[0]),
            "last_stamp_unix_s": float(traj.stamps[-1]),
            "is_ground_truth": False,
            "reference_kind": nav_run.REFERENCE_KIND,
            "caveat": "synthetic",
        }
        doc["trajectory_unavailable_reason"] = None
        (d / "run_summary.json").write_text(json.dumps(doc, indent=2))
        return d

    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        d_a = write_run(root, "20260801-100000_a", 0.0)
        d_b = write_run(root, "20260801-110000_b", 0.25)
        d_c = write_run(root, "20260801-120000_c", 0.0,
                        route_sig="ROUTE_DIFFERENT")

        run_a = nav_run.load_run(d_a)
        run_b = nav_run.load_run(d_b)
        run_c = nav_run.load_run(d_c)
        check("synthetic navigation runs load through the shared schema",
              run_a.trajectory is not None and run_b.trajectory is not None,
              "%d and %d poses" % (run_a.n_poses, run_b.n_poses))

        traj_a = load_trajectory(TM, run_a, log, rebase_time=True)
        traj_b = load_trajectory(TM, run_b, log, rebase_time=True)
        check("trajectories load into traj_metrics",
              traj_a is not None and len(traj_a) == run_a.n_poses,
              "%d poses" % len(traj_a))
        check("rebasing puts both runs on a common elapsed-time axis",
              abs(float(traj_a.stamps[0])) < 1e-9
              and abs(float(traj_b.stamps[0])) < 1e-9)

        # A run against ITSELF must be exactly zero. That is the number this
        # tool exists to stop anybody reporting, so it is worth demonstrating.
        self_rep = TM.evaluate(traj_a, TM.Trajectory(
            traj_a.stamps, traj_a.xyz, traj_a.quat, "itself"),
            reference_kind=nav_run.REFERENCE_KIND)
        check("a run compared with itself gives exactly zero, which is why the "
              "guards exist",
              self_rep["ate"]["translation"]["rmse"] < 1e-9,
              "%.3e m" % self_rep["ate"]["translation"]["rmse"])

        # A rigid 0.25 m offset is fully removed by SE(3) alignment: two runs
        # of the same route differing only by a translation ARE repeatable.
        rep = eval_repeatability(TM, run_a, traj_a, run_b, traj_b, {}, log,
                                 "se3", 0.05, [(1.0, "m")], [5.0], 2.0, True)
        check("a pure offset between two runs is removed by alignment",
              rep["ate"]["translation"]["rmse"] < 1e-6,
              "%.3e m" % rep["ate"]["translation"]["rmse"])
        check("repeatability output is labelled as such, not as accuracy",
              rep["reference_kind"] == nav_run.REFERENCE_KIND
              and any("NOT ACCURACY" in w for w in rep.get("warnings", [])))

        # The waypoint scatter must see the injected 0.25 m difference. The
        # synthetic leg records place wp1 offset by exactly that amount.
        scatter = waypoint_scatter(np, [run_a, run_b])
        check("waypoint arrival scatter available", scatter.get("available"),
              str(scatter.get("reason", "")))
        if scatter.get("available"):
            check("scatter recovers the injected 0.25 m arrival difference",
                  abs(scatter["max_pairwise_m"] - 0.25) < 1e-6,
                  "%.4f m" % scatter["max_pairwise_m"])

        # Guards.
        fatal, _ = nav_run.check_comparable([run_a, nav_run.load_run(d_a)])
        check("the same run twice is refused", bool(fatal))
        fatal, _ = nav_run.check_comparable([run_a, run_c])
        check("two different routes are refused", bool(fatal))
        fatal, caveats = nav_run.check_comparable([run_a, run_b])
        check("two runs of the same route are accepted", not fatal)

        # Self-consistency mode on a closed circuit must detect the loop.
        import evaluate_trajectory as ET
        traj_a_raw = load_trajectory(TM, run_a, log)
        report = eval_self_consistency(TM, ET, run_a, traj_a_raw, {}, log)
        lc = report.get("loop_closure_gap") or {}
        check("self-consistency mode detects the closed loop",
              bool(lc.get("closed")),
              "gap %s m" % fmt(lc.get("end_to_start_gap_m"), 4))
        check("a perfectly closed synthetic route reports a near-zero gap",
              lc.get("end_to_start_gap_m") is not None
              and lc["end_to_start_gap_m"] < 0.05,
              "%s m over %s m of path" % (fmt(lc.get("end_to_start_gap_m"), 4),
                                          fmt(lc.get("path_length_m"), 1)))
        check("self-consistency mode records that ATE was not computed",
              "ate" in (report.get("unavailable") or {}))
        check("provenance travels into the result",
              (report.get("navigation_run") or {}).get("route_sha1_16")
              == "ROUTE_SELFTEST")

    passed = sum(1 for _d, ok, _x in checks if ok)
    all_ok = passed == len(checks)
    log.summary(
        "Self-test %d/%d checks passed — the navigation-to-evaluation data "
        "path %s behaving as expected on synthetic runs with known injected "
        "differences" % (passed, len(checks), "is" if all_ok else "is NOT"),
        status="OK" if all_ok else "FAIL")
    return 0 if all_ok else 1


# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #

def build_parser():
    ap = argparse.ArgumentParser(
        description="Evaluate one or more navigation runs recorded by "
                    "sidewalk_navigation/waypoint_runner.py, using the metric "
                    "machinery in traj_metrics.py. Runs offline; needs no ROS.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    ap.add_argument("--run", action="append", default=[],
                    help="navigation run directory, or its run_summary.json "
                         "(repeatable)")
    ap.add_argument("--ref", default="",
                    help="independent reference trajectory in TUM format, "
                         "recorded during the same drive")
    ap.add_argument("--reference-kind", default="wheel_odometry",
                    choices=["motion_capture", "rtk_gnss", "closed_loop",
                             "cross_stack", "wheel_odometry",
                             "synthetic_ground_truth", "unspecified"],
                    help="WHAT you are calling truth, when --ref is given. "
                         "Recorded verbatim; set it honestly.")
    ap.add_argument("--repeatability", action="store_true",
                    help="compare two or more runs of the same route against "
                         "each other. Measures consistency, not accuracy.")
    ap.add_argument("--baseline", default="",
                    help="in repeatability mode, the run_id used as the "
                         "reference (default: the earliest run given)")
    ap.add_argument("--rebase-time", dest="rebase_time",
                    action="store_true", default=None,
                    help="rewrite timestamps to seconds since each run's "
                         "start (default: on in repeatability mode, off "
                         "otherwise)")
    ap.add_argument("--no-rebase-time", dest="rebase_time",
                    action="store_false",
                    help="keep absolute timestamps even in repeatability mode")
    ap.add_argument("--align", default="",
                    choices=["", "se3", "sim3", "origin", "none"],
                    help="alignment mode (default from config: se3)")
    ap.add_argument("--max-time-diff", type=float, default=-1.0,
                    help="association tolerance in seconds (default from config)")
    ap.add_argument("--time-offset", type=float, default=0.0,
                    help="seconds to add to estimate timestamps before matching")
    ap.add_argument("--config", default=str(DEFAULT_CONFIG))
    ap.add_argument("--name", default="",
                    help="name for the results directory (default: timestamp)")
    ap.add_argument("--no-report", action="store_true",
                    help="write JSON only; skip the HTML report")
    ap.add_argument("--describe", action="store_true",
                    help="print what each run contains and exit without "
                         "computing anything")
    ap.add_argument("--self-test", action="store_true",
                    help="verify the ingestion path against synthetic "
                         "navigation runs, then exit")
    return ap


def main() -> int:
    args = build_parser().parse_args()

    args.name = unquote(args.name)
    args.ref = unquote(args.ref)
    args.config = unquote(args.config)
    args.baseline = unquote(args.baseline)
    args.run = [unquote(r) for r in args.run]

    run_name = args.name or ("nav_self_test" if args.self_test
                             else "ingest_navigation")

    with RunLogger("sidewalk_evaluation", run_name=run_name) as log:
        try:
            return _run(args, log)
        except MissingDependency as exc:
            # numpy is missing. That is a fully diagnosed refusal, not a crash,
            # and it must not be reported as "Terminated by exception" — which
            # reads exactly like the traceback this package exists to avoid.
            log.error(str(exc))
            log.summary("Cannot compute metrics on this machine: numpy is not "
                        "installed", status="FAIL")
            return 3


def _run(args, log) -> int:
    if args.self_test:
        return run_self_test(log)

    if not args.run:
        log.error(
            "No navigation run was given. Use `--run DIR` at least once.\n"
            "  Navigation runs are written by "
            "sidewalk_navigation/waypoint_runner.py into\n"
            "      logs/sidewalk_navigation/runs/<run_id>/\n"
            "  each containing run_summary.json and trajectory.tum.")
        log.summary("Nothing to ingest: no --run given", status="FAIL")
        return 2

    # -------------------------------------------------------- load runs
    runs, errors = nav_run.load_runs(args.run)
    for err in errors:
        log.error(err)
    if not runs:
        log.summary("No readable navigation run", status="FAIL")
        return 2

    log.section("Navigation runs")
    for run in runs:
        for line in run.describe():
            log.info("  " + line)
        for issue in run.issues:
            log.warn("%s: %s" % (run.run_id, issue))

    if args.describe:
        log.summary("Described %d navigation run(s)" % len(runs),
                    status="OK")
        return 0

    # ---------------------------------------------------- comparability
    fatal, caveats = nav_run.check_comparable(runs)
    for msg in caveats:
        log.warn(msg)
    if fatal:
        for msg in fatal:
            log.error(msg)
        log.summary("These runs cannot be compared with one another",
                    status="FAIL")
        return 2

    if args.repeatability and len(runs) < 2:
        log.error(
            "--repeatability needs at least two runs; %d was given.\n"
            "  Repeatability is the spread of repeated attempts at one "
            "task. With a single attempt there is no spread to measure. "
            "Drive the route again and pass both runs." % len(runs))
        log.summary("Repeatability needs at least two runs", status="FAIL")
        return 2
    if args.ref and args.repeatability:
        log.error(
            "--ref and --repeatability answer different questions and "
            "cannot be combined in one invocation. Run the tool twice.")
        log.summary("Conflicting modes requested", status="FAIL")
        return 2

    TM = _import_metrics()
    np = TM.np
    import evaluate_trajectory as ET

    # ----------------------------------------------------------- config
    try:
        cfg = load_config(args.config)
    except FileNotFoundError as exc:
        log.error(str(exc))
        log.summary("Configuration missing", status="FAIL")
        return 2
    mcfg = cfg.get("metrics", {}) if isinstance(cfg.get("metrics"), dict) else {}
    align_mode = args.align or str(mcfg.get("align_mode", "se3"))
    max_diff = (args.max_time_diff if args.max_time_diff > 0
                else float(mcfg.get("max_time_diff_s", 0.02)))
    rpe_deltas = ET.parse_rpe_deltas(mcfg.get("rpe_deltas"))
    seg_lengths = [float(x) for x in (mcfg.get("drift_segment_lengths_m")
                                      or [5, 10, 20, 40, 80])]
    max_speed = float(mcfg.get("max_speed_mps", 2.0))

    rebase = args.rebase_time
    if rebase is None:
        rebase = bool(args.repeatability)
    if args.repeatability and not rebase:
        log.warn(
            "--no-rebase-time in repeatability mode: the two runs will be "
            "associated on absolute timestamps. Unless they were recorded "
            "from the same bag, they share no clock and almost nothing "
            "will associate.")
    if args.repeatability and rebase and max_diff < 0.05:
        # Association after rebasing is on elapsed time, where the two runs
        # genuinely differ. A 20 ms tolerance chosen for clock skew between
        # machines is far too tight for that and would associate almost
        # nothing.
        log.info(
            "Loosening the association tolerance from %.0f ms to 100 ms "
            "for this repeatability comparison. After rebasing, poses are "
            "matched on elapsed time since each run's start, and two "
            "drives of the same route differ in speed by far more than "
            "%.0f ms. This is a property of the question, not of the "
            "clocks." % (max_diff * 1000, max_diff * 1000))
        max_diff = 0.100

    out_dir = ensure_results_dir(args.name or timestamp_slug())
    log.info("Results directory: %s" % out_dir)

    # ----------------------------------------------------- reference
    ref = None
    if args.ref:
        ref_path = Path(args.ref).expanduser()
        if not ref_path.exists():
            log.error(
                "Reference trajectory not found: %s\n"
                "  Drop --ref to fall back to self-consistency only "
                "(closed-loop gap and pose-graph corrections), which is "
                "honest about ATE and RPE not being computed."
                % ref_path)
            log.summary("Reference trajectory missing", status="FAIL")
            return 2
        for run in runs:
            if run.trajectory and \
                    run.trajectory.resolve() == ref_path.resolve():
                log.error(
                    "The reference %s is the trajectory of run %s, which "
                    "is also being evaluated. Comparing an estimate with "
                    "itself yields zero error and measures nothing."
                    % (ref_path, run.run_id))
                log.summary("Reference is the estimate", status="FAIL")
                return 2
        ref = TM.load_tum(ref_path, name="reference")
        v = TM.validate_trajectory(ref)
        log.info("Reference '%s': %d poses, %.1f m over %.1f s"
                 % (ref.name, v["n_poses"], v["path_length_m"],
                    v["duration_s"]))
        for issue in v["issues"]:
            log.warn("Reference: %s" % issue)
        TM.save_tum(out_dir / "reference.tum", ref,
                    comment="reference kind: %s" % args.reference_kind)

    # -------------------------------------------------------- baseline
    baseline_run = baseline_traj = None
    if args.repeatability:
        ordered = sorted(runs, key=lambda r: (r.started_unix_s or 0.0,
                                              r.run_id))
        if args.baseline:
            match = [r for r in runs if args.baseline in (r.run_id,
                                                          r.label)]
            if not match:
                log.error("--baseline %r matches none of the runs given "
                          "(%s)." % (args.baseline,
                                     ", ".join(r.run_id for r in runs)))
                log.summary("Unknown baseline run", status="FAIL")
                return 2
            baseline_run = match[0]
        else:
            baseline_run = ordered[0]
        baseline_traj = load_trajectory(TM, baseline_run, log,
                                        rebase_time=rebase)
        if baseline_traj is None:
            log.error("The baseline run %s has no usable trajectory, so "
                      "there is nothing to compare the others against."
                      % baseline_run.run_id)
            log.summary("Baseline run unusable", status="FAIL")
            return 2
        log.info("Repeatability baseline: %s. Every other run is measured "
                 "against it; the baseline itself is not evaluated."
                 % baseline_run.run_id)

    # ------------------------------------------------------ evaluation
    summaries, exit_code = [], 0
    index = {"navigation_runs": [], "mode": None, "caveats": caveats}

    for run in runs:
        if args.repeatability and run is baseline_run:
            index["navigation_runs"].append(
                dict(run_provenance(run), role="baseline"))
            continue

        log.section("Evaluating: %s" % run.run_id)
        traj = load_trajectory(TM, run, log,
                               rebase_time=rebase and args.repeatability)
        index["navigation_runs"].append(
            dict(run_provenance(run), role="estimate"))
        if traj is None:
            exit_code = 1
            continue

        v = TM.validate_trajectory(traj)
        log.info("%s: %d poses, %.1f m over %.1f s (%.1f Hz)"
                 % (run.label, v["n_poses"], v["path_length_m"],
                    v["duration_s"], v["mean_rate_hz"]))
        for issue in v["issues"]:
            log.warn("%s: %s" % (run.label, issue))

        if args.repeatability:
            report = eval_repeatability(
                TM, baseline_run, baseline_traj, run, traj, mcfg, log,
                align_mode, max_diff, rpe_deltas, seg_lengths, max_speed,
                rebase)
            index["mode"] = "nav_repeatability"
        elif ref is not None:
            report = eval_against_reference(
                TM, run, traj, ref, args.reference_kind, mcfg, log,
                align_mode, max_diff, args.time_offset, rpe_deltas,
                seg_lengths, max_speed)
            index["mode"] = "nav_vs_reference"
        else:
            report = eval_self_consistency(TM, ET, run, traj, mcfg, log)
            index["mode"] = "nav_self_consistency"

        if "error" in report:
            log.error("%s: %s" % (run.label, report["error"]))
            exit_code = 1
        for w in report.get("warnings", []) or []:
            log.warn("%s: %s" % (run.label, w))

        if "ate" in report:
            h = TM.headline(report)
            log.metric("%s_ate_rmse_m" % run.label,
                       round(h["ate_rmse_m"], 5), "m")
            if h["rpe_trans_rmse_m"] is not None:
                log.metric("%s_rpe_%s_rmse_m" % (run.label, h["rpe_delta"]),
                           round(h["rpe_trans_rmse_m"], 5), "m")
            label = ("deviation from baseline" if args.repeatability
                     else "ATE")
            summaries.append("%s: %s %s m"
                             % (run.label, label, fmt(h["ate_rmse_m"], 3)))
        lc = report.get("loop_closure_gap") or {}
        if lc.get("end_to_start_gap_m") is not None:
            log.metric("%s_loop_gap_m" % run.label,
                       round(lc["end_to_start_gap_m"], 4), "m")
            if not args.repeatability and ref is None:
                summaries.append(
                    "%s: loop gap %s m over %s m"
                    % (run.label, fmt(lc["end_to_start_gap_m"], 3),
                       fmt(lc.get("path_length_m"), 1)))
        cx = report.get("corrections") or {}
        if cx.get("n_corrections") is not None:
            log.metric("%s_corrections" % run.label, cx["n_corrections"])

        # Artefacts, named exactly as evaluate_trajectory names them so
        # generate_report.py and compare_slam.py read this directory with
        # no change at all.
        if report.get("_aligned") is not None:
            TM.save_tum(out_dir / ("aligned_%s.tum" % run.label),
                        report["_aligned"],
                        comment="navigation run %s; mode %s"
                                % (run.run_id, report["mode"]))
        elif run.trajectory is not None:
            shutil.copyfile(run.trajectory,
                            out_dir / ("raw_%s.tum" % run.label))
        ET.write_plotdata(out_dir, run.label,
                          baseline_traj if args.repeatability else ref,
                          traj, report)
        write_json(out_dir / ("eval_%s.json" % run.label),
                   ET.strip_private(report))

    # --------------------------------------------- repeatability extras
    if args.repeatability:
        log.section("Waypoint arrival scatter")
        scatter = waypoint_scatter(np, runs)
        if scatter.get("available"):
            log.metric("waypoint_max_pairwise_m",
                       round(scatter["max_pairwise_m"], 4), "m")
            log.metric("waypoint_mean_rms_radial_m",
                       round(scatter["mean_rms_radial_m"], 4), "m")
            log.info(
                "Across %d run(s) and %d shared waypoint(s), the two "
                "arrivals furthest apart at any single waypoint differ by "
                "%.3f m. This needs no temporal association and no "
                "alignment, so unlike the ATE above it is unaffected by "
                "the two runs being driven at different speeds."
                % (scatter["n_runs"], scatter["n_waypoints_common"],
                   scatter["max_pairwise_m"]))
            for label, entry in sorted(scatter["per_waypoint"].items()):
                log.info("  %-14s max pairwise %6.3f m, max yaw dev "
                         "%6.2f deg" % (label, entry["max_pairwise_m"],
                                        entry["max_abs_yaw_dev_deg"]))
            summaries.append("waypoint scatter %s m"
                             % fmt(scatter["max_pairwise_m"], 3))
        else:
            log.warn("Waypoint arrival scatter unavailable: %s"
                     % scatter.get("reason"))
        write_json(out_dir / "repeatability.json", {
            "mode": "nav_repeatability",
            "baseline_run": baseline_run.run_id,
            "runs": [r.run_id for r in runs],
            "route_sha1_16": baseline_run.route_signature,
            "time_rebased": rebase,
            "reference_kind": nav_run.REFERENCE_KIND,
            "reference_kind_meaning": nav_run.REFERENCE_KIND_MEANING,
            "waypoint_arrival_scatter": scatter,
            "caveats": caveats,
        })

    write_json(out_dir / "navigation_runs.json", index)

    # ----------------------------------------------------------- report
    if not args.no_report:
        try:
            import generate_report
            doc = generate_report.build_report(
                out_dir, title=args.name or "Navigation run %s" % out_dir.name,
                log=log)
            (out_dir / "report.html").write_text(doc)
            log.info("Report: %s" % (out_dir / "report.html"))
        except MissingDependency as exc:
            log.warn("Report not generated: %s" % exc)
        except Exception as exc:                           # noqa: BLE001
            log.error(
                "Report generation failed (%s: %s). The JSON metrics in %s "
                "are complete and unaffected; re-run "
                "`generate_report.py --results %s` once the cause is fixed."
                % (type(exc).__name__, exc, out_dir, out_dir))
            exit_code = max(exit_code, 1)

    text = "; ".join(summaries) if summaries else \
        "ingested %d navigation run(s)" % len(runs)
    log.summary("%s  [%s]" % (text, out_dir.name),
                status="OK" if exit_code == 0 else "FAIL")
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
