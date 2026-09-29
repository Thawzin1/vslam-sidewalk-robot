#!/usr/bin/env python3
"""
evaluate_trajectory.py — The main entry point for Phase 5 analysis.

WHAT IT DOES
    Takes one reference trajectory and one or more estimated trajectories, in
    TUM format, and produces every number this project reports about
    localization accuracy: ATE, RPE at several separations, drift per distance,
    loop-closure gap, pose-graph correction magnitude, and — if a status log
    was recorded — tracking losses and relocalization behaviour. It then writes
    the results as JSON, the aligned trajectories back out as TUM files, and a
    single self-contained HTML report.

WHY IT IS AN OFFLINE TOOL AND NOT A ROS NODE
    Evaluation must be repeatable. If the numbers were computed live, changing
    a metric definition would mean re-driving the robot. Instead, the robot run
    produces trajectory files once, and this tool can be re-run against those
    files as many times as the analysis is refined. It imports no ROS at all,
    so it also runs on your own computer with the files copied across — which is how a
    supervisor will actually look at the results.

TYPICAL USE
    # Verify the evaluation code itself, with no hardware and no data:
    ./evaluate_trajectory.py --self-test

    # Evaluate one stack against wheel odometry:
    ./evaluate_trajectory.py \\
        --ref logs/sidewalk_evaluation/run12/odom.tum \\
        --est rtabmap=logs/sidewalk_evaluation/run12/rtabmap.tum \\
        --reference-kind wheel_odometry

    # Evaluate all three stacks from one recorded run at once:
    ./evaluate_trajectory.py --ref REF.tum \\
        --est rtabmap=rtabmap.tum \\
        --est zed=zed_tracking.tum \\
        --est orbslam3=orbslam3.tum \\
        --reference-kind closed_loop --name route_a_run3

    # No reference at all — measure drift from the fact that the route closed:
    ./evaluate_trajectory.py --loop-only --est rtabmap=rtabmap.tum
"""
from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

from eval_common import (REFERENCE_KINDS, UNQUOTABLE_KINDS,
                         MissingDependency, ProvenanceError, RunLogger,
                         ensure_results_dir, fmt, load_config,
                         timestamp_slug, write_json)

# Fail on a missing numpy with a sentence, not a stack trace. Everything below
# this point assumes the numerical stack exists.
try:
    import traj_metrics as TM
except MissingDependency as _exc:      # pragma: no cover - environment-dependent
    print(f"\nERROR: {_exc}\n", file=sys.stderr)
    sys.exit(3)

np = TM.np

DEFAULT_CONFIG = Path(__file__).resolve().parent.parent / "config" / "evaluation.yaml"


# --------------------------------------------------------------------------- #
# Argument plumbing
# --------------------------------------------------------------------------- #

def parse_named(items, what: str) -> dict:
    """Parse repeated `name=path` arguments, tolerating a bare path.

    A bare path gets its filename stem as the name, because typing
    `--est rtabmap=rtabmap.tum` twice a day gets old and the stem is almost
    always the right label anyway.
    """
    out = {}
    for raw in items or []:
        if "=" in raw:
            name, _, path = raw.partition("=")
            name = name.strip()
        else:
            path = raw
            name = Path(raw).stem
        if name in out:
            raise SystemExit(
                f"Duplicate {what} name '{name}'. Each {what} needs a unique "
                f"label because it becomes a column heading in the report. Use "
                f"`--{what} label=path` to disambiguate.")
        out[name] = Path(path).expanduser()
    return out


def expand_nav_runs(specs, log):
    """Turn navigation run directories into `label=path` estimate arguments.

    This is the whole integration with `sidewalk_navigation` from this side:
    a navigation run is read by `nav_run.py`, checked against the shared
    schema, and its TUM trajectory joins the estimate list like any other. No
    metric code changes, because a navigation trajectory is a trajectory.

    Returns (specs, fatal). `fatal` is True if any run could not be read; the
    caller stops, because silently evaluating three of four requested runs
    produces a comparison table with a missing column and no explanation.
    """
    import nav_run

    out, fatal = [], False
    runs, errors = nav_run.load_runs(specs)
    for err in errors:
        log.error(err)
        fatal = True
    for run in runs:
        for issue in run.issues:
            log.warn(f"{run.run_id}: {issue}")
        if run.trajectory is None:
            log.error(
                f"Navigation run {run.run_id} recorded no trajectory "
                f"({run.doc.get('trajectory_unavailable_reason') or 'no reason recorded'}), "
                f"so there is nothing here to evaluate.")
            fatal = True
            continue
        log.info(f"Navigation run {run.run_id}: {run.n_poses} poses, route "
                 f"{run.route_signature}, map {run.map_signature}, "
                 f"planner {run.local_planner or 'not recorded'}")
        out.append(f"{run.label}={run.trajectory}")

    if out:
        log.warn(
            "A navigation run's trajectory is the robot's OWN ESTIMATE, "
            "produced by the localisation system that was steering it. It is "
            "an ESTIMATE here, never the reference. To compare navigation runs "
            "with each other — which measures repeatability, not accuracy — "
            "use `ingest_navigation_run.py --repeatability`, which applies the "
            "route and map checks this entry point does not.")
    return out, fatal


def parse_rpe_deltas(spec) -> list:
    """Turn a config list like ["1m", "5m", "10m", "1s"] into (value, unit) pairs."""
    out = []
    for s in spec or []:
        s = str(s).strip().lower()
        unit = "m"
        if s.endswith("frames"):
            unit, s = "frames", s[:-6]
        elif s.endswith("s"):
            unit, s = "s", s[:-1]
        elif s.endswith("m"):
            unit, s = "m", s[:-1]
        try:
            out.append((float(s), unit))
        except ValueError:
            continue
    return out or [(1.0, "m"), (5.0, "m"), (10.0, "m"), (1.0, "s")]


def read_sidecar_kind(tum_path: Path) -> str:
    """Return the reference_kind declared in a .tum file's .meta.json sidecar.

    trajectory_recorder.py writes `<file>.tum.meta.json` beside every
    trajectory. When that sidecar carries a `reference_kind`, it was set at
    RECORD time by the thing that knew what the data was — worth more than
    any flag typed at evaluation time. Returns "" when there is no sidecar,
    no declared kind, or the sidecar is unreadable (never raises: an old or
    foreign .tum without a sidecar is a normal input, not an error).
    """
    import json
    sidecar = Path(str(tum_path) + ".meta.json")
    try:
        meta = json.loads(sidecar.read_text())
    except (OSError, ValueError):
        return ""
    kind = str(meta.get("reference_kind", "") or "")
    return kind if kind in REFERENCE_KINDS else ""


# --------------------------------------------------------------------------- #
# Loop-only mode: no reference trajectory exists
# --------------------------------------------------------------------------- #

def evaluate_loop_only(est: TM.Trajectory, cfg: dict, log) -> dict:
    """Everything that can be measured with NO reference trajectory at all.

    This is the honest fallback, and it is the mode most runs will actually
    use, because there is no motion-capture rig on a sidewalk.

    WHAT IS AND IS NOT AVAILABLE HERE
        Available:   drift from the closed-loop gap, pose-graph correction
                     magnitude and count, self-consistency checks, tracking
                     reliability if a status log exists, path length, rate.
        NOT available: ATE and RPE. Both are comparisons, and with nothing to
                     compare against they simply do not exist. This function
                     records that explicitly rather than emitting a zero, which
                     would be worse than no number at all.
    """
    log.warn("Running in loop-only mode: no reference trajectory was supplied, "
             "so ATE and RPE cannot be computed. Drift is derived from the "
             "closed-loop return to start.")
    report = {
        "reference_name": None,
        "estimate_name": est.name,
        "reference_kind": "closed_loop",
        "mode": "loop_only",
        "estimate_validation": TM.validate_trajectory(est),
        "loop_closure_gap": TM.loop_closure_gap(
            est,
            return_radius=float(cfg.get("loop_return_radius_m", 1.0)),
            min_path_before_return=float(cfg.get("loop_min_path_m", 10.0))),
        "corrections": TM.detect_correction_jumps(
            est, max_speed_mps=float(cfg.get("max_speed_mps", 2.0))),
        "unavailable": {
            "ate": ("requires a reference trajectory; none was supplied. The "
                    "closed-loop gap below is the substitute and constrains "
                    "only the endpoint, not the whole path."),
            "rpe": ("requires a reference trajectory to compare increments "
                    "against; none was supplied."),
        },
    }
    return report


# --------------------------------------------------------------------------- #
# Artefacts written for the report
# --------------------------------------------------------------------------- #

def write_plotdata(out_dir: Path, name: str, ref, est, report: dict) -> None:
    """Save the arrays the figures need, decoupled from the metric objects.

    The report generator is a separate program and must be able to redraw every
    figure from files alone, months later, with the original Python objects long
    gone. So the exact arrays that were plotted are persisted, not recomputed.
    """
    data = {"name": name}
    # Prefer the aligned trajectory. In loop-only mode nothing was aligned —
    # there was nothing to align to — so the raw estimate is plotted instead.
    # For a closed loop the SHAPE is still the interesting thing, and the map
    # view is often the fastest way to see that a route did not close.
    traj = report.get("_aligned")
    if traj is None:
        traj = est
        data["aligned"] = False
    else:
        data["aligned"] = True
    if traj is not None:
        data["est_xy"] = [traj.xyz[:, 0].tolist(), traj.xyz[:, 1].tolist()]
        data["est_z"] = traj.xyz[:, 2].tolist()
        data["est_t"] = (traj.stamps - traj.stamps[0]).tolist()
    if ref is not None:
        data["ref_xy"] = [ref.xyz[:, 0].tolist(), ref.xyz[:, 1].tolist()]
        data["ref_t"] = (ref.stamps - ref.stamps[0]).tolist()
    per = (report.get("ate") or {}).get("per_pose") or {}
    if per:
        data["err_t"] = per.get("stamps", [])
        data["err_m"] = per.get("trans_err_m", [])
        data["err_deg"] = per.get("rot_err_deg", [])
    write_json(out_dir / f"plotdata_{name}.json", data)


def strip_private(report: dict) -> dict:
    """Remove the non-serialisable working objects before writing JSON.

    Also drops the per-pose error arrays from the metrics file: they can be
    tens of thousands of floats, they already live in the plotdata file, and a
    metrics file a human cannot open in a text editor is a metrics file nobody
    checks.
    """
    out = {k: v for k, v in report.items() if not k.startswith("_")}
    if "ate" in out and isinstance(out["ate"], dict):
        ate = dict(out["ate"])
        ate.pop("per_pose", None)
        out["ate"] = ate
    return out


# --------------------------------------------------------------------------- #
# Self-test
# --------------------------------------------------------------------------- #

def run_self_test(log) -> int:
    """Prove the evaluation pipeline works, end to end, with no hardware.

    This is not a unit test for its own sake. It is the answer to "how do I
    know the ATE number is right?" — we inject a known error into a known
    trajectory and check that the tool recovers it. Run this after any change
    to `traj_metrics.py`, and run it on the Jetson once after provisioning to
    confirm the environment is sane before the first real experiment.
    """
    log.section("Self-test: synthetic trajectory with known injected error")

    ref = TM.synthetic_loop(duration_s=180.0, rate_hz=30.0, radius=10.0,
                            speed_mps=1.2, name="synthetic_truth")
    log.info(f"Reference: {len(ref)} poses, {TM.path_length(ref):.1f} m, "
             f"{ref.duration:.1f} s, closed circular route")

    checks = []

    # 1. Identity must give exactly zero. If this fails, nothing else matters.
    r = TM.evaluate(ref, TM.Trajectory(ref.stamps, ref.xyz, ref.quat, "identity"),
                    reference_kind="synthetic_ground_truth")
    v = r["ate"]["translation"]["rmse"]
    checks.append(("identity ATE is zero", v < 1e-9, f"{v:.3e} m"))
    log.metric("selftest_identity_ate_m", float(v), "m")

    # 2. A pure rigid offset must be completely removed by alignment. This is
    #    the test that catches a broken Umeyama, including the reflection bug.
    a = np.radians(64.0)
    R = np.array([[np.cos(a), -np.sin(a), 0.0],
                  [np.sin(a), np.cos(a), 0.0], [0.0, 0.0, 1.0]])
    moved = TM.Trajectory(ref.stamps, ref.xyz, ref.quat, "rigid").transformed(
        R, np.array([250.0, -80.0, 7.5]))
    r = TM.evaluate(ref, moved, reference_kind="synthetic_ground_truth")
    v = r["ate"]["translation"]["rmse"]
    checks.append(("rigid offset removed by alignment", v < 1e-6, f"{v:.3e} m"))
    log.metric("selftest_rigid_ate_m", float(v), "m")

    # 3. A known scale error must be recovered by Sim(3) alignment to 5 decimals.
    #    Direction matters: the estimate is built 3.7% TOO BIG, and Umeyama
    #    returns the factor that shrinks it back onto the reference, i.e.
    #    1/1.037 = 0.964320 — not 1.037. The human-facing figure is
    #    `estimate_oversize_pct`, which reads back as +3.7%.
    scaled = TM.Trajectory(ref.stamps, ref.xyz * 1.037, ref.quat, "scaled")
    r = TM.evaluate(ref, scaled, align_mode="sim3",
                    reference_kind="synthetic_ground_truth")
    v = r["alignment"]["scale_applied"]
    expected = 1.0 / 1.037
    checks.append(("Sim(3) recovers a 3.7% scale error",
                   abs(v - expected) < 1e-5,
                   f"{v:.6f} (expected {expected:.6f} = 1/1.037)"))
    log.metric("selftest_recovered_scale", float(v))

    pct = r["alignment"]["estimate_oversize_pct"]
    checks.append(("scale error reported as +3.7% oversize, right sign",
                   abs(pct - 3.7) < 1e-2, f"{pct:+.3f} %"))
    log.metric("selftest_estimate_oversize_pct", float(pct), "%")

    # 4. Realistic corruption: check the qualitative relationships that make ATE
    #    and RPE different metrics hold. Drift dominates ATE; white noise
    #    dominates RPE; so ATE must exceed the 1 m RPE by a wide margin.
    est = TM.corrupt(ref, drift_pct=1.5, noise_m=0.02,
                     yaw_bias_deg_per_m=0.03, seed=42, name="synthetic_slam")
    r = TM.evaluate(ref, est, reference_kind="synthetic_ground_truth")
    h = TM.headline(r)
    log.metric("selftest_ate_rmse_m", float(h["ate_rmse_m"]), "m")
    log.metric("selftest_rpe_1m_rmse_m", float(h["rpe_trans_rmse_m"]), "m")
    log.metric("selftest_drift_pct", float(h["drift_pct"]), "%")
    # Whole-trajectory ATE should be SEVERAL TIMES the per-metre RPE, because
    # drift accumulates along the path while local noise does not. 3x is a robust
    # bar that this synthetic generator clears (it produces ~4.5x); the old 5x was
    # over-tight for these generator parameters and false-failed a correct result.
    checks.append(("ATE is several times the per-metre RPE (drift accumulates, noise does not)",
                   h["ate_rmse_m"] > 3 * h["rpe_trans_rmse_m"],
                   f"ATE {h['ate_rmse_m']:.3f} m vs RPE {h['rpe_trans_rmse_m']:.4f} m (ratio {h['ate_rmse_m']/max(h['rpe_trans_rmse_m'],1e-9):.1f}x)"))
    checks.append(("all headline numbers finite",
                   all(v is None or np.isfinite(v)
                       for v in h.values() if isinstance(v, (int, float))),
                   "ok"))

    # 5. Loop detection must fire on a route that is a circle by construction.
    lc = TM.loop_closure_gap(ref, return_radius=1.0, min_path_before_return=10.0)
    checks.append(("closed loop detected on a circular route", bool(lc["closed"]),
                   f"closest approach {lc['closest_approach_m']:.3f} m"))

    # 6. A teleport must be detected as a pose-graph correction.
    jump_xyz = ref.xyz.copy()
    jump_xyz[len(ref) // 2:] += np.array([3.0, 0.0, 0.0])
    jumped = TM.Trajectory(ref.stamps, jump_xyz, ref.quat, "with_jump")
    cx = TM.detect_correction_jumps(jumped, max_speed_mps=2.0)
    checks.append(("injected 3 m teleport detected as a correction",
                   cx["n_corrections"] >= 1
                   and abs(cx["largest_correction_m"] - 3.0) < 0.5,
                   f"{cx['n_corrections']} correction(s), largest "
                   f"{cx['largest_correction_m']:.3f} m"))

    log.section("Self-test results")
    passed = 0
    for desc, ok, detail in checks:
        (log.info if ok else log.error)(
            f"[{'PASS' if ok else 'FAIL'}] {desc}  ({detail})")
        passed += bool(ok)

    all_ok = passed == len(checks)
    log.summary(f"Self-test {passed}/{len(checks)} checks passed — the "
                f"evaluation mathematics {'is' if all_ok else 'is NOT'} behaving "
                f"as expected on synthetic data with known error",
                status="OK" if all_ok else "FAIL")
    return 0 if all_ok else 1


# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #

def main() -> int:
    ap = argparse.ArgumentParser(
        description="Compute trajectory-accuracy metrics and build an HTML "
                    "report. Runs offline; needs no ROS.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    ap.add_argument("--ref", default="",
                    help="reference trajectory, TUM format")
    ap.add_argument("--est", action="append", default=[],
                    help="estimated trajectory as `label=path` (repeatable)")
    ap.add_argument("--nav-run", action="append", default=[],
                    help="a navigation run directory written by "
                         "sidewalk_navigation/waypoint_runner.py; its "
                         "trajectory joins the estimates (repeatable)")
    ap.add_argument("--status", action="append", default=[],
                    help="tracking-status log as `label=path` (repeatable)")
    ap.add_argument("--reference-kind", default="unspecified",
                    choices=list(REFERENCE_KINDS),
                    help="WHAT you are calling truth. Recorded in the report; "
                         "set it honestly. When the reference's .meta.json "
                         "sidecar declares a kind, this must agree with it "
                         "(or be left unspecified to adopt it).")
    ap.add_argument("--align", default="",
                    choices=["", "se3", "sim3", "origin", "none"],
                    help="alignment mode (default from config: se3)")
    ap.add_argument("--align-window", type=float, default=0.0,
                    help="fit the alignment on only the first N seconds of "
                         "overlap; errors are still measured over the whole "
                         "run. Guards against least-squares tail-drag on "
                         "diverging trajectories. The policy is recorded in "
                         "the output and printed in the report.")
    ap.add_argument("--split-at-gaps", action="store_true",
                    help="additionally evaluate each clean segment between "
                         "tracking-loss gaps with its own Sim(3) fit. "
                         "Separates real scale/calibration error (persists "
                         "per segment) from teleport inflation (vanishes).")
    ap.add_argument("--loop-only", action="store_true",
                    help="no reference available; report drift from the "
                         "closed-loop return to start only")
    ap.add_argument("--time-offset", type=float, default=0.0,
                    help="seconds to add to estimate timestamps before matching")
    ap.add_argument("--max-time-diff", type=float, default=-1.0,
                    help="association tolerance in seconds (default from config)")
    ap.add_argument("--config", default=str(DEFAULT_CONFIG))
    ap.add_argument("--name", default="",
                    help="name for the results directory (default: timestamp)")
    ap.add_argument("--no-report", action="store_true",
                    help="write JSON only; skip the HTML report")
    ap.add_argument("--self-test", action="store_true",
                    help="verify the metric code against synthetic data with "
                         "known injected error, then exit")
    args = ap.parse_args()

    # Free-text arguments may arrive from roslaunch still wrapped in the quotes
    # the launch file needed to protect their spaces. See eval_common.unquote.
    from eval_common import unquote
    args.name = unquote(args.name)
    args.ref = unquote(args.ref)
    args.config = unquote(args.config)
    # `label=path` pairs need the path half unquoted, not the whole token.
    def _unquote_pair(item: str) -> str:
        label, sep, path = item.partition("=")
        return f"{label}={unquote(path)}" if sep else unquote(item)
    args.est = [_unquote_pair(x) for x in args.est]
    args.status = [_unquote_pair(x) for x in args.status]
    args.nav_run = [unquote(x) for x in args.nav_run]

    run_name = args.name or ("self_test" if args.self_test else "evaluate")

    with RunLogger("sidewalk_evaluation", run_name=run_name) as log:
        if args.self_test:
            return run_self_test(log)

        # ---------------------------------------------------------- config
        try:
            cfg = load_config(args.config)
        except FileNotFoundError as exc:
            log.error(str(exc))
            log.summary("Configuration missing", status="FAIL")
            return 2
        metrics_cfg = cfg.get("metrics", {}) if isinstance(cfg.get("metrics"), dict) else {}

        align_mode = args.align or str(metrics_cfg.get("align_mode", "se3"))
        max_diff = args.max_time_diff if args.max_time_diff > 0 else \
            float(metrics_cfg.get("max_time_diff_s", 0.02))
        rpe_deltas = parse_rpe_deltas(metrics_cfg.get("rpe_deltas"))
        seg_lengths = [float(x) for x in
                       (metrics_cfg.get("drift_segment_lengths_m")
                        or [5, 10, 20, 40, 80])]
        max_speed = float(metrics_cfg.get("max_speed_mps", 2.0))

        # ---------------------------------------------------------- inputs
        est_specs = list(args.est)
        if args.nav_run:
            log.section("Navigation runs")
            nav_specs, nav_fatal = expand_nav_runs(args.nav_run, log)
            if nav_fatal:
                log.summary("One or more navigation runs could not be read",
                            status="FAIL")
                return 2
            est_specs.extend(nav_specs)

        try:
            estimates = parse_named(est_specs, "est")
            statuses = parse_named(args.status, "status")
        except SystemExit as exc:
            log.error(str(exc))
            raise

        if not estimates:
            log.error(
                "No estimated trajectory was given. Use `--est label=path.tum` "
                "or `--nav-run DIR` at least once.\n"
                "Trajectory files are produced by `trajectory_recorder.py` "
                "during a run, or exported from RTAB-Map / ORB-SLAM3 "
                "afterwards. They live in logs/sidewalk_evaluation/.")
            log.summary("Nothing to evaluate: no --est given", status="FAIL")
            return 2

        missing = [f"{n} -> {p}" for n, p in estimates.items() if not p.exists()]
        if missing:
            log.error("These trajectory files do not exist:\n  " +
                      "\n  ".join(missing) +
                      "\nCheck the path. If the recorder was running but the "
                      "file is absent, the SLAM stack never published a pose — "
                      "which is itself the finding.")
            log.summary("Input trajectory files missing", status="FAIL")
            return 2

        ref = None
        if args.ref and not args.loop_only:
            ref_path = Path(args.ref).expanduser()
            if not ref_path.exists():
                log.error(
                    f"Reference trajectory not found: {ref_path}\n"
                    f"If you have no reference — which is the normal situation "
                    f"without a motion-capture system — re-run with "
                    f"`--loop-only`. That measures drift from the route "
                    f"returning to its marked start, and clearly records that "
                    f"ATE and RPE were not computed rather than inventing them.")
                log.summary("Reference trajectory missing", status="FAIL")
                return 2
            ref = TM.load_tum(ref_path, name="reference")

            # ---------------------------------------- provenance, layer 1
            # The recorder writes a .meta.json sidecar beside every .tum.
            # When it declares what the reference IS, that declaration is
            # the ground truth about the ground truth — a CLI flag that
            # is refused rather than trusted.
            sidecar_kind = read_sidecar_kind(ref_path)
            if sidecar_kind:
                if args.reference_kind == "unspecified":
                    log.info(f"Reference kind adopted from "
                             f"{ref_path.name}.meta.json: '{sidecar_kind}'")
                    args.reference_kind = sidecar_kind
                elif args.reference_kind != sidecar_kind:
                    log.error(
                        f"--reference-kind '{args.reference_kind}' contradicts "
                        f"the reference's own sidecar "
                        f"({ref_path.name}.meta.json says '{sidecar_kind}').\n"
                        f"The sidecar was written by the recorder that "
                        f"produced the file and is the more trustworthy of "
                        f"the two. Drop the flag to adopt it, or fix the "
                        f"sidecar if it is genuinely wrong.")
                    log.summary("Reference kind contradicts sidecar",
                                status="FAIL")
                    return 2
            if args.reference_kind == "unspecified":
                log.error(
                    "A reference was given but its kind was not declared, and "
                    "its .meta.json sidecar (if any) does not declare one "
                    "either.\n"
                    "An ATE without a stated truth source is meaningless "
                    "(ENGINEERING_NOTES.md section 4.1). Say what this reference is:\n"
                    "  --reference-kind simulator_ground_truth   Gazebo "
                    "model_states truth\n"
                    "  --reference-kind motion_capture           external "
                    "tracking (e.g. TUM RGB-D)\n"
                    "  --reference-kind wheel_odometry           encoder "
                    "integration (short-range sanity)\n"
                    f"Full list: {', '.join(REFERENCE_KINDS)}")
                log.summary("Reference kind undeclared", status="FAIL")
                return 2
        elif not args.loop_only:
            log.error(
                "No reference trajectory given, and `--loop-only` was not set.\n"
                "ATE and RPE are comparisons: without something to compare "
                "against they do not exist, and this tool will not fabricate "
                "them. Either:\n"
                "  * pass `--ref FILE` (wheel odometry is a legitimate "
                "short-range reference — say so with "
                "`--reference-kind wheel_odometry`), or\n"
                "  * pass `--loop-only` to measure drift from a closed route.")
            log.summary("No reference and no --loop-only", status="FAIL")
            return 2

        out_dir = ensure_results_dir(args.name or timestamp_slug())
        log.info(f"Results directory: {out_dir}")
        log.info(f"Alignment mode: {align_mode}; association tolerance "
                 f"{max_diff * 1000:.0f} ms; reference kind: "
                 f"{args.reference_kind}")

        if ref is not None:
            v = TM.validate_trajectory(ref)
            log.info(f"Reference '{ref.name}': {v['n_poses']} poses, "
                     f"{v['path_length_m']:.1f} m over {v['duration_s']:.1f} s "
                     f"({v['mean_rate_hz']:.1f} Hz)")
            for issue in v["issues"]:
                log.warn(f"Reference: {issue}")
            TM.save_tum(out_dir / "reference.tum", ref,
                        comment=f"reference kind: {args.reference_kind}")

        # ------------------------------------------------------- evaluation
        summaries = []
        exit_code = 0
        for name, path in estimates.items():
            log.section(f"Evaluating: {name}")
            try:
                est = TM.load_tum(path, name=name)
            except (FileNotFoundError, ValueError) as exc:
                log.error(f"{name}: {exc}")
                exit_code = 1
                continue

            v = TM.validate_trajectory(est)
            log.info(f"{name}: {v['n_poses']} poses, {v['path_length_m']:.1f} m "
                     f"over {v['duration_s']:.1f} s ({v['mean_rate_hz']:.1f} Hz)")
            for issue in v["issues"]:
                log.warn(f"{name}: {issue}")

            if args.loop_only:
                report = evaluate_loop_only(est, metrics_cfg, log)
            else:
                report = TM.evaluate(
                    ref, est,
                    align_mode=align_mode,
                    max_time_diff=max_diff,
                    time_offset=args.time_offset,
                    rpe_deltas=rpe_deltas,
                    segment_lengths=seg_lengths,
                    max_speed_mps=max_speed,
                    align_window_s=args.align_window,
                    reference_kind=args.reference_kind)
                if args.align_window > 0 and "alignment" in report:
                    log.info(f"{name}: alignment fitted on the first "
                             f"{args.align_window:g} s only "
                             f"({report['alignment'].get('align_fit_poses')} "
                             f"poses); errors measured over the whole run.")

                # Per-clean-segment Sim(3): a real calibration error keeps its
                # scale inside every segment; teleport inflation does not
                # survive the cut.
                if args.split_at_gaps:
                    segs = TM.split_at_gaps(est)
                    log.section(f"{name}: per-segment Sim(3) "
                                f"({len(segs)} segment(s) between gaps)")
                    seg_rows = []
                    for a, b in segs:
                        if b - a < 50:
                            continue
                        sub = est.subset(range(a, b))
                        sub_r = TM.evaluate(ref, sub, align_mode="sim3",
                                            max_time_diff=max_diff,
                                            time_offset=args.time_offset,
                                            reference_kind=args.reference_kind)
                        if "error" in sub_r:
                            continue
                        seg_rows.append({
                            "start_s": float(sub.stamps[0]),
                            "end_s": float(sub.stamps[-1]),
                            "n_poses": len(sub),
                            "path_m": sub_r["drift"]["path_length_m"],
                            "scale_sim3":
                                sub_r["alignment"]["scale_estimated_sim3"],
                            "ate_rmse_m":
                                sub_r["ate"]["translation"]["rmse"],
                            "ate_median_m":
                                sub_r["ate"]["translation"]["median"],
                            "ate_p95_m": sub_r["ate"]["translation"]["p95"],
                        })
                    report["gap_segments"] = seg_rows
                    for s in seg_rows:
                        log.info(
                            f"  t={s['start_s']:8.1f}..{s['end_s']:8.1f}s  "
                            f"{s['n_poses']:5d} poses  {s['path_m']:7.1f} m  "
                            f"scale {s['scale_sim3']:.4f}  "
                            f"ATE {s['ate_rmse_m']:.3f} m")
                    if seg_rows:
                        longest = max(seg_rows, key=lambda s: s["n_poses"])
                        log.metric(f"{name}_longest_segment_scale_sim3",
                                   round(longest["scale_sim3"], 4))
                        log.metric(f"{name}_longest_segment_ate_rmse_m",
                                   round(longest["ate_rmse_m"], 4), "m")
                    else:
                        log.warn(f"{name}: no segment had the 50+ poses "
                                 f"needed for a stable per-segment fit")

            # Tracking status, if a log was recorded for this stack.
            if name in statuses:
                events = TM.parse_status_log(statuses[name])
                report["tracking"] = TM.compute_tracking_reliability(
                    events, run_duration_s=est.duration)
                tr = report["tracking"]
                if tr.get("available"):
                    log.metric(f"{name}_tracking_losses",
                               tr["tracking_loss_count"])
                    if tr.get("relocalization_success_rate_pct") is not None:
                        log.metric(f"{name}_reloc_success_rate",
                                   round(tr["relocalization_success_rate_pct"], 1), "%")

            if "error" in report:
                log.error(f"{name}: {report['error']}")
                exit_code = 1
            for w in report.get("warnings", []) or []:
                log.warn(f"{name}: {w}")

            # ------------------------------------------------- log metrics
            if "ate" in report:
                h = TM.headline(report)
                log.metric(f"{name}_ate_rmse_m", round(h["ate_rmse_m"], 5), "m")
                log.metric(f"{name}_ate_median_m", round(h["ate_median_m"], 5), "m")
                if h.get("ate_p95_m") is not None:
                    log.metric(f"{name}_ate_p95_m", round(h["ate_p95_m"], 5), "m")
                log.metric(f"{name}_ate_rot_rmse_deg",
                           round(h["ate_rot_rmse_deg"], 4), "deg")
                if h["rpe_trans_rmse_m"] is not None:
                    log.metric(f"{name}_rpe_{h['rpe_delta']}_rmse_m",
                               round(h["rpe_trans_rmse_m"], 5), "m")
                if h["drift_pct"] is not None:
                    log.metric(f"{name}_drift_pct", round(h["drift_pct"], 4), "%")
                if h["drift_deg_per_m"] is not None:
                    log.metric(f"{name}_drift_deg_per_m",
                               round(h["drift_deg_per_m"], 5), "deg/m")
                summaries.append(
                    f"{name}: ATE {fmt(h['ate_rmse_m'], 3)} m, "
                    f"drift {fmt(h['drift_pct'], 2)}%")
            lc = report.get("loop_closure_gap") or {}
            if lc.get("end_to_start_gap_m") is not None:
                log.metric(f"{name}_loop_gap_m",
                           round(lc["end_to_start_gap_m"], 4), "m")
                if args.loop_only:
                    summaries.append(
                        f"{name}: loop gap {fmt(lc['end_to_start_gap_m'], 3)} m "
                        f"over {fmt(lc.get('path_length_m'), 1)} m "
                        f"({fmt(lc.get('drift_pct_of_path'), 2)}%)")
            cx = report.get("corrections") or {}
            if cx.get("n_corrections") is not None:
                log.metric(f"{name}_corrections", cx["n_corrections"])

            # ----------------------------------------------- write artefacts
            if report.get("_aligned") is not None:
                TM.save_tum(out_dir / f"aligned_{name}.tum", report["_aligned"],
                            comment=(f"aligned to '{ref.name}' using "
                                     f"{align_mode}; reference kind "
                                     f"{args.reference_kind}"))
            else:
                # Loop-only mode: keep a copy of the raw estimate so the report
                # directory is self-contained and re-analysable.
                shutil.copyfile(path, out_dir / f"raw_{name}.tum")
            write_plotdata(out_dir, name, ref, est, report)
            write_json(out_dir / f"eval_{name}.json", strip_private(report))

        # ----------------------------------------------------------- report
        if not args.no_report:
            try:
                import generate_report
                doc = generate_report.build_report(
                    out_dir, title=args.name or f"Evaluation {out_dir.name}",
                    log=log)
                (out_dir / "report.html").write_text(doc)
                log.info(f"Report: {out_dir / 'report.html'}")
            except MissingDependency as exc:
                log.warn(f"Report not generated: {exc}")
            except ProvenanceError as exc:
                # Deliberate refusal, not a failure of the tooling: the report
                # will not print numbers whose truth source is unquotable.
                # The raw JSON stays on disk for diagnostics; nothing headline
                # gets rendered. Exit 4 = provenance refusal, distinct from
                # exit 2 (bad inputs) and 3 (missing dependency).
                log.error(str(exc))
                log.summary("Report refused: unquotable reference kind",
                            status="FAIL")
                return 4
            except Exception as exc:                       # noqa: BLE001
                # A broken report must not destroy the numbers, which are the
                # actual result and are already safely on disk.
                log.error(f"Report generation failed ({type(exc).__name__}: "
                          f"{exc}). The JSON metrics in {out_dir} are complete "
                          f"and unaffected; re-run "
                          f"`generate_report.py --results {out_dir}` once the "
                          f"cause is fixed.")
                exit_code = max(exit_code, 1)

        text = "; ".join(summaries) if summaries else \
            f"evaluated {len(estimates)} trajectory file(s)"
        log.summary(f"{text}  [{out_dir.name}]",
                    status="OK" if exit_code == 0 else "FAIL")
        return exit_code


if __name__ == "__main__":
    sys.exit(main())
