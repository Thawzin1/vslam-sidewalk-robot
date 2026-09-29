#!/usr/bin/env python3
"""
compare_slam.py — The three-way comparison that is the project's core result.

WHAT IT COMPARES
    RTAB-Map          the primary mapping and localization system for this
                      project, consuming ZED X depth, rectified images and IMU.
    ZED SDK tracking  Stereolabs' own visual-inertial positional tracking,
                      running inside the SDK on the same frames.
    ORB-SLAM3         a widely cited research baseline, stereo-inertial.

WHY A COMPARISON NEEDS ITS OWN PROGRAM
    Because the hard part is not computing the numbers — `evaluate_trajectory.py`
    already did that — it is making them comparable and refusing to present
    them when they are not.

    This program enforces three things that a hand-made table does not:

    1. **Identical input.** It checks that the trajectories being compared come
       from the same run, by comparing their time spans. Two stacks evaluated on
       different drives down the same sidewalk are not a comparison, and the
       table says so rather than quietly ranking them.

    2. **Repeats, not single runs.** Drift is a random walk; a single run is a
       single draw from a distribution. Given several result directories from
       repeats of the same route, this aggregates them and reports mean and
       spread. A winner whose advantage is smaller than the run-to-run spread
       is not a winner, and the output says that in words.

    3. **Cost alongside accuracy.** Where a `system_load.json` is present for a
       stack, its CPU, GPU, memory and latency appear in the same table. A
       system that wins on accuracy while saturating the Orin has not obviously
       won.

USAGE
    # One run, three stacks:
    ./compare_slam.py --results logs/sidewalk_evaluation/results/20260801-1030

    # Three repeats of the same route, aggregated:
    ./compare_slam.py --results RUN1 --results RUN2 --results RUN3 \\
                      --out-dir logs/sidewalk_evaluation/results/route_a_summary
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from eval_common import (MissingDependency, RunLogger, fmt, require_numpy,
                         write_json)

# Canonical display names, so the table reads the same whatever the operator
# happened to label the trajectory files.
DISPLAY = {
    "rtabmap": "RTAB-Map (primary)",
    "rtab_map": "RTAB-Map (primary)",
    "zed": "ZED SDK tracking",
    "zed_tracking": "ZED SDK tracking",
    "zedtracking": "ZED SDK tracking",
    "orbslam3": "ORB-SLAM3",
    "orb_slam3": "ORB-SLAM3",
    "orbslam": "ORB-SLAM3",
    "odom": "Wheel odometry",
    "wheel_odom": "Wheel odometry",
}

# (json path, column heading, decimals, lower_is_better)
METRICS = [
    (("ate", "translation", "rmse"), "ATE RMSE [m]", 4, True),
    (("ate", "translation", "median"), "ATE median [m]", 4, True),
    (("ate", "translation", "p95"), "ATE 95th pct [m]", 4, True),
    (("ate", "rotation", "rmse"), "Rotation RMSE [deg]", 3, True),
    (("rpe", "1m", "translation", "rmse"), "RPE @1 m [m]", 4, True),
    (("rpe", "10m", "translation", "rmse"), "RPE @10 m [m]", 4, True),
    (("drift", "overall_translation_pct", "mean"), "Drift [%]", 3, True),
    (("drift", "overall_rotation_deg_per_m", "mean"), "Drift [deg/m]", 4, True),
    (("loop_closure_gap", "end_to_start_gap_m"), "Loop gap [m]", 3, True),
    (("corrections", "n_corrections"), "Corrections", 0, False),
    (("tracking", "tracking_loss_count"), "Tracking losses", 0, True),
    (("tracking", "relocalization_success_rate_pct"), "Reloc. success [%]", 1, False),
    (("tracking", "time_to_relocalize", "median"), "Reloc. time [s]", 2, True),
]

COST_METRICS = [
    (("summary", "cpu_mean_pct"), "CPU mean [%]", 1, True),
    (("summary", "gpu_mean_pct"), "GPU mean [%]", 1, True),
    (("summary", "proc_rss_max_mb"), "RAM peak [MB]", 0, True),
    (("latency", "latency_ms", "median"), "Latency median [ms]", 1, True),
    (("latency", "latency_ms", "p95"), "Latency 95th [ms]", 1, True),
    (("latency", "dropped_frame_pct"), "Dropped frames [%]", 2, True),
]


def dig(obj, keys, default=None):
    node = obj
    for k in keys:
        if not isinstance(node, dict) or k not in node:
            return default
        node = node[k]
    return node if isinstance(node, (int, float)) else default


def display_name(key: str) -> str:
    k = key.lower().replace("-", "_")
    for pat, name in DISPLAY.items():
        if pat in k:
            return name
    return key


def load_runs(dirs, log):
    """Load every eval_*.json from every results directory.

    Returns {stack_name: [report, ...]} — one entry per repeat of the route.
    """
    runs = {}
    for d in dirs:
        d = Path(d)
        if not d.is_dir():
            log.error(f"Not a directory: {d}. Each --results argument must be "
                      f"a directory produced by evaluate_trajectory.py.")
            continue
        found = sorted(d.glob("eval_*.json"))
        if not found:
            log.warn(f"{d} contains no eval_*.json files — skipped. Did "
                     f"evaluate_trajectory.py finish successfully for this run?")
            continue
        for f in found:
            name = f.stem[len("eval_"):]
            try:
                data = json.loads(f.read_text())
            except (OSError, json.JSONDecodeError) as exc:
                log.error(f"Could not read {f}: {exc}")
                continue
            data["_run_dir"] = str(d)
            runs.setdefault(name, []).append(data)
        log.info(f"{d.name}: {len(found)} stack(s) — "
                 f"{', '.join(f.stem[5:] for f in found)}")
    return runs


def load_costs(dirs, log):
    """Load per-stack system_load JSONs.

    Two layouts are supported: `system_load.json` for the whole run (attributed
    to every stack, because they were competing), or
    `system_load_<stack>.json` for a per-stack replay, which is the cleaner
    measurement.
    """
    costs = {}
    shared = []
    for d in dirs:
        d = Path(d)
        for f in sorted(d.glob("system_load*.json")):
            try:
                data = json.loads(f.read_text())
            except (OSError, json.JSONDecodeError):
                continue
            stem = f.stem
            if stem == "system_load":
                shared.append(data)
            else:
                stack = stem[len("system_load_"):]
                costs.setdefault(stack, []).append(data)
    if shared:
        log.info(f"Found {len(shared)} whole-run system-load recording(s). "
                 f"These describe the machine with all stacks running "
                 f"concurrently, so they cannot be attributed to any single "
                 f"algorithm; they are reported once, not per row.")
    return costs, shared


def aggregate(np, values):
    """Mean and spread across repeats of the same route.

    The spread is what tells you whether a difference is real. Reported as the
    sample standard deviation, and as the range, because with three runs the
    range is more honest than a standard deviation computed from three points.
    """
    v = [x for x in values if x is not None and np.isfinite(x)]
    if not v:
        return None
    a = np.asarray(v, dtype=float)
    return {
        "mean": float(a.mean()),
        "std": float(a.std(ddof=1)) if a.size > 1 else 0.0,
        "min": float(a.min()),
        "max": float(a.max()),
        "n": int(a.size),
    }


def check_same_input(np, runs, log) -> list:
    """Warn if the stacks being compared did not see the same data.

    Compares the recorded time span of each estimate. Two trajectories from the
    same run overlap almost exactly; trajectories from different drives do not.
    This is the check that prevents the single most damaging mistake available
    in this project — presenting a comparison of three different experiments as
    a comparison of three algorithms.
    """
    caveats = []
    spans = {}
    for name, reports in runs.items():
        for i, r in enumerate(reports):
            v = r.get("estimate_validation") or {}
            dur = v.get("duration_s")
            if dur:
                spans.setdefault(i, {})[name] = float(dur)
    for i, per_run in spans.items():
        if len(per_run) < 2:
            continue
        durs = np.asarray(list(per_run.values()), dtype=float)
        if durs.max() <= 0:
            continue
        spread = float((durs.max() - durs.min()) / durs.max())
        if spread > 0.10:
            caveats.append(
                f"Run {i + 1}: the trajectories being compared span noticeably "
                f"different durations (" +
                ", ".join(f"{k} {v:.0f} s" for k, v in per_run.items()) +
                f"; {spread * 100:.0f}% apart). A valid comparison requires "
                f"identical input. Either one stack lost tracking and stopped "
                f"publishing — which is a finding worth reporting in its own "
                f"right — or these files are from different drives, in which "
                f"case this table is not a comparison of algorithms.")
    return caveats


def build_table(np, runs, costs, log):
    """Assemble headers, rows, best-cell highlights and caveats."""
    names = sorted(runs, key=lambda n: (
        0 if "rtab" in n.lower() else 1 if "zed" in n.lower()
        else 2 if "orb" in n.lower() else 3, n))
    n_repeats = max((len(v) for v in runs.values()), default=0)

    headers = ["Localization stack"]
    specs = []
    for keys, label, digits, lower in METRICS:
        vals = {n: [dig(r, keys) for r in runs[n]] for n in names}
        if all(all(v is None for v in vv) for vv in vals.values()):
            continue                      # nothing measured this; omit the column
        headers.append(label)
        specs.append((keys, label, digits, lower, vals))

    cost_specs = []
    if costs:
        for keys, label, digits, lower in COST_METRICS:
            vals = {}
            for n in names:
                vals[n] = [dig(c, keys) for c in costs.get(n, [])]
            if all(not vv or all(v is None for v in vv) for vv in vals.values()):
                continue
            headers.append(label)
            cost_specs.append((keys, label, digits, lower, vals))

    rows, best_cells = [], {}
    agg_store = {}
    for r_i, n in enumerate(names):
        row = [f"<code>{display_name(n)}</code>"]
        agg_store[n] = {}
        for c_i, (keys, label, digits, lower, vals) in enumerate(
                specs + cost_specs, start=1):
            a = aggregate(np, vals.get(n, []))
            agg_store[n][label] = a
            if a is None:
                row.append("—")
            elif a["n"] > 1:
                # Mean plus-or-minus the sample standard deviation across
                # repeats. The spread is not decoration: it is the yardstick
                # against which any claimed difference must be measured.
                row.append(f"{a['mean']:.{digits}f} ± {a['std']:.{digits}f}")
            else:
                row.append(f"{a['mean']:.{digits}f}")
        rows.append(row)

    # Highlight the winner in each column, but only where the win is larger
    # than the run-to-run spread. A highlight that is not statistically
    # supportable is worse than no highlight.
    for c_i, (keys, label, digits, lower, vals) in enumerate(
            specs + cost_specs, start=1):
        entries = [(agg_store[n][label], n, r_i)
                   for r_i, n in enumerate(names)
                   if agg_store[n].get(label) is not None]
        if len(entries) < 2:
            continue
        entries.sort(key=lambda e: e[0]["mean"], reverse=not lower)
        best, runner = entries[0], entries[1]
        gap = abs(best[0]["mean"] - runner[0]["mean"])
        noise = max(best[0]["std"], runner[0]["std"])
        if n_repeats > 1 and gap <= noise:
            continue                      # too close to call
        best_cells.setdefault(best[2], c_i)

    return headers, rows, best_cells, agg_store, names, n_repeats


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Build the three-way SLAM comparison table from one or "
                    "more evaluation results directories.")
    ap.add_argument("--results", action="append", required=True,
                    help="an evaluation results directory (repeat for each "
                         "repeat of the route)")
    ap.add_argument("--out-dir", default="",
                    help="where to write comparison.json and report.html "
                         "(default: the first --results directory)")
    ap.add_argument("--title", default="Three-way SLAM comparison")
    ap.add_argument("--route", default="",
                    help="description of the route these runs cover, e.g. "
                         "'campus loop, 240 m, dry, overcast'")
    ap.add_argument("--no-report", action="store_true")
    args = ap.parse_args()

    with RunLogger("sidewalk_evaluation", run_name="compare") as log:
        try:
            np = require_numpy(log)
        except MissingDependency as exc:
            print(f"\nERROR: {exc}\n", file=sys.stderr)
            log.summary("Cannot compare: numpy missing", status="FAIL")
            return 3

        log.section("Loading evaluation results")
        runs = load_runs(args.results, log)
        if not runs:
            log.error(
                "No evaluation results were found in any of the given "
                "directories. Each --results argument must point at a "
                "directory produced by `evaluate_trajectory.py`, containing "
                "eval_<stack>.json files.")
            log.summary("Nothing to compare", status="FAIL")
            return 2
        if len(runs) < 2:
            log.warn(
                f"Only one localization stack ({list(runs)[0]}) is present. "
                f"A comparison needs at least two. The table below will be "
                f"produced anyway so the numbers are recorded, but it is not "
                f"the three-way comparison the project set out to make.")

        costs, shared_costs = load_costs(args.results, log)
        caveats = check_same_input(np, runs, log)
        for c in caveats:
            log.warn(c)

        headers, rows, best_cells, agg, names, n_repeats = build_table(
            np, runs, costs, log)

        log.section("Comparison")
        log.info(f"{len(names)} stack(s), {n_repeats} repeat(s) of the route")
        for n in names:
            a = agg[n].get("ATE RMSE [m]")
            d = agg[n].get("Drift [%]")
            log.info(f"  {display_name(n):24s}  "
                     f"ATE {fmt(a['mean'] if a else None, 4, 'm')}  "
                     f"drift {fmt(d['mean'] if d else None, 3, '%')}")
            if a:
                log.metric(f"{n}_ate_rmse_mean_m", round(a["mean"], 5), "m")
                if a["n"] > 1:
                    log.metric(f"{n}_ate_rmse_std_m", round(a["std"], 5), "m")
            if d:
                log.metric(f"{n}_drift_mean_pct", round(d["mean"], 4), "%")

        # ---------------------------------------------------- honest caveats
        ref_kinds = {r.get("reference_kind", "unspecified")
                     for reports in runs.values() for r in reports}
        if "cross_stack" in ref_kinds:
            caveats.append(
                "At least one evaluation used another SLAM stack as its "
                "reference. That measures DISAGREEMENT between two estimators, "
                "not the error of either. Systems sharing a camera, a "
                "calibration and a viewpoint share their failure modes, so they "
                "can agree closely and both be wrong. Do not present these "
                "figures as accuracy.")
        if "nav_repeat_run" in ref_kinds:
            # ingest_navigation_run.py writes eval_*.json into the same results
            # directory this script globs, deliberately, so no plumbing is
            # needed. The cost of that convenience is that a REPEATABILITY
            # figure lands in a table of accuracy figures and, being a
            # like-against-like comparison, it is usually the smallest number
            # in the column — so it wins the ranking for the wrong reason.
            caveats.append(
                "At least one evaluation used another NAVIGATION RUN of the "
                "same route as its reference. That measures REPEATABILITY — "
                "how consistently one localisation system reproduces its own "
                "answer — and is NOT accuracy. Two runs can agree to a "
                "centimetre and both be two metres from where the robot "
                "physically was. Such a row is not comparable with rows "
                "measured against an independent reference, and it must not be "
                "ranked against them: it will usually look best simply because "
                "it was compared with itself.")
        if "unspecified" in ref_kinds:
            caveats.append(
                "At least one evaluation did not record what it used as truth. "
                "Re-run it with `--reference-kind` set. A trajectory error "
                "without a stated reference cannot be interpreted, and should "
                "not appear in a report.")
        if n_repeats < 3:
            caveats.append(
                f"Only {n_repeats} run(s) of this route are included. Drift is "
                f"a random walk: two runs of the same system over the same "
                f"route routinely differ by a factor of two by chance alone. "
                f"At least three repeats are needed before any ranking here "
                f"should be believed, and the ± figures above are the yardstick "
                f"for whether a difference is real.")
        else:
            caveats.append(
                f"Cells are highlighted only where the winning margin exceeds "
                f"the run-to-run spread across {n_repeats} repeats. An "
                f"unhighlighted column is one where the stacks are not "
                f"distinguishable by this data.")
        if shared_costs:
            caveats.append(
                "The compute-cost figures were recorded with all stacks "
                "running concurrently on one Jetson, so they describe the "
                "combined configuration rather than any single algorithm. To "
                "attribute cost per algorithm, replay the same rosbag once per "
                "stack and record `system_load_<stack>.json` for each pass.")
        if not costs and not shared_costs:
            caveats.append(
                "No compute-cost data is included. Accuracy without cost is "
                "half a result on a battery-powered robot — run "
                "`system_monitor.py` alongside the next experiment.")

        preamble = (
            f"Localization accuracy of {len(names)} stack(s) over "
            f"{n_repeats} run(s)"
            + (f" of {args.route}" if args.route else "")
            + ". RTAB-Map is this project's primary system; the ZED SDK's own "
              "positional tracking and ORB-SLAM3 are comparative baselines "
              "evaluated on identical recorded input. Where several runs are "
              "aggregated, cells show the mean plus or minus the sample "
              "standard deviation across runs.")

        out_dir = Path(args.out_dir) if args.out_dir else Path(args.results[0])
        out_dir.mkdir(parents=True, exist_ok=True)
        payload = {
            "title": args.title,
            "route": args.route,
            "preamble": preamble,
            "headers": headers,
            "rows": rows,
            "best_cells": {str(k): v for k, v in best_cells.items()},
            "caveats": caveats,
            "n_repeats": n_repeats,
            "stacks": {n: agg[n] for n in names},
            "source_dirs": [str(Path(d)) for d in args.results],
            "reference_kinds": sorted(ref_kinds),
        }
        write_json(out_dir / "comparison.json", payload)
        log.info(f"Wrote {out_dir / 'comparison.json'}")

        if not args.no_report:
            try:
                import generate_report
                doc = generate_report.build_report(out_dir, title=args.title,
                                                   log=log)
                (out_dir / "report.html").write_text(doc)
                log.info(f"Report: {out_dir / 'report.html'}")
            except MissingDependency as exc:
                log.warn(f"Report not generated: {exc}")
            except Exception as exc:                       # noqa: BLE001
                log.error(f"Report generation failed ({type(exc).__name__}: "
                          f"{exc}). comparison.json is complete and unaffected.")

        for c in caveats:
            log.warn(c)

        winner = None
        best_ate = None
        for n in names:
            a = agg[n].get("ATE RMSE [m]")
            if a and (best_ate is None or a["mean"] < best_ate):
                best_ate, winner = a["mean"], n
        text = (f"Compared {len(names)} stack(s) over {n_repeats} run(s)"
                + (f"; lowest ATE: {display_name(winner)} "
                   f"({best_ate:.3f} m)" if winner else ""))
        log.summary(text, status="WARN" if caveats else "OK")
        return 0


if __name__ == "__main__":
    sys.exit(main())
