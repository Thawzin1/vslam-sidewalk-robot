#!/usr/bin/env python3
"""
reset_gaps.py — per-reset ground-truth displacement and resets/100 m, from a
run monitor CSV.

WHY THIS EXISTS (ENGINEERING_NOTES.md 2.4)
    On every odometry reset RTAB-Map discards the outage's frames, and the
    graph link spanning the gap OMITS every metre the robot travelled — while
    carrying the most confident covariance in the whole graph. Whether that
    matters depends entirely on how far the robot MOVED during each outage:
    a stationary gap is harmless, a moving gap silently deletes path. So the
    repo rule is: never quote an ATE without resets/100 m and the summed gap
    displacement beside it. This script produces both.

    First used (as a throwaway) on mcity_v2_run2b: 13 resets, median 0.875 m
    per gap, 8.96 m of path deleted — the robot was moving at near full
    speed through almost every outage.

INPUT
    The 1 Hz CSV written by sidewalk_slam/scripts/run_monitor.py
    (columns: t,gt_x,gt_y,est_x_aligned,est_y_aligned,pos_err,yaw_err,
    quality,resets,accepted_lc,rejected_lc), archived per run at
    ~/.run_records/<run_id>/monitor.csv since 2026-08-22.

USAGE
    rosrun sidewalk_evaluation reset_gaps.py --run-id p1_A_r1
    rosrun sidewalk_evaluation reset_gaps.py --csv /path/to/monitor.csv --json out.json

    The 1 Hz sampling brackets each reset within one second of ground truth,
    so each per-gap figure is an upper bound tight to ~one sample.
"""
import os
import argparse
import csv
import json
import math
import sys
from pathlib import Path


def analyze(csv_path: Path) -> dict:
    rows = []
    with open(csv_path) as fh:
        for r in csv.DictReader(fh):
            try:
                rows.append((float(r["t"]), float(r["gt_x"]), float(r["gt_y"]),
                             int(r["resets"])))
            except (KeyError, ValueError):
                continue
    if len(rows) < 2:
        raise SystemExit(f"{csv_path}: fewer than 2 usable samples")

    # Ground-truth path length, integrated at the CSV's 1 Hz. Slightly under
    # the true length (chords, not arcs) — fine for a per-100 m rate.
    path_m = sum(math.hypot(b[1] - a[1], b[2] - a[2])
                 for a, b in zip(rows, rows[1:]))

    # A reset EPISODE is one 1 Hz sample interval in which the monitor's reset
    # counter increased. Several resets can land inside one interval, so the
    # number of episodes is NOT the number of resets - counting episodes
    # undercounted by 2x on a real run (85 episodes for 170 actual resets,
    # are reported separately because the displacement statistics below are
    # necessarily per-episode (that is the resolution the CSV has).
    events = []
    n_resets = 0
    prev = rows[0]
    for cur in rows[1:]:
        if cur[3] > prev[3]:
            jump = cur[3] - prev[3]
            n_resets += jump
            d = math.hypot(cur[1] - prev[1], cur[2] - prev[2])
            events.append({"reset": cur[3], "n_in_interval": jump,
                           "t_s": cur[0],
                           "gap_gt_displacement_m": round(d, 3)})
        prev = cur

    disp = sorted(e["gap_gt_displacement_m"] for e in events)
    total = sum(disp)
    return {
        "csv": str(csv_path),
        "samples": len(rows),
        "duration_s": round(rows[-1][0] - rows[0][0], 1),
        "gt_path_length_m": round(path_m, 2),
        "n_resets": n_resets,
        "n_reset_episodes": len(events),
        "resets_per_100m": round(100.0 * n_resets / path_m, 3) if path_m > 0 else None,
        "gap_displacement_total_m": round(total, 2),
        "gap_displacement_median_m": disp[len(disp) // 2] if disp else 0.0,
        "gap_displacement_max_m": disp[-1] if disp else 0.0,
        "events": events,
    }


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--csv", help="monitor CSV path")
    src.add_argument("--run-id",
                     help="shortcut for ~/.run_records/<run_id>/monitor.csv")
    ap.add_argument("--json", default="", help="also write the summary here")
    args = ap.parse_args()

    csv_path = (Path(args.csv).expanduser() if args.csv
                else Path(os.environ.get("RECORDS_DIR", str(Path.home() / ".run_records"))) / args.run_id / "monitor.csv")
    if not csv_path.exists():
        print(f"not found: {csv_path}", file=sys.stderr)
        return 2

    out = analyze(csv_path)

    print(f"{out['samples']} samples over {out['duration_s']:.0f} s, "
          f"GT path {out['gt_path_length_m']:.1f} m")
    print(f"{'reset':>5} {'t [s]':>8} {'gap GT displacement [m]':>24}")
    for e in out["events"]:
        print(f"{e['reset']:>5} {e['t_s']:>8.1f} "
              f"{e['gap_gt_displacement_m']:>24.3f}")
    print(f"\nresets: {out['n_resets']}   "
          f"resets/100m: {out['resets_per_100m']}   "
          f"gap displacement: total {out['gap_displacement_total_m']} m, "
          f"median {out['gap_displacement_median_m']} m, "
          f"max {out['gap_displacement_max_m']} m")
    gate = (out["resets_per_100m"] is not None
            and out["resets_per_100m"] < 2.0)
    print(f"GATE (resets/100m < 2): {'PASS' if gate else 'FAIL'}")

    if args.json:
        p = Path(args.json).expanduser()
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(out, indent=2))
        print(f"wrote {p}")
    return 0 if gate else 1


if __name__ == "__main__":
    sys.exit(main())
