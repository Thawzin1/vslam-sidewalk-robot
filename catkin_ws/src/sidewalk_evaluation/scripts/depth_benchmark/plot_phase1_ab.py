#!/usr/bin/env python3
"""
plot_phase1_ab.py — the Phase-1 deliverable figure: the same route, before and
after the drift fixes, on ground truth, with reset locations marked.

Two panels sharing axis limits:
    left  ("before") — a representative arm-A run (RGB-D, noise-free camera)
    right ("after")  — a representative arm-B1 run (stereo, real noise)
Each panel: ground truth as a solid green line, the SLAM estimate as a dashed
red line, and an x mark at the ground-truth position of every odometry reset
(from the run's monitor CSV, at the samples where the `resets` column
increments). The annotation line carries the three gate numbers for that run.

Solid for truth and dashed for the estimate is fixed across every trajectory
figure in this project, so the two are still told apart on a black-and-white
printout where the green and the red land on the same grey.

USAGE (offline; numpy+matplotlib, no ROS)
    plot_phase1_ab.py \\
        --before-tum-dir logs/sidewalk_slam/trajectories/p1_A_r3 \\
        --before-monitor ~/.run_records/p1_A_r3/monitor.csv \\
        --before-label  "A: RGB-D, no noise" \\
        --before-note   "resets/100m 5.2 - yield 58% - ATE med 20.6 m" \\
        --after-tum-dir  logs/sidewalk_slam/trajectories/p1_B1_r2 \\
        --after-monitor  ~/.run_records/p1_B1_r2/monitor.csv \\
        --after-label   "B1: stereo, noise 0.005" \\
        --after-note    "resets/100m 0.0 - yield 97% - ATE med 0.4 m" \\
        --out logs/sidewalk_evaluation/results/phase1/phase1_before_after.png
"""
import argparse
import csv
import math
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

BG = "#111318"
GRID = "#252932"
FG = "#c8ccd6"
TITLE = "#e8eaf0"
GT_C = "#39d353"
EST_C = "#ff6b6b"
RESET_C = "#f6c945"


def load_tum(path):
    xs, ys = [], []
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            p = line.split()
            xs.append(float(p[1]))
            ys.append(float(p[2]))
    return np.array(xs), np.array(ys)


def reset_positions(monitor_csv):
    """Ground-truth (x, y) at each increment of the `resets` column."""
    pts = []
    prev = None
    with open(Path(monitor_csv).expanduser()) as fh:
        for r in csv.DictReader(fh):
            try:
                cur = (float(r["gt_x"]), float(r["gt_y"]), int(r["resets"]))
            except (KeyError, ValueError):
                continue
            if prev is not None and cur[2] > prev[2]:
                pts.append((cur[0], cur[1]))
            prev = cur
    return pts


def draw_panel(ax, tum_dir, monitor, label, note):
    tum_dir = Path(tum_dir).expanduser()
    gt_x, gt_y = load_tum(tum_dir / "ground_truth_trajectory.tum")
    es_x, es_y = load_tum(tum_dir / "rtabmap_trajectory.tum")

    ax.set_facecolor(BG)
    # Solid = where the robot truly was, dashed = where the mapping system
    # thought it was. This is the project's standing convention for every
    # trajectory figure, and it is what makes the figure survive being
    # printed in black and white or read by someone who cannot separate the
    # green from the red. The colours stay as a second, redundant channel;
    # the line style is the one that always gets through, so the legend text
    # names it in words too rather than relying on the reader matching a
    # short dash pattern in a small legend box to a long line on the plot.
    ax.plot(gt_x, gt_y, color=GT_C, lw=1.5, linestyle="-",
            label="ground truth (solid)", zorder=4)
    ax.plot(es_x, es_y, color=EST_C, lw=1.3, linestyle="--",
            label="RTAB-Map estimate (dashed)", zorder=5)
    ax.scatter([gt_x[0]], [gt_y[0]], c=GT_C, s=70, marker="o",
               edgecolors="white", zorder=7, label="start")

    resets = reset_positions(monitor) if monitor else []
    if resets:
        rx, ry = zip(*resets)
        ax.scatter(rx, ry, c=RESET_C, s=90, marker="x", linewidths=2.2,
                   zorder=8, label=f"odometry reset ({len(resets)})")

    ax.set_title(label, color=TITLE, fontsize=12)
    ax.text(0.02, 0.02, note, transform=ax.transAxes, color=TITLE,
            fontsize=9, family="monospace",
            bbox=dict(facecolor="#1a1d24", edgecolor="#3a4050", pad=4))
    ax.set_xlabel("x (m)", color=FG)
    ax.set_ylabel("y (m)", color=FG)
    ax.tick_params(colors=FG)
    for sp in ax.spines.values():
        sp.set_color("#3a4050")
    ax.grid(True, color=GRID, lw=0.5)
    ax.set_aspect("equal")
    leg = ax.legend(loc="upper right", facecolor="#1a1d24",
                    edgecolor="#3a4050", fontsize=8)
    for t in leg.get_texts():
        t.set_color(TITLE)
    return (gt_x, gt_y, es_x, es_y)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    for side in ("before", "after"):
        ap.add_argument(f"--{side}-tum-dir", required=True)
        ap.add_argument(f"--{side}-monitor", default="")
        ap.add_argument(f"--{side}-label", required=True)
        ap.add_argument(f"--{side}-note", default="")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8), dpi=140)
    fig.patch.set_facecolor(BG)

    d1 = draw_panel(ax1, args.before_tum_dir, args.before_monitor,
                    args.before_label, args.before_note)
    d2 = draw_panel(ax2, args.after_tum_dir, args.after_monitor,
                    args.after_label, args.after_note)

    # Shared limits so the eye compares like with like: union of both panels'
    # data, padded. Without this the "after" panel would auto-zoom to its
    # smaller error and look misleadingly similar to the "before".
    all_x = np.concatenate([d for panel in (d1, d2) for d in (panel[0], panel[2])])
    all_y = np.concatenate([d for panel in (d1, d2) for d in (panel[1], panel[3])])
    pad_x = 0.05 * (all_x.max() - all_x.min() + 1)
    pad_y = 0.05 * (all_y.max() - all_y.min() + 1)
    for ax in (ax1, ax2):
        ax.set_xlim(all_x.min() - pad_x, all_x.max() + pad_x)
        ax.set_ylim(all_y.min() - pad_y, all_y.max() + pad_y)

    fig.suptitle("Phase 1 - same route, before and after the drift fixes",
                 color=TITLE, fontsize=14)
    out = Path(args.out).expanduser()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, facecolor=fig.get_facecolor(), bbox_inches="tight")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
