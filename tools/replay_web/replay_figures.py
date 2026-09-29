#!/usr/bin/env python3
"""replay_figures.py - the pictures and summary numbers for a finished replay job.

WHAT IT DOES
    slam_trajectory_figure()  a top-down picture of a mapping replay's path. The replay is drawn
                              dashed, the original drive (if its path files exist) solid; the
                              camera's tracking alone in grey, the map's corrected positions in colour.
    bench_mode_summary()      for one depth mode of a camera bench test: how many frames were
                              replayed and accepted, the sphere target's centre and its spread.
    bench_scatter_figure()    a top-down scatter of that centre, one dot per accepted frame.

HOW TO RUN
    Used by replay_job.py. By hand:
      python3 replay_figures.py slam  <replay out dir> [<original dir>]   -> trajectory.png
      python3 replay_figures.py bench <centres.csv> <MODE>                 -> <csv>.png + summary

INPUTS
    camera.tum / camera_corrected.tum (TUM format: time x y z qx qy qz qw per line), in the replay's
    output folder and, optionally, in a folder with the original drive's two files.
    centres_<MODE>.csv written by svo_replay_n.py (columns n,x,y,z,...; x ahead, y sideways, metres).

OUTPUTS   PNG pictures, and plain dictionaries of numbers for replay_job.py to store.

Wording rule for everything written here: a replay is compared with the original drive as
"agreement with the original drive", never as an "error" - neither of them is ground truth.
"""
import csv
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")          # no screen: draw straight into files
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

TRACKING_COLOUR = "#8a8f98"     # grey: the camera's tracking alone (never corrected)
CORRECTED_COLOUR = "#1f77b4"    # colour: the map's corrected positions


def read_tum(path):
    """[(t, x, y)] from a TUM file; an empty list if the file is missing."""
    rows = []
    try:
        with open(path) as f:
            for line in f:
                parts = line.split()
                if len(parts) >= 8 and not parts[0].startswith("#"):
                    rows.append((float(parts[0]), float(parts[1]), float(parts[2])))
    except OSError:
        pass
    return rows


def cut_to_window(rows, t0, t1):
    return [r for r in rows if t0 <= r[0] <= t1]


def rigid_align_2d(moving, fixed, max_dt=0.2):
    """Rotate and shift `moving` onto `fixed` (no scaling), using poses matched by time.

    The two paths start from different map origins (a replay starts its map at the first frame
    of the camera file, the drive at the start of the drive), so they must be brought into one
    frame before they can be drawn together. Same idea as evo_ape -a: one rotation and one
    shift for the whole path, found by least squares. Returns the moved rows, or the rows
    unchanged if fewer than 3 time matches exist."""
    if len(moving) < 3 or len(fixed) < 3:
        return moving
    fixed_t = np.array([r[0] for r in fixed])
    pairs_a, pairs_b = [], []
    for t, x, y in moving:
        i = int(np.argmin(np.abs(fixed_t - t)))
        if abs(fixed_t[i] - t) <= max_dt:
            pairs_a.append((x, y))
            pairs_b.append(fixed[i][1:])
    if len(pairs_a) < 3:
        return moving
    a, b = np.array(pairs_a), np.array(pairs_b)
    ca, cb = a.mean(axis=0), b.mean(axis=0)
    u, _, vt = np.linalg.svd((a - ca).T @ (b - cb))
    rot = (u @ vt).T
    if np.linalg.det(rot) < 0:          # a reflection is not a rigid move
        vt[-1] *= -1
        rot = (u @ vt).T
    shift = cb - rot @ ca
    moved = (rot @ np.array([[x, y] for _, x, y in moving]).T).T + shift
    return [(t, float(p[0]), float(p[1])) for (t, _, _), p in zip(moving, moved)]


def draw_path(ax, rows, colour, style, label):
    if rows:
        ax.plot([r[1] for r in rows], [r[2] for r in rows], style, color=colour, lw=1.4, label=label)


def slam_trajectory_figure(replay_dir, original_dir, png_path):
    """Top-down path picture. Returns a short note saying what was drawn."""
    replay_track = read_tum(os.path.join(replay_dir, "camera.tum"))
    replay_corr = read_tum(os.path.join(replay_dir, "camera_corrected.tum"))
    if not replay_track and not replay_corr:
        return "no path files in the replay's folder, so no picture"
    window_rows = replay_track or replay_corr
    t0, t1 = window_rows[0][0], window_rows[-1][0]

    fig, ax = plt.subplots(figsize=(7, 6), dpi=110)
    note = "replay only"
    if original_dir:
        orig_track = cut_to_window(read_tum(os.path.join(original_dir, "camera.tum")), t0, t1)
        orig_corr = cut_to_window(read_tum(os.path.join(original_dir, "camera_corrected.tum")), t0, t1)
        orig_track = rigid_align_2d(orig_track, replay_track)
        orig_corr = rigid_align_2d(orig_corr, replay_corr)
        draw_path(ax, orig_track, TRACKING_COLOUR, "-", "original drive, tracking alone")
        draw_path(ax, orig_corr, CORRECTED_COLOUR, "-", "original drive, map's corrected positions")
        if orig_track or orig_corr:
            note = "replay and original drive (original cut to the replay's time window and moved onto it " \
                   "by one rotation + shift, no scaling)"
    draw_path(ax, replay_track, TRACKING_COLOUR, "--", "replay, tracking alone")
    draw_path(ax, replay_corr, CORRECTED_COLOUR, "--", "replay, map's corrected positions")
    start = window_rows[0]
    ax.plot(start[1], start[2], "o", color="#2f7d4f", ms=7, label="replay start")
    # at least 1 m across, so a parked replay (millimetres of wobble) does not look like a drive
    xs = [r[1] for r in replay_track + replay_corr]
    ys = [r[2] for r in replay_track + replay_corr]
    for low, high, set_lim in ((min(xs), max(xs), ax.set_xlim), (min(ys), max(ys), ax.set_ylim)):
        if high - low < 1.0:
            middle = (high + low) / 2
            set_lim(middle - 0.5, middle + 0.5)
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Path seen from above: solid = original drive, dashed = replay", fontsize=10)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, loc="best")
    fig.tight_layout()
    fig.savefig(png_path)
    plt.close(fig)
    return note


# ------------------------------------------------------------------ camera bench test
def read_centres(csv_path):
    """[(x, y, z)] of the sphere target's centre, one per accepted frame, in metres."""
    rows = []
    try:
        with open(csv_path) as f:
            for row in csv.DictReader(f):
                try:
                    rows.append((float(row["x"]), float(row["y"]), float(row["z"])))
                except (KeyError, ValueError):
                    continue
    except OSError:
        pass
    return rows


def bench_mode_summary(csv_path, frames_replayed):
    """Centre (x ahead, y sideways) and spread for one depth mode. Spread = standard deviation of
    the per-frame centre, in millimetres: how much the reading wobbles from frame to frame."""
    centres = read_centres(csv_path)
    summary = {"frames_replayed": frames_replayed, "frames_accepted": len(centres)}
    if not centres:
        return summary
    pts = np.array(centres)
    summary.update({
        "centre_x_ahead_m": round(float(pts[:, 0].mean()), 4),
        "centre_y_sideways_m": round(float(pts[:, 1].mean()), 4),
        "centre_z_up_m": round(float(pts[:, 2].mean()), 4),
        "spread_x_mm": round(float(pts[:, 0].std(ddof=1)) * 1000, 1) if len(pts) > 1 else None,
        "spread_y_mm": round(float(pts[:, 1].std(ddof=1)) * 1000, 1) if len(pts) > 1 else None,
        "spread_3d_mm": round(float(np.sqrt(((pts - pts.mean(axis=0)) ** 2).sum(axis=1).mean())) * 1000, 1),
    })
    return summary


def bench_scatter_figure(csv_path, mode, frames_replayed, png_path):
    """One dot per accepted frame: sideways (left-right) against ahead (distance), in mm from the
    mean. A tight cluster means a steady reading."""
    centres = read_centres(csv_path)
    fig, ax = plt.subplots(figsize=(4.6, 4.6), dpi=110)
    if centres:
        pts = np.array(centres)
        mean = pts.mean(axis=0)
        dy = (pts[:, 1] - mean[1]) * 1000
        dx = (pts[:, 0] - mean[0]) * 1000
        ax.scatter(dy, dx, s=10, color=CORRECTED_COLOUR, alpha=0.7)
        reach = max(20.0, float(np.abs(np.concatenate([dx, dy])).max()) * 1.15)
        ax.set_xlim(-reach, reach)
        ax.set_ylim(-reach, reach)
        rms = math.sqrt(float(((pts - mean) ** 2).sum(axis=1).mean())) * 1000
        ax.add_patch(plt.Circle((0, 0), rms, fill=False, color="#b8692a", lw=1.2))
        ax.set_title("%s: %d of %d frames\ncentre %.3f m ahead, %.3f m sideways (circle: %.1f mm spread)"
                     % (mode, len(pts), frames_replayed, mean[0], mean[1], rms), fontsize=9)
    else:
        ax.text(0.5, 0.5, "no sphere target found\nin %d replayed frames" % frames_replayed,
                ha="center", va="center", transform=ax.transAxes, fontsize=11)
        ax.set_title("%s: 0 of %d frames" % (mode, frames_replayed), fontsize=9)
    ax.set_aspect("equal")
    ax.set_xlabel("sideways from the mean (mm)")
    ax.set_ylabel("ahead from the mean (mm)")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(png_path)
    plt.close(fig)


if __name__ == "__main__":
    if len(sys.argv) >= 3 and sys.argv[1] == "slam":
        out = os.path.join(sys.argv[2], "trajectory.png")
        print(slam_trajectory_figure(sys.argv[2], sys.argv[3] if len(sys.argv) > 3 else None, out), "->", out)
    elif len(sys.argv) >= 4 and sys.argv[1] == "bench":
        csv_file, depth_mode = sys.argv[2], sys.argv[3]
        count = len(read_centres(csv_file))
        bench_scatter_figure(csv_file, depth_mode, count, csv_file + ".png")
        print(bench_mode_summary(csv_file, count))
    else:
        print(__doc__)
