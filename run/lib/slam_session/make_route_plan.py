#!/usr/bin/env python3
"""make_route_plan.py OPTMAP.npz D10_CORRECTED.tum OUT.png - the SLAM session's route on drive 10's saved 2D map.
OPTMAP.npz: drive 10's Admin.opt_map (the camera map's own 2D occupancy grid, decoded read-only from the master:
g int8 -1 unknown / 0 free / 100 occupied, xm, ym, res). Candidate new areas are where drive 10's grid is free or
unknown BEYOND the end of its path (it looked in but never drove in)."""
import sys, numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
d = np.load(sys.argv[1]); g = d["g"]; xm, ym, res = float(d["xm"]), float(d["ym"]), float(d["res"])
T = np.loadtxt(sys.argv[2])
img = np.full(g.shape + (3,), 0.85); img[g == 0] = 1.0; img[g == 100] = 0.15
fig, ax = plt.subplots(figsize=(10, 11))
ext = [xm, xm + g.shape[1] * res, ym, ym + g.shape[0] * res]
ax.imshow(img, origin="lower", extent=ext, interpolation="nearest")
ax.plot(T[:, 1], T[:, 2], "--", color="0.45", lw=1, label="drive 10's path (camera map, corrected) - dashed")
cands = {"A": (13.9, 10.0, 2.2, 6.0, "tab:red", (16.4, 15.6), "A: corridor continues north\n(drive 10 turned at ~9.8 m;\nseen free to ~16 m)"),
         "B": (-6.0, -15.9, 5.3, 2.8, "tab:red", (-5.9, -16.3), "B: corridor continues west past\nthe bottom-left corner (seen ~6 m)"),
         "D": (-0.6, 1.0, 2.2, 7.4, "tab:purple", (1.9, 7.8), "D: corridor north of the start\nmark (seen to ~8 m) - last,\nbefore park"),
         "E": (15.9, -13.4, 5.1, 1.9, "tab:orange", (15.9, -10.3), "E (optional): corridor east\nat the bottom-right corner")}
for k, (x, y, w, h, c, (tx, ty), txt) in cands.items():
    ax.add_patch(Rectangle((x, y), w, h, fill=False, ec=c, lw=2.2))
    ax.text(tx, ty, txt, color=c, fontsize=8, va="top", fontweight="bold",
            bbox=dict(fc="white", ec="none", alpha=0.8, pad=1))
stops = [("S1", 14.8, 0.0), ("S2", 14.8, 15.5), ("S3", 14.8, -14.9), ("S4", 0.3, -14.8), ("S5", -5.0, -14.5),
         ("S6", 0.5, 8.0)]
for n, x, y in stops:
    ax.plot(x, y, "s", ms=10, mfc="gold", mec="k", zorder=5)
    ax.text(x + 0.35, y + 0.35, n, fontsize=9, fontweight="bold", zorder=6)
ax.annotate("", (3, -14.3), (12, -14.3), arrowprops=dict(arrowstyle="->", color="tab:blue", lw=2.5))
ax.text(4.5, -13.9, "P2: person walks ahead\nof the robot (~10 m)", color="tab:blue", fontsize=8, fontweight="bold")
ax.text(12.3, -17.2, "P1: person crosses\nin front, robot parked", color="tab:blue", fontsize=8, fontweight="bold")
ax.plot(0, 0, "o", ms=12, mfc="k", mec="w", zorder=6)
ax.text(-5.8, -2.2, "START and PARK: the start mark,\nfacing east (x+)", fontsize=9, fontweight="bold",
        bbox=dict(fc="white", ec="k", alpha=0.9))
route = np.array([[0, 0], [14.8, 0], [14.8, 15.5], [14.8, -14.9], [0.3, -14.8], [-5.0, -14.5], [0.3, -14.8], [0.1, 0],
                  [0.5, 8.0], [0.0, 0.0]])
ax.plot(route[:, 0] + 0.25, route[:, 1] + 0.25, "-", color="tab:green", lw=2.5, alpha=0.55,
        label="today's route: east, north (A), south, west (B), north, north (D), park")
for (x0, y0), (x1, y1) in zip(route[:-1], route[1:]):
    ax.annotate("", ((x0 + x1) / 2 + 0.25 + (x1 - x0) * 0.04, (y0 + y1) / 2 + 0.25 + (y1 - y0) * 0.04),
                ((x0 + x1) / 2 + 0.25, (y0 + y1) / 2 + 0.25), arrowprops=dict(arrowstyle="->", color="tab:green", lw=2))
ax.text(5.5, 11.5, "S = 30 s stop, hands off\nS2, S5, S6 = end of a new area:\nstop 30 s, then REVERSE out\n(do not turn round inside)",
        fontsize=9, bbox=dict(fc="lightyellow", ec="k"))
ax.set_xlim(-6.5, 21.5); ax.set_ylim(-20.5, 17); ax.set_aspect("equal"); ax.grid(alpha=0.3)
ax.set_xlabel("x (m) - drive 10's map frame (start mark = 0, 0)"); ax.set_ylabel("y (m)")
ax.set_title("SLAM session plan on drive 10's saved 2D map (white = seen free, black = wall, grey = never seen)\n"
             "boxes = NEW areas drive 10 only looked into (free cells beyond its path); keep each under 10 m in",
             fontsize=10)
ax.legend(loc="upper left", fontsize=8)
fig.tight_layout(); fig.savefig(sys.argv[3], dpi=130)
print("wrote", sys.argv[3])
