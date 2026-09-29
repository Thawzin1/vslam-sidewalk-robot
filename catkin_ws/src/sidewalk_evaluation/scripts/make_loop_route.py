#!/usr/bin/env python3
"""make_loop_route.py - design a clean lap route that fits inside the room.

WHY NOT JUST REPLAY A PREVIOUS DRIVE

    The first attempt took run 9a's first three laps and used them as the route.
    Drawn on the map it came out a tangle - the robot had wandered, doubled back
    and cut across, so the "route" crossed itself a dozen times. Unfollowable,
    and worse than no guide because it looks authoritative.

    Plain terms: a recording of someone wandering around a room is not
    directions.

HOW THIS ONE IS BUILT

    1. Take the free space the map actually found - the floor the robot has
       already driven on, so it is known-drivable, not assumed.
    2. Shrink it inward by the robot's half-width plus a margin. What survives
       is floor whose CENTRE the robot can occupy without any part of it
       touching an obstacle.
    3. Keep the piece containing the start point - the room the robot is in,
       not the corridor outside.
    4. Trace the outline of that piece. That outline is a loop, it stays clear
       of every wall by construction, and driving it is one lap of the room.
    5. Smooth it, so it is a route rather than a staircase of grid cells.

    usage:
      make_loop_route.py "Week 7/09a_run9a" --clearance 0.55 --out config/route_lab_loop.csv
"""
from __future__ import print_function

import argparse
import csv
import math
import os
import sys

import numpy as np

try:
    import cv2
except ImportError:
    sys.exit("  cv2 is required for this (it does the shrinking and tracing)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir")
    ap.add_argument("--clearance", type=float, default=0.55,
                    help="metres to keep between the robot's centre and anything solid. "
                         "The Husky is about 0.99 m wide, so half of that is 0.50 m; "
                         "0.55 leaves a little margin.")
    ap.add_argument("--start", default="0,0",
                    help="x,y of the start mark in map metres")
    ap.add_argument("--spacing", type=float, default=0.30)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    npy = None
    for root, _, files in os.walk(os.path.join(a.run_dir, "maps")):
        for f in files:
            if f.endswith("_grid.npy"):
                npy = os.path.join(root, f)
    if npy is None:
        sys.exit("  no *_grid.npy under %s/maps" % a.run_dir)

    g = np.load(npy)
    res, ox, oy = 0.05, -10.525, -10.525
    yml = npy.replace("_grid.npy", "_grid.yaml")
    if os.path.exists(yml):
        for line in open(yml):
            if line.startswith("resolution:"):
                res = float(line.split(":")[1])
            if line.startswith("origin:"):
                p = line.split("[")[1].split("]")[0].split(",")
                ox, oy = float(p[0]), float(p[1])

    free = (g == 0).astype(np.uint8)
    print("  free cells: %d   grid %dx%d at %.2f m" % (free.sum(), g.shape[1], g.shape[0], res))

    # 2. shrink inward by the clearance
    k = int(round(a.clearance / res))
    if k < 1:
        k = 1
    ker = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * k + 1, 2 * k + 1))
    safe = cv2.erode(free, ker)
    print("  after keeping %.2f m clear: %d cells remain" % (a.clearance, int(safe.sum())))
    if safe.sum() < 50:
        sys.exit("  almost nothing survives that clearance - the room is too "
                 "cluttered, or the clearance is too big. Try a smaller value.")

    # 3. the piece containing the start
    sx, sy = [float(v) for v in a.start.split(",")]
    scol = int((sx - ox) / res)
    srow = int((sy - oy) / res)
    n, lab = cv2.connectedComponents(safe)
    want = lab[srow, scol] if (0 <= srow < lab.shape[0] and 0 <= scol < lab.shape[1]) else 0
    if want == 0:
        # The start sits inside the clearance band. Take the piece nearest it.
        ys, xs = np.nonzero(safe)
        d = (ys - srow) ** 2 + (xs - scol) ** 2
        j = int(np.argmin(d))
        want = lab[ys[j], xs[j]]
        print("  the start mark itself is inside the clearance band; using the "
              "nearest drivable piece, %.2f m away" % (math.sqrt(d[j]) * res))
    piece = (lab == want).astype(np.uint8)
    print("  room piece: %d cells = %.1f m2" % (int(piece.sum()), piece.sum() * res * res))

    # 4. trace its outline
    cs, _ = cv2.findContours(piece, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2:]
    if not cs:
        sys.exit("  no outline found")
    c = max(cs, key=cv2.contourArea)
    peri = cv2.arcLength(c, True)
    c = cv2.approxPolyDP(c, 0.012 * peri, True)          # 5. smooth
    pts = [(float(p[0][0]), float(p[0][1])) for p in c]
    print("  outline: %.2f m around, %d corners after smoothing" % (peri * res, len(pts)))

    # rotate so the loop starts nearest the start mark
    di = [((x - scol) ** 2 + (y - srow) ** 2) for x, y in pts]
    i0 = int(np.argmin(di))
    pts = pts[i0:] + pts[:i0]
    pts.append(pts[0])                                   # close the loop

    # resample to even spacing in metres
    out = []
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        seg = math.hypot(x1 - x0, y1 - y0) * res
        steps = max(1, int(seg / a.spacing))
        for s in range(steps):
            t = float(s) / steps
            out.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t))
    out.append(pts[-1])

    d = os.path.dirname(a.out)
    if d and not os.path.isdir(d):
        os.makedirs(d)
    total = 0.0
    with open(a.out, "w") as fh:
        fh.write("x,y\n")
        prev = None
        for cx, cy in out:
            wx, wy = cx * res + ox, cy * res + oy
            fh.write("%.3f,%.3f\n" % (wx, wy))
            if prev:
                total += math.hypot(wx - prev[0], wy - prev[1])
            prev = (wx, wy)

    print()
    print("  wrote %s" % a.out)
    print("  %d points, one lap = %.2f m, min clearance %.2f m by construction"
          % (len(out), total, a.clearance))
    print()
    print("  This loop follows floor the robot has ALREADY driven on, shrunk")
    print("  inward, so every point of it is known-drivable. It is a guide in")
    print("  map coordinates - follow its shape, not its pixels, because the")
    print("  map's origin is where tracking began, not the floor mark.")


if __name__ == "__main__":
    main()
