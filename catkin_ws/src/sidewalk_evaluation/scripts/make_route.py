#!/usr/bin/env python3
"""make_route.py - turn a path you already drove into a route to follow.

WHY

    Runs 9c-9g must all drive the SAME short route, or the comparison between
    settings is really a comparison between drives. Describing a route in words
    ("two laps of the lab") leaves a lot of room; showing it on the live map
    while driving leaves much less.

    The route is not invented. It is taken from a path the robot has already
    driven and recorded - by default run 9a, which started on the same floor
    mark facing the same way.

HOW LAPS ARE FOUND

    A lap ends when the path comes back near where it started. So: walk the
    path, and every time it re-enters a small circle around the start point
    after having left it, that is one lap completed.

    Plain terms: you have done a lap when you get back to the mark.

THE CAVEAT THAT TRAVELS WITH THIS

    The route is in the map's own coordinates, and a map's origin is wherever
    the robot was standing when tracking began - NOT the floor mark. Two runs
    line up only as well as the parking does. So this is a GUIDE, accurate to
    a few tens of centimetres, not a survey. Follow its shape, not its pixels.

    usage:
      make_route.py "Week 7/09a_run9a" --laps 2 --out config/route_lab_2lap.csv
"""
from __future__ import print_function

import argparse
import csv
import math
import os
import sys


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir")
    ap.add_argument("--laps", type=int, default=2)
    ap.add_argument("--radius", type=float, default=1.2,
                    help="how near the start counts as 'back at the mark' (m)")
    ap.add_argument("--spacing", type=float, default=0.25,
                    help="distance between route points (m)")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    p = os.path.join(a.run_dir, "poses.csv")
    if not os.path.exists(p):
        sys.exit("  no poses.csv in %s" % a.run_dir)
    P = [(float(r["x"]), float(r["y"])) for r in csv.DictReader(open(p))]
    if len(P) < 10:
        sys.exit("  only %d poses - nothing to build a route from" % len(P))

    sx, sy = P[0]
    print("  %d poses, start (%.2f, %.2f)" % (len(P), sx, sy))

    # Walk the path. A lap completes on RE-ENTERING the circle after leaving it.
    laps_end = []
    outside = False
    for i, (x, y) in enumerate(P):
        d = math.hypot(x - sx, y - sy)
        if d > a.radius:
            outside = True
        elif outside:
            laps_end.append(i)
            outside = False
    print("  laps detected: %d  (returns to within %.1f m of the start)"
          % (len(laps_end), a.radius))
    for n, i in enumerate(laps_end[:6], 1):
        print("    lap %d completes at pose %d" % (n, i))

    if len(laps_end) < a.laps:
        print("  WANTED %d laps, FOUND %d. Using the whole path instead, and"
              % (a.laps, len(laps_end)))
        print("  saying so rather than silently returning something shorter.")
        cut = len(P)
    else:
        cut = laps_end[a.laps - 1] + 1

    seg = P[:cut]

    # Thin to roughly even spacing so the drawn line is smooth and small.
    out = [seg[0]]
    for x, y in seg[1:]:
        if math.hypot(x - out[-1][0], y - out[-1][1]) >= a.spacing:
            out.append((x, y))
    total = sum(math.hypot(out[i+1][0]-out[i][0], out[i+1][1]-out[i][1])
                for i in range(len(out)-1))

    d = os.path.dirname(a.out)
    if d and not os.path.isdir(d):
        os.makedirs(d)
    with open(a.out, "w") as fh:
        fh.write("x,y\n")
        for x, y in out:
            fh.write("%.3f,%.3f\n" % (x, y))

    print()
    print("  wrote %s" % a.out)
    print("  %d route points, %.2f m, covering %d lap(s)"
          % (len(out), total, min(a.laps, len(laps_end)) or 1))
    print()
    print("  REMINDER: the map's origin is where TRACKING began, not the floor")
    print("  mark. Treat this as a guide accurate to a few tens of centimetres.")


if __name__ == "__main__":
    main()
