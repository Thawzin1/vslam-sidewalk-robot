#!/usr/bin/env python3
"""score_map_quality.py - score occupancy maps objectively, so runs can be ranked.

WHY

    "The walls look blobby" is a real observation and an unusable one. This
    turns it into a number that can be compared between runs.

WHAT IS MEASURED, AND WHY EACH ONE

    THICKNESS    scan every row and every column; record the length of each
                 unbroken run of occupied cells. A wall seen edge-on is 1-3
                 cells (5-15 cm). A run of 20 cells is a metre of "wall" that
                 is not there. Reported as the median and the fraction over
                 0.30 m, because the median stays healthy while the tail goes
                 wrong - run 9a's median was a correct 0.15 m while 29.5 % of
                 its runs exceeded 0.30 m.

    SPECKLE      occupied cells with 0-1 occupied neighbours. This is what the
                 noise filter removes, and it was already at 0.9 % on run 9a,
                 so it is reported to prove a change did not make it worse.

    FREE/OCC     free cells per occupied cell. Ray tracing carves free space;
                 a setting that marks more of the room as obstacle will drop
                 this ratio even when thickness looks unchanged.

    COVERAGE     total known cells. A "cleaner" map that simply mapped less of
                 the room is not better, and this is the guard against reading
                 a smaller map as an improvement.

    THE HONEST COMPARISON RULE

    These are only comparable between runs that drove the SAME ROUTE. A longer
    route maps more room and produces more of everything. The script prints
    path length from poses.csv so a mismatched pair is visible rather than
    silently averaged.

    usage:
      score_map_quality.py "Week 7/09c_base" "Week 7/09d_normals" ...
"""
from __future__ import print_function

import csv
import math
import os
import sys

import numpy as np


def find(base, suffix):
    for root, _, files in os.walk(base):
        for f in files:
            if f.endswith(suffix):
                return os.path.join(root, f)
    return None


def runs_of_true(mask):
    """Lengths of every unbroken run, along rows and along columns."""
    out = []
    for arr in (mask, mask.T):
        for line in arr:
            n = 0
            for v in line:
                if v:
                    n += 1
                elif n:
                    out.append(n); n = 0
            if n:
                out.append(n)
    return np.array(out) if out else np.array([0])


def score(base):
    npy = find(base, "_grid.npy")
    if npy is None:
        print("  %-22s NO GRID FILE FOUND" % os.path.basename(base.rstrip("/")))
        return None
    # A grid file can exist and be EMPTY. bench_fix_normalK40's four artefact
    # noticed until the file was loaded five days later. Report it as a missing
    # measurement, do not crash, and never let it read as a score of zero.
    try:
        g = np.load(npy)
    except Exception as exc:
        print("  %-22s GRID UNREADABLE (%s) - %d bytes. NOT a zero score, a MISSING one."
              % (os.path.basename(base.rstrip("/")), str(exc)[:34], os.path.getsize(npy)))
        return None
    if g.size == 0:
        print("  %-22s GRID IS EMPTY - missing measurement, not a zero"
              % os.path.basename(base.rstrip("/")))
        return None
    res = 0.05
    yml = npy.replace("_grid.npy", "_grid.yaml")
    if os.path.exists(yml):
        for line in open(yml):
            if line.startswith("resolution:"):
                res = float(line.split(":")[1])

    occ = (g == 100)
    free = (g == 0)
    r = runs_of_true(occ)

    pad = np.zeros((g.shape[0] + 2, g.shape[1] + 2), dtype=bool)
    pad[1:-1, 1:-1] = occ
    n = np.zeros_like(pad, dtype=np.int16)
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dy or dx:
                n += np.roll(np.roll(pad, dy, 0), dx, 1).astype(np.int16)
    neigh = n[1:-1, 1:-1][occ] if occ.sum() else np.array([0])

    path = float("nan")
    p = os.path.join(base, "poses.csv")
    if os.path.exists(p):
        P = list(csv.DictReader(open(p)))
        xs = [float(q["x"]) for q in P]
        ys = [float(q["y"]) for q in P]
        path = sum(math.hypot(xs[i+1]-xs[i], ys[i+1]-ys[i])
                   for i in range(len(xs)-1))

    # THRESHOLD IN WHOLE CELLS, NEVER metres/res.
    #   0.30 / 0.05 evaluates to 5.999999999999999 in floating point, so
    #   "r > 0.30/res" silently means "r >= 6" instead of "r > 6". That one bit
    #   moved run 9a's answer from 29.5 % to 34.0 % and made two of this
    #   project's own tools disagree about the same map.
    THICK_CELLS = 6                      # 6 cells x 0.05 m = 0.30 m
    thick = int((r > THICK_CELLS).sum())

    return {
        "name": os.path.basename(base.rstrip("/")),
        "occ": int(occ.sum()),
        "free": int(free.sum()),
        "known": int(occ.sum() + free.sum()),
        "median_m": float(np.median(r)) * res,
        "p90_m": float(np.percentile(r, 90)) * res,
        "max_m": float(r.max()) * res,
        "frac_thick": 100.0 * thick / len(r),
        "speckle": 100.0 * float((neigh <= 1).sum()) / max(len(neigh), 1),
        "ratio": float(free.sum()) / max(occ.sum(), 1),
        "path": path,
    }


rows = [s for s in (score(b) for b in sys.argv[1:]) if s]
if not rows:
    sys.exit("  no runs with a *_grid.npy found in the given directories")

print("=" * 104)
print("  MAP QUALITY - lower thickness is better, but only compare runs that drove the SAME route")
print("=" * 104)
print("  %-18s %8s %8s %9s %8s %9s %8s %8s %9s"
      % ("run", "path m", "median", "worst", ">0.30m", "speckle",
         "free/occ", "OCCUPIED", "known"))
for s in rows:
    print("  %-18s %8.1f %7.2fm %8.2fm %7.1f%% %8.1f%% %8.2f %8d %9d"
          % (s["name"], s["path"], s["median_m"], s["max_m"],
             s["frac_thick"], s["speckle"], s["ratio"], s["occ"], s["known"]))

if len(rows) > 1:
    base = rows[0]
    print()
    print("  CHANGE AGAINST %s" % base["name"])
    for s in rows[1:]:
        dt = s["frac_thick"] - base["frac_thick"]
        dk = 100.0 * (s["known"] - base["known"]) / max(base["known"], 1)
        dp = 100.0 * (s["path"] - base["path"]) / max(base["path"], 1e-9)

        # THE ROUTE MUST MATCH, IN BOTH DIRECTIONS. The first version only
        # warned when a run mapped LESS than the baseline. It then called a run
        # that drove 69.7 m against the baseline's 23.3 m "BETTER" - three times
        # the route, forty percent more coverage, and no warning at all. A run
        # that covers MORE is exactly as incomparable as one that covers less.
        # nan path means a STATIONARY test - there is no route to mismatch, so
        # judge those on coverage alone. The first version compared nan and got
        # False from every inequality, so it silently let mismatched pairs
        # through while loudly rejecting others. A comparison that cannot be
        # made must fail closed, not open.
        have_path = (dp == dp) and (base["path"] == base["path"]) and base["path"] > 0.1
        bad_route = (abs(dk) > 15) or (have_path and abs(dp) > 20)
        if bad_route:
            print("    %-18s *** NOT COMPARABLE: path %+.0f %%, coverage %+.0f %%"
                  % (s["name"], dp, dk))
            print("    %-18s     (%.1f m vs the baseline's %.1f m). A different"
                  % ("", s["path"], base["path"]))
            print("    %-18s     route measures the DRIVE, not the setting."
                  % "")
            print("    %-18s     Re-run it on the same route before comparing."
                  % "")
            continue

        do = 100.0 * (s["occ"] - base["occ"]) / max(base["occ"], 1)
        verdict = "BETTER" if dt < -2 else ("worse" if dt > 2 else "no real change")

        # A MAP THAT LOST ITS WALLS SCORES "THIN". filter_r05_n5 reads 0.8 %
        # thick - the best number in the whole set - and it is the setting
        # documented to have deleted the black panel down to a single cell.
        # Thickness alone cannot tell a clean wall from a missing one.
        if do < -20 and dt < 0:
            print("    %-18s thick %+6.1f pp BUT %+.0f %% OCCUPIED CELLS - it is not"
                  % (s["name"], dt, do))
            print("    %-18s     thinner, it has LOST STRUCTURE. Not an improvement."
                  % "")
            continue
        print("    %-18s thick-fraction %+6.1f pp   occupied %+6.1f %%   coverage %+6.1f %%   %s"
              % (s["name"], dt, do, dk, verdict))
    print()
    print("  A change under about 2 percentage points is not a result. Two maps")
    print("  of the same room never come out identical, and the difference has")
    print("  to be bigger than that wobble before it means anything.")
