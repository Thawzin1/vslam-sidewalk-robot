#!/usr/bin/env python3
"""27 Sept (after s2_nav_01 route B touched a wall post): how thin can drive 10's saved walls be made for the LOCAL
costmap so that (1) every pose the robot really drove today, and drive 10's own corrected path, is collision-free
with the Husky outline, and (2) the wall corner posts are still there. Prints per erosion level (cells of 5 cm)."""
import csv, math, numpy as np, os, sys
from scipy import ndimage
HERE = os.path.dirname(os.path.abspath(__file__))
def load_pgm(p):
    with open(p, "rb") as f:
        assert f.readline().strip() == b"P5"
        l = f.readline()
        while l.startswith(b"#"): l = f.readline()
        w, h = map(int, l.split()); mx = int(f.readline()); d = np.frombuffer(f.read(), np.uint8).reshape(h, w)
    return np.flipud(d)                     # row 0 = bottom (map y origin)
img = load_pgm(os.path.join(HERE, "maps/d10_nav.pgm")); res = 0.05; ox, oy = -10.525, -21.69523
occ = img < 100                             # trinary: 0 = occupied, 254 free, 205 unknown
def poses(p):
    out = []
    for r in csv.DictReader(open(p)): out.append((float(r["x"]), float(r["y"]), float(r["yaw"])))
    return out
runs = {}
for d in ("nav_A", "nav_A_try1_range4m", "nav_B", "nav_B_part2"):
    p = os.path.expanduser("~/.run_records/s2_nav_01/%s/actual.csv" % d)
    if os.path.exists(p): runs[d] = poses(p)[::5]
tum = os.path.expanduser(os.path.join(os.environ.get("RESULTS_DIR", os.path.expanduser("~/vslam-sidewalk-robot/results")), "series2_static/s2_static_10/camera_corrected.tum"))
if os.path.exists(tum):
    P = []
    for l in open(tum):
        if l.startswith("#") or not l.strip(): continue
        t, x, y, z, qx, qy, qz, qw = map(float, l.split()[:8]); P.append((x, y, math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))))
    runs["drive10"] = P
L, W = 0.99, 0.67                            # Husky outline, no padding (the local check's own)
fx = np.linspace(-L / 2, L / 2, 21); fy = np.linspace(-W / 2, W / 2, 15)
FX, FY = np.meshgrid(fx, fy); FX = FX.ravel(); FY = FY.ravel()
def hits(o, x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    gx = ((x + c * FX - s * FY - ox) / res).astype(int); gy = ((y + s * FX + c * FY - oy) / res).astype(int)
    ok = (gx >= 0) & (gy >= 0) & (gx < o.shape[1]) & (gy < o.shape[0])
    return int(o[gy[ok], gx[ok]].sum())
touch = (14.29, -13.46, -2.31)             # where the front-left corner met the post (16:36)
for k in range(0, 6):
    o = ndimage.binary_erosion(occ, iterations=k) if k else occ
    bad = {n: sum(1 for p in ps if hits(o, *p) > 0) for n, ps in runs.items()}
    print("erode %d cells (%.2f m): occupied %6d | poses whose outline overlaps a wall: %s | touch pose overlaps: %d cells"
          % (k, k * res, int(o.sum()), bad, hits(o, *touch)))
