#!/usr/bin/env python3
"""27 Sept 2026 (s2_nav_01 route B: the front-left corner touched a wall post the camera could not see).
maps/d10_nav_local.{pgm,yaml}: drive 10's saved walls for the LOCAL (wheel-check) costmap, with ONE change:
every wall cell that drive 10's own robot outline passed over is removed ("where the robot physically drove there is
no wall"). That opens the depth-smeared bands at the 1.0 m pinch (x 12-13 m) and at the start mark that made sim run 6
stall, and keeps every wall the robot never drove through - including the post it touched today. Simple thinning
(erosion or skeleton) was tried first and erased that post (thin_walls_check.py). Only drive 10's corrected path is
used, never today's route B poses (those were next to the post)."""
import math, numpy as np, os
from scipy import ndimage
exec(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "thin_walls_check.py")).read().split("L, W = 0.99")[0])
L, W = 0.99 + 0.20, 0.67 + 0.20       # the outline plus the 0.10 m safety padding all round (27 Sept)
fx = np.linspace(-L / 2, L / 2, 41); fy = np.linspace(-W / 2, W / 2, 29); FX, FY = np.meshgrid(fx, fy); FX = FX.ravel(); FY = FY.ravel()
def cells(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    gx = ((x + c * FX - s * FY - ox) / res).astype(int); gy = ((y + s * FX + c * FY - oy) / res).astype(int)
    ok = (gx >= 0) & (gy >= 0) & (gx < occ.shape[1]) & (gy < occ.shape[0]); return gy[ok], gx[ok]
swept = np.zeros_like(occ)
d10 = runs["drive10"]
for i in range(len(d10) - 1):                       # interpolate between saved nodes (they are ~0.3 m apart)
    (x0, y0, a0), (x1, y1, a1) = d10[i], d10[i + 1]
    n = max(1, int(math.hypot(x1 - x0, y1 - y0) / 0.05))
    da = math.atan2(math.sin(a1 - a0), math.cos(a1 - a0))
    if math.hypot(x1 - x0, y1 - y0) > 1.5: continue  # a jump in the saved path, not a drive
    for j in range(n + 1):
        t = j / n; gy_, gx_ = cells(x0 + t * (x1 - x0), y0 + t * (y1 - y0), a0 + t * da); swept[gy_, gx_] = True
local = occ & ~swept
out = np.full(occ.shape, 254, np.uint8); out[local] = 0
with open(os.path.join(HERE, "maps/d10_nav_local.pgm"), "wb") as f:
    f.write(b"P5\n# drive 10 walls minus drive 10's swept outline (make_local_walls.py)\n%d %d\n255\n" % (out.shape[1], out.shape[0])); f.write(np.flipud(out).tobytes())
open(os.path.join(HERE, "maps/d10_nav_local.yaml"), "w").write("image: d10_nav_local.pgm\nresolution: 0.050000\norigin: [-10.525000, -21.695230, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\nmode: trinary\n")
def hits(o, x, y, yaw):
    gy_, gx_ = cells(x, y, yaw); return int(o[gy_, gx_].sum())
bad = {k: sum(1 for p in ps if hits(local, *p) > 0) for k, ps in runs.items()}
d = ndimage.distance_transform_edt(~local) * res; gx = int((14.29 - ox) / res); gy = int((-13.46 - oy) / res)
print("local walls %d cells (saved %d, removed %d where drive 10 drove). poses overlapping: %s. touch spot: nearest wall %.2f m -> a turn on the spot there is %s (needs 0.60 m corner radius + 0.10 m margin)"
      % (local.sum(), occ.sum(), (occ & swept).sum(), bad, d[gy, gx], "REFUSED" if d[gy, gx] < 0.70 else "ALLOWED"))
