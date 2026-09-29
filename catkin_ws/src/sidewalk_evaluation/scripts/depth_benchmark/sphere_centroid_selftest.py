#!/usr/bin/env python3
"""Offline validation of the PRODUCTION detection path — no ROS, no hardware.

Tests detect_frame() (what the live node calls), not the legacy detect_three_spheres().
Synthetic scenes are matched to the REAL measured data at ~4.2 m:
  - isosceles frame: base 1.10 m, rise 1.10 m, spheres r=0.20  -> sides 1.10/1.23/1.23
  - per-sphere point counts ~85/85/48  (the apex is genuinely sparse: it sits at the
    Helios +14.9 deg FOV edge, and that sparsity is what broke the first implementation)
  - ~160 background clutter points  (real ROI was ~380 pts, ~185 of them on spheres)
  - decoy spheres of correct radius at wrong positions, so the "best triple" logic is
    actually challenged rather than handed a clean scene

Reports, over many random seeds:
  A) detection rate, centroid-error distribution, and canonical-ordering correctness
  B) false-accept rate when a sphere is MISSING
  C) false-accept rate when spacing is WRONG

Run:  python3 sphere_centroid_selftest.py
"""
import sys, numpy as np
sys.path.insert(0, '/home/administrator/catkin_ws/src/matt_self_navigation/scripts')
from sphere_centroid import detect_frame, expected_sides

# ---- geometry of the real frame -------------------------------------------------
R, BASE, RISE = 0.20, 1.10, 1.10
DIST = 4.2                                   # frame distance, matches the real setup

# ---- detector params: MUST mirror sphere_centroid.launch ------------------------
P = dict(radius=R, radius_tol=0.07, thresh=0.03, iters=1500,
         min_inliers=10, base=BASE, rise=RISE, tol=0.06, max_candidates=7)

def front_cap(center, radius, n, noise, rng):
    """Points on the sensor-facing cap only (a lidar never sees the back of a sphere)."""
    view = -np.asarray(center, float)
    view /= np.linalg.norm(view)
    out = []
    while len(out) < n:
        d = rng.normal(size=3)
        d /= np.linalg.norm(d)
        if d.dot(view) > 0.15:
            out.append(center + radius * d + rng.normal(scale=noise, size=3))
    return np.array(out)

def frame_centers(base=BASE, rise=RISE, dist=DIST):
    """[bottom_-y, bottom_+y, apex] at `dist` metres ahead."""
    return np.array([[dist, -base / 2, 0.0],
                     [dist, +base / 2, 0.0],
                     [dist, 0.0, rise]])

def build_scene(rng, centers, n_pts, n_clutter=160, n_decoy=2, noise=0.015):
    """Frame spheres + background clutter + decoy spheres of the correct radius."""
    parts = [front_cap(c, R, n, noise, rng) for c, n in zip(centers, n_pts)]
    lo = np.array([DIST - 0.8, -1.3, -0.6])
    hi = np.array([DIST + 0.8, +1.3, +1.8])
    parts.append(rng.uniform(lo, hi, size=(n_clutter, 3)))          # background
    for _ in range(n_decoy):                                         # sphere-shaped decoys
        dc = rng.uniform(lo + 0.25, hi - 0.25)
        parts.append(front_cap(dc, R, int(rng.integers(25, 35)), noise, rng))
    cloud = np.vstack(parts)
    rng.shuffle(cloud)
    return cloud

def run(centers, n_pts, seeds, label, expect_detect):
    errs, ok_order, detected = [], 0, 0
    truth = centers.mean(axis=0)
    for s in range(seeds):
        rng = np.random.default_rng(1000 + s)
        cloud = build_scene(rng, centers, n_pts)
        res = detect_frame(cloud, rng=np.random.default_rng(5000 + s), **P)
        if res is None:
            continue
        detected += 1
        Pc, d = res
        errs.append(float(np.linalg.norm(Pc.mean(axis=0) - truth)))
        # canonical ordering: base shortest, apex highest
        if d[0] < d[1] and d[0] < d[2] and np.argmax(Pc[:, 2]) == 2:
            ok_order += 1
    rate = 100.0 * detected / seeds
    print('\n--- %s  (%d seeds) ---' % (label, seeds))
    if expect_detect:
        e = np.array(errs) * 1000 if errs else np.array([np.nan])
        print('  detection rate : %5.1f %%   (want high)' % rate)
        print('  centroid error : p50 %.1f mm | p95 %.1f mm | max %.1f mm' %
              (np.percentile(e, 50), np.percentile(e, 95), e.max()))
        print('  ordering OK    : %d / %d' % (ok_order, detected))
        good = rate >= 90 and np.percentile(e, 95) < 30 and ok_order == detected
    else:
        print('  FALSE-ACCEPT   : %5.1f %%   (want 0.0)' % rate)
        good = rate == 0.0
    print('  RESULT         : %s' % ('PASS' if good else 'FAIL'))
    return good

def main():
    print('expected sides: %s   detector params: iters=%d min_inl=%d tol=%.2f'
          % (np.round(expected_sides(BASE, RISE), 3), P['iters'], P['min_inliers'], P['tol']))
    results = []
    # A) realistic scene: bottoms dense, apex sparse (as measured on the real frame)
    results.append(run(frame_centers(), [85, 85, 48], 40,
                       'A. full frame, realistic density + decoys', True))
    # A2) harder: everything sparse, as it would be further away
    results.append(run(frame_centers(), [45, 45, 25], 40,
                       'A2. full frame, SPARSE (far-distance case)', True))
    # B) apex missing entirely -> must reject
    c2 = frame_centers()
    results.append(run(c2[:2], [85, 85], 30, 'B. apex MISSING -> must reject', False))
    # C) three spheres but wrong spacing -> must reject
    bad = frame_centers(base=0.75, rise=0.75)
    results.append(run(bad, [85, 85, 48], 30, 'C. WRONG spacing -> must reject', False))
    print('\n==================  OVERALL: %s  ==================' %
          ('PASS' if all(results) else 'FAIL'))
    return 0 if all(results) else 1

if __name__ == '__main__':
    sys.exit(main())
