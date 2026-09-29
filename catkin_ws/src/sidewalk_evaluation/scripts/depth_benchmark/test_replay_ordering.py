#!/usr/bin/env python3
"""test_replay_ordering.py - prove the offline replay labels the same ball the
live recorder labels, without needing a camera or a recording.

WHY THIS TEST EXISTS

    The offline replay used to re-sort the three fitted spheres by z before
    writing its row. The two lower balls are level to within the detector's own
    z_tol of 0.08 m, so their z order is noise - and when it disagreed with
    their y order, `leg1` and `n_bot1` in a replayed file described a DIFFERENT
    BALL from `leg1` and `n_bot1` in a live file.

    That failure is invisible. No error, no warning, no obviously wrong number:
    `base` is unaffected because it is the distance between the two lower balls
    either way, and the legs are within a few millimetres of each other on a
    symmetric target. Any per-ball comparison between a live run and a replayed
    one would simply have been comparing different spheres.

    So it gets a test, and the test builds the disagreement on purpose: the
    bottom pair is placed with the LEFT ball slightly HIGHER than the right, so
    sorting by z and sorting by y give opposite answers.

  usage:  python3 test_replay_ordering.py       (exit 0 = pass)
"""
from __future__ import print_function

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sphere_centroid import detect_frame, centroid_of

R = 0.20
DET = dict(radius=R, radius_tol=0.07, thresh=0.03, iters=1500, min_inliers=10,
           base=1.10, rise=1.10, tol=0.06, max_candidates=7, z_tol=0.08)

# The three true centres. x is range, y is left-right, z is up.
#   bottom LEFT  (y = -0.55) is placed 30 mm HIGHER than
#   bottom RIGHT (y = +0.55)
# so sorting the pair by z puts RIGHT first while sorting by y puts LEFT first.
# 30 mm is well inside the detector's z_tol of 80 mm, so the frame still passes
# its level check - which is exactly the situation that occurs in real data.
TRUE = np.array([[4.00, -0.55, 0.380],     # bottom left,  higher
                 [4.00, +0.55, 0.350],     # bottom right, lower
                 [4.00,  0.00, 1.465]])    # apex


def sphere_cap(centre, n=900, seed=0):
    """Points on the camera-facing cap of a sphere, as a real sensor returns."""
    rng = np.random.default_rng(seed)
    v = rng.normal(size=(n * 6, 3))
    v /= np.linalg.norm(v, axis=1, keepdims=True)
    v = v[v[:, 0] < -0.15][:n]              # keep the -x facing cap only
    return centre + R * v


def build_cloud():
    parts = [sphere_cap(TRUE[i], seed=10 + i) for i in range(3)]
    return np.vstack(parts).astype(np.float64)


def main():
    pts = build_cloud()
    res = detect_frame(pts, rng=np.random.default_rng(7), **DET)
    if res is None:
        print('FAIL: the detector did not find the frame in the synthetic cloud')
        return 1
    centers, d = res
    ok = True

    # --- 1. the canonical order the LIVE recorder relies on
    apex_i = int(np.argmax(centers[:, 2]))
    if apex_i != 2:
        print('FAIL: apex is at index %d, expected 2' % apex_i)
        ok = False
    if not centers[0, 1] < centers[1, 1]:
        print('FAIL: bottom pair not ordered by y  (%.3f then %.3f)'
              % (centers[0, 1], centers[1, 1]))
        ok = False
    else:
        print('pass: bottom pair ordered by y   (%.3f, then %.3f)'
              % (centers[0, 1], centers[1, 1]))

    # --- 2. the test's whole point: z order and y order DISAGREE here
    if centers[0, 2] > centers[1, 2]:
        print('pass: z order and y order disagree, as intended '
              '(first ball is %.0f mm higher)'
              % ((centers[0, 2] - centers[1, 2]) * 1000))
    else:
        print('FAIL: the fixture did not create the disagreement it needs')
        ok = False

    # --- 3. sides come back in canonical order, matching the CSV columns
    base = float(np.linalg.norm(centers[0] - centers[1]))
    leg1 = float(np.linalg.norm(centers[0] - centers[2]))
    leg2 = float(np.linalg.norm(centers[1] - centers[2]))
    for name, got, want in (('base', d[0], base), ('leg1', d[1], leg1),
                            ('leg2', d[2], leg2)):
        if abs(got - want) > 1e-9:
            print('FAIL: %s from detect_frame is %.6f, recomputed %.6f'
                  % (name, got, want))
            ok = False
    if ok:
        print('pass: d = [base, leg1, leg2] in canonical order')

    # --- 4. what the OLD replay would have written, for the record
    zsorted = centers[np.argsort(centers[:, 2])]
    old_leg1 = float(np.linalg.norm(zsorted[0] - zsorted[2]))
    if abs(old_leg1 - leg1) > 1e-9:
        print('pass: the old z-sort DID mislabel it — its leg1 was %.4f m '
              '(ball at y=%+.2f), the live recorder\'s is %.4f m (ball at '
              'y=%+.2f); a %.1f mm difference between two files that both '
              'call the column leg1'
              % (old_leg1, zsorted[0, 1], leg1, centers[0, 1],
                 abs(old_leg1 - leg1) * 1000))
    else:
        print('note: the z-sort happened to agree on this fixture')

    # --- 5. the centroid is order-independent, and must stay so
    c1 = centroid_of(centers)
    c2 = centroid_of(centers[[2, 0, 1]])
    if np.max(np.abs(np.asarray(c1) - np.asarray(c2))) > 1e-12:
        print('FAIL: the centroid depends on the order of its inputs')
        ok = False
    else:
        print('pass: the centroid is unaffected by ordering, as expected')

    print()
    print('RESULT %s' % ('PASS' if ok else 'FAIL'))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
