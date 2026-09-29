#!/usr/bin/env python3
"""sphere_reject_diagnose.py - WHY does a frame fail? One bucket per frame.

    Every number this project has published about the sphere frame is an
    acceptance rate: "38 of 200 frames gave a centroid". That tells us how often
    the detector succeeds and NOTHING about why it fails, because a frame that
    fails produces silence - the detector returns None and the reason dies
    inside it. This program supplies exactly that missing measurement, and the
    reason it matters is that the two candidate explanations demand opposite
    fixes:

      - if most failures are EMPTY_BOX or NO_SPHERE, the camera is not
        delivering enough usable 3D points on the spheres. That is a
        STEREO-DEPTH limitation: no amount of detector tuning fixes it, and the
        answer is exposure, depth mode, range, or a different sensor.

      - if most failures are FEW_CANDIDATES or SHAPE_GATE, the points ARE there
        and the geometry step is throwing them away. That is a DETECTOR
        limitation: the gates, the RANSAC budget or the tolerances are the
        problem, and the fix is in software.

    Right now we cannot tell those two apart. After this script we can.

    THE FIVE BUCKETS, in the order the pipeline reaches them. A frame lands in
    exactly one - the first gate it fails:

      1 EMPTY_BOX       fewer than 3*min_inliers points survived the ROI crop.
                        The box the detector looks in is essentially empty, so
                        there is not even enough material for three spheres.
      2 NO_SPHERE       the crop had points, but sequential RANSAC came back
                        with fewer than 3 sphere hypotheses. Points exist; none
                        of them are shaped like a 0.20 m ball.
      3 FEW_CANDIDATES  3 or more spheres were found, but no combination of
                        three of them had the right side lengths (1.10 / 1.23 /
                        1.23 m). We found balls; they are not OUR three balls.
      4 SHAPE_GATE      a triple passed the side-length test and then failed the
                        structure test - the bottom pair was not level, or the
                        apex was not the right height above it. A near miss: a
                        decoy with the right spacing, or a real frame with one
                        badly placed centre.
      5 ACCEPTED        a full detection. What the live node would have logged.

    HOW THIS RELATES TO sphere_centroid.py: it re-implements only the OUTER
    LOOP of detect_frame, and imports ransac_sphere / validate_triangle /
    expected_sides from that file unchanged. sphere_centroid.py is Nicolas's
    file, copied here byte-for-byte so the camera and the two LiDARs are scored
    by literally the same detector; MODIFYING IT IS FORBIDDEN, and adding a
    "reason" return value to detect_frame would be a modification. Re-writing
    the outer loop here is the only way to watch where the pipeline exits
    without touching his code. The inner maths - the RANSAC fit, the side-length
    gate, the expected triangle - is his, called directly, so the buckets
    describe the real detector and not a lookalike.

    THE --seeds OPTION, and why it is needed. RANSAC picks its
    sample points at random, so the same frame can succeed on one run and fail
    on the next with nothing changed but the random number generator. If a frame
    lands in a different bucket every time, its failure is the detector's own
    coin-flipping and could be bought off with more iterations. If it lands in
    the same bucket every time, the frame itself genuinely lacks the data, and
    more iterations would change nothing. --seeds N re-runs the SAME frames with
    N different seeds and reports how often the outcome held still.

    COST WARNING. The detector at iters=1500 on the ~95k points the camera puts
    in this box takes tens of seconds per frame. --seeds N multiplies that by N.
    A 40-frame, 5-seed run is a coffee-break job, not an interactive one, and
    the depth decode happens ONCE per frame (all seeds share the same points).

  usage, on the JETSON:
    python3 sphere_reject_diagnose.py <svo> <n_frames> <csv_out> [roi_x] [--seeds N]

    python3 sphere_reject_diagnose.py \
        ~/zedx_vs_lidar_data/svo/5m.svo2 40 /tmp/reject_5m.csv 5.0
    python3 sphere_reject_diagnose.py \
        ~/zedx_vs_lidar_data/svo/5m.svo2 20 /tmp/reject_5m.csv 5.0 --seeds 5
"""
import itertools
import os
import sys

import numpy as np
import pyzed.sl as sl

# Import from THIS directory, not a hardcoded ~/... path. The older
# scripts in this folder hardcode the Jetson's absolute path, which means they
# silently import a stale copy if one exists, and cannot be syntax-checked or
# unit-tested anywhere else. Resolving relative to __file__ makes the script
# import the sphere_centroid.py sitting beside it - always the right one.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sphere_centroid import (ransac_sphere, validate_triangle,  # noqa: E402
                             expected_sides)

# Box centre distance comes from argv. Width/height mirror the launch file:
# roi_sx 1.2, roi_sy 1.9, roi_sz 1.8 about roi_z 1.45 m above the floor, minus
# the measured camera height 0.6794 m - exactly as sphere_centroid_zedx.launch
# computes them. Same box as svo_mode_compare.py so the two are comparable.
ROI = dict(xmin=3.4, xmax=4.6, ymin=-0.95, ymax=0.95, zmin=-0.1294, zmax=1.6706)

# THE FROZEN DETECTOR. Every value copied from sphere_centroid_zedx.launch,
# which in turn copies Nicolas's params_used.txt. Change one of these and this
# script stops describing the runs it is meant to explain - the buckets would be
# counted for a detector that never recorded anything.
DET = dict(radius=0.20, radius_tol=0.07, thresh=0.03, iters=1500, min_inliers=10,
           base=1.10, rise=1.10, tol=0.06, max_candidates=7, z_tol=0.08)

# Bucket names in pipeline order. The order is the point: a frame is assigned
# the FIRST gate it fails, so reading the table top to bottom walks the frame
# through the pipeline and shows where the population drains away.
BUCKETS = ('EMPTY_BOX', 'NO_SPHERE', 'FEW_CANDIDATES', 'SHAPE_GATE', 'ACCEPTED')


def set_roi_x(roi_x):
    ROI['xmin'] = roi_x - 0.6
    ROI['xmax'] = roi_x + 0.6


def classify(pts, rng):
    """Run the detector's outer loop by hand and report WHERE it stopped.

    Returns (bucket_name, n_candidates, best_side_err).

    This is detect_frame() from sphere_centroid.py with two differences and no
    others: it records the exit point instead of returning None, and it keeps
    the closest-shaped triple even when that triple was rejected. The inner
    calls - ransac_sphere, validate_triangle, expected_sides - are his,
    unmodified, so any frame this function calls ACCEPTED is a frame the live
    node would also have accepted.

    best_side_err is the smallest side-length mismatch found over EVERY triple
    considered, passing or not: the sum of |measured - expected| across the
    three sorted sides, in metres. It is the one number that stays meaningful
    across buckets 3, 4 and 5, and it says how near the miss was. A
    FEW_CANDIDATES frame at 0.07 m was one hair outside the 0.06 m tolerance; a
    FEW_CANDIDATES frame at 0.9 m was never looking at the right object.
    """
    # Gate 1. The live node's own pre-check: three spheres need at least three
    # spheres' worth of inliers, so below that the frame is not worth searching.
    if len(pts) < 3 * DET['min_inliers']:
        return 'EMPTY_BOX', 0, float('nan')

    # Sequential RANSAC: find a sphere, delete its points, look again. Deleting
    # matters - without it every pass re-finds the same strongest ball.
    remaining = pts
    cands = []
    for _ in range(DET['max_candidates']):
        res = ransac_sphere(remaining, DET['radius'], DET['radius_tol'],
                            DET['thresh'], DET['iters'], DET['min_inliers'], rng)
        if res is None:
            break
        c, _, mask = res
        cands.append(c)
        remaining = remaining[~mask]

    # Gate 2. Fewer than three ball-shaped things in the box at all.
    if len(cands) < 3:
        return 'NO_SPHERE', len(cands), float('nan')

    exp = expected_sides(DET['base'], DET['rise'])
    best_err = float('inf')        # closest triple by shape, gates ignored
    passed_sides = False           # did ANY triple clear the side-length gate?
    best_accepted = None

    for combo in itertools.combinations(range(len(cands)), 3):
        P = np.array([cands[i] for i in combo])
        ok, d = validate_triangle(P, DET['base'], DET['rise'], DET['tol'])
        # Score every triple, including the rejected ones. detect_frame throws
        # this away; it is precisely the near-miss information we came for.
        err = float(np.abs(np.sort(d) - exp).sum())
        best_err = min(best_err, err)
        if not ok:
            continue
        passed_sides = True

        # Canonical order [bottom_a, bottom_b, apex]. Side lengths alone let a
        # decoy through - three balls 1.10/1.23/1.23 apart lying flat on the
        # floor pass - so the frame's actual vertical structure is checked too.
        apex = int(np.argmax(P[:, 2]))
        bottom = sorted([i for i in range(3) if i != apex], key=lambda i: P[i, 1])
        Po = P[[bottom[0], bottom[1], apex]]
        if abs(Po[0, 2] - Po[1, 2]) > DET['z_tol']:
            continue                                   # bottom pair not level
        if abs((Po[2, 2] - 0.5 * (Po[0, 2] + Po[1, 2])) - DET['rise']) > DET['z_tol']:
            continue                                   # apex at the wrong height
        if best_accepted is None or err < best_accepted:
            best_accepted = err

    if best_accepted is not None:
        return 'ACCEPTED', len(cands), best_err
    # Gate 4 before gate 3: reaching the shape checks at all is further through
    # the pipeline than failing the side lengths, so a frame that got there is
    # reported as the later, nearer-miss failure.
    if passed_sides:
        return 'SHAPE_GATE', len(cands), best_err
    return 'FEW_CANDIDATES', len(cands), best_err


def main(svo, want, csv_path, roi_x, seeds):
    set_roi_x(roi_x)
    init = sl.InitParameters()
    init.set_from_svo_file(svo)
    # NEURAL is what the live pipeline records with, and the whole point of
    # replaying an SVO is that the depth is reproduced, not re-decided.
    init.depth_mode = sl.DEPTH_MODE.NEURAL
    init.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
    init.coordinate_units = sl.UNIT.METER
    init.svo_real_time_mode = False
    init.depth_stabilization = 1
    cam = sl.Camera()
    if cam.open(init) != sl.ERROR_CODE.SUCCESS:
        print('OPEN FAILED: %s' % svo)
        return 1

    total = cam.get_svo_number_of_frames()
    step = max(1, total // (want + 1))
    cloud = sl.Mat()
    rt = sl.RuntimeParameters()
    rt.confidence_threshold = 50           # matches the live pipeline (verified 2026-09-05)
    rt.texture_confidence_threshold = 100

    f = open(csv_path, 'w')
    # The `seed` column is written even for a single-seed run (as seed 0) so one
    # reader handles both files and nobody has to guess whether a row is one of
    # several repeats of the same frame.
    f.write('frame,outcome,n_points_in_box,n_candidates,best_side_err,seed\n')

    counts = {b: 0 for b in BUCKETS}       # counted on seed 0 only - see below
    per_frame = []                         # [(frame_index, [outcome per seed])]

    # Jump straight to each sampled frame instead of grabbing sequentially: on a
    # 2000-frame file, sequential grabbing runs ~2000 depth inferences to use 40
    # of them. Safe here because depth_stabilization is 1 - there is no
    # meaningful temporal state that needs warming up.
    for k in range(want):
        if step > 3:
            cam.set_svo_position(min(k * step, total - 1))
        if cam.grab(rt) != sl.ERROR_CODE.SUCCESS:
            break
        cam.retrieve_measure(cloud, sl.MEASURE.XYZ)
        a = cloud.get_data()[:, :, :3].reshape(-1, 3)
        a = a[np.isfinite(a).all(axis=1)]
        m = ((a[:, 0] > ROI['xmin']) & (a[:, 0] < ROI['xmax']) &
             (a[:, 1] > ROI['ymin']) & (a[:, 1] < ROI['ymax']) &
             (a[:, 2] > ROI['zmin']) & (a[:, 2] < ROI['zmax']))
        pts = a[m].astype(np.float64)

        # All seeds run against the SAME cropped points. The depth decode is by
        # far the expensive part of a frame, and re-decoding per seed would also
        # be dishonest: it would let depth vary while we claim to be isolating
        # the effect of RANSAC's randomness.
        outcomes = []
        for s in range(seeds):
            bucket, n_cand, err = classify(pts, np.random.default_rng(s))
            outcomes.append(bucket)
            f.write('%d,%s,%d,%d,%s,%d\n'
                    % (k, bucket, len(pts), n_cand,
                       'nan' if err != err or err == float('inf') else '%.5f' % err, s))
            if s == 0:
                counts[bucket] += 1
        f.flush()
        per_frame.append((k, outcomes))
        print('  frame %3d  %6d pts  %s' % (k, len(pts), ' '.join(outcomes)))

    f.close()
    cam.close()

    n = len(per_frame)
    if n == 0:
        print('NO FRAMES GRABBED - is the SVO path right?')
        return 1

    # Seed 0 is the headline table because seed 0 is what every other script in
    # this folder uses (np.random.default_rng(0)), so these counts describe the
    # detector as it is actually run, not an average over configurations nobody
    # uses. The seed spread is reported separately below.
    print()
    print('WHY FRAMES FAIL   %s   roi_x %.2f m   n=%d frames   seed 0'
          % (os.path.basename(svo), roi_x, n))
    print('  bucket            count   share')
    for b in BUCKETS:
        print('  %-16s %5d  %5.1f%%' % (b, counts[b], 100.0 * counts[b] / n))
    print('  %-16s %5d  %5.1f%%' % ('(total)', n, 100.0))

    if seeds > 1:
        # Stability: how often does re-rolling RANSAC's dice change the answer?
        # A frame is "stable" if all N seeds agreed. Unstable frames are the
        # detector's randomness; stable failures are the data's own limits, and
        # only the second kind is evidence about the camera.
        stable = sum(1 for _, o in per_frame if len(set(o)) == 1)
        stable_acc = sum(1 for _, o in per_frame if set(o) == {'ACCEPTED'})
        ever_acc = sum(1 for _, o in per_frame if 'ACCEPTED' in o)
        print()
        print('SEED STABILITY   %d seeds x %d frames' % (seeds, n))
        print('  same bucket on every seed        %d of %d  (%.1f%%)'
              % (stable, n, 100.0 * stable / n))
        print('  accepted on EVERY seed           %d' % stable_acc)
        print('  accepted on at least one seed    %d' % ever_acc)
        print('  flipped between accept and fail  %d' % (ever_acc - stable_acc))
        if ever_acc - stable_acc:
            print()
            print('  frames whose outcome moved:')
            for k, o in per_frame:
                if len(set(o)) > 1:
                    print('    frame %3d  %s' % (k, ' '.join(o)))
        print()
        print('  Read it this way: frames that flip are limited by how long')
        print('  RANSAC searches, and more iterations would recover them.')
        print('  Frames that fail on every seed are limited by the points')
        print('  themselves, and no detector setting will bring them back.')

    print()
    print('  wrote %s' % csv_path)
    print('  EMPTY_BOX / NO_SPHERE dominant  -> the depth is the limit.')
    print('  FEW_CANDIDATES / SHAPE_GATE     -> the detector is the limit.')
    return 0


if __name__ == '__main__':
    # Hand-parsed rather than argparse to match the other scripts in this
    # folder, which are all positional. --seeds is pulled out from wherever it
    # appears so the positional arguments keep their places either way.
    argv = sys.argv[1:]
    n_seeds = 1
    if '--seeds' in argv:
        i = argv.index('--seeds')
        n_seeds = int(argv[i + 1])
        del argv[i:i + 2]
    if len(argv) < 3:
        print(__doc__)
        sys.exit(2)
    sys.exit(main(argv[0],
                  int(argv[1]),
                  argv[2],
                  float(argv[3]) if len(argv) > 3 else 4.0,
                  max(1, n_seeds)))
