#!/usr/bin/env python3
"""svo_sphere_dump.py - write the THREE sphere centres, not just their centroid.

WHY

    After correcting the camera's measured downward tilt, the 5 m station still
    reads the target 33-162 mm too near. That is not a depth scale error: a mode
    that scaled depth down would read the target NARROWER as well as NEARER, and
    NEURAL reads it 55 mm too WIDE while placing it 162 mm too NEAR. A single
    scale cannot do both.

    So the error must be in where the individual sphere centres land, and the
    replay throws those away - it keeps only their centroid. This dumps them.

    The detector is imported and called UNCHANGED. Nothing here modifies the
    frozen code; it only keeps a result that svo_replay_n.py discards.

  usage, on the JETSON:
    python3 svo_sphere_dump.py <svo> --mode NEURAL --frames 120 --out /tmp/d.csv
"""
from __future__ import print_function

import argparse
import csv
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sphere_centroid import detect_frame, centroid_of      # frozen, unmodified

try:
    import pyzed.sl as sl
except ImportError:
    sys.exit('pyzed not available - run this on the Jetson')

DET = dict(radius=0.20, radius_tol=0.07, thresh=0.03, iters=1500, min_inliers=10,
           base=1.10, rise=1.10, tol=0.06, max_candidates=7, z_tol=0.08)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('svo')
    ap.add_argument('--mode', default='NEURAL')
    ap.add_argument('--frames', type=int, default=120)
    ap.add_argument('--roi-x', type=float, default=5.0)
    ap.add_argument('--sensor-h', type=float, default=0.6972)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()

    init = sl.InitParameters()
    init.set_from_svo_file(a.svo)
    init.depth_mode = getattr(sl.DEPTH_MODE, a.mode)
    init.coordinate_units = sl.UNIT.METER
    init.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
    init.svo_real_time_mode = False
    init.depth_stabilization = 1
    init.camera_disable_self_calib = True
    # Set because svo_replay_n.py sets them, so this tool measures the same
    # instrument that produced the numbers being explained.
    #
    # frames both ways: 7 accepted either way, and the identical frames
    # (.......AA.AA....A...A...A.). An earlier comment here claimed they were
    # the cause of a zero-acceptance run. They were not. The real cause was that
    # THE FIRST ~7 FRAMES OF A RECORDING NEVER ACCEPT - depth stabilization
    # starts with no history - and three diagnostics in a row were cut off
    # inside that dead zone by a timeout, then explained away with a mechanism
    # instead of being run longer.
    init.depth_minimum_distance = 1.0
    init.depth_maximum_distance = 35.0
    cam = sl.Camera()
    if cam.open(init) != sl.ERROR_CODE.SUCCESS:
        sys.exit('could not open %s' % a.svo)
    rt = sl.RuntimeParameters()
    rt.confidence_threshold = 50
    rt.texture_confidence_threshold = 100

    lo = np.array([a.roi_x - 0.6, -0.95, -a.sensor_h + 0.55])
    hi = np.array([a.roi_x + 0.6, 0.95, -a.sensor_h + 2.35])

    total = cam.get_svo_number_of_frames()
    step = max(1, total // a.frames)
    cloud = sl.Mat()
    rows = []

    # WRITE AS WE GO, NOT AT THE END.
    #
    # The first run of this tool wrote its CSV only after the last frame, and a
    # 250 s timeout killed it at ~2 s per frame with nothing at all on disk -
    # four minutes of Jetson work thrown away twice over. The campaign already
    # learned this: a long job that keeps its result in memory has no partial
    # result, and no progress anyone can watch. Flush every row.
    hdr = (['frame'] + ['%s_%s' % (s, c) for s in ('b1', 'b2', 'ap') for c in 'xyz']
           + ['cx', 'cy', 'cz', 'base', 'leg1', 'leg2',
              'n_b1', 'n_b2', 'n_ap', 'r_b1', 'r_b2', 'r_ap', 'n_box'])
    fh = open(a.out, 'w')
    wr = csv.writer(fh)
    wr.writerow(hdr)
    fh.flush()
    prog = a.out + '.progress'
    planned = len(range(0, total, step))
    seen = 0

    # GRAB SEQUENTIALLY. Do NOT seek to each frame.
    #
    # depth_stabilization=1 fuses depth across recent frames, so a seek throws
    # that history away. Seeking before every single grab makes EVERY frame a
    # cold start - the densest possible version of the slice-boundary effect the
    # campaign measured - and the first version of this script did exactly that
    # and accepted 0 of its first 8 frames where the campaign accepts about a
    # third. The campaign seeks once and then grabs in order; match it, or these
    # sphere positions describe a different instrument from the one that
    # produced the numbers being explained.
    cam.set_svo_position(0)
    for idx in range(total):
        if cam.grab(rt) != sl.ERROR_CODE.SUCCESS:
            break
        if idx % step:
            continue
        cam.retrieve_measure(cloud, sl.MEASURE.XYZ)
        arr = cloud.get_data()[:, :, :3].reshape(-1, 3)
        arr = arr[np.isfinite(arr).all(axis=1)]
        pts = arr[np.all((arr >= lo) & (arr <= hi), axis=1)].astype(np.float64)
        if len(pts) < 3 * DET['min_inliers']:
            continue
        # seeded from the absolute frame index, exactly as the campaign does
        seen += 1
        tmp = prog + '.tmp'
        with open(tmp, 'w') as pf:
            pf.write('%d %d %d\n' % (seen, planned, len(rows)))
        os.replace(tmp, prog)      # atomic: the reader never sees half a line
        res = detect_frame(pts, rng=np.random.default_rng(idx), **DET)
        if res is None:
            continue
        centers, d = res      # canonical order: [bottom_a, bottom_b, apex]
        cen = centroid_of(centers)
        n = [int((np.abs(np.linalg.norm(pts - c, axis=1) - DET['radius'])
                  < DET['thresh']).sum()) for c in centers]
        r = [float(np.linalg.norm(c)) for c in centers]
        row = ([idx] + [round(float(v), 6) for c in centers for v in c]
               + [round(float(v), 6) for v in cen]
               + [round(v, 6) for v in d] + n + [round(v, 6) for v in r]
               + [len(pts)])
        rows.append(row)
        wr.writerow(row)
        fh.flush()
    cam.close()
    fh.close()
    print('%s  mode=%s  %d frames sampled, %d accepted -> %s'
          % (os.path.basename(a.svo), a.mode, len(range(0, total, step)), len(rows), a.out))
    if rows:
        A = np.array(rows, dtype=float)
        col = {h: i for i, h in enumerate(hdr)}
        for s, nm in (('b1', 'lower ball A'), ('b2', 'lower ball B'), ('ap', 'apex')):
            print('  %-13s x %.4f  y %+.4f  z %+.4f   range %.4f   pts %5.0f'
                  % (nm, A[:, col[s + '_x']].mean(), A[:, col[s + '_y']].mean(),
                     A[:, col[s + '_z']].mean(), A[:, col['r_' + s]].mean(),
                     A[:, col['n_' + s]].mean()))
    return 0


if __name__ == '__main__':
    sys.exit(main())
