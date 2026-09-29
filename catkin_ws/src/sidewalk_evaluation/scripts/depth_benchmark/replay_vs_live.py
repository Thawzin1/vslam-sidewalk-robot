#!/usr/bin/env python3
"""replay_vs_live.py - does replaying a recording give the LIVE RUN'S ANSWER?

WHY THIS IS NOT svo_live_equivalence.py

    That script compares DEPTH at a few pixels, which is the cleanest possible
    test of the replay path because no fitting is involved. This one compares
    the thing the depth study actually reports: the frame's CENTROID, through the
    frozen detector, so the comparison is against a live run's published
    numbers rather than an intermediate quantity.

    It matters that both exist. If the depth agrees and the centroid does not,
    the difference is in the detector's response to small depth changes, not in
    the replay.

WHY THIS VERSION AND NOT svo_replay_check.py

    That one has the walkway-1 4 m search box and its camera height baked in,
    along with a hard-coded live baseline in a print statement. Run at another
    station it silently crops the wrong volume. Everything here is a parameter.

WHAT IT COMPARES, AND AGAINST WHAT

    Replay -> frozen detector -> centroid per frame -> mean and RMS3D.
    Against the live CSV's own mean and RMS3D, computed here from the file
    rather than typed in.

    The honest reading of the result depends on what it is compared to. A
    difference is only meaningful against the LIVE RUN'S OWN frame-to-frame
    spread, and against how much the live camera disagrees with itself between
    sessions - measured at 7-17 mm at 3 m. Both are printed.

  usage, on the JETSON:
    python3 replay_vs_live.py --svo <file> --live <live_centroid.csv> \\
        --roi-x 4.0 --sensor-h 0.6972 --mode NEURAL --frames 25
"""
from __future__ import print_function

import argparse
import csv
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Nicolas's frozen values. These MUST match the launch file; changing one here
# to make a comparison come out better would invalidate the comparison itself.
DET = dict(radius=0.20, radius_tol=0.07, thresh=0.03, iters=1500, min_inliers=10,
           base=1.10, rise=1.10, tol=0.06, max_candidates=7, z_tol=0.08)


def live_stats(path):
    rows = list(csv.DictReader(open(path)))
    a = np.array([[float(v['x']), float(v['y']), float(v['z'])] for v in rows])
    s = a.std(axis=0, ddof=1)
    return dict(n=len(a), mean=a.mean(axis=0), sd=s,
                rms=float(np.sqrt((s ** 2).sum())))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--svo', required=True)
    ap.add_argument('--live', required=True, help='the live run centroid CSV')
    ap.add_argument('--roi-x', type=float, required=True)
    ap.add_argument('--roi-sx', type=float, default=1.2)
    ap.add_argument('--sensor-h', type=float, default=0.6972)
    ap.add_argument('--roi-z', type=float, default=1.45)
    ap.add_argument('--mode', default='NEURAL')
    ap.add_argument('--frames', type=int, default=25)
    ap.add_argument('--conf', type=int, default=50)
    ap.add_argument('--texture-conf', type=int, default=100)
    # SELF-CALIBRATION AND STABILIZATION ARE SEPARATE FLAGS, because they are
    # separate camera settings. The old single --pinned bundled them (off + 0),
    # and the default bundled the opposite (on + 1) - and the 4 m station's
    # live configuration is off + 1, which NEITHER could express. A proof test
    # that cannot reproduce the run it is proving against tests nothing.
    ap.add_argument('--self-calib', choices=('on', 'off'), default='off',
                    help="off matches every pinned station from 2026-09-09 on")
    ap.add_argument('--stabilization', type=int, choices=(0, 1), default=1,
                    help="1 is the SDK default and what the 4 m station ran")
    ap.add_argument('--pinned', action='store_true',
                    help="DEPRECATED shorthand for --self-calib off "
                         "--stabilization 0; kept so old notes still run")
    a = ap.parse_args()

    import pyzed.sl as sl
    from sphere_centroid import detect_frame, centroid_of

    L = live_stats(a.live)

    init = sl.InitParameters()
    init.set_from_svo_file(a.svo)
    init.depth_mode = getattr(sl.DEPTH_MODE, a.mode)
    init.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
    init.coordinate_units = sl.UNIT.METER
    init.svo_real_time_mode = False
    if a.pinned:
        a.self_calib, a.stabilization = 'off', 0
    init.camera_disable_self_calib = (a.self_calib == 'off')
    init.depth_stabilization = a.stabilization
    cam = sl.Camera()
    if cam.open(init) != sl.ERROR_CODE.SUCCESS:
        print('could not open %s' % a.svo)
        return 1

    half = a.roi_sx / 2.0
    ROI = dict(xmin=a.roi_x - half, xmax=a.roi_x + half, ymin=-0.95, ymax=0.95,
               zmin=-a.sensor_h + 0.55, zmax=-a.sensor_h + 0.55 + 1.80)
    total = cam.get_svo_number_of_frames()
    step = max(1, total // (a.frames + 1))
    rt = sl.RuntimeParameters()
    rt.confidence_threshold = a.conf
    rt.texture_confidence_threshold = a.texture_conf
    cloud = sl.Mat()
    rng = np.random.default_rng(0)

    cents, tried = [], 0
    for k in range(a.frames):
        cam.set_svo_position(min(k * step, total - 1))
        if cam.grab(rt) != sl.ERROR_CODE.SUCCESS:
            break
        cam.retrieve_measure(cloud, sl.MEASURE.XYZ)
        p = cloud.get_data()[:, :, :3].reshape(-1, 3)
        p = p[np.isfinite(p).all(axis=1)]
        m = ((p[:, 0] > ROI['xmin']) & (p[:, 0] < ROI['xmax']) &
             (p[:, 1] > ROI['ymin']) & (p[:, 1] < ROI['ymax']) &
             (p[:, 2] > ROI['zmin']) & (p[:, 2] < ROI['zmax']))
        box = p[m]
        tried += 1
        res = detect_frame(box, DET['radius'], DET['radius_tol'], DET['thresh'],
                           DET['iters'], DET['min_inliers'], DET['base'],
                           DET['rise'], DET['tol'], DET['max_candidates'], rng,
                           DET['z_tol'])
        if res is not None:
            cents.append(centroid_of(res[0]))
        print('  frame %d/%d  %s' % (k + 1, a.frames,
                                     'accepted' if res is not None else 'rejected'))
        sys.stdout.flush()
    cam.close()

    print('\n' + '=' * 74)
    print('  REPLAY versus LIVE   %s   mode %s%s'
          % (os.path.basename(a.svo), a.mode,
             '  (self-calib %s, stabilization %d)' % (a.self_calib, a.stabilization)))
    print('=' * 74)
    if len(cents) < 3:
        print('  only %d of %d replayed frames were accepted - too few to compare.'
              % (len(cents), tried))
        print('  That is itself a result: the recording does not reproduce the run.')
        return 1
    R = np.array(cents)
    rs = R.std(axis=0, ddof=1)
    rrms = float(np.sqrt((rs ** 2).sum()))
    d = R.mean(axis=0) - L['mean']

    print('  %-14s %10s %10s %10s %10s %8s' % ('', 'x', 'y', 'z', 'RMS3D', 'n'))
    print('  ' + '-' * 66)
    print('  %-14s %10.4f %10.4f %10.4f %8.2f mm %8d'
          % ('live', L['mean'][0], L['mean'][1], L['mean'][2], L['rms'], L['n']))
    print('  %-14s %10.4f %10.4f %10.4f %8.2f mm %8d'
          % ('replay', R.mean(axis=0)[0], R.mean(axis=0)[1], R.mean(axis=0)[2],
             rrms, len(R)))
    print('  %-14s %+9.1f %+10.1f %+10.1f    (mm)' % ('difference', d[0] * 1000,
                                                      d[1] * 1000, d[2] * 1000))
    print('\n  distance between the two means: %.1f mm' % (np.linalg.norm(d) * 1000))
    print('  replay acceptance: %d of %d frames' % (len(R), tried))
    print('\n  HOW TO READ THAT NUMBER:')
    print('    the live run\'s own frame-to-frame spread is %.1f mm (RMS3D)' % L['rms'])
    print('    the live camera disagreed with ITSELF by 7-17 mm between sessions')
    print('    at the 3 m station, which is the drift this is competing with.')
    print('    A difference inside those is agreement; well outside is not.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
