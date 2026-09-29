#!/usr/bin/env python3
"""svo_mode_compare.py - NEURAL vs NEURAL_PLUS vs ULTRA on identical recorded frames.

    Replays the same SVO once per depth mode and scores each mode on the sphere
    benchmark's own terms. Because every mode sees the same recorded photons,
    the mode is the ONLY variable - a comparison the live camera can never give.

    Scored per mode:
      - points per sphere            (coverage)
      - points on the frame's tube   (thin-structure visibility - the user's
                                      "the lines don't appear" question)
      - sphere surface residual      (how tightly points hug a 0.20 m sphere)
      - centroid mean + scatter      (the benchmark metric, small-n)

    Settings: depth_stabilization=1 and SDK-default confidence for ALL modes -
    uniform across modes, so mode differences are real even though the
    replay-vs-live equivalence question (see svo_replay_check.py) is still open.

  usage, on the JETSON:
    python3 svo_mode_compare.py /media/sidewalk/USB/centroid/ThawZin_ZEDX/svo/4m.svo2 12
"""
import sys

import numpy as np
import pyzed.sl as sl

sys.path.insert(0, '~/catkin_ws/src/sidewalk_evaluation/scripts')
from sphere_centroid import detect_frame, centroid_of  # noqa: E402

# Box centre distance comes from argv (was hardcoded 4 m; the series has many
# stations). Width/height mirror the launch: sx 1.2, and the z bounds are
# COMPUTED FROM THE SENSOR HEIGHT rather than baked in. The old constants
# (zmin -0.1294, zmax 1.6706) silently encoded sensor_h 0.6794 - walkway-1's
# camera height - and running this at walkway-3, where the camera sits at
# 0.6972 m, cropped a box 17.8 mm too high. The detector still found the frame,
# so nothing errored; the box was simply not the box the live run used.
SENSOR_H = 0.6972          # walkway-3; argv[4] overrides (0.6794 = walkway-1)
ROI = dict(xmin=3.4, xmax=4.6, ymin=-0.95, ymax=0.95,
           zmin=-SENSOR_H + 0.55, zmax=-SENSOR_H + 2.35)
DET = dict(radius=0.20, radius_tol=0.07, thresh=0.03, iters=1500, min_inliers=10,
           base=1.10, rise=1.10, tol=0.06, max_candidates=7, z_tol=0.08)
MODES = ('NEURAL', 'NEURAL_PLUS', 'ULTRA')


def set_roi_x(roi_x):
    ROI['xmin'] = roi_x - 0.6
    ROI['xmax'] = roi_x + 0.6


def run_mode(svo, mode_name, want):
    init = sl.InitParameters()
    init.set_from_svo_file(svo)
    init.depth_mode = getattr(sl.DEPTH_MODE, mode_name)
    init.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
    init.coordinate_units = sl.UNIT.METER
    init.svo_real_time_mode = False
    init.depth_stabilization = 1
    cam = sl.Camera()
    if cam.open(init) != sl.ERROR_CODE.SUCCESS:
        return None
    total = cam.get_svo_number_of_frames()
    step = max(1, total // (want + 1))
    cloud = sl.Mat()
    rt = sl.RuntimeParameters()
    cents, spheres_pts, tube_pts, resid = [], [], [], []
    # Jump straight to each sampled frame rather than grabbing sequentially -
    # on a 2000-frame file, sequential grabbing computes ~2000 depth inferences
    # to use 40 of them. Jumping is safe here because depth_stabilization is 1
    # (no meaningful temporal state to warm up).
    i = 0
    for k in range(want):
        if step > 3:
            cam.set_svo_position(min(k * step, total - 1))
        if cam.grab(rt) != sl.ERROR_CODE.SUCCESS:
            break
        i = k + 1
        cam.retrieve_measure(cloud, sl.MEASURE.XYZ)
        a = cloud.get_data()[:, :, :3].reshape(-1, 3)
        a = a[np.isfinite(a).all(axis=1)]
        m = ((a[:, 0] > ROI['xmin']) & (a[:, 0] < ROI['xmax']) &
             (a[:, 1] > ROI['ymin']) & (a[:, 1] < ROI['ymax']) &
             (a[:, 2] > ROI['zmin']) & (a[:, 2] < ROI['zmax']))
    # The tube slab from the live probe: vertical post between apex and base
        pts = a[m].astype(np.float64)
        res = detect_frame(pts, rng=np.random.default_rng(0), **DET)
        if res is None:
            continue
        centers, _ = res
        cents.append(centroid_of(centers))
        n_on, rr = 0, []
        for c in centers:
            d = np.linalg.norm(pts - c, axis=1)
            sel = d < 0.24
            n_on += int(sel.sum())
            rr.append(np.sqrt(np.mean((d[sel & (d > 0.14)] - 0.20) ** 2)))
        spheres_pts.append(n_on / 3.0)
        resid.append(float(np.mean(rr)) * 1000)
        cz = centers[:, 2]
        top, low = cz.max(), cz.min()
        tube = pts[(np.abs(pts[:, 1] - centers[2][1]) < 0.06) &
                   (pts[:, 2] > low + 0.28) & (pts[:, 2] < top - 0.28) &
                   (np.abs(pts[:, 0] - centers[2][0]) < 0.15)]
        tube_pts.append(len(tube))
    cam.close()
    if len(cents) < 5:
        return dict(mode=mode_name, n=len(cents), fail=True)
    C = np.array(cents)
    sd = C.std(axis=0, ddof=1) * 1000
    return dict(mode=mode_name, n=len(C), mean=C.mean(axis=0), sd=sd,
                rms3d=float(np.sqrt((sd ** 2).sum())),
                pts=float(np.mean(spheres_pts)), tube=float(np.mean(tube_pts)),
                resid=float(np.mean(resid)))


def main(svo, want):
    print('mode         n  pts/sphere  tube_pts  resid_mm     mean x/y/z              sigma x/y/z [mm]   RMS3D')
    for m in MODES:
        r = run_mode(svo, m, want)
        if r is None or r.get('fail'):
            print('%-11s FAILED (open error or <5 detections)' % m)
            continue
        print('%-11s %2d  %9.0f  %8.0f  %7.2f   (%.4f, %.4f, %.4f)   %.2f / %.2f / %.2f   %.2f'
              % (r['mode'], r['n'], r['pts'], r['tube'], r['resid'],
                 r['mean'][0], r['mean'][1], r['mean'][2],
                 r['sd'][0], r['sd'][1], r['sd'][2], r['rms3d']))


if __name__ == '__main__':
    # usage: svo_mode_compare.py <svo> [frames=12] [roi_x=4.0]
    if len(sys.argv) > 3:
        set_roi_x(float(sys.argv[3]))
    if len(sys.argv) > 4:
        # a different station means a different camera height, and the z crop
        # follows it: usage  svo_mode_compare.py <svo> [n] [roi_x] [sensor_h]
        SENSOR_H = float(sys.argv[4])
        ROI['zmin'] = -SENSOR_H + 0.55
        ROI['zmax'] = -SENSOR_H + 2.35
    main(sys.argv[1], int(sys.argv[2]) if len(sys.argv) > 2 else 12)
