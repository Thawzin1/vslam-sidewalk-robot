#!/usr/bin/env python3
"""live_armB_exposure.py - the exposure experiment, straight through the SDK.

WHY THE WRAPPER IS BYPASSED

    The ZED X is a GMSL camera: its exposure is set as EXPOSURE_TIME in
    microseconds. The ROS wrapper only implements the legacy percent-scale
    EXPOSURE control (grep: zero EXPOSURE_TIME references), which this camera
    silently ignores - verified live 2026-09-04: exposure 35 in the config
    produced a pixel-identical image to auto. So this script opens the camera
    natively, controls exposure properly, and runs the detection itself.

WHAT IT DOES

    1. Sweep: steps EXPOSURE_TIME down from auto, measuring image brightness
       and near-saturation, and prints the table - the glare study.
    2. Capture: at the chosen exposure, grabs frames, crops the search box,
       runs Nicolas's detector (imported, byte-identical), and writes a CSV in
       the series format. Confidence 50 to match the Arm A wrapper pipeline -
       EXPOSURE is the only changed variable.

  usage (wrapper must be STOPPED first - the camera is single-owner):
    python3 live_armB_exposure.py <exposure_us> <n_samples> <csv_path> [roi_x=5.0]
    exposure_us = 0 means sweep-only: print the table and exit.
"""
import sys
import time

import numpy as np
import pyzed.sl as sl

sys.path.insert(0, '~/catkin_ws/src/sidewalk_evaluation/scripts')
from sphere_centroid import detect_frame, centroid_of  # noqa: E402

DET = dict(radius=0.20, radius_tol=0.07, thresh=0.03, iters=1500, min_inliers=10,
           base=1.10, rise=1.10, tol=0.06, max_candidates=7, z_tol=0.08)


def open_cam():
    init = sl.InitParameters()
    init.camera_resolution = sl.RESOLUTION.HD1200
    init.camera_fps = 15
    init.depth_mode = sl.DEPTH_MODE.NEURAL
    init.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
    init.coordinate_units = sl.UNIT.METER
    init.depth_stabilization = 1
    cam = sl.Camera()
    st = cam.open(init)
    if st != sl.ERROR_CODE.SUCCESS:
        print('OPEN FAILED:', st)
        sys.exit(1)
    return cam


def img_stats(cam, img, rt, n=3):
    ms, br = [], []
    for _ in range(n):
        if cam.grab(rt) != sl.ERROR_CODE.SUCCESS:
            continue
        cam.retrieve_image(img, sl.VIEW.LEFT)
        g = img.get_data()[:, :, :3].mean(axis=2)
        ms.append(g.mean())
        br.append((g > 200).mean() * 100)
    return (float(np.mean(ms)), float(np.mean(br))) if ms else (0.0, 0.0)


def main():
    exp_us = int(sys.argv[1])
    want = int(sys.argv[2]) if len(sys.argv) > 2 else 50
    csv_path = sys.argv[3] if len(sys.argv) > 3 else '/tmp/armB.csv'
    roi_x = float(sys.argv[4]) if len(sys.argv) > 4 else 5.0
    roi = dict(xmin=roi_x - 0.6, xmax=roi_x + 0.6, ymin=-0.95, ymax=0.95,
               zmin=-0.1294, zmax=1.6706)

    cam = open_cam()
    img = sl.Mat()
    cloud = sl.Mat()
    rt = sl.RuntimeParameters()
    rt.confidence_threshold = 50          # match the Arm A wrapper pipeline
    rt.texture_confidence_threshold = 100
    time.sleep(1)

    m, b = img_stats(cam, img, rt)
    print('auto exposure  : mean %5.0f  bright>200 %5.2f%%' % (m, b))
    if exp_us == 0:
        for e in (16000, 10000, 6000, 3500, 2000, 1200):
            cam.set_camera_settings(sl.VIDEO_SETTINGS.AEC_AGC, 0)
            cam.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE_TIME, e)
            time.sleep(0.8)
            m, b = img_stats(cam, img, rt)
            print('exposure %5d us: mean %5.0f  bright>200 %5.2f%%' % (e, m, b))
        cam.close()
        return

    cam.set_camera_settings(sl.VIDEO_SETTINGS.AEC_AGC, 0)
    cam.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE_TIME, exp_us)
    time.sleep(1)
    m, b = img_stats(cam, img, rt)
    print('capture at %d us: mean %.0f  bright>200 %.2f%%' % (exp_us, m, b))

    f = open(csv_path, 'w')
    f.write('n,x,y,z,base,leg1,leg2,n_bot1,n_bot2,n_apex,tx,ty,tz,stamp\n')
    rng = np.random.default_rng(0)
    kept = tried = 0
    while kept < want:
        if cam.grab(rt) != sl.ERROR_CODE.SUCCESS:
            continue
        tried += 1
        cam.retrieve_measure(cloud, sl.MEASURE.XYZ)
        a = cloud.get_data()[:, :, :3].reshape(-1, 3)
        a = a[np.isfinite(a).all(axis=1)]
        msk = ((a[:, 0] > roi['xmin']) & (a[:, 0] < roi['xmax']) &
               (a[:, 1] > roi['ymin']) & (a[:, 1] < roi['ymax']) &
               (a[:, 2] > roi['zmin']) & (a[:, 2] < roi['zmax']))
        pts = a[msk].astype(np.float64)
        res = detect_frame(pts, rng=rng, **DET)
        if res is None:
            continue
        centers, sides = res
        c = centroid_of(centers)
        npts = [int((np.linalg.norm(pts - cc, axis=1) < 0.24).sum()) for cc in centers]
        kept += 1
        f.write('%d,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f,%d,%d,%d,nan,nan,nan,%.3f\n'
                % (kept, c[0], c[1], c[2], sides[0], sides[1], sides[2],
                   npts[0], npts[1], npts[2], time.time()))
        f.flush()
        if kept % 10 == 0:
            print('  %d/%d kept (acceptance %.0f%%)' % (kept, want, 100.0 * kept / tried))
    f.close()
    cam.close()
    C = np.genfromtxt(csv_path, delimiter=',', names=True)
    s = [float(C[c].std(ddof=1)) * 1000 for c in ('x', 'y', 'z')]
    print('ARM B RESULT: n=%d acceptance %.0f%%  sigma %.2f/%.2f/%.2f  RMS3D %.2f mm'
          % (kept, 100.0 * kept / tried, s[0], s[1], s[2], sum(v * v for v in s) ** .5))
    print('mean (%.4f, %.4f, %.4f)  pts/sphere %.0f'
          % (C['x'].mean(), C['y'].mean(), C['z'].mean(),
             np.mean([C['n_bot1'].mean(), C['n_bot2'].mean(), C['n_apex'].mean()])))


if __name__ == '__main__':
    main()
