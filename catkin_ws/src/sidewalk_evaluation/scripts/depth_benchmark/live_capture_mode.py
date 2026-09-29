#!/usr/bin/env python3
"""live_capture_mode.py - record a station at a CHOSEN depth mode, natively.

WHY THIS EXISTS

    The walkway-3 series records every distance twice, once per depth mode, so
    the two are compared as measurements rather than as a replay. The normal
    route is the ROS driver's depth_mode argument. Its own configuration file
    lists NEURAL_PLUS as a legal value, but the string-to-enum conversion is not
    in the nodelet source, so whether the driver actually accepts it has to be
    tested rather than assumed - and if it refuses, the whole plan stalls.

    This is the fallback: open the camera directly, set the depth mode by enum
    (which cannot silently fall back to something else), and run the frozen
    detector on the result. Same detector values, same crop box, same confidence
    settings as the live ROS pipeline, so a run from here is comparable with a
    run from there. The ONLY difference is which process owns the camera.

    It is also the honest way to record NEURAL_PLUS even if the driver does work,
    because here the mode in force is a value we pass, not a string we hope was
    understood. The mode is echoed back from the SDK at startup and written into
    the CSV's companion line, so the file says what produced it.

  usage, on the JETSON, with the ROS camera STOPPED (single owner):
    python3 live_capture_mode.py <MODE> <n_samples> <csv_path> <roi_x> [sensor_h]

    MODE is NEURAL, NEURAL_PLUS, ULTRA, QUALITY or PERFORMANCE.
    sensor_h defaults to walkway-3's measured 0.6972 m.
"""
from __future__ import print_function

import os
import sys
import time

import numpy as np
import pyzed.sl as sl

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sphere_centroid import detect_frame, centroid_of  # noqa: E402  frozen, unmodified

# Nicolas's frozen values. Identical to sphere_centroid_zedx.launch - if these
# ever diverge the two recording routes stop being comparable, which is the
# whole point of this script.
DET = dict(radius=0.20, radius_tol=0.07, thresh=0.03, iters=1500, min_inliers=10,
           base=1.10, rise=1.10, tol=0.06, max_candidates=7, z_tol=0.08)


def main():
    mode_name = sys.argv[1].upper()
    want = int(sys.argv[2])
    csv_path = sys.argv[3]
    roi_x = float(sys.argv[4])
    sensor_h = float(sys.argv[5]) if len(sys.argv) > 5 else 0.6972

    if not hasattr(sl.DEPTH_MODE, mode_name):
        print('NO SUCH DEPTH MODE: %s' % mode_name)
        print('this build offers: %s' % ', '.join(m.name for m in sl.DEPTH_MODE))
        return 1

    # ROI: centre plus size, vertical centre measured ABOVE THE FLOOR so it is
    # sensor-independent, converted here using the measured camera height -
    # exactly as the launch file does it.
    roi = dict(xmin=roi_x - 0.6, xmax=roi_x + 0.6, ymin=-0.95, ymax=0.95,
               zmin=1.45 - 0.9 - sensor_h, zmax=1.45 + 0.9 - sensor_h)

    init = sl.InitParameters()
    init.camera_resolution = sl.RESOLUTION.HD1200
    init.camera_fps = 15
    init.depth_mode = getattr(sl.DEPTH_MODE, mode_name)
    init.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
    init.coordinate_units = sl.UNIT.METER
    init.depth_stabilization = 1
    cam = sl.Camera()
    if cam.open(init) != sl.ERROR_CODE.SUCCESS:
        print('OPEN FAILED')
        return 1

    # Read the mode back from the SDK rather than trusting what we asked for.
    got = cam.get_init_parameters().depth_mode
    print('requested %s, camera reports %s' % (mode_name, got))
    if got != getattr(sl.DEPTH_MODE, mode_name):
        print('MODE MISMATCH - the camera did not honour the request. Stopping.')
        cam.close()
        return 1

    rt = sl.RuntimeParameters()
    rt.confidence_threshold = 50            # matches the live ROS pipeline
    rt.texture_confidence_threshold = 100
    cloud = sl.Mat()
    rng = np.random.default_rng(0)

    f = open(csv_path, 'w')
    f.write('n,x,y,z,base,leg1,leg2,n_bot1,n_bot2,n_apex,tx,ty,tz,stamp\n')
    with open(csv_path.replace('.csv', '_mode.txt'), 'w') as mf:
        mf.write('depth_mode=%s\nroi_x=%.2f\nsensor_h=%.4f\nconfidence=50\n'
                 'texture_confidence=100\nresolution=HD1200\nstabilization=1\n'
                 'detector=frozen (sphere_centroid_zedx.launch values)\n'
                 % (got, roi_x, sensor_h))

    kept = tried = 0
    t0 = time.time()
    while kept < want:
        if cam.grab(rt) != sl.ERROR_CODE.SUCCESS:
            continue
        tried += 1
        cam.retrieve_measure(cloud, sl.MEASURE.XYZ)
        a = cloud.get_data()[:, :, :3].reshape(-1, 3)
        a = a[np.isfinite(a).all(axis=1)]
        m = ((a[:, 0] > roi['xmin']) & (a[:, 0] < roi['xmax']) &
             (a[:, 1] > roi['ymin']) & (a[:, 1] < roi['ymax']) &
             (a[:, 2] > roi['zmin']) & (a[:, 2] < roi['zmax']))
        pts = a[m].astype(np.float64)
        if len(pts) < 3 * DET['min_inliers']:
            continue
        res = detect_frame(pts, rng=rng, **DET)
        if res is None:
            continue
        centers, sides = res
        c = centroid_of(centers)
        npts = [int((np.abs(np.linalg.norm(pts - cc, axis=1) - DET['radius'])
                     < DET['thresh']).sum()) for cc in centers]
        kept += 1
        f.write('%d,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f,%d,%d,%d,nan,nan,nan,%.6f\n'
                % (kept, c[0], c[1], c[2], sides[0], sides[1], sides[2],
                   npts[0], npts[1], npts[2], time.time()))
        f.flush()
        if kept % 25 == 0:
            el = (time.time() - t0) / 60.0
            print('  [N=%d/%d] %.0f%% accepted, %.1f min elapsed, %.1f min to go'
                  % (kept, tried, 100.0 * kept / tried, el, el * (want - kept) / kept))
    f.close()
    cam.close()

    a = np.genfromtxt(csv_path, delimiter=',', names=True)
    s = [float(a[c].std(ddof=1)) * 1000 for c in ('x', 'y', 'z')]
    print()
    print('%s  n=%d  acceptance %.1f%%' % (got, kept, 100.0 * kept / tried))
    print('  sigma %.2f/%.2f/%.2f mm   RMS3D %.2f mm'
          % (s[0], s[1], s[2], sum(v * v for v in s) ** 0.5))
    print('  mean (%.4f, %.4f, %.4f)  sides %.3f/%.3f/%.3f  pts/sphere %.0f'
          % (a['x'].mean(), a['y'].mean(), a['z'].mean(), a['base'].mean(),
             a['leg1'].mean(), a['leg2'].mean(),
             np.mean([a['n_bot1'].mean(), a['n_bot2'].mean(), a['n_apex'].mean()])))
    return 0


if __name__ == '__main__':
    sys.exit(main())
