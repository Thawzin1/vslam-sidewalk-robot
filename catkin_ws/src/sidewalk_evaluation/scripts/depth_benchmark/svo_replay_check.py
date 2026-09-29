#!/usr/bin/env python3
"""svo_replay_check.py - does a recorded SVO reproduce the live run's answer?

WHY THIS EXISTS

    The whole camera-native research arm (Arm C) rests on one assumption: that
    re-running depth estimation on a recorded SVO gives the same measurement the
    live camera gave. The wrapper records SVO with H265 - a LOSSY video
    compression - so that assumption must be TESTED, not trusted, before seven
    more distances are captured in this format.

    Method: replay the 4 m SVO through the SAME depth mode the live run used
    (NEURAL), crop the SAME search box, run the SAME frozen detector maths
    (imported from sphere_centroid.py - Nicolas's functions, byte-identical),
    and compare centroid mean and scatter against the live run's official
    figures. Same scene, same settings, same maths: any disagreement beyond
    sampling noise is the compression (or the replay path) talking.

    PASS: replay mean within ~2 mm of live mean per axis (sampling limit at
    n=12: about 1-2 mm), and scatter the same order. FAIL: means several mm
    apart or scatter clearly inflated -> switch the wrapper to a lossless SVO
    mode before the next distance is recorded.

  usage, on the JETSON (heavy: ~35 s per frame, GPU for NEURAL):
    python3 svo_replay_check.py /media/sidewalk/USB/centroid/ThawZin_ZEDX/svo/4m.svo2 12
"""
import sys

import numpy as np
import pyzed.sl as sl

sys.path.insert(0, '~/catkin_ws/src/sidewalk_evaluation/scripts')
from sphere_centroid import detect_frame, centroid_of  # noqa: E402  Nicolas's maths, untouched

# The live run's search box, from sphere_centroid_zedx.launch with sensor_h 0.6794
ROI = dict(xmin=3.4, xmax=4.6, ymin=-0.95, ymax=0.95, zmin=-0.1294, zmax=1.6706)
# Nicolas's frozen detector values - MUST match the launch, never "improved" here
DET = dict(radius=0.20, radius_tol=0.07, thresh=0.03, iters=1500, min_inliers=10,
           base=1.10, rise=1.10, tol=0.06, max_candidates=7, z_tol=0.08)


def main(svo_path, want):
    init = sl.InitParameters()
    init.set_from_svo_file(svo_path)
    init.depth_mode = sl.DEPTH_MODE.NEURAL
    # ROS body convention (x forward, y left, z up) so the ROI numbers and the
    # live CSV numbers are directly comparable.
    init.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
    init.coordinate_units = sl.UNIT.METER
    init.svo_real_time_mode = False
    # MATCH THE LIVE WRAPPER (zedx_front.yaml + common.yaml), or the comparison
    # is confounded. First attempt forgot these and read 4.6 mm of "H265 error"
    # in x that was really the SDK's replay defaults: depth_stabilization
    # defaults to 30 (temporal smoothing the live run, at 1, never had) and
    # confidence to 95 (the live run filters hard at 50).
    init.depth_stabilization = 1
    cam = sl.Camera()
    st = cam.open(init)
    if st != sl.ERROR_CODE.SUCCESS:
        print('OPEN FAILED:', st)
        return 1
    total = cam.get_svo_number_of_frames()
    # Spread the sampled frames across the whole file rather than taking a
    # burst - temporal spread matches how the live CSV sampled the scene.
    step = max(1, total // want)
    cloud = sl.Mat()
    rt = sl.RuntimeParameters()
    rt.confidence_threshold = 50          # live wrapper: depth_confidence 50
    rt.texture_confidence_threshold = 100  # live wrapper: depth_texture_conf 100
    cents, rng = [], np.random.default_rng(0)
    i = 0
    while len(cents) < want:
        if cam.grab(rt) != sl.ERROR_CODE.SUCCESS:
            break
        i += 1
        if (i - 1) % step:
            continue
        cam.retrieve_measure(cloud, sl.MEASURE.XYZ)
        a = cloud.get_data()[:, :, :3].reshape(-1, 3)
        a = a[np.isfinite(a).all(axis=1)]
        m = ((a[:, 0] > ROI['xmin']) & (a[:, 0] < ROI['xmax']) &
             (a[:, 1] > ROI['ymin']) & (a[:, 1] < ROI['ymax']) &
             (a[:, 2] > ROI['zmin']) & (a[:, 2] < ROI['zmax']))
        pts = a[m].astype(np.float64)
        res = detect_frame(pts, rng=rng, **DET)
        if res is None:
            print('frame %d: no detection (%d pts in box)' % (i, len(pts)))
            continue
        centers, sides = res
        c = centroid_of(centers)
        cents.append(c)
        print('frame %d: centroid (%.4f, %.4f, %.4f)  sides %.3f/%.3f/%.3f'
              % (i, c[0], c[1], c[2], *sides))
    cam.close()
    if len(cents) < 5:
        print('FAIL: only %d detections - replay path is broken' % len(cents))
        return 1
    C = np.array(cents)
    mu = C.mean(axis=0)
    sd = C.std(axis=0, ddof=1)
    print()
    print('REPLAY  n=%d  mean (%.4f, %.4f, %.4f)  sigma %.2f/%.2f/%.2f mm'
          % (len(C), mu[0], mu[1], mu[2], *(sd * 1000)))
    print('LIVE    n=200 mean (3.8546, 0.0452, 0.7426)  sigma 3.04/6.15/3.86 mm')
    d = (mu - np.array([3.8546, 0.0452, 0.7426])) * 1000
    print('mean difference: %.1f / %.1f / %.1f mm  (sampling limit at this n: ~%.1f mm)'
          % (d[0], d[1], d[2], 6.15 / np.sqrt(len(C))))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1], int(sys.argv[2]) if len(sys.argv) > 2 else 12))
