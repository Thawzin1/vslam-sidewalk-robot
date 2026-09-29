#!/usr/bin/env python3
"""svo_camera_pose.py - measure the camera's height and tilt FROM A RECORDING.

WHY THIS EXISTS

    At the 5 m station the camera places the target 167-249 mm ABOVE where both
    lasers put it, in every depth mode. The lasers sit 180 mm apart in height and
    agree to 2.7 mm, so mount height cannot do it. The suspect is the camera's
    downward tilt never reaching the point cloud.

    Testing that needs the camera's pose AT THE MOMENT OF RECORDING, and it must
    be measured WITHOUT using the lasers. Fitting a rotation that makes the camera
    agree with the laser would make "the camera agrees with the laser" circular
    and would destroy the only comparison this study exists to make.

    The floor supplies an independent reference. It is flat, it is in every frame,
    and its plane gives height and tilt at once, measured from exactly the point
    depth is measured from.

  usage, on the JETSON:
    python3 svo_camera_pose.py ~/zedx_vs_lidar_data/svo/w3_5.0m_block.svo2
    python3 svo_camera_pose.py <file> --frames 60 --mode NEURAL
    python3 svo_camera_pose.py ~/zedx_vs_lidar_data/svo/w3_4.0m_block.svo2 --target-range 4.0

  --target-range only scales the closing "how much would this tilt lift the
  target" line. The measured height, pitch and roll owe it nothing.
"""
from __future__ import print_function

import argparse
import math
import sys

import numpy as np

try:
    import pyzed.sl as sl
except ImportError:
    sys.exit('pyzed not available - run this on the Jetson')


def fit_plane(pts, thresh=0.02, iters=250, rng=None):
    """Largest flat surface. Plane is n.x + d = 0 with |n|=1, so |d| is the
    perpendicular distance from the camera's optical centre to it."""
    rng = rng or np.random.default_rng(0)
    n = len(pts)
    if n < 200:
        return None
    best = (0, None, None)
    for _ in range(iters):
        p = pts[rng.choice(n, 3, replace=False)]
        nrm = np.cross(p[1] - p[0], p[2] - p[0])
        ln = np.linalg.norm(nrm)
        if ln < 1e-9:
            continue
        nrm /= ln
        d = -float(nrm.dot(p[0]))
        cnt = int((np.abs(pts.dot(nrm) + d) < thresh).sum())
        if cnt > best[0]:
            best = (cnt, nrm, d)
    if best[1] is None or best[0] < 200:
        return None
    nrm, d = best[1], best[2]
    inl = pts[np.abs(pts.dot(nrm) + d) < thresh]
    c = inl.mean(axis=0)
    _, _, vt = np.linalg.svd(inl - c, full_matrices=False)
    nrm = vt[2] / np.linalg.norm(vt[2])
    d = -float(nrm.dot(c))
    return nrm, d, len(inl)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('svo')
    ap.add_argument('--frames', type=int, default=60)
    ap.add_argument('--mode', default='NEURAL')
    ap.add_argument('--range-min', type=float, default=1.0)
    ap.add_argument('--range-max', type=float, default=6.0)
    ap.add_argument('--target-range', type=float, default=5.0,
                    help='how far the target is, in metres. Only scales the '
                         'closing "this tilt would lift the target by" line; '
                         'the measured height, pitch and roll do not use it. '
                         'Default 5.0 for the station this was written for.')
    a = ap.parse_args()

    init = sl.InitParameters()
    init.set_from_svo_file(a.svo)
    init.depth_mode = getattr(sl.DEPTH_MODE, a.mode)
    init.coordinate_units = sl.UNIT.METER
    init.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
    init.svo_real_time_mode = False
    init.depth_stabilization = 1
    init.camera_disable_self_calib = True      # exactly what the campaign replayed with
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

    total = cam.get_svo_number_of_frames()
    step = max(1, total // a.frames)
    cloud = sl.Mat()
    rng = np.random.default_rng(0)
    H, P, R, N = [], [], [], []

    print('%s' % a.svo)
    print('  %d frames in the recording, sampling every %d' % (total, step))
    for i in range(0, total, step):
        cam.set_svo_position(i)
        if cam.grab(rt) != sl.ERROR_CODE.SUCCESS:
            continue
        cam.retrieve_measure(cloud, sl.MEASURE.XYZ)
        arr = cloud.get_data()[:, :, :3].reshape(-1, 3)
        arr = arr[np.isfinite(arr).all(axis=1)]
        if len(arr) < 2000:
            continue
        rad = np.linalg.norm(arr, axis=1)
        # near, and BELOW the camera: the floor. A wall would fail the axis vote.
        keep = (rad > a.range_min) & (rad < a.range_max) & (arr[:, 2] < -0.2)
        pts = arr[keep]
        if len(pts) > 40000:
            pts = pts[rng.choice(len(pts), 40000, replace=False)]
        fit = fit_plane(pts, rng=rng)
        if not fit:
            continue
        nrm, d, ninl = fit
        if nrm[2] < 0:                      # make the normal point UP
            nrm, d = -nrm, -d
        if abs(nrm[2]) < 0.9:               # not a floor - probably a wall
            continue
        # height is the perpendicular distance to the plane
        H.append(abs(d))
        # Tilt: the floor normal is straight up only if the camera is level.
        #
        # THE SIGN, DERIVED RATHER THAN ASSUMED. With x forward and z up, a camera
        # pitched NOSE DOWN by theta has its axes at x_c=(cos,0,-sin) and
        # z_c=(sin,0,cos) in world coordinates, so world "up" seen from the camera
        # is (-sin(theta), 0, cos(theta)) and atan2(nrm_x, nrm_z) returns MINUS
        # theta. The raw formula therefore reports nose-down as negative, which is
        # the opposite of what a reader expects - and the ROS version of this tool
        # printed exactly that, unnegated, which is how "-2.75 deg nose-up" got
        # into METHODOLOGY_AND_RESULTS 5.1 and sat there contradicting the 2.818
        # deg measured by geometry. They were always the same measurement.
        # Negate here so POSITIVE MEANS NOSE DOWN, as the printout claims.
        P.append(-math.degrees(math.atan2(nrm[0], nrm[2])))
        R.append(-math.degrees(math.atan2(nrm[1], nrm[2])))
        N.append(ninl)
    cam.close()

    if len(H) < 5:
        sys.exit('only %d usable floor fits - not enough to report' % len(H))
    H, P, R = np.array(H), np.array(P), np.array(R)

    def q(v):
        return np.percentile(v, 5), np.median(v), np.percentile(v, 95)

    print('  %d usable floor fits, %d points in the plane on average' % (len(H), int(np.mean(N))))
    print()
    print('  %-8s %10s %10s %10s   %s' % ('', '5th', 'MEDIAN', '95th', 'spread'))
    for nm, v, u in (('height', H, 'm'), ('pitch', P, 'deg'), ('roll', R, 'deg')):
        lo, md, hi = q(v)
        print('  %-8s %10.4f %10.4f %10.4f   %.4f %s' % (nm, lo, md, hi, hi - lo, u))
    print()
    print('  POSITIVE PITCH MEANS NOSE DOWN.')
    pm = float(np.median(P))
    print('  -> the camera looks %s by %.3f degrees'
          % ('DOWN' if pm > 0 else 'UP', abs(pm)))
    # The lift is d*sin(theta), so it depends on how far away the target is.
    # This printed 5.0 m unconditionally, which over-reports a 4 m station by
    # 25 % and a 9 m one by less than half the truth. Take the range instead.
    print('  -> an UNCORRECTED tilt of that size lifts a target at %.1f m by %.0f mm'
          % (a.target_range, 1000 * a.target_range * math.sin(math.radians(abs(pm)))))
    return 0


if __name__ == '__main__':
    sys.exit(main())
