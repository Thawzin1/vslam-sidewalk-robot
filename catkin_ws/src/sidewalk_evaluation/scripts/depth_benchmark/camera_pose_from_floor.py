#!/usr/bin/env python3
"""camera_pose_from_floor.py - measure how high the camera sits, and how it is tilted,
using the floor itself as the reference.

WHY THIS EXISTS

    The sphere-frame test needs the camera's height above the floor, because the
    region-of-interest box is specified relative to the FLOOR (so it is the same
    box for every sensor) and the detector needs it in SENSOR coordinates. Nicolas
    measured 0.79 m for the Ouster and 0.97 m for the Helios. The camera needs its
    own number.

    A tape measure cannot give it. The tape reaches the TOP OF THE HOUSING, but
    depth is measured from the LEFT LENS'S OPTICAL CENTRE, which sits inside the
    housing at a depth the datasheet does not publish. Measuring to the housing
    and calling it the camera height bakes in an unknown error of a few centimetres.

    So let the camera measure itself. The floor is a large, genuinely flat surface
    that is already in every frame. Fit a plane to it; the perpendicular distance
    from the camera to that plane IS the camera height, measured from exactly the
    point depth is measured from. The plane's tilt gives pitch and roll for free.

    Plain terms: instead of measuring down to the floor with a ruler, ask the
    camera how far away the floor looks. It answers from the right place.

WHAT COMES OUT

    height  metres from the optical centre straight down to the floor
    pitch   nose-up / nose-down tilt, degrees. Positive = looking down.
    roll    sideways tilt, degrees. Positive = right side down.

    Each with its spread across many frames. A stable answer means the fit is
    trustworthy; a wandering one means the floor is not being seen well enough
    (too dark, too shiny, too little of it in view) and the number must not be used.

  usage, on the JETSON with the camera running:
    rosrun sidewalk_evaluation camera_pose_from_floor.py \
        _cloud_topic:=/zedx_front/zed_node/point_cloud/cloud_registered _frames:=100
"""
from __future__ import print_function

import math

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2

_DT = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}


def fit_plane_ransac(pts, thresh=0.02, iters=200, rng=None):
    """Largest flat surface in the set. Returns (normal(3,), d, inlier_mask).

    Plane is  n . x + d = 0  with |n| = 1, so the distance from the origin - the
    camera - to the plane is simply |d|.
    """
    if rng is None:
        rng = np.random.default_rng(0)
    n = len(pts)
    if n < 50:
        return None
    best = (0, None, None)
    for _ in range(iters):
        idx = rng.choice(n, 3, replace=False)
        p0, p1, p2 = pts[idx]
        nrm = np.cross(p1 - p0, p2 - p0)
        ln = np.linalg.norm(nrm)
        if ln < 1e-9:
            continue
        nrm = nrm / ln
        d = -float(nrm.dot(p0))
        dist = np.abs(pts.dot(nrm) + d)
        mask = dist < thresh
        cnt = int(mask.sum())
        if cnt > best[0]:
            best = (cnt, nrm, d)
    if best[1] is None or best[0] < 50:
        return None
    # Least-squares refit on the inliers: RANSAC picks the plane, this places it.
    nrm, d = best[1], best[2]
    inl = pts[np.abs(pts.dot(nrm) + d) < thresh]
    c = inl.mean(axis=0)
    _, _, vt = np.linalg.svd(inl - c, full_matrices=False)
    nrm = vt[2] / np.linalg.norm(vt[2])
    d = -float(nrm.dot(c))
    mask = np.abs(pts.dot(nrm) + d) < thresh
    return nrm, d, mask


def describe_frame(nrm):
    """Which way is 'down' in this cloud? Returns (axis_index, sign, name).

    The ZED wrapper can publish in either of two conventions and the answer
    changes which number is the height. Rather than assume, read it off the
    floor's own normal: the floor is below the camera, so its normal points
    predominantly along whichever axis is vertical.
    """
    i = int(np.argmax(np.abs(nrm)))
    s = 1.0 if nrm[i] > 0 else -1.0
    name = {0: 'x', 1: 'y', 2: 'z'}[i]
    convention = ("OPTICAL (x right, y DOWN, z forward)" if i == 1 else
                  "BODY / REP-103 (x forward, y left, z UP)" if i == 2 else
                  "UNEXPECTED - vertical axis looks like x")
    return i, s, name, convention


class FloorPose(object):
    def __init__(self):
        g = rospy.get_param
        self.topic = g('~cloud_topic',
                       '/zedx_front/zed_node/point_cloud/cloud_registered')
        self.want = int(g('~frames', 100))
        self.thresh = float(g('~plane_thresh', 0.02))
        self.rmax = float(g('~range_max', 6.0))   # only nearby floor: flat and dense
        self.rmin = float(g('~range_min', 0.8))
        self.rng = np.random.default_rng(0)
        self.H, self.P, self.R = [], [], []
        self.n_seen = 0
        self.axis_votes = {}
        self.frame_axis = []
        rospy.Subscriber(self.topic, PointCloud2, self.cb, queue_size=2,
                         buff_size=2 ** 24)
        rospy.loginfo('camera_pose_from_floor: %s  collecting %d frames',
                      self.topic, self.want)

    def cb(self, msg):
        if len(self.H) >= self.want:
            return
        self.n_seen += 1
        names = [f.name for f in msg.fields]
        dt = np.dtype(dict(names=names,
                           formats=[_DT[f.datatype] for f in msg.fields],
                           offsets=[f.offset for f in msg.fields],
                           itemsize=msg.point_step))
        a = np.frombuffer(msg.data, dtype=dt)
        xyz = np.stack([a['x'], a['y'], a['z']], axis=1).astype(np.float64)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        r = np.linalg.norm(xyz, axis=1)
        xyz = xyz[(r > self.rmin) & (r < self.rmax)]
        if len(xyz) < 200:
            return
        if len(xyz) > 40000:                       # RANSAC does not need them all
            xyz = xyz[self.rng.choice(len(xyz), 40000, replace=False)]

        res = fit_plane_ransac(xyz, self.thresh, 200, self.rng)
        if res is None:
            return
        nrm, d, mask = res
        i, s, name, conv = describe_frame(nrm)
        self.axis_votes[name] = self.axis_votes.get(name, 0) + 1
        self.frame_axis.append(name)

        # Height is the PERPENDICULAR distance from the camera (the origin) to the
        # plane, which is exactly |d| once the normal is unit length. Sign-free, so
        # it does not matter which way the normal happens to point.
        #
        # Perpendicular is the right choice and tilt does not spoil it: the floor is
        # horizontal in the real world, so the shortest line from the camera to the
        # floor is vertical, whichever way the camera is pointing. Measuring instead
        # along the camera's own "down" axis WOULD change with tilt - at 5 degrees of
        # pitch that reads about 2.5 mm long on a 0.65 m mount - which is why this
        # uses the plane distance and not a coordinate.
        height = abs(d)

        # Tilt: angle between the floor normal and the vertical axis, split into
        # the two directions. Normal is flipped to point UP from the floor first.
        up = nrm * (-s)
        if i == 1:      # optical frame: y is down, forward is z, right is x
            pitch = math.degrees(math.atan2(up[2], abs(up[1])))
            roll = math.degrees(math.atan2(up[0], abs(up[1])))
        else:           # body frame: z is up, forward is x, left is y
            pitch = math.degrees(math.atan2(-up[0], abs(up[2])))
            roll = math.degrees(math.atan2(up[1], abs(up[2])))

        self.H.append(height)
        self.P.append(pitch)
        self.R.append(roll)
        if len(self.H) % 20 == 0:
            rospy.loginfo('  %d/%d  height %.4f m  pitch %+.2f deg  roll %+.2f deg '
                          '(%d floor points)',
                          len(self.H), self.want, height, pitch, roll, int(mask.sum()))
        if len(self.H) >= self.want:
            self.report(conv)
            rospy.signal_shutdown('done')

    def report(self, conv):
        H = np.array(self.H)
        P = np.array(self.P)
        R = np.array(self.R)

        # DROP FRAMES THAT FITTED THE WRONG SURFACE BEFORE JUDGING STABILITY.
        # In a corridor the odd frame locks onto a WALL instead of the floor. Its
        # height is nonsense, and one such frame in a hundred is enough to blow up
        # the standard deviation: measured 23.1 mm of "spread" on a set whose
        # 5th-95th range was 7.1 mm. The first version of this reported UNSTABLE
        # on an excellent measurement for exactly that reason.
        #
        # The frames themselves say which ones are wrong - a wall fit votes for a
        # different vertical axis - so use the majority axis and discard the rest.
        keep = np.ones(len(H), bool)
        if self.axis_votes:
            major = max(self.axis_votes, key=self.axis_votes.get)
            n_bad = sum(v for k, v in self.axis_votes.items() if k != major)
        else:
            major, n_bad = '?', 0
        if len(self.frame_axis) == len(H):
            keep = np.array([a == major for a in self.frame_axis])
        Hk, Pk, Rk = H[keep], P[keep], R[keep]
        if len(Hk) < 10:                      # not enough left to judge anything
            Hk, Pk, Rk = H, P, R

        def spread(a):
            """5th-95th range. Robust: a handful of bad fits cannot inflate it,
            where a standard deviation can and did."""
            return float(np.percentile(a, 95) - np.percentile(a, 5))

        def line(label, a, unit, dp):
            # Build each number with its own width, then print with plain %s.
            # An earlier version left a dead .format() string here that used
            # "{.{0}f}", which is not valid format-spec syntax - it raised
            # ValueError and killed the report AFTER all the work was done.
            num = lambda v: ('%%.%df' % dp) % v
            print('  %-22s %s %s   5th-95th spread %s   (%s to %s)'
                  % (label, num(np.median(a)), unit, num(spread(a)),
                     num(np.percentile(a, 5)), num(np.percentile(a, 95))))

        print()
        print('=' * 72)
        print('  CAMERA POSE FROM THE FLOOR      %d frames of %d offered'
              % (len(H), self.n_seen))
        print('=' * 72)
        print('  cloud frame convention : %s' % conv)
        print('  axis votes             : %s' % self.axis_votes)
        print('-' * 72)
        if n_bad:
            print('  DISCARDED %d of %d frames that fitted a different surface '
                  '(vertical axis %s, not %s) -' % (n_bad, len(H),
                  ', '.join(k for k in self.axis_votes if k != major), major))
            print('  almost always a WALL rather than the floor. Judged on the '
                  'remaining %d.' % len(Hk))
            print('-' * 72)
        line('height above floor', Hk, 'm', 4)
        line('pitch (down positive)', Pk, 'deg', 2)
        line('roll (right down pos)', Rk, 'deg', 2)
        print('-' * 72)

        # Robust thresholds. 5th-95th range, not standard deviation: one bad fit
        # in a hundred must not be able to condemn a good measurement.
        stable = (spread(Hk) < 0.010) and (spread(Pk) < 0.5) and (spread(Rk) < 0.5)
        print('  VERDICT: %s' % (
            'STABLE - safe to use' if stable else
            'UNSTABLE - do NOT use this number. The floor is not being seen well '
            'enough.\n           Try more light, point the camera at more open '
            'floor, or raise range_max.'))
        print()
        print('  Paste into sphere_centroid_zedx.launch:')
        print('      <arg name="sensor_h" value="%.4f"/>' % np.median(Hk))
        print()
        print('  Cross-check against the tape: you measured the TOP OF THE HOUSING at')
        print('  0.686 m (2 ft 3 in). The optical centre sits below that, so this')
        print('  number should come out a few centimetres SMALLER. If it is larger,')
        print('  something is wrong - say so rather than using it.')
        print('=' * 72)


if __name__ == '__main__':
    rospy.init_node('camera_pose_from_floor')
    FloorPose()
    rospy.spin()
