#!/usr/bin/env python3
"""station_timelapse.py - record the detector deciding, frame by frame.

WHAT IT RECORDS AND WHY IT RECORDS RATHER THAN DRAWS

    A still picture of a point cloud shows what the detector concluded. It does
    not show the detector WORKING - the ball surface filling in as frames
    accumulate, or which points get taken and which get thrown away each time.
    That is a moving picture, and this records the material for one.

    It deliberately does NOT render. Drawing frames on the Jetson would compete
    with the run being recorded, and the run is the thing that matters. This
    saves one compact snapshot per accepted frame and the animation is built
    afterwards on any computer, where nothing is at stake.

WHAT ONE SNAPSHOT HOLDS

    the three sphere centres the detector fitted, and a subsample of the points
    in the search box, each tagged with the decision made about it:

        0, 1, 2   accepted - within the detector's own 30 mm window of that ball
        -1        rejected - not on any ball

    The tag uses the detector's own inlier threshold, not a lookalike, so the
    animation shows the real decision rather than a re-enactment of it.

  usage, on the JETSON (camera, roi_viz and the detector running):
    rosrun sidewalk_evaluation station_timelapse.py _dist:=4.0 _tag:=NEURAL \\
        _snaps:=80 _points:=18000
"""
from __future__ import print_function

import os
import sys
import time

import numpy as np

BALL_R = 0.20
INLIER = 0.03          # the detector's own window; not an independent choice

_DT = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}


def cloud_xyz(msg):
    names = {f.name: f for f in msg.fields}
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
    out = []
    for k in ('x', 'y', 'z'):
        f = names[k]
        w = np.dtype(_DT[f.datatype]).itemsize
        col = raw[:, f.offset:f.offset + w]
        out.append(np.ascontiguousarray(col).view(_DT[f.datatype]).ravel().astype(np.float32))
    a = np.column_stack(out)
    return a[np.isfinite(a).all(axis=1)]


def main():
    import rospy
    from sensor_msgs.msg import PointCloud2
    from visualization_msgs.msg import MarkerArray

    rospy.init_node('station_timelapse', anonymous=True)
    g = rospy.get_param
    dist = float(g('~dist', 0.0))
    tag = g('~tag', 'run')
    want = int(g('~snaps', 80))
    npts = int(g('~points', 18000))
    outdir = g('~out', '~/station_figures')
    secs = float(g('~max_seconds', 2400))
    if dist <= 0:
        print('give the station distance: _dist:=4.0')
        return 2
    stem = 'w3_%.1fm_%s' % (dist, tag)
    d = os.path.join(outdir, stem)
    if not os.path.isdir(d):
        os.makedirs(d)

    box = {'pts': None}
    snaps = {'P': [], 'L': [], 'C': [], 't': []}
    t0 = [None]

    def cloud_cb(m):
        try:
            box['pts'] = cloud_xyz(m)
        except Exception:
            pass

    def marker_cb(ma):
        if len(snaps['P']) >= want or box['pts'] is None or not len(box['pts']):
            return
        C = np.array([[mk.pose.position.x, mk.pose.position.y, mk.pose.position.z]
                      for mk in ma.markers if mk.ns == 'spheres'], dtype=np.float32)
        if len(C) != 3:
            return
        P = box['pts']
        if len(P) > npts:
            P = P[np.random.choice(len(P), npts, replace=False)]
        D = np.stack([np.abs(np.linalg.norm(P - c, axis=1) - BALL_R) for c in C], axis=1)
        who = np.argmin(D, axis=1).astype(np.int8)
        lab = np.where(D[np.arange(len(P)), who] <= INLIER, who, -1).astype(np.int8)
        now = time.time()
        if t0[0] is None:
            t0[0] = now
        snaps['P'].append(P.astype(np.float32))
        snaps['L'].append(lab)
        snaps['C'].append(C)
        snaps['t'].append(np.float32(now - t0[0]))
        n = len(snaps['P'])
        print('  snapshot %d/%d  (%.0f %% of the box points landed on a ball)'
              % (n, want, 100.0 * (lab >= 0).mean()))
        sys.stdout.flush()

    rospy.Subscriber('/roi_viz_cam/inside', PointCloud2, cloud_cb,
                     queue_size=1, buff_size=2 ** 25)
    rospy.Subscriber('/sphere_centroid/markers', MarkerArray, marker_cb, queue_size=5)

    print('recording up to %d accepted frames (%d points each)...' % (want, npts))
    sys.stdout.flush()
    start = rospy.get_time()
    beat = start
    # A heartbeat, because an empty log is indistinguishable from a dead one.
    # It reports what has ARRIVED as well as what has been kept, so "the
    # detector is accepting nothing" and "this script is broken" cannot be
    # confused - they look identical without it.
    while not rospy.is_shutdown() and len(snaps['P']) < want \
            and rospy.get_time() - start < secs:
        rospy.sleep(0.2)
        if rospy.get_time() - beat > 60:
            beat = rospy.get_time()
            print('  ... %.0f min in: %d snapshots kept, box cloud %s'
                  % ((rospy.get_time() - start) / 60.0, len(snaps['P']),
                     ('%d points' % len(box['pts'])) if box['pts'] is not None
                     else 'NOT ARRIVING'))
            sys.stdout.flush()

    n = len(snaps['P'])
    if n == 0:
        print('NO ACCEPTED FRAMES in the window. Nothing to animate - which is '
              'itself the result at a station where the detector fails.')
        return 1
    path = os.path.join(d, stem + '_timelapse.npz')
    np.savez_compressed(path,
                        P=np.stack(snaps['P']), L=np.stack(snaps['L']),
                        C=np.stack(snaps['C']), t=np.array(snaps['t']),
                        ball_r=np.float32(BALL_R), inlier=np.float32(INLIER))
    acc = float(np.mean([(l >= 0).mean() for l in snaps['L']]))
    print('\n%d snapshots -> %s' % (n, path))
    print('mean share of box points accepted onto a ball: %.1f %%' % (100 * acc))
    print('elapsed over the recording: %.1f s' % snaps['t'][-1])
    return 0


if __name__ == '__main__':
    sys.exit(main())
