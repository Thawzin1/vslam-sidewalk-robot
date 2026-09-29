#!/usr/bin/env python3
"""two_ball_selftest.py - let the frame measure the camera, with no reference.

WHY THIS IS DIFFERENT FROM EVERY OTHER ACCURACY TEST HERE

    Every accuracy number in this project so far has been compared against
    something outside the camera: a tape, a laser plus a 114.3 mm mount offset,
    or the withdrawn image ruler. Each carries its own uncertainty into the
    answer, and the mount offset in particular has blocked accuracy claims for
    weeks.

    The frame removes that. Its two lower balls are 1.100 m apart because it is
    a rigid steel frame, and that number is the SAME 1.100 m the frozen detector
    is built around. So if the camera reconstructs both ball centres in 3D and
    measures the distance between them, it can be scored against a length that
    is already known - with no tape, no laser, no mount offset and no
    front-glass-to-lens distance anywhere in the chain.

    A depth error that is a SCALE error shows up here directly: if every depth
    is 2 % short, the reconstructed separation is 2 % short too, because the
    sideways size of anything is its pixel offset times its depth. A depth error
    that is a constant OFFSET barely shows up at all. So this separates the two,
    which no comparison against an external distance can do.

WHY IT WORKS AT 3 m WHERE THE FROZEN DETECTOR DOES NOT

    The detector needs three ball centres to build its triangle, and at 3 m the
    top ball is off the edge of the picture. This needs only the two lower ones,
    which are fully visible and fill their discs completely.

WHAT IT DOES NOT MEASURE

    Not the absolute distance to the frame - only the scale. A camera reading
    every distance 2 % short and one reading them correctly both get the base
    right if their error is a pure offset. Use it alongside the per-ball depth,
    not instead of it.

  usage, on the JETSON (camera node running):
    rosrun sidewalk_evaluation two_ball_selftest.py _frames:=200 \\
        _balls:='bot_left:730,443;bot_right:1196,443' _label:=3m_NEURAL
"""
from __future__ import print_function

import sys

import numpy as np

BALL_R = 0.20
TRUE_BASE = 1.100        # the frame's own base, and the detector's own constant
DISC_R = 18

_DT = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}


def parse_balls(text):
    out = []
    for part in text.split(';'):
        part = part.strip()
        if not part:
            continue
        nm, xy = part.split(':')
        cx, cy = xy.split(',')
        out.append((nm.strip(), int(round(float(cx))), int(round(float(cy)))))
    return out


def centre_of(u, v, x_cap, fx, fy, cx, cy, disc_r):
    """3D position of a ball's CENTRE, from the forward distance its disc reads.

    The disc sits on the near cap, not on the centre, so step back along the
    line of sight. How far back depends on how much of the cap the disc covers:
    a disc of radius r on a sphere of radius R has its median depth at
    sqrt(R^2 - (r/sqrt2)^2) in front of the centre.
    """
    ray = np.array([1.0, -(u - cx) / fx, -(v - cy) / fy])
    unit = ray / np.linalg.norm(ray)
    r_m = disc_r * x_cap / fx
    back = np.sqrt(max(BALL_R ** 2 - (r_m / np.sqrt(2.0)) ** 2, 0.0))
    return ray * x_cap + back * unit


def main():
    import rospy
    from sensor_msgs.msg import PointCloud2, CameraInfo

    rospy.init_node('two_ball_selftest', anonymous=True)
    g = rospy.get_param
    want = int(g('~frames', 200))
    label = g('~label', 'run')
    disc_r = int(g('~disc_r', DISC_R))
    balls = parse_balls(g('~balls', ''))
    if len(balls) < 2:
        print("give at least the two lower balls: "
              "_balls:='bot_left:730,443;bot_right:1196,443'")
        return 2
    balls = balls[:2]

    K, masks, grabbed = {}, {}, [0]
    per = {b[0]: [] for b in balls}
    seps = []

    def info_cb(m):
        if not K:
            K['fx'], K['fy'] = float(m.P[0]), float(m.P[5])
            K['cx'], K['cy'] = float(m.P[2]), float(m.P[6])

    def cb(m):
        if grabbed[0] >= want or m.height < 2 or not K:
            return
        names = {f.name: f for f in m.fields}
        raw = np.frombuffer(m.data, dtype=np.uint8).reshape(-1, m.point_step)
        f = names['x']
        w = np.dtype(_DT[f.datatype]).itemsize
        col = raw[:, f.offset:f.offset + w]
        x = np.ascontiguousarray(col).view(_DT[f.datatype]).ravel()
        x = x.reshape(m.height, m.width).astype(np.float64)
        if not masks:
            yy, xx = np.mgrid[0:m.height, 0:m.width]
            for nm, u, v in balls:
                masks[nm] = ((xx - u) ** 2 + (yy - v) ** 2) <= disc_r ** 2
        ok = np.isfinite(x)
        got = {}
        for nm, u, v in balls:
            sel = masks[nm] & ok
            if sel.sum() > 20:
                got[nm] = float(np.median(x[sel]))
                per[nm].append(got[nm])
        if len(got) == 2:
            P = [centre_of(u, v, got[nm], K['fx'], K['fy'], K['cx'], K['cy'], disc_r)
                 for nm, u, v in balls]
            seps.append(float(np.linalg.norm(P[0] - P[1])))
        grabbed[0] += 1

    rospy.Subscriber('/zedx_front/zed_node/left/camera_info', CameraInfo,
                     info_cb, queue_size=1)
    rospy.Subscriber('/zedx_front/zed_node/point_cloud/cloud_registered',
                     PointCloud2, cb, queue_size=1, buff_size=2 ** 24)
    t0 = rospy.get_time()
    while not rospy.is_shutdown() and grabbed[0] < want \
            and rospy.get_time() - t0 < 600:
        rospy.sleep(0.05)

    if len(seps) < 5:
        print("only %d usable frames of %d - are both discs on a ball?"
              % (len(seps), grabbed[0]))
        return 1

    s = np.array(seps)
    err = s.mean() - TRUE_BASE
    print("=" * 76)
    print("  THE FRAME MEASURING THE CAMERA - no tape, no laser, no mount offset")
    print("=" * 76)
    print("  [%s]  %d frames, discs at %s"
          % (label, len(s), ", ".join("%s (%d,%d)" % b for b in balls)))
    print("  fx %.2f, disc radius %d px\n" % (K['fx'], disc_r))
    for nm, _, _ in balls:
        v = np.array(per[nm])
        print("  %-10s depth %.4f m   spread %.1f mm over %d frames"
              % (nm, np.median(v), v.std(ddof=1) * 1000, len(v)))
    print("")
    print("  the two ball centres come out   %.4f m apart" % s.mean())
    print("  the frame says they are         %.4f m apart" % TRUE_BASE)
    print("  ERROR                           %+.1f mm   (%+.2f %%)"
          % (err * 1000, 100 * err / TRUE_BASE))
    print("  frame-to-frame spread           %.1f mm" % (s.std(ddof=1) * 1000))
    print("  standard error of the mean      %.2f mm"
          % (s.std(ddof=1) / np.sqrt(len(s)) * 1000))
    print("")
    print("  A percentage error here is a SCALE error in the depth. A camera")
    print("  reading every distance short by the same FRACTION gets this wrong;")
    print("  one reading every distance short by the same NUMBER OF MILLIMETRES")
    print("  gets it right. That is the distinction no comparison against an")
    print("  external distance can make, and it is why this test exists.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
