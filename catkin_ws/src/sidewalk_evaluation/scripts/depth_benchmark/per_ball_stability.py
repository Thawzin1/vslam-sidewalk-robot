#!/usr/bin/env python3
"""per_ball_stability.py - measure each ball on its own, with no fitting.

WHY

    Two things went wrong today that this avoids by construction.

    The rigid frame fit gave -112 mm on one run and -289 mm on a repeat of the
    same configuration, on the same target in the same lighting. It optimises,
    so it can land in a different solution each time. Nothing here optimises:
    it reads the depth the camera assigns to a disc of pixels at each ball's
    known image position, and takes a median. Run it twice and it agrees with
    itself or the camera genuinely changed.

    And in a single frame the three balls disagreed by 435 mm - the right one
    accurate to +5 mm, the apex 390 mm out. Any whole-frame number averages
    that away. This keeps the balls apart, which is the only way to see whether
    the pattern is STABLE (one ball is always wrong, so something about where
    it sits in the image matters) or RANDOM (it moves around, so it is noise).

    That distinction decides what to do next, and no number that mixes the
    three balls can make it.

  usage, on the JETSON (camera node running):
    rosrun sidewalk_evaluation per_ball_stability.py _frames:=60 \\
        _balls:='bot_left:732,439;bot_right:1202,439' _ref_dist:=2.88

    ~balls defaults to the 9 m station's positions. Pass it at every other
    station - a disc left at the wrong place reads the wall, not the ball.
"""
from __future__ import print_function

import sys

import numpy as np

# Default ball positions: the WALKWAY-3 9 m station, kept so that every run made
# in a completely different place in the image at every station, and a disc left
# at the 9 m position while the frame stands at 3 m reads the WALL BEHIND IT and
# reports a confident, wrong number. Pass ~balls at every other station.
BALLS = [('bot_left', 902, 503), ('bot_right', 1058, 503), ('apex', 963, 340)]
DISC_R = 18
BALL_R = 0.20

_DT = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}


def parse_balls(text):
    """'name:cx,cy;name:cx,cy' -> [(name, cx, cy), ...]"""
    out = []
    for part in text.split(';'):
        part = part.strip()
        if not part:
            continue
        nm, xy = part.split(':')
        cx, cy = xy.split(',')
        out.append((nm.strip(), int(round(float(cx))), int(round(float(cy)))))
    return out


def expected(ref_dist, disc_r, fx):
    """What a disc of pixels centred on a ball SHOULD read.

    The disc covers a curved cap, not a flat patch, so its depths run from the
    ball's near pole (ref_dist - BALL_R) to the cap's rim. Return the midpoint.
    """
    r_m = disc_r * ref_dist / fx
    near = ref_dist - BALL_R
    far = ref_dist - np.sqrt(max(BALL_R ** 2 - r_m ** 2, 0.0))
    return 0.5 * (near + far)


def main():
    import rospy
    from sensor_msgs.msg import PointCloud2, CameraInfo

    global BALLS, DISC_R
    rospy.init_node('per_ball_stability', anonymous=True)
    want = int(rospy.get_param('~frames', 60))
    label = rospy.get_param('~label', 'run')
    if rospy.get_param('~balls', ''):
        BALLS = parse_balls(rospy.get_param('~balls'))
    DISC_R = int(rospy.get_param('~disc_r', DISC_R))
    # The reference distance is an INPUT, never a built-in. The 8.87 m "image
    # ruler" this script used to hard-code was withdrawn (GUIDELINES section 20):
    # it measured the dark panels on the balls, not the ball centres. With no
    # reference given, the medians and spreads are still reported - only the
    # error column is withheld, because there is nothing honest to subtract.
    ref_dist = float(rospy.get_param('~ref_dist', 0.0))

    fxs = []
    rospy.Subscriber('/zedx_front/zed_node/left/camera_info', CameraInfo,
                     lambda m: fxs.append(float(m.P[0])), queue_size=1)
    per = {b[0]: [] for b in BALLS}
    fill = {b[0]: [] for b in BALLS}
    grabbed = [0]
    masks = {}

    def cb(m):
        if grabbed[0] >= want or m.height < 2:
            return
        names = {f.name: f for f in m.fields}
        raw = np.frombuffer(m.data, dtype=np.uint8).reshape(-1, m.point_step)
        f = names['x']
        col = raw[:, f.offset:f.offset + np.dtype(_DT[f.datatype]).itemsize]
        x = np.ascontiguousarray(col).view(_DT[f.datatype]).ravel()
        x = x.reshape(m.height, m.width).astype(np.float64)
        if not masks:
            yy, xx = np.mgrid[0:m.height, 0:m.width]
            for nm, cx, cy in BALLS:
                masks[nm] = ((xx - cx) ** 2 + (yy - cy) ** 2) <= DISC_R ** 2
        ok = np.isfinite(x)
        for nm, _, _ in BALLS:
            sel = masks[nm] & ok
            fill[nm].append(sel.sum() / float(masks[nm].sum()))
            if sel.sum() > 20:
                per[nm].append(float(np.median(x[sel])))
        grabbed[0] += 1

    rospy.Subscriber('/zedx_front/zed_node/point_cloud/cloud_registered',
                     PointCloud2, cb, queue_size=1, buff_size=2 ** 24)
    t0 = rospy.get_time()
    while not rospy.is_shutdown() and grabbed[0] < want and rospy.get_time() - t0 < 180:
        rospy.sleep(0.05)
    if grabbed[0] < 5:
        print("only %d frames - is the camera publishing an organised cloud?" % grabbed[0])
        return 1

    fx = fxs[-1] if fxs else 1258.46
    e = expected(ref_dist, DISC_R, fx) if ref_dist > 0 else None
    print("=" * 78)
    print("  EACH BALL ON ITS OWN - no fitting, nothing that can converge differently")
    print("=" * 78)
    print("  [%s]  %d frames, fx %.2f, disc radius %d px" % (label, grabbed[0], fx, DISC_R))
    print("  discs at: %s" % ", ".join("%s (%d,%d)" % b for b in BALLS))
    if e is None:
        print("  NO REFERENCE DISTANCE GIVEN - medians only, no error column.\n")
    else:
        print("  reference %.3f m given, so each disc should read %.3f m\n"
              % (ref_dist, e))
    print("  %-10s %10s %10s %10s %10s"
          % ("ball", "median m", "error mm", "spread mm", "filled %"))
    print("  " + "-" * 54)
    meds = {}
    for nm, _, _ in BALLS:
        v = np.array(per[nm])
        if len(v) < 3:
            print("  %-10s no depth" % nm)
            continue
        meds[nm] = float(np.median(v))
        err = "%+10.0f" % ((np.median(v) - e) * 1000) if e is not None else "%10s" % "-"
        print("  %-10s %10.3f %s %10.1f %9.0f%%"
              % (nm, np.median(v), err, v.std(ddof=1) * 1000, 100 * np.mean(fill[nm])))
    if len(meds) >= 2:
        vals = list(meds.values())
        print("\n  the %d balls disagree with each other by %.0f mm"
              % (len(vals), (max(vals) - min(vals)) * 1000))
        print("  worst: %s" % min(meds, key=lambda k: meds[k]))
        print("\n  Run this twice. If the SAME ball is worst both times, the fault")
        print("  depends on where the ball sits in the image - which points at the")
        print("  lens flare, the background behind it, or the ball's own surface.")
        print("  If the worst one moves, it is noise and the pattern means nothing.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
