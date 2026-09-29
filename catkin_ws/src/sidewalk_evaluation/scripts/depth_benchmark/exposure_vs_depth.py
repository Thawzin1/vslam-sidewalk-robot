#!/usr/bin/env python3
"""exposure_vs_depth.py - does the lens flare cause the depth error?

WHY THIS EXISTS

    The photograph of this corridor shows heavy veiling flare: rainbow halos
    thrown by the ceiling lights, with the apex ball sitting inside one of them.
    Flare is an internal reflection, so it lands in a DIFFERENT place in the left
    and right images - which is exactly the thing that makes a stereo matcher
    produce a confident wrong answer.

    That is a hypothesis, not a finding. This measures it.

    A glare test was run at 5 m in walkway-1 and killed one glare hypothesis
    there. It does not settle this one: different corridor, different lights,
    and the target sits directly beneath them here.

WHAT IT MEASURES, PER EXPOSURE SETTING

    brightness   the mean image level, so a setting that did nothing is visible
    blown        share of pixels at or near saturation around the balls, which
                 is the flare's own footprint
    depth        the median depth the camera assigns to a small disc at each
                 ball's known image position

    The point cloud is organised on the image grid, so a pixel maps straight to
    a 3D point and no search box or shape fitting is involved.

    Truth comes from the camera's own picture: the two bottom balls are 1.100 m
    apart in the world and about 156 px apart in the image, so the frame is at
    about 8.83 m and an 18 px disc on a ball reads about 8.65 m.

    EVERY SETTING IS READ BACK AFTER BEING SET. An earlier sweep in this project
    reported settings it had never actually applied, because the write silently
    failed - so a value that does not read back is reported as a failure, not
    quietly used.

  usage, on the JETSON (camera node running):
    rosrun sidewalk_evaluation exposure_vs_depth.py
"""
from __future__ import print_function

import sys
import time

import numpy as np

BALLS = {'bot_left': (902, 503), 'bot_right': (1058, 503), 'apex': (963, 340)}
DISC_R = 18
FX = 1258.3945
SEP_PX = 156.0
TRUE_DIST = FX * 1.100 / SEP_PX
BALL_R = 0.20

_DT = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}


def expected():
    r_m = DISC_R * TRUE_DIST / FX
    near = TRUE_DIST - BALL_R
    far = TRUE_DIST - np.sqrt(max(BALL_R ** 2 - r_m ** 2, 0.0))
    return 0.5 * (near + far)


def main():
    import rospy
    import dynamic_reconfigure.client as drc
    from sensor_msgs.msg import Image, PointCloud2

    rospy.init_node('exposure_vs_depth', anonymous=True)
    node = rospy.get_param('~node', '/zedx_front/zed_node')

    # The camera's settings service appears well after the node itself does.
    # A single 20 s attempt failed the first time this ran, 45 s after a camera
    # restart, so wait properly instead of assuming.
    cli = None
    for attempt in range(10):
        try:
            cli = drc.Client(node, timeout=20.0)
            break
        except Exception:
            print("  waiting for the camera's settings service (%d)..." % (attempt + 1))
            sys.stdout.flush()
    if cli is None:
        print("the camera's settings service never appeared - is the node up?")
        return 1

    state = {'img': None, 'cloud': None}

    def img_cb(m):
        a = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, -1)
        state['img'] = a[:, :, :3].mean(axis=2)

    def cloud_cb(m):
        if m.height < 2:
            state['cloud'] = None      # not organised - cannot index by pixel
            return
        names = {f.name: f for f in m.fields}
        raw = np.frombuffer(m.data, dtype=np.uint8).reshape(-1, m.point_step)
        f = names['x']
        col = raw[:, f.offset:f.offset + np.dtype(_DT[f.datatype]).itemsize]
        x = np.ascontiguousarray(col).view(_DT[f.datatype]).ravel()
        state['cloud'] = x.reshape(m.height, m.width).astype(np.float64)

    rospy.Subscriber('/zedx_front/zed_node/left/image_rect_color', Image,
                     img_cb, queue_size=1)
    rospy.Subscriber('/zedx_front/zed_node/point_cloud/cloud_registered',
                     PointCloud2, cloud_cb, queue_size=1, buff_size=2 ** 24)
    rospy.sleep(3.0)
    if state['cloud'] is None:
        print("the point cloud is not organised on the image grid - this test")
        print("indexes it by pixel and cannot run without that.")
        return 1

    h, w = state['cloud'].shape
    yy, xx = np.mgrid[0:h, 0:w]
    masks = {k: ((xx - c[0]) ** 2 + (yy - c[1]) ** 2) <= DISC_R ** 2
             for k, c in BALLS.items()}
    # A generous window around the whole frame, for the flare footprint.
    flare = (xx > 830) & (xx < 1130) & (yy > 250) & (yy < 580)

    exp_med = expected()
    print("=" * 84)
    print("  DOES THE LENS FLARE CAUSE THE DEPTH ERROR?")
    print("=" * 84)
    print("  the picture puts the frame at %.3f m, so a disc on a ball should read %.3f m"
          % (TRUE_DIST, exp_med))
    print("  anything far below that is the fault we are hunting\n")
    print("  %-14s %10s %9s %9s %9s %9s"
          % ("exposure", "brightness", "blown %", "left", "right", "apex"))
    print("  " + "-" * 66)

    settings = [('auto', None), ('60', 60), ('40', 40),
                ('25', 25), ('15', 15), ('8', 8)]
    rows = []
    for label, val in settings:
        try:
            if val is None:
                cli.update_configuration({'auto_exposure_gain': True})
            else:
                # Two calls, not one. Setting auto off and the value together in
                # a single call left auto ON and the value unchanged - the
                # wrapper appears to apply auto first and then ignore the manual
                # value. Turning auto off, letting it settle, then writing the
                # value, works. The read-back below is what caught this.
                cli.update_configuration({'auto_exposure_gain': False})
                rospy.sleep(2.0)
                cli.update_configuration({'exposure': int(val), 'gain': int(val)})
        except Exception as e:
            print("  %-14s could not set (%s)" % (label, e))
            continue
        rospy.sleep(4.0)
        # Read back. A setting that did not take must not be reported as tested.
        auto = rospy.get_param(node + '/auto_exposure_gain', None)
        got = rospy.get_param(node + '/exposure', None)
        if val is None:
            if auto is not True:
                print("  %-14s DID NOT TAKE (auto=%s) - not reported" % (label, auto))
                continue
        elif auto is not False or int(got) != int(val):
            print("  %-14s DID NOT TAKE (auto=%s exposure=%s) - not reported"
                  % (label, auto, got))
            continue

        rospy.sleep(2.0)
        img, cl = state['img'], state['cloud']
        if img is None or cl is None:
            print("  %-14s no frame" % label)
            continue
        bright = float(img[flare].mean())
        blown = float((img[flare] > 245).mean() * 100)
        med = {}
        for k, m in masks.items():
            sel = m & np.isfinite(cl)
            med[k] = float(np.median(cl[sel])) if sel.sum() > 20 else float('nan')
        rows.append((label, bright, blown, med))
        print("  %-14s %10.1f %8.2f%% %9.3f %9.3f %9.3f"
              % (label, bright, blown, med['bot_left'], med['bot_right'], med['apex']))

    # Leave the camera as it was found.
    cli.update_configuration({'auto_exposure_gain': True})
    print("\n  restored to automatic exposure")

    if len(rows) > 1:
        base = rows[0]
        best = min(rows, key=lambda r: abs(np.nanmean([r[3]['bot_left'],
                                                       r[3]['bot_right']]) - exp_med))
        b0 = np.nanmean([base[3]['bot_left'], base[3]['bot_right']])
        b1 = np.nanmean([best[3]['bot_left'], best[3]['bot_right']])
        print("\n  automatic exposure:  %+.0f mm from the picture's answer"
              % ((b0 - exp_med) * 1000))
        print("  best setting (%s):    %+.0f mm" % (best[0], (b1 - exp_med) * 1000))
        gain = abs(b0 - exp_med) - abs(b1 - exp_med)
        if gain * 1000 < 40:
            print("\n  VERDICT: exposure changes the picture but NOT the depth error.")
            print("  Flare is not the cause, and can be struck off the list.")
        else:
            print("\n  VERDICT: darkening the picture moved the depth by %.0f mm."
                  % (gain * 1000))
            print("  Flare is implicated - worth a proper run at the best setting.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
