#!/usr/bin/env python3
"""calibrate_reference.py - give the truth reference an uncertainty, at last.

WHY THIS EXISTS

    Every accuracy number produced on 2026-09-07 was compared against an "image
    ruler": the two bottom balls are 1.100 m apart in the world and 156.0 px
    apart in the image, so the frame is at fx * 1.100 / 156.0 = 8.87 m.

    An independent review found the fatal problem with that. The 156.0 px was
    read BY EYE off a single image, and the sensitivity is

        56.5 mm of range per pixel of separation

    so a one-pixel reading error is +/- 57 mm and two pixels is +/- 113 mm.
    **Every error quoted that day - -48, -70, -95, -361 mm - is the same size as
    its own reference's uncertainty.** The repository's own rule (root ENGINEERING_NOTES.md
    section 4.0) requires each source's uncertainty to be stated BEFORE
    comparing. Until this is fixed, none of those numbers can be falsified.

    There is a second, quieter problem. The 1.100 m base has never been measured
    by us. It comes from Nicolas's code; his prose says the rise was taped at
    ~1.12 m. Both the ruler and the outline estimator scale in direct proportion
    to it, so if it is wrong they are wrong together and neither can reveal it.

WHAT THIS FIXES, AND HOW

    A checkerboard has two properties a ball does not: its corners are located
    to a fraction of a pixel by a detector rather than by eye, and its square
    size is known exactly. Put one on the frame, and the camera's distance to it
    follows from the image alone - no stereo, no hand reading.

        distance, and an uncertainty that comes from the corner scatter across
        many frames rather than from a guess.

    Two numbers come out:

      1. the distance to the board, with a standard error
      2. the pixel-to-metre scale at that distance, which converts the balls'
         measured pixel separation into metres and so MEASURES the base that has
         until now been assumed

    Both are what the rest of the work has been missing.

WHAT YOU NEED TO DO AT THE FRAME

    - Tape a checkerboard flat on the ball frame, facing the camera, as close as
      possible to the plane the three ball centres lie in. Anywhere on the frame
      is fine; it does not need to be centred.
    - Keep it flat. A bowed board is the one error this cannot detect.
    - Tell this script the square size in millimetres and the number of INNER
      corners across and down (a board with 10 x 7 squares has 9 x 6 inner
      corners).

    Nothing else changes. Do not move the frame, the camera or the robot - the
    whole point is to measure the reference for the station as it stands.

  usage, on the JETSON (camera node running):
    rosrun sidewalk_evaluation calibrate_reference.py \\
        _square_mm:=25.0 _cols:=9 _rows:=6 _frames:=30
"""
from __future__ import print_function

import sys

import numpy as np


def main():
    import rospy
    import cv2
    from sensor_msgs.msg import Image, CameraInfo

    rospy.init_node('calibrate_reference', anonymous=True)
    g = rospy.get_param
    square = float(g('~square_mm', 25.0)) / 1000.0
    cols = int(g('~cols', 9))          # inner corners across
    rows = int(g('~rows', 6))          # inner corners down
    want = int(g('~frames', 30))

    state = {'K': None, 'D': None}
    imgs = []

    def info_cb(m):
        if state['K'] is None:
            state['K'] = np.array(m.P).reshape(3, 4)[:, :3]
            state['D'] = np.zeros(5)   # image_rect_color is already rectified

    def img_cb(m):
        if len(imgs) >= want:
            return
        a = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, -1)
        imgs.append(a[:, :, :3].mean(axis=2).astype(np.uint8))

    rospy.Subscriber('/zedx_front/zed_node/left/camera_info', CameraInfo,
                     info_cb, queue_size=1)
    rospy.Subscriber('/zedx_front/zed_node/left/image_rect_color', Image,
                     img_cb, queue_size=1)

    print("looking for a %d x %d inner-corner board, %.1f mm squares..."
          % (cols, rows, square * 1000))
    sys.stdout.flush()
    t0 = rospy.get_time()
    while not rospy.is_shutdown() and (len(imgs) < want or state['K'] is None) \
            and rospy.get_time() - t0 < 180:
        rospy.sleep(0.1)
    if not imgs or state['K'] is None:
        print("no images or no camera_info - is the camera node running?")
        return 1

    K = state['K']
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    obj = np.zeros((rows * cols, 3), np.float32)
    obj[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square

    dists, rmss, found = [], [], 0
    for im in imgs:
        ok, corners = cv2.findChessboardCornersSB(im, (cols, rows),
                                                  cv2.CALIB_CB_EXHAUSTIVE)
        if not ok:
            ok, corners = cv2.findChessboardCorners(
                im, (cols, rows),
                cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
            if ok:
                cv2.cornerSubPix(im, corners, (7, 7), (-1, -1),
                                 (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                                  40, 0.001))
        if not ok:
            continue
        found += 1
        okp, rvec, tvec = cv2.solvePnP(obj, corners, K, state['D'],
                                       flags=cv2.SOLVEPNP_ITERATIVE)
        if not okp:
            continue
        proj, _ = cv2.projectPoints(obj, rvec, tvec, K, state['D'])
        rms = float(np.sqrt(np.mean(np.sum(
            (proj.reshape(-1, 2) - corners.reshape(-1, 2)) ** 2, axis=1))))
        dists.append(float(tvec[2]))       # forward distance, pinhole z
        rmss.append(rms)

    print("\nboard found in %d of %d frames; %d gave a pose"
          % (found, len(imgs), len(dists)))
    if len(dists) < 5:
        print("\nToo few detections to state an uncertainty.")
        print("Check: is the whole board visible, flat, and not blown out?")
        print("Are cols/rows the INNER corner counts, not the square counts?")
        return 1

    d = np.array(dists)
    sd = d.std(ddof=1)
    sem = sd / np.sqrt(len(d))
    print("\n  distance to the board   %.4f m" % d.mean())
    print("  frame-to-frame spread   %.1f mm" % (sd * 1000))
    print("  standard error          %.1f mm      <-- THE NUMBER THAT WAS MISSING"
          % (sem * 1000))
    print("  reprojection residual   %.3f px (mean over frames)" % np.mean(rmss))
    print("  focal length used       fx %.2f  fy %.2f (from camera_info)" % (fx, fy))

    print("\n  For comparison, the reference this replaces:")
    print("    image ruler, 156.0 px read by eye  ->  %.4f m, uncertainty NOT STATED"
          % (fx * 1.100 / 156.0))
    print("    sensitivity of that ruler          ->  %.1f mm of range per pixel"
          % (fx * 1.100 / 156.0 ** 2 * 1000))

    print("\n  MEASURING THE BASE THAT HAS ALWAYS BEEN ASSUMED")
    print("  At %.4f m, one pixel spans %.4f mm." % (d.mean(), d.mean() / fx * 1000))
    print("  So a ball separation measured as S pixels is S x %.4f mm."
          % (d.mean() / fx * 1000))
    print("  Measure S with a detector, not by eye, and the 1.100 m base becomes")
    print("  a measurement instead of an assumption. If the board is not in the")
    print("  balls' plane, correct for the offset before using this.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
