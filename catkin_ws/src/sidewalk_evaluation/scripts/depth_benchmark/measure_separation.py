#!/usr/bin/env python3
"""measure_separation.py - replace the eye-read 156.0 px with a measurement.

THE PROBLEM

    Every accuracy figure of 2026-09-07 was compared against an "image ruler":
    the two bottom balls are 1.100 m apart in the world and 156.0 px apart in
    the image, so the frame is at fx * 1.100 / 156.0.

    That 156.0 was read BY EYE off a single image, and the sensitivity is
    56.5 mm of range per pixel. One pixel of reading error is +/- 57 mm - the
    same size as the errors being reported against it. An independent review
    called this the worst methodological gap of the day, and it was right.

    The ruler has TWO uncertain inputs. This script fixes one of them; the other
    needs a tape.

      the pixel separation   read by eye, +/- 1-2 px   -> FIXED HERE
      the 1.100 m base       never measured by us      -> needs a tape measure

WHY THE SEPARATION CAN BE MEASURED WELL BUT THE RADIUS CANNOT

    The outline detector has a known bias: a threshold-based edge on a soft
    silhouette lands slightly outside the true one, so measured radii come out
    about 0.35 px too large (see outline_frame_fit.py). That bias ruins a
    distance derived from ball SIZE.

    It does not ruin one derived from ball SEPARATION. If both balls' outlines
    are inflated by the same amount, both centres stay put and the distance
    between them is unchanged. The bias is common-mode and cancels.

    So this measures centre separation only, and deliberately ignores the radii.

WHAT IT REPORTS

    the separation in pixels, its standard error over many frames, and what that
    implies for the ruler's distance - with an uncertainty attached, which is
    the thing that has been missing.

  usage, on the JETSON (camera node running):
    rosrun sidewalk_evaluation measure_separation.py _frames:=40
"""
from __future__ import print_function

import sys

import numpy as np

# Approximate image positions of the two BOTTOM balls, used only to start the
# ray search. The detector re-solves each centre from detected edges, so these
# do not propagate into the answer.
GUESS = [(902, 503), (1058, 503)]
BASE_ASSUMED = 1.100          # the value that has never been measured by us


def find_centre(gray, gx, gy, rmin=14, rmax=42, rays=96):
    """Locate one ball's outline centre sub-pixel, by casting rays and fitting
    a circle to the strongest brightness step along each."""
    h, w = gray.shape
    cx, cy, rr = float(gx), float(gy), 0.0
    for _ in range(5):
        pts = []
        for th in np.linspace(0, 2 * np.pi, rays, endpoint=False):
            dx, dy = np.cos(th), np.sin(th)
            rs = np.arange(rmin, rmax, 0.25)
            xs = np.clip((cx + dx * rs).astype(int), 0, w - 1)
            ys = np.clip((cy + dy * rs).astype(int), 0, h - 1)
            prof = gray[ys, xs].astype(float)
            if len(prof) < 8:
                continue
            gmag = np.abs(np.diff(prof))
            i = int(np.argmax(gmag))
            if gmag[i] < 6:
                continue
            # Parabolic interpolation on the gradient peak: sub-pixel edge.
            if 0 < i < len(gmag) - 1:
                a, b, c = gmag[i - 1], gmag[i], gmag[i + 1]
                denom = (a - 2 * b + c)
                off = 0.5 * (a - c) / denom if abs(denom) > 1e-9 else 0.0
            else:
                off = 0.0
            r = rs[i] + off * 0.25
            pts.append((cx + dx * r, cy + dy * r))
        if len(pts) < 24:
            return None
        P = np.array(pts)
        for _ in range(3):
            A = np.column_stack([2 * P[:, 0], 2 * P[:, 1], np.ones(len(P))])
            b = (P ** 2).sum(axis=1)
            sol, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
            cx, cy = float(sol[0]), float(sol[1])
            rr = float(np.sqrt(max(sol[2] + cx * cx + cy * cy, 1e-6)))
            d = np.abs(np.linalg.norm(P - np.array([cx, cy]), axis=1) - rr)
            keep = d < np.percentile(d, 70)
            if keep.sum() >= 18:
                P = P[keep]
    return cx, cy, rr


def main():
    import rospy
    from sensor_msgs.msg import Image, CameraInfo

    rospy.init_node('measure_separation', anonymous=True)
    want = int(rospy.get_param('~frames', 40))
    K = {'fx': None}
    imgs = []

    def info_cb(m):
        if K['fx'] is None:
            K['fx'] = float(m.P[0])

    def img_cb(m):
        if len(imgs) >= want:
            return
        a = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, -1)
        imgs.append(a[:, :, :3].mean(axis=2))

    rospy.Subscriber('/zedx_front/zed_node/left/camera_info', CameraInfo,
                     info_cb, queue_size=1)
    rospy.Subscriber('/zedx_front/zed_node/left/image_rect_color', Image,
                     img_cb, queue_size=1)
    t0 = rospy.get_time()
    while not rospy.is_shutdown() and (len(imgs) < want or K['fx'] is None) \
            and rospy.get_time() - t0 < 150:
        rospy.sleep(0.1)
    if not imgs or K['fx'] is None:
        print("no images or no camera_info")
        return 1
    fx = K['fx']

    seps, radii = [], []
    for im in imgs:
        a = find_centre(im, *GUESS[0])
        b = find_centre(im, *GUESS[1])
        if a is None or b is None:
            continue
        seps.append(np.hypot(b[0] - a[0], b[1] - a[1]))
        radii.append(0.5 * (a[2] + b[2]))
    if len(seps) < 5:
        print("only %d frames measured - the outline detector is not locking on"
              % len(seps))
        return 1

    s = np.array(seps)
    sd, sem = s.std(ddof=1), s.std(ddof=1) / np.sqrt(len(s))
    d = fx * BASE_ASSUMED / s.mean()
    sens = fx * BASE_ASSUMED / s.mean() ** 2      # metres per pixel of separation

    print("=" * 72)
    print("  BALL SEPARATION, MEASURED - replacing the eye-read 156.0 px")
    print("=" * 72)
    print("  %d frames, fx %.2f from camera_info\n" % (len(s), fx))
    print("  separation      %.3f px" % s.mean())
    print("  spread          %.3f px  (frame to frame)" % sd)
    print("  standard error  %.4f px      <-- against +/- 1-2 px read by eye" % sem)
    print("  mean radius     %.2f px  (NOT used - it carries a known edge bias)"
          % np.mean(radii))
    print("\n  the ruler's distance, with the assumed %.3f m base:" % BASE_ASSUMED)
    print("    %.4f m  +/- %.1f mm   (from the separation alone)"
          % (d, sem * sens * 1000))
    print("    sensitivity %.1f mm of range per pixel of separation" % (sens * 1000))
    print("\n  WHAT REMAINS UNMEASURED: the %.3f m base itself." % BASE_ASSUMED)
    print("  It comes from Nicolas's code and has never been taped by us.")
    for e in (5, 10, 20):
        print("    a base wrong by %2d mm -> every distance wrong by %.0f mm here"
              % (e, d * e / (BASE_ASSUMED * 1000) * 1000))
    print("\n  So the reference is now limited by the TAPE, not by the reading -")
    print("  which is the right way round, and is one tape measurement from done.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
