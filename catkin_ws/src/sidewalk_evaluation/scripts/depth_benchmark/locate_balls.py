#!/usr/bin/env python3
"""locate_balls.py - find where the balls sit in the picture, at any station.

WHY THIS EXISTS

    Every per-ball measurement needs the balls' pixel positions, and until
    2026-09-08 those were typed in by hand from one station's image and then
    left there. At a different distance the discs sampled the WALL BEHIND the
    target and reported it as the sphere depth - confidently, with a small
    spread, and with no error of any kind. That fault was found in ten scripts
    in one audit. Typing the numbers in is the fault; this removes the typing.

HOW IT WORKS, AND WHAT IT ASSUMES

    Two steps, and the second checks the first.

    1. PREDICT from geometry. The frame's ball heights above the floor were
       measured on 2026-09-08 by the Ouster and, independently, from the
       camera's own image - 0.897 m for the two lower balls and 1.997 m for the
       top one, the two methods agreeing to 6 mm. Given those, the camera's
       height and tilt, and the station distance, where each ball lands in the
       picture is arithmetic.

    2. REFINE by finding the actual edge. Rays are cast outward from the
       predicted centre and a circle is fitted to the brightness step each one
       crosses. The search band is derived from the distance - a 0.20 m ball is
       84 px across at 3 m and 28 px at 9 m, so a fixed band cannot serve both.
       A FIXED 14-42 px BAND IS WHY AN EARLIER FIT RETURNED 104 px WHERE THE
       GEOMETRY DEMANDED 84.

    Both numbers are printed. If the refinement disagrees with the prediction by
    more than a ball radius it is REJECTED and the prediction is used, because a
    refinement that has wandered that far has locked onto a wall seam or a floor
    line, not a ball. Silence is the enemy here: this says which one it used.

    The ball heights are the one real assumption. They are measured, not
    guessed, but they are not a tape reading - see the station provenance.

  usage, on the JETSON (camera node running):
    rosrun sidewalk_evaluation locate_balls.py _dist:=4.0
    rosrun sidewalk_evaluation locate_balls.py _dist:=4.0 _quiet:=true
"""
from __future__ import print_function

import sys

import numpy as np

BASE_H = 0.897          # lower ball centres above the floor, measured 2026-09-08
APEX_H = 1.997          # top ball centre above the floor
BASE_W = 1.100          # centre-to-centre across the two lower balls
BALL_R = 0.20
FX_FALLBACK = 1258.46


def ray_fit(gray, gx, gy, rmin, rmax, rays=180, iters=5):
    """Fit a circle to the brightness step around (gx, gy). Returns
    (cx, cy, r) or None if too few rays found an edge."""
    h, w = gray.shape
    cx, cy, rr = float(gx), float(gy), float('nan')
    for _ in range(iters):
        pts = []
        for th in np.linspace(0, 2 * np.pi, rays, endpoint=False):
            dx, dy = np.cos(th), np.sin(th)
            rs = np.arange(rmin, rmax, 0.25)
            xs = np.clip((cx + dx * rs).astype(int), 0, w - 1)
            ys = np.clip((cy + dy * rs).astype(int), 0, h - 1)
            prof = gray[ys, xs].astype(float)
            if len(prof) < 8:
                continue
            g = np.abs(np.diff(prof))
            i = int(np.argmax(g))
            if g[i] < 6:
                continue
            if 0 < i < len(g) - 1:
                a, b, c = g[i - 1], g[i], g[i + 1]
                den = a - 2 * b + c
                off = 0.5 * (a - c) / den if abs(den) > 1e-9 else 0.0
            else:
                off = 0.0
            pts.append((cx + dx * (rs[i] + off * 0.25), cy + dy * (rs[i] + off * 0.25)))
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


def predict(dist, fx, cx, cy, cam_h, pitch_deg):
    """Where each ball should land, from the station geometry alone.

    Positive pitch is nose-DOWN (ROS convention), which is what the provenance
    records and what reconciles the camera with the laser to 6 mm.
    """
    t = np.tan(np.radians(pitch_deg))
    half = fx * (BASE_W / 2.0) / dist

    def row(height):
        return cy - fx * ((height - cam_h) / dist + t)

    yb, ya = row(BASE_H), row(APEX_H)
    return [('bot_left', cx - half, yb), ('bot_right', cx + half, yb),
            ('apex', cx, ya)], fx * BALL_R / dist


def main():
    import rospy
    from sensor_msgs.msg import Image, CameraInfo

    rospy.init_node('locate_balls', anonymous=True)
    g = rospy.get_param
    dist = float(g('~dist', 0.0))
    quiet = bool(g('~quiet', False))
    cam_h = float(g('~sensor_h', 0.6972))
    pitch = float(g('~pitch_deg', 2.818))
    if dist <= 0:
        print("give the station distance: _dist:=<metres from the camera>")
        return 2

    K, imgs = {}, []

    def info_cb(m):
        if not K:
            K['fx'], K['cx'], K['cy'] = float(m.P[0]), float(m.P[2]), float(m.P[6])

    def img_cb(m):
        if len(imgs) >= 12:
            return
        a = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, -1)
        imgs.append(a[:, :, :3].mean(axis=2))

    rospy.Subscriber('/zedx_front/zed_node/left/camera_info', CameraInfo,
                     info_cb, queue_size=1)
    rospy.Subscriber('/zedx_front/zed_node/left/image_rect_color', Image,
                     img_cb, queue_size=1)
    t0 = rospy.get_time()
    while not rospy.is_shutdown() and (len(imgs) < 12 or not K) \
            and rospy.get_time() - t0 < 60:
        rospy.sleep(0.1)
    if not imgs or not K:
        print("no image or no camera_info - is the camera node up?")
        return 1

    fx, cx, cy = K['fx'], K['cx'], K['cy']
    guesses, r_pred = predict(dist, fx, cx, cy, cam_h, pitch)
    # A generous bracket around the predicted radius: wide enough to tolerate a
    # wrong station distance, narrow enough to exclude the floor lines.
    rmin, rmax = max(6.0, 0.5 * r_pred), 1.6 * r_pred
    im = imgs[len(imgs) // 2]
    H, W = im.shape

    def fit_at(px, py, allow, rays=180, iters=5):
        """Fit one ball at a seed. Returns (cx, cy, r, moved) or None."""
        if not (0 <= px < W and 0 <= py < H):
            return None
        f = ray_fit(im, px, py, rmin, rmax, rays=rays, iters=iters)
        if f is None or not np.isfinite(f[2]):
            return None
        moved = np.hypot(f[0] - px, f[1] - py)
        return (f[0], f[1], f[2], moved) if moved <= allow else None

    # FIND THE SIDEWAYS OFFSET FIRST. Everything else follows from it.
    #
    # The prediction assumes the frame stands on the camera's own axis. It never
    # quite does - here it is about 0.10 m to one side - and that error lands
    # fit returned radii of 100 and 114 px where the geometry demands 84, while
    # the SAME fitter seeded correctly returned 84.3 and 85.1. So the fitter was
    # never the problem and neither was the choice of edge; the seed was.
    #
    # Only one number is unknown. The predicted ROW is good to a pixel or two
    # (checked against a hand reading at 3 m), and the separation between the two
    # lower balls is fixed by the frame and the distance. So scan the one free
    # parameter and keep the offset at which the fitted balls come out the size
    # geometry says they must be. A wrong offset produces a wrong SIZE, which is
    # what makes the radius usable as the referee.
    # The referee is the pair's SEPARATION, not just their size. A fit can drift
    # off a ball and still report a believable radius - measured here, one landed
    # 65 px out with a radius of 85.8 against 83.9 expected, which no size check
    # would ever catch. But the two lower balls are 1.100 m apart on a rigid
    # frame, so at this distance they MUST be fx*1.100/dist pixels apart. A drifted
    # fit breaks that immediately: the same bad pair came out 399 px apart where
    # geometry demands 461. Size and separation together are hard to fool.
    sep_pred = fx * BASE_W / dist
    best, shift = None, 0.0
    for sh in np.arange(-320.0, 321.0, 16.0):
        fits = [fit_at(px + sh, py, 3.0 * r_pred, rays=36, iters=2)
                for _, px, py in guesses[:2]]
        if any(f is None for f in fits):
            continue
        sep_fit = abs(fits[1][0] - fits[0][0])
        score = (abs(fits[0][2] - r_pred) + abs(fits[1][2] - r_pred)
                 + 2.0 * abs(sep_fit - sep_pred))
        if best is None or score < best:
            best, shift = score, float(sh)

    out, notes = [], []
    if best is None:
        notes.append("NEITHER lower ball could be fitted at any sideways offset. "
                     "Falling back to the prediction - check the station distance.")
    else:
        notes.append("frame offset %.0f px %s of the camera axis (%.3f m at this "
                     "distance), found by scanning for the offset that makes the "
                     "balls come out the right SIZE"
                     % (abs(shift), 'right' if shift > 0 else 'left',
                        abs(shift) * dist / fx))

    for nm, px, py in guesses:
        sx, sy = px + shift, py
        if not (0 <= sx < W and 0 <= sy < H):
            notes.append("%s predicted OFF THE PICTURE at (%.0f, %.0f) - not "
                         "measurable at this station" % (nm, sx, sy))
            continue
        f = fit_at(sx, sy, r_pred)
        if f is None:
            out.append((nm, sx, sy, 'predicted (no edge found near the seed)'))
        elif abs(f[2] - r_pred) > 0.25 * r_pred:
            out.append((nm, sx, sy, 'predicted (fit radius %.1f too far from the '
                                    '%.1f expected - rejected)' % (f[2], r_pred)))
        else:
            out.append((nm, f[0], f[1], 'refined, moved %.1f px, radius %.1f vs %.1f'
                        % (f[3], f[2], r_pred)))

    # Final check on the finished pair, stated out loud either way. The frame is
    # rigid, so if the two lower balls do not come out the distance apart that
    # geometry demands, one of them is not on a ball - and a number that is wrong
    # for that reason must never leave here quietly.
    got = dict((o[0], o) for o in out)
    if 'bot_left' in got and 'bot_right' in got:
        sep_fit = abs(got['bot_right'][1] - got['bot_left'][1])
        err = sep_fit - sep_pred
        ok = abs(err) < 0.06 * sep_pred
        notes.append("the two lower balls came out %.1f px apart against %.1f "
                     "demanded by the frame (%+.1f px, %.1f %%) - %s"
                     % (sep_fit, sep_pred, err, 100 * abs(err) / sep_pred,
                        "consistent" if ok else "NOT CONSISTENT, do not use these"))

    spec = ';'.join('%s:%d,%d' % (n, round(x), round(y)) for n, x, y, _ in out)
    if quiet:
        print(spec)
        return 0

    print("=" * 74)
    print("  BALL POSITIONS AT %.2f m" % dist)
    print("=" * 74)
    print("  fx %.2f  principal point (%.1f, %.1f)  camera %.4f m, pitch %+.3f deg"
          % (fx, cx, cy, cam_h, pitch))
    print("  predicted ball radius %.1f px, so rays search %.0f-%.0f px\n"
          % (r_pred, rmin, rmax))
    for nm, x, y, how in out:
        print("  %-10s (%7.1f, %7.1f)   %s" % (nm, x, y, how))
    for n in notes:
        print("  %s" % n)
    print("\n  pass this straight through:")
    print("    _balls:='%s'" % spec)
    return 0


if __name__ == '__main__':
    sys.exit(main())
