#!/usr/bin/env python3
"""outline_frame_fit.py - measure the ball frame from the PICTURE, with no
stereo depth at all.

WHY

    Measured on 2026-09-07 at the 9 m station, against the Helios laser:

        the camera's picture      -70 to -110 mm
        the camera's stereo depth -350 to -430 mm

    The picture looks several times closer to the laser than the depth does.
    **That comparison is not yet safe**, and the reason is below: the picture's
    own reference was read by eye, at 56.5 mm of range per pixel, so its
    uncertainty is the same size as the difference being claimed. Treat the gap
    as a lead, not a result, until the reference is calibrated.

    The idea is still worth having: a sphere of KNOWN radius at an unknown
    distance has a known apparent size, and three spheres in a KNOWN rigid
    triangle over-determine the whole 6-unknown pose. That is a measurement the
    lens can make without stereo correspondence at all - which matters, because
    correspondence on a smooth sphere is exactly what the metrology literature
    says passive stereo cannot do.

    This is Arm C - camera-native, a method a LiDAR cannot offer. It uses
    knowledge of the target (0.20 m radius, 1.10 m base, 1.10 m rise), which
    must be declared every time it is reported, and it can never share a table
    with Nicolas's frozen-detector numbers.

NOT INDEPENDENT OF THE IMAGE RULER — READ THIS BEFORE CITING ANY NUMBER

    This method must never be quoted as confirming the "image ruler" (the
    8.874 m from the balls being 156.0 px apart and 1.100 m apart in the world).
    But the reason is narrower than a first draft of this note claimed, and an
    independent checker corrected it — twice. What follows is the corrected
    version; the errors are listed at the end so nobody repeats them.

    **The one genuine shared assumption is the 1.100 m base.** Both this fit
    (`BASE`, used by `body_frame()`) and the ruler scale in direct proportion to
    it, with the same sign and no cancellation. If the frame is not 1.100 m
    across, both are wrong together and neither can reveal it.

    Two things that are NOT the problem, contrary to the first draft:

      - **The hand-read seeds do not propagate.** `detect_balls()` uses
        `guesses` only as a ray-casting origin and then re-solves each centre by
        circle fit over detected edges. The seed sets the basin of attraction,
        not the answer. (The seeds do share provenance with the ruler —
        1058 − 902 = 156.0 px exactly — so the two are not independently derived,
        but a few pixels of seed error does not reach the result.)
      - **The focal length is read from `camera_info` at runtime**, not taken
        from the ruler. And fx cancels in any ratio between the two channels
        anyway, since both scale with it.

    ONE CHANNEL IS GENUINELY INDEPENDENT, AND IT IS THE WEAKER ONE

    The ruler uses only the SEPARATION between balls. This fit also uses their
    measured RADII. So the two can be compared — carefully, because they are not
    the same quantity: `fx*R/r` gives the RANGE to a ball, while the ruler and
    `params[0]` give the FORWARD distance. Solved properly with this file's own
    exact model, at the reported pose:

        separation channel                        8.874 m
        radius channel                            8.768 m
        this fit reported                         8.825 m

        disagreement  106 +/- 86 mm  =  1.2 sigma  -> NOT SIGNIFICANT

    The uncertainty comes from the radii themselves: sd 0.48 px over three balls,
    so 0.28 px on their mean, at 309 mm of range per pixel. **Do not cite the
    channel disagreement as a finding.** It is consistent with zero.

    WHICH CHANNEL TO BELIEVE: THE SEPARATION ONE

    The radius channel has a known systematic the separation channel does not.
    At the ruler's distance the exact model predicts radii of 28.45 / 28.42 /
    28.63 px (mean 28.50). The detector measured 28.85 — an excess of +0.35 px,
    which is what a threshold-based edge does on a soft silhouette, and it biases
    the radius channel NEAR.

    Note also that `residuals()` weights the radius term x3, so the reported
    answer is deliberately pulled towards the channel with the known bias. That
    weighting was chosen to make the radii carry the distance; given the bias
    above it should be revisited.

    WHAT THE CHECKER CAUGHT IN THE EARLIER DRAFTS OF THIS NOTE

      1. "the seed error appears in both and cancels" — false; the seed is
         refined away. The base is the real shared term.
      2. "the radii should read 28.37 px" — that is the ON-AXIS value. Off axis
         the exact model gives 28.50, and for the apex the exact-vs-naive gap is
         +0.78 px — precisely the error this file's next section warns about. The
         note committed the error it documents.
      3. "excess of 0.48 px" — it is 0.35 px against the correct prediction.
      4. "the two channels disagree by 147 mm" — that compared a RANGE against a
         FORWARD distance. Done properly it is 106 mm.
      5. The disagreement was quoted bare, with no uncertainty. It is 1.2 sigma.
      6. The radii quoted are from the LAST FRAME ONLY (n=1), not the 12-frame
         run — `main()` prints `obs[:,2]` after the loop.

THE PROJECTION, DONE PROPERLY

    A sphere does NOT project to a circle centred on the projection of its
    centre. Off the optical axis it projects to an ELLIPSE whose centre is
    pushed outwards. An earlier silhouette detector in this project assumed a
    circle and was found, before it ever ran, to carry ~0.7 px of error at the
    apex ball - about 70 mm of range at 9 m, which is the size of the whole
    effect being chased.

    So the outline is generated exactly. The points where the viewing rays
    graze the sphere form a circle - not a great circle through the centre, but
    a smaller one nearer the camera:

        centre  C * (1 - R^2/d^2)          radius  R * sqrt(1 - R^2/d^2)

    in the plane perpendicular to C, where d = |C|. Projecting points around
    that circle gives the true outline with no small-angle approximation.

  usage, on the JETSON (camera node running):
    rosrun sidewalk_evaluation outline_frame_fit.py _frames:=20
"""
from __future__ import print_function

import sys

import numpy as np
from scipy.optimize import least_squares

BALL_R = 0.20
BASE = 1.10
RISE = 1.10

# NOTE: the intrinsics are read from camera_info at runtime (see info_cb) and
# these constants are NOT used anywhere. They are kept only as a record of the
# values seen on this camera, and deliberately not wired in - a stale hard-coded
# focal length silently corrupts every distance this file produces.


def body_frame():
    b = np.array([[0.0, +BASE / 2.0, 0.0],
                  [0.0, -BASE / 2.0, 0.0],
                  [0.0, 0.0, RISE]])
    return b - b.mean(axis=0)


def rot(rv):
    th = np.linalg.norm(rv)
    if th < 1e-12:
        return np.eye(3)
    k = rv / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * (K @ K)


def outline_points(C, fx, fy, cx, cy, n=48):
    """The exact image outline of a sphere of radius BALL_R centred at C.

    C is in the ROS convention used everywhere else here: x forward, y left,
    z up. The pinhole model wants x right, y down, z forward, so the axes are
    swapped once, here, rather than scattered through the file.
    """
    d = np.linalg.norm(C)
    if d <= BALL_R * 1.01:
        return None
    k = 1.0 - (BALL_R ** 2) / (d ** 2)
    Cs = C * k                                  # centre of the grazing circle
    Rs = BALL_R * np.sqrt(k)                    # its radius
    a = C / d
    tmp = np.array([0.0, 0.0, 1.0]) if abs(a[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    u = np.cross(a, tmp)
    u /= np.linalg.norm(u)
    v = np.cross(a, u)
    t = np.linspace(0, 2 * np.pi, n, endpoint=False)
    P = Cs[None, :] + Rs * (np.cos(t)[:, None] * u[None, :] +
                            np.sin(t)[:, None] * v[None, :])
    Xc, Yc, Zc = -P[:, 1], -P[:, 2], P[:, 0]    # ROS -> pinhole
    ok = Zc > 0.05
    if not ok.all():
        return None
    return np.column_stack([fx * Xc / Zc + cx, fy * Yc / Zc + cy])


def ellipse_of(C, fx, fy, cx, cy):
    """The outline's centre and mean radius in pixels - what a detector sees."""
    P = outline_points(C, fx, fy, cx, cy)
    if P is None:
        return None
    m = P.mean(axis=0)
    return m[0], m[1], float(np.linalg.norm(P - m, axis=1).mean())


def predict(params, K):
    fx, fy, cx, cy = K
    cs = body_frame() @ rot(params[3:6]).T + params[:3]
    out = []
    for C in cs:
        e = ellipse_of(C, fx, fy, cx, cy)
        if e is None:
            return None
        out.append(e)
    return np.array(out)


def residuals(params, obs, K, order):
    pred = predict(params, K)
    if pred is None:
        return np.full(obs.size, 1e3)
    p = pred[order]
    # Centres in pixels, and apparent radius in pixels. The radius is what
    # carries the distance, so it is weighted to matter as much as the centres.
    return np.concatenate([(p[:, 0] - obs[:, 0]),
                           (p[:, 1] - obs[:, 1]),
                           3.0 * (p[:, 2] - obs[:, 2])])


def fit_pose(obs, K, seed_dist=9.0):
    """obs: 3x3 array of (cx_px, cy_px, r_px), rows in ANY order."""
    import itertools
    best = None
    for order in itertools.permutations(range(3)):
        for zguess in (seed_dist - 1.0, seed_dist, seed_dist + 1.0):
            p0 = np.array([zguess, 0.0, 0.2, 0.0, 0.0, 0.0])
            try:
                r = least_squares(residuals, p0, args=(obs, K, list(order)),
                                  loss='soft_l1', f_scale=2.0, max_nfev=300)
            except Exception:
                continue
            cost = float(np.sum(r.fun ** 2))
            if best is None or cost < best[0]:
                best = (cost, r.x, list(order))
    return best


def detect_balls(gray, guesses, search=26, rmin=14, rmax=42):
    """Find each ball's outline centre and radius by casting rays outward.

    The balls are half blue and half white against a white wall, so a colour
    threshold finds half a ball and a brightness threshold finds a different
    half. Casting rays from an approximate centre and taking the strongest
    brightness STEP along each ray finds the physical edge either way, and
    fitting a circle to those edge points is insensitive to which half is dark.
    """
    out = []
    h, w = gray.shape
    for (gx, gy) in guesses:
        cx, cy = float(gx), float(gy)
        for _ in range(4):
            pts = []
            for th in np.linspace(0, 2 * np.pi, 72, endpoint=False):
                dx, dy = np.cos(th), np.sin(th)
                rs = np.arange(rmin, rmax, 0.5)
                xs = np.clip((cx + dx * rs).astype(int), 0, w - 1)
                ys = np.clip((cy + dy * rs).astype(int), 0, h - 1)
                prof = gray[ys, xs]
                if len(prof) < 6:
                    continue
                g = np.abs(np.diff(prof))
                i = int(np.argmax(g))
                if g[i] < 6:            # no real edge along this ray
                    continue
                pts.append((cx + dx * rs[i], cy + dy * rs[i]))
            if len(pts) < 18:
                break
            P = np.array(pts)
            # Algebraic circle fit, then drop the worst quarter and refit, so a
            # few rays that found the colour boundary instead of the outline
            # cannot drag the answer.
            for _ in range(2):
                A = np.column_stack([2 * P[:, 0], 2 * P[:, 1], np.ones(len(P))])
                b = (P ** 2).sum(axis=1)
                sol, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
                cx, cy = sol[0], sol[1]
                rr = np.sqrt(max(sol[2] + cx * cx + cy * cy, 1e-6))
                d = np.abs(np.linalg.norm(P - np.array([cx, cy]), axis=1) - rr)
                keep = d < np.percentile(d, 75)
                if keep.sum() >= 12:
                    P = P[keep]
        out.append((cx, cy, rr))
    return np.array(out)


def main():
    import rospy
    from sensor_msgs.msg import Image, CameraInfo

    rospy.init_node('outline_frame_fit', anonymous=True)
    g = rospy.get_param
    want = int(g('~frames', 20))
    seed = float(g('~seed_dist', 8.9))
    truth = float(g('~truth', 8.873))
    guesses = [(902, 503), (1058, 503), (963, 340)]

    K = {'v': None}
    imgs = []

    def info_cb(m):
        if K['v'] is None:
            K['v'] = (m.P[0], m.P[5], m.P[2], m.P[6])

    def img_cb(m):
        if len(imgs) >= want:
            return
        a = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, -1)
        imgs.append(a[:, :, :3].mean(axis=2).astype(float))

    rospy.Subscriber('/zedx_front/zed_node/left/camera_info', CameraInfo,
                     info_cb, queue_size=1)
    rospy.Subscriber('/zedx_front/zed_node/left/image_rect_color', Image,
                     img_cb, queue_size=1)
    t0 = rospy.get_time()
    while not rospy.is_shutdown() and (len(imgs) < want or K['v'] is None) \
            and rospy.get_time() - t0 < 120:
        rospy.sleep(0.1)
    if not imgs or K['v'] is None:
        print("no images or no camera_info")
        return 1
    Kv = K['v']
    print("=" * 76)
    print("  OUTLINE FIT - the frame measured from the picture, no stereo depth")
    print("=" * 76)
    print("  fx %.2f  fy %.2f  cx %.2f  cy %.2f" % Kv)
    print("  %d frames; the laser says the frame is at about 8.94 m\n" % len(imgs))

    ds, poses = [], []
    for im in imgs:
        obs = detect_balls(im, guesses)
        if len(obs) < 3 or not np.isfinite(obs).all():
            continue
        b = fit_pose(obs, Kv, seed)
        if b is None:
            continue
        poses.append(b[1])
        ds.append(b[1][0])
    if len(ds) < 3:
        print("  only %d frames fitted - the outline detector needs work here" % len(ds))
        return 1
    ds = np.array(ds)
    P = np.array(poses)
    print("  radii found (px, last frame): %s" % np.round(obs[:, 2], 2))
    print("  distance   mean %.4f m   spread %.1f mm   over %d frames"
          % (ds.mean(), ds.std(ddof=1) * 1000, len(ds)))
    print("  error vs the picture ruler (%.3f m): %+.0f mm"
          % (truth, (ds.mean() - truth) * 1000))
    print("  lateral %+.4f m   height %+.4f m"
          % (P[:, 1].mean(), P[:, 2].mean()))
    return 0


if __name__ == '__main__':
    sys.exit(main())
