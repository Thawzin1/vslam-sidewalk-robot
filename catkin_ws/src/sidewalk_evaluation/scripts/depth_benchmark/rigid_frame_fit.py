#!/usr/bin/env python3
"""rigid_frame_fit.py - fit the WHOLE three-ball frame as one rigid object,
over many frames at once.

WHY THIS EXISTS

    Nicolas's detector treats each ball separately: find three ball-shaped
    things, then check afterwards whether they happen to be the right distances
    apart. Every ball is on its own, so a badly-seen one drags the answer down
    and there is nothing to catch it.

    But the frame is RIGID. Its three balls are 1.10 m apart along the base and
    1.10 m up to the apex, and each is 0.20 m in radius. That is not a check to
    apply at the end - it is knowledge that should be used from the start.

    Fitting the frame as one body has 6 unknowns (where it is, and which way it
    faces) instead of 9 (three free centres). Three of the nine were never free,
    so the extra three are pure noise being fitted. Removing them means a badly
    seen ball leans on the other two instead of pulling them off.

    And the target NEVER MOVES. So frames can be accumulated before fitting,
    which matters because of what was measured on 2026-09-07 at 9 m:

        QUALITY  depth accurate to  +44 mm, but its two balls disagreed by
                 217 mm frame to frame, so the frozen detector accepted 0 of 45
        ULTRA    depth off by -396 mm, but internally consistent to 26 mm

    QUALITY's fault is inconsistency, not bias - and inconsistency is exactly
    what averaging removes. So the mode that could not be used at all may become
    the best one once frames are pooled.

HONESTY

    This is a DIFFERENT MEASUREMENT from the depth study's, and must never be put in
    the same table as Nicolas's numbers without both being labelled. His metric
    is frame-to-frame repeatability of a single-frame estimator. Pooling frames
    beats that by construction, and claiming the improvement as a like-for-like
    win would be dishonest.

    So precision here is reported the only fair way: the accumulated frames are
    split into independent BLOCKS, each block fitted on its own, and the spread
    of those block answers is the precision. A block is then comparable to one
    of his samples, just built from more data.

    The starting guess comes from the detector's own answer or from the image
    ruler. A starting guess is not an answer: the fit is driven by the points,
    and it is free to walk away from the seed. The report prints how far it
    moved, so a fit that merely sat on its seed is visible as one.

  usage, on the JETSON (camera node running):
    rosrun sidewalk_evaluation rigid_frame_fit.py _frames:=120 _blocks:=6
"""
from __future__ import print_function

import sys

import numpy as np
from scipy.optimize import least_squares

# ROS is imported inside main() on purpose: the geometry and the fitter below
# are plain numpy, and keeping them importable without a sourced ROS
# environment is what lets them be tested against synthetic data with known
# truth - which is how this file was checked before it was ever pointed at the
# camera.

_DT = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}

BALL_R = 0.20
BASE = 1.10          # the two bottom balls, apart
RISE = 1.10          # apex above the midpoint of the bottom pair
GATE = 0.45          # a point further than this from every ball takes no part


def body_frame():
    """The three ball centres in the frame's OWN coordinates, origin at their
    centroid. y is across the base, z is up. This is the geometry that is known
    in advance and never fitted."""
    b = np.array([[0.0, +BASE / 2.0, 0.0],
                  [0.0, -BASE / 2.0, 0.0],
                  [0.0, 0.0, RISE]])
    return b - b.mean(axis=0)


def rot(rv):
    """Rodrigues: a 3-vector whose direction is the axis and length the angle."""
    th = np.linalg.norm(rv)
    if th < 1e-12:
        return np.eye(3)
    k = rv / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * (K @ K)


def centres(params):
    t, rv = params[:3], params[3:6]
    return body_frame() @ rot(rv).T + t


def residuals(params, pts):
    """How far each point is from the nearest ball's SURFACE.

    Assignment is to the nearest ball and is recomputed every iteration, so the
    fit is free to reinterpret which ball a point belongs to as it moves.
    """
    c = centres(params)
    d = np.linalg.norm(pts[:, None, :] - c[None, :, :], axis=2)   # n x 3
    near = d.min(axis=1)
    return near - BALL_R


def _fit_once(pts, seed_t, seed_rv, schedule):
    """One descent, tightening the robust scale and the gate as it goes.

    Starting wide and narrowing (a graduated fit) stops a sharp robust loss from
    locking onto whatever happens to be near the first guess.
    """
    p0 = np.concatenate([np.asarray(seed_t, dtype=float),
                         np.asarray(seed_rv, dtype=float)])
    use = None
    for gate, fscale in schedule:
        c = centres(p0)
        d = np.linalg.norm(pts[:, None, :] - c[None, :, :], axis=2).min(axis=1)
        use = pts[d < gate]
        if len(use) < 60:
            return None
        r = least_squares(residuals, p0, args=(use,), loss='soft_l1',
                          f_scale=fscale, max_nfev=120,
                          xtol=1e-8, ftol=1e-8, gtol=1e-8)
        p0 = r.x
    c = centres(p0)
    d = np.linalg.norm(pts[:, None, :] - c[None, :, :], axis=2).min(axis=1)
    on = np.abs(d - BALL_R) < 0.03           # genuinely on a ball surface
    n_on = int(on.sum())
    rms = float(np.sqrt(np.mean((d[on] - BALL_R) ** 2)) * 1000) if n_on else float('nan')
    return p0, n_on, rms


# A surface fit does not need every pixel. Measured on the real cloud: 9,284
# points per frame with an 11-start sweep took 85 SECONDS PER FRAME, which is
# an hour per station and unworkable across a sweep. Neighbouring pixels on a
# ball are not independent measurements, so a few thousand carry the same
# surface at a fraction of the cost.
MAX_PTS = 3500


def fit(pts, seed_t, seed_rv=None, gate=GATE, span=0.30, step=0.15, rng=None):
    """Rigid fit that does NOT depend on where it was started.

    The default sweep is +/- 0.30 m because in practice the seed comes from the
    image ruler, which is good to about 50 mm - a wider sweep buys nothing and
    costs linearly. Pass a bigger span when the seed is genuinely uncertain.

    A single descent from one guess is not safe here. Tested on synthetic data
    with 30 % impostor points in front of the balls - the shape the camera
    actually delivers - a fit seeded 300 mm off converged onto the impostors and
    landed 273 mm wrong, while the same fit seeded at truth was right to 9 mm.

    So the seed is swept across +/- span metres in depth and every result is
    scored by HOW MANY POINTS ACTUALLY LAND ON A BALL SURFACE (within 30 mm).
    That number separates the cases cleanly - 1200 for the true pose against 691
    for the impostor trap - because the impostor cloud has no sphere in it and
    cannot make one.

    Returns (params, n_on_surface, rms_mm) or None.
    """
    # Subsample before fitting. A 20-frame pooled block holds ~134,000 points
    # and a multi-start sweep over that took over 22 minutes for one block,
    # which is time spent for nothing: adjacent pixels on a ball are not
    # independent measurements, so 20,000 carries the same surface.
    if len(pts) > MAX_PTS:
        r = rng if rng is not None else np.random.default_rng(0)
        pts = pts[r.choice(len(pts), MAX_PTS, replace=False)]
    schedule = [(max(gate, 0.60), 0.12), (gate, 0.06), (gate * 0.8, 0.03)]
    base_rv = np.asarray(seed_rv if seed_rv is not None else [0, 0, 0], dtype=float)
    best = None
    offs = np.arange(-span, span + 1e-9, step)
    for off in offs:
        st = np.asarray(seed_t, dtype=float) + np.array([off, 0.0, 0.0])
        out = _fit_once(pts, st, base_rv, schedule)
        if out is None:
            continue
        if best is None or out[1] > best[1]:
            best = out
    return best


def decode(msg):
    names = {f.name: f for f in msg.fields}
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
    out = []
    for k in ('x', 'y', 'z'):
        f = names[k]
        col = raw[:, f.offset:f.offset + np.dtype(_DT[f.datatype]).itemsize]
        out.append(np.ascontiguousarray(col).view(_DT[f.datatype]).ravel())
    xyz = np.stack(out, axis=1).astype(np.float64)
    return xyz[np.isfinite(xyz).all(axis=1)]


def main():
    import rospy
    from sensor_msgs.msg import PointCloud2

    rospy.init_node('rigid_frame_fit', anonymous=True)
    g = rospy.get_param
    topic = g('~cloud_topic', '/zedx_front/zed_node/point_cloud/cloud_registered')
    want = int(g('~frames', 120))
    nblocks = int(g('~blocks', 6))
    roi_x = float(g('~roi_x', 9.0))
    roi_sx = float(g('~roi_sx', 1.6))
    sensor_h = float(g('~sensor_h', 0.6972))
    roi_z = float(g('~roi_z', 1.45))
    seed_x = float(g('~seed_x', 0.0))            # 0 = use the box centre
    truth = float(g('~truth', 8.873))            # the image ruler's answer
    label = g('~label', 'run')

    lo = np.array([roi_x - roi_sx / 2, -0.95, roi_z - 0.9 - sensor_h])
    hi = np.array([roi_x + roi_sx / 2, 0.95, roi_z + 0.9 - sensor_h])

    frames = []

    def cb(msg):
        if len(frames) >= want:
            return
        xyz = decode(msg)
        m = np.all((xyz >= lo) & (xyz <= hi), axis=1)
        if m.sum() > 200:
            p = xyz[m]
            frames.append(p[::2] if len(p) > 6000 else p)

    rospy.Subscriber(topic, PointCloud2, cb, queue_size=1, buff_size=2 ** 24)
    print("collecting %d frames from %s ..." % (want, topic))
    t0 = rospy.get_time()
    while not rospy.is_shutdown() and len(frames) < want and rospy.get_time() - t0 < 400:
        rospy.sleep(0.1)
    if len(frames) < 12:
        print("only %d frames - is the camera publishing?" % len(frames))
        return 1
    print("got %d frames, %d points each on average\n"
          % (len(frames), int(np.mean([len(f) for f in frames]))))

    seed = np.array([seed_x if seed_x else roi_x, 0.0,
                     roi_z - sensor_h])

    print("=" * 78)
    print("  RIGID FRAME FIT - one 6-unknown body, not three free spheres")
    print("=" * 78)
    print("  seed %.3f m (a starting guess, not an answer)" % seed[0])
    print("  the image ruler independently says %.3f m\n" % truth)

    # 1. every frame on its own - comparable in spirit to the depth study's metric
    per = []
    for i, f in enumerate(frames):
        r = fit(f, seed)
        if r is not None:
            per.append(centres(r[0]).mean(axis=0))
        if (i + 1) % 20 == 0:
            print("    ... %d of %d frames fitted" % (i + 1, len(frames)))
            sys.stdout.flush()
    if per:
        per = np.array(per)
        s = per.std(axis=0, ddof=1) * 1000
        print("  ONE FRAME AT A TIME   n=%d of %d fitted" % (len(per), len(frames)))
        print("    centroid  x %.4f  y %+.4f  z %+.4f" % tuple(per.mean(axis=0)))
        print("    wobble    %.1f / %.1f / %.1f mm   3D %.1f mm"
              % (s[0], s[1], s[2], np.sqrt((s ** 2).sum())))
        print("    distance error vs the image ruler: %+.0f mm\n"
              % ((per[:, 0].mean() - truth) * 1000))

    # 2. pooled into blocks - each block is one independent answer built from
    #    more data, so the spread across blocks is an honest precision figure
    per_block = max(2, len(frames) // nblocks)
    blocks = [np.vstack(frames[i:i + per_block])
              for i in range(0, len(frames) - per_block + 1, per_block)]
    res = []
    for bi, b in enumerate(blocks):
        print("    ... block %d of %d (%d points)" % (bi + 1, len(blocks), len(b)))
        sys.stdout.flush()
        r = fit(b, seed)
        if r is None:
            continue
        p, ninl, rms = r
        res.append((centres(p).mean(axis=0), ninl, rms))
    if not res:
        print("  no block fitted - the seed may be far from the frame")
        return 1
    C = np.array([r[0] for r in res])
    s = C.std(axis=0, ddof=1) * 1000 if len(C) > 1 else np.zeros(3)
    print("  POOLED, %d blocks of %d frames each" % (len(res), per_block))
    print("    centroid  x %.4f  y %+.4f  z %+.4f" % tuple(C.mean(axis=0)))
    print("    wobble    %.1f / %.1f / %.1f mm   3D %.1f mm"
          % (s[0], s[1], s[2], np.sqrt((s ** 2).sum())))
    print("    surface fit residual %.1f mm on %d points"
          % (np.mean([r[2] for r in res]), int(np.mean([r[1] for r in res]))))
    err = (C[:, 0].mean() - truth) * 1000
    print("    distance error vs the image ruler: %+.0f mm" % err)
    print("    the fit moved %+.0f mm from its seed"
          % ((C[:, 0].mean() - seed[0]) * 1000))
    print("\n  [%s] pooled %d-frame blocks: %+.0f mm from truth, %.1f mm wobble"
          % (label, per_block, err, np.sqrt((s ** 2).sum())))

    # 3. how far does pooling actually get you? Averaging only helps against
    #    RANDOM error, so the curve flattening tells you how much of what is
    #    left is a fixed mistake that no amount of data will remove.
    if int(g('~curve', 1)):
        print("\n  HOW FAR DOES POOLING GET YOU?")
        print("    a flat curve means the rest of the error is fixed, not noise\n")
        print("    %10s %8s %12s %12s"
              % ("block size", "blocks", "wobble mm", "distance mm"))
        print("    " + "-" * 46)
        for bs in (1, 2, 5, 10, 20, 40, 80):
            if bs > len(frames) // 2:
                break
            bl = [np.vstack(frames[i:i + bs])
                  for i in range(0, len(frames) - bs + 1, bs)]
            cs = []
            for b in bl:
                r = fit(b, seed)
                if r is not None:
                    cs.append(centres(r[0]).mean(axis=0))
            if len(cs) < 2:
                continue
            C2 = np.array(cs)
            s2 = C2.std(axis=0, ddof=1) * 1000
            print("    %10d %8d %12.1f %+12.0f"
                  % (bs, len(cs), np.sqrt((s2 ** 2).sum()),
                     (C2[:, 0].mean() - truth) * 1000))
    return 0


if __name__ == '__main__':
    sys.exit(main())
