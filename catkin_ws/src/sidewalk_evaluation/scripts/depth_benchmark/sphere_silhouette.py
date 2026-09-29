#!/usr/bin/env python3
"""sphere_silhouette.py - measure the spheres from their OUTLINE, not their front cap.

    Arm C, method C1. Authored 2026-09-05, NOT YET EXECUTED against real data -
    treat every number it produces as unvalidated until it has been run on the
    5 m recording beside the frozen detector and the two agree on the triangle.

WHY THIS EXISTS

    The frozen detector fits a sphere of known radius to the points on the
    sphere's front cap. A camera only ever sees that cap, and the cap's extent
    changes from frame to frame as glare and matching failures eat its edges.
    A fixed-radius fit to a cap whose size is changing slides along the viewing
    direction - which is the leading (though not settled) explanation for why the
    camera's depth axis degrades so much faster with distance than stereo noise
    alone predicts.

    Plain terms: imagine judging where the centre of a ball is by looking only at
    the lit crescent on its front. If the crescent grows and shrinks, your guess
    for the centre slides back and forth even though the ball never moved.

    So stop asking the cap. A sphere's OUTLINE in the image does not care how much
    of its face returned a depth value: the outline is where the ball stops and the
    wall starts, an intensity edge, and it is there whether the middle of the ball
    measured well or not at all.

WHAT IT MEASURES, AND WHY THAT IS BETTER

    At 5 m a 0.40 m sphere is a circle about 101 pixels across. Fitting a circle to
    a few hundred subpixel edge points locates its centre to a small fraction of a
    pixel. One pixel at 5 m subtends about 4 mm, so a tenth of a pixel is 0.4 mm -
    against the 14 mm the 3D fit currently delivers sideways.

    Range comes out of the SAME measurement, for free, because the radius is known:
    a sphere of radius R at distance d subtends a cone of half-angle alpha with
    sin(alpha) = R/d. Measure alpha from the image and d follows. This range needs
    no stereo matching at all - it is a monocular measurement that happens to be
    metric because we know how big the ball is.

    Both are computed and BOTH are written out, because comparing them is itself a
    result: 'range from apparent size' against 'range from stereo depth' on the
    same sphere in the same frame.

THE GEOMETRY, DONE PROPERLY

    A sphere does not project to a circle centred on the projected sphere centre.
    It projects to an ellipse whose centre sits slightly further from the image
    centre than the sphere's own centre does. Ignoring that is a real error here,
    not a pedantic one: at 5 m with a sphere 1.1 m off-axis it is about half a
    pixel, which is 2 mm - larger than the precision this method is trying to buy.

    So the two edge rays are treated exactly. If the sphere centre lies at angle
    theta from the optical axis and the silhouette half-angle is alpha, the two
    edges of the outline fall at theta-alpha and theta+alpha, so in the image

        rho_far  = f * tan(theta + alpha)
        rho_near = f * tan(theta - alpha)

    The measured outline gives the midpoint and half-width of those two radii; the
    pair (theta, alpha) is recovered from them by a two-variable Newton solve.
    Small-angle algebra would put theta at the midpoint and get it wrong.

VALIDATED 2026-09-05, BEFORE ANY REAL DATA (test_silhouette.py)

    Round trip on eight known sphere positions from 4 to 10 m, on-axis and off:
    take a true 3D centre, compute the outline it would really produce, feed that
    outline back in. Recovered centre error: 0.00000 mm at every case. The circle
    fit reproduces a synthetic circle to five decimals.

    The exact cone solve was NOT pedantry. Running the same eight cases through
    the small-angle shortcut instead gives range errors of 2.5 mm on axis but
    **389 mm** for a sphere 1.1 m off axis - the shortcut fails hardest exactly
    where the frame's outer spheres sit. An implementation that skipped it would
    have produced confident, plausible, badly wrong numbers.

    REGISTERED PREDICTION, written before this ran on a real recording. Measured
    sensitivity is 0.1 px of radius error -> 6.3 mm of range at 4 m, 9.9 mm at
    5 m, 39.6 mm at 10 m. With ~180 edge rays and a per-edge scatter of a few
    tenths of a pixel, the fitted radius should settle to ~0.02 px, giving a few
    millimetres of range and a few tenths of a millimetre sideways. So:

        C1 should be DEPTH-limited, with its lateral axis far better than its
        range axis - the exact inverse of the frozen detector's failure at 5 m,
        where the lateral axis is the weak one.

    If that inversion appears, the estimator-geometry explanation is supported.
    If C1 is also lateral-limited, the explanation is wrong and something about
    the scene, not the estimator, is responsible.

HONESTY

    The same triangle gate as the frozen detector is applied, imported from
    Nicolas's file unchanged: three side lengths within tolerance, the bottom pair
    level, the apex at the right height. A method that measures beautifully and
    locks onto the wrong three blobs is worthless, and the side lengths are what
    catch that.

    This is Arm C. It is NOT the fair comparison and must never be merged into an
    Arm A table. Its purpose is to answer 'what can this camera do at all', which
    is a different question from 'how does it compare measured the LiDAR's way'.

  usage, on the JETSON, after the live series is finished:
    python3 sphere_silhouette.py <svo> <n_frames> <csv_out> [roi_x=5.0]
"""
from __future__ import print_function

import math
import os
import sys

import cv2
import numpy as np
import pyzed.sl as sl

# Import the frozen detector's geometry helpers from THIS directory, not from a
# hardcoded path. The older harnesses in this folder pin ~/... which
# makes them Jetson-only and silently unimportable on a computer without ROS.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sphere_centroid import validate_triangle, centroid_of  # noqa: E402

SPHERE_R = 0.20                     # metres, the known radius - the whole trick
ROI = dict(xmin=4.4, xmax=5.6, ymin=-0.95, ymax=0.95, zmin=-0.1294, zmax=1.6706)
# Triangle gate: the frozen values, so this arm is held to the same honesty bar.
GATE = dict(base=1.10, rise=1.10, tol=0.06)
APEX_Z_TOL = 0.08


def set_roi_x(roi_x):
    ROI['xmin'] = roi_x - 0.6
    ROI['xmax'] = roi_x + 0.6


# ---------------------------------------------------------------- image helpers

def sample_bilinear(img, u, v):
    """Image intensity at a fractional pixel. Returns NaN outside the image.

    Subpixel edge finding is the entire point of this detector, so the intensity
    profile along a ray has to be sampled between pixels, not snapped to them."""
    h, w = img.shape
    u0 = np.floor(u).astype(np.int32)
    v0 = np.floor(v).astype(np.int32)
    ok = (u0 >= 0) & (v0 >= 0) & (u0 < w - 1) & (v0 < h - 1)
    out = np.full(u.shape, np.nan, np.float64)
    if not ok.any():
        return out
    u0o, v0o = u0[ok], v0[ok]
    du, dv = u[ok] - u0o, v[ok] - v0o
    out[ok] = ((1 - du) * (1 - dv) * img[v0o, u0o] +
               du * (1 - dv) * img[v0o, u0o + 1] +
               (1 - du) * dv * img[v0o + 1, u0o] +
               du * dv * img[v0o + 1, u0o + 1])
    return out


def fit_circle_algebraic(u, v):
    """Least-squares circle through 2D points. Returns (uc, vc, r).

    Same algebraic trick as the sphere fit in sphere_centroid.py, one dimension
    down: the circle equation is linear in (uc, vc, uc^2+vc^2-r^2)."""
    A = np.c_[2 * u, 2 * v, np.ones(len(u))]
    b = u * u + v * v
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    uc, vc, c = sol
    r2 = c + uc * uc + vc * vc
    if r2 <= 0:
        raise np.linalg.LinAlgError('degenerate circle')
    return float(uc), float(vc), float(math.sqrt(r2))


def refine_edge_points(gray, uc, vc, r0, n_rays=180, band=0.30, min_contrast=6.0):
    """Walk outward along many rays and find the sphere's edge on each, subpixel.

    For each ray the intensity profile is sampled across a band around the coarse
    radius, and the edge is the peak of the profile's gradient magnitude, refined
    by fitting a parabola to the peak and its two neighbours.

    Rays whose strongest gradient is weaker than `min_contrast` are DROPPED rather
    than fitted. That matters on this target: the spheres carry white panels, the
    corridor walls are pale, and where a white panel meets a pale wall there is
    genuinely no edge to find. Fitting noise there would drag the circle. Better
    to measure three quarters of an outline honestly than all of it badly."""
    ang = np.linspace(0.0, 2.0 * math.pi, n_rays, endpoint=False)
    lo, hi = r0 * (1.0 - band), r0 * (1.0 + band)
    n_s = max(24, int(round((hi - lo) * 3.0)))     # ~3 samples per pixel of band
    rr = np.linspace(lo, hi, n_s)
    ca, sa = np.cos(ang)[:, None], np.sin(ang)[:, None]
    U = uc + ca * rr[None, :]
    V = vc + sa * rr[None, :]
    prof = sample_bilinear(gray, U.ravel(), V.ravel()).reshape(n_rays, n_s)

    g = np.abs(np.diff(prof, axis=1))              # gradient along each ray
    g[~np.isfinite(g)] = 0.0
    k = np.argmax(g, axis=1)
    peak = g[np.arange(n_rays), k]

    # Parabolic subpixel refinement on the gradient peak.
    km1 = np.clip(k - 1, 0, g.shape[1] - 1)
    kp1 = np.clip(k + 1, 0, g.shape[1] - 1)
    y0 = g[np.arange(n_rays), km1]
    y1 = peak
    y2 = g[np.arange(n_rays), kp1]
    denom = (y0 - 2 * y1 + y2)
    shift = np.where(np.abs(denom) > 1e-9, 0.5 * (y0 - y2) / np.where(np.abs(denom) > 1e-9, denom, 1.0), 0.0)
    shift = np.clip(shift, -1.0, 1.0)

    step = rr[1] - rr[0]
    r_edge = lo + (k.astype(np.float64) + 0.5 + shift) * step

    keep = (peak >= min_contrast) & (k > 0) & (k < g.shape[1] - 1)
    if keep.sum() < 24:
        return None
    return (uc + np.cos(ang[keep]) * r_edge[keep],
            vc + np.sin(ang[keep]) * r_edge[keep],
            int(keep.sum()))


def fit_outline(gray, uc0, vc0, r0):
    """Coarse seed -> refined circle, with one robust re-fit. Returns (uc,vc,r,n)."""
    res = refine_edge_points(gray, uc0, vc0, r0)
    if res is None:
        return None
    eu, ev, n_used = res
    try:
        uc, vc, r = fit_circle_algebraic(eu, ev)
    except np.linalg.LinAlgError:
        return None
    # Drop edge points that disagree with the fitted circle, then fit again. One
    # round only - a second would start sculpting the answer.
    d = np.abs(np.hypot(eu - uc, ev - vc) - r)
    mad = np.median(np.abs(d - np.median(d))) + 1e-9
    keep = d < np.median(d) + 2.5 * 1.4826 * mad
    if keep.sum() >= 24:
        try:
            uc, vc, r = fit_circle_algebraic(eu[keep], ev[keep])
            n_used = int(keep.sum())
        except np.linalg.LinAlgError:
            pass
    return float(uc), float(vc), float(r), int(n_used)


# ------------------------------------------------------------ cone geometry

def solve_theta_alpha(rho_mid, rho_half, f):
    """Recover (theta, alpha) from the outline's midpoint radius and half-width.

    Solves, by Newton iteration on two variables:
        f*(tan(theta+alpha) + tan(theta-alpha))/2 = rho_mid
        f*(tan(theta+alpha) - tan(theta-alpha))/2 = rho_half

    theta is the angle from the optical axis to the SPHERE CENTRE, alpha the
    silhouette half-angle. The small-angle shortcut (theta = atan(rho_mid/f),
    alpha = atan(rho_half/f)) is used as the starting guess and is what a naive
    implementation would stop at - it is wrong by about half a pixel for a sphere
    a metre off-axis, which is 2 mm at 5 m."""
    th = math.atan2(rho_mid, f)
    al = math.atan2(rho_half, f)
    for _ in range(24):
        tp, tm = math.tan(th + al), math.tan(th - al)
        F1 = f * 0.5 * (tp + tm) - rho_mid
        F2 = f * 0.5 * (tp - tm) - rho_half
        sp, sm = 1.0 / math.cos(th + al) ** 2, 1.0 / math.cos(th - al) ** 2
        # Jacobian of (F1, F2) wrt (theta, alpha)
        J = np.array([[f * 0.5 * (sp + sm), f * 0.5 * (sp - sm)],
                      [f * 0.5 * (sp - sm), f * 0.5 * (sp + sm)]])
        try:
            d = np.linalg.solve(J, np.array([F1, F2]))
        except np.linalg.LinAlgError:
            break
        th -= d[0]
        al -= d[1]
        if abs(d[0]) < 1e-12 and abs(d[1]) < 1e-12:
            break
        if not (1e-9 < al < 1.0) or abs(th) > 1.4:
            return None
    if not (1e-9 < al < 1.0):
        return None
    return th, al


def centre_from_outline(uc, vc, r_px, fx, fy, cx, cy):
    """Outline in pixels -> sphere centre in CAMERA-BODY coordinates (x fwd, y left, z up).

    The direction to the centre is taken along the line from the principal point
    through the outline centre; theta and alpha are solved exactly along that
    line, then range follows from sin(alpha) = R/d."""
    f = 0.5 * (fx + fy)
    du, dv = uc - cx, vc - cy
    rho_mid = math.hypot(du, dv)
    sol = solve_theta_alpha(rho_mid, r_px, f)
    if sol is None:
        return None
    th, al = sol
    d = SPHERE_R / math.sin(al)
    if rho_mid < 1e-6:
        ox, oy = 0.0, 0.0
    else:
        s = math.tan(th) / rho_mid
        ox, oy = du * s, dv * s          # optical-frame x right, y down (per unit z)
    n = np.array([ox, oy, 1.0])
    n /= np.linalg.norm(n)
    p_opt = n * d                        # optical: x right, y down, z forward
    # Optical -> ROS body used by the ROI and by every other script here.
    return np.array([p_opt[2], -p_opt[0], -p_opt[1]]), d, math.degrees(al)


# ------------------------------------------------------------------- per frame

def detect(gray, xyz, fx, fy, cx, cy):
    """Find up to three spheres and return their centres, or None.

    Coarse stage uses the DEPTH cloud only to say roughly where in the image a
    sphere is - it is allowed to be poor, because everything that decides the
    answer happens afterwards in the image."""
    finite = np.isfinite(xyz).all(axis=2)
    inbox = (finite &
             (xyz[:, :, 0] > ROI['xmin']) & (xyz[:, :, 0] < ROI['xmax']) &
             (xyz[:, :, 1] > ROI['ymin']) & (xyz[:, :, 1] < ROI['ymax']) &
             (xyz[:, :, 2] > ROI['zmin']) & (xyz[:, :, 2] < ROI['zmax']))
    mask = (inbox.astype(np.uint8)) * 255
    if mask.sum() == 0:
        return None
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    n_lab, lab, stats, cent = cv2.connectedComponentsWithStats(mask, connectivity=8)

    cands = []
    for i in range(1, n_lab):
        area = stats[i, cv2.CC_STAT_AREA]
        if area < 150:                   # smaller than any sphere at 10 m
            continue
        w, h = stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT]
        r0 = 0.25 * (w + h)              # mean half-extent as the seed radius
        if r0 < 8:
            continue
        cands.append((area, float(cent[i][0]), float(cent[i][1]), float(r0)))
    if len(cands) < 3:
        return None
    cands.sort(reverse=True)              # biggest blobs first

    found = []
    for _, u0, v0, r0 in cands[:7]:
        fit = fit_outline(gray, u0, v0, r0)
        if fit is None:
            continue
        uc, vc, r_px, n_edge = fit
        got = centre_from_outline(uc, vc, r_px, fx, fy, cx, cy)
        if got is None:
            continue
        c, d_mono, alpha_deg = got
        if not (ROI['xmin'] - 0.3 < c[0] < ROI['xmax'] + 0.3):
            continue                      # outline range disagrees with the box
        # Independent range from the stereo depth of the sphere's own face, for
        # comparison only - it never feeds the centre.
        rr = int(max(3, r_px * 0.5))
        u_i, v_i = int(round(uc)), int(round(vc))
        patch = xyz[max(0, v_i - rr):v_i + rr, max(0, u_i - rr):u_i + rr, 0]
        patch = patch[np.isfinite(patch)]
        d_stereo = float(np.median(patch)) + SPHERE_R if patch.size > 20 else float('nan')
        found.append(dict(c=c, u=uc, v=vc, r_px=r_px, n_edge=n_edge,
                          d_mono=d_mono, d_stereo=d_stereo, alpha=alpha_deg))
        if len(found) == 6:
            break
    if len(found) < 3:
        return None

    import itertools
    best = None
    for combo in itertools.combinations(range(len(found)), 3):
        P = np.array([found[i]['c'] for i in combo])
        ok, dists = validate_triangle(P, **GATE)
        if not ok:
            continue
        apex = int(np.argmax(P[:, 2]))
        bot = sorted([i for i in range(3) if i != apex], key=lambda i: P[i, 1])
        Po = P[[bot[0], bot[1], apex]]
        if abs(Po[0, 2] - Po[1, 2]) > APEX_Z_TOL:
            continue
        if abs((Po[2, 2] - 0.5 * (Po[0, 2] + Po[1, 2])) - GATE['rise']) > APEX_Z_TOL:
            continue
        exp = np.sort(np.array([GATE['base'],
                                math.hypot(GATE['base'] / 2.0, GATE['rise']),
                                math.hypot(GATE['base'] / 2.0, GATE['rise'])]))
        err = float(np.abs(np.sort(dists) - exp).sum())
        order = [combo[bot[0]], combo[bot[1]], combo[apex]]
        if best is None or err < best[0]:
            best = (err, Po, dists, order)
    if best is None:
        return None
    _, Po, dists, order = best
    return Po, dists, [found[i] for i in order]


def main(svo, want, csv_path, roi_x):
    set_roi_x(roi_x)
    init = sl.InitParameters()
    init.set_from_svo_file(svo)
    init.depth_mode = sl.DEPTH_MODE.NEURAL
    init.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
    init.coordinate_units = sl.UNIT.METER
    init.svo_real_time_mode = False
    init.depth_stabilization = 1
    cam = sl.Camera()
    if cam.open(init) != sl.ERROR_CODE.SUCCESS:
        print('OPEN FAILED')
        return 1

    # Rectified calibration, read at runtime. The settings file holds the RAW
    # intrinsics (fx 1270.07); the rectified images these measurements come from
    # use fx ~1258.5, and it drifts slightly between sessions because the camera
    # self-calibrates. Hardcoding either one puts a ~0.9 % scale error straight
    # into every range this method computes.
    lc = cam.get_camera_information().camera_configuration.calibration_parameters.left_cam
    fx, fy, cx, cy = lc.fx, lc.fy, lc.cx, lc.cy
    print('rectified intrinsics: fx %.3f fy %.3f cx %.3f cy %.3f' % (fx, fy, cx, cy))

    total = cam.get_svo_number_of_frames()
    step = max(1, total // (want + 1))
    img, cloud = sl.Mat(), sl.Mat()
    rt = sl.RuntimeParameters()
    rt.confidence_threshold = 50          # matches the live pipeline (verified 2026-09-05)
    rt.texture_confidence_threshold = 100

    f = open(csv_path, 'w')
    f.write('n,x,y,z,base,leg1,leg2,n_bot1,n_bot2,n_apex,tx,ty,tz,stamp\n')
    f2 = open(csv_path.replace('.csv', '_detail.csv'), 'w')
    f2.write('n,sphere,u,v,r_px,alpha_deg,d_mono,d_stereo\n')

    kept = tried = 0
    for k in range(want):
        if step > 3:
            cam.set_svo_position(min(k * step, total - 1))
        if cam.grab(rt) != sl.ERROR_CODE.SUCCESS:
            break
        tried += 1
        cam.retrieve_image(img, sl.VIEW.LEFT_GRAY)
        cam.retrieve_measure(cloud, sl.MEASURE.XYZ)
        gray = img.get_data()[:, :, 0].astype(np.float64)
        xyz = cloud.get_data()[:, :, :3]
        res = detect(gray, xyz, fx, fy, cx, cy)
        if res is None:
            continue
        Po, dists, det = res
        c = centroid_of(Po)
        kept += 1
        f.write('%d,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f,%d,%d,%d,nan,nan,nan,%d\n'
                % (kept, c[0], c[1], c[2], dists[0], dists[1], dists[2],
                   det[0]['n_edge'], det[1]['n_edge'], det[2]['n_edge'], k))
        for j, dd in enumerate(det):
            f2.write('%d,%d,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f\n'
                     % (kept, j, dd['u'], dd['v'], dd['r_px'], dd['alpha'],
                        dd['d_mono'], dd['d_stereo']))
        f.flush(); f2.flush()
        if kept % 10 == 0:
            print('  %d kept of %d tried' % (kept, tried))
    f.close(); f2.close(); cam.close()

    if kept < 5:
        print('ONLY %d DETECTIONS of %d frames - not enough to judge' % (kept, tried))
        return 1
    a = np.genfromtxt(csv_path, delimiter=',', names=True)
    s = [float(a[c].std(ddof=1)) * 1000 for c in ('x', 'y', 'z')]
    print()
    print('SILHOUETTE (Arm C1)  n=%d  acceptance %.0f%%' % (kept, 100.0 * kept / tried))
    print('  sigma x/y/z  %.2f / %.2f / %.2f mm   RMS3D %.2f mm'
          % (s[0], s[1], s[2], sum(v * v for v in s) ** 0.5))
    print('  mean (%.4f, %.4f, %.4f)  sides %.3f/%.3f/%.3f'
          % (a['x'].mean(), a['y'].mean(), a['z'].mean(),
             a['base'].mean(), a['leg1'].mean(), a['leg2'].mean()))
    print('  edge points per sphere: %.0f'
          % np.mean([a['n_bot1'].mean(), a['n_bot2'].mean(), a['n_apex'].mean()]))
    print()
    print('  Compare against Arm A on the SAME recording before believing any of it,')
    print('  and check the sides above against 1.10/1.23/1.23 first.')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1],
                  int(sys.argv[2]) if len(sys.argv) > 2 else 40,
                  sys.argv[3] if len(sys.argv) > 3 else '/tmp/c1.csv',
                  float(sys.argv[4]) if len(sys.argv) > 4 else 5.0))
