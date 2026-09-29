#!/usr/bin/env python3
"""bench_report.py — measure the three bench panels and draw what happened.

WHAT IT PRODUCES, and why each picture exists

    1. panels_view.png    the camera image with each panel outlined, so the
                          regions every number refers to can be checked by eye
                          rather than trusted.
    2. depth_error.png    each panel's depth minus its own best-fit plane, as a
                          colour map. This turns "65 mm rms" into something you
                          can see: a printed surface looks smooth, a blank one
                          looks like static.
    3. grid_top.png       the occupancy grid around the rig, with each panel's
                          footprint boxed. Occupied, free and unknown are
                          separate colours, so a panel that failed to register
                          is visible as a gap in the wall.

THE MEASUREMENT, and one correction to how it was done before
    An earlier pass sampled a depth window 0.44 m deep around a wall that is
    physically one cell thick. Most of that box is genuinely empty space in
    FRONT of the panel, so it reported ~75 % free for every panel and that
    number meant nothing. The count that carries information is OCCUPIED CELLS
    against the number a panel of that width should produce (width / cell size),
    and the free fraction only means something inside a window tight enough to
    contain the surface and little else.

    So this reports both, and states the window depth in cells beside them.

    usage:  rosrun sidewalk_evaluation bench_report.py _out:=~/bench_2026-08-29
"""
import math
import os

import numpy as np
import rospy
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import CameraInfo, Image

try:
    import cv2
except ImportError:                                   # pragma: no cover
    cv2 = None

# Panel edges in the camera image (full 1920-wide frame) and in robot y.
# Both are recorded because they are independent: the pixel columns come from
# the image, the y values from those columns through the MEASURED field of view.
PANELS = [
    # label            px0   px1     y_lo    y_hi   truth_w  colour(BGR)
    ("white_blank",    531,  920,   +0.10,  +0.83,   0.79,  (90, 200, 90)),
    ("printed_sign",   920, 1299,   -0.66,  +0.06,   0.79,  (240, 160, 60)),
    ("black_blank",   1299, 1635,   -1.33,  -0.70,   0.79,  (80, 90, 235)),
]
ROW0, ROW1 = 450, 820
CELL = 0.05


def q2m(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def main():
    rospy.init_node("bench_report", anonymous=True, disable_signals=True)
    out = os.path.expanduser(str(rospy.get_param("~out", "~/bench_2026-08-29")))
    nframes = int(rospy.get_param("~frames", 1000))
    if not os.path.isdir(out):
        os.makedirs(out)

    ci = rospy.wait_for_message(
        "/zedx_front/zed_node/depth/camera_info", CameraInfo, timeout=20)
    fx, fy, cx, cy = ci.K[0], ci.K[4], ci.K[2], ci.K[5]
    print("  camera: %dx%d  fx=%.1f  HFOV=%.2f  VFOV=%.2f deg"
          % (ci.width, ci.height, fx,
             math.degrees(2 * math.atan(ci.width / (2 * fx))),
             math.degrees(2 * math.atan(ci.height / (2 * fy)))))
    print("  frames requested: %d  (~%.0f s at 10 Hz)" % (nframes, nframes / 10.0))
    print()

    # ---- 1. the depth study's repeated measurement, one sample per frame ----------
    stats = {p[0]: [] for p in PANELS}
    last_depth = None
    t0 = rospy.Time.now()
    for i in range(nframes):
        try:
            m = rospy.wait_for_message(
                "/zedx_front/zed_node/depth/depth_registered", Image, timeout=15)
        except rospy.ROSException:
            print("  cloud stopped after %d frames" % i)
            break
        d = np.frombuffer(m.data, dtype=np.float32).reshape(m.height, m.width)
        last_depth = d
        for nm, c0, c1, _, _, _, _ in PANELS:
            sub = d[ROW0:ROW1, c0:c1]
            ok = np.isfinite(sub) & (sub > 0.3) & (sub < 10.0)
            if ok.sum() < 500:
                continue
            us, vs = np.meshgrid(np.arange(c0, c1), np.arange(ROW0, ROW1))
            Z = sub[ok]
            P = np.stack([(us[ok] - cx) * Z / fx, (vs[ok] - cy) * Z / fy, Z], 1)
            c = P.mean(0)
            _, _, vt = np.linalg.svd(P - c, full_matrices=False)
            r = (P - c).dot(vt[2])
            stats[nm].append((100.0 * ok.sum() / sub.size, Z.mean(),
                              r.std(), np.abs(r).max()))
        if i and i % 200 == 0:
            print("    %d/%d frames (%.0f s elapsed)"
                  % (i, nframes, (rospy.Time.now() - t0).to_sec()))
    el = (rospy.Time.now() - t0).to_sec()
    print("  collected in %.0f s" % el)
    print()

    print("  == DEPTH STUDY'S REPEATED MEASUREMENT (one sample per frame) ==")
    print("  panel          n     completeness      mean depth      "
          "plane residual        worst")
    for nm, _, _, _, _, _, _ in PANELS:
        a = np.array(stats[nm])
        if not len(a):
            print("  %-13s  no usable frames" % nm)
            continue
        print("  %-13s %4d  %5.2f +/-%4.2f %%  %.3f +/-%.3f m  "
              "%5.1f +/-%4.1f mm  %6.1f mm"
              % (nm, len(a), a[:, 0].mean(), a[:, 0].std(),
                 a[:, 1].mean(), a[:, 1].std(),
                 a[:, 2].mean() * 1000, a[:, 2].std() * 1000,
                 a[:, 3].max() * 1000))
    print()

    # ---- 2. the grid, in a window tight enough to mean something -----------
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    rospy.sleep(1.5)
    state = {"g": None}
    sub_ = rospy.Subscriber("/rtabmap/grid_map", OccupancyGrid,
                            lambda g: state.__setitem__("g", g), queue_size=1)
    t1 = rospy.Time.now()
    while state["g"] is None and (rospy.Time.now() - t1).to_sec() < 30:
        rospy.sleep(0.2)
    g = state["g"]
    if g is None:
        print("  NO GRID - is rtabmap mapping? Skipping the grid section.")
        return

    W, H, res = g.info.width, g.info.height, g.info.resolution
    ox, oy = g.info.origin.position.x, g.info.origin.position.y
    cells = np.array(g.data, dtype=np.int16).reshape(H, W)
    tr = buf.lookup_transform(g.header.frame_id, "base_footprint",
                              rospy.Time(0), rospy.Duration(5.0))
    yaw = math.atan2(2 * (tr.transform.rotation.w * tr.transform.rotation.z),
                     1 - 2 * tr.transform.rotation.z ** 2)
    tx, ty = tr.transform.translation.x, tr.transform.translation.y
    cw, sw = math.cos(yaw), math.sin(yaw)

    print("  == THE OCCUPANCY GRID ==")
    print("  window 2.50..2.70 m deep = %d cells, tight around the surface"
          % int(round(0.20 / res)))
    print("  panel          occupied  free  unknown   expected occupied   ratio")
    grid_rows = []
    for nm, _, _, ylo, yhi, tw, _ in PANELS:
        pts = [(x, y) for x in (2.50, 2.70) for y in (ylo, yhi)]
        gx = [tx + cw * x - sw * y for x, y in pts]
        gy = [ty + sw * x + cw * y for x, y in pts]
        i0 = max(0, int(math.floor((min(gx) - ox) / res)))
        i1 = min(W, int(math.ceil((max(gx) - ox) / res)))
        j0 = max(0, int(math.floor((min(gy) - oy) / res)))
        j1 = min(H, int(math.ceil((max(gy) - oy) / res)))
        s = cells[j0:j1, i0:i1]
        occ = int((s > 50).sum()); free = int(((s >= 0) & (s <= 50)).sum())
        unk = int((s < 0).sum())
        exp = (yhi - ylo) / res
        print("  %-13s %6d %6d %8d %14.1f %10.2f"
              % (nm, occ, free, unk, exp, occ / exp if exp else 0))
        grid_rows.append((nm, i0, i1, j0, j1, occ, free, unk, exp))
    print()
    print("  'expected occupied' is the panel's width divided by the cell size:")
    print("  the number of cells one thin wall of that width should fill.")

    if cv2 is None:
        print("  (cv2 missing - no images written)")
        return

    # ---- 3. the pictures ---------------------------------------------------
    img = rospy.wait_for_message(
        "/zedx_front/zed_node/left/image_rect_color", Image, timeout=20)
    rgb = np.frombuffer(img.data, dtype=np.uint8).reshape(
        img.height, img.width, -1)[:, :, :3].copy()
    for nm, c0, c1, _, _, _, col in PANELS:
        cv2.rectangle(rgb, (c0, ROW0), (c1, ROW1), col, 4)
        cv2.putText(rgb, nm, (c0 + 8, ROW0 - 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.1, col, 3)
    cv2.imwrite(os.path.join(out, "panels_view.png"),
                cv2.resize(rgb, (1280, 800)))

    # depth minus each panel's own plane, so only raggedness shows
    err = np.zeros((ROW1 - ROW0, PANELS[-1][2] - PANELS[0][1]), np.float32)
    for nm, c0, c1, _, _, _, _ in PANELS:
        sub = last_depth[ROW0:ROW1, c0:c1]
        ok = np.isfinite(sub) & (sub > 0.3) & (sub < 10.0)
        us, vs = np.meshgrid(np.arange(c0, c1), np.arange(ROW0, ROW1))
        if ok.sum() < 500:
            continue
        Z = sub[ok]
        P = np.stack([(us[ok] - cx) * Z / fx, (vs[ok] - cy) * Z / fy, Z], 1)
        c = P.mean(0)
        _, _, vt = np.linalg.svd(P - c, full_matrices=False)
        e = np.full(sub.shape, np.nan, np.float32)
        e[ok] = (P - c).dot(vt[2])
        err[:, c0 - PANELS[0][1]:c1 - PANELS[0][1]] = e
    vis = np.clip((err * 1000 + 100) / 200.0, 0, 1)
    vis[np.isnan(err)] = 0.5
    hm = cv2.applyColorMap((vis * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    hm[np.isnan(err)] = (40, 40, 40)
    for nm, c0, c1, _, _, _, col in PANELS:
        cv2.rectangle(hm, (c0 - PANELS[0][1], 0),
                      (c1 - PANELS[0][1], ROW1 - ROW0 - 1), col, 3)
        cv2.putText(hm, nm, (c0 - PANELS[0][1] + 8, 34),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
    cv2.putText(hm, "depth minus each panel's own plane, +/-100 mm full scale",
                (10, ROW1 - ROW0 - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (255, 255, 255), 2)
    cv2.imwrite(os.path.join(out, "depth_error.png"), cv2.resize(hm, (1280, 420)))

    # the grid, cropped around the rig
    ai = int((tx + 1.0 - ox) / res); bi = int((tx + 3.6 - ox) / res)
    aj = int((ty - 2.2 - oy) / res); bj = int((ty + 2.2 - oy) / res)
    ai, bi = max(0, ai), min(W, bi); aj, bj = max(0, aj), min(H, bj)
    crop = cells[aj:bj, ai:bi]
    pic = np.full(crop.shape + (3,), 60, np.uint8)
    pic[crop == -1] = (60, 60, 60)
    pic[(crop >= 0) & (crop <= 50)] = (235, 235, 235)
    pic[crop > 50] = (30, 30, 30)
    pic = cv2.resize(pic, (crop.shape[1] * 9, crop.shape[0] * 9),
                     interpolation=cv2.INTER_NEAREST)
    for nm, i0, i1, j0, j1, occ, _, _, exp in grid_rows:
        col = [p[6] for p in PANELS if p[0] == nm][0]
        cv2.rectangle(pic, ((i0 - ai) * 9, (j0 - aj) * 9),
                      ((i1 - ai) * 9, (j1 - aj) * 9), col, 2)
    # Flip BEFORE any text is drawn. Doing it after mirrors the labels, which
    # is what the first version of this figure did - the picture was right and
    # every word on it was backwards.
    pic = cv2.flip(pic, 0)          # +y is left; flip so the picture reads as seen
    for nm, i0, i1, j0, j1, occ, _, _, exp in grid_rows:
        col = [q[6] for q in PANELS if q[0] == nm][0]
        ytop = pic.shape[0] - (j1 - aj) * 9
        cv2.putText(pic, "%s  %d of %.0f cells" % (nm, occ, exp),
                    ((i0 - ai) * 9 - 4, max(18, ytop - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, col, 2)
    cv2.putText(pic, "black=occupied  white=free  grey=unknown",
                (10, pic.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (0, 140, 255), 2)
    cv2.putText(pic, "robot is off the left edge, looking right",
                (10, pic.shape[0] - 36), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                (0, 140, 255), 2)
    cv2.imwrite(os.path.join(out, "grid_top.png"), pic)
    print()
    print("  wrote panels_view.png, depth_error.png, grid_top.png to %s" % out)


if __name__ == "__main__":
    main()
