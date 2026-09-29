#!/usr/bin/env python3
"""panel_depth_stats.py — per-panel depth quality with error quantiles.

WHY QUANTILES AND NOT JUST RMS
    Research on ZED-class cameras reports depth errors that are heavy-tailed
    rather than Gaussian (Abdelsalam et al., Robotics and Autonomous Systems 179,
    2024, find a t-distribution fits better). When the tail is heavy, RMS
    understates the worst case, and it is the worst case that decides whether an
    obstacle lands in the right occupancy cell.

    So this reports the 50th, 90th and 99th percentile of |residual| beside the
    RMS, per the acceptance test recommended in the project's research review:
    if the 99th percentile exceeds one grid cell (0.05 m), that surface at that
    range is unreliable for asserting free space.

WHAT MORE SAMPLES DO AND DO NOT BUY
    The mean and RMS converge quickly - going from 60 to 1000 frames moved them
    by under 1.5 mm on this rig. More frames do NOT reduce the frame-to-frame
    spread, which is a property of the sensor. What they buy is a usable estimate
    of the TAIL: a 99th percentile needs a few hundred samples at minimum before
    it means anything. That is the only reason to run thousands.

EVERY RUN IS SAVED
    An earlier measurement at 3.11 m was printed to a terminal and never written
    anywhere, so it was lost when the session ended. This writes a CSV row per
    panel plus a text summary, both stamped with the condition label, before it
    prints anything.

    usage:
      rosrun sidewalk_evaluation panel_depth_stats.py \\
          _label:=dark_room_3.11m _frames:=3000 _out:=~/bench_2026-08-29
"""
import math
import os
import sys

import numpy as np
import rospy
from sensor_msgs.msg import CameraInfo, Image

# Panel image columns at the 3.11 m position, derived from a brightness profile
# across the panels rather than by eye: white and printed read ~172-176 mean
# grey in the lit session, the black panel and the background dividers ~74-85.
# Interiors only - the boundaries and the occluded right edge of the white panel
# are deliberately excluded.
PANELS_311 = [("white_blank", 740, 920), ("printed_sign", 980, 1240),
              ("black_blank", 1300, 1520)]
ROW0, ROW1 = 560, 780


def main():
    rospy.init_node("panel_depth_stats", anonymous=True, disable_signals=True)
    label = str(rospy.get_param("~label", "unlabelled"))
    n = int(rospy.get_param("~frames", 1000))
    out = os.path.expanduser(str(rospy.get_param("~out", "~/bench_2026-08-29")))
    note = str(rospy.get_param("~note", ""))
    if not os.path.isdir(out):
        os.makedirs(out)

    ci = rospy.wait_for_message(
        "/zedx_front/zed_node/depth/camera_info", CameraInfo, timeout=25)
    fx, fy, cx, cy = ci.K[0], ci.K[4], ci.K[2], ci.K[5]
    mode = rospy.get_param("/zedx_front/zed_node/depth/depth_mode", "unknown")

    print("  condition : %s" % label)
    print("  depth mode: %s        (recorded with the result, not assumed)" % mode)
    print("  camera    : %dx%d  fx=%.1f" % (ci.width, ci.height, fx))
    print("  frames    : %d" % n)
    print()

    acc = {p[0]: {"comp": [], "z": [], "rms": [], "res": []} for p in PANELS_311}
    t0 = rospy.Time.now()
    used = 0
    for i in range(n):
        try:
            m = rospy.wait_for_message(
                "/zedx_front/zed_node/depth/depth_registered", Image, timeout=15)
        except rospy.ROSException:
            print("  depth stream stopped after %d frames" % i)
            break
        d = np.frombuffer(m.data, dtype=np.float32).reshape(m.height, m.width)
        used += 1
        for nm, c0, c1 in PANELS_311:
            sub = d[ROW0:ROW1, c0:c1]
            ok = np.isfinite(sub) & (sub > 0.3) & (sub < 10.0)
            acc[nm]["comp"].append(100.0 * ok.sum() / sub.size)
            if ok.sum() < 300:
                continue
            us, vs = np.meshgrid(np.arange(c0, c1), np.arange(ROW0, ROW1))
            Z = sub[ok]
            P = np.stack([(us[ok] - cx) * Z / fx, (vs[ok] - cy) * Z / fy, Z], 1)
            c = P.mean(0)
            _, _, vt = np.linalg.svd(P - c, full_matrices=False)
            r = (P - c).dot(vt[2])
            acc[nm]["z"].append(Z.mean())
            acc[nm]["rms"].append(r.std())
            # keep a bounded random sample of residuals so the quantiles come
            # from the whole run without holding millions of points in memory
            acc[nm]["res"].append(np.abs(r[::max(1, len(r) // 400)]))
        if i and i % 500 == 0:
            el = (rospy.Time.now() - t0).to_sec()
            print("    %d/%d frames (%.0f s, %.1f Hz)" % (i, n, el, i / max(el, 1e-6)))

    el = (rospy.Time.now() - t0).to_sec()
    print("  collected %d frames in %.0f s (%.2f Hz)" % (used, el, used / max(el, 1e-6)))
    print()

    CELL = 50.0     # mm - the occupancy grid cell size this is judged against
    rows = []
    print("  %-13s %6s %8s %9s %8s %8s %8s %9s"
          % ("panel", "compl", "depth", "rms", "p50", "p90", "p99", "worst"))
    print("  %-13s %6s %8s %9s %8s %8s %8s %9s"
          % ("", "%", "m", "mm", "mm", "mm", "mm", "mm"))
    for nm, _, _ in PANELS_311:
        a = acc[nm]
        if not a["rms"]:
            print("  %-13s  no usable frames (completeness %.1f %%)"
                  % (nm, float(np.mean(a["comp"])) if a["comp"] else 0.0))
            rows.append((nm, float(np.mean(a["comp"])) if a["comp"] else 0.0,
                         *(float("nan"),) * 6))
            continue
        res = np.concatenate(a["res"]) * 1000.0
        p50, p90, p99 = np.percentile(res, [50, 90, 99])
        comp = float(np.mean(a["comp"]))
        z = float(np.mean(a["z"]))
        rms = float(np.mean(a["rms"])) * 1000.0
        worst = float(res.max())
        flag = "" if p99 < CELL else "   <-- p99 EXCEEDS ONE GRID CELL"
        print("  %-13s %6.2f %8.3f %9.1f %8.1f %8.1f %8.1f %9.1f%s"
              % (nm, comp, z, rms, p50, p90, p99, worst, flag))
        rows.append((nm, comp, z, rms, p50, p90, p99, worst))

    print()
    print("  ACCEPTANCE TEST: a surface whose 99th-percentile error exceeds one")
    print("  grid cell (%.0f mm) cannot be trusted to assert free space at this" % CELL)
    print("  range. RMS alone understates it - these errors are heavy-tailed.")

    csv = os.path.join(out, "panel_depth_stats.csv")
    new = not os.path.exists(csv)
    with open(csv, "a") as fh:
        if new:
            fh.write("label,depth_mode,frames,rate_hz,panel,completeness_pct,"
                     "mean_depth_m,rms_mm,p50_mm,p90_mm,p99_mm,worst_mm,note\n")
        for r in rows:
            fh.write("%s,%s,%d,%.2f,%s,%.3f,%.4f,%.2f,%.2f,%.2f,%.2f,%.2f,%s\n"
                     % (label, mode, used, used / max(el, 1e-6), r[0], r[1],
                        r[2], r[3], r[4], r[5], r[6], r[7], note.replace(",", ";")))
    print()
    print("  appended %d rows to %s" % (len(rows), csv))


if __name__ == "__main__":
    main()
