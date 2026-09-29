#!/usr/bin/env python3
"""lidar_board_check.py — an independent sensor's opinion of the same boards.

WHY THIS EXISTS

    The research notes are explicit about learned stereo (`research_bench-test-1.md`):

        keep NEURAL only if it reduces residuals "without introducing
        hallucinated geometry (check against a LiDAR reference)"

    A neural network can output a smooth, plausible, WRONG surface. Nothing
    inside the stereo pipeline can catch that, because the check and the thing
    being checked share every assumption. A LiDAR shares none of them: it emits
    its own light and times the return, so texture, albedo and print make no
    difference to it.

WHAT THIS CAN AND CANNOT SETTLE — read this before quoting any number

    CAN:  where the surface is, how wide it is, whether it is planar, and
          whether anything is there at all. That is what "hallucinated
          geometry" means and it is the question worth asking.

    CANNOT: adjudicate fine flatness. These sensors carry roughly 15-30 mm of
          range noise, while NEURAL reports 4-7 mm plane residuals on these
          boards. The LiDAR is LESS precise than the thing under test, so a
          LiDAR residual larger than the camera's proves nothing about the
          camera. Reporting it as though it did would be backwards.

    Plain terms: the LiDAR is the right instrument for "is the wall really
    there, and really that size", and the wrong instrument for "is the wall
    smooth to within a few millimetres".

NO CROSS-CALIBRATION IS USED

    The LiDAR sits somewhere else on the robot than the camera, and that offset
    has never been measured. So this deliberately reports only quantities that
    do not need it:
      - the wall's WIDTH (against the tape's 2.388 m)
      - its FLATNESS (a property of the surface, not of where it sits)
      - the RELATIVE depth between the three boards (the tape says the two
        blanks sit 12.7 mm behind the printed sign)
    Absolute distance is printed for information but explicitly not compared
    against the camera, because the mounting offset would dominate it.

    usage:
      rosrun sidewalk_evaluation lidar_board_check.py _topic:=/ouster/points \\
             _label:=ouster _out:=~/bench_2026-08-30
"""
import csv as csvmod
import math
import os

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

WALL_TRUE_M = 2.388          # 94 in, tape-measured, three boards end to end
BOARD_W = 0.796


def main():
    rospy.init_node("lidar_board_check", anonymous=True, disable_signals=True)
    topic = str(rospy.get_param("~topic", "/ouster/points"))
    label = str(rospy.get_param("~label", "lidar"))
    out = os.path.expanduser(str(rospy.get_param("~out", "~/bench")))
    nfr = int(rospy.get_param("~frames", 20))
    fwd_lo = float(rospy.get_param("~fwd_min", 2.70))
    fwd_hi = float(rospy.get_param("~fwd_max", 3.80))
    z_lo = float(rospy.get_param("~z_min", -0.80))   # in the sensor's own frame
    z_hi = float(rospy.get_param("~z_max", -0.05))
    names = str(rospy.get_param("~names", "left,middle,right")).split(",")
    if not os.path.isdir(out):
        os.makedirs(out)

    acc = []
    frame_id = ""
    for _ in range(nfr):
        try:
            m = rospy.wait_for_message(topic, PointCloud2, timeout=20)
        except rospy.ROSException:
            break
        frame_id = m.header.frame_id
        a = np.array([p[:3] for p in
                      pc2.read_points(m, ("x", "y", "z"), skip_nans=True)])
        if len(a):
            acc.append(a[np.isfinite(a).all(1)])
    if not acc:
        print("  no data on %s" % topic)
        return
    P = np.vstack(acc)
    print("  %s | frame %s | %d frames | %d points"
          % (topic, frame_id, len(acc), len(P)))

    # the board slab, in the LiDAR's own frame: x forward, y left, z up
    y_half = float(rospy.get_param("~y_half", 1.30))
    sel = ((P[:, 0] > fwd_lo) & (P[:, 0] < fwd_hi) &
           (P[:, 2] > z_lo) & (P[:, 2] < z_hi) & (np.abs(P[:, 1]) < y_half))
    W = P[sel]
    # Second pass: keep only what sits at the WALL's own range. A flat wall
    # occupies one narrow depth band; chairs and tables beside and behind it do
    # not. The band is set from the physics - a 0.99 m board leaning at most 6
    # degrees spans 0.10 m in depth - and never from the answer it produces.
    if len(W) > 200:
        centre = W[np.abs(W[:, 1]) < 0.40]
        if len(centre) > 50:
            r0 = float(np.median(centre[:, 0]))
            band = float(rospy.get_param("~depth_band", 0.20))
            W = W[np.abs(W[:, 0] - r0) < band]
            print("  wall range %.3f m; keeping points within +/-%.2f m of it"
                  % (r0, band))
    print("  %d points in the slab (%.2f-%.2f m ahead, z %.2f..%.2f, |y|<1.60)"
          % (len(W), fwd_lo, fwd_hi, z_lo, z_hi))
    if len(W) < 200:
        print("  too few to judge. Widen the slab, or the boards are not here.")
        return

    # ---- the wall as a whole -----------------------------------------------
    yl, yh = np.percentile(W[:, 1], [1, 99])
    width = yh - yl
    cen = W.mean(0)
    _, _, vt = np.linalg.svd(W - cen, full_matrices=False)
    n = vt[2] / np.linalg.norm(vt[2])
    resid = float((W - cen).dot(n).std())
    # how far the wall's normal is from horizontal-facing (an upright wall
    # faces sideways, so its normal has near-zero vertical component)
    lean = math.degrees(math.asin(min(1.0, abs(float(n[2])))))
    print()
    print("  === THE WALL, AS THE LIDAR SEES IT ===")
    print("  width          %.3f m   (tape 2.388 m, difference %+.0f mm)"
          % (width, 1000 * (width - WALL_TRUE_M)))
    print("  flatness       %.1f mm rms   <- includes this sensor's own"
          " 15-30 mm noise;" % (resid * 1000))
    print("                              NOT comparable with the camera's figure")
    print("  lean           %+.1f deg from upright" % lean)
    print("  distance       %.3f m from the LiDAR (mounting offset unknown,"
          % np.median(W[:, 0]))
    print("                 so this is NOT comparable with the camera either)")

    # ---- the three boards, by equal division of the detected wall -----------
    print()
    print("  board      points   width     vs 0.796 m   depth vs the middle board")
    rows = []
    w3 = width / 3.0
    mids = []
    for i in range(3):
        a, b = yh - (i + 1) * w3, yh - i * w3          # left (high y) first
        k = (W[:, 1] >= a) & (W[:, 1] < b)
        B = W[k]
        if len(B) < 60:
            print("  %-10s %6d   too few points" % (names[i], len(B)))
            mids.append(np.nan)
            continue
        bw = np.percentile(B[:, 1], 99) - np.percentile(B[:, 1], 1)
        mids.append(float(np.median(B[:, 0])))
        rows.append(dict(label=label, method="lidar_board_check_v1",
                         topic=topic, board=names[i], points=len(B),
                         width_m=round(bw, 4),
                         width_err_mm=round(1000 * (bw - BOARD_W), 1),
                         depth_median_m=round(mids[-1], 4),
                         wall_width_m=round(width, 4),
                         wall_width_err_mm=round(1000 * (width - WALL_TRUE_M), 1),
                         wall_flatness_rms_mm=round(resid * 1000, 1),
                         wall_lean_deg=round(lean, 2), frames=len(acc)))
    if len(mids) == 3 and np.isfinite(mids).all():
        base = mids[1]
        for i, r in enumerate(rows):
            d = mids[i] - base
            r["depth_vs_middle_mm"] = round(1000 * d, 1)
            print("  %-10s %6d   %.3f m   %+6.0f mm    %+6.0f mm"
                  % (r["board"], r["points"], r["width_m"],
                     r["width_err_mm"], 1000 * d))
        print()
        print("  The tape says the two blanks sit +12.7 mm behind the middle")
        print("  board. That is the only depth comparison here that does not")
        print("  need the LiDAR-to-camera offset, so it is the one to trust.")

    if rows:
        p = os.path.join(out, "lidar_board_check.csv")
        new = not os.path.exists(p)
        with open(p, "a", newline="") as f:
            w = csvmod.DictWriter(f, fieldnames=list(rows[0].keys()))
            if new:
                w.writeheader()
            w.writerows(rows)
        print("\n  appended to %s" % p)


if __name__ == "__main__":
    main()
