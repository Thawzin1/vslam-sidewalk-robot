#!/usr/bin/env python3
"""floor_vs_obstacle.py — is the map calling a standing board FLOOR?

WHAT IT READS, and why this is evidence rather than inference

    RTAB-Map splits every frame into two clouds before it builds the grid:

        /rtabmap/cloud_ground     -> these cells become FREE (drivable)
        /rtabmap/cloud_obstacles  -> these cells become OCCUPIED

    This counts, inside each board's footprint, how many points went into each.
    It is not a model of what the map might do - it is the two clouds the map
    actually used. If a vertical board's points are turning up in the ground
    cloud, the map is not failing to see it; it is seeing it and deciding it is
    floor, which is the far more dangerous outcome because the map then asserts
    drivable space rather than admitting ignorance.

HOW THE SPLIT IS MADE, and how it goes wrong

    Grid/NormalsSegmentation is true: each point is labelled by the direction
    its little patch of surface faces. Grid/MaxGroundAngle (30 degrees on this
    build - CHECK IT, do not assume, the printed default is 45) is the cut-off.
    Face within that angle of straight up and you are floor.

    Plain terms: the map sorts points by which way the surface is tilted. Tip a
    board back far enough and it genuinely looks like a ramp, so the map calls
    it one - correctly. But a board standing UP whose depth is noisy has patches
    tilted every which way, and some of those get mistaken for floor.

    So this number must always be read beside the board's measured LEAN from
    board_geometry.py. A high floor fraction on a leaning board is the map
    being right. On an upright board it is the map being wrong.

NAMING - THE TRAP THIS SCRIPT EXISTS TO AVOID

    An earlier version hard-coded "white_blank" as the left-hand window. The
    boards were later swapped and the script kept reporting the old names, so
    the black board's numbers appeared under the white board's label. Windows
    are therefore given on the command line and every row records BOTH the
    physical position and what is standing there.

    usage:
      rosrun sidewalk_evaluation floor_vs_obstacle.py \\
          _windows:="left:black_blank:0.037:0.867,middle:printed_sign:-0.793:0.037" \\
          _x_min:=3.10 _x_max:=3.50 _label:=swap _out:=~/bench_2026-08-30
"""
import csv as csvmod
import os

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2


def grab(topic, n):
    pts = []
    frame = ""
    for _ in range(n):
        m = rospy.wait_for_message(topic, PointCloud2, timeout=25)
        frame = m.header.frame_id
        a = np.array([p[:3] for p in
                      pc2.read_points(m, ("x", "y", "z"), skip_nans=True)])
        if len(a):
            pts.append(a)
    return (np.vstack(pts) if pts else np.empty((0, 3))), frame


def main():
    rospy.init_node("floor_vs_obstacle", anonymous=True, disable_signals=True)
    label = str(rospy.get_param("~label", "unlabelled"))
    out = os.path.expanduser(str(rospy.get_param("~out", "~/bench")))
    nfr = int(rospy.get_param("~frames", 6))
    x0 = float(rospy.get_param("~x_min", 3.10))
    x1 = float(rospy.get_param("~x_max", 3.50))
    spec = str(rospy.get_param("~windows", ""))
    if not spec:
        print("  give _windows:=\"position:content:y_min:y_max,...\"")
        return
    wins = []
    for part in spec.split(","):
        f = part.split(":")
        if len(f) != 4:
            print("  bad window %r - want position:content:y_min:y_max" % part)
            return
        wins.append((f[0], f[1], float(f[2]), float(f[3])))
    if not os.path.isdir(out):
        os.makedirs(out)

    got = {}
    for t in ("/rtabmap/cloud_ground", "/rtabmap/cloud_obstacles"):
        try:
            got[t] = grab(t, nfr)
            print("  %-28s %8d points   frame=%s"
                  % (t, len(got[t][0]), got[t][1]))
        except rospy.ROSException:
            print("  %-28s NOT PUBLISHING - is rtabmap running?" % t)
            got[t] = (np.empty((0, 3)), "")

    frames = {v[1] for v in got.values() if v[1]}
    if len(frames) > 1:
        print("  !! the two clouds are in DIFFERENT frames %s - counts would"
              " not be comparable." % frames)
        return
    print("\n  depth window x %.2f..%.2f m, %d frames of each cloud" % (x0, x1, nfr))
    print("  frame: %s   (windows must be given in this frame)"
          % (list(frames)[0] if frames else "?"))
    print()
    print("  position   what is there      called FLOOR   called OBSTACLE   %"
          " floor")
    rows = []
    for pos, content, y0, y1 in wins:
        n = {}
        for t in ("/rtabmap/cloud_ground", "/rtabmap/cloud_obstacles"):
            P = got[t][0]
            if not len(P):
                n[t] = 0
                continue
            m = ((P[:, 0] > x0) & (P[:, 0] < x1) &
                 (P[:, 1] > y0) & (P[:, 1] < y1))
            n[t] = int(m.sum())
        g, o = n["/rtabmap/cloud_ground"], n["/rtabmap/cloud_obstacles"]
        tot = g + o
        pct = (100.0 * g / tot) if tot else float("nan")
        print("  %-10s %-18s %8d       %10d       %s"
              % (pos, content, g, o,
                 ("%5.1f %%" % pct) if tot else "  no points"))
        rows.append(dict(label=label, method="floor_vs_obstacle_v1",
                         position=pos, content=content,
                         ground_points=g, obstacle_points=o,
                         pct_called_floor=(round(pct, 2) if tot else ""),
                         x_min=x0, x_max=x1, y_min=y0, y_max=y1, frames=nfr))
    if rows:
        p = os.path.join(out, "floor_vs_obstacle.csv")
        new = not os.path.exists(p)
        with open(p, "a", newline="") as f:
            w = csvmod.DictWriter(f, fieldnames=list(rows[0].keys()))
            if new:
                w.writeheader()
            w.writerows(rows)
        print("\n  appended to %s" % p)
        print("  READ THESE BESIDE THE MEASURED LEAN. A high floor fraction on a")
        print("  leaning board is the map being correct; on an upright board it")
        print("  is the map asserting drivable space where an obstacle stands.")


if __name__ == "__main__":
    main()
