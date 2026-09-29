#!/usr/bin/env python3
"""grid_sample.py — count occupancy-grid cells inside a rectangle, and draw them.

WHY THIS EXISTS
    The bench test asks a question that cannot be answered by looking at RViz:
    does a low-texture white surface become OCCUPIED, stay UNKNOWN, or get
    actively marked FREE? "It showed up" is not a result. This prints the three
    counts over a named region, so two conditions can be compared as numbers.

    The third outcome is the interesting one. Grid/RayTracing is true on this
    build, which means a ray reaching a FARTHER return carves every cell it
    passes through as free. If the white board returns no depth while the black
    boards beside it do, its cells are not merely unknown — the map asserts,
    confidently and wrongly, that you can drive through it. That is the actual
    scientific content of the test, and it looks identical to "nothing there"
    unless you count free cells separately from unknown ones.

THE ASCII MAP
    Counts alone cannot tell "a thin sliver of the board registered" from "the
    edges registered and the middle is a hole". The map prints the region cell
    by cell, so the shape is visible:

        #  occupied      .  free (asserted drivable)      ?  unknown

WHERE THE RECTANGLE IS
    Given in whatever frame you name (default base_footprint, the robot's
    footprint on the floor). Its corners are transformed into the grid's own
    frame, so this stays correct if the robot is not at the map origin. All four
    corners are transformed, not two, because a rotation between the frames
    turns an axis-aligned rectangle into a tilted one and taking two corners
    would silently shrink it.

    usage:
      rosrun sidewalk_slam grid_sample.py _x_min:=2.20 _x_max:=2.55 \\
             _y_min:=-0.40 _y_max:=0.40 _label:=B2_white_board
      rosrun sidewalk_slam grid_sample.py _label:=B2 _csv:=~/bench/bench.csv
"""
import math
import os
import sys

import numpy as np
import rospy
import tf2_ros
from nav_msgs.msg import OccupancyGrid


# RTAB-Map publishes 0 = free, 100 = occupied, -1 = unknown. Anything strictly
# above this counts as occupied; anything from 0 to here counts as free. The
# threshold is exposed because a probabilistic grid can sit in between, and
# lumping the middle in with "free" would understate an obstacle.
OCC_THRESHOLD = 50


def quat_yaw(q):
    """Yaw only. The grid is a horizontal plane; roll and pitch between these
    frames would mean the grid is not the floor, which is reported, not used."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    rospy.init_node("grid_sample", anonymous=True, disable_signals=True)
    topic = rospy.get_param("~topic", "/rtabmap/grid_map")
    frame = rospy.get_param("~frame", "base_footprint")
    label = str(rospy.get_param("~label", "unlabelled"))
    csv = os.path.expanduser(str(rospy.get_param("~csv", "")))

    # Default rectangle: the white board's footprint in the bench rig — 0.79 m
    # wide, centred on the robot's centreline, standing at 2.36 m. A vertical
    # board projects onto a 2D grid as a THIN LINE, so x is a narrow band around
    # 2.36 rather than a deep box; widen ~x_tol if the rig moved.
    x_min = float(rospy.get_param("~x_min", 2.20))
    x_max = float(rospy.get_param("~x_max", 2.55))
    y_min = float(rospy.get_param("~y_min", -0.40))
    y_max = float(rospy.get_param("~y_max", 0.40))
    draw = bool(rospy.get_param("~draw", True))

    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    rospy.sleep(1.5)

    # ---- PERSISTENT SUBSCRIBER, NOT wait_for_message ------------------------
    # This is not a style choice. docs/SOLVED.md ("every Phase-1 timelapse was
    # byte-identical"): RTAB-Map's grid publisher goes SILENT after its first,
    # near-empty publish unless a subscriber stays connected. map_manager.py
    # spawned a fresh map_saver per interval - connect, grab, disconnect - and
    # got one frozen near-empty grid for an entire multi-minute run. It looked
    # like a mapping failure and was not.
    #
    # rospy.wait_for_message() IS that same connect-grab-disconnect cycle. Using
    # it here would sample the first near-empty publish and report it as the
    # result, which for this test means reporting "all unknown" no matter what
    # the camera actually saw. So: subscribe once, hold it, and keep the latest.
    state = {"msg": None, "count": 0}

    def _cb(m):
        state["msg"] = m
        state["count"] += 1

    sub = rospy.Subscriber(topic, OccupancyGrid, _cb, queue_size=1)

    settle = float(rospy.get_param("~settle", 6.0))
    timeout = float(rospy.get_param("~timeout", 30.0))
    t0 = rospy.Time.now()
    last_sig, stable_since = None, None
    while not rospy.is_shutdown():
        el = (rospy.Time.now() - t0).to_sec()
        m = state["msg"]
        if m is not None:
            # Settle on content, not on a timer: a static robot adds one node
            # (RGBD/LinearUpdate 0.05 m) and the grid then stops changing, so
            # waiting a fixed period would just be dead time; but a grid still
            # filling in must not be sampled mid-growth.
            sig = (m.info.width, m.info.height, hash(bytes(bytearray(
                (v + 1) & 0xFF for v in m.data[::97]))))
            if sig != last_sig:
                last_sig, stable_since = sig, rospy.Time.now()
            elif (rospy.Time.now() - stable_since).to_sec() >= settle:
                break
        if el > timeout:
            if m is None:
                print("  NO MAP on %s within %.0f s." % (topic, timeout))
                print("  Is rtabmap running, and has it published a grid yet? A")
                print("  grid appears only once a node has been added, and with")
                print("  RGBD/LinearUpdate 0.05 a perfectly static robot adds")
                print("  exactly ONE. Nudge the robot a few cm if nothing comes.")
                sys.exit(2)
            print("  !! grid still changing after %.0fs - sampling anyway." % timeout)
            break
        rospy.sleep(0.2)

    grid = state["msg"]
    print("  publishes seen: %d  (a single publish that never repeats is the"
          % state["count"])
    print("                     documented silent-publisher symptom - see")
    print("                     docs/SOLVED.md. One IS normal for a static")
    print("                     robot, so judge it together with the counts.)")

    g_frame = grid.header.frame_id
    res = grid.info.resolution
    W, H = grid.info.width, grid.info.height
    ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
    cells = np.array(grid.data, dtype=np.int16).reshape(H, W)

    print("  map        : %s  %dx%d cells @ %.3f m  origin (%.3f, %.3f)"
          % (g_frame, W, H, res, ox, oy))
    print("  whole map  : occupied %d | free %d | unknown %d"
          % (int((cells > OCC_THRESHOLD).sum()),
             int(((cells >= 0) & (cells <= OCC_THRESHOLD)).sum()),
             int((cells < 0).sum())))

    # ---- put the rectangle's four corners into the grid frame ----------------
    corners = [(x_min, y_min), (x_max, y_min), (x_max, y_max), (x_min, y_max)]
    if frame != g_frame:
        try:
            tr = buf.lookup_transform(g_frame, frame, rospy.Time(0),
                                      rospy.Duration(5.0))
        except Exception as exc:                       # noqa: BLE001
            print("  NO TRANSFORM %s -> %s : %s" % (frame, g_frame, exc))
            print("  Pass _frame:=%s to give the rectangle in map coordinates"
                  % g_frame)
            sys.exit(2)
        t = tr.transform.translation
        yaw = quat_yaw(tr.transform.rotation)
        c, s = math.cos(yaw), math.sin(yaw)
        corners = [(t.x + c * x - s * y, t.y + s * x + c * y)
                   for (x, y) in corners]
        print("  region     : given in %s, yaw %+.2f deg into %s"
              % (frame, math.degrees(yaw), g_frame))
    else:
        print("  region     : given directly in %s" % g_frame)

    xs = [p[0] for p in corners]
    ys = [p[1] for p in corners]
    # Axis-aligned bounding box of the transformed corners. If the frames are
    # rotated this is a superset of the true rectangle — stated rather than
    # silently ignored, because it would inflate the counts.
    if frame != g_frame and abs(math.degrees(quat_yaw(tr.transform.rotation))) > 1.0:
        print("  !! frames are rotated; sampling the bounding box, which is")
        print("     LARGER than the rectangle you asked for.")

    i0 = int(math.floor((min(xs) - ox) / res))
    i1 = int(math.ceil((max(xs) - ox) / res))
    j0 = int(math.floor((min(ys) - oy) / res))
    j1 = int(math.ceil((max(ys) - oy) / res))
    ci0, ci1 = max(0, i0), min(W, i1)
    cj0, cj1 = max(0, j0), min(H, j1)

    print("  rectangle  : x %.2f..%.2f  y %.2f..%.2f m  ->  %d x %d cells"
          % (min(xs), max(xs), min(ys), max(ys), i1 - i0, j1 - j0))
    if (ci1 - ci0) <= 0 or (cj1 - cj0) <= 0:
        print("  THE REGION IS ENTIRELY OUTSIDE THE MAP. The map spans")
        print("  x %.2f..%.2f, y %.2f..%.2f m."
              % (ox, ox + W * res, oy, oy + H * res))
        sys.exit(2)
    if (ci0, ci1, cj0, cj1) != (i0, i1, j0, j1):
        print("  !! clipped to the map's edge — part of the region is off-map,")
        print("     and off-map is NOT the same as unknown. Counts below cover")
        print("     the clipped area only.")

    sub = cells[cj0:cj1, ci0:ci1]
    occ = int((sub > OCC_THRESHOLD).sum())
    free = int(((sub >= 0) & (sub <= OCC_THRESHOLD)).sum())
    unk = int((sub < 0).sum())
    tot = sub.size

    print()
    print("  ===== %s =====" % label)
    print("  OCCUPIED : %5d  (%5.1f %%)   the surface registered" % (occ, 100.0 * occ / tot))
    print("  FREE     : %5d  (%5.1f %%)   asserted DRIVABLE - the dangerous one"
          % (free, 100.0 * free / tot))
    print("  UNKNOWN  : %5d  (%5.1f %%)   never observed" % (unk, 100.0 * unk / tot))
    print("  total    : %5d cells" % tot)
    print()
    if free > occ and free > unk:
        print("  >> The region is mostly FREE. If a solid surface is standing")
        print("     there, the map is asserting you can drive through it. Check")
        print("     Grid/RayTracing - rays to farther returns carve near cells.")
    elif unk > occ and unk > free:
        print("  >> Mostly UNKNOWN: no depth came back and nothing carved it.")
        print("     Safer than FREE, but the surface is still invisible.")

    if draw:
        print()
        print("  # occupied   . free   ? unknown      (top of map = +y = robot's left)")
        for row in range(sub.shape[0] - 1, -1, -1):
            line = "".join("#" if v > OCC_THRESHOLD else ("?" if v < 0 else ".")
                           for v in sub[row])
            print("    " + line)

    if csv:
        new = not os.path.exists(csv)
        d = os.path.dirname(csv)
        if d and not os.path.isdir(d):
            os.makedirs(d)
        with open(csv, "a") as fh:
            if new:
                fh.write("label,stamp,occupied,free,unknown,total,"
                         "x_min,x_max,y_min,y_max,frame,resolution\n")
            fh.write("%s,%.3f,%d,%d,%d,%d,%.3f,%.3f,%.3f,%.3f,%s,%.3f\n"
                     % (label, grid.header.stamp.to_sec(), occ, free, unk, tot,
                        x_min, x_max, y_min, y_max, frame, res))
        print()
        print("  appended to %s" % csv)


if __name__ == "__main__":
    main()
