#!/usr/bin/env python3
"""grid_span.py — does the map draw the obstacle at its real WIDTH, unbroken?

WHY THIS EXISTS, AND WHY CELL COUNTS WERE NOT ENOUGH

    We were counting occupied cells inside each board's footprint. A count can
    look healthy while the map is useless: 40 cells clustered at one end and
    nothing across the middle counts the same as 40 cells spread evenly, but
    only one of those is a wall a robot would refuse to drive into.

    The user's test is the right one and this measures it directly:

        the three boards are 94 inches (2.388 m) end to end, so the map should
        show an unbroken occupied line 2.388 m wide.

    Plain terms: stand back and look at the map from above. Is there a solid
    dark line where the boards are, and is it as long as the boards really are?
    That is the whole question. Width and holes, not totals.

WHAT IT REPORTS, and what each number means for driving

    span            distance from the leftmost occupied cell to the rightmost.
                    Too short = the map does not know part of the wall exists.
    holes           columns inside that span with NO occupied cell anywhere.
                    Each one is a gap the planner may try to drive through.
    longest hole    the worst single gap. A robot 0.67 m wide needs about 14
                    contiguous empty cells to attempt a crossing, so one long
                    hole is far more dangerous than many short ones.
    free-in-holes   of those holes, how many are positively marked FREE rather
                    than unknown. This is the difference between the map saying
                    "I don't know what is there" and "I checked, drive on".
    depth extent    how far the occupied cells stretch front-to-back. A vertical
                    board projects to a thin line; a LEANING board projects to a
                    thick band, so this number reveals a lean without needing
                    the camera's depth image at all.

    usage:
      rosrun sidewalk_slam grid_span.py _expect_m:=2.388 _label:=r09n5 \\
             _x_min:=2.85 _x_max:=3.95 _y_min:=-1.90 _y_max:=1.20 \\
             _frame:=base_link _csv:=~/bench/span.csv
"""
import csv as csvmod
import math
import os
import sys

import numpy as np
import rospy
import tf2_ros
from nav_msgs.msg import OccupancyGrid

# RTAB-Map publishes 0 = free, 100 = occupied, -1 = unknown. Strictly above this
# counts as occupied. The middle band is kept OUT of "free" deliberately: a
# probabilistic cell that is merely leaning occupied must not be reported as a
# hole, because that would overstate the failure.
OCC_THRESHOLD = 50


def quat_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wait_for_settled_grid(topic, settle, timeout):
    """Hold the subscription open. See grid_sample.py for why this is not
    wait_for_message: RTAB-Map's grid publisher goes silent after one near-empty
    publish unless a subscriber stays connected, and a connect-grab-disconnect
    would sample exactly that near-empty grid and report it as the answer."""
    state = {"msg": None, "count": 0}

    def _cb(m):
        state["msg"] = m
        state["count"] += 1

    rospy.Subscriber(topic, OccupancyGrid, _cb, queue_size=1)
    t0 = rospy.Time.now()
    last_sig, stable_since = None, None
    while not rospy.is_shutdown():
        m = state["msg"]
        if m is not None:
            sig = (m.info.width, m.info.height,
                   hash(bytes(bytearray((v + 1) & 0xFF for v in m.data[::97]))))
            if sig != last_sig:
                last_sig, stable_since = sig, rospy.Time.now()
            elif (rospy.Time.now() - stable_since).to_sec() >= settle:
                break
        if (rospy.Time.now() - t0).to_sec() > timeout:
            if m is None:
                print("  NO MAP on %s within %.0f s. Is rtabmap running, and has"
                      % (topic, timeout))
                print("  it published a grid? A static robot adds exactly one"
                      " node, so nudge it if nothing arrives.")
                sys.exit(2)
            print("  !! grid still changing after %.0f s - sampling anyway."
                  % timeout)
            break
        rospy.sleep(0.2)
    return state["msg"], state["count"]


def main():
    rospy.init_node("grid_span", anonymous=True, disable_signals=True)
    topic = rospy.get_param("~topic", "/rtabmap/grid_map")
    frame = rospy.get_param("~frame", "base_link")
    label = str(rospy.get_param("~label", "unlabelled"))
    method = str(rospy.get_param("~method", "grid_span_v1"))
    csv_path = os.path.expanduser(str(rospy.get_param("~csv", "")))
    expect = float(rospy.get_param("~expect_m", 2.388))   # 94 inches, measured

    # A generous depth window on purpose. A board leaning back by A degrees
    # projects over H*sin(A) of depth - 0.99 m of board tipped 40 deg spreads
    # over 0.64 m - so a narrow window would clip a leaning board's own cells
    # and report holes that are really just outside the box.
    x_min = float(rospy.get_param("~x_min", 2.85))
    x_max = float(rospy.get_param("~x_max", 3.95))
    y_min = float(rospy.get_param("~y_min", -1.90))
    y_max = float(rospy.get_param("~y_max", 1.20))

    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    rospy.sleep(1.5)

    grid, npub = wait_for_settled_grid(
        topic, float(rospy.get_param("~settle", 6.0)),
        float(rospy.get_param("~timeout", 30.0)))

    g_frame = grid.header.frame_id
    res = grid.info.resolution
    W, H = grid.info.width, grid.info.height
    ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
    cells = np.array(grid.data, dtype=np.int16).reshape(H, W)
    print("  map: %s %dx%d @ %.3f m | publishes seen %d" % (g_frame, W, H, res, npub))

    # ---- the window's four corners into the grid frame ----------------------
    corners = [(x_min, y_min), (x_max, y_min), (x_max, y_max), (x_min, y_max)]
    yaw = 0.0
    if frame != g_frame:
        try:
            tr = buf.lookup_transform(g_frame, frame, rospy.Time(0),
                                      rospy.Duration(5.0))
        except Exception as exc:                                # noqa: BLE001
            print("  NO TRANSFORM %s -> %s : %s" % (frame, g_frame, exc))
            sys.exit(2)
        t = tr.transform.translation
        yaw = quat_yaw(tr.transform.rotation)
        c, s = math.cos(yaw), math.sin(yaw)
        corners = [(t.x + c * x - s * y, t.y + s * x + c * y)
                   for (x, y) in corners]
        if abs(math.degrees(yaw)) > 1.0:
            print("  NOTE: %s is rotated %+.1f deg from %s, so the window is the"
                  % (frame, math.degrees(yaw), g_frame))
            print("        bounding box of the rotated rectangle - slightly larger.")

    i0 = max(0, int(math.floor((min(p[0] for p in corners) - ox) / res)))
    i1 = min(W, int(math.ceil((max(p[0] for p in corners) - ox) / res)))
    j0 = max(0, int(math.floor((min(p[1] for p in corners) - oy) / res)))
    j1 = min(H, int(math.ceil((max(p[1] for p in corners) - oy) / res)))
    if i1 <= i0 or j1 <= j0:
        print("  window falls outside the map."); sys.exit(2)
    sub = cells[j0:j1, i0:i1]

    # ---- collapse front-to-back: for each sideways column, is anything there?
    occ = sub > OCC_THRESHOLD
    unk = sub < 0
    col_occ = occ.any(axis=1)                    # one entry per sideways column
    col_all_free = (~occ).all(axis=1) & (~unk).all(axis=1)

    if not col_occ.any():
        print("  NOTHING OCCUPIED anywhere in the window. The map has no wall here.")
        span = 0.0
        first = last = 0
        holes = int(len(col_occ)); longest = holes; free_holes = int(col_all_free.sum())
    else:
        idx = np.where(col_occ)[0]
        first, last = int(idx[0]), int(idx[-1])
        span = (last - first + 1) * res
        inside = col_occ[first:last + 1]
        holes = int((~inside).sum())
        free_holes = int(col_all_free[first:last + 1].sum())
        longest, run = 0, 0
        for v in inside:
            run = 0 if v else run + 1
            longest = max(longest, run)

    # depth extent of the occupied cells - reveals a lean without using depth
    if occ.any():
        rows = np.where(occ.any(axis=0))[0]
        depth_near = ox + (i0 + int(rows[0])) * res
        depth_far = ox + (i0 + int(rows[-1]) + 1) * res
        depth_extent = depth_far - depth_near
    else:
        depth_near = depth_far = depth_extent = 0.0

    print()
    print("  === THE WALL, AS THE MAP DRAWS IT ===")
    print("  span of occupied cells   %.3f m   (expected %.3f m = %.0f in)"
          % (span, expect, expect / 0.0254))
    print("  difference               %+.3f m  (%+.1f %%)"
          % (span - expect, 100.0 * (span - expect) / expect if expect else 0.0))
    print("  holes inside the span    %d columns of %d   (longest run %d = %.2f m)"
          % (holes, last - first + 1 if col_occ.any() else 0, longest, longest * res))
    print("  of those, marked FREE    %d   <- the map asserts these are drivable"
          % free_holes)
    print("  depth spread of the wall %.3f m   (a vertical board gives ~1-2 cells;"
          % depth_extent)
    print("                                     more than that means it leans)")

    # ---- top-down picture, collapsed to one row per sideways column ---------
    #
    # ORIENTATION, and why it is reversed here. In ROS (REP-103) x is forward
    # and y is LEFT, so a higher grid column index means further to the robot's
    # left. Printed in index order that puts the robot's left on the RIGHT of
    # the page - the mirror image of what you see standing behind the robot.
    # An earlier version printed it that way and labelled it "left on the left",
    # which would have put a hole on the wrong board. Reversed so the picture
    # matches the view.
    print()
    print("  looking down, as if standing behind the robot:")
    line = []
    for k in range(len(col_occ)):
        line.append("#" if col_occ[k] else ("." if col_all_free[k] else "?"))
    s = "".join(reversed(line))
    y_left = y_max
    y_right = y_min
    for k in range(0, len(s), 100):
        print("    " + s[k:k + 100])
    print("    left (y=%+.2f) %s right (y=%+.2f)"
          % (y_left, "-" * max(1, min(len(s), 100) - 26), y_right))
    print("    # occupied   . free (asserted drivable)   ? unknown")

    if csv_path:
        d = os.path.dirname(csv_path)
        if d and not os.path.isdir(d):
            os.makedirs(d)
        new = not os.path.exists(csv_path)
        with open(csv_path, "a", newline="") as f:
            w = csvmod.writer(f)
            if new:
                w.writerow(["label", "method", "stamp", "span_m", "expect_m",
                            "diff_m", "holes", "longest_hole_cells",
                            "free_holes", "depth_extent_m", "depth_near_m",
                            "resolution", "frame", "x_min", "x_max",
                            "y_min", "y_max"])
            w.writerow([label, method, "%.3f" % grid.header.stamp.to_sec(),
                        "%.3f" % span, "%.3f" % expect, "%+.3f" % (span - expect),
                        holes, longest, free_holes, "%.3f" % depth_extent,
                        "%.3f" % depth_near, res, frame,
                        x_min, x_max, y_min, y_max])
        print("\n  appended to %s" % csv_path)


if __name__ == "__main__":
    main()
