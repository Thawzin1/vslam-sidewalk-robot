#!/usr/bin/env python3
"""save_map_artifacts.py — store what a test run actually produced.

WHY THIS EXISTS

    Through 2026-08-30 we recorded seven mapping conditions and measured all of
    them, but stored only the MEASUREMENTS - spans, cell counts, text pictures.
    The maps themselves were never written to disk. When you later want to look
    at what condition 3 actually built, or put it in a report, there is nothing
    to look at.

    Plain terms: we wrote down the score of every match and kept no footage.

    This saves, for one run, everything a reader would want later:

      <label>_grid.pgm / .yaml   the 2D map in the standard ROS format, so it
                                 opens in any mapping tool and can be reloaded
                                 for navigation
      <label>_grid.png           the same map as a picture, colour-coded, so it
                                 can go straight into a document
      <label>_grid.npy           the raw cell values, for re-analysis without
                                 re-running the robot
      <label>_cloud.ply          the 3D point cloud, which opens in MeshLab,
                                 CloudCompare, Blender, or the offline viewer
                                 in docs/bench_*/
      <label>_meta.txt           every setting the run used, so the numbers can
                                 be traced back to the configuration

WHY IT DOES NOT USE wait_for_message
    RTAB-Map's publishers go quiet after one near-empty publish unless a
    subscriber stays connected (docs/SOLVED.md). A connect-grab-disconnect grabs
    that first empty publish and saves an empty map that looks like a failed run.

    usage:
      rosrun sidewalk_slam save_map_artifacts.py _label:=NEURAL_full_stack \\
             _out:=~/bench_2026-08-30/maps
"""
import os
import struct
import sys

import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

try:
    import cv2
except ImportError:                                         # pragma: no cover
    cv2 = None

OCC = 50


def hold(topic, msg_type, settle, timeout, what):
    """Subscribe and keep the latest until it stops changing."""
    state = {"m": None, "n": 0}

    def _cb(m):
        state["m"] = m
        state["n"] += 1

    rospy.Subscriber(topic, msg_type, _cb, queue_size=1)
    t0 = rospy.Time.now()
    last, since = None, None
    while not rospy.is_shutdown():
        m = state["m"]
        if m is not None:
            sig = len(m.data)
            if sig != last:
                last, since = sig, rospy.Time.now()
            elif (rospy.Time.now() - since).to_sec() >= settle:
                break
        if (rospy.Time.now() - t0).to_sec() > timeout:
            break
        rospy.sleep(0.2)
    if state["m"] is None:
        print("  %-12s NOTHING on %s within %.0f s" % (what, topic, timeout))
    else:
        print("  %-12s got it (%d publishes seen)" % (what, state["n"]))
    return state["m"]


def save_ply(path, pts, cols):
    """Binary PLY. Written by hand so there is no dependency on open3d or
    plyfile, neither of which is installed on this Jetson."""
    n = len(pts)
    with open(path, "wb") as f:
        f.write(b"ply\nformat binary_little_endian 1.0\n")
        f.write(("element vertex %d\n" % n).encode())
        f.write(b"property float x\nproperty float y\nproperty float z\n")
        if cols is not None:
            f.write(b"property uchar red\nproperty uchar green\n"
                    b"property uchar blue\n")
        f.write(b"end_header\n")
        for i in range(n):
            f.write(struct.pack("<fff", *pts[i]))
            if cols is not None:
                f.write(struct.pack("<BBB", *cols[i]))
    return n


def main():
    rospy.init_node("save_map_artifacts", anonymous=True, disable_signals=True)
    label = str(rospy.get_param("~label", "run"))
    out = os.path.expanduser(str(rospy.get_param("~out", "~/bench/maps")))
    settle = float(rospy.get_param("~settle", 5.0))
    timeout = float(rospy.get_param("~timeout", 40.0))
    cloud_topic = str(rospy.get_param("~cloud", "/rtabmap/cloud_map"))
    # The grid topic must be settable, not hardcoded. A node replayed from a
    # database with `rosrun rtabmap_slam rtabmap` publishes at the ROOT
    # (/grid_map), not under /rtabmap/. With the topic fixed at
    # /rtabmap/grid_map this script sat waiting on a name nothing published to,
    # reported "NOTHING", and looked like the replay had failed - while the map
    # was being published one name away.
    grid_topic = str(rospy.get_param("~grid", "/rtabmap/grid_map"))
    if not os.path.isdir(out):
        os.makedirs(out)
    base = os.path.join(out, label)

    # ---------------- 2D occupancy grid --------------------------------------
    g = hold(grid_topic, OccupancyGrid, settle, timeout, "2D grid")
    if g is not None:
        W, H = g.info.width, g.info.height
        res = g.info.resolution
        ox, oy = g.info.origin.position.x, g.info.origin.position.y
        cells = np.array(g.data, dtype=np.int16).reshape(H, W)
        np.save(base + "_grid.npy", cells)

        # Standard ROS map format: 254 free, 000 occupied, 205 unknown, and the
        # image is stored with row 0 at the TOP, which is the opposite of the
        # OccupancyGrid convention - hence the flip.
        pgm = np.full((H, W), 205, np.uint8)
        pgm[(cells >= 0) & (cells <= OCC)] = 254
        pgm[cells > OCC] = 0
        pgm = np.flipud(pgm)
        with open(base + "_grid.pgm", "wb") as f:
            f.write(b"P5\n# saved by save_map_artifacts.py\n")
            f.write(("%d %d\n255\n" % (W, H)).encode())
            f.write(pgm.tobytes())
        with open(base + "_grid.yaml", "w") as f:
            f.write("image: %s_grid.pgm\n" % label)
            f.write("resolution: %.6f\n" % res)
            f.write("origin: [%.6f, %.6f, 0.000000]\n" % (ox, oy))
            f.write("negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")

        if cv2 is not None:
            pic = np.zeros((H, W, 3), np.uint8)
            pic[cells < 0] = (60, 60, 60)                 # unknown  dark grey
            pic[(cells >= 0) & (cells <= OCC)] = (235, 235, 235)   # free
            pic[cells > OCC] = (30, 30, 200)              # occupied  red (BGR)
            pic = np.flipud(pic)
            # crop to what was actually mapped, so the picture is not 95% empty
            nz = np.argwhere(np.any(pic != 0, axis=2))
            if len(nz):
                r0, c0 = nz.min(0)
                r1, c1 = nz.max(0)
                pad = 10
                pic = pic[max(0, r0 - pad):r1 + pad, max(0, c0 - pad):c1 + pad]
            sc = max(1, int(700 / max(1, max(pic.shape[:2]))))
            pic = cv2.resize(pic, (pic.shape[1] * sc, pic.shape[0] * sc),
                             interpolation=cv2.INTER_NEAREST)
            cv2.imwrite(base + "_grid.png", pic)
        occ_n = int((cells > OCC).sum())
        print("       %dx%d @ %.3f m | occupied %d | free %d | unknown %d"
              % (W, H, res, occ_n,
                 int(((cells >= 0) & (cells <= OCC)).sum()),
                 int((cells < 0).sum())))

    # ---------------- 3D point cloud -----------------------------------------
    c = hold(cloud_topic, PointCloud2, settle, timeout, "3D cloud")
    if c is not None:
        fields = [f.name for f in c.fields]
        want = ("x", "y", "z", "rgb") if "rgb" in fields else ("x", "y", "z")
        raw = list(pc2.read_points(c, want, skip_nans=True))
        pts = np.array([[p[0], p[1], p[2]] for p in raw], np.float32)
        cols = None
        if "rgb" in want and len(raw):
            cols = []
            for p in raw:
                v = struct.unpack("<I", struct.pack("<f", p[3]))[0]
                cols.append(((v >> 16) & 255, (v >> 8) & 255, v & 255))
            cols = np.array(cols, np.uint8)
        n = save_ply(base + "_cloud.ply", pts, cols)
        print("       %d points%s -> %s_cloud.ply"
              % (n, " with colour" if cols is not None else "", label))

    # ---------------- the settings this run used -----------------------------
    with open(base + "_meta.txt", "w") as f:
        f.write("label: %s\n" % label)
        f.write("saved: %s\n" % rospy.Time.now().to_sec())
        f.write("cloud topic: %s\n\n" % cloud_topic)
        f.write("depth_mode: %s\n" % rospy.get_param(
            "/zedx_front/zed_node/depth/depth_mode", "?"))
        f.write("\n[rtabmap grid parameters, read live]\n")
        for k in ("Grid/Sensor", "Grid/CellSize", "Grid/RangeMax",
                  "Grid/DepthDecimation", "Grid/PreVoxelFiltering",
                  "Grid/NoiseFilteringRadius", "Grid/NoiseFilteringMinNeighbors",
                  "Grid/NormalsSegmentation", "Grid/NormalK",
                  "Grid/MaxGroundAngle", "Grid/MaxObstacleHeight",
                  "Grid/MinClusterSize", "Grid/ClusterRadius",
                  "Grid/RayTracing", "Grid/3D"):
            f.write("%-34s %s\n" % (k, rospy.get_param(
                "/rtabmap/rtabmap/" + k, "(unset)")))
    print("  meta         -> %s_meta.txt" % label)
    print("\n  all artefacts in %s" % out)


if __name__ == "__main__":
    main()
