#!/usr/bin/env python3
"""depth_vs_cloud.py — does the point cloud keep what the depth image saw?

THE QUESTION

    board_geometry.py reads the DEPTH IMAGE and reports every board at 100%
    valid pixels. A first look at the POINT CLOUD suggested one board was
    largely absent. Both cannot be true, and it matters a great deal which is:
    the 3D map, and everything built on it, consumes the cloud, not the image.

WHY THIS COMPARISON IS EXACT

    The ZED publishes an ORGANIZED cloud - 1200 x 1920, one point per pixel,
    the same shape as the depth image. So pixel (r,c) and cloud point (r,c) are
    the same measurement. Counting them against each other needs no coordinate
    frames, no projection, and no assumption about which way y points.

    That matters because an earlier attempt at this compared a NaN-stripped
    point list against image windows by projecting through a guessed frame
    convention, got the convention wrong, and reported that the camera had seen
    nothing when it had seen 1.2 million points.

    Plain terms: instead of asking "are there points roughly where the board
    should be", this asks "for this exact pixel, did the depth image have a
    number, and did the cloud". Same pixel, same instant, no interpretation.

WHAT IT REPORTS

    depth valid     % of the board's pixels with a usable depth value
    cloud valid     % of the SAME pixels with a usable point
    dropped         pixels the depth image had and the cloud does not.
                    Anything much above zero means the cloud is discarding
                    real measurements, and the 3D map never sees that surface.
    depth quantiles p50 / p90 / p99 of the range over the board, as the
                    research notes ask for - rms alone hides the tail that
                    actually plants a phantom obstacle in the map.

    usage:
      rosrun sidewalk_evaluation depth_vs_cloud.py _tape_m:=3.1242 \\
             _label:=NEURAL _out:=~/bench_2026-08-30
"""
import csv as csvmod
import math
import os

import message_filters
import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import CameraInfo, Image, PointCloud2

CAM_H, CAM_PITCH, BOARD_H = 0.6953, 0.0565, 0.99


def main():
    rospy.init_node("depth_vs_cloud", anonymous=True, disable_signals=True)
    tape = float(rospy.get_param("~tape_m", 3.1242))
    label = str(rospy.get_param("~label", "run"))
    out = os.path.expanduser(str(rospy.get_param("~out", "~/bench")))
    nb = int(rospy.get_param("~boards", 3))
    names = str(rospy.get_param("~names", "left,middle,right")).split(",")
    npairs = int(rospy.get_param("~pairs", 10))
    if not os.path.isdir(out):
        os.makedirs(out)

    ci = rospy.wait_for_message(
        "/zedx_front/zed_node/depth/camera_info", CameraInfo, timeout=20)
    fx, fy, cx, cy = ci.K[0], ci.K[4], ci.K[2], ci.K[5]

    def row_at(h_m, rng):
        dz = CAM_H - h_m
        zc = rng * math.cos(CAM_PITCH) + dz * math.sin(CAM_PITCH)
        yc = -rng * math.sin(CAM_PITCH) + dz * math.cos(CAM_PITCH)
        return int(round(cy + fy * yc / zc))

    r_top, r_bot = row_at(0.86 * BOARD_H, tape), row_at(0.10 * BOARD_H, tape)

    got = {"pairs": []}

    def cb(dmsg, cmsg):
        if len(got["pairs"]) >= npairs:
            return
        dt = abs((dmsg.header.stamp - cmsg.header.stamp).to_sec())
        got["pairs"].append((dmsg, cmsg, dt))

    ds = message_filters.Subscriber(
        "/zedx_front/zed_node/depth/depth_registered", Image)
    cs = message_filters.Subscriber(
        "/zedx_front/zed_node/point_cloud/cloud_registered", PointCloud2)
    sync = message_filters.ApproximateTimeSynchronizer(
        [ds, cs], queue_size=20, slop=0.02)
    sync.registerCallback(cb)

    t0 = rospy.Time.now()
    while len(got["pairs"]) < npairs and (rospy.Time.now() - t0).to_sec() < 60:
        rospy.sleep(0.1)
    if not got["pairs"]:
        print("  NO SYNCHRONISED PAIRS in 60 s. Is the camera publishing both?")
        return
    print("  %d matched depth/cloud pairs, worst stamp difference %.1f ms"
          % (len(got["pairs"]),
             1000 * max(p[2] for p in got["pairs"])))

    # ---- locate the board wall once, from the first frame -------------------
    d0 = np.frombuffer(got["pairs"][0][0].data, dtype=np.float32).reshape(
        got["pairs"][0][0].height, got["pairs"][0][0].width)
    H, W = d0.shape
    mid = (r_top + r_bot) // 2
    band = d0[max(0, mid - 40):mid + 40, :]
    cut = tape + 0.75
    good = np.zeros(W, bool)
    for c in range(W):
        col = band[:, c]
        ok = np.isfinite(col) & (col > 1.2) & (col < 10.0)
        if ok.sum() < 30:
            continue
        v = col[ok]
        p10, p90 = np.percentile(v, [10, 90])
        good[c] = (np.median(v) < cut) and (p90 - p10 < 1.2)
    runs, s = [], None
    for c in range(W):
        if good[c] and s is None:
            s = c
        elif not good[c] and s is not None:
            if c - s > 200:
                runs.append((s, c))
            s = None
    if s is not None and W - s > 200:
        runs.append((s, W))
    if not runs:
        print("  no board wall found")
        return
    C0, C1 = max(runs, key=lambda r: r[1] - r[0])
    w3 = (C1 - C0) / float(nb)
    print("  wall at columns %d..%d, rows %d..%d  (%.3f m across at the tape "
          "range)" % (C0, C1, r_top, r_bot, (C1 - C0) * tape / fx))
    print()

    print("  board          pixels   depth valid   cloud valid   DROPPED     "
          "range p50 / p90 / p99")
    rows = []
    for i in range(nb):
        nm = names[i] if i < len(names) else "board%d" % i
        a, b = int(C0 + i * w3), int(C0 + (i + 1) * w3)
        u0, u1 = a + int(0.10 * w3), b - int(0.10 * w3)
        dv, cv, dropped, npx = [], [], [], 0
        q = []
        for dmsg, cmsg, _ in got["pairs"]:
            d = np.frombuffer(dmsg.data, dtype=np.float32).reshape(
                dmsg.height, dmsg.width)[r_top:r_bot, u0:u1]
            # organized cloud: read x,y,z in image order and reshape to match
            pts = np.frombuffer(cmsg.data, dtype=np.uint8).reshape(
                cmsg.height, cmsg.width, cmsg.point_step)[r_top:r_bot, u0:u1, :]
            # z of the camera-frame point is at byte offset 8; but validity is
            # what matters and any NaN in x/y/z marks an unusable point.
            px = pts[:, :, 0:4].copy().view(np.float32).reshape(d.shape)
            d_ok = np.isfinite(d) & (d > 1.2) & (d < 10.0)
            c_ok = np.isfinite(px)
            npx = d.size
            dv.append(100.0 * d_ok.sum() / d.size)
            cv.append(100.0 * c_ok.sum() / d.size)
            dropped.append(100.0 * (d_ok & ~c_ok).sum() / max(1, d_ok.sum()))
            q.append(np.percentile(d[d_ok], [50, 90, 99]) if d_ok.any()
                     else [np.nan] * 3)
        q = np.array(q)
        print("  %-13s %7d   %6.2f %%      %6.2f %%     %6.2f %%    "
              "%.3f / %.3f / %.3f m"
              % (nm, npx, np.mean(dv), np.mean(cv), np.mean(dropped),
                 np.mean(q[:, 0]), np.mean(q[:, 1]), np.mean(q[:, 2])))
        rows.append(dict(label=label, method="depth_vs_cloud_v1", board=nm,
                         pixels=npx, depth_valid_pct=round(float(np.mean(dv)), 3),
                         cloud_valid_pct=round(float(np.mean(cv)), 3),
                         dropped_pct=round(float(np.mean(dropped)), 3),
                         range_p50_m=round(float(np.mean(q[:, 0])), 4),
                         range_p90_m=round(float(np.mean(q[:, 1])), 4),
                         range_p99_m=round(float(np.mean(q[:, 2])), 4),
                         tape_m=tape, cols="%d-%d" % (a, b),
                         rows_px="%d-%d" % (r_top, r_bot),
                         pairs=len(got["pairs"])))
    if rows:
        p = os.path.join(out, "depth_vs_cloud.csv")
        new = not os.path.exists(p)
        with open(p, "a", newline="") as f:
            w = csvmod.DictWriter(f, fieldnames=list(rows[0].keys()))
            if new:
                w.writeheader()
            w.writerows(rows)
        print("\n  appended to %s" % p)
        worst = max(r["dropped_pct"] for r in rows)
        if worst < 1.0:
            print("  >> The cloud keeps essentially everything the depth image")
            print("     has. Any earlier claim that a board was missing from the")
            print("     cloud was an artefact of how it was analysed, not the data.")
        else:
            print("  >> The cloud DISCARDS up to %.1f %% of what the depth image"
                  % worst)
            print("     measured. Everything built on the cloud - the 3D map, the")
            print("     occupancy grid - never sees those returns.")


if __name__ == "__main__":
    main()
