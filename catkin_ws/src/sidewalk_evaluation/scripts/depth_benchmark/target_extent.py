#!/usr/bin/env python3
"""target_extent.py — measure a flat target's recovered size from the point cloud.

METHOD 1, "paper_extent" — Schulte-Tigges et al., Sensors 2022, 22, 7146,
Scenario 2 ("Static Square Meter Reference Plane"), section 3.3.

    Their procedure, applied unchanged: place a cut-out box around the target in
    the point cloud, discard every point outside it, and take the minimum and
    maximum along each axis. For a flat plane of known size the extents should
    reproduce it, and the depth extent should be zero:

        zAxis_max - zAxis_min  ~=  the target's height
        yAxis_max - yAxis_min  ~=  the target's width
        xAxis_max - xAxis_min  ~=  0

    Repeat over many frames; the values scatter around the truth and the spread
    is the measurement's own noise. The depth study used 1000 frames.

    NOTE WHAT THIS METHOD DELIBERATELY DOES NOT DO: it does not fit anything.
    The depth study introduced Scenario 2 precisely to avoid the RANSAC fitting used
    in Scenario 1, so that a result could not be an artefact of the fitter. Min
    and max are raw order statistics of the surviving points. Keep it that way —
    adding a fit here would reintroduce exactly what the scenario exists to rule
    out.

    ONE HONEST WEAKNESS, WHICH IS WHY METHOD 2 EXISTS: min and max are the two
    most outlier-sensitive statistics there are. A single stray point widens the
    reported extent, and a target that returns nothing at all in its middle
    still reports the full width if its two edges return. So this method can say
    "the width is right" about a surface with a hole in it.

METHOD 2, "profile_edges" — added here, reported separately, never mixed in.

    Bin the surviving points along the width and count them per bin. A surface
    that returns depth evenly gives a flat profile; one that does not gives a
    trough. The trough's own edges are then a measurement of where the material
    boundary lies, which is the quantity of interest when a white panel is set
    into a dark surround. Reported as its own row so the two methods can be
    compared rather than blended.

WHY BOTH, AND WHY SEPARATELY
    The instruction for this test is that each method's result is recorded for
    each run, and that a method is judged on the data rather than chosen in
    advance. Every row in the CSV therefore carries run_id, method and target,
    and no row is derived from another.

    usage:
      rosrun sidewalk_evaluation target_extent.py _run:=B2 _target:=white_board
      rosrun sidewalk_evaluation target_extent.py _run:=B2 _target:=wall _frames:=60
"""
import math
import os
import sys

import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2


# ---------------------------------------------------------------------------
# read 99 in = 2.5146 m from the board to the camera's FRONT LENS SURFACE, and
# the left lens sits 0.071 m ahead of base_footprint, so the board stands at
# 2.586 m plus the few millimetres from the front glass back to the optical
# centre — a distance the datasheet does not give. The boxes below are
# deliberately deep enough that this does not matter, exactly as the depth study's
# were (it used 60 cm in front of its plane and 80 cm behind).
# ---------------------------------------------------------------------------
BOARD_X = 2.586

TARGETS = {
    # name          truth (width_y, height_z, depth_x)   box (x0,x1, y0,y1, z0,z1)
    "white_board": ((0.79, 0.99, 0.0),
                    (BOARD_X - 0.30, BOARD_X + 0.40, -0.42, 0.42, 0.00, 1.10)),
    "wall":        ((2.08, 1.02, 0.0),
                    (BOARD_X - 0.30, BOARD_X + 0.40, -1.10, 1.10, 0.00, 1.10)),
}


def quat_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def read_xyz(msg, stride):
    """Raw-buffer read of x/y/z. A per-point Python loop over 2.3 million points
    takes minutes; this takes milliseconds."""
    off = {f.name: f.offset for f in msg.fields}
    n = len(msg.data) // msg.point_step
    buf = np.frombuffer(msg.data, dtype=np.uint8)[:n * msg.point_step]
    buf = buf.reshape(n, msg.point_step)[::stride]
    out = np.empty((buf.shape[0], 3), dtype=np.float32)
    for i, k in enumerate(("x", "y", "z")):
        o = off[k]
        out[:, i] = buf[:, o:o + 4].copy().view(np.float32).ravel()
    return out[np.isfinite(out).all(axis=1)]


def main():
    rospy.init_node("target_extent", anonymous=True, disable_signals=True)
    topic = rospy.get_param(
        "~cloud", "/zedx_front/zed_node/point_cloud/cloud_registered")
    ref = rospy.get_param("~frame", "base_footprint")
    run = str(rospy.get_param("~run", "unlabelled"))
    tname = str(rospy.get_param("~target", "white_board"))
    frames = int(rospy.get_param("~frames", 30))
    stride = int(rospy.get_param("~stride", 7))
    nbins = int(rospy.get_param("~bins", 40))
    csv = os.path.expanduser(str(rospy.get_param("~csv", "")))

    if tname not in TARGETS:
        print("  unknown target %r. Known: %s" % (tname, ", ".join(TARGETS)))
        sys.exit(2)
    (tw, th, td), (x0, x1, y0, y1, z0, z1) = TARGETS[tname]

    # The box may be overridden per axis. Two reasons this is not hardcoded-only:
    # the rig can move, and the reference frame can change - the presets above
    # are in base_footprint, whose z=0 IS the floor, but base_link sits 0.13228 m
    # higher, so the same wall spans -0.132..0.888 there. Overriding is the
    # honest way to work in a different frame; silently reusing floor-relative
    # bounds in a base_link measurement would clip the bottom of the target and
    # report a short height as if it were a finding.
    x0 = float(rospy.get_param("~x_min", x0)); x1 = float(rospy.get_param("~x_max", x1))
    y0 = float(rospy.get_param("~y_min", y0)); y1 = float(rospy.get_param("~y_max", y1))
    z0 = float(rospy.get_param("~z_min", z0)); z1 = float(rospy.get_param("~z_max", z1))
    if rospy.get_param("~preview", False):
        # Preview: ignore the box and report what the cloud contains in a wide
        # window, so the target can be located before any measurement is made.
        x0, x1, y0, y1, z0, z1 = 0.5, 6.0, -2.5, 2.5, -1.0, 2.5
        print("  PREVIEW MODE - box ignored, reporting the whole forward volume.")
        print("  Use this to find the target, NOT to measure it.")
        print()

    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    rospy.sleep(1.5)

    print("  target      : %s   truth  width %.3f  height %.3f  depth %.3f m"
          % (tname, tw, th, td))
    print("  cut-out box : x %.2f..%.2f  y %.2f..%.2f  z %.2f..%.2f  (in %s)"
          % (x0, x1, y0, y1, z0, z1, ref))
    print("  frames      : %d   (the depth study used 1000; the spread below says"
          % frames)
    print("                whether this many is enough)")
    print()

    rows, prof_sum, kept = [], np.zeros(nbins), 0
    frame_pts = []          # cropped points, kept for the locating histograms
    for i in range(frames):
        try:
            msg = rospy.wait_for_message(topic, PointCloud2, timeout=15)
        except rospy.ROSException:
            print("  NO CLOUD on %s. Is the camera running?" % topic)
            sys.exit(2)
        try:
            tr = buf.lookup_transform(ref, msg.header.frame_id,
                                      rospy.Time(0), rospy.Duration(4.0))
        except Exception as exc:                        # noqa: BLE001
            print("  NO TRANSFORM %s -> %s : %s" % (msg.header.frame_id, ref, exc))
            sys.exit(2)

        R = quat_to_matrix(tr.transform.rotation)
        t = np.array([tr.transform.translation.x,
                      tr.transform.translation.y,
                      tr.transform.translation.z])
        p = read_xyz(msg, stride).astype(np.float64).dot(R.T) + t

        m = ((p[:, 0] > x0) & (p[:, 0] < x1) &
             (p[:, 1] > y0) & (p[:, 1] < y1) &
             (p[:, 2] > z0) & (p[:, 2] < z1))
        sel = p[m]
        if len(sel) < 20:
            continue
        kept += 1
        rows.append((sel[:, 1].max() - sel[:, 1].min(),      # width  (y)
                     sel[:, 2].max() - sel[:, 2].min(),      # height (z)
                     sel[:, 0].max() - sel[:, 0].min(),      # depth  (x)
                     sel[:, 0].mean(), len(sel)))
        h, _ = np.histogram(sel[:, 1], bins=nbins, range=(y0, y1))
        prof_sum += h
        if len(frame_pts) < 3:      # a few frames is plenty to locate a surface
            frame_pts.append(sel)
        rospy.sleep(0.05)

    if kept < 3:
        print("  ONLY %d USABLE FRAMES. Either the box is in the wrong place or" % kept)
        print("  the target returns almost nothing. Widen the box first, and if")
        print("  that does not help, that emptiness IS the result - record it.")
        sys.exit(1)

    a = np.array([r[:4] for r in rows])
    med = np.median(a, axis=0)
    sd = a.std(axis=0)
    npts = int(np.median([r[4] for r in rows]))

    print("  == METHOD 1: paper_extent  (Sensors 2022, 22, 7146, section 3.3) ==")
    print("  frames used %d/%d, median %d points in the box" % (kept, frames, npts))
    print()
    print("                measured            truth      error")
    print("    width  (y)  %.3f +/- %.3f m     %.3f     %+.3f m"
          % (med[0], sd[0], tw, med[0] - tw))
    print("    height (z)  %.3f +/- %.3f m     %.3f     %+.3f m"
          % (med[1], sd[1], th, med[1] - th))
    print("    depth  (x)  %.3f +/- %.3f m     %.3f     %+.3f m   <- a plane has none"
          % (med[2], sd[2], td, med[2] - td))
    print("    distance    %.3f m to the surface" % med[3])
    print()
    print("  Min and max are the most outlier-sensitive statistics there are, so")
    print("  a correct width here does NOT prove the surface returned evenly.")
    print("  Method 2 is what tests that.")
    print()

    # ---- locating diagnostic, printed before the methods are believed --------
    # A cut-out box is only meaningful once it contains ONE surface. The depth
    # extent above is the test: a flat target should give ~0, and anything much
    # larger means the box spans more than one thing, which makes every other
    # number in this report a mixture. These histograms say where the surfaces
    # actually are, so the box can be placed rather than guessed.
    if rospy.get_param("~locate", True):
        allp = np.vstack([r for r in frame_pts]) if frame_pts else np.zeros((0, 3))
        print("  -- WHERE THE POINTS ARE (place the box from this) --")
        for ax, lo, hi, nm in ((0, x0, x1, "depth  x"), (2, z0, z1, "height z")):
            h, edges = np.histogram(allp[:, ax], bins=24, range=(lo, hi))
            pk = h.max() if h.max() else 1
            print("    %s:" % nm)
            for b in range(len(h) - 1, -1, -1) if ax == 2 else range(len(h)):
                print("      %+.3f m | %-32s %6d"
                      % ((edges[b] + edges[b + 1]) / 2,
                         "#" * int(round(32.0 * h[b] / pk)), h[b]))
        print()

    prof = prof_sum / max(kept, 1)
    print("  == METHOD 2: profile_edges  (added here, reported separately) ==")
    print("  points per bin across the width, averaged over %d frames:" % kept)
    peak = prof.max() if prof.max() > 0 else 1.0
    bw = (y1 - y0) / nbins
    for b in range(nbins - 1, -1, -1):
        yc = y0 + (b + 0.5) * bw
        bar = "#" * int(round(40.0 * prof[b] / peak))
        print("    y %+.3f m | %-40s %5.0f" % (yc, bar, prof[b]))

    # A trough is any run of bins under a fraction of the profile's own peak.
    thr = float(rospy.get_param("~trough_frac", 0.25)) * peak
    low = prof < thr
    best, cur = (0, 0, 0), None
    for b in range(nbins):
        if low[b]:
            cur = b if cur is None else cur
            if b - cur + 1 > best[0]:
                best = (b - cur + 1, cur, b)
        else:
            cur = None
    print()
    if best[0] >= 2:
        e0 = y0 + best[1] * bw
        e1 = y0 + (best[2] + 1) * bw
        print("  TROUGH: %d bins under %.0f%% of peak, spanning y %+.3f .. %+.3f m"
              % (best[0], 100 * float(rospy.get_param("~trough_frac", 0.25)), e0, e1))
        print("     width %.3f m against the white board's true %.3f m  (error %+.3f m)"
              % (e1 - e0, tw, (e1 - e0) - tw))
        print("     centre %+.3f m against a true 0.000 m  (offset %+.3f m)"
              % ((e0 + e1) / 2, (e0 + e1) / 2))
        print("  >> A trough where the white board stands means it returns LESS")
        print("     depth than the dark boards beside it. Its edges are then a")
        print("     direct measurement of the material boundary.")
    else:
        print("  NO TROUGH: the profile is even across the width. The white")
        print("  surface returned depth like everything else - which is a")
        print("  RESULT, not a failure of the test.")

    if csv:
        new = not os.path.exists(csv)
        d = os.path.dirname(csv)
        if d and not os.path.isdir(d):
            os.makedirs(d)
        with open(csv, "a") as fh:
            if new:
                fh.write("run_id,method,target,frames_used,points_median,"
                         "width_m,width_sd,height_m,height_sd,depth_m,depth_sd,"
                         "distance_m,truth_width_m,truth_height_m,"
                         "trough_width_m,trough_centre_m\n")
            twid = (y0 + (best[2] + 1) * bw) - (y0 + best[1] * bw) if best[0] >= 2 else float("nan")
            tcen = ((y0 + best[1] * bw) + (y0 + (best[2] + 1) * bw)) / 2 if best[0] >= 2 else float("nan")
            fh.write("%s,paper_extent,%s,%d,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
                     "%.4f,%.3f,%.3f,,\n"
                     % (run, tname, kept, npts, med[0], sd[0], med[1], sd[1],
                        med[2], sd[2], med[3], tw, th))
            fh.write("%s,profile_edges,%s,%d,%d,,,,,,,%.4f,%.3f,%.3f,%.4f,%.4f\n"
                     % (run, tname, kept, npts, med[3], tw, th, twid, tcen))
        print()
        print("  two rows appended to %s  (one per method, never blended)" % csv)


if __name__ == "__main__":
    main()
