#!/usr/bin/env python3
"""cloud_board_check.py — in 3D, does each board come back as a board?

THE QUESTION, in the That is checkable against things we measured with a tape, so it is a
test with a right answer rather than a picture to admire:

    each board is 0.796 m wide and 0.99 m tall
    all three end to end are 2.388 m (94 inches)
    they stand upright (measured lean under 7 degrees) on a flat floor

WHAT IT REPORTS PER BOARD, and what would count as failing

    points           how many landed on it. Too few and the surface is
                     effectively invisible to anything downstream.
    width, height    the reconstructed size. Should match 0.796 x 0.99 m. Coming
                     out WIDER than truth means the surface is bleeding into its
                     background - the classic sign of over-smoothed depth.
                     Narrower means the edges are being eaten.
    flatness         distance of the points from one fitted plane. This is the
                     3D version of the number that separated a leaning board
                     from a badly measured one.
    lean             which way the surface faces. 0 = standing straight up.
    coverage         what fraction of the board's true area actually has points
                     within it, on a 2 cm grid. A board can have the right
                     outline and be full of holes; this catches that.

    Reads a binary PLY written by save_map_artifacts.py. No open3d or plyfile
    needed - neither is installed on this Jetson.

    usage:
      rosrun sidewalk_evaluation cloud_board_check.py \\
          _ply:=~/bench_2026-08-30/clouds/NEURAL_camera_raw_cloud.ply \\
          _label:=NEURAL _out:=~/bench_2026-08-30
"""
import csv as csvmod
import math
import os
import struct
import sys

import numpy as np

CAM_H, CAM_PITCH = 0.6953, 0.0565
BOARD_W, BOARD_H = 0.796, 0.99


DT = {b"float": "<f4", b"float32": "<f4", b"double": "<f8", b"uchar": "u1",
      b"uint8": "u1", b"char": "i1", b"int8": "i1", b"int": "<i4",
      b"int32": "<i4", b"uint": "<u4", b"uint32": "<u4", b"short": "<i2",
      b"ushort": "<u2"}


def read_ply(path):
    """Binary little-endian PLY.

    Handles more than x,y,z,r,g,b, because the two producers here disagree:
      save_map_artifacts.py writes  x y z red green blue
      rtabmap-export writes         x y z red green blue nx ny nz curvature
                                    AND a second "element camera" after it

    An earlier version summed EVERY property in the file into the vertex
    stride, including the trailing camera element's, and then asked numpy for
    more bytes than the vertex block holds. So: only properties belonging to
    `element vertex` count, and reading stops at the next element.
    """
    with open(path, "rb") as f:
        if f.readline().strip() != b"ply":
            raise ValueError("not a PLY")
        if b"binary_little_endian" not in f.readline().strip():
            raise ValueError("only binary_little_endian is handled")
        n, props, cur = 0, [], None
        while True:
            ln = f.readline()
            if not ln:
                raise ValueError("header never ended")
            ln = ln.strip()
            if ln.startswith(b"element "):
                parts = ln.split()
                cur = parts[1]
                if cur == b"vertex":
                    n = int(parts[2])
            elif ln.startswith(b"property") and cur == b"vertex":
                p = ln.split()
                if p[1] == b"list":          # variable-length; not used by vertices
                    raise ValueError("list properties on vertex are not handled")
                props.append((p[1], p[2].decode()))
            elif ln == b"end_header":
                break
        dtype = np.dtype([(nm, DT[ty]) for ty, nm in props])
        raw = f.read(n * dtype.itemsize)
    if len(raw) < n * dtype.itemsize:
        raise ValueError("vertex block truncated: wanted %d bytes, got %d"
                         % (n * dtype.itemsize, len(raw)))
    arr = np.frombuffer(raw, dtype=dtype, count=n)
    return (np.stack([arr["x"], arr["y"], arr["z"]], 1).astype(np.float64),
            [nm for _, nm in props])


def to_room_frame(P):
    """Put the points into X forward, Y left, Z up-from-the-floor.

    The convention is DETECTED, not assumed. The two sources differ and an
    earlier version guessed wrong, rotating an already-upright cloud and
    landing every point outside the search box - which read as "the camera saw
    nothing" when the camera had seen 1.2 million points.

      optical frame (x right, y down, z forward): forward values live in z
      body frame    (x forward, y left, z up):    forward values live in x

    Whichever axis holds the large positive median is the forward one.
    """
    med = np.median(P, axis=0)
    if abs(med[2]) > abs(med[0]) and med[2] > 1.0:
        # optical: rotate out the camera's downward pitch, then relabel
        cp, sp = math.cos(CAM_PITCH), math.sin(CAM_PITCH)
        return np.stack([P[:, 2] * cp - P[:, 1] * sp,
                         -P[:, 0],
                         -(P[:, 1] * cp + P[:, 2] * sp) + CAM_H], 1), "optical"
    # already body-like; z is measured from the lens, so lift it to the floor
    return np.stack([P[:, 0], P[:, 1], P[:, 2] + CAM_H], 1), "body"


def main():
    try:
        import rospy
        get = lambda k, d: rospy.get_param("~" + k, d)      # noqa: E731
        rospy.init_node("cloud_board_check", anonymous=True, disable_signals=True)
    except Exception:                                        # noqa: BLE001
        args = dict(a.split("=", 1) for a in sys.argv[1:] if "=" in a)
        get = lambda k, d: args.get(k, d)                    # noqa: E731

    ply = os.path.expanduser(str(get("ply", "")))
    label = str(get("label", "cloud"))
    out = os.path.expanduser(str(get("out", "~/bench")))
    # board centres sideways (+ is the robot's left), given on the command line
    centres = str(get("centres", "0.452,-0.378,-1.208"))
    names = str(get("names", "left,middle,right")).split(",")
    rng = float(get("range_m", 3.12))
    if not ply or not os.path.exists(ply):
        print("  give _ply:=<path to a .ply>")
        return
    if not os.path.isdir(out):
        os.makedirs(out)

    P, props = read_ply(ply)
    R, conv = to_room_frame(P)
    print("  %s: %d points, %s frame detected (properties: %s)"
          % (os.path.basename(ply), len(P), conv, ",".join(props)))
    print("     forward %.2f..%.2f m | sideways %+.2f..%+.2f m | up %.2f..%.2f m"
          % (R[:, 0].min(), R[:, 0].max(), R[:, 1].min(), R[:, 1].max(),
             R[:, 2].min(), R[:, 2].max()))

    keep = (R[:, 0] > rng - 0.60) & (R[:, 0] < rng + 0.60) & \
           (R[:, 2] > 0.05) & (R[:, 2] < 1.15)
    W = R[keep]
    print("  %d points in the board slab (%.2f-%.2f m ahead, 0.05-1.15 m up)"
          % (len(W), rng - 0.60, rng + 0.60))
    if len(W) == 0:
        print("  NOTHING in that slab. Check _range_m against the forward range"
              " printed above before reading anything into this.")
        return
    print()
    print("  board    points   width x height     vs true         flatness   lean   coverage")
    print("                    (m)                0.796 x 0.99    (rms mm)   (deg)  (%)")
    rows = []
    for i, c in enumerate([float(v) for v in centres.split(",")]):
        nm = names[i] if i < len(names) else "board%d" % i
        m = (W[:, 1] > c - BOARD_W * 0.62) & (W[:, 1] < c + BOARD_W * 0.62)
        B = W[m]
        if len(B) < 500:
            print("  %-8s %6d   too few points to judge" % (nm, len(B)))
            continue
        # trim the outer 2% each way so a few stragglers do not set the extent
        wl, wh = np.percentile(B[:, 1], [2, 98])
        hl, hh = np.percentile(B[:, 2], [2, 98])
        width, height = wh - wl, hh - hl
        cen = B.mean(0)
        _, _, vt = np.linalg.svd(B - cen, full_matrices=False)
        n = vt[2] / np.linalg.norm(vt[2])
        resid = float((B - cen).dot(n).std())
        lean = 90.0 - math.degrees(math.acos(min(1.0, abs(float(n[2])))))
        # coverage: of the 2 cm squares the board should fill, how many have a point
        gy = np.floor((B[:, 1] - wl) / 0.02).astype(int)
        gz = np.floor((B[:, 2] - hl) / 0.02).astype(int)
        filled = len(set(zip(gy.tolist(), gz.tolist())))
        expect = max(1, int(round(width / 0.02)) * int(round(height / 0.02)))
        cov = 100.0 * filled / expect
        print("  %-8s %6d   %.3f x %.3f      %+.0f x %+.0f mm    %6.1f   %+4.1f   %5.1f"
              % (nm, len(B), width, height,
                 1000 * (width - BOARD_W), 1000 * (height - BOARD_H),
                 resid * 1000, lean, cov))
        rows.append(dict(label=label, method="cloud_board_check_v1", board=nm,
                         points=len(B), width_m=round(width, 4),
                         height_m=round(height, 4),
                         width_err_mm=round(1000 * (width - BOARD_W), 1),
                         height_err_mm=round(1000 * (height - BOARD_H), 1),
                         flatness_rms_mm=round(resid * 1000, 1),
                         lean_deg=round(lean, 2), coverage_pct=round(cov, 1),
                         centre_y_m=c, source=os.path.basename(ply)))
    if rows:
        tot = sum(r["width_m"] for r in rows)
        print()
        print("  three boards end to end: %.3f m against a tape-measured 2.388 m"
              "  (%+.0f mm)" % (tot, 1000 * (tot - 2.388)))
        p = os.path.join(out, "cloud_board_check.csv")
        new = not os.path.exists(p)
        with open(p, "a", newline="") as f:
            w = csvmod.DictWriter(f, fieldnames=list(rows[0].keys()))
            if new:
                w.writeheader()
            w.writerows(rows)
        print("  appended to %s" % p)


if __name__ == "__main__":
    main()
