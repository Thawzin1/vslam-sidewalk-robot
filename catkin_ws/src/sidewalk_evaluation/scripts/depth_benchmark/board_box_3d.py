#!/usr/bin/env python3
"""board_box_3d.py — measure each board inside a box placed from the tape.

THE METHOD, and the two conditions that make it valid

    A box drawn around each board excludes the floor, the furniture and the
    room behind, so what is measured is the board and only the board. Two
    things decide whether that is a measurement or a self-fulfilling prophecy:

    1. THE BOX COMES FROM THE TAPE, NEVER FROM THE POINTS.
       Its width and height are the boards' measured size (0.796 x 0.99 m) and
       its distance is the tape reading. Nothing about where the camera thinks
       the surface is enters into placing it. Fitting the box to the points
       would make "the points are in the box" a tautology - the same
       circularity recorded three times in docs/DO_NOT_REPEAT.md.

    2. TIGHT SIDEWAYS AND VERTICALLY, GENEROUS IN DEPTH.
       Depth error is the quantity under test. On 2026-08-30 ULTRA placed blank
       boards ~110 mm further away than the tape allows, with a p99 tail 546 mm
       out. A box drawn tight in depth would have excluded exactly those
       readings and reported "the board is missing" - turning the finding into
       an artefact. So the depth window is deliberately loose, and WHERE inside
       it the points fall is reported as a result rather than filtered away.

    Plain terms: draw the box where the tape says the board is, make it snug
    left-right and top-bottom where we trust the geometry, and leave it long in
    the direction where the camera might be wrong - then report how far along
    that length the readings actually landed.

WHY IT WORKS FROM THE DEPTH IMAGE AND NOT FROM A SAVED CLOUD

    The pixel windows are validated: they reproduce the tape-measured 2.388 m
    wall to about 10 mm. Two earlier attempts to do this from a saved PLY got
    the coordinate convention wrong and reported a board absent that was
    present at 100 %. The depth image needs no frame convention at all - a
    pixel and the camera's own calibration give the 3D point directly.

    usage:
      rosrun sidewalk_evaluation board_box_3d.py _tape_blank:=3.1242 \\
          _tape_sign:=3.1115 _label:=ULTRA _out:=~/bench_2026-08-30
"""
import csv as csvmod
import math
import os

import numpy as np
import rospy
from sensor_msgs.msg import CameraInfo, Image

CAM_H, CAM_PITCH = 0.6953, 0.0565
BOARD_W, BOARD_H = 0.796, 0.99


def main():
    rospy.init_node("board_box_3d", anonymous=True, disable_signals=True)
    tape_blank = float(rospy.get_param("~tape_blank", 3.1242))
    tape_sign = float(rospy.get_param("~tape_sign", 3.1115))
    label = str(rospy.get_param("~label", "run"))
    out = os.path.expanduser(str(rospy.get_param("~out", "~/bench")))
    frames = int(rospy.get_param("~frames", 30))
    names = str(rospy.get_param("~names", "left,middle,right")).split(",")
    # generous in depth, on purpose - see the docstring
    back = float(rospy.get_param("~depth_back", 0.60))
    front = float(rospy.get_param("~depth_front", 0.30))
    if not os.path.isdir(out):
        os.makedirs(out)

    ci = rospy.wait_for_message(
        "/zedx_front/zed_node/depth/camera_info", CameraInfo, timeout=20)
    fx, fy, cx, cy = ci.K[0], ci.K[4], ci.K[2], ci.K[5]

    fr = []
    for _ in range(frames):
        m = rospy.wait_for_message(
            "/zedx_front/zed_node/depth/depth_registered", Image, timeout=20)
        fr.append(np.frombuffer(m.data, dtype=np.float32).reshape(
            m.height, m.width))
    z = np.nanmedian(np.stack(fr), axis=0)
    H, W = z.shape

    def row_at(h_m, rng):
        dz = CAM_H - h_m
        zc = rng * math.cos(CAM_PITCH) + dz * math.sin(CAM_PITCH)
        yc = -rng * math.sin(CAM_PITCH) + dz * math.cos(CAM_PITCH)
        return int(round(cy + fy * yc / zc))

    # Vertical extent of the BOX: the board's real height, from the tape range.
    r_top = max(0, row_at(0.92 * BOARD_H, tape_blank))
    r_bot = min(H, row_at(0.06 * BOARD_H, tape_blank))

    # Sideways: locate the wall by a coarse gate (near, and not jumbled), then
    # divide into equal touching boards. The gate only picks WHERE to look; it
    # cannot flatter a bad board, which still passes it and still reads badly.
    mid = (r_top + r_bot) // 2
    band = z[max(0, mid - 40):mid + 40, :]
    cut = tape_blank + 0.75
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
    wall_m = (C1 - C0) * tape_blank / fx
    w3 = (C1 - C0) / 3.0
    print("  wall columns %d..%d = %.3f m at the tape range (tape 2.388 m, %+.0f mm)"
          % (C0, C1, wall_m, 1000 * (wall_m - 2.388)))
    print("  box: %.3f m wide x %.3f m tall, depth from tape-%.2f to tape+%.2f m"
          % (BOARD_W, BOARD_H * 0.86, front, back))
    print()
    print("  board        in box   coverage   where in the box the points landed")
    print("               points      %       p50 vs tape   p90      p99     spread")
    rows = []
    for i in range(3):
        nm = names[i] if i < len(names) else "board%d" % i
        tape = tape_sign if i == 1 else tape_blank
        a, b = int(C0 + i * w3), int(C0 + (i + 1) * w3)
        # 6 % inset: the outermost pixels straddle board and background and are
        # the noisiest thing in the picture. Same fraction on every board and at
        # every range, so two results are always comparable.
        u0, u1 = a + int(0.06 * w3), b - int(0.06 * w3)
        sub = z[r_top:r_bot, u0:u1]
        finite = np.isfinite(sub) & (sub > 0.5) & (sub < 12.0)
        inbox = finite & (sub > tape - front) & (sub < tape + back)
        n_in = int(inbox.sum())
        if n_in < 500:
            print("  %-12s %6d   too few points inside the box" % (nm, n_in))
            continue
        zz = sub[inbox]
        p50, p90, p99 = np.percentile(zz, [50, 90, 99])

        # coverage: of the box's face, split into 2 cm squares, how many hold a
        # point. Catches a board with the right outline and holes in the middle.
        vs, us = np.where(inbox)
        Y = -((us + u0) - cx) * zz / fx
        Zup = CAM_H - ((vs + r_top) - cy) * zz / fy * math.cos(CAM_PITCH)
        gy = np.floor((Y - Y.min()) / 0.02).astype(int)
        gz = np.floor((Zup - Zup.min()) / 0.02).astype(int)
        filled = len(set(zip(gy.tolist(), gz.tolist())))
        expect = max(1, int(round((Y.max() - Y.min()) / 0.02)) *
                     int(round((Zup.max() - Zup.min()) / 0.02)))
        cov = min(100.0, 100.0 * filled / expect)

        # plane fit inside the box
        P = np.stack([Y, Zup, zz], 1)
        cen = P.mean(0)
        _, _, vt = np.linalg.svd(P - cen, full_matrices=False)
        resid = float((P - cen).dot(vt[2]).std())

        outside = int((finite & ~inbox).sum())
        print("  %-12s %6d    %5.1f     %+6.0f mm   %+6.0f   %+6.0f   %5.0f mm"
              % (nm, n_in, cov, 1000 * (p50 - tape), 1000 * (p90 - tape),
                 1000 * (p99 - tape), 1000 * (p99 - p50)))
        rows.append(dict(label=label, method="board_box_3d_v1", board=nm,
                         tape_m=tape, points_in_box=n_in,
                         points_outside_box=outside,
                         coverage_pct=round(cov, 1),
                         p50_vs_tape_mm=round(1000 * (p50 - tape), 1),
                         p90_vs_tape_mm=round(1000 * (p90 - tape), 1),
                         p99_vs_tape_mm=round(1000 * (p99 - tape), 1),
                         p50_to_p99_mm=round(1000 * (p99 - p50), 1),
                         plane_resid_mm=round(resid * 1000, 1),
                         box_w_m=BOARD_W, box_h_m=round(BOARD_H * 0.86, 3),
                         depth_front_m=front, depth_back_m=back,
                         wall_width_m=round(wall_m, 4), frames=frames))
    if rows:
        print()
        print("  board        points OUTSIDE the box (finite depth, wrong place)")
        for r in rows:
            tot = r["points_in_box"] + r["points_outside_box"]
            print("  %-12s %6d of %6d  =  %5.2f %%"
                  % (r["board"], r["points_outside_box"], tot,
                     100.0 * r["points_outside_box"] / tot))
        print("  A high figure here is the finding, not a filtering problem: the")
        print("  camera returned a depth, and it fell outside where the board is.")
        p = os.path.join(out, "board_box_3d.csv")
        new = not os.path.exists(p)
        with open(p, "a", newline="") as f:
            w = csvmod.DictWriter(f, fieldnames=list(rows[0].keys()))
            if new:
                w.writeheader()
            w.writerows(rows)
        print("\n  appended to %s" % p)


if __name__ == "__main__":
    main()
