#!/usr/bin/env python3
"""board_geometry.py — what the test boards physically ARE, measured fresh.

WHY THIS RUNS BEFORE EVERY MEASUREMENT, NOT AFTER

    On 2026-08-30 a whole morning's comparison had to be withdrawn because one
    board was leaning back about 40 degrees while the other two stood at 6-10.
    Every result had been read as "blank surface versus printed surface" when
    it was really "blank AND tipped over versus printed AND upright" - two
    differences in one comparison.

    Nothing in the map, the depth statistics or the cell counts revealed that.
    It only showed up when a plane was fitted to each board and asked which way
    it faced. So that fit now happens FIRST, every session, and its output is
    recorded beside every result as the condition those results were taken in.

    Plain terms: before comparing how well the camera sees three boards, check
    that the three boards are actually standing the same way. Otherwise you are
    measuring the furniture, not the camera.

WHAT IT MEASURES, and what each number is for

    distance      median depth over the board's face. Compared against the tape.
    lean          how far the board is tipped back from upright, in degrees,
                  worked out from the direction its surface faces. 0 = standing
                  straight up. This is the covariate that ruined the earlier
                  comparison, so it is now always reported.
    flatness      how far the points sit from the single best-fit plane. A
                  LEANING board is still flat, so this separates "tipped over"
                  from "badly measured" - two things that look identical in a
                  depth histogram and mean opposite things.
    width         the board's width worked out from its pixel edges and range.
                  Checked against the tape-measured 94 in / 2.388 m for all
                  three, which is what confirms the edge-finding is landing on
                  real board edges rather than on furniture.
    completeness  fraction of the board's pixels that returned any depth.

HOW THE BOARDS ARE FOUND, and why it is not circular

    A column counts as board if it is (a) nearer than a cut-off well in front of
    the room behind and (b) not wildly jumbled - under 1.2 m between its 10th
    and 90th percentile. Furniture fails (b) with spreads of 2.3-3.0 m; the
    worst board column measured 0.89 m. Both are coarse yes/no gates that pick
    only WHERE to look. Neither can flatter a bad board: a board reading badly
    still passes both, it just reads badly. The wall's outer edges then divide
    into equal touching boards, whose count is given by --boards.

    usage:
      rosrun sidewalk_evaluation board_geometry.py _tape_m:=3.1115 \\
             _label:=swap_white_black _out:=~/bench_2026-08-30
"""
import csv as csvmod
import math
import os

import numpy as np
import rospy
from sensor_msgs.msg import CameraInfo, Image

try:
    import cv2
except ImportError:                                        # pragma: no cover
    cv2 = None

# Where the camera is. Mirrors sidewalk_bringup/urdf/husky_a200_real.urdf and
# zedx_front.launch; a drift between them shows up as a lean offset common to
# all three boards, which is why the boards are reported individually.
CAM_H = 0.6953           # left lens above the floor, m
CAM_PITCH = 0.0565       # 3.24 deg nose-down, measured by gravity
BOARD_H = 0.99           # board height, m


def main():
    rospy.init_node("board_geometry", anonymous=True, disable_signals=True)
    tape = float(rospy.get_param("~tape_m", 0.0))          # 0 = none available
    tape_note = str(rospy.get_param("~tape_note", ""))
    label = str(rospy.get_param("~label", "unlabelled"))
    nb = int(rospy.get_param("~boards", 3))
    total_true = float(rospy.get_param("~total_width_m", 2.388))   # 94 inches
    frames = int(rospy.get_param("~frames", 30))
    out = os.path.expanduser(str(rospy.get_param("~out", "~/bench")))
    names = str(rospy.get_param("~names", "left,middle,right")).split(",")
    depth_topic = rospy.get_param(
        "~depth", "/zedx_front/zed_node/depth/depth_registered")
    if not os.path.isdir(out):
        os.makedirs(out)

    ci = rospy.wait_for_message(
        "/zedx_front/zed_node/depth/camera_info", CameraInfo, timeout=20)
    fx, fy, cx, cy = ci.K[0], ci.K[4], ci.K[2], ci.K[5]

    fr = []
    for _ in range(frames):
        m = rospy.wait_for_message(depth_topic, Image, timeout=20)
        fr.append(np.frombuffer(m.data, dtype=np.float32).reshape(
            m.height, m.width))
    z = np.nanmedian(np.stack(fr), axis=0)
    H, W = z.shape

    guess = tape if tape > 0.5 else 3.0

    def row_at(h_m, rng):
        dz = CAM_H - h_m
        zc = rng * math.cos(CAM_PITCH) + dz * math.sin(CAM_PITCH)
        yc = -rng * math.sin(CAM_PITCH) + dz * math.cos(CAM_PITCH)
        return int(round(cy + fy * yc / zc))

    r_top = max(0, row_at(0.86 * BOARD_H, guess))
    r_bot = min(H, row_at(0.10 * BOARD_H, guess))
    mid = (r_top + r_bot) // 2
    band = z[max(0, mid - 40):mid + 40, :]

    # ---- locate the board wall ---------------------------------------------
    cut = guess + 0.75          # well in front of the room behind
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
        print("  NO BOARD WALL FOUND. Is the camera pointed at the boards?")
        return
    C0, C1 = max(runs, key=lambda r: r[1] - r[0])
    span_m = (C1 - C0) * guess / fx

    print("  camera %dx%d  fx=%.1f cx=%.1f cy=%.1f" % (W, H, fx, cx, cy))
    if tape > 0.5:
        print("  tape: %.4f m (%.1f in)%s" % (tape, tape / 0.0254,
                                              "  " + tape_note if tape_note else ""))
    else:
        print("  tape: NONE GIVEN - distances below have nothing independent to"
              " check them against")
    print("  board wall at columns %d..%d  = %.3f m across  (tape says %.3f m,"
          % (C0, C1, span_m, total_true))
    print("     difference %+.0f mm = %.0f px of edge error at each end)"
          % (1000 * (span_m - total_true),
             abs(span_m - total_true) * fx / guess / 2))
    print()

    # room-up, expressed in the camera's optical frame (x right, y down, z fwd)
    cp, sp = math.cos(CAM_PITCH), math.sin(CAM_PITCH)
    up = np.array([0.0, -cp, -sp])

    w3 = (C1 - C0) / float(nb)
    rows = []
    print("  board      distance    vs tape    LEAN from     flatness    width"
          "     valid")
    print("             (median)               upright       (rms)")
    for i in range(nb):
        nm = names[i] if i < len(names) else "board%d" % i
        a, b = int(C0 + i * w3), int(C0 + (i + 1) * w3)
        u0, u1 = a + int(0.10 * w3), b - int(0.10 * w3)
        face = z[r_top:r_bot, u0:u1]
        ok = np.isfinite(face) & (face > 1.2) & (face < 10.0)
        if ok.sum() < 500:
            print("  %-10s too few valid pixels (%d)" % (nm, int(ok.sum())))
            continue
        us, vs = np.meshgrid(np.arange(u0, u1), np.arange(r_top, r_bot))
        Z = face[ok]
        P = np.stack([(us[ok] - cx) * Z / fx,
                      (vs[ok] - cy) * Z / fy, Z], 1)
        c = P.mean(0)
        _, _, vt = np.linalg.svd(P - c, full_matrices=False)
        n = vt[2] / np.linalg.norm(vt[2])
        # angle between the surface normal and straight up. An upright board
        # faces horizontally -> 90 deg. Lean is how far short of 90 it falls.
        ang_from_up = math.degrees(math.acos(min(1.0, abs(float(n.dot(up))))))
        lean = 90.0 - ang_from_up
        resid = float((P - c).dot(n).std())
        med = float(np.median(Z))
        width = (b - a) * med / fx
        comp = 100.0 * ok.sum() / face.size
        dtape = (med - tape) if tape > 0.5 else float("nan")
        print("  %-10s %6.3f m   %+7s   %+6.1f deg    %6.1f mm  %5.3f m  %5.1f %%"
              % (nm, med,
                 ("%+.0f mm" % (1000 * dtape)) if tape > 0.5 else "  n/a",
                 lean, resid * 1000, width, comp))
        rows.append(dict(label=label, method="board_geometry_v1", board=nm,
                         distance_m=round(med, 4),
                         tape_m=(round(tape, 4) if tape > 0.5 else ""),
                         vs_tape_mm=(round(1000 * dtape, 1) if tape > 0.5 else ""),
                         lean_deg=round(lean, 2),
                         flatness_rms_mm=round(resid * 1000, 1),
                         width_m=round(width, 4), valid_pct=round(comp, 2),
                         cols="%d-%d" % (a, b), frames=frames,
                         wall_width_m=round(span_m, 4),
                         wall_width_true_m=total_true,
                         tape_note=tape_note))

    if rows:
        leans = [r["lean_deg"] for r in rows]
        print()
        print("  LEANS: %s   spread %.1f deg"
              % (", ".join("%s %.1f" % (r["board"], r["lean_deg"]) for r in rows),
                 max(leans) - min(leans)))
        if max(leans) - min(leans) > 8.0:
            print("  >> The boards are NOT standing the same way. Any comparison")
            print("     between them mixes surface with lean and cannot separate")
            print("     the two. Say so in the write-up or level them first.")
        else:
            print("  >> Leans agree within %.1f deg, so a comparison between"
                  % (max(leans) - min(leans)))
            print("     these boards is about their SURFACES.")

        p = os.path.join(out, "board_geometry.csv")
        new = not os.path.exists(p)
        with open(p, "a", newline="") as f:
            w = csvmod.DictWriter(f, fieldnames=list(rows[0].keys()))
            if new:
                w.writeheader()
            w.writerows(rows)
        print("\n  appended to %s" % p)

    # annotated picture, so the regions every number refers to can be checked
    if cv2 is not None:
        try:
            im = rospy.wait_for_message(
                "/zedx_front/zed_node/left/image_rect_color", Image, timeout=10)
            rgb = np.frombuffer(im.data, dtype=np.uint8).reshape(
                im.height, im.width, -1)[:, :, :3].copy()
            cols = [(90, 200, 90), (240, 160, 60), (80, 90, 235)]
            for i, r in enumerate(rows):
                a, b = [int(v) for v in r["cols"].split("-")]
                col = cols[i % len(cols)]
                cv2.rectangle(rgb, (a, r_top), (b, r_bot), col, 4)
                cv2.putText(rgb, "%s  %.3fm  lean %+.0f deg"
                            % (r["board"], r["distance_m"], r["lean_deg"]),
                            (a + 6, r_top - 16), cv2.FONT_HERSHEY_SIMPLEX,
                            0.95, col, 3)
            cv2.putText(rgb, "%s   tape %.3f m" % (label, tape), (50, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 4)
            f = os.path.join(out, "geometry_%s.jpg" % label)
            cv2.imwrite(f, cv2.resize(rgb, (1200, 750)),
                        [cv2.IMWRITE_JPEG_QUALITY, 90])
            print("  wrote %s" % f)
        except Exception as exc:                            # noqa: BLE001
            print("  (no picture: %s)" % exc)


if __name__ == "__main__":
    main()
