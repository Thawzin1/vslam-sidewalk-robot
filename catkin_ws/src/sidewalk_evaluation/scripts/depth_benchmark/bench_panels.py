#!/usr/bin/env python3
"""bench_panels.py — one shared description of where the test boards are.

WHAT PROBLEM THIS SOLVES

    Two programs measure the same three cardboard boards, and until now each
    one had its own hand-drawn list of which image pixels to look at:

        bench_report.py       white board: 389 x 370 pixels   (78% of the board)
        panel_depth_stats.py  white board: 180 x 220 pixels   (31% of the board)

    Plain terms: imagine measuring how bumpy a table is, but one person looks
    at the whole table and the other looks at a small patch in the middle. The
    small patch always looks flatter. That is not a difference in the table.

    It matters here because the way a blank board fails is a LARGE SMOOTH BEND
    across the whole board, not fine speckle. A small viewing box sits inside
    the bend and never sees it, so the board scores better simply for being
    looked at less. That is exactly what produced the impossible result where
    two boards appeared to get BETTER as they moved further away.

HOW IT IS FIXED

    Describe each board once by real-world facts that do not change when it
    moves — how wide it is, how tall, and where its middle sits left or right
    of the camera. Then work out the pixel box fresh every run from the
    camera's own calibration and the measured distance.

    The result: both programs always look at the SAME FRACTION of the board,
    at any distance, and two measurements can be compared.

WHAT IS RECORDED SO NOBODY IS MISLED LATER

    Every run writes a `window_key` such as "0.63x0.76" — the size in metres
    of the patch that was actually measured. Two results are comparable only
    if that string matches. It is a string on purpose: comparing two rows
    becomes a sort in a spreadsheet, not arithmetic and trust.

WHAT THIS DELIBERATELY DOES NOT DO

    It does not find the boards by looking for them. A brightness or depth
    detector would be circular here: how well the camera sees these surfaces
    is the very thing under test, so a detector would shrink the box on a bad
    run and flatter the result on exactly the run where it matters. Detection
    is used only as a CHECK that aborts (see check_geometry), never to decide
    where to measure.
"""
import collections
import math
import os
import sys


# ---------------------------------------------------------------------------
# THE TEST SETUP, described in real-world terms
# ---------------------------------------------------------------------------
LAYOUT = "three_panel_static_2026_08_29"

PANEL_W_M = 0.79          # each board, side to side
PANEL_H_M = 0.99          # each board, floor to top

# How far each board's middle sits left (+) or right (-) of straight ahead.
#
# *** THESE ARE A STARTING GUESS ONLY. MEASURE THEM EVERY SESSION. ***
#
# Use calibrate_from_image() below. Two reasons, and the second is the one
# that bites:
#
#      straight line.
#
#   2. These numbers are NOT independent evidence of anything. They were
#      back-solved from one session's hand-drawn pixel boxes assuming the
#      image centre sat at 952.5 pixels. The camera's own calibration says
#      1005.4. That is 53 pixels, which is 109 mm of sideways error at 2.6 m
#      and 189 mm at 4.5 m.
#
#      A check that these reproduce those hand-drawn boxes therefore proves
#      NOTHING - both carry the same wrong assumption, so it is one number
#      compared with itself. That is the same failure already recorded twice
#      in docs/DO_NOT_REPEAT.md, and it happened again here while writing
#      this file.
PANEL_CENTRE_Y_M = collections.OrderedDict([
    ("white_blank",   +0.466),
    ("printed_sign",  -0.323),
    ("black_blank",   -1.107),
])

# Colours used to outline each board in the pictures. Kept here so the figures
# and the tables cannot drift apart.
PANEL_COLOUR_BGR = {
    "white_blank":  (90, 200, 90),
    "printed_sign": (240, 160, 60),
    "black_blank":  (80, 90, 235),
}

# Where the camera is. MIRRORS sidewalk_bringup/urdf/husky_a200_real.urdf and
# robot_frames.yaml - those are the source of truth. check_geometry() compares
# against the live transform tree when one is available, so a drift shows up
# rather than silently biasing every row.
CAM_HEIGHT_M = 0.6953     # left lens above the floor
CAM_PITCH_RAD = 0.0565    # 3.24 degrees nose-down, measured

# How much of each board to measure.
#
# Across the width: 10% in from each edge. The edges are where a pixel sees
# part board and part background, and those mixed pixels are the noisiest
# thing in the picture.
#
# Up the height: from 10% to 86% ABOVE THE FLOOR - deliberately not symmetric.
# The bottom edge meets the floor and the top edge meets open air; those are
# different problems, and this band is also what reproduces the boxes that
# were drawn by hand and used for every result so far.
WIN_FRAC_U = (0.10, 0.90)
WIN_FRAC_V = (0.10, 0.86)

# change here can be checked against them rather than trusted.
LEGACY_PANELS_2586 = {
    "white_blank":  (531, 920),
    "printed_sign": (920, 1299),
    "black_blank":  (1299, 1635),
}
LEGACY_ROWS_2586 = (450, 820)
LEGACY_RANGE_M = 2.586


# Positionally identical to the 7-tuple bench_report.py already unpacks, so
# every existing `for nm, c0, c1, _, _, _, col in PANELS` keeps working and
# named access (p.px0) becomes available for free.
Panel = collections.namedtuple(
    "Panel", "label px0 px1 y_lo y_hi truth_w colour row0 row1")


def _row_for_height(p_m, range_m, fy, cy,
                    h=CAM_HEIGHT_M, pitch=CAM_PITCH_RAD):
    """Image row of a point `p_m` above the floor, on a board `range_m` away.

    Why this is not simply "scale the rows by distance": the camera is tilted
    3.24 degrees down, and the board stands on the FLOOR, not on the camera's
    line of sight. Scaling rows about the image centre is wrong by 13-15
    pixels at 3.11 m, which slides the box down toward the floor junction.
    """
    dz = h - p_m
    z_cam = range_m * math.cos(pitch) + dz * math.sin(pitch)
    y_cam = -range_m * math.sin(pitch) + dz * math.cos(pitch)
    return cy + fy * y_cam / z_cam


def project_windows(K, range_m, y_offset_m=0.0,
                    frac_u=WIN_FRAC_U, frac_v=WIN_FRAC_V, centres=None):
    """Work out each board's pixel box for this camera at this distance.

    K is (fx, fy, cx, cy) straight from the camera's own calibration message.
    """
    fx, fy, cx, cy = K
    u0f, u1f = frac_u
    v0f, v1f = frac_v
    out = []
    for label, cy_m in (centres or PANEL_CENTRE_Y_M).items():
        y_c = cy_m + y_offset_m
        # The box's left edge in the picture is the MORE POSITIVE y, because
        # y points left and image columns count rightward.
        y_hi = y_c + PANEL_W_M * (0.5 - u0f)
        y_lo = y_c - PANEL_W_M * (0.5 - u0f)
        px0 = int(round(cx - fx * y_hi / range_m))
        px1 = int(round(cx - fx * y_lo / range_m))
        r_top = _row_for_height(PANEL_H_M * v1f, range_m, fy, cy)
        r_bot = _row_for_height(PANEL_H_M * v0f, range_m, fy, cy)
        out.append(Panel(label, px0, px1, y_lo, y_hi, PANEL_W_M,
                         PANEL_COLOUR_BGR[label],
                         int(round(r_top)), int(round(r_bot))))
    return out


def calibrate_from_image(depth_img, K, np_mod, row_band,
                         range_hint_m=None, expect_w_m=PANEL_W_M,
                         tol_m=0.12):
    """Measure where the three boards actually are, from the live camera.

    WHY THIS EXISTS RATHER THAN A CONSTANT. The boards get moved between
    sessions, and the stored centres are only a starting guess whose provenance
    is not independent (see the note on PANEL_CENTRE_Y_M). Measuring them each
    session is both more accurate and self-correcting.

    HOW IT AVOIDS BEING CIRCULAR. It uses DEPTH, not brightness, to find where
    the group of boards begins and ends. Depth says "something solid is here at
    the board distance" - it does not care whether the surface is easy or hard
    to see, which is the quantity under test. Brightness would care, and would
    quietly shrink the measured region on a dark board, which is exactly the
    bias this whole file exists to remove.

    It then assumes the three boards are equal width and touching, which the
    photographs show, and CHECKS that assumption: if the width it derives
    disagrees with the real board width by more than `tol_m`, it refuses and
    says so rather than returning a plausible wrong answer.

    Returns (centres_dict, info_dict) or (None, info_dict) on refusal.
    """
    fx, fy, cx, cy = K
    r0, r1 = row_band
    band = depth_img[r0:r1, :]
    good = np_mod.isfinite(band) & (band > 0.3) & (band < 12.0)
    if range_hint_m is None:
        if good.sum() < 2000:
            return None, {"error": "almost no depth returned - is the camera "
                                   "pointed at the boards, and are the lights on?"}
        range_hint_m = float(np_mod.median(band[good]))
    near = good & (np_mod.abs(band - range_hint_m) < 0.30)
    per_col = near.sum(axis=0)
    if per_col.max() < 10:
        return None, {"error": "no column has enough returns at %.2f m - the "
                               "distance may be wrong" % range_hint_m}
    thresh = 0.35 * float(np_mod.median(per_col[per_col > 0]))
    cols = np_mod.where(per_col > thresh)[0]
    if len(cols) < 60:
        return None, {"error": "the boards span only %d columns, too few to "
                               "trust" % len(cols)}
    c_lo, c_hi = int(cols.min()), int(cols.max())
    span_px = c_hi - c_lo
    board_px = span_px / 3.0
    board_w_m = board_px * range_hint_m / fx
    info = {"range_m": range_hint_m, "col_lo": c_lo, "col_hi": c_hi,
            "span_px": span_px, "derived_board_w_m": board_w_m}
    if abs(board_w_m - expect_w_m) > tol_m:
        info["error"] = (
            "the three boards work out %.3f m wide each, but they should be "
            "%.3f m. Either one board is hidden, something else is in the "
            "picture at the same distance, or they are not touching."
            % (board_w_m, expect_w_m))
        return None, info
    centres = collections.OrderedDict()
    for i, label in enumerate(PANEL_CENTRE_Y_M):
        c_mid = c_lo + board_px * (i + 0.5)
        centres[label] = float(-(c_mid - cx) * range_hint_m / fx)
    info["centres"] = centres
    return centres, info


def window_metres(frac_u=WIN_FRAC_U, frac_v=WIN_FRAC_V):
    """The measured patch in metres on the board's surface."""
    return (PANEL_W_M * (frac_u[1] - frac_u[0]),
            PANEL_H_M * (frac_v[1] - frac_v[0]))


def window_key(frac_u=WIN_FRAC_U, frac_v=WIN_FRAC_V):
    """The comparability stamp. Two results compare only if this matches."""
    w, h = window_metres(frac_u, frac_v)
    return "%.2fx%.2f" % (w, h)


def as_stats_tuples(panels):
    """(label, first_column, last_column) - the shape panel_depth_stats uses."""
    return [(p.label, p.px0, p.px1) for p in panels]


def seed_range(depth_img, panels_nominal, row_band, np_mod):
    """Measure the distance to the boards from a depth picture.

    Uses the MIDDLE of the whole board group and takes the median, not the
    mean, because these errors have a long tail and a mean chases outliers.

    ONE DISTANCE IS USED FOR ALL THREE BOARDS, never each board's own. The
    boards are in one line by construction, and the small differences between
    their measured depths ARE the error being studied - sizing a board's own
    box by its own biased depth would let the measurement feed back into
    itself. In the lit session white read 103 mm long; using that would have
    shrunk white's box by 4% and quietly flattered it.
    """
    vals = []
    for p in panels_nominal:
        c = (p.px0 + p.px1) // 2
        half = max(8, (p.px1 - p.px0) // 8)
        sub = depth_img[row_band[0]:row_band[1], c - half:c + half]
        ok = np_mod.isfinite(sub) & (sub > 0.3) & (sub < 12.0)
        if ok.sum() > 50:
            vals.append(float(np_mod.median(sub[ok])))
    if not vals:
        return None
    return float(np_mod.median(np_mod.array(vals)))


def check_geometry(depth_img, panels, range_m, np_mod,
                   range_tol_m=0.35, min_on_plane=0.70,
                   max_occluded=0.02, min_side_px=60, image_wh=None):
    """Refuse to measure the wrong thing, loudly.

    Returns (ok, list_of_problems). Every problem names what is wrong in
    ordinary words, because a run that silently measures a chair leg is worse
    than one that stops.
    """
    problems = []
    if image_wh:
        W, H = image_wh
        for p in panels:
            if p.px0 < 0 or p.px1 > W or p.row0 < 0 or p.row1 > H:
                problems.append(
                    "%s: its box falls outside the picture (%d..%d x %d..%d). "
                    "The boards have probably moved sideways, or the distance "
                    "given is wrong." % (p.label, p.px0, p.px1, p.row0, p.row1))
    for p in panels:
        if p.px1 - p.px0 < min_side_px or p.row1 - p.row0 < min_side_px:
            problems.append(
                "%s: box is only %dx%d pixels, too small to measure. Either "
                "the boards are much further away than stated, or the fraction "
                "was set too small." % (p.label, p.px1 - p.px0, p.row1 - p.row0))
    order = [p.px0 for p in panels]
    if order != sorted(order):
        problems.append(
            "the boards' boxes are not in left-to-right order, which means the "
            "layout no longer matches this file. Re-derive the centres.")
    for a, b in zip(panels, panels[1:]):
        if a.px1 > b.px0:
            problems.append("%s and %s overlap in the picture."
                            % (a.label, b.label))
    for p in panels:
        sub = depth_img[p.row0:p.row1, p.px0:p.px1]
        ok = np_mod.isfinite(sub) & (sub > 0.3) & (sub < 12.0)
        n = int(ok.sum())
        if n < 0.10 * sub.size:
            problems.append(
                "%s: only %.0f%% of its box returned any depth. Too little to "
                "measure." % (p.label, 100.0 * n / sub.size))
            continue
        d = sub[ok]
        med = float(np_mod.median(d))
        if abs(med - range_m) > range_tol_m:
            problems.append(
                "%s: sits at %.2f m but the distance in use is %.2f m. Either "
                "the boards are not in one line, or something else is in the "
                "box - a divider behind them, or a chair in front."
                % (p.label, med, range_m))
        on_plane = float((np_mod.abs(d - range_m) < 0.25).sum()) / n
        if on_plane < min_on_plane:
            problems.append(
                "%s: only %.0f%% of its points are near one flat surface. The "
                "box is catching two different things at once, so a flatness "
                "figure from it would be meaningless."
                % (p.label, 100.0 * on_plane))
        near = float((d < range_m - 0.15).sum()) / n
        if near > max_occluded:
            problems.append(
                "%s: %.1f%% of its points are more than 15 cm NEARER than the "
                "board. Something is standing in front of it."
                % (p.label, 100.0 * near))
    return (not problems), problems


def measurement_columns():
    """Extra CSV columns that make comparability checkable rather than assumed."""
    return ["window_key", "window_w_m", "window_h_m", "window_frac_w",
            "window_frac_h", "range_m_used", "range_source",
            "px_c0", "px_c1", "px_r0", "px_r1", "window_area_px",
            "fx_px", "cx_px", "cy_px", "geom_source"]


def measurement_values(panel, K, range_m, range_source,
                       frac_u=WIN_FRAC_U, frac_v=WIN_FRAC_V):
    fx, fy, cx, cy = K
    w_m, h_m = window_metres(frac_u, frac_v)
    return [window_key(frac_u, frac_v), "%.3f" % w_m, "%.3f" % h_m,
            "%.2f" % (frac_u[1] - frac_u[0]), "%.2f" % (frac_v[1] - frac_v[0]),
            "%.3f" % range_m, range_source,
            panel.px0, panel.px1, panel.row0, panel.row1,
            (panel.px1 - panel.px0) * (panel.row1 - panel.row0),
            "%.1f" % fx, "%.1f" % cx, "%.1f" % cy,
            "bench_panels.py:" + LAYOUT]


def _self_check():
    """What can and cannot be proved without the camera.

    NOT CHECKED HERE, ON PURPOSE: whether the stored board centres are right.
    An earlier version of this function compared them against the hand-drawn
    pixel boxes and reported a 2-3 pixel match, which looked like strong
    evidence and was worthless - both were derived assuming the same wrong
    image centre, so it was one number compared with itself. The centres can
    only be checked against the live camera, by calibrate_from_image().

    Run directly:  python3 bench_panels.py
    """
    ok = True
    fx = fy = 1258.5
    cx, cy = 1005.4, 587.1
    print("  using the camera's OWN calibration: fx %.1f  cx %.1f  cy %.1f"
          % (fx, cx, cy))
    print()
    print("  the same window at three distances - the pixel box shrinks with")
    print("  range, but the fraction of the board it covers does not:")
    print()
    print("    range     white board box      pixels     board fraction")
    for Z in (2.586, 3.11, 4.5):
        p = project_windows((fx, fy, cx, cy), Z)[0]
        w_px, h_px = p.px1 - p.px0, p.row1 - p.row0
        w_m = w_px * Z / fx
        print("    %.3f m   cols %4d..%-4d      %3dx%-3d    %.3f m of %.2f m"
              % (Z, p.px0, p.px1, w_px, h_px, w_m, PANEL_W_M))
        if abs(w_m - PANEL_W_M * (WIN_FRAC_U[1] - WIN_FRAC_U[0])) > 0.01:
            print("      ^ WRONG: should be %.3f m at every distance"
                  % (PANEL_W_M * (WIN_FRAC_U[1] - WIN_FRAC_U[0])))
            ok = False
    print()
    w, h = window_metres()
    print("  standing window: %.3f x %.3f m of each board   key = %s"
          % (w, h, window_key()))
    print("  that is %.0f%% of the width and %.0f%% of the height, at any range."
          % (100 * (WIN_FRAC_U[1] - WIN_FRAC_U[0]),
             100 * (WIN_FRAC_V[1] - WIN_FRAC_V[0])))
    print()
    print("  BOARD CENTRES ARE NOT CHECKED HERE - they must be measured against")
    print("  the live camera with calibrate_from_image(). See the note above")
    print("  PANEL_CENTRE_Y_M for why checking them offline proves nothing.")
    return ok


if __name__ == "__main__":
    sys.exit(0 if _self_check() else 1)
