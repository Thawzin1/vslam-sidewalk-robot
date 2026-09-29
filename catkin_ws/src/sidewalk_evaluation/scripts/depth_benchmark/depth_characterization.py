#!/usr/bin/env python3
"""
depth_characterization.py — Measure what the ZED X's depth is actually worth.

WHY THIS COMES BEFORE THE SLAM RESULTS
    RTAB-Map builds its map out of depth measurements. If the depth is
    systematically 4% short at 10 m, the map is 4% short at 10 m, and no amount
    of tuning the pose graph will fix it. So before attributing any mapping
    error to an algorithm, the sensor itself has to be characterised.

    This also produces the error model that everything downstream should be
    interpreted against. A 15 cm ATE means something quite different if the raw
    depth noise at typical operating range is 5 cm than if it is 30 cm.

THE EXPERIMENT
    Point the camera square-on at a large flat wall, at a measured distance.
    Collect a few hundred frames. From a central region of interest, compute:

      * **bias**      mean measured depth minus the true distance. A ruler
                      error: everything is uniformly too near or too far.
      * **std dev**   frame-to-frame and pixel-to-pixel scatter. The noise.
      * **invalid %** fraction of pixels that returned no depth at all. On a
                      blank painted wall this is a direct measure of how much
                      texture the stereo matcher needs; on a textureless
                      surface it can be most of the image.
      * **flatness**  fit a plane to the 3D points and measure the RMS residual.
                      This is the sharpest test available, because a wall
                      really is flat. Any curvature the camera reports — and
                      stereo systems classically report a bowl or a saddle from
                      residual lens-distortion error — is pure sensor error
                      with nothing else it could be.

    Repeat at 1, 2, 5, 10 and 15 m. Those distances all sit inside the ZED X's
    IDEAL depth range of 1 m to 20 m; the camera keeps reporting depth out to
    its rated maximum of 35 m, but Stereolabs stops calling that ideal and
    publishes no accuracy figure past 20 m. See `config/depth_targets.yaml`
    for why 20 m is worth adding to the list and why it has not been added
    here unilaterally.

WHAT COUNTS AS PASSING — AND WHY IT IS A BOUND, NOT A TARGET
    The datasheet publishes exactly two depth-accuracy figures for our column
    of the table (ZED X body, 4.6 mm lens), and both are inequalities:

        Depth Accuracy    < 0.4% to 2 m  (6.6 ft)
                          < 7%   at 20 m (65.6 ft)

    Note the "<". Neither figure says how accurate the camera is; each says
    the worst the manufacturer will allow it to be. So the curve this module
    draws through them is an UPPER BOUND, and a measurement inside it proves
    only one thing: the camera is not out of specification. It is not evidence
    that the camera is performing well, and a reading far inside the bound
    says more about how loose the bound is than about the hardware. That is
    why the function is called `datasheet_bound_m` rather than
    `spec_limit_m`, and why the result key is `within_datasheet_bound` rather
    than `within_spec`.

    Being OUTSIDE the bound is the informative case. It means something in the
    chain is wrong, and the failure-modes section of the package documentation
    lists the usual culprits.

    THE NUMBERS THAT USED TO BE HERE WERE NOT REAL. Until 2026-08-29 this
    module interpolated between "0.2% at 1 m" and "3.1% at 15 m", described in
    three separate places as datasheet reference points. Neither figure is in
    the datasheet: it has no 1 m entry and no 15 m entry for any model. The
    invented pair is twice as tight as the real bound at 1 m and about 37%
    too tight at 15 m — tighter, in every case, than the manufacturer's own
    limit. Real hardware was about to be judged against a fabricated
    tolerance, in the strict direction, so a perfectly healthy camera could
    have been failed by this script. The before-and-after numbers are
    tabulated in `datasheet_bound_m`.

MEASURING THE TRUE DISTANCE — THE PART PEOPLE GET WRONG
    The reference distance is measured from the **left imager's optical
    centre**, not from the front of the housing, the tripod screw, or the
    mounting plate. On the ZED X those differ by a couple of centimetres, which
    is ten times the entire error budget at 1 m. Measure to the marked lens
    plane, record the tape measure's own resolution, and put both in the log.

HARDWARE STATUS
    The ZED X is not wired at the time of writing. Run this the day it is. The
    node detects the camera's absence and says so specifically rather than
    hanging or throwing.
"""
from __future__ import annotations

import sys
from pathlib import Path

from eval_common import (REPO, MissingDependency, RunLogger, load_config,
                         require_numpy, write_json)

DEFAULT_CONFIG = Path(__file__).resolve().parent.parent / "config" / "depth_targets.yaml"

# --------------------------------------------------------------------------- #
# THE CAMERA SPECIFICATION LIVES IN ONE PLACE, AND THIS FILE IS NOT IT.
#
#     catkin_ws/src/sidewalk_perception/scripts/perception_common.py  ->  ZEDX
#
# That dict is transcribed field by field from
#
#     Official Resources/ZED X - ZED X Mini - Datasheet.pdf   (Rev 1.6)
#
# with a per-field [VERIFIED — section] note saying which table each number
# came out of. This module imports it instead of restating it, because the
# specification previously existed as four independent copies and they had
# already drifted: the copy in this package said 4 mm lens, 80 degree
# horizontal field of view and f/1.8 — none of which Stereolabs sells for this
# body — and carried two depth-error figures that are in no datasheet at all.
# Duplication was the root defect, so the duplicate is removed rather than
# corrected in place.
#
# The import is safe off-robot: perception_common imports nothing from ROS at
# module scope, deliberately, so `generate_report.py` can still be run on a
# computer with no ROS installed.
# --------------------------------------------------------------------------- #

# The two anchor DISTANCES are not stored as their own fields in ZEDX; they are
# encoded in the names of the two accuracy keys, `depth_err_to_2m_pct` and
# `depth_err_at_20m_pct`. They are named here so the arithmetic below reads
# plainly, and the key names are looked up through these constants so the two
# can never silently disagree.
SPEC_NEAR_M = 2.0
SPEC_FAR_M = 20.0
_NEAR_KEY = "depth_err_to_2m_pct"
_FAR_KEY = "depth_err_at_20m_pct"

# LOUD FALLBACK. Used only if the import above fails — on a partial checkout,
# or if this package is copied somewhere on its own. It is a duplicate, which
# is the very thing being removed, so it is kept to the minimum this module
# reads, it is announced on stderr, it is recorded in every results file it
# touches, and it is CHECKED against the real dict whenever the real dict is
# importable (see `_mirror_drift`) so it cannot quietly rot.
_FALLBACK_ZEDX = {
    "model": "ZED X",
    "sku": "ZED-312110",
    "lens_mm": 4.6,
    "aperture_f": 2.0,
    "polarizer": False,
    "baseline_mm": 120.0,
    "fov_h_deg": 74.7,
    "fov_v_deg": 51.0,
    "depth_min_m": 1.0,
    "depth_max_m": 35.0,
    "depth_ideal_max_m": 20.0,
    _NEAR_KEY: 0.4,
    _FAR_KEY: 7.0,
}


def _mirror_drift(authoritative: dict) -> list:
    """Fields where the local fallback mirror disagrees with the real dict.

    A mirror that is never compared is a mirror that is already wrong and
    nobody has noticed. This runs once at import, costs microseconds, and is
    the only thing standing between "we removed the duplicate" and "we made a
    fifth one".
    """
    bad = []
    for key, mine in _FALLBACK_ZEDX.items():
        theirs = authoritative.get(key, "<missing>")
        if isinstance(mine, float) and isinstance(theirs, (int, float)):
            if abs(float(theirs) - mine) > 1e-9:
                bad.append(f"{key}: mirror {mine} vs source {theirs}")
        elif theirs != mine:
            bad.append(f"{key}: mirror {mine!r} vs source {theirs!r}")
    return bad


def _load_camera_spec():
    """Return (spec dict, provenance sentence) for the ZED X.

    The provenance sentence is written into the results JSON next to the
    pass/fail verdict. A threshold whose origin is not recorded beside the
    verdict it produced is exactly the defect this module is being repaired
    for: the previous thresholds carried the word "datasheet" and no way to
    check it.
    """
    cand = REPO / "catkin_ws" / "src" / "sidewalk_perception" / "scripts"
    if cand.is_dir() and str(cand) not in sys.path:
        sys.path.insert(0, str(cand))
    try:
        from perception_common import ZEDX
    except Exception as exc:                               # noqa: BLE001
        sys.stderr.write(
            "\n"
            "  ==============================================================\n"
            "  WARNING: the authoritative camera constants could not be\n"
            "  imported, so a LOCAL COPY is being used instead.\n"
            f"      tried : {cand / 'perception_common.py'}\n"
            f"      reason: {exc.__class__.__name__}: {exc}\n"
            "\n"
            "  The copy is believed correct, but copies are how this file\n"
            "  came to be judging real hardware against invented numbers in\n"
            "  the first place. Every result written while this warning is\n"
            "  showing says so in its `spec_source` field. Fix the import\n"
            "  before publishing any depth figure:\n"
            "      ls catkin_ws/src/sidewalk_perception/scripts/\n"
            "  ==============================================================\n"
            "\n")
        return dict(_FALLBACK_ZEDX), (
            "LOCAL FALLBACK COPY inside depth_characterization.py — "
            "perception_common.ZEDX could not be imported. Treat any verdict "
            "produced under this provenance as provisional.")
    drift = _mirror_drift(ZEDX)
    if drift:
        sys.stderr.write(
            "\n  WARNING: the fallback mirror in depth_characterization.py has\n"
            "  drifted from perception_common.ZEDX. The authoritative dict is\n"
            "  being used, so this run is fine, but fix the mirror:\n"
            + "".join(f"      {d}\n" for d in drift) + "\n")
    return dict(ZEDX), (
        "sidewalk_perception/scripts/perception_common.py :: ZEDX, "
        "transcribed from 'ZED X - ZED X Mini - Datasheet.pdf' (Rev 1.6)")


ZEDX, SPEC_SOURCE = _load_camera_spec()

# Fractional form of the two published bounds. Both are "<" inequalities in the
# source document; see `datasheet_bound_m` for what that does to their meaning.
SPEC_NEAR_FRAC = float(ZEDX[_NEAR_KEY]) / 100.0        # 0.004  = "< 0.4% to 2 m"
SPEC_FAR_FRAC = float(ZEDX[_FAR_KEY]) / 100.0          # 0.07   = "< 7% at 20 m"


def datasheet_bound_m(distance_m: float) -> float:
    """UPPER BOUND on acceptable depth error at a distance, in metres.

    THIS IS A LIMIT, NOT AN EXPECTATION, and the distinction is the whole
    point of the rewrite. The datasheet's two entries for the ZED X 4.6 mm
    column are `< 0.4% to 2m` and `< 7% at 20m`. Both are inequalities, so a
    curve fitted through them bounds the error from above; it does not predict
    it. A camera reading well inside this curve is not thereby "accurate" —
    it is merely not out of specification, which is a much weaker claim. The
    caller stores the verdict as `within_datasheet_bound` for that reason.

    SHAPE OF THE CURVE, IN THREE PIECES.

      * at or below 2 m — flat 0.4%. The datasheet says "< 0.4% TO 2m", not
        "at 2m", and for this one column it is the only entry written that
        way (the other three models' columns all say "at"). Read literally it
        bounds the whole near range, so the bound at 1 m is 0.4% of 1 m, not
        some smaller interpolated figure. The old code interpolated down to
        0.2% at 1 m, halving the allowance at the exact distance where the
        allowance is already only millimetres.
      * 2 m to 20 m — log-log interpolation between the two bounds, i.e. a
        straight line on log-log axes.
      * beyond 20 m — EXTRAPOLATION. The datasheet bounds nothing past 20 m,
        even though the camera reports depth to 35 m. The value is still
        returned so the curve is continuous, but `bound_basis()` labels it
        and the label is written into the results file. Do not quote it as a
        specification.

    THE INTERPOLATION IS NOT THE PHYSICS, AND THE GAP IS INFORMATIVE. Stereo
    depth error grows with the SQUARE of range — error ≈ Z² · δd / (f · B),
    with Z the range, B the 120 mm baseline, f the focal length in pixels
    and δd the disparity-matching precision in pixels — so fractional error
    should grow linearly with Z. These two bounds do not: they imply
    fractional error ∝ Z^1.243, hence absolute error ∝ Z^2.243, steeper than
    quadratic. Fit the quadratic coefficient to each anchor separately and
    the two answers disagree:

        0.4% of 2 m  = 0.008 m  ->  a = 0.008 / 2²  = 0.0020 m⁻¹
        7%   of 20 m = 1.4   m  ->  a = 1.4   / 20² = 0.0035 m⁻¹

    That is expected of two independently-chosen worst-case bounds and is not
    a mistake in the datasheet, but it does mean this curve must not be
    reported as a noise model. The project's actual noise model is separate
    (`sidewalk_perception/obstacle_segmenter.range_noise_m`).

    Doubling the range roughly quadruples the depth error, and the only levers
    are a longer baseline (fixed at 120 mm by the housing), a longer focal
    length (fixed by the 4.6 mm lens and its 73 degree horizontal field of
    view), or better sub-pixel disparity — which is what the SDK's NEURAL
    depth modes buy, at a GPU cost that shows up in the latency figures.

    CHANGE IN BEHAVIOUR, 2026-08-29. Old anchors (1 m, 0.2%) and (15 m, 3.1%),
    both fabricated; new anchors (2 m, 0.4%) and (20 m, 7%), both quoted. The
    bound got LOOSER everywhere except at 2 m, where the two curves cross:

        distance    old limit    new bound    new / old
          1 m         2.00 mm      4.00 mm      2.00x
          2 m         8.07 mm      8.00 mm      0.99x
          5 m        50.98 mm     62.47 mm      1.23x
         10 m       205.65 mm    295.74 mm      1.44x
         15 m       465.00 mm    734.32 mm      1.58x
         20 m       829.55 mm   1400.00 mm      1.69x

    Every row except 2 m means the same measurement that would have been
    reported as OUT OF SPEC may now be reported as within the manufacturer's
    bound. That is a correction, not a relaxation: the old envelope was not a
    specification anybody published.
    """
    import math
    if not distance_m > 0:
        return float("nan")
    if distance_m <= SPEC_NEAR_M:
        frac = SPEC_NEAR_FRAC
    else:
        ratio = (math.log(distance_m / SPEC_NEAR_M)
                 / math.log(SPEC_FAR_M / SPEC_NEAR_M))
        frac = SPEC_NEAR_FRAC * (SPEC_FAR_FRAC / SPEC_NEAR_FRAC) ** ratio
    return frac * distance_m


def bound_basis(distance_m: float) -> str:
    """Which part of the datasheet the bound at this distance rests on.

    Recorded in the results JSON beside the verdict, so a reader can tell a
    quoted bound from an interpolated one from an extrapolated one without
    reading this file.
    """
    near_pct, far_pct = SPEC_NEAR_FRAC * 100.0, SPEC_FAR_FRAC * 100.0
    if not distance_m > 0:
        return "undefined — the distance must be positive"
    if distance_m <= SPEC_NEAR_M:
        return (f"[VERIFIED — datasheet 'ZED X Available Models', ZED X 4.6mm "
                f"column] quoted as '< {near_pct:g}% to {SPEC_NEAR_M:g}m', "
                f"i.e. a bound over the whole near range, so it is applied "
                f"flat rather than interpolated downward")
    if distance_m <= SPEC_FAR_M:
        return (f"[INFERENCE] log-log interpolation between the two quoted "
                f"bounds, '< {near_pct:g}% to {SPEC_NEAR_M:g}m' and "
                f"'< {far_pct:g}% at {SPEC_FAR_M:g}m'. The datasheet publishes "
                f"nothing between them")
    return (f"[UNVERIFIED] EXTRAPOLATED past {SPEC_FAR_M:g} m, the furthest "
            f"distance the datasheet bounds. The camera reports depth to "
            f"{float(ZEDX.get('depth_max_m', 35.0)):g} m but Stereolabs "
            f"guarantees nothing beyond {SPEC_FAR_M:g} m. Do not quote this "
            f"as a specification")


def decode_depth(msg, np):
    """sensor_msgs/Image -> a float32 numpy array in METRES, plus a note.

    Handles the two encodings the ZED ROS wrapper emits:
      * `32FC1` — metres already. Invalid pixels are NaN or Inf.
      * `16UC1` — millimetres as unsigned 16-bit. Invalid pixels are 0.

    Decoded by hand rather than through cv_bridge, deliberately: cv_bridge
    drags in an OpenCV whose Python bindings are a recurring source of version
    conflicts on JetPack, and this conversion is four lines. If cv_bridge is
    present and healthy it would give the same answer.
    """
    enc = (msg.encoding or "").lower()
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if enc in ("32fc1", "32fc"):
        arr = buf.view(np.float32).reshape(msg.height, msg.width).astype(np.float32)
        note = "32FC1, metres"
    elif enc in ("16uc1", "mono16"):
        raw = buf.view(np.uint16).reshape(msg.height, msg.width).astype(np.float32)
        arr = raw / 1000.0
        arr[raw == 0] = np.nan          # 0 is the ZED SDK's "no measurement"
        note = "16UC1, millimetres converted to metres"
    else:
        raise ValueError(
            f"Unsupported depth encoding '{msg.encoding}'. This node handles "
            f"32FC1 (metres) and 16UC1 (millimetres). Set the ZED wrapper's "
            f"`depth_mode`/`openni_depth_mode` accordingly, or add the "
            f"encoding here — but check the units first: silently "
            f"misinterpreting millimetres as metres produces a beautifully "
            f"self-consistent map that is 1000x the wrong size.")
    return arr, note


def analyse_patch(depth_m, fx, fy, cx, cy, roi, truth_m, np) -> dict:
    """Compute bias, noise, dropout and flatness for one region of interest.

    `roi` is (u0, v0, u1, v1) in pixels. `truth_m` is the tape-measured range.
    """
    u0, v0, u1, v1 = roi
    patch = depth_m[v0:v1, u0:u1]
    total = patch.size
    if total == 0:
        return {"error": "region of interest is empty — check roi_fraction"}

    valid_mask = np.isfinite(patch) & (patch > 0.05) & (patch < 40.0)
    n_valid = int(np.count_nonzero(valid_mask))
    invalid_pct = 100.0 * (total - n_valid) / total

    if n_valid < 50:
        return {
            "error": (f"only {n_valid} of {total} pixels in the region "
                      f"returned a depth value ({invalid_pct:.1f}% invalid). "
                      f"A blank wall gives the stereo matcher nothing to match. "
                      f"Tape a newspaper, a poster or a projected pattern to "
                      f"it, or aim at a textured surface."),
            "invalid_pct": invalid_pct,
            "n_valid": n_valid,
        }

    vals = patch[valid_mask].astype(float)

    # Robust centre and spread. A wall at 5 m with one pixel that latched onto
    # the ceiling at 30 m would drag a plain mean by centimetres; the median and
    # the MAD do not care.
    mean = float(np.mean(vals))
    median = float(np.median(vals))
    std = float(np.std(vals))
    mad = float(np.median(np.abs(vals - median)))
    robust_std = 1.4826 * mad

    # Back-project the valid pixels into 3D using the pinhole model, then fit a
    # plane. Note this uses the RECTIFIED intrinsics from camera_info — if
    # rectification is off, the residual measures distortion, not depth noise.
    vs, us = np.nonzero(valid_mask)
    us = us + u0
    vs = vs + v0
    z = patch[valid_mask].astype(float)
    x = (us - cx) * z / fx
    y = (vs - cy) * z / fy
    pts = np.stack([x, y, z], axis=1)

    centroid = pts.mean(axis=0)
    cov = np.cov((pts - centroid).T)
    evals_, evecs = np.linalg.eigh(cov)
    # The eigenvector of the smallest eigenvalue is the plane normal: it is the
    # direction in which the point cloud is thinnest. For a flat wall the cloud
    # is a slab, and its thickness IS the sensor's depth noise.
    normal = evecs[:, 0]
    residual = (pts - centroid) @ normal
    flatness_rms = float(np.sqrt(np.mean(residual ** 2)))
    flatness_max = float(np.max(np.abs(residual)))

    # Angle between the wall normal and the camera's optical axis. If this is
    # more than a few degrees the wall is not square-on, and part of the depth
    # spread across the patch is real geometry rather than noise.
    axis = np.array([0.0, 0.0, 1.0])
    cosang = abs(float(np.dot(normal, axis)))
    tilt_deg = float(np.degrees(np.arccos(min(1.0, max(-1.0, cosang)))))

    bias = mean - truth_m if truth_m else None
    bound = datasheet_bound_m(truth_m) if truth_m else None

    out = {
        "truth_m": truth_m,
        "n_pixels": int(total),
        "n_valid": n_valid,
        "invalid_pct": invalid_pct,
        "mean_m": mean,
        "median_m": median,
        "std_m": std,
        "robust_std_m": robust_std,
        "min_m": float(np.min(vals)),
        "max_m": float(np.max(vals)),
        "flatness_rms_m": flatness_rms,
        "flatness_max_m": flatness_max,
        "wall_tilt_deg": tilt_deg,
        "roi_px": [int(u0), int(v0), int(u1), int(v1)],
    }
    if truth_m:
        out["bias_m"] = bias
        out["bias_pct"] = 100.0 * bias / truth_m
        # rather than kept as aliases: the old names carried values from an
        # envelope that no datasheet publishes, and a reader who saw them
        # would reasonably assume otherwise. Anything still emitting the old
        # names is out of date and should say so loudly rather than blend in.
        out["datasheet_bound_m"] = bound
        out["within_datasheet_bound"] = bool(abs(bias) <= bound)
        out["bound_basis"] = bound_basis(truth_m)
        out["spec_source"] = SPEC_SOURCE
    if tilt_deg > 5.0:
        out["warning"] = (
            f"the wall appears tilted {tilt_deg:.1f} degrees from square-on. "
            f"Across the region of interest that adds real depth variation "
            f"which will be counted as noise. Re-aim the camera perpendicular "
            f"to the wall before trusting the flatness figure.")
    return out


def self_test() -> int:
    """Check the acceptance curve against the datasheet, with no hardware.

    Needs no camera, no ROS, no numpy. It exists because the numbers this
    module used to enforce were never checked against anything — they were
    labelled "datasheet" and believed. Anything asserting a datasheet figure
    should be executable against that figure, so here it is.
    """
    print("\ndepth_characterization --self-test")
    print("=" * 72)
    print(f"camera constants from : {SPEC_SOURCE}")
    print(f"model                 : {ZEDX.get('model')} "
          f"(SKU {ZEDX.get('sku')}), {float(ZEDX.get('lens_mm', 0)):g} mm lens, "
          f"f/{float(ZEDX.get('aperture_f', 0)):g}, "
          f"{float(ZEDX.get('fov_h_deg', 0)):g} deg horizontal")
    print(f"published bounds      : < {SPEC_NEAR_FRAC * 100:g}% to "
          f"{SPEC_NEAR_M:g} m   and   < {SPEC_FAR_FRAC * 100:g}% at "
          f"{SPEC_FAR_M:g} m")
    print("-" * 72)

    fails = []

    def check(label, got, want, tol=1e-9):
        ok = abs(got - want) <= tol
        print(f"  {'PASS' if ok else 'FAIL'}  {label:<52} "
              f"{got:.6f} m")
        if not ok:
            fails.append(f"{label}: got {got!r}, expected {want!r}")

    # The two anchors must be reproduced EXACTLY, not approximately. If the
    # curve does not pass through the published points it is not the
    # datasheet's curve, whatever else it is.
    check(f"bound at {SPEC_NEAR_M:g} m equals {SPEC_NEAR_FRAC * 100:g}% of "
          f"{SPEC_NEAR_M:g} m",
          datasheet_bound_m(SPEC_NEAR_M), SPEC_NEAR_FRAC * SPEC_NEAR_M)
    check(f"bound at {SPEC_FAR_M:g} m equals {SPEC_FAR_FRAC * 100:g}% of "
          f"{SPEC_FAR_M:g} m",
          datasheet_bound_m(SPEC_FAR_M), SPEC_FAR_FRAC * SPEC_FAR_M)
    # "< 0.4% TO 2 m" is a bound over the near range, so the FRACTION must be
    # flat below 2 m rather than tapering toward zero. This is the assertion
    # the old code failed: it interpolated down to 0.2% at 1 m.
    check("near range is flat in percentage terms (1 m = 0.4% of 1 m)",
          datasheet_bound_m(1.0), SPEC_NEAR_FRAC * 1.0)

    mono = all(datasheet_bound_m(z) < datasheet_bound_m(z + 0.25)
               for z in [x / 4.0 for x in range(2, 140)])
    print(f"  {'PASS' if mono else 'FAIL'}  "
          f"{'bound increases monotonically from 0.5 m to 35 m':<52}")
    if not mono:
        fails.append("bound is not monotonic in distance")

    nan = datasheet_bound_m(0.0)
    ok = nan != nan                                   # NaN is not equal to NaN
    print(f"  {'PASS' if ok else 'FAIL'}  "
          f"{'a non-positive distance returns NaN, not a number':<52}")
    if not ok:
        fails.append(f"datasheet_bound_m(0.0) returned {nan!r}, expected NaN")

    print("-" * 72)
    print(f"{'distance':>10} {'bound':>12} {'bound':>12} {'fraction':>10}   basis")
    print(f"{'[m]':>10} {'[m]':>12} {'[mm]':>12} {'[%]':>10}")
    for z in (1.0, 2.0, 5.0, 10.0, 15.0, 20.0, 25.0):
        b = datasheet_bound_m(z)
        basis = bound_basis(z).split("]")[0].lstrip("[")
        print(f"{z:>10.1f} {b:>12.6f} {b * 1000:>12.2f} "
              f"{100 * b / z:>10.3f}   {basis}")
    print("=" * 72)
    if fails:
        print("SELF-TEST FAILED:")
        for f in fails:
            print(f"  * {f}")
        return 1
    print("Self-test passed. This checks the ACCEPTANCE CURVE only; it says\n"
          "nothing about any camera, and no camera is required to run it.\n")
    return 0


def main() -> int:
    import argparse
    ap = argparse.ArgumentParser(
        description="Characterise ZED X depth accuracy against a flat wall.")
    ap.add_argument("--self-test", action="store_true",
                    help="check the datasheet acceptance curve and exit. "
                         "Needs no camera, no ROS and no numpy.")
    ap.add_argument("--distance", type=float, default=None,
                    help="nominal distance to the wall in metres "
                         "(1, 2, 5, 10 or 15). Required unless --self-test.")
    ap.add_argument("--truth", type=float, default=0.0,
                    help="the ACTUAL tape-measured distance from the LEFT "
                         "imager's lens plane to the wall. Defaults to "
                         "--distance, but measure it: the difference between "
                         "'about 5 m' and 4.97 m is larger than the entire "
                         "error budget at short range.")
    ap.add_argument("--frames", type=int, default=200,
                    help="frames to average (default 200, about 7 s at 30 FPS)")
    ap.add_argument("--timeout", type=float, default=20.0,
                    help="seconds to wait for the first depth frame")
    ap.add_argument("--config", default=str(DEFAULT_CONFIG))
    ap.add_argument("--out", default="",
                    help="results JSON to create or append to")
    ap.add_argument("--note", default="",
                    help="free text recorded with the measurement, e.g. "
                         "'indoor, painted concrete, newspaper taped on'")
    args = ap.parse_args()

    if args.self_test:
        return self_test()
    # `--distance` was `required=True` until --self-test existed. argparse
    # cannot express "required unless another flag", so the check is explicit
    # here rather than letting a missing distance surface later as a
    # TypeError on None.
    if args.distance is None:
        ap.error("--distance is required (or use --self-test, which needs no "
                 "camera)")

    # Free-text arguments may arrive from roslaunch still wrapped in the quotes
    # the launch file needed to protect their spaces. See eval_common.unquote.
    from eval_common import unquote
    args.note = unquote(args.note)
    args.out = unquote(args.out)

    truth = args.truth if args.truth > 0 else args.distance

    with RunLogger("sidewalk_evaluation",
                   run_name=f"depth_{args.distance:g}m") as log:
        try:
            np = require_numpy(log)
        except MissingDependency as exc:
            print(f"\nERROR: {exc}\n", file=sys.stderr)
            log.summary("Cannot characterise depth: numpy missing", status="FAIL")
            return 3
        try:
            from eval_common import require_rospy
            rospy = require_rospy(log)
            from sensor_msgs.msg import CameraInfo, Image
        except (MissingDependency, ImportError) as exc:
            print(f"\nERROR: {exc}\n", file=sys.stderr)
            log.summary("Cannot characterise depth: ROS unavailable",
                        status="FAIL")
            return 3

        try:
            cfg = load_config(args.config)
        except FileNotFoundError as exc:
            log.error(str(exc))
            log.summary("Configuration missing", status="FAIL")
            return 2

        cam_ns = str(cfg.get("camera_namespace", "/zedx_front/zed_node"))
        depth_topic = str(cfg.get("depth_topic", f"{cam_ns}/depth/depth_registered"))
        info_topic = str(cfg.get("camera_info_topic", f"{cam_ns}/depth/camera_info"))
        roi_frac = float(cfg.get("roi_fraction", 0.25))

        log.section("Depth characterization")
        log.info(f"Nominal distance {args.distance:g} m, "
                 f"tape-measured truth {truth:.3f} m")
        log.info(f"Depth topic : {depth_topic}")
        log.info(f"Info topic  : {info_topic}")
        if args.note:
            log.info(f"Conditions  : {args.note}")
        if args.truth <= 0:
            log.warn("--truth was not given, so the nominal distance is being "
                     "used as ground truth. Measure the real distance to the "
                     "LEFT imager's lens plane and pass it; at 1 m the whole "
                     "datasheet bound is 4 mm wide (0.4% of 1 m), which is "
                     "smaller than the offset between the lens plane and the "
                     "front of the housing.")

        rospy.init_node("depth_characterization", anonymous=True)

        # ------------------------------------------------------ camera_info
        # Intrinsics are read live from camera_info and never hardcoded. The
        # ZED SDK publishes the intrinsics for the resolution actually being
        # streamed; a hardcoded set silently becomes wrong the first time
        # somebody changes resolution, and the error looks like a depth bias.
        log.info("Waiting for camera_info...")
        try:
            info = rospy.wait_for_message(info_topic, CameraInfo,
                                          timeout=args.timeout)
        except Exception:                                  # noqa: BLE001
            log.error(
                f"No CameraInfo on {info_topic} within {args.timeout:.0f} s.\n"
                f"  * Is the ZED X physically connected? It uses a GMSL2 FAKRA "
                f"Z coax to the ZED Link Quad card, which needs its own 9-19 V "
                f"supply — the Jetson does not power it.\n"
                f"  * Is the daemon running?  systemctl status zed_x_daemon\n"
                f"  * Is the wrapper running? rosnode list | grep zed\n"
                f"  * Is the topic named differently? rostopic list | grep -i "
                f"camera_info\n"
                f"  * Run tools/health_check.sh for a full diagnosis.")
            log.summary("No camera detected — depth characterization aborted",
                        status="FAIL")
            return 4

        fx, fy = float(info.K[0]), float(info.K[4])
        cx, cy = float(info.K[2]), float(info.K[5])
        log.info(f"Intrinsics from camera_info: {info.width}x{info.height}, "
                 f"fx={fx:.2f} fy={fy:.2f} cx={cx:.2f} cy={cy:.2f}")
        if fx <= 0 or fy <= 0:
            log.error("camera_info reports a zero or negative focal length. "
                      "The camera is publishing but its calibration is not "
                      "loaded — check the SDK's calibration file for this "
                      "serial number.")
            log.summary("Invalid camera intrinsics", status="FAIL")
            return 4

        # A central ROI. Central because the edges of the 73 degree horizontal
        # field of view carry the most residual distortion — the datasheet
        # quotes -6.7% TV distortion for this lens — and because the centre is
        # where the wall is most reliably square-on.
        w, h = int(info.width), int(info.height)
        rw, rh = int(w * roi_frac), int(h * roi_frac)
        roi = (w // 2 - rw // 2, h // 2 - rh // 2,
               w // 2 + rw // 2, h // 2 + rh // 2)
        log.info(f"Region of interest: {rw}x{rh} px at image centre "
                 f"({roi_frac * 100:.0f}% of each dimension)")

        # --------------------------------------------------------- collect
        frames = []
        encoding_note = ""
        log.info(f"Collecting {args.frames} depth frames — hold the camera "
                 f"still...")
        collected = {"n": 0}

        def cb(msg):
            nonlocal encoding_note
            if collected["n"] >= args.frames:
                return
            try:
                arr, note = decode_depth(msg, np)
            except ValueError as exc:
                log.error(str(exc))
                rospy.signal_shutdown("unsupported depth encoding")
                return
            encoding_note = note
            frames.append(arr[roi[1]:roi[3], roi[0]:roi[2]].copy())
            collected["n"] += 1

        sub = rospy.Subscriber(depth_topic, Image, cb, queue_size=5)
        deadline = rospy.Time.now().to_sec() + args.timeout + args.frames / 5.0
        rate = rospy.Rate(20)
        while (not rospy.is_shutdown() and collected["n"] < args.frames
               and rospy.Time.now().to_sec() < deadline):
            rate.sleep()
        sub.unregister()

        if not frames:
            log.error(
                f"No depth images arrived on {depth_topic}. CameraInfo was "
                f"publishing, so the camera is alive but depth is not being "
                f"produced. Most likely the ZED wrapper was launched with "
                f"depth disabled, or `depth_mode` is NONE. Check the wrapper's "
                f"`common.yaml`, then `rostopic hz {depth_topic}`.")
            log.summary("Camera present but no depth stream", status="FAIL")
            return 4
        if len(frames) < args.frames:
            log.warn(f"Only {len(frames)} of {args.frames} requested frames "
                     f"arrived before the timeout. The statistics below are "
                     f"computed on what was received.")

        # ---------------------------------------------------------- analyse
        # Two passes with different meanings, and it matters which one is
        # quoted where:
        #   * the temporal mean image averages the noise away, and is the right
        #     basis for BIAS and for FLATNESS (systematic shape).
        #   * the per-frame scatter is the right basis for NOISE, which is what
        #     a single-frame SLAM measurement actually sees.
        stack = np.stack(frames, axis=0)
        with np.errstate(invalid="ignore"):
            mean_img = np.nanmean(stack, axis=0)

        full = np.full((h, w), np.nan, dtype=np.float32)
        full[roi[1]:roi[3], roi[0]:roi[2]] = mean_img
        result = analyse_patch(full, fx, fy, cx, cy, roi, truth, np)

        if "error" in result:
            log.error(result["error"])
            log.summary(f"Depth characterization at {args.distance:g} m failed: "
                        f"insufficient valid pixels", status="FAIL")
            return 5

        # Temporal noise: the standard deviation of one pixel across frames,
        # averaged over the patch. Distinct from the spatial std above.
        with np.errstate(invalid="ignore"):
            temporal = np.nanstd(stack, axis=0)
        result["temporal_std_m"] = float(np.nanmean(temporal))
        result["nominal_m"] = args.distance
        result["n_frames"] = len(frames)
        result["encoding"] = encoding_note
        result["note"] = args.note
        result["intrinsics"] = {"fx": fx, "fy": fy, "cx": cx, "cy": cy,
                                "width": w, "height": h,
                                "source": "live camera_info (never hardcoded)"}

        log.section(f"Results at {args.distance:g} m")
        for k, unit, digits in (("mean_m", "m", 4), ("bias_m", "m", 4),
                                ("bias_pct", "%", 3), ("std_m", "m", 4),
                                ("temporal_std_m", "m", 4),
                                ("invalid_pct", "%", 2),
                                ("flatness_rms_m", "m", 5),
                                ("wall_tilt_deg", "deg", 2)):
            if result.get(k) is not None:
                log.metric(f"depth_{args.distance:g}m_{k}",
                           round(float(result[k]), digits), unit)

        if result.get("within_datasheet_bound") is False:
            log.warn(
                f"Bias {result['bias_m'] * 100:+.2f} cm exceeds the "
                f"manufacturer's upper bound of "
                f"±{result['datasheet_bound_m'] * 100:.2f} cm at "
                f"{truth:.2f} m ({result['bound_basis']}). "
                f"Before blaming the camera, check in this "
                f"order: (1) was the distance measured to the LEFT imager's "
                f"lens plane; (2) is the wall square-on (tilt reported above "
                f"is {result.get('wall_tilt_deg', 0):.1f} deg); (3) is the "
                f"streamed resolution the one the calibration was made for; "
                f"(4) is either lens dirty or fogged.")
        if result.get("warning"):
            log.warn(result["warning"])

        # -------------------------------------------------- accumulate JSON
        out_path = Path(args.out) if args.out else \
            (log.repo / "logs" / "sidewalk_evaluation" /
             "depth_characterization.json")
        existing = {"measurements": [], "notes": []}
        if out_path.exists():
            try:
                import json
                existing = json.loads(out_path.read_text())
            except Exception:                              # noqa: BLE001
                log.warn(f"Could not parse the existing {out_path.name}; "
                         f"starting a fresh file. The old one is not deleted.")
        # Replace any previous measurement at the same nominal distance, so
        # repeating a distance corrects it rather than duplicating it.
        existing.setdefault("measurements", [])
        existing["measurements"] = [
            m for m in existing["measurements"]
            if abs(float(m.get("nominal_m", -1)) - args.distance) > 1e-6]
        existing["measurements"].append(result)
        existing.setdefault("notes", [])
        spec_note = (
            f"Acceptance is an UPPER BOUND, not an expected error. The ZED X "
            f"datasheet publishes two inequalities for this model: "
            f"< {SPEC_NEAR_FRAC * 100:g}% to {SPEC_NEAR_M:g} m and "
            f"< {SPEC_FAR_FRAC * 100:g}% at {SPEC_FAR_M:g} m. Below "
            f"{SPEC_NEAR_M:g} m the near bound is applied flat (the datasheet "
            f"says 'to', not 'at'); between the two it is interpolated on "
            f"log-log axes; past {SPEC_FAR_M:g} m it is extrapolated and "
            f"unsupported. A measurement inside the bound shows only that the "
            f"camera is not out of specification — it is not a performance "
            f"figure, and this curve is not a noise model. Source: "
            f"{SPEC_SOURCE}.")
        # Drop any note from the withdrawn 0.2%/1 m and 3.1%/15 m envelope, so
        # a results file that survives the correction does not keep asserting
        # a datasheet provenance that was never real.
        existing["notes"] = [
            n for n in existing["notes"]
            if "0.2%" not in str(n) and "3.1%" not in str(n)]
        if spec_note not in existing["notes"]:
            existing["notes"].append(spec_note)
        existing["camera"] = (
            f"Stereolabs {ZEDX.get('model', 'ZED X')}, SKU "
            f"{ZEDX.get('sku', 'ZED-312110')}, "
            f"{float(ZEDX.get('baseline_mm', 120.0)):g} mm baseline, "
            f"{float(ZEDX.get('lens_mm', 4.6)):g} mm lens, "
            f"f/{float(ZEDX.get('aperture_f', 2.0)):g}, "
            f"{float(ZEDX.get('fov_h_deg', 73.0)):g} deg horizontal field of "
            f"view, no polarizer")
        existing["spec_source"] = SPEC_SOURCE
        write_json(out_path, existing)
        log.info(f"Accumulated results: {out_path}")

        done = sorted(float(m["nominal_m"]) for m in existing["measurements"])
        want = [float(x) for x in (cfg.get("distances_m") or [1, 2, 5, 10, 15])]
        remaining = [d for d in want if not any(abs(d - x) < 1e-6 for x in done)]
        if remaining:
            log.info(f"Still to measure: "
                     f"{', '.join(f'{d:g} m' for d in remaining)}")
        else:
            log.info("All configured distances measured. Build the report with "
                     "`generate_report.py`.")

        status = "OK" if result.get("within_datasheet_bound", True) else "WARN"
        log.summary(
            f"Depth at {args.distance:g} m: bias {result['bias_m'] * 100:+.2f} cm "
            f"({result['bias_pct']:+.2f}%), sigma {result['std_m'] * 100:.2f} cm, "
            f"{result['invalid_pct']:.1f}% invalid, flatness "
            f"{result['flatness_rms_m'] * 1000:.1f} mm RMS — "
            f"{'within' if result.get('within_datasheet_bound') else 'OUTSIDE'} "
            f"the datasheet upper bound "
            f"(±{(result.get('datasheet_bound_m') or 0) * 100:.2f} cm)",
            status=status)
        return 0


if __name__ == "__main__":
    sys.exit(main())
