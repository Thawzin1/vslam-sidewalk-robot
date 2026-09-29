#!/usr/bin/env python3
"""
perception_common.py — shared plumbing for every node in `sidewalk_perception`.

WHY THIS EXISTS
    The four executables in this package (sensor_validator, frame_audit,
    svo_manager, make_manifest) all need the same four awkward things:

      1. The shared RunLogger, which lives in a *different* package. Importing
         across packages is not something ROS 1 makes pleasant for Python, so
         the search-and-fallback dance is written once, here.

      2. A way to fail *usefully* when hardware or software is absent. The
         camera is not wired yet, the robot is offline, and this code is partly
         edited on a computer with no ROS at all. A traceback in that situation
         is a bug, not a diagnosis. Everything here raises `Unavailable`, which
         carries a plain-language reason and a suggested fix.

      3. Environment provenance — L4T version, ZED SDK version, driver version,
         camera serial. The dataset manifest is worthless if it does not record
         which software produced the data, and six months from now nobody will
         remember. So we probe it automatically, and record "unknown" honestly
         rather than guessing.

      4. Streaming statistics. Validating a 15-minute soak run at the ZED X's
         200 Hz IMU rate means 180,000 samples; we want mean, spread and tail
         percentiles without holding the whole run in a naive Python list of
         dicts.

    Nothing in this module imports rospy at module scope. That is deliberate:
    `make_manifest.py` and the HTML generator must work on a machine with no
    ROS installed, because documentation is written long before the robot runs.

HARDWARE THIS PACKAGE ASSUMES
    Stereolabs ZED X, SKU ZED-312110 — 4.6 mm lens, no polarizer, 120 mm stereo
    baseline, dual global-shutter colour sensors, 2x1920x1200 up to 60 FPS,
    73 deg horizontal FOV, f/2.0, usable depth 1-35 m, integrated 200 Hz IMU.
    Connected over GMSL2 (FAKRA Z, power over coax) to a ZED Link Quad capture
    card (SKU ACC-212000) in a Jetson AGX Orin running L4T R35.6.1.

    Every figure in that sentence is read off `Official Resources/ZED X - ZED X
    Mini - Datasheet.pdf` (Rev 1.6). It used to say "4 mm ... ~80 deg ... f/1.8
    ... 400 Hz", which is the ZED X's optical block as somebody remembered it,
    not as the manufacturer publishes it. See the ZEDX dict below for the
    per-field provenance and for what the wrong numbers cost.
"""
from __future__ import annotations

import json
import math
import os
import re
import subprocess
import sys
from datetime import datetime
from pathlib import Path

# --------------------------------------------------------------------------- #
# Authoritative hardware constants.
#
# THE SOURCE, AND WHY THAT SENTENCE IS NOT A FORMALITY
#     Every value below is read off ONE document:
#
#         Official Resources/ZED X - ZED X Mini - Datasheet.pdf   (Rev 1.6)
#
#     and each entry names the section it came from. Reproduce any of them with
#
#         pdftotext -layout "Official Resources/ZED X - ZED X Mini - Datasheet.pdf" -
#
#     WHICH CAMERA THIS IS, SETTLED. Our unit is SKU ZED-312110. The datasheet
#     pins that SKU down twice and the two agree: the "ZED X Available Models"
#     table lists ZED-312110 under "SKU No Polarizer" in the "ZED X 4.6mm"
#     column, and the later "ZED X Part Numbers" table lists the same number
#     under ZED X / "Narrow lens (4.6mm)" / "No Polarizer". So: ZED X body,
#     4.6 mm lens, no polarising filter. Not the Mini, not the 2.2 mm wide lens.
#     That identification is what makes the whole "ZED X 4.6mm" column of the
#     specification table ours, including the 1.0 m minimum depth this project
#     already relies on everywhere.
#
#     The optical block of this dict was written from memory, not from the
#     datasheet, and every one of its five optical numbers was wrong:
#
#         lens_mm     4.0  -> 4.6      fov_h_deg  80 -> 73
#         aperture_f  1.8  -> 2.0      fov_v_deg  52 -> 45
#                                      fov_d_deg  91 -> 87
#
#     The vertical figure is the one that mattered. 52 deg EXCEEDS the
#     datasheet's stated maximum of 45 deg, so it was not "right at some
#     resolutions and wrong at others" — it was wrong everywhere, and wrong in
#     the optimistic direction. obstacle_segmenter.py asks where the ground
#     first enters the picture, h / tan(pitch + halfFOV); a half-FOV inflated
#     from 22.5 to 26 deg tilts the bottom of the frame further down than the
#     lens can actually see, and reports the ground appearing 1.24 m ahead of
#     the lens when the truth is 1.44 m. The near blind zone — the band of
#     pavement in front of the bumper this robot NEVER observes — was therefore
#     under-reported by 20 cm (0.818 m claimed, 1.018 m real). That is the same
#     failure, in the same direction, as the 132 mm camera-height bug fixed
#     earlier the same day: a safety number made to look better than it is.
#
#     imu_rate_hz was also wrong: 400 Hz, against a datasheet that says 200 Hz
#     in two separate places. That one is not merely cosmetic either — see the
#     entry below.
#
# THESE ARE NOT INTRINSICS AND MUST NEVER BE USED AS INTRINSICS.
#     fx, fy, cx, cy and the true baseline always come from live `camera_info`,
#     because the per-unit factory calibration differs from these nominal
#     optics by enough to matter at 15 m. What the values below are FOR is
#     sanity checks ("is the delivered focal length in the right postcode?"),
#     the geometric physics in the docs, and the dataset manifest.
# --------------------------------------------------------------------------- #
ZEDX = {
    # --- identity ---------------------------------------------------------- #
    # [VERIFIED — datasheet "ZED X Available Models" + "ZED X Part Numbers"]
    "model": "ZED X",
    "sku": "ZED-312110",
    "lens_mm": 4.6,          # [VERIFIED — "Lens Options"/"Focal Length": 4.6mm
                             # (0.18"). WAS 4.0 — wrong, and 4.0 mm is not an
                             # option Stereolabs sells for this body at all;
                             # the two lenses are 2.2 mm and 4.6 mm.]
    "polarizer": False,      # [VERIFIED — ZED-312110 is the "No Polarizer" SKU]

    # --- stereo geometry --------------------------------------------------- #
    # [VERIFIED — "Technical Specifications"/"Baseline": ZED X 12cm (4.72").
    # This was the one optical constant that was already right. It is also the
    # number that sets metric scale for the whole project, so it is worth
    # saying plainly that it has now been checked against the document rather
    # than inherited: 12 cm = 120.0 mm.]
    "baseline_mm": 120.0,

    # --- imaging ----------------------------------------------------------- #
    # ARRAY SIZE IS NOT OUTPUT SIZE, AND CONFUSING THEM SHIFTS THE PRINCIPAL
    # POINT. The sensor array is 1928 x 1208; the camera outputs 1920 x 1200.
    # The eight-pixel difference is border the ISP crops. Everything downstream
    # (region of interest, decimation stride, ground sampling per pixel row)
    # must use the OUTPUT size, which is what max_width/max_height are.
    # [VERIFIED — "Dual Image Sensors": Array Size 1928 x 1208 pixels; Output
    # Resolution 1920x1200 @60fps.]
    "sensor_mp_each": 2.3,          # "1/2.6" 2.3MP RGB"
    "sensor_array_width": 1928,     # array, NOT what you receive
    "sensor_array_height": 1208,    # array, NOT what you receive
    "max_width": 1920,              # output — use this one
    "max_height": 1200,             # output — use this one
    "max_fps": 60,                  # at 1920x1200; 120 fps only at 960x600 binned

    # --- optics ------------------------------------------------------------ #
    # [VERIFIED — "ZED X Available Models", ZED X 4.6mm column:
    #  "Field of View  Max. 73 deg (H) x 45 deg (V) x 87 deg (D)", "Aperture f/2.0".]
    #
    # NOTE THE WORD "Max." IN THE DATASHEET. These are the widest angles the
    # optics deliver, at the full 16:10 output. The 1920x1080 mode is a CROP,
    # not a rescale, so it sees a narrower vertical angle — roughly
    # 45 * 1080/1200 = 40.5 deg — while the 960x600 mode is BINNED and keeps
    # the full 45 deg. Nothing in this package runs a cropped mode today, so
    # the single value is honest; if a 1080-height mode is ever configured,
    # every vertical-FOV consumer here (obstacle_segmenter's ground intercept
    # above all) needs the cropped angle passed in explicitly, and each of
    # those functions already accepts fov_v_deg as an argument for exactly
    # this reason.
    # ===================================================================
    # THE DATASHEET NUMBERS, AND THAT IS DELIBERATE.
    #
    # Earlier the same day these were changed from 80/52/91 to the
    # datasheet's 73/45/87, with a long justification. THAT WAS WRONG.
    # The camera's own factory calibration, read live off camera_info at
    # 1920x1200, reports:
    #
    #     rectified  (left/camera_info)      74.67 H   50.98 V
    #     RAW lens   (left_raw/camera_info)  74.17 H   50.58 V
    #     datasheet                          73.0  H   45.0  V
    #
    # The RAW stream carries non-zero distortion coefficients and still
    # reads ~74 x 51, so this is NOT a rectification artefact - it is what
    # the lens actually delivers. The implied focal length is 3.78 mm
    # (fx 1258.5 px x 3 um pixel pitch), not the datasheet's 4.6 mm, and
    # the datasheet's own three angles are not pinhole-consistent with each
    # other (73 and 45 imply an 80.6 deg diagonal, not the 87 printed).
    #
    # SO: for anything computed from what the camera OUTPUTS - where the
    # ground first appears, blind-zone depth, ground sampling per pixel -
    # the calibration wins and the datasheet does not. The datasheet
    # describes a product line; camera_info describes THIS unit.
    #
    # The cost of the mistake, recorded so it is not repeated: at 45 deg
    # the first ground return computes as 1.442 m, at the true 51 deg it is
    # 1.267 m. The "correction" overstated the blind zone by 175 mm and was
    # reported as a finding. The superseded 52.0 was within 1 deg of truth.
    #
    # RE-MEASURE ON ANY NEW CAMERA OR RESOLUTION - these are per-unit:
    #     rostopic echo -n1 /zedx_front/zed_node/left/camera_info
    #     HFOV = 2*atan(width/(2*fx)) ; VFOV = 2*atan(height/(2*fy))
    # ===================================================================
    "fov_h_deg": 74.7,       # MEASURED. datasheet says 73.0; was 80.0
    "fov_v_deg": 51.0,       # MEASURED. datasheet says 45.0; was 52.0
    "fov_d_deg": 84.0,       # DERIVED from the measured fx/fy, self-consistent
    "fov_source": "camera_info 2026-08-29, ZED-312110, 1920x1200 rectified",
    "fov_h_deg_datasheet": 73.0,   # kept for provenance, NOT for geometry
    "fov_v_deg_datasheet": 45.0,
    "focal_px_measured": 1258.5,   # fx = fy at 1920x1200, rectified
    "aperture_f": 2.0,       # WAS 1.8
    "tv_distortion_pct": -6.7,   # [VERIFIED — "TV Distortion  -6.7% <".
                                 # Relevant only to raw images; every topic this
                                 # project subscribes to is rectified.]

    # --- depth ------------------------------------------------------------- #
    # [VERIFIED — "ZED X Available Models", ZED X 4.6mm column:
    #  "Depth Range Max 1.0m to 35m", "Ideal Range 1.0m to 20m".]
    "depth_min_m": 1.0,
    "depth_max_m": 35.0,
    "depth_ideal_max_m": 20.0,   # beyond this the camera still reports depth,
                                 # but Stereolabs stops calling it ideal

    # DEPTH ACCURACY — THE KEYS THAT USED TO CARRY INVENTED NUMBERS.
    #
    # This dict previously held:
    #     "depth_err_at_1m_pct":  0.2
    #     "depth_err_at_15m_pct": 3.1
    # and the accompanying prose in three packages said those were "the
    # datasheet's two anchor points". THEY ARE NOT IN THE DATASHEET. There is
    # no 1 m figure and no 15 m figure anywhere in the document. What it
    # actually publishes for this column is two entirely different anchors:
    #
    #     "Depth Accuracy   < 0.4% to 2m (6.6ft)
    #                       < 7% at 20m (65.6ft)"
    #
    # The keys are therefore RENAMED to the ranges the manufacturer really
    # quotes, rather than left in place with a corrected value — a key called
    # `depth_err_at_1m_pct` cannot be made honest, because the datasheet says
    # nothing about 1 m.
    #
    # THE RENAME IS SAFE. THE NUMBERS ARE NOT CONTAINED. Those are two
    # different claims and an earlier version of this comment ran them
    # together, saying "nothing in the repository read either old key ... grep
    # confirms: the only hits were this dict". The first half is true and the
    # second half was false, in the way that matters most: it told the next
    # reader to stop looking.
    #
    #   TRUE — no other module imports the old KEY NAMES, so renaming them
    #   cannot break a caller:
    #
    #       $ git grep -n "depth_err_at_1m_pct\|depth_err_at_15m_pct"
    #       perception_common.py:179:    #     "depth_err_at_1m_pct":  0.2
    #       perception_common.py:180:    #     "depth_err_at_15m_pct": 3.1
    #       perception_common.py:191:    # `depth_err_at_1m_pct` cannot be ...
    #       docs/SOLVED.md:697:Two further keys, `depth_err_at_1m_pct: 0.2` ...
    #       docs/SOLVED.md:715:a key called `depth_err_at_1m_pct` cannot ...
    #
    #   (all five hits are this file's own prose and the write-up of it)
    #
    #   FALSE — the two invented VALUES were copied out of this dict long ago
    #   and hardcoded elsewhere, where they are not documentation at all:
    #
    #       sidewalk_evaluation/scripts/depth_characterization.py:66-67
    #           SPEC_NEAR_M, SPEC_NEAR_FRAC = 1.0,  0.002
    #           SPEC_FAR_M,  SPEC_FAR_FRAC  = 15.0, 0.031
    #       These are the live PASS/FAIL envelope of the depth-accuracy tool.
    #       A measurement is scored against them; they are not a comment.
    #
    #       sidewalk_evaluation/scripts/generate_report.py:336
    #           lo, hi = 0.002, 0.031
    #       The same envelope again, redrawn onto the report figure.
    #
    #       sidewalk_evaluation/config/depth_targets.yaml:86-87
    #           error_at_1m_pct:  0.2
    #           error_at_15m_pct: 3.1
    #
    #   Beyond those three, the pair is also quoted as prose in the multicam,
    #   navigation and evaluation packages and in docs/guide. Those are being
    #   corrected as separate work; sidewalk_evaluation is owned by another
    #   change in flight and is deliberately not touched from here.
    #
    # The lesson is the same one this whole correction is about: a confident
    # completeness claim ("grep confirms") is itself a specification, and it
    # has to be earned by running the command and reading the output. Renaming
    # a key does not delete the number it used to hold.
    #
    # WORTH KNOWING, because it explains where 0.2/3.1 probably came from.
    # Fit sigma = a*z^2 to each real anchor separately and the two disagree:
    #     0.4% of 2 m  = 0.008 m  ->  a = 0.008/2^2  = 0.0020 m^-1
    #     7%   of 20 m = 1.4 m    ->  a = 1.4/20^2   = 0.0035 m^-1
    # The old invented pair fits a = 0.0020 at BOTH points exactly, which is
    # what you get by picking a coefficient first and then writing down two
    # "datasheet" percentages that reproduce it. The consequence is flagged,
    # not silently fixed, in obstacle_segmenter.py's noise-model section:
    # 0.0020 matches the NEAR anchor and is optimistic at range.
    #
    # Note also the datasheet's own wording: "< 0.4% TO 2m", not "at 2m" — for
    # this column it reads as a bound over the near range rather than a point
    # measurement, and the other three columns in the same row say "at". We
    # quote it as the document writes it and do not resolve the ambiguity.
    "depth_err_to_2m_pct": 0.4,      # [VERIFIED — "< 0.4% to 2m (6.6ft)"]
    "depth_err_at_20m_pct": 7.0,     # [VERIFIED — "< 7% at 20m (65.6ft)"]

    # --- IMU --------------------------------------------------------------- #
    # RATE: 200 Hz, NOT 400. [VERIFIED — the datasheet says so twice:
    # "Technical Specifications"/"Motion Sensors": "200 Hz 16-bits Accelerometer
    # (up to 12g) / 200 Hz 16-bits Gyroscope (up to 1000 deg/s)", and
    # "Sensors Specifications"/"Motion Sensors": "Output Data Rate  200 Hz".]
    #
    # 400 Hz is the ZED 2 / ZED 2i figure and was almost certainly carried over
    # from that camera. It is not harmless here. sensor_validator.py checks the
    # achieved IMU rate against this constant with a 5% tolerance
    # (validation_thresholds.yaml: imu_rate_tolerance_pct), so a perfectly
    # healthy ZED X delivering its rated 200 Hz would have been reported as
    # 50% low and FAILED validation — the validator would have condemned the
    # correct hardware. And make_orbslam3_config.py writes this number into the
    # generated ORB-SLAM3 settings as IMU.Frequency, where it scales the
    # discrete-time inertial noise; see slam_common.ZEDX_IMU_RATE_HZ.
    #
    # [UNVERIFIED — the rate the ROS wrapper actually PUBLISHES.] The datasheet
    # states what the part produces; the zed-ros-wrapper can decimate or
    # interpolate, and `zedx_front.yaml` sets `max_pub_rate: 400.0` as a
    # ceiling. A ceiling above the source rate is a no-op, so no configuration
    # value needs to change — but the delivered rate is a fact about the live
    # system and the camera is switched off. Settle it with the camera running:
    #     rostopic hz /zedx_front/zed_node/imu/data
    "imu_rate_hz": 200.0,            # WAS 400.0
    "imu_accel_range_g": 12.0,       # [VERIFIED — "Accelerometer Range  +/- 12G"]
    "imu_gyro_range_dps": 1000.0,    # [VERIFIED — "Gyroscope Range  +/- 1000 dps"]
    "imu_accel_res_mg": 0.36,        # [VERIFIED — "Accelerometer Resolution  0.36 mg"]
    "imu_gyro_res_dps": 0.03,        # [VERIFIED — "Gyroscope Resolution  0.03 dps"]

    # THE THREE FIGURES THIS DICT WAS MISSING, AND WHY THEY UNBLOCK WORK.
    #
    # [VERIFIED — "Sensors Specifications" / "Motion Sensors" table:
    #     Accelerometer Noise Density   2.3 mg
    #     Gyroscope Noise Density       0.20 dps
    #     Sensitivity Error             +/- 0.5%  ]
    #
    # Three places in this repository asserted that Stereolabs publishes the
    # ranges and resolution but NOT the noise density, and concluded that a
    # 3+ hour Allan-variance run was required before any stereo-inertial
    # result could be believed. That assertion is FALSE — the figures are in
    # the table above, printed directly beneath the resolution figures those
    # same notes cited. The claim has been corrected in
    # sidewalk_slam/config/orbslam3_zedx_stereo_inertial.template.yaml and in
    # sidewalk_slam/explain.yaml.
    #
    # THE UNIT CAVEAT, STATED PLAINLY RATHER THAN GUESSED AT. A noise density
    # is conventionally quoted per square-root hertz — ug/sqrt(Hz),
    # dps/sqrt(Hz). THIS DATASHEET WRITES NEITHER DENOMINATOR. It says "2.3 mg"
    # and "0.20 dps" full stop. So the unit stored here is the unit printed,
    # and the key names say mg and dps and nothing more. Two readings are
    # possible and the document does not choose between them:
    #
    #   (a) per sqrt(Hz) as written. Then 2.3 mg/sqrt(Hz) = 0.0226
    #       m/s^2/sqrt(Hz) and 0.20 dps/sqrt(Hz) = 0.00349 rad/s/sqrt(Hz),
    #       which are 10x and 20x WORSE than typical consumer MEMS parts —
    #       implausible for a part the same page advertises as "ultra low
    #       noise".
    #   (b) total RMS noise over the sensor's output bandwidth. Divide by
    #       sqrt(BW); at the 200 Hz output rate a ~100 Hz bandwidth gives
    #       0.23 mg/sqrt(Hz) and 0.02 dps/sqrt(Hz), squarely in the normal
    #       band for this class of part.
    #
    # Reading (b) is the more plausible one on physical grounds, and this
    # comment says so — but "more plausible" is not "verified", so no
    # conversion is performed here and nothing downstream consumes these as
    # sqrt(Hz) densities. What WOULD settle it: Stereolabs' own support, or the
    # Allan-variance run, which additionally yields the bias-instability and
    # random-walk terms that this datasheet genuinely does not publish.
    "imu_accel_noise_density_mg": 2.3,     # unit as printed: mg, no per-sqrt-Hz
    "imu_gyro_noise_density_dps": 0.20,    # unit as printed: dps, no per-sqrt-Hz
    "imu_sensitivity_error_pct": 0.5,      # +/- 0.5%

    # --- connection -------------------------------------------------------- #
    # [VERIFIED — "Connector  Serial Coax GMSL2 connector - FAKRA Z type";
    # "Power  Power via GMSL2 (PoC), typical consumption 1.46W (0.122A 12V)".]
    "interface": "GMSL2 / FAKRA Z, power over coax",
    # [UNVERIFIED against a document — the capture card is not described in the
    # camera datasheet. These two strings came from the purchase record and are
    # descriptive metadata for the manifest, not physics.]
    "capture_card": "ZED Link Capture Card Quad",
    "capture_card_sku": "ACC-212000",
}

# Exit codes. Used consistently by every executable in this package so that a
# shell script or a CI job can tell "the measurement failed" apart from "the
# measurement could not be attempted".
EXIT_PASS = 0
EXIT_FAIL = 1          # ran to completion, and the data did not meet spec
EXIT_UNAVAILABLE = 2   # could not run at all — hardware or software missing
EXIT_INTERRUPTED = 3   # operator pressed Ctrl-C


class Unavailable(Exception):
    """Raised when something required is absent.

    Carries a `reason` (what is missing, specifically) and a `fix` (the next
    command the operator should actually type). Nodes catch this at top level,
    print both, and exit `EXIT_UNAVAILABLE`. This is the mechanism that keeps
    "camera not plugged in" from ever surfacing as a stack trace.
    """

    def __init__(self, reason: str, fix: str = ""):
        super().__init__(reason)
        self.reason = reason
        self.fix = fix


# --------------------------------------------------------------------------- #
# Repo location and the shared logger
# --------------------------------------------------------------------------- #

def find_repo_root(start=None) -> Path:
    """Locate the repository by landmark rather than by relative path.

    roslaunch does not promise a working directory, and an installed node runs
    from `install/lib/<pkg>/`, nowhere near the source tree. Searching upward
    for a known landmark is the only reliable option.
    """
    here = (Path(start) if start else Path(__file__).resolve()).parent
    for cand in [here, *here.parents]:
        if (cand / "tools" / "explain.py").exists() and (cand / "catkin_ws").exists():
            return cand
    env = os.environ.get("SIDEWALK_REPO")
    if env and Path(env).exists():
        return Path(env)
    default = Path.home() / "vslam-sidewalk-robot"
    return default if default.exists() else Path.cwd()


REPO = find_repo_root()

# The shared logger lives in sidewalk_bringup. Add its scripts directory to the
# import path before trying the import, so this works from source *and* from an
# install space where catkin has flattened both packages into lib/.
_BRINGUP_SCRIPTS = REPO / "catkin_ws" / "src" / "sidewalk_bringup" / "scripts"
if _BRINGUP_SCRIPTS.is_dir() and str(_BRINGUP_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_BRINGUP_SCRIPTS))

# Also expose sidewalk_bringup as a real package, so the canonical
# `from sidewalk_bringup.run_logger import ...` works from source too.
_BRINGUP_SRC = REPO / "catkin_ws" / "src" / "sidewalk_bringup" / "src"
if _BRINGUP_SRC.is_dir() and str(_BRINGUP_SRC) not in sys.path:
    sys.path.insert(0, str(_BRINGUP_SRC))

try:
    try:
        # Canonical import: works from a source workspace and
        # after catkin_make install, because sidewalk_bringup
        # exports src/ via catkin_python_setup().
        from sidewalk_bringup.run_logger import RunLogger
    except ImportError:
        from run_logger import RunLogger
except ImportError:  # pragma: no cover - only hit if bringup is missing
    class RunLogger(object):  # type: ignore
        """Minimal stand-in so this package still works standalone.

        Deliberately API-compatible with the real one. If you see log files
        whose header says FALLBACK, sidewalk_bringup was not found and the
        project-wide RUNLOG.md index is NOT being updated — fix the path
        rather than living with it.
        """

        def __init__(self, package, run_name="run", echo=True, repo_root=None):
            self.package, self.run_name, self.echo = package, run_name, echo
            self.repo = Path(repo_root) if repo_root else REPO
            self.dir = self.repo / "logs" / package
            self.dir.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
            self.path = self.dir / ("%s_%s.log" % (stamp, run_name))
            self.metrics_path = self.dir / ("%s_%s.metrics.json" % (stamp, run_name))
            self._metrics = {}
            self._closed = False
            self._w("=== FALLBACK LOGGER (sidewalk_bringup/run_logger.py not found) ===")
            self._w("RUN: %s  PACKAGE: %s  AT: %s"
                    % (run_name, package, datetime.now().isoformat()))

        def _w(self, line):
            with open(self.path, "a") as fh:
                fh.write(line + "\n")
            if self.echo:
                print(line, flush=True)

        def info(self, msg):
            self._w("INFO   " + str(msg))

        def warn(self, msg):
            self._w("WARN   " + str(msg))

        def error(self, msg):
            self._w("ERROR  " + str(msg))

        def metric(self, key, value, unit=""):
            self._metrics[key] = {"value": value, "unit": unit}
            self._w("METRIC %s = %s %s" % (key, value, unit))

        def section(self, title):
            self._w("\n--- %s ---" % title)

        def summary(self, text, status=""):
            if self._closed:
                return
            self._closed = True
            self._w("SUMMARY [%s] %s" % (status or "OK", text))
            if self._metrics:
                self.metrics_path.write_text(json.dumps(self._metrics, indent=2))

        def __enter__(self):
            return self

        def __exit__(self, et, ev, tb):
            if et is not None:
                self.error("Terminated by exception: %s: %s" % (et.__name__, ev))
                self.summary("Run failed: %s" % et.__name__, status="FAIL")
            else:
                self.summary("Run completed", status="OK")
            return False


# --------------------------------------------------------------------------- #
# ROS availability
# --------------------------------------------------------------------------- #

def require_rospy():
    """Import rospy, or explain precisely why we cannot.

    Three distinct failures get three distinct messages, because they have
    three completely different fixes and confusing them wastes hours:
      - ROS is not installed on this machine at all
      - ROS is installed but this shell never sourced it
      - ROS is sourced but roscore is not running
    The third is checked separately by `require_master()`.
    """
    try:
        import rospy  # noqa: F401
        return rospy
    except ImportError:
        if os.path.isdir("/opt/ros"):
            found = ", ".join(sorted(os.listdir("/opt/ros"))) or "nothing"
            raise Unavailable(
                "rospy is not importable, but ROS appears to be installed "
                "(/opt/ros contains: %s). This shell has not sourced it."
                % found,
                "source /opt/ros/noetic/setup.bash && "
                "source ~/catkin_ws/devel/setup.bash")
        raise Unavailable(
            "ROS is not installed on this machine (no /opt/ros). This node "
            "must run on the Jetson AGX Orin, which has ROS Noetic. It cannot "
            "run on a computer without the ROS setup.",
            "Run this on the Jetson, or use the offline tools instead: "
            "frame_audit.py --bag <file> and make_manifest.py both work "
            "without ROS.")


def require_master(rospy, timeout_s: float = 3.0):
    """Confirm a ROS master is actually reachable before we block on topics.

    Without this, a node with no roscore just hangs silently, which reads to an
    operator as "the camera is broken" when in fact nothing was ever started.
    """
    import socket
    try:
        rospy.get_master().getSystemState()
    except Exception as exc:
        uri = os.environ.get("ROS_MASTER_URI", "(ROS_MASTER_URI unset)")
        raise Unavailable(
            "No ROS master reachable at %s (%s: %s)."
            % (uri, type(exc).__name__, exc),
            "Start one with `roscore`, or if the master runs on the robot, "
            "check ROS_MASTER_URI and ROS_IP are both set and that the two "
            "machines can ping each other. Hostname here is %s."
            % socket.gethostname())


def require_numpy():
    """numpy is needed for depth-image statistics; nothing else here needs it."""
    try:
        import numpy  # noqa: F401
        return numpy
    except ImportError:
        raise Unavailable(
            "numpy is not installed, so depth-pixel statistics cannot be "
            "computed. Every other check in this node works without it.",
            "sudo apt install python3-numpy   (or run with --skip-depth)")


# --------------------------------------------------------------------------- #
# Environment provenance
# --------------------------------------------------------------------------- #

def _run(cmd, timeout=6):
    """Run a local command, returning stdout or None. Never raises.

    Provenance probing must not be able to take a node down, and it must never
    touch the network — every command here is a local file read or a local
    package query.
    """
    try:
        out = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                             timeout=timeout, check=False)
        text = out.stdout.decode("utf-8", "replace").strip()
        return text or None
    except Exception:
        return None


def probe_l4t_version():
    """Read the L4T (Linux for Tegra) release from the Jetson's release file."""
    f = Path("/etc/nv_tegra_release")
    if not f.exists():
        return None
    try:
        txt = f.read_text()
    except Exception:
        return None
    # Format: "# R35 (release), REVISION: 6.1, GCID: ..., BOARD: ..."
    m = re.search(r"R(\d+).*?REVISION:\s*([0-9.]+)", txt)
    if m:
        return "R%s.%s" % (m.group(1), m.group(2).rstrip("."))
    return txt.splitlines()[0].strip() if txt.strip() else None


def probe_jetpack_version():
    """JetPack version, from the metapackage if present.

    L4T R35.6.x corresponds to JetPack 5.1.5. We report what dpkg says rather
    than hardcoding that mapping, and fall back to the mapping only as a hint.
    """
    v = _run(["dpkg-query", "-W", "-f=${Version}", "nvidia-jetpack"])
    if v:
        return v
    l4t = probe_l4t_version() or ""
    if l4t.startswith("R35.6"):
        return "5.1.5 (inferred from L4T %s, not confirmed by dpkg)" % l4t
    return None


def probe_zed_sdk_version():
    """ZED SDK version.

    The SDK writes a CMake version file on install; that is the cheapest and
    most reliable source. Note the project is pinned to the 4.x line: SDK 5.x
    requires JetPack 6, and this Jetson is on JetPack 5.1.5.
    """
    for cand in ("/usr/local/zed/zed-config-version.cmake",
                 "/usr/local/zed/lib/zed-config-version.cmake"):
        p = Path(cand)
        if p.exists():
            try:
                m = re.search(r'PACKAGE_VERSION\s+"?([0-9][0-9.]*)', p.read_text())
                if m:
                    return m.group(1)
            except Exception:
                pass
    # Fall back to the version stamped into the shared library filename.
    libdir = Path("/usr/local/zed/lib")
    if libdir.is_dir():
        try:
            for f in libdir.iterdir():
                m = re.match(r"libsl_zed\.so\.([0-9][0-9.]*)$", f.name)
                if m:
                    return m.group(1)
        except Exception:
            pass
    return None


def probe_zedlink_driver_version():
    """Version of the ZED Link Quad capture-card driver .deb.

    Stereolabs has shipped this under a couple of package names across
    releases, so we try each. This driver is what binds the two MAX96712
    deserializers to /dev/video*; a mismatch between it and the SDK is a
    classic cause of "camera detected but no frames".
    """
    for name in ("stereolabs-zedlink-quad", "zed-link-quad",
                 "stereolabs-zedlink-duo", "stereolabs-zedlink-mono"):
        v = _run(["dpkg-query", "-W", "-f=${Version}", name])
        if v and "no packages found" not in v.lower():
            return "%s %s" % (name, v)
    return None


def probe_zed_daemon_running():
    """Is zed_x_daemon alive?

    The GMSL2 cameras are not opened directly by the SDK; the daemon owns them
    and hands out access. If it is dead, every ZED tool reports "camera not
    detected", which looks identical to a cable fault. Distinguishing the two
    saves a lot of time with a screwdriver.
    """
    out = _run(["pgrep", "-a", "zed_x_daemon"])
    return bool(out)


def probe_video_devices():
    """List /dev/video* — the capture card's GMSL2 channels as the kernel sees them."""
    d = Path("/dev")
    if not d.is_dir():
        return []
    try:
        return sorted(str(p) for p in d.glob("video*"))
    except Exception:
        return []


def probe_environment(include_camera=True):
    """Collect everything that should be recorded alongside a dataset.

    Returns a plain dict, with `None` where something genuinely could not be
    determined. Recording an honest `null` is far better than recording a
    plausible guess — a guess in a provenance record is worse than a blank,
    because a blank prompts someone to go and check.
    """
    import platform
    import socket
    env = {
        "probed_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "hostname": socket.gethostname(),
        "kernel": platform.release(),
        "machine": platform.machine(),
        "python": platform.python_version(),
        "l4t_version": probe_l4t_version(),
        "jetpack_version": probe_jetpack_version(),
        "zed_sdk_version": probe_zed_sdk_version(),
        "zedlink_driver": probe_zedlink_driver_version(),
        "ros_distro": os.environ.get("ROS_DISTRO"),
        "ros_master_uri": os.environ.get("ROS_MASTER_URI"),
    }
    if include_camera:
        env["zed_x_daemon_running"] = probe_zed_daemon_running()
        env["video_devices"] = probe_video_devices()
    return env


def is_jetson():
    """True when running on Tegra hardware. Used only to tailor advice text."""
    return Path("/etc/nv_tegra_release").exists()


# --------------------------------------------------------------------------- #
# Streaming statistics
# --------------------------------------------------------------------------- #

class Stats(object):
    """Accumulates a stream of floats and reports mean, spread and tails.

    Uses Welford's online algorithm for the mean and variance rather than
    summing squares. With 15 minutes of 200 Hz IMU samples the naive sum of
    squares loses precision; Welford does not, and costs the same.

    Samples are also retained (capped) so real percentiles can be computed.
    Percentiles matter more than the mean here: a camera that averages 30 FPS
    but stalls for 300 ms once a minute is useless for SLAM, and the mean will
    never show that. The p99 will.
    """

    # ~24 MB of float64. A 15-minute soak at the ZED X's 200 Hz IMU rate is
    # 180 000 samples, so the cap is never reached and no percentile is ever
    # computed from a truncated sample. THE VALUE IS DELIBERATELY UNCHANGED.
    # ZEDX["imu_rate_hz"] was corrected from the ZED 2's 400 Hz to the ZED X's
    # datasheet 200 Hz; 600 000 was already generous against 360 000 and is
    # more so against 180 000. Lowering it would save memory this code has
    # never been short of, and would narrow a safety margin that exists for
    # the case where somebody points this at a faster sensor. Only the comment
    # was wrong.
    MAX_KEPT = 600000

    def __init__(self, name=""):
        self.name = name
        self.n = 0
        self._mean = 0.0
        self._m2 = 0.0
        self.min = float("inf")
        self.max = float("-inf")
        self.samples = []
        self._dropped_samples = 0

    def add(self, x):
        x = float(x)
        self.n += 1
        d = x - self._mean
        self._mean += d / self.n
        self._m2 += d * (x - self._mean)
        if x < self.min:
            self.min = x
        if x > self.max:
            self.max = x
        if len(self.samples) < self.MAX_KEPT:
            self.samples.append(x)
        else:
            self._dropped_samples += 1

    @property
    def mean(self):
        return self._mean if self.n else float("nan")

    @property
    def stdev(self):
        return math.sqrt(self._m2 / (self.n - 1)) if self.n > 1 else 0.0

    def pct(self, p):
        """Nearest-rank percentile. Returns NaN if nothing was recorded."""
        if not self.samples:
            return float("nan")
        s = sorted(self.samples)
        k = max(0, min(len(s) - 1, int(math.ceil(p / 100.0 * len(s))) - 1))
        return s[k]

    def count_above(self, threshold):
        return sum(1 for x in self.samples if x > threshold)

    def as_dict(self, unit=""):
        return {
            "n": self.n,
            "mean": _r(self.mean),
            "stdev": _r(self.stdev),
            "min": _r(self.min) if self.n else None,
            "max": _r(self.max) if self.n else None,
            "p50": _r(self.pct(50)),
            "p95": _r(self.pct(95)),
            "p99": _r(self.pct(99)),
            "unit": unit,
            "samples_truncated": self._dropped_samples,
        }


def _r(x, nd=6):
    """Round for JSON, turning inf/NaN into None so the file stays valid JSON."""
    try:
        if x is None or math.isnan(x) or math.isinf(x):
            return None
        return round(float(x), nd)
    except (TypeError, ValueError):
        return None


# --------------------------------------------------------------------------- #
# Stereo depth physics
#
# These helpers exist so the numbers quoted in the documentation and the
# thresholds used by the validator come from one place and cannot disagree.
# --------------------------------------------------------------------------- #

def focal_px_from_fov(width_px=None, fov_h_deg=None):
    """Nominal focal length in pixels from the horizontal field of view.

    f_px = (W/2) / tan(FOV_h / 2)

    This is a *sanity reference only*. The real value comes from camera_info's
    projection matrix P[0]. If the two disagree by more than a few percent,
    either the wrong resolution is configured or the calibration is suspect —
    which is exactly the kind of silent error this package exists to catch.
    """
    width_px = width_px or ZEDX["max_width"]
    fov_h_deg = fov_h_deg or ZEDX["fov_h_deg"]
    return (width_px / 2.0) / math.tan(math.radians(fov_h_deg) / 2.0)


def depth_from_disparity(disparity_px, focal_px=None, baseline_m=None):
    """The stereo depth equation: Z = f*B/d.

    Z  depth to the point            [m]
    f  focal length                  [pixels]
    B  stereo baseline, 120 mm here  [m]
    d  disparity, the horizontal shift of the same feature between the two
       images                        [pixels]

    Disparity is measured in pixels, so depth is a *reciprocal* function of a
    quantised measurement. That single fact drives everything about how this
    camera behaves at range.
    """
    focal_px = focal_px or focal_px_from_fov()
    baseline_m = baseline_m if baseline_m is not None else ZEDX["baseline_mm"] / 1000.0
    if disparity_px <= 0:
        return float("inf")
    return focal_px * baseline_m / disparity_px


def depth_uncertainty_m(z_m, focal_px=None, baseline_m=None, sigma_d_px=0.25):
    """Propagate disparity error into depth error.

        sigma_Z ~= Z^2 * sigma_d / (f * B)

    Differentiate Z = fB/d with respect to d and you get dZ/dd = -fB/d^2, and
    substituting d = fB/Z gives the Z-squared term. The practical consequence:
    doubling the distance quadruples the depth uncertainty.

    HOW THIS COMPARES WITH WHAT STEREOLABS PUBLISHES. The datasheet quotes two
    accuracy figures for our column, and only two: "< 0.4% to 2m" and "< 7% at
    20m" (ZEDX["depth_err_to_2m_pct"], ZEDX["depth_err_at_20m_pct"]). Those are
    the numbers to compare against — not the 0.2%-at-1 m / 3.1%-at-15 m pair
    this docstring used to quote, which appears nowhere in the document and
    which this project invented at some point. See the long note in the ZEDX
    dict.

    The two real anchors do NOT lie on one quadratic: 0.4% at 2 m implies a
    coefficient of 0.0020 m^-1, 7% at 20 m implies 0.0035 m^-1. That is not a
    contradiction — they are inequality bounds ("<"), and the far one has more
    headroom built in — but it does mean any single-coefficient z^2 model
    matches one anchor and not the other, and whoever quotes such a model must
    say which. obstacle_segmenter.py uses 0.0020, i.e. the near anchor, and
    says so.

    `sigma_d_px` defaults to a quarter pixel, a common figure for a good
    sub-pixel stereo matcher on well-textured scenes. Featureless surfaces —
    a wet road, a blank wall, fresh snow — are much worse, and no amount of
    calibration fixes that.
    """
    focal_px = focal_px or focal_px_from_fov()
    baseline_m = baseline_m if baseline_m is not None else ZEDX["baseline_mm"] / 1000.0
    return (z_m ** 2) * sigma_d_px / (focal_px * baseline_m)


# --------------------------------------------------------------------------- #
# Small formatting helpers shared by the report writers
# --------------------------------------------------------------------------- #

def fmt_hz(x):
    return "n/a" if x is None or (isinstance(x, float) and math.isnan(x)) else "%.2f Hz" % x


def fmt_ms(x):
    return "n/a" if x is None or (isinstance(x, float) and math.isnan(x)) else "%.3f ms" % (x * 1000.0)


def worst(*statuses):
    """Combine PASS/WARN/FAIL verdicts, worst wins."""
    order = {"PASS": 0, "WARN": 1, "FAIL": 2}
    out = "PASS"
    for s in statuses:
        if order.get(s, 0) > order[out]:
            out = s
    return out


def die(log, exc):
    """Uniform handling of an `Unavailable` at the top level of a node.

    Prints the reason and the fix, records both in the run log, and returns the
    exit code. Callers do `return die(log, exc)` so the shape of every main()
    is identical.
    """
    log.error("CANNOT RUN: %s" % exc.reason)
    if exc.fix:
        log.error("SUGGESTED FIX: %s" % exc.fix)
    log.summary("Could not run — %s" % exc.reason, status="BLOCKED")
    return EXIT_UNAVAILABLE


def html_escape(x):
    import html as _html
    return _html.escape("" if x is None else str(x))


def load_simple_yaml(path):
    """Read a flat-ish YAML file without requiring PyYAML.

    Only supports what this package's own config files use: nested maps with
    scalar leaves, plus lists of scalars. If PyYAML happens to be installed we
    use it, because it is strictly better; the hand parser is the fallback so
    that a freshly flashed Jetson works before pip has been touched.
    """
    text = Path(path).read_text()
    try:
        import yaml  # type: ignore
        return yaml.safe_load(text) or {}
    except ImportError:
        pass

    # Strip blanks and comments once, keeping (indent, content) pairs, then
    # parse recursively. A block's type (map or list) is decided by looking at
    # its first line, which is exactly how YAML itself resolves the ambiguity.
    rows = []
    for raw in text.splitlines():
        if not raw.strip() or raw.lstrip().startswith("#"):
            continue
        line = raw.split(" #")[0].rstrip()
        if not line.strip():
            continue
        rows.append((len(line) - len(line.lstrip()), line.strip()))

    # A list item like "- name: foo" opens a map whose remaining keys are
    # indented beneath it. We detect that with this pattern rather than a bare
    # `":" in text` test, so that scalar items which merely contain a colon
    # for maps.
    map_item = re.compile(r"^[A-Za-z_][\w.\-]*:(\s|$)")

    def parse(i, indent):
        """Parse the block starting at row `i` at column `indent`.

        Returns (value, next_row_index).
        """
        if rows[i][1].startswith("- "):
            items = []
            while i < len(rows) and rows[i][0] == indent and rows[i][1].startswith("- "):
                content = rows[i][1][2:]
                if map_item.match(content):
                    # Re-anchor the item's first line to the column its sibling
                    # keys occupy, then parse the whole thing as an ordinary
                    # map. This is what makes lists-of-maps work — the
                    # `recording.files` section of every dataset manifest.
                    item_col = indent + 2
                    rows[i] = (item_col, content)
                    entry, i = parse(i, item_col)
                    items.append(entry)
                else:
                    items.append(_scalar(content))
                    i += 1
            return items, i
        node = {}
        while i < len(rows) and rows[i][0] == indent:
            key, _, val = rows[i][1].partition(":")
            key, val = key.strip(), val.strip()
            i += 1
            if val == "" and i < len(rows) and rows[i][0] > indent:
                node[key], i = parse(i, rows[i][0])
            elif val == "":
                node[key] = None
            else:
                node[key] = _scalar(val)
        return node, i

    if not rows:
        return {}
    value, _ = parse(0, rows[0][0])
    return value if isinstance(value, dict) else {"_root": value}


def _scalar(v):
    v = v.strip()
    if len(v) >= 2 and v[0] == v[-1] and v[0] in "\"'":
        return v[1:-1]
    low = v.lower()
    if low in ("true", "yes"):
        return True
    if low in ("false", "no"):
        return False
    if low in ("null", "~", ""):
        return None
    try:
        return int(v)
    except ValueError:
        pass
    try:
        return float(v)
    except ValueError:
        pass
    return v


def dump_simple_yaml(obj, indent=0):
    """Serialise a dict/list/scalar tree to YAML without PyYAML.

    Manifests must be writable on a machine that may not have PyYAML, and they
    must remain human-editable — an operator will absolutely open one and fix a
    typo in the trajectory description by hand.
    """
    pad = "  " * indent
    out = []
    if isinstance(obj, dict):
        for k, v in obj.items():
            if isinstance(v, dict) and v:
                out.append("%s%s:" % (pad, k))
                out.append(dump_simple_yaml(v, indent + 1))
            elif isinstance(v, (list, tuple)) and v:
                out.append("%s%s:" % (pad, k))
                for item in v:
                    if isinstance(item, dict):
                        body = dump_simple_yaml(item, indent + 2).splitlines()
                        out.append("%s  - %s" % (pad, body[0].strip()))
                        out.extend(body[1:])
                    else:
                        out.append("%s  - %s" % (pad, _emit(item)))
            elif isinstance(v, (list, tuple)):
                out.append("%s%s: []" % (pad, k))
            elif isinstance(v, dict):
                out.append("%s%s: {}" % (pad, k))
            else:
                out.append("%s%s: %s" % (pad, k, _emit(v)))
    else:
        out.append("%s%s" % (pad, _emit(obj)))
    return "\n".join(out)


def _emit(v):
    if v is None:
        return "null"
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, (int, float)):
        return repr(v)
    s = str(v)
    if s == "" or any(c in s for c in ":#{}[]&*!|>'\"%@`") or s.strip() != s:
        return '"%s"' % s.replace("\\", "\\\\").replace('"', '\\"')
    return s


def load_css():
    """The project stylesheet, inlined into generated HTML.

    Generated pages must open from a USB stick on a machine with no internet —
    that is the actual situation in the lab — so nothing may be linked.
    """
    f = REPO / "docs" / "assets" / "base.css"
    try:
        return f.read_text() if f.exists() else ""
    except Exception:
        return ""


if __name__ == "__main__":
    # Running this module directly prints the environment it detects. This is
    # the quickest way to answer "does this machine even have the camera stack?"
    print("Repo root      : %s" % REPO)
    print("Is Jetson      : %s" % is_jetson())
    env = probe_environment()
    width = max(len(k) for k in env)
    for k, v in env.items():
        print("%-*s : %s" % (width, k, v))
    print()
    f = focal_px_from_fov()
    print("Nominal focal length at %dpx wide, %.0f deg HFOV: %.1f px"
          % (ZEDX["max_width"], ZEDX["fov_h_deg"], f))
    print("Depth uncertainty (sigma_d = 0.25 px):")
    for z in (1, 2, 5, 10, 15, 25, 35):
        s = depth_uncertainty_m(z)
        print("   Z = %5.1f m  ->  sigma_Z = %6.3f m  (%.2f%%)" % (z, s, 100.0 * s / z))
