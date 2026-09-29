#!/usr/bin/env python3
"""
obstacle_segmenter.py — turn the ZED X depth cloud into the two things the
navigation stack actually consumes: an obstacle cloud with the drivable ground
removed, and a curb/edge channel.

    rosrun sidewalk_perception obstacle_segmenter.py
    rosrun sidewalk_perception obstacle_segmenter.py --self-test
    rosrun sidewalk_perception obstacle_segmenter.py --geometry

WHY THIS EXISTS
    `sidewalk_navigation/config/costmap_common.yaml` declares two observation
    sources fed by this package:

        zed_obstacles -> /sidewalk_perception/obstacles_cloud  (PointCloud2)
        curb_scan     -> /sidewalk_perception/curb_scan        (LaserScan)

    and `nav_preflight.py` marks the first of those `critical: True`. Until this
    node existed, nothing in `sidewalk_perception` published either topic, so
    preflight hard-failed and the local costmap had no live obstacles at all.
    A robot in that state navigates purely on the recorded map — it drives into
    anything that was not there on the day the map was made.

    NOTE THE MESSAGE TYPES. The curb channel is a LaserScan, not a cloud. That
    is a deliberate choice made in costmap_common.yaml and this node honours it:
    forcing the detector to commit to ONE range per bearing is a much stronger
    and more debuggable statement than a fog of 3D points. A curb/edge
    PointCloud2 is published as well, but only for RViz and for debugging; the
    costmap does not read it.

===========================================================================
THE GEOMETRY — WHERE THE GROUND ACTUALLY IS, AND WHERE IT IS NOT
===========================================================================

Everything below is derived from `sidewalk_bringup/config/robot_frames.yaml`
(authoritative) and the ZED X datasheet constants in `perception_common.ZEDX`.

TWO HEIGHT CONVENTIONS EXIST AND THIS SECTION USES THE SECOND ONE.
`robot_frames.yaml` states the camera's z RELATIVE TO base_link, and base_link
sits 0.13228 m above the floor. Every formula here needs the height ABOVE THE
FLOOR, because every one of them asks where a ray from the lens meets the
GROUND. The conversion is done once, in `load_mount_geometry()`, which adds
`base.base_link_height_above_floor` to `camera_front.z`:

    0.563 (base_link-relative) + 0.13228 = 0.69528 m above the floor

Reading the base_link-relative z as a floor height was a real bug in this file
and it under-reported the near blind zone by 24 cm — see the comment above
MOUNT for the full account and for the assertion that now prevents it.

FIGURES CHECKED AUTOMATICALLY. Every value in the block below is recomputed
from the live mount by `--self-test` section 1 and compared against what is
written here. Do not hand-edit them: run `--geometry` and paste. If the mount
is re-measured and this block is not updated, the self-test FAILS — that is
deliberate, and it replaces an earlier check which was gated on the old
placeholder values and therefore fell silent the moment they changed.

    cam_height_m          = 0.69528  m above the FLOOR (not above base_link)
    cam_pitch_rad         = 0.0565   rad nose down (3.237 deg), MEASURED
    cam_x_offset_m        = 0.071    m, the LEFT LENS forward of base_link
    footprint_length_m    = 0.990    m
    bumper_x_m            = 0.495    m, half the footprint length
    ground_first_lens_m   = 1.268    m ahead of the lens
    ground_first_base_m   = 1.339    m ahead of base_link
    optical_depth_there_m = 1.305    m along the optical Z axis
    blind_band_m          = 0.844    m, bumper to first ground return

    vertical FOV is 51.0 deg (half-FOV 25.5 deg) and the minimum usable depth is
    1.0 m along the optical Z axis; both come from ZEDX and are not repeated
    as checked figures.

    THE FOV FIGURE CHANGED ON 2026-08-29 AND MOVED FOUR OF THE NUMBERS ABOVE.
    ZEDX["fov_v_deg"] said 52 deg. The ZED X datasheet says the vertical field
    of view is at most 45 deg, so 52 was not merely imprecise — it was above
    the manufacturer's stated MAXIMUM, and therefore wrong at every resolution
    the camera offers rather than only at some. It had been written from
    memory. Correcting it moved the first ground return OUT, from 1.242 to
    1.442 m ahead of the lens, and deepened the near blind zone from 0.818 to
    1.018 m. Every one of those 20 cm was pavement this file claimed to see and
    does not. That is the same direction of error, on the same safety number,
    as the 132 mm camera-height bug corrected earlier the same day.

WHERE DOES THE GROUND FIRST ENTER VIEW?

    The bottom edge of the image is depressed below horizontal by

        theta + halfFOV = 3.237 + 22.5 = 25.737 deg

    That ray meets the ground at a horizontal distance from the LENS of

        x = h / tan(25.737 deg) = 0.69528 / 0.4821 = 1.442 m

    We must also check the 1 m minimum depth, because a ray can leave the
    frame before the camera is willing to measure along it. The slant range to
    that ground point is h / sin(25.737 deg) = 1.601 m, and the ZED reports
    depth along the OPTICAL Z AXIS, not slant range, so the depth there is

        1.601 * cos(22.5 deg) = 1.479 m

    which is comfortably beyond the 1.0 m floor. So the field of view is the
    binding constraint, not the minimum depth — by 48 cm, where the old
    placeholder mount left only 8 cm. Note which way the FOV correction pushed
    that margin: a NARROWER field of view puts the bottom edge of the frame
    further out, so the first ground point is further away, so the optical
    depth there is larger and the minimum-depth floor is even less likely to
    take over. The margin got better and the safety number got worse, from the
    single change — they are not the same quantity and it is worth not
    confusing them. Increase the downward pitch far enough and the depth floor
    still binds; `--geometry` recomputes both and reports which one wins, so
    this stays honest if the mount is re-measured or a cropped video mode
    (which narrows the vertical FOV again) is ever configured.

    In base_link that first ground point is at

        0.071 + 1.442 = 1.513 m forward of the wheel axle.

THE NEAR BLIND ZONE — STATED HONESTLY

    The front bumper is at x = +0.495 m. The ground is first observed at
    x = +1.513 m. Between those two lies a band roughly

        1.513 - 0.495 = 1.018 m deep, full width of the robot,

    in which THIS NODE NEVER SEES THE GROUND. Not "sees it poorly" — never
    sees it. A curb, a pothole, a dropped bottle or a child's foot inside that
    band is invisible to the front camera.

    THIS NUMBER HAS BEEN UNDER-REPORTED TWICE, BY TWO INDEPENDENT BUGS, BOTH
    IN THE SAME DIRECTION. Under-reporting is the dangerous direction: it
    claims the robot is blind over a shorter stretch of ground than it really
    is, and the stopping-distance argument below is built on that stretch.

        0.582 m   while base_link-relative height was read as floor height
        0.818 m   height fixed, but vertical FOV still the remembered 52 deg
        1.018 m   both corrected — this is the current figure

    (0.89 m appears in still older text and came from the pre-measurement
    placeholder mount; it is not part of that sequence.) Two separate mistakes
    each shaved the same safety margin, and neither was caught by anything
    until the source document was read. That is the argument for reading the
    datasheet rather than recalling it.

    At the 0.6 m/s speed cap enforced by velocity_guard.py that band is about
    1.70 seconds of travel — was 1.36 s before the FOV correction, so a third
    of a second more of committed travel across ground nothing has looked at.
    The mitigation is not in this node; it is that the costmap RETAINS what was
    marked when the hazard was further away, and that the speed cap keeps
    stopping distance shorter than the band. The residual risk is a hazard that
    appears in the band without ever having been seen from further away —
    something that falls, rolls or steps into it. Nothing here detects that. A
    second, downward-looking sensor is the only real fix, and none is fitted.

    This is also why the node's region of interest starts at 1.0 m rather than
    at 0: asking for ground data closer than that returns nothing, and a
    silently empty region is worse than an explicitly excluded one. Note that
    the first ground return is now at 1.513 m, so roi_x_min = 1.00 m keeps a
    0.51 m margin of permanently empty region — it was 0.31 m under the 52 deg
    FOV, and 0.08 m when the height was being read 132 mm low. roi_x_min is a
    TUNING VALUE and is deliberately left alone here: raising it to ~1.45 m
    would stop the node asking for a half-metre of ground that cannot exist,
    which is a real and separate improvement, but changing it in the same edit
    as a geometry correction would confound the two. See the roi_x_min note in
    config/obstacle_segmenter.yaml, which now carries the recommendation.

===========================================================================
WHY A FITTED PLANE AND NOT A HEIGHT THRESHOLD
===========================================================================

The obvious implementation is "any point above 10 cm is an obstacle". It is
wrong on a real sidewalk, for three independent reasons that all push the same
way:

  1. CAMBER. Pavements are built with a 1-2 % cross-slope so water runs off.
     2 % over the +/-4 m half-width of the region of interest is 8 cm of
     genuine, drivable height change — most of a 10 cm budget spent before
     anything has happened.

  2. ROBOT PITCH. The robot pitches on its tyres and suspension under
     acceleration and braking. A mere 1.5 deg of pitch tilts the whole cloud;
     at 8 m ahead that is 8 * tan(1.5 deg) = 0.21 m. A fixed 10 cm cut would
     paint the far half of the sidewalk as a solid wall every time the robot
     sets off. This is not hypothetical — it is what a fixed cut does.

  3. GRADIENT. Driveway aprons and kerb ramps run to 8 % (4.6 deg). The
     sidewalk itself is not flat and was never intended to be.

A plane fitted to the data absorbs all three, because all three are, to first
order, exactly a plane. What remains after subtracting the fitted plane is
what genuinely sticks up out of the pavement — which is the question we
actually wanted to ask.

WHY THE FIT MUST BE SEEDED

    RANSAC finds the plane with the most support. It has no idea which plane
    is "the ground". Point a camera down a sidewalk with a building along one
    side and the wall will frequently return MORE points than the pavement:
    it is closer to perpendicular to the viewing rays, so it gets far more
    pixels per square metre, and it is usually better textured than plain
    concrete. Unseeded RANSAC will happily report the wall as the ground
    plane, at which point the pavement becomes a 10 m tall "obstacle" and the
    wall becomes "drivable". The robot would try to drive into the building.

    That failure is not rare or exotic. It is the default outcome on a narrow
    sidewalk. So the fit is CONSTRAINED by the prior we get for free from TF:

      - the cloud is transformed into base_link first, where the ground is
        z = 0 with normal +z by definition of the frame;
      - candidate planes whose normal is more than `max_plane_tilt_deg` from
        vertical are REJECTED outright (a wall is ~90 deg off and cannot pass);
      - candidate planes more than `max_plane_offset_m` from the origin in
        height are REJECTED (the robot is standing on the ground, so the
        ground passes through z ~ 0 underneath it);
      - the seed plane itself (z = 0, normal +z) is always evaluated as a
        candidate, so a degenerate frame degrades to the TF prior rather than
        to nonsense.

    The gate is deliberately loose enough (10 deg) to swallow camber, pitch
    and ramps several times over, and far too tight to admit a wall. That gap
    — 4.6 deg of real world versus 90 deg of wall — is what makes a simple
    angular gate a sufficient discriminator here.

WHY ORDINARY LEAST SQUARES IN z, NOT TOTAL LEAST SQUARES

    The refinement step fits z = a*x + b*y + c to the inliers rather than
    doing a proper orthogonal-distance (eigenvector) fit. That would be the
    wrong choice for an arbitrary plane, because OLS-in-z blows up as the
    plane approaches vertical. It is the right choice HERE precisely because
    the tilt gate has already guaranteed the plane is within 10 deg of
    horizontal, where cos(tilt) >= 0.985 and the difference between the two
    fits is under 1.5 %. In exchange we get a 3x3 linear solve that runs in
    pure Python with no eigen-decomposition, which is what lets the entire
    algorithm be exercised by --self-test on a machine with no numpy.

===========================================================================
CURBS: A DISCONTINUITY, NOT AN OBSTACLE
===========================================================================

A curb is not a thing standing on the ground. It is a STEP IN the ground —
the ground surface itself is discontinuous. That distinction drives the whole
detector:

  - A 120 mm curb is barely above the 0.10 m floor of the general obstacle
    channel, and a 120 mm DROP is entirely below it. Neither is reliably
    visible to a height-band filter. Hence a separate channel.

  - We detect it as a step in LOCAL height relative to the fitted plane
    between adjacent ground cells, not as an absolute height. A step survives
    camber, pitch and gradient for the same reason the plane fit does.

  - SIGN MATTERS, AND IT IS NOT SYMMETRIC.
        up-step   (+): a kerb the robot would have to climb. Untraversable,
                       but a collision with it is a bump.
        down-step (-): a drop to the roadway. FAR more dangerous, and worse,
                       it is INVISIBLE to every height-band filter — the road
                       surface is below the ground plane, so it fails the
                       `min_obstacle_height` test and reads as free space,
                       and a raytrace passes straight over it and CLEARS the
                       cells beyond. A drop-off is the one hazard the costmap
                       actively talks itself out of believing in.

    So down-steps get two extra protections here: they are reported at the
    NEAR lip of the drop rather than at the far side (see below), and, by
    default, they are additionally injected into the obstacle cloud as
    virtual points at robot height so that a costmap configured without the
    curb channel still refuses to drive off the edge.

  - THE EDGE IS REPORTED AT THE NEAR CELL, ALWAYS. When two adjacent ground
    cells differ by more than the step threshold, the actual discontinuity
    lies somewhere between them and we do not know where. Reporting the
    nearer of the two is the conservative choice for a drop: the barrier
    lands on the lip you must not cross, not one cell past it.

===========================================================================
THE DEPTH-NOISE FLOOR, AND WHY CURB DETECTION STOPS AT ~4 m
===========================================================================

ZED X depth error grows with the square of range. The coefficient used
throughout this project is

        sigma_range(z) ~= 0.002 * z^2   metres

WHERE THAT 0.002 REALLY COMES FROM — CORRECTED 2026-08-29.

    This paragraph used to say the coefficient was obtained "fitting the
    datasheet's two anchor points (0.2 % at 1 m, 3.1 % at 15 m)". NEITHER OF
    THOSE ANCHORS IS IN THE DATASHEET. There is no 1 m figure and no 15 m
    figure in the document. What Stereolabs publishes for the ZED X 4.6 mm is

        "Depth Accuracy   < 0.4% to 2m       -> 0.008 m, giving a = 0.00200
                          < 7%   at 20m"     -> 1.4 m,   giving a = 0.00350

    So 0.002 is right, but for a reason nobody had written down: it is the
    NEAR anchor. It reproduces the 2 m bound exactly and is optimistic by 1.75x
    at the far one. Both datasheet figures are inequalities, so a single
    quadratic cannot honour both, and the choice of which to honour is a real
    engineering decision that was previously disguised as a fit.

    WHAT ADOPTING THE FAR ANCHOR WOULD DO — RECOMMENDED, NOT APPLIED. With
    a = 0.0035 the edge-localisation limit below falls from 5.02 m to 3.81 m,
    which is BELOW the shipped curb_max_range_m of 3.90 m; the self-test's
    "configured curb range does not exceed the depth-noise limit" check would
    then fail, correctly, and curb_max_range_m would have to come down to about
    3.80 m. That is a tuning change and a behaviour change, so it is flagged
    here and NOT made as a side effect of a datasheet correction — one variable
    per experiment. The measurement that settles it is the one already named at
    the end of this section: park a known distance from a measured curb and
    look at the spread of the reported edge range.

    Keeping 0.002 is the OPTIMISTIC choice, and the numbers below inherit that.

There are two different ways that error hurts a curb detector, and the
intuitive one is NOT the binding constraint.

  (a) HEIGHT noise — the one everybody worries about. A range error displaces
      the point ALONG the viewing ray. Near the ground at horizontal distance
      z, that ray has depression angle alpha = atan(h/z), so only sin(alpha)
      of the error lands in the vertical direction:

          sigma_height(z) = 0.002 * z^2 * sin(atan(h/z))
                         ~= 0.002 * z^2 * (h/z)  for z >> h
                          = 0.002 * h * z = 0.00139 * z     metres

      LINEAR in range, not quadratic — the grazing geometry works in our
      favour. At 5 m that is 6.9 mm; at 10 m, 13.9 mm; at 15 m, 20.8 mm.
      Against a 100-150 mm curb, none of those is fatal on its own.
      (These grew by 12 % against the figures quoted before 2026-08-29,
      because the coefficient is 0.002 * h and h itself grew from a mistakenly
      base_link-relative 0.62 to the true floor-relative 0.69528. A HIGHER
      camera looks down more steeply, so more of each range error lands in the
      vertical direction. Still far inside the curb height.)

  (b) EDGE LOCALISATION — the one that actually binds. The same range error
      slides the point ALONG THE GROUND by cos(alpha) * sigma_range, which
      for a grazing ray is essentially the whole of it:

          sigma_along_ground(z) ~= 0.002 * z^2   metres

          z =  5 m  ->  0.05 m
          z =  8 m  ->  0.13 m
          z = 10 m  ->  0.20 m
          z = 15 m  ->  0.45 m

      The costmap resolution is 0.05 m. At 5 m the uncertainty in WHERE the
      curb is has grown to exactly one costmap cell. At 10 m it is four
      cells, and a "curb" smeared over 20 cm of ground is not a usable
      barrier — it is either a phantom wall across the sidewalk or a hole in
      the real one, depending on which way the noise went that frame.

  (c) A third, independent argument lands in the same place: ground sampling
      density. One pixel row spans (z^2 + h^2)/h * (FOV_v/rows) of ground,
      which is 2.4 cm at 5 m but 9.5 cm at 10 m and 21.2 cm at 15 m. Beyond
      ~11.3 m a single pixel row covers more ground than the entire width of a
      curb, so the step is not merely noisy, it is unsampled.
      (This one improved twice over. The span goes as 1/h, so the corrected —
      higher — camera samples the far ground more finely; and it goes as FOV_v,
      so the corrected — NARROWER, 45 not 52 deg — field of view spreads the
      same 1200 pixel rows over less sky and samples finer again. Both push the
      sensor-only limits out. Neither is what binds — see below.)

      AND THIS ONE IS NOT COMFORTABLE, BECAUSE OF `max_points`. The sentence
      above describes the SENSOR. It does not describe this node, which
      decimates the cloud by striding it (see `cloud_to_xyz_np`) down to
      `max_points`. At the default 40 000 that is a stride of 58 over a
      1920x1200 cloud, and the ground grid needs `min_cell_points` (3) points
      in a 0.10 m cell before that cell exists at all.

      Two models bracket the answer and they do not agree:

        pessimistic (uniform density): points per cell = cell_area /
          (stride * area_per_pixel). Reaches 3 at 4.46 m.

        optimistic (raster aliasing): striding by 58 with a row length of 1920
          keeps ~33 samples in EVERY row, so the row (depth) direction keeps
          its full resolution and only the column direction is thinned. Cells
          that contain a kept column then hold res/row_spacing points, which
          reaches 3 at 5.91 m, with about half the cells empty in vertical
          stripes.

      Which one holds depends on the exact aliasing between the stride and the
      image width, and neither has been measured. `--geometry` prints both.
      The failure, if the pessimistic model is nearer the truth, is SILENT: an
      empty curb scan, indistinguishable from "there was no curb there".
      THIS IS AN OPEN ITEM, not a settled one. What would confirm it: log
      `len(cells)` against range on a recorded SVO and find where cells start
      dropping out. If that range is below `curb_max_range_m`, either raise
      `max_points` (80 000 buys the pessimistic bound 5.65 m, at real CPU
      cost in the pure-Python gridder) or lower `curb_max_range_m` to match.

  CONCLUSION: the shipped `config/obstacle_segmenter.yaml` sets
  `curb_max_range_m` to 3.90 m, which is set by the PESSIMISTIC cell-population
  bound (4.46 m) and not by edge localisation (5.02 m). The two limits are
  independent and the tighter one wins; --geometry prints both and marks which.
  The 5.02 m edge-localisation figure is where the range would sit if
  decimation were not in the picture, and it is what costmap_common.yaml's own
  5.0 m hard stop was derived from.

  The cell-population bound moved out from 4.11 m to 4.46 m with the FOV
  correction, for the reason in (c): a narrower vertical field of view packs
  the same pixel rows into less ground, so cells stay populated further out.
  The 3.90 m configured value therefore sits further inside its binding limit
  than before, not closer to it. That is slack, not a problem, and it is NOT a
  licence to raise curb_max_range_m in this edit — see the recommendation note
  in config/obstacle_segmenter.yaml.

  !! THE BUILT-IN FALLBACK DISAGREES WITH THE SHIPPED CONFIG. `DEFAULTS`
  below still carries curb_max_range_m = 5.00, which the config file overrides
  to 3.90 on every normal run. They are only both live if the YAML fails to
  install. That divergence is FLAGGED, NOT FIXED HERE — it is a tuning value
  and changing it is a separate, deliberate decision, not a side effect of a
  geometry correction.

  It is exposed as a parameter because the honest answer depends on pavement
  texture, which sets the sub-pixel matching quality and which we have not
  measured. Beyond this range the node still computes steps, but does not emit
  them into the scan.

  WHAT WOULD CONFIRM IT: park the robot a known distance from a measured curb,
  record 200 frames, and look at the spread of the reported edge range. If the
  standard deviation at 5 m is materially worse than 0.05 m, this number is
  too optimistic and must come down. That measurement has NOT been made — no
  camera is wired to this repository.

===========================================================================
GRACEFUL DEGRADATION
===========================================================================
No camera is wired and no robot is online. Every path here either runs or
explains precisely what is absent and exits non-zero. --help, --self-test and
--geometry work on a machine with no ROS, no numpy and no camera; numpy is
imported lazily inside the runtime path only.

HONESTY
Nothing in this file has been run against real hardware. Every threshold is a
reasoned starting value derived from datasheet physics and the measured mount
geometry, and each one carries a note in config/obstacle_segmenter.yaml saying
what measurement would confirm or refute it.

EXIT CODES (shared with the rest of this package)
    0 PASS   1 FAIL   2 UNAVAILABLE   3 INTERRUPTED
"""
from __future__ import annotations

import argparse
import math
import os
import random
import re
import sys
from pathlib import Path

# perception_common installs the sys.path shim for the shared RunLogger and
# imports nothing from ROS at module scope. Importing it here is what makes the
# canonical logger import work identically from source and from an install
# space. If it is missing we are not in this package at all, which is a real
# error worth surfacing plainly rather than as a bare ImportError traceback.
try:
    from perception_common import (
        EXIT_FAIL, EXIT_INTERRUPTED, EXIT_PASS, EXIT_UNAVAILABLE, REPO, ZEDX,
        RunLogger, Unavailable, die, load_simple_yaml, require_master,
        require_rospy,
    )
except ImportError:  # pragma: no cover - only when run outside the package
    sys.stderr.write(
        "\nobstacle_segmenter.py: cannot import perception_common.py.\n"
        "  It must sit beside this file in sidewalk_perception/scripts/.\n"
        "  If you copied this script somewhere else, copy that one too.\n\n")
    raise

DEFAULT_CONFIG = (Path(__file__).resolve().parent.parent
                  / "config" / "obstacle_segmenter.yaml")

# Where the authoritative mount geometry lives. Read, never written.
ROBOT_FRAMES = (REPO / "catkin_ws" / "src" / "sidewalk_bringup"
                / "config" / "robot_frames.yaml")


# --------------------------------------------------------------------------- #
# Defaults.
#
# Every one of these is also in config/obstacle_segmenter.yaml with its full
# justification. They are duplicated here so the node runs correctly when the
# config file is missing — a node that silently does nothing because a YAML
# file did not get installed is a worse failure than a node with defaults.
# The config file is the documentation; this dict is the safety net.
# --------------------------------------------------------------------------- #
DEFAULTS = {
    # ---- inputs ----
    "cloud_topic": "/zedx_front/zed_node/point_cloud/cloud_registered",
    # Only "cloud" is implemented. The key exists so that a future depth-image
    # path has an obvious place to attach; setting anything else raises a
    # specific Unavailable rather than silently doing nothing. There are
    # deliberately NO depth_topic/camera_info_topic keys — config that names an
    # unimplemented feature reads as though the feature exists.
    "source": "cloud",

    # ---- outputs (WIRED TO costmap_common.yaml — do not rename lightly) ----
    "obstacles_topic": "/sidewalk_perception/obstacles_cloud",
    "curb_scan_topic": "/sidewalk_perception/curb_scan",
    "curb_cloud_topic": "/sidewalk_perception/curb_cloud",
    "ground_cloud_topic": "/sidewalk_perception/ground_cloud",
    "publish_debug_clouds": True,

    # ---- frames ----
    "target_frame": "base_link",
    "scan_frame_id": "base_link",
    "tf_timeout_s": 0.20,

    # ---- region of interest, metres in base_link ----
    "roi_x_min": 1.00,
    "roi_x_max": 8.00,
    "roi_y_abs_max": 4.00,
    "roi_z_min": -1.00,
    "roi_z_max": 2.00,

    # ---- decimation ----
    "max_points": 40000,

    # ---- ground plane fit ----
    "ground_band_m": 0.30,
    "ransac_iterations": 80,
    "ransac_inlier_tol_m": 0.040,
    "ransac_min_inlier_fraction": 0.25,
    "max_plane_tilt_deg": 10.0,
    "max_plane_offset_m": 0.15,
    "min_ground_points": 200,
    "plane_smoothing": 0.30,
    "random_seed": 0,

    # ---- obstacle extraction ----
    "obstacle_min_height": 0.10,
    "obstacle_max_height": 1.90,

    # ---- curb / step detection ----
    "grid_res_m": 0.10,
    "min_cell_points": 3,
    "curb_min_step_m": 0.060,
    "curb_max_step_m": 0.250,
    "curb_max_range_m": 5.00,
    "curb_min_range_m": 1.00,
    "mark_down_steps_in_obstacle_cloud": True,
    "down_step_virtual_heights": [0.15, 0.30, 0.45],

    # ---- synthetic scan ----
    "scan_angle_min": -1.5708,
    "scan_angle_max": 1.5708,
    "scan_angle_increment": 0.008727,   # 0.5 deg
    "scan_range_min": 0.20,
    "scan_range_max": 8.00,

    # ---- runtime ----
    "rate_hz": 10.0,
    "log_period_s": 10.0,
}

# ── THE TWO HEIGHT CONVENTIONS, AND WHY THEY MUST NEVER BE CONFUSED ────────
#
# There are two different "how high is the camera" numbers in this project and
# they differ by 132 mm. Every geometry function BELOW takes the second one.
#
#   RELATIVE TO base_link   what robot_frames.yaml's camera_front.z states, and
#                           says so explicitly in its own header. base_link is
#                           the robot's centre of rotation, which the URDF puts
#                           0.13228 m ABOVE the floor — NOT on the floor.
#   ABOVE THE FLOOR         what `cam_height_m` means here, and what every
#                           ground-intercept, blind-zone, depth-noise and
#                           sampling-density formula in this file needs, because
#                           they all ask "where does a ray from the lens meet
#                           the GROUND".
#
#       height above floor = camera_front.z + base.base_link_height_above_floor
#
# THIS HAS ALREADY BEEN THE BUG ONCE. camera_front.z was read straight into
# cam_height_m, so every safety figure this module derived was computed for a
# camera 132 mm lower than the real one. The direction of the error is the
# dangerous one: a lower camera sees the ground SOONER, so the near blind zone
# in front of the bumper was reported as 0.582 m when the true depth is
# 0.818 m — the module understated, by 24 cm, the band in which it is blind.
# load_mount_geometry() now performs the conversion at one clearly marked
# place, and self-test section 1 asserts the result is floor-referenced.
#
# (0.818, not 0.819. This comment said 0.819 for one revision, which is the
# exact failure mode it warns about in miniature: a safety figure re-typed by
# hand instead of taken from --geometry. The computed value is 0.81816 m and
# the docstring's checked-figures block, which the self-test now compares
# against the live mount unconditionally, is the only place it belongs.)
#
# `cam_x_offset_m` needs NO such conversion: x is measured from base_link and
# is used against the footprint, which is also measured from base_link. Only
# the vertical axis has two datums.

# Mount geometry, in THIS module's convention: cam_height_m is ABOVE THE FLOOR.
#
# These literals are the fallback used only when robot_frames.yaml cannot be
# read, so that --geometry and --self-test still work on a bare checkout. They
# are NOT independent numbers — they are that file's measured values of
#
#   cam_height_m    0.563 (camera_front.z) + 0.13228 (base_link above floor)
#                   = 0.69528, quoted to 5 dp as 0.69528
#   cam_pitch_rad   camera_front.pitch, measured 3.24 deg nose down
#   cam_x_offset_m  camera_front.x, the LEFT LENS forward of base_link
#
# THEY GO STALE THE MOMENT robot_frames.yaml IS RE-MEASURED. Nothing checks
# them against that file automatically — a check would have to read the file,
# and the whole point of these is to cover the case where it cannot be read.
# The mount source is printed at the top of --geometry for exactly this reason:
# if it does not name robot_frames.yaml, the numbers under it are these
# literals and their age is whatever this comment says.
MOUNT = {
    "cam_height_m": 0.69528,
    "cam_pitch_rad": 0.0565,
    "cam_x_offset_m": 0.071,
    "footprint_length_m": 0.99,
    "footprint_width_m": 0.67,
}

# Floor of the sanity assertion in self-test section 1 and in the conversion
# below. Any plausible mast height on this robot clears this comfortably; a
# base_link-relative z that has NOT been converted lands under it (0.563 today,
# and any future re-measure of a camera on a 0.99 x 0.67 m chassis stays far
# below 0.65 in that convention). It is a convention tripwire, not a spec.
MIN_PLAUSIBLE_CAM_HEIGHT_M = 0.65


def load_mount_geometry(path=ROBOT_FRAMES):
    """Read the authoritative mount geometry, or keep the literals above.

    robot_frames.yaml is owned by sidewalk_bringup and is the single source of
    truth for where the camera is. We read it rather than hardcoding, because
    the moment somebody measures the real robot and updates that file, every
    number derived here must move with it. Failure to read is not fatal — the
    literals above are that file's values transcribed — but it IS reported,
    because silently using stale geometry is precisely the class of error
    robot_frames.yaml's own header warns about.

    THE ONE CONVERSION THAT HAPPENS HERE is the height datum change described
    at length above: robot_frames.yaml states camera_front.z relative to
    base_link, and everything downstream of this function wants it relative to
    the FLOOR. The two differ by base.base_link_height_above_floor. Reading z
    without adding it is the bug this function exists to have fixed.
    """
    out = dict(MOUNT)
    out["source"] = "built-in literals (robot_frames.yaml not read)"
    try:
        if not Path(path).exists():
            return out
        cfg = load_simple_yaml(path) or {}
        cam = cfg.get("camera_front") or {}
        base = cfg.get("base") or {}

        # THE DATUM CHANGE. base_link_height_above_floor was added to
        # read the number instead of re-typing 0.13228 here — a re-typed
        # constant is a second source of truth, and this file already has one
        # too many.
        #
        # If the key is ABSENT we do NOT quietly assume zero. Assuming zero is
        # the old bug wearing a different hat: it produces a camera height that
        # is 132 mm too low, no error, and a blind zone under-reported by
        # 24 cm. We fall back to the whole MOUNT block instead, which is
        # already floor-referenced and correct, and we say so in `source` so
        # --geometry shows it on its first line.
        if "base_link_height_above_floor" not in base:
            out["source"] = (
                "built-in literals (%s has no base.base_link_height_above_floor,"
                " so camera_front.z cannot be converted from base_link-relative"
                " to floor-relative)" % path)
            return out
        base_link_h = float(base["base_link_height_above_floor"])

        if "z" in cam:
            # base_link-relative  ->  floor-relative. See the block comment
            # above MOUNT for why these are different quantities.
            out["cam_height_m"] = float(cam["z"]) + base_link_h
        if "pitch" in cam:
            out["cam_pitch_rad"] = float(cam["pitch"])
        if "x" in cam:
            # No datum change on x: robot_frames.yaml measures it from
            # base_link and so does the footprint it is compared against.
            out["cam_x_offset_m"] = float(cam["x"])
        if "footprint_length" in base:
            out["footprint_length_m"] = float(base["footprint_length"])
        if "footprint_width" in base:
            out["footprint_width_m"] = float(base["footprint_width"])

        # THE TRIPWIRE. If the conversion above were ever dropped, or the file
        # were edited to state z in the floor convention (so that adding the
        # datum double-counts it), the result stops being a plausible camera
        # height on this robot. Refuse it and fall back rather than publish
        # safety geometry derived from a number in the wrong convention — the
        # failure this whole comment block exists to prevent is a SILENT one.
        if out["cam_height_m"] < MIN_PLAUSIBLE_CAM_HEIGHT_M:
            out = dict(MOUNT)
            out["source"] = (
                "built-in literals (%s gave camera_front.z + "
                "base_link_height_above_floor = %.5f m, below the %.2f m floor "
                "for a floor-referenced camera height on this robot — the two "
                "height conventions have been mixed somewhere)"
                % (path, float(cam.get("z", 0.0)) + base_link_h,
                   MIN_PLAUSIBLE_CAM_HEIGHT_M))
            return out

        out["source"] = str(path)

        # TWO PROVENANCE FLAGS, WITH DIFFERENT SCOPES. robot_frames.yaml's
        # top-level `measured:` is scoped TO ITS `base:` BLOCK — that file's
        # own header says so explicitly, and names this module's --geometry
        # banner as one of the two consumers it wrote the scope down for. It
        # caveats the CHASSIS FOOTPRINT and nothing else.
        #
        # The camera pose has its own nested `camera_front.measured`, true
        # and printing "every number below is a placeholder" was correct while
        # both were placeholders and became a FALSE claim about a measured
        # mount the day the camera was measured. Keep them separate.
        out["measured"] = bool(cfg.get("measured", False))
        out["camera_measured"] = bool(cam.get("measured", False))
    except Exception as exc:          # never let a config read kill the node
        out["source"] = "built-in literals (%s reading %s)" % (
            exc.__class__.__name__, path)
    return out


# --------------------------------------------------------------------------- #
# GEOMETRY — pure functions, no dependencies. These are the numbers quoted in
# the module docstring; computing them rather than hardcoding them means the
# docstring cannot silently drift away from the mount.
# --------------------------------------------------------------------------- #

def optical_depth_of_ground_point(x_m, h_m, pitch_rad):
    """Depth (along the optical Z axis) of the ground point x_m ahead of the lens.

    The ZED reports depth along the optical axis, not slant range, and the
    difference is 10 % at the bottom of the frame. Getting this wrong makes the
    minimum-depth analysis wrong in the direction that matters.
    """
    if x_m <= 0.0:
        return 0.0
    slant = math.hypot(x_m, h_m)
    alpha = math.atan2(h_m, x_m)       # depression of the ray to that point
    return slant * math.cos(alpha - pitch_rad)


def ground_first_visible(h_m=None, pitch_rad=None, fov_v_deg=None,
                         depth_min_m=None):
    """Horizontal distance FROM THE LENS at which the ground first appears.

    Two constraints compete and either may bind:
      FOV   — the bottom edge of the image meets the ground at h/tan(pitch+hfov)
      DEPTH — the camera refuses to report closer than depth_min along Z

    Returns (x_m, which_binds). `which_binds` is "fov", "depth" or "never".
    Solved by bisection rather than algebra because the depth constraint is
    transcendental in x and bisection is three lines, exact to 1e-6 m here, and
    impossible to get subtly wrong.
    """
    h_m = MOUNT["cam_height_m"] if h_m is None else h_m
    pitch_rad = MOUNT["cam_pitch_rad"] if pitch_rad is None else pitch_rad
    fov_v_deg = ZEDX["fov_v_deg"] if fov_v_deg is None else fov_v_deg
    depth_min_m = ZEDX["depth_min_m"] if depth_min_m is None else depth_min_m

    half = math.radians(fov_v_deg) / 2.0
    lower_depression = pitch_rad + half
    if lower_depression <= 1e-6:
        # Camera pitched up, or so wide that the bottom ray never descends.
        return float("inf"), "never"

    x_fov = h_m / math.tan(lower_depression)
    if optical_depth_of_ground_point(x_fov, h_m, pitch_rad) >= depth_min_m:
        return x_fov, "fov"

    # Minimum depth binds. Depth increases monotonically with x for x > 0 in
    # this configuration, so bisect for the x where depth == depth_min.
    lo, hi = x_fov, max(x_fov * 2.0, x_fov + 1.0)
    for _ in range(200):
        if optical_depth_of_ground_point(hi, h_m, pitch_rad) >= depth_min_m:
            break
        hi *= 2.0
        if hi > 1e4:
            return float("inf"), "never"
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        if optical_depth_of_ground_point(mid, h_m, pitch_rad) >= depth_min_m:
            hi = mid
        else:
            lo = mid
    return hi, "depth"


def range_noise_m(z_m):
    """sigma of the reported range at horizontal distance z. 0.002 * z^2.

    THE COEFFICIENT IS THE DATASHEET'S NEAR ANCHOR, NOT A FIT TO BOTH.
    Stereolabs publishes two accuracy bounds for the ZED X 4.6 mm, "< 0.4% to
    2m" and "< 7% at 20m", which imply 0.00200 and 0.00350 respectively. They
    are inequalities and no single quadratic satisfies both as equalities, so a
    choice is unavoidable; 0.002 honours the near one and is optimistic by
    1.75x at 20 m. Adopting 0.0035 instead would pull `curb_range_limit_m` from
    5.02 m down to 3.81 m, below the shipped `curb_max_range_m` of 3.90 m.
    That is flagged in the module docstring and deliberately not applied here.

    (Before 2026-08-29 the docstring claimed this was fitted to "0.2% at 1 m
    and 3.1% at 15 m". Those two figures are not in the datasheet at all.)
    """
    return 0.002 * z_m * z_m


def height_noise_m(z_m, h_m=None):
    """Vertical component of the range noise for a near-ground point.

    Only sin(alpha) of a range error lands in the vertical direction, where
    alpha is the ray's depression angle. Grazing geometry is our friend here.
    """
    h_m = MOUNT["cam_height_m"] if h_m is None else h_m
    if z_m <= 0.0:
        return 0.0
    alpha = math.atan2(h_m, z_m)
    return range_noise_m(z_m) * math.sin(alpha)


def along_ground_noise_m(z_m, h_m=None):
    """Horizontal component of the range noise — the edge-localisation error.

    This, not height noise, is what limits usable curb range. See the module
    docstring.
    """
    h_m = MOUNT["cam_height_m"] if h_m is None else h_m
    if z_m <= 0.0:
        return 0.0
    alpha = math.atan2(h_m, z_m)
    return range_noise_m(z_m) * math.cos(alpha)


def ground_sampling_m(z_m, h_m=None, fov_v_deg=None, rows=None):
    """Ground distance spanned by ONE pixel row at horizontal distance z.

    d(z)/d(alpha) = -(z^2 + h^2)/h, so one pixel row of angular size
    (FOV_v / rows) covers (z^2 + h^2)/h * that angle of ground.
    """
    h_m = MOUNT["cam_height_m"] if h_m is None else h_m
    fov_v_deg = ZEDX["fov_v_deg"] if fov_v_deg is None else fov_v_deg
    rows = ZEDX["max_height"] if rows is None else rows
    per_px = math.radians(fov_v_deg) / float(rows)
    return (z_m * z_m + h_m * h_m) / h_m * per_px


def grid_hole_range_m(grid_res_m=0.10, h_m=None):
    """Range at which ONE pixel row spans a whole ground-grid cell.

    A THIRD, independent limit on curb detection, and the one that fails most
    quietly. The ground grid needs several points per cell for its median to
    mean anything (`min_cell_points`). Ground sampling coarsens as z^2, so
    beyond some range a cell contains one pixel row or none, cells start
    dropping out, the grid develops holes, 4-connectivity between adjacent
    cells breaks, and step detection simply stops finding anything.

    It produces no error and no warning — just an empty curb scan — which is
    exactly the failure mode that gets mistaken for "there was no curb there".
    Worth knowing where it sits relative to the depth-noise limit that is
    supposed to bind first.
    """
    h_m = MOUNT["cam_height_m"] if h_m is None else h_m
    lo, hi = 0.5, 60.0
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        if ground_sampling_m(mid, h_m) < grid_res_m:
            lo = mid
        else:
            hi = mid
    return lo


def decimation_stride(max_points, width=None, height=None):
    """The stride `cloud_to_xyz_np` will apply to a full-frame organised cloud.

    Kept deliberately identical in form to the runtime expression so the two
    cannot drift: the runtime computes it from the message's actual point
    count, this computes it from the sensor's full frame, and for a full-frame
    organised cloud those are the same number.
    """
    width = ZEDX["max_width"] if width is None else width
    height = ZEDX["max_height"] if height is None else height
    n = float(width) * float(height)
    if n <= max_points:
        return 1
    return int(math.ceil(n / float(max_points)))


def points_per_cell(z_m, max_points, grid_res_m=0.10, h_m=None,
                    width=None, height=None, fov_v_deg=None, fov_h_deg=None):
    """(pessimistic, optimistic) point counts in one ground cell at range z.

    See the module docstring. Both models describe the SAME total point budget;
    they differ only in whether the surviving points are spread uniformly or
    concentrated into full-resolution rows by raster aliasing. The truth
    depends on gcd(stride, image width) and has not been measured, so the node
    reports both rather than picking one and sounding certain.
    """
    h_m = MOUNT["cam_height_m"] if h_m is None else h_m
    width = ZEDX["max_width"] if width is None else width
    height = ZEDX["max_height"] if height is None else height
    fov_v_deg = ZEDX["fov_v_deg"] if fov_v_deg is None else fov_v_deg
    fov_h_deg = ZEDX["fov_h_deg"] if fov_h_deg is None else fov_h_deg
    if z_m <= 0.0:
        return 0.0, 0.0
    d_alpha_v = math.radians(fov_v_deg) / float(height)
    d_alpha_h = math.radians(fov_h_deg) / float(width)
    row_span = (z_m * z_m + h_m * h_m) / h_m * d_alpha_v    # metres, along x
    col_span = math.hypot(z_m, h_m) * d_alpha_h             # metres, along y
    stride = decimation_stride(max_points, width, height)
    rows_per_cell = grid_res_m / row_span
    pessimistic = (grid_res_m * grid_res_m) / (stride * row_span * col_span)
    optimistic = rows_per_cell            # a cell that owns one kept column
    return pessimistic, optimistic


def cell_population_limits_m(max_points, min_cell_points=3, grid_res_m=0.10,
                             h_m=None):
    """(pessimistic, optimistic) ranges beyond which ground cells go empty.

    Beyond these the ground grid loses cells, 4-connectivity between adjacent
    cells breaks, and step detection stops finding anything — with no error and
    no warning, just an empty curb scan. Compare against `curb_max_range_m`.
    """
    def solve(idx):
        lo, hi = 0.3, 60.0
        for _ in range(200):
            mid = 0.5 * (lo + hi)
            if points_per_cell(mid, max_points, grid_res_m,
                               h_m)[idx] >= min_cell_points:
                lo = mid
            else:
                hi = mid
        return lo
    return solve(0), solve(1)


def curb_range_limit_m(cell_m=0.05, h_m=None):
    """Range at which edge-localisation noise reaches one costmap cell.

    Solve 0.002 * z^2 * cos(atan(h/z)) = cell.  For z >> h the cosine is ~1 and
    the answer is sqrt(cell/0.002); the exact solve is done by bisection so the
    near-field cosine is not fudged.
    """
    h_m = MOUNT["cam_height_m"] if h_m is None else h_m
    lo, hi = 0.1, 60.0
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        if along_ground_noise_m(mid, h_m) < cell_m:
            lo = mid
        else:
            hi = mid
    return lo


# --------------------------------------------------------------------------- #
# THE DOCSTRING-VERSUS-REALITY BRIDGE
#
# The module docstring quotes a block of concrete figures (camera height, first
# ground return, blind band, and so on). Those figures are the reason anybody
# reads this file: they are what gets quoted into reports, into
# config/obstacle_segmenter.yaml's tuning rationale, and into the safety
# argument for the near blind zone. A docstring figure that no longer matches
# the mount is therefore not a cosmetic problem — it is a wrong safety number
# with a plausible provenance.
#
# These two functions make the comparison mechanical: `documented_figures()`
# reads what the docstring CLAIMS, `computed_figures()` derives what the live
# mount ACTUALLY gives, and self-test section 1 asserts they agree. Neither
# function knows or cares which mount is loaded, so the check cannot be gated
# on a particular set of numbers the way its predecessor was.
# --------------------------------------------------------------------------- #

# The figures the docstring is required to state, spelled out here rather than
# discovered by scanning. That difference matters: if the block is EDITED DOWN
# — a line deleted, a name renamed — a scan would simply compare fewer things
# and still report success, which is the same "silently checks less" failure
# that the value-gated predecessor had. Naming them means a missing line is a
# FAILURE, not a shorter test.
DOCUMENTED_FIGURE_NAMES = (
    "cam_height_m",
    "cam_pitch_rad",
    "cam_x_offset_m",
    "footprint_length_m",
    "bumper_x_m",
    "ground_first_lens_m",
    "ground_first_base_m",
    "optical_depth_there_m",
    "blind_band_m",
)


def documented_figures(doc=None):
    """Parse the checked-figures block out of the module docstring.

    Returns {name: (value_as_written, decimal_places)}. The decimal count is
    carried because it defines the tolerance: a figure written to 3 dp is a
    claim accurate to +/-0.0005, and demanding more than that of a number a
    human is expected to read and re-type would make the check fire on
    rounding rather than on drift.

    The pattern is anchored to exactly four spaces of indent followed by one of
    the names in DOCUMENTED_FIGURE_NAMES, so the working arithmetic elsewhere
    in the docstring (which is indented eight spaces, and which uses `=` freely)
    cannot be mistaken for a checked figure.
    """
    doc = __doc__ if doc is None else doc
    out = {}
    for name in DOCUMENTED_FIGURE_NAMES:
        m = re.search(r"^ {4}%s +=\s+(-?\d+\.\d+)\b" % re.escape(name),
                      doc or "", re.M)
        if m:
            written = m.group(1)
            out[name] = (float(written), len(written.split(".")[1]))
    return out


def computed_figures(mount=None):
    """Recompute, from the live mount, every figure the docstring states.

    Deliberately duplicates a few lines of `geometry_report` rather than
    scraping its formatted output: the report rounds for display, and a check
    that parsed its own printout would be comparing two roundings of the same
    number and could never detect a small drift.
    """
    mount = mount or MOUNT
    h = mount["cam_height_m"]
    pitch = mount["cam_pitch_rad"]
    xoff = mount["cam_x_offset_m"]
    flen = mount["footprint_length_m"]
    bumper = flen / 2.0
    x_first, _binds = ground_first_visible(h, pitch)
    return {
        "cam_height_m": h,
        "cam_pitch_rad": pitch,
        "cam_x_offset_m": xoff,
        "footprint_length_m": flen,
        "bumper_x_m": bumper,
        "ground_first_lens_m": x_first,
        "ground_first_base_m": x_first + xoff,
        # Depth AT THE FIRST GROUND POINT, which is what the docstring's prose
        # derives. When the FOV binds this is the depth at the bottom of the
        # frame; when the minimum-depth floor binds instead it equals that
        # floor by construction. Either way it is the depth at the point the
        # two figures above describe.
        "optical_depth_there_m": optical_depth_of_ground_point(x_first, h,
                                                               pitch),
        "blind_band_m": (x_first + xoff) - bumper,
    }


def geometry_report(mount=None, cfg=None):
    """Human-readable derivation of every geometric number this node relies on.

    Printed by --geometry. This exists so that the numbers in the docstring can
    be CHECKED rather than believed, and so that re-measuring the mount
    immediately shows its consequences.
    """
    mount = mount or MOUNT
    cfg = cfg or DEFAULTS
    h = mount["cam_height_m"]
    pitch = mount["cam_pitch_rad"]
    xoff = mount["cam_x_offset_m"]
    flen = mount["footprint_length_m"]

    half = math.radians(ZEDX["fov_v_deg"]) / 2.0
    lower = pitch + half
    upper = pitch - half
    x_first, binds = ground_first_visible(h, pitch)
    x_first_base = x_first + xoff
    bumper = flen / 2.0
    blind = x_first_base - bumper

    L = []
    a = L.append
    a("MOUNT GEOMETRY (source: %s)" % mount.get("source", "built-in"))
    # PROVENANCE, WITH THE RIGHT SCOPE ON EACH FLAG.
    #
    # robot_frames.yaml's top-level `measured:` is scoped to its `base:` block.
    # That file's header says so in as many words, and it names THIS BANNER as
    # one of the two places the flag is consumed, precisely so that nobody
    # widens it again. It caveats the chassis footprint. It does not speak for
    # the camera, which carries its own `camera_front.measured`.
    #
    # This banner previously read "these are PLACEHOLDER dimensions, not this
    # robot. Every number below inherits that." That was accurate while the
    # was measured and the sentence became actively misleading: it told the
    # reader that a tape-and-transform-tree measurement, cross-checked against
    # gravity, was a guess. A caveat that cries wolf over measured data gets
    # scrolled past, and then it is not there for the number that IS a guess.
    if mount.get("measured") is False:
        a("  !! robot_frames.yaml says measured: false. THAT FLAG IS SCOPED TO")
        a("     ITS base: BLOCK — footprint length and width below are the")
        a("     Clearpath A200 manufacturer spec, NOT a tape measurement of")
        a("     this unit with its sensor payload fitted.")
    if mount.get("camera_measured") is True:
        a("     The camera height/pitch/x-offset are NOT covered by that flag:")
        a("     camera_front.measured is true (measured 2026-08-29).")
    elif "camera_measured" in mount:
        a("  !! camera_front.measured is false — the camera height, pitch and")
        a("     offset below are PLACEHOLDERS. Every geometric number derived")
        a("     from them, including the blind zone, inherits that.")
    a("  camera height              %8.3f m  above the FLOOR" % h)
    a("  camera pitch (down)        %8.3f rad  (%.3f deg)"
      % (pitch, math.degrees(pitch)))
    a("  camera x offset            %8.3f m  forward of base_link" % xoff)
    a("  footprint length           %8.3f m  -> front bumper at x = %+.3f m"
      % (flen, bumper))
    a("")
    a("FIELD OF VIEW")
    a("  vertical FOV               %8.1f deg (half = %.1f deg)"
      % (ZEDX["fov_v_deg"], math.degrees(half)))
    a("  top edge                   %8.2f deg %s horizontal"
      % (abs(math.degrees(upper)), "below" if upper > 0 else "above"))
    a("  bottom edge                %8.2f deg below horizontal"
      % math.degrees(lower))
    a("")
    a("WHERE THE GROUND FIRST APPEARS")
    a("  binding constraint         %8s" % binds.upper())
    a("    FOV limit                %8.3f m ahead of the lens"
      % (h / math.tan(lower) if lower > 0 else float('inf')))
    a("    optical depth there      %8.3f m  (camera floor is %.2f m)"
      % (optical_depth_of_ground_point(h / math.tan(lower) if lower > 0 else 1.0,
                                       h, pitch), ZEDX["depth_min_m"]))
    a("  first ground point         %8.3f m ahead of the LENS" % x_first)
    a("                             %8.3f m ahead of base_link" % x_first_base)
    a("")
    a("NEAR BLIND ZONE  (nothing here is ever observed)")
    a("  from front bumper          %8.3f m" % bumper)
    a("  to first ground return     %8.3f m" % x_first_base)
    a("  depth of blind band        %8.3f m, full robot width" % blind)
    a("  at the 0.6 m/s speed cap that is %.2f s of travel" % (blind / 0.6))
    a("  MITIGATION: costmap memory + speed cap. NOT detection. A hazard that")
    a("  enters this band without first being seen further out is invisible.")
    a("")
    # NOTE these three are appended verbatim, NOT through %-formatting, so the
    # percent signs are single. Writing them doubled (as a format string would
    # need) printed a literal "0.4%%" in the report.
    a("DEPTH NOISE  (sigma_range = 0.002 * z^2 — the datasheet's NEAR anchor,")
    a("              '< 0.4% to 2m'. Its far anchor, '< 7% at 20m', would give")
    a("              0.0035 and a 3.81 m edge limit. See range_noise_m.)")
    a("   z      sigma_range   sigma_height   sigma_along_ground   ground/px")
    for z in (1.0, 2.0, 3.0, 5.0, 8.0, 10.0, 15.0):
        a("  %5.1f m   %7.3f m      %7.4f m         %7.3f m       %7.3f m"
          % (z, range_noise_m(z), height_noise_m(z, h),
             along_ground_noise_m(z, h), ground_sampling_m(z, h)))
    a("")
    a("CURB DETECTION RANGE — independent limits, the TIGHTEST wins")
    lim = curb_range_limit_m(0.05, h)
    hole = grid_hole_range_m(cfg["grid_res_m"], h)
    pess, opt = cell_population_limits_m(cfg["max_points"],
                                         cfg["min_cell_points"],
                                         cfg["grid_res_m"], h)
    stride = decimation_stride(cfg["max_points"])
    limits = [
        ("edge noise reaches one 0.05 m costmap cell", lim),
        ("cells fall below min_cell_points, uniform model", pess),
        ("cells fall below min_cell_points, aliased model", opt),
        ("one sensor pixel row spans a whole ground cell", hole),
        ("one sensor pixel row spans a whole 0.12 m curb",
         grid_hole_range_m(0.12, h)),
    ]
    tightest = min(v for _, v in limits)
    for i, (name, v) in enumerate(limits, 1):
        a("  %d. %-46s %6.2f m%s"
          % (i, name, v, "   <-- BINDS" if v == tightest else ""))
    a("     (2) and (3) depend on max_points = %d -> stride %d over a %dx%d"
      % (cfg["max_points"], stride, ZEDX["max_width"], ZEDX["max_height"]))
    a("     cloud. They BRACKET the truth; which is nearer has NOT been")
    a("     measured. Limits 4 and 5 describe the raw sensor and ignore that")
    a("     decimation entirely, so they are NOT usable bounds for this node.")
    a("  configured curb_max_range_m                     %.2f m%s"
      % (cfg["curb_max_range_m"],
         "" if cfg["curb_max_range_m"] <= tightest
         else "   <-- EXCEEDS THE TIGHTEST LIMIT ABOVE"))
    a("  height noise at that range                      %.4f m"
      % height_noise_m(cfg["curb_max_range_m"], h))
    a("  as a fraction of a 0.12 m curb                  %.1f %%"
      % (100.0 * height_noise_m(cfg["curb_max_range_m"], h) / 0.12))
    a("  NOT VERIFIED ON HARDWARE. Confirm by parking a known distance from a")
    a("  measured curb and checking the spread of the reported edge range.")
    return "\n".join(L)


# --------------------------------------------------------------------------- #
# PURE-PYTHON CORE
#
# Everything from here to the ROS section operates on plain lists of (x, y, z)
# tuples and imports nothing. This is the REFERENCE implementation: it is what
# --self-test exercises, and it is what defines correct behaviour. The numpy
# fast paths further down must agree with it, and the self-test checks that
# they do whenever numpy happens to be installed.
#
# It is not fast enough for 40 000 points at 10 Hz — see `require_numpy_runtime`,
# which refuses to start the live node without numpy rather than running the
# reference code at 0.5 Hz and letting the costmap go stale.
# --------------------------------------------------------------------------- #

class Plane(object):
    """A plane n.x + d = 0 with |n| = 1 and n pointing UP (nz >= 0).

    The upward normal convention is not cosmetic: it is what makes
    `height_of(p)` a SIGNED height above the ground, so that "positive means
    sticking up" holds everywhere without a sign check at each use site.
    """

    __slots__ = ("nx", "ny", "nz", "d", "inliers", "n_points", "rms")

    def __init__(self, nx, ny, nz, d, inliers=0, n_points=0, rms=0.0):
        self.nx, self.ny, self.nz, self.d = nx, ny, nz, d
        self.inliers = inliers
        self.n_points = n_points
        self.rms = rms

    def height_of(self, p):
        """Signed distance of point p above the plane, in metres."""
        return self.nx * p[0] + self.ny * p[1] + self.nz * p[2] + self.d

    @property
    def tilt_deg(self):
        """Angle between the plane normal and vertical."""
        return math.degrees(math.acos(max(-1.0, min(1.0, self.nz))))

    @property
    def offset_at_origin(self):
        """Height of base_link's origin above the plane. Should be ~0."""
        return self.d

    def as_tuple(self):
        return (self.nx, self.ny, self.nz, self.d)

    def __repr__(self):
        return ("Plane(n=(%.4f, %.4f, %.4f), d=%.4f, tilt=%.2f deg, "
                "inliers=%d/%d)" % (self.nx, self.ny, self.nz, self.d,
                                    self.tilt_deg, self.inliers, self.n_points))


SEED_PLANE = Plane(0.0, 0.0, 1.0, 0.0)


def plane_from_three(p0, p1, p2):
    """Plane through three points, normal forced upward. None if degenerate."""
    ux, uy, uz = p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]
    vx, vy, vz = p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]
    nx = uy * vz - uz * vy
    ny = uz * vx - ux * vz
    nz = ux * vy - uy * vx
    norm = math.sqrt(nx * nx + ny * ny + nz * nz)
    if norm < 1e-9:
        return None                      # collinear or coincident
    nx, ny, nz = nx / norm, ny / norm, nz / norm
    if nz < 0.0:
        nx, ny, nz = -nx, -ny, -nz
    d = -(nx * p0[0] + ny * p0[1] + nz * p0[2])
    return Plane(nx, ny, nz, d)


def solve3(A, b):
    """Solve a 3x3 linear system by Gaussian elimination with partial pivoting.

    Written out rather than pulled from numpy because the whole point of this
    section is that it runs without numpy. Partial pivoting is not optional:
    without it, a ground patch centred near x = 0 makes the first pivot tiny
    and the solution garbage.
    """
    M = [list(A[0]) + [b[0]], list(A[1]) + [b[1]], list(A[2]) + [b[2]]]
    for col in range(3):
        piv = max(range(col, 3), key=lambda r: abs(M[r][col]))
        if abs(M[piv][col]) < 1e-12:
            return None                  # singular: degenerate point set
        if piv != col:
            M[col], M[piv] = M[piv], M[col]
        inv = 1.0 / M[col][col]
        for r in range(col + 1, 3):
            f = M[r][col] * inv
            if f:
                for c in range(col, 4):
                    M[r][c] -= f * M[col][c]
    x = [0.0, 0.0, 0.0]
    for r in (2, 1, 0):
        s = M[r][3] - sum(M[r][c] * x[c] for c in range(r + 1, 3))
        x[r] = s / M[r][r]
    return x


def fit_plane_ls(points):
    """Least-squares plane through `points`, fitting z = a*x + b*y + c.

    Valid only for near-horizontal planes; see the module docstring for why
    that restriction is free here and what it buys. Returns None if the points
    are degenerate (all in a line in the xy projection, or fewer than three).
    """
    n = len(points)
    if n < 3:
        return None
    sx = sy = sz = sxx = sxy = syy = sxz = syz = 0.0
    for x, y, z in points:
        sx += x
        sy += y
        sz += z
        sxx += x * x
        sxy += x * y
        syy += y * y
        sxz += x * z
        syz += y * z
    A = ((sxx, sxy, sx),
         (sxy, syy, sy),
         (sx, sy, float(n)))
    sol = solve3(A, (sxz, syz, sz))
    if sol is None:
        return None
    a, b, c = sol
    # z = ax + by + c  ->  -a x - b y + z - c = 0, then normalise.
    norm = math.sqrt(a * a + b * b + 1.0)
    return Plane(-a / norm, -b / norm, 1.0 / norm, -c / norm)


def plane_passes_gates(plane, max_tilt_deg, max_offset_m):
    """The seed constraint. This is what stops a wall being called 'ground'."""
    if plane is None:
        return False
    if not (plane.nz > 0.0):
        return False
    if plane.tilt_deg > max_tilt_deg:
        return False
    if abs(plane.offset_at_origin) > max_offset_m:
        return False
    return True


def score_plane(plane, points, tol):
    """Inlier count and RMS residual over the inliers."""
    n = 0
    ss = 0.0
    for p in points:
        h = plane.height_of(p)
        if -tol <= h <= tol:
            n += 1
            ss += h * h
    plane.inliers = n
    plane.n_points = len(points)
    plane.rms = math.sqrt(ss / n) if n else float("inf")
    return n


def fit_ground_plane(points, tol=0.04, iterations=80, max_tilt_deg=10.0,
                     max_offset_m=0.15, min_inlier_fraction=0.25,
                     seed=SEED_PLANE, rng=None, enforce_gates=True):
    """Seeded, gated RANSAC ground-plane fit. THE reference implementation.

    `enforce_gates=False` exists ONLY so the self-test can demonstrate what
    unseeded RANSAC does to a wall-dominated scene. Never turn it off at
    runtime; the whole safety argument rests on it.

    Returns (Plane, info_dict). The plane is never None: if no candidate wins,
    the seed plane derived from TF is returned and `info['fell_back']` is True.
    Refusing to return a plane would mean refusing to publish, and a costmap
    that stops receiving messages goes stale silently, which is worse than a
    costmap fed from the TF prior.
    """
    info = {"candidates": 0, "rejected_tilt": 0, "rejected_offset": 0,
            "rejected_degenerate": 0, "fell_back": False,
            "inlier_fraction": 0.0}
    n = len(points)
    if n < 3:
        info["fell_back"] = True
        s = Plane(*seed.as_tuple())
        score_plane(s, points, tol)
        return s, info

    rng = rng or random.Random(0)

    # Candidate zero is always the TF-derived seed. If every random sample is
    # rejected we still have a plane that is right by construction whenever the
    # robot is standing on level-ish ground.
    best = Plane(*seed.as_tuple())
    score_plane(best, points, tol)
    best_ok = plane_passes_gates(best, max_tilt_deg, max_offset_m)
    if not best_ok:
        best.inliers = -1                 # force any gated candidate to win

    for _ in range(iterations):
        i0 = rng.randrange(n)
        i1 = rng.randrange(n)
        i2 = rng.randrange(n)
        if i0 == i1 or i1 == i2 or i0 == i2:
            continue
        cand = plane_from_three(points[i0], points[i1], points[i2])
        if cand is None:
            info["rejected_degenerate"] += 1
            continue
        info["candidates"] += 1
        if enforce_gates:
            if cand.tilt_deg > max_tilt_deg:
                info["rejected_tilt"] += 1
                continue
            if abs(cand.offset_at_origin) > max_offset_m:
                info["rejected_offset"] += 1
                continue
        score_plane(cand, points, tol)
        # Inliers first, RMS as the tie-break. Two planes with identical
        # support are not equally good: the one hugging its inliers more
        # tightly is the one that will still be right next frame.
        if (cand.inliers > best.inliers
                or (cand.inliers == best.inliers and cand.rms < best.rms)):
            best = cand

    if best.inliers <= 0:
        info["fell_back"] = True
        best = Plane(*seed.as_tuple())
        score_plane(best, points, tol)
        return best, info

    # Refine on the consensus set. RANSAC picks the right SUBSET; least squares
    # over that subset picks the right PLANE. Doing only the first leaves the
    # answer resting on three randomly chosen, individually noisy points.
    inliers = [p for p in points if -tol <= best.height_of(p) <= tol]
    refined = fit_plane_ls(inliers)
    if refined is not None:
        keep = (plane_passes_gates(refined, max_tilt_deg, max_offset_m)
                if enforce_gates else True)
        if keep:
            score_plane(refined, points, tol)
            if refined.inliers >= best.inliers * 0.9:
                best = refined

    frac = best.inliers / float(n) if n else 0.0
    info["inlier_fraction"] = frac
    if frac < min_inlier_fraction:
        # Not enough of the scene agrees on any one plane. Usually means the
        # camera is looking at something that is not a sidewalk — a flight of
        # steps, a crowd at close range, or nothing at all. Fall back to the TF
        # prior and let the caller warn.
        info["fell_back"] = True
        fallback = Plane(*seed.as_tuple())
        score_plane(fallback, points, tol)
        return fallback, info
    return best, info


# ---- ground gridding and step detection ---------------------------------- #

class Cell(object):
    """One ground grid cell: its centre, its height above the plane, its count."""

    __slots__ = ("i", "j", "x", "y", "h", "count")

    def __init__(self, i, j, x, y, h, count):
        self.i, self.j, self.x, self.y, self.h, self.count = i, j, x, y, h, count

    @property
    def rng(self):
        """Planar range from base_link's origin."""
        return math.hypot(self.x, self.y)


def _median(vals):
    s = sorted(vals)
    n = len(s)
    if n == 0:
        return 0.0
    m = n // 2
    return s[m] if n % 2 else 0.5 * (s[m - 1] + s[m])


def build_ground_grid(points, plane, res=0.10, ground_band=0.30,
                      min_cell_points=3):
    """Bin near-ground points into a 2D grid keyed on (i, j), taking the MEDIAN
    height per cell.

    Median, not mean: a single stereo outlier at the wrong depth is a common
    occurrence on low-texture pavement, and one such point in a cell moves the
    mean by centimetres — the same order as the curb we are hunting for. The
    median ignores it entirely. This is the cheapest robustness available and
    it is the difference between a curb detector and a noise detector.

    `min_cell_points` then discards cells too sparse for the median to mean
    anything. On real data the far edge of the region of interest is exactly
    where cells go sparse, which is also where the noise is worst, so this
    threshold does double duty.
    """
    buckets = {}
    for p in points:
        h = plane.height_of(p)
        if -ground_band <= h <= ground_band:
            key = (int(math.floor(p[0] / res)), int(math.floor(p[1] / res)))
            buckets.setdefault(key, []).append(h)
    cells = {}
    for (i, j), hs in buckets.items():
        if len(hs) < min_cell_points:
            continue
        cells[(i, j)] = Cell(i, j, (i + 0.5) * res, (j + 0.5) * res,
                             _median(hs), len(hs))
    return cells


# 4-connected neighbours only. Diagonals are deliberately excluded: a diagonal
# pair is 1.41 cells apart, so the same physical step measured diagonally looks
# like a shallower gradient, and a curb running at 45 degrees to the grid would
# be reported with a systematically different step size than one running along
# it. Keeping the comparison isotropic in SPACING matters more than catching
# every edge orientation, and a curb is a continuous line that will always
# present some 4-connected pairs along its length.
_NEIGHBOURS = ((1, 0), (-1, 0), (0, 1), (0, -1))


class Step(object):
    """A detected height discontinuity between two adjacent ground cells."""

    __slots__ = ("x", "y", "rng", "delta", "kind", "near_h", "far_h")

    def __init__(self, x, y, rng, delta, kind, near_h, far_h):
        self.x, self.y, self.rng = x, y, rng
        self.delta = delta
        self.kind = kind          # "up" | "down"
        self.near_h, self.far_h = near_h, far_h

    def __repr__(self):
        return ("Step(%s %.3f m at x=%.2f y=%.2f r=%.2f)"
                % (self.kind, self.delta, self.x, self.y, self.rng))


def detect_steps(cells, min_step=0.06, max_step=0.25,
                 min_range=1.0, max_range=5.0):
    """Find height discontinuities between adjacent ground cells.

    For each adjacent pair we identify which cell is NEARER the robot and take
    delta = h(far) - h(near):

        delta >= +min_step   the ground rises going away from us  -> UP-STEP
        delta <= -min_step   the ground falls going away from us  -> DOWN-STEP

    The reported position is ALWAYS THE NEAR CELL. The true discontinuity lies
    somewhere in the ~10 cm between the two cell centres and we cannot say
    where. Placing the barrier on the near lip is the conservative choice, and
    for a down-step it is the difference between a costmap that stops the robot
    at the edge of the roadway and one that stops it 10 cm after the edge.

    `max_step` rejects anything taller than a curb. A 0.6 m wall base produces
    an enormous "step" at its foot, but a wall is not a curb — it is already
    handled, correctly and with its full height, by the obstacle channel.
    Letting it through here would put a duplicate, weaker mark in the curb scan
    and would mask the genuine curb that may run along its foot.

    `max_range` is the depth-noise limit derived in the module docstring.
    """
    steps = []
    for (i, j), c in cells.items():
        for di, dj in _NEIGHBOURS:
            other = cells.get((i + di, j + dj))
            if other is None:
                continue
            # Consider each unordered pair once, from the near side.
            if c.rng > other.rng:
                continue
            if c.rng == other.rng and (i, j) > (other.i, other.j):
                continue
            near, far = c, other
            delta = far.h - near.h
            mag = abs(delta)
            if mag < min_step or mag > max_step:
                continue
            if not (min_range <= near.rng <= max_range):
                continue
            steps.append(Step(near.x, near.y, near.rng, delta,
                              "up" if delta > 0 else "down",
                              near.h, far.h))
    return steps


def steps_to_scan(steps, angle_min=-1.5708, angle_max=1.5708,
                  increment=0.008727, range_min=0.2, range_max=8.0):
    """Collapse detected steps into one range per bearing — a synthetic LaserScan.

    Where several steps share a bearing we keep the NEAREST, and a down-step
    always beats an up-step at equal range. Both rules follow from the same
    asymmetry: over-reporting a hazard costs a detour, under-reporting one
    costs the robot.

    Bearings with no detection are left as +inf. costmap_common.yaml sets
    `inf_is_valid: false` for this source, so an infinite range is correctly
    read as "nothing seen here" rather than as "clear to the horizon" — which
    is the right semantics for a detector that is silent on most bearings.
    """
    nbins = int(math.floor((angle_max - angle_min) / increment)) + 1
    ranges = [float("inf")] * nbins
    kinds = [None] * nbins
    for s in steps:
        if not (range_min <= s.rng <= range_max):
            continue
        bearing = math.atan2(s.y, s.x)
        if bearing < angle_min or bearing > angle_max:
            continue
        b = int(round((bearing - angle_min) / increment))
        if b < 0 or b >= nbins:
            continue
        if s.rng < ranges[b]:
            ranges[b] = s.rng
            kinds[b] = s.kind
        elif s.rng == ranges[b] and s.kind == "down":
            kinds[b] = "down"
    return ranges, kinds


def extract_obstacles(points, plane, min_h=0.10, max_h=1.90):
    """Points whose height above the FITTED plane falls in the obstacle band.

    The floor is 0.10 m and it is not arbitrary: below that we are inside the
    plane-fit residual plus the depth noise, and cannot tell a bump from the
    pavement. Curbs live below this line, which is exactly why they have their
    own channel. The ceiling is 1.90 m — well above the robot, low enough to
    ignore tree canopy and building facades that would otherwise fill the
    costmap with permanent obstacles the robot can drive right under.
    """
    out = []
    for p in points:
        h = plane.height_of(p)
        if min_h <= h <= max_h:
            out.append(p)
    return out


def down_step_virtual_points(steps, plane, heights=(0.15, 0.30, 0.45),
                             band_min=0.10, band_max=1.90):
    """Synthesise obstacle points on the near lip of every down-step.

    WHY THIS IS NECESSARY, AND WHY IT IS SLIGHTLY UNCOMFORTABLE

    A drop to the roadway defeats the obstacle channel twice over. The road
    surface sits BELOW the ground plane, so it fails `min_obstacle_height` and
    contributes nothing; and the rays that reach it pass over the lip, so
    raytracing actively CLEARS the cells at the edge. Left alone, the costmap's
    considered opinion of a 150 mm drop into traffic is "free space".

    Injecting virtual points at robot height on the near lip turns that drop
    into something the ordinary obstacle machinery understands, so a costmap
    that is misconfigured, or that has the curb channel disabled, still refuses
    to drive over it.

    The discomfort is real and worth stating: these points are NOT measured.
    They are an assertion by this node, in a cloud that is otherwise pure
    sensor data, and they live on a topic whose costmap source has
    `clearing: true` — so a frame in which the drop is not detected will
    raytrace through and erase them. This is a belt-and-braces measure that
    strengthens a good configuration; it does not rescue a bad one. The
    authoritative channel remains curb_scan, which is never cleared.

    Set `mark_down_steps_in_obstacle_cloud: false` to disable.

    THE CLAMP, AND WHY IT IS NOT OPTIONAL

    The heights are measured from the FITTED PLANE, but costmap_2d filters the
    obstacle cloud on ABSOLUTE z in the costmap's global frame — it knows
    nothing about our plane. Those two references differ by however far the
    fitted plane sits from z = 0, which at a drop-off is precisely where the
    fit is most ambiguous: with roadway and pavement both in view the plane
    settles somewhere between them, several centimetres below the lip.

    That is enough to push the lowest virtual point under
    `min_obstacle_height` and have the costmap silently discard it. The first
    version of this function did exactly that — the self-test caught virtual
    points emitted at z = 0.07 m against a 0.10 m floor, i.e. a drop-off
    barrier that would have been thrown away by the very layer it exists to
    reach. Clamping into the band the costmap actually applies is what makes
    the guarantee real rather than nominal.
    """
    out = []
    lo = band_min + 0.02          # small margin so rounding cannot drop them
    hi = band_max - 0.02
    if hi <= lo:
        return out
    for s in steps:
        if s.kind != "down":
            continue
        # Height of the plane at (x, y): solve n.p + d = 0 for z.
        if abs(plane.nz) < 1e-6:
            continue
        z_plane = -(plane.nx * s.x + plane.ny * s.y + plane.d) / plane.nz
        for dh in heights:
            z = z_plane + dh
            out.append((s.x, s.y, min(hi, max(lo, z))))
    return out


def segment(points, cfg, plane_prior=SEED_PLANE, rng=None):
    """Run the whole pipeline on a list of (x, y, z) tuples in base_link.

    This is the single entry point shared by the live node and by --self-test,
    which is the only way to be confident that what the self-test proves is
    what the robot actually runs.
    """
    roi = [p for p in points
           if cfg["roi_x_min"] <= p[0] <= cfg["roi_x_max"]
           and abs(p[1]) <= cfg["roi_y_abs_max"]
           and cfg["roi_z_min"] <= p[2] <= cfg["roi_z_max"]]

    result = {
        "n_input": len(points),
        "n_roi": len(roi),
        "plane": None,
        "plane_info": {},
        "obstacles": [],
        "curb_points": [],
        "steps": [],
        "ranges": [],
        "kinds": [],
        "ok": False,
        "reason": "",
    }

    if len(roi) < cfg["min_ground_points"]:
        result["reason"] = ("only %d points in the region of interest "
                            "(need %d)" % (len(roi), cfg["min_ground_points"]))
        result["plane"] = Plane(*plane_prior.as_tuple())
        nbins = int(math.floor((cfg["scan_angle_max"] - cfg["scan_angle_min"])
                               / cfg["scan_angle_increment"])) + 1
        result["ranges"] = [float("inf")] * nbins
        result["kinds"] = [None] * nbins
        return result

    # Candidate ground points: everything within a generous band of the prior.
    # Restricting the RANSAC input this way is a second, independent guard on
    # top of the plane gates — a wall's points mostly are not in this band at
    # all, so they cannot vote.
    band = cfg["ground_band_m"] + cfg["max_plane_offset_m"]
    candidates = [p for p in roi if abs(plane_prior.height_of(p)) <= band]
    if len(candidates) < 3:
        candidates = roi

    plane, info = fit_ground_plane(
        candidates,
        tol=cfg["ransac_inlier_tol_m"],
        iterations=cfg["ransac_iterations"],
        max_tilt_deg=cfg["max_plane_tilt_deg"],
        max_offset_m=cfg["max_plane_offset_m"],
        min_inlier_fraction=cfg["ransac_min_inlier_fraction"],
        seed=plane_prior,
        rng=rng,
    )

    obstacles = extract_obstacles(roi, plane,
                                  cfg["obstacle_min_height"],
                                  cfg["obstacle_max_height"])
    cells = build_ground_grid(roi, plane,
                              res=cfg["grid_res_m"],
                              ground_band=cfg["ground_band_m"],
                              min_cell_points=cfg["min_cell_points"])
    steps = detect_steps(cells,
                         min_step=cfg["curb_min_step_m"],
                         max_step=cfg["curb_max_step_m"],
                         min_range=cfg["curb_min_range_m"],
                         max_range=cfg["curb_max_range_m"])
    ranges, kinds = steps_to_scan(steps,
                                  angle_min=cfg["scan_angle_min"],
                                  angle_max=cfg["scan_angle_max"],
                                  increment=cfg["scan_angle_increment"],
                                  range_min=cfg["scan_range_min"],
                                  range_max=cfg["scan_range_max"])

    curb_points = []
    for s in steps:
        if abs(plane.nz) > 1e-6:
            z = -(plane.nx * s.x + plane.ny * s.y + plane.d) / plane.nz
        else:
            z = 0.0
        curb_points.append((s.x, s.y, z + max(0.0, s.delta)))

    if cfg["mark_down_steps_in_obstacle_cloud"]:
        obstacles = obstacles + down_step_virtual_points(
            steps, plane, cfg["down_step_virtual_heights"],
            cfg["obstacle_min_height"], cfg["obstacle_max_height"])

    result.update({
        "plane": plane,
        "plane_info": info,
        "cells": cells,
        "obstacles": obstacles,
        "curb_points": curb_points,
        "steps": steps,
        "ranges": ranges,
        "kinds": kinds,
        "ok": True,
    })
    return result


# --------------------------------------------------------------------------- #
# NUMPY FAST PATHS
#
# Imported lazily, INSIDE functions, never at module scope. That is the whole
# reason --help, --geometry and --self-test work on a machine with no numpy.
# sidewalk_evaluation gates numpy at import time and its --help is broken as a
# result; this file must not repeat that.
# --------------------------------------------------------------------------- #

def require_numpy_runtime():
    """numpy is MANDATORY for the live node, and optional for everything else.

    The pure-Python core is the reference implementation and is correct, but it
    is roughly two orders of magnitude too slow for 40 000 points at 10 Hz. A
    node that runs the reference code live would publish at well under 1 Hz,
    which trips `expected_update_rate: 0.5` in costmap_common.yaml and leaves
    the local costmap running on stale obstacles — a silent, dangerous
    degradation. Refusing to start is the honest behaviour.
    """
    try:
        import numpy  # noqa: F401
        return numpy
    except ImportError:
        raise Unavailable(
            "numpy is not installed. The live segmenter needs it: the "
            "pure-Python reference implementation is ~100x too slow to keep "
            "the local costmap fresh, and publishing late is more dangerous "
            "than not publishing at all. --self-test and --geometry do NOT "
            "need numpy and still work.",
            "sudo apt install python3-numpy   (on the Jetson; this is already "
            "present in the ROS Noetic base install)")


_PC2_DTYPES = {1: "i1", 2: "u1", 3: "i2", 4: "u2",
               5: "i4", 6: "u4", 7: "f4", 8: "f8"}


def cloud_to_xyz_np(msg, np, max_points=40000):
    """Decode a PointCloud2 into an (N, 3) float64 array of finite points.

    Uses a structured-dtype view over the raw buffer rather than
    sensor_msgs.point_cloud2.read_points, which is a Python generator and takes
    seconds on a 2.3 MP cloud. The dtype is built from the message's own field
    table, so this works whether the cloud is xyz, xyzrgb, or xyz with padding.

    Decimation happens BEFORE any float maths, by striding the array. Striding
    a ZED cloud is not a uniform spatial sample — the cloud is organised
    row-major over the image, so a stride skips whole scanlines in a regular
    pattern. That is acceptable here because the ground is a large smooth
    surface and we are fitting a plane to it, not looking for small isolated
    features; and it is far cheaper than a true voxel filter. If small
    obstacles start being missed at range, this is the first thing to revisit.
    """
    names, formats, offsets = [], [], []
    for f in msg.fields:
        if f.datatype not in _PC2_DTYPES or f.count != 1:
            continue
        if f.name in names:
            continue
        names.append(f.name)
        formats.append(_PC2_DTYPES[f.datatype])
        offsets.append(f.offset)
    if not all(k in names for k in ("x", "y", "z")):
        raise Unavailable(
            "the point cloud on this topic has no x/y/z fields (it has: %s). "
            "That is not a depth cloud." % (", ".join(names) or "none"),
            "Check `rostopic echo -n1 --noarr <topic>` and point --cloud-topic "
            "at the registered point cloud, normally "
            "/zedx_front/zed_node/point_cloud/cloud_registered")

    dt = np.dtype({"names": names, "formats": formats,
                   "offsets": offsets, "itemsize": msg.point_step})
    if msg.is_bigendian:
        dt = dt.newbyteorder(">")

    arr = np.frombuffer(msg.data, dtype=dt)
    n = arr.shape[0]
    if n == 0:
        return np.zeros((0, 3), dtype=np.float64)
    stride = 1 if n <= max_points else int(math.ceil(n / float(max_points)))
    arr = arr[::stride]

    xyz = np.empty((arr.shape[0], 3), dtype=np.float64)
    xyz[:, 0] = arr["x"]
    xyz[:, 1] = arr["y"]
    xyz[:, 2] = arr["z"]
    # NaN and inf are the ZED's way of saying "no depth here" and they are the
    # majority of pixels on sky, on glass and beyond 35 m. Dropping them here
    # is not an optimisation; leaving them in poisons every mean and every
    # comparison downstream.
    return xyz[np.isfinite(xyz).all(axis=1)]


def transform_matrix_np(tf_msg, np):
    """4x4 homogeneous transform from a geometry_msgs/TransformStamped."""
    t = tf_msg.transform.translation
    q = tf_msg.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        raise Unavailable(
            "TF returned a zero-length rotation quaternion, which cannot be "
            "inverted. Something is publishing a malformed transform.",
            "Find it with `rosrun tf tf_monitor` and check the static "
            "transform publishers started by sidewalk_bringup.")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    M = np.identity(4)
    M[0, 0] = 1 - 2 * (y * y + z * z)
    M[0, 1] = 2 * (x * y - z * w)
    M[0, 2] = 2 * (x * z + y * w)
    M[1, 0] = 2 * (x * y + z * w)
    M[1, 1] = 1 - 2 * (x * x + z * z)
    M[1, 2] = 2 * (y * z - x * w)
    M[2, 0] = 2 * (x * z - y * w)
    M[2, 1] = 2 * (y * z + x * w)
    M[2, 2] = 1 - 2 * (x * x + y * y)
    M[0, 3], M[1, 3], M[2, 3] = t.x, t.y, t.z
    return M


def apply_transform_np(xyz, M, np):
    return xyz.dot(M[:3, :3].T) + M[:3, 3]


def fit_ground_plane_np(xyz, np, tol=0.04, iterations=80, max_tilt_deg=10.0,
                        max_offset_m=0.15, min_inlier_fraction=0.25,
                        seed=SEED_PLANE, rng=None):
    """Vectorised twin of `fit_ground_plane`. Same contract, same gates.

    Kept deliberately close in structure to the reference so the two can be
    read side by side; the self-test asserts they agree on shared inputs
    whenever numpy is available.
    """
    info = {"candidates": 0, "rejected_tilt": 0, "rejected_offset": 0,
            "rejected_degenerate": 0, "fell_back": False,
            "inlier_fraction": 0.0}
    n = int(xyz.shape[0])
    if n < 3:
        info["fell_back"] = True
        p = Plane(*seed.as_tuple())
        p.n_points = n
        return p, info

    rng = rng or random.Random(0)

    def score(nx, ny, nz, d):
        h = xyz[:, 0] * nx + xyz[:, 1] * ny + xyz[:, 2] * nz + d
        m = np.abs(h) <= tol
        cnt = int(m.sum())
        rms = float(np.sqrt((h[m] ** 2).mean())) if cnt else float("inf")
        return cnt, rms, m

    best = Plane(*seed.as_tuple())
    best.inliers, best.rms, _ = score(best.nx, best.ny, best.nz, best.d)
    best.n_points = n
    if not plane_passes_gates(best, max_tilt_deg, max_offset_m):
        best.inliers = -1

    for _ in range(iterations):
        i0, i1, i2 = rng.randrange(n), rng.randrange(n), rng.randrange(n)
        if i0 == i1 or i1 == i2 or i0 == i2:
            continue
        cand = plane_from_three(xyz[i0], xyz[i1], xyz[i2])
        if cand is None:
            info["rejected_degenerate"] += 1
            continue
        info["candidates"] += 1
        if cand.tilt_deg > max_tilt_deg:
            info["rejected_tilt"] += 1
            continue
        if abs(cand.offset_at_origin) > max_offset_m:
            info["rejected_offset"] += 1
            continue
        cnt, rms, _ = score(cand.nx, cand.ny, cand.nz, cand.d)
        cand.inliers, cand.rms, cand.n_points = cnt, rms, n
        if cnt > best.inliers or (cnt == best.inliers and rms < best.rms):
            best = cand

    if best.inliers <= 0:
        info["fell_back"] = True
        best = Plane(*seed.as_tuple())
        best.inliers, best.rms, _ = score(best.nx, best.ny, best.nz, best.d)
        best.n_points = n
        return best, info

    # Least-squares refinement on the consensus set, z = ax + by + c, solved
    # with the same 3x3 normal equations as the reference implementation.
    _, _, mask = score(best.nx, best.ny, best.nz, best.d)
    pts = xyz[mask]
    if pts.shape[0] >= 3:
        X, Y, Z = pts[:, 0], pts[:, 1], pts[:, 2]
        A = ((float((X * X).sum()), float((X * Y).sum()), float(X.sum())),
             (float((X * Y).sum()), float((Y * Y).sum()), float(Y.sum())),
             (float(X.sum()), float(Y.sum()), float(pts.shape[0])))
        sol = solve3(A, (float((X * Z).sum()), float((Y * Z).sum()),
                         float(Z.sum())))
        if sol is not None:
            a, b, c = sol
            nrm = math.sqrt(a * a + b * b + 1.0)
            ref = Plane(-a / nrm, -b / nrm, 1.0 / nrm, -c / nrm)
            if plane_passes_gates(ref, max_tilt_deg, max_offset_m):
                cnt, rms, _ = score(ref.nx, ref.ny, ref.nz, ref.d)
                if cnt >= best.inliers * 0.9:
                    ref.inliers, ref.rms, ref.n_points = cnt, rms, n
                    best = ref

    info["inlier_fraction"] = best.inliers / float(n)
    if info["inlier_fraction"] < min_inlier_fraction:
        info["fell_back"] = True
        fb = Plane(*seed.as_tuple())
        fb.inliers, fb.rms, _ = score(fb.nx, fb.ny, fb.nz, fb.d)
        fb.n_points = n
        return fb, info
    return best, info


# --------------------------------------------------------------------------- #
# THE ROS NODE
# --------------------------------------------------------------------------- #

class ObstacleSegmenter(object):
    """Subscribe to the ZED cloud, publish the obstacle cloud and the curb scan."""

    def __init__(self, cfg, mount, log):
        self.cfg = cfg
        self.mount = mount
        self.log = log
        self.rospy = None
        self.np = None
        self.tf_buffer = None
        self.pubs = {}
        self.rng = random.Random(cfg["random_seed"])
        self.frames = 0
        self.frames_no_tf = 0
        self.frames_fallback_plane = 0
        self.frames_empty = 0
        self.last_log = 0.0
        self.smoothed = None       # temporally smoothed plane

    # ---- setup ---------------------------------------------------------- #

    def setup(self):
        self.np = require_numpy_runtime()
        self.rospy = require_rospy()
        require_master(self.rospy, timeout_s=3.0)

        try:
            import tf2_ros
        except ImportError:
            raise Unavailable(
                "tf2_ros is not importable, so the point cloud cannot be "
                "transformed from the camera's optical frame into base_link. "
                "Every threshold in this node is expressed in base_link, so "
                "running without TF would silently produce meaningless output.",
                "sudo apt install ros-noetic-tf2-ros, then re-source the "
                "workspace.")

        from sensor_msgs.msg import PointCloud2, LaserScan  # noqa: F401

        self.rospy.init_node("obstacle_segmenter", anonymous=False,
                             disable_signals=True)
        self.tf_buffer = tf2_ros.Buffer(self.rospy.Duration(5.0))
        self._tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        c = self.cfg
        self.pubs["obstacles"] = self.rospy.Publisher(
            c["obstacles_topic"], PointCloud2, queue_size=1)
        self.pubs["curb_scan"] = self.rospy.Publisher(
            c["curb_scan_topic"], LaserScan, queue_size=1)
        if c["publish_debug_clouds"]:
            self.pubs["curb_cloud"] = self.rospy.Publisher(
                c["curb_cloud_topic"], PointCloud2, queue_size=1)
            self.pubs["ground_cloud"] = self.rospy.Publisher(
                c["ground_cloud_topic"], PointCloud2, queue_size=1)

        if c["source"] != "cloud":
            raise Unavailable(
                "source='%s' is not implemented. Only 'cloud' is supported: "
                "the node consumes the registered PointCloud2 from the ZED "
                "wrapper." % c["source"],
                "Set source: cloud in config/obstacle_segmenter.yaml, and "
                "confirm the wrapper is publishing %s." % c["cloud_topic"])

        self.log.info("Subscribing to %s" % c["cloud_topic"])
        self.sub = self.rospy.Subscriber(
            c["cloud_topic"], PointCloud2, self.on_cloud, queue_size=1,
            buff_size=2 ** 24)

    def wait_for_input(self, timeout_s=20.0):
        """Confirm the camera is actually producing a cloud before we settle in.

        Without this the node sits silently forever when the camera is not
        wired, which reads to an operator as "the segmenter is broken" when in
        fact nothing ever arrived. The distinction matters: one is a software
        bug, the other is a coax connector.
        """
        from sensor_msgs.msg import PointCloud2
        try:
            self.rospy.wait_for_message(self.cfg["cloud_topic"], PointCloud2,
                                        timeout=timeout_s)
        except Exception:
            raise Unavailable(
                "no point cloud arrived on %s within %.0f s."
                % (self.cfg["cloud_topic"], timeout_s),
                "In order: (1) is the ZED node running — `rosnode list | grep "
                "zed`? (2) is the point cloud enabled — the wrapper publishes "
                "it only when point_cloud_freq > 0, and zedx_front.yaml sets "
                "10.0; (3) is the camera itself alive — `rostopic hz "
                "/zedx_front/zed_node/depth/depth_registered`. If depth flows "
                "but the cloud does not, it is a wrapper setting, not the "
                "camera.")

    # ---- per-frame work -------------------------------------------------- #

    def lookup_prior(self, source_frame, stamp):
        """Ground-plane prior for THIS frame, derived from TF.

        In base_link the ground is z = 0 with normal +z, by definition of the
        frame. So the prior in base_link is always the seed plane, and what TF
        actually gives us is the transform that puts the cloud INTO base_link.
        That transform is where the mount geometry — and therefore the prior —
        really enters. Returns (matrix, plane_prior) or (None, None).
        """
        try:
            tr = self.tf_buffer.lookup_transform(
                self.cfg["target_frame"], source_frame, stamp,
                self.rospy.Duration(self.cfg["tf_timeout_s"]))
        except Exception:
            return None, None
        return transform_matrix_np(tr, self.np), SEED_PLANE

    def on_cloud(self, msg):
        try:
            self._process(msg)
        except Unavailable as exc:
            # A per-frame Unavailable is a configuration error that will repeat
            # on every frame. Say it once per log period rather than 10x/s.
            self._throttled(lambda: self.log.error(
                "cannot process cloud: %s  FIX: %s" % (exc.reason, exc.fix)))
        except Exception as exc:              # never let one bad frame kill us
            self._throttled(lambda: self.log.error(
                "unexpected error on a frame (%s: %s). Skipping it; the node "
                "stays up so the costmap keeps receiving whatever else works."
                % (exc.__class__.__name__, exc)))

    def _throttled(self, fn):
        now = self.rospy.get_time()
        if now - self.last_log >= self.cfg["log_period_s"]:
            self.last_log = now
            fn()

    def _process(self, msg):
        np = self.np
        c = self.cfg
        self.frames += 1

        M, prior = self.lookup_prior(msg.header.frame_id, msg.header.stamp)
        if M is None:
            self.frames_no_tf += 1
            self._throttled(lambda: self.log.warn(
                "no transform %s -> %s at the cloud's timestamp. Publishing "
                "nothing this frame. If this persists, sidewalk_bringup is not "
                "publishing the static frame tree — check `rosrun rqt_tf_tree "
                "rqt_tf_tree`."
                % (msg.header.frame_id, c["target_frame"])))
            return

        xyz = cloud_to_xyz_np(msg, np, max_points=c["max_points"])
        if xyz.shape[0] == 0:
            self.frames_empty += 1
            self._publish_empty(msg.header.stamp)
            return
        xyz = apply_transform_np(xyz, M, np)

        # Region of interest, vectorised.
        m = ((xyz[:, 0] >= c["roi_x_min"]) & (xyz[:, 0] <= c["roi_x_max"])
             & (np.abs(xyz[:, 1]) <= c["roi_y_abs_max"])
             & (xyz[:, 2] >= c["roi_z_min"]) & (xyz[:, 2] <= c["roi_z_max"]))
        roi = xyz[m]
        if roi.shape[0] < c["min_ground_points"]:
            self.frames_empty += 1
            self._throttled(lambda: self.log.warn(
                "only %d points in the region of interest (need %d). Either "
                "the camera is looking at open sky, or the ROI in the config "
                "does not match where the ground actually is — run "
                "obstacle_segmenter.py --geometry to see where that is."
                % (roi.shape[0], c["min_ground_points"])))
            self._publish_empty(msg.header.stamp)
            return

        band = c["ground_band_m"] + c["max_plane_offset_m"]
        seed = self.smoothed or prior
        hprior = (roi[:, 0] * seed.nx + roi[:, 1] * seed.ny
                  + roi[:, 2] * seed.nz + seed.d)
        cand = roi[np.abs(hprior) <= band]
        if cand.shape[0] < 3:
            cand = roi

        plane, info = fit_ground_plane_np(
            cand, np,
            tol=c["ransac_inlier_tol_m"],
            iterations=c["ransac_iterations"],
            max_tilt_deg=c["max_plane_tilt_deg"],
            max_offset_m=c["max_plane_offset_m"],
            min_inlier_fraction=c["ransac_min_inlier_fraction"],
            seed=seed, rng=self.rng)
        if info["fell_back"]:
            self.frames_fallback_plane += 1

        plane = self._smooth(plane)

        # Obstacles.
        h = roi[:, 0] * plane.nx + roi[:, 1] * plane.ny + roi[:, 2] * plane.nz \
            + plane.d
        obs = roi[(h >= c["obstacle_min_height"])
                  & (h <= c["obstacle_max_height"])]

        # Ground grid and steps: run on the near-ground subset only, which is a
        # small fraction of the cloud, so the pure-Python reference code is
        # fast enough here and we get to use ONE implementation for the part of
        # the algorithm that is hardest to get right.
        # Restrict the grid to points that could actually contribute a step.
        # detect_steps only reports edges whose NEAR cell is within
        # curb_max_range_m, so a cell beyond that plus one neighbour's width is
        # dead weight. This is not just an optimisation: build_ground_grid runs
        # in pure Python (deliberately — it is the part of the algorithm most
        # worth having a single, self-tested implementation of), so its cost is
        # linear in the point count and it is the one place in the frame where
        # Python speed could threaten the 10 Hz budget.
        grid_limit = c["curb_max_range_m"] + 2.0 * c["grid_res_m"]
        near_m = ((np.abs(h) <= c["ground_band_m"])
                  & (np.hypot(roi[:, 0], roi[:, 1]) <= grid_limit))
        near = roi[near_m]
        near_list = [tuple(p) for p in near.tolist()]
        cells = build_ground_grid(near_list, plane,
                                  res=c["grid_res_m"],
                                  ground_band=c["ground_band_m"],
                                  min_cell_points=c["min_cell_points"])
        steps = detect_steps(cells,
                             min_step=c["curb_min_step_m"],
                             max_step=c["curb_max_step_m"],
                             min_range=c["curb_min_range_m"],
                             max_range=c["curb_max_range_m"])
        ranges, kinds = steps_to_scan(
            steps,
            angle_min=c["scan_angle_min"], angle_max=c["scan_angle_max"],
            increment=c["scan_angle_increment"],
            range_min=c["scan_range_min"], range_max=c["scan_range_max"])

        obs_list = [tuple(p) for p in obs.tolist()]
        if c["mark_down_steps_in_obstacle_cloud"]:
            obs_list += down_step_virtual_points(
                steps, plane, c["down_step_virtual_heights"],
                c["obstacle_min_height"], c["obstacle_max_height"])

        stamp = msg.header.stamp
        self._publish_cloud("obstacles", obs_list, stamp)
        self._publish_scan(ranges, stamp)
        if c["publish_debug_clouds"]:
            curb_pts = []
            for s in steps:
                z = (-(plane.nx * s.x + plane.ny * s.y + plane.d) / plane.nz
                     if abs(plane.nz) > 1e-6 else 0.0)
                curb_pts.append((s.x, s.y, z + max(0.0, s.delta)))
            self._publish_cloud("curb_cloud", curb_pts, stamp)
            self._publish_cloud("ground_cloud", near_list[::4], stamp)

        n_down = sum(1 for s in steps if s.kind == "down")
        self._throttled(lambda: self._heartbeat(plane, info, obs_list, steps,
                                                n_down, roi.shape[0]))

    def _smooth(self, plane):
        """Exponentially smooth the plane across frames.

        The plane is a property of the pavement and the robot's suspension.
        Neither changes materially in 100 ms, so a plane that jumps frame to
        frame is measuring noise, not geometry — and every jump moves the
        0.10 m obstacle floor, which makes obstacles flicker in and out of the
        costmap at the boundary. Smoothing costs a little lag when the robot
        genuinely pitches; that lag is bounded by the same physics that makes
        the smoothing safe.

        `plane_smoothing` is the weight given to the NEW measurement.
        """
        a = self.cfg["plane_smoothing"]
        if self.smoothed is None or a >= 1.0:
            self.smoothed = plane
            return plane
        s = self.smoothed
        nx = a * plane.nx + (1 - a) * s.nx
        ny = a * plane.ny + (1 - a) * s.ny
        nz = a * plane.nz + (1 - a) * s.nz
        d = a * plane.d + (1 - a) * s.d
        norm = math.sqrt(nx * nx + ny * ny + nz * nz)
        if norm < 1e-9:
            self.smoothed = plane
            return plane
        out = Plane(nx / norm, ny / norm, nz / norm, d / norm,
                    plane.inliers, plane.n_points, plane.rms)
        self.smoothed = out
        return out

    def _heartbeat(self, plane, info, obs, steps, n_down, n_roi):
        self.log.info(
            "frame %d | roi %d pts | plane tilt %.2f deg, offset %+.3f m, "
            "inliers %.0f%% | obstacles %d | steps %d (down %d)%s"
            % (self.frames, n_roi, plane.tilt_deg, plane.offset_at_origin,
               100.0 * info.get("inlier_fraction", 0.0), len(obs), len(steps),
               n_down, "  [PLANE FIT FELL BACK TO TF PRIOR]"
               if info.get("fell_back") else ""))

    # ---- publishing ------------------------------------------------------ #

    def _cloud_msg(self, points, stamp):
        from sensor_msgs.msg import PointCloud2, PointField
        np = self.np
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = self.cfg["target_frame"]
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(points)
        msg.is_dense = True
        if points:
            arr = np.asarray(points, dtype=np.float32)
            msg.data = arr.tobytes()
        else:
            msg.data = b""
        return msg

    def _publish_cloud(self, key, points, stamp):
        pub = self.pubs.get(key)
        if pub is not None:
            pub.publish(self._cloud_msg(points, stamp))

    def _publish_scan(self, ranges, stamp):
        from sensor_msgs.msg import LaserScan
        c = self.cfg
        msg = LaserScan()
        msg.header.stamp = stamp
        msg.header.frame_id = c["scan_frame_id"]
        msg.angle_min = c["scan_angle_min"]
        msg.angle_increment = c["scan_angle_increment"]
        # angle_max is DERIVED from the bin count, not copied from the config.
        # The configured arc (pi rad) is not an exact multiple of the 0.5 deg
        # increment, so ranges[] holds 360 bins covering 359 increments and the
        # last bearing is 1.5622 rad, not the configured 1.5708. Publishing the
        # configured value would make the message internally inconsistent:
        # consumers that size their bearing table from
        # round((angle_max-angle_min)/angle_increment)+1 would expect 361 bins
        # and disagree with laser_geometry, which sizes from len(ranges).
        # Every curb bearing would then be reported ~0.5 deg off.
        msg.angle_max = (c["scan_angle_min"]
                         + max(0, len(ranges) - 1) * c["scan_angle_increment"])
        msg.time_increment = 0.0
        msg.scan_time = 1.0 / max(1e-3, c["rate_hz"])
        msg.range_min = c["scan_range_min"]
        msg.range_max = c["scan_range_max"]
        msg.ranges = ranges
        msg.intensities = []
        self.pubs["curb_scan"].publish(msg)

    def _publish_empty(self, stamp):
        """Publish EMPTY messages rather than nothing when a frame yields nothing.

        This is deliberate and it is not the same as publishing nothing.
        costmap_common.yaml sets `expected_update_rate: 0.5` on both sources;
        going silent trips that and costmap_2d starts warning about a stale
        sensor, which safety_monitor.py acts on. An empty cloud says "I looked
        and there was nothing", which is true and is what we mean. Silence says
        "I have stopped working", which would be a lie.
        """
        self._publish_cloud("obstacles", [], stamp)
        nbins = int(math.floor((self.cfg["scan_angle_max"]
                                - self.cfg["scan_angle_min"])
                               / self.cfg["scan_angle_increment"])) + 1
        self._publish_scan([float("inf")] * nbins, stamp)
        if self.cfg["publish_debug_clouds"]:
            self._publish_cloud("curb_cloud", [], stamp)
            self._publish_cloud("ground_cloud", [], stamp)

    # ---- main loop ------------------------------------------------------- #

    def run(self):
        self.setup()
        self.log.section("Geometry")
        for line in geometry_report(self.mount, self.cfg).splitlines():
            self.log.info(line)
        self.log.section("Waiting for the camera")
        self.wait_for_input()
        self.log.info("Cloud is flowing. Segmenting.")
        self.log.section("Running")
        try:
            self.rospy.spin()
        except KeyboardInterrupt:
            self.log.warn("Interrupted by operator.")
            self._emit_summary()
            return EXIT_INTERRUPTED
        self._emit_summary()
        return EXIT_PASS

    def _emit_summary(self):
        self.log.metric("frames_processed", self.frames)
        self.log.metric("frames_without_tf", self.frames_no_tf)
        self.log.metric("frames_empty_roi", self.frames_empty)
        self.log.metric("frames_plane_fell_back", self.frames_fallback_plane)
        status = "OK"
        if self.frames == 0:
            status = "FAIL"
        elif (self.frames_no_tf + self.frames_fallback_plane) > self.frames / 4:
            status = "WARN"
        self.log.summary(
            "Segmented %d frames; %d had no TF, %d had an empty ROI, %d fell "
            "back to the TF ground prior."
            % (self.frames, self.frames_no_tf, self.frames_empty,
               self.frames_fallback_plane), status=status)


# --------------------------------------------------------------------------- #
# SELF-TEST
#
# Pure Python stdlib. No numpy at import, no ROS, no camera. Every scene below
# is synthetic and every assertion is a property we can state before running
# it. This is the only correctness evidence that exists for this node, so it is
# deliberately adversarial rather than confirmatory: several cases exist to
# demonstrate the algorithm FAILING when its safeguards are removed.
# --------------------------------------------------------------------------- #

def _grid_points(x0, x1, y0, y1, step, zfn):
    pts = []
    x = x0
    while x <= x1 + 1e-9:
        y = y0
        while y <= y1 + 1e-9:
            pts.append((x, y, zfn(x, y)))
            y += step
        x += step
    return pts


def _noisy(pts, sigma, rng):
    if sigma <= 0:
        return pts
    return [(x, y, z + rng.gauss(0.0, sigma)) for x, y, z in pts]


class _T(object):
    """Minimal test harness. No pytest on the Jetson, and none needed."""

    def __init__(self, log):
        self.log = log
        self.passed = 0
        self.failed = 0

    def check(self, name, ok, detail=""):
        if ok:
            self.passed += 1
            self.log.info("  PASS  %-56s %s" % (name, detail))
        else:
            self.failed += 1
            self.log.error("  FAIL  %-56s %s" % (name, detail))
        return ok

    def close(self, name, a, b, tol, detail=""):
        return self.check(name, abs(a - b) <= tol,
                          detail or "%.4f vs %.4f (tol %.4f)" % (a, b, tol))


def self_test(log, cfg=None, mount=None):
    """Run every synthetic scene. Returns EXIT_PASS or EXIT_FAIL."""
    cfg = dict(DEFAULTS if cfg is None else cfg)
    mount = mount or MOUNT
    t = _T(log)
    rng = random.Random(20260720)

    # ------------------------------------------------------------------ #
    log.section("1. Mount geometry")
    # ------------------------------------------------------------------ #
    h = mount["cam_height_m"]
    pitch = mount["cam_pitch_rad"]
    x_first, binds = ground_first_visible(h, pitch)
    t.check("ground first visible is ahead of the robot, not at its feet",
            x_first > 0.8, "%.3f m ahead of the lens" % x_first)
    t.check("the binding constraint is identified",
            binds in ("fov", "depth"), binds)
    # ------------------------------------------------------------------ #
    # THE DOCSTRING CHECK — UNCONDITIONAL BY CONSTRUCTION.
    #
    # WHAT THIS REPLACES, AND WHY THE REPLACEMENT IS NOT COSMETIC. The
    # previous version of this check was:
    #
    #     if abs(h - 0.62) < 1e-6 and abs(pitch - 0.087) < 1e-6:
    #         t.close("matches the hand calculation in the docstring",
    #                 x_first, 1.032, 0.005)
    #         t.check("FOV binds, not the 1 m minimum depth", ...)
    #
    # — it compared the docstring against reality ONLY WHILE THE MOUNT STILL
    # HELD THE OLD PLACEHOLDER VALUES. The day the camera was actually
    # measured, h and pitch changed, the guard went false, both checks inside
    # it stopped running, and the suite carried on printing PASS with no hint
    # that the docstring's figures were no longer being policed by anything.
    # A guard that switches a check off at exactly the moment the guarded
    # thing changes is worse than having no check: silence reads as success,
    # and the numbers it was protecting are safety geometry.
    #
    # The replacement takes its EXPECTED values from the docstring text and
    # its ACTUAL values from the geometry functions, so there is no value it
    # can be gated on. Re-measure robot_frames.yaml and this FAILS, naming
    # the figure and printing both numbers, until somebody runs --geometry
    # and pastes the new block in. That is the designed behaviour: the
    # docstring is a safety claim, and a stale safety claim should stop the
    # suite, not decorate it.
    # ------------------------------------------------------------------ #
    doc_figs = documented_figures()
    live_figs = computed_figures(mount)
    missing = [n for n in DOCUMENTED_FIGURE_NAMES if n not in doc_figs]
    t.check("the docstring's checked-figures block is complete",
            not missing,
            "all %d figures present" % len(DOCUMENTED_FIGURE_NAMES) if not missing
            else "MISSING from the docstring: %s — a deleted line must fail "
                 "here, not silently shrink the comparison" % ", ".join(missing))
    for name in DOCUMENTED_FIGURE_NAMES:
        if name not in doc_figs:
            continue
        written, places = doc_figs[name]
        # Half of the last written digit: the docstring states the figure to
        # the precision it states it to, and nothing finer is being claimed.
        tol = 0.5 * 10.0 ** (-places) + 1e-9
        t.close("docstring %-21s matches the live mount" % name,
                live_figs[name], written, tol,
                "computed %.5f vs docstring %.5f (tol %.5f) — if this fails, "
                "run --geometry and paste, do not hand-edit"
                % (live_figs[name], written, tol))
    # Reported unconditionally, whichever constraint binds. When the FOV binds
    # this sits above the camera's minimum depth with margin; when the minimum
    # depth binds it equals that floor by construction. Either way the first
    # ground point must be somewhere the camera is willing to measure — if it
    # is not, `ground_first_visible` has returned a point that does not exist.
    t.check("the first ground point is at or beyond the camera's depth floor",
            live_figs["optical_depth_there_m"] >= ZEDX["depth_min_m"] - 1e-9,
            "%s binds; optical depth there is %.3f m, floor %.2f m"
            % (binds.upper(), live_figs["optical_depth_there_m"],
               ZEDX["depth_min_m"]))
    blind = (x_first + mount["cam_x_offset_m"]) - mount["footprint_length_m"] / 2.0
    t.check("a near blind zone exists and is reported", blind > 0.0,
            "%.3f m deep, from the bumper to the first ground return" % blind)

    # Noise model sanity: height noise must be LINEAR in range, which is the
    # non-obvious claim the range limit argument rests on.
    r5, r10 = height_noise_m(5.0, h), height_noise_m(10.0, h)
    t.close("height noise is linear in range (2x range -> 2x noise)",
            r10 / r5, 2.0, 0.05, "%.4f m at 5 m, %.4f m at 10 m" % (r5, r10))
    g5, g10 = along_ground_noise_m(5.0, h), along_ground_noise_m(10.0, h)
    t.close("edge-localisation noise is quadratic (2x range -> 4x noise)",
            g10 / g5, 4.0, 0.10, "%.3f m at 5 m, %.3f m at 10 m" % (g5, g10))
    # The property that matters is NOT that the configured range sits close to
    # the depth-noise limit — it is that it does not EXCEED it. Two independent
    # limits apply (depth noise, and cell population after decimation) and the
    # configured value must respect the tighter of the two. Sitting comfortably
    # inside both is a correct and desirable state, not a discrepancy.
    #
    # The earlier version of this check asserted proximity to the depth-noise
    # limit alone. That silently encoded the assumption that depth noise always
    # binds. It does not: with decimation to max_points, cell population binds
    # first, and asserting proximity would push the range back UP into the band
    # where the ground grid is too sparse to form cells at all.
    _noise_limit = curb_range_limit_m(0.05, h)
    t.check("configured curb range does not exceed the depth-noise limit",
            cfg["curb_max_range_m"] <= _noise_limit + 1e-9,
            "configured %.2f m <= depth-noise limit %.2f m"
            % (cfg["curb_max_range_m"], _noise_limit))

    # ------------------------------------------------------------------ #
    log.section("2. Threshold self-consistency")
    # ------------------------------------------------------------------ #
    t.check("inlier tolerance is below the minimum curb height",
            cfg["ransac_inlier_tol_m"] < 0.10,
            "%.3f m < 0.100 m — otherwise a curb top joins the ground plane"
            % cfg["ransac_inlier_tol_m"])
    t.check("curb min step is above the inlier tolerance",
            cfg["curb_min_step_m"] > cfg["ransac_inlier_tol_m"],
            "%.3f m > %.3f m" % (cfg["curb_min_step_m"],
                                 cfg["ransac_inlier_tol_m"]))
    t.check("curb min step is below a standard 100 mm curb",
            cfg["curb_min_step_m"] < 0.10, "%.3f m" % cfg["curb_min_step_m"])
    t.check("curb max step is above a tall 150 mm curb",
            cfg["curb_max_step_m"] > 0.15, "%.3f m" % cfg["curb_max_step_m"])
    t.check("ground band exceeds the tallest detectable step",
            cfg["ground_band_m"] > cfg["curb_max_step_m"],
            "%.2f m > %.2f m" % (cfg["ground_band_m"], cfg["curb_max_step_m"]))
    t.check("obstacle floor is above the inlier tolerance",
            cfg["obstacle_min_height"] > cfg["ransac_inlier_tol_m"],
            "%.3f m > %.3f m" % (cfg["obstacle_min_height"],
                                 cfg["ransac_inlier_tol_m"]))
    t.check("ROI starts at or beyond the first visible ground",
            cfg["roi_x_min"] >= 0.9,
            "roi_x_min %.2f m (ground appears at %.2f m in base_link)"
            % (cfg["roi_x_min"], x_first + mount["cam_x_offset_m"]))
    t.check("curb max range does not exceed the ROI",
            cfg["curb_max_range_m"] <= cfg["roi_x_max"],
            "%.1f m <= %.1f m" % (cfg["curb_max_range_m"], cfg["roi_x_max"]))
    t.check("topic names match costmap_common.yaml exactly",
            cfg["obstacles_topic"] == "/sidewalk_perception/obstacles_cloud"
            and cfg["curb_scan_topic"] == "/sidewalk_perception/curb_scan",
            "%s , %s" % (cfg["obstacles_topic"], cfg["curb_scan_topic"]))

    # ------------------------------------------------------------------ #
    log.section("3. Flat ground, nothing on it")
    # ------------------------------------------------------------------ #
    flat = _noisy(_grid_points(1.0, 8.0, -3.0, 3.0, 0.10, lambda x, y: 0.0),
                  0.005, rng)
    res = segment(flat, cfg, rng=random.Random(1))
    t.check("plane found", res["ok"] and res["plane"] is not None)
    t.close("plane is level", res["plane"].tilt_deg, 0.0, 1.0,
            "tilt %.3f deg" % res["plane"].tilt_deg)
    t.close("plane passes through z = 0", res["plane"].offset_at_origin, 0.0,
            0.02, "offset %+.4f m" % res["plane"].offset_at_origin)
    t.check("no obstacles on empty pavement", len(res["obstacles"]) == 0,
            "%d points" % len(res["obstacles"]))
    t.check("no curbs on empty pavement", len(res["steps"]) == 0,
            "%d steps" % len(res["steps"]))

    # ------------------------------------------------------------------ #
    log.section("4. Camber + robot pitch: the case a fixed height cut fails")
    # ------------------------------------------------------------------ #
    # 2 % cross-slope for drainage, plus 1.5 deg of nose-up pitch under
    # acceleration. Both are entirely normal and entirely drivable.
    slope_y = 0.02
    pitch_rad = math.radians(1.5)
    def tilted(x, y):
        return -x * math.tan(pitch_rad) + abs(y) * slope_y * -1.0
    # A real 0.30 m obstacle at 4 m, 0.4 m across.
    def with_box(x, y):
        z = tilted(x, y)
        if 4.0 <= x <= 4.4 and -0.2 <= y <= 0.2:
            z += 0.30
        return z
    scene = _noisy(_grid_points(1.0, 8.0, -4.0, 4.0, 0.10, with_box), 0.005, rng)

    naive = [p for p in scene
             if cfg["roi_x_min"] <= p[0] <= cfg["roi_x_max"]
             and abs(p[2]) >= cfg["obstacle_min_height"]]
    t.check("a fixed height cut DOES misfire on camber and pitch",
            len(naive) > 200,
            "%d points wrongly called obstacles by |z| > %.2f m"
            % (len(naive), cfg["obstacle_min_height"]))

    res = segment(scene, cfg, rng=random.Random(2))
    t.check("fitted plane absorbs the tilt",
            res["plane"].tilt_deg > 1.0,
            "fitted tilt %.2f deg (scene has 1.5 deg pitch + 1.1 deg camber)"
            % res["plane"].tilt_deg)
    # Everything that survives must be the box.
    survivors = res["obstacles"]
    on_box = [p for p in survivors if 3.9 <= p[0] <= 4.5 and abs(p[1]) <= 0.3]
    t.check("the real obstacle is found", len(on_box) > 5,
            "%d points on the box" % len(on_box))
    t.check("the drivable pavement is NOT called an obstacle",
            len(survivors) - len(on_box) < 0.02 * len(naive),
            "%d false positives, versus %d for the fixed cut"
            % (len(survivors) - len(on_box), len(naive)))

    # ------------------------------------------------------------------ #
    log.section("5. Wall-dominated scene: why the seed matters")
    # ------------------------------------------------------------------ #
    # A building wall along y = +2 m, densely sampled (it faces the camera, so
    # it genuinely does get more pixels than the pavement), plus the pavement.
    ground = _grid_points(1.0, 8.0, -2.0, 2.0, 0.20, lambda x, y: 0.0)
    wall = []
    x = 1.0
    while x <= 8.0:
        z = 0.0
        while z <= 3.0:
            wall.append((x, 2.0, z))
            z += 0.04
        x += 0.04
    scene = _noisy(ground + wall, 0.004, rng)
    t.check("the wall really does outnumber the ground",
            len(wall) > 2 * len(ground),
            "%d wall points vs %d ground points" % (len(wall), len(ground)))

    unseeded, _ = fit_ground_plane(scene, tol=cfg["ransac_inlier_tol_m"],
                                   iterations=200, rng=random.Random(3),
                                   enforce_gates=False)
    t.check("UNSEEDED RANSAC fits the wall, not the ground",
            unseeded.tilt_deg > 45.0,
            "unseeded plane tilt %.1f deg — this is the failure the gates exist "
            "to prevent" % unseeded.tilt_deg)

    seeded, info = fit_ground_plane(scene, tol=cfg["ransac_inlier_tol_m"],
                                    iterations=200,
                                    max_tilt_deg=cfg["max_plane_tilt_deg"],
                                    max_offset_m=cfg["max_plane_offset_m"],
                                    rng=random.Random(3))
    t.check("SEEDED RANSAC fits the ground", seeded.tilt_deg < 2.0,
            "seeded plane tilt %.3f deg, rejected %d wall candidates on tilt"
            % (seeded.tilt_deg, info["rejected_tilt"]))
    t.close("seeded plane sits at z = 0", seeded.offset_at_origin, 0.0, 0.02)

    res = segment(scene, cfg, rng=random.Random(3))
    wall_marked = [p for p in res["obstacles"] if abs(p[1] - 2.0) < 0.15]
    t.check("the wall is marked as an obstacle (not as ground)",
            len(wall_marked) > 100, "%d wall points marked" % len(wall_marked))
    ground_marked = [p for p in res["obstacles"] if abs(p[1] - 2.0) >= 0.15]
    t.check("the pavement is not marked", len(ground_marked) == 0,
            "%d pavement points wrongly marked" % len(ground_marked))

    # ------------------------------------------------------------------ #
    log.section("6. Up-step curb (kerb)")
    # ------------------------------------------------------------------ #
    def up_curb(x, y):
        return 0.12 if x >= 3.0 else 0.0
    scene = _noisy(_grid_points(1.0, 5.0, -2.0, 2.0, 0.05, up_curb), 0.004, rng)
    res = segment(scene, cfg, rng=random.Random(4))
    ups = [s for s in res["steps"] if s.kind == "up"]
    t.check("an up-step is detected", len(ups) > 0, "%d up-steps" % len(ups))
    t.check("no down-steps are invented",
            len([s for s in res["steps"] if s.kind == "down"]) == 0)
    if ups:
        xs = sorted(s.x for s in ups)
        mid = xs[len(xs) // 2]
        t.close("reported at the near lip of the step", mid, 2.95, 0.15,
                "median x %.3f m (true edge at 3.00 m)" % mid)
        t.close("step magnitude matches the 0.12 m curb",
                abs(ups[0].delta), 0.12, 0.02)
    ranges = res["ranges"]
    hits = [r for r in ranges if r != float("inf")]
    t.check("the curb appears in the synthetic scan", len(hits) > 0,
            "%d bearings with a return" % len(hits))

    # ------------------------------------------------------------------ #
    log.section("7. Down-step (drop to the roadway) — the dangerous one")
    # ------------------------------------------------------------------ #
    def drop(x, y):
        return -0.15 if x >= 3.0 else 0.0
    scene = _noisy(_grid_points(1.0, 5.0, -2.0, 2.0, 0.05, drop), 0.004, rng)
    res = segment(scene, cfg, rng=random.Random(5))
    downs = [s for s in res["steps"] if s.kind == "down"]
    t.check("a down-step is detected", len(downs) > 0,
            "%d down-steps" % len(downs))
    t.check("it is NOT classified as an up-step",
            len([s for s in res["steps"] if s.kind == "up"]) == 0)
    if downs:
        xs = sorted(s.x for s in downs)
        mid = xs[len(xs) // 2]
        t.check("reported at the NEAR lip, before the drop", mid < 3.0,
                "median x %.3f m (drop begins at 3.00 m)" % mid)

    # The critical safety property: a drop must not read as free space.
    naive_obs = [p for p in scene if 0.10 <= p[2] <= 1.90]
    t.check("a height-band filter alone sees NOTHING at a drop-off",
            len(naive_obs) == 0,
            "0 points in the 0.10-1.90 m band — the road is below the plane")
    virt = down_step_virtual_points(downs, res["plane"],
                                    cfg["down_step_virtual_heights"],
                                    cfg["obstacle_min_height"],
                                    cfg["obstacle_max_height"])
    t.check("virtual barrier points are synthesised on the near lip",
            len(virt) > 0, "%d virtual points" % len(virt))
    t.check("virtual points land inside the costmap obstacle band",
            all(cfg["obstacle_min_height"] <= p[2] <= cfg["obstacle_max_height"]
                for p in virt),
            "z from %.2f to %.2f m" % (min(p[2] for p in virt),
                                       max(p[2] for p in virt)) if virt else "")
    t.check("they are present in the published obstacle cloud",
            any(p[2] > 0.10 for p in res["obstacles"]),
            "%d obstacle points total" % len(res["obstacles"]))

    # ------------------------------------------------------------------ #
    log.section("8. Range gating: curbs beyond the noise limit are dropped")
    # ------------------------------------------------------------------ #
    def far_curb(x, y):
        return 0.12 if x >= 7.0 else 0.0
    # Point spacing MUST be finer than grid_res_m or every cell falls below
    # min_cell_points and the grid is empty — which is a real effect, tested
    # separately below, but would make this a vacuous test of the range gate.
    scene = _noisy(_grid_points(1.0, 8.0, -2.0, 2.0, 0.05, far_curb), 0.004, rng)
    res = segment(scene, cfg, rng=random.Random(6))
    t.check("a curb at 7 m is NOT emitted (limit is %.1f m)"
            % cfg["curb_max_range_m"],
            all(r == float("inf") for r in res["ranges"]),
            "%d steps found internally, %d emitted"
            % (len(res["steps"]),
               len([r for r in res["ranges"] if r != float("inf")])))

    wide = dict(cfg)
    wide["curb_max_range_m"] = 8.0
    res2 = segment(scene, wide, rng=random.Random(6))
    t.check("...but IS emitted when the limit is raised, proving the gate is "
            "the reason", any(r != float("inf") for r in res2["ranges"]),
            "%d bearings with a return"
            % len([r for r in res2["ranges"] if r != float("inf")]))

    # ------------------------------------------------------------------ #
    log.section("8b. Ground sampling density — a SECOND, independent range limit")
    # ------------------------------------------------------------------ #
    # Discovered while debugging the test above: if the point spacing reaches
    # grid_res_m, every cell falls below min_cell_points and the ground grid
    # empties, so no steps can be found at all. On the real camera the ground
    # sampling interval grows as z^2, so this imposes its own range ceiling
    # entirely separately from the depth-noise argument — and it fails SILENTLY
    # (no steps, no error) rather than noisily, which is the dangerous kind.
    sparse = _noisy(_grid_points(1.0, 5.0, -2.0, 2.0, cfg["grid_res_m"],
                                 lambda x, y: 0.12 if x >= 3.0 else 0.0),
                    0.004, rng)
    r_sparse = segment(sparse, cfg, rng=random.Random(60))
    t.check("point spacing == grid_res empties the grid (silent, no error)",
            len(r_sparse.get("cells") or {}) == 0
            and len(r_sparse["steps"]) == 0,
            "%d cells, %d steps — this is why min_cell_points needs a range "
            "argument behind it" % (len(r_sparse.get("cells") or {}),
                                    len(r_sparse["steps"])))
    dense = _noisy(_grid_points(1.0, 5.0, -2.0, 2.0, cfg["grid_res_m"] / 3.0,
                                lambda x, y: 0.12 if x >= 3.0 else 0.0),
                   0.004, rng)
    r_dense = segment(dense, cfg, rng=random.Random(60))
    t.check("...and adequate density restores detection", len(r_dense["steps"]) > 0,
            "%d steps at 1/3 the cell size" % len(r_dense["steps"]))
    # Where does the REAL camera cross that line? Note carefully that the
    # answer depends on max_points, not only on the sensor: the node strides
    # the cloud before it ever builds a grid. An earlier version of this check
    # used the raw sensor resolution, concluded the grid was good to 9 m, and
    # was wrong by more than the curb range itself.
    hole_range = grid_hole_range_m(cfg["grid_res_m"], mount["cam_height_m"])
    pess, opt = cell_population_limits_m(cfg["max_points"],
                                         cfg["min_cell_points"],
                                         cfg["grid_res_m"],
                                         mount["cam_height_m"])
    t.check("the sensor-only grid limit is NOT used as the bound (it ignores "
            "max_points and is far too optimistic)",
            hole_range > opt,
            "sensor-only %.2f m vs decimation-aware %.2f m — the difference "
            "IS the decimation" % (hole_range, opt))
    t.check("the optimistic (raster-aliased) cell-population limit covers the "
            "configured curb range",
            opt >= cfg["curb_max_range_m"],
            "%.2f m vs curb limit %.2f m" % (opt, cfg["curb_max_range_m"]))
    # THE IMPORTANT ONE. The configured range must sit inside the PESSIMISTIC
    # limit, not merely the optimistic one.
    #
    # Which of the two density models actually applies has not been measured. If
    # the optimistic (raster-aliasing) model were assumed and the uniform model
    # turned out to be nearer the truth, then between the two limits the ground
    # grid would hold too few points per cell to form cells at all — and an empty
    # curb scan is indistinguishable from "there is no curb there". The robot
    # would report clear ground, confidently, in a band where it is simply blind.
    #
    # A curb is the boundary between pavement and road, so that is the one
    # silent failure this system least affords. Respecting the tighter bound
    # costs planning horizon; ignoring it costs a robot in traffic.
    t.check("the configured curb range respects the PESSIMISTIC cell-population "
            "limit, not just the optimistic one",
            pess >= cfg["curb_max_range_m"],
            "pessimistic %.2f m >= configured %.2f m (optimistic %.2f m). "
            "Both density models now cover the configured range, so a curb "
            "inside it forms cells under either. Raising the range requires "
            "measuring len(cells) vs range on a recorded SVO2 first."
            % (pess, cfg["curb_max_range_m"], opt))

    # ------------------------------------------------------------------ #
    log.section("9. A wall base is not reported as a curb")
    # ------------------------------------------------------------------ #
    ground = _grid_points(1.0, 5.0, -2.0, 2.0, 0.10, lambda x, y: 0.0)
    pillar = []
    for gx in (3.0, 3.1, 3.2):
        for gy in (-0.1, 0.0, 0.1):
            z = 0.0
            while z <= 1.2:
                pillar.append((gx, gy, z))
                z += 0.05
    res = segment(_noisy(ground + pillar, 0.004, rng), cfg,
                  rng=random.Random(7))
    t.check("the pillar is in the obstacle cloud",
            len([p for p in res["obstacles"] if 2.9 <= p[0] <= 3.3]) > 10)
    t.check("the pillar is NOT reported as a curb step",
            all(s.rng > 3.5 or abs(s.x - 3.1) > 0.5 for s in res["steps"]),
            "%d steps, none at the pillar" % len(res["steps"]))

    # ------------------------------------------------------------------ #
    log.section("10. Degenerate and hostile inputs (must not raise)")
    # ------------------------------------------------------------------ #
    for name, pts in (
            ("empty cloud", []),
            ("single point", [(2.0, 0.0, 0.0)]),
            ("two points", [(2.0, 0.0, 0.0), (3.0, 0.0, 0.0)]),
            ("all collinear", [(1.0 + 0.01 * i, 0.0, 0.0) for i in range(500)]),
            ("all identical", [(2.0, 0.0, 0.0)] * 500),
            ("everything out of ROI", [(50.0, 0.0, 0.0)] * 500),
            ("everything below ROI", [(2.0, 0.0, -5.0)] * 500)):
        try:
            r = segment(pts, cfg, rng=random.Random(8))
            ok = (r["plane"] is not None and isinstance(r["ranges"], list)
                  and isinstance(r["obstacles"], list))
            t.check("%s handled without raising" % name, ok,
                    r["reason"] or "ok")
        except Exception as exc:
            t.check("%s handled without raising" % name, False,
                    "%s: %s" % (exc.__class__.__name__, exc))

    # A vertical-only point set must not yield a plane that passes the gates.
    vert = [(2.0, 0.0, 0.01 * i) for i in range(300)]
    p, inf = fit_ground_plane(vert, rng=random.Random(9))
    t.check("a purely vertical point set falls back to the TF prior",
            inf["fell_back"] and p.tilt_deg < 1e-6,
            "tilt %.4f deg, fell_back=%s" % (p.tilt_deg, inf["fell_back"]))

    # ------------------------------------------------------------------ #
    log.section("11. Scan conversion")
    # ------------------------------------------------------------------ #
    steps = [Step(3.0, 0.0, 3.0, -0.12, "down", 0.0, -0.12),
             Step(2.0, 0.0, 2.0, 0.12, "up", 0.0, 0.12)]
    ranges, kinds = steps_to_scan(steps, angle_min=cfg["scan_angle_min"],
                                  angle_max=cfg["scan_angle_max"],
                                  increment=cfg["scan_angle_increment"],
                                  range_min=cfg["scan_range_min"],
                                  range_max=cfg["scan_range_max"])
    b0 = int(round((0.0 - cfg["scan_angle_min"]) / cfg["scan_angle_increment"]))
    t.close("nearest step wins at a shared bearing", ranges[b0], 2.0, 1e-6)
    t.check("unseen bearings stay at infinity",
            ranges[0] == float("inf") and ranges[-1] == float("inf"))
    t.check("bin count matches angle_min/max/increment",
            len(ranges) == int(math.floor(
                (cfg["scan_angle_max"] - cfg["scan_angle_min"])
                / cfg["scan_angle_increment"])) + 1,
            "%d bins" % len(ranges))
    behind = [Step(-2.0, 0.0, 2.0, -0.12, "down", 0.0, -0.12)]
    rb, _ = steps_to_scan(behind, angle_min=cfg["scan_angle_min"],
                          angle_max=cfg["scan_angle_max"],
                          increment=cfg["scan_angle_increment"])
    t.check("a step behind the robot is outside the scan arc and is dropped",
            all(r == float("inf") for r in rb))

    # ------------------------------------------------------------------ #
    log.section("12. Linear algebra")
    # ------------------------------------------------------------------ #
    # 2a + b + c = 10 ; a + 3b + 2c = 13 ; a = 2  ->  a=2, b=-1, c=7.
    # (Verified by substitution: 4-1+7=10, 2-3+14=13.) The expected values
    # here were wrong on the first run and the SOLVER was right — worth
    # recording, because a hand-written linear solver is exactly the kind of
    # code one is tempted to "fix" until the test agrees with it.
    sol = solve3(((2.0, 1.0, 1.0), (1.0, 3.0, 2.0), (1.0, 0.0, 0.0)),
                 (10.0, 13.0, 2.0))
    t.check("3x3 solve is correct", sol is not None
            and abs(sol[0] - 2.0) < 1e-9 and abs(sol[1] + 1.0) < 1e-9
            and abs(sol[2] - 7.0) < 1e-9, str(sol))
    # Residual check: substitute back rather than trusting the expected triple.
    if sol:
        r = (2 * sol[0] + sol[1] + sol[2] - 10.0,
             sol[0] + 3 * sol[1] + 2 * sol[2] - 13.0,
             sol[0] - 2.0)
        t.check("3x3 solution satisfies the original equations",
                all(abs(v) < 1e-9 for v in r),
                "residuals %.2e %.2e %.2e" % r)
    t.check("a singular system returns None rather than nonsense",
            solve3(((1.0, 2.0, 3.0), (2.0, 4.0, 6.0), (3.0, 6.0, 9.0)),
                   (1.0, 2.0, 3.0)) is None)
    # Partial pivoting: a system whose first pivot is zero.
    sol = solve3(((0.0, 1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
                 (1.0, 2.0, 3.0))
    t.check("partial pivoting handles a zero leading pivot",
            sol is not None and abs(sol[0] - 2.0) < 1e-9
            and abs(sol[1] - 1.0) < 1e-9 and abs(sol[2] - 3.0) < 1e-9,
            str(sol))
    known = fit_plane_ls([(0.0, 0.0, 1.0), (1.0, 0.0, 1.1), (0.0, 1.0, 1.05),
                          (1.0, 1.0, 1.15)])
    t.check("least-squares plane recovers a known tilt",
            known is not None and abs(known.height_of((0.5, 0.5, 1.075))) < 1e-6,
            repr(known))

    # ------------------------------------------------------------------ #
    log.section("13. numpy fast path agrees with the reference (if available)")
    # ------------------------------------------------------------------ #
    try:
        import numpy as _np
    except ImportError:
        _np = None
        log.warn("  SKIP  numpy is not installed on this machine. The pure-"
                 "Python reference above is fully exercised; the vectorised "
                 "twin used at runtime is NOT, and must be checked on the "
                 "Jetson before the node is trusted.")
    if _np is not None:
        pts = _noisy(_grid_points(1.0, 6.0, -2.0, 2.0, 0.10,
                                  lambda x, y: 0.01 * x + 0.005 * y),
                     0.004, random.Random(11))
        arr = _np.asarray(pts, dtype=_np.float64)
        pa, _ = fit_ground_plane(pts, tol=cfg["ransac_inlier_tol_m"],
                                 iterations=120, rng=random.Random(12))
        pb, _ = fit_ground_plane_np(arr, _np, tol=cfg["ransac_inlier_tol_m"],
                                    iterations=120, rng=random.Random(12))
        t.close("numpy and reference agree on plane tilt",
                pa.tilt_deg, pb.tilt_deg, 0.20,
                "%.4f vs %.4f deg" % (pa.tilt_deg, pb.tilt_deg))
        t.close("numpy and reference agree on plane offset",
                pa.offset_at_origin, pb.offset_at_origin, 0.005)
        t.check("both recover the known 0.57 deg synthetic slope",
                abs(pa.tilt_deg - math.degrees(math.atan(
                    math.hypot(0.01, 0.005)))) < 0.3,
                "fitted %.3f deg, true %.3f deg"
                % (pa.tilt_deg,
                   math.degrees(math.atan(math.hypot(0.01, 0.005)))))

    # ------------------------------------------------------------------ #
    log.section("Self-test verdict")
    # ------------------------------------------------------------------ #
    log.metric("selftest_passed", t.passed)
    log.metric("selftest_failed", t.failed)
    if t.failed:
        log.error("%d of %d checks FAILED." % (t.failed, t.passed + t.failed))
        log.summary("Self-test failed: %d of %d checks."
                    % (t.failed, t.passed + t.failed), status="FAIL")
        return EXIT_FAIL
    log.info("All %d checks passed." % t.passed)
    log.info("")
    log.info("WHAT THIS DOES AND DOES NOT PROVE")
    log.info("  Proves: the geometry, the plane fit, the seed gates, the step")
    log.info("  classifier and the scan conversion behave as specified on")
    log.info("  synthetic data with clean, independent, Gaussian noise.")
    log.info("  Does NOT prove: anything about real stereo depth. Real ZED")
    log.info("  noise on low-texture wet pavement is correlated in blobs, not")
    log.info("  independent per point, and no synthetic test can stand in for")
    log.info("  that. Every threshold here remains a reasoned starting value")
    log.info("  until it is measured against a recorded SVO of a real curb.")
    log.summary("Self-test passed: %d checks on synthetic data. NOT validated "
                "against hardware." % t.passed, status="OK")
    return EXIT_PASS


# --------------------------------------------------------------------------- #
# Entry point
# --------------------------------------------------------------------------- #

def strip_ros_args(argv):
    """Drop roslaunch's injected __name:= / __log:= and any _param:= args."""
    return [a for a in argv if not a.startswith("__") and ":=" not in a]


def build_parser():
    p = argparse.ArgumentParser(
        prog="obstacle_segmenter.py",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description=(
            "Segment the ZED X point cloud into a ground-removed obstacle "
            "cloud and a curb/edge channel for the navigation costmap."),
        epilog=(
            "MODES\n"
            "  (no flags)    run as a ROS node (needs ROS, numpy, TF and the "
            "camera)\n"
            "  --self-test   run the synthetic correctness suite; needs "
            "nothing but Python\n"
            "  --geometry    print the mount geometry derivation and the "
            "noise tables\n\n"
            "PUBLISHES\n"
            "  /sidewalk_perception/obstacles_cloud   PointCloud2, base_link\n"
            "  /sidewalk_perception/curb_scan         LaserScan,   base_link\n"
            "  /sidewalk_perception/curb_cloud        PointCloud2  (debug)\n"
            "  /sidewalk_perception/ground_cloud      PointCloud2  (debug)\n\n"
            "The first two names and types are dictated by "
            "sidewalk_navigation/config/costmap_common.yaml. Do not rename "
            "them without changing that file in the same commit."))
    p.add_argument("--self-test", action="store_true",
                   help="run the synthetic correctness suite and exit "
                        "(no ROS, no numpy, no camera required)")
    p.add_argument("--geometry", action="store_true",
                   help="print the derivation of every geometric number this "
                        "node depends on, and exit")
    p.add_argument("--config", default=str(DEFAULT_CONFIG),
                   help="YAML config (default: %(default)s)")
    p.add_argument("--cloud-topic", default=None,
                   help="override the input point cloud topic")
    p.add_argument("--curb-max-range", type=float, default=None,
                   help="override curb_max_range_m, the depth-noise limit "
                        "beyond which curb detection is not trustworthy")
    p.add_argument("--no-debug-clouds", action="store_true",
                   help="do not publish the curb/ground debug clouds")
    p.add_argument("--run-name", default="obstacle_segmenter",
                   help="name recorded in the run log and RUNLOG.md")
    return p


def load_config(path, log=None):
    """Merge the YAML config over the built-in defaults.

    Unknown keys are reported rather than ignored: a typo in a YAML key would
    otherwise leave the default silently in force, which is exactly the kind of
    "I changed it and nothing happened" bug that costs an afternoon.
    """
    cfg = dict(DEFAULTS)
    if not path or not os.path.exists(path):
        if log:
            log.warn("config file %s not found; using built-in defaults. "
                     "Every threshold is documented in that file, so it is "
                     "worth finding out why it is missing." % path)
        return cfg
    try:
        raw = load_simple_yaml(path) or {}
    except Exception as exc:
        if log:
            log.warn("could not parse %s (%s: %s); using built-in defaults."
                     % (path, exc.__class__.__name__, exc))
        return cfg
    flat = {}
    for k, v in raw.items():
        if isinstance(v, dict):
            flat.update(v)
        else:
            flat[k] = v
    unknown = []
    for k, v in flat.items():
        if k in cfg:
            if v is not None:
                cfg[k] = v
        else:
            unknown.append(k)
    if unknown and log:
        log.warn("ignoring %d unknown key(s) in %s: %s — check for typos, a "
                 "misspelled key does NOT change behaviour."
                 % (len(unknown), path, ", ".join(sorted(unknown))))
    return cfg


def main(argv=None):
    argv = strip_ros_args(sys.argv[1:] if argv is None else argv)
    args = build_parser().parse_args(argv)

    # Update the module-level MOUNT in place rather than rebinding it. The
    # geometry helpers default their `h_m` argument to MOUNT["cam_height_m"]
    # at CALL time, so mutating the dict makes every derived number follow the
    # measured mount automatically. Rebinding the name would work too but is
    # easy to get subtly wrong, and this needs no `global`.
    mount = load_mount_geometry()
    MOUNT.update(mount)

    # --geometry must work with no logger, no ROS and no filesystem writes, so
    # it deliberately bypasses RunLogger and prints to stdout.
    if args.geometry:
        cfg = load_config(args.config)
        print(geometry_report(mount, cfg))
        return EXIT_PASS

    with RunLogger("sidewalk_perception", run_name=args.run_name) as log:
        cfg = load_config(args.config, log)
        if args.cloud_topic:
            cfg["cloud_topic"] = args.cloud_topic
        if args.curb_max_range is not None:
            cfg["curb_max_range_m"] = args.curb_max_range
        if args.no_debug_clouds:
            cfg["publish_debug_clouds"] = False

        if args.self_test:
            log.info("Self-test: synthetic point data, pure Python stdlib. "
                     "No ROS, no numpy, no camera.")
            log.info("Mount geometry source: %s" % mount.get("source"))
            try:
                return self_test(log, cfg, mount)
            except KeyboardInterrupt:
                log.warn("Interrupted.")
                log.summary("Self-test interrupted", status="WARN")
                return EXIT_INTERRUPTED

        node = ObstacleSegmenter(cfg, mount, log)
        try:
            return node.run()
        except Unavailable as exc:
            return die(log, exc)
        except KeyboardInterrupt:
            log.warn("Interrupted before the node started.")
            log.summary("Interrupted by operator", status="WARN")
            return EXIT_INTERRUPTED


if __name__ == "__main__":
    sys.exit(main())
