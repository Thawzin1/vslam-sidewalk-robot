#!/usr/bin/env python3
"""
make_orbslam3_config.py — write an ORB-SLAM3 settings file from LIVE camera
calibration, so the intrinsics can never be stale.

WHY THIS EXISTS
    ORB-SLAM3 needs fx, fy, cx, cy, the image size and `Camera.bf` (= fx times
    the stereo baseline in metres). It is entirely normal to copy those numbers
    out of a datasheet once and forget about them. That is a trap.

    * The numbers belong to the individual camera, not to the model. Two ZED X
      units off the same production line differ.
    * They change with resolution. Switching from 1920x1200 to 1280x720 halves
      neither fx nor cx by the same factor unless you also know how the SDK
      cropped and scaled.
    * They change after an SDK update or a factory recalibration.

    And when they are wrong, nothing crashes. The reconstructed trajectory comes
    out smooth, self-consistent, and scaled by a constant error. You only find
    out when you compare it against ground truth, by which point the recording
    session is over.

    So this tool refuses to invent anything. It reads sensor_msgs/CameraInfo -
    live from the running camera, from a recorded bag, or from a JSON dump made
    earlier - and fills in a reviewed template. If it cannot obtain real
    calibration, it exits non-zero and says exactly what was missing.

WHAT IT CROSS-CHECKS
    The stereo baseline is recovered from the RIGHT camera's projection matrix
    (P[3] = -fx * Tx) and compared against the ZED X nominal 120 mm. A
    disagreement of more than a couple of percent means the wrong topic was
    used, the images are not rectified, or the calibration is corrupt - all of
    which are worth stopping for.

SOURCES (choose exactly one)
    --from-topic   subscribe to live camera_info topics (needs a running camera)
    --from-bag     read the first camera_info of each side from a rosbag
    --from-json    read a dump previously written with --dump-json

USAGE
    rosrun sidewalk_slam make_orbslam3_config.py \
        --from-topic --camera-name zedx_front \
        --out ~/orbslam3_zedx.yaml --dump-json ~/zedx_calib.json

    python3 make_orbslam3_config.py --from-json ~/zedx_calib.json \
        --out /tmp/orbslam3_zedx.yaml            # works with no ROS at all
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from slam_common import (SlamExit, ZEDX_IMU_RATE_HZ, ZEDX_NOMINAL_BASELINE_M,  # noqa: E402
                         die, make_logger, probe_ros)

PKG_DIR = Path(__file__).resolve().parent.parent
TEMPLATE_STEREO = PKG_DIR / "config" / "orbslam3_zedx_stereo.template.yaml"
TEMPLATE_INERTIAL = PKG_DIR / "config" / "orbslam3_zedx_stereo_inertial.template.yaml"

# Conservative consumer-MEMS defaults. Explicitly ESTIMATES - see the template.
#
# THE DATASHEET DOES PUBLISH TWO OF THESE, AND THIS PROJECT SPENT MONTHS SAYING
# Specifications" / "Motion Sensors" table lists, directly beneath the
# resolution figures that were being cited:
#
#     Accelerometer Noise Density   2.3 mg
#     Gyroscope Noise Density       0.20 dps
#     Sensitivity Error             +/- 0.5%
#
# They are carried in perception_common.ZEDX as imu_accel_noise_density_mg,
# imu_gyro_noise_density_dps and imu_sensitivity_error_pct.
#
# WHY THE VALUES BELOW ARE NEVERTHELESS UNCHANGED. The datasheet writes those
# two figures with NO per-root-hertz denominator — "2.3 mg", not
# "2.3 mg/sqrt(Hz)" — while ORB-SLAM3's IMU.NoiseAcc and IMU.NoiseGyro are
# strictly per-root-hertz densities (Tracking.cc prints their units as
# "m/s^2/sqrt(Hz)" and "rad/s/sqrt(Hz)").
#
# THE ARITHMETIC, BOTH SENSORS, NO ROUNDING IN OUR FAVOUR. Conversions:
# 1 mg = 9.80665e-3 m/s^2, 1 dps = pi/180 = 1.745329e-2 rad/s.
#
#   Read literally, as densities:
#       accel  2.3 mg   -> 2.2555e-2 m/s^2/sqrt(Hz)  = 11.3x IMU_NOISE_ACC
#       gyro   0.20 dps -> 3.4907e-3 rad/s/sqrt(Hz)  = 20.5x IMU_NOISE_GYRO
#   Both implausible for a part the same datasheet calls "ultra low noise".
#
#   Read as total RMS over the sensor bandwidth, divided by sqrt(BW). The
#   datasheet states no bandwidth; assume the Nyquist of the 200 Hz output
#   rate, BW = 100 Hz, sqrt(BW) = 10:
#       accel  2.2555e-3 m/s^2/sqrt(Hz)  = 1.13x IMU_NOISE_ACC (2.0e-3)
#       gyro   3.4907e-4 rad/s/sqrt(Hz)  = 2.05x IMU_NOISE_GYRO (1.7e-4)
#
# THAT SECOND READING IS "1.13x AND 2.05x", NOT "very close to these
# note in orbslam3_zedx_stereo_inertial.template.yaml said "within a factor of
# ~1.2" — both of which describe the ACCELEROMETER and quietly extend it to
# the gyroscope, which is off by more than a factor of two. One flattering
# number covering two sensors is the same failure the depth-percentage
# correction is about.
#
# Choosing a different bandwidth does not fix it. Both figures divide by the
# same sqrt(BW), so the gyro stays 20.5/11.3 = 1.82x further out than the
# accelerometer at every bandwidth. Matching the accelerometer exactly needs
# BW = 127 Hz; matching the gyroscope exactly needs BW = 422 Hz.
#
# The document does not say which reading is meant.
#
# Substituting a number whose unit is ambiguous, into a field where the unit is
# not, would be a worse error than the honest estimate it replaced — and it
# would be a TUNING CHANGE landing in the same commit as a datasheet
# correction. So: not applied, flagged here, and the two candidate conversions
# are written out above so whoever settles the unit does not have to redo the
# arithmetic. What settles it: Stereolabs support, or the Allan-variance run,
# which is still worth doing because it ALSO yields the bias-instability and
# random-walk terms (IMU_GYRO_WALK, IMU_ACC_WALK) that the datasheet genuinely
# does not publish. That run is now an improvement, not a prerequisite.
#
# RECOMMENDED, NOT APPLIED. If the bandwidth reading is confirmed, the value
# that moves is IMU_NOISE_GYRO: 1.7e-4 -> 3.5e-4, because the manufacturer's
# implied gyroscope noise is roughly twice what this file assumes, and the
# current assumption is the optimistic one (the optimiser is told the
# gyroscope is quieter than its maker claims). IMU_NOISE_ACC would barely
# move, 2.0e-3 -> 2.26e-3. Neither is edited here: they are tuning values,
# "one variable per experiment" is a standing rule, and both depend on a unit
# that is still unsettled.
IMU_DEFAULTS = {
    "IMU_NOISE_GYRO": 1.7e-4,   # rad/s/sqrt(Hz)
    "IMU_NOISE_ACC": 2.0e-3,    # m/s^2/sqrt(Hz)
    "IMU_GYRO_WALK": 1.9e-5,    # rad/s^2/sqrt(Hz)
    "IMU_ACC_WALK": 3.0e-3,     # m/s^3/sqrt(Hz)
}

# Key renames for ORB-SLAM3 master's "File.version 1.0" schema. The known-good
# configuration on this Jetson (~/oak-d-params.yaml) uses the flat legacy names,
# so legacy is the default; this mapping exists for the day the ORB-SLAM3
# checkout is updated.
V1_RENAMES = {
    "Camera.fx": "Camera1.fx",
    "Camera.fy": "Camera1.fy",
    "Camera.cx": "Camera1.cx",
    "Camera.cy": "Camera1.cy",
    "Camera.k1": "Camera1.k1",
    "Camera.k2": "Camera1.k2",
    "Camera.p1": "Camera1.p1",
    "Camera.p2": "Camera1.p2",
    "Camera.bf": "Stereo.b",           # NOTE: v1 wants the baseline, not fx*b
    "ThDepth": "Stereo.ThDepth",
}


# --------------------------------------------------------------------------- #
# Argument handling
# --------------------------------------------------------------------------- #

def parse_args(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)

    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--from-topic", action="store_true",
                     help="subscribe to live camera_info (requires ROS + camera)")
    src.add_argument("--from-bag", metavar="BAG",
                     help="read camera_info from a rosbag (requires ROS python)")
    src.add_argument("--from-json", metavar="JSON",
                     help="read a calibration dump made earlier (no ROS needed)")

    ap.add_argument("--camera-name", default="zedx_front",
                    help="camera namespace, e.g. zedx_front / zedx_rear. Kept "
                         "per-camera so the second ZED X does not collide. "
                         "(default: zedx_front)")
    ap.add_argument("--node-name", default="zed_node",
                    help="ZED wrapper node name (default: zed_node)")
    ap.add_argument("--left-info", default=None,
                    help="override the left camera_info topic entirely")
    ap.add_argument("--right-info", default=None,
                    help="override the right camera_info topic entirely")

    ap.add_argument("--out", required=True, help="path of the settings file to write")
    ap.add_argument("--dump-json", default=None,
                    help="also write the raw calibration as JSON, so this can "
                         "be regenerated offline without the camera")

    ap.add_argument("--imu", action="store_true",
                    help="emit a stereo-INERTIAL configuration (includes the "
                         "IMU block and the camera-IMU extrinsic)")
    ap.add_argument("--imu-frame", default=None,
                    help="TF frame of the IMU (default: <camera>_imu_link)")
    ap.add_argument("--camera-frame", default=None,
                    help="TF frame of the left camera optical centre "
                         "(default: <camera>_left_camera_optical_frame)")

    ap.add_argument("--fps", type=float, default=None,
                    help="frame rate to declare. If omitted in --from-topic "
                         "mode it is MEASURED from message arrival times.")
    ap.add_argument("--features", type=int, default=1200,
                    help="ORBextractor.nFeatures (default: 1200)")
    ap.add_argument("--th-depth", type=float, default=37.5,
                    help="close/far threshold in MULTIPLES OF THE BASELINE. "
                         "37.5 mirrors the previous OAK-D configuration. "
                         "(default: 37.5)")
    ap.add_argument("--th-depth-metres", type=float, default=None,
                    help="set the close/far threshold in metres instead; "
                         "converted to baseline multiples for you")
    ap.add_argument("--schema", choices=("legacy", "v1"), default="legacy",
                    help="legacy = flat Camera.* keys, matching the working "
                         "~/oak-d-params.yaml on this Jetson. v1 = the "
                         "Camera1.*/File.version schema used by ORB-SLAM3 "
                         "master. (default: legacy)")
    ap.add_argument("--timeout", type=float, default=15.0,
                    help="seconds to wait for camera_info in --from-topic mode")
    ap.add_argument("--baseline-tolerance", type=float, default=0.02,
                    help="fractional disagreement with the nominal 120 mm "
                         "baseline that is tolerated before erroring "
                         "(default: 0.02 = 2%%)")
    return ap.parse_args(argv)


def info_topics(args):
    base = f"/{args.camera_name}/{args.node_name}"
    left = args.left_info or f"{base}/left/camera_info"
    right = args.right_info or f"{base}/right/camera_info"
    return left, right


# --------------------------------------------------------------------------- #
# Calibration acquisition
# --------------------------------------------------------------------------- #

def _camera_info_to_dict(msg):
    """Reduce a sensor_msgs/CameraInfo to the plain fields we care about."""
    return {
        "width": int(msg.width),
        "height": int(msg.height),
        "frame_id": msg.header.frame_id,
        "distortion_model": msg.distortion_model,
        "D": [float(x) for x in msg.D],
        "K": [float(x) for x in msg.K],
        "P": [float(x) for x in msg.P],
        "R": [float(x) for x in msg.R],
    }


def from_topic(log, args):
    """Subscribe to both camera_info topics and grab one message from each."""
    ros = probe_ros()
    if not ros["present"]:
        die(log, "--from-topic needs a sourced ROS environment; none was found.",
            f"{ros['detail']}. This authoring machine has no ROS. Run this on "
            f"the Jetson, or capture the calibration once with --dump-json and "
            f"regenerate offline with --from-json.")
    try:
        import rospy
        from sensor_msgs.msg import CameraInfo
    except ImportError as exc:
        die(log, f"ROS python packages are not importable: {exc}",
            "Source the ROS setup file and the catkin workspace, then retry.")

    left_topic, right_topic = info_topics(args)
    log.info(f"waiting up to {args.timeout:.0f}s for {left_topic}")
    log.info(f"waiting up to {args.timeout:.0f}s for {right_topic}")

    rospy.init_node("make_orbslam3_config", anonymous=True, disable_signals=True)

    got = {}
    stamps = []

    def _measure(msg):
        # Message arrival times give us the real frame rate, which is more
        # honest than whatever the config file claims the camera is set to.
        stamps.append(msg.header.stamp.to_sec())

    sub = rospy.Subscriber(left_topic, CameraInfo, _measure, queue_size=50)
    for name, topic in (("left", left_topic), ("right", right_topic)):
        try:
            msg = rospy.wait_for_message(topic, CameraInfo, timeout=args.timeout)
        except Exception:
            sub.unregister()
            die(log, f"No message received on {topic} within {args.timeout:.0f}s.",
                "The camera stack is not publishing. Start it first (Phase 2, "
                "sidewalk_perception), then check with `rostopic hz " + topic +
                "`. If the topic does not exist at all, the ZED wrapper is not "
                "running; if it exists but is silent, zed_x_daemon or the GMSL2 "
                "link is down.")
        got[name] = _camera_info_to_dict(msg)
        log.info(f"{name}: {msg.width}x{msg.height} frame_id={msg.header.frame_id}")

    if args.fps is None and len(stamps) >= 3:
        # Wait a moment longer so there is something to measure.
        deadline = rospy.Time.now().to_sec() + 2.0
        while rospy.Time.now().to_sec() < deadline and len(stamps) < 40:
            rospy.sleep(0.05)
    sub.unregister()

    measured_fps = None
    if len(stamps) >= 3:
        span = stamps[-1] - stamps[0]
        if span > 0:
            measured_fps = (len(stamps) - 1) / span
            log.metric("measured_camera_info_rate", round(measured_fps, 2), "Hz")

    return {"left": got["left"], "right": got["right"],
            "measured_fps": measured_fps,
            "origin": f"live topics {left_topic} / {right_topic}"}


def from_bag(log, args):
    """Pull the first camera_info of each side out of a recording."""
    try:
        import rosbag
    except ImportError as exc:
        die(log, f"the `rosbag` python module is not importable: {exc}",
            "This needs a sourced ROS environment. On a machine without ROS, "
            "use --from-json with a dump captured on the Jetson.")

    path = Path(args.from_bag).expanduser()
    if not path.exists():
        die(log, f"bag file does not exist: {path}",
            "Check the path. Bags recorded by sidewalk_perception live under "
            "the dataset directory recorded in its manifest.")

    left_topic, right_topic = info_topics(args)
    got, stamps = {}, []
    try:
        with rosbag.Bag(str(path), "r") as bag:
            for topic, msg, _t in bag.read_messages(topics=[left_topic, right_topic]):
                if topic == left_topic:
                    stamps.append(msg.header.stamp.to_sec())
                    got.setdefault("left", _camera_info_to_dict(msg))
                elif topic == right_topic:
                    got.setdefault("right", _camera_info_to_dict(msg))
                if len(got) == 2 and len(stamps) > 60:
                    break
    except Exception as exc:
        die(log, f"could not read {path}: {type(exc).__name__}: {exc}",
            "If the message is about an unindexed bag, run `rosbag reindex` "
            "first - that happens when a recording was killed rather than "
            "stopped cleanly.")

    missing = [k for k in ("left", "right") if k not in got]
    if missing:
        die(log, f"the bag contains no camera_info for: {', '.join(missing)}",
            f"Expected topics {left_topic} and {right_topic}. List what the bag "
            f"really contains with `rosbag info {path}` and pass the right "
            f"names with --left-info / --right-info, or --camera-name.")

    measured_fps = None
    if len(stamps) >= 3 and (stamps[-1] - stamps[0]) > 0:
        measured_fps = (len(stamps) - 1) / (stamps[-1] - stamps[0])

    return {"left": got["left"], "right": got["right"],
            "measured_fps": measured_fps, "origin": f"bag {path}"}


def from_json(log, args):
    """Reload a dump. This is the path that works on a machine with no ROS."""
    path = Path(args.from_json).expanduser()
    if not path.exists():
        die(log, f"calibration dump does not exist: {path}",
            "Create one on the Jetson with "
            "`make_orbslam3_config.py --from-topic ... --dump-json <path>`.")
    try:
        data = json.loads(path.read_text())
    except (OSError, ValueError) as exc:
        die(log, f"could not parse {path}: {type(exc).__name__}: {exc}",
            "The file must be a JSON dump written by this script.")
    for side in ("left", "right"):
        if side not in data:
            die(log, f"calibration dump is missing the '{side}' camera.",
                "Re-dump it from a source that has both stereo halves.")
    data.setdefault("measured_fps", None)
    data["origin"] = f"json dump {path}"
    return data


# --------------------------------------------------------------------------- #
# Camera-IMU extrinsic
# --------------------------------------------------------------------------- #

def lookup_tbc(log, args):
    """Look up T_imu_leftcam from the live TF tree.

    Returns (16 floats row-major, description). Falls back to identity with a
    very loud warning, because an identity extrinsic is a physically wrong
    answer that will still run.
    """
    imu_frame = args.imu_frame or f"{args.camera_name}_imu_link"
    cam_frame = args.camera_frame or f"{args.camera_name}_left_camera_optical_frame"

    identity = [1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0]

    if not probe_ros()["present"]:
        log.warn("No ROS environment: cannot read the camera-IMU extrinsic "
                 "from TF. Falling back to IDENTITY.")
        log.warn("IDENTITY IS WRONG. It claims the IMU sits exactly at the "
                 "left lens optical centre with the same axes. On the ZED X it "
                 "does not. Regenerate this file on the Jetson with the camera "
                 "running before trusting any stereo-inertial result.")
        return identity, "IDENTITY FALLBACK - NOT A REAL CALIBRATION"

    try:
        import rospy
        import tf2_ros
    except ImportError as exc:
        log.warn(f"tf2_ros not importable ({exc}); using IDENTITY extrinsic.")
        return identity, "IDENTITY FALLBACK - tf2_ros unavailable"

    if not rospy.core.is_initialized():
        rospy.init_node("make_orbslam3_config_tf", anonymous=True,
                        disable_signals=True)

    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    log.info(f"looking up TF {imu_frame} <- {cam_frame}")
    deadline = rospy.Time.now().to_sec() + max(5.0, args.timeout)
    tr = None
    while rospy.Time.now().to_sec() < deadline:
        try:
            tr = buf.lookup_transform(imu_frame, cam_frame, rospy.Time(0))
            break
        except Exception:
            rospy.sleep(0.1)

    if tr is None:
        log.warn(f"TF {imu_frame} <- {cam_frame} never became available; "
                 f"using IDENTITY extrinsic.")
        log.warn("Check the frame names against `rosrun tf2_tools view_frames.py`. "
                 "The ZED wrapper names its frames from the camera namespace, so "
                 "if you renamed the camera these defaults are wrong.")
        return identity, f"IDENTITY FALLBACK - TF {imu_frame}<-{cam_frame} missing"

    t = tr.transform.translation
    q = tr.transform.rotation
    m = quat_to_matrix(q.x, q.y, q.z, q.w)
    tbc = [m[0][0], m[0][1], m[0][2], t.x,
           m[1][0], m[1][1], m[1][2], t.y,
           m[2][0], m[2][1], m[2][2], t.z,
           0.0, 0.0, 0.0, 1.0]
    lever = math.sqrt(t.x ** 2 + t.y ** 2 + t.z ** 2)
    log.metric("imu_camera_lever_arm", round(lever, 4), "m")
    if lever > 0.30:
        log.warn(f"The IMU-to-camera lever arm reads {lever:.3f} m. On a ZED X "
                 f"that distance is a few centimetres at most - this is almost "
                 f"certainly the wrong pair of frames.")
    return tbc, f"TF lookup {imu_frame} <- {cam_frame}"


def quat_to_matrix(x, y, z, w):
    """Rotation matrix from a scalar-last quaternion. Written out rather than
    pulled from tf.transformations so this file has no ROS import at module
    scope and stays importable on a bare machine."""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    x, y, z, w = x / n, y / n, z / n, w / n
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


# --------------------------------------------------------------------------- #
# Derivation and rendering
# --------------------------------------------------------------------------- #

def derive(log, calib, args):
    """Turn raw camera_info into the numbers ORB-SLAM3 wants, with checks."""
    left, right = calib["left"], calib["right"]

    P = left.get("P") or []
    if len(P) != 12:
        die(log, "the left camera_info has no valid 3x4 projection matrix P.",
            "This usually means the topic is an unrectified/raw camera_info. "
            "Point --left-info at the rectified stream (…/left/camera_info from "
            "the ZED wrapper, which corresponds to image_rect_color).")

    fx, fy, cx, cy = P[0], P[5], P[2], P[6]
    if fx <= 0 or fy <= 0:
        die(log, f"nonsensical focal lengths in P: fx={fx}, fy={fy}.",
            "The camera has not published a real calibration yet. Wait for the "
            "ZED SDK to finish loading the factory calibration and retry.")

    # Rectified stereo pairs must share focal lengths; if they do not, the
    # images are not a rectified pair and every disparity is meaningless.
    Pr = right.get("P") or []
    if len(Pr) != 12:
        die(log, "the right camera_info has no valid projection matrix P.",
            "Same cause as above: check --right-info points at the rectified "
            "right stream.")
    if abs(Pr[0] - fx) / fx > 1e-3:
        die(log, f"left fx ({fx:.3f}) and right fx ({Pr[0]:.3f}) disagree.",
            "A rectified stereo pair shares one focal length by definition. "
            "You are looking at raw, unrectified topics, or at two topics from "
            "different cameras.")

    # P[3] of the RIGHT camera is -fx * Tx, where Tx is the baseline in metres.
    # This is where metric scale physically enters the system.
    baseline = -Pr[3] / fx
    if baseline <= 0:
        die(log, f"recovered a non-positive stereo baseline ({baseline:.4f} m).",
            "The left and right topics are swapped. In ROS the RIGHT camera "
            "carries the negative P[3]; the left one has P[3] = 0.")

    err = abs(baseline - ZEDX_NOMINAL_BASELINE_M) / ZEDX_NOMINAL_BASELINE_M
    log.metric("stereo_baseline", round(baseline, 6), "m")
    log.metric("baseline_vs_nominal_error", round(err * 100.0, 3), "%")
    if err > args.baseline_tolerance:
        die(log,
            f"the recovered baseline is {baseline * 1000:.1f} mm, which differs "
            f"from the ZED X nominal 120 mm by {err * 100:.1f}%.",
            "Either these are not ZED X topics, or the images are not rectified, "
            "or the SDK calibration is corrupt. Re-run "
            "`ZED_Explorer` / the SDK calibration tool before continuing. If you "
            "are deliberately using a different camera, raise "
            "--baseline-tolerance and record why in the run log.")
    if err > 0.005:
        log.warn(f"baseline differs from nominal by {err * 100:.2f}% - within "
                 f"tolerance, but worth noting: this is a per-unit calibration "
                 f"difference and it scales every distance the system reports.")

    width = int(left.get("width") or 0)
    height = int(left.get("height") or 0)
    if width <= 0 or height <= 0:
        die(log, "camera_info reports a zero image size.",
            "The wrapper published a placeholder before the camera opened. "
            "Retry once images are actually flowing.")

    if int(right.get("width") or 0) != width or int(right.get("height") or 0) != height:
        die(log, "left and right images have different sizes "
                 f"({width}x{height} vs {right.get('width')}x{right.get('height')}).",
            "A stereo pair must be the same size. Check the resolution settings "
            "in the camera launch.")

    # Distortion should be ~zero on rectified topics. Non-zero means somebody
    # subscribed to the raw stream, and the zeros hardcoded in the template
    # would then be a lie.
    D = left.get("D") or []
    max_d = max((abs(d) for d in D), default=0.0)
    if max_d > 1e-6:
        log.warn(f"the left camera_info carries non-zero distortion "
                 f"(max |D| = {max_d:.4g}). The template writes zeros because it "
                 f"assumes rectified input. Confirm you are using "
                 f"image_rect_color, not image_raw.")

    fps = args.fps or calib.get("measured_fps") or 30.0

    th_depth = args.th_depth
    if args.th_depth_metres is not None:
        th_depth = args.th_depth_metres / baseline
        log.info(f"--th-depth-metres {args.th_depth_metres} m -> "
                 f"ThDepth {th_depth:.2f} baselines")
    th_depth_m = th_depth * baseline
    log.metric("orbslam3_far_threshold", round(th_depth_m, 3), "m")
    if th_depth_m < 3.0:
        log.warn(f"the close/far threshold works out at only {th_depth_m:.2f} m. "
                 f"On a 120 mm baseline the classic ThDepth of 37.5 (tuned for "
                 f"KITTI's 540 mm baseline) is very short. Consider "
                 f"--th-depth-metres 8 for indoor rooms or 15 outdoors.")

    bf = fx * baseline
    log.metric("camera_bf", round(bf, 4))

    return {
        "FX": f"{fx:.6f}", "FY": f"{fy:.6f}",
        "CX": f"{cx:.6f}", "CY": f"{cy:.6f}",
        "WIDTH": str(width), "HEIGHT": str(height),
        "FPS": f"{fps:.1f}",
        "BF": f"{bf:.6f}",
        "BASELINE_MM": f"{baseline * 1000:.2f}",
        "TH_DEPTH": f"{th_depth:.2f}",
        "TH_DEPTH_M": f"{th_depth_m:.2f}",
        "N_FEATURES": str(args.features),
        # THE ONE VALUE IN THIS DICT THAT IS NOT MEASURED FROM THE LIVE CAMERA.
        #
        # Everything above — FX, FY, CX, CY, BF, the baseline, the image size —
        # is read out of the camera_info this run actually received, which is
        # the whole design of this generator and the reason it refuses to run
        # without one. IMU_FREQ is different: it comes from a constant, and
        # ORB-SLAM3 scales all four inertial noise terms by sqrt of it
        # (Tracking.cc:613). A wrong value here is not caught by anything.
        #
        # See slam_common.ZEDX_IMU_RATE_HZ for the full account of what that
        # mis-scaling does and in which direction.
        #
        # WHAT WOULD BE BETTER, AND IS NOT DONE HERE: measure the achieved rate
        # off the live IMU topic the same way the intrinsics are measured, and
        # write THAT. The datasheet describes the part; ORB-SLAM3 wants the
        # frequency of the stream it is fed, and the wrapper sits in between
        # (zedx_front.yaml sets sensors/max_pub_rate). Adding a measurement
        # pass is a behaviour change to a generator that currently produces
        # identical output for identical input, so it is proposed rather than
        # slipped in. Until then, confirm by hand before a stereo-inertial run:
        #     rostopic hz /zedx_front/zed_node/imu/data
        "IMU_FREQ": f"{ZEDX_IMU_RATE_HZ:.0f}",
        "_baseline": baseline,
        "_fx": fx,
    }


# Lines between these two markers exist to warn a human reading the TEMPLATE.
# They would be confusing in the generated output ("this file is a template" in
# a file that is not), so they are stripped on the way through.
TEMPLATE_ONLY_OPEN = "# <<TEMPLATE-ONLY"
TEMPLATE_ONLY_CLOSE = "# TEMPLATE-ONLY>>"


def strip_template_only(text):
    out, skipping = [], False
    for line in text.splitlines():
        if line.strip().startswith(TEMPLATE_ONLY_OPEN):
            skipping = True
            continue
        if line.strip().startswith(TEMPLATE_ONLY_CLOSE):
            skipping = False
            continue
        if not skipping:
            out.append(line)
    return "\n".join(out)


def render(template_path, values):
    """Substitute double-brace placeholders. Fails loudly on anything left."""
    text = strip_template_only(template_path.read_text())
    for key, val in values.items():
        if key.startswith("_"):
            continue
        text = text.replace("{{" + key + "}}", str(val))
    leftovers = []
    idx = 0
    while True:
        a = text.find("{{", idx)
        if a < 0:
            break
        b = text.find("}}", a)
        if b < 0:
            break
        leftovers.append(text[a + 2:b])
        idx = b + 2
    return text, leftovers


def apply_v1_schema(text):
    """Convert the legacy flat keys to the ORB-SLAM3 v1.0 schema.

    Line-level rename rather than a YAML round-trip: cv::FileStorage YAML is not
    quite YAML (the `%YAML:1.0` directive, `!!opencv-matrix` tags), and a
    general parser mangles it.
    """
    out = ["%YAML:1.0", 'File.version: "1.0"']
    for line in text.splitlines():
        if line.startswith("%YAML"):
            continue
        stripped = line.strip()
        if stripped and not stripped.startswith("#") and ":" in stripped:
            key = stripped.split(":", 1)[0].strip()
            if key in V1_RENAMES:
                new = V1_RENAMES[key]
                line = line.replace(key + ":", new + ":", 1)
        out.append(line)
    return "\n".join(out) + "\n"


def provenance_header(args, calib, values):
    stamp = datetime.now().isoformat(timespec="seconds")
    return "\n".join([
        "#",
        "# GENERATED FILE - do not edit by hand; regenerate instead.",
        f"#   generated : {stamp}",
        f"#   by        : sidewalk_slam/scripts/make_orbslam3_config.py",
        f"#   source    : {calib.get('origin', 'unknown')}",
        f"#   camera    : {args.camera_name}/{args.node_name}",
        f"#   schema    : {args.schema}",
        f"#   baseline  : {values['BASELINE_MM']} mm (measured from right P[3])",
        "#",
        "# Editing this file by hand defeats the entire point: the next person",
        "# to change resolution will regenerate it and silently drop your edit.",
        "# Change the template in sidewalk_slam/config/ instead.",
        "#",
    ])


# --------------------------------------------------------------------------- #

def main(argv=None):
    args = parse_args(argv)
    with make_logger("make_orbslam3_config") as log:
        log.section("Acquiring calibration")
        if args.from_topic:
            calib = from_topic(log, args)
        elif args.from_bag:
            calib = from_bag(log, args)
        else:
            calib = from_json(log, args)
        log.info(f"calibration source: {calib['origin']}")

        if args.dump_json:
            dump = Path(args.dump_json).expanduser()
            dump.parent.mkdir(parents=True, exist_ok=True)
            dump.write_text(json.dumps(
                {k: v for k, v in calib.items() if k != "origin"}, indent=2))
            log.info(f"raw calibration dumped to {dump}")

        log.section("Deriving ORB-SLAM3 parameters")
        values = derive(log, calib, args)

        template = TEMPLATE_STEREO
        if args.imu:
            template = TEMPLATE_INERTIAL
            log.section("Camera-IMU extrinsic")
            tbc, tbc_src = lookup_tbc(log, args)
            values["TBC_DATA"] = ", ".join(f"{v:.9f}" for v in tbc)
            values["TBC_SOURCE"] = tbc_src
            values.update({k: repr(v) for k, v in IMU_DEFAULTS.items()})

        if not template.exists():
            die(log, f"template not found: {template}",
                "The package is incomplete. Check that config/ was installed "
                "alongside scripts/ (CMakeLists.txt installs both).")

        log.section("Writing settings file")
        text, leftovers = render(template, values)
        if leftovers:
            die(log, f"template placeholders were left unfilled: "
                     f"{', '.join(sorted(set(leftovers)))}",
                "This is a bug in make_orbslam3_config.py or a template that "
                "has gained a new token. Do NOT hand-edit the output - ORB-SLAM3 "
                "would fail to parse it anyway.")

        if args.schema == "v1":
            log.warn("Emitting the ORB-SLAM3 v1.0 schema. The known-working "
                     "configuration on this Jetson (~/oak-d-params.yaml) uses "
                     "the legacy flat schema, so v1 output has NOT been "
                     "validated against the installed ORB-SLAM3 build.")
            log.warn("In v1, Stereo.b is the baseline in METRES, not fx*b. The "
                     "rename is handled, but check the value before trusting it.")
            text = apply_v1_schema(text)

            # The rename above turns `Camera.bf` into `Stereo.b`, but the two
            # keys mean DIFFERENT THINGS: bf is fx*baseline, Stereo.b is the
            # baseline in metres. The key rename is not enough; the value must
            # be converted too. This is a plain text substitution that depends
            # on `values['BF']` being formatted exactly as it was rendered into
            # the template, so verify it actually fired. If it ever silently
            # no-ops the file keeps fx*baseline (~1000x too large) under a key
            # that ORB-SLAM3 will happily accept, and every depth in the run is
            # wrong by three orders of magnitude with no error anywhere.
            stale = f"Stereo.b: {values['BF']}"
            if stale not in text:
                die(log, "v1 schema conversion could not locate the Stereo.b "
                         "value to convert from fx*baseline to metres",
                    "This is a bug in make_orbslam3_config.py: the rendered "
                    "`Camera.bf` value no longer matches values['BF'] "
                    "verbatim, so the substitution silently did nothing. "
                    "Refusing to emit a settings file whose baseline would be "
                    "wrong by a factor of fx. Fix the substitution rather than "
                    "hand-editing the output.")
            text = text.replace(stale, f"Stereo.b: {values['_baseline']:.6f}")

        # Insert the provenance block immediately after the %YAML directive,
        # which cv::FileStorage requires to be the very first line.
        lines = text.splitlines()
        body = "\n".join([lines[0], provenance_header(args, calib, values)]
                         + lines[1:]) + "\n"

        out = Path(args.out).expanduser()
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(body)
        log.info(f"wrote {out} ({len(body)} bytes)")
        log.metric("output_bytes", len(body))

        mode = "stereo-inertial" if args.imu else "stereo"
        log.summary(
            f"ORB-SLAM3 {mode} settings generated for {args.camera_name} "
            f"({values['WIDTH']}x{values['HEIGHT']}, baseline "
            f"{values['BASELINE_MM']} mm, fx {values['FX']}) -> {out}",
            status="OK")
        return 0


if __name__ == "__main__":
    # SlamExit is how every tool in this package reports an already-diagnosed
    # refusal. The reason has been printed and logged by the time it reaches
    # here; all that is left is to hand the shell a non-zero status.
    try:
        sys.exit(main())
    except SlamExit as _exit:
        sys.exit(_exit.code)
