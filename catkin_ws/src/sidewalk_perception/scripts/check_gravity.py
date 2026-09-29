#!/usr/bin/env python3
"""check_gravity.py - decide WHY the floor plane was tilted, using gravity.

THE POINT
    measure_camera_tilt.py fits the floor and reports the angle the model is
    missing. It cannot say WHERE that angle comes from, because base_footprint
    is bolted to the robot body, not to the world. Three different physical
    situations produce the identical plane fit and demand opposite fixes.

    This script answers it with different physics. The ZED X has an
    accelerometer; at rest it measures gravity, which is an absolute vertical
    reference owed to nothing in the transform tree. Rotate that measured
    vector into base_link USING THE MODEL and the residual tilt says which
    situation is real:

        ~0.0 deg  camera really is tilted in its mount, robot level
                  -> the applied cam_pitch is correct
        ~3.2 deg  camera and body are both level
                  -> the tilt came from the floor patch; cam_pitch is an ERROR
        ~6.5 deg  camera level in its mount, robot body pitched on its wheels
                  -> cam_pitch is wrong, and the body attitude is the story

    Nothing here reads the depth image, so it cannot inherit the plane fit's
    degeneracy.

WHAT IT ASSUMES
    The robot is STATIONARY. Any real acceleration adds to gravity and is
    indistinguishable from a tilt. The check below rejects the sample if the
    magnitude strays from 9.81 or if the reading is not steady.

    It also trusts the factory IMU->camera extrinsic that the ZED SDK
    publishes. That is a manufacturer calibration, not something measured here.

REFERENCE READING - 2026-08-29, robot parked on lab floor, 600 samples
    mean accel  [-0.4094, -0.1454, +9.7987]  |a| = 9.8083, per-axis sd ~0.025
    in base_link [+0.1538, -0.2342, +9.8043]
    RESIDUAL PITCH -0.90 deg   RESIDUAL ROLL -1.37 deg
    => camera genuinely tilted in its mount; cam_pitch 0.0565 rad is correct.

    The ~0.9 deg pitch and ~1.6 deg roll disagreement against the plane fit
    (which read -0.03 and +0.24) is NOT resolved. It is within the combined
    accelerometer bias, the factory IMU extrinsic, and a plausibly ~1 deg lab
    floor. Do not average the two methods to make it go away - they answer
    different questions. The plane fit gives the camera-to-base_link extrinsic;
    this gives the cause. Only the second was in doubt.
"""
import math

import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import Imu


def quat_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def main():
    rospy.init_node("check_gravity", anonymous=True, disable_signals=True)
    topic = rospy.get_param("~imu", "/zedx_front/zed_node/imu/data")
    target = rospy.get_param("~frame", "base_link")
    n = int(rospy.get_param("~samples", 400))

    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    rospy.sleep(1.5)

    acc, frame = [], None
    for _ in range(n):
        m = rospy.wait_for_message(topic, Imu, timeout=15)
        frame = m.header.frame_id
        acc.append([m.linear_acceleration.x,
                    m.linear_acceleration.y,
                    m.linear_acceleration.z])
    a = np.array(acc)

    mean = a.mean(axis=0)
    sd = a.std(axis=0)
    mag = np.linalg.norm(mean)

    print("  IMU frame        : %s" % frame)
    print("  samples          : %d" % len(a))
    print("  mean accel (m/s2): [%+.4f, %+.4f, %+.4f]  |a| = %.4f" %
          (mean[0], mean[1], mean[2], mag))
    print("  per-axis stddev  : [%.4f, %.4f, %.4f]" % (sd[0], sd[1], sd[2]))
    if abs(mag - 9.81) > 0.15:
        print("  !! |a| is %.3f, not ~9.81 - the robot is NOT at rest, or the" % mag)
        print("     accelerometer is unscaled. RESULT NOT TRUSTWORTHY.")
    if sd.max() > 0.35:
        print("  !! reading is noisy/moving (max stddev %.3f). Re-run at rest." % sd.max())
    print()

    tr = buf.lookup_transform(target, frame, rospy.Time(0), rospy.Duration(5.0))
    R = quat_to_matrix(tr.transform.rotation)
    g = R.dot(mean)            # gravity expressed in base_link, VIA THE MODEL

    print("  gravity rotated into %s using the CURRENT model:" % target)
    print("      [%+.4f, %+.4f, %+.4f]" % (g[0], g[1], g[2]))
    print("      (a perfect model on a level robot gives [0, 0, %+.2f] - note the" % abs(mag))
    print("       sign: an accelerometer at rest measures specific force, i.e. UP)")
    print()

    # SIGN CONVENTION - THE EASY MISTAKE, AND IT WAS MADE HERE FIRST.
    # An accelerometer at rest does NOT report the gravity vector. It reports
    # SPECIFIC FORCE: the normal force holding it up. So a level sensor reads
    # +9.81 on its z axis, pointing UP, not -9.81 pointing down. Treating it as
    # "down" flips every angle by 180 deg, which is exactly what this script did
    # on its first run.
    up = g / np.linalg.norm(g)

    # With base_link rotated by Ry(theta) from the world (+theta = nose down),
    # the world's up vector (0,0,1) expressed in base_link is (-sin t, 0, cos t).
    # So a POSITIVE x component of the measured up means theta is NEGATIVE:
    # the nose is pitched UP relative to true vertical.
    pitch = math.degrees(math.atan2(-up[0], up[2]))
    roll = math.degrees(math.atan2(up[1], up[2]))

    print("  RESIDUAL PITCH vs gravity : %+.2f deg" % pitch)
    print("  RESIDUAL ROLL  vs gravity : %+.2f deg" % roll)
    print()
    print("  ---- reading this ----")
    ap = abs(pitch)
    if ap < 1.2:
        v = ("~0 deg: the camera IS genuinely tilted in its mount and the robot "
             "is level. The applied cam_pitch of 3.24 deg is CORRECT.")
    elif 2.2 < ap < 4.4:
        v = ("~3.2 deg: camera and body are both level - the 3.24 deg came from "
             "the FLOOR PATCH, and the applied cam_pitch is an ERROR. Revert it.")
    elif 5.4 < ap < 7.6:
        v = ("~6.5 deg: the camera is level in its mount and the ROBOT BODY is "
             "pitched on its wheels. cam_pitch is wrong. Investigate tyres/load.")
    else:
        v = ("%.2f deg matches none of the three predictions. Do not act on this "
             "until it is understood." % ap)
    print("  " + v)


if __name__ == "__main__":
    main()
