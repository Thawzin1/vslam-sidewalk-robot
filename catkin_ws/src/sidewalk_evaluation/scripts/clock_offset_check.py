#!/usr/bin/env python3
"""clock_offset_check.py - do the robot and the Jetson agree what time it is?

WHY THIS EXISTS

    Run 8 wants to measure the camera's left-right mounting angle by comparing
    two independent accounts of the same motion: what the WHEELS say the robot
    did, and what the CAMERA says it did. The difference between them is the
    angle.

    Plain terms: drive straight two metres. The wheels say "forward 2.00 m,
    sideways 0.00 m". The camera says "forward 2.00 m, sideways 0.10 m". The
    robot did not go sideways, so that 0.10 m is the camera looking off-axis,
    and the angle is atan(0.10 / 2.00) = about 2.9 degrees.

    That subtraction is only meaningful if both accounts refer to the SAME
    MOMENT. The wheels live on the robot's computer and its ROS master; the
    camera lives on the Jetson and a different master. They are two clocks.

    If the clocks are 0.5 s apart and the robot drives at 0.5 m/s, the
    comparison is offset by 0.25 m of real travel - which would be read as a
    7 degree camera yaw that does not exist. The measurement would be confident
    and wrong, which is the worst kind.

WHY IT MEASURES MESSAGE STAMPS AND NOT `date`

    Comparing `date` over ssh measures the clocks plus the round trip, and it
    does not test the thing that actually matters: the timestamp written INTO
    each ROS message, which is what any fusion or comparison will use. A
    machine can have a correct system clock and still stamp messages from a
    different time source. So this subscribes to the robot's real topics and
    compares each message's own stamp against the Jetson's clock at the moment
    it arrived.

HOW TO READ THE RESULT

    A small POSITIVE offset is normal and healthy - that is transport delay,
    the time the message spent travelling. Expect a few milliseconds on the
    wired link.

    A NEGATIVE offset means the robot's clock is AHEAD of the Jetson's: the
    message claims to have been created in the Jetson's future. Nothing can be
    fused across that.

    A LARGE offset either way means the clocks are not disciplined to the same
    source, and the fix is NTP on both, not arithmetic afterwards.

    usage:
      rosrun sidewalk_evaluation clock_offset_check.py
      rosrun sidewalk_evaluation clock_offset_check.py _seconds:=30
"""
from __future__ import print_function

import collections
import os
import sys

ROBOT_MASTER = os.environ.get("ROBOT_MASTER_URI", "http://%s:11311" % os.environ.get("ROBOT_USB_ADDR", "robot"))
ROBOT_IP = os.environ.get("ROS_IP", "<jetson-usb-address>")

# How far apart the clocks may be before the yaw measurement is worthless.
# At 0.5 m/s, 20 ms of clock error is 10 mm of travel - well under the ~100 mm
# sideways drift a 3 degree yaw produces over 2 m, so 20 ms is comfortable.
GOOD_S = 0.020
POOR_S = 0.100


def main():
    os.environ["ROS_MASTER_URI"] = ROBOT_MASTER
    os.environ.setdefault("ROS_IP", ROBOT_IP)
    import rospy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, JointState

    rospy.init_node("clock_offset_check", anonymous=True, disable_signals=True)
    secs = float(rospy.get_param("~seconds", 20.0))

    print("  robot master: %s" % ROBOT_MASTER)
    print("  listening %.0f s ..." % secs)

    # Do NOT hard-code one topic name. This project has already lost time to a
    # config subscribing to /odom, which does not exist on this robot. Try the
    # plausible ones and report which actually delivered.
    CANDIDATES = [
        ("/odometry/filtered", Odometry),
        ("/husky_velocity_controller/odom", Odometry),
        ("/odom", Odometry),
        ("/imu/data", Imu),
        ("/imu_um7/data", Imu),
        ("/imu/data_raw", Imu),
        ("/joint_states", JointState),
    ]

    seen = collections.defaultdict(list)

    def make(name):
        def cb(msg):
            st = msg.header.stamp.to_sec() if hasattr(msg, "header") else 0.0
            if st > 0:
                seen[name].append(rospy.Time.now().to_sec() - st)
        return cb

    for topic, typ in CANDIDATES:
        rospy.Subscriber(topic, typ, make(topic))

    t0 = rospy.Time.now()
    while (rospy.Time.now() - t0).to_sec() < secs:
        rospy.sleep(0.2)

    if not seen:
        print()
        print("  NOTHING ARRIVED. Either the robot is off, or none of these")
        print("  topics exists on it:")
        for t, _ in CANDIDATES:
            print("    %s" % t)
        print("  Check with:  ROS_MASTER_URI=%s rostopic list" % ROBOT_MASTER)
        sys.exit(2)

    print()
    print("  %-38s %6s %12s %10s" % ("topic", "msgs", "median", "spread"))
    worst = 0.0
    for name in sorted(seen):
        v = sorted(seen[name])
        med = v[len(v) // 2]
        print("  %-38s %6d %+11.4f s %8.4f s"
              % (name, len(v), med, v[-1] - v[0]))
        worst = max(worst, abs(med))

    print()
    if worst <= GOOD_S:
        print("  GOOD. Worst offset %.4f s, under %.3f s." % (worst, GOOD_S))
        print("  At 0.5 m/s that is %.0f mm of travel - small against the"
              % (worst * 500))
        print("  ~100 mm sideways drift a 3 degree camera yaw makes over 2 m.")
        print("  Comparing the wheels against the camera is meaningful.")
        sys.exit(0)

    if worst <= POOR_S:
        print("  MARGINAL. Worst offset %.4f s = %.0f mm of travel at 0.5 m/s."
              % (worst, worst * 500))
        print("  A yaw measurement is still possible but drive SLOWLY - the")
        print("  error this causes is proportional to speed.")
        sys.exit(0)

    print("  TOO FAR APART. Worst offset %.4f s = %.0f mm at 0.5 m/s, which is"
          % (worst, worst * 500))
    print("  the same size as the effect being measured. A yaw computed across")
    print("  this gap would be confident and wrong.")
    print()
    print("  Fix the clocks, do not correct for it afterwards:")
    print("    timedatectl                     # on BOTH machines")
    print("    sudo timedatectl set-ntp true   # if NTP is inactive")
    sys.exit(1)


if __name__ == "__main__":
    main()
