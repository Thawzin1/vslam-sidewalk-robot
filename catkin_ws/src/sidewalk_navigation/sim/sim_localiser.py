#!/usr/bin/env python3
"""sim_localiser.py - SIM ONLY: stands in for RTAB-Map's localisation (map -> odom).

RTAB-Map in localisation mode keeps odom -> base_link (the blend) untouched and publishes a correction
map -> odom that changes in steps, each time it recognises a place. This stand-in does the same with the
simulator's true pose: every PERIOD s (default 2 s, roughly the real rate of fixes on drive 10's map: 577 fixes
over 246 m at ~0.3 m/s) it sets map -> odom = true(map -> base_link) * inverse(odom -> base_link), optionally
with Gaussian noise (NOISE_XY m, NOISE_YAW deg) to mimic an imperfect fix, and republishes it at 20 Hz between
steps. Before the first step it publishes identity (the robot starts on the start mark: KIDNAP_PRIOR 0 0 0).

Why: the simulated Husky's wheels slip when it turns on the spot (dry run 2: 1.2 m of true travel during a
"turn on the spot"), so wheel odometry alone drifts away from the truth; the real run has RTAB-Map to pull it
back, and the dry run should too.
usage: sim_localiser.py [period_s] [noise_xy_m] [noise_yaw_deg]
"""
import math
import random
import sys

import rospy
import tf2_ros
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TransformStamped

PERIOD = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
NXY = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
NYAW = math.radians(float(sys.argv[3])) if len(sys.argv) > 3 else 0.0


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


rospy.init_node("sim_localiser")
buf = tf2_ros.Buffer()
tf2_ros.TransformListener(buf)
br = tf2_ros.TransformBroadcaster()
truth = {}


def on_ms(m):
    if "husky" in m.name:
        p = m.pose[m.name.index("husky")]
        truth["x"], truth["y"], truth["yaw"] = p.position.x, p.position.y, yaw_of(p.orientation)


rospy.Subscriber("/gazebo/model_states", ModelStates, on_ms, queue_size=1)
corr = [0.0, 0.0, 0.0]
last_fix = rospy.Time(0)
rate = rospy.Rate(20)
while not rospy.is_shutdown():
    now = rospy.Time.now()
    if truth and (now - last_fix).to_sec() >= PERIOD:
        try:
            o = buf.lookup_transform("odom", "base_link", rospy.Time(0), rospy.Duration(0.1)).transform
            ox, oy, oyaw = o.translation.x, o.translation.y, yaw_of(o.rotation)
            tx = truth["x"] + random.gauss(0, NXY)
            ty = truth["y"] + random.gauss(0, NXY)
            tyaw = truth["yaw"] + random.gauss(0, NYAW)
            cyaw = tyaw - oyaw
            c, s = math.cos(cyaw), math.sin(cyaw)
            corr = [tx - (c * ox - s * oy), ty - (s * ox + c * oy), cyaw]
            last_fix = now
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException):
            pass
    t = TransformStamped()
    t.header.stamp = now + rospy.Duration(0.1)   # post-dated like RTAB-Map's map -> odom (tf_tolerance 0.100,
    t.header.frame_id = "map"                    # printed in every drive's mapping.log)
    t.child_frame_id = "odom"
    t.transform.translation.x, t.transform.translation.y = corr[0], corr[1]
    t.transform.rotation.z, t.transform.rotation.w = math.sin(corr[2] / 2), math.cos(corr[2] / 2)
    br.sendTransform(t)
    try:
        rate.sleep()
    except rospy.ROSInterruptException:
        break
