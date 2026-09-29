#!/usr/bin/env python3
"""sim_joy.py - SIM ONLY: stands in for the user's thumb on the joystick's autonomy button.

Publishes sensor_msgs/Joy on /joy_teleop/joy 20 times a second (the robot's joy_node rate, teleop_ps4.yaml
autorepeat_rate 20) with button BUTTON (default 7) pressed, unless the file RELEASE exists (then released), and
publishes nothing at all while the file SILENT exists (a joystick that has disconnected).
usage: sim_joy.py <control_dir> [button]
"""
import os
import sys
import time

import rospy
from sensor_msgs.msg import Joy

d = sys.argv[1]
btn = int(sys.argv[2]) if len(sys.argv) > 2 else 7
rospy.init_node("sim_joy")
pub = rospy.Publisher("/joy_teleop/joy", Joy, queue_size=5)
while not rospy.is_shutdown():
    if not os.path.exists(os.path.join(d, "SILENT")):
        m = Joy()
        m.header.stamp = rospy.Time.now()
        m.axes = [0.0] * 11
        m.buttons = [0] * 13
        m.buttons[btn] = 0 if os.path.exists(os.path.join(d, "RELEASE")) else 1
        pub.publish(m)
    time.sleep(0.05)
