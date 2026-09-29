#!/usr/bin/env python3
"""turn_probe.py - SIM ONLY: how fast does the simulated Husky really turn on the spot for a commanded rate?

Publishes pure turn commands (forward speed 0) on /cmd_vel (twist_mux 'external') for 4 s each at
0.2, 0.3, 0.4, 0.6 rad/s, and measures the true turn rate and the true drift of the centre (Gazebo) over the
last 2 s of each. Run BEFORE the command receiver starts (nothing else may publish /cmd_vel then).
Writes <out>/turn_probe.json.
*Plain terms: a skid-steer robot has to scrub its tyres sideways to turn on the spot; below some speed the
simulated one does not turn at all.*
"""
import json
import math
import os
import sys
import time

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist

out = sys.argv[1]
rospy.init_node("turn_probe")
st = {}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def on_ms(m):
    if "husky" in m.name:
        p = m.pose[m.name.index("husky")]
        st.update(x=p.position.x, y=p.position.y, yaw=yaw_of(p.orientation), t=time.monotonic())


rospy.Subscriber("/gazebo/model_states", ModelStates, on_ms, queue_size=1)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
time.sleep(2.0)
res = []
for wz in (0.2, 0.3, 0.4, 0.6):
    t0 = time.monotonic()
    samples = []
    while time.monotonic() - t0 < 4.0:
        t = Twist()
        t.angular.z = wz
        pub.publish(t)
        if time.monotonic() - t0 > 2.0 and st:
            samples.append((st["t"], st["yaw"], st["x"], st["y"]))
        time.sleep(0.05)
    pub.publish(Twist())
    time.sleep(1.5)
    if len(samples) > 5:
        yaws = [s[1] for s in samples]
        unw = [yaws[0]]
        for y in yaws[1:]:
            d = (y - unw[-1] + math.pi) % (2 * math.pi) - math.pi
            unw.append(unw[-1] + d)
        dt = samples[-1][0] - samples[0][0]
        res.append({"cmd_wz": wz, "true_wz": round((unw[-1] - unw[0]) / dt, 3),
                    "centre_drift_mps": round(math.hypot(samples[-1][2] - samples[0][2],
                                                         samples[-1][3] - samples[0][3]) / dt, 3)})
        rospy.loginfo("%s", res[-1])
json.dump(res, open(os.path.join(out, "turn_probe.json"), "w"), indent=1)
print(json.dumps(res))
