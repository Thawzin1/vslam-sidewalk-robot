#!/usr/bin/env python3
"""gyro_zero_now.py - re-zero the robot's gyroscope on the start mark, only if it is still.

RUNS ON THE ROBOT. Called by robot_side.sh start, before the recording begins. Tested the same morning: the parked heading
drift fell from -2.35 to -0.76 deg/min after one re-zero, and was -0.61 a few minutes later
(results_robot_tests_2026-09-24.md (project records)).

  plain terms: the turn sensor is told "you are not turning right now; take whatever you
  read as zero". Doing that while the robot moves would teach it a wrong zero, so it first
  listens to the wheels for 3 s and refuses unless they are completely still.

What it changes: one command to the UM7 IMU through its driver's own service,
/imu_um7/reset with zero_gyros=true ONLY (reset_ekf and set_mag_ref stay false). No file,
parameter or launch file of the colleague's is touched.

  usage:  gyro_zero_now.py OUT.json
  exit:   0 re-zeroed; 1 skipped because the wheels moved or were silent; 2 service failed
"""
import json
import sys
import time

import rospy
from nav_msgs.msg import Odometry

LISTEN_S = 3.0
SETTLE_S = 5.0              # the test used 10 s; the drift after 10 s was already settled
STILL_V, STILL_W = 0.005, 0.005


def main():
    out_path = sys.argv[1] if len(sys.argv) > 1 else None
    rospy.init_node("gyro_zero_now", anonymous=True, disable_signals=True)
    seen = {"n": 0, "v": 0.0, "w": 0.0}

    def wheel(m):
        seen["n"] += 1
        seen["v"] = max(seen["v"], abs(m.twist.twist.linear.x))
        seen["w"] = max(seen["w"], abs(m.twist.twist.angular.z))

    sub = rospy.Subscriber("/husky_velocity_controller/odom", Odometry, wheel, queue_size=50)
    time.sleep(LISTEN_S)
    sub.unregister()
    res = dict(when=time.strftime("%Y-%m-%d %H:%M:%S %Z"), wheel_samples=seen["n"],
               wheel_max_speed_m_s=round(seen["v"], 5), wheel_max_turn_rad_s=round(seen["w"], 5))
    if seen["n"] < 10 or seen["v"] >= STILL_V or seen["w"] >= STILL_W:
        res["result"] = "SKIPPED: wheels %s" % ("silent" if seen["n"] < 10 else "moving")
        code = 1
    else:
        try:
            from um7.srv import Reset
            rospy.wait_for_service("/imu_um7/reset", timeout=5)
            rospy.ServiceProxy("/imu_um7/reset", Reset)(zero_gyros=True, reset_ekf=False,
                                                        set_mag_ref=False)
            time.sleep(SETTLE_S)
            res["result"] = "RE-ZEROED (zero_gyros only), settled %.0f s" % SETTLE_S
            code = 0
        except Exception as e:
            res["result"] = "FAILED: %s" % e
            code = 2
    if out_path:
        with open(out_path, "w") as f:
            json.dump(res, f, indent=1)
    print("   gyro: " + res["result"])
    sys.stdout.flush()
    return code


if __name__ == "__main__":
    sys.exit(main())
