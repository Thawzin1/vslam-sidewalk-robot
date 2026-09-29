#!/usr/bin/env python3
"""safety_tests.py - SIM ONLY: does every stop path actually stop the (simulated) Husky, and how fast?

While move_base drives the simulated Husky down the long corridor at full speed (0.30 m/s cap), each test
triggers one stop condition, measures from the trigger until the TRUE speed (Gazebo) falls below 0.02 m/s,
holds 3 s, releases, and waits until the robot is driving again:

  T1 button released        the user lets go of the autonomy button     (sim_joy.py: RELEASE file)
  T2 joystick silent        the joystick disconnects                      (sim_joy.py: SILENT file)
  T3 link silent            the wire is pulled / the Jetson freezes       (SIGSTOP the sender: no bytes, socket open)
  T4 joystick override      the user presses L1, stick centred            (zero Twist on /joy_teleop/cmd_vel, 20 Hz)
  T5 e-stop lock            anything publishes true on /e_stop            (twist_mux lock, priority 255)
  T6 pause file             the operator touches the pause file        (Jetson gate)
  T7 sender killed          the Jetson program dies (connection closes)   (SIGKILL the sender; restarted after)

Writes <out>/safety_tests.json and <out>/safety_speed.csv (t, true speed, test) for the figure.
usage: safety_tests.py <ctl_dir> <out_dir> <sender_pid_file> <sender_restart_cmd_file>
"""
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time

import actionlib
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool

ctl, out, pidfile, restart_file = sys.argv[1:5]
os.makedirs(out, exist_ok=True)
rospy.init_node("safety_tests")
lock = threading.Lock()
state = {"v": 0.0, "x": 0.0, "y": 0.0, "t": 0.0}
f_speed = open(os.path.join(out, "safety_speed.csv"), "w")
f_speed.write("t,v,x,y,test\n")
current = {"name": ""}
T0 = time.monotonic()


def on_ms(m):
    if "husky" not in m.name:
        return
    i = m.name.index("husky")
    tw, p = m.twist[i], m.pose[i]
    now = time.monotonic() - T0
    with lock:
        if now - state["t"] >= 0.02:
            state.update(v=math.hypot(tw.linear.x, tw.linear.y), x=p.position.x, y=p.position.y, t=now)
            f_speed.write("%.3f,%.4f,%.4f,%.4f,%s\n" % (now, state["v"], state["x"], state["y"], current["name"]))


rospy.Subscriber("/gazebo/model_states", ModelStates, on_ms, queue_size=1)
pub_joy_cmd = rospy.Publisher("/joy_teleop/cmd_vel", Twist, queue_size=5)
pub_estop = rospy.Publisher("/e_stop", Bool, queue_size=5, latch=True)
cli = actionlib.SimpleActionClient("move_base", MoveBaseAction)
cli.wait_for_server(rospy.Duration(60))
legs = [(13.0, 0.0, 0.0), (1.5, 0.0, 180.0)]
leg = {"k": 0}


def send_leg():
    x, y, yaw = legs[leg["k"] % 2]
    g = MoveBaseGoal()
    g.target_pose.header.frame_id = "map"
    g.target_pose.header.stamp = rospy.Time.now()
    g.target_pose.pose.position.x, g.target_pose.pose.position.y = x, y
    g.target_pose.pose.orientation.z = math.sin(math.radians(yaw) / 2)
    g.target_pose.pose.orientation.w = math.cos(math.radians(yaw) / 2)
    cli.send_goal(g)
    leg["k"] += 1


def speed():
    with lock:
        return state["v"], state["x"], state["y"]


def wait_driving(timeout=90):
    """until the true speed has been above 0.2 m/s for 2 s; re-send a leg if the last one finished"""
    t_end = time.monotonic() + timeout
    since = None
    while time.monotonic() < t_end and not rospy.is_shutdown():
        if cli.get_state() in (3, 4, 5, 8, 9):   # finished (SUCCEEDED/ABORTED/REJECTED/...): next leg
            send_leg()
            time.sleep(1.0)
        v, _, _ = speed()
        if v > 0.2:
            since = since or time.monotonic()
            if time.monotonic() - since > 2.0:
                return True
        else:
            since = None
        time.sleep(0.05)
    return False


def sender_pid():
    return int(open(pidfile).read().strip())


def measure(name, trigger, release):
    current["name"] = name
    if not wait_driving():
        return {"test": name, "result": "robot never reached driving speed before the test"}
    v0, x0, y0 = speed()
    t_trig = time.monotonic()
    t_trig_rel = t_trig - T0
    trigger()
    px, py, dist, t_stop = x0, y0, 0.0, None
    while time.monotonic() - t_trig < 5.0:
        v, x, y = speed()
        dist += math.hypot(x - px, y - py)
        px, py = x, y
        if v < 0.02:
            t_stop = time.monotonic() - t_trig
            break
        time.sleep(0.01)
    # hold 3 s: does it STAY stopped?
    crept = 0.0
    hold_end = time.monotonic() + 3.0
    hx, hy = px, py
    while time.monotonic() < hold_end:
        _, x, y = speed()
        crept = max(crept, math.hypot(x - hx, y - hy))
        time.sleep(0.05)
    release()
    resumed = wait_driving(60)
    r = {"test": name, "t_trigger": round(t_trig_rel, 3), "speed_before_mps": round(v0, 3),
         "stop_time_s": round(t_stop, 3) if t_stop is not None else None,
         "distance_after_trigger_m": round(dist, 3), "moved_while_held_m": round(crept, 3),
         "resumed_after_release": resumed,
         "pass": t_stop is not None and crept < 0.02}
    current["name"] = ""
    rospy.loginfo("%s", r)
    return r


def touch(n):
    open(os.path.join(ctl, n), "w").close()


def rm(n):
    try:
        os.remove(os.path.join(ctl, n))
    except OSError:
        pass


joy_on = {"on": False}


def joy_loop():
    while not rospy.is_shutdown():
        if joy_on["on"]:
            pub_joy_cmd.publish(Twist())
        time.sleep(0.05)


threading.Thread(target=joy_loop, daemon=True).start()


def kill_sender():
    os.kill(sender_pid(), signal.SIGKILL)


def restart_sender():
    cmd = open(restart_file).read().strip()
    p = subprocess.Popen(["bash", "-c", cmd], start_new_session=True)
    time.sleep(1.0)
    rospy.loginfo("sender restarted (launcher pid %d)", p.pid)


pub_estop.publish(Bool(data=False))
send_leg()
tests = [
    ("T1 button released", lambda: touch("RELEASE"), lambda: rm("RELEASE")),
    ("T2 joystick silent", lambda: touch("SILENT"), lambda: rm("SILENT")),
    ("T3 link silent (sender frozen)", lambda: os.kill(sender_pid(), signal.SIGSTOP),
     lambda: os.kill(sender_pid(), signal.SIGCONT)),
    ("T4 joystick L1 override (stick centred)", lambda: joy_on.update(on=True), lambda: joy_on.update(on=False)),
    ("T5 e-stop lock", lambda: pub_estop.publish(Bool(data=True)), lambda: pub_estop.publish(Bool(data=False))),
    ("T6 pause file (Jetson gate)", lambda: touch("NAV_PAUSE"), lambda: rm("NAV_PAUSE")),
    ("T7 sender killed (connection closed)", kill_sender, restart_sender),
]
res = []
for name, trig, rel in tests:
    res.append(measure(name, trig, rel))
    with open(os.path.join(out, "safety_tests.json"), "w") as f:
        json.dump(res, f, indent=1)
cli.cancel_all_goals()
f_speed.close()
print(json.dumps(res, indent=1))
