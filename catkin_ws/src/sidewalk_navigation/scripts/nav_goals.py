#!/usr/bin/env python3
"""nav_goals.py - send the goal points one after another and record what was PLANNED and what was DRIVEN.

RUNS ON the Jetson (same ROS master as move_base), sim and real alike.

    goals YAML:  goals: [{name: G1_east_end, x: 13.5, y: 0.2, yaw_deg: 0, timeout_s: 150}, ...]
                 (map frame = drive 10's frame: start mark = 0, 0; facing east = yaw 0)

For every goal it sends a move_base goal, waits for the result (or the timeout), then stands still for
--dwell s, and records into --out:
    goals_result.json  per goal: result, time taken, where the robot ended, distance to the goal
    actual.csv         t, x, y, yaw  - map -> base_link (where the ROBOT'S OWN localisation puts it), 10 Hz
    odom.csv           t, x, y, yaw  - odom -> base_link (the wheels + gyroscope + camera blend alone)
    truth.csv          t, x, y, yaw, v - the simulator's true pose and speed (sim only, --truth-model)
    plans.jsonl        one line per route the planner published: t, goal index, the route's points
    cmd.csv            t, vx, wz, state - every command sent to the robot (/nav/cmd_vel_sent)
    robot_status.jsonl the robot side's own status (why it moves or stops; the joystick button)
    fixes.csv          t, kind, id - localisation fixes from /rtabmap/info (real run only)
Progress line (ENGINEERING_NOTES.md rule 13): <jobs_dir>/<run>_nav.progress, e.g. "NAV goal 2/3 G2 dist 4.1 m 63s".

*Plain terms: it tells the robot "go there", then "go there", and writes down both the route it was given
and the path it actually took, so the two can be drawn on one map.*

usage:  nav_goals.py --run s2_nav_01 --goals goals_d10.yaml --out ~/.run_records/s2_nav_01/nav
"""
import argparse
import csv
import json
import math
import os
import sys
import threading
import time

import actionlib
import rospy
import tf2_ros
import yaml
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import TwistStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path
from std_msgs.msg import String

STATES = {GoalStatus.SUCCEEDED: "SUCCEEDED", GoalStatus.ABORTED: "ABORTED", GoalStatus.PREEMPTED: "PREEMPTED",
          GoalStatus.REJECTED: "REJECTED", GoalStatus.LOST: "LOST", GoalStatus.RECALLED: "RECALLED"}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Recorder(object):
    def __init__(self, a):
        self.a = a
        os.makedirs(a.out, exist_ok=True)
        self.lock = threading.Lock()
        self.goal_idx = -1
        self.t0 = rospy.Time.now().to_sec()
        self.f_act = open(os.path.join(a.out, "actual.csv"), "w")
        self.f_odom = open(os.path.join(a.out, "odom.csv"), "w")
        self.f_plan = open(os.path.join(a.out, "plans.jsonl"), "w")
        self.f_cmd = open(os.path.join(a.out, "cmd.csv"), "w")
        self.f_rs = open(os.path.join(a.out, "robot_status.jsonl"), "w")
        self.f_act.write("t,x,y,yaw,goal\n")
        self.f_odom.write("t,x,y,yaw,goal\n")
        self.f_cmd.write("t,vx,wz,state,goal\n")
        self.last = None
        self.tfb = tf2_ros.Buffer(rospy.Duration(30))
        self.tfl = tf2_ros.TransformListener(self.tfb)
        rospy.Subscriber(a.plan_topic, Path, self.on_plan, queue_size=5)
        rospy.Subscriber("/nav/cmd_vel_sent", TwistStamped, self.on_cmd, queue_size=50)
        rospy.Subscriber("/nav/robot_status", String, self.on_rs, queue_size=50)
        if a.truth_model:
            from gazebo_msgs.msg import ModelStates
            self.f_truth = open(os.path.join(a.out, "truth.csv"), "w")
            self.f_truth.write("t,x,y,yaw,v,goal\n")
            self.truth_last = 0.0
            self.truth = None
            rospy.Subscriber("/gazebo/model_states", ModelStates, self.on_truth, queue_size=1)
        if a.fixes:
            try:
                from rtabmap_msgs.msg import Info
                self.f_fix = open(os.path.join(a.out, "fixes.csv"), "w")
                self.f_fix.write("t,kind,id,node\n")
                rospy.Subscriber("/rtabmap/info", Info, self.on_info, queue_size=20)
            except ImportError:
                rospy.logwarn("rtabmap_msgs not importable: fixes.csv not written")
        rospy.Timer(rospy.Duration(0.1), self.sample)

    def rel(self, stamp=None):
        return (stamp.to_sec() if stamp else rospy.Time.now().to_sec()) - self.t0

    def on_plan(self, m):
        pts = [[round(p.pose.position.x, 3), round(p.pose.position.y, 3)] for p in m.poses]
        with self.lock:
            self.f_plan.write(json.dumps({"t": round(self.rel(), 3), "goal": self.goal_idx, "pts": pts}) + "\n")

    def on_cmd(self, m):
        with self.lock:
            self.f_cmd.write("%.3f,%.3f,%.3f,%s,%d\n" % (self.rel(m.header.stamp), m.twist.linear.x,
                                                        m.twist.angular.z, m.header.frame_id, self.goal_idx))

    def on_rs(self, m):
        with self.lock:
            self.f_rs.write(json.dumps({"t": round(self.rel(), 3), "goal": self.goal_idx,
                                        "status": json.loads(m.data)}) + "\n")

    def on_truth(self, m):
        if self.a.truth_model not in m.name:
            return
        now = self.rel()
        if now - self.truth_last < 0.05:
            return
        self.truth_last = now
        i = m.name.index(self.a.truth_model)
        p, tw = m.pose[i], m.twist[i]
        v = math.hypot(tw.linear.x, tw.linear.y)
        self.truth = (p.position.x, p.position.y, yaw_of(p.orientation), v)
        with self.lock:
            self.f_truth.write("%.3f,%.4f,%.4f,%.4f,%.4f,%d\n" % (now, p.position.x, p.position.y,
                                                                 yaw_of(p.orientation), v, self.goal_idx))

    def on_info(self, m):
        kind = "recognition" if m.loopClosureId > 0 else "nearby" if m.proximityDetectionId > 0 else None
        if kind:
            with self.lock:
                self.f_fix.write("%.3f,%s,%d,%d\n" % (self.rel(m.header.stamp), kind,
                                                      max(m.loopClosureId, m.proximityDetectionId), m.refId))

    def pose(self, parent, child="base_link"):
        try:
            t = self.tfb.lookup_transform(parent, child, rospy.Time(0), rospy.Duration(0.05))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException):
            return None
        tr = t.transform
        return tr.translation.x, tr.translation.y, yaw_of(tr.rotation)

    def sample(self, _e):
        now = self.rel()
        p = self.pose(self.a.map_frame)
        o = self.pose(self.a.odom_frame)
        with self.lock:
            if p:
                self.f_act.write("%.3f,%.4f,%.4f,%.4f,%d\n" % (now, p[0], p[1], p[2], self.goal_idx))
                self.last = p
            if o:
                self.f_odom.write("%.3f,%.4f,%.4f,%.4f,%d\n" % (now, o[0], o[1], o[2], self.goal_idx))

    def flush(self):
        with self.lock:
            for f in [self.f_act, self.f_odom, self.f_plan, self.f_cmd, self.f_rs] + \
                     [getattr(self, n) for n in ("f_truth", "f_fix") if hasattr(self, n)]:
                f.flush()


def progress(a, line):
    p = os.path.join(os.path.expanduser(a.jobs_dir), "%s_nav.progress" % a.run)
    with open(p + ".tmp", "w") as f:
        f.write(line + "\n")
    os.replace(p + ".tmp", p)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", required=True)
    ap.add_argument("--goals", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--dwell", type=float, default=5.0, help="s to stand still at each goal")
    ap.add_argument("--map-frame", default="map")
    ap.add_argument("--odom-frame", default="odom")
    ap.add_argument("--plan-topic", default="/move_base/GlobalPlanner/plan")
    ap.add_argument("--truth-model", default="", help="sim: Gazebo model name, e.g. husky")
    ap.add_argument("--fixes", action="store_true", help="record localisation fixes from /rtabmap/info")
    ap.add_argument("--jobs-dir", default="~/jobs")
    ap.add_argument("--start-at", type=int, default=0, help="skip the first N goals (resume)")
    a = ap.parse_args(rospy.myargv()[1:])
    goals = yaml.safe_load(open(a.goals))["goals"]
    rospy.init_node("nav_goals", anonymous=False)
    rec = Recorder(a)
    cli = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    progress(a, "NAV waiting for move_base 0/%d 0s" % len(goals))
    if not cli.wait_for_server(rospy.Duration(120)):
        progress(a, "NAV FAILED: move_base not available")
        sys.exit("move_base action server not available")
    results = []
    t_start = time.monotonic()
    for k, g in enumerate(goals):
        if k < a.start_at:
            continue
        if rospy.is_shutdown():
            break
        with rec.lock:
            rec.goal_idx = k
        mg = MoveBaseGoal()
        mg.target_pose.header.frame_id = a.map_frame
        mg.target_pose.header.stamp = rospy.Time.now()
        mg.target_pose.pose.position.x = float(g["x"])
        mg.target_pose.pose.position.y = float(g["y"])
        yaw = math.radians(float(g.get("yaw_deg", 0.0)))
        mg.target_pose.pose.orientation.z = math.sin(yaw / 2)
        mg.target_pose.pose.orientation.w = math.cos(yaw / 2)
        t_send = rospy.Time.now()
        tw0 = time.monotonic()
        cli.send_goal(mg)
        rospy.loginfo("goal %d/%d %s -> (%.2f, %.2f, %.0f deg)", k + 1, len(goals), g["name"], g["x"], g["y"],
                      g.get("yaw_deg", 0.0))
        timeout = float(g.get("timeout_s", 180))
        state = None
        while not rospy.is_shutdown():
            if cli.wait_for_result(rospy.Duration(1.0)):
                state = cli.get_state()
                break
            el = time.monotonic() - tw0
            d = math.hypot(rec.last[0] - g["x"], rec.last[1] - g["y"]) if rec.last else float("nan")
            progress(a, "NAV goal %d/%d %s  dist %.1f m  leg %ds  %ds" % (k + 1, len(goals), g["name"], d, el,
                                                                       time.monotonic() - t_start))
            rec.flush()
            if el > timeout:
                cli.cancel_goal()
                state = "TIMEOUT"
                break
        el = time.monotonic() - tw0
        p = rec.last
        truth = getattr(rec, "truth", None)
        r = {"k": k, "name": g["name"], "goal": [g["x"], g["y"], g.get("yaw_deg", 0.0)],
             "result": STATES.get(state, str(state)), "time_s": round(el, 1),
             "t_send_rel": round(t_send.to_sec() - rec.t0, 3),
             "end_estimate": [round(v, 3) for v in p] if p else None,
             "dist_estimate_to_goal_m": round(math.hypot(p[0] - g["x"], p[1] - g["y"]), 3) if p else None}
        if truth:
            r["end_truth"] = [round(v, 3) for v in truth[:3]]
            r["dist_truth_to_goal_m"] = round(math.hypot(truth[0] - g["x"], truth[1] - g["y"]), 3)
        results.append(r)
        rospy.loginfo("goal %s: %s in %.1f s, estimate %.2f m from goal", g["name"], r["result"], el,
                      r["dist_estimate_to_goal_m"] or -1)
        with open(os.path.join(a.out, "goals_result.json"), "w") as f:
            json.dump({"run": a.run, "goals_file": os.path.abspath(a.goals), "results": results}, f, indent=1)
        t_end = time.monotonic() + a.dwell
        while time.monotonic() < t_end and not rospy.is_shutdown():
            progress(a, "NAV goal %d/%d %s %s, standing %ds  %ds" % (k + 1, len(goals), g["name"], r["result"],
                                                                   t_end - time.monotonic(),
                                                                   time.monotonic() - t_start))
            time.sleep(1.0)
    with rec.lock:
        rec.goal_idx = -1
    rec.flush()
    ok = sum(1 for r in results if r["result"] == "SUCCEEDED")
    progress(a, "NAV DONE %d/%d goals reached  %ds" % (ok, len(results), time.monotonic() - t_start))
    print("DONE %d/%d reached" % (ok, len(results)))


if __name__ == "__main__":
    main()
