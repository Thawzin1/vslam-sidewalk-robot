#!/usr/bin/env python3
"""nav_cmd_sender.py - the JETSON's half of the navigation command link.

RUNS ON the Jetson, on the Jetson's own ROS master (the one RTAB-Map and move_base run on). Standard library +
rospy + stock message classes.

WHAT IT DOES
    Takes the navigation program's wheel commands (move_base -> /nav/cmd_vel), checks them, and sends them
    20 times a second over ONE TCP connection to nav_cmd_recv.py on the robot (port 8112, the USB-C wire,
    <robot-usb-address>). Every message carries a "go" or "stop" mark and a reason.

    *Plain terms: the Jetson's side of a remote hand on the robot's steering wheel. It only ever says "go
    this fast" while it is sure the navigation is healthy; otherwise it says "stop", and the robot's side
    stops on its own anyway if the messages stop coming.*

IT SENDS "STOP" (zero speed, gate 0) WHEN
    - move_base has not produced a command in the last cmd_stale s (0.3 s): no goal, or the planner is stuck
    - the robot position (odometry, default /fused/odometry = wheels + gyroscope + camera blend) is older
      than odom_stale s (0.5 s)
    - the camera obstacle points (/nav/obstacles_cloud) are older than cloud_stale s (1.0 s): the robot
      would be driving blind to anything that moved since
    - the pause file exists (default ~/slam_series2/NAV_PAUSE): `touch` it to stop, `rm` it to resume
    - there is no map -> base_link position (localisation) for more than 1 s
    - OFF ROUTE: the robot is more than leash_m (1.0 m) from the route planned when the current goal was sent
      (added after sim dry run 5, where the wheel-command chooser crept the robot 2 m off its route when turning
      on the spot stalled). A big localisation correction trips it too - which is the right moment to stop.
    - a number is not finite
    and it clamps every command: forward 0 .. max_vx (0.30 m/s), turn -max_wz .. +max_wz (0.40 rad/s).
    The robot side applies the SAME limits again, plus the joystick button and the e-stop.

VISIBLE FROM A PHONE (ENGINEERING_NOTES.md rules 8 and 13), on the Jetson:
    <jobs_dir>/<run>_navcmd.progress   one line every 2 s: link, gate, what the ROBOT says it is doing
    <jobs_dir>/<run>_navcmd_events.log connects, drops, gate changes, robot stop reasons
    topics: /nav/cmd_vel_sent (geometry_msgs/TwistStamped, what was sent) and /nav/robot_status
    (std_msgs/String, the robot's JSON status) - both recorded by nav_goals.py.

PARAMETERS (private, ~name)
    run REQUIRED (the robot side refuses any other run name) · robot "<robot-usb-address>" (comma list, tried in
    order; the USB-C wire only by default) · port 8112 · cmd_in /nav/cmd_vel · odom_topic /fused/odometry ·
    cloud_topic /nav/obstacles_cloud · pause_file ~/slam_series2/NAV_PAUSE · max_vx 0.30 · max_wz 0.40 ·
    rate 20 · cmd_stale 0.3 · odom_stale 0.5 · cloud_stale 1.0 · leash_m 1.0 (0 = off) ·
    plan_topic /move_base/GlobalPlanner/plan · goal_topic /move_base/goal · jobs_dir ~/jobs · allow_sim_time false

  usage (on the Jetson, from start_nav.sh):
    setsid nohup python3 nav_cmd_sender.py __name:=nav_cmd_sender _run:=s2_nav_01 \\
        > ~/.run_records/s2_nav_01/navcmd_sender.log 2>&1 < /dev/null &
"""
from __future__ import print_function

import json
import math
import os
import socket
import struct
import threading
import time

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import Twist, TwistStamped
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

PROTO = 1
VERSION = "nav_cmd_sender 2026-09-27c"
FRAME = struct.Struct("<cI")
CMD = struct.Struct("<IqffBB")
MAX_BODY = 65536
GO, IDLE, POS_STALE, CLOUD_STALE, PAUSED, STARTUP, BADNUM, NO_LOC, OFF_ROUTE, TOO_CLOSE, UNSAFE_TURN, NO_CLEAR = 0, 1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12
REASONS = {GO: "go", IDLE: "planner idle", POS_STALE: "position stale", CLOUD_STALE: "camera obstacles stale",
           PAUSED: "pause file", STARTUP: "startup", BADNUM: "bad number", NO_LOC: "no localisation",
           OFF_ROUTE: "off route", TOO_CLOSE: "too close to a wall", UNSAFE_TURN: "turn on the spot not safe here",
           NO_CLEAR: "clearance unknown"}


def dist_to_polyline(p, poly):
    if len(poly) == 1:
        return float(np.hypot(*(p - poly[0])))
    a, b = poly[:-1], poly[1:]
    ab = b - a
    t = np.clip(((p - a) * ab).sum(1) / np.maximum((ab ** 2).sum(1), 1e-12), 0, 1)
    return float(np.hypot(*(p - (a + t[:, None] * ab)).T).min())


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


def recv_exact(sock, n):
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("closed by the robot")
        buf += chunk
    return buf


class Sender(object):
    def __init__(self):
        self.run = rospy.get_param("~run")
        self.addrs = [a.strip() for a in str(rospy.get_param("~robot", os.environ.get("ROBOT_USB_ADDR", ""))).split(",") if a.strip()]
        self.port = int(rospy.get_param("~port", 8112))
        self.max_vx = float(rospy.get_param("~max_vx", 0.30))
        self.max_wz = float(rospy.get_param("~max_wz", 0.40))
        self.rate = float(rospy.get_param("~rate", 20.0))
        self.cmd_stale = float(rospy.get_param("~cmd_stale", 0.3))
        self.odom_stale = float(rospy.get_param("~odom_stale", 0.5))
        self.cloud_stale = float(rospy.get_param("~cloud_stale", 1.0))
        self.pause_file = os.path.expanduser(rospy.get_param("~pause_file", "~/slam_series2/NAV_PAUSE"))
        self.leash = float(rospy.get_param("~leash_m", 1.0))
        # real time, against the local costmap (saved walls + what the camera sees now). clearance = shortest distance
        # from the robot's outline to a wall/obstacle cell; turn_r = the circle the outline sweeps when turning on the spot.
        self.half_l = float(rospy.get_param("~half_length", 0.495)); self.half_w = float(rospy.get_param("~half_width", 0.335))
        self.margin = float(rospy.get_param("~safety_margin", 0.05))     # m kept free around the TURN circle (0.10 in sim run 15 stalled in the 1.0 m pinch)
        self.stop_at = float(rospy.get_param("~stop_below", 0.02))      # m: stop if the outline is this close to a wall cell
        self.slow_at = float(rospy.get_param("~slow_below", 0.40))       # m: full speed above, slowing below
        self.min_v = float(rospy.get_param("~creep_speed", 0.10))        # m/s at the margin
        self.turn_r = math.hypot(self.half_l, self.half_w)               # 0.60 m for the Husky outline
        self.cm = None; self.clear = None; self.cclear = None; self.t_cm = None
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.jobs = os.path.expanduser(rospy.get_param("~jobs_dir", os.environ.get("JOBS_DIR", "~/jobs")))
        if self.max_vx > 0.5 or self.max_wz > 1.0:
            raise SystemExit("REFUSED: limits above 0.5 m/s / 1.0 rad/s")
        if rospy.get_param("/use_sim_time", False) and not rospy.get_param("~allow_sim_time", False):
            raise SystemExit("REFUSED: /use_sim_time is true on this master (a real run would freeze)")
        os.makedirs(self.jobs, exist_ok=True)
        self.prog = os.path.join(self.jobs, "%s_navcmd.progress" % self.run)
        self.evlog = open(os.path.join(self.jobs, "%s_navcmd_events.log" % self.run), "a")
        self.lock = threading.Lock()
        self.t_cmd = self.t_odom = self.t_cloud = None
        self.first_plan = None          # the route planned when the current goal was sent
        self.want_plan = False
        self.t_pose = None
        self.pose = None
        self.off = 0.0
        self.tfb = tf2_ros.Buffer(rospy.Duration(10))
        self.tfl = tf2_ros.TransformListener(self.tfb)
        self.cmd = (0.0, 0.0)
        self.sock = None
        self.link = "DOWN"
        self.robot = {}
        self.gate_prev = None
        self.seq = 0
        self.n_go = self.n_stop = 0
        self.t0 = time.monotonic()
        self.pub_sent = rospy.Publisher("/nav/cmd_vel_sent", TwistStamped, queue_size=10)
        self.pub_robot = rospy.Publisher("/nav/robot_status", String, queue_size=10)
        rospy.Subscriber(rospy.get_param("~cmd_in", "/nav/cmd_vel"), Twist, self.on_cmd, queue_size=1)
        rospy.Subscriber(rospy.get_param("~costmap_topic", "/move_base/local_costmap/costmap"), OccupancyGrid,
                         self.on_costmap, queue_size=1, buff_size=2 ** 22)
        rospy.Subscriber(rospy.get_param("~odom_topic", "/fused/odometry"), Odometry, self.on_odom, queue_size=1)
        rospy.Subscriber(rospy.get_param("~cloud_topic", "/nav/obstacles_cloud"), PointCloud2, self.on_cloud,
                         queue_size=1, buff_size=2 ** 22)
        rospy.Subscriber(rospy.get_param("~goal_topic", "/move_base/goal"), MoveBaseActionGoal, self.on_goal,
                         queue_size=5)
        rospy.Subscriber(rospy.get_param("~plan_topic", "/move_base/GlobalPlanner/plan"), Path, self.on_plan,
                         queue_size=5)
        self.event("START %s run %s robot %s:%d limits %.2f m/s %.2f rad/s" % (
            VERSION, self.run, ",".join(self.addrs), self.port, self.max_vx, self.max_wz))

    def event(self, what):
        line = "%s %s" % (time.strftime("%H:%M:%S"), what)
        print("EVENT %d %s" % (time.time_ns(), what), flush=True)
        self.evlog.write(line + "\n")
        self.evlog.flush()

    def on_cmd(self, m):
        with self.lock:
            self.cmd = (m.linear.x, m.angular.z)
            self.t_cmd = time.monotonic()

    def on_odom(self, _m):
        self.t_odom = time.monotonic()

    def on_cloud(self, _m):
        self.t_cloud = time.monotonic()

    def on_goal(self, _m):
        with self.lock:
            self.first_plan = None
            self.want_plan = True
        self.event("NEW GOAL: waiting for its first route (leash %.1f m)" % self.leash)

    def on_plan(self, m):
        if len(m.poses) < 2:
            return
        with self.lock:
            if self.want_plan:
                self.first_plan = np.array([[p.pose.position.x, p.pose.position.y] for p in m.poses])
                self.want_plan = False

    def on_costmap(self, m):
        """Distance from every cell to the nearest wall/obstacle cell (cost 100 = lethal only; 99 is the inscribed keep-away band, not a wall)."""
        from scipy import ndimage
        a = np.array(m.data, np.int16).reshape(m.info.height, m.info.width)
        d = ndimage.distance_transform_edt(a < 100) * m.info.resolution
        with self.lock:
            self.cm = (d, m.info.origin.position.x, m.info.origin.position.y, m.info.resolution, m.header.frame_id)
            self.t_cm = time.monotonic()

    def clearance(self):
        """(outline clearance, centre clearance) in metres from the local costmap, or (None, None)."""
        with self.lock:
            cm = self.cm
        if cm is None:
            return None, None
        d, ox, oy, res, frame = cm
        try:
            t = self.tfb.lookup_transform(frame, self.base_frame, rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException):
            return None, None
        x, y = t.transform.translation.x, t.transform.translation.y
        q = t.transform.rotation; yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        c, s_ = math.cos(yaw), math.sin(yaw); best = 9.9
        L, W = self.half_l, self.half_w
        pts = [(u, v) for u in np.linspace(-L, L, 21) for v in (-W, W)] + [(u, v) for u in (-L, L) for v in np.linspace(-W, W, 15)]
        for u, v in pts:
            gx = int((x + c * u - s_ * v - ox) / res); gy = int((y + s_ * u + c * v - oy) / res)
            if 0 <= gy < d.shape[0] and 0 <= gx < d.shape[1]:
                best = min(best, d[gy, gx])
        gx = int((x - ox) / res); gy = int((y - oy) / res)
        cen = d[gy, gx] if (0 <= gy < d.shape[0] and 0 <= gx < d.shape[1]) else None
        # next 45 deg in each direction - the circle test (27b) refused turns almost everywhere in a 1.7 m corridor,
        # the planner then turned while driving forward and those arcs carried the robot towards the walls.
        sweep = {}
        for sgn in (1, -1):
            m_ = 9.9
            for da in np.radians(np.arange(5, 50, 5)):
                ca, sa = math.cos(yaw + sgn * da), math.sin(yaw + sgn * da)
                for u, v in pts:
                    gx2 = int((x + ca * u - sa * v - ox) / res); gy2 = int((y + sa * u + ca * v - oy) / res)
                    if 0 <= gy2 < d.shape[0] and 0 <= gx2 < d.shape[1]:
                        m_ = min(m_, d[gy2, gx2])
            sweep[sgn] = m_
        self.sweep = sweep
        return best, cen

    def update_pose(self, now):
        try:
            t = self.tfb.lookup_transform(self.map_frame, self.base_frame, rospy.Time(0))
            self.pose = np.array([t.transform.translation.x, t.transform.translation.y])
            self.t_pose = now
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException):
            pass

    def gate(self, now):
        with self.lock:
            vx, wz = self.cmd
            t_cmd = self.t_cmd
        if os.path.exists(self.pause_file):
            return 0.0, 0.0, PAUSED
        if self.t_odom is None or now - self.t_odom > self.odom_stale:
            return 0.0, 0.0, POS_STALE
        if self.t_cloud is None or now - self.t_cloud > self.cloud_stale:
            return 0.0, 0.0, CLOUD_STALE
        self.update_pose(now)
        if self.t_pose is None or now - self.t_pose > 1.0:
            return 0.0, 0.0, NO_LOC
        with self.lock:
            fp = self.first_plan
        if self.leash > 0 and fp is not None:
            self.off = dist_to_polyline(self.pose, fp)
            if self.off > self.leash:
                return 0.0, 0.0, OFF_ROUTE
        if t_cmd is None or now - t_cmd > self.cmd_stale:
            return 0.0, 0.0, IDLE
        if not (math.isfinite(vx) and math.isfinite(wz)):
            return 0.0, 0.0, BADNUM
        vx, wz = min(max(vx, 0.0), self.max_vx), min(max(wz, -self.max_wz), self.max_wz)
        # slow down as it gets close; turn on the spot only where the whole swept circle keeps the margin
        if self.t_cm is None or now - self.t_cm > 2.0:
            return 0.0, 0.0, NO_CLEAR
        self.clear, self.cclear = self.clearance()
        if self.clear is None:
            return 0.0, 0.0, NO_CLEAR
        if self.clear < self.stop_at:
            return 0.0, 0.0, TOO_CLOSE
        if vx < 0.05 and abs(wz) > 0.05:
            sw = getattr(self, "sweep", {}).get(1 if wz > 0 else -1)
            if sw is None or sw < self.margin:            # the rotating outline would come within 5 cm of a wall
                return 0.0, 0.0, UNSAFE_TURN
        if self.clear < self.slow_at:
            f = max(0.0, (self.clear - self.stop_at) / max(1e-3, self.slow_at - self.stop_at))
            vx = min(vx, self.min_v + (self.max_vx - self.min_v) * f)
        return vx, wz, GO

    def connect_loop(self):
        while not rospy.is_shutdown():
            if self.sock is not None:
                time.sleep(0.2)
                continue
            for a in self.addrs:
                try:
                    s = socket.create_connection((a, self.port), timeout=2.0)
                    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    body = json.dumps({"proto": PROTO, "run": self.run, "sender": VERSION}).encode()
                    s.sendall(FRAME.pack(b"H", len(body)) + body)
                    s.settimeout(2.0)
                    kind, n = FRAME.unpack(recv_exact(s, FRAME.size))
                    reply = json.loads(recv_exact(s, n).decode()) if n <= MAX_BODY else {}
                    if kind != b"W":
                        self.event("ROBOT REFUSED %s: %s" % (a, reply.get("why", reply)))
                        s.close()
                        continue
                    s.settimeout(None)
                    self.robot = {"welcome": reply}
                    with self.lock:
                        self.sock = s
                        self.link = "up %s" % a
                    self.event("CONNECTED %s receiver %s limits %s button %s" % (
                        a, reply.get("receiver"), reply.get("limits"), reply.get("deadman_button")))
                    threading.Thread(target=self.read_loop, args=(s,), daemon=True).start()
                    break
                except (OSError, ValueError, ConnectionError, struct.error) as e:
                    log("connect %s:%d failed: %s" % (a, self.port, e))
            if self.sock is None:
                time.sleep(1.0)

    def drop(self, s, why):
        with self.lock:
            if self.sock is s:
                self.sock = None
                self.link = "DOWN"
                self.event("LINK DOWN: %s" % why)
        try:
            s.close()
        except OSError:
            pass

    def read_loop(self, s):
        try:
            while not rospy.is_shutdown():
                kind, n = FRAME.unpack(recv_exact(s, FRAME.size))
                body = recv_exact(s, n) if n <= MAX_BODY else b""
                if kind == b"S":
                    st = json.loads(body.decode())
                    prev = self.robot.get("status", {})
                    self.robot["status"] = st
                    self.robot["t"] = time.monotonic()
                    self.pub_robot.publish(String(data=body.decode()))
                    if prev.get("why") != st.get("why"):
                        self.event("ROBOT: %s" % st.get("why"))
                elif kind == b"E":
                    self.event("ROBOT ERROR: %s" % body.decode(errors="replace"))
        except (OSError, ValueError, ConnectionError, struct.error) as e:
            self.drop(s, str(e))

    def send_loop(self):
        period = 1.0 / self.rate
        nxt = time.monotonic()
        last_prog = 0.0
        while not rospy.is_shutdown():
            now = time.monotonic()
            vx, wz, why = self.gate(now)
            if why != self.gate_prev:
                self.event("GATE %s" % ("OPEN" if why == GO else "SHUT: " + REASONS[why]
                                    + (" (%.2f m from the route)" % self.off if why == OFF_ROUTE else "")
                                    + (" (outline %.2f m, centre %.2f m from the nearest wall)" % (self.clear or -1, self.cclear or -1)
                                       if why in (TOO_CLOSE, UNSAFE_TURN) else "")))
                self.gate_prev = why
            self.seq += 1
            body = CMD.pack(self.seq, time.time_ns(), vx, wz, 1 if why == GO else 0, why)
            with self.lock:
                s = self.sock
            if s is not None:
                try:
                    s.sendall(FRAME.pack(b"C", len(body)) + body)
                    if why == GO:
                        self.n_go += 1
                    else:
                        self.n_stop += 1
                except OSError as e:
                    self.drop(s, str(e))
            ts = TwistStamped()
            ts.header.stamp = rospy.Time.now()
            ts.header.frame_id = "base_link" if (s is not None and why == GO) else "not_sent" if s is None else "stop"
            ts.twist.linear.x, ts.twist.angular.z = vx, wz
            self.pub_sent.publish(ts)
            if now - last_prog >= 2.0:
                last_prog = now
                self.progress(now, why)
            nxt += period
            time.sleep(max(0.0, nxt - time.monotonic()))
            if time.monotonic() - nxt > 1.0:
                nxt = time.monotonic()

    def progress(self, now, why):
        st = self.robot.get("status", {})
        age = now - self.robot["t"] if "t" in self.robot else None
        robot = ("robot %s (%s), button %s" % ("MOVING" if st.get("moving") else "stopped", st.get("why"),
                 {True: "held", False: "released", None: "off"}.get(st.get("deadman_held"), "?"))
                 if age is not None and age < 2.0 else "robot status not heard")
        sw = getattr(self, "sweep", {})
        cl = ("  clear %.2f m (turn: left %.2f right %.2f m)" % (self.clear, sw.get(1, -1), sw.get(-1, -1))
              if self.clear is not None else "  clear ?")
        line = "NAVCMD link %s  gate %s  %s%s  go %d stop %d  %ds" % (
            self.link, "OPEN" if why == GO else "SHUT(%s)" % REASONS[why], robot, cl, self.n_go, self.n_stop,
            now - self.t0)
        try:
            with open(self.prog + ".tmp", "w") as f:
                f.write(line + "\n")
            os.replace(self.prog + ".tmp", self.prog)
        except OSError:
            pass


def main():
    rospy.init_node("nav_cmd_sender")
    s = Sender()
    threading.Thread(target=s.connect_loop, daemon=True).start()

    def on_shutdown():
        body = CMD.pack(s.seq + 1, time.time_ns(), 0.0, 0.0, 0, 8)
        with s.lock:
            sk = s.sock
        if sk is not None:
            try:
                sk.sendall(FRAME.pack(b"C", len(body)) + body)
            except OSError:
                pass
        s.event("SHUTDOWN go %d stop %d" % (s.n_go, s.n_stop))
    rospy.on_shutdown(on_shutdown)
    s.send_loop()


if __name__ == "__main__":
    main()
