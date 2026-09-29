#!/usr/bin/env python3
"""nav_cmd_recv.py - the ROBOT's half of the navigation command link. Deployed to the robot 27 Sept 2026 (s2_nav_01..03).

RUNS ON the robot's computer (Husky cpr-a200-0985, ROS Noetic, Python 3.8), ONLY after the user has approved it
(DEPLOY_NOTES.md), from our own folder on the robot's card. Standard library + rospy + stock message classes.
It is the ONLY program of ours that can move the robot, so it is small and every rule it follows is listed here.

WHAT IT DOES
    Receives wheel commands (forward speed, turn rate) from nav_cmd_sender.py on the Jetson over ONE TCP
    connection on the USB-C wire, and publishes them on the robot's /cmd_vel - the input Clearpath's own
    twist_mux (husky_control/config/twist_mux.yaml) names "external", priority 1, the LOWEST of all:

        input                          priority   who
        e_stop (lock, std_msgs/Bool)     255      blocks every input below while true
        joy_teleop/cmd_vel                10      the joystick, while its enable button (L1) is held
        kb_teleop/cmd_vel                  9      keyboard teleop
        twist_marker_server/cmd_vel        8      the RViz marker
        cmd_vel  <- THIS PROGRAM           1      (Clearpath: "external")

    *Plain terms: it is a remote hand on the steering wheel that ANY other control - the joystick above all -
    pushes aside at once. It changes nothing in the robot's own programs.*

WHEN IT PUBLISHES A MOVING COMMAND - only while ALL of these hold, checked 20 times a second:
    1. LINK   a command arrived from the Jetson less than cmd_timeout (0.5 s) ago, on the robot's own clock
              (arrival time, so no clock agreement with the Jetson is needed)
    2. GATE   the Jetson marked that command "go" (it sends "stop" when the route planner is idle, its
              position is stale, the camera obstacles are stale, or the pause file exists)
    3. HOLD   the user is HOLDING the autonomy button on the joystick (deadman_button, default 7 = R2 on the
              robot's PS4 layout, teleop_ps4.yaml) and the joystick's own messages are fresh (< 0.3 s; the
              robot's joy_node repeats them 20 times a second, teleop_ps4.yaml autorepeat_rate 20)
    4. NO E-STOP  nothing has published true on /e_stop (the lock above)
    5. SANE   both numbers are finite
    and the command is clamped AGAIN here: forward 0 .. max_vx (0.30 m/s), turn -max_wz .. +max_wz (0.40 rad/s).
    Reversing is impossible from this program (negative forward speed becomes 0): nothing sees behind the robot.

WHEN ANY ONE FAILS
    it publishes a ZERO command at once, keeps publishing zeros for zero_hold (0.5 s) so a lost message cannot
    matter, then publishes NOTHING. Silence hands the robot back to twist_mux, whose own 0.5 s timeout and
    the wheel controller's own 0.25 s timeout (husky_control control.yaml cmd_vel_timeout) stop the wheels
    even if this program itself dies.

    Stop-time budget (worst case, at the 0.30 m/s cap):
        link silent (cable out / Jetson frozen)  <= 0.5 s + one 0.05 s tick  -> <= 0.17 m rolled
        button released / e-stop                 <= 0.05 s (next joystick message) + one tick
        this program killed                      <= 0.25 s (wheel controller timeout)
        joystick L1 pressed                      immediately: twist_mux gives the joystick priority 10

WHAT IT NEVER DOES
    - publishes on any topic other than /cmd_vel, or changes any file or setting of the robot's
    - listens on WiFi: it binds ONLY the USB-C wire address (bind, default <robot-usb-address>). Nothing on the
      lab network can reach it. (bind:=0.0.0.0 is refused unless allow_any_bind:=true, sim only)
    - accepts a sender with a different run name, or a second sender (a new connection replaces the old,
      and the old one's last command is discarded)
    - starts if /use_sim_time is true (unless allow_sim_time:=true, sim only)

THE CONNECTION (frames like the wheel bridge's: 1 byte type, 4 bytes length little-endian, then the body)
    H hello    Jetson -> robot  JSON {proto, run, sender}
    W welcome  robot -> Jetson  JSON {proto, run, receiver, host, limits, deadman_button}
    E error    robot -> Jetson  JSON {why}; then the robot closes the connection
    C command  Jetson -> robot  seq u32, Jetson send time ns i64, vx f32, wz f32, gate u8, reason u8
    S status   robot -> Jetson  JSON, every 0.5 s: what the robot is doing and why

usage (on the robot, ONLY after approval - DEPLOY_NOTES.md step R3):
    setsid nohup python3 nav_cmd_recv.py __name:=nav_cmd_recv _run:=s2_nav_01 \\
        > ~/jobs/s2_nav_01_navcmd_recv.log 2>&1 < /dev/null &
"""
from __future__ import print_function

import json
import math
import os
import socket
import struct
import threading
import time

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

PROTO = 1
VERSION = "nav_cmd_recv 2026-09-27a"
FRAME = struct.Struct("<cI")
CMD = struct.Struct("<IqffBB")
MAX_BODY = 4096
REASONS = {0: "go", 1: "planner idle", 2: "position stale", 3: "camera obstacles stale", 4: "pause file",
           5: "startup", 6: "bad number", 7: "no localisation", 8: "shutdown", 9: "off route"}


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


def event(what):
    print("EVENT %d %s" % (time.time_ns(), what), flush=True)


def send_frame(sock, kind, body):
    sock.sendall(FRAME.pack(kind, len(body)) + body)


def recv_exact(sock, n):
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("closed by the Jetson")
        buf += chunk
    return buf


class Receiver(object):
    def __init__(self):
        self.run = rospy.get_param("~run")
        self.bind = rospy.get_param("~bind", os.environ.get("ROBOT_USB_ADDR", ""))
        self.port = int(rospy.get_param("~port", 8112))
        self.allow_any = bool(rospy.get_param("~allow_any_bind", False))
        self.max_vx = float(rospy.get_param("~max_vx", 0.30))
        self.max_wz = float(rospy.get_param("~max_wz", 0.40))
        self.cmd_timeout = float(rospy.get_param("~cmd_timeout", 0.5))
        self.zero_hold = float(rospy.get_param("~zero_hold", 0.5))
        self.rate = float(rospy.get_param("~rate", 20.0))
        self.deadman = int(rospy.get_param("~deadman_button", 7))
        self.joy_timeout = float(rospy.get_param("~joy_timeout", 0.3))
        self.joy_topic = rospy.get_param("~joy_topic", "/joy_teleop/joy")
        self.out_topic = rospy.get_param("~cmd_topic", "/cmd_vel")
        self.jobs = os.path.expanduser(rospy.get_param("~jobs_dir", os.environ.get("JOBS_DIR", "~/jobs")))
        # hard ceilings: a mistyped parameter can lower the limits, never raise them past these
        if self.max_vx > 0.5 or self.max_wz > 1.0 or self.max_vx < 0 or self.max_wz < 0:
            raise SystemExit("REFUSED: limits max_vx %.2f / max_wz %.2f outside 0..0.5 m/s, 0..1.0 rad/s"
                             % (self.max_vx, self.max_wz))
        if self.bind in ("", "0.0.0.0", "::") and not self.allow_any:
            raise SystemExit("REFUSED: bind %r would listen on WiFi too; the wire address only" % self.bind)
        if rospy.get_param("/use_sim_time", False) and not rospy.get_param("~allow_sim_time", False):
            raise SystemExit("REFUSED: /use_sim_time is true on this master")
        self.lock = threading.Lock()
        self.cmd = None            # (vx, wz, gate, reason, t_arrival_monotonic, seq)
        self.conn_id = 0
        self.connected = False
        self.joy_btn = None
        self.joy_t = None
        self.estop = False
        self.moving = False
        self.zero_until = 0.0
        self.why = "startup"
        self.n_move = self.n_zero = self.n_rx = 0
        self.max_gap = 0.0
        self.pub = rospy.Publisher(self.out_topic, Twist, queue_size=1)
        if self.deadman >= 0:
            rospy.Subscriber(self.joy_topic, Joy, self.on_joy, queue_size=5)
        rospy.Subscriber("/e_stop", Bool, self.on_estop, queue_size=5)
        os.makedirs(self.jobs, exist_ok=True)
        self.prog = os.path.join(self.jobs, "%s_navcmd_recv.progress" % self.run)
        self.t0 = time.monotonic()
        log(VERSION, "run", self.run, "bind %s:%d" % (self.bind, self.port), "limits %.2f m/s %.2f rad/s"
            % (self.max_vx, self.max_wz), "deadman button", self.deadman, "->", self.out_topic)

    # ---------------- inputs ----------------
    def on_joy(self, m):
        with self.lock:
            self.joy_t = time.monotonic()
            self.joy_btn = (len(m.buttons) > self.deadman and m.buttons[self.deadman] == 1)

    def on_estop(self, m):
        with self.lock:
            if bool(m.data) != self.estop:
                event("e_stop %s" % bool(m.data))
            self.estop = bool(m.data)

    # ---------------- the decision, 20 times a second ----------------
    def decide(self, now):
        with self.lock:
            c = self.cmd if self.connected else None
            joy_ok = self.deadman < 0 or (self.joy_t is not None and now - self.joy_t < self.joy_timeout
                                          and self.joy_btn)
            joy_why = ("joystick silent" if (self.joy_t is None or now - self.joy_t >= self.joy_timeout)
                       else "button released")
            estop = self.estop
        if c is None:
            return None, "no link"
        vx, wz, gate, reason, t_arr, _ = c
        if now - t_arr > self.cmd_timeout:
            return None, "link silent %.2f s" % (now - t_arr)
        if not (math.isfinite(vx) and math.isfinite(wz)):
            return None, "bad number"
        if gate != 1:
            return None, "Jetson: " + REASONS.get(reason, str(reason))
        if estop:
            return None, "e_stop"
        if not joy_ok:
            return None, "deadman " + joy_why
        return (min(max(vx, 0.0), self.max_vx), min(max(wz, -self.max_wz), self.max_wz)), "moving"

    def tick_loop(self):
        period = 1.0 / self.rate
        nxt = time.monotonic()
        last_status = 0.0
        while not rospy.is_shutdown():
            now = time.monotonic()
            cmd, why = self.decide(now)
            t = Twist()
            if cmd is not None:
                t.linear.x, t.angular.z = cmd
                self.pub.publish(t)
                self.n_move += 1
                if not self.moving:
                    event("MOVING allowed")
                self.moving = True
                self.zero_until = 0.0
            else:
                if self.moving:
                    event("STOP: %s" % why)
                    self.zero_until = now + self.zero_hold
                self.moving = False
                if now < self.zero_until:
                    self.pub.publish(t)       # zeros for zero_hold seconds, then silence
                    self.n_zero += 1
            if why != self.why:
                self.why = why
            if now - last_status >= 0.5:
                last_status = now
                self.status(now)
            nxt += period
            time.sleep(max(0.0, nxt - time.monotonic()))
            if time.monotonic() - nxt > 1.0:
                nxt = time.monotonic()

    def status(self, now):
        with self.lock:
            joy_age = None if self.joy_t is None else round(now - self.joy_t, 3)
            st = {"t": time.time(), "moving": self.moving, "why": self.why, "connected": self.connected,
                  "deadman_held": bool(self.joy_btn) if self.deadman >= 0 else None, "joy_age_s": joy_age,
                  "estop": self.estop, "n_move": self.n_move, "n_zero": self.n_zero, "n_rx": self.n_rx,
                  "max_gap_s": round(self.max_gap, 3)}
            sock = getattr(self, "sock", None)
        line = "NAVCMD_RECV %s  %s  link %s  button %s  moving-cmds %d  rx %d  %ds" % (
            "MOVING" if st["moving"] else "stopped", st["why"], "up" if st["connected"] else "DOWN",
            {True: "held", False: "released", None: "off"}[st["deadman_held"]], st["n_move"], st["n_rx"],
            now - self.t0)
        try:
            with open(self.prog + ".tmp", "w") as f:
                f.write(line + "\n")
            os.replace(self.prog + ".tmp", self.prog)
        except OSError:
            pass
        if sock is not None:
            try:
                send_frame(sock, b"S", json.dumps(st).encode())
            except OSError:
                pass

    # ---------------- the connection ----------------
    def serve(self):
        srv = None
        while not rospy.is_shutdown():
            if srv is None:
                try:
                    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    srv.bind((self.bind, self.port))
                    srv.listen(2)
                    srv.settimeout(1.0)
                    event("listening %s:%d" % (self.bind, self.port))
                except OSError as e:
                    log("cannot listen on %s:%d (%s) - is the USB-C wire up? retry in 2 s" % (self.bind, self.port, e))
                    srv = None
                    time.sleep(2.0)
                    continue
            try:
                s, addr = srv.accept()
            except socket.timeout:
                continue
            threading.Thread(target=self.session, args=(s, addr), daemon=True).start()

    def session(self, s, addr):
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        s.settimeout(2.0)
        try:
            kind, n = FRAME.unpack(recv_exact(s, FRAME.size))
            if kind != b"H" or n > MAX_BODY:
                raise ValueError("expected hello")
            hello = json.loads(recv_exact(s, n).decode())
            if hello.get("proto") != PROTO or hello.get("run") != self.run:
                send_frame(s, b"E", json.dumps({"why": "run/proto mismatch: want run %s proto %d, got %s"
                                                 % (self.run, PROTO, hello)}).encode())
                event("REFUSED %s: %s" % (addr[0], hello))
                s.close()
                return
        except Exception as e:  # noqa - any bad hello closes this connection only
            event("bad hello from %s: %s" % (addr[0], e))
            s.close()
            return
        try:   # the welcome goes out BEFORE the socket is shared with the status writer (no interleaving)
            send_frame(s, b"W", json.dumps({"proto": PROTO, "run": self.run, "receiver": VERSION,
                                            "host": socket.gethostname(), "deadman_button": self.deadman,
                                            "limits": [self.max_vx, self.max_wz]}).encode())
        except OSError as e:
            event("welcome failed to %s: %s" % (addr[0], e))
            s.close()
            return
        with self.lock:
            old = getattr(self, "sock", None)
            self.conn_id += 1
            my_id = self.conn_id
            self.sock = s
            self.cmd = None                 # an old connection's last command is never reused
            self.connected = True
        if old is not None:
            try:
                old.close()
            except OSError:
                pass
        event("CONNECTED %s (%s)" % (addr[0], hello.get("sender")))
        s.settimeout(self.cmd_timeout * 4)
        last = None
        try:
            while not rospy.is_shutdown():
                kind, n = FRAME.unpack(recv_exact(s, FRAME.size))
                if n > MAX_BODY:
                    raise ValueError("frame too long")
                body = recv_exact(s, n)
                if kind != b"C" or n != CMD.size:
                    continue
                seq, _t_send, vx, wz, gate, reason = CMD.unpack(body)
                now = time.monotonic()
                with self.lock:
                    if self.conn_id != my_id:
                        break
                    if last is not None:
                        self.max_gap = max(self.max_gap, now - last)
                    last = now
                    self.cmd = (float(vx), float(wz), int(gate), int(reason), now, seq)
                    self.n_rx += 1
        except (OSError, ValueError, ConnectionError, struct.error) as e:
            event("DISCONNECTED %s: %s" % (addr[0], e))
        finally:
            with self.lock:
                if self.conn_id == my_id:
                    self.connected = False
                    self.cmd = None
                    self.sock = None
            try:
                s.close()
            except OSError:
                pass


def main():
    rospy.init_node("nav_cmd_recv", disable_signals=False)
    r = Receiver()
    threading.Thread(target=r.serve, daemon=True).start()

    def on_shutdown():
        z = Twist()
        for _ in range(5):                  # a clean exit always ends on "stop"
            r.pub.publish(z)
            time.sleep(0.02)
        event("SHUTDOWN moving-cmds %d zeros %d rx %d max_gap %.3f s" % (r.n_move, r.n_zero, r.n_rx, r.max_gap))
    rospy.on_shutdown(on_shutdown)
    r.tick_loop()


if __name__ == "__main__":
    main()
