#!/usr/bin/env python3
"""robot_bridge_send.py - the ROBOT's half of the robot-to-Jetson bridge (drive 3 fusion).

RUNS ON the robot's computer (Husky cpr-a200-0985, ROS Noetic, Python 3.8), started by
robot_side.sh from our own folder on the robot's USB drive. Standard library + rospy only.

WHAT IT DOES
    Listens to four of the robot's measurement streams and copies them, byte for byte,
    over ONE TCP connection (the internet's "reliable, in order" connection) to the
    Jetson, which re-times them onto its own clock (robot_bridge_recv.py).

      /husky_velocity_controller/odom   wheels only, ~10 Hz
      /imu/data                         gyroscope (after the robot's own filter), ~19.5 Hz
      /imu/data_raw                     the same gyroscope before that filter, ~19.5 Hz
      /odometry/filtered                the robot's own wheel+gyro blend, 50 Hz

    *Plain terms: a one-way copier. It listens, it never talks back to the robot.*

WHAT IT NEVER DOES (why it is safe on a colleague's computer)
    - It PUBLISHES NOTHING on the robot's ROS network, so it cannot move the robot or change
      any of the robot's programs. It only adds one listener per topic, like `rostopic echo`.
    - It changes no file and no setting of the robot's.
    - It reads only two kinds of message from the Jetson: a hello and a clock ping. There is
      no path from the Jetson to any robot topic. *Plain terms: it can look, never touch.*
    - The robot's programs never wait for it: messages go into a queue holding at most
      0.5 s of data, and if the WiFi stalls the OLDEST are dropped first (and counted).
    - It does not decode messages (rospy.AnyMsg hands over the raw bytes).

THE CONNECTION (bridge_design.md section 3)
    The robot LISTENS on TCP port 8111 (all interfaces); the Jetson connects out to it.
    A new connection replaces the old one at once (after a WiFi drop the old one is often
    half-dead and would take minutes to time out). The Jetson must say hello with the same
    run name, so a forgotten test receiver cannot take a real drive's stream.

    Frames on the stream: 1 byte type, 4 bytes length (little-endian), then the body.
      H hello    Jetson -> robot   JSON {proto, run, receiver}
      W welcome  robot -> Jetson   JSON {proto, run, sender, host, test}
      E error    robot -> Jetson   JSON {why}; then the robot closes the connection
      A announce robot -> Jetson   JSON {no, topic, type, md5}: once per topic per connection
      D data     robot -> Jetson   topic no (1 B), per-topic sequence no (4 B), robot receive
                                   time in ns (8 B), then the raw message bytes
                                   (18 bytes of framing with the type and length)
      P ping     Jetson -> robot   ping no (4 B), t1 (8 B, Jetson clock, ns)
      Q pong     robot -> Jetson   ping no, t1, t2 (robot clock when the ping arrived),
                                   t3 (robot clock just before the pong is written)
      S stats    robot -> Jetson   JSON, every 5 s: received / sent / dropped per topic,
                                   deepest queue
    All times are whole nanoseconds (Python integers), never floating-point seconds.

TEST MODE (only for test T1 on the Jetson, never on the robot)
    ~test:=true enables four fault injections, each announced loudly in the log:
      ~test_clock_offset_s  S   this program's "robot clock" = wall clock + S seconds
      ~test_clock_step_at   E   ... plus ~test_clock_step_s from wall-clock epoch E onward
      ~test_drop_at         E   at wall-clock epoch E, close the connection and refuse
                                connections for ~test_drop_refuse_s seconds
      ~test_hold_every      N   hold every Nth data frame back ~test_hold_ms before writing
                                it (and so everything queued behind it: a WiFi stall)
    Without ~test:=true any ~test_* parameter is IGNORED, and the log says so.
    Event lines start with "EVENT <wall-clock ns>"; the wall clock there is always the REAL
    one, never the test clock, so a test can line them up with other programs' logs.

  usage (on the robot, from robot_side.sh):
    setsid nohup python3 robot_bridge_send.py __name:=robot_bridge_send _run:=s2_static_03 \\
        _port:=8111 > ~/jobs/s2_static_03_bridge_send.log 2>&1 < /dev/null &
"""
from __future__ import print_function

import collections
import json
import os
import socket
import struct
import sys
import threading
import time

import rospy

PROTO = 1
VERSION = "robot_bridge_send 2026-09-24a"
DEFAULT_TOPICS = [
    "/husky_velocity_controller/odom",
    "/imu/data",
    "/imu/data_raw",
    "/odometry/filtered",
]
FRAME = struct.Struct("<cI")        # type, body length
DATA = struct.Struct("<BIq")        # topic no, sequence no, robot receive time (ns)
PING = struct.Struct("<Iq")         # ping no, t1
PONG = struct.Struct("<Iqqq")       # ping no, t1, t2, t3
MAX_CTRL = 65536                    # the Jetson only ever sends hello (small JSON) and pings


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


def event(what):
    """One machine-readable line; the ns is the REAL wall clock, even in test mode."""
    print("EVENT %d %s" % (time.time_ns(), what), flush=True)


class RobotClock(object):
    """The robot's clock. In test mode, a fake one: wall + offset (+ a step from a set time)."""

    def __init__(self, test, off_s, step_at, step_s):
        self.test = test
        self.off_ns = int(round(off_s * 1e9)) if test else 0
        self.step_at_ns = int(round(step_at * 1e9)) if (test and step_at > 0) else None
        self.step_ns = int(round(step_s * 1e9)) if test else 0

    def now_ns(self):
        t = time.time_ns()
        if not self.test:
            return t
        if self.step_at_ns is not None and t >= self.step_at_ns:
            return t + self.off_ns + self.step_ns
        return t + self.off_ns


def frame(kind, body):
    return FRAME.pack(kind, len(body)) + body


def recv_exact(sock, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise EOFError("connection closed by the Jetson")
        buf.extend(chunk)
    return bytes(buf)


def set_keepalive(sock):
    """Short TCP keepalive (2 s idle, 1 s interval, 3 probes), no small-packet batching,
    and a small send buffer so a WiFi stall cannot hide seconds of data in the kernel."""
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    for name, val in (("TCP_KEEPIDLE", 2), ("TCP_KEEPINTVL", 1), ("TCP_KEEPCNT", 3)):
        if hasattr(socket, name):
            sock.setsockopt(socket.IPPROTO_TCP, getattr(socket, name), val)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)


class Sender(object):
    def __init__(self):
        g = lambda k, d: rospy.get_param("~" + k, d)
        self.run = str(g("run", ""))
        self.port = int(g("port", 8111))
        self.bind = str(g("bind", "0.0.0.0"))
        self.topics = list(g("topics", DEFAULT_TOPICS))
        self.queue_s = float(g("queue_s", 0.5))
        self.test = bool(g("test", False))
        test_keys = ("test_clock_offset_s", "test_clock_step_at", "test_clock_step_s",
                     "test_drop_at", "test_drop_refuse_s", "test_hold_every", "test_hold_ms")
        given = [k for k in test_keys if rospy.has_param("~" + k)]
        if not self.test and given:
            log("NOTE: %s given without _test:=true - IGNORED (real clock, no faults)" % given)
        self.clock = RobotClock(self.test, float(g("test_clock_offset_s", 0.0)),
                                float(g("test_clock_step_at", 0.0)),
                                float(g("test_clock_step_s", 0.0)))
        self.drop_at = float(g("test_drop_at", 0.0)) if self.test else 0.0
        self.drop_refuse_s = float(g("test_drop_refuse_s", 5.0))
        self.hold_every = int(g("test_hold_every", 0)) if self.test else 0
        self.hold_s = float(g("test_hold_ms", 150)) / 1000.0
        if not self.run:
            raise SystemExit("robot_bridge_send: _run:=<run name> is required")
        if len(self.topics) > 255:
            raise SystemExit("robot_bridge_send: at most 255 topics")

        self.lock = threading.Condition()
        self.data = collections.deque()      # (enqueued monotonic s, no, seq, t_robot_ns, bytes)
        self.ctrl = collections.deque()      # control frames, always written before data
        self.conn = None                     # current client socket, or None
        self.conn_id = 0
        self.types = {}                      # topic no -> (type, md5)
        self.seq = [0] * len(self.topics)
        self.received = [0] * len(self.topics)
        self.sent = [0] * len(self.topics)
        self.dropped = [0] * len(self.topics)
        self.deepest = 0
        self.frames_written = 0
        self.holds = 0
        self.listener = None
        self.accepting = threading.Event()
        self.accepting.set()

    # ------------------------------------------------------------ robot side
    def on_msg(self, msg, no):
        t_robot = self.clock.now_ns()
        mono = time.monotonic()
        with self.lock:
            self.seq[no] = (self.seq[no] + 1) & 0xFFFFFFFF
            self.received[no] += 1
            if no not in self.types:
                h = getattr(msg, "_connection_header", None) or {}
                self.types[no] = (h.get("type", ""), h.get("md5sum", ""))
                log("topic %d %s: type %s md5 %s" % (no, self.topics[no], self.types[no][0],
                                                     self.types[no][1]))
                if self.conn is not None:
                    self.ctrl.append(("A", no))
            if self.conn is None:
                return                        # nobody to send to: the robot's own bag has it
            self.data.append((mono, no, self.seq[no], t_robot, msg._buff))
            while self.data and mono - self.data[0][0] > self.queue_s:
                old = self.data.popleft()
                self.dropped[old[1]] += 1
            if len(self.data) > self.deepest:
                self.deepest = len(self.data)
            self.lock.notify()

    # ------------------------------------------------------------ connection
    def announce_body(self, no):
        t, m = self.types[no]
        return json.dumps({"no": no, "topic": self.topics[no], "type": t, "md5": m}).encode()

    def close_conn(self, why, only_id=None):
        with self.lock:
            if self.conn is None or (only_id is not None and only_id != self.conn_id):
                return
            c, self.conn = self.conn, None
            self.data.clear()
            self.ctrl.clear()
            self.lock.notify_all()
        try:
            c.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        c.close()
        event("DISCONNECT %s" % why)
        log("connection closed: %s" % why)

    def handshake(self, c, addr):
        c.settimeout(3.0)
        kind, n = FRAME.unpack(recv_exact(c, FRAME.size))
        if kind != b"H" or n > MAX_CTRL:
            raise ValueError("first frame was not a hello")
        hello = json.loads(recv_exact(c, n).decode())
        if int(hello.get("proto", -1)) != PROTO:
            why = "protocol %s, this sender speaks %d" % (hello.get("proto"), PROTO)
        elif hello.get("run") != self.run:
            why = "run name %r, this sender serves %r" % (hello.get("run"), self.run)
        else:
            why = None
        if why:
            c.sendall(frame(b"E", json.dumps({"why": why}).encode()))
            raise ValueError("refused %s: %s" % (addr[0], why))
        return hello

    def serve(self):
        """Accept loop: one client at a time; a new one replaces the old one."""
        while not rospy.is_shutdown():
            self.accepting.wait(0.5)
            if not self.accepting.is_set():
                continue
            if self.listener is None:
                try:
                    ls = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    ls.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    ls.bind((self.bind, self.port))
                    ls.listen(2)
                    ls.settimeout(0.5)
                except OSError as e:
                    log("!! cannot listen on %s:%d: %s - retrying in 2 s" % (self.bind, self.port, e))
                    time.sleep(2)
                    continue
                self.listener = ls
                event("LISTENING %s:%d" % (self.bind, self.port))
                log("listening on %s:%d for run %s" % (self.bind, self.port, self.run))
            try:
                c, addr = self.listener.accept()
            except socket.timeout:
                continue
            except OSError:
                continue                       # listener closed under us (test drop)
            if not self.accepting.is_set():
                c.close()
                continue
            try:
                hello = self.handshake(c, addr)
            except Exception as e:             # noqa - any bad hello: close and carry on
                log("!! handshake with %s failed: %s" % (addr[0], e))
                event("REFUSED %s %s" % (addr[0], str(e).replace("\n", " ")[:200]))
                c.close()
                continue
            set_keepalive(c)
            c.settimeout(3.0)                  # the Jetson pings twice a second
            self.close_conn("replaced by a new connection from %s" % addr[0])
            with self.lock:
                self.conn_id += 1
                cid = self.conn_id
                self.conn = c
                self.ctrl.clear()
                welcome = {"proto": PROTO, "run": self.run, "sender": VERSION,
                           "host": socket.gethostname(), "test": self.test,
                           "topics": self.topics}
                self.ctrl.append(("W", json.dumps(welcome).encode()))
                for no in sorted(self.types):
                    self.ctrl.append(("A", no))
                self.lock.notify_all()
            event("CONNECT %s %s" % (addr[0], hello.get("receiver", "")))
            log("connected: %s (%s)" % (addr[0], hello.get("receiver", "")))
            threading.Thread(target=self.reader, args=(c, cid), daemon=True).start()

    def reader(self, c, cid):
        """Pings in, pongs queued. t2 is read the moment the ping is complete."""
        try:
            while not rospy.is_shutdown():
                kind, n = FRAME.unpack(recv_exact(c, FRAME.size))
                if n > MAX_CTRL:
                    raise ValueError("frame of %d bytes from the Jetson" % n)
                body = recv_exact(c, n)
                if kind == b"P":
                    t2 = self.clock.now_ns()
                    pno, t1 = PING.unpack(body)
                    with self.lock:
                        if self.conn_id == cid:
                            self.ctrl.append(("Q", (pno, t1, t2)))
                            self.lock.notify()
                # anything else from the Jetson is ignored on purpose
        except Exception as e:                 # noqa - EOF, timeout, reset
            self.close_conn("reader: %s" % e, only_id=cid)

    def writer(self):
        while not rospy.is_shutdown():
            with self.lock:
                while (not rospy.is_shutdown() and self.conn is not None
                       and not self.ctrl and not self.data):
                    self.lock.wait(0.5)
                if self.conn is None:
                    self.lock.wait(0.5)
                    continue
                c, cid = self.conn, self.conn_id
                if self.ctrl:
                    item = ("C", self.ctrl.popleft())
                elif self.data:
                    item = ("D", self.data.popleft())
                else:
                    continue
            try:
                if item[0] == "C":
                    kind, arg = item[1]
                    if kind == "Q":
                        pno, t1, t2 = arg
                        buf = frame(b"Q", PONG.pack(pno, t1, t2, self.clock.now_ns()))
                    elif kind == "A":
                        buf = frame(b"A", self.announce_body(arg))
                    else:
                        buf = frame(kind.encode(), arg)
                    c.sendall(buf)
                else:
                    mono, no, seq, t_robot, raw = item[1]
                    if time.monotonic() - mono > self.queue_s:
                        with self.lock:
                            self.dropped[no] += 1
                        continue
                    self.frames_written += 1
                    if self.hold_every and self.frames_written % self.hold_every == 0:
                        self.holds += 1
                        event("HOLD %d %.0f ms" % (self.holds, self.hold_s * 1000))
                        time.sleep(self.hold_s)
                    c.sendall(frame(b"D", DATA.pack(no, seq, t_robot) + raw))
                    with self.lock:
                        self.sent[no] += 1
            except Exception as e:             # noqa - the connection died under us
                self.close_conn("writer: %s" % e, only_id=cid)

    # ------------------------------------------------------------ test faults
    def test_drop(self):
        wait = self.drop_at - time.time()
        if wait <= 0:
            log("TEST drop time already passed (restarted process?) - no drop")
            return
        time.sleep(wait)
        event("TEST_DROP start: closing the connection, refusing for %.1f s" % self.drop_refuse_s)
        self.accepting.clear()
        ls, self.listener = self.listener, None
        if ls is not None:
            ls.close()
        self.close_conn("TEST drop")
        time.sleep(self.drop_refuse_s)
        self.accepting.set()                   # serve() re-opens the port and logs LISTENING
        event("TEST_DROP end: accepting again")

    # ------------------------------------------------------------ stats
    def stats(self):
        with self.lock:
            s = {"t_robot_ns": self.clock.now_ns(),
                 "received": dict(zip(self.topics, self.received)),
                 "sent": dict(zip(self.topics, self.sent)),
                 "dropped": dict(zip(self.topics, self.dropped)),
                 "deepest": self.deepest, "queue": len(self.data), "holds": self.holds,
                 "connected": self.conn is not None, "test": self.test}
            if self.conn is not None:
                self.ctrl.append(("S", json.dumps(s).encode()))
                self.lock.notify()
        return s

    def run_forever(self):
        for no, t in enumerate(self.topics):
            rospy.Subscriber(t, rospy.AnyMsg, self.on_msg, callback_args=no,
                             queue_size=100, tcp_nodelay=True)
        threading.Thread(target=self.serve, daemon=True).start()
        threading.Thread(target=self.writer, daemon=True).start()
        if self.test and self.drop_at > 0:
            threading.Thread(target=self.test_drop, daemon=True).start()
        t0 = time.time()
        next_stats = t0 + 5.0
        while not rospy.is_shutdown():
            time.sleep(0.2)                    # short, so a TERM is acted on at once
            if time.time() < next_stats:
                continue
            next_stats += 5.0
            s = self.stats()
            if self.test:
                log("!! TEST MODE - fake robot clock, injected faults; never use on the robot")
            log("up %4.0fs  connected %s  received %s  sent %s  dropped %s  deepest %d  holds %d"
                % (time.time() - t0, s["connected"], self.received, self.sent, self.dropped,
                   s["deepest"], s["holds"]))


def main():
    rospy.init_node("robot_bridge_send", anonymous=False)
    s = Sender()
    log("%s  run %s  port %d  topics %s  pid %d" % (VERSION, s.run, s.port, s.topics, os.getpid()))
    if s.test:
        log("!! TEST MODE: robot clock = wall %+.3f s, step %+.3f s at epoch %s, drop at %s "
            "for %.1f s, hold every %d frames for %.0f ms. NEVER use this on the robot."
            % (s.clock.off_ns / 1e9, s.clock.step_ns / 1e9,
               (s.clock.step_at_ns / 1e9) if s.clock.step_at_ns else "never",
               s.drop_at or "never", s.drop_refuse_s, s.hold_every, s.hold_s * 1000))
        event("TEST_MODE offset_ns %d step_ns %d step_at_ns %s" %
              (s.clock.off_ns, s.clock.step_ns, s.clock.step_at_ns))
    rospy.on_shutdown(lambda: s.close_conn("sender shutting down"))
    try:
        s.run_forever()
    finally:
        if s.listener is not None:
            s.listener.close()
        log("final  received %s  sent %s  dropped %s  deepest %d"
            % (s.received, s.sent, s.dropped, s.deepest))
        event("EXIT")


if __name__ == "__main__":
    main()
