#!/usr/bin/env python3
"""robot_bridge_recv.py - the JETSON's half of the robot-to-Jetson bridge (drive 3 fusion).

RUNS ON the Jetson, on the drive's ROS master, started by start_drive.sh only when FUSION=1.
Standard library + rospy + the stock nav_msgs / sensor_msgs / diagnostic_msgs classes.

WHAT IT DOES
    Connects out to robot_bridge_send.py on the robot (TCP port 8111), receives the robot's
    wheel, gyroscope and own-filter messages, puts each one on the JETSON's clock, and
    republishes it on the Jetson:

      robot topic                        Jetson topic        frame change
      /husky_velocity_controller/odom -> /robot/wheel_odom   odom -> robot_odom
      /imu/data                       -> /robot/imu          none (already base_link)
      /imu/data_raw                   -> /robot/imu_raw      none
      /odometry/filtered              -> /robot/ekf_odom     odom -> robot_odom
      (its own health)                -> /robot_bridge/status  1 Hz, diagnostic_msgs

    The robot's "odom" is renamed "robot_odom" because on the Jetson "odom" belongs to the
    Jetson's own filter. "robot_odom" is only a label: nothing publishes or looks it up.
    IT PUBLISHES NO TF (no transform at all), so it can never become a second owner of
    odom -> base_link.

    *Plain terms: the robot's measurements arrive with the robot's clock on them; this
    re-labels each with the Jetson's time, drops any it cannot vouch for, and passes the
    rest on.*

THE CLOCK (bridge_design.md section 3.5, the same four-timestamp exchange NTP uses)
    Twice a second: Jetson notes t1, robot notes t2 on arrival and t3 on reply, Jetson
    notes t4.  offset = ((t2-t1)+(t3-t4))/2 (robot minus Jetson), round trip
    = (t4-t1)-(t3-t2); the true offset lies within offset +- round trip/2.
      - keep the last 16 pings; estimate = the one with the SHORTEST round trip
      - LOCK after 6 pings whose best bound is <= 20 ms; nothing is published before that
      - then follow the estimate by at most 0.5 ms per ping (1 ms per second)
      - 3 pings in a row more than 50 ms (+ their own bound) away = a STEP (the robot's clock
        was corrected): the applied offset jumps to the new value, logged and counted
      - after a reconnection, 2 pings that agree with the previous offset (within their
        bound + 2 ms) resume publishing; otherwise a full 6-ping lock
    Each message: stamp_jetson = stamp_robot - offset, then its AGE (arrival minus that):
      - age < -20 ms  (stamped in the future)   -> dropped, counted
      - age > 300 ms  (stale)                   -> dropped, counted
      - not later than the last one published on its topic (out of order) -> dropped
    *Plain terms: a late reading is harmless if its time stamp is right; a wrong time stamp
    is what ruins a blend. So anything whose time cannot be trusted is thrown away.*

DRIVE-4 CHANGE A1 (2026-09-25, receiver only; the sender and the protocol are unchanged)
    Rehearsal T4b: after the receiver was restarted, data came back only 6.1 s later -
    0.5 s to start and connect, 2.5 s to lock the clock (6 pings at 2 a second), then 3.0 s
    in which the thread that reads the link read NOTHING (the pongs of 5 pings sent 0.5 s
    apart were all read in one burst at the end, the process used 0 % of a core, its 1 Hz
    status kept publishing, and it never declared the link down). Every older message in
    that burst was then over 0.3 s old and dropped as stale. The same 2-3 s gaps appear
    ~20 times in drive 3 and 5 times in T4b with the link UP. The one thing that thread did
    that can wait seconds without using a core is WRITING FILES on the Jetson's busy
    internal disk (a clock row per ping, event lines, the progress file renamed into place)
    [INFERENCE from the timing above, not a traced system call]. So:
      1. The link-reading thread never touches a file: event lines, clock rows, progress
         lines and screen prints go into a queue that ONE separate thread writes out.
      2. While the clock is not locked, ping 10 times a second (lock after 6 pings in ~0.5 s
         instead of ~2.5 s); 2 a second once UP, as before.
      3. When the robot cannot be reached at all, try again every 0.5 s (was 0.5, 1, then
         every 2 s: drive 3 reconnected 3 s after the WiFi came back), each try waiting at
         most 1 s for the connection. A sender that REFUSES the hello still backs off.
      4. The link-reading thread's own pauses over 0.5 s are counted ("loop_stalls", with the
         longest) in the status and the final line, so drive 4 shows whether any remain.
    *Plain terms: the part that listens to the robot no longer waits for the disk, locks the
    clock five times faster, and knocks on the robot's door every half second after a drop.*

WHEN THE LINK DROPS
    No bytes for 1.5 s (or the connection closes) = DOWN. While DOWN it publishes NOTHING on
    /robot/* - never a repeat of an old value, which would tell the filter the robot had
    stopped. Reconnects by itself: after 0.5 s, 1 s, then every 2 s, trying the robot
    addresses in the order given. Refuses to start if /use_sim_time is true (the stamps
    would be on a different clock).

VISIBLE FROM A PHONE (ENGINEERING_NOTES.md rules 8 and 13), all on the Jetson:
    <jobs_dir>/<run>_bridge.progress     one line, every 5 s and on every state change
    <jobs_dir>/<run>_bridge_clock.csv    one row per ping (lets the robot's own bag be
                                         converted to Jetson time afterwards)
    <jobs_dir>/<run>_bridge_events.log   connects, drops, locks, steps, refusals, final counts

PARAMETERS (private, ~name)
    robot      REQUIRED  robot address(es), comma-separated, tried in order,
                         e.g. <robot-usb-address>,<robot-wifi-address> (cable first, then WiFi)
    run        REQUIRED  the run name; the robot's sender refuses any other
    port       8111      jobs_dir  ~/jobs
    (clock and gate constants above are parameters too: see Receiver.__init__)

  usage (on the Jetson, from start_drive.sh with FUSION=1):
    setsid nohup python3 $TOOLS_DIR/robot_bridge_recv.py __name:=robot_bridge_recv \\
        _robot:=<robot-wifi-address> _port:=8111 _run:=s2_static_03 \\
        > ~/.run_records/s2_static_03/bridge_recv.log 2>&1 < /dev/null &
"""
from __future__ import print_function

import collections
import json
import os
try:
    import queue
except ImportError:          # Python 2 (never on the Jetson)
    import Queue as queue
import socket
import struct
import threading
import time

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

PROTO = 1
VERSION = "robot_bridge_recv 2026-09-25a"
FRAME = struct.Struct("<cI")
DATA = struct.Struct("<BIq")
PING = struct.Struct("<Iq")
PONG = struct.Struct("<Iqqq")
MAX_FRAME = 4 * 1024 * 1024
MS = 1000000

# robot topic -> (Jetson topic, message class, short name for the progress line)
TOPIC_MAP = collections.OrderedDict([
    ("/husky_velocity_controller/odom", ("/robot/wheel_odom", Odometry, "wheel")),
    ("/imu/data", ("/robot/imu", Imu, "imu")),
    ("/imu/data_raw", ("/robot/imu_raw", Imu, "imu_raw")),
    ("/odometry/filtered", ("/robot/ekf_odom", Odometry, "ekf")),
])


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


def pct(v, p):
    if not v:
        return None
    s = sorted(v)
    k = (len(s) - 1) * p / 100.0
    lo = int(k)
    hi = min(lo + 1, len(s) - 1)
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


class ClockFilter(object):
    """Robot-minus-Jetson clock offset, in integer nanoseconds."""

    def __init__(self, window, lock_pings, lock_bound_ns, step_ns, step_pings, track_ns,
                 agree_tol_ns, rtt_ignore_ns):
        self.window = window
        self.lock_pings = lock_pings
        self.lock_bound_ns = lock_bound_ns
        self.step_ns = step_ns
        self.step_pings = step_pings
        self.track_ns = track_ns
        self.agree_tol_ns = agree_tol_ns
        self.rtt_ignore_ns = rtt_ignore_ns
        self.pings = collections.deque(maxlen=window)    # (theta, delta)
        self.applied = None
        self.locked = False
        self.agree = 0
        self.step_run = []
        self.steps = 0
        self.rtts = collections.deque(maxlen=120)        # last minute of round trips

    def new_connection(self):
        """Re-measure from scratch (the path may have changed); keep the old offset only
        as the thing a quick resume must agree with."""
        self.pings.clear()
        self.locked = False
        self.agree = 0
        self.step_run = []

    def best(self):
        if not self.pings:
            return None, None, None
        theta, delta = min(self.pings, key=lambda p: p[1])
        q = sorted(self.pings, key=lambda p: p[1])[:max(2, len(self.pings) // 2)]
        th = [p[0] for p in q]
        med = pct(th, 50)
        spread = 1.4826 * pct([abs(x - med) for x in th], 50) if len(th) > 1 else 0.0
        return theta, max(delta, 0) // 2, int(spread)

    def add(self, t1, t2, t3, t4):
        """Returns (theta, delta, event) where event is None, LOCK, RELOCK_QUICK or STEP."""
        theta = ((t2 - t1) + (t3 - t4)) // 2
        delta = (t4 - t1) - (t3 - t2)
        self.rtts.append(delta)
        half = max(delta, 0) // 2
        event = None
        if not self.locked:
            self.pings.append((theta, delta))
            if self.applied is not None:                 # a reconnection: quick resume?
                if abs(theta - self.applied) <= half + self.agree_tol_ns:
                    self.agree += 1
                else:
                    self.agree = -10 ** 6                # disagreed once: full lock only
                if self.agree >= 2:
                    self.locked, event = True, "RELOCK_QUICK"
            if not self.locked and len(self.pings) >= self.lock_pings:
                est, bound, _ = self.best()
                if bound <= self.lock_bound_ns:
                    prev = self.applied
                    self.applied, self.locked, event = est, True, "LOCK"
                    if prev is not None and abs(est - prev) > self.step_ns:
                        self.steps += 1
                        event = "LOCK_NEW_OFFSET"
            return theta, delta, event
        if delta > self.rtt_ignore_ns:
            return theta, delta, None                    # too slow to judge anything by
        if abs(theta - self.applied) > self.step_ns + half:
            self.step_run.append((theta, delta))         # kept OUT of the window for now
            if len(self.step_run) >= self.step_pings:
                self.pings.clear()
                self.pings.extend(self.step_run)
                self.applied = min(self.step_run, key=lambda p: p[1])[0]
                self.step_run = []
                self.steps += 1
                event = "STEP"
            return theta, delta, event
        self.step_run = []
        self.pings.append((theta, delta))
        est, _, _ = self.best()
        move = max(-self.track_ns, min(self.track_ns, est - self.applied))
        self.applied += move
        return theta, delta, None


class TopicStats(object):
    def __init__(self):
        self.published = 0
        self.lost = 0
        self.missed_while_down = 0
        self.stale = 0
        self.future = 0
        self.out_of_order = 0
        self.held_locking = 0
        self.refused = 0
        self.last_seq = None          # this connection
        self.seq_before_drop = None
        self.last_pub_ns = None       # whole run: never publish an older or equal stamp
        self.pub_times = collections.deque()
        self.ages = collections.deque(maxlen=500)


class Receiver(object):
    def __init__(self):
        g = lambda k, d: rospy.get_param("~" + k, d)
        self.addrs = [a.strip() for a in str(g("robot", "")).split(",") if a.strip()]
        self.run = str(g("run", ""))
        self.port = int(g("port", 8111))
        self.jobs = os.path.expanduser(str(g("jobs_dir", os.environ.get("JOBS_DIR", "~/jobs"))))
        self.down_after = float(g("down_after_s", 1.5))
        self.ping_period = 1.0 / float(g("ping_hz", 2.0))
        self.lock_ping_period = 1.0 / float(g("lock_ping_hz", 10.0))   # A1: while not locked
        self.retry_s = float(g("retry_s", 0.5))                         # A1: robot unreachable
        self.connect_timeout = float(g("connect_timeout_s", 1.0))       # A1: was 2.0
        self.stall_s = float(g("stall_report_s", 0.5))                  # A1: loop pause worth counting
        self.max_age_ns = int(float(g("max_age_s", 0.3)) * 1e9)
        self.max_future_ns = int(float(g("max_future_s", 0.02)) * 1e9)
        self.clock = ClockFilter(
            window=int(g("clock_window", 16)), lock_pings=int(g("lock_pings", 6)),
            lock_bound_ns=int(float(g("lock_bound_s", 0.02)) * 1e9),
            step_ns=int(float(g("step_s", 0.05)) * 1e9), step_pings=int(g("step_pings", 3)),
            track_ns=int(float(g("track_s_per_ping", 0.0005)) * 1e9),
            agree_tol_ns=int(float(g("agree_tol_s", 0.002)) * 1e9),
            rtt_ignore_ns=int(float(g("rtt_ignore_s", 0.1)) * 1e9))
        if not self.addrs or not self.run:
            raise SystemExit("robot_bridge_recv: _robot:=<address[,address]> and _run:=<run> "
                             "are both required")
        os.makedirs(self.jobs, exist_ok=True)
        base = os.path.join(self.jobs, self.run + "_bridge")
        self.progress_path = base + ".progress"
        self.events = open(base + "_events.log", "a", buffering=1)
        new_csv = not os.path.exists(base + "_clock.csv")
        self.csv = open(base + "_clock.csv", "a", buffering=1)
        if new_csv:
            self.csv.write("t1_ns,t2_ns,t3_ns,t4_ns,theta_ns,delta_ns,estimate_ns,applied_ns,"
                           "bound_ns,spread_ns,state,event\n")
        # A1: every file write and screen print goes through this queue to ONE writer thread,
        # so the thread that reads the link never waits for the disk
        self.ioq = queue.Queue()
        self.io_thread = threading.Thread(target=self.io_loop, daemon=True)
        self.io_thread.start()
        self.loop_stalls = 0
        self.loop_stall_max = 0.0

        self.lock = threading.RLock()
        self.state = "STARTING"
        self.state_since = time.time()
        self.state_why = ""
        self.peer = ""
        self.t_start = time.time()
        self.connects = 0
        self.sender_stats = {}
        self.announced = {}           # topic no -> robot topic (for this connection)
        self.stats = collections.OrderedDict((t, TopicStats()) for t in TOPIC_MAP)
        self.unknown_frames = 0
        self.pubs = {t: rospy.Publisher(v[0], v[1], queue_size=20) for t, v in TOPIC_MAP.items()}
        self.status_pub = rospy.Publisher("/robot_bridge/status", DiagnosticArray, queue_size=5)
        self.sock = None
        self.session_alive = False

    # ------------------------------------------------------------ bookkeeping
    def io_loop(self):
        """A1: the only thread that writes files or prints. Items: (kind, payload)."""
        while True:
            kind, a = self.ioq.get()
            try:
                if kind == "event":
                    self.events.write(a[0] + "\n")
                    log(a[1])
                elif kind == "csv":
                    self.csv.write(a)
                elif kind == "progress":
                    tmp = self.progress_path + ".tmp"
                    with open(tmp, "w") as fh:
                        fh.write(a + "\n")
                    os.replace(tmp, self.progress_path)
                elif kind == "print":
                    log(a)
                elif kind == "flush":
                    a.set()
            except Exception as e:          # noqa - a full or failing disk must never stop the bridge
                try:
                    log("!! cannot write (%s): %s" % (kind, e))
                except Exception:
                    pass
            finally:
                self.ioq.task_done()

    def io_flush(self, timeout=5.0):
        """Wait (at most timeout s) until everything queued so far is written."""
        done = threading.Event()
        self.ioq.put(("flush", done))
        return done.wait(timeout)

    def event(self, what):
        line = "%s %d %s" % (time.strftime("%Y-%m-%d %H:%M:%S"), time.time_ns(), what)
        self.ioq.put(("event", (line, what)))

    def set_state(self, state, why=""):
        with self.lock:
            if state == self.state:
                return
            self.state, self.state_since, self.state_why = state, time.time(), why
        self.event("STATE %s %s" % (state, why))
        self.write_progress()

    def progress_line(self):
        with self.lock:
            el = int(time.time() - self.t_start)
            in_state = int(time.time() - self.state_since)
            now = time.monotonic()
            rates = []
            for t, (_, _, short) in TOPIC_MAP.items():
                s = self.stats[t]
                while s.pub_times and now - s.pub_times[0] > 5.0:
                    s.pub_times.popleft()
                rates.append("%s %.1f Hz" % (short, len(s.pub_times) / 5.0))
            lost = sum(s.lost for s in self.stats.values())
            stale = sum(s.stale for s in self.stats.values())
            fut = sum(s.future for s in self.stats.values())
            off = ("%+.2f+-%.2f ms" % (self.clock.applied / MS,
                                       (self.clock.best()[1] or 0) / MS)
                   if self.clock.applied is not None else "not measured")
            counts = "lost %d  stale %d  future %d  steps %d" % (lost, stale, fut,
                                                                self.clock.steps)
            if self.state == "UP":
                return "BRIDGE UP  offset %s  %s  %s  %ds" % (off, "  ".join(rates), counts, el)
            if self.state in ("LOCKING", "STARTING", "CONNECTING"):
                return ("BRIDGE %s %s for %d s  peer %s  offset %s  %ds"
                        % (self.state, self.state_why, in_state, self.peer or "-", off, el))
            if self.loop_stalls:
                counts += "  stalls %d (max %.1f s)" % (self.loop_stalls, self.loop_stall_max)
            return ("!! BRIDGE DOWN for %d s (%s)  robot %s:%d  last offset %s  %s  %ds"
                    % (in_state, self.state_why, ",".join(self.addrs), self.port, off, counts, el))

    def write_progress(self, line=None):
        line = line or self.progress_line()
        self.ioq.put(("progress", line))          # A1: written by io_loop, never by the caller

    def status_loop(self):
        next_prog = 0.0
        while not rospy.is_shutdown():
            time.sleep(1.0)
            try:
                self.publish_status()
            except Exception as e:           # noqa - status must never kill the bridge
                self.ioq.put(("print", "!! status: %s" % e))
            if time.time() >= next_prog:
                next_prog = time.time() + 5.0
                self.write_progress()

    def publish_status(self):
        with self.lock:
            st = DiagnosticStatus()
            st.name = "robot_bridge"
            st.hardware_id = "robot %s:%d" % (",".join(self.addrs), self.port)
            st.level = {"UP": DiagnosticStatus.OK, "LOCKING": DiagnosticStatus.WARN}.get(
                self.state, DiagnosticStatus.ERROR)
            st.message = self.state
            est, bound, spread = self.clock.best()
            rt = list(self.clock.rtts)
            kv = [("state", self.state), ("state_s", "%.1f" % (time.time() - self.state_since)),
                  ("why", self.state_why), ("peer", self.peer),
                  ("applied_offset_ms", "%.3f" % (self.clock.applied / MS)
                   if self.clock.applied is not None else "nan"),
                  ("bound_ms", "%.3f" % (bound / MS) if bound is not None else "nan"),
                  ("spread_ms", "%.3f" % (spread / MS) if spread is not None else "nan"),
                  ("rtt_min_ms", "%.3f" % (min(rt) / MS) if rt else "nan"),
                  ("rtt_p95_ms", "%.3f" % (pct(rt, 95) / MS) if rt else "nan"),
                  ("steps", str(self.clock.steps)), ("connects", str(self.connects)),
                  ("loop_stalls", str(self.loop_stalls)),
                  ("loop_stall_max_s", "%.2f" % self.loop_stall_max)]
            now = time.monotonic()
            for t, (_, _, short) in TOPIC_MAP.items():
                s = self.stats[t]
                while s.pub_times and now - s.pub_times[0] > 5.0:
                    s.pub_times.popleft()
                a = list(s.ages)
                kv += [(short + "_rate_hz", "%.2f" % (len(s.pub_times) / 5.0)),
                       (short + "_age_p95_ms", "%.2f" % (pct(a, 95) / MS) if a else "nan"),
                       (short + "_published", str(s.published)), (short + "_lost", str(s.lost)),
                       (short + "_stale", str(s.stale)), (short + "_future", str(s.future)),
                       (short + "_out_of_order", str(s.out_of_order)),
                       (short + "_held_locking", str(s.held_locking))]
            for k in ("received", "dropped"):
                for t, n in (self.sender_stats.get(k) or {}).items():
                    short = TOPIC_MAP.get(t, (None, None, t))[2]
                    kv.append(("robot_%s_%s" % (k, short), str(n)))
            st.values = [KeyValue(k, v) for k, v in kv]
        arr = DiagnosticArray()
        arr.header.stamp = rospy.Time.now()
        arr.status = [st]
        self.status_pub.publish(arr)

    def final(self, crashed=None):
        with self.lock:
            summ = {"elapsed_s": round(time.time() - self.t_start, 1), "connects": self.connects,
                    "steps": self.clock.steps,
                    "applied_offset_ns": self.clock.applied,
                    "topics": {t: {k: getattr(s, k) for k in (
                        "published", "lost", "missed_while_down", "stale", "future",
                        "out_of_order", "held_locking", "refused")}
                        for t, s in self.stats.items()},
                    "unknown_frames": self.unknown_frames, "sender_stats": self.sender_stats,
                    "loop_stalls": self.loop_stalls, "loop_stall_max_s": round(self.loop_stall_max, 3)}
        self.event("FINAL " + json.dumps(summ, sort_keys=True))
        tot = sum(v["published"] for v in summ["topics"].values())
        head = ("BRIDGE complete - receiver closed on request" if crashed is None else
                "!! BRIDGE FAILED - the receiver itself crashed (%s)" % crashed)
        self.write_progress("%s after %d s; published %d, lost %d, stale %d, future %d, steps %d"
                            % (head, summ["elapsed_s"], tot,
                               sum(v["lost"] for v in summ["topics"].values()),
                               sum(v["stale"] for v in summ["topics"].values()),
                               sum(v["future"] for v in summ["topics"].values()), summ["steps"]))
        self.io_flush()

    # ------------------------------------------------------------ the link
    def connect(self):
        for a in self.addrs:
            try:
                s = socket.create_connection((a, self.port), timeout=self.connect_timeout)
            except OSError as e:
                with self.lock:
                    self.state_why = "%s:%d %s" % (a, self.port, getattr(e, "strerror", None) or e)
                continue
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            for name, val in (("TCP_KEEPIDLE", 2), ("TCP_KEEPINTVL", 1), ("TCP_KEEPCNT", 3)):
                if hasattr(socket, name):
                    s.setsockopt(socket.IPPROTO_TCP, getattr(socket, name), val)
            return s, a
        return None, None

    def ping_loop(self, s, stop):
        n = 0
        while not stop.is_set() and not rospy.is_shutdown():
            n += 1
            try:
                s.sendall(FRAME.pack(b"P", PING.size) + PING.pack(n & 0xFFFFFFFF, time.time_ns()))
            except OSError:
                return                         # the reader will notice the dead link
            # A1: 10 a second until the clock is locked (6 pings in ~0.5 s), then 2 a second
            stop.wait(self.ping_period if self.clock.locked else self.lock_ping_period)

    def session(self, s, addr):
        """One connection, from hello to the moment the link is judged down. Returns why."""
        s.settimeout(2.0)
        hello = {"proto": PROTO, "run": self.run, "receiver": VERSION + " on " + socket.gethostname()}
        body = json.dumps(hello).encode()
        s.sendall(FRAME.pack(b"H", len(body)) + body)
        buf = bytearray()
        welcome = None
        deadline = time.monotonic() + 2.0
        while welcome is None:
            if time.monotonic() > deadline:
                return "no welcome from the sender within 2 s"
            chunk = s.recv(65536)
            if not chunk:
                return "sender closed the connection during the hello"
            buf.extend(chunk)
            if len(buf) >= FRAME.size:
                kind, n = FRAME.unpack_from(buf, 0)
                if len(buf) >= FRAME.size + n:
                    msg = json.loads(bytes(buf[FRAME.size:FRAME.size + n]).decode())
                    del buf[:FRAME.size + n]
                    if kind == b"E":
                        self.event("REFUSED by the sender: %s" % msg.get("why"))
                        return "refused by the sender: %s" % msg.get("why")
                    if kind != b"W" or msg.get("run") != self.run:
                        return "unexpected first frame %r" % kind
                    welcome = msg
        with self.lock:
            self.connects += 1
            self.peer = "%s:%d" % (addr, self.port)
            self.announced = {}
            for st in self.stats.values():
                st.seq_before_drop = st.last_seq
                st.last_seq = None
            self.clock.new_connection()
        self.event("CONNECT %s:%d sender %s host %s test %s" % (
            addr, self.port, welcome.get("sender"), welcome.get("host"), welcome.get("test")))
        if welcome.get("test"):
            self.event("NOTE the sender is in TEST MODE (fake robot clock, injected faults)")
        self.set_state("LOCKING", "clock")
        stop = threading.Event()
        threading.Thread(target=self.ping_loop, args=(s, stop), daemon=True).start()
        s.settimeout(0.1)
        last_bytes = time.monotonic()
        last_loop = time.monotonic()
        try:
            why = self.drain(buf, time.time_ns())      # frames that came with the welcome
            if why:
                return why
            while not rospy.is_shutdown():
                now_loop = time.monotonic()
                if now_loop - last_loop > self.stall_s:      # A1: this thread itself paused
                    with self.lock:
                        self.loop_stalls += 1
                        self.loop_stall_max = max(self.loop_stall_max, now_loop - last_loop)
                last_loop = now_loop
                try:
                    chunk = s.recv(262144)
                except socket.timeout:
                    if time.monotonic() - last_bytes > self.down_after:
                        return "no bytes for %.1f s" % self.down_after
                    continue
                t_arr = time.time_ns()                 # when these bytes reached the Jetson
                if not chunk:
                    return "the sender closed the connection"
                last_bytes = time.monotonic()
                buf.extend(chunk)
                why = self.drain(buf, t_arr)
                if why:
                    return why
            return "shutting down"
        finally:
            stop.set()

    def drain(self, buf, t_arr):
        """Handle every complete frame in buf; returns a reason to end the session, or None."""
        while len(buf) >= FRAME.size:
            kind, n = FRAME.unpack_from(buf, 0)
            if n > MAX_FRAME:
                return "corrupt stream (frame of %d bytes)" % n
            if len(buf) < FRAME.size + n:
                break
            body = bytes(buf[FRAME.size:FRAME.size + n])
            del buf[:FRAME.size + n]
            why = self.handle(kind, body, t_arr)
            if why:
                return why
        return None

    def handle(self, kind, body, t_arr):
        if kind == b"D":
            self.on_data(body, t_arr)
        elif kind == b"Q":
            self.on_pong(PONG.unpack(body), t_arr)
        elif kind == b"A":
            a = json.loads(body.decode())
            topic, no = a.get("topic"), int(a.get("no"))
            if topic not in TOPIC_MAP:
                self.event("ANNOUNCE ignored (not carried): %s" % topic)
                return None
            cls = TOPIC_MAP[topic][1]
            if a.get("type") != cls._type or a.get("md5") != cls._md5sum:
                self.event("REFUSED topic %s: robot says %s/%s, Jetson has %s/%s" % (
                    topic, a.get("type"), a.get("md5"), cls._type, cls._md5sum))
                return None
            with self.lock:
                self.announced[no] = topic
            self.event("ANNOUNCE %d %s %s %s" % (no, topic, a.get("type"), a.get("md5")))
        elif kind == b"S":
            with self.lock:
                self.sender_stats = json.loads(body.decode())
        elif kind == b"E":
            return "error from the sender: %s" % body.decode(errors="replace")
        else:
            with self.lock:
                self.unknown_frames += 1
        return None

    def on_pong(self, q, t4):
        pno, t1, t2, t3 = q
        with self.lock:
            theta, delta, ev = self.clock.add(t1, t2, t3, t4)
            est, bound, spread = self.clock.best()
            state = self.state
        self.ioq.put(("csv", "%d,%d,%d,%d,%d,%d,%s,%s,%s,%s,%s,%s\n" % (
            t1, t2, t3, t4, theta, delta, est, self.clock.applied, bound, spread, state, ev or "")))
        if ev:
            self.event("%s applied offset %.3f ms (bound %.3f ms, spread %.3f ms, %d pings)" % (
                ev, self.clock.applied / MS, (bound or 0) / MS, (spread or 0) / MS,
                len(self.clock.pings)))
            if ev in ("LOCK", "RELOCK_QUICK", "LOCK_NEW_OFFSET"):
                self.set_state("UP", ev)
            self.write_progress()

    def on_data(self, body, t_arr):
        no, seq, _t_robot_recv = DATA.unpack_from(body, 0)
        with self.lock:
            topic = self.announced.get(no)
            if topic is None:
                self.unknown_frames += 1
                return
            s = self.stats[topic]
            if s.last_seq is None:
                if s.seq_before_drop is not None and seq > s.seq_before_drop:
                    s.missed_while_down += seq - s.seq_before_drop - 1
            elif seq != (s.last_seq + 1) & 0xFFFFFFFF:
                s.lost += (seq - s.last_seq - 1) & 0xFFFFFFFF
            s.last_seq = seq
            if self.state != "UP" or self.clock.applied is None:
                s.held_locking += 1
                return
            applied = self.clock.applied
        cls = TOPIC_MAP[topic][1]
        m = cls()
        m.deserialize(body[DATA.size:])
        robot_ns = m.header.stamp.secs * 1000000000 + m.header.stamp.nsecs
        corr = robot_ns - applied
        age = t_arr - corr
        with self.lock:
            if age < -self.max_future_ns:
                s.future += 1
                return
            if age > self.max_age_ns:
                s.stale += 1
                return
            if s.last_pub_ns is not None and corr <= s.last_pub_ns:
                s.out_of_order += 1
                return
            s.last_pub_ns = corr
            s.published += 1
            s.pub_times.append(time.monotonic())
            s.ages.append(age)
        m.header.stamp = rospy.Time(corr // 1000000000, corr % 1000000000)
        if m.header.frame_id == "odom":
            m.header.frame_id = "robot_odom"
        self.pubs[topic].publish(m)

    def run_forever(self):
        threading.Thread(target=self.status_loop, daemon=True).start()
        waits = [0.5, 1.0, 2.0]          # a sender that REFUSED the hello: 0.5 s, 1 s, then every 2 s
        fails = 0
        self.set_state("CONNECTING", "to %s:%d" % (",".join(self.addrs), self.port))
        while not rospy.is_shutdown():
            s, addr = self.connect()
            if s is None:
                if self.state != "DOWN":
                    self.set_state("DOWN", self.state_why or "no connection")
                # A1: the robot could not be reached at all - knock again every retry_s (0.5 s),
                # so a WiFi that comes back is caught within ~0.5 s (was up to 2 s)
                self.sleep(self.retry_s)
                continue
            self.sock = s
            before = self.connects
            try:
                why = self.session(s, addr)
            except Exception as e:             # noqa - reset, broken pipe, bad frame
                why = "%s: %s" % (type(e).__name__, e)
            try:
                s.close()
            except OSError:
                pass
            self.sock = None
            if rospy.is_shutdown():
                break
            self.set_state("DOWN", why)
            # a session that got as far as the welcome restarts the retry ladder;
            # one refused at the hello keeps climbing it (no hammering every 0.5 s)
            fails = 0 if self.connects > before else fails
            self.sleep(waits[min(fails, len(waits) - 1)])
            fails += 1

    @staticmethod
    def sleep(w):
        t_end = time.time() + w
        while time.time() < t_end and not rospy.is_shutdown():
            time.sleep(0.05)


def main():
    rospy.init_node("robot_bridge_recv", anonymous=False)
    if rospy.get_param("/use_sim_time", False) in (True, "true", "True", 1):
        log("REFUSED: /use_sim_time is true on this master - the stamps would be on a "
            "different clock from the robot's")
        raise SystemExit(2)
    r = Receiver()
    log("%s  run %s  robot %s port %d  pid %d  files %s_bridge.*" % (
        VERSION, r.run, r.addrs, r.port, os.getpid(), os.path.join(r.jobs, r.run)))
    r.event("START %s robot %s port %d pid %d" % (VERSION, ",".join(r.addrs), r.port, os.getpid()))
    r.write_progress("BRIDGE STARTING  robot %s:%d  0s" % (",".join(r.addrs), r.port))

    def on_shutdown():
        s = r.sock
        if s is not None:
            try:
                s.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
    rospy.on_shutdown(on_shutdown)
    crashed = None
    try:
        r.run_forever()
    except Exception as e:                     # noqa - recorded, then re-raised
        crashed = "%s: %s" % (type(e).__name__, e)
        raise
    finally:
        r.final(crashed)
        r.event("EXIT")
        r.io_flush()                           # A1: the writer thread is a daemon - empty it first


if __name__ == "__main__":
    main()
