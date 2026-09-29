#!/usr/bin/env python3
"""robot_storage_agent.py - the robot's half of the Jetson's storage page.

Runs ON THE ROBOT, from its card's tools folder. Started by
`robot_side.sh storage -` (and at the end of `robot_side.sh start <run>`); it is
never started twice (robot_side.sh matches whole names, ENGINEERING_NOTES.md rule 8).

It answers one question over the network - how full are the robot's two disks -
so the storage page on the Jetson (the storage page (STORAGE_PAGE_URL)) can show them
beside the Jetson's own:

    GET /storage.json   ->  {"host", "time", "disks": [ "/" , the card ]}

READ-ONLY. It measures with statvfs (the operating system's own "how big is this
disk, how full is it" question - the same numbers `df` prints) and reads
/proc/mounts (the list of what is attached where). It writes nothing except a
few log lines to stderr, which robot_side.sh sends to ~/jobs/robot_storage_agent.log.
It publishes nothing on ROS and needs no ROS at all. Standard library only
(Python 3.8, the robot's version).

VERSION 4 (2026-09-26):
  - /health.json gains "battery": the robot's battery voltage and how many seconds old it is.
    Read by a CHILD program, robot_battery_reader.py (beside this file), which listens to the
    robot's own /status message (like `rostopic echo`, publishes nothing). This agent only
    reads the child's output lines, in its own thread; a request never waits for it. If no
    reading arrives for 30 s the child is stopped (by its own process number) and started
    again after a pause (5 s, doubling to at most 2 min while it keeps failing), so a restart
    of the robot's ROS heals by itself. Nothing else changes; /storage.json is untouched.
    *Plain terms: a voltage is a reading, not a charge level - it is shown, never judged.*
VERSION 3 (2026-09-25 on the public status page):
  - adds GET /health.json - the robot computer's memory, processor (total and per core, load
    average), graphics-processor use, temperatures, uptime and HOW MANY programs run (never
    their names: a colleague's computer, readable by anyone on the lab WiFi) - measured
    by ONE background thread every 2 s from /proc and /sys, read-only, answered without waiting.
    /storage.json is byte-for-byte the same shape as version 2, so the storage page is untouched.
VERSION 2 (2026-09-25), after an independent check of version 1:
  - Each disk is measured in ITS OWN background thread, every 2 s, and a request is
    answered from the latest measurement without waiting. Version 1 measured inside
    the request while holding a lock, so one hung card (a stuck mount) would have
    blocked every request, and the Jetson's asks (one every 5 s) would have piled up
    as threads on the colleague's computer. Each disk now carries "age_s" - how many
    seconds old its figures are - so the Jetson can say "it may be stuck" instead of
    showing old figures as new.
  - It moves to "/" at start, so it never keeps the card busy (a reporter started
    from inside the card's folder made unmounting the card fail with "target is busy").
  - A broken connection is logged once per kind, not as a full traceback each time
    (the default), on a robot disk that is 95 % full.
Cost: two statvfs calls and one read of /proc/mounts every 2 s.

*Plain terms: it can look at how full the robot's disks are, and nothing else.*

"Mounted" is decided from /proc/mounts, never from "the folder exists": an empty
folder where the card should be would otherwise report the robot's own disk under
the card's name (the bug the Jetson's old page had with its own card, 2026-09-24).
A path that is not a mount point gets NO size figures.

    python3 robot_storage_agent.py                            # port 8113, "/" + the card
    python3 robot_storage_agent.py --port 8193 --card /dev/shm   # a stand-in test elsewhere
"""
import argparse
import errno
import json
import os
import re
import select
import signal
import socket
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

VERSION = 4
DEFAULT_PORT = 8113
DEFAULT_CARD = "/media/administrator/USB Drive"
MEASURE_PERIOD = 2.0         # seconds between measurements of each disk

os.environ["TZ"] = "America/Toronto"      # every logged time is Hamilton time
time.tzset()

_log_lock = threading.Lock()
_logged = set()


def log(msg, once_key=None):
    """One line to stderr. With once_key, a repeating error is logged only once,
    so a fault that recurs every 5 s cannot fill the robot's 95 %-full disk."""
    with _log_lock:
        if once_key is not None:
            if once_key in _logged or len(_logged) > 50:
                return
            _logged.add(once_key)
        sys.stderr.write("%s robot_storage_agent: %s\n"
                         % (time.strftime("%m-%d %H:%M:%S"), msg))
        sys.stderr.flush()


_OCTAL = re.compile(r"\\([0-7]{3})")


def decode_mount_field(s):
    """/proc/mounts writes a space as \\040 (and tab, newline, backslash likewise).
    The robot's card is mounted at '/media/administrator/USB Drive', with a space."""
    return _OCTAL.sub(lambda m: chr(int(m.group(1), 8)), s)


def read_mounts(path="/proc/mounts"):
    out = []
    try:
        with open(path) as f:
            for line in f:
                p = line.split()
                if len(p) >= 4:
                    out.append({"device": decode_mount_field(p[0]),
                                "mountpoint": decode_mount_field(p[1]),
                                "fstype": p[2], "options": p[3].split(",")})
    except OSError as exc:
        log("cannot read %s: %s" % (path, exc), once_key="mounts")
    return out


def disk(key, path, mounts):
    """Size figures for `path` ONLY if something is mounted exactly there."""
    real = os.path.realpath(path)
    d = {"key": key, "path": path, "exists": os.path.isdir(path), "mounted": False}
    entry = None
    for m in mounts:                      # the LAST entry wins: a later mount covers an earlier one
        if m["mountpoint"] == real:
            entry = m
    if entry is None:
        return d
    d.update(mounted=True, device=entry["device"], fstype=entry["fstype"],
             read_only="ro" in entry["options"])
    try:
        st = os.statvfs(real)
    except OSError as exc:                # e.g. a pulled-out stick still listed as mounted
        d["error"] = plain_os_error(exc)
        log("statvfs %s failed: %s" % (path, exc), once_key="statvfs:" + path)
        return d
    d["total"] = st.f_blocks * st.f_frsize
    d["used"] = (st.f_blocks - st.f_bfree) * st.f_frsize
    d["avail"] = st.f_bavail * st.f_frsize        # what an ordinary user can still write (df "Avail")
    return d


_PLAIN_ERRNO = {errno.EIO: "it stopped answering (a read/write error)",
                errno.ENOTCONN: "the program reading it has stopped",
                errno.ENOENT: "the folder is gone",
                errno.EACCES: "no permission to look at it",
                errno.ESTALE: "the attachment has gone stale",
                errno.ENODEV: "the device is gone"}


def plain_os_error(exc):
    """The error in ordinary words, for the Jetson's page; the raw text is logged."""
    return _PLAIN_ERRNO.get(getattr(exc, "errno", None), "it could not be read (%s)"
                            % (getattr(exc, "strerror", None) or type(exc).__name__).lower())


def snapshot(root, card, mounts_file):
    mounts = read_mounts(mounts_file)
    return {"version": VERSION, "host": socket.gethostname(), "time": time.time(),
            "disks": [disk("robot_root", root, mounts), disk("robot_card", card, mounts)]}


class Meter:
    """Measures ONE disk over and over in its own thread, and keeps the latest answer.
    A request never waits for it: if the card's mount hangs, only this thread hangs,
    the other disk keeps being measured, and the card's "age_s" grows for all to see."""

    def __init__(self, key, path, mounts_file, period=MEASURE_PERIOD):
        self.key, self.path, self.mounts_file, self.period = key, path, mounts_file, period
        self.lock = threading.Lock()
        self.result, self.t = None, None
        self.started = time.monotonic()
        threading.Thread(target=self.run, name="measure-" + key, daemon=True).start()

    def run(self):
        while True:
            try:
                r = disk(self.key, self.path, read_mounts(self.mounts_file))
            except Exception as exc:      # never let a measuring thread die silently
                r = {"key": self.key, "path": self.path, "error": "the reporter could not "
                     "measure it (%s)" % type(exc).__name__}
                log("measuring %s failed: %r" % (self.path, exc),
                    once_key="measure:%s:%s" % (self.key, type(exc).__name__))
            with self.lock:
                self.result, self.t = r, time.monotonic()
            time.sleep(self.period)

    def current(self):
        with self.lock:
            r, t = self.result, self.t
        now = time.monotonic()
        if r is None:                     # not measured yet (or the first measurement hangs)
            return {"key": self.key, "path": self.path, "measuring": True,
                    "age_s": round(now - self.started, 1)}
        out = dict(r)
        out["age_s"] = round(now - t, 1)
        return out


# ---- VERSION 3: the robot computer's health (/health.json) ----------------------------
TICK = os.sysconf("SC_CLK_TCK")
PAGE = os.sysconf("SC_PAGE_SIZE")


def _read(path):
    with open(path) as f:
        return f.read()


HEALTH_PERIOD = 5.0          # every 5 s is plenty for a status page

class Health:
    """One thread, every HEALTH_PERIOD s: memory, processor per core, load, graphics
    processor, temperatures, program count. Read-only (/proc and /sys only)."""

    def __init__(self, period=HEALTH_PERIOD):
        self.period = period
        self.lock = threading.Lock()
        self.result, self.t = None, None
        self.prev_cpu, self.prev_pids, self.prev_t = None, {}, None
        self.started = time.monotonic()
        threading.Thread(target=self.run, name="health", daemon=True).start()

    def run(self):
        while True:
            try:
                r = self.sample()
            except Exception as exc:
                r = {"error": "the reporter could not measure (%s)" % type(exc).__name__}
                log("health sample failed: %r" % exc, once_key="health:" + type(exc).__name__)
            with self.lock:
                self.result, self.t = r, time.monotonic()
            time.sleep(self.period)

    def sample(self):
        out = {}
        m = {}
        for line in _read("/proc/meminfo").splitlines():
            k, v = line.split(":", 1)
            m[k] = int(v.split()[0]) * 1024
        total, avail = m["MemTotal"], m["MemAvailable"]
        out["memory"] = {"total": total, "avail": avail, "used": total - avail,
                         "avail_pct": round(100.0 * avail / total, 1),
                         "swap_total": m.get("SwapTotal", 0),
                         "swap_used": m.get("SwapTotal", 0) - m.get("SwapFree", 0)}
        cpu = {}
        for line in _read("/proc/stat").splitlines():
            if not line.startswith("cpu"):
                break
            f = line.split()
            v = list(map(int, f[1:]))
            cpu[f[0]] = (sum(v), v[3] + (v[4] if len(v) > 4 else 0))
        la = _read("/proc/loadavg").split()
        c = {"cores": len(cpu) - 1, "load1": float(la[0]), "load5": float(la[1]),
             "total_pct": None, "per_core": []}
        prev = self.prev_cpu
        self.prev_cpu = cpu
        if prev:
            def pct(k):
                if k not in prev:
                    return None
                dt, di = cpu[k][0] - prev[k][0], cpu[k][1] - prev[k][1]
                return round(100.0 * (dt - di) / dt, 1) if dt > 0 else 0.0
            c["total_pct"] = pct("cpu")
            c["per_core"] = [pct("cpu%d" % i) for i in range(c["cores"])]
        out["cpu"] = c
        g = {"pct": None, "error": "no GPU load file"}
        for p in ("/sys/devices/gpu.0/load", "/sys/devices/platform/gpu.0/load"):
            try:
                g = {"pct": round(int(_read(p).strip()) / 10.0, 1)}
                break
            except (OSError, ValueError):
                continue
        out["gpu"] = g
        temps = []
        for z in sorted(os.listdir("/sys/class/thermal")):
            if not z.startswith("thermal_zone"):
                continue
            try:
                t = int(_read("/sys/class/thermal/%s/temp" % z).strip()) / 1000.0
                name = _read("/sys/class/thermal/%s/type" % z).strip()
            except (OSError, ValueError):
                continue
            if -40 < t < 150:
                temps.append({"name": name, "c": round(t, 1)})
        out["temps"] = temps
        # only a COUNT of programs - never their names or process numbers: this is a colleague's
        # computer and /health.json is readable by anyone on the lab WiFi (second review, defect 5)
        out["top"] = {"count": sum(1 for d in os.listdir("/proc") if d.isdigit())}
        try:
            out["uptime_s"] = round(float(_read("/proc/uptime").split()[0]))
        except (OSError, ValueError):
            pass
        return out

    def current(self):
        with self.lock:
            r, t = self.result, self.t
        now = time.monotonic()
        if r is None:
            return {"measuring": True, "age_s": round(now - self.started, 1)}
        out = dict(r)
        out["age_s"] = round(now - t, 1)
        return out


# ---- VERSION 4: the robot's battery voltage, from the child robot_battery_reader.py --------
BATTERY_READER = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot_battery_reader.py")
BATTERY_STALE_S = 30.0       # no reading for this long (1 a second is normal): restart the child
BATTERY_FIRST_S = 60.0       # ... allowing longer for its first one (importing ROS takes a few s)
BATTERY_ERR = "/dev/shm/slam_battery_reader.err"   # the child's error text: memory, not the full disk


class Battery:
    """Keeps the latest battery voltage from the child program. One thread; never blocks
    a request. Reports the voltage and its age, or why there is none - never a judgement."""

    def __init__(self, script=BATTERY_READER):
        self.script = script
        self.lock = threading.Lock()
        self.volts, self.t = None, None           # latest reading, monotonic time it arrived
        self.state = "starting the reader"
        self.starts = 0
        threading.Thread(target=self.run, name="battery", daemon=True).start()

    def _set_state(self, s):
        with self.lock:
            self.state = s

    def run(self):
        pause = 5.0
        while True:
            got = False
            try:
                got = self.run_child()
            except Exception as exc:              # never let this thread die silently
                self._set_state("the reader failed (%s)" % type(exc).__name__)
                log("battery reader failed: %r" % exc, once_key="battery:" + type(exc).__name__)
            pause = 5.0 if got else min(120.0, pause * 2)
            time.sleep(pause)

    def run_child(self):
        """Runs the child once, until it exits or goes quiet. True if it gave any reading."""
        if not os.path.isfile(self.script):
            self._set_state("robot_battery_reader.py is missing beside the agent")
            return False
        with self.lock:
            self.starts += 1
            self.state = "reader started, waiting for the first reading"
        try:
            err = open(BATTERY_ERR, "w")
        except OSError:
            err = subprocess.DEVNULL
        try:
            # ROS's own settings first, exactly as a terminal gets them (the agent itself is started
            # from cron at boot with none); `exec` makes the child python3 keep bash's process number
            p = subprocess.Popen(["/bin/bash", "-c", 'source /opt/ros/noetic/setup.bash >/dev/null 2>&1; '
                                  'exec "$1" "$2"', "bash", sys.executable, self.script],
                                 stdin=subprocess.DEVNULL,
                                 stdout=subprocess.PIPE, stderr=err, cwd="/")
        finally:
            if err is not subprocess.DEVNULL:
                err.close()
        fd = p.stdout.fileno()
        buf, got = b"", False
        last = time.monotonic()
        limit = BATTERY_FIRST_S
        try:
            while True:
                r, _, _ = select.select([fd], [], [], 5.0)
                if r:
                    chunk = os.read(fd, 4096)
                    if not chunk:
                        self._set_state("the reader stopped (exit code %s)" % p.poll())
                        break
                    buf = (buf + chunk)[-4096:]
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        try:
                            v = float(line)
                        except ValueError:
                            continue
                        if v == v and 0.0 <= v <= 100.0:
                            now = time.monotonic()
                            with self.lock:
                                self.volts, self.t = v, now
                                self.state = "reading"
                            last, got, limit = now, True, BATTERY_STALE_S
                if time.monotonic() - last > limit:
                    self._set_state("no reading from the robot's /status for %.0f s - restarting the "
                                    "reader (is the robot's ROS running?)" % limit)
                    log("battery: no reading for %.0f s, restarting the reader" % limit,
                        once_key="battery:quiet")
                    break
        finally:
            if p.poll() is None:
                p.terminate()                     # our own child, by its process handle
                try:
                    p.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    p.kill()
                    p.wait(timeout=5)
            p.stdout.close()
        return got

    def current(self):
        with self.lock:
            v, t, state, starts = self.volts, self.t, self.state, self.starts
        out = {"source": "/status battery_voltage", "state": state, "reader_starts": starts}
        if v is not None:
            out["volts"] = round(v, 3)
            out["age_s"] = round(time.monotonic() - t, 1)
        return out


class Server(ThreadingHTTPServer):
    daemon_threads = True

    def handle_error(self, request, client_address):
        """The default prints a full traceback for every broken connection (e.g. the
        Jetson giving up after its 1 s limit). Once per kind, through log()."""
        exc = sys.exc_info()[1]
        kind = type(exc).__name__
        log("a request from %s failed: %s: %s (logged once per kind)"
            % (client_address[0], kind, exc), once_key="request:" + kind)


def make_handler(meters, host, health=None, battery=None):
    class Handler(BaseHTTPRequestHandler):
        timeout = 5                       # a connection that never sends a request is closed

        def log_message(self, fmt, *args):
            pass                          # asked every 5 s - never log each request

        def do_GET(self):
            path = self.path.split("?")[0]
            if path == "/health.json" and health is not None:
                body = json.dumps({"version": VERSION, "host": host, "time": time.time(),
                                   "period_s": HEALTH_PERIOD,
                                   "health": health.current(),
                                   "battery": battery.current() if battery is not None else None}).encode()
            elif path == "/storage.json":
                body = json.dumps({"version": VERSION, "host": host, "time": time.time(),
                                   "period_s": MEASURE_PERIOD,
                                   "disks": [m.current() for m in meters]}).encode()
            else:
                self.send_response(404)
                self.end_headers()
                return
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    return Handler


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--root", default="/", help="the robot's own disk (default /)")
    ap.add_argument("--card", default=DEFAULT_CARD, help="the robot's card (default %(default)s)")
    ap.add_argument("--mounts-file", default="/proc/mounts", help=argparse.SUPPRESS)
    ap.add_argument("--once", action="store_true", help="print one answer and exit")
    a = ap.parse_args()
    if a.once:
        print(json.dumps(snapshot(a.root, a.card, a.mounts_file), indent=1))
        return
    # Paths made absolute BEFORE moving to "/", so a relative test path still means the same.
    a.root, a.card, a.mounts_file = (os.path.abspath(x) for x in (a.root, a.card, a.mounts_file))
    try:
        os.chdir("/")                     # never keep the card (or any folder) busy
    except OSError as exc:
        log("could not move to /: %s" % exc)
    meters = [Meter("robot_root", a.root, a.mounts_file), Meter("robot_card", a.card, a.mounts_file)]
    try:
        srv = Server(("0.0.0.0", a.port), make_handler(meters, socket.gethostname(), Health(), Battery()))
    except OSError as exc:
        log("cannot listen on port %d: %s - is another copy already running?" % (a.port, exc))
        sys.exit(1)

    def stop(signum, frame):
        log("stopping on signal %d" % signum)
        threading.Thread(target=srv.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    log("serving /storage.json and /health.json on port %d (pid %d): '%s' and '%s'"
        % (a.port, os.getpid(), a.root, a.card))
    srv.serve_forever()
    srv.server_close()
    log("stopped")


if __name__ == "__main__":
    main()
