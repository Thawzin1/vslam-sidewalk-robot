#!/usr/bin/env python3
"""Jetson health for the public live-map page (slam_live_push.py adds it to every push).

What it reports, all read from the Jetson itself, nothing changed:
  memory   - used / available / swap (on this Jetson: zram, compressed memory inside RAM), and the
             out-of-memory guard's line (earlyoom kills the mapping program FIRST when available
             memory AND free swap are both under 15 %: /etc/default/earlyoom -m 15 -s 15)
  cpu      - total and per-core use since the previous call (from /proc/stat), load average
  gpu      - graphics-processor use (the Orin's /sys/devices/gpu.0/load, per mille)
  temps    - every thermal zone the kernel lists, degrees C
  top      - the 6 programs using the most memory and the 6 using the most processor
  robot    - the robot computer's own health (memory, processor, temperatures, program COUNT)
             from its reporter robot_storage_agent.py version 3, GET :8113/health.json - asked
             by a background thread, so a robot that is off never slows a push down;
             since version 4 (2026-09-26) also the robot's BATTERY VOLTAGE (a reading, not a charge
             level - shown, never judged) with its age, the last good reading, and when the robot
             last answered at all (so the page can say "robot offline - last seen HH:MM")
  storage  - the storage page's own answer (http://127.0.0.1:8092/api/storage: Jetson disk,
             USB stick, microSD card, robot card) - or, if that page is down, a plain disk
             reading of / and every mount under /media/sidewalk

Cheap by design: one pass over /proc every call (~2 s), no subprocesses, every part guarded so
one failure never stops the others. Run it alone to print one sample:
    python3 jetson_status.py
"""
import glob
import json
import os
import threading
import time
import urllib.error
import urllib.request

EARLYOOM_PCT = 15.0          # /etc/default/earlyoom on this Jetson: -m 15 -s 15 (rule 23): it acts
                             # only when BOTH memory and swap are under 15 % free
STORAGE_URL = "http://127.0.0.1:8092/api/storage"
# the robot, as the drive scripts address it (start_drive.sh: ROBOT_ADDRS, comma list)
ROBOT_ADDRS = [a.strip() for a in os.environ.get(
    "ROBOT_ADDRS", os.environ.get("PEER_HOST", os.environ.get("ROBOT_WIFI_ADDR", ""))).split(",") if a.strip()]
ROBOT_PORT = 8113
# the storage page's address file (storage_monitor.py re-reads it every cycle: the robot's WiFi
# address comes from the router and can change); read here on every ask for the same reason
ROBOT_ADDR_FILE = os.path.expanduser("~/storage_robot_addrs.txt")
MAX_ANSWER = 65536           # a health answer is ~3 kB; anything larger is refused, never forwarded


def robot_addrs():
    try:
        with open(ROBOT_ADDR_FILE) as f:
            addrs = [a.strip() for a in f.read().replace(",", " ").split() if a.strip()
                     and not a.strip().startswith("#")]
        if addrs:
            return addrs
    except OSError:
        pass
    return ROBOT_ADDRS


def _num(v, lo=None, hi=None):
    """A finite number within limits, else None - nothing from the network reaches the page
    unless it is a plain number or a short plain name."""
    if isinstance(v, bool) or not isinstance(v, (int, float)) or v != v or v in (float("inf"), float("-inf")):
        return None
    if (lo is not None and v < lo) or (hi is not None and v > hi):
        return None
    return v


def _name(v, n=40):
    return v[:n] if isinstance(v, str) else None


def clean_health(h):
    """Rebuild the robot's health report from known fields only (review defects 2 and 4).
    The robot's busiest-program NAMES are not forwarded: it is a colleague's computer
    (ENGINEERING_NOTES.md rule 18) and the site is shared by link - only counts and totals leave it."""
    if not isinstance(h, dict):
        return {"error": "the robot's health report was not understood"}
    out = {"age_s": _num(h.get("age_s"), 0), "uptime_s": _num(h.get("uptime_s"), 0)}
    if h.get("measuring"):
        out["measuring"] = True
    if isinstance(h.get("error"), str):
        out["error"] = h["error"][:120]
    m = h.get("memory")
    if isinstance(m, dict):
        out["memory"] = {k: _num(m.get(k), 0) for k in
                         ("total", "avail", "used", "avail_pct", "swap_total", "swap_used")}
    c = h.get("cpu")
    if isinstance(c, dict):
        pc = c.get("per_core")
        out["cpu"] = {"cores": _num(c.get("cores"), 1, 256), "load1": _num(c.get("load1"), 0),
                      "load5": _num(c.get("load5"), 0), "total_pct": _num(c.get("total_pct"), 0, 100),
                      "per_core": [_num(x, 0, 100) for x in pc[:256]] if isinstance(pc, list) else []}
    g = h.get("gpu")
    if isinstance(g, dict):
        out["gpu"] = {"pct": _num(g.get("pct"), 0, 100)}
    t = h.get("temps")
    if isinstance(t, list):
        out["temps"] = [{"name": _name(x.get("name")), "c": _num(x.get("c"), -40, 150)}
                        for x in t[:32] if isinstance(x, dict)]
    tp = h.get("top")
    if isinstance(tp, dict):
        out["top"] = {"count": _num(tp.get("count"), 0)}
    return out


BATTERY_FRESH_S = 30.0       # a reading older than this is kept only as "the last reading", never current


def clean_battery(b):
    """The robot's battery block, known fields only (same rule as clean_health). None if absent."""
    if not isinstance(b, dict):
        return None
    return {"volts": _num(b.get("volts"), 0, 100), "age_s": _num(b.get("age_s"), 0),
            "state": _name(b.get("state"), 160), "reader_starts": _num(b.get("reader_starts"), 0)}


TICK = os.sysconf("SC_CLK_TCK")
PAGE = os.sysconf("SC_PAGE_SIZE")

_prev = {"cpu": None, "pids": {}, "t": None}


def _read(path):
    with open(path) as f:
        return f.read()


def memory():
    m = {}
    for line in _read("/proc/meminfo").splitlines():
        k, v = line.split(":", 1)
        m[k] = int(v.split()[0]) * 1024
    total, avail = m["MemTotal"], m["MemAvailable"]
    return {
        "total": total, "avail": avail, "used": total - avail,
        "avail_pct": round(100.0 * avail / total, 1),
        "swap_total": m.get("SwapTotal", 0),
        "swap_used": m.get("SwapTotal", 0) - m.get("SwapFree", 0),
        "earlyoom_pct": EARLYOOM_PCT,
        "earlyoom_avail": int(total * EARLYOOM_PCT / 100.0),
    }


def _cpu_times():
    out = {}
    for line in _read("/proc/stat").splitlines():
        if not line.startswith("cpu"):
            break
        f = line.split()
        v = list(map(int, f[1:]))
        idle = v[3] + (v[4] if len(v) > 4 else 0)
        out[f[0]] = (sum(v), idle)
    return out


def cpu():
    now = _cpu_times()
    prev = _prev["cpu"]
    _prev["cpu"] = now
    la = _read("/proc/loadavg").split()
    res = {"cores": len(now) - 1, "load1": float(la[0]), "load5": float(la[1]),
           "total_pct": None, "per_core": []}
    if prev:
        def pct(k):
            if k not in prev:
                return None                        # a core came online since last time
            dt = now[k][0] - prev[k][0]
            di = now[k][1] - prev[k][1]
            return round(100.0 * (dt - di) / dt, 1) if dt > 0 else 0.0
        res["total_pct"] = pct("cpu")
        res["per_core"] = [pct("cpu%d" % i) for i in range(res["cores"])]
    return res


def gpu():
    for p in ("/sys/devices/gpu.0/load", "/sys/devices/platform/gpu.0/load"):
        try:
            return {"pct": round(int(_read(p).strip()) / 10.0, 1)}
        except (OSError, ValueError):
            continue
    return {"pct": None, "error": "no GPU load file"}


def temps():
    out = []
    for z in sorted(glob.glob("/sys/class/thermal/thermal_zone*")):
        try:
            t = int(_read(z + "/temp").strip()) / 1000.0
            name = _read(z + "/type").strip()
        except (OSError, ValueError):
            continue
        if -40 < t < 150:                          # some zones report a placeholder value
            out.append({"name": name, "c": round(t, 1)})
    return out


def top_programs(n=6):
    """Most memory and most processor, by program. Processor share is over the time since the
    previous call, as % of ONE core (so 250 % = two and a half cores busy)."""
    now_t = time.time()
    prev_t = _prev["t"]
    _prev["t"] = now_t
    seen = {}
    rows = []
    for d in os.listdir("/proc"):
        if not d.isdigit():
            continue
        try:
            st = _read("/proc/%s/stat" % d)
            rss = int(_read("/proc/%s/statm" % d).split()[1]) * PAGE
        except (OSError, ValueError, IndexError):
            continue
        comm = st[st.find("(") + 1:st.rfind(")")]
        if comm in ("python3", "python", "bash", "sh"):
            # "python3" says nothing: name the script it runs instead (first .py/.sh argument)
            try:
                args = _read("/proc/%s/cmdline" % d).split("\0")
                script = next((a for a in args[1:] if a.endswith((".py", ".sh"))), None)
                if script:
                    comm = os.path.basename(script)
            except OSError:
                pass
        f = st[st.rfind(")") + 2:].split()
        ticks = int(f[11]) + int(f[12])            # utime + stime
        seen[d] = ticks
        cpu_pct = None
        if prev_t and d in _prev["pids"]:
            dt = now_t - prev_t
            if dt > 0:
                cpu_pct = round(100.0 * (ticks - _prev["pids"][d]) / TICK / dt, 1)
        rows.append({"pid": int(d), "name": comm, "rss": rss, "cpu_pct": cpu_pct})
    _prev["pids"] = seen
    by_mem = sorted(rows, key=lambda r: -r["rss"])[:n]
    by_cpu = sorted([r for r in rows if r["cpu_pct"]], key=lambda r: -r["cpu_pct"])[:n]
    return {"by_mem": by_mem, "by_cpu": by_cpu, "count": len(rows)}


def _statvfs(path):
    s = os.statvfs(path)
    total = s.f_blocks * s.f_frsize
    avail = s.f_bavail * s.f_frsize
    return total, total - s.f_bfree * s.f_frsize, avail


class _Latest:
    """Runs fn every period s in its own thread; sample() only reads the latest answer, so a
    slow storage page never delays a map push (review defect 5)."""

    def __init__(self, fn, period=2.0):
        self.fn, self.period = fn, period
        self.lock = threading.Lock()
        self.value, self.at, self.started = None, None, False

    def run(self):
        while True:
            try:
                v = self.fn()
            except Exception as exc:
                v = {"ok": False, "error": "%s: %s" % (type(exc).__name__, exc), "devices": []}
            with self.lock:
                self.value, self.at = v, time.time()
            time.sleep(self.period)

    def current(self):
        if not self.started:
            self.started = True
            threading.Thread(target=self.run, name="storage", daemon=True).start()
            self.value, self.at = self.fn(), time.time()     # the first answer, once
        with self.lock:
            out = dict(self.value)
            out["asked_at"] = round(self.at, 3)
        return out


def storage():
    """Prefer the storage page's own judgement (it knows the card's history and the drive
    floor); fall back to plain disk readings, and say which one this is."""
    try:
        with urllib.request.urlopen(STORAGE_URL, timeout=1.5) as r:
            j = json.loads(r.read().decode("utf-8"))
        devs = []
        for d in j.get("devices", []):
            e = {k: d.get(k) for k in (
                "key", "machine", "title", "what", "state", "mountpoint", "level",
                "total", "used", "avail", "headline", "status", "age_s")}
            # a status that ends "...:" introduces a command held separately - add it, so
            # the sentence does not dangle (review defect 9)
            cmds = [c for c in (d.get("commands") or []) if isinstance(c, str)]
            if cmds:
                e["commands"] = cmds[:3]
            devs.append(e)
        rob = j.get("robot", {}) or {}
        return {"source": "storage page (port 8092)", "ok": True,
                "verdict": j.get("verdict"),
                "robot": {"online": rob.get("online"), "header": rob.get("header"),
                          "status": rob.get("status"),
                          "commands": [c for c in (rob.get("commands") or []) if isinstance(c, str)][:3]},
                "devices": devs}
    except Exception as exc:                       # page down: read the disks directly
        devs = []
        mounts = ["/"] + sorted(glob.glob("/media/sidewalk/*"))
        for mp in mounts:
            try:
                if mp != "/" and not os.path.ismount(mp):
                    continue
                total, used, avail = _statvfs(mp)
            except OSError:
                continue
            devs.append({"key": mp, "machine": "jetson", "mountpoint": mp, "state": "mounted",
                         "title": "Jetson internal disk" if mp == "/" else
                         "Jetson storage " + os.path.basename(mp),
                         "total": total, "used": used, "avail": avail, "level": None})
        return {"source": "direct disk reading (storage page not answering: %s)"
                          % type(exc).__name__, "ok": False, "devices": devs}


class RobotHealth:
    """Asks the robot's reporter every 2 s in its own thread; sample() only reads the latest.
    Says WHY there is no answer: refused = the robot is on but its reporter is not running
    (start it with robot_side.sh storage -); timeout/unreachable = robot off or out of WiFi."""

    def __init__(self, period=2.0):
        self.period = period
        self.lock = threading.Lock()
        self.last = {"online": False, "why": "not asked yet"}
        self.last_ok = None
        self.last_seen = None        # last time the robot's reporter answered at all (Jetson clock)
        self.battery_last = None     # {"volts", "at"}: the newest fresh reading, kept when the robot goes quiet
        self.started = False

    def start(self):
        if not self.started:
            self.started = True
            threading.Thread(target=self.run, name="robot-health", daemon=True).start()

    def ask(self):
        why = "no address"
        for a in robot_addrs():
            url = "http://%s:%d/health.json" % ("[%s]" % a if ":" in a else a, ROBOT_PORT)
            try:
                t0 = time.monotonic()
                with urllib.request.urlopen(url, timeout=1.0) as r:
                    raw = r.read(MAX_ANSWER + 1)
                if len(raw) > MAX_ANSWER:
                    return {"online": True, "addr": a, "why": "the robot's answer was too large "
                            "(over %d kB) and was not used" % (MAX_ANSWER // 1024)}
                if time.monotonic() - t0 > 3.0:
                    return {"online": True, "addr": a, "why": "the robot's reporter answered too slowly"}
                j = json.loads(raw.decode("utf-8"))
                if not isinstance(j, dict) or "health" not in j:
                    return {"online": True, "addr": a, "why": "the robot's reporter is an older "
                            "version with no health report (needs version 3)"}
                return {"online": True, "addr": a, "host": _name(j.get("host")),
                        "version": _num(j.get("version")), "period_s": _num(j.get("period_s"), 0),
                        "health": clean_health(j["health"]),
                        "battery": clean_battery(j.get("battery"))}
            except urllib.error.HTTPError as e:
                why = ("the robot's reporter is an older version with no health report "
                       "(needs version 3)") if e.code == 404 else "HTTP %d" % e.code
                return {"online": True, "addr": a, "why": why}
            except Exception as exc:
                r = getattr(exc, "reason", exc)
                if isinstance(exc, ValueError):          # includes a JSON decode error
                    return {"online": True, "addr": a, "why": "the robot answered, but the answer "
                            "was not understood"}
                if isinstance(r, ConnectionRefusedError):
                    return {"online": True, "addr": a, "why": "robot is on, but its reporter "
                            "is not running (on the robot: robot_side.sh storage -)"}
                elif "timed out" in str(r) or isinstance(r, TimeoutError):
                    why = "no answer within 1 s - probably off, charging or out of WiFi"
                else:
                    why = "cannot reach it (%s)" % type(r).__name__
        return {"online": False, "why": why}

    def run(self):
        while True:
            r = self.ask()
            r["asked_at"] = round(time.time(), 3)
            with self.lock:
                if isinstance(r.get("health"), dict) and r["health"].get("memory"):
                    self.last_ok = r["asked_at"]       # only a report with real figures counts
                if r.get("host") is not None:
                    self.last_seen = r["asked_at"]
                b = r.get("battery")
                if (b and b.get("volts") is not None and b.get("age_s") is not None
                        and b["age_s"] <= BATTERY_FRESH_S):
                    # when it was read, on the Jetson's clock (the robot's own clock is not trusted:
                    # it booted at "06-17" this morning) - so only the robot's AGE figure is used
                    self.battery_last = {"volts": b["volts"], "at": round(r["asked_at"] - b["age_s"], 1)}
                self.last = r
            time.sleep(self.period)

    def current(self):
        self.start()
        with self.lock:
            out = dict(self.last)
            out["last_ok"] = self.last_ok
            out["last_seen"] = self.last_seen
            out["battery_last"] = dict(self.battery_last) if self.battery_last else None
        return out


_robot = RobotHealth()
_storage = _Latest(storage)


def sample():
    out = {"at": round(time.time(), 3), "host": os.uname().nodename}
    for name, fn in (("memory", memory), ("cpu", cpu), ("gpu", gpu), ("temps", temps),
                     ("top", top_programs), ("storage", _storage.current),
                     ("robot", _robot.current)):
        try:
            out[name] = fn()
        except Exception as exc:                   # one broken part never hides the rest
            out[name] = {"error": "%s: %s" % (type(exc).__name__, exc)}
    try:
        out["uptime_s"] = round(float(_read("/proc/uptime").split()[0]))
    except (OSError, ValueError):
        pass
    return out


if __name__ == "__main__":
    sample()
    time.sleep(2.5)
    print(json.dumps(sample(), indent=1))
