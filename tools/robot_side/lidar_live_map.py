#!/usr/bin/env python3
"""lidar_live_map.py - the LiDAR's view of the drive, live, as one small picture.

RUNS ON THE ROBOT, on its live ROS master, alongside the LiDAR recording.
Serves http://<robot>:8095/map.png and /stats.json; the Jetson's live map page
(live_map_server.py ~peer) shows it beside the camera's map.

NEEDS odom_debias.py IN THE SAME FOLDER (copy both when deploying). That file
holds the drift maths; this one only calls it, so the live view and the
offline tool can never disagree about the arithmetic.

WHAT IT IS - AND WHAT IT IS NOT
    Each Helios scan (/helios/points, 57,600 points, 10 a second - it takes 2 a
    second) is placed using the robot's own wheel+IMU position (tf
    odom -> base_link, from its EKF - extended Kalman filter, the program that
    blends the wheels with the IMU), looked up at the SCAN'S OWN TIME STAMP.
    Points 0.15-2.0 m above the floor are walls; points near the floor mark
    floor the LiDAR has seen. Nothing is corrected by loop closures.

    *Plain terms: it draws what the laser sees, wherever the wheels say the
    robot was at the moment of that scan.*

TWO CORRECTIONS (occupancy report, section 4.1, rank 1)
    1. Pose at the scan's own time. The first version used the LATEST pose
       (rospy.Time(0)), which can be up to half a second newer than the scan:
       about 0.2 m of driving, or 15 degrees in a fast turn, smeared into the
       walls. If the scan's own time cannot be looked up, it falls back to the
       latest pose and counts it ("used latest pose" in /stats.json).
    2. The gyroscope's parked drift. The robot's heading turns by itself at
       2.7-5.0 degrees a minute, even parked (drives 1 and 2). While the robot
       stands on the start mark - the drive procedure asks for at least 60 s -
       this measures that rate (a straight line through heading against time),
       then subtracts rate x elapsed time from every heading and re-adds the
       position steps. /stats.json shows "drift -x.xx deg/min over N s parked".
       Until it is measured the picture is drawn UNCORRECTED and says so; when
       it is first measured (robot still parked) the picture restarts once, so
       the uncorrected first minute does not leave a smeared fan of walls.
       If the robot drives off before 60 s, the whole drive stays uncorrected
       and /stats.json says why.
    No TF frame and no topic is published - both corrections live inside this
    picture only (a new TF frame above `odom` would give `odom` two parents
    once the LiDAR reference runs: report section 2.1 item 9).

    It is a VIEW, for watching the drive. The LiDAR reference used for every
    number is made AFTER the drive, by replaying the recording.

A FROZEN PICTURE NEVER LOOKS HEALTHY (added 2026-09-24 after review)
    All the work happens in one background thread. Before this change, any error
    in it - a malformed cloud, or the drift maths given a parked minute with only
    two poses - ended that thread silently: the page kept serving the last
    picture, and /stats.json kept its last, healthy-looking lines. Now an error in
    one scan is caught, counted and shown ("processing errors", with the last
    one), and the thread carries on with the next scan. /stats.json also shows
    "last scan drawn: N s ago", and "worker: STOPPED" if the thread has ended
    anyway, so a frozen view can always be told from a parked robot.

WHY NOT RUN THE COLLEAGUE'S LiDAR MAPPING LIVE INSTEAD
    Its settings store the floor as obstacle (Grid/GroundIsObstacle=true), so
    its live grid map would draw as one black blob, and running it live would
    put a second SLAM system on the computer that drives the wheels. This reads
    two topics and does arithmetic; it changes nothing of theirs.

ALIGNED WITH THE CAMERA MAP BY CONSTRUCTION
    Positions are expressed relative to where the robot stood when this
    started - the start mark - facing +x. The camera map's frame also begins at
    its first node, on the same mark, facing +x. The drift correction counts
    from that same first pose, so it keeps the start heading.

POSE LOG (for registered test D3)
    Every placed scan appends one line to ~/jobs/<run>_lidar_live_poses.csv:
    scan stamp, pose time, whether the scan's own time was used, the raw and
    the corrected pose, and the rate. `odom_debias.py wheel.tum --compare-live
    <that file>` then measures how far the live poses are from the offline tool's.

  usage (on the robot, detached):
    setsid nohup python3 lidar_live_map.py _run:=s2_static_03 > ~/jobs/lidar_live.log 2>&1 &
  parameters: ~run, ~port (8095), ~scale (2), ~cloud (/helios/points),
              ~min_parked_s (60: shortest parked start the drift is measured on),
              ~pose_log (default ~/jobs/<run>_lidar_live_poses.csv; "" = none)
"""
import os
# One thread for the maths library. Left to itself numpy's BLAS spreads each
# small matrix product over every core and spins them: the first test took
for _v in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")

import collections
import json
import math
import sys
import threading
import time
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    from odom_debias import LiveDebias, correction_se2
except ImportError as _e:   # deployment mistake: say exactly what is missing, then stop
    sys.exit("lidar_live_map.py: odom_debias.py must sit in the same folder (%s): %s"
             % (os.path.dirname(os.path.abspath(__file__)), _e))

CELL = 0.05
N = 4000                              # 200 m square around the start mark
WALL = (0.15, 2.0)                    # m above the base: walls, not floor or ceiling
FLOOR = (-0.30, 0.0)                  # the floor sits ~0.13 m below base_link
MAX_RANGE = 20.0
LOOKUP_WAIT = 0.2                     # s a pose lookup may wait for the odometry to catch up
# A scan's own stamp is only looked up if it is between 1 s ahead of and 9 s behind the
# robot's clock: the TF buffer below keeps 10 s. Anything else (a LiDAR reporting its own
# unset clock - the Helios did, as 2017, when decoded with use_lidar_clock) goes straight
# to the latest pose instead of waiting LOOKUP_WAIT for a pose that can never come.
STAMP_USABLE = (-1.0, 9.0)

S = {"msg": None, "lock": threading.Lock(), "wall": np.zeros((N, N), np.uint16),
     "floor": np.zeros((N, N), np.uint16), "path": [], "pose": None, "scans": 0,
     "t0": time.time(), "start": None, "run": "", "err": "",
     "debias": LiveDebias(), "own": 0, "latest": 0, "failed": 0, "why_latest": "",
     "ages": collections.deque(maxlen=50), "restarted": "", "log": None,
     # health of the background thread, shown in /stats.json:
     "why_failed": "",        # the last reason a pose lookup failed ("failed" counts them)
     "errors": 0,             # scans (or loop turns) that raised an error and were skipped
     "last_error": "",        # the last such error, with when it happened
     "last_ok": None,         # wall-clock time the last scan was drawn
     "thread": None}          # the worker thread, so stats() can tell if it has ended
# Full error text (traceback) goes to this program's log for the first few errors, then
# only every 100th, so a fault repeating twice a second cannot fill the robot's disk.
TRACEBACKS_FIRST = 5


def mat(tr):
    t, q = tr.transform.translation, tr.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, (t.x, t.y, t.z)
    return T


def planar(ang, tx, ty):
    """4x4 matrix: turn by ang about the vertical, then shift by (tx, ty). Applied on the
    left of a pose in the odom frame it moves the pose flat on the floor, keeping its
    roll and pitch."""
    c, s = math.cos(ang), math.sin(ang)
    D = np.eye(4)
    D[0, 0], D[0, 1], D[1, 0], D[1, 1], D[0, 3], D[1, 3] = c, -s, s, c, tx, ty
    return D


STRIDE = 4                            # every 4th point: 14,400 a scan is plenty for walls


def xyz(msg):
    d = np.frombuffer(msg.data, np.uint8).reshape(-1, msg.point_step)[::STRIDE]
    p = d[:, :12].copy().view(np.float32).reshape(-1, 3)
    p = p[np.isfinite(p).all(1)]
    r = np.hypot(p[:, 0], p[:, 1])
    return p[(r > 0.3) & (r < MAX_RANGE)]


def cb(msg):
    with S["lock"]:
        S["msg"] = msg


def lookup_poses(buf, frame, stamp, now):
    """The robot's pose (odom -> base_link) and the LiDAR's (odom -> frame) as 4x4
    matrices, at the SCAN'S OWN time stamp; the latest ones if that is impossible.
    Returns (Tob, Tos, pose_time_s, own_time_used, why_not). Raises if neither works."""
    wait = rospy.Duration(LOOKUP_WAIT)
    age = (now - stamp).to_sec()
    if STAMP_USABLE[0] < age < STAMP_USABLE[1]:
        try:
            tb = buf.lookup_transform("odom", "base_link", stamp, wait)
            ts = buf.lookup_transform("odom", frame, stamp, wait)
            return mat(tb), mat(ts), stamp.to_sec(), True, ""
        except Exception as e:                   # not in the buffer: fall back, and say why
            why = type(e).__name__
    else:
        why = "scan stamp %.1f s old" % age
    tb = buf.lookup_transform("odom", "base_link", rospy.Time(0), wait)
    ts = buf.lookup_transform("odom", frame, rospy.Time(0), wait)
    return mat(tb), mat(ts), tb.header.stamp.to_sec(), False, why


def process(buf, msg, now):
    """Place one scan: pose at its own time, drift correction, walls and floor into the
    picture. Returns True if the scan was drawn."""
    try:
        Tob, Tos, tp, own, why = lookup_poses(buf, msg.header.frame_id, msg.header.stamp, now)
    except Exception as e:                      # no transform at all yet
        with S["lock"]:
            S["err"], S["failed"] = type(e).__name__, S["failed"] + 1
            S["why_failed"] = type(e).__name__
        return False
    DB = S["debias"]
    raw = (float(Tob[0, 3]), float(Tob[1, 3]), math.atan2(Tob[1, 0], Tob[0, 0]))
    with S["lock"]:
        xc, yc, yawc, started = DB.add(tp, *raw)
        if started and DB.still_parked:
            # the first minute was drawn with the drifting heading; the robot has not
            # moved, so the next scans redraw the same view, corrected
            S["wall"][:] = 0
            S["floor"][:] = 0
            S["path"] = []
            S["restarted"] = "picture restarted once, %.0f s in, when the drift was measured" % (
                time.time() - S["t0"])
        S["own" if own else "latest"] += 1
        if not own:
            S["why_latest"] = why
        S["ages"].append((now - msg.header.stamp).to_sec())
    D = planar(*correction_se2(raw, (xc, yc, yawc)))
    Tob, Tos = D @ Tob, D @ Tos
    if S["start"] is None:
        S["start"] = np.linalg.inv(Tob)         # start mark = origin, facing +x
    Ts = (S["start"] @ Tos).astype(np.float32)
    p = xyz(msg) @ Ts[:3, :3].T + Ts[:3, 3]
    ij = np.floor(p[:, :2] / CELL).astype(np.int64) + N // 2
    ok = (ij >= 0).all(1) & (ij < N).all(1)
    ij, z = ij[ok], p[ok, 2]
    w = (z >= WALL[0]) & (z <= WALL[1])
    f = (z >= FLOOR[0]) & (z <= FLOOR[1])
    wu, wc = np.unique(ij[w, 1] * N + ij[w, 0], return_counts=True)
    fu, fc = np.unique(ij[f, 1] * N + ij[f, 0], return_counts=True)
    with S["lock"]:
        for arr, u, c in ((S["wall"], wu, wc), (S["floor"], fu, fc)):
            flat = arr.reshape(-1)
            flat[u] = np.minimum(flat[u].astype(np.uint32) + c, 65000).astype(np.uint16)
        B = S["start"] @ Tob
        pose = (B[0, 3], B[1, 3], math.atan2(B[1, 0], B[0, 0]))
        if not S["path"] or math.hypot(pose[0] - S["path"][-1][0], pose[1] - S["path"][-1][1]) > 0.1:
            S["path"].append(pose[:2])
        S["pose"], S["scans"], S["err"] = pose, S["scans"] + 1, ""
        S["last_ok"] = time.time()
    if S["log"] is not None:
        try:
            S["log"].write("%.6f,%.6f,%d,%.4f,%.4f,%.3f,%.4f,%.4f,%.3f,%d,%s\n" % (
                msg.header.stamp.to_sec(), tp, own, raw[0], raw[1], math.degrees(raw[2]),
                xc, yc, math.degrees(yawc), DB.correcting,
                "%.4f" % DB.rate if DB.correcting else "nan"))
            S["log"].flush()                    # the process exits with os._exit: nothing is flushed later
        except (OSError, ValueError):
            S["log"] = None
    return True


def note_error(e, where):
    """Count an error, keep a one-line description for /stats.json, and log the full text."""
    with S["lock"]:
        S["errors"] += 1
        n = S["errors"]
        S["last_error"] = "%s: %s: %s (%.0f s after start)" % (
            where, type(e).__name__, str(e)[:120], time.time() - S["t0"])
    if n <= TRACEBACKS_FIRST or n % 100 == 0:
        # to stdout and flushed at once: on Python 3.8 a redirected stderr is fully
        # buffered, and this program exits with os._exit, which flushes nothing
        print("error %d, %s:" % (n, where), flush=True)
        traceback.print_exc(file=sys.stdout)
        sys.stdout.flush()


def process_safely(buf, msg, now):
    """process(), except that an error in one scan is counted and shown instead of ending
    the worker thread. Returns True if the scan was drawn."""
    try:
        return process(buf, msg, now)
    except Exception as e:                      # noqa: BLE001 - any fault: skip this scan, keep going
        note_error(e, "processing a scan")
        return False


def worker(buf):
    # reset=True: if the clock jumps backwards (a replay restarted with /use_sim_time),
    # the rate restarts its timing instead of raising ROSTimeMovedBackwardsException
    # (checked on rospy 1.17.4, Noetic: Rate(hz, reset=False) raises it by default).
    rate = rospy.Rate(2, reset=True)
    while not rospy.is_shutdown():
        try:
            rate.sleep()
        except Exception as e:                  # noqa: BLE001 - never let timing end the thread
            if rospy.is_shutdown():
                return
            note_error(e, "waiting for the next scan")
            time.sleep(0.5)
            continue
        with S["lock"]:
            msg, S["msg"] = S["msg"], None
        if msg is None:
            continue
        process_safely(buf, msg, rospy.Time.now())


def render(scale):
    with S["lock"]:
        wall = S["wall"] >= 2                        # two hits: not a single stray return
        seen = (S["floor"] >= 1) & ~wall
        path, pose = list(S["path"]), S["pose"]
    known = wall | seen
    nz = np.argwhere(known)
    if not len(nz):
        return None
    (r0, c0), (r1, c1) = nz.min(0) - 8, nz.max(0) + 9
    r0, c0, r1, c1 = max(r0, 0), max(c0, 0), min(r1, N), min(c1, N)
    img = np.full((r1 - r0, c1 - c0, 3), 55, np.uint8)
    img[seen[r0:r1, c0:c1]] = (232, 232, 232)
    img[wall[r0:r1, c0:c1]] = (40, 40, 200)          # the camera page's colours, BGR
    px = lambda x, y: (int(x / CELL) + N // 2 - c0, int(y / CELL) + N // 2 - r0)
    for a, b in zip(path, path[1:]):
        cv2.line(img, px(*a), px(*b), (0, 170, 255), 1)
    if pose:
        c = px(pose[0], pose[1])
        cv2.circle(img, c, 4, (60, 200, 60), -1)
        cv2.line(img, c, (int(c[0] + 9 * math.cos(pose[2])), int(c[1] + 9 * math.sin(pose[2]))),
                 (60, 200, 60), 2)
    img = np.flipud(img)                             # y up, as on the camera page
    if scale > 1:
        img = cv2.resize(img, (img.shape[1] * scale, img.shape[0] * scale),
                         interpolation=cv2.INTER_NEAREST)
    ok, b = cv2.imencode(".png", img)
    return b.tobytes() if ok else None


def stats():
    with S["lock"]:
        path, pose = S["path"], S["pose"]
        dist = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:]))
        d = S["debias"].describe()
        corr = S["debias"].correcting
        own, lat = S["own"], S["latest"]
        ages = sorted(S["ages"])
        out = {}
        th = S["thread"]
        if th is not None and not th.is_alive():   # first line on the page: nothing below is live
            out["worker"] = "STOPPED - the picture is frozen; restart lidar_live_map.py"
        out.update({"source": "LiDAR walls placed by wheel+IMU odometry%s (not loop-corrected)"
                              % (" minus its parked heading drift" if corr else ", heading drift NOT removed"),
                    "run": S["run"], "scans": S["scans"]})
        if S["last_ok"] is not None:
            out["last scan drawn"] = "%.0f s ago" % (time.time() - S["last_ok"])
        out.update({"wall cells": int((S["wall"] >= 2).sum()),
                    "distance": "%.1f m" % dist,
                    "pos": ("%.2f, %.2f m" % pose[:2]) if pose else "-",
                    "drift": d["drift"]})
        if "drift +-" in d:
            out["drift +-"] = d["drift +-"]
        out["correction"] = d["correction"]
        if S["errors"]:
            out["processing errors"] = "%d, each skipped and the view carried on (last: %s)" % (
                S["errors"], S["last_error"])
        if S["failed"]:
            out["pose lookups failed"] = "%d (last: %s)" % (S["failed"], S["why_failed"])
        if own + lat:
            out["pose at scan time"] = "%d of %d scans" % (own, own + lat)
        if lat:
            out["used latest pose"] = "%d (%s)" % (lat, S["why_latest"])
        if ages:
            out["scan age"] = "%.2f s" % ages[len(ages) // 2]
        if S["restarted"]:
            out["note"] = S["restarted"]
        out["running"] = "%d s" % (time.time() - S["t0"])
        if S["err"]:
            out["waiting for"] = S["err"]
        return out


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        p = self.path.split("?")[0]
        if p == "/map.png":
            png = render(self.server.scale)
            if png is None:
                self.send_response(503); self.end_headers(); return
            self.send_response(200)
            self.send_header("Content-Type", "image/png")
            self.send_header("Cache-Control", "no-store")
            self.end_headers(); self.wfile.write(png)
        elif p == "/stats.json":
            b = json.dumps(stats()).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.end_headers(); self.wfile.write(b)
        elif p in ("/", "/index.html"):
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(b"<!doctype html><meta charset=utf-8><title>LiDAR live</title>"
                             b"<body style='background:#0e1116;color:#e8ecf2;font:14px sans-serif'>"
                             b"<img id=m src=/map.png style='max-width:100%'><pre id=s></pre><script>"
                             b"setInterval(async()=>{m.src='/map.png?'+Date.now();"
                             b"s.textContent=JSON.stringify(await (await fetch('/stats.json')).json(),null,1)},1000)"
                             b"</script>")
        else:
            self.send_response(404); self.end_headers()


def open_pose_log(path):
    """A new CSV for this run's placed-scan poses; never appends to or overwrites an old one."""
    if not path:
        return None
    path = os.path.expanduser(path)
    base, ext = os.path.splitext(path)
    k = 1
    while os.path.exists(path):
        k += 1
        path = "%s_%d%s" % (base, k, ext)
    try:
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        fh = open(path, "w")
        fh.write("scan_stamp,pose_time,own_time,raw_x,raw_y,raw_yaw_deg,corr_x,corr_y,corr_yaw_deg,"
                 "correcting,rate_deg_per_min\n")
        print("pose log: %s" % path, flush=True)
        return fh
    except OSError as e:
        print("pose log NOT written (%s): %s" % (path, e), flush=True)
        return None


def main():
    import signal
    # rospy is told not to handle signals, and serve_forever() does not return
    # on SIGINT - the first test ignored an interrupt and kept the port. Exit
    # outright on SIGINT/SIGTERM: the pose log is flushed after every line, so
    # nothing is lost.
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *a: os._exit(0))
    rospy.init_node("lidar_live_map", anonymous=True, disable_signals=True)
    S["run"] = rospy.get_param("~run", "")
    port = int(rospy.get_param("~port", 8095))
    S["debias"] = LiveDebias(min_parked_s=float(rospy.get_param("~min_parked_s", 60.0)))
    default_log = "~/jobs/%s_lidar_live_poses.csv" % S["run"] if S["run"] else ""
    S["log"] = open_pose_log(rospy.get_param("~pose_log", default_log))
    buf = tf2_ros.Buffer(rospy.Duration(10))
    tf2_ros.TransformListener(buf)
    rospy.Subscriber(rospy.get_param("~cloud", "/helios/points"), PointCloud2, cb,
                     queue_size=1, buff_size=2 ** 24)
    S["thread"] = threading.Thread(target=worker, args=(buf,), daemon=True)
    S["thread"].start()
    srv = ThreadingHTTPServer(("0.0.0.0", port), H)
    srv.scale = int(rospy.get_param("~scale", 2))
    print("LiDAR live map on port %d (drift measured after %.0f s parked)"
          % (port, S["debias"].min_parked_s), flush=True)
    srv.serve_forever()


if __name__ == "__main__":
    main()
