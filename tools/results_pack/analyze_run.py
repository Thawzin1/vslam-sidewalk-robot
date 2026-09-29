#!/usr/bin/env python3
"""analyze_run.py - the numbers and figures of one fused drive's results pack (ENGINEERING_NOTES.md rule 22).

Jetson only. Reads, never writes, the run's records:
    ~/.run_records/<run>/fusion.bag      /fused/odometry (the blend), /robot/ekf_odom (the robot's own
                                          wheels + gyroscope estimate), /rtabmap/odom (the camera's
                                          tracking), /robot/wheel_odom (is the robot moving?)
    ~/.run_records/<run>/turns.log        the live turn watcher (times in UTC, converted here)
    ~/.run_records/<run>/bridge_recv.log  the WiFi link's own log (FINAL line = its counters)
    ~/.run_records/<run>/cpu_jetson.json  processor load of the fusion processes
    <db> (optional, read-only)           RTAB-Map per-node timing from its Statistics table
and writes into the pack folder: numbers.json, turns.csv, paths_2hz.csv, trajectory.png,
turns_difference.png.

Every path here is TRACKING ALONE (rule 20): odometry of each source, never corrected by loop
closures - except camera_corrected.tum, the map's corrected node positions (Admin.opt_poses).
Nothing here is ground truth (rule 19); the robot's estimate is the comparison for short turns only.

usage: analyze_run.py <run> <pack_dir> [--db DB] [--title "Drive 3"]
"""
import argparse
import calendar
import json
import math
import os
import re
import sqlite3
import sys
import time
import zlib

import numpy as np

os.environ["TZ"] = "America/Toronto"
time.tzset()
BLUE, ORANGE, AQUA, YELLOW = "#2a78d6", "#eb6834", "#1baf7a", "#eda100"
INK, MUTED, GRID = "#22252a", "#6e7074", "#e4e2dc"


def ham(t, fmt="%H:%M:%S"):
    return time.strftime(fmt, time.localtime(t))


def yaw_q(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def read_bag(path, progress):
    import rosbag
    want = {"/fused/odometry": "blend", "/robot/ekf_odom": "robot", "/rtabmap/odom": "camera",
            "/robot/wheel_odom": "wheel"}
    out = {v: [] for v in want.values()}
    n = 0
    t0 = time.time()
    with rosbag.Bag(path) as b:
        for topic, m, t in b.read_messages(topics=list(want)):
            p = m.pose.pose
            cov0 = m.pose.covariance[0]
            out[want[topic]].append((m.header.stamp.to_sec(), p.position.x, p.position.y,
                                     yaw_q(p.orientation), m.twist.twist.linear.x,
                                     m.twist.twist.angular.z, cov0))
            n += 1
            if n % 20000 == 0 and progress:
                with open(progress, "w") as fh:
                    fh.write("PACK_ANALYZE bag %d msgs  %ds\n" % (n, time.time() - t0))
    return {k: np.array(v) for k, v in out.items()}


def to_start_frame(a):
    """Move a path so its first pose is (0,0) facing +x - each source keeps its own shape."""
    x, y, th = a[:, 1] - a[0, 1], a[:, 2] - a[0, 2], a[:, 3]
    c, s = math.cos(-th[0]), math.sin(-th[0])
    return np.c_[a[:, 0], c * x - s * y, s * x + c * y, np.unwrap(th - th[0])]


def path_len(p):
    return float(np.sum(np.linalg.norm(np.diff(p[:, 1:3], axis=0), axis=1)))


def gap(p):
    d = float(np.linalg.norm(p[-1, 1:3] - p[0, 1:3]))
    h = math.degrees(math.atan2(math.sin(p[-1, 3] - p[0, 3]), math.cos(p[-1, 3] - p[0, 3])))
    return round(d, 3), round(h, 2)


def lost_stretches(cam):
    """RTAB-Map publishes a null pose with covariance 9999 when its tracking is lost."""
    lost = (cam[:, 6] >= 9998) | ((cam[:, 1] == 0) & (cam[:, 2] == 0) & (cam[:, 3] == 0))
    spans, k = [], 0
    while k < len(cam):
        if lost[k]:
            j = k
            while j < len(cam) and lost[j]:
                j += 1
            end = cam[j, 0] if j < len(cam) else cam[-1, 0]
            spans.append((cam[k, 0], end))
            k = j
        else:
            k += 1
    return lost, spans


def still_heading(blend, wheel):
    """The blend's heading change while the robot's wheels read exactly zero (true turn 0).
    A 'stop' is one unbroken stretch of such samples. Same wording as the drive-2 replay review
    (results_conditioner_round3.md section 9: wheels exactly zero, signed sum over stops, total)."""
    ws = wheel[np.argsort(wheel[:, 0])]
    flag = (np.abs(ws[:, 4]) < 1e-6) & (np.abs(ws[:, 5]) < 1e-6)   # "exactly zero" up to 1e-16 float noise
    idx = np.clip(np.searchsorted(ws[:, 0], blend[:, 0]), 1, len(ws) - 1)
    near = np.where(np.abs(ws[idx, 0] - blend[:, 0]) < np.abs(ws[idx - 1, 0] - blend[:, 0]), idx, idx - 1)
    still = flag[near] & (np.abs(ws[near, 0] - blend[:, 0]) < 0.3)
    stops, k = [], 0
    while k < len(still):
        if still[k]:
            j = k
            while j < len(still) and still[j]:
                j += 1
            if j - k > 1:
                stops.append((k, j - 1))
            k = j
        else:
            k += 1
    th = np.unwrap(blend[:, 3])
    nets = [math.degrees(th[j] - th[i]) for i, j in stops]
    steps = sum(float(np.sum(np.abs(np.diff(th[i:j + 1])))) for i, j in stops)
    return {"stops": len(stops),
            "seconds_still": round(sum(blend[j, 0] - blend[i, 0] for i, j in stops), 1),
            "signed_sum_deg": round(sum(nets), 3),
            "sum_abs_net_per_stop_deg": round(sum(abs(x) for x in nets), 3),
            "sum_abs_steps_deg": round(math.degrees(steps), 3),
            "largest_stop_net_deg": round(max(nets, key=abs), 3) if nets else None,
            "rule": "robot wheel odometry speed and turn rate both zero, below 1e-6 - the log holds 1e-16 float noise, not 0.0 (nearest wheel message within 0.3 s)"}


class UtcClock:
    """The watcher logs print only HH:MM:SS in UTC. Turn each into epoch seconds on the right UTC day:
    start from an anchor (the bag's first stamp) and pick, for every line, the day (-1/0/+1) that lands
    nearest the previous line - so a drive that crosses UTC midnight (20:00 Hamilton in summer time)
    moves to the next day. Hamilton times then come from ham(), which applies daylight saving itself."""
    def __init__(self, anchor):
        self.prev = float(anchor)

    def __call__(self, hms):
        g = time.gmtime(self.prev)
        day0 = calendar.timegm((g.tm_year, g.tm_mon, g.tm_mday, 0, 0, 0, 0, 0, 0))
        h, m, s = (int(x) for x in hms.split(":"))
        base = day0 + h * 3600 + m * 60 + s
        t = min((base - 86400, base, base + 86400), key=lambda x: abs(x - self.prev))
        self.prev = t
        return t


def parse_turns(path, anchor):
    rows, clock = [], UtcClock(anchor)
    rx = re.compile(r"(\d\d:\d\d:\d\d) TURN (\d+) (\w+) ended after ([\d.]+) s: blend ([+-][\d.]+) deg, "
                    r"robot ([+-][\d.]+) deg, camera ([+-][\d.]+|-) deg \(camera lost (\d+)% of it\); "
                    r"blend - robot ([+-][\d.]+) deg")
    for line in open(path):
        m = rx.match(line)
        if m:
            t = clock(m.group(1))
            b, r = float(m.group(5)), float(m.group(6))
            c = None if m.group(7) == "-" else float(m.group(7))
            rows.append({"turn": int(m.group(2)), "ended_hamilton": ham(t), "direction": m.group(3),
                         "seconds": float(m.group(4)), "blend_deg": b, "robot_deg": r,
                         "camera_deg": c, "camera_lost_pct": int(m.group(8)),
                         "blend_minus_robot_deg": float(m.group(9)),
                         "camera_minus_robot_deg": None if c is None else round(c - r, 1)})
    return rows


def rtabmap_timing(db):
    con = sqlite3.connect("file:%s?immutable=1" % db, uri=True)
    tot, ram = [], []
    for (blob,) in con.execute("SELECT data FROM Statistics ORDER BY id"):
        b = bytes(blob)
        try:
            s = zlib.decompress(b[:-12]).decode()
        except Exception:
            s = b.decode(errors="ignore")
        d = dict(kv.split(":", 1) for kv in s.split(";") if ":" in kv)
        if "Timing/Total/ms" in d:
            tot.append(float(d["Timing/Total/ms"]))
        if "Memory/RAM_usage/MB" in d:
            ram.append(float(d["Memory/RAM_usage/MB"]))
    out = {}
    if tot:
        t = np.array(tot)
        out["rtabmap_ms_per_node"] = {"n": len(t), "median": round(float(np.median(t)), 1),
                                     "p95": round(float(np.percentile(t, 95)), 1),
                                     "max": round(float(t.max()), 1)}
    if ram:
        out["rtabmap_ram_mb"] = {"median": round(float(np.median(ram)), 0), "max": round(max(ram), 0)}
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("pack")
    ap.add_argument("--db", default="")
    ap.add_argument("--title", default="")
    ap.add_argument("--progress", default="")
    ap.add_argument("--camera-label", default="camera, tracking alone (/rtabmap/odom)")
    ap.add_argument("--turns", default="", help="turn log to read instead of <records>/turns.log "
                    "(drive 4: replayed after the drive by turns_replay.py)")
    ap.add_argument("--turns-xlabel", default="turn number (turn watcher's rule; drive 4: replayed from the recording)",
                    help="x-axis label of turns_difference.png (drive 5 added this; the default is drive 4's text)")
    ap.add_argument("--crash-time", type=float, default=None,
                    help="epoch s when the camera program died (drive 5): marked on trajectory.png and turns_difference.png;"
                         " absent = no mark (drives 3-4 unchanged)")
    ap.add_argument("--crash-label", default="camera crashed")
    ap.add_argument("--note", default="No LiDAR estimate yet for this run (made on the robot, which is off).")
    a = ap.parse_args()
    rec = os.path.join(os.path.expanduser(os.environ.get("RECORDS_DIR", "~/.run_records")), a.run)
    os.makedirs(a.pack, exist_ok=True)
    num = {"run": a.run, "sources": {}}
    S = num["sources"]

    bag = read_bag(os.path.join(rec, "fusion.bag"), a.progress)
    S["paths"] = "%s/fusion.bag (/fused/odometry, /robot/ekf_odom, /rtabmap/odom, /robot/wheel_odom)" % rec
    t_anchor = min(float(v[0, 0]) for v in bag.values() if len(v))   # date for the HH:MM:SS logs
    lost, spans = lost_stretches(bag["camera"])
    cam_ok = bag["camera"][~lost]
    paths = {"blend": to_start_frame(bag["blend"]), "robot": to_start_frame(bag["robot"]),
             "camera": to_start_frame(cam_ok)}
    num["tracking_alone_gaps"] = {k: dict(zip(("end_to_start_m", "heading_deg"), gap(p)),
                                          path_m=round(path_len(p), 1),
                                          first=ham(p[0, 0]), last=ham(p[-1, 0]))
                                  for k, p in paths.items()}
    num["camera_tracking_lost"] = {
        "stretches": len(spans), "seconds_total": round(sum(b - a_ for a_, b in spans), 1),
        "longest_s": round(max([b - a_ for a_, b in spans] or [0]), 1),
        "share_of_messages_pct": round(100.0 * lost.mean(), 1),
        "rule": "/rtabmap/odom message with covariance >= 9998 or an all-zero pose (RTAB-Map's lost signal)"}
    num["blend_heading_while_still"] = still_heading(bag["blend"], bag["wheel"])

    # small 2 Hz copy of the paths for figures
    with open(os.path.join(a.pack, "paths_2hz.csv"), "w") as fh:
        fh.write("# each path moved to start at (0,0) facing +x; tracking alone (odometry); "
                 "time = epoch s; source %s\nsource,t,x,y,heading_deg\n" % S["paths"])
        for k, p in paths.items():
            keep = np.r_[0, np.where(np.diff(np.floor(p[:, 0] * 2)) > 0)[0] + 1]
            for r in p[keep]:
                fh.write("%s,%.2f,%.3f,%.3f,%.2f\n" % (k, r[0], r[1], r[2], math.degrees(r[3])))

    turns = []
    tl = a.turns or os.path.join(rec, "turns.log")
    if os.path.exists(tl):
        S["turns"] = tl + (" (turn watcher's rule REPLAYED after the drive from fusion.bag; UTC in the file, Hamilton here)"
                           if a.turns else " (live turn watcher; UTC in the file, Hamilton here)")
        turns = parse_turns(tl, t_anchor)
        with open(os.path.join(a.pack, "turns.csv"), "w") as fh:
            keys = list(turns[0].keys())
            fh.write("# source: %s\n%s\n" % (S["turns"], ",".join(keys)))
            for r in turns:
                fh.write(",".join(str(r[k]) for k in keys) + "\n")
        br = np.array([r["blend_minus_robot_deg"] for r in turns])
        cr = np.array([r["camera_minus_robot_deg"] for r in turns if r["camera_minus_robot_deg"] is not None])
        num["turns"] = {"n": len(turns),
                        "abs_blend_minus_robot_deg": {"median": round(float(np.median(np.abs(br))), 2),
                                                      "p95": round(float(np.percentile(np.abs(br), 95)), 2),
                                                      "max": round(float(np.abs(br).max()), 1)},
                        "abs_camera_minus_robot_deg": {"median": round(float(np.median(np.abs(cr))), 2),
                                                       "p95": round(float(np.percentile(np.abs(cr), 95)), 2),
                                                       "max": round(float(np.abs(cr).max()), 1)},
                        "within_1deg_blend": int(np.sum(np.abs(br) <= 1.0)),
                        "within_1deg_camera": int(np.sum(np.abs(cr) <= 1.0)),
                        "turns_with_no_camera_value": len(turns) - len(cr)}

    blog = os.path.join(rec, "bridge_recv.log")
    if os.path.exists(blog):
        S["link"] = blog
        L = open(blog).read().splitlines()
        downs = [l for l in L if "STATE DOWN" in l]
        fin = [l for l in L if " FINAL " in l]
        bclock = UtcClock(t_anchor)
        down_t = [bclock(l[:8]) if re.match(r"\d\d:\d\d:\d\d", l) else None for l in L]
        link = {"down_events": len(downs),
                "down_lines_hamilton": [ham(t) + l[8:] for l, t in zip(L, down_t) if "STATE DOWN" in l]}
        # one FINAL per receiver session; the sender's counters run on across sessions,
        # so each session's share is taken against the difference (as t4_score.py does)
        finals = [json.loads(l.split(" FINAL ", 1)[1]) for l in fin]
        link["receiver_sessions"] = len(finals)
        link["connects"] = sum(f["connects"] for f in finals)
        link["sessions"] = []
        for i, f in enumerate(finals):
            ses = {"elapsed_s": f["elapsed_s"], "per_topic": {}}
            for tp, v in f["topics"].items():
                prev = finals[i - 1]["sender_stats"] if i else {"sent": {}, "received": {}}
                sent = f["sender_stats"]["sent"].get(tp, 0) - prev["sent"].get(tp, 0)
                recd = f["sender_stats"]["received"].get(tp, 0) - prev["received"].get(tp, 0)
                ses["per_topic"][tp] = {"robot_read": recd, "robot_sent": sent,
                                        "published_on_jetson": v["published"],
                                        "pct_of_sent": round(100.0 * v["published"] / sent, 2) if sent else None,
                                        "pct_of_robot_read": round(100.0 * v["published"] / recd, 2) if recd else None,
                                        "missed_while_down": v["missed_while_down"], "stale": v["stale"],
                                        "lost": v["lost"]}
            link["sessions"].append(ses)
        num["link"] = link
    for name in ("t4_score.json",):
        pth = os.path.join(rec, name)
        if os.path.exists(pth):
            S["score"] = pth

    cj = os.path.join(rec, "cpu_jetson.json")
    if os.path.exists(cj):
        S["cpu"] = cj
        c = json.load(open(cj))
        num["processor_fusion"] = {"total": c["total"], "gate": c["gate"]["verdict"],
                                   "per_process": {v["label"]: {k: v[k] for k in ("median_cores", "p99_cores", "max_cores")}
                                                   for v in c["per_process"].values()}}
    if a.db:
        S["rtabmap_timing"] = a.db + " (Statistics table, read-only)"
        num["processor_rtabmap"] = rtabmap_timing(a.db)

    json.dump(num, open(os.path.join(a.pack, "numbers.json"), "w"), indent=1)
    figures(a, paths, spans, turns)
    print(json.dumps(num, indent=1))


def figures(a, paths, spans, turns):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.family": "DejaVu Serif", "font.size": 11, "axes.edgecolor": MUTED,
                         "axes.labelcolor": INK, "xtick.color": MUTED, "ytick.color": MUTED,
                         "text.color": INK})
    title = a.title or a.run
    # drive 4 review fix (rule 20): tracking alone and corrected are drawn on SEPARATE panels, never one axis.
    # Comparison sources SOLID (robot's own wheels + gyroscope; LiDAR estimate), camera / blend / map DASHED.
    fig, (ax, bx) = plt.subplots(1, 2, figsize=(15, 6.2), dpi=130, sharex=True, sharey=True)
    corr = os.path.join(a.pack, "camera_corrected.tum")
    series = [("camera", BLUE, a.camera_label),
              ("blend", ORANGE, "blend of camera + robot wheels + gyroscope"),
              ("robot", YELLOW, "robot's own wheels + gyroscope" + (
                  " (as received on the Jetson: ends %s, %.0f s before the blend - not end-to-start)" % (ham(paths["robot"][-1, 0]), paths["blend"][-1, 0] - paths["robot"][-1, 0])
                  if paths["blend"][-1, 0] - paths["robot"][-1, 0] > 60 else ""))]
    for k, col, lab in series:
        p = paths[k]
        ax.plot(p[:, 1], p[:, 2], "-" if k == "robot" else "--", color=col, lw=2, label=lab)
        ax.plot(p[-1, 1], p[-1, 2], "o", color=col, ms=8, mec="white", mew=2)
        if a.crash_time is not None and p[0, 0] <= a.crash_time <= p[-1, 0] + 1.0:
            j = int(np.argmin(np.abs(p[:, 0] - a.crash_time)))
            ax.plot(p[j, 1], p[j, 2], "X", color=col, ms=13, mec=INK, mew=1.2, zorder=6,
                    label="%s (%s) - on this path" % (a.crash_label, ham(a.crash_time)) if k == "robot" else None)
            if k == "robot":
                ax.annotate("%s %s" % (a.crash_label, ham(a.crash_time)), (p[j, 1], p[j, 2]), xytext=(10, -14),
                            textcoords="offset points", fontsize=9, color=INK)

    def start_frame(fn):
        k = np.loadtxt(fn, usecols=range(8))
        th0 = math.atan2(2 * (k[0, 7] * k[0, 6] + k[0, 4] * k[0, 5]), 1 - 2 * (k[0, 5] ** 2 + k[0, 6] ** 2))
        x, y = k[:, 1] - k[0, 1], k[:, 2] - k[0, 2]
        c, s = math.cos(-th0), math.sin(-th0)
        return c * x - s * y, s * x + c * y

    if os.path.exists(corr):
        x, y = start_frame(corr)
        _m = json.load(open(corr + ".meta.json")) if os.path.exists(corr + ".meta.json") else {}
        _how = "graph re-optimised by optimize_graph_se2.py - database not closed" if "Gauss-Newton" in _m.get("method", "") else "Admin.opt_poses"
        bx.plot(x, y, "--", color=AQUA, lw=2, label="camera map, corrected by its loop closures (%s)" % _how)
        bx.plot(x[-1], y[-1], "o", color=AQUA, ms=8, mec="white", mew=2)
    lt = os.path.join(a.pack, "lidar.tum")
    if os.path.exists(lt):
        x, y = start_frame(lt)
        bx.plot(x, y, "-", color=INK, lw=2.2,
                label="LiDAR estimate (robot's LiDAR map: wheels + gyroscope\ncorrected by LiDAR loop closures)")
        bx.plot(x[-1], y[-1], "o", color=INK, ms=8, mec="white", mew=2)
    for axx, sub in ((ax, "tracking alone (odometry, never corrected)"),
                     (bx, "corrected by loop closures")):
        axx.plot(0, 0, "o", color=INK, ms=10, mec="white", mew=2)
        axx.annotate("start", (0, 0), xytext=(8, 8), textcoords="offset points", color=INK)
        axx.set_aspect("equal")
        axx.grid(color=GRID, lw=0.8)
        for sp in ("top", "right"):
            axx.spines[sp].set_visible(False)
        axx.set_xlabel("metres (each path starts at the start mark, facing right)")
        axx.set_title(sub, loc="left", fontsize=11)
        axx.legend(loc="upper center", bbox_to_anchor=(0.5, -0.14), ncol=1, fontsize=9, frameon=False)
    ax.set_ylabel("metres")
    fig.suptitle("%s - where each source put the robot" % title, x=0.01, ha="left", fontsize=13)
    fig.text(0.01, 0.925, a.note + " Dots mark where each path ended. Nothing here is ground truth.",
             fontsize=8, color=MUTED)
    fig.tight_layout(rect=(0, 0, 1, 0.92))
    fig.savefig(os.path.join(a.pack, "trajectory.png"), bbox_inches="tight")
    plt.close(fig)

    if turns:
        n = np.array([r["turn"] for r in turns])
        br = np.array([r["blend_minus_robot_deg"] for r in turns])
        cr = np.array([np.nan if r["camera_minus_robot_deg"] is None else r["camera_minus_robot_deg"]
                       for r in turns])
        lostp = np.array([r["camera_lost_pct"] for r in turns])
        fig, ax = plt.subplots(figsize=(12, 5), dpi=130)
        ax.axhline(0, color=MUTED, lw=1)
        ax.axhspan(-1, 1, color=GRID, lw=0, zorder=0)
        ax.plot(n, np.clip(cr, -29, 29), "o", color=BLUE, ms=8, mec="white", mew=2, label="camera minus robot")
        ax.plot(n, np.clip(br, -29, 29), "o", color=ORANGE, ms=8, mec="white", mew=2, label="blend minus robot")
        dead = np.zeros(len(n), bool)
        if a.crash_time is not None:
            day = time.strftime("%Y-%m-%d ", time.localtime(a.crash_time))
            tend = np.array([time.mktime(time.strptime(day + r["ended_hamilton"], "%Y-%m-%d %H:%M:%S")) for r in turns])
            dead = tend >= a.crash_time
        bl = (lostp >= 50) & ~dead
        ax.plot(n[bl], np.clip(br[bl], -29, 29), "o", ms=15, mfc="none", mec=INK, mew=1.2,
                label="blind turn (camera lost at least half of it)")
        if dead.any():
            ax.plot(n[dead], np.clip(br[dead], -29, 29), "s", ms=13, mfc="none", mec=MUTED, mew=1.0,
                    label="camera dead (after the crash: blend = wheels + robot gyroscope only)")
            xc = (n[~dead].max() + n[dead].min()) / 2.0 if (~dead).any() else n[dead].min() - 0.5
            ax.axvline(xc, color=INK, lw=1.4, ls="-")
            ax.annotate("%s %s" % (a.crash_label, ham(a.crash_time)), (xc, 27), xytext=(6, 0),
                        textcoords="offset points", fontsize=9, color=INK, va="center")
        big = np.where(np.nan_to_num(np.abs(cr)) > 15)[0]
        for i in big:
            ax.annotate("%+.1f" % cr[i], (n[i], np.clip(cr[i], -29, 29)), xytext=(6, 0),
                        textcoords="offset points", fontsize=8, color=INK, va="center")
        ib = int(np.argmax(np.abs(br)))
        ax.annotate("blend %+.1f (turn %d)" % (br[ib], n[ib]), (n[ib], br[ib]), xytext=(10, -4),
                    textcoords="offset points", fontsize=8, color=INK)
        ax.set_ylim(-30, 30)
        ax.set_xlabel(a.turns_xlabel)
        ax.set_ylabel("degrees, minus the robot's own heading change")
        ax.set_title("%s - each turn: camera and blend minus the robot's wheels + gyroscope\n"
                     "(grey band = within 1 deg)" % title, loc="left", fontsize=11)
        ax.grid(color=GRID, lw=0.8, axis="y")
        for sp in ("top", "right"):
            ax.spines[sp].set_visible(False)
        ax.legend(frameon=False, loc="upper left")
        fig.text(0.01, 0.01, "Values beyond +-30 deg are drawn at the edge and labelled. The robot's heading is "
                 "a comparison for short turns, not ground truth.", fontsize=8, color=MUTED)
        fig.tight_layout(rect=(0, 0.03, 1, 1))
        fig.savefig(os.path.join(a.pack, "turns_difference.png"))
        plt.close(fig)


if __name__ == "__main__":
    sys.exit(main())
