#!/usr/bin/env python3
"""score_slam.py - pass line S2 (no wrong joins) for a continued-map SLAM session, read-only. Jetson or any computer.

  usage: score_slam.py DB OLD_FACTS.json WHEEL.tum OUT_DIR [--start-xy 0,0] [--start-yaw-deg 0]

For every link between a today's node and a drive-10 node (link types 1 recognised-again, 2 nearby re-match),
in the order of today's node time:
  implied  = where the link puts today's node on drive 10's map: drive 10's SAVED position of the old node (before
             today, old_map_facts.json) combined with the link's measured offset (Link.transform).
  expected = where the robot's own wheels + gyroscope say it is: the wheel path from the ANCHOR (the start mark at
             the session's first node, facing east; afterwards the previous link that passed), rotated so its
             heading at the anchor matches the anchor's heading on drive 10's map.
  wrong    = |implied - expected| > 1.5 m + 2 % of the wheel distance since the anchor (PASS_LINES.md S2).
A passing link becomes the next anchor; a failing one does not. WHEEL.tum = the robot's /odometry/filtered or the
Jetson's /robot/ekf_odom, TUM format, on the same clock as Node.stamp (the Jetson's).
Writes OUT_DIR/s2_links.csv and s2_summary.json. *Plain terms: every time the map says "I am at this spot of drive
10's floor", check it against how far and which way the wheels say the robot went since the last trusted spot.*
Uncertainty: parking 0.2-0.5 m at the start; wheel heading drift about 5 deg/min (why the anchor moves at every
passing link, keeping the wheel stretches short).
"""
from __future__ import print_function
import argparse, csv, json, os, sqlite3
import numpy as np


def se2(P):
    return np.array([P[0, 3], P[1, 3], np.arctan2(P[1, 0], P[0, 0])])


def compose(a, b):          # a (+) b, 2D
    c, s = np.cos(a[2]), np.sin(a[2])
    return np.array([a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2]])


def inverse(a):
    c, s = np.cos(a[2]), np.sin(a[2])
    return np.array([-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2]])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("db"); ap.add_argument("old_facts"); ap.add_argument("wheel"); ap.add_argument("out")
    ap.add_argument("--start-xy", default="0,0"); ap.add_argument("--start-yaw-deg", type=float, default=0.0)
    ap.add_argument("--tol-m", type=float, default=1.5); ap.add_argument("--tol-frac", type=float, default=0.02)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    old = json.load(open(a.old_facts)); old_max = int(old["node_id_max"])
    oldxy = {int(k): v for k, v in old["old_graph_xy"].items()}
    c = sqlite3.connect("file:%s?mode=ro" % a.db, uri=True)
    stamp = dict(c.execute("SELECT id, stamp FROM Node"))
    first_today = c.execute("SELECT MIN(id) FROM Node WHERE id > ?", (old_max,)).fetchone()[0]
    # old nodes' saved headings (radians) - written by slam_closed_check.py --before (old_graph_yaw)
    oldyaw = {int(k): v for k, v in old.get("old_graph_yaw", {}).items()}
    W = np.loadtxt(a.wheel)
    wt = W[:, 0]
    wyaw = 2 * np.arctan2(W[:, 6], W[:, 7])          # planar yaw from qz, qw

    def wheel_at(t):
        i = int(np.clip(np.searchsorted(wt, t), 1, len(wt) - 1))
        f = (t - wt[i - 1]) / max(1e-9, wt[i] - wt[i - 1])
        return np.array([W[i - 1, 1] + f * (W[i, 1] - W[i - 1, 1]), W[i - 1, 2] + f * (W[i, 2] - W[i - 1, 2]),
                         wyaw[i - 1] + f * np.angle(np.exp(1j * (wyaw[i] - wyaw[i - 1])))]), abs(t - wt[i]) < 1.0

    def wheel_dist(t0, t1):
        m = (wt >= t0) & (wt <= t1)
        p = W[m][:, 1:3]
        return float(np.sum(np.hypot(*np.diff(p, axis=0).T))) if len(p) > 1 else 0.0
    rows = []
    seen = set()
    for f_id, t_id, ty, tb in c.execute("SELECT from_id, to_id, type, transform FROM Link WHERE type IN (1,2) "
                                        "AND from_id != to_id"):
        new, oldn = (f_id, t_id) if f_id > old_max else (t_id, f_id)
        if not (new > old_max >= oldn) or (new, oldn) in seen:
            continue
        seen.add((new, oldn))
        T = se2(np.frombuffer(bytes(tb), np.float32).reshape(3, 4))
        if oldn not in oldxy or oldn not in oldyaw:
            continue
        k = np.array([oldxy[oldn][0], oldxy[oldn][1], oldyaw[oldn]])
        # Link.transform maps FROM -> TO: pose_to = pose_from (+) T
        implied = compose(k, T) if f_id == oldn else compose(k, inverse(T))
        rows.append({"t": stamp[new], "new": new, "old": oldn, "type": ty, "implied": implied})
    rows.sort(key=lambda r: r["t"])
    sx, sy = [float(v) for v in a.start_xy.split(",")]
    anchor = {"t": stamp[first_today], "pose": np.array([sx, sy, np.radians(a.start_yaw_deg)])}
    wrong = 0
    out = []
    for r in rows:
        w0, ok0 = wheel_at(anchor["t"]); w1, ok1 = wheel_at(r["t"])
        rel = compose(inverse(w0), w1)                       # wheel motion since the anchor, in the anchor's frame
        expected = compose(anchor["pose"], rel)
        dist = wheel_dist(anchor["t"], r["t"])
        err = float(np.hypot(*(r["implied"][:2] - expected[:2])))
        tol = a.tol_m + a.tol_frac * dist
        bad = err > tol
        wrong += int(bad)
        out.append({"t": round(r["t"], 3), "today_id": r["new"], "drive10_id": r["old"], "type": r["type"],
                    "implied_x": round(r["implied"][0], 3), "implied_y": round(r["implied"][1], 3),
                    "wheel_x": round(expected[0], 3), "wheel_y": round(expected[1], 3), "wheel_dist_m": round(dist, 2),
                    "disagreement_m": round(err, 3), "tolerance_m": round(tol, 3), "wrong": bad,
                    "wheel_data_near": ok0 and ok1})
        if not bad:
            anchor = {"t": r["t"], "pose": r["implied"]}
    with open(os.path.join(a.out, "s2_links.csv"), "w") as fh:
        wtr = csv.DictWriter(fh, fieldnames=list(out[0].keys()) if out else ["t"])
        wtr.writeheader(); [wtr.writerow(o) for o in out]
    d = np.array([o["disagreement_m"] for o in out]) if out else np.array([])
    summ = {"links_between_sessions": len(out), "wrong": wrong, "S2_pass": wrong == 0 and len(out) > 0,
            "disagreement_m": {"median": round(float(np.median(d)), 3), "p95": round(float(np.percentile(d, 95)), 3),
                               "max": round(float(d.max()), 3)} if len(d) else None,
            "links_without_wheel_data": sum(1 for o in out if not o["wheel_data_near"])}
    json.dump(summ, open(os.path.join(a.out, "s2_summary.json"), "w"), indent=1)
    print(json.dumps(summ))


if __name__ == "__main__":
    main()
