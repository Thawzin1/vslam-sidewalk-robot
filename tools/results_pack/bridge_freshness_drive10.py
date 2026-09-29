#!/usr/bin/env python3
"""bridge_freshness_drive10.py - drive 10 (s2_static_10): for how much of the drive did the blend run WITHOUT fresh
robot data (wheels, gyroscope), because the WiFi link to the robot flickered?

Reads (read-only, Jetson): ~/.run_records/s2_static_10/fusion.bag - /ekf_in/wheel_odom and /ekf_in/imu (exactly what
the blend's EKF was given; receive time = when the Jetson had it), /fused/odometry (the blend's path, for the map).
Rule: the blend's EKF drops an input older than sensor_timeout 0.5 s (catkin_ws/src/sidewalk_slam/config/ekf_fused.yaml:49),
so a moment counts as "no fresh robot data" when the newest robot message received is more than 0.5 s old.
Also reported at 2 s and 5 s. Window: the mapping program's life, 02:02:42 (camera guard 'watching') to 02:31:15
(autostop 'interrupting the mapping'), Hamilton.

Plain terms: the robot sends its wheel and turn-rate readings ten and twenty times a second over WiFi. Whenever none had
arrived for half a second, the blend was steering on the camera alone. This adds up those moments and draws where they were.

Writes into <pack>: bridge_freshness.json, bridge_freshness.png.  usage: bridge_freshness_drive10.py <pack>"""
import json, os, sys, time
import numpy as np
import rosbag
os.environ["TZ"] = "America/Toronto"; time.tzset()
P = sys.argv[1]
REC = sys.argv[2] if len(sys.argv) > 2 else os.path.join(os.path.expanduser(os.environ.get("RECORDS_DIR", "~/.run_records")), os.path.basename(os.path.abspath(P)))
W0 = time.mktime(time.strptime("2026-09-27 02:02:42", "%Y-%m-%d %H:%M:%S"))
W1 = time.mktime(time.strptime("2026-09-27 02:31:15", "%Y-%m-%d %H:%M:%S"))
hm = lambda t: time.strftime("%H:%M:%S", time.localtime(t))
rx = {"/ekf_in/wheel_odom": [], "/ekf_in/imu": []}
blend = []
with rosbag.Bag(os.path.join(REC, "fusion.bag")) as b:
    for topic, m, t in b.read_messages(topics=list(rx) + ["/fused/odometry"]):
        if topic == "/fused/odometry":
            p = m.pose.pose.position; blend.append((t.to_sec(), p.x, p.y))
        else:
            rx[topic].append(t.to_sec())
blend = np.array(blend)
grid = np.arange(W0, W1, 0.05)            # 20 Hz sampling of "how old is the newest message"


def age(ts):
    ts = np.sort(np.array(ts))
    k = np.searchsorted(ts, grid, side="right") - 1
    a = np.where(k >= 0, grid - ts[np.clip(k, 0, None)], np.inf)
    return a


def stretches(mask, dt=0.05):
    out, i = [], 0
    idx = np.where(np.diff(np.r_[0, mask.astype(int), 0]))[0]
    for s, e in zip(idx[::2], idx[1::2]):
        out.append((grid[s], grid[e - 1] + dt))
    return out


res = {"window": [hm(W0), hm(W1)], "window_s": round(W1 - W0, 1),
       "rule": "no fresh robot data = newest robot message received more than 0.5 s ago (EKF sensor_timeout 0.5, ekf_fused.yaml:49)",
       "inputs": {}}
aw, ai = age(rx["/ekf_in/wheel_odom"]), age(rx["/ekf_in/imu"])
both = np.minimum(aw, ai)       # neither wheels NOR gyroscope fresh
for name, a in (("wheels (/ekf_in/wheel_odom)", aw), ("gyroscope (/ekf_in/imu)", ai), ("neither wheels nor gyroscope", both)):
    d = {}
    for thr in (0.5, 2.0, 5.0):
        m = a > thr
        d["over_%gs" % thr] = {"seconds": round(float(m.sum() * 0.05), 1), "percent_of_drive": round(float(100 * m.mean()), 2),
                               "stretches": len(stretches(m))}
    st = stretches(a > 0.5)
    d["longest_stretch_s"] = round(max([e - s for s, e in st], default=0), 2)
    d["longest_stretches"] = [{"from": hm(s), "seconds": round(e - s, 2)} for s, e in sorted(st, key=lambda x: x[0] - x[1])[:8]]
    res["inputs"][name] = d
# message rate per 10 s (wheels), lowest windows
tw = np.array(rx["/ekf_in/wheel_odom"]); tw = tw[(tw >= W0) & (tw <= W1)]
edges = np.arange(W0, W1 + 10, 10.0); cnt, _ = np.histogram(tw, edges); rate = cnt / 10.0
res["wheel_rate_per_10s"] = {"median_hz": round(float(np.median(rate)), 2), "lowest_hz": round(float(rate.min()), 2),
                              "windows_below_2hz": int((rate < 2).sum()), "windows_below_1hz": int((rate < 1).sum()), "windows_total": len(rate)}
json.dump(res, open(os.path.join(P, "bridge_freshness.json"), "w"), indent=2)

import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
fig, (a1, a2) = plt.subplots(1, 2, figsize=(16, 7.5), gridspec_kw={"width_ratios": [1.3, 1]})
mid = (edges[:-1] + edges[1:]) / 2
a1.plot((mid - W0) / 60, rate, color="#2a78d6", lw=1.2, label="wheel messages the blend received, per 10 s (nominal 10 a second)")
for s, e in stretches(both > 0.5):
    a1.axvspan((s - W0) / 60, (e - W0) / 60, color="#d62728", alpha=0.25, lw=0)
a1.plot([], [], color="#d62728", alpha=0.4, lw=8, label="neither wheels nor gyroscope fresh (> 0.5 s old): blend on the camera alone")
a1.set_xlabel("minutes since the mapping started (02:02:42 Hamilton)"); a1.set_ylabel("messages a second"); a1.grid(alpha=0.3)
a1.set_title("Drive 10: robot data reaching the blend over WiFi"); a1.legend(loc="lower left", fontsize=8.5)
bl = blend[(blend[:, 0] >= W0) & (blend[:, 0] <= W1)]
k = np.clip(np.searchsorted(grid, bl[:, 0]) - 1, 0, len(grid) - 1); bad = both[k] > 0.5
a2.plot(bl[:, 1], bl[:, 2], "--", color="#6e7074", lw=0.8, label="blend path (/fused/odometry), tracking alone")
a2.scatter(bl[bad, 1], bl[bad, 2], s=3, color="#d62728", label="where the blend had no fresh robot data", zorder=3)
a2.plot(bl[0, 1], bl[0, 2], "ko", ms=7, label="start")
a2.set_aspect("equal"); a2.grid(alpha=0.3); a2.set_xlabel("x (m)"); a2.set_ylabel("y (m)"); a2.legend(loc="best", fontsize=8.5)
a2.set_title("Where it happened (blend's own position, not corrected)")
plt.tight_layout(); plt.savefig(os.path.join(P, "bridge_freshness.png"), dpi=110)
print(json.dumps(res["inputs"]["neither wheels nor gyroscope"]["over_0.5s"]), res["wheel_rate_per_10s"])
