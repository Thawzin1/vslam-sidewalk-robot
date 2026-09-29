#!/usr/bin/env python3
"""freeze_timing_drive7.py - when did drive 7's camera freezes happen, and did each overlap a database save >= 1.5 s,
a 'ZED Diagnostic' pause (the camera program's own picture loop), a restart from TF, or robot motion?
Reads timing.npz (extract_timing.py) with the same gap rule as pass_check.py. usage: <timing.npz> <pack_dir> <mapping.log>"""
import json, os, re, sys, time
import numpy as np
os.environ["TZ"] = "America/Toronto"; time.tzset()
z = np.load(sys.argv[1]); P = sys.argv[2]
hm = lambda t: time.strftime("%H:%M:%S", time.localtime(t))
g = np.sort(z["zedx_front_zed_node_imu_data"][:, 1]); SPAN = (g[0], g[-1])
def gaps(t, thr):
    t = np.sort(t); return [(t[k], t[k + 1]) for k in np.where(np.diff(t) > thr)[0]]
it = z["rtabmap_iter"]
saves = [(r[0] - r[5] - r[4], r[0] - r[5], r[4]) for r in it if r[4] >= 1.5]
odom = gaps(z["rtabmap_odom"][:, 0], 1.5)
diag = gaps(z["zed_diag_t"], 2.0)
w = z["robot_wheel_odom"]
rs = [float(m.group(1)) for l in open(sys.argv[3], errors="replace") if "Odometry automatically reset to latest odometry pose available from TF" in l
      for m in [re.search(r"\[\s*(\d+\.\d+)\]", l)] if m]
ov = lambda a, b, c, d: min(b, d) - max(a, c) > 0
rows = []
for a, b in odom:
    sel = (w[:, 0] >= a) & (w[:, 0] <= b)
    rows.append({"from": hm(a), "seconds": round(float(b - a), 2),
                 "overlaps_zed_diagnostic_pause": any(ov(a, b, c, d) for c, d in diag),
                 "overlaps_db_save_over_1p5s": [round(float(s[2]), 2) for s in saves if ov(a, b, s[0], s[1])],
                 "restart_from_tf_within_5s_after": any(b - 1 <= t <= b + 5 for t in rs)})
out = {"camera_tracker_silences_over_1p5s": rows,
       "n": len(rows), "n_overlapping_zed_diagnostic_pause": sum(r["overlaps_zed_diagnostic_pause"] for r in rows),
       "n_overlapping_db_save_over_1p5s": sum(bool(r["overlaps_db_save_over_1p5s"]) for r in rows),
       "db_saves_over_1p5s": len(saves), "longest_db_save_s": round(max([s[2] for s in saves] or [0]), 2),
       "wheel_odom_columns": "not used for motion (column meaning not checked)"}
json.dump(out, open(os.path.join(P, "freeze_timing.json"), "w"), indent=1)
print(json.dumps(out, indent=1))
