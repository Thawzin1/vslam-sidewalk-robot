#!/usr/bin/env python3
"""robot_wheels_from_robot_bag.py - the robot's OWN wheels + gyroscope (/odometry/filtered, recorded on the robot in its LiDAR
bag and exported as wheel.tum by the LiDAR replay) -> end-to-start gap, heading gap and path length over two windows:
  same_window_as_jetson_copy: numbers.json robot first..last (what the Jetson received over WiFi)
  whole_drive_to_parked:      numbers.json robot first .. the end of wheel.tum (robot parked, recording stopped)
Path 'every message' = straight steps between every message; '2hz' = positions interpolated every 0.5 s, straight steps summed.
Heading gap = end yaw - start yaw, wrapped to +-180 deg.
Control: run on drive 9 it must reproduce s2_static_09/robot_wheels_from_robot_bag.json (written earlier by hand-run code).
usage: robot_wheels_from_robot_bag.py <pack_dir> <wheel.tum> [--out file]"""
import json, math, os, sys, time
import numpy as np
os.environ["TZ"] = "America/Toronto"; time.tzset()
P, WT = sys.argv[1], sys.argv[2]
out = sys.argv[sys.argv.index("--out") + 1] if "--out" in sys.argv else os.path.join(P, "robot_wheels_from_robot_bag.json")
W = np.loadtxt(WT, comments="#")
W = W[np.argsort(W[:, 0])]
num = json.load(open(os.path.join(P, "numbers.json")))["tracking_alone_gaps"]["robot"]
day = time.strftime("%Y-%m-%d ", time.localtime(W[0, 0]))
tod = lambda s: time.mktime(time.strptime(day + s, "%Y-%m-%d %H:%M:%S"))
yaw = lambda q: math.atan2(2 * (q[3] * q[2] + q[0] * q[1]), 1 - 2 * (q[1] ** 2 + q[2] ** 2))
h = lambda t: time.strftime("%H:%M:%S", time.localtime(t))


def win(t0, t1):
    k = (W[:, 0] >= t0) & (W[:, 0] <= t1 + 0.999)
    A = W[k]
    g = np.arange(A[0, 0], A[-1, 0], 0.5)
    xy2 = np.c_[np.interp(g, A[:, 0], A[:, 1]), np.interp(g, A[:, 0], A[:, 2])]
    dyaw = math.degrees(yaw(A[-1, 4:8]) - yaw(A[0, 4:8]))
    dyaw = (dyaw + 180) % 360 - 180
    return {"from_hamilton": h(A[0, 0]), "to_hamilton": h(A[-1, 0]),
            "end_to_start_m": round(float(math.hypot(A[-1, 1] - A[0, 1], A[-1, 2] - A[0, 2])), 3),
            "heading_deg": round(dyaw, 2),
            "path_m_every_message": round(float(np.sum(np.hypot(*np.diff(A[:, 1:3], axis=0).T))), 1),
            "path_m_2hz": round(float(np.sum(np.hypot(*np.diff(xy2, axis=0).T))), 1), "messages": int(len(A))}


r = {"source": "wheel.tum = the robot's own /odometry/filtered (wheels + gyroscope), recorded ON THE ROBOT in its LiDAR bag "
               "(lidar_ref_f3dof/, pulled from the robot); unaffected by the Jetson's WiFi link",
     "tool": "03_methods/results_packs_2026-09-26/robot_wheels_from_robot_bag.py",
     "wheel_tum_span_hamilton": [h(W[0, 0]), h(W[-1, 0])],
     "same_window_as_jetson_copy": win(tod(num["first"]), tod(num["last"])),
     "whole_drive_to_parked": win(tod(num["first"]), W[-1, 0])}
json.dump(r, open(out, "w"), indent=1)
print(json.dumps(r, indent=1))
