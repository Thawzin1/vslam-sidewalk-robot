#!/usr/bin/env python3
"""size_fit_agreement.py - agreement with the LiDAR estimate AFTER a size fit, for information only.

Why: a taped 15.00 m test (01_runs/series2_static/s2_scale_01/RESULTS.md) showed the robot's wheels read about 6 % long
and the camera within 1 %. The LiDAR estimate takes its size from the wheels, so part of the SE(3) disagreement is the
LiDAR estimate's size, not the camera's shape. This script repeats compare_lidar.py's comparison with the same reference
(lidar_reference_dense.tum), the same pairing (50 ms, traj_metrics.associate) and the same fit code (traj_metrics.umeyama),
once rigid (SE(3), must reproduce agreement.json exactly - the control) and once with size allowed (Sim(3)).
The size-fitted figure is NEVER the headline (ENGINEERING_NOTES.md section 4 rule 4: stereo has real metric scale).
usage: size_fit_agreement.py <pack_dir>   -> <pack_dir>/size_fit.json"""
import json, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "..", "catkin_ws", "src", "sidewalk_evaluation", "scripts"))
import traj_metrics as TM

P = os.path.abspath(sys.argv[1])
ag = json.load(open(os.path.join(P, "agreement.json")))
ref = TM.load_tum(os.path.join(P, "lidar_reference_dense.tum"), "lidar")
out = {"run": os.path.basename(P), "reference": "lidar_reference_dense.tum (LiDAR map, filled in with the wheels; NOT ground truth)",
       "association_s": ag["association_tolerance_s"], "time_offset_s": ag["time_offset_applied_s"],
       "note": "size-fitted (Sim(3)) numbers are for information only; the rigid (SE(3)) numbers are the result", "estimates": {}}
for label, fn in (("camera_corrected", "camera_corrected.tum"), ("camera_tracking", "camera.tum")):
    est = TM.load_tum(os.path.join(P, fn), label)
    ir, ie = TM.associate(ref, est, max_diff=ag["association_tolerance_s"], offset=ag["time_offset_applied_s"])
    src, dst = est.xyz[ie], ref.xyz[ir]
    row = {"n_pairs": int(len(ie))}
    for mode, ws in (("se3", False), ("sim3", True)):
        R, t, c = TM.umeyama(src, dst, with_scale=ws)
        e = np.linalg.norm((c * (R @ src.T)).T + t - dst, axis=1)
        row[mode] = {"scale_applied": round(float(c), 4), "median_m": round(float(np.median(e)), 3),
                     "p95_m": round(float(np.percentile(e, 95)), 3), "max_m": round(float(e.max()), 3)}
    a = ag["estimates"][label]
    row["control_se3_reproduces_agreement_json"] = bool(
        abs(row["se3"]["median_m"] - a["disagreement_median_m"]) <= 0.001 and abs(row["se3"]["p95_m"] - a["disagreement_p95_m"]) <= 0.001)
    row["agreement_json_se3"] = [a["disagreement_median_m"], a["disagreement_p95_m"]]
    out["estimates"][label] = row
    print(label, json.dumps(row))
json.dump(out, open(os.path.join(P, "size_fit.json"), "w"), indent=1)
