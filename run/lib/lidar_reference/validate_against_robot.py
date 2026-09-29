#!/usr/bin/env python3
"""validate_against_robot.py - does the Jetson's LiDAR yardstick agree with the robot's own, on the same recording?

THE QUESTION: the robot made drive 10's Force3DoF LiDAR estimate (947 nodes, 76 loop closures, floor gap 0.044 m,
263.3 m). The Jetson replays the SAME recording through the SAME pipeline with the same one change. If the two
agree to within what cannot matter downstream, the replay can move to the Jetson and the robot can charge.

WHY THEY WILL NOT BE IDENTICAL (ENGINEERING_NOTES.md section 4 rule 0 - judge against uncertainty, not against zero):
  - helios_restamp.py stamps each cloud with the replay clock at the moment it ARRIVES, so which cloud pairs
    with which odometry message depends on each machine's timing; RTAB-Map adds a node once per second of
    replay time (Rtabmap/DetectionRate=1), so the nodes fall on slightly different clouds;
  - loop closures are accepted by ICP (point matching) on those clouds, so the closure set can differ;
  - RTAB-Map 0.21.13 on the Jetson vs 0.21.10 on the robot (same parameters SET; see the parameter check).
  Between closures the LiDAR estimate IS the wheel+IMU path (RGBD/NeighborLinkRefining=false), so the two paths
  can differ only where a closure pulled differently.

PASS LINES, stated before the Jetson replay was run (27 Sept 2026):
  C0 same input      wheel.tum identical row for row (same bag, same topic, deterministic extraction).
                     Fails -> the two did not read the same recording; nothing else is meaningful.
  C1 parameters      every RTAB-Map parameter the colleague's launch SETS, plus Reg/Force3DoF=true, has the same
                     value in the Jetson map's Info table (and, with --robot-params, in the robot map's). Other
                     differences (version defaults) are listed, not failed.
  C2 path agreement  THE MAIN CHECK. Jetson path at the robot's node times (linear interpolation, gaps <= 2 s),
                     rigid fit (rotation + shift, no resizing - section 4 rule 4): median <= 0.04 m and
                     95th percentile <= 0.13 m, with >= 90 % of the robot's nodes paired.
                     Why these numbers: they are half of ONE drive's own spread for the camera-vs-LiDAR agreement
                     median (0.08 m) and 95th percentile (0.27 m) used in compare_drives_4_8_9_10 (rows G, H). A
                     yardstick shift smaller than that cannot move any drive-to-drive verdict.
  C3 end gap         Jetson floor gap within 0.05 m of the robot's 0.044 m (i.e. <= 0.094 m).
  C4 loop closures   within +-20 % of the robot's 76 (61-91). A judgement band - the run-to-run spread of the
                     closure count has never been measured; run the Jetson replay twice (--repeat) to measure
                     it. C2 is what matters downstream; C4 failing with C2 passing = "different closures, same map".
  C5 nodes, length   nodes within +-3 % of 947; path length within +-1 % of 263.3 m.
  C6 downstream      (with --downstream) drive 10's camera-vs-LiDAR agreement recomputed with the Jetson
                     yardstick (compare_lidar.py, unchanged) differs from agreement.json's 0.685 / 1.151 m by
                     <= 0.04 m (median) and <= 0.13 m (95th percentile) - the same half-spread bands as C2.
  Verdict: PASS = C0, C1, C2, C3, C5 (and C6 when run) pass. C4 is reported beside it.
Plain terms: "agree" means the Jetson's LiDAR path lies on top of the robot's to within a few centimetres
typically and about a hand's width at worst, and moving from one to the other would not change any number we
have reported by more than half of the spread one drive already has.

NOTHING HERE IS GROUND TRUTH: both are the same independent LiDAR estimate (rule 19), computed twice.

usage:
  validate_against_robot.py --jetson ~/lidar_replays/s2_static_10/lidar_ref_f3dof \
      --robot "results/series2_static/s2_static_10/lidar_ref_f3dof" \
      [--robot-params validation_s2_static_10/robot_params_db.txt] [--repeat <2nd Jetson folder>] \
      [--downstream "results/series2_static/s2_static_10"] [--out <folder>]
"""
import argparse
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SLAM = os.path.dirname(os.path.dirname(HERE))
REPO = os.path.dirname(SLAM)

# what the colleague's rtabmap_3d.launch sets (the two Icp/PM* names are renamed by RTAB-Map itself), + our one change
LAUNCH_SET = {
    "Rtabmap/DetectionRate": "1", "RGBD/NeighborLinkRefining": "false", "RGBD/ProximityBySpace": "true",
    "RGBD/ProximityMaxGraphDepth": "0", "RGBD/ProximityPathMaxNeighbors": "1", "RGBD/AngularUpdate": "0.05",
    "RGBD/LinearUpdate": "0.05", "Mem/NotLinkedNodesKept": "false", "Mem/STMSize": "30", "Reg/Strategy": "1",
    "Grid/CellSize": "0.02", "Grid/RangeMax": "20", "Grid/ClusterRadius": "1", "Grid/GroundIsObstacle": "true",
    "Optimizer/GravitySigma": "0.3", "Icp/VoxelSize": "0.3", "Icp/PointToPlaneK": "20",
    "Icp/PointToPlaneRadius": "0", "Icp/PointToPlane": "false", "Icp/Iterations": "10", "Icp/Epsilon": "0.001",
    "Icp/MaxTranslation": "3", "Icp/MaxCorrespondenceDistance": "1",
    # the launch's old name Icp/PM=true is copied AS TEXT to Icp/Strategy, so the map stores "true"; RTAB-Map reads
    # that as the number 1 = libpointmatcher (checked on the Jetson's 0.21.13 by compiling a call to
    "Icp/Strategy": "true",
    "Icp/OutlierRatio": "0.7", "Icp/CorrespondenceRatio": "0.4",
}
F3DOF = {"Reg/Force3DoF": "true"}
C2_MED, C2_P95, C2_PAIRED = 0.04, 0.13, 0.90
C3_TOL, C4_BAND, C5_NODES, C5_LEN = 0.05, 0.20, 0.03, 0.01
C6_MED, C6_P95 = 0.04, 0.13


def read_tum(path):
    rows = []
    with open(path) as fh:
        for line in fh:
            p = line.split()
            if not p or line.startswith("#") or len(p) < 8:
                continue
            rows.append([float(v) for v in p[:8]])
    return np.array(sorted(rows))


def read_params(path):
    d = {}
    with open(path) as fh:
        for line in fh:
            if " = " in line:
                k, v = line.rstrip("\n").split(" = ", 1)
                d[k.strip()] = v.strip()
    return d


def num_eq(a, b):
    try:
        return math.isclose(float(a), float(b), rel_tol=1e-9, abs_tol=1e-12)
    except (TypeError, ValueError):
        return str(a).lower() == str(b).lower()


def interp_xy(src, t, max_gap=2.0):
    """src rows t x y ... ; -> (xy at t, ok mask): linear, only where both neighbours are within max_gap s"""
    ts = src[:, 0]
    i = np.searchsorted(ts, t)
    ok = (i > 0) & (i < len(ts))
    i = np.clip(i, 1, len(ts) - 1)
    t0, t1 = ts[i - 1], ts[i]
    ok &= (t1 - t0) <= max_gap
    w = np.where(t1 > t0, (t - t0) / np.maximum(t1 - t0, 1e-9), 0.0)
    xy = src[i - 1, 1:3] * (1 - w)[:, None] + src[i, 1:3] * w[:, None]
    return xy, ok


def rigid_fit_2d(a, b):
    """rotation + shift taking a onto b (least squares, no scale) -> aligned a"""
    ca, cb = a.mean(0), b.mean(0)
    H = (a - ca).T @ (b - cb)
    U, _, Vt = np.linalg.svd(H)
    D = np.diag([1.0, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    return (R @ (a - ca).T).T + cb, math.degrees(math.atan2(R[1, 0], R[0, 0]))


def path_agreement(jet, rob):
    xy, ok = interp_xy(jet, rob[:, 0])
    a, b = xy[ok], rob[ok, 1:3]
    raw = np.hypot(*(a - b).T)
    al, rot = rigid_fit_2d(a, b)
    fit = np.hypot(*(al - b).T)
    q = lambda v, p: float(np.percentile(v, p)) if len(v) else float("nan")
    return {"paired": int(ok.sum()), "robot_nodes": len(rob), "paired_frac": float(ok.mean()),
            "same_frame_median_m": q(raw, 50), "same_frame_p95_m": q(raw, 95), "same_frame_max_m": q(raw, 100),
            "rigid_fit_median_m": q(fit, 50), "rigid_fit_p95_m": q(fit, 95), "rigid_fit_max_m": q(fit, 100),
            "rigid_fit_rotation_deg": rot}


def load_side(d):
    meta = json.load(open(os.path.join(d, "lidar.tum.meta.json")))
    facts = json.load(open(os.path.join(d, "lidar_map_facts.json")))
    return {"tum": read_tum(os.path.join(d, "lidar.tum")), "closures": meta.get("loop_closures"),
            "nodes": meta.get("poses"), "gap": facts.get("closed_loop_gap_m"), "path": facts.get("path_m_nodes")}


def wheel_same(a, b):
    ra, rb = read_tum(a), read_tum(b)
    return len(ra) == len(rb) and bool(np.array_equal(ra, rb)), len(ra), len(rb)


def downstream(jdir, run_dir):
    """compare_lidar.py (unchanged) on a temporary folder holding the Jetson yardstick + drive's camera paths"""
    run = os.path.basename(os.path.normpath(run_dir))
    name = run + "_jetson_yardstick_check"
    res_dir = os.path.join(REPO, "logs", "sidewalk_evaluation", "results", name + "_vs_lidar")
    existed = os.path.exists(res_dir)
    tmp = tempfile.mkdtemp(prefix="lidar_jetson_check_")
    d = os.path.join(tmp, name)
    os.makedirs(d)
    for f in ("lidar.tum", "lidar.tum.meta.json", "wheel.tum", "lidar_map.npz"):
        os.symlink(os.path.join(jdir, f), os.path.join(d, f))
    for f in ("camera_corrected.tum", "camera.tum"):
        if os.path.exists(os.path.join(run_dir, f)):
            os.symlink(os.path.join(run_dir, f), os.path.join(d, f))
    rc = subprocess.call([sys.executable, os.path.join(SLAM, "scripts", "compare_lidar.py"), d],
                         stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    new = json.load(open(os.path.join(d, "agreement.json"))) if rc == 0 else None
    old = json.load(open(os.path.join(run_dir, "agreement.json")))
    shutil.rmtree(tmp, ignore_errors=True)
    if not existed:                       # compare_lidar.py's copy under logs/ is ours to tidy (it copied it into d)
        shutil.rmtree(res_dir, ignore_errors=True)
    if new is None:
        return {"ok": False, "why": "compare_lidar.py failed (exit %d)" % rc}
    n, o = new["estimates"]["camera_corrected"], old["estimates"]["camera_corrected"]
    dm = abs(n["disagreement_median_m"] - o["disagreement_median_m"])
    dp = abs(n["disagreement_p95_m"] - o["disagreement_p95_m"])
    return {"ok": dm <= C6_MED and dp <= C6_P95, "robot_yardstick": [o["disagreement_median_m"], o["disagreement_p95_m"]],
            "jetson_yardstick": [n["disagreement_median_m"], n["disagreement_p95_m"]], "diff_median_m": dm,
            "diff_p95_m": dp, "camera_tracking_jetson": [new["estimates"].get("camera_tracking", {}).get(k) for k in
                                                         ("disagreement_median_m", "disagreement_p95_m")]}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--jetson", required=True)
    ap.add_argument("--robot", required=True)
    ap.add_argument("--robot-params")
    ap.add_argument("--repeat", help="a second Jetson replay of the same bag: measures the Jetson's own spread")
    ap.add_argument("--downstream", help="the drive's run folder (camera_corrected.tum, agreement.json)")
    ap.add_argument("--out", help="where validation.json/.md go (default: the Jetson folder)")
    a = ap.parse_args()
    out = a.out or a.jetson
    os.makedirs(out, exist_ok=True)
    J, R = load_side(a.jetson), load_side(a.robot)
    res = {"jetson": os.path.abspath(a.jetson), "robot": os.path.abspath(a.robot), "checks": {}}
    C = res["checks"]

    same, nj, nr = wheel_same(os.path.join(a.jetson, "wheel.tum"), os.path.join(a.robot, "wheel.tum"))
    C["C0_same_input"] = {"ok": same, "rows_jetson": nj, "rows_robot": nr}

    jp = read_params(os.path.join(a.jetson, "rtabmap_params_db.txt"))
    want = dict(LAUNCH_SET, **F3DOF) if "f3dof" in os.path.basename(os.path.normpath(a.jetson)) else dict(LAUNCH_SET)
    bad = {k: jp.get(k) for k, v in want.items() if not num_eq(jp.get(k), v)}
    c1 = {"ok": not bad, "set_by_launch_checked": len(want), "jetson_wrong": bad}
    if a.robot_params and os.path.exists(a.robot_params):
        rp = read_params(a.robot_params)
        rbad = {k: rp.get(k) for k, v in want.items() if not num_eq(rp.get(k), v)}
        diff = {k: [rp.get(k), jp.get(k)] for k in sorted(set(rp) | set(jp)) if not num_eq(rp.get(k), jp.get(k))}
        c1.update({"robot_wrong": rbad, "ok": c1["ok"] and not rbad,
                   "other_differences_robot_vs_jetson": diff, "n_other_differences": len(diff)})
    else:
        c1["robot_params"] = "not given - run fetch_robot_reference.sh when the robot is on"
    C["C1_parameters"] = c1

    pa = path_agreement(J["tum"], R["tum"])
    pa["ok"] = pa["rigid_fit_median_m"] <= C2_MED and pa["rigid_fit_p95_m"] <= C2_P95 and pa["paired_frac"] >= C2_PAIRED
    C["C2_path_agreement"] = pa
    C["C3_end_gap"] = {"ok": abs(J["gap"] - R["gap"]) <= C3_TOL, "jetson_m": J["gap"], "robot_m": R["gap"]}
    C["C4_loop_closures"] = {"ok": abs(J["closures"] - R["closures"]) <= C4_BAND * R["closures"],
                             "jetson": J["closures"], "robot": R["closures"],
                             "band": [math.ceil(R["closures"] * (1 - C4_BAND)), math.floor(R["closures"] * (1 + C4_BAND))]}
    C["C5_nodes_length"] = {"ok": abs(J["nodes"] - R["nodes"]) <= C5_NODES * R["nodes"]
                            and abs(J["path"] - R["path"]) <= C5_LEN * R["path"],
                            "nodes": [J["nodes"], R["nodes"]], "path_m": [J["path"], R["path"]]}
    if a.repeat:
        Q = load_side(a.repeat)
        C["repeat_jetson_vs_jetson"] = dict(path_agreement(Q["tum"], J["tum"]),
                                            closures=[J["closures"], Q["closures"]], gaps=[J["gap"], Q["gap"]])
    if a.downstream:
        C["C6_downstream"] = downstream(os.path.abspath(a.jetson), os.path.abspath(a.downstream))
    need = ["C0_same_input", "C1_parameters", "C2_path_agreement", "C3_end_gap", "C5_nodes_length"]
    if "C6_downstream" in C:
        need.append("C6_downstream")
    res["verdict"] = "PASS" if all(C[k]["ok"] for k in need) else "FAIL"
    res["verdict_counts"] = need
    with open(os.path.join(out, "validation.json"), "w") as fh:
        json.dump(res, fh, indent=2, default=float)

    f = lambda v: "%.3f" % v if isinstance(v, float) else str(v)
    L = ["# Jetson LiDAR yardstick vs the robot's - %s" % os.path.basename(os.path.dirname(os.path.normpath(a.jetson))),
         "", "**Verdict: %s** (counted: %s; C4 reported beside it). Agreement between two computations of the same"
         " independent LiDAR estimate - not ground truth (rule 19)." % (res["verdict"], ", ".join(need)), "",
         "| check | Jetson | robot | pass line | result |", "|---|---|---|---|---|",
         "| C0 same input (wheel.tum rows) | %d | %d | identical | %s |" % (nj, nr, "pass" if same else "**FAIL**"),
         "| C1 parameters set by the launch (+ Force3DoF) | %s | %s | all equal | %s |" % (
             "all equal" if not bad else "wrong: %s" % bad,
             ("all equal" if not c1.get("robot_wrong") else "wrong: %s" % c1.get("robot_wrong")) if "robot_wrong" in c1 else "not fetched",
             "pass" if c1["ok"] else "**FAIL**"),
         "| C2 path, rigid fit: median / 95th pct | %s / %s m | - | <= %.2f / %.2f m, >= %d %% paired (%d of %d) | %s |" % (
             f(pa["rigid_fit_median_m"]), f(pa["rigid_fit_p95_m"]), C2_MED, C2_P95, C2_PAIRED * 100, pa["paired"],
             pa["robot_nodes"], "pass" if pa["ok"] else "**FAIL**"),
         "| C2 (information) same frame, no fit: median / 95th / max | %s / %s / %s m | - | - | - |" % (
             f(pa["same_frame_median_m"]), f(pa["same_frame_p95_m"]), f(pa["same_frame_max_m"])),
         "| C3 floor end gap | %s m | %s m | within %.2f m | %s |" % (f(J["gap"]), f(R["gap"]), C3_TOL,
                                                                      "pass" if C["C3_end_gap"]["ok"] else "**FAIL**"),
         "| C4 loop closures | %s | %s | %d-%d (judgement band) | %s |" % (J["closures"], R["closures"],
             C["C4_loop_closures"]["band"][0], C["C4_loop_closures"]["band"][1],
             "pass" if C["C4_loop_closures"]["ok"] else "outside band"),
         "| C5 nodes / path length | %s / %s m | %s / %s m | +-3 %% / +-1 %% | %s |" % (J["nodes"], f(J["path"]), R["nodes"],
             f(R["path"]), "pass" if C["C5_nodes_length"]["ok"] else "**FAIL**")]
    if "C6_downstream" in C:
        d6 = C["C6_downstream"]
        if "jetson_yardstick" in d6:
            L.append("| C6 camera map vs LiDAR, median / 95th | %s / %s m | %s / %s m | differ <= %.2f / %.2f m | %s |" % (
                f(d6["jetson_yardstick"][0]), f(d6["jetson_yardstick"][1]), f(d6["robot_yardstick"][0]),
                f(d6["robot_yardstick"][1]), C6_MED, C6_P95, "pass" if d6["ok"] else "**FAIL**"))
        else:
            L.append("| C6 downstream | - | - | - | **FAIL**: %s |" % d6.get("why"))
    if "repeat_jetson_vs_jetson" in C:
        r = C["repeat_jetson_vs_jetson"]
        L += ["", "Jetson repeat (same bag twice): path rigid-fit median %s m, 95th %s m; closures %s; gaps %s m." % (
            f(r["rigid_fit_median_m"]), f(r["rigid_fit_p95_m"]), r["closures"], r["gaps"])]
    if c1.get("n_other_differences"):
        L += ["", "Parameters that differ robot (0.21.10) vs Jetson (0.21.13), none set by the launch (%d):" % c1["n_other_differences"], ""]
        L += ["- `%s`: robot `%s`, Jetson `%s`" % (k, v[0], v[1]) for k, v in c1["other_differences_robot_vs_jetson"].items()]
    L += ["", "*Plain terms: C2 asks whether the two LiDAR paths lie on top of each other; the pass line is half of the"
          " spread one drive's camera-vs-LiDAR number already has, so a PASS means switching machines cannot change"
          " any reported verdict.*"]
    with open(os.path.join(out, "validation.md"), "w") as fh:
        fh.write("\n".join(L) + "\n")
    print("\n".join(L))
    return 0 if res["verdict"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
