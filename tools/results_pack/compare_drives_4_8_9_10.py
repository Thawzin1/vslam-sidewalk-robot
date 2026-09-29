#!/usr/bin/env python3
"""compare_drives_4_8_9_10.py - did the WHOLE-FLOOR drive (drive 10, s2_static_10) keep the quality of drive 4 (s2_static_04,
the short reference drive) and of drive 9 (s2_static_09, the first longer drive, about the same length)?
Drive 8 (drive 4's route repeated, against its own Force3DoF replay) is shown for information, as in compare_drives_4_8_9.py.

A copy of compare_drives_4_8_9.py's method (that script and its outputs in s2_static_09/ are left untouched), drives changed:
  Route overlap, defined here and nowhere else:
    1. each drive's route = its camera map's corrected path (camera_corrected.tum), filled in every 5 cm (drive 9: the graph
       RE-OPTIMISED by optimize_graph_se2.py; drive 10: RTAB-Map's own Admin.opt_poses);
    2. the later drive's route is lined up on the earlier one's by rotation + shift only (2D ICP, pairs closer than 1.0 m only,
       started from the best of 36 headings);
    3. a point is "on the shared route" if it lies within R = 0.5 m of the other drive's route (0.3 / 0.75 / 1.0 m printed too).
  Pairs lined up: 4 vs 8, 4 vs 9, 4 vs 10, 9 vs 10.
  Shared-route agreement: the matched camera/LiDAR pairs whose camera position lies on the shared route (same SE(3) alignment as
  the drive's agreement.json - not refitted on the subset). Control: the whole-drive figures must reproduce agreement.json, and
  drive 9's shared-route figures against drive 4 must reproduce s2_static_09/compare_drives_4_8_9.json.
Yardsticks: drive 4 = colleague-mode LiDAR replay; drives 8, 9, 10 = their Force3DoF replays.
Every row is judged against the spread expected for one drive, with its source; combined as sqrt(s_a^2 + s_b^2).
Reads chart.csv (build_chart.py, run first) and the packs. Nothing measured is typed by hand.
Writes compare_drives_4_8_9_10.md / .json, shared_route.json, shared_route.png into drive 10's pack.  usage: compare_drives_4_8_9_10.py"""
import csv, json, math, os, sys
import numpy as np
from scipy.spatial import cKDTree
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "catkin_ws", "src", "sidewalk_evaluation", "scripts"))
import traj_metrics as TM
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
SR = os.path.abspath(os.path.join(HERE, "..", ".."))
RUNS = os.path.join(SR, "01_runs", "series2_static")
CH = os.path.join(SR, "04_figures", "camera_vs_lidar_2026-09-26", "chart.csv")
pack = lambda d: os.path.join(RUNS, "s2_static_%02d" % d)
P10, P9 = pack(10), pack(9)
F8 = os.path.join(RUNS, "s2_static_08", "lidar_ref_f3dof", "compare_same_method")
R_SHARED = 0.5
rows = {int(r["drive"]): r for r in csv.DictReader(open(CH))}
D = (4, 8, 9, 10)
f = lambda d, k: float(rows[d][k])
J = lambda d, fn: json.load(open(os.path.join(pack(d), fn)))
Jx = lambda d, fn: J(d, fn) if os.path.exists(os.path.join(pack(d), fn)) else None
TEAL, RUST, INK = "#1F8A8C", "#C0532B", "#22252A"
AG_DIR = {4: pack(4), 8: F8, 9: P9, 10: P10}
AGJ = {d: json.load(open(os.path.join(AG_DIR[d], "agreement.json"))) for d in D}
SFJ = {d: json.load(open(os.path.join(AG_DIR[d], "size_fit.json"))) for d in D}
YARD = {4: "colleague-mode LiDAR replay", 8: "Force3DoF LiDAR replay (for information)", 9: "Force3DoF LiDAR replay", 10: "Force3DoF LiDAR replay"}
LM = {d: json.load(open(os.path.join(AG_DIR[d] if d == 8 else pack(d), "lidar.tum.meta.json"))) for d in D}
LT = {d: np.loadtxt(os.path.join(AG_DIR[d] if d == 8 else pack(d), "lidar.tum"), comments="#") for d in D}
LF = {d: (json.load(open(os.path.join(RUNS, "s2_static_08", "lidar_ref_f3dof", "lidar_map_facts.json"))) if d == 8 else J(d, "lidar_map_facts.json")) for d in D}
floor_gap = lambda a: float(math.hypot(a[-1, 1] - a[0, 1], a[-1, 2] - a[0, 2]))
REF9 = json.load(open(os.path.join(P9, "compare_drives_4_8_9.json")))


def corrected(d):
    return np.loadtxt(os.path.join(pack(d), "camera_corrected.tum"), comments="#")


def dense(a, step=0.05):
    xy, t = a[:, 1:3], a[:, 0]
    out, tt = [xy[0]], [t[0]]
    for i in range(len(xy) - 1):
        p, q = xy[i], xy[i + 1]
        n = max(1, int(np.ceil(np.hypot(*(q - p)) / step)))
        for k in range(1, n + 1):
            out.append(p + (q - p) * k / n); tt.append(t[i] + (t[i + 1] - t[i]) * k / n)
    return np.array(out), np.array(tt)


def icp2d(B, A, iters=60, gate=1.0, R=None, t=None):
    T = cKDTree(A)
    R = np.eye(2) if R is None else R
    t = np.zeros(2) if t is None else t
    for _ in range(iters):
        Bc = B @ R.T + t
        dd, ii = T.query(Bc)
        m = dd < gate
        P, Q = Bc[m], A[ii[m]]
        cp, cq = P.mean(0), Q.mean(0)
        U, S, Vt = np.linalg.svd((P - cp).T @ (Q - cq))
        Ri = Vt.T @ U.T
        if np.linalg.det(Ri) < 0:
            Vt[-1] *= -1; Ri = Vt.T @ U.T
        R, t = Ri @ R, Ri @ (t - cp) + cq
    return R, t


def seglen(P, mask):
    s = np.hypot(*np.diff(P, axis=0).T)
    return float(np.sum(s[mask[1:] & mask[:-1]]))


route = {d: dense(corrected(d)) for d in D}
SH = {"definition": ("camera map corrected path (camera_corrected.tum; drive 9: graph re-optimised by optimize_graph_se2.py; drive 10:"
                     " Admin.opt_poses), filled in every 5 cm; the later drive lined up on the earlier one by 2D rotation + shift (ICP,"
                     " pairs < 1.0 m, best of 36 starting headings; drive 10 on drive 9: headings within 30 deg of the one implied by both drives' alignment on drive 4); a point is on the shared route if within %.1f m of the other drive's"
                     " route; direction of travel not tested" % R_SHARED), "R_m": R_SHARED, "pairs": {}}
aligned, onshared = {}, {}
for base, d in ((4, 8), (4, 9), (4, 10), (9, 10)):
    A, _ = route[base]
    B, tb = route[d]
    best = None
    heads = np.arange(0, 360, 10)
    if (base, d) == (9, 10):
        # The floor's corridors are nearly symmetric about the start junction, so drive 10 on drive 9 fits almost as well turned by
        # same way (so the true rotation must be near 0, within the by-eye parking heading), and both line up on drive 4 (100 % and 93 % of drive 4 covered), so the heading is taken from that chain
        # (drive 9 -> drive 4 -> drive 10) and only +-30 deg around it is searched.
        r49 = SH["pairs"]["4_vs_9"]["rotation_deg"]; r410 = SH["pairs"]["4_vs_10"]["rotation_deg"]
        chain = r410 - r49
        SH["pairs_note_9_vs_10"] = ("heading searched only within +-30 deg of %.2f deg = rotation(4<-10) %.2f - rotation(4<-9) %.2f; an unconstrained"
                                    " search locks on about -90 deg (the corridors' near-symmetry). Both maps start at the robot's start pose on the same mark facing the same way, so the true rotation must be near 0 (within the by-eye parking heading); -90 deg would mean the robot started facing sideways" % (chain, r410, r49))
        heads = np.arange(chain - 30, chain + 30.01, 5)
    for h in np.radians(heads):
        R0 = np.array([[math.cos(h), -math.sin(h)], [math.sin(h), math.cos(h)]])
        R, t = icp2d(B, A, iters=40, R=R0, t=A[0] - R0 @ B[0])
        dd, _ = cKDTree(A).query(B @ R.T + t)
        dA, _ = cKDTree(B @ R.T + t).query(A)
        sc = (float(np.mean(dA < R_SHARED)), -float(np.median(dd[dd < R_SHARED])) if (dd < R_SHARED).any() else -9)
        if best is None or sc > best[0]:
            best = (sc, R, t)
    _, R, t = best
    Bc = B @ R.T + t
    aligned[(base, d)] = Bc
    dB, _ = cKDTree(A).query(Bc)
    dA, _ = cKDTree(Bc).query(A)
    kd, kb = "drive%d" % d, "drive%d" % base
    SH["pairs"]["%d_vs_%d" % (base, d)] = {
        "rotation_deg": round(math.degrees(math.atan2(R[1, 0], R[0, 0])), 2), "shift_m": [round(float(x), 3) for x in t],
        "start_offset_after_alignment_m": round(float(np.hypot(*(Bc[0] - A[0]))), 3),
        "%s_route_covered_pct" % kb: round(100 * float(np.mean(dA <= R_SHARED)), 1),
        "route_m": {kb: round(seglen(A, np.ones(len(A), bool)), 1), kd: round(seglen(Bc, np.ones(len(Bc), bool)), 1)},
        "shared_m": {kb: round(seglen(A, dA <= R_SHARED), 1), kd: round(seglen(Bc, dB <= R_SHARED), 1)},
        "sensitivity_by_R": {str(r): {"%s_m" % kd: round(seglen(Bc, dB <= r), 1), "%s_m" % kb: round(seglen(A, dA <= r), 1)} for r in (0.3, 0.5, 0.75, 1.0)},
        "distance_to_%s_route_on_shared_m" % kb: {"median": round(float(np.median(dB[dB <= R_SHARED])), 3),
                                                 "p95": round(float(np.percentile(dB[dB <= R_SHARED], 95)), 3)}}
    onshared[(base, d, d)] = (tb, dB <= R_SHARED)            # drive d's points on the part shared with base
    onshared[(base, d, base)] = (route[base][1], dA <= R_SHARED)   # base's points on the part shared with d


def on_route(key, times):
    tt, m = onshared[key]
    i = np.clip(np.searchsorted(tt, times), 0, len(tt) - 1)
    j = np.clip(i - 1, 0, len(tt) - 1)
    k = np.where(np.abs(tt[j] - times) < np.abs(tt[i] - times), j, i)
    return m[k]


def shared_agreement(d, key, which):
    """same pairing and SE(3) fit as compare_lidar.py (control: the whole-drive figures must reproduce agreement.json)"""
    Pd = AG_DIR[d]
    ag = AGJ[d]
    ref = TM.load_tum(os.path.join(Pd, "lidar_reference_dense.tum"), "lidar")
    est = TM.load_tum(os.path.join(Pd, "camera_corrected.tum" if which == "camera_corrected" else "camera.tum"), which)
    ir, ie = TM.associate(ref, est, max_diff=ag["association_tolerance_s"], offset=ag["time_offset_applied_s"])
    src, dst = est.xyz[ie], ref.xyz[ir]
    R, t, c = TM.umeyama(src, dst, with_scale=False)
    em = np.linalg.norm((R @ src.T).T + t - dst, axis=1)
    m = on_route(key, est.stamps[ie])
    whole = (round(float(np.median(em)), 3), round(float(np.percentile(em, 95)), 3))
    a = ag["estimates"][which]
    R2, t2, _ = TM.umeyama(src[m], dst[m], with_scale=False)
    e2 = np.linalg.norm((R2 @ src[m].T).T + t2 - dst[m], axis=1)
    return {"n_pairs_shared": int(m.sum()), "n_pairs_all": int(len(m)),
            "refit_on_shared_median_m": round(float(np.median(e2)), 3), "refit_on_shared_p95_m": round(float(np.percentile(e2, 95)), 3),
            "median_m": round(float(np.median(em[m])), 3), "p95_m": round(float(np.percentile(em[m], 95)), 3),
            "off_route_median_m": round(float(np.median(em[~m])), 3) if (~m).any() else None,
            "off_route_p95_m": round(float(np.percentile(em[~m], 95)), 3) if (~m).any() else None,
            "whole_median_m": whole[0], "whole_p95_m": whole[1],
            "control_whole_reproduces_agreement_json": bool(abs(whole[0] - a["disagreement_median_m"]) <= 0.001 and abs(whole[1] - a["disagreement_p95_m"]) <= 0.001)}


SA = {}
for base, d in ((4, 8), (4, 9), (4, 10), (9, 10)):
    for which in ("camera_corrected", "camera_tracking"):
        SA["%d_on_%dvs%d_%s" % (d, base, d, which)] = shared_agreement(d, (base, d, d), which)
        SA["%d_on_%dvs%d_%s" % (base, base, d, which)] = shared_agreement(base, (base, d, base), which)
SH["agreement_on_shared_route"] = SA
# control against the earlier comparison (same method, same inputs for drives 4 and 9)
_r = REF9["shared_route"]["agreement_on_shared_route"]
SH["control_vs_compare_drives_4_8_9"] = {k: bool(abs(SA["9_on_4vs9_%s" % k]["median_m"] - _r["9_%s" % k]["median_m"]) <= 0.001 and
                                                 abs(SA["4_on_4vs9_%s" % k]["median_m"] - _r["4vs9_%s" % k]["median_m"]) <= 0.001)
                                          for k in ("camera_corrected", "camera_tracking")}
print("control vs compare_drives_4_8_9.json:", SH["control_vs_compare_drives_4_8_9"])
print("controls vs agreement.json:", all(v["control_whole_reproduces_agreement_json"] for v in SA.values()))

fig, axs = plt.subplots(2, 2, figsize=(16, 15))
for ax, (base, d) in zip(axs.ravel(), ((4, 10), (9, 10), (4, 9), (4, 8))):
    A = route[base][0]; Bc = aligned[(base, d)]
    ax.plot(A[:, 0], A[:, 1], "--", color=TEAL, lw=1.3, label="drive %d, camera map corrected" % base)
    ax.plot(Bc[:, 0], Bc[:, 1], "--", color=RUST, lw=1.0, label="drive %d, camera map corrected (lined up on drive %d)" % (d, base))
    _, msk = onshared[(base, d, d)]
    ax.plot(np.where(msk, Bc[:, 0], np.nan), np.where(msk, Bc[:, 1], np.nan), "-", color=RUST, lw=5, alpha=0.25,
            label="drive %d on the shared route (within %.1f m of drive %d's)" % (d, R_SHARED, base))
    ax.plot(np.where(~msk, Bc[:, 0], np.nan), np.where(~msk, Bc[:, 1], np.nan), "-", color=INK, lw=1.6, label="drive %d NOT on the shared route" % d)
    ax.plot(*A[0], "o", color=INK, ms=7, label="start (drive %d)" % base)
    p = SH["pairs"]["%d_vs_%d" % (base, d)]
    ax.set_title("drive %d vs drive %d: %.1f of %.1f m of drive %d's route shared\n(drive %d: %.1f of %.1f m covered); lined up by %+.1f deg, shift %.2f m" % (
        base, d, p["shared_m"]["drive%d" % d], p["route_m"]["drive%d" % d], d, base, p["shared_m"]["drive%d" % base], p["route_m"]["drive%d" % base],
        p["rotation_deg"], math.hypot(*p["shift_m"])), fontsize=10)
    ax.set_aspect("equal"); ax.grid(alpha=0.3); ax.set_xlabel("x (m), drive %d's camera map" % base); ax.set_ylabel("y (m)")
    ax.legend(fontsize=7.5, loc="best")
fig.suptitle("Shared route: every path is a camera map after its loop closures (dashed; drive 9's graph re-optimised - database not closed). "
             "Not ground truth; direction of travel not tested.", fontsize=10)
fig.tight_layout(rect=(0, 0, 1, 0.96))
fig.savefig(os.path.join(P10, "shared_route.png"), dpi=110)
json.dump(SH, open(os.path.join(P10, "shared_route.json"), "w"), indent=1)

# ---------------- spreads expected for ONE drive (same sources as compare_drives_4_8_9.py) ----------------
sd = lambda vals: float(np.std(vals, ddof=1))
valid_lidar = [d for d in (3, 4, 5) if rows[d]["lidar_estimate_passes_own_check"] == "True"]
S_GAP = 0.2
S_GAP_SRC = ("where the robot parked: s2_scale_01 found stops at one mark up to 0.19 m apart even with a wheel stop "
             "(its RESULTS.md); drives 4, 8, 9 and 10 ended on the start mark by eye (tape-mark parking is only +-0.2-0.5 m)")
S_HEAD, S_HEAD_SRC = 3.0, "parking heading by eye, working figure (not measured)"
S_MED = sd([f(d, "agree_camera_map_corrected_median_m") for d in valid_lidar])
S_P95 = sd([f(d, "agree_camera_map_corrected_p95_m") for d in valid_lidar])
S_AG_SRC = ("drive-to-drive spread (sample standard deviation) of the camera map's whole-drive agreement on drives %s, whose LiDAR"
            " estimate passed its own check; different routes, so it may overstate the spread for a repeat" % ", ".join(map(str, valid_lidar)))
S_SCALE = sd([f(d, "best_fit_scale_camera_corrected_not_applied") for d in valid_lidar])
POIS = "counting spread, the square root of each count (Poisson, the least spread a count of chance events has)"
dur = {d: J(d, "facts.json")["duration_s"] for d in D}
lost10 = {d: f(d, "camera_tracking_lost_s") / J(d, "facts.json")["duration_s"] * 600 for d in (3, 4, 5, 6, 8, 9, 10) if Jx(d, "facts.json")}
S_LOST10 = sd([lost10[d] for d in (3, 4, 5, 6) if d in lost10])
S_LOST10_SRC = "drive-to-drive spread (sample standard deviation) of lost seconds per 10 min, drives %s" % ", ".join(str(d) for d in (3, 4, 5, 6) if d in lost10)
TURN = {d: J(d, "numbers.json")["turns"] for d in (4, 5, 6, 8, 9, 10)}
S_TURN_MED = sd([TURN[d]["abs_camera_minus_robot_deg"]["median"] for d in (4, 5, 6)])
S_TURN_BL = sd([TURN[d]["abs_blend_minus_robot_deg"]["median"] for d in (4, 5, 6)])
S_TURN_SRC = "drive-to-drive spread (sample standard deviation) of the per-turn median, drives 4, 5, 6"
PW = {d: Jx(d, "power_temperature.json") for d in (4, 6, 8, 9, 10)}
PW5 = Jx(5, "power_temperature_camera_alive.json")
pw_ref = [PW[4], PW[6]] + ([PW5] if PW5 else [])
RT = {d: J(d, "numbers.json")["processor_rtabmap"]["rtabmap_ms_per_node"] for d in (4, 5, 6, 8, 9, 10)}
S_RT = sd([RT[d]["median"] for d in (4, 5, 6)])
S_RT_SRC = "drive-to-drive spread (sample standard deviation) of drives 4, 5, 6 (numbers.json processor_rtabmap)"

T = []
PAIRS = ((10, 4), (10, 9), (9, 4), (8, 4))   # (drive, against)


def verdict(a, b, sa, sb, fmt, unit):
    comb = math.hypot(sa, sb); diff = abs(b - a); fa = fmt.replace("+", "")
    if comb <= 0:
        return ("same" if diff == 0 else "differs (no spread defined)"), None
    r = diff / comb
    if diff <= comb:
        return "**agrees** within %s%s (difference %s%s)" % (fa % comb, unit, fa % diff, unit), r
    return "**differs** by %.2f times (difference %s%s, combined %s%s)" % (r, fa % diff, unit, fa % comb, unit), r


def judge(label, v, s, src, fmt="%.2f", unit=" m", note="", scope="whole drive", group="camera and blend", only=None):
    row = {"row": label, "scope": scope, "spread": "%s%s" % (fmt.replace("+", "") % s[10], unit), "spread_source": src, "group": group, "note": note}
    for d in D:
        row["drive%d" % d] = "-" if v.get(d) is None else (fmt % v[d]) + unit
    for a, b in PAIRS:
        k = "%d_vs_%d" % (a, b)
        if (only and k not in only) or v.get(a) is None or v.get(b) is None:
            row["verdict_" + k], row["ratio_" + k] = "-", None
        else:
            row["verdict_" + k], row["ratio_" + k] = verdict(v[b], v[a], s[b], s[a], fmt, unit)
    T.append(row)


def context(label, vals, why, scope="whole drive", group="context"):
    row = {"row": label, "scope": scope, "spread": "-", "spread_source": "-", "group": group, "note": "not judged: " + why}
    for d, x in zip(D, vals):
        row["drive%d" % d] = x
    for a, b in PAIRS:
        row["verdict_%d_vs_%d" % (a, b)], row["ratio_%d_vs_%d" % (a, b)] = "-", None
    T.append(row)


same = lambda x: {d: x for d in D}
PR = SH["pairs"]
context("route: shared length with drive 4 (drive's own / drive 4's, within %.1f m)" % R_SHARED,
        ["-"] + ["%.1f of %.1f m / %.1f of %.1f m" % (PR["4_vs_%d" % d]["shared_m"]["drive%d" % d], PR["4_vs_%d" % d]["route_m"]["drive%d" % d],
                                                        PR["4_vs_%d" % d]["shared_m"]["drive4"], PR["4_vs_%d" % d]["route_m"]["drive4"]) for d in (8, 9, 10)],
        "defines the comparison (shared_route.json)", scope="route")
context("route: drive 10 shared with drive 9 (drive 10's / drive 9's, within %.1f m)" % R_SHARED,
        ["-", "-", "-", "%.1f of %.1f m / %.1f of %.1f m" % (PR["9_vs_10"]["shared_m"]["drive10"], PR["9_vs_10"]["route_m"]["drive10"],
                                                              PR["9_vs_10"]["shared_m"]["drive9"], PR["9_vs_10"]["route_m"]["drive9"])],
        "defines the drive 10 vs drive 9 comparison", scope="route")
context("distance driven - camera map corrected / robot's own wheels + gyroscope / LiDAR estimate", ["%.1f m / %.1f m / %.1f m" % (
    J(d, "map_corrected_facts.json")["path_m_nodes"], f(d, "distance_robot_wheels_m"), LF[d]["path_m_nodes"]) for d in D],
    "a description of the route; the wheels read about 6 % long (tape test)")
context("drive duration", ["%.0f s" % dur[d] for d in D], "description")
context("LiDAR yardstick used", [YARD[d] for d in D], "which LiDAR estimate each drive is compared with")
context("camera tracker setting / start", ["GEN_2, camera freshly started", "GEN_2", "GEN_2, fresh Jetson boot", "GEN_2, fresh Jetson boot"], "setting")
for k, lab in (("camera_tracking_alone_gap_m", "start-to-end gap, camera tracking alone (0 closures)"),
               ("map_nodes_blend_tracking_alone_gap_m", "start-to-end gap, map nodes tracking alone (= blend)"),
               ("robot_wheels_gyro_tracking_alone_gap_m", "start-to-end gap, robot's own wheels + gyroscope, tracking alone")):
    judge(lab, {d: f(d, k) for d in D}, same(S_GAP), S_GAP_SRC, group="robot's wheels + gyroscope" if "robot" in k else "camera and blend")
judge("heading gap, robot's own wheels + gyroscope", {d: f(d, "robot_wheels_gyro_heading_gap_deg") for d in D}, same(S_HEAD), S_HEAD_SRC,
      fmt="%+.1f", unit=" deg", group="robot's wheels + gyroscope")
judge("start-to-end gap, **camera map corrected** (floor)", {d: f(d, "camera_map_corrected_gap_floor_m") for d in D}, same(S_GAP), S_GAP_SRC,
      note="- closures beside it (rule 20): %s; drive 9's corrected path is the graph RE-OPTIMISED (database not closed)" % "; ".join(
          "drive %d %s (%s join the start)" % (d, rows[d]["camera_map_loop_closures"], rows[d]["camera_map_closures_joining_start"]) for d in D))
cl10 = {d: f(d, "camera_map_loop_closures") / J(d, "map_corrected_facts.json")["path_m_nodes"] * 10 for d in D}
judge("camera map loop closures per 10 m of its route", cl10,
      {d: math.sqrt(f(d, "camera_map_loop_closures")) / J(d, "map_corrected_facts.json")["path_m_nodes"] * 10 for d in D}, POIS + ", scaled",
      fmt="%.1f", unit="", note="- counts: %s" % " / ".join(rows[d]["camera_map_loop_closures"] for d in D))
lgap = {d: floor_gap(LT[d]) for d in D}
judge("start-to-end gap, **LiDAR estimate corrected** (floor)", lgap, same(S_GAP), S_GAP_SRC, group="LiDAR map",
      note="- LiDAR closures %s" % " / ".join(str(LM[d]["loop_closures"]) for d in D))
lcl10 = {d: LM[d]["loop_closures"] / LF[d]["path_m_nodes"] * 10 for d in D}
judge("LiDAR estimate loop closures per 10 m", lcl10, {d: math.sqrt(LM[d]["loop_closures"]) / LM[d]["loop_closures"] * lcl10[d] for d in D},
      POIS + ", scaled", fmt="%.1f", unit="", group="LiDAR map", note="- drive 4's LiDAR estimate is the colleague-mode replay, drives 8-10 the Force3DoF replay")
yn = lambda d: "yes" if AGJ[d]["reference_valid"] else "NO"
context("LiDAR yardstick passes its own check", [yn(d) for d in D], "a pass/fail check", group="LiDAR map")
fr = {d: float(str(rows[d]["camera_freezes_over_1p5s"]).split()[0]) for d in D}
fr10 = {d: fr[d] / dur[d] * 600 for d in D}
judge("camera freezes (tracker silent > 1.5 s), per 10 min", fr10, {d: math.sqrt(max(fr[d], 1)) / dur[d] * 600 for d in D},
      POIS + " (at least 1), scaled", fmt="%.1f", unit="", note="- counts: %s; drive 8's crash outage and drive 9's hang outage are not counted as freezes" % " / ".join("%d" % fr[d] for d in D))
rs10 = {d: f(d, "camera_tracking_restarts") / dur[d] * 600 for d in D}
judge("camera tracking restarts from the blend, per 10 min", rs10, {d: math.sqrt(max(f(d, "camera_tracking_restarts"), 1)) / dur[d] * 600 for d in D},
      POIS + " (at least 1), scaled", fmt="%.1f", unit="", note="- counts: %s" % " / ".join("%d" % f(d, "camera_tracking_restarts") for d in D))
judge("camera tracking lost, seconds per 10 min", {d: lost10[d] for d in D}, same(S_LOST10), S_LOST10_SRC, fmt="%.1f", unit=" s",
      note="- counts only time the tracker SAID it was lost")
context("camera program outages", [rows[d]["camera_program_crashes"] for d in D], "one event is not a rate")
judge("turns: abs(camera - robot), median of all turns", {d: TURN[d]["abs_camera_minus_robot_deg"]["median"] for d in D}, same(S_TURN_MED),
      S_TURN_SRC, fmt="%.1f", unit=" deg", note="- turns: %s" % " / ".join("%d" % TURN[d]["n"] for d in D))
judge("turns: abs(blend - robot), median of all turns", {d: TURN[d]["abs_blend_minus_robot_deg"]["median"] for d in D}, same(S_TURN_BL),
      S_TURN_SRC, fmt="%.1f", unit=" deg")
AGG = "agreement with the LiDAR estimate"
for which, lab in (("camera_corrected", "camera map corrected"), ("camera_tracking", "map nodes (= blend) tracking alone")):
    # against drive 4: each later drive over its part shared with drive 4; drive 4's own figure over its part shared with drive 10
    v = {4: SA["4_on_4vs10_%s" % which]["median_m"], 8: SA["8_on_4vs8_%s" % which]["median_m"], 9: SA["9_on_4vs9_%s" % which]["median_m"],
         10: SA["10_on_4vs10_%s" % which]["median_m"]}
    judge("**%s, ON THE ROUTE SHARED WITH DRIVE 4**, %s: median (SE(3))" % (AGG, lab), v, same(S_MED), S_AG_SRC, scope="shared with drive 4", group=AGG,
          only=("10_vs_4",), note="- drive 4's figure is over its part shared with drive 10 (with drive 9: %.2f m, drive 8: %.2f m); pairs %d / %d / %d / %d" % (
              SA["4_on_4vs9_%s" % which]["median_m"], SA["4_on_4vs8_%s" % which]["median_m"], SA["4_on_4vs10_%s" % which]["n_pairs_shared"],
              SA["8_on_4vs8_%s" % which]["n_pairs_shared"], SA["9_on_4vs9_%s" % which]["n_pairs_shared"], SA["10_on_4vs10_%s" % which]["n_pairs_shared"]))
    v = {4: SA["4_on_4vs10_%s" % which]["p95_m"], 8: SA["8_on_4vs8_%s" % which]["p95_m"], 9: SA["9_on_4vs9_%s" % which]["p95_m"], 10: SA["10_on_4vs10_%s" % which]["p95_m"]}
    judge("**%s, ON THE ROUTE SHARED WITH DRIVE 4**, %s: 95th percentile (SE(3))" % (AGG, lab), v, same(S_P95),
          S_AG_SRC.replace("agreement", "agreement (95th percentile)"), scope="shared with drive 4", group=AGG, only=("10_vs_4",))
    for q, S in (("median", S_MED), ("p95", S_P95)):
        v = {9: SA["9_on_9vs10_%s" % which]["%s_m" % q], 10: SA["10_on_9vs10_%s" % which]["%s_m" % q]}
        judge("**%s, ON THE ROUTE SHARED BY DRIVES 9 AND 10**, %s: %s (SE(3))" % (AGG, lab, "median" if q == "median" else "95th percentile"), v, same(S),
              S_AG_SRC if q == "median" else S_AG_SRC.replace("agreement", "agreement (95th percentile)"), scope="shared by 9 and 10", group=AGG, only=("10_vs_9",),
              note="- pairs %d / %d" % (SA["9_on_9vs10_%s" % which]["n_pairs_shared"], SA["10_on_9vs10_%s" % which]["n_pairs_shared"]) if q == "median" else "")
for q, S in (("median", S_MED), ("p95", S_P95)):
    judge("%s, camera map corrected, whole drive: %s (SE(3))" % (AGG, "median" if q == "median" else "95th percentile"),
          {d: AGJ[d]["estimates"]["camera_corrected"]["disagreement_%s_m" % q] for d in D}, same(S), S_AG_SRC if q == "median" else S_AG_SRC.replace("agreement", "agreement (95th percentile)"), group=AGG,
          note="- whole-drive lengths %s m" % " / ".join("%.0f" % J(d, "map_corrected_facts.json")["path_m_nodes"] for d in D))
judge("best-fit size factor, camera map corrected vs LiDAR estimate (not applied)", {d: AGJ[d]["estimates"]["camera_corrected"]["sim3_scale_vs_reference"] for d in D},
      same(S_SCALE), "drive-to-drive spread (sample standard deviation) of drives %s (valid LiDAR estimates)" % ", ".join(map(str, valid_lidar)), fmt="%.3f", unit="", group=AGG)
judge("for information, size fitted (Sim(3)): camera map corrected, whole drive, median", {d: SFJ[d]["estimates"]["camera_corrected"]["sim3"]["median_m"] for d in D},
      same(S_MED), S_AG_SRC + " (borrowed)", group="for information")
judge("for information, size fitted (Sim(3)): camera map corrected, whole drive, 95th percentile", {d: SFJ[d]["estimates"]["camera_corrected"]["sim3"]["p95_m"] for d in D},
      same(S_P95), S_AG_SRC.replace("agreement", "agreement (95th percentile)") + " (borrowed)", group="for information")
judge("RTAB-Map time per map snapshot, median", {d: RT[d]["median"] for d in D}, same(S_RT), S_RT_SRC, fmt="%.0f", unit=" ms", group="processor")
for lab, get, fmt in (("total power, median", lambda p: p["power_total_W"]["median"], "%.1f W"),
                      ("hottest point on the chip (tj), peak", lambda p: p["tj_C_max"]["value"], "%.1f C"),
                      ("processor load, mean of 12 cores, median", lambda p: p["cpu_mean_of_12_cores_pct"]["median"], "%.1f %%")):
    context(lab, [fmt % get(PW[d]) for d in D], "the drive-to-drive spread of drives 4, 5 (camera-alive part) and 6 (%s) comes from 3 drives and"
            " is too small to trust as a yardstick" % (fmt % sd([get(p) for p in pw_ref])), group="processor")

GROUPS = ["camera and blend", "robot's wheels + gyroscope", "LiDAR map", AGG, "processor", "for information"]
COUNTED = ("camera and blend", AGG, "processor")


def tally(ts, key):
    return (sum(t[key] <= 1 for t in ts), sum(1 < t[key] <= 3 for t in ts), sum(t[key] > 3 for t in ts))


def summary_for(pair, name):
    key = "ratio_" + pair
    J_ = [t for t in T if t[key] is not None and t["group"] in COUNTED]
    ok, mid, big = tally(J_, key)
    rw = tally([t for t in T if t[key] is not None and t["group"] == "robot's wheels + gyroscope"], key)
    li = tally([t for t in T if t[key] is not None and t["group"] == "LiDAR map"], key)
    over = sorted([t for t in J_ if t[key] > 3], key=lambda t: -t[key])
    return ("**Verdict, %s (computed): of %d judged rows about the camera, the blend, the agreement with the LiDAR estimate and the mapping program,"
            " %d agree within the combined uncertainty, %d differ by 1-3 times it, and %d differ by more than 3 times it%s.** Counted separately: the"
            " robot's own wheels + gyroscope (%d / %d / %d agree / 1-3 times / over 3 times) and the LiDAR estimate itself (%d / %d / %d)." % (
                name, len(J_), ok, mid, big, (": " + "; ".join("%s (%.1f times)" % (t["row"].replace("**", ""), t[key]) for t in over)) if big else "",
                *rw, *li)), {"judged": len(J_), "agree": ok, "1to3": mid, "over3": big}


sum10_4, c10_4 = summary_for("10_vs_4", "drive 10 against drive 4")
sum10_9, c10_9 = summary_for("10_vs_9", "drive 10 against drive 9")
row = lambda name: [t for t in T if t["row"].startswith(name)][0]
rm4 = row("**%s, ON THE ROUTE SHARED WITH DRIVE 4**, camera map corrected: median" % AGG)["ratio_10_vs_4"]
rp4 = row("**%s, ON THE ROUTE SHARED WITH DRIVE 4**, camera map corrected: 95th" % AGG)["ratio_10_vs_4"]
rm9 = row("**%s, ON THE ROUTE SHARED BY DRIVES 9 AND 10**, camera map corrected: median" % AGG)["ratio_10_vs_9"]
rp9 = row("**%s, ON THE ROUTE SHARED BY DRIVES 9 AND 10**, camera map corrected: 95th" % AGG)["ratio_10_vs_9"]
wm = row("%s, camera map corrected, whole drive: median" % AGG); wp = row("%s, camera map corrected, whole drive: 95th" % AGG)
grade = lambda a, b: "yes" if max(a, b) <= 1 else ("partly" if max(a, b) <= 3 else "no")
g4, g9 = grade(rm4, rp4), grade(rm9, rp9)
s10, s9 = SA["10_on_4vs10_camera_corrected"], SA["4_on_4vs10_camera_corrected"]
t10, t9 = SA["10_on_9vs10_camera_corrected"], SA["9_on_9vs10_camera_corrected"]
sf = {d: SFJ[d]["estimates"]["camera_corrected"]["sim3"] for d in D}
ag = {d: AGJ[d]["estimates"]["camera_corrected"] for d in D}
plain = ("**In plain words.** Drive 10's camera map came home to %.2f m with %s loop closures (%s of them joining the start, two in the last"
         " 15 s of driving, 02:30:02 and 02:30:14), and its LiDAR estimate to %.2f m with %d closures, although the robot's own wheels + gyroscope that the LiDAR estimate is"
         " built on ended %.2f m / %+.0f deg off. **Against the LiDAR estimate, drive 10's camera map sits a median %.2f m away (95th percentile %.2f m) over"
         " its whole %.0f m route - more than drive 9 (%.2f / %.2f m over %.0f m) and drive 4 (%.2f / %.2f m over %.0f m): %s against drive 9, %s against"
         " drive 4.** [INFERENCE] Most of that is size, not shape: its best-fit size factor is %.3f (drive 9 %.3f, drive 4 %.3f; the tape test predicts about"
         " 1.07 because the robot's wheels, which give the LiDAR estimate its size, read about 6 %% long; the size difference is measured, blaming it on the wheels is inferred), and with one size change allowed (for"
         " information only - the camera's size is real) drive 10 reads %.2f / %.2f m against drive 9's %.2f / %.2f m and drive 4's %.2f / %.2f m."
         " On the route shared with drive 4 (%.0f m of drive 10's route), drive 10 reads %.2f / %.2f m against drive 4's %.2f / %.2f m on the same part"
         " - %s. On the route shared with drive 9 (%.0f m), drive 10 reads %.2f / %.2f m against drive 9's %.2f / %.2f m - %s." % (
             f(10, "camera_map_corrected_gap_floor_m"), rows[10]["camera_map_loop_closures"], rows[10]["camera_map_closures_joining_start"],
             lgap[10], LM[10]["loop_closures"], f(10, "robot_wheels_gyro_tracking_alone_gap_m"), f(10, "robot_wheels_gyro_heading_gap_deg"),
             ag[10]["disagreement_median_m"], ag[10]["disagreement_p95_m"], J(10, "map_corrected_facts.json")["path_m_nodes"],
             ag[9]["disagreement_median_m"], ag[9]["disagreement_p95_m"], J(9, "map_corrected_facts.json")["path_m_nodes"],
             ag[4]["disagreement_median_m"], ag[4]["disagreement_p95_m"], J(4, "map_corrected_facts.json")["path_m_nodes"],
             "median %s, 95th percentile %s" % (wm["verdict_10_vs_9"].split(" (")[0].replace("**", ""), wp["verdict_10_vs_9"].split(" (")[0].replace("**", "")),
             "median %s, 95th percentile %s" % (wm["verdict_10_vs_4"].split(" (")[0].replace("**", ""), wp["verdict_10_vs_4"].split(" (")[0].replace("**", "")),
             ag[10]["sim3_scale_vs_reference"], ag[9]["sim3_scale_vs_reference"], ag[4]["sim3_scale_vs_reference"],
             sf[10]["median_m"], sf[10]["p95_m"], sf[9]["median_m"], sf[9]["p95_m"], sf[4]["median_m"], sf[4]["p95_m"],
             PR["4_vs_10"]["shared_m"]["drive10"], s10["median_m"], s10["p95_m"], s9["median_m"], s9["p95_m"],
             "the same within the stated spread" if g4 == "yes" else "median %.1f times, 95th percentile %.1f times the combined spread" % (rm4, rp4),
             PR["9_vs_10"]["shared_m"]["drive10"], t10["median_m"], t10["p95_m"], t9["median_m"], t9["p95_m"],
             "the same within the stated spread" if g9 == "yes" else "median %.1f times, 95th percentile %.1f times the combined spread" % (rm9, rp9)))
md = [plain, "", sum10_4, "", sum10_9, "",
      "*How to read this: two drives never give identical numbers. For each row the spread one drive is expected to have is stated, with where it"
      " comes from (column \"from\"); the two are combined as sqrt(s_a^2 + s_b^2). \"Agrees within X\" = the difference is smaller than that;"
      " \"differs by N times\" = N times larger (3 or more is a real difference). N = 1 drive each; the distance spreads are working figures."
      " \"Shared\" rows use only the part both drives covered; \"whole drive\" rows are per 10 min or per 10 m where the number grows with length."
      " Drive 8's column is for information (its yardstick: its own Force3DoF LiDAR replay). The drive 9 vs drive 4 column repeats"
      " s2_static_09/compare_drives_4_8_9.md with the same method (a check that nothing moved).*", "",
      "**Shared route - how it was defined.** %s. Drive 10 lined up on drive 4 by %+.1f deg and %.2f m (start then %.2f m from drive 4's): **%.1f of its"
      " %.1f m within %.1f m of drive 4's route; drive 4: %.1f of %.1f m (%.0f %%) within %.1f m of drive 10's.** Drive 10 lined up on drive 9 by %+.1f deg"
      " and %.2f m (start then %.2f m from drive 9's): **%.1f of its %.1f m within %.1f m of drive 9's route; drive 9: %.1f of %.1f m (%.0f %%).**"
      " Sensitivity (drive 10's length shared with drive 9 at R = 0.3 / 0.5 / 0.75 / 1.0 m): %s. Figure: `shared_route.png`. Control: the whole-drive"
      " figures recomputed here reproduce every agreement.json (%s), and drive 9's shared-route figures against drive 4 reproduce"
      " compare_drives_4_8_9.json (%s)." % (
          SH["definition"], PR["4_vs_10"]["rotation_deg"], math.hypot(*PR["4_vs_10"]["shift_m"]), PR["4_vs_10"]["start_offset_after_alignment_m"],
          PR["4_vs_10"]["shared_m"]["drive10"], PR["4_vs_10"]["route_m"]["drive10"], R_SHARED, PR["4_vs_10"]["shared_m"]["drive4"],
          PR["4_vs_10"]["route_m"]["drive4"], PR["4_vs_10"]["drive4_route_covered_pct"], R_SHARED,
          PR["9_vs_10"]["rotation_deg"], math.hypot(*PR["9_vs_10"]["shift_m"]), PR["9_vs_10"]["start_offset_after_alignment_m"],
          PR["9_vs_10"]["shared_m"]["drive10"], PR["9_vs_10"]["route_m"]["drive10"], R_SHARED, PR["9_vs_10"]["shared_m"]["drive9"],
          PR["9_vs_10"]["route_m"]["drive9"], PR["9_vs_10"]["drive9_route_covered_pct"],
          " / ".join("%.1f m" % v["drive10_m"] for v in PR["9_vs_10"]["sensitivity_by_R"].values()),
          "passed" if all(v["control_whole_reproduces_agreement_json"] for v in SA.values()) else "FAILED",
          "passed" if all(SH["control_vs_compare_drives_4_8_9"].values()) else "FAILED"), "",
      "Drive 10 off the route shared with drive 4: camera map corrected vs LiDAR estimate median %.2f m, 95th percentile %.2f m; off the route shared"
      " with drive 9: %.2f / %.2f m." % (s10["off_route_median_m"], s10["off_route_p95_m"], t10["off_route_median_m"] or float("nan"), t10["off_route_p95_m"] or float("nan")), "",
      "| row | group | scope | drive 4 | drive 8 (info) | drive 9 | drive 10 | spread per drive | from | drive 10 vs drive 4 | drive 10 vs drive 9 | drive 9 vs drive 4 | note |",
      "|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
srcs = []
for t in T:
    if t["spread_source"] != "-" and t["spread_source"] not in srcs:
        srcs.append(t["spread_source"])
for t in T:
    md.append("| %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s |" % (
        t["row"], t["group"], t["scope"], t["drive4"], t["drive8"], t["drive9"], t["drive10"], t["spread"],
        chr(65 + srcs.index(t["spread_source"])) if t["spread_source"] in srcs else "-", t["verdict_10_vs_4"], t["verdict_10_vs_9"],
        t["verdict_9_vs_4"].replace("**", ""), t["note"].lstrip("- ")))
md += ["", "Spread sources (column \"from\"):"] + ["- **%s** = %s" % (chr(65 + i), s) for i, s in enumerate(srcs)]
md += ["", "Caveats: (1) the 0.20 m for tracking-alone gaps is only the parking floor; tracking drift varies far more from drive to drive and"
       " grows with route length, so \"differs\" on those rows between a 85 m and a 250 m drive is expected and weak. (2) The agreement spreads come from"
       " whole drives on different routes (drives 3-5); the shared-route rows borrow them. (3) The shared-route agreement uses the whole-drive SE(3)"
       " alignment. (4) Drive 4's yardstick is the colleague-mode LiDAR replay, drives 9 and 10 the Force3DoF replay: one parameter apart"
       " (FINDINGS.md section 11). (5) Drive 9's corrected camera path is the re-optimised graph (its database was never closed); drive 10's is"
       " RTAB-Map's own saved result. (6) Counts use the Poisson floor, the least spread a count can have. (7) The size factor is the"
       " LiDAR estimate's size as much as the camera's: the LiDAR estimate takes its size from the robot's wheels, which the tape test found about"
       " 6 % long; the size-fitted rows are for information, never the result (ENGINEERING_NOTES.md section 4 rule 4)."]
json.dump({"plain_verdict": plain, "summary_10_vs_4": sum10_4, "summary_10_vs_9": sum10_9, "counts_10_vs_4": c10_4, "counts_10_vs_9": c10_9,
           "grade_shared_with_4": g4, "grade_shared_with_9": g9, "shared_route": SH, "rows": T,
           "spreads": {"gap_m": S_GAP, "heading_deg": S_HEAD, "agreement_median_m": round(S_MED, 3), "agreement_p95_m": round(S_P95, 3),
                       "lost_s_per_10min": round(S_LOST10, 1), "scale": round(S_SCALE, 4), "turn_camera_median_deg": round(S_TURN_MED, 2),
                       "turn_blend_median_deg": round(S_TURN_BL, 2), "rtabmap_ms": round(S_RT, 1), "valid_lidar_drives": valid_lidar}},
          open(os.path.join(P10, "compare_drives_4_8_9_10.json"), "w"), indent=1)
open(os.path.join(P10, "compare_drives_4_8_9_10.md"), "w").write("\n".join(md) + "\n")
print("\n".join(md))
