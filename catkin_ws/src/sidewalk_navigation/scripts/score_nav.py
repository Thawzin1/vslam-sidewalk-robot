#!/usr/bin/env python3
"""score_nav.py - score a navigation run against PASS_LINES.md and draw planned vs driven on drive 10's map.

Reads what nav_goals.py wrote (<run>/actual.csv, plans.jsonl, goals_result.json, robot_status.jsonl, and when
present truth.csv (sim) / fixes.csv (real) / odom.csv) and writes:
    <out>/nav_score.json   N1-N4 numbers, pass/fail, and the plain meaning of each
    <out>/nav_paths.png    the saved map; per leg: the FIRST route planned (dotted), the robot's own
                           estimate of where it drove (dashed), and - sim only - where it truly drove (solid)
Deviation = for every recorded position (10 Hz) during a leg, the distance to the nearest point of a route:
    "vs first plan"  the route drawn in the figure (the plan made when the goal was sent)
    "vs plan in force" the latest route the planner had published at that moment (it re-plans every 1 s)
Line styles follow the project convention: truth/reference solid, estimate dashed; planned dotted.

usage: score_nav.py --run-dir <nav out dir> --map maps/d10_nav.yaml --out <dir> [--title TEXT]
       [--lidar lidar_in_map.csv]   (real run: the LiDAR second opinion, already put in the map frame)
"""
import argparse
import csv
import json
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402

LEG_COLOURS = ["tab:blue", "tab:orange", "tab:green", "tab:red", "tab:purple", "tab:brown"]


def read_csv(p):
    if not os.path.exists(p):
        return None
    with open(p) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return None
    return {k: np.array([float(r[k]) if k != "state" and k != "kind" else 0 for r in rows])
            for k in rows[0].keys() if k not in ("state", "kind")}


def load_map(yaml_path):
    m = yaml.safe_load(open(yaml_path))
    with open(os.path.join(os.path.dirname(yaml_path), m["image"]), "rb") as f:
        f.readline()
        line = f.readline()
        while line.startswith(b"#"):
            line = f.readline()
        w, h = [int(t) for t in line.split()]
        f.readline()
        img = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w)
    res, ox, oy = float(m["resolution"]), float(m["origin"][0]), float(m["origin"][1])
    return img, [ox, ox + w * res, oy, oy + h * res]


def seg_dist(p, poly):
    """distance from points p (N,2) to polyline poly (M,2)"""
    if len(poly) == 1:
        return np.hypot(*(p - poly[0]).T)
    a, b = poly[:-1], poly[1:]
    ab = b - a
    L2 = np.maximum((ab ** 2).sum(1), 1e-12)
    out = np.full(len(p), np.inf)
    for i in range(0, len(p), 2000):
        q = p[i:i + 2000, None, :]
        t = np.clip(((q - a) * ab).sum(2) / L2, 0, 1)
        d = np.hypot(*(q - (a + t[..., None] * ab)).transpose(2, 0, 1))
        out[i:i + 2000] = d.min(1)
    return out


def pct(v, q):
    return float(np.percentile(v, q)) if len(v) else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-dir", required=True)
    ap.add_argument("--map", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--title", default="")
    ap.add_argument("--lidar", default="")
    ap.add_argument("--reach-m", type=float, default=0.50)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    R = a.run_dir
    res = json.load(open(os.path.join(R, "goals_result.json")))["results"]
    act = read_csv(os.path.join(R, "actual.csv"))
    tru = read_csv(os.path.join(R, "truth.csv"))
    plans = [json.loads(l) for l in open(os.path.join(R, "plans.jsonl")) if l.strip()]
    status = [json.loads(l) for l in open(os.path.join(R, "robot_status.jsonl")) if l.strip()] \
        if os.path.exists(os.path.join(R, "robot_status.jsonl")) else []
    fixes = read_csv(os.path.join(R, "fixes.csv"))

    score = {"run_dir": os.path.abspath(R), "legs": []}
    all_first, all_force, all_truth_first = [], [], []
    for r in res:
        k = r["k"]
        lp = [p for p in plans if p["goal"] == k and len(p["pts"]) > 1]
        leg = {"k": k, "name": r["name"], "result": r["result"], "time_s": r["time_s"],
               "end_estimate_to_goal_m": r.get("dist_estimate_to_goal_m"),
               "end_truth_to_goal_m": r.get("dist_truth_to_goal_m")}
        if act is not None and lp:
            m = act["goal"] == k
            P = np.c_[act["x"][m], act["y"][m]]
            T = act["t"][m]
            first = np.array(lp[0]["pts"])
            d_first = seg_dist(P, first)
            pt = np.array([p["t"] for p in lp])
            d_force = np.empty(len(P))
            for i, (pp, tt) in enumerate(zip(P, T)):
                j = max(0, np.searchsorted(pt, tt, side="right") - 1)
                d_force[i] = seg_dist(pp[None], np.array(lp[j]["pts"]))[0]
            all_first += list(d_first)
            all_force += list(d_force)
            leg.update(n_samples=int(len(P)), n_plans=len(lp),
                       planned_length_m=round(float(np.hypot(*np.diff(first, axis=0).T).sum()), 2),
                       driven_length_estimate_m=round(float(np.hypot(*np.diff(P, axis=0).T).sum()), 2)
                       if len(P) > 1 else 0.0,
                       dev_first_median_m=round(pct(d_first, 50), 3), dev_first_p95_m=round(pct(d_first, 95), 3),
                       dev_first_max_m=round(float(d_first.max()), 3),
                       dev_force_median_m=round(pct(d_force, 50), 3), dev_force_p95_m=round(pct(d_force, 95), 3))
            if tru is not None:
                mt = tru["goal"] == k
                Q = np.c_[tru["x"][mt], tru["y"][mt]]
                if len(Q):
                    d_tf = seg_dist(Q, first)
                    all_truth_first += list(d_tf)
                    leg.update(truth_dev_first_median_m=round(pct(d_tf, 50), 3),
                               truth_dev_first_p95_m=round(pct(d_tf, 95), 3))
        score["legs"].append(leg)

    # N2: every robot-side stop that was not "the planner went idle" (goal reached) while a leg was active
    stops = []
    prev = None
    for s in status:
        st = s["status"]
        if prev is not None and prev.get("moving") and not st.get("moving") and s["goal"] >= 0:
            why = st.get("why", "")
            if "planner idle" not in why:
                stops.append({"t": s["t"], "goal": s["goal"], "why": why})
        prev = st
    n_reached = sum(1 for r in res if r["result"] == "SUCCEEDED" and (r.get("dist_estimate_to_goal_m") or 9) <= a.reach_m)
    score["N1"] = {"pass": n_reached == len(res) and len(res) > 0, "reached": n_reached, "goals": len(res),
                   "line": "every goal: move_base SUCCEEDED within its time limit, robot estimate <= %.2f m from the goal" % a.reach_m}
    score["N2"] = {"pass_software_part": len(stops) == 0, "unplanned_stops": stops,
                   "line": "no stop by the safety layer except the planned end of each leg; collisions and "
                           "hand-overs are judged by the user on the spot (PROTOCOL.md)"}
    if all_first:
        score["N3"] = {"dev_first_median_m": round(pct(all_first, 50), 3), "dev_first_p95_m": round(pct(all_first, 95), 3),
                       "dev_force_median_m": round(pct(all_force, 50), 3), "dev_force_p95_m": round(pct(all_force, 95), 3),
                       "n": len(all_first)}
        if all_truth_first:
            score["N3"].update(truth_dev_first_median_m=round(pct(all_truth_first, 50), 3),
                               truth_dev_first_p95_m=round(pct(all_truth_first, 95), 3))
    # N4: the localisation's correction map -> odom = actual (map -> base_link) composed with inverse(odom -> base_link),
    # both sampled together at 10 Hz by nav_goals.py. A step in it = the estimate jumping at a fix.
    odo = read_csv(os.path.join(R, "odom.csv"))
    if act is not None and odo is not None:
        n = min(len(act["t"]), len(odo["t"]))
        ta, to = act["t"][:n], odo["t"][:n]
        ok = np.abs(ta - to) < 0.06
        ax_, ay_, ayaw = act["x"][:n][ok], act["y"][:n][ok], act["yaw"][:n][ok]
        ox_, oy_, oyaw = odo["x"][:n][ok], odo["y"][:n][ok], odo["yaw"][:n][ok]
        cyaw = ayaw - oyaw
        cx = ax_ - (np.cos(cyaw) * ox_ - np.sin(cyaw) * oy_)
        cy = ay_ - (np.sin(cyaw) * ox_ + np.cos(cyaw) * oy_)
        steps = np.hypot(np.diff(cx), np.diff(cy))
        dyaw = np.degrees(np.abs((np.diff(cyaw) + np.pi) % (2 * np.pi) - np.pi))
        driven = np.r_[0, np.cumsum(np.hypot(np.diff(ox_), np.diff(oy_)))]
        tt = ta[ok]
        n4 = {"largest_correction_step_m": round(float(steps.max()), 3) if len(steps) else None,
              "largest_correction_step_deg": round(float(dyaw.max()), 2) if len(dyaw) else None,
              "steps_over_0.3m": int((steps > 0.3).sum()), "driven_m_by_wheels_blend": round(float(driven[-1]), 2)}
        if len(tt) == 0:
            n4.update(n_fixes=None, note="no position samples matched the wheel log in time, so fixes cannot be placed by distance")
        elif fixes is not None and len(fixes["t"]):
            ft = np.sort(fixes["t"])
            dfix = np.interp(ft, tt, driven)
            gaps = np.diff(np.r_[0.0, dfix, driven[-1]])
            n4.update(n_fixes=int(len(ft)), longest_stretch_without_fix_m=round(float(gaps.max()), 2),
                      first_fix_after_m=round(float(dfix[0]), 2), fixes_per_10m=round(10 * len(ft) / max(driven[-1], 1e-6), 2))
        else:
            n4.update(n_fixes=0, note="no fixes.csv (sim: the stand-in corrects every 2 s; real run: /rtabmap/info)")
        score["N4"] = n4
    json.dump(score, open(os.path.join(a.out, "nav_score.json"), "w"), indent=1)

    # ---------------- figure ----------------
    img, ext = load_map(a.map)
    xs = [p for pl in plans for p in pl["pts"]] + ([[x, y] for x, y in zip(act["x"], act["y"])] if act is not None else [])
    xs = np.array(xs) if xs else np.array([[0, 0]])
    lo, hi = xs.min(0) - 2.5, xs.max(0) + 2.5
    asp = (hi[1] - lo[1]) / max(hi[0] - lo[0], 1e-6)
    h_map = min(max(11 * asp, 3.0), 11.0)
    fig, (ax, ax2) = plt.subplots(2, 1, figsize=(15, h_map + 5.2), gridspec_kw={"height_ratios": [h_map + 1.8, 3.4]})
    ax.imshow(img, cmap="gray", vmin=0, vmax=255, extent=ext, interpolation="nearest")
    for r in res:
        k = r["k"]
        c = LEG_COLOURS[k % len(LEG_COLOURS)]
        lp = [p for p in plans if p["goal"] == k and len(p["pts"]) > 1]
        if lp:
            f = np.array(lp[0]["pts"])
            ax.plot(f[:, 0], f[:, 1], ":", color=c, lw=2.2, label="leg %d %s: route planned at the start (dotted)" % (k + 1, r["name"]))
        if act is not None:
            m = act["goal"] == k
            ax.plot(act["x"][m], act["y"][m], "--", color=c, lw=1.6, label="leg %d: robot's own estimate of where it drove (dashed)" % (k + 1))
        if tru is not None:
            mt = tru["goal"] == k
            ax.plot(tru["x"][mt], tru["y"][mt], "-", color="k", lw=0.9, alpha=0.8,
                    label="simulator truth (solid)" if k == 0 else None)
        g = r["goal"]
        ax.plot(g[0], g[1], "*", ms=16, mfc=c, mec="k", zorder=6)
        ax.annotate("", (g[0] + 0.9 * math.cos(math.radians(g[2])), g[1] + 0.9 * math.sin(math.radians(g[2]))),
                    (g[0], g[1]), arrowprops=dict(arrowstyle="->", color=c, lw=2))
        ax.text(g[0] + 0.3, g[1] + 0.45, "%s\n%s %.0f s" % (r["name"], r["result"], r["time_s"]), fontsize=8,
                color=c, fontweight="bold", bbox=dict(fc="white", ec="none", alpha=0.8, pad=1))
    if a.lidar and os.path.exists(a.lidar):
        L = np.loadtxt(a.lidar, delimiter=",", skiprows=1)
        ax.plot(L[:, 1], L[:, 2], "-", color="tab:gray", lw=1.2, label="LiDAR estimate (second opinion, solid)")
    ax.plot(0, 0, "o", ms=9, mfc="k", mec="w", zorder=7)
    ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1]); ax.set_aspect("equal")
    ax.set_xlabel("x (m) - drive 10's map frame (start mark = 0, 0; east = +x)"); ax.set_ylabel("y (m)")
    ax.set_title((a.title + "\n" if a.title else "") + "planned (dotted) vs driven (dashed = robot's estimate%s) on drive 10's saved map"
                 % (", solid = simulator truth" if tru is not None else ""), fontsize=10)
    ax.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0), fontsize=7.5, ncol=1, borderaxespad=0)
    ax.grid(alpha=0.25)
    if act is not None and all_first:
        for r in res:
            k = r["k"]
            lp = [p for p in plans if p["goal"] == k and len(p["pts"]) > 1]
            m = act["goal"] == k
            if not lp or m.sum() < 2:
                continue
            P = np.c_[act["x"][m], act["y"][m]]
            s = np.r_[0, np.cumsum(np.hypot(*np.diff(P, axis=0).T))]
            if odo is not None:     # distance by the wheels + gyroscope blend: no localisation jumps in it
                mo = odo["goal"] == k
                O = np.c_[odo["x"][mo], odo["y"][mo]]
                so = np.r_[0, np.cumsum(np.hypot(*np.diff(O, axis=0).T))] if len(O) > 1 else None
                if so is not None:
                    s = np.interp(act["t"][m], odo["t"][mo], so)
            ax2.plot(s, 100 * seg_dist(P, np.array(lp[0]["pts"])), "--", color=LEG_COLOURS[k % 6], lw=1.3,
                     label="leg %d, estimate vs first plan" % (k + 1))
            if tru is not None:
                mt = tru["goal"] == k
                Q = np.c_[tru["x"][mt], tru["y"][mt]]
                if len(Q) > 1:
                    sq = np.r_[0, np.cumsum(np.hypot(*np.diff(Q, axis=0).T))]
                    ax2.plot(sq, 100 * seg_dist(Q, np.array(lp[0]["pts"])), "-", color=LEG_COLOURS[k % 6], lw=0.8,
                             alpha=0.7, label="leg %d, truth vs first plan" % (k + 1))
        ax2.set_xlabel("distance driven in the leg (m, by the wheels + gyroscope)"); ax2.set_ylabel("off the planned route (cm)")
        ax2.grid(alpha=0.3); ax2.legend(fontsize=7.5, ncol=2)
        n3 = score["N3"]
        ax2.set_title("how far off the planned route: median %.1f cm, 95th percentile %.1f cm (estimate vs first plan)"
                      % (100 * n3["dev_first_median_m"], 100 * n3["dev_first_p95_m"]), fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(a.out, "nav_paths.png"), dpi=120)
    print(json.dumps({k: v for k, v in score.items() if k != "legs"}, indent=1))
    for l in score["legs"]:
        print(l)


if __name__ == "__main__":
    main()
