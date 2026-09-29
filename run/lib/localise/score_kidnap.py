#!/usr/bin/env python3
"""score_kidnap.py - K1-K3 (PASS_LINES.md addenda 07:13, 07:40, 09:10, 09:59) plus P2/P3, for one session, with the
registered wall fit (v2 by default). Small files only; run nice -n 19 on the Jetson.

  A FIX = RTAB-Map ACCEPTED it: watcher events of kind 'uncertainty' (published uncertainty dropped below 9999),
  'recognised' or 'nearby'. 'pending' (held) and 'cache link' rows are NOT fixes.
  K1: first fix within 120 s and 3 m of driving, at >= 4 of 5 starts.
  K2 (amended 07:40): at every start with a fix except at most one, the position AT THE MOMENT OF THE FIRST FIX is
      within 0.30 m of the LiDAR-derived position at that same moment (P2's two-step method, wall fit v2).
  K3: zero fixes, at any moment, more than 1.0 m from the LiDAR-derived position.
  P2 (as registered 03:59): read at the first parked moment >= 5 s after the first RECOGNITION event (recognised/nearby).
  P3: zero recognition events more than 1.0 m from the LiDAR-derived position.
  Also written (for the figure): every accepted track position with its LiDAR-derived position.

  usage: score_kidnap.py --rec ~/.run_records/<run> --lidar <folder> --d4 <drive 4 pack> --out <dir> [--fit v1|v2]
"""
import argparse, csv, json, math, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import score_localise as S

ACCEPT = ("uncertainty", "recognised", "nearby")
RECOG = ("recognised", "nearby")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rec", required=True); ap.add_argument("--lidar", required=True)
    ap.add_argument("--d4", required=True); ap.add_argument("--out", required=True)
    ap.add_argument("--fit", choices=("v1", "v2"), default="v2")
    a = ap.parse_args()
    rec = os.path.expanduser(a.rec)
    starts = list(csv.DictReader(open(os.path.join(rec, "starts.csv"))))
    ev = list(csv.DictReader(open(os.path.join(rec, "localise_events.csv"))))
    track = list(csv.DictReader(open(os.path.join(rec, "localise_track.csv"))))
    d4 = S.D4(a.d4)
    st, sx, sy, syaw, sw = S.read_lidar(a.lidar)
    det = {}
    if a.fit == "v1":
        R, t, res = S.icp2d(sw, d4.walls); dev = rr = None
    else:
        det = {}
        R, t, res, dev, rr = S.fit_v2(sw, d4.walls, detail=det)
    th = math.atan2(R[1, 0], R[0, 0])
    ok = abs(math.degrees(th)) <= S.ICP_MAX_DEG and np.hypot(*t) <= S.ICP_MAX_T and res <= S.ICP_MAX_RES
    if a.fit == "v2":
        ok = ok and dev <= S.FIT2_MAX_DEV and rr <= S.FIT2_MAX_ROT
    out = {"run": os.path.basename(rec.rstrip("/")), "fit": {"kind": a.fit, "shift_m": [round(float(t[0]), 3), round(float(t[1]), 3)],
           "rot_deg": round(math.degrees(th), 2), "median_pair_m": round(res, 3), "subset_dev_m": dev and round(dev, 3),
           "subset_rot_range_deg": rr and round(rr, 2), "within_own_check": bool(ok), "drift_detail": det}, "starts": []}
    NS = "not scorable this session: the LiDAR reference failed its own check (wall fit v2 halves disagree)"

    def expected(T):
        """LiDAR-derived position on drive 4's camera map at time T: (x, y, heading_deg, spread_m) or None"""
        p = S.interp_pose(st, sx, sy, syaw, T, max_gap=5.0)
        if p is None:
            return None
        q = R.dot([p[0], p[1]]) + t
        e = d4.expected_camera(q[0], q[1], p[2] + th)
        if e is None:
            return None
        return e[0], e[1], math.degrees(p[2] + th + e[2]), e[3]

    fig_rows = []
    for s in starts:
        k = int(s["start"]); t0 = float(s["t_epoch"])
        e_all = [e for e in ev if int(e["start"]) == k and e["kind"] in ACCEPT and e["map_x"]]
        e_rec = [e for e in e_all if e["kind"] in RECOG]
        row = {"start": k, "mark": s["mark"], "accepted_fixes": len(e_all), "recognition_events": len(e_rec)}
        if e_all:
            f = e_all[0]
            row.update(first_fix_kind=f["kind"], first_fix_s=float(f["s_since_start"]), first_fix_odom_m=float(f["odom_m_since_start"]),
                       first_fix_xy=[float(f["map_x"]), float(f["map_y"])])
            row["K1"] = "PASS" if (row["first_fix_s"] <= 120 and row["first_fix_odom_m"] <= 3.0) else "FAIL"
            x = expected(float(f["t_epoch"]))
            if x is None:
                row["K2"] = "not scorable: no LiDAR-derived position at the fix moment"
            else:
                d = math.hypot(row["first_fix_xy"][0] - x[0], row["first_fix_xy"][1] - x[1])
                row.update(K2_disagreement_m=round(d, 3), K2_expected_xy=[round(x[0], 3), round(x[1], 3)],
                           K2_uncertainty_m=round(math.sqrt(res ** 2 + x[3] ** 2 + 0.03 ** 2), 3))
                row["K2"] = ("PASS" if d <= S.P2_M else "FAIL") if ok else NS
            ds = []
            for e in e_all:
                x = expected(float(e["t_epoch"]))
                if x is not None:
                    ds.append((math.hypot(float(e["map_x"]) - x[0], float(e["map_y"]) - x[1]), e["kind"], float(e["s_since_start"])))
            row["fixes_scored"] = len(ds)
            if ds:
                w = max(ds)
                row.update(K3_worst_m=round(w[0], 3), K3_worst_at=[w[1], w[2]], K3_median_m=round(float(np.median([d[0] for d in ds])), 3),
                           K3_over_1m=sum(d[0] > 1.0 for d in ds))
                rd = [d for d in ds if d[1] in RECOG]
                row["P3_worst_m"] = round(max(rd)[0], 3) if rd else None
        else:
            row["K1"] = "FAIL (no accepted fix)"
        if e_rec:
            park = S.parked_time_after(track, k, float(e_rec[0]["t_epoch"]) + 5.0)
            if park and park["map_x"]:
                x = expected(float(park["t_epoch"]))
                if x is None:
                    row["P2"] = "not scorable: no LiDAR pose at that moment"
                else:
                    d = math.hypot(float(park["map_x"]) - x[0], float(park["map_y"]) - x[1])
                    row.update(P2_disagreement_m=round(d, 3), P2_read_s=round(float(park["t_epoch"]) - t0, 1))
                    row["P2"] = ("PASS" if d <= S.P2_M else "FAIL") if ok else NS
        # figure: every track row after the first fix, and the LiDAR-derived position all start long
        for r in track:
            if int(r["start"]) != k:
                continue
            T = float(r["t_epoch"]); x = expected(T)
            fig_rows.append([k, round(T - t0, 2), int(r["localised"]), r["map_x"], r["map_y"],
                             "" if x is None else round(x[0], 3), "" if x is None else round(x[1], 3), r["accepted_now"]])
        out["starts"].append(row)
    n = len(out["starts"])
    fixed = [r for r in out["starts"] if r["accepted_fixes"]]
    k2 = [r for r in fixed if r.get("K2") in ("PASS", "FAIL")]
    out["K1"] = "%d of %d starts (pass line: at least 4 of 5)" % (sum(r["K1"] == "PASS" for r in out["starts"]), n)
    out["K2"] = "%d of %d fixed starts scorable, %d of them within 0.30 m (pass line: every fixed start except at most one)" % (
        len(k2), len(fixed), sum(r["K2"] == "PASS" for r in k2))
    out["K3"] = "%d fixes over 1.0 m out of %d scored (pass line: 0)" % (sum(r.get("K3_over_1m", 0) for r in fixed),
                                                                          sum(r.get("fixes_scored", 0) for r in fixed))
    if not ok:
        out["K2"] = out["K3"] = out["P2_P3"] = NS
    os.makedirs(a.out, exist_ok=True)
    json.dump(out, open(os.path.join(a.out, "score_kidnap.json"), "w"), indent=1)
    with open(os.path.join(a.out, "track_vs_lidar.csv"), "w") as f:
        w = csv.writer(f); w.writerow(["start", "s_since_start", "localised", "map_x", "map_y", "lidar_x", "lidar_y", "accepted_now"]); w.writerows(fig_rows)
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
