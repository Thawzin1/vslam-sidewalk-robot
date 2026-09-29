#!/usr/bin/env python3
"""score_localise.py - score a localisation session against its registered pass lines (PASS_LINES.md).

Runs AFTER the session, on the Jetson (nice -n 19), from small files only:
  --rec      ~/.run_records/<run>/   starts.csv, localise_events.csv, localise_track.csv (the watcher's)
  --lidar    folder with THIS session's LiDAR products from replay_lidar.sh on the robot:
             lidar.tum (corrected LiDAR path, its own frame, origin = where the recording began
             = the start mark) and lidar_map.npz (its walls). Optional: without it only P1 and P3-live
             are scored and P2 is "not scored".
  --d4       drive 4's results pack (lidar.tum, lidar_map.npz, camera_corrected.tum)

HOW A LiDAR POSITION IS TURNED INTO "WHERE THE CAMERA MAP SHOULD HAVE PUT THE ROBOT" (two steps,
neither uses the camera localiser's own output, so the check is not circular)
  A. session LiDAR frame -> drive 4 LiDAR frame: the session's LiDAR walls are fitted onto drive 4's
     LiDAR walls (2D ICP, iterative closest point: repeatedly pair each wall cell with the nearest
     wall cell of the other map and solve the best rotation + shift; the worst 5 % of pairs are
     ignored; a coarse search over rotation and shift first, so it cannot stop part-way). Starts from "no shift" (both recordings began on the start mark). Its own check:
     the fit must stay within 0.30 m and 3 deg of "no shift" and its median pair distance must be
     <= 0.10 m, or P2 is declared NOT SCORABLE (not failed).
  B. drive 4 LiDAR frame -> drive 4 CAMERA map, LOCALLY: drive 4 recorded both paths at the same
     moments. Take the drive-4 LiDAR positions within 0.75 m of the point, look up where drive 4's
     camera map had the robot at those same moments, and apply the median of those offsets.
     *Plain terms: drive 4's camera map is not a perfect copy of the building (it is 6 % stretched
     and up to half a metre off the LiDAR map in places), so "the right place on the map" means
     "where the map drew this spot", read off drive 4 itself, not the building's true coordinates.*
  The uncertainty quoted beside each figure combines (square root of the sum of squares) step A's
  median pair distance, step B's offset spread (median absolute deviation x 1.4826) and 0.03 m for
  LiDAR range noise. A start PASSES P2 when its disagreement is <= 0.30 m (PASS_LINES.md); the heading difference is reported beside it.

  usage: score_localise.py --rec ~/.run_records/s2_loc_01 --lidar <folder> --d4 <drive 4 pack> [--out DIR]
         score_localise.py --selftest --d4 <drive 4 pack>
"""
import argparse
import csv
import json
import math
import os
import sys

import numpy as np
from scipy.spatial import cKDTree

P2_M = 0.30
P2_DEG = 5.0
ICP_MAX_T, ICP_MAX_DEG, ICP_MAX_RES = 0.30, 3.0, 0.10
# stability check - the same fit on 6 random halves of the walls and on the east and west halves of the map must
# stay within FIT2_MAX_DEV m of the median of those fits, with rotations within FIT2_MAX_ROT deg of each other.
FIT2_TRIM, FIT2_MAX_DEV, FIT2_MAX_ROT = 0.80, 0.10, 1.0
NEAR_M = 0.75
FALSE_M = 1.0
P1_S, P1_M = 120.0, 3.0     # first fix within 120 s and 3 m of driving after the start
P5_S = 1.0                  # median processing time per picture under the 1 Hz budget


def read_tum(p):
    d = np.loadtxt(p, comments="#")
    yaw = 2 * np.arctan2(d[:, 6], d[:, 7])
    return d[:, 0], d[:, 1], d[:, 2], yaw


def first_pose_frame(tum_path):
    """(x0, y0, yaw0) of a LiDAR path's first node. The LiDAR map's own frame is the robot's odometry
    frame (drive 4: first node at (-2.28, -4.04) facing -111.6 deg), so every LiDAR path AND its walls
    are moved so that the first node - the robot parked on the start mark - is (0, 0) facing +x,
    exactly as the drive packs do (results_packs_2026-09-26/analyze_run.py line 70)."""
    t, x, y, yaw = read_tum(tum_path)
    return x[0], y[0], yaw[0]


def to_start_frame(xy, f):
    x0, y0, a = f
    c, s = math.cos(-a), math.sin(-a)
    d = np.asarray(xy, float) - [x0, y0]
    return np.c_[c * d[:, 0] - s * d[:, 1], s * d[:, 0] + c * d[:, 1]]


def read_lidar(folder):
    """a LiDAR session's path and walls, both in its start-mark frame"""
    f = first_pose_frame(os.path.join(folder, "lidar.tum"))
    t, x, y, yaw = read_tum(os.path.join(folder, "lidar.tum"))
    xy = to_start_frame(np.c_[x, y], f)
    w = to_start_frame(walls(os.path.join(folder, "lidar_map.npz")), f)
    return t, xy[:, 0], xy[:, 1], yaw - f[2], w


def walls(npz):
    z = np.load(npz, allow_pickle=True)
    img, (x0, x1, y0, y1) = z["img"], z["extent"]
    r, c = np.nonzero(img < 0.5)             # 0.08 = wall (20999 cells on drive 4 = cells_solid), 1 = background
    h, w = img.shape
    x = x0 + (c + 0.5) * (x1 - x0) / w
    y = y1 - (r + 0.5) * (y1 - y0) / h          # row 0 = top (render_map.py draws origin='upper')
    return np.c_[x, y]


def icp2d(src, dst, iters=80, trim=0.95, search_deg=6.0, step_deg=0.5):
    """coarse search over the starting rotation (+-search_deg, every step_deg), a short ICP from each,
    then the best one refined: a single ICP from 'no turn' stopped 0.8 deg short on the test map,
    because a T of long corridors lets the fit slide along them"""
    tree = cKDTree(dst)
    best = None
    for a0 in np.arange(-search_deg, search_deg + 1e-9, step_deg):
        th = math.radians(a0)
        R0 = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
        R, t, res = _icp(src, tree, dst, R0, np.zeros(2), 25, trim)
        if best is None or res < best[2]:
            best = (R, t, res)
    # then a grid over the shift (+-0.8 m every 0.05 m) scored by the share of a 3000-cell sample that
    # lands within 0.05 m of a wall: point-to-point ICP creeps along corridors and stops short
    R = best[0]
    rng = np.random.default_rng(0)
    sub = src[rng.choice(len(src), min(3000, len(src)), replace=False)].dot(R.T)
    g = np.arange(-0.8, 0.8001, 0.05)
    sc, t = -1, best[1]
    for dx in g:
        for dy in g:
            tt = best[1] + np.array([dx, dy])
            d, _ = tree.query(sub + tt)
            f = np.mean(d < 0.05)
            if f > sc:
                sc, t = f, tt
    return _icp(src, tree, dst, R, t, iters, trim)


def fit_v2(src, dst, detail=None):
    """registered wall fit v2: icp2d at trim 0.80, and its stability over 8 subsets of the walls"""
    R, t, res = icp2d(src, dst, trim=FIT2_TRIM)
    rng = np.random.default_rng(7)
    mx = np.median(src[:, 0])
    subs = [src[rng.random(len(src)) < 0.5] for _ in range(6)] + [src[src[:, 0] < mx], src[src[:, 0] >= mx]]
    T, A = [], []
    for s in subs:
        Rs, ts, _ = icp2d(s, dst, trim=FIT2_TRIM)
        T.append(ts); A.append(math.degrees(math.atan2(Rs[1, 0], Rs[0, 0])))
    T = np.array(T)
    if detail is not None:     # per-subset fits, for the drift report (amendment 26 Sept: halves' disagreement)
        detail["subsets"] = [{"name": n, "shift_m": [round(float(a), 3), round(float(b), 3)], "rot_deg": round(r, 2)}
                             for n, (a, b), r in zip(["random%d" % i for i in range(6)] + ["west", "east"], T, A)]
        detail["east_vs_west_m"] = round(float(np.hypot(*(T[7] - T[6]))), 3)
        detail["east_vs_west_deg"] = round(A[7] - A[6], 2)
    dev = float(np.max(np.hypot(*(T - np.median(T, 0)).T)))
    rot_range = float(max(A) - min(A))
    return R, t, res, dev, rot_range


def _icp(src, tree, dst, R, t, iters, trim):
    for _ in range(iters):
        s = src.dot(R.T) + t
        d, i = tree.query(s)
        keep = d <= np.quantile(d, trim)
        a, b = s[keep], dst[i[keep]]
        ma, mb = a.mean(0), b.mean(0)
        U, _, Vt = np.linalg.svd((a - ma).T.dot(b - mb))
        dR = Vt.T.dot(U.T)
        if np.linalg.det(dR) < 0:
            Vt[-1] *= -1
            dR = Vt.T.dot(U.T)
        R, t = dR.dot(R), dR.dot(t - ma) + mb
    d, _ = tree.query(src.dot(R.T) + t)
    return R, t, float(np.median(d[d <= np.quantile(d, trim)]))


def interp_pose(tt, x, y, yaw, t, max_gap=5.0):
    i = np.searchsorted(tt, t)
    if i <= 0 or i >= len(tt):
        return None
    if tt[i] - tt[i - 1] > max_gap:
        # robot parked between two map nodes is fine only if they are the same place
        # (drive 4: parked 146 s on the start mark between two LiDAR nodes 0.057 m apart)
        if math.hypot(x[i] - x[i - 1], y[i] - y[i - 1]) > 0.10:
            return None
    f = (t - tt[i - 1]) / max(tt[i] - tt[i - 1], 1e-9)
    dy = math.atan2(math.sin(yaw[i] - yaw[i - 1]), math.cos(yaw[i] - yaw[i - 1]))
    return x[i - 1] + f * (x[i] - x[i - 1]), y[i - 1] + f * (y[i] - y[i - 1]), yaw[i - 1] + f * dy


class D4:
    def __init__(self, d):
        self.lt, self.lx, self.ly, self.lyaw, self.walls = read_lidar(d)
        # the camera map's own frame already starts at (0, 0) facing +x on the start mark (node 1)
        self.ct, self.cx, self.cy, self.cyaw = read_tum(os.path.join(d, "camera_corrected.tum"))

    def expected_camera(self, px, py, pyaw=None, max_dyaw_deg=45.0, exclude_t=None, exclude_s=10.0):
        """drive-4 LiDAR frame point -> where drive 4's camera map drew it (median local offset).
        With pyaw: only drive-4 passes facing within max_dyaw_deg of it (the localiser matches
        pictures taken facing the same way, so those passes are the part of the map it uses)."""
        d = np.hypot(self.lx - px, self.ly - py)
        ok = d <= NEAR_M
        if pyaw is not None:
            dy = np.abs(np.degrees(np.arctan2(np.sin(self.lyaw - pyaw), np.cos(self.lyaw - pyaw))))
            ok &= dy <= max_dyaw_deg
        if exclude_t is not None:     # self-test only: leave out the queried pass itself
            ok &= np.abs(self.lt - exclude_t) > exclude_s
        idx = np.nonzero(ok)[0]
        offs, hoffs = [], []
        for i in idx:
            c = interp_pose(self.ct, self.cx, self.cy, self.cyaw, self.lt[i], max_gap=3.0)
            if c is None:
                continue
            offs.append((c[0] - self.lx[i], c[1] - self.ly[i]))
            hoffs.append(math.atan2(math.sin(c[2] - self.lyaw[i]), math.cos(c[2] - self.lyaw[i])))
        if len(offs) < 3:
            return None
        o = np.array(offs)
        med = np.median(o, axis=0)
        mad = 1.4826 * np.median(np.hypot(*(o - med).T))
        return px + med[0], py + med[1], float(np.median(hoffs)), float(mad), len(offs)


def parked_time_after(track, start, t_from):
    """first time >= t_from when the robot's odometry moved < 0.02 m over 5 s (parked)"""
    rows = [r for r in track if int(r["start"]) == start and float(r["t_epoch"]) >= t_from]
    for j, r in enumerate(rows):
        t = float(r["t_epoch"])
        later = [q for q in rows[j:] if float(q["t_epoch"]) <= t + 5.0]
        if later and float(later[-1]["t_epoch"]) >= t + 4.0 and \
                float(later[-1]["odom_m_since_start"]) - float(r["odom_m_since_start"]) < 0.02:
            return later[-1]
    return None


def score(a):
    rec = os.path.expanduser(a.rec)
    starts = list(csv.DictReader(open(os.path.join(rec, "starts.csv"))))
    events = list(csv.DictReader(open(os.path.join(rec, "localise_events.csv"))))
    track = list(csv.DictReader(open(os.path.join(rec, "localise_track.csv"))))
    d4 = D4(a.d4)
    out = {"run": os.path.basename(rec.rstrip("/")), "starts": [], "icp": None}
    T = None
    if a.lidar:
        st, sx, sy, syaw, sw = read_lidar(a.lidar)
        if a.fit == "v1":      # as registered 26 Sept 03:59 (session 2's verdicts)
            R, t, res = icp2d(sw, d4.walls)
            dev = rot_range = None
        else:
            R, t, res, dev, rot_range = fit_v2(sw, d4.walls)
        th = math.degrees(math.atan2(R[1, 0], R[0, 0]))
        ok = abs(th) <= ICP_MAX_DEG and np.hypot(*t) <= ICP_MAX_T and res <= ICP_MAX_RES
        if a.fit == "v2":
            ok = ok and dev <= FIT2_MAX_DEV and rot_range <= FIT2_MAX_ROT
        out["icp"] = {"fit": a.fit, "shift_m": [round(float(t[0]), 3), round(float(t[1]), 3)], "rot_deg": round(th, 2),
                      "median_pair_m": round(res, 3), "within_own_check": bool(ok),
                      "subset_largest_deviation_m": None if dev is None else round(dev, 3),
                      "subset_rotation_range_deg": None if rot_range is None else round(rot_range, 2)}
        T = (R, t, res, ok)
    for s in starts:
        k = int(s["start"])
        ev = [e for e in events if int(e["start"]) == k]
        first = next((e for e in ev if e["first_fix"] == "1"), None)
        row = {"start": k, "mark": s["mark"], "fixed": first is not None}
        if first:
            row.update(first_kind=first["kind"], first_s=float(first["s_since_start"]),
                       first_odom_m=float(first["odom_m_since_start"]),
                       suspect_jumps=sum(int(e["suspect"]) for e in ev))
            park = parked_time_after(track, k, float(first["t_epoch"]) + 5.0)
            if park and park["map_x"]:
                row["eval_t"] = float(park["t_epoch"])
                row["camera_xy_yaw"] = [float(park["map_x"]), float(park["map_y"]), float(park["map_yaw_deg"])]
            if T and "eval_t" in row:
                R, t, res, ok = T
                p = interp_pose(st, sx, sy, syaw, row["eval_t"], max_gap=5.0)
                if p is None:
                    row["P2"] = "not scorable: no LiDAR pose at that moment"
                else:
                    q = R.dot([p[0], p[1]]) + t
                    qyaw = p[2] + math.atan2(R[1, 0], R[0, 0])
                    e = d4.expected_camera(q[0], q[1], qyaw)
                    if e is None:
                        row["P2"] = "not scorable: drive 4 did not pass within %.2f m of this spot" % NEAR_M
                    else:
                        ex, ey, eh, mad, n = e
                        cx, cy, cyaw = row["camera_xy_yaw"]
                        dis = math.hypot(cx - ex, cy - ey)
                        dh = math.degrees(math.atan2(math.sin(math.radians(cyaw) - qyaw - eh),
                                                     math.cos(math.radians(cyaw) - qyaw - eh)))
                        sig = math.sqrt(res ** 2 + mad ** 2 + 0.03 ** 2)
                        row.update(expected_xy=[round(ex, 3), round(ey, 3)], disagreement_m=round(dis, 3),
                                   heading_diff_deg=round(dh, 1), uncertainty_m=round(sig, 3),
                                   local_pairs=n, false_localisation=dis > FALSE_M)
                        if not ok:
                            row["P2"] = "not scorable: the LiDAR map fit failed its own check"
                        else:
                            # distance only (the roadmap's criterion); heading is REPORTED: at turn-around
                            # spots drive 4's own heading lookup is noisy (known-answer test: -5.0 deg there)
                            row["P2"] = "PASS" if dis <= P2_M else "FAIL"
        # P3 over EVERY fix of this start, not only the first: where the LiDAR says the robot was
        if T and ev:
            R, t, res, ok = T
            worst = None
            for e in ev:
                if not e["map_x"]:
                    continue
                p = interp_pose(st, sx, sy, syaw, float(e["t_epoch"]), max_gap=5.0)
                if p is None:
                    continue
                q = R.dot([p[0], p[1]]) + t
                x = d4.expected_camera(q[0], q[1], p[2] + math.atan2(R[1, 0], R[0, 0]))
                if x is None:
                    continue
                dd = math.hypot(float(e["map_x"]) - x[0], float(e["map_y"]) - x[1])
                worst = dd if worst is None else max(worst, dd)
            row["worst_fix_disagreement_m"] = None if worst is None else round(worst, 3)
            row["P3_false_fix"] = None if worst is None else worst > FALSE_M
        row["P1"] = "PASS" if (first and row["first_s"] <= P1_S and row["first_odom_m"] <= P1_M
                              and not row.get("P3_false_fix")) else "FAIL"
        out["starts"].append(row)
    n = len(out["starts"])
    out["P1_found_itself"] = "%d of %d starts (pass line: at least 4 of 5)" % (
        sum(r["P1"] == "PASS" for r in out["starts"]), n)
    sc = [r for r in out["starts"] if r.get("P2") in ("PASS", "FAIL")]
    out["P2_placed_right"] = "%d of %d scorable fixed starts within %.2f m (pass line: all but at most one; heading reported, not judged)" % (
        sum(r["P2"] == "PASS" for r in sc), len(sc), P2_M)
    fl = [r for r in out["starts"] if r.get("P3_false_fix") is not None]
    out["P3_false_fixes"] = ("%d starts with a fix over %.1f m from the LiDAR-derived place (pass line: 0); live suspect jumps %d"
                             % (sum(bool(r["P3_false_fix"]) for r in fl), FALSE_M,
                                sum(r.get("suspect_jumps", 0) for r in out["starts"]))) if T else "not scored (no LiDAR)"
    if a.log and os.path.exists(a.log):
        import re
        txt = open(a.log, errors="ignore").read()
        v = np.array([float(x) for x in re.findall(r"RTAB-Map=([0-9.]+)s", txt)])
        if len(v):
            out["P5_keeps_up"] = {"pictures": int(len(v)), "median_s": round(float(np.median(v)), 3),
                                  "p95_s": round(float(np.quantile(v, 0.95)), 3), "over_1s": int((v > 1.0).sum()),
                                  "pass": bool(np.median(v) < P5_S)}
    return out


def selftest(a):
    d4 = D4(a.d4)
    rng = np.random.default_rng(1)
    th, sh = math.radians(3.0), np.array([0.40, -0.20])
    R0 = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
    src = (d4.walls - sh).dot(R0)            # a map that is d4 moved by the inverse transform
    src = src[rng.random(len(src)) < 0.8] + rng.normal(0, 0.02, (0, 2)).sum() # and missing 20 % of its walls
    src = src + rng.normal(0, 0.02, src.shape)  # and 2 cm of wall noise
    R, t, res = icp2d(src, d4.walls)
    got = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    ok1 = abs(got - 3.0) < 0.3 and np.hypot(*(t - sh)) < 0.05
    print("ICP recovers a known 3.0 deg / (0.40, -0.20) m shift: got %.2f deg, (%.3f, %.3f) m, pairs %.3f m -> %s"
          % (got, t[0], t[1], res, "OK" if ok1 else "FAIL"))
    # plumbing: at drive-4 LiDAR moments, the expected camera position must reproduce drive 4's camera map
    errs = []
    for i in range(0, len(d4.lt), 10):
        e = d4.expected_camera(d4.lx[i], d4.ly[i], d4.lyaw[i], exclude_t=d4.lt[i])
        c = interp_pose(d4.ct, d4.cx, d4.cy, d4.cyaw, d4.lt[i], 3.0)
        if e and c:
            errs.append(math.hypot(e[0] - c[0], e[1] - c[1]))
    errs = np.array(errs)
    # CROSS-PASS RESOLUTION: the lookup uses only drive-4 moments more than 10 s away from the queried
    # one (other passes over the same spot), so it measures how consistently drive 4's camera map drew the
    # same spot on different passes. (Without the exclusion it was 0.032/0.090 m - that only checked the
    # plumbing, because the queried pass itself was in the lookup.)
    ok2 = len(errs) > 10 and np.quantile(errs, 0.95) < P2_M / 2
    print("local LiDAR->camera-map lookup vs drive 4's own camera map at the same moments (the method's own"
          " cross-pass resolution, samples within 10 s of the query excluded): median %.3f m, 95th pct %.3f m, max %.3f m, n=%d -> %s (needs 95th pct < %.2f m)"
          % (np.median(errs), np.quantile(errs, 0.95), errs.max(), len(errs), "OK" if ok2 else "FAIL", P2_M / 2))
    return ok1 and ok2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rec")
    ap.add_argument("--lidar", default="")
    ap.add_argument("--d4", required=True)
    ap.add_argument("--out", default="")
    ap.add_argument("--log", default="", help="the session's mapping.log (for P5)")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--fit", choices=("v1", "v2"), default="v2",
                    help="wall fit: v2 = registered 26 Sept for session 3 on (trim 0.80 + subset stability check); "
                         "v1 = the original (trim 0.95), which scored session 2")
    a = ap.parse_args()
    if a.selftest:
        sys.exit(0 if selftest(a) else 1)
    out = score(a)
    txt = json.dumps(out, indent=1)
    if a.out:
        os.makedirs(a.out, exist_ok=True)
        open(os.path.join(a.out, "score.json"), "w").write(txt)
    print(txt)


if __name__ == "__main__":
    main()
