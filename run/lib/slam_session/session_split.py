#!/usr/bin/env python3
"""session_split.py - split a continued (multi-session) RTAB-Map database into today's and drive 10's parts,
read-only, so the drive results-pack tools (which assume ONE session) can be used on it. Jetson or any computer.

  usage: session_split.py DB OLD_FACTS.json OUTDIR [--old-db MASTER.db]

  DB              the session's closed database (~/slam_series2/<run>.db)
  OLD_FACTS.json  ~/.run_records/<run>/old_map_facts.json (written by start_drive.sh SLAM=1 step 0: the old map's
                  highest node id, its node ids and its saved (corrected) positions BEFORE today)

WRITES (OUTDIR):
  camera.tum            today's nodes, TRACKING ALONE (Node.pose), ids > old max  - rule 20 kind 1
  camera_corrected.tum  today's nodes, CORRECTED (Admin.opt_poses)               - rule 20 kind 2 (empty if the
                        session never closed properly)
  old_after.tum         drive 10's nodes where TODAY's closed map puts them (Admin.opt_poses, ids <= old max)
  old_shift.csv         per drive-10 node: saved position before today vs after today, and the shift (m)
  split_facts.json      counts per map id; links by kind: within today (1 global / 2 nearby), today<->drive 10
                        (the joins), within drive 10; first and last join; both start-to-end gaps with closure
                        counts (rule 20); old-node shift median / 95th percentile / max; NEW AREA (S5): floor within
                        1.0 m of today's corrected path that is more than 1.5 m from every drive-10 node, in m2 at
                        0.10 m cells, and today's path length outside drive 10's corridor.
TUM stamps are the nodes' own (Node.stamp). Nothing here is ground truth (rule 19).
*Plain terms: separates "what today added" from "what drive 10 had", so each can be measured and drawn.*
"""
from __future__ import print_function
import json, os, sqlite3, struct, sys, zlib
import numpy as np


def dec(b):
    b = bytes(b)
    rows, cols, typ = struct.unpack("<iii", b[-12:])
    raw = zlib.decompress(b[:-12])
    return np.frombuffer(raw, dtype={4: np.int32, 5: np.float32, 6: np.float64}[typ % 8])


def quat(R):
    w = np.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = np.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2
    y = np.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2
    z = np.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2
    x = np.copysign(x, R[2, 1] - R[1, 2]); y = np.copysign(y, R[0, 2] - R[2, 0]); z = np.copysign(z, R[1, 0] - R[0, 1])
    return x, y, z, w


def write_tum(path, rows, header):
    with open(path, "w") as fh:
        fh.write("# " + header + "\n")
        for t, P in rows:
            qx, qy, qz, qw = quat(P[:, :3])
            fh.write("%.6f %.4f %.4f %.4f %.6f %.6f %.6f %.6f\n" % (t, P[0, 3], P[1, 3], P[2, 3], qx, qy, qz, qw))


def main():
    if len(sys.argv) < 4:
        sys.exit(__doc__)
    db, facts_f, out = sys.argv[1:4]
    os.makedirs(out, exist_ok=True)
    old = json.load(open(facts_f))
    old_max = int(old["node_id_max"])
    old_xy = {int(k): np.array(v) for k, v in old.get("old_graph_xy", {}).items()}
    c = sqlite3.connect("file:%s?mode=ro" % db, uri=True)
    stamp = dict(c.execute("SELECT id, stamp FROM Node"))
    mapid = dict(c.execute("SELECT id, map_id FROM Node"))
    weight = dict(c.execute("SELECT id, weight FROM Node"))
    f = {"db": db, "old_max": old_max,
         "nodes_per_map": {str(m): n for m, n in c.execute("SELECT map_id, COUNT(*) FROM Node GROUP BY map_id")}}
    # tracking alone, today's nodes in the graph (weight > -9: the rest were discarded standing still)
    raw = [(stamp[i], np.frombuffer(bytes(p), np.float32).reshape(3, 4)) for i, p in
           c.execute("SELECT id, pose FROM Node WHERE id > ? AND weight > -9 ORDER BY id", (old_max,))]
    write_tum(os.path.join(out, "camera.tum"), raw, "TRACKING ALONE (Node.pose), today's nodes id > %d" % old_max)
    f["today_nodes_valid"] = len(raw)
    ids_b, poses_b = c.execute("SELECT opt_ids, opt_poses FROM Admin").fetchone()
    corr = {}
    if ids_b is not None and poses_b is not None:
        ids = dec(ids_b); P = dec(poses_b).reshape(-1, 3, 4)
        corr = {int(i): p for i, p in zip(ids, P)}
    today = sorted(i for i in corr if i > old_max)
    write_tum(os.path.join(out, "camera_corrected.tum"), [(stamp[i], corr[i]) for i in today],
              "CORRECTED (Admin.opt_poses at today's close), today's nodes id > %d" % old_max)
    olds = sorted(i for i in corr if i <= old_max)
    write_tum(os.path.join(out, "old_after.tum"), [(stamp[i], corr[i]) for i in olds],
              "drive 10's nodes where today's closed map puts them")
    f["saved_graph"] = {"today": len(today), "drive10": len(olds), "joined": bool(today and olds)}
    # links, once per pair, by kind
    pairs = {}
    for a, b, t in c.execute("SELECT from_id, to_id, type FROM Link WHERE from_id != to_id AND type IN (1,2,3,4)"):
        pairs[(min(a, b), max(a, b))] = t
    kinds = {"within_today": {}, "today_to_drive10": {}, "within_drive10": {}}
    joins = []
    for (a, b), t in pairs.items():
        k = "within_drive10" if b <= old_max else ("today_to_drive10" if a <= old_max else "within_today")
        kinds[k][str(t)] = kinds[k].get(str(t), 0) + 1
        if k == "today_to_drive10":
            joins.append((stamp[b], b, a, t))
    joins.sort()
    f["closures_by_kind"] = kinds
    f["joins"] = {"count": len(joins),
                  "first": dict(zip(("stamp", "today_id", "drive10_id", "type"), joins[0])) if joins else None,
                  "last": dict(zip(("stamp", "today_id", "drive10_id", "type"), joins[-1])) if joins else None}
    if raw:
        f["session_first_stamp"] = raw[0][0]
        f["join_after_s"] = round(joins[0][0] - raw[0][0], 1) if joins else None
    # rule 20: both gaps, today's first vs last node, with closure counts beside them
    if raw:
        g = raw[-1][1][:2, 3] - raw[0][1][:2, 3]
        f["gap_tracking_alone_m"] = round(float(np.hypot(*g)), 3)
    if today:
        g = corr[today[-1]][:2, 3] - corr[today[0]][:2, 3]
        f["gap_corrected_m"] = round(float(np.hypot(*g)), 3)
        f["gap_closures"] = {"within_today": sum(kinds["within_today"].values()),
                             "today_to_drive10": sum(kinds["today_to_drive10"].values())}
    # how far today moved drive 10's nodes
    rows, sh = [], []
    for i in olds:
        if i in old_xy:
            d = float(np.hypot(*(corr[i][:2, 3] - old_xy[i])))
            sh.append(d); rows.append("%d,%.3f,%.3f,%.3f,%.3f,%.3f" % (i, old_xy[i][0], old_xy[i][1], corr[i][0, 3],
                                                                       corr[i][1, 3], d))
    with open(os.path.join(out, "old_shift.csv"), "w") as fh:
        fh.write("drive10_id,x_before,y_before,x_after,y_after,shift_m\n" + "\n".join(rows) + "\n")
    if sh:
        a = np.array(sh)
        f["drive10_shift_m"] = {"median": round(float(np.median(a)), 3), "p95": round(float(np.percentile(a, 95)), 3),
                                "max": round(float(a.max()), 3), "n": len(sh)}
    # S5: new floor (0.10 m cells): within 1.0 m of today's corrected path, > 1.5 m from every drive-10 node
    if today and olds:
        T = np.array([corr[i][:2, 3] for i in today]); O = np.array([corr[i][:2, 3] for i in olds])
        # densify today's path to 0.1 m steps
        pts = [T[0]]
        for p, q in zip(T[:-1], T[1:]):
            n = max(1, int(np.hypot(*(q - p)) / 0.1))
            pts.extend(p + (q - p) * k / n for k in range(1, n + 1))
        pts = np.array(pts)
        lo = pts.min(0) - 1.2; hi = pts.max(0) + 1.2
        gx = np.arange(lo[0], hi[0], 0.1); gy = np.arange(lo[1], hi[1], 0.1)
        X, Y = np.meshgrid(gx, gy); C = np.c_[X.ravel(), Y.ravel()]
        near_today = np.zeros(len(C), bool); far_old = np.ones(len(C), bool)
        for chunk in range(0, len(C), 20000):
            cc = C[chunk:chunk + 20000]
            dt = np.min(np.hypot(cc[:, None, 0] - pts[None, ::3, 0], cc[:, None, 1] - pts[None, ::3, 1]), axis=1)
            do = np.min(np.hypot(cc[:, None, 0] - O[None, :, 0], cc[:, None, 1] - O[None, :, 1]), axis=1)
            near_today[chunk:chunk + 20000] = dt <= 1.0
            far_old[chunk:chunk + 20000] = do > 1.5
        f["new_area_m2"] = round(float((near_today & far_old).sum()) * 0.01, 1)
        dpo = np.min(np.hypot(pts[:, None, 0] - O[None, :, 0], pts[:, None, 1] - O[None, :, 1]), axis=1)
        f["new_path_m_outside_drive10"] = round(float((dpo > 1.5).sum()) * 0.1, 1)
    json.dump(f, open(os.path.join(out, "split_facts.json"), "w"), indent=1)
    print(json.dumps({k: f.get(k) for k in ("nodes_per_map", "today_nodes_valid", "saved_graph", "joins",
                                            "gap_tracking_alone_m", "gap_corrected_m", "drive10_shift_m",
                                            "new_area_m2", "new_path_m_outside_drive10")}))


if __name__ == "__main__":
    main()
