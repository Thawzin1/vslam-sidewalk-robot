#!/usr/bin/env python3
"""slam_closed_check.py - facts about a continued (multi-session) RTAB-Map database, read-only (Jetson).

  --before DB OUT.json      record the old map's facts (run on the fresh working copy, before RTAB-Map opens it)
  --after  DB BEFORE.json [OUT.json]   after the session closed: did the new session get written and closed properly?

WHY: check_closed.py (the drives' check) looks for Admin.opt_poses. A working copy of drive 10 ALREADY has
drive 10's opt_poses, so that check would pass even if the new session never closed. This one asks whether
the saved state is NEWER than the session start and contains the NEW session's nodes.
*Plain terms: proves the file on disk holds today's session, closed properly, and not just drive 10 again.*

Checks (--after), each PASS/FAIL:
  1 new nodes written          Node.id max > old max
  2 new map id                 at least one node with map_id > old max map_id
  3 closed properly            Admin.time_enter later than the session start (UTC in the DB) AND opt_ids holds
                               at least one new node id
  4 old map intact             every old node id still present (the old nodes are never deleted)
  5 dictionary + links         Word table not empty; neighbour links stored both ways (as check_closed.py)
Reported (not judged): whether the saved graph also holds OLD node ids (= the sessions were joined), links
between the sessions by type (1 global, 2 nearby), counts per map id, Info rows.
Exit 0 = all PASS.
"""
from __future__ import print_function
import json, sqlite3, struct, sys, zlib, datetime
import numpy as np


def dec(b):
    b = bytes(b)
    rows, cols, typ = struct.unpack("<iii", b[-12:])
    raw = zlib.decompress(b[:-12])
    dt = {4: np.int32, 5: np.float32, 6: np.float64}[typ % 8]
    return np.frombuffer(raw, dtype=dt)


def facts(db):
    c = sqlite3.connect("file:%s?mode=ro" % db, uri=True)
    q = lambda s, *p: c.execute(s, p).fetchone()
    f = {}
    f["nodes"], f["node_id_min"], f["node_id_max"] = q("SELECT COUNT(*), MIN(id), MAX(id) FROM Node")
    f["map_id_max"] = q("SELECT MAX(map_id) FROM Node")[0]
    f["nodes_per_map"] = {str(m): n for m, n in c.execute("SELECT map_id, COUNT(*) FROM Node GROUP BY map_id")}
    f["valid_nodes_per_map"] = {str(m): n for m, n in c.execute(
        "SELECT map_id, COUNT(*) FROM Node WHERE weight > -9 GROUP BY map_id")}
    f["info_rows"] = q("SELECT COUNT(*) FROM Info")[0]
    f["info_last_time_utc"] = q("SELECT MAX(time_enter) FROM Info")[0]
    f["admin_time_utc"] = q("SELECT time_enter FROM Admin")[0]
    ids_b = q("SELECT opt_ids FROM Admin")[0]
    ids = dec(ids_b) if ids_b is not None and len(bytes(ids_b)) > 12 else np.array([], np.int32)
    f["opt_ids_count"] = int(ids.size)
    f["opt_ids_max"] = int(ids.max()) if ids.size else None
    f["words"] = q("SELECT COUNT(*) FROM Word")[0]
    fwd, back = q("SELECT COALESCE(SUM(from_id < to_id),0), COALESCE(SUM(from_id > to_id),0) FROM Link WHERE type=0")
    f["neighbour_links_fwd_back"] = [fwd, back]
    f["_opt_ids"] = ids
    return c, f


def main():
    if len(sys.argv) not in (4, 5) or sys.argv[1] not in ("--before", "--after"):
        sys.exit(__doc__)
    mode, db, other = sys.argv[1:4]
    out_json = sys.argv[4] if len(sys.argv) == 5 else None
    c, f = facts(db)
    if mode == "--before":
        ids = f.pop("_opt_ids")
        f["old_ids_all"] = [r[0] for r in c.execute("SELECT id FROM Node ORDER BY id")]
        # the old map's corrected positions (x, y) - the live watcher uses them to flag a join to a far-away place
        pb = c.execute("SELECT opt_poses FROM Admin").fetchone()[0]
        P = dec(pb).reshape(-1, 3, 4) if pb is not None else np.zeros((0, 3, 4))
        f["old_graph_xy"] = {str(int(i)): [round(float(p[0, 3]), 3), round(float(p[1, 3]), 3)] for i, p in zip(ids, P)}
        f["old_graph_yaw"] = {str(int(i)): round(float(np.arctan2(p[1, 0], p[0, 0])), 5) for i, p in zip(ids, P)}
        f["recorded_utc"] = datetime.datetime.utcnow().strftime("%Y-%m-%d %H:%M:%S")
        json.dump(f, open(other, "w"))
        print("old map: %d nodes (ids %d-%d), map ids up to %d, %d in the saved graph, Admin saved %s UTC"
              % (f["nodes"], f["node_id_min"], f["node_id_max"], f["map_id_max"], f["opt_ids_count"], f["admin_time_utc"]))
        return 0
    b = json.load(open(other))
    old_max = b["node_id_max"]
    ids = f.pop("_opt_ids")
    new_in_graph = int((ids > old_max).sum())
    old_in_graph = int((ids <= old_max).sum())
    present = set(r[0] for r in c.execute("SELECT id FROM Node WHERE id <= ?", (old_max,)))
    missing_old = len(set(b["old_ids_all"]) - present)
    inter = {}
    for t, n in c.execute("SELECT type, COUNT(*) FROM Link WHERE from_id > ? AND to_id <= ? AND from_id != to_id "
                          "GROUP BY type", (old_max, old_max)):
        inter[str(t)] = n
    fwd, back = f["neighbour_links_fwd_back"]
    checks = [
        ("new nodes written (Node.id max > %d)" % old_max, f["node_id_max"] > old_max,
         "max id %d, %d new rows" % (f["node_id_max"], f["nodes"] - b["nodes"])),
        ("new map id (> %d)" % b["map_id_max"], f["map_id_max"] > b["map_id_max"],
         "nodes per map id %s (valid: %s)" % (f["nodes_per_map"], f["valid_nodes_per_map"])),
        ("closed properly (Admin saved after the session start, with new nodes in the saved graph)",
         f["admin_time_utc"] > b["recorded_utc"] and new_in_graph > 0,
         "Admin %s UTC vs start %s UTC; saved graph %d nodes = %d new + %d old"
         % (f["admin_time_utc"], b["recorded_utc"], ids.size, new_in_graph, old_in_graph)),
        ("old map intact (all %d old node ids present)" % len(b["old_ids_all"]), missing_old == 0,
         "%d missing" % missing_old),
        ("dictionary + neighbour links both ways", f["words"] > 0 and abs(fwd - back) <= 1 and fwd > 0,
         "%d words; neighbour links %d/%d" % (f["words"], fwd, back)),
    ]
    ok = True
    for name, passed, detail in checks:
        ok = ok and passed
        print("  %s  %-80s %s" % ("PASS" if passed else "FAIL", name, detail))
    joined = old_in_graph > 0 and new_in_graph > 0
    print("  (reported) sessions joined in the saved graph: %s; links new->old by type %s "
          "(1 = recognised again, 2 = nearby re-match); Info rows %d -> %d"
          % ("YES" if joined else "NO - the new session floats separately", inter, b["info_rows"], f["info_rows"]))
    out = dict(f, joined=joined, new_in_graph=new_in_graph, old_in_graph=old_in_graph, links_new_to_old=inter,
               missing_old=missing_old, all_pass=ok)
    if out_json:
        json.dump(out, open(out_json, "w"), indent=1)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
