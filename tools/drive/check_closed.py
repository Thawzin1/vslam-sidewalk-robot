#!/usr/bin/env python3
"""check_closed.py - was this RTAB-Map database shut down properly?

WHY (drive 1 of series 2, 2026-09-24)
    Drive 1's mapping was never stopped - the Jetson lost its network, then its
    power, with the program still running. The database survived, but three
    things that are written ONLY on a proper shutdown were missing:

      Admin.opt_poses   the map's corrected positions (after loop closures)
      Word table        the visual dictionary - localization cannot work without it
      neighbour links   stored one way only; rtabmap-export then silently drops
                        the closure node and reports a wrong trajectory

    Every one of those looked fine from the outside: the file was the right
    size and opened without error. So this reads them directly.

    *Plain terms: a map that was not closed properly still opens, but it has
    lost its corrected shape and its memory of what places look like. This
    says which, before anyone powers the Jetson off.*

    Read-only (immutable=1). Safe to run on a database another program just
    closed. Do NOT run it while the mapping is still running - it would read
    a half-written file and report nonsense.

    A LiDAR-only map (the robot's) has no visual dictionary by design, so
    --lidar skips that one check.

  usage:
    check_closed.py <database.db> [--lidar]   exit 0 = closed properly, 1 = not
"""
from __future__ import print_function

import sqlite3
import struct
import sys


def main():
    argv = [a for a in sys.argv[1:] if a != "--lidar"]
    lidar = "--lidar" in sys.argv[1:]
    if len(argv) != 1:
        sys.exit(__doc__)
    db = argv[0]
    con = sqlite3.connect("file:%s?immutable=1" % db, uri=True)
    q = lambda sql: con.execute(sql).fetchone()

    nodes = q("SELECT COUNT(*) FROM Node")[0]
    opt = q("SELECT length(opt_poses), opt_ids FROM Admin")
    opt_len = (opt[0] or 0) if opt else 0
    # How many nodes the saved graph holds (compressData2 trailer: rows, cols).
    # A robot that never moved has ONE graph node and therefore no links at all
    # - the other nodes it made while standing still are not in the graph.
    graph_nodes = 0
    if opt and opt[1] is not None:
        rows, cols, _ = struct.unpack("<iii", bytes(opt[1])[-12:])
        graph_nodes = rows * cols
    words = q("SELECT COUNT(*) FROM Word")[0]
    fwd, back = q("SELECT COALESCE(SUM(from_id < to_id), 0), COALESCE(SUM(from_id > to_id), 0) "
                  "FROM Link WHERE type = 0")
    closures = q("SELECT COUNT(*) FROM Link WHERE type IN (1, 2, 3, 4) AND from_id > to_id")[0]

    checks = [
        ("corrected positions saved (Admin.opt_poses)", opt_len > 0,
         "%d bytes" % opt_len),
        ("visual dictionary saved (Word table)", lidar or words > 0,
         "not expected in a LiDAR map" if lidar else "%d words" % words),
        ("neighbour links stored both ways",
         (fwd == back == 0 and max(nodes, graph_nodes) <= 1)
         or (fwd == back == 0 and graph_nodes == 1)
         or (fwd > 0 and back > 0 and abs(fwd - back) <= 1),
         "%d forward, %d back (%d nodes in the saved graph)" % (fwd, back, graph_nodes)),
    ]
    print("  %s: %d nodes, %d loop closures" % (db, nodes, closures))
    ok = True
    for name, passed, detail in checks:
        ok = ok and passed
        print("    %s  %-46s %s" % ("PASS" if passed else "FAIL", name, detail))
    if ok:
        print("  CLOSED PROPERLY - safe to power off.")
    else:
        print("  NOT CLOSED PROPERLY. Do not reboot or power off yet if the mapping")
        print("  might still be running; if it is not, the missing parts are lost and")
        print("  the corrected positions can be rebuilt with optimize_graph_se2.py.")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
