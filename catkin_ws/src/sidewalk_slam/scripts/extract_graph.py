#!/usr/bin/env python3
"""extract_graph.py - pull a run's pose graph out of its database as CSV.

The graph is the RESULT. It is under a megabyte; the database around it is
several GB of stored images that nothing downstream needs. Getting this out is
what actually protects a run.

TWO FAILURES THIS HANDLES, BOTH LEARNED THE HARD WAY

  1. A DAMAGED B-TREE REVISITS ROWS. A plain scan once produced 19,718 "poses"
     for a 1,269-node map. Reading ORDER BY id uses the index instead, and ids
     already seen are skipped.

  2. A DAMAGED DATABASE RETURNS THE WRONG TYPE. On two files the pose column
     came back as an INTEGER where a blob belongs. The first version of this
     script called len() on it, threw TypeError, and died PART WAY THROUGH -
     leaving a poses.csv of 701 rows for a 1,241-node map. A truncated file
     that looks exactly like a complete one is worse than no file, because it
     gets used.

     So: every row is now handled defensively, bad rows are COUNTED and
     REPORTED, and the run ends with an explicit statement of how many nodes
     were readable out of how many exist.

  usage:
    extract_graph.py <database.db> <output_dir>
"""
from __future__ import print_function

import math
import os
import struct
import sqlite3
import sys

if len(sys.argv) < 3:
    sys.exit("  usage: extract_graph.py <database.db> <output_dir>")

db, out = sys.argv[1], sys.argv[2]
if not os.path.isdir(out):
    os.makedirs(out)

con = sqlite3.connect("file:%s?immutable=1" % db, uri=True)

cols = [r[1] for r in con.execute("PRAGMA table_info(Node)")]
has_stamp = "stamp" in cols

try:
    total = con.execute("SELECT COUNT(*) FROM Node").fetchone()[0]
except sqlite3.Error:
    total = -1

sel = "SELECT id, %s, pose FROM Node ORDER BY id" % ("stamp" if has_stamp else "0")

n = 0
seen = set()
bad_type = 0
bad_short = 0
bad_unpack = 0
read_errors = 0

rows = []
try:
    for row in con.execute(sel):
        rows.append(row)
except sqlite3.Error as exc:
    # A damaged file can stop mid-iteration. Keep what was read and SAY SO.
    read_errors = 1
    print("  READ STOPPED EARLY: %s" % str(exc)[:70])

with open(os.path.join(out, "poses.csv"), "w") as fh:
    fh.write("id,stamp,x,y,z,yaw_deg\n")
    for nid, stamp, blob in rows:
        if nid in seen:
            continue
        seen.add(nid)
        if blob is None:
            bad_type += 1
            continue
        if not isinstance(blob, (bytes, bytearray, memoryview)):
            # A damaged row can hand back an int. Count it, do not crash.
            bad_type += 1
            continue
        b = bytes(blob)
        if len(b) < 48:
            bad_short += 1
            continue
        try:
            v = struct.unpack("<12f", b[:48])
        except struct.error:
            bad_unpack += 1
            continue
        yaw = math.degrees(math.atan2(v[4], v[0]))
        try:
            st = float(stamp or 0.0)
        except (TypeError, ValueError):
            st = 0.0
        fh.write("%d,%.6f,%.4f,%.4f,%.4f,%.2f\n" % (nid, st, v[3], v[7], v[11], yaw))
        n += 1

m = 0
try:
    with open(os.path.join(out, "links.csv"), "w") as fh:
        fh.write("from_id,to_id,type\n")
        for a, b_, t in con.execute("SELECT from_id, to_id, type FROM Link"):
            try:
                fh.write("%d,%d,%d\n" % (int(a), int(b_), int(t)))
                m += 1
            except (TypeError, ValueError):
                pass
except sqlite3.Error as exc:
    print("  LINK READ STOPPED EARLY: %s" % str(exc)[:70])
con.close()

print("  poses.csv: %d rows" % n)
print("  links.csv: %d rows (stored twice each - de-duplicate before counting)" % m)

skipped = bad_type + bad_short + bad_unpack
if skipped or read_errors:
    print()
    print("  *** THIS GRAPH IS INCOMPLETE ***")
    if bad_type:
        print("      %d rows had no usable pose (wrong type or NULL)" % bad_type)
    if bad_short:
        print("      %d rows had a pose too short to decode" % bad_short)
    if bad_unpack:
        print("      %d rows failed to decode" % bad_unpack)
    if read_errors:
        print("      the read stopped early - there may be more beyond it")
    if total >= 0:
        print("      %d of %d nodes recovered (%.1f %%)"
              % (n, total, 100.0 * n / max(total, 1)))
    print("      DO NOT DELETE THIS DATABASE - the CSV is not a full substitute.")
    # Leave a marker beside the CSV so nobody has to remember this.
    with open(os.path.join(out, "INCOMPLETE.txt"), "w") as fh:
        fh.write("This graph is INCOMPLETE.\n"
                 "%d of %d nodes recovered.\n"
                 "%d rows unreadable (type %d, short %d, decode %d).\n"
                 "The source database must be kept.\n"
                 % (n, total, skipped, bad_type, bad_short, bad_unpack))
elif total >= 0 and n == total:
    print("  complete: all %d nodes recovered" % n)
