#!/usr/bin/env python3
"""db_to_tum.py - turn an RTAB-Map database into a TUM trajectory file.

WHY THIS EXISTS
---------------
The camera's map and the LiDAR's map are each an RTAB-Map database. To ask how
much they agree, both have to come out as the same kind of file, and the tool
that compares them (sidewalk_evaluation/scripts/evaluate_trajectory.py, and evo)
reads the TUM format:

    timestamp  tx ty tz  qx qy qz qw        (one line per pose, space separated)

extract_graph.py already pulls poses out of a database, but it writes
x,y,z,yaw_deg - which throws away the full rotation and cannot be fed to a
trajectory comparison. This script keeps the whole orientation.

It works on EITHER database, so one tool produces both sides of the comparison:

    db_to_tum.py  lidar_run1.db   out/lidar.tum
    db_to_tum.py  camera_run1.db  out/camera.tum

WHAT IT GUARDS AGAINST  (all three have already happened in this project)
-------------------------------------------------------------------------
1. A DAMAGED B-TREE REVISITS ROWS - a plain scan once produced 19,718 "poses"
   for a 1,269-node map. Reading ORDER BY id and skipping ids already seen
   fixes it.
2. A DAMAGED DATABASE RETURNS THE WRONG TYPE - the pose column has come back as
   an INTEGER where a blob belongs. Every row is handled defensively; bad rows
   are counted and reported rather than crashing part way through and leaving a
   truncated file that looks complete.
3. A NULL OR ZERO STAMP cannot be aligned against anything. Those rows are
   counted separately and, by default, refused - a trajectory whose timestamps
   are all 0.0 will silently "align" to anything and produce a meaningless
   number.

The run ends by stating how many poses were written out of how many nodes exist.
If anything was skipped it says so, and writes INCOMPLETE.txt beside the output.

  usage:
    db_to_tum.py <database.db> <output.tum> [--allow-zero-stamps] [--quiet]
"""
from __future__ import print_function

import json
import math
import os
import sqlite3
import struct
import sys


def matrix_to_quaternion(r00, r01, r02, r10, r11, r12, r20, r21, r22):
    """3x3 rotation matrix -> (qx, qy, qz, qw).

    Uses the branch with the largest denominator, which is the numerically
    stable way to do this - the naive single-branch formula loses precision
    when the trace is near zero.
    """
    trace = r00 + r11 + r22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (r21 - r12) / s
        qy = (r02 - r20) / s
        qz = (r10 - r01) / s
    elif r00 > r11 and r00 > r22:
        s = math.sqrt(1.0 + r00 - r11 - r22) * 2.0
        qw = (r21 - r12) / s
        qx = 0.25 * s
        qy = (r01 + r10) / s
        qz = (r02 + r20) / s
    elif r11 > r22:
        s = math.sqrt(1.0 + r11 - r00 - r22) * 2.0
        qw = (r02 - r20) / s
        qx = (r01 + r10) / s
        qy = 0.25 * s
        qz = (r12 + r21) / s
    else:
        s = math.sqrt(1.0 + r22 - r00 - r11) * 2.0
        qw = (r10 - r01) / s
        qx = (r02 + r20) / s
        qy = (r12 + r21) / s
        qz = 0.25 * s
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n == 0.0:
        return 0.0, 0.0, 0.0, 1.0
    return qx / n, qy / n, qz / n, qw / n


def main(argv):
    args = [a for a in argv[1:] if not a.startswith("--")]
    flags = set(a for a in argv[1:] if a.startswith("--"))
    if len(args) < 2:
        sys.exit("  usage: db_to_tum.py <database.db> <output.tum> "
                 "[--allow-zero-stamps] [--quiet]")

    db, out_path = args[0], args[1]
    allow_zero = "--allow-zero-stamps" in flags
    quiet = "--quiet" in flags

    if not os.path.exists(db):
        sys.exit("  no such database: %s" % db)

    out_dir = os.path.dirname(os.path.abspath(out_path))
    if out_dir and not os.path.isdir(out_dir):
        os.makedirs(out_dir)

    # immutable=1 so a database still being written, or on a read-only mount,
    # can still be read.
    con = sqlite3.connect("file:%s?immutable=1" % db, uri=True)

    cols = [r[1] for r in con.execute("PRAGMA table_info(Node)")]
    has_stamp = "stamp" in cols
    if not has_stamp:
        print("  WARNING: this database has no stamp column - every timestamp "
              "will be 0.0 and the result cannot be time-aligned.")

    try:
        total = con.execute("SELECT COUNT(*) FROM Node").fetchone()[0]
    except sqlite3.Error:
        total = -1

    sel = "SELECT id, %s, pose FROM Node ORDER BY id" % (
        "stamp" if has_stamp else "0")

    rows = []
    read_stopped_early = False
    try:
        for row in con.execute(sel):
            rows.append(row)
    except sqlite3.Error as exc:
        read_stopped_early = True
        print("  READ STOPPED EARLY: %s" % str(exc)[:70])
    con.close()

    seen = set()
    bad_type = bad_short = bad_unpack = zero_stamp = 0
    written = 0
    first_stamp = last_stamp = None
    path_m = 0.0
    prev_xyz = None

    with open(out_path, "w") as fh:
        fh.write("# TUM trajectory from %s\n" % os.path.basename(db))
        fh.write("# timestamp tx ty tz qx qy qz qw\n")
        for nid, stamp, blob in rows:
            if nid in seen:
                continue
            seen.add(nid)
            if not isinstance(blob, (bytes, bytearray, memoryview)):
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

            try:
                st = float(stamp or 0.0)
            except (TypeError, ValueError):
                st = 0.0
            if st <= 0.0:
                zero_stamp += 1
                if not allow_zero:
                    continue

            # RTAB-Map stores a 3x4 row-major [R|t].
            tx, ty, tz = v[3], v[7], v[11]
            qx, qy, qz, qw = matrix_to_quaternion(
                v[0], v[1], v[2], v[4], v[5], v[6], v[8], v[9], v[10])

            fh.write("%.6f %.6f %.6f %.6f %.9f %.9f %.9f %.9f\n"
                     % (st, tx, ty, tz, qx, qy, qz, qw))
            written += 1
            if first_stamp is None:
                first_stamp = st
            last_stamp = st
            if prev_xyz is not None:
                path_m += math.sqrt((tx - prev_xyz[0]) ** 2
                                    + (ty - prev_xyz[1]) ** 2
                                    + (tz - prev_xyz[2]) ** 2)
            prev_xyz = (tx, ty, tz)

    skipped = bad_type + bad_short + bad_unpack
    meta = {
        "source_database": os.path.abspath(db),
        "output": os.path.abspath(out_path),
        "nodes_in_database": total,
        "poses_written": written,
        "skipped_bad_pose": skipped,
        "skipped_zero_stamp": 0 if allow_zero else zero_stamp,
        "zero_stamps_seen": zero_stamp,
        "read_stopped_early": read_stopped_early,
        "duration_s": (round(last_stamp - first_stamp, 3)
                       if (first_stamp is not None and last_stamp is not None)
                       else None),
        "path_length_m": round(path_m, 3),
        "format": "TUM: timestamp tx ty tz qx qy qz qw",
    }
    with open(out_path + ".meta.json", "w") as fh:
        json.dump(meta, fh, indent=2)
        fh.write("\n")

    if not quiet:
        print("  %s -> %s" % (os.path.basename(db), out_path))
        print("  %d poses written of %d nodes" % (written, total))
        if meta["duration_s"] is not None:
            print("  %.1f s, %.2f m of path" % (meta["duration_s"], path_m))

    if skipped or read_stopped_early or (zero_stamp and not allow_zero):
        print()
        print("  *** THIS TRAJECTORY IS INCOMPLETE ***")
        if bad_type:
            print("      %d rows had no usable pose (wrong type or NULL)"
                  % bad_type)
        if bad_short:
            print("      %d rows had a pose too short to decode" % bad_short)
        if bad_unpack:
            print("      %d rows failed to decode" % bad_unpack)
        if zero_stamp and not allow_zero:
            print("      %d rows had no timestamp and were REFUSED - they "
                  "cannot be aligned." % zero_stamp)
            print("      Pass --allow-zero-stamps only if you know why.")
        if read_stopped_early:
            print("      the read stopped early - there may be more beyond it")
        if total > 0:
            print("      %d of %d nodes recovered (%.1f %%)"
                  % (written, total, 100.0 * written / total))
        print("      DO NOT DELETE THE SOURCE DATABASE.")
        with open(out_path + ".INCOMPLETE.txt", "w") as fh:
            fh.write("This trajectory is INCOMPLETE.\n"
                     "%d of %d nodes written.\n"
                     "%d rows had an unusable pose; %d had no timestamp.\n"
                     "The source database must be kept.\n"
                     % (written, total, skipped, zero_stamp))
        return 2

    if written == 0:
        print("  NOTHING WAS WRITTEN - the trajectory is empty.")
        return 3
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
