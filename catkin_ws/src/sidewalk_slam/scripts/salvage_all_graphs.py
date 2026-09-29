#!/usr/bin/env python3
"""salvage_all_graphs.py - pull the pose graph out of every database, while they still read.

WHY

    On 2026-08-31 a full integrity sweep found SEVEN of fifteen RTAB-Map
    databases on the microSD card structurally damaged - 47 %. Every one of
    them still yielded its pose graph, because the damage is concentrated in
    the stored images, which are about 99.9 % of the bytes and are needed for
    almost nothing.

    That may not last. This project has lost files AT REST, hours after a clean
    check, more than once. The graph - the robot's positions and the links
    between them - is what every result in the project is actually made of. It
    is a few megabytes against 62.8 GB.

    Plain terms: the photo album is rotting, but the index card with all the
    facts on it is still readable. Copy the index cards now.

WHY IT DEDUPLICATES BY NODE ID

    A corrupted b-tree can revisit rows during a scan, so a plain SELECT can
    return the same row several times. Two databases here report MORE poses
    than they have nodes, which is impossible and is exactly this. Deduplicating
    by id turns a silently wrong count into a correct one.

WHERE IT WRITES

    The Jetson's INTERNAL disk (~/graph_salvage), never the card. Small files,
    and not on the medium under suspicion.

    usage:
      rosrun sidewalk_slam salvage_all_graphs.py
      rosrun sidewalk_slam salvage_all_graphs.py --out /somewhere/else
"""
from __future__ import print_function

import argparse
import glob
import math
import os
import sqlite3
import struct
import sys

DEFAULT_DIR = "/media/sidewalk/SIDEWALK128/rtabmap_maps"


def salvage(db_path, out_root):
    stem = os.path.basename(db_path)[:-3]
    out = os.path.join(out_root, stem)
    if not os.path.isdir(out):
        os.makedirs(out)

    try:
        con = sqlite3.connect("file:%s?immutable=1" % db_path, uri=True)
    except sqlite3.Error as exc:
        print("  %-32s CANNOT OPEN: %s" % (stem, str(exc)[:44]))
        return None

    cur = con.cursor()
    n_pose = 0
    n_link = 0
    dupes = 0
    notes = []

    try:
        seen = set()
        with open(os.path.join(out, "poses.csv"), "w") as fh:
            fh.write("id,stamp,x,y,z,yaw_deg\n")
            for nid, stamp, blob in cur.execute(
                    "SELECT id,stamp,pose FROM Node ORDER BY id"):
                if nid in seen:
                    dupes += 1
                    continue
                seen.add(nid)
                if blob is None or len(bytes(blob)) < 48:
                    continue
                # 3x4 row-major float32 transform: 12 floats, 48 bytes
                v = struct.unpack("<12f", bytes(blob)[:48])
                fh.write("%d,%.6f,%.4f,%.4f,%.4f,%.2f\n"
                         % (nid, stamp, v[3], v[7], v[11],
                            math.degrees(math.atan2(v[4], v[0]))))
                n_pose += 1
    except (sqlite3.Error, struct.error) as exc:
        notes.append("poses truncated: %s" % str(exc)[:40])

    try:
        with open(os.path.join(out, "links.csv"), "w") as fh:
            fh.write("from_id,to_id,type\n")
            for a, b, t in cur.execute("SELECT from_id,to_id,type FROM Link"):
                fh.write("%s,%s,%s\n" % (a, b, t))
                n_link += 1
    except sqlite3.Error as exc:
        notes.append("links truncated: %s" % str(exc)[:40])

    con.close()

    tail = ""
    if dupes:
        tail += "  [%d duplicate rows skipped - damaged b-tree]" % dupes
    if notes:
        tail += "  [%s]" % "; ".join(notes)
    print("  %-32s %6d poses  %6d link rows%s" % (stem, n_pose, n_link, tail))
    return {"stem": stem, "poses": n_pose, "links": n_link, "dupes": dupes}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=DEFAULT_DIR)
    ap.add_argument("--out", default=os.path.expanduser("~/graph_salvage"))
    a = ap.parse_args()

    files = sorted(glob.glob(os.path.join(a.dir, "*", "*.db")),
                   key=os.path.getmtime)
    if not files:
        print("  no databases found under %s" % a.dir)
        return 1

    total_gb = sum(os.path.getsize(f) for f in files) / (1024.0 ** 3)
    print("  %d databases, %.1f GB of file, salvaging the graphs into %s"
          % (len(files), total_gb, a.out))
    print()

    results = [r for r in (salvage(f, a.out) for f in files) if r]

    print()
    print("  %d of %d salvaged" % (len(results), len(files)))
    damaged = [r for r in results if r["dupes"]]
    if damaged:
        print("  %d had duplicate rows from b-tree damage - their counts would"
              % len(damaged))
        print("  have been WRONG without deduplication:")
        for r in damaged:
            print("    %s: %d duplicates skipped" % (r["stem"], r["dupes"]))
    empty = [r for r in results if r["poses"] == 0]
    if empty:
        print("  %d yielded NO poses at all:" % len(empty))
        for r in empty:
            print("    %s" % r["stem"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
