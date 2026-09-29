#!/usr/bin/env python3
"""db_corrected_tum.py - the CORRECTED path out of any RTAB-Map database, as TUM.

WHY
    A database's Node.pose column is the raw odometry, never corrected by loop
    closures (docs/SOLVED.md, 2026-09-24). The corrected path is in
    Admin.opt_poses - but only if the mapping program was shut down properly.

    This prefers RTAB-Map's own saved Admin.opt_poses, which is exact and full
    3D (the robot's LiDAR map is 3D: it keeps gravity links and is not forced
    flat). If that is missing it falls back to optimize_graph_se2.py, which
    rebuilds the graph in the plane - exact for the camera runs (they are
    forced flat), an APPROXIMATION for a 3D map, and the meta file says which.

    *Plain terms: take the map's own finished drawing of where the robot went
    if it saved one; if it did not, redraw it from the same measurements and
    say that is what was done.*

    For the robot's LiDAR map, remember what the "odometry" is: its launch file
    (self_navigation rtabmap_3d.launch) takes the wheel+IMU estimate
    /odometry/filtered and sets RGBD/NeighborLinkRefining=false, so between
    loop closures the LiDAR path IS the wheel+IMU path. The LiDAR corrects it
    where its scans match a place seen before. The closure count in the meta
    file says how often that happened.

  usage:
    db_corrected_tum.py <database.db> <out.tum> [--label "LiDAR (Helios)"]
      writes t x y z qx qy qz qw id, plus <out.tum>.meta.json
"""
from __future__ import print_function

import argparse
import json
import math
import os
import sqlite3
import struct
import subprocess
import sys
import zlib

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))


def decode(blob, want):
    """RTAB-Map compressData2: zlib stream + int32 rows, cols, OpenCV type."""
    b = bytes(blob)
    rows, cols, typ = struct.unpack("<iii", b[-12:])
    if typ != want:
        raise ValueError("unexpected matrix type %d (wanted %d)" % (typ, want))
    a = np.frombuffer(zlib.decompress(b[:-12]), dtype=np.int32 if typ == 4 else np.float32)
    if a.size != rows * cols:
        raise ValueError("matrix size %d does not match %dx%d" % (a.size, rows, cols))
    return a


def quat(R):
    tr = np.trace(R)
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    k = int(np.argmax(np.diag(R)))
    if k == 0:
        s = math.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return (0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s)
    if k == 1:
        s = math.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return ((R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s)
    s = math.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return ((R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("db")
    ap.add_argument("out")
    ap.add_argument("--label", default="")
    args = ap.parse_args()

    con = sqlite3.connect("file:%s?immutable=1" % args.db, uri=True)
    stamp = dict(con.execute("SELECT id, stamp FROM Node"))
    closures = con.execute("SELECT COUNT(*) FROM Link WHERE type IN (1,2,3,4) "
                           "AND from_id > to_id").fetchone()[0]
    row = con.execute("SELECT opt_ids, opt_poses FROM Admin").fetchone()
    con.close()

    meta = {"source_database": os.path.abspath(args.db), "pose_kind": "corrected",
            "label": args.label, "loop_closures": closures,
            "tool": "tools/db/db_corrected_tum.py"}

    if row and row[0] is not None and row[1] is not None:
        ids = decode(row[0], 4)
        P = decode(row[1], 5).reshape(-1, 3, 4).astype(float)
        with open(args.out, "w") as fh:
            fh.write("# CORRECTED poses from Admin.opt_poses (RTAB-Map's own, saved at "
                     "shutdown) of %s; cols t x y z qx qy qz qw id\n" % os.path.basename(args.db))
            for i, T in zip(ids, P):
                qx, qy, qz, qw = quat(T[:, :3])
                fh.write("%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %d\n"
                         % (stamp[int(i)], T[0, 3], T[1, 3], T[2, 3], qx, qy, qz, qw, int(i)))
        first, last = P[0][:, 3], P[-1][:, 3]
        meta.update({"method": "Admin.opt_poses - RTAB-Map's own saved optimisation, full 3D",
                     "exact": True, "poses": int(ids.size),
                     "first_node": int(ids[0]), "last_node": int(ids[-1]),
                     "end_gap_corrected_m": round(float(np.linalg.norm(last - first)), 4)})
        with open(args.out + ".meta.json", "w") as fh:
            json.dump(meta, fh, indent=2)
    else:
        print("  Admin.opt_poses is empty - the mapping was not shut down properly.")
        print("  Falling back to a planar re-optimisation (optimize_graph_se2.py).")
        rc = subprocess.call([sys.executable, os.path.join(HERE, "optimize_graph_se2.py"),
                              args.db, args.out], stdout=subprocess.DEVNULL)
        if rc != 0:
            sys.exit("  the fallback failed too")
        with open(args.out + ".meta.json") as fh:
            m = json.load(fh)
        m.update({k: v for k, v in meta.items() if k not in m})
        m["exact"] = False
        m["method"] += (" - FALLBACK because Admin.opt_poses was empty; exact for a map "
                        "forced flat (the camera runs), an approximation for a 3D map")
        meta = m
        with open(args.out + ".meta.json", "w") as fh:
            json.dump(meta, fh, indent=2)
    print(json.dumps(meta, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
