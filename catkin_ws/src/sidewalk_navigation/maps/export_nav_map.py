#!/usr/bin/env python3
"""export_nav_map.py - write a saved camera map's own 2D floor plan as a map_server map (PGM picture + YAML).

RUNS ON the Jetson. Opens the map database READ-ONLY (sqlite "immutable" mode: the file cannot be changed,
not even its journal), reads Admin.opt_map - the 2D occupancy grid RTAB-Map saved when the drive closed, in the
same frame as the saved graph (start mark = 0, 0, facing east = +x) - and writes:

    <out>.pgm   one pixel per 5 cm cell: black 0 = wall, white 254 = seen free, grey 205 = never seen
    <out>.yaml  map_server's description: resolution, origin (the lower-left corner of the picture), thresholds
    <out>.json  provenance: source file, its sha256 (first 16), cell counts, extent

*Plain terms: the floor plan the robot drew on drive 10, saved as a picture the navigation program can read.*

Format facts (checked 27 Sept 2026 against the drive-10 master, read-only):
  - opt_map = zlib-compressed int8 cells followed by 12 bytes: rows, cols, OpenCV type (int32 each);
    decoded result is byte-identical to slam_session_2026-09-27/d10_optmap.npz (802 x 675 cells).
  - cells: -1 unknown, 0 free, 100 occupied. Row 0 is the LOWEST y (opt_map_y_min).
  - map_server's picture has its top row at the HIGHEST y, so the grid is flipped up-down before writing.
  - origin = (opt_map_x_min, opt_map_y_min): rtabmap_ros publishes its grid with exactly this origin, and a ROS
    OccupancyGrid origin is the corner of cell (0, 0).

usage:  export_nav_map.py ~/slam_series2/localise/s2_static_10_map.db  maps/d10_nav
"""
import hashlib
import json
import os
import sqlite3
import sys
import zlib

import numpy as np


def sha16(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for blk in iter(lambda: f.read(1 << 22), b""):
            h.update(blk)
    return h.hexdigest()[:16]


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    db, out = os.path.realpath(os.path.expanduser(sys.argv[1])), sys.argv[2]
    con = sqlite3.connect("file:%s?mode=ro&immutable=1" % db, uri=True)
    blob, xm, ym, res = con.execute(
        "select opt_map, opt_map_x_min, opt_map_y_min, opt_map_resolution from Admin").fetchone()
    con.close()
    if not blob:
        sys.exit("REFUSED: %s has no saved 2D map (Admin.opt_map empty: the drive was not closed properly)" % db)
    rows, cols, cvtype = np.frombuffer(blob[-12:], dtype=np.int32)
    g = np.frombuffer(zlib.decompress(blob[:-12]), dtype=np.int8).reshape(rows, cols)
    vals = set(np.unique(g).tolist())
    if not vals <= {-1, 0, 100}:
        sys.exit("REFUSED: unexpected cell values %s" % sorted(vals))
    img = np.full(g.shape, 205, dtype=np.uint8)
    img[g == 0] = 254
    img[g == 100] = 0
    img = np.flipud(img)
    with open(out + ".pgm", "wb") as f:
        f.write(b"P5\n# exported from %s Admin.opt_map\n%d %d\n255\n" % (os.path.basename(db).encode(), cols, rows))
        f.write(img.tobytes())
    with open(out + ".yaml", "w") as f:
        f.write("image: %s.pgm\n" % os.path.basename(out))
        f.write("resolution: %.6f\n" % res)
        f.write("origin: [%.6f, %.6f, 0.0]\n" % (xm, ym))
        f.write("negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\nmode: trinary\n")
    prov = {
        "source_db": db, "source_sha256_16": sha16(db), "opened": "read-only, sqlite immutable=1",
        "rows": int(rows), "cols": int(cols), "resolution_m": float(res),
        "origin_xy_m": [float(xm), float(ym)],
        "extent_m": {"x": [float(xm), float(xm + cols * res)], "y": [float(ym), float(ym + rows * res)]},
        "cells": {"unknown": int((g == -1).sum()), "free": int((g == 0).sum()), "occupied": int((g == 100).sum())},
    }
    with open(out + ".json", "w") as f:
        json.dump(prov, f, indent=1)
    print(json.dumps(prov, indent=1))


if __name__ == "__main__":
    main()
