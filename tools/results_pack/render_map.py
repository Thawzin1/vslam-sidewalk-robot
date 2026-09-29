#!/usr/bin/env python3
"""render_map.py (copy for drive 4: adds --source-label only) - draw a run's floor-plan map straight from its database.

WHY
    The usual way to see an RTAB-Map map is its own GUI or RViz, both heavy.
    This rebuilds the 2D occupancy map from the database file alone, with no
    ROS and no GUI, and writes a PNG plus a one-page HTML summary. It runs on
    the Jetson; your own computer only has to open a web page.

    It also works on a database that was NOT shut down cleanly. Drive 1 of
    series 2 lost its visual dictionary to an unclean stop, but every node kept
    its own small occupancy patch, and those are all this needs.

HOW THE MAP IS STORED (verified on this build, 2026-09-24)
    Each row of the Data table carries three patches in the node's own frame:
        ground_cells    floor the robot saw
        obstacle_cells  something solid
        empty_cells     space a ray passed through (ray tracing)
    Each blob is zlib-compressed float data followed by three little-endian
    int32s - rows, cols, OpenCV type. Type 29 is CV_32FC4: four floats per
    cell, x y z and a packed colour. Cell size is in the same row (0.05 m).
    The node's pose is 12 floats, a row-major 3x4 [R|t].

HOW CELLS ARE COMBINED
    Every cell of every node is moved into the map frame by that node's pose
    and counted. A map cell is drawn as SOLID if it was seen as an obstacle
    more often than as free, as FREE if the reverse, and left UNKNOWN (grey) if
    it was never seen. Unknown is not the same as empty - it means the robot
    never looked there.

WHICH POSITIONS (added 2026-09-24, after the drive-1 diagnosis)
    By default this draws from Node.pose, which is the camera's raw tracking
    (odometry): written once when each node is made and never corrected by loop
    closures. So the default picture is NOT the map RTAB-Map believes in - it is
    what the tracking alone would have drawn.

    --poses <corrected.tum> draws from corrected positions instead (the file
    optimize_graph_se2.py writes, one line per graph node with the node id as
    the ninth column). Nodes that are not in the graph - the robot standing
    still, mostly - take the correction of the nearest earlier graph node.

    Every output names which positions it was drawn from.

  usage:
    render_map.py <database.db> <output_dir> [--title "..."]
                  [--poses corrected.tum] [--name map_corrected]
                  [--npz cells.npz] [--no-png]

    --npz writes the finished occupancy picture (as numbers), its extent and
    the path, so the map can be drawn under another figure somewhere else -
    the LiDAR comparison draws the robot's LiDAR map this way. --no-png skips
    the picture, so only numpy is needed.
"""
from __future__ import print_function

import argparse
import json
import math
import os
import sqlite3
import struct
import sys
import zlib

import numpy as np

# matplotlib is imported only when a picture is drawn, so --npz --no-png runs
# on a machine that has numpy and nothing else (the robot's computer).

TEAL = "#1F8A8C"
RUST = "#C0532B"
INK = "#22252A"
GREY = "#B8B4AC"


def decode_cells(blob):
    """blob -> (N, 3) float array of x, y, z in the node frame, or None."""
    if blob is None:
        return None
    b = bytes(blob)
    if len(b) < 12:
        return None
    rows, cols, cvtype = struct.unpack("<iii", b[-12:])
    try:
        raw = zlib.decompress(b[:-12])
    except zlib.error:
        return None
    n = rows * cols
    if n == 0:
        return None
    # channels from the OpenCV type code: type = depth + (channels-1)*8
    channels = (cvtype >> 3) + 1
    arr = np.frombuffer(raw, dtype=np.float32)
    if arr.size != n * channels:
        return None
    arr = arr.reshape(n, channels)
    return arr[:, :3]


def pose_matrix(blob):
    v = struct.unpack("<12f", bytes(blob)[:48])
    R = np.array([[v[0], v[1], v[2]], [v[4], v[5], v[6]], [v[8], v[9], v[10]]])
    t = np.array([v[3], v[7], v[11]])
    return R, t


def yaw_of(R):
    return math.atan2(R[1, 0], R[0, 0])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def read_corrected(path):
    """corrected.tum -> {node id: (x, y, yaw)}; the id is the ninth column."""
    out = {}
    with open(path) as fh:
        for line in fh:
            p = line.split()
            if not p or line.startswith("#") or len(p) < 9:
                continue
            qx, qy, qz, qw = (float(v) for v in p[4:8])
            yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
            out[int(float(p[8]))] = (float(p[1]), float(p[2]), yaw)
    return out


def apply_corrections(poses, corrected):
    """Move every node from its odometry pose to its corrected pose.

    For a graph node k the correction is C_k = T_corrected(k) * T_odometry(k)^-1,
    a rotation about the vertical plus a shift in the plane (the runs use
    Reg/Force3DoF). A node outside the graph takes C of the nearest earlier
    graph node, so it keeps its place relative to that node."""
    ids = sorted(corrected)
    fixes = {}
    for k in ids:
        if k not in poses:
            continue
        R, t = poses[k]
        x, y, yaw = corrected[k]
        Rc = rot_z(yaw - yaw_of(R))
        tc = np.array([x, y, t[2]]) - Rc @ t
        fixes[k] = (Rc, tc)
    have = sorted(fixes)
    if not have:
        sys.exit("  none of the corrected ids match a node in the database")
    out = {}
    j = 0
    for nid in sorted(poses):
        while j + 1 < len(have) and have[j + 1] <= nid:
            j += 1
        Rc, tc = fixes[have[j]]
        R, t = poses[nid]
        out[nid] = (Rc @ R, Rc @ t + tc)
    return out, len(have)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("db")
    ap.add_argument("out")
    ap.add_argument("--title", default="")
    ap.add_argument("--source-label", default="",
                    help="drive 4 copy: overrides the 'drawn from' line and facts.json positions_source "
                         "(in a fused drive Node.pose is the blend's position, not the camera's)")
    ap.add_argument("--poses", default="",
                    help="corrected.tum from optimize_graph_se2.py")
    ap.add_argument("--npz", default="", help="also write the grid + path as .npz")
    ap.add_argument("--zband", nargs=2, type=float, default=None, metavar=("ZMIN", "ZMAX"),
                    help="keep only obstacle cells this high above the robot's base "
                         "(m). The robot's LiDAR map stores the floor as obstacle "
                         "(Grid/GroundIsObstacle=true); 0.15 2.0 keeps the walls")
    ap.add_argument("--cell", type=float, default=0.0,
                    help="draw at this cell size instead of the stored one (the LiDAR "
                         "map stores 0.02 m; 0.05 m is plenty for a picture)")
    ap.add_argument("--no-png", action="store_true", help="skip the picture")
    ap.add_argument("--name", default="map",
                    help="output base name: <name>.png and <name>_facts.json "
                         "(default map.png / facts.json)")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    con = sqlite3.connect("file:%s?immutable=1" % args.db, uri=True)

    poses = {}
    stamps = {}
    for nid, stamp, blob in con.execute(
            "SELECT id, stamp, pose FROM Node ORDER BY id"):
        if blob is None or len(bytes(blob)) < 48:
            continue
        poses[nid] = pose_matrix(blob)
        stamps[nid] = stamp

    source = "odometry (Node.pose) - the camera's tracking, NOT corrected by loop closures"
    if args.source_label and not args.poses:
        source = args.source_label
    graph_nodes = None
    if args.poses:
        poses, graph_nodes = apply_corrections(poses, read_corrected(args.poses))
        source = "corrected (after loop closures) - %s" % os.path.basename(args.poses)

    occ, free = {}, {}
    cell = None
    used = 0
    for nid, g, o, e, cs in con.execute(
            "SELECT id, ground_cells, obstacle_cells, empty_cells, cell_size "
            "FROM Data ORDER BY id"):
        if nid not in poses:
            continue
        cell = cell or args.cell or (cs if cs else 0.05)
        R, t = poses[nid]
        got = False
        for blob, target in ((g, free), (e, free), (o, occ)):
            pts = decode_cells(blob)
            if pts is None or len(pts) == 0:
                continue
            if args.zband and target is occ:
                pts = pts[(pts[:, 2] >= args.zband[0]) & (pts[:, 2] <= args.zband[1])]
                if len(pts) == 0:
                    continue
            got = True
            w = pts @ R.T + t
            ij = np.floor(w[:, :2] / cell).astype(np.int64)
            keys, counts = np.unique(ij, axis=0, return_counts=True)
            for (i, j), c in zip(keys, counts):
                target[(i, j)] = target.get((i, j), 0) + int(c)
        used += int(got)
    con.close()

    if not occ and not free:
        sys.exit("  no occupancy cells could be decoded from %s" % args.db)

    allk = list(occ.keys()) + list(free.keys())
    imin = min(k[0] for k in allk); imax = max(k[0] for k in allk)
    jmin = min(k[1] for k in allk); jmax = max(k[1] for k in allk)
    W, H = imax - imin + 1, jmax - jmin + 1
    # A map with no free cells at all (the robot's LiDAR map) is walls only:
    # draw it on white, because grey would claim "never seen" for everything.
    img = np.full((H, W), 0.72 if free else 1.0)    # unknown = grey
    n_free = n_occ = 0
    for k in set(allk):
        o, f = occ.get(k, 0), free.get(k, 0)
        x, y = k[0] - imin, k[1] - jmin
        if o > f:
            img[H - 1 - y, x] = 0.08; n_occ += 1    # solid = near black
        else:
            img[H - 1 - y, x] = 1.0; n_free += 1    # free = white

    # trajectory
    ids = sorted(poses)
    xy = np.array([[poses[i][1][0], poses[i][1][1]] for i in ids])
    path = float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1))) if len(xy) > 1 else 0.0
    gap = float(np.linalg.norm(xy[-1] - xy[0])) if len(xy) > 1 else 0.0
    dur = (stamps[ids[-1]] - stamps[ids[0]]) if ids else 0.0

    extent = [imin * cell, (imax + 1) * cell, jmin * cell, (jmax + 1) * cell]
    if args.npz:
        np.savez_compressed(args.npz, img=img.astype(np.float32), extent=np.array(extent),
                            path_xy=xy, path_ids=np.array(ids), cell=cell,
                            positions="corrected" if args.poses else "odometry")
    if args.no_png:
        print(json.dumps({"npz": args.npz, "nodes": len(poses), "cells_solid": n_occ,
                          "cells_free": n_free, "closed_loop_gap_m": round(gap, 3)}))
        return 0
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig_w = min(14, max(7, W / 60.0))
    fig_h = fig_w * H / max(W, 1) + 0.9
    fig, ax = plt.subplots(figsize=(fig_w, min(fig_h, 14)))
    ax.imshow(img, cmap="gray", vmin=0, vmax=1, extent=extent,
              interpolation="nearest", origin="upper")
    ax.plot(xy[:, 0], xy[:, 1], color=TEAL, lw=1.4, alpha=0.95)
    ax.plot(xy[0, 0], xy[0, 1], "o", color=TEAL, ms=9, mec="white", mew=1.5)
    ax.plot(xy[-1, 0], xy[-1, 1], "s", color=RUST, ms=8, mec="white", mew=1.5)
    ax.annotate("start", xy[0], textcoords="offset points", xytext=(8, 8),
                color=TEAL, fontsize=9, fontweight="bold")
    ax.annotate("end", xy[-1], textcoords="offset points", xytext=(8, -14),
                color=RUST, fontsize=9, fontweight="bold")
    # 2 m scale bar
    sx, sy = extent[0] + 0.5, extent[2] + 0.5
    ax.plot([sx, sx + 2.0], [sy, sy], color=INK, lw=3)
    ax.text(sx + 1.0, sy + 0.25, "2 m", ha="center", fontsize=9, color=INK)
    ax.set_aspect("equal")
    ax.set_xticks([]); ax.set_yticks([])
    for s in ax.spines.values():
        s.set_visible(False)
    ax.set_title(args.title or os.path.basename(args.db), loc="left",
                 fontsize=12, color=INK)
    ax.text(0.0, -0.02, "drawn from: " + source, transform=ax.transAxes,
            ha="left", va="top", fontsize=8.5,
            color=TEAL if args.poses else RUST)
    fig.tight_layout()
    png = os.path.join(args.out, args.name + ".png")
    fig.savefig(png, dpi=140, facecolor="white")
    plt.close(fig)

    facts = {
        "database": os.path.abspath(args.db),
        "positions": "corrected" if args.poses else "odometry",
        "positions_source": source,
        "graph_nodes_corrected": graph_nodes,
        "nodes": len(poses),
        "nodes_with_grids": used,
        "duration_s": round(dur, 1),
        "path_m_nodes": round(path, 2),
        "closed_loop_gap_m": round(gap, 3),
        "cells_solid": n_occ,
        "cells_free": n_free,
        "cell_size_m": cell,
        "map_width_m": round(W * cell, 1),
        "map_height_m": round(H * cell, 1),
    }
    facts_name = "facts.json" if args.name == "map" else args.name + "_facts.json"
    with open(os.path.join(args.out, facts_name), "w") as fh:
        json.dump(facts, fh, indent=2)
    print(json.dumps(facts, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
