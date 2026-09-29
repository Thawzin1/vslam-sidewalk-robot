#!/usr/bin/env python3
"""optimize_graph_se2.py - rebuild the CORRECTED map positions from a database.

WHY THIS EXISTS (found 2026-09-24, drive 1 of series 2)
    An RTAB-Map database keeps two different sets of positions, and they are
    easy to confuse:

      Node.pose          where the camera's step-by-step tracking (odometry)
                         put each map point, written ONCE when the point is
                         made and never changed afterwards. Loop closures do
                         not touch it.
      Admin.opt_poses    where the map puts each point AFTER loop closures have
                         pulled it into shape. Written only when the mapping
                         program is shut down properly.

    db_to_tum.py, render_map.py and extract_graph.py all read Node.pose. So a
    map drawn from them is the uncorrected tracking, and an end gap measured
    from them is odometry drift, not the SLAM result. Drive 1 was never shut
    down properly, so its Admin.opt_poses is empty - this script rebuilds it.

    *Plain terms: the database holds the robot's raw diary of where it thought
    it was, and separately the map's corrected version. Only the raw diary is
    guaranteed to be there. This recomputes the corrected version from the
    same links the map itself used.*

HOW
    Gauss-Newton on the pose graph in the plane (x, y, heading), because every
    run in this project uses Reg/Force3DoF=true. Edges are link types 0
    (neighbour), 1 (global closure), 2 (proximity closure), 3 (local time),
    4 ; self-links (gravity 9, prior 7) are skipped. The first graph node
    is held fixed at its odometry position, as RTAB-Map does with
    RGBD/OptimizeFromGraphEnd=false. Links stored in the newer->older direction
    are inverted, with their information matrix carried across.

VALIDATED
    Run 8 (series 1), whose database WAS closed properly: this reproduces
    RTAB-Map's own saved Admin.opt_poses on all 616 graph nodes - median
    difference 0.16 mm, largest 0.35 mm.

  usage:
    optimize_graph_se2.py <database.db> <out.tum>
      writes one line per graph node: t x y z qx qy qz qw id
      plus <out.tum>.meta.json naming the database (so build_slam_db.py treats
      it as a node series and does not resample it)
"""
from __future__ import print_function

import collections
import json
import math
import os
import sqlite3
import sys

import numpy as np

CLOSURE_TYPES = (1, 2, 3, 4)


def wrap(t):
    return (t + np.pi) % (2 * np.pi) - np.pi


def se2(blob):
    a = np.frombuffer(bytes(blob), dtype=np.float32)[:12].reshape(3, 4).astype(float)
    return np.array([a[0, 3], a[1, 3], math.atan2(a[1, 0], a[0, 0])])


def load(db):
    con = sqlite3.connect("file:%s?immutable=1" % db, uri=True)
    odom, weight, stamp = {}, {}, {}
    for i, b, w, s in con.execute("SELECT id, pose, weight, stamp FROM Node"):
        if b is None or len(bytes(b)) < 48:
            continue
        odom[i], weight[i], stamp[i] = se2(b), w, s
    edges = {}
    for f, t, ty, inf, tr in con.execute(
            "SELECT from_id, to_id, type, information_matrix, transform FROM Link"):
        if f == t or ty not in (0,) + CLOSURE_TYPES or f not in odom or t not in odom:
            continue
        z = se2(tr)
        I3 = np.frombuffer(bytes(inf), dtype=np.float64).reshape(6, 6)[np.ix_([0, 1, 5], [0, 1, 5])]
        if f > t:
            # Store every edge older -> newer. Invert the measurement and carry
            # the information matrix through the Jacobian of the inversion.
            p, th = z[:2].copy(), z[2]
            c, s = math.cos(th), math.sin(th)
            RT = np.array([[c, s], [-s, c]])
            dRT = np.array([[-s, c], [-c, -s]])
            J = np.zeros((3, 3)); J[:2, :2] = -RT; J[:2, 2] = -dRT @ p; J[2, 2] = -1
            Jinv = np.linalg.inv(J)
            I3 = Jinv.T @ I3 @ Jinv
            z = np.array([*(-RT @ p), -th])
            f, t = t, f
        edges.setdefault((f, t, ty), (z, I3))
    con.close()
    return odom, weight, stamp, edges


def optimize(odom, weight, edges):
    adj = collections.defaultdict(set)
    for f, t, _ in edges:
        adj[f].add(t); adj[t].add(f)
    root = min(i for i in odom if weight[i] >= 0)
    seen, todo = {root}, [root]
    while todo:
        u = todo.pop()
        for v in adj[u] - seen:
            seen.add(v); todo.append(v)
    nodes = sorted(seen)
    ix = {n: k for k, n in enumerate(nodes)}
    E = [(ix[f], ix[t], ty, z, I) for (f, t, ty), (z, I) in edges.items()
         if f in ix and t in ix]
    x = np.array([odom[n] for n in nodes])
    N = len(nodes)
    for _ in range(50):
        H = np.zeros((3 * N, 3 * N)); g = np.zeros(3 * N)
        for a, b, _ty, z, I in E:
            xi, xj = x[a], x[b]
            c, s = math.cos(xi[2]), math.sin(xi[2])
            RT = np.array([[c, s], [-s, c]])
            dRT = np.array([[-s, c], [-c, -s]])
            dx = xj[:2] - xi[:2]
            e = np.concatenate([RT @ dx - z[:2], [wrap(xj[2] - xi[2] - z[2])]])
            A = np.zeros((3, 3)); A[:2, :2] = -RT; A[:2, 2] = dRT @ dx; A[2, 2] = -1
            B = np.zeros((3, 3)); B[:2, :2] = RT; B[2, 2] = 1
            sa, sb = slice(3 * a, 3 * a + 3), slice(3 * b, 3 * b + 3)
            H[sa, sa] += A.T @ I @ A; H[sa, sb] += A.T @ I @ B
            H[sb, sa] += B.T @ I @ A; H[sb, sb] += B.T @ I @ B
            g[sa] += A.T @ I @ e; g[sb] += B.T @ I @ e
        r = 3 * ix[root]
        H[r:r + 3, :] = 0; H[:, r:r + 3] = 0; H[r:r + 3, r:r + 3] = np.eye(3); g[r:r + 3] = 0
        step = np.linalg.solve(H, -g)
        x = x + step.reshape(-1, 3); x[:, 2] = wrap(x[:, 2])
        if np.abs(step).max() < 1e-9:
            break
    closures = collections.Counter(ty for _a, _b, ty, _z, _I in E if ty in CLOSURE_TYPES)
    return nodes, x, root, closures


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    db, out = sys.argv[1], sys.argv[2]
    odom, weight, stamp, edges = load(db)
    nodes, x, root, closures = optimize(odom, weight, edges)
    last = nodes[-1]
    with open(out, "w") as fh:
        fh.write("# CORRECTED (loop-closure optimised) poses, rebuilt by optimize_graph_se2.py "
                 "from %s; cols t x y z qx qy qz qw id\n" % os.path.basename(db))
        for n, p in zip(nodes, x):
            fh.write("%.6f %.6f %.6f 0 0 0 %.6f %.6f %d\n"
                     % (stamp[n], p[0], p[1], math.sin(p[2] / 2), math.cos(p[2] / 2), n))
    gap_opt = float(np.linalg.norm(x[-1][:2] - x[0][:2]))
    gap_odo = float(np.linalg.norm(odom[last][:2] - odom[root][:2]))
    meta = {
        "source_database": os.path.abspath(db),
        "pose_kind": "corrected",
        "method": "SE(2) Gauss-Newton over Link types 0-4, first graph node fixed",
        "tool": "tools/db/optimize_graph_se2.py",
        "graph_nodes": len(nodes),
        "first_node": root,
        "last_node": last,
        "closure_edges_by_type": {str(k): v for k, v in sorted(closures.items())},
        "end_gap_corrected_m": round(gap_opt, 4),
        "end_gap_odometry_same_nodes_m": round(gap_odo, 4),
    }
    with open(out + ".meta.json", "w") as fh:
        json.dump(meta, fh, indent=2)
    print(json.dumps(meta, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
