#!/usr/bin/env python3
"""compare_returns.py - the head-to-head that runs 9a and 9b exist for.

THE QUESTION

    Both runs drove the same route to the same landmark. Only the way home
    differed:

        9a  drove BACKWARD, never turning - camera faced the same way throughout
        9b  TURNED AROUND and drove forward - camera faced the opposite way

    If what matters is the camera's heading, 9a's return closes loops and 9b's
    does not. If what matters is travelling back over old ground, both fail.

WHY THIS SCRIPT AND NOT AN EYEBALL

    Three numbers decide it, and two of them are easy to get wrong:

    1. CLOSURES ON THE RETURN LEG. The leg must come from the WHEELS, not from
       memory - segment_run.py does that. Closures must be counted as distinct
       UNORDERED pairs, because RTAB-Map stores every link twice and counting
       rows doubles everything.

    2. ZERO-INLIER REJECTIONS WITH MANY MATCHES. Not the bare rejection count.
       A rejection with 5 matches and 0 inliers is a weak candidate; one with
       30 matches and 0 inliers is the map recognising the place and failing to
       line it up, which is the reversal failure. Run 8 scored 3, run 7c's
       reversed lap scored 7, run 9a scored 0.

    3. HOW FAR THROUGH THE RUN CLOSURES REACHED. Run 7c stopped at 74 %.

    usage:
      rosrun sidewalk_evaluation compare_returns.py
"""
from __future__ import print_function

import math
import os
import re
import sqlite3
import subprocess
import sys

HOME = os.path.expanduser("~")
MAPDIR = "/media/sidewalk/SIDEWALK128/rtabmap_maps/2026-09-01"
RUNS = [("9a  backward return", "lab_map_09a"),
        ("9b  turned return", "lab_map_09b")]

ZERO_RE = re.compile(
    r"Rejected loop closure \d+ -> \d+: Not enough inliers 0/20 \(matches=(\d+)\)")


def db_facts(db):
    con = sqlite3.connect("file:%s?immutable=1" % db, uri=True)
    poses = {}
    import struct
    for nid, blob in con.execute("SELECT id, pose FROM Node ORDER BY id"):
        if blob and len(blob) >= 48:
            v = struct.unpack("<12f", bytes(blob)[:48])
            poses[nid] = (v[3], v[7], v[11])
    q = ("SELECT DISTINCT MIN(from_id,to_id), MAX(from_id,to_id) FROM Link "
         "WHERE type IN (1,2,3,4)")
    pairs = [p for p in con.execute(q) if p[0] != p[1]]
    con.close()
    if not poses:
        return None
    ids = sorted(poses)
    lo, hi = ids[0], ids[-1]
    path = sum(math.sqrt(sum((poses[a][i] - poses[b][i]) ** 2 for i in range(3)))
               for a, b in zip(ids, ids[1:]))
    err = math.sqrt(sum((poses[lo][i] - poses[hi][i]) ** 2 for i in range(3)))
    highest = max(max(a, b) for a, b in pairs) if pairs else lo
    return {"nodes": len(poses), "path": path, "err": err,
            "closures": len(pairs), "highest": highest, "hi": hi, "lo": lo,
            "pct": 100.0 * (highest - lo) / max(hi - lo, 1)}


def log_facts(log):
    if not os.path.exists(log):
        return None
    txt = open(log, "rb").read().decode("utf-8", "replace")
    zero = ZERO_RE.findall(txt)
    return {"rejections": txt.count("Rejected loop closure"),
            "zero_inlier": len(zero),
            "strong": sum(1 for m in zero if int(m) >= 20),
            "matches": sorted(int(m) for m in zero)}


def return_leg(run):
    """Ask segment_run.py, which cuts legs from the raw wheel odometry.

    NEVER TRUNCATE THE ERROR. The first version of this function returned
    str(exc)[:80], which cut the message off exactly where the cause would
    have been - and truncated error reporting has produced a false conclusion
    in this project three times already. If the child fails, show what it said.

    segment_run.py imports rosbag, which needs the ROS environment. A plain
    subprocess does not inherit it when this is run from a bare shell, so the
    environment is set up explicitly rather than assumed.
    """
    script = os.path.join(HOME, "catkin_ws/src/sidewalk_evaluation/scripts",
                          "segment_run.py")
    if not os.path.exists(script):
        return None, "segment_run.py not found at %s" % script

    env = dict(os.environ)
    # Make rosbag importable even when this is launched without sourcing ROS.
    for p in ("/opt/ros/noetic/lib/python3/dist-packages",
              os.path.join(HOME, "catkin_ws/devel/lib/python3/dist-packages")):
        if os.path.isdir(p) and p not in env.get("PYTHONPATH", ""):
            env["PYTHONPATH"] = p + ":" + env.get("PYTHONPATH", "")

    proc = subprocess.Popen([sys.executable, script, "--run", run],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            env=env)
    out = proc.communicate()[0].decode("utf-8", "replace")
    if proc.returncode != 0:
        tail = "\n".join("      | " + l for l in out.strip().splitlines()[-8:])
        return None, "segment_run.py exited %d. It said:\n%s" % (
            proc.returncode, tail)

    m = re.search(r">>> RETURN LEG \(last leg over 1\.5 m\): (\S+)\s*\n"
                  r"\s+([\d.]+) s, ([\d.]+) m, (\d+) nodes, (\d+) closures, ([\d.]+) per node",
                  out)
    if not m:
        tail = "\n".join("      | " + l for l in out.strip().splitlines()[-8:])
        return None, "no return leg in its output. Last lines:\n%s" % tail
    return {"kind": m.group(1), "secs": float(m.group(2)),
            "m": float(m.group(3)), "nodes": int(m.group(4)),
            "closures": int(m.group(5)), "rate": float(m.group(6))}, None


print("=" * 78)
print("  RETURN-LEG COMPARISON - the only valid one")
print("  (within a run, forward-vs-backward reads LOCATION as DIRECTION)")
print("=" * 78)

results = {}
for label, run in RUNS:
    db = os.path.join(MAPDIR, run + ".db")
    log = os.path.join(HOME, ".run_records", run, "mapping.log")
    print()
    print("  %s   [%s]" % (label, run))
    if not os.path.exists(db):
        print("    database not found - run not done yet")
        continue
    d = db_facts(db)
    l = log_facts(log)
    leg, err = return_leg(run)
    results[run] = (d, l, leg)
    if d:
        print("    nodes %d   path %.2f m   closed-loop error %.3f m (%.2f %%)"
              % (d["nodes"], d["path"], d["err"], 100 * d["err"] / max(d["path"], 1e-9)))
        print("    closures %d   highest node in a closure %d of %d = %.1f %%"
              % (d["closures"], d["highest"], d["hi"], d["pct"]))
    if l:
        print("    closure rejections %d   zero-inlier %d   OF WHICH 20+ matches: %d"
              % (l["rejections"], l["zero_inlier"], l["strong"]))
        if l["matches"]:
            print("      match counts on zero-inlier rejections: %s"
                  % ", ".join(str(x) for x in l["matches"][:14]))
    if leg:
        print("    RETURN LEG: %s  %.0f s, %.2f m, %d nodes, %d closures, %.3f per node"
              % (leg["kind"], leg["secs"], leg["m"], leg["nodes"],
                 leg["closures"], leg["rate"]))
    elif err:
        print("    return leg: %s" % err)

a = results.get("lab_map_09a")
b = results.get("lab_map_09b")
print()
print("=" * 78)
if not (a and b and a[2] and b[2]):
    print("  Both runs needed before a verdict. Nothing concluded.")
    sys.exit(0)

ra, rb = a[2]["rate"], b[2]["rate"]
sa = a[1]["strong"] if a[1] else 0
sb = b[1]["strong"] if b[1] else 0
print("  RETURN-LEG CLOSURE RATE   9a backward %.3f   vs   9b turned %.3f" % (ra, rb))
print("  REVERSAL SIGNATURE (0 inliers, 20+ matches)   9a %d   vs   9b %d" % (sa, sb))
print()
if rb < 0.5 * ra and sb > sa:
    print("  THE PREDICTION HELD. Turning the camera around breaks loop closure;")
    print("  driving backward with the camera unturned does not. Direction of")
    print("  TRAVEL is not the cause - camera HEADING is.")
    print("  Consequence: a sidewalk out-and-back can return in reverse and keep")
    print("  its map, with no extra hardware.")
elif rb >= 0.5 * ra:
    print("  THE PREDICTION FAILED. The turned return closed loops about as well")
    print("  as the backward one. Camera heading is NOT what broke run 7c, and")
    print("  the explanation in ENGINEERING_NOTES.md section 2.13 needs replacing.")
else:
    print("  MIXED. The rate fell but the signature did not appear, or vice")
    print("  versa. State both numbers; do not round this into a verdict.")
print()
print("  N=1 per condition. This settles a mechanism, not an error figure.")
