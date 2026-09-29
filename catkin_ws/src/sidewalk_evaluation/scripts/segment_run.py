#!/usr/bin/env python3
"""segment_run.py - cut a run into legs, and judge loop closure in each one.

WHY THIS EXISTS

    Run 9a drives out along a walkway and comes back BACKWARD, without turning.
    Run 9b drives out the same way and comes back after TURNING AROUND. The
    prediction is that these behave oppositely: the backward return keeps
    closing loops because the camera faces the same way throughout, and the
    turned return stops closing because it does not.

    Testing that means asking "how did loop closure behave during the return
    leg specifically", which needs the run cut into legs first. Doing that from
    memory at one in the morning is how a boundary gets guessed, and a guessed
    boundary would put the answer in question.

HOW THE LEGS ARE FOUND

    From the recorded WHEEL odometry, not from notes. The driver pauses for
    about five seconds before every change of direction, and a five-second
    stationary gap is unmistakable in the data.

    Within each leg, the sign of forward velocity says what it was:

        forward   : median linear velocity clearly positive
        BACKWARD  : median linear velocity clearly negative   <- the interesting one
        turning   : little linear motion, sustained rotation
        stopped   : neither

    Plain terms: the wheels tell us which way the robot was going, second by
    second. We only have to look for where that changed.

WHICH ODOMETRY, AND WHY IT MATTERS

    /husky_velocity_controller/odom - the RAW wheels.

    NOT /odometry/filtered, which is the wheels fused with the robot's motion
    sensor. Measured on 2026-09-01: the raw stream's timestamps sit 1.4 ms from
    the Jetson's clock with a 1.2 ms spread; the fused stream sits at 18.8 ms
    with a 41 ms spread - thirteen times worse. For lining legs up against map
    node timestamps, that wobble is the measurement.

    usage:
      rosrun sidewalk_evaluation segment_run.py --run lab_map_09a
"""
from __future__ import print_function

import argparse
import os
import sys

# Thresholds. Stated here rather than buried, because they define the answer.
STOP_SPEED = 0.04      # m/s below this counts as stationary
STOP_SECS = 2.5        # a gap this long or longer separates two legs
MIN_LEG_SECS = 3.0     # shorter than this is not a leg, it is a wobble
TURN_RATE = 0.25       # rad/s sustained with little travel = turning in place


def load_wheel_odom(bag_path):
    """(t, vx, wz) per message, from the raw wheel odometry."""
    import rosbag
    out = []
    topic = "/husky_velocity_controller/odom"
    with rosbag.Bag(bag_path) as b:
        avail = set(b.get_type_and_topic_info()[1].keys())
        if topic not in avail:
            print("  %s NOT IN THE BAG. Present: %s" % (topic, sorted(avail)))
            return []
        for _, msg, t in b.read_messages(topics=[topic]):
            st = msg.header.stamp.to_sec() or t.to_sec()
            out.append((st, msg.twist.twist.linear.x, msg.twist.twist.angular.z))
    return out


def find_legs(samples):
    """Split on sustained stillness; classify what is between the gaps."""
    if not samples:
        return []
    legs, cur = [], []
    still_since = None
    for s in samples:
        moving = abs(s[1]) > STOP_SPEED or abs(s[2]) > TURN_RATE
        if moving:
            if still_since is not None and (s[0] - still_since) >= STOP_SECS:
                if cur:
                    legs.append(cur)
                cur = []
            still_since = None
            cur.append(s)
        else:
            if still_since is None:
                still_since = s[0]
    if cur:
        legs.append(cur)

    out = []
    for leg in legs:
        t0, t1 = leg[0][0], leg[-1][0]
        if (t1 - t0) < MIN_LEG_SECS:
            continue
        vx = sorted(s[1] for s in leg)
        wz = sorted(abs(s[2]) for s in leg)
        mvx = vx[len(vx) // 2]
        mwz = wz[len(wz) // 2]
        dist = 0.0
        for a, b in zip(leg, leg[1:]):
            dist += abs(a[1]) * (b[0] - a[0])
        if abs(mvx) < STOP_SPEED and mwz > TURN_RATE:
            kind = "TURNING IN PLACE"
        elif mvx < -STOP_SPEED:
            kind = "BACKWARD"
        elif mvx > STOP_SPEED:
            kind = "forward"
        else:
            kind = "stopped"
        out.append({"t0": t0, "t1": t1, "kind": kind, "median_vx": mvx,
                    "dist": dist, "n": len(leg)})
    return out


def node_stamps(db):
    import sqlite3
    con = sqlite3.connect("file:%s?immutable=1" % db, uri=True)
    return {i: s for i, s in con.execute(
        "SELECT id, stamp FROM Node ORDER BY id")}


def closures(db):
    """Distinct UNORDERED pairs. RTAB-Map stores every link twice."""
    import sqlite3
    con = sqlite3.connect("file:%s?immutable=1" % db, uri=True)
    q = ("SELECT DISTINCT MIN(from_id,to_id), MAX(from_id,to_id) FROM Link "
         "WHERE type IN (1,2,3,4)")
    return list(con.execute(q))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", required=True, help="e.g. lab_map_09a")
    ap.add_argument("--db", default=None)
    ap.add_argument("--bag", default=None)
    a = ap.parse_args()

    home = os.path.expanduser("~")
    bag = a.bag or "%s/.run_records/%s/motion/%s_robot.bag" % (home, a.run, a.run)
    db = a.db or ("/media/sidewalk/SIDEWALK128/rtabmap_maps/2026-09-01/%s.db"
                  % a.run)

    for p, what in ((bag, "robot motion bag"), (db, "database")):
        if not os.path.exists(p):
            print("  MISSING %s: %s" % (what, p))
            return 2

    print("=" * 74)
    print("  %s" % a.run)
    samples = load_wheel_odom(bag)
    print("  wheel odometry samples: %d" % len(samples))
    if not samples:
        return 2

    legs = find_legs(samples)
    stamps = node_stamps(db)
    pairs = closures(db)
    print("  map nodes: %d    loop closures (distinct pairs): %d"
          % (len(stamps), len(pairs)))

    if not stamps:
        print("  no nodes - nothing to attribute")
        return 1

    print()
    print("  %-3s %-17s %7s %8s %7s %8s %9s" %
          ("#", "leg", "secs", "metres", "nodes", "closures", "per node"))
    for i, leg in enumerate(legs, 1):
        ids = [n for n, s in stamps.items() if leg["t0"] <= s <= leg["t1"]]
        idset = set(ids)
        # A closure BELONGS to this leg if its newer end was recorded here.
        c = sum(1 for lo, hi in pairs if hi in idset)
        rate = (float(c) / len(ids)) if ids else float("nan")
        print("  %-3d %-17s %7.1f %8.2f %7d %8d %9.3f"
              % (i, leg["kind"], leg["t1"] - leg["t0"], leg["dist"],
                 len(ids), c, rate))

    print()
    print("  THE COMPARISON THIS RUN EXISTS FOR")
    fwd = [l for l in legs if l["kind"] == "forward"]
    bwd = [l for l in legs if l["kind"] == "BACKWARD"]
    trn = [l for l in legs if l["kind"] == "TURNING IN PLACE"]

    def rate_for(group):
        ids, c = set(), 0
        for leg in group:
            got = {n for n, s in stamps.items() if leg["t0"] <= s <= leg["t1"]}
            ids |= got
        c = sum(1 for lo, hi in pairs if hi in ids)
        return len(ids), c, (float(c) / len(ids) if ids else float("nan"))

    for name, group in (("forward legs", fwd), ("BACKWARD legs", bwd),
                        ("turning in place", trn)):
        n, c, r = rate_for(group)
        print("    %-18s %4d nodes, %4d closures, %.3f closures per node"
              % (name, n, c, r))

    print()
    print("  DO NOT COMPARE FORWARD AGAINST BACKWARD WITHIN ONE RUN.")
    print("  Closure rate depends on WHERE the robot is, not only which way it")
    print("  is going. Validated against run 8, driven forward throughout: its")
    print("  forward legs range from 0.000 to 1.000 closures per node, because")
    print("  legs over fresh ground cannot close and legs over mapped ground")
    print("  can. An outbound leg is fresh ground and a return leg is mapped")
    print("  ground, so a within-run comparison reads LOCATION as DIRECTION.")
    print()
    print("  THE VALID COMPARISON IS BETWEEN RUNS: 9a's backward return against")
    print("  9b's turned return. Same ground, same travel direction, only the")
    print("  camera's facing differs. Run this on both and compare the line")
    print("  below across the two.")

    # The return leg: the last substantial leg that is not the parking wobble.
    ret = [l for l in legs if l["dist"] > 1.5]
    if ret:
        last = ret[-1]
        ids = {n for n, s in stamps.items()
               if last["t0"] <= s <= last["t1"]}
        c = sum(1 for lo, hi in pairs if hi in ids)
        r = float(c) / len(ids) if ids else float("nan")
        print()
        print("  >>> RETURN LEG (last leg over 1.5 m): %s" % last["kind"])
        print("      %.1f s, %.2f m, %d nodes, %d closures, %.3f per node"
              % (last["t1"] - last["t0"], last["dist"], len(ids), c, r))
        print("      ^ THIS is the number to compare against the other run.")
    else:
        print()
        print("  No leg over 1.5 m found - cannot identify a return leg.")

    print()
    print("  Legs come from the WHEELS (raw /husky_velocity_controller/odom),")
    print("  independent of anything the camera or the map reported.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
