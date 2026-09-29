#!/usr/bin/env python3
"""tf_one_parent_check.py - does every coordinate frame have exactly one parent?

RUNS ON whichever computer's ROS master is being checked (the robot before and after a
live LiDAR launch; the robot's isolated replay master 11312 during a replay; the Jetson's
spare master for tests). It only LISTENS to /tf and /tf_static for a few seconds; it
publishes nothing and changes nothing.

WHY
    TF (ROS's live record of where each part of the robot is) is a family tree of named
    frames. Every frame may have only ONE parent. If two programs both claim to be the
    parent of "odom" - for example our LiDAR odometry (odom_lidar_ref -> odom) and a
    drift-corrected frame (odom_debiased -> odom) - every position lookup flips between
    the two answers and every map built on it is quietly wrong. REPORT.md section 2.1
    item 9 found exactly this in the first plan. docs/DO_NOT_REPEAT.md records the other
    form of the same trap: two programs publishing the SAME line (odom -> base_link).

    *Plain terms: listen for five seconds to everything that says "frame A sits on frame
    B", and refuse if any frame is claimed by two parents, or one line by two programs.*

WHAT FAILS (exit 1)
    * a frame heard with two or more different parents        (the core check)
    * one parent -> child line published by two different programs
      (downgrade to a warning with --allow-shared-edges)
    * --expect-parent CHILD=PARENT given, and CHILD was not heard with exactly that one parent
    * --require-frame NAME given, and NAME was never heard
WHAT CANNOT BE JUDGED (exit 2)
    * no ROS master at ROS_MASTER_URI, or nothing at all heard on /tf and /tf_static
WHAT PASSES (exit 0)
    * everything else. The window is measured on the WALL clock, so it also works on a
      replay master with /use_sim_time true (where ROS time only moves with /clock).

  usage:  tf_one_parent_check.py [--seconds 5] [--expect-parent odom=odom_lidar_ref]
                                 [--require-frame base_link] [--json out.json]
          ROS_MASTER_URI picks the master. Before a LIVE launch on the robot, run it with
          no --expect-parent (odom must have at most one parent); after the launch, run it
          with --expect-parent odom=odom_lidar_ref.
"""
from __future__ import print_function

import argparse
import json
import os
import socket
import sys
import threading
import time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=5.0, help="how long to listen (wall clock)")
    ap.add_argument("--expect-parent", action="append", default=[], metavar="CHILD=PARENT",
                    help="CHILD must be heard with exactly this one parent (repeatable)")
    ap.add_argument("--require-frame", action="append", default=[], metavar="NAME",
                    help="NAME must be heard, as a parent or a child (repeatable)")
    ap.add_argument("--allow-shared-edges", action="store_true",
                    help="one line published by two programs is a warning, not a failure")
    ap.add_argument("--json", default="")
    a = ap.parse_args()

    expect = {}
    for e in a.expect_parent:
        if "=" not in e:
            print("--expect-parent needs CHILD=PARENT, got %r" % e)
            return 2
        c, p = e.split("=", 1)
        expect[c.strip().lstrip("/")] = p.strip().lstrip("/")

    master = os.environ.get("ROS_MASTER_URI", "(ROS_MASTER_URI not set)")
    print("tf_one_parent_check: listening to /tf and /tf_static for %.1f s on %s" % (a.seconds, master))

    import rosgraph
    socket.setdefaulttimeout(3.0)
    try:
        rosgraph.Master("/tf_one_parent_check").getPid()
    except Exception as ex:                                   # no master, or it did not answer
        print("  CANNOT JUDGE: no ROS master answering at %s (%s)" % (master, ex))
        return 2
    socket.setdefaulttimeout(None)

    import rospy
    from tf2_msgs.msg import TFMessage

    edges = {}          # (parent, child) -> {"count": n, "topics": set, "publishers": set}
    heard = {"msgs": 0}
    lock = threading.Lock()                  # callbacks run on rospy's threads

    def make_cb(topic):
        def cb(msg):
            pub = "?"
            hdr = getattr(msg, "_connection_header", None)
            if hdr:
                pub = hdr.get("callerid", "?")
            with lock:
                heard["msgs"] += 1
                for tr in msg.transforms:
                    k = (tr.header.frame_id.lstrip("/"), tr.child_frame_id.lstrip("/"))
                    e = edges.setdefault(k, {"count": 0, "topics": set(), "publishers": set()})
                    e["count"] += 1
                    e["topics"].add(topic)
                    e["publishers"].add(pub)
        return cb

    rospy.init_node("tf_one_parent_check", anonymous=True, disable_signals=True)
    subs = [rospy.Subscriber("/tf", TFMessage, make_cb("/tf"), queue_size=200),
            rospy.Subscriber("/tf_static", TFMessage, make_cb("/tf_static"), queue_size=200)]
    t_end = time.time() + a.seconds                           # WALL clock: works with /use_sim_time true
    while time.time() < t_end and not rospy.is_shutdown():
        time.sleep(0.1)
    for sub in subs:
        sub.unregister()
    with lock:
        snapshot = {k: {"count": v["count"], "topics": set(v["topics"]), "publishers": set(v["publishers"])}
                    for k, v in edges.items()}
        n_msgs = heard["msgs"]

    if n_msgs == 0:
        print("  CANNOT JUDGE: nothing heard on /tf or /tf_static in %.1f s" % a.seconds)
        rospy.signal_shutdown("done")
        return 2

    parents = {}
    for (p, c) in snapshot:
        parents.setdefault(c, set()).add(p)
    frames = set(parents) | {p for (p, _) in snapshot}

    print("  %-24s -> %-28s %-18s %8s  %s" % ("parent", "child", "topic", "msgs/s", "published by"))
    for (p, c), e in sorted(snapshot.items()):
        print("  %-24s -> %-28s %-18s %8.1f  %s" % (p, c, ",".join(sorted(e["topics"])),
                                                   e["count"] / a.seconds, ", ".join(sorted(e["publishers"]))))
    fail, warn = [], []
    for c, ps in sorted(parents.items()):
        if len(ps) > 1:
            who = "; ".join("%s (by %s)" % (p, ", ".join(sorted(snapshot[(p, c)]["publishers"]))) for p in sorted(ps))
            fail.append("frame '%s' has %d parents: %s" % (c, len(ps), who))
        if c in ps:
            fail.append("frame '%s' is its own parent" % c)
    for (p, c), e in sorted(snapshot.items()):
        if len(e["publishers"]) > 1:
            m = "the line %s -> %s is published by %d programs: %s" % (p, c, len(e["publishers"]),
                                                                        ", ".join(sorted(e["publishers"])))
            (warn if a.allow_shared_edges else fail).append(m)
    for c, p in sorted(expect.items()):
        got = sorted(parents.get(c, set()))
        if got != [p]:
            fail.append("expected '%s' to have the single parent '%s'; heard %s"
                        % (c, p, ", ".join("'%s'" % g for g in got) if got else "no parent at all"))
    for f in a.require_frame:
        if f.lstrip("/") not in frames:
            fail.append("frame '%s' was never heard" % f)

    odom_p = sorted(parents.get("odom", set()))
    print("  odom's parent(s): %s" % (", ".join(odom_p) if odom_p else "none heard (odom is a root, or absent)"))
    for w in warn:
        print("  WARNING: " + w)
    for f in fail:
        print("  FAIL: " + f)
    verdict = "FAIL" if fail else "PASS"
    print("  %s: %d frames, %d parent->child lines, %d messages in %.1f s"
          % (verdict, len(frames), len(snapshot), n_msgs, a.seconds))
    if a.json:
        with open(a.json, "w") as fh:
            json.dump({"master": master, "seconds": a.seconds, "verdict": verdict, "fail": fail, "warn": warn,
                       "odom_parents": odom_p,
                       "edges": [{"parent": p, "child": c, "msgs": e["count"], "topics": sorted(e["topics"]),
                                  "publishers": sorted(e["publishers"])} for (p, c), e in sorted(snapshot.items())]},
                      fh, indent=2)
    rospy.signal_shutdown("done")
    return 1 if fail else 0


if __name__ == "__main__":
    sys.exit(main())
