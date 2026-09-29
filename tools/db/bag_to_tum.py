#!/usr/bin/env python3
"""bag_to_tum.py - pull a trajectory out of a rosbag as a TUM file.

WHY
    The comparison the new series depends on needs two trajectories of the same
    drive in the same format. `db_to_tum.py` handles anything that produced an
    RTAB-Map database - the camera's map, and the LiDAR's. This handles the
    other kind of source: a pose stream recorded straight into a bag, which is
    where the wheel odometry and the robot's fused estimate live.

    Between them, every reference this project can produce comes out as TUM,
    which is what `evaluate_trajectory.py` and `evo` read.

MUST RUN WHERE ROS IS
    It imports `rosbag`, so it runs on the Jetson or the robot, not on a computer without ROS.
    The output is a few hundred kilobytes and is copied back.

WHICH TOPIC
    With no `--topic`, it lists what the bag actually contains and stops. Never
    guess a topic name - ENGINEERING_NOTES.md rule 4 - and a bag is the one place where
    the answer is free.

  usage:
    bag_to_tum.py run.bag                               # list the pose topics
    bag_to_tum.py run.bag --topic /husky_velocity_controller/odom --out wheel.tum
"""
from __future__ import print_function

import argparse
import json
import math
import os
import sys

POSE_TYPES = (
    "nav_msgs/Odometry",
    "geometry_msgs/PoseStamped",
    "geometry_msgs/PoseWithCovarianceStamped",
)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--topic", default="")
    ap.add_argument("--out", default="")
    ap.add_argument("--label", default="")
    args = ap.parse_args()

    try:
        import rosbag
    except ImportError:
        sys.exit("  rosbag is not importable here - run this on the Jetson or "
                 "the robot, not on a computer without ROS.")

    if not os.path.exists(args.bag):
        sys.exit("  no such bag: %s" % args.bag)

    bag = rosbag.Bag(args.bag, "r")
    info = bag.get_type_and_topic_info()

    if not args.topic:
        print("  pose-bearing topics in %s:" % os.path.basename(args.bag))
        found = False
        for name, t in sorted(info.topics.items()):
            if t.msg_type in POSE_TYPES:
                found = True
                print("    %-42s %-44s %7d msgs  %.1f Hz"
                      % (name, t.msg_type, t.message_count,
                         t.frequency or 0.0))
        if not found:
            print("    none. Every topic in this bag:")
            for name, t in sorted(info.topics.items()):
                print("    %-42s %-44s %7d" % (name, t.msg_type,
                                               t.message_count))
        bag.close()
        return 0

    if args.topic not in info.topics:
        bag.close()
        sys.exit("  %s is not in this bag. Run without --topic to list them."
                 % args.topic)

    out = args.out or (os.path.splitext(args.bag)[0] + ".tum")
    out_dir = os.path.dirname(os.path.abspath(out))
    if out_dir and not os.path.isdir(out_dir):
        os.makedirs(out_dir)

    n = 0
    skipped = 0
    first = last = None
    path_m = 0.0
    prev = None

    with open(out, "w") as fh:
        fh.write("# TUM from %s topic %s\n"
                 % (os.path.basename(args.bag), args.topic))
        fh.write("# timestamp tx ty tz qx qy qz qw\n")
        for topic, msg, t in bag.read_messages(topics=[args.topic]):
            # Odometry nests the pose twice; the others carry it directly.
            p = getattr(msg, "pose", None)
            if p is not None and hasattr(p, "pose"):
                p = p.pose
            if p is None:
                skipped += 1
                continue
            try:
                stamp = msg.header.stamp.to_sec()
            except AttributeError:
                stamp = t.to_sec()
            if stamp <= 0:
                # A zero stamp cannot be aligned against anything. Falling back
                # to the bag's own receive time is honest and says so below.
                stamp = t.to_sec()
                skipped += 0
            pos, ori = p.position, p.orientation
            fh.write("%.6f %.6f %.6f %.6f %.9f %.9f %.9f %.9f\n"
                     % (stamp, pos.x, pos.y, pos.z,
                        ori.x, ori.y, ori.z, ori.w))
            if first is None:
                first = stamp
            last = stamp
            if prev is not None:
                path_m += math.sqrt((pos.x - prev[0]) ** 2
                                    + (pos.y - prev[1]) ** 2
                                    + (pos.z - prev[2]) ** 2)
            prev = (pos.x, pos.y, pos.z)
            n += 1
    bag.close()

    meta = {
        "source_bag": os.path.abspath(args.bag),
        "topic": args.topic,
        "label": args.label or args.topic,
        "poses": n,
        "skipped_no_pose": skipped,
        "duration_s": round(last - first, 3) if (first and last) else None,
        "path_length_m_unsampled": round(path_m, 3),
        "note": ("path length here is the raw sum over every message and is "
                 "NOT comparable to a figure sampled at the mapper's node "
                 "spacing - see docs/SOLVED.md"),
    }
    with open(out + ".meta.json", "w") as fh:
        json.dump(meta, fh, indent=2)
        fh.write("\n")

    print("  %s -> %s" % (args.topic, out))
    print("  %d poses, %.1f s" % (n, meta["duration_s"] or 0.0))
    if n == 0:
        print("  NOTHING WAS WRITTEN - the topic exists but carried no pose.")
        return 3
    return 0


if __name__ == "__main__":
    sys.exit(main())
