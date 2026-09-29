#!/usr/bin/env python3
"""list_folder.py - print a local copy of the Teams folder as the tables in DATA_INDEX.md.

Usage:  python3 data/list_folder.py <folder> > listing.md
One section per top-level folder, one table per sub-folder, largest files first.
Standard library only; nothing is changed, only read.
"""
import os
import sys

KINDS = [(".svo2", "ZED X camera recording (stereo pictures + camera IMU)"), (".db", "RTAB-Map database"),
         ("_lidar.bag", "the robot's LiDAR recording (raw packets)"), (".bag", "a ROS bag recording"),
         ("_gyro_zero.json", "the gyroscope's standing-still bias"), (".tum", "a path in TUM format"),
         (".png", "a figure or photo"), (".mp4", "a video"), (".py", "a script"), (".sh", "a script")]


def size(n):
    for unit, step in (("GB", 1e9), ("MB", 1e6), ("KB", 1e3)):
        if n >= step:
            return "%.2f %s" % (n / step, unit) if unit != "KB" else "%d KB" % round(n / step)
    return "%d B" % n


def what(name):
    return next((text for end, text in KINDS if name.endswith(end)), "-")


def main(root):
    for top in sorted(d for d in os.listdir(root) if os.path.isdir(os.path.join(root, d))):
        groups = {}
        for here, _, files in os.walk(os.path.join(root, top)):
            for f in files:
                path = os.path.join(here, f)
                rel = os.path.relpath(path, os.path.join(root, top))
                sub = rel.split(os.sep)[0] if os.sep in rel else "(top level)"
                groups.setdefault(sub, []).append((os.path.getsize(path), rel))
        total = sum(n for g in groups.values() for n, _ in g)
        print("## %s/\n\n%d files, %s in all.\n" % (top, sum(len(g) for g in groups.values()), size(total)))
        for sub in sorted(groups):
            items = sorted(groups[sub], reverse=True)
            print("### %s  (%d files, %s)\n" % (sub, len(items), size(sum(n for n, _ in items))))
            print("| file | size | what it contains |\n|---|---|---|")
            for n, rel in items:
                print("| `%s` | %s | %s |" % (rel, size(n), what(rel)))
            print()


if __name__ == "__main__":
    if len(sys.argv) != 2 or not os.path.isdir(sys.argv[1]):
        sys.exit("usage: python3 list_folder.py <local copy of the Teams folder>")
    main(sys.argv[1])
