#!/usr/bin/env python3
"""check_goals.py MAP.yaml GOALS.yaml [...] - is every goal on seen-free floor, with room to turn on the spot?

For each goal: the saved map's cell there (must be free, white), and the clearance = distance to the nearest
wall cell. Turning on the spot needs the robot's outer radius + padding = sqrt(0.495^2 + 0.335^2) + 0.05 =
0.648 m. Exit 1 if any goal fails.
*Plain terms: checks that no goal is inside a wall, in unexplored space, or somewhere too tight to turn round.*
"""
import math
import os
import sys

import numpy as np
import yaml
from scipy.ndimage import distance_transform_edt

NEED = math.hypot(0.495, 0.335) + 0.05


def load(yaml_path):
    m = yaml.safe_load(open(yaml_path))
    with open(os.path.join(os.path.dirname(yaml_path), m["image"]), "rb") as f:
        f.readline()
        line = f.readline()
        while line.startswith(b"#"):
            line = f.readline()
        w, h = [int(t) for t in line.split()]
        f.readline()
        img = np.flipud(np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w))
    return img, float(m["resolution"]), float(m["origin"][0]), float(m["origin"][1])


def main():
    img, res, ox, oy = load(sys.argv[1])
    clear = distance_transform_edt(img != 0) * res
    bad = 0
    for gf in sys.argv[2:]:
        print(gf)
        for g in yaml.safe_load(open(gf))["goals"]:
            r, c = int((g["y"] - oy) / res), int((g["x"] - ox) / res)
            cell = {254: "free", 0: "WALL", 205: "UNSEEN"}.get(int(img[r, c]), str(img[r, c]))
            ok = cell == "free" and clear[r, c] >= NEED
            bad += not ok
            print("  %-18s (%6.2f, %6.2f) yaw %4.0f  cell %-6s clearance %.2f m (need %.2f)  %s" % (
                g["name"], g["x"], g["y"], g.get("yaw_deg", 0), cell, clear[r, c], NEED, "ok" if ok else "FAIL"))
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
