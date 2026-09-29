#!/usr/bin/env python3
"""plot_safety.py <safety_dir> - one small panel per stop test: the simulated Husky's TRUE speed around the trigger.
The trigger time is recovered as (first moment below 0.02 m/s after steady driving) - (measured stop time).
Writes <safety_dir>/safety_stops.png."""
import csv
import json
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

d = sys.argv[1]
res = json.load(open(os.path.join(d, "safety_tests.json")))
rows = list(csv.DictReader(open(os.path.join(d, "safety_speed.csv"))))
n = len(res)
cols = 4
fig, axs = plt.subplots((n + cols - 1) // cols, cols, figsize=(14, 3.2 * ((n + cols - 1) // cols)), squeeze=False)
for ax, r in zip(axs.flat, res):
    seg = [(float(x["t"]), float(x["v"])) for x in rows if x["test"] == r["test"].split(",")[0]]
    if not seg or r.get("stop_time_s") is None:
        ax.set_title(r["test"] + "\n(no stop measured)", fontsize=9)
        continue
    t, v = np.array(seg).T
    if "t_trigger" in r:                     # recorded by safety_tests.py (from 27 Sept run 7 on)
        t_trig = r["t_trigger"]
    else:                                     # run 6: the stop = first moment below 0.02 m/s that LASTS 1 s
        t_stop = t[-1]
        for i in range(len(v)):
            if v[i] < 0.02 and np.all(v[(t >= t[i]) & (t <= t[i] + 1.0)] < 0.02):
                t_stop = t[i]
                break
        t_trig = t_stop - r["stop_time_s"]
    ax.plot(t - t_trig, v, "-", color="k", lw=1.2)
    ax.axvline(0, color="tab:red", lw=1.5)
    ax.axvline(r["stop_time_s"], color="tab:blue", ls="--", lw=1.2)
    ax.set_xlim(-3, 6)
    ax.set_ylim(-0.02, 0.4)
    ax.set_title("%s\nstopped in %.2f s, rolled %.2f m%s" % (r["test"], r["stop_time_s"], r["distance_after_trigger_m"],
                 "" if r.get("resumed_after_release") else ", did NOT resume"), fontsize=8.5)
    ax.set_xlabel("s from the trigger (red line)", fontsize=8)
    ax.set_ylabel("true speed (m/s)", fontsize=8)
    ax.grid(alpha=0.3)
for ax in list(axs.flat)[n:]:
    ax.axis("off")
fig.suptitle("Simulator: every stop path stops the Husky (true speed from Gazebo; red = trigger, blue dashed = "
             "speed below 0.02 m/s)", fontsize=10)
fig.tight_layout()
fig.savefig(os.path.join(d, "safety_stops.png"), dpi=115)
print("wrote", os.path.join(d, "safety_stops.png"))
