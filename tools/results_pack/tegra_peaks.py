#!/usr/bin/env python3
"""tegra_peaks.py - power, temperature, memory and load peaks from a tegrastats log, over a time window.
tegrastats writes one line a second with its clock in UTC (MM-DD-YYYY HH:MM:SS); windows are given in Hamilton
time and converted here. Total power = VDD_GPU_SOC + VDD_CPU_CV + VIN_SYS_5V0 + VDDQ_VDD2_1V8AO (the four named rails, NC ones are 0;
VDDQ_VDD2_1V8AO too, as in the T5b score, which this reproduces: 35.4 W at 23:51:02). Instantaneous values (the first of each rail's now/average pair).
usage: tegra_peaks.py <log> "<YYYY-MM-DD HH:MM:SS>" "<YYYY-MM-DD HH:MM:SS>"  (both Hamilton)"""
import re, sys, json, datetime
import os, time
import numpy as np
os.environ["TZ"] = "America/Toronto"; time.tzset()
log, a, b = sys.argv[1:4]     # "YYYY-MM-DD HH:MM:SS" Hamilton, start and end
w0 = time.mktime(time.strptime(a, "%Y-%m-%d %H:%M:%S"))
w1 = time.mktime(time.strptime(b, "%Y-%m-%d %H:%M:%S"))
R = []
prev = None
for L in open(log):
    m = re.match(r"(\d\d)-(\d\d)-(\d{4}) (\d\d:\d\d:\d\d) RAM (\d+)/(\d+)MB", L)
    if not m:
        continue
    t = datetime.datetime.strptime("%s-%s-%s %s" % (m[3], m[1], m[2], m[4]), "%Y-%m-%d %H:%M:%S").replace(
        tzinfo=datetime.timezone.utc).timestamp()
    if not (w0 <= t <= w1):
        continue
    rail = lambda n: int(re.search(n + r" (\d+)mW/", L).group(1)) / 1000.0
    cpu = [int(x) for x in re.findall(r"(\d+)%@\d+", L.split("CPU [")[1].split("]")[0])]
    gpu = re.search(r"GR3D_FREQ (\d+)%", L)
    R.append(dict(t=t, ram_used=int(m[5]), ram_tot=int(m[6]),
                  W=rail("VDD_GPU_SOC") + rail("VDD_CPU_CV") + rail("VIN_SYS_5V0") + rail("VDDQ_VDD2_1V8AO"),
                  tj=float(re.search(r"tj@([\d.]+)C", L).group(1)),
                  cpu_t=float(re.search(r"CPU@([\d.]+)C", L).group(1)),
                  gpu_t=float(re.search(r"GPU@(-?[\d.]+)C", L).group(1)),
                  cpu_mean=float(np.mean(cpu)), gpu=int(gpu.group(1)) if gpu else 0, gap=(t - prev) if prev else 0))
    prev = t
hm = lambda t: time.strftime("%H:%M:%S", time.localtime(t))
def pk(k, f=max):
    r = f(R, key=lambda x: x[k]); return {"value": round(r[k], 1), "at_hamilton": hm(r["t"])}
out = {"log": log, "window_hamilton": [a, b], "samples": len(R),
       "power_total_W": dict(max=pk("W"), median=round(float(np.median([r["W"] for r in R])), 1)),
       "tj_C_max": pk("tj"), "cpu_temp_C_max": pk("cpu_t"), "gpu_temp_C_max": pk("gpu_t"),
       "gpu_load_pct": dict(max=pk("gpu"), median=float(np.median([r["gpu"] for r in R]))),
       "cpu_mean_of_12_cores_pct": dict(max=pk("cpu_mean"), median=round(float(np.median([r["cpu_mean"] for r in R])), 1)),
       "tegrastats_skips_over_2s": [(hm(r["t"]), r["gap"]) for r in R if r["gap"] > 2]}
lo = min(R, key=lambda r: r["ram_tot"] - r["ram_used"])
out["ram_free_pct_lowest"] = {"value": round(100.0 * (lo["ram_tot"] - lo["ram_used"]) / lo["ram_tot"], 1),
                              "used_GB": round(lo["ram_used"] / 1024, 1), "at_hamilton": hm(lo["t"])}
print(json.dumps(out, indent=1))
