#!/usr/bin/env python3
"""tegra_window.py LOG T0 T1 [--label X] - summarise a tegrastats log between two times (epoch s, Jetson clock).

tegrastats = NVIDIA's once-a-second log of processor load, graphics-chip (GPU) load, memory and power.
Prints one JSON line: GPU busy % (mean, 95th percentile, max), all-12-cores CPU busy % (mean of the per-core
mean), memory used, and the power rail readings. Read-only.

*Plain terms: how hard the Jetson worked during one stretch of a test, as averages with their spread.*
"""
import json, re, sys, time, datetime
import numpy as np


def main():
    log, t0, t1 = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
    label = sys.argv[sys.argv.index("--label") + 1] if "--label" in sys.argv else ""
    gpu, cpu, ram, pgpu, pcpu = [], [], [], [], []
    for line in open(log, errors="replace"):
        m = re.match(r"(\d\d-\d\d-\d{4} \d\d:\d\d:\d\d) ", line)
        if not m:
            continue
        t = time.mktime(datetime.datetime.strptime(m.group(1), "%m-%d-%Y %H:%M:%S").timetuple())
        if not (t0 <= t <= t1):
            continue
        g = re.search(r"GR3D_FREQ (\d+)%", line)
        c = re.search(r"CPU \[([^\]]+)\]", line)
        r = re.search(r"RAM (\d+)/", line)
        vg = re.search(r"VDD_GPU_SOC (\d+)mW", line)
        vc = re.search(r"VDD_CPU_CV (\d+)mW", line)
        if g:
            gpu.append(int(g.group(1)))
        if c:
            vals = [int(x.split("%")[0]) for x in c.group(1).split(",") if "%" in x]
            if vals:
                cpu.append(float(np.mean(vals)))
        if r:
            ram.append(int(r.group(1)))
        if vg:
            pgpu.append(int(vg.group(1)))
        if vc:
            pcpu.append(int(vc.group(1)))

    def s(v, nd=1):
        if not v:
            return None
        a = np.asarray(v, float)
        return {"mean": round(float(a.mean()), nd), "p95": round(float(np.percentile(a, 95)), nd),
                "max": round(float(a.max()), nd), "n": len(v)}
    print(json.dumps({"label": label, "gpu_busy_pct": s(gpu), "cpu_busy_pct_all_cores": s(cpu),
                      "ram_used_mb": s(ram, 0), "gpu_soc_power_mw": s(pgpu, 0), "cpu_cv_power_mw": s(pcpu, 0)}))


if __name__ == "__main__":
    main()
