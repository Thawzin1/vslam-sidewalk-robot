#!/usr/bin/env python3
"""cpu_sampler.py - how many processor cores does each chosen process use, second by second?

RUNS ON the computer whose processes are measured (the robot during the full-speed
replay that gates live use; the Jetson for tests). Python 3.8, standard library only.
Reads /proc; changes nothing; never stops or signals any process.

WHY
    The live LiDAR map may run on the robot's own computer, which also drives the wheels.
    REPORT.md section 4.3 gates live use on the whole pipeline (decoder, body filter,
    scan matching, map) using at most 2 cores in a typical second (median) and at most
    4 cores in its busiest seconds (99th percentile: the value 99 % of seconds stay under).
    "ps %cpu" cannot show this: it is an average over the process's whole life (man ps),
    so a 3-second burst during a loop-closure check vanishes into it.

    So: every second, read each process's own CPU-time counter from /proc/<pid>/stat
    (fields 14 utime and 15 stime: time spent running its code and the kernel's on its
    behalf, all threads included) and divide the increase by the wall time that passed.
    1.00 = one core busy for the whole second.

    *Plain terms: a stopwatch on each program, read once a second, so short bursts show.*

WHICH PROCESSES (all matching is by EXACT, WHOLE values - ENGINEERING_NOTES.md rule 8 - never by
"contains"; this program's own PID and its parent's are always skipped)
    --pid N              that process (repeatable)
    --node NAME          processes started with the whole argument "__name:=NAME", which is
                         how roslaunch names every node it starts (repeatable)
    --master-uri URI     every process whose environment holds exactly the line
                         ROS_MASTER_URI=URI, i.e. everything started for that ROS master
    --exclude-comm NAME  drop processes whose kernel name (/proc/<pid>/comm) is exactly NAME,
                         e.g. rosmaster, rosout (repeatable)
    Processes are chosen once, at the start (add --rescan S to look for new ones every S s).
    A process that exits is reported as exited at that second; a PID reused by a new
    program is detected (its start time changes) and not mixed with the old one.

OUTPUT
    <out>.csv    one row per second per process: wall time, pid, label, cores
    <out>.json   per process and for the TOTAL: median, 99th percentile, max, seconds
                 sampled; the gate verdict (--gate-median / --gate-p99, defaults 2 and 4)
    a .progress line (--progress FILE) rewritten every 10 s for the jobs page (port 8096)

  usage:  cpu_sampler.py --master-uri http://localhost:11312 --exclude-comm rosmaster \\
              --exclude-comm rosout --seconds 1500 --out /path/cpu --progress ~/jobs/x_cpu.progress
          cpu_sampler.py --node icp_odometry --node rtabmap --seconds 60 --out /tmp/cpu
          stops at --seconds, at Ctrl-C / SIGTERM, or when every sampled process has exited,
          and always writes the summary.
  exit:   0 = sampled and within the gate; 1 = sampled and over the gate;
          2 = nothing matched, or nothing could be sampled
"""
from __future__ import print_function

import argparse
import csv
import json
import os
import signal
import sys
import time

HZ = os.sysconf("SC_CLK_TCK")          # kernel clock ticks per second, the unit of utime/stime


def read_stat(pid):
    """(utime+stime ticks, start time ticks, comm) or None if the process is gone."""
    try:
        with open("/proc/%d/stat" % pid) as fh:
            s = fh.read()
    except (IOError, OSError):
        return None
    # the name sits in brackets and may itself contain spaces or brackets: split after the LAST ')'
    comm = s[s.index("(") + 1:s.rindex(")")]
    f = s[s.rindex(")") + 2:].split()
    # f[0] is field 3 (state); utime = field 14 -> f[11], stime = field 15 -> f[12], starttime = field 22 -> f[19]
    return int(f[11]) + int(f[12]), int(f[19]), comm


def argv(pid):
    try:
        with open("/proc/%d/cmdline" % pid, "rb") as fh:
            return [x.decode("utf-8", "replace") for x in fh.read().split(b"\0") if x]
    except (IOError, OSError):
        return []


def environ_lines(pid):
    try:
        with open("/proc/%d/environ" % pid, "rb") as fh:
            return [x.decode("utf-8", "replace") for x in fh.read().split(b"\0") if x]
    except (IOError, OSError):
        return []                        # another user's process: its environment is not readable


def label_of(pid, comm):
    for x in argv(pid):
        if x.startswith("__name:="):
            return x[len("__name:="):]
    args = argv(pid)
    if comm.startswith("python") and len(args) > 1:
        return os.path.basename(args[1])           # a Python script: name it by the script, not "python3"
    return comm


def choose(a, skip):
    """PIDs matching the selection, by exact whole values only."""
    chosen = set()
    for pid in a.pid:
        if pid in skip:
            continue
        if read_stat(pid) is not None:
            chosen.add(pid)
        else:
            print("  --pid %d: no such process" % pid)
    if a.node or a.master_uri:
        want_args = {"__name:=" + n for n in a.node}
        want_env = "ROS_MASTER_URI=" + a.master_uri if a.master_uri else None
        for d in os.listdir("/proc"):
            if not d.isdigit():
                continue
            pid = int(d)
            if pid in skip:
                continue
            if want_args and want_args.intersection(argv(pid)):       # whole-argument equality
                chosen.add(pid)
            elif want_env and want_env in environ_lines(pid):          # whole-line equality
                chosen.add(pid)
    out = set()
    for pid in chosen:
        st = read_stat(pid)
        if st is None:
            continue
        if st[2] in a.exclude_comm:                                     # whole-name equality
            continue
        out.add(pid)
    return out


def pct(v, p):
    """p-th percentile, linear interpolation (numpy's default), no numpy needed."""
    if not v:
        return None
    s = sorted(v)
    k = (len(s) - 1) * p / 100.0
    lo = int(k)
    hi = min(lo + 1, len(s) - 1)
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pid", type=int, action="append", default=[])
    ap.add_argument("--node", action="append", default=[])
    ap.add_argument("--master-uri", default="")
    ap.add_argument("--exclude-comm", action="append", default=[])
    ap.add_argument("--seconds", type=float, default=0.0, help="stop after this long (0 = until stopped)")
    ap.add_argument("--interval", type=float, default=1.0)
    ap.add_argument("--rescan", type=float, default=0.0, help="look for new matching processes every S s")
    ap.add_argument("--out", required=True, help="output path WITHOUT extension (.csv and .json are added)")
    ap.add_argument("--progress", default="")
    ap.add_argument("--label", default="CPU_SAMPLER")
    ap.add_argument("--gate-median", type=float, default=2.0, help="cores, whole pipeline (REPORT.md 4.3)")
    ap.add_argument("--gate-p99", type=float, default=4.0, help="cores, whole pipeline (REPORT.md 4.3)")
    a = ap.parse_args()
    if not (a.pid or a.node or a.master_uri):
        print("choose processes with --pid, --node or --master-uri")
        return 2

    skip = {os.getpid(), os.getppid()}
    pids = choose(a, skip)
    t_start = time.time()

    def progress(line):
        if a.progress:
            tmp = a.progress + ".tmp"
            with open(tmp, "w") as fh:
                fh.write(line + "\n")
            os.replace(tmp, a.progress)

    if not pids:
        print("  nothing matched: no process to sample")
        progress("%s  FAILED - nothing matched the selection" % a.label)
        return 2

    info = {}                  # pid -> {"label", "start", "last_ticks", "samples": [], "exited_at": None}
    for pid in pids:
        st = read_stat(pid)
        if st:
            info[pid] = {"label": label_of(pid, st[2]), "comm": st[2], "start": st[1],
                         "last_ticks": st[0], "samples": [], "exited_at": None}
    t_last = time.monotonic()          # the counters above were read at this moment
    print("  sampling %d processes every %.1f s:" % (len(info), a.interval))
    for pid, v in sorted(info.items()):
        print("    pid %-7d %-16s %s" % (pid, v["comm"], v["label"]))

    stop = {"now": False}

    def on_signal(signum, frame):
        stop["now"] = True
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    totals = []
    fh = open(a.out + ".csv", "w")
    w = csv.writer(fh)
    w.writerow(["wall_time", "seconds_from_start", "pid", "label", "cores"])
    t0 = t_last
    n = 0
    next_prog = 0.0
    next_rescan = a.rescan if a.rescan > 0 else float("inf")
    while not stop["now"]:
        n += 1
        target = t0 + n * a.interval
        while not stop["now"]:
            d = target - time.monotonic()
            if d <= 0:
                break
            time.sleep(min(d, 0.2))
        if stop["now"]:
            break
        now = time.monotonic()
        dt = now - t_last
        t_last = now
        wall = time.time()
        total = 0.0
        alive = 0
        for pid, v in sorted(info.items()):
            if v["exited_at"] is not None:
                continue
            st = read_stat(pid)
            if st is None or st[1] != v["start"]:          # gone, or the PID now belongs to a new program
                v["exited_at"] = round(wall - t_start, 1)
                print("  pid %d (%s) exited at %.0f s" % (pid, v["label"], wall - t_start))
                continue
            cores = (st[0] - v["last_ticks"]) / float(HZ) / dt
            v["last_ticks"] = st[0]
            v["samples"].append(cores)
            total += cores
            alive += 1
            w.writerow(["%.3f" % wall, "%.1f" % (wall - t_start), pid, v["label"], "%.3f" % cores])
        if alive == 0:
            print("  every sampled process has exited")
            break
        totals.append(total)
        el = now - t0
        if el >= next_prog:
            next_prog = el + 10.0
            fh.flush()
            nm = ("%d/%d" % (int(el), int(a.seconds))) if a.seconds > 0 else "%d" % int(el)
            progress("%s  %s s  %d processes, now %.2f cores, median %.2f, p99 %.2f  %ds"
                     % (a.label, nm, alive, total, pct(totals, 50), pct(totals, 99), int(el)))
        if el >= next_rescan:
            next_rescan = el + a.rescan
            for pid in choose(a, skip) - set(info):
                st = read_stat(pid)
                if st:
                    info[pid] = {"label": label_of(pid, st[2]), "comm": st[2], "start": st[1],
                                 "last_ticks": st[0], "samples": [], "exited_at": None}
                    print("  added pid %d (%s) at %.0f s" % (pid, info[pid]["label"], el))
        if a.seconds > 0 and el >= a.seconds:
            break
    fh.close()

    def summ(v):
        return {"seconds": len(v), "median_cores": None if not v else round(pct(v, 50), 3),
                "p99_cores": None if not v else round(pct(v, 99), 3),
                "max_cores": None if not v else round(max(v), 3)}

    per = {str(pid): dict(summ(v["samples"]), label=v["label"], comm=v["comm"], exited_at_s=v["exited_at"])
           for pid, v in sorted(info.items())}
    tot = summ(totals)
    if not totals:
        verdict = "NOT MEASURED"
    else:
        verdict = "PASS" if (tot["median_cores"] <= a.gate_median and tot["p99_cores"] <= a.gate_p99) else "FAIL"
    res = {"started": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(t_start)),
           "interval_s": a.interval, "clock_ticks_per_s": HZ,
           "selection": {"pid": a.pid, "node": a.node, "master_uri": a.master_uri, "exclude_comm": a.exclude_comm},
           "total": tot, "per_process": per,
           "gate": {"median_cores_max": a.gate_median, "p99_cores_max": a.gate_p99, "verdict": verdict,
                    "means": "PASS: the chosen processes together stay light enough in typical and busiest "
                             "seconds; FAIL: they would compete with the robot's own driving programs"}}
    with open(a.out + ".json", "w") as jf:
        json.dump(res, jf, indent=2)
    print("  %-26s %8s %8s %8s %8s" % ("process", "seconds", "median", "p99", "max"))
    for pid, v in sorted(per.items(), key=lambda kv: -(kv[1]["median_cores"] or 0)):
        print("  %-26s %8d %8s %8s %8s%s" % (v["label"][:26], v["seconds"], v["median_cores"], v["p99_cores"],
                                            v["max_cores"], "" if v["exited_at_s"] is None
                                            else "  (exited at %.0f s)" % v["exited_at_s"]))
    print("  %-26s %8d %8s %8s %8s" % ("TOTAL", tot["seconds"], tot["median_cores"], tot["p99_cores"], tot["max_cores"]))
    print("  gate (median <= %.1f, p99 <= %.1f cores): %s" % (a.gate_median, a.gate_p99, verdict))
    progress("%s  complete  %d s sampled, total median %s cores, p99 %s: %s"
             % (a.label, tot["seconds"], tot["median_cores"], tot["p99_cores"], verdict))
    if verdict == "NOT MEASURED":
        return 2
    return 0 if verdict == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
