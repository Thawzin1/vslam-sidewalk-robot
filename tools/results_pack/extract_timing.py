#!/usr/bin/env python3
"""extract_timing.py (copy for drive 4: the only changes are a missing cpu_jetson.csv is allowed and the
progress file is named after the run) - drive 3 (s2_static_03): when did each stream stop, and who stopped it?

Read-only inputs (Jetson internal disk):
  ~/.run_records/s2_static_03/fusion.bag      header stamp AND rosbag receive time, every message
  ~/.run_records/s2_static_03/mapping.log     rtabmap per-iteration timings ("Maps update=...")
                                              and rgbd_odometry per-frame lines
  ~/.run_records/s2_static_03/cpu_jetson.csv  a 1-second sampler (its own write times)
  ~/jobs/s2_static_03_bridge_clock.csv        bridge pings: Jetson send t1, robot t2/t3, Jetson receive t4
Writes: timing.npz beside this script.

Plain terms: every message carries two clocks - the time it was measured (header stamp) and the
time the recorder wrote it down (receive time). A gap in the first means nothing was measured or
it was thrown away; a gap in the second only means the recorder was not writing. Comparing the
two, stream by stream, tells a stalled computer apart from a dropped radio link.
"""
import os, re, sys, time
import numpy as np
import rosbag

RUN = sys.argv[1] if len(sys.argv) > 1 else "s2_static_03"   # e.g. the drive-4 rehearsal run name
REC = os.path.join(os.path.expanduser(os.environ.get("RECORDS_DIR", "~/.run_records")), RUN)
OUT = sys.argv[2] if len(sys.argv) > 2 else os.path.join(os.path.dirname(os.path.abspath(__file__)), "timing.npz")
PROG = os.path.expanduser("~/jobs/%s_timing_extract.progress" % RUN)

topics = ["/zedx_front/zed_node/imu/data", "/rtabmap/odom", "/robot/wheel_odom", "/robot/imu",
          "/robot/ekf_odom", "/fused/odometry", "/tf", "/diagnostics", "/robot_bridge/status",
          "/ekf_in/vo_odom"]
t0 = time.time()
data = {t: [] for t in topics}
zdiag = []   # 'ZED Diagnostic' is updated only inside the camera node's picture loop
bag = rosbag.Bag(os.path.join(REC, "fusion.bag"))
info = bag.get_type_and_topic_info().topics
TOTAL = sum(info[t].message_count for t in topics if t in info)
n = 0
for topic, m, tb in bag.read_messages(topics=topics):
    n += 1
    if n % 50000 == 0:
        with open(PROG, "w") as f:
            f.write("EXTRACT %s %d/%d  %ds\n" % (RUN, n, TOTAL, time.time() - t0))
    if topic == "/diagnostics":
        for stt in m.status:
            if stt.name.endswith("ZED Diagnostic"):
                zdiag.append(m.header.stamp.to_sec())
    if topic == "/tf":
        st = m.transforms[0].header.stamp.to_sec() if m.transforms else np.nan
    else:
        st = m.header.stamp.to_sec()
    data[topic].append((st, tb.to_sec()))
bag.close()

rit = re.compile(r"\[(\d+\.\d+)\]: rtabmap \((\d+)\): Rate=[\d.]+s, Limit=[\d.]+s, Conversion=([\d.]+)s, "
                 r"RTAB-Map=([\d.]+)s, Maps update=([\d.]+)s pub=([\d.]+)s")
rod = re.compile(r"\[(\d+\.\d+)\]: Odom: quality=(\d+),.*update time=([\d.]+)s, delay=([\d.]+)s")
its, odl = [], []
bad_lines = 0
with open(os.path.join(REC, "mapping.log"), errors="replace") as f:
    for line in f:
        line = re.sub(r"\x1b\[[0-9;]*m", "", line)
        mm = rit.search(line)
        try:   # drive 6: a log line garbled by two writers at once ('0.22411.00') - skip and count it
            if mm:
                its.append(tuple(float(x) for x in mm.groups()))
                continue
            mm = rod.search(line)
            if mm:
                odl.append(tuple(float(x) for x in mm.groups()))
        except ValueError:
            bad_lines += 1

_cpu = os.path.join(REC, "cpu_jetson.csv")   # drive 4 had no processor sampler: tolerate its absence
cpu_t = np.unique(np.loadtxt(_cpu, delimiter=",", skiprows=1, usecols=0)) if os.path.exists(_cpu) else np.array([])
clk = []
with open(os.path.expanduser("~/jobs/%s_bridge_clock.csv" % RUN)) as f:
    next(f)
    for line in f:
        p = line.split(",")
        try:   # drive 9 (power-cycled): the file ends in NUL bytes - skip unreadable lines, counted with the garbled ones
            clk.append(tuple(int(x) / 1e9 for x in p[:4]))
        except ValueError:
            bad_lines += 1

arr = {k.strip("/").replace("/", "_"): np.array(v) for k, v in data.items()}
np.savez_compressed(OUT, rtabmap_iter=np.array(its), odom_log=np.array(odl), cpu_sampler_t=cpu_t,
                    bridge_clock=np.array(clk), zed_diag_t=np.array(zdiag), **arr)
with open(PROG, "w") as f:
    f.write("EXTRACT %s %d/%d  %ds done\n" % (RUN, n, TOTAL, time.time() - t0))
print({k: v.shape for k, v in arr.items()}, "iters", len(its), "odom lines", len(odl),
      "cpu samples", len(cpu_t), "pings", len(clk), "garbled log lines skipped", bad_lines)
