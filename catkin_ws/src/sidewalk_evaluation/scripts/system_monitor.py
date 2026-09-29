#!/usr/bin/env python3
"""
system_monitor.py — What the accuracy actually costs.

WHY A RESEARCH RESULT NEEDS THIS
    "RTAB-Map was 20% more accurate than ORB-SLAM3" is half a sentence. On a
    battery-powered sidewalk robot the other half is what it cost: how much CPU,
    how much GPU, how much memory, how much delay between the shutter closing
    and a pose being usable, and how many camera frames were dropped on the
    floor because the pipeline could not keep up.

    A system that is slightly more accurate while saturating an AGX Orin is not
    obviously the better choice — it leaves nothing for perception, planning or
    control, and it will thermally throttle on a hot day, at which point its
    accuracy advantage evaporates precisely when you needed it.

WHAT IT MEASURES
    CPU        aggregate utilisation from /proc/stat, plus per-core, plus the
               load average. Aggregate alone is misleading on a 12-core Orin: a
               single-threaded bottleneck shows as 8% total while one core is
               pinned at 100%, so the busiest-core figure is reported too.
    GPU        from the Jetson's sysfs load node where present. There is no
               portable way to do this, so several known paths are tried and
               the metric is reported as unavailable rather than guessed if
               none exist.
    Memory     process resident set and system-wide usage. The Orin has 29 GB
               shared between CPU and GPU — there is no separate VRAM, so a
               large GPU allocation shows up as ordinary RAM pressure.
    Thermal    the Orin's thermal zones. Included because sustained throttling
               is the most common cause of a run whose second half performs
               worse than its first, and it is invisible unless you look.
    Latency    optional, if a topic pair is given: header stamp versus arrival
               time. This is the age of the information the planner steers on.
    Drops      inferred from gaps in the sequence of capture timestamps.

RUNS WITH OR WITHOUT ROS
    Without ROS it samples the machine only. With ROS (`--topic`) it also
    measures latency and dropped frames on a live stream. Missing pieces are
    reported as unavailable; none of them abort the run.
"""
from __future__ import annotations

import argparse
import os
import re
import sys
import time
from pathlib import Path

from eval_common import (MissingDependency, RunLogger, require_numpy,
                         timestamp_slug, write_json)

# Places the GPU load percentage has lived across L4T releases and Orin
# variants. Read the first one that exists. Values are 0..1000 (per mille).
GPU_LOAD_PATHS = [
    "/sys/devices/platform/gpu.0/load",
    "/sys/devices/gpu.0/load",
    "/sys/devices/platform/17000000.ga10b/load",
    "/sys/devices/platform/17000000.gpu/load",
    "/sys/class/devfreq/17000000.ga10b/device/load",
]


class CpuSampler:
    """Aggregate and per-core CPU utilisation from /proc/stat.

    Utilisation is a rate, not a state, so it can only be obtained by
    differencing two readings. The first sample after construction is therefore
    meaningless and is discarded — a monitor that reports 100% for its first
    second, as several naive implementations do, corrupts the mean of a short
    run.
    """

    def __init__(self):
        self.prev = self._read()

    @staticmethod
    def _read():
        out = {}
        try:
            with open("/proc/stat") as fh:
                for line in fh:
                    if not line.startswith("cpu"):
                        continue
                    parts = line.split()
                    key = parts[0]
                    vals = [int(v) for v in parts[1:]]
                    idle = vals[3] + (vals[4] if len(vals) > 4 else 0)
                    out[key] = (sum(vals), idle)
        except OSError:
            return {}
        return out

    def sample(self):
        cur = self._read()
        agg, per_core = None, []
        for key, (total, idle) in cur.items():
            if key not in self.prev:
                continue
            dt_total = total - self.prev[key][0]
            dt_idle = idle - self.prev[key][1]
            if dt_total <= 0:
                continue
            pct = 100.0 * (dt_total - dt_idle) / dt_total
            if key == "cpu":
                agg = pct
            else:
                per_core.append(pct)
        self.prev = cur
        return agg, per_core


def read_gpu_pct():
    """GPU utilisation as a percentage, or None if this machine cannot say.

    None is a legitimate answer and is preserved all the way into the report.
    Substituting 0 would be a lie that makes a GPU-bound configuration look
    free.
    """
    for p in GPU_LOAD_PATHS:
        try:
            raw = Path(p).read_text().strip()
            val = float(raw)
            # The node is documented as per mille (0-1000) but some releases
            # expose 0-100. Disambiguate by magnitude.
            return val / 10.0 if val > 100.0 else val
        except (OSError, ValueError):
            continue
    return None


def read_mem():
    """System memory in MB: (used, total, percent used)."""
    try:
        info = {}
        with open("/proc/meminfo") as fh:
            for line in fh:
                k, _, v = line.partition(":")
                m = re.search(r"(\d+)", v)
                if m:
                    info[k.strip()] = int(m.group(1))
        total = info.get("MemTotal", 0) / 1024.0
        avail = info.get("MemAvailable", info.get("MemFree", 0)) / 1024.0
        used = total - avail
        return used, total, (100.0 * used / total if total else None)
    except OSError:
        return None, None, None


def read_thermal():
    """Every thermal zone, in degrees Celsius, keyed by its type name."""
    zones = {}
    base = Path("/sys/class/thermal")
    if not base.is_dir():
        return zones
    for zone in sorted(base.glob("thermal_zone*")):
        try:
            name = (zone / "type").read_text().strip()
            milli = float((zone / "temp").read_text().strip())
            zones[name] = milli / 1000.0
        except (OSError, ValueError):
            continue
    return zones


def read_proc_rss_mb(pid):
    """Resident set size of one process, in MB."""
    try:
        with open(f"/proc/{pid}/status") as fh:
            for line in fh:
                if line.startswith("VmRSS:"):
                    return int(re.search(r"(\d+)", line).group(1)) / 1024.0
    except (OSError, AttributeError, ValueError):
        return None
    return None


def find_pids(pattern):
    """PIDs whose command line matches a substring. Used to attribute memory
    to a specific SLAM stack rather than to the machine as a whole."""
    pids = []
    if not pattern:
        return pids
    for d in Path("/proc").iterdir():
        if not d.name.isdigit():
            continue
        try:
            cmd = (d / "cmdline").read_bytes().replace(b"\0", b" ").decode(
                "utf-8", "replace")
        except OSError:
            continue
        if pattern.lower() in cmd.lower():
            pids.append(int(d.name))
    return pids


def summarise(np, samples, key):
    vals = [s[key] for s in samples if s.get(key) is not None]
    if not vals:
        return {}
    a = np.asarray(vals, dtype=float)
    return {"mean": float(a.mean()), "max": float(a.max()),
            "min": float(a.min()), "p95": float(np.percentile(a, 95))}


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Record CPU / GPU / memory / thermal load, and optionally "
                    "latency and dropped frames on a live ROS topic.")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="seconds to sample (0 = until Ctrl-C)")
    ap.add_argument("--interval", type=float, default=1.0,
                    help="seconds between samples")
    ap.add_argument("--topic", default="",
                    help="ROS topic to measure latency and drops on, e.g. "
                         "/zedx_front/zed_node/left/image_rect_color")
    ap.add_argument("--topic-type", default="Image",
                    choices=["Image", "PoseStamped", "Odometry",
                             "PoseWithCovarianceStamped"],
                    help="message type of --topic")
    ap.add_argument("--expected-hz", type=float, default=0.0,
                    help="nominal publish rate, used to infer dropped frames "
                         "(e.g. 30 or 60 for the ZED X)")
    ap.add_argument("--process", default="",
                    help="substring of a process command line to attribute "
                         "memory to, e.g. 'rtabmap'")
    ap.add_argument("--out", default="",
                    help="output JSON (default: "
                         "logs/sidewalk_evaluation/<stamp>_system_load.json)")
    ap.add_argument("--label", default="",
                    help="what configuration this recording describes, e.g. "
                         "'rtabmap alone, 30 FPS, NEURAL depth'")
    args = ap.parse_args()

    # Free-text arguments may arrive from roslaunch still wrapped in the quotes
    # the launch file needed to protect their spaces. See eval_common.unquote.
    from eval_common import unquote
    args.label = unquote(args.label)
    args.process = unquote(args.process)
    args.out = unquote(args.out)
    args.topic = unquote(args.topic)

    with RunLogger("sidewalk_evaluation", run_name="system_monitor") as log:
        try:
            np = require_numpy(log)
        except MissingDependency as exc:
            print(f"\nERROR: {exc}\n", file=sys.stderr)
            log.summary("Cannot monitor: numpy missing", status="FAIL")
            return 3

        log.section("System load monitoring")
        if args.label:
            log.info(f"Configuration under test: {args.label}")

        # ------------------------------------------------- capability probe
        # State up front what this machine can and cannot report, so an
        # 'unavailable' in the final report is never a surprise.
        gpu_probe = read_gpu_pct()
        if gpu_probe is None:
            log.warn(
                "GPU utilisation is not readable on this machine. None of the "
                "known sysfs load nodes exist:\n  " +
                "\n  ".join(GPU_LOAD_PATHS) +
                "\nOn the Jetson AGX Orin under L4T R35.6.1 one of these "
                "should be present. If this is the development PC rather than "
                "the Jetson, that is expected. GPU load will be reported as "
                "unavailable rather than guessed.")
        else:
            log.info(f"GPU load node found; currently {gpu_probe:.1f}%")

        therm = read_thermal()
        if therm:
            log.info("Thermal zones: " +
                     ", ".join(f"{k}={v:.1f}C" for k, v in
                               sorted(therm.items())[:6]))

        pids = find_pids(args.process)
        if args.process:
            if pids:
                log.info(f"Attributing memory to PID(s) {pids} matching "
                         f"'{args.process}'")
            else:
                log.warn(f"No running process matches '{args.process}'. "
                         f"Per-process memory will not be recorded. Start the "
                         f"stack first, then this monitor.")

        # ---------------------------------------------------- optional ROS
        rospy = None
        latency_pairs = []
        recv_count = {"n": 0}
        if args.topic:
            try:
                from eval_common import require_rospy
                rospy = require_rospy(log)
                from sensor_msgs.msg import Image
                from geometry_msgs.msg import (PoseStamped,
                                               PoseWithCovarianceStamped)
                from nav_msgs.msg import Odometry
                TYPES = {"Image": Image, "PoseStamped": PoseStamped,
                         "Odometry": Odometry,
                         "PoseWithCovarianceStamped": PoseWithCovarianceStamped}
                rospy.init_node("system_monitor", anonymous=True)

                def cb(msg):
                    now = rospy.Time.now().to_sec()
                    hdr = getattr(msg, "header", None)
                    stamp = hdr.stamp.to_sec() if hdr is not None else 0.0
                    if stamp > 0:
                        latency_pairs.append((stamp, now))
                    recv_count["n"] += 1

                rospy.Subscriber(args.topic, TYPES[args.topic_type], cb,
                                 queue_size=500)
                log.info(f"Measuring latency on {args.topic} "
                         f"({args.topic_type})")
            except (MissingDependency, ImportError) as exc:
                log.warn(f"Latency measurement disabled: {exc}")
                rospy = None

        # ------------------------------------------------------- sampling
        cpu = CpuSampler()
        time.sleep(min(0.25, args.interval))     # let the first delta be real

        samples = []
        warnings = []
        t0 = time.time()
        log.info(f"Sampling every {args.interval:g} s" +
                 (f" for {args.duration:g} s" if args.duration > 0
                  else " until Ctrl-C"))
        throttle_warned = False
        try:
            while True:
                now = time.time()
                if args.duration > 0 and (now - t0) >= args.duration:
                    break
                if rospy is not None and rospy.is_shutdown():
                    break

                agg, per_core = cpu.sample()
                used_mb, total_mb, mem_pct = read_mem()
                zones = read_thermal()
                hottest = max(zones.values()) if zones else None
                rss = None
                if pids:
                    vals = [v for v in (read_proc_rss_mb(p) for p in pids)
                            if v is not None]
                    rss = sum(vals) if vals else None

                samples.append({
                    "t": now - t0,
                    "cpu_pct": agg,
                    "cpu_busiest_core_pct": max(per_core) if per_core else None,
                    "gpu_pct": read_gpu_pct(),
                    "mem_pct": mem_pct,
                    "mem_used_mb": used_mb,
                    "proc_rss_mb": rss,
                    "temp_max_c": hottest,
                })

                # A hot Orin silently reduces clocks, and every latency number
                # after that point belongs to a different machine. Say so once,
                # loudly, at the moment it happens.
                if hottest is not None and hottest > 85.0 and not throttle_warned:
                    throttle_warned = True
                    msg = (f"Thermal zone reached {hottest:.1f} C at "
                           f"t={now - t0:.0f}s. The Orin throttles above about "
                           f"this point; latency and frame-rate figures "
                           f"recorded after this moment describe a throttled "
                           f"machine and are not comparable with earlier ones.")
                    log.warn(msg)
                    warnings.append(msg)

                if len(samples) % 30 == 0:
                    log.info(f"t={now - t0:6.0f}s  cpu={agg or 0:5.1f}%  "
                             f"gpu={(read_gpu_pct() or float('nan')):5.1f}%  "
                             f"mem={mem_pct or 0:5.1f}%  "
                             f"msgs={recv_count['n']}")

                time.sleep(args.interval)
        except KeyboardInterrupt:
            log.info("Stopped by operator.")

        if len(samples) < 2:
            log.error(
                f"Only {len(samples)} sample(s) collected — nothing "
                f"meaningful can be summarised. Run for at least a few "
                f"seconds with `--duration`.")
            log.summary("Too few samples to summarise", status="FAIL")
            return 5

        # ------------------------------------------------------- summarise
        summary = {}
        for key, out_key in (("cpu_pct", "cpu"), ("gpu_pct", "gpu"),
                             ("mem_pct", "mem"), ("proc_rss_mb", "proc_rss"),
                             ("temp_max_c", "temp"),
                             ("cpu_busiest_core_pct", "cpu_busiest_core")):
            st = summarise(np, samples, key)
            for stat, val in st.items():
                unit = "%" if out_key in ("cpu", "gpu", "mem",
                                          "cpu_busiest_core") else (
                    "MB" if "rss" in out_key else "C")
                summary[f"{out_key}_{stat}_{'pct' if unit == '%' else ('mb' if unit == 'MB' else 'c')}"] = val
        summary["mem_max_mb"] = (summarise(np, samples, "mem_used_mb") or {}).get("max")
        summary["n_samples"] = len(samples)
        summary["duration_s"] = samples[-1]["t"]

        for k, v in sorted(summary.items()):
            if isinstance(v, (int, float)):
                log.metric(k, round(float(v), 2))

        # --------------------------------------------------- latency block
        latency = {"available": False,
                   "reason": "no --topic was given, so no latency was measured"}
        if args.topic and latency_pairs:
            import traj_metrics as TM
            latency = TM.compute_latency(latency_pairs,
                                         expected_hz=args.expected_hz)
            latency["samples_ms"] = [1000.0 * (b - a) for a, b in latency_pairs]
            lm = latency.get("latency_ms", {})
            for k in ("median", "mean", "p95", "max"):
                if lm.get(k) is not None:
                    log.metric(f"latency_{k}_ms", round(lm[k], 2), "ms")
            if latency.get("dropped_frame_pct") is not None:
                log.metric("dropped_frame_pct",
                           round(latency["dropped_frame_pct"], 3), "%")
                log.metric("effective_rate_hz",
                           round(latency.get("effective_rate_hz") or 0.0, 2), "Hz")
            if latency.get("warning"):
                log.warn(latency["warning"])
                warnings.append(latency["warning"])
            if (args.expected_hz > 0 and latency.get("effective_rate_hz")
                    and latency["effective_rate_hz"] < 0.9 * args.expected_hz):
                msg = (f"Effective rate {latency['effective_rate_hz']:.1f} Hz "
                       f"is more than 10% below the configured "
                       f"{args.expected_hz:.0f} Hz. The pipeline is not keeping "
                       f"up. Check in this order: the ZED SDK depth mode "
                       f"(NEURAL costs far more GPU than PERFORMANCE), the "
                       f"streamed resolution, and whether both deserializer "
                       f"groups on the ZED Link Quad card are in use.")
                log.warn(msg)
                warnings.append(msg)
        elif args.topic:
            msg = (f"No messages with a usable header timestamp arrived on "
                   f"{args.topic}. Either nothing is publishing there — check "
                   f"`rostopic hz {args.topic}` — or the messages carry a zero "
                   f"stamp, which makes latency unmeasurable.")
            log.warn(msg)
            warnings.append(msg)
            latency = {"available": False, "reason": msg}

        # ------------------------------------------------------------ save
        out = Path(args.out) if args.out else \
            (log.repo / "logs" / "sidewalk_evaluation" /
             f"{timestamp_slug()}_system_load.json")
        payload = {
            "label": args.label,
            "host": os.uname().nodename,
            "gpu_available": gpu_probe is not None,
            "process_filter": args.process,
            "pids": pids,
            "summary": summary,
            "latency": latency,
            "samples": samples,
            "warnings": warnings,
            "note": ("Load figures describe the WHOLE machine unless a "
                     "--process filter was given. When several SLAM stacks run "
                     "concurrently they compete, so these numbers characterise "
                     "the configuration as a whole, not any one algorithm. To "
                     "attribute cost per algorithm, replay the same rosbag "
                     "three times running one stack each pass."),
        }
        write_json(out, payload)
        log.info(f"Wrote {out}")
        log.info(f"Copy it into an evaluation results directory as "
                 f"`system_load.json` for it to appear in the HTML report.")

        # Build the one-line summary defensively: any of these may legitimately
        # be None on a machine that cannot report them, and a formatting crash
        # here would throw away a perfectly good recording that is already on
        # disk.
        from eval_common import fmt
        bits = [f"{summary['duration_s']:.0f} s monitored",
                f"CPU mean {fmt(summary.get('cpu_mean_pct'), 1, '%')} "
                f"(peak {fmt(summary.get('cpu_max_pct'), 1, '%')})",
                f"GPU mean {fmt(summary.get('gpu_mean_pct'), 1, '%')}"
                if gpu_probe is not None else "GPU unavailable",
                f"RAM peak {fmt(summary.get('mem_max_mb'), 0, 'MB')}"]
        lat_med = ((latency.get("latency_ms") or {}).get("median")
                   if latency.get("available") else None)
        if lat_med is not None:
            bits.append(f"latency median {fmt(lat_med, 1, 'ms')}")
        if latency.get("dropped_frame_pct") is not None:
            bits.append(f"drops {fmt(latency['dropped_frame_pct'], 2, '%')}")
        log.summary(", ".join(bits), status="WARN" if warnings else "OK")
        return 0


if __name__ == "__main__":
    sys.exit(main())
