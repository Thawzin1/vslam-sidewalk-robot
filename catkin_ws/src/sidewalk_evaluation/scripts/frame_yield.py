#!/usr/bin/env python3
"""
frame_yield.py — measure how many camera frames actually become odometry poses,
stage by stage, so a low yield can be BLAMED on a specific stage instead of
tuned at blindly.

WHY THIS EXISTS
    Phase-1 gate: frame yield >= 95%. The mcity_v2_run2b analysis found the
    pipeline producing poses at ~8.8 Hz from a 15 Hz camera (~59%), which
    doubles effective inter-frame motion — the exact mechanism behind tracking
    loss. But "yield is 59%" names no culprit: the frame could be lost at the
    synchroniser (stamps never paired), or at odometry (frame arrived, no pose
    computed). Those have different fixes, so they must be measured apart.

STAGES
    0  camera      left/camera_info (stereo) or rgb/camera_info (RGB-D) —
                   tiny messages, safe to subscribe anywhere. The denominator.
    1  sync        /rtabmap/rgbd_image — the synchroniser's output. LARGE
                   (~16 MB frames): subscribed only in --mode live, never
                   alongside a recorded run.
    2  odometry    /rtabmap/odom — poses out. Null-pose messages (all-zero
                   orientation) are odometry-lost markers and counted
                   separately as invalid.

ALL RATES ARE SIM TIME. Under real-time-factor << 1 wall rates are
meaningless; every ratio here uses message header stamps / /clock.

USAGE
    # preflight, all three stages, 30 s (do NOT use during a recorded run):
    rosrun sidewalk_evaluation frame_yield.py --mode live [--rgbd] [--duration 30]

    # alongside a recorded run (camera_info + odom only, both tiny):
    rosrun sidewalk_evaluation frame_yield.py --mode run \\
        --out ~/.run_records/<run_id>/frame_yield.json [--rgbd]
    # ... stops and writes the JSON on SIGINT/SIGTERM.

The first 10 callbacks per topic are DISCARDED before counting — cached or
backlogged messages have produced false measurements on this system before
(docs/DO_NOT_REPEAT.md, wait_for_message entry).
"""
import argparse
import json
import signal
import sys
from pathlib import Path

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo

GATE_PCT = 95.0
DISCARD = 10


def strip_ros_args(argv):
    return [a for a in (argv if argv is not None else sys.argv[1:])
            if ":=" not in a]


class StageCounter:
    """Counts messages and tracks the sim-time span they arrived over."""

    def __init__(self, name):
        self.name = name
        self.seen = 0          # raw callbacks, including the discarded warmup
        self.count = 0
        self.invalid = 0       # odometry only: null-pose (lost) messages
        self.first_stamp = None
        self.last_stamp = None

    def hit(self, stamp, valid=True):
        self.seen += 1
        if self.seen <= DISCARD:
            return
        if valid:
            self.count += 1
        else:
            self.invalid += 1
        t = stamp.to_sec()
        if self.first_stamp is None:
            self.first_stamp = t
        self.last_stamp = t

    @property
    def span(self):
        if self.first_stamp is None or self.last_stamp is None:
            return 0.0
        return self.last_stamp - self.first_stamp

    def rate(self):
        return (self.count + self.invalid - 1) / self.span if self.span > 0 else 0.0

    def summary(self):
        return {
            "messages": self.count,
            "invalid": self.invalid,
            "sim_span_s": round(self.span, 3),
            "sim_rate_hz": round(self.rate(), 3),
        }


def odom_is_valid(msg):
    """A 'lost' odometry message carries an all-zero quaternion (an invalid
    rotation — real poses always have |q| = 1)."""
    q = msg.pose.pose.orientation
    return abs(q.x) + abs(q.y) + abs(q.z) + abs(q.w) > 1e-9


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mode", choices=("live", "run"), default="live",
                    help="live = all 3 stages for --duration seconds (preflight "
                         "only: stage 1 subscribes 16 MB frames). run = camera + "
                         "odom only, safe alongside a recorded run, stops on "
                         "SIGINT and writes --out.")
    ap.add_argument("--rgbd", action="store_true",
                    help="RGB-D pipeline: count rgb/camera_info instead of "
                         "left/camera_info as the camera stage.")
    ap.add_argument("--camera-ns", default="/zedx_front/zed_node")
    ap.add_argument("--duration", type=float, default=30.0,
                    help="live mode: measurement window, SIM seconds")
    ap.add_argument("--out", default="",
                    help="run mode: JSON summary path, written at shutdown")
    args = ap.parse_args(strip_ros_args(None))

    rospy.init_node("frame_yield", anonymous=True)

    cam_topic = (f"{args.camera_ns}/rgb/camera_info" if args.rgbd
                 else f"{args.camera_ns}/left/camera_info")

    camera = StageCounter("camera")
    sync = StageCounter("sync")
    odom = StageCounter("odometry")

    rospy.Subscriber(cam_topic, CameraInfo,
                     lambda m: camera.hit(m.header.stamp), queue_size=50)
    rospy.Subscriber("/rtabmap/odom", Odometry,
                     lambda m: odom.hit(m.header.stamp, odom_is_valid(m)),
                     queue_size=50)
    if args.mode == "live":
        # Import here: only live mode pays for the big subscription.
        from rtabmap_msgs.msg import RGBDImage
        rospy.Subscriber("/rtabmap/rgbd_image", RGBDImage,
                         lambda m: sync.hit(m.header.stamp), queue_size=2,
                         buff_size=2 ** 26)

    def report():
        out = {
            "mode": args.mode,
            "camera_topic": cam_topic,
            "camera": camera.summary(),
            "odometry": odom.summary(),
        }
        if args.mode == "live":
            out["sync"] = sync.summary()

        cam_n = camera.count
        # Yield definitions: every ratio against DELIVERED camera frames — if
        # the camera itself underdelivers vs its nominal 15 Hz, that is
        # reported as camera_rate and judged separately, not hidden in the
        # yield (denominator disclosed).
        if cam_n > 0:
            out["yield_odom_valid_pct"] = round(100.0 * odom.count / cam_n, 2)
            out["yield_odom_all_pct"] = round(
                100.0 * (odom.count + odom.invalid) / cam_n, 2)
            if args.mode == "live":
                out["yield_sync_pct"] = round(100.0 * sync.count / cam_n, 2)
        out["gate_pct"] = GATE_PCT
        out["gate_pass"] = bool(cam_n > 0 and
                                out.get("yield_odom_valid_pct", 0.0) >= GATE_PCT)
        return out

    def print_report(out):
        print("\n=== frame yield (all rates SIM time) ===")
        print(f"  camera   {out['camera']['sim_rate_hz']:6.2f} Hz  "
              f"({out['camera']['messages']} frames / "
              f"{out['camera']['sim_span_s']:.1f} s)   [{out['camera_topic']}]")
        if "sync" in out:
            print(f"  sync     {out['sync']['sim_rate_hz']:6.2f} Hz  "
                  f"-> yield {out.get('yield_sync_pct', 0):.1f}% of camera")
        print(f"  odometry {out['odometry']['sim_rate_hz']:6.2f} Hz  "
              f"valid {out['odometry']['messages']}, "
              f"lost {out['odometry']['invalid']}  "
              f"-> valid yield {out.get('yield_odom_valid_pct', 0):.1f}% of camera")
        verdict = "PASS" if out["gate_pass"] else "FAIL"
        print(f"  GATE (>= {GATE_PCT:.0f}% valid poses per camera frame): {verdict}")

    if args.mode == "live":
        t0 = rospy.Time.now()
        rate = rospy.Rate(5)
        while (not rospy.is_shutdown()
               and (rospy.Time.now() - t0).to_sec() < args.duration):
            rate.sleep()
        out = report()
        print_report(out)
        sys.exit(0 if out["gate_pass"] else 1)
    else:
        done = {"flag": False}

        def _stop(_s, _f):
            done["flag"] = True

        signal.signal(signal.SIGINT, _stop)
        signal.signal(signal.SIGTERM, _stop)
        rate = rospy.Rate(2)
        while not rospy.is_shutdown() and not done["flag"]:
            rate.sleep()
        out = report()
        print_report(out)
        if args.out:
            p = Path(args.out).expanduser()
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_text(json.dumps(out, indent=2))
            print(f"  wrote {p}")


if __name__ == "__main__":
    main()
