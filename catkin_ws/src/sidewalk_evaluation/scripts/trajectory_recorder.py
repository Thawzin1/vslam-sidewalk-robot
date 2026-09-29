#!/usr/bin/env python3
"""
trajectory_recorder.py — Capture what each SLAM stack claims, while it runs.

WHAT IT DOES
    Subscribes to the pose output of several localization systems at once —
    RTAB-Map, the ZED SDK's own tracking, ORB-SLAM3, and the robot base's wheel
    odometry — and writes each to its own TUM-format file. It also records a
    tracking-status log and per-pose latency samples for each.

    The output is exactly what `evaluate_trajectory.py` consumes. Recording and
    analysis are separate programs on purpose: the recording happens once, on
    the robot, and can never be repeated; the analysis happens many times, on
    whatever machine, as the metrics are refined.

WHY RECORD ALL STACKS SIMULTANEOUSLY
    The project's central claim is a comparison between three systems. A
    comparison is only valid if the systems saw **identical input** — the same
    frames, the same lighting, the same pedestrian who walked in front of the
    camera at t=91 s. Running them one after another down the same sidewalk
    does not achieve that; the world changes between runs.

    Two ways to get identical input, both supported here:
      1. Run all three live, concurrently, on the same Jetson. Honest about
         real-time behaviour, but they compete for CPU and GPU, so the latency
         and load figures are for the three-way configuration, not for any one
         system alone.
      2. Record a rosbag once, then replay it three times, recording one stack
         each pass. Perfectly identical input and clean per-system resource
         figures, but no longer real-time evidence.

    Do both, and say which is which. `config/evaluation.yaml` documents the
    recommended protocol.

TIMESTAMPS — THE THING TO GET RIGHT
    Every pose is stamped with the **sensor capture time** taken from the
    message header, not the time the message arrived. Those differ by the
    processing latency, which is precisely the quantity under study. Using
    arrival time would silently shift every trajectory by a variable amount and
    make the comparison between a fast stack and a slow one meaningless.

    Where a message has no usable header stamp, arrival time is used and the
    fact is recorded in the file header and in the log, because that trajectory
    is then not directly comparable with the others.
"""
from __future__ import annotations

import sys
from pathlib import Path

from eval_common import (MissingDependency, RunLogger, load_config,
                         timestamp_slug, write_json)

DEFAULT_CONFIG = Path(__file__).resolve().parent.parent / "config" / "evaluation.yaml"


class SourceRecorder:
    """One pose source -> one TUM file, one status file, one latency file.

    Files are opened once and flushed after every pose. That costs a little
    throughput and buys the property that matters far more on a research robot:
    if the node is killed, the battery dies, or the Jetson thermal-throttles
    into oblivion, everything recorded up to that instant is already on disk.
    A buffered writer would lose the last few seconds — which, on the run where
    something interesting went wrong, are the seconds you needed.
    """

    def __init__(self, name: str, topic: str, msg_type: str, out_dir: Path,
                 log, status_topic: str = "", frame_note: str = ""):
        self.name = name
        self.topic = topic
        self.msg_type = msg_type
        self.log = log
        self.status_topic = status_topic

        self.traj_path = out_dir / f"{name}.tum"
        self.status_path = out_dir / f"{name}.status"
        self.latency_path = out_dir / f"{name}.latency"

        self.count = 0
        self.first_stamp = None
        self.last_stamp = None
        self.used_arrival_time = 0
        self.last_status = None
        self.status_events = 0

        self._traj = open(self.traj_path, "w", buffering=1)
        self._traj.write("# timestamp tx ty tz qx qy qz qw\n")
        self._traj.write(f"# source topic: {topic}  ({msg_type})\n")
        self._traj.write(f"# stamped with SENSOR CAPTURE TIME from the message "
                         f"header where available\n")
        if frame_note:
            self._traj.write(f"# {frame_note}\n")
        self._status = open(self.status_path, "w", buffering=1)
        self._status.write("# timestamp STATUS [note]\n")
        self._latency = open(self.latency_path, "w", buffering=1)
        self._latency.write("# capture_time_s publish_time_s\n")

    # ------------------------------------------------------------- callbacks
    def on_pose(self, capture_t: float, arrival_t: float, pos, quat,
                had_header: bool) -> None:
        """Record one pose. `pos` is (x, y, z); `quat` is (x, y, z, w)."""
        if not had_header:
            self.used_arrival_time += 1
        self._traj.write(
            f"{capture_t:.9f} {pos[0]:.6f} {pos[1]:.6f} {pos[2]:.6f} "
            f"{quat[0]:.7f} {quat[1]:.7f} {quat[2]:.7f} {quat[3]:.7f}\n")
        # The gap between the shutter and this instant is the end-to-end age of
        # the information the navigation stack is about to steer on.
        self._latency.write(f"{capture_t:.9f} {arrival_t:.9f}\n")
        self.count += 1
        if self.first_stamp is None:
            self.first_stamp = capture_t
        self.last_stamp = capture_t

    def on_status(self, t: float, status: str, note: str = "") -> None:
        """Record a tracking-status transition.

        Only transitions are written, not every message. A stack that publishes
        LOST at 30 Hz for two seconds is one failure, not sixty, and the metric
        code counts episodes. Writing every message would inflate the file and
        tempt someone into counting lines.
        """
        status = status.upper()
        if status == self.last_status:
            return
        self.last_status = status
        self._status.write(f"{t:.9f} {status}"
                           f"{(' ' + note) if note else ''}\n")
        self.status_events += 1
        if status == "LOST":
            self.log.warn(f"{self.name}: TRACKING LOST at t={t:.3f}"
                          f"{(' — ' + note) if note else ''}")
        elif status == "RELOCALIZED":
            self.log.info(f"{self.name}: relocalized at t={t:.3f}")

    def close(self) -> dict:
        for fh in (self._traj, self._status, self._latency):
            try:
                fh.close()
            except Exception:                              # noqa: BLE001
                pass
        duration = ((self.last_stamp - self.first_stamp)
                    if self.first_stamp is not None and self.last_stamp is not None
                    else 0.0)
        return {
            "name": self.name,
            "topic": self.topic,
            "msg_type": self.msg_type,
            "poses": self.count,
            "duration_s": duration,
            "mean_rate_hz": (self.count - 1) / duration if duration > 0 else 0.0,
            "status_transitions": self.status_events,
            "poses_without_header_stamp": self.used_arrival_time,
            "trajectory_file": str(self.traj_path),
        }


def build_sources(cfg: dict) -> list:
    """Read the pose-source table out of the config.

    Config shape (see config/evaluation.yaml):

        sources:
          rtabmap:
            topic: /rtabmap/localization_pose
            type: PoseWithCovarianceStamped
            status_topic: /rtabmap/info
          zed:
            topic: /zedx_front/zed_node/pose
            type: PoseStamped

    Note the per-camera namespace in the ZED topic. A second ZED X facing the
    opposite direction is planned; every topic and frame id in this project is
    namespaced so that adding it is a config change, not a rewrite.
    """
    srcs = cfg.get("sources")
    if not isinstance(srcs, dict) or not srcs:
        return []
    out = []
    for name, spec in srcs.items():
        if not isinstance(spec, dict):
            continue
        if spec.get("enabled") is False:
            continue
        out.append({
            "name": str(name),
            "topic": str(spec.get("topic", "")),
            "type": str(spec.get("type", "PoseStamped")),
            "status_topic": str(spec.get("status_topic", "") or ""),
        })
    return [s for s in out if s["topic"]]


def main() -> int:
    import argparse
    ap = argparse.ArgumentParser(
        description="Record the pose output of every SLAM stack to TUM files.")
    ap.add_argument("--config", default=str(DEFAULT_CONFIG))
    ap.add_argument("--out", default="",
                    help="output directory (default: "
                         "logs/sidewalk_evaluation/<timestamp>_record)")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="stop automatically after this many seconds "
                         "(0 = run until Ctrl-C)")
    ap.add_argument("--wait", type=float, default=20.0,
                    help="seconds to wait for the first message on each topic "
                         "before declaring a source dead")
    args = ap.parse_args()

    run_name = "record"
    with RunLogger("sidewalk_evaluation", run_name=run_name) as log:
        # --------------------------------------------------- dependency gate
        try:
            from eval_common import require_rospy
            rospy = require_rospy(log)
        except MissingDependency as exc:
            print(f"\nERROR: {exc}\n", file=sys.stderr)
            log.summary("Cannot record: rospy unavailable", status="FAIL")
            return 3

        try:
            from geometry_msgs.msg import (PoseStamped,
                                           PoseWithCovarianceStamped)
            from nav_msgs.msg import Odometry
        except ImportError as exc:
            log.error(f"ROS message packages are not importable ({exc}). The "
                      f"workspace is probably not sourced: run "
                      f"`source ~/catkin_ws/devel/setup.bash`.")
            log.summary("Cannot record: ROS messages unavailable", status="FAIL")
            return 3

        try:
            cfg = load_config(args.config)
        except FileNotFoundError as exc:
            log.error(str(exc))
            log.summary("Configuration missing", status="FAIL")
            return 2

        sources = build_sources(cfg)
        if not sources:
            log.error(
                "No pose sources are configured. Add a `sources:` block to "
                f"{args.config} listing each SLAM stack's pose topic. "
                "Without it there is nothing to record — see the commented "
                "example in the shipped config.")
            log.summary("No pose sources configured", status="FAIL")
            return 2

        out_dir = Path(args.out) if args.out else \
            (log.repo / "logs" / "sidewalk_evaluation" /
             f"{timestamp_slug()}_record")
        out_dir.mkdir(parents=True, exist_ok=True)
        log.info(f"Recording to {out_dir}")

        rospy.init_node("trajectory_recorder", anonymous=True,
                        disable_signals=False)

        recorders = {}
        subs = []

        def make_pose_cb(rec: SourceRecorder):
            def cb(msg):
                now = rospy.Time.now().to_sec()
                # Pull position/orientation out of whichever wrapper this is.
                if hasattr(msg, "pose") and hasattr(msg.pose, "pose"):
                    p = msg.pose.pose          # Odometry / PoseWithCovariance
                elif hasattr(msg, "pose"):
                    p = msg.pose               # PoseStamped
                else:
                    p = msg                    # bare Pose
                hdr = getattr(msg, "header", None)
                stamp = hdr.stamp.to_sec() if hdr is not None else 0.0
                had_header = stamp > 0.0
                capture_t = stamp if had_header else now
                rec.on_pose(
                    capture_t, now,
                    (p.position.x, p.position.y, p.position.z),
                    (p.orientation.x, p.orientation.y,
                     p.orientation.z, p.orientation.w),
                    had_header)
                # A pose arriving at all is evidence tracking is alive. Stacks
                # that publish an explicit status overwrite this via their own
                # status topic; for the rest, presence is the only signal there
                # is, and silence is what a loss looks like.
                if rec.last_status is None:
                    rec.on_status(capture_t, "OK", "first pose received")
            return cb

        TYPES = {
            "PoseStamped": PoseStamped,
            "PoseWithCovarianceStamped": PoseWithCovarianceStamped,
            "Odometry": Odometry,
        }

        for spec in sources:
            cls = TYPES.get(spec["type"])
            if cls is None:
                log.error(f"{spec['name']}: unsupported message type "
                          f"'{spec['type']}'. Supported: "
                          f"{', '.join(sorted(TYPES))}. If the stack publishes "
                          f"something else, remap it with a topic_tools relay "
                          f"or add the type here.")
                continue
            rec = SourceRecorder(spec["name"], spec["topic"], spec["type"],
                                 out_dir, log)
            recorders[spec["name"]] = rec
            subs.append(rospy.Subscriber(spec["topic"], cls,
                                         make_pose_cb(rec), queue_size=200))
            log.info(f"Subscribed: {spec['name']} <- {spec['topic']} "
                     f"({spec['type']})")

        if not recorders:
            log.error("No usable pose sources after type checking.")
            log.summary("No usable pose sources", status="FAIL")
            return 2

        # --------------------------------------------- wait for first data
        # Graceful degradation: a topic that never publishes is a specific,
        # nameable failure, not a hang. Say which topic, and say what that
        # usually means, then keep recording whatever IS alive.
        log.info(f"Waiting up to {args.wait:.0f} s for the first message on "
                 f"each topic...")
        deadline = rospy.Time.now().to_sec() + args.wait
        rate = rospy.Rate(5)
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() < deadline:
            if all(r.count > 0 for r in recorders.values()):
                break
            rate.sleep()

        silent = [n for n, r in recorders.items() if r.count == 0]
        for n in silent:
            log.error(
                f"'{n}' published nothing on {recorders[n].topic} within "
                f"{args.wait:.0f} s. Check, in order: (1) is that stack "
                f"actually running — `rosnode list`; (2) is the topic name "
                f"right — `rostopic list | grep -i {n}`; (3) is it publishing "
                f"but nobody is moving — `rostopic hz {recorders[n].topic}`. "
                f"An empty trajectory file will be written for it so the "
                f"absence is recorded rather than silently missing.")
        if len(silent) == len(recorders):
            log.summary(f"No pose data on any of {len(recorders)} configured "
                        f"topics — nothing was recorded", status="FAIL")
            for r in recorders.values():
                r.close()
            return 4

        # ---------------------------------------------------------- record
        log.info("Recording. Press Ctrl-C to stop." +
                 (f" Auto-stop after {args.duration:.0f} s."
                  if args.duration > 0 else ""))
        start = rospy.Time.now().to_sec()
        last_report = start
        try:
            while not rospy.is_shutdown():
                now = rospy.Time.now().to_sec()
                if args.duration > 0 and (now - start) >= args.duration:
                    log.info(f"Reached the requested {args.duration:.0f} s.")
                    break
                if now - last_report >= 15.0:
                    last_report = now
                    parts = [f"{n}={r.count}" for n, r in recorders.items()]
                    log.info(f"t={now - start:6.0f}s  poses: "
                             + "  ".join(parts))
                    # A source that stops mid-run is the interesting failure
                    # this loop exists to catch.
                    for n, r in recorders.items():
                        if r.last_stamp is not None and now - r.last_stamp > 5.0:
                            r.on_status(now, "LOST",
                                        f"no pose for {now - r.last_stamp:.1f}s")
                rospy.sleep(0.2)
        except (KeyboardInterrupt, rospy.ROSInterruptException):
            log.info("Stopped by operator.")

        # ----------------------------------------------------------- finish
        summary = {"output_dir": str(out_dir), "sources": []}
        parts = []
        for n, r in recorders.items():
            info = r.close()
            summary["sources"].append(info)
            log.metric(f"{n}_poses", info["poses"])
            log.metric(f"{n}_rate_hz", round(info["mean_rate_hz"], 2), "Hz")
            if info["poses_without_header_stamp"]:
                log.warn(
                    f"{n}: {info['poses_without_header_stamp']} pose(s) had no "
                    f"header timestamp and were stamped with arrival time "
                    f"instead. Those poses carry the processing latency baked "
                    f"in and are not directly comparable with the other "
                    f"stacks — note this when reporting.")
            parts.append(f"{n} {info['poses']} poses @ "
                         f"{info['mean_rate_hz']:.1f} Hz")
        write_json(out_dir / "recording_summary.json", summary)

        log.info(f"Next step:  rosrun sidewalk_evaluation "
                 f"evaluate_trajectory.py --loop-only "
                 + " ".join(f"--est {n}={out_dir}/{n}.tum" for n in recorders))
        log.summary(f"Recorded {len(recorders)} pose source(s): "
                    + "; ".join(parts) + f"  -> {out_dir.name}",
                    status="OK" if not silent else "WARN")
        return 0


if __name__ == "__main__":
    sys.exit(main())
