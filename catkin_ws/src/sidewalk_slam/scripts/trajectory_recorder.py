#!/usr/bin/env python3
"""
trajectory_recorder.py — record any pose stream to a single TUM trajectory file.

WHY THIS EXISTS
    The three SLAM stacks under comparison do not agree on how to tell you where
    the robot is:

      * RTAB-Map publishes nav_msgs/Odometry and, more authoritatively, a
        map -> odom correction on TF.
      * The ZED SDK wrapper publishes both nav_msgs/Odometry and a
        geometry_msgs/PoseStamped.
      * ORB-SLAM3 does not publish anything useful at all; it writes a text file
        when you shut it down.

    If each stack were recorded by its own bespoke snippet, the comparison would
    be measuring the recording code as much as the SLAM. So there is one
    recorder, one output format, and one set of rules about what counts as a
    pose.

WHICH SOURCE SHOULD YOU RECORD?
    Prefer TF. A SLAM system's *published pose topic* is often the raw odometry
    before the pose graph is corrected, whereas the TF chain map -> base_link
    carries the corrected estimate. After a loop closure the two differ by
    exactly the drift that was just removed - which is the interesting quantity.

    Recording TF also lets you express every stack's answer for the SAME rigid
    body (the robot base), instead of one at the camera optical centre and
    another at the IMU. Two trajectories that differ only by a fixed lever arm
    look like a real difference in accuracy when you plot them, and they are not.

USAGE
    rosrun sidewalk_slam trajectory_recorder.py \
        --stack rtabmap --run-id 20260720-1400_indoor_loop \
        --source tf --fixed-frame map --body-frame base_link --rate 20

    rosrun sidewalk_slam trajectory_recorder.py \
        --stack zed_sdk --run-id 20260720-1400_indoor_loop \
        --source topic --topic /zedx_front/zed_node/odom --type odom
"""
from __future__ import annotations

import argparse
import signal
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from slam_common import (SlamExit, STACKS, TumWriter, die, make_logger,  # noqa: E402
                         probe_ros, tum_path)


def strip_ros_args(argv):
    """Drop the arguments roslaunch appends, so argparse does not choke.

    roslaunch adds `__name:=` and `__log:=` to every node it starts, and
    remappings arrive as `from:=to`. argparse rejects all of them as
    unrecognised and exits, which is invisible unless you read the launch
    log - the node simply never appears.

    This only bit once this script started being launched from
    record_mapping_run.launch; run by hand through `rosrun` without an
    explicit `__name:=`, nothing was ever appended and it worked fine.

    rospy.myargv() is the canonical way to do this, but importing rospy
    just to parse arguments is heavy and would make `--help` depend on a
    ROS environment. The rule it applies is simply "drop anything
    containing :=", which is what this does.
    """
    return [a for a in (argv if argv is not None else sys.argv[1:])
            if ":=" not in a]


def parse_args(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stack", required=True,
                    help=f"which SLAM stack this trajectory comes from. "
                         f"Known: {', '.join(STACKS)}. Free text is allowed for "
                         f"ablations, but the evaluation package expects the "
                         f"known names.")
    ap.add_argument("--run-id", required=True,
                    help="identifier shared by all stacks in one comparison")
    ap.add_argument("--out", default=None,
                    help="explicit output path (default: the canonical "
                         "logs/sidewalk_slam/trajectories/<run-id>/ location)")

    ap.add_argument("--source", choices=("tf", "topic"), default="tf")
    ap.add_argument("--fixed-frame", default="map",
                    help="TF source: the world-fixed frame (default: map)")
    ap.add_argument("--body-frame", default="base_link",
                    help="TF source: the frame whose motion is recorded "
                         "(default: base_link)")
    ap.add_argument("--rate", type=float, default=20.0,
                    help="TF source: sampling rate in Hz (default: 20)")

    ap.add_argument("--topic", default=None,
                    help="topic source: the topic to subscribe to")
    ap.add_argument("--type", choices=("odom", "pose", "pose_cov"),
                    default="odom", help="topic source: message type")

    ap.add_argument("--duration", type=float, default=0.0,
                    help="stop after this many seconds (0 = until shut down)")
    ap.add_argument("--startup-timeout", type=float, default=30.0,
                    help="fail if no pose arrives within this many seconds")
    ap.add_argument("--input-name", default="",
                    help="name of the recording being replayed, for the sidecar")
    ap.add_argument("--reference-kind", default="",
                    help="what this trajectory IS as a truth source, written "
                         "into the .meta.json sidecar (e.g. "
                         "simulator_ground_truth for /ground_truth/odom). "
                         "evaluate_trajectory.py reads it back and refuses a "
                         "contradicting CLI flag — declaring provenance at "
                         "RECORD time, by the thing that knew, beats any flag "
                         "typed at evaluation time.")
    return ap.parse_args(strip_ros_args(argv))


def _require_ros(log):
    ros = probe_ros()
    if not ros["present"]:
        die(log, "trajectory_recorder.py needs a running ROS environment.",
            f"{ros['detail']}. This machine has no ROS installed; run this node "
            f"on the Jetson.")
    try:
        import rospy  # noqa: F401
    except ImportError as exc:
        die(log, f"rospy is not importable: {exc}",
            "Source /opt/ros/noetic/setup.bash and your catkin workspace.")


def record_from_topic(log, args, writer):
    import rospy

    if not args.topic:
        die(log, "--source topic requires --topic.",
            "For example --topic /zedx_front/zed_node/odom --type odom")

    if args.type == "odom":
        from nav_msgs.msg import Odometry as MsgType
    elif args.type == "pose":
        from geometry_msgs.msg import PoseStamped as MsgType
    else:
        from geometry_msgs.msg import PoseWithCovarianceStamped as MsgType

    state = {"first": True, "frame": "", "child": ""}

    def cb(msg):
        if args.type == "odom":
            pose = msg.pose.pose
            state["child"] = msg.child_frame_id
        elif args.type == "pose_cov":
            pose = msg.pose.pose
        else:
            pose = msg.pose
        state["frame"] = msg.header.frame_id
        # Use the message's own header stamp, never rospy.Time.now(). Under
        # `use_sim_time` with a bag replay, now() is the simulated clock and can
        # differ from the stamp the SLAM system actually reasoned about; using
        # the header keeps two replays of the same bag comparable.
        t = msg.header.stamp.to_sec()
        ok = writer.add(t, pose.position.x, pose.position.y, pose.position.z,
                        pose.orientation.x, pose.orientation.y,
                        pose.orientation.z, pose.orientation.w)
        if state["first"] and ok:
            state["first"] = False
            log.info(f"first pose received at t={t:.3f} in frame "
                     f"'{state['frame']}'")

    rospy.Subscriber(args.topic, MsgType, cb, queue_size=200)
    log.info(f"subscribed to {args.topic} ({args.type})")

    _spin_until_done(log, args, writer, state)
    writer.frame_id = state["frame"]
    writer.child_frame_id = state["child"]


def record_from_tf(log, args, writer):
    import rospy
    import tf2_ros

    buf = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
    tf2_ros.TransformListener(buf)
    log.info(f"sampling TF {args.fixed_frame} -> {args.body_frame} "
             f"at {args.rate:.1f} Hz")

    writer.frame_id = args.fixed_frame
    writer.child_frame_id = args.body_frame

    rate = rospy.Rate(max(1.0, args.rate))
    start = rospy.Time.now().to_sec()
    state = {"first": True}
    last_stamp = None
    warned = False

    while not rospy.is_shutdown():
        try:
            tr = buf.lookup_transform(args.fixed_frame, args.body_frame,
                                      rospy.Time(0))
        except Exception as exc:
            now = rospy.Time.now().to_sec()
            if state["first"] and (now - start) > args.startup_timeout:
                die(log,
                    f"no transform {args.fixed_frame} -> {args.body_frame} "
                    f"appeared within {args.startup_timeout:.0f}s "
                    f"({type(exc).__name__}).",
                    "Either the SLAM node has not started publishing yet, or "
                    "the frame names are wrong. Check with "
                    "`rosrun rqt_tf_tree rqt_tf_tree`. A frame floating "
                    "unconnected in that diagram is the usual cause.")
            if not warned and (now - start) > 5.0:
                warned = True
                log.warn(f"still waiting for {args.fixed_frame} -> "
                         f"{args.body_frame} ({type(exc).__name__})")
            rate.sleep()
            continue

        t = tr.header.stamp.to_sec()
        # TF is sampled on a timer, so the same transform can be read twice
        # before the publisher updates it. Writing duplicates would inflate the
        # pose count and bias any rate-based statistic, so they are skipped.
        if last_stamp is not None and t <= last_stamp:
            rate.sleep()
            continue
        last_stamp = t

        tl, q = tr.transform.translation, tr.transform.rotation
        ok = writer.add(t, tl.x, tl.y, tl.z, q.x, q.y, q.z, q.w)
        if state["first"] and ok:
            state["first"] = False
            log.info(f"first transform captured at t={t:.3f}")

        if args.duration > 0 and (rospy.Time.now().to_sec() - start) >= args.duration:
            log.info(f"reached --duration {args.duration:.1f}s; stopping")
            break
        rate.sleep()


def _spin_until_done(log, args, writer, state):
    import rospy
    start = rospy.Time.now().to_sec()
    warned = False
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        now = rospy.Time.now().to_sec()
        if state["first"]:
            if (now - start) > args.startup_timeout:
                die(log, f"no message on {args.topic} within "
                         f"{args.startup_timeout:.0f}s.",
                    f"Check the topic exists and is publishing: "
                    f"`rostopic hz {args.topic}`. If it exists but is silent, "
                    f"the SLAM node is running but not producing a pose - look "
                    f"at its own log for a tracking failure.")
            if not warned and (now - start) > 5.0:
                warned = True
                log.warn(f"still waiting for the first message on {args.topic}")
        if args.duration > 0 and (now - start) >= args.duration:
            log.info(f"reached --duration {args.duration:.1f}s; stopping")
            break
        r.sleep()


def main(argv=None):
    args = parse_args(argv)
    run_name = f"record_{args.stack}"
    with make_logger(run_name) as log:
        _require_ros(log)
        import rospy

        if args.stack not in STACKS:
            log.warn(f"'{args.stack}' is not one of the known stacks "
                     f"({', '.join(STACKS)}). The file will be written, but the "
                     f"evaluation package may not pick it up automatically.")

        out = Path(args.out).expanduser() if args.out else tum_path(args.run_id,
                                                                    args.stack)
        log.info(f"trajectory will be written to {out}")

        rospy.init_node(f"trajectory_recorder_{args.stack}", anonymous=True,
                        disable_signals=True)

        writer = TumWriter(out, stack=args.stack,
                           source=(args.topic if args.source == "topic"
                                   else f"tf:{args.fixed_frame}->{args.body_frame}"),
                           input_name=args.input_name,
                           extra={"run_id": args.run_id,
                                  "source_kind": args.source,
                                  **({"reference_kind": args.reference_kind}
                                     if args.reference_kind else {})})

        # A recorder that loses its data on Ctrl-C is worse than useless, so
        # SIGINT/SIGTERM are handled explicitly: close the file, write the
        # sidecar, then let the normal exit path report the summary.
        def _stop(signum, _frame):
            log.info(f"received signal {signum}; closing trajectory cleanly")
            rospy.signal_shutdown("signal")

        signal.signal(signal.SIGINT, _stop)
        signal.signal(signal.SIGTERM, _stop)

        try:
            if args.source == "topic":
                record_from_topic(log, args, writer)
            else:
                record_from_tf(log, args, writer)
        finally:
            meta = writer.close()

        log.metric("poses_written", meta["poses_written"])
        log.metric("poses_rejected", meta["poses_rejected"])
        if meta["duration_s"]:
            log.metric("trajectory_duration", round(meta["duration_s"], 2), "s")
            if meta["poses_written"] > 1:
                log.metric("effective_pose_rate",
                           round(meta["poses_written"] / meta["duration_s"], 2),
                           "Hz")

        if meta["poses_written"] == 0:
            die(log, "no poses were recorded at all.",
                "The recorder ran but the SLAM stack never produced a pose. "
                "That is a SLAM failure, not a recorder failure - check whether "
                "it ever initialised. For ORB-SLAM3 and RTAB-Map alike, a "
                "featureless first few seconds (blank wall, lens cap, dark "
                "room) prevents initialisation entirely.")
        if meta["poses_rejected"]:
            log.warn(f"{meta['poses_rejected']} poses were rejected as NaN or "
                     f"degenerate and are NOT in the file. That usually means "
                     f"the estimator diverged at some point.")

        log.summary(
            f"{args.stack}: recorded {meta['poses_written']} poses over "
            f"{meta['duration_s'] or 0:.1f}s to {out.name}",
            status="WARN" if meta["poses_rejected"] else "OK")
        return 0


if __name__ == "__main__":
    # SlamExit is how every tool in this package reports an already-diagnosed
    # refusal. The reason has been printed and logged by the time it reaches
    # here; all that is left is to hand the shell a non-zero status.
    try:
        sys.exit(main())
    except SlamExit as _exit:
        sys.exit(_exit.code)
