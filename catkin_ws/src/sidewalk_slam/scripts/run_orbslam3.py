#!/usr/bin/env python3
"""
run_orbslam3.py — launch the ORB-SLAM3 baseline safely and collect its
trajectory in the project's common TUM format.

WHY A WRAPPER INSTEAD OF A PLAIN <node> TAG
    ORB-SLAM3 is not a ROS package in any normal sense. It is a CMake library
    with example executables, one set of which happens to link against roscpp.
    Pointing roslaunch straight at it produces unhelpful failures, and the three
    things that actually go wrong are all invisible from a launch file:

      1. The vocabulary is still `ORBvoc.txt.tar.gz`. Nobody extracted it. The
         binary starts, prints "Loading ORB Vocabulary", and then either hangs
         or dies with no explanation. This is the single most common ORB-SLAM3
         first-run failure.
      2. The ROS examples were never built, because `Examples/ROS` has to be on
         ROS_PACKAGE_PATH at build time. `build.sh` succeeds; `build_ros.sh` is
         a separate script people skip.
      3. The settings file is stale or missing. See make_orbslam3_config.py.

    Each of those gets a specific sentence here instead of a traceback.

    The wrapper also fixes the working directory. ORB-SLAM3 writes
    `CameraTrajectory.txt` / `KeyFrameTrajectory.txt` into whatever directory it
    was started in - which under roslaunch is ~/.ros, where results from
    different runs quietly overwrite each other.

A NOTE ON THE OUTPUT FRAME
    ORB-SLAM3's trajectory is expressed in ITS OWN world frame: the pose of the
    first successfully initialised keyframe, in the camera optical convention
    (x right, y down, z forward). RTAB-Map's is in `map`, in the ROS convention
    (x forward, y left, z up), at the robot base. The two are not directly
    comparable and must not be plotted on the same axes without alignment. The
    evaluation package performs the rigid (SE(3)) alignment; this script only
    records the convention in the sidecar so that alignment is possible at all.

USAGE
    rosrun sidewalk_slam run_orbslam3.py \
        --settings ~/orbslam3_zedx.yaml --run-id 20260720-1400_indoor \
        --mode stereo --camera-name zedx_front
"""
from __future__ import annotations

import argparse
import signal
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from slam_common import (SlamExit, TumWriter, die, make_logger, probe_orbslam3,  # noqa: E402
                         probe_ros, read_tum, trajectory_dir, tum_path)


def parse_args(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--settings", required=True,
                    help="ORB-SLAM3 settings yaml, produced by "
                         "make_orbslam3_config.py")
    ap.add_argument("--run-id", required=True,
                    help="identifier shared by all stacks in one comparison")
    ap.add_argument("--mode", choices=("stereo", "stereo_inertial"),
                    default="stereo")
    ap.add_argument("--orbslam3-root", default=None,
                    help="override the ORB-SLAM3 directory "
                         "(default: auto-detect, normally ~/Developer/ORB_SLAM3)")
    ap.add_argument("--vocabulary", default=None,
                    help="override the vocabulary path")

    ap.add_argument("--camera-name", default="zedx_front")
    ap.add_argument("--node-name", default="zed_node")
    ap.add_argument("--left-topic", default=None)
    ap.add_argument("--right-topic", default=None)
    ap.add_argument("--imu-topic", default=None)

    ap.add_argument("--rectify", action="store_true",
                    help="ask ORB-SLAM3 to rectify the images itself. Do NOT "
                         "use this with the ZED SDK's image_rect_* topics - "
                         "they are already rectified and rectifying twice "
                         "destroys the epipolar geometry.")
    ap.add_argument("--equalize", action="store_true",
                    help="stereo_inertial only: apply CLAHE histogram "
                         "equalisation. Helps in dim indoor corridors, costs CPU.")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="stop after this many wall-clock seconds (0 = run "
                         "until interrupted)")
    ap.add_argument("--dry-run", action="store_true",
                    help="check everything and print the command, but do not "
                         "start ORB-SLAM3")
    return ap.parse_args(argv)


def resolve_topics(args):
    base = f"/{args.camera_name}/{args.node_name}"
    # ORB-SLAM3 works on greyscale internally. Subscribing to the greyscale
    # rectified streams saves a colour->grey conversion and, more importantly,
    # a third of the bandwidth on every frame at 1920x1200.
    left = args.left_topic or f"{base}/left_gray/image_rect_gray"
    right = args.right_topic or f"{base}/right_gray/image_rect_gray"
    imu = args.imu_topic or f"{base}/imu/data"
    return left, right, imu


def preflight(log, args):
    """Everything that can be checked before spending a run. Exits on failure."""
    ros = probe_ros()
    if not ros["present"] and not args.dry_run:
        die(log, "run_orbslam3.py needs a sourced ROS environment.",
            f"{ros['detail']}. Run this on the Jetson.")

    orb = probe_orbslam3(args.orbslam3_root)
    if not orb["present"]:
        die(log, "no ORB-SLAM3 installation was found.",
            "Expected it at ~/Developer/ORB_SLAM3 (that is where the prior work "
            "on this Jetson lives). Pass --orbslam3-root if it is elsewhere.")
    log.info(f"ORB-SLAM3 root: {orb['root']}")

    vocab = args.vocabulary or orb["vocabulary"]
    if not vocab:
        if orb["vocabulary_still_compressed"]:
            die(log, "the ORB vocabulary is still a compressed archive.",
                f"Extract it: cd {orb['root']}/Vocabulary && "
                f"tar -xzf ORBvoc.txt.tar.gz. It expands to about 145 MB. "
                f"Until you do, ORB-SLAM3 will start and then hang or die "
                f"without a useful message.")
        die(log, "ORBvoc.txt was not found.",
            f"Look in {orb['root']}/Vocabulary/. The vocabulary ships with the "
            f"repository; if the directory is empty the clone was shallow or "
            f"git-lfs was not installed.")
    log.info(f"vocabulary: {vocab}")

    settings = Path(args.settings).expanduser()
    if not settings.exists():
        die(log, f"settings file does not exist: {settings}",
            "Generate it from live calibration: "
            "`rosrun sidewalk_slam make_orbslam3_config.py --from-topic "
            "--camera-name " + args.camera_name + " --out " + str(settings) + "`. "
            "Do not copy one from another camera - the intrinsics are per-unit.")

    head = settings.read_text(errors="replace").splitlines()[:1]
    if not head or not head[0].startswith("%YAML"):
        die(log, f"{settings} does not begin with the '%YAML:1.0' directive.",
            "ORB-SLAM3 reads settings with cv::FileStorage, which requires that "
            "directive on the very first line. Regenerate the file rather than "
            "patching it.")

    binary_key = "Stereo" if args.mode == "stereo" else "Stereo_Inertial"
    binary = orb["binaries"].get(binary_key)
    if not binary:
        if not orb["ros_examples_dir"]:
            die(log, f"the ORB-SLAM3 ROS examples directory is missing "
                     f"({orb['root']}/Examples/ROS/ORB_SLAM3).",
                "This ORB-SLAM3 checkout has never had its ROS examples built. "
                "Build them: export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:"
                f"{orb['root']}/Examples/ROS && cd {orb['root']} && "
                "chmod +x build_ros.sh && ./build_ros.sh")
        die(log, f"the '{binary_key}' ROS executable was not built.",
            f"Run ./build_ros.sh in {orb['root']} with "
            f"{orb['root']}/Examples/ROS on ROS_PACKAGE_PATH. If the build "
            f"fails on a missing 'Pangolin' or 'boost_system', those are the "
            f"two dependencies that are usually absent on a fresh JetPack.")
    log.info(f"executable: {binary}")

    if args.mode == "stereo_inertial":
        text = settings.read_text(errors="replace")
        if "IMU.T_b_c1" not in text:
            die(log, "stereo_inertial mode was requested but the settings file "
                     "has no IMU block.",
                "Regenerate it with `make_orbslam3_config.py --imu`.")
        if "IDENTITY FALLBACK" in text:
            log.warn("The settings file records that the camera-IMU extrinsic "
                     "is an IDENTITY FALLBACK, not a real calibration. The "
                     "run will proceed, but any stereo-inertial result from it "
                     "is not publishable. Regenerate the settings with the "
                     "camera running so the TF lookup succeeds.")

    return orb, vocab, binary, settings


def build_command(args, binary, vocab, settings):
    left, right, imu = resolve_topics(args)
    cmd = [binary, str(vocab), str(settings)]
    if args.mode == "stereo":
        # third positional argument is do_rectify
        cmd.append("true" if args.rectify else "false")
    else:
        # third positional argument is do_equalize
        cmd.append("true" if args.equalize else "false")

    # ROS name remapping passed on the command line. The upstream examples
    # hardcode /camera/left/image_raw etc., so every topic has to be remapped.
    cmd += [f"/camera/left/image_raw:={left}",
            f"/camera/right/image_raw:={right}"]
    if args.mode == "stereo_inertial":
        cmd.append(f"/imu:={imu}")
    return cmd, (left, right, imu)


def collect_trajectory(log, args, workdir, topics):
    """Normalise whatever ORB-SLAM3 wrote into the project's canonical file."""
    candidates = ["CameraTrajectory.txt", "KeyFrameTrajectory.txt",
                  "f_dataset.txt", "kf_dataset.txt"]
    found = [workdir / c for c in candidates if (workdir / c).exists()]
    if not found:
        die(log, "ORB-SLAM3 exited without writing a trajectory file.",
            f"Nothing matching {candidates} appeared in {workdir}. The usual "
            f"cause is that the system never initialised: ORB-SLAM3 needs a "
            f"textured, well-lit scene and some parallax before it will start. "
            f"Check its own console output for 'Not enough matches' or "
            f"'Fail to initialize'. Killing it with SIGKILL rather than SIGINT "
            f"also skips the save - always stop it with Ctrl-C.")

    # Prefer the full camera trajectory over the sparser keyframe-only one: the
    # keyframe trajectory is decimated and comparing it against RTAB-Map's dense
    # output would understate ORB-SLAM3's temporal resolution.
    src = None
    for prefer in ("CameraTrajectory.txt", "f_dataset.txt"):
        cand = workdir / prefer
        if cand in found:
            src = cand
            break
    if src is None:
        src = found[0]
        log.warn(f"only the keyframe trajectory was produced ({src.name}). It "
                 f"is decimated relative to the other stacks; note that when "
                 f"comparing pose counts.")
    log.info(f"collecting {src.name}")

    poses, problems = read_tum(src)
    for p in problems[:10]:
        log.warn(f"{src.name}: {p}")
    if len(problems) > 10:
        log.warn(f"{src.name}: ...and {len(problems) - 10} more parse problems")
    if not poses:
        die(log, f"{src} contains no usable poses.",
            "ORB-SLAM3 created the file but wrote nothing to it, which means "
            "tracking never succeeded for a single frame.")

    out = tum_path(args.run_id, "orbslam3")
    with TumWriter(out, stack="orbslam3",
                   source=f"ORB-SLAM3 {args.mode} -> {src.name}",
                   frame_id="orbslam3_world (first keyframe, optical convention)",
                   child_frame_id="left_camera_optical_frame",
                   extra={"run_id": args.run_id,
                          "mode": args.mode,
                          "left_topic": topics[0],
                          "right_topic": topics[1],
                          "imu_topic": topics[2] if args.mode == "stereo_inertial" else None,
                          "raw_file": str(src),
                          "convention_note":
                              "Poses are in ORB-SLAM3's own world frame using "
                              "the camera optical convention (x right, y down, "
                              "z forward), NOT the ROS map frame. SE(3) "
                              "alignment is required before comparing.",
                          "settings_file": str(Path(args.settings).expanduser()),
                          }) as w:
        for p in poses:
            w.add(*p)
        written = w.count
    log.metric("orbslam3_poses", written)
    log.info(f"wrote {out}")
    return out, written


def main(argv=None):
    args = parse_args(argv)
    with make_logger(f"orbslam3_{args.mode}") as log:
        log.section("Preflight")
        orb, vocab, binary, settings = preflight(log, args)

        workdir = trajectory_dir(args.run_id) / "orbslam3_workdir"
        workdir.mkdir(parents=True, exist_ok=True)

        cmd, topics = build_command(args, binary, vocab, settings)
        log.section("Launch")
        log.info(f"working directory: {workdir}")
        log.info("command: " + " ".join(cmd))
        log.info(f"left  : {topics[0]}")
        log.info(f"right : {topics[1]}")
        if args.mode == "stereo_inertial":
            log.info(f"imu   : {topics[2]}")

        if args.dry_run:
            log.summary("Dry run: ORB-SLAM3 preflight passed, nothing launched",
                        status="OK")
            return 0

        # Loading the 145 MB vocabulary takes 15-40 s on the Orin. If images are
        # already flowing during that window they are simply missed, so the
        # runner starts ORB-SLAM3 before it starts the playback.
        log.info("starting ORB-SLAM3 (vocabulary load takes ~20-40 s on the Orin)")
        started = time.time()
        try:
            proc = subprocess.Popen(cmd, cwd=str(workdir))
        except OSError as exc:
            die(log, f"could not start {binary}: {type(exc).__name__}: {exc}",
                "The file exists but is not executable, or was built for a "
                "different architecture. Check with `file " + str(binary) + "`.")

        rc = None
        try:
            if args.duration > 0:
                try:
                    rc = proc.wait(timeout=args.duration)
                except subprocess.TimeoutExpired:
                    log.info(f"--duration {args.duration:.0f}s reached; asking "
                             f"ORB-SLAM3 to shut down and save")
                    # SIGINT, not SIGKILL: the trajectory is written in the
                    # shutdown handler. SIGKILL loses the entire run.
                    proc.send_signal(signal.SIGINT)
                    try:
                        rc = proc.wait(timeout=120)
                    except subprocess.TimeoutExpired:
                        log.warn("ORB-SLAM3 did not exit within 120 s of SIGINT; "
                                 "terminating. The trajectory may be truncated.")
                        proc.terminate()
                        rc = proc.wait(timeout=30)
            else:
                rc = proc.wait()
        except KeyboardInterrupt:
            log.info("interrupted; forwarding SIGINT so the trajectory is saved")
            proc.send_signal(signal.SIGINT)
            try:
                rc = proc.wait(timeout=120)
            except subprocess.TimeoutExpired:
                proc.terminate()
                rc = proc.wait(timeout=30)

        elapsed = time.time() - started
        log.metric("orbslam3_wall_time", round(elapsed, 1), "s")
        log.info(f"ORB-SLAM3 exited with code {rc} after {elapsed:.1f}s")
        if rc not in (0, -2, 130, None):
            log.warn(f"non-zero exit code {rc}. The trajectory is collected "
                     f"anyway if one was written, but treat the run as suspect.")

        log.section("Collecting trajectory")
        out, written = collect_trajectory(log, args, workdir, topics)

        log.summary(f"ORB-SLAM3 {args.mode} baseline finished: {written} poses "
                    f"-> {out.name}", status="OK")
        return 0


if __name__ == "__main__":
    # SlamExit is how every tool in this package reports an already-diagnosed
    # refusal. The reason has been printed and logged by the time it reaches
    # here; all that is left is to hand the shell a non-zero status.
    try:
        sys.exit(main())
    except SlamExit as _exit:
        sys.exit(_exit.code)
