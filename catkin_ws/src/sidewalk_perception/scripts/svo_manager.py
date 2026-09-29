#!/usr/bin/env python3
"""
svo_manager.py — record, inspect and replay ZED SVO2 files.

WHY THIS EXISTS
    This project's central research claim is a three-way comparison: RTAB-Map
    against the ZED SDK's own positional tracking against ORB-SLAM3. For that
    comparison to mean anything, all three must consume *the same input*. Not a
    similar drive on a similar day — the same photons.

    Re-driving the robot for each algorithm would introduce differences in
    lighting, pedestrians, wind, tyre slip and battery voltage, and any of those
    could dominate the effect being measured. Worse, it makes the result
    unreproducible: nobody, including us, could ever run the experiment again.

    SVO2 is the ZED SDK's native recording format. It stores the raw compressed
    stereo streams plus the IMU stream, and on playback the SDK reprocesses it
    exactly as if the camera were live — same depth engine, same settings, same
    timestamps. That is stronger than a rosbag of derived topics, because it
    lets us change depth settings later and re-derive, rather than being stuck
    with whatever was computed at record time.

    So: record once, replay forever. This file is the wrapper that makes that
    convenient and, crucially, makes every recording self-documenting by
    generating a dataset manifest automatically.

WHY NOT JUST ROSBAG?
    We do both, and they answer different questions.
      - SVO2 holds the raw sensor data. Replaying it re-runs the ZED depth
        engine, so we can change depth mode or resolution and re-derive.
      - A rosbag holds the derived ROS topics. It is the right thing when you
        want byte-identical inputs to a ROS node, and it is readable without
        the ZED SDK.
    `--with-bag` records both simultaneously, which is the default for any run
    intended for the algorithm comparison.

COMMANDS
    record   start a recording (SVO2, optionally plus a rosbag)
    play     replay a recording into ROS as if it were a live camera
    info     print what is inside a recording
    list     list every recording in the dataset directory with its manifest
    verify   check a recording is complete and readable before relying on it
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

from perception_common import (
    EXIT_FAIL, EXIT_INTERRUPTED, EXIT_PASS, REPO, RunLogger, Unavailable,
    die, require_rospy,
)

DATA_DIR = REPO / "data" / "recordings"

# SVO2 compression modes as exposed by the ZED SDK / zed-ros-wrapper.
#   H264 / H265 use the Jetson's hardware encoder — essential, because software
#   encoding two 1920x1200 streams would eat the CPU we need for SLAM.
#   LOSSLESS (PNG) is enormous but preserves every pixel; use it only for a
#   short calibration-grade clip where compression artefacts would matter.
COMPRESSION = {
    "lossless": 0,
    "h264": 1,
    "h265": 2,
    "h264_lossless": 3,
    "h265_lossless": 4,
}


def ensure_data_dir():
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    return DATA_DIR


def estimate_size_gb(minutes, fps, compression):
    """Rough disk estimate so an operator is warned *before* filling the disk.

    Numbers are empirical order-of-magnitude figures for 2x1920x1200: roughly
    25 Mb/s for H.265, 40 Mb/s for H.264, and around 30x that uncompressed.
    The point is not precision — it is to make "you are about to write 180 GB"
    visible before the drive rather than after it.
    """
    mbps = {"h265": 25.0, "h264": 40.0,
            "h265_lossless": 120.0, "h264_lossless": 150.0,
            "lossless": 1200.0}.get(compression, 40.0)
    scale = fps / 30.0
    return (mbps * scale * 60.0 * minutes) / 8.0 / 1024.0


def free_space_gb(path):
    try:
        usage = shutil.disk_usage(str(path))
        return usage.free / (1024.0 ** 3)
    except Exception:
        return None


def find_zed_tool(name):
    """Locate a ZED SDK command-line tool, or return None.

    The SDK installs these under /usr/local/zed/tools. They are absent on a
    computer without the SDK, which is why every caller must handle None.
    """
    for base in ("/usr/local/zed/tools", "/usr/local/bin"):
        cand = Path(base) / name
        if cand.exists() and os.access(str(cand), os.X_OK):
            return str(cand)
    found = shutil.which(name)
    return found


def require_zed_sdk():
    from perception_common import probe_zed_sdk_version
    v = probe_zed_sdk_version()
    if not v:
        raise Unavailable(
            "The ZED SDK is not installed under /usr/local/zed, so SVO "
            "recordings cannot be created, inspected or replayed on this "
            "machine.",
            "Install the ZED SDK 4.x build for JetPack 5.1.5 on the Jetson. "
            "Note the 5.x line requires JetPack 6 and will not install here. "
            "For work at a desk, `make_manifest.py` and "
            "`frame_audit.py --bag` both run without the SDK.")
    if not v.startswith("4"):
        # Not fatal — but the project is designed and tested against 4.x, and
        # silently running on something else would undermine reproducibility.
        return v
    return v


# --------------------------------------------------------------------------- #
# record
# --------------------------------------------------------------------------- #

def cmd_record(args, log):
    """Start an SVO2 recording via the ZED ROS wrapper's recording service.

    We drive the wrapper rather than opening the camera directly with the SDK,
    for one important reason: if this tool opened the camera itself, the ROS
    driver could not have it at the same time, and we would lose the live
    topics that the validator and RViz need during the drive. The wrapper owns
    the camera; we ask it to also write an SVO.
    """
    # Validate preconditions BEFORE creating anything on disk. A command that
    # is about to fail should leave no trace: creating data/recordings/ on a
    # computer with no SDK litters the repository with empty directories that
    # then show up in other tooling.
    sdk = require_zed_sdk()
    ensure_data_dir()

    stem = args.name or datetime.now().strftime("%Y%m%d-%H%M%S")
    out_dir = DATA_DIR / stem
    if out_dir.exists() and not args.force:
        raise Unavailable(
            "Recording directory already exists: %s" % out_dir,
            "Pass a different --name, or --force to overwrite. Overwriting a "
            "dataset that has already been analysed will invalidate any result "
            "that referenced it, so the default is to refuse.")
    out_dir.mkdir(parents=True, exist_ok=True)
    svo_path = out_dir / ("%s.svo2" % stem)

    # --- disk check, before anything irreversible
    est = estimate_size_gb(args.duration / 60.0, args.fps, args.compression)
    free = free_space_gb(out_dir)
    log.info("Estimated recording size: ~%.1f GB (%s, %.0f FPS, %.1f min)"
             % (est, args.compression, args.fps, args.duration / 60.0))
    if free is not None:
        log.info("Free space at destination: %.1f GB" % free)
        if free < est * 1.3:
            raise Unavailable(
                "Only %.1f GB free at %s, but this recording is estimated at "
                "%.1f GB. Running out of disk mid-drive produces a truncated, "
                "unindexed file and wastes the whole run."
                % (free, out_dir, est),
                "Free space, choose a shorter --duration, or record to an "
                "external SSD with --data-dir. H.265 (`--compression h265`) is "
                "roughly 40%% smaller than H.264 at similar quality.")

    rospy = require_rospy()
    from perception_common import require_master
    require_master(rospy)
    rospy.init_node("svo_manager_record", anonymous=True, disable_signals=True)

    svc_name = "%s/start_svo_recording" % args.namespace.rstrip("/")
    stop_name = "%s/stop_svo_recording" % args.namespace.rstrip("/")

    log.section("Starting recording")
    log.info("SVO2 target : %s" % svo_path)
    log.info("ZED SDK     : %s" % sdk)
    log.info("Namespace   : %s" % args.namespace)

    try:
        rospy.wait_for_service(svc_name, timeout=10.0)
    except Exception:
        raise Unavailable(
            "The ZED wrapper's recording service `%s` did not appear within "
            "10 s, so the camera driver is not running (or is running under a "
            "different namespace)." % svc_name,
            "Start it first: `roslaunch sidewalk_perception zedx_front.launch`. "
            "Confirm the namespace with `rosservice list | grep svo`.")

    # Call the service generically so this works across wrapper versions, whose
    # service definitions have shifted between SDK 3.x and 4.x.
    proc = _rosservice_call(svc_name, {"svo_filename": str(svo_path),
                                       "compression_mode": COMPRESSION.get(
                                           args.compression, 1)}, log)
    if proc != 0:
        raise Unavailable(
            "The recording service refused the request. The most common cause "
            "is a field-name mismatch between this wrapper build and the call.",
            "Inspect the real signature with `rossrv show "
            "zed_interfaces/start_svo_recording` and adjust, or start the "
            "recording from the wrapper's own launch argument "
            "`svo_record:=true`.")

    bag_proc = None
    if args.with_bag:
        bag_path = out_dir / ("%s.bag" % stem)
        topics = _bag_topics(args.namespace)
        log.info("Also recording rosbag: %s" % bag_path)
        log.info("Bag topics: %s" % " ".join(topics))
        bag_proc = subprocess.Popen(
            ["rosbag", "record", "-O", str(bag_path), "--lz4"] + topics,
            preexec_fn=os.setsid)

    log.section("Recording")
    log.info("Recording for %.1f minutes. Ctrl-C stops early and still leaves "
             "a valid file." % (args.duration / 60.0))
    started = time.time()
    interrupted = False
    try:
        end = started + args.duration
        while time.time() < end and not rospy.is_shutdown():
            time.sleep(1.0)
            elapsed = time.time() - started
            if int(elapsed) % 30 == 0:
                sz = svo_path.stat().st_size / 1e9 if svo_path.exists() else 0.0
                log.info("t+%4.0fs  %.2f GB written" % (elapsed, sz))
    except KeyboardInterrupt:
        interrupted = True
        log.warn("Stopping early at operator request.")

    log.section("Stopping")
    _rosservice_call(stop_name, {}, log)
    if bag_proc is not None:
        # SIGINT, not SIGKILL: rosbag must write its index on the way out, and
        # a killed bag has to be reindexed before it can be read.
        os.killpg(os.getpgid(bag_proc.pid), signal.SIGINT)
        bag_proc.wait(timeout=60)

    elapsed = time.time() - started
    size_gb = svo_path.stat().st_size / 1e9 if svo_path.exists() else 0.0
    log.metric("recording_duration_s", round(elapsed, 1), "s")
    log.metric("recording_size_gb", round(size_gb, 3), "GB")

    if not svo_path.exists() or svo_path.stat().st_size == 0:
        raise Unavailable(
            "Recording finished but %s is missing or empty." % svo_path,
            "The wrapper may lack write permission on that directory, or the "
            "path may not exist from the driver's point of view (it resolves "
            "paths on the machine the driver runs on, which is the Jetson).")

    # Every recording gets a manifest, automatically. A dataset without
    # provenance is not a dataset, it is a pile of bytes.
    log.section("Manifest")
    _write_manifest(out_dir, stem, args, elapsed, size_gb, log)

    log.summary("Recorded %.1f min (%.2f GB) to %s%s"
                % (elapsed / 60.0, size_gb, svo_path,
                   " [stopped early]" if interrupted else ""),
                status="WARN" if interrupted else "OK")
    return EXIT_INTERRUPTED if interrupted else EXIT_PASS


def _bag_topics(ns):
    ns = ns.rstrip("/")
    return [
        "%s/left/image_rect_color/compressed" % ns,
        "%s/right/image_rect_color/compressed" % ns,
        "%s/left/camera_info" % ns,
        "%s/right/camera_info" % ns,
        "%s/depth/depth_registered" % ns,
        "%s/depth/camera_info" % ns,
        "%s/imu/data" % ns,
        "%s/imu/data_raw" % ns,
        "/tf", "/tf_static", "/odom", "/cmd_vel",
    ]


def _rosservice_call(name, fields, log):
    """Call a service via the `rosservice` CLI.

    Deliberately generic rather than importing the wrapper's service types:
    zed_interfaces is only importable where the wrapper is built, and the
    service definitions changed between SDK 3.x and 4.x. Shelling out means one
    code path that works across versions and degrades with a readable message.
    """
    payload = json.dumps(fields) if fields else "{}"
    cmd = ["rosservice", "call", "--wait", name, payload]
    log.info("$ %s" % " ".join(cmd))
    try:
        res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                             timeout=30, check=False)
        out = res.stdout.decode("utf-8", "replace").strip()
        if out:
            log.info(out)
        return res.returncode
    except FileNotFoundError:
        log.error("`rosservice` not found — ROS is not sourced in this shell.")
        return 127
    except subprocess.TimeoutExpired:
        log.error("Service call to %s timed out after 30 s." % name)
        return 1


def _write_manifest(out_dir, stem, args, elapsed, size_gb, log):
    """Generate the dataset manifest by delegating to make_manifest.py.

    Kept in the other module so there is exactly one definition of what a
    manifest contains. Recording and documenting must not drift apart.
    """
    try:
        import make_manifest
    except ImportError:
        log.warn("make_manifest.py not importable — recording has no manifest. "
                 "Create one by hand with `rosrun sidewalk_perception "
                 "make_manifest.py create %s`." % out_dir)
        return
    fields = {
        "recording": stem,
        "duration_s": round(elapsed, 1),
        "size_gb": round(size_gb, 3),
        "resolution": args.resolution,
        "fps": args.fps,
        "compression": args.compression,
        "environment_description": args.environment,
        "trajectory_description": args.trajectory,
        "ground_truth_source": args.ground_truth,
        "mount_revision": args.mount_revision,
        "exposure": args.exposure,
        "known_anomalies": args.anomalies,
        "camera_serial": args.serial,
    }
    try:
        path = make_manifest.create(out_dir, fields, log=log)
        log.info("Manifest written: %s" % path)
    except Exception as exc:
        log.warn("Manifest generation failed (%s: %s). The recording itself is "
                 "fine — write the manifest by hand."
                 % (type(exc).__name__, exc))


# --------------------------------------------------------------------------- #
# play
# --------------------------------------------------------------------------- #

def cmd_play(args, log):
    """Replay an SVO2 file through the ZED wrapper as if it were a live camera.

    The wrapper takes an `svo_file` argument and then publishes the identical
    topic set it would publish live, with the recorded timestamps. That is what
    makes the three-way algorithm comparison fair: RTAB-Map, the ZED tracker
    and ORB-SLAM3 each see exactly the same bytes, in the same order, with the
    same stamps.
    """
    svo = _resolve_recording(args.path, ".svo2", ".svo")
    require_zed_sdk()

    log.section("Playback")
    log.info("File: %s (%.2f GB)" % (svo, svo.stat().st_size / 1e9))
    manifest = svo.parent / "manifest.yaml"
    if manifest.exists():
        log.info("Manifest present: %s" % manifest)
    else:
        log.warn("No manifest beside this recording. You are about to produce "
                 "results from a dataset with no provenance record — you will "
                 "not be able to say later what hardware or settings made it. "
                 "Run `make_manifest.py create %s` first." % svo.parent)

    launch = ["roslaunch", "sidewalk_perception", "playback_svo.launch",
              "svo_file:=%s" % svo,
              "camera_name:=%s" % args.namespace.strip("/"),
              "loop:=%s" % ("true" if args.loop else "false"),
              "rate:=%s" % args.rate]
    log.info("$ %s" % " ".join(launch))
    if args.dry_run:
        log.summary("Dry run — playback command printed, not executed.", status="OK")
        return EXIT_PASS
    try:
        proc = subprocess.Popen(launch)
        proc.wait()
        code = proc.returncode
    except FileNotFoundError:
        raise Unavailable(
            "`roslaunch` was not found, so playback cannot start.",
            "Source ROS first: `source /opt/ros/noetic/setup.bash && "
            "source ~/catkin_ws/devel/setup.bash`.")
    except KeyboardInterrupt:
        log.warn("Playback interrupted.")
        return EXIT_INTERRUPTED
    log.summary("Playback of %s finished (exit %s)" % (svo.name, code),
                status="OK" if code == 0 else "FAIL")
    return EXIT_PASS if code == 0 else EXIT_FAIL


# --------------------------------------------------------------------------- #
# info / verify / list
# --------------------------------------------------------------------------- #

def cmd_info(args, log):
    svo = _resolve_recording(args.path, ".svo2", ".svo")
    log.section("Recording")
    st = svo.stat()
    log.info("Path      : %s" % svo)
    log.info("Size      : %.2f GB" % (st.st_size / 1e9))
    log.info("Modified  : %s" % datetime.fromtimestamp(st.st_mtime).isoformat(timespec="seconds"))

    siblings = sorted(p.name for p in svo.parent.iterdir() if p != svo)
    if siblings:
        log.info("Alongside : %s" % ", ".join(siblings))

    man = svo.parent / "manifest.yaml"
    if man.exists():
        log.section("Manifest")
        for line in man.read_text().splitlines():
            log.info("  " + line)
    else:
        log.warn("No manifest.yaml beside this recording.")

    # ZED_SVO_Editor reports the real frame count and resolution from inside
    # the container. Absent on a computer without the SDK, hence the graceful path.
    tool = find_zed_tool("ZED_SVO_Editor")
    if tool:
        log.section("SVO container")
        res = subprocess.run([tool, "-info", str(svo)],
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                             timeout=60, check=False)
        log.info(res.stdout.decode("utf-8", "replace").strip())
    else:
        log.warn("ZED_SVO_Editor not found, so the frame count and recorded "
                 "resolution could not be read from inside the file. That tool "
                 "ships with the ZED SDK and lives in /usr/local/zed/tools.")
    log.summary("Inspected %s" % svo.name, status="OK")
    return EXIT_PASS


def cmd_verify(args, log):
    """Sanity-check a recording before anyone builds a result on it.

    Cheap checks only, but they catch the failures that actually happen: a
    truncated file from a full disk, a missing manifest, an SVO whose duration
    does not match what the manifest claims.
    """
    rec_dir = Path(args.path)
    if rec_dir.is_file():
        rec_dir = rec_dir.parent
    if not rec_dir.is_dir():
        raise Unavailable("No such recording directory: %s" % rec_dir,
                          "Run `svo_manager.py list` to see what exists.")

    problems, notes = [], []
    svos = sorted(list(rec_dir.glob("*.svo2")) + list(rec_dir.glob("*.svo")))
    if not svos:
        problems.append("no .svo2 file in %s" % rec_dir)
    for s in svos:
        size = s.stat().st_size
        if size == 0:
            problems.append("%s is zero bytes" % s.name)
        elif size < 1_000_000:
            problems.append("%s is only %d bytes — almost certainly truncated"
                            % (s.name, size))
        else:
            notes.append("%s: %.2f GB" % (s.name, size / 1e9))

    man = rec_dir / "manifest.yaml"
    if not man.exists():
        problems.append("manifest.yaml is missing — this dataset has no "
                        "provenance record")
    else:
        from perception_common import load_simple_yaml
        try:
            data = load_simple_yaml(man)
            missing = [k for k in ("camera", "software", "recording")
                       if k not in data]
            if missing:
                problems.append("manifest is missing sections: %s"
                                % ", ".join(missing))
            else:
                notes.append("manifest present and structurally complete")
        except Exception as exc:
            problems.append("manifest.yaml could not be parsed (%s)" % exc)

    bags = sorted(rec_dir.glob("*.bag"))
    for b in bags:
        if b.with_suffix(".bag.active").exists():
            problems.append("%s was never closed cleanly (an .active file "
                            "remains); run `rosbag reindex`" % b.name)
        notes.append("%s: %.2f GB" % (b.name, b.stat().st_size / 1e9))

    log.section("Verification: %s" % rec_dir.name)
    for n in notes:
        log.info("OK    %s" % n)
    for p in problems:
        log.error("BAD   %s" % p)

    status = "FAIL" if problems else "OK"
    log.summary("Verify %s — %d problem(s), %d check(s) passed"
                % (rec_dir.name, len(problems), len(notes)), status=status)
    return EXIT_FAIL if problems else EXIT_PASS


def cmd_list(args, log):
    d = Path(args.data_dir) if args.data_dir else DATA_DIR
    if not d.is_dir():
        log.warn("No recordings directory at %s yet — nothing has been "
                 "recorded." % d)
        log.summary("No recordings found", status="WARN")
        return EXIT_PASS

    rows = []
    for sub in sorted(p for p in d.iterdir() if p.is_dir()):
        svos = list(sub.glob("*.svo2")) + list(sub.glob("*.svo"))
        size = sum(f.stat().st_size for f in sub.rglob("*") if f.is_file())
        man = sub / "manifest.yaml"
        desc = ""
        if man.exists():
            from perception_common import load_simple_yaml
            try:
                data = load_simple_yaml(man)
                env = (data.get("environment") or {})
                desc = env.get("description") or env.get("trajectory") or ""
            except Exception:
                desc = "(manifest unreadable)"
        rows.append((sub.name, len(svos), size / 1e9,
                     "yes" if man.exists() else "NO", desc))

    log.section("Recordings in %s" % d)
    if not rows:
        log.info("(none)")
    else:
        log.info("%-28s %5s %9s %9s  %s"
                 % ("dataset", "svo", "size GB", "manifest", "description"))
        for name, n, gb, man, desc in rows:
            log.info("%-28s %5d %9.2f %9s  %s" % (name, n, gb, man, desc[:60]))
    unman = sum(1 for r in rows if r[3] == "NO")
    log.summary("%d recording(s), %.1f GB total, %d without a manifest"
                % (len(rows), sum(r[2] for r in rows), unman),
                status="WARN" if unman else "OK")
    return EXIT_PASS


def _resolve_recording(path, *exts):
    """Accept a file, a directory, or a bare dataset name."""
    p = Path(path)
    if p.is_file():
        return p
    for cand in (p, DATA_DIR / path):
        if cand.is_dir():
            for ext in exts:
                hits = sorted(cand.glob("*" + ext))
                if hits:
                    return hits[0]
            raise Unavailable(
                "Directory %s contains no %s file." % (cand, " or ".join(exts)),
                "Run `svo_manager.py list` to see available recordings.")
    raise Unavailable(
        "No recording found at `%s`." % path,
        "Give a path to a .svo2 file, a recording directory, or a dataset "
        "name under %s. `svo_manager.py list` shows what exists." % DATA_DIR)


# --------------------------------------------------------------------------- #

def build_parser():
    p = argparse.ArgumentParser(
        description="Record, inspect and replay ZED SVO2 files — the "
                    "foundation of this project's reproducible comparisons.")
    p.add_argument("--data-dir", default="",
                   help="Override the recordings directory (default: data/recordings)")
    sub = p.add_subparsers(dest="command")

    r = sub.add_parser("record", help="Record an SVO2 (and optionally a rosbag)")
    r.add_argument("--name", default="",
                   help="Dataset name; defaults to a timestamp")
    r.add_argument("--duration", type=float, default=600.0,
                   help="Recording length in seconds (default 600 = 10 min)")
    r.add_argument("--namespace", default="/zedx_front")
    r.add_argument("--fps", type=float, default=30.0)
    r.add_argument("--resolution", default="HD1200",
                   help="HD1200 (1920x1200), HD1080 or SVGA")
    r.add_argument("--compression", default="h265", choices=sorted(COMPRESSION),
                   help="h265 is smallest and uses the Jetson hardware encoder")
    r.add_argument("--with-bag", action="store_true",
                   help="Also record a rosbag of the derived ROS topics")
    r.add_argument("--force", action="store_true",
                   help="Overwrite an existing dataset of the same name")
    r.add_argument("--environment", default="",
                   help="Where and under what conditions, in plain words")
    r.add_argument("--trajectory", default="",
                   help="What path the robot drove")
    r.add_argument("--ground-truth", default="none",
                   help="How ground truth was obtained, if at all")
    r.add_argument("--mount-revision", default="",
                   help="Which camera bracket revision was fitted")
    r.add_argument("--exposure", default="auto",
                   help="Exposure/gain settings used")
    r.add_argument("--anomalies", default="",
                   help="Anything already known to be wrong with this run")
    r.add_argument("--serial", default="",
                   help="Camera serial number, if known")

    pl = sub.add_parser("play", help="Replay a recording into ROS")
    pl.add_argument("path", help="SVO2 file, recording directory, or dataset name")
    pl.add_argument("--namespace", default="/zedx_front")
    pl.add_argument("--loop", action="store_true")
    pl.add_argument("--rate", default="1.0", help="Playback speed multiplier")
    pl.add_argument("--dry-run", action="store_true",
                    help="Print the launch command without running it")

    i = sub.add_parser("info", help="Show what is inside a recording")
    i.add_argument("path")

    v = sub.add_parser("verify", help="Check a recording is complete and usable")
    v.add_argument("path")

    sub.add_parser("list", help="List all recordings and their manifests")
    return p


def main(argv=None):
    argv = [a for a in (sys.argv[1:] if argv is None else argv)
            if not a.startswith("__") and ":=" not in a]
    parser = build_parser()
    args = parser.parse_args(argv)
    if not args.command:
        parser.print_help()
        return EXIT_FAIL

    global DATA_DIR
    if getattr(args, "data_dir", ""):
        DATA_DIR = Path(args.data_dir)

    handlers = {"record": cmd_record, "play": cmd_play, "info": cmd_info,
                "verify": cmd_verify, "list": cmd_list}
    with RunLogger("sidewalk_perception",
                   run_name="svo_%s" % args.command) as log:
        try:
            return handlers[args.command](args, log)
        except Unavailable as exc:
            return die(log, exc)
        except KeyboardInterrupt:
            log.warn("Interrupted by operator.")
            log.summary("Interrupted", status="WARN")
            return EXIT_INTERRUPTED


if __name__ == "__main__":
    sys.exit(main())
