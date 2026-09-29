#!/usr/bin/env python3
"""
slam_runner.py — run RTAB-Map, the ZED SDK tracker and ORB-SLAM3 over the SAME
recorded input and emit three trajectories in one common format.

WHY THIS EXISTS
    The core research contribution of this project is a three-way comparison. A
    comparison is only worth reading if the three systems were given identical
    input, and "identical" is much harder than it sounds:

      * Identical DATA. Not "the same route walked three times" - the same
        recorded file, replayed. A sidewalk at 14:05 is not the sidewalk at
        14:20; the light has moved, the pedestrians have moved, and any
        difference you then measure is partly the world changing.
      * Identical TIME. Replay must publish simulated time (`rosbag play
        --clock`) so every stack stamps its output with the recorded time. Then
        pose N of one trajectory can be compared against pose N of another.
      * Identical CONDITIONS. The stacks run one after another, never together,
        so none of them is competing for CPU with the others. See the long note
        in config/slam_runner.yaml.

    Doing that by hand, three times, without drift in the procedure, is not
    realistic. Hence one script that does it the same way every time and writes
    down what it did.

WHAT IT PRODUCES
    logs/sidewalk_slam/trajectories/<run_id>/
        rtabmap_trajectory.tum       + .meta.json
        zed_sdk_trajectory.tum       + .meta.json
        orbslam3_trajectory.tum      + .meta.json
        run_manifest.json            what was run, with what, and how it went

    The evaluation package consumes exactly that directory.

USAGE
    # See the plan without running anything (works on any machine):
    rosrun sidewalk_slam slam_runner.py --input ~/data/indoor_loop.bag --dry-run

    # Do it for real, on the Jetson:
    rosrun sidewalk_slam slam_runner.py \
        --input ~/data/indoor_loop.bag --input-type bag \
        --settings ~/orbslam3_zedx.yaml --execute
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from slam_common import (SlamExit, STACKS, die, make_logger, map_dir, new_run_id,  # noqa: E402
                         probe_ros, read_tum, trajectory_dir, tum_path)

PKG_DIR = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG = PKG_DIR / "config" / "slam_runner.yaml"


def parse_args(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--input", required=True,
                    help="the recording every stack will be run over "
                         "(a .bag or a .svo2)")
    ap.add_argument("--input-type", choices=("auto", "bag", "svo"), default="auto")
    ap.add_argument("--run-id", default=None,
                    help="identifier for this comparison (default: timestamped)")
    ap.add_argument("--config", default=str(DEFAULT_CONFIG),
                    help="runner configuration (default: the packaged one)")
    ap.add_argument("--stacks", default=",".join(STACKS),
                    help=f"comma-separated subset to run "
                         f"(default: {','.join(STACKS)})")
    ap.add_argument("--camera-name", default="zedx_front")
    ap.add_argument("--node-name", default="zed_node")
    ap.add_argument("--settings", default=None,
                    help="ORB-SLAM3 settings file. Required if the orbslam3 "
                         "stack is enabled.")
    ap.add_argument("--rate", type=float, default=1.0,
                    help="playback rate multiplier. Values above 1.0 make the "
                         "comparison meaningless - the stacks start dropping "
                         "frames at different points. Below 1.0 is legitimate "
                         "and useful: it removes real-time pressure so you are "
                         "measuring the algorithms, not the hardware.")
    ap.add_argument("--timeout", type=float, default=3600.0,
                    help="hard limit per stack, in seconds (default: 3600)")
    ap.add_argument("--execute", action="store_true",
                    help="actually run. Without it the runner prints the plan "
                         "and exits, which is the only thing possible on a "
                         "machine with no ROS.")
    ap.add_argument("--dry-run", action="store_true",
                    help="explicit alias for the default plan-only behaviour")
    return ap.parse_args(argv)


def load_config(log, path):
    p = Path(path).expanduser()
    if not p.exists():
        die(log, f"runner configuration not found: {p}",
            "The packaged default lives at "
            "sidewalk_slam/config/slam_runner.yaml. If it is missing, the "
            "package was installed without its config directory.")
    try:
        import yaml
    except ImportError as exc:
        die(log, f"PyYAML is not available: {exc}",
            "Install it with `sudo apt install python3-yaml`. It ships with "
            "ROS Noetic, so on the Jetson this means ROS is not sourced.")
    try:
        cfg = yaml.safe_load(p.read_text())
    except Exception as exc:
        die(log, f"could not parse {p}: {type(exc).__name__}: {exc}",
            "Check the indentation. YAML is whitespace-sensitive and a tab "
            "character anywhere in the file will do this.")
    if not isinstance(cfg, dict) or "stacks" not in cfg:
        die(log, f"{p} does not look like a runner configuration.",
            "It must contain top-level `input:` and `stacks:` sections.")
    return cfg


def detect_input_type(log, args):
    p = Path(args.input).expanduser()
    if not p.exists():
        die(log, f"input recording does not exist: {p}",
            "Recordings are produced in Phase 2 by sidewalk_perception "
            "(svo_manager.py for SVO2, rosbag record for bags). Check the "
            "dataset manifest for the exact path.")
    if args.input_type != "auto":
        return p, args.input_type
    suffix = p.suffix.lower()
    if suffix == ".bag":
        return p, "bag"
    if suffix in (".svo", ".svo2"):
        return p, "svo"
    die(log, f"cannot tell what kind of recording {p.name} is.",
        "Pass --input-type bag or --input-type svo explicitly.")


def substitute(template, subs):
    """Fill {placeholders}. Missing keys are left alone rather than raising, so
    an unrecognised placeholder shows up in the printed command instead of
    crashing the runner three stacks into a two-hour experiment."""
    out = template
    for k, v in subs.items():
        out = out.replace("{" + k + "}", str(v))
    return out


class Managed:
    """A child process the runner is responsible for stopping.

    Every child is started in its own process GROUP. roslaunch spawns node
    processes as children of itself; killing only roslaunch leaves those
    orphaned, still holding the ROS master's node registry and still bound to
    their ports, and the next stack then fails with a name collision that looks
    like a totally unrelated bug. Signalling the whole group avoids that.
    """

    def __init__(self, log, name, cmd, cwd=None, env=None):
        self.log = log
        self.name = name
        self.cmd = cmd
        self.proc = None
        self.started = None
        self._cwd = cwd
        self._env = env

    def start(self):
        self.log.info(f"start [{self.name}]: {self.cmd}")
        self.started = time.time()
        # Output is deliberately NOT captured: each stack's own console output
        # (RTAB-Map's warnings, ORB-SLAM3's tracking messages) is the first
        # thing you need when a run goes wrong, and swallowing it into a pipe
        # that nobody reads is how a run becomes undiagnosable.
        self.proc = subprocess.Popen(
            ["bash", "-lc", self.cmd],
            cwd=self._cwd, env=self._env,
            start_new_session=True)          # own process group
        return self

    def poll(self):
        return None if self.proc is None else self.proc.poll()

    def wait(self, timeout):
        if self.proc is None:
            return 0
        try:
            return self.proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            return None

    def stop(self, grace=20.0):
        """SIGINT the group, then escalate. Never SIGKILL first.

        RTAB-Map writes its database and ORB-SLAM3 writes its trajectory in
        their shutdown handlers. SIGKILL loses both, i.e. loses the entire run.
        """
        if self.proc is None or self.proc.poll() is not None:
            return
        pgid = os.getpgid(self.proc.pid)
        self.log.info(f"stop  [{self.name}]: SIGINT to process group {pgid}")
        try:
            os.killpg(pgid, signal.SIGINT)
        except (ProcessLookupError, PermissionError) as exc:
            self.log.warn(f"could not signal {self.name}: {exc}")
            return
        try:
            self.proc.wait(timeout=grace)
            return
        except subprocess.TimeoutExpired:
            pass
        self.log.warn(f"[{self.name}] ignored SIGINT for {grace:.0f}s; "
                      f"escalating to SIGTERM. Its output may be incomplete.")
        try:
            os.killpg(pgid, signal.SIGTERM)
            self.proc.wait(timeout=10)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            self.log.warn(f"[{self.name}] escalating to SIGKILL.")
            try:
                os.killpg(pgid, signal.SIGKILL)
            except ProcessLookupError:
                pass


def ensure_roscore(log):
    """Start a master if none is reachable, and report which one we are using."""
    rc = subprocess.run(["bash", "-lc", "rostopic list >/dev/null 2>&1"],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if rc.returncode == 0:
        log.info(f"using the existing ROS master at "
                 f"{os.environ.get('ROS_MASTER_URI', '(default)')}")
        return None
    log.info("no ROS master reachable; starting one")
    core = Managed(log, "roscore", "roscore").start()
    for _ in range(60):
        time.sleep(0.5)
        probe = subprocess.run(["bash", "-lc", "rostopic list >/dev/null 2>&1"],
                               stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL)
        if probe.returncode == 0:
            log.info("ROS master is up")
            return core
    core.stop()
    die(log, "roscore did not come up within 30 s.",
        "Check for a stale master: `ps aux | grep rosmaster`. Also check "
        "ROS_MASTER_URI - if it points at the robot at <old-robot-address> and that "
        "machine is offline, every ROS command will hang exactly like this.")


def recorder_command(stack, spec, subs, run_id):
    """Build the trajectory_recorder invocation for one stack."""
    source = (spec or {}).get("source", "none")
    if source == "none":
        return None
    base = (f"rosrun sidewalk_slam trajectory_recorder.py "
            f"--stack {stack} --run-id {run_id} "
            f"--input-name {subs['input_name']}")
    if source == "tf":
        return (f"{base} --source tf "
                f"--fixed-frame {spec.get('fixed_frame', 'map')} "
                f"--body-frame {spec.get('body_frame', 'base_link')} "
                f"--rate {spec.get('rate', 20.0)}")
    if source == "topic":
        topic = substitute(str(spec.get("topic", "")), subs)
        return (f"{base} --source topic --topic {topic} "
                f"--type {spec.get('type', 'odom')}")
    return None


def run_one_stack(log, args, cfg, stack, subs, run_id, input_type):
    """Warm up the stack, replay the recording, shut everything down cleanly."""
    spec = cfg["stacks"][stack]
    label = spec.get("label", stack)
    log.section(f"Stack: {label}")

    play_tmpl = (cfg["input"]["bag_play_cmd"] if input_type == "bag"
                 else cfg["input"]["svo_play_cmd"])
    play_cmd = substitute(play_tmpl, subs)
    launch_cmd = substitute(spec.get("launch", "") or "", subs)
    rec_cmd = recorder_command(stack, spec.get("recorder"), subs, run_id)

    result = {"stack": stack, "label": label, "launch": launch_cmd,
              "playback": play_cmd, "recorder": rec_cmd,
              "status": "not_run", "wall_time_s": None,
              "trajectory": str(tum_path(run_id, stack))}

    started = time.time()
    children = []
    try:
        # Simulated time. Set BEFORE any node starts: a node that has already
        # latched wall time will not switch over, and you get a trajectory whose
        # stamps are today's date rather than the recording's.
        subprocess.run(["bash", "-lc", "rosparam set /use_sim_time true"],
                       check=False)

        if launch_cmd:
            children.append(Managed(log, f"{stack}:stack", launch_cmd).start())
            warm = float(spec.get("warmup_s", 5.0))
            log.info(f"warming up for {warm:.0f}s before playback starts")
            time.sleep(warm)
            dead = [c for c in children if c.poll() is not None]
            if dead:
                result["status"] = "stack_died_during_warmup"
                log.error(f"[{stack}] the stack exited during warmup "
                          f"(exit code {dead[0].poll()}).")
                log.error("Nothing was replayed. Read the stack's own output "
                          "above this line - it failed before seeing any data, "
                          "so this is a configuration or build problem, not a "
                          "SLAM problem.")
                return result

        if rec_cmd:
            children.append(Managed(log, f"{stack}:recorder", rec_cmd).start())
            time.sleep(2.0)

        log.info("starting playback")
        play = Managed(log, f"{stack}:playback", play_cmd).start()
        children.append(play)

        rc = play.wait(timeout=args.timeout)
        if rc is None:
            result["status"] = "timeout"
            log.error(f"playback did not finish within {args.timeout:.0f}s.")
            log.error("Either the recording is longer than the timeout, or "
                      "playback stalled waiting for a subscriber. Raise "
                      "--timeout, or check `rosbag info` for the true length.")
        else:
            log.info(f"playback finished (exit code {rc})")
            cooldown = float(cfg["input"].get("cooldown_s", 10.0))
            log.info(f"cooling down {cooldown:.0f}s so the last frames are "
                     f"processed and the final graph optimisation completes")
            time.sleep(cooldown)
            result["status"] = "played"

    finally:
        # Stop in reverse order: playback first, then the recorder (so it
        # captures everything the stack emits during its own shutdown), then
        # the stack itself.
        for child in reversed(children):
            child.stop()
        result["wall_time_s"] = round(time.time() - started, 1)

    traj = tum_path(run_id, stack)
    poses, problems = read_tum(traj)
    result["poses"] = len(poses)
    result["parse_problems"] = len(problems)
    if not poses:
        result["status"] = "no_trajectory"
        log.error(f"[{stack}] produced no trajectory.")
        log.error("The stack ran and the recording was replayed, but not one "
                  "pose came out. For RTAB-Map check that odometry ever "
                  "initialised; for ORB-SLAM3 check its console for "
                  "initialisation failures; for the ZED SDK check that "
                  "positional tracking was actually enabled in the wrapper.")
    else:
        span = poses[-1][0] - poses[0][0]
        log.metric(f"{stack}_poses", len(poses))
        log.metric(f"{stack}_trajectory_span", round(span, 2), "s")
        result["duration_s"] = round(span, 3)
        if result["status"] == "played":
            result["status"] = "ok"
    return result


def print_plan(log, args, cfg, subs, stacks, input_path, input_type, run_id):
    log.section("Plan")
    log.info(f"run id      : {run_id}")
    log.info(f"input       : {input_path} ({input_type})")
    log.info(f"playback    : rate x{args.rate}")
    log.info(f"output dir  : {trajectory_dir(run_id)}")
    log.info("")
    play_tmpl = (cfg["input"]["bag_play_cmd"] if input_type == "bag"
                 else cfg["input"]["svo_play_cmd"])
    for stack in stacks:
        spec = cfg["stacks"][stack]
        log.info(f"--- {spec.get('label', stack)}")
        launch = substitute(spec.get("launch", "") or "", subs)
        log.info(f"    launch  : {launch or '(none - runs inside the camera stack)'}")
        rec = recorder_command(stack, spec.get("recorder"), subs, run_id)
        log.info(f"    record  : {rec or '(the stack writes its own trajectory)'}")
        log.info(f"    warmup  : {spec.get('warmup_s', 5.0)} s")
        log.info(f"    replay  : {substitute(play_tmpl, subs)}")
        log.info(f"    output  : {tum_path(run_id, stack)}")


def main(argv=None):
    args = parse_args(argv)
    run_id = args.run_id or new_run_id("compare")

    with make_logger(f"slam_runner_{run_id}") as log:
        cfg = load_config(log, args.config)
        input_path, input_type = detect_input_type(log, args)

        requested = [s.strip() for s in args.stacks.split(",") if s.strip()]
        unknown = [s for s in requested if s not in cfg["stacks"]]
        if unknown:
            die(log, f"unknown stack(s): {', '.join(unknown)}",
                f"The configuration defines: {', '.join(cfg['stacks'])}.")
        stacks = [s for s in requested
                  if cfg["stacks"][s].get("enabled", True)]
        disabled = [s for s in requested if s not in stacks]
        for s in disabled:
            log.warn(f"stack '{s}' is disabled in {args.config}; skipping")
        if not stacks:
            die(log, "no stacks are enabled.",
                "Enable at least one in the runner configuration, or pass "
                "--stacks with a stack that is enabled.")

        db = map_dir() / f"{run_id}_rtabmap.db"
        settings = args.settings or ""
        if "orbslam3" in stacks and not settings:
            die(log, "the orbslam3 stack is enabled but --settings was not given.",
                "Generate a settings file first: `rosrun sidewalk_slam "
                "make_orbslam3_config.py --from-bag " + str(input_path) +
                " --out ~/orbslam3_zedx.yaml`, then pass --settings. The "
                "intrinsics must come from the same camera that made this "
                "recording.")

        subs = {
            "input": str(input_path),
            "input_name": input_path.name,
            "rate": args.rate,
            "run_id": run_id,
            "camera_name": args.camera_name,
            "node_name": args.node_name,
            "settings": settings,
            "db": str(db),
            "pkg": str(PKG_DIR),
        }

        print_plan(log, args, cfg, subs, stacks, input_path, input_type, run_id)

        if not args.execute or args.dry_run:
            log.info("")
            log.info("This was a plan only. Add --execute to run it.")
            log.summary(f"Plan generated for run {run_id} over "
                        f"{input_path.name} ({len(stacks)} stacks)", status="OK")
            return 0

        ros = probe_ros()
        if not ros["present"]:
            die(log, "--execute was requested but there is no ROS environment.",
                f"{ros['detail']}. This authoring machine has no ROS installed. "
                f"Run the comparison on the Jetson.")

        # Only now, when something is actually going to be written, do the
        # output directories get created. A dry run leaves no trace.
        trajectory_dir(run_id, create=True)
        map_dir(create=True)

        core = ensure_roscore(log)
        results = []
        try:
            for stack in stacks:
                results.append(run_one_stack(log, args, cfg, stack, subs,
                                             run_id, input_type))
        finally:
            if core is not None:
                core.stop()

        manifest = {
            "run_id": run_id,
            "input": str(input_path),
            "input_type": input_type,
            "playback_rate": args.rate,
            "camera_name": args.camera_name,
            "orbslam3_settings": settings,
            "rtabmap_database": str(db),
            "trajectory_format": "TUM: timestamp tx ty tz qx qy qz qw",
            "note": ("Stacks were run SEQUENTIALLY over identical recorded "
                     "input with simulated time, so their timestamps are "
                     "directly comparable. Reference frames are NOT the same "
                     "across stacks - see each trajectory's .meta.json."),
            "stacks": results,
        }
        mpath = trajectory_dir(run_id, create=True) / "run_manifest.json"
        mpath.write_text(json.dumps(manifest, indent=2))
        log.info(f"manifest written to {mpath}")

        log.section("Result")
        ok = [r for r in results if r["status"] == "ok"]
        for r in results:
            log.info(f"  {r['stack']:10s} {r['status']:24s} "
                     f"{r.get('poses', 0):7d} poses  "
                     f"{r['wall_time_s']:.0f}s")
        log.metric("stacks_succeeded", len(ok))
        log.metric("stacks_attempted", len(results))

        status = "OK" if len(ok) == len(results) else (
            "FAIL" if not ok else "WARN")
        log.summary(
            f"Three-way comparison run {run_id} over {input_path.name}: "
            f"{len(ok)}/{len(results)} stacks produced trajectories",
            status=status)
        return 0 if ok else 3


if __name__ == "__main__":
    # SlamExit is how every tool in this package reports an already-diagnosed
    # refusal. The reason has been printed and logged by the time it reaches
    # here; all that is left is to hand the shell a non-zero status.
    try:
        sys.exit(main())
    except SlamExit as _exit:
        sys.exit(_exit.code)
