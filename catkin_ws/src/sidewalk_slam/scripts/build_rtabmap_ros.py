#!/usr/bin/env python3
"""
build_rtabmap_ros.py — plan (and optionally execute) a build of rtabmap_ros
against the RTAB-Map core library that is ALREADY installed on the Jetson.

WHY THIS EXISTS
    The Jetson already has RTAB-Map core 0.21.10 built from source and installed
    into /usr/local (`/usr/local/lib/librtabmap_core.so.0.21.10`, sources under
    ~/Developer/rtabmap). What it does not have is the ROS layer: only
    `rtabmap_msgs` is built, so `rtabmap_slam`, `rtabmap_odom` and `rtabmap_sync`
    - the nodes this whole phase depends on - are missing.

    The tempting one-liner is:

        sudo apt install ros-noetic-rtabmap-ros

    That is the wrong move, and this script exists mainly to stop it happening.
    The apt package drags in `ros-noetic-rtabmap`, a SECOND copy of the core
    library, into /opt/ros/noetic/lib. You then have two `librtabmap_core.so`
    files of different versions on the loader path. The link succeeds. The
    symbols resolve. And at runtime you get a database schema mismatch or a
    silent segfault inside the memory manager, hours later, in the middle of a
    mapping run. Diagnosing that costs a week.

    So: DETECT the existing installation, build the ROS layer against it, and
    check the versions match. Never install a second core.

    Note also that a partially-built workspace is its own hazard. If
    `rtabmap_msgs` was built from a different rtabmap_ros checkout than the one
    you are about to add, the generated message headers and the new sources
    disagree, and the error appears in a file you never touched.

WHAT THIS SCRIPT DOES AND DOES NOT DO
    By default it only INSPECTS and PRINTS. It writes a ready-to-run shell
    script into logs/sidewalk_slam/ and tells you to read it before running it.
    Compiling a large C++ tree is not something a helper script should do behind
    your back, and this authoring machine has no ROS at all.

    Pass --execute (on the Jetson, with ROS sourced) to actually run the plan.

USAGE
    python3 build_rtabmap_ros.py                     # inspect + write the plan
    python3 build_rtabmap_ros.py --workspace ~/catkin_ws
    python3 build_rtabmap_ros.py --execute           # actually build (Jetson)
"""
from __future__ import annotations

import argparse
import os
import shutil
import stat
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from slam_common import (SlamExit, make_logger, probe_ros, probe_rtabmap_core,  # noqa: E402
                         probe_rtabmap_ros, run_cmd, slam_log_dir, die)

# The rtabmap_ros release must match the core library release. Upstream tags the
# two repositories in lockstep; mixing 0.21.10 core with a newer rtabmap_ros HEAD
# is a supported-looking configuration that is not actually supported.
DEFAULT_REPO = "https://github.com/introlab/rtabmap_ros.git"

# Sub-packages this project actually needs. rtabmap_viz and the rviz plugins are
# optional on a headless robot but useful on the desk, so they are listed
# separately rather than dropped.
REQUIRED_PKGS = ["rtabmap_msgs", "rtabmap_conversions", "rtabmap_util",
                 "rtabmap_sync", "rtabmap_odom", "rtabmap_slam",
                 "rtabmap_launch"]
OPTIONAL_PKGS = ["rtabmap_viz", "rtabmap_rviz_plugins", "rtabmap_python",
                 "rtabmap_demos", "rtabmap_examples"]


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--workspace", default="~/catkin_ws",
                    help="catkin workspace root on the Jetson (default: ~/catkin_ws)")
    ap.add_argument("--repo", default=DEFAULT_REPO,
                    help="rtabmap_ros git repository URL")
    ap.add_argument("--jobs", type=int, default=4,
                    help="parallel compile jobs. Keep this LOW on the Orin: "
                         "each g++ process on this tree peaks around 2 GB, and "
                         "-j8 on 29 GB of RAM with a desktop running will OOM. "
                         "(default: 4)")
    ap.add_argument("--execute", action="store_true",
                    help="actually run the build. Without this the script only "
                         "inspects and writes the plan.")
    ap.add_argument("--allow-version-mismatch", action="store_true",
                    help="proceed even if the rtabmap_ros tag does not match "
                         "the installed core version. You almost never want this.")
    return ap.parse_args(argv)


def core_version_tag(version):
    """Turn a SONAME version like '0.21.10' into the matching git tag.

    Upstream tags releases as plain '0.21.10'. Some SONAMEs carry only the
    major.minor, in which case we cannot pin exactly and say so.
    """
    if not version:
        return None
    parts = version.split(".")
    if len(parts) >= 3:
        return ".".join(parts[:3])
    return None


def inspect(log, args):
    """Gather the full picture before proposing anything."""
    ros = probe_ros()
    core = probe_rtabmap_core()
    ros_pkgs = probe_rtabmap_ros()

    log.section("1. ROS environment")
    log.info(ros["detail"])
    if not ros["present"]:
        log.warn("No ROS environment is sourced in this shell.")
        log.warn("That is expected on a computer without ROS.")
        log.warn("On the Jetson, run: source /opt/ros/noetic/setup.bash")

    log.section("2. Installed RTAB-Map core library")
    if core["present"]:
        for lib in core["libraries"]:
            log.info(f"found {lib['path']}  (version {lib['version']})")
        log.metric("rtabmap_core_version", core["version"])
        log.info(f"CMake package config: {core['cmake_config'] or 'NOT FOUND'}")
        if not core["cmake_config"]:
            log.warn("RTABMapConfig.cmake was not found. Without it, "
                     "find_package(RTABMap) cannot locate the existing install "
                     "and CMake will fall back to whatever else is on the "
                     "prefix path - which is exactly the duplication we are "
                     "trying to avoid.")
        cli = [k for k, v in core["tools"].items() if v]
        log.info(f"command-line tools on PATH: {', '.join(cli) if cli else '(none)'}")
    else:
        log.warn("librtabmap_core.so.* was NOT found on this machine.")
        log.warn("On a computer without ROS this is expected. On the Jetson it "
                 "would mean the core install has been removed - stop and "
                 "investigate before building anything.")

    log.section("3. rtabmap_ros packages already present")
    for pkg, where in ros_pkgs["packages"].items():
        mark = "built/found" if where else "MISSING"
        log.info(f"  {pkg:24s} {mark}{('  ' + where) if where else ''}")
    if ros_pkgs["partial"]:
        log.warn("The workspace is PARTIALLY populated with rtabmap_ros "
                 "packages. This is the documented state of the Jetson: "
                 "rtabmap_msgs exists, the rest do not.")
        log.warn("The fix is to add the remaining packages from the SAME "
                 "checkout at the SAME tag. Do not clone a second, newer copy "
                 "alongside the existing rtabmap_msgs - the generated message "
                 "headers will not match the new sources.")

    return ros, core, ros_pkgs


def build_plan(log, args, core, ros_pkgs):
    """Produce the exact command sequence, with a reason for every line."""
    ws = Path(os.path.expanduser(args.workspace))
    src = ws / "src"
    tag = core_version_tag(core.get("version"))
    cmake_dir = core.get("cmake_dir") or "/usr/local/lib/cmake/rtabmap"

    steps = []

    def step(comment, cmd):
        steps.append((comment, cmd))

    step("Source ROS. Every new terminal needs this; without it catkin_make "
         "cannot find a single message package.",
         "source /opt/ros/noetic/setup.bash")

    step("Make sure the workspace source directory exists.",
         f"mkdir -p {src}")

    existing_msgs = ros_pkgs["packages"].get("rtabmap_msgs")
    if existing_msgs and "rtabmap_ros" not in str(existing_msgs):
        step("NOTE: rtabmap_msgs already exists at "
             f"{existing_msgs} outside a rtabmap_ros checkout. Move or remove "
             "it before cloning, or catkin will see two packages with the same "
             "name and pick one arbitrarily.",
             f"# inspect: ls -la {existing_msgs}")

    if tag:
        step(f"Clone rtabmap_ros at the tag that matches the installed core "
             f"({core.get('version')}). Matching versions is not optional: the "
             f"ROS layer calls into core across a C++ ABI that changes between "
             f"minor releases.",
             f"git clone {args.repo} -b {tag} --depth 1 {src}/rtabmap_ros")
        step("If that tag does not exist upstream, fall back to the release "
             "branch and then check out the closest tag by hand.",
             f"# git clone {args.repo} -b noetic-devel {src}/rtabmap_ros && "
             f"cd {src}/rtabmap_ros && git tag -l '0.21.*'")
    else:
        step("The installed core did not report a full x.y.z version, so the "
             "tag cannot be pinned automatically. Check it by hand with "
             "`rtabmap --version` and clone that exact tag.",
             f"git clone {args.repo} -b noetic-devel {src}/rtabmap_ros")

    step("Pull in ROS build dependencies for the new packages ONLY. "
         "--skip-keys=rtabmap is the critical flag: without it rosdep helpfully "
         "installs ros-noetic-rtabmap, which is the second copy of the core "
         "library that this whole exercise exists to prevent.",
         "rosdep install --from-paths " + str(src) + " --ignore-src -r -y "
         "--skip-keys=rtabmap --skip-keys=rtabmap_ros")

    step("Build. RTABMap_DIR points CMake at the EXISTING install so "
         "find_package(RTABMap) resolves to /usr/local and not to anything "
         "apt may have left behind. Release mode matters: a Debug build of the "
         "feature extractor is roughly an order of magnitude slower and will "
         "not keep up with the camera.",
         f"cd {ws} && catkin_make -j{args.jobs} "
         f"-DCMAKE_BUILD_TYPE=Release "
         f"-DRTABMap_DIR={cmake_dir} "
         f"-DRTABMAP_DIR={cmake_dir}")

    step("Source the workspace so the new nodes are on the package path.",
         f"source {ws}/devel/setup.bash")

    step("Verify. This must print a path inside your workspace, not inside "
         "/opt/ros. If it prints /opt/ros, an apt copy is shadowing the build.",
         "rospack find rtabmap_slam")

    step("Verify the ROS layer links against the SAME core library you "
         "inspected above. If two versions appear here, stop and remove one.",
         f"ldd {ws}/devel/lib/librtabmap_slam_plugins.so 2>/dev/null | "
         "grep -i rtabmap || ldd $(rospack find rtabmap_slam)/../../lib/*.so "
         "2>/dev/null | grep -i rtabmap_core")

    step("Final sanity check: the standalone binary and the ROS node should "
         "report the same version.",
         "rtabmap --version && rosrun rtabmap_slam rtabmap --version")

    return steps, ws


def write_script(log, steps, ws, args):
    """Write the plan out as a real, readable shell script."""
    out = slam_log_dir() / "build_rtabmap_ros_plan.sh"
    lines = [
        "#!/usr/bin/env bash",
        "#",
        "# Generated by sidewalk_slam/scripts/build_rtabmap_ros.py",
        "#",
        "# READ THIS BEFORE RUNNING IT. It compiles a large C++ tree and, on a",
        "# Jetson AGX Orin, takes roughly 25-45 minutes. Run it on the Jetson,",
        "# in a terminal with ROS Noetic sourced, and NOT while a recording or",
        "# a backup is in progress - the build will saturate every core.",
        "#",
        "set -euo pipefail",
        "",
    ]
    for comment, cmd in steps:
        lines.append("# " + "\n# ".join(_wrap(comment, 74)))
        lines.append(cmd)
        lines.append("")
    out.write_text("\n".join(lines))
    out.chmod(out.stat().st_mode | stat.S_IXUSR)
    log.info(f"Build plan written to: {out}")
    return out


def _wrap(text, width):
    words, line, out = text.split(), "", []
    for w in words:
        if len(line) + len(w) + 1 > width:
            out.append(line)
            line = w
        else:
            line = (line + " " + w).strip()
    if line:
        out.append(line)
    return out or [""]


def execute(log, steps, args):
    """Run the plan for real. Only reached with --execute."""
    ros = probe_ros()
    if not ros["present"]:
        die(log,
            "--execute was requested but no ROS environment is sourced.",
            "This machine has no ROS installed. Run this on the Jetson, in a "
            "terminal where you have run `source /opt/ros/noetic/setup.bash`.")

    if shutil.which("catkin_make") is None:
        die(log, "catkin_make is not on PATH even though ROS_DISTRO is set.",
            "Source /opt/ros/noetic/setup.bash again in this shell.")

    log.section("Executing build plan")
    for i, (comment, cmd) in enumerate(steps, start=1):
        if cmd.lstrip().startswith("#"):
            log.info(f"[{i}/{len(steps)}] (informational) {comment}")
            continue
        log.info(f"[{i}/{len(steps)}] {cmd}")
        # `source` is a shell builtin, so every step runs through bash -lc and
        # the environment from earlier steps is re-established each time by
        # re-sourcing. Chaining them in one shell would be fragile if a step
        # fails halfway.
        rc, out, err = run_cmd(["bash", "-lc", cmd], timeout=3600)
        if out.strip():
            log.info(out.strip()[-4000:])
        if rc != 0:
            log.error(err.strip()[-4000:] or f"exit code {rc}")
            die(log, f"Build step {i} failed: {cmd}",
                "Read the compiler output above. The most common causes are a "
                "missing rosdep dependency and an out-of-memory kill - if the "
                "message mentions 'Killed', rerun with a lower --jobs.")
    log.info("All build steps completed.")


def main(argv=None):
    args = parse_args(argv)
    with make_logger("build_rtabmap_ros") as log:
        log.info("Planning a build of rtabmap_ros against the EXISTING "
                 "RTAB-Map core installation.")
        log.info("This script never installs a second copy of the core library.")

        ros, core, ros_pkgs = inspect(log, args)

        tag = core_version_tag(core.get("version"))
        if core["present"] and not tag and not args.allow_version_mismatch:
            log.warn("Could not derive an exact release tag from the installed "
                     "core version. The plan will use the release branch; "
                     "confirm the tag by hand before building.")

        log.section("4. Proposed build plan")
        steps, ws = build_plan(log, args, core, ros_pkgs)
        for i, (comment, cmd) in enumerate(steps, start=1):
            log.info(f"[{i}] {comment}")
            log.info(f"     $ {cmd}")

        script = write_script(log, steps, ws, args)

        log.section("5. Missing pieces this build will provide")
        still_missing = [p for p in REQUIRED_PKGS
                         if not ros_pkgs["packages"].get(p)]
        log.metric("required_packages_missing", len(still_missing))
        log.info(f"required and missing: {', '.join(still_missing) or '(none)'}")
        optional_missing = [p for p in OPTIONAL_PKGS
                            if not ros_pkgs["packages"].get(p)]
        log.info(f"optional and missing: {', '.join(optional_missing) or '(none)'}")

        if args.execute:
            execute(log, steps, args)
            log.summary("rtabmap_ros build executed successfully", status="OK")
            return 0

        if not still_missing and core["present"]:
            log.summary("rtabmap_ros already complete - no build needed", status="OK")
            return 0

        log.summary(
            f"Build plan generated ({len(still_missing)} required packages "
            f"missing). Review {script.name}, then rerun with --execute on the "
            f"Jetson.", status="OK")
        return 0


if __name__ == "__main__":
    # SlamExit is how every tool in this package reports an already-diagnosed
    # refusal. The reason has been printed and logged by the time it reaches
    # here; all that is left is to hand the shell a non-zero status.
    try:
        sys.exit(main())
    except SlamExit as _exit:
        sys.exit(_exit.code)
