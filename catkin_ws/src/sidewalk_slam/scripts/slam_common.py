#!/usr/bin/env python3
"""
slam_common.py — shared helpers for every node and tool in `sidewalk_slam`.

WHY THIS EXISTS
    Phase 3-4 runs three different SLAM systems (RTAB-Map, the ZED SDK's own
    positional tracking, and ORB-SLAM3) over the *same* recorded input. The
    scientific value of that comparison depends entirely on the three of them
    being treated identically: same input, same output format, same clock, same
    bookkeeping.

    The moment each stack gets its own bespoke result-writing code, the
    comparison stops being a comparison and becomes three anecdotes. So all the
    shared machinery lives here:

      * locating the repository and the per-run output directory,
      * writing trajectories in exactly one format (TUM),
      * a sidecar JSON that records *how* each trajectory was produced,
      * environment probes that answer "is rtabmap/ZED SDK/ORB-SLAM3 actually
        present on this machine?" without importing or launching anything.

    Nothing in this module imports rospy, numpy or any ROS message type at
    module scope. That is deliberate: these helpers must work on the authoring
    computer without ROS as well as on the Jetson, and `python3 -m
    py_compile` must succeed everywhere.

TRAJECTORY FORMAT — TUM
    One pose per line, whitespace separated::

        timestamp tx ty tz qx qy qz qw

    * `timestamp` is seconds since the UNIX epoch as a float. When running from
      a recording with `use_sim_time`, this is the *recorded* time, not wall
      time — which is what makes two runs of the same bag comparable.
    * `tx ty tz` are metres.
    * `qx qy qz qw` is a unit quaternion, **scalar last**. This is the ROS
      convention and the TUM convention; it is NOT the convention used by
      Eigen's constructor or by MATLAB, both of which put w first. Getting this
      backwards produces a trajectory that looks plausible and is wrong, which
      is the worst kind of bug.

    The previous OAK-D work on the Jetson already used this format, and `evo`
    reads it directly, so the evaluation package gets a free ride.
"""
from __future__ import annotations

import json
import math
import os
import shutil
import subprocess
import sys
from datetime import datetime
from pathlib import Path

PACKAGE = "sidewalk_slam"

# The three stacks under comparison. The key is used in filenames and in the
# evaluation package, so it is a stable identifier — do not rename casually.
STACKS = ("rtabmap", "zed_sdk", "orbslam3")

STACK_LABEL = {
    "rtabmap": "RTAB-Map (primary)",
    "zed_sdk": "ZED SDK positional tracking (baseline)",
    "orbslam3": "ORB-SLAM3 stereo/stereo-inertial (baseline)",
}

# Nominal hardware constants for the ZED X (SKU ZED-312110), read off
# `Official Resources/ZED X - ZED X Mini - Datasheet.pdf` (Rev 1.6). The
# authoritative copy of all of these lives in
# sidewalk_perception/scripts/perception_common.py::ZEDX, with per-field
# provenance; they are restated here because sidewalk_slam must not depend on
# sidewalk_perception. Keep the two in step.
#
# MOSTLY sanity references to cross-check values measured at runtime — but the
# comment that used to sit here said "never written into a SLAM configuration
# directly", and for ZEDX_IMU_RATE_HZ THAT WAS FALSE. make_orbslam3_config.py
# writes it straight into the generated settings file as IMU.Frequency (see
# that file, the "IMU_FREQ" entry in the values dict). The claim held for the
# intrinsics, which genuinely do come from live camera_info, and it was quietly
# extended to a constant it did not cover. Corrected below, per constant.
ZEDX_NOMINAL_BASELINE_M = 0.120   # [VERIFIED — datasheet "Baseline: ZED X 12cm
                                  # (4.72")".] Cross-check only: the baseline
                                  # actually used is recovered from the right
                                  # camera_info's P[3] = -fx*Tx.

# [VERIFIED — datasheet, twice: "Motion Sensors 200 Hz 16-bits Accelerometer /
# 200 Hz 16-bits Gyroscope" and "Output Data Rate 200 Hz".]
#
# WAS 400.0, AND THIS ONE IS NOT A LABEL — IT IS WRITTEN INTO ORB-SLAM3.
# make_orbslam3_config.py emits it as IMU.Frequency, and ORB-SLAM3 does not
# merely print that number. [VERIFIED — ORB_SLAM3/src/Tracking.cc:613-614, and
# again at 1411-1412 for the legacy parser, in the checkout snapshotted at
# backup/jetson-20260720/home/Developer/ORB_SLAM3:
#
#     const float sf = sqrt(mImuFreq);
#     mpImuCalib = new IMU::Calib(Tbc, Ng*sf, Na*sf, Ngw/sf, Naw/sf);
#
# ] So the declared frequency scales all four inertial noise terms before they
# ever reach the preintegration. Declaring 400 Hz for a 200 Hz sensor makes
# sf too large by sqrt(2) = 1.414, and the two halves move in OPPOSITE
# directions, which is why this is worth spelling out rather than calling it
# "a covariance error":
#
#     Ng, Na  (white noise)      multiplied by sf -> 41% LARGER than intended.
#                                Each IMU sample is treated as noisier than it
#                                is, so inertial data is under-weighted against
#                                vision.
#     Ngw, Naw (bias walk)       divided by sf -> 29% SMALLER than intended.
#                                The gyro and accelerometer biases are treated
#                                as more stable than they are, so the optimiser
#                                is slower to re-estimate a drifting bias.
#
# Both are silent. No crash, no warning, no obviously wrong output — just a
# trajectory whose inertial terms are weighted wrong in a way that looks
# entirely plausible. Note that the noise terms themselves (IMU_DEFAULTS in
# make_orbslam3_config.py) are admitted estimates, so this is a factor on top
# of an already-uncertain number; that does not make it harmless, because the
# frequency is a fact that can simply be got right.
#
# NO STEREO-INERTIAL RESULT HAS BEEN PRODUCED YET, so nothing published is
# affected. Any generated orbslam3_*_inertial.yaml sitting on disk from before
# by hand — regenerating also picks up the live T_b_c1 from TF.
#
# [UNVERIFIED — the rate the zed-ros-wrapper actually publishes.] The datasheet
# states what the part produces. Confirm against the live topic before the
# first stereo-inertial run, because IMU.Frequency should describe the stream
# ORB-SLAM3 receives:  rostopic hz /zedx_front/zed_node/imu/data
ZEDX_IMU_RATE_HZ = 200.0

ZEDX_DEPTH_MIN_M = 1.0   # [VERIFIED — "Depth Range Max 1.0m to 35m", ZED X
ZEDX_DEPTH_MAX_M = 35.0  # 4.6mm column. Ideal range stops at 20 m.]


# --------------------------------------------------------------------------- #
# Logging
# --------------------------------------------------------------------------- #

def _fallback_logger_class():
    """Build a minimal stand-in for RunLogger.

    Defined lazily inside a function so that the real implementation is always
    preferred and this code path costs nothing when the import succeeds.
    """

    class _FallbackRunLogger:
        """Bare-bones logger used only if sidewalk_bringup is unreachable.

        It keeps the same API surface so calling code never has to branch, but
        it says loudly that it is a fallback: a run logged by this class did not
        land in logs/RUNLOG.md and is therefore not part of the project record.
        """

        def __init__(self, package, run_name="run", echo=True, repo_root=None):
            self.package = package
            self.run_name = run_name
            self.echo = echo
            self.repo = repo_root or repo_root_dir()
            self.dir = self.repo / "logs" / package
            try:
                self.dir.mkdir(parents=True, exist_ok=True)
                stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
                self.path = self.dir / f"{stamp}_{run_name}.log"
            except OSError:
                self.path = None
            self._emit("WARN", "run_logger.py not importable - using fallback "
                               "logger. This run will NOT appear in RUNLOG.md.")

        def _emit(self, level, msg):
            line = f"[{datetime.now().strftime('%H:%M:%S')}] {level:6s} {msg}"
            if self.echo:
                print(line, flush=True)
            if self.path is not None:
                try:
                    with open(self.path, "a") as fh:
                        fh.write(line + "\n")
                except OSError:
                    pass

        def info(self, msg):
            self._emit("INFO", msg)

        def warn(self, msg):
            self._emit("WARN", msg)

        def error(self, msg):
            self._emit("ERROR", msg)

        def metric(self, key, value, unit=""):
            self._emit("METRIC", f"{key} = {value}{(' ' + unit) if unit else ''}")

        def section(self, title):
            self._emit("INFO", f"--- {title} ---")

        def summary(self, text, status=""):
            self._emit("INFO", f"SUMMARY [{status or 'OK'}]: {text}")

        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc, tb):
            if exc_type is not None:
                self.error(f"Terminated by exception: {exc_type.__name__}: {exc}")
                self.summary(f"Run failed: {exc_type.__name__}", status="FAIL")
            else:
                self.summary("Run completed", status="OK")
            return False

    return _FallbackRunLogger


def get_logger_class():
    """Return the shared RunLogger, or a compatible fallback.

    The shared helper lives in a sibling package, sidewalk_bringup. Three cases,
    tried in order:

      1. `run_logger` is already importable — true when something else has
         already put it on sys.path, or when both scripts sit in one directory.
      2. Source/devel space: walk to ../../sidewalk_bringup/scripts and import
         from there. This is the normal case during development.
      3. Neither worked — return the fallback logger, which prints and writes
         the per-run file but does NOT append to logs/RUNLOG.md.

    Note that catkin_install_python does NOT put the two packages' scripts in
    one directory: each lands in lib/<package>/, so case 1 does not hold in a
    true install space and case 2 does not either (there is no source tree).
    An install-space deployment therefore runs on the fallback logger unless
    sidewalk_bringup's script directory is on PYTHONPATH. package.xml declares
    an exec_depend on sidewalk_bringup so the package is at least present.
    """
    try:
        try:
            # Canonical import: works from a source workspace and
            # after catkin_make install, because sidewalk_bringup
            # exports src/ via catkin_python_setup().
            from sidewalk_bringup.run_logger import RunLogger
        except ImportError:
            from run_logger import RunLogger
        return RunLogger
    except ImportError:
        pass

    sibling = (Path(__file__).resolve().parent.parent.parent
               / "sidewalk_bringup" / "scripts")
    if sibling.is_dir() and str(sibling) not in sys.path:
        sys.path.insert(0, str(sibling))
    try:
        from run_logger import RunLogger  # type: ignore
        return RunLogger
    except ImportError:
        return _fallback_logger_class()


def make_logger(run_name, echo=True):
    """Convenience: construct a logger for this package."""
    return get_logger_class()(PACKAGE, run_name=run_name, echo=echo)


# --------------------------------------------------------------------------- #
# Paths
# --------------------------------------------------------------------------- #

def repo_root_dir(start=None):
    """Locate the repository root by looking for known landmarks.

    roslaunch does not promise a working directory, so nothing may be resolved
    relative to cwd. We walk upward from this file looking for the same markers
    run_logger.py uses, which keeps both helpers agreeing on where the repo is.
    """
    here = (Path(start) if start else Path(__file__).resolve()).parent
    for cand in [here, *here.parents]:
        if (cand / "tools" / "explain.py").exists() and (cand / "catkin_ws").exists():
            return cand
    env = os.environ.get("SIDEWALK_REPO")
    if env and Path(env).exists():
        return Path(env)
    default = Path.home() / "vslam-sidewalk-robot"
    return default if default.exists() else Path.cwd()


def slam_log_dir(create=True):
    """logs/sidewalk_slam/"""
    d = repo_root_dir() / "logs" / PACKAGE
    if create:
        d.mkdir(parents=True, exist_ok=True)
    return d


def trajectory_dir(run_id, create=False):
    """logs/sidewalk_slam/trajectories/<run_id>/

    Deliberately under logs/ rather than a new top-level data/ directory:
    `tools/explain.py --check` treats every directory under data/ as a package
    that must carry an explain.yaml, so creating data/trajectories/ would break
    the documentation check for the whole project.

    `create` defaults to False so that merely ASKING where a run's output would
    go - which the dry-run planner does - does not litter the logs directory
    with empty folders for experiments that were never performed.
    """
    d = slam_log_dir(create=create) / "trajectories" / run_id
    if create:
        d.mkdir(parents=True, exist_ok=True)
    return d


def map_dir(create=False):
    """logs/sidewalk_slam/maps/ — RTAB-Map databases and exported maps."""
    d = slam_log_dir(create=create) / "maps"
    if create:
        d.mkdir(parents=True, exist_ok=True)
    return d


def new_run_id(prefix="run"):
    """A sortable, collision-free identifier for one comparison run."""
    return f"{datetime.now().strftime('%Y%m%d-%H%M%S')}_{prefix}"


# --------------------------------------------------------------------------- #
# Quaternion helpers (pure Python — no numpy dependency on purpose)
# --------------------------------------------------------------------------- #

def quat_norm(qx, qy, qz, qw):
    return math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)


def normalise_quat(qx, qy, qz, qw):
    """Return a unit quaternion, or None if the input is degenerate.

    Floating point drift makes stored quaternions creep off the unit sphere.
    A non-unit quaternion silently applies a scaling to every rotated vector,
    so we renormalise on write rather than hoping downstream tools do it.
    """
    n = quat_norm(qx, qy, qz, qw)
    if n < 1e-9 or not math.isfinite(n):
        return None
    return qx / n, qy / n, qz / n, qw / n


def quat_angle_between(a, b):
    """Smallest rotation angle in radians between two (x, y, z, w) quaternions.

    q and -q represent the same rotation, hence the abs() on the dot product.
    Used by the validator to spot physically impossible attitude jumps.
    """
    dot = abs(sum(ai * bi for ai, bi in zip(a, b)))
    dot = max(-1.0, min(1.0, dot))
    return 2.0 * math.acos(dot)


# --------------------------------------------------------------------------- #
# TUM trajectory I/O
# --------------------------------------------------------------------------- #

class TumWriter:
    """Append-only writer for a TUM-format trajectory plus a metadata sidecar.

    Every trajectory file gets a `.meta.json` next to it recording which stack
    produced it, which topic or frame pair it came from, and which input it was
    replayed over. Six months from now, a bare `.txt` full of numbers with no
    provenance is not evidence of anything.
    """

    HEADER = "# timestamp tx ty tz qx qy qz qw"

    def __init__(self, path, stack, source, frame_id="", child_frame_id="",
                 input_name="", extra=None, write_header=True):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.stack = stack
        self.source = source
        self.frame_id = frame_id
        self.child_frame_id = child_frame_id
        self.input_name = input_name
        self.extra = dict(extra or {})

        self.count = 0
        self.rejected = 0
        self.first_stamp = None
        self.last_stamp = None

        self._fh = open(self.path, "w")
        if write_header:
            # '#' is the default comment character for numpy.loadtxt, which is
            # what evo uses, so the header is free to a reader and invaluable to
            # a human opening the file cold.
            self._fh.write(self.HEADER + "\n")

    def add(self, stamp, tx, ty, tz, qx, qy, qz, qw):
        """Write one pose. Returns True if accepted.

        Bad poses are dropped rather than written, because a single NaN row
        poisons every downstream alignment routine. The count of rejects is kept
        and reported: silently discarding data would be worse than the NaN.
        """
        vals = (stamp, tx, ty, tz, qx, qy, qz, qw)
        if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in vals):
            self.rejected += 1
            return False
        q = normalise_quat(qx, qy, qz, qw)
        if q is None:
            self.rejected += 1
            return False
        qx, qy, qz, qw = q

        # %.9f on the timestamp preserves nanosecond resolution: ROS stamps are
        # nanosecond integers, and truncating them destroys the very timing
        # information the three-way comparison depends on.
        self._fh.write(f"{stamp:.9f} {tx:.6f} {ty:.6f} {tz:.6f} "
                       f"{qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}\n")
        self.count += 1
        if self.first_stamp is None:
            self.first_stamp = stamp
        self.last_stamp = stamp
        return True

    def close(self):
        if self._fh and not self._fh.closed:
            self._fh.flush()
            self._fh.close()
        meta = {
            "format": "TUM: timestamp tx ty tz qx qy qz qw (quaternion scalar-last)",
            "stack": self.stack,
            "stack_label": STACK_LABEL.get(self.stack, self.stack),
            "source": self.source,
            "frame_id": self.frame_id,
            "child_frame_id": self.child_frame_id,
            "input": self.input_name,
            "poses_written": self.count,
            "poses_rejected": self.rejected,
            "first_stamp": self.first_stamp,
            "last_stamp": self.last_stamp,
            "duration_s": (None if self.first_stamp is None
                           else round(self.last_stamp - self.first_stamp, 6)),
            # Recorded evidence of the achieved rate, so "was the ground truth
            # actually >= 50 Hz" is answered by the sidecar, not by assumption.
            "mean_rate_hz": (
                round((self.count - 1) / (self.last_stamp - self.first_stamp), 2)
                if self.count > 1 and self.last_stamp > self.first_stamp
                else None),
            "written_at": datetime.now().isoformat(),
            "package": PACKAGE,
        }
        meta.update(self.extra)
        self.path.with_suffix(self.path.suffix + ".meta.json").write_text(
            json.dumps(meta, indent=2))
        return meta

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
        return False


def read_tum(path):
    """Load a TUM trajectory. Returns (poses, problems).

    `poses` is a list of 8-tuples; `problems` is a list of human-readable
    strings describing every line that could not be parsed. The validator
    reports those verbatim, because "line 4102 has 7 fields, expected 8" is a
    diagnosis and "parse error" is not.
    """
    poses, problems = [], []
    p = Path(path)
    if not p.exists():
        return poses, [f"file does not exist: {p}"]
    with open(p) as fh:
        for n, raw in enumerate(fh, start=1):
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) != 8:
                problems.append(f"line {n}: {len(parts)} fields, expected 8")
                continue
            try:
                poses.append(tuple(float(x) for x in parts))
            except ValueError:
                problems.append(f"line {n}: non-numeric field")
    return poses, problems


def tum_path(run_id, stack, create=False):
    """The one canonical place a stack's trajectory is written."""
    return trajectory_dir(run_id, create=create) / f"{stack}_trajectory.tum"


# --------------------------------------------------------------------------- #
# Environment probes
#
# These answer "what is installed?" WITHOUT importing or executing anything
# heavyweight. They exist so that every tool in this package can fail with a
# specific sentence instead of a traceback. A traceback is a bug, not a
# diagnosis.
# --------------------------------------------------------------------------- #

def _first_existing(paths):
    for p in paths:
        q = Path(os.path.expanduser(str(p)))
        if q.exists():
            return q
    return None


def probe_ros():
    """Is a ROS 1 environment sourced in this shell?"""
    distro = os.environ.get("ROS_DISTRO", "")
    return {
        "present": bool(distro) and shutil.which("roslaunch") is not None,
        "distro": distro,
        "roslaunch": shutil.which("roslaunch"),
        "rosbag": shutil.which("rosbag"),
        "package_path": os.environ.get("ROS_PACKAGE_PATH", ""),
        "master_uri": os.environ.get("ROS_MASTER_URI", ""),
        "detail": (f"ROS_DISTRO={distro or '(unset)'}, "
                   f"roslaunch={'found' if shutil.which('roslaunch') else 'NOT found'}"),
    }


def probe_rtabmap_core(search=None):
    """Find the standalone RTAB-Map core library and read its version.

    We look for the versioned shared object rather than asking a package
    manager, because on the Jetson the library was built from source and
    installed to /usr/local — apt has never heard of it. The version is taken
    from the filename suffix, which is the SONAME the linker will actually pick.
    """
    roots = search or [
        "/usr/local/lib", "/usr/lib", "/usr/lib/aarch64-linux-gnu",
        "/usr/lib/x86_64-linux-gnu",
    ]
    found = []
    for root in roots:
        d = Path(root)
        if not d.is_dir():
            continue
        try:
            for so in d.glob("librtabmap_core.so.*"):
                version = so.name.split("librtabmap_core.so.", 1)[1]
                # Skip bare major-version symlinks like ".so.0.21" if a fuller
                # version string is also present; keep them otherwise.
                found.append({"path": str(so), "version": version})
        except OSError:
            continue
    found.sort(key=lambda f: len(f["version"]), reverse=True)

    cmake_cfg = _first_existing([
        "/usr/local/lib/cmake/rtabmap/RTABMapConfig.cmake",
        "/usr/lib/cmake/rtabmap/RTABMapConfig.cmake",
    ])
    return {
        "present": bool(found),
        "libraries": found,
        "version": found[0]["version"] if found else None,
        "cmake_config": str(cmake_cfg) if cmake_cfg else None,
        "cmake_dir": str(cmake_cfg.parent) if cmake_cfg else None,
        "source_dir": str(_first_existing(["~/Developer/rtabmap"]) or ""),
        "tools": {name: shutil.which(name) for name in
                  ("rtabmap", "rtabmap-export", "rtabmap-databaseViewer",
                   "rtabmap-info", "rtabmap-reprocess")},
    }


def probe_rtabmap_ros(workspace_src=None):
    """Which rtabmap_ros sub-packages exist, in the workspace or installed.

    rtabmap_ros 0.21 is not one package — it is a family (rtabmap_msgs,
    rtabmap_conversions, rtabmap_odom, rtabmap_sync, rtabmap_slam, rtabmap_util,
    rtabmap_viz, rtabmap_launch, ...). Knowing that only rtabmap_msgs is built
    is exactly the situation on the Jetson, and it is what the build helper has
    to reason about.
    """
    wanted = ["rtabmap_msgs", "rtabmap_conversions", "rtabmap_odom",
              "rtabmap_sync", "rtabmap_slam", "rtabmap_util", "rtabmap_viz",
              "rtabmap_launch", "rtabmap_rviz_plugins", "rtabmap_python",
              "rtabmap_demos", "rtabmap_examples"]

    src = Path(workspace_src) if workspace_src else (
        repo_root_dir() / "catkin_ws" / "src")

    search_dirs = [src]
    distro = os.environ.get("ROS_DISTRO", "")
    if distro:
        search_dirs.append(Path(f"/opt/ros/{distro}/share"))
    for entry in os.environ.get("ROS_PACKAGE_PATH", "").split(":"):
        if entry:
            search_dirs.append(Path(entry))

    status = {}
    for pkg in wanted:
        hit = None
        for d in search_dirs:
            if not d.is_dir():
                continue
            for cand in (d / pkg, d / "rtabmap_ros" / pkg):
                if (cand / "package.xml").exists():
                    hit = str(cand)
                    break
            if hit:
                break
        status[pkg] = hit

    return {
        "packages": status,
        "found": [k for k, v in status.items() if v],
        "missing": [k for k, v in status.items() if not v],
        "partial": bool([v for v in status.values() if v]) and
                   bool([v for v in status.values() if not v]),
        "workspace_src": str(src),
    }


def probe_zed_sdk():
    """Is the ZED SDK (and the ROS 1 wrapper) present?"""
    sdk = _first_existing(["/usr/local/zed"])
    version_file = None
    version = None
    if sdk:
        version_file = _first_existing([sdk / "zed-config-version.cmake",
                                        sdk / "include" / "sl" / "Camera.hpp"])
        # The SDK ships a plain-text version marker; read it if it is there.
        vf = sdk / "settings" / "version.txt"
        if vf.exists():
            try:
                version = vf.read_text().strip().splitlines()[0]
            except OSError:
                version = None
    # zed_x_daemon is what actually talks to the GMSL2 capture card. If the SDK
    # is present but the daemon is not running, the camera appears to be absent
    # even though it is wired correctly - a distinction worth reporting.
    daemon = shutil.which("zed_x_daemon")
    if daemon is None:
        cand = _first_existing(["/usr/local/zed/tools/ZED_X_Daemon",
                                "/usr/local/zed/tools/zed_x_daemon"])
        daemon = str(cand) if cand else None

    return {
        "present": sdk is not None,
        "root": str(sdk) if sdk else None,
        "version": version,
        "version_hint": str(version_file) if version_file else None,
        "daemon": daemon,
        "explorer": shutil.which("ZED_Explorer"),
    }


def probe_orbslam3(root=None):
    """Locate an ORB-SLAM3 build and check the pieces that usually go missing.

    The two classic failures are (a) the vocabulary is still a .tar.gz because
    nobody extracted it, and (b) the ROS examples were never built because
    ROS_PACKAGE_PATH did not include Examples/ROS at build time. Both are
    checked explicitly here so the error message can say which one it is.
    """
    base = Path(os.path.expanduser(str(root))) if root else _first_existing(
        ["~/Developer/ORB_SLAM3", "~/ORB_SLAM3", "/opt/ORB_SLAM3"])
    if base is None:
        return {"present": False, "root": None,
                "detail": "no ORB_SLAM3 directory found in the usual places"}

    vocab_txt = base / "Vocabulary" / "ORBvoc.txt"
    vocab_gz = base / "Vocabulary" / "ORBvoc.txt.tar.gz"
    ros_dir = base / "Examples" / "ROS" / "ORB_SLAM3"

    binaries = {}
    for name in ("Stereo", "Stereo_Inertial", "Mono", "RGBD"):
        cand = _first_existing([
            ros_dir / name,
            base / "Examples" / "Stereo" / f"stereo_{name.lower()}",
        ])
        binaries[name] = str(cand) if cand else None

    return {
        "present": True,
        "root": str(base),
        "vocabulary": str(vocab_txt) if vocab_txt.exists() else None,
        "vocabulary_still_compressed": (not vocab_txt.exists()) and vocab_gz.exists(),
        "ros_examples_dir": str(ros_dir) if ros_dir.is_dir() else None,
        "binaries": binaries,
        "ros_binaries_built": any(binaries[k] for k in ("Stereo", "Stereo_Inertial")),
        "reference_config": str(_first_existing(["~/oak-d-params.yaml"]) or ""),
    }


def probe_lidar():
    """LiDAR is not part of the build yet.

    This probe exists so that every LiDAR-adjacent option in this package can be
    gated on a single, honest answer rather than each script inventing its own
    guess. It reports absence, and it must keep reporting absence until a real
    device is on the robot.
    """
    return {
        "present": False,
        "detail": ("No LiDAR is fitted to this robot. Every LiDAR code path in "
                   "sidewalk_slam is scaffolding only and is disabled."),
    }


def run_cmd(cmd, timeout=30):
    """Run a command, capture output, never raise.

    Returns (returncode, stdout, stderr). A returncode of -1 means the binary
    was not found; -2 means it timed out. Callers turn these into sentences.
    """
    try:
        p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                           timeout=timeout, text=True)
        return p.returncode, p.stdout, p.stderr
    except FileNotFoundError:
        return -1, "", f"executable not found: {cmd[0]}"
    except subprocess.TimeoutExpired:
        return -2, "", f"timed out after {timeout}s: {' '.join(cmd)}"
    except OSError as exc:
        return -3, "", f"{type(exc).__name__}: {exc}"


class SlamExit(Exception):
    """A deliberate, already-reported termination.

    Raised by die(). It is NOT SystemExit, for one specific reason: every tool
    here runs its work inside `with RunLogger(...)`, and the logger's context
    manager reports any escaping exception. Raising SystemExit made a clean,
    fully-diagnosed refusal print "Terminated by exception: SystemExit", which
    reads exactly like the crash this package exists to avoid. With a named
    exception the same line reads as what it is - a controlled stop after the
    reason has already been printed.

    Each tool converts it back into a process exit code at the bottom of its
    own file, so the shell still sees a non-zero status.
    """

    def __init__(self, code=2, message=""):
        super().__init__(message or
                         f"controlled exit after a reported failure (code {code})")
        self.code = code


def die(log, message, hint="", code=2):
    """Report a specific, actionable failure and stop.

    Every early exit in this package goes through here so the shape of a failure
    is always the same: what went wrong, then what to do about it. A traceback
    is a bug; this is a diagnosis.
    """
    log.error(message)
    if hint:
        log.error(f"FIX: {hint}")
    log.summary(message, status="FAIL")
    raise SlamExit(code)


if __name__ == "__main__":
    # Running this module directly prints an environment report. It is the
    # fastest way to answer "what does this machine actually have installed?"
    # and it is safe everywhere: it reads, it never launches.
    with make_logger("environment_probe") as log:
        log.section("ROS")
        ros = probe_ros()
        log.info(ros["detail"])

        log.section("RTAB-Map core")
        core = probe_rtabmap_core()
        if core["present"]:
            log.info(f"core version {core['version']} at {core['libraries'][0]['path']}")
            log.info(f"cmake config: {core['cmake_config'] or 'NOT FOUND'}")
        else:
            log.warn("librtabmap_core.so.* not found")

        log.section("rtabmap_ros packages")
        ros_pkgs = probe_rtabmap_ros()
        log.info(f"found:   {', '.join(ros_pkgs['found']) or '(none)'}")
        log.info(f"missing: {', '.join(ros_pkgs['missing']) or '(none)'}")

        log.section("ZED SDK")
        zed = probe_zed_sdk()
        log.info(f"present={zed['present']} root={zed['root']}")

        log.section("ORB-SLAM3")
        orb = probe_orbslam3()
        log.info(json.dumps(orb, indent=2))

        log.section("LiDAR")
        log.info(probe_lidar()["detail"])

        log.summary("Environment probe complete", status="OK")
