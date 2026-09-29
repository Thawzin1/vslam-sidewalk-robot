#!/usr/bin/env python3
"""
eval_common.py — Shared plumbing for every script in `sidewalk_evaluation`.

WHY THIS EXISTS
    The evaluation scripts run in three quite different situations, and each
    one breaks a different assumption:

      1. On the Jetson, inside a sourced catkin workspace, with ROS running.
      2. On the Jetson, offline, chewing through yesterday's recorded files.
      3. On any computer with no ROS at all, so a supervisor can regenerate the
         report from the trajectory files alone.

    Rather than have every script re-solve "where is run_logger", "is numpy
    installed", "is matplotlib installed", the answers live here once.

    The governing rule of this project is that **a traceback is a bug, not a
    diagnosis**. When something is missing, the user must be told exactly what
    is missing and exactly how to install it, and the process must exit
    non-zero. `require_numpy()` and `require_matplotlib()` exist to make that
    the path of least resistance.

NOT A ROS NODE
    Nothing in this file imports rospy. It is deliberately importable on a bare
    Python 3 install.
"""
from __future__ import annotations

import json
import os
import re
import sys
from datetime import datetime
from pathlib import Path

# --------------------------------------------------------------------------- #
# Repository location
# --------------------------------------------------------------------------- #


def find_repo_root(start: Path | None = None) -> Path:
    """Locate the project root by looking for a known landmark.

    Neither roslaunch nor rosrun guarantees a working directory, so relative
    paths are not trustworthy. We walk upward from this file looking for the
    pair of landmarks that only the repository root has.
    """
    here = (start or Path(__file__).resolve()).parent
    for cand in [here, *here.parents]:
        if (cand / "tools" / "explain.py").exists() and (cand / "catkin_ws").exists():
            return cand
    env = os.environ.get("SIDEWALK_REPO")
    if env and Path(env).exists():
        return Path(env)
    default = Path.home() / "vslam-sidewalk-robot"
    return default if default.exists() else Path.cwd()


REPO = find_repo_root()

# Default place results land. Kept beside the logs so an experiment's numbers
# and its narrative record stay together.
RESULTS_DIR = REPO / "logs" / "sidewalk_evaluation" / "results"


# --------------------------------------------------------------------------- #
# Shared logger — imported defensively
#
# run_logger.py lives in sidewalk_bringup. Inside a built workspace it is on the
# path already; run straight from a source checkout it is not. We add the
# sibling package's scripts directory before trying, and keep a minimal local
# fallback so this package still works standalone (e.g. copied onto a computer
# with just the trajectory files).
# --------------------------------------------------------------------------- #

_BRINGUP_SCRIPTS = REPO / "catkin_ws" / "src" / "sidewalk_bringup" / "scripts"
if _BRINGUP_SCRIPTS.is_dir() and str(_BRINGUP_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_BRINGUP_SCRIPTS))

# Also expose sidewalk_bringup as a real package, so the canonical
# `from sidewalk_bringup.run_logger import ...` works from source too.
_BRINGUP_SRC = REPO / "catkin_ws" / "src" / "sidewalk_bringup" / "src"
if _BRINGUP_SRC.is_dir() and str(_BRINGUP_SRC) not in sys.path:
    sys.path.insert(0, str(_BRINGUP_SRC))

try:
    try:
        # Canonical import: works from a source workspace and
        # after catkin_make install, because sidewalk_bringup
        # exports src/ via catkin_python_setup().
        from sidewalk_bringup.run_logger import RunLogger
    except ImportError:
        from run_logger import RunLogger
except ImportError:  # pragma: no cover - exercised only on a partial checkout
    class RunLogger:  # type: ignore[no-redef]
        """Minimal stand-in for the shared logger.

        Deliberately API-compatible with the real one so that swapping between
        them changes nothing at the call sites. It writes the same directory
        layout and the same RUNLOG.md line, just with less ceremony.
        """

        def __init__(self, package: str, run_name: str = "run", echo: bool = True,
                     repo_root: Path | None = None):
            self.package = package
            self.run_name = run_name
            self.echo = echo
            self.repo = repo_root or REPO
            self._metrics: dict = {}
            self._errors = 0
            self._warns = 0
            self._closed = False
            import time
            self._time = time
            self.started = time.time()
            stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
            self.dir = self.repo / "logs" / package
            self.dir.mkdir(parents=True, exist_ok=True)
            self.path = self.dir / f"{stamp}_{run_name}.log"
            self.metrics_path = self.dir / f"{stamp}_{run_name}.metrics.json"
            self._emit(f"=== {package} / {run_name} @ {datetime.now().isoformat()} "
                       "(fallback logger: sidewalk_bringup/run_logger.py not found) ===")

        def _emit(self, line: str) -> None:
            with open(self.path, "a") as fh:
                fh.write(line + "\n")
            if self.echo:
                print(line, flush=True)

        def info(self, msg: str) -> None:
            self._emit(f"INFO   {msg}")

        def warn(self, msg: str) -> None:
            self._warns += 1
            self._emit(f"WARN   {msg}")

        def error(self, msg: str) -> None:
            self._errors += 1
            self._emit(f"ERROR  {msg}")

        def metric(self, key: str, value, unit: str = "") -> None:
            self._metrics[key] = {"value": value, "unit": unit}
            self._emit(f"METRIC {key} = {value}{(' ' + unit) if unit else ''}")

        def section(self, title: str) -> None:
            self._emit("")
            self._emit(f"--- {title} " + "-" * max(0, 68 - len(title)))

        def summary(self, text: str, status: str = "") -> None:
            if self._closed:
                return
            self._closed = True
            elapsed = self._time.time() - self.started
            self._emit(f"SUMMARY {text}  ({elapsed:.1f}s)")
            if self._metrics:
                self.metrics_path.write_text(json.dumps(self._metrics, indent=2))
            if not status:
                status = "FAIL" if self._errors else ("WARN" if self._warns else "OK")
            index = self.repo / "logs" / "RUNLOG.md"
            index.parent.mkdir(parents=True, exist_ok=True)
            if not index.exists():
                index.write_text(
                    "# Run Log\n\n"
                    "| When | Package | Run | Status | Duration | Summary | Log |\n"
                    "|---|---|---|---|---|---|---|\n")
            with open(index, "a") as fh:
                fh.write(f"| {datetime.now().strftime('%Y-%m-%d %H:%M')} "
                         f"| `{self.package}` | {self.run_name} | **{status}** "
                         f"| {elapsed:.1f}s | {text} | [{self.path.name}]"
                         f"(sidewalk_evaluation/{self.path.name}) |\n")

        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc, tb):
            if exc_type is not None:
                self.error(f"Terminated by exception: {exc_type.__name__}: {exc}")
                self.summary(f"Run failed: {exc_type.__name__}", status="FAIL")
            else:
                self.summary("Run completed", status="OK")
            return False


# --------------------------------------------------------------------------- #
# Reference-kind provenance — the single source of truth
#
# ENGINEERING_NOTES.md section 4.1: never quote an ATE without stating the reference and its
# provenance. These tuples used to be duplicated across evaluate_trajectory,
# generate_report and ingest_navigation_run, which is exactly how the
# `synthetic_ground_truth` (the only sim-sounding kind on offer) and the
# report printed the self-test description for it. One list, one meaning.
# --------------------------------------------------------------------------- #

REFERENCE_KINDS = (
    "motion_capture",           # external tracking system (incl. public datasets like TUM)
    "rtk_gnss",                 # centimetre GPS
    "simulator_ground_truth",   # the physics engine's own pose (Gazebo model_states)
    "closed_loop",              # route returns to a surveyed start
    "cross_stack",              # another SLAM stack as the yardstick
    "wheel_odometry",           # encoder integration
    "nav_repeat_run",           # repeatability, not accuracy
    "synthetic_ground_truth",   # mathematically generated — SELF-TEST ONLY
    "unspecified",
)

# Kinds that justify printing an accuracy number for a (simulated or real)
# robot. Everything else either carries an honest caveat (wheel_odometry,
# closed_loop, ...) or is refused outright (UNQUOTABLE_KINDS).
VERIFIED_TRUTH_KINDS = ("motion_capture", "rtk_gnss", "simulator_ground_truth")

# Kinds a robot-data report must REFUSE to print numbers for:
# `unspecified` proves nothing was declared; `synthetic_ground_truth` is the
# self-test harness — it says nothing whatsoever about any robot.
UNQUOTABLE_KINDS = ("unspecified", "synthetic_ground_truth")


class ProvenanceError(RuntimeError):
    """Raised when a report would print numbers whose truth source is
    undeclared or unquotable. The message names the offending system and the
    honest re-run command — ENGINEERING_NOTES.md section 4.1 requires the refusal, this class
    makes it actionable."""


# --------------------------------------------------------------------------- #
# Dependency gates
# --------------------------------------------------------------------------- #

class MissingDependency(RuntimeError):
    """Raised when a required third-party module is not installed.

    Carries a message written for a human who has to fix it, not a stack trace.
    """


def require_numpy(log=None):
    """Return the numpy module, or fail with an actionable message.

    numpy is genuinely required: every metric in this package is linear algebra
    over trajectories, and reimplementing SVD in pure Python would be slower and
    less trustworthy than telling the user to run one apt command.
    """
    try:
        import numpy as np  # noqa: WPS433
        return np
    except ImportError:
        msg = ("numpy is not installed, and every metric in this package needs "
               "it.\n"
               "  On the Jetson (Ubuntu 20.04 / ROS Noetic):\n"
               "      sudo apt install python3-numpy\n"
               "  Or, if you are working inside a virtualenv:\n"
               "      pip3 install numpy")
        if log:
            log.error(msg)
        raise MissingDependency(msg)


def require_matplotlib(log=None):
    """Return (matplotlib, pyplot) with a headless backend, or raise.

    Plots are a nice-to-have, not a requirement: the numbers are the result and
    the figures only illustrate them. Callers are expected to catch
    MissingDependency and continue without figures, noting the omission in the
    report rather than aborting the analysis.

    The Agg backend is forced because this normally runs over SSH with no X
    display; without it matplotlib fails at import with an opaque Qt error.
    """
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt  # noqa: WPS433
        return matplotlib, plt
    except ImportError:
        msg = ("matplotlib is not installed, so no figures can be drawn. All "
               "numeric results are still valid and the HTML report will be "
               "generated without plots.\n"
               "  To enable figures:  sudo apt install python3-matplotlib")
        if log:
            log.warn(msg)
        raise MissingDependency(msg)


def require_rospy(log=None):
    """Return rospy, or fail with a message that names the actual problem.

    Three different mistakes produce 'No module named rospy', and they have
    three different fixes, so we say all three rather than making the user guess.
    """
    try:
        import rospy  # noqa: WPS433
        return rospy
    except ImportError:
        msg = ("rospy is not importable. This script is a live ROS node and "
               "cannot run without it.\n"
               "  1. Did you source the ROS environment?   "
               "source /opt/ros/noetic/setup.bash\n"
               "  2. Did you source this workspace?         "
               "source ~/catkin_ws/devel/setup.bash\n"
               "  3. Are you on the Jetson? This development PC runs Ubuntu "
               "26.04 and has no ROS installed at all — offline analysis "
               "scripts (evaluate_trajectory.py, compare_slam.py, "
               "generate_report.py) work here, but live recording nodes do not.")
        if log:
            log.error(msg)
        raise MissingDependency(msg)


# --------------------------------------------------------------------------- #
# Tiny YAML subset reader
#
# Same reasoning as tools/explain.py: PyYAML may not be present on a freshly
# flashed Jetson, and the config files here are flat maps and lists of scalars.
# When PyYAML *is* available we use it, because it is obviously more correct.
# --------------------------------------------------------------------------- #

def load_config(path) -> dict:
    """Read a config YAML. Uses PyYAML when available, else a flat-subset parser.

    Supported without PyYAML: `key: scalar`, `key: [a, b, c]`, one level of
    nested mapping via indentation, `#` comments. That covers every file in
    `config/`. Anything more elaborate belongs in PyYAML territory anyway.
    """
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(
            f"Config file not found: {path}\n"
            f"Expected one of the files in "
            f"catkin_ws/src/sidewalk_evaluation/config/. If you passed a "
            f"custom path, check for a typo.")
    text = path.read_text()
    try:
        import yaml  # noqa: WPS433
        data = yaml.safe_load(text)
        return data if isinstance(data, dict) else {}
    except ImportError:
        pass

    out: dict = {}
    stack = [(-1, out)]
    for raw in text.splitlines():
        if not raw.strip() or raw.lstrip().startswith("#"):
            continue
        line = raw.split("  #")[0].rstrip()
        indent = len(line) - len(line.lstrip())
        if ":" not in line:
            continue
        key, _, val = line.strip().partition(":")
        key, val = key.strip(), val.strip()
        while stack and indent <= stack[-1][0]:
            stack.pop()
        parent = stack[-1][1] if stack else out
        if val == "":
            child: dict = {}
            parent[key] = child
            stack.append((indent, child))
        else:
            parent[key] = _coerce(val)
    return out


def _coerce(val: str):
    """Turn a YAML scalar string into a Python value, conservatively."""
    val = val.strip()
    if len(val) >= 2 and val[0] == val[-1] and val[0] in "\"'":
        return val[1:-1]
    if val.startswith("[") and val.endswith("]"):
        inner = val[1:-1].strip()
        if not inner:
            return []
        return [_coerce(p) for p in inner.split(",")]
    low = val.lower()
    if low in ("true", "yes"):
        return True
    if low in ("false", "no"):
        return False
    if low in ("null", "none", "~"):
        return None
    if re.fullmatch(r"[-+]?\d+", val):
        return int(val)
    try:
        return float(val)
    except ValueError:
        return val


# --------------------------------------------------------------------------- #
# Small conveniences shared by several scripts
# --------------------------------------------------------------------------- #

def unquote(value: str) -> str:
    """Strip a surrounding pair of quotes from a command-line argument.

    WHY THIS IS NEEDED
        roslaunch passes a node's `args` attribute through as one string. When
        an argument legitimately contains spaces — a `--label` or a `--note` —
        it has to be quoted in the launch file. Depending on how the string is
        split, those quotes sometimes survive into `sys.argv`, and an empty
        optional argument arrives as the two-character string `''` rather than
        as an empty string. That then tests as truthy and produces a file
        literally named `''`.

        Cheap insurance, applied to every free-text argument that a launch file
        may supply.
    """
    if not value:
        return ""
    v = value.strip()
    while len(v) >= 2 and v[0] == v[-1] and v[0] in "\"'":
        v = v[1:-1].strip()
    return v


def timestamp_slug() -> str:
    """A filesystem-safe stamp used to name result directories."""
    return datetime.now().strftime("%Y%m%d-%H%M%S")


def ensure_results_dir(name: str = "") -> Path:
    """Create and return a directory for this evaluation run's artefacts."""
    d = RESULTS_DIR / (name or timestamp_slug())
    d.mkdir(parents=True, exist_ok=True)
    return d


def write_json(path, obj) -> Path:
    """Dump JSON with numpy scalars coerced to plain Python.

    json.dump chokes on numpy float64. Rather than sprinkle float() calls
    through the metric code, we normalise once here on the way out.
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(_jsonable(obj), indent=2, sort_keys=False))
    return path


def _jsonable(obj):
    if isinstance(obj, dict):
        return {str(k): _jsonable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_jsonable(v) for v in obj]
    if hasattr(obj, "tolist"):          # numpy array or scalar
        return _jsonable(obj.tolist())
    if isinstance(obj, (str, int, float, bool)) or obj is None:
        return obj
    return str(obj)


def fmt(value, digits: int = 4, unit: str = "") -> str:
    """Format a number for a table cell, tolerating None and NaN."""
    if value is None:
        return "n/a"
    try:
        f = float(value)
    except (TypeError, ValueError):
        return str(value)
    if f != f:                           # NaN
        return "n/a"
    if f == float("inf") or f == float("-inf"):
        return "inf"
    if abs(f) >= 1e4 or (f != 0 and abs(f) < 1e-3):
        s = f"{f:.{max(1, digits - 1)}e}"
    else:
        s = f"{f:.{digits}f}"
    return f"{s} {unit}".strip()


if __name__ == "__main__":
    # Self-check: report what this machine can and cannot do. Useful as a first
    # command on any new machine — it answers "will the evaluation run here?"
    print(f"repo root      : {REPO}")
    print(f"results dir    : {RESULTS_DIR}")
    print(f"run_logger     : "
          f"{'shared (sidewalk_bringup)' if RunLogger.__module__ == 'run_logger' else 'local fallback'}")
    for name, fn in (("numpy", require_numpy),
                     ("matplotlib", require_matplotlib),
                     ("rospy", require_rospy)):
        try:
            fn()
            print(f"{name:15s}: available")
        except MissingDependency:
            print(f"{name:15s}: MISSING")
