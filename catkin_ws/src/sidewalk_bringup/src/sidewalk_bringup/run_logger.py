#!/usr/bin/env python3
"""
run_logger.py — The shared logging helper used by every package in this project.

WHY THIS EXISTS
    A research result is only worth as much as the record behind it. Six weeks
    after an experiment, "the mapping run on Tuesday looked good" is worthless;
    "run 2026-07-20T14:03, 1920x1200 @ 30 FPS, 4 dropped frames, ATE 0.14 m" is
    evidence.

    So every executable in this project logs through this one helper. That buys
    three things:

      1. Every run leaves a timestamped file, automatically, without the author
         of each node having to think about it.
      2. Metrics are machine-readable. `.metric()` writes structured key/value
         lines that the evaluation tooling parses later without regex guesswork.
      3. There is a single human-readable index — `logs/RUNLOG.md` — where one
         line per run tells you what happened and points at the detail. This is
         the file to open when someone asks "what did you actually do?".

    It deliberately has no dependencies beyond the Python standard library, so
    it works before ROS is installed, inside a container, or on a bare machine.

USAGE
    from run_logger import RunLogger

    log = RunLogger("sidewalk_perception", run_name="soak_test")
    log.info("Starting 15 minute frame-rate soak test")
    log.metric("configured_fps", 30)
    log.metric("dropped_frames", 4)
    log.warn("Frame interval exceeded threshold at t=412s")
    log.summary("Soak test PASSED — 4 dropped frames in 27000 (0.015%)")

    The summary line is what lands in RUNLOG.md, so write it for a human.
"""
from __future__ import annotations

import json
import os
import socket
import sys
import time
from datetime import datetime
from pathlib import Path


def _find_repo_root(start: Path | None = None) -> Path:
    """Walk upward looking for the repository marker.

    Nodes get launched from unpredictable working directories — roslaunch does
    not guarantee cwd — so we locate the repo by searching for a known landmark
    rather than trusting relative paths.
    """
    here = (start or Path(__file__).resolve()).parent
    for cand in [here, *here.parents]:
        if (cand / "tools" / "explain.py").exists() and (cand / "catkin_ws").exists():
            return cand
    # Fall back to an env var, then to the user's project directory, then cwd.
    env = os.environ.get("SIDEWALK_REPO")
    if env and Path(env).exists():
        return Path(env)
    default = Path.home() / "vslam-sidewalk-robot"
    return default if default.exists() else Path.cwd()


class RunLogger:
    """Per-run structured logger writing to logs/<package>/<timestamp>.log."""

    LEVELS = ("INFO", "WARN", "ERROR", "METRIC")

    def __init__(self, package: str, run_name: str = "run", echo: bool = True,
                 repo_root: Path | None = None):
        self.package = package
        self.run_name = run_name
        self.echo = echo
        self.repo = repo_root or _find_repo_root()

        self.started = time.time()
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")

        self.dir = self.repo / "logs" / package
        self.dir.mkdir(parents=True, exist_ok=True)
        self.path = self.dir / f"{stamp}_{run_name}.log"
        self.metrics_path = self.dir / f"{stamp}_{run_name}.metrics.json"

        self._metrics: dict = {}
        self._counts = {lvl: 0 for lvl in self.LEVELS}
        self._closed = False

        # A header block makes each log self-describing: anyone opening the file
        # cold can tell what produced it and on which machine.
        self._raw("=" * 72)
        self._raw(f"RUN      : {run_name}")
        self._raw(f"PACKAGE  : {package}")
        self._raw(f"STARTED  : {datetime.now().isoformat()}")
        self._raw(f"HOST     : {socket.gethostname()}")
        self._raw(f"USER     : {os.environ.get('USER', 'unknown')}")
        self._raw(f"PYTHON   : {sys.version.split()[0]}")
        self._raw(f"COMMAND  : {' '.join(sys.argv)}")
        self._raw("=" * 72)

    # ------------------------------------------------------------- internals
    def _raw(self, text: str) -> None:
        with open(self.path, "a") as fh:
            fh.write(text + "\n")
        if self.echo:
            print(text, flush=True)

    def _write(self, level: str, msg: str) -> None:
        self._counts[level] = self._counts.get(level, 0) + 1
        elapsed = time.time() - self.started
        line = f"[{datetime.now().strftime('%H:%M:%S')}] [{elapsed:8.2f}s] {level:6s} {msg}"
        with open(self.path, "a") as fh:
            fh.write(line + "\n")
        if self.echo:
            colour = {"INFO": "", "WARN": "\033[33m", "ERROR": "\033[31m",
                      "METRIC": "\033[36m"}.get(level, "")
            reset = "\033[0m" if colour else ""
            print(f"{colour}{line}{reset}", flush=True)

    # ---------------------------------------------------------------- public
    def info(self, msg: str) -> None:
        self._write("INFO", msg)

    def warn(self, msg: str) -> None:
        self._write("WARN", msg)

    def error(self, msg: str) -> None:
        self._write("ERROR", msg)

    def metric(self, key: str, value, unit: str = "") -> None:
        """Record a measurement.

        Metrics are kept separately in JSON as well as the text log, because the
        evaluation phase needs to load dozens of runs and compare them. Parsing
        prose is fragile; parsing JSON is not.
        """
        self._metrics[key] = {"value": value, "unit": unit,
                              "t": round(time.time() - self.started, 3)}
        self._write("METRIC", f"{key} = {value}{(' ' + unit) if unit else ''}")

    def section(self, title: str) -> None:
        self._raw("")
        self._raw(f"--- {title} " + "-" * max(0, 68 - len(title)))

    def summary(self, text: str, status: str = "") -> None:
        """Close the run and add one line to the project-wide RUNLOG.md index."""
        if self._closed:
            return
        self._closed = True

        elapsed = time.time() - self.started
        self._raw("")
        self._raw("-" * 72)
        self._raw(f"SUMMARY  : {text}")
        self._raw(f"DURATION : {elapsed:.1f}s")
        self._raw(f"COUNTS   : {self._counts['INFO']} info, "
                  f"{self._counts['WARN']} warn, {self._counts['ERROR']} error, "
                  f"{self._counts['METRIC']} metrics")
        self._raw("-" * 72)

        if self._metrics:
            self.metrics_path.write_text(json.dumps(self._metrics, indent=2))

        # Derive a status if the caller did not supply one.
        if not status:
            status = "FAIL" if self._counts["ERROR"] else (
                "WARN" if self._counts["WARN"] else "OK")

        index = self.repo / "logs" / "RUNLOG.md"
        if not index.exists():
            index.parent.mkdir(parents=True, exist_ok=True)
            index.write_text(
                "# Run Log\n\n"
                "One line per execution of anything in this project. This is the\n"
                "index of what was actually done, in order. The `log` column links\n"
                "to the full detail for that run.\n\n"
                "| When | Package | Run | Status | Duration | Summary | Log |\n"
                "|---|---|---|---|---|---|---|\n")
        try:
            rel = self.path.relative_to(self.repo)
        except ValueError:
            rel = self.path
        with open(index, "a") as fh:
            fh.write(f"| {datetime.now().strftime('%Y-%m-%d %H:%M')} "
                     f"| `{self.package}` | {self.run_name} | **{status}** "
                     f"| {elapsed:.1f}s | {text} | [{self.path.name}]({rel}) |\n")

    # Context-manager support, so a crash still closes the log cleanly.
    def __enter__(self) -> "RunLogger":
        return self

    def __exit__(self, exc_type, exc, tb) -> bool:
        if exc_type is not None:
            self.error(f"Terminated by exception: {exc_type.__name__}: {exc}")
            self.summary(f"Run failed: {exc_type.__name__}", status="FAIL")
        else:
            self.summary("Run completed", status="OK")
        return False


if __name__ == "__main__":
    # Self-test: exercise the logger so its output can be inspected directly.
    with RunLogger("sidewalk_bringup", run_name="logger_selftest") as log:
        log.info("Self-test of the shared logging helper")
        log.section("Simulated measurements")
        log.metric("example_frame_rate", 29.97, "fps")
        log.metric("example_dropped_frames", 0)
        log.warn("This is what a warning looks like")
        log.info("Self-test complete")
