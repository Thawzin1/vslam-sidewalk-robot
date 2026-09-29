#!/usr/bin/env python3
"""
run_logger.py — executable wrapper around the shared logging library.

THE REAL IMPLEMENTATION LIVES IN  src/sidewalk_bringup/run_logger.py

This file exists for two reasons and contains no logic of its own:

  1. **Backwards compatibility.** Sibling packages that add this scripts/
     directory to sys.path and then `from run_logger import RunLogger` keep
     working unchanged.

  2. **A runnable self-test.** `rosrun sidewalk_bringup run_logger.py` exercises
     the logger end to end and leaves a real log file you can inspect.

WHY THE IMPLEMENTATION MOVED
    It used to live here. That meant every other package reached into this
    folder by absolute path to import it — which works from a source workspace
    and silently breaks after `catkin_make install`, because catkin installs
    scripts to lib/<package>/ rather than onto the Python path.

    The failure mode was quiet, which is the worst kind: the import fails, each
    package falls back to its own stub logger, and the project-wide RUNLOG.md
    index stops being written. Nothing errors. You just gradually lose the
    record of what was run.

    Now the library is a proper Python package exported by catkin_python_setup(),
    so the canonical import is:

        from sidewalk_bringup.run_logger import RunLogger

    which resolves identically from source and from an install space.
"""

import sys
from pathlib import Path

# Prefer the properly installed package. This is the path that works after
# `catkin_make install` and is what new code should rely on.
try:
    from sidewalk_bringup.run_logger import RunLogger, _find_repo_root  # noqa: F401
except ImportError:
    # Fall back to loading the sibling source file directly. This covers the
    # case where the workspace has not been built yet, or where someone is
    # running straight out of a git checkout with no catkin at all — which is
    # exactly how the documentation tooling uses it.
    _src = Path(__file__).resolve().parent.parent / "src"
    if _src.is_dir() and str(_src) not in sys.path:
        sys.path.insert(0, str(_src))
    from sidewalk_bringup.run_logger import RunLogger, _find_repo_root  # noqa: F401


__all__ = ["RunLogger"]


if __name__ == "__main__":
    # Self-test: exercise the logger so its behaviour can be inspected directly.
    # Run it after provisioning to confirm logging works before relying on it.
    with RunLogger("sidewalk_bringup", run_name="logger_selftest") as log:
        log.info("Self-test of the shared logging helper")
        log.info(f"Implementation loaded from: {RunLogger.__module__}")
        log.section("Simulated measurements")
        log.metric("example_frame_rate", 29.97, "fps")
        log.metric("example_dropped_frames", 0)
        log.warn("This is what a warning looks like")
        log.info("Self-test complete — check logs/RUNLOG.md for the index entry")
