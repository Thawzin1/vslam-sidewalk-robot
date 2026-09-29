# Contributing - the house rules for code in this repository

These rules kept a one-person field project understandable at 3 a.m. in a corridor. Keep them.

## Names and shape
- **Plain names.** A file, function or variable says what it is in ordinary words: `camera_guard.py`, not
  `cg.py`; `stop_started`, not `sweep2`. No codes for test conditions ("arm B1") without the words beside them.
- **Short functions, one job each.** If a shell function needs a scroll bar, split it.
- **A header on every file**: what it does, how to run it (the exact line), inputs and outputs, and one line
  of plain meaning (*"Plain terms: ..."*). Every technical term gets a short bracketed explanation the first
  time it appears. Write "section", never the "section " symbol.
- **No dated change notes in code.** "2026-09-26: fixed X" belongs in the git history
  (one section per file), not in the source. Comments explain WHY the code is as it is, not WHEN it changed.
- **No machine addresses, user names, tokens or absolute home paths in code or docs.** They come from `env.sh`
  through `run/lib/paths.sh` (shell) or the same environment variable names (Python). Add new ones there.

## Running things
- **Every job that outlives one command writes a `.progress` line** in `$JOBS_DIR` (one short line, rewritten
  at least every ~30 s, `N/M` for a bar, `123s` for elapsed time). The jobs page (`tools/jobs_dashboard.py`)
  finds it by itself. See docs/OPERATIONS.md, "The progress-line convention".
- **Start detached** (`setsid nohup ... &`) anything a dropped connection must not kill, and stop it **by
  process number**, never by searching command lines for a name (substring matches have reported dead jobs
  as alive four times in this project).
- **Every script passes `bash -n` / `python3 -m py_compile`; every launch file parses as XML.** Shell scripts
  start with `set -euo pipefail` unless they source ROS setup files (which need `set +u`), and say so.

## Results and wording
- **Every number traces to a run folder** under `results/<series>/<run>/`. No number in a document or
  README without the file it came from.
- **Report both start-to-end gaps, labelled**: the camera's tracking alone (a database's `Node.pose`) and the
  map's corrected positions (loop closures), with the closure count beside the corrected one.
- **The LiDAR trajectory is "agreement with an independent LiDAR estimate", never "error against ground
  truth."** State its uncertainty beside it (range noise 15-30 mm; the LiDAR-to-camera mounting offset was
  never measured). Ground truth means motion capture, RTK GNSS or a simulator; nothing here qualifies.
- **Never N=1 without saying so; median with range, not only a mean; every figure carries its spread.**
- **One variable per experiment.** Record once, replay many.
- **Parameters are verified on the build before they are named** (`rtabmap --params`, the actual file),
  never quoted from memory. Several "obvious" RTAB-Map parameters do not exist on this build
  (docs/DO_NOT_REPEAT.md).

## Pull requests
- Small, one topic each, with the command you ran to check it and its output.
- Do not commit recordings (`.svo2`, `.bag`, `.db`), videos or anything over 5 MB; they go to the data store
  listed in `data/DATA_INDEX.md`.
- Credit anything you did not write (see CREDITS.md), and do not modify the colleague's robot stack.
