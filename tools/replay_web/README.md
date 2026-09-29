# replay_web - the "Replay" tab of the live website

A web page and a small backend that let lab students start, watch and stop replays of recorded data
on the Jetson, with no robot:

1. **Mapping replay of a drive** - wraps `run/replay.sh` (see `docs/REPLAY.md`): `<run>.svo2` (the
   camera recording) + `fusion.bag` (wheels and gyroscope) are played through the same programs as the
   live drive, and RTAB-Map builds a new map. Options: depth mode, stop after N seconds, a name.
2. **Camera bench test (depth modes on sphere targets)** - recomputes depth from a station's `.svo2`
   in each chosen depth mode with
   `catkin_ws/src/sidewalk_evaluation/scripts/depth_benchmark/svo_replay_n.py`, and reports the
   three-sphere target's centre (x ahead, y sideways) and its frame-to-frame spread in mm.

The page shows the job's state and progress bar, the last 30 log lines, the live map picture while a
mapping replay runs, per-mode frame counts while a bench test runs, the results when done (numbers
and pictures), the recordings on the Jetson (with Delete for copied-in inputs), how to copy a recording in from the
Autonomous Service Robot Teams folder, and a history of past jobs.

## Files

| file | what it is |
|---|---|
| `replay_control_server.py` | the backend: web interface on 127.0.0.1:8098, lock, safety checks, listings, starts jobs |
| `replay_job.py` | runs one job (mapping replay or bench test) and keeps its state file current |
| `replay_common.py` | settings, safety checks and helpers shared by the two above |
| `replay_figures.py` | the trajectory picture, the bench scatter pictures and the bench numbers (matplotlib) |
| `map_view_local.py` | runs `live_map_server.py` unchanged, but listening on 127.0.0.1 only |
| `keep_replay_control.sh` | starts the backend if it is not running; for cron |
| `replay.html` | the page; goes next to the site's other pages and uses their `common.js` and `site.css` |

## What it needs

- ROS Noetic, the ZED SDK with its Python module (`pyzed`), this repository's workspace built, and
  the tools `run/replay.sh` needs (`docs/REPLAY.md`). System `python3` with numpy and matplotlib.
  The backend itself uses only the Python standard library.
- The recordings, copied onto the Jetson by hand: download the run folder from the Autonomous Service
  Robot Teams folder on any computer and copy it over the lab network into `~/replay_inputs/slam/<run>/`
  (`<run>.svo2` + `fusion.bag`) or `~/replay_inputs/bench/`, e.g.
  `rsync -s -r --partial <run>/ <user>@<jetson>:~/replay_inputs/slam/<run>/`.
- The live website server `tools/live_site/live_site_server.py`, with the change described in
  `tools/live_site/REPLAY_CHANGE.md`: it checks the view key and forwards `/replay/api/...` here, and
  serves `/replay.html`. The tab is `['replay.html', 'Replay']` in `nav()` in the site's `common.js`.

## How to start it

```bash
# by hand
REPO_ROOT=/path/to/this/repository python3 tools/replay_web/replay_control_server.py

# for good: two crontab lines (crontab -e)
* * * * * REPO_ROOT=/path/to/repository /path/to/repository/tools/replay_web/keep_replay_control.sh
@reboot sleep 60 && REPO_ROOT=/path/to/repository /path/to/repository/tools/replay_web/keep_replay_control.sh
```

Copy `replay.html` into the site's page folder (the one `live_site_server.py` serves). Check with
`curl -s 127.0.0.1:8098/replay/api/status`. Its log is `$JOBS_DIR/replay_control_server.log`.

Settings are environment variables with defaults (full list in `replay_common.py`): `REPO_ROOT`,
`RECORDS_DIR` (~/.run_records), `WORK_DIR` (~/slam_series2), `JOBS_DIR` (~/jobs),
`REPLAY_INPUTS_DIR` (~/replay_inputs, with `slam/<run>/` and `bench/`), `REPLAY_RESULTS_DIR`
($WORK_DIR/replay/web), `EXTRA_RECORD_DIRS` (more folders to list, ':'-separated),
`REPLAY_ROS_PORT` (11399), `MAP_VIEW_PORT` (8195), `REPLAY_CONTROL_PORT` (8098).

## Web addresses (all behind the site's view key)

| address | what |
|---|---|
| `/replay.html` | the page |
| `GET /replay/api/status` | job state, progress, last 30 log lines, current refusals, free disk |
| `GET /replay/api/recordings` | recordings on the Jetson, with sizes and what each is for |
| `GET /replay/api/history` | past jobs |
| `GET /replay/api/check?kind=slam&extra_bytes=N` | what a new job would be refused for now (starts nothing) |
| `GET /replay/api/livemap.png`, `livemap.json` | the running mapping replay's map picture and status lines |
| `GET /replay/api/result/<job>/[file]` | a job's result folder, read-only, small files only |
| `POST /replay/api/start`, `stop`, `delete` | the buttons (JSON bodies only) |

## Safety

Anyone with the site link may start and stop replays (there is no second password), so the backend
protects the machine instead:

- **One job at a time** - a lock file `$JOBS_DIR/replay_web/lock`, created atomically.
- **Never during a robot session** - refused while any `$JOBS_DIR/*_drive.progress`,
  `*_localise.progress`, `*_slam.progress` or `*_nav.progress` is younger than 2 minutes
  (`*_to_drive.progress` files are file-copy jobs and are not counted), or while a process named
  exactly `rtabmap` talks to the live ROS master 11311 (read from its environment; unreadable counts
  as live).
- **Disk** - refused if free space minus the expected outputs would fall under 3 GB.
- **Own ROS master** (11399) and own map picture port (8195), both checked free before a mapping replay.
- **Stop in the right order** - a playing mapping replay is stopped through `replay.sh`'s own path
  (marker file `svo_end.json`): RTAB-Map is closed first and waited for, then the rest, and the
  summary is still written. During start-up the job runner stops the processes itself in the same
  order. The result shows whether SQLite finds the map database intact.
- Processes are found by number and exact name, never by text search (`pgrep -f` matches itself).
- Only paths inside the known recording folders are accepted; Delete works only inside
  `REPLAY_INPUTS_DIR` and on whole result folders.
- Jobs run under `nice -n 19`.

## Limits

The same as `docs/REPLAY.md` section 7: the replay reproduces the measurements, not the same frames
(node and closure counts differ a little between runs); start from a parked moment (the first ~5 s
play before the fine sync); depth is recomputed; only what the camera file covers can be replayed;
normal speed (rate 1.0) only; one replay per machine. Results are described as agreement with the
original drive, not as error. A bench test on a recording without the sphere target stand finds no
target and only proves that depth recomputation and measurement run.

The bench test replays with self-calibration off and, by default, depth stabilisation off
(`svo_replay_n.py --depth-stabilization 0`), so two replays of one file agree; "on (1)" matches the
live camera's setting.
