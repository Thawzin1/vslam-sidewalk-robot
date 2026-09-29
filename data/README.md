# data/ - where the recordings are

The recordings themselves (camera recordings, map databases, LiDAR (laser scanner) recordings,
timelapse videos) are too large for this repository. They are all in the **Autonomous Service
Robot** Microsoft Teams folder (the lab team's shared online folder). Ask Prof. Moein Mehrtash
(McMaster University) for access.

| file | what it is |
|---|---|
| `DATA_INDEX.md` | the index of that Teams folder: every run, every file, its size and what it holds. Paths are relative to the top of the Teams folder. |
| `list_folder.py` | prints a local copy of the Teams folder (or any part of it) as the same tables |

## Getting a recording onto the Jetson for a replay

1. Download the run's folder from Teams on any computer (for a SLAM drive: `series2_raw/<run>/`).
2. Copy it to the Jetson over the lab network. `rsync -s` (`-s` protects the spaces in folder
   names) resumes a copy that was cut off:

   ```bash
   rsync -s -av --partial <run>/ <user>@<jetson>:~/replay_inputs/slam/<run>/
   ```

   Camera-only bench recordings go to `~/replay_inputs/bench/`. `scp -r` works too.
   The Jetson's login and its address on the lab network: ask Prof. Mehrtash or the lab.
3. Run the replay (`run/replay.sh <run> --svo <file.svo2> --bag <fusion.bag>`, see `docs/REPLAY.md`).

A drive recording is 0.5-17 GB; check the Jetson has room first (`df -h ~`).

## Listing a folder

```bash
python3 data/list_folder.py <local copy of the Teams folder> > listing.md
```

It only reads. One section per top-level folder, one table per sub-folder, largest files first.
The "what it contains" column is guessed from the file ending; the run descriptions in
`DATA_INDEX.md` (what each drive was for, which results pack came from it) were written by hand
from each run's `RESULTS.md` and are not produced by the script.
