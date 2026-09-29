# SLAM session - results pack plan (dry run, 27 Sept 2026)

*Rule 22: every drive gets a stored results pack (numbers, figures, timelapse) in
`results/<series>/<run>/` (made on the Jetson). This file says which existing tools
work unchanged on a **continued (multi-session) database**, which need the split first, and what is new. Checked on
the bench database `s2_slam_bench_01.db` (1674 nodes: drive 10's 1558 + 116 new, map ids 0 and 1) before it was
deleted.*

## 1. The problem in one line

Every pack tool assumes **one session per database**. A continued map holds drive 10 (node ids 1-1558, map id 0,
recorded 27 Sept 02:02-02:31) **and** today (ids 1559+, map id 1). Tools that read "all nodes" mix the two.

| tool (Jetson `~/slam_series2/tools/` unless said) | what it does on a continued map (bench) | use it how |
|---|---|---|
| `db_to_tum.py` | **wrong as is**: 1674 poses spanning 26 086 s and 260 m - drive 10's tracking + today's, in two unrelated tracking frames | use `session_split.py` -> `camera.tum` (today only, tracking alone) |
| `db_corrected_tum.py` | writes all 862 saved-graph poses (drive 10's 860 + today's 2); its "end gap" (0.18 m) is **drive 10's node 1 to today's last node** - not a start-to-end gap of today (rule 20) | use `session_split.py` -> `camera_corrected.tum` (today) and `old_after.tum` (drive 10 after today) |
| `optimize_graph_se2.py` (tools/) | rebuilds corrected poses from the Link table; needed only if the session did NOT close properly. Untested on two map ids; a never-joined session is two separate graphs | only if `slam_closed_check.py` says NOT closed; then check it per map id |
| `render_map.py` | reads `Node.pose` (tracking alone) of all nodes -> the two sessions drawn in two frames on one picture | run with `--poses <session_split camera_corrected.tum + old_after.tum>`; colour today's nodes separately (new figure below) |
| `compare_lidar.py` / `size_fit_agreement.py` / `score_f3dof.py` | work on a run folder with `camera_corrected.tum` + `lidar.tum` matched by time | unchanged, on the split `camera_corrected.tum` (today's times only) - S4 |
| `check_closed.py` | passes on ANY copy of drive 10 (its `Admin.opt_poses` exists already) | **not enough**: `slam_closed_check.py --after` (new nodes in the saved graph, Admin newer than the start) - run by `stop_slam.sh` |
| `db_check.py` | SQLite reads every page | unchanged |
| timelapse (`drive_media_recorder.py`) | worked on the bench (35 frames, 3.5 s video) | unchanged |
| `turns_replay.py`, `extract_timing.py`, `bridge_freshness_*`, `tegra_peaks.py`, `robot_wheels_from_robot_bag.py` | read `fusion.bag`, `mapping.log`, `tegrastats.log`, the robot bag - one session each by construction | unchanged |
| `analyze_run.py` (results_packs_2026-09-26) | counts closures and nodes from the database | needs the split: closures **within today**, **today <-> drive 10** (the joins) and **within drive 10** reported separately (`session_split.py` `closures_by_kind`) |
| `map_3d` export (drive 10's `map_3d_clean/scripts`) | exports every node's cloud with its saved pose | colour by session; export today's nodes alone for the "new area" 3D view |

## 2. New tools (this folder)

| tool | output | pass line |
|---|---|---|
| `session_split.py DB old_map_facts.json OUT` | `camera.tum`, `camera_corrected.tum` (today), `old_after.tum`, `old_shift.csv`, `split_facts.json` (joins, closures by kind, both gaps with counts, drive-10 shift, new area) | S1 (join time), S3 (b, c), S5 |
| `score_slam.py DB old_map_facts.json wheel.tum OUT` | `s2_links.csv`, `s2_summary.json` | S2 |
| `slam_closed_check.py --after DB old_map_facts.json OUT.json` | closed properly + joined | S3 (a) |
| `slam_watch.py` (live) | `slam_events.csv`, `slam_summary.json` | S1 live, SUSPECT flag |
| `depth_person_mask.py` (live or replay) | `mask_stats.csv`, `mask_events.csv` (people boxes per picture) | D0, M1 |

`session_split.py` was run on the bench database: joins 1 (today 1561 -> drive 10 node 144, type 1), both gaps
0.002 m with 0 closures within today (the camera never moved), drive-10 shift median 0.003 m (the old map untouched
by a 2-picture session), new area 0 m2 - all as expected for a parked camera.

## 3. The pack, in order (after "park", robot parked)

1. `stop_slam.sh <run>` (checks + master checksum) - Jetson, minutes.
2. `session_split.py` -> `results/series2_slam/<run>/split/`.
3. Robot: LiDAR Force3DoF replay of the session (as drive 10: `replay_lidar.sh`, wrap f3dof), then pull the two small
   files (`lidar.tum`, `lidar_map.npz`), rule 17/robot-data rule.
4. Wheels: `wheel.tum` from the robot's own bag (`robot_wheels_from_robot_bag.py`, as drive 10) - S2 input.
5. `score_slam.py` (S2), `compare_lidar.py` on the split `camera_corrected.tum` (S4).
6. Figures: (a) the combined map from above, drive 10's nodes grey, today's nodes coloured, the joins marked, the
   new areas outlined (S5); (b) drive-10 shift map (arrows, S3 b); (c) S2 disagreement per link against time;
   (d) LiDAR vs camera (LiDAR solid, camera dashed; "agreement with the LiDAR estimate", rule 19).
7. RESULTS.md with both gap kinds + closure counts (rule 20), the pass-line table, the timelapse, a line in
   `INDEX.md` (project records).

## 4. The people ON/OFF replay (D0-D3) - after the session, on the Jetson, no robot needed

Record once, replay twice (ENGINEERING_NOTES.md section 4 rule 5). Each replay: a fresh copy of drive 10's master, the session's
SVO, the detector ON in both, only the mask differs. Camera-only (no wheels), identical in both.

```bash
S="run/lib/"      # Jetson
# replay A (mask OFF) and B (mask ON), one at a time, each into its own copy:
bash "$S/replay_svo_slam.sh" <run> off      # tested form: BENCH_RESULTS.md section 5 (smoke test, mask on)
bash "$S/replay_svo_slam.sh" <run> on       # each: fresh copy, every frame, detector on, ~session length
```

Not yet written: the D-scorer (`score_people.py`). Measured per replay from its database + `mask_events.csv`: occupied grid cells and 3D points within 0.5 m of the
people's recorded positions (D0/D1); tracking vs the session's wheels over each person window (D2); wrong links by the
S2 method (D3). Disk: two more 3.5 GB copies - one at a time, each deleted after its numbers are taken.
