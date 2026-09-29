# SLAM session - deploy and run notes (for the Jetson session)

**Nothing live needs replacing.** `start_slam.sh` runs this folder's staged copy of `start_drive.sh`. That copy is the
live `~/slam_series2/tools/start_drive.sh` (sha256 `b0dd724f...`, in `start_drive.sh.BASE_sha256`) plus lines that
act only when `SLAM=1`; without `SLAM=1` it behaves identically. Five original lines were replaced, each by an
equivalent form whose extra part is empty without SLAM=1 (`diff ~/slam_series2/tools/start_drive.sh start_drive.sh`
shows them: the camera command gets `"${CAMCFG[@]}"`, step 3's `if` becomes `if SLAM ... elif FUSION`, and the three
`auto_stop_mapping.py` calls get `$AS_EXTRA`). Everything else it uses is already deployed (camera guard, timelapse
recorder, `stop_fusion.sh`, `auto_stop_mapping.py` - which already has `--launch-file`, the bridge, the blend).
Drive 10's master map `~/slam_series2/localise/s2_static_10_map.db` is only ever read (checksum + copy).

All paths are on the **Jetson** unless marked **robot**. Folder:
`<repo>/run/lib/slam_session/`, below `$S`.

## Files

| file | what it is |
|---|---|
| `start_slam.sh <run> [map.db]` | the one start command: `SLAM=1 SVO=1 FUSION=1` + the staged `start_drive.sh`; `BENCH=1` = camera only; `MASK=1` = people mask; `LOOP_THR=0.11` = drive 10's bar |
| `start_drive.sh` + `.BASE_sha256` | staged copy with the SLAM=1 mode (steps 0, 2S, 3, 3S, 3d, 4, 5 and the closing lines); `start_slam.sh` refuses if the live script changed since |
| `slam_run.launch` | the drives' stack on the copy, mapping mode, pinned `Mem/IncrementalMemory` true, `Mem/InitWMWithAllNodes` true, `RGBD/AggressiveLoopThr` 0.05, `Rtabmap/LoopThr` (arg, 0.08); `mask:=true` switches RTAB-Map to the masked depth |
| `slam_watch.py` | jobs-page line `<run>_slam`: JOINED yes/no (from the map's own working graph), links to the old map, `!! SUSPECT JOIN` |
| `slam_closed_check.py` | `--before` (old map facts, run at step 0) / `--after` (closed properly, joined, old map intact) |
| `stop_slam.sh <run> [--now]` | after "park" has closed the map: `db_check.py`, `slam_closed_check.py --after`, master checksum; `--now` = close at once (STOP file) |
| `zedx_front_od.yaml` + `zedx_front.yaml.BASE_sha256` | camera settings with the people detector (lines 1-185 = the live yaml; the start refuses if the live yaml changed) |
| `depth_person_mask.py` | the people mask node (`--selftest` = known-answer test); stops itself when the session's rtabmap exits |
| `od_bench.sh` | one-time detector optimisation (done 27 Sept, 649 s) + cost measurement; `rate_probe.py`, `tegra_window.py` its helpers |
| `bench_slam.sh <run> <MASK> [SVO]` | the bench test (camera only); `fake_person_test.py` its live made-up-person test |
| `session_split.py`, `score_slam.py` | results pack: today vs drive 10 split, S1/S3/S5 numbers; S2 wheel check |
| `make_route_plan.py` -> `route_plan.png`, `d10_optmap.npz` | the protocol figure (drive 10's own saved 2D grid, decoded read-only) |
| `DESIGN.md`, `PASS_LINES.md`, `PROTOCOL.md`, `RESULTS_PACK_PLAN.md`, `BENCH_RESULTS.md` | the documents |

## At the lab (the Jetson operator runs these; the user only does PROTOCOL.md)

Run id `s2_slam_01`. Robot parked on the start mark, facing east down the long corridor.

```bash
S="<repo>/run/lib/slam_session"
# 0. space: the session needs about 13-22 GB (DESIGN.md section 3); free it first under storage sense (rule 24)
df -h /
# 1. robot side FIRST (LiDAR recording + bridge sender), robot parked on the mark
ssh robot 'bash "/media/administrator/USB Drive/slam_series2/tools/robot_side.sh" start s2_slam_01'
# 2. Jetson (LIDAR_PEER / ROBOT_ADDRS as for the last drive if the robot's address changed)
MASK=1 bash "$S/start_slam.sh" s2_slam_01      # or without MASK=1 - see BENCH_RESULTS.md section 4
# 3. "park" (robot back on the mark, facing east):
touch ~/slam_series2/PARK_s2_slam_01            # auto_stop_mapping.py closes the map after 60 s still
bash "$S/stop_slam.sh" s2_slam_01               # checks + master checksum
ssh robot 'bash "/media/administrator/USB Drive/slam_series2/tools/robot_side.sh" stop s2_slam_01'
bash ~/slam_series2/tools/restore_camera_mode.sh   # camera back to the depth study's mode after SVO=1
```

- **Watch (rule 8 and rule 13):** jobs page lines `s2_slam_01_slam`, `_mask`, `_camera`, `_bridge`,
  `_ekf_inputs`, `_svo`, `_media`, `_autostop`; arm the usual 1-minute watcher on `~/.run_records/s2_slam_01/`
  (ALERT, rtabmap alive, the `_slam` line's freshness, `!! SUSPECT JOIN`).
- **If `!! SUSPECT JOIN`:** `touch ~/slam_series2/STOP_s2_slam_01`, then restart with `s2_slam_02` from the mark.
- **Never pass the master's path as the run's database.** The staged script refuses it.

## If the drive script or the camera yaml changed

`start_slam.sh` refuses when `~/slam_series2/tools/start_drive.sh` no longer matches `start_drive.sh.BASE_sha256`.
Re-stage: copy the new live script over `$S/start_drive.sh`, re-apply the blocks marked `SLAM` (search `SLAM` in the
old copy in git), `bash -n`, write the new checksum (`sha256sum ~/slam_series2/tools/start_drive.sh >
$S/start_drive.sh.BASE_sha256`), then re-run the bench: `bash $S/bench_slam.sh s2_slam_bench_NN 1 1`.
Likewise MASK=1 refuses when the live `zedx_front.yaml` changed: rebuild `zedx_front_od.yaml` from its first 185 lines
+ the object_detection block, and rewrite `zedx_front.yaml.BASE_sha256`.

## Not changed here (decisions for the user)

- `rtabmap_common.launch`'s eval bug (SOLVED.md 26 Sept) is unchanged; `slam_run.launch` pins the values after the
  include, as the localisation demo did.
- The master map `~/slam_series2/localise/s2_static_10_map.db` is mode `-rw-r--r--` (writable by its owner), unlike
  drive 4's master (`-r--r--r--`). Suggest `chmod 444` on it; not done here (the task said never modify it).
