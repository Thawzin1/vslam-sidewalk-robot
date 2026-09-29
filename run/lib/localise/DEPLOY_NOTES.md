# Localisation demo - deploy and run notes (for the Jetson session)

**Nothing live needs replacing.** `start_localise.sh` runs this folder's staged copy of `start_drive.sh`.
That copy is the live `~/slam_series2/tools/start_drive.sh` of 26 Sept 03:06 Hamilton, plus lines that act only when
`LOCALISE=1`; without `LOCALISE=1` it behaves identically. Only 3 original lines were replaced, each by an
equivalent guarded form (`diff` shows them). Everything else it uses is already deployed: the camera guard,
the timelapse recorder, `stop_fusion.sh`, the bridge, and the blend.

All paths are on the **Jetson** unless marked **robot**. Folder:
`<repo>/run/lib/localise/` (**Jetson**), below `$L`.

## Files

| file | what it is |
|---|---|
| `start_localise.sh <run> [map.db]` | the one start command: `LOCALISE=1 FUSION=1` + the staged `start_drive.sh`; `BENCH=1` = camera only |
| `start_drive.sh` | staged copy with the LOCALISE=1 mode (steps 0, 3, 3L, 4, 5 and the closing messages) |
| `start_drive.sh.BASE_sha256` | checksum of the live script it was staged from; `start_localise.sh` refuses if the live one has changed since |
| `localise_run.launch` | drive 4's stack with `localization:=true`, the **mapping-mode fix** (RESULTS.md finding 1), and `min_inliers` (default 20) |
| `localise_watch.py` | the jobs-page line `<run>_localise` ("LOCALISED yes/no, since, distance to the map path"), plus the events, track and summary files |
| `next_start.sh <run> <k> <mark>` | forget and reload: a fresh copy of the master map, `/rtabmap/load_database` (clear: false), and a line in starts.csv |
| `stop_localise.sh <run>` | the safe stop order, the P4 check, and removal of the working copies |
| `score_localise.py` | scores P1-P5 after the LiDAR replay; `--selftest` checks the scorer |
| `PASS_LINES.md`, `PROTOCOL.md`, `marks.png`, `RESULTS.md` | registered lines, the user's steps, the spots, and the bench results |

## At the lab (the Jetson operator runs these; the user only does PROTOCOL.md)

Run id: `s2_loc_01`. The robot is parked on the start mark, facing down the long corridor.

```bash
# 1. robot side FIRST, with the robot parked on the start mark (LiDAR recording + bridge sender)
ssh robot 'bash "/media/administrator/USB Drive/slam_series2/tools/robot_side.sh" start s2_loc_01'
# 2. Jetson (LIDAR_PEER / ROBOT_ADDRS as for drive 5 if the robot's address has changed)
bash "$L/start_localise.sh" s2_loc_01          # waits for READY (ready gate) like a drive
# 3. at each new spot, robot parked, when the user says "start k"
bash "$L/next_start.sh" s2_loc_01 2 A_east
bash "$L/next_start.sh" s2_loc_01 3 B_west
bash "$L/next_start.sh" s2_loc_01 4 A_west
bash "$L/next_start.sh" s2_loc_01 5 C_north
# 4. on "park" (robot back on the start mark, 60 s still)
bash "$L/stop_localise.sh" s2_loc_01
ssh robot 'bash "/media/administrator/USB Drive/slam_series2/tools/robot_side.sh" stop s2_loc_01'
```

- **Watch (rule 8 and rule 13):** the jobs page (<jobs page, port 8096>) line `s2_loc_01_localise`,
  plus the `_camera`, `_bridge` and `_ekf_inputs` lines as on a drive. Arm the usual 1-minute watcher on
  `~/.run_records/s2_loc_01/` (ALERT, rtabmap alive, the progress line's freshness).
- **The start-mark mark name for start 1:** `FIRST_MARK=start_mark` (the default).
- **To localise with the 25-point check** (the repo localisation file's value), prefix `MIN_INLIERS=25`.
  PASS_LINES.md registers **20**, so do not change it once the session has begun.
- The master map `~/slam_series2/localise/s2_static_04_map.db` is read-only (444) and is never given to
  RTAB-Map. **Never pass the master's path to `/rtabmap/load_database`**: `clear: true` erases the file it is
  given (CoreWrapper.cpp loadDatabaseCallback), and the folder is writable. Both scripts refuse the master's path.
- **The live map page** (:8095) shows drive 4's saved map, with the robot at its assumed position until the
  first fix. The yes/no is on the jobs page, not on the map page (the map page would need a code change; not
  done).

## After the lab

1. **Robot:** the LiDAR replay, parked, as for every drive, using the same `replay_lidar.sh` (mode colleague)
   that made drive 4's `lidar.tum` (checksum `b01380a0...` in drive 4's `provenance.txt`):
   `replay_lidar.sh s2_loc_01`. It writes `<card>/slam_series2/s2_loc_01/lidar.tum` and `lidar_map.npz`
   (**robot**, card).
2. Copy **only those two small files** to the Jetson (robot data stays on the robot, rule 17).
3. Score:
   ```bash
   nice -n 19 python3 "$L/score_localise.py" --rec ~/.run_records/s2_loc_01 --lidar <folder with the 2 files> \
       --d4 "results/series2_static/s2_static_04" --log ~/.run_records/s2_loc_01/mapping.log \
       --out "results/series2_static/s2_loc_01"   # a results pack folder of your choice
   ```
   Then write the results pack (rule 22: numbers, figure, and the timelapse from `~/.run_records/s2_loc_01/media/`).

## If the drive script changed

`start_localise.sh` refuses when `~/slam_series2/tools/start_drive.sh` no longer matches
`start_drive.sh.BASE_sha256`. To re-stage: copy the new live script over `$L/start_drive.sh`, re-apply the
blocks marked `LOCALISE` (search `LOCALISE` in the old copy in git), `bash -n`, and write the new checksum
with `sha256sum ~/slam_series2/tools/start_drive.sh > $L/start_drive.sh.BASE_sha256`. Then re-run the bench:
`BENCH=1 bash $L/start_localise.sh s2_loc_bench_02`, followed by `stop_localise.sh`.

## The mapping-mode bug in the live launch file (not changed here)

`rtabmap_common.launch`'s `Mem/IncrementalMemory` eval (RESULTS.md finding 1) is left as it is. Changing a
shared launch file is a decision for the user. Drives are unaffected, because mapping wants `true`, which is
what it always produces. Anyone using `rtabmap_localization.launch` gets mapping. The fix is one line: compare
`str(arg('localization')).lower() in ('true','1')`. The same applies to `delete_db_on_start`.
