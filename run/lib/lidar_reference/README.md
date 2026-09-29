# The LiDAR yardstick, replayed on the Jetson (27 Sept 2026)

**Why:** the LiDAR yardstick (the robot's LiDAR map, used to judge the camera map) was replayed on the robot,
so the robot had to stay on for ~35 min after every drive. Replaying on the Jetson lets the robot charge.
*Plain terms: the same recording goes through the same program with the same settings, only on the other computer.*

**What it is (rule 19):** the colleague's LiDAR mapping (Matt Jing's `self_navigation` package on the robot:
`3dreplay_pipeline.launch` sensor:=helios -> Helios decoder -> `helios_restamp.py` -> robot body filter ->
`box_crop.py` -> `rtabmap_3d.launch`), unchanged, with ONE setting added: `Reg/Force3DoF=true` (the map may only
slide and turn on the floor; it may not believe the robot tipped). It is wheel+IMU odometry corrected by LiDAR
loop closures - **an independent estimate, not ground truth**. Nothing of the colleague's is edited (rule 18).

## What is where

| what | device | path |
|---|---|---|
| this folder (scripts, wrappers, README) | Jetson (and git) | `run/lib/lidar_reference/` |
| the LiDAR workspace (build + environment) | Jetson internal disk | `~/lidar_slam_ws/` - enter it ONLY with `source ~/lidar_slam_ws/setup_lidar_slam.bash` |
| recordings copied off the robot (working copies) | Jetson internal disk | `~/lidar_bags/<run>_lidar.bag` (+ `.sha256`) |
| replay outputs | Jetson internal disk | `~/lidar_replays/<run>/lidar_ref_<variant>/` |
| the original recordings | robot's 128 GB card | `/media/administrator/USB Drive/slam_series2/<run>_lidar.bag` |
| the robot's drive-10 result (the thing to agree with) | Jetson (git) | `results/series2_static/s2_static_10/lidar_ref_f3dof/` |

`~/lidar_ws` (Jetson) is **the depth study's** Helios decoder workspace (2026-09-20, Scenario 3) and is not used or
touched here (rule 16); hence the separate `~/lidar_slam_ws`.

## Files in this folder

| file | what it is |
|---|---|
| `replay_colleague_variant_jetson.sh` | the replay: `<run> wrap f3dof` (or `wrap colleague`). Its header lists every difference from the robot's script |
| `variants/f3dof.launch` | includes `robot_copy/f3dof.launch` byte for byte + ONE Jetson line: the decoder reads `robot_replay_config.yaml` |
| `variants/colleague.launch` | the colleague's pipeline with no change (comparison only) + the same decoder line |
| `robot_copy/f3dof.launch`, `robot_copy/replay_colleague_variant.sh` | the robot's files, recovered byte-exact (md5 `0767fc1e...`, `d50d096a...` = the md5s in FINDINGS.md) |
| `robot_replay_config.yaml` | the Helios decoder settings the robot used (**reconstructed**: mirror copy + `split_angle: 180`; see R1) |
| `pull_bag_from_robot.sh` | copies one recording robot card -> `~/lidar_bags`, sha256 computed on BOTH machines must match |
| `fetch_robot_reference.sh` | read-only look at the robot: its decoder config (R1), md5 of the colleague's files (R2), its map's parameters (R3) |
| `validate_against_robot.py` | the agreement check, pass lines stated in its header (below) |
| `watch_jetson_replay.sh` | the rule-8 watcher: one line per state change, incl. failure and silent death |
| `dry_test.sh` (+ `dry_test_synthetic_input.py`, `dry_test_stage_counter.py`) | 25 s start-up test with no recording; result in `dry_test_result.txt` |

## What differs from the robot (all recorded in each run's `provenance.txt`)

1. **RTAB-Map 0.21.13** (Jetson, apt) vs **0.21.10** (robot). Same values SET; a changed default would differ -
   `rtabmap_params_db.txt` (every parameter the map ran with) is diffed against the robot's map by the validation.
   Both builds have libpointmatcher and GTSAM (`provenance.txt`). *Checked:* the colleague's old name `Icp/PM=true`
   reaches RTAB-Map as `Icp/Strategy="true"`, which 0.21.13 reads as 1 = libpointmatcher (compiled test of
   `Parameters::parse`), as intended.
2. **Decoder config:** `robot_replay_config.yaml`. The Jetson's mirror copy of the colleague's
   `rslidar_sdk/config/replay_config.yaml` (md5 `7bea8030...`) lacks `split_angle: 180`, which the robot's copy
   (md5 `cb9ca662...`) has - drive 10's `replay.log` start-up dump shows split_angle 180 and every other value
   equal. *Plain terms: where the decoder cuts one LiDAR turn into one cloud - at the back (180 degrees) on the robot.*
3. **Environment:** `rslidar_sdk` built from the robot mirror (v1.5.19); `self_navigation` + `mcm07_husky` copied
   unchanged and found without compiling; the body filter unpacked into the home folder (see "Installed").
4. **Isolated ROS master on port 11350** (robot: 11312). `/use_sim_time` is set true only there.
5. `rosbag play` at nice 19 too (robot: normal priority); pipeline at nice 19 as on the robot.
6. **Jetson safety (rule 23):** refuses to start while a drive/localisation is live, and if one starts mid-replay
   (or `~/slam_series2/HELPERS_STOP` appears) it stops: playback interrupted, map closed, folder renamed `.stopped-<time>`.
7. Failed/stopped attempts are **renamed, never removed**, so a re-run starts clean (rule 14).

**Timing is not deterministic, on either machine.** `helios_restamp.py` stamps each cloud with the replay clock
when it arrives, and the map takes one node per second of replay time; so two replays of one recording pick
slightly different clouds. Measured on the synthetic 40 s recording, Jetson twice: same 38 nodes and 3 closures,
paths 2.4 mm apart (median), end gaps 6.921 vs 6.955 m. That is why the validation has bands, not equality.

## Commands

### 0. Already done (27 Sept, robot off) - the dry test
```bash
cd "<repo>/run/lib/lidar_reference" && ./dry_test.sh f3dof 25
```
Result `dry_test_result.txt`: **PASS** - all 5 of the colleague's nodes up; decoder read `robot_replay_config.yaml`
and listens on `/helios/packets`; `Reg/Force3DoF` = true on the map node; clouds reached every stage
(250 -> 250 restamped -> 249 body-filtered -> 249 cropped; the 250 points planted on the robot's body and in the
GPS-arm box were all removed); 24 map nodes, map closed with corrected positions; nothing left running.
The same warnings as the robot's own replay log (Icp/PM renamed, Grid/Sensor set to 0, 3 links without shapes).
Also run end to end on a synthetic 40 s recording through `replay_colleague_variant_jetson.sh` (all products made).
*Not tested without the robot: the Helios decode of real packets* (it is the robot's own code and config; the
validation below tests it).

### When the robot is on (parked or charging with its computer on) - Jetson terminal
Robot on the USB link (`ssh robot-usb`, <robot-usb-address>). The robot is only read; nothing there changes.

**R1-R3** (1 min): the robot's decoder config, its file checksums, and its drive-10 map's parameters
```bash
cd "<repo>/run/lib/lidar_reference" && ./fetch_robot_reference.sh s2_static_10
```
Pass: `robot copy: cb9ca662...`, `settings equal (comments ignored): True`, every R2 line `same`. If the settings
differ, copy `robot_copy/replay_config.yaml.robot` over `robot_replay_config.yaml` before replaying.

**Copy drive 10's recording** (9.17 GB, about 6 min at 26 MB/s, then sha256 on both machines ~2 min):
```bash
cd "<repo>/run/lib/lidar_reference" && ./pull_bag_from_robot.sh s2_static_10
```
Space: needs the copy to leave >= 3 GB free (Jetson had ~13 GB on 27 Sept -> ~4 GB after). **The robot can be
turned off to charge as soon as this prints `complete`.**

### Replay on the Jetson (robot not needed; ~31 min of recording at 1x + ~2 min)
```bash
cd "<repo>/run/lib/lidar_reference" && setsid nohup ./replay_colleague_variant_jetson.sh s2_static_10 wrap f3dof > ~/lidar_replays/s2_static_10_f3dof_jetson.out 2>&1 < /dev/null &
```
Watcher (rule 8), under the Monitor tool: `"<repo>/run/lib/lidar_reference/watch_jetson_replay.sh" s2_static_10 f3dof`.
Progress: `~/jobs/s2_static_10_lidar_ref_f3dof_jetson.progress` on the jobs page (<jobs page, port 8096>).
**Never during a drive** (the script refuses/stops itself; also see the risk below).

Optional, to measure the Jetson's own repeat spread (recommended once): rename the output folder and run again,
e.g. `mv ~/lidar_replays/s2_static_10/lidar_ref_f3dof ~/lidar_replays/s2_static_10/lidar_ref_f3dof_A` then the same command.

### Validation (1 min)
```bash
cd "<repo>/run/lib/lidar_reference" && python3 validate_against_robot.py --jetson ~/lidar_replays/s2_static_10/lidar_ref_f3dof --robot "../../01_runs/series2_static/s2_static_10/lidar_ref_f3dof" --robot-params validation_s2_static_10/robot_params_db.txt --downstream "../../01_runs/series2_static/s2_static_10" --out validation_s2_static_10
```
(add `--repeat ~/lidar_replays/s2_static_10/lidar_ref_f3dof_A` if the replay was run twice)

**Pass lines, stated before the Jetson replay** (robot: 947 nodes, 76 loop closures, floor gap 0.044 m, 263.3 m):

| # | check | pass line | what passing / failing means |
|---|---|---|---|
| C0 | same input: `wheel.tum` | identical, row for row | fail = the two did not read the same recording; stop there |
| C1 | parameters the launch sets (+ Force3DoF) | all equal in the Jetson map (and the robot's, with R3) | fail = a setting did not arrive; other version-default differences are listed, not failed |
| C2 | **path agreement** (main check): Jetson path at the robot's node times, rigid fit, no resizing | median <= **0.04 m**, 95th percentile <= **0.13 m**, >= 90 % of nodes paired | pass = the two LiDAR paths lie on top of each other to within half of the spread one drive's camera-vs-LiDAR number already has (0.08 / 0.27 m), so switching machines cannot move any verdict |
| C3 | floor end gap | within 0.05 m of 0.044 m | fail = the Jetson's map does not close its loop like the robot's |
| C4 | loop closures | 61-91 (+-20 %) - a judgement band, reported, not counted | the closure count is timing-sensitive; C2 is what matters downstream |
| C5 | nodes / path length | +-3 % of 947 / +-1 % of 263.3 m | fail = the map sampled the drive differently |
| C6 | downstream: drive 10's camera-vs-LiDAR agreement recomputed with the Jetson yardstick (`compare_lidar.py`, unchanged) | differs from 0.685 / 1.151 m by <= 0.04 / 0.13 m | pass = every drive-10 number in RESULTS.md would stand |

Verdict PASS = C0, C1, C2, C3, C5, C6. *Controls run 27 Sept:* the robot's own result against itself gives
0.000 m and reproduces 0.685 / 1.151 m exactly; 15 mm of added noise passes (0.018 / 0.040 m); 150 mm fails
(0.168 / 0.355 m).

**After a PASS:** future drives' yardsticks are made on the Jetson (pull, then replay); record the result in
docs/SOLVED.md. **Then delete the Jetson copy of the recording** (`~/lidar_bags/s2_static_10_lidar.bag`; the
original stays on the robot card) - the Jetson must be back to >= 13 GB free before the next drive (rule 24).
The replay folder's map (`rtab_helios.db`, ~0.2-1 GB) goes the storage-sense way; the small products go to the run's folder in git.

## Installed (docs/INSTALLED.md, 27 Sept)

Nothing system-wide; no sudo. `~/lidar_slam_ws/`: rslidar_sdk built (catkin_make, 24 s); 25 Ubuntu/ROS packages
(`ros-noetic-robot-body-filter` 1.3.2, `ros-noetic-sensor-filters` 1.1.1 and the 23 they need - moveit core/perception,
geometric_shapes, fcl, octomap, libomp ...) downloaded with `apt-get download` and unpacked with `dpkg -x` into
`~/lidar_slam_ws/deps/root` (75 MB; list + sha256 in `deps/debs.sha256`). A system install would have been
`sudo apt-get install ros-noetic-robot-body-filter ros-noetic-sensor-filters` (25 new packages, none upgraded) -
not needed.

## Risks / open points

- **Other Jetson tools find "the mapping program" by name alone.** `start_drive.sh` (step 3: `ps ... $2 == "rtabmap"`)
  would also see a replay's rtabmap.
  The replay stops itself within 5 s of `start_drive.sh` appearing, but closing its map takes a little while - so
  **do not start a drive while a Jetson replay runs**; stop it first (`touch ~/slam_series2/HELPERS_STOP`).
- `robot_replay_config.yaml` is a reconstruction until R1 confirms it.
- The robot's copies of `mcm07_husky` files and the rslidar_sdk source were not checksummed on the robot before
  (R2 does it); the self_navigation files match the robot's drive-10 checksums exactly.
- Ouster decode is not built: the f3dof yardstick uses the Helios only.
