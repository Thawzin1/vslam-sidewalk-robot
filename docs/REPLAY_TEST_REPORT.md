# TEST_REPORT.md - building and testing the drive replay on the Jetson, 2026-09-29

Machine: Jetson AGX Orin (Ubuntu 20.04, ROS Noetic, RTAB-Map 0.21.13, ZED SDK 4.2.5, pyzed 4.2.5).
No robot connected; the camera was attached but not used. Everything ran under `nice -n 19` on a
private ROS master (port 11399). Two large file uploads by other jobs were running throughout.

## 1. Test data

```
# downloaded from the Teams folder (series2_raw/s2_fusion_T5b/) and copied to the Jetson:
~/handover_build/replay_test/s2_fusion_T5b/        <- series2_raw/s2_fusion_T5b/s2_fusion_T5b/ (without media/)
~/handover_build/replay_test/s2_fusion_T5b_b/      <- series2_raw/s2_fusion_T5b/s2_fusion_T5b_b/
~/handover_build/replay_test/original_db/          <- series2_raw/s2_fusion_T5b/s2_fusion_T5b.db  (3.3 GB, deleted again after extraction)
```

`rosbag info fusion.bag` (2.87 GB): start 1790394660.08, end 1790395652.81, 992.7 s. Topics as
inventoried, plus `/robot/imu_raw`, `/robot/ekf_odom`, `/diagnostics`; no images. `/robot/wheel_odom`
9289 msgs, `/robot/imu` 17963 msgs, `/zedx_front/zed_node/left/camera_info` 27742 msgs.

Run `s2_fusion_T5b` (2026-09-26, a fusion test): the robot stood parked for the first 569 s, then
drove about 10 m out and back twice (t = 569-930 s of the drive), 588 map nodes, 48 loop closures
(14 global + 34 proximity), 137 odometry resets, closed properly. Its camera recording was restarted
once, so there are two SVO files:

| file | frames | covers (drive time) | what the robot did |
|---|---|---|---|
| `s2_fusion_T5b.svo2`, 466 MB | 2115 (13.9/s) | 15.4 s to 167.7 s (152.2 s) | parked |
| `s2_fusion_T5b_b.svo2`, 2.09 GB | 9562 (14.0/s) | 299.8 s to 984.7 s (684.9 s) | parked, then the whole drive, then parked |

The SVO's first-frame timestamp equals a live `camera_info` header stamp in the bag to the microsecond
(checked with the rosbag Python API), so the two recordings share one clock. Live latency of the
camera messages was 0.177 s median (receive time minus stamp); wheel messages 0.004 s.

## 2. What was verified in the wrapper source before designing

See docs/REPLAY.md section 4. In short: playback stamps pictures with `ros::Time::now()`
(4089-4091), the sensor thread also publishes IMU with the original recorded stamps (663,
3462-3489), `svo_realtime` -> `svo_real_time_mode` (1173, 207; Camera.hpp 7684-7693), no
`use_sim_time` handling, and the end-of-file exit (4015-4020) is unreachable (4010-4014).
`rosbag play --pause --clock` keeps publishing a standing clock and offers `pause_playback`
(tested with a 3 s synthetic bag on a throw-away master, port 11398).

## 3. Runs

All runs used this command form (the paths are the test folders; a student would normally leave
`--bag`/`--svo`/`--out` out and use `$RECORDS_DIR`):

```
RECORDS_DIR=~/handover_build/replay_test JOBS_DIR=~/jobs nice -n 19 bash ~/handover_build/replay_dev/run/replay.sh \
    s2_fusion_T5b --bag <fusion.bag> --svo <recording.svo2> --out <folder> [--max-s 60]
```

| # | recording | outcome | what it taught / fixed |
|---|---|---|---|
| 0 | T5b parked | stopped at once: `RLException: Invalid roslaunch XML syntax: not well-formed (invalid token): line 12` | `--` is illegal inside an XML comment (my text diagram). Diagrams rewritten with `==>`; both launch files now checked with `xml.dom.minidom` |
| 1 | T5b parked | ran 150 s, never "saw" the first picture, timed out after 300 s; cleaned up everything (master down, no processes left) | (a) the starter set its done flag before the resume service call returned, so rospy shut down mid-call (`ServiceException: returned no response`) - fixed; (b) **the wrapper does not exit at the end of the SVO** (spins on "END OF SVO FILE REACHED", source 4010-4020) - `replay_watch.py` now declares the end. Its database, summarised by hand: 143 nodes, offset -0.257 s |
| 2 | T5b parked | **clean end to end**, 189 s wall: 140 nodes, 0 closures, tracking gap 0.017 m, offset -0.712 s, drift +0.013 s over 150 s | the start offset differs between runs of one file (-0.26 s, -0.71 s), so `replay_starter.py` was rewritten as a measured sync (REPLAY.md section 5.1) |
| 3 | **T5b_b, the drive** | **clean end to end**, 732 s wall for 685 s of recording; sync: bag ahead 2.767 s, held 2.767 s, after -0.066 s; closed the map in 3 s | the result below. `monitor.csv` had landed in `~/.run_records/s2_fusion_T5b_replay/` (the launch's default) - the launch now puts it in the output folder; the file was moved and the summary re-run |
| 4 | T5b parked, `--max-s 60` | **clean**, 101 s wall: sync 2.785 s -> -0.073 s, 60 nodes, 0 resets, drift +0.010 s (IMU witness); `monitor.csv` in the output folder | final code check after the last two fixes |

### Test 3 console output (the summary as printed)

```
2026-09-29 16:37:09 bag resumed and brought into step: offset before 2.767 s, held 2.767 s, after -0.066 s
2026-09-29 16:48:34 stopping: the camera recording ended (bag clock passed the recording's last frame time) (after 685 s of playback)
2026-09-29 16:48:42 RTAB-Map closed its database after 3s
2026-09-29 16:48:47 all stopped (732s wall time)
replay summary for s2_fusion_T5b
  map nodes               642
  loop closures           66 (21 global + 45 proximity)
  tracking-alone gap      0.153 m over 47.53 m of path (642 poses)
  corrected gap           0.109 m (274 poses, corrected poses saved: True)
  odometry resets         167
  clock offset by SVO frame counter  start -0.056 s, end -0.027 s, drift +0.029 s (643 samples)
  clock offset by camera IMU stamps  start -0.071 s, end -0.061 s, drift +0.009 s (3494 samples)
```
(Times in these logs are UTC, the Jetson's system clock; Hamilton time is 4 h earlier.)

### Comparison with the original drive

```
python3 ~/slam_series2/tools/db_to_tum.py         s2_fusion_T5b.db original/camera.tum --quiet
python3 ~/slam_series2/tools/db_corrected_tum.py  s2_fusion_T5b.db original/camera_corrected.tum --label "camera original"
python3 replay_compare.py --original ~/handover_build/replay_test/original --replay out/T5b_moving --out out/T5b_moving/compare.json

window 1790394962.806 .. 1790395644.203 (681.4 s)
kind             which       poses   length m    gap m
tracking_alone   original      397      47.36     0.09
tracking_alone   replay        642      47.53    0.153
tracking_alone   difference after rigid alignment: rmse 0.105 m, median 0.084 m, max 0.232 m
corrected        original      181      45.93     0.48
corrected        replay        274      46.08    0.109
corrected        WARNING: original starts more than 5 s after the window - its gap is not start-to-end; ...
corrected        difference after rigid alignment: rmse 0.092 m, median 0.045 m, max 0.250 m
```
The 0.48 m is not a gap: the original's saved corrected graph jumps from node 1 (t = 644) to t = 1210,
when the robot had already moved 0.38 m, so the window cuts off its start. The whole-drive figures from
the project's own tool (`end_gap_corrected_m`) are the fair comparison: **0.0955 m original, 0.1093 m
replay**. The tool now prints that warning itself.

Other checks, same window (the original's `mapping.log` against the replay's `replay_launch.log`):
- camera tracking lost (`Odom: quality=0`): 403 of 6564 live updates, 419 of 7127 replayed
- RTAB-Map time per update: 0.697 s mean live (25 over 1 s), 0.463 s replayed (4 over 1 s)
- `cannot be used` (a silent feature-detector downgrade, ENGINEERING_NOTES.md section 2.6): 0 matches in either replay log

The full table and what it means are in docs/REPLAY.md section 6.

## 4. Files left on the Jetson

| where (Jetson internal disk) | what | size |
|---|---|---|
| `~/handover_build/replay_test/s2_fusion_T5b/` | the run record: `fusion.bag`, the parked SVO, logs | 3.3 GB |
| `~/handover_build/replay_test/s2_fusion_T5b_b/s2_fusion_T5b_b.svo2` | the drive's SVO | 2.1 GB |
| `~/handover_build/replay_test/out/T5b_moving/` | test 3: the new map (1.3 GB) and every output | 1.4 GB |
| `~/handover_build/replay_test/original/` | the original drive's two TUM files + counts (its 3.3 GB database was deleted; the Teams folder keeps it) | 90 KB |
| `~/handover_build/replay_test/out/T5b_parked*`, `T5b_final_check` | tests 1, 2, 4 without their databases | < 40 MB |

Free space stayed above 7.5 GB throughout, so the large test files were left in place; all of it can
be fetched again from the Teams folder.

