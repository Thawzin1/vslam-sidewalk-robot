# REPLAY.md - re-running a recorded mapping drive without the robot

*For the student who continues this work: everything here was checked against the source code on
the Jetson (file and line numbers are given), not taken from memory. Plain-language explanations are
in brackets the first time a technical word appears.*

## 1. What a replay is, and what it is for

Every mapping drive of series 2 left two recordings behind:

| file | what it holds | where it lives |
|---|---|---|
| `fusion.bag` | a ROS bag (a time-stamped log of ROS messages) with the robot's wheel odometry (the distance the wheels say they rolled), its gyroscope, the camera's calibration and everything the live programs computed | `~/.run_records/<run>/` during a drive; long-term in the Teams folder at `series2_raw/<run>/<run>/` |
| `<run>.svo2` | the ZED X camera's own recording: both stereo pictures of every frame plus the camera's motion sensors. Depth is NOT stored; the ZED SDK computes it again on playback, in any depth mode | same folders |

The replay plays the two back together through the **same programs the live drive used** and
builds a **new map** from them. That makes the drive repeatable: change one setting (a depth mode, an
RTAB-Map parameter), replay, and any difference in the result is caused by that setting alone
(ENGINEERING_NOTES.md section 4, rule 5: record once, replay many). No robot, no lab, no camera needed - only
the Jetson (or any machine with the ZED SDK, ROS Noetic and this repository's workspace).

## 2. How to run it

```bash
# 1. the recordings must be on this machine (section 8 says how to get them there):
#    download series2_raw/<run>/<run>/ from the Teams folder, copy it to ~/replay_inputs/slam/<run>/

# 2. the replay (about as long as the recording itself, plus a minute of start-up):
run/replay.sh <run>                       # defaults: NEURAL depth, output in ~/slam_series2/replay/<run>/
run/replay.sh <run> --svo /path/x.svo2 --bag /path/fusion.bag --out /path/out   # explicit files
run/replay.sh <run> --max-s 120           # stop after 120 s of playback
run/replay.sh --help                      # every option
```

Watch it live on the jobs page (the line `REPLAY <run> <played>/<total> s  <elapsed>s` in
`~/jobs/<run>_replay.progress`). When it finishes, `<out>/replay_summary.txt` prints the numbers and
`<out>/replay_summary.json` holds them for scripts.

Paths come from environment variables so nothing is hard-wired to one machine:
`REPO_ROOT` (the repository), `RECORDS_DIR` (default `~/.run_records`), `WORK_DIR` (default
`~/slam_series2`, the small database tools are in `$WORK_DIR/tools`), `JOBS_DIR` (default `~/jobs`),
`REPLAY_ROS_PORT` (default 11399).

## 3. How the data flows

```
                 (paused until the first picture; publishes the recorded time on /clock)
 fusion.bag ===> rosbag play ==============================+==> /robot/wheel_odom  /robot/imu  /tf_static
                                                           |
 <run>.svo2 ===> ZED wrapper, file playback, real time ====+==> pictures + depth (recomputed) + camera IMU
   (replay_camera.launch)                                  |
                                                           v
                              ekf_input_conditioner.py  +  ekf_fused         (fused_odometry.launch)
                              the blend: wheels + gyroscope + camera motion -> TF odom => base_link
                                                           |
                                                           v
                              rgbd_sync => rgbd_odometry => rtabmap          (record_mapping_run.launch)
                                                           |
                                                           v
                 <out>/<run>_replay.db   +   replay_outputs.bag   +   replay_summary.json
```

Only the robot's **inputs** are played from the bag (`/robot/wheel_odom /robot/imu /robot/imu_raw
/tf_static`). Everything the live programs computed (`/tf`, `/fused/*`, `/ekf_in/*`, `/rtabmap/*`)
stays in the bag unplayed, because the replay computes it again.

Pieces (all in this folder, meant to be merged into `catkin_ws/src/sidewalk_slam/`):

| file | job |
|---|---|
| `run/replay.sh` | the one command: finds the files, starts a private ROS master, starts everything in order, watches, stops in the right order, writes the summary |
| `launch/replay_drive.launch` | bag player (paused, `/clock`), the blend, the mapping, a small output recorder |
| `launch/replay_camera.launch` | the camera from its recording, in a launch of its own (section 5.3 says why) |
| `scripts/replay_svo_index.py` | reads every frame's capture time from the SVO with the ZED Python SDK (once per recording) |
| `scripts/replay_starter.py` | resumes the paused bag on the first camera picture, measures the offset and holds the bag to remove it (section 5.1) |
| `scripts/replay_watch.py` | the progress line for the jobs page + the clock-drift evidence (`clock_drift.csv`) |
| `scripts/replay_summarize.py` | node and closure counts, both start-to-end gaps, the clock drift -> `replay_summary.json` |
| `scripts/replay_compare.py` | sets the replay beside the original drive over the same time window (evo, rigid alignment only) |

## 4. The timing question, settled from source

The whole design rests on one question: **when the ZED wrapper plays a file back, what time does it
write on its messages, and does it play at the recorded speed?**

Checked in `catkin_ws/src/zed-ros-wrapper/zed_nodelets/src/zed_nodelet/src/zed_wrapper_nodelet.cpp`
(the build on the Jetson) and `/usr/local/zed/include/sl/Camera.hpp` (SDK 4.2.5):

1. **Pictures are stamped with the CURRENT ROS time, not the recorded time.** Lines 4089-4095:
   `if (mSvoMode) mFrameTimestamp = ros::Time::now(); else ... getTimestamp(TIME_REFERENCE::IMAGE)`.
   The SDK image time (line 4095) is used only with a live camera. The same choice is made at
   3115-3117 (image data) and 3812-3814 (start-up). So the file's own timestamps never appear on the
   picture topics.
2. **The camera IMU topic carries BOTH kinds of stamp in playback.** The grab loop publishes sensor
   data with the picture's stamp (3134-3137, `publishSensData(stamp)`), while the separate sensor
   thread (started unconditionally at 663) calls `publishSensData()` with `t == 0`, which in SVO mode
   reads `getSensorsData(TIME_REFERENCE::IMAGE)` and stamps with the ORIGINAL recorded time
   (3462-3489). Harmless for the programs downstream (both agree to within the offset measured in
   section 6) and useful: it is one of the two drift witnesses.
3. **`svo_realtime` paces playback by the wall clock.** Read at 1173 into `mSvoRealtime`, passed to
   `InitParameters.svo_real_time_mode` at 207. `Camera.hpp:7684-7693`: real-time mode "will bring
   the SDK closer to a real simulation ... by using the images' timestamps"; "frames will be dropped
   when playing too slowly". The wrapper's loop also sleeps with `ros::Rate(mPubFrameRate)` (3801).
4. **The wrapper never reads `use_sim_time`** (no match in the file). It follows it indirectly:
   `ros::Time::now()` and `ros::Rate` use `/clock` when `/use_sim_time` is true.
5. **End of file: the node does NOT exit on this build.** There is an exit for
   `END_OF_SVOFILE_REACHED` at 4015-4020, but it sits behind `if (mGrabStatus != CAMERA_REBOOTING)
   { ...; continue; }` (4010-4014), so it is never reached: the node spins at the end of the file
   printing "END OF SVO FILE REACHED" warnings (seen in the test, `camera.log`). `replay_watch.py`
   therefore declares the end itself - from the wrapper's frame counter in `/diagnostics`
   ("Playing SVO  Frame: k/N", 4518-4523) or when the bag clock passes the last frame's capture
   time - and `replay.sh` stops on that.
6. `rosbag play --pause` publishes a standing `/clock` while paused and offers
   `/<node>/pause_playback` (std_srvs/SetBool, `data: false` resumes) - both verified on this build
   with a test bag.

**What follows for the design.** Under `/use_sim_time true` with `rosbag play --clock`, the
wrapper's "now" IS the bag's timeline, so replayed pictures land on the wheel data's time axis by
themselves. What remains is pacing: the SDK's real-time mode and `rosbag play` at rate 1.0 both
follow the wall clock, so the streams advance together. What is left is a constant START offset
(the camera's start-up time), which `replay_starter.py` measures and removes (section 5.1), and any
slow drift, which `replay_watch.py` measures for the whole run. The recorded SVO time base is the
same clock as the bag's: in run `s2_fusion_T5b` the SVO's first frame time equals a live
`camera_info` stamp in the bag to the microsecond, so the SVO's frame times can be used directly
to position the bag.

### Why the rate is fixed at 1.0

The SDK paces by the wall clock, `rosbag play --rate 0.5` would pace by half of it, and the two
would separate by one second per two seconds played. The apparent alternative - `svo_realtime
false`, every frame processed, paced by the wrapper's own 15 Hz loop under the bag's clock - was
rejected without testing: it consumes exactly 15 frames per bag-second, but the recording averaged
13.9 frames/s with irregular gaps, so the camera stream would run about 8 % fast and drift 12 s over
a 150 s recording. Slower-than-real-time replay needs the fallback in section 7.

## 5. Decisions worth knowing

### 5.1 The start - a measured sync, not a guess
`replay_svo_index.py` reads the SVO once (about 65 frames/s on the Jetson; a 20-minute recording
takes 4-5 minutes) and saves every frame's capture time. The camera then takes 10-30 s to open the
file (loading the NEURAL depth model). The SDK starts its real-time playback clock at the first frame
it reads, but the first picture reaches ROS a variable 0.3-1 s later (the first depth computation).
Resuming the bag on that first picture therefore left the camera AHEAD of the wheel data by an amount
that changed between runs of the same file: **-0.26 s in test 1, -0.71 s in test 2**.

So `replay_starter.py` closes the loop:

```
 bag paused at (first frame time + 3 s margin)          camera opens, first picture appears
        |                                                         |
        +------------------ resume the bag <----------------------+
        |   now the bag is AHEAD by (3 s - camera start-up), unknown but positive
        v
 measure it for 2 s: camera IMU messages carry the ORIGINAL recorded stamp in playback,
 so  offset = (bag clock) - (that stamp)
        |
        v
 pause the bag for exactly 'offset' seconds (the SDK keeps playing by the wall clock) -> resume
        |
        v
 measure again, write unpaused.json: offset before / hold applied / offset after
```

*Plain terms: start the wheel recording a little early on purpose, see by how much it is early,
and hold it back by exactly that much.* The first ~5 s of the camera recording play before the fine
sync is done, so a replay should start from a moment the robot was parked (every series-2 drive
starts parked for at least a minute).

### 5.2 The order of shutdown
When the camera recording has been played out (or the bag ends, or `--max-s`), `replay.sh` interrupts
the bag player and the camera launch, waits 5 s, then interrupts **RTAB-Map itself** and waits with
no deadline for it to close its database, and only then interrupts the launch. roslaunch alone
gives each node 15 s before killing it; a map still writing would be cut short (that is how drive 1
lost its corrected positions, `auto_stop_mapping.py` header).

### 5.3 Why the camera has its own launch file
`zed_camera.launch.xml:69` starts the camera node with `required="true"`; when a required node
exits, roslaunch shuts down everything in that launch. The wrapper's source intends to exit at the
end of the file (and a fixed build would), and a crash of the camera node (three happened in one
night, `camera_segfault_2026-09-26`) has the same effect. In one launch either would take RTAB-Map
down under the 15 s limit; in its own launch, `replay.sh` notices and closes the map properly.

### 5.4 Settings kept identical to the live drives, and the one that is not
NEURAL depth, tracker `GEN_2`, `vo_publish_tf:=false` (the blend owns odom => base_link),
`force_3dof`, `wait_imu_to_init`, the same `rtabmap_zedx.yaml` and `ekf_fused.yaml` - all through
the same launch files the drives used, so a change there reaches the replay too. The one
deliberate difference: `self_calib:=false` (the SDK does not re-estimate the lens-to-lens geometry
at start), so that two replays of one recording agree; `--self-calib true` restores the live
behaviour.

## 6. Test results

(TEST_REPORT.md has the exact commands and outputs.)

Test drive: `s2_fusion_T5b` (2026-09-26): parked for a minute, then about 10 m out and back twice,
parked again. Two camera files; the second (`s2_fusion_T5b_b.svo2`, 9562 frames, 684.9 s) covers the
whole motion. The original map database was downloaded from the Teams folder for this comparison and deleted
afterwards; its trajectories are kept in `replay_test/original/`.

**Replay against the original drive** (test 3, the moving segment; one replay, so N = 1):

| figure | original drive | replay | what it means |
|---|---|---|---|
| map nodes | 588 (whole drive) | 642 | the replay's mapping step ran faster (0.46 s against 0.70 s mean per update), so it kept more places; the live Jetson was also capturing, recording the SVO and serving pages |
| loop closures (one per pair) | 48 = 14 global + 34 proximity | 66 = 21 global + 45 proximity | more nodes, more chances to recognise a place; the drive's motion lies entirely inside the replayed window |
| odometry resets (run_monitor) | 137 | 167 | the camera tracker loses itself about as often: frames with quality 0 in the same window 403 of 6564 live, 419 of 7127 replayed (6.1 % and 5.9 %) |
| start-to-end gap, tracking alone (`Node.pose`) | 0.091 m | 0.153 m | how far the camera's own tracking ended from where it started; both kinds are rule 20 figures |
| start-to-end gap, map's corrected positions (`Admin.opt_poses`) | 0.095 m, 48 closures | 0.109 m, 66 closures | how well the map recovered |
| path length (tracking alone, same window) | 47.36 m | 47.53 m | agree to 0.4 % |
| difference between the two paths after one rigid alignment (evo_ape -a) | - | tracking alone: median 0.084 m, rmse 0.105 m, max 0.232 m; corrected: median 0.045 m, rmse 0.092 m, max 0.250 m | the replay draws the same route to within about 5-10 cm |

*Plain terms: the replay drew the same route, the same length, and closed back on the start almost
as tightly as the live drive. It is not a copy - it kept different pictures - and the numbers show
how much that matters: centimetres.* Before reading these as agreement, remember ENGINEERING_NOTES.md
section 4 rule 0: two runs of one instrument over the same data differ by construction, and N = 1
here does not tell how much two replays differ from each other.

**Clock drift between the camera stream and the bag** (offset = bag clock minus the capture time
of the picture being shown; negative = the camera is ahead):

| test | start sync | offset at start | offset at end | drift | over |
|---|---|---|---|---|---|
| 1 (parked, 152 s, first starter) | resume on first picture | -0.257 s (SVO counter) / -0.337 s (IMU) | -0.234 / -0.339 s | +0.023 / -0.002 s | 150 s |
| 2 (parked, same file, first starter) | resume on first picture | -0.712 / -0.792 s | -0.699 / -0.785 s | +0.013 / +0.008 s | 150 s |
| 3 (moving, 685 s, final starter) | measured and held (2.767 s) | -0.056 / -0.071 s | -0.027 / -0.061 s | +0.029 / +0.009 s | 685 s |
| 4 (parked, 60 s, final code check) | measured and held (2.785 s) | +0.023 / -0.066 s | +0.054 / -0.056 s | +0.031 / +0.010 s | 60 s |

The two witnesses (the wrapper's frame counter, and the camera IMU messages that keep their original
stamp) agree to 10-80 ms; the frame counter is only reported once a second, so it is the coarser one.
**Once started, the streams stay in step: the drift is under 0.03 s over 11 minutes.** The start
offset was the real problem (it varied from -0.26 s to -0.71 s between two runs of the same file),
and the measured sync brings it to about -0.06 s. That remainder is the time the wrapper takes to
turn a frame into a picture, which the live drive had as well (camera messages then arrived 0.177 s
after their stamps).

*Plain terms: the two recordings start together to within about a fifteenth of a second and do
not wander apart.*

## 7. Limits - what a replay reproduces and what it cannot

- **The same measurements, not the same frames or the same timing.** The SVO holds the frames the
  live camera kept (14.0 per second of the sensor's 30). On playback the SDK's real-time mode
  delivers them by the wall clock, and the programs downstream take whichever they keep up with.
  The replayed Jetson carries less load than the live one did (no live capture, no recording, no
  web pages), so RTAB-Map ran faster (0.46 s against 0.70 s per update) and kept more places (642
  against 588 nodes). Node counts, closure counts and the tracking-alone gap therefore differ from
  the original; two replays will also differ from each other a little. A replay on a busier machine
  would keep fewer.
- **The first ~5 s of the recording play before the fine sync** (section 5.1) and are about 0.1-3 s
  off; start from a parked moment.
- **Depth is recomputed**, which is a feature (any depth mode can be tried) and a caveat (the SDK's
  depth is not bit-identical between runs; `self_calib:=false` removes one source of variation).
- **Only what the SVO covers can be replayed.** A drive's SVO may cover part of the drive (T5b's first
  file covers the 152 s the robot stood parked; the second file the drive itself).
- **The blend's parked calibration window is shorter.** Live, the conditioner learned the gyroscope
  offset from the first parked minute; the replay starts at the SVO's first frame, so the bag's
  first seconds before that are not played.
- **The camera IMU arrives with two stamps** (section 4, point 2), so the blend's 50 Hz binning of it
  is not identical to live. The conditioner handled it without error in the tests.
- **Rate 1.0 only** (section 4). A machine slower than the Jetson drops more frames; the replay then
  tells you about that machine, not about the drive.
- **One replay per machine at a time.** Two NEURAL depth computations on one Jetson would each
  drop frames; the private master port (`REPLAY_ROS_PORT`) exists so a replay never disturbs a live
  drive, not so that several replays can share the GPU.
- **The camera node does not exit at the end of its file** (section 4, point 5); the replay stops it.
  A future wrapper that does exit is handled too (section 5.3).

### The fallback design (not built - not needed)
If real-time pacing had proved unreliable: step A converts the SVO once into a ROS bag of compressed
pictures (`compressed` JPEG colour + `compressedDepth` PNG depth, or the raw stereo pair) stamped
with the SDK's own times; step B plays that bag and `fusion.bag` together with `--clock`, in perfect
step at any rate. It costs disk (a 20-minute drive is tens of GB as images) and fixes the depth mode
at conversion time. It was not needed: section 6 shows real-time pacing stays in step to under 0.03 s over 11 minutes.

## 8. Getting a run onto the Jetson

Every run is in the **Autonomous Service Robot** Microsoft Teams folder (access: ask Prof. Moein
Mehrtash, McMaster University), under `series2_raw/<run>/`; `data/DATA_INDEX.md` lists every file and
its size. Download the run's folder on any computer, then copy it to the Jetson over the lab network:

```bash
# on your computer, from the folder you downloaded into (-s keeps paths with spaces intact):
rsync -s -av --progress <run>/ <jetson-user>@<jetson>:~/replay_inputs/slam/<run>/
# or:  scp -r <run> <jetson-user>@<jetson>:~/replay_inputs/slam/
# then on the Jetson:
run/replay.sh <run> --svo ~/replay_inputs/slam/<run>/<run>/<run>.svo2 --bag ~/replay_inputs/slam/<run>/<run>/fusion.bag
```
The run record (`<run>/<run>/`) holds `fusion.bag`, `<run>.svo2`, the logs and `monitor.csv`; the
original map database `<run>.db` sits one level up. Some runs have a second camera file in a sibling
folder (`<run>_b/`) when the recording was restarted mid-drive - pass it with `--svo`. Keep 13 GB
free on the Jetson before copying a run in (the disk-space page on port 8092 shows it).
