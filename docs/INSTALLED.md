# What has been installed, on which machine, and why

**Why this file exists.** The Jetson has been reflashed once and will be again.
A list of exactly what was added to it is the difference between rebuilding in an
hour and rediscovering it over days. Same for the robot's computer.

**The rule that governs this file** is ENGINEERING_NOTES.md section 0.3 rule 11: install what
the work needs rather than stopping to ask, but **name it on the Jetson before
doing it, ask first on the robot, and record every install here.**

## How to read the machine column

| name | what it is |
|---|---|
| **Jetson** | the AGX Orin, internal disk. Carries ROS Noetic, the ZED SDK and three SLAM builds. Say what and why first. Prefer a virtual environment or a home-directory build over a system package. |
| **robot** | the Husky's own computer, ROS master for the wheels. **Ask before every install.** Breaking it stops all driving. |

## Verifying an install

An install that printed no error is **not** proof the tool works — the package can
land in a different Python from the one that runs by default. Always follow an
install by invoking the thing:

```bash
python3 -c "import matplotlib; print(matplotlib.__version__)"
```

A version number is proof. A silent apt log is not.

## The log

| date | machine | what | command | needed for |
|---|---|---|---|---|
The entries are the dated sections below, one per install.

**A lesson kept because the mistake is instructive.** A script
failed once with `ModuleNotFoundError: No module named 'matplotlib'`, and that
single failure was taken as proof the package was absent. It was not:
`matplotlib 3.10.7` and `numpy 2.3.5` were already installed and import correctly
in the identical invocation. Nothing was installed. **One failed import is not an
inventory** — check with an explicit import before concluding something is
missing, exactly as this file requires checking after installing.

## Known-present, verified 2026-08-31

| machine | package | version | how verified |
|---|---|---|---|
| Jetson | rtabmap | 0.21.13 | `rtabmap --version` |
| Jetson | rtabmap-export, -info, -reprocess, -databaseViewer | with rtabmap | `command -v` each, all in `/opt/ros/noetic/bin` |
| Jetson | sqlite3 **command-line tool** | **ABSENT** | `command -v sqlite3` → nothing. Python's `sqlite3` module is present and was used instead. |
| Jetson | Python | 3.8 (pinned by ROS Noetic) | — |


## 2026-09-19 — s3_drive.py onto the robot

| | |
|---|---|
| what | `s3_drive.py`, a straight-line constant-speed driver for Scenario 3 |
| machine | **the robot** (<old-robot-address>), `/home/administrator/s3_drive.py` |
| command | `rsync -a s3_drive.py robot:/home/administrator/s3_drive.py` then `chmod +x` |
| why | Scenario 3 reproduces Nicolas's Test 3 at 0.40 and 0.85 m/s. A joystick cannot hold a speed, and the speed is the experimental condition. `move_base` was rejected for the same reason — a planner varies speed as it re-plans. |
| installs | nothing. No packages, no ROS node added to startup, nothing written into Nicolas's catkin workspace. |
| verified | md5 `1e03e7844cc1e2b7320d27e4ae4b6660` identical both ends; `--dry-run` on the robot read live wheel odometry and published nothing |
| permission | asked and granted 2026-09-19 15:07 Hamilton |

## 2026-09-20 — Helios LiDAR decoder, on the JETSON

Needed to turn the Scenario 3 bags' raw `/helios/packets` into point clouds, so
Nicolas's frozen `sphere_centroid.py` can run on the laser side and produce the
ground-truth distance GEOMETRY.txt requires. Without it Scenario 3 has no reference
to compare the camera against.

**System packages** (the only apt work; user ran `sudo -v` first):

```bash
sudo apt-get install -y libpcap-dev libspdlog-dev
```

**A separate catkin workspace, `~/lidar_ws`** — deliberately NOT
`~/catkin_ws`, which carries the ZED pipeline this whole project runs
on. A failed build there would cost far more than the separation saves.

| package | source |
|---|---|
| `rslidar_sdk` | copied from `robot_mirror/<robot>/catkin_ws_src/` — the robot's own |
| `rslidar_msg` | **written here**, see below |

**Why `rslidar_msg` is ours.** RoboSense publish it only for ROS 2 — every branch
(main, master, dev, dev_opt, release) declares `ament_cmake` and `rclcpp`, so none
builds under Noetic, and the robot's mirror omitted the package entirely. It is a
data definition, not code: four fields, copied byte-for-byte from upstream, and
checked against the recordings themselves — the bags declare
`rslidar_msg/RslidarPacket` with exactly those slots in that order and 1248 bytes of
data per message, the standard MSOP packet. Source in
`02_zedx_pipeline/rslidar_msg/`.

**The Ouster is NOT built.** Its ROS wrapper in the mirror is version 0.12.1, which
upstream never tagged — its own changelog says "unreleased" — and the `ouster-sdk`
submodule it needs was not mirrored. Pairing a dev-snapshot wrapper with a released
client risks an API mismatch, and Helios alone gives the ground truth at 1500
packets/s against the Ouster's 640. The sources are parked at
`~/lidar_ws/parked/`, not deleted.

**Verify by invoking:**

```bash
ls -l ~/lidar_ws/devel/lib/rslidar_sdk/rslidar_sdk_node      # on the Jetson
```

## 2026-09-26 ~02:00 Hamilton - robot: storage/health reporter starts at every boot
- **Machine:** the robot's computer, user `administrator`. No sudo, nothing installed, colleague's files untouched.
- **What:** `~/slam_autostart_storage.sh` (robot home disk) + one line in the administrator crontab:
  `@reboot /home/administrator/slam_autostart_storage.sh`. It waits up to 10 min for the 128 GB card to mount,
  then runs `robot_side.sh storage -` (starts only `robot_storage_agent.py` on :8113; refuses a second copy).
  Log: `~/jobs/slam_autostart_storage.log` (robot).
- **Why:** - the robot
  is off while charging, so the status pages lost the robot after every charge until started by hand.
- **Verified:** started now (pid 15326, /health.json and /storage.json answer from the Jetson); the starter run in
  cron's bare environment reports "already running". The real boot path is proven only at the next robot boot.
- **Undo:** `crontab -r` on the robot (it was the only line) and `rm ~/slam_autostart_storage.sh`.

## 2026-09-26 07:55 Hamilton - Jetson: live status sender kept alive (cron)
- What: `~/slam_series2/tools/keep_live_push.sh` (in this repository: `tools/live_site/keep_live_push.sh`),
  run by the Jetson crontab `@reboot sleep 60 && ...keep_live_push.sh` and `* * * * * ...keep_live_push.sh`.
- Does: starts `slam_live_push.py` (writes the snapshot the Jetson-served website shows) if it is not running (whole-argument
  process check with its absent-name control). Log: `~/jobs/keep_live_push.log`.
- Why: the sender died at 07:33 when the disk filled and nothing restarted it. Remove both cron lines to undo.

## Disk space on the Jetson (no install)
- Nothing to install: `catkin_ws/src/sidewalk_bringup/scripts/storage_monitor.py` shows both Jetson disks on a web
  page (port 8092). Move finished recordings to the Autonomous Service Robot Teams folder, check the copy (download it
  again and compare size and checksum), then delete them from the Jetson.

## 2026-09-27 ~23:45 Hamilton - wifi-roam-helper systemd service (Jetson)
- What: `/etc/systemd/system/wifi-roam-helper.service` running `~/slam_series2/tools/wifi_roam_helper.py` as root (no package).
- Command: `bash ~/slam_series2/tools/install_wifi_roam_helper.sh` after the user's `sudo -v`.
- Why: the Realtek WiFi driver (rtl88x2ce) cannot report signal changes, so wpa_supplicant re-scanned only every 5 min;
  drive 9 lost its link for 5 min. The helper checks every 2 s, roams below -67 dBm, reassociates after 15 s with no
  gateway, keeps power saving off. Verified: `systemctl is-active` = active, progress line shows -55 dBm, link ok.
- Robot: installed 2026-09-26 ~23:50 by the user in their robot terminal (same installer, args: card script path, /home/administrator/jobs); active, -61 dBm, link ok; power saving turned off by the helper.

## 2026-09-27 ~09:10 Hamilton - ZED people detector optimised for this Jetson (Jetson, no package installed)
- What: the ZED SDK 4.2.5 wrote its one-time TensorRT optimisation of the object-detection model:
  `/usr/local/zed/resources/.objects_performance_3.2.model_optimized-fbcbl-...` (21 MB, owner sidewalk).
- Command: none - it happens the first time the camera starts with
  `zedx_front_od.yaml` ( detector on, MULTI_CLASS_BOX_FAST, people only). No sudo, no network download.
- Why: the SLAM session's people mask (MASK=1). Without the file the detector's first start takes 649 s with NO
  camera pictures; with it, 2 s. Verified: `ls /usr/local/zed/resources/.objects_performance*`; bench 03 ran the
  detector at 13-15 Hz. Undo: delete that file (the next start re-optimises for 11 min).

## 2026-09-27 ~10:00 Hamilton - Jetson: the LiDAR yardstick workspace `~/lidar_slam_ws` (home folder, no sudo)
- What: a separate catkin workspace for replaying the colleague's Helios LiDAR mapping on the Jetson (SLAM work,
  rule 16; the depth study's `~/lidar_ws` is left untouched). (1) `rslidar_sdk` v1.5.19 copied from
  `~/robot_mirror/<robot>/catkin_ws_src/` and built; (2) `self_navigation` (matt_self_navigation) and
  `mcm07_husky` copied unchanged into `~/lidar_slam_ws/nobuild/` (found by rospack, never compiled);
  (3) `ros-noetic-robot-body-filter` 1.3.2 + `ros-noetic-sensor-filters` 1.1.1 and the 23 packages they need
  (moveit-core, moveit-ros-perception/-planning/-occupancy-map-monitor, geometric-shapes, fcl, octomap, srdfdom,
  cras-*, point-cloud-transport, ruckig, pybind11-catkin, libomp ...) as .deb files UNPACKED into
  `~/lidar_slam_ws/deps/root` (75 MB) - not installed into the system; plus an empty marker file
  `deps/root/opt/ros/noetic/.catkin` so catkin/roslaunch look there for node programs.
- Commands: `catkin_make -DCMAKE_BUILD_TYPE=Release -j6` in a clean environment (`env -i`, only /opt/ros/noetic
  sourced; 24 s); `apt-get download <the 25 packages>` then `dpkg -x <each>.deb ~/lidar_slam_ws/deps/root`
  (list + sha256: `~/lidar_slam_ws/deps/debs.sha256`). Environment: `source ~/lidar_slam_ws/setup_lidar_slam.bash`
  (drops whatever the shell had sourced, incl. ~/catkin_ws).
- Why: the robot had to stay on ~35 min after every drive to make its LiDAR yardstick; on the Jetson it can charge.
  A system install (`sudo apt-get install ros-noetic-robot-body-filter ros-noetic-sensor-filters`, 25 new, 0 upgraded)
  was avoidable, so it was avoided (rule 11: prefer a home-folder install on the Jetson).
- Verified by invoking: `rospack find` resolves all 7 packages; `ldd` of the filter program and plugin: nothing
  missing; a dry test PASSED (all five nodes, clouds
  through every stage, map written and closed). Undo: `rm -rf ~/lidar_slam_ws`.

## 2026-09-27 (Hamilton) - Jetson - python-docx + cairosvg in a home-folder virtual environment
- Command: `python3 -m venv ~/venvs/report && ~/venvs/report/bin/pip install --upgrade pip && ~/venvs/report/bin/pip install --only-binary=:all: "lxml<6" && ~/venvs/report/bin/pip install python-docx cairosvg`
- Why: write Word documents and turn SVG diagrams into PNG pictures (used to check this repository's diagrams).
  Home-folder environment only; no system package touched.
- Trap: lxml 6 has no prebuilt package for this Jetson's Python 3.8 and fails to build (missing libxml2/libxslt headers);
  `lxml<6` installs from a prebuilt package.
- Verified: `~/venvs/report/bin/python -c "import docx, cairosvg, lxml"` -> lxml 5.4.0, cairosvg 2.7.1.

## 2026-09-29 Hamilton - Jetson: replay web page backend kept alive (cron)
- **What:** `tools/replay_web/replay_control_server.py` (the "Replay" tab's backend, 127.0.0.1:8098), kept running
  by two lines in the Jetson user crontab:
  `* * * * * REPO_ROOT=<repository> <repository>/tools/replay_web/keep_replay_control.sh` and
  `@reboot sleep 60 && REPO_ROOT=<repository> <repository>/tools/replay_web/keep_replay_control.sh`
  (add `EXTRA_RECORD_DIRS=<folder>` to list recordings kept outside `~/.run_records` and `~/replay_inputs`).
- **Why:** lab students start and watch replays from the live website; the backend must survive restarts like
  `live_site_server.py` does. Nothing new was installed (standard-library Python, system matplotlib).
- **Verify:** `curl -s 127.0.0.1:8098/replay/api/status` answers JSON; its log is `$JOBS_DIR/replay_control_server.log`.
- **Undo:** remove both cron lines; stop the backend by its process number.
