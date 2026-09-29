# Hardware - what the robot is, what runs where, and what to trust

Project: VSLAM-Based Navigation for Autonomous Sidewalk Robots (VSLAM: visual simultaneous
localisation and mapping - the robot builds a map from camera images while working out where it is
on that map). Mitacs Globalink 2026, McMaster University. Supervisor: Prof. Moein Mehrtash.
Author: Thaw Zin.

This page is for someone who has never seen the hardware. Addresses, account names and passwords
are deliberately left out; they are listed, without values, in
Ask the project owner for them.

---

## 1. The robot

- **Base:** Clearpath Husky A200, a four-wheel, skid-steer (turns by driving the left and right wheels
  at different speeds) outdoor research robot. Manufacturer figures: about 50 kg, 130 mm ground
  clearance, 75 kg maximum payload (20 kg on rough ground).
- **Top deck:** PACS (Clearpath's accessory mounting system) - a grid of M5 threaded holes 80 mm
  apart, cells labelled A01 (front-left) to G08 (back-right). A mast on cell D05 carries the Ouster
  LiDAR, and our camera clamps to that same mast's support rods (section 5).
- **Also on the robot:** a second LiDAR (RoboSense Helios), a GNSS antenna (satellite positioning,
  not used indoors), and a red emergency stop button. **Find the emergency stop before driving.**
- **Wheel odometry and IMU:** the base measures how far each wheel turned (wheel odometry) and has a
  motion sensor (IMU - inertial measurement unit: gyroscope plus accelerometer). The robot's filtered
  heading drifts about 5 degrees per minute even while parked; stand still for at least 60 s at the
  start and end of every drive so the drift can be measured and subtracted.

## 2. The computers, and what runs where

| machine | what it is | what it runs | notes |
|---|---|---|---|
| **Jetson** | NVIDIA Jetson AGX Orin Developer Kit (a small computer with a graphics processor), mounted on the robot's deck | the ZED X camera, RTAB-Map mapping, recording, the live map web page, the storage and jobs web pages | runs with **no monitor**, but its internal display session must stay alive or the camera cannot open |
| **Robot computer** | a second Jetson Orin, inside the Husky base | the ROS master (the coordinator every ROS program registers with) for the wheels; wheel odometry, IMU, both LiDARs; a colleague's LiDAR mapping package | **not ours - ask before changing anything on it**; if it breaks, the robot cannot be driven |
| **Your own computer** | any computer on the lab network | nothing robotic; it is the viewing screen (the Jetson's web pages, a remote terminal) and the machine you download recordings from the Teams folder with | needs no ROS |

*Plain terms: the camera computer is kept separate from the robot's computer on purpose, so our
software can never stop the robot from being driven safely.*

Data flow during a drive: the camera sends images to the Jetson; the robot computer sends wheel and
IMU readings to the Jetson over the USB-C cable (10-20 Hz); an EKF (extended Kalman filter - a
standard method for combining noisy measurements) merges camera, wheels and IMU into one position;
RTAB-Map builds the map; the LiDARs are recorded on the robot and only used afterwards as a
reference.

## 3. The camera: Stereolabs ZED X

- Stereo camera (two lenses side by side, which gives depth the way two eyes do), lenses **0.120 m**
  apart, 1920 x 1200 pixels, built-in IMU at 400 Hz. Horizontal field of view about 73 degrees
  (datasheet maximum for the 4.6 mm lens).
- Connects by **GMSL2** (an automotive camera cable standard running to a capture card), **not USB**.
  The USB diagnostic tool therefore always reports "camera not detected" - ignore it.
- Needs two background services on the Jetson (`nvargus-daemon`, `zed_x_daemon`) and the Jetson's
  display session. Camera software: ZED SDK 4.2.5 with the ZED ROS wrapper (the bridge into ROS).

**The 15 Hz reality.** The sensor captures on a 30 Hz clock, but **everything downstream runs at
15 Hz - including the SVO recorder** (SVO: Stereolabs' own lossless recording format). The ROS
wrapper caps its picture loop at the publish rate, and recordings are taken inside that loop.
Measured: 284 of 313 gaps between recorded frames are exactly 1/15 s. Plan all timing on 15 Hz.

*Plain terms: the camera looks 30 times a second, but only every second picture is kept.*

Depth mode (how the camera computes distance): drives used NEURAL; QUALITY measured more accurate,
NEURAL more repeatable. Always say which mode a number came from.

## 4. The LiDARs - reference only, and not our code

- **Ouster OS1-32** and **RoboSense Helios 32**: spinning laser scanners (LiDAR - light detection and
  ranging) on the robot's mast. They agree with each other to about 5 mm.
- They belong to the robot and are driven by a **colleague's package, `self_navigation`**, on the
  robot computer. It records **raw LiDAR packets, not expanded point clouds** (clouds are about
  18 GB per 20-minute run; packets are a small fraction of that), and it has replay, body-filter
  and LiDAR-only RTAB-Map mapping launch files. **Use it, credit it, do not modify it.**
- The two LiDARs cannot run at the same time from separate launches: they share one fixed node
  name, so starting the second silently stops the first. The colleague's packet recorder captures
  both at once.
- A LiDAR path is **a second opinion, not ground truth** (the true answer). Write "agreement with an
  independent LiDAR estimate", never "error against ground truth". The LiDAR-to-camera mounting
  offset has never been measured.

## 5. The camera mount

- A 3D-printed bracket, designed in SolidWorks: two holder blocks (left and right) grip the camera,
  and front and back clamp pieces tighten onto the **two 8 mm support rods of the Ouster mast**
  (50 mm apart). Part files: `catkin_ws/src/sidewalk_bringup/meshes/zedx_holder/*.stl`.
- Design brief: [docs/mount/index.html](mount/index.html). Note: that brief was written before the
  PACS grid was confirmed on this robot, so its "may be pre-PACS" warning is out of date.
- **Measured camera position (29 Aug 2026)**, left lens relative to `base_link` (the robot's
  reference point: middle of the wheel axle, at ground level; x forward, y left, z up):
  x = 0.071 m, y = +0.020 m, z = 0.563 m (about 0.70 m above the floor),
  tilted **3.24 degrees nose-down** (0.0565 rad). Yaw (left-right aim) was not measured.
- The pose lives in `catkin_ws/src/sidewalk_bringup/config/robot_frames.yaml` and the robot model
  files `urdf/*.xacro`. **These are measurements, not tuning knobs** - a 1 degree tilt error moves a
  point 20 m away by about 35 cm. If the bracket moves, re-measure; see docs/OPERATIONS.md,
  "Editing the ZED X camera's position" (the pose is kept in four places that must agree).

## 6. Power

| what | powered by |
|---|---|
| Husky base, robot computer, LiDARs | the Husky's own battery |
| Jetson and ZED X camera | an **EcoFlow River Plus** portable power station on the deck, via the Jetson's USB-C power input |

Because the two are independent, **the robot can be switched off to charge while the Jetson keeps
working**. But a charging robot is offline: no wheels, no LiDAR, no robot computer. The battery
voltage reading is not a readiness check - ask whoever charged it.

## 7. Network links (no addresses here - ask Prof. Mehrtash or the lab for the logins)

| link | used for | notes |
|---|---|---|
| **USB-C cable, Jetson to robot computer** | wheel and IMU data into the mapping; fast file copies (about 26 MB/s) | a small private two-machine network; each end has a fixed address |
| **Lab WiFi** (several access points) | the robot's live LiDAR panel, general access | the robot's WiFi address comes from DHCP (automatic address hand-out) and **changes after charging**; a roaming helper moves the Jetson to the strongest access point, cutting outages from about 70 s to 16-22 s |
| **Lab network, your computer to Jetson** | remote terminal (ssh, a remote-login program), the Jetson's web pages, copying recordings onto the Jetson with `scp` or `rsync -s` | never copy big files during a drive |

## 8. Storage devices, and which to trust

| device | trust | use |
|---|---|---|
| Jetson internal disk (about 54 GB) | **trusted** | everything live: code, recordings in progress, results. Keep at least 13 GB free before a drive |
| Jetson's microSD card, label **SIDEWALK128** | **do not trust - never write to it** | failing since 8 Sept 2026 (checksum errors, writes at 0.1-0.4 MB/s, later silently discarding writes). Six incidents, including destroyed recordings |
| USB stick **CAM_REC** | **do not trust** | failed a write-then-read-back check (corrupted copies) |
| 128 GB microSD in the robot computer | trusted, surface-tested | our LiDAR recordings only (folder `slam_series2`); the robot's own small disk must never receive recordings |
| **Autonomous Service Robot** Microsoft Teams folder (access: ask Prof. Moein Mehrtash, McMaster University) | long-term home | every recording and result, laid out as in `data/DATA_INDEX.md`; delete a Jetson copy only after the Teams copy has been downloaded again and checked |

Always verify a copy by reading it back from the device, not from memory (a checksum of a file just
written can come from the computer's memory cache and prove nothing).

## 9. Software stack - fixed, do not upgrade

| component | version | consequence |
|---|---|---|
| Jetson | AGX Orin, L4T R35.4.1, JetPack 5.1.x, **12 cores**, 29 GB memory | CUDA 11.4, TensorRT 8.5.2 |
| Operating system, middleware | Ubuntu 20.04.6, **ROS 1 Noetic** (end of life), 64-bit ARM | **Python 3.8 is fixed** - anything needing 3.10 or newer is out |
| Simulator | **Gazebo Classic 11.15.1** | ROS 2 simulators are not options |
| SLAM | **RTAB-Map 0.21.13** | parameter names differ from newer versions: check with `rtabmap --params` |
| Vision | OpenCV **4.2.0**, no contrib/nonfree modules | SIFT and several feature detectors are unavailable; RTAB-Map falls back to others **silently** |
| Graph optimisers | g2o, GTSAM, Vertigo, TORO yes; **Ceres no** | anything needing Ceres is blocked |
| Visual-inertial front ends | none compiled in | only odometry strategies 0 and 1 exist |
| Trajectory tool | evo **1.31.1** (pinned) | 1.32+ needs Python 3.10 |
| Camera | ZED X, 0.120 m, 1920 x 1200, 30 Hz sensor / **15 Hz downstream**, GMSL2 | see section 3 |
| Robot | Clearpath Husky A200 | |

The Jetson was re-flashed (operating system reinstalled) once, and rebuilding cost days. Prefer
virtual environments or home-folder builds over system packages; record every install in
`docs/INSTALLED.md`.

**Never suggest:** ROS 2, gz-sim (new Gazebo), Isaac ROS / cuVSLAM (ROS 2 and a newer JetPack
only), Cartographer (no Noetic release), slam_toolbox (needs Ceres), VINS-Fusion, SVO Pro, DSO.

## 10. Known hardware traps (one line each)

Headings are quoted exactly so they can be searched for.

- Camera "not detected" but fine: GMSL2 is not USB, and opening needs a display - SOLVED.md,
  "\"Camera not detected\" on a working ZED X — two separate traps, neither about the cable".
- Camera will not open over a forwarded remote login - SOLVED.md, "The ZED X will not open over
  SSH because X forwarding hijacks `DISPLAY`".
- Camera freezes grow with Jetson uptime; **restart the Jetson before a drive** if it has been up
  more than a few hours - SOLVED.md, "Camera freezes (ZED driver picture-loop stalls) grow with
  Jetson uptime (2026-09-26)".
- 16-bit depth cuts camera freezes about 3 times - SOLVED.md, "16-bit depth cuts the camera freezes
  about 3x".
- Heavy 3D viewers on the Jetson froze the whole machine (11 GB) - SOLVED.md, "Jetson \"crash\" that
  was a memory livelock — RViz 11 GB (2026-08-21)".
- Memory limits are accepted but not enforced on this Jetson - SOLVED.md, "`MemoryLimit` on this
  Jetson is accepted but NOT enforced (2026-08-21)".
- Never write to the Jetson's microSD card - DO_NOT_REPEAT.md, "Writing anything to the Jetson's
  microSD card (SIDEWALK128) - it is failing (2026-09-24)".
- Big buffered writes to a slow card can reboot the Jetson - DO_NOT_REPEAT.md, "A big buffered write
  to the slow SD card can livelock and reboot the Jetson (2026-09-10)".
- The CAM_REC stick corrupts data - DO_NOT_REPEAT.md, "USB stick CAM_REC writes corrupt data".
- The robot's WiFi address changes after charging - DO_NOT_REPEAT.md, "Starting a drive with the
  robot's address written into start_drive.sh"; OPERATIONS.md, "Robot address changes after charging".
- Robot heading drifts about 5 degrees a minute even parked - SOLVED.md, "Why the robot's wheel+IMU
  path doubles corridors: its heading drifts ~5 degrees a minute, even parked".
- The robot's LiDAR map is wheel and IMU odometry corrected by LiDAR - SOLVED.md, "The robot's
  \"LiDAR map\" is wheel+IMU odometry corrected by LiDAR - say so".
- ROS 1 cannot read the Helios recordings without extra message definitions - SOLVED.md, "ROS 1
  could not read the RoboSense Helios bags at all — no `rslidar_msg` exists for it".
- A roaming helper that switches access points on a strong link causes long outages -
  DO_NOT_REPEAT.md, "WiFi roaming helper v2 reassociated on a STRONG link".
- A robot not charged enough ends a session early - DO_NOT_REPEAT.md, "Localisation demo session 1
  ... robot not charged enough".
