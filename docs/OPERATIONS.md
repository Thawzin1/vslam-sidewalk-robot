# OPERATIONS.md — hands-on procedures for this project

Operational companion to the repo's `ENGINEERING_NOTES.md` (the rulebook). The rulebook says
what is true and what is forbidden; this file says how to actually drive the
machines. Read the relevant section before any hands-on sim/VNC/mapping session.

---

## How we work together (teaching values)

Command execution is governed by `ENGINEERING_NOTES.md` section 0.1. These values govern
explanation and code style on top of it:

- **Understanding matters as much as results.** The user is a research student —
  the point of this project is to genuinely understand the commands, code,
  folders, and packages involved, not just to have things work. Explanations
  should build real comprehension, not leave things as black boxes.
- **Ask, don't assume.** If a request, a design decision, or which of several
  approaches to take is unclear, ask directly instead of guessing.
- **Keep code and packages clean.** No dead code, no unnecessary complexity,
  sensible organization — the user needs to be able to read and follow it, not
  just run it.
- **Plain language in explanations.** Avoid unexplained technical jargon in
  chat. If a technical term is unavoidable, say briefly what it means in plain
  words alongside it.

## Instruction format for hands-on / terminal work

This project involves live, multi-terminal work across two machines (your own
computer and the Jetson, reached with `ssh <jetson>` over the lab network), often with several SSH sessions
and a physical console open at once. When walking the user through running
something, every instruction step must be structured exactly like this, in order:

1. **Where** — which machine: "your computer", "Jetson" or "robot".
2. **Which terminal** — if more than one session could be open, name the exact
   one (e.g. "Terminal 1 — the one running `roslaunch`", "a new terminal", "the
   physical console"). Never assume a session or working directory carries over
   from a previous step without saying so.
3. **Code to run** — the exact command(s), in a code block. No placeholders
   unless unavoidable.
4. **What it does** — one or two plain sentences: why this command, what it
   changes.
5. **What to paste back** — say exactly what output is needed to confirm the
   step.
6. **What to check** — what success looks like, and what a likely failure looks
   like, so a wrong result is recognizable without guessing.

Don't bundle multiple unrelated steps into one block without this structure.
Prefer one clearly-labeled step over several run together — each step must be
independently checkable and teach the underlying mechanism.

## Editing the ZED X camera's position (robot + camera model + driver)

**Rewritten 2026-08-29.** The previous version of this section told you to move
the camera by editing `zedx_xyz` in the simulation's xacro file (an XML template
file that ROS expands into a robot model). That instruction is now wrong in two
separate ways, and following it today would leave the project worse than before:

- The snippet it quoted no longer matches the file. It printed
  `zedx_parent` as `oak-d-base-frame` and `zedx_xyz` as `0 0 0.02`. Neither is
  what the file says today.
- `zedx_xyz` is **explicitly reserved by the source for temporary nudges**, not
  for moving the camera. A permanent move made there desynchronises the
  simulation from the real robot's model **and** from the camera driver,
  silently — nothing in ROS compares the three.

**The snippet as the file actually reads now.** Verified 2026-08-29 against
`sidewalk_sim/urdf/husky_real_zedx.urdf.xacro` in this repository
(`catkin_ws/src/sidewalk_sim/urdf/`); the
**Jetson** runs the same tree at `~/catkin_ws/src/…`.
The comment is quoted in full because it is the part that carries the rule:

```xml
  <!-- zedx_base_frame is the ZED X's own dedicated mount point, built into
       husky_real.urdf's "ZED X Camera Integration" section - the same way
       the sensor arch already has its own oak-d-base-frame for a different,
       earlier camera. It isn't shared with anything else, so zedx_xyz here
       is normally "0 0 0": the position IS the frame, tuned once and baked
       in, rather than an offset re-applied on every launch. Change the
       frame's own position in husky_real.urdf if the mount ever needs to
       move for real; use zedx_xyz here only for a temporary, try-it-and-see
       nudge. D05 (a PACS cell on the main deck) also still works as
       zedx_parent, for an unrelated, lower mount. -->
  <xacro:arg name="zedx_parent" default="zedx_base_frame"/>
  <xacro:arg name="zedx_xyz" default="0 0 0"/>
  <xacro:arg name="zedx_rpy" default="0 0 0"/>
```

Read the three defaults as a unit: `zedx_parent` names the frame the camera
hangs off, and that frame — `zedx_base_frame` — **is** the mount pose now, so
`zedx_xyz` and `zedx_rpy` are deliberately zero. There is nothing to edit here
to move the camera. Editing them anyway does not fail — and that is the danger.
It silently stacks an extra offset on top of `zedx_base_frame`, in the
simulation only. The four files listed below, which are where the pose is
actually stated, know nothing about it, so the simulated camera quietly stops
agreeing with the real robot's model, with the camera driver, and with the
configuration file the perception code reads. Nothing in ROS compares them, so
nothing reports it.

### The camera pose now lives in FOUR places, and all four must move together

Since the geometry was measured on 2026-08-29 there is no single file that "is"
the camera pose. Four files each state it, for four different consumers, and a
change to one without the others produces plausible-but-wrong output rather than
an error message. Every path below is given as it sits in this repository
(`catkin_ws/src/…`); the **Jetson** runs the same tree at `~/catkin_ws/src/…`,
so an edit made in one copy must be copied to the other.

| # | File (under `catkin_ws/src/`) | What it drives | Which point it states |
|---|---|---|---|
| 1 | `sidewalk_perception/launch/zedx_front.launch` — `cam_pos_x/y/z`, `cam_roll/pitch/yaw` | The **ZED driver's own frame tree** (`zedx_front_base_link` → `camera_center` → left/right lens → optical frames → `imu_link`). This is what RTAB-Map actually uses. | The **mount point** |
| 2 | `sidewalk_bringup/urdf/husky_a200_real.urdf` — `zedx_base_joint` origin | The **real robot's model**, which is what RViz draws on the real robot (`real_robot_model.launch` loads this file). | The **mount point** |
| 3 | `sidewalk_sim/urdf/husky_real.urdf` — the same `zedx_base_joint`, kept byte-for-byte identical | The **simulated robot**, in Gazebo. | The **mount point** |
| 4 | `sidewalk_bringup/config/robot_frames.yaml` — the `camera_front:` block | The perception and navigation code that reads geometry from a configuration file rather than from the transform tree. | The **left lens** — a *different* point, deliberately |

The as-built numbers, all relative to `base_link` (the robot's coordinate origin,
at the midpoint of the drive-wheel axle, projected to the ground — it sits
0.13228 m above the floor, so it is **not** on the floor):

```
Files 1, 2, 3 — the MOUNT point (zedx_base_frame / the driver's zedx_front_base_link)
    x =  0.081    y = -0.040    z =  0.547
    roll = 0.0    pitch = 0.0565 rad (3.24 deg NOSE DOWN)    yaw = 0.0

File 4 — the LEFT LENS (camera_front in robot_frames.yaml)
    x =  0.071    y =  0.020    z =  0.563
    roll = 0.0    pitch = 0.0565    yaw = 0.0
```

**Why file 4 is different on purpose, and must stay different.** The ZED SDK
(the camera manufacturer's software library) reports every depth measurement from
the **left lens**, not from the housing centre and not from the mounting screw.
So the left lens is the point whose position on the robot has to be known. The
ZED X housing is about 163 mm wide and carries a 120 mm stereo baseline (the gap
between the two lenses), which puts the left lens roughly 60 mm to the robot's
left of the mount point. That is the whole content of the difference:

```
    mount            ( 0.081, -0.040, 0.547)
  + ZED internal     (-0.010, +0.060, +0.016)   read from the live transform tree
  = left lens        ( 0.071,  0.020, 0.563)    = 0.6953 m above the FLOOR
```

**Do not "tidy" the two blocks into agreement.** Copying the mount numbers into
`robot_frames.yaml` bakes in a 60 mm lateral error before anything is measured,
and it looks like a cleanup while doing it.

### The verification command — run this after ANY change to the four files

**Where:** the Jetson, in a terminal with the camera already running (the ZED
driver must be up, or there is no driver tree to compare against).

```bash
rosrun tf tf_echo zedx_base_frame zedx_front_base_link
```

**What it does.** `tf_echo` prints the transform (the position-and-rotation
relationship) between two named frames in the live tree. `zedx_base_frame` is the
mount point as the **robot model** (files 2 and 3) states it. `zedx_front_base_link`
is the same mount point as the **driver** (file 1) states it. They describe one
physical place, so if the files agree the two frames land on top of each other.

**What success looks like:** translation `[0.000, 0.000, 0.000]` and rotation
(RPY) `[0.000, 0.000, 0.000]`, to within a millimetre and a milliradian.

**What a non-zero reading means:** the robot model and the driver disagree about
where the camera is. The reading is the disagreement itself, which makes it
directly diagnosable:

- **Non-zero translation, zero rotation** — one of `cam_pos_x/y/z` (file 1) and
  the `zedx_base_joint` `xyz` (files 2 and 3) was changed without the other. The
  printed vector is exactly the amount by which they differ.
- **Non-zero rotation** — same, for `cam_pitch` against the joint's `rpy`. A
  reading of about 0.0565 rad in pitch means one side has the measured tilt and
  the other still has zero. A reading of about **0.113 rad (double)** means the
  tilt got applied twice, which is a real mistake made easy by the tree's shape.
- **`Frame zedx_front_base_link does not exist`** — the driver is not running,
  or `camera_name` is not `zedx_front`. This is not a geometry failure; start the
  camera and re-run.

**These two pitch values DO NOT SUM.** The ZED driver's chain hangs off
`base_link` directly and does **not** descend from `zedx_base_frame`, so the
0.0565 in file 1 and the 0.0565 in files 2 and 3 are two parallel descriptions of
one mount, not a rotation applied twice. This is exactly why the check above is
worth running: with two independent chains stating the same thing, the only way
to know they still agree is to ask.

### The two measurement tools — one gives the ANGLE, one gives the CAUSE

Both live in `sidewalk_perception/scripts/` and run **on the Jetson**, with the
camera live and the robot **parked and stationary**.

**1. The angle.**

```bash
rosrun sidewalk_perception measure_camera_tilt.py _frames:=5
```

It transforms the camera's depth point cloud (the three-dimensional set of points
the camera measures) into `base_footprint` — the frame whose `z = 0` plane *is*
the ground by construction — and fits a plane to the floor using RANSAC (a
fitting method that finds the plane the largest number of points agree on and
discards the rest, so chair legs and walls do not drag the answer the way a
plain average would). Whatever tilt comes out is the angle the model does **not**
know about, and it can be added to `cam_roll` / `cam_pitch` directly.

*Reference reading, 2026-08-29:* -3.24 deg (spread 0.28) before the correction
and -0.03 deg (spread 0.27) after, at roughly 16,500 inlier points per frame,
against the script's stated ±0.3 deg noise floor. The reconstructed floor went
from rising 226 mm over 4 m to rising 2 mm.

**2. The cause.**

```bash
rosrun sidewalk_perception check_gravity.py _samples:=600
```

The ZED X contains an accelerometer (a sensor that measures acceleration; at rest
it measures gravity). Gravity is an absolute vertical reference that owes nothing
to the transform tree, so rotating the measured gravity vector into `base_link`
*through the current model* and asking how far it lands from vertical is a test
the transform tree cannot fake.

*Reference reading, 2026-08-29, 600 samples with the robot parked on the lab
floor:* residual pitch 0.90 deg from vertical.

### The caveat that makes tool 2 necessary — read before believing tool 1

**The plane fit alone cannot say whether an angle comes from the camera, from
the robot body, or from the patch of floor being looked at.** `base_footprint` is
bolted to the **robot**, not to the world — it is `base_link` minus a fixed
0.13228 m and nothing more. So the fit reports the **sum** of three things it
cannot separate:

1. the camera tilted in its mount;
2. the robot body pitched relative to its own wheel-contact plane (soft tyre,
   forward payload, sagging suspension);
3. the floor patch inside the fit window sloping differently from the floor
   under the wheels — a ramp, a crown, a threshold, a settled slab.

A **uniform** floor slope is invisible to it, because that tilts the robot, the
camera and `base_footprint` together and cancels out.

**Re-running the plane fit after applying a correction is not independent
evidence.** The second fit constrains the same sum as the first, so getting ~0
back is arithmetic confirming itself. It proves the edit reached the live tree
with the right sign — a flipped sign would have read about -6.5 deg — and
nothing more. (An earlier version of this project's notes claimed the floor's
226 mm rise over 4 m "independently confirmed" the fitted normal. It does not:
rise ÷ 4 is the same fitted normal restated, with the plane offset cancelling.
Two statistics of one fit are not two witnesses.)

**Only the gravity check decomposes the sum**, because the three cases predict
different answers and the measured one picks between them:

| Gravity residual | What it means |
|---|---|
| ~0.0 deg | The camera really is tilted in its mount, robot level → the applied `cam_pitch` is correct |
| ~3.2 deg | Camera and body both level → the tilt came from the floor patch, and `cam_pitch` is an **error** |
| ~6.5 deg | Camera level in its mount, robot body pitched on its wheels → `cam_pitch` is wrong and the body attitude is the story |

The 0.90 deg reading is the first row, which is why 0.0565 rad stands as the
camera's real mounting pitch.

**What is still not known.** Roll is left at 0: the plane fit read +0.24 deg
(inside its own noise floor) and gravity read -1.37 deg, and the two disagree by
more than either is worth — that gap sits within the combined accelerometer bias,
the factory calibration between the camera and its internal sensor, and a
plausibly ~1 deg lab floor. Do not average them to make it go away; they answer
different questions. Yaw is **unmeasurable by both methods** — a rotation about
vertical leaves a horizontal plane horizontal and leaves gravity unchanged — so
it stays 0. Roll and yaw being written as 0 is *"nobody has measured this"*, not
*"this was measured and found square"*. Say so wherever it matters.

### To see a change in simulation

Save the files, then (Ctrl-C the running one first if needed) — **this is a
long-running, interactive command, so the user runs it, not the Jetson operator**:

```bash
roslaunch sidewalk_sim husky_real_gazebo.launch gui:=false rviz:=true zedx:=true
```
Check via the same VNC connection as always: `vncviewer <jetson-address>::5900`.

**If the sim still shows the old position after relaunching:** Ctrl-C only stops
the process in that one terminal — if another launch was ever started elsewhere
(another terminal, or a background command) and left running, the new launch can
end up sharing a simulation with old, stale data instead of a clean one. Check
for leftovers before assuming the file edit is wrong:
```bash
pgrep -af 'gzserver|gzclient|rviz|roslaunch|rosmaster|robot_state_publisher|rtabmap|rgbd_sync|rgbd_odometry|controller_spawner|cmd_vel_relay|teleop'
```
More than one `roslaunch`/`rosmaster`/`robot_state_publisher` line means there
are leftovers. Kill everything listed by its exact process ID
(`kill -9 <pid> <pid> ...`), confirm the same `pgrep` command comes back empty,
then relaunch once, fresh. `robot_state_publisher` and
`rtabmap`/`rgbd_sync`/`rgbd_odometry` are **not** scoped to one ROS master by
this grep — if the real camera (its own master on port 11312) is also running at
the same time as the sim (port 11311), cross-check `ps` output or `rosnode list`
on the specific `ROS_MASTER_URI` before killing anything, or you'll kill the
real camera's own node instead of the sim's (happened once, 2026-08-20).

## Gazebo simulation failures: crashes vs. silent hangs

Gazebo Classic under sustained load can fail two different ways — tell them apart
before picking a fix, since both look similar from RViz but have different
symptoms underneath:

**1. Crash (segfault).** The `roslaunch` log shows a line like
`process has died [pid ..., exit code 139]`. Everything stops at once. No known
code bug causes this — it's a known Gazebo Classic stability limit under heavy
load for long runs. Fix: clean kill of any survivors + relaunch fresh (see
leftover-process check above).

**2. Silent hang — no crash, no exit code, but a piece is still dead.** Confirmed
2026-08-20 after ~3 hours of runtime: the robot model stayed on screen but its
wheels disappeared from RViz. Gazebo's physics was still running fine (`/clock`
still ticking, the `/gazebo` node still responsive) — only the `ros_control`
`controller_manager` plugin had stopped responding, so `/joint_states` went
silent and the wheel joints had no transform to render. Nothing in any log said
"died," because nothing did — it just stopped answering. **Don't assume a
rendering glitch just because something is missing on screen — check the data
pipeline first:**
```bash
rostopic hz /joint_states                              # "no new messages" = dead
rosservice call /controller_manager/list_controllers   # "controller: []" = never loaded / died
rostopic hz /clock                                      # confirms Gazebo's physics itself is still alive
```
If `/clock` is ticking but `/joint_states` is silent, Gazebo's core is fine and
only the controller plugin died — restarting RViz alone won't fix it, the whole
sim needs a clean restart.

**Either way, plain `kill -2` (SIGINT) may not be enough to restart cleanly.** If
the thing that's actually stuck is `controller_spawner` (waiting forever on a
service call that will never answer), it can swallow `roslaunch`'s SIGINT and
never exit, holding `gzserver`'s and `rviz`'s parent process open even after
they've already died. Send SIGINT, wait a few seconds, then check for survivors
and force them:
```bash
kill -2 <roslaunch-pid>
sleep 6
pgrep -af 'gzserver|rviz|roslaunch|robot_state_publisher|controller_spawner'
# anything still listed here (commonly roslaunch itself + controller_spawner) → kill -9 it
```

## Visual SLAM needs TEXTURE, not colour — the sim world's most important property

A stereo camera works by finding small recognisable patterns ("features") and
matching them between frames. A flat, evenly-coloured surface has no pattern
anywhere on it, so it yields no features, so odometry cannot tell that the
camera moved at all.

This bit hard on 2026-08-20. `office.world` had been given a different material
per wall to stop RTAB-Map confusing one room with another — but those materials
were flat paint colours (`Gazebo/Red`, `Gazebo/Turquoise`, ...). Measured on the
running sim, the feature detector found **13 usable points in a whole 1920x1200
image**, against the 20 that `Vis/MinInliers` needs for a single pose. Odometry
reported `quality=0` on every frame and the map came out bent and wavy. After
re-texturing, the same measurement returned **1000** (the detector's cap) and
odometry quality went to ~880.

**Only a minority of Gazebo's built-in materials actually carry an image
texture.** Textured (safe for VSLAM): `Bricks`, `Wood`, `WoodFloor`,
`CeilingTiled`, `PaintedWall`, `WoodPallet`, `Road`, `Residential`,
`Pedestrian`, `Motorway`, `Trunk`, `Primary`, `Secondary`, `Steps`,
`BuildingFrame`, `Runway`, `Grass`, `PioneerBody`. Flat colour (useless for
VSLAM): `Red`, `Green`, `Blue`, `Yellow`, `Orange`, `Purple`, `Indigo`,
`Turquoise`, `Grey`, `DarkGrey`, `White`, `Black`, `Gold` and every `*Glow` /
`*Transparent` variant. Check before using one:
```bash
grep -A 12 "^material Gazebo/NAME" /usr/share/gazebo-11/media/materials/scripts/gazebo.material
```
A `texture_unit` naming a real image file means textured; only
`ambient`/`diffuse` lines means flat.

Two further rules the same episode established:
- **Distinct AND textured, both at once.** Distinct materials prevent false loop
  closures; textured materials make odometry possible. Solving one by
  sacrificing the other just swaps which failure you get.
- **Texture scale matters as much as texture presence.** Gazebo paints one copy
  of an image across each face of a box, so a 14 m wall shows a single stretched
  copy while a 0.5 m crate shows a full sharp one. Big surfaces should be split
  into tiles (the floor is a 4x4 grid for this reason) and small props are worth
  far more features than their size suggests.

**Quick way to check any world before spending a drive on it** — this is the
measurement that settles it, run it against the live sim:
```bash
python3 -c "
import rospy, numpy as np, cv2
from sensor_msgs.msg import Image
rospy.init_node('feat', anonymous=True)
m = rospy.wait_for_message('/zedx_front/zed_node/rgb/image_rect_color', Image, timeout=25)
a = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, -1)
g = cv2.cvtColor(a[:,:,:3], cv2.COLOR_BGR2GRAY)
print('ORB keypoints:', len(cv2.ORB_create(nfeatures=1000).detect(g, None)))
"
```
Under ~50 means the world is too bare; healthy is several hundred.
(Caveat: for anything beyond a one-off spot check, prefer a draining subscriber
that discards the first ~10 frames — `wait_for_message` has returned stale
cached frames on this system; see `docs/SOLVED.md`.)

**The opposite trap: a PERFECT simulated camera also breaks SLAM.** Gazebo adds
no sensor noise unless asked, so visual odometry reports standard deviations
around 0.0001 m — "I know where I am to a tenth of a millimetre". RTAB-Map
judges loop closures against that confidence, so a sane closure disagreeing by
11 cm scores as a huge outlier and is rejected. Run #4 rejected **all 224**
loop-closure candidates this way; the outdoor mcity run rejected all 301.
Symptom to watch for: `Rejecting all added loop closures ... error ratio of N`
where the quoted `abs error` is physically small (centimetres). The mechanism
(covariance clamp) and the rule (fix the sensor noise, never tune the back-end
around it — and `<noise>` on a depth sensor is silently ignored) are in
`ENGINEERING_NOTES.md` section 2.2 and section 2.3.

## Stale ROS parameters survive killing the node that set them

ROS 1 does not delete a node's private parameters when the node exits — they
stay in `rosmaster` until the master itself restarts. So editing a value out of
a `.yaml` and relaunching does **not** remove it from a long-running session:
the new node loads the file, and *also* still sees the old key that a previous
launch wrote.

Symptom seen on 2026-08-20: `Optimizer/Slam2D` and `Grid/FromDepth` were deleted
from `rtabmap_zedx.yaml`, the file verified clean on the Jetson — and RTAB-Map
still logged "Parameter name changed"/"doesn't exist anymore" warnings for both,
because `rosmaster` had been up for hours and still held them.

Check the parameter server, not just the file:
```bash
rosparam list | grep -i <ParamName>
rosparam delete /rtabmap/rtabmap/<Param>        # then relaunch the node
```
(Both `rgbd_odometry` and `rtabmap` get the YAML loaded into them, so a key
usually needs deleting under `/rtabmap/rgbd_odometry/...` as well.)

## Big images + deep queues = the whole machine dies, not just a node

On 2026-08-21 the Jetson dropped off the network entirely about a minute after
`record_mapping_run.launch` was started on top of a healthy sim. Not a crashed
process — the machine stopped answering ping (ICMP) and the remote-network
status showed it `offline ... rx 0`. Root cause (established after recovery): **RViz held
11.14 GB** (MapCloud accumulates the 3D cloud unbounded; its Camera display
re-subscribes to the full 1920x1200 stream a second time), rtabmap held 5.18 GB,
and the machine thrashed its 15 GB of zram swap so hard that the network service could
not send keepalives. `uptime` proved it **never rebooted** — a livelock, not a
crash, which is why `journalctl -b -1` had nothing.

**First, tell a dead PROCESS from a dead MACHINE.** These need completely
different responses and look identical from a hung SSH session:
```bash
ping -c 4 -W 2 <jetson-address>     # no reply = the machine, not the process
```
If ping answers but SSH does not, it is a service. If ping does not answer, no
amount of remote work will help. **And check `uptime` the moment it is back:**
if uptime exceeds the outage, it never went down and the cause is still running
and still holding memory.

**Standing mitigations (all in place, verify before heavy runs):**
- `queue_size` in the SLAM launch chain is **5** (was 30 ≈ 2.9 GB of buffered
  16 MB frames across six queues; now ≈ 0.48 GB). Passed through
  `rtabmap_mapping.launch` and `record_mapping_run.launch` — tune per run, never
  by editing the node file. Deep queues on real-time topics also make odometry
  process stale frames, which *causes* tracking loss — small is correct, not a
  compromise.
- **Recorded drives run `rviz:=false`.** Watch the monitor CSV / STATUS file and
  the Gazebo window instead.
- **earlyoom** is installed and configured (`/etc/default/earlyoom`): acts at
  15% free RAM+swap, never kills `sshd|Xorg|x11vnc` (and the network service), prefers
  `rviz|gzserver|gzclient|rtabmap|rgbd_*`. Verify with
  `systemctl is-active earlyoom` and `journalctl -u earlyoom -n 5`.
- Before adding anything to a running sim, measure **memory** headroom (`free -g`),
  not just CPU. The Jetson's RAM is unified with the GPU. Free cores are not
  free memory.

## The three-device working scheme (2026-08-24 — the standing pattern)

**Policy (ENGINEERING_NOTES.md section 0.3): the Jetson runs with no monitor; every window opens on
your own computer.** Files are browsed in VS Code over the remote connection; application windows are
forwarded; browser pages carry the lightweight live views. Ask Prof. Mehrtash or the lab for the
Jetson and robot logins.

**Daily pattern:**
1. **Files / editing** — VS Code Remote-SSH to the Jetson (set up a key once with `ssh-copy-id <jetson-user>@<jetson>`
   — one password entry, then never again). Do the same for the robot's computer.
2. **Application windows on the PC** — `ssh -X <jetson>` then run the program; the window appears
   on the PC. Verified working for graphics programs (direct rendering reported "Yes" over the
   forwarded display, 2026-08-24; RViz proven live on 2026-08-23). Expect RViz to be usable but not
   silky over the network — set its frame rate to 15. Gazebo's viewer: same mechanism, heavier — for
   the simulation prefer running the simulator headless on the Jetson and using RViz/browser views.
   ZED-X viewer applications (ZED_Explorer, ZED_Depth_Viewer): these open the camera through the
   Jetson's OWN display context, so **they cannot draw their window on the PC while doing so** —
   for camera viewing on the PC, use the browser stream or an RViz image panel instead.
3. **Browser views (lightest, always available)** — live map `http://<jetson-address>:8080/stream?topic=/map_image`,
   camera pages at `http://<jetson-address>:8080/`. No display setup at all.
4. **The Jetson's invisible session** — automatic login keeps a display session alive with no
   monitor; **do not kill it**: the ZED X cannot open without it (`docs/SOLVED.md`). VNC
   (`demo/vnc.sh`) remains a fallback mirror only; with no monitor attached the mirrored desktop
   renders as a wallpaper-only screen, which is why it is no longer the primary path.

## Remote GUI access to the Jetson (RViz, Gazebo, any window)

The Jetson has a monitor physically connected. Logging in locally (as
`sidewalk`, at the physical console) starts a real GNOME/X11 session — but **GDM
starts a fresh Xorg server for that login rather than reusing the greeter's**,
so the live session is very likely NOT display `:0`. `:0` stays the
(now-inactive) greeter. This was the root cause of a long debugging session on
2026-08-17 — don't re-assume `:0`.

**x11vnc mirrors that live session over the network**, so the exact same window is
visible and controllable (mouse/keyboard both ways) from your computer. It is installed on the Jetson but is **NOT
autostarted** (a persistent autostart entry was attempted and blocked by the
permission classifier as a standing/security-relevant change — do not retry
that; start it fresh each login instead, per below). You need a VNC viewer (for example
TigerVNC's `vncviewer`) on your computer.

**Shortcut, added 2026-08-23 — `demo/vnc.sh` does all three steps below.**
It sources `tools/vnc_display_env.sh` to re-derive the display, refuses to
start if nobody is logged in (with a message saying so), and is idempotent.
After a reboot: `ssh <jetson>` → `demo/vnc.sh` → `vncviewer <jetson-address>::5900`.
The manual sequence is kept below because you still need it when the shortcut
reports something unexpected.

**RUNNING THE JETSON WITH NO MONITOR ATTACHED (2026-08-23).** Three things
make this work, and all three are now in place:
1. `Option "AllowEmptyInitialConfiguration" "true"` in `/etc/X11/xorg.conf` —
   Xorg starts even with nothing plugged into DP-0.
2. **Automatic login is enabled** (`/etc/gdm3/custom.conf`,
   `AutomaticLoginEnable = true` / `AutomaticLogin = sidewalk`; the original
   is backed up alongside as `custom.conf.bak.pre-autologin`). Without it a
   monitorless reboot leaves nobody logged in, so there is no session, no
   display — **and the ZED X will not open**, because Argus needs a display
   context (see `docs/SOLVED.md`). That failure looks like a broken camera,
   not a missing login.
3. x11vnc still does **not** autostart, deliberately — run `demo/vnc.sh` once
   per boot. Do not re-attempt a persistent autostart entry; that was tried
   and blocked as a standing security change.

Unplugging the monitor from an **already-running** session is safe: the
session survives and VNC keeps mirroring it. It is only the reboot path that
needs the above.

**Every time the Jetson's physical session is freshly logged into** (after a
reboot, logout, or if x11vnc isn't already running), redo this sequence — from
any SSH session, non-interactively is fine:

1. Find the live display and its auth file — do NOT assume `:1`, re-derive it:
   ```bash
   ps aux | grep -i "[X]org"
   loginctl list-sessions --no-legend
   ```
   Look for the Xorg process on the `seat0`/active session (uid 1000 =
   `sidewalk`, not uid 124 = `gdm`). Its `-auth <path>` argument is the ground
   truth for `XAUTHORITY` (found this reliably where `-auth guess` and manual
   `~/.Xauthority` guesses both failed). Cross-check with `/tmp/.X11-unix/` —
   the *newest* `X<N>` socket corresponds to that Xorg process; `<N>` is the
   display number.

   **Shortcut:** `source tools/vnc_display_env.sh` does steps 1's derivation
   for you and exports `DISPLAY`/`XAUTHORITY` in the current shell — re-run
   it fresh after any reboot/logout, same as the manual version. It does not
   start x11vnc itself (still step 2, below, once per login).

2. Start x11vnc against that exact display/auth (only needs doing once per
   login session — leave it running in the background after):
   ```bash
   /usr/bin/x11vnc -display :<N> -auth <path-from-step-1> \
     -listen <jetson-address> -rfbport 5900 -shared -forever -noxdamage -bg \
     -o ~/.x11vnc.log
   ```
   `-listen <jetson-address>` scopes it to one network interface only, matching
   the trust model already used for SSH — never bind `0.0.0.0` here. Confirm
   success via `EXIT_CODE=0` and `PORT=5900` in its own output, or
   `tail ~/.x11vnc.log` for `screen setup finished`.

3. Connect from your computer:
   ```bash
   vncviewer <jetson-address>::5900
   ```

**Launching any GUI app so it appears in that shared session** (from an SSH
command, not typed live inside the VNC window) needs three things together —
missing any one causes a silent-looking failure:

```bash
source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && source ~/.sidewalk_env.sh && \
DISPLAY=:<N> XAUTHORITY=<path-from-step-1> <the actual command>
```

- The three `source` lines are required because **non-interactive SSH shells
  skip `.bashrc`** (Ubuntu's default has a `[ -z "$PS1" ] && return` guard near
  the top) — without them `rosrun`/`roscore`/etc. are "command not found" even
  though the exact same line works fine typed into a live interactive SSH
  session.
- `DISPLAY` and `XAUTHORITY` must both be the values re-derived in step 1 above,
  not `:0` and not a guessed `~/.Xauthority`.

**Trap: this applies even in a normal interactive ssh session, not
just one-off SSH commands.** If your computer's `~/.ssh/config` has `ForwardX11 yes`
for the Jetson (handy so VS Code's terminal forwards too), then *any*
plain `ssh <jetson>` login — typed live, no `-X` needed — silently already
has a working `DISPLAY` pointing back at your computer. Run `roslaunch ...
rviz:=true` without overriding `DISPLAY`/`XAUTHORITY` first, and RViz opens on
your computer instead of the shared VNC session — no error, it just looks like it
worked, on the wrong screen. Always `export DISPLAY=:<N>` and
`export XAUTHORITY=<path>` (step 1 above) at the start of any interactive
session before launching a GUI app this way.

Simplest alternative for one-off things: skip all of the above and just open a
terminal *from inside the VNC window itself* — it's a genuine child of the real
session and never has a display/auth problem, at the cost of typing there
instead of over SSH.

---

## Session-start checklist

Run this, in order, at the start of every lab session; do not skip steps that "look fine".

**1. Connections** (from your computer):
```bash
ssh -o BatchMode=yes -o ConnectTimeout=8 <jetson> 'echo JETSON OK; uptime -p'
ssh <jetson> 'ls /dev/video0 /dev/video1 && systemctl is-active nvargus-daemon zed_x_daemon && ls /tmp/.X11-unix/'
ping -c2 -W2 <robot-address>          # the robot, only if it is expected to be powered on
```
Camera check is *presence + services*, not opening the camera — opening it belongs to the run
procedure, which also carries the DISPLAY/ForwardX11 rules.

**2. Disk-space page** (skip the start if already running — check first):
```bash
ssh <jetson> 'ps -eo args --no-headers | grep -q "[s]torage_monitor.py" || setsid nohup python3 ~/catkin_ws/src/sidewalk_bringup/scripts/storage_monitor.py > /tmp/storage_monitor.log 2>&1 < /dev/null &'
```
Then open the page on port 8092 of the Jetson. Keep at least 13 GB free before a drive. Finished
recordings go to the Autonomous Service Robot Teams folder (ask Prof. Mehrtash for access): copy them
off, check the copy (download it again and compare size and checksum), then delete them from the Jetson.

**3. Hygiene** (all on the Jetson, all read-only checks first):
```bash
ps -eo pid,etime,comm --no-headers | grep -E "roscore|rosmaster|rosout|rtabmap|map_saver|gzserver|gzclient|data_player"   # expect none; kill by PID if found
rosparam get /use_sim_time 2>/dev/null    # must be false (or no master at all); a leftover true silently freezes a run
df -h /                                   # free space, into your notes
```
Never `pgrep -f`/`pkill -f` — they self-match their own command line (three documented hits).
Never write to the Jetson's microSD card SIDEWALK128: it is failing (docs/HARDWARE.md section 8).

## Robot address changes after charging (26 Sept 2026)
The lab WiFi gives the robot a new address after it has been off to charge: <an earlier robot-wifi-address> -> <robot-wifi-address> (24 Sept)
-> <robot-wifi-address> (26 Sept). `start_drive.sh` still falls back to <robot-wifi-address>. Before every drive or localisation
session, on the Jetson:
1. Check that `ssh robot hostname -I` answers. If not, get the address from the robot's screen or the router, and
   update `HostName` under `Host robot` in `~/.ssh/config`.
2. Start with the address given explicitly: `LIDAR_PEER=http://<addr>:8095 start_localise.sh <run>` (the bridge reads
   its host from the same variable).
A "sender is not answering on <addr>:8111" refusal right after charging almost always means step 1 was skipped.
(Recorded in DO_NOT_REPEAT.md, same date.)

---

# The `run/` launchers (handover edition, September 2026)

Every workflow has ONE entry point in `run/`. Each launcher has `--help`, sources `run/lib/paths.sh` (the
folders and addresses; set yours in `env.sh`, copied from `env.example.sh`), and writes or relies on a
`.progress` line in `$JOBS_DIR` so the jobs page (`tools/jobs_dashboard.py`, port 8096) shows it live.
Times below are what the September runs took. "records" = `$RECORDS_DIR/<run>/` (default `~/.run_records/<run>/`),
"work" = `$WORK_DIR` (default `~/slam_series2`).

## The progress-line convention (read this once)

A job writes ONE short line of plain text to `$JOBS_DIR/<job>.progress`, rewritten at least every ~30 s:
- if it contains `N/M` the page draws a bar; if it also contains a number followed by `s` (seconds elapsed)
  the page works out the rate and the time left; anything else is shown as-is.
  Example: `PACK s2_static_10 6/11 stream timing  893s`
- liveness is judged by the FILE's age, never by looking for a process: the page learns each job's own
  write interval and calls it stalled after several of them. A watcher (a script that alerts on failure
  and on silence, not only on success) is still needed for anything that runs unattended.
- start the page: `python3 tools/jobs_dashboard.py` (it finds the jobs by itself); the storage page is
  `catkin_ws/src/sidewalk_bringup/scripts/storage_monitor.py` (port 8092); the live map page is started
  by every drive (port 8095).

## run/mapping.sh - a mapping drive (drives 3-10 pattern)

Start: on the robot `run/robot_side.sh start <run>` (gyro re-zero, LiDAR packet recording, live LiDAR view,
wheel bridge sender); then on the Jetson `run/mapping.sh <run> [--svo]`. It starts, checking each: roscore;
a FRESH camera (NEURAL depth, tracker GEN_2); the bridge receiver (refuses if the robot does not answer);
the wheels + gyroscope + camera blend and the check that only the blend publishes odom -> base_link;
RTAB-Map (`record_mapping_run.launch`, `vo_publish_tf:=false`); the live map page; the fusion recording
(`records/fusion.bag`); with `--svo` the camera's own recording (`records/<run>.svo2`) and its guard; the
camera guard (restarts a dead camera, raises `records/ALERT`); the progress line; the live timelapse
recorder; the stop-on-"park" helper; and finally the READY gate (do not move the robot until it prints
READY: it waits for the gyroscope's false turn to be measured and kept, `<run>_ekf_inputs.progress`).
Watch: `<run>.progress`, `<run>_camera.progress`, `<run>_bridge.progress`, `<run>_svo.progress`, `<run>_media.progress`;
the live map page. End: park the robot on its start mark, keep still, `run/park.sh <run>` (45 s hold, then
the map closes properly; `--now` closes at once). Then `run/robot_side.sh stop <run>` on the robot, and
after an `--svo` drive `bash tools/drive/restore_camera_mode.sh`.
Produces: `work/<run>.db` (the map), `records/` with mapping.log, camera.log, fusion.bag, tegrastats.log,
autostop.log (the closing verdict), media/<run>_timelapse.mp4, tf_check_*.json. Refuses: an existing
database for that run name, `/use_sim_time` true, a camera busy recording for someone else.
Options: `--no-fusion` (camera only), `--tracker GEN_1`, `--robot-addr A,B`, `--lidar-peer URL`.

## run/localise.sh - find the robot on a saved map

Start: robot side as above; `run/localise.sh <run> [map.db]` (default map `work/localise/s2_static_10_map.db`;
`--bench` = camera only at a desk). The saved map is the MASTER: a working copy `records/work/start1.db` is
made, checksummed against it, and given an off-map start guess (`--kidnap-prior "40 40 90"`; `"0 0 0"` when
the robot really is on the start mark, as navigation needs; `none` keeps the map's saved position). RTAB-Map
runs `run/lib/localise/localise_run.launch` (localisation mode, `Mem/IncrementalMemory=false`), and the
watcher writes `<run>_localise.progress` ("LOCALISED yes/no", fixes, suspect jumps).
Next mark: park on it, hands off, `run/localise.sh next <run> <k> <mark>` (a fresh copy, the map reloaded,
`records/starts.csv` gets a line). End: `run/localise.sh stop <run>` - stops the localiser, the fusion pieces,
the guard, the camera (unless `--keep-camera`), the watcher; checks the master's checksum (pass line P4)
and deletes the working copies. Produces: `records/localise_summary.json`, `localise_events.csv`,
`localise_track.csv`, `starts.csv`, `P4_map_untouched.txt`, the fusion bag (with /rtabmap/info and
/rtabmap/localization_pose). Scoring afterwards: `run/lib/localise/score_localise.py`, `score_kidnap.py`
(pass lines in `run/lib/localise/PASS_LINES*.md`).

## run/slam_session.sh - continue a saved map

Start: robot side as above; `run/slam_session.sh <run> [map.db] [--mask] [--no-svo] [--loop-thr 0.08]`.
A COPY of the saved map becomes `work/<run>.db`; RTAB-Map opens it in mapping mode (`slam_run.launch`) and
today's drive is added; `--mask` blanks people out of the depth before the map (`depth_person_mask.py`,
needs the camera's people detector optimised once on this machine: `run/lib/slam_session/od_bench.sh`).
The camera's obstacle points are shown live on the map page (`live_obstacles.launch`, off with
`--no-live-obstacles`). Watch: `<run>_slam.progress` (JOINED yes/no, the working graph, link events).
End: `run/park.sh <run>`, then `run/slam_session.sh stop <run>` (`--now` if something is wrong): reads every
page of the database, `slam_closed_check.py` (new session written, closed properly, old map intact), the
master's checksum, and writes `records/stop_slam_report.txt`. Then `score_slam.py`, `session_split.py`
(pass lines in `run/lib/slam_session/PASS_LINES.md`). `replay_svo_slam.sh` replays a recorded session
through the camera driver offline.

## run/navigate.sh - drive a route of goals on the saved map

Order: robot `run/robot_side.sh start <run>`; robot: start `nav_cmd_recv.py` by hand (the command receiver;
it listens only on the USB-C link and only while the operator holds the autonomy button - see
`catkin_ws/src/sidewalk_navigation/doc/DEPLOY_NOTES.md`); Jetson `run/localise.sh <run> --kidnap-prior "0 0 0"`,
wait for READY; then `run/navigate.sh <run> A|B|B2`. It refuses unless the localisation is really up (blend,
camera depth, map -> base_link), checks the goals are not in a wall, starts `nav_run.launch` (map server for
`maps/d10_nav_clean.yaml`, a second one for the local wall map, camera obstacle points, `move_base` with DWA),
reads back every navigation setting move_base loaded (`records/nav_params_move_base.yaml`,
`nav_dynparam_readback.txt`), starts the command sender and waits for the robot to answer. The gate is SHUT
(`work/NAV_PAUSE`) until you press Enter with the button held; then `nav_goals.py` sends the route and
records into `records/nav_<route>/` (`goals_result.json`, planned routes, driven path); line `<run>_nav`.
Pause: `run/navigate.sh pause <run>` (the robot is told stop within 0.05 s). End: `run/navigate.sh stop <run>`
(stops only the process numbers in `records/nav_pids`), then `run/localise.sh stop <run>`. Score:
`catkin_ws/src/sidewalk_navigation/scripts/score_nav.py`. A simulator dry run exists:
`catkin_ws/src/sidewalk_navigation/sim/run_sim_dryrun.sh`.

## run/lidar_reference.sh - the independent LiDAR estimate for a drive

Needs the colleague's LiDAR stack built on the Jetson in `~/lidar_slam_ws` (`setup_lidar_slam.bash`;
`self_navigation`'s `3dreplay_pipeline.launch`, `rslidar_sdk`, the body filter - not part of this repository,
see CREDITS.md and `run/lib/lidar_reference/README.md`). `run/lidar_reference.sh <run>`: copies
`<run>_lidar.bag` off the robot's card to `$LIDAR_BAG_DIR` (sha256-checked on both ends; refuses while a
drive is live or space is short), replays it on an isolated ROS master (port 11350) through the colleague's
pipeline with `Reg/Force3DoF=true`, and writes `$LIDAR_REPLAY_DIR/<run>/lidar_ref_f3dof/` (rtab_helios.db,
lidar.tum, wheel.tum, lidar_map.png, facts, provenance.txt with every version and checksum). Line
`<run>_lidar_ref_f3dof_jetson.progress`; a replay takes about as long as the drive. `--validate DIR`
compares with a replay made on the robot. The result is a second opinion, not ground truth (rule: write
"agreement with the independent LiDAR estimate").

## run/evaluate.sh - the results pack

`run/evaluate.sh <run>` (camera half, 11 steps) then `run/evaluate.sh <run> --lidar` (6 steps) once the LiDAR
replay exists. Step list and every flag: `run/lib/pack_drive.sh --help`. Output: `results/<series>/<run>/`
(RESULTS.md itself is written by a per-drive script, `tools/results_pack/write_results_drive10.py` being the
drive-10 example: copy it, change the run name and the words). Restart after a failure with `--from N`.
Lines `pack_<run>.progress`, `pack_<run>_analyze.progress`. The pack never writes outside its output and
work folders and stops at a 3 GB disk floor. `--export-3d` makes the 3D point cloud (large; keep it out of git).

## run/simulate.sh - the no-hardware path

`run/simulate.sh <world> <route> [run] [--arm B1]` drives a simulated Husky with a simulated ZED X (two
plain cameras with image noise, so the odometry covariance is honest) through one of the Gazebo worlds in
`catkin_ws/src/sidewalk_sim/worlds` along a route file, records the map, samples frame yield, builds a
timelapse and writes `records/checklist.txt` (resets, quality lines, backend exceptions, `cannot be used`
detector downgrades). It refuses to start while Gazebo or a roslaunch is already running and stops only what
it started. Needs a display for Gazebo's server (see "Remote GUI access" above; `SIM_DISPLAY=:1`). Map
databases land in `work/sim_maps/<date>/`. Line `sim_<run>.progress`.

## run/robot_side.sh - the robot computer's half

Runs ON the robot (from this repository copied to its card). `start <run>`: gyro re-zero while still,
LiDAR packet recording (`record_lidar.sh`: raw packets from both LiDARs + /tf + /odometry/filtered + /imu/data
to the card, never expanded clouds - 18 GB a run as clouds), the live LiDAR view (port 8095 on the robot),
the bridge sender (port 8111; it publishes nothing on the robot). `stop <run>` after the map has closed:
closes the recording properly, the view, and LAST the sender. `storage -`: the read-only disk reporter
(port 8113) for the storage page. `selftest -`: the process matcher against a running and an absent name.

## run/replay.sh

A stub until the replay build lands; the pieces are `run/lib/slam_session/replay_svo_slam.sh` and
`catkin_ws/src/sidewalk_slam/launch/slam_from_recording.launch`.
