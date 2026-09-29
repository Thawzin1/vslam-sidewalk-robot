# Navigation demo - the Husky drives itself to goal points on drive 10's camera map

*Written 27 Sept 2026 (Hamilton) on the Jetson, for the navigation demonstration. Deadline Monday 28 Sept.
Nothing live was changed and nothing was installed: every file is in this folder (Jetson:
`<repo>/catkin_ws/src/sidewalk_navigation/`), and **nothing was copied to or run on
the robot** (it is a colleague's machine - ENGINEERING_NOTES.md rule 18; the robot-side receiver is written, not deployed).
ROS Noetic navigation 1.17.3, RTAB-Map 0.21.13, Clearpath husky packages 0.6.10 (all checked with `dpkg -l`).*

*Labels: **[VERIFIED - file:line or command]** read in a file or checked on this build; **[SIM]** measured in the Gazebo
dry run (section 9); **[INFERENCE]** reasoned from verified facts; **[UNVERIFIED]** not checked yet, with how to check.*

*Words used: **goal** = a point on the map (x, y, and the way to face) the robot is told to reach; **route** = the path
the planner draws from the robot to the goal; **costmap** = a grid over the floor where each 5 cm cell holds how bad it
is to be there (0 free ... 254 wall); **twist** = one wheel command: forward speed (m/s) + turn rate (rad/s);
**twist_mux** = the robot's own switch that picks which of several wheel-command inputs is obeyed; **deadman button** = a
button that must be HELD for the robot to move - let go and it stops.*

---

## 0. In five lines

1. **Where it runs:** everything that thinks runs on the **Jetson**, next to RTAB-Map's localisation: `move_base` (the ROS
   navigation program) plans a route on **drive 10's own saved 2D map** and turns it into wheel commands 10 times a second.
2. **Which map:** the route is planned on drive 10's saved occupancy grid (`Admin.opt_map`, exported read-only to
   `maps/d10_nav.*`); live obstacles come from the ZED X depth (**unmasked**, so people count), split into floor and
   not-floor by RTAB-Map's own normals test; RTAB-Map's localisation supplies where the robot is on that map.
3. **How commands reach the wheels without touching the colleague's stack:** a small sender on the Jetson passes each
   command over **the USB-C wire only** to a small receiver of ours on the robot, which publishes it on `/cmd_vel` - the
   input Clearpath's own twist_mux gives the **lowest priority (1)**; the joystick (10) and the e-stop lock (255) beat it.
4. **Limits:** 0.30 m/s and 0.40 rad/s, enforced three times (planner, Jetson sender, robot receiver); no reversing.
5. **Safety:** the robot moves only while the user **holds R2** on the joystick; letting go, any silence on the wire for
   0.5 s, a stale position or stale camera, straying more than 1.0 m from its route, the pause file, the e-stop lock or
   pressing L1 each stop it - 7 of 7 stop paths measured in the simulator, twice (section 6.3) - and the user walks
   beside it with the joystick throughout. Simulator dry run with the final settings: route A reached 2 of 2 goals in
   both runs; route B 2 of 4 (section 9).

---

## 1. The picture

```
 JETSON  (its own ROS master, localhost:11311)                                      ROBOT (Husky, ROS master of the wheels)
 ┌───────────────────────────────────────────────────────────────────────┐         ┌──────────────────────────────────────────┐
 │  ZED X ──depth (UNMASKED)──► point_cloud_xyz ──► obstacles_detection  │         │                                          │
 │    │                                   floor / not-floor points       │         │  joystick (PS4) ─► joy_node ─► /joy_teleop/joy (20 Hz)
 │    │                                          │                       │         │                    │        │                │
 │    │ (masked depth, people blanked)           ▼                       │         │                    │   teleop_twist_joy (L1)  │
 │    └──► RTAB-Map LOCALISATION ──map->odom──► move_base ◄── goals      │         │                    │        ▼                ▼
 │         on drive 10's map          ▲         │  global route on       │         │                    │  /joy_teleop/cmd_vel   R2 held?
 │                                    │         │  drive 10's map        │ USB-C   │                    │  (priority 10)          │
 │  wheels+gyro (bridge) ─► blend ────┘         │  DWA wheel commands    │ wire    │                    ▼        │                │
 │  (odom->base_link, /fused/odometry)          ▼  /nav/cmd_vel          │ :8112   │   ┌────────────── twist_mux ─────────────┐  │
 │  map_server: drive 10's 2D map ─► costmaps   │                        │         │   │ e_stop lock        255 (blocks all) │  │
 │                                       nav_cmd_sender.py ══════════════════════════► nav_cmd_recv.py ─► /cmd_vel  priority 1 │◄─┘
 │                                       clamp 0.30/0.40, "stop" if:    │ 20 Hz   │   │ (clamp again; moves only while R2   │
 │                                       planner idle / position stale  │ go|stop │   │  held, link fresh, no e-stop)       │
 │                                       camera stale / pause file      │         │   └──────────────┬──────────────────────┘
 └───────────────────────────────────────────────────────────────────────┘         │                  ▼ husky_velocity_controller
                                                                                    │                  (stops by itself 0.25 s after
                                                                                    │                   its last command) ─► wheels
                                                                                    └──────────────────────────────────────────┘
```

*Plain terms: the Jetson looks, decides and plans; the robot only obeys, and only while the user's thumb is on R2.
Anything the robot's own switch ranks higher - the joystick, the e-stop - overrides the Jetson at once.*

---

## 2. What runs where, and why

| piece | machine | started by | new or existing | why there |
|---|---|---|---|---|
| ZED X camera, wheel bridge, the blend (wheels + gyroscope + camera EKF), RTAB-Map localisation on drive 10's map | Jetson | `start_localise.sh` (localisation demo, unchanged) with `KIDNAP_PRIOR="0 0 0"` | existing, proven on s2_loc_04 (whole floor, 577 fixes over 246 m, none in a wrong place) | the map and the camera are on the Jetson |
| `map_server` (drive 10's saved 2D map on `/nav/map`), `point_cloud_xyz` + `obstacles_detection` (camera obstacles), `move_base` | Jetson | `start_nav.sh` -> `nav_run.launch` | new (standard packages, all installed) | needs the map, the camera and the localisation, all on the Jetson |
| `nav_cmd_sender.py` | Jetson | `start_nav.sh` | new | the only reader of `move_base`'s commands |
| `nav_goals.py` (goals + recording), `score_nav.py` (pass lines + figure) | Jetson | `go_nav.sh`, after the run | new | |
| `nav_cmd_recv.py` | **robot** | the user's approval, then DEPLOY_NOTES.md step R3 | new, **written, not deployed** | the only place `/cmd_vel` exists; one small program of ours, no change to anything of the colleague's |
| twist_mux, joystick teleop, wheel controller, hardware e-stop | robot | the robot's own boot (`/etc/ros/noetic/ros.d/base.launch` -> `husky_control control.launch` + `teleop.launch`) | existing, untouched | Clearpath's stack |

**Two ROS masters, joined by one TCP link, not one shared master.** The Jetson keeps its own master (as for every drive:
`/use_sim_time`, the bridge and the camera all assume it). Joining the robot's master would make the Jetson's nodes depend
on the lab network for every topic; one small, explicit link carrying one command 20 times a second is easier to make
fail safe. The wheel bridge (robot -> Jetson) already works this way; the command link is its mirror image
(Jetson -> robot).

---

## 3. The map and the costmaps

### 3.1 The saved map: drive 10's own 2D grid [VERIFIED]

- Source: `~/slam_series2/localise/s2_static_10_map.db` (the read-only master, sha256 `98f71ed1b18f5e73...`), column
  `Admin.opt_map` = the 2D occupancy grid RTAB-Map saved when drive 10 closed. Opened with sqlite `immutable=1` (cannot
  write, not even a journal). Decoded grid is byte-identical to the SLAM session's `d10_optmap.npz`.
- 802 x 675 cells of 5 cm (40.1 x 33.75 m); 92 736 free, 21 252 wall, the rest never seen. Origin (-10.525, -21.695) m.
  Same frame as RTAB-Map's localisation (the saved graph: start mark = (0, 0), facing east = +x).
- Written by `maps/export_nav_map.py` as `maps/d10_nav.pgm/.yaml` (map_server format, flipped so the picture's top row is
  the highest y) + `d10_nav.json` (provenance).
- Corridor widths on this map (distance between wall cells): north corridor 2.05-2.10 m, east 1.70 m, south 2.25-2.35 m,
  west 1.75-1.90 m. The robot with padding is 1.09 x 0.77 m and needs a 1.30 m circle to turn on the spot, so every
  corridor fits it with 0.4-0.8 m to spare; the planner keeps it to the middle (section 4).

**Why a fixed exported map and not RTAB-Map's live `/rtabmap/grid_map`:** in localisation mode the saved map does not
change, the exported one exists before the first localisation fix, and it can be checked (goal clearances,
`check_goals.py`) before the robot moves. RTAB-Map still supplies *where* the robot is on it (map -> odom).
`/rtabmap/grid_map` stays available as an alternative (`map_yaml` is an argument; switching needs `static_layer.map_topic`).

### 3.2 Layers (costmap_2d, Lu et al. 2014) [VERIFIED - config files; SIM read-back `params_move_base.yaml`]

| costmap | frame | layers | inflation | purpose |
|---|---|---|---|---|
| global (whole floor) | map | static (drive 10's walls) + obstacle (camera) + inflation | 0.65 m, cost scaling 3.0 | the ROUTE: short, but kept to the middle of the corridor |
| local (6 x 6 m, moves with the robot) | odom | obstacle (camera) + inflation | 0.65 m, cost scaling 5.0 | the WHEEL COMMANDS: nothing the camera has seen may touch the robot's outline in the next 2 s |

- **Inflation radius 0.65 m** = the robot's outer radius (half-diagonal of 0.99 x 0.67 m = 0.598 m) + 0.05 m padding: a
  cell with zero keep-away cost then means the robot fits there *whichever way it faces*.
- **The local costmap is in `odom`, not `map`,** on purpose: each localisation fix moves map -> odom in a step; camera
  obstacles the robot is steering around must not jump with it.
- **The saved walls are NOT in the local costmap** - tried and reverted (dry run 6): drive 10's walls are
  depth-smeared bands thicker than the real walls, and they narrow the long corridor to about 1.0 m at x = 12.4-13.2 m;
  with them in the collision check the robot refused every move through that spot (drive 10 itself drove through it
  many times). Instead, turns on the spot use at least 0.30 rad/s (0.20 stalled in the simulator, run 5) and the
  sender's **leash** stops the robot if it strays more than 1.0 m from its route - the answer to run 5, where the
  simulated robot crept into the corridor's end wall while a turn on the spot stalled. (Keeping camera marks under the
  robot's own outline was also tried, run 7: the planner then found the robot's own cell blocked and every goal
  aborted, so the package default - wipe them - stays.)
- **The camera never erases a saved wall** (`combination_method` maximum over the static layer). It only adds obstacles
  and clears its own earlier marks.

### 3.3 Live obstacles from the camera, and people

```
 ZED X depth 1920x1200 ─► every 8th pixel ─► points ─► 5 cm voxels ─► normals test (Grid/MaxGroundAngle 30 deg)
 (UNMASKED)               (point_cloud_xyz)   <= 4 m                    ├─ faces up, below 0.10 m over base_link ─► FLOOR points
                                                                        │     -> costmap: CLEAR the cells the ray crossed
                                                                        └─ anything else up to 1.60 m ─────────► OBSTACLE points
                                                                              -> costmap: MARK the cell, clear the ray before it
```

- Same test and the same `Grid/` values as `rtabmap_zedx.yaml`, the file drive 10 mapped with, so "obstacle" live means
  what "wall" means on the saved map [VERIFIED - config/obstacles.yaml; `rtabmap --params` lists every name].
- **People:** the SLAM session masks people out of the depth RTAB-Map sees (`depth_person_mask.py`), so they do not end
  up in the map or confuse recognition. **The costmap uses the raw depth**, so a person in front of the robot is an
  obstacle and the route bends round them or the robot waits (ENGINEERING_NOTES.md section 5: "mask the SLAM input only").
  A person who walks away is erased from the costmap once the camera sees the floor behind where they stood
  (ray tracing: the "floor" source clears).
- **Blind zone:** the ZED gives nothing nearer than 1.0 m (`min_depth: 1.0`, `zedx_front_od.yaml`) - about 0.5 m in
  front of the bumper - and nothing behind or beside the robot. Hence: no reversing, 0.30 m/s, and the person beside it.

---

## 4. Route and wheel commands

| part | chosen | reference | setting that matters |
|---|---|---|---|
| state machine, recoveries | `move_base` | Marder-Eppstein et al. 2010, "The Office Marathon", ICRA | recoveries only clear camera obstacles; never spins by itself; aborts after 3 failed plans |
| route | `global_planner/GlobalPlanner`, Dijkstra, gradient path | Dijkstra 1959; NavFn = Konolige 2000 gradient method (A* = Hart, Nilsson, Raphael 1968, one switch away) | `allow_unknown: false` (never through grey), `cost_factor 3.0` (prefer the middle), re-plan every 1 s |
| wheel commands | `dwa_local_planner/DWAPlannerROS` | Fox, Burgard, Thrun 1997, "The dynamic window approach to collision avoidance" | 0.30 m/s, 0.40 rad/s, turn on the spot >= 0.30 rad/s (0.20 stalled in the simulator, run 5), 2 s look-ahead, no reverse |
| alternative | TEB (`teb_local_planner`) | Rösmann et al. 2012 / 2017 | **not installed**; not needed for corridors (section 8) |

Every setting name was read back from the running move_base in the simulator (`sim_results/<run>/params_move_base.yaml`,
`dynparam_readback.txt`): a misspelt name is silently ignored, so the read-back is the proof (ENGINEERING_NOTES.md section 7 rule 4).

**Goals and which way the robot faces (ENGINEERING_NOTES.md section 2.13a).** Localisation recognises places by what the camera
sees, so the robot should face the way drive 10 faced when it passed. Drive 10 drove **every corridor in both
directions** (`01_runs/series2_static/s2_static_10/camera_corrected.tum`: long corridor east at 157-216 s and west at
233-289 s, and again later; the other three likewise), so turning round at a goal does not leave the camera facing
pictures the map never saw. Goals (`goals_d10_route_A.yaml`, `goals_d10_route_B.yaml`; clearances checked by
`check_goals.py`, turning needs 0.65 m):

| route | goal | x, y (m) | face | clearance | leg |
|---|---|---|---|---|---|
| **A** out and back (~28 m) | G1 east junction | 14.30, 0.00 | west (turn round) | 1.11 m | 14.3 m east along the long corridor |
|  | G2 start mark | 0.25, 0.00 | east (as parked for every drive) | 0.90 m | 14 m back west |
| **B** the loop (~58 m, only after A passes) | B1 east junction | 14.30, 0.00 | south | 1.11 m | |
|  | B2 south-east corner | 14.50, -14.70 | west | 1.20 m | |
|  | B3 south-west corner | 0.15, -14.10 | north | 1.00 m | |
|  | B4 start mark | 0.25, 0.00 | east | 0.90 m | |

---

## 5. Getting the commands to the wheels - without touching the colleague's stack

### 5.1 What the robot already has [VERIFIED - mirror + installed package]

- `/etc/ros/noetic/ros.d/base.launch` (mirror `~/robot_mirror/cpr-a200-0985/etc_ros/noetic/ros.d/base.launch`) starts
  `husky_node` (10 Hz control, max 1.0 m/s), `husky_control/launch/control.launch` and `teleop.launch`.
- `control.launch` runs `twist_mux` with `husky_control/config/twist_mux.yaml` and its output remapped to
  `husky_velocity_controller/cmd_vel`. That file (read from the Jetson's copy of the same Clearpath package, 0.6.10):

  | input | topic | timeout | priority |
  |---|---|---|---|
  | lock `e_stop` | `/e_stop` (std_msgs/Bool) | none | 255 - while true, every input below is ignored |
  | joy | `/joy_teleop/cmd_vel` | 0.5 s | 10 |
  | kb | `/kb_teleop/cmd_vel` | 0.5 s | 9 |
  | interactive_marker | `/twist_marker_server/cmd_vel` | 0.5 s | 8 |
  | **external** | **`/cmd_vel`** | 0.5 s | **1 (lowest)** |

- The wheel controller stops the wheels 0.25 s after its last command (`control.yaml cmd_vel_timeout: 0.25`), max
  1.0 m/s, 2.0 rad/s.
- The joystick: `HUSKY_LOGITECH` is not set in `mcm07_husky/scripts/environment`, so the PS4 settings load
  (`teleop_ps4.yaml`): **L1 (button 4) = drive enable, R1 (5) = fast; `joy_node` repeats every 20 Hz** (`autorepeat_rate: 20`).
  `teleop_twist_joy` publishes only while L1 or R1 is held.
- **[UNVERIFIED on the robot itself]** that the robot's installed husky_control is the same 0.6.x with the same
  twist_mux.yaml and teleop_ps4.yaml, and that the controller is the PS4 one. Read-only checks in DEPLOY_NOTES.md step
  R1 (`rosparam get /twist_mux`, `rostopic hz /joy_teleop/joy`, and pressing R2 while watching `/joy_teleop/joy`).
- The colleague's `self_navigation` also has a `move_base` publishing `/cmd_vel` (`matt_self_navigation/launch/move_base.launch`).
  It must not be running: R1 checks `rostopic info /cmd_vel` shows **no publisher** before ours starts.

### 5.2 Our receiver on the robot - what it may and may not do

`nav_cmd_recv.py` publishes on `/cmd_vel` only, i.e. twist_mux's lowest-priority input. It changes no file, no setting
and no program of the robot's. It listens **only on the USB-C wire address <robot-usb-address>, port 8112** (it refuses to
listen on all addresses), accepts only a sender that names the same run, and publishes a moving command only while
**all** of these hold, checked 20 times a second:

| gate | holds when | plain meaning |
|---|---|---|
| LINK | a command arrived < 0.5 s ago (robot's own clock, arrival time) | the Jetson is still talking |
| GATE | the Jetson marked it "go" | the Jetson's own checks passed (5.3) |
| HOLD | R2 (button 7) held and the joystick's messages < 0.3 s old | the user's thumb is on the button |
| NO E-STOP | nothing has published true on `/e_stop` | nobody pulled the software stop |
| SANE | both numbers finite | no garbage |

and it clamps again to 0.30 m/s / 0.40 rad/s with no reverse. When any gate fails it publishes zeros for 0.5 s, then
**nothing** - silence hands the robot back to twist_mux, and the wheel controller's own 0.25 s timeout stops it even if
our program dies.

### 5.3 The Jetson sender

`nav_cmd_sender.py` reads `move_base`'s commands (`/nav/cmd_vel` - move_base is remapped so it can never reach a real
`/cmd_vel`), and sends "stop" instead when: move_base has said nothing for 0.3 s (no goal / stuck), the blend's
position is older than 0.5 s, the camera obstacle points are older than 1.0 s (driving blind), there has been no
map -> base_link position for 1 s, the pause file `~/slam_series2/NAV_PAUSE` exists, or - **the leash** - the robot is
more than 1.0 m from the route planned when the current goal was sent (added after dry run 5, where the robot crept
2 m off its route; a big localisation correction trips it too, which is exactly when a person should look). Its jobs-page line `<run>_navcmd` shows the link, its gate and **what the robot says
it is doing and why** (the robot sends its status back twice a second), e.g.
`NAVCMD link up <robot-usb-address> gate OPEN robot MOVING (moving), button held go 288 stop 282 28s`.

### 5.4 The wire [VERIFIED - `~/.ssh/config` on the Jetson; memory robot-wired-links]

Jetson <jetson-usb-address> <-> robot <robot-usb-address> (`ssh robot-usb`). Commands use **only** this wire: no WiFi roaming gaps,
sub-millisecond latency. **Trap (measured 25 Aug):** if another computer is also cabled to the robot by ethernet, the robot brings up
<jetson-usb-address> itself and stops reaching the Jetson. That cable must be out during the run (PROTOCOL.md). The wheel
bridge can use the wire too (`ROBOT_ADDRS=<robot-usb-address>,<WiFi address>`).

---

## 6. Safety

### 6.1 The layers, from the user's hand outwards

```
   user's thumb on R2 ──(release)────────────────────────────────► receiver: zeros, then silence ─► wheels stop
   user presses L1 (stick centred) ─► joystick input, priority 10 ─► twist_mux ignores /cmd_vel ─► wheels stop
   red hardware e-stop on the Husky ─────────────────────────────► motor power cut, independent of all software
   /e_stop true (software lock) ─────────────────────────────────► twist_mux blocks everything below 255
   wire pulled / Jetson frozen ─► no command 0.5 s ──────────────► receiver: zeros, then silence
   receiver itself dies ─► no /cmd_vel at all ───────────────────► wheel controller timeout 0.25 s
   Jetson: position stale / camera stale / planner idle / pause ─► sender sends "stop" ─► receiver: zeros
   Jetson: > 1.0 m off the planned route (the leash) ────────────► sender sends "stop" ─► receiver: zeros
   planner: obstacle within the outline in the next 2 s ────────► DWA picks no moving command ─► stop
```

### 6.2 Stop-time budget (worst case at the 0.30 m/s cap) [INFERENCE from the timeouts above]

| cause | detected after | then | robot rolls at most |
|---|---|---|---|
| R2 released / e-stop / pause file | <= 0.05 s (next joystick message or tick) | + controller braking (3.0 m/s², 0.1 s) | ~0.03 m |
| wire pulled, Jetson frozen | 0.5 s | + braking | ~0.17 m |
| receiver killed | 0.25 s (controller timeout) | + braking | ~0.09 m |
| L1 pressed | immediately (priority) | + braking | ~0.02 m |

### 6.3 Measured in the simulator [SIM]

Every stop path stopped the simulated Husky from 0.30 m/s and held it stopped for 3 s, then let it drive on when
released - twice, in dry runs 6 and 9 (7 of 7 each). Figure: `sim_results/run9/safety/safety_stops.png`.

| test (run 9) | how it was triggered | stopped after | rolled |
|---|---|---|---|
| T1 R2 released | joystick stand-in reports the button up | 0.51 s | 0.09 m |
| T2 joystick silent | joystick stand-in stops publishing | 1.06 s | 0.17 m |
| T3 link silent (wire pulled / Jetson frozen) | sender process frozen (SIGSTOP): socket open, no bytes | 0.78 s | 0.19 m |
| T4 L1 pressed, stick centred | zero commands on `/joy_teleop/cmd_vel` (twist_mux priority 10) | 0.22 s | 0.03 m |
| T5 e-stop lock | `true` on `/e_stop` | 0.61 s | 0.13 m |
| T6 pause file | `touch NAV_PAUSE` on the Jetson | 0.21 s | 0.05 m |
| T7 sender killed | SIGKILL: connection closed | 0.35 s | 0.07 m |

*Plain terms: whichever way the robot is told to stop, it stops within about one second and less than 20 cm, and
stays stopped.* The times are wall-clock times of a simulator running slower than real time under load, so they are
upper bounds on the software path; the real wheels' braking has to be seen on the robot (DEPLOY_NOTES.md R4 checks the
chain with the motors off; PROTOCOL.md step 5 is the first moving check, with the user holding R2).

The robot receiver's refusal paths were tested separately on a private ROS master (`sim/recv_selftest.sh`,
`sim_results/recv_selftest/recv_selftest.txt`): it refuses to listen on all addresses, refuses limits above its
ceilings, refuses a sender with another run name, publishes nothing without the joystick button, clamps 0.9 m/s /
1.5 rad/s to 0.30 / 0.40 and a reverse command to 0, and stops when the sender goes quiet - 6 of 6.

### 6.4 What the user does (PROTOCOL.md)

Walks beside and slightly behind the robot, **joystick in hand, R2 held**; lets go of R2 to stop for any reason; presses
L1 to take over and drive by hand. The hardware e-stop is the backstop. Nobody else walks in front of the robot during
route A (people are a later test: the costmap handles them, but not before the plain run passes).

---

## 7. Published methods -> what is used

| method | where it is used |
|---|---|
| ROS navigation stack, `move_base` + `costmap_2d` (Marder-Eppstein, Berger, Foote, Gerkey, Konolige 2010, "The Office Marathon: Robust navigation in an indoor office environment", ICRA) | the whole navigation layer |
| Layered costmaps (Lu, Hershberger, Smart 2014, IROS) | static + obstacle + inflation layers |
| Dijkstra 1959 / NavFn gradient path (Konolige 2000, IROS); A* (Hart, Nilsson, Raphael 1968) | route |
| Dynamic window approach (Fox, Burgard, Thrun 1997, IEEE Robotics & Automation Magazine) | wheel commands |
| RTAB-Map localisation feeding navigation (Labbé & Michaud 2019, J. Field Robotics 36(2)) | map -> odom on drive 10's map; the saved occupancy grid as the navigation map; `obstacles_detection` for live obstacles |
| TEB (Rösmann, Feiten, Wösch, Hoffmann, Bertram 2012; Rösmann et al. 2017) | considered, not used (not installed; corridors need no overtaking manoeuvres) |
| Hold-to-run ("enabling device") control, as for teaching industrial robots | the R2 autonomy button |

---

## 8. Installed / missing (checked with `dpkg -l`, 27 Sept)

Installed on the Jetson (ROS Noetic): move-base 1.17.3, navfn, global-planner, dwa-local-planner, base-local-planner,
costmap-2d, clear-costmap-recovery, rotate-recovery, map-server, amcl, twist-mux 3.1.3, robot-localization 2.7.7,
rtabmap-ros 0.21.13 (+ rtabmap-util nodelets `point_cloud_xyz`, `obstacles_detection`; rtabmap-costmap-plugins),
husky-gazebo / husky-control / husky-navigation 0.6.10, gazebo-ros 2.9.3.

**Missing: only `ros-noetic-teb-local-planner`** (candidate 0.9.1 available). **Not needed** for this design (DWA is the
default and was used in the dry run). If wanted later, the one line on the Jetson after `sudo -v`:
`sudo apt install ros-noetic-teb-local-planner` (then record it in docs/INSTALLED.md).

Nothing is installed on the robot: the receiver needs only `rospy`, `geometry_msgs`, `sensor_msgs`, `std_msgs`, which
every ROS Noetic robot has (the wheel bridge sender already runs there with the same imports).

---

## 9. Simulator dry run (Gazebo, Jetson) [SIM]

### 9.1 Set-up

- **World:** drive 10's saved 2D map pulled up into 1.2 m walls (`sim/grid_to_world.py` -> `sim/d10_floor.world`,
  2 055 boxes), so the simulated corridors have exactly the saved map's widths - including its narrow places.
- **Robot:** Clearpath's own Husky model with its stock control stack - the same twist_mux priorities and wheel
  controller as the real robot - and a depth camera (RealSense model, 640 x 480, 87 deg) at the ZED X's measured mounting
  place (0.071, 0.02, 0.563 m, 3.24 deg down). One simulator-only calibration: the wheel controller's turn factor
  1.875 -> 1.25, because with Clearpath's value the simulated robot turned 1.5 times faster than commanded
  (`sim/turn_probe.py`: 0.2 / 0.3 / 0.4 / 0.6 rad/s commanded -> 0.31 / 0.38 / 0.60 / 0.90 true; after: 0.20 / 0.29 /
  0.40 / 0.58). The real robot's controller is not touched.
- **Localisation stand-in:** `sim/sim_localiser.py` corrects map -> odom from the simulator's truth every 2 s with
  5 cm / 1 deg noise, post-dated 0.1 s like RTAB-Map's (`tf_tolerance` 0.100, read from every drive's mapping.log).
- **Everything else is this folder's real code and settings:** map_server with `maps/d10_nav.yaml`, the camera
  obstacle chain, move_base with `config/*.yaml`, `nav_cmd_sender.py` -> TCP -> `nav_cmd_recv.py` -> `/cmd_vel` ->
  twist_mux -> wheels, a joystick stand-in holding R2 (`sim/sim_joy.py`), `nav_goals.py` and `score_nav.py`.
- Own ROS master (port 11411) and Gazebo master (11445); the live master on 11311 was never touched. Every process was
  started in its own session and stopped by its recorded process group; the ports were checked closed after each run.

### 9.2 What each run taught (in order; N = 1 per run)

| run | change | route A result | what it showed -> what was changed |
|---|---|---|---|
| 1 | first try | map server died | roslaunch splits `args=` on the spaces in the folder path -> quoted |
| 2 | wheel odometry only (map -> odom fixed) | both goals ABORTED during the 180 deg turns | the estimate followed the plan to 3 cm but the TRUE path was 0.9 m off: wheels slip in turns on the spot, so the real run needs RTAB-Map's corrections; a turn on the spot is "no progress" to move_base -> `oscillation_timeout` 15 -> 30 s |
| 3 | localisation stand-in | aborted in 4 s ("extrapolation into the future") | map -> odom must be post-dated, as RTAB-Map does |
| 4 | stand-in post-dated | **2 / 2 reached** (79 s, 103 s; 0.05 / 0.04 m from the goals, truth) | route B's first corner: 100 s of hunting - the simulated robot turned 1.5x faster than commanded -> simulator-only turn calibration |
| 5 | turn calibration | G1 aborted | at 0.20 rad/s the turn on the spot stalled (heading unchanged for 4 s); the wheel-command chooser then crept the robot 2 m off its route into the corridor's end wall -> **minimum turn 0.30 rad/s** and the **1.0 m leash** |
| 6 | + saved walls in the local costmap | G1 reached; G2 stuck for good | the saved map narrows the long corridor to ~1.0 m at x = 12.4-13.2 m (depth-smeared walls); with them in the collision check no move was allowed -> saved walls out of the local costmap, no speed-growth of the outline (`max_scaling_factor` 0). Safety tests 7 / 7 |
| 7 | + camera marks kept under the robot | every goal "NO PATH" | the robot's own cell stayed blocked -> package default restored |
| 8 | defaults restored | G1 reached; G2 overshot the start mark by 1 m | one costmap update took 4.5 s (Jetson load 8-9, simulator at nice 19) -> simulator point-cloud load cut to the real camera's (decimation 4); for the real run: no helpers or uploads during the drive |
| **9** | **final settings** | **2 / 2 reached** (85 s, 102 s; 0.23 / 0.10 m from the goals, truth) | safety tests **7 / 7** |
| **10** | **final settings, repeat** | **2 / 2 reached** (64 s, 64 s; 0.18 / 0.31 m from the goals, truth) | route B: see 9.4 |

### 9.3 Route A with the final settings (runs 9 and 10)

| | run 9 | run 10 |
|---|---|---|
| goals reached (N1: SUCCEEDED, estimate <= 0.50 m) | 2 / 2 | 2 / 2 |
| time per leg (14 m + a 180 deg turn) | 85 s, 102 s | 64 s, 64 s |
| robot's estimate vs first plan: median / 95th percentile (N3) | 0.083 / 0.235 m | 0.074 / 0.229 m |
| TRUE path vs first plan: median / 95th percentile | 0.087 / 0.215 m | 0.085 / 0.285 m |
| unplanned stops by the safety layer (N2, software part) | 0 | 0 |

Figures: `sim_results/run9/A_score/nav_paths.png`, `sim_results/run10/A_score/nav_paths.png` - planned route dotted,
the robot's estimate dashed, the simulator's truth solid; below, the distance off the planned route along each leg.
*Plain terms: in the simulator the robot followed its planned route to within about 8 cm typically and about 25 cm at
worst, and stopped within 10-30 cm of each goal.* Two runs of the same settings are not five (ENGINEERING_NOTES.md section 4
rule 2): this shows the chain works, not how reliably.

### 9.4 Route B (the loop) with the final settings (run 10)

**2 of 4 goals reached.** B1 east junction (90 s, truth 0.21 m from the goal, including the turn to face south) and B2
south-east corner (54 s, truth 0.32 m) were reached. At B3, the south-west corner, the robot arrived within 0.22 m of
the goal (t = 362 s) and began its 90 deg turn on the spot to face north; during that slow turn the simulated Husky
**slid about 1 m west**, past the corner into the corridor's western continuation, until its outline sat in the
keep-away zone of the wall, where no route can start ("Failed to get a plan"; B3 aborted after 117 s, B4 at once).
The robot's estimate agreed with the truth to 0.05-0.14 m throughout that leg, so this was not a localisation error:
it is the simulated skid-steer's centre drifting while it turns on the spot (turn_probe.py: 0.04 m/s of drift during
pure turns). Whether the real Husky drifts like that is not known - the drives' turn tables measure heading, not the
centre. **Consequence:** route A (two turns on the spot, both in wide junctions with 0.9-1.1 m clearance) is the demo;
route B is attempted only after route A passes on the real robot, with the user ready to let go of R2 at each corner.
Figure: `sim_results/run10/B_score/nav_paths.png`.

### 9.5 What the dry run does NOT show

- RTAB-Map's real localisation (its fixes come every 0.4 m or so on drive 10's map, in steps; the stand-in corrects
  every 2 s with 5 cm noise), the ZED X's real depth (1 m blind zone, noise on plain walls), the real Husky's turning on
  the real floor, the real WiFi/USB link, or a person walking in front.
- Largest single correction of the position in the simulator: 1.0 m (run 9), from the simulated wheels slipping in turns
  on the spot between two 2 s corrections - a simulator artefact, but the kind of jump N4 and the leash watch for.

---

## 10. Limits and risks (stated before the run)

1. **The robot's size is the manufacturer's (0.99 x 0.67 m), not measured;** the camera mount and sensor arch are
   covered only by 5 cm of padding. Measure before driving close to walls.
2. **Skid-steer turning:** a Husky scrubs its tyres to turn on the spot; wheel odometry is worst exactly then. The blend
   uses the gyroscope for heading, and RTAB-Map corrects position. In the simulator, without a correction, a turn on the
   spot moved the true position 1.2 m while the wheels said it had not moved (section 9).
3. **Localisation start:** with `KIDNAP_PRIOR="0 0 0"` the robot is placed on the start mark from the first second; the
   first recognised fix may still take several metres (DIAG.md: 6 m from a cold start). Route A starts along the long
   corridor, where drive 10 has the most views.
4. **Blind zone** < 1 m in front, and nothing sees behind or beside: no reversing, low speed, a person beside it.
5. **The robot's twist_mux, joystick model and button numbers are read from the mirror and the Jetson's copy of the
   same Clearpath package, not from the robot itself** - DEPLOY_NOTES.md R1 checks them read-only before anything runs.
6. **Two ROS masters:** the Jetson's `/cmd_vel` does not exist on the robot and vice versa; only the TCP link crosses.
   A forgotten test receiver cannot take the link (run names must match).
7. **Timing:** move_base must not be starved. In dry run 8 (Jetson load 8-9: Gazebo, file uploads, other jobs;
   the simulator deliberately at `nice 19`) one costmap update took 4.5 s and the robot overshot the start mark by
   1 m into the junction. On the real run the navigation layer runs at normal priority, and **no helper work and no
   storage uploads run during the drive** (ENGINEERING_NOTES.md rule 23; pause the storage sense's uploads first).
8. **Simulator is not the building:** the simulated walls are drive 10's map, so map and world agree perfectly; the
   camera stand-in is a RealSense model (640 x 480, 87 deg) at the ZED's mounting position, not a ZED X.

---

## 11. Files (all Jetson, this folder)

| file | what |
|---|---|
| `DESIGN.md`, `PASS_LINES.md`, `PROTOCOL.md`, `DEPLOY_NOTES.md` | this design, the registered pass lines, the user's steps, the deploy steps to approve |
| `maps/export_nav_map.py`, `maps/d10_nav.{pgm,yaml,json}` | drive 10's saved 2D map for map_server, with provenance |
| `config/*.yaml` | move_base, global planner, DWA, costmaps, camera obstacles |
| `move_base_nav.launch`, `nav_run.launch` | move_base (shared), the real-run navigation layer |
| `nav_cmd_sender.py` (Jetson), `nav_cmd_recv.py` (robot, **not deployed**) | the command link |
| `nav_goals.py`, `score_nav.py`, `check_goals.py` | goals + recording, scoring + figure, goal checks |
| `goals_d10_route_A.yaml`, `goals_d10_route_B.yaml` | the goal points |
| `start_nav.sh`, `go_nav.sh`, `stop_nav.sh` | real run: start the layer, send a route, stop |
| `sim/` | dry run: `grid_to_world.py` + `d10_floor.world`, `sim_nav.launch`, `run_sim_dryrun.sh <out> [routeA\|routeB\|both] [safety\|nosafety]`, `sim_localiser.py`, `sim_joy.py`, `sim_husky_extras.yaml` (simulator-only turn calibration), `safety_tests.py` + `plot_safety.py`, `turn_probe.py` (`PROBE=1`), `recv_selftest.sh` (the robot receiver's refusals) |
| `sim_results/` | dry-run outputs, parameter read-backs (`<run>/params_move_base.yaml`, `dynparam_readback.txt`, `params_obstacles.yaml`, `params_twist_mux.yaml`), figures: `run9/A_score/nav_paths.png`, `run10/A_score/`, `run10/B_score/`, `run9/safety/safety_stops.png`, `run6/safety/`, `run2/A_score/` (wheel odometry only); runs 3-8 kept as the record of section 9.2 |
