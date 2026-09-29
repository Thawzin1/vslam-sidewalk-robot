# Navigation - deploy steps (FOR THE USER TO APPROVE before anything touches the robot)

*27 Sept 2026. Every path names its machine. Jetson folder = `<repo>/catkin_ws/src/sidewalk_navigation/`
(below `$N`). Robot card folder = `/media/administrator/USB Drive/slam_series2/tools/` (below `$T`, where our other robot
tools already live).*

**What needs approval, in one line:** copying ONE file of ours (`nav_cmd_recv.py`) to our own folder on the robot's card,
running four read-only checks there, and running that one program during the navigation session. Nothing of the
colleague's is changed, nothing is installed, and the program stops itself if anything is wrong.

*Plain terms: the robot gets a small "remote hand" program of ours that obeys the Jetson only while you hold R2, and that
Clearpath's own switch ranks below the joystick. It is removed (or simply not started) when you say so.*

---

## Robot steps (R) - each needs the user's yes

| step | where | command (run by the Jetson operator over `ssh robot-usb` once approved) | what it does | changes anything? |
|---|---|---|---|---|
| **R0** copy | Jetson -> robot card | `scp -p "$N/nav_cmd_recv.py" robot-usb:"'$T/'"` then `ssh robot-usb sha256sum "'$T/nav_cmd_recv.py'"` (compare with the Jetson's) | puts our receiver next to our other robot tools | adds one file to OUR folder on the card |
| **R1** read-only checks | robot | `rosparam get /twist_mux` ; `rostopic info /cmd_vel` ; `rostopic hz -w 20 /joy_teleop/joy` (10 s) ; `rostopic echo /joy_teleop/joy/buttons` while the user presses and holds R2 | confirms: twist_mux has `cmd_vel` at priority 1 and `joy_teleop/cmd_vel` at 10 and the `e_stop` lock; **`/cmd_vel` has NO publisher** (the colleague's move_base is not running); the joystick repeats at ~20 Hz; **which button number R2 is** (expected 7) | no (reads only) |
| **R2** the wire | robot + Jetson | Jetson: `ping -c 5 <robot-usb-address>`; robot: `ip -br addr \| grep <the USB-C link prefix>` | the USB-C link is up and no other computer's ethernet cable is plugged into the robot (it would take <jetson-usb-address>) | no |
| **R3** start the receiver (only for the session) | robot | `cd "$T" && setsid nohup python3 nav_cmd_recv.py __name:=nav_cmd_recv _run:=<run> _deadman_button:=<from R1> > ~/jobs/<run>_navcmd_recv.log 2>&1 < /dev/null & echo $! > ~/jobs/<run>_navcmd_recv.pid` | listens on <robot-usb-address>:8112 only; publishes `/cmd_vel` only while R2 is held and the Jetson says go | publishes on `/cmd_vel` (twist_mux priority 1) while running |
| **R4** bench check, wheels OFF the ground or robot E-STOPPED | robot + Jetson | with `start_nav.sh` up and NO goal: the jobs-page line `<run>_navcmd` must read `robot stopped (Jetson: planner idle)`; press/release R2 -> `button held/released` follows within 1 s | the whole chain without motion | no |
| **R5** stop the receiver | robot | `kill -INT $(cat ~/jobs/<run>_navcmd_recv.pid)` (sends zeros, exits) | ends the session | removes our process |

Why these are safe to approve: R1/R2/R4 read only; R3 publishes on the lowest-priority input of Clearpath's own switch,
so the joystick (L1) and the e-stop override it by design, and it moves the robot only while R2 is held.

---

## Jetson steps (J) - the Jetson operator runs these (no approval needed beyond the run itself)

```bash
# J0 (Jetson) nothing heavy alongside (sim dry run 8: one costmap update took 4.5 s under load and the robot overshot
#    a goal by 1 m). Check that no big copy or upload is running on the Jetson and that no helper jobs run (rule 23).

# J1 (Jetson) localisation on drive 10's map, told it starts ON the start mark facing east; wheel bridge over the wire first
MAP_DB=~/slam_series2/localise/s2_static_10_map.db KIDNAP_PRIOR="0 0 0" \
ROBOT_ADDRS=<robot-usb-address>,<robot WiFi address> \
bash "<repo>/run/lib/localise/start_localise.sh" s2_nav_01 \
     ~/slam_series2/localise/s2_static_10_map.db
# (robot side first, as for every localisation session:  robot_side.sh start s2_nav_01  - LiDAR recording + wheel bridge)

# J2 (Jetson) the navigation layer: map server, camera obstacles, move_base, command sender; waits for the robot's receiver
bash "$N/start_nav.sh" s2_nav_01

# J3 (Jetson) ONLY when the user holds R2 beside the robot and says "go"
bash "$N/go_nav.sh" s2_nav_01 A

# J4 (Jetson) after the last goal (or at any time): stop the navigation layer, then the localisation as usual
bash "$N/stop_nav.sh" s2_nav_01
bash "<repo>/run/lib/localise/stop_localise.sh" s2_nav_01

# J5 (Jetson) score + figure (planned vs driven), then the LiDAR second opinion when the robot-side replay exists
nice -n 19 python3 "$N/score_nav.py" --run-dir ~/.run_records/s2_nav_01/nav_A --map "$N/maps/d10_nav.yaml" \
    --out "results/navigation/s2_nav_01" --title "s2_nav_01 route A"
```

- `start_localise.sh` refuses if the live drive script changed since it was staged (its own sha256 check; both staged
  copies matched on 27 Sept 10:00 Hamilton). Re-stage per `localisation_demo_2026-09-26/DEPLOY_NOTES.md` if it refuses.
- **Watch (rules 8 and 13):** jobs page `<jobs page, port 8096>` lines `s2_nav_01_nav` (goal k/N, distance to goal),
  `s2_nav_01_navcmd` (link, gate, what the robot is doing and why), `s2_nav_01_localise`, `_camera`, `_bridge`. Arm a
  1-minute watcher on those files that alerts on: link DOWN, gate SHUT for a reason other than "planner idle",
  `_localise` saying no fix for > 60 s while moving, any Traceback in `~/.run_records/s2_nav_01/nav*.log`.
- **Stop at once, from the Jetson:** `touch ~/slam_series2/NAV_PAUSE` (the gate shuts; the robot is sent "stop").

## Traps found while preparing (for docs/SOLVED.md)

- **roslaunch splits `args=` on spaces.** The folder path contains spaces (folder names with spaces in them), so
  `map_server args="$(arg here)/maps/d10_nav.yaml"` died with its usage message. Fix: single quotes inside:
  `args="'$(arg here)/maps/d10_nav.yaml'"` (roslaunch uses shlex). `rosparam file=` paths are not affected.
- **`~/sidewalk_sim/demo_sim_jetson.sh` runs `pkill -f rosmaster` and `pkill -f gzserver`** - it would kill the LIVE
  master on 11311 (drives, localisation, the live map page) and violates rule 8. The dry run here uses its own ports
  (ROS 11411, Gazebo 11445) and stops only the process groups it started.
- **A map -> odom publisher must post-date its stamp** (RTAB-Map uses `tf_tolerance` 0.100 s, printed in every drive's
  mapping.log); without it move_base fails "extrapolation into the future" and aborts the goal within 4 s (sim run 3).
- **`$!` after `cd X && cmd &` is the subshell, not the program** - killing it leaves the program running (sim run 1).
  Put `cd` on its own line and background the program alone.
