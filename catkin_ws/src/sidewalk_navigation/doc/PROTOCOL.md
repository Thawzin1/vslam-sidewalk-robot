# Navigation demo - what the user does (physical steps only)

*27 Sept 2026. the Jetson operator runs every command and every check (DEPLOY_NOTES.md); the user does only what needs hands and eyes.
Route A first (long corridor, out and back, ~28 m). Route B (the whole loop, ~58 m) only if route A passed.*

**Before the day:** the robot is charged (tell the Jetson operator how charged - the voltage reading is not a readiness check), and the
user has said **yes to the robot steps R0-R5** in DEPLOY_NOTES.md.

---

## The one rule

> **The robot moves only while you HOLD R2 (the lower right trigger on the joystick).**
> **Let go of R2 = it stops.** Press **L1** (upper left) and use the left stick = you drive it yourself.
> The red button on the Husky cuts the motors whatever the software does.

*Plain terms: you are the brake. If anything looks wrong - too close to a wall, a person steps out, it turns the wrong
way - just let go. Nothing breaks; the Jetson operator resumes it when you press R2 again.*

---

## Steps

| # | where | you do | the Jetson operator does meanwhile | you should see / hear |
|---|---|---|---|---|
| 1 | lab | Unplug **any other computer's ethernet cable from the robot** (it steals the Jetson's wire address). Leave the **Jetson-robot USB-C cable** in. | checks the wire (R2) | - |
| 2 | lab | Switch the robot on. Park it **on the start mark, facing east down the long corridor**, as for every drive. Joystick on and paired. | robot-side start (LiDAR recording + wheel bridge); read-only checks R1 | the Jetson operator asks you to **press and hold R2 for 3 s**, then let go (this finds R2's button number) |
| 3 | lab | Stand still beside the robot. | starts the localisation (J1) and the navigation layer (J2), starts the receiver on the robot (R3) | the Jetson operator says **"READY - gate shut"** |
| 4 | lab | Wheel check: **press the red e-stop**, hold R2 for 5 s, let go, release the e-stop. | watches the robot say "button held" / "button released" (R4) | the robot does not move (motors off) |
| 5 | lab | Stand **beside and slightly behind** the robot, on the side away from the nearest wall. Hold **R2** and say **"go"**. | `go_nav.sh s2_nav_01 A` | the robot sets off east at walking-slow speed (0.3 m/s) |
| 6 | long corridor | Walk with it, R2 held. Keep people out of its way for this first run. | watches the jobs page: goal 1 of 2, distance to goal | it stops at the east junction (~14 m), **turns round on the spot** to face west, pauses ~10 s |
| 7 | east junction | Keep holding R2 through the turn and the pause. | goal 2 sent automatically | it drives back west |
| 8 | start mark | Keep holding R2 until it has **turned to face east** and stopped. Then let go. Say **"done"**. | stops the navigation layer (J4) | "NAV DONE 2/2" on the jobs page |
| 9 | start mark | **Stand still 60 s** (the end-of-run stillness, as for every drive), then say **"park"**. | stops localisation and robot side, starts the scoring (J5) | - |
| 10 | - | If route A passed and there is battery: repeat 5-9 with **route B** (the Jetson operator: `go_nav.sh s2_nav_01 B`): east, then south down the east corridor, west along the south corridor, north up the west corridor, back to the start mark. | | 4 stops, one at each corner, each with a turn on the spot. **Watch each corner turn: if the robot slides sideways towards a wall while turning, let go of R2** - in the simulator it slid 1 m at the south-west corner (DESIGN.md section 9.4) |

## When to let go of R2 (and say why afterwards)

- the robot is heading for a wall, a door frame, a bin or a foot - anything within about **30 cm** of its path;
- it turns the wrong way, or starts to spin;
- a person walks towards it;
- anything you are not sure about.

Each let-go is recorded with its time (the robot reports "button released"), so say briefly why - it goes into the
results (pass line N2).

## If something goes wrong

| you see | do | then |
|---|---|---|
| it does not start when you hold R2 and say "go" | keep holding; wait 10 s | the Jetson operator reads the jobs page: the robot line says why (e.g. "Jetson: position stale") |
| it stops by itself mid-corridor and stays stopped > 10 s | keep holding R2 | the Jetson operator checks: camera, localisation, or the route blocked |
| it hits anything | let go of R2, press the red e-stop | the run is stopped and recorded as N2 failed |
| the joystick disconnects | the robot stops by itself (no joystick messages = stop) | re-pair, hold R2 again |

**Charging:** the robot must stay ON from step 2 to step 9 (about 20 minutes for route A). It can charge right after
"park" once the Jetson operator has confirmed the LiDAR recording is closed.
