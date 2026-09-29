# SLAM session - what you do (physical steps only)

*About 20-30 minutes in all (about 120 m of driving, 6 stops of 30 s). the Jetson operator runs every command, every check,
and tells you when to move. You drive, stop, walk, and say "park". Figure: `route_plan.png` (this folder) - drive 10's map from above with the route, the
stops and the candidate new areas.*

*Words: **start mark** = the taped spot every drive starts on; **JOINED** = the robot has recognised drive 10's map;
**reverse** = drive backwards without turning the robot round; **30 s stop** = robot still, hands off, 30 seconds.*

---

## Before (the Jetson operator checks and tells you; you only act on what the Jetson operator asks)

1. Robot charged enough - the Jetson operator will ask you (the battery reading is not a readiness check).
2. Robot and Jetson on the same lab WiFi; no other computer's ethernet cable plugged into the Jetson.
3. Walk the route once ahead of the robot (switches the motion-sensor lights on in the corridors; also lets you
   look into the candidate new areas and decide which ones are real open corridors - areas A, B, D, E in the route table and on `route_plan.png`).
4. Park the robot **on the start mark, facing east down the long corridor** - exactly as for every drive (drive 10
   started the same way; this is what lets the robot recognise the place at once).

## Start

5. Say "ready". the Jetson operator starts the robot's recording and the Jetson (`start_slam.sh`).
6. **Hands off, robot still** until the Jetson operator says **READY** (the gyroscope is being measured, about 30-60 s).
7. After READY, stay still **at most 60 s more**. the Jetson operator tells you "JOINED" (it usually comes within seconds).
   **If the Jetson operator has not said JOINED after 60 s: drive on anyway, slowly, east down the long corridor.**
   *Why: standing still, the recognition score does not improve; driving along drive 10's route is what makes it
   recognise (localisation diagnosis, 27 Sept).*
8. If the Jetson operator says **STOP** (a "suspect join" - it recognised the wrong place): stop, drive back to the start mark,
   and the Jetson operator restarts with the next run id. Nothing is lost.

## The route (slow, the usual speed; 30 s stops, hands off, where marked)

Drive 10 drove every corridor in both directions, so either direction along a known corridor is fine. **In a new
area, come back out in REVERSE** (do not turn round inside it): reversing, the camera keeps facing into the new area
and sees what it saw going in, so the map can check itself there (ENGINEERING_NOTES.md section 2.13a). Keep each new area
**under 10 m in**.

| # | where | what you do | new area? |
|---|---|---|---|
| 1 | start mark -> east along the long corridor (15 m) to the far corner | drive | no |
| 2 | far corner (**S1** on the figure) | **30 s stop** | |
| 3 | turn left (north) up the side corridor to where drive 10 turned round (about 10 m up) | drive | no |
| 4 | **area A**: carry on north - the corridor continues (drive 10's map shows it free to about 16 m up, never driven) | drive in slowly, **at most 6 m** past drive 10's turn | **yes** |
| 5 | end of A (**S2**) | **30 s stop**; then **REVERSE** back to drive 10's turn point (~10 m up), turn round there, drive south | |
| 6 | past the far corner, south down the right-hand corridor (about 15 m) to the bottom-right corner (**S3**) | drive; **30 s stop** there - with **person event P1** | no |
| 7 | *(optional, only if time and battery allow)* **area E**: the corridor going east at that corner | in at most 5 m, 30 s stop, reverse out | yes |
| 8 | west along the bottom corridor (about 15 m) - **person event P2** on this straight | drive | no |
| 9 | bottom-left corner (**S4**) | **30 s stop** | |
| 10 | **area B**: straight on, west, past the corner - the corridor continues (seen about 6 m) | drive in, **at most 6 m** | **yes** |
| 11 | end of B (**S5**) | **30 s stop**; then **REVERSE** back to the corner, turn right (north) | |
| 12 | north up the left-hand corridor (about 15 m) back to the start mark | drive | no |
| 13 | **area D**: arriving at the start mark from the south, carry **straight on north** across the junction - the corridor continues (seen to about 8 m, never driven) | drive in **at most 8 m** | **yes** |
| 14 | end of D (**S6**) | **30 s stop**; then **REVERSE** back to the start mark | |
| 15 | **start mark, facing east** (turn on the mark to face east, as at the start) | stop, say **"park"**, hands off: the robot stands still 60 s while the map closes | |

- If an area turns out to be closed (a door shut, furniture), skip it and say so; at least **one** new area of about
  6-8 m is needed for pass line S5. A, B and D all show as open floor on drive 10's saved map (`route_plan.png`).
- **Every ~5 minutes of driving without a stop, make an extra 30 s stop** (the blend re-measures the gyroscope).
- **WiFi:** the far corridors dropped the WiFi link about 1 time in 9 on drive 10. The live map page may freeze
  there - **keep driving**; everything is recorded on the Jetson and the robot. the Jetson operator will not ask you to stop for it.

## Person events (for the "moving people" test)

- **P1 (robot parked, at the bottom-right corner stop S3, route row 6):** once the robot has stood still ~10 s, walk **across in front of the camera,
  2-4 m away**, left to right, then back right to left (about 10 s in view), then out of view. Then finish the 30 s.
- **P2 (robot moving, along the bottom corridor, route row 8):** walk **ahead of the robot, in its view, 2-4 m in front**,
  for about 10 m (walking backwards facing it, or ask a colleague to walk towards it). Keep driving slowly.
- Anyone else walking past at any time is fine and useful - no need to stop.
- Nothing to announce: the camera records who was in view and when.

## After "park"

E1. Stay still until the Jetson operator says **"closed"** (the map is saved - about 1-3 minutes).
E2. the Jetson operator stops the robot's recording. **Leave the robot parked and ON** until the Jetson operator says the LiDAR replay on the
    robot has finished (it runs on the robot, parked); **then charge it** (turning it off to charge is fine after
    that; the Jetson has its own power).

*What the Jetson operator does meanwhile (no action from you): watches the jobs page lines `s2_slam_01_slam` (JOINED),
`_camera`, `_bridge`, `_mask`; checks the map closed properly and drive 10's master map is untouched; then the
results pack (RESULTS_PACK_PLAN.md).*
