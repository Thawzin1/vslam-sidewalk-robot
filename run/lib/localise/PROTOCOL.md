# Localisation demo - what you do at the lab (about 30 min)

**What it shows:** the robot is given drive 4's map, with no new map being drawn, and has to recognise where
it is. You put it on 5 spots; each time the map program forgets where it is, and we watch whether it finds itself.
**You only do the physical part.** the Jetson operator runs every command on the Jetson and the robot, and tells you when to move.

**Picture of the spots:** `marks.png` in this folder. Arrows show which way the **camera** must face.

**Bring:** a tape measure and floor tape. **Robot charged.**
**Lighting: as for drive 4** (corridor lights on). The desk test (about 03:35, at night) could not recognise
the start mark from a wrong guess while standing still (RESULTS.md, finding 2). **The 2 m drive at each spot
is the only known difference from that test, so do not skip it.**

---

**Changed after session 2 (26 Sept ~07:25):** at every start the map program is told the robot is 40 m OUTSIDE the
building, so on the live map the robot is off the map until it really recognises the spot. That is expected. "Localised"
now means RTAB-Map accepted a fix, and the page shows how far the dot is from the tape (PASS_LINES.md, kidnapped-start
addendum).

## Before you start: tape 3 spots (5 min)

Measure along the **middle** of the corridor from the **start mark** (the tape from every drive).
Put an X on the floor at each spot.

| spot | where | how to find it |
|---|---|---|
| **A** | long corridor, **6.0 m** from the start mark | stand on the start mark facing down the long corridor (the way the robot starts every drive), measure 6.0 m straight ahead |
| **B** | long corridor, **10.5 m** from the start mark | same line, 10.5 m; roughly 1-1.5 m before the spot where drive 4 turned round at the far end |
| **C** | side corridor, **4.0 m** from the start mark | the side corridor that branches off to the **right** of the start mark when you face down the long corridor; measure 4.0 m down its middle |

## The session

**Rule at every stop: park, wait for the Jetson operator's "go", then give the robot ONE small nudge with the controller (not by hand) - about 10 cm
forward, or turn about 5 deg - and then hands off, stand still for the full wait.** A push while parked
is learned as gyroscope drift.

*Why the nudge (added 26 Sept 2026, after session 1):* the map program only looks again when the robot
has moved at least 5 cm or turned about 3 deg since its last picture. Parked dead still, it stops
looking: in session 1 it recognised the start mark on its 2nd picture and then never checked again, and
the watcher missed that one moment (RESULTS.md, "Session 1 diagnosis"). The nudge makes it look again
at every spot. Keep it small: 10 cm, not more.

| # | you do | wait | then say |
|---|---|---|---|
| 0 | Robot on the **start mark**, facing down the long corridor, exactly as for a drive. Say **"start localise"**. the Jetson operator starts the robot's side and the Jetson's. | until the Jetson operator says **READY** (about 3 min) | - |
| 1 | **Start 1 (start mark).** When the Jetson operator says "go": nudge about 10 cm forward (or turn about 5 deg), then keep still. | 60 s | "done 1" |
| 2 | Drive slowly to **A**, the camera facing **away** from the start. Park on the X. Say **"start 2"**. | the Jetson operator says "go", nudge 10 cm or turn 5 deg, then 60 s still | "done 2" |
| 3 | Drive to **B**, **spin round on the spot** so the camera faces **back toward the start**. Park on the X. Say **"start 3"**. | "go", nudge, then 60 s still | "done 3" |
| 4 | Drive back to **A**, still facing **toward the start**. Park on the X. Say **"start 4"**. | "go", nudge, then 60 s still | "done 4" |
| 5 | Drive to the start mark and turn **right** into the side corridor. Drive to **C** and **spin round** so the camera faces **back to the junction**. Park on the X. Say **"start 5"**. | "go", nudge, then 60 s still | "done 5" |
| 6 | Drive back to the **start mark** and park facing down the long corridor, as at the start. Say **"park"**. | 60 s still | - |

**If the Jetson operator says "not localised" after the 60 s at a start:** drive **slowly, straight ahead, 2 m** (the way
the camera faces), stop, and wait **30 s** more. Then say "done", localised or not. Never more than that: the
pass line allows at most 3 m of driving.

**Why the robot must face a given way:** the map recognises a place by what the camera sees, so it only knows
the views it saw on drive 4 (ENGINEERING_NOTES.md section 2.13a). In the side corridor, drive 4 always faced the junction.

## Time

| part | minutes (estimate) |
|---|---|
| taping 3 spots | 5 |
| start-up to READY | 3-4 |
| 5 starts (15 s reload + 60 s wait + at most 2 m and 30 s each) | 7-11 |
| driving between spots | 5 |
| park and shut down | 2 |
| **total** | **about 25-30** |

Afterwards (no action from you): the robot replays its LiDAR recording, parked, on its own card. the Jetson operator then
scores the session against `PASS_LINES.md` and writes the results.
