# Localisation demo - pass lines (registered 26 Sept 2026, 03:59 Hamilton (file time), BEFORE any lab run)

**Honest note on timing:** these lines were written about 6 minutes after the desk test ended (03:53) and
were shaped by it: the 20-point check, the 0.30 m line and the uncertainty figures all use what the desk
test showed. They are registered before any LAB run, not before any data.

**What is being shown (roadmap step 2, Mitacs deliverable D3):** the robot is given drive 4's saved map with
mapping switched off (localisation mode: RTAB-Map (the mapping program) reads the map and adds nothing to it)
and has to work out where on that map it is, from what the camera sees.

**The session:** one session, **5 starts** on 3 taped marks plus the start mark (PROTOCOL.md, `marks.png`).
At each start the map program is made to forget where the robot is (`next_start.sh` reloads a fresh copy of the
map), so each start is a new "where am I?". Its default guess is **drive 4's last saved position**
(`RGBD/StartAtOrigin=false`: the map resumes from `Admin.opt_last_localization`), which happens to be on the
start mark, (0.08, -0.01), because drive 4 parked there. So start 1 (on
the start mark) begins with a correct guess. Starts 2-5 begin with a guess **4 to 10.5 m wrong**, and only
real recognition can fix them.
*Plain terms: start 1 is the easy control; starts 2-5 are the actual test.*

**N:** 5 starts in 1 session (N = 1 session, not the project's N = 5). Every result is given as counts
("4 of 5"), never as a percentage.

**Scored by** `score_localise.py` from the watcher's files and the robot's LiDAR replay of the same session.
The LiDAR is a second opinion, not ground truth (ENGINEERING_NOTES.md rule 19). Every distance below is "agreement with the
LiDAR estimate".

---

## P1 - it finds itself

**Pass: at least 4 of 5 starts get a first fix within 120 s of the start AND within 3 m of driving, and that
fix is not a false one (P3).**

- A "fix" is a recognition event on `/rtabmap/info`. It is either "recognised" (the current picture matched a
  saved picture by appearance, and its 3D check passed) or a "nearby re-match" (a match against saved pictures
  near where the robot is currently believed to be). The robot's dot sitting in the right place does **not**
  count, because the map program starts with a guess (see above).
- **The 120 s only just fits the protocol:** 15 s reload + 60 s parked + about 10 s to drive 2 m slowly +
  30 s parked = about 115 s. A start that needs the 2 m push has almost no spare time.
- *Pass means:* on a fresh start the robot says "I have been here before" within two minutes and at most one
  short push forward. *Fail means:* it stayed lost, or needed more driving than a short push, at two or more starts.
- *Why it is worth doing:* this project has never shown localisation. Every run so far drew a new map. The
  project's own `rtabmap_localization.launch` could never have shown it: it always started in mapping mode
  (see RESULTS.md, finding 1).

## P2 - it puts itself in the right place

**Pass: at every start that got a fix, except at most one, the robot's position on the map (read about 5 s
after the first fix, with the robot parked) is within 0.30 m of where the LiDAR says drive 4's map should
have it.** The heading difference is reported beside it, not judged.

- **How the LiDAR position is turned into a position on drive 4's camera map.** Two steps, and neither uses
  the camera localiser's own answer, so the check is not circular:
  1. **This session's LiDAR frame to drive 4's LiDAR frame.** Both LiDAR paths are first moved so that their
     first node, the robot parked on the start mark, is (0, 0) facing down the corridor. This is how the drive
     packs already do it. Then this session's LiDAR walls are fitted onto drive 4's LiDAR walls with a 2D ICP
     (iterative closest point: pair every wall cell with the nearest wall cell of the other map, solve the
     best rotation and shift, repeat). The fit starts from a search over ±6° and ±0.8 m. That search absorbs
     the few centimetres and degrees by which the robot is placed on the start mark differently each time.
     *Own check:* the fit must stay within 0.30 m and 3° of "no shift", with a median pair distance of 0.10 m
     or less. If not, P2 is declared **NOT SCORABLE**, not failed.
  2. **Drive 4's LiDAR frame to drive 4's camera map, locally.** Drive 4 recorded both paths at the same
     moments. The scorer takes drive 4's LiDAR positions within 0.75 m of the point, facing within 45° the
     same way, and looks up where drive 4's camera map had the robot at those moments. It then applies the
     median offset.
     *Plain terms: drive 4's camera map is not a perfect copy of the building. It is about 6 % stretched and
     up to half a metre off the LiDAR map in places (drive 4 RESULTS.md: median 0.26 m, 95th percentile
     0.49 m). So "the right place on the map" means "where this map drew that spot". That is read off drive
     4 itself, not off the building's true coordinates.*
- **Uncertainty, stated in advance** (combined as the square root of the sum of squares, ENGINEERING_NOTES.md section 4 rule 0):
  - the ICP fit's median pair distance: 0.019 m in the self-test;
  - step 2's cross-pass resolution: **0.050 m median, 0.097 m 95th percentile, 0.119 m largest**
    (`selftest_output.txt`). This is drive 4's camera map at 29 moments, predicted from its OTHER passes
    only (moments within 10 s excluded). The earlier 0.032 / 0.090 m figure included the queried pass
    itself, so it only checked the plumbing;
  - LiDAR range noise: 0.03 m (ENGINEERING_NOTES.md rule 19: 15-30 mm).

  Combined, the tested parts come to **about 0.06 m typical and 0.11 m at the 95th percentile**.
  **Two parts are NOT tested yet, and the combined figure leaves them out:**
  - the LiDAR's own drift between the start mark and each spot, in this session (drive 4's LiDAR ended
    0.032 m from its start after correction, but the spots are up to 10.5 m out);
  - the wall-fit error between two DIFFERENT sessions (the self-test fitted drive 4 against a moved copy of
    itself; a real second session has different clutter and scan coverage).

  The scorer prints the fit's own pair distance for the real session. If it is over 0.10 m, P2 is not scorable. Each start's own figure is
  printed beside its result. The LiDAR-to-camera mounting offset has never been measured. Step 2 absorbs it,
  because the same robot carried both sensors on both days.
- *Why 0.30 m:* it is about 3 times the combined uncertainty, and about half the Husky's width (0.67 m).
  *Pass means:* the robot put itself on the map within half its own width of where the LiDAR says. *Fail means:*
  it "found itself" in roughly the right place but off by more than half a robot.
- **Needs:** the robot's LiDAR recording must **begin with the robot parked on the start mark** (PROTOCOL.md
  step 1), because step 1 depends on it.

## P3 - no false localisation

**Pass: zero fixes, at any start and at any moment (not only the first), that put the robot more than 1.0 m
from where the LiDAR says it is.**

- Live, the watcher flags every later jump over 1.0 m within a start as a "suspect jump" on the jobs page.
  The LiDAR check afterwards confirms or clears it.
- *Pass means:* when it says "I am here", it is never confidently wrong by a metre or more. *Fail means:* at
  least once it placed itself in the wrong spot. For navigation that is worse than staying lost, because a
  lost robot stops and a wrongly-placed one drives into a wall.

## P4 - the saved map is untouched

**Pass: drive 4's master map has the same checksum after the session as before (`master.sha256`), and the
last working copy still holds the master's number of nodes (540).**

- `stop_localise.sh` checks this automatically and writes `P4_map_untouched.txt`.
- *Pass means:* the demo read the map and changed nothing. This matters because the same map is the reference
  for every later session.

## P5 - it keeps up

**Pass: the map program's processing time per picture has a median under 1.0 s, its budget at one picture a
second (`Rtabmap/DetectionRate` 1). Source: the `RTAB-Map=...s` figure on each picture's line in `mapping.log`.**
Memory is reported, not judged: no memory log is started with the run, and there is no source to score it from.

- *Pass means:* the localiser is not falling behind the camera.

---

**What can be claimed if P1-P4 pass:** "From 5 starts in one session, the robot recognised where it was on a
map it had built the night before. It did so within 2 minutes at k of 5 starts, was placed within 0.30 m of an
independent LiDAR estimate at those starts, and was never confidently wrong by 1 m." Today this project cannot
claim any of that.

**What cannot be claimed even then:** repeatability across sessions or lighting (N = 1 session), or accuracy
against ground truth (there is none).

**Settings registered with these lines:**
- `Rtabmap/LoopThr` = **0.15** (from `rtabmap_localization.yaml`). Drive 4 mapped with 0.11. This is how sure
  the appearance stage must be before a place is even tried. 0.15 is **stricter** and kept on purpose: a wrong
  place costs more in localisation than in mapping.
- `Vis/MinInliers` = **20** (the fewest picture points that must line up
in 3D before a match is accepted). This is drive 4's own mapping value. The repo's localisation file says 25,
but the bench (RESULTS.md) showed the correct place reaching 20-21 and being refused at 25. It must not be
changed during or after the session to make a start pass.

## Addendum, 26 Sept 2026 ~09:35 Hamilton (after session 1) - protocol change only; pass lines unchanged

After session 1 (s2_loc_01) the protocol gained one step: at every spot, after "go", the robot is nudged
about 10 cm forward or turned about 5 deg, then kept still (PROTOCOL.md). Reason: RTAB-Map skips
recognition while the robot moves less than `RGBD/LinearUpdate` 0.05 m / `RGBD/AngularUpdate` 0.05 rad
(verified on this build), so a dead-still robot is checked only on its first pictures
(`session1_diag/`, RESULTS.md "Session 1 diagnosis"). The nudge counts toward P1's 3 m. **P0-P5 are
unchanged, and so are the settings: the minimum-movement values stay at 0.05 (the option of setting them
to 0 was not adopted).** The watcher now also counts a fix from RTAB-Map's published uncertainty and its
odometry-cache links (tested: `session1_diag/test_watch_replay.sh`).

## Addendum, 26 Sept 2026 07:13 Hamilton (file time; committed in 140708b at 07:14 Hamilton - the registration proof), after session 2, BEFORE any lab use: the "kidnapped start" pass lines
*(The heading first said "~07:25"; corrected on review, same day: the file was written at 07:13.)*

**Why:** session 2 (RESULTS.md "Session 2") never tested finding itself from nothing. Every reload started from the
map's saved position, the start mark. That guess looked right on the map, but at 4 of 5 spots it was 4-10 m wrong, and
RTAB-Map held back every correct recognition. P0-P5 above stay registered, and session 2 is scored against them. The
lines below are **new** and apply to the next session and after.

**Settings registered with these lines** (must not be changed during or after a session to make a start pass):
- each start's working copy gets an **off-map guess, (40, 40) facing 90 deg** (`set_prior.py`, via `next_start.sh` and
  `start_drive.sh`, `KIDNAP_PRIOR` default). *Plain terms: at every start the map program is told "you are 40 m outside
  the building", so only a real recognition can bring the robot back onto the map.*
- `Rtabmap/LoopThr` = **0.05** (the recognition score a place must reach before it is tried; it was 0.15). Chosen
  because with 0.15 no correct recognition was ever accepted from a wrong guess (session 2: 3 of 5 starts; offline: 0 of
  3), and with 0.05 all 3 offline stretches were accepted. *Pass means the bar is low enough to be reached; the 3D check
  below is what stops wrong places.*
- `Vis/MinInliers` 20, `RGBD/MaxOdomCacheSize` 10 (the second-opinion rule, kept), minimum movement 0.05 m / 0.05 rad:
  unchanged.
- **A fix = RTAB-Map ACCEPTED it**: its published localisation uncertainty drops below 9999 (watcher v3). A recognition
  it is holding ("pending") does not count, and neither does the dot's position.
- Each taped spot's expected place on the map comes from the LiDAR (PASS_LINES P2 method), NOT from the nearest point of
  drive 4's path. Until session 2's LiDAR is scored, `marks.json` holds the taped distances, for the live page only.

### K1 - it finds itself from a wrong guess
**Pass: at least 4 of 5 starts get an accepted fix within 120 s of the start and within 3 m of driving.**
*Pass means:* told it is 40 m away, it still recognises where it is within two minutes and one short push. *Fail means:*
at two or more spots it stayed off the map.
*Why it is worth doing:* it is the claim session 2 could not make: "finds itself on a saved map from nothing", not
"keeps its place once placed".

### K2 - in the right place
**Pass: at every start with an accepted fix, except at most one, the position about 5 s after that fix (robot parked)
is within 0.30 m of where the LiDAR says drive 4's map should have that taped spot.** This is the same method,
uncertainty and not-scorable rule as P2. The distance to the provisional taped position is also reported, not judged.
*Pass means:* the dot lands on the tape, not merely somewhere in the corridor.

### K3 - never confidently wrong
**Pass: zero accepted fixes, at any moment of the session, more than 1.0 m from the LiDAR-derived position.**
*Fail means:* at least once it accepted a wrong place. At 0.05 this is the main risk of the new setting, and it is the
line that would reject the setting.

**Evidence for choosing 0.05, registered before the lab:** offline, 3 stretches of drive 6 against a copy of drive 4's
map, off-map guess. Accepted at pictures 4, 3 and 5. 44 accepted pictures, 0.16-0.52 m from drive 6's own tracking, 0 over
1.0 m. Processing time median 0.58-0.71 s. N = 3 stretches of 1 drive, all moving; recognition while fully parked at
0.05 is untested, so the nudge and 2 m drive stay in the protocol. K1-K3 are judged on the lab session only.

## Addendum, 26 Sept 2026 07:40 Hamilton (independent review of K1-K3, BEFORE any lab use)

**K2 is amended; K1 and K3 are unchanged.**
- **K2 now reads:** at every start with an accepted fix, except at most one, the position **at the moment of that fix**
  is within 0.30 m of the LiDAR-derived position **at that same moment**, taken through drive 4's map as in P2.
  This is the same method as K3.
  *Why:* a fix can come during the nudge or the 2 m drive, so "about 5 s after, parked" could be read at a different
  place from the fix. The distance to the provisional taped spot is still reported, not judged.
- The live page's "WRONG PLACE?" now compares with the **taped spot moved on by the odometry since the start**, not the
  bare spot. It is a live hint only; K1-K3 are judged from the LiDAR afterwards.
- The value of `Rtabmap/LoopThr` for the lab is decided by the whole-of-drive-6 offline test (RESULTS.md, "Review
  fixes"). If that test changes the value from 0.05, a further dated addendum records it here before any lab use.

## Addendum, 26 Sept 2026 09:10 Hamilton (whole-of-drive-6 offline test, BEFORE any lab use): LoopThr 0.05 -> 0.08

**`Rtabmap/LoopThr` for the next session is 0.08** (`localise_run.launch` default; checked with `roslaunch --dump-params`:
0.08. `RGBD/AggressiveLoopThr` stays at this build's default 0.05, so before the first link the bar is still 0.05).
K1-K3, the off-map guess and every other setting are unchanged.

**Evidence** (`session2_diag/offline_kidnap_windows/`). Drive 6 was cut into 33 stretches of 40 pictures. Each was run
against a read-only copy of drive 4's map with the off-map guess, once at 0.05 and once at 0.08. They were scored against
drive 6's LiDAR, taken through drive 4's map as in P2. All 66 runs completed, and the map copy's checksum was unchanged.

| | 0.05 | 0.08 |
|---|---|---|
| stretches wholly outside drive 4's map that got a fix (should be 0) | **3 of 20** (at pictures 10, 10, 32) | **0 of 20** |
| stretches mostly inside the map that got a fix | 4 of 4 (at pictures 3, 3, 7, 3) | 3 of 4 (at pictures 17, 27, 11) |
| stretches with an accepted picture more than 1 m off | 9 of 16 | 5 of 10 |
| recognitions more than 1 m off | 84 of 120 scored | 13 of 28 scored |

*Plain terms:* at 0.05 the map three times said "I know where I am" while the robot was somewhere drive 4 never
mapped. At 0.08 that never happened, and in-map places were still found, only more slowly (11-27 pictures, about
11-27 s). K3 is the line that rejects a setting, so the stricter value is taken.

**Not settled by this test, and carried as a risk into the session.** Late in drive 6 (stretches from picture 601 on),
fixes at BOTH values sit 1.1-2.3 m from the LiDAR estimate. The same stretches show it at both values, so the threshold
is not what causes it. The localiser keeps the robot within 0.1-0.3 m of drive 4's route there, while the LiDAR puts it
0.6-1.6 m to the side.
- **The reference is suspect there.** Drive 6's LiDAR map does not close (end gap 5.08 m after correction), and its
  walls near those stretches sit up to 0.85 m off drive 4's.
- **But that does not fully explain 1.5-2.3 m.**

So K3 may fail in the lab at 0.08 too, and this test cannot say which way.

## Addendum, 26 Sept 2026 09:59 Hamilton (BEFORE any session-3 data exists): wall fit v2 for P2, K2 and K3

**What changes:** only the wall fit, step 1 of the P2 method. The fit lines this session's LiDAR walls up with drive 4's
LiDAR walls (2D ICP, iterative closest point). K2 and K3 use it, and so does P2 for any later session. Every
threshold (0.30 m, 1.0 m, "all but at most one") and every other step are unchanged. Session 2's verdicts stay as
scored with the original fit (v1). The fit v2 figures for session 2 are reported beside them, labelled "for
information", and change no verdict.

**Fit v2** (`score_localise.py --fit v2`, now the default; `--fit v1` reproduces session 2):
1. **Trim 0.80** instead of 0.95. The fit ignores the 20 % of wall pairs that are furthest apart, not 5 %.
   *Plain terms:* a second visit has furniture, people and doors where the map had none. With 5 % ignored, those
   still pull on the fit.
2. **Stability check.** The same fit is repeated on 8 parts of the session's walls: 6 random halves, and the east and
   west halves of the map. Every repeat must land within **0.10 m** of the median of the repeats, and their rotations
   within **1.0 deg** of each other.
   *Plain terms:* if a different half of the walls gives a different answer, no answer can be trusted. That is exactly
   how a corridor lets a fit slide.
3. **The v1 checks still apply:** shift at most 0.30 m, rotation at most 3 deg, median pair distance at most 0.10 m.
   If any check fails, P2/K2/K3 are **NOT SCORABLE**, not failed.

**Why these values.** They were chosen for stability only. Evidence is in `session2_diag/wall_fit_stability/`:
`trim_sweep.txt`, `bootstrap.txt` and the two scripts.

| trim | session 2 onto drive 4: largest deviation of the 8 repeats | known answer, 3.00 deg / (0.40, -0.20) m, 20 % of walls missing, 15 % clutter added: largest deviation |
|---|---|---|
| 0.95 (v1) | **0.834 m** (east half alone: (-0.91, -0.54) m) | 0.003 m |
| 0.90 | 0.328 m | 0.002 m |
| **0.80 (v2)** | **0.020 m** | **0.019 m** |
| 0.70 | 0.027 m | 0.079 m |

- **0.80 is the only trim tested that is steady on both.** Trims above it are unsteady on the one real two-session
  case. Trim 0.70 starts to lose the known answer.
- **The 0.10 m limit** is 5 times the 0.02 m spread seen at 0.80. It is still well under the 0.30 m pass line, so a fit
  that passes cannot by itself move a start across the line.
- **Tested on:** session 2 only. Drives 1, 2, 3, 5 and 6 cover 40-80 m areas against drive 4's 34 x 26 m, so a
  whole-map fit onto drive 4 is not a valid test for them. Drives 3 and 5 fail at every trim for that reason.
- **Checked against session 2's verdicts:** v2 changes none of them. Start 1 is still not scorable and start 4 still
  fails P2 (0.40 m against 0.48 m).
- **Honest note on timing:** these values were chosen AFTER session 2's figures were seen, because that is where the
  instability showed up. They were chosen on stability alone, and they are registered before any session-3 data
  exists.

## Addendum, 26 Sept 2026 11:21 Hamilton (session 3 run, its LiDAR replay NOT yet finished or looked at): the LiDAR must pass its own tape check

**Why:** watching the live LiDAR view during session 3, the user thought it drifted a lot, but was not sure. So before any
session-3 LiDAR output is opened, this check decides whether the LiDAR may be used as the reference at all.

**The check.** The robot stood on the same taped spot more than once in session 3. Each visit gives one pair:
- **Pair 1:** the start mark at the start of the recording (the first LiDAR node, parked before start 1) and at park
  (the last parked LiDAR position at the end of the session).
- **Pair 2:** spot A at start 2 (A_east) and at start 4 (A_west). The robot faced opposite ways on these two visits.

How each pair is compared:
- Each visit's position is the LiDAR's corrected position (`lidar.tum`, its own frame; no wall fit, no camera data).
  It is taken at the moment the start was recorded in `starts.csv`, with the robot parked on the tape before the nudge.
  For park, it is the last parked position. Where no LiDAR node falls at that moment, the nearest node in the same
  parked period is used.
- The distance between the two positions of a pair is the "repeat gap". Pair 1's gap is the LiDAR map's own
  start-to-park gap; it should be about 0, because both visits are the same tape mark.
- Uncertainty: placing the robot on the tape by eye, about +-0.05 m per visit; LiDAR range noise 0.03 m.

**Verdict of the check:**
- **Both repeat gaps within 0.30 m:** the LiDAR is usable. P2, P3, K2 and K3 are scored against it as registered
  (wall fit v2, addendum 09:59), and labelled "agreement with the LiDAR estimate".
- **Either gap over 0.30 m:** the LiDAR is declared unusable for session 3. P2, P3, K2 and K3 are reported as "LiDAR
  reference failed its own check" and are NOT scored against it. The camera's fixes are then judged only against the
  tape: the taped spot moved on by the odometry since parking, which the watcher logs as `dist_to_mark_m`. That
  comparison is reported as "agreement with the tape", not as a pass or fail of P2, P3, K2 or K3.

*Plain terms:* if the LiDAR says the robot was in two different places while it stood on the same tape mark twice, the
LiDAR cannot be trusted to mark the camera's answers. 0.30 m is the same line the camera is held to.
This check's result is the first row of session 3's results table.

## Amendment, 26 Sept 2026 11:22 Hamilton, to the 11:21 addendum (session-3 LiDAR replay still NOT finished or looked at): parking on the tape is only an estimate

**The user's statement (26 Sept 2026):** "Tape mark stops are estimates not accurate, same stops". Asked how far, the
user chose "About 20-50 cm". So a parked robot can be up to about 0.5 m from the exact tape mark, and two parkings on
the "same" mark can differ by up to about 0.7 m. The 11:21 check assumed about 0.05 m per visit, which is wrong. It is
amended as follows.

1. **Primary LiDAR-drift check: the stability check in wall fit v2** (addendum 09:59), which does not depend on
   parking. The session's LiDAR walls are fitted onto drive 4's separately for the east half, the west half and 6
   random halves. All fits must agree within 0.10 m and 1.0 deg.
   *Plain terms:* a LiDAR map that drifted is bent, so its two halves line up with drive 4 in different places.
   The halves' disagreement (largest deviation, plus the east-vs-west difference on its own) is the **headline drift
   measure**.
   - **If it passes:** the LiDAR is usable. P2, P3, K2 and K3 are scored against it, labelled "agreement with the LiDAR
     estimate".
   - **If it fails:** P2, P3, K2 and K3 are **"not scorable this session: the LiDAR reference failed its own check"**.
2. **Secondary: the tape repeat visits** (the start mark at the start vs at park; spot A at start 2 vs start 4), limit
   **0.70 m** (the square root of 0.5 squared plus 0.5 squared).
   - A gap over 0.70 m shows the LiDAR drifted, and makes the LiDAR unusable as in point 1.
   - A gap within 0.70 m proves little: drift smaller than the parking spread cannot be seen this way. It is reported
     as that.
3. **The tape is NOT a fallback reference for the 0.30 m pass lines.** A reference with +-0.5 m of parking uncertainty
   cannot test a 0.30 m line. The watcher's camera-vs-tape figures (taped spot moved on by the odometry) are reported
   only as "consistent with the parked spot within the +-0.5 m parking uncertainty" (or not), never as a pass or fail.
   The 11:21 fallback sentence is withdrawn.
4. **Consequence for P2's and K2's "taped spot" wording:** the expected place comes from the LiDAR (as registered)
   and never from the tape. The provisional `marks.json` stays a live hint only.

**For future sessions:** give each spot a physical stop the wheels touch, such as tape-corner blocks. Failing that,
measure the parked robot's offset from the mark with a tape measure at every start, so the tape can reach about
+-0.05 m.

## Addendum, 26 Sept 2026 11:45 Hamilton (BEFORE the run): decision rule for LoopThr 0.065

**What is run:** the same 33-stretch offline test of drive 6 as for 0.05 and 0.08 (addendum 09:10), at `Rtabmap/LoopThr`
**0.065 only**. It uses the same scripts (`session2_diag/offline_kidnap_windows/`), the same 40-picture stretches and a
fresh copy of drive 4's map with the same off-map guess (40, 40, 90 deg). The copy's sha256 must equal the earlier
copy's (7f241120...), which proves the input is identical. Scoring is the same as before (`score_windows.py`).

**Decision rule, fixed now:**
- **Adopt 0.065** as the `localise_run.launch` default **only if 0 of the 20 stretches wholly outside drive 4's map
  get a fix** (0.05 gave 3 of 20; 0.08 gave 0 of 20).
- **If one or more get a fix, 0.08 stays.**
- The in-map fix rate (of the 4 stretches mostly inside the map) and the time to the first fix (in pictures) are
  reported beside 0.05 and 0.08. They are **reported, not judged**; the rule above alone decides.
- Accepted pictures more than 1 m from the LiDAR estimate are reported too. Drive 6's late stretches show that at
  both earlier values, and its cause is still open (addendum 09:10).
- If adopted, the launch default changes, the launch's parameter dump is checked, and this addendum is followed by a
  dated line recording the change.

*Plain terms:* 0.065 sits between a value that once said "I know where I am" somewhere never mapped (0.05) and one that
never did but was too strict to accept two correct starts in session 3 (0.08). It is used only if, like 0.08, it
never says it knows a place it has never seen.

**Result of the 0.065 rule, 12:34 Hamilton:** 33 of 33 runs completed, the map copy was unchanged, and the copy's
sha256 matched. **2 of the 20 stretches wholly outside drive 4's map got a fix** (stretches 681 and 1201, at pictures
11 and 39). **The rule fails: `Rtabmap/LoopThr` stays 0.08**, and the launch default was not changed.
For information: in-map, 4 of 4 were fixed at pictures 3, 3, 8 and 9 (0.05: 3, 3, 7, 3; 0.08: 3 of 4, at 17, 27 and
11). Evidence: `session2_diag/offline_kidnap_windows/loopthr_065/`.
