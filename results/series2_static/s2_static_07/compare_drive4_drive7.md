**In plain words: partly.** Drive 7 came home about as well as drive 4 - the camera map's corrected gap 0.17 m against 0.05 m agrees within the parking uncertainty, and the size difference against the LiDAR estimate is the same (1.078 against 1.063). But it did **not** repeat how drive 4 got there: the camera map made 30 loop closures against 181 (none joining the start), the camera froze 16 times against 1, the blend's own tracking ended 0.99 m out against 0.06 m, and the LiDAR map made 0 closures against 75, so drive 7 has no LiDAR yardstick. Drive 7 also went further down the second corridor, so the two drives are not the same route.

**Verdict (computed): of 20 judged rows, 9 agree within the combined uncertainty, 7 differ by 1-3 times it, and 4 differ by more than 3 times it: camera map loop closures (count) (10.4 times); LiDAR map loop closures (count) (8.7 times); camera freezes (tracker silent > 1.5 s) (3.6 times); start-to-end gap, map nodes tracking alone (= blend) (3.3 times).**

*Plain terms: two drives never give identical numbers. For each row, the spread one drive is expected to have is stated, with where it comes from; the two are combined as sqrt(s4^2 + s7^2). "Agrees within X" = the difference is smaller than that; "differs by N times" = it is N times larger (3 or more is a real difference). The spreads for distances are working figures, not a measured repeatability - N = 1 drive each. Drive 4 and drive 7 share the two corridors but not the whole route (drive 7 went about twice as far down the second one), so this is a like-for-like comparison only for the kinds of numbers that do not grow with route length.*

| row | drive 4 | drive 7 | spread per drive (4 / 7) | from | agrees? |
|---|---|---|---|---|---|
| distance driven - robot's own wheels + gyroscope / LiDAR map path | 90 m / 89 m | 109 m / 108 m | - | - | not judged: the route differs (drive 7 went further down the second corridor); both read about 6 % long (tape test) |
| distance driven - camera map corrected / wheels + gyroscope at 2 Hz (camera is the honest ruler, tape test) | 85.1 m / 89.0 m | 103.4 m / 107.8 m | - | - | not judged: route differs |
| start-to-end gap, camera tracking alone (0 closures) | 0.25 m | 0.60 m | 0.20 / 0.20 m | A | **differs** by 1.2 times the combined uncertainty (difference 0.35 m, combined 0.28 m) |
| start-to-end gap, map nodes tracking alone (= blend) | 0.06 m | 0.99 m | 0.20 / 0.20 m | A | **differs** by 3.3 times the combined uncertainty (difference 0.94 m, combined 0.28 m) |
| start-to-end gap, robot's own wheels + gyroscope, tracking alone | 0.27 m | 0.12 m | 0.20 / 0.20 m | A | **agrees** within 0.28 m (difference 0.14 m) |
| heading gap, robot's own wheels + gyroscope | +2.3 deg | +2.7 deg | 3.0 / 3.0 deg | B | **agrees** within 4.2 deg (difference 0.4 deg) |
| start-to-end gap, **camera map corrected** (floor) | 0.05 m | 0.17 m | 0.20 / 0.20 m | A | **agrees** within 0.28 m (difference 0.12 m) - but beside it: drive 4 181 closures (3 join the start), drive 7 30 (0 join the start); drive 7's gap is not propped up by an end-to-start closure, drive 4's partly is (rule 20) |
| camera map loop closures (count) | 181 | 30 | 13 / 5 | C | **differs** by 10.4 times the combined uncertainty (difference 151, combined 15) - drive 7's is below every earlier drive (320, 181, 72, 135) |
| start-to-end gap, **LiDAR map corrected** (floor) | 0.03 m | 0.16 m | 0.20 / 0.20 m | A | **agrees** within 0.28 m (difference 0.13 m) - drive 7's LiDAR map had 0 closures, so its gap is the uncorrected wheels + gyroscope |
| LiDAR map loop closures (count) | 75 | 0 | 9 / 0 | C | **differs** by 8.7 times the combined uncertainty (difference 75, combined 9) |
| LiDAR estimate passes its own check | yes | NO | - | - | not judged: a pass/fail check |
| same gaps in 3D: camera map | 0.05 m | 0.17 m | 0.20 / 0.20 m | A | **agrees** within 0.28 m (difference 0.12 m) |
| same gaps in 3D: LiDAR map | 0.05 m | 0.82 m | 0.20 / 0.20 m | A | **differs** by 2.7 times the combined uncertainty (difference 0.78 m, combined 0.28 m) |
| camera freezes (tracker silent > 1.5 s) | 1 | 16 | 1 / 4 | C | **differs** by 3.6 times the combined uncertainty (difference 15, combined 4) - drive 7 ran 1.3 times as long; per 10 min: 1.0 against 11.6 |
| camera tracking restarts from the blend | 26 | 9 | 5 / 3 | C | **differs** by 2.9 times the combined uncertainty (difference 17, combined 6) |
| camera tracking lost, seconds in total | 26.2 s | 14.9 s | 18.1 / 18.1 s | D | **agrees** within 25.6 s (difference 11.3 s) |
| camera program crashes | none | none (camera guard: 0 restarts) | - | - | not judged: same: none on either |
| **agreement with the LiDAR estimate**, camera map corrected: median (SE(3)) | 0.26 m | 0.45 m | 0.08 / 0.08 m | E | **differs** by 1.6 times the combined uncertainty (difference 0.19 m, combined 0.12 m) - drive 7's LiDAR estimate failed its own check (0 LiDAR closures), so its side is the camera against the robot's uncorrected wheels + gyroscope |
| **agreement with the LiDAR estimate**, camera map corrected: 95th percentile (SE(3)) | 0.49 m | 0.88 m | 0.27 / 0.27 m | E | **differs** by 1.0 times the combined uncertainty (difference 0.39 m, combined 0.38 m) - drive 7's LiDAR estimate failed its own check (0 LiDAR closures), so its side is the camera against the robot's uncorrected wheels + gyroscope |
| agreement, map nodes (= blend) tracking alone: median | 0.12 m | 0.38 m | 0.08 / 0.08 m | F | **differs** by 2.3 times the combined uncertainty (difference 0.26 m, combined 0.12 m) - drive 7's LiDAR estimate failed its own check (0 LiDAR closures), so its side is the camera against the robot's uncorrected wheels + gyroscope |
| agreement, map nodes (= blend) tracking alone: 95th percentile | 0.24 m | 0.75 m | 0.27 / 0.27 m | F | **differs** by 1.3 times the combined uncertainty (difference 0.51 m, combined 0.38 m) - drive 7's LiDAR estimate failed its own check (0 LiDAR closures), so its side is the camera against the robot's uncorrected wheels + gyroscope |
| for information, size fitted: camera map corrected median | 0.09 m | 0.16 m | 0.08 / 0.08 m | G | **agrees** within 0.12 m (difference 0.07 m) - drive 7's LiDAR estimate failed its own check (0 LiDAR closures), so its side is the camera against the robot's uncorrected wheels + gyroscope |
| for information, size fitted: camera map corrected 95th percentile | 0.19 m | 0.36 m | 0.27 / 0.27 m | G | **agrees** within 0.38 m (difference 0.17 m) - drive 7's LiDAR estimate failed its own check (0 LiDAR closures), so its side is the camera against the robot's uncorrected wheels + gyroscope |
| best-fit size factor, camera map corrected vs LiDAR estimate (not applied) | 1.063 | 1.078 | 0.015 / 0.015 | H | **agrees** within 0.021 (difference 0.015) |

Spread sources (column "from"):
- **A** = where the robot parked: s2_scale_01 found stops at one mark up to 0.19 m apart even with a wheel stop (its RESULTS.md, uncertainty), and drives 4 and 7 parked on the start mark by eye
- **B** = parking heading by eye, working figure (not measured)
- **C** = counting spread, the square root of each count (Poisson, the least spread a count of chance events has)
- **D** = drive-to-drive spread (sample standard deviation) of drives 3-6
- **E** = drive-to-drive spread (sample standard deviation) of the camera map's agreement on drives 3, 4, 5, the drives whose LiDAR estimate passed its own check; different routes, so it may overstate the spread for a repeat
- **F** = drive-to-drive spread (sample standard deviation) of the camera map's agreement on drives 3, 4, 5, the drives whose LiDAR estimate passed its own check; different routes, so it may overstate the spread for a repeat (borrowed from the camera map's rows)
- **G** = drive-to-drive spread (sample standard deviation) of the camera map's agreement on drives 3, 4, 5, the drives whose LiDAR estimate passed its own check; different routes, so it may overstate the spread for a repeat (borrowed)
- **H** = drive-to-drive spread (sample standard deviation) of drives 3, 4, 5 (valid LiDAR estimates)

Caveats on the spreads: (1) the 0.20 m for tracking-alone gaps is only the measurement floor (where the robot parked); how much tracking drift varies from drive to drive is not known and is surely larger, so "differs" on those rows is weaker than it reads. (2) The blend's agreement rows borrow the camera map's spread; the blend's own values on drives 3-5 (1.84 / 0.12 / 0.12 m median) vary far more because of drive 3's link drop, so their "differs" verdicts rest on that borrowing.
