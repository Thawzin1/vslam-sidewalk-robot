Drive 10 (s2_static_10) camera map - CLEANED 3D point cloud, 2026-09-27 ~10:00-10:40 Hamilton, made on the Jetson.
Replaces nothing: ../map_3d/ (the first export, rtabmap-export) is kept as the "before".

WHAT WAS WRONG (the user's finding)
  At two corridor ends - region 1: x 13.5..16.5, y -2..2 (right end of the top corridor) and region 2: x -3..2,
  y -17.5..-12.5 (bottom-left junction) - the map showed fan-shaped wall points standing in the open walkway.

WHY (figure d10_corner_diagnosis.png; data nodes_diag.csv, end_wall_shortfall.csv)
  [VERIFIED] The wrong points come from few map pictures: 26-28 of the 860 make 95 % of them in each region.
  [VERIFIED] Those pictures were taken while DRIVING STRAIGHT AT THE END WALL from 3-7 m away, not while turning:
             most turned < 3 deg/s; rank correlation between a picture's share of wrong points and its turning
             rate is -0.11 over all 860 pictures; camera restarts (180 "Odometry automatically reset" lines in
             mapping.log) do not line up either (-0.14), and the worst region-1 pictures came before the first one.
             Dropping every picture taken while turning faster than 10 deg/s (133 pictures) changed nothing (row F).
  [VERIFIED] The end wall is placed TOO CLOSE, and by more the farther away the camera was. Wall position taken
             from close-range views (< 1.5 m); per picture facing the wall:
                 true distance   region 1 (white, blank wall)   region 2 (plain green wall)
                 1-3 m           0.04-0.06 m short              0.04-0.07 m short
                 3-4 m           0.55 m short (15 %)            0.10 m short
                 4-5 m           0.79 m short (18 %)            0.37 m short (8 %)
                 5-7 m           0.85 m short (14 %)            0.51 m short (8 %)
             Ordinary stereo noise (0.12 m baseline, 1258 px focal length, 0.5 px matching error: z^2 x 0.5 / (1258
             x 0.12)) would be 0.05 m at 4 m and 0.12 m at 6 m, so this is about 5-11 times larger. The same curve appears on two separate passes past each wall (pictures 177-201
             and 507-520; 354-378 and 1160-1180). A position (pose) error would differ between passes; a depth
             error repeats. So it is a DEPTH fault, not a map-position fault.
  [VERIFIED] Checks that were NOT the cause: depth jumps at object edges ("flying pixels") - wrong points sit on
             smooth depth like good ones (median local jump 14-26 mm); a noise filter removes almost none (row D).
  [INFERENCE] Mechanism: the camera's depth for a large, blank, evenly coloured wall has nothing to match between
             the left and right pictures, so the depth there is an estimate filled in from surroundings - smooth,
             plausible-looking, and 8-20 % too short at 3+ m (see the depth pictures of 194 and 362 in the figure).
  plain terms: far away, a plain wall "looks" nearer than it is to this camera; each frame from the approach puts
  the wall at a slightly different wrong place, and together they fan out across the corridor.

HOW IT WAS MEASURED ("wrong point" - two independent definitions)
  strip:      wall-height points (0.3-1.8 m above the floor) within 0.35 m of the robot's own corrected path - the
              robot drove through there, so nothing can stand there (people walking by could add a few).
  open floor: wall-height points in 10 cm cells that close-range views (< 1.5 m, where depth is good) saw as floor
              with no wall-height points; one cell eroded at the edges. Region 1 5.1 m2, region 2 7.2 m2.
  Also: area covered (10 cm cells with any point) and corridor-wall spread (5-95 % of y across the north and south
  walls of the top corridor, x 3-10 m; includes doors and objects, so only its change matters).

FILTERS TRIED - one change at a time against A (same detail as ../map_3d: every 4th pixel, 3 cm grid)
                                   wrong (strip)        wrong (open floor)   area   wall spread N/S
                                   whole  r1     r2      r1     r2           m2     cm
  A  before: depth to 6 m          124.0k 82.4k  23.6k   88.5k  64.8k        282    29.9 / 20.8
  B  depth to 4 m                   78.3k 56.7k  11.0k   61.8k  36.5k        239    28.0 / 20.0
  C  depth to 3 m                   35.3k 26.7k   2.9k   28.6k  10.5k        218    28.3 / 19.5
  D  6 m + noise filter 10 cm/30   118.8k 81.8k  22.6k   87.4k  63.0k        270    29.8 / 20.4
  F  6 m, turning pictures dropped 122.9k 82.4k  23.2k   88.5k  64.8k        264    29.7 / 19.9
  G  6 m, blank-wall depth dropped  59.0k 24.6k  23.3k   26.8k  64.3k        282    31.7 / 20.1
       beyond 2 m (texture < 6)
  H  C + G  (CHOSEN)                12.7k  5.6k   2.9k    4.7k  10.4k        217    29.5 / 19.1
  I  depth to 2.5 m                 16.0k 11.6k   1.1k   10.5k   3.8k        205    29.2 / 19.5
  Picture: d10_filter_comparison.png (both regions, every row). Numbers: metrics.json.
  H wins: whole-floor wrong points -90 %, region 1 -93 % (strip) / -95 % (open floor), region 2 -88 % / -84 %.
  Region 1 needs the blank-wall rule (the white wall is wrong already at 2-3 m measured range); region 2 needs the
  3 m limit (the green wall has enough camera noise to pass the texture test). Cost: 23 % less floor area covered
  (282 -> 217 m2) - what is lost is mostly far views into side corridors and rooms the robot never entered.
  "blank" = mean image gradient < 6 grey levels/pixel over a ~60 x 60 pixel window of the left camera picture
  (the camera's own noise floor is ~4.5-5; texture on the wrong points was ~5, on good walls 8-50).

THE CLEANED MAP (H at higher detail)
  drive10_map_3d_clean.ply  372 MB, 24,817,099 points, binary PLY, x y z + camera colour (no normals).
      every 2nd pixel, 1 cm grid (centre of the points in each cell), depth <= 3 m, and depth beyond 2 m dropped
      where the picture is blank. Of 421 M depth pixels within 6 m: 99.4 M dropped for > 3 m, 34.6 M for blank
      beyond 2 m (final_counts.json). Open in CloudCompare / MeshLab.
      Its own numbers: wall-height points on open floor 4.0 % (region 1) and 6.1 % (region 2) of the region's
      wall-height points, against 46.1 % and 29.2 % before; inside the robot's own strip, whole floor 0.77 %
      against 5.55 % before.
  d10_clean_topdown.png         same style as ../map_3d/d10_topdown.png (points below 2 m, height colours, path dashed).
  d10_clean_before_after.png    whole floor + both regions, before (A) and after (cleaned map).
  d10_corner_diagnosis.png      the diagnosis figure.
  Browser 3D viewer: pointcloud.html (project records) (Jetson site, needs the
      key), loads drive10_map_clean_light.ply (955,533 points, 14 MB) and on request drive10_map_clean_web.ply
      (2,866,601 points = this map on a 2.5 cm grid, 43 MB). Not copied to the website.

POSITIONS (rule 20)
  Every picture's depth is placed at the MAP'S CORRECTED positions: camera_corrected.tum = Admin.opt_poses, the
  optimisation RTAB-Map saved at the proper shutdown (identical to --opt 2 in ../map_3d). 860 pictures.

TOOL - NOT rtabmap-export
  The database (3.3 GB) could not be copied: that would have left < 3 GB free on the Jetson, and rtabmap-export
  writes to the database it opens. So the map was rebuilt with our own reader (scripts/): the original database
  opened READ-ONLY through sqlite (file:...?mode=ro), depth pictures decoded from RTAB-Map's "RVL" compression
  (scripts/rvl.c), the camera model and camera-to-robot mounting read from Data.calibration, each depth pixel
  projected and moved to the corrected position, then a voxel grid (keeps the centre of the points in each cell,
  like rtabmap-export --voxel).
  Check [VERIFIED]: with the ../map_3d settings (6 m, every 4th pixel, 3 cm) this rebuild matches
  ../map_3d/drive10_map_3d.ply to 3.5-3.8 mm median, 20 mm at the 99th percentile (nearest-neighbour both ways) -
  inside one 3 cm cell; 4,810,106 points against 4,517,220.

LIMITS
  - The "true" wall position comes from the same camera at close range (< 1.5 m), not from the LiDAR; close-range
    depth was 0.04-0.06 m short itself in the table above. Agreement with the LiDAR map was not checked here.
  - Both wrong-point counts depend on the voxel size, so compare shares, not counts, between the 3 cm and 1 cm maps.
  - Some wrong points remain (region 2 junction, ~6 % of its wall-height points): views at 2-3 m of the
    plain green wall. A 2.5 m limit would cut region 2 further (row I) but loses more area and helps region 1 less.
  - The same fault will affect every drive that approaches a blank wall head-on; the rule (3 m + blank-wall) is a
    map-building choice and does not change the map's positions or the loop closures.
