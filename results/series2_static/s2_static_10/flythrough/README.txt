Drive 10 (s2_static_10, the whole 2nd floor, 27 Sept 2026) - 3D "robot driving through the map" videos.
Made on the Jetson, 27 Sept 2026 ~05:10 Hamilton, by flythrough.py in this folder (run under nice -n 19, 6 worker
processes; log: render.log). Not committed.

WHAT THE VIDEOS SHOW
  drive10_camera_view.mp4  first-person: the view from the robot's camera height, looking along the robot's heading.
  drive10_chase_view.mp4   the same route seen from behind and above the robot: the robot is a yellow box
                           (Husky-sized, 0.99 x 0.67 x 0.39 m, red face = front), the path driven so far is a cyan line.
  Both: top-right inset = top-down mini-map (the map's walls in grey, whole route thin grey, route so far cyan,
  yellow dot + yellow tick = robot and its heading). Bottom caption: drive name + speed, then drive time, distance so far.
  drive10_*_still_NNNN.png  single frames for slides (NNNN = video frame number, 30 per second).

INPUTS (Jetson, this run folder)
  ../map_3d/drive10_map_3d.ply   4,517,220 points with camera colours, 3 cm voxel grid, made by rtabmap-export --opt 2,
                                 i.e. placed at the map's CORRECTED positions (Admin.opt_poses, saved by RTAB-Map at shutdown).
  ../camera_corrected.tum        the same corrected positions (Admin.opt_poses; 860 graph nodes, 28 min 04 s,
                                 250.1 m along the node-to-node path). camera.tum (tracking alone) was NOT used, so the
                                 path and the cloud come from one and the same set of positions (ENGINEERING_NOTES.md rule 20:
                                 these are the map's corrected positions, not the camera's raw tracking).

METHOD (plain terms: our own small renderer - each map point is drawn as a small square on the screen, nearer points
cover farther ones, and the frames are joined into a video by ffmpeg, a standard video-making program)
  - Perspective projection of every map point within 28-32 m in front of the viewer; each point is a square whose size
    matches a 3 cm point at that distance (1-9 pixels); a depth buffer keeps the nearest point per pixel.
  - Fog: points fade into the dark background from 6 m (first-person) / 12 m (chase) out to the far limit.
  - Slight height shading (points near the floor 20 % darker) so walls read as 3D.
  - CEILING CROPPED: points higher than 2.35 m above the floor are removed (ceiling, lights), so the view is not
    blocked by a lid; 3,725,679 of 4,517,220 points kept. Floor height taken from the cloud: -0.17 m in the map frame
    (the most common height of the lowest band of points). The empty dark area above the walls is this crop, not
    missing data.
  - Path: the corrected node positions, interpolated to a 0.1 s grid and smoothed (position: Gaussian, 1 s; heading:
    Gaussian, 3 s, from the nodes' own orientation) so the view does not jitter. Smoothing rounds corners slightly
    (tens of centimetres); the cyan trail and mini-map use the unsmoothed corrected positions.
  - First-person eye: 0.55 m above the floor, at the robot's reference point (the camera itself sits a little ahead of
    it on the robot), tilted 4 deg down, 90 deg wide view. Chase eye: 4.5 m behind, 4.2 m above the floor, looking
    down 40 deg, 75 deg wide view (higher and steeper than a 3 m / 2.5 m / 25 deg view so the corridor walls do not
    hide the robot).
  - 1280 x 720, 30 frames per second, H.264 (libx264, crf 20, yuv420p, faststart).

SPEED-UP (x15, stops shortened)
  - While the robot moves: 15 s of real drive time = 1 s of video.
  - The robot also stood still a lot (start stand, the planned pauses, turns in place). 13 stops longer than 8 s
    (node gaps > 8 s with < 0.35 m of movement; 617.8 s of standing in total, including the 143.8 s stand at the start)
    are each shown as only 8 real seconds (about half a second of video). Shown real time: 1170.4 s of 1684.2 s.
  - Result: 2,340 frames = 78.0 s per video. The drive-time clock in the caption is the real time, so it jumps
    forward during a shortened stop.
  So "x15 speed" is exact while moving; over the whole drive the average is about x21.6.

HONESTY NOTES
  - This is the camera's map (ZED X + RTAB-Map), not a LiDAR scan and not ground truth (ENGINEERING_NOTES.md rule 19).
  - Everything is shown at the map's corrected positions after 199 loop closures; the camera's raw tracking would look
    different (see RESULTS.md for both kinds of start-to-end gap).
  - Points the camera saw up to 6 m to the side (doorways, side corridors) appear as they are in the map.

FILES AND RESULTS
  drive10_camera_view.mp4        78.0 s, 2,340 frames, 128.9 MB (13.2 Mbit/s: point clouds compress poorly)
  drive10_camera_view_small.mp4  the same video re-encoded at crf 28, 50.8 MB, for when file size matters
  drive10_chase_view.mp4         78.0 s, 2,340 frames, 53.3 MB
  drive10_camera_still_{0150,0560,1300,2000}.png, drive10_chase_still_{0150,0700,1300,2000}.png (1280 x 720)
  Render time: 695 s (first-person) + 717 s (chase) with 6 processes under nice -n 19; ~1.0-2.3 s per frame per process.
  In the chase view, a cyan line AHEAD of the robot is an earlier pass along the same corridor (the route drives some
  corridors twice), not a path it has yet to drive.
