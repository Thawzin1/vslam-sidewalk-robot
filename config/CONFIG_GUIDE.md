# Configuration guide

A plain-language guide to every configuration file the September 2026 workflows used: mapping drives,
localisation, the SLAM session, navigation, and the LiDAR reference replay.

## What this folder is

`config/` is a **documented mirror**. The copies that `roslaunch` (the ROS program that starts a set of
programs from an XML launch file) actually reads live inside the ROS packages under `catkin_ws/src/` and
under `run/lib/`. The paths in each section below point to those copies. `config/collect_configs.sh`
refreshes the mirror from them, so **edit the package copy, then re-run the collector**. Never edit only the mirror.

Words used throughout:

- **RTAB-Map** (the mapping program: it builds a map from camera pictures and works out where the robot is on it).
  Version 0.21.13 on this project's Jetson (the NVIDIA computer mounted on the robot).
- **node** (one saved place in the map: a picture, its depth, and the robot's position when it was taken).
- **loop closure** (recognising a place seen before and correcting the map so the two visits line up).
- **odometry** (the running estimate of how far and which way the robot has moved since the start; it drifts).
- **occupancy grid** (a flat map made of 5 cm squares, each marked free, occupied, or never seen).
- **base_link** (the robot's own reference point: centre of the body, **0.13228 m above the floor**, not at floor level).
- **TF** (ROS's list of how each coordinate frame sits relative to the others, such as `map -> odom -> base_link`).
- **default** (the value a program uses when nobody sets one). "Build default" means the value `rtabmap --params`
  prints on this exact build.

How to check a value really took effect: a misspelt name is silently ignored by RTAB-Map, robot_localization
and move_base alike. Read it back from the running system, e.g. `rosparam get /rtabmap/rtabmap/Grid/RangeMax`.

"Project rules section N" refers to the original project's rules file (ENGINEERING_NOTES.md, not included in this
repository); the facts needed from it are repeated in the rows that cite it. docs/SOLVED.md and
docs/DO_NOT_REPEAT.md hold the measured history behind many values.

Marks used below: **[not verified]** means the meaning comes from general knowledge of the package, not from
this project's files or `rtabmap --params`. Treat it as a hint to check, not a fact.

### Which workflow reads which file

| workflow | files |
|---|---|
| Mapping drive (new map) | `rtabmap_zedx.yaml`, `zedx_front.yaml`, `ekf_fused.yaml` (with the blend on), `rtabmap_common.launch`, `rtabmap_mapping.launch`, `record_mapping_run.launch`, `fused_odometry.launch`, `robot_frames.yaml` (checks only) |
| Localisation (find the robot on an old map) | the mapping set plus `rtabmap_localization.yaml` and `localise_run.launch` |
| SLAM session (keep adding to an old map) | the mapping set plus `slam_run.launch`, `zedx_front_od.yaml` (people detector on) or `zedx_front_od_replay.yaml` (replaying a recording), `obstacles.yaml` (live obstacles) |
| Navigation (robot drives itself to goal points) | the localisation set plus all `sidewalk_navigation/config/*.yaml` |
| LiDAR reference replay | `robot_replay_config.yaml` (plus the colleague's own launch files on the robot, not documented here) |

---

## 1. rtabmap_zedx.yaml - the main RTAB-Map settings

**Path:** `catkin_ws/src/sidewalk_slam/config/rtabmap_zedx.yaml`
**Used by:** every RTAB-Map workflow (mapping, localisation, SLAM session). Loaded by `rtabmap_common.launch`
into both the odometry program (`rgbd_odometry`) and the map program (`rtabmap`).
**Format note:** every value is written as a quoted string on purpose; that is how RTAB-Map's own reader wants it.

The file's own comments are long and carry the history of each value (which run showed what). Read them before
changing anything. The "default" column is from `rtabmap --params` on this build.

### 1.1 Settings that differ from the build default

| setting | value (default) | what it controls (plain words) |
|---|---|---|
| `Reg/Force3DoF` | true (false) | Treat the robot as moving on a flat floor: only x, y and heading (yaw) are estimated; height, roll and pitch are held at zero. Right for indoor floors; wrong outdoors on kerb ramps and sloped pavement. The launch argument `force_3dof` overrides this value (section 12). Replaces the removed name `Optimizer/Slam2D`. |
| `Optimizer/Strategy` | 1 (**2**) | Which maths library straightens the map after a loop closure: 1 = g2o. **The build default is 2 (GTSAM)**, so this line is a real change, not a restatement. Must be pinned: defaults differ between builds. |
| `RGBD/LinearUpdate` | 0.05 (0.1) | Add a new node only after the robot has moved at least 5 cm. Stops a parked robot filling the map with copies of one view. |
| `RGBD/AngularUpdate` | 0.05 (0.1) | Same, for turning: at least 0.05 rad (about 3 degrees). |
| `Mem/STMSize` | 30 (10) | Short-term memory (the most recent nodes, which are kept out of loop-closure matching). Stops the map "recognising" the place it stood in two seconds ago. |
| `Vis/CorNNDR` | 0.7 (0.8) | Nearest-neighbour distance ratio (a picture-point match is accepted only if the best candidate is clearly better than the second best). Tightened because many ambiguous matches were producing 0 usable matches out of 64. |
| `Vis/MinDepth` | 1.0 (0 = no limit) | Ignore picture points closer than 1 m: the ZED X gives no valid depth nearer than that. |
| `Vis/MaxDepth` | 15.0 (0 = no limit) | Ignore picture points further than 15 m, where stereo depth is mostly guesswork. |
| `Odom/ResetCountdown` | 10 (0 = never) | After this many consecutive frames where odometry cannot work out the motion, restart odometry from the last good position. Was 1, which caused 147 restarts in one run. **A restart hides the distance travelled during the outage** (project rules section 2.4): always report resets beside any accuracy number. |
| `OdomF2M/MaxSize` | 1000 (2000) | Size of odometry's own small local map (how many picture features it remembers to track against). |
| `Odom/ImageDecimation` | 4 (1) | Shrink each picture to a quarter of its width and height (1920x1200 to 480x300) before odometry works on it. A computing-time decision: at full size odometry could not keep up. |
| `Odom/Holonomic` | false (true) | Tell odometry the Husky cannot move sideways, so sideways motion is derived from forward motion and heading instead of estimated. |
| `Grid/RangeMin` | 1.0 (0.0) | Do not use depth points nearer than 1 m for the occupancy grid (the camera's blind zone). |
| `Grid/RangeMax` | 8.0 (5.0) | Do not use depth points further than 8 m for the grid; beyond that the depth error spans several cells. **Must stay pinned** together with `Grid/Sensor` (project rules section 2.5: RTAB-Map rewrites both silently if a laser scan is subscribed). |
| `Grid/MaxGroundAngle` | 30 (45) | Official description: the largest angle between a point's surface direction and the ground's that still counts as ground. Steeper surfaces become obstacles. |
| `Grid/MinGroundHeight` | -0.20 (0.0 = off) | Lower edge of the height band, **measured from base_link, not the floor**: -0.20 is 68 mm below the floor. Points below it are **deleted**, not marked. Consequence: the road at the bottom of a 150 mm kerb drop is thrown away. The file explains why this was deliberately not changed (one variable per experiment). **Typing 0.0 switches the limit off; it does not put it at base_link.** |
| `Grid/MaxGroundHeight` | 0.10 (0.0 = off) | Upper edge of what may count as ground, from base_link (0.232 m above the floor). |
| `Grid/MaxObstacleHeight` | 1.60 (0.0 = off) | Ignore anything higher than this above base_link (1.732 m above the floor), so door frames the robot passes under are not mapped as walls. |
| `Grid/NoiseFilteringRadius` | 0.09 (0.0 = off) | Drop lone depth points: a point with fewer than `Grid/NoiseFilteringMinNeighbors` (5) neighbours within 9 cm is removed. Was 0.05, equal to the cell size, which deleted flat surfaces (a point on a 5 cm grid has only 4 neighbours at exactly 5 cm). **If `Grid/CellSize` changes, this must change with it** (docs/SOLVED.md). |
| `Grid/RayTracing` | true (false) | Mark every cell between the camera and each seen surface as free, so a person who walked past is erased once the floor behind them is seen. |
| `GridGlobal/MinSize` | 20 (0.0) | Reserve a 20 m map from the start so the grid is not resized constantly as it grows. |
| `RGBD/ProximityMaxGraphDepth` | 120 (50) | Proximity detection (checking whether the robot is physically near an already-mapped place even when it looks different) only considers nodes at most 120 links back along the map. Caps how much drift a correction has to absorb. |
| `RGBD/LocalRadius` | 5.0 (10) | Radius in metres of the "local map" used by proximity detection. |
| `RGBD/NeighborLinkRefining` | true (false) | When a node is added, re-check its link to the previous node with picture matching, correcting small odometry errors. |
| `DbSqlite3/JournalMode` | 1 (3) | How the map database (one SQLite file) protects half-finished writes: 1 = TRUNCATE, the undo record is kept in a file beside the database, so it survives the program crashing. The default 3 keeps it in memory, which a crash loses. Added after two damaged databases. |
| `Mem/SaveDepth16Format` | true (false) | Store depth as 16-bit whole millimetres instead of 32-bit decimals. Cuts each node from about 6.3 MB to about 2 MB, which shortened the database saves that were freezing the camera. Cost: depths over about 65 m are dropped, and databases made this way replay slightly differently from older ones. |

*Plain terms for the height band:* picture a slab of air from 68 mm under the floor to 1.73 m above it. Only depth
points inside the slab exist for the grid; everything outside is thrown away before it is sorted into floor and obstacle.

### 1.2 Settings at the build default but pinned on purpose

The project rule is: pin anything whose default could differ between builds, and anything a comment warns about.

| setting | value | what it controls (plain words) |
|---|---|---|
| `Reg/Strategy` | 0 | How two nodes are lined up: 0 = by picture features only. 1 would be LiDAR (laser scanner) shape matching. The camera map uses no LiDAR. |
| `Optimizer/Iterations` | 20 | How many rounds the map-straightening maths runs per update. |
| `RGBD/OptimizeMaxError` | 3.0 | Safety check on loop closures: after straightening, if any link had to stretch more than 3 times its expected error, the closure is rejected. **A ratio, not a distance.** Cannot be used together with `Optimizer/Robust` (project rules section 4.7). The long comment records the simulator experiments that led back to 3.0. |
| `Optimizer/Robust` | false | Vertigo (a method that lets the maths switch off closures that disagree with the rest) is off. Tried in simulation: 2,111 closures accepted and the map got 18 times worse. |
| `RGBD/OptimizeFromGraphEnd` | false | Anchor the map to the first node, so the map does not jump every time it is straightened. |
| `Optimizer/GravitySigma` | 0.3 | How firmly the map is kept level using the direction of gravity from the motion sensor (smaller = firmer). |
| `Rtabmap/DetectionRate` | 1.0 | Run the expensive map update once per second. |
| `Rtabmap/TimeThr` | 0 | No time limit per map update (0 = unlimited). The comment suggests about 700 ms for long live runs. |
| `Rtabmap/MemoryThr` | 0 | No limit on how many nodes stay in working memory (the part of the map searched for loop closures). |
| `Rtabmap/LoopThr` | 0.11 | Recognition score (0 to 1) a picture must reach before RTAB-Map tries to join it to an old place. Lower = more attempts, more wrong ones. Overridden to 0.08 by the localisation and SLAM-session launch files (section 12). |
| `Mem/BinDataKept` | true | Keep the raw pictures and depth in the database. This is what lets a map be re-exported or re-processed later, and why an hour of mapping takes gigabytes. |
| `Mem/RehearsalSimilarity` | 0.6 | How alike two consecutive nodes must be to be merged into one [meaning of "rehearsal" not verified beyond the official one-word description]. |
| `Kp/DetectorStrategy` | 8 | Which feature finder (the method that picks distinctive corners in a picture) is used for place recognition: 8 = GFTT corners with ORB descriptions. **Usable on this build: 2, 7, 8, 9, 10 only**; others fall back to 8 silently with only a warning (project rules section 2.6). Grep logs for "cannot be used". |
| `Kp/MaxFeatures` | 500 | At most 500 features per picture for place recognition. |
| `Vis/FeatureType` | 8 | Same choice as above, for lining up two nodes and for odometry. |
| `Vis/MaxFeatures` | 1000 | At most 1000 features per picture for lining up and odometry. |
| `Vis/MinInliers` | 20 | At least 20 picture points must agree in 3D before a match (loop closure or odometry step) is accepted. Raising it makes closures rarer and safer. |
| `Vis/EstimationType` | 1 | How motion is computed from matches: 1 = PnP (3D points in one picture against 2D points in the other). Needs good depth on only one side. |
| `Vis/InlierDistance` | 0.1 | Only used by the 3D-to-3D method (type 0), so it has no effect with type 1. |
| `Vis/CorGuessWinSize` | 40 | When a motion guess exists, search for each match within 40 pixels of where the guess predicts it. |
| `Vis/PnPVarianceMedianRatio` | 4 | How the confidence of a PnP result is computed; 4 or more is recommended for stereo depth to play down far, noisy points. |
| `Rtabmap/ImagesAlreadyRectified` | true | The pictures arrive already corrected for lens distortion, so RTAB-Map must not correct them again. |
| `Stereo/OpticalFlow` | true | Only used in stereo mode (two pictures instead of picture plus depth): match left and right by tracking motion. Inert in the drives, which use picture plus depth. |
| `Stereo/MaxDisparity` | 128.0 | Stereo mode only: the largest left-right shift (in pixels) searched. |
| `Odom/Strategy` | 0 | Odometry method: 0 = frame-to-map (each new picture is matched against a small local map, not just the previous picture). Only 0 and 1 exist on this build. |
| `Odom/GuessMotion` | true | Use the last measured motion as the starting guess for the next one. |
| `Grid/Sensor` | 1 | Build the occupancy grid from depth pictures (0 would be a laser scan). **Must stay pinned** with `Grid/RangeMax` (project rules section 2.5). Replaces the removed name `Grid/FromDepth`. |
| `Grid/3D` | true | Also keep a 3D version of the map, not only the flat one. |
| `Grid/CellSize` | 0.05 | Map squares are 5 cm. Halving it would multiply memory by four. `Grid/NoiseFilteringRadius` depends on it. |
| `Grid/DepthDecimation` | 4 | Use every 4th depth pixel when building the grid; the single biggest speed setting for the grid. |
| `Grid/NormalsSegmentation` | true | Tell floor from obstacle by which way each small surface faces, not by height alone, so a ramp is not called a wall. |
| `Grid/NoiseFilteringMinNeighbors` | 5 | See `Grid/NoiseFilteringRadius` above. |
| `Grid/ClusterRadius` | 0.10 | Largest gap in metres between points that are grouped into one obstacle. |
| `GridGlobal/OccupancyThr` | 0.5 | A square counts as occupied once the evidence for it passes 50 %. |
| `GridGlobal/Eroded` | false | Do not shrink obstacle areas in the global map. |
| `RGBD/ProximityBySpace` | true | Proximity detection is on (see `RGBD/ProximityMaxGraphDepth`). |
| `RGBD/ProximityByTime` | false | Do not also check the most recent nodes for proximity. |
| `DbSqlite3/Synchronous` | 0 | Whether the database waits for the disk to confirm each save: 0 = does not wait. Was 1 (waits) until drive 4, when a 4.45 s confirmed save froze the camera. JournalMode 1 still protects against the program crashing; a power cut mid-save can still damage the file, so **the integrity check after every drive stays**. To undo: set back to 1. |

### 1.3 Build settings that matter but are NOT in the file

| setting | build value | why it matters |
|---|---|---|
| `Grid/MapFrameProjection` | false | This is what makes base_link, not the floor, the zero of the height band above. |
| `Grid/GroundIsObstacle` | false | Keeps the floor-by-surface-direction method in use. |
| `Grid/PreVoxelFiltering` | true | Points are snapped to the 5 cm grid before filtering, which is why the noise-filter radius must exceed 5 cm. |
| `Vis/DepthAsMask`, `Mem/DepthAsMask` | true | Features are taken only where depth exists. The people mask (section 3) relies on this: blank a person's depth and RTAB-Map ignores the person (project rules section 5). |
| `RGBD/AggressiveLoopThr` | 0.05 | Lower recognition score used before a new session has joined an old map (section 12). |

---

## 2. rtabmap_localization.yaml - turns the mapper into a localiser

**Path:** `catkin_ws/src/sidewalk_slam/config/rtabmap_localization.yaml`
**Used by:** localisation (and navigation, which runs on localisation). Loaded **after** `rtabmap_zedx.yaml`
when the launch argument `localization:=true`, so it only lists differences.

| setting | value (default) | what it controls (plain words) |
|---|---|---|
| `Mem/IncrementalMemory` | false (true) | **The switch.** false = localisation mode: the map is read, nothing is added. **Warning:** `rtabmap_common.launch` sets this again afterwards and its test is broken, so it always ends up true (mapping). `localise_run.launch` sets it back to false (section 12; docs/SOLVED.md). |
| `Mem/InitWMWithAllNodes` | true (false) | Load every saved node into working memory at start, so the robot can recognise any place straight away, at the cost of startup time and memory. |
| `RGBD/StartAtOrigin` | false (false) | Official meaning: false = assume the robot restarts where the previous session stopped; true = assume it starts at the map origin. (The file's comment describes this as "do not start a new map", which does not match the official description.) |
| `Rtabmap/StartNewMapOnLoopClosure` | false (false) | Do not start a second map automatically. |
| `Rtabmap/PublishRAMUsage` | true (false) | Official meaning: include memory use in RTAB-Map's statistics. (The file's comment says it publishes the pose early; that does not match the official description [not verified].) |
| `Rtabmap/LoopThr` | 0.15 (0.11) | Stricter recognition score. **Overridden to 0.08 by `localise_run.launch`**: at 0.15, correct places were never accepted (docs/DO_NOT_REPEAT.md). |
| `Vis/MinInliers` | 25 (20) | Stricter 3D agreement. **Overridden to 20 by `localise_run.launch`**: the correct start place reached 20-21 matches and was refused at 25. |
| `Mem/BinDataKept` | false (true) | Do not keep raw pictures for new data, since nothing is written back to the map. |

---

## 3. zedx_front.yaml - ZED X camera driver settings

**Path:** `catkin_ws/src/sidewalk_perception/config/zedx_front.yaml`
**Used by:** every live workflow, through `zedx_front.launch` (the camera's launch file). Read by zed-ros-wrapper
(Stereolabs' ROS driver for the camera, ZED SDK 4.x). Several values are **replaced by launch arguments** at start:
the drive scripts start the camera with `depth_mode:=NEURAL` and `pos_tracking_mode:=GEN_2`, so what the drives ran
is NEURAL depth even though this file says ULTRA. Always read back with
`rosparam get /zedx_front/zed_node/<section>/<name>`.

| setting | value | what it controls (plain words) |
|---|---|---|
| `general/camera_name` | zedx_front | Prefix of every topic and frame name. A second camera must use a different name. |
| `general/camera_model` | zedx | Which camera model the driver expects. |
| `general/serial_number` | 0 | 0 = use whichever single camera is connected. Set the real serial number if a second camera is fitted. |
| `general/grab_resolution` | HD1200 | Capture at 1920x1200 per eye (the best depth precision). A text key; an old numeric setting was silently ignored. |
| `general/pub_resolution` | NATIVE | Publish at the capture size, with no halving. |
| `general/grab_frame_rate` | 30 | Sensor capture rate, 30 pictures per second. |
| `general/pub_frame_rate` | 15 | Publishing rate. **This also caps the driver's capture loop and the SVO recorder (the camera's own recording format) at 15 Hz** (project rules section 1; measured 2026-09-13). |
| `general/gpu_id` | -1 | Let the driver choose the graphics processor. |
| `general/base_frame` | base_link | The robot body frame the camera is attached to. |
| `general/sdk_verbose` | 1 | Print the camera's diagnostic messages. Capture them to a log; that is how crashes get diagnosed. |
| `general/svo_compression` | 0 | Recording format: 0 = truly lossless (large, compressed on the processor). The drive scripts pass their own value (default 2, lossy) because mode 0 does not fit on the disk for a 30-minute drive. Read only when the camera starts. |
| `depth/depth_mode` | ULTRA | Depth method. **Drives override this to NEURAL** (a neural-network depth method, more accurate on plain surfaces, more graphics load). The file's "~80 % of the GPU" comment is outdated: measured about 55 % mean (project rules section 5). |
| `depth/sensing_mode` | 0 | STANDARD: do not fill holes in the depth picture with invented values. |
| `depth/depth_stabilization` | 1 | Smooth depth over time (uses the camera's own tracking). |
| `depth/min_depth` | 1.0 | Nearest depth reported, in metres. Below 1 m the two lenses barely overlap. |
| `depth/max_depth` | 35.0 | Furthest depth reported (the camera's rated range). |
| `depth/openni_depth_mode` | false | Publish depth as decimal metres, not whole millimetres. |
| (not set) `depth_confidence`, `depth_texture_conf` | driver defaults 50, 100 | How sure the camera must be before a depth pixel is kept. **Known gap**: not pinned in the file; verified live at 50 and 100. They sit at the node's top level, not under `depth/`. |
| `video/auto_exposure_gain` | true | Automatic exposure and gain. |
| `video/exposure`, `video/gain` | 80, 80 | Percent values used only if automatic exposure is turned off. |
| `video/auto_whitebalance` | true | Automatic colour balance. |
| `video/brightness`, `contrast`, `saturation`, `sharpness` | 4 each | Picture adjustments. |
| `sensors/sensors_timestamp_sync` | false | Stamp motion-sensor readings with the camera's clock. |
| `sensors/max_pub_rate` | 400.0 | Ceiling on the motion-sensor publishing rate. The sensor itself runs at 200 Hz (datasheet), so this ceiling has no effect. |
| `sensors/publish_imu_tf` | true | Publish where the camera's motion sensor (IMU, inertial measurement unit: gyroscope plus accelerometer) sits. |
| `pos_tracking/pos_tracking_enabled` | true | The camera's own position tracker runs, as a comparison and because depth stabilisation needs it. Nothing downstream should treat it as the official position. |
| `pos_tracking/publish_tf` | false | The camera tracker must not publish `odom -> base_link`: only one program may (two publishers made the robot flicker between answers). |
| `pos_tracking/publish_map_tf` | false | Same for `map -> odom`: RTAB-Map owns it. |
| `pos_tracking/area_memory` | false | The camera tracker's own place recognition is off (fair comparison). |
| `pos_tracking/imu_fusion` | true | The camera tracker uses its motion sensor. |
| `pos_tracking/floor_alignment` | false | Do not level the tracker's frame on the floor at start. |
| `pos_tracking/two_d_mode` | false | Camera tracker estimates all six motions (sidewalks are not flat). |
| `point_cloud/point_cloud_freq` | 10.0 | Rate of the 3D point output. RTAB-Map does not use it; it builds its own from depth. |
| `object_detection/od_enabled` | false | The camera's object detector is off in this file (see section 4 for the version that turns it on). |

---

## 4. zedx_front_od.yaml and zedx_front_od_replay.yaml - camera with the people detector on

**Paths:** `run/lib/slam_session/zedx_front_od.yaml` and `run/lib/slam_session/zedx_front_od_replay.yaml`
**Used by:** the SLAM session with the people mask on (`mask:=true` in `slam_run.launch`). The start script
refuses to run if the main `zedx_front.yaml` has changed since these copies were made (it checks a checksum).
**Everything above the object-detection section is identical to `zedx_front.yaml`** (section 3).

| setting | value | what it controls (plain words) |
|---|---|---|
| `object_detection/od_enabled` | true | Start the camera's object detector with the camera. The model must be prepared once first (`od_bench.sh`). |
| `object_detection/model` | MULTI_CLASS_BOX_FAST | The fast detector model: the only one whose file exists on this Jetson. |
| `object_detection/confidence_threshold` | 50 | Report only boxes the detector is at least 50 % sure of. |
| `object_detection/max_range` | 15.0 | Ignore people further than 15 m. |
| `object_detection/allow_reduced_precision_inference` | true | Let the detector use faster, lower-precision arithmetic [effect on accuracy not verified]. |
| `object_detection/prediction_timeout` | 0.5 | A briefly hidden person keeps their box for 0.5 s. |
| `object_detection/object_tracking_enabled` | true | The same person keeps one identity number from picture to picture. |
| `object_detection/mc_people` | true | Detect people. |
| `mc_vehicle`, `mc_bag`, `mc_animal`, `mc_electronics`, `mc_fruit_vegetable`, `mc_sport` | false | All other object classes off. |
| `general/svo_realtime` | false | **Replay copy only.** When replaying a recording, process every recorded frame instead of pacing to the wall clock, so two replays of one recording see the same frames. |

*How the mask works:* `depth_person_mask.py` writes zero depth into each person's box and republishes the depth
picture; because RTAB-Map takes features only where depth exists, people drop out of the map. The navigation
obstacles use the **unmasked** depth, so people still count as obstacles.

---

## 5. ekf_fused.yaml - the blend of wheels, gyroscopes and camera

**Path:** `catkin_ws/src/sidewalk_slam/config/ekf_fused.yaml`
**Used by:** every drive from drive 3 on with `FUSION=1`, started by `fused_odometry.launch` (section 13).
The program is robot_localization's `ekf_localization_node` 2.7.7. An EKF (extended Kalman filter: a program that
keeps one best guess of the robot's position and blends each new reading into it, weighted by how much it trusts
that sensor). It becomes the **only** publisher of `odom -> base_link`. robot_localization does not warn about
misspelt names; the file says every name was checked against the installed library.

Inputs, all cleaned first by `ekf_input_conditioner.py`:

| input | from | fused | rate |
|---|---|---|---|
| `odom0` = `/ekf_in/wheel_odom` | the robot's wheels, relayed | forward speed, sideways speed (always 0), turn rate | about 10 Hz |
| `imu0` = `/ekf_in/imu` | the robot's gyroscope (UM7), relayed | turn rate only | 19.5 Hz |
| `imu1` = `/ekf_in/zed_imu` | the camera's own gyroscope, on the Jetson | turn rate only | about 50 Hz |
| `odom1` = `/ekf_in/vo_odom` | RTAB-Map's camera odometry | forward speed, sideways speed, turn rate | about 14 Hz |

Positions are never fused, only speeds: a position is a running total whose error only grows.

| setting | value | what it controls (plain words) |
|---|---|---|
| `frequency` | 50 | Publish a position 50 times a second. |
| `sensor_timeout` | 0.5 | If **no** input at all arrives for 0.5 s, keep predicting on its own. There is no per-sensor timeout in this version. 0.5 rather than 0.1 because camera readings arrive 0.18-0.25 s late. |
| `predict_to_current_time` | false | Publish the position at the time of the newest reading, not guessed forward to "now". |
| `smooth_lagged_data` | true | A late reading is put back in its proper place in time and everything after it is recomputed. |
| `history_length` | 1.0 | How far back (seconds) a late reading may be put. Older ones are applied as if current. |
| `two_d_mode` | true | Height, roll and pitch held at zero; matches `Reg/Force3DoF`. |
| `map_frame`, `odom_frame`, `base_link_frame` | map, odom, base_link | Names of the three frames. |
| `world_frame` | odom | The filter works in `odom` and publishes `odom -> base_link`; RTAB-Map publishes `map -> odom`. |
| `publish_tf` | true | Publish the `odom -> base_link` link. |
| `publish_acceleration` | false | Do not publish acceleration [not verified]. |
| `permit_corrected_publication` | false | Never re-publish an older time after a late reading rewinds the filter. |
| `transform_time_offset`, `transform_timeout` | 0.0, 0.0 | No frame look-ups needed: every input already arrives in base_link. |
| `print_diagnostics` | true | Publish rejected readings and timing problems on `/diagnostics`; the page to watch if the blend misbehaves. |
| `debug` | false | No debug file. |
| `dynamic_process_noise_covariance` | false | Keep the process noise fixed rather than scaling it with speed [not verified]. |
| `odom0_config` (and `odom1_config`) | vx, vy, vyaw true | Which of the 15 quantities each input supplies (order: x, y, z, roll, pitch, yaw, then their speeds, then accelerations). |
| `imu0_config`, `imu1_config` | vyaw only | Turn rate only. The robot gyroscope's heading is not used: with no compass it just carries the gyro's drift. |
| `*_differential`, `*_relative` | false | Do not convert positions into differences; not needed since only speeds are fused. Differential mode would be far too confident and would read a "tracking lost" message as a jump to the origin. |
| `odom0_queue_size`, `imu0_queue_size` | 20 | Two seconds of readings may wait, so a WiFi stall that releases a burst loses nothing. |
| `imu1_queue_size` | 100 | Two seconds of 50 Hz camera-gyro readings. |
| `odom1_queue_size` | 10 | Camera odometry buffer. |
| `*_nodelay` | true | Send each small message at once rather than let the network batch them. |
| `odom1_twist_rejection_threshold` | 6.0 | Throw away a camera reading more than 6 standard deviations from what the filter expects (a "Mahalanobis gate": distance measured in units of expected spread). Only wildly wrong camera answers are dropped. Wheels and gyroscopes have no gate: they carry the robot when the camera is lost. |
| `process_noise_covariance` | package defaults, written out | How much the true motion may change between readings beyond what the constant-speed model predicts. Pinned so the value is visible. |
| `initial_estimate_covariance` | 1e-9 on the diagonal | Starting uncertainty: tiny, because `odom` is defined as where the robot starts. |

Notes: the header says `rtabmap_common.launch` hard-codes the camera odometry's TF publishing; that is **out of
date**, it is now the `vo_publish_tf` argument (section 12). The conditioner also replaces the gyroscopes' claimed
variance with 2.5e-5 and removes the gyro offset measured while parked (details in `fused_odometry.launch`, section 13).

---

## 6. robot_frames.yaml - the robot's physical geometry

**Path:** `catkin_ws/src/sidewalk_bringup/config/robot_frames.yaml`
**Used by:** the start-up checks (`preflight.launch`, `nav_preflight.py`, `obstacle_segmenter.py`) and
`bringup.launch`. **Every number is a statement about the physical robot, not a tuning value**: measure, do not adjust.
Units are metres and radians; x forward, y left, z up; all offsets are from base_link.

| setting | value | what it controls (plain words) |
|---|---|---|
| `measured` | false | Applies to the **chassis block only**: the chassis sizes are the manufacturer's figures, never measured on this unit. Must stay a single top-level value (a checker reads it by line). |
| `measured_by`, `measured_date` | empty | Who measured the chassis and when; empty because nobody has. |
| `robot_model`, `spec_source` | Clearpath Husky A200, the user manual | Where the chassis numbers come from. |
| `mount_revision` | rev-0-placeholder | Version label of the camera mount. |
| `base/frame_id` | base_link | The robot's reference frame. |
| `base/footprint_length`, `footprint_width` | 0.99, 0.67 | Outline of the body (manufacturer figures). Used for the navigation outline (section 7). |
| `base/wheel_separation` | 0.55 | Distance between left and right wheel centres. |
| `base/wheel_radius` | 0.165 | Wheel radius. |
| `base/ground_clearance` | 0.13 | Gap under the chassis. **Not the same thing** as the next line despite the similar number. |
| `base/base_link_height_above_floor` | 0.13228 | Height of base_link above the floor, from the robot model. Add it to any base_link height to get height above the floor. |
| `camera_front/frame_id`, `optical_frame` | zedx_front_left_camera_frame, ..._optical_frame | The left lens frames, where the camera measures depth from. |
| `camera_front/measured`, `measured_date`, `measured_method` | true, 2026-08-29, tape plus floor-plane fit | This block **was** measured. |
| `camera_front/x, y, z` | 0.071, 0.020, 0.563 | Left lens position from base_link (0.695 m above the floor). **Not the mount position**: the camera launch file's `cam_pos_*` arguments use the mount, which is 60 mm to the side. |
| `camera_front/roll` | 0.0 | Sideways tilt. |
| `camera_front/pitch` | 0.0565 | Measured 3.24 degrees nose down. The same value must be in both robot models (the files the file warns about). |
| `camera_front/yaw` | 0.0 | **Unmeasured**, not a claim of zero. |
| `camera_rear/*` | installed false, placeholder pose facing backwards | A second, rear-facing camera that was never fitted. |
| `imu/source`, `frame_id` | zed_x_internal, zedx_front_imu_link | Records which motion sensor is used. |
| `imu/rate_hz` | 400 | **Out of date**: the ZED X datasheet gives 200 Hz (section 3), and the camera gyro was measured at about 252 Hz published in drive 3 [not verified which figure this key should hold]. |
| `imu/external_imu_present` | false | No separate motion sensor on the Jetson side. Note: since drive 3 the blend (section 5) also uses the **robot's** gyroscope; this key predates that. |
| `expected_frames` | base_link and three camera frames | The frames the start-up check requires in TF. Edit here only; the check reads this list. |

---

## 7. costmap_common.yaml - shared settings for both navigation costmaps

**Path:** `catkin_ws/src/sidewalk_navigation/config/costmap_common.yaml`
**Used by:** navigation (`move_base_nav.launch` loads it into both costmaps). A costmap (a grid over the floor where
each 5 cm square holds 0 = free up to 254 = wall) is built from layers: the saved walls, what the camera sees now, and
a keep-away zone around both. Package: costmap_2d 1.17.3.

| setting | value | what it controls (plain words) |
|---|---|---|
| `footprint` | 0.99 x 0.67 m rectangle | The robot outline used for collision checks, centred on base_link. |
| `footprint_padding` | 0.05 | 5 cm added all round (1.09 x 0.77 m) for the camera mount and sensor arch, which are not in the manufacturer outline. 0.10 was tried and left the planner with "no valid control" at a junction. |
| `robot_base_frame` | base_link | The robot frame. |
| `transform_tolerance` | 0.5 | How old (seconds) a position may be before navigation refuses to use it. |
| `resolution` | 0.05 | 5 cm squares, the same as the saved map. |
| `static_layer/map_topic` | /nav/map | The saved map (drive 10's grid) the walls come from. |
| `static_layer/first_map_only`, `subscribe_to_updates` | true, false | The saved map never changes during a run. |
| `static_layer/track_unknown_space` | true | Never-seen squares stay "unknown", and the route will not go through them. |
| `static_layer/use_maximum` | false | How the layer merges with those below [not verified]. |
| `static_layer/lethal_cost_threshold` | 100 | Map values at or above this become walls [not verified]. |
| `static_layer/unknown_cost_value` | -1 | Map value that means "unknown" [not verified]. |
| `static_layer/trinary_costmap` | true | Read the map as exactly three states: free, unknown, wall [not verified]. |
| `obstacle_layer/enabled` | true | The live camera layer is on. |
| `obstacle_layer/track_unknown_space` | false | The camera layer does not keep its own "unknown" state. |
| `obstacle_layer/combination_method` | 1 | Take the larger of saved-map cost and camera cost, so the camera never erases a saved wall. |
| `obstacle_layer/footprint_clearing_enabled` | true | Camera marks under the robot's own outline are wiped. Turning it off made the planner think the robot's own square was blocked. Consequence: a wall already inside the camera's 1 m blind zone is forgotten once the robot overlaps it. |
| `observation_sources` | obstacles, floor | Two inputs: not-floor points and floor points. |
| `obstacles/topic` | /nav/obstacles_cloud | Not-floor points from `obstacles_detection` (section 11). |
| `obstacles/marking`, `clearing` | true, true | Not-floor points mark squares occupied and clear the squares in front of them. |
| `obstacles/min_obstacle_height` | -0.10 | Ignore points lower than 3 cm above the floor (base_link is 0.132 m up). |
| `obstacles/max_obstacle_height` | 1.20 | Ignore points above 1.2 m (above the robot's tallest part). |
| `obstacles/obstacle_range` | 2.5 | Mark obstacles only up to 2.5 m away. At 4.0 m, plain end walls read too close and aborted a goal. |
| `obstacles/raytrace_range` | 5.0 | Clear free space up to 5 m away. |
| `obstacles/expected_update_rate` | 1.0 | Warn if the points are more than 1 s old (the command sender stops the robot at 1 s). |
| `obstacles/inf_is_valid` | false | Do not treat "infinitely far" readings as free-space evidence [not verified]. |
| `obstacles/observation_persistence` | 0.0 | Use only the newest set of points [not verified]. |
| `floor/*` | marking false, clearing true, range 2.5 / 5.0, heights -0.50 to 1.20 | Floor points never mark, only clear, so a person who walked away is erased once the floor behind them is seen. |

---

## 8. costmap_global.yaml and costmap_local.yaml - the two costmaps

**Paths:** `catkin_ws/src/sidewalk_navigation/config/costmap_global.yaml`, `.../costmap_local.yaml`
**Used by:** navigation. The **global** costmap covers the whole saved floor and the route is planned on it; the
**local** costmap is a small moving square the wheel commands are checked against.

### Global

| setting | value | what it controls (plain words) |
|---|---|---|
| `global_frame` | map | Works in the map frame (drive 10's frame). |
| `update_frequency`, `publish_frequency` | 2.0, 1.0 | Recompute twice a second; publish once a second (for display only). |
| `rolling_window` | false | Size comes from the saved map (33.75 x 40.1 m). |
| `track_unknown_space` | true | Keep "unknown" squares. |
| `plugins` | static, obstacle, inflation | The three layers, in that order. |
| `inflation_layer/inflation_radius` | 0.65 | Keep-away zone width. Must be at least the outline's corner distance plus padding (0.648 m), so a zero-cost square really means "the robot fits here facing any way". |
| `inflation_layer/cost_scaling_factor` | 3.0 | How quickly the keep-away cost fades with distance; low keeps the route in the middle of 1.7-2.3 m corridors. |
| `inflation_layer/inflate_unknown` | false | Do not add a keep-away zone around unknown squares. |

### Local

| setting | value | what it controls (plain words) |
|---|---|---|
| `global_frame` | odom | Works in `odom`, not `map`: odom is smooth, while map jumps a little at each recognised place, which must not teleport nearby obstacles. |
| `update_frequency`, `publish_frequency` | 5.0, 2.0 | Recompute 5 times a second; publish twice a second. |
| `rolling_window`, `width`, `height` | true, 6.0, 6.0 | A 6 x 6 m square that moves with the robot. |
| `track_unknown_space` | false | No unknown state. |
| `always_send_full_costmap` | true | Send the whole square every time; the command sender's own clearance check needs it. |
| `plugins` | local_walls, obstacle, inflation | Saved walls **minus** the squares drive 10 itself drove over, plus the camera, plus keep-away. Added after the robot's corner touched a wall post the camera could not see; removing drive-10 squares keeps a 1.0 m narrow passage open. |
| `local_walls/map_topic` | /nav/map_local | The trimmed wall map. Other `local_walls` keys match the global static layer. |
| `inflation_layer/inflation_radius` | 0.65 | As global. |
| `inflation_layer/cost_scaling_factor` | 5.0 | Steeper: near obstacles matter more to the wheels. |

---

## 9. move_base.yaml, global_planner.yaml, dwa.yaml - the navigation programs

**Paths:** `catkin_ws/src/sidewalk_navigation/config/move_base.yaml`, `global_planner.yaml`, `dwa.yaml`
**Used by:** navigation. move_base (ROS's navigation program, version 1.17.3) finds a route and turns it into wheel
commands. The file says every name was checked by reading it back from the running program.

### move_base.yaml

| setting | value | what it controls (plain words) |
|---|---|---|
| `base_global_planner` | global_planner/GlobalPlanner | The route finder (below). |
| `base_local_planner` | dwa_local_planner/DWAPlannerROS | The wheel-command chooser (below). |
| `controller_frequency` | 10.0 | Ten steering decisions a second (the Husky's own control loop runs at 10 Hz). |
| `planner_frequency` | 1.0 | Re-plan the route every second, so a position correction or new obstacle reshapes it within 1 s. |
| `controller_patience` | 3.0 | Seconds without any safe wheel command before trying the recovery steps. |
| `planner_patience` | 5.0 | Seconds without any route before trying the recovery steps. |
| `max_planning_retries` | 3 | Then give up on this goal (the robot stays stopped). |
| `oscillation_timeout`, `oscillation_distance` | 30.0, 0.20 | "Stuck" means less than 0.2 m of progress in 30 s. Long because a slow turn on the spot does not count as progress. |
| `shutdown_costmaps` | false | Keep the costmaps running between goals. |
| `recovery_behavior_enabled` | true | Recovery steps are allowed. |
| `clearing_rotation_allowed` | false | Never spin on the spot by itself: an unplanned spin in a narrow corridor is the move the person walking beside the robot cannot predict. |
| `recovery_behaviors` | conservative_clear, aggressive_clear | When stuck: forget camera obstacles further than 3 m (`reset_distance 3.0`), then all of them (`0.0`). Only `obstacle_layer`; saved walls are never forgotten. |

### global_planner.yaml

| setting | value | what it controls (plain words) |
|---|---|---|
| `use_dijkstra` | true | Dijkstra's shortest-path search (false = A*, same route here with fewer squares searched). |
| `use_grid_path` | false | Follow the smooth downhill of the cost field rather than hop square to square: a smoother route to follow. |
| `use_quadratic` | true | Smoother cost estimate between squares. |
| `allow_unknown` | false | Never route through unknown squares. If a goal is refused as "no plan", pick one on known free floor. |
| `default_tolerance` | 0.30 | If the goal square is blocked, accept the nearest free square within 0.3 m. |
| `cost_factor` | 3.0 | How strongly nearness to walls is avoided, relative to distance. |
| `neutral_cost` | 50 | Cost of one step over free floor. |
| `lethal_cost` | 253 | Squares at or above this are walls. |
| `orientation_mode` | 1 | Each route point faces along the route. |
| `orientation_window_size` | 1 | How many route points are used to work out that direction [not verified]. |
| `publish_potential`, `visualize_potential` | false | Do not publish the search's working grid. |

### dwa.yaml

DWA (dynamic window approach: ten times a second, try a grid of forward-speed and turn-rate pairs the robot can
reach, simulate each 2 s ahead, drop any that touch an obstacle, and pick the best by staying on route, nearing the
goal and keeping clear). Speed limits are enforced again by the Jetson sender and the robot receiver.

| setting | value | what it controls (plain words) |
|---|---|---|
| `max_vel_trans`, `max_vel_x` | 0.30 | Top speed 0.30 m/s (the user's brief). |
| `min_vel_trans` | 0.05 | Below this a candidate counts as "not moving". |
| `min_vel_x` | 0.0 | **No reversing**: only one camera, facing forward. |
| `max_vel_y`, `min_vel_y`, `acc_lim_y`, `vy_samples` | 0, 0, 0, 1 | The Husky cannot move sideways. |
| `max_vel_theta` | 0.40 | Top turn rate 0.40 rad/s (23 degrees a second). |
| `min_vel_theta` | 0.30 | Slowest turn on the spot it will ask for. Below this the skid-steer Husky just rocks; 0.2 stalled in the simulator. |
| `acc_lim_x`, `acc_lim_trans` | 1.0 | Speed-up limit, m/s per second. |
| `acc_lim_theta` | 1.5 | Turn speed-up limit, rad/s per second. |
| `sim_time` | 2.0 | Seconds each candidate is simulated ahead (0.6 m at top speed). |
| `sim_granularity`, `angular_sim_granularity` | 0.025, 0.05 | Distance / angle between collision checks along a candidate. |
| `vx_samples`, `vth_samples` | 10, 21 | How many speeds and turn rates are tried. Odd turn count so "exactly straight" is always a candidate. |
| `path_distance_bias` | 32.0 | Weight on staying on the route. |
| `goal_distance_bias` | 20.0 | Weight on heading for the goal. |
| `occdist_scale` | 0.02 | Weight on keeping away from obstacles (double the package default, for narrow corridors). |
| `forward_point_distance` | 0.325 | Point ahead of the robot used to score heading. |
| `stop_time_buffer` | 0.2 | Seconds of stopping room required before a collision. |
| `scaling_speed`, `max_scaling_factor` | 0.25, 0.0 | The outline is **not** enlarged at higher speed: the package default would make it too wide for 1.0-1.4 m narrow parts. |
| `xy_goal_tolerance` | 0.25 | "Arrived" once the robot centre is within 25 cm of the goal. |
| `yaw_goal_tolerance` | 0.30 | Final heading may be 17 degrees off. |
| `latch_xy_goal_tolerance` | true | Once within 25 cm, only turn on the spot, never creep about. |
| `oscillation_reset_dist` | 0.05 | Distance to move before back-and-forth detection resets [not verified]. |
| `prune_plan` | true | Drop route points the robot has already passed [not verified]. |
| `publish_traj_pc`, `publish_cost_grid_pc` | false | Do not publish the candidate paths or scoring grid for display. |

---

## 10. robot_replay_config.yaml - the LiDAR decoder for the reference replay

**Path:** `run/lib/lidar_reference/robot_replay_config.yaml`
**Used by:** the LiDAR reference replay: the colleague's replay pipeline on the robot (`3dreplay_pipeline.launch`,
Helios sensor) run with `Reg/Force3DoF=true`, driven from the Jetson. This file is the settings of the Helios decoder
(the program that turns the RoboSense Helios LiDAR's raw network packets into 3D points). **It is a reconstruction**:
the Jetson's mirror of the colleague's `replay_config.yaml` plus one line (`split_angle: 180`) that the robot's copy
has. Replace it with the robot's own copy when the robot is on (README step R1 in that folder). The colleague's
package is not edited.

A LiDAR trajectory is a second opinion, **not ground truth**: report "agreement with the LiDAR estimate" (project rule 19).

| setting | value | what it controls (plain words) |
|---|---|---|
| `common/msg_source` | 2 | Read packets from ROS (a recording being replayed), not from a live sensor or a capture file. |
| `common/send_packet_ros` | false | Do not re-publish packets. |
| `common/send_point_cloud_ros` | true | Publish the decoded 3D points. |
| `driver/lidar_type` | RSHELIOS | Sensor model. |
| `driver/msop_port`, `difop_port` | 6699, 7788 | Network ports of the live sensor; not used when reading from ROS [not verified]. |
| `driver/imu_port` | 0 | No LiDAR motion sensor. |
| `driver/user_layer_bytes`, `tail_layer_bytes` | 0, 0 | No extra bytes in each packet. |
| `driver/min_distance`, `max_distance` | 0.2, 200 | Keep points between 0.2 m and 200 m. |
| `driver/use_lidar_clock` | true | Time-stamp points with the LiDAR's clock, not the computer's. |
| `driver/dense_points` | false | Keep empty (NaN, not-a-number) points in place rather than removing them. |
| `driver/ts_first_point` | true | A sweep's time stamp is its first point, not its last. |
| `driver/start_angle`, `end_angle` | 0, 360 | Keep the full circle. |
| `driver/split_angle` | 180 | The direction at which one full sweep is cut into one point cloud. **The line that differs from the mirror copy.** |
| `driver/pcap_*` | repeat true, rate 1.0, a path | Only used when reading a capture file (msg_source 3); unused here. |
| `ros/ros_frame_id` | rslidar | Frame name of the points. |
| `ros/ros_recv_packet_topic`, `ros_send_packet_topic` | /helios/packets | Topic the recorded packets are read from. |
| `ros/ros_send_imu_data_topic` | /helios/imu_data | Unused (no LiDAR motion sensor). |
| `ros/ros_send_point_cloud_topic` | /helios/points | Topic the decoded points are published on. |
| `ros/ros_queue_length` | 100 | Message queue depth. |

---

## 11. obstacles.yaml - live floor / not-floor split

**Path:** `catkin_ws/src/sidewalk_navigation/config/obstacles.yaml`
**Used by:** navigation (`nav_run.launch`) and the SLAM session's live obstacles (`live_obstacles.launch`).
Settings for `rtabmap_util/obstacles_detection`, which splits the camera's 3D points into floor and not-floor. The
`Grid/` values match `rtabmap_zedx.yaml` on purpose, so a live obstacle means the same as a wall on the saved map.

| setting | value | what it controls (plain words) |
|---|---|---|
| `frame_id` | base_link | Frame the points are sorted in (heights are from base_link). |
| `wait_for_transform` | 0.2 | Seconds to wait for a frame look-up. |
| `Grid/NormalsSegmentation` | true | Floor = points whose surface faces up. |
| `Grid/MaxGroundAngle` | 30 | As section 1.1. |
| `Grid/MinGroundHeight`, `Grid/MaxGroundHeight`, `Grid/MaxObstacleHeight` | -0.20, 0.10, 1.60 | Same height band as section 1.1, from base_link. |
| `Grid/CellSize` | 0.05 | Points are snapped to 5 cm before sorting. |
| `Grid/NoiseFilteringRadius`, `NoiseFilteringMinNeighbors` | 0.09, 5 | Drop lone points, as section 1.1. |
| `Grid/ClusterRadius`, `Grid/MinClusterSize` | 0.1, 10 | Group points into obstacles; groups smaller than 10 points are ignored (official description: minimum cluster size to project). |
| `Grid/FlatObstacleDetected` | true | Official description: "flat obstacles detected" (default) [exact effect not verified]. |

---

## 12. Launch-file overrides of RTAB-Map settings

Launch files set some RTAB-Map values **after** the YAML files are loaded; the last setting wins. These are what the
program actually ran with.

### rtabmap_common.launch (every RTAB-Map workflow)

**Path:** `catkin_ws/src/sidewalk_slam/launch/rtabmap_common.launch`. Starts `rgbd_sync` (packs colour, depth and
calibration into one message), `rgbd_odometry` and `rtabmap`. Does **not** start the camera.

| setting / argument | value | what it controls (plain words) |
|---|---|---|
| `Reg/Force3DoF` (on odometry and map) | from `force_3dof` (default true) | Overrides the YAML value; outdoors pass `force_3dof:=false`. |
| `Mem/IncrementalMemory` | computed from `localization` | **Broken**: roslaunch hands the test a true/false value, not the text "true", so it always comes out true (mapping). `localise_run.launch` fixes it after the include (docs/SOLVED.md). |
| `localization` | false | false = build a map, true = also load `rtabmap_localization.yaml`. |
| `database_path` | a file on the microSD card | The map file. Drives pass their own path on the Jetson's internal disk. |
| `delete_db_on_start` | false | **Destructive if true**: erases the map file at start. |
| `visual_odometry` | true | RTAB-Map computes its own camera odometry. |
| `vo_publish_tf` | true | Whether camera odometry publishes `odom -> base_link`. **false when the blend runs** (`FUSION=1`); never false without the blend, or nothing publishes the link. |
| `approx_sync` | false | Require exactly matching time stamps. The drives pass true. |
| `queue_size` | 5 | Depth of six message queues. Each frame is 16 MB; 30 allowed about 2.9 GB of buffered frames, and old frames make odometry lose tracking. |
| `wait_imu_to_init` | true | Hold odometry until the first motion-sensor reading, so the map starts level. |
| `stereo` | false | Use picture plus depth (false) or left/right pictures (true). |
| `extra_rtabmap_params` | empty | Change one RTAB-Map setting for one run on the command line (e.g. `--Grid/RayTracing false`) without editing the YAML. **Record it with the run.** |
| `frame_id` | base_link | The body whose position is reported. |
| `use_sim_time` | false | true only when replaying a recording. A leftover true freezes a live run. |
| `rtabmap_viz`, `rviz` | false | Viewers off on the robot. |
| `depth_scale` (rgbd_sync) | 1.0 | Leave depth values unchanged. |
| `guess_frame_id` (odometry) | empty | Odometry takes no outside motion guess. |
| `publish_tf` (rtabmap) | true | RTAB-Map publishes `map -> odom`. |
| `tf_delay` (rtabmap) | 0.05 | Interval in seconds between `map -> odom` publications [not verified]. |
| `map_always_update` (rtabmap) | false | Update the published map only when a node is added [not verified]. |
| `subscribe_rgbd`, `subscribe_depth`, `subscribe_scan` | true, false, false | Take the packed message; no laser scan (which would also rewrite `Grid/Sensor`, project rules section 2.5). |

### rtabmap_mapping.launch and record_mapping_run.launch (mapping drives)

**Paths:** `catkin_ws/src/sidewalk_slam/launch/rtabmap_mapping.launch`, `.../record_mapping_run.launch`.
They pass arguments down to `rtabmap_common.launch` and set no RTAB-Map setting themselves.
`record_mapping_run.launch` also starts the trajectory recorders and the 1 Hz run monitor.

| argument | default | what the drives passed | meaning |
|---|---|---|---|
| `use_sim_time` | true (record) | false | Real time on the robot. |
| `approx_sync` | true (record) | true | Allow nearly matching time stamps. |
| `wait_imu_to_init` | false (record) | true | The real camera has a motion sensor to wait for. |
| `force_3dof` | true | true | Flat-floor mode. |
| `ground_truth` | true | false | No simulator truth on the real robot. |
| `vo_publish_tf` | true | false with `FUSION=1` | The blend owns `odom -> base_link`. |
| `rtabmap_log` | empty | the run's log file | The monitor counts loop closures and resets from it. |
| `monitor_out` | per-run folder | default | One monitor file per run, so runs never overwrite each other. |

### localise_run.launch (localisation, navigation)

**Path:** `run/lib/localise/localise_run.launch`. Same stack as drive 4, with `localization:=true`,
`delete_db_on_start` hard-wired false, and `database_path` required to be a **working copy**, never the master map.

| setting | value | what it controls (plain words) |
|---|---|---|
| `Mem/IncrementalMemory` | false | Re-set after the include: **the fix that makes localisation actually happen**. The start script also checks the log says "Localization mode". |
| `Vis/MinInliers` | 20 (argument `min_inliers`) | Back to the mapping value; at 25 the correct start place was refused. |
| `Rtabmap/LoopThr` | 0.08 (argument `loop_thr`) | At 0.15 correct places were never accepted; 0.05 gave 3 fixes in places outside the map; 0.08 gave none. |
| other arguments | approx_sync true, queue_size 5, wait_imu_to_init true, force_3dof true, visual_odometry true | Drive 4's values. |

### slam_run.launch (SLAM session: keep drawing on an old map)

**Path:** `run/lib/slam_session/slam_run.launch`. Opens a **copy** of drive 10's map in mapping mode, so new places are
added and recognised places join old and new parts. `delete_db_on_start` hard-wired false.

| setting | value | what it controls (plain words) |
|---|---|---|
| `Mem/IncrementalMemory` | true | Mapping mode, written explicitly. |
| `Mem/InitWMWithAllNodes` | true | Every saved place can be recognised from the start (on this map it loads the same 860 nodes as the default). |
| `RGBD/AggressiveLoopThr` | 0.05 (the default) | Recognition score needed for the **first** join to the old map; written so the log proves it. |
| `Rtabmap/LoopThr` | 0.08 (argument `loop_thr`) | Recognition score needed **after** the join; 0.08 is what whole-floor localisation used on this map with no wrong fixes. `loop_thr:=0.11` restores drive 10's value. |
| `mask` | false | true = RTAB-Map reads the people-masked depth (needs the camera started with `zedx_front_od.yaml`, section 4). |
| `depth_raw`, `depth_masked` | the camera's depth topic and its `_masked` twin | Which depth picture is used. |

---

## 13. fused_odometry.launch - starts the blend

**Path:** `catkin_ws/src/sidewalk_slam/launch/fused_odometry.launch`
**Used by:** drives with `FUSION=1`. Starts `ekf_input_conditioner.py` (our small program that cleans the inputs) and the
EKF with `ekf_fused.yaml` (section 5). Both are `required`: if either dies, both stop and nothing publishes
`odom -> base_link`. **Do not restart it mid-drive**: a new EKF starts at the origin, a jump the map cannot absorb.

| argument | default | what it controls (plain words) |
|---|---|---|
| `run` | fused | Run name; the conditioner writes a `.progress` line for the jobs page. |
| `config` | `ekf_fused.yaml` | The EKF settings file. |
| `vo_lin_var_scale` | 8.9 | Multiplier on the camera's own claimed speed uncertainty: how much camera speed counts against the wheels. |
| `vo_yaw_var_scale` | 0.14 | Same, for the camera's turn rate. |
| `vo_lin_var_floor`, `vo_yaw_var_floor` | 4e-4, 4e-6 | The camera's uncertainty is never allowed below these. |
| `hold` | true | When every input has gone silent, report "not moving" rather than invent a path. |
| `hold_after_s` | 0.3 | How long all inputs must be silent before the hold starts. |
| remap `odometry/filtered` -> `/fused/odometry` | | Keeps the blend's output separate from the robot's own `/odometry/filtered` in recordings. |

---

## 14. Things to check before trusting any of this

- **One publisher per TF link.** `odom -> base_link` comes from the blend (with `FUSION=1`) or from camera odometry,
  never both; `map -> odom` from RTAB-Map only. The camera driver publishes neither.
- **Read back, do not assume.** RTAB-Map, robot_localization and move_base all ignore misspelt names silently.
- **Silent detector fallback.** Grep every RTAB-Map log for "cannot be used" (project rules section 2.6).
- **Localisation mode.** Check the log says "Localization mode (Mem/IncrementalMemory=false)".
- **Database integrity.** With `DbSqlite3/Synchronous` 0, run the integrity check after every drive.
- **Parameter names that do not exist on this build** (do not add them): `Odom/CovarianceScale`,
  `Reg/VarianceFromInliersCount`, `Optimizer/Slam2D`, `Grid/FromDepth` (project rules section 3).
