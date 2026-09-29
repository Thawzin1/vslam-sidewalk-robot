# SLAM session - continue drive 10's map, add new areas, keep people out of it

*Written 27 Sept 2026 (Hamilton) on the Jetson. RTAB-Map (the mapping program) 0.21.13, ZED SDK 4.2.5, ZED ROS
wrapper 3.5.0. Nothing live was changed: every file here is a staged copy (DEPLOY_NOTES.md). Labels:
**[VERIFIED - file:line]** read in the program's own source (the copies in
`src_ref/` (project records), whose line numbers match this build's log messages - see
`../localisation_demo_2026-09-26/whole_floor_diag/DIAG.md` section 2) or checked on this build;
**[INFERENCE]** reasoned from verified facts; **[BENCH]** seen on the bench test of 27 Sept (BENCH_RESULTS.md);
**[UNTESTED]** not yet seen working.*

*Words used: **node** = one saved map snapshot (a picture + its depth + where it was taken); **session** = one run
of the mapping program; **map id** = the number RTAB-Map gives each separate piece of map; **loop closure** = the
map recognising a place it has seen and pulling itself straight; **join** = the first loop closure between the new
session and drive 10's map.*

---

## 0. In five lines

1. RTAB-Map is started in **mapping mode** on a **copy** of drive 10's closed map (`~/slam_series2/<run>.db`), with
   no delete flag. It loads drive 10's 860 map places into memory and **starts a new map (map id 1)** at the robot's
   first picture; node numbers continue from 1559.
2. Until it recognises a drive-10 place, the new map **floats separately**: RTAB-Map does not use the start mark or
   any saved "last position" to place it. The only way to join is **recognising a place by appearance** (score >=
   0.05, then >= 20 matching 3D points). Nearby re-matching cannot reach the old map before the join.
3. At the join the two maps become **one graph**. Drive 10's first node stays fixed, the new session is pulled onto
   the old map, and from then on every recognition (score >= 0.08) and every nearby re-match can **correct old and
   new parts together**. New corridors are simply new nodes of the same graph.
4. The session is recorded so it can be replayed: the **camera's own recording (SVO=1)** at full picture rate,
   the database itself (every node's picture and depth), and the usual fusion recording.
5. People: the ZED camera's **people detector** gives a box per person; `depth_person_mask.py` blanks the depth
   inside each box before RTAB-Map sees it, so people are used neither for tracking, nor for recognising places,
   nor drawn into the map (the published "remove the moving class before tracking" idea, box-level). The occupancy
   grid additionally clears places a person stood once later rays see through them (ray tracing, already on).

---

## 1. How RTAB-Map continues an existing map on this build

### 1.1 The picture

```
 BEFORE THE JOIN  (new session floats; RTAB-Map treats the two as unrelated)

   drive 10's map  (map id 0, nodes 1..1558, 860 in the graph)        new session (map id 1, nodes 1559..)
   ┌─────────────────────────────────────────┐                        ●──●──●──●   <- robot
   │  ●──●──●──●──●──●──●──●──●──●──●──●──●  │                        (its own frame = where tracking
   │  │                                  │   │                         started; nothing links it to
   │  ●   (whole 2nd-floor loop)         ●   │                         the old map yet)
   │  ●──●──●──●──●──●──●──●──●──●──●──●──●  │
   └─────────────────────────────────────────┘
        ^ node 1 = start mark, fixed

 THE JOIN  (one recognition: the new picture looks like old node k, score >= 0.05, >= 20 matching 3D points)

   new ●  ─────── loop closure (link type 1) ───────►  old node k

 AFTER THE JOIN  (one graph; node 1 stays put, everything else may move)

   ●──●──●──●──●──●──●──●──●──●──●──●──●
   │  ▲ new nodes re-trace old corridors: ●   ◄── more recognitions (score >= 0.08) and nearby re-matches
   ●  │ each closure pulls old AND new     ●       (within 5 m and 120 graph steps) keep the two consistent
   ●──●──●──●──●──●──●──●──●──●──●──●──●
                          └──●──●──●──●  ◄── a NEW side corridor: new nodes hanging off the joined graph
```

*Plain terms: RTAB-Map keeps drive 10's floor plan and starts a fresh sheet for today. The moment it recognises
one place from the floor plan, it glues today's sheet onto it at that place. After that it is one map: new
corridors are added to it, and every place recognised again straightens both the old and the new part.*

### 1.2 Step by step, with where it is decided

| step | what happens | where (0.21.13) | label |
|---|---|---|---|
| a. open | `database_path` = the working copy. The node is started with **no** `--delete_db_on_start` (hard-wired false in `slam_run.launch`; also, `rtabmap_common.launch`'s own test never passes it, SOLVED.md 26 Sept). Checked: `roslaunch --args /rtabmap/rtabmap slam_run.launch ...` shows no delete flag | CoreWrapper.cpp; `slam_run.launch` | [VERIFIED] |
| b. what is loaded into memory | `Mem/InitWMWithAllNodes` false (default) loads "the last session's nodes" = nodes whose `time_enter` is at or after the last `Info` row; true loads every node with weight > -9. **On drive 10's map both give the same 860 nodes**: its other 698 nodes have weight -9 (discarded while standing still) and the 860 are exactly the last-session set. Pinned **true** so every graph node can be recognised in any later session | Memory.cpp:237-257; DBDriverSqlite3.cpp:2416-2440 (its "children" filter is always true, so weight is the only filter), 3607-3625; counted on the master, read-only | [VERIFIED] |
| c. numbering | new node ids continue from the highest id (1559 on); the new session gets **map id = highest map id + 1 = 1** | Memory.cpp:374-377 | [VERIFIED] |
| d. first picture | not linked to any old node: a neighbour link is only made to the previous picture of the **same** map id in short-term memory, which is empty at start | Memory.cpp:998-1000 | [VERIFIED] |
| e. where the robot is drawn before the join | **the saved "last position" is ignored in mapping mode** (only localisation mode uses it: `if(!_memory->isIncremental() ...)`), so the map correction starts as identity: the robot is at its own tracking position. Because every drive starts on the start mark facing east with tracking at zero, and drive 10's node 1 is at (0.002, 0.000) m, 0.06 deg in its saved graph, the two frames happen to coincide to parking accuracy (0.2-0.5 m, [[tape-mark parking]]). RTAB-Map does not use that | Rtabmap.cpp:1341-1346 (localisation only); Admin.opt_poses of the master | [VERIFIED] |
| f. the local graph before the join | every new node carries a gravity link (drive 10's map has 1554), and `Optimizer/GravitySigma` 0.3 > 0 with `Mem/UseOdomGravity` false triggers a graph optimisation at every node; the optimisation takes only the nodes **linked** to the newest one (graph search from it), so the working graph becomes **the new session alone**, and the old 860 drop out of it | Rtabmap.cpp:3196-3210, 3789, 5160-5166, 3922 | [VERIFIED] source; **not observable on the bench** (both joins came within 4 s) |
| g. what can join | (1) **nearby re-match cannot**: it only considers nodes with a graph path of at most `RGBD/ProximityMaxGraphDepth` (120) steps, and there is no path to the old map. (2) **Recognition by appearance can**, and because the best-matching old node is not in the working graph the bar is `RGBD/AggressiveLoopThr` **0.05**, not `Rtabmap/LoopThr` | Rtabmap.cpp:2666-2689 (path check); 2146-2157 (bar) | [VERIFIED] |
| h. the join itself (bench: 2.5 s and 3.6 s after the map loaded, to drive-10 nodes 138 and 144 at the start mark; BENCH_RESULTS.md) | one picture over 0.05, then the 3D check (`Vis/MinInliers` 20 matching points) -> link type 1 from the new node to the old one. **No "second opinion" in mapping mode**: the wait-for-a-second-recognition rule (`RGBD/MaxOdomCacheSize`) is localisation-only. The graph check (`RGBD/OptimizeMaxError` 3.0) cannot catch a wrong FIRST join: it is the only link between the two graphs, so any value fits it exactly | Rtabmap.cpp:3244-3248 (cache only if not incremental), 3806-3905 (check) | [VERIFIED] source; the last sentence [INFERENCE] |
| i. after the join | one connected graph; with `RGBD/OptimizeFromGraphEnd` false the graph is fixed at its **lowest id = drive 10's node 1**, so drive 10's frame is kept and today's session is moved onto it (the live map -> odom link jumps once). Every later recognition uses **0.08** (the old nodes are now in the working graph), nearby re-matches reach old nodes within 5 m and 120 steps, and the graph check now has many links to compare, so a wrong recognition that disagrees with the rest is refused | Rtabmap.cpp:5160-5166, 2146-2157, 2666-2689, 3806-3905 | [VERIFIED] |
| j. old parts get corrected too | only node 1 is fixed; every old node is free in the optimisation, so a new closure that disagrees with drive 10's layout moves old nodes as well. The old pictures and depth are not changed; their positions are | Rtabmap.cpp optimizeGraph (5190-5260) | [VERIFIED] |
| k. closing | on "park", the saved graph (`Admin.opt_poses`) = the working graph of the last node. **Joined: old + new nodes. Never joined: the new session only** (the old map stays in the file, outside the saved graph). `slam_closed_check.py` reports which | Rtabmap.cpp:521, 736 (saveOptimizedPoses) | [VERIFIED] source; [BENCH] joined case (862 and 864 = 860 old + today's); never-joined case not seen |
| l. new areas | a corridor drive 10 never entered just produces new nodes linked to the previous ones; once the robot returns to known ground, a recognition or nearby re-match ties the end of the new corridor back to the old map | as a drive | [INFERENCE] |

**Settings that do NOT apply in mapping mode** (checked so nobody sets them expecting an effect):
- `RGBD/StartAtOrigin` - "Used only in localization mode" (`rtabmap --params`); in the source it sits inside the
  localisation branch (Rtabmap.cpp:381-389). **[VERIFIED]**
- `RGBD/MaxOdomCacheSize` - "Used only in localization mode" (`rtabmap --params`; Rtabmap.cpp:3244). **[VERIFIED]**
- The **off-map (kidnapped) start** used by the localisation demo (`set_prior.py`, a false position of (40, 40)
  written into the copy): mapping mode never reads it (step e), so it is **not used**; the honesty it gave -
  "any fix must come from real recognition" - is built in here, because nearby re-matching cannot reach the old map
  before the join (step g). **[VERIFIED]**
- `Rtabmap/StartNewMapOnLoopClosure` (default false) stays false: with true, RTAB-Map would **delete every new
  picture** until a recognition joins it to the old map, and hold back nearby re-matching until then; with false
  (as every drive), today's pictures are mapped from the first second even if the join comes late. It would not
  stop a wrong join, only a floating one. **[VERIFIED - `rtabmap --params`, Rtabmap.cpp:2634-2641, 4264-4278]**

### 1.3 Settings for the session

| setting | this session | default on this build | drive 10 | what it controls | where set |
|---|---|---|---|---|---|
| `database_path` | `~/slam_series2/<run>.db` = a byte-checked copy of `~/slam_series2/localise/s2_static_10_map.db` (sha256 `98f71ed1...`) | - | new file | which map is continued; the master is never opened by RTAB-Map | start_drive.sh step 0 |
| delete on start | never (no `--delete_db_on_start`) | not passed | not passed | erasing the file first = losing drive 10 | slam_run.launch |
| `Mem/IncrementalMemory` | **true** | true | true | true = mapping (add places); false = localisation only | slam_run.launch (pinned) |
| `Mem/InitWMWithAllNodes` | **true** | false | false | which saved places can be recognised from the start (here: the same 860 either way) | slam_run.launch (pinned) |
| `RGBD/AggressiveLoopThr` | 0.05 | 0.05 | 0.05 | recognition score needed for the **join** (while the best old place is not yet in the working graph) | slam_run.launch (written so the log shows it) |
| `Rtabmap/LoopThr` | **0.08** | 0.11 | **0.11** | recognition score needed **after** the join. 0.08 = the value the whole-floor localisation used on this map (577 fixes over 246 m, 0 in a wrong place); `LOOP_THR=0.11` gives drive 10's own | slam_run.launch (argument) |
| `Vis/MinInliers` | 20 | 20 | 20 | matching 3D points needed before a recognition is accepted | rtabmap_zedx.yaml |
| `RGBD/OptimizeMaxError` | 3.0 | 3.0 | 3.0 | how badly a new closure may disagree with the rest of the graph before it is refused | rtabmap_zedx.yaml |
| `RGBD/ProximityMaxGraphDepth`, `RGBD/LocalRadius` | 120, 5.0 m | 50, 10 m | 120, 5.0 m | nearby re-match reach | rtabmap_zedx.yaml |
| `RGBD/OptimizeFromGraphEnd` | false | false | false | false = the lowest node id (drive 10's node 1) is held fixed | rtabmap_zedx.yaml |
| `Grid/RayTracing`, `Grid/3D` | true, true | false, true | true, true | occupancy grid clears space seen through (section 7.3) | rtabmap_zedx.yaml |
| everything else | as drives 3-10 (`rtabmap_zedx.yaml`, md5 as deployed; 16-bit depth, unconfirmed saves, Force3DoF) | | | | |

**One-variable note (ENGINEERING_NOTES.md section 4 rule 5).** Against drive 10's mapping, this session changes **the start
state** (continuing a map instead of an empty one) and **`Rtabmap/LoopThr` 0.11 -> 0.08**. That is two things; it is
a demonstration, not an A/B experiment, and PASS_LINES.md judges it as such. If a clean comparison with drive 10 is
wanted instead, run with `LOOP_THR=0.11`.

---

## 2. The start, and the lesson from the localisation diagnosis

*Evidence: `../localisation_demo_2026-09-26/whole_floor_diag/DIAG.md` (s2_loc_04, same map, same start mark).*

- In localisation mode the first fix took 213 s because RTAB-Map **held** a correct recognition and waited for a
  second one over 0.08; standing still, none came. **Mapping mode has no such wait** (section 1.2 h): the first
  recognition over **0.05** with 20 matching points joins at once.
- On the start mark, s2_loc_04 recognised the right place at **0.4 s** (node 138, 0.28 m along the corridor, 97
  matching points, score over 0.05) and again at 2.8 s (score 0.0815). **[VERIFIED - DIAG.md sections 1 and 6]**
  So the join is expected within the first seconds on the mark. [INFERENCE, N = 1 start of that kind]
- **Procedure kept from DIAG.md:** stand still on the mark (hands off) until READY and at most **60 s** more. If the
  jobs page line `<run>_slam` still says "JOINED no", **drive on slowly along drive 10's route** (east down the long
  corridor - drive 10 drove every corridor in both directions, so either direction is in the map).
- **The price of "no second opinion":** a single wrong recognition before the join would glue today's sheet onto the
  wrong place, and nothing inside RTAB-Map would notice (1.2 h). Two guards:
  1. **live**: `slam_watch.py` flags `!! SUSPECT JOIN` when the joined old place is further from the start mark
     than the robot can have driven + 3 m (drive 10's saved positions are loaded from the copy). Then: touch
     `STOP_<run>`, and restart with the next run id - the working copy is disposable, the master untouched.
  2. **afterwards**: PASS_LINES.md S2 checks every link to the old map against the robot's own wheels.

---

## 3. Recording, so the session can be replayed

| what | where (Jetson unless said) | holds | replay use |
|---|---|---|---|
| the database | `~/slam_series2/<run>.db` | every node's colour picture + depth (Data table: drive 10's 1558 nodes all have both - checked on the master) and every link | `rtabmap-reprocess`, `rtabmap-export`, the pack scripts. Only one picture per node (about 1 per second while moving), not the full stream |
| camera recording (**SVO=1**, default in `start_slam.sh`) | `~/.run_records/<run>/<run>.svo2`, H265 (mode 2) | **every** camera picture pair at the camera rate | re-run the camera tracker, the people detector and RTAB-Map offline on identical pictures (section 7.5) |
| fusion recording (FUSION=1) | `~/.run_records/<run>/fusion.bag` | odometry, wheels, gyroscope, blend, `/tf`, camera calibration, **people boxes** (`obj_det/objects`, MASK=1) | the blend offline; S2 wheel check; which frames had people |
| robot LiDAR packets | robot's card, `robot_side.sh` | both LiDARs | Force3DoF LiDAR replay for S4 |

`Mem/LocalizationDataSaved` is a localisation-mode setting and is not needed here: in mapping mode every node is
saved anyway. **Disk budget** (Jetson, internal; measured on the bench, BENCH_RESULTS.md section 6): working copy
3.5 GB + about **4 GB** growth per 30 min (2.3 MB per new node, **even discarded standing-still nodes are kept**) +
camera recording **6-14 GB** per 30 min (3.4 MB/s parked measured; up to 8 MB/s moving, drive 4) + fusion.bag 0.3 GB
= **about 14-22 GB**. The Jetson had 12.3 GB free on 27 Sept 09:40; storage sense (rule 24) must release the rest
before the session (for example s2_loc_04's 3.1 GB fusion.bag once its pack is done), or run `SVO=0` (then the
people ON/OFF comparison is impossible). The recording starts **before** the map (step 2S), so it holds the join.

---

## 4. Published methods -> what we use and why

| problem | published method | what it does | what we use here | status |
|---|---|---|---|---|
| continue an existing map, add new areas | Labbé & Michaud, *Online global loop closure detection for large-scale multi-session graph-based SLAM*, IROS 2014, 2661-2666; Labbé & Michaud, *RTAB-Map as an open-source lidar and visual SLAM library for large-scale and long-term online operation*, J. Field Robotics 36(2), 416-446, 2019 | each session is a new map in the same database; maps are joined when a place is recognised across sessions; graph optimisation merges them | exactly this, RTAB-Map 0.21.13, mapping mode on a copy of drive 10 (section 1) | **implemented; bench-tested (mechanics)** |
| keep a growing map fast enough | Labbé & Michaud, *Appearance-based loop closure detection for online large-scale and long-term operation*, IEEE T-RO 29(3), 734-745, 2013 (memory management: working vs long-term memory) | move old places out of working memory when time runs short | **off** (`Rtabmap/TimeThr` 0, `Rtabmap/MemoryThr` 0: every place stays recognisable). About 860 + 900 nodes by the end, fewer than drive 10 handled live | not needed at this size |
| people in the pictures | Bescos et al., *DynaSLAM*, IEEE RA-L 3(4), 4076-4083, 2018 (Mask R-CNN outlines + multi-view check); Yu et al., *DS-SLAM*, IROS 2018, 1168-1174 (SegNet + moving-consistency check); Zhong et al., *Detect-SLAM*, WACV 2018, 1001-1010 (SSD boxes, moving probability propagated) | find the moving class, remove its pixels before tracking and mapping | ZED people detector (boxes) -> depth set to 0 inside each box (enlarged) -> RTAB-Map ignores those pixels for tracking, recognition and the map (`Vis/DepthAsMask`, `Mem/DepthAsMask` both true on this build). Closest to Detect-SLAM's boxes; no outlines (the wrapper hard-codes `enable_segmentation = false`, zed_wrapper_nodelet.cpp:2011) and no moving-consistency check (a standing person is removed too) | **implemented; live-tested with a made-up person (PASS); cost measured (section 7.2); a whole real person at 2-4 m not yet seen** |
| things that moved (a person stood there, a chair moved) | Hornung et al., *OctoMap*, Autonomous Robots 34(3), 189-206, 2013 | probabilistic occupancy: each ray through a cell lowers its "occupied" belief, each hit raises it | `Grid/RayTracing` true + `Grid/3D` true (OctoMap built in: `rtabmap --version` "With OctoMap: true"); the 2D grid adds +0.85 per hit and -0.41 per miss (log-odds of `GridGlobal/ProbHit` 0.7, `ProbMiss` 0.4) | **on since drive 1 (rtabmap_zedx.yaml)**; its effect on a moved object not measured |
| moving objects tracked as objects | (ZED SDK object tracking) | id, 3D position and velocity per person | the detector's tracking is on (ids in `mask_events.csv`); nothing uses velocity | future work |

**Honest limits (stated before the session):**
- **The 3D point cloud keeps what drive 10 saw.** A chair that moved since drive 10 stays in drive 10's nodes'
  pictures and depth; the export of the combined map (`map_3d` scripts) draws it in its old place as well as its new
  one. Ray tracing clears only the **occupancy grid**, and only where today's rays pass.
- **Clearing needs evidence.** In the 2D grid a cell hit once needs about 3 later "seen through" rays before it
  reads free (0.85 - 3 x 0.41 < 0) [VERIFIED - GlobalMap.cpp:37-67, OccupancyGrid.cpp:492, 617]. A person seen from
  one place only, never seen through afterwards, stays in the grid unless masked.
- **Boxes are coarse.** The wall behind a person inside the box loses its depth too (fewer features that moment,
  never a wrong point). A person the detector misses (turned away, far, partly hidden) is not masked.
- **People in drive 10's own pictures** (if any) are in the old map; masking applies to today's pictures only.

---

## 5. What is live-watchable (rule 13) and watched (rule 8)

Jobs page <jobs page, port 8096> lines, all written by the programs themselves:
`<run>_slam` (JOINED yes/no, links to the old map, SUSPECT flag), `<run>_mask` (MASK=1: pictures with people,
detector alive/stale), plus every drive line (`_camera`, `_bridge`, `_ekf_inputs`, `_svo`, `_media`, `_autostop`,
the run's own progress). Live map page <live map page, port 8095>: before the join it shows today's new sheet only
(the working graph, step f); at the join the whole floor appears. [INFERENCE for the page; on the bench the join came before the page had restarted]

---

## 6. What happens if something goes wrong

| situation | what to do | why it is safe |
|---|---|---|
| "JOINED no" after 60 s standing | drive on slowly along drive 10's route | nothing is lost: today's pictures are mapped from the first second (StartNewMapOnLoopClosure false) |
| never joins | park as usual; the file then holds two unconnected maps; report it as a failed S1 | old map intact (checked), new session saved on its own |
| `!! SUSPECT JOIN` | `touch ~/slam_series2/STOP_<run>`; restart on the mark with the next run id | the working copy is disposable; the master is never opened by RTAB-Map |
| camera crash | the camera guard restarts it (as drives) | as drives |
| WiFi drops in far corridors | keep driving; everything runs on the Jetson | as drives |
| detector dies (MASK=1) | the mask passes depth through; `_mask` line says DETECTOR STALE | mapping never waits on the detector |

---

## 7. People and moving objects (the user's scope addition, 27 Sept)

### 7.1 The chain

```
 ZED X ──► zed_node (NEURAL depth, GEN_2 tracker, people detector FAST, people only)
             │ rgb/image_rect_color ─────────────────────────────────────────────┐
             │ depth/depth_registered ──► depth_person_mask.py ──► depth_registered_masked
             │ obj_det/objects (boxes) ──────────┘   (depth = 0 inside each       │
             │                                        Person box + 10 % + 16 px)  │
             ▼                                                                    ▼
          SVO recording (raw, unmasked)                     rgbd_sync ──► rgbd_odometry (tracking)
                                                                    └───► rtabmap (recognition + map + grid)
```

*Plain terms: the camera marks where the people are; a small program blanks the distance values there; the map
program then treats those pixels as "nothing measured". The raw camera recording keeps everything, so the same
session can be replayed with the blanking on and off.*

- Why depth and not the colour picture: RTAB-Map's own rule on this build is "no depth = no feature"
  (`Vis/DepthAsMask`, `Mem/DepthAsMask` true - `rtabmap --params`), and the map and grid are made from depth. Blanking
  depth reaches all three consumers with one change and no RTAB-Map modification (ENGINEERING_NOTES.md section 5).
- Time matching: the boxes carry the camera's grab time stamp, the same as the depth picture's; the mask uses the
  closest box message at most 0.35 s older (or 0.1 s newer). No fresh boxes = depth passes unchanged, never a stall.
- Camera settings: a staged copy of `zedx_front.yaml` (`zedx_front_od.yaml`, first 185 lines byte-identical to the
  live file, guarded by its sha256) switches the detector on with the FAST model, people only, confidence 50,
  15 m range. The ACCURATE model (the wrapper's default) has no model file on this Jetson.
- One-time preparation, done on the bench 27 Sept: the ZED library optimises the detector for this graphics chip the
  first time it runs (**649 s with no pictures**). `od_bench.sh` did it; the start refuses MASK=1 if the optimised
  file is missing. Restart afterwards: 2 s.
- **What the detector did not see:** a person closer than about 1 m, head and shoulders only, was not boxed (bench
  03, BENCH_RESULTS.md section 3). At that range the camera has no depth anyway (`Vis/MinDepth` 1.0), so the map
  could not have used those pixels; but the tracking lost the picture for about 20 s.

### 7.2 Cost (measured, not assumed - ENGINEERING_NOTES.md section 5 compute rule)

BENCH_RESULTS.md section 2. In one line: with the full stack running (NEURAL depth, tracking, mapping, recording),
the map's tracking rate was **7.47 Hz with the detector + mask** against **7.04 Hz without the detector** and
**6.87 Hz with neither** - no measurable slow-down (N = 1 window of 60 s each, parked). The detector's cost is about
**+2.2 W** and +3 CPU points at camera level; the mask adds 5-28 ms per picture and keeps up at 14-15 pictures a
second. **Decision: MASK=1 for the session** (BENCH_RESULTS.md section 4).

### 7.3 Things that moved between sessions - ray tracing

Already on (`Grid/RayTracing` true since drive 1). How it shows: the live grid (map page) and the saved 2D map show
a cell as free once enough of today's rays pass through where something stood in drive 10 or earlier today; the 3D
cloud export does not (section 4 limits). No setting change proposed. What can be shown after the session: for the
place where the person stood (from `mask_events.csv` times and the robot position), count occupied grid cells in
the unmasked replay vs the masked one (PASS_LINES.md D1).

### 7.4 Offline ON-vs-OFF comparison (one variable, record once - replay twice)

The SVO recording is replayed twice through the same stack into two fresh copies of drive 10's map: once with
`mask:=false`, once with `mask:=true` (the detector runs on the replayed pictures both times; only the blanking
differs). Everything else is identical pictures, identical settings. Commands: RESULTS_PACK_PLAN.md section 4.
Measured: ghost points and occupied grid cells in the person's swept region, tracking disturbance while the person
passes, wrong loop closures (PASS_LINES.md D1-D3).
