Drive 10 camera map (s2_static_10.db, the whole 2nd floor) exported as a 3D point cloud, 2026-09-27 ~03:10 Hamilton, on the
Jetson, on ONE COPY of the database in ~/jobs/d10_export/ (deleted afterwards; the original was never opened for writing).

Method - same as drive 9 (s2_static_09/map_3d) and drive 6, plus --opt 2:
  rtabmap-export --cloud --poses --opt 2 --max_range 6 --decimation 4 --voxel 0.03   (run under nice -n 19; log: export.log)
  --opt 2 = use the optimised positions ALREADY SAVED in the database (Admin.opt_poses, written by RTAB-Map at the proper
  shutdown) instead of re-optimising; export.log: "860 optimized poses loaded". The camera depth of every graph node is
  placed at those positions, 3 cm voxel grid, normals.

Which positions (rule 20): the MAP'S CORRECTED positions (after loop closures), exactly RTAB-Map's own. Checked against the
results pack's ../camera_corrected.tum (db_corrected_tum.py, also Admin.opt_poses): 860 of 860 nodes, floor-plane difference
0.0 mm (identical). A first export pass WITHOUT --opt 2 (the default, a fresh full re-optimisation) was replaced: its positions
differed from the saved ones by median 61 mm, 95th percentile 170 mm, largest 201 mm on the floor plane - a different
optimisation of the same links, so not the map's saved result.

Files:
  drive10_map_3d.ply  140.0 MB, 4,517,220 points after the 3 cm voxel grid; x -7.4..22.1 m, y -21.1..17.4 m,
                      height -0.2..2.8 m (render.log). Open in MeshLab / CloudCompare. Kept on the Jetson; not for git.
  d10_3d.png          two 3D views (render.py = drive 9's script, title changed): camera colours; coloured by height.
  d10_topdown.png     2D top-down view (topdown.py): points below 2 m above the floor, coloured by height, with the robot
                      path from ../camera_corrected.tum drawn DASHED and labelled as the map's corrected positions. Points
                      outside the route are what the camera saw up to 6 m away (side corridors, doorways).
  export.log, render.log, export_poses.txt (t x y z qx qy qz qw id), render.py, topdown.py.
