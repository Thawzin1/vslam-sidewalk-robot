Drive 9 camera map (s2_static_09.db) exported as a 3D point cloud, 2026-09-27 ~05:20 Hamilton, on the Jetson,
on ONE COPY of the database in ~/jobs/d9_export/ (deleted afterwards; the original was never opened for writing).

Method - same as drive 6 (s2_static_06/map_3d) so the maps look alike:
  rtabmap-export --cloud --poses --max_range 6 --decimation 4 --voxel 0.03   (run under nice -n 19; log: export.log)
  = RTAB-Map's own full global optimisation of the saved graph (892 poses, 2084 links), then the camera depth
    of every graph node placed at those optimised positions, 3 cm voxel grid, normals.

Which positions (rule 20): the MAP'S CORRECTED positions (after loop closures). Drive 9's database was never
closed (Jetson power-cycled), so it has no Admin.opt_poses; rtabmap-export re-optimises the graph from the
database's own links. Checked against the results pack's re-optimised graph (../camera_corrected.tum, made by
tools/db/optimize_graph_se2.py): 892 nodes in common, floor-plane difference median 2.9 mm,
95th percentile 3.7 mm, largest 7.3 mm - well inside the 3 cm point spacing, so the cloud sits on the same
corrected positions as the results pack. (Node 1201, the last graph node, is in camera_corrected.tum but not in
the export's graph; one node at the end of the route.) The export's own poses are in export_poses.txt
(t x y z qx qy qz qw id). No database rewrite was needed (db_rewrite_poses.py not used).

Files:
  drive9_map_3d.ply   110.5 MB, 3,565,858 points (19.2 M before the 3 cm voxel grid); x -14.2..16.8 m,
                      y -20.1..7.4 m, height -0.2..2.8 m (render.log). Open in MeshLab / CloudCompare.
  d9_3d.png           two 3D views (render.py = drive 6's script, title changed): camera colours; coloured by height.
  d9_topdown.png      2D top-down view (topdown.py): points below 2 m above the floor, coloured by height, with the
                      robot path from ../camera_corrected.tum drawn DASHED and labelled as the map's corrected
                      positions (re-optimised graph). The 3D points outside the route are what the camera saw up to
                      6 m away (side corridors, doorways).
  export.log, render.log, export_poses.txt, render.py, topdown.py.
