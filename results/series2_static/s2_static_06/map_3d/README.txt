Drive 6 camera map (s2_static_06.db) exported as a 3D point cloud, 2026-09-26 Hamilton, on a COPY of the database (deleted afterwards).
rtabmap-export --cloud --max_range 6 --decimation 4 --voxel 0.03 (full global optimisation of the saved graph; camera depth) - same settings as drive 4.
4,835,503 points (export.log); extent x -13.5..13.2 m, y -32.0..7.2 m, height -0.2..2.8 m (render.log).
Open drive6_map_3d.ply in any 3D viewer (e.g. MeshLab, CloudCompare). d6_3d.png = two views (render.py, a copy of drive 4's with the title changed).
