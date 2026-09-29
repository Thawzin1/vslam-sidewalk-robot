# depth_benchmark/ - the camera depth-accuracy study (a separate study), kept here as ROS package code

These scripts measure how accurately the ZED X camera reports distance: spheres, flat boards and
panels at known ranges, replayed camera recordings in every depth mode, and the figures for that
study. They belong to the ZED X vs LiDAR camera depth study, not to the
navigation work in the rest of this repository. They stay because they are part of this ROS package
(launch files such as `sphere_centroid_zedx.launch` and `depth_characterization.launch` start them)
and because `catkin_make` installs them.

They import `eval_common.py` and `traj_metrics.py` as sibling modules; the two symbolic links in this
folder point at the copies one level up, so the imports keep working from the source tree. In the
installed workspace all scripts land in one folder, so nothing changes there.

Nothing in `run/` uses these scripts.
