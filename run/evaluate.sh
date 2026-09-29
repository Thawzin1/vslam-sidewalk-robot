#!/bin/bash
# evaluate.sh - build a drive's results pack: numbers, figures, checks (and the LiDAR half when it exists).
#
#   usage:  run/evaluate.sh <run> [--lidar] [--series series2_static] [--db FILE] [--records DIR]
#                                 [--out DIR] [--from N] [--title "Drive 10"] [--export-3d]
#                                 [--tegra-start "YYYY-MM-DD HH:MM:SS" --tegra-end "..."]
#
# Camera half (default): closed-database check, the two camera paths (tracking alone; corrected by loop
# closures), two map pictures, the turns replayed from the recording, stream timing and freezes, the live
# timelapse stills, power and temperature, the link/bridge statistics and figures, WiFi freshness.
# --lidar adds: the LiDAR estimate from run/lidar_reference.sh copied in, the robot's own wheels, a flatness
# check, the trajectory figure with the LiDAR line, the agreement table and the size-fit check.
# Runs at the lowest priority (nice 19); writes only under --out and its work folder.
# Progress: <jobs>/pack_<run>.progress.  Details and every step: run/lib/pack_drive.sh --help.
# *Plain terms: turn one drive's raw files into the numbers and pictures the write-up quotes.*
set -euo pipefail
LIB="$(cd "$(dirname "$(readlink -f "$0")")/lib" && pwd)"
[ $# -ge 1 ] && [ "$1" != "-h" ] && [ "$1" != "--help" ] || { sed -n '2,15p' "$0" | sed 's/^# \{0,1\}//'; exit 1; }
exec nice -n 19 bash "$LIB/pack_drive.sh" "$@"
