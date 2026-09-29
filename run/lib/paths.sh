#!/bin/bash
# paths.sh - the one place that says where this repository keeps its files.
#
# Every launcher and helper sources this file instead of writing a folder path of its own.
# Override any value by exporting it before running a script (or in env.sh, see env.example.sh).
#
#   REPO_ROOT     this repository (found from this file's own location)
#   CATKIN_WS     the built ROS workspace (default: <repo>/catkin_ws)
#   RECORDS_DIR   one folder per run with its logs, checks and media (default ~/.run_records)
#   WORK_DIR      run databases and the small flag files PARK_<run>, STOP_<run>, NAV_PAUSE (default ~/slam_series2)
#   JOBS_DIR      one short ".progress" line per running job, read by the jobs page (default ~/jobs)
#   TOOLS_DIR     the drive-time helpers (camera guard, auto stop, bridge receiver ...)
#   DB_TOOLS_DIR  the map-database readers (db_to_tum.py, render_map.py ...)
#   RESULTS_DIR   the per-run results packs kept in the repository
#   SIDEWALK_ENV  the shell environment file that sets up ROS on this machine
#   *_ADDR        network addresses: never written in the code; set them in env.sh (see env.example.sh)
#   *_URL         the browser pages built from JETSON_ADDR
#
# *Plain terms: change a folder here once and every script follows.*
_PATHS_HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export REPO_ROOT="${REPO_ROOT:-$(cd "$_PATHS_HERE/../.." && pwd)}"
export SIDEWALK_REPO="${SIDEWALK_REPO:-$REPO_ROOT}"          # the ROS packages read this name
export CATKIN_WS="${CATKIN_WS:-$REPO_ROOT/catkin_ws}"
export RECORDS_DIR="${RECORDS_DIR:-$HOME/.run_records}"
export WORK_DIR="${WORK_DIR:-$HOME/slam_series2}"
export JOBS_DIR="${JOBS_DIR:-$HOME/jobs}"
export TOOLS_DIR="${TOOLS_DIR:-$REPO_ROOT/tools/drive}"
export DB_TOOLS_DIR="${DB_TOOLS_DIR:-$REPO_ROOT/tools/db}"
export PACK_TOOLS_DIR="${PACK_TOOLS_DIR:-$REPO_ROOT/tools/results_pack}"
export RESULTS_DIR="${RESULTS_DIR:-$REPO_ROOT/results}"
export LIDAR_BAG_DIR="${LIDAR_BAG_DIR:-$HOME/lidar_bags}"
export LIDAR_REPLAY_DIR="${LIDAR_REPLAY_DIR:-$HOME/lidar_replays}"
export SIDEWALK_ENV="${SIDEWALK_ENV:-$HOME/.sidewalk_env.sh}"
# a private env.sh (not committed) may set the addresses and other machine-specific values
[ -r "$REPO_ROOT/env.sh" ] && . "$REPO_ROOT/env.sh"
# addresses: empty until env.sh sets them (see env.example.sh; the lab-network addresses: ask Prof. Mehrtash or the lab)
export JETSON_ADDR="${JETSON_ADDR:-localhost}"
export ROBOT_WIFI_ADDR="${ROBOT_WIFI_ADDR:-}"
export ROBOT_USB_ADDR="${ROBOT_USB_ADDR:-}"
export JETSON_USB_ADDR="${JETSON_USB_ADDR:-}"
export LIVE_MAP_URL="${LIVE_MAP_URL:-http://$JETSON_ADDR:8095/}"
export JOBS_PAGE_URL="${JOBS_PAGE_URL:-http://$JETSON_ADDR:8096/}"
export STORAGE_PAGE_URL="${STORAGE_PAGE_URL:-http://$JETSON_ADDR:8092/}"
export LIVE_SITE_URL="${LIVE_SITE_URL:-http://$JETSON_ADDR:8097/}"
export LIVE_SITE_KEY_FILE="${LIVE_SITE_KEY_FILE:-$HOME/.config/live_site_key}"
mkdir -p "$JOBS_DIR" "$RECORDS_DIR" 2>/dev/null || true
unset _PATHS_HERE
