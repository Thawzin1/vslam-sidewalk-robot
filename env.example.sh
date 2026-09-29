#!/bin/bash
# env.example.sh - copy to env.sh (which is NOT committed) and fill in your machines' values.
# run/lib/paths.sh reads env.sh; every script gets these as environment variables.
# Nothing here is secret in itself, but the real addresses stay out of the public repository.
# The robot's and the Jetson's addresses on the lab network: ask Prof. Mehrtash or the lab.

# ROS on this machine (the Jetson): sourced by every drive script. The original file did
#   source /opt/ros/noetic/setup.bash; source <catkin_ws>/devel/setup.bash
#   export ROS_IP=127.0.0.1; export ROS_MASTER_URI=http://${ROS_IP}:11311
#   export SIDEWALK_REPO="<this repository>"; CUDA on PATH/LD_LIBRARY_PATH
export SIDEWALK_ENV="$HOME/.sidewalk_env.sh"

# folders (defaults shown; change if your disk layout differs)
export RECORDS_DIR="$HOME/.run_records"     # one folder per run: logs, checks, media
export WORK_DIR="$HOME/slam_series2"        # run databases and the PARK_/STOP_/NAV_PAUSE flag files
export JOBS_DIR="$HOME/jobs"                # the one-line .progress files the jobs page reads
export LIDAR_BAG_DIR="$HOME/lidar_bags"     # LiDAR recordings pulled off the robot (big working copies)
export LIDAR_REPLAY_DIR="$HOME/lidar_replays"

# addresses (fill in; leave empty to run camera-only)
export JETSON_ADDR=""            # the Jetson as seen from your own computer or phone (its lab-network address)
export ROBOT_WIFI_ADDR=""        # the robot computer's WiFi address (DHCP: check it each session)
export ROBOT_USB_ADDR=""         # the robot end of the Jetson<->robot USB-C link
export JETSON_USB_ADDR=""        # the Jetson end of that link
export ROBOT_SSH="robot"         # ssh alias for the robot's computer, from the Jetson

# the live site (optional): the view-key file and the push settings file are private
export LIVE_SITE_KEY_FILE="$HOME/.config/live_site_key"
