#!/bin/bash
# record_lidar.sh - record the LiDAR packets for one drive.
#
# Lives on the CARD, not inside anyone else's package: this project changes
# nothing on the robot (ENGINEERING_NOTES.md rule 18).
#
# odometry, IMU and transforms compress to 4.6 MB/s, so a 20-minute drive is
# about 5.6 GB. The same drive stored as expanded clouds would be ~18 GB.
#
#   usage:  ./record_lidar.sh s2_static_01
#           Ctrl-C when the drive is finished - or, when started detached,
#           SIGINT to the `record` process (exact name), which closes the bag
#           properly. The comparison needs its wheel+IMU and LiDAR packets.
set -u
RUN="${1:?usage: record_lidar.sh <run_id>}"
D="/media/administrator/USB Drive/slam_series2"

set +u      # ROS's setup scripts read unset variables; strict mode aborts them over ssh
source /opt/ros/noetic/setup.bash
set -u
# The robot's .bashrc still says ROS_IP=<the old lab-WiFi robot address> - its MARC269 address,
# advertises an address it does not have is unreachable to anything on another
# machine. Local recording happens to survive it (tested: 10,475 packets in 6 s)
# because the drivers advertise the hostname - but this script does not rely on
# that. Unset it, and let ROS use the hostname like the drivers do.
unset ROS_IP
# The LIVE master, the one the sensors are publishing to. Never the replay one.
export ROS_MASTER_URI=http://localhost:11311

# stop the recording [ we will move the recording files, free up the space before the next
# drive ]"). Space is made BEFORE each drive by moving the last drive's files off, verified;
# the free space is printed below for the record and never stops the recording.
# The one check kept is that the card is THERE: if it is not mounted, "$D" does not exist and
FREE=$(df -BG --output=avail "$D" 2>/dev/null | tail -1 | tr -dc 0-9)
if [ -z "$FREE" ]; then
    echo "REFUSING: $D is not there - is the robot's card mounted?" >&2
    exit 1
fi

if [ -e "$D/${RUN}_lidar.bag" ]; then
    echo "REFUSING: $D/${RUN}_lidar.bag already exists." >&2
    echo "Pick a different run id rather than overwriting a drive." >&2
    exit 1
fi

echo "recording ${RUN}   (${FREE} GB free on the card)"
echo "press Ctrl-C when the drive is finished"
# (occupancy research report, section 5):
#   /ouster/imu_packets              the Ouster's own 100 Hz IMU - makes laser+IMU
#                                    methods testable offline later (~5 KB/s)
#   /husky_velocity_controller/odom  raw wheel odometry, before the robot's filter
#   /imu/data_raw                    raw UM7 readings - separates wheel heading from
#   /helios_pcl/scan /ouster_pcl/scan the robot's own live 2D scans (~0.6 Mbit/s each)
# A topic that does not exist is simply waited for by rosbag - it never stops
# the recording - but the names are checked with rostopic list before a drive.
exec rosbag record -O "$D/${RUN}_lidar.bag" --lz4 \
    /helios/packets /ouster/lidar_packets /ouster/imu_packets \
    /tf /tf_static /odometry/filtered /imu/data \
    /husky_velocity_controller/odom /imu/data_raw \
    /helios_pcl/scan /ouster_pcl/scan
