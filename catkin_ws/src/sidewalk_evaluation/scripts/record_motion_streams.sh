#!/usr/bin/env bash
# record_motion_streams.sh - record the fast motion data a mapping run throws away.
#
# WHY THIS EXISTS
#
#   The map database stores about ONE position per second. Three questions we
#   want answered live faster than that and are therefore invisible in it:
#
#     1. Does the robot shake while driving? Shaking happens maybe 5-20 times a
#        second. One sample per second cannot see it, the same way a clock's
#        second hand cannot tell you how fast a fly's wings beat.
#     2. Is the camera bolted on straight? Measured by comparing what the WHEELS
#        say the robot did against what the CAMERA says it did, over a straight
#        stretch. Both accounts have to be at full rate to line up.
#     3. Would combining the camera with the wheels give a better answer than
#        either alone? That comparison needs both streams recorded together.
#
#   All three need the SAME recording. This script makes it, and it adds no
#   driving time at all - it just listens while a normal run happens.
#
# WHY TWO SEPARATE RECORDINGS
#
#   The camera runs on the Jetson and its ROS master. The wheels and the robot's
#   motion sensors run on the ROBOT's computer and a DIFFERENT ROS master. One
#   recorder cannot see both. So this makes two files, one per machine, and they
#   are lined up afterwards by their timestamps - which is why
#   clock_offset_check.py must pass BEFORE this is worth running.
#
# HOW BIG
#
#   Positions and motion readings only, no pictures. Expect a few MB per minute,
#   against the map database's ~340 MB per minute. It goes on the Jetson's
#   INTERNAL disk, not the microSD card, because the card has damaged seven
#   things and these files are small enough not to need its capacity.
#
#   usage:
#     record_motion_streams.sh lab_map_08
#     record_motion_streams.sh lab_map_08 --robot-only    # if the camera is not up yet
#
#   Stop it with Ctrl+C when the drive ends. It reports what it actually caught.

set -o pipefail

RUN_ID="${1:?usage: record_motion_streams.sh <run_id> [--robot-only|--jetson-only]}"
MODE="${2:-both}"

source /opt/ros/noetic/setup.bash
source "$CATKIN_WS/devel/setup.bash" 2>/dev/null
set -u                      # AFTER sourcing - ROS setup files read unset vars

OUT="$RECORDS_DIR/${RUN_ID}/motion"
mkdir -p "$OUT"

JETSON_MASTER="http://localhost:11311"
ROBOT_MASTER="http://$ROBOT_USB_ADDR:11311"
ROBOT_IP="$JETSON_USB_ADDR"

# --- disk guard. A full internal disk took this machine off the network on
FREE_GB=$(df --output=avail -BG / | tail -1 | tr -dc '0-9')
if [ "${FREE_GB:-0}" -lt 8 ]; then
  echo "  REFUSING TO START: only ${FREE_GB} GB free on the internal disk."
  echo "  A full disk kills a run mid-write and has taken this Jetson down before."
  exit 1
fi
echo "  internal disk free: ${FREE_GB} GB"

# --- Count PUBLISHERS, never trust a topic listing. A dead node's registration
# --- lingers on the master, so 'rostopic list' has reported topics nobody
# --- publishes and sent a script down its "everything is fine" branch.
have_publisher() {         # $1 = topic, uses whatever ROS_MASTER_URI is set
  local n
  n=$(timeout 8 rostopic info "$1" 2>/dev/null | sed -n '/Publishers/,/Subscribers/p' | grep -c ' \* ')
  [ "${n:-0}" -gt 0 ]
}

pick_live() {              # echoes the subset of "$@" that genuinely has a publisher
  local live=""
  for t in "$@"; do
    if have_publisher "$t"; then live="$live $t"; echo "    live:    $t" >&2
    else                          echo "    absent:  $t" >&2; fi
  done
  echo "$live"
}

PIDS=()
cleanup() {
  echo
  echo "  stopping recorders ..."
  for p in "${PIDS[@]:-}"; do kill -INT "$p" 2>/dev/null || true; done
  sleep 3
  echo
  echo "  === what was actually caught ==="
  local any=0
  for f in "$OUT"/*.bag; do
    [ -e "$f" ] || continue
    any=1
    sz=$(stat -c %s "$f")
    printf "    %8.2f MB  %s\n" "$(echo "$sz" | awk '{print $1/1048576}')" "$(basename "$f")"
    # Size matters, not existence - a zero-byte file has passed as success here.
    if [ "$sz" -lt 5000 ]; then
      echo "               ^^^ SUSPICIOUSLY SMALL - probably caught nothing"
    else
      timeout 60 rosbag info "$f" 2>/dev/null | sed -n '/topics:/,$p' | head -8 | sed 's/^/               /'
    fi
  done
  [ "$any" = 0 ] && echo "    NO BAG FILES AT ALL. Nothing was recorded."
  echo
  echo "  files in $OUT"
  exit 0
}
trap cleanup INT TERM

# ---------------------------------------------------------------- Jetson side
if [ "$MODE" != "--robot-only" ]; then
  export ROS_MASTER_URI="$JETSON_MASTER"
  unset ROS_IP 2>/dev/null || true
  echo
  echo "  === JETSON master ($JETSON_MASTER) - the camera's own account ==="
  J_TOPICS=$(pick_live \
      /rtabmap/odom \
      /rtabmap/odom_info \
      /zedx_front/zed_node/imu/data \
      /zedx_front/zed_node/odom \
      /zedx_front/zed_node/pose \
      /tf /tf_static)
  if [ -n "${J_TOPICS// /}" ]; then
    # shellcheck disable=SC2086
    rosbag record -O "$OUT/${RUN_ID}_jetson.bag" --lz4 $J_TOPICS \
        > "$OUT/jetson_record.log" 2>&1 &
    PIDS+=($!)
    echo "    recording -> ${RUN_ID}_jetson.bag"
  else
    echo "    NOTHING LIVE on the Jetson master. Is the camera and mapping stack up?"
  fi
fi

# ----------------------------------------------------------------- robot side
if [ "$MODE" != "--jetson-only" ]; then
  export ROS_MASTER_URI="$ROBOT_MASTER" ROS_IP="$ROBOT_IP"
  echo
  echo "  === ROBOT master ($ROBOT_MASTER) - the wheels' independent account ==="
  R_TOPICS=$(pick_live \
      /odometry/filtered \
      /husky_velocity_controller/odom \
      /imu/data \
      /imu_um7/data \
      /imu/data_raw \
      /joint_states \
      /cmd_vel)
  if [ -n "${R_TOPICS// /}" ]; then
    # shellcheck disable=SC2086
    rosbag record -O "$OUT/${RUN_ID}_robot.bag" --lz4 $R_TOPICS \
        > "$OUT/robot_record.log" 2>&1 &
    PIDS+=($!)
    echo "    recording -> ${RUN_ID}_robot.bag"
  else
    echo "    NOTHING LIVE on the robot master. Is the robot powered on and"
    echo "    off the charger? Check: ROS_MASTER_URI=$ROBOT_MASTER rostopic list"
  fi
fi

if [ "${#PIDS[@]}" -eq 0 ]; then
  echo
  echo "  Nothing to record. Exiting rather than pretending to work."
  exit 2
fi

echo
echo "  RECORDING. Drive the run. Press Ctrl+C here when the drive is finished."
echo "  For the camera-angle measurement, include at least one stretch of"
echo "  10 metres driven as straight as you can manage, at a steady speed."
echo

# Report on CHANGE, not on a timer, and notice a recorder dying silently.
last=""
while true; do
  sleep 30
  alive=0
  for p in "${PIDS[@]}"; do kill -0 "$p" 2>/dev/null && alive=$((alive+1)); done
  sz=$(du -sm "$OUT" 2>/dev/null | cut -f1)
  free=$(df --output=avail -BG / | tail -1 | tr -dc '0-9')
  now="${alive}|${sz}|${free}"
  if [ "$now" != "$last" ]; then
    echo "    recorders alive ${alive}/${#PIDS[@]} | ${sz} MB captured | ${free} GB free"
    [ "$alive" -lt "${#PIDS[@]}" ] && echo "    *** A RECORDER HAS DIED - see $OUT/*_record.log"
    [ "${free:-99}" -lt 5 ] && echo "    *** DISK NEARLY FULL - stop the run"
    last="$now"
  fi
done
