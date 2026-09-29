#!/bin/bash
# pull_bag_from_robot.sh - copy ONE drive's LiDAR recording off the robot card onto the Jetson, sha256-checked.
#
#   robot card (original, stays there)                     Jetson internal disk (a working copy)
#   /media/administrator/USB Drive/slam_series2/<run>_lidar.bag  ->  ~/lidar_bags/<run>_lidar.bag (+ .sha256)
#
# Over the USB link by default (ssh alias robot-usb, $ROBOT_USB_ADDR, ~26 MB/s: 9.2 GB in about 6 min);
# pass "robot" as the 2nd argument for WiFi. rsync -s because the card's path has a space (rule 17).
# The original is never touched. The copy is a BIG WORKING COPY: one at a time,
# never started if it would leave under 3 GB free, deleted once its replay is validated - the robot card
# (and the Autonomous Service Robot Teams folder) holds the original.
#
# Refuses when: a drive / localisation session is live on the Jetson (rule 23), HELPERS_STOP exists, the robot
# is recording or replaying (whole-name process match on the robot, rule 8), the recording was written in the
# last 2 minutes, or the space is short. Checks: sha256 computed ON THE ROBOT and ON THE JETSON must match.
#
#   usage: ./pull_bag_from_robot.sh <run> [robot-usb|robot]
#   progress: $JOBS_DIR/<run>_lidar_bag_pull.progress (jobs page :8096)
# repository folders and addresses (REPO_ROOT, RECORDS_DIR, WORK_DIR, JOBS_DIR, TOOLS_DIR, *_ADDR): run/lib/paths.sh
. "$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)/../paths.sh"
set -u
RUN="${1:?usage: pull_bag_from_robot.sh <run> [robot-usb|robot]}"
HOST="${2:-robot-usb}"
[[ "$RUN" =~ ^[A-Za-z0-9_]+$ ]] || { echo "run id must be letters, digits, _" >&2; exit 2; }
SRC="/media/administrator/USB Drive/slam_series2/${RUN}_lidar.bag"
DIR="${LIDAR_BAG_DIR:-$HOME/lidar_bags}"
DST="$DIR/${RUN}_lidar.bag"
FLOOR_GB=3
STOPFILE="$WORK_DIR/HELPERS_STOP"
PROG="$JOBS_DIR/${RUN}_lidar_bag_pull.progress"
TAG="${RUN}_LIDAR_BAG_PULL"
mkdir -p "$JOBS_DIR" "$DIR"
say() { echo "$1" > "$PROG.tmp" && mv "$PROG.tmp" "$PROG"; echo "$(TZ=America/Toronto date +%H:%M:%S) $1"; }
fail() { say "$TAG  FAILED - $1"; exit 1; }

[ -e "$STOPFILE" ] && fail "$STOPFILE exists - not starting"
for pn in $(ps -eo comm=); do
    case "$pn" in start_drive.sh|rgbd_odometry|stereo_odometry|rtabmap) fail "'$pn' is running on the Jetson (a drive, a localisation session or a replay) - not copying now" ;; esac
done
if [ -s "$DST" ] && [ -s "$DST.sha256" ]; then
    say "$TAG  already here: $DST (sha256 $(cut -c1-16 "$DST.sha256")...) - nothing to do"
    exit 0
fi
[ -e "$DST" ] && fail "$DST exists without a .sha256 (an unfinished copy?) - rename or remove it first"

R=$(timeout 30 ssh "$HOST" "stat -c '%s %Y' \"$SRC\" 2>/dev/null || echo MISSING; date +%s; ps -eo comm=" 2>&1) \
    || fail "the robot does not answer on '$HOST' ($R)"
[ "$(echo "$R" | head -1)" = MISSING ] && fail "no $SRC on the robot"
SIZE=$(echo "$R" | sed -n 1p | cut -d' ' -f1); MT=$(echo "$R" | sed -n 1p | cut -d' ' -f2); NOW=$(echo "$R" | sed -n 2p)
[ $(( NOW - MT )) -ge 120 ] || fail "the recording was written $(( NOW - MT )) s ago - is the robot still recording?"
for pn in $(echo "$R" | tail -n +3); do
    case "$pn" in record_lidar.sh|rosbag|record|replay_lidar.sh|replay_colleagu|robot_side.sh)
        fail "'$pn' is running on the robot (recording or replaying) - copy afterwards" ;;
    esac
done
FREE=$(df -B1 --output=avail "$DIR" | tail -1 | tr -dc 0-9)
LEFT=$(( (FREE - SIZE) / 1073741824 ))
[ $(( FREE - SIZE )) -ge $(( FLOOR_GB * 1073741824 )) ] \
    || fail "the copy ($(( SIZE / 1048576 )) MB) would leave ${LEFT} GB free on the Jetson; the floor for a working copy is ${FLOOR_GB} GB (free space first: see the disk page, port 8092)"

say "$TAG  0/$(( SIZE / 1048576 )) MB  copying over $HOST  0s"
T0=$(date +%s)
nice -n 19 rsync -s -t --partial-dir=.rsync-partial "$HOST:$SRC" "$DIR/" > "$DIR/.${RUN}_pull.log" 2>&1 &
RS=$!
while [ -d "/proc/$RS" ] && ! grep -q '^State:.*Z' "/proc/$RS/status" 2>/dev/null; do
    [ -e "$STOPFILE" ] && { kill -INT "$RS"; fail "$STOPFILE appeared - copy interrupted (resumable: re-run)"; }
    GOT=$(du -cb "$DIR"/.${RUN}_lidar.bag.* "$DIR/.rsync-partial/${RUN}_lidar.bag" "$DST" 2>/dev/null | tail -1 | cut -f1)
    say "$TAG  $(( ${GOT:-0} / 1048576 ))/$(( SIZE / 1048576 )) MB  copying over $HOST  $(( $(date +%s) - T0 ))s" > /dev/null
    sleep 20
done
wait "$RS"; RC=$?
[ "$RC" = 0 ] || fail "rsync exit $RC - see $DIR/.${RUN}_pull.log (re-run resumes)"
[ "$(stat -c %s "$DST")" = "$SIZE" ] || fail "size differs after the copy: $(stat -c %s "$DST") here, $SIZE on the robot"

say "$TAG  $(( SIZE / 1048576 ))/$(( SIZE / 1048576 )) MB  copied in $(( $(date +%s) - T0 )) s; sha256 on both machines  $(( $(date +%s) - T0 ))s"
( timeout 1800 ssh "$HOST" "nice -n 19 sha256sum \"$SRC\"" > "$DIR/.${RUN}_remote.sha256" 2>&1 ) &
RSH=$!
LOCAL=$(nice -n 19 sha256sum "$DST" | cut -c1-64)
wait "$RSH"
REMOTE=$(cut -c1-64 "$DIR/.${RUN}_remote.sha256")
if [ -n "$LOCAL" ] && [ "$LOCAL" = "$REMOTE" ]; then
    echo "$LOCAL  ${RUN}_lidar.bag" > "$DST.sha256"
    echo "copied $(TZ=America/Toronto date '+%Y-%m-%d %H:%M %Z') from $HOST:$SRC ($SIZE bytes), sha256 equal on the robot and the Jetson" > "$DST.source.txt"
    rm -f "$DIR/.${RUN}_remote.sha256" "$DIR/.${RUN}_pull.log"
    say "$TAG  complete  $(( SIZE / 1048576 )) MB, sha256 ${LOCAL:0:16}... equal on both machines  $(( $(date +%s) - T0 ))s"
    exit 0
fi
mv "$DST" "$DST.MISMATCH-$(date +%s)"
fail "sha256 differs: Jetson $LOCAL, robot ${REMOTE:-<none: $(head -c 200 "$DIR/.${RUN}_remote.sha256")>} - the copy was renamed .MISMATCH-*, not used"
