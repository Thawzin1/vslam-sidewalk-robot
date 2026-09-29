#!/bin/bash
# robot_side.sh - the robot's half of a drive: start or stop, in one command.
#
#   start <run>  re-zero the gyro if the wheels are still (gyro_zero_now.py), then
#                the LiDAR recording (record_lidar.sh, packets + wheel/IMU)
#                and the live LiDAR view for the browser (lidar_live_map.py),
#                for drive 3's fusion): it copies the wheels, the gyroscope and the
#                robot's own filter to the Jetson over one TCP connection on port 8111.
#   stop  <run>  interrupt the recording's `record` process so the bag is
#                closed properly, stop the live view, and LAST stop the bridge
#                sender (so the Jetson's map is never starved of wheels before
#                it has closed - run this after "park" has closed the map)
#   selftest -   the process matcher against the running sender AND against a
#                name known to be absent (ENGINEERING_NOTES.md rule 8); starts nothing
#   storage  -   start the storage reporter (robot_storage_agent.py, port 8113) if it
#                is not already running; `start` also does this, as its last step.
#                READ-ONLY: it tells the Jetson's storage page ($STORAGE_PAGE_URL)
#                how full this robot's disk and card are, and writes nothing but its
#                own log. Left running after a drive - it costs almost nothing.
#
# The bridge sender PUBLISHES NOTHING on the robot and changes no file or setting
# of the robot's: it only listens, like `rostopic echo`. *Plain terms: it can look
# at the robot's measurements, never touch the robot.*
#
# Runs ON THE ROBOT, from its card. All jobs are detached (setsid nohup), so a
# dropped connection changes nothing. Process checks compare WHOLE names and
# arguments, never substrings (ENGINEERING_NOTES.md rule 8).
#
#   usage:  robot_side.sh start s2_static_03
#           robot_side.sh stop  s2_static_03
#           robot_side.sh selftest -
# repository folders and addresses (REPO_ROOT, RECORDS_DIR, WORK_DIR, JOBS_DIR, TOOLS_DIR, *_ADDR): run/lib/paths.sh
. "$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)/../../run/lib/paths.sh"
set -u
ACT="${1:?usage: robot_side.sh start|stop <run>}"
RUN="${2:?usage: robot_side.sh start|stop <run>}"
T="/media/administrator/USB Drive/slam_series2/tools"
mkdir -p "$JOBS_DIR"

live_pids() {
    local d
    for d in /proc/[0-9]*; do
        mapfile -d '' -t A < "$d/cmdline" 2>/dev/null || continue
        [ "${#A[@]}" -ge 2 ] && [ "$(basename -- "${A[0]}")" = python3 ] \
            && [ "$(basename -- "${A[1]}")" = lidar_live_map.py ] && echo "${d#/proc/}"
    done
}

# pids_of NAME - PIDs of `python3 .../NAME ...`: argv[0]'s base name must be EXACTLY
# python3 and argv[1]'s EXACTLY NAME. Whole names, never "contains" (a substring match
# reported dead jobs as alive four times in this project). Skips this shell and its parent.
pids_of() {
    local want="$1" d pid
    for d in /proc/[0-9]*; do
        pid="${d#/proc/}"
        { [ "$pid" = "$$" ] || [ "$pid" = "$PPID" ]; } && continue
        mapfile -d '' -t A < "$d/cmdline" 2>/dev/null || continue
        [ "${#A[@]}" -ge 2 ] && [ "$(basename -- "${A[0]}")" = python3 ] \
            && [ "$(basename -- "${A[1]}")" = "$want" ] && echo "$pid"
    done
}
BRIDGE_PORT=8111
STORAGE_PORT=8113
# start_storage_agent - the storage reporter, started once: a second copy is never
start_storage_agent() {
    local p
    p=$(pids_of robot_storage_agent.py | tr '\n' ' ')
    if [ -n "$p" ]; then
        echo "   storage reporter: already running (pid ${p% })"
        return 0
    fi
    setsid nohup python3 "$T/robot_storage_agent.py" --port "$STORAGE_PORT" \
        >> "$JOBS_DIR/robot_storage_agent.log" 2>&1 < /dev/null &
    sleep 2
    if curl -s -m 2 "http://127.0.0.1:${STORAGE_PORT}/storage.json" > /dev/null; then
        echo "   storage reporter: serving on :$STORAGE_PORT (pid $(pids_of robot_storage_agent.py | tr '\n' ' '))"
    else
        echo "   !! storage reporter NOT answering on :$STORAGE_PORT - see $JOBS_DIR/robot_storage_agent.log"
        echo "      (nothing else is affected; the Jetson's storage page will show the robot as offline)"
    fi
}

case "$ACT" in
start)
    # Every topic the recorder asks for, checked against the live master first:
    # rosbag waits silently for a missing topic, so a wrong name would only be
    set +u; source /opt/ros/noetic/setup.bash; set -u
    export ROS_MASTER_URI=http://localhost:11311; unset ROS_IP
    LIVE=$(timeout 10 rostopic list 2>/dev/null)
    MISSING=""
    for t in /helios/packets /ouster/lidar_packets /ouster/imu_packets /tf /tf_static \
             /odometry/filtered /imu/data /husky_velocity_controller/odom /imu/data_raw \
             /helios_pcl/scan /ouster_pcl/scan; do
        echo "$LIVE" | grep -qx "$t" || MISSING="$MISSING $t"
    done
    [ -n "$MISSING" ] && echo "   note - not on the live master, will be absent from the bag:$MISSING" \
                      || echo "   all recorded topics present on the live master"
    # Re-zero the gyro on the start mark, before anything is recorded (user's decision
    python3 "$T/gyro_zero_now.py" "/media/administrator/USB Drive/slam_series2/${RUN}_gyro_zero.json" \
        || echo "   !! gyro NOT re-zeroed (see above) - the drive can go on; the correction still works"
    setsid nohup bash "$T/record_lidar.sh" "$RUN" > "$JOBS_DIR/${RUN}_lidar_record.out" 2>&1 < /dev/null &
    sleep 6
    B="/media/administrator/USB Drive/slam_series2/${RUN}_lidar.bag.active"
    S1=$(stat -c %s "$B" 2>/dev/null || echo 0); sleep 4; S2=$(stat -c %s "$B" 2>/dev/null || echo 0)
    if [ "$S2" -gt "$S1" ]; then
        echo "   LiDAR recording: growing ($(( (S2 - S1) / 4000000 )) MB/s)"
    else
        echo "   !! LiDAR recording NOT growing - see $JOBS_DIR/${RUN}_lidar_record.out"
    fi
    for p in $(live_pids); do kill -TERM "$p"; done      # a view left from before
    setsid nohup python3 "$T/lidar_live_map.py" _run:="$RUN" \
        > "$JOBS_DIR/${RUN}_lidar_live.log" 2>&1 < /dev/null &
    sleep 8
    curl -s -m 3 http://127.0.0.1:8095/stats.json > /dev/null \
        && echo "   live LiDAR view: serving on :8095" \
        || echo "   !! live LiDAR view not answering - see $JOBS_DIR/${RUN}_lidar_live.log (recording unaffected)"
    # --- the bridge sender, LAST (drive 3 fusion). Read-only clock check first: the
    # robot's clock battery is dead, so until its time program has locked to internet
    # time its clock can JUMP mid-drive. The bridge catches a jump, but waiting is better.
    if ntpq -pn 2>/dev/null | grep -q '^\*' && [ "$(date +%Y)" -ge 2026 ]; then
        echo "   robot clock: locked to internet time"
    else
        echo "   !! robot clock NOT settled (no '*' peer in ntpq -pn, or the year is wrong) - a clock"
        echo "      jump may happen mid-drive; the bridge catches it, but waiting 2-3 min is better"
    fi
    for p in $(pids_of robot_bridge_send.py); do kill -TERM "$p"; done     # one left from before
    sleep 1
    setsid nohup python3 "$T/robot_bridge_send.py" __name:=robot_bridge_send \
        _run:="$RUN" _port:="$BRIDGE_PORT" \
        > "$JOBS_DIR/${RUN}_bridge_send.log" 2>&1 < /dev/null &
    sleep 3
    if [ -n "$(pids_of robot_bridge_send.py)" ] \
            && ss -ltn | awk '{print $4}' | grep -qE "(^|:)${BRIDGE_PORT}\$"; then
        echo "   bridge sender: listening on :$BRIDGE_PORT (pid $(pids_of robot_bridge_send.py | tr '\n' ' '))"
    else
        echo "   !! bridge sender NOT listening on :$BRIDGE_PORT - see $JOBS_DIR/${RUN}_bridge_send.log"
        echo "      (LiDAR recording unaffected; a FUSION=1 drive on the Jetson will refuse to start)"
    fi
    # the storage reporter for the Jetson's storage page, after everything else
    start_storage_agent
    ;;
storage)
    start_storage_agent
    ;;
stop)
    P=$(ps -eo pid=,comm= | awk '$2 == "record" {print $1}')
    [ -n "$P" ] && kill -INT $P
    for i in $(seq 1 30); do [ -z "$(ps -eo comm= | grep -x record)" ] && break; sleep 2; done
    for p in $(live_pids); do kill -TERM "$p"; done
    B="/media/administrator/USB Drive/slam_series2/${RUN}_lidar.bag"
    if [ -s "$B" ] && [ ! -e "$B.active" ]; then
        echo "   LiDAR recording closed: $(du -h "$B" | cut -f1)"
    else
        echo "   !! the recording did not close properly - look before doing anything else"
    fi
    echo "   live view stopped: $([ -z "$(live_pids)" ] && echo yes || echo NO)"
    # the bridge sender LAST, after the bag has closed (see the header)
    for p in $(pids_of robot_bridge_send.py); do kill -TERM "$p"; done
    for i in $(seq 1 10); do [ -z "$(pids_of robot_bridge_send.py)" ] && break; sleep 1; done
    echo "   bridge sender stopped: $([ -z "$(pids_of robot_bridge_send.py)" ] && echo yes || echo NO)"
    ;;
selftest)
    # ENGINEERING_NOTES.md rule 8: a process check only ever asked about running things passes every
    # test while being permanently broken - so it is also asked about a name that is ABSENT.
    RUNNING=$(pids_of robot_bridge_send.py | tr '\n' ' ')
    ABSENT=$(pids_of no_such_script_7f3a.py | tr '\n' ' ')
    echo "   robot_bridge_send.py -> [${RUNNING% }]"
    echo "   no_such_script_7f3a.py -> [${ABSENT% }]"
    if [ -z "$ABSENT" ]; then echo "   matcher OK on an absent name"; else echo "   !! matcher BROKEN"; exit 1; fi
    ;;
*) echo "usage: robot_side.sh start|stop|selftest <run>" >&2; exit 2;;
esac
