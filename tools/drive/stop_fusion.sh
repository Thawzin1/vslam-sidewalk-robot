#!/bin/bash
# stop_fusion.sh - stops a FUSION=1 drive's blend pieces, in the safe order, by process number.
# RUNS ON: the Jetson (deployed to $TOOLS_DIR/ with start_drive.sh).
#
#   usage:  stop_fusion.sh <run_id>               stop them now
#           stop_fusion.sh <run_id> --after-map   first wait until the mapping (rtabmap) has
#                                                 exited, then stop them - start_drive.sh starts
#                                                 this detached, so "park" ends everything
#           stop_fusion.sh --selftest             the process check against a name known to
#                                                 be ABSENT must answer "not running"
#
# ORDER (DESIGN.md section 6.1), each found by the number saved at start in
# $RECORDS_DIR/<run>/fusion_pids, and each re-checked by its WHOLE program name before any
# signal is sent (a saved number can be reused by an unrelated program after a restart):
#   1. the fusion recording (rosbag "record")  SIGINT, then wait for fusion.bag.active to go
#   2. the blend (roslaunch fused_odometry)     SIGINT: roslaunch stops the EKF and conditioner
#   3. the bridge receiver                      SIGTERM
# Plain terms: close the recording first so its file is complete, then switch the blend off,
# then the link to the robot. The map itself is closed by auto_stop_mapping.py, never here.
#
# Never pgrep -f / pkill -f (ENGINEERING_NOTES.md rule 8): names are compared as WHOLE arguments.
# repository folders and addresses (REPO_ROOT, RECORDS_DIR, WORK_DIR, JOBS_DIR, TOOLS_DIR, *_ADDR): run/lib/paths.sh
. "$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)/../../run/lib/paths.sh"
set -u
[ "${1:-}" = "--selftest" ] && SELFTEST=1 || SELFTEST=0

# is_prog PID WANT: yes if PID is running WANT, by whole names only.
#   WANT "record"                -> comm is exactly "record"
#   WANT "roslaunch"             -> argv[1] basename is exactly "roslaunch" (python3 script)
#   WANT "robot_bridge_recv.py"  -> argv[1] basename is exactly that (python3 script)
is_prog() {
    local pid="$1" want="$2" A comm
    [ -n "$pid" ] && [ -r "/proc/$pid/cmdline" ] || { echo no; return; }
    comm=$(cat "/proc/$pid/comm" 2>/dev/null)
    if [ "$want" = record ]; then [ "$comm" = record ] && echo yes || echo no; return; fi
    mapfile -d '' -t A < "/proc/$pid/cmdline" 2>/dev/null || { echo no; return; }
    [ "${#A[@]}" -ge 2 ] && [ "$(basename -- "${A[1]}")" = "$want" ] && echo yes || echo no
}

if [ "$SELFTEST" = 1 ]; then
    r1=$(is_prog "$$" no_such_script_7f3a.py)
    r2=$(is_prog "$$" record)
    echo "absent name: $r1 (must be no); this shell as 'record': $r2 (must be no)"
    [ "$r1" = no ] && [ "$r2" = no ] && exit 0 || exit 1
fi

RUN="${1:?usage: stop_fusion.sh <run_id> [--after-map] | --selftest}"
REC="$RECORDS_DIR/$RUN"
PIDS="$REC/fusion_pids"
say() { echo "$(TZ=America/Toronto date '+%F %T %Z') $*"; }
[ -r "$PIDS" ] || { say "no $PIDS - nothing recorded as started for $RUN"; exit 2; }
# shellcheck disable=SC1090
source "$PIDS"          # sets RECORD_PID, FUSED_PID, BRIDGE_PID

if [ "${2:-}" = "--after-map" ]; then
    say "waiting for the mapping (rtabmap) to exit before stopping the blend"
    while ps -eo comm= | awk '$1 == "rtabmap" {f=1} END {exit !f}'; do sleep 5; done
    say "the mapping has exited"
fi

stop_one() {   # name pid want signal
    local name="$1" pid="$2" want="$3" sig="$4"
    if [ "$(is_prog "$pid" "$want")" != yes ]; then
        say "$name: pid ${pid:-none} is not running '$want' - nothing to stop"
        return 0
    fi
    kill "-$sig" "$pid"
    for i in $(seq 1 60); do [ "$(is_prog "$pid" "$want")" = yes ] || break; sleep 1; done
    if [ "$(is_prog "$pid" "$want")" = yes ]; then
        say "!! $name (pid $pid) still running 60 s after SIG$sig - NOT killed; look at it by hand"
        return 1
    fi
    say "$name stopped (pid $pid, SIG$sig)"
}

rc=0
# 0. the camera recording (SVO=1 drives; review round 1, item 1: a SECOND stopper beside svo_guard.sh).
#    svo_guard.sh --stop does nothing if this drive has no $REC/$RUN.svo2, and is harmless when the
#    guard has already stopped it: the wrapper refuses a stop when idle ("Recording was not active")
#    and the guard counts that as stopped only if the file is not growing.
if [ -e "$REC/$RUN.svo2" ]; then
    G="$(dirname "$0")/svo_guard.sh"
    if [ -r "$G" ]; then
        bash "$G" --stop "$RUN" 2>&1 | sed 's/^/   /' ; [ "${PIPESTATUS[0]}" = 0 ] || rc=1
    else
        say "!! $G missing - cannot check the camera recording; run: rosservice call /zedx_front/zed_node/stop_svo_recording"; rc=1
    fi
    say "camera still in the drive's recording mode: after the drive run  bash $(dirname "$0")/restore_camera_mode.sh  (rule 16)"
fi
stop_one "fusion recording" "${RECORD_PID:-}" record INT || rc=1
for i in $(seq 1 60); do [ -e "$REC/fusion.bag.active" ] || break; sleep 1; done
if [ -e "$REC/fusion.bag.active" ]; then say "!! $REC/fusion.bag.active still there - the bag did not close"; rc=1
elif [ -e "$REC/fusion.bag" ]; then say "fusion.bag closed: $(du -h "$REC/fusion.bag" | cut -f1)"; fi
stop_one "the blend (fused_odometry.launch)" "${FUSED_PID:-}" roslaunch INT || rc=1
stop_one "the bridge receiver" "${BRIDGE_PID:-}" robot_bridge_recv.py TERM || rc=1
say "done (exit $rc)"
exit $rc
