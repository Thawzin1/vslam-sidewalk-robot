#!/bin/bash
# svo_guard.sh - watches drive <run>'s camera recording (SVO2) and stops it, CHECKED, at the right moment.
#
#   usage:  svo_guard.sh <run_id> <rtabmap_pid>   watch; start_drive.sh step 3d starts it detached, nice 19,
#                                                 with the process number of THIS drive's mapping (RT)
#           svo_guard.sh --stop <run_id>           stop the recording now, checked (the second stopper:
#                                                 stop_fusion.sh calls this after the map has closed; safe
#                                                 to call when nothing is recording)
#           svo_guard.sh --selftest                the process checks against names known to be ABSENT
#
# WATCH MODE, every 5 s:
#   - one line to $JOBS_DIR/<run>_svo.progress for the jobs page ($JOBS_PAGE_URL):
#       SVO <run> <elapsed>/1800  <elapsed>s  <size> MB  <rate> MB/s  free <free> MB
#   - stops the camera recording (never the map) when
#       a) THIS drive's mapping process (the number given, whose name must read "rtabmap") has
#          gone for 2 checks in a row (10 s) - the drive is over; or
#       b) free space on the Jetson disk is under SVO_FLOOR_MB (default 2500 MB) - the map's database
#          is still being written, and the map matters more than the recording.
#   - "NOT GROWING" on the progress line if the file has not grown for 30 s while mapping.
# WHY 5 s (review round 1, item 7): normal end = at most 10 s to notice + ~1 s service call + 5 s
# check that the file stopped growing = ~16 s, inside the rehearsal's 30 s pass line with 14 s spare.
# A stop that needed retries (up to 3 x 15 s) overruns 30 s ON PURPOSE: that is a failure to report.
#
# STOPPING, CHECKED (review round 1, item 1): up to 3 tries. A try counts only if the file then stays
# the same size for 5 s. A reply "done: True" alone is not enough; a refused call ("Recording was not
# active", which the wrapper returns as a service error) is fine IF the file is not growing.
# After 3 failed tries the progress line says so in capitals and gives the command to run by hand, and
# (watch mode) it keeps trying every 30 s while the file grows (review round 2); --stop mode gives up after 3.
#
# *Plain terms: the camera keeps running after a drive, so without this the recording would never stop
#  and would fill the disk. This stops it when the map closes, or earlier if space runs short, and it
#  checks that the file really stopped growing rather than trusting the answer.*
#
# Process checks read /proc/<pid>/comm for ONE given number, never a search by name (ENGINEERING_NOTES.md rule 8).
# repository folders and addresses (REPO_ROOT, RECORDS_DIR, WORK_DIR, JOBS_DIR, TOOLS_DIR, *_ADDR): run/lib/paths.sh
. "$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)/../../run/lib/paths.sh"
set -u
SRV=/zedx_front/zed_node/stop_svo_recording
say() { echo "$(TZ=America/Toronto date '+%F %T %Z') $*"; }
is_rtabmap() { [ -n "$1" ] && [ "$(cat "/proc/$1/comm" 2>/dev/null)" = rtabmap ]; }

if [ "${1:-}" = "--selftest" ]; then
    # a number that cannot be a process, and this shell (which is not rtabmap): both must say "no"
    is_rtabmap 999999999 && r1=yes || r1=no
    is_rtabmap "$$" && r2=yes || r2=no
    is_rtabmap "" && r3=yes || r3=no
    echo "absent pid: $r1, this shell: $r2, empty: $r3 (all must be no)"
    [ "$r1$r2$r3" = nonono ]; exit $?
fi

MODE=watch
if [ "${1:-}" = "--stop" ]; then MODE=stop; shift; fi
RUN="${1:?usage: svo_guard.sh <run_id> <rtabmap_pid> | --stop <run_id> | --selftest}"
REC="$RECORDS_DIR/$RUN"
SVO="$REC/$RUN.svo2"
PROG="$JOBS_DIR/${RUN}_svo.progress"
mkdir -p "$JOBS_DIR"
set +u; source "$SIDEWALK_ENV"; set -u

# stop_checked: up to 3 tries; returns 0 only when the file has stayed the same size for 5 s
stop_checked() {
    local t ans a b
    for t in 1 2 3; do
        ans=$(timeout 15 rosservice call "$SRV" 2>&1 | tr '\n' ' ')
        say "stop try $t: $ans"
        a=$(stat -c %s "$SVO" 2>/dev/null || echo 0); sleep 5
        b=$(stat -c %s "$SVO" 2>/dev/null || echo 0)
        if [ "$a" = "$b" ]; then
            say "confirmed: $SVO unchanged at $b bytes over 5 s"
            return 0
        fi
        say "!! still growing ($a -> $b bytes) after try $t"
    done
    return 1
}
finish() {   # finish WHY
    local inf sz
    if stop_checked; then
        sleep 1
        inf=$(DISPLAY= /usr/local/zed/tools/ZED_SVO_Editor -inf "$SVO" 2>/dev/null | grep -E "Number of Frames|Compression mode" | tr -s ' ' | tr '\n' ' ')
        sz=$(stat -c %s "$SVO" 2>/dev/null || echo 0)
        say "stopped ($1). $((sz / 1000000)) MB. $inf"
        echo "SVO $RUN STOPPED $((sz / 1000000)) MB  ${inf}- $1" > "$PROG"
        return 0
    fi
    say "!! COULD NOT STOP THE CAMERA RECORDING after 3 tries - run by hand: rosservice call $SRV"
    echo "SVO $RUN !! STOP FAILED - STILL RECORDING - run: rosservice call $SRV" > "$PROG"
    # watch mode keeps trying every 30 s for as long as the file grows (review round 2, item 4) - above
    # all after a disk-floor stop, where every further MB eats the map's room. --stop mode (called by
    # stop_fusion.sh) does NOT loop, so the blend and bridge are still stopped after it.
    [ "$MODE" = watch ] || return 1
    local n=1   # rounds of up to 3 tries each
    while :; do
        sleep 30
        n=$((n + 1))
        if stop_checked; then
            sz=$(stat -c %s "$SVO" 2>/dev/null || echo 0)
            say "stopped LATE in round $n ($1). $((sz / 1000000)) MB"
            echo "SVO $RUN STOPPED LATE in round $n $((sz / 1000000)) MB - $1" > "$PROG"
            return 0
        fi
        sz=$(stat -c %s "$SVO" 2>/dev/null || echo 0)
        FREE=$(df -BM --output=avail "$HOME" | tail -1 | tr -dc 0-9)
        echo "SVO $RUN !! STOP FAILED, $n rounds - STILL RECORDING $((sz / 1000000)) MB free $FREE MB - run: rosservice call $SRV" > "$PROG"
    done
}

if [ "$MODE" = stop ]; then
    [ -e "$SVO" ] || { say "no $SVO - this drive had no camera recording; nothing to stop"; exit 0; }
    say "second stopper for $RUN"
    finish "second stopper (after the map closed)"; exit $?
fi

RT="${2:?usage: svo_guard.sh <run_id> <rtabmap_pid>}"
is_rtabmap "$RT" || { say "!! pid $RT is not rtabmap - stopping the recording now"; finish "no valid mapping pid"; exit 1; }
FLOOR="${SVO_FLOOR_MB:-2500}"
T0=$(date +%s); GONE=0; LAST=0; LAST_T=$T0; STILL=0; WHY=""
say "watching $SVO for mapping pid $RT (floor ${FLOOR} MB free, every 5 s)"
while :; do
    sleep 5
    NOW=$(date +%s); EL=$((NOW - T0))
    SZ=$(stat -c %s "$SVO" 2>/dev/null || echo 0)
    FREE=$(df -BM --output=avail "$HOME" | tail -1 | tr -dc 0-9)
    RATE=$(awk -v a="$SZ" -v b="$LAST" -v t=$((NOW - LAST_T)) 'BEGIN{printf "%.1f", (a-b)/1e6/(t>0?t:1)}')
    if [ "$SZ" -gt "$LAST" ]; then STILL=0; else STILL=$((STILL + 5)); fi
    LAST=$SZ; LAST_T=$NOW
    if is_rtabmap "$RT"; then GONE=0; else GONE=$((GONE + 1)); fi
    FLAG=""; [ "$STILL" -ge 30 ] && [ "$GONE" = 0 ] && FLAG="  !! NOT GROWING ${STILL}s"
    echo "SVO $RUN $EL/1800  ${EL}s  $((SZ / 1000000)) MB  $RATE MB/s  free $FREE MB$FLAG" > "$PROG"
    if [ "$FREE" -lt "$FLOOR" ]; then WHY="disk floor: ${FREE} MB free < ${FLOOR} MB (the map keeps running)"; break; fi
    if [ "$GONE" -ge 2 ]; then WHY="mapping pid $RT has exited (drive over)"; break; fi
done
finish "$WHY"
