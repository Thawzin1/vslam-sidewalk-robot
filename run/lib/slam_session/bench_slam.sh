#!/bin/bash
# bench_slam.sh <run> <MASK 0|1> [SVO 0|1] - BENCH test of the SLAM mode, camera only, robot OFF. Jetson.
#
# Proves the mechanics before any lab time: the staged start (SLAM=1) opens a COPY of drive 10's map in mapping
# mode, starts a new map id, writes new nodes, closes properly on "park", and leaves the master untouched.
# With MASK=1 it also measures the people detector's cost (detector ON 60 s vs OFF 60 s, same camera session)
# and runs the live made-up-person test (fake_person_test.py). Leaves the camera as found (stopped), and the
# camera mode file as found. Keeps the small records in bench/<run>/; the 3.5 GB working copy and the camera
# recording are NOT deleted here (the caller does that after reading the results).
# Progress: $JOBS_DIR/<run>_bench.progress. Stops early if $WORK_DIR/HELPERS_STOP appears.
# repository folders and addresses (REPO_ROOT, RECORDS_DIR, WORK_DIR, JOBS_DIR, TOOLS_DIR, *_ADDR): run/lib/paths.sh
. "$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)/../paths.sh"
set -u
RUN="${1:?usage: bench_slam.sh <run> <MASK 0|1> [SVO 0|1]}"; M="${2:?MASK 0|1}"; SV="${3:-1}"
HERE="$(cd "$(dirname "$0")" && pwd)"
B="$HERE/bench/$RUN"; mkdir -p "$B"
REC="$RECORDS_DIR/$RUN"
PROG="$JOBS_DIR/${RUN}_bench.progress"; T0=$(date +%s)
say() { echo "$(TZ=America/Toronto date '+%F %T %Z') $*" | tee -a "$B/bench.log"; }
prog() { echo "SLAM_BENCH $RUN $1  $(( $(date +%s) - T0 ))s" > "$PROG"; }
set +u; source "$SIDEWALK_ENV"; set -u
cam_launches() { local d A x c; for d in /proc/[0-9]*; do c=$(cat "$d/comm" 2>/dev/null)
    [ "$c" = roslaunch ] || [ "$c" = python3 ] || continue
    mapfile -d '' -t A < "$d/cmdline" 2>/dev/null || continue
    [ "$c" = python3 ] && { [ "${#A[@]}" -ge 2 ] && [ "$(basename -- "${A[1]}")" = roslaunch ] || continue; }
    for x in "${A[@]:1}"; do [ "$(basename -- "$x")" = zedx_front.launch ] && { echo "${d#/proc/}"; break; }; done; done; }
cam_nodes() { local d A; for d in /proc/[0-9]*; do [ "$(cat "$d/comm" 2>/dev/null)" = zed_wrapper_nod ] || continue
    mapfile -d '' -t A < "$d/cmdline" 2>/dev/null || continue
    [ "${#A[@]}" -ge 1 ] && [ "$(basename -- "${A[0]}")" = zed_wrapper_node ] && echo "${d#/proc/}"; done; }
rt_running() { ps -eo comm= | awk '$1 == "rtabmap" {f=1} END {exit !f}'; }
helpers_stop() { [ -e "$WORK_DIR/HELPERS_STOP" ]; }
park_and_close() {
    touch "$WORK_DIR/${1}_$RUN"
    say "${1}_$RUN written - waiting for the map to close"
    local T=$(date +%s)
    while rt_running; do [ $(( $(date +%s) - T )) -gt 1500 ] && { say "!! map not closed after 25 min"; break; }; prog "closing the map ($1)"; sleep 5; done
}

[ -n "$(cam_launches)$(cam_nodes)" ] && { say "REFUSED: a camera is already running (bench must leave the camera as found)"; exit 1; }
rt_running && { say "REFUSED: a map is running"; exit 1; }
helpers_stop && { say "REFUSED: HELPERS_STOP"; exit 1; }
cp "$WORK_DIR/camera_svo_mode.txt" "$B/camera_svo_mode.txt.found"
say "bench $RUN MASK=$M SVO=$SV; disk $(df -h --output=avail / | tail -1) free; camera mode file saved"
prog "starting (start_slam.sh BENCH=1 MASK=$M SVO=$SV)"
T_START=$(date +%s)
BENCH=1 MASK="$M" SVO="$SV" LIDAR_PEER=http://$ROBOT_WIFI_ADDR:8095 bash "$HERE/start_slam.sh" "$RUN" > "$B/start.out" 2>&1
RC=$?
say "start_slam.sh exited $RC after $(( $(date +%s) - T_START )) s: $(grep -a -E 'REFUSED|PASS: mapping|left alone' "$B/start.out" | tail -2 | tr '\n' ' ')"
if [ "$RC" != 0 ] || ! rt_running; then
    say "!! start failed - closing whatever runs"; rt_running && park_and_close STOP
else
    sleep 30
    prog "measuring 60 s (A: $( [ "$M" = 1 ] && echo 'detector ON + mask' || echo 'no detector') )"
    A0=$(date +%s)
    python3 "$HERE/rate_probe.py" --seconds 60 --label A --out "$B/rates_A.json" \
        $( [ "$M" = 1 ] && echo --objects /zedx_front/zed_node/obj_det/objects ) \
        /rtabmap/odom /rtabmap/info /zedx_front/zed_node/rgb/camera_info > /dev/null 2>&1
    A1=$(date +%s)
    say "A: $(python3 -c "import json;d=json.load(open('$B/rates_A.json'));print({k:v['rate_hz'] for k,v in d['topics'].items()}, d.get('objects',{}).get('rate_hz'))")"
    if [ "$M" = 1 ] && ! helpers_stop; then
        say "detector OFF: $(timeout 30 rosservice call /zedx_front/zed_node/enable_object_detection 'data: false' 2>&1 | tr '\n' ' ')"
        sleep 15
        prog "measuring 60 s (B: detector OFF, mask passes depth through)"
        B0=$(date +%s)
        python3 "$HERE/rate_probe.py" --seconds 60 --label B --out "$B/rates_B.json" \
            /rtabmap/odom /rtabmap/info /zedx_front/zed_node/rgb/camera_info > /dev/null 2>&1
        B1=$(date +%s)
        say "B: $(python3 -c "import json;d=json.load(open('$B/rates_B.json'));print({k:v['rate_hz'] for k,v in d['topics'].items()})")"
        prog "made-up person test (20 s)"
        python3 "$HERE/fake_person_test.py" --seconds 20 --out "$B/fake_person_test.json" 2>&1 | tail -1 | tee -a "$B/bench.log"
        say "detector ON again: $(timeout 60 rosservice call /zedx_front/zed_node/enable_object_detection 'data: true' 2>&1 | tr '\n' ' ')"
        sleep 10
        cp "$REC/mask_stats.csv" "$REC/mask_events.csv" "$B/" 2>/dev/null
        head -1 "$JOBS_DIR/${RUN}_mask.progress" >> "$B/bench.log" 2>/dev/null
        python3 "$HERE/tegra_window.py" "$REC/tegrastats.log" "$B0" "$B1" --label B_detector_off > "$B/tegra_B.json"
    fi
    python3 "$HERE/tegra_window.py" "$REC/tegrastats.log" "$A0" "$A1" --label A > "$B/tegra_A.json"
    head -1 "$JOBS_DIR/${RUN}_slam.progress" >> "$B/bench.log" 2>/dev/null
    prog "parking (60 s still, then the map closes)"
    park_and_close PARK
fi
prog "checks (stop_slam.sh)"
bash "$HERE/stop_slam.sh" "$RUN" > "$B/stop_slam.out" 2>&1
cat "$B/stop_slam.out" >> "$B/bench.log"
# leave the camera as found: it was not running
for p in $(cam_launches); do kill -INT "$p" 2>/dev/null; done
for i in $(seq 1 40); do [ -z "$(cam_launches)$(cam_nodes)" ] && break; sleep 1; done
for p in $(cam_launches) $(cam_nodes); do kill -TERM "$p" 2>/dev/null; done; sleep 3
[ -z "$(cam_launches)$(cam_nodes)" ] && say "camera stopped (as found)" || say "!! camera still running: $(cam_launches) $(cam_nodes)"
cp "$B/camera_svo_mode.txt.found" "$WORK_DIR/camera_svo_mode.txt" && say "camera mode file restored as found"
# small records only (the big files stay where they are until the caller deletes them)
for f in mapping.log autostop.log slam_watch.log slam_events.csv slam_summary.json slam_check.json old_map_facts.json \
         master.sha256 stop_slam_report.txt camera_guard.log depth_person_mask.log monitor.csv tf_check_2.txt svo_guard.log; do
    [ -f "$REC/$f" ] && cp "$REC/$f" "$B/" 2>/dev/null
done
ls -la "$REC" > "$B/run_records_listing.txt"; ls -la "$WORK_DIR/$RUN.db" >> "$B/run_records_listing.txt" 2>&1
prog "DONE - $(grep -a -c PASS "$B/stop_slam.out") PASS / $(grep -a -c FAIL "$B/stop_slam.out") FAIL in the checks"
say "done"
