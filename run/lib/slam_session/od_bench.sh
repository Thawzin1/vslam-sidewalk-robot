#!/bin/bash
# od_bench.sh - ONE-TIME preparation + cost measurement of the ZED people detector (object detection, "OD"),
#
# WHY: the detector's model file (/usr/local/zed/resources/objects_performance_3.2.model) has never been
# optimised for this Jetson's graphics chip (no ".objects_performance_3.2.model_optimized-*" file). The ZED
# library does that optimisation the first time the detector starts, and it can take many minutes, during
# which the camera sends NO pictures. That must never happen at the start of a session, so it is done here,
# once. Then the same camera session measures the cost: pictures per second, GPU and CPU load with the
# detector ON (60 s) and OFF (60 s), switched by the wrapper's own service (enable_object_detection).
# *Plain terms: switch the people detector on once so it gets ready, then time the camera with it on and off.*
#
#   usage:  bash od_bench.sh [outdir]      (default: this folder's bench/od_<time>)
# Leaves the camera as it found it: it refuses if a camera or a map is already running, and stops the camera
# it started (by process number, whole-name checks, never pgrep -f - ENGINEERING_NOTES.md rule 8).
# Progress: $JOBS_DIR/od_bench.progress (jobs page, rule 13). Stops at once if $WORK_DIR/HELPERS_STOP appears.
# repository folders and addresses (REPO_ROOT, RECORDS_DIR, WORK_DIR, JOBS_DIR, TOOLS_DIR, *_ADDR): run/lib/paths.sh
. "$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)/../paths.sh"
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
OUT="${1:-$HERE/bench/od_$(TZ=America/Toronto date +%Y%m%d_%H%M)}"
mkdir -p "$OUT"
PROG="$JOBS_DIR/od_bench.progress"
T0=$(date +%s)
say() { echo "$(TZ=America/Toronto date '+%F %T %Z') $*" | tee -a "$OUT/od_bench.log"; }
prog() { echo "OD_BENCH $1  $(( $(date +%s) - T0 ))s" > "$PROG"; }
set +u; source "$SIDEWALK_ENV"; set -u

cam_launches() { local d A x c; for d in /proc/[0-9]*; do c=$(cat "$d/comm" 2>/dev/null)
    [ "$c" = roslaunch ] || [ "$c" = python3 ] || continue
    mapfile -d '' -t A < "$d/cmdline" 2>/dev/null || continue
    [ "$c" = python3 ] && { [ "${#A[@]}" -ge 2 ] && [ "$(basename -- "${A[1]}")" = roslaunch ] || continue; }
    for x in "${A[@]:1}"; do [ "$(basename -- "$x")" = zedx_front.launch ] && { echo "${d#/proc/}"; break; }; done; done; }
cam_nodes() { local d A; for d in /proc/[0-9]*; do [ "$(cat "$d/comm" 2>/dev/null)" = zed_wrapper_nod ] || continue
    mapfile -d '' -t A < "$d/cmdline" 2>/dev/null || continue
    [ "${#A[@]}" -ge 1 ] && [ "$(basename -- "${A[0]}")" = zed_wrapper_node ] && echo "${d#/proc/}"; done; }
stop_camera() {
    for p in $(cam_launches); do kill -INT "$p" 2>/dev/null; done
    for i in $(seq 1 40); do [ -z "$(cam_launches)$(cam_nodes)" ] && break; sleep 1; done
    for p in $(cam_launches) $(cam_nodes); do kill -TERM "$p" 2>/dev/null; done; sleep 3
    [ -z "$(cam_launches)$(cam_nodes)" ] && say "camera stopped" || say "!! camera still running: $(cam_launches) $(cam_nodes)"
}
TEG_PID=""
cleanup() { [ -n "$TEG_PID" ] && sudo -n pkill -P "$TEG_PID" -x tegrastats 2>/dev/null; true; }   # only OUR tegrastats (child of our sudo)

[ -n "$(cam_launches)$(cam_nodes)" ] && { say "REFUSED: a camera is already running"; prog "REFUSED camera already running"; exit 1; }
[ -n "$(ps -eo comm= | awk '$1=="rtabmap"')" ] && { say "REFUSED: a map (rtabmap) is running"; prog "REFUSED rtabmap running"; exit 1; }
timeout 5 rostopic list > /dev/null 2>&1 || { say "REFUSED: no roscore"; exit 1; }
sha256sum -c --status "$HERE/zedx_front.yaml.BASE_sha256" || { say "REFUSED: live zedx_front.yaml changed since the staged copy"; exit 1; }
ls /usr/local/zed/resources/.objects_performance* > "$OUT/optimised_before.txt" 2>&1
say "optimised detector files before: $(cat "$OUT/optimised_before.txt")"

sudo -n tegrastats --interval 1000 --logfile "$OUT/tegrastats.log" & TEG_PID=$!
trap cleanup EXIT
say "starting the camera (NEURAL, GEN_2, people detector ON, FAST model) - log $OUT/camera.log"
T_CAM=$(date +%s)
DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority setsid nohup \
    roslaunch sidewalk_perception zedx_front.launch depth_mode:=NEURAL pos_tracking_mode:=GEN_2 \
    config_file:="$HERE/zedx_front_od.yaml" > "$OUT/camera.log" 2>&1 < /dev/null &
OBJ=/zedx_front/zed_node/obj_det/objects
# wait (no deadline shorter than 40 min) for the first detector message = optimisation finished
FIRST=""
while :; do
    [ -e "$WORK_DIR/HELPERS_STOP" ] && { say "HELPERS_STOP - stopping"; stop_camera; prog "STOPPED by HELPERS_STOP"; exit 2; }
    W=$(( $(date +%s) - T_CAM ))
    [ -z "$(cam_nodes)" ] && [ "$W" -gt 60 ] && { say "!! the camera program died - see camera.log"; prog "FAILED camera died"; stop_camera; exit 3; }
    if timeout 8 rostopic echo -n1 "$OBJ" > /dev/null 2>&1; then FIRST=$W; break; fi
    [ "$W" -gt 2400 ] && { say "!! no detector output after 40 min"; prog "FAILED no detector output 40 min"; stop_camera; exit 4; }
    prog "optimising/starting the detector, waiting ${W}s (camera log: $(grep -a -c . "$OUT/camera.log") lines)"
    sleep 12
done
say "first detector message ${FIRST} s after the camera start"
ls -la /usr/local/zed/resources/.objects_performance* > "$OUT/optimised_after.txt" 2>&1
grep -a -i -E "object|optim|model|Detection" "$OUT/camera.log" | tail -20 > "$OUT/camera_od_lines.txt"
sleep 10
prog "measuring detector ON 60 s"
A0=$(date +%s)
python3 "$HERE/rate_probe.py" --seconds 60 --label od_on --out "$OUT/rates_od_on.json" --objects "$OBJ" \
    /zedx_front/zed_node/depth/depth_registered /zedx_front/zed_node/rgb/camera_info > /dev/null 2>&1
A1=$(date +%s)
ANS=$(timeout 30 rosservice call /zedx_front/zed_node/enable_object_detection "data: false" 2>&1 | tr '\n' ' ')
say "detector OFF: $ANS"
sleep 10
prog "measuring detector OFF 60 s"
B0=$(date +%s)
python3 "$HERE/rate_probe.py" --seconds 60 --label od_off --out "$OUT/rates_od_off.json" \
    /zedx_front/zed_node/depth/depth_registered /zedx_front/zed_node/rgb/camera_info > /dev/null 2>&1
B1=$(date +%s)
ANS=$(timeout 60 rosservice call /zedx_front/zed_node/enable_object_detection "data: true" 2>&1 | tr '\n' ' ')
T_ON=$(date +%s)
timeout 60 rostopic echo -n1 "$OBJ" > /dev/null 2>&1 && RE=$(( $(date +%s) - T_ON )) || RE=">60"
say "detector ON again: $ANS - first message after ${RE} s (the start-up cost once optimised)"
cleanup; TEG_PID=""
python3 "$HERE/tegra_window.py" "$OUT/tegrastats.log" "$A0" "$A1" --label od_on > "$OUT/tegra_od_on.json"
python3 "$HERE/tegra_window.py" "$OUT/tegrastats.log" "$B0" "$B1" --label od_off > "$OUT/tegra_od_off.json"
stop_camera
python3 - "$OUT" "$FIRST" "$RE" <<'EOF'
import json, sys, os
o = sys.argv[1]
r = {k: json.load(open(os.path.join(o, k + ".json"))) for k in ("rates_od_on", "rates_od_off", "tegra_od_on", "tegra_od_off")}
r["first_detector_message_s_after_camera_start"] = sys.argv[2]
r["restart_after_optimised_s"] = sys.argv[3]
json.dump(r, open(os.path.join(o, "od_bench_summary.json"), "w"), indent=1)
d_on = r["rates_od_on"]["topics"]["/zedx_front/zed_node/depth/depth_registered"]["rate_hz"]
d_off = r["rates_od_off"]["topics"]["/zedx_front/zed_node/depth/depth_registered"]["rate_hz"]
g_on = (r["tegra_od_on"]["gpu_busy_pct"] or {}).get("mean"); g_off = (r["tegra_od_off"]["gpu_busy_pct"] or {}).get("mean")
print("DONE depth %s Hz on / %s Hz off; GPU %s%% on / %s%% off; detector %s Hz" % (d_on, d_off, g_on, g_off,
      r["rates_od_on"].get("objects", {}).get("rate_hz")))
EOF
say "summary: $OUT/od_bench_summary.json"
prog "DONE $(python3 -c "import json;r=json.load(open('$OUT/od_bench_summary.json'));print('depth',r['rates_od_on']['topics']['/zedx_front/zed_node/depth/depth_registered']['rate_hz'],'Hz on /',r['rates_od_off']['topics']['/zedx_front/zed_node/depth/depth_registered']['rate_hz'],'off')")"
