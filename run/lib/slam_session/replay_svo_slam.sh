#!/bin/bash
# replay_svo_slam.sh <run> <off|on> [max_seconds] - replay a SLAM session's camera recording (SVO) through the same
# SLAM stack into a FRESH copy of drive 10's map, with the people mask OFF or ON. Jetson, robot not needed.
#
# WHY: the people test (PASS_LINES.md D0-D3) compares mask OFF vs ON on IDENTICAL pictures - record once, replay twice
# (ENGINEERING_NOTES.md section 4 rule 5). Both replays run the people detector (so the load is the same); only the blanking
# differs. Camera-only (no wheels): the same for both.
# Pinned for repeatability (zedx-offline-replay memory): every recorded frame processed (svo_realtime false, via
# zedx_front_od_replay.yaml), self-calibration off, depth stabilisation 1 (the live value), NEURAL, GEN_2.
# *Plain terms: play the session's video back through the map program twice, once with people blanked and once
# without, so any difference in the map is caused by the blanking alone.*
#
# Output: $WORK_DIR/<run>_replay_<mode>.db (a copy of the master, grown by the replay), records in
# $RECORDS_DIR/<run>_replay_<mode>/ (mapping.log, mask_*.csv, tegrastats). Stops when the recording ends (no new
# odometry for 30 s) or after max_seconds (default: no limit), closing the map properly (auto_stop STOP file).
# Refuses while a camera or a map runs, or $WORK_DIR/HELPERS_STOP exists. Progress: $JOBS_DIR/<run>_replay_<mode>.progress
# repository folders and addresses (REPO_ROOT, RECORDS_DIR, WORK_DIR, JOBS_DIR, TOOLS_DIR, *_ADDR): run/lib/paths.sh
. "$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)/../paths.sh"
set -u
RUN="${1:?usage: replay_svo_slam.sh <run> <off|on> [max_seconds]}"; MODE="${2:?off|on}"; MAXS="${3:-0}"
case "$MODE" in off) MASKARG=false;; on) MASKARG=true;; *) echo "mode must be off or on" >&2; exit 2;; esac
HERE="$(cd "$(dirname "$0")" && pwd)"
SVOF="$RECORDS_DIR/$RUN/$RUN.svo2"
RR="${RUN}_replay_$MODE"; REC="$RECORDS_DIR/$RR"; DB="$WORK_DIR/$RR.db"
MAP_DB="${MAP_DB:-$WORK_DIR/localise/s2_static_10_map.db}"
PROG="$JOBS_DIR/$RR.progress"; T0=$(date +%s)
say() { echo "$(TZ=America/Toronto date '+%F %T %Z') $*" | tee -a "$REC/replay.log"; }
prog() { echo "REPLAY $RR $1  $(( $(date +%s) - T0 ))s" > "$PROG"; }
set +u; source "$SIDEWALK_ENV"; set -u
rt_running() { ps -eo comm= | awk '$1 == "rtabmap" {f=1} END {exit !f}'; }
cam_running() { ps -eo comm= | awk '$1 == "zed_wrapper_nod" {f=1} END {exit !f}'; }
[ -r "$SVOF" ] || { echo "no recording $SVOF" >&2; exit 1; }
[ -e "$DB" ] && { echo "$DB exists - delete it first" >&2; exit 1; }
[ -e "$WORK_DIR/HELPERS_STOP" ] && { echo "HELPERS_STOP" >&2; exit 1; }
cam_running && { echo "a camera is running" >&2; exit 1; }
rt_running && { echo "a map is running" >&2; exit 1; }
FREE=$(df -BM --output=avail "$HOME" | tail -1 | tr -dc 0-9); MB=$(( $(stat -c %s "$MAP_DB") / 1048576 ))
[ $(( FREE - MB )) -ge 3072 ] || { echo "not enough space: ${FREE} MB free, the copy needs ${MB} MB + 3 GB kept free" >&2; exit 1; }
mkdir -p "$REC"
say "replay $RR: $SVOF -> copy of $MAP_DB, mask $MODE"
sha256sum "$MAP_DB" > "$REC/master.sha256"
cp "$MAP_DB" "$DB" && chmod u+w "$DB" && cmp -s "$MAP_DB" "$DB" || { say "copy failed"; rm -f "$DB"; exit 1; }
python3 "$HERE/slam_closed_check.py" --before "$DB" "$REC/old_map_facts.json" >> "$REC/replay.log"
OLD_MAX=$(python3 -c "import json,sys;print(json.load(open(sys.argv[1]))['node_id_max'])" "$REC/old_map_facts.json")
prog "camera from the recording"
DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority setsid nohup roslaunch sidewalk_perception zedx_front.launch \
    svo_file:="$SVOF" depth_mode:=NEURAL pos_tracking_mode:=GEN_2 self_calib:=false depth_stabilization:=1 \
    config_file:="$HERE/zedx_front_od_replay.yaml" > "$REC/camera.log" 2>&1 < /dev/null &
CAM=$!
for i in $(seq 1 60); do timeout 5 rostopic echo -n1 /zedx_front/zed_node/rgb/camera_info > /dev/null 2>&1 && break; sleep 2; done
say "camera (from the recording) publishing"
if [ "$MODE" = on ]; then
    setsid nohup python3 "$HERE/depth_person_mask.py" __name:=depth_person_mask _run:="$RR" > "$REC/depth_person_mask.log" 2>&1 < /dev/null &
    sleep 3
fi
setsid nohup roslaunch "$HERE/slam_run.launch" run_id:="$RR" database_path:="$DB" rtabmap_log:="$REC/mapping.log" \
    vo_publish_tf:=true loop_thr:=0.08 mask:=$MASKARG > "$REC/mapping.log" 2>&1 < /dev/null &
RT=""; for i in $(seq 1 45); do RT=$(ps -eo pid=,comm= | awk '$2 == "rtabmap" {print $1}'); [ -n "$RT" ] && break; sleep 2; done
[ -n "$RT" ] || { say "map did not start"; kill -INT "$CAM"; exit 1; }
echo "$RT" > "$REC/rtabmap.pid"
setsid nohup bash -c 'sudo -n tegrastats --interval 1000 --logfile "$1" & T=$!; while kill -0 "$2" 2>/dev/null; do sleep 5; done; sudo -n pkill -P "$T" -x tegrastats' _ "$REC/tegrastats.log" "$RT" > /dev/null 2>&1 < /dev/null &
setsid nohup python3 "$HERE/slam_watch.py" --run "$RR" --old-max "$OLD_MAX" --rtabmap-pid "$RT" \
    --old-facts "$REC/old_map_facts.json" > "$REC/slam_watch.log" 2>&1 < /dev/null &
setsid nohup python3 "$TOOLS_DIR/auto_stop_mapping.py" --run "$RR" --db "$DB" --launch-file slam_run.launch \
    > "$REC/autostop.log" 2>&1 < /dev/null &
# wait for the end of the recording: no new odometry for 30 s (after the map has started receiving)
T_ST=$(date +%s); LAST=$(date +%s); N0=0
while :; do
    [ -e "$WORK_DIR/HELPERS_STOP" ] && { say "HELPERS_STOP - closing"; break; }
    N=$(timeout 6 rostopic echo -n1 /rtabmap/odom/header/seq 2>/dev/null | head -1 | tr -dc 0-9)
    if [ -n "$N" ] && [ "$N" != "$N0" ]; then LAST=$(date +%s); N0=$N; fi
    [ $(( $(date +%s) - LAST )) -gt 30 ] && [ $(( $(date +%s) - T_ST )) -gt 90 ] && { say "recording ended (no odometry for 30 s)"; break; }
    [ "$MAXS" -gt 0 ] && [ $(( $(date +%s) - T_ST )) -gt "$MAXS" ] && { say "max ${MAXS} s reached"; break; }
    prog "replaying (odometry message $N0); $(head -1 "$JOBS_DIR/${RR}_slam.progress" 2>/dev/null | cut -c1-120)"
    sleep 10
done
touch "$WORK_DIR/STOP_$RR"
while rt_running; do prog "closing the map"; sleep 5; done
sleep 3
kill -INT "$CAM" 2>/dev/null; for i in $(seq 1 30); do cam_running || break; sleep 1; done
cam_running && say "!! camera from the recording still running"
python3 "$HERE/slam_closed_check.py" --after "$DB" "$REC/old_map_facts.json" "$REC/slam_check.json" >> "$REC/replay.log" 2>&1
sha256sum -c --status "$REC/master.sha256" && say "master unchanged" || say "!! master CHANGED"
rm -f "$WORK_DIR/STOP_$RR"
say "done: $(grep -a complete "$REC/autostop.log" | tail -1)"
prog "DONE $(grep -a -c PASS "$REC/replay.log") PASS / $(grep -a -c FAIL "$REC/replay.log") FAIL"
