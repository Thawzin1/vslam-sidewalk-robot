#!/bin/bash
# stop_slam.sh - after a SLAM session (or bench) has closed: the checks, and the master's checksum. Jetson.
#
#   usage:  stop_slam.sh <run>          after "park" has closed the map (auto_stop_mapping.py says "complete")
#           stop_slam.sh <run> --now    something is wrong: close the map NOW (STOP_<run>), wait, then check
#
# The map itself is always closed by auto_stop_mapping.py (started by start_drive.sh): it interrupts rtabmap
# and waits with no deadline for the database to close, then stops the fusion pieces. This script never kills
# rtabmap. It then runs, read-only:
#   db_check.py            SQLite reads every page of $WORK_DIR/<run>.db (whole file?)
#   slam_closed_check.py   new session written, new map id, closed properly, old map intact
#   master checksum        drive 10's master map is byte-identical to before the session
# and writes $RECORDS_DIR/<run>/stop_slam_report.txt. The camera is left running (as after a drive); after an
# SVO=1 session run  bash $TOOLS_DIR/restore_camera_mode.sh  (camera back to the depth study's mode).
# repository folders and addresses (REPO_ROOT, RECORDS_DIR, WORK_DIR, JOBS_DIR, TOOLS_DIR, *_ADDR): run/lib/paths.sh
. "$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)/../paths.sh"
set -u
RUN="${1:?usage: stop_slam.sh <run> [--now]}"
[ -f "$RECORDS_DIR/$RUN/live_obstacles.pid" ] && kill -INT "$(cat "$RECORDS_DIR/$RUN/live_obstacles.pid")" 2>/dev/null && echo "live obstacle step stopped"
HERE="$(cd "$(dirname "$0")" && pwd)"
REC="$RECORDS_DIR/$RUN"; DB="$WORK_DIR/$RUN.db"
OUT="$REC/stop_slam_report.txt"
say() { echo "$*" | tee -a "$OUT"; }
running() { ps -eo comm= | awk '$1 == "rtabmap" {f=1} END {exit !f}'; }
: > "$OUT"
say "stop_slam.sh $RUN  $(TZ=America/Toronto date '+%F %T %Z')"
if running; then
    if [ "${2:-}" = "--now" ]; then
        touch "$WORK_DIR/STOP_$RUN"
        say "STOP_$RUN written - auto_stop_mapping.py closes the map now (no hold); waiting for it"
    else
        say "the map (rtabmap) is still running - say 'park' (touch $WORK_DIR/PARK_$RUN), or use --now"; exit 1
    fi
fi
T=$(date +%s)
while running; do
    [ $(( $(date +%s) - T )) -gt 1800 ] && { say "!! rtabmap still running after 30 min - NOT killed; look by hand"; exit 3; }
    sleep 5
done
for i in $(seq 1 60); do grep -q "complete" "$REC/autostop.log" 2>/dev/null && break; sleep 2; done
say "autostop: $(grep -a -E 'complete|FAILED' "$REC/autostop.log" 2>/dev/null | tail -1)"
python3 "$TOOLS_DIR/db_check.py" "$DB" 2>&1 | tee -a "$OUT"
python3 "$HERE/slam_closed_check.py" --after "$DB" "$REC/old_map_facts.json" "$REC/slam_check.json" 2>&1 | tee -a "$OUT"
if sha256sum -c --status "$REC/master.sha256"; then
    say "PASS  master map unchanged ($(cut -c1-16 "$REC/master.sha256")...)"
else
    say "FAIL  master map CHANGED or unreadable - $(cat "$REC/master.sha256")"
fi
[ -f "$REC/slam_summary.json" ] && say "watch summary: $(python3 -c "import json,sys;d=json.load(open(sys.argv[1]));print('joined', d.get('join'), '| working graph at the end', d.get('graph'), '| new link events to old', d.get('ev_old'), 'within new', d.get('ev_new'), '| suspect', d.get('suspect'))" "$REC/slam_summary.json")"
grep -q "(NOT default) - camera started by $RUN;" "$WORK_DIR/camera_svo_mode.txt" 2>/dev/null \
    && say "camera still in SVO mode for $RUN: run  bash $TOOLS_DIR/restore_camera_mode.sh"
say "report: $OUT"
