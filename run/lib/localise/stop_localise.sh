#!/bin/bash
# stop_localise.sh - end a localisation session (start_drive.sh LOCALISE=1), in the safe order.
#
#   usage:  stop_localise.sh <run> [--keep-camera]
#
# ORDER, every process found by WHOLE names from /proc, never "contains" (ENGINEERING_NOTES.md rule 8):
#   1. the localiser (roslaunch of localise_run.launch): SIGINT, wait for `rtabmap` to exit (<=120 s)
#   2. FUSION pieces, if this session had them: stop_fusion.sh (recording, blend, bridge)
#   3. the camera guard (it exits by itself once rtabmap has gone - waited for, up to 30 s),
#      THEN the camera (unless --keep-camera): a guard still running would restart it
#   4. the watcher (localise_watch.py)
#   5. PASS LINE P4: the master map's checksum must equal the one taken at the start, and the last
#      working copy must still hold the master's node count (localisation added nothing permanent)
#   6. the working copies are deleted (scratch; ENGINEERING_NOTES.md rule 9) - the master is never touched
# The live map page (:8095) and the timelapse recorder are left as after a drive (the recorder
# finishes by itself 120 s after rtabmap has gone).
# repository folders and addresses (REPO_ROOT, RECORDS_DIR, WORK_DIR, JOBS_DIR, TOOLS_DIR, *_ADDR): run/lib/paths.sh
. "$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)/../paths.sh"
set -u
RUN="${1:?usage: stop_localise.sh <run> [--keep-camera]}"
KEEPCAM="${2:-}"
REC="$RECORDS_DIR/$RUN"
TOOLS="$TOOLS_DIR"
MAP_DB="${MAP_DB:-$WORK_DIR/localise/s2_static_04_map.db}"
set +u; source "$SIDEWALK_ENV"; set -u

# pids_arg COMM1 SCRIPT: processes whose comm is COMM1 (or python3) and that have SCRIPT as a whole
# argument (base name), skipping this shell and its parent
pids_arg() {
    local want="$1" d pid c A x
    for d in /proc/[0-9]*; do
        pid="${d#/proc/}"
        { [ "$pid" = "$$" ] || [ "$pid" = "$PPID" ]; } && continue
        c=$(cat "$d/comm" 2>/dev/null) || continue
        [ "$c" = roslaunch ] || [ "$c" = python3 ] || continue
        mapfile -d '' -t A < "$d/cmdline" 2>/dev/null || continue
        for x in "${A[@]:1}"; do [ "$(basename -- "$x")" = "$want" ] && { echo "$pid"; break; }; done
    done
}
comm_pids() { ps -eo pid=,comm= | awk -v w="$1" '$2 == w {print $1}'; }

echo "== 1. the localiser"
L=$(pids_arg localise_run.launch | tr '\n' ' ')
if [ -n "$L" ]; then
    for p in $L; do kill -INT "$p" 2>/dev/null; done
    for i in $(seq 1 120); do [ -z "$(comm_pids rtabmap)" ] && break; sleep 1; done
fi
R=$(comm_pids rtabmap | tr '\n' ' ')
if [ -n "$R" ]; then
    echo "   rtabmap still running ($R) after 120 s - SIGTERM"
    for p in $R; do kill -TERM "$p"; done; sleep 10
fi
[ -z "$(comm_pids rtabmap)" ] && echo "   rtabmap stopped" || echo "   !! rtabmap STILL running: $(comm_pids rtabmap)"
grep -E 'Saving memory|Closing|done!' "$REC/mapping.log" 2>/dev/null | tail -2 | sed 's/^/   log: /'

echo "== 2. fusion pieces"
if [ -r "$REC/fusion_pids" ] && grep -q '^BRIDGE_PID=' "$REC/fusion_pids"; then
    bash "$TOOLS/stop_fusion.sh" "$RUN" 2>&1 | sed 's/^/   /'
else
    echo "   none (camera-only session)"
fi

echo "== 3. camera guard, then the camera"
for i in $(seq 1 30); do [ -z "$(pids_arg camera_guard.py)" ] && break; sleep 1; done
G=$(pids_arg camera_guard.py | tr '\n' ' ')
[ -n "$G" ] && { echo "   camera guard still up ($G) - SIGTERM"; for p in $G; do kill -TERM "$p"; done; sleep 2; }
if [ "$KEEPCAM" = "--keep-camera" ]; then
    echo "   camera left running (--keep-camera)"
else
    C=$(pids_arg zedx_front.launch | tr '\n' ' ')
    for p in $C; do kill -INT "$p" 2>/dev/null; done
    for i in $(seq 1 30); do [ -z "$(pids_arg zedx_front.launch)$(comm_pids zed_wrapper_nod)" ] && break; sleep 1; done
    for p in $(pids_arg zedx_front.launch) $(comm_pids zed_wrapper_nod); do echo "   camera pid $p still up - SIGTERM"; kill -TERM "$p"; done
    sleep 3
    [ -z "$(pids_arg zedx_front.launch)$(comm_pids zed_wrapper_nod)" ] && echo "   camera stopped" \
        || echo "   !! camera still running: $(pids_arg zedx_front.launch) $(comm_pids zed_wrapper_nod)"
fi

echo "== 4. the watcher"
W=$(pids_arg localise_watch.py | tr '\n' ' ')
[ -n "$W" ] && for p in $W; do kill -INT "$p"; done
sleep 2
[ -z "$(pids_arg localise_watch.py)" ] && echo "   stopped" || echo "   !! still running: $(pids_arg localise_watch.py)"

echo "== 5. P4: the master map is untouched"
if [ -r "$REC/master.sha256" ]; then
    if sha256sum -c --status "$REC/master.sha256"; then
        echo "   PASS: master checksum unchanged ($(cut -c1-16 "$REC/master.sha256")...)"; P4=PASS
    else
        echo "   FAIL: the master's checksum CHANGED"; P4=FAIL
    fi
else
    echo "   no $REC/master.sha256 - cannot check"; P4=UNKNOWN
fi
LASTDB=$(tail -1 "$REC/starts.csv" 2>/dev/null | cut -d, -f4)
NM=$(python3 -c "import sqlite3,sys;print(sqlite3.connect('file:'+sys.argv[1]+'?mode=ro',uri=True).execute('select count(*) from Node').fetchone()[0])" "$MAP_DB" 2>/dev/null)
NW=$(python3 -c "import sqlite3,sys;print(sqlite3.connect('file:'+sys.argv[1]+'?mode=ro',uri=True).execute('select count(*) from Node').fetchone()[0])" "$LASTDB" 2>/dev/null)
echo "   nodes: master $NM, last working copy $NW (equal = localisation added nothing permanent)"
echo "$(date '+%F %T %Z') P4 checksum $P4; nodes master=$NM last_copy=$NW" >> "$REC/P4_map_untouched.txt"

echo "== 6. working copies (scratch) removed"
if [ -d "$REC/work" ]; then
    du -sh "$REC/work" | sed 's/^/   /'
    for f in "$REC"/work/start*.db; do
        [ -e "$f" ] || continue
        [ "$(readlink -f "$f")" = "$(readlink -f "$MAP_DB")" ] && { echo "   !! $f IS the master - kept"; continue; }
        rm -f -- "$f" "$f-journal" "$f-wal" "$f-shm"
    done
    rmdir "$REC/work" 2>/dev/null && echo "   removed $REC/work"
fi
echo "== summary: $REC/localise_summary.json, events $REC/localise_events.csv"
python3 - "$REC/localise_summary.json" <<'EOF' 2>/dev/null
import json, sys
d = json.load(open(sys.argv[1]))
for s in d["starts"]:
    print("   start %s %-10s first fix: %s after %s s, %s m driven; fixes %s (recognised %s); suspect jumps %s"
          % (s["start"], s["mark"], s["first_fix_kind"] or "NONE", s["first_fix_s"], s["first_fix_odom_m"],
             s["fixes"], s["global"], s["suspect_jumps"]))
EOF
