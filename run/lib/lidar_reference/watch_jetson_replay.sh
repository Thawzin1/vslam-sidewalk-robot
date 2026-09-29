#!/bin/bash
# watch_jetson_replay.sh - the rule-8 watcher for a Jetson LiDAR replay: prints ONE line on every state change,
# including failure and SILENT death (progress line older than 3 min while not finished, or the map program
# gone while replaying). Meant to be run under the Monitor tool. Exits when the replay completes or fails.
#   usage: watch_jetson_replay.sh <run> [variant=f3dof]
# repository folders and addresses (REPO_ROOT, RECORDS_DIR, WORK_DIR, JOBS_DIR, TOOLS_DIR, *_ADDR): run/lib/paths.sh
. "$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)/../paths.sh"
RUN="$1"; VAR="${2:-f3dof}"
P="$JOBS_DIR/${RUN}_lidar_ref_${VAR}_jetson.progress"; last=""
while :; do
    line=$(cat "$P" 2>/dev/null); age=$(( $(date +%s) - $(stat -c %Y "$P" 2>/dev/null || echo 0) ))
    phase=$(echo "$line" | grep -oE "complete|FAILED|STOPPED|closing|making|replaying" | head -1)
    rt=""
    for p in $(ps -eo pid=,comm= | awk '$2=="rtabmap"{print $1}'); do
        tr '\0' '\n' 2>/dev/null < "/proc/$p/environ" | grep -qx "ROS_MASTER_URI=http://localhost:11350" && rt="rtabmap $p"
    done
    st="${phase:-waiting}"
    [ "$age" -gt 180 ] && [ "$phase" != complete ] && [ "$phase" != FAILED ] && [ "$phase" != STOPPED ] && st="STALLED (progress ${age}s old)"
    [ "$phase" = replaying ] && [ -z "$rt" ] && st="MAP PROGRAM DEAD while replaying"
    if [ "$st" != "$last" ]; then echo "$(TZ=America/Toronto date +%H:%M) $RUN $VAR: $st | $line"; last="$st"; fi
    case "$phase" in complete|FAILED|STOPPED) exit 0 ;; esac
    sleep 60
done
