#!/bin/bash
# next_start.sh - begin the next localisation START on a new mark, without restarting anything else.
#
#   usage:  next_start.sh <run> <k> <mark>      e.g.  next_start.sh s2_loc_01 2 A_east
#
# WHAT IT DOES (the robot parked on the mark, hands off)
#   1. copies the MASTER map to a fresh working copy  $REC/work/start<k>.db  (compared byte for byte)
#   2. tells the running localiser to drop everything it knows about where the robot is and load that
#      copy: rosservice /rtabmap/load_database, clear: false. clear is ALWAYS false - clear:true ERASES
#      the file it is given (CoreWrapper.cpp loadDatabaseCallback, UFile::erase), and the master's folder
#      is writable, so this script refuses any path that resolves to the master.
#   3. appends "k,mark,time,db" to $REC/starts.csv - the watcher starts counting this start from here
#   4. deletes the PREVIOUS start's working copy (a scratch file: localisation adds nothing to it;
#      ENGINEERING_NOTES.md rule 9). The master is untouched throughout.
#
# *Plain terms: the camera, the blend and the robot link keep running; only the map program "forgets
# where it is" and starts over from its default guess (drive 4's last saved position, RGBD/StartAtOrigin=false - on the start mark). Whether it then finds
# the robot on the NEW mark is the test.*
#
# Why a fresh copy each time: on closing, RTAB-Map saves "where the robot was last" into the database
# (Rtabmap.cpp saveOptimizedPoses(..., _lastLocalizationPose)), and loads it as its next guess. A reused
# copy would start the next start from the previous answer - not a fresh start.
# repository folders and addresses (REPO_ROOT, RECORDS_DIR, WORK_DIR, JOBS_DIR, TOOLS_DIR, *_ADDR): run/lib/paths.sh
. "$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)/../paths.sh"
set -u
RUN="${1:?usage: next_start.sh <run> <k> <mark>}"
K="${2:?usage: next_start.sh <run> <k> <mark>}"
MARK="${3:?usage: next_start.sh <run> <k> <mark>}"
REC="$RECORDS_DIR/$RUN"
MAP_DB="${MAP_DB:-$WORK_DIR/localise/s2_static_04_map.db}"
die() { echo "   REFUSED: $*" >&2; exit 1; }
set +u; source "$SIDEWALK_ENV"; set -u

[ -r "$REC/starts.csv" ] || die "$REC/starts.csv missing - start the session with start_localise.sh first"
case "$K" in ''|*[!0-9]*) die "k must be a number";; esac
LAST=$(tail -1 "$REC/starts.csv" | cut -d, -f1)
[ "$K" -eq $((LAST + 1)) ] || die "the last start was $LAST, so this one is $((LAST + 1)), not $K"
RT=$(ps -eo pid=,comm= | awk '$2 == "rtabmap" {print $1}')
[ -n "$RT" ] || die "no rtabmap process is running"
NEW="$REC/work/start$K.db"
OLD=$(tail -1 "$REC/starts.csv" | cut -d, -f4)
[ "$(readlink -f "$NEW")" = "$(readlink -f "$MAP_DB")" ] && die "the new working copy path IS the master"
[ -e "$NEW" ] && die "$NEW already exists"

T0=$(date +%s.%N)
cp "$MAP_DB" "$NEW" && chmod u+w "$NEW" || die "copy failed"
cmp -s "$MAP_DB" "$NEW" || die "the fresh copy differs from the master"
# KIDNAPPED RESET (added after session 2): put a deliberately WRONG, OFF-MAP "last localisation" into the
# COPY, so the reload's guess is nowhere near any map node (49 m from the nearest; RGBD/LocalRadius is 5.0 m in localise_run.launch, from rtabmap_zedx.yaml:646).
# Then nearby re-matches cannot fire and the dot is visibly off the map until a real recognition is ACCEPTED.
# Session 2 used the saved guess (the start mark), which looked plausible and was 4-11 m wrong (RESULTS.md).
# KIDNAP_PRIOR="" restores the old behaviour.
KIDNAP_PRIOR="${KIDNAP_PRIOR-40 40 90}"
if [ -n "$KIDNAP_PRIOR" ]; then
    python3 "$(dirname "$(readlink -f "$0")")/set_prior.py" "$NEW" $KIDNAP_PRIOR || die "could not set the kidnapped prior"
fi
T1=$(date +%s.%N)
echo "   fresh copy $NEW ($(awk -v a="$T0" -v b="$T1" 'BEGIN{printf "%.1f", b-a}') s)${KIDNAP_PRIOR:+, guess set OFF the map at ($KIDNAP_PRIOR)}"
OUT=$(timeout 300 rosservice call /rtabmap/load_database "{database_path: '$NEW', clear: false}" 2>&1)
RC=$?
T2=$(date +%s.%N)
[ $RC -eq 0 ] || die "load_database failed (rc $RC): $OUT - the localiser keeps its previous map; see $REC/mapping.log"
# reload was counted to the previous start because this line was written after the log check below)
echo "$K,$MARK,$T2,$NEW" >> "$REC/starts.csv"
# the log line can reach mapping.log a few seconds after the service answers (roslaunch output buffering)
LM=""
for i in $(seq 1 15); do
    tail -5000 "$REC/mapping.log" | grep -a -q 'LoadDatabase: Localization mode' && { LM=1; break; }
    sleep 1
done
[ -n "$LM" ] || echo "   !! mapping.log does not show 'LoadDatabase: Localization mode' after 15 s - check it"
echo "   start $K ($MARK) - map reloaded in $(awk -v a="$T1" -v b="$T2" 'BEGIN{printf "%.1f", b-a}') s; the watcher counts from now"
grep -a -E 'LoadDatabase: Working Memory' "$REC/mapping.log" | tail -1 | sed 's/^/   log: /'
if [ -n "$OLD" ] && [ "$OLD" != "$NEW" ] && [ -e "$OLD" ] \
        && [ "$(readlink -f "$OLD")" != "$(readlink -f "$MAP_DB")" ]; then
    rm -f -- "$OLD" "$OLD-journal" && echo "   removed the previous working copy $OLD (scratch)"
fi
echo "   now: stand still, hands off; watch ${RUN}_localise on $JOBS_PAGE_URL"
