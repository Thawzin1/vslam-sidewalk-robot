#!/bin/bash
# watch_run.sh — one compact state line for a live mapping run, PLUS anything new
# and alarming in the camera and mapping logs since the last check.
#
# Called once a minute by a watcher on another machine (ENGINEERING_NOTES.md section 0.3 rule 8:
# every job that outlives one command gets an armed 1-minute watcher that fires
# on failure and silent death, not only on success).
#
# READ-ONLY on everything that matters. It inspects; it never touches the camera,
# the database, or any running node. It keeps one small cursor file per run under
# /tmp so it can report only what is NEW since the previous poll.
#
#   Run 5's camera died with a segmentation fault and the root cause could NOT be
#   determined, because the camera's output went to a terminal and was lost. From
#   run 6 the run procedure tees both the camera and the mapping stack to files:
#       $RECORDS_DIR/<run_id>/camera.log
#       $RECORDS_DIR/<run_id>/mapping.log
#   This script surfaces new errors from them within a minute of appearing, so a
#   recurrence is diagnosed from evidence instead of guessed at afterwards.
#
# Lives in the repo, NOT in /tmp — a Jetson reboot wiped the first copy of this
# file mid-session, right as a run was starting.
#
# Uses `ps -eo comm` exact matching, NEVER `pgrep -f`: that self-matches its own
# command line and has reported a dead job as alive three times in this project.
# NOTE the process name is truncated to 15 characters by Linux, so the camera is
# `zed_wrapper_nod` with no trailing "e" — a soak test once reported the camera
# dead while it was happily publishing at 14.1 Hz because of exactly this.
#
#   usage:  bash watch_run.sh <run_id>        e.g.  bash watch_run.sh lab_map_06

RUN=${1:-lab_map_06}
REC="$RECORDS_DIR/$RUN"
CAMLOG="$REC/camera.log"
MAPLOG="$REC/mapping.log"
CUR="/tmp/watch_${RUN}_cursor"

alive() { ps -eo comm --no-headers | grep -qx "$1"; }

# ---------- stack state ----------
if alive rtabmap && alive zed_wrapper_nod; then STACK=up
elif alive rtabmap || alive zed_wrapper_nod; then STACK=PARTIAL
else STACK=down; fi

# ---------- run health, from the monitor's own status line ----------
S="$REC/monitor_STATUS.txt"
if [ -f "$S" ]; then
  T=$(grep -oP 't=\K[0-9]+' "$S" 2>/dev/null | head -1)
  RESETS=$(grep -oP 'resets=\K[0-9]+' "$S" 2>/dev/null | head -1)
  LC=$(grep -oP 'ACCEPTED_LC=\K[0-9?]+' "$S" 2>/dev/null | head -1)
else
  T=""; RESETS=""; LC=""
fi

FRAMES=$(find "$REPO_ROOT/logs/sidewalk_slam/maps/timelapse/$RUN" -name '*.pgm' 2>/dev/null | wc -l)
IDISK=$(df -BG --output=avail / 2>/dev/null | tail -1 | tr -dc '0-9')
SD=$(df -BG --output=avail /media/sidewalk/SIDEWALK128 2>/dev/null | tail -1 | tr -dc '0-9')

# ---------- alert FLAGS, not raw numbers ----------
# Keying the watcher on raw numbers would fire every minute as they tick, and a
# watcher that cries wolf gets auto-stopped.
ALERT=""
[ -n "$RESETS" ] && [ "$RESETS" -gt 0 ] 2>/dev/null && ALERT="${ALERT}RESETS "
[ -n "$IDISK" ] && [ "$IDISK" -lt 4 ] 2>/dev/null && ALERT="${ALERT}INTERNAL_DISK_LOW "
[ -n "$SD" ] && [ "$SD" -lt 15 ] 2>/dev/null && ALERT="${ALERT}CARD_LOW "
# The card unmounting mid-run would be silent and fatal — it is where the map goes.
findmnt -n /media/sidewalk/SIDEWALK128 >/dev/null 2>&1 || ALERT="${ALERT}CARD_UNMOUNTED "

echo "stack=$STACK alert=${ALERT:-none} | t=${T:-?}s resets=${RESETS:-?} closures=${LC:-?} frames=$FRAMES internal=${IDISK}GB card=${SD}GB"

# ---------- what is NEW in the camera log ----------
# Only lines that would change what we do: the divergence that preceded run 5's
# crash, the death banner, and hard errors. Everything else is start-up noise.
if [ -f "$CAMLOG" ]; then
  LAST=$(cat "${CUR}_cam" 2>/dev/null || echo 0)
  NOW=$(wc -l < "$CAMLOG" 2>/dev/null || echo 0)
  if [ "$NOW" -gt "$LAST" ] 2>/dev/null; then
    NEW=$(tail -n +$((LAST+1)) "$CAMLOG" 2>/dev/null \
          | grep -iE "diverged|has died|SIGSEGV|ERROR|CAMERA FAILED|Cannot initialize|Argus.*Error" \
          | grep -viE "sl::ERROR_CODE sl::Camera::open" \
          | head -6)
    [ -n "$NEW" ] && { echo "  --- NEW IN camera.log ---"; echo "$NEW" | cut -c1-150 | sed 's/^/  /'; }
    echo "$NOW" > "${CUR}_cam"
  fi
fi

# ---------- the texture problem, measured live ----------
# Run 5's one real correlation: RTAB-Map's feature count collapsed 220 -> 78 at
# the moment the camera's tracker diverged, six times in twenty minutes. Low
# feature counts are where visual SLAM drifts, so surface them as they happen.
if [ -f "$MAPLOG" ]; then
  LASTM=$(cat "${CUR}_map" 2>/dev/null || echo 0)
  NOWM=$(wc -l < "$MAPLOG" 2>/dev/null || echo 0)
  if [ "$NOWM" -gt "$LASTM" ] 2>/dev/null; then
    Q=$(tail -n +$((LASTM+1)) "$MAPLOG" 2>/dev/null | grep -oE 'quality=[0-9]+' | grep -oE '[0-9]+')
    if [ -n "$Q" ]; then
      STATS=$(echo "$Q" | awk '{n++; s+=$1; if($1<100) low++; if(min==""||$1<min) min=$1}
                               END {printf "features: mean %.0f  min %d  below-100 %d of %d", s/n, min, low+0, n}')
      echo "  $STATS"
      LOWC=$(echo "$Q" | awk '$1<100{c++} END{print c+0}')
      [ "$LOWC" -gt 0 ] 2>/dev/null && echo "  ^ texture-poor stretch — prefer areas with more visual detail"
    fi
    echo "$NOWM" > "${CUR}_map"
  fi
fi

# ---------- if the camera died, capture its last words immediately ----------
if [ "$STACK" = "PARTIAL" ] && ! alive zed_wrapper_nod && [ -f "$CAMLOG" ]; then
  echo "  === CAMERA DIED — last 12 lines of camera.log ==="
  tail -12 "$CAMLOG" | cut -c1-150 | sed 's/^/  /'
  echo "  === the map is still in memory. Keep the robot STILL and restart terminal 1 only. ==="
fi
