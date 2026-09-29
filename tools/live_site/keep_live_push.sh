#!/bin/bash
# keep_live_push.sh - keep slam_live_push.py (the Jetson -> public status/map page sender) running.
# it reboots every time"). Starts the sender only if no process has that exact file name as a
# whole argument (never a substring match - the pgrep -f trap, ENGINEERING_NOTES.md rule 8).
# repository folders and addresses (REPO_ROOT, RECORDS_DIR, WORK_DIR, JOBS_DIR, TOOLS_DIR, *_ADDR): run/lib/paths.sh
. "$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)/../../run/lib/paths.sh"
T=$TOOLS_DIR/slam_live_push.py
LOG=$JOBS_DIR/keep_live_push.log
running(){ local want="$1"
  for p in $(ls /proc | grep -x '[0-9]*'); do
    [ "$p" = "$$" ] && continue; [ "$p" = "$PPID" ] && continue
    { while IFS= read -r -d "" a; do [ "${a##*/}" = "$want" ] && return 0; done < /proc/$p/cmdline; } 2>/dev/null
  done; return 1; }
running __a_name_that_cannot_be_running__ && { echo "$(TZ=America/Toronto date '+%F %T') check broken, nothing started" >> $LOG; exit 1; }
running slam_live_push.py && exit 0
cd $HOME && TZ=America/Toronto setsid nohup nice -n 19 python3 "$T" >> $JOBS_DIR/slam_live_push.log 2>&1 < /dev/null &
echo "$(TZ=America/Toronto date '+%F %T') started slam_live_push.py (pid $!)" >> $LOG
