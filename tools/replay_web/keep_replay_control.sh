#!/bin/bash
# keep_replay_control.sh - keeps replay_control_server.py (the Replay page's backend, 127.0.0.1:8098) running.
#
# Run by cron every minute and once after boot. If the backend is already running it does nothing;
# otherwise it starts it in a session of its own (setsid) with its output appended to
# $JOBS_DIR/replay_control_server.log. Replay jobs themselves are separate processes and are not
# touched by this script.
#
# Settings (environment, all optional): REPO_ROOT (default: two folders above this script),
# JOBS_DIR (default ~/jobs), and every setting listed in replay_common.py - they are passed on.
#
# The running check compares each python3 process's first argument WHOLE with this folder's
# replay_control_server.py - never a text search, which would also match this script's own
# command line or an editor that has the file open.
HERE="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
SERVER="$HERE/replay_control_server.py"
export REPO_ROOT="${REPO_ROOT:-$(cd "$HERE/../.." && pwd)}"
JOBS_DIR="${JOBS_DIR:-$HOME/jobs}"
mkdir -p "$JOBS_DIR"
for p in $(ps -eo pid=,comm= | awk '$2=="python3"{print $1}'); do
    first_arg="$(tr '\0' '\n' < "/proc/$p/cmdline" 2>/dev/null | sed -n 2p)"
    [ "$first_arg" = "$SERVER" ] && exit 0
done
cd "$HERE" || exit 1
setsid nohup python3 "$SERVER" >> "$JOBS_DIR/replay_control_server.log" 2>&1 < /dev/null &
echo "$(TZ=America/Toronto date '+%F %T') started replay_control_server (repository $REPO_ROOT)" >> "$JOBS_DIR/replay_control_server.log"
