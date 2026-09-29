#!/bin/bash
# keeps live_site_server.py (the Jetson-served live website, port 8097) alive; run by cron every minute and at boot
# repository folders and addresses (REPO_ROOT, RECORDS_DIR, WORK_DIR, JOBS_DIR, TOOLS_DIR, *_ADDR): run/lib/paths.sh
. "$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)/../../run/lib/paths.sh"
SERVER=${1:-live_site_server.py}   # the program's file name; an argument only for testing the check
# Running check: a python3 process whose first argument has exactly this base name. Each argument
# is compared WHOLE, never searched as text (a text search matches its own command line).
running() {
  local p first
  for p in $(ps -eo pid=,comm= | awk '$2=="python3"{print $1}'); do
    first=$(tr '\0' '\n' < "/proc/$p/cmdline" 2>/dev/null | sed -n 2p)
    [ "${first##*/}" = "$1" ] && return 0
  done
  return 1
}
running "$SERVER" && exit 0
[ -n "$1" ] && { echo "not running: $SERVER"; exit 1; }    # test mode: report, start nothing
setsid nohup python3 "$TOOLS_DIR/live_site_server.py" >> "$JOBS_DIR/live_site_server.log" 2>&1 < /dev/null &
echo "$(TZ=America/Toronto date '+%F %T') started live_site_server" >> "$JOBS_DIR/live_site_server.log"
