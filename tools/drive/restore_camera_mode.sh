#!/bin/bash
# restore_camera_mode.sh - after an SVO=1 drive, put the camera back to its DEFAULT recording mode
# (the yaml's svo_compression 0 = LOSSLESS), so the depth study's recordings are never made in the SLAM
# drive's lossy mode by accident (ENGINEERING_NOTES.md rule 16; review round 1, item 3).
#
#   usage:  restore_camera_mode.sh             restore now (refuses while a drive is mapping)
#           restore_camera_mode.sh --dry-run   say what it would do, change nothing
#           restore_camera_mode.sh --selftest  the process checks against names known to be ABSENT
#
# WHY: the camera reads its recording mode only when it starts (zed_wrapper_nodelet.cpp:1178), so after
# an SVO=1 drive it keeps running in mode 2 (H265, lossy). The depth study's ~/s2_camera.sh starts a recording
# in whatever mode is running and does not check. This restarts the camera WITHOUT the svo_compression
# argument, so it comes back with the yaml's 0, and checks the running value afterwards. It relaunches
# with depth_mode:=NEURAL, exactly as start_drive.sh step 2 does, so nothing else about the camera changes.
# It REFUSES while mapping runs, while any $RECORDS_DIR/*/*.svo2 grew in the last 10 s, or while the
# camera's /diagnostics "SVO Recording" entry reads anything but NOT ACTIVE (or is missing).
# It also writes the state to $WORK_DIR/camera_svo_mode.txt (one line; start_drive.sh writes the
# same file when it starts the camera in a non-default mode). The depth study's scripts are NOT edited.
#
# *Plain terms: the drive borrows the camera with a different recording setting; this hands it back
#  with the setting the depth study expects, and checks that it took.*
#
# Process checks: /proc/<pid>/comm and whole-argument matches, never pgrep -f (ENGINEERING_NOTES.md rule 8).
# repository folders and addresses (REPO_ROOT, RECORDS_DIR, WORK_DIR, JOBS_DIR, TOOLS_DIR, *_ADDR): run/lib/paths.sh
. "$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)/../../run/lib/paths.sh"
set -u
say() { echo "$(TZ=America/Toronto date '+%F %T %Z') $*"; }
STATE="$WORK_DIR/camera_svo_mode.txt"
comm_running() {   # exact process name, whole-word compare
    ps -eo comm= | awk -v n="$1" '$1 == n {f=1} END {exit !f}'
}
camera_launch_pids() {   # roslaunch processes whose arguments include exactly "zedx_front.launch"
    local d A i
    for d in /proc/[0-9]*; do
        mapfile -d '' -t A < "$d/cmdline" 2>/dev/null || continue
        [ "${#A[@]}" -ge 3 ] && [ "$(basename -- "${A[1]}")" = roslaunch ] || continue
        for i in "${A[@]:2}"; do [ "$i" = zedx_front.launch ] && { echo "${d#/proc/}"; break; }; done
    done
}
if [ "${1:-}" = "--selftest" ]; then
    comm_running no_such_prog_7f3a && r1=yes || r1=no
    echo "absent name reported running: $r1 (must be no)"; [ "$r1" = no ]; exit $?
fi
DRY=0; [ "${1:-}" = "--dry-run" ] && DRY=1
set +u; source "$SIDEWALK_ENV"; set -u

comm_running rtabmap && { say "REFUSED: a mapping program (rtabmap) is running - restore only after the drive has closed"; exit 1; }
# a camera recording still running must be stopped first, never cut off by a restart (review round 2, item 3):
# (a) any drive's SVO2 file that grew in the last 10 s
svo_sizes() { local f; for f in "$HOME"/.run_records/*/*.svo2; do [ -e "$f" ] && echo "$f $(stat -c %s "$f")"; done; }
S1=$(svo_sizes); sleep 10; S2=$(svo_sizes)
if [ "$S1" != "$S2" ]; then
    say "REFUSED: a camera recording is still growing:"; diff <(echo "$S1") <(echo "$S2") | sed 's/^/   /'
    say "stop it first (svo_guard.sh --stop <run>, or rosservice call /zedx_front/zed_node/stop_svo_recording)"; exit 1
fi
# (b) the camera's own diagnostics entry "SVO Recording" (zed_wrapper_nodelet.cpp:4618-4643): ACTIVE,
#     No entry within 6 s = cannot tell, so refuse too (a restart must never cut a recording off).
DIAG=$(timeout 6 rostopic echo /diagnostics 2>/dev/null | grep -A1 'key: "SVO Recording"' | grep 'value:' | head -1)
if [ -z "$DIAG" ]; then
    say "REFUSED: no \"SVO Recording\" entry on /diagnostics within 6 s - cannot confirm the camera is not recording"; exit 1
fi
if ! echo "$DIAG" | grep -q '"NOT ACTIVE"'; then
    say "REFUSED: the camera reports an SVO recording in progress ($DIAG) - stop it first"; exit 1
fi
CM=$(rosparam get /zedx_front/zed_node/general/svo_compression 2>/dev/null || echo unset)
if [ "$CM" = 0 ]; then
    say "camera already in mode 0 (default) - nothing to do"
    [ "$DRY" = 1 ] || echo "$(TZ=America/Toronto date '+%F %T %Z') mode 0 (default) - checked, no restart needed" > "$STATE"
    exit 0
fi
PIDS=$(camera_launch_pids)
N=$(echo "$PIDS" | grep -c . || true)
say "running camera mode: $CM; camera launch process(es): ${PIDS:-none}"
[ "$N" = 1 ] || { say "REFUSED: expected exactly one camera roslaunch, found $N - look by hand"; exit 1; }
if [ "$DRY" = 1 ]; then
    say "DRY RUN: would SIGINT roslaunch $PIDS, wait for zed_wrapper_node to exit, relaunch zedx_front.launch depth_mode:=NEURAL (no svo_compression), and check the mode reads 0"
    exit 0
fi
kill -INT "$PIDS"
for i in $(seq 1 60); do comm_running zed_wrapper_nod || break; sleep 1; done
comm_running zed_wrapper_nod && { say "!! the camera program is still running 60 s after SIGINT - NOT killed; look by hand"; exit 1; }
say "camera stopped; relaunching in the default mode"
DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority setsid nohup \
    roslaunch sidewalk_perception zedx_front.launch depth_mode:=NEURAL \
    > "$RECORDS_DIR/camera_restore_$(date +%Y%m%d_%H%M%S).log" 2>&1 < /dev/null &
for i in $(seq 1 18); do timeout 5 rostopic echo -n1 /zedx_front/zed_node/rgb/camera_info >/dev/null 2>&1 && break; done
CM=$(rosparam get /zedx_front/zed_node/general/svo_compression 2>/dev/null || echo unset)
if [ "$CM" = 0 ] && timeout 8 rostopic echo -n1 /zedx_front/zed_node/rgb/camera_info >/dev/null 2>&1; then
    say "RESTORED: camera publishing, svo_compression = 0"
    echo "$(TZ=America/Toronto date '+%F %T %Z') mode 0 (default) - restored after a SLAM drive" > "$STATE"
    exit 0
fi
say "!! NOT RESTORED: svo_compression reads '$CM' or no pictures - the depth study must NOT record until fixed"
echo "$(TZ=America/Toronto date '+%F %T %Z') !! NOT RESTORED (mode '$CM') - paper must not record" > "$STATE"
exit 1
