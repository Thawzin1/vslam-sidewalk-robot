#!/bin/bash
#
# WHAT IT IS: the Jetson form of the robot's tools/replay_colleague_variant.sh "wrap <variant>" mode
# (byte-exact copy in robot_copy/, md5 d50d096a109ffcec30a0cb6782767387). Same pipeline, same parameters,
# same products, so the robot can charge while a replay runs.
#   pipeline : the colleague's self_navigation 3dreplay_pipeline.launch sensor:=helios (unchanged, credited:
#              Matt Jing's self_navigation package on the robot, ENGINEERING_NOTES.md rule 18), started through
#              variants/<variant>.launch. f3dof = the robot's own f3dof.launch (included byte for byte) =
#              the colleague's pipeline + ONE change, Reg/Force3DoF=true (the map may only slide in x, y and
#              turn; it may not believe the robot tipped). colleague = no change at all.
#   input    : ~/lidar_bags/<run>_lidar.bag (Jetson internal disk), copied off the robot card by
#              pull_bag_from_robot.sh (sha256-checked). Override the folder with LIDAR_BAG_DIR.
#   output   : ~/lidar_replays/<run>/lidar_ref_<variant>/ (Jetson internal disk). Override with LIDAR_REPLAY_DIR.
#              rtab_helios.db, lidar.tum (+ .meta.json), wheel.tum (+ .meta.json), lidar_map.png/.npz,
#              lidar_map_facts.json, products.log, provenance.txt, replay.log, play.log, replay_core.log,
#              rtabmap_params_db.txt (what the map program actually used), SHA256SUMS.
#
# WHAT DIFFERS FROM THE ROBOT, and only this (every line is recorded in provenance.txt):
#   1. RTAB-Map is 0.21.13 (Jetson, apt) instead of 0.21.10 (robot). Same parameters are SET; a default that
#      changed between the two versions would still differ - rtabmap_params_db.txt lets validate_against_robot.py
#      list every such difference against the robot's own map.
#   2. The Helios decoder reads robot_replay_config.yaml (see its header: the robot's decoder settings,
#      split_angle 180) - the Jetson's mirror copy of the colleague's config lacks that line.
#   3. The ROS environment is ~/lidar_slam_ws/setup_lidar_slam.bash (rslidar_sdk built from the robot mirror,
#      self_navigation + mcm07_husky copied unchanged, the body filter unpacked in the home folder).
#   4. An isolated ROS master on port 11350 (the robot uses 11312; on the Jetson 11311 is the drives' master).
#   5. rosbag play also runs at nice 19 (the robot: normal priority); the pipeline at nice 19 as on the robot.
#   6. The robot's "reference" mode (lidar_reference.launch) is not carried over - it is not the yardstick.
#   7. JETSON SAFETY (rule 23: the Jetson is the drive computer): it refuses to start while a drive or a
#      localisation session is live, and if one starts mid-replay it stops the replay (bag playback
#      interrupted, the partial map closed, the folder renamed .stopped-<time>). Same if
#      $WORK_DIR/HELPERS_STOP appears.
#   8. A failed or stopped attempt's folder is RENAMED (.failed-/.stopped-<time>), never removed, so a re-run
#      starts clean (ENGINEERING_NOTES.md rule 14) - the robot's colleague mode refused instead.
#   9. Free space: 3 GB demanded on the output disk before a map is written (the largest LiDAR map so far was
#      1021 MB; a map's final size is not known when it starts, so it reserves like a recording - rule 15).
#
# THE TRAPS THE ROBOT SCRIPT GUARDS AGAINST ARE KEPT: /use_sim_time true ONLY on the isolated master (or
# RTAB-Map writes an empty map without an error); the map is closed by SIGINT to the exact rtabmap PID on
# this master and waited for with NO deadline (its corrected path is written only at shutdown); process
# checks compare whole names from ps -eo comm, never pgrep -f / substring (ENGINEERING_NOTES.md rule 8).
#
# WHAT THE RESULT IS: an independent LiDAR estimate - wheel+IMU odometry corrected by LiDAR loop closures -
# NOT ground truth (ENGINEERING_NOTES.md rule 19). Say "agreement with the LiDAR estimate".
#
#   usage:  ./replay_colleague_variant_jetson.sh <run> wrap f3dof          e.g. s2_static_10 wrap f3dof
#           ./replay_colleague_variant_jetson.sh <run> wrap colleague
#           RATE=0.5 ./replay_colleague_variant_jetson.sh <run> wrap f3dof   (slower, if clouds were dropped)
#   progress: $JOBS_DIR/<run>_lidar_ref_<variant>_jetson.progress (jobs page :8096)
# repository folders and addresses (REPO_ROOT, RECORDS_DIR, WORK_DIR, JOBS_DIR, TOOLS_DIR, *_ADDR): run/lib/paths.sh
. "$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)/../paths.sh"
set -u
RUN="${1:?usage: replay_colleague_variant_jetson.sh <run_id> wrap <variant>}"
MODE="${2:-}"
VARIANT="${3:-}"
[ "$MODE" = wrap ] || { echo "only mode 'wrap <variant>' exists on the Jetson (variants: $(ls "$(dirname "$0")/variants" | sed 's/\.launch//' | tr '\n' ' '))" >&2; exit 2; }
[[ "$VARIANT" =~ ^[A-Za-z0-9_]+$ ]] || { echo "wrap needs a variant name" >&2; exit 2; }
[[ "$RUN" =~ ^[A-Za-z0-9_]+$ ]] || { echo "run id must be letters, digits, _" >&2; exit 2; }

HERE="$(cd "$(dirname "$0")" && pwd)"
SCRIPTS="$DB_TOOLS_DIR"          # the map-database readers (check_closed, db_corrected_tum, bag_to_tum, render_map)
ENVF="$HOME/lidar_slam_ws/setup_lidar_slam.bash"
BAGDIR="${LIDAR_BAG_DIR:-$HOME/lidar_bags}"
BAG="$BAGDIR/${RUN}_lidar.bag"
OUTROOT="${LIDAR_REPLAY_DIR:-$HOME/lidar_replays}"
OUT="$OUTROOT/$RUN/lidar_ref_$VARIANT"
DB="$OUT/rtab_helios.db"
WRAP="$HERE/variants/$VARIANT.launch"
RATE="${RATE:-1.0}"
NEED_GB=3
PORT=11350
STOPFILE="$WORK_DIR/HELPERS_STOP"
PROG="$JOBS_DIR/${RUN}_lidar_ref_${VARIANT}_jetson.progress"
TAG="${RUN}_LIDAR_REF_${VARIANT}_JETSON"
mkdir -p "$JOBS_DIR"
say() { echo "$1" > "$PROG.tmp" && mv "$PROG.tmp" "$PROG"; echo "$(TZ=America/Toronto date +%H:%M:%S) $1"; }
fail() { say "$TAG  FAILED - $1"; exit 1; }

[ -s "$WRAP" ] || fail "no $WRAP"
[ -s "$ENVF" ] || fail "no $ENVF (the LiDAR workspace; see README.md)"

set +u
source "$ENVF"
set -u
export ROS_MASTER_URI="http://localhost:$PORT"
export ROSCONSOLE_STDOUT_LINE_BUFFERED=1

# A process of THIS replay master: exact name AND this master (never a substring).
replay_pid() {
    local p
    for p in $(ps -eo pid=,comm= | awk -v n="$1" '$2 == n {print $1}'); do
        if tr '\0' '\n' 2>/dev/null < "/proc/$p/environ" | grep -qx "ROS_MASTER_URI=http://localhost:$PORT"; then
            echo "$p"
        fi
    done
}
# Is a drive or a localisation session live on this Jetson? Whole-name matches only (rule 8).
#  - the drive's own programs; - a mapping program on ANY other master; - a drive's autostop line fresh < 120 s.
drive_live() {
    local p n
    while read -r p n; do
        case "$n" in
            start_drive.sh|rgbd_odometry|stereo_odometry|stereo_odometr|rgbd_odometr|localise.sh) echo "$n (pid $p)"; return 0 ;;
            rtabmap)
                tr '\0' '\n' 2>/dev/null < "/proc/$p/environ" | grep -qx "ROS_MASTER_URI=http://localhost:$PORT" \
                    || { echo "a mapping program on another ROS master (pid $p)"; return 0; } ;;
        esac
    done < <(ps -eo pid=,comm=)
    local f
    for f in "$HOME"/jobs/*_autostop.progress; do
        [ -e "$f" ] || continue
        [ $(( $(date +%s) - $(stat -c %Y "$f") )) -lt 120 ] && { echo "fresh $(basename "$f")"; return 0; }
    done
    return 1
}

[ -e "$STOPFILE" ] && fail "$STOPFILE exists - helpers are told to stop; not starting"
L=$(drive_live) && fail "a drive or localisation session is live ($L); the Jetson is the drive computer - not starting (rule 23)"
[ -s "$BAG" ] || fail "no recording at $BAG (copy it with pull_bag_from_robot.sh first)"
[ -z "$(replay_pid rtabmap)" ] || fail "another replay's mapping is still running on the $PORT master; wait for it to finish"
# A folder that already holds this variant's map is refused; one left by an attempt that stopped before
# producing a map is moved aside (renamed), never removed.
[ -e "$DB" ] && fail "$DB already exists; not overwriting it (rename the folder to re-run)"
if [ -e "$OUT" ]; then
    mv "$OUT" "$OUT.failed-$(TZ=America/Toronto date +%Y%m%d-%H%M%S)" || fail "cannot move the old $OUT aside"
fi
mkdir -p "$OUT"
FREE=$(df -BG --output=avail "$OUT" 2>/dev/null | tail -1 | tr -dc 0-9)
[ -n "$FREE" ] || fail "cannot read the free space of $OUT"
[ "$FREE" -ge "$NEED_GB" ] || fail "only ${FREE} GB free under $OUTROOT; a replay's map needs ${NEED_GB} GB reserved"

# ---- provenance (ENGINEERING_NOTES.md section 4 rule 8), written before anything runs
{
  echo "run=$RUN mode=wrap variant=$VARIANT rate=$RATE machine=JETSON port=$PORT"
  echo "started $(TZ=America/Toronto date '+%Y-%m-%d %H:%M:%S %Z') on $(hostname) ($(tr -d '\0' < /proc/device-tree/model 2>/dev/null))"
  echo "bag $BAG $(stat -c %s "$BAG") bytes"
  [ -s "$BAG.sha256" ] && echo "bag sha256 (checked at copy): $(cut -c1-64 "$BAG.sha256")"
  echo "-- RTAB-Map on this machine (the robot's replays used 0.21.10-1focal.20250213)"
  dpkg -l | awk '/ros-noetic-rtabmap(-odom|-slam|-util|-sync)? /{print $2, $3}'
  QT_QPA_PLATFORM=offscreen rtabmap --version 2>&1 | grep -E "RTAB-Map:|With (libpointmatcher|GTSAM|g2o|OctoMap)" | sed 's/^/  /'
  echo "-- our files"
  md5sum "$0" "$WRAP" "$HERE/robot_copy/f3dof.launch" "$HERE/robot_copy/replay_colleague_variant.sh" \
         "$HERE/robot_replay_config.yaml" "$ENVF" \
         "$SCRIPTS/check_closed.py" "$SCRIPTS/db_corrected_tum.py" "$SCRIPTS/bag_to_tum.py" "$SCRIPTS/render_map.py" 2>&1
  echo "-- the colleague's files as found by this environment (robot's drive-10 provenance, for comparison:"
  echo "   3dreplay_pipeline 744381369c1af0049aae5b6f04c85201, rtabmap_3d 099b269839710d212eca59648c0a2758,"
  echo "   helios_body_filter 2ee62a3c3e405b856e3d8a69be880ac0, helios_restamp 07f5171db81d0559ac06d89f30900126,"
  echo "   box_crop 0568a9c6d92120274cab1a23bbd2dea3, rslidar replay_config cb9ca662ae093bf9763782b3da43ef5d)"
  SN="$(rospack find self_navigation 2>/dev/null)"; MH="$(rospack find mcm07_husky 2>/dev/null)"
  md5sum "$SN/launch/3dreplay_pipeline.launch" "$SN/launch/rtabmap_3d.launch" "$SN/launch/helios_body_filter.launch" \
         "$SN/scripts/helios_restamp.py" "$SN/scripts/box_crop.py" \
         "$MH/config/husky_full.urdf" "$MH/config/robot_body_filter_helios.yaml" 2>&1
  RS="$(rospack find rslidar_sdk 2>/dev/null)"
  echo "rslidar_sdk $RS (rs_driver $(grep -oE 'VERSION [0-9.]+' "$RS/src/rs_driver/CMakeLists.txt" | head -1)); node md5 $(md5sum < "$HOME/lidar_slam_ws/devel/lib/rslidar_sdk/rslidar_sdk_node" | cut -c1-32)"
  echo "decoder config actually used: $HERE/robot_replay_config.yaml"
  grep -n "use_lidar_clock\|ts_first_point\|split_angle" "$HERE/robot_replay_config.yaml"
  echo "body filter: $(rospack find robot_body_filter 2>/dev/null) ($(ls "$HOME"/lidar_slam_ws/deps/debs | grep -E 'robot-body-filter|sensor-filters' | tr '\n' ' '))"
  echo "memory available at start: $(awk '/MemAvailable/{printf "%.1f GB", $2/1048576}' /proc/meminfo); free on output disk: ${FREE} GB"
} > "$OUT/provenance.txt" 2>&1

CORE=""
if ! timeout 5 rostopic list >/dev/null 2>&1; then
    setsid nohup roscore -p "$PORT" > "$OUT/replay_core.log" 2>&1 < /dev/null &
    CORE=$!
    sleep 6
fi
rosparam set /use_sim_time true
[ "$(rosparam get /use_sim_time)" = "true" ] || fail "could not set sim time on the $PORT master"

setsid nohup nice -n 19 roslaunch "$WRAP" db_dir:="$OUT" > "$OUT/replay.log" 2>&1 < /dev/null &
LAUNCH=$!
PLAY=""

# close everything this replay started; the map by SIGINT with NO deadline (trap 2)
close_all() {
    local p
    for p in $(replay_pid rtabmap); do
        kill -INT "$p" 2>/dev/null
        while [ -d "/proc/$p" ] && ! grep -q '^State:.*Z' "/proc/$p/status" 2>/dev/null; do sleep 2; done
    done
    rosnode kill -a > /dev/null 2>&1          # this master only (ROS_MASTER_URI exported above)
    kill -INT "$LAUNCH" 2>/dev/null
    for _ in $(seq 1 30); do
        { [ -d "/proc/$LAUNCH" ] && ! grep -q '^State:.*Z' "/proc/$LAUNCH/status" 2>/dev/null; } || break
        sleep 2
    done
    [ -n "$CORE" ] && kill -INT "$CORE" 2>/dev/null
}
# stop, keep the attempt under a new name so a re-run starts clean, report
abort() {   # $1 = failed|stopped   $2 = why
    [ -n "$PLAY" ] && kill -INT "$PLAY" 2>/dev/null
    say "$TAG  ${1^^} - $2 (closing what was started)"
    close_all
    local moved="$OUT.$1-$(TZ=America/Toronto date +%Y%m%d-%H%M%S)"
    if mv "$OUT" "$moved" 2>/dev/null; then
        say "$TAG  ${1^^} - $2; this attempt kept in $moved - re-running starts clean"
    else
        say "$TAG  ${1^^} - $2; could not rename $OUT - move it aside before re-running"
    fi
    exit 1
}
sleep 14
RT=$(replay_pid rtabmap)
[ -n "$RT" ] || abort failed "the LiDAR mapping did not start; see $OUT/replay.log"
echo "config_path given to the decoder: $(rosparam get /helios_driver/config_path 2>&1)" >> "$OUT/provenance.txt"
echo "Reg/Force3DoF on the map node: $(rosparam get /rtabmap/rtabmap/Reg/Force3DoF 2>&1)" >> "$OUT/provenance.txt"

DUR=$(rosbag info -y -k duration "$BAG" 2>/dev/null | cut -d. -f1); DUR=${DUR:-1200}
nice -n 19 rosbag play "$BAG" --clock -r "$RATE" > "$OUT/play.log" 2>&1 &
PLAY=$!
T0=$(date +%s); LAST=0
while [ -d "/proc/$PLAY" ] && ! grep -q '^State:.*Z' "/proc/$PLAY/status" 2>/dev/null; do
    [ -e "$STOPFILE" ] && abort stopped "$STOPFILE appeared"
    L=$(drive_live) && abort stopped "a drive or localisation session started ($L) - the Jetson is the drive computer"
    [ -d "/proc/$RT" ] || abort failed "the mapping process (pid $RT) died during the replay; see $OUT/replay.log"
    E=$(( $(date +%s) - T0 ))
    if [ $(( E - LAST )) -ge 15 ]; then
        C=$(awk -v e="$E" -v r="$RATE" -v d="$DUR" 'BEGIN{c=int(e*r); if(c>d-1)c=d-1; print c}')
        say "$TAG  ${C}/${DUR} s  replaying at ${RATE}x  ${E}s" > /dev/null
        LAST=$E
    fi
    sleep 5
done
wait "$PLAY" 2>/dev/null
PLAY=""
sleep 10            # let the last scans through the pipeline

# ---- close the map: interrupt by PID, then wait with NO deadline (trap 2)
DIED=""
[ -d "/proc/$RT" ] || DIED="the mapping process had already died during the replay (see replay.log); "
kill -INT "$RT" 2>/dev/null
W0=$(date +%s); NEXT=0
while [ -d "/proc/$RT" ] && ! grep -q '^State:.*Z' "/proc/$RT/status" 2>/dev/null; do
    W=$(( $(date +%s) - W0 ))
    if [ "$W" -ge "$NEXT" ]; then
        NEXT=$(( W + 30 ))
        if [ "$W" -lt 1800 ]; then
            say "$TAG  ${DUR}/${DUR} s  closing the LiDAR map (pid $RT)  ${W}s" > /dev/null
        else
            say "$TAG  ${DUR}/${DUR} s  STILL closing the LiDAR map after $(( W / 60 )) min (pid $RT) - NOT killed, waiting  ${W}s" > /dev/null
        fi
    fi
    sleep 2
done
echo "$(TZ=America/Toronto date +%H:%M:%S) LiDAR map closed after $(( $(date +%s) - W0 )) s"
close_all

say "$TAG  map closed - making the reference products"
{
  nice -n 19 python3 "$SCRIPTS/check_closed.py" "$DB" --lidar
  nice -n 19 python3 "$SCRIPTS/db_corrected_tum.py" "$DB" "$OUT/lidar.tum" --label "LiDAR (Helios), $RUN (Jetson replay, $VARIANT)"
  nice -n 19 python3 "$SCRIPTS/bag_to_tum.py" "$BAG" --topic /odometry/filtered --out "$OUT/wheel.tum"
  nice -n 19 python3 "$SCRIPTS/render_map.py" "$DB" "$OUT" --poses "$OUT/lidar.tum" \
      --zband 0.15 2.0 --cell 0.05 --npz "$OUT/lidar_map.npz" --name lidar_map \
      --title "$RUN - the LiDAR map (walls 0.15-2.0 m up), corrected positions (Jetson replay, $VARIANT)"
  # what the map program actually ran with (RTAB-Map writes every parameter into the map's Info table)
  python3 - "$DB" "$OUT/rtabmap_params_db.txt" <<'PY'
import sqlite3, sys
c = sqlite3.connect("file:%s?mode=ro" % sys.argv[1], uri=True)
rows = [r[0] for r in c.execute("SELECT parameters FROM Info") if r[0]]
with open(sys.argv[2], "w") as fh:
    for kv in sorted(set(p for r in rows for p in r.split(";") if p)):
        fh.write(kv.replace(":", " = ", 1) + "\n")
print("  %d parameters written to %s (from %d Info rows)" % (sum(1 for _ in open(sys.argv[2])), sys.argv[2], len(rows)))
PY
} > "$OUT/products.log" 2>&1
(cd "$OUT" && sha256sum lidar.tum lidar.tum.meta.json wheel.tum wheel.tum.meta.json lidar_map.png lidar_map.npz \
     lidar_map_facts.json rtabmap_params_db.txt provenance.txt > SHA256SUMS 2>/dev/null)

if grep -q "CLOSED PROPERLY" "$OUT/products.log" && [ -s "$OUT/lidar.tum" ] && [ -s "$OUT/wheel.tum" ] && [ -s "$OUT/lidar_map.npz" ]; then
    N=$(grep -vc '^#' "$OUT/lidar.tum")
    CL=$(python3 -c "import json;print(json.load(open('$OUT/lidar.tum.meta.json')).get('loop_closures','?'))")
    GAP=$(python3 -c "import json;print(json.load(open('$OUT/lidar_map_facts.json')).get('closed_loop_gap_m','?'))")
    say "$TAG  complete  ${N} LiDAR nodes, ${CL} loop closures, floor gap ${GAP} m"
    exit 0
fi
fail "${DIED}products missing or the map did not close properly; see $OUT/products.log"
