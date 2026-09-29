#!/bin/bash
# pack_drive.sh - one drive's results pack, step by step: camera half, then (with --lidar) the LiDAR half.
#
#   usage:  run/lib/pack_drive.sh <run> [--lidar] [--series series2_static] [--db FILE] [--records DIR]
#                                       [--out DIR] [--work DIR] [--from N] [--title "Drive 10"]
#                                       [--tegra-start "YYYY-MM-DD HH:MM:SS" --tegra-end "..."]
#                                       [--camera-label TEXT] [--note TEXT] [--export-3d] [--lidar-from DIR]
#
# Defaults: db <work>/<run>.db, records <records>/<run>, out <results>/<series>/<run>, work ~/results_packs_work/<run>,
# tegra window = the whole tegrastats.log, title = the run name, lidar-from <replays>/<run>/lidar_ref_f3dof.
# --from N restarts at step N after a failure (the steps are numbered in the output).
#
# CAMERA HALF (11 steps, the order the September packs used)
#    1 closed check          check_closed.py: was the database closed properly (its corrected positions saved)?
#    2 camera paths          db_to_tum.py (tracking alone) and db_corrected_tum.py (corrected by loop closures)
#    3 map, tracking alone   render_map.py (results_pack fork): the map's nodes as the tracker placed them
#    4 map, corrected        render_map.py (db tools): the same map after the loop-closure corrections
#    5 turns replayed        turns_replay.py from the fusion recording (the turn watcher's rule, offline)
#    6 stream timing         extract_timing.py + pass_check.py + freeze_timing: gaps and freezes in the streams
#    7 timelapse + db check  the live timelapse copied in with four stills; db_check.py reads every page
#    8 power/temperature     tegra_peaks.py over the drive's tegrastats.log window
#    9 analyze               analyze_run.py: bag statistics, bridge link, turns, the trajectory figures
#   10 WiFi freshness        bridge_freshness.py: how long the blend ran without fresh robot data
#   11 3D export             rtabmap-export on ONE copy of the database (only with --export-3d and > 3 GB free)
# LIDAR HALF (--lidar, 6 steps): copy the LiDAR estimate in, the robot's own wheels, flatness check,
#   analyze again with the LiDAR line, compare_lidar (the agreement table), size-fit check.
# RESULTS.md is written by a per-drive script (tools/results_pack/write_results_drive10.py is the drive-10 one).
#
# Every step runs at nice 19; the original database is opened read-only; nothing is written outside --out and --work.
# A disk floor of 3 GB stops the pack rather than filling the disk. Progress: <jobs>/pack_<run>.progress.
# *Plain terms: from a drive's raw files to the numbers and pictures the write-up quotes, one numbered step at a time.*
set -u
export TZ=America/Toronto
# repository folders and addresses (REPO_ROOT, RECORDS_DIR, WORK_DIR, JOBS_DIR, TOOLS_DIR, *_ADDR): run/lib/paths.sh
. "$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)/paths.sh"
usage() { sed -n '2,29p' "$0" | sed 's/^# \{0,1\}//'; exit "${1:-0}"; }
[ $# -ge 1 ] || usage 1
case "$1" in -h|--help) usage;; esac
RUN="$1"; shift
LIDAR=0; SERIES=series2_static; DB=""; REC=""; P=""; WORK=""; FROM=1; TITLE=""; TS=""; TE=""; EXPORT3D=0; F3=""
CAMLABEL="camera tracking (/rtabmap/odom), re-started from the blend at each reset"
NOTE=""
while [ $# -gt 0 ]; do
    case "$1" in
        --lidar) LIDAR=1;;
        --series) SERIES="$2"; shift;;
        --db) DB="$2"; shift;;
        --records) REC="$2"; shift;;
        --out) P="$2"; shift;;
        --work) WORK="$2"; shift;;
        --from) FROM="$2"; shift;;
        --title) TITLE="$2"; shift;;
        --tegra-start) TS="$2"; shift;;
        --tegra-end) TE="$2"; shift;;
        --camera-label) CAMLABEL="$2"; shift;;
        --note) NOTE="$2"; shift;;
        --export-3d) EXPORT3D=1;;
        --lidar-from) F3="$2"; shift;;
        -h|--help) usage;;
        *) echo "unknown option: $1" >&2; usage 1;;
    esac; shift
done
DB="${DB:-$WORK_DIR/$RUN.db}"
REC="${REC:-$RECORDS_DIR/$RUN}"
P="${P:-$RESULTS_DIR/$SERIES/$RUN}"
WORK="${WORK:-$HOME/results_packs_work/$RUN}"
TITLE="${TITLE:-$RUN}"
F3="${F3:-$LIDAR_REPLAY_DIR/$RUN/lidar_ref_f3dof}"
PT="$PACK_TOOLS_DIR"; DT="$DB_TOOLS_DIR"
TAG=PACK; [ "$LIDAR" = 1 ] && TAG=PACK_LIDAR
PROG="$JOBS_DIR/pack_$RUN.progress"; [ "$LIDAR" = 1 ] && PROG="$JOBS_DIR/pack_${RUN}_lidar.progress"
mkdir -p "$WORK" "$P"
T0=$(date +%s)
STEP="$WORK/.pack_step"
# p "text": the step marker, the progress line for the jobs page, and the console
p() { echo "$1" > "$STEP"; echo "$TAG $RUN $1  $(( $(date +%s) - T0 ))s" > "$PROG"; echo "$(date '+%H:%M:%S') $1"; }
# a heartbeat rewrites the progress line every 20 s so the jobs page can tell slow from dead
( while kill -0 $$ 2>/dev/null; do s=$(cat "$STEP" 2>/dev/null); case "$s" in DONE*|FAILED*) exit 0;; esac
    echo "$TAG $RUN $s  $(( $(date +%s) - T0 ))s" > "$PROG"; sleep 20; done ) &
HB=$!
trap 'kill $HB 2>/dev/null' EXIT
floor() { F=$(df --output=avail -B1M "$HOME" | tail -1); [ "$F" -lt 3072 ] && { p "FAILED disk floor: only ${F} MB free (< 3 GB) - stopped"; exit 2; }; }
run() { [ "$FROM" -le "$1" ]; }
fail() { p "FAILED $1"; exit 1; }
[ -r "$DB" ] || fail "database $DB not found (--db)"
[ -d "$REC" ] || fail "record folder $REC not found (--records)"

if [ "$LIDAR" = 0 ]; then
# ============================ camera half ============================
if run 1; then
p "1/11 closed check"; floor
nice -n 19 python3 "$TOOLS_DIR/check_closed.py" "$DB" > "$P/closed_check.txt" 2>&1
fi
if run 2; then
p "2/11 camera paths (tracking alone; corrected = Admin.opt_poses)"
nice -n 19 python3 "$DT/db_to_tum.py" "$DB" "$P/camera.tum" > "$P/camera_tum.log" 2>&1 || fail "db_to_tum"
nice -n 19 python3 "$DT/db_corrected_tum.py" "$DB" "$P/camera_corrected.tum" --label "camera, $RUN" > "$P/camera_corrected.log" 2>&1 || fail "db_corrected_tum"
fi
if run 3; then
p "3/11 map tracking alone"; floor
nice -n 19 python3 "$PT/render_map.py" "$DB" "$WORK/view" --title "$RUN - map nodes, tracking alone (the blend's position in a fused drive)" \
   --source-label "Node.pose - map nodes, tracking alone (= the blend in a fused drive), not corrected" > "$WORK/map.log" 2>&1 || fail "map"
fi
if run 4; then
p "4/11 map corrected"
nice -n 19 python3 "$DT/render_map.py" "$DB" "$WORK/view" --poses "$P/camera_corrected.tum" --name map_corrected \
   --title "$RUN - camera, corrected by its loop closures (Admin.opt_poses)" > "$WORK/map_corrected.log" 2>&1 || fail "map_corrected"
cp "$WORK/view/map.png" "$WORK/view/map_corrected.png" "$P/"
for f in "$WORK"/view/*facts*.json; do [ -e "$f" ] && cp "$f" "$P/"; done
fi
if run 5; then
p "5/11 turns replayed from the bag"; floor
if [ -r "$REC/fusion.bag" ]; then
    nice -n 19 python3 "$PT/turns_replay.py" "$REC/fusion.bag" "$P/turns_replayed.log" > "$WORK/turns.log.out" 2>&1 || fail "turns"
else
    echo "no fusion.bag in $REC (camera-only drive?) - turns step skipped" | tee "$WORK/turns.log.out"
fi
fi
if run 6; then
p "6/11 stream timing (freezes)"; floor
nice -n 19 python3 "$PT/extract_timing.py" "$RUN" "$WORK/timing.npz" > "$WORK/extract_timing.out" 2>&1 || fail "extract_timing"
nice -n 19 python3 "$PT/pass_check.py" "$WORK/timing.npz" "$DB" > "$P/freezes_pass_check.json" 2> "$WORK/pass_check.err" || fail "pass_check"
nice -n 19 python3 "$PT/freeze_timing_drive7.py" "$WORK/timing.npz" "$P" "$REC/mapping.log" > "$WORK/freeze_timing.out" 2>&1 || fail "freeze_timing"
fi
if run 7; then
p "7/11 live timelapse copied in, stills, db check"; floor
if [ -r "$REC/media/${RUN}_timelapse.mp4" ]; then
    cp "$REC/media/${RUN}_timelapse.mp4" "$P/timelapse.mp4"
    cp "$REC/media/index.csv" "$P/timelapse_live_index.csv" 2>/dev/null
    N=$(ffprobe -v error -count_frames -select_streams v:0 -show_entries stream=nb_read_frames -of csv=p=0 "$P/timelapse.mp4")
    for k in 25 50 75 100; do f=$(( N * k / 100 )); [ $f -gt $((N-1)) ] && f=$((N-1))
      nice -n 19 ffmpeg -y -loglevel error -i "$P/timelapse.mp4" -vf "select=eq(n\,$f)" -vframes 1 "$P/timelapse_$k.png"; done
else
    echo "no live timelapse in $REC/media - stills skipped" > "$WORK/timelapse.out"
fi
nice -n 19 python3 "$TOOLS_DIR/db_check.py" "$DB" > "$P/db_check.txt" 2>&1
fi
if run 8; then
p "8/11 power and temperature"
if [ -r "$REC/tegrastats.log" ]; then
    # default window: the log's own first and last lines (tegrastats stamps each line with the date and time)
    [ -n "$TS" ] || TS=$(head -1 "$REC/tegrastats.log" | awk '{print $1" "$2}')
    [ -n "$TE" ] || TE=$(tail -1 "$REC/tegrastats.log" | awk '{print $1" "$2}')
    nice -n 19 python3 "$PT/tegra_peaks.py" "$REC/tegrastats.log" "$TS" "$TE" > "$P/power_temperature.json" 2> "$WORK/tegra.err" || fail "tegra"
else
    echo '{"note": "no tegrastats.log for this run"}' > "$P/power_temperature.json"
fi
fi
if run 9; then
p "9/11 analyze (bag, link, turns, figures)"; floor
TURNS=(); [ -r "$P/turns_replayed.log" ] && TURNS=(--turns "$P/turns_replayed.log" --turns-xlabel "turn number (turn watcher's rule, replayed from the recording)")
NOTEARG=(); [ -n "$NOTE" ] && NOTEARG=(--note "$NOTE")
nice -n 19 python3 "$PT/analyze_run.py" "$RUN" "$P" --db "$DB" --title "$TITLE" "${TURNS[@]}" \
   --progress "$JOBS_DIR/pack_${RUN}_analyze.progress" --camera-label "$CAMLABEL" "${NOTEARG[@]}" > "$WORK/analyze.out" 2>&1 || fail "analyze"
fi
if run 10; then
p "10/11 WiFi link: time without fresh robot data"; floor
nice -n 19 python3 "$PT/bridge_freshness_drive10.py" "$P" "$REC" > "$WORK/freshness.out" 2>&1 || fail "bridge_freshness"
fi
if run 11; then
if [ "$EXPORT3D" = 1 ]; then
p "11/11 3D export on ONE copy of the database"
mkdir -p "$P/map_3d"
EXP="$JOBS_DIR/${RUN}_export"
SZ=$(( $(stat -c %s "$DB") / 1048576 ))
FREE=$(df --output=avail -B1M "$HOME" | tail -1)
if [ $(( FREE - SZ - 700 )) -lt 3072 ]; then
  echo "3D export SKIPPED: ${FREE} MB free, copy ${SZ} MB + ~700 MB output would leave < 3 GB ($(date '+%H:%M %Z'))" > "$P/map_3d/skipped.txt"
  p "DONE 10/11; 3D export skipped (space)"; exit 0
fi
mkdir -p "$EXP"
cp "$DB" "$EXP/copy.db" || { rm -f "$EXP/copy.db"; fail "copy for export"; }
( cd "$EXP" && nice -n 19 rtabmap-export --cloud --poses --max_range 6 --decimation 4 --voxel 0.03 --output "$RUN" --output_dir "$EXP" "$EXP/copy.db" ) > "$P/map_3d/export.log" 2>&1
rm -f "$EXP/copy.db"
PLY=$(ls "$EXP"/*.ply 2>/dev/null | head -1)
[ -n "$PLY" ] || fail "3D export (no .ply; see map_3d/export.log)"
mv "$PLY" "$P/map_3d/${RUN}_map_3d.ply"
POSES=$(ls "$EXP"/*poses*.txt 2>/dev/null | head -1); [ -n "$POSES" ] && mv "$POSES" "$P/map_3d/export_poses.txt"
rmdir "$EXP" 2>/dev/null || ls -la "$EXP" >> "$P/map_3d/export.log"
echo "3D map: $P/map_3d/${RUN}_map_3d.ply (large: keep it out of the repository; see data/DATA_INDEX.md)"
else
p "11/11 3D export not asked for (--export-3d)"
fi
fi
p "DONE 11/11 camera half complete (RESULTS.md is written by the per-drive write_results script)"
exit 0
fi

# ============================ LiDAR half ============================
[ -d "$F3" ] || fail "LiDAR products folder $F3 not found - run run/lidar_reference.sh $RUN first, or pass --lidar-from"
if run 1; then
p "1/6 LiDAR yardstick (Force3DoF replay) into the pack root"; floor
for f in lidar.tum wheel.tum wheel.tum.meta.json lidar_map.npz lidar_map.png lidar_map_facts.json; do [ -e "$F3/$f" ] && cp "$F3/$f" "$P/$f"; done
nice -n 19 python3 - "$F3/lidar.tum.meta.json" "$P/lidar.tum.meta.json" "$RUN" <<'PY'
import json, sys
m = json.load(open(sys.argv[1]))
m["label"] = "LiDAR estimate (Force3DoF replay), %s" % sys.argv[3]
m["yardstick"] = ("Force3DoF ('no tipping') replay of the colleague's LiDAR mapping, one parameter changed (Reg/Force3DoF=true). "
                  "An independent estimate, not ground truth; its start line may carry the robot's unlocked clock, the message stamps come from the recording.")
json.dump(m, open(sys.argv[2], "w"), indent=2)
PY
fi
if run 2; then
p "2/6 robot's own wheels + gyroscope (its own recording)"
nice -n 19 python3 "$PT/robot_wheels_from_robot_bag.py" "$P" "$P/wheel.tum" > "$WORK/robot_wheels.out" 2>&1 || fail "robot_wheels"
fi
if run 3; then
p "3/6 flatness check of the LiDAR estimate"
nice -n 19 python3 "$PT/score_f3dof.py" "$RUN" "$P/lidar.tum" "$P/lidar.tum.meta.json" "$P/lidar_flatness_check.json" > "$WORK/flatness.out" 2>&1 || fail "flatness"
fi
if run 4; then
p "4/6 analyze again (trajectory figure now with the LiDAR line)"; floor
TURNS=(); [ -r "$P/turns_replayed.log" ] && TURNS=(--turns "$P/turns_replayed.log" --turns-xlabel "turn number (turn watcher's rule, replayed from the recording)")
nice -n 19 python3 "$PT/analyze_run.py" "$RUN" "$P" --db "$DB" --title "$TITLE" "${TURNS[@]}" \
   --progress "$JOBS_DIR/pack_${RUN}_analyze.progress" --camera-label "$CAMLABEL" \
   --note "${NOTE:-LiDAR estimate (Force3DoF replay): the robot's LiDAR map replayed with Reg/Force3DoF=true, an independent estimate (for the blend not independent: both use the robot's wheels and gyroscope).}" > "$WORK/analyze_lidar.out" 2>&1 || fail "analyze"
fi
if run 5; then
p "5/6 LiDAR comparison (yardstick: Force3DoF replay)"; floor
nice -n 19 python3 "$PT/compare_lidar.py" "$P" --tracking-label "map nodes (= blend), tracking alone" \
   --reference-note "Yardstick: LiDAR estimate (Force3DoF replay). For the blend it is NOT independent: both use the robot's wheels and gyroscope." > "$WORK/compare_lidar.out" 2>&1 || fail "compare_lidar"
fi
if run 6; then
p "6/6 size-fit check"
nice -n 19 python3 "$PT/size_fit_agreement.py" "$P" > "$WORK/size_fit.out" 2>&1 || fail "size_fit"
fi
p "DONE 6/6 LiDAR half complete"
