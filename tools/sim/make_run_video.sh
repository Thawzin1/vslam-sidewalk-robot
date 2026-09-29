#!/usr/bin/env bash
#
# make_run_video.sh — turn ONE finished mapping run into ONE presentable video.
#
#   ./make_run_video.sh <RUN_ID>                 # after the run: build the video
#   ./make_run_video.sh --capture <RUN_ID>       # during the run: record the
#                                                #   overhead camera to disk
#
# WHY THIS EXISTS
#   A finished run leaves two things worth showing a supervisor side by side:
#   the world the robot was actually driving through, and the map it was
#   building while it drove. Until now the first of those was never captured at
#   all, and turning the second into something presentable was a hand-typed
#   ffmpeg command that lived in somebody's shell history and nowhere else.
#   Every step below used to be manual. All of it is now one line in one file.
#
#   It also refuses to declare success without checking the one thing that
#   silently destroyed thirty-two previous videos: whether the map-building
#   footage is actually moving. See "THE FROZEN-FOOTAGE CHECK" below.
#
# WHAT IT PRODUCES  (all under the run's own records folder, then copied to the
# results folder so the presentable copies live with the figures)
#   <RUN_ID>_sidebyside.mp4        the archival composition, real timeline
#   <RUN_ID>_sidebyside_1p5x.mp4   the one you actually show people
#   <RUN_ID>_sidebyside.provenance.txt
#
# WHAT IT NEEDS FROM THE RUN
#   1. the map-building timelapse, already produced by the existing tooling:
#        rosrun sidewalk_slam map_manager.py timelapse       (during the run)
#        rosrun sidewalk_slam map_manager.py timelapse-video (after the run)
#      p1_run.sh already does both.
#   2. an overhead-camera clip, which does NOT exist unless somebody started
#      the companion recorder during the run. That is what --capture is for,
#      and the harness has to be wired up to call it. There is no way to
#      recover this footage after the fact — the topic is gone when the
#      simulator stops.
#
# HOW THE OVERHEAD FOOTAGE GETS TO DISK, AND WHY THIS WAY
#   The downward-looking camera publishes sensor_msgs/Image on
#   /overhead_cam/image_raw while the simulator is running. Four ways to get
#   that onto disk were considered:
#
#     * Screen recording of the simulator window. Ruled out on arrival:
#       recorded runs are launched with gui:=false (that is what gets the
#       simulator to a real-time factor near 1.0 in the first place), so there
#       is no window to record, and the machine is usually driven over a
#       remote connection with nothing on its screen at all.
#     * The 3-D visualiser. Banned outright for recorded runs — it once held
#       11 gigabytes of memory and froze the whole machine for 29 minutes.
#       See docs/DO_NOT_REPEAT.md.
#     * Record the image topic into a bag file, extract the frames afterwards.
#       Correct, and rejected on cost. An uncompressed colour image message is
#       width x height x 3 bytes; even a modest 640 x 480 at 10 frames per
#       second is about 9 megabytes per second, so a twelve-minute run writes
#       roughly six and a half gigabytes — onto the same disk the mapping
#       database is being written to, while the mapping stack is competing for
#       the same eight cores. Then it needs a second full decode pass to turn
#       those messages into frames. Compressing the bag file only moves the
#       cost from the disk to the processor, which is the scarcer of the two
#       on this machine.
#     * image_view's video_recorder node. CHOSEN. It subscribes to the image
#       topic and hands each frame straight to a motion-JPEG encoder writing
#       to a file, so it holds one frame in memory at a time and never grows a
#       queue — which matters on a machine with this project's history of
#       memory incidents. It writes on the order of one to two megabytes per
#       second instead of nine, and needs no second pass afterwards. It is
#       safe with no screen attached: the installed binary links the OpenCV
#       video-writing library but NOT the OpenCV window library (verified with
#       ldd on the Jetson), so it opens no window and never touches a display.
#
#   TWO THINGS THE HARNESS MUST GET RIGHT WHEN IT WIRES THIS UP:
#     a. --fps must be less than or equal to the camera's update rate in the
#        world file. The recorder throttles itself to that number using the
#        simulator clock and writes one frame per interval; if you ask for
#        more frames per second than the camera actually publishes, the file
#        claims a timeline shorter than the run really was and everything
#        plays too fast.
#     b. Stop it with SIGINT (kill -2), never SIGKILL (kill -9). The video
#        file is finalised when the encoder is closed on a clean exit. Killed
#        outright, the clip is left without its index and may be unplayable.
#        Note that the existing timelapse recorder in p1_run.sh is stopped
#        with kill -9, which is fine for that one (it writes complete image
#        files as it goes) and is NOT fine for this one.
#
# THE FROZEN-FOOTAGE CHECK
#   All sixteen Phase-1 map-building videos, and sixteen more after them, were
#   frozen: byte-identical from the first frame to the last, because the map
#   topic goes silent unless something holds a subscription open, and the
#   snapshot tool connects and disconnects for every single snapshot. The
#   recorder has since been fixed (docs/SOLVED.md), but nothing ever checked
#   the output, so thirty-two useless files were produced, archived and
#   reported before a human noticed by eye.
#
#   The tell is trivial to compute and was never computed: identical frames
#   have identical checksums. This script counts how many of the run's
#   snapshots are actually distinct before it encodes anything, prints that
#   count next to the total whether it passes or fails, and refuses to build a
#   video out of a run whose map never moved.
#
# No 'set -u': --capture sources ROS's own setup scripts, which reference
# unset variables and would abort the script at the source lines. The
# ${n:?} guards and explicit existence checks below cover the arguments.
# repository folders and addresses (REPO_ROOT, RECORDS_DIR, WORK_DIR, JOBS_DIR, TOOLS_DIR, *_ADDR): run/lib/paths.sh
. "$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)/../../run/lib/paths.sh"
set -o pipefail

# --------------------------------------------------------------- house style
say()  { printf '\033[1;36m[make_run_video]\033[0m %s\n' "$*"; }
info() { printf '        %s\n' "$*"; }
warn() { printf '\033[1;33m  WARN\033[0m  %s\n' "$*"; }
die()  { printf '\033[1;31m  FAIL\033[0m  %s\n' "$*" >&2; exit 1; }

# ------------------------------------------------------------------ defaults
REPO="${SIDEWALK_REPO:-$REPO_ROOT}"

MODE="build"
RUN_ID=""
# These three defaults are not guesses — they are read off the camera that is
# actually in the world files. Both office.world and office2.world define one
# static overhead model publishing /overhead_cam/image_raw at 800 by 600 and
# <update_rate>10</update_rate>. Capturing at 10 frames per second therefore
# takes every frame the camera produces and no more, and a 800 by 600 panel
# shows the picture at its native size with no rescaling at all. If a world
# ever changes its camera, change these to match it — capturing above the
# camera's rate is wiring note (a) above, and it makes the clip play too fast.
TOPIC="/overhead_cam/image_raw"
CAPTURE_FPS=10
OVERHEAD=""                     # default derived from RUN_ID once known
MAP_VIDEO=""
FRAMES_DIR=""
RESULTS_DIR=""
SPEED=1.5                       # the sped-up version everyone actually watches
OUT_FPS=15                      # output frame rate of the composition
PANEL_W=800                     # each half of the side-by-side, in pixels
PANEL_H=600
TARGET_DURATION=""              # seconds; default = the map video's own length
MIN_DISTINCT=5                  # fewer distinct snapshots than this = frozen
IGNORE_FROZEN=false
CHECK_OVERHEAD=false
ALLOW_MISSING_OVERHEAD=false

usage() {
  sed -n '2,40p' "$0" | sed 's/^# \{0,1\}//'
  cat <<'EOF'

Options (build mode):
  --overhead FILE          overhead clip (default: <records>/<RUN_ID>/overhead_cam.avi)
  --map-video FILE         map-building clip (default: logs/.../maps/<RUN_ID>_timelapse.avi)
  --frames-dir DIR         map snapshots to checksum
                           (default: logs/.../maps/timelapse/<RUN_ID>/)
  --results-dir DIR        where the presentable copies are filed
  --speed N                speed-up factor for the second output (default: 1.5)
  --out-fps N              output frame rate (default: 15)
  --panel WxH              size of each half (default: 800x600)
  --target-duration SEC    make both halves span this many seconds
                           (default: the map-building clip's own length)
  --min-distinct N         frozen-footage threshold (default: 5)
  --check-overhead         also scan the overhead clip for frozen stretches
                           (costs one extra decode pass; off by default)
  --allow-missing-overhead build a single-panel video if there is no overhead clip
  --ignore-frozen          build anyway, and stamp FROZEN into the filename

Options (capture mode):
  --topic NAME             image topic (default: /overhead_cam/image_raw)
  --fps N                  frames per second to record (default: 10)
  --out FILE               output clip
EOF
}

need() {  # need <option> <value> — refuse an option given without its value
  [ -n "$2" ] && case "$2" in --*) false ;; *) true ;; esac \
    || die "$1 needs a value after it."
}

while [ $# -gt 0 ]; do
  case "$1" in
    --capture)                 MODE="capture" ;;
    --topic)                   need "$1" "$2"; TOPIC="$2"; shift ;;
    --fps)                     need "$1" "$2"; CAPTURE_FPS="$2"; shift ;;
    --out|--overhead)          need "$1" "$2"; OVERHEAD="$2"; shift ;;
    --map-video)               need "$1" "$2"; MAP_VIDEO="$2"; shift ;;
    --frames-dir)              need "$1" "$2"; FRAMES_DIR="$2"; shift ;;
    --results-dir)             need "$1" "$2"; RESULTS_DIR="$2"; shift ;;
    --speed)                   need "$1" "$2"; SPEED="$2"; shift ;;
    --out-fps)                 need "$1" "$2"; OUT_FPS="$2"; shift ;;
    --panel)                   need "$1" "$2"
                               case "$2" in
                                 *x*) PANEL_W="${2%x*}"; PANEL_H="${2#*x}" ;;
                                 *)   die "--panel wants WIDTHxHEIGHT, for example 800x600 (got '$2')." ;;
                               esac; shift ;;
    --target-duration)         need "$1" "$2"; TARGET_DURATION="$2"; shift ;;
    --min-distinct)            need "$1" "$2"; MIN_DISTINCT="$2"; shift ;;
    --check-overhead)          CHECK_OVERHEAD=true ;;
    --allow-missing-overhead)  ALLOW_MISSING_OVERHEAD=true ;;
    --ignore-frozen)           IGNORE_FROZEN=true ;;
    -h|--help)                 usage; exit 0 ;;
    -*)                        die "unknown option: $1  (try --help)" ;;
    *)                         RUN_ID="$1" ;;
  esac
  shift
done

[ -n "$RUN_ID" ] || { usage; exit 2; }

# Whole numbers are checked before use, not trusted. MIN_DISTINCT especially:
# it is the right-hand side of the frozen-footage comparison, and a value the
# shell cannot parse as a number makes that comparison fail rather than
# evaluate — which, with no 'set -e', would quietly skip the one check this
# script exists to perform.
for pair in "MIN_DISTINCT:$MIN_DISTINCT" "OUT_FPS:$OUT_FPS" \
            "CAPTURE_FPS:$CAPTURE_FPS" "PANEL_W:$PANEL_W" "PANEL_H:$PANEL_H"; do
  case "${pair#*:}" in
    ''|*[!0-9]*) die "${pair%%:*} must be a whole number (got '${pair#*:}')." ;;
  esac
done
[ "$MIN_DISTINCT" -ge 1 ] || die "--min-distinct must be at least 1."
[ "$OUT_FPS" -ge 1 ]      || die "--out-fps must be at least 1."
# The run identifier becomes part of several file paths and of an ffmpeg filter
# string, so anything outside this set is refused rather than escaped.
case "$RUN_ID" in
  *[!A-Za-z0-9._-]*) die "run identifier '$RUN_ID' contains characters other than letters, digits, dot, underscore and hyphen." ;;
esac

RECORDS="$RECORDS_DIR/$RUN_ID"
[ -n "$OVERHEAD" ]    || OVERHEAD="$RECORDS/overhead_cam.avi"
[ -n "$MAP_VIDEO" ]   || MAP_VIDEO="$REPO/logs/sidewalk_slam/maps/${RUN_ID}_timelapse.avi"
[ -n "$FRAMES_DIR" ]  || FRAMES_DIR="$REPO/logs/sidewalk_slam/maps/timelapse/$RUN_ID"
[ -n "$RESULTS_DIR" ] || RESULTS_DIR="$REPO/logs/sidewalk_evaluation/results/videos"

# =========================================================== CAPTURE MODE ===
# Runs DURING the mapping run, alongside the map-timelapse recorder. It blocks
# until interrupted, exactly like the timelapse recorder, so the harness must
# background it and stop it at teardown. The lines to add to p1_run.sh are
# printed by --help and repeated here so they are findable by grep:
#
#   # ... next to the existing timelapse recorder ...
#   nohup "$REPO/tools/make_run_video.sh" --capture "$RUN_ID" \
#     > "$REC/overhead.log" 2>&1 < /dev/null &
#   OVERHEAD_PID=$!
#
#   # ... in teardown, BEFORE the simulator is stopped, and with -2 not -9 ...
#   kill -2 "$OVERHEAD_PID" 2>/dev/null; sleep 3
#
# It must also be added to p1_run.sh's sweep() argument pattern, next to
# 'map_manager\.py timelapse', so a stray recorder from a previous run cannot
# survive into the next one and write into the wrong file.
if [ "$MODE" = "capture" ]; then
  [ -f /opt/ros/noetic/setup.bash ] || die "/opt/ros/noetic/setup.bash is missing — this half of the script only runs on a machine with ROS installed."
  source /opt/ros/noetic/setup.bash
  [ -f "$CATKIN_WS/devel/setup.bash" ] && source "$CATKIN_WS/devel/setup.bash"

  command -v rosrun >/dev/null 2>&1 || die "rosrun is not on PATH after sourcing ROS."
  mkdir -p "$(dirname "$OVERHEAD")" || die "cannot create $(dirname "$OVERHEAD")"

  say "recording $TOPIC at ${CAPTURE_FPS} frames per second"
  info "output: $OVERHEAD"
  info "stop this with SIGINT (kill -2) so the clip is closed properly."
  info "if this exits immediately, the overhead camera is not publishing —"
  info "check the world file has the downward-looking camera in it."

  # image:=<topic> is the remap image_view's recorder expects. The encoder is
  # motion-JPEG in an AVI container: every frame stands alone, so a clip that
  # gets truncated is still readable up to the truncation point, which is the
  # right failure mode for something that runs unattended for ten minutes.
  exec rosrun image_view video_recorder \
    image:="$TOPIC" \
    _filename:="$OVERHEAD" \
    _fps:="$CAPTURE_FPS" \
    _codec:=MJPG \
    _encoding:=bgr8
fi

# ============================================================= BUILD MODE ===
command -v ffmpeg  >/dev/null 2>&1 || die "ffmpeg is not on PATH (sudo apt install ffmpeg)."
command -v ffprobe >/dev/null 2>&1 || die "ffprobe is not on PATH (it ships with ffmpeg)."

say "run $RUN_ID"

# ---------------------------------------------------------------------------
# STEP 0 — the frozen-footage check, before anything is encoded.
#
# Done first on purpose: a frozen run must not consume several minutes of a
# loaded machine's processor before being rejected. Identical images have
# identical checksums, so the number of distinct checksums across the run's
# snapshots is exactly the number of moments the map actually changed.
# A healthy reference run: 154 distinct out of 168. A broken one: 1 out of 192.
# ---------------------------------------------------------------------------
say "checking the map-building footage is not frozen"
[ -d "$FRAMES_DIR" ] || die "no snapshot folder at $FRAMES_DIR
       Without the source snapshots this script cannot tell a real map from a
       frozen one, and it will not certify a video it has not checked. Pass
       --frames-dir if the snapshots are somewhere else."

TOTAL_FRAMES=$(find "$FRAMES_DIR" -maxdepth 1 -name '*.pgm' | wc -l)
[ "$TOTAL_FRAMES" -gt 0 ] || die "no .pgm snapshots in $FRAMES_DIR
       The timelapse recorder never saved anything for this run."

DISTINCT_FRAMES=$(find "$FRAMES_DIR" -maxdepth 1 -name '*.pgm' -print0 \
  | xargs -0 md5sum \
  | awk '{print $1}' | sort -u | wc -l)

# Printed on every path, pass or fail, so the number is in the log of every
# run forever rather than only being looked at once something is wrong.
info "distinct snapshots: $DISTINCT_FRAMES of $TOTAL_FRAMES total"

# The keep-alive subscriber inside the timelapse recorder counts how many
# times the map topic really published. It is an independent second opinion on
# the same question, from a structurally different source, so it is worth
# surfacing next to the checksum count rather than trusting either alone.
TL_LOG="$RECORDS/timelapse.log"
if [ -f "$TL_LOG" ]; then
  PUBLISHES=$(grep -o 'grid_map_publishes_seen[^0-9]*[0-9]\+' "$TL_LOG" 2>/dev/null \
              | grep -o '[0-9]\+$' | tail -1)
  [ -n "$PUBLISHES" ] && info "map topic publishes observed during the run: $PUBLISHES"
fi

FROZEN_TAG=""
if [ "$DISTINCT_FRAMES" -lt "$MIN_DISTINCT" ]; then
  printf '\033[1;31m
  ============================================================
   FROZEN MAP FOOTAGE — %s of %s snapshots are distinct.
  ============================================================\033[0m\n' \
    "$DISTINCT_FRAMES" "$TOTAL_FRAMES" >&2
  info "The map never changed during this run, so a video of it shows nothing."
  info "This is the failure that ruined thirty-two earlier videos. It is"
  info "almost always the map topic going silent for want of a subscriber"
  info "(docs/SOLVED.md), not the mapping itself failing — check the run's"
  info "database with 'map_manager.py info --db <database>' before concluding"
  info "the run was bad. Re-record the timelapse; the run may still be fine."
  if [ "$IGNORE_FROZEN" = true ]; then
    warn "--ignore-frozen given: building anyway, and stamping FROZEN into the"
    warn "filename so this file can never be mistaken for a usable result."
    FROZEN_TAG="_FROZEN"
  else
    die "refusing to build a video from frozen footage (pass --ignore-frozen to override)."
  fi
elif [ "$((DISTINCT_FRAMES * 10))" -lt "$TOTAL_FRAMES" ]; then
  # Not fatal: a run that finishes with the robot parked legitimately repeats
  # its last frame. Worth saying out loud all the same.
  warn "fewer than one snapshot in ten is distinct — the map spent most of the"
  warn "run not changing. Worth a look before this goes in front of anyone."
fi

# ---------------------------------------------------------------------------
# STEP 1 — locate and measure the two clips.
# ---------------------------------------------------------------------------
[ -f "$MAP_VIDEO" ] || die "no map-building video at $MAP_VIDEO
       Build it first:
         rosrun sidewalk_slam map_manager.py timelapse-video --name $RUN_ID"

HAVE_OVERHEAD=true
if [ ! -f "$OVERHEAD" ]; then
  HAVE_OVERHEAD=false
  if [ "$ALLOW_MISSING_OVERHEAD" != true ]; then
    die "no overhead-camera clip at $OVERHEAD
       This footage can only be captured while the simulator is running, so it
       cannot be recovered now. The run needs to be repeated with the capture
       companion started alongside the mapping stack:
         $0 --capture $RUN_ID
       See the wiring notes at the top of this script for the exact lines to
       add to tools/p1_run.sh. To build a map-only video from this run in the
       meantime, pass --allow-missing-overhead."
  fi
  warn "no overhead clip — building a single-panel map-only video instead."
fi

# format=duration is what both containers actually carry; the per-stream
# duration is the fallback for files whose container header is thin.
probe_duration() {
  local d
  d=$(ffprobe -v error -show_entries format=duration -of csv=p=0 "$1" 2>/dev/null | head -1)
  case "$d" in ''|N/A|0|0.000000) d="" ;; esac
  if [ -z "$d" ]; then
    d=$(ffprobe -v error -select_streams v:0 -show_entries stream=duration \
        -of csv=p=0 "$1" 2>/dev/null | head -1)
    case "$d" in ''|N/A|0|0.000000) d="" ;; esac
  fi
  printf '%s' "$d"
}

MAP_DUR=$(probe_duration "$MAP_VIDEO")
info "map-building clip:  $MAP_VIDEO ($(du -h "$MAP_VIDEO" | cut -f1), ${MAP_DUR:-unknown} s)"
if [ "$HAVE_OVERHEAD" = true ]; then
  OVER_DUR=$(probe_duration "$OVERHEAD")
  info "overhead clip:      $OVERHEAD ($(du -h "$OVERHEAD" | cut -f1), ${OVER_DUR:-unknown} s)"
  [ -n "$OVER_DUR" ] || warn "the overhead clip does not declare a duration; it will be
       stacked at its own speed rather than retimed to match the map."
fi

# Optional, off by default because it costs a full extra decode of the longest
# clip in the set. freezedetect is ffmpeg's own answer to exactly the question
# the checksum count answers for the map side.
if [ "$CHECK_OVERHEAD" = true ] && [ "$HAVE_OVERHEAD" = true ]; then
  say "scanning the overhead clip for frozen stretches (this decodes it once)"
  FREEZES=$(ffmpeg -v info -nostdin -i "$OVERHEAD" \
              -vf freezedetect=noise=0.003:duration=2 -map 0:v -f null - 2>&1 \
            | grep -c 'freeze_start')
  if [ "$FREEZES" -gt 0 ]; then
    warn "the overhead clip contains $FREEZES frozen stretch(es) of two seconds"
    warn "or more — the simulator may have stalled, or the camera stopped."
  else
    info "overhead clip: no frozen stretches of two seconds or more"
  fi
fi

# ---------------------------------------------------------------------------
# STEP 2 — compose the two clips side by side.
#
# Three things have to be reconciled, and all three used to be done by eye:
#
#   Different sizes. The two clips have unrelated pixel dimensions, and the map
#   one is whatever size the explored area came out at. Horizontal stacking
#   demands equal heights, so each clip is scaled to fit inside an identical
#   box without distorting it, then padded out to fill that box exactly.
#
#   Different durations. The overhead camera runs in real time; the map clip is
#   one snapshot per second of the run played back at five frames per second,
#   so it is several times shorter. Stacked as-is the two halves would show
#   completely different moments of the run at any given instant, which makes
#   the whole comparison meaningless. Both are therefore retimed onto a single
#   shared timeline — by default the map clip's own length, which keeps the map
#   playing at the speed it was authored for and compresses the world view to
#   match, giving a short video where the left and right halves agree about
#   what moment they are showing.
#
#   Which half is which. Neither picture is self-explanatory to somebody seeing
#   it for the first time, so each half carries a caption over a dark band.
# ---------------------------------------------------------------------------

# yuv420p, which is what any player will expect, needs even dimensions.
PANEL_W=$(( (PANEL_W / 2) * 2 ))
PANEL_H=$(( (PANEL_H / 2) * 2 ))
[ "$PANEL_W" -ge 160 ] && [ "$PANEL_H" -ge 120 ] || die "--panel is too small: ${PANEL_W}x${PANEL_H}"

FONT=""
for f in /usr/share/fonts/truetype/dejavu/DejaVuSans.ttf \
         /usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf; do
  [ -f "$f" ] && { FONT="$f"; break; }
done
[ -n "$FONT" ] || warn "no font file found — the halves will not be captioned."

CAPTION_LEFT="Overhead camera - the world it is driving through"
CAPTION_RIGHT="Map being built - what the robot knows so far"
STAMP="run ${RUN_ID}"

# Size the captions to fit the panel rather than assuming they do. drawtext
# will not wrap or shrink text: a caption wider than its panel simply runs off
# the edge, which is how the right-hand caption lost its last word the first
# time this was tried at 800 pixels. In DejaVu Sans the average glyph is a
# little under 0.62 of the nominal size, so the widest caption is measured
# against 92 % of the panel width and the size is taken from whichever of that
# and the plain proportional size is smaller.
MAXLEN=${#CAPTION_LEFT}
[ "${#CAPTION_RIGHT}" -gt "$MAXLEN" ] && MAXLEN=${#CAPTION_RIGHT}
FONTSIZE=$(awk -v w="$PANEL_W" -v n="$MAXLEN" 'BEGIN{
  fit = (w * 0.92) / (0.62 * n); prop = w / 30
  s = (fit < prop) ? fit : prop
  s = int(s); if (s < 10) s = 10
  print s
}')

# Retiming factors. setpts multiplies presentation timestamps, so a factor
# below 1 speeds a clip up and above 1 slows it down.
TARGET="${TARGET_DURATION:-$MAP_DUR}"
factor_for() {  # factor_for <clip duration> -> multiplier, 1.0 if unknowable
  awk -v d="$1" -v t="$TARGET" 'BEGIN{
    if (d+0 <= 0 || t+0 <= 0) { print "1.0"; exit }
    printf "%.6f", t/d
  }'
}
F_MAP=$(factor_for "$MAP_DUR")
F_OVER=1.0
[ "$HAVE_OVERHEAD" = true ] && [ -n "$OVER_DUR" ] && F_OVER=$(factor_for "$OVER_DUR")
[ -n "$TARGET" ] && info "shared timeline: ${TARGET} s (world x $F_OVER, map x $F_MAP)"

# One panel's worth of filtering: retime, fit into the box, pad to fill it,
# caption it.
#
# NOTE — there is deliberately no 'fps' filter in this chain, and putting one
# here is a mistake that looks like it works. Measured on this machine
# (ffmpeg 4.2.7): a 90-second clip through "setpts=PTS*0.455556" alone comes
# out at 41.2 seconds as intended, but through "setpts=PTS*0.455556,fps=15"
# it comes out at 90 seconds again — the frame-rate filter re-derives its
# output timestamps and throws the retiming away, without a warning and
# without failing. The result is a perfectly playable video in which the two
# halves show different moments of the run, which is precisely the sort of
# quietly-wrong output this script exists to stop shipping.
#
# The constant output frame rate is therefore set once, at the encoder, with
# -r below. That is also the cheaper arrangement: this way the scaling and
# padding run on the source frames only, instead of on an inflated set of
# duplicated frames.
panel_chain() {  # panel_chain <input label> <setpts factor> <caption> <out label>
  local text_bit=""
  [ -n "$FONT" ] && text_bit=",drawtext=fontfile='${FONT}':text='${3}':expansion=none:fontcolor=white:fontsize=${FONTSIZE}:x=(w-text_w)/2:y=10:box=1:boxcolor=black@0.55:boxborderw=8"
  printf "[%s]setpts=PTS*%s,scale=%s:%s:force_original_aspect_ratio=decrease,pad=%s:%s:(ow-iw)/2:(oh-ih)/2:color=0x101418%s[%s];" \
    "$1" "$2" "$PANEL_W" "$PANEL_H" "$PANEL_W" "$PANEL_H" "$text_bit" "$4"
}

mkdir -p "$RECORDS" || die "cannot create $RECORDS"
COMPOSED="$RECORDS/${RUN_ID}${FROZEN_TAG}_sidebyside.mp4"

if [ "$HAVE_OVERHEAD" = true ]; then
  FILTER="$(panel_chain 0:v "$F_OVER" "$CAPTION_LEFT" L)"
  FILTER="${FILTER}$(panel_chain 1:v "$F_MAP" "$CAPTION_RIGHT" R)"
  # shortest=0 keeps the output as long as the longer half: if one clip does
  # run out early despite the retiming, its last frame is held rather than the
  # whole video being cut short at that point.
  FILTER="${FILTER}[L][R]hstack=inputs=2:shortest=0[S]"
  INPUTS=(-i "$OVERHEAD" -i "$MAP_VIDEO")
else
  FILTER="$(panel_chain 0:v "$F_MAP" "$CAPTION_RIGHT" S)"
  FILTER="${FILTER%;}"
  INPUTS=(-i "$MAP_VIDEO")
fi
if [ -n "$FONT" ]; then
  FILTER="${FILTER};[S]drawtext=fontfile='${FONT}':text='${STAMP}':expansion=none:fontcolor=white@0.75:fontsize=$((FONTSIZE * 3 / 4)):x=12:y=h-th-10:box=1:boxcolor=black@0.45:boxborderw=6[V]"
  MAPOUT="[V]"
else
  MAPOUT="[S]"
fi

say "composing the side-by-side video"
# -an throughout: neither source has audio, and asking for a track that does
# not exist is an error rather than a no-op.
# veryfast + constant-rate-factor 23 is deliberate. This machine is running a
# simulator and a mapping stack; a slower preset buys picture quality nobody
# will notice in a screen recording of a grid map and costs cores that are
# needed elsewhere.
ffmpeg -y -hide_banner -loglevel warning -nostdin \
  "${INPUTS[@]}" \
  -filter_complex "$FILTER" -map "$MAPOUT" -r "$OUT_FPS" \
  -c:v libx264 -preset veryfast -crf 23 -pix_fmt yuv420p \
  -movflags +faststart -an "$COMPOSED" \
  || die "ffmpeg failed while composing. Run the same command with
       -loglevel info to see what it objected to."
[ -s "$COMPOSED" ] || die "ffmpeg reported success but $COMPOSED is empty."
info "composed: $COMPOSED ($(du -h "$COMPOSED" | cut -f1))"

# Confirm the retiming actually happened. ffmpeg will happily produce a
# well-formed video whose halves are out of step with each other — that is
# exactly how the frame-rate-filter mistake described above went unnoticed
# until the durations were compared. One ffprobe call closes that gap.
COMPOSED_DUR=$(probe_duration "$COMPOSED")
if [ -n "$TARGET" ] && [ -n "$COMPOSED_DUR" ]; then
  DUR_OK=$(awk -v a="$COMPOSED_DUR" -v b="$TARGET" 'BEGIN{
    d = a - b; if (d < 0) d = -d; print (d <= b * 0.10) ? "yes" : "no" }')
  if [ "$DUR_OK" != yes ]; then
    warn "the composed video is ${COMPOSED_DUR} s but the shared timeline was"
    warn "meant to be ${TARGET} s. The two halves are very likely NOT showing"
    warn "the same moment of the run as each other. Do not present this until"
    warn "it is understood."
  else
    info "shared timeline honoured: ${COMPOSED_DUR} s against ${TARGET} s intended"
  fi
fi

# ---------------------------------------------------------------------------
# STEP 3 — the sped-up version.
#
# This is the step that was tribal knowledge: a hand-typed command nobody wrote
# down, reconstructed from shell history each time. It is now a line in a file.
# It re-encodes the composed video rather than redoing the whole composition,
# which is much cheaper — the composed file is small and already H.264.
# ---------------------------------------------------------------------------
# 1.5 becomes "1p5", not "1.5". See the naming note at the bottom of this file.
SPEED_TAG=$(awk -v s="$SPEED" 'BEGIN{
  if (s+0 <= 0) { print "bad"; exit }
  t = sprintf("%.2f", s); sub(/0+$/, "", t); sub(/\.$/, "", t); gsub(/\./, "p", t); print t
}')
[ "$SPEED_TAG" = "bad" ] && die "--speed must be greater than zero (got '$SPEED')."
FAST="$RECORDS/${RUN_ID}${FROZEN_TAG}_sidebyside_${SPEED_TAG}x.mp4"

say "building the ${SPEED}x version"
ffmpeg -y -hide_banner -loglevel warning -nostdin -i "$COMPOSED" \
  -filter:v "setpts=PTS/${SPEED}" -r "$OUT_FPS" \
  -c:v libx264 -preset veryfast -crf 23 -pix_fmt yuv420p \
  -movflags +faststart -an "$FAST" \
  || die "ffmpeg failed while speeding the video up."
[ -s "$FAST" ] || die "ffmpeg reported success but $FAST is empty."
info "sped up: $FAST ($(du -h "$FAST" | cut -f1))"

# ---------------------------------------------------------------------------
# STEP 4 — file the results, with their provenance.
#
# Also previously manual, and therefore previously forgotten. Every number this
# project reports has to trace back to a run identifier (ENGINEERING_NOTES.md rule 9), and
# a video shown to a supervisor is a reported result like any other — so the
# counts that certified it are written out beside it.
# ---------------------------------------------------------------------------
mkdir -p "$RESULTS_DIR" || die "cannot create $RESULTS_DIR"
PROV="$RECORDS/${RUN_ID}${FROZEN_TAG}_sidebyside.provenance.txt"
{
  echo "run_id:                    $RUN_ID"
  echo "built_at:                  $(date -Is)"
  echo "distinct_map_snapshots:    $DISTINCT_FRAMES"
  echo "total_map_snapshots:       $TOTAL_FRAMES"
  echo "frozen_threshold:          $MIN_DISTINCT"
  echo "map_topic_publishes_seen:  ${PUBLISHES:-not recorded}"
  if [ "$HAVE_OVERHEAD" = true ]; then
    echo "overhead_clip:           $OVERHEAD"
  else
    echo "overhead_clip:           none - map-only video"
  fi
  echo "overhead_source_duration:  ${OVER_DUR:-not applicable}"
  echo "map_clip:                  $MAP_VIDEO"
  echo "map_source_duration:       ${MAP_DUR:-unknown}"
  echo "shared_timeline_seconds:   ${TARGET:-not retimed}"
  echo "panel_size:                ${PANEL_W}x${PANEL_H}"
  echo "output_frame_rate:         $OUT_FPS"
  echo "speed_factor:              $SPEED"
  echo "ffmpeg:                    $(ffmpeg -version 2>/dev/null | head -1)"
  echo "git_rev:                   $(cd "$REPO" && git rev-parse HEAD 2>/dev/null || echo unknown)"
} > "$PROV"

cp -f "$COMPOSED" "$FAST" "$PROV" "$RESULTS_DIR/" \
  || die "could not copy the finished videos into $RESULTS_DIR"

say "done"
info "map footage:  $DISTINCT_FRAMES of $TOTAL_FRAMES snapshots distinct"
info "filed in:     $RESULTS_DIR/"
info "  $(basename "$COMPOSED")"
info "  $(basename "$FAST")     <- show this one"
info "  $(basename "$PROV")"
if [ -n "$FROZEN_TAG" ]; then
  warn "these files are stamped FROZEN and are NOT a usable result."
  # A deliberately-built broken artefact still reports failure to the caller,
  # so a harness cannot record this run as having produced a good video.
  exit 1
fi
exit 0

# ---------------------------------------------------------------------------
# NAMING CONVENTION — pinned here, one only.
#
# Two were in use: <name>_1.5x.mp4 and <name>_1p5x.mp4. This script writes
# _1p5x.mp4 and nothing else. Three reasons, in order of weight:
#
#   1. It is already what the project mostly does — sixteen files on the Jetson
#      use _1p5x against one using _1.5x. Standardising on the majority means
#      renaming one file, not sixteen.
#   2. A second dot before the extension breaks naive extension handling.
#      "${name##*.}" in a shell gives "5x"; Python's os.path.splitext gives
#      ".5x". Anything that sorts, filters or uploads by extension can get this
#      wrong, and some mail clients rewrite such names on the way out.
#   3. It survives being pasted anywhere without quoting or escaping.
#
# The letter p stands in for the decimal point, so any factor works the same
# way: 1.5 -> 1p5x, 2 -> 2x, 1.25 -> 1p25x. The suffix is generated from the
# --speed value, so the convention cannot drift by hand again.
# ---------------------------------------------------------------------------
