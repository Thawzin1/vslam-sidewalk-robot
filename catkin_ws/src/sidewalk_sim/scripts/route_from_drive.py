#!/usr/bin/env python3
"""
route_from_drive.py — turn a recorded ground-truth trajectory from a
human-driven exploration run into a route file that route_player.py can
replay.

WHY THIS EXISTS
    The two paper-derived indoor routes (routes/office1_full.yaml and
    routes/office2_full.yaml) were built by walking each world file's
    collision geometry on paper — nobody had driven either one, and both
    are known to be unable to enter several rooms because furniture blocks
    the doorways (see office1_full.yaml's own header for the measured
    clearances). Once a human drives an office layout by hand, deleting the
    blocking furniture live in the simulator as they go, hand-tuning the
    paper route against furniture that just moved stops being useful — the
    drive itself is now the best evidence of a path that actually works.
    This tool turns that drive into a route file, instead.

    It is standalone on purpose: pure Python standard library plus PyYAML,
    no ROS import anywhere. It only reads a trajectory file that some other
    process already recorded (record_mapping_run.launch, read in full while
    building this tool — see below) and writes a YAML file. That means it
    runs equally well on the Jetson or on your own computer with no ROS
    environment sourced at all, which matters because turning a drive into
    a route file is exactly the kind of thing you want to do sitting at a
    desk after the drive, not while the simulator is still running.

WHERE THE INPUT COMES FROM (verified by reading the actual recording code,
not assumed)
    record_mapping_run.launch starts two trajectory_recorder.py instances.
    The ground-truth one is launched with
        --stack ground_truth --run-id <run_id> --source topic
        --topic /ground_truth/odom --type odom
        --reference-kind simulator_ground_truth
    and no --out, so trajectory_recorder.py falls back to slam_common.py's
    tum_path(run_id, stack), which is
        trajectory_dir(run_id) / f"{stack}_trajectory.tum"
    i.e. exactly
        logs/sidewalk_slam/trajectories/<run_id>/ground_truth_trajectory.tum
    That filename — ground_truth_trajectory.tum, not "ground_truth.tum" or
    any other guess — is read out of slam_common.py's TumWriter naming,
    not assumed. A ".meta.json" sidecar sits next to it
    (TumWriter.close(), same module) and is read here too, if present, for
    extra provenance in the header this tool writes.

WHY THE CONVERSION IS NOT A STRAIGHT REPLAY
    A human drives continuously: smooth curves, gradual turns while moving,
    pauses, corrections. route_player.py only knows two motions — drive
    dead straight at a held heading (forward), or rotate in place (turn).
    So the continuous path has to be approximated as straight segments
    joined by in-place turns, close enough that replaying the primitives
    retraces essentially the same path, without the file filling up with
    dozens of one-degree corrections from ordinary human steering wobble.

THE ALGORITHM, IN ORDER
    1. Parse the TUM file (timestamp tx ty tz qx qy qz qw, '#' comments
       skipped) and reduce each quaternion to a single heading angle. This
       project pins Reg/Force3DoF true everywhere (ENGINEERING_NOTES.md, robot stays
       flat), so only the yaw component of the quaternion is meaningful;
       quat_to_yaw() below is exactly route_player.py's own yaw_of()
       formula, re-verified here against hand-built quaternions of known
       heading — see _selftest_quat_to_yaw().
    2. Simplify the (x, y) path with the Ramer-Douglas-Peucker algorithm
       (implemented from scratch below — small, and nothing in this
       project's Python 3.8 standard-library-only tooling can be assumed
       to have it installed) to collapse the driven curve into straight
       segments, within --tolerance metres.
    3. Separately, walk the raw samples looking for genuine in-place (or
       near in-place) turns: heading changing substantially while position
       barely advances — a human pausing to rotate, or a tight pivot
       through a doorway. These survive as their own {turn} primitives
       even where step 2's position-only simplification would have smoothed
       straight through them, because position barely moved.
    4. Merge the two: the simplified path's corners plus the detected
       pivots become one ordered list of waypoints. Heading changes below
       --min-turn-deg are rounded to zero and dropped (steering wobble, not
       a real turn); segments shorter than --min-forward-m are merged into
       their neighbours (also noise, not a real leg) — a genuine pivot is
       never dropped this way, whatever its neighbouring segment lengths.
    5. Round distances to 0.01 m and angles to 1 degree — this file is
       meant to be read by a person, not grepped as a float dump.
    6. Forward-simulate the emitted primitives from the recording's own
       first pose, exactly the way route_player.py would execute them
       (drive straight at the held heading, rotate in place by the held
       delta) and compare the simulated end pose against the recording's
       actual last pose. The discrepancy is reported; anything past 0.5 m
       or 10 degrees is a loud warning, not a silent write.
    7. Write the route YAML, matching office1_full.yaml / mcity_building_
       loop.yaml's header and per-primitive comment conventions.

WHAT THIS TOOL DELIBERATELY DOES NOT DO
    It does not drive anything, does not import rospy, and does not check
    the emitted route against a world file's collision geometry the way
    office1_full.yaml's construction script did — that check needs the
    world file open. This route already came from a drive that worked, so
    the aim here is DEED-for-deed reproduction of that drive within the
    stated tolerances, not an independent clearance check. Per the header
    this tool writes: replay it once, watched, before trusting it for a
    measured run — see the "WHY A SHAKEDOWN..." note below.

USAGE
    python3 route_from_drive.py \\
        --tum logs/sidewalk_slam/trajectories/<run_id>/ground_truth_trajectory.tum \\
        --out catkin_ws/src/sidewalk_sim/routes/office1_driven.yaml \\
        --tolerance 0.15 --min-turn-deg 5 --min-forward-m 0.25 \\
        --label "office1, furniture cleared live, first successful manual drive"
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import yaml

# --------------------------------------------------------------------------- #
# Tunable defaults — all three are exposed on the command line (see main()).
# --------------------------------------------------------------------------- #

# Position-simplification tolerance for the Ramer-Douglas-Peucker pass
# (step 2). 0.15 m sits inside the 10-20 cm band the design brief for this
# tool calls for, and the choice is grounded in this project's own measured
# numbers rather than picked blind:
#   - office1_full.yaml's own clearance survey found worst-case clearance
#     as low as 0.650 m on a straight leg (ENGINEERING_NOTES.md's clearance rule wants
#     0.8-1.0 m; this project's tightest doorways don't reach it). Losing
#     more than about 0.15 m of path fidelity on either side of the centre
#     line risks turning a leg that JUST fit into one that clips a doorframe
#     that isn't there in the paper geometry but was real furniture the

#     human actually threaded past.
#   - ordinary human steering wobble at a slow, careful indoor speed is
#     usually a few centimetres per second of lateral drift — well under
#     0.15 m even sampled at 20 Hz over a few seconds between corrections.
# Loose enough to absorb that wobble, tight enough that a doorway threading
# manoeuvre a human actually navigated is not silently smoothed onto the
# wrong side of a doorframe.
DEFAULT_TOLERANCE_M = 0.15

# Below this many degrees of heading change between two consecutive
# simplified-path segments, do not emit a {turn} primitive at all — hold
# the previous heading instead. A "few degrees" per the design brief;
# route_player.py's own in-place-turn tolerance is 2 degrees (TURN_TOL in
# route_player.py), so 5 degrees here is comfortably above the replay
# system's own noise floor without being so loose that a real dog-leg
# (office1_full.yaml uses a real 29-degree one round its reception desk)
# gets rounded away.
DEFAULT_MIN_TURN_DEG = 5.0

# Below this many metres, a straight segment is treated as noise and merged
# into its neighbours rather than emitted as its own {forward} primitive.
# 0.25 m is a quarter of the Husky's own footprint length (0.99 m,
# sidewalk_navigation/config/costmap_common.yaml) — shorter than that isn't
# a leg of the drive, it's the RDP simplification or the pivot detector
# picking a vertex a few samples early or late.
DEFAULT_MIN_FORWARD_M = 0.25

# --------------------------------------------------------------------------- #
# Pivot (in-place turn) detection constants (step 3). These are NOT exposed
# on the command line: they classify raw sensor samples, not the shape of
# the simplified path, and the design brief only asks for the step-4
# thresholds (min-turn-deg, min-forward-m) to be tunable. Values are
# justified individually below.
# --------------------------------------------------------------------------- #

# Classification window for the pivot detector: at each sample, look this
# many SECONDS away (not samples — the ground-truth recorder is a topic
# subscriber sampled at whatever rate the publisher provides, "~100 Hz
# after the publisher's duplicate-stamp guard" per record_mapping_run.
# launch's own comment, not a fixed 20 Hz; a time window is the only choice
# that is correct regardless of the actual recording rate) and classify
# based on NET displacement and heading change across that window, rather
# than a raw consecutive-sample difference. This is not a stylistic choice:
# a first implementation of this tool used raw consecutive-sample
# differencing and it FAILED its own synthetic self-test — independent
# per-sample position jitter (a human recording is not noise-free, and
# this project's own testing convention adds jitter deliberately, see
# make_synthetic_tum.py) produces a sample-to-sample "speed" far above any
# real in-place-turn threshold even while the robot is genuinely
# stationary, because two independent noisy samples 1/rate seconds apart
# can differ by the jitter amplitude in either direction. Net displacement
# across a wider time window averages that independent noise down (it
# behaves like a random walk, growing with the square root of the window,
# not linearly) while genuine translation during driving still shows up at
# its true speed, since real displacement grows LINEARLY with the window.
#
# detect_pivots() builds TWO such windows per sample — one looking forward
# in time, one looking backward — and classifies a sample as rotating if
# EITHER one qualifies. A single forward-only window (this tool's first
# working version) systematically undercounts a real pivot's own delta_deg,
# worse for a faster (shorter-duration) turn: near the pivot's trailing
# edge the forward window necessarily spills past the end of the rotation
# into the straight leg that follows, diluting the measured angular speed
# there below threshold even though the sample is still genuinely rotating,
# which narrows the confirmed run and can drop a real pivot under
# PIVOT_MIN_TOTAL_HEADING_DEG entirely. The backward window is diluted the
# same way at the LEADING edge instead — spilling into the straight leg
# that came before — so between the two, a sample truly inside a pivot
# whose own duration is at least PIVOT_WINDOW_S always has at least one
# dilution-free window looking at it. See detect_pivots()'s own docstring
# for the full reasoning, including the resolution limit this still leaves
# for pivots shorter than PIVOT_WINDOW_S itself.
PIVOT_WINDOW_S = 0.3

# A window counts as "rotating in place" only if net displacement across it
# implies a translational speed below this figure. Chosen well under any
# deliberate driving speed (route_player.py's own default forward speed is
# 0.6 m/s; a careful indoor human drive is unlikely to be faster) so a
# human pausing to turn, or creeping through a tight pivot, is caught,
# while an ordinary curving drive at walking pace is not.
PIVOT_MAX_LINEAR_SPEED_MPS = 0.12

# ...and only if the heading is changing faster than this over the same
# window. Ordinary steering-wobble heading noise while driving straight is
# a couple of degrees over a couple of seconds at most (well under 2
# deg/s); a deliberate in-place turn is an order of magnitude faster than
# that.
PIVOT_MIN_ANGULAR_SPEED_DEGPS = 8.0

# A run of "rotating" windows only counts as a genuine pivot event once the
# TOTAL heading change across the run reaches this many degrees. This sits
# above DEFAULT_MIN_TURN_DEG on purpose: a confirmed pivot event is meant to
# be unambiguously a real, deliberate turn, not something the ordinary
# corner-turn logic in step 4 would already have caught on its own.
PIVOT_MIN_TOTAL_HEADING_DEG = 12.0

# Two runs of "rotating" samples separated by this many SECONDS or fewer
# are merged into one pivot event — enough to bridge a single noisy sample
# in the middle of an otherwise continuous pause-and-turn without merging
# two turns that happen to be genuinely close together in time but are
# otherwise unrelated.
#
# a fixed SAMPLE count. That is the identical mistake PIVOT_TURN_TREND_
# MIN_DEGPS's own comment diagnoses for the turning-point swing filter,
# living in a different piece of this same function: a fixed sample count
# bridges LESS real time the faster the recording, so at high sample
# rates a single genuine pivot's own is_rotating[] classification noise
# (transient windowed-speed dropouts a couple of samples wide, from
# ordinary per-sample jitter) increasingly fails to bridge back together,
# fragmenting one physical pivot into many separate raw runs upstream of
# _find_turning_points() ever running. Measured directly while chasing the
# (75 deg pivot @ 15 deg/s, sigma=2.0 deg jitter) and raising ONLY the
# recording rate, the run's own raw-rotating-run count before any merging
# grew from ~9 at 25 Hz to ~43 at 150 Hz for the SAME physical motion —
# this is what was still driving fragmentation upward with rate even
# after the turning-point swing filter itself was replaced with a
# time-windowed, checkpoint-cadenced rate test.
#
# Value: 2/3 of PIVOT_WINDOW_S (0.2 s). Chosen empirically, not guessed:
# a first attempt at 1/3 of PIVOT_WINDOW_S (0.1 s) removed the HIGH-rate
# tail of the regression (0% fragmentation from 50-150 Hz) but left a
# LOW-rate gap — 20-25 Hz still fragmented 7-25% of the exact regression
# case, because an ordinary 2-3 sample dropout spans proportionally MORE
# real time at a low recording rate (a 3-sample gap is 0.12 s at 25 Hz,
# already past a 0.1 s bridge), so the fixed sample count's original
# failure mode reappeared in miniature even after switching to time units,
# just from the opposite direction. 0.2 s bridges that comfortably at
# every rate from 20-150 Hz (0% fragmentation, N=100 trials/rate) while
# staying well under the ~0.2-0.3 s minimum real separation this project
# has measured between two genuine, oppositely-signed doorway-zigzag
# signed pivots separated by an explicit pause from 0.05 s up to 0.3 s
# still correctly register as two separate events at this gap value,
# across 20/40/100 Hz. See docs/SOLVED.md's route_from_drive.py entry for
# the full rate-sweep numbers.
PIVOT_MAX_GAP_S = PIVOT_WINDOW_S * 2.0 / 3.0

# Minimum SMOOTHED, TIME-WINDOWED heading-rate magnitude, in degrees per
# second, that detect_pivots()._find_turning_points() requires before it
# will confirm a NEW turning point inside an already-confirmed pivot run —
# the confirmation threshold behind a rate-sign hysteresis ("Schmitt
# trigger") filter, not a degree-magnitude swing test. See that function's
# docstring for the full mechanism; this comment is about why the
# threshold is a RATE over a TIME window rather than the raw accumulated-
# degrees "swing from the running extreme" test an earlier version of this
#
# THE BUG THAT RETIRED PIVOT_SWING_MIN_DEG: that version compared each raw
# SAMPLE's cumulative heading value directly against a running extreme,
# with a FIXED-DEGREE pullback required to confirm a reversal. It was
# validated (see its own former comment, still in git history) against a
# synthetic generator whose sample count per trial happened to be capped
# independent of recording rate — a 90-degree pivot there only ever
# produced on the order of 46 samples. But this project's own recorder
# runs at up to ~100 Hz (PIVOT_WINDOW_S's docstring), and a human pausing
# to rotate for a couple of seconds at that rate produces HUNDREDS of raw
# samples for the exact same physical motion. Reproduced directly (2026-
# 08-23): holding a genuine single 75-degree pivot's angle, turn rate, and
# per-sample jitter sigma all fixed and raising ONLY the sample count (via
# recording rate, ~120 samples up to ~1500 samples for the SAME physical
# rotation), fragmentation of that one real pivot into multiple spurious
# {turn} primitives rose from 0% to 65%. Through the real command-line
# tool at a realistic 50 Hz, one genuine 75-degree in-place turn was
# written as FOUR separate primitives ({turn: 48}{turn: 17}{turn: 24}
# {turn: -15}) — and the tool's own self-check reported PASS both before
# and after, because the sum of the fragments still lands near the true
# angle; only the primitive COUNT was wrong.
#
# WHY THIS IS STRUCTURAL, NOT A BAD CONSTANT VALUE: the run's cumulative
# heading curve is built by walking real per-sample yaw jitter one raw
# sample at a time. Comparing a FIXED-DEGREE threshold against that raw
# curve is, in effect, running one independent noise comparison PER RAW
# SAMPLE — an order-statistics problem: the chance that AT LEAST ONE of
# many independent noisy comparisons crosses a fixed threshold purely by
# chance grows with how many comparisons are made, i.e. with sample count,
# for a FIXED physical motion and FIXED per-sample jitter. No single fixed
# degree value can be simultaneously safe at ~46 samples and safe at
# ~1500 samples for the same real rotation — raising it stops a real,
# small reversal from registering at low sample counts before it stops
# noise from registering at high sample counts (see the near-floor
# recovery data in this project's own round-4 sweep history), and
# lowering it does the reverse. This is exactly PIVOT_WINDOW_S's own
# already-solved problem (independent per-sample jitter, tested one raw
# sample-pair apart, "produces a sample-to-sample speed far above any real
# threshold even while stationary") — applied to the SAME raw-sample-
# comparison mistake in a different place in this file.
#
# THE FIX: apply the identical remedy PIVOT_WINDOW_S already established
# for classifying WHETHER a sample is rotating — a fixed TIME width, not a
# fixed sample count — to decide the SIGN of the rotation at each
# candidate split point instead. _pivot_window_signed_rate() (below)
# computes net heading change over a PIVOT_WINDOW_S-wide time window on
# each side of a candidate point and divides by elapsed time, exactly
# mirroring _pivot_window_speeds()'s own windowing. Because the window
# width is fixed in TIME, the number of INDEPENDENT (not just numerous)
# comparisons a genuine pivot's full duration can generate stays bounded
# by (duration / PIVOT_WINDOW_S) regardless of how finely it was sampled —
# raising the recording rate makes neighbouring windows overlap and
# correlate more, not multiplies the count of independent noise draws the
# way raw per-sample comparisons do. A turning point is confirmed only
# once this windowed RATE crosses from confirmed-one-direction to
# confirmed-the-opposite-direction (both sides past this threshold,
# hysteresis-style) — never from a single raw sample's instantaneous
# value.
#
# THRESHOLD VALUE: reuses PIVOT_MIN_ANGULAR_SPEED_DEGPS's own figure
# (8.0 deg/s) rather than inventing an unrelated number — the same
# question ("is this window's net rotation unambiguously real, not
# noise?") is being asked in both places, just once to admit a sample into
# a pivot run at all and once to decide which of two directions a split
# point inside an already-admitted run belongs to. Verified empirically
# at this value across BOTH failure directions and a full 15-150 Hz
# recording-rate sweep — see docs/SOLVED.md's route_from_drive.py entry
PIVOT_TURN_TREND_MIN_DEGPS = PIVOT_MIN_ANGULAR_SPEED_DEGPS

# How often, in seconds of REAL TIME (never raw sample count), detect_
# pivots()._find_turning_points() evaluates its PIVOT_TURN_TREND_MIN_DEGPS
# rate-confirmation test along a candidate run. This is what actually
# fixes the round-5 regression; PIVOT_TURN_TREND_MIN_DEGPS's rate-based
# criterion alone was NOT enough on its own -- see that constant's comment
# for the measured proof. A windowed rate estimate's own noise does not
# grow with sample count (it depends on two endpoint samples within a
# fixed-width time window, not on how many samples sit between them), but
# a threshold TEST evaluated once per RAW sample still runs one test per
# sample -- so at high recording rates, samples fall close enough together
# that a run of them can each independently graze a noisy patch near the
# threshold, and the total number of test opportunities (not any single
# test's own noise level) kept growing with sample count. Spacing the
# tests by a fixed TIME interval instead bounds the number of independent
# test opportunities by (a run's own real duration / this constant),
# regardless of how finely it happened to be sampled -- the same
# discipline PIVOT_WINDOW_S already applies to the window WIDTH, applied
# here to the CADENCE of the decisions built on top of it.
#
# Set to PIVOT_WINDOW_S / 2 -- fine enough to resolve reversals well
# inside the shortest realistic phase durations in this project's own
# multi-reversal test cases (round 4's own +15/-20/+14 degree repro has
# individual phase durations from well under a second up to a couple of
# seconds at realistic turn rates; half a PIVOT_WINDOW_S gives several
# checkpoints even inside the shortest of those), while still keeping the
# per-run checkpoint count bounded by duration rather than sample count.
# round-5 verification (75 deg pivot @ 15 deg/s, sigma=2.0 deg jitter):
# fragmentation/drop rate across 60 trials per rate held at 0% from 25 Hz
# through 150 Hz (the per-raw-sample-cadence attempt above climbed from 8%
# to 73% over the same range at the identical rate threshold) -- see
# docs/SOLVED.md's route_from_drive.py entry for the full rate-sweep
# table, covering both single-pivot survival and multi-reversal recovery.
PIVOT_TURN_CHECKPOINT_SPACING_S = PIVOT_WINDOW_S / 2.0

# When deciding whether an RDP-selected path vertex already covers a
# detected pivot (and should be upgraded to carry the pivot's own measured
# heading delta, rather than the pivot being inserted as a second, nearly-
# coincident waypoint), an RDP vertex within this many samples of the
# pivot's own sample range counts as "the same place".
PIVOT_MERGE_BUFFER_SAMPLES = 5

# Step 6 self-check: how far the simulated end pose is allowed to drift
# from the recording's actual last pose before this tool warns loudly
# instead of writing the file quietly. Matches the design brief's own
# figures.
SELF_CHECK_MAX_POS_M = 0.5
SELF_CHECK_MAX_HEADING_DEG = 10.0


# --------------------------------------------------------------------------- #
# Small geometry helpers — no numpy, matching this project's existing
# pure-Python convention for this kind of arithmetic (slam_common.py,
# route_player.py both do the same).
# --------------------------------------------------------------------------- #

def quat_to_yaw(qx, qy, qz, qw):
    """2D heading (yaw, radians) from a quaternion.

    Only valid because this project pins Reg/Force3DoF true everywhere
    (ENGINEERING_NOTES.md section 1): the robot stays flat, so roll and pitch are
    assumed to be zero and only the yaw component of the quaternion carries
    real information. This is byte-for-byte route_player.py's own yaw_of()
    formula — reusing the identical formula the replay system itself uses
    is deliberate, so a heading computed here and a heading read back by
    route_player.py during replay agree by construction, not by luck.
    """
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def ang_diff(a, b):
    """Shortest signed angle a-b, in radians, result in (-pi, pi]."""
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


def norm_angle(a):
    """Wrap a radian angle into (-pi, pi] — keeps ang_diff() well-behaved
    after many accumulated additions."""
    return ang_diff(a, 0.0)


def _selftest_quat_to_yaw():
    """Verify quat_to_yaw() against hand-built quaternions of known heading
    before trusting it on real data, exactly as the design brief for this
    tool requires. This runs on every invocation (a handful of arithmetic
    operations — unmeasurable cost) rather than being a one-off scratch
    check that could silently rot if the formula were ever edited.
    Raises AssertionError with a specific, diagnosable message on failure;
    the tool refuses to proceed past a broken heading formula.
    """
    cases_deg = [0.0, 90.0, 180.0, -90.0, 45.0, -135.0]
    for expected_deg in cases_deg:
        half = math.radians(expected_deg) / 2.0
        qx, qy, qz, qw = 0.0, 0.0, math.sin(half), math.cos(half)
        got_deg = math.degrees(quat_to_yaw(qx, qy, qz, qw))
        diff_deg = abs(math.degrees(ang_diff(math.radians(expected_deg),
                                             math.radians(got_deg))))
        assert diff_deg < 1e-6, (
            f"quat_to_yaw self-test FAILED: a quaternion built for "
            f"{expected_deg:.1f} degrees of yaw decoded back as "
            f"{got_deg:.4f} degrees (difference {diff_deg:.6f} degrees). "
            f"Refusing to trust this tool's output against real recordings "
            f"until the formula is fixed.")


# --------------------------------------------------------------------------- #
# TUM parsing
# --------------------------------------------------------------------------- #

class Sample:
    __slots__ = ("t", "x", "y", "yaw")

    def __init__(self, t, x, y, yaw):
        self.t = t
        self.x = x
        self.y = y
        self.yaw = yaw


def parse_tum(path):
    """Read a TUM trajectory file into a list of Sample(t, x, y, yaw).

    Format (matches slam_common.py's TumWriter, which produced every TUM
    file in this project): one pose per line, whitespace-separated,
    "timestamp tx ty tz qx qy qz qw", quaternion scalar-last, '#' comment
    lines skipped. This tool re-implements the parse itself rather than
    importing slam_common.py from the sibling sidewalk_slam package —
    deliberately, so this tool stays a genuinely standalone file: it works
    if run from a bare checkout with no catkin workspace built, no
    sys.path surgery, on a machine that has never sourced ROS at all.

    z is parsed and discarded: this project's robot stays flat
    (Reg/Force3DoF pinned true — ENGINEERING_NOTES.md), so only x, y and the
    quaternion's yaw component carry information route_player.py can act
    on.

    Malformed lines are skipped, not fatal — a human recording can include
    the odd short-lived degenerate pose from the recorder's own NaN
    rejection accounting in TumWriter.add(), and losing a handful of
    samples out of a few thousand costs nothing here. The count of skipped
    lines is returned so the caller can report it rather than hide it.
    """
    p = Path(path)
    if not p.exists():
        print(f"error: TUM file does not exist: {p}", file=sys.stderr)
        sys.exit(1)

    samples = []
    skipped = 0
    with open(p) as fh:
        for lineno, raw in enumerate(fh, start=1):
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) != 8:
                skipped += 1
                continue
            try:
                t, tx, ty, tz, qx, qy, qz, qw = (float(v) for v in parts)
            except ValueError:
                skipped += 1
                continue
            if not all(math.isfinite(v) for v in (t, tx, ty, tz, qx, qy, qz, qw)):
                skipped += 1
                continue
            samples.append(Sample(t, tx, ty, quat_to_yaw(qx, qy, qz, qw)))

    if len(samples) < 2:
        print(f"error: {p} yielded fewer than 2 usable poses "
              f"({len(samples)} parsed, {skipped} skipped) — nothing to "
              f"convert.", file=sys.stderr)
        sys.exit(1)

    # TUM files are supposed to be monotonically increasing in time; a
    # recording is not something this tool can fix, but a badly out-of-
    # order file would silently corrupt every distance/speed computation
    # below, so sort defensively and say so if it was actually necessary.
    if any(samples[i].t > samples[i + 1].t for i in range(len(samples) - 1)):
        print("warning: timestamps were not monotonically increasing in "
              f"{p} — sorting by timestamp before processing.",
              file=sys.stderr)
        samples.sort(key=lambda s: s.t)

    return samples, skipped


def load_meta_sidecar(tum_path):
    """Read the .meta.json sidecar TumWriter.close() writes next to every
    TUM file, if it exists. Purely informational — used to enrich the
    generated route's provenance header (ENGINEERING_NOTES.md rule 8: every run writes
    provenance). Returns {} if absent or unreadable; never fatal, since the
    trajectory file itself is all this tool strictly needs.
    """
    p = Path(tum_path)
    sidecar = p.with_suffix(p.suffix + ".meta.json")
    if not sidecar.exists():
        return {}
    try:
        return json.loads(sidecar.read_text())
    except (OSError, ValueError):
        return {}


# --------------------------------------------------------------------------- #
# Step 2 — Ramer-Douglas-Peucker position simplification
# --------------------------------------------------------------------------- #

def _perp_distance(pt, a, b):
    """Perpendicular distance from pt to the infinite line through a, b
    (the standard Ramer-Douglas-Peucker measure — distance to the LINE, not
    the clamped segment)."""
    x0, y0 = pt
    x1, y1 = a
    x2, y2 = b
    dx, dy = x2 - x1, y2 - y1
    if dx == 0.0 and dy == 0.0:
        return math.hypot(x0 - x1, y0 - y1)
    return abs(dy * x0 - dx * y0 + x2 * y1 - y2 * x1) / math.hypot(dx, dy)


def douglas_peucker_indices(points, tolerance):
    """Ramer-Douglas-Peucker simplification, implemented from scratch (no
    library assumed available on this project's pinned Python 3.8 / bare
    standard library toolchain — see ENGINEERING_NOTES.md section 1).

    Returns a sorted list of indices into `points` to keep; index 0 and the
    last index are always kept.

    Implemented iteratively with an explicit stack rather than recursively:
    a 20-45 minute human drive at 20 Hz is up to roughly 24000-54000
    samples, comfortably past Python's default recursion limit (1000) if
    this were written the textbook-recursive way. Worst case is still
    O(n^2) like any Douglas-Peucker implementation (a pathological input
    that never drops a point), but for an offline, run-once conversion tool
    that is an acceptable cost — nothing here runs while the robot is
    moving.
    """
    n = len(points)
    if n < 3:
        return list(range(n))

    keep = {0, n - 1}
    stack = [(0, n - 1)]
    while stack:
        i, j = stack.pop()
        if j <= i + 1:
            continue
        a, b = points[i], points[j]
        max_d, max_k = -1.0, -1
        for k in range(i + 1, j):
            d = _perp_distance(points[k], a, b)
            if d > max_d:
                max_d, max_k = d, k
        if max_d > tolerance:
            keep.add(max_k)
            stack.append((i, max_k))
            stack.append((max_k, j))
    return sorted(keep)


# --------------------------------------------------------------------------- #
# Step 3 — genuine in-place / near-in-place turn detection
# --------------------------------------------------------------------------- #

class PivotEvent:
    __slots__ = ("i0", "i1", "x", "y", "delta_deg", "duration_s", "arc_len_m")

    def __init__(self, i0, i1, x, y, delta_deg, duration_s, arc_len_m):
        self.i0 = i0          # first sample index of the event
        self.i1 = i1          # last sample index of the event
        self.x = x            # representative position (start of the event
        self.y = y            #   — position barely moves during a pivot,
                              #   by definition, so the start point stands
                              #   in for the whole event)
        self.delta_deg = delta_deg    # signed total heading change, degrees
        self.duration_s = duration_s
        self.arc_len_m = arc_len_m    # how far position DID drift, for the
                                      # header's honesty about the approx


def _pivot_window_speeds(samples, i, j):
    """Net translational and angular speed between samples i and j (i < j),
    the shared arithmetic behind both directions of detect_pivots()'s
    windowed classifier: NET displacement between the window's two
    endpoints (not the sum of consecutive per-step distances — see
    PIVOT_WINDOW_S's own comment for why a raw consecutive-sample
    difference does not work here) divided by elapsed time, and total
    heading change (summed as consecutive per-step diffs, not yaw[j]-yaw[i]
    directly, so a turn covering more than half a revolution within one
    window still unwraps correctly) divided by the same elapsed time.

    Returns None if i and j do not bound a usable forward-time interval
    (no room for a window at this end of the recording, or a non-positive
    duration — the latter should already be impossible after parse_tum()'s
    monotonic-timestamp sort, but is checked here defensively rather than
    trusted).
    """
    if j <= i:
        return None
    dt = samples[j].t - samples[i].t
    if dt <= 0.0:
        return None
    net_disp = math.hypot(samples[j].x - samples[i].x,
                          samples[j].y - samples[i].y)
    heading_change = sum(
        math.degrees(ang_diff(samples[k + 1].yaw, samples[k].yaw))
        for k in range(i, j))
    return net_disp / dt, abs(heading_change) / dt


def _pivot_window_signed_rate(samples, i, lo, hi, window_s):
    """Signed, time-windowed heading-rate estimate (degrees/second) AT
    sample i, restricted to the index range [lo, hi]. The mirror-image
    sibling of _pivot_window_speeds(): same windowing discipline (a fixed
    TIME width, never a fixed sample count — see PIVOT_WINDOW_S's own
    comment for why that is the only choice whose noise sensitivity does
    not grow with the recording's sample rate) and the same NET-across-
    the-window arithmetic (heading change computed end-to-end over the
    window, not accumulated and re-tested sample by sample — see
    PIVOT_TURN_TREND_MIN_DEGPS's comment for why that distinction is the
    entire point). Two differences, both because the caller here is
    _find_turning_points(), not the top-level per-sample classifier:

      - SIGNED, not abs() — a turn's direction is exactly what this is
        used to tell apart.
      - Both directions are combined into ONE estimate centred on i: the
        LARGER-MAGNITUDE of the forward and backward windows when both
        exist (see the dilution comment further down, right where that
        comparison happens, for why magnitude-picking and not averaging),
        whichever one exists otherwise.

    Returns None only when i has no usable window in EITHER direction
    within [lo, hi] — i.e. lo == hi == i, which detect_pivots() never asks
    for (a run of a single sample has nothing to split).
    """
    j = i
    while j + 1 <= hi and samples[j + 1].t - samples[i].t <= window_s:
        j += 1
    k = i
    while k - 1 >= lo and samples[i].t - samples[k - 1].t <= window_s:
        k -= 1

    fwd = None
    if j > i:
        dt = samples[j].t - samples[i].t
        if dt > 0.0:
            heading_change = sum(
                math.degrees(ang_diff(samples[m + 1].yaw, samples[m].yaw))
                for m in range(i, j))
            fwd = heading_change / dt

    bwd = None
    if k < i:
        dt = samples[i].t - samples[k].t
        if dt > 0.0:
            heading_change = sum(
                math.degrees(ang_diff(samples[m + 1].yaw, samples[m].yaw))
                for m in range(k, i))
            bwd = heading_change / dt

    # DELIBERATELY the larger-MAGNITUDE of the two, not their average.
    # First implementation of this function averaged them, reasoning that
    # a centred, blended estimate was the natural fit for an interior
    # reintroduces the exact dilution problem PIVOT_WINDOW_S's own
    # docstring already diagnoses for the top-level classifier, just
    # unmitigated here: a middle phase whose own physical duration is
    # short relative to PIVOT_WINDOW_S (a fast reversal sandwiched between
    # two longer opposite-direction legs, e.g. a small sub-floor phase
    # between two confirmable ones) has BOTH its forward and backward
    # windows spilling into the neighbouring, opposite-signed legs --
    # averaging them combines two diluted readings into a still-diluted
    # one, and can leave the blended rate unable to cross
    # PIVOT_TURN_TREND_MIN_DEGPS in the middle phase's own true direction
    # at all, silently failing to split it out (measured concretely:
    # +30/-8/+25 degrees at 20 deg/s, zero jitter -- a deterministic case
    # with no noise to blame -- collapsed from 2 correctly-recovered
    # events down to 0 additional splits with the averaged version).
    # Dilution always pulls a window's magnitude TOWARD the blend with
    # its neighbour, i.e. TOWARD zero relative to the true local rate --
    # so of the two windows, whichever is LESS diluted is the one with
    # the LARGER magnitude, mirroring the top-level classifier's own
    # "either window, whichever passes" logic adapted from a pass/fail
    # union to a magnitude comparison (there is no pass/fail threshold to
    # union over here, only a continuous rate to pick the more trustworthy
    # reading of).
    if fwd is not None and bwd is not None:
        return fwd if abs(fwd) >= abs(bwd) else bwd
    if fwd is not None:
        return fwd
    if bwd is not None:
        return bwd
    return None


def detect_pivots(samples):
    """Find genuine in-place (or near in-place) turns: heading changing
    substantially while position barely advances (step 3 of the module
    docstring's algorithm). These are detected independently of the
    position-only Douglas-Peucker pass, on purpose — RDP works on (x, y)
    alone and can smooth straight through a real doorway pivot precisely
    BECAUSE position barely moved there, which is exactly the failure this
    separate pass exists to catch.

    Classification, per sample i: build TWO windows of up to PIVOT_WINDOW_S
    seconds each — one looking FORWARD from i to some sample j, one looking
    BACKWARD from i to some sample k — and compute net translational/
    angular speed over each with _pivot_window_speeds(). Sample i counts as
    "rotating" if EITHER window's implied translational speed is below
    PIVOT_MAX_LINEAR_SPEED_MPS and implied angular speed is above
    PIVOT_MIN_ANGULAR_SPEED_DEGPS.

    Using only the forward window (this function's first working version)
    systematically UNDERCOUNTS a real pivot's own delta_deg, worse for a
    faster (shorter-duration) turn: near the pivot's trailing edge, the
    forward window necessarily spills past the end of the rotation into the
    straight leg that follows, diluting the measured angular speed there
    below threshold even though the sample is still genuinely rotating —
    and the larger the fraction of the window spent on that inert straight
    driving, the worse the dilution, which is exactly why a faster (shorter)
    true pivot at the same magnitude undercounts harder. Because the
    confirmed run's own delta_deg (below) is summed only over the samples
    that survive classification, a diluted trailing edge shrinks the
    reported run and undercounts the true rotation — or, if enough of the
    run falls away that the remainder no longer reaches
    PIVOT_MIN_TOTAL_HEADING_DEG, drops the pivot entirely with no warning.

    The backward window is the exact mirror image: reliable at a pivot's
    trailing edge and interior, diluted instead at its LEADING edge, where
    it spills into the straight leg that came BEFORE the rotation started.
    Taking either window's verdict means a sample genuinely inside a pivot
    whose own duration is at least PIVOT_WINDOW_S always has at least one
    dilution-free window looking at it — which is what actually recovers
    the true extent of the run, rather than just shifting the same dilution
    problem onto a different (smaller or faster) true pivot the way raising
    PIVOT_MIN_TOTAL_HEADING_DEG or shrinking PIVOT_MAX_LINEAR_SPEED_MPS
    would. (A pivot whose own physical duration is shorter than
    PIVOT_WINDOW_S can still dilute both windows for every one of its
    samples — this is a genuine resolution limit of any fixed-length
    windowed classifier, not something either direction alone or the two
    combined can fully undo. In practice this only bites pivots at or
    below this tool's own PIVOT_MIN_TOTAL_HEADING_DEG confirmation floor
    driven at very fast in-place turn rates — cases this tool is already
    designed to treat as unconfirmed, below-the-noise-floor turns, not a
    regression of the fix.)

    Runs of rotating samples (tolerating small gaps, PIVOT_MAX_GAP_S seconds)
    are candidate pivots; a candidate is confirmed only once its TOTAL
    heading change reaches PIVOT_MIN_TOTAL_HEADING_DEG, which filters out
    momentary noise blips that individually look like fast rotation but
    never add up to a real turn. delta_deg is summed over the confirmed
    run's own [i0, i1] sample range exactly as before — the fix here is
    entirely in which samples survive to be part of that range, not in how
    the range's own delta is measured once found.
    """
    n = len(samples)
    if n < 2:
        return []

    is_rotating = [False] * n
    for i in range(n):
        # Forward window: samples[i..j], j the furthest sample within
        # PIVOT_WINDOW_S seconds AHEAD of i.
        j = i
        while j + 1 < n and samples[j + 1].t - samples[i].t <= PIVOT_WINDOW_S:
            j += 1
        forward = _pivot_window_speeds(samples, i, j)

        # Backward window: samples[k..i], k the furthest sample within
        # PIVOT_WINDOW_S seconds BEHIND i.
        k = i
        while k - 1 >= 0 and samples[i].t - samples[k - 1].t <= PIVOT_WINDOW_S:
            k -= 1
        backward = _pivot_window_speeds(samples, k, i)

        rotating = False
        for result in (forward, backward):
            if result is None:
                continue    # no room for this window at this end of the
                            # recording (or i itself is the last/first
                            # sample) — the other direction may still apply
            lin_speed, ang_speed = result
            if (lin_speed < PIVOT_MAX_LINEAR_SPEED_MPS
                    and ang_speed > PIVOT_MIN_ANGULAR_SPEED_DEGPS):
                rotating = True
                break
        is_rotating[i] = rotating

    # Group contiguous "rotating" samples into raw runs...
    raw_runs = []
    i = 0
    while i < n:
        if not is_rotating[i]:
            i += 1
            continue
        j = i
        while j < n and is_rotating[j]:
            j += 1
        raw_runs.append((i, j - 1))     # samples [i, j-1] are rotating
        i = j

    # ...then merge runs separated by only a few non-rotating samples, so a
    # single noisy sample in the middle of one continuous pause-and-turn
    # doesn't get reported as two separate, smaller pivots.
    #
    # This must NOT fuse two runs that rotate in OPPOSITE directions. A
    # doorway zigzag — two real, close-together, oppositely-signed pivots —
    # produces exactly two short raw runs a handful of samples apart, and
    # blindly summing delta_deg across their combined span makes the two
    # real turns cancel toward zero, silently dropping both underneath
    # degree pair at a 0.05-0.07 m gap vanished entirely, self-check under
    # its own warning threshold because the near-zero net delta looked like
    # a small, honest discrepancy rather than two missing turns). Runs that
    # rotate the SAME way still merge exactly as before — that is the
    # "interrupted by one noisy classification gap" case this loop exists
    # for. A run with a near-zero delta of its own (an ambiguous single
    # noisy sample, not a real rotation either way) does not block a merge.
    def _run_delta_deg(s, e):
        return sum(math.degrees(ang_diff(samples[k + 1].yaw, samples[k].yaw))
                    for k in range(s, e))

    # Direction estimate used ONLY for the sign-aware merge decision below
    # -- NOT the literal per-run delta (_run_delta_deg over the run's own
    # raw [s, e] span), which the original round-3 fix used. A raw run
    # created by a brief mid-pivot classification dropout (the exact
    # dropouts PIVOT_MAX_GAP_S exists to bridge) can be as short as one or
    # two samples, and a literal delta over that few samples is, at higher
    # recording rates, essentially one raw per-sample jitter draw with no
    # averaging at all -- unstable enough in SIGN that it can read as
    # "opposite direction" from the genuine single rotation it is actually
    # part of purely by chance, refusing a merge that should have happened
    # and fragmenting one physical pivot into several confirmed events.
    # even after PIVOT_MAX_GAP_SAMPLES was converted to the time-based
    # PIVOT_MAX_GAP_S above, the exact regression case (75 deg pivot @
    # 15 deg/s, sigma=2.0 deg jitter) still fragmented MORE often at
    # higher recording rates for the identical physical motion -- this
    # literal-delta sign check was the remaining source.
    #
    # _pivot_window_signed_rate() (the same windowing discipline as
    # everywhere else in this function) anchored right at the gap's own
    # two boundary samples fixes the "depends on how many raw samples the
    # adjoining run happens to contain" problem, but the WINDOW WIDTH
    # passed to it still matters and is NOT interchangeable with PIVOT_
    # MAX_GAP_S: a first attempt used PIVOT_MAX_GAP_S itself (0.1 s at the
    # time) as the window width, reasoning that keeping it narrow would
    # avoid smearing into a genuinely different, oppositely-rotating
    # pivot nearby. Measured directly, that made fragmentation WORSE, not
    # not shrink with window width (it is fixed by the two boundary
    # samples' own jitter, see PIVOT_TURN_TREND_MIN_DEGPS's comment), so
    # dividing by a SMALLER window width inflates the noise-to-signal
    # ratio in the resulting RATE rather than improving it -- the true
    # 15 deg/s signal over a 0.1 s window is only 1.5 degrees of true
    # heading change, already smaller than the jitter floor itself. Using
    # the full, already-validated PIVOT_WINDOW_S (0.3 s) instead gives the
    # true signal enough space to dominate the fixed noise floor, and was
    # re-verified NOT to blur across genuine close pivots: two real,
    # oppositely-signed pivots separated by an explicit pause from 0.05 s
    # up to 0.4 s still correctly register as two separate events at this
    # window width, across 20/40/100 Hz (see docs/SOLVED.md's route_from_
    # drive.py entry for the numbers) -- PIVOT_WINDOW_S already sits
    # comfortably under the real separation any two DELIBERATE pivots in
    # this project's own recordings have ever been measured at.
    def _gap_direction_rate(anchor_idx):
        return _pivot_window_signed_rate(samples, anchor_idx, 0, n - 1,
                                         PIVOT_WINDOW_S)

    SIGN_TOL_DEGPS = 2.0
    merged_runs = []
    for s, e in raw_runs:
        if merged_runs:
            ps, pe = merged_runs[-1]
            if samples[s].t - samples[pe].t <= PIVOT_MAX_GAP_S:
                prev_rate = _gap_direction_rate(pe)
                this_rate = _gap_direction_rate(s)
                prev_amb = prev_rate is None or abs(prev_rate) < SIGN_TOL_DEGPS
                this_amb = this_rate is None or abs(this_rate) < SIGN_TOL_DEGPS
                same_direction = (prev_amb or this_amb
                                   or (prev_rate >= 0) == (this_rate >= 0))
                if same_direction:
                    merged_runs[-1] = (ps, e)
                    continue
        merged_runs.append((s, e))

    # 0.2-0.3 s between two real, oppositely-signed pivots) can have window
    # dilution bridge the gap so completely that is_rotating[] never drops
    # between them at all -- the two pivots never separate into distinct
    # raw runs in the first place, so the sign-aware merge above has
    # nothing to act on; it is already a single run when this function
    # first sees it. Detect that case here by looking for genuine internal
    # reversals in the run's own cumulative heading change (it climbs to a
    # real extremum, then visibly turns back), and split the run at each
    # one rather than trusting its single net delta -- which, for two (or
    # more) opposite real rotations, is close to their sum/difference, not
    # any one of them individually, and can easily read as "too small to be
    # a real pivot" even though real pivots are sitting right there.
    #
    # A run can contain more than one such reversal -- e.g. three or more
    # genuine in-place turns run together with only brief holds between
    # them, never dropping out of is_rotating[] because position never
    # moves. An earlier version of this function found only the SINGLE
    # globally-largest reversal and required the WHOLE remainder after it
    # to independently clear PIVOT_MIN_TOTAL_HEADING_DEG before recursing
    # any further -- which silently dropped a run's ENTIRE contents
    # whenever its NET heading change happened to be small, even though
    # every individual rotation inside it was well above the confirmation
    # settle wobble, net delta about +9 degrees, all three real turns
    # vanished with the self-check still reporting PASS, because the
    # resulting heading error is bounded by the small NET delta, not by the
    # magnitude of the missing rotations). The fix below finds EVERY
    # genuine turning point in one pass, splits the run into the leaf
    # segments between them, and applies PIVOT_MIN_TOTAL_HEADING_DEG
    # independently to each leaf's OWN net delta afterwards -- never
    # gating the decision to split on whether some remainder clears the
    # floor first, which is the exact mechanism of the bug above.
    def _cumulative(s, e):
        cum, out = 0.0, {s: 0.0}
        for k in range(s, e):
            cum += math.degrees(ang_diff(samples[k + 1].yaw, samples[k].yaw))
            out[k + 1] = cum
        return out

    def _find_turning_points(s, e):
        """Return the sorted sample indices [s, ..., e] at which the run
        genuinely reverses direction. s and e are always included, even
        when no interior reversal exists.

        REWRITTEN 2026-08-23 (round 5) to fix a sample-rate regression --
        see PIVOT_TURN_TREND_MIN_DEGPS's own comment for the full
        diagnosis and docs/SOLVED.md's route_from_drive.py entry for the
        swept numbers. Summary of the ORIGINAL bug: the prior version
        compared the run's raw cumulative heading-change curve
        (cum(k) == yaw[k]-yaw[s], unwrapped) sample-by-sample against a
        running extreme, confirming a reversal once the curve pulled back
        from that extreme by a FIXED number of degrees -- in effect, one
        independent noise comparison PER RAW SAMPLE, an order-statistics
        problem whose false-positive rate grows with how many samples a
        pivot happens to have been recorded with, for the exact same
        physical motion.

        THE FIRST ATTEMPT AT A FIX (still visible in this function's own
        git history, and worth recording so it is not tried again) simply
        swapped the CRITERION -- comparing a smoothed, time-windowed
        heading RATE (_pivot_window_signed_rate(), same windowing style as
        the already-proven PIVOT_WINDOW_S classifier) against a rate
        threshold instead of comparing raw degrees against a degree
        threshold -- but kept evaluating that criterion at EVERY RAW
        SAMPLE, exactly as before. Measured directly (2026-08-23): this
        genuinely improved things (the exact regression case's 65%
        fragmentation rate at 100 Hz dropped to single digits at 25 Hz)
        but did NOT remove the rate-dependence -- fragmentation of the
        very same physical pivot at the SAME jitter level still climbed
        from 8% (25 Hz) to 73% (150 Hz) as sample count grew. The reason:
        a windowed RATE estimate's own value does not get noisier with
        sample count (its telescoping arithmetic depends only on two
        endpoint samples within the fixed-width window, not on how many
        samples sit between them), but evaluating a THRESHOLD TEST against
        that estimate once per raw sample still runs one test per sample
        -- and at high sample rates the samples fall closely enough
        together that a chain of them can each independently graze past
        the threshold near a noisy patch, so the total NUMBER of test
        opportunities (not the noise level of any one of them) was still
        growing with sample count. Fixing the criterion alone was not
        enough; the CADENCE at which the criterion is evaluated had to
        change too.

        THIS version fixes that: the confirmation test is evaluated only
        at CHECKPOINTS spaced by PIVOT_TURN_CHECKPOINT_SPACING_S seconds
        of real time along the run (see that constant's own comment),
        never once per raw sample. The number of checkpoints -- and so the
        number of independent chances for noise to trip the confirmation
        test -- is bounded by (this run's own real duration /
        PIVOT_TURN_CHECKPOINT_SPACING_S) regardless of recording rate.
        Running-extreme bookkeeping on the raw cum(k) curve (to record
        WHERE -- which sample index -- the true local peak or trough of
        the run actually was, for use as the split boundary) still updates
        at every raw sample: that is a plain running max/min, not a
        threshold test, so it has nothing for noise to spuriously trip and
        does not reintroduce the problem.

        Every reversal found here is returned as a candidate leaf boundary
        with NO floor test applied -- PIVOT_MIN_TOTAL_HEADING_DEG is
        checked once, downstream, against each final leaf's own net delta
        (summed from the RAW per-sample yaw differences over that leaf's
        exact [i0, i1] range, same as always -- only which samples end up
        in which leaf changed here, not how a leaf's own delta is
        measured once found). Splitting first and filtering only at the
        end (rather than gating the split itself on the remainder already
        clearing the floor) is the round-4 fix this rewrite preserves
        unchanged: a leaf too small to confirm on its own is simply
        dropped later, instead of vetoing the split that would have
        exposed its (confirmable) neighbours.

        IMPORTANT: a turning point is only ever added to the result at the
        moment a reversal is CONFIRMED by the checkpoint rate test --
        never speculatively for "the running extreme reached so far" once
        the loop simply runs out of samples still mid-trend. An earlier
        version of this function closed the final leg at that last,
        UNCONFIRMED running extreme instead of at e, which silently
        trimmed every ordinary single pivot whose run happens to end on a
        small sub-threshold pullback (recording noise settling after the
        true rotation, not a second real turn). The run's own end (e)
        always closes the final leg, confirmed reversal or not -- this
        rewrite keeps that fix exactly as it was.
        """
        cum_at = _cumulative(s, e)
        if e <= s:
            return [s]

        # Checkpoints: sample indices spaced by real TIME
        # (PIVOT_TURN_CHECKPOINT_SPACING_S), not by raw sample count. s and
        # e always act as checkpoints (e unconditionally, appended below if
        # the spacing loop did not already land on it exactly).
        checkpoints = [s]
        next_t = samples[s].t + PIVOT_TURN_CHECKPOINT_SPACING_S
        for idx in range(s + 1, e + 1):
            if samples[idx].t >= next_t:
                checkpoints.append(idx)
                next_t = samples[idx].t + PIVOT_TURN_CHECKPOINT_SPACING_S
        if checkpoints[-1] != e:
            checkpoints.append(e)
        checkpoint_set = set(checkpoints)

        turning = [s]
        trend = 0                      # 0 undetermined, +1 rising, -1 falling
        extreme_idx, extreme_val = s, cum_at[s]
        for idx in range(s + 1, e + 1):
            val = cum_at[idx]
            # Running-extreme bookkeeping: full raw resolution, every
            # sample, regardless of checkpoint cadence -- a running max/min
            # is not a threshold test, nothing here for noise to
            # spuriously trip (see the docstring above).
            if trend == 1 and val >= extreme_val:
                extreme_idx, extreme_val = idx, val
            elif trend == -1 and val <= extreme_val:
                extreme_idx, extreme_val = idx, val

            if idx not in checkpoint_set:
                continue    # confirmation test only runs at checkpoints

            rate = _pivot_window_signed_rate(samples, idx, s, e, PIVOT_WINDOW_S)
            r = rate if rate is not None else 0.0
            if trend == 0:
                if r >= PIVOT_TURN_TREND_MIN_DEGPS:
                    trend = 1
                    extreme_idx, extreme_val = idx, val
                elif r <= -PIVOT_TURN_TREND_MIN_DEGPS:
                    trend = -1
                    extreme_idx, extreme_val = idx, val
                # else: the windowed rate at this checkpoint isn't
                # unambiguously one direction yet -- no direction
                # confirmed, keep scanning from s.
            elif trend == 1:
                if r <= -PIVOT_TURN_TREND_MIN_DEGPS:
                    turning.append(extreme_idx)
                    trend, extreme_idx, extreme_val = -1, idx, val
            else:   # trend == -1
                if r >= PIVOT_TURN_TREND_MIN_DEGPS:
                    turning.append(extreme_idx)
                    trend, extreme_idx, extreme_val = 1, idx, val

        if turning[-1] != e:
            turning.append(e)
        return turning

    def _split_at_reversal(s, e):
        if e <= s:
            return [(s, e)]
        turning = _find_turning_points(s, e)
        return list(zip(turning, turning[1:]))

    split_runs = []
    for s, e in merged_runs:
        split_runs.extend(_split_at_reversal(s, e))

    events = []
    for s, e in split_runs:
        i0, i1 = s, e            # sample index range covered by this run
        delta_deg = sum(math.degrees(ang_diff(samples[k + 1].yaw, samples[k].yaw))
                        for k in range(i0, i1))
        if abs(delta_deg) < PIVOT_MIN_TOTAL_HEADING_DEG:
            continue              # too small to be confirmed as a real pivot
        arc_len = sum(math.hypot(samples[k + 1].x - samples[k].x,
                                 samples[k + 1].y - samples[k].y)
                     for k in range(i0, i1))
        events.append(PivotEvent(
            i0=i0, i1=i1, x=samples[i0].x, y=samples[i0].y,
            delta_deg=delta_deg, duration_s=samples[i1].t - samples[i0].t,
            arc_len_m=arc_len))
    return events


# --------------------------------------------------------------------------- #
# Step 4 — merge shape vertices and pivots into one waypoint list, then
# drop/merge segments below the noise floor.
# --------------------------------------------------------------------------- #

def build_waypoints(samples, rdp_indices, pivot_events):
    """Merge the Douglas-Peucker corners (step 2) and the detected pivots
    (step 3) into one time-ordered list of waypoint dicts:
        {idx, x, y, is_pivot, pivot_delta_deg, pivot_duration_s,
         pivot_exit_x, pivot_exit_y}
    An RDP vertex that falls inside (or within PIVOT_MERGE_BUFFER_SAMPLES
    of) a detected pivot's own sample range is dropped in favour of the
    pivot's own point — the pivot's position and measured heading delta are
    a more faithful description of that stretch of the recording than an
    arbitrary in-between RDP vertex would be.

    Every pivot waypoint also carries pivot_exit_x/pivot_exit_y —
    samples[pe.i1]'s own recorded position, i.e. wherever the robot
    physically was when the pivot's rotation actually finished. This exists
    so emit_primitives() can anchor the LEG LEAVING a pivot at the position
    driving actually resumed from, instead of pe.x/pe.y (samples[pe.i0],
    the START of the rotation) — a full pivot-duration-stale point for that
    purpose. See emit_primitives()'s own docstring for why this specific
    substitution (position, not heading) is the robust one.
    """
    pivot_ranges = [(pe.i0 - PIVOT_MERGE_BUFFER_SAMPLES,
                     pe.i1 + PIVOT_MERGE_BUFFER_SAMPLES) for pe in pivot_events]

    waypoints = []
    for pe in pivot_events:
        waypoints.append({
            "idx": pe.i0, "x": pe.x, "y": pe.y, "is_pivot": True,
            "pivot_delta_deg": pe.delta_deg, "pivot_duration_s": pe.duration_s,
            "pivot_exit_x": samples[pe.i1].x, "pivot_exit_y": samples[pe.i1].y,
        })
    for idx in rdp_indices:
        if any(lo <= idx <= hi for lo, hi in pivot_ranges):
            continue
        s = samples[idx]
        waypoints.append({
            "idx": idx, "x": s.x, "y": s.y, "is_pivot": False,
            "pivot_delta_deg": None, "pivot_duration_s": None,
            "pivot_exit_x": None, "pivot_exit_y": None,
        })

    waypoints.sort(key=lambda w: w["idx"])

    # Defensive: the recording's first and last sample must always anchor
    # the waypoint list (they define the spawn-pose assumption in the
    # written header, and the step-6 self-check's target respectively).
    # Ordinary Douglas-Peucker construction already guarantees this; this
    # only fires if a pivot's merge window somehow swallowed an endpoint.
    if waypoints[0]["idx"] != 0:
        s = samples[0]
        waypoints.insert(0, {"idx": 0, "x": s.x, "y": s.y, "is_pivot": False,
                             "pivot_delta_deg": None, "pivot_duration_s": None,
                             "pivot_exit_x": None, "pivot_exit_y": None})
    last = len(samples) - 1
    if waypoints[-1]["idx"] != last:
        s = samples[last]
        waypoints.append({"idx": last, "x": s.x, "y": s.y, "is_pivot": False,
                          "pivot_delta_deg": None, "pivot_duration_s": None,
                          "pivot_exit_x": None, "pivot_exit_y": None})
    return waypoints


def _pivot_adjacent(wps, m):
    """True if waypoint m in the CURRENT wps list is a confirmed pivot, or
    is the waypoint immediately next to one on either side.

    This is deliberately broader than "is m itself a pivot": a short leg
    sitting directly before or after a pivot is real information (the
    doorway geometry a human actually threaded, or the gap between two
    close, deliberate corrections), not RDP/pivot-detector placement noise
    — collapse_short_segments() must never delete or shorten the segment
    structure across a confirmed pivot's own neighbourhood, whatever its
    length. Ordinary path-corner geometry away from any pivot is still
    simplified exactly as before; only this neighbourhood is protected.
    """
    if wps[m]["is_pivot"]:
        return True
    if m > 0 and wps[m - 1]["is_pivot"]:
        return True
    if m < len(wps) - 1 and wps[m + 1]["is_pivot"]:
        return True
    return False


def collapse_short_segments(waypoints, min_forward_m):
    """Drop/merge waypoints that create a segment shorter than
    min_forward_m — ordinary noise from where exactly RDP or the pivot
    detector happened to land a vertex, not a real leg of the drive.

    A confirmed pivot's own neighbourhood (the pivot waypoint itself, and
    whichever waypoints sit immediately before/after it — see
    _pivot_adjacent()) is never touched this way, whatever its neighbouring
    segment lengths are — the brief step 4's own exception for step 3's
    pivots, deliberately widened past just the pivot's own waypoint: a
    short leg immediately adjoining a pivot (e.g. the gap between two real,
    close-together in-place corrections while threading a doorway) is
    exactly the shape a length-floor-only check would otherwise delete,
    silently merging or cancelling two genuine, separate turns (see
    docs/SOLVED.md's route_from_drive.py entry for the failure this
    protects against). Likewise the very first and very last waypoint are
    never dropped: they are the recording's own start and end pose, which
    the written header's spawn-pose note and the step-6 self-check both
    depend on staying exactly what was actually recorded.

    When a short segment's later endpoint can be dropped, it is; otherwise
    (the later endpoint is protected) the earlier endpoint is dropped
    instead, if that one is not protected either. If BOTH endpoints of a
    short segment are protected — which can genuinely happen right at the
    very start or end of a recording, or in a pivot's own neighbourhood —
    the segment is simply left un-lengthened; the emission step below knows
    to skip emitting a {forward} for a sub-floor distance regardless of why
    it is short, and never loses the pivot itself in the process.
    """
    wps = list(waypoints)
    changed = True
    while changed and len(wps) > 2:
        changed = False
        for k in range(1, len(wps)):
            d = math.hypot(wps[k]["x"] - wps[k - 1]["x"],
                           wps[k]["y"] - wps[k - 1]["y"])
            if d >= min_forward_m:
                continue
            if not _pivot_adjacent(wps, k) and k != len(wps) - 1:
                del wps[k]
                changed = True
                break
            if not _pivot_adjacent(wps, k - 1) and k - 1 != 0:
                del wps[k - 1]
                changed = True
                break
        # else: every short segment left standing has both ends protected —
        # nothing more this pass can safely remove.
    return wps


# --------------------------------------------------------------------------- #
# Primitive emission
# --------------------------------------------------------------------------- #

def emit_primitives(waypoints, start_yaw_rad, min_turn_deg, min_forward_m):
    """Walk the cleaned waypoint list and emit {turn}/{forward} primitives,
    holding heading between them exactly the way route_player.py's own
    forward()/turn() do. Returns (primitives, ending_heading_rad).

    Each primitive dict carries the position it leaves the robot at
    ('x', 'y') purely so the YAML writer can generate the same
    "# to (x, y)" / "# face ..." per-primitive comments office1_full.yaml
    and mcity_building_loop.yaml already use — it has no bearing on replay.

    Heading is tracked using the ROUNDED degree values that will actually
    be written to the file (round() applied here, not deferred to the
    writer), because the step-6 self-check needs to simulate the exact
    primitive list a human will later read out of the YAML and
    route_player.py will actually execute — simulating unrounded internal
    angles would silently under-report the real discrepancy.

    ORDINARY CORNER-TURN HEADING, AND WHY THE LEG LEAVING A PIVOT IS
    SPECIAL-CASED: for a leg between two ordinary (non-pivot) waypoints, or
    a leg arriving AT a pivot, the direction of that leg is computed the
    obvious way, atan2(dy, dx) between the two waypoints' positions — safe,
    because the "target" side is always a Douglas-Peucker vertex (or, for
    the leg into a pivot, the pivot's own arrival point, which many prior
    samples of the incoming leg all converge toward), so the line's
    direction is corroborated by more than just one single sample.

    The leg LEAVING a pivot does not have that property, and an earlier
    version of this function anchored it at pe.x/pe.y — samples[pe.i0], the
    START of the pivot's rotation — which is stale by the pivot's own full
    duration by the time driving actually resumes. Anchoring instead at
    pivot_exit_x/pivot_exit_y (samples[pe.i1], where the robot physically
    was when rotation finished) removes that staleness and is the more
    defensible position-based estimate for that specific leg.

    A YAW-based anchor (samples[pe.i1].yaw, the recorded heading right as
    rotation finished) was tried and reverted: it is sensitive to exactly
    where detect_pivots() drew the confirmed run's own boundary, and under
    heavier per-sample jitter that boundary can land measurably early or
    late — while POSITION barely moves across a pivot's own span regardless
    of where the boundary was drawn (that is the entire premise a pivot is
    detected on), so a position anchor stays valid even when the confirmed
    run's exact extent is imperfect. See docs/DO_NOT_REPEAT.md for the
    concrete regression (loop_revisit.tum) that ruled the yaw-based version
    out.
    """
    primitives = []
    current_heading = start_yaw_rad
    prev = waypoints[0]

    # A pivot as the very FIRST waypoint (the operator rotating in place
    # before ever driving off) is invisible to the loop below: that loop
    # only inspects wp["is_pivot"] for waypoints[1:], and prev is assigned
    # from waypoints[0] without ever itself passing through the pivot
    # turn silently lost that value and instead emitted a corner-turn to
    # some later waypoint under the same "measured from the recording"
    # framing the real pivot handling below uses — self-check still
    # reported PASS. Handle waypoints[0] with the identical logic the loop
    # applies to every later pivot waypoint, before the loop starts.
    if prev["is_pivot"]:
        delta_r = round(prev["pivot_delta_deg"])
        if abs(delta_r) >= min_turn_deg and delta_r != 0:
            primitives.append({"type": "turn", "value": float(delta_r),
                               "x": prev["x"], "y": prev["y"], "pivot": True,
                               "duration_s": prev["pivot_duration_s"]})
            current_heading = norm_angle(current_heading + math.radians(delta_r))

    for wp in waypoints[1:]:
        dx, dy = wp["x"] - prev["x"], wp["y"] - prev["y"]
        dist = math.hypot(dx, dy)

        if dist >= min_forward_m:
            if prev["is_pivot"]:
                seg_heading = math.atan2(wp["y"] - prev["pivot_exit_y"],
                                         wp["x"] - prev["pivot_exit_x"])
            else:
                seg_heading = math.atan2(dy, dx)
            turn_deg = math.degrees(ang_diff(seg_heading, current_heading))
            turn_r = round(turn_deg)
            if abs(turn_r) >= min_turn_deg and turn_r != 0:
                primitives.append({"type": "turn", "value": float(turn_r),
                                   "x": prev["x"], "y": prev["y"],
                                   "pivot": False})
                current_heading = norm_angle(current_heading + math.radians(turn_r))
            primitives.append({"type": "forward", "value": round(dist, 2),
                               "x": wp["x"], "y": wp["y"]})
        # else: sub-floor segment that survived collapse_short_segments()
        # only because both its endpoints were protected (see that
        # function's docstring) — no {forward} emitted, heading untouched,
        # and control falls straight through to the pivot handling below
        # using this same waypoint's position, which is ~= prev's by
        # definition of being sub-floor.

        if wp["is_pivot"]:
            delta_r = round(wp["pivot_delta_deg"])
            if abs(delta_r) >= min_turn_deg and delta_r != 0:
                primitives.append({"type": "turn", "value": float(delta_r),
                                   "x": wp["x"], "y": wp["y"], "pivot": True,
                                   "duration_s": wp["pivot_duration_s"]})
                current_heading = norm_angle(current_heading + math.radians(delta_r))

        prev = wp

    return primitives, current_heading


def compact_primitives(primitives):
    """Merge consecutive primitives of the same type with nothing between
    them. This is always physically exact, not just tidy: two {turn}
    primitives with no {forward} between them rotate the robot twice at
    the SAME location, which is identical to rotating once by their sum
    (route_player.py's turn() only ever applies a delta to whatever heading
    it is currently holding); two {forward} primitives with no {turn}
    between them drive straight at the same held heading twice in a row,
    identical to driving the summed distance once. Entries that net to
    zero after merging (e.g. a +90 ordinary corner immediately followed by
    a -90 correction) are dropped entirely rather than written as a
    {turn: 0} or {forward: 0.0} no-op.

    EXCEPTION: a {turn} that came from a confirmed pivot (pivot=True) is
    NEVER merged with a neighbouring {turn}, pivot or not. A pivot's value
    is a direct, independently-measured rotation (detect_pivots(), summed
    over the confirmed run's own yaw samples) — folding it together with an
    adjacent ordinary corner-turn (or another pivot) would silently average
    a real, deliberate rotation against a second, separately-computed one
    and then relabel the result "in-place pivot measured from the
    recording", which is not what was measured. Two real, distinct
    rotations sitting next to each other in time — a pivot immediately
    followed by a small ordinary corner correction, or two close pivots —
    are legitimate as two separate {turn} primitives in the output file;
    route_player.py executes consecutive {turn} primitives with no issue.
    Merging still happens, exactly as before, for two ordinary (non-pivot)
    turns next to each other, and unconditionally for {forward} primitives.
    """
    merged = []
    for p in primitives:
        mergeable = (merged and merged[-1]["type"] == p["type"]
                    and not (p["type"] == "turn"
                             and (merged[-1].get("pivot") or p.get("pivot"))))
        if mergeable:
            merged[-1]["value"] = round(merged[-1]["value"] + p["value"],
                                        2 if p["type"] == "forward" else 0)
            merged[-1]["x"], merged[-1]["y"] = p["x"], p["y"]
        else:
            merged.append(dict(p))

    out = []
    for p in merged:
        if p["type"] == "forward" and p["value"] <= 0.0:
            continue
        if p["type"] == "turn" and p["value"] == 0.0:
            continue
        out.append(p)
    return out


def simulate_primitives(primitives, x0, y0, yaw0_rad):
    """Forward-simulate the emitted primitive list exactly the way
    route_player.py's forward()/turn() would: drive straight at the held
    heading for the primitive's distance, or rotate in place by the
    primitive's delta. This is the step-6 self-check the design brief
    requires — comparing this simulated end pose against the recording's
    actual last pose is the only evidence in this file that the emitted
    route reproduces what was driven, rather than merely looking plausible.
    """
    x, y, yaw = x0, y0, yaw0_rad
    for p in primitives:
        if p["type"] == "forward":
            x += p["value"] * math.cos(yaw)
            y += p["value"] * math.sin(yaw)
        else:
            yaw = norm_angle(yaw + math.radians(p["value"]))
    return x, y, yaw


# --------------------------------------------------------------------------- #
# YAML rendering
# --------------------------------------------------------------------------- #

_COMPASS = [
    (0.0, "east"), (45.0, "north-east"), (90.0, "north"),
    (135.0, "north-west"), (180.0, "west"), (-135.0, "south-west"),
    (-90.0, "south"), (-45.0, "south-east"),
]


def _compass_label(heading_deg, tol_deg=6.0):
    """A cardinal/intercardinal label if heading_deg is close to one,
    matching office1_full.yaml's own "# face south" / "# face north-east"
    comment style; None if the heading doesn't land near one of the eight
    principal directions, in which case the caller falls back to printing
    the bare number."""
    h = ((heading_deg + 180.0) % 360.0) - 180.0
    for ref, label in _COMPASS:
        d = abs(((h - ref + 180.0) % 360.0) - 180.0)
        if d <= tol_deg:
            return label
    return None


def render_route_yaml(primitives, header_lines, running_heading_deg0):
    """Build the full route file text: the header comment block followed by
    the primitive list, one YAML flow-mapping per line with a trailing
    comment — the exact format office1_full.yaml and
    mcity_building_loop.yaml already use, so this tool's output reads like
    a route file a person on this project wrote, not a generic YAML dump.
    """
    lines = list(header_lines)
    lines.append("")

    heading_deg = running_heading_deg0

    for p in primitives:
        if p["type"] == "turn":
            heading_deg = ((heading_deg + p["value"] + 180.0) % 360.0) - 180.0
            label = _compass_label(heading_deg)
            facing = (f"face {label}" if label
                     else f"turn to heading {heading_deg:.0f} deg")
            provenance = (" (in-place pivot measured from the recording, "
                         f"{p.get('duration_s') or 0:.1f} s)"
                         if p.get("pivot") else " (path corner)")
            entry = f"- {{turn: {p['value']:g}}}"
            pad = " " * max(1, 22 - len(entry))
            lines.append(f"{entry}{pad}# {facing}{provenance}")
        else:
            x, y = p["x"], p["y"]
            entry = f"- {{forward: {p['value']:.2f}}}"
            pad = " " * max(1, 22 - len(entry))
            lines.append(f"{entry}{pad}# to ({x:.2f}, {y:.2f})")
    lines.append("")
    return "\n".join(lines)


# --------------------------------------------------------------------------- #
# Header construction
# --------------------------------------------------------------------------- #

def build_header(args, samples, skipped, meta, primitives, total_forward_m,
                 self_check, run_id, tolerance_note):
    """Build the route file's header comment block: source, spawn pose,
    generation parameters, totals, the step-6 self-check result, and the
    explicit distinction from the depth study-derived routes this project also
    carries. Matches office1_full.yaml / mcity_building_loop.yaml's own
    banner-and-explanation convention.
    """
    first, last = samples[0], samples[-1]
    spawn_yaw_deg = math.degrees(first.yaw)
    out_name = Path(args.out).name

    pos_disc, heading_disc_deg = self_check
    self_check_ok = (pos_disc <= SELF_CHECK_MAX_POS_M
                     and heading_disc_deg <= SELF_CHECK_MAX_HEADING_DEG)

    lines = []
    lines.append(f"# {out_name} — route generated from a human-driven "
                 f"recording by route_from_drive.py.")
    if args.label:
        lines.append(f"# LABEL: {args.label}")
    lines.append("#")
    lines.append("# UNLIKE office1_full.yaml / office2_full.yaml / "
                 "mcity_building_loop.yaml, which were built by")
    lines.append("# walking a world file's collision geometry on paper and "
                 "have never been driven, this route")
    lines.append("# reproduces a path a human already drove successfully in "
                 "the simulator. That is strictly")
    lines.append("# stronger evidence than a paper-derived route — nobody "
                 "had to guess whether it fits past the")
    lines.append("# furniture, because it already did, once, with a human "
                 "watching.")
    lines.append("#")
    lines.append("# IT IS STILL NOT A GUARANTEE OF A CLEAN REPLAY. "
                 "route_player.py drives on WHEEL-ODOMETRY")
    lines.append("# distance and INERTIAL MEASUREMENT UNIT heading, not on "
                 "the ground truth these primitives were computed")
    lines.append("# from — the same sensors every other route in this "
                 "project replays on, and the same reason")
    lines.append("# office1_full.yaml and mcity_building_loop.yaml both "
                 "call for a slow, watched shakedown")
    lines.append("# before being trusted for a measured run. Do the same "
                 "here before using this file in a")
    lines.append("# campaign.")
    lines.append("#")
    lines.append("# ---------------------------------------------------------"
                 "----------------")
    lines.append("# SOURCE")
    lines.append("# ---------------------------------------------------------"
                 "----------------")
    run_id_display = run_id or "(could not be inferred — see note below)"
    lines.append(f"#   recorded trajectory : {args.tum}")
    lines.append(f"#   run id               : {run_id_display}")
    if not run_id:
        lines.append("#     the TUM path did not match the canonical "
                     "logs/sidewalk_slam/trajectories/<run_id>/")
        lines.append("#     layout record_mapping_run.launch writes to "
                     "(trajectory_recorder.py + slam_common.py's")
        lines.append("#     tum_path()), so the run id could not be read "
                     "back out of the path alone.")
    if meta:
        ref_kind = meta.get("reference_kind", "(not recorded)")
        lines.append(f"#   meta.json reference_kind : {ref_kind}")
        if ref_kind != "simulator_ground_truth":
            lines.append("#     WARNING: this sidecar does not declare "
                         "simulator_ground_truth — confirm the input")
            lines.append("#     really is the ground-truth recorder's "
                         "output (traj_rec_truth in")
            lines.append("#     record_mapping_run.launch), not the SLAM "
                         "estimate (traj_rec_slam). Replaying a")
            lines.append("#     route built from the SLAM estimate would "
                         "bake that run's own drift into every")
            lines.append("#     future replay.")
        if meta.get("poses_written") is not None:
            lines.append(f"#   meta.json poses_written  : {meta['poses_written']}")
        if meta.get("mean_rate_hz") is not None:
            lines.append(f"#   meta.json mean_rate_hz   : {meta['mean_rate_hz']}")
    else:
        lines.append("#   no .meta.json sidecar found next to the TUM file "
                     "— proceeding on the TUM file's")
        lines.append("#   own content alone. Provenance is weaker without "
                     "it; if this is unexpected, check")
        lines.append("#   record_mapping_run.launch actually ran with "
                     "ground_truth:=true.")
    if skipped:
        lines.append(f"#   {skipped} malformed line(s) in the TUM file were "
                     "skipped during parsing.")
    lines.append(f"#   samples used          : {len(samples)} "
                 f"(t = {first.t:.3f} .. {last.t:.3f}, "
                 f"{last.t - first.t:.1f} s)")
    lines.append("#")
    lines.append("# ---------------------------------------------------------"
                 "----------------")
    lines.append("# SPAWN POSE — THE ROUTE IS MEANINGLESS FROM ANY OTHER "
                 "START")
    lines.append("# ---------------------------------------------------------"
                 "----------------")
    lines.append("# Every primitive below is relative to the one before it, "
                 "taken from the recording's own")
    lines.append("# first sample — start anywhere else and the primitives "
                 "point somewhere else entirely.")
    lines.append("#")
    lines.append(f"#   spawn_x:={first.x:.3f}   spawn_y:={first.y:.3f}   "
                 f"spawn_yaw:={spawn_yaw_deg:.1f}")
    lines.append("#")
    lines.append("# spawn_z is not controlled by this route (the flat-floor "
                 "assumption behind Reg/Force3DoF")
    lines.append("# means height never enters this tool's arithmetic) — use "
                 "whatever the world's own launch")
    lines.append("# file already defaults it to.")
    lines.append("#")
    lines.append("# Reset the robot to this pose before every recorded run "
                 "(ENGINEERING_NOTES.md rule 10): RTAB-Map")
    lines.append("# anchors its map frame wherever the robot starts, so a "
                 "different start silently offsets")
    lines.append("# every error reading.")
    lines.append("#")
    lines.append("# ---------------------------------------------------------"
                 "----------------")
    lines.append("# HOW THIS FILE WAS BUILT")
    lines.append("# ---------------------------------------------------------"
                 "----------------")
    lines.append("# route_from_drive.py simplified the recorded (x, y) path "
                 "with the Ramer-Douglas-Peucker")
    lines.append(f"# algorithm at a tolerance of {args.tolerance:.3f} m "
                 f"({tolerance_note}),")
    lines.append("# then separately searched the raw recording for genuine "
                 "in-place turns (heading changing")
    lines.append("# substantially — a total of at least "
                 f"{PIVOT_MIN_TOTAL_HEADING_DEG:.0f} degrees — while position "
                 "barely advanced), so a")
    lines.append("# real doorway pivot the human actually drove survives "
                 "even where the position-only")
    lines.append("# simplification alone would have smoothed straight "
                 "through it. Heading changes below")
    lines.append(f"# {args.min_turn_deg:.0f} degrees between segments were "
                 f"rounded to zero and dropped as ordinary")
    lines.append("# steering wobble; straight segments shorter than "
                 f"{args.min_forward_m:.2f} m were merged into their")
    lines.append("# neighbours for the same reason. Every distance below is "
                 "rounded to 0.01 m and every")
    lines.append("# angle to 1 degree — this file is meant to be read, not "
                 "grepped as a float dump.")
    lines.append("#")
    lines.append("# SELF-CHECK: the primitives below were forward-simulated "
                 "from the spawn pose above,")
    lines.append("# exactly the way route_player.py executes them (drive "
                 "straight at the held heading,")
    lines.append("# rotate in place by the held delta), and the simulated "
                 "end pose was compared against")
    lines.append("# the recording's own actual last pose:")
    lines.append("#")
    lines.append(f"#   position discrepancy : {pos_disc:.3f} m")
    lines.append(f"#   heading discrepancy  : {heading_disc_deg:.1f} deg")
    if self_check_ok:
        lines.append(f"#   PASS — within the {SELF_CHECK_MAX_POS_M:.1f} m / "
                     f"{SELF_CHECK_MAX_HEADING_DEG:.0f} deg bound this tool "
                     "warns past.")
    else:
        lines.append("#   *** WARNING: EXCEEDS the "
                     f"{SELF_CHECK_MAX_POS_M:.1f} m / "
                     f"{SELF_CHECK_MAX_HEADING_DEG:.0f} deg bound this tool "
                     "warns past. ***")
        lines.append("#   The straight-line/in-place-turn approximation may "
                     "have cut a corner this specific")
        lines.append("#   drive actually needed (a curve hugging a doorway "
                     "edge, for instance). Consider a")
        lines.append("#   tighter --tolerance before trusting this file, and "
                     "treat the shakedown replay below")
        lines.append("#   as mandatory, not optional.")
    lines.append("#")
    lines.append(f"#   primitive count       : {len(primitives)}")
    lines.append(f"#   total forward distance: {total_forward_m:.2f} m")
    lines.append("#")
    lines.append("# ---------------------------------------------------------"
                 "----------------")
    lines.append("# WHY A SHAKEDOWN REPLAY COMES FIRST, EVEN THOUGH A HUMAN "
                 "ALREADY DROVE THIS PATH")
    lines.append("# ---------------------------------------------------------"
                 "----------------")
    lines.append("# The primitives above were COMPUTED from ground truth, "
                 "but route_player.py will EXECUTE")
    lines.append("# them on wheel-odometry distance and inertial measurement "
                 "unit heading — the same two")
    lines.append("# platform sensors every other scripted route in this "
                 "project uses, and neither one is")
    lines.append("# ground truth. Wheel slip, encoder drift over the route's "
                 "total distance, and inertial")
    lines.append("# measurement unit heading error can all make the "
                 "replayed path diverge from the path")
    lines.append("# that was actually driven, even though the primitives "
                 "themselves are an accurate description")
    lines.append("# of that drive. Run this once, slowly, with someone "
                 "watching the simulator window and ready to")
    lines.append("# stop it, before trusting it for an unattended or "
                 "measured campaign run — exactly the")
    lines.append("# same rule office1_full.yaml and mcity_building_loop.yaml "
                 "state for themselves, for the")
    lines.append("# same reason.")
    lines.append("#")
    lines.append("# Reading the comments below: each \"forward\" comment "
                 "gives the world coordinate the")
    lines.append("# robot should be standing on when that primitive "
                 "finishes (same convention as")
    lines.append("# office1_full.yaml / mcity_building_loop.yaml). Headings: "
                 "0 degrees is +X (east), 90 is")
    lines.append("# +Y (north), turns are counter-clockwise-positive. A "
                 "\"(in-place pivot measured from")
    lines.append("# the recording)\" note marks a turn step 3 detected as a "
                 "genuine pause-and-rotate in the")
    lines.append("# original drive, as distinct from an ordinary \"(path "
                 "corner)\" turn the simplified")
    lines.append("# path's own geometry required.")
    return lines


# --------------------------------------------------------------------------- #
# run id inference — matches slam_common.py's tum_path()/trajectory_dir()
# layout: logs/sidewalk_slam/trajectories/<run_id>/<stack>_trajectory.tum
# --------------------------------------------------------------------------- #

def infer_run_id(tum_path):
    p = Path(tum_path).resolve()
    if p.parent.parent.name == "trajectories":
        return p.parent.name
    return None


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #

def parse_args(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tum", required=True,
                    help="path to the recorded ground-truth TUM trajectory "
                         "(logs/sidewalk_slam/trajectories/<run_id>/"
                         "ground_truth_trajectory.tum)")
    ap.add_argument("--out", required=True,
                    help="output route YAML path")
    ap.add_argument("--tolerance", type=float, default=DEFAULT_TOLERANCE_M,
                    help=f"Douglas-Peucker position-simplification tolerance "
                         f"in metres (default: {DEFAULT_TOLERANCE_M})")
    ap.add_argument("--min-turn-deg", type=float, default=DEFAULT_MIN_TURN_DEG,
                    help="heading changes below this many degrees are "
                         f"dropped as noise (default: {DEFAULT_MIN_TURN_DEG})")
    ap.add_argument("--min-forward-m", type=float, default=DEFAULT_MIN_FORWARD_M,
                    help="straight segments shorter than this many metres "
                         "are merged into their neighbours (default: "
                         f"{DEFAULT_MIN_FORWARD_M})")
    ap.add_argument("--label", default="",
                    help="free-text description written into the output "
                         "file's header (e.g. which world/session this "
                         "came from)")
    args = ap.parse_args(argv)
    if args.tolerance <= 0:
        ap.error("--tolerance must be > 0")
    if args.min_turn_deg < 0:
        ap.error("--min-turn-deg must be >= 0")
    if args.min_forward_m < 0:
        ap.error("--min-forward-m must be >= 0")
    return args


def main(argv=None):
    _selftest_quat_to_yaw()
    args = parse_args(argv)

    samples, skipped = parse_tum(args.tum)
    meta = load_meta_sidecar(args.tum)
    run_id = infer_run_id(args.tum)

    points = [(s.x, s.y) for s in samples]
    rdp_idx = douglas_peucker_indices(points, args.tolerance)
    pivots = detect_pivots(samples)
    waypoints = build_waypoints(samples, rdp_idx, pivots)
    waypoints = collapse_short_segments(waypoints, args.min_forward_m)

    primitives, _ = emit_primitives(waypoints, samples[0].yaw,
                                    args.min_turn_deg, args.min_forward_m)
    primitives = compact_primitives(primitives)

    if not primitives:
        print("error: no primitives survived simplification — the "
              "recording may be too short, too static, or --tolerance / "
              "--min-forward-m too loose for its length. Nothing written.",
              file=sys.stderr)
        sys.exit(1)

    sim_x, sim_y, sim_yaw = simulate_primitives(
        primitives, samples[0].x, samples[0].y, samples[0].yaw)
    actual = samples[-1]
    pos_disc = math.hypot(sim_x - actual.x, sim_y - actual.y)
    heading_disc_deg = abs(math.degrees(ang_diff(sim_yaw, actual.yaw)))
    self_check = (pos_disc, heading_disc_deg)

    total_forward_m = round(sum(p["value"] for p in primitives
                               if p["type"] == "forward"), 2)

    tolerance_note = ("within the 10-20 cm band this tool defaults to — "
                      "see DEFAULT_TOLERANCE_M's own comment in "
                      "route_from_drive.py for the full justification"
                      if abs(args.tolerance - DEFAULT_TOLERANCE_M) < 1e-9
                      else "a value explicitly overridden on the command "
                      "line, not this tool's own default")

    header = build_header(args, samples, skipped, meta, primitives,
                          total_forward_m, self_check, run_id, tolerance_note)
    text = render_route_yaml(primitives, header,
                             math.degrees(samples[0].yaw))

    # Validate the file this tool is about to write actually parses as the
    # list-of-primitive-mappings route_player.py expects, before writing
    # it — a route file that fails to load is worse than one this tool
    # simply refused to produce.
    parsed_back = yaml.safe_load(text)
    if not isinstance(parsed_back, list) or not parsed_back:
        print("error: internal error — the generated route YAML did not "
              "parse back as a non-empty list. Refusing to write it.",
              file=sys.stderr)
        sys.exit(1)
    for entry in parsed_back:
        if not isinstance(entry, dict) or not ({"forward"} & entry.keys()
                                               or {"turn"} & entry.keys()):
            print(f"error: internal error — a generated primitive does not "
                 f"parse as {{forward: ...}} or {{turn: ...}}: {entry!r}. "
                 f"Refusing to write it.", file=sys.stderr)
            sys.exit(1)

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(text + "\n")

    print(f"route_from_drive: {len(primitives)} primitive(s), "
         f"{total_forward_m:.2f} m forward total")
    print(f"route_from_drive: self-check discrepancy: {pos_disc:.3f} m, "
         f"{heading_disc_deg:.1f} deg")
    if pos_disc > SELF_CHECK_MAX_POS_M or heading_disc_deg > SELF_CHECK_MAX_HEADING_DEG:
        print(f"route_from_drive: *** WARNING *** self-check discrepancy "
             f"exceeds {SELF_CHECK_MAX_POS_M:.1f} m / "
             f"{SELF_CHECK_MAX_HEADING_DEG:.0f} deg — inspect the route "
             f"before trusting it. See the file's own header.",
             file=sys.stderr)
    if skipped:
        print(f"route_from_drive: {skipped} malformed TUM line(s) were "
             f"skipped during parsing.")
    print(f"route_from_drive: wrote {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
