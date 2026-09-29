#!/usr/bin/env python3
"""
nav_run.py — the reading half of the navigation -> evaluation data path.

WHY THIS EXISTS
    `sidewalk_navigation`'s waypoint runner drives a fixed route and writes
    what happened. This module reads it. Nothing else in `sidewalk_evaluation`
    knows the layout of a navigation run, so when the schema changes there is
    exactly one file on this side to change.

    It contains NO metrics. Every number this package reports about a
    trajectory comes from `traj_metrics.py`, which already implements ATE, RPE,
    drift, loop closure and correction detection against the TUM benchmark
    definitions. Reimplementing any of that here would produce a second set of
    numbers that agree with the first until one day they do not.

    What this module does is narrower and duller: locate the files, check that
    what was written is what this reader expects, and refuse — loudly and
    specifically — when it is not.

WHAT IT REFUSES TO DO
    It will not hand a navigation trajectory to the ATE machinery as though it
    were ground truth. A navigation run's trajectory is the robot's own
    estimate, produced by the localisation system that was steering it, so
    comparing it against itself proves nothing. `check_comparable()` exists to
    catch the mistakes that look plausible: two runs on different routes, two
    runs on different maps, or a run compared against itself.

NOT A ROS NODE, AND NO NUMPY
    Pure standard library, so it imports and self-tests on a machine with
    neither ROS nor numpy installed:

        python3 nav_run.py --self-test

# ===========================================================================
# NAVIGATION RUN-SUMMARY SCHEMA — version 1
#
# THIS BLOCK IS DUPLICATED VERBATIM IN TWO FILES AND MUST STAY IDENTICAL:
#     catkin_ws/src/sidewalk_navigation/scripts/nav_run_io.py   (writer)
#     catkin_ws/src/sidewalk_evaluation/scripts/nav_run.py      (reader)
# If you change one, change the other in the same commit. The reader checks
# `schema_version` and refuses anything it does not recognise, so drift between
# the two ends is a loud failure rather than a silent wrong number.
#
# A navigation run writes ONE directory:
#
#   logs/sidewalk_navigation/runs/<run_id>/
#       run_summary.json          <- the document described below
#       trajectory.tum            <- TUM, written by sidewalk_slam's TumWriter
#       trajectory.tum.meta.json  <- TumWriter's own sidecar
#
# COORDINATE FRAME AND UNITS  (applies to every pose and every length below)
#   * Poses are the pose of `base_link` expressed in the `map` frame, i.e. the
#     transform map -> base_link, sampled from TF.
#   * `map` is the frame the localisation system publishes into: x and y in the
#     ground plane, z up, right-handed. Its origin is wherever mapping began,
#     which is arbitrary but fixed for a given map database.
#   * Translations are METRES. Rotations are unit quaternions in
#     (qx, qy, qz, qw) order — SCALAR LAST, the ROS and TUM convention, NOT the
#     scalar-first order used by Eigen's constructor or by MATLAB. Yaw fields,
#     where they appear, are RADIANS.
#   * Timestamps are SECONDS since the UNIX epoch on the ROS clock. Under
#     `use_sim_time` they are recorded (bag) time, not wall time.
#
# trajectory.tum
#   One pose per line: `timestamp tx ty tz qx qy qz qw`. Byte-for-byte the same
#   format `sidewalk_slam` writes, produced by the same `TumWriter` class, so
#   `sidewalk_evaluation`'s `traj_metrics.load_tum()` reads it with no special
#   case anywhere.
#
# run_summary.json — top-level keys
#   schema                str   always "sidewalk_navigation.run_summary"
#   schema_version        int   always 1
#   run_id                str   sortable, "<YYYYmmdd-HHMMSS>_<run_name>"
#   run_name              str
#   package               str   "sidewalk_navigation"
#   node                  str   "waypoint_runner"
#   status                str   "completed" | "aborted" | "failed_start"
#   completed             bool  true only when status == "completed"
#   abort_reason          str|null   why it stopped early; null if it did not
#   started_unix_s        float ROS clock, seconds since the UNIX epoch
#   ended_unix_s          float ROS clock, seconds since the UNIX epoch
#   started_iso           str   local wall clock, human reference only
#   ended_iso             str   local wall clock, human reference only
#   duration_s            float ended_unix_s - started_unix_s
#   provenance            obj   see below
#   trajectory            obj|null   null means no trajectory was recorded at
#                                    all. A non-null object together with a
#                                    non-null trajectory_unavailable_reason
#                                    means recording started and then stopped
#                                    part way; n_poses says how much survived.
#   trajectory_unavailable_reason  str|null  why the trajectory is absent or
#                                    incomplete; null when it is neither
#   metrics               obj   {name: {value, unit, description}}
#   legs                  list  one entry per waypoint attempt, in order
#   leg_field_units       obj   {leg field name: unit string}
#   written_at            str   ISO local time this file was written
#
# provenance
#   map.database_path     str|null  the map database driven against
#   map.param             str       the ROS parameter it was read from
#   map.exists            bool
#   map.size_bytes        int|null
#   map.sha1_16           str|null  first 16 hex chars of the file's SHA-1.
#                                   Two runs on the same map share it; a map
#                                   rebuilt overnight does not.
#   waypoints.file        str|null  path of the loaded waypoint YAML, if known
#   waypoints.param       str       parameter namespace the list came from
#   waypoints.count       int
#   waypoints.names       list[str]
#   waypoints.frame       str       frame the waypoint x/y/yaw are given in
#   waypoints.sha1_16     str       hash of the NORMALISED waypoint list. Two
#                                   runs of the same route share it; this is
#                                   the field the evaluation side uses to
#                                   refuse to compare two different routes.
#   planner.local         str|null  local planner in use
#   planner.global        str|null  global planner in use
#   planner.move_base_ns  str
#   frames.map_frame      str
#   frames.base_frame     str
#   localization.pose_source  str   how the trajectory poses were obtained
#   runner.loops                int
#   runner.reverse_on_loop      bool
#   runner.continue_on_failure  bool
#   runner.goal_timeout_s       float
#   runner.settle_time_s        float
#   runner.sample_rate_hz       float
#   ros.master_uri        str
#   ros.use_sim_time      bool
#   ros.distro            str
#
# trajectory
#   file                  str   filename, relative to this directory
#   meta_file             str   TumWriter's sidecar, relative to this directory
#   format                str   "TUM"
#   columns               str   "timestamp tx ty tz qx qy qz qw"
#   quaternion_order      str   "xyzw_scalar_last"
#   length_unit           str   "m"
#   time_unit             str   "s_unix_ros_clock"
#   frame_id              str   world frame the poses are expressed in
#   child_frame_id        str   body frame whose pose is recorded
#   pose_source           str
#   sample_rate_hz        float requested sampling rate
#   n_poses               int   poses actually written
#   n_rejected            int   poses TumWriter refused (non-finite, degenerate)
#   n_duplicate_stamps_skipped  int  TF returned the same stamp twice
#   first_stamp_unix_s    float|null
#   last_stamp_unix_s     float|null
#   is_ground_truth       bool  ALWAYS false. See the caveat below.
#   reference_kind        str   the value the evaluation side must use when
#                               this file is passed as a reference
#   caveat                str   plain-language restatement of the caveat below
#
# metrics — every entry carries its own unit; nothing is unit-less by accident
#   legs_attempted        count
#   legs_succeeded        count
#   success_rate_pct      %
#   total_distance_m      m      summed odometry-integrated path length
#   total_time_s          s      summed per-leg durations
#   mean_speed_mps        m/s    total_distance_m / total_time_s
#   mean_final_error_m    m      mean over legs that reported one
#   max_final_error_m     m
#   mean_path_efficiency  ratio  straight_line / path_length, 0..1
#
# legs[] — leg_field_units documents the unit of every numeric field
#   index                 int    0-based position in the executed order
#   leg                   str    label, e.g. "loop1.wp3"
#   name                  str    waypoint name from the route file
#   state                 str    SUCCEEDED | ABORTED | REJECTED | PREEMPTED |
#                                LOST | CANCELLED | TIMEOUT | STATE_<n>
#   ok                    bool   state == SUCCEEDED
#   duration_s            s
#   path_length_m         m      integrated from fused odometry, not from the
#                                map pose, so loop-closure jumps are not
#                                counted as distance travelled
#   straight_line_m       m
#   path_efficiency       ratio
#   final_error_m         m
#   goal_x_m, goal_y_m    m      requested goal, map frame
#   goal_yaw_rad          rad
#   end_x_m, end_y_m      m      measured arrival pose, map frame
#   end_yaw_rad           rad
#   start_unix_s          s
#   end_unix_s            s
#
# WHAT THIS TRAJECTORY IS, AND WHAT IT IS NOT
#   It is the robot's OWN ESTIMATE of where it went, produced by the very
#   localisation system that was steering it. It is not ground truth, and it
#   carries no independent information about where the robot physically was.
#   If the localisation drifted two metres, the trajectory drifted with it and
#   agrees with itself perfectly the whole way.
#
#   Legitimate uses:
#     * REPEATABILITY — drive the identical route N times and measure how much
#       the estimate differs from run to run. The spread is real information
#       about the system's consistency.
#     * PLANNER COMPARISON — same map, same waypoints, same localisation, two
#       different local planners. Path length, path efficiency and duration are
#       then attributable to the planner.
#     * SELF-CONSISTENCY — closed-loop gap, and the size and count of
#       pose-graph corrections, which are visible only in a LIVE pose stream
#       and have already been absorbed by any post-hoc trajectory export.
#
#   NOT legitimate:
#     * ACCURACY. Comparing this trajectory with itself, or with the pose
#       stream of the same localisation system, measures nothing at all. An
#       accuracy claim needs a reference that does not share the estimator:
#       wheel odometry over short ranges, a surveyed endpoint, RTK GNSS or
#       motion capture. `sidewalk_evaluation` records which one was used in
#       `reference_kind` and will not compute ATE without one.
# ===========================================================================
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

# --------------------------------------------------------------------------- #
# The reader's own copy of the schema's vocabulary.
#
# Deliberately a COPY rather than an import from sidewalk_navigation. Importing
# the writer's constants would make the two ends agree by construction, which
# sounds attractive and is exactly wrong: the check below would then be a
# tautology and a real drift between the packages would pass silently. Here the
# reader states independently what it expects, and says so by name when the
# writer sends something else.
# --------------------------------------------------------------------------- #

SCHEMA = "sidewalk_navigation.run_summary"
SUPPORTED_SCHEMA_VERSIONS = (1,)

SUMMARY_FILENAME = "run_summary.json"

STATUSES = ("completed", "aborted", "failed_start")

EXPECTED_METRIC_UNITS = {
    "legs_attempted": "count",
    "legs_succeeded": "count",
    "success_rate_pct": "%",
    "total_distance_m": "m",
    "total_time_s": "s",
    "mean_speed_mps": "m/s",
    "mean_final_error_m": "m",
    "max_final_error_m": "m",
    "mean_path_efficiency": "ratio",
}

EXPECTED_LEG_FIELD_UNITS = {
    "index": "count",
    "duration_s": "s",
    "path_length_m": "m",
    "straight_line_m": "m",
    "path_efficiency": "ratio",
    "final_error_m": "m",
    "goal_x_m": "m",
    "goal_y_m": "m",
    "goal_yaw_rad": "rad",
    "end_x_m": "m",
    "end_y_m": "m",
    "end_yaw_rad": "rad",
    "start_unix_s": "s",
    "end_unix_s": "s",
}

# The value that MUST appear in `reference_kind` whenever one navigation run is
# evaluated against another. Kept distinct from "cross_stack" because that means
# a different SLAM system on the same data; this means the same system on a
# different drive, which is a different question with a different answer.
REFERENCE_KIND = "nav_repeat_run"

REFERENCE_KIND_MEANING = (
    "nav_repeat_run: the reference is another navigation run of the same route, "
    "produced by the same localisation system. The resulting ATE and RPE "
    "measure REPEATABILITY — how consistently the system reproduces its own "
    "answer — and are NOT an accuracy measurement. Two runs can agree to a "
    "centimetre and both be two metres from where the robot physically was."
)


class NavRunError(RuntimeError):
    """A navigation run that cannot be read, with a sentence saying why.

    Carries a message written for the person who has to fix it. Callers print
    `str(exc)` and exit non-zero; they do not add commentary, because the
    message is already the diagnosis.
    """


class NavRun(object):
    """One navigation run, read from disk and checked against this schema.

    Attributes
    ----------
    dir            directory the run lives in
    summary_path   path of run_summary.json
    doc            the parsed document, exactly as written
    trajectory     Path to the TUM file, or None if the run recorded none
    issues         list of human-readable complaints. Non-fatal: the run is
                   usable, but something about it deserves saying out loud.
    """

    def __init__(self, directory, doc, summary_path, trajectory, issues):
        self.dir = Path(directory)
        self.doc = doc
        self.summary_path = Path(summary_path)
        self.trajectory = Path(trajectory) if trajectory else None
        self.issues = list(issues)

    # ------------------------------------------------------------- identity
    @property
    def run_id(self) -> str:
        return str(self.doc.get("run_id") or self.dir.name)

    @property
    def run_name(self) -> str:
        return str(self.doc.get("run_name") or self.run_id)

    @property
    def label(self) -> str:
        """The name used for this run's result files and plot legends.

        It is the full run_id and nothing shorter. A prettier abbreviation was
        tried and rejected: result files outlive the directory they came from,
        and a file called `eval_waypoint_demo.json` that could have come from
        any of five runs is worse than a long filename. The run_id already
        begins with a sortable timestamp, so it is unique by construction.
        """
        return self.run_id

    @property
    def status(self) -> str:
        return str(self.doc.get("status") or "unknown")

    @property
    def completed(self) -> bool:
        return bool(self.doc.get("completed"))

    @property
    def abort_reason(self):
        return self.doc.get("abort_reason")

    # ----------------------------------------------------------- provenance
    def prov(self, *keys, default=None):
        node = self.doc.get("provenance") or {}
        for key in keys:
            if not isinstance(node, dict) or key not in node:
                return default
            node = node[key]
        return node

    @property
    def route_signature(self):
        return self.prov("waypoints", "sha1_16")

    @property
    def map_signature(self):
        return self.prov("map", "sha1_16")

    @property
    def map_path(self):
        return self.prov("map", "database_path")

    @property
    def local_planner(self):
        return self.prov("planner", "local")

    @property
    def waypoint_names(self):
        names = self.prov("waypoints", "names", default=[])
        return [str(n) for n in names] if isinstance(names, list) else []

    # --------------------------------------------------------------- values
    def metric(self, name, default=None):
        """The numeric value of one run-level metric, or `default`."""
        entry = (self.doc.get("metrics") or {}).get(name)
        if isinstance(entry, dict):
            return entry.get("value", default)
        return default

    def metric_unit(self, name, default=""):
        entry = (self.doc.get("metrics") or {}).get(name)
        return entry.get("unit", default) if isinstance(entry, dict) else default

    @property
    def legs(self):
        legs = self.doc.get("legs")
        return [leg for leg in legs if isinstance(leg, dict)] \
            if isinstance(legs, list) else []

    def arrival_poses(self):
        """{leg label: (x, y, yaw)} for every leg that recorded where it ended.

        This is the association-free half of a repeatability comparison. The
        leg label is stable across runs of the same route — `loop2.wp3` is the
        same physical waypoint on the same lap every time — so the arrival
        poses of two runs can be compared directly, with no need to match the
        two trajectories in time at all. That matters, because two runs happen
        at different wall-clock times and at different speeds, and time-based
        association between them is the weakest link in the whole comparison.
        """
        out = {}
        for leg in self.legs:
            if not all(k in leg for k in ("end_x_m", "end_y_m", "end_yaw_rad")):
                continue
            key = str(leg.get("leg") or "leg%s" % leg.get("index"))
            try:
                out[key] = (float(leg["end_x_m"]), float(leg["end_y_m"]),
                            float(leg["end_yaw_rad"]))
            except (TypeError, ValueError):
                continue
        return out

    @property
    def n_poses(self) -> int:
        traj = self.doc.get("trajectory") or {}
        try:
            return int(traj.get("n_poses") or 0)
        except (TypeError, ValueError):
            return 0

    @property
    def started_unix_s(self):
        return _as_float(self.doc.get("started_unix_s"))

    @property
    def duration_s(self):
        return _as_float(self.doc.get("duration_s"))

    # --------------------------------------------------------------- report
    def describe(self):
        """Lines describing this run, fit to print straight into a run log."""
        lines = [
            "run_id      : %s" % self.run_id,
            "status      : %s%s" % (
                self.status,
                "" if not self.abort_reason else "  (%s)" % self.abort_reason),
            "route       : %d waypoint(s) [%s], signature %s" % (
                len(self.waypoint_names),
                ", ".join(self.waypoint_names[:6])
                + (", ..." if len(self.waypoint_names) > 6 else ""),
                self.route_signature),
            "map         : %s (signature %s)" % (
                self.map_path or "not recorded", self.map_signature or "none"),
            "planner     : %s" % (self.local_planner or "not recorded"),
        ]
        if self.trajectory is not None:
            lines.append("trajectory  : %s (%d poses)"
                         % (self.trajectory.name, self.n_poses))
        else:
            lines.append("trajectory  : NONE — %s"
                         % (self.doc.get("trajectory_unavailable_reason")
                            or "no reason recorded"))
        for name in ("total_distance_m", "total_time_s", "mean_speed_mps",
                     "mean_path_efficiency", "mean_final_error_m"):
            value = self.metric(name)
            if value is not None:
                lines.append("%-12s: %s %s"
                             % (name, value, self.metric_unit(name)))
        return lines


def _as_float(value):
    try:
        f = float(value)
        return f if math.isfinite(f) else None
    except (TypeError, ValueError):
        return None


# --------------------------------------------------------------------------- #
# Loading
# --------------------------------------------------------------------------- #

def load_run(path) -> NavRun:
    """Read one navigation run. Raises NavRunError with a specific message.

    `path` may be the run directory or the run_summary.json inside it, because
    both are things a person will reasonably type.
    """
    p = Path(path).expanduser()

    if p.is_dir():
        summary_path = p / SUMMARY_FILENAME
        directory = p
    elif p.name == SUMMARY_FILENAME or p.suffix == ".json":
        summary_path = p
        directory = p.parent
    else:
        raise NavRunError(
            "%s is neither a navigation run directory nor a %s file.\n"
            "  A navigation run directory is written by "
            "sidewalk_navigation/waypoint_runner.py and normally lives in\n"
            "      logs/sidewalk_navigation/runs/<run_id>/"
            % (p, SUMMARY_FILENAME))

    if not summary_path.exists():
        raise NavRunError(
            "No %s in %s.\n"
            "  Either this is not a navigation run directory, or the run "
            "predates the run-summary schema, or waypoint_runner.py could not "
            "write it — in which case the reason was reported as an ERROR in "
            "that run's log under logs/sidewalk_navigation/."
            % (SUMMARY_FILENAME, directory))

    try:
        doc = json.loads(summary_path.read_text())
    except OSError as exc:
        raise NavRunError("Cannot read %s: %s" % (summary_path, exc))
    except json.JSONDecodeError as exc:
        raise NavRunError(
            "%s is not valid JSON (%s).\n"
            "  A truncated file usually means the run was killed while "
            "writing. The per-leg numbers are still in that run's text log."
            % (summary_path, exc))

    if not isinstance(doc, dict):
        raise NavRunError("%s does not contain a JSON object." % summary_path)

    schema = doc.get("schema")
    if schema != SCHEMA:
        raise NavRunError(
            "%s declares schema %r, but this reader understands only %r.\n"
            "  If this file came from somewhere other than "
            "sidewalk_navigation/waypoint_runner.py, it is not a navigation "
            "run and must not be read as one."
            % (summary_path, schema, SCHEMA))

    version = doc.get("schema_version")
    if version not in SUPPORTED_SCHEMA_VERSIONS:
        raise NavRunError(
            "%s is schema version %r; this reader supports %s.\n"
            "  The schema is documented in a comment block duplicated in two "
            "files, which must be changed together:\n"
            "      catkin_ws/src/sidewalk_navigation/scripts/nav_run_io.py\n"
            "      catkin_ws/src/sidewalk_evaluation/scripts/nav_run.py\n"
            "  A version this reader does not know is a genuine incompatibility, "
            "not something to work around: reading it would produce numbers "
            "whose meaning nobody can state."
            % (summary_path, version,
               ", ".join(str(v) for v in SUPPORTED_SCHEMA_VERSIONS)))

    issues = _check_document(doc, directory)

    trajectory = None
    traj = doc.get("trajectory")
    if isinstance(traj, dict) and traj.get("file"):
        candidate = directory / str(traj["file"])
        if candidate.exists():
            trajectory = candidate
        else:
            issues.append(
                "run_summary.json names the trajectory %s, but that file is "
                "not in %s. Only the metrics can be used from this run."
                % (traj["file"], directory))

    return NavRun(directory, doc, summary_path, trajectory, issues)


def _check_document(doc, directory):
    """Cross-check a summary against this reader's expectations.

    Returns a list of complaints. None of them are fatal — a run with an
    unfamiliar metric is still a run, and discarding it would lose real data —
    but every one of them is printed, because a silent mismatch between the two
    ends of this data path is the exact failure this schema exists to prevent.
    """
    issues = []

    status = doc.get("status")
    if status not in STATUSES:
        issues.append("status is %r, which is not one of %s."
                      % (status, ", ".join(STATUSES)))
    elif status == "aborted":
        issues.append(
            "This run was ABORTED (%s). Its trajectory stops early, so its "
            "path length and duration are not comparable with a completed run "
            "of the same route. It remains valid for the portion it covers."
            % (doc.get("abort_reason") or "no reason recorded"))
    elif status == "failed_start":
        issues.append("This run never drove: %s"
                      % (doc.get("abort_reason") or "no reason recorded"))

    metrics = doc.get("metrics") or {}
    for name, entry in metrics.items():
        if not isinstance(entry, dict) or "value" not in entry:
            issues.append("metric %r is not a {value, unit, description} "
                          "object." % name)
            continue
        expected = EXPECTED_METRIC_UNITS.get(name)
        unit = entry.get("unit")
        if expected is None:
            issues.append(
                "metric %r is not in this reader's table of known metrics, so "
                "its unit (%r) is taken on trust. Add it to "
                "EXPECTED_METRIC_UNITS in nav_run.py, and to METRIC_UNITS in "
                "nav_run_io.py, in the same commit." % (name, unit))
        elif unit != expected:
            issues.append(
                "metric %r arrived with unit %r but this reader expects %r. "
                "The two ends of the schema have drifted apart; do not use "
                "this number until they agree." % (name, unit, expected))

    units = doc.get("leg_field_units")
    if not isinstance(units, dict):
        issues.append("leg_field_units is missing, so leg field units are "
                      "being assumed rather than read.")
    else:
        for field, expected in EXPECTED_LEG_FIELD_UNITS.items():
            if field in units and units[field] != expected:
                issues.append(
                    "leg field %r is declared in unit %r but this reader "
                    "expects %r." % (field, units[field], expected))

    prov = doc.get("provenance") or {}
    if not prov.get("waypoints", {}).get("sha1_16"):
        issues.append(
            "No route signature in the provenance, so this run cannot be "
            "checked against another for being the same route. Any comparison "
            "involving it rests on the operator's memory.")
    map_info = prov.get("map") or {}
    if not map_info.get("exists"):
        issues.append(
            "The map database %s was not present when the run started. The "
            "run may still be valid, but 'which map was this?' now has no "
            "verifiable answer." % (map_info.get("database_path") or "(unset)"))

    traj = doc.get("trajectory")
    if traj is None:
        issues.append(
            "This run recorded no trajectory (%s). Only the per-leg metrics "
            "are available; no ATE, RPE, drift or loop-closure figure can be "
            "computed from it."
            % (doc.get("trajectory_unavailable_reason") or "no reason recorded"))
    elif isinstance(traj, dict):
        if traj.get("is_ground_truth"):
            issues.append(
                "The trajectory claims is_ground_truth=true. A navigation run "
                "is the robot's own estimate and can never be ground truth. "
                "Do not use this file as a reference for an accuracy claim.")
        if traj.get("quaternion_order") not in (None, "xyzw_scalar_last"):
            issues.append(
                "The trajectory declares quaternion order %r. Everything in "
                "this project is scalar-last (x, y, z, w); reading a "
                "scalar-first file as scalar-last produces a rotation error "
                "that looks like a plausible trajectory."
                % traj.get("quaternion_order"))
        skipped = traj.get("n_duplicate_stamps_skipped") or 0
        written = traj.get("n_poses") or 0
        if written and skipped > written:
            issues.append(
                "%d samples were skipped as duplicate timestamps against %d "
                "written. The trajectory was polled considerably faster than "
                "localisation publishes; lower trajectory_rate in "
                "waypoint_demo.launch so the file reflects the sampling that "
                "actually happened." % (skipped, written))
        if written and written < 10:
            issues.append(
                "Only %d pose(s) were recorded. Nothing meaningful can be "
                "measured from that; localisation was almost certainly "
                "unavailable for most of the run." % written)
    return issues


def load_runs(paths):
    """Load several runs. Returns (runs, errors); neither raises.

    Errors are returned rather than raised so that one unreadable run does not
    discard the others — with five repeats of a route, four are still a result.
    """
    runs, errors = [], []
    for p in paths or []:
        try:
            runs.append(load_run(p))
        except NavRunError as exc:
            errors.append(str(exc))
    return runs, errors


# --------------------------------------------------------------------------- #
# Comparability
# --------------------------------------------------------------------------- #

def check_comparable(runs):
    """Say what is wrong with treating these runs as repeats of one experiment.

    Returns (fatal, caveats). `fatal` entries mean the comparison is invalid
    and the caller must stop; `caveats` mean it is valid but qualified and the
    qualification must appear in the report.

    This function is the guard rail around the mistake this whole data path
    makes easy: it is now trivial to point the ATE machinery at two navigation
    runs and get a small, confident, meaningless number.
    """
    fatal, caveats = [], []

    if len(runs) < 2:
        return fatal, caveats

    resolved = {}
    for run in runs:
        try:
            key = run.summary_path.resolve()
        except OSError:
            key = run.summary_path
        if key in resolved:
            fatal.append(
                "The run %s was given twice. Comparing a trajectory with "
                "itself yields zero error and proves nothing at all."
                % run.run_id)
        resolved[key] = run

    sigs = {r.route_signature for r in runs}
    if len(sigs) > 1:
        detail = "; ".join("%s -> %s" % (r.run_id, r.route_signature)
                           for r in runs)
        if None in sigs:
            caveats.append(
                "At least one run carries no route signature, so it cannot be "
                "confirmed that these runs drove the same route (%s). If they "
                "did not, the comparison below is meaningless." % detail)
        else:
            fatal.append(
                "These runs did not drive the same route: %s.\n"
                "  Repeatability is defined as the spread of repeated attempts "
                "at ONE task. Comparing different routes measures the "
                "difference between the routes." % detail)

    maps = {r.map_signature for r in runs if r.map_signature}
    if len(maps) > 1:
        caveats.append(
            "These runs used different map databases (%s). The map defines the "
            "coordinate frame the trajectories are expressed in, so their "
            "origins are not the same point on the ground. Alignment will "
            "absorb the offset, but any residual is a difference between maps "
            "as much as between runs."
            % ", ".join(sorted(maps)))

    planners = {str(r.local_planner) for r in runs if r.local_planner}
    if len(planners) > 1:
        caveats.append(
            "These runs used different local planners (%s). That is a valid "
            "experiment — it is the planner comparison this data path is for — "
            "but it is NOT a repeatability measurement, and the spread below "
            "must not be reported as one." % ", ".join(sorted(planners)))

    aborted = [r.run_id for r in runs if r.status != "completed"]
    if aborted:
        caveats.append(
            "Not every run completed (%s). A run that stopped early covers "
            "less of the route, so metrics summed over the whole route are not "
            "comparable between them." % ", ".join(aborted))

    empty = [r.run_id for r in runs if r.trajectory is None]
    if empty:
        caveats.append(
            "These runs recorded no trajectory and contribute only their "
            "per-leg metrics: %s" % ", ".join(empty))

    return fatal, caveats


# --------------------------------------------------------------------------- #
# Self-test — pure standard library, no numpy, no ROS, no hardware
# --------------------------------------------------------------------------- #

def synthetic_doc(run_id="20260801-101500_a", route_sig="route0000000000",
                   map_sig="map00000000000000", status="completed",
                   planner="dwa_local_planner/DWAPlannerROS", offset=0.0):
    """A minimal but schema-complete document, for testing the reader alone."""
    return {
        "schema": SCHEMA,
        "schema_version": 1,
        "run_id": run_id,
        "run_name": "selftest",
        "package": "sidewalk_navigation",
        "node": "waypoint_runner",
        "status": status,
        "completed": status == "completed",
        "abort_reason": None if status == "completed" else "synthetic",
        "started_unix_s": 1700000000.0,
        "ended_unix_s": 1700000120.0,
        "started_iso": None,
        "ended_iso": None,
        "duration_s": 120.0,
        "provenance": {
            "map": {"database_path": "/tmp/map.db", "param": "test",
                    "exists": True, "size_bytes": 1, "sha1_16": map_sig},
            "waypoints": {"file": None, "param": "/waypoints", "count": 2,
                          "names": ["start_dock", "return_dock"],
                          "frame": "map", "sha1_16": route_sig},
            "planner": {"local": planner, "global": "global_planner/GlobalPlanner",
                        "move_base_ns": "/move_base"},
            "frames": {"map_frame": "map", "base_frame": "base_link"},
            "localization": {"pose_source": "synthetic"},
            "runner": {"loops": 1, "reverse_on_loop": False,
                       "continue_on_failure": False, "goal_timeout_s": 180.0,
                       "settle_time_s": 0.5, "sample_rate_hz": 10.0},
            "ros": {"master_uri": "", "use_sim_time": False, "distro": ""},
        },
        "trajectory": None,
        "trajectory_unavailable_reason": "synthetic document, no file",
        "metrics": {
            "legs_attempted": {"value": 2, "unit": "count", "description": ""},
            "legs_succeeded": {"value": 2, "unit": "count", "description": ""},
            "total_distance_m": {"value": 41.2, "unit": "m", "description": ""},
            "total_time_s": {"value": 120.0, "unit": "s", "description": ""},
        },
        "legs": [
            {"index": 0, "leg": "loop1.wp1", "name": "start_dock",
             "state": "SUCCEEDED", "ok": True, "duration_s": 60.0,
             "path_length_m": 20.6, "end_x_m": 0.10 + offset, "end_y_m": -0.05,
             "end_yaw_rad": 0.01},
            {"index": 1, "leg": "loop1.wp2", "name": "return_dock",
             "state": "SUCCEEDED", "ok": True, "duration_s": 60.0,
             "path_length_m": 20.6, "end_x_m": 0.02, "end_y_m": 0.03 + offset,
             "end_yaw_rad": -0.02},
        ],
        "leg_field_units": dict(EXPECTED_LEG_FIELD_UNITS),
        "written_at": "2026-08-01T10:17:00",
    }


def _self_test() -> int:
    import tempfile

    checks = []

    def check(desc, ok, detail=""):
        checks.append((desc, bool(ok), detail))

    def write(directory, doc):
        directory.mkdir(parents=True, exist_ok=True)
        (directory / SUMMARY_FILENAME).write_text(json.dumps(doc, indent=2))
        return directory

    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)

        # 1. A well-formed run reads cleanly.
        good = write(root / "good", synthetic_doc())
        run = load_run(good)
        check("well-formed run loads", run.run_id == "20260801-101500_a",
              run.run_id)
        check("metrics readable with their units",
              run.metric("total_distance_m") == 41.2
              and run.metric_unit("total_distance_m") == "m")
        check("arrival poses extracted", len(run.arrival_poses()) == 2,
              str(sorted(run.arrival_poses())))
        check("route signature exposed",
              run.route_signature == "route0000000000")
        check("describe() produces lines", len(run.describe()) >= 6)

        # 2. Loading via the JSON path directly must work too.
        run2 = load_run(good / SUMMARY_FILENAME)
        check("loads when given the JSON path", run2.run_id == run.run_id)

        # 3. A wrong schema version must be refused, not coerced.
        bad_ver = synthetic_doc()
        bad_ver["schema_version"] = 99
        d = write(root / "badver", bad_ver)
        try:
            load_run(d)
            check("unknown schema version refused", False, "no exception")
        except NavRunError as exc:
            check("unknown schema version refused", "99" in str(exc))

        # 4. A foreign document must be refused.
        d = write(root / "foreign", {"schema": "something_else",
                                     "schema_version": 1})
        try:
            load_run(d)
            check("foreign schema refused", False, "no exception")
        except NavRunError:
            check("foreign schema refused", True)

        # 5. A missing summary must name the file it wanted.
        (root / "empty").mkdir()
        try:
            load_run(root / "empty")
            check("missing summary refused", False, "no exception")
        except NavRunError as exc:
            check("missing summary refused", SUMMARY_FILENAME in str(exc))

        # 6. A drifted unit must be reported by name, not silently accepted.
        drifted = synthetic_doc()
        drifted["metrics"]["total_distance_m"]["unit"] = "ft"
        d = write(root / "drift", drifted)
        run3 = load_run(d)
        check("unit drift reported by name",
              any("total_distance_m" in i and "ft" in i for i in run3.issues),
              "; ".join(run3.issues)[:90])

        # 7. Comparability: same route passes, different routes are fatal.
        a = load_run(write(root / "a", synthetic_doc(run_id="20260801-1_a")))
        b = load_run(write(root / "b", synthetic_doc(run_id="20260801-1_b",
                                                      offset=0.04)))
        fatal, caveats = check_comparable([a, b])
        check("two runs of the same route are comparable", not fatal,
              "; ".join(fatal)[:90])

        c = load_run(write(root / "c", synthetic_doc(run_id="20260801-1_c",
                                                      route_sig="OTHERROUTE00")))
        fatal, _ = check_comparable([a, c])
        check("different routes rejected as not comparable", bool(fatal),
              "; ".join(fatal)[:90])

        # 8. The same run twice must be caught. This is THE mistake this data
        #    path makes easy, and it yields a beautiful, meaningless zero.
        fatal, _ = check_comparable([a, load_run(root / "a")])
        check("the same run given twice is rejected", bool(fatal),
              "; ".join(fatal)[:90])

        # 9. Different planners: valid experiment, but not repeatability.
        e = load_run(write(root / "e", synthetic_doc(
            run_id="20260801-1_e", planner="teb_local_planner/TebLocalPlannerROS")))
        fatal, caveats = check_comparable([a, e])
        check("different planners flagged as not a repeatability measurement",
              not fatal and any("planner" in c for c in caveats),
              "; ".join(caveats)[:90])

        # 10. An aborted run loads, and says so.
        f = load_run(write(root / "f", synthetic_doc(run_id="20260801-1_f",
                                                      status="aborted")))
        check("aborted run loads and is flagged",
              any("ABORTED" in i for i in f.issues), "; ".join(f.issues)[:90])

        # 11. load_runs must survive a bad entry without losing the good ones.
        runs, errors = load_runs([good, root / "empty", root / "b"])
        check("load_runs keeps the readable runs and reports the rest",
              len(runs) == 2 and len(errors) == 1)

    width = max(len(d) for d, _, _ in checks)
    passed = 0
    for desc, ok, detail in checks:
        print("[%s] %s%s" % ("PASS" if ok else "FAIL", desc.ljust(width),
                             ("  " + detail) if detail else ""))
        passed += ok
    print("\n%d/%d checks passed" % (passed, len(checks)))
    return 0 if passed == len(checks) else 1


def main() -> int:
    import argparse
    ap = argparse.ArgumentParser(
        description="Reader and validator for sidewalk_navigation run "
                    "summaries. Contains no metrics: see traj_metrics.py.")
    ap.add_argument("--self-test", action="store_true",
                    help="check the reader against synthetic documents, "
                         "including ones it must refuse, then exit")
    ap.add_argument("--schema", action="store_true",
                    help="print the schema documentation and exit")
    ap.add_argument("run", nargs="*",
                    help="navigation run directories to describe")
    args = ap.parse_args()

    if args.schema:
        print(__doc__)
        return 0
    if args.self_test:
        return _self_test()
    if not args.run:
        ap.print_help()
        return 0

    runs, errors = load_runs(args.run)
    for err in errors:
        print("ERROR: %s\n" % err, file=sys.stderr)
    for run in runs:
        print("\n".join(run.describe()))
        for issue in run.issues:
            print("  NOTE: %s" % issue)
        print("")
    if len(runs) > 1:
        fatal, caveats = check_comparable(runs)
        for msg in fatal:
            print("NOT COMPARABLE: %s" % msg, file=sys.stderr)
        for msg in caveats:
            print("CAVEAT: %s" % msg)
        if fatal:
            return 2
    return 0 if runs and not errors else 1


if __name__ == "__main__":
    sys.exit(main())
