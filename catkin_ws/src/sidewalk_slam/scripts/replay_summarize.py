#!/usr/bin/env python3
"""replay_summarize.py - the numbers of a finished replay, in one JSON file and one printed table.

WHY
    A replay is only useful if it can be compared with the drive it re-ran. This gathers the
    same figures the project reports for every drive (ENGINEERING_NOTES.md rule 20: BOTH start-to-end gaps,
    each with its closure count) plus the one figure only a replay has: how far the camera stream
    drifted from the wheel data.

HOW TO RUN (run/replay.sh does this)
    replay_summarize.py --run <run> --db <new map .db> --out-dir <dir> --tools-dir <dir with
                        db_to_tum.py and db_corrected_tum.py> --svo-index <svo_index.csv>
                        [--wall-s <seconds the replay took>] [--monitor-csv <run_monitor csv>]

OUTPUT  <out_dir>/camera.tum            the camera's tracking alone (Node.pose, never corrected)
        <out_dir>/camera_corrected.tum  the map's corrected positions (Admin.opt_poses, or re-optimised)
        <out_dir>/replay_summary.json   every figure below, machine readable
        A short table on standard output.

THE TWO GAPS (rule 20)
    tracking_gap_m   distance between the first and last pose of camera.tum - how lost the camera's
                     own tracking got, loop closures never correct it
    corrected_gap_m  the same distance in camera_corrected.tum - how well the map recovered
    A gap only means "returned to the start" if the drive did return; a one-way drive has a large
    gap by design.

CLOCK DRIFT (from clock_drift.csv written by replay_watch.py)
    offset_s = (bag clock) - (true capture time of the picture shown at that moment), so positive
    means the pictures lag the wheel data. Reported at the start and end of the run; the change
    between them is the drift. Two witnesses: the SVO frame counter and the camera IMU stamps.
"""
import argparse
import csv
import json
import math
import os
import sqlite3
import subprocess
import sys


def run_tool(tools_dir, script, arguments):
    """Run one of the project's small database tools and return its printed output."""
    cmd = [sys.executable, os.path.join(tools_dir, script)] + arguments
    done = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)
    return done.returncode, done.stdout


def read_tum(path):
    """A TUM file: one pose per line, 't x y z qx qy qz qw [id]'. Returns a list of (t, x, y, z)."""
    poses = []
    if not os.path.exists(path):
        return poses
    with open(path) as f:
        for line in f:
            parts = line.split()
            if len(parts) < 8 or parts[0].startswith("#"):
                continue
            poses.append(tuple(float(v) for v in parts[:4]))
    return poses


def start_to_end_gap(poses):
    """Distance in the floor plane between the first and the last pose (metres)."""
    if len(poses) < 2:
        return None
    dx = poses[-1][1] - poses[0][1]
    dy = poses[-1][2] - poses[0][2]
    return round(math.hypot(dx, dy), 3)


def path_length(poses):
    """Sum of the straight-line steps between consecutive poses, in the floor plane (metres)."""
    total = 0.0
    for a, b in zip(poses, poses[1:]):
        total += math.hypot(b[1] - a[1], b[2] - a[2])
    return round(total, 2)


def database_counts(db_path):
    """Node and closure counts straight from the database.

    Link.type: 0 = neighbour, 1 = global loop closure, 2/3 = proximity closure (space/time),
    4 = user, 9 = gravity. Every link is stored in both directions, so closures are counted once
    per pair (from_id > to_id), the same way db_corrected_tum.py and the drives' reports count them.
    """
    con = sqlite3.connect("file:%s?immutable=1" % db_path, uri=True)
    nodes = con.execute("SELECT COUNT(*) FROM Node").fetchone()[0]
    links_by_type = dict(con.execute("SELECT type, COUNT(*) FROM Link WHERE from_id > to_id GROUP BY type").fetchall())
    opt_poses = con.execute("SELECT opt_poses IS NOT NULL FROM Admin LIMIT 1").fetchone()
    con.close()
    return {
        "nodes": nodes,
        "loop_closures": sum(n for t, n in links_by_type.items() if t in (1, 2, 3, 4)),
        "loop_closures_global": links_by_type.get(1, 0),
        "loop_closures_proximity": links_by_type.get(2, 0) + links_by_type.get(3, 0),
        "links_by_type_one_way": {str(k): v for k, v in links_by_type.items()},
        "corrected_poses_saved": bool(opt_poses and opt_poses[0]),
    }


def median(values):
    s = sorted(values)
    return s[len(s) // 2] if s else None


def drift_from_witness(samples, window_s=30.0):
    """samples = [(sim_time, offset_s)]. Median offset in the first and last window, and the change."""
    if len(samples) < 2:
        return None
    samples.sort()
    t0, t1 = samples[0][0], samples[-1][0]
    first = [o for t, o in samples if t <= t0 + window_s]
    last = [o for t, o in samples if t >= t1 - window_s]
    return {
        "samples": len(samples),
        "offset_start_s": round(median(first), 3),
        "offset_end_s": round(median(last), 3),
        "drift_s": round(median(last) - median(first), 3),
        "offset_min_s": round(min(o for _, o in samples), 3),
        "offset_max_s": round(max(o for _, o in samples), 3),
        "span_s": round(t1 - t0, 1),
    }


def clock_drift(out_dir, svo_index_csv):
    """Combine clock_drift.csv (replay_watch.py) with the SVO index into offset figures."""
    drift_csv = os.path.join(out_dir, "clock_drift.csv")
    if not os.path.exists(drift_csv):
        return {"note": "no clock_drift.csv"}
    frame_time = {}
    with open(svo_index_csv) as f:
        for row in csv.DictReader(f):
            frame_time[int(row["frame"])] = int(row["timestamp_ns"]) / 1e9
    frame_samples, imu_samples = [], []
    with open(drift_csv) as f:
        for row in csv.DictReader(f):
            t = float(row["sim_time"])
            if row["kind"] == "svo_frame":
                # getSVOPosition() counts frames already read, so the picture on screen is k-1
                k = int(row["value"]) - 1
                if k in frame_time:
                    frame_samples.append((t, t - frame_time[k]))
            elif row["kind"] == "imu_delta":
                imu_samples.append((t, float(row["value"])))
    # The IMU topic carries two kinds of message: stamped "now" (delta within a few ms) and stamped
    # with the original recorded time (delta = the true offset). Anything within 20 ms of zero is
    # taken as the first kind; if the true offset is itself under 20 ms the two merge, which is fine.
    original_stamped = [(t, d) for t, d in imu_samples if abs(d) > 0.02]
    return {
        "by_svo_frame_counter": drift_from_witness(frame_samples),
        "by_camera_imu_stamps": drift_from_witness(original_stamped),
        "imu_messages_logged": len(imu_samples),
        "imu_messages_with_original_stamp": len(original_stamped),
    }


def last_monitor_value(monitor_csv, column):
    """The last value of one column of run_monitor.py's CSV (e.g. 'resets')."""
    if not monitor_csv or not os.path.exists(monitor_csv):
        return None
    value = None
    with open(monitor_csv) as f:
        for row in csv.DictReader(f):
            if row.get(column) not in (None, ""):
                value = row[column]
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--run", required=True)
    parser.add_argument("--db", required=True)
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--tools-dir", required=True)
    parser.add_argument("--svo-index", required=True)
    parser.add_argument("--wall-s", type=float, default=None)
    parser.add_argument("--monitor-csv", default=None)
    parser.add_argument("--extra", default=None, help="a JSON file whose keys are copied into the summary")
    args = parser.parse_args()

    tracking_tum = os.path.join(args.out_dir, "camera.tum")
    corrected_tum = os.path.join(args.out_dir, "camera_corrected.tum")
    rc1, out1 = run_tool(args.tools_dir, "db_to_tum.py", [args.db, tracking_tum, "--quiet"])
    rc2, out2 = run_tool(args.tools_dir, "db_corrected_tum.py", [args.db, corrected_tum, "--label", "camera replay"])
    with open(os.path.join(args.out_dir, "tum_tools.log"), "w") as f:
        f.write("db_to_tum.py rc=%d\n%s\ndb_corrected_tum.py rc=%d\n%s\n" % (rc1, out1, rc2, out2))

    tracking = read_tum(tracking_tum)
    corrected = read_tum(corrected_tum)
    summary = {"run": args.run, "database": args.db, "wall_time_s": args.wall_s}
    summary.update(database_counts(args.db))
    summary.update({
        "tracking_poses": len(tracking),
        "tracking_gap_m": start_to_end_gap(tracking),
        "tracking_path_length_m": path_length(tracking),
        "corrected_poses": len(corrected),
        "corrected_gap_m": start_to_end_gap(corrected),
        "corrected_path_length_m": path_length(corrected),
        "first_pose_stamp": tracking[0][0] if tracking else None,
        "last_pose_stamp": tracking[-1][0] if tracking else None,
        "odometry_resets": last_monitor_value(args.monitor_csv, "resets"),
        "clock_drift": clock_drift(args.out_dir, args.svo_index),
    })
    if args.extra and os.path.exists(args.extra):
        with open(args.extra) as f:
            summary.update(json.load(f))
    with open(os.path.join(args.out_dir, "replay_summary.json"), "w") as f:
        json.dump(summary, f, indent=1)

    print("replay summary for %s" % args.run)
    print("  map nodes               %s" % summary["nodes"])
    print("  loop closures           %s (%s global + %s proximity)" % (
        summary["loop_closures"], summary["loop_closures_global"], summary["loop_closures_proximity"]))
    print("  tracking-alone gap      %s m over %s m of path (%d poses)" % (
        summary["tracking_gap_m"], summary["tracking_path_length_m"], len(tracking)))
    print("  corrected gap           %s m (%d poses, corrected poses saved: %s)" % (
        summary["corrected_gap_m"], len(corrected), summary["corrected_poses_saved"]))
    print("  odometry resets         %s" % summary["odometry_resets"])
    for name, w in (("SVO frame counter", summary["clock_drift"].get("by_svo_frame_counter")),
                    ("camera IMU stamps", summary["clock_drift"].get("by_camera_imu_stamps"))):
        if w:
            print("  clock offset by %-18s start %+.3f s, end %+.3f s, drift %+.3f s (%d samples)" % (
                name, w["offset_start_s"], w["offset_end_s"], w["drift_s"], w["samples"]))
        else:
            print("  clock offset by %-18s not measurable" % name)
    return 0


if __name__ == "__main__":
    sys.exit(main())
