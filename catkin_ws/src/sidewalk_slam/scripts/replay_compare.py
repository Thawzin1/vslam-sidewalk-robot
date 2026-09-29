#!/usr/bin/env python3
"""replay_compare.py - set a replayed drive beside the original drive, over the same time window.

WHY
    A replay proves itself by reproducing the drive. The original database and the replayed one
    both export the same two TUM files (db_to_tum.py: tracking alone; db_corrected_tum.py: the
    map's corrected positions). A camera recording usually covers only part of a drive, so the
    original is cut to the replay's time window before anything is compared.

HOW TO RUN
    replay_compare.py --original <dir> --replay <dir> [--window T0 T1] [--out compare.json]
    Each <dir> holds camera.tum and camera_corrected.tum. The window defaults to the replay's
    first and last tracking stamp.

WHAT IT PRINTS
    For each kind (tracking alone / corrected): poses in the window, path length, start-to-end
    gap, for the original and the replay. Then, if evo is installed, the absolute position
    difference between the two paths after one rigid alignment (evo_ape -a: rotation and
    translation only, never scale - ENGINEERING_NOTES.md section 4 rule 4). Two runs of the same programs
    on the same pictures are NOT expected to be identical: the SDK's real-time playback drops
    different frames than the live camera did, so this is a "how close" number, not a pass/fail.
"""
import argparse
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile


def read_tum(path):
    poses = []
    with open(path) as f:
        for line in f:
            parts = line.split()
            if len(parts) >= 8 and not parts[0].startswith("#"):
                poses.append([float(v) for v in parts[:8]])
    return poses


def in_window(poses, t0, t1):
    return [p for p in poses if t0 <= p[0] <= t1]


def gap(poses):
    if len(poses) < 2:
        return None
    return round(math.hypot(poses[-1][1] - poses[0][1], poses[-1][2] - poses[0][2]), 3)


def length(poses):
    return round(sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(poses, poses[1:])), 2)


def write_tum(poses, path):
    with open(path, "w") as f:
        for p in poses:
            f.write(" ".join("%.6f" % v for v in p) + "\n")


def evo_ape(reference, estimate, t_max_diff):
    """Absolute pose error after rigid alignment, as evo reports it. None if evo is missing."""
    if shutil.which("evo_ape") is None:
        return None
    cmd = ["evo_ape", "tum", reference, estimate, "-a", "--t_max_diff", str(t_max_diff), "--no_warnings"]
    done = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)
    stats = {}
    for line in done.stdout.splitlines():
        parts = line.split()
        if len(parts) == 2 and parts[0] in ("rmse", "mean", "median", "std", "min", "max"):
            stats[parts[0]] = float(parts[1])
    stats["evo_exit_code"] = done.returncode
    if not stats or done.returncode != 0:
        stats["evo_output_tail"] = done.stdout[-600:]
    return stats


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--original", required=True)
    parser.add_argument("--replay", required=True)
    parser.add_argument("--window", nargs=2, type=float, default=None, metavar=("T0", "T1"))
    parser.add_argument("--t-max-diff", type=float, default=0.25,
                        help="how far apart two stamps may be to count as the same moment (s)")
    parser.add_argument("--out", default=None)
    parser.add_argument("--late-start-s", type=float, default=5.0,
                        help="warn when a path's first pose in the window is later than this")
    args = parser.parse_args()

    result = {"window": None, "kinds": {}}
    replay_tracking = read_tum(os.path.join(args.replay, "camera.tum"))
    t0, t1 = args.window if args.window else (replay_tracking[0][0], replay_tracking[-1][0])
    result["window"] = [t0, t1]
    tmp = tempfile.mkdtemp(prefix="replay_compare_")
    for kind, name in (("tracking_alone", "camera.tum"), ("corrected", "camera_corrected.tum")):
        orig = in_window(read_tum(os.path.join(args.original, name)), t0, t1)
        rep = in_window(read_tum(os.path.join(args.replay, name)), t0, t1)
        # A corrected graph keeps only some nodes (RTAB-Map drops parked ones), so a path cut to the
        # window can start well after the window does - and then its "gap" is not start-to-end at all.
        late = [w for w, poses in (("original", orig), ("replay", rep))
                if poses and poses[0][0] - t0 > args.late_start_s]
        entry = {
            "late_start_warning": late,
            "original": {"poses": len(orig), "path_length_m": length(orig), "gap_m": gap(orig)},
            "replay": {"poses": len(rep), "path_length_m": length(rep), "gap_m": gap(rep)},
        }
        if len(orig) >= 3 and len(rep) >= 3:
            a, b = os.path.join(tmp, kind + "_orig.tum"), os.path.join(tmp, kind + "_replay.tum")
            write_tum(orig, a)
            write_tum(rep, b)
            entry["ape_after_rigid_alignment_m"] = evo_ape(a, b, args.t_max_diff)
        result["kinds"][kind] = entry
    shutil.rmtree(tmp, ignore_errors=True)

    print("window %.3f .. %.3f (%.1f s)" % (t0, t1, t1 - t0))
    print("%-16s %-9s %7s %10s %8s" % ("kind", "which", "poses", "length m", "gap m"))
    for kind, entry in result["kinds"].items():
        for which in ("original", "replay"):
            e = entry[which]
            print("%-16s %-9s %7d %10s %8s" % (kind, which, e["poses"], e["path_length_m"], e["gap_m"]))
        if entry["late_start_warning"]:
            print("%-16s WARNING: %s starts more than %.0f s after the window - its gap is not start-to-end; "
                  "use the whole-drive figure (db_corrected_tum.py's end_gap_corrected_m)" % (
                      kind, " and ".join(entry["late_start_warning"]), args.late_start_s))
        ape = entry.get("ape_after_rigid_alignment_m")
        if ape and "rmse" in ape:
            print("%-16s difference after rigid alignment: rmse %.3f m, median %.3f m, max %.3f m" % (
                kind, ape["rmse"], ape["median"], ape["max"]))
        elif ape:
            print("%-16s evo could not compare: %s" % (kind, ape.get("evo_output_tail", "")[-200:].strip()))
    if args.out:
        with open(args.out, "w") as f:
            json.dump(result, f, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
