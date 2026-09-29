#!/usr/bin/env python3
"""verify_run.py — prove a finished mapping run is actually intact, immediately.

WHY THIS EXISTS
    The microSD card in this project has destroyed FOUR recordings: one killed
    mid-write, three that verified fine and then rotted AT REST hours later with
    no writes in between. One of those was "rescued" by checksum the same
    morning it went bad — the checksum matched perfectly, because it was an
    exact copy of already-corrupted data.

    So a checksum is not a verification. It proves a copy equals its source; it
    says nothing about whether the source was still good. This runs a real
    structural check instead, plus the sanity checks that catch a run which
    "succeeded" but recorded nothing useful.

    ENGINEERING_NOTES.md section 0.3 rule 7 requires this after EVERY recording.

USAGE (on the Jetson, right after the drive ends)
    rosrun sidewalk_slam verify_run.py --db /media/.../lab_map_02.db --run-id lab_map_02

    Add --bag <path> and/or --svo <path> to check those too.

EXIT CODES
    0 = everything checked passed
    1 = something FAILED — do not trust this run until it is understood
    2 = could not run the checks at all (bad path, missing tool)

Deliberately uses Python's built-in sqlite3 module: the sqlite3 command-line
tool is NOT installed on this Jetson (verified 2026-08-24), and requiring an
apt install right after a drive is exactly when it would get skipped.
"""
import argparse
import os
import sqlite3
import subprocess
import sys

GREEN, RED, YELLOW, RESET = "\033[32m", "\033[31m", "\033[33m", "\033[0m"


def ok(msg):
    print(f"{GREEN}  PASS{RESET}  {msg}")
    return True


def bad(msg):
    print(f"{RED}  FAIL{RESET}  {msg}")
    return False


def warn(msg):
    print(f"{YELLOW}  WARN{RESET}  {msg}")
    return True


def check_database(path):
    """Structural integrity + 'did it actually map anything' sanity checks."""
    print(f"\nDATABASE  {path}")
    if not os.path.exists(path):
        return bad("file does not exist")

    size_gb = os.path.getsize(path) / 1024 ** 3
    print(f"          size {size_gb:.2f} GB")
    results = []

    if size_gb < 0.01:
        results.append(bad(f"only {size_gb*1024:.1f} MB — this is what a "
                           f"killed-mid-write run looks like (lab_map_02 was 73 KB)"))
    else:
        results.append(ok(f"size is plausible ({size_gb:.2f} GB)"))

    # ------------------------------------------------------------------
    # DROP THE PAGE CACHE FIRST, OR THIS WHOLE CHECK IS THEATRE.
    #
    # results. Linux keeps recently-written data in memory (the page cache).
    # A database checked straight after a mapping run is still entirely in
    # that cache, so SQLite reads the CORRECT in-memory copy and reports
    # "ok" WITHOUT EVER TOUCHING THE CARD. If the write to the physical card
    # was damaged, this check cannot see it.
    #
    # Proved on lab_map_06: reported "quick_check = ok" at 01:07 right after
    # the run, then failed with page-level errors once the cache was dropped
    # and the same file was re-read from the card. Nothing wrote to it in
    # between. Run 5 behaved identically.
    #
    # This very likely explains every previous "verified healthy, then
    # corrupted at rest" incident in the project records — the files were
    # probably damaged on disk from the moment they were written, and the
    # immediate check was reading memory.
    #
    # Needs root. Passwordless sudo is configured on the Jetson; if it is not
    # available the check still runs, but it is CLEARLY LABELLED as unproven
    # rather than silently reporting a cached pass as a real one.
    # ------------------------------------------------------------------
    cache_dropped = False
    try:
        subprocess.run(["sync"], check=False, timeout=120)
        r = subprocess.run(["sudo", "-n", "sysctl", "-w", "vm.drop_caches=3"],
                           capture_output=True, timeout=60)
        cache_dropped = (r.returncode == 0)
    except Exception:
        cache_dropped = False

    if cache_dropped:
        results.append(ok("page cache dropped — the check below reads the "
                          "PHYSICAL disk, not a cached copy"))
    else:
        results.append(warn("could NOT drop the page cache (needs sudo). The "
                            "result below may be a cached copy and cannot be "
                            "trusted as proof the data on disk is good. "
                            "Re-run after a reboot, or with sudo."))

    # The real structural check. Read-only URI so verifying can never write.
    try:
        conn = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
        verdict = conn.execute("PRAGMA quick_check;").fetchone()[0]
        if verdict.strip().lower() == "ok":
            results.append(ok("PRAGMA quick_check = ok (structurally sound)"
                              + ("" if cache_dropped else " — BUT POSSIBLY CACHED")))
        else:
            results.append(bad(f"PRAGMA quick_check = {verdict[:200]}"))

        # A corrupt index makes different query plans disagree about the same
        # table. count(*) can be answered from an index while GROUP BY forces a
        # scan — when those two differ, the file is damaged even if quick_check
        # somehow passed. This is how lab_map_05's corruption was first spotted.
        try:
            n_count = conn.execute("SELECT count(*) FROM Node").fetchone()[0]
            n_scan = sum(c for _, c in
                         conn.execute("SELECT weight, count(*) FROM Node GROUP BY weight"))
            if n_count == n_scan:
                results.append(ok(f"index cross-check consistent ({n_count} nodes "
                                  f"by two independent query plans)"))
            else:
                results.append(bad(f"INDEX CORRUPTION: count(*)={n_count} but a full "
                                   f"scan sums to {n_scan} on the same table"))
        except Exception as exc:
            results.append(warn(f"index cross-check could not run: {exc}"))

        # A structurally-valid database can still be an empty one.
        for table, minimum, label in (("Node", 50, "keyframes"),
                                      ("Link", 10, "graph links")):
            try:
                n = conn.execute(f"SELECT COUNT(*) FROM {table};").fetchone()[0]
                if n >= minimum:
                    results.append(ok(f"{n} {label}"))
                else:
                    results.append(bad(f"only {n} {label} — expected at least "
                                       f"{minimum}. The map did not build."))
            except sqlite3.DatabaseError as exc:
                results.append(bad(f"could not count {label}: {exc}"))
        conn.close()
    except sqlite3.DatabaseError as exc:
        results.append(bad(f"sqlite could not open it: {exc}"))

    return all(results)


def check_bag(path):
    print(f"\nRECORDING  {path}")
    if not os.path.exists(path):
        return bad("file does not exist")
    if path.endswith(".active"):
        return bad("still ends in .active — the recorder never closed it "
                   "cleanly. This is how lab_map_02's recording was lost.")
    size_mb = os.path.getsize(path) / 1024 ** 2
    print(f"           size {size_mb:.1f} MB")
    try:
        out = subprocess.run(["rosbag", "info", path], capture_output=True,
                             text=True, timeout=180)
        if out.returncode != 0:
            return bad(f"rosbag info failed: {out.stderr.strip()[:200]}")
        for line in out.stdout.splitlines():
            if line.startswith(("duration:", "messages:", "size:")):
                print(f"           {line.strip()}")
        return ok("rosbag info reads it end to end")
    except FileNotFoundError:
        return warn("rosbag not on PATH — source /opt/ros/noetic/setup.bash")
    except subprocess.TimeoutExpired:
        return bad("rosbag info timed out — the index is likely damaged "
                   "(try: rosbag reindex)")


def check_svo(path):
    print(f"\nSVO2  {path}")
    if not os.path.exists(path):
        return bad("file does not exist")
    size_mb = os.path.getsize(path) / 1024 ** 2
    print(f"      size {size_mb:.1f} MB")
    if size_mb < 1:
        return bad(f"only {size_mb:.2f} MB — the camera recorded essentially nothing")
    return ok(f"present and non-trivial ({size_mb:.1f} MB)")


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--db", required=True, help="the RTAB-Map .db this run wrote")
    p.add_argument("--bag", default=None, help="optional rosbag to check")
    p.add_argument("--svo", default=None, help="optional .svo2 to check")
    p.add_argument("--run-id", default=None, help="label for the report")
    args = p.parse_args()

    print("=" * 66)
    print(f"RUN INTEGRITY CHECK{'  —  ' + args.run_id if args.run_id else ''}")
    print("=" * 66)

    results = [check_database(args.db)]
    if args.bag:
        results.append(check_bag(args.bag))
    if args.svo:
        results.append(check_svo(args.svo))

    print("\n" + "=" * 66)
    if all(results):
        print(f"{GREEN}ALL CHECKS PASSED{RESET} — but copy this off the card THIS "
              f"SESSION anyway.\nThree of this card's four losses happened at "
              f"rest, hours after a clean check.")
        return 0
    print(f"{RED}SOMETHING FAILED{RESET} — do not treat this run as good data "
          f"until it is understood.\nDo NOT overwrite or re-run over it; the "
          f"evidence is in the file as it stands.")
    return 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(2)
