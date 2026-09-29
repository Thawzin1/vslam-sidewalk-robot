#!/usr/bin/env python3
"""sweep_db_integrity.py - check every map database properly, and prove it read the disk.

WHY THIS REPLACES THE ONE-OFF SWEEP OF 2026-08-31

    That sweep found 7 of 15 databases damaged, which was true, and then
    supported two conclusions that were both false. Each came from a defect
    in the CHECKING, not in the databases:

    1. IT SAVED ONLY THE FIRST ERROR LINE per file. Three tidy "failure
       families" appeared across those 15 first-lines, and a causal story was
       built on two of today's runs sharing one of them. The story died when
       someone opened a four-day-old file in the repo showing that the oldest
       database had the "new" mode on its SECOND line. The families were a
       property of the reporting.

       Plain terms: it wrote down each patient's first symptom and then
       grouped the patients by it.

    2. IT DROPPED THE PAGE CACHE ONCE, AT THE START. Two files had been read
       by other tools minutes earlier, so they were still in memory. Their
       checks ran at 193 and 125 MB/s against a card that tops out near 88,
       and thirteen honest files that all sat at 77-80. Those two verdicts
       describe a copy in memory, not the bytes on the card - and one of them
       was the single "clean" result the whole causal story rested on.

WHAT THIS DOES DIFFERENTLY

    - PRAGMA integrity_check(100000), not quick_check. Stricter, and the error
      limit is raised from its default of 100. quick_check skips index-content
      and constraint checking, so its "ok" means "no fault of the kind it looks
      for", never "sound".
    - Keeps EVERY error line and tallies them by family.
    - Drops the page cache BEFORE EACH FILE, not once per run.
    - Computes MB/s per file and FLAGS ANY READ FASTER THAN THE MEDIUM CAN GO.
      One line of arithmetic that catches a cached read every time.
    - Records cheap structural facts first: file size against
      page_count x page_size. If they disagree, SQLite has been ignoring part
      of the file, and every verdict on it only ever covered the part it
      believed in.

    A verdict without its read rate is not evidence. This prints both.

    usage:
      rosrun sidewalk_slam sweep_db_integrity.py
      rosrun sidewalk_slam sweep_db_integrity.py --quick      # first pass, faster
      rosrun sidewalk_slam sweep_db_integrity.py --media-mbs 87.7
"""
from __future__ import print_function

import argparse
import collections
import glob
import os
import re
import sqlite3
import subprocess
import sys
import time

DEFAULT_DIR = "/media/sidewalk/SIDEWALK128/rtabmap_maps"
# Measured sequential read for this card on ext4, the project records.
DEFAULT_MEDIA_MBS = 87.7

FAMILIES = [
    "2nd reference to page",
    "Rowid out of order",
    "Child page depth differs",
    "never used",
    "row missing from index",
    "wrong # of entries in index",
    "extends off end of page",
    "freelist",
]


def drop_caches():
    """Return (ok, note). Needs sudo. the Jetson operator cannot type a password, so the
    user runs `sudo -v` first; -n means this fails fast instead of hanging."""
    try:
        subprocess.check_call(["sync"])
        subprocess.check_call(
            ["sudo", "-n", "sh", "-c", "echo 3 > /proc/sys/vm/drop_caches"],
            stderr=subprocess.STDOUT)
        return True, ""
    except (subprocess.CalledProcessError, OSError):
        return False, "CACHE NOT DROPPED - this file may have been read from memory"


def structural_facts(path):
    """Cheap, one second. Does SQLite believe the whole file is a database?"""
    out = {"bytes": os.path.getsize(path)}
    try:
        con = sqlite3.connect("file:%s?immutable=1" % path, uri=True)
        pc = con.execute("PRAGMA page_count").fetchone()[0]
        ps = con.execute("PRAGMA page_size").fetchone()[0]
        out["free"] = con.execute("PRAGMA freelist_count").fetchone()[0]
        con.close()
        out["claimed"] = pc * ps
    except sqlite3.Error as exc:
        out["error"] = str(exc)[:60]
    return out


def check(path, quick, media_mbs):
    name = os.path.basename(path)
    facts = structural_facts(path)
    gb = facts["bytes"] / (1024.0 ** 3)

    cache_ok, cache_note = drop_caches()

    pragma = "quick_check" if quick else "integrity_check(100000)"
    t0 = time.time()
    lines = []
    bailed = False
    try:
        con = sqlite3.connect("file:%s?immutable=1" % path, uri=True)
        cur = con.execute("PRAGMA %s" % pragma)
        for row in cur:
            lines.extend(str(row[0]).splitlines())
        con.close()
    except sqlite3.Error as exc:
        # A bail-out is a RESULT, not an inconclusive. integrity_check refuses
        # to finish once errors run into the thousands, which means badly
        # damaged, not unknown.
        bailed = True
        lines.append("PRAGMA ABORTED: %s" % str(exc)[:80])
    secs = max(time.time() - t0, 1e-6)
    mbs = (facts["bytes"] / 1048576.0) / secs

    errs = [l for l in lines if l.strip() and l.strip() != "ok"
            and not l.startswith("*** in database")]
    ok = (not errs) and not bailed

    tally = collections.Counter()
    for l in errs:
        hit = next((f for f in FAMILIES if f in l), "other")
        tally[hit] += 1

    return {
        "name": name, "gb": gb, "secs": secs, "mbs": mbs, "ok": ok,
        "errors": errs, "tally": tally, "bailed": bailed,
        "cache_ok": cache_ok, "cache_note": cache_note,
        "claimed": facts.get("claimed"), "bytes": facts["bytes"],
        "free": facts.get("free"),
        "too_fast": mbs > media_mbs,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=DEFAULT_DIR)
    ap.add_argument("--quick", action="store_true",
                    help="quick_check instead of the full integrity_check")
    ap.add_argument("--media-mbs", type=float, default=DEFAULT_MEDIA_MBS,
                    help="the medium's known read speed; reads faster than this "
                         "came from memory and their verdicts are void")
    ap.add_argument("--out", default=os.path.expanduser("~/db_integrity_full.txt"))
    a = ap.parse_args()

    files = sorted(glob.glob(os.path.join(a.dir, "*", "*.db")),
                   key=os.path.getmtime)
    if not files:
        print("  no databases under %s" % a.dir)
        return 1

    fh = open(a.out, "w", 1)

    def emit(s):
        print(s)
        fh.write(s + "\n")

    emit("  %s sweep of %d databases, %.1f GB"
         % ("quick_check" if a.quick else "FULL integrity_check", len(files),
            sum(os.path.getsize(f) for f in files) / (1024.0 ** 3)))
    emit("  medium read speed taken as %.1f MB/s - anything faster was cached"
         % a.media_mbs)
    emit("")

    results = []
    for f in files:
        r = check(f, a.quick, a.media_mbs)
        results.append(r)
        flag = ""
        if r["too_fast"]:
            flag = "  <<< %.0f MB/s - READ FROM MEMORY, VERDICT VOID" % r["mbs"]
        elif not r["cache_ok"]:
            flag = "  <<< %s" % r["cache_note"]
        emit("  %-9s %6.2f GB %5.0fs %6.1f MB/s  %-30s%s"
             % ("OK" if r["ok"] else "DAMAGED", r["gb"], r["secs"], r["mbs"],
                r["name"], flag))
        if r["claimed"] and abs(r["claimed"] - r["bytes"]) > 65536:
            emit("             file is %.2f GB but SQLite believes %.2f GB is database"
                 % (r["bytes"] / 1073741824.0, r["claimed"] / 1073741824.0))
        if r["errors"]:
            emit("             %d error lines. by kind:" % len(r["errors"]))
            for kind, n in r["tally"].most_common():
                emit("               %-32s %d" % (kind, n))
            for l in r["errors"][:3]:
                emit("               e.g. %s" % l[:100])

    emit("")
    void = [r for r in results if r["too_fast"] or not r["cache_ok"]]
    bad = [r for r in results if not r["ok"]]
    emit("  %d of %d DAMAGED." % (len(bad), len(results)))
    if void:
        emit("  %d verdicts are VOID - read faster than the medium, or the cache"
             % len(void))
        emit("  could not be dropped. These are neither pass nor fail:")
        for r in void:
            emit("    %s  (%.0f MB/s)" % (r["name"], r["mbs"]))
        emit("  So the damaged count is a FLOOR, not a total.")
    if a.quick:
        emit("  NOTE: quick_check skips index-content and constraint checks.")
        emit("  An 'ok' here means 'no fault of the kind this test looks for'.")
    emit("")
    emit("  full output: %s" % a.out)
    fh.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
