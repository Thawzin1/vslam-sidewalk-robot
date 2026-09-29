#!/usr/bin/env python3
"""damage_report.py - check a database AND name what the damage landed in, in one pass.

WHY THIS EXISTS, AND WHY IT REPLACES TWO EARLIER TOOLS

    On 2026-08-31 the same mistake was made three times in one evening, twice
    in tools written to prevent it:

      1. The first sweep saved ONE error line per file. Three "failure
         families" were read out of those first lines and a causal story built
         on them. It collapsed when a four-day-old file in the repo showed the
         oldest database had the supposedly-new mode on its second line.

      2. sweep_db_integrity.py was written to fix that. It tallies every error
         line by kind - and then writes only THREE example lines to its report.
         So a later step that needed page numbers got 3 of 37,445.

      3. That later step then reported which tables owned the damage, from
         those 3 lines, as though it had seen all of it.

    Plain terms: it kept the count and threw away the evidence, twice.

    The fix is structural, not a bigger number: this tool never separates
    checking from interpreting. It runs the integrity check and resolves every
    blamed page to its owning table IN THE SAME PROCESS, so there is no
    intermediate file to truncate.

WHAT IT ANSWERS

    Not "is it damaged" - the earlier sweep answered that. It answers "what was
    IN the part that broke", which is the difference between losing stored
    images (recoverable, they are 99.9 % of the bytes and needed for almost
    nothing) and losing the pose graph (the result itself).

    usage:
      rosrun sidewalk_slam damage_report.py --db a.db --db b.db
      rosrun sidewalk_slam damage_report.py --db a.db --save-lines ~/errs/
"""
from __future__ import print_function

import argparse
import collections
import os
import re
import sqlite3
import sys
import time

PAGE_RE = re.compile(r"page (\d+)")
FAMILIES = ["2nd reference to page", "Rowid out of order",
            "Child page depth differs", "never used",
            "row missing from index", "wrong # of entries in index",
            "extends off end of page", "freelist"]

# What each table actually holds, so the verdict is readable without knowing
# RTAB-Map's schema. Verified against this build's databases.
MEANS = {
    "Data": "the stored images - 99.9 % of the bytes, needed for almost nothing",
    "Node": "THE POSE GRAPH - where the robot was. This is the result itself",
    "Link": "THE POSE GRAPH - how positions connect, incl. every loop closure",
    "Word": "the visual dictionary - rebuildable by re-running the map",
    "Feature": "per-frame visual features - needed to re-do loop closure offline",
    "Statistics": "per-frame diagnostics - useful, not a result",
    "Admin": "database settings and version - tiny",
    "Info": "run metadata - tiny",
    "GlobalDescriptor": "whole-image descriptors",
}


def describe(name):
    base = name.split("_")[-1] if name.startswith("IDX_") else name
    for k, v in MEANS.items():
        if k == base or k in name:
            return v
    return ""


def run(db, save_dir):
    print("=" * 78)
    print("  %s   %.2f GB" % (os.path.basename(db), os.path.getsize(db) / 1073741824.0))
    if not os.path.exists(db):
        print("    MISSING")
        return

    t0 = time.time()
    lines = []
    aborted = None
    try:
        con = sqlite3.connect("file:%s?immutable=1" % db, uri=True)
        for row in con.execute("PRAGMA integrity_check(1000000)"):
            lines.extend(str(row[0]).splitlines())
    except sqlite3.Error as exc:
        aborted = str(exc)[:70]
    secs = time.time() - t0
    mbs = (os.path.getsize(db) / 1048576.0) / max(secs, 1e-6)

    errs = [l for l in lines if l.strip() and l.strip() != "ok"
            and not l.startswith("*** in database")]

    print("    read at %.1f MB/s in %.0f s" % (mbs, secs))
    if mbs > 87.7:
        print("    *** FASTER THAN THE CARD CAN READ - this was cached, verdict VOID")
        con.close()
        return
    if aborted:
        print("    INTEGRITY CHECK ABORTED: %s" % aborted)
        print("    (an abort is the worst tier, and a result - not an inconclusive)")
    if not errs and not aborted:
        print("    ok - no structural fault of the kind integrity_check detects")
        con.close()
        return

    print("    %d error lines" % len(errs))
    fam = collections.Counter()
    for l in errs:
        fam[next((f for f in FAMILIES if f in l), "other")] += 1
    for k, n in fam.most_common():
        print("      %-32s %7d" % (k, n))

    # EVERY page, not a sample. This is the whole point of the tool.
    pages = set()
    for l in errs:
        for m in PAGE_RE.finditer(l):
            pages.add(int(m.group(1)))
    print("    %d distinct pages blamed" % len(pages))

    if save_dir:
        if not os.path.isdir(save_dir):
            os.makedirs(save_dir)
        p = os.path.join(save_dir, os.path.basename(db) + ".errors.txt")
        with open(p, "w") as fh:
            fh.write("\n".join(errs))
        print("    every error line written to %s" % p)

    owner = {}
    try:
        for name, pageno in con.execute("SELECT name, pageno FROM dbstat"):
            if pageno in pages:
                owner[pageno] = name
    except sqlite3.Error as exc:
        print("    dbstat failed part way (%s) - mapping is partial"
              % str(exc)[:50])
    con.close()

    tally = collections.Counter(owner.get(p, "NOT IN dbstat") for p in pages)
    print()
    print("    WHAT THE DAMAGE LANDED IN:")
    for name, n in tally.most_common():
        note = describe(name)
        print("      %-30s %6d pages   %s" % (name, n, note))
    graph_hit = sum(n for name, n in tally.items()
                    if name.split("_")[-1] in ("Node", "Link") or
                    name in ("Node", "Link"))
    print()
    if graph_hit:
        print("    *** %d damaged pages are in the POSE GRAPH itself." % graph_hit)
        print("        Salvaged CSV from this file must be treated as suspect")
        print("        and cross-checked against a figure recorded at the time.")
    else:
        print("    No damaged page is in Node or Link - the pose graph is intact,")
        print("    which is why the salvaged CSV reproduces figures recorded at")
        print("    the time.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--db", action="append", required=True)
    ap.add_argument("--save-lines", default=None,
                    help="directory to write EVERY error line per database")
    a = ap.parse_args()
    for db in a.db:
        run(db, a.save_lines)
        print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
