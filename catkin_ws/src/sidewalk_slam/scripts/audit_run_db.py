#!/usr/bin/env python3
"""audit_run_db.py - ask the database itself, not the counters that watched it.

WHY THIS EXISTS

    On 2026-08-31 three separate numbers in this project turned out to be
    absent rather than zero:

      - a visual-word count read back as -1 because the statistic key name was
        wrong. It was briefly reported as "NEURAL produces zero visual words",
        which would have been a headline finding and was false.
      - a tracking figure read from a column of the run log that nothing ever
        writes to. It holds -1. A watcher alarmed on it as "tracking has
        collapsed" twice before the cause was spotted.
      - a rejected-loop-closure count that has read exactly 0 on every run ever
        recorded here, which is either true or a counter nobody wired up.

    Plain terms: a number that was never filled in looks exactly like a number
    that was measured and came out zero. Telling those apart is the whole job
    of this script.

WHAT IT DOES

    Opens the run's database directly and counts what is actually stored -
    nodes, links by kind, visual words, and how many words each frame carried.
    Then, if a run log is given, it compares those counts against what the
    live counters claimed at the time.

    Agreement means both are probably right. Disagreement means at least one
    is wrong, and the database wins, because it holds the thing itself rather
    than a report about it.

WHY IT DOES NOT TRUST ITS OWN LINK-TYPE TABLE

    The numbers RTAB-Map stores for link kinds are an internal enumeration that
    can be renumbered between versions. This script prints the distribution of
    whatever type numbers it finds and labels them from a table that MUST be
    checked against the installed headers before the labels are quoted:

        grep -n "kNeighbor\\|kGlobalClosure\\|kLocalSpaceClosure" \\
             /usr/include/rtabmap-0.21/rtabmap/core/Link.h

    Until that check is done the counts are trustworthy and the LABELS are not.
    The script says so in its own output rather than letting a reader assume.

SAFETY

    Read-only. Opens the file with SQLite's immutable flag so it cannot write,
    cannot create a journal beside the database, and cannot block a writer.
    Still: do NOT run this against a database a live run is writing to. This
    project's microSD card has destroyed four files, and adding read traffic
    to a card mid-write is how two of them died.

    usage:
      rosrun sidewalk_slam audit_run_db.py --db /path/to/run.db
      rosrun sidewalk_slam audit_run_db.py --db a.db --db b.db     # compare runs
      rosrun sidewalk_slam audit_run_db.py --db run.db --csv monitor.csv
"""
from __future__ import print_function

import argparse
import os
import sqlite3
import sys

# UNVERIFIED against this build's headers - see the docstring. Counts are real;
# these names are a guess until someone greps Link.h.
LINK_TYPE_NAMES = {
    0: "neighbour (consecutive frames - NOT a loop closure)",
    1: "global loop closure",
    2: "local space / proximity closure",
    3: "local time closure",
    4: "user-added closure",
    5: "virtual closure",
    6: "merged neighbour",
    7: "pose prior",
    8: "landmark",
    9: "gravity",
}
CLOSURE_TYPES = (1, 2, 3, 4)


def q1(cur, sql, default=None):
    """Run a query that should return one number. Return default if the table
    does not exist, rather than dying - schemas differ between versions, and a
    missing table is information, not a crash."""
    try:
        cur.execute(sql)
        row = cur.fetchone()
        if row is None or row[0] is None:
            return default
        return row[0]
    except sqlite3.Error:
        return default


def tables(cur):
    cur.execute("SELECT name FROM sqlite_master WHERE type='table'")
    return sorted(r[0] for r in cur.fetchall())


def audit(path):
    if not os.path.exists(path):
        print("  MISSING: %s" % path)
        return None
    size = os.path.getsize(path)
    print("=" * 74)
    print("  %s" % path)
    print("  %.2f GB on disk" % (size / (1024.0 ** 3)))
    if size == 0:
        print("  ZERO BYTES. Nothing to audit - the run wrote no database.")
        return None

    # immutable=1: read-only, no journal, cannot disturb anything.
    uri = "file:%s?immutable=1" % os.path.abspath(path).replace("?", "%3f")
    try:
        con = sqlite3.connect(uri, uri=True)
    except sqlite3.Error as exc:
        print("  CANNOT OPEN: %s" % exc)
        return None
    cur = con.cursor()

    present = tables(cur)
    print("  tables: %s" % ", ".join(present))

    out = {"path": path, "size": size}

    nodes = q1(cur, "SELECT COUNT(*) FROM Node")
    out["nodes"] = nodes
    print()
    print("  nodes (mapped frames)      %s" % fmt(nodes))

    # --- visual words: the statistic that was misread -----------------------
    words = q1(cur, "SELECT COUNT(*) FROM Word")
    out["words"] = words
    if words is None:
        print("  visual words               NO 'Word' TABLE - cannot tell")
        print("                             absent from zero. Do NOT report 0.")
    else:
        print("  visual words in dictionary %s" % fmt(words))

    # How many words each frame actually carried. A dictionary can be large
    # while individual frames are empty, and it is per-frame words that decide
    # whether a place can ever be recognised again.
    fpn = None
    for sql in ("SELECT COUNT(*) FROM Feature",
                "SELECT COUNT(*) FROM Map_Node_Word"):
        fpn = q1(cur, sql)
        if fpn is not None:
            break
    if fpn is not None and nodes:
        print("  word observations          %s  (~%.0f per frame)"
              % (fmt(fpn), float(fpn) / nodes))
    out["word_obs"] = fpn

    # --- links: the closure counts ------------------------------------------
    #
    # COUNTING ROWS HERE DOUBLES EVERY CLOSURE. RTAB-Map stores each link twice,
    # once as A->B and once as B->A. Counting rows once had run 7c reported as
    # 84 global and 250 proximity closures; the true figures are 42 and 125.
    #
    # This is DEMONSTRATED per run rather than assumed, by printing the ratio of
    # rows to distinct unordered pairs. Gravity links are self-links (from==to)
    # and are NOT doubled - their ratio is 1.00 - so a blanket "divide by two"
    # would be wrong too.
    print()
    try:
        cur.execute("SELECT type, COUNT(*) FROM Link GROUP BY type ORDER BY type")
        rows = cur.fetchall()
    except sqlite3.Error:
        rows = []
    if not rows:
        print("  NO 'Link' TABLE OR NO LINKS. A database with frames but no")
        print("  links has no pose graph - this is the failure mode that a")
        print("  Ctrl+C mid-write produced on 2026-08-30, and no integrity")
        print("  check catches it.")
        out["closures"] = None
    else:
        print("  links by kind. DISTINCT UNORDERED PAIRS is the real count;")
        print("  'rows' is what the table holds and is double for real links.")
        print("  (counts VERIFIED, labels UNVERIFIED - Link.h is not installed")
        print("   on this machine, so the names below are not confirmed.)")
        print()
        print("    %-4s %-40s %8s %9s %6s" % ("type", "label (unverified)",
                                              "rows", "distinct", "ratio"))
        closures = 0
        by_type = {}
        for t, n in rows:
            d = q1(cur,
                   "SELECT COUNT(*) FROM (SELECT DISTINCT "
                   "MIN(from_id,to_id) a, MAX(from_id,to_id) b "
                   "FROM Link WHERE type=%d)" % int(t))
            if d is None or d == 0:
                d = n
            ratio = float(n) / d
            by_type[t] = {"rows": n, "distinct": d, "ratio": ratio}
            mark = " <-- closure" if t in CLOSURE_TYPES else ""
            print("    %-4s %-40s %8s %9s %6.2f%s"
                  % (t, LINK_TYPE_NAMES.get(t, "UNKNOWN - check Link.h")[:40],
                     fmt(n), fmt(d), ratio, mark))
            if t in CLOSURE_TYPES:
                closures += d
        out["closures"] = closures
        out["links_by_type"] = by_type
        print()
        print("    %-46s %s" % ("TOTAL loop closures (distinct pairs)",
                                fmt(closures)))
        # If a real-link ratio is not 2.00, the doubling assumption does not
        # hold for this file and the total must not be quoted without saying so.
        odd = [t for t, v in by_type.items()
               if t in CLOSURE_TYPES and abs(v["ratio"] - 2.0) > 0.01]
        if odd:
            print("    *** types %s do NOT have a 2.00 row/pair ratio."
                  % ", ".join(str(t) for t in sorted(odd)))
            print("    *** The mirroring assumption does not hold here. Do not")
            print("    *** quote this total without explaining why.")
    con.close()
    return out


def fmt(n):
    return "(none)" if n is None else "{:,}".format(n)


def compare_csv(out, csv_path):
    """The live counters said X. The database holds Y. Do they agree?"""
    if out is None or not os.path.exists(csv_path):
        return
    try:
        with open(csv_path) as fh:
            lines = [l.strip() for l in fh if l.strip()]
    except IOError:
        return
    if len(lines) < 2:
        return
    head = lines[0].split(",")
    last = lines[-1].split(",")
    row = dict(zip(head, last))

    def num(key):
        try:
            return int(float(row.get(key, "")))
        except (TypeError, ValueError):
            return None

    live = num("accepted_lc")
    rej = num("rejected_lc")
    print()
    print("  --- live counters vs what the database actually holds ---")
    print("  monitor.csv accepted_lc    %s" % fmt(live))
    print("  monitor.csv rejected_lc    %s" % fmt(rej))
    db = out.get("closures")
    if live is None or db is None:
        print("  cannot compare - one side is absent (which is NOT zero)")
        return
    diff = abs(live - db)
    if diff == 0:
        print("  AGREE exactly. Both are probably right.")
    else:
        print("  DISAGREE by %d. The database wins; it holds the links" % diff)
        print("  themselves rather than a report about them. Some difference")
        print("  is expected if the counter stopped before the run did.")
    if rej == 0:
        print()
        print("  rejected_lc is 0. That is either true, or nothing ever writes")
        print("  to that column. It has read 0 on EVERY run recorded here, which")
        print("  is the pattern an unwired counter makes. Settle it by finding a")
        print("  rejection in the mapping log before quoting 'zero rejected':")
        print("    grep -ac 'Rejected loop closure\\|rejected hypothesis' mapping.log")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--db", action="append", required=True,
                    help="database to audit; repeat to compare runs")
    ap.add_argument("--csv", default=None,
                    help="the run's monitor.csv, to check the live counters")
    args = ap.parse_args()

    results = []
    for path in args.db:
        r = audit(path)
        if r and args.csv:
            compare_csv(r, args.csv)
        results.append(r)
        print()

    good = [r for r in results if r]
    if len(good) > 1:
        print("=" * 74)
        print("  SIDE BY SIDE")
        print("  %-40s %12s %12s" % ("", "words", "closures"))
        for r in good:
            print("  %-40s %12s %12s"
                  % (os.path.basename(r["path"])[:40],
                     fmt(r.get("words")), fmt(r.get("closures"))))
        print()
        print("  A run showing (none) has no such table - that is not a zero.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
