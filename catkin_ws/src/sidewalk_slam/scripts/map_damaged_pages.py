#!/usr/bin/env python3
"""map_damaged_pages.py - which TABLE owns each damaged page?

WHY

    An integrity check reports page numbers: "On tree page 1047212 cell 162:
    2nd reference to page 1059115". A page number on its own says nothing about
    what was lost. The same number could be a stored image, the pose graph, or
    an index.

    Plain terms: the check tells you which shelf collapsed. This tells you what
    was on the shelf.

    It matters here because three unrelated files - different sizes, four days
    apart - report the SAME COUNT of 398 damaged pages. A fault leaving a
    fixed-size footprint is a mechanism, not bad luck, and naming the table it
    lands in is the shortest route to identifying it.

HOW

    SQLite's dbstat virtual table lists every page with the table or index that
    owns it. Verified present on this build (ENABLE_DBSTAT_VTAB, SQLite 3.31.1).

    Reading dbstat walks the b-trees, so on a badly damaged file it can fail
    part way. That is reported rather than hidden - a partial map is still
    useful, and a total failure is itself a severity reading.

SAFETY

    Read-only, opened immutable. Do not run against a database a live run is
    writing to.

    usage:
      rosrun sidewalk_slam map_damaged_pages.py --db a.db --report full_check.txt
"""
from __future__ import print_function

import argparse
import collections
import os
import re
import sqlite3
import sys

PAGE_RE = re.compile(r"page (\d+)")


def damaged_pages_for(report_path, db_name):
    """Pull every page number the integrity report blamed for this database."""
    if not report_path or not os.path.exists(report_path):
        return None
    pages = set()
    in_file = False
    for line in open(report_path):
        if re.match(r"\s*(OK|DAMAGED)\s", line):
            in_file = db_name in line
            continue
        if in_file:
            for m in PAGE_RE.finditer(line):
                pages.add(int(m.group(1)))
    return pages


def owners(db_path, limit_pages=None):
    """page number -> owning table/index name, from dbstat."""
    out = {}
    partial = None
    try:
        con = sqlite3.connect("file:%s?immutable=1" % db_path, uri=True)
        cur = con.execute("SELECT name, pageno FROM dbstat")
        n = 0
        for name, pageno in cur:
            n += 1
            if limit_pages is None or pageno in limit_pages:
                out[pageno] = name
        con.close()
        return out, None, n
    except sqlite3.Error as exc:
        partial = str(exc)[:70]
        return out, partial, len(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--db", action="append", required=True)
    ap.add_argument("--report", default=None,
                    help="the full integrity_check output, to pull page numbers from")
    a = ap.parse_args()

    for db in a.db:
        name = os.path.basename(db)
        print("=" * 78)
        print("  %s" % name)
        if not os.path.exists(db):
            print("    MISSING")
            continue

        want = damaged_pages_for(a.report, name)
        if want is None:
            print("    no report given - mapping ALL pages by owner instead")
        elif not want:
            print("    the report blames no pages in this file (it may be clean,")
            print("    or the check aborted before naming any)")
        else:
            print("    %d distinct page numbers blamed by the integrity check"
                  % len(want))

        omap, err, seen = owners(db, want)
        if err:
            print("    dbstat FAILED PART WAY after %d pages: %s" % (seen, err))
            print("    (a partial map is still evidence; a total failure is a")
            print("     severity reading in itself)")
        if not omap:
            print("    nothing mapped")
            continue

        if want:
            tally = collections.Counter(omap.get(p, "UNMAPPED") for p in want)
            unmapped = sum(1 for p in want if p not in omap)
            print("    blamed pages by owning table:")
            for owner, n in tally.most_common():
                print("      %-34s %6d" % (owner, n))
            if unmapped:
                print("      (%d blamed pages are not in dbstat at all - they are"
                      % unmapped)
                print("       outside what SQLite believes the database contains)")
        else:
            tally = collections.Counter(omap.values())
            print("    all pages by owning table (top 12):")
            for owner, n in tally.most_common(12):
                print("      %-34s %8d pages" % (owner, n))
        print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
