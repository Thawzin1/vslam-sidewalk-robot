#!/usr/bin/env python3
"""db_check.py <database.db> - after-drive integrity check of a map database (read-only).
Added 2026-09-26 with DbSqlite3/Synchronous=0: with saves no longer confirmed by the disk, a hang or power
loss mid-save could damage the file, so every drive's database is checked once it is closed.
Plain terms: asks SQLite (the database library) to read every page and say whether the file is whole.
Prints PASS/FAIL and exits 0/1. Opens the file read-only; never writes.
"""
import sqlite3, sys, time
p = sys.argv[1]
t = time.time()
c = sqlite3.connect("file:%s?mode=ro" % p, uri=True)
r = [x[0] for x in c.execute("PRAGMA quick_check").fetchall()]
n = c.execute("SELECT COUNT(*) FROM Node").fetchone()[0]
print("%s %s: quick_check=%s, %d map nodes, %.1f s" % ("PASS" if r == ["ok"] else "FAIL", p, r[:3], n, time.time() - t))
sys.exit(0 if r == ["ok"] else 1)
