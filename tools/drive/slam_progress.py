#!/usr/bin/env python3
"""slam_progress.py - make a mapping run visible on the jobs page while it drives.

WHY
    Every test in this project has to be watchable live from a browser, from
    anywhere (ENGINEERING_NOTES.md rule 13). The jobs page at the jobs page (JOBS_PAGE_URL)
    discovers jobs by itself: any file ending `.progress` containing one short
    line of plain text becomes a card.

    A mapping run currently writes `monitor.csv` at 1 Hz and nothing else, so it
    is invisible on that page. The user is often away from the machine, on a
    phone, twenty minutes into a drive, and needs to know whether it is
    advancing, stuck, or dead.

WHAT IT WRITES
    One line, rewritten every few seconds, next to the run's output:

        S2_STATIC_01  399 samples  closures 78  quality 92  1180s

    and, when the run dies or the driver pauses too long:

        S2_STATIC_01  STALLED - no new data for 63s  (last: 399 samples)  1243s

    A mapping run has no fixed end - the drive finishes when the driver stops -
    so there is no `N/M` to draw a bar from, and the page shows the line as-is.
    That is the documented behaviour for jobs without a known total.

HOW IT DECIDES THE RUN IS DEAD
    By the FILE, never by looking for a process. A check that asks whether some
    command line *contains* a name matches itself, and has reported dead jobs as
    alive four times in this project - and on 2026-09-23 killed the shell that
    ran it. So: if `monitor.csv` has not grown for longer than `--stall-after`
    seconds, the line says STALLED and keeps saying it. The page then shows the
    run as stalled rather than quietly leaving the last good line on screen,
    where a dead run and a slow one look identical.

  usage, on the Jetson, in its own terminal alongside the run:
    python3 slam_progress.py --run s2_static_01
    python3 slam_progress.py --run s2_static_01 --interval 10 --stall-after 45
"""
from __future__ import print_function

import argparse
import math
import os
import signal
import sys
import time

DEFAULT_RECORDS = os.path.expanduser(os.environ.get("RECORDS_DIR", "~/.run_records"))


def read_monitor(path):
    """Last usable row of monitor.csv, plus the row count.

    The file is being appended to while we read it, so the final line can be a
    half-written one. Parse from the end and take the first line that is whole.
    """
    try:
        with open(path) as fh:
            lines = fh.readlines()
    except (IOError, OSError):
        return None, 0
    if len(lines) < 2:
        return None, max(0, len(lines) - 1)
    header = [c.strip() for c in lines[0].split(",")]
    for line in reversed(lines[1:]):
        parts = line.strip().split(",")
        if len(parts) != len(header):
            continue          # torn final line
        return dict(zip(header, parts)), len(lines) - 1
    return None, len(lines) - 1


def camera_note(run_dir):
    """The camera guard's state (camera_guard.py, 2026-09-26), as words for this line.

    Drive 5's camera crashed 12 minutes in and this line kept saying "running" for
    9 minutes, because the map's own counters keep ticking without pictures. The
    guard rewrites camera_guard.json every 2 s; a file older than 15 s is not
    believed. Returns (prefix, suffix): prefix "!! " makes the jobs page show a
    red card.
    """
    import json
    path = os.path.join(run_dir, "camera_guard.json")
    try:
        st = os.stat(path)
        with open(path) as fh:
            d = json.load(fh)
    except (IOError, OSError, ValueError):
        return "", ""
    if time.time() - st.st_mtime > 15:
        return "", "camera guard silent %ds" % (time.time() - st.st_mtime)
    try:    # a malformed file (a list, a non-number) must never stop this line (review item 5)
        state = d.get("state")
        if state in ("down", "gave_up"):
            return "!! ", "CAMERA DOWN since %s%s" % (
                d.get("down_since_hms") or "?", " - GAVE UP" if state == "gave_up" else "")
        if d.get("restarts"):
            return "", "CAMERA RESTARTED %d" % int(d["restarts"])
    except Exception:
        return "", "camera guard state unreadable"
    return "", ""


def num(row, key):
    try:
        return float(row.get(key, "") or "")
    except (TypeError, ValueError):
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", required=True, help="run id, e.g. s2_static_01")
    ap.add_argument("--records", default=DEFAULT_RECORDS,
                    help="where the run's folder lives (default ~/.run_records)")
    ap.add_argument("--jobs-dir", default=os.path.expanduser(os.environ.get("JOBS_DIR", "~/jobs")),
                    help="the folder the jobs page actually scans (default "
                         "~/jobs). VERIFIED against jobs_dashboard.py on "
                         "2026-09-23: it walks ~/closeout_5m, ~/jobs, "
                         "~/zedx_vs_lidar_data and /tmp, three deep - and NOT "
                         "~/.run_records. A progress file written only beside "
                         "the run would never appear on the page.")
    ap.add_argument("--interval", type=float, default=15.0,
                    help="seconds between rewrites (default 15; rule 13 asks "
                         "for at least every ~30)")
    ap.add_argument("--expect-minutes", type=float, default=20.0,
                    help="roughly how long the drive is planned to take "
                         "(default 20). Used ONLY to emit an N/M counter, "
                         "because the jobs page cannot judge a job that has "
                         "none: its `moves` counter only increments when "
                         "`cur is not None or moves > 0`, which never "
                         "bootstraps for a counter-less job, so such a job "
                         "sits in `starting` for ever and is never called "
                         "running or stalled. Verified against "
                         "jobs_dashboard.py and /data.json on 2026-09-23.")
    ap.add_argument("--stall-after", type=float, default=60.0,
                    help="seconds without monitor.csv growing before the line "
                         "says STALLED (default 60)")
    args = ap.parse_args()

    run_dir = os.path.join(args.records, args.run)
    monitor = os.path.join(run_dir, "monitor.csv")

    # Two copies, deliberately. The one in ~/jobs is what the page finds; the
    # one beside the run is provenance, so a finished run carries its own last
    # reported state without depending on a folder that gets tidied.
    # ~/jobs is chosen over the other scanned roots because ~/zedx_vs_lidar_data
    # belongs to the camera depth study, and SLAM writes nothing there (rule 16).
    progress = os.path.join(args.jobs_dir, "%s.progress" % args.run)
    progress_beside = os.path.join(run_dir, "%s.progress" % args.run)

    for d in (run_dir, args.jobs_dir):
        if not os.path.isdir(d):
            os.makedirs(d)

    label = args.run.upper()
    t0 = time.time()
    last_rows = -1
    last_growth = time.time()
    started = False

    def write(line):
        for target in (progress, progress_beside):
            tmp = target + ".tmp"
            with open(tmp, "w") as fh:
                fh.write(line + "\n")
            os.replace(tmp, target)   # never leave a half-written line on show

    # A watcher that is killed rather than interrupted must not leave its last
    # line frozen on the page: a job stuck at "3/20 min waiting" looks alive
    # the KeyboardInterrupt handler never ran, and the stale line stayed up.
    class _Leave(Exception):
        pass

    def _on_signal(signum, frame):
        raise _Leave(signal.Signals(signum).name)

    for sig in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT):
        try:
            signal.signal(sig, _on_signal)
        except (ValueError, OSError):
            pass        # not available in this context; not fatal

    write("%s  0/%d min  waiting for the run to start  0s"
          % (label, max(1, int(round(args.expect_minutes)))))
    print("  jobs page: %s" % progress)
    print("  beside run: %s" % progress_beside)
    print("  watch at the jobs page (JOBS_PAGE_URL)")

    try:
        while True:
            now = time.time()
            elapsed = int(now - t0)
            row, rows = read_monitor(monitor)

            if rows > last_rows:
                last_rows = rows
                last_growth = now
                started = started or rows > 0

            # N/M in minutes. Capped one below the total while the run is
            # alive, and the word "running" is always present, so the page can
            # never decide by itself that the drive has finished - only the
            # driver knows that.
            total_min = max(1, int(round(args.expect_minutes)))
            cur_min = min(int(elapsed // 60), total_min - 1)

            if not started:
                write("%s  %d/%d min  waiting for the run to start  %ds"
                      % (label, cur_min, total_min, elapsed))
            elif now - last_growth > args.stall_after:
                # "STALLED" is in the page's failure vocabulary, so this shows
                # as an alarm even though the page's own stall detection can
                # never fire for this job.
                write("%s  %d/%d min  STALLED - no new data for %ds "
                      "(last: %d samples)  %ds"
                      % (label, cur_min, total_min, int(now - last_growth),
                         last_rows, elapsed))
            else:
                bits = ["%s  %d/%d min  running" % (label, cur_min, total_min)]
                if row:
                    q = num(row, "quality")
                    lc = num(row, "accepted_lc")
                    rej = num(row, "rejected_lc")
                    res = num(row, "resets")
                    bits.append("%d samples" % last_rows)
                    # -1 is the monitor's "not known yet" sentinel, written
                    # before the mapper has reported anything. Showing it as
                    # "closures -1" reads like a failure; it is just early.
                    if lc is not None and lc >= 0:
                        bits.append("closures %d" % int(lc))
                    if rej is not None and rej > 0:
                        bits.append("rejected %d" % int(rej))
                    if res is not None and res > 0:
                        # Resets hide distance: RTAB-Map discards every frame
                        # during the outage, so the link across the gap omits
                        # the metres travelled. Always visible, never buried.
                        bits.append("RESETS %d" % int(res))
                    if q is not None and q >= 0:
                        bits.append("quality %d" % int(q))
                else:
                    bits.append("%d samples" % last_rows)
                pre, cam = camera_note(run_dir)
                if cam:
                    bits.append(cam)
                bits.append("%ds" % elapsed)
                write(pre + "  ".join(bits))

            time.sleep(args.interval)
    except (KeyboardInterrupt, _Leave) as why:
        # "stopped" is in the page's FAILURE vocabulary, deliberately - the
        # watchdog uses it to mean alarm. Say "complete" for a clean finish so
        # a normal end does not render as a red job. The sample count is in the
        # line so a watcher that ended before the run did is visible as such.
        how = "by hand" if isinstance(why, KeyboardInterrupt) else str(why)
        write("%s  watcher complete (%s) after %ds  (%d samples)"
              % (label, how, int(time.time() - t0), last_rows))
        print("\n  watcher ended: %s" % how)
        return 0


if __name__ == "__main__":
    sys.exit(main())
