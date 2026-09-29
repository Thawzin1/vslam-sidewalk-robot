#!/usr/bin/env python3
"""drive_media_recorder.py - save the live map page's pictures DURING a drive, then make the timelapse.

WHY (ENGINEERING_NOTES.md rule 22, 2026-09-25)
    Every drive stores its results: numbers, figures and a timelapse. Drive 3 had no timelapse, because nothing saved the live picture
    while it was being drawn; afterwards it can only be rebuilt from the map database, and a
    rebuilt one is not what the driver saw. This program saves what the live page showed, as
    it showed it.

    *Plain terms: every few seconds it takes a copy of the two map pictures on the live page
    (camera map and LiDAR map), and when the drive closes it glues them into one short video
    with ffmpeg (a standard video-making program).*

WHAT IT READS (read-only, from this Jetson; the same pictures the browser gets)
    http://127.0.0.1:8095/map.png          camera map (live_map_server.py)
    http://127.0.0.1:8095/peer/map.png     LiDAR panel, passed through from the robot
    http://127.0.0.1:8095/stats.json       tracking / map-correction text for the caption
    It never starts, stops or changes anything belonging to the drive. It talks to nothing
    but that page, and only by asking for pictures - the same as one more browser tab, asking
    once every 3 s instead of once a second.

WHAT IT WRITES
    ~/.run_records/<run>/media/cam/NNNNNN.png      camera map, as served
    ~/.run_records/<run>/media/lidar/NNNNNN.png    LiDAR panel, as served (absent = offline then)
    ~/.run_records/<run>/media/index.csv           one row per frame (a small table file): Hamilton
                                                   time, what the page answered, how many bytes
    ~/.run_records/<run>/media/frames_<YYYYmmdd_HHMMSS>/NNNNNN.png
                                                   side-by-side frames with a caption, one NEW folder
                                                   per build (Hamilton time) - nothing is ever deleted
                                                   by this program (ENGINEERING_NOTES.md rule 9); the log prints
                                                   an rm for older frame folders for the user to run
    ~/.run_records/<run>/media/<run>_timelapse.mp4 the video (H.264, the usual video compression,
                                                   10 frames a second). A rebuild first renames an
                                                   existing video to <run>_timelapse.before_<time>.mp4
    ~/.run_records/<run>/media/recorder.log
    ~/.run_records/<run>/media/recorder.pid        the process number (pid) while it runs; overwritten
                                                   with "exited <Hamilton time>" when it ends, so a
                                                   stale number is never mistaken for this program
    ~/jobs/<run>_media.progress                    one line for the jobs page (:8096)
    All of these follow $HOME, so a test run with HOME=<scratch folder> writes nothing into the
    real ~/jobs or ~/.run_records (tests MUST do that: the public-page pusher reads ~/jobs).
    About 80 KB per camera picture and 150 KB per built frame: a 20-minute drive at 3 s is
    400 frames, roughly 100 MB with both panels (measured: 41 frames = 9.4 MB, camera only).

WHEN IT STOPS (whichever comes first), and then builds the video
    1. the stop-on-"park" helper's line (~/jobs/<run>_autostop.progress, written after this
       recorder started) says "complete" - the drive has closed; or it says "FAILED" AND the
       mapping program is no longer running. A "FAILED" line while the mapping still runs
       (auto_stop_mapping.py: "expected one mapping process, found 2; stopped nothing", or
       "has not exited after N min; NOT killed") is logged and recording CONTINUES - the drive
       is not over, and rule 2 below ends the recording when the mapping really goes;
    2. the mapping program (a process named exactly "rtabmap", matched on the whole name, never
       a substring - ENGINEERING_NOTES.md rule 8) was seen and has then been gone for 120 s;
    3. a stop signal (SIGINT or SIGTERM - the polite "please stop" messages a program can be
       sent). Use:  python3 drive_media_recorder.py --run <run> --stop
       which reads media/recorder.pid, checks that the process with that number really is this
       program for this run (its command line has drive_media_recorder.py as a whole argument,
       by base name, and "--run <run>"), and only then sends SIGINT. Never kill a bare number
       from the file by hand: once the recorder has ended, the number can belong to anything;
    4. --max-minutes (default 150) as a last resort, or --duration-s for a test.

WHY IT SHOULD NOT HURT THE MAPPING (and the small costs it does have)
    - nice 19 and idle-class disk priority (it only gets the processor and the disk when
      nothing else wants them);
    - the costs it DOES cause elsewhere, said plainly: each /map.png request makes
      live_map_server.py (normal priority, nice 0) draw a fresh picture - 36-42 ms per picture
      on the small bench map (measured 25 Sept), growing with the map; and each /peer/map.png
      request makes the page fetch the robot's picture over WiFi, the same link the FUSION
      bridge uses. At one request of each every 3 s both are small (about 1-2 % of one core,
      one extra picture per 3 s on the link), but they are not zero;
    - every request has a short timeout (4 s), so a stalled page only costs missed frames,
      recorded as such in index.csv; the loop never waits longer than that;
    - it never loads a browser; the pictures are the page's own small PNGs;
    - the video is built only AFTER the drive has closed (or on demand with --build);
    - any error while saving a frame (disk full, page gone) is logged and skipped; the
      recorder keeps going and says so on its progress line.

USAGE (on the Jetson)
    started by start_drive.sh step 4b; by hand:
      setsid nohup nice -n 19 python3 drive_media_recorder.py --run s2_static_04 \
          > /dev/null 2>&1 < /dev/null &
    build (or rebuild) the video only, from frames already saved:
      nice -n 19 python3 drive_media_recorder.py --run s2_static_04 --build
    stop it early (safely - checks the process number first):
      python3 drive_media_recorder.py --run s2_static_04 --stop
    test it WITHOUT touching the real ~/jobs (which the public-page pusher reads):
      HOME=<scratch folder> python3 drive_media_recorder.py --run t1 --duration-s 30

Python 3.8; standard library + numpy + cv2 (both already used by live_map_server.py).
"""
import argparse
import csv
import json
import os
import signal
import subprocess
import sys
import time
import urllib.error
import urllib.request

HOME = os.path.expanduser("~")
JOBS = os.path.join(HOME, "jobs")
os.environ["TZ"] = "America/Toronto"          # every time written is Hamilton time
time.tzset()

STOP = {"sig": None}
PANEL = 800          # each panel fitted into PANEL x PANEL
BAR = 64             # caption bar height (px); composite is 2*PANEL x (PANEL+BAR), even sizes


def hm(t=None):
    return time.strftime("%Y-%m-%d %H:%M:%S %Z", time.localtime(t))


class Out(object):
    def __init__(self, run):
        self.run = run
        self.rec = os.path.join(HOME, ".run_records", run)
        self.media = os.path.join(self.rec, "media")
        self.cam = os.path.join(self.media, "cam")
        self.lidar = os.path.join(self.media, "lidar")
        self.index = os.path.join(self.media, "index.csv")
        self.video = os.path.join(self.media, "%s_timelapse.mp4" % run)
        self.logf = os.path.join(self.media, "recorder.log")
        self.pidf = os.path.join(self.media, "recorder.pid")
        self.progress = os.path.join(JOBS, "%s_media.progress" % run)

    def log(self, msg):
        line = "%s %s" % (hm(), msg)
        print(line, flush=True)
        try:
            with open(self.logf, "a") as f:
                f.write(line + "\n")
        except OSError:
            pass

    def prog(self, line):
        try:
            os.makedirs(JOBS, exist_ok=True)
            tmp = self.progress + ".tmp"
            with open(tmp, "w") as f:
                f.write(line + "\n")
            os.replace(tmp, self.progress)
        except OSError:
            pass


def be_gentle():
    try:
        cur = os.nice(0)
        if cur < 19:
            os.nice(19 - cur)
    except OSError:
        pass
    try:   # idle disk class: allowed for one's own process without sudo
        subprocess.call(["ionice", "-c3", "-p", str(os.getpid())],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=5)
    except Exception:
        pass


def get(url, timeout):
    """(status, body) - never raises; status 0 = no answer (timeout, refused)."""
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            return r.status, r.read()
    except urllib.error.HTTPError as e:
        try:
            return e.code, e.read()
        except Exception:
            return e.code, b""
    except Exception as e:
        return 0, str(e).encode()[:120]


def mapping_running(name="rtabmap"):
    """True if a process whose name is EXACTLY `name` exists (whole-name match, rule 8)."""
    for d in os.listdir("/proc"):
        if not d.isdigit():
            continue
        try:
            with open("/proc/%s/comm" % d) as f:
                if f.read().strip() == name:
                    return True
        except OSError:
            continue
    return False


def autostop_line(run, since):
    """("complete"|"FAILED", line) from the stop-on-'park' helper, written after `since`; else None."""
    p = os.path.join(JOBS, "%s_autostop.progress" % run)
    try:
        if os.path.getmtime(p) < since:
            return None
        with open(p, errors="replace") as f:
            line = f.readline().strip()
    except OSError:
        return None
    if " complete" in line:
        return "complete", line
    if "FAILED" in line:
        return "FAILED", line
    return None


def run_matches(pid, run):
    """True if process `pid` is drive_media_recorder.py for `run`: whole arguments, base name
    compared, never a substring (ENGINEERING_NOTES.md rule 8)."""
    try:
        with open("/proc/%d/cmdline" % pid, "rb") as f:
            argv = [a.decode("utf-8", "replace") for a in f.read().split(b"\0") if a]
    except (OSError, ValueError):
        return False
    if not any(os.path.basename(a) == "drive_media_recorder.py" for a in argv):
        return False
    return any(argv[i] == "--run" and argv[i + 1] == run for i in range(len(argv) - 1)) \
        or ("--run=" + run) in argv


def stop_recorder(o):
    """--stop: SIGINT the recorder for this run, only after checking the pid really is it."""
    try:
        with open(o.pidf) as f:
            txt = f.read().strip()
    except OSError:
        print("no %s - nothing to stop" % o.pidf)
        return 1
    if not txt.isdigit():
        print("recorder not running (%s says: %s) - nothing sent" % (o.pidf, txt))
        return 1
    pid = int(txt)
    if pid in (os.getpid(), os.getppid()) or not run_matches(pid, o.run):
        print("pid %d is NOT drive_media_recorder.py --run %s (ended, number reused?) - nothing sent"
              % (pid, o.run))
        return 1
    os.kill(pid, signal.SIGINT)
    print("sent SIGINT to pid %d (drive_media_recorder.py --run %s); it will build the video and "
          "write its final line to %s" % (pid, o.run, o.progress))
    return 0


def highest_frame(o):
    """The highest frame number already used for this run: camera pictures, LiDAR pictures and
    index.csv rows (malformed rows ignored). 0 if none."""
    n = 0
    for d in (o.cam, o.lidar):
        try:
            names = os.listdir(d)
        except OSError:
            continue
        for name in names:
            if name[:6].isdigit():
                n = max(n, int(name[:6]))
    try:
        with open(o.index, newline="", errors="replace") as f:
            # skip lines holding NUL bytes (a crash can leave them; csv would stop outright -
            # review of round 3, defect 1)
            for row in csv.reader(line for line in f if "\0" not in line):
                if row and row[0].strip().isdigit():
                    n = max(n, int(row[0]))
    except (OSError, csv.Error):
        pass
    return n


def read_index(o):
    """(rows, skipped): the index.csv rows that are complete and readable, in frame order with
    duplicates dropped (the first row for a number wins), and how many rows were skipped. A row
    is skipped if it is cut off (a crash mid-write), has an unreadable frame number or time, or
    repeats a frame number (repair round 3, defect 4)."""
    rows, skipped, seen = [], 0, set()
    nul = [0]

    def clean(f):                     # a crash can also leave NUL bytes, which stop csv outright
        for line in f:
            if "\0" in line:
                nul[0] += 1
                continue
            yield line
    try:
        with open(o.index, newline="", errors="replace") as f:
            for r in csv.DictReader(clean(f)):
                try:
                    if None in r or any(v is None for v in r.values()):
                        raise ValueError("wrong number of fields")
                    i = int(r["frame"])
                    t = float(r["epoch_s"])
                    if i <= 0 or t != t:
                        raise ValueError("bad value")
                except (KeyError, TypeError, ValueError):
                    skipped += 1
                    continue
                if i in seen:
                    skipped += 1
                    continue
                seen.add(i)
                r["_frame"], r["_t"] = i, t
                rows.append(r)
    except (OSError, csv.Error):
        pass
    rows.sort(key=lambda r: r["_frame"])
    return rows, skipped + nul[0]


def on_signal(sig, _f):
    STOP["sig"] = sig


# ------------------------------------------------------------------ record
def record(args, o):
    for d in (o.cam, o.lidar):
        os.makedirs(d, exist_ok=True)
    with open(o.pidf, "w") as f:
        f.write("%d\n" % os.getpid())
    STOP["pidf"] = o.pidf
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    t0 = time.time()
    new_index = not os.path.exists(o.index) or os.path.getsize(o.index) == 0
    # Restarted for the same run: continue AFTER the highest frame number used anywhere - the
    # camera pictures, the LiDAR pictures AND the index.csv rows. A row whose camera picture is
    # missing (page did not answer) still used its number; restarting from the highest camera
    # picture alone would reuse that number and give two index rows the same frame (repair
    # round 3, defect 3).
    n = highest_frame(o)
    if not new_index:
        # a crash mid-write can leave the last row cut off with no line end; start the new rows
        # on a fresh line so the first of them is not glued onto the broken one
        try:
            with open(o.index, "rb") as f:
                f.seek(-1, os.SEEK_END)
                if f.read(1) != b"\n":
                    with open(o.index, "a") as g:
                        g.write("\n")
        except OSError:
            pass
    fidx = open(o.index, "a", newline="")
    w = csv.writer(fidx)
    if new_index:
        w.writerow(["frame", "hamilton_time", "epoch_s", "cam_http", "cam_bytes",
                    "lidar_http", "lidar_bytes", "tracking", "corrections", "note"])
    o.log("start pid %d run %s every %.1f s from %s (nice %d)"
          % (os.getpid(), o.run, args.interval, args.local, os.nice(0)))
    cam_ok = lidar_ok = miss = errs = 0
    seen_mapping = False
    gone_since = None
    why = None
    last_ok = None
    failed_seen = None          # a FAILED line seen while the mapping was still running
    while True:
        tick = time.time()
        n += 1
        note = ""
        cs, cb = get(args.local + "/map.png", args.timeout)
        if cs == 0:
            # the page did not answer at all (stalled or down): do not spend two more timeouts
            # on it this round - a stall then costs one timeout per frame, not three
            ls, lb, ss, sb = 0, b"skipped: page not answering", 0, b""
        else:
            ls, lb = get(args.local + "/peer/map.png", args.timeout)
            ss, sb = get(args.local + "/stats.json", args.timeout)
        track = corr = ""
        if ss == 200:
            try:
                st = json.loads(sb.decode("utf-8"))
                track = str(st.get("tracking", ""))[:90]
                corr = str(st.get("last map correction", ""))[:90]
            except Exception:
                pass
        try:
            if cs == 200 and cb[:8] == b"\x89PNG\r\n\x1a\n":
                with open(os.path.join(o.cam, "%06d.png" % n), "wb") as f:
                    f.write(cb)
                cam_ok += 1
                last_ok = tick
            else:
                miss += 1
            if ls == 200 and lb[:8] == b"\x89PNG\r\n\x1a\n":
                with open(os.path.join(o.lidar, "%06d.png" % n), "wb") as f:
                    f.write(lb)
                lidar_ok += 1
        except OSError as e:                      # disk full etc: note it, keep going
            errs += 1
            note = "write error: %s" % e
            if errs in (1, 10, 100):
                o.log("frame %d %s" % (n, note))
        try:
            w.writerow([n, hm(tick), "%.3f" % tick, cs, len(cb) if cs == 200 else 0,
                        ls, len(lb) if ls == 200 else 0, track, corr, note])
            fidx.flush()
        except OSError:
            pass

        # ---- should it stop? ------------------------------------------------------
        el = time.time() - t0
        if STOP["sig"] is not None:
            why = "signal %s" % STOP["sig"]
        elif args.duration_s and el >= args.duration_s:
            why = "test duration %.0f s reached" % args.duration_s
        elif el >= args.max_minutes * 60:
            why = "backstop: %.0f min" % args.max_minutes
        else:
            running = mapping_running(args.mapping_name)
            got = autostop_line(o.run, t0)
            if got and got[0] == "complete":
                why = "the drive closed: " + got[1][:160]
            elif got and got[0] == "FAILED" and not running:
                why = "the drive closed (helper FAILED, mapping not running): " + got[1][:160]
            elif got and got[0] == "FAILED" and got[1] != failed_seen:
                failed_seen = got[1]
                o.log("helper says FAILED but the mapping (%s) is STILL RUNNING - keep recording: %s"
                      % (args.mapping_name, got[1][:160]))
            if not why and not args.no_drive_watch:
                if running:
                    seen_mapping, gone_since = True, None
                elif seen_mapping:
                    gone_since = gone_since or time.time()
                    if time.time() - gone_since >= args.gone_s:
                        why = "the mapping program (%s) has been gone for %.0f s" % (
                            args.mapping_name, args.gone_s)
        o.prog("MEDIA %s recording  frame %d  camera %d  LiDAR %d  missed %d%s  last ok %s  %.0fs"
               % (o.run, n, cam_ok, lidar_ok, miss, ("  WRITE ERRORS %d" % errs) if errs else "",
                  time.strftime("%H:%M:%S", time.localtime(last_ok)) if last_ok else "never", el))
        if why:
            break
        end = tick + args.interval
        while STOP["sig"] is None and time.time() < end:
            time.sleep(min(0.25, max(0.0, end - time.time())))
    fidx.close()
    o.log("recording stopped (%s): last frame number %d, this session camera %d, LiDAR %d, missed %d, write errors %d"
          % (why, n, cam_ok, lidar_ok, miss, errs))
    return why


# ------------------------------------------------------------------ build
def fit(img, box):
    import cv2
    import numpy as np
    out = np.full((box, box, 3), 40, np.uint8)
    if img is None:
        return out, False
    h, w = img.shape[:2]
    s = min(box / float(w), box / float(h))
    nw, nh = max(1, int(round(w * s))), max(1, int(round(h * s)))
    img = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_AREA if s < 1 else cv2.INTER_NEAREST)
    y, x = (box - nh) // 2, (box - nw) // 2
    out[y:y + nh, x:x + nw] = img[:, :, :3]
    return out, True


def build(args, o):
    import cv2
    import numpy as np
    rows, skipped = read_index(o)
    if skipped:
        o.log("build: skipped %d malformed index.csv row(s) (cut off, unreadable or repeated frame "
              "number); the video uses the %d good rows" % (skipped, len(rows)))
    rows = [r for r in rows if os.path.exists(os.path.join(o.cam, "%06d.png" % r["_frame"]))]
    if not rows:
        o.log("build: no camera frames saved - no video")
        o.prog("MEDIA %s FAILED - no camera frames were saved, no video" % o.run)
        return 1
    # a NEW folder per build: this program never deletes anything (ENGINEERING_NOTES.md rule 9)
    older = sorted(d for d in os.listdir(o.media)
                   if d == "frames" or d.startswith("frames_"))
    frames = os.path.join(o.media, "frames_" + time.strftime("%Y%m%d_%H%M%S"))
    k2 = 1
    while os.path.exists(frames):
        frames = os.path.join(o.media, "frames_%s_%d" % (time.strftime("%Y%m%d_%H%M%S"), k2))
        k2 += 1
    os.makedirs(frames)
    if older:
        o.log("build: older frame folders are kept; to free the space the USER may run (Jetson): "
              "rm -r " + " ".join("'%s'" % os.path.join(o.media, d) for d in older))
    t_start = rows[0]["_t"]
    font = cv2.FONT_HERSHEY_SIMPLEX
    tb = time.time()
    last_lidar = None
    for k, r in enumerate(rows, 1):
        i = r["_frame"]
        cam = cv2.imread(os.path.join(o.cam, "%06d.png" % i), cv2.IMREAD_COLOR)
        lp = os.path.join(o.lidar, "%06d.png" % i)
        lid = cv2.imread(lp, cv2.IMREAD_COLOR) if os.path.exists(lp) else None
        a, _ = fit(cam, PANEL)
        b, have = fit(lid, PANEL)
        if not have:
            cv2.putText(b, "LiDAR view offline at this moment", (150, PANEL // 2), font, 0.9,
                        (200, 200, 200), 2, cv2.LINE_AA)
        img = np.full((PANEL + BAR, 2 * PANEL, 3), 20, np.uint8)
        img[BAR:, :PANEL] = a
        img[BAR:, PANEL:] = b
        cv2.line(img, (PANEL, BAR), (PANEL, PANEL + BAR), (90, 90, 90), 2)
        t = r["_t"]
        el = t - t_start
        cv2.putText(img, "%s   %s   +%d:%02d   recorded live during the drive"
                    % (o.run, time.strftime("%H:%M:%S %Z", time.localtime(t)), el // 60, el % 60),
                    (12, 26), font, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(img, "camera map (left) | LiDAR view (right)   tracking: %s   map corrections: %s"
                    % (r.get("tracking", "")[:40], r.get("corrections", "")[:34]),
                    (12, 54), font, 0.5, (200, 200, 200), 1, cv2.LINE_AA)
        cv2.imwrite(os.path.join(frames, "%06d.png" % k), img)
        if k % 20 == 0 or k == len(rows):
            o.prog("MEDIA %s building frames %d/%d  %.0fs" % (o.run, k, len(rows), time.time() - tb))
    tmp = o.video + ".part.mp4"
    cmd = ["nice", "-n", "19", "ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
           "-framerate", str(args.fps), "-i", os.path.join(frames, "%06d.png"),
           "-frames:v", str(len(rows)),
           "-c:v", "libx264", "-preset", "veryfast", "-crf", "23", "-pix_fmt", "yuv420p",
           "-threads", "2", "-movflags", "+faststart", tmp]
    o.prog("MEDIA %s encoding %d/%d frames  %.0fs" % (o.run, len(rows), len(rows), time.time() - tb))
    r = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=1800)
    if r.returncode != 0 or not os.path.exists(tmp):
        o.log("build: ffmpeg failed (%d): %s" % (r.returncode, r.stdout.decode(errors="replace")[-400:]))
        o.prog("MEDIA %s FAILED - ffmpeg exit %d, frames kept in %s" % (o.run, r.returncode, frames))
        return 1
    if os.path.exists(o.video):                   # keep the previous video, never overwrite it
        kept = o.video[:-len(".mp4")] + ".before_%s.mp4" % time.strftime("%Y%m%d_%H%M%S")
        os.rename(o.video, kept)
        o.log("build: previous video kept as %s" % kept)
    os.replace(tmp, o.video)
    dur = len(rows) / float(args.fps)
    o.log("build: %s - %d frames at %d a second = %.1f s of video (%.1f MB)"
          % (o.video, len(rows), args.fps, dur, os.path.getsize(o.video) / 1e6))
    o.prog("MEDIA %s complete - %d frames, %.1f s video %s%s  %.0fs"
           % (o.run, len(rows), dur, os.path.basename(o.video),
              ("  (%d malformed index rows skipped)" % skipped) if skipped else "", time.time() - tb))
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--run", required=True)
    ap.add_argument("--local", default="http://127.0.0.1:8095")
    ap.add_argument("--interval", type=float, default=3.0, help="seconds between frames")
    ap.add_argument("--timeout", type=float, default=4.0, help="per request, seconds")
    ap.add_argument("--fps", type=int, default=10, help="video frames per second")
    ap.add_argument("--max-minutes", type=float, default=150)
    ap.add_argument("--duration-s", type=float, default=0, help="tests: stop after this long")
    ap.add_argument("--no-drive-watch", action="store_true",
                    help="tests: do not stop when the mapping program disappears")
    ap.add_argument("--mapping-name", default="rtabmap",
                    help="tests only: the exact process name that counts as 'the mapping'")
    ap.add_argument("--gone-s", type=float, default=120.0,
                    help="stop after the mapping has been gone this long (seconds)")
    ap.add_argument("--stop", action="store_true",
                    help="send SIGINT to the recorder of --run, after checking its pid is really it")
    ap.add_argument("--build", action="store_true", help="only build the video from saved frames")
    ap.add_argument("--no-build", action="store_true", help="record only")
    args = ap.parse_args()
    if not args.run.replace("_", "").replace("-", "").isalnum():
        raise SystemExit("run id must be letters, digits, _ or -")
    o = Out(args.run)
    if args.stop:
        return stop_recorder(o)
    be_gentle()
    os.makedirs(o.media, exist_ok=True)
    if args.build:
        return build(args, o)
    try:
        try:
            record(args, o)
        except Exception as e:
            # never die silently: the jobs page must say the recorder stopped (review of round 3,
            # defect 2). The drive itself is unaffected.
            o.log("recorder crashed: %s: %s" % (type(e).__name__, e))
            o.prog("MEDIA %s FAILED - recorder crashed (%s: %s); frames so far kept, "
                   "build them with --build" % (o.run, type(e).__name__, e))
            return 1
        if args.no_build:
            o.prog("MEDIA %s complete - recorded, video not built (--no-build)" % o.run)
            return 0
        try:
            return build(args, o)
        except Exception as e:
            o.log("build crashed: %s: %s" % (type(e).__name__, e))
            o.prog("MEDIA %s FAILED - build crashed (%s); frames kept, rerun with --build" % (o.run, e))
            return 1
    finally:
        # never leave a bare process number behind: it could be reused by another program
        if STOP.get("pidf"):
            try:
                with open(STOP["pidf"], "w") as f:
                    f.write("exited %s\n" % hm())
            except OSError:
                pass


if __name__ == "__main__":
    sys.exit(main())
