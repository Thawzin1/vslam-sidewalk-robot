#!/usr/bin/env python3
"""slam_live_push.py - write the drive's live map for the project website, every 2 s.

WHY THIS EXISTS
    The drive's live map (live_map_server.py, port 8095 on the Jetson) is a page on the
    Jetson itself. The project website (live_site_server.py, beside this file) shows the same
    map and numbers, plus the status lines of the drive's helper programs and the robot's
    LiDAR panel. Every 2 s this program reads the local page and writes one JSON snapshot
    file, which the website hands to whoever has the link.

        Jetson
        live_map_server.py :8095
             |  (127.0.0.1 only)
        slam_live_push.py --writes every 2 s--> ~/jobs/live_snapshot.json
                                                     |
        live_site_server.py :8097  --/api/state--> the page (every 2 s)

WHAT IT READS (all local, all read-only)
    http://127.0.0.1:8095/stats.json        tracking, closures, robot position
    http://127.0.0.1:8095/map.png           the camera map picture, dot and heading drawn in
    http://127.0.0.1:8095/peer/stats.json   the robot's LiDAR panel (every 2nd push;
    http://127.0.0.1:8095/peer/map.png       every 5th while it is offline)
    ~/jobs/<run>.progress, <run>_autostop, _bridge, _ekf_inputs, _turns, _media .progress
        for the NEWEST DRIVE OR REHEARSAL (see RUN_RE below: test, bench and replay runs are
        never followed; between drives it stays on the last drive)

    It never starts, stops or changes anything that belongs to the drive.

LIVENESS
    ~/jobs/slam_live_push.progress - one line, rewritten after every push, picked up by the
    jobs page (port 8096). A final line is written on a clean stop.

START / STOP (on the Jetson)
    TZ=America/Toronto setsid nohup nice -n 19 python3 $TOOLS_DIR/slam_live_push.py \
        >> ~/jobs/slam_live_push.log 2>&1 < /dev/null &
    kill -INT <pid>        # pid is in ~/jobs/slam_live_push.pid; sends one last
                           # "stopped on purpose" snapshot so the page says so

Python 3.8, standard library only.
"""
import base64
import glob
import hashlib
import json
import os
import re
import signal
import socket
import sys
import time
import urllib.error
import urllib.parse
import urllib.request

# every time this program prints or writes is Hamilton time, even when started without TZ
# (the system clock is UTC; ENGINEERING_NOTES.md Hamilton-time rule)
os.environ["TZ"] = "America/Toronto"
time.tzset()

# USB storage, live occupancy map)" and "include the robot's pc and its card"). jetson_status.py
# sits beside this file; if it is missing or breaks, the map is still sent without it.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    import jetson_status
except Exception as _exc:                          # never let the status add-on stop the map
    jetson_status = None
    _JETSON_STATUS_ERR = "%s: %s" % (type(_exc).__name__, _exc)

LOCAL = os.environ.get("SLAM_LIVE_LOCAL", "http://127.0.0.1:8095")
JOBS = os.path.expanduser(os.environ.get("JOBS_DIR", "~/jobs"))
PROGRESS = os.path.join(JOBS, "slam_live_push.progress")
PIDFILE = os.path.join(JOBS, "slam_live_push.pid")

INTERVAL_S = 2.0          # while the local page answers
INTERVAL_DOWN_S = 2.0     # project rule 2026-09-27: every stream every 2 s, drive or not
MAX_PAYLOAD = 700000      # bytes of JSON; the pictures are dropped (and the page told) above this
PEER_EVERY = 2            # LiDAR panel every Nth push while it answers ...
PEER_EVERY_OFF = 5        # ... and every Nth while it does not (each miss costs ~1.5 s of waiting)
LINE_MAX = 400            # characters kept from each status line

# which status files belong to a run: <run><suffix>.progress
SUFFIXES = {"autostop": "_autostop", "bridge": "_bridge",
            "ekf_inputs": "_ekf_inputs", "turns": "_turns", "media": "_media"}
# status files that decide WHICH run is newest. "_media" is forwarded but NOT used to choose:
# rebuilding an old drive's timelapse (drive_media_recorder.py --build) rewrites that drive's
# _media line, and must not pull the public page back to the old drive.
CHOOSE_SUFFIXES = [""] + [v for k, v in SUFFIXES.items() if k != "media"]

STOP = {"sig": None}


def log(msg):
    print(time.strftime("%Y-%m-%d %H:%M:%S %Z"), msg, flush=True)


def local_get(path, timeout):
    """(status, headers, body) from the local page; HTTP errors are answers, not crashes."""
    try:
        with urllib.request.urlopen(LOCAL + path, timeout=timeout) as r:
            return r.status, r.headers, r.read()
    except urllib.error.HTTPError as e:
        return e.code, e.headers, e.read()


# kind, and showed the bench test d4a1_old instead of a drive).
# Only real drives and rehearsals, named by start_drive.sh's run id:
#   s2_static_NN, s2_dynamic_NN (optionally one letter, e.g. s2_static_04b)   drives
#   s2_fusion_T<N>, s2_fusion_T<N><letter> (s2_fusion_T4, s2_fusion_T4b)      rehearsals
# The whole name must match: replays and copies of a drive (s2_static_02_T2copy,
# s2_static_01_lidar_replay), checks (s2_startcheck, s2_dashcheck, s2_sigterm_test) and bench
# or replay runs (d4a1_old, d4b_val_r3b_x1, fusion_r3_T3_hold, t2_page_replay) never match.
# And the run must have ~/.run_records/<run>/mapping.log - only start_drive.sh writes that file
# (step 3, the mapping). The folder alone is not enough: drive_media_recorder.py makes
# ~/.run_records/<run>/media/ for ANY --run value, so a test run with a drive-like name
# (e.g. --run s2_static_99) would otherwise make itself followable.
# Known limits: a test that runs start_drive.sh itself under a drive-like name IS followed
# (it is a real mapping start); and a test must never write into the real ~/jobs - run tests
# properly" line onto the public page through the OLD selection rule).
RUN_RE = re.compile(r"^s2_(?:(?:static|dynamic)_[0-9]{2}[a-z]?|fusion_T[0-9]{1,2}[a-z]?)\Z")
RECORDS = os.path.expanduser(os.environ.get("RECORDS_DIR", "~/.run_records"))


def is_drive(run):
    return bool(RUN_RE.match(run)) and os.path.isfile(os.path.join(RECORDS, run, "mapping.log"))


def candidate_runs():
    """{run: newest status-file time} for every run whose name ends one of the choosing files."""
    seen = {}
    for suf in CHOOSE_SUFFIXES:
        for p in glob.glob(os.path.join(JOBS, "*%s.progress" % suf)):
            run = os.path.basename(p)[:-len(suf + ".progress")]
            try:
                m = os.path.getmtime(p)
            except OSError:
                continue
            if m > seen.get(run, -1):
                seen[run] = m
    return seen


def newest_run():
    """The DRIVE or REHEARSAL whose status files were written most recently, or None.
    Between drives this stays on the last drive (never falls back to a bench run)."""
    best = None
    for run, m in candidate_runs().items():
        if is_drive(run) and (best is None or m > best[0]):
            best = (m, run)
    return best[1] if best else None


def read_line(path, now):
    try:
        m = os.path.getmtime(path)
        with open(path, errors="replace") as f:
            line = f.readline(LINE_MAX + 1).strip()
        return {"line": line[:LINE_MAX], "age_s": round(now - m, 1)}
    except OSError:
        return None


def picture(status, headers, body, now):
    if status != 200 or not body:
        return {"error": "HTTP %s: %s" % (status, body[:80].decode("utf-8", "replace").strip())}
    out = {"b64": base64.b64encode(body).decode("ascii"), "bytes": len(body),
           "sha": hashlib.sha1(body).hexdigest()[:16], "fetched_at": round(now, 3)}
    for h in ("X-Map-Left", "X-Map-Top", "X-Map-M-Per-Px", "X-Grid-N"):
        if headers.get(h):
            out[h[2:].lower().replace("-", "_")] = headers.get(h)
    return out


LOCAL_SNAP = os.path.join(JOBS, "live_snapshot.json")   # what live_site_server.py serves


def write_snapshot(data):
    """Write one snapshot atomically (a reader never sees half a file)."""
    tmp = LOCAL_SNAP + ".tmp"
    with open(tmp, "w") as f:
        f.write('{"received_at":%d,"body":' % int(time.time() * 1000))
        f.write(data.decode())
        f.write("}")
    os.replace(tmp, LOCAL_SNAP)


def write_progress(line):
    tmp = PROGRESS + ".tmp"
    with open(tmp, "w") as f:
        f.write(line + "\n")
    os.replace(tmp, PROGRESS)


def on_signal(sig, _frame):
    STOP["sig"] = sig


def dry_run():
    """--dry-run: print which run would be followed and why; reads no secret, pushes nothing."""
    cands = candidate_runs()
    for run, m in sorted(cands.items(), key=lambda kv: -kv[1]):
        why = ("DRIVE/REHEARSAL" if is_drive(run) else
               "skipped: name is a drive's but no ~/.run_records/%s/mapping.log" % run if RUN_RE.match(run) else
               "skipped: not a drive or rehearsal name")
        print("%s  %-34s %s" % (time.strftime("%Y-%m-%d %H:%M:%S %Z", time.localtime(m)), run, why))
    print("WOULD FOLLOW: %s" % newest_run())
    return 0


def main():
    if "--dry-run" in sys.argv[1:]:
        return dry_run()
    if jetson_status is None:
        log("status add-on not loaded (the map is still sent): %s" % _JETSON_STATUS_ERR)
    cur = os.nice(0)
    if cur < 19:
        os.nice(19 - cur)
    os.makedirs(JOBS, exist_ok=True)
    with open(PIDFILE, "w") as f:
        f.write("%d\n" % os.getpid())
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    started = time.time()
    host = socket.gethostname()
    seq = ok_n = fail_n = 0
    last_err = None
    last_ok_at = None
    peer_cache = None            # last LiDAR panel, re-sent between its fetches
    peer_ok = True
    local_was_ok = None
    payload = {"v": 1, "kind": "slam_live_push", "seq": 0, "interval_s": INTERVAL_S,
               "started_at": round(started, 3), "host": host, "pid": os.getpid()}
    log("start pid %d -> %s (local %s)" % (os.getpid(), LOCAL_SNAP, LOCAL))

    while STOP["sig"] is None:
        t0 = time.time()
        seq += 1
        # ---- the drive's own page ------------------------------------------------
        local = {"ok": False, "error": None, "stats": None}
        mp = None
        try:
            code, _, body = local_get("/stats.json", 1.5)
            if code != 200:
                raise OSError("stats.json answered HTTP %s" % code)
            local["stats"] = json.loads(body.decode("utf-8"))
            local["ok"] = True
            code, hdr, body = local_get("/map.png", 2.5)
            mp = picture(code, hdr, body, time.time())
            if "size_m" not in mp and local["stats"].get("size_m"):
                mp["size_m"] = local["stats"]["size_m"]
        except Exception as e:                        # the page is down: say so, keep pushing
            local["error"] = "%s: %s" % (type(e).__name__, e)
            mp = {"error": "local page not answering"}
        if local["ok"] != local_was_ok:
            log("local page %s%s" % ("answering" if local["ok"] else "NOT answering",
                                      "" if local["ok"] else " (%s)" % local["error"]))
            local_was_ok = local["ok"]

        # ---- the robot's LiDAR panel, passed through the local page -----------------
        every = PEER_EVERY if peer_ok else PEER_EVERY_OFF
        if local["ok"] and (peer_cache is None or seq % every == 0):
            try:
                code, _, body = local_get("/peer/stats.json", 2.5)
                if code != 200:
                    raise OSError("HTTP %s" % code)
                ps = json.loads(body.decode("utf-8"))
                code, hdr, body = local_get("/peer/map.png", 2.5)
                peer_cache = picture(code, hdr, body, time.time())
                peer_cache["stats"] = ps
                peer_cache["ok"] = "error" not in peer_cache
                peer_ok = True
            except Exception as e:
                peer_cache = {"ok": False, "error": "%s: %s" % (type(e).__name__, e)}
                peer_ok = False
        elif not local["ok"]:
            peer_cache = {"ok": False, "error": "the Jetson's live-map page is not answering"}

        # ---- the status lines for the newest run -----------------------------------
        now = time.time()
        run = newest_run()
        progress = {}
        if run:
            d = read_line(os.path.join(JOBS, run + ".progress"), now)
            if d:
                progress["drive"] = d
            for k, suf in SUFFIXES.items():
                d = read_line(os.path.join(JOBS, run + suf + ".progress"), now)
                if d:
                    progress[k] = d

        interval = INTERVAL_S if local["ok"] else INTERVAL_DOWN_S
        payload = {
            "v": 1, "kind": "slam_live_push", "seq": seq, "interval_s": interval,
            "pushed_at": round(now, 3), "started_at": round(started, 3),
            "host": host, "pid": os.getpid(), "run": run,
            "local": local, "map": mp, "peer": peer_cache, "progress": progress,
            "push": {"ok": ok_n, "fail": fail_n, "last_error": last_err},
        }
        if jetson_status is not None:
            try:
                payload["jetson"] = jetson_status.sample()
            except Exception as exc:
                payload["jetson"] = {"error": "%s: %s" % (type(exc).__name__, exc)}
        else:
            payload["jetson"] = {"error": "jetson_status.py could not be loaded beside the sender "
                                          "(%s)" % _JETSON_STATUS_ERR}
        data = json.dumps(payload, separators=(",", ":")).encode()
        if len(data) > MAX_PAYLOAD and len(json.dumps(payload.get("jetson"))) > 50000:
            # the status add-on must never be what stops the map (status-site review, defect 4)
            payload["jetson"] = {"error": "the computers' status was too large to send this time"}
            data = json.dumps(payload, separators=(",", ":")).encode()
        if len(data) > MAX_PAYLOAD and peer_cache and peer_cache.get("b64"):
            payload["peer"] = dict(peer_cache, b64=None, ok=False,
                                   error="LiDAR picture left out: snapshot over %d kB" % (MAX_PAYLOAD // 1000))
            data = json.dumps(payload, separators=(",", ":")).encode()
        if len(data) > MAX_PAYLOAD and mp and mp.get("b64"):
            payload["map"] = {"error": "camera picture too large to send (%d kB)" % (mp["bytes"] // 1000)}
            data = json.dumps(payload, separators=(",", ":")).encode()

        # ---- send ----------------------------------------------------------------
        tp = time.time()
        try:
            write_snapshot(data)
            code, rbody = 200, b'{"ok":true}'
            ok = code == 200 and b'"ok":true' in rbody
            if not ok:
                raise OSError("HTTP %s: %s" % (code, rbody[:120].decode("utf-8", "replace")))
            ok_n += 1
            if last_err:
                log("push recovered after: %s" % last_err)
            last_err = None
            last_ok_at = time.time()
        except Exception as e:
            fail_n += 1
            err = "%s: %s" % (type(e).__name__, e)
            if err != last_err:
                log("push FAILED: %s" % err)
            last_err = err
        ms = (time.time() - tp) * 1000

        try:
            write_progress("SLAM_LIVE_PUSH  run %s  push %d  %s  %.0f ms  %d kB  local page %s  "
                           "fails %d  last ok %s  %.0fs" % (
                               run or "-", seq, "ok" if last_err is None else "FAILING",
                               ms, len(data) // 1024, "ok" if local["ok"] else "DOWN", fail_n,
                               time.strftime("%H:%M:%S", time.localtime(last_ok_at)) if last_ok_at else "never",
                               time.time() - started))
        except OSError as e:
            log("could not write %s: %s" % (PROGRESS, e))

        # sleep to the next slot, waking early on a signal
        end = t0 + interval
        while STOP["sig"] is None and time.time() < end:
            time.sleep(min(0.25, max(0.0, end - time.time())))

    # ---- clean stop: tell the page it was on purpose -----------------------------------
    name = {signal.SIGINT: "SIGINT", signal.SIGTERM: "SIGTERM"}.get(STOP["sig"], str(STOP["sig"]))
    try:
        payload.update(stopping=True, stop_reason=name, pushed_at=round(time.time(), 3))
        write_snapshot(json.dumps(payload, separators=(",", ":")).encode())
    except Exception as e:
        log("final 'stopped' snapshot failed: %s" % e)
    write_progress("SLAM_LIVE_PUSH  complete - stopped by %s after %d pushes (%d failed)  %.0fs"
                   % (name, seq, fail_n, time.time() - started))
    log("stopped by %s after %d pushes (%d failed)" % (name, seq, fail_n))


if __name__ == "__main__":
    try:
        main()
    except SystemExit as e:
        log("refused to start: %s" % e)
        raise
