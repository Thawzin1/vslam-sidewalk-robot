#!/usr/bin/env python3
"""replay_control_server.py - the small web backend behind the site's "Replay" page.

WHAT IT DOES
    Lets lab students start, watch and stop replays of recorded data on the Jetson, with no robot:
      - a mapping ("SLAM") replay of a recorded drive: run/replay.sh on <run>.svo2 + fusion.bag
      - a camera bench test: depth recomputed from a station's .svo2 in chosen depth modes, and the
        sphere target's centre and spread measured in each
      - deleting recordings that were copied in for replays (only inside REPLAY_INPUTS_DIR)
    It owns the one-job-at-a-time lock, refuses to start when a robot session is live or the disk
    would run too low, starts each job with setsid (replay_job.py) and reports its state.

    It listens on 127.0.0.1 only. People reach it through live_site_server.py, which checks the
    site's view key and then forwards every /replay/api/... request here. So this program has no
    password of its own: whoever holds the site link may start and stop replays (the owner's
    decision for the lab students); the safety checks below are what protect the machine.

HOW TO RUN
    python3 replay_control_server.py          (keep_replay_control.sh starts it from cron)
    Settings: see replay_common.py (REPO_ROOT, RECORDS_DIR, WORK_DIR, JOBS_DIR, ... ).

WEB INTERFACE (all answers are JSON unless said otherwise)
    GET  /replay/api/status              the job's state, progress, last 30 log lines, safety checks
    GET  /replay/api/recordings          recordings on this Jetson, with sizes and what they are for
    GET  /replay/api/history             past jobs, newest first
    GET  /replay/api/check?kind=slam|bench&extra_bytes=N
                                         the refusals a new job would get now (starts nothing)
    GET  /replay/api/livemap.png         the map picture of the running mapping replay
    GET  /replay/api/livemap.json        its status lines
    GET  /replay/api/result/<job>/[file] a job's result folder (read-only; small files only)
    POST /replay/api/start     {"kind":"slam", "bag":..., "svo":..., "depth_mode":..., "max_s":..., "name":...}
                               {"kind":"bench", "svo":..., "modes":[...], "frames":..., "roi_x":..., ...}
    POST /replay/api/stop      {}
    POST /replay/api/delete    {"path":...}   (only inside the replay inputs or results folders)
    POST bodies must be sent as application/json: a plain web form on another site cannot send
    that, so another page cannot press these buttons on a visitor's behalf.

INPUTS    recordings under RECORDS_DIR, REPLAY_INPUTS_DIR and EXTRA_RECORD_DIRS. New recordings are
          downloaded from the Autonomous Service Robot Teams folder on any computer and copied to
          REPLAY_INPUTS_DIR/slam/<run>/ or REPLAY_INPUTS_DIR/bench/ over the lab network (the page says how).
OUTPUTS   job state in $JOBS_DIR/replay_web/, results in $REPLAY_RESULTS_DIR/<job id>/
"""
import http.server
import json
import mimetypes
import os
import shutil
import signal
import socketserver
import subprocess
import threading
import time
import urllib.parse
import urllib.request

import replay_common as rc

HERE = os.path.dirname(os.path.abspath(__file__))
JOB_SCRIPT = os.path.join(HERE, "replay_job.py")
MAX_BODY_BYTES = 64 * 1024
MAX_RESULT_FILE_BYTES = 30 * 1000 ** 2
RESULT_FILE_TYPES = (".png", ".json", ".txt", ".csv", ".log", ".tum")
START_LOCK = threading.Lock()          # two clicks at once must not both pass the lock check


# ------------------------------------------------------------------ recordings on this Jetson
def record_roots():
    """(label, folder, deletable) for every folder that may hold recordings."""
    roots = [("drive records", rc.RECORDS_DIR, False),
             ("copied in for mapping replays", os.path.join(rc.INPUTS_DIR, "slam"), True),
             ("camera bench recordings", os.path.join(rc.INPUTS_DIR, "bench"), True)]
    roots += [("extra recordings folder", d, False) for d in rc.EXTRA_RECORD_DIRS]
    return roots


def find_recordings(folder, depth=3):
    """Every *.svo2 and fusion.bag at most `depth` folders below `folder`."""
    found = []
    if not os.path.isdir(folder):
        return found
    for here, dirs, files in os.walk(folder):
        level = os.path.relpath(here, folder).count(os.sep) + (0 if here == folder else 1)
        dirs[:] = [d for d in dirs if not d.startswith(".") and level < depth]
        for name in files:
            if name.endswith(".svo2") or name == "fusion.bag":
                found.append(os.path.join(here, name))
    return found


def local_recordings():
    files = []
    for label, root, deletable in record_roots():
        for path in find_recordings(root):
            try:
                stat = os.stat(path)
            except OSError:
                continue
            is_bag = path.endswith(".bag")
            usable = stat.st_size > 0          # an empty file is a recording that never started
            files.append({
                "path": path, "where": label, "folder": os.path.relpath(os.path.dirname(path), root),
                "name": os.path.basename(path), "size": stat.st_size, "mtime": stat.st_mtime,
                "kind": "bag" if is_bag else "svo",
                "used_for": ("empty file - cannot be replayed" if not usable else
                             "mapping replay (the robot's wheels and gyroscope)" if is_bag else
                             "mapping replay (the camera) or camera bench test"),
                "usable": usable,
                "deletable": deletable,
            })
    return files


def slam_sets(files):
    """Pair every fusion.bag with the camera files that belong to the same drive: in the same
    folder, or in a sibling folder whose name starts with the run name (a drive whose camera
    recording was restarted has <run>_b/ next to <run>/)."""
    svos = [f for f in files if f["kind"] == "svo" and f["usable"]]
    sets = []
    for bag in (f for f in files if f["kind"] == "bag" and f["usable"]):
        run_dir = os.path.dirname(bag["path"])
        run = os.path.basename(run_dir)
        parent = os.path.dirname(run_dir)
        matches = [s for s in svos if os.path.dirname(s["path"]) == run_dir or
                   (os.path.dirname(os.path.dirname(s["path"])) == parent and
                    os.path.basename(os.path.dirname(s["path"])).startswith(run))]
        if matches:
            sets.append({"run": run, "bag": bag["path"], "bag_size": bag["size"], "where": bag["where"],
                         "svos": [{"path": s["path"], "name": s["name"], "size": s["size"]} for s in matches]})
    return sets


def allowed_input(path, ending):
    """A path typed into a request is used only if it is a recording inside a known folder."""
    if not path or not path.endswith(ending) or not os.path.isfile(path):
        return False
    return any(rc.inside(root, path) for _, root, _ in record_roots())


# ------------------------------------------------------------------ job state
def job_alive(state):
    return rc.runs_script(state.get("runner_pid"), "replay_job.py")


def current_state():
    """The job state, corrected if its runner died without saying so (killed, machine restarted)."""
    state = rc.load_current()
    if state.get("state") in rc.ACTIVE_STATES and not job_alive(state):
        if rc.now_s() - state.get("updated_at", 0) > 20:
            state["state"] = "failed"
            state["reason"] = "the job's runner program stopped without finishing (killed, or the Jetson restarted)"
            rc.save_state(state)
            rc.write_json_atomic(os.path.join(rc.HISTORY_DIR, "%s.json" % state.get("id", "unknown")), state)
            if rc.lock_holder() == state.get("id"):
                os.remove(rc.LOCK_FILE)
    elif state.get("state") not in rc.ACTIVE_STATES and rc.lock_holder() and \
            rc.now_s() - os.path.getmtime(rc.LOCK_FILE) > 60:
        os.remove(rc.LOCK_FILE)           # a lock left behind by a job that already ended
    return state


def status():
    state = current_state()
    active = state.get("state") in rc.ACTIVE_STATES
    try:
        with open(rc.PROGRESS_FILE) as f:
            line = f.read().strip()
    except OSError:
        line = ""
    return {
        "job": state, "active": active, "progress_line": line,
        "log_tail": rc.tail_lines(state.get("log", ""), 30) if state.get("log") else [],
        "refusals_now": [] if active else rc.start_refusals(),
        "disk_free_bytes": rc.free_bytes(rc.RESULTS_DIR), "disk_floor_bytes": rc.DISK_FLOOR_BYTES,
        "map_view_up": active and state.get("kind") == "slam" and rc.port_answers(rc.MAP_VIEW_PORT),
        "depth_modes": rc.DEPTH_MODES, "server_now": rc.now_s(),
    }


def history(limit=50):
    rows = []
    try:
        names = sorted(os.listdir(rc.HISTORY_DIR), reverse=True)[:limit]
    except OSError:
        return rows
    for name in names:
        job = rc.read_json(os.path.join(rc.HISTORY_DIR, name))
        if job:
            job["result_folder_exists"] = os.path.isdir(job.get("out_dir", ""))
            rows.append(job)
    return rows


def make_job(kind, params, name, extra_bytes):
    job_id = time.strftime("%Y%m%d_%H%M%S") + "_" + rc.safe_name(name, kind)
    out_dir = os.path.join(rc.RESULTS_DIR, job_id)
    return {"id": job_id, "kind": kind, "params": params, "out_dir": out_dir, "expected_bytes": extra_bytes,
            "created_at": rc.now_s()}


def launch(job):
    """Take the lock, write the job description and start the runner in a session of its own."""
    with START_LOCK:
        refusals = rc.start_refusals(job["expected_bytes"], check_ports=job["kind"] == "slam")
        if refusals:
            return 409, {"started": False, "refusals": refusals}
        os.makedirs(rc.STATE_DIR, exist_ok=True)
        try:
            fd = os.open(rc.LOCK_FILE, os.O_CREAT | os.O_EXCL | os.O_WRONLY)
        except FileExistsError:
            return 409, {"started": False, "refusals": ["another replay job is running"]}
        with os.fdopen(fd, "w") as f:
            f.write(job["id"])
        spec_path = os.path.join(rc.STATE_DIR, "jobs", job["id"] + ".json")
        rc.write_json_atomic(spec_path, job)
        os.makedirs(job["out_dir"], exist_ok=True)
        runner_log = open(os.path.join(job["out_dir"], "runner.log"), "a")
        proc = subprocess.Popen(["nice", "-n", "19", "python3", JOB_SCRIPT, spec_path], cwd=HERE,
                                stdout=runner_log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                start_new_session=True)     # setsid: the job outlives this server
        rc.save_state(dict(job, state="starting",
                           reason="starting", runner_pid=proc.pid, started_at=rc.now_s(),
                           log=os.path.join(job["out_dir"], "console.log"), progress={}, result={}))
        threading.Thread(target=proc.wait, daemon=True).start()   # collect it when it ends
        return 200, {"started": True, "id": job["id"]}


# ------------------------------------------------------------------ requests that start or change things
def number(value, low, high, default):
    try:
        v = float(value)
    except (TypeError, ValueError):
        return default
    return min(high, max(low, v))


def start_slam(body):
    bag, svo = body.get("bag", ""), body.get("svo", "")
    if not allowed_input(bag, "fusion.bag"):
        return 400, {"started": False, "refusals": ["choose a fusion.bag from the list"]}
    if not allowed_input(svo, ".svo2"):
        return 400, {"started": False, "refusals": ["choose a camera recording (.svo2) from the list"]}
    mode = body.get("depth_mode", "NEURAL")
    if mode not in rc.DEPTH_MODES:
        return 400, {"started": False, "refusals": ["unknown depth mode %s" % mode]}
    max_s = int(number(body.get("max_s"), 0, 7200, 0))
    # expected outputs: the database grows about 2 MB per played second; a camera file holds about
    # 3 MB per second of recording, which gives its length before it has been read
    seconds = os.path.getsize(svo) / 3.0e6
    if max_s:
        seconds = min(seconds, max_s)
    extra = int(seconds * rc.SLAM_OUTPUT_BYTES_PER_S)
    run = os.path.basename(os.path.dirname(bag))
    params = {"run": run, "bag": bag, "svo": svo, "depth_mode": mode, "max_s": max_s, "rate": 1.0}
    return launch(make_job("slam", params, body.get("name") or run, extra))


def start_bench(body):
    svo = body.get("svo", "")
    if not allowed_input(svo, ".svo2"):
        return 400, {"started": False, "refusals": ["choose a camera recording (.svo2) from the list"]}
    modes = [m for m in body.get("modes", []) if m in rc.DEPTH_MODES]
    if not modes:
        return 400, {"started": False, "refusals": ["choose at least one depth mode"]}
    params = {"svo": svo, "modes": modes,
              "frames": int(number(body.get("frames"), 1, 5000, 60)),
              "roi_x": number(body.get("roi_x"), 0.5, 20.0, 4.0),
              "sensor_h": number(body.get("sensor_h"), 0.0, 3.0, 0.6972),
              "stabilization": 1 if str(body.get("stabilization", "0")) == "1" else 0,
              "workers": int(number(body.get("workers"), 1, 6, 4))}
    name = body.get("name") or "bench_" + os.path.splitext(os.path.basename(svo))[0]
    return launch(make_job("bench", params, name, int(rc.BENCH_OUTPUT_BYTES)))


def stop_job():
    state = current_state()
    if state.get("state") not in rc.ACTIVE_STATES:
        return 409, {"stopped": False, "why": "no job is running"}
    pid = state.get("runner_pid")
    if not job_alive(state):
        return 409, {"stopped": False, "why": "the job's runner is not running"}
    os.kill(int(pid), signal.SIGTERM)       # the runner then stops its work in the right order
    return 200, {"stopped": True, "why": "stop requested; a mapping replay closes RTAB-Map first, "
                                        "which can take a minute"}


def delete_path(body):
    path = os.path.realpath(body.get("path", ""))
    in_inputs = rc.inside(rc.INPUTS_DIR, path)
    in_results = rc.inside(rc.RESULTS_DIR, path) and os.path.dirname(path) == os.path.realpath(rc.RESULTS_DIR)
    if not (in_inputs or in_results) or not os.path.exists(path):
        return 400, {"deleted": False, "why": "only recordings copied into the replay inputs folder, and replay result folders, can be deleted here"}
    state = current_state()
    if state.get("state") in rc.ACTIVE_STATES:
        params = state.get("params", {})
        in_use = [params.get("bag"), params.get("svo"), params.get("dest"), state.get("out_dir")]
        for used in (os.path.realpath(u) for u in in_use if u):
            if used == path or rc.inside(path, used) or rc.inside(used, path):
                return 409, {"deleted": False, "why": "the running job uses this"}
    size = sum(os.path.getsize(os.path.join(d, f)) for d, _, fs in os.walk(path) for f in fs) \
        if os.path.isdir(path) else os.path.getsize(path)
    if os.path.isdir(path):
        shutil.rmtree(path)
    else:
        os.remove(path)
    return 200, {"deleted": True, "path": path, "freed_bytes": size}


# ------------------------------------------------------------------ the HTTP side
class Handler(http.server.BaseHTTPRequestHandler):
    server_version = "replay-control/1"

    def log_message(self, fmt, *args):
        pass

    def send_body(self, code, content_type, data):
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def send_json(self, code, obj):
        self.send_body(code, "application/json", json.dumps(obj, separators=(",", ":")).encode())

    def proxy_map_view(self, name):
        try:
            with urllib.request.urlopen("http://127.0.0.1:%d/%s" % (rc.MAP_VIEW_PORT, name), timeout=5) as r:
                return self.send_body(200, r.headers.get("Content-Type", "application/octet-stream"), r.read())
        except Exception:
            return self.send_json(503, {"error": "no map picture yet (the replay's map appears once it is playing)"})

    def send_result(self, rest):
        parts = [p for p in rest.split("/") if p]
        if not parts:
            return self.send_json(404, {"error": "which job?"})
        folder = os.path.join(rc.RESULTS_DIR, rc.safe_name(parts[0], "none"))
        if not os.path.isdir(folder):
            return self.send_json(404, {"error": "no such result folder"})
        if len(parts) == 1:
            return self.send_body(200, "text/html; charset=utf-8", result_index_html(parts[0], folder).encode())
        path = os.path.join(folder, os.path.basename(parts[1]))
        if not (rc.inside(folder, path) and os.path.isfile(path) and path.endswith(RESULT_FILE_TYPES)
                and os.path.getsize(path) <= MAX_RESULT_FILE_BYTES):
            return self.send_json(404, {"error": "not a result file that can be shown here"})
        kind = mimetypes.guess_type(path)[0] or "text/plain"
        if kind.startswith("text/") or path.endswith((".log", ".tum", ".csv")):
            kind = "text/plain; charset=utf-8"
        with open(path, "rb") as f:
            return self.send_body(200, kind, f.read())

    def do_GET(self):
        url = urllib.parse.urlsplit(self.path)
        query = urllib.parse.parse_qs(url.query)
        route = url.path[len("/replay/api/"):] if url.path.startswith("/replay/api/") else None
        if route == "status":
            return self.send_json(200, status())
        if route == "recordings":
            files = local_recordings()
            return self.send_json(200, {"files": files, "slam_sets": slam_sets(files),
                                        "bench_svos": [f for f in files if f["kind"] == "svo" and f["usable"]],
                                        "inputs_dir": rc.INPUTS_DIR})
        if route == "history":
            return self.send_json(200, {"jobs": history()})
        if route == "check":
            extra = int(number((query.get("extra_bytes") or ["0"])[0], 0, 1e13, 0))
            kind = (query.get("kind") or ["slam"])[0]
            refusals = rc.start_refusals(extra, check_ports=kind == "slam")
            return self.send_json(200, {"would_start": not refusals, "refusals": refusals, "extra_bytes": extra,
                                        "disk_free_bytes": rc.free_bytes(rc.RESULTS_DIR)})
        if route == "livemap.png":
            return self.proxy_map_view("map.png")
        if route == "livemap.json":
            return self.proxy_map_view("stats.json")
        if route is not None and route.startswith("result/"):
            return self.send_result(route[len("result/"):])
        return self.send_json(404, {"error": "unknown address"})

    def do_POST(self):
        url = urllib.parse.urlsplit(self.path)
        route = url.path[len("/replay/api/"):] if url.path.startswith("/replay/api/") else None
        if not (self.headers.get("Content-Type") or "").startswith("application/json"):
            return self.send_json(415, {"error": "send JSON (Content-Type: application/json)"})
        length = int(self.headers.get("Content-Length") or 0)
        if length > MAX_BODY_BYTES:
            return self.send_json(413, {"error": "request too large"})
        try:
            body = json.loads(self.rfile.read(length).decode() or "{}")
        except ValueError:
            return self.send_json(400, {"error": "not valid JSON"})
        if route == "start":
            kind = body.get("kind")
            if kind == "slam":
                return self.send_json(*start_slam(body))
            if kind == "bench":
                return self.send_json(*start_bench(body))
            return self.send_json(400, {"error": "kind must be slam or bench"})
        if route == "stop":
            return self.send_json(*stop_job())
        if route == "delete":
            return self.send_json(*delete_path(body))
        return self.send_json(404, {"error": "unknown address"})


def result_index_html(job_id, folder):
    """A plain list of a result folder's files; small ones are links, big ones (maps, bags) are
    only named, because they are too large to send through the website."""
    rows = []
    for name in sorted(os.listdir(folder)):
        path = os.path.join(folder, name)
        if not os.path.isfile(path):
            continue
        size = os.path.getsize(path)
        link = name.endswith(RESULT_FILE_TYPES) and size <= MAX_RESULT_FILE_BYTES
        label = ('<a href="/replay/api/result/%s/%s">%s</a>' % (urllib.parse.quote(job_id), urllib.parse.quote(name), name)
                 if link else name + " (kept on the Jetson)")
        rows.append("<tr><td>%s</td><td style='text-align:right'>%.1f MB</td></tr>" % (label, size / 1e6))
    return ("<!doctype html><meta charset=utf-8><meta name=viewport content='width=device-width,initial-scale=1'>"
            "<title>Replay results</title><body style='font-family:sans-serif;padding:16px;background:#0e1116;color:#e8ecf2'>"
            "<style>a{color:#8ab4f8}td{padding:3px 10px 3px 0;overflow-wrap:anywhere}</style>"
            "<h2>Result folder %s</h2><p>Read-only. On the Jetson it is %s</p><table>%s</table></body>"
            % (job_id, folder, "".join(rows)))


class Server(socketserver.ThreadingMixIn, http.server.HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


if __name__ == "__main__":
    os.makedirs(rc.HISTORY_DIR, exist_ok=True)
    os.makedirs(os.path.join(rc.INPUTS_DIR, "slam"), exist_ok=True)
    os.makedirs(os.path.join(rc.INPUTS_DIR, "bench"), exist_ok=True)
    print("replay control on http://127.0.0.1:%d/replay/api/status (repository %s)" % (rc.CONTROL_PORT, rc.REPO_ROOT))
    Server(("127.0.0.1", rc.CONTROL_PORT), Handler).serve_forever()
