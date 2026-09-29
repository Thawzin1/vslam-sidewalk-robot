#!/usr/bin/env python3
"""replay_common.py - the settings, safety checks and small helpers shared by the replay web tools.

WHAT IT DOES
    Both replay_control_server.py (the web backend) and replay_job.py (the program that runs one
    replay job) import this file, so that they agree on where files live, what the job state file
    looks like, and when a replay must refuse to start.

HOW TO RUN
    Not run on its own. `python3 replay_common.py` prints the settings it resolved and the result
    of the safety checks, which is a quick way to see what the web page would say.

SETTINGS (environment variables, all optional; the same names as run/lib/paths.sh)
    REPO_ROOT            the repository (default: two folders above this file)
    RECORDS_DIR          drive recordings, one folder per run (default ~/.run_records)
    WORK_DIR             maps and working files (default ~/slam_series2)
    JOBS_DIR             one-line .progress files read by the jobs page (default ~/jobs)
    REPLAY_INPUTS_DIR    recordings copied in for replays: slam/<run>/ and bench/ (default ~/replay_inputs)
    REPLAY_RESULTS_DIR   one folder per replay job (default $WORK_DIR/replay/web)
    EXTRA_RECORD_DIRS    more folders to list recordings from, separated by ':' (default none)
    REPLAY_ROS_PORT      the replay's private ROS master (default 11399; the live one is 11311)
    MAP_VIEW_PORT        the replay's own live map picture server, on 127.0.0.1 (default 8195)
    REPLAY_CONTROL_PORT  the web backend, on 127.0.0.1 (default 8098)
"""
import json
import os
import re
import shutil
import socket
import time

HOME = os.path.expanduser("~")
HERE = os.path.dirname(os.path.abspath(__file__))


def setting(name, default):
    """An environment variable with a default; '~' is expanded so defaults can say '~/...'."""
    return os.path.expanduser(os.environ.get(name, default))


REPO_ROOT = setting("REPO_ROOT", os.path.dirname(os.path.dirname(HERE)))
RECORDS_DIR = setting("RECORDS_DIR", "~/.run_records")
WORK_DIR = setting("WORK_DIR", "~/slam_series2")
JOBS_DIR = setting("JOBS_DIR", "~/jobs")
INPUTS_DIR = setting("REPLAY_INPUTS_DIR", "~/replay_inputs")
RESULTS_DIR = setting("REPLAY_RESULTS_DIR", os.path.join(WORK_DIR, "replay", "web"))
EXTRA_RECORD_DIRS = [setting("X", d) for d in os.environ.get("EXTRA_RECORD_DIRS", "").split(":") if d]
REPLAY_ROS_PORT = int(os.environ.get("REPLAY_ROS_PORT", "11399"))
LIVE_ROS_PORT = 11311
MAP_VIEW_PORT = int(os.environ.get("MAP_VIEW_PORT", "8195"))
CONTROL_PORT = int(os.environ.get("REPLAY_CONTROL_PORT", "8098"))

STATE_DIR = os.path.join(JOBS_DIR, "replay_web")        # job state, lock, history
LOCK_FILE = os.path.join(STATE_DIR, "lock")
CURRENT_FILE = os.path.join(STATE_DIR, "current.json")
HISTORY_DIR = os.path.join(STATE_DIR, "history")
PROGRESS_FILE = os.path.join(JOBS_DIR, "replay_web.progress")   # the jobs page finds this by itself

# A replay must leave at least this much free disk, counting what it will write.
DISK_FLOOR_BYTES = 3 * 1000 ** 3
# A mapping replay writes its database at about 2 MB per second of recording (measured: 1.4 GB
# of outputs for 685 s of the s2_fusion_T5b drive). Used to estimate the space a replay needs.
SLAM_OUTPUT_BYTES_PER_S = 2.2e6
BENCH_OUTPUT_BYTES = 50e6           # CSV files and pictures: small

# A drive, localisation, SLAM or navigation session writes one of these progress lines while it
# is live. A line younger than this means "live now".
LIVE_SESSION_SUFFIXES = ("_drive.progress", "_localise.progress", "_slam.progress", "_nav.progress")
LIVE_SESSION_FRESH_S = 120
# Names ending "_to_drive.progress" are file-copy jobs, not robot drives; they match
# "_drive.progress" by accident and are not live sessions.
NOT_A_SESSION_SUFFIXES = ("_to_drive.progress",)

DEPTH_MODES = ["NEURAL", "NEURAL_PLUS", "QUALITY", "ULTRA", "PERFORMANCE"]
ACTIVE_STATES = ("starting", "running", "stopping")
FINAL_STATES = ("done", "failed")


# ------------------------------------------------------------------ small file helpers
def now_s():
    return time.time()


def write_json_atomic(path, obj):
    """Write to a temporary file, then rename: a reader never sees half a file."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(obj, f, indent=1)
    os.replace(tmp, path)


def read_json(path, default=None):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return default


def write_line_atomic(path, line):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        f.write(line.rstrip("\n") + "\n")
    os.replace(tmp, path)


def tail_lines(path, count=30):
    """The last `count` lines of a text file, read from its end so a large log costs little."""
    try:
        with open(path, "rb") as f:
            f.seek(0, os.SEEK_END)
            size = f.tell()
            f.seek(max(0, size - 64 * 1024))
            text = f.read().decode("utf-8", "replace")
    except OSError:
        return []
    return text.splitlines()[-count:]


def safe_name(text, fallback="replay"):
    """Letters, digits, '-' and '_' only, so a name typed on the page can never become a path."""
    cleaned = re.sub(r"[^A-Za-z0-9_-]+", "_", str(text or "")).strip("_")[:60]
    return cleaned or fallback


def inside(folder, path):
    """True if `path` really lies inside `folder` (after following links and '..')."""
    folder = os.path.realpath(folder)
    path = os.path.realpath(path)
    return path.startswith(folder + os.sep)


def free_bytes(path):
    """Free space on the disk that holds `path` (or its nearest existing parent)."""
    while path and not os.path.exists(path):
        path = os.path.dirname(path)
    return shutil.disk_usage(path or HOME).free


# ------------------------------------------------------------------ processes, by number and exact name
def process_name(pid):
    """The program name of a process, from /proc/<pid>/comm (exact, never a text search)."""
    try:
        with open("/proc/%d/comm" % pid) as f:
            return f.read().strip()
    except OSError:
        return None


def process_args(pid):
    try:
        with open("/proc/%d/cmdline" % pid, "rb") as f:
            return [a.decode("utf-8", "replace") for a in f.read().split(b"\0") if a]
    except OSError:
        return []


def is_alive(pid):
    return bool(pid) and os.path.isdir("/proc/%d" % int(pid))


def runs_script(pid, script_name):
    """True if process `pid` is python running the script whose file name is exactly `script_name`.
    Each argument is compared whole, by base name, so a process whose command line merely
    CONTAINS the name (a text editor, a grep, this check itself) never counts."""
    if not is_alive(pid):
        return False
    args = process_args(int(pid))
    return any(os.path.basename(a) == script_name for a in args[1:3])


def all_pids():
    return [int(d) for d in os.listdir("/proc") if d.isdigit()]


def children_of(pid):
    """Direct children of a process, from /proc/<child>/stat (field 4 is the parent)."""
    kids = []
    for p in all_pids():
        try:
            with open("/proc/%d/stat" % p) as f:
                stat = f.read()
            parent = int(stat.rsplit(")", 1)[1].split()[1])
        except (OSError, ValueError, IndexError):
            continue
        if parent == pid:
            kids.append(p)
    return kids


def descendants_of(pid):
    found, todo = [], [pid]
    while todo:
        for kid in children_of(todo.pop()):
            found.append(kid)
            todo.append(kid)
    return found


def ros_master_port_of(pid):
    """Which ROS master a process talks to: ROS_MASTER_URI in its environment, else the ROS default
    11311. Returns None if the environment cannot be read (a process of another user)."""
    try:
        with open("/proc/%d/environ" % pid, "rb") as f:
            env = f.read().split(b"\0")
    except OSError:
        return None
    for entry in env:
        if entry.startswith(b"ROS_MASTER_URI="):
            match = re.search(rb":(\d+)/?$", entry)
            return int(match.group(1)) if match else LIVE_ROS_PORT
    return LIVE_ROS_PORT


def port_answers(port):
    """True if something accepts connections on 127.0.0.1:<port>."""
    with socket.socket() as s:
        s.settimeout(0.5)
        return s.connect_ex(("127.0.0.1", port)) == 0


# ------------------------------------------------------------------ the safety checks
def live_session_files():
    """Progress lines of live drives/sessions: [(file name, age in seconds)]."""
    live = []
    try:
        names = os.listdir(JOBS_DIR)
    except OSError:
        return live
    for name in names:
        if not name.endswith(LIVE_SESSION_SUFFIXES) or name.endswith(NOT_A_SESSION_SUFFIXES):
            continue
        try:
            age = now_s() - os.path.getmtime(os.path.join(JOBS_DIR, name))
        except OSError:
            continue
        if age < LIVE_SESSION_FRESH_S:
            live.append((name, round(age)))
    return live


def live_rtabmap_pids():
    """RTAB-Map processes that belong to the live ROS master (11311), or whose master cannot be
    read (counted as live, to be safe). A replay's own RTAB-Map (port 11399) is not counted."""
    found = []
    for pid in all_pids():
        if process_name(pid) != "rtabmap":
            continue
        port = ros_master_port_of(pid)
        if port is None or port == LIVE_ROS_PORT:
            found.append(pid)
    return found


def lock_holder():
    """The job id written in the lock file, or None if no job holds the lock."""
    try:
        with open(LOCK_FILE) as f:
            return f.read().strip() or "unknown"
    except OSError:
        return None


def start_refusals(extra_bytes=0, folder=None, check_lock=True, check_ports=False):
    """Every reason a new job must not start now, in plain words. An empty list means "go".
    extra_bytes: what the job will add to the disk (its expected outputs)."""
    reasons = []
    if check_lock:
        holder = lock_holder()
        if holder:
            reasons.append("another replay job is running (%s); only one runs at a time" % holder)
    for name, age in live_session_files():
        reasons.append("a robot drive or session looks live on this Jetson: %s was updated %d s ago" % (name, age))
    for pid in live_rtabmap_pids():
        reasons.append("RTAB-Map is running on the live ROS master (process %d); a replay would compete "
                       "with it for the processor and the graphics chip" % pid)
    free = free_bytes(folder or RESULTS_DIR)
    after = free - extra_bytes
    if after < DISK_FLOOR_BYTES:
        reasons.append("not enough disk: %.1f GB free, this job needs about %.1f GB, which would leave %.1f GB "
                       "(the floor is %.0f GB)" % (free / 1e9, extra_bytes / 1e9, after / 1e9, DISK_FLOOR_BYTES / 1e9))
    if check_ports:
        if port_answers(REPLAY_ROS_PORT):
            reasons.append("something already answers on the replay's ROS port %d (a replay started by hand?)"
                           % REPLAY_ROS_PORT)
        if port_answers(MAP_VIEW_PORT):
            reasons.append("the replay map picture port %d is already in use" % MAP_VIEW_PORT)
    return reasons


# ------------------------------------------------------------------ the job state file
def load_current():
    return read_json(CURRENT_FILE, {"state": "idle"})


def save_state(state):
    state["updated_at"] = now_s()
    write_json_atomic(CURRENT_FILE, state)


def progress_numbers(line):
    """(done, total, elapsed_s) from a progress line such as 'REPLAY run 12/685 s  30s'."""
    done = total = elapsed = None
    match = re.search(r"(\d+(?:\.\d+)?)\s*/\s*(\d+(?:\.\d+)?)", line or "")
    if match:
        done, total = float(match.group(1)), float(match.group(2))
    match = re.search(r"(?:^|\s)(\d+)s(?:\s|$)", line or "")
    if match:
        elapsed = int(match.group(1))
    return done, total, elapsed


if __name__ == "__main__":
    for name in ("REPO_ROOT", "RECORDS_DIR", "WORK_DIR", "JOBS_DIR", "INPUTS_DIR", "RESULTS_DIR",
                 "EXTRA_RECORD_DIRS", "REPLAY_ROS_PORT", "MAP_VIEW_PORT",
                 "CONTROL_PORT"):
        print("%-18s %s" % (name, globals()[name]))
    print("start now?", start_refusals() or "yes")
