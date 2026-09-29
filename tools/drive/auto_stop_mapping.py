#!/usr/bin/env python3
"""auto_stop_mapping.py - shut the mapping down properly, on the Jetson, by itself.

WHY (drive 1 of series 2, 2026-09-24)
    Drive 1's map was never shut down. The Jetson's wifi card fell off its
    internal connection mid-drive, so nobody could reach it to stop the mapping,
    and the Jetson later lost power with the program still running. What is
    written only on a proper shutdown was lost: the map's corrected positions,
    its visual dictionary, and half its links (check_closed.py shows which).

    This runs ON the Jetson, started alongside the mapping, and needs no network.
    When told the robot is parked, it stops the mapping the careful way, waits
    however long the database takes to close, and checks the result.

    *Plain terms: a helper that sits on the Jetson and, when told "park",
    waits for the robot to settle on the mark and switches the map off
    properly - by itself, so a dropped connection partway through cannot
    stop it finishing the job.*

WHEN IT STOPS THE MAPPING - ONLY WHEN THE USER SAYS "PARK" (user's rule, 2026-09-24)
    Being still is NEVER enough on its own: a pause mid-drive must not end the
    map. The user says "park"; the Jetson operator (or anyone who can reach the Jetson by
    any route) creates the file ~/slam_series2/PARK_<run>. Then:

      it waits until the robot has been STILL for --park-hold seconds (60 s),
      because the map needs a moment on the mark to recognise the start - drive
      1's end-to-start closure came only after 21 s standing still - and 60 s
      parked also measures that drive's gyroscope drift at the end (the robot's
      heading drifts ~5 deg/min even parked; research report 2026-09-24). Still means
      the camera's own position has not moved more than 5 cm or 3 degrees.
      If the robot is not still within --park-timeout (3 min), it closes anyway.

    Two other ways it ends, neither of which can happen during a normal drive:
      STOP_<run> file     close NOW, no hold (for when something is wrong)
      --max-minutes (90)  a last-resort backstop, far beyond any drive, for the
                          night the Jetson loses its network AND nobody can say
                          "park" - drive 1's failure. The map closes properly
                          instead of dying with the power.
    --auto-park restores the old behaviour (stop after --still-seconds still,
    not before --min-minutes). It is OFF unless asked for.

    A tracking loss looks like standing still (the position freezes while the
    camera is lost), so a message the tracker marks as lost RESETS the
    stillness clock, and a silence of more than 10 s with no messages at all
    stops the clock from counting.

HOW IT STOPS IT, AND WHY NOT JUST CTRL-C
    Ctrl-C on roslaunch gives each program 15 seconds to finish (verified on
    the Jetson: roslaunch/nodeprocess.py, DEFAULT_TIMEOUT_SIGINT = 15.0), then
    kills it. Closing a multi-gigabyte map can take longer than that, and a
    killed map is a map not shut down properly. So this sends the interrupt to
    the mapping program itself (found by its exact process name, `rtabmap`,
    never a substring search - ENGINEERING_NOTES.md rule 8) and waits with NO deadline
    for it to exit. Only then does it interrupt the mapping's roslaunch, so
    the helpers around it close too. The camera is left running for the next
    drive.

FUSION DRIVES (--fusion, given by start_drive.sh only when FUSION=1; added 2026-09-24)
    After the map has closed, the fusion programs are stopped too, so the map is never
    starved of wheels before it has closed. The safe order (recording, then the blend,
    then the robot bridge) lives in ONE place, stop_fusion.sh beside this file, which
    reads ~/.run_records/<run>/fusion_pids (shell lines RECORD_PID=, FUSED_PID=,
    BRIDGE_PID=). If stop_fusion.sh is not deployed, this helper stops BRIDGE_PID itself,
    and only if that number still belongs to robot_bridge_recv.py (argv[1]'s base name,
    whole names only - ENGINEERING_NOTES.md rule 8), so a number since given to another program is
    never touched. Without --fusion nothing here runs and it behaves exactly as before.

VISIBLE FROM A PHONE
    Writes ~/jobs/<run>_autostop.progress for the jobs page (JOBS_PAGE_URL), with
    an N/M minutes counter, every 15 s; and a final line saying what happened.

  usage (on the Jetson, detached, right after the mapping starts):
    setsid nohup python3 $TOOLS_DIR/auto_stop_mapping.py --run s2_static_02 \\
        > ~/.run_records/s2_static_02/autostop.log 2>&1 < /dev/null &
"""
from __future__ import print_function

import argparse
import math
import os
import signal
import subprocess
import sys
import time

HOME = os.path.expanduser("~")
TOOLS = os.path.dirname(os.path.abspath(__file__))


# ----------------------------------------------------------- processes
def procs_named(name):
    """PIDs whose process name is EXACTLY `name` - and whose first argument's
    base name is too. Never a substring match: that is what made dead jobs
    look alive four times in this project. Skips this process and its parent."""
    out = []
    me = {os.getpid(), os.getppid()}
    for d in os.listdir("/proc"):
        if not d.isdigit() or int(d) in me:
            continue
        try:
            with open("/proc/%s/comm" % d) as fh:
                comm = fh.read().strip()
            with open("/proc/%s/cmdline" % d, "rb") as fh:
                argv = [a.decode("utf-8", "replace") for a in fh.read().split(b"\0") if a]
        except OSError:
            continue
        if comm == name and argv and os.path.basename(argv[0]) == name:
            out.append(int(d))
    return out


def roslaunch_of(launch_file):
    """PIDs of roslaunch processes that were given exactly this launch file."""
    out = []
    me = {os.getpid(), os.getppid()}
    for d in os.listdir("/proc"):
        if not d.isdigit() or int(d) in me:
            continue
        try:
            with open("/proc/%s/cmdline" % d, "rb") as fh:
                argv = [a.decode("utf-8", "replace") for a in fh.read().split(b"\0") if a]
        except OSError:
            continue
        # roslaunch is a Python script: argv is [python3, .../roslaunch, pkg, file, ...]
        names = [os.path.basename(a) for a in argv]
        if "roslaunch" in names[:2] and launch_file in names:
            out.append(int(d))
    return out


def progress_watchers(run):
    """PIDs of `python3 .../slam_progress.py --run <run> ...` - whole arguments."""
    out = []
    me = {os.getpid(), os.getppid()}
    for d in os.listdir("/proc"):
        if not d.isdigit() or int(d) in me:
            continue
        try:
            with open("/proc/%s/cmdline" % d, "rb") as fh:
                argv = [a.decode("utf-8", "replace") for a in fh.read().split(b"\0") if a]
        except OSError:
            continue
        if (len(argv) >= 4 and os.path.basename(argv[0]).startswith("python")
                and os.path.basename(argv[1]) == "slam_progress.py"
                and "--run" in argv and argv[argv.index("--run") + 1:][:1] == [run]):
            out.append(int(d))
    return out


def alive(pid):
    try:
        with open("/proc/%d/stat" % pid) as fh:
            return fh.read().split()[2] != "Z"
    except OSError:
        return False


def is_program(pid, name):
    """True if PID is alive and the base name of its argv[1] is EXACTLY name (a python3 script)."""
    if not alive(pid):
        return False
    try:
        with open("/proc/%d/cmdline" % pid, "rb") as fh:
            argv = [a.decode("utf-8", "replace") for a in fh.read().split(b"\0") if a]
    except OSError:
        return False
    return len(argv) >= 2 and os.path.basename(argv[1]) == name


def stop_fusion(args, prog):
    """--fusion only: after the map has closed, stop the fusion programs (see the header)."""
    sf = os.path.join(TOOLS, "stop_fusion.sh")
    if os.path.isfile(sf):
        r = subprocess.run(["bash", sf, args.run], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                           universal_newlines=True)
        for ln in r.stdout.splitlines():
            print("   stop_fusion.sh: " + ln, flush=True)
        last = (r.stdout.strip().splitlines() or ["no output"])[-1]
        prog.write("%s  fusion: stop_fusion.sh exit %d - %s" % (prog.label, r.returncode, last[-120:]))
        return
    path = os.path.join(HOME, ".run_records", args.run, "fusion_pids")
    pid = None
    try:
        with open(path) as fh:
            for ln in fh:
                k, _, v = ln.strip().partition("=")
                if k == "BRIDGE_PID" and v.strip().isdigit():
                    pid = int(v.strip())
    except OSError:
        pass
    if pid is None:
        prog.write("%s  fusion: no BRIDGE_PID in %s - nothing to end" % (prog.label, path))
        return
    if not is_program(pid, "robot_bridge_recv.py"):
        prog.write("%s  fusion: bridge receiver %d already gone" % (prog.label, pid))
        return
    os.kill(pid, signal.SIGTERM)
    t = time.time()
    while is_program(pid, "robot_bridge_recv.py") and time.time() - t < 30:
        time.sleep(0.5)
    prog.write("%s  fusion: bridge receiver %d %s" % (
        prog.label, pid, "ended (TERM, %.0f s)" % (time.time() - t)
        if not is_program(pid, "robot_bridge_recv.py") else "STILL RUNNING 30 s after TERM - look at it"))


# ------------------------------------------------------------ progress
class Progress(object):
    def __init__(self, run, max_minutes):
        self.path = os.path.join(HOME, "jobs", "%s_autostop.progress" % run)
        self.label = "%s_AUTOSTOP" % run.upper()
        self.m = int(max_minutes)
        os.makedirs(os.path.dirname(self.path), exist_ok=True)

    def write(self, text):
        tmp = self.path + ".tmp"
        with open(tmp, "w") as fh:
            fh.write(text + "\n")
        os.rename(tmp, self.path)
        print(time.strftime("%H:%M:%S"), text, flush=True)

    def tick(self, elapsed_s, state):
        n = min(int(elapsed_s // 60), self.m - 1)
        self.write("%s  %d/%d min  %s  %ds" % (self.label, n, self.m, state, int(elapsed_s)))


# ---------------------------------------------------------------- watch
def watch(args, prog):
    import rospy
    from nav_msgs.msg import Odometry

    st = {"anchor": None, "still_since": None, "last_msg": None, "msgs": 0, "lost": 0}

    def yaw(q):
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def cb(m):
        now = time.time()
        st["last_msg"] = now
        st["msgs"] += 1
        # RTAB-Map marks a lost frame with a huge covariance (9999) and a null pose.
        if m.pose.covariance[0] >= 9999 or m.twist.covariance[0] >= 9999:
            st["lost"] += 1
            st["anchor"], st["still_since"] = None, None
            return
        p = m.pose.pose
        cur = (p.position.x, p.position.y, yaw(p.orientation))
        a = st["anchor"]
        if a is None:
            st["anchor"], st["still_since"] = cur, now
            return
        moved = math.hypot(cur[0] - a[0], cur[1] - a[1])
        turned = abs(math.degrees((cur[2] - a[2] + math.pi) % (2 * math.pi) - math.pi))
        if moved > args.still_m or turned > args.still_deg:
            st["anchor"], st["still_since"] = cur, now

    rospy.init_node("auto_stop_mapping", anonymous=True, disable_signals=True)
    rospy.Subscriber(args.odom_topic, Odometry, cb, queue_size=5)

    t0 = time.time()
    stop_file = os.path.join(HOME, "slam_series2", "STOP_%s" % args.run)
    park_file = os.path.join(HOME, "slam_series2", "PARK_%s" % args.run)
    park_at = None
    last_tick = 0
    while True:
        now = time.time()
        el = now - t0
        silent = st["last_msg"] is None or now - st["last_msg"] > 10
        still = 0 if (silent or st["still_since"] is None) else now - st["still_since"]

        if os.path.exists(stop_file):
            return "asked to stop now (%s found)" % os.path.basename(stop_file)
        if el >= args.max_minutes * 60:
            return "backstop - %d min with no 'park'" % args.max_minutes
        if park_at is None and os.path.exists(park_file):
            park_at = now
            prog.tick(el, "PARK received - waiting for %d s still before closing" % args.park_hold)
        if park_at is not None:
            if still >= args.park_hold:
                return "parked: 'park' received, then still for %d s" % still
            if now - park_at >= args.park_timeout:
                return "parked: 'park' received %d s ago (never fully still)" % (now - park_at)
        elif args.auto_park and el >= args.min_minutes * 60 and still >= args.still_seconds:
            return "auto-park: still for %d s" % still
        if not procs_named("rtabmap"):
            return None                     # the mapping ended some other way

        if now - last_tick >= 15:
            if st["last_msg"] is None:
                state = "armed, waiting for the first camera position on %s" % args.odom_topic
            elif silent:
                # drive 5 this said "armed" for 9 minutes while the camera was dead. Without
                # positions the 'still' test cannot pass, so 'park' closes after --park-timeout.
                state = ("CAMERA DOWN - NO camera positions for %d s (a 'park' now closes after the "
                         "%d s timeout, not after %d s still)" % (now - st["last_msg"], args.park_timeout,
                                                                  args.park_hold))
                prog.write("!! %s  %d/%d min  %s  %ds" % (prog.label, min(int(el // 60), prog.m - 1),
                                                          prog.m, state, int(el)))
                last_tick = now
                time.sleep(1)
                continue
            elif park_at is not None:
                state = "PARKED - still for %d/%d s before closing" % (still, args.park_hold)
            else:
                state = "mapping - waiting for 'park'"
            prog.tick(el, state)
            last_tick = now
        time.sleep(1)


# ----------------------------------------------------------------- stop
def stop(args, prog, why):
    pids = procs_named("rtabmap")
    if len(pids) != 1:
        prog.write("%s  FAILED - expected one mapping process, found %d (%s); stopped nothing"
                   % (prog.label, len(pids), pids))
        return 2
    pid = pids[0]
    prog.write("%s  stopping: %s - interrupting the mapping (pid %d), waiting for the "
               "database to close" % (prog.label, why, pid))
    t = time.time()
    os.kill(pid, signal.SIGINT)
    while alive(pid):
        w = time.time() - t
        if w > args.close_limit_minutes * 60:
            prog.write("%s  FAILED - the mapping has not exited after %d min; NOT killed. "
                       "Do not power off; look at it by hand" % (prog.label, w // 60))
            return 3
        if int(w) % 15 == 0:
            prog.tick(time.time() - args.t0, "closing the database, %ds so far" % int(w))
        time.sleep(1)
    close_s = time.time() - t

    # The helpers around it: interrupt the mapping's own roslaunch (not the camera's).
    for rl in roslaunch_of(args.launch_file):
        try:
            os.kill(rl, signal.SIGINT)
        except OSError:
            pass
    # And the jobs-page line for this drive (slam_progress.py): once the map is
    # closed its data stops, and left alone it turns to STALLED - a finished
    # write "watcher complete". Matched by WHOLE arguments, never a substring.
    for pid in progress_watchers(args.run):
        try:
            os.kill(pid, signal.SIGTERM)
        except OSError:
            pass
    if args.fusion:
        stop_fusion(args, prog)

    rc = subprocess.call([sys.executable, os.path.join(TOOLS, "check_closed.py"), args.db])
    verdict = "CLOSED PROPERLY" if rc == 0 else "NOT CLOSED PROPERLY"
    prog.write("%s  complete  %s after %.0f s (%s) - %s"
               % (prog.label, verdict, close_s, why, os.path.basename(args.db)))
    return 0 if rc == 0 else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--run", required=True)
    ap.add_argument("--db", default="", help="default ~/slam_series2/<run>.db")
    ap.add_argument("--odom-topic", default="/rtabmap/odom")
    ap.add_argument("--park-hold", type=float, default=60,
                    help="after 'park', seconds of stillness before closing")
    ap.add_argument("--park-timeout", type=float, default=180,
                    help="after 'park', close anyway after this many seconds")
    ap.add_argument("--max-minutes", type=float, default=90,
                    help="last-resort backstop if 'park' never arrives")
    ap.add_argument("--auto-park", action="store_true",
                    help="ALSO stop on stillness alone (off: only 'park' ends a drive)")
    ap.add_argument("--min-minutes", type=float, default=8, help="--auto-park only")
    ap.add_argument("--still-seconds", type=float, default=60, help="--auto-park only")
    ap.add_argument("--still-m", type=float, default=0.05)
    ap.add_argument("--still-deg", type=float, default=3.0)
    ap.add_argument("--close-limit-minutes", type=float, default=30)
    ap.add_argument("--launch-file", default="record_mapping_run.launch")
    ap.add_argument("--fusion", action="store_true",
                    help="after the map closes, also stop ~/.run_records/<run>/fusion_pids")
    args = ap.parse_args()
    args.db = args.db or os.path.join(HOME, "slam_series2", args.run + ".db")

    prog = Progress(args.run, args.max_minutes)
    args.t0 = time.time()
    if not procs_named("rtabmap"):
        prog.write("%s  FAILED - no mapping process (rtabmap) is running; nothing to watch"
                   % prog.label)
        return 2
    why = watch(args, prog)
    if why is None:
        if args.fusion:
            stop_fusion(args, prog)
        prog.write("%s  complete - the mapping ended on its own before any stop was needed; "
                   "run check_closed.py on it" % prog.label)
        return 0
    return stop(args, prog, why)


if __name__ == "__main__":
    sys.exit(main())
