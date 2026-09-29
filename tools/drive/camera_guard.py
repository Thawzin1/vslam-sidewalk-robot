#!/usr/bin/env python3
"""camera_guard.py - notice when the ZED X camera dies during a drive, say so loudly, and
restart it with exactly the command that started it.

RUNS ON: the Jetson. Deployed 2026-09-26 (after review, 15/15 restart test passed); used on drives 6-10. Lives in tools/drive/.

WHY (drive 5, 26 Sept 2026, Hamilton time)
    The camera program (zed_wrapper_node, the ZED X driver) crashed at 02:25:26, twelve minutes
    into the drive (exit -11, a segmentation fault: the program touched memory it did not own and
    the system killed it). Its launch file marks it REQUIRED, so roslaunch (the ROS program that
    starts a group of programs) shut the whole camera down. The map and the blend kept running
    with no pictures, and for 9 minutes nothing said so: the drive's progress line kept saying
    "running". *Plain terms: the eyes went out and nobody was told.*

    usage:  camera_guard.py --run <run_id> --rtabmap-pid <pid> [--launch-pid <pid>]
                            [--max-restarts 5] [--svo 0|1]
            camera_guard.py --selftest        the process checks against names known to be ABSENT

WHAT IT DOES, every 2 s (start_drive.sh starts it detached, right after the mapping starts)
    - is the camera program alive?  By its exact process name ("comm", which Linux cuts to 15
      letters: zed_wrapper_node shows as "zed_wrapper_nod") AND its full program name from
      /proc/<pid>/cmdline, for the ONE process number it is watching - never a search by
      "contains" (ENGINEERING_NOTES.md rule 8).
    - are pictures still arriving?  The time since the last /zedx_front/zed_node/rgb/camera_info
      (a tiny message sent with every picture; the mapping already asks for it, so listening
      costs the camera nothing extra).
    DOWN = the camera process has gone, or no picture for STALE_S (15 s) although it runs
    (a freeze). On DOWN:
      1. writes the alert (one line, below) and "!! CAMERA DOWN" on its progress line;
      2. makes sure the old camera is fully gone - waits for its roslaunch to finish shutting
         down, then SIGINT / SIGTERM / SIGKILL, by process number, ONLY to the roslaunch and
         zed_wrapper_node it adopted or started itself;
      3. starts the camera again with the SAME command line and the SAME environment
         (DISPLAY, XAUTHORITY, depth mode, SVO compression) read from the old roslaunch's
         /proc/<pid>/cmdline and /proc/<pid>/environ - nothing retyped;
      4. waits up to 90 s for pictures, then up to 30 s for /rtabmap/odom (the camera tracker's
         position output) to speak again, and reports both times.
    SVO=1: the camera's own recording is NOT restarted (it would need a new file name and the
    mode checks in start_drive.sh step 3d). The alert says the recording ended at the crash.
    After --max-restarts attempts it gives up, loudly, and keeps reporting until the drive ends.
    It stops by itself when THIS drive's mapping process (--rtabmap-pid, name must read
    "rtabmap") has gone for 2 checks, or when ~/.run_records/<run>/CAMERA_GUARD_STOP exists.
    It never restarts the camera after the drive has ended.

WHAT IT WRITES
    ~/jobs/<run>_camera.progress         the jobs page line (the jobs page (JOBS_PAGE_URL))
    ~/.run_records/<run>/ALERT           ONE line, rewritten on every change of state: what the
                                         main session's watcher turns into a phone notification
    ~/.run_records/<run>/camera_guard.json   state for the drive's progress line (slam_progress.py)
    ~/.run_records/camera_guard_live.json    the same, at a fixed place, for the live map page
    ~/.run_records/<run>/camera_guard.log    everything, with Hamilton times
"""
from __future__ import print_function

import argparse
import json
import os
import signal
import subprocess
import sys
import time

HOME = os.path.expanduser("~")
CAM_COMM = "zed_wrapper_nod"          # what `ps -eo comm` shows (15-letter cut), checked on this Jetson
CAM_EXE = "zed_wrapper_node"          # the full program name, from /proc/<pid>/cmdline
LAUNCH_COMM = "roslaunch"
LAUNCH_FILE = "zedx_front.launch"
INFO_TOPIC = "/zedx_front/zed_node/rgb/camera_info"
ODOM_TOPIC = "/rtabmap/odom"
PERIOD_S = 2.0
STALE_S = 15.0          # no picture this long while the process runs = a freeze
IMAGES_WAIT_S = 90.0    # start_drive.sh gives a fresh camera 90 s too
ODOM_WAIT_S = 30.0
os.environ["TZ"] = "America/Toronto"
time.tzset()


def hms(t=None):
    return time.strftime("%H:%M:%S", time.localtime(t if t is not None else time.time()))


# ------------------------------------------------------------ process checks (by number)
def comm_of(pid):
    try:
        with open("/proc/%d/comm" % int(pid)) as fh:
            return fh.read().strip()
    except (IOError, OSError, ValueError, TypeError):
        return None


def argv_of(pid):
    try:
        with open("/proc/%d/cmdline" % int(pid), "rb") as fh:
            return [a.decode("utf-8", "replace") for a in fh.read().split(b"\0") if a]
    except (IOError, OSError, ValueError, TypeError):
        return []


def ppid_of(pid):
    try:
        with open("/proc/%d/stat" % int(pid)) as fh:
            return int(fh.read().rsplit(")", 1)[1].split()[1])
    except (IOError, OSError, ValueError, TypeError, IndexError):
        return None


def is_camera(pid):
    """The ZED driver: short name AND full program name match, whole words."""
    a = argv_of(pid)
    return comm_of(pid) == CAM_COMM and bool(a) and os.path.basename(a[0]) == CAM_EXE


def is_camera_launch(pid):
    """A roslaunch whose arguments include zedx_front.launch as a whole argument.

    Started as `roslaunch ...` its comm is "roslaunch"; started as `python3 .../roslaunch ...`
    its comm is "python3" (found in the first live test, 2026-09-26). Both forms are accepted,
    each by WHOLE names: comm roslaunch, or comm python3 with argv[1]'s base name roslaunch."""
    a = argv_of(pid)
    c = comm_of(pid)
    if not (c == LAUNCH_COMM or (c == "python3" and len(a) > 1 and os.path.basename(a[1]) == LAUNCH_COMM)):
        return False
    i = 1 if c == LAUNCH_COMM else 2
    return any(os.path.basename(x) == LAUNCH_FILE for x in a[i:])


def is_rtabmap(pid):
    return comm_of(pid) == "rtabmap"


def all_pids():
    me = {os.getpid(), os.getppid()}
    for d in os.listdir("/proc"):
        if d.isdigit() and int(d) not in me:
            yield int(d)


def cameras():
    return [p for p in all_pids() if is_camera(p)]


def camera_launches():
    return [p for p in all_pids() if is_camera_launch(p)]


def selftest():
    """Every check asked about something known NOT to be there must say no (rule 8)."""
    absent = 999999999
    me = os.getpid()
    sleeper = subprocess.Popen(["sleep", "30"])           # a real process with the wrong name
    # decoys whose command lines CONTAIN every name we look for - a "contains" check says yes
    decoy1 = subprocess.Popen(["bash", "-c", "sleep 30 # zed_wrapper_node roslaunch zedx_front.launch rtabmap"])
    decoy2 = subprocess.Popen(["bash", "-c", "exec -a zed_wrapper_node sleep 30"])   # right argv[0], wrong comm
    time.sleep(0.3)
    try:
        checks = {
            "is_camera(absent pid)": is_camera(absent),
            "is_camera(this guard)": is_camera(me),
            "is_camera(a sleep)": is_camera(sleeper.pid),
            "is_camera(None)": is_camera(None),
            "is_camera_launch(absent pid)": is_camera_launch(absent),
            "is_camera_launch(this guard)": is_camera_launch(me),
            "is_rtabmap(absent pid)": is_rtabmap(absent),
            "is_rtabmap(a sleep)": is_rtabmap(sleeper.pid),
            "is_camera(decoy naming everything)": is_camera(decoy1.pid),
            "is_camera_launch(decoy naming everything)": is_camera_launch(decoy1.pid),
            "is_rtabmap(decoy naming everything)": is_rtabmap(decoy1.pid),
            "is_camera(decoy with argv[0] zed_wrapper_node)": is_camera(decoy2.pid),
            "this guard in cameras()": me in cameras(),
            "this guard in camera_launches()": me in camera_launches(),
        }
    finally:
        for p in (sleeper, decoy1, decoy2):
            p.kill()
            p.wait()
    bad = [k for k, v in checks.items() if v]
    for k, v in checks.items():
        print("  %-46s %s" % (k, "yes  <- WRONG" if v else "no   (right)"))
    # and the positive side, against whatever is really running now
    print("  cameras running now: %s   camera launches now: %s" % (cameras(), camera_launches()))
    print("SELFTEST %s" % ("FAILED: " + ", ".join(bad) if bad else "PASSED (all absent names say no)"))
    return 1 if bad else 0


# ------------------------------------------------------------ the guard
class Guard(object):
    def __init__(self, a):
        self.a = a
        self.run = a.run
        self.rec = os.path.join(HOME, ".run_records", a.run)
        os.makedirs(self.rec, exist_ok=True)
        os.makedirs(os.path.join(HOME, "jobs"), exist_ok=True)
        self.prog = os.path.join(HOME, "jobs", "%s_camera.progress" % a.run)
        self.alert = os.path.join(self.rec, "ALERT")
        self.state_run = os.path.join(self.rec, "camera_guard.json")
        self.state_live = os.path.join(HOME, ".run_records", "camera_guard_live.json")
        self.logf = open(os.path.join(self.rec, "camera_guard.log"), "a", buffering=1)
        self.stop_file = os.path.join(self.rec, "CAMERA_GUARD_STOP")
        self.outages = os.path.join(self.rec, "camera_outages.csv")
        self.cause = ""
        self.t0 = time.time()
        self.launch = None      # the roslaunch we own (adopted or started)
        self.cam = None         # its zed_wrapper_node
        self.argv = None        # the exact camera command line
        self.env = None         # and its environment
        self.restarts = 0       # successful restarts
        self.attempts = 0       # restart attempts, successful or not
        self.state = "starting"
        self.down_since = None
        self.last_event = ""
        self.info_rx = None     # time of the last camera_info
        self.odom_rx = None     # time of the last /rtabmap/odom
        self.odom_lost = None   # was the last odom a "lost" one (covariance 9999)?

    # ---- output
    def log(self, text):
        line = "%s %s" % (time.strftime("%F %T %Z"), text)
        print(line, flush=True)
        self.logf.write(line + "\n")

    def _atomic(self, path, text):
        tmp = path + ".tmp"
        with open(tmp, "w") as fh:
            fh.write(text)
        os.replace(tmp, path)

    def record_outage(self, outcome):
        """One row per outage (review item 4): the results pack turns these windows into distance
        driven on the blend alone (wheels + gyroscope, no camera) and resets per 100 m (ENGINEERING_NOTES.md 2.4)."""
        if self.down_since is None:
            return
        end = time.time()
        new = not os.path.exists(self.outages)
        with open(self.outages, "a") as fh:
            if new:
                fh.write("start_hamilton,end_hamilton,start_epoch,end_epoch,seconds,cause,outcome\n")
            fh.write("%s,%s,%.3f,%.3f,%.1f,%s,%s\n" % (
                time.strftime("%F %T", time.localtime(self.down_since)), time.strftime("%F %T", time.localtime(end)),
                self.down_since, end, end - self.down_since, self.cause.replace(",", ";"), outcome.replace(",", ";")))
        self.log("outage recorded: %.1f s, %s" % (end - self.down_since, outcome))

    def raise_alert(self, text):
        """ONE line in ALERT, rewritten on every change of state (the watcher notifies on change)."""
        self.last_event = text
        self._atomic(self.alert, "%s %s %s\n" % (hms(), self.run, text))
        self.log("ALERT: " + text)

    def publish(self):
        now = time.time()
        el = int(now - self.t0)
        m = min(el // 60, self.a.max_minutes - 1)
        img = "never" if self.info_rx is None else "%.1f s old" % (now - self.info_rx)
        if self.state == "ok":
            what = "ok, pictures %s, restarts %d of %d" % (img, self.restarts, self.a.max_restarts)
            if self.restarts:
                what = "CAMERA RESTARTED %d (last: %s), pictures %s" % (self.restarts, self.last_event, img)
        elif self.state == "down":
            what = "!! CAMERA DOWN since %s - %s" % (hms(self.down_since), self.last_event)
        elif self.state == "gave_up":
            what = "!! CAMERA DOWN since %s - GAVE UP after %d attempts, the map has no pictures" % (
                hms(self.down_since), self.attempts)
        elif self.state == "ended":
            what = "complete - drive over, restarts %d" % self.restarts
        else:
            what = "starting - %s" % self.last_event
        line = "CAMERA %s  %d/%d min  %s  %ds" % (self.run.upper(), m, self.a.max_minutes, what, el)
        if what.startswith("!!"):
            line = "!! " + line.replace("  !! ", "  ")      # "!!" first = a red card on the jobs page
        self._atomic(self.prog, line + "\n")
        st = {"run": self.run, "state": self.state, "t": now,
              "down_since": self.down_since, "down_since_hms": hms(self.down_since) if self.down_since else None,
              "restarts": self.restarts, "attempts": self.attempts, "max_restarts": self.a.max_restarts,
              "last_event": self.last_event, "image_age_s": None if self.info_rx is None else now - self.info_rx,
              "guard_pid": os.getpid(), "camera_pid": self.cam, "launch_pid": self.launch}
        js = json.dumps(st)
        self._atomic(self.state_run, js + "\n")
        self._atomic(self.state_live, js + "\n")

    # ---- ROS
    def ros_start(self):
        import rospy
        from sensor_msgs.msg import CameraInfo
        from nav_msgs.msg import Odometry
        rospy.init_node("camera_guard", anonymous=True, disable_signals=True)

        def info_cb(_m):
            self.info_rx = time.time()

        def odom_cb(m):
            self.odom_rx = time.time()
            self.odom_lost = m.pose.covariance[0] >= 9999 or m.twist.covariance[0] >= 9999

        rospy.Subscriber(INFO_TOPIC, CameraInfo, info_cb, queue_size=1)
        rospy.Subscriber(ODOM_TOPIC, Odometry, odom_cb, queue_size=1)

    # ---- adopting and controlling our camera
    def adopt(self):
        """Take the camera start_drive.sh just started as ours, and copy its command + environment."""
        launches = []
        if self.a.launch_pid:
            if is_camera_launch(self.a.launch_pid):
                launches = [self.a.launch_pid]
            else:
                self.log("given launch pid %d is not a camera roslaunch - looking for the one that is"
                         % self.a.launch_pid)
        if not launches:
            launches = camera_launches()
        if len(launches) != 1:
            self.log("!! cannot adopt a camera: %d camera roslaunch processes (%s) - need exactly one"
                     % (len(launches), launches))
            return False
        self.launch = launches[0]
        self.argv = argv_of(self.launch)
        # /proc shows "/usr/bin/python3 /opt/ros/noetic/bin/roslaunch ...": run the roslaunch script
        # itself, as start_drive.sh does, so the restarted camera's name is "roslaunch" again
        if len(self.argv) > 1 and os.path.basename(self.argv[0]).startswith("python") \
                and os.path.basename(self.argv[1]) == LAUNCH_COMM:
            self.argv = self.argv[1:]
        try:
            with open("/proc/%d/environ" % self.launch, "rb") as fh:
                self.env = dict(kv.split(b"=", 1) for kv in fh.read().split(b"\0") if b"=" in kv)
            self.env = {k.decode(): v.decode("utf-8", "replace") for k, v in self.env.items()}
        except (IOError, OSError) as e:
            self.log("!! cannot read the camera's environment: %s" % e)
            return False
        self.cam = self.find_child_camera(self.launch)
        self.log("adopted camera: roslaunch pid %d, zed_wrapper_node pid %s" % (self.launch, self.cam))
        self.log("  command: %s" % " ".join(self.argv))
        self.log("  DISPLAY=%s XAUTHORITY=%s" % (self.env.get("DISPLAY"), self.env.get("XAUTHORITY")))
        return True

    def find_child_camera(self, launch):
        kids = [p for p in cameras() if ppid_of(p) == launch]
        return kids[0] if len(kids) == 1 else None

    def wait_gone(self, pid, check, seconds):
        # an outage, or the drive line, the map banner and the jobs card drop the alarm mid-outage
        end = time.time() + seconds
        while time.time() < end:
            if not check(pid):
                return True
            self.publish()
            time.sleep(0.5)
        return not check(pid)

    def clear_old(self, self_exit_wait):
        """Make sure OUR old camera is completely gone. Signals only numbers we own, re-checked."""
        if self.launch and is_camera_launch(self.launch):
            # a REQUIRED node dying makes roslaunch shut down by itself: after a crash, give it the
            # chance first; after a freeze (driver alive, silent) there is nothing to wait for
            if not self.wait_gone(self.launch, is_camera_launch, self_exit_wait):
                for sig, wait in ((signal.SIGINT, 15), (signal.SIGTERM, 5), (signal.SIGKILL, 3)):
                    if not is_camera_launch(self.launch):
                        break
                    self.log("old roslaunch %d still up - sending %s" % (self.launch, sig.name))
                    os.kill(self.launch, sig)
                    self.wait_gone(self.launch, is_camera_launch, wait)
        own = [p for p in (self.launch, self.cam) if p]
        if self.cam and is_camera(self.cam):
            for sig, wait in ((signal.SIGTERM, 5), (signal.SIGKILL, 3)):
                if not is_camera(self.cam):
                    break
                self.log("old zed_wrapper_node %d still up - sending %s" % (self.cam, sig.name))
                os.kill(self.cam, sig)
                self.wait_gone(self.cam, is_camera, wait)
        mine = [p for p in own if is_camera(p) or is_camera_launch(p)]
        if mine:
            self.log("!! OUR OLD CAMERA WILL NOT DIE (pids %s, even after SIGKILL) - NOT starting another" % mine)
            return "own"
        left = cameras()
        if left:
            # somebody else's camera: never start a second one beside it
            self.log("!! another camera process is running: %s - NOT starting another" % left)
            return "other"
        return None

    def relaunch(self):
        t = time.time()
        out = open(os.path.join(self.rec, "camera.log"), "ab")
        out.write(("\n===== camera_guard restart attempt %d at %s =====\n"
                   % (self.attempts, time.strftime("%F %T %Z"))).encode())
        out.flush()
        p = subprocess.Popen(self.argv, env=self.env, stdin=subprocess.DEVNULL, stdout=out,
                             stderr=subprocess.STDOUT, start_new_session=True, cwd=HOME)
        out.close()
        self.launch = p.pid
        self._child = p
        self.cam = None
        self.log("restart attempt %d: roslaunch pid %d started" % (self.attempts, p.pid))
        # pictures back?
        while time.time() - t < IMAGES_WAIT_S:
            if p.poll() is not None:
                self.log("!! the new roslaunch exited (code %s) after %.0f s" % (p.returncode, time.time() - t))
                return None
            if self.drive_over():
                self.log("the drive ended while waiting for pictures - stop waiting (the camera just "
                         "started is left running; the next drive restarts it fresh)")
                return None
            if self.cam is None:
                self.cam = self.find_child_camera(self.launch)
            if self.info_rx is not None and self.info_rx > t and self.cam:
                return self.info_rx - t
            self.publish()
            time.sleep(0.5)
        self.log("!! no pictures %.0f s after the restart" % IMAGES_WAIT_S)
        return None

    def recover(self, reason, crashed):
        self.state = "down"
        self.down_since = time.time()
        self.cause = reason
        svo = " The camera's own recording (SVO) ended at the crash and is NOT restarted." if self.a.svo else ""
        self.raise_alert("CAMERA DOWN at %s (%s) - restarting.%s" % (hms(self.down_since), reason, svo))
        self.publish()
        while self.attempts < self.a.max_restarts:
            if self.drive_over():
                self.log("the drive ended while the camera was down - not restarting")
                return
            self.attempts += 1
            self.last_event = "restart attempt %d of %d" % (self.attempts, self.a.max_restarts)
            self.publish()
            blocked = self.clear_old(20 if crashed and self.attempts == 1 else 0)
            if blocked == "own":
                self.raise_alert("CAMERA DOWN since %s - the old camera program WILL NOT DIE (even SIGKILL); "
                                 "not starting a second one - attempt %d of %d (see camera_guard.log)"
                                 % (hms(self.down_since), self.attempts, self.a.max_restarts))
            elif blocked == "other":
                self.raise_alert("CAMERA DOWN since %s - ANOTHER camera program (not this drive's) is in the "
                                 "way; not starting a second one - attempt %d of %d"
                                 % (hms(self.down_since), self.attempts, self.a.max_restarts))
            if blocked:
                for _ in range(20):
                    self.publish()
                    time.sleep(0.5)
                continue
            if self.drive_over():     # review item 3: never start a camera after the map has closed
                self.log("the drive ended while the old camera was being cleared - not restarting")
                return
            dt = self.relaunch()
            if dt is None:
                self.raise_alert("CAMERA DOWN since %s - restart attempt %d of %d FAILED"
                                 % (hms(self.down_since), self.attempts, self.a.max_restarts))
                continue
            self.restarts += 1
            down_s = time.time() - self.down_since
            self.log("pictures back %.1f s after restart attempt %d (camera down %.1f s in all)"
                     % (dt, self.attempts, down_s))
            t_back = time.time()
            odom = "no mapping to check"
            if is_rtabmap(self.a.rtabmap_pid):
                while time.time() - t_back < ODOM_WAIT_S:
                    if self.odom_rx is not None and self.odom_rx > t_back:
                        break
                    self.publish()
                    time.sleep(0.5)
                if self.odom_rx is not None and self.odom_rx > t_back:
                    odom = "camera positions back after %.1f s%s" % (
                        self.odom_rx - t_back, " but LOST (null position)" if self.odom_lost else "")
                else:
                    odom = "!! camera positions NOT back after %.0f s" % ODOM_WAIT_S
            self.log("odometry: " + odom)
            self.state = "ok"
            self.raise_alert("CAMERA RESTARTED %d at %s - down %.0f s, pictures back %.0f s after "
                             "the restart; %s%s" % (self.restarts, hms(), down_s, dt, odom,
                                                    " - SVO recording NOT restarted" if self.a.svo else ""))
            self.record_outage("restarted (%d); %s" % (self.restarts, odom))
            self.down_since = None
            self.publish()
            return
        self.state = "gave_up"
        self.raise_alert("CAMERA DOWN since %s - GAVE UP after %d restart attempts. The map has no "
                         "pictures: say 'park' / STOP and restart the camera by hand"
                         % (hms(self.down_since), self.attempts))
        self.publish()

    def drive_over(self):
        return os.path.exists(self.stop_file) or not is_rtabmap(self.a.rtabmap_pid)

    def loop(self):
        if not is_rtabmap(self.a.rtabmap_pid) and not os.path.exists(self.stop_file):
            self.log("!! pid %s is not rtabmap - nothing to guard" % self.a.rtabmap_pid)
            self.last_event = "no mapping process %s" % self.a.rtabmap_pid
            self.state = "ended"
            self.publish()
            return 1
        self.ros_start()
        if not self.adopt():
            self.state = "down"
            self.down_since = time.time()
            self.raise_alert("CAMERA GUARD could not find the camera it should watch - camera NOT guarded")
            self.publish()
            return 1
        self.state = "ok"
        self.last_event = "watching"
        self.log("watching: rtabmap pid %s, max %d restarts, SVO=%d, stop file %s"
                 % (self.a.rtabmap_pid, self.a.max_restarts, self.a.svo, self.stop_file))
        gone = 0
        started = time.time()
        while True:
            if os.path.exists(self.stop_file):
                why = "stop file %s" % self.stop_file
                break
            gone = 0 if is_rtabmap(self.a.rtabmap_pid) else gone + 1
            if gone >= 2:
                why = "mapping pid %s has exited (drive over)" % self.a.rtabmap_pid
                break
            if self.state == "ok":
                now = time.time()
                alive = self.cam is not None and is_camera(self.cam)
                if self.cam is None:
                    self.cam = self.find_child_camera(self.launch)
                    alive = self.cam is not None
                ref = self.info_rx if self.info_rx is not None else started
                if not alive:
                    self.recover("camera process gone", True)
                elif now - ref > STALE_S:
                    self.recover("no pictures for %.0f s although the camera process runs" % (now - ref), False)
            self.publish()
            time.sleep(PERIOD_S)
        self.log("stopping: %s. Camera left %s (it is restarted fresh at the next drive)"
                 % (why, "running" if self.cam and is_camera(self.cam) else "NOT running"))
        if self.state in ("down", "gave_up"):
            self.record_outage("still down when the drive ended (%s)" % self.state)
        if self.state != "gave_up":
            self.state = "ended"
        self.publish()
        return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--run")
    ap.add_argument("--rtabmap-pid", type=int)
    ap.add_argument("--launch-pid", type=int, default=0,
                    help="the camera's roslaunch process number (start_drive.sh knows it); "
                         "0 = find the one camera roslaunch running")
    ap.add_argument("--max-restarts", type=int, default=5)
    ap.add_argument("--max-minutes", type=int, default=90, help="only for the N/M on the jobs page")
    ap.add_argument("--svo", type=int, default=0)
    a = ap.parse_args()
    if a.selftest:
        return selftest()
    if not a.run or not a.rtabmap_pid:
        ap.error("--run and --rtabmap-pid are required")
    g = Guard(a)
    def _leave(signum, frame):
        raise KeyboardInterrupt(signal.Signals(signum).name)
    for s in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(s, _leave)
    try:
        return g.loop()
    except KeyboardInterrupt:
        g.log("stopped by a signal")
        if g.state in ("down", "gave_up"):
            g.record_outage("still down when the guard was stopped by a signal")
        g.state = "ended" if g.state != "gave_up" else g.state
        g.last_event = "guard stopped by a signal"
        g.publish()
        return 0


if __name__ == "__main__":
    sys.exit(main())
