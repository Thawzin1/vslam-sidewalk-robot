#!/usr/bin/env python3
"""
preflight_check.py — Run this before every experiment. It changes nothing.

WHY THIS EXISTS
    Robotics experiments fail for boring reasons. The disk was full. The clock
    had drifted. A transform was missing so every measurement was silently
    referenced to the wrong place. The camera was publishing at 12 FPS instead
    of 30 because of a bandwidth limit nobody checked.

    Each of these produces data that LOOKS fine and is worthless. You often do
    not discover the problem until you sit down to analyse a day of recordings.

    So this script checks the boring things first, and refuses to give a green
    light unless they pass. Two minutes here routinely saves a day.

    It is deliberately read-only. It starts nothing, changes nothing, and can
    be run at any time without side effects.

DIFFERENCE FROM tools/health_check.sh
    health_check.sh checks the MACHINE — driver installed, camera enumerated,
    SDK present. Run it after provisioning or when something is broken.

    This script checks the RUNNING ROS SYSTEM — are topics actually flowing, is
    the transform tree complete, are timestamps sane. Run it once the system is
    up and you are about to record data that matters.

USAGE
    roslaunch sidewalk_bringup preflight.launch                 # PREFERRED
    roslaunch sidewalk_bringup preflight.launch strict:=true    # warnings fail too

    Prefer the launch file. It is what supplies the two lists this script
    checks against — the topics with their expected rates, and the transform
    frames — and without them the two most valuable checks have nothing to
    check. Run it AFTER bringup.launch, which is what puts the robot's geometry
    on the parameter server.

    rosrun sidewalk_bringup preflight_check.py                  # fallback
    rosrun sidewalk_bringup preflight_check.py _strict:=true

    The rosrun form still works and is useful for a quick look at the disk,
    clock and ROS-environment checks. It sets no private parameters, so the
    frame list is taken from /sidewalk/expected_frames if bringup.launch has
    been run, and there is no topic list at all. Either shortfall is reported
    as a WARNING rather than passed over in silence — see check_topics and
    check_transforms for why that distinction is the whole point.
"""

import os
import shutil
import subprocess
import sys

import rospy

try:
    try:
        # Canonical import: works from a source workspace and
        # after catkin_make install, because sidewalk_bringup
        # exports src/ via catkin_python_setup().
        from sidewalk_bringup.run_logger import RunLogger
    except ImportError:
        from run_logger import RunLogger
except ImportError:                                        # pragma: no cover
    class RunLogger:
        def __init__(self, *a, **k): pass
        def info(self, m): print(m)
        def warn(self, m): print("WARN: " + m)
        def error(self, m): print("ERROR: " + m)
        def metric(self, k, v, unit=""): print(f"{k}={v}{unit}")
        def section(self, t): print(f"--- {t} ---")
        def summary(self, t, status=""): print(t)


# Minimum free space before recording. An SVO2 stereo recording at 1920x1200
# consumes several GB per minute, and a filesystem that fills mid-run corrupts
# the recording AND can destabilise the machine.
MIN_FREE_GB = 10.0

# The camera is expected to hold close to its configured rate. Sustained
# shortfall usually means a bandwidth or power problem, not a software one.
#
# THIS IS A MULTIPLIER ON WHATEVER expected_topics DECLARES, so the floor moves
# with that list and only with it: 15 Hz declared gives a 12.75 Hz floor, 200 Hz
# to publish 15, and 400.0 for a 200 Hz IMU, which put the floors at 25.5 and
# 340 — both unreachable by healthy hardware, so both gates warned on every run.
# The tolerance was never the fault. What it multiplied was.
#
# IT IS ONE-SIDED BY DESIGN — check_topics() below compares only
# `hz < want_hz * FPS_TOLERANCE` and has no upper bound. A rate ABOVE the
# expected one passes silently. That makes an over-declared rate loud and an
# under-declared rate mute, which is the asymmetry to keep in mind when editing
# the list in preflight.launch.
#
# Unlike sidewalk_perception, this figure is a module constant with no parameter
# binding: it cannot be overridden from a launch file. The equivalent number
# there lives in config/validation_thresholds.yaml as thresholds/fps_tolerance_pct
# (5%, i.e. a much tighter band). The two are independent and can drift apart.
FPS_TOLERANCE = 0.85

# Where the transform-tree frame list comes from if this node's own
# ~expected_frames was not set.
#
# THE LIST HAS ONE HOME: sidewalk_bringup/config/robot_frames.yaml, which is the
# file that owns the robot's geometry. It reaches this script by two routes:
#
#   ~expected_frames          preflight.launch loads robot_frames.yaml into this
#                             node's PRIVATE namespace, so the file's top-level
#                             expected_frames: key arrives here directly. This
#                             is the normal route and the one with teeth.
#   /sidewalk/expected_frames bringup.launch loads the same file under the
#                             /sidewalk/ namespace, for everything else in the
#                             project that wants the geometry. Used here as a
#                             fallback so that a bare
#                             `rosrun sidewalk_bringup preflight_check.py`,
#                             which sets no private parameters at all, still
#                             checks the real list instead of silently checking
#                             nothing.
#
# WHY A FALLBACK RATHER THAN A HARD REQUIREMENT. The docstring above advertises
# the bare rosrun form, and under rosrun there is no launch file to set private
# parameters. Before this fallback existed that invocation reached
# check_transforms() with an empty list and printed "No expected frames
# configured — skipping" — a clean run with the most dangerous check switched
# off. If BOTH routes are empty the script now says so as a WARNING, not as a
# note, so that strict:=true refuses to give a green light.
FRAMES_FALLBACK_PARAM = "/sidewalk/expected_frames"


class Preflight:
    def __init__(self):
        rospy.init_node("preflight_check", anonymous=True)
        self.log = RunLogger("sidewalk_bringup", run_name="preflight")
        self.strict = rospy.get_param("~strict", False)

        # Whether a camera is supposed to be running. Without this the script
        # cannot tell "deliberately running without the camera" from "the topic
        # list failed to arrive", and would have to treat an empty list as
        # normal in both cases — which is how a check ends up silently doing
        # nothing. Defaults to true: if nobody has said otherwise, an empty
        # topic list is a misconfiguration and should be said out loud.
        self.camera_expected = rospy.get_param("~camera_expected", True)

        # Which topics to check, and the rate each should hold. Overridable
        # from a launch file so different phases can check different things.
        self.expected_topics = self._load_expected_topics()

        # The frame list, and WHERE IT CAME FROM. The source is carried around
        # and printed rather than thrown away, because the failure this gate
        # from a place nobody realised it was arriving from. Printing the source
        # makes that visible on every run instead of on the day it goes wrong.
        self.expected_frames, self.frames_source = self._load_expected_frames()

        self.passed, self.warned, self.failed = [], [], []

    # ------------------------------------------------------------------
    def _load_expected_topics(self):
        """Read ~expected_topics and return it as a list of (topic, want_hz).

        ACCEPTS TWO SHAPES, and the reason is worth reading before simplifying
        this away.

        The list-of-pairs shape is the correct one:

            expected_topics:
              - {topic: /some/topic, hz: 30.0}

        The mapping shape — `/some/topic: 30.0` — is what this file used to be
        given, and it DOES NOT SURVIVE roslaunch. When a <rosparam> value is a
        dictionary, roslaunch sets each key as its own parameter, and a key
        beginning with "/" is a global name. Topic keys all begin with "/", so
        every entry was written to the global parameter server instead of into
        ~expected_topics, which was left with no children at all. The check then
        found an empty dictionary and skipped itself on every run since it was
        written. See the long note in preflight.launch for the verification.

        The mapping shape is still accepted here so that anything setting it
        programmatically — where the key-splitting does not apply, because
        rospy.set_param on a whole dict stores it under the given name — keeps
        working. It is not the shape to write in a launch file.
        """
        raw = rospy.get_param("~expected_topics", None)
        if not raw:
            return []

        if isinstance(raw, dict):
            return [(topic, hz) for topic, hz in raw.items()]

        pairs = []
        for entry in raw:
            if isinstance(entry, dict) and "topic" in entry:
                pairs.append((entry["topic"], entry.get("hz")))
            elif isinstance(entry, (list, tuple)) and len(entry) == 2:
                pairs.append((entry[0], entry[1]))
            else:
                # Loud, not skipped. A malformed entry means one fewer thing
                # being checked, and the whole point of this script is that a
                # check which quietly does not happen is worse than no check.
                self.log.warn(f"Ignoring malformed ~expected_topics entry: "
                              f"{entry!r} — want {{topic: /a/b, hz: 30.0}}")
        return pairs

    def _load_expected_frames(self):
        """Return (frames, human-readable description of where they came from).

        Tries this node's own parameter first so that an explicit override in a
        launch file always wins, then falls back to the copy bringup.launch
        loads. See FRAMES_FALLBACK_PARAM above for why both routes exist.
        """
        frames = rospy.get_param("~expected_frames", None)
        if frames:
            return frames, ("~expected_frames — this node's own parameter, "
                            "set by preflight.launch from "
                            "config/robot_frames.yaml")

        frames = rospy.get_param(FRAMES_FALLBACK_PARAM, None)
        if frames:
            # Loud on purpose. Reaching the fallback means preflight.launch was
            # not the thing that started this node, so anything it would
            # normally configure is also absent. Worth knowing before reading
            # the results.
            self.log.warn(
                f"~expected_frames was not set; falling back to "
                f"{FRAMES_FALLBACK_PARAM}. That is the same list, loaded from "
                f"config/robot_frames.yaml by bringup.launch, so the frame "
                f"check below is still meaningful — but this node was not "
                f"started by preflight.launch, so no topic list was supplied "
                f"either. Prefer: roslaunch sidewalk_bringup preflight.launch")
            return frames, (f"{FRAMES_FALLBACK_PARAM} — loaded from "
                            f"config/robot_frames.yaml by bringup.launch")

        return [], ""

    # ------------------------------------------------------------------
    def ok(self, msg):
        self.passed.append(msg)
        self.log.info(f"PASS  {msg}")

    def warn(self, msg, fix=""):
        self.warned.append((msg, fix))
        self.log.warn(f"WARN  {msg}" + (f"  -> {fix}" if fix else ""))

    def bad(self, msg, fix=""):
        self.failed.append((msg, fix))
        self.log.error(f"FAIL  {msg}" + (f"  -> {fix}" if fix else ""))

    # ------------------------------------------------------------------
    def check_disk(self):
        self.log.section("Disk space")
        repo = os.environ.get("SIDEWALK_REPO",
                              os.path.expanduser("~/vslam-sidewalk-robot"))
        target = repo if os.path.exists(repo) else os.path.expanduser("~")
        free_gb = shutil.disk_usage(target).free / (1024 ** 3)
        self.log.metric("free_disk", round(free_gb, 1), "GB")

        if free_gb < MIN_FREE_GB / 2:
            self.bad(f"Only {free_gb:.1f} GB free — recording will fail",
                     "Free space or move recordings to external storage")
        elif free_gb < MIN_FREE_GB:
            self.warn(f"{free_gb:.1f} GB free (want {MIN_FREE_GB:.0f}+)",
                      "Stereo recording consumes several GB per minute")
        else:
            self.ok(f"{free_gb:.1f} GB free")

    def check_clock(self):
        self.log.section("Time base")
        try:
            out = subprocess.run(["timedatectl", "show", "-p",
                                  "NTPSynchronized", "--value"],
                                 capture_output=True, text=True, timeout=5)
            if out.stdout.strip() == "yes":
                self.ok("Clock is NTP-synchronized")
            else:
                self.warn("Clock is not NTP-synchronized",
                          "Timestamp mismatch corrupts sensor fusion and "
                          "makes multi-machine recordings unalignable")
        except Exception as ex:
            self.warn(f"Could not check clock sync ({ex})")

        # use_sim_time being wrongly set is a classic and very confusing fault:
        # if it is true with no clock publisher, every node blocks forever
        # waiting for time that never arrives, and the system just... hangs.
        if rospy.get_param("/use_sim_time", False):
            self.warn("/use_sim_time is TRUE",
                      "Correct only when replaying a bag with --clock. "
                      "Otherwise nodes will hang waiting for a clock.")
        else:
            self.ok("/use_sim_time is false (live operation)")

    def check_ros_env(self):
        self.log.section("ROS environment")
        ros_ip = os.environ.get("ROS_IP")
        master = os.environ.get("ROS_MASTER_URI", "<unset>")
        self.log.info(f"ROS_MASTER_URI = {master}")

        if not ros_ip:
            # This exact fault existed on the previous system.
            self.warn("ROS_IP is not set",
                      "Nodes may advertise an address others cannot reach, "
                      "giving topics that connect but never deliver data. "
                      "Run: source ~/.sidewalk_env.sh")
        else:
            self.ok(f"ROS_IP = {ros_ip}")

    def check_topics(self):
        self.log.section("Topics")
        if not self.expected_topics:
            if self.camera_expected:
                # A camera is supposed to be running and nothing told us what to
                # check on it. Say so as a warning rather than a note: with
                # strict:=true this now blocks, which is the correct answer
                # before recording data that matters.
                self.warn("No expected topics configured, but a camera is "
                          "expected — the topic and frame-rate checks did not "
                          "run",
                          "Start this with `roslaunch sidewalk_bringup "
                          "preflight.launch`, which supplies the list. If the "
                          "list IS set, check it is a YAML list of "
                          "{topic: ..., hz: ...} pairs and not a mapping — a "
                          "mapping's keys escape to global parameters")
            else:
                self.log.info("No expected topics configured and "
                              "camera_expected is false — skipping by request")
            return

        try:
            published = {name for name, _ in rospy.get_published_topics()}
        except Exception as ex:
            self.bad(f"Cannot reach the ROS master ({ex})",
                     "Is roscore running?")
            return

        for topic, want_hz in self.expected_topics:
            if topic not in published:
                self.bad(f"Topic missing: {topic}",
                         "The node that should publish it is not running")
                continue

            hz = self._measure_hz(topic)
            if hz is None:
                self.bad(f"{topic} exists but delivered no messages",
                         "Publisher is registered but not actually sending — "
                         "often a camera that enumerated but is not streaming")
            elif want_hz and hz < want_hz * FPS_TOLERANCE:
                self.warn(f"{topic} at {hz:.1f} Hz, expected ~{want_hz} Hz",
                          "Check bandwidth, capture-card power margin, and "
                          "whether two cameras are sharing a deserializer group")
                self.log.metric(f"hz{topic.replace('/', '_')}", round(hz, 2), "Hz")
            else:
                self.ok(f"{topic} at {hz:.1f} Hz")
                self.log.metric(f"hz{topic.replace('/', '_')}", round(hz, 2), "Hz")

    def _measure_hz(self, topic, samples=20, timeout=5.0):
        """Measure a topic's real rate by timing actual arrivals.

        Deliberately measures rather than trusting the configured value. The
        gap between "configured 30 FPS" and "delivering 30 FPS" is precisely
        where bandwidth and power problems hide.
        """
        import rostopic
        stamps = []

        try:
            msg_class, real_topic, _ = rostopic.get_topic_class(topic, blocking=False)
            if msg_class is None:
                return None
        except Exception:
            return None

        def cb(_msg):
            stamps.append(rospy.Time.now().to_sec())

        sub = rospy.Subscriber(real_topic, msg_class, cb)
        deadline = rospy.Time.now().to_sec() + timeout
        rate = rospy.Rate(50)
        while len(stamps) < samples and rospy.Time.now().to_sec() < deadline:
            if rospy.is_shutdown():
                break
            rate.sleep()
        sub.unregister()

        if len(stamps) < 2:
            return None
        span = stamps[-1] - stamps[0]
        return (len(stamps) - 1) / span if span > 0 else None

    def check_transforms(self):
        self.log.section("Transform tree")
        if not self.expected_frames:
            # LOUD, not a note. This is the single most dangerous check in the
            # script — a missing transform stops nothing and silently references
            # every measurement to the wrong place — so "the check did not run"
            # must not read like "the check passed". As a warning it is listed
            # in the summary and, under strict:=true, it blocks.
            self.warn("No expected frames configured — THE TRANSFORM-TREE "
                      "CHECK DID NOT RUN",
                      "Neither ~expected_frames nor "
                      f"{FRAMES_FALLBACK_PARAM} is set. Run "
                      "`roslaunch sidewalk_bringup preflight.launch`, or start "
                      "bringup.launch first so config/robot_frames.yaml is on "
                      "the parameter server")
            return

        # Which list was used, printed every run. See __init__ for why.
        self.log.info(f"Frame list source: {self.frames_source}")

        try:
            import tf2_ros
            buf = tf2_ros.Buffer()
            tf2_ros.TransformListener(buf)
            # The listener needs a moment to accumulate the tree before any
            # lookup can succeed; querying immediately always fails.
            rospy.sleep(2.0)
        except Exception as ex:
            self.bad(f"Could not start a transform listener ({ex})")
            return

        base = self.expected_frames[0]
        for frame in self.expected_frames[1:]:
            try:
                buf.lookup_transform(base, frame, rospy.Time(0),
                                     rospy.Duration(2.0))
                self.ok(f"transform {base} -> {frame}")
            except Exception as ex:
                self.bad(f"No transform {base} -> {frame}",
                         f"Anything using {frame} will be referenced to the "
                         f"wrong place, silently. ({type(ex).__name__})")

    # ------------------------------------------------------------------
    def run(self):
        print("=" * 68)
        print(" PREFLIGHT CHECK — sidewalk VSLAM robot")
        print("=" * 68)

        self.check_disk()
        self.check_clock()
        self.check_ros_env()
        self.check_topics()
        self.check_transforms()

        print()
        print("=" * 68)
        print(f" {len(self.passed)} passed   {len(self.warned)} warnings   "
              f"{len(self.failed)} failures")
        print("=" * 68)

        if self.failed:
            print("\n BLOCKING ISSUES:")
            for msg, fix in self.failed:
                print(f"   • {msg}")
                if fix:
                    print(f"     → {fix}")

        if self.warned:
            print("\n WARNINGS:")
            for msg, fix in self.warned:
                print(f"   • {msg}")
                if fix:
                    print(f"     → {fix}")

        blocking = bool(self.failed) or (self.strict and bool(self.warned))
        if blocking:
            print("\n NOT CLEAR TO PROCEED.")
            print(" Data recorded now may be unusable. Fix the above first.\n")
            self.log.summary(
                f"Preflight FAILED — {len(self.failed)} blocking, "
                f"{len(self.warned)} warnings", status="FAIL")
            return 1

        print("\n CLEAR TO PROCEED.\n")
        self.log.summary(
            f"Preflight passed — {len(self.passed)} checks, "
            f"{len(self.warned)} warnings", status="OK")
        return 0


if __name__ == "__main__":
    try:
        sys.exit(Preflight().run())
    except rospy.ROSInterruptException:
        sys.exit(130)
    except Exception as ex:
        # A preflight check that crashes is worse than useless — it tells you
        # nothing about the robot. Report clearly and exit non-zero.
        print(f"\nPreflight could not complete: {type(ex).__name__}: {ex}")
        print("This usually means roscore is not running. Start it with: roscore")
        sys.exit(2)
