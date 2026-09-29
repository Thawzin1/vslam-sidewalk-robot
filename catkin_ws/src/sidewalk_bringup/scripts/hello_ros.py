#!/usr/bin/env python3
"""
hello_ros.py — The smallest complete demonstration of how ROS works.

READ THIS FILE. It is meant to be read more than it is meant to be run.

────────────────────────────────────────────────────────────────────────────
THE ONE IDEA YOU NEED
────────────────────────────────────────────────────────────────────────────

A robot is not one program. It is many small programs running at the same
time, each doing one job, passing messages to each other.

    the camera program  ──picture──▶  the mapping program  ──map──▶  the
    driving program

In ROS these programs are called **nodes**. The named channels they send
messages over are called **topics**.

The crucial property — and the thing that makes robotics software tolerable —
is that a node does not know who it is talking to. The camera node shouts
"here is a picture" onto a channel called `/image`. It has no idea whether
anyone is listening, or whether it is one program or five. You can attach a
recorder, or a display, or a mapping algorithm, without changing the camera
node at all.

An analogy that holds up well: it is a radio broadcast, not a phone call.
The station transmits on a frequency. Zero listeners or a million, the
station behaves identically. You tune in to receive.

────────────────────────────────────────────────────────────────────────────
THE THREE THINGS THIS FILE DEMONSTRATES
────────────────────────────────────────────────────────────────────────────

  1. A PUBLISHER  — a node that broadcasts messages onto a topic.
  2. A SUBSCRIBER — a node that listens to a topic and reacts.
  3. A SERVICE    — a direct question-and-answer, when broadcasting is wrong.

Publish/subscribe is for continuous streams: images, positions, wheel speeds.
Nobody waits for a reply; data keeps flowing.

A service is for one-off requests where you need an answer: "reset the map",
"what is your battery level?". The caller waits. That waiting is exactly why
services are wrong for sensor data — a slow reply would stall the robot.

────────────────────────────────────────────────────────────────────────────
HOW TO RUN IT
────────────────────────────────────────────────────────────────────────────

Terminal 1 — start the ROS master. Every ROS 1 system needs exactly one.
It is the switchboard: nodes register with it and it introduces them to
each other. (Note: it only introduces them. Once two nodes are connected,
their data flows directly, not through the master.)

    roscore

Terminal 2 — run this file:

    rosrun sidewalk_bringup hello_ros.py

Terminal 3 — inspect the running system from outside. This is the part
worth internalising: you can observe a live robot without modifying it.

    rostopic list                      # what channels exist right now
    rostopic echo /sidewalk/heartbeat  # watch the messages go past
    rostopic hz   /sidewalk/heartbeat  # measure the actual rate
    rosnode list                       # what programs are running
    rosnode info /hello_ros            # what this node publishes and subscribes
    rosservice call /sidewalk/reset_counter  # call the service

`rostopic hz` is not a toy. It is the tool you will use to prove the camera is
really delivering 30 frames per second rather than claiming to.
"""

import rospy

from std_msgs.msg import String, Int32
from std_srvs.srv import Trigger, TriggerResponse

# The shared project logger. Imported defensively so this file still runs if
# it is copied somewhere else on its own — a teaching file should never fail
# for a reason unrelated to what it is teaching.
try:
    try:
        # Canonical import: works from a source workspace and
        # after catkin_make install, because sidewalk_bringup
        # exports src/ via catkin_python_setup().
        from sidewalk_bringup.run_logger import RunLogger
    except ImportError:
        from run_logger import RunLogger
except ImportError:                                        # pragma: no cover
    class RunLogger:                                       # minimal stand-in
        def __init__(self, *a, **k): pass
        def info(self, m): rospy.loginfo(m)
        def warn(self, m): rospy.logwarn(m)
        def error(self, m): rospy.logerr(m)
        def metric(self, k, v, unit=""): rospy.loginfo(f"{k}={v}{unit}")
        def section(self, t): rospy.loginfo(f"--- {t} ---")
        def summary(self, t, status=""): rospy.loginfo(t)


class HelloRos:
    """One node that publishes, subscribes to itself, and offers a service.

    Publishing and subscribing to the same topic in one node is artificial —
    real systems split these across programs. It is done here so the whole
    round trip is visible in a single file you can read top to bottom.
    """

    def __init__(self):
        # ── Step 1: become a node ────────────────────────────────────────
        # Until this runs, this is just a Python script. After it, the ROS
        # master knows we exist and can introduce us to other nodes.
        rospy.init_node("hello_ros", anonymous=False)

        self.log = RunLogger("sidewalk_bringup", run_name="hello_ros")
        self.count = 0
        self.received = 0

        # ── Step 2: parameters ───────────────────────────────────────────
        # Values read from the parameter server rather than hardcoded, so
        # behaviour can change from a launch file without editing code.
        # Every tunable number in this project lives in a config file for
        # exactly this reason: an experiment you cannot reconfigure is an
        # experiment you cannot repeat with a different setting.
        self.rate_hz = rospy.get_param("~rate_hz", 1.0)
        self.message = rospy.get_param("~message", "sidewalk robot alive")

        # ── Step 3: a PUBLISHER ──────────────────────────────────────────
        # queue_size is the count of messages held if subscribers cannot keep
        # up. Too small drops data; too large adds latency by delivering stale
        # data. For a 1 Hz heartbeat it barely matters. For a 60 FPS camera it
        # matters enormously — an oversized queue means you eventually process
        # images describing where the robot WAS, not where it is.
        self.pub_beat = rospy.Publisher(
            "/sidewalk/heartbeat", String, queue_size=10)
        self.pub_count = rospy.Publisher(
            "/sidewalk/counter", Int32, queue_size=10)

        # ── Step 4: a SUBSCRIBER ─────────────────────────────────────────
        # The callback runs on a background thread every time a message
        # arrives. Nothing calls it explicitly. Keep callbacks fast: slow ones
        # back up the queue and you start processing history.
        self.sub = rospy.Subscriber(
            "/sidewalk/counter", Int32, self.on_counter)

        # ── Step 5: a SERVICE ────────────────────────────────────────────
        # Trigger is the simplest service type: no input, returns success
        # plus a message. Used here to reset the counter on demand.
        self.srv = rospy.Service(
            "/sidewalk/reset_counter", Trigger, self.on_reset)

        self.log.info(f"hello_ros started at {self.rate_hz} Hz")
        self.log.info("Publishing : /sidewalk/heartbeat, /sidewalk/counter")
        self.log.info("Subscribing: /sidewalk/counter")
        self.log.info("Service    : /sidewalk/reset_counter")

        rospy.on_shutdown(self.on_shutdown)

    # ────────────────────────────────────────────────────────────────────
    def on_counter(self, msg):
        """Runs automatically whenever a message lands on /sidewalk/counter."""
        self.received += 1
        # Log only occasionally. A callback that prints on every message at
        # camera frame rates will itself become the bottleneck — a real and
        # frequently-encountered mistake.
        if self.received % 5 == 0:
            self.log.info(f"received counter value {msg.data} "
                          f"({self.received} messages so far)")

    def on_reset(self, _request):
        """Handles a service call. The caller waits for what we return."""
        previous = self.count
        self.count = 0
        self.log.info(f"counter reset by service call (was {previous})")
        return TriggerResponse(
            success=True,
            message=f"Counter reset from {previous} to 0")

    # ────────────────────────────────────────────────────────────────────
    def spin(self):
        """The main loop.

        rospy.Rate does NOT simply sleep for 1/rate seconds. It measures how
        long the loop body took and sleeps for the remainder, holding the true
        average rate. Doing this by hand with time.sleep() makes the loop run
        slightly slow, and the error accumulates.
        """
        rate = rospy.Rate(self.rate_hz)

        while not rospy.is_shutdown():
            self.count += 1

            # rospy.Time.now() — always prefer ROS time over wall-clock time.
            # When replaying recorded data, ROS time follows the recording,
            # so the same code works live and in playback unchanged.
            stamp = rospy.Time.now().to_sec()

            self.pub_beat.publish(String(
                data=f"{self.message} | tick {self.count} | t={stamp:.2f}"))
            self.pub_count.publish(Int32(data=self.count))

            if self.count % 10 == 0:
                self.log.metric("ticks_published", self.count)

            rate.sleep()

    def on_shutdown(self):
        self.log.metric("total_published", self.count)
        self.log.metric("total_received", self.received)
        self.log.summary(
            f"hello_ros finished — published {self.count}, "
            f"received {self.received}",
            status="OK")


if __name__ == "__main__":
    try:
        HelloRos().spin()
    except rospy.ROSInterruptException:
        # Raised on Ctrl-C or shutdown. Expected, not an error.
        pass
