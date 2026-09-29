#!/usr/bin/env python3
"""robot_battery_reader.py - hands the robot's battery voltage to robot_storage_agent.py.

Runs ON THE ROBOT, started ONLY by robot_storage_agent.py (version 4) as its child
program, never by hand. Added 2026-09-26.

It LISTENS to the robot's own status message, /status (husky_msgs/HuskyStatus,
published once a second by the colleague's husky_node), and writes ONE line per
message to its standard output:  "<battery_voltage>\\n". Nothing else.

READ-ONLY, like `rostopic echo /status`:
  - one subscriber, queue of 1; it PUBLISHES NOTHING - not even the usual log topic
    (/rosout is switched off with disable_rosout=True);
  - an anonymous node name (slam_battery_reader_<number>), so it can never take the
    name of, and so shut down, any other node on the colleague's ROS;
  - its own ROS log goes to memory (/dev/shm), never onto the robot's 88 %-full disk.

*Plain terms: it reads the number the robot already announces every second, and
passes it on. It cannot change anything on the robot.*

Why a separate program and not a thread inside the agent: ROS's Python library keeps
global state and cannot reconnect by itself if the robot's ROS is restarted. As a child,
it can be stopped and started again cleanly by the agent whenever readings stop, and a
fault in it can never stop the storage and health reports.

It exits by itself when the agent goes away (its output pipe breaks, or its parent
process number changes).
"""
import os
import sys
import threading
import time

# ROS's settings come from `source /opt/ros/noetic/setup.bash`, done by the agent before starting
# this program (without them rospy cannot even set up its logging: ResourceNotFound rosgraph)
os.environ.setdefault("ROS_MASTER_URI", "http://localhost:11311")
os.environ["ROS_LOG_DIR"] = "/dev/shm/slam_battery_reader_ros_log"

PARENT = os.getppid()


def watch_parent():
    while True:
        time.sleep(2.0)
        if os.getppid() != PARENT:
            os._exit(0)


def main():
    threading.Thread(target=watch_parent, name="watch-parent", daemon=True).start()
    import rospy                                    # after the watcher: a slow import still ends with the agent
    from husky_msgs.msg import HuskyStatus

    def on_msg(m):
        try:
            sys.stdout.write("%.4f\n" % float(m.battery_voltage))
            sys.stdout.flush()
        except (BrokenPipeError, OSError, ValueError, TypeError):
            os._exit(0)                             # the agent has gone: stop

    # disable_rostime: never asks the master for /use_sim_time (so it never waits for it);
    # disable_signals: SIGTERM keeps its plain meaning (stop now)
    rospy.init_node("slam_battery_reader", anonymous=True, disable_signals=True,
                    disable_rosout=True, disable_rostime=True)
    rospy.Subscriber("/status", HuskyStatus, on_msg, queue_size=1)
    while True:
        time.sleep(3600)


if __name__ == "__main__":
    main()
