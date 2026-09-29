#!/usr/bin/env python3
"""replay_starter.py - bring the paused bag into step with the camera recording, then let both run.

WHY
    In a replay the camera recording and the robot's bag are two separate players. The bag starts
    paused. The camera takes 10-30 s to open its file (loading the depth model); the ZED SDK starts
    its real-time playback clock at the first frame it reads, and the first picture is published a
    variable 0.3-1 s later (the first NEURAL depth computation). So "resume the bag on the first
    picture" alone leaves a start offset that changes from run to run (measured -0.26 s and -0.71 s
    on two runs of the same recording). This node closes that loop:

    1. run/replay.sh places the paused bag a safety margin AHEAD of the recording's first frame time
       (--bag-position = first frame time + margin, default margin 3 s).
    2. On the first camera picture, resume the bag. The bag is now ahead of the camera by
       (margin - camera start-up latency), a positive, unknown amount.
    3. Measure it: in playback the wrapper's sensor thread stamps camera IMU messages with the
       ORIGINAL recorded time (zed_wrapper_nodelet.cpp:3462-3489), so for those messages
       (bag clock now) - (message stamp) is the offset. Take the median over a couple of seconds.
    4. Pause the bag for exactly that long (the SDK's playback clock keeps running meanwhile, so
       the camera catches up), resume, measure once more and write the result to the marker file.

    What is left afterwards is the reaction time of two service calls (tens of ms) and the picture
    processing time inside the wrapper (about 0.1 s); replay_watch.py logs both witnesses for the
    whole run and replay_summarize.py reports them.

HOW TO RUN (run/replay.sh does this)
    replay_starter.py --bag-position <seconds> --marker <out_dir>/unpaused.json
                      [--service /replay_bag/pause_playback] [--imu-topic ...] [--measure-s 2.0]

OUTPUT  the marker file: {"resumed_sim_time", "offset_before_s", "pause_applied_s",
        "offset_after_s", ...}. Exit code 0 = done, 1 = failed.
"""
import argparse
import json
import sys
import time

import rospy
from sensor_msgs.msg import CameraInfo, Imu
from std_srvs.srv import SetBool


def median(values):
    s = sorted(values)
    return s[len(s) // 2]


class Starter(object):
    def __init__(self, args):
        self.args = args
        self.first_picture = None      # (wall time, sim time, stamp) of the first camera picture
        self.offsets = []              # (bag clock now) - (original stamp), one per IMU message
        self.collecting = False
        rospy.wait_for_service(args.service, timeout=120)
        self.set_pause = rospy.ServiceProxy(args.service, SetBool)
        rospy.Subscriber(args.camera_info_topic, CameraInfo, self.on_picture, queue_size=1)
        rospy.Subscriber(args.imu_topic, Imu, self.on_imu, queue_size=200)
        rospy.loginfo("replay_starter: bag paused at %.3f; waiting for the first picture on %s",
                      args.bag_position, args.camera_info_topic)

    def on_picture(self, msg):
        if self.first_picture is None:
            self.first_picture = (time.time(), rospy.Time.now().to_sec(), msg.header.stamp.to_sec())

    def on_imu(self, msg):
        if not self.collecting:
            return
        delta = rospy.Time.now().to_sec() - msg.header.stamp.to_sec()
        # messages stamped with the bag clock itself sit within a few ms of zero; the others carry
        # the original recorded time and measure the offset
        if abs(delta) > 0.02:
            self.offsets.append(delta)

    def measure(self, seconds):
        """Median offset over a wall-time window; empty list if no original-stamped message came."""
        self.offsets = []
        self.collecting = True
        time.sleep(seconds)
        self.collecting = False
        return list(self.offsets)

    def run(self):
        t0 = time.time()
        while self.first_picture is None:
            if rospy.is_shutdown() or time.time() - t0 > self.args.timeout:
                rospy.logerr("replay_starter: no picture within %.0f s - giving up", self.args.timeout)
                return 1
            time.sleep(0.02)
        reply = self.set_pause(False)                       # data False = resume
        resumed = {"wall_time": time.time(), "sim_time": rospy.Time.now().to_sec(),
                   "first_picture_stamp": self.first_picture[2], "service_reply": reply.message}
        rospy.loginfo("replay_starter: bag resumed at sim time %.3f", resumed["sim_time"])

        before = self.measure(self.args.measure_s)
        result = dict(resumed_sim_time=resumed["sim_time"], first_picture_stamp=resumed["first_picture_stamp"],
                      offset_before_s=None, pause_applied_s=0.0, offset_after_s=None, samples_before=len(before))
        if before:
            offset = median(before)
            result["offset_before_s"] = round(offset, 3)
            if offset > self.args.min_correction_s:
                # the bag is ahead: hold it while the camera's playback clock runs on
                self.set_pause(True)
                time.sleep(max(0.0, offset - self.args.service_latency_s))
                self.set_pause(False)
                result["pause_applied_s"] = round(offset, 3)
                after = self.measure(self.args.measure_s)
                result["samples_after"] = len(after)
                if after:
                    result["offset_after_s"] = round(median(after), 3)
            elif offset < -self.args.min_correction_s:
                rospy.logwarn("replay_starter: the camera is AHEAD of the bag by %.3f s - the sync margin "
                              "(--bag-position) was too small; nothing can be corrected, the offset stays", -offset)
                result["offset_after_s"] = round(offset, 3)
            else:
                result["offset_after_s"] = round(offset, 3)
        else:
            rospy.logwarn("replay_starter: no original-stamped camera IMU message in %.1f s - offset not measured",
                          self.args.measure_s)
        with open(self.args.marker, "w") as f:
            json.dump(result, f, indent=1)
        rospy.loginfo("replay_starter: offset before %s s, paused %.3f s, after %s s",
                      result["offset_before_s"], result["pause_applied_s"], result["offset_after_s"])
        return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--service", default="/replay_bag/pause_playback")
    parser.add_argument("--camera-info-topic", default="/zedx_front/zed_node/rgb/camera_info")
    parser.add_argument("--imu-topic", default="/zedx_front/zed_node/imu/data")
    parser.add_argument("--bag-position", type=float, required=True, help="the sim time the bag is paused at")
    parser.add_argument("--marker", required=True, help="file written when the sync is done")
    parser.add_argument("--measure-s", type=float, default=2.0, help="how long to measure the offset (wall s)")
    parser.add_argument("--min-correction-s", type=float, default=0.05, help="offsets below this are left alone")
    parser.add_argument("--service-latency-s", type=float, default=0.02,
                        help="time a pause/resume call takes, subtracted from the hold")
    parser.add_argument("--timeout", type=float, default=600.0, help="give up after this many wall seconds")
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("replay_starter")
    return Starter(args).run()


if __name__ == "__main__":
    sys.exit(main())
