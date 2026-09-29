#!/usr/bin/env python3
"""replay_watch.py - progress line and clock-drift log for a running replay.

WHY
    Every long job here is watchable from a browser: it writes one line to
    $JOBS_DIR/<run>_replay.progress at least every 30 s (the jobs page draws a bar from N/M and
    the seconds elapsed). This node also records the evidence for the one question a replay must
    answer about itself: do the replayed pictures stay in step with the wheel data?

HOW THE DRIFT IS MEASURED
    Two independent witnesses, both logged to <out_dir>/clock_drift.csv:
    1. /diagnostics: the ZED wrapper reports "Playing SVO  Frame: k/N" about once a second.
       With the SVO index (replay_svo_index.py) frame k has a known capture time t_k, so
       offset = (bag clock now) - t_k.
    2. The camera IMU topic: in SVO playback the wrapper's sensor thread stamps IMU messages
       with the ORIGINAL recorded time while its picture loop stamps them with the bag clock
       (zed_wrapper_nodelet.cpp:3462-3489 and :3134-3137). For each IMU message,
       delta = (bag clock now) - (message stamp): near zero for the picture-loop messages,
       equal to the true offset for the original-time messages. replay_summarize.py separates
       the two populations.
    Positive offset = the camera stream runs BEHIND the bag (the picture shown "now" was taken
    earlier). The summary reports the start value, the end value and the change (the drift).

HOW TO RUN (run/replay.sh does this)
    replay_watch.py --run <run> --out-dir <dir> --jobs-dir <dir> --svo-index <csv> --total-s <s>

OUTPUT  $JOBS_DIR/<run>_replay.progress   "REPLAY <run> <played>/<total> s  <elapsed>s"
        <out_dir>/clock_drift.csv         kind,sim_time,value   (kind = svo_frame | imu_delta)
        <out_dir>/svo_end.json            written once the camera recording has been played to its end

HOW THE END OF THE RECORDING IS DETECTED
    The wrapper does not stop at the end of the file on this build: the exit for
    END_OF_SVOFILE_REACHED (zed_wrapper_nodelet.cpp:4015-4020) sits behind a test that sends every
    error other than CAMERA_REBOOTING back to the top of the loop (:4010-4014), so the node spins
    there printing warnings. This node therefore declares the end itself, when the wrapper's frame
    counter reports the last frame, or when the bag clock has passed the recording's last capture
    time by a small margin - whichever comes first. run/replay.sh watches for the file.
"""
import argparse
import csv
import json
import os
import re
import sys
import time

import rospy
from diagnostic_msgs.msg import DiagnosticArray
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu

FRAME_RE = re.compile(r"Frame:\s*(\d+)/(\d+)")


class Watch(object):
    def __init__(self, args):
        self.args = args
        self.t_start_wall = time.time()
        self.sim_now = None
        self.sim_first = None
        self.last_progress_wall = 0.0
        self.csv_file = open(os.path.join(args.out_dir, "clock_drift.csv"), "w")
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow(["kind", "sim_time", "value"])
        self.imu_count = 0
        self.end_written = False
        rospy.Subscriber("/clock", Clock, self.on_clock, queue_size=1)
        rospy.Subscriber("/diagnostics", DiagnosticArray, self.on_diagnostics, queue_size=5)
        rospy.Subscriber(args.imu_topic, Imu, self.on_imu, queue_size=200)

    def on_clock(self, msg):
        self.sim_now = msg.clock.to_sec()
        if self.sim_first is None:
            self.sim_first = self.sim_now
        if time.time() - self.last_progress_wall >= self.args.every:
            self.write_progress()
        if self.sim_now > self.args.svo_start + self.args.total_s + self.args.end_margin:
            self.mark_end("bag clock passed the recording's last frame time")

    def mark_end(self, why):
        if self.end_written:
            return
        self.end_written = True
        with open(os.path.join(self.args.out_dir, "svo_end.json"), "w") as f:
            json.dump({"sim_time": self.sim_now, "wall_time": time.time(), "why": why}, f, indent=1)
        rospy.loginfo("replay_watch: camera recording ended - %s", why)

    def on_diagnostics(self, msg):
        # only the wrapper's "Playing SVO" entry carries the frame index
        for status in msg.status:
            for kv in status.values:
                if kv.key == "Playing SVO":
                    m = FRAME_RE.search(kv.value)
                    if m and self.sim_now is not None:
                        self.csv.writerow(["svo_frame", "%.6f" % msg.header.stamp.to_sec(), m.group(1)])
                        if int(m.group(1)) >= int(m.group(2)):
                            self.mark_end("frame counter reached %s/%s" % (m.group(1), m.group(2)))

    def on_imu(self, msg):
        if self.sim_now is None:
            return
        self.imu_count += 1
        if self.imu_count % self.args.imu_every == 0:
            delta = self.sim_now - msg.header.stamp.to_sec()
            self.csv.writerow(["imu_delta", "%.6f" % self.sim_now, "%.6f" % delta])

    def write_progress(self):
        self.last_progress_wall = time.time()
        played = 0.0
        if self.sim_now is not None and self.args.svo_start > 0:
            played = max(0.0, self.sim_now - self.args.svo_start)
        elapsed = int(time.time() - self.t_start_wall)
        line = "REPLAY %s %d/%d s  %ds" % (self.args.run, int(played), int(self.args.total_s), elapsed)
        path = os.path.join(self.args.jobs_dir, "%s_replay.progress" % self.args.run)
        tmp = path + ".tmp"
        with open(tmp, "w") as f:
            f.write(line + "\n")
        os.rename(tmp, path)   # one atomic write: the page never reads a half line
        self.csv_file.flush()

    def close(self):
        self.csv_file.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--run", required=True)
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--jobs-dir", required=True)
    parser.add_argument("--svo-start", type=float, required=True, help="capture time of the first SVO frame, seconds")
    parser.add_argument("--total-s", type=float, required=True, help="length of the camera recording, seconds")
    parser.add_argument("--imu-topic", default="/zedx_front/zed_node/imu/data")
    parser.add_argument("--every", type=float, default=10.0, help="progress line period, wall seconds")
    parser.add_argument("--imu-every", type=int, default=5, help="log every Nth IMU message")
    parser.add_argument("--end-margin", type=float, default=3.0,
                        help="seconds past the recording's last frame time before the end is declared")
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("replay_watch")
    watch = Watch(args)
    watch.write_progress()
    # wall-clock loop: rospy.spin() would be fine too, but a wall timer keeps the progress line
    # moving even while the bag is paused and sim time stands still
    while not rospy.is_shutdown():
        time.sleep(1.0)
        if time.time() - watch.last_progress_wall >= args.every:
            watch.write_progress()
    watch.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
