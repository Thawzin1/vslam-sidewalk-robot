#!/usr/bin/env python3
"""
run_monitor.py — sample SLAM health once a second while a mapping run is driven.

WHY THIS EXISTS
    A mapping drive costs 20-45 minutes of somebody's time and cannot be
    replayed. Without a record, "the mapping drifted" is an observation nobody
    can act on: drifted by how much, starting when, and was it odometry losing
    tracking or a bad loop closure being accepted? Those have opposite fixes.

    With a record it becomes a diagnosis. This is how the 2026-08-21 run was
    understood: loop closure proposed a 1.0 m correction and RTAB-Map rejected
    it, and the CSV independently showed the robot really was 1.149 m off - so
    the closure was RIGHT and the rejection test was wrong. That conclusion is
    not reachable by watching RViz.

WHAT IT RECORDS, once a second
    pos_err / yaw_err   SLAM estimate (TF map->base_link) against Gazebo ground
                        truth. This is the number that matters.
    quality             RTAB-Map's inlier count. Under ~20 odometry cannot
                        compute a pose at all.
    resets              Odometry re-anchor events. Each one is a deliberate
                        discontinuity in the pose estimate.
    accepted/rejected   Loop closures. The ratio between them is the single
                        most diagnostic pair of numbers in the file.

    Two outputs: a CSV of the whole run for afterwards, and a one-line STATUS
    file holding the latest sample, so progress can be polled cheaply without
    re-reading the CSV.

THE MAP-FRAME OFFSET, AND WHY THIS ALIGNS INSTEAD OF DEMANDING AN ORIGIN RESET
    RTAB-Map anchors its map frame wherever the robot happens to be when
    mapping starts, so `map` is rigidly offset from `world` by the robot's
    starting pose. Left uncorrected, every pos_err below is inflated by that
    offset: one run opened reading 2.653 m of "drift" before the robot had
    moved, and an Mcity run opened reading 114.975 m.

    The obvious fix - always start at the world origin - does not generalise.
    In Mcity the origin measures 6 ORB keypoints against the 20 RTAB-Map
    needs, so starting there guarantees failure; the viable spawn is 115 m
    away. Demanding an origin reset would rule out most of the map.

    So instead this captures the SE(2) transform between the two frames on the
    first good sample and applies it to every subsequent estimate. Error is
    then measured relative to where the run actually began, which is what
    "drift" means anyway, and the spawn point is free to be wherever the
    features are.

    This is alignment-at-start, not the Umeyama fit used for a publishable
    ATE. It is the right choice for a live readout - it needs no future
    samples, so it can report from the first second - but a formal ATE should
    still be computed afterwards from the recorded TUM trajectories.

USAGE
    Normally started for you by record_mapping_run.launch. Standalone:
        rosrun sidewalk_slam run_monitor.py _log_file:=/path/to/rtabmap.log \\
                                            _out_file:=/path/to/monitor.csv
"""
import math
import os
import re

import rospy
import tf2_ros
from nav_msgs.msg import Odometry

# rtabmap_msgs is only present when RTAB-Map's ROS packages are installed.
# Import defensively so this node still records pose error on a machine
# without them, reporting the closure columns as -1 (unknown) rather than
# dying or, worse, printing a confident 0.
try:
    from rtabmap_msgs.msg import Info
except ImportError:  # pragma: no cover - depends on the host's ROS install
    try:
        from rtabmap_ros.msg import Info      # pre-0.21 package name
    except ImportError:
        Info = None

# ACCEPTED loop closures CANNOT be counted from the log. Do not "fix" the regex.
#
#   accepted closures: this ACCEPT_RE matched 0. So did every candidate
#   replacement ("Loop closure detected", "Global loop closure", "Accepted
#   loop", "Add local loop closure in SPACE") - 0 matches across all 62,832
#   lines. RTAB-Map's core logger does not write acceptances into this file at
#   any level, while rejections are WARN and always land. A log that can only
#   show one direction was being read as proof of the other, and the monitor
#   confidently printed ACCEPTED_LC=0 for an entire run that had 632.
#   Same class of bug as docs/SOLVED.md's "0/1144 loop closures accepted".
#
# Acceptances now come from RTAB-Map's Info topic instead (see _info_cb).
# REJECT_RE below is genuine - those lines really are in the log.
REJECT_RE = re.compile(r"Rejecting all added loop closures")
RESET_RE = re.compile(r"Odometry automatically reset")
QUAL_RE = re.compile(r"Odom: quality=(\d+)")

HEADER = ("t,gt_x,gt_y,est_x_aligned,est_y_aligned,pos_err,yaw_err,"
          "quality,resets,accepted_lc_global,accepted_lc_proximity,"
          "accepted_lc,rejected_lc\n")


def yaw_of(q):
    """Yaw in degrees from a quaternion."""
    return math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


class RunMonitor:
    def __init__(self):
        self.log_file = rospy.get_param("~log_file", "")
        self.out_file = rospy.get_param("~out_file",
                                        "/tmp/mapping_monitor.csv")
        self.status_file = self.out_file.replace(".csv", "_STATUS.txt")
        self.rate_hz = rospy.get_param("~rate", 1.0)

        self.gt = None
        # SE(2) transform from the SLAM map frame to the world frame, captured
        # on the first good sample. None until then.
        self.align = None
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf)
        rospy.Subscriber("/ground_truth/odom", Odometry, self._gt_cb,
                         queue_size=1)

        # Accepted loop closures are counted from RTAB-Map's own Info topic,
        # NOT from its log. See the comment on ACCEPT_RE above for why the log
        # can never work. loopClosureId is the global recognition ("I have been
        # here before"); proximityDetectionId is the local-space one ("this is
        # the same place as a nearby node"). Both are non-zero exactly when a
        # closure was accepted on that frame, which is what rtabmap-info later
        # reports as GlobalClosure / LocalSpaceClosure.
        self.acc_global = 0
        self.acc_prox = 0
        self.info_seen = False
        if Info is not None:
            rospy.Subscriber("/rtabmap/info", Info, self._info_cb, queue_size=50)
        else:
            rospy.logwarn("run_monitor: rtabmap_msgs/Info unavailable; the "
                          "accepted-loop-closure columns will read -1 "
                          "(unknown) rather than a false 0.")

        out_dir = os.path.dirname(self.out_file)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
        with open(self.out_file, "w") as fh:
            fh.write(HEADER)

        rospy.loginfo("run_monitor: %s  <- log %s", self.out_file,
                      self.log_file or "(none)")

    def _gt_cb(self, msg):
        self.gt = msg.pose.pose

    def _info_cb(self, msg):
        """Count accepted loop closures from RTAB-Map's own Info topic."""
        self.info_seen = True
        if msg.loopClosureId != 0:
            self.acc_global += 1
        if msg.proximityDetectionId != 0:
            self.acc_prox += 1

    def _accepted(self):
        """(global, proximity) accepted closures, or (-1, -1) if unmeasurable.

        -1 means "not known", never 0. A tool that prints 0 when the truth is
        632 is worse than one that admits it cannot tell.
        """
        if Info is None or not self.info_seen:
            return -1, -1
        return self.acc_global, self.acc_prox

    def _scan_log(self):
        """Count events in the RTAB-Map log. Cheap enough at 1 Hz.

        Rejections and resets only - acceptances are not in the log at all.
        """
        if not self.log_file:
            return 0, 0, -1
        try:
            with open(self.log_file, "rb") as fh:
                data = fh.read().decode("utf-8", "ignore")
        except OSError:
            return 0, 0, -1
        quals = QUAL_RE.findall(data)
        return (len(REJECT_RE.findall(data)),
                len(RESET_RE.findall(data)),
                int(quals[-1]) if quals else -1)

    def spin(self):
        t0 = rospy.Time.now().to_sec()
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            rej, res, qual = self._scan_log()
            acc_g, acc_p = self._accepted()
            acc = -1 if acc_g < 0 else acc_g + acc_p
            t = rospy.Time.now().to_sec() - t0

            # The SLAM estimate comes from TF, which RTAB-Map publishes on real
            # hardware exactly as it does in sim. Ground truth does NOT exist
            # on real hardware - there is no Gazebo to ask.
            try:
                tr = self.buf.lookup_transform(
                    "map", "base_link", rospy.Time(0),
                    rospy.Duration(0.5)).transform
                est = (tr.translation.x, tr.translation.y,
                       math.radians(yaw_of(tr.rotation)))
            except Exception:
                est = None

            # Blank, not zero. A zero here would be a confident lie about a
            # quantity that was never measured.
            gt_x = gt_y = est_x = est_y = pos_err = yaw_err = ""

            if est is not None and self.gt is not None:
                ex, ey, eyaw = est
                gx, gy, gyaw = (self.gt.position.x, self.gt.position.y,
                                math.radians(yaw_of(self.gt.orientation)))

                # Capture map->world alignment once, from the first sample.
                if self.align is None:
                    dth = gyaw - eyaw
                    c, s = math.cos(dth), math.sin(dth)
                    self.align = (dth, gx - (c * ex - s * ey),
                                       gy - (s * ex + c * ey))
                    rospy.loginfo(
                        "run_monitor: map->world alignment captured "
                        "(dx %.2f dy %.2f dyaw %.1f deg). Errors below are "
                        "relative to the run start.",
                        self.align[1], self.align[2], math.degrees(dth))

                dth, ox, oy = self.align
                c, s = math.cos(dth), math.sin(dth)
                ax, ay = c * ex - s * ey + ox, s * ex + c * ey + oy
                dx, dy = ax - gx, ay - gy
                dyaw = (math.degrees(eyaw + dth - gyaw) + 180.0) % 360.0 - 180.0
                gt_x, gt_y = "%.3f" % gx, "%.3f" % gy
                est_x, est_y = "%.3f" % ax, "%.3f" % ay
                pos_err = "%.3f" % math.hypot(dx, dy)
                yaw_err = "%.2f" % dyaw
            elif est is not None:
                # 2). Previously both writes below sat inside `if self.gt is not
                # None`, so a real run - which has no ground truth by
                # definition - produced a header-only CSV and no STATUS file at
                # all. Run 1 was monitored this way and recorded nothing; its
                # 249 closures had to be recovered from the database afterwards.
                # There is still plenty worth recording without ground truth:
                # the estimated path itself, odometry quality, resets, and the
                # loop-closure counts, which in a lab with no truth reference
                # ARE the measurement.
                est_x, est_y = "%.3f" % est[0], "%.3f" % est[1]

            with open(self.out_file, "a") as fh:
                fh.write("%.1f,%s,%s,%s,%s,%s,%s,%d,%d,%d,%d,%d,%d\n"
                         % (t, gt_x, gt_y, est_x, est_y, pos_err, yaw_err,
                            qual, res, acc_g, acc_p, acc, rej))

            # "?" rather than a number when a quantity is unmeasurable, so a
            # reader is never handed a confident zero that is wrong.
            acc_txt = "?" if acc < 0 else "%d (%d global + %d prox)" % (
                acc, acc_g, acc_p)
            err_txt = ("pos_err=%s m  yaw_err=%s deg" % (pos_err, yaw_err)
                       if pos_err else "pos_err=n/a (no ground truth)")
            pose_txt = ("est=(%s, %s)" % (est_x, est_y) if est_x
                        else "est=(waiting for map->base_link TF)")
            with open(self.status_file, "w") as fh:
                fh.write("t=%.0fs  %s  %s  quality=%d  resets=%d  "
                         "ACCEPTED_LC=%s  rejected_LC=%d\n"
                         % (t, pose_txt, err_txt, qual, res, acc_txt, rej))
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("run_monitor")
    RunMonitor().spin()
