#!/usr/bin/env python3
"""
ground_truth_odom.py — republish Gazebo's true robot pose as nav_msgs/Odometry.

WHY THIS EXISTS
    In simulation we know exactly where the robot is. Gazebo computes it: that is
    what the physics engine IS. That makes the simulator the one place in this
    whole project where a SLAM estimate can be checked against truth rather than
    against another estimate.

    Two things need that:

      * DEBUGGING. "The map looks wrong" is not a measurement. "The estimate is
        1.8 m and 24 degrees away from truth, starting at t=412 s" tells you
        when it broke and by how much, which is the difference between guessing
        at a cause and finding one.
      * THE BENCHMARK. Absolute Trajectory Error is the standard SLAM accuracy
        metric, and it is defined against ground truth. On the real sidewalk
        there is none (short of RTK), which is exactly why the comparison work
        is worth doing in sim first.

    Gazebo already publishes the truth on /gazebo/model_states, but as a
    ModelStates message: an unordered pile of every model in the world, with no
    header and no timestamp. Nothing downstream consumes that. This node picks
    out the one model we care about, stamps it, and publishes it as ordinary
    Odometry - so the existing trajectory_recorder.py can record it exactly the
    same way it records a real SLAM stack's output, and the two files are then
    directly comparable.

    NOTE: rtabmap_util ships a gazebo_ground_truth.py that looks similar. It is
    Python 2, publishes to TF rather than a topic, and refers to an undefined
    `roslog` in its own error path. Hence this one.

USAGE
    rosrun sidewalk_sim ground_truth_odom.py
    rosrun sidewalk_sim ground_truth_odom.py _model:=husky _topic:=/ground_truth/odom

    Record it alongside the SLAM estimate:
        rosrun sidewalk_slam trajectory_recorder.py --stack ground_truth \\
            --run-id <id> --source topic --topic /ground_truth/odom --type odom
"""
import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry


class GroundTruthOdom:
    def __init__(self):
        self.model = rospy.get_param("~model", "husky")
        topic = rospy.get_param("~topic", "/ground_truth/odom")
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.child_frame_id = rospy.get_param("~child_frame_id", "base_link")

        self.pub = rospy.Publisher(topic, Odometry, queue_size=10)
        self.warned = False
        self.count = 0
        self.dropped = 0
        self.last_stamp = rospy.Time(0)

        rospy.Subscriber("/gazebo/model_states", ModelStates,
                         self.callback, queue_size=1)
        rospy.loginfo("ground_truth_odom: model '%s' -> %s (%s -> %s)",
                      self.model, topic, self.frame_id, self.child_frame_id)

    def callback(self, msg):
        try:
            i = msg.name.index(self.model)
        except ValueError:
            # Warn once, not once per message at 1 kHz.
            if not self.warned:
                rospy.logwarn(
                    "ground_truth_odom: model '%s' is not in /gazebo/model_states. "
                    "Present: %s", self.model, ", ".join(msg.name))
                self.warned = True
            return

        # ModelStates arrives at ~1 kHz but the sim clock ticks at ~100 Hz, so
        # many consecutive callbacks see the SAME rospy.Time.now(). Publishing
        # them all produced a reference trajectory with ~25k duplicate stamps
        # (flagged by evaluate_trajectory as "non-increasing timestamps").
        # One pose per distinct clock tick is the honest rate; ~100 Hz remains
        # far above the >=50 Hz the evaluation needs.
        now = rospy.Time.now()
        if now <= self.last_stamp:
            self.dropped += 1
            return
        self.last_stamp = now

        odom = Odometry()
        # ModelStates carries no header, so stamp it here. Under use_sim_time
        # this is Gazebo's clock, which is what makes it line up with the SLAM
        # estimate when the two trajectories are compared afterwards.
        odom.header.stamp = now
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose = msg.pose[i]
        odom.twist.twist = msg.twist[i]
        self.pub.publish(odom)

        self.count += 1
        if self.count == 1:
            p = odom.pose.pose.position
            rospy.loginfo("ground_truth_odom: publishing (first pose %.3f %.3f %.3f)",
                          p.x, p.y, p.z)


if __name__ == "__main__":
    rospy.init_node("ground_truth_odom")
    GroundTruthOdom()
    rospy.spin()
