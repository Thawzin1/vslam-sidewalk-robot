#!/usr/bin/env python3
"""cloud_decimator.py — republish a point cloud small enough to look at.

WHY THIS EXISTS
    The ZED X publishes /point_cloud/cloud_registered at 1920x1200 = 2.3 million
    points per message. Measured on this rig 2026-08-29:

        rostopic bw  ->  307.76 MB/s,  36.86 MB per message

    That is fine on the Jetson's own loopback, and completely unusable through an
    X-forwarded RViz on another computer: the window stops repainting and the desktop
    shows "rviz is not responding". The bottleneck is the link and the client, not
    the camera.

    This node subscribes once on the Jetson, throws away all but every Nth point,
    and republishes. RViz then subscribes to a topic that is ~3 orders of
    magnitude smaller, and stays interactive.

WHY STRIDING AND NOT A VOXEL GRID
    A voxel filter (pcl_ros/VoxelGrid) is the "proper" answer and gives evenly
    spaced output. It is also a nodelet that must be loaded into a manager, adds a
    dependency, and costs CPU on a Jetson that measured load 7.47 while this was
    happening. Striding is O(1) per output point, needs nothing but numpy, and for
    LOOKING AT WHERE A SURFACE IS it is equally good: the floor plane is still a
    plane when you keep one point in four hundred.

    Striding is NOT appropriate if you need uniform spatial density (e.g. feeding
    a mapper). This node is a diagnostic aid. Do not put it in a SLAM pipeline.

THE POINT ORDER CAVEAT
    An organised cloud (height > 1) is stored row-major, so a fixed stride samples
    a regular lattice across the image — which is what you want. If the stride
    happens to share a factor with the row width you get vertical banding rather
    than a lattice; `--stride` defaults to a value coprime with 1920 to avoid it.

    usage:
      rosrun sidewalk_perception cloud_decimator.py \
          _in:=/zedx_front/zed_node/point_cloud/cloud_registered \
          _out:=/zedx_front/cloud_light _stride:=400 _rate:=2.0
"""
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2


class Decimator(object):
    def __init__(self):
        self.stride = max(1, int(rospy.get_param("~stride", 401)))
        self.rate = float(rospy.get_param("~rate", 2.0))
        topic_in = rospy.get_param("~in",
                                   "/zedx_front/zed_node/point_cloud/cloud_registered")
        topic_out = rospy.get_param("~out", "/zedx_front/cloud_light")

        self.min_period = 1.0 / self.rate if self.rate > 0 else 0.0
        self.last = 0.0
        self.reported = False

        self.pub = rospy.Publisher(topic_out, PointCloud2, queue_size=1)
        # queue_size 1 + buff_size large enough for one 37 MB message. Without the
        # big buff_size, rospy fragments the read and falls behind, which shows up
        # as latency that looks like the decimator being slow when it is not.
        self.sub = rospy.Subscriber(topic_in, PointCloud2, self.cb,
                                    queue_size=1, buff_size=64 * 1024 * 1024)

        rospy.loginfo("cloud_decimator: %s -> %s  (every %d-th point, max %.1f Hz)",
                      topic_in, topic_out, self.stride, self.rate)

    def cb(self, msg):
        now = rospy.get_time()
        if self.min_period and (now - self.last) < self.min_period:
            return
        self.last = now

        step = msg.point_step
        n_in = len(msg.data) // step
        if n_in == 0:
            return

        # Reinterpret the raw buffer as rows of `point_step` bytes and keep every
        # Nth row. No per-point Python loop, no field decoding — the bytes are
        # copied through untouched, so every field (including rgb) survives.
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        buf = buf[:n_in * step].reshape(n_in, step)
        kept = buf[::self.stride]

        out = PointCloud2()
        out.header = msg.header
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = step
        out.height = 1                      # striding destroys the 2D structure
        out.width = kept.shape[0]
        out.row_step = out.point_step * out.width
        out.is_dense = msg.is_dense
        out.data = kept.tobytes()
        self.pub.publish(out)

        if not self.reported:
            rospy.loginfo("cloud_decimator: %d -> %d points per message (%.1f%%)",
                          n_in, out.width, 100.0 * out.width / n_in)
            self.reported = True


if __name__ == "__main__":
    rospy.init_node("cloud_decimator")
    Decimator()
    rospy.spin()
