#!/usr/bin/env python3
"""map_to_image.py — publish RTAB-Map's occupancy grid as a plain Image topic,
so the live map can be watched in a browser instead of needing a desktop.

WHY THIS EXISTS
    Watching the map normally means RViz on the Jetson's screen, mirrored over
    VNC. That breaks the moment the Jetson runs with NO MONITOR attached: the
    graphics driver falls back to a 640x480 framebuffer with no active output
    and the desktop renders nothing at all, so VNC shows pure black. Resizing
    the framebuffer does not help - there is nothing drawing into it. Fixing it
    properly needs an Xorg restart, which would kill a running camera and
    mapping session mid-run.

    So instead of making the desktop work, this skips the desktop. It renders
    the grid itself and publishes it as an Image, which web_video_server (already
    running on :8080) serves straight to a browser. No desktop, no VNC, no X.

WHAT YOU SEE
    White  = free space the robot has confirmed
    Black  = obstacles
    Grey   = not yet seen
    Red    = the robot's current position and heading
    Green  = where the robot has been

USAGE
    rosrun sidewalk_slam map_to_image.py          (or: python3 map_to_image.py)
    then open  http://<jetson-address>:8080/stream?topic=/map_image
"""
import os
import math

import cv2
import numpy as np
import rospy
import tf2_ros
from cv_bridge import CvBridge
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image

OUT_TOPIC = "/map_image"
GRID_TOPIC = "/rtabmap/grid_map"
LONG_EDGE = 900          # px; upscale small grids so the browser view is legible


class MapImage:
    def __init__(self):
        self.bridge = CvBridge()
        self.grid = None
        self.trail = []          # world-frame (x, y) breadcrumbs
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf)
        self.pub = rospy.Publisher(OUT_TOPIC, Image, queue_size=1)
        rospy.Subscriber(GRID_TOPIC, OccupancyGrid, self._grid_cb, queue_size=1)
        self.publishes = 0

    def _grid_cb(self, msg):
        self.grid = msg

    def _robot_xy_yaw(self):
        try:
            t = self.buf.lookup_transform("map", "base_link", rospy.Time(0),
                                          rospy.Duration(0.2)).transform
        except Exception:
            return None
        q = t.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.translation.x, t.translation.y, yaw

    def render(self):
        g = self.grid
        if g is None or g.info.width == 0 or g.info.height == 0:
            # Nothing mapped yet - say so rather than publishing a blank frame,
            # so a black browser tab is never ambiguous.
            img = np.full((240, 640, 3), 40, np.uint8)
            cv2.putText(img, "waiting for /rtabmap/grid_map ...", (30, 130),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 200), 2)
            return img

        w, h, res = g.info.width, g.info.height, g.info.resolution
        ox, oy = g.info.origin.position.x, g.info.origin.position.y
        cells = np.asarray(g.data, dtype=np.int8).reshape(h, w)

        img = np.full((h, w, 3), 128, np.uint8)          # unknown -> grey
        img[cells == 0] = (255, 255, 255)                # free     -> white
        img[cells > 0] = (0, 0, 0)                       # occupied -> black

        def to_px(x, y):
            return int((x - ox) / res), int((y - oy) / res)

        pose = self._robot_xy_yaw()
        if pose is not None:
            rx, ry, ryaw = pose
            if not self.trail or math.hypot(rx - self.trail[-1][0],
                                            ry - self.trail[-1][1]) > 0.05:
                self.trail.append((rx, ry))
                if len(self.trail) > 20000:
                    self.trail = self.trail[-20000:]

        for i in range(1, len(self.trail)):
            p0, p1 = to_px(*self.trail[i - 1]), to_px(*self.trail[i])
            cv2.line(img, p0, p1, (0, 180, 0), 1)

        if pose is not None:
            px, py = to_px(rx, ry)
            cv2.circle(img, (px, py), 4, (0, 0, 255), -1)
            tip = (int(px + 12 * math.cos(ryaw)), int(py + 12 * math.sin(ryaw)))
            cv2.line(img, (px, py), tip, (0, 0, 255), 2)

        img = cv2.flip(img, 0)         # grid origin is bottom-left; images are top-left

        scale = LONG_EDGE / float(max(w, h))
        if scale > 1.0:
            img = cv2.resize(img, None, fx=scale, fy=scale,
                             interpolation=cv2.INTER_NEAREST)

        bar = np.full((34, img.shape[1], 3), 30, np.uint8)
        cv2.putText(bar, "%.1f x %.1f m   %d cells   trail %d" %
                    (w * res, h * res, int((cells >= 0).sum()), len(self.trail)),
                    (10, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 220), 1)
        return np.vstack([bar, img])

    def spin(self):
        rate = rospy.Rate(2.0)
        while not rospy.is_shutdown():
            msg = self.bridge.cv2_to_imgmsg(self.render(), encoding="bgr8")
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "map"
            self.pub.publish(msg)
            self.publishes += 1
            if self.publishes % 60 == 0:
                rospy.loginfo("map_to_image: %d frames published, trail %d pts",
                              self.publishes, len(self.trail))
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("map_to_image")
    MapImage().spin()
