#!/usr/bin/env python3
"""roi_viz.py — live ROI box preview for the sphere-frame capture.

Draws the ROI prism in RViz and republishes ONLY the points inside it, so you can visually
confirm all three spheres are captured before collecting data. Parameters are re-read every
cycle, so you can nudge the box live with `rosparam set` and watch it move.

  rosrun self_navigation roi_viz.py                          # /helios/points
  rosrun self_navigation roi_viz.py _cloud_topic:=/ouster/points

RViz: Fixed Frame = rslidar (or os_sensor)
      add MarkerArray  -> /roi_viz/box        (green wireframe = ROI, red spheres = detection)
      add PointCloud2  -> /roi_viz/inside     (points inside the ROI)
      add PointCloud2  -> /helios/points      (full cloud, for context)

Tune live (metres):
  rosparam set /roi_viz/x 7.0          # frame distance   <-- the usual one
  rosparam set /roi_viz/y 0.0          # lateral offset
  rosparam set /roi_viz/z 1.45         # box centre height ABOVE THE FLOOR

Box size is fixed by the frame geometry (1.10 base + 0.40 sphere = 1.5 m wide/tall) but can
be changed too: rosparam set /roi_viz/sx 1.2  (and sy, sz).

When it looks right, print the launch line:  rosparam set /roi_viz/show_cmd true
"""
import sys
import numpy as np, rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from visualization_msgs.msg import Marker, MarkerArray
sys.path.insert(0, '/home/administrator/catkin_ws/src/matt_self_navigation/scripts')
from sphere_centroid import detect_frame

_DT = {1:'i1',2:'u1',3:'i2',4:'u2',5:'i4',6:'u4',7:'f4',8:'f8'}
# Measured sensor heights above the floor. Nicolas measured the two LiDARs; the
# camera's value is NOT a constant here because the mount has been changed - it
# is measured per session with camera_pose_from_floor.py and passed in as
# _sensor_h. See the note in __init__.
SENSOR_H = {'ouster': 0.79, 'helios': 0.97}

def decode(msg):
    names = [f.name for f in msg.fields]
    dt = np.dtype(dict(names=names, formats=[_DT[f.datatype] for f in msg.fields],
                       offsets=[f.offset for f in msg.fields], itemsize=msg.point_step))
    a = np.frombuffer(msg.data, dtype=dt)
    xyz = np.stack([a['x'], a['y'], a['z']], axis=1).astype(np.float64)
    return xyz[np.isfinite(xyz).all(axis=1)]

def box_marker(header, lo, hi, ns, mid, rgb, width=0.02):
    """Wireframe box as a LINE_LIST (12 edges)."""
    m = Marker(header=header, ns=ns, id=mid, type=Marker.LINE_LIST, action=Marker.ADD)
    m.scale.x = width
    m.color.r, m.color.g, m.color.b, m.color.a = rgb[0], rgb[1], rgb[2], 1.0
    m.pose.orientation.w = 1.0
    c = [(lo[0], lo[1], lo[2]), (hi[0], lo[1], lo[2]), (hi[0], hi[1], lo[2]), (lo[0], hi[1], lo[2]),
         (lo[0], lo[1], hi[2]), (hi[0], lo[1], hi[2]), (hi[0], hi[1], hi[2]), (lo[0], hi[1], hi[2])]
    edges = [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7)]
    from geometry_msgs.msg import Point
    for i, j in edges:
        m.points.append(Point(*c[i])); m.points.append(Point(*c[j]))
    return m

class RoiViz:
    def __init__(self):
        self.topic = rospy.get_param('~cloud_topic', '/helios/points')
        self.tag = 'ouster' if 'ouster' in self.topic else 'helios'
        # 'helios' for ANY topic it does not recognise, so a camera topic would
        # have silently used the Helios's 0.97 m. The camera sits near 0.65 m, so
        # the box would have been placed about 0.32 m too high and found nothing -
        # and it would have looked like a camera failure, not a setup error.
        # _sensor_h overrides the table; the two LiDARs are unaffected.
        self.sensor_h = float(rospy.get_param('~sensor_h', SENSOR_H[self.tag]))
        if not rospy.has_param('~sensor_h'):
            rospy.logwarn('roi_viz: no _sensor_h given - using the %s table value '
                          '%.2f m. For the camera you MUST pass the measured height.',
                          self.tag, self.sensor_h)
        # defaults sized from the physical frame
        for k, v in (('x', 4.35), ('y', 0.0), ('z', 1.45),
                     ('sx', 1.2), ('sy', 1.9), ('sz', 1.8),
                     ('min_inliers', 10), ('ransac_iters', 1500), ('side_tol', 0.06),
                     ('detect', True), ('show_cmd', False)):
            if not rospy.has_param('~' + k):
                rospy.set_param('~' + k, v)
        self.pub_m = rospy.Publisher('~box', MarkerArray, queue_size=2)
        self.pub_p = rospy.Publisher('~inside', PointCloud2, queue_size=2)
        self.floor = None
        self.last_report = 0.0
        rospy.Subscriber(self.topic, PointCloud2, self.cb, queue_size=1, buff_size=2**24)
        rospy.loginfo('roi_viz: %s (sensor %.2f m above floor). Tune with: rosparam set /roi_viz/x <m>',
                      self.topic, self.sensor_h)

    def cb(self, msg):
        now = rospy.get_time()
        if now - self.last_report < 0.5:      # throttle to ~2 Hz
            return
        xyz = decode(msg)
        if len(xyz) < 100:
            return
        # Floor = -(known sensor height). Do NOT infer it from the densest z bin: in a large
        # space the near-horizontal beams pile up on distant walls at sensor height and win
        # the histogram, which put the Ouster box ~0.8 m too high.  The histogram value is
        # still computed and reported so a real discrepancy is visible.
        self.floor = -self.sensor_h
        h, e = np.histogram(xyz[:, 2], bins=150)
        guess = float(e[int(np.argmax(h))])
        if abs(guess - self.floor) > 0.25:
            rospy.logwarn_throttle(20, 'floor: using known %.2f (histogram says %.2f - '
                                       'probably a wall/table, ignored)', self.floor, guess)

        g = rospy.get_param
        cx, cy, cz = g('~x'), g('~y'), g('~z')          # cz is ABOVE THE FLOOR
        sx, sy, sz = g('~sx'), g('~sy'), g('~sz')
        czs = cz + self.floor                            # -> sensor frame
        lo = np.array([cx - sx/2, cy - sy/2, czs - sz/2])
        hi = np.array([cx + sx/2, cy + sy/2, czs + sz/2])

        m = np.all((xyz >= lo) & (xyz <= hi), axis=1)
        inside = xyz[m]

        ma = MarkerArray()
        ma.markers.append(box_marker(msg.header, lo, hi, 'roi', 0, (0.1, 1.0, 0.1)))

        det = None
        mi = int(g('~min_inliers'))
        if g('~detect') and len(inside) > 3 * mi:
            det = detect_frame(inside, radius=0.20, radius_tol=0.07, thresh=0.03,
                               iters=int(g('~ransac_iters')), min_inliers=mi,
                               base=1.10, rise=1.10, tol=g('~side_tol'),
                               max_candidates=7, rng=np.random.default_rng(0))
        if det is not None:
            P, d = det
            # points actually on each fitted sphere -> tells you if you are near the density floor
            npts = [int((np.abs(np.linalg.norm(inside - s, axis=1) - 0.20) < 0.03).sum())
                    for s in P]
            for i, c in enumerate(P):
                mk = Marker(header=msg.header, ns='fit', id=i, type=Marker.SPHERE, action=Marker.ADD)
                mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = c
                mk.pose.orientation.w = 1.0
                mk.scale.x = mk.scale.y = mk.scale.z = 0.40
                mk.color.r, mk.color.g, mk.color.a = 1.0, 0.3, 0.55
                ma.markers.append(mk)
            status = 'FRAME OK  sides %.3f/%.3f/%.3f  pts/sphere %d/%d/%d' % (
                d[0], d[1], d[2], npts[0], npts[1], npts[2])
        else:
            for i in range(3):     # clear stale fit markers
                ma.markers.append(Marker(header=msg.header, ns='fit', id=i, action=Marker.DELETE))
            status = 'no frame detected in ROI'

        self.pub_m.publish(ma)
        self.pub_p.publish(pc2.create_cloud_xyz32(msg.header, inside.astype(np.float32)))
        rospy.loginfo('ROI x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f] (floor %.2f) | %d pts inside | %s',
                      lo[0], hi[0], lo[1], hi[1], lo[2], hi[2], self.floor, len(inside), status)

        if g('~show_cmd'):
            rospy.set_param('~show_cmd', False)
            print('\n---------------- USE THIS ----------------')
            print('roslaunch self_navigation sphere_centroid.launch \\')
            print('    sensor:=ouster run_id:=%dm_run1 \\' % round(np.hypot(cx, cy)))
            print('    roi_x:=%.2f roi_y:=%.2f roi_z:=%.2f \\' % (cx, cy, cz))
            print('    roi_sx:=%.2f roi_sy:=%.2f roi_sz:=%.2f' % (sx, sy, sz))
            print('(repeat with sensor:=helios, IDENTICAL args)')
            print('------------------------------------------\n')
        self.last_report = now

if __name__ == '__main__':
    rospy.init_node('roi_viz')
    RoiViz()
    rospy.spin()
