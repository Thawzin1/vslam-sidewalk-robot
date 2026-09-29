#!/usr/bin/env python3
"""Test 2 - static reference plane. Crop a box around the plate, measure its extent per axis.

Deliberately contains NO model fitting. Test 2 exists to validate Test 1: if the sphere result
came from RANSAC favouring one sensor, a fit-free measurement would disagree with it. So this is
min/max on a cropped cloud and nothing more.

Sensor frame convention (both sensors are identity-rotated vs base_link, so this holds for both):
    x = forward  -> plate is flat, so ext_x should be ~0. This is the interesting one: whatever
                    it reads is range noise plus edge fringing.
    y = left     -> plate WIDTH
    z = up       -> plate HEIGHT

Reports raw min/max extent AND a 1st-99th percentile extent. The gap between them quantifies
outliers / edge fringing directly, rather than just noting that it happens.

CSV: n,ext_x,ext_y,ext_z,npts,p_ext_x,p_ext_y,p_ext_z,resid,tilt_deg,
     skew,resid_c,ext_x_c,n_rej,stamp   (_c = after robust outlier rejection)
"""
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from visualization_msgs.msg import Marker


def extents(p):
    """raw (min/max) and robust (1st-99th percentile) extent per axis"""
    raw = p.max(axis=0) - p.min(axis=0)
    lo = np.percentile(p, 1, axis=0)
    hi = np.percentile(p, 99, axis=0)
    return raw, hi - lo


def plane_residual(p, nsig=3.0):
    """Fit a plane; return raw stats, robust (outlier-rejected) stats, and the rejection count.

    ext_x conflates sensor range noise (what we want) with any lean in the plate (which we do
    not) and with multipath / edge-fringing outliers (which we want to measure separately, not
    have contaminate the noise figure). A 1 deg tilt over 1.2 m contributes 21 mm to ext_x, far
    more than the ~5-10 mm of noise. The residual is tilt-invariant so it separates that out.

    MULTIPATH: light going sensor -> wall/bystander -> plate -> sensor travels farther than the
    direct path, so those returns are reported BEHIND the plate -- never in front. That makes the
    residual distribution one-sided, which `skew` measures directly: the Helios reads about +4.5
    here while the Ouster reads about -0.3 (symmetric, i.e. plain noise).

    Rejection is done against the FITTED PLANE, not a fixed x window. Cropping x would define
    away the very quantity ext_x measures; sigma-clipping against the fit does not, and the
    rejected count is itself reported as the fringing/multipath measure.

    Least-squares, not RANSAC -- no model selection, nothing that could favour one sensor, so the
    fit-free character of Test 2 is preserved. Raw ext_x is still reported as the depth study does.
    """
    c = p.mean(axis=0)
    # smallest-singular-vector of the centred cloud = plane normal
    n = np.linalg.svd(p - c, full_matrices=False)[2][-1]
    r = (p - c) @ n
    if n[0] < 0:                 # orient so +r means FARTHER from the sensor
        r = -r
    rms = float(np.sqrt(np.mean(r ** 2)))
    tilt = float(np.degrees(np.arccos(min(1.0, abs(n[0])))))
    sd = r.std()
    skew = float(((r - r.mean()) ** 3).mean() / sd ** 3) if sd > 1e-9 else 0.0

    keep = np.abs(r - r.mean()) < nsig * sd
    if keep.sum() < 10:
        keep = np.ones(len(p), bool)
    q = p[keep]
    cc = q.mean(axis=0)
    nn = np.linalg.svd(q - cc, full_matrices=False)[2][-1]
    rms_c = float(np.sqrt(np.mean(((q - cc) @ nn) ** 2)))
    ext_x_c = float(q[:, 0].max() - q[:, 0].min())
    return rms, tilt, skew, rms_c, ext_x_c, int((~keep).sum())


class PlaneExtent(object):
    def __init__(self):
        g = rospy.get_param
        self.topic = g('~cloud_topic', '/helios/points')
        # Box is centre + size, and roi_z is measured ABOVE THE FLOOR (sensor_h converts it into
        # the sensor frame), so the same numbers work for either lidar. Re-read every frame, so
        # `rosparam set /plane_extent/roi_z 1.22` retunes it live with no relaunch.
        self.sensor_h = g('~sensor_h', 0.97)
        self.read_roi()
        self.min_pts = int(g('~min_points', 30))
        self.nsig = g('~reject_sigma', 3.0)   # robust plane-outlier rejection threshold
        self.target = int(g('~target_samples', 0))    # 0 = run until Ctrl+C
        self.report = int(g('~report_every', 50))
        self.csv = g('~csv_path', '')
        self.truth_y = g('~truth_y', 0.0)             # 0 = unknown, skip accuracy line
        self.truth_z = g('~truth_z', 0.0)

        self.rows = []
        self.seen = 0
        self.pub_box = rospy.Publisher('~box', Marker, queue_size=2, latch=True)
        self.pub_pts = rospy.Publisher('~cropped', PointCloud2, queue_size=2)
        self.fcsv = open(self.csv, 'w') if self.csv else None
        if self.fcsv:
            self.fcsv.write('n,ext_x,ext_y,ext_z,npts,p_ext_x,p_ext_y,p_ext_z,'
                            'resid,tilt_deg,skew,resid_c,ext_x_c,n_rej,stamp\n')
        rospy.Subscriber(self.topic, PointCloud2, self.cb, queue_size=2, buff_size=2 ** 24)
        rospy.loginfo('plane_extent: %s  ROI x[%.2f %.2f] y[%.2f %.2f] z[%.2f %.2f]',
                      self.topic, *self.roi)

    def read_roi(self):
        """centre+size (roi_z above floor) -> six bounds in the sensor frame"""
        g = rospy.get_param
        cx, cy, cz = g('~roi_x', 7.0), g('~roi_y', 0.0), g('~roi_z', 1.22)
        sx, sy, sz = g('~roi_sx', 0.30), g('~roi_sy', 0.85), g('~roi_sz', 1.45)
        cz -= self.sensor_h                       # floor-relative -> sensor frame
        self.roi = [cx - sx / 2.0, cx + sx / 2.0,
                    cy - sy / 2.0, cy + sy / 2.0,
                    cz - sz / 2.0, cz + sz / 2.0]

    def publish_box(self, header):
        m = Marker(header=header, ns='plane_roi', id=0, type=Marker.CUBE, action=Marker.ADD)
        m.pose.position.x = 0.5 * (self.roi[0] + self.roi[1])
        m.pose.position.y = 0.5 * (self.roi[2] + self.roi[3])
        m.pose.position.z = 0.5 * (self.roi[4] + self.roi[5])
        m.pose.orientation.w = 1.0
        m.scale.x = max(self.roi[1] - self.roi[0], 1e-3)
        m.scale.y = max(self.roi[3] - self.roi[2], 1e-3)
        m.scale.z = max(self.roi[5] - self.roi[4], 1e-3)
        m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.9, 0.3, 0.18
        self.pub_box.publish(m)

    def cb(self, msg):
        if not self.target:          # tuning mode: pick up rosparam edits without a relaunch
            self.read_roi()
        self.publish_box(msg.header)
        names = [f.name for f in msg.fields]
        dt = np.dtype({'names': names,
                       'formats': ['<f4' if f.datatype == 7 else '<u4' if f.datatype == 6
                                   else '<f8' if f.datatype == 8 else '<u1' for f in msg.fields],
                       'offsets': [f.offset for f in msg.fields],
                       'itemsize': msg.point_step})
        a = np.frombuffer(msg.data, dtype=dt)
        p = np.stack([a['x'], a['y'], a['z']], axis=1).astype(np.float64)
        p = p[np.isfinite(p).all(axis=1)]
        r = self.roi
        m = ((p[:, 0] >= r[0]) & (p[:, 0] <= r[1]) &
             (p[:, 1] >= r[2]) & (p[:, 1] <= r[3]) &
             (p[:, 2] >= r[4]) & (p[:, 2] <= r[5]))
        q = p[m]
        self.seen += 1

        # always republish the crop so the box can be tuned live in RViz
        self.pub_pts.publish(pc2.create_cloud_xyz32(msg.header, q.tolist()))

        if len(q) < self.min_pts:
            if self.seen % 20 == 0:
                rospy.logwarn_throttle(5, 'only %d points in box (need %d) - is the box on the plate?',
                                       len(q), self.min_pts)
            return

        raw, rob = extents(q)
        resid, tilt, skew, resid_c, ext_x_c, n_rej = plane_residual(q, self.nsig)
        # column order must match the CSV header and the indices used in summary()
        self.rows.append(np.concatenate([raw, [len(q)], rob,
                                         [resid, tilt, skew, resid_c, ext_x_c, n_rej]]))
        if self.fcsv:
            self.fcsv.write('%d,%.5f,%.5f,%.5f,%d,%.5f,%.5f,%.5f,%.5f,%.3f,'
                            '%.3f,%.5f,%.5f,%d,%.6f\n'
                            % (len(self.rows), raw[0], raw[1], raw[2], len(q),
                               rob[0], rob[1], rob[2], resid, tilt,
                               skew, resid_c, ext_x_c, n_rej,
                               msg.header.stamp.to_sec()))
            self.fcsv.flush()

        if not self.target and len(self.rows) % 10 == 0:
            # tuning mode: fast compact feedback while the box is being adjusted
            rospy.loginfo('TUNE  n=%4d  w %6.1f mm (%+6.1f)  h %7.1f mm (%+7.1f)  '
                          'depth %5.1f/%5.1f mm  resid %4.1f/%4.1f mm  yaw %+.2f  skew %+.2f',
                          len(q), raw[1] * 1000, (raw[1] - self.truth_y) * 1000 if self.truth_y else 0,
                          raw[2] * 1000, (raw[2] - self.truth_z) * 1000 if self.truth_z else 0,
                          raw[0] * 1000, ext_x_c * 1000, resid * 1000, resid_c * 1000, tilt, skew)

        done = bool(self.target) and len(self.rows) >= self.target
        if done or len(self.rows) % self.report == 0:
            self.summary()
        if done:
            if self.fcsv:
                self.fcsv.close(); self.fcsv = None
            rospy.loginfo('reached target of %d samples -> stopping', self.target)
            rospy.signal_shutdown('target reached')

    def summary(self):
        A = np.array(self.rows)
        n = len(A)
        rospy.loginfo('--- n=%d  (%.0f%% of %d frames had enough points)',
                      n, 100.0 * n / max(self.seen, 1), self.seen)
        rospy.loginfo('    pts/frame  %.0f', A[:, 3].mean())
        for i, ax in enumerate(('x', 'y', 'z')):
            rospy.loginfo('    ext_%s  %.4f +/- %.4f m   (p1-p99 %.4f)  [mm: sd %.1f]',
                          ax, A[:, i].mean(), A[:, i].std(ddof=1),
                          A[:, 4 + i].mean(), A[:, i].std(ddof=1) * 1000)
        rospy.loginfo('    plane resid %.1f mm rms   tilt %.2f deg  '
                      '(tilt inflates ext_x by ~%.1f mm over this plate)',
                      A[:, 7].mean() * 1000, A[:, 8].mean(),
                      1000 * A[:, 2].mean() * np.tan(np.radians(A[:, 8].mean())))
        rospy.loginfo('    ROBUST (%.0f-sigma reject): ext_x %.1f mm (raw %.1f)  resid %.1f mm '
                      '(raw %.1f)  rejected %.2f pts/frame (%.2f%%)  skew %+.2f',
                      self.nsig, A[:, 11].mean() * 1000, A[:, 0].mean() * 1000,
                      A[:, 10].mean() * 1000, A[:, 7].mean() * 1000,
                      A[:, 12].mean(), 100 * A[:, 12].mean() / A[:, 3].mean(), A[:, 9].mean())
        for truth, i, ax in ((self.truth_y, 1, 'y'), (self.truth_z, 2, 'z')):
            if truth:
                rospy.loginfo('    ext_%s error vs %.3f m: %+.1f mm', ax, truth,
                              (A[:, i].mean() - truth) * 1000)


if __name__ == '__main__':
    rospy.init_node('plane_extent')
    PlaneExtent()
    rospy.spin()
