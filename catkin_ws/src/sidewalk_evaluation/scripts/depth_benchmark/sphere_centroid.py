#!/usr/bin/env python3
# sphere_centroid.py — replicate the Schulte-Tigges et al. (Sensors 2022, 22, 7146) Scenario-1
# centroid measurement: find the 3-sphere calibration frame in a LiDAR cloud via RANSAC sphere
# fitting, compute the triangle centroid (mean of the 3 sphere centers), and report per-axis
# precision (mean/std) over many frames — the depth study's Figure-6 metric.
#
# Frame geometry (depth study): 3 spheres, 40 cm diameter (r=0.20 m), equilateral triangle, ~110 cm
# between centers. Only frames where all 3 spheres are found are counted.
#
# Core math (fit/ransac/detect/validate) is pure numpy so it is unit-testable offline without ROS
# or hardware — see sphere_centroid_selftest.py. The ROS node wraps it for live/replay clouds.
import numpy as np

# ------------------------- core geometry (numpy only) -------------------------

def fit_sphere_algebraic(pts):
    """Linear least-squares sphere fit. pts:(N,3) -> (center(3,), radius). Fast hypothesis fit."""
    A = np.hstack([2.0 * pts, np.ones((len(pts), 1))])
    b = np.sum(pts * pts, axis=1)
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    c = sol[:3]
    radius = float(np.sqrt(max(sol[3] + c.dot(c), 1e-12)))
    return c, radius

def refine_center_fixed_radius(pts, c0, radius, iters=12):
    """Gauss-Newton refine of the center for a KNOWN radius. LiDAR only sees the front cap of a
    sphere, which biases a free fit along the view axis; constraining the radius fixes that."""
    c = np.asarray(c0, float).copy()
    for _ in range(iters):
        v = pts - c
        dist = np.maximum(np.linalg.norm(v, axis=1), 1e-6)
        J = -v / dist[:, None]          # d(dist)/dc
        f = dist - radius               # residual
        try:
            dc = -np.linalg.solve(J.T @ J, J.T @ f)
        except np.linalg.LinAlgError:
            break
        c = c + dc
        if np.linalg.norm(dc) < 1e-5:
            break
    return c

def ransac_sphere(pts, radius=0.20, radius_tol=0.05, thresh=0.02,
                  iters=300, min_inliers=20, rng=None):
    """Detect ONE fixed-radius sphere via RANSAC with local (neighbourhood) sampling.
    Returns (center(3,), radius, inlier_mask) or None."""
    n = len(pts)
    if n < min_inliers: # if not enough points, end
        return None
    if rng is None: #use defualt rng if undefined
        rng = np.random.default_rng()
    r2 = (2.2 * radius) ** 2            # sample 4 points within ~1 sphere-diameter of a seed
    best_cnt, best_mask = 0, None #initialize best cases
    for _ in range(iters): #Main loop, goes a predefined number of iterations (iters)
        i0 = int(rng.integers(n)) #pick random point in point list (index)
        near = np.where(np.sum((pts - pts[i0]) ** 2, axis=1) < r2)[0] #return 1d array of indexes of 2d array pts that meet the condition of being 44cm from the random, including the seed
        if len(near) < 4: #ignore seeds with less than 4 points
            continue
        idx = rng.choice(near, 4, replace=False) #grab 4 random points from near (at least 4 surface points required to define a sphere)
        try: #this check will fit the points to a sphere, then check if it makes sense. if it doesn't then it keeps it
            c, r = fit_sphere_algebraic(pts[idx]) #function to fit the points to a sphere
            # above we see function returns center (xyz), and radius
        except np.linalg.LinAlgError:
            continue
        if not (radius - radius_tol <= r <= radius + radius_tol): #flat surfaces have infinite radius, we need them to be close to our expected radius
            continue
        d = np.abs(np.linalg.norm(pts - c, axis=1) - r) #same method to shift array to get distances from point c. d will be another array, but of distances from the surface of sphere.(-r creates this offset, and absolute(abs) fits it to both sides)
        mask = d < thresh #points truly on the surface would be around 0 +- our chosen threshhold representing sensor accuracy.
        cnt = int(mask.sum()) #total number of points that are on the surface (return of mask. mask is a list of booleans, trues return 1 then u sum them)
        if cnt > best_cnt: #remake our best count as we go through iterations
            best_cnt, best_mask = cnt, mask #point of keeping mask is that its index aligned, so we know which points in pts are true in mask, and thus part of the sphere
    if best_cnt < min_inliers: #if we dont get enough inliers, so no sphere here
        return None
    inl = pts[best_mask] #inl is the winning sphere's points, which is pts filtered by best_mask
    c0, _ = fit_sphere_algebraic(inl) #remake the center with the full list of points on the sphere, throw out radius into _
    c = refine_center_fixed_radius(inl, c0, radius)     # known-radius refine
    mask = np.abs(np.linalg.norm(pts - c, axis=1) - radius) < thresh #recompute the mak, since we recalulated the center and moved it
    if int(mask.sum()) < min_inliers: #quick double check its still good, needed after new mask calc
        return None
    return c, radius, mask #all good, return final circle

def detect_three_spheres(pts, radius=0.20, radius_tol=0.05, thresh=0.02,
                         iters=300, min_inliers=20, rng=None):
    """Sequential RANSAC: find a sphere, remove its inliers, repeat 3x. Returns (3,3) or None."""
    remaining = pts
    centers = []
    for _ in range(3):
        res = ransac_sphere(remaining, radius, radius_tol, thresh, iters, min_inliers, rng)
        if res is None:
            return None
        c, _, mask = res
        centers.append(c)
        remaining = remaining[~mask]
    return np.array(centers)

def expected_sides(base=1.10, rise=None):
    """Side lengths of the target triangle, sorted. base = bottom-pair separation,
    rise = vertical height from the bottom pair up to the apex sphere.
    rise=None -> equilateral (the depth study's frame). Our frame: base=1.10, rise=1.10 -> legs 1.230."""
    if rise is None:
        rise = base * np.sqrt(3) / 2.0
    leg = float(np.hypot(base / 2.0, rise))
    return np.sort(np.array([base, leg, leg]))

def validate_triangle(centers, base=1.10, rise=1.10, tol=0.12):
    """Gate on the known frame geometry (shape-agnostic: compares sorted side lengths).
    Returns (ok, pairwise_distances(3,) in fixed order 01,02,12)."""
    d = np.array([np.linalg.norm(centers[i] - centers[j]) for i, j in ((0, 1), (0, 2), (1, 2))])
    ok = bool(np.all(np.abs(np.sort(d) - expected_sides(base, rise)) < tol))
    return ok, d

def detect_frame(pts, radius=0.20, radius_tol=0.05, thresh=0.02, iters=300, min_inliers=20,
                 base=1.10, rise=1.10, tol=0.12, max_candidates=6, rng=None, z_tol=0.08):
    """Robust frame detection: extract up to `max_candidates` spheres, then pick the TRIPLE whose
    side lengths best match the known geometry. Beats taking the first 3 detections, because
    clutter blobs can out-rank a sparse apex sphere. Returns (centers(3,3), dists(3,)) or None."""
    import itertools
    remaining = pts #working copy
    cands = [] # list of center candidates
    for _ in range(max_candidates): #maximum 7 total spheres
        res = ransac_sphere(remaining, radius, radius_tol, thresh, iters, min_inliers, rng)
        if res is None: #ransac sphere bricked
            break
        c, _, mask = res # store res in its variables
        cands.append(c) #add center candidate to list
        remaining = remaining[~mask] #remove candidates points from list
    if len(cands) < 3: # not enough spheres, its bricked
        return None
    exp = expected_sides(base, rise) # expected triangle
    best = None
    for combo in itertools.combinations(range(len(cands)), 3): # amount of exclusive combinations of 3 elements in candidates
        P = np.array([cands[i] for i in combo]) #create triangle with combo 1
        ok, d = validate_triangle(P, base, rise, tol) #check if triangle fits
        if not ok: #check next combo if it doesnt fit
            continue
        # canonical order [bottom_a, bottom_b, apex] so side identities are stable
        apex = int(np.argmax(P[:, 2])) #highest z index is top sphere
        bottom = sorted([i for i in range(3) if i != apex], key=lambda i: P[i, 1]) # bottom pair ordered by y
        Po = P[[bottom[0], bottom[1], apex]] #ordered triangle points
        # SHAPE checks: side lengths alone let a decoy pass, so also require the frame's
        # actual vertical structure -- bottom pair level, apex the right height above them.
        if abs(Po[0, 2] - Po[1, 2]) > z_tol: #bottom pair must be level
            continue
        if abs((Po[2, 2] - 0.5 * (Po[0, 2] + Po[1, 2])) - rise) > z_tol: #apex rise must match
            continue
        do = np.array([np.linalg.norm(Po[i] - Po[j]) for i, j in ((0, 1), (0, 2), (1, 2))]) #sides in canonical order
        err = float(np.abs(np.sort(do) - exp).sum()) # measure error of ok triangle
        if best is None or err < best[0]: #make new best combo
            best = (err, Po, do)
    if best is None:
        return None
    return best[1], best[2]

def _quat_to_R(q):
    """quaternion -> 3x3 rotation matrix (for the tracking ROI / odom logging)"""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]])

def _lookup(buf, target, source, stamp):
    """returns (R, T) taking a point in `source` -> `target`, or None"""
    import rospy as _r
    tr = buf.lookup_transform(target, source, stamp, _r.Duration(0.05))
    return (_quat_to_R(tr.transform.rotation),
            np.array([tr.transform.translation.x,
                      tr.transform.translation.y,
                      tr.transform.translation.z]))

def centroid_of(centers):
    return np.asarray(centers).mean(axis=0)

# ------------------------------- ROS node -------------------------------

def _run_node():
    import rospy
    from sensor_msgs.msg import PointCloud2
    from geometry_msgs.msg import PointStamped
    from visualization_msgs.msg import Marker, MarkerArray
    _DT = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'} #map ros PointField to numpy dtype string (float32)

    class SphereCentroid:
        def __init__(self):
            g = rospy.get_param #get all the params from function call
            self.topic  = g('~cloud_topic', '/ouster/points')
            self.radius = g('~sphere_radius', 0.20)
            self.rtol   = g('~radius_tol', 0.05)
            self.thresh = g('~inlier_thresh', 0.02)
            self.iters  = g('~ransac_iters', 300)
            self.mininl = g('~min_inliers', 20)
            self.base   = g('~triangle_base', 1.10)   # bottom-pair separation
            self.rise   = g('~triangle_rise', 1.10)   # vertical rise to the apex sphere
            self.stol   = g('~side_tol', 0.12)
            self.maxcand= g('~max_candidates', 6)
            # --- Test 3 (dynamic): follow a target that is static in `track_frame` ---
            # '' = fixed ROI (Tests 1/2).  Set track_frame:=odom + target_x/y/z to make the ROI
            # follow the frame as the robot drives toward it. Same detector either way, so the
            # static baseline and the moving run are directly comparable.
            self.track  = g('~track_frame', '')      # frame to LOG the centroid in
            self.trackroi = bool(g('~track_roi', False))  # also MOVE the roi with it?
            self.tgt    = np.array([g('~target_x', 0.0), g('~target_y', 0.0), g('~target_z', 0.0)])
            self.tf_buf = None
            if self.track:
                import tf2_ros
                self.tf_buf = tf2_ros.Buffer(rospy.Duration(60.0))
                self.tf_lis = tf2_ros.TransformListener(self.tf_buf)
                rospy.loginfo('tracking target (%.2f,%.2f,%.2f) in frame %s',
                              self.tgt[0], self.tgt[1], self.tgt[2], self.track)
            self.ztol   = g('~apex_z_tol', 0.08)     # bottom-pair level + apex-rise check     # extract N spheres, pick best matching triple
            self.report = g('~report_every', 50)
            self.target = int(g('~target_samples', 0))  # stop after N accepted (0 = run until Ctrl+C)
            self.csv    = g('~csv_path', '')
            self.roi = [g('~roi_xmin', 0.5), g('~roi_xmax', 30.0),
                        g('~roi_ymin', -10.0), g('~roi_ymax', 10.0),
                        g('~roi_zmin', -3.0), g('~roi_zmax', 3.0)]
            self.rng = np.random.default_rng(0) #consistent seed
            self.samples, self.total = [], 0
            self.pub_c = rospy.Publisher('~centroid', PointStamped, queue_size=5)
            self.pub_m = rospy.Publisher('~markers', MarkerArray, queue_size=5)
            self.fcsv = open(self.csv, 'w') if self.csv else None
            if self.fcsv:
                self.fcsv.write('n,x,y,z,base,leg1,leg2,n_bot1,n_bot2,n_apex,tx,ty,tz,stamp\n')
            rospy.Subscriber(self.topic, PointCloud2, self.cb, queue_size=2, buff_size=2 ** 24)
            rospy.loginfo('sphere_centroid: %s  r=%.2f base=%.2f rise=%.2f (sides %s)  ROI=%s',
                          self.topic, self.radius, self.base, self.rise,
                          np.round(expected_sides(self.base, self.rise), 3), self.roi)

        def cb(self, msg):
            self.total += 1 #count callbacks from pointcloud
            names = [f.name for f in msg.fields]
            dt = np.dtype(dict(names=names, formats=[_DT[f.datatype] for f in msg.fields],
                               offsets=[f.offset for f in msg.fields], itemsize=msg.point_step))
            a = np.frombuffer(msg.data, dtype=dt) # view data lightly
            xyz = np.stack([a['x'], a['y'], a['z']], axis=1).astype(np.float64) #cast data to float64, stacked as N,3 plain.
            xyz = xyz[np.isfinite(xyz).all(axis=1)] #drop bad/infinite distance points
            roi = list(self.roi)
            if self.trackroi and self.tf_buf is not None:
                # place the ROI where the (static) target currently appears to the sensor
                try:
                    R, T = _lookup(self.tf_buf, msg.header.frame_id, self.track, msg.header.stamp)
                except Exception:
                    return                      # no tf yet -> cannot place the box, skip frame
                ctr = R @ self.tgt + T
                hx = (self.roi[1]-self.roi[0])/2.0
                hy = (self.roi[3]-self.roi[2])/2.0
                hz = (self.roi[5]-self.roi[4])/2.0
                roi = [ctr[0]-hx, ctr[0]+hx, ctr[1]-hy, ctr[1]+hy, ctr[2]-hz, ctr[2]+hz]
            x0, x1, y0, y1, z0, z1 = roi 
            m = ((xyz[:, 0] >= x0) & (xyz[:, 0] <= x1) & (xyz[:, 1] >= y0) &
                 (xyz[:, 1] <= y1) & (xyz[:, 2] >= z0) & (xyz[:, 2] <= z1)) # crop the ROI to the area we'll use
            pts = xyz[m] # set pts to final pointcloud
            if len(pts) < 3 * self.mininl: #need at least 3 spheres worth of points
                return
            res = detect_frame(pts, self.radius, self.rtol, self.thresh, self.iters,
                               self.mininl, self.base, self.rise, self.stol,
                               self.maxcand, self.rng, self.ztol) # big function call, will return all three spheres.
            if res is None: #if its bricked
                return
            centers, d = res # get centers and triangle dimensions
            # points actually on each fitted sphere -> the density behind sigma
            npts = [int((np.abs(np.linalg.norm(pts - sc, axis=1) - self.radius)
                         < self.thresh).sum()) for sc in centers]
            c = centroid_of(centers) #get centroid of spheres
            # centroid in the tracking frame: for a STATIC target this should
            # be a fixed point, so its scatter = odometry + lidar + distortion
            tc = [float("nan")] * 3
            if self.track and self.tf_buf is not None:
                try:
                    R2, T2 = _lookup(self.tf_buf, self.track,
                                     msg.header.frame_id, msg.header.stamp)
                    tc = list(R2 @ c + T2)
                except Exception:
                    pass
            self.samples.append(c)
            self._publish(msg.header, centers, c) #publish the results
            if self.fcsv:
                self.fcsv.write('%d,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,%d,%d,%d,%.5f,%.5f,%.5f,%.6f\n' %
                                (len(self.samples), c[0], c[1], c[2], d[0], d[1], d[2],
                                 npts[0], npts[1], npts[2],
                                  tc[0], tc[1], tc[2], msg.header.stamp.to_sec()))
                self.fcsv.flush()
            done = bool(self.target) and len(self.samples) >= self.target
            if done or len(self.samples) % self.report == 0:
                self._summary()
            if done: #hit the requested sample count -> finish cleanly
                if self.fcsv:
                    self.fcsv.close(); self.fcsv = None
                rospy.loginfo('reached target of %d samples -> stopping', self.target)
                rospy.signal_shutdown('target reached')

        def _publish(self, header, centers, c):
            ps = PointStamped(header=header)
            ps.point.x, ps.point.y, ps.point.z = c
            self.pub_c.publish(ps)
            ma = MarkerArray()
            for i, cc in enumerate(centers):
                mk = Marker(header=header, ns='spheres', id=i, type=Marker.SPHERE, action=Marker.ADD)
                mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = cc
                mk.pose.orientation.w = 1.0
                mk.scale.x = mk.scale.y = mk.scale.z = 2 * self.radius
                mk.color.g, mk.color.b, mk.color.a = 0.8, 1.0, 0.5
                ma.markers.append(mk)
            mc = Marker(header=header, ns='centroid', id=99, type=Marker.SPHERE, action=Marker.ADD)
            mc.pose.position.x, mc.pose.position.y, mc.pose.position.z = c
            mc.pose.orientation.w = 1.0
            mc.scale.x = mc.scale.y = mc.scale.z = 0.08
            mc.color.r, mc.color.a = 1.0, 1.0
            ma.markers.append(mc)
            self.pub_m.publish(ma)

        def _summary(self):
            a = np.array(self.samples)
            if len(a) <2:
                return
            mean, std = a.mean(axis=0), a.std(axis=0, ddof=1) #sample std, matches analyze centroid
            rospy.loginfo('[N=%d/%d] centroid mean=(%.3f,%.3f,%.3f)m  std=(%.4f,%.4f,%.4f)m (x,y,z)',
                          len(a), self.total, *mean, *std)

    rospy.init_node('sphere_centroid')
    SphereCentroid()
    rospy.spin()

if __name__ == '__main__':
    _run_node()
