#!/usr/bin/env python3
"""measure_camera_tilt.py — measure the camera's unmodelled roll and pitch.

THE IDEA
    Park the robot on flat, level floor. Transform the camera's depth cloud into
    base_footprint, whose z=0 plane IS the floor by construction. If the mounting
    angles declared in the launch file are correct, the reconstructed floor comes
    out flat at z=0. Any tilt in the fitted plane is the angle the model does NOT
    know about, and can be added to cam_roll / cam_pitch directly.

WHY THIS MATTERS MORE THAN THE HEIGHT
    robot_frames.yaml's own header states it: a 1 degree pitch error displaces a
    point at 20 m by about 35 cm, and at 1 m by 1.7 cm. A mounting angle you
    cannot see by eye is invisible up close and severe at range — which is
    exactly where the camera is relied on for obstacle detection. Height errors
    are constant; angular errors grow with distance.

METHOD
    RANSAC plane fit, not least squares. The camera sees the floor AND whatever
    is standing on it; least squares would be dragged by every chair leg and
    wall. RANSAC finds the plane that the most points agree on and ignores the
    rest, which for a floor-filling view is the floor.

WHAT IT DOES NOT TELL YOU — READ THIS BEFORE BELIEVING A RESULT
    Yaw. A rotation about the vertical axis leaves a horizontal plane
    horizontal, so the floor carries no information about it. Yaw needs a
    different target — a known straight edge, or driving a straight line and
    comparing heading.

    CAUSE. This is the important one, and an earlier version of this docstring
    got it wrong. It said the confound was "the floor is not level". It is not.
    base_footprint is rigidly bolted to the robot body (base_link minus a fixed
    0.13228 m), NOT to the world — so a uniform floor slope tilts the robot,
    the camera and base_footprint together and CANCELS OUT. It is invisible
    here. The confounds that actually survive are:

        (i)   the camera tilted in its mount
        (ii)  the robot body pitched relative to its own wheel-contact plane
              (soft tyre, forward payload, sagging suspension)
        (iii) the floor patch inside the 1.2-5.0 m fit window sloping
              DIFFERENTLY from the patch under the wheels — a ramp, a crown,
              a threshold, a settled slab

    What this script reports is the SUM of those three. It cannot decompose
    them, and re-running it after applying a correction cannot either: the
    second fit constrains the same sum as the first, so getting ~0 back is
    arithmetic confirming itself, not evidence about cause. Turning the robot
    and re-running separates (iii) from (i)+(ii), but never (i) from (ii).

    TO ESTABLISH CAUSE, USE GRAVITY:
        rosrun sidewalk_perception check_gravity.py
    The camera's accelerometer measures true vertical, which owes nothing to
    the transform tree, and separates all three outright. Run it before
    trusting any number from here as a mounting angle.

    usage:  rosrun sidewalk_perception measure_camera_tilt.py
            rosrun sidewalk_perception measure_camera_tilt.py _frames:=5
"""
import math

import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2


def quat_to_matrix(q):
    """Rotation matrix from a geometry_msgs Quaternion."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def read_xyz(msg, stride):
    """Raw-buffer read of the x/y/z fields. Avoids a per-point Python loop over
    2.3 million points, which takes minutes."""
    off = {f.name: f.offset for f in msg.fields}
    n = len(msg.data) // msg.point_step
    buf = np.frombuffer(msg.data, dtype=np.uint8)[:n * msg.point_step]
    buf = buf.reshape(n, msg.point_step)[::stride]
    out = np.empty((buf.shape[0], 3), dtype=np.float32)
    for i, k in enumerate(("x", "y", "z")):
        o = off[k]
        out[:, i] = buf[:, o:o + 4].copy().view(np.float32).ravel()
    return out[np.isfinite(out).all(axis=1)]


def ransac_plane(pts, iters=600, tol=0.02, rng=None):
    """Return (normal, d, inlier_mask) for the plane n.p + d = 0 with most support."""
    rng = rng or np.random.default_rng(0)
    best = (None, None, None, -1)
    n_pts = len(pts)
    for _ in range(iters):
        idx = rng.choice(n_pts, 3, replace=False)
        p0, p1, p2 = pts[idx]
        nrm = np.cross(p1 - p0, p2 - p0)
        norm = np.linalg.norm(nrm)
        if norm < 1e-9:
            continue
        nrm = nrm / norm
        d = -nrm.dot(p0)
        inl = np.abs(pts.dot(nrm) + d) < tol
        c = int(inl.sum())
        if c > best[3]:
            best = (nrm, d, inl, c)
    nrm, d, inl, _ = best
    if nrm is None:
        return None, None, None
    # refine on the inliers via SVD — RANSAC picks the support set, SVD gives the
    # accurate normal for it.
    sel = pts[inl]
    c = sel.mean(axis=0)
    _, _, vt = np.linalg.svd(sel - c, full_matrices=False)
    nrm = vt[2] / np.linalg.norm(vt[2])
    if nrm[2] < 0:
        nrm = -nrm                      # keep it pointing up
    return nrm, -nrm.dot(c), inl


def main():
    rospy.init_node("measure_camera_tilt", anonymous=True, disable_signals=True)
    topic = rospy.get_param("~cloud", "/zedx_front/zed_node/point_cloud/cloud_registered")
    target = rospy.get_param("~frame", "base_footprint")
    stride = int(rospy.get_param("~stride", 37))
    frames = int(rospy.get_param("~frames", 3))
    zlo = float(rospy.get_param("~z_min", -0.40))
    zhi = float(rospy.get_param("~z_max", 0.35))
    xlo = float(rospy.get_param("~x_min", 1.2))
    xhi = float(rospy.get_param("~x_max", 5.0))

    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    rospy.sleep(1.5)

    results = []
    for i in range(frames):
        msg = rospy.wait_for_message(topic, PointCloud2, timeout=20)
        tr = buf.lookup_transform(target, msg.header.frame_id,
                                  rospy.Time(0), rospy.Duration(4.0))
        R = quat_to_matrix(tr.transform.rotation)
        t = np.array([tr.transform.translation.x,
                      tr.transform.translation.y,
                      tr.transform.translation.z])

        pts = read_xyz(msg, stride).astype(np.float64)
        pts = pts.dot(R.T) + t                       # -> base_footprint

        # Floor candidates: forward of the depth minimum, within useful range,
        # and near the height the floor is expected at. The z window is wide
        # enough to catch a badly-misplaced floor but tight enough to exclude
        # walls and table tops.
        m = ((pts[:, 2] > zlo) & (pts[:, 2] < zhi) &
             (pts[:, 0] > xlo) & (pts[:, 0] < xhi))
        cand = pts[m]
        if len(cand) < 400:
            rospy.logwarn("frame %d: only %d floor candidates — skipping", i, len(cand))
            continue

        nrm, d, inl = ransac_plane(cand)
        if nrm is None:
            continue

        # Angles the model is missing. A correct model puts the normal at
        # (0,0,1); these are the rotations that would take it there.
        pitch = math.degrees(math.atan2(nrm[0], nrm[2]))   # about y, forward tilt
        roll = math.degrees(math.atan2(-nrm[1], nrm[2]))   # about x, side tilt
        z_at = lambda x: -(d + nrm[0] * x) / nrm[2]
        results.append((pitch, roll, z_at(0.0), z_at(2.0), z_at(4.0),
                        int(inl.sum()), len(cand)))
        rospy.sleep(0.4)

    if not results:
        print("  NO RESULT — no frame had enough floor points. Is the camera "
              "pointing at open floor, beyond %.1f m?" % xlo)
        return

    a = np.array([r[:5] for r in results])
    med = np.median(a, axis=0)
    spread = a.max(axis=0) - a.min(axis=0)

    print("  frames used: %d   (inliers/candidates: %s)"
          % (len(results), ", ".join("%d/%d" % (r[5], r[6]) for r in results)))
    print()
    print("  UNMODELLED PITCH : %+.2f deg   (spread across frames %.2f)" % (med[0], spread[0]))
    print("  UNMODELLED ROLL  : %+.2f deg   (spread across frames %.2f)" % (med[1], spread[1]))
    print()
    print("  fitted floor height at   0 m : %+.3f m   (should be 0.000)" % med[2])
    print("                           2 m : %+.3f m" % med[3])
    print("                           4 m : %+.3f m" % med[4])
    print()
    print("  => the floor %s %.0f mm over 4 m of range"
          % ("RISES" if med[4] > med[2] else "FALLS", abs(med[4] - med[2]) * 1000))
    print()
    print("  If pitch is within about +/-0.3 deg, treat the camera as level: that")
    print("  is the noise floor of a stereo plane fit at this range, and it is")
    print("  4 mm at 1 m. Larger than that, set cam_pitch to the NEGATIVE of the")
    print("  value above in zedx_front.launch and re-run this to confirm it lands")
    print("  near zero.")


if __name__ == "__main__":
    main()
