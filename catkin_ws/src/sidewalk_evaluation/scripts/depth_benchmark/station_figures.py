#!/usr/bin/env python3
"""station_figures.py - capture everything a station needs for the report.

WHY IT SAVES DATA AND NOT JUST PICTURES

    The subproject's rule is that publication figures REGENERATE from scripts
    (ENGINEERING_NOTES.md section 5) - no hand-edited image ever enters the draft. So this
    saves the raw arrays in one .npz beside every quick-look PNG. The PNGs are
    for looking at now; the .npz is what a figure script will read later, in the
    project's own colours, long after the frame has been moved.

WHY IT MUST RUN WHILE THE FRAME IS AT THIS DISTANCE

    Some of this is recoverable later and some is not. Anything derived from raw
    stereo - the images, and depth in any mode - can be regenerated from a
    lossless SVO. But the DETECTOR'S OWN OUTPUT at this station (which points it
    accepted, where it put the spheres, what was inside the search box) exists
    only while the run is happening, and the LiDAR clouds are on a different
    machine that is not recording raw. Those are captured here.

    It only subscribes. It does not touch the camera's settings, does not
    restart anything, and does not run RANSAC - so it is safe to run while a
    station is recording, which is exactly when it is useful.

  usage, on the JETSON (camera, roi_viz and the detector running):
    rosrun sidewalk_evaluation station_figures.py _dist:=4.0 _tag:=NEURAL
"""
from __future__ import print_function

import os
import sys
import time

import numpy as np

_DT = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}


def cloud_xyz(msg):
    names = {f.name: f for f in msg.fields}
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
    out = []
    for k in ('x', 'y', 'z'):
        f = names[k]
        w = np.dtype(_DT[f.datatype]).itemsize
        col = raw[:, f.offset:f.offset + w]
        out.append(np.ascontiguousarray(col).view(_DT[f.datatype]).ravel().astype(np.float32))
    return np.column_stack(out)


def img_array(msg):
    """Decode an Image message without cv_bridge (which drags in a different
    OpenCV and has bitten this project before)."""
    enc = msg.encoding
    if enc in ('32FC1',):
        a = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
    elif enc in ('16UC1', 'mono16'):
        a = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
    elif enc in ('mono8', '8UC1'):
        a = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
    else:
        n = len(msg.data) // (msg.height * msg.width)
        a = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, n)
        if n >= 3:
            a = a[:, :, :3][:, :, ::-1]      # BGRA -> RGB
    return np.array(a)


def main():
    import rospy
    from sensor_msgs.msg import Image, PointCloud2, CameraInfo
    from stereo_msgs.msg import DisparityImage
    from visualization_msgs.msg import MarkerArray

    rospy.init_node('station_figures', anonymous=True)
    g = rospy.get_param
    dist = float(g('~dist', 0.0))
    tag = g('~tag', 'run')
    outdir = g('~out', '~/station_figures')
    if dist <= 0:
        print('give the station distance: _dist:=4.0')
        return 2
    stem = 'w3_%.1fm_%s' % (dist, tag)
    d = os.path.join(outdir, stem)
    if not os.path.isdir(d):
        os.makedirs(d)

    got = {}

    def grab(key, conv):
        def cb(m):
            if key not in got:
                try:
                    got[key] = conv(m)
                except Exception as e:
                    got[key] = ('ERROR', str(e))
        return cb

    subs = [
        ('left', '/zedx_front/zed_node/left/image_rect_color', Image, img_array),
        ('right', '/zedx_front/zed_node/right/image_rect_color', Image, img_array),
        ('depth', '/zedx_front/zed_node/depth/depth_registered', Image, img_array),
        ('conf', '/zedx_front/zed_node/confidence/confidence_map', Image, img_array),
        ('cloud', '/zedx_front/zed_node/point_cloud/cloud_registered', PointCloud2, cloud_xyz),
        ('inside', '/roi_viz_cam/inside', PointCloud2, cloud_xyz),
    ]
    for key, topic, typ, conv in subs:
        rospy.Subscriber(topic, typ, grab(key, conv), queue_size=1, buff_size=2 ** 25)

    rospy.Subscriber('/zedx_front/zed_node/disparity/disparity_image', DisparityImage,
                     grab('disp', lambda m: img_array(m.image)), queue_size=1,
                     buff_size=2 ** 25)
    rospy.Subscriber('/zedx_front/zed_node/left/camera_info', CameraInfo,
                     grab('K', lambda m: np.array(m.P).reshape(3, 4)), queue_size=1)
    rospy.Subscriber('/sphere_centroid/markers', MarkerArray,
                     grab('spheres', lambda ma: np.array(
                         [[mk.pose.position.x, mk.pose.position.y, mk.pose.position.z]
                          for mk in ma.markers if mk.ns == 'spheres'])), queue_size=5)

    want = ['left', 'right', 'depth', 'conf', 'cloud', 'inside', 'disp', 'K']
    t0 = rospy.get_time()
    # The detector only publishes spheres when it ACCEPTS a frame, which can take
    # a while, so wait longer for it - but never block on it forever, because at a
    # station where the detector fails it will never come and the rest of the
    # capture is exactly what documents that failure.
    while not rospy.is_shutdown() and rospy.get_time() - t0 < 180:
        if all(k in got for k in want) and ('spheres' in got or rospy.get_time() - t0 > 120):
            break
        rospy.sleep(0.2)

    print('captured: %s' % ', '.join(sorted(got)))
    missing = [k for k in want if k not in got]
    if missing:
        print('MISSING (recorded as missing, not faked): %s' % ', '.join(missing))
    if 'spheres' not in got:
        print('NO SPHERES - the detector accepted no frame in the capture window. '
              'That is a result, not an error, and the rest of the capture documents it.')

    np.savez_compressed(os.path.join(d, stem + '_raw.npz'),
                        **{k: v for k, v in got.items()
                           if isinstance(v, np.ndarray)})
    print('raw arrays -> %s_raw.npz' % stem)

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    def save(fig, name):
        p = os.path.join(d, stem + '_' + name + '.png')
        fig.savefig(p, dpi=130, bbox_inches='tight', facecolor='white')
        plt.close(fig)
        print('  %s' % os.path.basename(p))

    if 'left' in got:
        f, a = plt.subplots(figsize=(9, 5.6))
        a.imshow(got['left'])
        a.set_title('%s  left rectified image' % stem)
        a.axis('off')
        save(f, 'left')

    if 'right' in got:
        f, a = plt.subplots(figsize=(9, 5.6))
        a.imshow(got['right'])
        a.set_title('%s  right rectified image' % stem)
        a.axis('off')
        save(f, 'right')

    if 'depth' in got and isinstance(got['depth'], np.ndarray):
        z = got['depth'].astype(float)
        finite = np.isfinite(z) & (z > 0)
        lo, hi = (np.percentile(z[finite], 1), np.percentile(z[finite], 99)) \
            if finite.any() else (0, 1)
        f, a = plt.subplots(figsize=(9.6, 5.6))
        im = a.imshow(np.where(finite, z, np.nan), cmap='viridis', vmin=lo, vmax=hi)
        a.set_title('%s  depth, full range   %.1f %% of pixels have a value'
                    % (stem, 100.0 * finite.mean()))
        a.axis('off')
        f.colorbar(im, ax=a, shrink=0.82, label='metres')
        save(f, 'depth')

        # A second panel scaled to the TARGET, not the corridor. The full-range
        # version is dominated by a background ten metres further away, which
        # squeezes the whole target into a couple of colours - useless for
        # seeing the balls' own shape, which is the thing being measured.
        f, a = plt.subplots(figsize=(9.6, 5.6))
        im = a.imshow(np.where(finite, z, np.nan), cmap='viridis',
                      vmin=dist - 0.6, vmax=dist + 0.6)
        a.set_title('%s  depth, scaled to the target (%.1f m +/- 0.6 m)' % (stem, dist))
        a.axis('off')
        f.colorbar(im, ax=a, shrink=0.82, label='metres')
        save(f, 'depth_target')

        f, a = plt.subplots(figsize=(9, 5.6))
        a.imshow(finite, cmap='gray')
        a.set_title('%s  where the camera HAS depth (white) and has none (black)' % stem)
        a.axis('off')
        save(f, 'depth_fill')

    if 'conf' in got and isinstance(got['conf'], np.ndarray):
        c = got['conf'].astype(float)
        f, a = plt.subplots(figsize=(9.6, 5.6))
        im = a.imshow(c, cmap='magma')
        a.set_title('%s  confidence map (lower = the camera trusts it more)' % stem)
        a.axis('off')
        f.colorbar(im, ax=a, shrink=0.82)
        save(f, 'confidence')

    if 'disp' in got and isinstance(got['disp'], np.ndarray):
        dp = got['disp'].astype(float)
        ok = np.isfinite(dp) & (dp != 0)
        f, a = plt.subplots(figsize=(9.6, 5.6))
        im = a.imshow(np.where(ok, dp, np.nan), cmap='plasma')
        a.set_title('%s  disparity (pixels of shift between the two lenses)' % stem)
        a.axis('off')
        f.colorbar(im, ax=a, shrink=0.82, label='px')
        save(f, 'disparity')

    def cloud_fig(P, name, title, spheres=None, cap=60000):
        if P is None or not len(P):
            return
        Q = P[np.isfinite(P).all(axis=1)]
        if len(Q) > cap:
            Q = Q[np.random.choice(len(Q), cap, replace=False)]
        f = plt.figure(figsize=(14, 5.0))
        views = [(18, -72, 'oblique'), (0, -90, 'from the side'), (89, -90, 'from above')]
        for i, (el, az, vn) in enumerate(views):
            a = f.add_subplot(1, 3, i + 1, projection='3d')
            a.scatter(Q[:, 0], Q[:, 1], Q[:, 2], s=0.25, c=Q[:, 2],
                      cmap='viridis', linewidths=0)
            if spheres is not None and len(spheres):
                u = np.linspace(0, 2 * np.pi, 26)
                v = np.linspace(0, np.pi, 14)
                for c in spheres:
                    a.plot_wireframe(
                        c[0] + 0.20 * np.outer(np.cos(u), np.sin(v)),
                        c[1] + 0.20 * np.outer(np.sin(u), np.sin(v)),
                        c[2] + 0.20 * np.outer(np.ones_like(u), np.cos(v)),
                        color='crimson', linewidth=0.4, alpha=0.85)
            # EQUAL ASPECT, or a 0.20 m sphere draws as an ellipsoid and the
            # figure misrepresents the very geometry it exists to show.
            R = np.vstack([Q] + ([spheres] if spheres is not None and len(spheres) else []))
            ctr = (R.max(axis=0) + R.min(axis=0)) / 2.0
            half = float((R.max(axis=0) - R.min(axis=0)).max()) / 2.0 + 0.15
            a.set_xlim(ctr[0] - half, ctr[0] + half)
            a.set_ylim(ctr[1] - half, ctr[1] + half)
            a.set_zlim(ctr[2] - half, ctr[2] + half)
            try:
                a.set_box_aspect((1, 1, 1))
            except Exception:
                pass
            a.view_init(elev=el, azim=az)
            a.set_xlabel('x fwd (m)', fontsize=7, labelpad=-4)
            a.set_ylabel('y left (m)', fontsize=7, labelpad=-4)
            a.set_zlabel('z up (m)', fontsize=7, labelpad=-4)
            a.tick_params(labelsize=5.5, pad=-2)
            a.set_title(vn, fontsize=9)
        f.suptitle(title, fontsize=11)
        save(f, name)

    if 'cloud' in got:
        cloud_fig(got['cloud'], 'cloud_full',
                  '%s  the whole point cloud the camera produced' % stem)
    if 'inside' in got:
        cloud_fig(got['inside'], 'cloud_box',
                  '%s  points inside the search box, with the spheres the detector fitted'
                  % stem, spheres=got.get('spheres'), cap=40000)

    with open(os.path.join(d, stem + '_capture.txt'), 'w') as fh:
        fh.write('station %s\ncaptured %s UTC\n' % (stem, time.strftime('%F %T', time.gmtime())))
        fh.write('present: %s\n' % ', '.join(sorted(k for k in got)))
        fh.write('missing: %s\n' % (', '.join(missing) if missing else 'none'))
        if 'spheres' in got and isinstance(got['spheres'], np.ndarray):
            fh.write('detector sphere centres:\n')
            for c in got['spheres']:
                fh.write('  %8.4f %8.4f %8.4f\n' % tuple(c))
        else:
            fh.write('detector sphere centres: NONE - no frame accepted in the window\n')
        if 'cloud' in got:
            fh.write('cloud points: %d\n' % len(got['cloud']))
        if 'inside' in got:
            fh.write('points inside the search box: %d\n' % len(got['inside']))
    print('\nall figures -> %s' % d)
    return 0


if __name__ == '__main__':
    sys.exit(main())
