#!/usr/bin/env python3
"""svo_station_figures.py - rebuild a station's camera figures from its recording.

WHY THIS EXISTS

    Stations are expensive to revisit: the frame is moved by hand and a station
    takes over an hour. So when a station is asked for figures AFTER the frame
    has moved on, the only way back is the raw recording.

    Lossless SVO files hold the raw stereo pair, which is everything the camera
    ever had. Images, depth in ANY depth mode, confidence, disparity and the
    point cloud all follow from it. So for a station with a banked SVO, nothing
    about the camera is lost - which is exactly why the SVOs are worth their
    size.

    THE LASERS HAVE NO EQUIVALENT. Their clouds are not recorded raw, so a
    station whose laser clouds were not captured while the frame stood there
    cannot be recovered at all. That asymmetry is worth knowing before deciding
    a station is finished.

WHAT IT CANNOT REBUILD

    The detector's live acceptance history and the exact frames it accepted -
    those are in the CSV, not here. This runs the frozen detector on the
    replayed cloud to place the spheres, which reproduces the geometry but is a
    fresh RANSAC, not a replay of the original one.

  usage, on the JETSON:
    rosrun sidewalk_evaluation svo_station_figures.py \\
        --svo ~/zedx_vs_lidar_data/svo/w3_3m.svo2 --dist 3.0 --mode NEURAL
"""
from __future__ import print_function

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--svo', required=True)
    ap.add_argument('--dist', type=float, required=True)
    ap.add_argument('--mode', default='NEURAL')
    ap.add_argument('--frame', type=int, default=-1, help='-1 = the middle frame')
    ap.add_argument('--sensor-h', type=float, default=0.6972)
    ap.add_argument('--out', default='~/station_figures')
    a = ap.parse_args()

    import pyzed.sl as sl

    init = sl.InitParameters()
    init.set_from_svo_file(a.svo)
    init.depth_mode = getattr(sl.DEPTH_MODE, a.mode)
    init.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
    init.coordinate_units = sl.UNIT.METER
    init.svo_real_time_mode = False
    init.depth_stabilization = 1
    cam = sl.Camera()
    if cam.open(init) != sl.ERROR_CODE.SUCCESS:
        print('could not open %s' % a.svo)
        return 1

    total = cam.get_svo_number_of_frames()
    idx = total // 2 if a.frame < 0 else a.frame
    cam.set_svo_position(min(idx, total - 1))
    rt = sl.RuntimeParameters()
    rt.confidence_threshold = 50           # what the live pipeline records with
    rt.texture_confidence_threshold = 100
    if cam.grab(rt) != sl.ERROR_CODE.SUCCESS:
        print('could not grab frame %d of %d' % (idx, total))
        return 1

    left, depth, conf, cloud = sl.Mat(), sl.Mat(), sl.Mat(), sl.Mat()
    cam.retrieve_image(left, sl.VIEW.LEFT)
    cam.retrieve_measure(depth, sl.MEASURE.DEPTH)
    cam.retrieve_measure(conf, sl.MEASURE.CONFIDENCE)
    cam.retrieve_measure(cloud, sl.MEASURE.XYZ)
    calib = cam.get_camera_information().camera_configuration.calibration_parameters
    fx = calib.left_cam.fx
    L = left.get_data()[:, :, :3][:, :, ::-1].copy()
    Z = depth.get_data().copy()
    Cf = conf.get_data().copy()
    P = cloud.get_data()[:, :, :3].reshape(-1, 3)
    P = P[np.isfinite(P).all(axis=1)]
    cam.close()

    stem = 'w3_%.1fm_%s_fromSVO' % (a.dist, a.mode)
    d = os.path.join(a.out, stem)
    if not os.path.isdir(d):
        os.makedirs(d)

    # the same search box the live pipeline uses
    h = a.sensor_h
    box = P[(np.abs(P[:, 0] - a.dist) < 0.6) & (np.abs(P[:, 1]) < 0.95)
            & (P[:, 2] > -h + 0.55) & (P[:, 2] < -h + 2.35)]

    spheres = None
    try:
        from sphere_centroid import detect_frame
        rng = np.random.default_rng(0)
        res = detect_frame(box, 0.20, 0.07, 0.03, 1500, 10, 1.10, 1.10, 0.06,
                           7, rng, 0.08)
        if res is not None:
            spheres = np.asarray(res[0])
            print('detector found the frame: %d sphere centres' % len(spheres))
        else:
            print('DETECTOR FOUND NOTHING on this frame - which at some stations '
                  'is the result, not a failure of this script')
    except Exception as e:
        print('could not run the frozen detector here (%s) - figures still made' % e)

    np.savez_compressed(os.path.join(d, stem + '_raw.npz'),
                        left=L, depth=Z, conf=Cf, cloud=P.astype(np.float32),
                        box=box.astype(np.float32), fx=np.float32(fx),
                        **({'spheres': spheres} if spheres is not None else {}))

    import matplotlib
    matplotlib.use('Agg')
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    import matplotlib.pyplot as plt

    def save(f, n):
        p = os.path.join(d, stem + '_' + n + '.png')
        f.savefig(p, dpi=130, bbox_inches='tight', facecolor='white')
        plt.close(f)
        print('  %s' % os.path.basename(p))

    f, ax = plt.subplots(figsize=(9, 5.6))
    ax.imshow(L)
    ax.set_title('%s  left image, frame %d of %d' % (stem, idx, total))
    ax.axis('off')
    save(f, 'left')

    fin = np.isfinite(Z) & (Z > 0)
    for name, lo, hi, ttl in [
            ('depth', np.percentile(Z[fin], 1) if fin.any() else 0,
             np.percentile(Z[fin], 99) if fin.any() else 1, 'full range'),
            ('depth_target', a.dist - 0.6, a.dist + 0.6,
             'scaled to the target (%.1f m +/- 0.6 m)' % a.dist)]:
        f, ax = plt.subplots(figsize=(9.6, 5.6))
        im = ax.imshow(np.where(fin, Z, np.nan), cmap='viridis', vmin=lo, vmax=hi)
        ax.set_title('%s  depth, %s   %.1f %% of pixels have a value'
                     % (stem, ttl, 100.0 * fin.mean()))
        ax.axis('off')
        f.colorbar(im, ax=ax, shrink=0.82, label='metres')
        save(f, name)

    f, ax = plt.subplots(figsize=(9, 5.6))
    ax.imshow(fin, cmap='gray')
    ax.set_title('%s  where the camera HAS depth (white)' % stem)
    ax.axis('off')
    save(f, 'depth_fill')

    f, ax = plt.subplots(figsize=(9.6, 5.6))
    im = ax.imshow(Cf, cmap='magma')
    ax.set_title('%s  confidence map' % stem)
    ax.axis('off')
    f.colorbar(im, ax=ax, shrink=0.82)
    save(f, 'confidence')

    def cloud_fig(Q, name, title, sph=None, cap=60000):
        if not len(Q):
            return
        R = Q if len(Q) <= cap else Q[np.random.choice(len(Q), cap, replace=False)]
        f = plt.figure(figsize=(14, 5.0))
        for i, (el, az, vn) in enumerate([(18, -72, 'oblique'), (0, -90, 'from the side'),
                                          (89, -90, 'from above')]):
            ax = f.add_subplot(1, 3, i + 1, projection='3d')
            ax.scatter(R[:, 0], R[:, 1], R[:, 2], s=0.3, c=R[:, 2], cmap='viridis',
                       linewidths=0)
            if sph is not None and len(sph):
                u = np.linspace(0, 2 * np.pi, 24)
                v = np.linspace(0, np.pi, 12)
                for c in sph:
                    ax.plot_wireframe(
                        c[0] + 0.20 * np.outer(np.cos(u), np.sin(v)),
                        c[1] + 0.20 * np.outer(np.sin(u), np.sin(v)),
                        c[2] + 0.20 * np.outer(np.ones_like(u), np.cos(v)),
                        color='crimson', linewidth=0.4, alpha=0.85)
            A = np.vstack([R] + ([sph] if sph is not None and len(sph) else []))
            ctr = (A.max(axis=0) + A.min(axis=0)) / 2.0
            half = float((A.max(axis=0) - A.min(axis=0)).max()) / 2.0 + 0.15
            ax.set_xlim(ctr[0] - half, ctr[0] + half)
            ax.set_ylim(ctr[1] - half, ctr[1] + half)
            ax.set_zlim(ctr[2] - half, ctr[2] + half)
            try:
                ax.set_box_aspect((1, 1, 1))
            except Exception:
                pass
            ax.view_init(elev=el, azim=az)
            ax.set_xlabel('x fwd (m)', fontsize=7, labelpad=-4)
            ax.set_ylabel('y left (m)', fontsize=7, labelpad=-4)
            ax.set_zlabel('z up (m)', fontsize=7, labelpad=-4)
            ax.tick_params(labelsize=5.5, pad=-2)
            ax.set_title(vn, fontsize=9)
        f.suptitle(title, fontsize=11)
        save(f, name)

    cloud_fig(P, 'cloud_full', '%s  the whole point cloud' % stem)
    cloud_fig(box, 'cloud_box',
              '%s  inside the search box%s'
              % (stem, ', with the spheres the detector fitted' if spheres is not None
                 else '  -  NO FRAME FOUND'), sph=spheres, cap=40000)

    with open(os.path.join(d, stem + '_capture.txt'), 'w') as fh:
        fh.write('%s\nrebuilt from %s, frame %d of %d\n' % (stem, a.svo, idx, total))
        fh.write('depth mode %s, fx %.2f read from the recording\n' % (a.mode, fx))
        fh.write('cloud points %d, inside the search box %d\n' % (len(P), len(box)))
        fh.write('detector: %s\n' % ('found the frame' if spheres is not None
                                     else 'FOUND NOTHING'))
        if spheres is not None:
            for c in spheres:
                fh.write('  %8.4f %8.4f %8.4f\n' % tuple(c))
    print('\n-> %s' % d)
    return 0


if __name__ == '__main__':
    sys.exit(main())
