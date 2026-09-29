#!/usr/bin/env python3
"""plane_replay_n.py - re-derive Scenario 2's plane measurement from a recording,
in any depth mode, in parallel.

WHY THIS EXISTS. Scenario 2 was recorded in NEURAL at every station, because a live
run can only hold one depth mode at a time. The recording lets the SAME photons be
re-measured in every mode, so the modes differ and nothing else does. That is the
whole premise of the comparison, and it is what the station folder's own
WHERE_IS_THE_RECORDING.txt says the recording is kept for.

THE INSTRUMENT IS NICOLAS'S, IMPORTED UNMODIFIED.
`extents()` and `plane_residual()` are module-level pure functions - they take an
(N,3) array of points and return the measurement - so they are imported and called
exactly as his node calls them. Nothing in this file touches them.
Fingerprint: plane_extent.py md5 11753c0ff450cd5a48230e8f988d0fb6.

THE ONE THING RE-IMPLEMENTED, AND HOW IT IS CHECKED.
His `cb()` parses a PointCloud2, drops non-finite points, and crops to the box
before calling those two functions. Offline there is no PointCloud2, so the parse is
replaced by the SDK's own XYZ measure and the crop is re-implemented - eight lines.
A re-implementation is exactly where a silent disagreement would hide, so it is not
trusted: replaying in the mode the live run used must REPRODUCE the live CSV. That
check can fail, and until it passes no other mode's numbers mean anything.

NO RANDOMNESS, UNLIKE THE SPHERE REPLAY.
svo_replay_n.py must seed a generator per frame because RANSAC draws random samples,
and a result that depends on the worker count is not reproducible. `plane_residual`
fits by least squares and rejects by sigma - it is deterministic. So slicing cannot
change the answer here, and there is nothing to seed.

SLICE BOUNDARIES ARE FREE HERE. This station ran with depth_stabilization = 0
(environment.txt, read back from the live node), so there is no temporal filter to
restart at a slice boundary. With stabilization on, a worker starting mid-stream
would carry a different history from the live run; with it off, it cannot.
"""

import argparse
import multiprocessing as mp
import os
import sys

import numpy as np

# Nicolas's node, and therefore his measurement. Imported, never edited.
NICOLAS_DIR = os.environ.get(
    'PLANE_EXTENT_DIR',
    '~/catkin_ws/src/sidewalk_evaluation/scripts')

# His CSV header, verbatim. The column order is load-bearing: summary() and every
# downstream reader index into it positionally.
HEADER = ('n,ext_x,ext_y,ext_z,npts,p_ext_x,p_ext_y,p_ext_z,resid,tilt_deg,'
          'skew,resid_c,ext_x_c,n_rej,stamp')


def roi_bounds(roi_x, roi_y, roi_z, sx, sy, sz, sensor_h):
    """centre+size (roi_z above the floor) -> six bounds in the sensor frame.

    This is plane_extent.read_roi() transcribed. It is the only arithmetic of his
    that is repeated here rather than imported, because his version reads rosparam.
    Kept in one function so the reproduction check exercises it directly.
    """
    cz = roi_z - sensor_h                 # floor-relative -> sensor frame
    return np.array([roi_x - sx / 2.0, roi_y - sy / 2.0, cz - sz / 2.0]), \
           np.array([roi_x + sx / 2.0, roi_y + sy / 2.0, cz + sz / 2.0])


def crop(p, lo, hi):
    """His crop: drop non-finite, then keep points inside the closed box.

    Closed on both sides (>= and <=), matching cb() exactly. An open bound would
    move every extent by however many points sit on the face of the box.
    """
    p = p[np.isfinite(p).all(axis=1)]
    m = ((p[:, 0] >= lo[0]) & (p[:, 0] <= hi[0]) &
         (p[:, 1] >= lo[1]) & (p[:, 1] <= hi[1]) &
         (p[:, 2] >= lo[2]) & (p[:, 2] <= hi[2]))
    return p[m].astype(np.float64)


def plan(svo, workers, total):
    per = max(1, total // max(1, workers))
    jobs, shard, start = [], 0, 0
    while start < total:
        end = min(total, start + per)
        jobs.append([svo, start, end, None, None, None, shard, None])
        shard += 1
        start = end
    return jobs


def worker(job):
    svo, first, last, mode, box, min_pts, shard, progdir = job
    nsig = box['nsig']
    import pyzed.sl as sl
    sys.path.insert(0, NICOLAS_DIR)
    from plane_extent import extents, plane_residual      # HIS, unmodified

    lo, hi = roi_bounds(box['roi_x'], box['roi_y'], box['roi_z'],
                        box['sx'], box['sy'], box['sz'], box['sensor_h'])

    done = measured = empty = 0
    span = max(1, last - first)
    pfile = os.path.join(progdir, 'shard_%04d' % shard) if progdir else None

    def flush():
        if not pfile:
            return
        tmp = pfile + '.tmp'
        with open(tmp, 'w') as fh:
            # Same six fields the sphere replay writes, so one dashboard reads both.
            # For a plane, `empty` is the box coming back with too few points - a
            # DEPTH failure - and the fifth slot is 0 because there is no fit that
            # can fail: least squares always returns a plane.
            fh.write('%d %d %d %d %d %d\n'
                     % (shard, done, span, measured, empty, 0))
        os.replace(tmp, pfile)

    init = sl.InitParameters()
    init.set_from_svo_file(svo)
    init.depth_mode = getattr(sl.DEPTH_MODE, mode)
    init.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
    init.coordinate_units = sl.UNIT.METER
    init.svo_real_time_mode = False
    init.depth_stabilization = 0           # this station's live setting
    init.camera_disable_self_calib = True  # general/self_calib = false, live
    init.depth_minimum_distance = 1.0
    init.depth_maximum_distance = 35.0

    cam = sl.Camera()
    st = cam.open(init)
    if st != sl.ERROR_CODE.SUCCESS:
        # NEVER return an empty list here without saying why. Five of ten workers
        # once landed on a half-built depth engine, contributed no rows and no
        # message, and the run reported success having skipped 342 frames.
        return shard, [], 0, 'CAMERA OPEN FAILED: %s' % st

    rt = sl.RuntimeParameters()
    rt.confidence_threshold = 50           # common.yaml depth_confidence
    rt.texture_confidence_threshold = 100  # common.yaml depth_texture_conf

    cloud = sl.Mat()
    rows = []
    cam.set_svo_position(first)
    stopped = None
    for _ in range(first, last):
        g = cam.grab(rt)
        if g != sl.ERROR_CODE.SUCCESS:
            # END OF FILE AND A CORRUPT FRAME ARE NOT THE SAME EVENT. A bare
            # `break` reported both as a clean finish, so a shard that died
            # mid-slice returned success and its missing frames vanished. Only
            # the end of the recording ends a slice quietly.
            if g != sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
                stopped = str(g)
            break
        cam.retrieve_measure(cloud, sl.MEASURE.XYZ)
        # NOT get_seconds(): pyzed's Timestamp.get_seconds() is INTEGER division
        # by 1e9, so 1789379259306349000 ns comes back as 1789379259, and the
        # '%.6f' field below dresses it in six fabricated decimals under a header
        # copied from his node - whose cb() writes microseconds. At 15 Hz that
        # collapses fifteen frames onto one key. The sphere replay never did this.
        stamp = cam.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds() / 1e9
        q = crop(cloud.get_data()[:, :, :3].reshape(-1, 3), lo, hi)
        done += 1
        if done % 5 == 0:
            flush()
        if len(q) < min_pts:
            empty += 1
            continue
        raw, rob = extents(q)
        resid, tilt, skew, resid_c, ext_x_c, n_rej = plane_residual(q, nsig)
        measured += 1
        rows.append((raw[0], raw[1], raw[2], len(q), rob[0], rob[1], rob[2],
                     resid, tilt, skew, resid_c, ext_x_c, n_rej, stamp))
    flush()
    cam.close()
    if stopped is not None:
        return shard, rows, done, ('GRAB FAILED after %d of %d frames: %s'
                                   % (done, span, stopped))
    if done < span:
        return shard, rows, done, ('SHORT: %d of %d frames, with no error from the SDK'
                                   % (done, span))
    return shard, rows, done, None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--svo', required=True)
    ap.add_argument('--mode', default='NEURAL')
    ap.add_argument('--roi-x', type=float, required=True)
    ap.add_argument('--roi-y', type=float, required=True)
    ap.add_argument('--roi-z', type=float, required=True)
    ap.add_argument('--roi-sx', type=float, required=True)
    ap.add_argument('--roi-sy', type=float, required=True)
    ap.add_argument('--roi-sz', type=float, required=True)
    ap.add_argument('--sensor-h', type=float, required=True)
    ap.add_argument('--min-points', type=int, default=30)
    ap.add_argument('--nsig', type=float, default=3.0)
    ap.add_argument('--workers', type=int, default=4)
    ap.add_argument('--limit', type=int, default=0,
                    help='process at most this many frames, spread across the '
                         'slices - for proving the harness before a long run')
    ap.add_argument('--progress-dir', default='')
    ap.add_argument('--out', required=True)
    a = ap.parse_args()

    import pyzed.sl as sl
    ip = sl.InitParameters()
    ip.set_from_svo_file(a.svo)
    c = sl.Camera()
    if c.open(ip) != sl.ERROR_CODE.SUCCESS:
        print('cannot open %s' % a.svo)
        return 2
    total = c.get_svo_number_of_frames()
    c.close()
    if total <= 0:
        print('the recording reports %d frames' % total)
        return 2

    jobs = plan(a.svo, a.workers, total)
    if a.limit:
        # Spread the budget ACROSS the slices. Taking it from the front would run
        # exactly one worker while claiming the parallel path had been exercised.
        per = max(1, a.limit // max(1, len(jobs)))
        keep, seen = [], 0
        for j in jobs:
            if seen >= a.limit:
                break
            take = min(per, a.limit - seen, j[2] - j[1])
            if take <= 0:
                continue
            j[2] = j[1] + take
            seen += take
            keep.append(j)
        jobs = keep
        print('  LIMITED to %d frames across %d slices, for validation' % (seen, len(jobs)))

    box = dict(roi_x=a.roi_x, roi_y=a.roi_y, roi_z=a.roi_z,
               sx=a.roi_sx, sy=a.roi_sy, sz=a.roi_sz,
               sensor_h=a.sensor_h, nsig=a.nsig)
    lo, hi = roi_bounds(a.roi_x, a.roi_y, a.roi_z, a.roi_sx, a.roi_sy, a.roi_sz, a.sensor_h)
    print('  box in the sensor frame: x[%.3f %.3f] y[%.3f %.3f] z[%.3f %.3f]'
          % (lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]))
    print('  %d frames -> %d slices on %d workers, mode %s'
          % (total, len(jobs), a.workers, a.mode))

    if a.progress_dir:
        os.makedirs(a.progress_dir, exist_ok=True)
    for j in jobs:
        j[3], j[4], j[5], j[7] = a.mode, box, a.min_points, a.progress_dir or None

    # Build the depth engine once, alone, before anybody races for it.
    print('  warming the %s depth engine (single process, so nothing races)' % a.mode)
    sys.stdout.flush()
    warm = list(jobs[0])
    warm[2] = warm[1] + 1
    sh, rws, dn, err = worker(tuple(warm))
    if err:
        print('  ABORTING: %s. Nothing was run.' % err)
        return 3

    planned = sum(j[2] - j[1] for j in jobs)
    ctx = mp.get_context('spawn')
    out_rows, replayed, failures = [], 0, []
    with ctx.Pool(a.workers) as pool:
        for sh, rws, dn, err in pool.imap_unordered(worker, [tuple(j) for j in jobs]):
            if err:
                failures.append('shard %d: %s' % (sh, err))
                continue
            out_rows += rws
            replayed += dn
    if failures:
        # A mode that did not see the same frames as the others cannot be compared
        # with them. Say so loudly rather than writing a short file that looks whole.
        print('  %d SLICE(S) FAILED:' % len(failures))
        for f in failures:
            print('    ' + f)

    out_rows.sort(key=lambda r: r[-1])          # by timestamp, so n is time order
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    with open(a.out, 'w') as fh:
        fh.write(HEADER + '\n')
        for i, r in enumerate(out_rows, 1):
            fh.write('%d,%.5f,%.5f,%.5f,%d,%.5f,%.5f,%.5f,%.5f,%.3f,'
                     '%.3f,%.5f,%.5f,%d,%.6f\n'
                     % (i, r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8],
                        r[9], r[10], r[11], r[12], r[13]))
    print('  wrote %d rows of %d frames ACTUALLY REPLAYED -> %s'
          % (len(out_rows), replayed, a.out))

    # THE RUN MUST BE ABLE TO FAIL ON TRUNCATION. The sphere replay reconciles
    # these two numbers and this file did not, so a slice that stopped early and
    # a complete pass printed the same shape of line and both exited 0. A mode
    # that did not see the same frames as the others cannot be compared with them.
    with open(a.out + '.provenance', 'w') as fh:
        fh.write('mode=%s\nframes_in_file=%d\nframes_planned=%d\n'
                 'frames_replayed=%d\nrows=%d\nlimited=%s\nslices=%d\nworkers=%d\n'
                 % (a.mode, total, planned, replayed, len(out_rows),
                    'yes' if a.limit else 'no', len(jobs), a.workers))
    if replayed != planned:
        print('  REFUSING TO REPORT THIS AS COMPLETE: %d frames were planned but %d '
              'were replayed - %d never ran.' % (planned, replayed, planned - replayed))
        return 1
    if not a.limit and planned != total:
        print('  REFUSING TO REPORT THIS AS COMPLETE: the file holds %d frames but only '
              '%d were planned.' % (total, planned))
        return 1
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
