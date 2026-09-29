#!/usr/bin/env python3
"""svo_replay_n.py - replay recordings through the frozen detector, in parallel.

WHY THIS EXISTS

    The live camera manages about one accepted measurement every 110 seconds, so
    n=1000 live is impossible - it would take thirty hours per depth mode. An SVO
    holds the RAW stereo, and depth is recomputed at playback in whatever mode is
    asked for, so one recording session can be replayed five times to give every
    mode its n=1000 without the frame ever being needed again.

    The cost is compute, not camera time: the same RANSAC still runs on ~104,000
    points per frame, about 30 s each. Single-threaded that is over 20 hours per
    mode. The Jetson has 8 cores and the detector is single-threaded, so the
    frames are split across worker processes.

MATCHING THE LIVE RUN EXACTLY

    A replay that computes depth differently from the live run is measuring a
    different instrument, and the comparison is void. Every one of these is read
    from the live configuration, not chosen here:

        camera_disable_self_calib = True    live runs with self_calib:=false, so
                                            the lens geometry must NOT re-roll at
                                            playback either. Without this the SDK
                                            re-estimates it and the replay drifts
                                            away from the run it is meant to match.
        depth_stabilization       = 1       the SDK default and the live setting
        confidence_threshold      = 50      live: depth_confidence
        texture_confidence_thresh = 100     live: depth_texture_conf
        svo_real_time_mode        = False   process every frame, never skip
        coordinate system/units             right-handed Z-up X-forward, metres

    --depth-stabilization 0 switches the temporal filter off. Use it to compare
    depth modes: the filter carries history from frame to frame, and a slice that
    starts mid-file has none, so only with 0 do two replays of one file agree to
    every printed digit. The default, 1, stays the live setting.

    THE FRAMES ARE PROCESSED IN FULL, NOT SAMPLED. svo_mode_compare.py jumps
    through a file to take 40 spread-out frames; that is right for comparing
    modes cheaply and wrong here, because n=1000 needs every frame the recording
    holds.

OUTPUT
    A CSV in the same 14 columns the live recorder writes, so every existing
    analysis script reads it unchanged.
"""
from __future__ import print_function

import argparse
import csv
import multiprocessing as mp
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

DET = dict(radius=0.20, radius_tol=0.07, thresh=0.03, iters=1500, min_inliers=10,
           base=1.10, rise=1.10, tol=0.06, max_candidates=7, z_tol=0.08)
HDR = ['n', 'x', 'y', 'z', 'base', 'leg1', 'leg2',
       'n_bot1', 'n_bot2', 'n_apex', 'tx', 'ty', 'tz', 'stamp']


def worker(job):
    """Process one contiguous slice of one SVO. Returns a list of rows."""
    svo, first, last, mode, roi_x, sensor_h, shard, progdir = job[:8]
    stabilization = job[8] if len(job) > 8 else 1
    import pyzed.sl as sl
    from sphere_centroid import detect_frame, centroid_of

    # LIVE PROGRESS, AND WHY A REJECTION HAPPENED.
    #
    # A slice of ~90 frames takes half an hour, so slice-completion messages are
    # far too coarse to watch. Each worker writes its own one-line counter file
    # every few frames instead, and the dashboard adds them up. Rejections are
    # split into the two ways a frame can fail, because they mean opposite
    # things: 'sparse' is the box coming back nearly empty - depth did not
    # reconstruct the spheres at all - while 'nofit' is a box full of points in
    # which RANSAC could not find three spheres of the right size in the right
    # arrangement. The first is a depth failure, the second a geometry failure.
    done = accepted = sparse = nofit = 0
    span = max(1, last - first)
    pfile = os.path.join(progdir, 'shard_%04d' % shard) if progdir else None

    def flush():
        if not pfile:
            return
        tmp = pfile + '.tmp'
        with open(tmp, 'w') as fh:
            fh.write('%d %d %d %d %d %d\n'
                     % (shard, done, span, accepted, sparse, nofit))
        os.replace(tmp, pfile)          # atomic: the reader never sees a half line

    init = sl.InitParameters()
    init.set_from_svo_file(svo)
    init.depth_mode = getattr(sl.DEPTH_MODE, mode)
    init.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
    init.coordinate_units = sl.UNIT.METER
    init.svo_real_time_mode = False
    init.depth_stabilization = stabilization
    init.camera_disable_self_calib = True      # the live run is pinned; so is this
    init.depth_minimum_distance = 1.0
    init.depth_maximum_distance = 35.0

    cam = sl.Camera()
    st = cam.open(init)
    if st != sl.ERROR_CODE.SUCCESS:
        # THIS USED TO `return shard, []` - INDISTINGUISHABLE FROM "found nothing".
        #
        # depth model's optimized engine did not exist yet, one worker began
        # building it, and the others read the half-written file and got
        # "NEURAL CORRUPTED MODEL". Their slices contributed no rows and no
        # message, so 342 of 935 frames were never replayed at all while the run
        # reported success. A mode that did not see the same frames as the others
        # cannot be compared with them, which is the entire point of the campaign.
        return shard, [], 0, 'CAMERA OPEN FAILED: %s' % st

    rt = sl.RuntimeParameters()
    rt.confidence_threshold = 50
    rt.texture_confidence_threshold = 100

    lo = np.array([roi_x - 0.6, -0.95, -sensor_h + 0.55])
    hi = np.array([roi_x + 0.6, 0.95, -sensor_h + 2.35])

    cloud = sl.Mat()
    rows = []
    cam.set_svo_position(first)
    for idx in range(first, last):
        # RANSAC IS SEEDED PER FRAME, NOT PER WORKER.
        #
        # Each worker used to open its own default_rng(0), so every shard drew
        # the same random sequence against different frames - and changing
        # --workers changed which draws landed on which frame, which changed
        # the answer. A replay whose result depends on how many cores you gave
        # it is not reproducible. Seeding from the absolute frame index makes
        # the output identical for any worker count, and identical between
        # runs. It does NOT reproduce the live recorder's stream, which is one
        # continuous generator over the whole run; nothing offline can, and the
        # comparison is against the live MEAN, not frame by frame.
        rng = np.random.default_rng(idx)
        if cam.grab(rt) != sl.ERROR_CODE.SUCCESS:
            break
        cam.retrieve_measure(cloud, sl.MEASURE.XYZ)
        a = cloud.get_data()[:, :, :3].reshape(-1, 3)
        a = a[np.isfinite(a).all(axis=1)]
        pts = a[np.all((a >= lo) & (a <= hi), axis=1)].astype(np.float64)
        done += 1
        if done % 5 == 0:
            flush()
        if len(pts) < 3 * DET['min_inliers']:
            sparse += 1
            continue
        res = detect_frame(pts, rng=rng, **DET)
        if res is None:
            nofit += 1
            continue
        accepted += 1
        # ORDERING: TAKE WHAT THE DETECTOR GIVES, DO NOT RE-SORT.
        #
        # detect_frame already returns its points in the canonical order
        # [bottom_a, bottom_b, apex], where the apex is the highest z and the
        # bottom PAIR IS ORDERED BY y (sphere_centroid.py:137-139), and it
        # returns the three side lengths in that same order. The live recorder
        # writes those straight out (sphere_centroid.py:262-282).
        #
        # This function used to re-sort all three by z and recompute the sides.
        # base survived that, because it is the distance between the two lower
        # balls either way. leg1/leg2 and n_bot1/n_bot2 did NOT: the bottom pair
        # is level to within z_tol = 0.08 m, so their z order is noise, and the
        # two could exchange places from frame to frame. A replayed leg1 and a
        # live leg1 could therefore describe different balls - silently, with
        # no error and no visible symptom, and any per-ball analysis comparing
        # the two would have been comparing different spheres.
        centers, d = res
        counts = [int((np.abs(np.linalg.norm(pts - sc, axis=1) - DET['radius'])
                       < DET['thresh']).sum()) for sc in centers]
        cen = centroid_of(centers)
        ts = cam.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds() / 1e9
        rows.append([0, cen[0], cen[1], cen[2],
                     float(d[0]), float(d[1]), float(d[2]),
                     counts[0], counts[1], counts[2],
                     float('nan'), float('nan'), float('nan'), ts])
    cam.close()
    flush()
    return shard, rows, done, None


def warm_up(svo, mode, stabilization=1):
    """Open one camera in this depth mode and grab a frame, before any worker
    exists. Returns True if the mode is actually usable on this machine."""
    import pyzed.sl as sl
    init = sl.InitParameters()
    init.set_from_svo_file(svo)
    init.depth_mode = getattr(sl.DEPTH_MODE, mode)
    init.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
    init.coordinate_units = sl.UNIT.METER
    init.svo_real_time_mode = False
    init.depth_stabilization = stabilization
    init.camera_disable_self_calib = True
    cam = sl.Camera()
    if cam.open(init) != sl.ERROR_CODE.SUCCESS:
        return False
    ok = cam.grab(sl.RuntimeParameters()) == sl.ERROR_CODE.SUCCESS
    cam.close()
    return ok


def plan(svos, workers, per_file=False):
    """Split the recordings into `workers` contiguous slices of similar size.

    With per_file=True, EACH RECORDING BECOMES EXACTLY ONE SLICE instead.

    Why that option exists: depth stabilization is temporal, so every slice
    begins with no history and the first frames after a boundary are computed
    differently from the ones after them. One slice per recording is the fewest
    boundaries possible while still running in parallel - and it matches how the
    live camera actually saw each recording, as one continuous run. Comparing a
    normal split against this one measures what the boundaries cost, on identical
    frames.
    """
    import pyzed.sl as sl
    sizes = []
    for s in svos:
        cam = sl.Camera()
        init = sl.InitParameters()
        init.set_from_svo_file(s)
        if cam.open(init) != sl.ERROR_CODE.SUCCESS:
            print('  cannot open %s - skipped' % s)
            continue
        sizes.append((s, cam.get_svo_number_of_frames()))
        cam.close()
    total = sum(n for _, n in sizes)
    if not total:
        return [], 0
    jobs, shard = [], 0
    if per_file:
        for s, n in sizes:
            jobs.append([s, 0, n, None, None, None, shard, None])
            shard += 1
        return jobs, total
    per = max(1, total // workers)
    for s, n in sizes:
        start = 0
        while start < n:
            end = min(n, start + per)
            jobs.append([s, start, end, None, None, None, shard, None])
            shard += 1
            start = end
    return jobs, total


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--svo', nargs='+', required=True)
    ap.add_argument('--mode', default='NEURAL')
    ap.add_argument('--roi-x', type=float, default=4.0)
    ap.add_argument('--sensor-h', type=float, default=0.6972)
    ap.add_argument('--workers', type=int, default=4)
    ap.add_argument('--limit', type=int, default=0,
                    help='process at most this many frames in total. For proving '
                         'the tool works before an 8-hour campaign depends on it - '
                         'a full pass is ~30 s per frame.')
    ap.add_argument('--slice-per-file', action='store_true',
                    help='one slice per recording instead of even slices - the '
                         'fewest temporal-filter restarts possible while still '
                         'parallel. Use it to measure what slice boundaries cost.')
    ap.add_argument('--progress-dir', default='',
                    help='directory for one live counter file per worker, so a '
                         'dashboard can show frames done and why each rejection '
                         'happened while the pass is still running')
    ap.add_argument('--depth-stabilization', type=int, default=1,
                    help='the SDK temporal depth filter: 1 = the live setting (default), '
                         '0 = off, so that two replays of one file agree exactly')
    ap.add_argument('--out', required=True)
    a = ap.parse_args()

    jobs, total = plan(a.svo, a.workers, a.slice_per_file)
    if not jobs:
        print('no readable recordings'); return 2
    if a.limit:
        # SPREAD THE BUDGET ACROSS THE SLICES, NOT DOWN THE FIRST ONE.
        #
        # This loop used to walk the slices taking as much as it could from
        # each and breaking when the budget ran out - so any --limit below
        # total/workers was consumed entirely by slice 0 and exactly ONE worker
        # ran. The comment claimed the parallel path was still exercised. It was
        # not, and the parallel path is the whole thing --limit exists to
        # de-risk before an eight-hour campaign depends on it.
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
        jobs, total = keep, seen
        print('  LIMITED to %d frames across %d workers, for validation'
              % (total, len(jobs)))
    # NOTHING IS DELETED HERE. The caller gives each pass its OWN progress
    # directory, so one pass's counters can never be mistaken for the next
    # pass's, and no stale file has to be cleared away.
    if a.progress_dir:
        os.makedirs(a.progress_dir, exist_ok=True)
    for j in jobs:
        j[3], j[4], j[5] = a.mode, a.roi_x, a.sensor_h
        j[7] = a.progress_dir or None
        j.append(a.depth_stabilization)
    print('  %d frames across %d files -> %d slices on %d workers'
          % (total, len(a.svo), len(jobs), a.workers))

    # BUILD THE DEPTH ENGINE FIRST, SINGLE-THREADED.
    #
    # The SDK compiles an optimized engine for a depth mode on first use and
    # caches it. Ten workers starting together all miss the cache, one builds it
    # and the rest read the partial file - which is how NEURAL_PLUS lost five of
    # its ten workers. Opening one camera here, alone, means the file is complete
    # and valid before anybody races for it. Costs seconds when cached, minutes
    # once, and nothing thereafter.
    print('  warming the %s depth engine (single process, so nothing races)' % a.mode)
    sys.stdout.flush()
    if not warm_up(a.svo[0], a.mode, a.depth_stabilization):
        print('  ABORTING: could not open a camera in %s at all. Nothing was run.' % a.mode)
        return 3

    out = []
    # SPAWN THE WORKERS, DO NOT FORK THEM.
    #
    # plan() opens every recording with the ZED SDK to read its frame count, and
    # the SDK starts threads and takes internal locks the moment a camera opens.
    # multiprocessing's Linux default is fork, which copies the parent's memory
    # but only the calling thread - so the children inherit those locks already
    # held, by a thread that does not exist in them. Every worker then blocked in
    # futex_wait forever: six processes, 0 % CPU, no output, no error, no timeout.
    # It looks exactly like a slow run, which is the dangerous part.
    #
    # spawn starts each worker as a fresh interpreter that imports this module
    # and pyzed itself, so it inherits no locks. It costs one or two seconds per
    # worker at startup, against hours of processing.
    ctx = mp.get_context('spawn')
    with ctx.Pool(a.workers) as pool:
        processed, failures = 0, []
        for shard, rows, done, err in pool.imap_unordered(worker, [tuple(j) for j in jobs]):
            out.extend(rows)
            processed += done
            if err:
                failures.append((shard, err))
                print('    slice %d FAILED: %s' % (shard, err))
            else:
                print('    slice %d done: %d of %d frames accepted (running total %d)'
                      % (shard, len(rows), done, len(out)))
            sys.stdout.flush()

    out.sort(key=lambda r: r[-1])
    for i, r in enumerate(out, 1):
        r[0] = i
    with open(a.out, 'w') as f:
        w = csv.writer(f)
        w.writerow(HDR)
        for r in out:
            w.writerow(['%.5f' % v if isinstance(v, float) else v for v in r])
    # REPORT WHAT WAS PROCESSED, NOT WHAT WAS PLANNED. The old line divided by
    # the planned total, so a run that silently skipped a third of its frames
    # printed a plausible-looking acceptance rate and an exit code of 0.
    print('  wrote %d accepted samples of %d frames ACTUALLY REPLAYED (%.1f%%) -> %s'
          % (len(out), processed, 100.0 * len(out) / max(processed, 1), a.out))
    if processed != total:
        print('  WARNING: %d frames were planned but %d were replayed - %d never ran.'
              % (total, processed, total - processed))
    if failures:
        print('  FAILED: %d of %d slices never opened a camera:' % (len(failures), len(jobs)))
        for shard, err in failures:
            print('    slice %d: %s' % (shard, err))
        print('  This run did NOT see the same frames as the other modes and must')
        print('  NOT be compared with them. Re-run it.')
        return 4
    return 0


if __name__ == '__main__':
    sys.exit(main())
