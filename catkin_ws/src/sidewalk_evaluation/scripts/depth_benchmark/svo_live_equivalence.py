#!/usr/bin/env python3
"""svo_live_equivalence.py - does replaying a recording give the live answer?

THE QUESTION THIS SETTLES

    If depth can be recomputed offline from a recording, in any mode, then a
    station needs only ONE short live recording instead of hours of live
    running per depth mode - and the frame can be moved as soon as the recording
    is made. That would change the whole schedule.

    Everything rests on one assumption: that replaying gives the SAME answer the
    live camera gave. svo_mode_compare.py's own header says that question "is
    still open". This closes it.

METHOD, chosen so that nothing can drift except the thing being tested

    Read the depth at each ball's known image position and take a median over a
    disc of pixels - the per-ball method, which does NO fitting. A fitting method
    could land in a different solution on a second run for reasons that have
    nothing to do with replay, and that would confound the test. Here the only
    difference between live and replay is the path the photons took to the
    number.

    Same discs, same disc radius, same depth mode, same confidence settings as
    the live run. Any disagreement beyond the live run's own frame-to-frame
    spread is the replay path talking.

  usage, on the JETSON:
    python3 svo_live_equivalence.py --svo <file> --mode QUALITY \\
        --balls 'bot_left:730,443;bot_right:1196,443' --frames 25
"""
from __future__ import print_function

import argparse
import sys

import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--svo', required=True)
    ap.add_argument('--mode', default='QUALITY')
    ap.add_argument('--balls', required=True)
    ap.add_argument('--frames', type=int, default=25)
    ap.add_argument('--disc-r', type=int, default=18)
    ap.add_argument('--conf', type=int, default=50)
    ap.add_argument('--texture-conf', type=int, default=100)
    ap.add_argument('--pinned', action='store_true',
                    help='pin the three things that can make a replay differ')
    a = ap.parse_args()

    import pyzed.sl as sl

    balls = []
    for part in a.balls.split(';'):
        if not part.strip():
            continue
        nm, xy = part.split(':')
        u, v = xy.split(',')
        balls.append((nm.strip(), int(u), int(v)))

    init = sl.InitParameters()
    init.set_from_svo_file(a.svo)
    init.depth_mode = getattr(sl.DEPTH_MODE, a.mode)
    init.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
    init.coordinate_units = sl.UNIT.METER
    init.svo_real_time_mode = False       # process every frame, drop none
    init.depth_stabilization = 1          # what the live pipeline uses
    if a.pinned:
        # THE THREE THINGS THAT CAN MAKE A REPLAY DIFFER FROM A LIVE RUN, and
        # from another replay of the same file. Verified present on SDK 4.2.5.
        #
        #   self-calibration runs at open() EVEN ON AN SVO, and Stereolabs
        #   document that the parameters it returns "might vary between two
        #   executions". Default is False, i.e. it runs. Left alone, two
        #   replays of one file are not guaranteed to use the same calibration.
        #
        #   depth_stabilization filters depth across neighbouring frames. This
        #   script SEEKS between sampled frames, so that history is not the
        #   history the live run had. Setting it to 0 removes the dependence.
        init.camera_disable_self_calib = True
        init.depth_stabilization = 0
    cam = sl.Camera()
    st = cam.open(init)
    if st != sl.ERROR_CODE.SUCCESS:
        print('could not open: %s' % st)
        return 1

    total = cam.get_svo_number_of_frames()
    rt = sl.RuntimeParameters()
    rt.confidence_threshold = a.conf
    rt.texture_confidence_threshold = a.texture_conf
    cloud = sl.Mat()
    step = max(1, total // (a.frames + 1))
    per = dict((b[0], []) for b in balls)
    fill = dict((b[0], []) for b in balls)
    got = 0
    for k in range(a.frames):
        cam.set_svo_position(min(k * step, total - 1))
        if cam.grab(rt) != sl.ERROR_CODE.SUCCESS:
            break
        cam.retrieve_measure(cloud, sl.MEASURE.XYZ)
        arr = cloud.get_data()
        x = arr[:, :, 0]
        h, w = x.shape
        yy, xx = np.mgrid[0:h, 0:w]
        for nm, u, v in balls:
            m = ((xx - u) ** 2 + (yy - v) ** 2) <= a.disc_r ** 2
            ok = m & np.isfinite(x)
            fill[nm].append(ok.sum() / float(m.sum()))
            if ok.sum() > 20:
                per[nm].append(float(np.median(x[ok])))
        got += 1
    cam.close()

    print('=' * 74)
    print('  REPLAYED  %s  in %s' % (a.svo.split('/')[-1], a.mode))
    print('=' * 74)
    print('  %d frames sampled across %d in the recording' % (got, total))
    print('  discs of radius %d px at: %s\n'
          % (a.disc_r, ', '.join('%s (%d,%d)' % b for b in balls)))
    print('  %-10s %12s %12s %10s' % ('ball', 'median m', 'spread mm', 'filled %'))
    print('  ' + '-' * 48)
    for nm, _, _ in balls:
        v = np.array(per[nm])
        if len(v) < 3:
            print('  %-10s no depth' % nm)
            continue
        print('  %-10s %12.4f %12.1f %9.0f%%'
              % (nm, np.median(v), v.std(ddof=1) * 1000, 100 * np.mean(fill[nm])))
    return 0


if __name__ == '__main__':
    sys.exit(main())
