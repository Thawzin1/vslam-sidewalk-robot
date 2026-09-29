#!/usr/bin/env python3
"""compare_sensors.py - the sphere-frame benchmark table, for any set of sensors.

Nicolas's compare_runs.py does this for exactly two sensors, ouster and helios,
with their names written into it. This is the same table widened so the camera
sits beside them, plus one thing his version could not do: an ACCURACY check.

TWO DIFFERENT QUESTIONS, AND THIS PRINTS BOTH

  PRECISION - how much does the answer wobble when nothing moves?
              The standard deviation over many frames. This is the depth study's
              Figure-6 metric and the only thing his script reported.

  ACCURACY  - how far is the answer from the truth?
              Different question. A scale that always reads 2 kg heavy but never
              varies has perfect precision and poor accuracy.

              We can ask it here because Nicolas's TWO LiDARs agree with each
              other on mean distance to within about 5 mm at every range
              (4.033 vs 4.038 m; 7.014 vs 7.009 m). Two independent instruments
              agreeing that closely make a usable reference. So a third sensor's
              mean can be measured against them - which tests the camera's
              datasheet claim of "< 2 % up to 10 m" on real hardware.

              This is NOT ground truth from a tape. It is agreement with two
              LiDARs, and it is reported that way.

  usage:
    compare_sensors.py <dir> [<dir2> ...]        # every run found, by distance
    compare_sensors.py <dir> --run 4m            # one run in detail
    compare_sensors.py <dir> --ref ouster,helios # who counts as the reference
"""
from __future__ import print_function

import glob
import os
import sys

import numpy as np

# Sensors treated as the distance reference for the accuracy column. Both are
# LiDARs, both were recorded by Nicolas, and they agree with each other.
DEFAULT_REF = ('ouster', 'helios')
EXPECT_SIDES = (1.10, 1.23, 1.23)

# The camera sits FORWARD of the LiDAR mast, so it honestly reads every
# distance shorter than the LiDARs by that offset. Tape-measured by Thaw Zin
# 10 mm (and the optical centre sits a little BEHIND the glass, which eats a
# few mm more). Same idea as MOUNT_DZ_MM = 170.0 in Nicolas's compare_runs.py,
# just along x instead of z. Applied to zedx ONLY, in the accuracy column ONLY
# - precision (scatter) is untouched by a constant shift.
MOUNT_DX_MM = {'zedx': 115.0}


def load(path):
    """Read one <run_id>_<sensor>.csv. Returns None if it holds no real data.

    An aborted capture leaves a header-only file. numpy reads that as a
    zero-row array, and a mean of nothing is a crash, not a measurement.
    """
    try:
        d = np.genfromtxt(path, delimiter=',', names=True)
    except Exception:
        return None
    if d is None or d.size == 0:
        return None
    d = d.reshape(1) if d.ndim == 0 else d
    return d if len(d) >= 2 else None


def collect(dirs):
    """{run_id: {sensor: record}} across every directory given."""
    out = {}
    for d in dirs:
        for path in sorted(glob.glob(os.path.join(d, '*.csv'))):
            base = os.path.basename(path)[:-4]
            run, _, sensor = base.rpartition('_')
            if not run or not sensor:
                continue
            rec = load(path)
            if rec is None:
                out.setdefault(run, {})[sensor] = {'empty': True, 'path': path}
                continue
            x, y, z = rec['x'], rec['y'], rec['z']
            e = {
                'n': len(rec), 'path': path,
                'mean': (float(np.mean(x)), float(np.mean(y)), float(np.mean(z))),
                'sd_mm': (float(np.std(x, ddof=1) * 1000),
                          float(np.std(y, ddof=1) * 1000),
                          float(np.std(z, ddof=1) * 1000)),
                'empty': False,
            }
            e['rms3d_mm'] = float(np.sqrt(sum(s * s for s in e['sd_mm'])))

            # ---- SHORT-WINDOW SIGMA: the only figure comparable across sensors.
            #
            # Nicolas's 1000 LiDAR samples span about 100 seconds. The camera's
            # detector needs ~16 s per frame on a 95,000-point cloud, so its 1000
            # samples span over four hours. A standard deviation taken across four
            # hours contains every slow drift there is - camera temperature, the
            # light changing, the mount settling - and calling that "sensor noise"
            # would overstate it badly. The camera's own height was already seen
            # moving 1.8 mm inside one minute.
            #
            # So sigma is also computed inside short windows and the windows'
            # results averaged. That measures frame-to-frame scatter over a span
            # comparable to Nicolas's whole run, whatever the capture took in
            # wall-clock time.
            #
            # The GAP between the two is not an error - it is the slow drift,
            # reported separately as drift_mm.
            if 'stamp' in rec.dtype.names and len(rec) > 20:
                t = np.asarray(rec['stamp'], float)
                e['span_s'] = float(t[-1] - t[0])
                win = []
                lo = 0
                for hi in range(len(t)):
                    if t[hi] - t[lo] >= 100.0:        # 100 s, matching the LiDAR runs
                        if hi - lo >= 10:
                            win.append(np.sqrt(sum(
                                np.std(v[lo:hi], ddof=1) ** 2 for v in (x, y, z))) * 1000)
                        lo = hi
                e['rms3d_100s_mm'] = float(np.median(win)) if win else float('nan')
                e['n_windows'] = len(win)
                # Drift is what the long run has that the short windows do not.
                if win and np.isfinite(e['rms3d_100s_mm']):
                    d2 = e['rms3d_mm'] ** 2 - e['rms3d_100s_mm'] ** 2
                    e['drift_mm'] = float(np.sqrt(d2)) if d2 > 0 else 0.0
                else:
                    e['drift_mm'] = float('nan')
            else:
                e['span_s'] = float('nan')
                e['rms3d_100s_mm'] = float('nan')
                e['n_windows'] = 0
                e['drift_mm'] = float('nan')
            for k, nm in (('base', 'base'), ('leg1', 'leg1'), ('leg2', 'leg2')):
                e[nm] = float(np.mean(rec[k])) if k in rec.dtype.names else float('nan')
            pts = [rec[k] for k in ('n_bot1', 'n_bot2', 'n_apex')
                   if k in rec.dtype.names]
            e['pts'] = float(np.mean(np.concatenate(pts))) if pts else float('nan')
            # The first 200, so a 1000-sample run is still comparable with a
            # 200-sample one taken earlier.
            if len(rec) >= 200:
                e['rms3d_200_mm'] = float(np.sqrt(sum(
                    np.std(v[:200], ddof=1) ** 2 * 1e6 for v in (x, y, z))))
            else:
                e['rms3d_200_mm'] = float('nan')
            out.setdefault(run, {})[sensor] = e
    return out


def sides_ok(e):
    """Did the detector actually lock onto the frame, or onto something else?

    The honesty check. Sigma can look beautiful on a confident fit to the wrong
    three blobs, so the measured side lengths are what decide whether a row means
    anything at all.
    """
    got = np.sort([e['base'], e['leg1'], e['leg2']])
    if not np.all(np.isfinite(got)):
        return None
    return bool(np.all(np.abs(got - np.sort(EXPECT_SIDES)) < 0.05))


def table(runs, refs):
    order = sorted(runs, key=lambda r: (
        float(''.join(c for c in r if c.isdigit() or c == '.') or 1e9), r))
    sensors = sorted({s for v in runs.values() for s in v})

    print('=' * 104)
    print('  SPHERE-FRAME CENTROID PRECISION      sigma = standard deviation over n frames')
    print('=' * 104)
    print('  %-8s %-8s %6s %7s %7s %7s %7s %8s %9s %8s %7s %s'
          % ('run', 'sensor', 'n', 'mean_x', 'sig_x', 'sig_y', 'sig_z',
             'RMS3D', 'RMS3D@100s', 'drift', 'pts/sph', 'sides'))
    print('  %-8s %-8s %6s %7s %7s %7s %7s %8s %9s %8s %7s   %s'
          % ('', '', '', '[m]', '[mm]', '[mm]', '[mm]', 'whole', 'COMPARABLE',
             '[mm]', '', '<- use RMS3D@100s to compare sensors'))
    print('-' * 104)
    for run in order:
        for s in sensors:
            e = runs[run].get(s)
            if e is None:
                continue
            if e.get('empty'):
                print('  %-8s %-9s %6s   -- NO DATA: the capture produced no accepted '
                      'frames (this is a result, not a gap)' % (run, s, ''))
                continue
            ok = sides_ok(e)
            flag = ('ok' if ok else '*** SIDES WRONG - DO NOT USE ***'
                    if ok is False else '?')
            print('  %-8s %-8s %6d %7.3f %7.2f %7.2f %7.2f %8.2f %9.2f %8.2f %7.0f %s'
                  % (run, s, e['n'], e['mean'][0], e['sd_mm'][0], e['sd_mm'][1],
                     e['sd_mm'][2], e['rms3d_mm'], e['rms3d_100s_mm'],
                     e['drift_mm'], e['pts'], flag))
            if e.get('span_s') == e.get('span_s') and e['span_s'] > 600:
                print('  %-8s %-8s   captured over %.1f HOURS in %d windows - the '
                      'whole-run RMS3D above is inflated by slow drift;'
                      % ('', '', e['span_s'] / 3600.0, e['n_windows']))
                print('  %-8s %-8s   RMS3D@100s is the like-for-like figure.'
                      % ('', ''))
        print('-' * 104)

    # ---------------- accuracy against the agreeing LiDAR pair ----------------
    print()
    print('=' * 104)
    print('  ACCURACY - mean distance against the LiDAR reference (%s)' % ', '.join(refs))
    print('=' * 104)
    print('  A reference is only used when the reference sensors agree with EACH OTHER,')
    print('  because two instruments that disagree cannot both be truth.')
    print()
    print('  %-8s %-9s %10s %12s %10s %s'
          % ('run', 'sensor', 'mean_x[m]', 'vs ref [mm]', 'as %', 'reference'))
    print('-' * 104)
    for run in order:
        have = [runs[run][s]['mean'][0] for s in refs
                if s in runs[run] and not runs[run][s].get('empty')]
        if len(have) < 2:
            spread = float('nan')
            ref = have[0] if have else None
            note = 'only one reference sensor - agreement unknown' if have else 'no reference'
        else:
            ref = float(np.mean(have))
            spread = (max(have) - min(have)) * 1000
            note = 'refs agree to %.1f mm' % spread
        if ref is None:
            print('  %-8s %-9s %10s %12s %10s %s' % (run, '-', '-', '-', '-', note))
            continue
        for s in sorted(runs[run]):
            e = runs[run][s]
            if e.get('empty'):
                continue
            dx = MOUNT_DX_MM.get(s, 0.0)
            d_mm = (e['mean'][0] - ref) * 1000 + dx
            tag = '(is a reference)' if s in refs else note
            if dx:
                tag += '  [mount +%d mm applied]' % dx
            print('  %-8s %-9s %10.3f %12.1f %10.2f %s'
                  % (run, s, e['mean'][0], d_mm, 100.0 * d_mm / 1000.0 / ref, tag))
        print('-' * 104)
    print()
    print('  NOTE: "vs ref" is agreement with the LiDAR pair, NOT error against a tape.')
    print('  The zedx row includes the tape-measured 115 mm forward-mount correction')
    print('  (+/- ~10 mm, see MOUNT_DX_MM) - its residual carries that uncertainty.')
    print('  A camera reading consistently high or low by a fixed percentage is a')
    print('  calibration offset, which is a different fault from a noisy reading and is')
    print('  usually correctable. Say which one was seen.')


def detail(runs, run_id):
    if run_id not in runs:
        sys.exit('  no run called %r. Have: %s' % (run_id, ', '.join(sorted(runs))))
    print('=' * 72)
    print('  Run: %s' % run_id)
    print('=' * 72)
    for s in sorted(runs[run_id]):
        e = runs[run_id][s]
        if e.get('empty'):
            print('  %-10s NO DATA (%s)' % (s, os.path.basename(e['path'])))
            continue
        print('  %-10s n=%d' % (s, e['n']))
        print('      mean      (%.4f, %.4f, %.4f) m' % e['mean'])
        print('      sigma     (%.2f, %.2f, %.2f) mm    RMS3D %.2f mm'
              % (e['sd_mm'] + (e['rms3d_mm'],)))
        print('      sides     %.3f / %.3f / %.3f m   expected 1.10 / 1.23 / 1.23  -> %s'
              % (e['base'], e['leg1'], e['leg2'],
                 'ok' if sides_ok(e) else 'WRONG - the detector locked onto something else'))
        print('      pts/sphere %.0f' % e['pts'])
    print('=' * 72)


if __name__ == '__main__':
    args = [a for a in sys.argv[1:]]
    refs = DEFAULT_REF
    if '--ref' in args:
        i = args.index('--ref')
        refs = tuple(args[i + 1].split(','))
        del args[i:i + 2]
    run_id = None
    if '--run' in args:
        i = args.index('--run')
        run_id = args[i + 1]
        del args[i:i + 2]
    dirs = [a for a in args if not a.startswith('-')]
    if not dirs:
        sys.exit(__doc__)
    runs = collect(dirs)
    if not runs:
        sys.exit('  no <run_id>_<sensor>.csv files under: %s' % ', '.join(dirs))
    if run_id:
        detail(runs, run_id)
    else:
        table(runs, refs)
