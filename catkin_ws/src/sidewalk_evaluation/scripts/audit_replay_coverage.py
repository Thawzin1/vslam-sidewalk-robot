#!/usr/bin/env python3
"""audit_replay_coverage.py - did this replay actually see every frame it claimed?

WHY THIS EXISTS

    On 2026-09-10 a NEURAL_PLUS pass reported "511 accepted samples of 935 frames
    offered (55%)" and exited 0. It had replayed 593 frames. Five of its ten
    workers never opened a camera - the depth mode's optimized engine was being
    built by a sixth and they read the partial file - and a worker that fails to
    open returned an empty list, which is exactly what a worker that honestly
    found nothing returns.

    Nothing about the result looked wrong. The sigma was plausible and sat
    comfortably beside the other mode's. But a mode that saw 593 frames cannot be
    compared with one that saw 935, and comparing modes on identical frames is
    the only reason the campaign exists.

    svo_replay_n.py now refuses to hide this. THIS SCRIPT IS FOR RUNS MADE BEFORE
    THAT FIX, and as a standing independent check afterwards - it reads the
    per-worker counters, which are written by the workers themselves as they go
    and owe nothing to the summary line.

WHAT IT CHECKS

    1. Frames actually replayed against frames the recordings hold.
    2. Any worker that replayed nothing at all - the signature of a failed open.
    3. Acceptance per source recording, which is how a per-file problem shows
       itself. The camera loses detections with each recording it makes, so a
       decline across a burst sequence is expected and is NOT a fault; a single
       file at zero is.

  usage, on the JETSON:
    python3 audit_replay_coverage.py ULTRA_bursts
    python3 audit_replay_coverage.py            # every pass it can find
"""
from __future__ import print_function

import os
import sys

PROGROOT = '~/replay_prog'
# Frames each pass's recordings hold, read with the SDK, not assumed.
EXPECTED = {'bursts': 935, 'block': 2502, 'ctl_1worker': 125, 'ctl_nworker': 125,
            'ctl_perfile': 935}
# Frames per burst file, in the order plan() walks them.
BURSTS = [125, 126, 123, 118, 115, 111, 109, 108]


def read_shards(d):
    out = {}
    try:
        names = [n for n in os.listdir(d) if n.startswith('shard_') and not n.endswith('.tmp')]
    except OSError:
        return out
    for n in sorted(names):
        try:
            f = open(os.path.join(d, n)).read().split()
            if len(f) < 6:
                continue
            s, done, span, acc, sparse, nofit = (int(x) for x in f[:6])
        except (OSError, ValueError):
            continue
        out[s] = dict(done=done, span=span, accepted=acc, sparse=sparse, nofit=nofit)
    return out


def audit(name):
    d = os.path.join(PROGROOT, name)
    sh = read_shards(d)
    if not sh:
        print('%-26s no worker counters found' % name)
        return None
    # MATCH THE PASS NAME AT THE END, NOT BY SPLITTING AT THE FIRST UNDERSCORE.
    #
    # `name.split('_', 1)[1]` turned "NEURAL_PLUS_bursts" into "PLUS_bursts",
    # which matched nothing in EXPECTED, so `want` was None and the frame-count
    # check quietly did not run - and this script stamped COMPLETE on the very
    # pass it was written to catch. Depth modes have underscores in them; pass
    # names are a closed set, so test against that set from the right-hand end.
    tag = next((t for t in EXPECTED if name.endswith('_' + t)), '')
    want = EXPECTED.get(tag)
    if not want:
        print('  ** UNRECOGNISED PASS NAME "%s" - cannot check coverage.' % name)
        print('     Known passes: %s' % ', '.join(sorted(EXPECTED)))
    done = sum(v['done'] for v in sh.values())
    acc = sum(v['accepted'] for v in sh.values())
    sparse = sum(v['sparse'] for v in sh.values())
    nofit = sum(v['nofit'] for v in sh.values())
    dead = sorted(s for s, v in sh.items() if v['done'] == 0)

    print('=' * 72)
    print('%s' % name)
    print('  workers reporting     : %d' % len(sh))
    print('  frames replayed       : %d%s' % (done, ('  of %d in the recordings' % want) if want else ''))
    print('  accepted              : %d  (%.1f%% of frames REPLAYED)'
          % (acc, 100.0 * acc / max(done, 1)))
    print('  rejected, depth empty : %d   <- camera produced almost no points' % sparse)
    print('  rejected, no fit      : %d   <- points were there, geometry did not fit' % nofit)

    ok = True
    if dead:
        ok = False
        print('  ** %d WORKER(S) REPLAYED NOTHING: slices %s'
              % (len(dead), ', '.join(str(s) for s in dead)))
        print('     That is what a failed camera open looks like from outside.')
    if not want:
        ok = False          # unrecognised name: cannot vouch for it either way
    if want and done < want:
        ok = False
        print('  ** %d FRAMES NEVER RAN (%d of %d, %.0f%%).'
              % (want - done, done, want, 100.0 * done / want))
        print('     This run did NOT see the same frames as a complete one and')
        print('     MUST NOT be compared with it. Re-run it.')
    elif want and done > want:
        ok = False
        print('  ** MORE frames replayed (%d) than the recordings hold (%d).' % (done, want))
        print('     Counters may be left over from an earlier pass in the same folder.')

    gaps = [s for s in range(max(sh) + 1) if s not in sh] if sh else []
    if gaps:
        ok = False
        print('  ** SLICES THAT NEVER REPORTED AT ALL: %s'
              % ', '.join(str(s) for s in gaps))
        print('     A worker that fails before its first counter write leaves NO')
        print('     file, so these are ABSENT rather than zero - which is why a')
        print('     "did any worker report nothing?" test alone does not find them.')

    # per-recording acceptance, for the bursts pass only, where the mapping is known
    if tag == 'bursts' and len(sh) >= 2:
        print('  per recording:')
        for b, frames in enumerate(BURSTS):
            pair = [sh.get(2 * b), sh.get(2 * b + 1)]
            d_ = sum(p['done'] for p in pair if p)
            a_ = sum(p['accepted'] for p in pair if p)
            flag = '   <- NOTHING REPLAYED' if d_ == 0 else ''
            print('    burst%02d  %3d of %3d frames   %3d accepted  %5.1f%%%s'
                  % (b + 1, d_, frames, a_, 100.0 * a_ / max(d_, 1), flag))
    print('  VERDICT: %s' % ('COMPLETE - safe to compare with other modes' if ok
                             else 'INCOMPLETE - not comparable, re-run'))
    return ok


def main():
    names = sys.argv[1:]
    if not names:
        try:
            names = sorted(n for n in os.listdir(PROGROOT)
                           if os.path.isdir(os.path.join(PROGROOT, n))
                           and not n.startswith('_'))
        except OSError:
            print('no %s on this machine - run this on the Jetson' % PROGROOT)
            return 2
    bad = 0
    for n in names:
        if audit(n) is False:
            bad += 1
    print('=' * 72)
    print('%d pass(es) audited, %d incomplete' % (len(names), bad))
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
