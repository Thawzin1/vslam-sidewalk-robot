#!/usr/bin/env python3
"""turns_replay.py - the live turn watcher's exact rule, replayed after the drive from fusion.bag.

WHY: drive 4 (s2_static_04) ran without the live turn watcher (~/.run_records/s2_static_04/ has no
turns.log). This replays tools/drive/turn_watch.py's rule unchanged, message by message in the
order the recorder received them (rosbag receive time = the closest thing to the live arrival order):
  a turn = the wheels' turn rate above 0.10 rad/s for 0.3 s; it ends after 1.0 s below 0.05 rad/s;
  headings unwrapped; "camera" = the tracker's own heading, "-" when lost for > 95 % of the turn;
  the camera counts as lost when its pose or twist covariance >= 9999 * 0.999.
Writes the same line format as the live turns.log (HH:MM:SS in UTC, as the live one), so
analyze_run.py reads it unchanged. The line times are the recorder's receive times, not wall clock.

*Plain terms: the program that writes down each turn was not running during drive 4, so the same rule is
run afterwards over the drive's recording. Same rule, same input; only the moment it ran differs.*

usage: turns_replay.py <fusion.bag> <out turns log>      (Jetson, read-only on the bag)
"""
import math, sys, time
import rosbag

BAD = 9999.0 * 0.999


def yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Unwrap(object):
    def __init__(self):
        self.prev, self.acc = None, None

    def add(self, v):
        if self.prev is None:
            self.acc = v
        else:
            d = v - self.prev
            while d > math.pi:
                d -= 2 * math.pi
            while d < -math.pi:
                d += 2 * math.pi
            self.acc += d
        self.prev = v
        return self.acc


bag, out = sys.argv[1], sys.argv[2]
u = {'fused': Unwrap(), 'robot': Unwrap(), 'camera': Unwrap()}
h = {'fused': None, 'robot': None, 'camera': None}
st = dict(cam_lost=False, turn=None, n=0, above=None, below=None, dist=0.0, lwt=None, sv=0, ndir=0)
lines = []
now = [0.0]


def say(s):
    lines.append('%s %s' % (time.strftime('%H:%M:%S', time.gmtime(now[0])), s))


def end_turn(t):
    tr = st['turn']
    st['turn'], st['below'], st['above'] = None, None, None
    d = {}
    for k in ('fused', 'robot', 'camera'):
        a, b = tr['start'].get(k), h.get(k)
        d[k] = None if a is None or b is None else b - a
    lost = tr['cam_bad'] / float(tr['cam_n']) if tr['cam_n'] else 1.0
    if lost > 0.95:
        d['camera'] = None
    err = None if d['fused'] is None or d['robot'] is None else d['fused'] - d['robot']
    f = lambda v: '-' if v is None else '%+.1f' % v
    say('TURN %d %s ended after %.1f s: blend %s deg, robot %s deg, camera %s deg (camera lost %d%% of it); '
        'blend - robot %s deg' % (tr['n'], tr['side'], t - tr['t'] - 1.0, f(d['fused']), f(d['robot']),
                                  f(d['camera']), round(100 * lost), f(err)))


with rosbag.Bag(bag) as b:
    say_first = True
    for topic, m, tb in b.read_messages(topics=['/robot/wheel_odom', '/fused/odometry', '/robot/ekf_odom',
                                                '/rtabmap/odom']):
        now[0] = tb.to_sec()
        if say_first:
            say('WATCH START (replayed after the drive from fusion.bag by turns_replay.py)')
            say_first = False
        if topic == '/fused/odometry':
            h['fused'] = math.degrees(u['fused'].add(yaw(m.pose.pose.orientation)))
        elif topic == '/robot/ekf_odom':
            h['robot'] = math.degrees(u['robot'].add(yaw(m.pose.pose.orientation)))
        elif topic == '/rtabmap/odom':
            bad = m.pose.covariance[0] >= BAD or m.twist.covariance[0] >= BAD
            st['cam_lost'] = bad
            if st['turn'] is not None:
                st['turn']['cam_n'] += 1
                st['turn']['cam_bad'] += int(bad)
            if not bad:
                h['camera'] = math.degrees(u['camera'].add(yaw(m.pose.pose.orientation)))
        else:
            t = m.header.stamp.to_sec()
            wz, vx = m.twist.twist.angular.z, m.twist.twist.linear.x
            if st['lwt'] is not None and 0 < t - st['lwt'] < 1.0:
                st['dist'] += abs(vx) * (t - st['lwt'])
            st['lwt'] = t
            sv = 1 if vx > 0.05 else (-1 if vx < -0.05 else 0)
            if sv != 0:
                if st['sv'] != 0 and sv != st['sv']:
                    st['ndir'] += 1
                    say('DIRECTION CHANGE #%d: now %s (driven %.1f m)' % (st['ndir'], 'FORWARD' if sv > 0 else 'BACKWARD', st['dist']))
                st['sv'] = sv
            if st['turn'] is None:
                if abs(wz) > 0.10:
                    st['above'] = st['above'] or t
                    if t - st['above'] >= 0.3:
                        st['n'] += 1
                        st['turn'] = dict(n=st['n'], t=t, side='LEFT' if wz > 0 else 'RIGHT', cam_n=0, cam_bad=0,
                                          start=dict(h))
                        say('TURN %d %s started (driven %.1f m)%s' % (st['n'], st['turn']['side'], st['dist'],
                                                                     ' - camera LOST' if st['cam_lost'] else ''))
                else:
                    st['above'] = None
            else:
                if abs(wz) < 0.05:
                    st['below'] = st['below'] or t
                    if t - st['below'] >= 1.0:
                        end_turn(t)
                else:
                    st['below'] = None
say('WATCH END (end of fusion.bag): %d turns, %d direction changes, %.1f m by the wheels' % (st['n'], st['ndir'], st['dist']))
open(out, 'w').write('\n'.join(lines) + '\n')
print(lines[-1])
