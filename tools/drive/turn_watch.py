#!/usr/bin/env python3
"""turn_watch.py - during a drive, one line per TURN and per change of direction, with how far the
blend, the robot and the camera each say the robot turned; plus tracking losses, loop closures and
the bridge's state.  RUNS ON: the Jetson, beside a FUSION=1 drive.  Listens only; changes nothing.

  python3 turn_watch.py __name:=turn_watch _run:=s2_static_03

Writes ~/.run_records/<run>/turns.log (one line per event) and ~/jobs/<run>_turns.progress (the
jobs page, rule 13).  Stops by itself when the drive's map has closed (autostop.log says complete).

A turn = the wheels' turn rate above 0.10 rad/s (~6 deg/s) for 0.3 s; it ends after 1.0 s below
0.05 rad/s.  A direction change = the wheels' forward speed changes sign (above 0.05 m/s each way).
Headings are unwrapped; "camera" is the tracker's own heading, "-" when it was lost for the whole turn.
*Plain terms: every time the robot turns, write down how much each of the three thinks it turned.*
"""
import json
import math
import os
import time
import urllib.request

import rospy
from nav_msgs.msg import Odometry

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


class TurnWatch(object):
    def __init__(self):
        self.run = rospy.get_param('~run')
        self.rec = os.path.expanduser('~/.run_records/%s' % self.run)
        self.log_p = os.path.join(self.rec, 'turns.log')
        self.prog_p = os.path.expanduser('~/jobs/%s_turns.progress' % self.run)
        self.t0 = time.time()
        self.u = {'fused': Unwrap(), 'robot': Unwrap(), 'camera': Unwrap()}
        self.h = {'fused': None, 'robot': None, 'camera': None}
        self.cam_lost = False
        self.turn = None            # dict while turning
        self.n_turns = 0
        self.n_dir = 0
        self.above_since = None
        self.below_since = None
        self.last_sign_v = 0
        self.dist = 0.0
        self.last_wt = None
        self.state = {}
        self.bridge = None
        self.sum_abs_err = 0.0
        self.say('WATCH START %s' % self.run)
        rospy.Subscriber('/robot/wheel_odom', Odometry, self.on_wheel, queue_size=50)
        rospy.Subscriber('/fused/odometry', Odometry, lambda m: self.on_yaw('fused', m), queue_size=50)
        rospy.Subscriber('/robot/ekf_odom', Odometry, lambda m: self.on_yaw('robot', m), queue_size=50)
        rospy.Subscriber('/rtabmap/odom', Odometry, self.on_cam, queue_size=50)
        rospy.Timer(rospy.Duration(2.0), self.poll)

    def say(self, s):
        line = '%s %s' % (time.strftime('%H:%M:%S'), s)
        with open(self.log_p, 'a') as f:
            f.write(line + '\n')

    def on_yaw(self, k, m):
        self.h[k] = math.degrees(self.u[k].add(yaw(m.pose.pose.orientation)))

    def on_cam(self, m):
        bad = m.pose.covariance[0] >= BAD or m.twist.covariance[0] >= BAD
        self.cam_lost = bad
        if self.turn is not None:
            self.turn['cam_n'] += 1
            self.turn['cam_bad'] += int(bad)
        if not bad:
            self.h['camera'] = math.degrees(self.u['camera'].add(yaw(m.pose.pose.orientation)))

    def on_wheel(self, m):
        t = m.header.stamp.to_sec()
        wz, vx = m.twist.twist.angular.z, m.twist.twist.linear.x
        if self.last_wt is not None and 0 < t - self.last_wt < 1.0:
            self.dist += abs(vx) * (t - self.last_wt)
        self.last_wt = t
        sv = 1 if vx > 0.05 else (-1 if vx < -0.05 else 0)
        if sv != 0:
            if self.last_sign_v != 0 and sv != self.last_sign_v:
                self.n_dir += 1
                self.say('DIRECTION CHANGE #%d: now %s (driven %.1f m)'
                         % (self.n_dir, 'FORWARD' if sv > 0 else 'BACKWARD', self.dist))
            self.last_sign_v = sv
        if self.turn is None:
            if abs(wz) > 0.10:
                self.above_since = self.above_since or t
                if t - self.above_since >= 0.3:
                    self.n_turns += 1
                    self.turn = dict(t=t, side='LEFT' if wz > 0 else 'RIGHT', cam_n=0, cam_bad=0,
                                     start={k: v for k, v in self.h.items()}, lost_at_start=self.cam_lost)
                    self.say('TURN %d %s started (driven %.1f m)%s' % (self.n_turns, self.turn['side'], self.dist,
                                                                     ' - camera LOST' if self.cam_lost else ''))
            else:
                self.above_since = None
        else:
            if abs(wz) < 0.05:
                self.below_since = self.below_since or t
                if t - self.below_since >= 1.0:
                    self.end_turn(t)
            else:
                self.below_since = None

    def end_turn(self, t):
        tr, self.turn, self.below_since, self.above_since = self.turn, None, None, None
        d = {}
        for k in ('fused', 'robot', 'camera'):
            a, b = tr['start'].get(k), self.h.get(k)
            d[k] = None if a is None or b is None else b - a
        lost = tr['cam_bad'] / float(tr['cam_n']) if tr['cam_n'] else 1.0
        if lost > 0.95:
            d['camera'] = None
        err = None if d['fused'] is None or d['robot'] is None else d['fused'] - d['robot']
        if err is not None:
            self.sum_abs_err += abs(err)
        f = lambda v: '-' if v is None else '%+.1f' % v
        self.say('TURN %d %s ended after %.1f s: blend %s deg, robot %s deg, camera %s deg (camera lost %d%% of it); '
                 'blend - robot %s deg' % (self.n_turns, tr['side'], t - tr['t'] - 1.0, f(d['fused']), f(d['robot']),
                                           f(d['camera']), round(100 * lost), f(err)))

    def poll(self, _e):
        try:
            s = json.loads(urllib.request.urlopen('http://127.0.0.1:8095/stats.json', timeout=1.5).read().decode())
            st = s.get('_state', {})
            for key, label in (('corr_loop', 'LOOP CLOSURE'), ('corr_prox', 'PROXIMITY CLOSURE')):
                v = st.get(key)
                if v is not None and self.state.get(key) is not None and v > self.state[key]:
                    self.say('%s: now %d (%s)' % (label, v, s.get('last map correction', '')[:100]))
                if v is not None:
                    self.state[key] = v
            lost = bool(st.get('lost'))
            if self.state.get('lost') is not None and lost != self.state['lost']:
                self.say('TRACKING %s' % ('LOST' if lost else 'BACK (%s)' % s.get('tracking', '')))
            self.state['lost'] = lost
        except Exception as e:                                   # the page is optional here
            if self.state.get('page') != 'down':
                self.say('live map page not answering (%s) - turns still logged' % type(e).__name__)
            self.state['page'] = 'down'
        try:
            b = open(os.path.expanduser('~/jobs/%s_bridge.progress' % self.run)).readline().split()[:2]
            b = ' '.join(b)
            if b != self.bridge:
                self.say('BRIDGE %s' % b)
                self.bridge = b
        except Exception:
            pass
        with open(self.prog_p + '.tmp', 'w') as f:
            f.write('TURNS %s %d turns, %d direction changes, %s loop + %s proximity closures, %.1f m, '
                    'sum |blend - robot| %.1f deg  %ds\n'
                    % (self.run, self.n_turns, self.n_dir, self.state.get('corr_loop', '?'),
                       self.state.get('corr_prox', '?'), self.dist, self.sum_abs_err, time.time() - self.t0))
        os.replace(self.prog_p + '.tmp', self.prog_p)
        try:
            a = open(os.path.join(self.rec, 'autostop.log')).read().splitlines()[-1]
            if 'complete' in a:
                self.say('WATCH END: the map has closed')
                rospy.signal_shutdown('drive closed')
        except Exception:
            pass


if __name__ == '__main__':
    rospy.init_node('turn_watch')
    TurnWatch()
    rospy.spin()
