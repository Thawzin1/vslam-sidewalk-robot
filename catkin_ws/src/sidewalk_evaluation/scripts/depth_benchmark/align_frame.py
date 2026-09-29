#!/usr/bin/env python3
"""align_frame.py - a live readout for aiming the target frame, so the person
moving it can watch the numbers instead of waiting for someone to compute them.

WHY THIS EXISTS

    At 4 m the frame was squared by running one measuring tool, reading its
    output, working out the correction by hand, calling it out, waiting for the
    nudge, and running the tool again. That works - it got 124.6 mm down to
    4.0 mm - but each cycle costs minutes, and the same job is waiting at 6, 7,
    8 and 9 m.

    This prints all three corrections, in millimetres and in plain words, every
    few seconds, while the frame is being moved.

WHAT IT MEASURES, AND WHY NOT WITH THE DETECTOR

    It does NOT use the frozen sphere detector. At 5 m that detector was firing
    on 8 looks out of 22, and a number that only appears a third of the time is
    useless to someone standing with their hands on the frame. The three
    quantities here need only the raw points, and the direct per-ball depth
    measurement was repeatable to 7 mm across two runs at the same station.

    Three corrections. None of them needs the cloud levelled - see the note
    above clusters() for why the floor is neither available nor required:

      OFFSET  is the frame centred on the camera's axis? Sideways error.
      TURN    is the frame square to the camera, or rotated about vertical?
              Read from the two lower balls being at different ranges.
      LEAN    is the frame standing upright, or tipped toward the camera?
              Read from the apex sitting nearer or further than the base.

    The lean is the one no previous tool reported, and at 5 m it was the largest
    error of the three: the apex sat 105 mm NEARER than the base midpoint where
    an upright frame demands about 54 mm FURTHER - a tip of about 8 degrees.

THE CAP BIAS DOES NOT INVALIDATE ANY OF THIS

    The camera sees only the near face of each ball, so every cluster's mean
    range sits biased toward the camera by roughly the depth of the visible cap.
    All three balls carry that same bias, so it cancels in every DIFFERENCE
    taken below, which is what all three corrections are made of. It would only
    matter if this claimed an absolute distance, and it does not.

  usage, on the Jetson, with the camera already running:
      rosrun sidewalk_evaluation align_frame.py _x:=5.0
      python3 align_frame.py _x:=5.0 _sensor_h:=0.6972
"""
from __future__ import print_function

import sys

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

# from the camera image, the two agreeing to 6 mm. These are the ball CENTRE
# heights above the floor, not the frame's tube.
BALL_Z_LOW = 0.897
BALL_Z_TOP = 1.997
BASE_M = 1.10          # between the two lower ball centres
RISE_M = BALL_Z_TOP - BALL_Z_LOW

# How close is close enough. Beyond these the run is worth stopping to fix; the
# 4 m station was closed at 4.0 mm and 0.31 degrees, so these are achievable
# rather than aspirational.
OK_OFFSET_MM = 25.0
OK_TURN_DEG = 1.0
OK_LEAN_DEG = 1.0

BAND = 0.10            # half-height of the slice taken around each ball centre


# ---------------------------------------------------------------------------
# WHY THERE IS NO FLOOR-LEVELLING STEP HERE
#
#   The first version of this fitted the floor and rotated the cloud onto it.
#   It never once succeeded: the cloud is in the CAMERA's frame and the camera
#   is pitched up 2.818 degrees, so the floor is not flat in these coordinates -
#   at 5 m it sits at z = -0.94 rather than -0.70, and it drops further the
#   further out you look. roi_viz.py sidesteps the same problem by using a KNOWN
#   floor height and explicitly ignoring what the histogram claims.
#
#   It turns out no levelling is needed, because of which quantities are wanted:
#
#     OFFSET is measured in y. A pitch is a rotation ABOUT the y axis, so it
#            cannot change y at all. Pitch-free by construction.
#     TURN   is the difference in range between two balls at the SAME height.
#            A pitch shifts both by the same amount, so the difference survives.
#     LEAN   compares the apex against the base, 1.10 m apart in height, so this
#            one genuinely does depend on the pitch - and only this one.
#
#   So the pitch enters in exactly one place, as the expected range difference
#   between apex and base for a frame that is standing perfectly upright:
#
#       x_apex - x_base  =  RISE * sin(pitch)   =  +54 mm at pitch 2.818 deg
#
#   A frame leaning toward the camera reads LESS than that. At 5 m it read
#   -105 mm, so it was leaning by 160 mm over its rise, about 8 degrees.
# ---------------------------------------------------------------------------


def expected_apex_lead(pitch_deg):
    """Range difference apex-minus-base for a perfectly upright frame, metres."""
    return RISE_M * np.sin(np.radians(pitch_deg))


def clusters(pts, sh, pitch_deg):
    """The three ball centres, as (right, left, apex), in the camera's frame.

    Slices are taken around each ball's known centre height, which avoids the
    frame's own tubes and the floor without needing a clustering algorithm.
    Within the lower slice the two balls are split at the midpoint of their
    spread in y, which is safe because they are 1.10 m apart.

    The slice heights have to be taken in the CAMERA's tilted z, so each ball's
    height above the floor is converted through the pitch before slicing.
    Getting this wrong slices the tubes instead of the balls.
    """
    th = np.radians(pitch_deg)
    # a ball at height h above the floor, at range r, sits at camera-frame
    # z = -r sin(th) + (h - sh) cos(th)
    r = np.median(pts[:, 0])
    z_low = -r * np.sin(th) + (BALL_Z_LOW - sh) * np.cos(th)
    z_top = -r * np.sin(th) + (BALL_Z_TOP - sh) * np.cos(th)
    lo = pts[np.abs(pts[:, 2] - z_low) < BAND]
    hi = pts[np.abs(pts[:, 2] - z_top) < BAND]
    if len(lo) < 200 or len(hi) < 100:
        return None

    # A horizontal slice across the whole box catches the frame's own tubes and
    # whatever stands behind it. The first version did exactly that and reported
    # the base as 1.784 m against the frame's real 1.10 m - so every number it
    # printed was contaminated. The balls are the NEAREST thing in their slice,
    # so keep only what sits within a ball's diameter of the closest surface.
    def nearest_shell(g, depth=0.45):
        if len(g) < 40:
            return g
        front = np.percentile(g[:, 0], 5)
        return g[g[:, 0] < front + depth]

    lo, hi = nearest_shell(lo), nearest_shell(hi)
    if len(lo) < 200 or len(hi) < 100:
        return None

    # Then window each lower ball where the frame's OWN geometry says it must
    # be: half a base either side of the frame's centre. Using the known 1.10 m
    # is what stops a stray patch of wall from being read as a ball.
    c = float(np.median(lo[:, 1]))
    half = BASE_M / 2.0
    a = lo[np.abs(lo[:, 1] - (c - half)) < 0.22]     # smaller y  = camera's right
    b = lo[np.abs(lo[:, 1] - (c + half)) < 0.22]     # larger  y  = camera's left
    if len(a) < 80 or len(b) < 80:
        return None
    # one refinement pass, now that the centre is known better than the median
    c = 0.5 * (float(np.median(a[:, 1])) + float(np.median(b[:, 1])))
    a = lo[np.abs(lo[:, 1] - (c - half)) < 0.22]
    b = lo[np.abs(lo[:, 1] - (c + half)) < 0.22]
    t = hi[np.abs(hi[:, 1] - c) < 0.30]
    if len(a) < 80 or len(b) < 80 or len(t) < 60:
        return None
    return [np.median(g, axis=0) for g in (a, b, t)], (len(a), len(b), len(t))


class Aligner(object):
    def __init__(self):
        g = rospy.get_param
        self.x = float(g('~x', 5.0))
        self.sh = float(g('~sensor_h', 0.6972))
        self.sx = float(g('~sx', 1.2))
        self.period = float(g('~period', 3.0))
        self.pitch = float(g('~pitch', 2.818))   # camera looks UP by this
        self.topic = g('~cloud_topic',
                       '/zedx_front/zed_node/point_cloud/cloud_registered')
        self.last = rospy.Time(0)
        self.n = 0
        print('align_frame: station %.2f m, camera %.4f m above the floor' %
              (self.x, self.sh))
        print('move the frame and watch; nothing is being recorded.\n')
        rospy.Subscriber(self.topic, PointCloud2, self.cb, queue_size=1,
                         buff_size=2 ** 26)

    def cb(self, msg):
        now = rospy.Time.now()
        if (now - self.last).to_sec() < self.period:
            return
        self.last = now
        pts = np.array([p[:3] for p in pc2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)], dtype=np.float64)
        if len(pts) < 2000:
            print('  (only %d points - is the camera publishing?)' % len(pts))
            return
        # the search box, generous in x because the frame is being moved
        m = ((np.abs(pts[:, 0] - self.x) < self.sx) &
             (np.abs(pts[:, 1]) < 1.6) &
             (pts[:, 2] > -self.sh - 0.8) & (pts[:, 2] < -self.sh + 2.6))
        box = pts[m]
        if len(box) < 2000:
            print('  no frame in the box at x = %.2f m (%d points)'
                  % (self.x, len(box)))
            return
        got = clusters(box, self.sh, self.pitch)
        if got is None:
            print('  cannot separate three balls yet at x = %.2f m' % self.x)
            return
        (A, B, T), counts = got

        # --- the three corrections -------------------------------------------
        # A is the ball at smaller y. With the ROS convention (+y to the left of
        # the camera's forward axis) that is the RIGHT-hand ball as seen from
        # behind the robot, which is how it is reported below.
        centre_y = 0.5 * (A[1] + B[1])
        offset_mm = centre_y * 1000.0            # + means the frame is left

        dy = B[1] - A[1]                          # ~ the base, +ve
        dx = B[0] - A[0]                          # left ball minus right ball
        turn_deg = np.degrees(np.arctan2(dx, dy)) if dy > 0.3 else float('nan')
        turn_mm = dx * 1000.0

        # An upright frame does NOT put its apex at the same range as its base -
        # the camera looks up 2.818 deg, so the apex is expected to sit ~54 mm
        # FURTHER. Lean is the departure from that, not from zero.
        base_x = 0.5 * (A[0] + B[0])
        lean_mm = ((T[0] - base_x) - expected_apex_lead(self.pitch)) * 1000.0
        lean_deg = np.degrees(np.arctan2(lean_mm / 1000.0, RISE_M))

        self.n += 1
        ok = (abs(offset_mm) < OK_OFFSET_MM and abs(turn_deg) < OK_TURN_DEG
              and abs(lean_deg) < OK_LEAN_DEG)

        print('-- %3d --  %d/%d/%d pts   base %.3f m   apex lead %+.0f mm '
              '(upright wants %+.0f)'
              % (self.n, counts[0], counts[1], counts[2], dy,
                 (T[0] - base_x) * 1000.0,
                 expected_apex_lead(self.pitch) * 1000.0))
        print('   OFFSET %+7.1f mm   %s' % (
            offset_mm,
            'CENTRED' if abs(offset_mm) < OK_OFFSET_MM else
            ('slide the frame %.0f mm to your RIGHT' % abs(offset_mm)
             if offset_mm > 0 else
             'slide the frame %.0f mm to your LEFT' % abs(offset_mm))))
        print('   TURN   %+7.2f deg  (%+.0f mm)   %s' % (
            turn_deg, turn_mm,
            'SQUARE' if abs(turn_deg) < OK_TURN_DEG else
            ('pivot: bring the RIGHT ball %.0f mm CLOSER' % abs(turn_mm)
             if turn_mm < 0 else
             'pivot: bring the LEFT ball %.0f mm CLOSER' % abs(turn_mm))))
        print('   LEAN   %+7.2f deg  (%+.0f mm)   %s' % (
            lean_deg, lean_mm,
            'UPRIGHT' if abs(lean_deg) < OK_LEAN_DEG else
            ('tip the TOP AWAY from you by %.0f mm' % abs(lean_mm)
             if lean_mm < 0 else
             'tip the TOP TOWARD you by %.0f mm' % abs(lean_mm))))
        if ok:
            print('   >>> ALL THREE WITHIN TOLERANCE - hold it there <<<')
        print('')


def main():
    rospy.init_node('align_frame', anonymous=True)
    Aligner()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        sys.exit(0)
