#!/usr/bin/env python3
"""
route_player.py — drive a fixed, scripted route by publishing /cmd_vel,
servoed on WHEEL ODOMETRY ONLY, so every run of an A/B campaign gets the same
stimulus without human driving.

WHAT THIS IS AND IS NOT
    A stimulus generator for repeatable SLAM experiments. It deliberately
    reads NOTHING from the vision system under test: heading feedback comes
    exclusively from the platform's wheel-encoder odometry
    (/husky_velocity_controller/odom), which exists identically in every arm
    of the A/B. Vision errors therefore cannot steer the robot — the input to
    the experiment stays independent of the thing being measured.

    It is NOT the Phase-2 waypoint driver (sidewalk_navigation/
    waypoint_runner.py drives move_base goals on a map). Do not grow this
    script toward it.

WHY IMU HEADING + ENCODER DISTANCE (v1 and v2 both failed their shakedowns)
    v1 was pure open-loop: skid-steer arcs under-rotate by a friction-
    dependent amount, and the robot walked ~2 m wide into a parked car.
    v2 servoed headings on WHEEL odometry: in-place skid-steer turns slip so
    much that the encoders report a 180 the wheels never fully made — the
    odom frame itself rotates against the world with every turn, and the
    "corrected" track wandered 14 m off the road in three laps.
    v3 (this one) takes HEADING from the platform IMU (/imu/data — absolute
    orientation, immune to wheel slip, and a sensor the real Husky carries
    too) and DISTANCE from the wheel encoders. Both are platform sensors,
    identical in every A/B arm, and completely blind to the vision system
    under test.

ROUTE FILE — a YAML list of primitives, executed in order:
    - {forward: 9.0}          # metres, at --speed, holding the entry heading
    - {turn: 180}             # degrees CCW (negative = CW), in place
    Optional per-primitive overrides: speed (m/s), omega (rad/s).

USAGE
    rosrun sidewalk_sim route_player.py --route .../routes/mcity_loop.yaml
    Exits 0 at route end (after a zero twist). Progress is printed unbuffered.

OPTIONAL: --recover-on-stuck (2026-08-22, exploratory routes only)
    Off by default — every validated A/B campaign run (mcity_loop.yaml, N=16)
    used the behaviour above, unmodified, and that path stays exactly as-is.
    This flag exists because a shakedown of a new building-perimeter route
    found a real failure mode neither v1 nor v2's fixes covered: wheels can
    keep turning (so wheel-odom "distance travelled" keeps counting up)
    while the chassis is jammed against a kerb it can't climb - a forward()
    call built entirely on wheel-odom distance has no way to notice this,
    and in that shakedown it produced 762 tracking resets in ~150 s before
    the robot broke free at the wrong heading.
    When set, a SEPARATE method (_forward_with_recovery, forward() itself is
    untouched) drives toward an absolute target point instead: entry ground
    truth position + dist_m along the entry heading. It re-aims at that
    target continuously (not "hold the entry heading", the v3 default) and
    watches ground truth displacement as a stall trigger - if true
    displacement over a short window is far below what the commanded speed
    implies, that is not distance, it is wheel slip against an obstacle.
    Recovery is stop -> back up -> rotate to break contact -> resume aiming
    at the same original target, sweeping the rotation wider on repeated
    stalls in the same spot. First version of this backed up, turned, then
    kept driving straight in the new heading for the rest of the ORIGINAL
    wheel-odom distance with no idea where the road actually was - it broke
    contact fine but then sailed off toward a roundabout well outside the
    route until an external safety monitor killed it. Re-aiming at the
    actual target after every recovery is what fixes that.
    /ground_truth/odom is simulator-only - a real robot would need a
    bump/current-spike/IMU-jerk signal instead, this is the sim-convenient
    stand-in for the same idea, and it is why this mode is opt-in and never
    used by a validated campaign run.
"""
import argparse
import math
import sys
from collections import deque

import rospy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

PUB_HZ = 20.0
KP_HEADING = 1.2       # P gain: rad/s commanded per rad of heading error
MAX_CORR = 0.4         # cap on the heading correction, rad/s
TURN_TOL = math.radians(2.0)

STALL_WINDOW_S = 2.5     # how far back to look for real displacement
STALL_MIN_FRACTION = 0.3  # true displacement below this fraction of the
                          # commanded distance (speed * window) => stuck
STALL_MIN_FLOOR_M = 0.12  # ...but never demand less than this much slop
RECOVER_BACKUP_S = 3.0    # reverse duration, fixed-time not fixed-distance
                          # (wheel-odom distance is exactly what just lied)
RECOVER_BACKUP_SPEED = -0.3
RECOVER_MAX_ATTEMPTS = 40  # generous, not unlimited — a real dead end must
                           # still be reported, not spun on forever


def log(msg):
    print(msg, flush=True)          # flush: visible under nohup redirection
    rospy.loginfo(msg)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def ang_diff(a, b):
    """Shortest signed angle a-b."""
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


class RoutePlayer:
    def __init__(self, odom_topic, imu_topic, cmd_topic, gt_topic=None):
        self.pose = None          # wheel odom: (x, y) — DISTANCE duty only
        self.yaw = None           # IMU: absolute heading — HEADING duty only
        self.gt_pose = None        # ground truth (x, y) — STALL DETECTION ONLY
        self.pub = rospy.Publisher(cmd_topic, Twist, queue_size=1)
        rospy.Subscriber(odom_topic, Odometry, self._odom_cb, queue_size=5)
        rospy.Subscriber(imu_topic, Imu, self._imu_cb, queue_size=5)
        if gt_topic:
            rospy.Subscriber(gt_topic, Odometry, self._gt_cb, queue_size=5)

    def _odom_cb(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y)

    def _imu_cb(self, msg):
        self.yaw = yaw_of(msg.orientation)

    def _gt_cb(self, msg):
        p = msg.pose.pose
        self.gt_pose = (p.position.x, p.position.y)

    def wait_ready(self, timeout=30.0, need_gt=False):
        t0 = rospy.Time.now()
        while not rospy.is_shutdown():
            ok = (self.pose is not None and self.yaw is not None
                  and self.pub.get_num_connections() > 0)
            if need_gt:
                ok = ok and self.gt_pose is not None
            if ok:
                return True
            if (rospy.Time.now() - t0).to_sec() > timeout:
                return False
            rospy.sleep(0.2)
        return False

    def stop(self):
        for _ in range(3):
            self.pub.publish(Twist())
            rospy.sleep(0.05)

    def _drive_timed(self, linear, angular, duration_s):
        """Open-loop timed drive — used only for the recovery back-up, where
        the sensor that would normally measure distance (wheel odom) is
        exactly what is unreliable right after a stall."""
        rate = rospy.Rate(PUB_HZ)
        tw = Twist()
        t0 = rospy.Time.now()
        while (not rospy.is_shutdown()
               and (rospy.Time.now() - t0).to_sec() < duration_s):
            tw.linear.x = linear
            tw.angular.z = angular
            self.pub.publish(tw)
            rate.sleep()
        self.stop()

    def recover(self, attempt):
        """Back up, then rotate by a sweep angle to break contact with
        whatever is blocking forward motion. Does NOT try to pick the
        "correct" new heading — that is _forward_with_recovery's job right
        after this returns, by re-aiming at the actual target. This only
        needs to turn far enough to stop pushing against the same spot.
        Sweeps wider on repeated stalls: +/-45, +/-90, +/-135, +/-180..."""
        log(f"route_player: STUCK (attempt {attempt}) — backing up "
            f"{RECOVER_BACKUP_S:.1f}s, then turning to break contact")
        self._drive_timed(RECOVER_BACKUP_SPEED, 0.0, RECOVER_BACKUP_S)
        step = 45 * (((attempt - 1) // 2) + 1)
        sign = 1 if attempt % 2 == 1 else -1
        deg = sign * step
        log(f"route_player: recovery turn {deg:+d} deg")
        self.turn(math.radians(deg), 0.5)

    def forward(self, dist_m, speed):
        """Drive dist_m (wheel-odom arc length) holding the ENTRY heading
        (IMU). This is the validated v3 behaviour, byte-for-byte unchanged —
        every campaign run used exactly this. No stall detection here."""
        hdg = self.yaw
        px, py = self.pose
        travelled = 0.0
        rate = rospy.Rate(PUB_HZ)
        tw = Twist()
        while not rospy.is_shutdown() and travelled < dist_m:
            x, y = self.pose
            travelled += math.hypot(x - px, y - py)
            px, py = x, y
            tw.linear.x = speed
            tw.angular.z = max(-MAX_CORR,
                               min(MAX_CORR,
                                   KP_HEADING * ang_diff(hdg, self.yaw)))
            self.pub.publish(tw)
            rate.sleep()

    def _forward_with_recovery(self, dist_m, speed):
        """Exploratory-route-only alternative to forward(): drives toward an
        ABSOLUTE target point (start ground-truth position + dist_m along the
        entry heading) rather than holding a fixed heading for a fixed
        wheel-odom distance. Continuously re-aims at that target, which is
        what makes recovery actually work: after a stall breaks contact with
        an obstacle at some new, unplanned heading, the next step is to point
        back at where the segment was always trying to go, not to keep
        driving straight in whatever direction the recovery turn left it
        facing (which sent an earlier version of this off toward the
        roundabout, well outside the route entirely).
        Completion is ground-truth distance-to-target < 0.5 m, OR wheel-odom
        travelled exceeding 1.5x dist_m as a backstop in case ground truth
        itself misbehaves."""
        gx0, gy0 = self.gt_pose
        hdg0 = self.yaw
        target = (gx0 + dist_m * math.cos(hdg0), gy0 + dist_m * math.sin(hdg0))
        px, py = self.pose
        travelled = 0.0
        rate = rospy.Rate(PUB_HZ)
        tw = Twist()
        gt_hist = deque()  # (time_s, x, y), for the stall window
        attempts = 0
        while not rospy.is_shutdown():
            gx, gy = self.gt_pose
            remaining = math.hypot(target[0] - gx, target[1] - gy)
            if remaining < 0.5:
                break
            x, y = self.pose
            travelled += math.hypot(x - px, y - py)
            px, py = x, y
            if travelled > dist_m * 1.5:
                log("route_player: wheel-odom safety backstop hit "
                    "(1.5x planned distance) — ending segment early")
                break

            aim = math.atan2(target[1] - gy, target[0] - gx)
            tw.linear.x = speed
            tw.angular.z = max(-MAX_CORR,
                               min(MAX_CORR,
                                   KP_HEADING * ang_diff(aim, self.yaw)))
            self.pub.publish(tw)
            rate.sleep()

            now = rospy.Time.now().to_sec()
            gt_hist.append((now, gx, gy))
            while gt_hist and now - gt_hist[0][0] > STALL_WINDOW_S:
                gt_hist.popleft()
            if gt_hist and now - gt_hist[0][0] >= STALL_WINDOW_S * 0.9:
                t0, x0, y0 = gt_hist[0]
                real_disp = math.hypot(gx - x0, gy - y0)
                expected = speed * (now - t0)
                threshold = max(STALL_MIN_FLOOR_M, STALL_MIN_FRACTION * expected)
                if real_disp < threshold:
                    attempts += 1
                    if attempts > RECOVER_MAX_ATTEMPTS:
                        log(f"route_player: gave up after "
                            f"{RECOVER_MAX_ATTEMPTS} recovery attempts — "
                            f"target likely unreachable, continuing to next "
                            f"primitive")
                        return
                    self.recover(attempts)
                    px, py = self.pose
                    gt_hist.clear()

    def turn(self, delta_rad, omega):
        """Rotate in place by delta_rad (signed, CCW positive) from the
        entry heading, per the IMU — immune to skid-steer wheel slip."""
        target = self.yaw + delta_rad
        rate = rospy.Rate(PUB_HZ)
        tw = Twist()
        while not rospy.is_shutdown():
            err = ang_diff(target, self.yaw)
            if abs(err) < TURN_TOL:
                break
            w = max(0.15, min(omega, abs(err) * 1.5))
            tw.angular.z = w if err > 0 else -w
            self.pub.publish(tw)
            rate.sleep()


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--route", required=True, help="YAML route file")
    ap.add_argument("--cmd-topic", default="/cmd_vel")
    ap.add_argument("--odom-topic", default="/husky_velocity_controller/odom",
                    help="WHEEL odometry, distance duty only — never a vision "
                         "topic; the route must stay blind to the system "
                         "under test")
    ap.add_argument("--imu-topic", default="/imu/data",
                    help="platform IMU, heading duty only (absolute "
                         "orientation, immune to wheel slip)")
    ap.add_argument("--speed", type=float, default=0.6, help="m/s for forward")
    ap.add_argument("--omega", type=float, default=0.5, help="rad/s for turns")
    ap.add_argument("--recover-on-stuck", action="store_true",
                    help="exploratory routes only — see module docstring. "
                         "Off (unchanged v3 behaviour) for every validated "
                         "campaign route.")
    ap.add_argument("--gt-topic", default="/ground_truth/odom",
                    help="simulator-only ground truth, used ONLY as the "
                         "stall trigger when --recover-on-stuck is set")
    args = ap.parse_args(
        [a for a in sys.argv[1:] if ":=" not in a])

    with open(args.route) as fh:
        route = yaml.safe_load(fh)
    if not isinstance(route, list) or not route:
        log(f"route file {args.route} is not a non-empty list")
        return 2

    rospy.init_node("route_player", anonymous=True)
    rp = RoutePlayer(args.odom_topic, args.imu_topic, args.cmd_topic,
                     gt_topic=args.gt_topic if args.recover_on_stuck else None)
    if not rp.wait_ready(need_gt=args.recover_on_stuck):
        log("missing wheel odometry, IMU, ground truth (if --recover-on-stuck), "
            "or /cmd_vel subscriber after 30 s — sim up?")
        return 3

    total_fwd = sum(float(s["forward"]) for s in route if "forward" in s)
    log(f"route_player: {len(route)} primitive(s), {total_fwd:.0f} m forward total")

    try:
        for i, seg in enumerate(route, 1):
            if rospy.is_shutdown():
                break
            if "forward" in seg:
                d = float(seg["forward"])
                v = float(seg.get("speed", args.speed))
                log(f"route_player: {i}/{len(route)} forward {d:.1f} m @ {v:.2f} m/s")
                if args.recover_on_stuck:
                    rp._forward_with_recovery(d, v)
                else:
                    rp.forward(d, v)
            elif "turn" in seg:
                deg = float(seg["turn"])
                w = float(seg.get("omega", args.omega))
                log(f"route_player: {i}/{len(route)} turn {deg:+.0f} deg @ {w:.2f} rad/s")
                rp.turn(math.radians(deg), w)
            else:
                log(f"route_player: segment {i} has neither forward nor turn — skipped")
    finally:
        rp.stop()
    log("route_player: route complete, robot stopped")
    return 0


if __name__ == "__main__":
    sys.exit(main())
