#!/usr/bin/env python3
"""robot_battery_check.py — is there enough charge to finish the run?

WHY THIS EXISTS

    On 2026-08-31 run 7b was driven for two loops and the robot ran out of
    power partway through. The database ended with 2.48 m of odometry, eight
    graph links and NO loop closures, and the whole run had to be deleted. The
    software behaved correctly throughout; nobody had checked the charge.

    Plain terms: the robot ran out of battery mid-experiment and the run was
    wasted. Thirty seconds of checking beforehand would have saved twenty
    minutes of driving.

WHY IT READS /diagnostics AND NOT A HUSKY-SPECIFIC TOPIC

    The Husky publishes its own status message, but deserialising that needs
    husky_msgs installed wherever this runs - and this runs on the JETSON,
    which is a different machine from the robot and need not have the robot's
    message packages. /diagnostics is diagnostic_msgs/DiagnosticArray, which
    ships with ROS itself, so this works from anywhere that can reach the
    robot's master.

WHY IT TALKS TO A DIFFERENT MASTER

    The wheels, IMU and battery are on the ROBOT's ROS master
    (<robot-usb-address>:11311). The camera and mapping are on the JETSON's
    (localhost:11311). They are separate systems that happen to sit on one
    machine's network. This script points at the robot's.

VERIFIED against the live robot 2026-08-31. The Husky publishes:

    husky_node: power_status    "Charge (%)", "Battery Capacity (Wh)"
    husky_node: system_status   "Battery Voltage", plus motor driver voltages,
                                currents and temperatures
    husky_node: safety_status   "No battery", "Emergency Stop", "Lockout"

ONE MESSAGE IS NOT ENOUGH. /diagnostics is published by several nodes, each at
its own rate, and each message carries only that node's statuses. The first
version of this script read a single message, happened to catch the joystick
driver's, and reported "nothing mentions a battery" on a robot that was
publishing its charge perfectly well. It now accumulates for a few seconds.

    usage:
      rosrun sidewalk_bringup robot_battery_check.py
      rosrun sidewalk_bringup robot_battery_check.py _min_volts:=24.5
"""
import os
import re
import sys

MASTER = os.environ.get("ROBOT_MASTER_URI", "http://%s:11311" % os.environ.get("ROBOT_USB_ADDR", "robot"))

# A Husky A200 runs a 24 V lead-acid pack. Rough working figures - REPLACE with
# measured ones once the discharge curve has been observed on this robot:
#   ~28.8 V  charging / freshly off charge
#   ~25.5 V  comfortably usable
#   ~24.0 V  getting low, a long run is a gamble
#   ~22.0 V  cutoff territory; the robot will stop without warning
FULL_V, GOOD_V, LOW_V, CUTOFF_V = 28.8, 25.5, 24.0, 22.0


def main():
    os.environ["ROS_MASTER_URI"] = MASTER
    import rospy
    from diagnostic_msgs.msg import DiagnosticArray

    rospy.init_node("robot_battery_check", anonymous=True, disable_signals=True)
    min_v = float(rospy.get_param("~min_volts", GOOD_V))
    wait = float(rospy.get_param("~timeout", 20.0))

    print("  robot master: %s" % MASTER)
    collect = float(rospy.get_param("~collect", 8.0))

    # ACCUMULATE. Each node publishes its own statuses at its own rate, so a
    # single message shows one node and nothing else.
    seen = {}
    got_any = {"v": False}

    def cb(m):
        got_any["v"] = True
        for st in m.status:
            seen[st.name] = st

    rospy.Subscriber("/diagnostics", DiagnosticArray, cb)
    t0 = rospy.Time.now()
    while (rospy.Time.now() - t0).to_sec() < collect:
        rospy.sleep(0.2)
    if not got_any["v"]:
        print("  CANNOT REACH THE ROBOT: nothing on /diagnostics in %.0f s" % collect)
        print("  Is it powered on and off the charger? This check needs its ROS")
        print("  master at %s." % MASTER)
        sys.exit(2)
    print("  heard from %d diagnostic publishers in %.0f s" % (len(seen), collect))

    # Flag the safety states first - a robot that will not move is worse than
    # a flat one, and the reason is right here.
    saf = seen.get("husky_node: safety_status")
    if saf:
        bad = [kv.key for kv in saf.values
               if kv.key in ("Emergency Stop", "Lockout", "No battery")
               and str(kv.value).lower() in ("true", "1")]
        if bad:
            print("  ROBOT WILL NOT DRIVE: %s" % ", ".join(bad))

    found = []
    for st in seen.values():
        blob = (st.name + " " + st.message).lower()
        for kv in st.values:
            k = kv.key.lower()
            if any(w in k or w in blob for w in
                   ("batt", "volt", "charge", "capacity", "soc")):
                found.append((st.name, kv.key, kv.value))

    if not found:
        print("  /diagnostics is publishing, but nothing in it mentions a")
        print("  battery, voltage or charge. Status names seen:")
        for name in sorted(seen)[:12]:
            print("    %s" % name)
        print("  Pin the right topic down by hand, then update this script.")
        sys.exit(3)

    print("  what the robot reports:")
    volts = None
    pct = None
    for name, key, val in found:
        print("    %-38s %-22s %s" % (name[:38], key[:22], val))
        m = re.match(r"^\s*([0-9]+\.?[0-9]*)", str(val))
        if not m:
            continue
        v = float(m.group(1))
        kl = key.lower()
        if volts is None and ("volt" in kl) and 10.0 < v < 60.0:
            volts = v
        if pct is None and ("percent" in kl or "charge" in kl or "soc" in kl) \
                and 0.0 <= v <= 100.0:
            pct = v

    print()
    if volts is None and pct is None:
        print("  Found battery-ish entries but could not read a number from them.")
        print("  Look at the list above and set the key names in this script.")
        sys.exit(3)

    if volts is not None:
        frac = max(0.0, min(1.0, (volts - CUTOFF_V) / (FULL_V - CUTOFF_V)))
        print("  battery: %.2f V   (roughly %.0f %% of the usable range %.1f-%.1f V)"
              % (volts, 100 * frac, CUTOFF_V, FULL_V))
    if pct is not None:
        print("  battery: %.0f %% reported" % pct)

    # The verdict, in terms of what it means for a run rather than a bare number
    v = volts if volts is not None else None
    if v is None:
        ok = (pct is None) or (pct >= 40)
        print("  (no voltage available; judging on the reported percentage)")
    else:
        ok = v >= min_v

    print()
    if v is not None and v < LOW_V:
        print("  DO NOT START A RUN. %.2f V is below %.1f V, and the robot can stop"
              % (v, LOW_V))
        print("  without warning near %.1f V. That is what ended run 7b." % CUTOFF_V)
        sys.exit(1)
    if not ok:
        print("  MARGINAL. %.2f V is under the %.1f V asked for. A short run may"
              % (v, min_v))
        print("  finish; two loops probably will not. Charge, or shorten the plan.")
        sys.exit(1)
    print("  OK to run. Record this figure with the run - a result from a robot")
    print("  whose charge state nobody wrote down cannot be compared with another.")
    sys.exit(0)


if __name__ == "__main__":
    main()
