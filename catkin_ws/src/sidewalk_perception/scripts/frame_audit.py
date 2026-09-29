#!/usr/bin/env python3
"""
frame_audit.py — audit coordinate frame names and timestamps, offline or live.

WHY THIS EXISTS
    A previous tool in this project, `read_frame_id.py`, iterated a bag's
    CameraInfo topic and printed each message's timestamp and header.frame_id.
    That tiny script caught real bugs, because frame_id errors are silent: ROS
    will happily carry a message whose frame_id is wrong, empty, or duplicated
    across two physical cameras. Nothing crashes. The map just comes out wrong,
    and you lose a week finding out why.

    This is that tool, generalised. It answers four questions about a recording
    or a live stream:

      1. What frame_id does every topic actually use, and does it ever change
         mid-stream?
      2. Do the frame names follow this project's per-camera namespacing, so
         that the second (rear-facing) ZED X can be added without a collision?
      3. Are the timestamps monotonic, and how are the gaps distributed?
      4. Do topics that should be simultaneous actually carry the same stamp?

    It works on a bag file with no camera, no robot and no roscore, which is
    the normal situation when reviewing data after a drive.

WHY THE NAMESPACING QUESTION MATTERS SO MUCH HERE
    One ZED X exists today. A second, facing the opposite direction, arrives
    later. If both publish `zed_left_camera_optical_frame`, the transform tree
    gets two different physical meanings for one name. TF will resolve it to
    whichever was published most recently. The resulting map is not obviously
    broken — it is subtly, unfixably wrong. Enforcing the namespace now, while
    only one camera exists, costs one line of config.
"""
from __future__ import annotations

import argparse
import json
import sys
from collections import OrderedDict
from datetime import datetime
from pathlib import Path

from perception_common import (
    EXIT_FAIL, EXIT_INTERRUPTED, EXIT_PASS, REPO, RunLogger, Stats,
    Unavailable, _r, die, require_master, require_rospy, worst,
)

# Frame names the ZED wrapper produces by default, i.e. without a per-camera
# prefix. Seeing any of these bare is the collision hazard described above.
GENERIC_ZED_FRAMES = (
    "zed_camera_center", "zed_left_camera_frame", "zed_right_camera_frame",
    "zed_left_camera_optical_frame", "zed_right_camera_optical_frame",
    "zed_imu_link", "base_link", "camera_link", "map", "odom",
)

# Frames that are legitimately shared between cameras — these are robot-level
# or world-level, not sensor-level, so they must NOT be namespaced.
SHARED_FRAMES = ("map", "odom", "base_link", "base_footprint", "world")


class TopicRecord(object):
    """Everything we learn about one topic while walking it."""

    def __init__(self, name, msg_type=""):
        self.name = name
        self.msg_type = msg_type
        self.count = 0
        self.frame_ids = OrderedDict()
        self.first_stamp = None
        self.last_stamp = None
        self.prev_stamp = None
        self.backwards = []      # (index, delta) for the first few offenders
        self.zero_stamps = 0
        self.gaps = Stats("gap")
        self.seq_missing = 0
        self._last_seq = None

    def observe(self, header, index):
        self.count += 1
        fid = header.frame_id
        self.frame_ids[fid] = self.frame_ids.get(fid, 0) + 1
        t = header.stamp.to_sec()
        if t == 0.0:
            self.zero_stamps += 1
        if self.first_stamp is None:
            self.first_stamp = t
        else:
            dt = t - self.prev_stamp
            if dt < 0 and len(self.backwards) < 10:
                self.backwards.append((index, dt))
            elif dt > 0:
                self.gaps.add(dt)
        self.prev_stamp = t
        self.last_stamp = t
        seq = getattr(header, "seq", None)
        if seq is not None:
            if self._last_seq is not None and seq > self._last_seq + 1:
                self.seq_missing += seq - self._last_seq - 1
            self._last_seq = seq

    @property
    def hz(self):
        if self.count < 2 or self.first_stamp is None:
            return float("nan")
        span = self.last_stamp - self.first_stamp
        return (self.count - 1) / span if span > 0 else float("nan")

    def as_dict(self):
        return {
            "topic": self.name,
            "type": self.msg_type,
            "messages": self.count,
            "frame_ids": dict(self.frame_ids),
            "first_stamp": _r(self.first_stamp, 6),
            "last_stamp": _r(self.last_stamp, 6),
            "duration_s": _r((self.last_stamp - self.first_stamp)
                             if self.first_stamp is not None else 0, 3),
            "rate_hz": _r(self.hz, 3),
            "backwards_jumps": len(self.backwards),
            "zero_stamps": self.zero_stamps,
            "frames_missing_by_seq": self.seq_missing,
            "gap_s": self.gaps.as_dict("s"),
        }


# --------------------------------------------------------------------------- #
# Sources: a bag file, or a live stream
# --------------------------------------------------------------------------- #

def audit_bag(path, args, log):
    """Walk a bag file and record every stamped message.

    `rosbag` is importable without a running master, which is the whole point:
    this must work at a desk, days after the drive.
    """
    p = Path(path)
    if not p.exists():
        raise Unavailable(
            "Bag file does not exist: %s" % p,
            "Check the path. Recordings normally live under data/ — try "
            "`ls -la %s`." % (p.parent if str(p.parent) not in ("", ".") else REPO / "data"))
    if p.stat().st_size == 0:
        raise Unavailable(
            "Bag file %s is zero bytes — the recording captured nothing." % p,
            "This usually means `rosbag record` was started before the camera "
            "driver was publishing. Start the driver, confirm with "
            "`rostopic hz`, and only then start recording.")

    try:
        import rosbag
    except ImportError:
        raise Unavailable(
            "The `rosbag` Python module is not available, so bag files cannot "
            "be opened on this machine.",
            "Run this on the Jetson after sourcing ROS, or install "
            "`python3-rosbag`. No roscore is needed — only the library.")

    try:
        bag = rosbag.Bag(str(p), "r")
    except Exception as exc:
        raise Unavailable(
            "Could not open %s as a rosbag (%s: %s). It may be truncated — a "
            "recording killed with SIGKILL rather than Ctrl-C leaves an "
            "unindexed bag." % (p, type(exc).__name__, exc),
            "Try `rosbag reindex %s`, then `rosbag check %s`." % (p, p))

    records = OrderedDict()
    try:
        info = bag.get_type_and_topic_info()
        log.info("Bag: %s (%.1f MB)" % (p, p.stat().st_size / 1e6))
        log.info("Topics in bag: %d" % len(info.topics))

        selected = _select_topics(list(info.topics), args)
        if not selected:
            raise Unavailable(
                "None of the bag's %d topics matched the filter. Available: %s"
                % (len(info.topics), ", ".join(sorted(info.topics))),
                "Drop --topics to audit everything, or pass a substring that "
                "appears in the names above.")
        for t in selected:
            records[t] = TopicRecord(t, info.topics[t].msg_type)

        log.info("Auditing %d topic(s): %s" % (len(selected), ", ".join(selected)))
        index = 0
        skipped_unstamped = 0
        for topic, msg, _bag_t in bag.read_messages(topics=selected):
            index += 1
            header = getattr(msg, "header", None)
            if header is None:
                skipped_unstamped += 1
                continue
            records[topic].observe(header, index)
            if args.verbose and records[topic].count <= args.head:
                # This is the original read_frame_id.py behaviour, preserved:
                # print timestamp and frame_id, one line per message.
                log.info("  %-46s %.9f  %s"
                         % (topic, header.stamp.to_sec(), header.frame_id))
        if skipped_unstamped:
            log.warn("%d messages had no header and were skipped. Messages "
                     "without a header cannot be time-aligned with anything "
                     "else, which makes them useless for SLAM."
                     % skipped_unstamped)
    finally:
        bag.close()
    return records


def audit_live(args, log):
    """Sample live topics for a fixed window.

    Much less thorough than a bag audit — it only sees what arrives during the
    window — but useful for confirming frame names immediately after bringup,
    before committing to a long recording.
    """
    import time
    rospy = require_rospy()
    require_master(rospy)
    rospy.init_node("frame_audit", anonymous=True, disable_signals=True)

    published = [t for t, _ in rospy.get_published_topics()]
    selected = _select_topics(published, args)
    if not selected:
        raise Unavailable(
            "No published topic matched the filter. Currently published: %s"
            % (", ".join(sorted(published)[:40]) or "nothing at all"),
            "If that list is empty, no driver is running — start "
            "`roslaunch sidewalk_perception zedx_front.launch` first.")

    records = OrderedDict((t, TopicRecord(t)) for t in selected)
    subs = []
    counter = {"i": 0}

    # AnyMsg lets us subscribe without knowing the type, but it delivers raw
    # bytes. The header is the first field of every stamped message, so we
    # deserialise just enough of it: seq (uint32), stamp (2x uint32), then a
    # length-prefixed frame_id string. This avoids importing every possible
    # message type, and works on custom types too.
    import struct
    from rospy import AnyMsg

    class _H(object):
        __slots__ = ("seq", "stamp", "frame_id")

    class _T(object):
        __slots__ = ("to_sec_v",)

        def __init__(self, v):
            self.to_sec_v = v

        def to_sec(self):
            return self.to_sec_v

    def on_any(msg, topic):
        buf = msg._buff
        if len(buf) < 12:
            return
        seq, secs, nsecs = struct.unpack_from("<III", buf, 0)
        (slen,) = struct.unpack_from("<I", buf, 12)
        if 16 + slen > len(buf) or slen > 512:
            return  # not a stamped message; the leading bytes meant something else
        frame_id = buf[16:16 + slen].decode("utf-8", "replace")
        h = _H()
        h.seq, h.stamp, h.frame_id = seq, _T(secs + nsecs * 1e-9), frame_id
        counter["i"] += 1
        records[topic].observe(h, counter["i"])

    for t in selected:
        subs.append(rospy.Subscriber(t, AnyMsg, on_any, callback_args=t,
                                     queue_size=50))
    log.info("Sampling %d topic(s) for %.0f s..." % (len(selected), args.duration))
    end = time.time() + args.duration
    try:
        while time.time() < end and not rospy.is_shutdown():
            time.sleep(0.2)
    except KeyboardInterrupt:
        log.warn("Interrupted — analysing what was collected.")
    for s in subs:
        s.unregister()
    return records


def _select_topics(available, args):
    if args.topics:
        wanted = [w.strip() for w in args.topics.split(",") if w.strip()]
        return sorted(t for t in available if any(w in t for w in wanted))
    if args.all:
        return sorted(available)
    # Default: the sensor topics whose frames actually matter for geometry.
    keys = ("camera_info", "image", "depth", "imu", "point_cloud", "cloud")
    hits = sorted(t for t in available if any(k in t.lower() for k in keys))
    return hits or sorted(available)


# --------------------------------------------------------------------------- #
# Checks
# --------------------------------------------------------------------------- #

def evaluate(records, args):
    """Turn the raw per-topic records into verdicts."""
    checks = []

    def add(name, status, detail, value=None):
        checks.append({"check": name, "status": status, "detail": detail,
                       "value": value})

    all_frames = OrderedDict()
    for rec in records.values():
        for fid, n in rec.frame_ids.items():
            all_frames.setdefault(fid, []).append((rec.name, n))

    for rec in records.values():
        if rec.count == 0:
            add("%s: data present" % rec.name, "WARN",
                "No messages on this topic.")
            continue

        # --- frame_id stability
        if len(rec.frame_ids) > 1:
            add("%s: frame_id stability" % rec.name, "FAIL",
                "frame_id changed mid-stream: %s. A topic must keep one "
                "frame_id for its entire life; anything else means two "
                "publishers are fighting over the same topic name."
                % ", ".join("%s (x%d)" % kv for kv in rec.frame_ids.items()),
                list(rec.frame_ids))
        else:
            fid = list(rec.frame_ids)[0]
            if not fid:
                add("%s: frame_id present" % rec.name, "FAIL",
                    "frame_id is empty. Any consumer doing geometry with this "
                    "topic has no idea what the coordinates are relative to.",
                    "")
            else:
                add("%s: frame_id" % rec.name, "PASS",
                    "`%s`, stable across all %d messages" % (fid, rec.count), fid)

        # --- monotonicity
        if rec.backwards:
            worst_jump = min(d for _, d in rec.backwards)
            add("%s: timestamp monotonicity" % rec.name, "FAIL",
                "time went backwards %d time(s), worst by %.3f ms. RTAB-Map "
                "silently discards out-of-order data, so this shows up as "
                "'mapping stopped working' with no error message."
                % (len(rec.backwards), worst_jump * 1e3),
                len(rec.backwards))
        elif rec.zero_stamps:
            add("%s: timestamp validity" % rec.name, "FAIL",
                "%d messages had a zero timestamp — the publisher never set "
                "header.stamp." % rec.zero_stamps, rec.zero_stamps)
        else:
            add("%s: timestamp monotonicity" % rec.name, "PASS",
                "all %d stamps strictly increasing over %.1f s"
                % (rec.count, (rec.last_stamp or 0) - (rec.first_stamp or 0)),
                rec.count)

        # --- gap distribution
        if rec.gaps.n > 10:
            med = rec.gaps.pct(50)
            mx = rec.gaps.max
            ratio = mx / med if med > 0 else float("inf")
            status = "PASS" if ratio < 2.0 else ("WARN" if ratio < 5.0 else "FAIL")
            add("%s: gaps" % rec.name, status,
                "median interval %.2f ms, worst %.2f ms (%.1fx median), "
                "p99 %.2f ms; %d frames missing by sequence number"
                % (med * 1e3, mx * 1e3, ratio, rec.gaps.pct(99) * 1e3,
                   rec.seq_missing),
                _r(ratio, 2))

    # --- namespacing across the whole recording
    for fid, users in all_frames.items():
        if not fid:
            continue
        base = fid.lstrip("/")
        if base in SHARED_FRAMES:
            continue
        if base in GENERIC_ZED_FRAMES or (base.startswith("zed_")
                                          and args.camera_prefix not in base):
            add("frame namespacing: %s" % fid, "WARN",
                "`%s` is a generic ZED frame name with no per-camera prefix, "
                "used by %s. A second ZED X facing the opposite direction is "
                "planned; if it publishes this same name, the transform tree "
                "will hold two different physical meanings for one frame and "
                "the resulting map will be wrong with no error shown. Set "
                "`general/camera_name` in config/zedx_front.yaml so frames "
                "come out as e.g. `%s_left_camera_optical_frame`."
                % (fid, ", ".join(u for u, _ in users), args.camera_prefix),
                fid)
        elif args.camera_prefix and args.camera_prefix not in base:
            add("frame namespacing: %s" % fid, "WARN",
                "`%s` does not contain the expected camera prefix `%s`."
                % (fid, args.camera_prefix), fid)
        else:
            add("frame namespacing: %s" % fid, "PASS",
                "`%s` is namespaced correctly (used by %d topic(s))"
                % (fid, len(users)), fid)

    # --- optical frame convention
    for rec in records.values():
        if "image" not in rec.name.lower() or not rec.frame_ids:
            continue
        fid = list(rec.frame_ids)[0]
        if fid and "optical" not in fid:
            add("optical convention: %s" % rec.name, "WARN",
                "image topic uses frame `%s`, which is not marked optical. "
                "Image coordinates are Z-forward/X-right/Y-down; the robot "
                "body convention is X-forward/Y-left/Z-up. Publishing image "
                "data in a body frame rotates the whole reconstruction by 90 "
                "degrees." % fid, fid)

    # --- cross-topic simultaneity
    checks.extend(_check_pairs(records))

    # --- single time base across the recording
    spans = [(r.name, r.first_stamp, r.last_stamp)
             for r in records.values() if r.first_stamp is not None]
    if len(spans) > 1:
        starts = [s for _, s, _ in spans]
        spread = max(starts) - min(starts)
        status = "PASS" if spread < 5.0 else ("WARN" if spread < 60.0 else "FAIL")
        add("single time base", status,
            "topic start times span %.3f s. A spread of a few seconds is just "
            "staggered startup; a spread of minutes, or an offset near a whole "
            "number of hours, means two clocks (check NTP and time zones on "
            "every machine involved)." % spread, _r(spread, 3))

    verdict = worst(*[c["status"] for c in checks]) if checks else "FAIL"
    return verdict, checks


def _check_pairs(records):
    """Check that topics which should be simultaneous carry the same stamp.

    Left and right images of a stereo pair must share a timestamp exactly. If
    they do not, disparity is being computed between two different instants,
    and while the robot is moving that is a real geometric error: at 1.5 m/s,
    a 10 ms skew shifts the scene 15 mm between the two views, which at 10 m
    range is a larger error than the camera's own depth precision.

    This also anticipates the two-camera future. The ZED Link Quad card holds
    two MAX96712 deserializers, and they are NOT synchronised to each other by
    default — so a camera on group A and a camera on group B can free-run
    relative to one another even though each is internally consistent.
    """
    out = []
    names = list(records)

    def find(*subs):
        for n in names:
            low = n.lower()
            if all(s in low for s in subs):
                return records[n]
        return None

    pairs = [
        ("stereo image pair", find("left", "image"), find("right", "image")),
        ("stereo camera_info pair", find("left", "camera_info"),
         find("right", "camera_info")),
        ("left image vs depth", find("left", "image"), find("depth")),
    ]
    for label, a, b in pairs:
        if not a or not b or a is b or not a.count or not b.count:
            continue
        d_first = abs((a.first_stamp or 0) - (b.first_stamp or 0))
        d_last = abs((a.last_stamp or 0) - (b.last_stamp or 0))
        skew = max(d_first, d_last)
        if a.count != b.count:
            out.append({
                "check": "%s: message counts" % label, "status": "WARN",
                "detail": ("%s has %d messages, %s has %d — a difference of %d. "
                           "The two halves of a stereo pair should arrive in "
                           "lockstep."
                           % (a.name, a.count, b.name, b.count,
                              abs(a.count - b.count))),
                "value": abs(a.count - b.count)})
        status = "PASS" if skew < 0.001 else ("WARN" if skew < 0.010 else "FAIL")
        out.append({
            "check": "%s: simultaneity" % label, "status": status,
            "detail": ("stamps differ by up to %.3f ms. At a walking pace of "
                       "1.5 m/s that corresponds to %.1f mm of scene motion "
                       "between the two views."
                       % (skew * 1e3, skew * 1.5 * 1000.0)),
            "value": _r(skew * 1e3, 4)})
    return out


# --------------------------------------------------------------------------- #

def report(records, verdict, checks, args, log):
    log.section("Per-topic summary")
    for rec in records.values():
        fids = ", ".join(rec.frame_ids) or "(none)"
        log.info("%-46s %7d msg  %8.2f Hz  frame_id: %s"
                 % (rec.name, rec.count, rec.hz if rec.count > 1 else 0.0, fids))

    log.section("Checks")
    for c in checks:
        line = "%-6s %s — %s" % (c["status"], c["check"], c["detail"])
        if c["status"] == "FAIL":
            log.error(line)
        elif c["status"] == "WARN":
            log.warn(line)
        else:
            log.info(line)

    log.section("Metrics")
    for rec in records.values():
        if rec.count:
            log.metric("%s.messages" % rec.name, rec.count)
            log.metric("%s.rate_hz" % rec.name, _r(rec.hz, 3), "Hz")

    payload = {
        "generated_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "source": args.bag if args.bag else "live",
        "camera_prefix": args.camera_prefix,
        "verdict": verdict,
        "checks": checks,
        "topics": {n: r.as_dict() for n, r in records.items()},
    }
    out_dir = REPO / "logs" / "sidewalk_perception"
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / ("%s_frame_audit.json" % datetime.now().strftime("%Y%m%d-%H%M%S"))
    path.write_text(json.dumps(payload, indent=2, default=str))
    log.info("Machine-readable audit: %s" % path)

    n_fail = sum(1 for c in checks if c["status"] == "FAIL")
    n_warn = sum(1 for c in checks if c["status"] == "WARN")
    log.summary("Frame audit %s on %s — %d topics, %d checks, %d failed, %d warned."
                % (verdict, args.bag or "live stream", len(records),
                   len(checks), n_fail, n_warn), status=verdict)
    return payload


def build_parser():
    p = argparse.ArgumentParser(
        description="Audit frame_ids and timestamps in a bag file or a live "
                    "ROS stream.")
    src = p.add_mutually_exclusive_group()
    src.add_argument("--bag", help="Path to a .bag file to audit offline")
    src.add_argument("--live", action="store_true",
                     help="Sample live topics instead (needs a running master)")
    p.add_argument("--topics", default="",
                   help="Comma-separated substrings; only matching topics are "
                        "audited. Default: sensor topics only.")
    p.add_argument("--all", action="store_true",
                   help="Audit every topic, not just sensor topics")
    p.add_argument("--camera-prefix", default="zedx_front",
                   help="Prefix every sensor frame_id should contain "
                        "(default: zedx_front)")
    p.add_argument("--duration", type=float, default=20.0,
                   help="Live sampling window in seconds")
    p.add_argument("--verbose", action="store_true",
                   help="Print timestamp and frame_id for the first messages "
                        "of each topic — the original read_frame_id.py output")
    p.add_argument("--head", type=int, default=10,
                   help="How many messages per topic --verbose prints")
    p.add_argument("--run-name", default="frame_audit")
    return p


def main(argv=None):
    argv = [a for a in (sys.argv[1:] if argv is None else argv)
            if not a.startswith("__") and ":=" not in a]
    args = build_parser().parse_args(argv)

    if not args.bag and not args.live:
        print("Specify either --bag <file> (offline, no ROS master needed) or "
              "--live (samples running topics).\n", file=sys.stderr)
        build_parser().print_help(sys.stderr)
        return EXIT_FAIL

    with RunLogger("sidewalk_perception", run_name=args.run_name) as log:
        try:
            records = audit_bag(args.bag, args, log) if args.bag else audit_live(args, log)
            if not records:
                raise Unavailable("Nothing to audit — no topics were recorded.",
                                  "Check the source contains stamped messages.")
            verdict, checks = evaluate(records, args)
            report(records, verdict, checks, args, log)
            return EXIT_PASS if verdict != "FAIL" else EXIT_FAIL
        except Unavailable as exc:
            return die(log, exc)
        except KeyboardInterrupt:
            log.warn("Interrupted by operator.")
            log.summary("Interrupted", status="WARN")
            return EXIT_INTERRUPTED


if __name__ == "__main__":
    sys.exit(main())
