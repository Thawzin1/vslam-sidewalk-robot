#!/usr/bin/env python3
"""
sensor_validator.py — measure whether the ZED X data stream is trustworthy.

WHY THIS EXISTS
    Every later phase of this project compares three SLAM systems (RTAB-Map,
    the ZED SDK's own tracker, and ORB-SLAM3) on identical recorded input. That
    comparison is only meaningful if the input is sound. If the camera silently
    drops one frame in fifty, or timestamps arrive from two different clocks,
    all three systems degrade — and they degrade by *different* amounts, which
    would look exactly like a genuine algorithmic result. We would publish a
    conclusion about SLAM algorithms that was actually a conclusion about a
    loose FAKRA connector.

    So this node measures the stream before anyone is allowed to trust it, and
    produces a number rather than an impression.

WHAT IT MEASURES
    1. Achieved frame rate versus configured frame rate, per topic.
    2. Dropped frames, two independent ways: exactly, from gaps in the ROS
       header sequence counter, and by inference from inter-arrival gaps. The
       two should agree. When they do not, that itself is informative — see the
       failure modes in EXPLAIN.html.
    3. Timestamp monotonicity: does time ever go backwards? A single backwards
       stamp will make RTAB-Map reject data with no obvious error.
    4. Inter-frame jitter: spread of the interval between consecutive frames.
       The mean is nearly useless here; the tail is what breaks SLAM.
    5. Time-base consistency: are the left image, right image, depth image and
       IMU all stamped from the same clock? The ZED Link Quad card carries two
       MAX96712 deserializers which are NOT synchronised to each other by
       default, so this check becomes critical the moment the second camera
       arrives.
    6. IMU achieved rate against the 200 Hz datasheet nominal. (Read 400 Hz
       until 2026-08-29. That was the ZED 2 figure; the ZED X datasheet says
       200 Hz in two places. With a 5% tolerance, a healthy camera delivering
       its rated rate would have been scored 50% low and FAILED — this check
       was condemning correct hardware.)
    7. frame_id correctness on every topic, against the per-camera namespace.
    8. Depth validity: what fraction of pixels are NaN, infinite, zero, or
       outside the camera's rated 1-35 m window.

HOW TO READ THE RESULT
    Exit 0  PASS         — the stream met every threshold
    Exit 1  FAIL         — it ran, and the data did not meet spec
    Exit 2  UNAVAILABLE  — it could not run at all (and it will say why)
    Exit 3  INTERRUPTED  — you pressed Ctrl-C (partial results are still saved)
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from datetime import datetime
from pathlib import Path

from perception_common import (
    EXIT_FAIL, EXIT_INTERRUPTED, EXIT_PASS, REPO, ZEDX,
    RunLogger, Stats, Unavailable, _r, die, depth_uncertainty_m, fmt_hz,
    focal_px_from_fov, load_simple_yaml, probe_environment, require_master,
    require_numpy, require_rospy, worst,
)

DEFAULT_CONFIG = Path(__file__).resolve().parent.parent / "config" / "validation_thresholds.yaml"


# --------------------------------------------------------------------------- #
# Per-topic measurement
# --------------------------------------------------------------------------- #

class TopicMonitor(object):
    """Accumulates timing statistics for one topic.

    One instance per topic. Deliberately does almost no work in the callback:
    at 1920x1200 and 60 FPS across six topics the callback runs several hundred
    times a second, and anything expensive there would distort the very
    intervals we are trying to measure. The observer must not perturb the
    observed.
    """

    def __init__(self, name, expected_hz, kind="generic"):
        self.name = name
        self.expected_hz = float(expected_hz) if expected_hz else None
        self.kind = kind

        self.count = 0
        self.first_stamp = None
        self.last_stamp = None
        self.first_recv = None
        self.last_recv = None

        # Interval between successive *header* stamps: the camera's own view of
        # time. This is the number that matters for SLAM, because SLAM uses the
        # header stamp, not the arrival time.
        self.stamp_interval = Stats("stamp_interval")
        # Interval between successive *arrivals* at this node. Compare against
        # the above to separate a camera problem from a transport problem.
        self.recv_interval = Stats("recv_interval")
        # header stamp minus arrival time — the pipeline latency, and also the
        # detector for "these stamps came from a different clock".
        self.latency = Stats("latency")

        self.backwards = 0          # timestamps that went backwards
        self.duplicate_stamps = 0   # two messages claiming the same instant
        self.zero_stamps = 0        # unstamped messages — a driver bug
        self.seq_gaps = 0           # number of discontinuities in header.seq
        self.seq_missing = 0        # total messages missing, summed over gaps
        self._last_seq = None
        self.frame_ids = {}         # frame_id -> count, to catch mid-run changes

    def observe(self, header, recv_time):
        self.count += 1
        stamp = header.stamp.to_sec()

        if stamp == 0.0:
            self.zero_stamps += 1

        fid = header.frame_id
        self.frame_ids[fid] = self.frame_ids.get(fid, 0) + 1

        if self.first_stamp is None:
            self.first_stamp = stamp
            self.first_recv = recv_time
        else:
            dt = stamp - self.last_stamp
            if dt < 0:
                self.backwards += 1
            elif dt == 0:
                self.duplicate_stamps += 1
            else:
                self.stamp_interval.add(dt)
            dr = recv_time - self.last_recv
            if dr > 0:
                self.recv_interval.add(dr)

        self.latency.add(recv_time - stamp)
        self.last_stamp = stamp
        self.last_recv = recv_time

        # header.seq is assigned by the publisher and increments by exactly one
        # per published message. A gap is therefore an *exact* count of frames
        # the publisher never sent us — far better evidence than inferring
        # drops from timing. (ROS 1 only; ROS 2 removed seq, which is one
        # reason this diagnosis is harder there.)
        seq = getattr(header, "seq", None)
        if seq is not None:
            if self._last_seq is not None and seq > self._last_seq + 1:
                self.seq_gaps += 1
                self.seq_missing += seq - self._last_seq - 1
            self._last_seq = seq

    # ------------------------------------------------------------------ views
    @property
    def achieved_hz(self):
        """Rate computed from header stamps over the whole window.

        Using (n-1)/(last-first) rather than averaging per-frame rates: the
        latter over-weights the short intervals and flatters a stream that
        stutters.
        """
        if self.count < 2 or self.first_stamp is None:
            return float("nan")
        span = self.last_stamp - self.first_stamp
        return (self.count - 1) / span if span > 0 else float("nan")

    @property
    def wallclock_hz(self):
        if self.count < 2 or self.first_recv is None:
            return float("nan")
        span = self.last_recv - self.first_recv
        return (self.count - 1) / span if span > 0 else float("nan")

    def inferred_drops(self):
        """Estimate dropped frames from interval gaps alone.

        If frames should arrive every T seconds and one interval is 3.1 T, then
        two frames went missing. This is the only estimate available when the
        publisher does not maintain header.seq, so it is worth computing even
        though the sequence counter is more reliable.

        The threshold is adaptive, and that matters. A fixed "count anything
        over 1.5 T" rule collapses on a jittery stream: with a 33 ms nominal
        period and 11 ms of jitter, ordinary intervals routinely exceed 50 ms
        and every one of them is scored as a phantom drop. Measured against a
        synthetic stream with 9 real drops, the fixed rule reported 98.

        So a gap must be too large to be explained by the jitter actually
        observed — at least 1.5 nominal periods AND at least four standard
        deviations above nominal — before it counts.

        That widening is CAPPED, and the cap is not cosmetic. A single dropped
        frame produces a gap of exactly 2 T. If the jitter term were allowed to
        push the threshold past 2 T, single-frame drops would become
        mathematically undetectable and this method would confidently return
        zero on a stream that really was dropping frames. That happens as soon
        as sigma exceeds T/4, which is an ordinary amount of jitter — at 30 FPS
        it is only 8.3 ms. So the threshold is clamped below 2 T, and when the
        clamp binds, `drop_inference_reliable()` returns False so the caller can
        say the estimate is jitter-limited instead of reporting a bare zero.
        """
        if not self.expected_hz or not self.stamp_interval.samples:
            return 0
        t = 1.0 / self.expected_hz
        if t <= 0:
            return 0
        threshold = self._drop_threshold(t)
        drops = 0
        for dt in self.stamp_interval.samples:
            if dt > threshold:
                n = int(round(dt / t)) - 1
                if n > 0:
                    drops += n
        return drops

    # Upper clamp on the jitter-widened threshold, as a multiple of the nominal
    # period. Must stay strictly below 2.0 or a single dropped frame (a gap of
    # exactly 2 T) can never be counted. 1.75 leaves a quarter-period of margin
    # on each side of the 2 T signature.
    DROP_THRESHOLD_CAP = 1.75

    def _drop_threshold(self, t):
        return min(max(1.5 * t, t + 4.0 * self.stamp_interval.stdev),
                   self.DROP_THRESHOLD_CAP * t)

    def drop_inference_reliable(self):
        """False when jitter is too large for gap-timing to resolve drops.

        When 4 sigma of the measured jitter exceeds the clamp, ordinary late
        frames are indistinguishable from real drops on timing alone. The
        sequence counter remains authoritative; only this estimate degrades.
        """
        if not self.expected_hz or not self.stamp_interval.samples:
            return False
        t = 1.0 / self.expected_hz
        if t <= 0:
            return False
        return (t + 4.0 * self.stamp_interval.stdev) <= self.DROP_THRESHOLD_CAP * t

    def as_dict(self):
        return {
            "topic": self.name,
            "kind": self.kind,
            "messages": self.count,
            "expected_hz": self.expected_hz,
            "achieved_hz_from_stamps": _r(self.achieved_hz, 4),
            "achieved_hz_from_arrival": _r(self.wallclock_hz, 4),
            "frame_ids": self.frame_ids,
            "timestamps_backwards": self.backwards,
            "timestamps_duplicated": self.duplicate_stamps,
            "timestamps_zero": self.zero_stamps,
            "seq_gaps": self.seq_gaps,
            "frames_missing_by_seq": self.seq_missing,
            "frames_missing_inferred_from_gaps": self.inferred_drops(),
            "drop_inference_reliable": self.drop_inference_reliable(),
            "stamp_interval_s": self.stamp_interval.as_dict("s"),
            "arrival_interval_s": self.recv_interval.as_dict("s"),
            "latency_s": self.latency.as_dict("s"),
        }


class DepthMonitor(object):
    """Measures how much of the depth image is actually usable.

    A ZED X depth image is dense in the sense that every pixel has a slot, but
    a large fraction of those slots routinely hold NaN — the stereo matcher
    could not find a correspondence there. Featureless asphalt, a blank wall,
    sky, direct sun, and anything closer than the 1 m minimum all produce
    invalid pixels. Knowing the fraction, and where it sits, tells you whether
    a mapping failure was the algorithm's fault or the scene's.

    Sampled rather than exhaustive: decoding every pixel of every frame at 60
    FPS in Python would fall behind and skew the timing measurements this node
    exists to make. One frame per second is ample for a distribution.
    """

    def __init__(self, np, sample_every_s=1.0):
        self.np = np
        self.sample_every_s = sample_every_s
        self._last_sample = 0.0
        self.frames_sampled = 0
        self.valid_pct = Stats("valid_pct")
        self.near_pct = Stats("near_pct")     # closer than the rated minimum
        self.far_pct = Stats("far_pct")       # beyond the rated maximum
        self.median_depth = Stats("median_depth")
        self.encoding = None
        self.size = None
        self.decode_errors = 0

    def maybe_sample(self, msg, now):
        if now - self._last_sample < self.sample_every_s:
            return
        self._last_sample = now
        np = self.np
        self.encoding = msg.encoding
        self.size = (msg.width, msg.height)
        try:
            if msg.encoding == "32FC1":
                # ZED SDK default: metres, float32, NaN where unmatched.
                dt = np.dtype(np.float32)
                dt = dt.newbyteorder(">") if msg.is_bigendian else dt
                arr = np.frombuffer(msg.data, dtype=dt)
                arr = arr.reshape(msg.height, msg.step // 4)[:, :msg.width]
                depth = arr.astype(np.float64)
            elif msg.encoding == "16UC1":
                # Millimetres, integer, 0 where unmatched.
                dt = np.dtype(np.uint16)
                dt = dt.newbyteorder(">") if msg.is_bigendian else dt
                arr = np.frombuffer(msg.data, dtype=dt)
                arr = arr.reshape(msg.height, msg.step // 2)[:, :msg.width]
                depth = arr.astype(np.float64) / 1000.0
                depth[depth == 0.0] = np.nan  # normalise "no data" to NaN
            else:
                self.decode_errors += 1
                return
        except Exception:
            self.decode_errors += 1
            return

        total = depth.size
        if not total:
            return
        finite = np.isfinite(depth)
        positive = finite & (depth > 0.0)
        valid = positive & (depth >= ZEDX["depth_min_m"]) & (depth <= ZEDX["depth_max_m"])

        self.frames_sampled += 1
        self.valid_pct.add(100.0 * int(np.count_nonzero(valid)) / total)
        self.near_pct.add(100.0 * int(np.count_nonzero(positive & (depth < ZEDX["depth_min_m"]))) / total)
        self.far_pct.add(100.0 * int(np.count_nonzero(positive & (depth > ZEDX["depth_max_m"]))) / total)
        if np.count_nonzero(valid):
            self.median_depth.add(float(np.median(depth[valid])))

    def as_dict(self):
        return {
            "frames_sampled": self.frames_sampled,
            "encoding": self.encoding,
            "size": list(self.size) if self.size else None,
            "decode_errors": self.decode_errors,
            "valid_pixels_pct": self.valid_pct.as_dict("%"),
            "closer_than_min_range_pct": self.near_pct.as_dict("%"),
            "beyond_max_range_pct": self.far_pct.as_dict("%"),
            "median_valid_depth_m": self.median_depth.as_dict("m"),
        }


# --------------------------------------------------------------------------- #
# The node
# --------------------------------------------------------------------------- #

class SensorValidator(object):

    def __init__(self, args, cfg, log):
        self.args = args
        self.cfg = cfg
        self.log = log
        self.ns = args.namespace.rstrip("/")
        self.monitors = {}
        self.depth = None
        self.camera_info = None       # first CameraInfo seen, for intrinsics
        self.np = None
        self.rospy = None
        self.started_at = None

    # ----------------------------------------------------------------- setup
    def topic(self, suffix):
        return "%s/%s" % (self.ns, suffix.lstrip("/"))

    def build(self, rospy):
        """Subscribe to everything we intend to measure.

        Message types are imported here rather than at module scope so that the
        module can be imported for its docstring, or unit-tested, on a machine
        with no ROS.
        """
        from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2

        self.rospy = rospy
        fps = self.args.fps
        imu_hz = self.args.imu_rate

        wanted = [
            ("left_rect_image", self.topic(self.cfg_get("topics/left_image",
                                                        "left/image_rect_color")), Image, fps),
            ("right_rect_image", self.topic(self.cfg_get("topics/right_image",
                                                         "right/image_rect_color")), Image, fps),
            ("depth_image", self.topic(self.cfg_get("topics/depth_image",
                                                    "depth/depth_registered")), Image, fps),
            ("left_camera_info", self.topic(self.cfg_get("topics/left_info",
                                                         "left/camera_info")), CameraInfo, fps),
            ("right_camera_info", self.topic(self.cfg_get("topics/right_info",
                                                          "right/camera_info")), CameraInfo, fps),
            ("imu", self.topic(self.cfg_get("topics/imu", "imu/data")), Imu, imu_hz),
        ]
        if not self.args.skip_cloud:
            wanted.append(("point_cloud",
                           self.topic(self.cfg_get("topics/point_cloud",
                                                   "point_cloud/cloud_registered")),
                           PointCloud2, fps))

        subs = []
        for kind, topic, msg_type, expected in wanted:
            mon = TopicMonitor(topic, expected, kind=kind)
            self.monitors[kind] = mon
            # queue_size is generous and buff_size large: a small queue would
            # make *us* drop frames, and we would then report the camera as
            # faulty. The measuring instrument must not create the defect.
            subs.append(rospy.Subscriber(
                topic, msg_type,
                lambda m, k=kind: self._on_msg(k, m),
                queue_size=200, buff_size=2 ** 24, tcp_nodelay=True))
        return subs

    def cfg_get(self, path, default=None):
        node = self.cfg
        for part in path.split("/"):
            if not isinstance(node, dict) or part not in node:
                return default
            node = node[part]
        return node if node is not None else default

    # -------------------------------------------------------------- callback
    def _on_msg(self, kind, msg):
        now = self.rospy.Time.now().to_sec()
        mon = self.monitors[kind]
        mon.observe(msg.header, now)

        if kind == "left_camera_info" and self.camera_info is None:
            self.camera_info = msg
        if kind == "depth_image" and self.depth is not None:
            self.depth.maybe_sample(msg, now)

    # ------------------------------------------------------------------- run
    def run(self):
        rospy = require_rospy()
        require_master(rospy)

        if not self.args.skip_depth:
            try:
                self.np = require_numpy()
                self.depth = DepthMonitor(self.np, self.args.depth_sample_period)
            except Unavailable as exc:
                self.log.warn("Depth statistics disabled: %s" % exc.reason)

        rospy.init_node("sensor_validator", anonymous=True, disable_signals=True)
        self.log.section("Configuration")
        self.log.info("Camera namespace : %s" % self.ns)
        self.log.info("Configured FPS   : %s" % self.args.fps)
        self.log.info("Configured IMU Hz: %s" % self.args.imu_rate)
        self.log.info("Soak duration    : %.1f s (%.1f min)"
                      % (self.args.duration, self.args.duration / 60.0))

        subs = self.build(rospy)
        for s in subs:
            self.log.info("Subscribed: %s" % s.name)

        # Wait for first data before starting the clock, so that driver startup
        # latency is not charged against the frame rate. But bound the wait:
        # hanging forever is the failure mode this project keeps hitting.
        self.log.section("Waiting for data")
        if not self._await_first_data(rospy, self.args.startup_timeout):
            silent = [m.name for m in self.monitors.values() if m.count == 0]
            raise Unavailable(
                "No messages arrived on any expected topic within %.0f s. "
                "Silent topics: %s" % (self.args.startup_timeout, ", ".join(silent)),
                self._diagnose_silence())

        self.log.section("Measuring")
        self.started_at = time.time()
        interrupted = self._soak(rospy)

        for s in subs:
            s.unregister()

        report = self.analyse()
        self.emit(report)
        if interrupted:
            return EXIT_INTERRUPTED
        return EXIT_PASS if report["verdict"] != "FAIL" else EXIT_FAIL

    def _await_first_data(self, rospy, timeout):
        deadline = time.time() + timeout
        rate = rospy.Rate(5)
        while time.time() < deadline and not rospy.is_shutdown():
            if any(m.count > 0 for m in self.monitors.values()):
                # Give the remaining topics a moment; they may start staggered.
                time.sleep(min(2.0, max(0.0, deadline - time.time())))
                return True
            rate.sleep()
        return False

    def _diagnose_silence(self):
        """Turn 'nothing arrived' into an actionable next step.

        The order matters: it walks the chain outward from the kernel, so the
        first thing that is broken is the first thing reported.
        """
        env = probe_environment()
        if not env.get("video_devices"):
            return ("No /dev/video* devices exist, so the kernel does not see the "
                    "ZED Link Quad capture card at all. Check the card is seated, "
                    "that its external 9-19 V supply is connected, and that the "
                    "ZED Link driver .deb is installed "
                    "(`dpkg -l | grep -i zedlink`). This is a hardware/driver "
                    "problem, not a ROS problem.")
        if not env.get("zed_x_daemon_running"):
            return ("Video devices exist but zed_x_daemon is not running. The "
                    "GMSL2 cameras are owned by the daemon, not opened directly "
                    "by the SDK, so nothing can grab frames until it is up. "
                    "Start it with `sudo systemctl start zed_x_daemon` and check "
                    "`systemctl status zed_x_daemon`.")
        if not env.get("zed_sdk_version"):
            return ("The ZED SDK does not appear to be installed under "
                    "/usr/local/zed. Install the 4.x SDK for JetPack 5.1.5 — note "
                    "that the 5.x line requires JetPack 6 and will not work on "
                    "this Jetson.")
        return ("The camera stack looks present, so the driver node is probably "
                "not publishing under the namespace this validator is watching. "
                "Run `rostopic list | grep -i zed` and compare against `%s`. "
                "If the names differ, pass the right one with "
                "`_namespace:=/your_ns` or fix camera_name in "
                "config/zedx_front.yaml." % self.ns)

    def _soak(self, rospy):
        """Run the measurement window, reporting progress as it goes.

        A 15-minute run with no output looks indistinguishable from a hang, so
        it prints a heartbeat. The heartbeat also gives an early warning: if
        the rate has already collapsed at minute two there is no point waiting
        until minute fifteen.
        """
        end = self.started_at + self.args.duration
        next_beat = self.started_at + self.args.progress_period
        try:
            while time.time() < end and not rospy.is_shutdown():
                time.sleep(0.2)
                now = time.time()
                if now >= next_beat:
                    next_beat = now + self.args.progress_period
                    self._heartbeat(now)
        except KeyboardInterrupt:
            self.log.warn("Interrupted by operator — analysing partial data "
                          "(%.0f s of %.0f s collected)."
                          % (time.time() - self.started_at, self.args.duration))
            return True
        return False

    def _heartbeat(self, now):
        elapsed = now - self.started_at
        parts = []
        for kind in ("left_rect_image", "depth_image", "imu"):
            mon = self.monitors.get(kind)
            if mon and mon.count:
                parts.append("%s %s" % (kind.split("_")[0], fmt_hz(mon.achieved_hz)))
        self.log.info("t+%5.0fs / %.0fs   %s"
                      % (elapsed, self.args.duration, "   ".join(parts) or "no data yet"))

    # -------------------------------------------------------------- analysis
    def analyse(self):
        """Turn the raw counters into per-check verdicts.

        Every check returns PASS, WARN or FAIL together with the number that
        justified it. A verdict with no number behind it is an opinion, and
        this file does not deal in opinions.
        """
        checks = []
        fps_tol = float(self.cfg_get("thresholds/fps_tolerance_pct", 5.0))
        drop_warn = float(self.cfg_get("thresholds/dropped_frame_warn_pct", 0.1))
        drop_fail = float(self.cfg_get("thresholds/dropped_frame_fail_pct", 1.0))
        jitter_warn = float(self.cfg_get("thresholds/jitter_p99_warn_ratio", 1.5))
        jitter_fail = float(self.cfg_get("thresholds/jitter_p99_fail_ratio", 3.0))
        imu_tol = float(self.cfg_get("thresholds/imu_rate_tolerance_pct", 5.0))
        depth_warn = float(self.cfg_get("thresholds/depth_valid_warn_pct", 40.0))
        depth_fail = float(self.cfg_get("thresholds/depth_valid_fail_pct", 15.0))
        sync_warn = float(self.cfg_get("thresholds/stereo_sync_warn_ms", 1.0))
        sync_fail = float(self.cfg_get("thresholds/stereo_sync_fail_ms", 10.0))

        def check(name, status, detail, value=None, unit=""):
            checks.append({"check": name, "status": status, "detail": detail,
                           "value": value, "unit": unit})
            return status

        # ---- 1/2/3/4: per-topic rate, drops, monotonicity, jitter
        for kind, mon in sorted(self.monitors.items()):
            if mon.count == 0:
                check("%s: present" % kind, "FAIL",
                      "No messages received on %s during the entire run." % mon.name)
                continue

            # Rate
            if mon.expected_hz:
                err = 100.0 * abs(mon.achieved_hz - mon.expected_hz) / mon.expected_hz
                st = "PASS" if err <= fps_tol else ("WARN" if err <= 2 * fps_tol else "FAIL")
                check("%s: rate" % kind, st,
                      "achieved %.2f Hz against %.1f Hz configured (%.1f%% off)"
                      % (mon.achieved_hz, mon.expected_hz, err),
                      _r(mon.achieved_hz, 3), "Hz")

            # Dropped frames, exact count from the sequence counter
            expected_total = mon.count + mon.seq_missing
            drop_pct = 100.0 * mon.seq_missing / expected_total if expected_total else 0.0
            st = "PASS" if drop_pct < drop_warn else ("WARN" if drop_pct < drop_fail else "FAIL")
            inferred = mon.inferred_drops()
            detail = ("%d of %d frames missing (%.4f%%) across %d gaps in "
                      "header.seq; independent gap-timing estimate: %d"
                      % (mon.seq_missing, expected_total, drop_pct,
                         mon.seq_gaps, inferred))
            # When the two methods disagree materially, say what that means.
            # The disagreement is a finding in its own right, not noise.
            spread = abs(inferred - mon.seq_missing)
            reliable = mon.drop_inference_reliable()
            if not reliable:
                # Do not read meaning into the disagreement when the timing
                # estimator is jitter-limited: it under-reports by construction,
                # and the "frames vanished silently" story below would send the
                # reader to debug the driver queue over what is really jitter.
                detail += (". The gap-timing estimate is jitter-limited here "
                           "(p99 interval spread is a large fraction of the "
                           "frame period), so it under-counts and the two "
                           "numbers cannot be meaningfully compared. The "
                           "sequence count is authoritative; treat the timing "
                           "estimate as a lower bound only.")
            elif spread > max(3, 0.5 * max(inferred, mon.seq_missing)):
                if inferred > mon.seq_missing:
                    detail += (". The timing estimate is much higher than the "
                               "sequence count, which means the stream is "
                               "arriving late rather than incomplete — frames "
                               "are bunching and stalling, not going missing. "
                               "Suspect CPU contention or thermal throttling on "
                               "the Jetson, not the GMSL2 link. Trust the "
                               "sequence count for how many frames were lost.")
                else:
                    detail += (". The sequence count is higher than the timing "
                               "estimate, meaning frames vanished without "
                               "leaving a gap in time. That is the signature of "
                               "a publisher dropping frames it never stamped — "
                               "check the driver's queue depth rather than the "
                               "camera.")
            check("%s: dropped frames" % kind, st, detail, _r(drop_pct, 4), "%")

            # Monotonicity — any violation at all is a failure. There is no
            # acceptable rate of time running backwards.
            issues = []
            if mon.backwards:
                issues.append("%d backwards" % mon.backwards)
            if mon.duplicate_stamps:
                issues.append("%d duplicated" % mon.duplicate_stamps)
            if mon.zero_stamps:
                issues.append("%d zero" % mon.zero_stamps)
            check("%s: timestamp monotonicity" % kind,
                  "FAIL" if issues else "PASS",
                  ", ".join(issues) if issues else
                  "all %d timestamps strictly increasing" % mon.count,
                  mon.backwards + mon.duplicate_stamps + mon.zero_stamps)

            # Jitter, judged against the nominal period rather than in absolute
            # milliseconds — 5 ms of jitter is nothing at 5 FPS and fatal at 60.
            if mon.expected_hz and mon.stamp_interval.n:
                nominal = 1.0 / mon.expected_hz
                ratio = mon.stamp_interval.pct(99) / nominal
                st = ("PASS" if ratio <= jitter_warn else
                      "WARN" if ratio <= jitter_fail else "FAIL")
                check("%s: jitter" % kind, st,
                      "p99 interval %.2f ms = %.2fx the nominal %.2f ms "
                      "(mean %.2f ms, sd %.2f ms, max %.2f ms)"
                      % (mon.stamp_interval.pct(99) * 1e3, ratio, nominal * 1e3,
                         mon.stamp_interval.mean * 1e3,
                         mon.stamp_interval.stdev * 1e3,
                         mon.stamp_interval.max * 1e3),
                      _r(ratio, 3), "x nominal")

        # ---- 5: one time base?
        checks.extend(self._check_time_base(sync_warn, sync_fail))

        # ---- 6: IMU rate against the datasheet nominal
        #
        # THE NOMINAL IS 200 Hz, NOT 400. This check reads it from
        # figure, carried across to a camera whose datasheet states 200 Hz in
        # both its "Motion Sensors" tables. The tolerance
        # (validation_thresholds.yaml: imu_rate_tolerance_pct) is 5%, so a
        # perfectly healthy ZED X publishing its rated 200 Hz produced a 50%
        # error and a FAIL verdict. The check was not merely mislabelled: it
        # would have condemned correct hardware, and the first person to run it
        # against the real camera would have gone looking for a fault that was
        # not there. The label is now derived from the constant rather than
        # written out, so the two can no longer disagree.
        #
        # THIS CHECK IGNORES --imu-rate, AND THAT IS THE POINT. --imu-rate says
        # what the wrapper was CONFIGURED to publish and is already the yardstick
        # for the "imu: rate" and "imu: jitter" checks above. This one asks the
        # different question of whether the HARDWARE is delivering its rated
        # output, so it must not be movable from the command line — an argument
        # that could redefine the datasheet would let a failing camera be argued
        # into passing. See the note on the argument itself in build_parser().
        imu = self.monitors.get("imu")
        if imu and imu.count:
            err = 100.0 * abs(imu.achieved_hz - ZEDX["imu_rate_hz"]) / ZEDX["imu_rate_hz"]
            st = "PASS" if err <= imu_tol else ("WARN" if err <= 2 * imu_tol else "FAIL")
            check("imu: rate vs %.0f Hz datasheet nominal" % ZEDX["imu_rate_hz"], st,
                  "achieved %.1f Hz against the datasheet's %.0f Hz (%.1f%% off). "
                  "A rate well below nominal usually means the wrapper's "
                  "sensors/max_pub_rate is throttling it, not that the IMU is "
                  "faulty. A rate ABOVE it means the wrapper is interpolating or "
                  "republishing, which is worth knowing before anyone treats the "
                  "extra samples as independent measurements. This line is "
                  "measured against the datasheet and is NOT affected by "
                  "--imu-rate; the 'imu: rate' check above is the one that uses "
                  "it."
                  % (imu.achieved_hz, ZEDX["imu_rate_hz"], err),
                  _r(imu.achieved_hz, 2), "Hz")

        # ---- 7: frame_id correctness
        checks.extend(self._check_frame_ids())

        # ---- 8: depth validity
        if self.depth and self.depth.frames_sampled:
            v = self.depth.valid_pct.mean
            st = "PASS" if v >= depth_warn else ("WARN" if v >= depth_fail else "FAIL")
            check("depth: valid pixels", st,
                  "%.1f%% of pixels valid on average over %d sampled frames "
                  "(p05-equivalent worst frame %.1f%%). %.1f%% fell closer than "
                  "the %.0f m minimum and %.1f%% beyond the %.0f m maximum. "
                  "Low validity on a featureless surface is physics, not a "
                  "fault — stereo needs texture to match."
                  % (v, self.depth.frames_sampled, self.depth.valid_pct.min,
                     self.depth.near_pct.mean, ZEDX["depth_min_m"],
                     self.depth.far_pct.mean, ZEDX["depth_max_m"]),
                  _r(v, 2), "%")
        elif self.depth:
            check("depth: valid pixels", "WARN",
                  "No depth frames could be decoded (%d decode errors). "
                  "Check the depth topic is publishing 32FC1 or 16UC1."
                  % self.depth.decode_errors)

        # ---- intrinsics sanity, using camera_info rather than the datasheet
        checks.extend(self._check_intrinsics())

        verdict = worst(*[c["status"] for c in checks]) if checks else "FAIL"
        return {
            "run": self.args.run_name,
            "generated_at": datetime.now().astimezone().isoformat(timespec="seconds"),
            "namespace": self.ns,
            "duration_requested_s": self.args.duration,
            "duration_actual_s": _r(time.time() - self.started_at, 2) if self.started_at else 0,
            "configured_fps": self.args.fps,
            "configured_imu_hz": self.args.imu_rate,
            "environment": probe_environment(),
            "verdict": verdict,
            "checks": checks,
            "topics": {k: m.as_dict() for k, m in sorted(self.monitors.items())},
            "depth": self.depth.as_dict() if self.depth else None,
        }

    def _check_time_base(self, warn_ms, fail_ms):
        """Do all topics share one clock, and are the stereo pairs simultaneous?

        Two separate questions:

        (a) Stereo simultaneity. The left and right images of a pair must carry
            the *same* stamp. If they do not, every disparity is computed
            between two moments in time, and while the robot is moving that is
            a geometric error that no calibration can remove.

        (b) Clock identity. If the header stamps come from the camera's own
            oscillator while ROS runs on system time, the two drift apart. The
            symptom is a latency figure that creeps steadily over a long run
            rather than staying flat — which is exactly why this check needs a
            10-15 minute soak to be meaningful, and why a 30-second smoke test
            would pass while the system is quietly broken.
        """
        out = []
        left = self.monitors.get("left_rect_image")
        right = self.monitors.get("right_rect_image")

        if left and right and left.count and right.count:
            # Both monitors saw the same stream of pairs; comparing their last
            # stamps is a cheap proxy for pair alignment across the whole run.
            skew_ms = abs(left.last_stamp - right.last_stamp) * 1e3
            st = "PASS" if skew_ms <= warn_ms else ("WARN" if skew_ms <= fail_ms else "FAIL")
            out.append({
                "check": "stereo pair simultaneity",
                "status": st,
                "detail": ("left and right stamps differ by %.3f ms at end of run. "
                           "Non-zero skew while moving biases every disparity, and "
                           "therefore every depth: at 1.5 m/s, 10 ms of skew "
                           "displaces the scene by 15 mm between the two views."
                           % skew_ms),
                "value": _r(skew_ms, 4), "unit": "ms"})

        # Latency drift: compare the first and last quarter of the latency
        # samples. A steady offset is fine and expected; a *trend* is the
        # signature of two clocks running at different rates.
        for kind in ("left_rect_image", "imu"):
            mon = self.monitors.get(kind)
            if not mon or mon.latency.n < 40:
                continue
            s = mon.latency.samples
            q = max(1, len(s) // 4)
            early = sum(s[:q]) / q
            late = sum(s[-q:]) / q
            drift_ms = (late - early) * 1e3
            span_min = max(1e-9, (mon.last_recv - mon.first_recv)) / 60.0
            rate = drift_ms / span_min if span_min > 0 else 0.0
            st = "PASS" if abs(rate) < 5.0 else ("WARN" if abs(rate) < 50.0 else "FAIL")
            out.append({
                "check": "%s: clock drift" % kind,
                "status": st,
                "detail": ("header-to-arrival latency moved %.2f ms over %.1f min "
                           "(%.2f ms/min). A constant offset is normal pipeline "
                           "delay; a trend means the stamps and ROS time come "
                           "from different oscillators."
                           % (drift_ms, span_min, rate)),
                "value": _r(rate, 4), "unit": "ms/min"})

        # Sanity: is anything stamped in the future, or absurdly stale?
        for kind, mon in sorted(self.monitors.items()):
            if not mon.count:
                continue
            if mon.latency.min < -0.05:
                out.append({
                    "check": "%s: stamps in the future" % kind,
                    "status": "FAIL",
                    "detail": ("messages arrived up to %.1f ms *before* their own "
                               "timestamp. The publisher's clock is ahead of this "
                               "machine's. If the driver runs on a different host, "
                               "run NTP on both."
                               % (-mon.latency.min * 1e3)),
                    "value": _r(mon.latency.min, 5), "unit": "s"})
        return out

    def _check_frame_ids(self):
        """Verify every topic's frame_id, and that it is namespaced per camera.

        This matters far more than it looks. A second ZED X facing the opposite
        direction is arriving later in the project. If both cameras publish
        `zed_left_camera_optical_frame`, the transform tree acquires two
        different meanings for one name and the resulting map is silently
        wrong — no error, no crash, just a map that does not close loops.
        Catching it now costs nothing; catching it after a week of driving
        costs a week of driving.
        """
        out = []
        expected_prefix = self.args.frame_prefix or self.ns.strip("/")
        for kind, mon in sorted(self.monitors.items()):
            if not mon.count:
                continue
            ids = mon.frame_ids
            if len(ids) > 1:
                out.append({
                    "check": "%s: frame_id stability" % kind, "status": "FAIL",
                    "detail": ("frame_id changed during the run: %s. A topic must "
                               "keep one frame_id for its whole life."
                               % ", ".join("%s (x%d)" % (k, v) for k, v in ids.items())),
                    "value": len(ids), "unit": "distinct ids"})
                continue
            fid = list(ids)[0]
            if not fid:
                out.append({
                    "check": "%s: frame_id present" % kind, "status": "FAIL",
                    "detail": ("frame_id is empty. Anything doing geometry with "
                               "this topic cannot know what the numbers are "
                               "relative to."),
                    "value": "", "unit": ""})
            elif expected_prefix and expected_prefix not in fid:
                out.append({
                    "check": "%s: frame_id namespacing" % kind, "status": "WARN",
                    "detail": ("frame_id is `%s`, which does not contain the "
                               "camera prefix `%s`. Namespace it now, before the "
                               "second (rear-facing) ZED X arrives and collides "
                               "with it." % (fid, expected_prefix)),
                    "value": fid, "unit": ""})
            else:
                out.append({
                    "check": "%s: frame_id" % kind, "status": "PASS",
                    "detail": "`%s`, stable for all %d messages" % (fid, mon.count),
                    "value": fid, "unit": ""})

        # Optical vs body frame convention: the ZED wrapper publishes images in
        # an optical frame (Z forward, X right, Y down) distinct from the body
        # frame (X forward, Y left, Z up). Mixing them up rotates the whole map
        # by 90 degrees, which is a classic and very confusing first-week bug.
        img = self.monitors.get("left_rect_image")
        if img and img.count and img.frame_ids:
            fid = list(img.frame_ids)[0]
            if "optical" not in fid:
                out.append({
                    "check": "image frame convention", "status": "WARN",
                    "detail": ("the image frame_id `%s` does not say `optical`. "
                               "Image data belongs in an optical frame (Z forward, "
                               "X right, Y down), not the robot body convention "
                               "(X forward, Y left, Z up). Confusing the two "
                               "rotates the entire map by 90 degrees." % fid),
                    "value": fid, "unit": ""})
        return out

    def _check_intrinsics(self):
        """Cross-check live camera_info against the ZED X datasheet optics.

        Intrinsics are never hardcoded anywhere in this project — they come
        from camera_info at runtime, because per-unit factory calibration
        differs from nominal by enough to matter at range. But the *nominal*
        value is still a useful smoke test: if the delivered focal length is
        far from what a 4.6 mm lens at this resolution should give, either the
        resolution is not what was configured or the calibration file is wrong.

        THE REFERENCE MOVED ON 2026-08-29 AND THIS CHECK GOT SHARPER.
        ZEDX["fov_h_deg"] was corrected from a remembered 80 deg to the
        datasheet's 73 deg, so the nominal focal length this compares against
        went from 1144 px to 1297 px at 1920 wide — 13.4% higher. The
        PASS/WARN/FAIL bands (15% / 30%) are unchanged, but they are now
        centred on the right place. The old reference was far enough out that a
        correctly calibrated ZED X would have landed near the edge of the PASS
        band for no reason, and a genuinely misconfigured resolution could have
        hidden inside it.

        The bands themselves are deliberately wide. They are asking "is this
        the right camera at the right resolution?", not "is this calibration
        good?" — the second question is answered by the factory calibration,
        not by a nominal-optics comparison, and tightening these numbers would
        turn per-unit variation into false alarms.
        """
        out = []
        info = self.camera_info
        if info is None:
            return [{"check": "intrinsics", "status": "WARN",
                     "value": None, "unit": "",
                     "detail": ("No CameraInfo was received, so intrinsics could "
                                "not be checked. ORB-SLAM3 config generation in "
                                "Phase 4 reads this topic, and will fail without "
                                "it.")}]
        fx = info.P[0] if getattr(info, "P", None) else info.K[0]
        cx = info.P[2] if getattr(info, "P", None) else info.K[2]
        nominal = focal_px_from_fov(info.width, ZEDX["fov_h_deg"])
        err = 100.0 * abs(fx - nominal) / nominal if nominal else 0.0
        st = "PASS" if err < 15.0 else ("WARN" if err < 30.0 else "FAIL")
        out.append({
            "check": "intrinsics plausibility", "status": st,
            "detail": ("camera_info reports fx = %.1f px at %dx%d; a %.0f deg "
                       "horizontal FOV at that width implies about %.1f px "
                       "(%.1f%% apart). Principal point cx = %.1f versus an image "
                       "centre of %.1f."
                       % (fx, info.width, info.height, ZEDX["fov_h_deg"], nominal,
                          err, cx, info.width / 2.0)),
            "value": _r(fx, 2), "unit": "px"})

        # The 120 mm baseline is the single number that, with the focal length,
        # sets this camera's usable range. Report the consequence explicitly so
        # that "why does depth get bad far away?" has an answer in the log
        # rather than in someone's memory.
        #
        # THE TWO RANGES QUOTED ARE 2 m AND 20 m ON PURPOSE. They are the only
        # two distances at which Stereolabs publishes an accuracy figure for
        # this camera ("< 0.4% to 2m", "< 7% at 20m"), so a reader can put the
        # model's prediction beside the manufacturer's bound and see whether
        # a pair of "datasheet" percentages that do not appear in the datasheet
        # at all — comparing against them proved nothing.
        if fx:
            e2 = depth_uncertainty_m(2.0, fx)
            e20 = depth_uncertainty_m(20.0, fx)
            out.append({
                "check": "stereo baseline (from camera_info)", "status": "PASS",
                "detail": ("Datasheet baseline is %.0f mm. Depth uncertainty "
                           "scales as Z^2 * sigma_d / (f*B), so with fx = %.0f px "
                           "and B = %.3f m, a quarter-pixel disparity error gives "
                           "about %.0f mm of depth error at 2 m (%.2f%%, against "
                           "the datasheet bound of <%.1f%%) and %.2f m at 20 m "
                           "(%.1f%%, against <%.0f%%). That is why this camera is "
                           "specified 1-35 m and not further."
                           % (ZEDX["baseline_mm"], fx, ZEDX["baseline_mm"] / 1000.0,
                              1000.0 * e2, 100.0 * e2 / 2.0,
                              ZEDX["depth_err_to_2m_pct"],
                              e20, 100.0 * e20 / 20.0,
                              ZEDX["depth_err_at_20m_pct"])),
                "value": ZEDX["baseline_mm"], "unit": "mm"})
        return out

    # ----------------------------------------------------------------- output
    def emit(self, report):
        self.log.section("Results")
        width = max(len(c["check"]) for c in report["checks"]) if report["checks"] else 10
        for c in report["checks"]:
            line = "%-6s %-*s  %s" % (c["status"], width, c["check"], c["detail"])
            if c["status"] == "FAIL":
                self.log.error(line)
            elif c["status"] == "WARN":
                self.log.warn(line)
            else:
                self.log.info(line)

        self.log.section("Metrics")
        for kind, mon in sorted(self.monitors.items()):
            if not mon.count:
                continue
            self.log.metric("%s.achieved_hz" % kind, _r(mon.achieved_hz, 3), "Hz")
            self.log.metric("%s.frames_received" % kind, mon.count)
            self.log.metric("%s.frames_missing" % kind, mon.seq_missing)
            self.log.metric("%s.jitter_p99_ms" % kind, _r(mon.stamp_interval.pct(99) * 1e3, 3), "ms")
        if self.depth and self.depth.frames_sampled:
            self.log.metric("depth.valid_pixels_pct", _r(self.depth.valid_pct.mean, 2), "%")

        out_dir = REPO / "logs" / "sidewalk_perception"
        out_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        path = out_dir / ("%s_%s_validation.json" % (stamp, self.args.run_name))
        path.write_text(json.dumps(report, indent=2, default=str))
        self.log.info("Full machine-readable report: %s" % path)

        n_fail = sum(1 for c in report["checks"] if c["status"] == "FAIL")
        n_warn = sum(1 for c in report["checks"] if c["status"] == "WARN")
        self.log.summary(
            "Sensor validation %s over %.1f min — %d checks, %d failed, %d warned."
            % (report["verdict"], report["duration_actual_s"] / 60.0,
               len(report["checks"]), n_fail, n_warn),
            status=report["verdict"])


# --------------------------------------------------------------------------- #

def build_parser():
    p = argparse.ArgumentParser(
        description="Measure ZED X stream health: rate, drops, jitter, time "
                    "base, IMU rate, frame_ids and depth validity.")
    p.add_argument("--namespace", default="/zedx_front",
                   help="Camera topic namespace (default: /zedx_front)")
    p.add_argument("--fps", type=float, default=30.0,
                   help="Configured image frame rate to measure against")
    # "accepted and recorded but ignored", and that is not correct — it is worth
    # writing down precisely, because the half-truth is the kind that gets an
    # option deleted and a capability lost with it.
    #
    # build() assigns it to `imu_hz` and passes it as the imu TopicMonitor's
    # expected_hz. Through that one hop it sets TWO live verdicts in analyse():
    #     "imu: rate"     achieved Hz judged against THIS number, banded by
    #                     thresholds/fps_tolerance_pct
    #     "imu: jitter"   the nominal interval it compares the p99 against is
    #                     1.0 / THIS number
    # What it does NOT touch is check 6, "imu: rate vs ... datasheet nominal",
    # which reads ZEDX["imu_rate_hz"] directly and on purpose — see the note
    # beside that check. Those are two different questions: "is the wrapper
    # delivering what it was configured for" and "is the hardware delivering
    # what the datasheet promises", and a soak that answers only one of them is
    # weaker than one that answers both.
    #
    # KEPT rather than removed, for that reason. Removing it would leave no way
    # to validate a stream that has been deliberately throttled — set
    # sensors/max_pub_rate in zedx_front.yaml below 200 and the configured rate
    # genuinely is not the datasheet rate any more. Wiring check 6 to it instead
    # come from one constant so they cannot disagree.
    p.add_argument("--imu-rate", type=float, default=ZEDX["imu_rate_hz"],
                   help="CONFIGURED IMU publish rate. Drives the 'imu: rate' "
                        "and 'imu: jitter' checks only. Change it only if "
                        "sensors/max_pub_rate in zedx_front.yaml has been set "
                        "below the sensor's own rate; it does NOT move the "
                        "separate datasheet check, which always uses %.0f Hz. "
                        "(default: %.0f Hz)"
                        % (ZEDX["imu_rate_hz"], ZEDX["imu_rate_hz"]))
    p.add_argument("--duration", type=float, default=900.0,
                   help="Soak length in seconds (default 900 = 15 minutes)")
    p.add_argument("--quick", action="store_true",
                   help="Shorthand for --duration 60: a smoke test, NOT a "
                        "substitute for a soak. Clock drift and thermal "
                        "throttling need at least ten minutes to appear.")
    p.add_argument("--startup-timeout", type=float, default=20.0,
                   help="How long to wait for the first message before giving up")
    p.add_argument("--progress-period", type=float, default=30.0,
                   help="Seconds between heartbeat lines")
    p.add_argument("--depth-sample-period", type=float, default=1.0,
                   help="Seconds between depth frames sampled for pixel statistics")
    p.add_argument("--skip-depth", action="store_true",
                   help="Do not decode depth images (skips the numpy dependency)")
    p.add_argument("--skip-cloud", action="store_true",
                   help="Do not subscribe to the point cloud (saves bandwidth)")
    p.add_argument("--frame-prefix", default="",
                   help="Expected substring in every frame_id "
                        "(defaults to the namespace)")
    p.add_argument("--config", default=str(DEFAULT_CONFIG),
                   help="Threshold config YAML")
    p.add_argument("--run-name", default="sensor_validation",
                   help="Name recorded in the log and RUNLOG.md")
    return p


def strip_ros_args(argv):
    """Drop roslaunch's injected __name:= / __log:= arguments.

    roslaunch appends these to every node it starts, and argparse would reject
    them. Private `_param:=value` args are also removed here; this node reads
    its settings from the command line and the config file rather than the
    parameter server, so that it behaves identically when run by hand.
    """
    return [a for a in argv
            if not a.startswith("__") and ":=" not in a]


def main(argv=None):
    argv = strip_ros_args(sys.argv[1:] if argv is None else argv)
    args = build_parser().parse_args(argv)
    if args.quick:
        args.duration = 60.0

    cfg = {}
    if os.path.exists(args.config):
        try:
            cfg = load_simple_yaml(args.config) or {}
        except Exception as exc:
            print("Could not parse %s (%s) — using built-in defaults."
                  % (args.config, exc), file=sys.stderr)

    with RunLogger("sidewalk_perception", run_name=args.run_name) as log:
        validator = SensorValidator(args, cfg, log)
        try:
            code = validator.run()
        except Unavailable as exc:
            return die(log, exc)
        except KeyboardInterrupt:
            log.warn("Interrupted before measurement began.")
            log.summary("Interrupted by operator", status="WARN")
            return EXIT_INTERRUPTED
        # `emit()` has already written the real summary; the context manager
        # sees the log as closed and will not overwrite it with a generic one.
        return code


if __name__ == "__main__":
    sys.exit(main())
