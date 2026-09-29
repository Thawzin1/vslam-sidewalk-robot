#!/usr/bin/env python3
"""odom_debias.py - remove the robot's parked gyroscope drift from its own odometry.

RUNS ON
    * any computer, offline, on a recorded path (a TUM file: one pose per line,
      "time x y z qx qy qz qw" - the standard trajectory text format);
    * the robot, live, imported by lidar_live_map.py (copy this file into the
      same folder as lidar_live_map.py when deploying).
    Python 3.8 and numpy only.

PUBLISHES NOTHING - no ROS topic, and above all no TF frame. (TF is ROS's live
record of where each part of the robot is, kept as a chain of named frames;
each frame may have only ONE parent frame. A corrected frame placed above
`odom` would give `odom` two parents once the LiDAR reference also runs, and
break the chain - report section 2.1 item 9.)

WHAT IT FIXES
    The robot's odometry (/odometry/filtered: its running estimate of how it has
    moved, blending wheel turns with an IMU - an inertial measurement unit, a
    small sensor that measures turning) works out which way the robot faces by
    adding up small turns. Its gyroscope (the turning sensor) reports a small
    false turn all the time, and nothing subtracts it: -2.7 to -5.0 degrees per
    minute on drives 1 and 2, even while parked. Over a 23-minute drive that is
    about 115 degrees, which is what doubled every corridor in the LiDAR map.

    *Plain terms: the robot's own record stays exactly as it is. This makes a
    corrected copy that says "and by now that record has turned this much too
    far".*

HOW - three steps, the SAME functions offline and live
    1. find_start_parked(): the stretch at the START of the path where the
       robot stands still - moving slower than 0.005 m/s (5 mm a second) and
       turning slower than 2 degrees a second - for at least MIN seconds
       (20 s offline by default; the live view asks for 60 s, the drive
       procedure's minimum).
    2. fit_rate(): a straight line through heading against time over that
       stretch. Its slope is the drift rate, in degrees per minute.
    3. correct_path(): subtract (rate x time since the start) from every
       heading, and add the position steps up again, each step turned by the
       correction at its own moment.

    ASSUMPTION, stated: the drift rate stays the same while driving as it was
    while parked. Drive 2 supports this (parked stretches -4.7 to -5.0 deg/min
    from start to end); drive 1 does not fully (start -2.7, end -3.7 deg/min).
    It does nothing for distance, only heading.

  usage (any computer, offline):
    odom_debias.py WHEEL.tum -o OUT.tum            corrected path + OUT.tum.meta.json
    odom_debias.py WHEEL.tum -o OUT.tum --min-parked 60
    odom_debias.py WHEEL.tum -o OUT.tum --rate -4.95   use a given rate instead of fitting
    odom_debias.py WHEEL.tum --compare-live POSES.csv  how far a live pose log
                                                       (lidar_live_map.py) is from
                                                       this file's corrected path
  (Every run writes only the files named. Without -o it writes <input>_debiased.tum
   beside the input and REFUSES if that file already exists; -o overwrites what it names.)
"""
from __future__ import print_function

import argparse
import hashlib
import json
import math
import os
import sys
import time

import numpy as np

# ---------------- settings: what each one controls ----------------
# The robot counts as standing still while it moves slower than this (m/s).
# 0.005 m/s = 5 mm a second; parked, the robot's own position does not change
# at all (checked on drives 1 and 2: one single position for the whole stretch).
SPEED_MAX = 0.005
# ... AND while it turns slower than this (degrees a second). A turn on the spot
# has no forward speed, so the speed test alone would call it "parked". Parked,
# the heading moves at most 0.5 deg/s between readings (drives 1 and 2); a real
# turn is 10-50 deg/s.
YAW_RATE_MAX_DEG_S = 2.0
# If the robot is nudged before it has stood still long enough (someone lines it
# up on the start mark), the search starts again at the next stop - but only
# while it is still within this distance of where it was first seen (m).
START_RADIUS_M = 0.5
# The standard error of the rate comes from cutting the stretch into this many
# equal pieces and fitting each one (see fit_rate).
BLOCKS = 6
MIN_PARKED_OFFLINE_S = 20.0
MIN_PARKED_LIVE_S = 60.0
# The live view switches the correction on only when the parked stretch holds at
# least this many poses. It gets about 2 a second (one per placed scan), so a
# normal 60 s park gives about 120. Fewer means the LiDAR or the robot's position
# record went quiet while parked; a rate fitted to a handful of poses has no
# usable uncertainty (with fewer than 9 poses there are not even 3 pieces to
# take one from), so the view keeps waiting instead of drawing with a guess.
MIN_POSES_LIVE = 30


# ---------------- small helpers ----------------
def wrap(a):
    """Angle(s) into -pi .. +pi."""
    return (np.asarray(a, dtype=float) + np.pi) % (2 * np.pi) - np.pi


def yaw_from_quat(qx, qy, qz, qw):
    return np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def heading_correction(t, rate_rad_s, t_ref):
    """What to ADD to the recorded heading at time t: minus (rate x time since t_ref)."""
    return -rate_rad_s * (np.asarray(t, dtype=float) - t_ref)


def rotate_steps(dx, dy, t_a, t_b, rate_rad_s, t_ref):
    """Turn each position step (dx, dy), taken between times t_a and t_b, by the
    heading correction at the step's midpoint. Works on single numbers or arrays.

    Why the midpoint: the correction changes by (rate x step time) during a step -
    0.002 degrees for a 0.5 s step at 5 deg/min - so any choice is fine; the
    midpoint is the more exact one."""
    dm = heading_correction(0.5 * (np.asarray(t_a, dtype=float) + np.asarray(t_b, dtype=float)),
                            rate_rad_s, t_ref)
    c, s = np.cos(dm), np.sin(dm)
    return c * dx - s * dy, s * dx + c * dy


def correction_se2(raw, corrected):
    """The flat (x, y, heading) move that carries a raw pose onto its corrected pose,
    as (angle, tx, ty): corrected = rotate(angle) * raw + (tx, ty). The live view
    applies it, as a 4x4 matrix, to the scan's 3D pose, so roll and pitch are kept."""
    ang = float(corrected[2] - raw[2])
    c, s = math.cos(ang), math.sin(ang)
    return ang, corrected[0] - (c * raw[0] - s * raw[1]), corrected[1] - (s * raw[0] + c * raw[1])


# ---------------- step 1: the parked stretch at the start ----------------
def moving_steps(t, x, y, yaw_unwrapped, speed_max=SPEED_MAX, yaw_rate_max_deg_s=YAW_RATE_MAX_DEG_S):
    """For each step between neighbouring poses: True if the robot was moving
    (too fast, or turning too fast) during it."""
    dt = np.diff(t)
    step = np.hypot(np.diff(x), np.diff(y))
    dyaw = np.abs(np.diff(yaw_unwrapped))
    with np.errstate(divide="ignore", invalid="ignore"):
        v = np.where(dt > 0, step / np.where(dt > 0, dt, 1.0), np.where(step > 0, np.inf, 0.0))
        w = np.where(dt > 0, dyaw / np.where(dt > 0, dt, 1.0), np.where(dyaw > 0, np.inf, 0.0))
    return (v >= speed_max) | (np.degrees(w) >= yaw_rate_max_deg_s)


def find_start_parked(t, x, y, yaw, min_s=MIN_PARKED_OFFLINE_S, speed_max=SPEED_MAX,
                      yaw_rate_max_deg_s=YAW_RATE_MAX_DEG_S, start_radius_m=START_RADIUS_M):
    """The stretch at the START where the robot stands still.

    Returns a dict:
      status   "measured"      a stretch of at least min_s exists (the rate can be fitted)
               "parked_so_far" still standing (or being nudged near the start), not yet min_s
               "moved_away"    it drove off (left start_radius_m) before standing still min_s
      i0, i1   first and last sample of the stretch (inclusive)
      length_s its length in seconds
      ended    True if the robot moved after it (the stretch is complete)
      restarts how many times a nudge near the start restarted the search
    """
    t = np.asarray(t, dtype=float)
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    n = len(t)
    if n < 2:
        return {"status": "parked_so_far", "i0": 0, "i1": max(n - 1, 0), "length_s": 0.0,
                "ended": False, "restarts": 0}
    yu = np.unwrap(np.asarray(yaw, dtype=float))
    mv = moving_steps(t, x, y, yu, speed_max, yaw_rate_max_deg_s)
    far = np.hypot(x - x[0], y - y[0]) > start_radius_m
    i, restarts = 0, 0
    while True:
        nz = np.flatnonzero(mv[i:])
        j, ended = (i + int(nz[0]), True) if len(nz) else (n - 1, False)
        out = {"i0": i, "i1": j, "length_s": float(t[j] - t[i]), "ended": ended, "restarts": restarts}
        if out["length_s"] >= min_s:
            out["status"] = "measured"
            return out
        if not ended:
            out["status"] = "parked_so_far"
            return out
        # it moved before min_s - resume at the next still step, if it is still near the start
        nz2 = np.flatnonzero(~mv[j:])
        k = j + int(nz2[0]) if len(nz2) else n - 1
        if far[:k + 1].any():
            out["status"] = "moved_away"
            return out
        if not len(nz2):                      # still moving, but near the start: wait
            return {"status": "parked_so_far", "i0": n - 1, "i1": n - 1, "length_s": 0.0,
                    "ended": False, "restarts": restarts + 1}
        i, restarts = k, restarts + 1


# ---------------- step 2: the drift rate ----------------
def _slope(t, y):
    tc = t - t.mean()
    sxx = float((tc * tc).sum())
    if sxx <= 0:
        return float("nan"), tc, sxx
    return float((tc * (y - y.mean())).sum() / sxx), tc, sxx


def fit_rate(t, yaw, blocks=BLOCKS):
    """Straight line through heading against time. Returns a dict, rates in deg/min:

      rate_deg_per_min      the slope (negative = the recorded heading creeps clockwise)
      se_deg_per_min        its standard error, from the scatter of the slopes of
                            `blocks` equal pieces: std(piece slopes) / sqrt(pieces)
      se_ols_deg_per_min    the textbook line-fit standard error - REPORTED, NOT USED
      resid_rms_deg         how far heading strays from the line (root-mean-square)
      lag1                  correlation between neighbouring residuals

    Why not the textbook standard error: it assumes each reading's error is
    independent of the next. Here neighbouring headings are almost perfectly
    correlated (lag-one correlation 0.997-0.999 on drives 1 and 2), so the textbook
    figure is far too small: 37 times on drive 1's start, 85 times on drive 2's.
    The piece-by-piece scatter needs no such assumption. On drives 1 and 2 it
    agrees with how much back-to-back (not overlapping) windows disagree with
    each other: 20 s windows 0.19 deg/min (13 pairs) against a median piece
    figure of 0.20; 60 s windows 0.13 against 0.12, but from ONE pair only."""
    t = np.asarray(t, dtype=float)
    y = np.unwrap(np.asarray(yaw, dtype=float))
    n = len(t)
    if n < 3 or t[-1] - t[0] <= 0:
        raise ValueError("need at least 3 poses spanning some time to fit a rate")
    slope, tc, sxx = _slope(t, y)
    resid = y - y.mean() - slope * tc
    se_ols = math.sqrt(float((resid ** 2).sum()) / (n - 2) / sxx) if n > 2 else float("nan")
    lag1 = float(np.corrcoef(resid[:-1], resid[1:])[0, 1]) if n > 3 and resid.std() > 0 else float("nan")
    k = min(int(blocks), n // 3)
    se_block = float("nan")
    if k >= 3:
        rates = []
        for idx in np.array_split(np.arange(n), k):
            r, _, s = _slope(t[idx], y[idx])
            if s > 0:
                rates.append(r)
        if len(rates) >= 3:
            se_block = float(np.std(rates, ddof=1) / math.sqrt(len(rates)))
    per_min = math.degrees(1.0) * 60.0
    endpoint = (y[-1] - y[0]) / (t[-1] - t[0])
    return {"rate_deg_per_min": slope * per_min, "se_deg_per_min": se_block * per_min,
            "se_ols_deg_per_min": se_ols * per_min, "resid_rms_deg": math.degrees(math.sqrt(float((resid ** 2).mean()))),
            "lag1": lag1, "n": int(n), "length_s": float(t[-1] - t[0]), "blocks": int(k),
            # REPORTED ONLY: (last heading - first heading) / time, the estimator C_drift.md
            # section 5 used. Not applied anywhere.
            "rate_endpoint_deg_per_min": float(endpoint) * per_min}


def all_parked_stretches(t, x, y, yaw, min_s=MIN_PARKED_OFFLINE_S, speed_max=SPEED_MAX,
                         yaw_rate_max_deg_s=YAW_RATE_MAX_DEG_S):
    """Every stretch, anywhere in the path, where the robot stood still at least min_s
    (same rule as find_start_parked). For reports only - e.g. comparing the start
    stretch's rate with the one measured after parking. List of (i0, i1)."""
    t = np.asarray(t, dtype=float)
    mv = moving_steps(t, np.asarray(x, dtype=float), np.asarray(y, dtype=float),
                      np.unwrap(np.asarray(yaw, dtype=float)), speed_max, yaw_rate_max_deg_s)
    out, i, n = [], 0, len(t)
    while i < n - 1:
        if mv[i]:
            i += 1
            continue
        nz = np.flatnonzero(mv[i:])
        j = i + int(nz[0]) if len(nz) else n - 1
        if t[j] - t[i] >= min_s:
            out.append((i, j))
        i = j + 1
    return out


# ---------------- step 3: the corrected path ----------------
def correct_path(t, x, y, yaw, rate_deg_per_min, t_ref=None):
    """Corrected (x, y, heading) arrays. The corrected path starts exactly where the
    recorded one starts, facing the same way (at t_ref, the first time by default);
    after that every heading gets minus (rate x time since t_ref) added, and every
    position step is turned by that amount. Headings come back unwrapped (they may
    run past +-180 degrees); wrap() them for display."""
    t = np.asarray(t, dtype=float)
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    yaw = np.asarray(yaw, dtype=float)
    r = math.radians(rate_deg_per_min) / 60.0
    t_ref = float(t[0]) if t_ref is None else float(t_ref)
    yaw_c = yaw + heading_correction(t, r, t_ref)
    if len(t) < 2:
        return x.copy(), y.copy(), yaw_c
    sx, sy = rotate_steps(np.diff(x), np.diff(y), t[:-1], t[1:], r, t_ref)
    xc = x[0] + np.concatenate([[0.0], np.cumsum(sx)])
    yc = y[0] + np.concatenate([[0.0], np.cumsum(sy)])
    return xc, yc, yaw_c


# ---------------- the same three steps, one pose at a time (live) ----------------
class LiveDebias(object):
    """Fed one pose at a time by lidar_live_map.py (about 2 a second).

    Until the robot has stood still for min_parked_s at the start, poses pass
    through UNCORRECTED and describe() says so. From then on every pose is
    corrected; the rate keeps being refined while the robot is still parked, and
    is frozen the moment it drives off. If it drives off (leaves start_radius_m)
    before min_parked_s, nothing is corrected for the whole drive, and describe()
    says why. Every decision calls the same find_start_parked / fit_rate /
    correct_path / rotate_steps as the offline tool - there is one copy of the maths.

    QUALITY GATE (added 2026-09-24 after review): standing still long enough is
    not enough on its own. The stretch must also hold at least min_poses poses,
    and the fit must give a finite rate AND a finite standard error. Otherwise the
    view keeps measuring while the robot is still parked, or - if it has already
    driven off - stays uncorrected for the whole drive ("unusable") and says why.
    Before this gate, a parked minute with only 2 poses in it (the LiDAR or the
    position record gone quiet) made fit_rate raise, which silently ended the live
    view's worker thread; with 3 to 8 poses the correction switched ON with
    "drift +- nan".
    """

    def __init__(self, min_parked_s=MIN_PARKED_LIVE_S, speed_max=SPEED_MAX,
                 yaw_rate_max_deg_s=YAW_RATE_MAX_DEG_S, start_radius_m=START_RADIUS_M, blocks=BLOCKS,
                 min_poses=MIN_POSES_LIVE):
        self.min_parked_s = float(min_parked_s)
        self.speed_max = speed_max
        self.yaw_rate_max_deg_s = yaw_rate_max_deg_s
        self.start_radius_m = start_radius_m
        self.blocks = blocks
        self.min_poses = int(min_poses)
        # waiting | measuring | measured | frozen | moved_away | unusable
        self.status = "waiting"
        self.rate = None                 # deg/min, once measured
        self.fit = None
        self.stretch = None
        self.why_not = ""                # why a long-enough park has not given a rate yet
        self.t_ref = None
        self._hist = ([], [], [], [])    # t, x, y, unwrapped yaw - kept only until frozen
        self._last = None                # last raw pose (t, x, y, unwrapped yaw)
        self._corr = None                # corrected (x, y) at the last pose

    @property
    def correcting(self):
        return self.rate is not None

    @property
    def still_parked(self):
        """True while the robot has not yet left the stretch the rate comes from."""
        return self.status in ("waiting", "measuring", "measured")

    def _rad_s(self):
        return math.radians(self.rate) / 60.0

    def add(self, t, x, y, yaw):
        """One raw pose in. Returns (x, y, heading, started): the corrected pose
        (the raw one while not correcting), and started=True on the one call where
        the correction switched on."""
        t, x, y, yaw = float(t), float(x), float(y), float(yaw)
        if self._last is None:
            yu = yaw
            self.t_ref = t
        else:
            yu = self._last[3] + float(wrap(yaw - self._last[3]))
            if t <= self._last[0]:           # out of order or repeated: answer, store nothing
                return self._peek(t, x, y, yu) + (False,)
        if self.status in ("frozen", "moved_away", "unusable"):
            if self.status == "frozen":
                lt, lx, ly, _ = self._last
                sx, sy = rotate_steps(x - lx, y - ly, lt, t, self._rad_s(), self.t_ref)
                self._corr = (self._corr[0] + float(sx), self._corr[1] + float(sy))
            self._last = (t, x, y, yu)
            return self._out(t, x, y, yu) + (False,)
        for lst, v in zip(self._hist, (t, x, y, yu)):
            lst.append(v)
        T, X, Y, U = (np.asarray(h, dtype=float) for h in self._hist)
        st = find_start_parked(T, X, Y, U, self.min_parked_s, self.speed_max,
                               self.yaw_rate_max_deg_s, self.start_radius_m)
        self.stretch = st
        started = False
        if st["status"] == "measured":
            fit, why = self._gated_fit(T, U, st)
            if fit is None and self.rate is None:
                # long enough, but not enough poses (or no usable uncertainty): no rate yet
                self.why_not = why
                if st["ended"]:
                    self.status = "unusable"         # it drove off: uncorrected all drive
                    self._hist = ([], [], [], [])
                else:
                    self.status = "measuring"        # still parked: keep collecting
                self._last = (t, x, y, yu)
                return self._out(t, x, y, yu) + (False,)
            was = self.rate is not None
            if fit is not None:                      # (a failed REfit keeps the previous rate)
                self.fit, self.why_not = fit, ""
                self.rate = fit["rate_deg_per_min"]
            xc, yc, _ = correct_path(T, X, Y, U, self.rate, t_ref=self.t_ref)
            self._corr = (float(xc[-1]), float(yc[-1]))
            started = not was
            if st["ended"]:
                self.status = "frozen"
                self._hist = ([], [], [], [])  # not needed any more
            else:
                self.status = "measured"
        elif st["status"] == "moved_away":
            self.status = "moved_away"
            self.rate = None
            self._hist = ([], [], [], [])
        else:
            self.status = "measuring"
            self.why_not = ""
        self._last = (t, x, y, yu)
        return self._out(t, x, y, yu) + (started,)

    def _gated_fit(self, T, U, st):
        """fit_rate on the parked stretch, or (None, why) if the stretch is too thin to trust."""
        n = int(st["i1"] - st["i0"] + 1)
        if n < self.min_poses:
            return None, "only %d poses arrived in %.0f s parked (needs %d)" % (n, st["length_s"], self.min_poses)
        fit = fit_rate(T[st["i0"]:st["i1"] + 1], U[st["i0"]:st["i1"] + 1], self.blocks)
        if not (math.isfinite(fit["rate_deg_per_min"]) and math.isfinite(fit["se_deg_per_min"])):
            return None, "the rate had no usable uncertainty (%d poses in %.0f s parked)" % (n, st["length_s"])
        return fit, ""

    def _out(self, t, x, y, yu):
        if not self.correcting:
            return (x, y, float(wrap(yu)))
        return (self._corr[0], self._corr[1],
                float(wrap(yu + heading_correction(t, self._rad_s(), self.t_ref))))

    def _peek(self, t, x, y, yu):
        if not self.correcting:
            return (x, y, float(wrap(yu)))
        lt, lx, ly, _ = self._last
        sx, sy = rotate_steps(x - lx, y - ly, lt, t, self._rad_s(), self.t_ref)
        return (self._corr[0] + float(sx), self._corr[1] + float(sy),
                float(wrap(yu + heading_correction(t, self._rad_s(), self.t_ref))))

    def describe(self):
        """Short plain strings for the live page's /stats.json."""
        need = self.min_parked_s
        st = self.stretch or {}
        if self.status == "waiting":
            return {"drift": "waiting for the first pose - drawn UNCORRECTED", "correction": "OFF"}
        if self.status == "measuring":
            if self.why_not:                 # parked long enough, but too few poses so far
                return {"drift": "not measured yet: %s - drawn UNCORRECTED" % self.why_not,
                        "correction": "OFF (keep the robot still)"}
            return {"drift": "not measured yet: parked %.0f of the %.0f s needed - drawn UNCORRECTED"
                             % (st.get("length_s", 0.0), need),
                    "correction": "OFF (keep the robot still)"}
        if self.status == "moved_away":
            return {"drift": "NOT measured: the robot drove off after %.0f s parked (needs %.0f s) - "
                             "drawn UNCORRECTED for this drive" % (st.get("length_s", 0.0), need),
                    "correction": "OFF"}
        if self.status == "unusable":
            return {"drift": "NOT measured: the robot drove off, and the parked start was unusable: %s - "
                             "drawn UNCORRECTED for this drive" % self.why_not,
                    "correction": "OFF"}
        f = self.fit
        d = {"drift": "%+.2f deg/min over %.0f s parked" % (self.rate, f["length_s"]),
             "drift +-": "%.2f deg/min" % f["se_deg_per_min"],
             "correction": "ON"}
        if self.status == "measured":
            d["drift"] += " (still parked - refining)"
        return d


# ---------------- files ----------------
def read_tum(path):
    """All rows of a TUM file as an array (time, x, y, z, qx, qy, qz, qw), in time order,
    plus the comment lines at its top."""
    rows, head = [], []
    with open(path) as fh:
        for line in fh:
            if line.startswith("#"):
                if not rows:
                    head.append(line.rstrip("\n"))
                continue
            p = line.split()
            if len(p) < 8:
                continue
            rows.append([float(v) for v in p[:8]])
    a = np.array(rows, dtype=float).reshape(-1, 8)
    order = np.argsort(a[:, 0], kind="stable")
    return a[order], head, bool((order != np.arange(len(order))).any())


def rotate_quat_z(q, ang):
    """Quaternion(s) (qx, qy, qz, qw) turned by ang about the vertical, on the left, so any
    roll and pitch are kept."""
    sz, cz = np.sin(ang / 2), np.cos(ang / 2)
    qx, qy, qz, qw = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.column_stack([cz * qx - sz * qy, cz * qy + sz * qx, cz * qz + sz * qw, cz * qw - sz * qz])


def sha256_of(path):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for b in iter(lambda: fh.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def hamilton_now():
    os.environ["TZ"] = "America/Toronto"
    time.tzset()
    return time.strftime("%Y-%m-%d %H:%M:%S %Z")


def end_numbers(t, x, y, yaw):
    """Last pose against first: 2D distance (m) and heading change wrapped to +-180 (deg),
    plus the unwrapped net turn (deg)."""
    yu = np.unwrap(yaw)
    return {"end_gap_m": round(float(math.hypot(x[-1] - x[0], y[-1] - y[0])), 4),
            "end_heading_change_deg": round(float(np.degrees(wrap(yu[-1] - yu[0]))), 3),
            "net_turn_deg": round(float(np.degrees(yu[-1] - yu[0])), 3)}


def debias_file(path, out, min_parked=MIN_PARKED_OFFLINE_S, rate=None, t_ref=None, quiet=False):
    """The command-line tool's work, callable from other scripts. Returns the meta dict."""
    a, head, resorted = read_tum(path)
    t, x, y, z = a[:, 0], a[:, 1], a[:, 2], a[:, 3]
    yaw = np.unwrap(yaw_from_quat(a[:, 4], a[:, 5], a[:, 6], a[:, 7]))
    st = find_start_parked(t, x, y, yaw, min_parked)
    fit = None
    if st["status"] == "measured":
        fit = fit_rate(t[st["i0"]:st["i1"] + 1], yaw[st["i0"]:st["i1"] + 1])
    if rate is None:
        if fit is None:
            raise SystemExit("odom_debias: no parked stretch of at least %.0f s at the start of %s "
                             "(found %.1f s, status %s). Nothing written. Give --rate to force one."
                             % (min_parked, path, st["length_s"], st["status"]))
        rate, source = fit["rate_deg_per_min"], "fitted to the parked stretch at the start"
    else:
        source = "given on the command line (the fitted value is kept below for comparison)"
    t_ref = float(t[0]) if t_ref is None else float(t_ref)
    xc, yc, yawc = correct_path(t, x, y, yaw, rate, t_ref=t_ref)
    qc = rotate_quat_z(a[:, 4:8], yawc - yaw)
    with open(out, "w") as fh:
        for h in head:
            fh.write(h + "\n")
        fh.write("# corrected by odom_debias.py: heading drift %+.4f deg/min removed, from t_ref %.6f\n"
                 % (rate, t_ref))
        for i in range(len(t)):
            fh.write("%.6f %.6f %.6f %.6f %.9f %.9f %.9f %.9f\n"
                     % (t[i], xc[i], yc[i], z[i], qc[i, 0], qc[i, 1], qc[i, 2], qc[i, 3]))
    meta = {
        "tool": "odom_debias.py", "created_hamilton": hamilton_now(),
        "source": os.path.abspath(path), "source_sha256": sha256_of(path),
        "output": os.path.abspath(out), "poses": int(len(t)),
        "source_was_out_of_time_order": resorted,
        "rate_deg_per_min": round(float(rate), 4), "rate_source": source,
        "fit": None if fit is None else {k: (round(v, 5) if isinstance(v, float) else v) for k, v in fit.items()},
        "parked_stretch_used": {
            "status": st["status"], "start_s_from_first_pose": round(float(t[st["i0"]] - t[0]), 3),
            "length_s": round(st["length_s"], 3), "samples": int(st["i1"] - st["i0"] + 1),
            "t_start": float(t[st["i0"]]), "t_end": float(t[st["i1"]]),
            "ended_by_motion": st["ended"], "restarts_after_nudges": st["restarts"],
            "min_required_s": min_parked},
        "rules": {"standing_still": "speed < %.3f m/s and turning < %.1f deg/s between neighbouring poses"
                                    % (SPEED_MAX, YAW_RATE_MAX_DEG_S),
                  "nudge_radius_m": START_RADIUS_M,
                  "standard_error": "std of the slopes of %d equal pieces / sqrt(pieces); the textbook "
                                    "line-fit figure (se_ols) assumes independent readings and is far too small"
                                    % BLOCKS},
        "t_ref": t_ref,
        "correction": "heading(t) - rate * (t - t_ref); every position step turned by the correction at "
                      "its midpoint and added up again from the first pose",
        "before": end_numbers(t, x, y, yaw), "after": end_numbers(t, xc, yc, yawc),
        "publishes_tf": False,
        "plain_terms": "the recorded path is unchanged; this is a corrected copy that removes the "
                       "heading the gyroscope drifted while parked, assumed to go on at the same rate "
                       "while driving. Not a reference: it corrects heading only, never distance.",
    }
    with open(out + ".meta.json", "w") as fh:
        json.dump(meta, fh, indent=1)
    if not quiet:
        f = meta["fit"] or {}
        print("odom_debias: %s" % path)
        print("  parked stretch at the start: %.1f s (%s, %d poses)"
              % (st["length_s"], st["status"], meta["parked_stretch_used"]["samples"]))
        if f:
            print("  fitted drift: %+.3f deg/min, standard error %.3f (textbook %.4f, too small)"
                  % (f["rate_deg_per_min"], f["se_deg_per_min"], f["se_ols_deg_per_min"]))
        print("  rate applied: %+.4f deg/min (%s)" % (rate, source))
        print("  end gap %.3f m -> %.3f m;  end heading change %+.2f -> %+.2f deg"
              % (meta["before"]["end_gap_m"], meta["after"]["end_gap_m"],
                 meta["before"]["end_heading_change_deg"], meta["after"]["end_heading_change_deg"]))
        print("  wrote %s (+ .meta.json)" % out)
    return meta


def compare_live(path, log, min_parked=MIN_PARKED_OFFLINE_S):
    """Registered test D3's measurement: each scan pose the live view logged, against this
    file's corrected path at the same time. Done twice - with the rate fitted here, and with
    the live view's own rate - so an implementation fault is not confused with a slightly
    different rate from fewer, slower samples."""
    a, _, _ = read_tum(path)
    t, x, y = a[:, 0], a[:, 1], a[:, 2]
    yaw = np.unwrap(yaw_from_quat(a[:, 4], a[:, 5], a[:, 6], a[:, 7]))
    everything = np.atleast_1d(np.genfromtxt(log, delimiter=",", names=True))
    t_ref = float(everything["pose_time"][0])        # the live view counts from its first pose
    L = everything[everything["correcting"] > 0.5]
    if not len(L):
        return {"verdict_registered_D3": "N/A", "why": "the live log has no corrected poses"}
    st = find_start_parked(t, x, y, yaw, min_parked)
    rates = {"live": float(L["rate_deg_per_min"][-1])}
    if st["status"] == "measured":
        rates["offline"] = fit_rate(t[st["i0"]:st["i1"] + 1], yaw[st["i0"]:st["i1"] + 1])["rate_deg_per_min"]
    res = {"poses_compared": int(len(L)), "t_ref": t_ref}
    for name, r in rates.items():
        xc, yc, _ = correct_path(t, x, y, yaw, r, t_ref=t_ref)
        # anchor at t_ref, where the live view started: shift so the corrected path passes
        # through the recorded pose there (the live view starts from that raw pose)
        xc = xc - np.interp(t_ref, t, xc) + np.interp(t_ref, t, x)
        yc = yc - np.interp(t_ref, t, yc) + np.interp(t_ref, t, y)
        xi = np.interp(L["pose_time"], t, xc)
        yi = np.interp(L["pose_time"], t, yc)
        d = np.hypot(L["corr_x"] - xi, L["corr_y"] - yi)
        res["rate_" + name] = {"rate_deg_per_min": round(r, 4), "median_m": round(float(np.median(d)), 4),
                               "p95_m": round(float(np.percentile(d, 95)), 4), "max_m": round(float(d.max()), 4)}
    res["verdict_registered_D3"] = ("PASS" if res.get("rate_offline", res["rate_live"])["max_m"] <= 0.05
                                    else "FAIL")
    return res


def main(argv=None):
    ap = argparse.ArgumentParser(description="remove the parked gyroscope drift from a recorded odometry path "
                                             "(TUM file); publishes nothing")
    ap.add_argument("tum", help="the recorded path, e.g. wheel.tum")
    ap.add_argument("-o", "--out", default="", help="corrected TUM to write (default: <input>_debiased.tum)")
    ap.add_argument("--min-parked", type=float, default=MIN_PARKED_OFFLINE_S,
                    help="shortest parked stretch accepted, s (default %(default)s)")
    ap.add_argument("--rate", type=float, default=None, help="use this drift rate (deg/min) instead of fitting")
    ap.add_argument("--t-ref", type=float, default=None,
                    help="time the correction counts from (default: the first pose)")
    ap.add_argument("--compare-live", default="", help="a lidar_live_map.py pose log (CSV) to check")
    a = ap.parse_args(argv)
    if a.compare_live:
        print(json.dumps(compare_live(a.tum, a.compare_live, a.min_parked), indent=1))
        return 0
    out = a.out or os.path.splitext(a.tum)[0] + "_debiased.tum"
    if os.path.abspath(out) == os.path.abspath(a.tum):
        raise SystemExit("odom_debias: refusing to write over the input file")
    if not a.out and (os.path.exists(out) or os.path.exists(out + ".meta.json")):
        # the default lands beside the input, i.e. inside a recorded run's folder: never
        # replace an earlier result there without being told to by name
        raise SystemExit("odom_debias: %s already exists. Nothing written. Name the output with -o "
                         "(which overwrites what it names) or move the old one first." % out)
    debias_file(a.tum, out, a.min_parked, a.rate, a.t_ref)
    return 0


if __name__ == "__main__":
    sys.exit(main())
