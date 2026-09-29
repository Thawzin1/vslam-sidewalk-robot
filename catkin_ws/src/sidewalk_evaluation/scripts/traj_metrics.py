#!/usr/bin/env python3
"""
traj_metrics.py — The trajectory-error mathematics, implemented on numpy alone.

WHY THIS EXISTS
    The standard tool for this job is `evo`. It is good, and you should run it
    as an independent cross-check if you can install it. This module exists
    anyway, for three reasons:

      1. **It must run on the Jetson as delivered.** JetPack 5.1.5 ships Python
         3.8 and numpy. Adding pip packages to a robot that is about to be
         flashed, backed up, and handed to someone else is a liability.
      2. **A research result should not be a black box to its author.** If the
         thesis says "ATE RMSE 0.14 m", the author must be able to say exactly
         what was aligned to what, with or without scale, and why. That
         understanding is much easier to defend when the twenty lines that
         produce the number are in the repository.
      3. **Cross-checking needs two implementations.** Agreement between this
         module and `evo` on the same file is evidence; agreement of `evo` with
         itself is not.

    Every formula below follows the definitions in Sturm et al., "A Benchmark
    for the Evaluation of RGB-D SLAM Systems" (IROS 2012) — the TUM benchmark —
    with the Umeyama alignment from Umeyama, "Least-Squares Estimation of
    Transformation Parameters Between Two Point Patterns" (PAMI 1991).

CONVENTIONS USED THROUGHOUT
    * Trajectories are stored in **TUM format**: one line per pose,
      `timestamp tx ty tz qx qy qz qw`, timestamps in seconds, translation in
      metres, quaternion in **(x, y, z, w)** order and Hamilton convention.
      This matches what the prior ORB-SLAM3 work on the Jetson already emitted,
      so old files remain readable.
    * A pose is the transform **from the body frame into the world frame**:
      a world-frame point is `p_world = R @ p_body + t`. This is the ROS
      convention and the TUM convention; it is the opposite of some
      photogrammetry conventions, which is a classic source of trajectories
      that look mirrored.
    * Angles are returned in **degrees**, because that is what a mechanical
      engineer reads a spec sheet in. Internally everything is radians.

NOT A ROS NODE
    Pure numpy. Importable and testable with no ROS present.
"""
from __future__ import annotations

from pathlib import Path

from eval_common import require_numpy

np = require_numpy()


# --------------------------------------------------------------------------- #
# Rotation helpers
#
# We avoid tf.transformations deliberately: it is a ROS dependency, and this
# module has to work on a computer without ROS. The maths is short enough to own.
# --------------------------------------------------------------------------- #

def quat_to_mat(q) -> "np.ndarray":
    """Convert a single (x, y, z, w) quaternion to a 3x3 rotation matrix.

    Normalising first is not paranoia. Quaternions written to file at float
    precision, or produced by a tracker that renormalises only occasionally,
    routinely arrive with |q| off by 1e-6. Squaring that through a rotation
    matrix introduces a small scale factor that quietly contaminates every
    downstream distance.
    """
    q = np.asarray(q, dtype=float).reshape(4)
    n = np.linalg.norm(q)
    if n < 1e-12:
        # A zero quaternion is not recoverable. Identity is the least harmful
        # substitute, and the caller is warned by validate_trajectory().
        return np.eye(3)
    x, y, z, w = q / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def quats_to_mats(quats) -> "np.ndarray":
    """Vectorised quat_to_mat over an (N, 4) array -> (N, 3, 3)."""
    q = np.asarray(quats, dtype=float).reshape(-1, 4)
    n = np.linalg.norm(q, axis=1, keepdims=True)
    n[n < 1e-12] = 1.0
    q = q / n
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    m = np.empty((q.shape[0], 3, 3))
    m[:, 0, 0] = 1 - 2 * (y * y + z * z)
    m[:, 0, 1] = 2 * (x * y - z * w)
    m[:, 0, 2] = 2 * (x * z + y * w)
    m[:, 1, 0] = 2 * (x * y + z * w)
    m[:, 1, 1] = 1 - 2 * (x * x + z * z)
    m[:, 1, 2] = 2 * (y * z - x * w)
    m[:, 2, 0] = 2 * (x * z - y * w)
    m[:, 2, 1] = 2 * (y * z + x * w)
    m[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return m


def mat_to_quat(R) -> "np.ndarray":
    """3x3 rotation matrix -> (x, y, z, w) quaternion, via Shepperd's method.

    The branch on the largest diagonal element matters: the naive
    `w = sqrt(1 + trace)/2` formula loses all precision when the rotation
    approaches 180 degrees, which happens routinely on a robot that turns
    around at the end of a sidewalk.
    """
    R = np.asarray(R, dtype=float).reshape(3, 3)
    t = np.trace(R)
    if t > 0.0:
        s = np.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w])


def rotation_angle_deg(R) -> "np.ndarray":
    """Geodesic angle of a rotation (or a stack of rotations), in degrees.

    This is the single number that answers "how wrong is this orientation?",
    independent of axis convention: the smallest angle you must rotate through
    to go from one orientation to the other.

        theta = arccos((trace(R) - 1) / 2)

    The clip is essential. Floating-point error pushes the argument to 1.0000001
    for near-identity rotations — exactly the case that dominates a good
    trajectory — and arccos then returns NaN, silently poisoning the RMSE.
    """
    R = np.asarray(R, dtype=float)
    if R.ndim == 2:
        tr = np.trace(R)
    else:
        tr = np.trace(R, axis1=-2, axis2=-1)
    cos_theta = np.clip((tr - 1.0) / 2.0, -1.0, 1.0)
    return np.degrees(np.arccos(cos_theta))


def yaw_deg(R) -> "np.ndarray":
    """Heading angle about +Z, in degrees.

    For a differential-drive robot on a sidewalk, yaw is the orientation that
    actually matters — roll and pitch are constrained by the ground plane and
    the suspension. Reporting yaw separately from the full geodesic angle makes
    it obvious whether an orientation error is a real heading error or just
    the chassis rocking.
    """
    R = np.asarray(R, dtype=float)
    if R.ndim == 2:
        return np.degrees(np.arctan2(R[1, 0], R[0, 0]))
    return np.degrees(np.arctan2(R[:, 1, 0], R[:, 0, 0]))


# --------------------------------------------------------------------------- #
# Trajectory container and file I/O
# --------------------------------------------------------------------------- #

class Trajectory:
    """A time-stamped sequence of poses.

    Attributes
    ----------
    stamps : (N,) float seconds
    xyz    : (N, 3) metres, world frame
    quat   : (N, 4) (x, y, z, w)
    name   : label used in plots, tables and log lines
    """

    def __init__(self, stamps, xyz, quat, name: str = "trajectory"):
        self.stamps = np.asarray(stamps, dtype=float).reshape(-1)
        self.xyz = np.asarray(xyz, dtype=float).reshape(-1, 3)
        self.quat = np.asarray(quat, dtype=float).reshape(-1, 4)
        self.name = name
        if not (len(self.stamps) == len(self.xyz) == len(self.quat)):
            raise ValueError(
                f"Trajectory '{name}' is internally inconsistent: "
                f"{len(self.stamps)} timestamps, {len(self.xyz)} positions, "
                f"{len(self.quat)} orientations.")

    def __len__(self) -> int:
        return len(self.stamps)

    @property
    def duration(self) -> float:
        return float(self.stamps[-1] - self.stamps[0]) if len(self) > 1 else 0.0

    @property
    def rotations(self) -> "np.ndarray":
        return quats_to_mats(self.quat)

    def subset(self, idx) -> "Trajectory":
        idx = np.asarray(idx, dtype=int)
        return Trajectory(self.stamps[idx], self.xyz[idx], self.quat[idx], self.name)

    def transformed(self, R, t, scale: float = 1.0) -> "Trajectory":
        """Apply `p -> scale * R @ p + t` to positions and `R` to orientations.

        Used to put an estimated trajectory into the reference frame after
        alignment. Note the scale multiplies translation only: a rotation
        cannot be scaled, and applying scale to orientation would be a
        category error that shows up as non-orthogonal rotation matrices.
        """
        R = np.asarray(R, dtype=float).reshape(3, 3)
        t = np.asarray(t, dtype=float).reshape(3)
        xyz = scale * (self.xyz @ R.T) + t
        mats = R @ self.rotations
        quat = np.array([mat_to_quat(m) for m in mats])
        return Trajectory(self.stamps, xyz, quat, self.name)


def load_tum(path, name: str = "") -> Trajectory:
    """Read a TUM-format trajectory file.

    Tolerates `#` comment lines, blank lines and both space and comma
    separation, because these files get produced by four different programs and
    at least one of them will eventually use commas.
    """
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(
            f"Trajectory file not found: {path}\n"
            f"Trajectories are written by `trajectory_recorder.py` into "
            f"logs/sidewalk_evaluation/. If you expected one here, check that "
            f"the recorder was actually running during the experiment — it "
            f"writes a line only when it receives a pose.")

    stamps, xyz, quat = [], [], []
    bad = 0
    for lineno, raw in enumerate(path.read_text().splitlines(), start=1):
        s = raw.strip()
        if not s or s.startswith("#"):
            continue
        parts = s.replace(",", " ").split()
        if len(parts) < 8:
            bad += 1
            continue
        try:
            vals = [float(p) for p in parts[:8]]
        except ValueError:
            bad += 1
            continue
        stamps.append(vals[0])
        xyz.append(vals[1:4])
        quat.append(vals[4:8])

    if not stamps:
        raise ValueError(
            f"{path} contains no usable poses "
            f"({bad} malformed line(s) skipped).\n"
            f"Expected TUM format: `timestamp tx ty tz qx qy qz qw`, one pose "
            f"per line. If the file has a different column order it must be "
            f"converted first — silently reinterpreting columns would produce "
            f"plausible-looking but meaningless numbers.")

    traj = Trajectory(stamps, xyz, quat, name or path.stem)
    # Timestamps out of order break every windowed metric below. Sorting is the
    # right fix; a tracker that emits out-of-order poses is worth knowing about,
    # which is why validate_trajectory() reports it.
    order = np.argsort(traj.stamps, kind="stable")
    return traj.subset(order)


def save_tum(path, traj: Trajectory, comment: str = "") -> Path:
    """Write a Trajectory back out in TUM format."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = ["# timestamp tx ty tz qx qy qz qw"]
    if comment:
        lines += [f"# {c}" for c in comment.splitlines()]
    for t, p, q in zip(traj.stamps, traj.xyz, traj.quat):
        lines.append(f"{t:.9f} {p[0]:.6f} {p[1]:.6f} {p[2]:.6f} "
                     f"{q[0]:.7f} {q[1]:.7f} {q[2]:.7f} {q[3]:.7f}")
    path.write_text("\n".join(lines) + "\n")
    return path


def validate_trajectory(traj: Trajectory) -> dict:
    """Sanity-check a trajectory and describe anything suspicious.

    This runs before every evaluation. Most "the SLAM is broken" reports turn
    out to be one of these: duplicate timestamps from a node publishing twice,
    a stationary recording where the robot never moved, or unnormalised
    quaternions from a tracker that skipped renormalisation.
    """
    issues = []
    n = len(traj)
    if n < 2:
        issues.append(f"only {n} pose(s) — nothing can be measured")

    if n >= 2:
        dt = np.diff(traj.stamps)
        if np.any(dt <= 0):
            issues.append(f"{int(np.sum(dt <= 0))} non-increasing timestamp(s) "
                          f"(duplicate or out-of-order poses)")
        gaps = dt[dt > 0]
        if len(gaps):
            median_dt = float(np.median(gaps))
            big = int(np.sum(gaps > 10 * max(median_dt, 1e-6)))
            if big:
                issues.append(f"{big} gap(s) longer than 10x the median pose "
                              f"interval ({median_dt * 1000:.1f} ms) — the "
                              f"tracker probably lost and regained tracking")

    qn = np.linalg.norm(traj.quat, axis=1)
    off = int(np.sum(np.abs(qn - 1.0) > 1e-3))
    if off:
        issues.append(f"{off} quaternion(s) not unit-norm (max deviation "
                      f"{float(np.max(np.abs(qn - 1.0))):.2e}) — normalised on read")

    if not np.all(np.isfinite(traj.xyz)):
        issues.append("non-finite position values present (NaN or inf)")

    span = float(np.max(np.linalg.norm(traj.xyz - traj.xyz[0], axis=1))) if n else 0.0
    if n >= 2 and span < 0.05:
        issues.append(f"the trajectory never moves more than {span * 100:.1f} cm "
                      f"from its start — was the robot actually driven?")

    return {
        "n_poses": n,
        "duration_s": traj.duration,
        "mean_rate_hz": (n - 1) / traj.duration if traj.duration > 0 else 0.0,
        "path_length_m": path_length(traj),
        "max_displacement_m": span,
        "issues": issues,
    }


def split_at_gaps(traj: Trajectory, gap_factor: float = 10.0):
    """Split a trajectory into clean segments at its tracking-loss gaps.

    Same definition of "gap" as validate_trajectory(): an inter-pose interval
    longer than `gap_factor` x the median interval. validate_trajectory only
    COUNTS them; this returns the (start, end_exclusive) index ranges of the
    clean stretches between them, longest-friendly for slicing with
    Trajectory.subset(range(a, b)).

    Why this exists: a tracker that loses and regains tracking teleports
    across each gap, and whole-run statistics (RMSE, Sim(3) scale) then
    measure the teleports, not the tracking. Per-segment evaluation is how
    the two are told apart — the 2026-08-22 outdoor run's 2.15x Sim(3)
    "calibration error" warning was resolved exactly this way.
    """
    n = len(traj)
    if n < 2:
        return [(0, n)] if n else []
    dt = np.diff(traj.stamps)
    pos = dt[dt > 0]
    if not len(pos):
        return [(0, n)]
    thresh = gap_factor * max(float(np.median(pos)), 1e-6)
    cut_after = np.nonzero(dt > thresh)[0]      # gap between i and i+1
    segments, start = [], 0
    for i in cut_after:
        segments.append((start, int(i) + 1))
        start = int(i) + 1
    segments.append((start, n))
    return segments


# --------------------------------------------------------------------------- #
# Temporal association
# --------------------------------------------------------------------------- #

def associate(ref: Trajectory, est: Trajectory, max_diff: float = 0.02,
              offset: float = 0.0):
    """Pair each estimate pose with the nearest reference pose in time.

    Returns (idx_ref, idx_est) index arrays of equal length.

    WHY THIS IS SUBTLE
        Two trackers running on the same data emit poses at different rates and
        on different clocks. Comparing pose number 100 of one with pose number
        100 of the other is meaningless. We must compare *the same instant*.

        `max_diff` is the tolerance. At walking pace (~1.4 m/s) a 20 ms
        mismatch is 2.8 cm of genuine motion that will be charged to the
        algorithm as error. That is already comparable to a good ATE, which is
        why the default is tight and why the association statistics are logged:
        if a large fraction of poses fail to associate, the comparison is not
        trustworthy and you need to fix clock sync, not squint at the number.

        `offset` shifts the estimate's clock. Use it when two stacks were
        timestamped against different clock sources; determine it once by
        cross-correlation or from a known sync event, and record it in the log.
    """
    if len(ref) == 0 or len(est) == 0:
        return np.array([], dtype=int), np.array([], dtype=int)

    est_t = est.stamps + offset
    ref_t = ref.stamps

    # searchsorted gives the insertion point; the nearest reference sample is
    # then either that index or the one before it. O(N log N) rather than the
    # O(N*M) double loop the TUM reference script uses.
    pos = np.searchsorted(ref_t, est_t)
    pos = np.clip(pos, 1, len(ref_t) - 1) if len(ref_t) > 1 else np.zeros_like(pos)
    left = np.clip(pos - 1, 0, len(ref_t) - 1)
    right = np.clip(pos, 0, len(ref_t) - 1)
    d_left = np.abs(ref_t[left] - est_t)
    d_right = np.abs(ref_t[right] - est_t)
    best = np.where(d_left <= d_right, left, right)
    best_d = np.minimum(d_left, d_right)

    keep = best_d <= max_diff
    idx_est = np.nonzero(keep)[0]
    idx_ref = best[keep]

    # One reference pose must not be consumed twice: with a slow reference and
    # a fast estimate that would duplicate the same ground-truth sample and
    # artificially flatten the error. Keep the closest match per reference pose.
    if len(idx_ref):
        order = np.lexsort((best_d[keep], idx_ref))
        idx_ref_s, idx_est_s = idx_ref[order], idx_est[order]
        first = np.ones(len(idx_ref_s), dtype=bool)
        first[1:] = idx_ref_s[1:] != idx_ref_s[:-1]
        idx_ref, idx_est = idx_ref_s[first], idx_est_s[first]
        order2 = np.argsort(idx_est)
        idx_ref, idx_est = idx_ref[order2], idx_est[order2]

    return idx_ref, idx_est


# --------------------------------------------------------------------------- #
# Alignment
# --------------------------------------------------------------------------- #

def umeyama(src, dst, with_scale: bool = False):
    """Least-squares similarity alignment: find (R, t, c) minimising

        sum_i || dst_i - (c * R @ src_i + t) ||^2

    Parameters
    ----------
    src : (N, 3) points to be moved   (the ESTIMATE)
    dst : (N, 3) points to move onto  (the REFERENCE)

    Returns (R, t, c).

    WHY ALIGNMENT IS NECESSARY AT ALL
        A SLAM system defines its own world origin — usually wherever it
        happened to start, with whatever orientation the camera had at that
        instant. The reference trajectory has a different origin. Comparing raw
        coordinates would measure the arbitrary difference in origins, not the
        quality of the mapping. Rigidly aligning first removes exactly the
        degrees of freedom the algorithm was never asked to determine.

    WHY SCALE IS OPTIONAL AND USUALLY OFF
        Monocular SLAM has no way to know metric scale, so evaluating it
        requires solving for `c` too (Sim(3) alignment). We are running a
        **stereo** camera with a 120 mm baseline and a metric IMU: scale is
        observable, and an error in it is a real error. So the default is
        SE(3), `with_scale=False`, and `c` is returned as 1.0.

        The estimated scale is still reported separately as a diagnostic. If
        Sim(3) alignment finds c = 1.04, the stereo extrinsics are 4% off —
        which usually means the baseline in the calibration is wrong, or the
        wrong resolution's intrinsics were loaded.

    THE DETERMINANT CORRECTION
        Without the `S` matrix, the SVD can return a reflection (det = -1)
        instead of a rotation when the point cloud is nearly planar — which a
        sidewalk trajectory very much is, since the robot stays on the ground.
        A reflected "alignment" fits beautifully and is physically nonsense.
        This is the single most common bug in hand-rolled ATE code.
    """
    src = np.asarray(src, dtype=float).reshape(-1, 3)
    dst = np.asarray(dst, dtype=float).reshape(-1, 3)
    n = src.shape[0]
    if n < 3:
        raise ValueError(
            f"Umeyama alignment needs at least 3 associated poses; got {n}. "
            f"With fewer, the alignment is under-determined and any ATE it "
            f"produced would be meaningless.")

    mu_src = src.mean(axis=0)
    mu_dst = dst.mean(axis=0)
    src_c = src - mu_src
    dst_c = dst - mu_dst

    # Cross-covariance. Note the order: (dst^T src) / n, so that R maps src->dst.
    sigma = (dst_c.T @ src_c) / n
    U, D, Vt = np.linalg.svd(sigma)

    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1.0

    R = U @ S @ Vt

    if with_scale:
        var_src = float(np.sum(src_c ** 2) / n)
        c = float(np.trace(np.diag(D) @ S) / var_src) if var_src > 1e-12 else 1.0
    else:
        c = 1.0

    t = mu_dst - c * (R @ mu_src)
    return R, t, c


def align_trajectory(ref: Trajectory, est: Trajectory, idx_ref, idx_est,
                     mode: str = "se3", n_align: int = -1):
    """Align `est` onto `ref` and return (aligned_est_full, info).

    mode
        "se3"    rigid alignment, no scale. The default for stereo/VIO.
        "sim3"   similarity alignment, scale solved for. For monocular, or as a
                 diagnostic of stereo scale error.
        "origin" align only the first `n_align` poses. This is the honest choice
                 when you want to see raw accumulated drift rather than the
                 best-fit whole-path error — it answers "if I trust the start,
                 where does it end up?" rather than "what is the best this
                 trajectory could look like?".
        "none"   no alignment; both trajectories are assumed already in the
                 same frame (e.g. both are outputs of the same TF tree).

    The alignment is computed on the *associated* subset but applied to the
    *whole* estimated trajectory, so plots show every estimated pose.
    """
    mode = (mode or "se3").lower()
    src = est.xyz[idx_est]
    dst = ref.xyz[idx_ref]

    if mode == "none":
        R, t, c = np.eye(3), np.zeros(3), 1.0
    elif mode == "origin":
        k = len(src) if n_align <= 0 else min(n_align, len(src))
        if k < 3:
            raise ValueError(
                "Origin alignment needs at least 3 poses at the start of the "
                "run. Increase `n_align`, or stand the robot still for a second "
                "before driving so there are poses to align on.")
        R, t, c = umeyama(src[:k], dst[:k], with_scale=False)
    elif mode == "sim3":
        R, t, c = umeyama(src, dst, with_scale=True)
    else:
        R, t, c = umeyama(src, dst, with_scale=False)

    # Always report what a scale-solving alignment *would* have found, even in
    # SE(3) mode. A scale factor far from 1.0 is a calibration alarm.
    try:
        _, _, c_diag = umeyama(src, dst, with_scale=True)
    except ValueError:
        c_diag = float("nan")

    aligned = est.transformed(R, t, c)
    # `scale_applied`/`scale_estimated_sim3` are CORRECTION factors: the number
    # the estimate must be multiplied by to match the reference. A reader wants
    # the opposite question answered — "how wrong is the estimate's own scale?"
    # — so derive it explicitly rather than leaving the direction to be guessed.
    # est_size / ref_size = 1/c, so a c of 0.9643 is an estimate 3.7% too big.
    oversize_pct = (100.0 * (1.0 / c_diag - 1.0)
                    if np.isfinite(c_diag) and abs(c_diag) > 1e-12
                    else float("nan"))
    info = {
        "mode": mode,
        "scale_applied": c,
        "scale_estimated_sim3": c_diag,
        "estimate_oversize_pct": oversize_pct,
        "n_used_for_alignment": int(len(src) if mode != "origin"
                                    else min(n_align if n_align > 0 else len(src), len(src))),
        "rotation_deg": float(rotation_angle_deg(R)),
        "translation_m": float(np.linalg.norm(t)),
    }
    return aligned, info


# --------------------------------------------------------------------------- #
# Absolute Trajectory Error
# --------------------------------------------------------------------------- #

def _stats(values, unit: str = "") -> dict:
    """Standard descriptive statistics for an error series."""
    v = np.asarray(values, dtype=float)
    v = v[np.isfinite(v)]
    if v.size == 0:
        return {"n": 0, "unit": unit}
    return {
        "n": int(v.size),
        "rmse": float(np.sqrt(np.mean(v ** 2))),
        "mean": float(np.mean(v)),
        "median": float(np.median(v)),
        "std": float(np.std(v)),
        "min": float(np.min(v)),
        "max": float(np.max(v)),
        "p95": float(np.percentile(v, 95)),
        "unit": unit,
    }


def compute_ate(ref: Trajectory, est_aligned: Trajectory, idx_ref, idx_est) -> dict:
    """Absolute Trajectory Error over the associated, aligned poses.

    Translational ATE_i = || p_ref,i - p_est,i ||  after alignment.
    Rotational  ATE_i = geodesic angle of  R_ref,i^T @ R_est,i.

    RMSE is the headline number because it is what the literature quotes, but
    it is dominated by the worst excursion. Median tells you what a typical
    pose looks like, and a large RMSE/median ratio is the signature of a few
    catastrophic moments (a tracking loss, a bad loop closure) rather than
    uniformly mediocre accuracy. Both are reported for that reason.
    """
    p_ref = ref.xyz[idx_ref]
    p_est = est_aligned.xyz[idx_est]
    trans_err = np.linalg.norm(p_ref - p_est, axis=1)

    R_ref = quats_to_mats(ref.quat[idx_ref])
    R_est = quats_to_mats(est_aligned.quat[idx_est])
    # R_ref^T R_est is the residual rotation expressed in the reference body
    # frame — the rotation you would still have to apply to make them agree.
    R_err = np.einsum("nji,njk->nik", R_ref, R_est)
    rot_err = rotation_angle_deg(R_err)

    result = {
        "translation": _stats(trans_err, "m"),
        "rotation": _stats(rot_err, "deg"),
        "per_pose": {
            "stamps": (ref.stamps[idx_ref] - ref.stamps[idx_ref][0]).tolist()
            if len(idx_ref) else [],
            "trans_err_m": trans_err.tolist(),
            "rot_err_deg": rot_err.tolist(),
        },
    }
    # Also split the error along/across the direction of travel. On a sidewalk
    # these behave very differently: lateral error puts the robot in the road,
    # longitudinal error just makes it early or late.
    if len(idx_ref) > 2:
        d = np.gradient(p_ref, axis=0)
        norms = np.linalg.norm(d, axis=1, keepdims=True)
        norms[norms < 1e-9] = 1.0
        fwd = d / norms
        e = p_est - p_ref
        along = np.abs(np.einsum("ij,ij->i", e, fwd))
        across = np.linalg.norm(e - fwd * np.einsum("ij,ij->i", e, fwd)[:, None], axis=1)
        result["along_track"] = _stats(along, "m")
        result["cross_track"] = _stats(across, "m")
    return result


# --------------------------------------------------------------------------- #
# Relative Pose Error
# --------------------------------------------------------------------------- #

def compute_rpe(ref: Trajectory, est: Trajectory, idx_ref, idx_est,
                delta: float = 1.0, delta_unit: str = "m") -> dict:
    """Relative Pose Error over a fixed spatial or temporal separation.

    For each pair of poses separated by `delta`, RPE measures the difference
    between the motion the reference made and the motion the estimate made:

        E_i = (Q_i^-1 Q_{i+d})^-1 (P_i^-1 P_{i+d})

    where Q are reference poses and P are estimated poses.

    WHY THIS IS THE MORE HONEST METRIC
        RPE needs **no alignment**. It compares increments, and an increment is
        invariant to where you put the world origin. That makes it immune to
        the objection that ATE can be flattered by a lucky global fit.

        It also isolates local quality. A system with excellent per-step
        odometry but a slow global drift has small RPE and large ATE. A system
        with jittery per-step estimates but a good loop closure has large RPE
        and small ATE. Reporting only one of the two hides half the behaviour.

    delta_unit
        "m"      pairs separated by `delta` metres of reference path length.
                 Preferred: it makes drift figures comparable between a slow
                 careful run and a fast one.
        "s"      pairs separated by `delta` seconds.
        "frames" pairs separated by `delta` associated poses. Simplest, but the
                 result depends on the recording rate, so it is the least
                 comparable across runs.

    Note the result is reported both as a raw pose difference and normalised
    per unit of delta, since "3 cm of error" only means something once you know
    it accumulated over 1 m rather than over 100 m.
    """
    if len(idx_ref) < 2:
        return {"error": "fewer than 2 associated poses; RPE not computable"}

    R_ref_all = quats_to_mats(ref.quat[idx_ref])
    p_ref_all = ref.xyz[idx_ref]
    R_est_all = quats_to_mats(est.quat[idx_est])
    p_est_all = est.xyz[idx_est]
    t_all = ref.stamps[idx_ref]

    n = len(idx_ref)
    pairs = []

    if delta_unit == "frames":
        step = max(1, int(round(delta)))
        pairs = [(i, i + step) for i in range(0, n - step)]
    elif delta_unit == "s":
        # For each start index, find the first index at least `delta` later.
        j = np.searchsorted(t_all, t_all + delta, side="left")
        pairs = [(i, int(j[i])) for i in range(n) if j[i] < n]
    else:  # metres of reference path
        seg = np.linalg.norm(np.diff(p_ref_all, axis=0), axis=1)
        cum = np.concatenate([[0.0], np.cumsum(seg)])
        j = np.searchsorted(cum, cum + delta, side="left")
        pairs = [(i, int(j[i])) for i in range(n) if j[i] < n]

    if not pairs:
        return {"error": f"no pose pairs separated by {delta} {delta_unit} exist "
                         f"in this trajectory (total path "
                         f"{path_length(ref):.1f} m, duration {ref.duration:.1f} s)"}

    trans_err, rot_err, actual_delta = [], [], []
    for i, j in pairs:
        # Motion the reference made between i and j, in frame i.
        dR_ref = R_ref_all[i].T @ R_ref_all[j]
        dp_ref = R_ref_all[i].T @ (p_ref_all[j] - p_ref_all[i])
        # Motion the estimate made over the same interval, in its own frame i.
        dR_est = R_est_all[i].T @ R_est_all[j]
        dp_est = R_est_all[i].T @ (p_est_all[j] - p_est_all[i])
        # The residual between the two motions.
        E_R = dR_ref.T @ dR_est
        E_p = dp_est - dp_ref
        trans_err.append(float(np.linalg.norm(E_p)))
        rot_err.append(float(rotation_angle_deg(E_R)))
        actual_delta.append(float(np.linalg.norm(dp_ref)))

    trans_err = np.asarray(trans_err)
    rot_err = np.asarray(rot_err)
    actual = np.asarray(actual_delta)
    denom = np.where(actual > 1e-6, actual, np.nan)

    return {
        "delta": delta,
        "delta_unit": delta_unit,
        "n_pairs": len(pairs),
        "translation": _stats(trans_err, "m"),
        "rotation": _stats(rot_err, "deg"),
        "translation_pct_of_delta": _stats(100.0 * trans_err / denom, "%"),
        "rotation_per_metre": _stats(rot_err / denom, "deg/m"),
    }


# --------------------------------------------------------------------------- #
# Drift
# --------------------------------------------------------------------------- #

def path_length(traj: Trajectory) -> float:
    """Total distance travelled along the trajectory, in metres.

    This is the denominator of every drift figure, so it is worth stating what
    it is *not*: it is not the straight-line distance from start to finish, and
    it is not the wheel-odometry distance. It is the sum of the straight-line
    hops between consecutive recorded poses, which slightly *underestimates*
    true arc length on a curve — by less than 0.1% at 30 Hz and walking pace,
    but by a lot if you decimate the trajectory first. Do not decimate.
    """
    if len(traj) < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(traj.xyz, axis=0), axis=1)))


def compute_drift(ref: Trajectory, est_aligned: Trajectory, idx_ref, idx_est,
                  segment_lengths=(5.0, 10.0, 20.0, 40.0, 80.0)) -> dict:
    """Drift as a fraction of distance travelled — the KITTI-style measure.

    For every segment of the reference path of a given length, we measure how
    far apart the estimate and the reference have moved by the end of that
    segment, having agreed at its start. Averaging over all start points and
    over several segment lengths gives a figure that is largely independent of
    how long the experiment happened to be.

    WHY NOT JUST FINAL ERROR / TOTAL DISTANCE
        Because a single number from a single run is one sample of a random
        process. Drift is a random walk: two runs of the same system over the
        same route can differ by a factor of two purely by chance. Averaging
        over hundreds of overlapping segments is a far more stable estimator,
        and it is what makes results comparable to published KITTI numbers.

    Returns translational drift in **percent of distance travelled** and
    rotational drift in **degrees per metre**, per segment length and overall.
    """
    if len(idx_ref) < 2:
        return {"error": "fewer than 2 associated poses; drift not computable"}

    p_ref = ref.xyz[idx_ref]
    R_ref = quats_to_mats(ref.quat[idx_ref])
    p_est = est_aligned.xyz[idx_est]
    R_est = quats_to_mats(est_aligned.quat[idx_est])

    seg = np.linalg.norm(np.diff(p_ref, axis=0), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    total = float(cum[-1])

    per_length = {}
    all_t_pct, all_r_perm = [], []

    for L in segment_lengths:
        if total < L:
            per_length[f"{L:g}m"] = {
                "n_segments": 0,
                "note": f"route is only {total:.1f} m long; no {L:g} m segment exists",
            }
            continue
        ends = np.searchsorted(cum, cum + L, side="left")
        t_pct, r_perm = [], []
        for i in range(len(cum)):
            j = int(ends[i])
            if j >= len(cum):
                continue
            actual_L = cum[j] - cum[i]
            if actual_L < 1e-6:
                continue
            # Re-anchor both trajectories at the start of the segment, then
            # compare where each ended up. This is exactly "agree here, how far
            # apart are you L metres later".
            dR_ref = R_ref[i].T @ R_ref[j]
            dp_ref = R_ref[i].T @ (p_ref[j] - p_ref[i])
            dR_est = R_est[i].T @ R_est[j]
            dp_est = R_est[i].T @ (p_est[j] - p_est[i])
            t_pct.append(100.0 * float(np.linalg.norm(dp_est - dp_ref)) / actual_L)
            r_perm.append(float(rotation_angle_deg(dR_ref.T @ dR_est)) / actual_L)
        if t_pct:
            per_length[f"{L:g}m"] = {
                "n_segments": len(t_pct),
                "translation_pct": _stats(t_pct, "%"),
                "rotation_deg_per_m": _stats(r_perm, "deg/m"),
            }
            all_t_pct.extend(t_pct)
            all_r_perm.extend(r_perm)

    # The simple end-of-run figure as well, clearly labelled, because it is what
    # people intuitively expect and it is useful to see both.
    final_err = float(np.linalg.norm(p_ref[-1] - p_est[-1]))

    return {
        "path_length_m": total,
        "segment_lengths_m": list(segment_lengths),
        "per_segment_length": per_length,
        "overall_translation_pct": _stats(all_t_pct, "%") if all_t_pct else {"n": 0},
        "overall_rotation_deg_per_m": _stats(all_r_perm, "deg/m") if all_r_perm else {"n": 0},
        "final_position_error_m": final_err,
        "final_error_pct_of_path": (100.0 * final_err / total) if total > 1e-6 else None,
    }


# --------------------------------------------------------------------------- #
# Loop closure and the closed-loop proxy for ground truth
# --------------------------------------------------------------------------- #

def loop_closure_gap(traj: Trajectory, return_radius: float = 1.0,
                     min_path_before_return: float = 10.0) -> dict:
    """Measure how far the estimate says the robot is from where it started,
    at the moment it has physically returned to the start.

    THE PROBLEM THIS SOLVES
        ATE requires a reference trajectory. We have no motion-capture system,
        no RTK GNSS, and a sidewalk. So what is the truth?

        A closed loop is the answer available for free. If the robot is driven
        out and back to a physically marked start point — a chalk cross, a tile
        joint, a taped square — then the *true* displacement between the first
        and last pose is known to be zero, to whatever precision the driver can
        park the robot on the mark. Everything the estimator reports at the end
        is therefore accumulated error.

        This gives one very high-quality datum instead of a full trajectory. It
        cannot produce a per-pose ATE curve. It absolutely can produce a
        credible, defensible drift percentage, and it is the standard method
        when mocap is unavailable.

    ASSUMPTIONS, STATED PLAINLY
        1. The robot really did return to the same spot. The parking accuracy
           (a few centimetres, if a physical mark is used) is the noise floor of
           this measurement and must be reported alongside the result.
        2. The robot returned in the same *orientation*, if the yaw gap is to
           be interpreted. Mark a heading line, not just a point.
        3. The run is long enough that accumulated drift exceeds the parking
           error. Below about 10 m of path this measures the driver, not the
           algorithm — hence `min_path_before_return`.

    Parameters
    ----------
    return_radius : how close, in the ESTIMATE's own coordinates, a later pose
        must come to the start before we call it a loop candidate. Generous by
        default, because a drifting estimate may report itself several metres
        away from a start it is physically standing on.
    """
    if len(traj) < 2:
        return {"closed": False, "reason": "trajectory too short"}

    d_from_start = np.linalg.norm(traj.xyz - traj.xyz[0], axis=1)
    seg = np.linalg.norm(np.diff(traj.xyz, axis=0), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seg)])

    eligible = np.nonzero(cum >= min_path_before_return)[0]
    if eligible.size == 0:
        return {
            "closed": False,
            "reason": (f"path length {cum[-1]:.1f} m is below the "
                       f"{min_path_before_return:.1f} m minimum; over such a "
                       f"short route the parking error would dominate the drift "
                       f"being measured"),
            "path_length_m": float(cum[-1]),
        }

    R0 = quat_to_mat(traj.quat[0])
    R_end = quat_to_mat(traj.quat[-1])

    # The end-of-run gap: what the estimator claims, having been driven back to
    # a start point we know it is standing on.
    end_gap = float(d_from_start[-1])
    end_yaw_gap = float(rotation_angle_deg(R0.T @ R_end))

    # The closest approach after the minimum path, for runs where the operator
    # stopped slightly past the mark.
    k = int(eligible[np.argmin(d_from_start[eligible])])
    closest = float(d_from_start[k])

    detected = closest <= return_radius
    total = float(cum[-1])

    out = {
        "closed": bool(detected),
        "path_length_m": total,
        "end_to_start_gap_m": end_gap,
        "end_to_start_yaw_gap_deg": end_yaw_gap,
        "closest_approach_m": closest,
        "closest_approach_index": k,
        "closest_approach_path_m": float(cum[k]),
        "drift_pct_of_path": (100.0 * end_gap / total) if total > 1e-6 else None,
        "yaw_drift_deg_per_m": (end_yaw_gap / total) if total > 1e-6 else None,
        "assumption": ("Assumes the robot was physically returned to its marked "
                       "start pose. The gap reported here is accumulated "
                       "estimator error PLUS however far off the mark the robot "
                       "was actually parked; record that parking error and "
                       "subtract it in interpretation."),
    }
    if not detected:
        out["reason"] = (
            f"the estimate never comes within {return_radius:.2f} m of its own "
            f"start after {min_path_before_return:.1f} m of travel (closest "
            f"{closest:.2f} m). Either the route was not a loop, or the drift "
            f"is larger than the search radius — check the trajectory plot "
            f"before concluding either.")
    return out


def detect_correction_jumps(traj: Trajectory, max_speed_mps: float = 2.0,
                            max_yaw_rate_dps: float = 180.0,
                            sigma_factor: float = 6.0) -> dict:
    """Find the discontinuities where the SLAM back-end rewrote history.

    WHAT A LOOP CLOSURE LOOKS LIKE IN THE DATA
        The front-end tracks smoothly. Then the back-end recognises a place it
        has seen before, adds a constraint, re-optimises the pose graph, and the
        current pose estimate **teleports** — sometimes by metres. The robot did
        not move; the map's opinion of where it was changed.

        That jump is the correction magnitude, and it is a genuinely useful
        metric. Large corrections mean the system had accumulated a lot of drift
        before recognising the place — good recall, poor odometry. Frequent tiny
        corrections mean a well-constrained graph. No corrections at all on a
        route that definitely revisits itself means loop closure is not firing,
        which is a configuration bug worth finding.

    HOW WE DETECT IT
        A physical limit and a statistical one, and we require both, because
        each alone produces false positives:

        * Physical: this robot cannot exceed `max_speed_mps`. Any inter-pose
          translation implying more than that is not motion.
        * Statistical: the jump must also exceed `sigma_factor` standard
          deviations of the run's own inter-pose motion, so a fast straight
          section is not flagged just for being fast.

    LIMITATION, STATED HONESTLY
        This detects corrections applied to the *live* pose stream. A system
        that publishes only the final optimised trajectory shows no jumps at all
        — not because it never corrected, but because the correction was applied
        before we ever saw the data. For RTAB-Map, record the live
        `/rtabmap/localization_pose` or the TF stream, not the post-hoc export,
        if you want this metric to mean anything.
    """
    if len(traj) < 3:
        return {"n_corrections": 0, "reason": "trajectory too short"}

    dt = np.diff(traj.stamps)
    dt = np.where(dt > 1e-6, dt, np.nan)
    dp = np.linalg.norm(np.diff(traj.xyz, axis=0), axis=1)
    speed = dp / dt

    R = quats_to_mats(traj.quat)
    dR = np.einsum("nji,njk->nik", R[:-1], R[1:])
    dtheta = rotation_angle_deg(dR)
    yaw_rate = dtheta / dt

    finite = np.isfinite(dp)
    med = float(np.median(dp[finite])) if finite.any() else 0.0
    mad = float(np.median(np.abs(dp[finite] - med))) if finite.any() else 0.0
    # 1.4826 converts median-absolute-deviation to a standard-deviation
    # equivalent for Gaussian data; using MAD rather than std keeps the
    # threshold from being inflated by the very jumps we are looking for.
    robust_sigma = 1.4826 * mad
    # A perfectly regular pose stream — constant speed, no noise, which is
    # exactly what the synthetic self-test data looks like — has MAD == 0. That
    # would make the statistical threshold infinite and hide every jump,
    # including a deliberately injected one. Floor the estimated spread at 10%
    # of the typical step so the test stays meaningful on clean data.
    robust_sigma = max(robust_sigma, 0.1 * med if med > 0 else 1e-6)
    stat_thresh = med + sigma_factor * robust_sigma

    # NaN comparisons are already False in numpy, so a missing dt simply fails
    # both tests rather than producing a spurious detection.
    is_jump = ((speed > max_speed_mps) | (yaw_rate > max_yaw_rate_dps)) & \
              (dp > stat_thresh)

    idx = np.nonzero(is_jump)[0]
    events = [{
        "index": int(i + 1),
        "time_s": float(traj.stamps[i + 1] - traj.stamps[0]),
        "translation_m": float(dp[i]),
        "rotation_deg": float(dtheta[i]),
        "implied_speed_mps": float(speed[i]) if np.isfinite(speed[i]) else None,
    } for i in idx]

    mags = np.array([e["translation_m"] for e in events]) if events else np.array([])

    return {
        "n_corrections": len(events),
        "events": events,
        "magnitude": _stats(mags, "m") if mags.size else {"n": 0, "unit": "m"},
        "largest_correction_m": float(np.max(mags)) if mags.size else 0.0,
        "total_correction_m": float(np.sum(mags)) if mags.size else 0.0,
        "detection_threshold_m": float(stat_thresh) if np.isfinite(stat_thresh) else None,
        "assumed_max_speed_mps": max_speed_mps,
        "note": ("Zero corrections on a route that revisits itself means loop "
                 "closure never fired — check the detection threshold and the "
                 "vocabulary/memory settings, not the odometry."),
    }


# --------------------------------------------------------------------------- #
# Tracking status: losses, relocalization, latency, dropped frames
# --------------------------------------------------------------------------- #

VALID_STATUSES = ("OK", "LOST", "RELOCALIZED", "INITIALIZING")


def parse_status_log(path) -> list:
    """Read a tracking-status event file.

    Format: one event per line, `timestamp STATUS [optional note]`, where
    STATUS is one of OK / LOST / RELOCALIZED / INITIALIZING. `#` comments and
    blank lines are ignored. `trajectory_recorder.py` writes this file
    alongside each trajectory.

    A deliberately dumb format: it is greppable, diffable, human-writable when
    you need to annotate a run by hand, and it survives a node being killed
    mid-write.
    """
    path = Path(path)
    if not path.exists():
        return []
    events = []
    for raw in path.read_text().splitlines():
        s = raw.strip()
        if not s or s.startswith("#"):
            continue
        parts = s.split()
        if len(parts) < 2:
            continue
        try:
            ts = float(parts[0])
        except ValueError:
            continue
        status = parts[1].upper()
        events.append({"t": ts, "status": status,
                       "note": " ".join(parts[2:]) if len(parts) > 2 else ""})
    return sorted(events, key=lambda e: e["t"])


def compute_tracking_reliability(events: list, run_duration_s: float = 0.0) -> dict:
    """Tracking losses, relocalization success rate, and time-to-relocalize.

    DEFINITIONS, because these terms are used loosely in the literature
        * **Tracking loss**: a transition into LOST. Counted once per episode,
          not once per LOST message, so a system that spams the status at 30 Hz
          while lost is not penalised a hundred times for one failure.
        * **Relocalization success**: a LOST episode that ends in RELOCALIZED
          (or OK) before the recording stops. An episode still LOST at the end
          of the run counts as a failure, not as missing data — that is the
          honest reading, since the robot never recovered.
        * **Time-to-relocalize**: seconds from entering LOST to recovering.
          Reported as a distribution, because the mean is misleading: typical
          recovery is under a second and the occasional failure takes thirty.

    WHY THIS MATTERS OPERATIONALLY
        A sidewalk robot that loses tracking is a robot standing in the middle
        of a pedestrian route. The time-to-relocalize distribution, not its
        mean, is what determines whether that is acceptable.
    """
    if not events:
        return {
            "available": False,
            "reason": ("no tracking-status events were recorded. Relocalization "
                       "and tracking-loss metrics require the SLAM stack to "
                       "publish its own status; run `trajectory_recorder.py` "
                       "with the correct `status_topic` for this stack, or "
                       "annotate the run by hand.")
        }

    episodes = []
    lost_since = None
    n_lost_msgs = 0
    for ev in events:
        st = ev["status"]
        if st == "LOST":
            n_lost_msgs += 1
            if lost_since is None:
                lost_since = ev["t"]
        elif st in ("OK", "RELOCALIZED"):
            if lost_since is not None:
                episodes.append({
                    "lost_at_s": lost_since,
                    "recovered_at_s": ev["t"],
                    "duration_s": ev["t"] - lost_since,
                    "recovered": True,
                    "how": st,
                })
                lost_since = None
    if lost_since is not None:
        end = events[-1]["t"]
        episodes.append({
            "lost_at_s": lost_since,
            "recovered_at_s": None,
            "duration_s": end - lost_since,
            "recovered": False,
            "how": "never — still lost when the recording ended",
        })

    n = len(episodes)
    n_ok = sum(1 for e in episodes if e["recovered"])
    reloc_times = np.array([e["duration_s"] for e in episodes if e["recovered"]])

    t0, t1 = events[0]["t"], events[-1]["t"]
    duration = run_duration_s if run_duration_s > 0 else max(t1 - t0, 1e-9)
    time_lost = float(sum(e["duration_s"] for e in episodes))

    return {
        "available": True,
        "run_duration_s": duration,
        "tracking_loss_count": n,
        "tracking_loss_rate_per_min": 60.0 * n / duration if duration > 0 else None,
        "relocalization_success_count": n_ok,
        "relocalization_success_rate_pct": (100.0 * n_ok / n) if n else None,
        "time_to_relocalize": _stats(reloc_times, "s") if reloc_times.size
                              else {"n": 0, "unit": "s"},
        "total_time_lost_s": time_lost,
        "availability_pct": 100.0 * (1.0 - time_lost / duration) if duration > 0 else None,
        "episodes": episodes,
        "n_lost_messages": n_lost_msgs,
    }


def compute_latency(pairs, expected_hz: float = 0.0) -> dict:
    """Processing latency and dropped-frame rate from (capture, output) stamps.

    `pairs` is an iterable of (sensor_capture_time, result_published_time), both
    in seconds on the same clock.

    WHAT LATENCY MEANS HERE, PRECISELY
        The interval between the shutter closing on the ZED X and a pose being
        available to the navigation stack. It is NOT the SLAM algorithm's
        compute time alone — it includes GMSL2 transport, the ZED SDK's depth
        computation on the GPU, ROS serialisation, and the queue the message sat
        in. That total is the number that matters, because that is the age of
        the information move_base is steering on.

        At 1.4 m/s, every 100 ms of latency is 14 cm of position that the
        planner does not know about yet. This is why the metric is here and not
        in an appendix.

    DROPPED FRAMES
        Estimated from the timing pattern: if capture stamps arrive at a nominal
        `expected_hz` and one interval is close to an integer multiple of the
        nominal period, the intervening frames were dropped. This is inference,
        not ground truth — the authoritative count is the ZED SDK's own dropped
        frame counter, which `system_monitor.py` reads when available.
    """
    pairs = [(float(a), float(b)) for a, b in pairs]
    if not pairs:
        return {"available": False,
                "reason": "no latency samples were recorded"}

    cap = np.array([p[0] for p in pairs])
    out = np.array([p[1] for p in pairs])
    lat_ms = 1000.0 * (out - cap)

    negative = int(np.sum(lat_ms < 0))
    result = {
        "available": True,
        "n_samples": len(pairs),
        "latency_ms": _stats(lat_ms[lat_ms >= 0], "ms"),
    }
    if negative:
        result["warning"] = (
            f"{negative} sample(s) had the result timestamped BEFORE the "
            f"capture. That is physically impossible and means two different "
            f"clocks are in play — most likely the Jetson and another computer with no "
            f"NTP agreement. Those samples were excluded; fix the clock sync "
            f"before trusting any latency number from this run.")

    order = np.argsort(cap)
    cap_s = cap[order]
    if len(cap_s) > 2:
        gaps = np.diff(cap_s)
        nominal = 1.0 / expected_hz if expected_hz > 0 else float(np.median(gaps))
        result["nominal_period_s"] = nominal
        result["effective_rate_hz"] = 1.0 / float(np.median(gaps)) if np.median(gaps) > 0 else None
        if nominal > 0:
            # round() rather than floor(): jitter makes a real interval land
            # slightly either side of the nominal period, and floor would call
            # every slightly-early frame a drop.
            multiples = np.round(gaps / nominal)
            dropped = int(np.sum(np.maximum(multiples - 1, 0)))
            expected_total = len(cap_s) + dropped
            result["dropped_frames_estimated"] = dropped
            result["dropped_frame_pct"] = 100.0 * dropped / expected_total if expected_total else 0.0
            result["frame_interval_jitter_ms"] = float(np.std(gaps) * 1000.0)
            result["dropped_frames_method"] = (
                "inferred from gaps in capture timestamps against a nominal "
                f"period of {nominal * 1000:.2f} ms; cross-check against the "
                "ZED SDK's own dropped-frame counter where available")
    return result


# --------------------------------------------------------------------------- #
# Top-level evaluation
# --------------------------------------------------------------------------- #

def evaluate(ref: Trajectory, est: Trajectory, *,
             align_mode: str = "se3",
             max_time_diff: float = 0.02,
             time_offset: float = 0.0,
             rpe_deltas=((1.0, "m"), (5.0, "m"), (10.0, "m"), (1.0, "s")),
             segment_lengths=(5.0, 10.0, 20.0, 40.0, 80.0),
             max_speed_mps: float = 2.0,
             align_window_s: float = 0.0,
             reference_kind: str = "unspecified") -> dict:
    """Run the whole metric suite for one estimated trajectory against one
    reference, and return a single nested dictionary of results.

    `align_window_s` > 0 fits the alignment on only the first N seconds of the
    associated overlap, then measures error over the WHOLE run. Why this
    exists: least-squares alignment over a diverging trajectory drags the fit
    away from the start to appease the tail (Zhang & Scaramuzza, IROS 2018,
    show first-state vs all-state alignment swinging ATE ~150%) — the
    2026-08-22 outdoor run opened with 12.3 m of "error" while the robot was
    still stationary, purely from tail-drag. The window makes "error at the
    start" mean what it says. The policy is recorded in the output
    (`alignment.align_window_s`) so a report can print it.

    `reference_kind` is recorded verbatim in the output and MUST be set
    honestly. It is the answer to the question a reader will ask first:
    *what are you calling truth?* Recognised values used elsewhere in this
    package (the canonical list is `eval_common.REFERENCE_KINDS`):

        "motion_capture"   external mm-accurate system. We have no lab mocap;
                           this kind is legitimate for public datasets whose
                           truth came from one (e.g. TUM RGB-D, 100 Hz mocap).
        "rtk_gnss"         centimetre GNSS. Not available on a sidewalk under
                           trees, which is most of the intended environment.
        "simulator_ground_truth"
                           the physics engine's own model pose, republished
                           from /gazebo/model_states. Exact within the
                           simulated world — genuine truth for a SIMULATED
                           robot, and says nothing about the real one.
        "closed_loop"      the route returns to a physically marked start; only
                           the endpoint is truth. See loop_closure_gap().
        "cross_stack"      another SLAM stack's output used as the reference.
                           This measures DISAGREEMENT, not error. Two systems
                           can agree beautifully and both be wrong. Never
                           present a cross-stack ATE as accuracy.
        "wheel_odometry"   the robot base's own odometry. Good short-term, has
                           its own unbounded drift, and is degraded by wheel
                           slip on wet concrete. A useful sanity reference over
                           tens of metres, not over hundreds.
        "nav_repeat_run"   another run of the SAME route, recorded by
                           sidewalk_navigation's waypoint runner, using the
                           same localisation system. This measures
                           REPEATABILITY — how consistently the system
                           reproduces its own answer — and is emphatically NOT
                           accuracy. Two runs can agree to a centimetre while
                           both being two metres from where the robot
                           physically was. Written by
                           `ingest_navigation_run.py --repeatability`; see
                           `nav_run.py` for the schema that carries it.
        "synthetic_ground_truth"
                           a mathematically generated trajectory with a known,
                           deliberately injected error. SELF-TEST ONLY — it
                           verifies this code, never a robot. A report that
                           finds robot data labelled with it refuses to print
                           numbers (eval_common.UNQUOTABLE_KINDS).
    """
    report: dict = {
        "reference_name": ref.name,
        "estimate_name": est.name,
        "reference_kind": reference_kind,
    }

    report["reference_validation"] = validate_trajectory(ref)
    report["estimate_validation"] = validate_trajectory(est)

    idx_ref, idx_est = associate(ref, est, max_diff=max_time_diff, offset=time_offset)
    n_assoc = len(idx_ref)
    report["association"] = {
        "n_associated": n_assoc,
        "n_reference": len(ref),
        "n_estimate": len(est),
        "associated_pct_of_estimate": 100.0 * n_assoc / len(est) if len(est) else 0.0,
        "max_time_diff_s": max_time_diff,
        "time_offset_s": time_offset,
    }
    if n_assoc < 3:
        report["error"] = (
            f"only {n_assoc} pose(s) could be matched in time between "
            f"'{ref.name}' and '{est.name}' within {max_time_diff * 1000:.0f} ms. "
            f"Reference spans {ref.stamps[0]:.3f}..{ref.stamps[-1]:.3f}, "
            f"estimate spans {est.stamps[0]:.3f}..{est.stamps[-1]:.3f}. "
            f"If those ranges do not overlap the two files are from different "
            f"runs; if they do overlap, the clocks disagree — measure the offset "
            f"and pass it as `time_offset`.")
        return report
    if report["association"]["associated_pct_of_estimate"] < 50.0:
        report.setdefault("warnings", []).append(
            f"only {report['association']['associated_pct_of_estimate']:.0f}% of "
            f"estimated poses found a reference match. The metrics below are "
            f"computed on a biased subset and should be treated as indicative "
            f"only.")

    # Optionally fit the alignment on only the opening window. The mask
    # narrows the FIT; every metric below still runs over the full
    # association, so nothing is hidden — the tail's error is simply no longer
    # allowed to drag the fit away from the start.
    fit_ref, fit_est = idx_ref, idx_est
    if align_window_s > 0:
        t0 = float(ref.stamps[idx_ref][0])
        m = ref.stamps[idx_ref] <= t0 + align_window_s
        if int(m.sum()) < 3:
            report["error"] = (
                f"--align-window {align_window_s:g}s leaves only "
                f"{int(m.sum())} associated pose(s) to fit the alignment on "
                f"(3 needed). Widen the window or drop it.")
            return report
        fit_ref, fit_est = idx_ref[m], idx_est[m]

    est_aligned, align_info = align_trajectory(ref, est, fit_ref, fit_est,
                                               mode=align_mode)
    align_info["align_window_s"] = float(align_window_s)
    if align_window_s > 0:
        align_info["align_fit_poses"] = int(len(fit_ref))
    report["alignment"] = align_info
    if abs(align_info["scale_estimated_sim3"] - 1.0) > 0.02:
        _pct = align_info["estimate_oversize_pct"]
        _dir = "too large" if _pct > 0 else "too small"
        report.setdefault("warnings", []).append(
            f"Sim(3) alignment finds a correction factor of "
            f"{align_info['scale_estimated_sim3']:.4f}, i.e. the estimated "
            f"trajectory is {abs(_pct):.1f}% {_dir}. "
            f"On a stereo camera scale is directly observable "
            f"from the 120 mm baseline, so this points at a calibration error — "
            f"check that the intrinsics loaded match the resolution actually "
            f"being streamed.")

    report["ate"] = compute_ate(ref, est_aligned, idx_ref, idx_est)

    report["rpe"] = {}
    for delta, unit in rpe_deltas:
        report["rpe"][f"{delta:g}{unit}"] = compute_rpe(
            ref, est, idx_ref, idx_est, delta=delta, delta_unit=unit)

    report["drift"] = compute_drift(ref, est_aligned, idx_ref, idx_est,
                                    segment_lengths=segment_lengths)
    report["loop_closure_gap"] = loop_closure_gap(est)
    report["corrections"] = detect_correction_jumps(est, max_speed_mps=max_speed_mps)

    # Keep the aligned trajectory so the plotting code does not have to redo
    # the alignment and risk using different settings than the numbers did.
    report["_aligned"] = est_aligned
    report["_idx_ref"] = idx_ref
    report["_idx_est"] = idx_est
    return report


def headline(report: dict) -> dict:
    """Pull the handful of numbers that belong in a comparison table.

    Deliberately small. A table with thirty columns communicates nothing; these
    are the six figures that actually distinguish one SLAM stack from another.
    """
    def dig(*keys, default=None):
        node = report
        for k in keys:
            if not isinstance(node, dict) or k not in node:
                return default
            node = node[k]
        return node

    rpe_key = next((k for k in dig("rpe", default={}) or {} if k.endswith("m")), None)
    # RMSE alone saturates on outliers (a handful of tracking teleports
    # dominate it entirely — ENGINEERING_NOTES.md section 4.3), so median and 95th travel with
    # it everywhere.
    return {
        "ate_rmse_m": dig("ate", "translation", "rmse"),
        "ate_median_m": dig("ate", "translation", "median"),
        "ate_p95_m": dig("ate", "translation", "p95"),
        "ate_rot_rmse_deg": dig("ate", "rotation", "rmse"),
        "rpe_trans_rmse_m": dig("rpe", rpe_key, "translation", "rmse") if rpe_key else None,
        "rpe_trans_median_m": dig("rpe", rpe_key, "translation", "median") if rpe_key else None,
        "rpe_trans_p95_m": dig("rpe", rpe_key, "translation", "p95") if rpe_key else None,
        "rpe_delta": rpe_key,
        "drift_pct": dig("drift", "overall_translation_pct", "mean"),
        "drift_deg_per_m": dig("drift", "overall_rotation_deg_per_m", "mean"),
        "final_error_m": dig("drift", "final_position_error_m"),
        "path_length_m": dig("drift", "path_length_m"),
        "loop_gap_m": dig("loop_closure_gap", "end_to_start_gap_m"),
        "n_corrections": dig("corrections", "n_corrections"),
        "n_associated": dig("association", "n_associated"),
    }


# --------------------------------------------------------------------------- #
# Synthetic data — how this module is tested without hardware
# --------------------------------------------------------------------------- #

def synthetic_loop(duration_s: float = 120.0, rate_hz: float = 30.0,
                   radius: float = 8.0, speed_mps: float = 1.0,
                   name: str = "synthetic_reference") -> Trajectory:
    """A perfectly known closed-loop trajectory: a circuit at constant speed.

    Used as the reference in `--self-test`. The robot faces along its direction
    of travel, as a differential-drive base necessarily does — it cannot move
    sideways, and building that constraint into the synthetic data means the
    test exercises the same geometry the real robot will.
    """
    n = max(4, int(duration_s * rate_hz))
    t = np.linspace(0.0, duration_s, n)
    omega = speed_mps / radius
    theta = omega * t
    xyz = np.stack([radius * np.cos(theta) - radius,
                    radius * np.sin(theta),
                    np.zeros_like(theta)], axis=1)
    # Heading is tangent to the circle: yaw = theta + 90 deg.
    yaw = theta + np.pi / 2.0
    quat = np.stack([np.zeros_like(yaw), np.zeros_like(yaw),
                     np.sin(yaw / 2.0), np.cos(yaw / 2.0)], axis=1)
    return Trajectory(t + 1.7e9, xyz, quat, name)


def corrupt(traj: Trajectory, *, drift_pct: float = 1.0, noise_m: float = 0.01,
            yaw_bias_deg_per_m: float = 0.05, scale_err: float = 0.0,
            seed: int = 0, name: str = "synthetic_estimate") -> Trajectory:
    """Corrupt a trajectory the way a real SLAM system does.

    Three distinct error mechanisms, because they have distinct signatures and
    a test that only adds white noise proves almost nothing:

    1. **White measurement noise** (`noise_m`) — independent per pose. Shows up
       in RPE, barely moves ATE, because it averages out.
    2. **Random-walk drift** (`drift_pct`) — the integral of noise. Barely moves
       RPE, dominates ATE. This is the mechanism that makes long runs hard.
    3. **Systematic yaw bias** (`yaw_bias_deg_per_m`) — a constant heading error
       per metre, from miscalibrated extrinsics or a gyro bias. The nastiest of
       the three, because the resulting position error grows with the SQUARE of
       distance: a constant curvature error bends the whole path.

    Also applies a rigid transform, so the alignment step has something real to
    undo and a bug in it cannot pass unnoticed.
    """
    rng = np.random.default_rng(seed)
    n = len(traj)
    seg = np.concatenate([[0.0], np.linalg.norm(np.diff(traj.xyz, axis=0), axis=1)])
    cum = np.cumsum(seg)

    # (2) Random walk whose standard deviation grows as sqrt(distance), scaled
    # so that the expected error at the end is drift_pct of the path length.
    total = max(cum[-1], 1e-9)
    step_sigma = (drift_pct / 100.0) * total / np.sqrt(max(n - 1, 1))
    walk = np.cumsum(rng.normal(0.0, step_sigma, size=(n, 3)), axis=0)
    walk[:, 2] *= 0.3          # vertical drift is smaller: the ground is flat

    # (3) Systematic yaw bias integrated over distance.
    yaw_err = np.radians(yaw_bias_deg_per_m) * cum
    xyz = traj.xyz.copy()
    for i in range(n):
        c, s = np.cos(yaw_err[i]), np.sin(yaw_err[i])
        Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        xyz[i] = Rz @ xyz[i]

    xyz = xyz * (1.0 + scale_err) + walk
    # (1) White noise on top.
    xyz += rng.normal(0.0, noise_m, size=xyz.shape)

    mats = quats_to_mats(traj.quat)
    quat = np.empty_like(traj.quat)
    for i in range(n):
        c, s = np.cos(yaw_err[i]), np.sin(yaw_err[i])
        Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        quat[i] = mat_to_quat(Rz @ mats[i])

    # A rigid offset representing the estimator's arbitrary world origin.
    a = np.radians(37.0)
    R0 = np.array([[np.cos(a), -np.sin(a), 0.0],
                   [np.sin(a), np.cos(a), 0.0],
                   [0.0, 0.0, 1.0]])
    t0 = np.array([12.0, -5.0, 0.4])
    out = Trajectory(traj.stamps, xyz, quat, name)
    return out.transformed(R0, t0)


if __name__ == "__main__":
    # Module self-test. Verifies the maths against cases with known answers,
    # so a refactor that breaks the alignment cannot pass silently.
    print("traj_metrics self-test")
    ok = True

    ref = synthetic_loop()
    print(f"  reference: {len(ref)} poses, {path_length(ref):.1f} m, "
          f"{ref.duration:.1f} s")

    # 1. Identity: a trajectory compared with itself must give exactly zero.
    r = evaluate(ref, Trajectory(ref.stamps, ref.xyz, ref.quat, "copy"))
    zero = r["ate"]["translation"]["rmse"]
    print(f"  identity ATE RMSE      : {zero:.3e} m  (expect ~0)")
    ok &= zero < 1e-9

    # 2. Rigid transform only: alignment must undo it completely.
    a = np.radians(53.0)
    R0 = np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])
    moved = Trajectory(ref.stamps, ref.xyz, ref.quat, "moved").transformed(
        R0, np.array([100.0, -40.0, 3.0]))
    r = evaluate(ref, moved)
    rigid = r["ate"]["translation"]["rmse"]
    print(f"  rigid-offset ATE RMSE  : {rigid:.3e} m  (expect ~0 after alignment)")
    ok &= rigid < 1e-6

    # 3. Known scale error: Sim(3) must recover it.
    #    NOTE THE DIRECTION. The estimate here is 1.05x too big. Umeyama solves
    #    dst ~= c*R*src with src = ESTIMATE and dst = REFERENCE, so the factor it
    #    returns is the one that SHRINKS the estimate back onto the reference:
    #    c = 1/1.05, not 1.05. Asserting 1.05 here fails on correct code.
    scaled = Trajectory(ref.stamps, ref.xyz * 1.05, ref.quat, "scaled")
    r = evaluate(ref, scaled, align_mode="sim3")
    got = r["alignment"]["scale_applied"]
    expect = 1.0 / 1.05
    print(f"  recovered scale        : {got:.5f}  (expect {expect:.5f} = 1/1.05)")
    ok &= abs(got - expect) < 1e-6
    # And the derived, human-facing figure must read back as the 5% oversize.
    size_err = r["alignment"]["estimate_oversize_pct"]
    print(f"  estimate oversize      : {size_err:+.3f} %  (expect +5.000)")
    ok &= abs(size_err - 5.0) < 1e-3

    # 4. Realistic corruption: numbers must be finite and in a sane range.
    est = corrupt(ref, drift_pct=1.0, noise_m=0.02, yaw_bias_deg_per_m=0.05, seed=7)
    r = evaluate(ref, est, reference_kind="synthetic_ground_truth")
    h = headline(r)
    print(f"  corrupted ATE RMSE     : {h['ate_rmse_m']:.4f} m")
    print(f"  corrupted RPE (1 m)    : {h['rpe_trans_rmse_m']:.4f} m")
    print(f"  corrupted drift        : {h['drift_pct']:.3f} %")
    ok &= h["ate_rmse_m"] is not None and 0.0 < h["ate_rmse_m"] < 20.0
    # The point of the corruption model: drift dominates ATE, noise dominates
    # RPE, so ATE must be much larger than the per-metre RPE.
    ok &= h["ate_rmse_m"] > h["rpe_trans_rmse_m"]

    # 5. split_at_gaps: one injected 5 s hole must yield exactly two clean
    #    segments whose lengths sum to the original pose count.
    keep = np.ones(len(ref), dtype=bool)
    t_hole = ref.stamps[0] + 40.0
    keep[(ref.stamps > t_hole) & (ref.stamps < t_hole + 5.0)] = False
    holed = ref.subset(np.nonzero(keep)[0])
    segs = split_at_gaps(holed)
    total = sum(b - a for a, b in segs)
    print(f"  split_at_gaps          : {len(segs)} segment(s) covering "
          f"{total}/{len(holed)} poses  (expect 2 segments, full coverage)")
    ok &= len(segs) == 2 and total == len(holed)

    # 6. align_window: fitting on only the first 10 s must still fully undo a
    #    rigid offset (the transform is global, so a correct fit from any
    #    window aligns the whole trajectory).
    r = evaluate(ref, moved, align_window_s=10.0)
    win = r["ate"]["translation"]["rmse"]
    print(f"  10s-window rigid ATE   : {win:.3e} m  (expect ~0; "
          f"fit on {r['alignment']['align_fit_poses']} poses)")
    ok &= win < 1e-6 and r["alignment"]["align_window_s"] == 10.0

    print(f"\n  {'PASS' if ok else 'FAIL'}")
    raise SystemExit(0 if ok else 1)
