#!/usr/bin/env python3
"""compare_lidar.py (copy for drive 4, review fixes 26 Sept: --tracking-label names the tracking-alone
path (in a fused drive the map nodes carry the BLEND's position), --reference-note adds to the footer, and the
right panel is split so tracking alone and corrected are never on one axis (ENGINEERING_NOTES.md rule 20); the best-fit
scale and the unapplied clock shift are printed on the figure. Numbers unchanged.)
compare_lidar.py - the camera's path drawn over the robot's LiDAR map, with the numbers.

WHAT THE REFERENCE IS - say it exactly, every time (ENGINEERING_NOTES.md rule 19)
    The robot's LiDAR map (self_navigation rtabmap_3d.launch, a colleague's
    work) builds its path from the wheel+IMU estimate /odometry/filtered, with
    RGBD/NeighborLinkRefining=false, and corrects it wherever its 3D scans
    match a place seen before. So the reference is:

        wheel+IMU odometry, corrected by LiDAR loop closures.

    It is an INDEPENDENT ESTIMATE, not ground truth. Its own error is not
    measured here: LiDAR range noise is 15-30 mm, the two LiDARs agree to about
    5 mm, and the LiDAR-to-camera mounting offset has never been measured.
    Report "agreement with an independent LiDAR estimate", never "error".

WHY THE REFERENCE IS FILLED IN FIRST
    The LiDAR map keeps a position about once a second (Rtabmap/DetectionRate=1),
    and so does the camera map, each at its own moments. Matched at the
    project's 50 ms tolerance, two once-a-second series pair up only ~10 % of
    the time. So the reference is filled in between its corrected positions
    with the 50 Hz wheel+IMU stream: every moment gets the wheel estimate,
    shifted by the correction the LiDAR map applied at the most recent node.
    At each LiDAR node it equals the LiDAR map exactly.

    *Plain terms: the LiDAR map gives a trusted position once a second; the
    wheels fill in the gaps between them, the way you would join dots.*

WHAT IT PRODUCES (in the run's results folder)
    lidar_reference_dense.tum     the filled-in reference
    lidar_comparison.png          LiDAR map + both paths | disagreement along the route
    agreement.json                the numbers, with every source file named
    lidar_comparison/             evaluate_trajectory.py's own outputs

  usage:
    compare_lidar.py <run folder>  [--time-offset S]
      the folder must hold lidar.tum (corrected LiDAR nodes, id in column 9),
      wheel.tum (/odometry/filtered), camera_corrected.tum and camera.tum;
      lidar_map.npz (render_map.py --npz) is drawn underneath if present.
"""
from __future__ import print_function

import argparse
import bisect
import json
import math
import os
import shutil
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SLAM_ROOT = os.path.dirname(os.path.dirname(HERE))   # copy lives in 03_methods/<folder>/
REPO = os.path.dirname(SLAM_ROOT)
EVAL = os.path.join(REPO, "catkin_ws", "src", "sidewalk_evaluation", "scripts",
                    "evaluate_trajectory.py")
RESULTS = os.path.join(REPO, "logs", "sidewalk_evaluation", "results")
TOL = 0.05          # 50 ms, measured on run 8 (03_methods/PIPELINE_CHECK_2026-09-23.md)
# The reference must itself close: the robot is parked on the start mark by eye
# (~0.1 m), so a reference ending more than 0.5 m from its start has drifted.
REF_GAP_OK = 0.5

TEAL, RUST, GREY, INK, FAINT = "#1F8A8C", "#C0532B", "#6E6A63", "#22252A", "#ECE9E3"


def read_tum(path):
    """-> array of rows t x y yaw (planar - the robot drives on a floor)."""
    rows = []
    with open(path) as fh:
        for line in fh:
            p = line.split()
            if not p or line.startswith("#") or len(p) < 8:
                continue
            t, x, y = float(p[0]), float(p[1]), float(p[2])
            qx, qy, qz, qw = (float(v) for v in p[4:8])
            rows.append((t, x, y, math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))))
    a = np.array(sorted(rows))
    return a


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def se2_compose(a, b):
    """a (+) b for rows of (x, y, yaw)."""
    c, s = np.cos(a[..., 2]), np.sin(a[..., 2])
    return np.stack([a[..., 0] + c * b[..., 0] - s * b[..., 1],
                     a[..., 1] + s * b[..., 0] + c * b[..., 1],
                     wrap(a[..., 2] + b[..., 2])], axis=-1)


def se2_inverse(a):
    c, s = np.cos(a[..., 2]), np.sin(a[..., 2])
    return np.stack([-c * a[..., 0] - s * a[..., 1], s * a[..., 0] - c * a[..., 1], -a[..., 2]], axis=-1)


def interp(traj, t):
    """Planar pose of traj (t x y yaw) at times t, linear; NaN outside its span."""
    out = np.full((len(t), 3), np.nan)
    ok = (t >= traj[0, 0]) & (t <= traj[-1, 0])
    tt = t[ok]
    out[ok, 0] = np.interp(tt, traj[:, 0], traj[:, 1])
    out[ok, 1] = np.interp(tt, traj[:, 0], traj[:, 2])
    out[ok, 2] = wrap(np.interp(tt, traj[:, 0], np.unwrap(traj[:, 3])))
    return out


def densify(nodes, wheel):
    """Corrected LiDAR nodes + dense wheel stream -> dense corrected reference."""
    w_at_nodes = interp(wheel, nodes[:, 0])
    keep = ~np.isnan(w_at_nodes[:, 0])
    nodes, w_at_nodes = nodes[keep], w_at_nodes[keep]
    corr = se2_compose(nodes[:, 1:4], se2_inverse(w_at_nodes))    # C_k = T_lidar(k) (-) T_wheel(t_k)
    t = wheel[:, 0]
    sel = (t >= nodes[0, 0]) & (t <= nodes[-1, 0])
    t, w = t[sel], wheel[sel, 1:4]
    k = np.searchsorted(nodes[:, 0], t, side="right") - 1
    k = np.clip(k, 0, len(nodes) - 1)
    dense = se2_compose(corr[k], w)
    return np.column_stack([t, dense]), int(keep.sum())


def write_tum(path, traj, comment):
    with open(path, "w") as fh:
        fh.write("# %s\n# timestamp tx ty tz qx qy qz qw\n" % comment)
        for t, x, y, yaw in traj:
            fh.write("%.6f %.6f %.6f 0 0 0 %.8f %.8f\n" % (t, x, y, math.sin(yaw / 2), math.cos(yaw / 2)))


def planar_rmse_after_fit(ref, est):
    """SE(2) least-squares fit of est onto ref positions; RMSE of what is left."""
    A, B = est[:, :2], ref[:, :2]
    ma, mb = A.mean(0), B.mean(0)
    H = (A - ma).T @ (B - mb)
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[1] *= -1
        R = Vt.T @ U.T
    res = (A - ma) @ R.T + mb - B
    return float(np.sqrt((res ** 2).sum(1).mean()))


def estimate_offset(ref, est):
    """Clock offset between the two machines, from the data: the shift of the
    camera's timestamps that makes the two paths agree best. A diagnostic -
    it is applied only if asked, because it can also absorb real error."""
    def cost(dt):
        p = interp(ref, est[:, 0] + dt)
        ok = ~np.isnan(p[:, 0])
        return planar_rmse_after_fit(p[ok], est[ok, 1:3]) if ok.sum() > 20 else np.inf
    grid = np.arange(-3.0, 3.0001, 0.1)
    best = min(grid, key=cost)
    fine = np.arange(best - 0.1, best + 0.1001, 0.01)
    best = min(fine, key=cost)
    return float(round(best, 3)), cost(0.0), cost(best)


def along_track(ref):
    d = np.r_[0, np.cumsum(np.hypot(np.diff(ref[:, 1]), np.diff(ref[:, 2])))]
    return d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir")
    ap.add_argument("--time-offset", type=float, default=0.0,
                    help="seconds added to the camera's stamps (default 0: both "
                         "machines are expected on internet time - check it)")
    ap.add_argument("--tracking-label", default="camera, tracking alone")
    ap.add_argument("--reference-note", default="")
    ap.add_argument("--tracking-lw", type=float, default=1.0,
                    help="line width of the dotted map-nodes line on the map panel (drive 5 review: 2.4 to make it "
                         "visible; the default keeps drive 4's figure)")
    args = ap.parse_args()
    D = os.path.abspath(args.run_dir)
    run = os.path.basename(D)
    need = ["lidar.tum", "wheel.tum", "camera_corrected.tum"]
    missing = [n for n in need if not os.path.exists(os.path.join(D, n))]
    if missing:
        sys.exit("  missing in %s: %s" % (D, ", ".join(missing)))

    nodes = read_tum(os.path.join(D, "lidar.tum"))
    wheel = read_tum(os.path.join(D, "wheel.tum"))
    dense, used = densify(nodes, wheel)
    meta = {}
    if os.path.exists(os.path.join(D, "lidar.tum.meta.json")):
        with open(os.path.join(D, "lidar.tum.meta.json")) as fh:
            meta = json.load(fh)
    ref_path = os.path.join(D, "lidar_reference_dense.tum")
    write_tum(ref_path, dense, "LiDAR reference for %s: wheel+IMU odometry (/odometry/filtered) "
              "anchored to the LiDAR map's corrected nodes (%d nodes, %s loop closures); "
              "planar" % (run, used, meta.get("loop_closures", "?")))
    print("  reference: %d LiDAR nodes filled in to %d poses (%.1f Hz)"
          % (used, len(dense), len(dense) / max(dense[-1, 0] - dense[0, 0], 1e-9)))

    ests = [("camera_corrected", "camera_corrected.tum", "map's corrected path")]
    if os.path.exists(os.path.join(D, "camera.tum")):
        ests.append(("camera_tracking", "camera.tum", "tracking alone"))

    est_c = read_tum(os.path.join(D, "camera_corrected.tum"))
    off_est, rmse0, rmse_best = estimate_offset(dense, est_c)
    print("  clock check from the data: best shift %+.3f s (fit %.3f m) vs no shift (fit %.3f m)"
          % (off_est, rmse_best, rmse0))
    # Warn only when a shift makes a REAL difference. A badly bent map fits
    # poorly at every shift, and the search then wanders to whatever shift
    # happens to shave a few percent off - drive 1 "found" +3.1 s for a 3 %
    if abs(off_est) > 0.2 and rmse0 - rmse_best > max(0.05, 0.2 * rmse0):
        print("  !! the paths agree noticeably better with the camera shifted %+.2f s. Check the"
              " robot's clock (timedatectl) before trusting the numbers; re-run with"
              " --time-offset if it is really off." % off_est)

    name = "%s_vs_lidar" % run
    cmd = [sys.executable, EVAL, "--ref", ref_path, "--reference-kind", "cross_stack",
           "--max-time-diff", str(TOL), "--time-offset", str(args.time_offset),
           "--name", name, "--no-report"]
    for label, fn, _ in ests:
        cmd += ["--est", "%s=%s" % (label, os.path.join(D, fn))]
    rc = subprocess.call(cmd, stdout=subprocess.DEVNULL)
    res = os.path.join(RESULTS, name)
    if rc != 0 or not os.path.isdir(res):
        sys.exit("  evaluate_trajectory.py failed (exit %d) - run it by hand to see why" % rc)
    keep = os.path.join(D, "lidar_comparison")
    os.makedirs(keep, exist_ok=True)
    for fn in os.listdir(res):
        shutil.copy2(os.path.join(res, fn), keep)

    # ------------------------------------------------- the reference's own check
    # Every drive ends parked on the start mark, so the reference's own
    # start-to-end gap is a test of the reference itself. Drive 2's first
    # reference (the colleague's config: wheel+IMU, 0 LiDAR closures) ended
    # 20.7 m out - worse than the camera it was meant to check. A reference that
    # fails this must never be quoted as a yardstick.
    ref_gap = float(np.hypot(nodes[-1, 1] - nodes[0, 1], nodes[-1, 2] - nodes[0, 2]))
    ref_closures = meta.get("loop_closures")
    ref_valid = ref_gap <= REF_GAP_OK and (ref_closures or 0) > 0
    ref_verdict = ("passes its own check: ends %.2f m from its start, %s LiDAR loop closures"
                   % (ref_gap, ref_closures) if ref_valid else
                   "FAILED ITS OWN CHECK: ends %.2f m from its start although the robot parked "
                   "on the start mark, with %s LiDAR loop closures - the differences below "
                   "measure the reference's drift as much as the camera's" % (ref_gap, ref_closures))
    print("  reference " + ref_verdict)

    # ------------------------------------------------------------ numbers
    out = {"run": run, "reference_kind": "cross_stack",
           "reference_end_gap_m": round(ref_gap, 3), "reference_valid": ref_valid,
           "reference_verdict": ref_verdict,
           "reference": "LiDAR map (robot, self_navigation rtabmap_3d.launch): wheel+IMU "
                        "odometry corrected by LiDAR loop closures - an independent estimate, "
                        "not ground truth",
           "reference_exact": meta.get("exact"), "reference_loop_closures": meta.get("loop_closures"),
           "reference_nodes": used, "time_offset_applied_s": args.time_offset,
           "time_offset_estimated_s": off_est, "association_tolerance_s": TOL,
           "reference_file": os.path.relpath(ref_path, REPO), "estimates": {}}
    per_pose = {}
    for label, fn, desc in ests:
        with open(os.path.join(keep, "eval_%s.json" % label)) as fh:
            e = json.load(fh)
        a = e["ate"]["translation"]
        out["estimates"][label] = {
            "what": desc, "source_file": os.path.relpath(os.path.join(D, fn), REPO),
            "matched_pct": round(e["association"]["associated_pct_of_estimate"], 1),
            "n_matched": e["association"]["n_associated"],
            "disagreement_median_m": round(a["median"], 3),
            "disagreement_p95_m": round(a["p95"], 3),
            "disagreement_max_m": round(a["max"], 3),
            "disagreement_rmse_m": round(a["rmse"], 3),
            "sim3_scale_vs_reference": round(e["alignment"]["scale_estimated_sim3"], 4),
        }
        al = read_tum(os.path.join(keep, "aligned_%s.tum" % label))
        p = interp(dense, al[:, 0] + args.time_offset)
        ok = ~np.isnan(p[:, 0])
        dist = np.interp(al[ok, 0] + args.time_offset, dense[:, 0], along_track(dense))
        per_pose[label] = (al, dist, np.hypot(al[ok, 1] - p[ok, 0], al[ok, 2] - p[ok, 1]))
    with open(os.path.join(D, "agreement.json"), "w") as fh:
        json.dump(out, fh, indent=2)
    for label, v in out["estimates"].items():
        print("  %-17s median %.3f m, 95th pct %.3f m, max %.3f m  (%s%% of poses matched)"
              % (label, v["disagreement_median_m"], v["disagreement_p95_m"],
                 v["disagreement_max_m"], v["matched_pct"]))

    # ------------------------------------------------------------ figure
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.family": "serif",
                         "font.serif": ["Liberation Serif", "DejaVu Serif"], "font.size": 10})
    fig = plt.figure(figsize=(14, 7.6))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.1, 1], hspace=0.55)
    ax = fig.add_subplot(gs[:, 0])
    bxs = {"camera_tracking": fig.add_subplot(gs[0, 1]), "camera_corrected": fig.add_subplot(gs[1, 1])}
    npz = os.path.join(D, "lidar_map.npz")
    if os.path.exists(npz):
        m = np.load(npz)
        ax.imshow(m["img"], cmap="gray", vmin=0, vmax=1, extent=list(m["extent"]),
                  interpolation="nearest", origin="upper")
    ax.plot(dense[:, 1], dense[:, 2], color=GREY, lw=2.0, label="LiDAR reference (solid)")
    al, _, _ = per_pose["camera_corrected"]
    ax.plot(al[:, 1], al[:, 2], color=TEAL, lw=1.6, ls="--", label="camera, map's corrected path (dashed)")
    if "camera_tracking" in per_pose:
        tr = per_pose["camera_tracking"][0]
        ax.plot(tr[:, 1], tr[:, 2], color=RUST, lw=args.tracking_lw, ls=":", alpha=0.8 if args.tracking_lw <= 1.0 else 1.0,
                zorder=4,
                label="%s (dotted)" % args.tracking_label)
    ax.plot(dense[0, 1], dense[0, 2], "o", color=INK, ms=8, mec="white", mew=1.5)
    ax.annotate("start", (dense[0, 1], dense[0, 2]), textcoords="offset points",
                xytext=(8, 8), fontsize=9, color=INK)
    # Frame the route, not the whole map: walls far from the path say nothing
    # about this comparison and shrink the part that does.
    xs = np.r_[dense[:, 1], al[:, 1]]; ys = np.r_[dense[:, 2], al[:, 2]]
    ax.set_xlim(xs.min() - 3, xs.max() + 3); ax.set_ylim(ys.min() - 3, ys.max() + 3)
    ax.plot([xs.min() - 2.5, xs.min() - 0.5], [ys.min() - 2.5] * 2, color=INK, lw=3)
    ax.text(xs.min() - 1.5, ys.min() - 2.2, "2 m", ha="center", fontsize=8.5, color=INK)
    ax.set_aspect("equal")
    ax.set_xticks([]); ax.set_yticks([])
    for s in ax.spines.values():
        s.set_visible(False)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.01), fontsize=8.5, frameon=False, ncol=1)
    ax.set_title("The LiDAR map, with the camera's path drawn on it", loc="left", fontsize=11)

    names = {"camera_tracking": args.tracking_label,
             "camera_corrected": "camera map's corrected path (after its loop closures)"}
    for label, colour, lw in (("camera_tracking", RUST, 1.2), ("camera_corrected", TEAL, 1.6)):
        bx = bxs[label]
        if label not in per_pose:
            bx.set_visible(False)
            continue
        _, dist, err = per_pose[label]
        v = out["estimates"][label]
        bx.plot(dist, err, color=colour, lw=lw)
        bx.set_title("%s\nmedian %.2f m, 95th pct %.2f m; best-fit scale %.3f"
                     % (names[label], v["disagreement_median_m"], v["disagreement_p95_m"],
                        v["sim3_scale_vs_reference"]), loc="left", fontsize=9.5)
        bx.set_ylabel("apart (m)")
        bx.spines["top"].set_visible(False); bx.spines["right"].set_visible(False)
        bx.grid(color=FAINT)
    bx = bxs["camera_corrected"]
    bx.set_xlabel("distance along the route (m, from the LiDAR reference)")
    foot = ["Reference = wheel+IMU odometry corrected by %s LiDAR loop closures - an independent estimate, NOT ground truth."
            % meta.get("loop_closures", "?")]
    if args.reference_note:
        foot.append(args.reference_note)
    foot += ["Its own error is not measured here; the LiDAR-to-camera mounting offset has never been measured.",
             "Paths lined up by the best rotation + shift (no scaling); 'best-fit scale' = the size change that would fit best",
             "if scaling were allowed (not applied). Clock shift estimated %+.2f s, not applied." % off_est]
    bx.text(0, -0.30, "\n".join(foot), transform=bx.transAxes, fontsize=8, color=GREY, va="top")
    fig.suptitle("%s - agreement with an independent LiDAR estimate" % run, x=0.01, ha="left",
                 fontsize=13)
    if not ref_valid:
        fig.text(0.01, 0.905, "REFERENCE NOT USABLE: it " + ref_verdict.replace("FAILED", "failed"),
                 ha="left", fontsize=10.5, color=RUST, wrap=True)
    fig.subplots_adjust(left=0.02, right=0.98, top=0.9, bottom=0.2, wspace=0.12)
    png = os.path.join(D, "lidar_comparison.png")
    fig.savefig(png, dpi=150, facecolor="white")
    print("  figure: %s" % os.path.relpath(png, REPO))
    return 0


if __name__ == "__main__":
    sys.exit(main())
