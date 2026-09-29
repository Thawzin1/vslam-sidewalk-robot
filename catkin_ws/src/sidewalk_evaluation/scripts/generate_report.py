#!/usr/bin/env python3
"""
generate_report.py — Turn a directory of evaluation results into one HTML file.

WHY THIS EXISTS
    An evaluation that lives in a terminal scrollback is not a result. It has to
    survive being emailed to a supervisor, opened on a computer with no ROS, read
    six months later, and attached to a report.

    So the output is a **single self-contained HTML file**. Every figure is
    embedded as a base64 `data:` URI; there are no external images, no CDN
    links, no web fonts and no JavaScript libraries. Copy the one file anywhere
    and it renders identically, offline, forever. This is the same rule
    `tools/explain.py` follows, for the same reason: in a lab, the machine you
    need to show something on is always the one with no internet.

    The report is also written to be **read by someone who was not there**. Each
    section states what was measured, against what reference, and what the
    number means — not just the number.

USAGE
    ./generate_report.py --results logs/sidewalk_evaluation/results/20260720-1400
    ./generate_report.py --results DIR --out /tmp/report.html --title "Route A"

    Normally you do not call this directly: `evaluate_trajectory.py` and
    `compare_slam.py` invoke it for you once they have written their JSON.

INPUTS IT LOOKS FOR (all optional; it reports on whatever is present)
    eval_*.json                  per-stack trajectory metrics
    plotdata_*.json              per-stack arrays for the figures
    reference.tum, aligned_*.tum trajectories, for the map view
    depth_characterization.json  depth accuracy against a flat wall
    system_load.json             CPU / GPU / memory / latency
    comparison.json              the three-way comparison table
"""
from __future__ import annotations

import argparse
import base64
import html
import io
import json
import sys
from datetime import datetime
from pathlib import Path

from eval_common import (UNQUOTABLE_KINDS, MissingDependency, ProvenanceError,
                         RunLogger, fmt, require_matplotlib, require_numpy)

# --------------------------------------------------------------------------- #
# Presentation constants
#
# A fixed, colour-blind-safe palette assigned by stack name, so the same system
# is the same colour in every figure of every report. Nothing is more confusing
# in a comparison than a legend that changes between plots.
# --------------------------------------------------------------------------- #

STACK_COLOURS = {
    "reference":  "#111827",
    "rtabmap":    "#0072B2",
    "zed":        "#D55E00",
    "orbslam3":   "#009E73",
    "orb_slam3":  "#009E73",
    "odom":       "#CC79A7",
    "wheel_odom": "#CC79A7",
}
FALLBACK_COLOURS = ["#0072B2", "#D55E00", "#009E73", "#CC79A7",
                    "#56B4E9", "#E69F00", "#7F7F7F"]


def colour_for(name: str, i: int = 0) -> str:
    key = str(name).lower().replace("-", "_")
    for k, v in STACK_COLOURS.items():
        if k in key:
            return v
    return FALLBACK_COLOURS[i % len(FALLBACK_COLOURS)]


CSS = """
:root{--bg:#ffffff;--fg:#1f2430;--mut:#5b6472;--line:#e3e7ee;--acc:#0b5fa5;
--warn:#8a5a00;--warnbg:#fff8e6;--bad:#9b1c1c;--badbg:#fdecec;
--okbg:#eaf6ee;--ok:#15603a;--card:#fbfcfe;}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);
font:16px/1.62 -apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,Helvetica,Arial,sans-serif}
.wrap{max-width:1080px;margin:0 auto;padding:0 24px}
header.hd{background:linear-gradient(180deg,#f4f7fb,#ffffff);
border-bottom:1px solid var(--line);padding:36px 0 26px;margin-bottom:28px}
h1{font-size:30px;line-height:1.25;margin:6px 0 8px;letter-spacing:-.01em}
h2{font-size:21px;margin:0 0 14px;letter-spacing:-.01em}
h3{font-size:15px;margin:20px 0 8px;color:var(--mut);
text-transform:uppercase;letter-spacing:.06em}
.crumb{font-size:13px;color:var(--mut);text-transform:uppercase;letter-spacing:.09em}
.lede{font-size:18px;color:var(--mut);margin:0 0 12px;max-width:70ch}
section.card{border:1px solid var(--line);border-radius:10px;padding:22px 24px;
margin:0 0 22px;background:var(--card)}
section.card.hi{background:#f2f7fd;border-color:#cfe0f2}
p{margin:0 0 12px;max-width:78ch}
code{font:13px/1.5 ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;
background:#eef1f6;padding:1px 5px;border-radius:4px}
table{border-collapse:collapse;width:100%;font-size:14px;margin:6px 0 4px}
th,td{border-bottom:1px solid var(--line);padding:8px 10px;text-align:left;
vertical-align:top}
th{background:#eef2f8;font-weight:600;font-size:13px;
text-transform:uppercase;letter-spacing:.04em;color:var(--mut)}
td.num,th.num{text-align:right;font-variant-numeric:tabular-nums;
font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace}
td.best{background:var(--okbg);font-weight:700;color:var(--ok)}
.scroll{overflow-x:auto}
figure{margin:16px 0 6px}
figure img{width:100%;height:auto;border:1px solid var(--line);
border-radius:8px;background:#fff}
figcaption{font-size:13px;color:var(--mut);margin-top:8px;max-width:78ch}
.note{background:#f4f7fb;border-left:3px solid var(--acc);
padding:10px 14px;font-size:14px;margin:12px 0;border-radius:0 6px 6px 0}
.warn{background:var(--warnbg);border-left:3px solid #d19b00;
padding:10px 14px;font-size:14px;margin:12px 0;border-radius:0 6px 6px 0;
color:var(--warn)}
.bad{background:var(--badbg);border-left:3px solid #c53030;
padding:10px 14px;font-size:14px;margin:12px 0;border-radius:0 6px 6px 0;
color:var(--bad)}
.kpis{display:flex;flex-wrap:wrap;gap:12px;margin:8px 0 4px}
.kpi{flex:1 1 150px;border:1px solid var(--line);border-radius:8px;
padding:12px 14px;background:#fff}
.kpi .v{font-size:22px;font-weight:700;font-variant-numeric:tabular-nums;
font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace}
.kpi .k{font-size:12px;color:var(--mut);text-transform:uppercase;
letter-spacing:.05em;margin-top:2px}
footer.ft{border-top:1px solid var(--line);margin-top:34px;padding:20px 0 44px;
font-size:13px;color:var(--mut)}
ul{margin:0 0 12px;padding-left:22px}
li{margin:3px 0;max-width:78ch}
@media print{section.card{break-inside:avoid}header.hd{background:none}}
"""


def e(x) -> str:
    return html.escape(str(x if x is not None else ""))


# --------------------------------------------------------------------------- #
# Figures
#
# Each function returns raw PNG bytes, or None if the data it needs is absent.
# Returning None rather than raising is deliberate: a missing depth
# characterization should cost you one figure, not the whole report.
# --------------------------------------------------------------------------- #

def _fig_to_png(fig, plt, dpi: int = 130) -> bytes:
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=dpi, bbox_inches="tight",
                facecolor="white")
    plt.close(fig)
    return buf.getvalue()


def _style(ax):
    """House style: light grid, no top/right spines. Applied everywhere so the
    figures read as one document rather than a scrapbook."""
    ax.grid(True, alpha=0.25, linewidth=0.7)
    ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)


def fig_trajectory_xy(plt, ref_xy, estimates) -> bytes | None:
    """Top-down map view. The plot a human actually believes.

    Equal aspect ratio is non-negotiable here: a stretched axis makes a
    circular route look elliptical and invents a systematic error that is not
    there.

    The reference is drawn solid and every estimate dashed, and both say so
    in their legend entry. This is the project's standing convention for
    trajectory figures: a report is read on paper as often as on screen, and
    once it has been photocopied or printed on a monochrome printer the
    colour that separated the reference from the estimates is gone. Line
    style is the channel that survives that, so it carries the distinction
    that matters most and the colours are left to do the easier job of
    telling several estimates apart from each other.
    """
    if ref_xy is None and not estimates:
        return None
    fig, ax = plt.subplots(figsize=(7.4, 6.4))
    if ref_xy is not None and len(ref_xy):
        ax.plot(ref_xy[0], ref_xy[1], "-", color=STACK_COLOURS["reference"],
                lw=2.4, label="reference (solid)", zorder=5)
        ax.plot(ref_xy[0][0], ref_xy[1][0], "o", color="#15603a", ms=9,
                zorder=7, label="start")
        ax.plot(ref_xy[0][-1], ref_xy[1][-1], "s", color="#9b1c1c", ms=9,
                zorder=7, label="end (reference)")
    for i, (name, xy) in enumerate(estimates):
        if xy is None or not len(xy[0]):
            continue
        # The label is the stack's own name, supplied by the caller, so the
        # word "dashed" is appended rather than written into a fixed string.
        ax.plot(xy[0], xy[1], "--", color=colour_for(name, i), lw=1.5,
                alpha=0.9, label=f"{name} (dashed)")
        ax.plot(xy[0][-1], xy[1][-1], "x", color=colour_for(name, i), ms=9, mew=2)
    ax.set_xlabel("x  [m]")
    ax.set_ylabel("y  [m]")
    ax.set_title("Trajectory, viewed from above (after rigid alignment)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.legend(loc="best", fontsize=9, framealpha=0.92)
    _style(ax)
    return _fig_to_png(fig, plt)


def fig_error_vs_time(plt, series) -> bytes | None:
    """Position error against time, one line per stack.

    Read the *shape*, not the peak. A staircase means loop closures snapping
    the estimate back. A smooth upward curve means unbounded drift with no
    closures firing. A spike that never recovers means a tracking loss the
    system never undid.
    """
    series = [s for s in series if s[1] is not None and len(s[1])]
    if not series:
        return None
    fig, ax = plt.subplots(figsize=(9.2, 3.9))
    for i, (name, t, err) in enumerate(series):
        ax.plot(t, err, "-", lw=1.3, color=colour_for(name, i), label=name)
    ax.set_xlabel("time since start of run  [s]")
    ax.set_ylabel("absolute position error  [m]")
    ax.set_title("Absolute trajectory error over time")
    ax.legend(loc="best", fontsize=9)
    _style(ax)
    return _fig_to_png(fig, plt)


def fig_error_cdf(plt, np, series) -> bytes | None:
    """Cumulative distribution of per-pose error.

    More informative than a histogram for this quantity, because it answers the
    operational question directly: "for what fraction of the run was the robot
    within X of where it thought it was?" The 95th percentile — where the curve
    crosses 0.95 — is what a safety argument is built on, not the mean.
    """
    series = [s for s in series if s[1] is not None and len(s[1])]
    if not series:
        return None
    fig, ax = plt.subplots(figsize=(6.6, 4.2))
    for i, (name, err) in enumerate(series):
        v = np.sort(np.asarray(err, dtype=float))
        y = np.arange(1, len(v) + 1) / len(v)
        ax.plot(v, y, "-", lw=1.7, color=colour_for(name, i), label=name)
    ax.axhline(0.95, color="#9b1c1c", ls="--", lw=1.0, alpha=0.8)
    ax.text(ax.get_xlim()[1], 0.95, " 95th pct", va="center", fontsize=9,
            color="#9b1c1c")
    ax.set_xlabel("absolute position error  [m]")
    ax.set_ylabel("fraction of poses at or below")
    ax.set_ylim(0, 1.02)
    ax.set_title("Error distribution (cumulative)")
    ax.legend(loc="lower right", fontsize=9)
    _style(ax)
    return _fig_to_png(fig, plt)


def fig_rpe_bars(plt, np, rpe_by_stack) -> bytes | None:
    """Relative pose error at each evaluated separation, grouped by stack."""
    if not rpe_by_stack:
        return None
    deltas = sorted({d for v in rpe_by_stack.values() for d in v})
    if not deltas:
        return None
    names = list(rpe_by_stack)
    fig, ax = plt.subplots(figsize=(8.2, 4.0))
    w = 0.8 / max(len(names), 1)
    x = np.arange(len(deltas))
    for i, name in enumerate(names):
        vals = [rpe_by_stack[name].get(d, float("nan")) for d in deltas]
        ax.bar(x + i * w - 0.4 + w / 2, vals, width=w * 0.92,
               color=colour_for(name, i), label=name)
    ax.set_xticks(x)
    ax.set_xticklabels(deltas)
    ax.set_xlabel("separation between the pose pair being compared")
    ax.set_ylabel("RPE, translation RMSE  [m]")
    ax.set_title("Relative pose error — local accuracy, no alignment applied")
    ax.legend(fontsize=9)
    _style(ax)
    return _fig_to_png(fig, plt)


def fig_drift_segments(plt, np, drift_by_stack) -> bytes | None:
    """Translational drift as a percentage, per segment length.

    A flat line across segment lengths is the signature of a well-behaved
    system: error proportional to distance. A line that rises with segment
    length means something worse than linear is happening — usually an
    uncorrected heading bias, whose position error grows with the square of
    distance travelled.
    """
    if not drift_by_stack:
        return None
    lengths = sorted({L for v in drift_by_stack.values() for L in v},
                     key=lambda s: float(str(s).rstrip("m")))
    if not lengths:
        return None
    fig, ax = plt.subplots(figsize=(7.6, 4.0))
    for i, (name, d) in enumerate(drift_by_stack.items()):
        vals = [d.get(L, float("nan")) for L in lengths]
        ax.plot(range(len(lengths)), vals, "o-", lw=1.8, ms=6,
                color=colour_for(name, i), label=name)
    ax.set_xticks(range(len(lengths)))
    ax.set_xticklabels(lengths)
    ax.set_xlabel("segment length over which drift was measured")
    ax.set_ylabel("translational drift  [% of distance]")
    ax.set_title("Drift versus distance travelled (KITTI-style segments)")
    ax.legend(fontsize=9)
    _style(ax)
    return _fig_to_png(fig, plt)


def fig_depth(plt, np, depth) -> bytes | None:
    """Depth bias and noise against distance, with the ZED X datasheet bound.

    The shaded band is the manufacturer's UPPER BOUND, not its expected
    accuracy. The datasheet publishes two inequalities for this model —
    `< 0.4% to 2 m` and `< 7% at 20 m` — so a measurement inside the band
    shows only that the camera is not out of specification, which is a much
    weaker statement than "the camera is accurate". A measurement outside it
    means something in the chain is wrong (calibration file, resolution
    mismatch, a smudge on the lens, or a target that is not as flat or as
    perpendicular as assumed).

    The curve is drawn by the SAME function the measurement script judges
    against, imported rather than reimplemented. It used to be a second
    hand-written copy of the interpolation here, which is how the figure and
    the table could in principle have disagreed. Both now come from
    `depth_characterization.datasheet_bound_m`.
    """
    meas = [m for m in (depth or {}).get("measurements", [])
            if m.get("truth_m") is not None and m.get("bias_m") is not None]
    if not meas:
        return None
    meas.sort(key=lambda m: m["truth_m"])
    d = np.array([m["truth_m"] for m in meas], dtype=float)
    bias = np.array([m["bias_m"] for m in meas], dtype=float)
    std = np.array([m.get("std_m") or 0.0 for m in meas], dtype=float)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11.0, 4.1))

    # Datasheet bound, taken from the measurement script so the figure and the
    # table can never diverge. Imported here, not at module scope, because
    # importing it pulls in eval_common and this module must stay importable
    # for the non-depth figures on a machine that has neither.
    from depth_characterization import datasheet_bound_m
    dd = np.linspace(max(1.0, d.min() * 0.8), max(15.0, d.max() * 1.1), 200)
    env = np.array([datasheet_bound_m(float(z)) for z in dd])
    ax1.fill_between(dd, -env, env, color="#0072B2", alpha=0.12,
                     label="ZED X datasheet upper bound")
    ax1.errorbar(d, bias, yerr=std, fmt="o", ms=7, capsize=4, lw=1.6,
                 color="#D55E00", label="measured bias ±1σ")
    ax1.axhline(0.0, color="#111827", lw=1.0)
    ax1.set_xlabel("true distance to wall  [m]")
    ax1.set_ylabel("depth bias (measured − true)  [m]")
    ax1.set_title("Depth accuracy against a flat wall")
    ax1.legend(fontsize=9)
    _style(ax1)

    flat = np.array([m.get("flatness_rms_m") or float("nan") for m in meas])
    inval = np.array([m.get("invalid_pct") or 0.0 for m in meas])
    ax2.plot(d, 1000 * flat, "s-", lw=1.8, ms=7, color="#009E73",
             label="plane-fit RMS residual  [mm]")
    ax2b = ax2.twinx()
    ax2b.plot(d, inval, "^--", lw=1.6, ms=7, color="#CC79A7",
              label="invalid pixels  [%]")
    ax2b.set_ylabel("invalid pixels  [%]", color="#CC79A7")
    ax2.set_xlabel("true distance to wall  [m]")
    ax2.set_ylabel("flatness RMS  [mm]", color="#009E73")
    ax2.set_title("Depth noise and dropout")
    h1, l1 = ax2.get_legend_handles_labels()
    h2, l2 = ax2b.get_legend_handles_labels()
    ax2.legend(h1 + h2, l1 + l2, fontsize=9, loc="upper left")
    _style(ax2)
    fig.tight_layout()
    return _fig_to_png(fig, plt)


def fig_latency(plt, np, samples) -> bytes | None:
    """Latency histogram. The tail is the story, so the 95th percentile is drawn."""
    v = np.asarray([s for s in (samples or []) if s is not None], dtype=float)
    v = v[np.isfinite(v) & (v >= 0)]
    if v.size < 5:
        return None
    fig, ax = plt.subplots(figsize=(7.0, 3.6))
    ax.hist(v, bins=min(60, max(10, v.size // 20)), color="#0072B2", alpha=0.85)
    p95 = float(np.percentile(v, 95))
    ax.axvline(p95, color="#9b1c1c", ls="--", lw=1.4,
               label=f"95th percentile = {p95:.1f} ms")
    ax.axvline(float(np.median(v)), color="#15603a", ls="-", lw=1.4,
               label=f"median = {float(np.median(v)):.1f} ms")
    ax.set_xlabel("capture-to-pose latency  [ms]")
    ax.set_ylabel("frames")
    ax.set_title("Processing latency")
    ax.legend(fontsize=9)
    _style(ax)
    return _fig_to_png(fig, plt)


def fig_system_load(plt, np, load) -> bytes | None:
    """CPU, GPU and memory over the run.

    Plotted together on one time axis because the interesting events are
    correlations: a latency spike at the moment CPU saturates is a resource
    problem, the same spike with CPU at 40% is an algorithmic one.
    """
    samples = (load or {}).get("samples") or []
    if len(samples) < 3:
        return None
    t = np.array([s.get("t", 0.0) for s in samples], dtype=float)
    t = t - t[0]
    fig, ax = plt.subplots(figsize=(9.2, 3.8))
    plotted = False
    for key, label, colour in (("cpu_pct", "CPU  [%]", "#0072B2"),
                               ("gpu_pct", "GPU  [%]", "#D55E00"),
                               ("mem_pct", "RAM  [%]", "#009E73")):
        vals = np.array([s.get(key) if s.get(key) is not None else np.nan
                         for s in samples], dtype=float)
        if np.all(np.isnan(vals)):
            continue
        ax.plot(t, vals, "-", lw=1.4, color=colour, label=label)
        plotted = True
    if not plotted:
        plt.close(fig)
        return None
    ax.set_ylim(0, 105)
    ax.set_xlabel("time since start of run  [s]")
    ax.set_ylabel("utilisation  [%]")
    ax.set_title("System load on the Jetson AGX Orin")
    ax.legend(fontsize=9, ncol=3)
    _style(ax)
    return _fig_to_png(fig, plt)


# --------------------------------------------------------------------------- #
# HTML assembly
# --------------------------------------------------------------------------- #

def embed(png: bytes | None) -> str | None:
    """PNG bytes -> a data: URI. This is what makes the report self-contained."""
    if not png:
        return None
    return "data:image/png;base64," + base64.b64encode(png).decode("ascii")


def figure_block(uri: str | None, caption: str) -> str:
    if not uri:
        return ""
    return (f'<figure><img alt="{e(caption[:80])}" src="{uri}">'
            f'<figcaption>{caption}</figcaption></figure>')


def kpi_row(items) -> str:
    if not items:
        return ""
    out = ['<div class="kpis">']
    for label, value in items:
        out.append(f'<div class="kpi"><div class="v">{e(value)}</div>'
                   f'<div class="k">{e(label)}</div></div>')
    out.append("</div>")
    return "\n".join(out)


def table(headers, rows, best_cols=None) -> str:
    """Render a table, optionally highlighting the best cell in given columns."""
    best_cols = best_cols or {}
    out = ['<div class="scroll"><table><thead><tr>']
    for i, h in enumerate(headers):
        cls = ' class="num"' if i > 0 else ""
        out.append(f"<th{cls}>{e(h)}</th>")
    out.append("</tr></thead><tbody>")
    for r_i, row in enumerate(rows):
        out.append("<tr>")
        for c_i, cell in enumerate(row):
            cls = "num" if c_i > 0 else ""
            if best_cols.get(r_i) == c_i:
                cls = (cls + " best").strip()
            # Column 0 is the row label and is allowed to carry markup that the
            # caller built (typically a <code> tag). Every other cell is data
            # and is escaped, so a stray '<' in a value cannot break the page.
            content = cell if c_i == 0 else e(cell)
            out.append(f'<td class="{cls}">{content}</td>')
        out.append("</tr>")
    out.append("</tbody></table></div>")
    return "\n".join(out)


def _load_json(path: Path):
    try:
        return json.loads(path.read_text())
    except Exception as exc:                     # noqa: BLE001 - report, do not crash
        return {"_load_error": f"{path.name}: {exc}"}


def build_report(results_dir: Path, title: str = "", log=None) -> str:
    """Assemble the HTML. Returns the document as a string."""
    np = require_numpy(log)
    try:
        _, plt = require_matplotlib(log)
        can_plot = True
    except MissingDependency as exc:
        plt = None
        can_plot = False
        plot_warning = str(exc)

    results_dir = Path(results_dir)
    evals = {}
    plotdata = {}
    for f in sorted(results_dir.glob("eval_*.json")):
        evals[f.stem[len("eval_"):]] = _load_json(f)
    for f in sorted(results_dir.glob("plotdata_*.json")):
        plotdata[f.stem[len("plotdata_"):]] = _load_json(f)

    depth = _load_json(results_dir / "depth_characterization.json") \
        if (results_dir / "depth_characterization.json").exists() else None
    load = _load_json(results_dir / "system_load.json") \
        if (results_dir / "system_load.json").exists() else None
    comparison = _load_json(results_dir / "comparison.json") \
        if (results_dir / "comparison.json").exists() else None

    P: list = []
    A = P.append

    # ---------------------------------------------------------------- header
    doc_title = title or f"Evaluation — {results_dir.name}"
    A('<!doctype html><html lang="en"><head><meta charset="utf-8">')
    A('<meta name="viewport" content="width=device-width,initial-scale=1">')
    A(f"<title>{e(doc_title)} — Mitacs VSLAM</title>")
    A(f"<style>{CSS}</style></head><body>")
    A('<header class="hd"><div class="wrap">')
    A('<div class="crumb">Phase 5 &middot; <code>sidewalk_evaluation</code> '
      '&middot; Mitacs Globalink 2026 &middot; Project 50124</div>')
    A(f"<h1>{e(doc_title)}</h1>")
    A('<p class="lede">Quantitative evaluation of mapping and localization '
      'performance for the autonomous sidewalk robot. Every figure in this file '
      'is embedded in the file itself — it can be emailed, archived, or opened '
      'on a machine with no internet and no software installed.</p>')
    A(f'<div class="crumb">Generated {datetime.now().strftime("%Y-%m-%d %H:%M")} '
      f'from <code>{e(results_dir)}</code></div>')
    A("</div></header>")
    A('<main class="wrap">')

    if not can_plot:
        A(f'<div class="warn"><strong>Figures omitted.</strong> {e(plot_warning)}'
          '</div>')

    # ------------------------------------------------------- what was measured
    A('<section class="card hi"><h2>What this report measures, in plain words</h2>')
    A("<p>Two questions are being answered, and they are not the same question.</p>")
    A("<ul>"
      "<li><strong>How far off is the whole path?</strong> Drive a route, "
      "compare the path the robot <em>thinks</em> it took with the path it "
      "<em>actually</em> took, and measure the gap. That is <strong>absolute "
      "trajectory error (ATE)</strong>.</li>"
      "<li><strong>How wrong is each individual step?</strong> Forget the whole "
      "path; look at one metre of travel at a time and ask how accurately the "
      "robot measured just that metre. That is <strong>relative pose error "
      "(RPE)</strong>.</li>"
      "</ul>")
    A("<p>A system can be excellent at one and poor at the other. Small steps "
      "with a slow accumulating bias gives good RPE and bad ATE. Jittery steps "
      "that get periodically snapped back into place by recognising a familiar "
      "spot gives bad RPE and good ATE. Reporting only one number hides half "
      "of what the system does.</p>")
    A("</section>")

    # ------------------------------------------------------------- reference
    kinds = {v.get("reference_kind", "unspecified") for v in evals.values()}

    # Provenance gate, layer 2. Layer 1 (evaluate_trajectory) stops the
    # mistake at evaluation time; this backstop catches old or foreign results
    # directories re-rendered standalone — which is exactly how the
    # robot data. A report whose truth source is unquotable refuses to render
    # rather than printing numbers a reader will quote out of context.
    offending = {name: v.get("reference_kind", "unspecified")
                 for name, v in evals.items()
                 if v.get("reference_kind", "unspecified") in UNQUOTABLE_KINDS}
    if offending:
        detail = "; ".join(f"'{n}' is labelled '{k}'"
                           for n, k in sorted(offending.items()))
        raise ProvenanceError(
            f"Refusing to render accuracy numbers: {detail}.\n"
            f"'unspecified' means nobody declared what truth was; "
            f"'synthetic_ground_truth' is the self-test harness and says "
            f"nothing about any robot.\n"
            f"Re-run the evaluation with an honest kind, e.g.:\n"
            f"  evaluate_trajectory.py --ref <truth.tum> --est <name>=<est.tum> "
            f"--reference-kind simulator_ground_truth\n"
            f"(or motion_capture for public datasets, wheel_odometry for a "
            f"short-range sanity reference).")

    A('<section class="card"><h2>What is being used as truth</h2>')
    A("<p>Every error figure in this report is a comparison against something. "
      "That something is stated here, because a trajectory error is only as "
      "meaningful as the reference it was measured against.</p>")
    A(_reference_explainer(kinds))
    A("</section>")

    # ------------------------------------------------- per-trajectory results
    if evals:
        # The heading has to tell the truth about what is in the table. If any
        # row was produced by comparing one navigation run against another, the
        # column called "ATE RMSE" is a repeatability spread, not an error
        # against truth, and a section headed "Trajectory accuracy" would be a
        # false label on the one number a reader will quote out of context.
        _repeat_rows = sorted(n for n, v in evals.items()
                              if v.get("reference_kind") == "nav_repeat_run")
        if _repeat_rows and len(_repeat_rows) == len(evals):
            A('<section class="card"><h2>Trajectory repeatability</h2>')
        elif _repeat_rows:
            A('<section class="card"><h2>Trajectory accuracy and '
              'repeatability</h2>')
        else:
            A('<section class="card"><h2>Trajectory accuracy</h2>')
        if _repeat_rows:
            A('<div class="warn"><strong>Not an accuracy measurement: '
              + e(", ".join(_repeat_rows)) +
              '.</strong> These rows compare one navigation run against '
              'another run of the same route by the same localisation system. '
              'The ATE and RPE columns measure how consistently that system '
              'reproduces its own answer, not how close it is to where the '
              'robot physically was. An accuracy figure needs a reference that '
              'does not share the estimator.</div>')

        headers = ["System", "ATE RMSE [m]", "ATE median [m]", "ATE 95th [m]",
                   "Rotation RMSE [deg]", "RPE 1 m [m]", "Drift [%]",
                   "Drift [deg/m]", "Path [m]", "Poses"]
        rows = []
        for name, r in evals.items():
            if "error" in r:
                rows.append([f"<code>{e(name)}</code>", "—", "—", "—", "—",
                             "—", "—", "—", "—", "failed"])
                continue
            ate = r.get("ate", {})
            drift = r.get("drift", {})
            rpe1 = (r.get("rpe", {}) or {}).get("1m", {})
            rows.append([
                f"<code>{e(name)}</code>",
                fmt(ate.get("translation", {}).get("rmse"), 4),
                fmt(ate.get("translation", {}).get("median"), 4),
                fmt(ate.get("translation", {}).get("p95"), 4),
                fmt(ate.get("rotation", {}).get("rmse"), 3),
                fmt(rpe1.get("translation", {}).get("rmse"), 4),
                fmt(drift.get("overall_translation_pct", {}).get("mean"), 3),
                fmt(drift.get("overall_rotation_deg_per_m", {}).get("mean"), 4),
                fmt(drift.get("path_length_m"), 1),
                str(r.get("association", {}).get("n_associated", "—")),
            ])
        A(table(headers, rows, _best_cells(rows, lower_is_better=(1, 2, 3, 4, 5, 6, 7))))

        # Any warnings the metric code raised belong right under the table, not
        # buried at the end. A caveat nobody reads is not a caveat.
        for name, r in evals.items():
            for w in r.get("warnings", []) or []:
                A(f'<div class="warn"><strong>{e(name)}:</strong> {e(w)}</div>')
            if "error" in r:
                A(f'<div class="bad"><strong>{e(name)} — not evaluated.</strong> '
                  f'{e(r["error"])}</div>')
            for issue in (r.get("estimate_validation", {}) or {}).get("issues", []):
                A(f'<div class="warn"><strong>{e(name)} input data:</strong> '
                  f'{e(issue)}</div>')

        # The alignment policy is part of the result. A windowed fit changes
        # what "error at time t" means, so a report that used one must say so.
        for name, r in evals.items():
            aw = (r.get("alignment") or {}).get("align_window_s") or 0
            if aw > 0:
                A(f'<div class="note"><strong>{e(name)} alignment policy:'
                  f'</strong> the reference alignment was fitted on the first '
                  f'{aw:g} s of overlap only '
                  f'({(r.get("alignment") or {}).get("align_fit_poses", "?")} '
                  f'poses); all errors are still measured over the whole run. '
                  f'This prevents least-squares tail-drag: without it, a '
                  f'diverging tail drags the fit away from the start and '
                  f'manufactures error where the robot had not yet moved.</div>')

        # RMSE is L2 and saturates on outliers — a handful of tracking
        # teleports dominate it entirely (ENGINEERING_NOTES.md section 4.3). Median and 95th
        # per RPE separation let a reader see whether the distribution's body
        # or its tail is carrying the headline number.
        rh = ["System", "Δ", "RMSE [m]", "Median [m]", "95th [m]", "Pairs"]
        rr = []
        for name, r in evals.items():
            for k in sorted((r.get("rpe") or {}).keys()):
                tr_ = (r["rpe"][k].get("translation") or {})
                if tr_.get("rmse") is None:
                    continue
                rr.append([f"<code>{e(name)}</code>", e(k),
                           fmt(tr_.get("rmse"), 4), fmt(tr_.get("median"), 4),
                           fmt(tr_.get("p95"), 4),
                           str(r["rpe"][k].get("n_pairs", "—"))])
        if rr:
            A("<h3>Relative pose error distribution</h3>")
            A("<p>When RMSE sits far above the median, outliers — usually "
              "tracking-loss teleports — are carrying it; the median is what "
              "typical operation looked like. When they agree, the error is "
              "genuinely uniform.</p>")
            A(table(rh, rr))

        # Drift distribution per segment length, same reasoning.
        dh = ["System", "Segment [m]", "Mean [%]", "Median [%]", "95th [%]"]
        dr_ = []
        for name, r in evals.items():
            for k, v in sorted(((r.get("drift") or {})
                                .get("per_segment_length") or {}).items()):
                tp = v.get("translation_pct") or {}
                if tp.get("mean") is None:
                    continue
                dr_.append([f"<code>{e(name)}</code>", e(str(k)),
                            fmt(tp.get("mean"), 3), fmt(tp.get("median"), 3),
                            fmt(tp.get("p95"), 3)])
        if dr_:
            A("<h3>Drift distribution by segment length</h3>")
            A(table(dh, dr_))

        # Per-clean-segment Sim(3), when --split-at-gaps ran: the decisive
        # test between real calibration error (scale persists per segment)
        # and teleport inflation (scale collapses to ~1 on clean data).
        gh = ["System", "Span [s]", "Poses", "Path [m]", "Sim(3) scale",
              "ATE RMSE [m]", "ATE median [m]"]
        gr = []
        for name, r in evals.items():
            for s in r.get("gap_segments") or []:
                gr.append([f"<code>{e(name)}</code>",
                           f"{s['start_s']:.0f}–{s['end_s']:.0f}",
                           str(s["n_poses"]), fmt(s.get("path_m"), 1),
                           fmt(s.get("scale_sim3"), 4),
                           fmt(s.get("ate_rmse_m"), 3),
                           fmt(s.get("ate_median_m"), 3)])
        if gr:
            A("<h3>Per-segment evaluation between tracking gaps</h3>")
            A("<p>Each clean stretch between tracking losses gets its own "
              "Sim(3) fit. A real scale/calibration error survives the cut "
              "and shows in every segment; inflation caused by teleports "
              "across the gaps does not — segment scales near 1.0000 acquit "
              "the calibration.</p>")
            A(table(gh, gr))

        if can_plot:
            ref_xy = None
            for pd in plotdata.values():
                if pd.get("ref_xy"):
                    ref_xy = [np.array(pd["ref_xy"][0]), np.array(pd["ref_xy"][1])]
                    break
            ests = [(n, [np.array(pd["est_xy"][0]), np.array(pd["est_xy"][1])])
                    for n, pd in plotdata.items() if pd.get("est_xy")]
            A(figure_block(embed(fig_trajectory_xy(plt, ref_xy, ests)),
                           "Top-down view of the route. The estimate has been "
                           "rigidly aligned to the reference first, because a "
                           "SLAM system chooses its own arbitrary origin — "
                           "without alignment this plot would show that "
                           "arbitrary choice rather than any real error."))

            series_t = [(n, np.array(pd.get("err_t") or []),
                         np.array(pd.get("err_m") or []))
                        for n, pd in plotdata.items()]
            A(figure_block(embed(fig_error_vs_time(plt, series_t)),
                           "Position error against time. A rising curve is "
                           "drift accumulating. A sudden downward step is a "
                           "loop closure correcting it. A step up that never "
                           "comes back down is a tracking failure."))

            series_c = [(n, np.array(pd.get("err_m") or []))
                        for n, pd in plotdata.items()]
            A(figure_block(embed(fig_error_cdf(plt, np, series_c)),
                           "How often the robot was within a given distance of "
                           "where it believed it was. The 95th percentile, not "
                           "the average, is the number a safety case rests on."))

            rpe_by = {}
            for name, r in evals.items():
                d = {}
                for k, v in (r.get("rpe") or {}).items():
                    val = (v.get("translation") or {}).get("rmse")
                    if val is not None:
                        d[k] = val
                if d:
                    rpe_by[name] = d
            A(figure_block(embed(fig_rpe_bars(plt, np, rpe_by)),
                           "Relative pose error at several separations. Note "
                           "that no alignment is applied for this metric — it "
                           "compares increments of motion, which are "
                           "independent of where the world origin was placed."))

            drift_by = {}
            for name, r in evals.items():
                d = {}
                for k, v in ((r.get("drift") or {}).get("per_segment_length") or {}).items():
                    val = (v.get("translation_pct") or {}).get("mean")
                    if val is not None:
                        d[k] = val
                if d:
                    drift_by[name] = d
            A(figure_block(embed(fig_drift_segments(plt, np, drift_by)),
                           "Drift as a percentage of distance travelled, "
                           "measured over segments of several lengths. Flat "
                           "across segment lengths means error grows in "
                           "proportion to distance, which is the well-behaved "
                           "case. Rising means a systematic heading bias, "
                           "whose position error grows with the square of "
                           "distance."))
        A("</section>")

        # ------------------------------------------- loop closure & tracking
        A('<section class="card"><h2>Loop closure, drift and tracking reliability</h2>')
        A("<p>When a route returns to its starting point, the truth at that "
          "moment is known exactly and for free: the displacement is zero. "
          "Whatever gap the estimator still reports is accumulated error, "
          "plus however far off the physical mark the robot was parked. "
          "That parking error is the noise floor of this measurement and must "
          "be recorded with the result.</p>")
        lh = ["System", "Loop closed?", "End-to-start gap [m]", "Yaw gap [deg]",
              "Drift [% of path]", "Corrections detected",
              "Correction median [m]", "Correction 95th [m]",
              "Largest correction [m]"]
        lr = []
        for name, r in evals.items():
            lc = r.get("loop_closure_gap") or {}
            cx = r.get("corrections") or {}
            mag = cx.get("magnitude") or {}
            lr.append([
                f"<code>{e(name)}</code>",
                "yes" if lc.get("closed") else "no",
                fmt(lc.get("end_to_start_gap_m"), 3),
                fmt(lc.get("end_to_start_yaw_gap_deg"), 2),
                fmt(lc.get("drift_pct_of_path"), 3),
                str(cx.get("n_corrections", "—")),
                fmt(mag.get("median"), 3),
                fmt(mag.get("p95"), 3),
                fmt(cx.get("largest_correction_m"), 3),
            ])
        A(table(lh, lr))
        for name, r in evals.items():
            lc = r.get("loop_closure_gap") or {}
            if lc.get("reason"):
                A(f'<div class="note"><strong>{e(name)}:</strong> {e(lc["reason"])}</div>')
            cx = r.get("corrections") or {}
            if cx.get("n_corrections") == 0:
                A(f'<div class="note"><strong>{e(name)}:</strong> no pose-graph '
                  f'corrections were observed. On a route that revisits itself '
                  f'that is a finding, not a clean bill of health — it means '
                  f'loop closure never fired.</div>')

        rel_rows = []
        for name, r in evals.items():
            tr = r.get("tracking") or {}
            if not tr.get("available"):
                continue
            rel_rows.append([
                f"<code>{e(name)}</code>",
                str(tr.get("tracking_loss_count", "—")),
                fmt(tr.get("relocalization_success_rate_pct"), 1),
                fmt((tr.get("time_to_relocalize") or {}).get("median"), 2),
                fmt((tr.get("time_to_relocalize") or {}).get("max"), 2),
                fmt(tr.get("availability_pct"), 2),
            ])
        if rel_rows:
            A("<h3>Relocalization</h3>")
            A(table(["System", "Tracking losses", "Reloc. success [%]",
                     "Median time to reloc. [s]", "Worst [s]",
                     "Availability [%]"], rel_rows))
        else:
            A('<div class="note">No tracking-status stream was recorded, so '
              'tracking-loss and relocalization metrics are unavailable for '
              'this run. These require each SLAM stack to publish its own '
              'status; see <code>trajectory_recorder.py</code> and the '
              '<code>status_topic</code> setting in '
              '<code>config/evaluation.yaml</code>.</div>')
        A("</section>")

    # --------------------------------------------------------------- depth
    A('<section class="card"><h2>Depth sensor characterization</h2>')
    if depth and depth.get("measurements"):
        A("<p>The camera is placed square-on to a flat wall at a series of "
          "measured distances. At each distance we record how far off the "
          "average reading is (<strong>bias</strong>), how noisy it is "
          "(<strong>standard deviation</strong>), what fraction of pixels "
          "returned nothing at all (<strong>invalid</strong>), and how flat the "
          "wall appears to the camera (<strong>plane-fit residual</strong>). "
          "That last one is the sharpest test: a wall really is flat, so any "
          "curvature the camera reports is the camera's error and nothing "
          "else.</p>")
        dh = ["Nominal [m]", "Measured true [m]", "Bias [m]", "Bias [%]",
              "Std dev [m]", "Invalid [%]", "Flatness RMS [mm]",
              "Datasheet bound [m]", "Within bound"]
        dr = []
        for m in sorted(depth["measurements"], key=lambda x: x.get("truth_m", 0)):
            # `within_spec`, which were computed against an envelope of two
            # figures that appear in no datasheet. They are NOT silently
            # re-read under the new column names — a stale verdict would look
            # exactly like a current one. Such a row is marked instead, and
            # the measurement has to be re-scored by re-running the script.
            stale = ("datasheet_bound_m" not in m) and ("spec_limit_m" in m)
            dr.append([
                fmt(m.get("nominal_m"), 1),
                fmt(m.get("truth_m"), 3),
                fmt(m.get("bias_m"), 4),
                fmt(m.get("bias_pct"), 3),
                fmt(m.get("std_m"), 4),
                fmt(m.get("invalid_pct"), 2),
                fmt((m.get("flatness_rms_m") or 0) * 1000, 2),
                "withdrawn envelope" if stale else fmt(m.get("datasheet_bound_m"), 4),
                ("RE-SCORE" if stale else
                 ("yes" if m.get("within_datasheet_bound") else "NO")),
            ])
        A(table(dh, dr))
        A("<p>The bound column is the manufacturer's <strong>upper "
          "limit</strong>, not its expected accuracy. The ZED X datasheet "
          "publishes two inequalities for this model — under 0.4% out to 2 m, "
          "and under 7% at 20 m — and the column interpolates between them. A "
          "measurement inside the bound proves only that the camera is not out "
          "of specification; it is not evidence that the camera is performing "
          "well, and a reading far inside says more about how loose the bound "
          "is than about the hardware.</p>")
        if can_plot:
            A(figure_block(embed(fig_depth(plt, np, depth)),
                           "Left: measured depth bias against the ZED X "
                           "datasheet upper bound (under 0.4% out to 2 m, "
                           "under 7% at 20 m, interpolated between). "
                           "Right: how flat the wall appears, and what fraction "
                           "of pixels returned no depth at all."))
        for n in depth.get("notes", []) or []:
            A(f'<div class="note">{e(n)}</div>')
    else:
        A('<div class="note">No depth characterization was performed for this '
          'run. It requires the ZED X to be physically wired and pointed at a '
          'flat wall at known distances — see <code>rosrun '
          'sidewalk_evaluation depth_characterization.py</code>. Without it, '
          'the mapping results in this report cannot be attributed between '
          '"the depth sensor is inaccurate" and "the SLAM algorithm is '
          'inaccurate".</div>')
    A("</section>")

    # ------------------------------------------------------ compute & timing
    A('<section class="card"><h2>Compute cost, latency and dropped frames</h2>')
    if load:
        s = load.get("summary") or {}
        A(kpi_row([
            ("CPU mean", fmt(s.get("cpu_mean_pct"), 1, "%")),
            ("CPU peak", fmt(s.get("cpu_max_pct"), 1, "%")),
            ("GPU mean", fmt(s.get("gpu_mean_pct"), 1, "%")),
            ("RAM peak", fmt(s.get("mem_max_mb"), 0, "MB")),
            ("Latency median", fmt((load.get("latency") or {}).get("latency_ms", {}).get("median"), 1, "ms")),
            ("Latency 95th", fmt((load.get("latency") or {}).get("latency_ms", {}).get("p95"), 1, "ms")),
            ("Dropped frames", fmt((load.get("latency") or {}).get("dropped_frame_pct"), 2, "%")),
        ]))
        A("<p>Latency here is the full path from the shutter closing on the "
          "ZED X to a pose being available to the navigation stack — transport, "
          "depth computation, algorithm and message queue together. At walking "
          "pace, roughly 1.4 m/s, every 100 ms of latency is 14 cm of travel "
          "the planner does not yet know about. That is why it is reported "
          "beside accuracy rather than in an appendix: a perfectly accurate "
          "pose that arrives late is a stale pose.</p>")
        if can_plot:
            A(figure_block(embed(fig_system_load(plt, np, load)),
                           "Resource utilisation over the run. Look for "
                           "correlations: a latency spike while the CPU is "
                           "saturated is a resource problem; the same spike at "
                           "40% CPU is an algorithmic one."))
            lat_samples = (load.get("latency") or {}).get("samples_ms")
            A(figure_block(embed(fig_latency(plt, np, lat_samples)),
                           "Distribution of capture-to-pose latency. The tail "
                           "matters more than the average — one late pose "
                           "during an avoidance manoeuvre is worse than a "
                           "uniformly mediocre average."))
        for w in load.get("warnings", []) or []:
            A(f'<div class="warn">{e(w)}</div>')
    else:
        A('<div class="note">No system-load recording is present for this run. '
          'Run <code>rosrun sidewalk_evaluation system_monitor.py</code> '
          'alongside the SLAM stack to capture it. Accuracy figures without '
          'compute cost are only half a result: a system that is 20% more '
          'accurate at three times the power budget may be the wrong choice '
          'for a battery-powered sidewalk robot.</div>')
    A("</section>")

    # ------------------------------------------------------------ comparison
    if comparison and comparison.get("rows"):
        A('<section class="card"><h2>Three-way comparison of localization stacks</h2>')
        A("<p>" + e(comparison.get("preamble", "")) + "</p>")
        A(table(comparison.get("headers", []),
                [[c for c in row] for row in comparison["rows"]],
                {int(k): int(v) for k, v in (comparison.get("best_cells") or {}).items()}))
        for n in comparison.get("caveats", []) or []:
            A(f'<div class="warn">{e(n)}</div>')
        A("</section>")

    # ------------------------------------------------------- how to read this
    A('<section class="card"><h2>How to read these numbers honestly</h2>')
    A("<ul>"
      "<li>An ATE figure without a stated reference is meaningless. Check the "
      "<em>what is being used as truth</em> section above before quoting any "
      "number from this report.</li>"
      "<li>Comparing two SLAM stacks against each other measures "
      "<strong>disagreement</strong>, not accuracy. Both can agree and both be "
      "wrong.</li>"
      "<li>Drift is a random walk. One run is one sample. Three runs of the "
      "same route can differ by a factor of two by chance alone — repeat every "
      "experiment at least three times and report the spread, not the best "
      "run.</li>"
      "<li>These figures apply to the conditions of this run: this route, this "
      "lighting, this speed, this weather. A sidewalk robot's hardest "
      "conditions — low winter sun straight into a 4.6 mm f/2.0 lens with no "
      "polarizer, wet reflective concrete, falling snow — are exactly the ones "
      "least likely to have been captured on a convenient test day.</li>"
      "<li>If <code>evo</code> is available, cross-check with "
      "<code>evo_ape tum reference.tum aligned_&lt;stack&gt;.tum -a</code>. Two "
      "independent implementations agreeing is evidence; one implementation "
      "agreeing with itself is not.</li>"
      "</ul>")
    A("</section>")

    A('<footer class="ft">Generated by '
      '<code>sidewalk_evaluation/scripts/generate_report.py</code> &middot; '
      f'{datetime.now().strftime("%Y-%m-%d %H:%M")} &middot; '
      'Mitacs Globalink 2026 &middot; Project 50124 &middot; THAW ZIN &middot; '
      'Supervisor: Prof. Moein Mehrtash</footer>')
    A("</main></body></html>")
    return "\n".join(P)


def _reference_explainer(kinds) -> str:
    """Describe, per reference kind actually used, what it does and does not prove."""
    text = {
        "motion_capture": (
            "<strong>Motion capture.</strong> Millimetre-accurate external "
            "tracking. This is real ground truth. We have no lab mocap of our "
            "own — this kind is legitimate for <em>public datasets</em> whose "
            "truth came from one (e.g. TUM RGB-D, recorded against a 100 Hz "
            "capture system). If it appears over data recorded here, check "
            "why."),
        "rtk_gnss": (
            "<strong>RTK GNSS.</strong> Centimetre-accurate satellite "
            "positioning. Valid outdoors with a clear sky view; unreliable on "
            "the tree-lined and building-flanked sidewalks that are the "
            "intended environment, where multipath errors of several metres "
            "occur without warning."),
        "closed_loop": (
            "<strong>Closed loop.</strong> The route returns to a physically "
            "marked start point, so the true displacement between the first and "
            "last pose is known to be zero. This gives one very trustworthy "
            "datum rather than a full trajectory: it yields a defensible drift "
            "percentage but cannot produce a per-pose error curve. Its accuracy "
            "is limited by how precisely the robot was parked on the mark — "
            "record that parking error and state it with the result."),
        "cross_stack": (
            "<strong>Another SLAM stack.</strong> This measures "
            "<em>disagreement between two estimators</em>, not the error of "
            "either. Two systems sharing a sensor, a calibration and a "
            "viewpoint share their failure modes too; they can agree closely "
            "and both be wrong. Never present a cross-stack figure as "
            "accuracy."),
        "wheel_odometry": (
            "<strong>Wheel odometry from the robot base.</strong> Excellent "
            "over short distances and completely independent of vision, which "
            "makes it a genuinely useful cross-check. It has its own unbounded "
            "heading drift and is degraded by wheel slip on wet concrete, so it "
            "is a sanity reference over tens of metres, not hundreds."),
        "simulator_ground_truth": (
            "<strong>Simulator ground truth.</strong> The physics engine's own "
            "model pose, republished from <code>/gazebo/model_states</code> by "
            "<code>ground_truth_odom.py</code> and recorded alongside the "
            "estimate. Exact within the simulated world, so every error figure "
            "below is genuinely the SLAM system's error — <em>for a simulated "
            "robot with a simulated camera</em>. It says nothing about "
            "real-hardware performance; the sim-to-real gap is measured "
            "separately (Phase 4)."),
        "synthetic_ground_truth": (
            "<strong>Synthetic — self-test only.</strong> A mathematically "
            "generated trajectory with a known, deliberately injected error, "
            "used to verify that the evaluation code itself is correct. It "
            "says nothing whatsoever about any robot, which is why a report "
            "that finds robot data labelled with it refuses to render "
            "(exit 4) rather than print quotable-looking numbers."),
        "nav_repeat_run": (
            "<strong>Another navigation run of the same route.</strong> The "
            "reference is a second drive of the identical waypoint list by the "
            "<em>same</em> localisation system, ingested by "
            "<code>ingest_navigation_run.py</code>. The figures in the table "
            "below are therefore <strong>REPEATABILITY, NOT ACCURACY</strong>: "
            "they say how consistently the system reproduces its own answer. "
            "Two runs can agree to a centimetre and both be two metres from "
            "where the robot physically was. Note also that the two runs share "
            "no clock, so their poses are associated by elapsed time — a run "
            "driven at a different speed shows an apparent error along an "
            "identical line. The waypoint arrival scatter in "
            "<code>repeatability.json</code> needs no such association and is "
            "the cleaner figure."),
        "unspecified": (
            "<strong>Unspecified.</strong> The reference kind was not recorded "
            "for this evaluation. Fix this before the result is used: pass "
            "<code>--reference-kind</code> to "
            "<code>evaluate_trajectory.py</code>."),
    }
    out = ["<ul>"]
    for k in sorted(kinds):
        out.append(f"<li>{text.get(k, '<strong>' + e(k) + '</strong> — no description available.')}</li>")
    out.append("</ul>")
    return "\n".join(out)


def _best_cells(rows, lower_is_better=()) -> dict:
    """Work out which cell in each column is best, for highlighting.

    Returns {row_index: column_index} — at most one highlight per row, on the
    column where that row wins. Deliberately restrained: highlighting every
    winning cell turns the table into a colouring book.
    """
    best: dict = {}
    if not rows:
        return best
    for col in lower_is_better:
        vals = []
        for r_i, row in enumerate(rows):
            if col >= len(row):
                continue
            try:
                vals.append((float(str(row[col]).split()[0]), r_i))
            except (ValueError, IndexError):
                continue
        if len(vals) > 1:
            _, r_i = min(vals)
            best.setdefault(r_i, col)
    return best


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Render an evaluation results directory into one "
                    "self-contained HTML report.")
    ap.add_argument("--results", required=True,
                    help="directory containing eval_*.json etc.")
    ap.add_argument("--out", default="",
                    help="output path (default: <results>/report.html)")
    ap.add_argument("--title", default="", help="report title")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    results = Path(args.results)
    if not results.is_dir():
        print(f"ERROR: results directory does not exist: {results}\n"
              f"       Run `evaluate_trajectory.py` first — it creates one "
              f"directory per evaluation under "
              f"logs/sidewalk_evaluation/results/.", file=sys.stderr)
        return 2

    with RunLogger("sidewalk_evaluation", run_name="report",
                   echo=not args.quiet) as log:
        log.info(f"Building report from {results}")
        try:
            doc = build_report(results, title=args.title, log=log)
        except MissingDependency as exc:
            log.error(str(exc))
            log.summary("Report generation failed: missing dependency",
                        status="FAIL")
            print(f"\nERROR: {exc}", file=sys.stderr)
            return 3
        except ProvenanceError as exc:
            # A refusal, not a malfunction: the results directory declares a
            # truth source no robot number may be quoted against. Exit 4 is
            # reserved for this so scripts can tell it from bad inputs (2)
            # and missing dependencies (3).
            log.error(str(exc))
            log.summary("Report refused: unquotable reference kind",
                        status="FAIL")
            print(f"\nREFUSED: {exc}", file=sys.stderr)
            return 4

        out = Path(args.out) if args.out else results / "report.html"
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(doc)
        size_kb = out.stat().st_size / 1024.0
        log.metric("report_size_kb", round(size_kb, 1), "kB")
        log.info(f"Wrote {out}")
        log.summary(f"Report written to {out.name} ({size_kb:.0f} kB, "
                    f"self-contained)", status="OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
