#!/usr/bin/env python3
"""
make_manifest.py — the dataset record. Every recording documents itself.

WHY THIS EXISTS
    This is the piece that turns a folder of large binary files into something
    a research project can actually use.

    A recording on its own is close to worthless six months later. Suppose a
    result looks strange and you go back to the data. You need to know: which
    physical camera made this, on which lens, at which resolution and frame
    rate, with which exposure settings, on which driver and SDK version, held
    on which version of the mounting bracket, driven along what path, in what
    weather, with what — if any — ground truth to compare against, and whether
    anything was already known to be wrong at the time.

    None of that is recoverable after the fact. If it is not captured at record
    time, it is gone. And every one of those fields has, in published robotics
    work, turned out to be the explanation for an anomalous result: a firmware
    update that changed the timestamping, a bracket that was re-printed 2 mm
    taller, a run done at dusk when everything else was done at noon.

    So this tool writes a manifest beside every recording, in three forms:
      - `manifest.yaml`  human-editable, and the authoritative copy
      - `manifest.json`  machine-readable, for the evaluation phase
      - `DATASET.html`   a page a non-technical reader can open and understand

    The HTML page is not decoration. A supervisor should be able to double-click
    a dataset and see what it contains without installing anything.

WHAT COUNTS AS A COMPLETE MANIFEST
    The `validate` command scores a manifest against the required fields and
    tells you exactly which are missing. A dataset with an incomplete manifest
    should not be used for a published result; `svo_manager.py verify` refuses
    to pass it.

USAGE
    make_manifest.py create <recording_dir> [--field key=value ...]
    make_manifest.py edit   <recording_dir> --field trajectory=...
    make_manifest.py render <recording_dir>       # regenerate DATASET.html
    make_manifest.py index                        # build the dataset index page
    make_manifest.py validate <recording_dir>     # check completeness
"""
from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime
from pathlib import Path

from perception_common import (
    EXIT_FAIL, EXIT_PASS, REPO, ZEDX, RunLogger, Unavailable, die,
    dump_simple_yaml, html_escape, load_css, load_simple_yaml,
    probe_environment,
)

DATA_DIR = REPO / "data" / "recordings"

# Fields that must be filled in for a dataset to be usable in a published
# comparison. Everything else is useful; these are load-bearing.
REQUIRED = [
    ("recording/name", "what this dataset is called"),
    ("recording/date", "when it was recorded"),
    ("recording/duration_s", "how long it is"),
    ("camera/model", "which camera"),
    ("camera/serial", "which physical unit — matters when the second camera arrives"),
    ("camera/resolution", "the resolution actually used"),
    ("camera/fps", "the frame rate actually used"),
    ("software/zed_sdk_version", "which SDK processed the stereo"),
    ("software/l4t_version", "which Jetson OS image"),
    ("mount/revision", "which bracket revision — changes the extrinsics"),
    ("environment/description", "where and in what conditions"),
    ("environment/trajectory", "what path was driven"),
    ("ground_truth/source", "what the result will be compared against"),
]

FIELD_HELP = {
    "environment/description": (
        "Plain words. 'Campus north sidewalk, overcast, ~12 C, light "
        "pedestrian traffic, damp asphalt.' Lighting and surface texture "
        "matter more than they sound: stereo matching needs texture, and wet "
        "asphalt reflects rather than scatters, so it produces far fewer valid "
        "depth pixels than dry asphalt."),
    "environment/trajectory": (
        "What path the robot drove, including whether it returned to its start. "
        "A closed loop is essential for testing loop closure; an open path can "
        "only test odometry drift."),
    "ground_truth/source": (
        "What the estimated trajectory will be compared against. Options used "
        "in this project: 'none', 'manual tape-measure endpoints', 'surveyed "
        "markers', 'wheel odometry (drifts, weak reference)', 'return-to-start "
        "loop closure error'. Say 'none' honestly rather than implying more "
        "than you have."),
    "mount/revision": (
        "Which version of the camera bracket was fitted. Any change to the "
        "mount changes the camera-to-base transform, which invalidates "
        "comparisons across the change. Record it even if it seems obvious."),
    "camera/serial": (
        "The physical unit's serial number. A second ZED X arrives later; "
        "without serials it becomes impossible to tell which datasets came "
        "from which unit, and per-unit factory calibration differs."),
    "known_anomalies": (
        "Anything already known to be wrong: 'lost GMSL link for ~2 s around "
        "t=340', 'operator walked through frame at the start', 'bracket was "
        "loose, visible vibration'. Recording a known fault costs nothing; "
        "rediscovering it later costs days."),
}


# --------------------------------------------------------------------------- #
# Building
# --------------------------------------------------------------------------- #

def blank_manifest():
    """The manifest skeleton, with every field present even when unknown.

    Present-but-null is deliberately different from absent. A null prompts
    someone to go and fill it in; an absent key just looks like the schema
    changed.
    """
    return {
        "schema_version": 1,
        "recording": {
            "name": None,
            "date": None,
            "operator": None,
            "duration_s": None,
            "size_gb": None,
            "files": [],
            "compression": None,
            "notes": None,
        },
        # THE CAMERA BLOCK IS COPIED FROM ZEDX, SO IT INHERITS ITS CORRECTIONS.
        #
        # checked against the datasheet PDF instead of memory:
        #     lens_mm      4.0 -> 4.6      (4 mm is not a lens Stereolabs sells
        #                                   for this body; the options are
        #                                   2.2 mm and 4.6 mm)
        #     imu_rate_hz  400 -> 200
        # Manifests written before that date carry the wrong values. They are
        # not rewritten — a manifest is a record of what was believed at record
        # time, and silently editing history is worse than a wrong field with a
        # date on it. Anyone reading an older manifest should take lens_mm and
        # imu_rate_hz from here instead, and the rest of that manifest still
        # stands: nothing else in the block was affected.
        #
        # The three FOV/aperture fields are NEW. They are recorded because the
        # blind-zone and ground-sampling geometry of any recording depends
        # directly on the vertical field of view, and until now a dataset
        # carried no trace of which value was assumed when it was made — which
        # is precisely how the 52 deg figure survived as long as it did.
        # `schema_version` stays 1: these are additive optional fields, no
        # reader keys off the exact set, and nothing in REQUIRED changes.
        "camera": {
            "model": ZEDX["model"],
            "sku": ZEDX["sku"],
            "serial": None,
            "lens_mm": ZEDX["lens_mm"],
            "polarizer": ZEDX["polarizer"],
            "baseline_mm": ZEDX["baseline_mm"],
            "fov_h_deg": ZEDX["fov_h_deg"],
            "fov_v_deg": ZEDX["fov_v_deg"],
            "aperture_f": ZEDX["aperture_f"],
            "resolution": None,
            "fps": None,
            "exposure": None,
            "gain": None,
            "white_balance": None,
            "interface": ZEDX["interface"],
            "capture_card": ZEDX["capture_card"],
            "capture_card_sku": ZEDX["capture_card_sku"],
            "imu_rate_hz": ZEDX["imu_rate_hz"],
        },
        "software": {
            "zed_sdk_version": None,
            "zedlink_driver": None,
            "l4t_version": None,
            "jetpack_version": None,
            "ros_distro": None,
            "zed_ros_wrapper_branch": None,
            "hostname": None,
            "kernel": None,
        },
        "mount": {
            "revision": None,
            "height_m": None,
            "pitch_deg": None,
            "notes": None,
        },
        "environment": {
            "description": None,
            "trajectory": None,
            "surface": None,
            "weather": None,
            "lighting": None,
            "pedestrians": None,
        },
        "ground_truth": {
            "source": None,
            "format": "TUM: timestamp tx ty tz qx qy qz qw",
            "file": None,
            "notes": None,
        },
        "known_anomalies": None,
        "validation": {
            "sensor_validator_report": None,
            "frame_audit_report": None,
            "verdict": None,
        },
        "generated_at": None,
        "generated_by": "sidewalk_perception/make_manifest.py",
    }


def create(rec_dir, fields=None, log=None, overwrite=False):
    """Write a manifest for a recording directory.

    Auto-fills everything that can be determined by looking at the machine and
    the files themselves, so the operator only has to supply the things only a
    human knows: where they drove, what the weather was, what broke.
    """
    rec_dir = Path(rec_dir)
    if not rec_dir.is_dir():
        raise Unavailable(
            "No such recording directory: %s" % rec_dir,
            "Create the recording first with `svo_manager.py record`, or point "
            "at an existing directory under %s." % DATA_DIR)

    path = rec_dir / "manifest.yaml"
    man = blank_manifest()
    if path.exists() and not overwrite:
        # Merge rather than clobber: an operator may have hand-edited prose,
        # and destroying that would train people not to write it.
        try:
            existing = load_simple_yaml(path)
            man = _deep_merge(man, existing)
        except Exception:
            if log:
                log.warn("Existing manifest.yaml could not be parsed; it will "
                         "be replaced. A copy is at manifest.yaml.bak")
            path.rename(path.with_suffix(".yaml.bak"))

    fields = dict(fields or {})
    env = probe_environment(include_camera=False)

    # --- things the machine knows about itself
    man["software"]["zed_sdk_version"] = env.get("zed_sdk_version")
    man["software"]["zedlink_driver"] = env.get("zedlink_driver")
    man["software"]["l4t_version"] = env.get("l4t_version")
    man["software"]["jetpack_version"] = env.get("jetpack_version")
    man["software"]["ros_distro"] = env.get("ros_distro")
    man["software"]["hostname"] = env.get("hostname")
    man["software"]["kernel"] = env.get("kernel")

    # --- things the files themselves know
    files, total = [], 0
    for f in sorted(rec_dir.rglob("*")):
        if f.is_file() and f.suffix.lower() in (".svo2", ".svo", ".bag", ".txt", ".csv"):
            size = f.stat().st_size
            total += size
            files.append({"name": str(f.relative_to(rec_dir)),
                          "size_gb": round(size / 1e9, 4),
                          "modified": datetime.fromtimestamp(
                              f.stat().st_mtime).isoformat(timespec="seconds")})
    man["recording"]["files"] = files
    if total and not man["recording"].get("size_gb"):
        man["recording"]["size_gb"] = round(total / 1e9, 3)
    if not man["recording"].get("name"):
        man["recording"]["name"] = rec_dir.name
    if not man["recording"].get("date"):
        man["recording"]["date"] = datetime.now().astimezone().isoformat(timespec="seconds")
    if not man["recording"].get("operator"):
        import os
        man["recording"]["operator"] = os.environ.get("USER")

    # --- things only a human knows, supplied by the caller
    mapping = {
        "recording": ("recording", "name"),
        "duration_s": ("recording", "duration_s"),
        "size_gb": ("recording", "size_gb"),
        "compression": ("recording", "compression"),
        "notes": ("recording", "notes"),
        "camera_serial": ("camera", "serial"),
        "resolution": ("camera", "resolution"),
        "fps": ("camera", "fps"),
        "exposure": ("camera", "exposure"),
        "gain": ("camera", "gain"),
        "mount_revision": ("mount", "revision"),
        "mount_height_m": ("mount", "height_m"),
        "mount_pitch_deg": ("mount", "pitch_deg"),
        "environment_description": ("environment", "description"),
        "trajectory_description": ("environment", "trajectory"),
        "surface": ("environment", "surface"),
        "weather": ("environment", "weather"),
        "lighting": ("environment", "lighting"),
        "ground_truth_source": ("ground_truth", "source"),
        "ground_truth_file": ("ground_truth", "file"),
    }
    for key, value in fields.items():
        if value in (None, ""):
            continue
        if key in mapping:
            sec, sub = mapping[key]
            man[sec][sub] = value
        elif key == "known_anomalies" or key == "anomalies":
            man["known_anomalies"] = value
        elif "/" in key:
            sec, sub = key.split("/", 1)
            man.setdefault(sec, {})[sub] = value
        else:
            man.setdefault("extra", {})[key] = value

    man["generated_at"] = datetime.now().astimezone().isoformat(timespec="seconds")

    path.write_text(dump_simple_yaml(man) + "\n")
    (rec_dir / "manifest.json").write_text(json.dumps(man, indent=2, default=str))
    render(rec_dir, man)
    return path


def _deep_merge(base, override):
    """Merge `override` into `base`, keeping non-null override values."""
    out = dict(base)
    if not isinstance(override, dict):
        return out
    for k, v in override.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _deep_merge(out[k], v)
        elif v not in (None, "", [], {}):
            out[k] = v
    return out


def load(rec_dir):
    rec_dir = Path(rec_dir)
    path = rec_dir / "manifest.yaml"
    if not path.exists():
        raise Unavailable(
            "No manifest.yaml in %s — this recording has no provenance record."
            % rec_dir,
            "Create one now: `rosrun sidewalk_perception make_manifest.py "
            "create %s`. Do it before analysing the data, not after." % rec_dir)
    return load_simple_yaml(path)


def get_path(man, dotted):
    node = man
    for part in dotted.split("/"):
        if not isinstance(node, dict):
            return None
        node = node.get(part)
    return node


def validate(man):
    """Score a manifest against the required fields.

    Returns (verdict, missing, present_count).
    """
    missing = []
    for key, why in REQUIRED:
        v = get_path(man, key)
        if v in (None, "", "null", "unknown"):
            missing.append((key, why))
    if not missing:
        return "PASS", missing, len(REQUIRED)
    # Anything missing from the hardware/software provenance is fatal for
    # reproducibility. Missing prose is bad but recoverable if the operator
    # still remembers.
    hard = [k for k, _ in missing
            if k.startswith(("camera/", "software/", "mount/", "recording/"))]
    return ("FAIL" if hard else "WARN"), missing, len(REQUIRED) - len(missing)


# --------------------------------------------------------------------------- #
# HTML rendering
# --------------------------------------------------------------------------- #

def _row(label, value, note=""):
    v = value
    if v is None or v == "":
        v = '<span class="missing">not recorded</span>'
    elif isinstance(v, bool):
        v = "yes" if v else "no"
    else:
        v = html_escape(v)
    note_html = ('<div class="fnote">%s</div>' % html_escape(note)) if note else ""
    return "<tr><td>%s</td><td>%s%s</td></tr>" % (html_escape(label), v, note_html)


def _section(title, rows, intro=""):
    body = "".join(rows)
    intro_html = "<p>%s</p>" % html_escape(intro) if intro else ""
    return ('<section class="card"><h2>%s</h2>%s'
            '<table class="tbl"><tbody>%s</tbody></table></section>'
            % (html_escape(title), intro_html, body))


def render(rec_dir, man=None):
    """Write DATASET.html — the page a non-technical reader opens.

    Self-contained, styled with the project stylesheet inlined. No CDN, no web
    fonts, no JavaScript: it has to open from a USB stick on a lab machine with
    no internet, which is the actual situation this gets used in.
    """
    rec_dir = Path(rec_dir)
    man = man if man is not None else load(rec_dir)
    verdict, missing, n_present = validate(man)

    rec = man.get("recording") or {}
    cam = man.get("camera") or {}
    sw = man.get("software") or {}
    mount = man.get("mount") or {}
    envr = man.get("environment") or {}
    gt = man.get("ground_truth") or {}

    css = load_css() + """
    .missing { color:#b03030; font-style:italic; }
    .fnote { font-size:.85em; opacity:.7; margin-top:.25em; }
    .pill { display:inline-block; padding:.15em .6em; border-radius:1em;
            font-size:.85em; border:1px solid currentColor; margin-left:.4em; }
    .filelist td { font-variant-numeric: tabular-nums; }
    """

    P = []
    A = P.append
    name = rec.get("name") or rec_dir.name
    A('<!doctype html><html lang="en"><head><meta charset="utf-8">')
    A('<meta name="viewport" content="width=device-width,initial-scale=1">')
    A("<title>Dataset: %s — Mitacs VSLAM</title>" % html_escape(name))
    A("<style>%s</style></head><body>" % css)

    A('<header class="hd"><div class="wrap">')
    A('<a class="back" href="../../../docs/index.html">&larr; All phases</a>')
    A('<div class="crumb">Dataset &middot; <code>%s</code></div>' % html_escape(name))
    A("<h1>Recording: %s</h1>" % html_escape(name))
    A('<p class="lede">%s</p>'
      % html_escape(envr.get("description")
                    or "A recording from the sidewalk robot's ZED X camera."))
    badge = {"PASS": "s-ready", "WARN": "s-wip", "FAIL": "s-gate"}[verdict]
    A('<span class="badge %s">Provenance record: %s (%d of %d required fields)</span>'
      % (badge, verdict, n_present, len(REQUIRED)))
    A("</div></header>")
    A('<main id="main" class="wrap">')

    # ---- plain-language summary first, always
    A('<section class="card hi"><h2><span class="ic">◆</span> In plain language</h2>')
    A("<p>%s</p>" % html_escape(
        "This folder holds a recording made by the robot's stereo camera. A "
        "stereo camera is two cameras a fixed distance apart, like a pair of "
        "eyes; comparing the two views is how the robot works out how far away "
        "things are. Everything the camera saw and felt during this run — both "
        "video streams and the motion sensor — is stored here."))
    A("<p>%s</p>" % html_escape(
        "The point of saving it is that the same recording can be fed to "
        "several different mapping programs afterwards. Because they all see "
        "exactly the same drive, any difference in their results comes from "
        "the programs themselves and not from the weather, the traffic or the "
        "route. That is what makes the comparison fair."))
    A("<p>%s</p>" % html_escape(
        "The table below is the record of exactly how this recording was made. "
        "It exists because a video with no notes attached becomes unusable "
        "surprisingly fast — within months, nobody can remember which camera "
        "was fitted, how it was mounted, or what the weather was doing."))
    if envr.get("trajectory"):
        A("<p><strong>The route driven:</strong> %s</p>"
          % html_escape(envr["trajectory"]))
    A("</section>")

    if verdict != "PASS":
        A('<section class="card an"><h2>This record is incomplete</h2>')
        A("<p>%s</p>" % html_escape(
            "The following details were not filled in. Each one is something "
            "that cannot be recovered later by looking at the files — if it is "
            "not written down now, while someone still remembers, it is gone. "
            "A dataset with gaps here should not be used to support a "
            "published result."))
        A("<ul>")
        for key, why in missing:
            A("<li><code>%s</code> — %s%s</li>"
              % (html_escape(key), html_escape(why),
                 (" " + html_escape(FIELD_HELP[key])) if key in FIELD_HELP else ""))
        A("</ul></section>")

    # ---- the record itself
    A(_section("The recording", [
        _row("Name", rec.get("name")),
        _row("Recorded", rec.get("date")),
        _row("Operator", rec.get("operator")),
        _row("Duration", "%s s (%.1f min)" % (rec.get("duration_s"),
                                              float(rec["duration_s"]) / 60.0)
             if rec.get("duration_s") else None),
        _row("Total size", "%s GB" % rec.get("size_gb") if rec.get("size_gb") else None),
        _row("Compression", rec.get("compression"),
             "H.265 and H.264 use the Jetson's hardware video encoder. "
             "Software encoding would consume CPU that the mapping needs."),
        _row("Notes", rec.get("notes")),
    ]))

    A(_section("The camera that made it", [
        _row("Model", "%s (%s)" % (cam.get("model"), cam.get("sku"))),
        _row("Serial number", cam.get("serial"),
             "Identifies the physical unit. A second ZED X is planned, and "
             "each unit has its own factory calibration."),
        _row("Lens", "%s mm%s" % (cam.get("lens_mm"),
                                  "" if cam.get("polarizer") else ", no polarizer")),
        _row("Stereo baseline", "%s mm" % cam.get("baseline_mm"),
             "The distance between the two lenses. This single number, with "
             "the lens focal length, sets how far away the camera can still "
             "judge distance usefully."),
        _row("Resolution", cam.get("resolution")),
        _row("Frame rate", "%s FPS" % cam.get("fps") if cam.get("fps") else None),
        _row("Exposure", cam.get("exposure"),
             "Long exposures blur the image while the robot moves, and motion "
             "blur destroys the sharp corners that visual tracking depends on."),
        _row("Gain", cam.get("gain")),
        _row("IMU rate", "%s Hz" % cam.get("imu_rate_hz") if cam.get("imu_rate_hz") else None),
        _row("Connection", cam.get("interface")),
        _row("Capture card", "%s (%s)" % (cam.get("capture_card"),
                                          cam.get("capture_card_sku"))),
    ], "The camera is a Stereolabs ZED X: two colour sensors behind two lenses, "
       "a fixed distance apart, plus a built-in motion sensor."))

    A(_section("The software that processed it", [
        _row("ZED SDK", sw.get("zed_sdk_version"),
             "The SDK computes depth from the two images. A different SDK "
             "version can produce measurably different depth from identical "
             "input, so comparing across versions is not valid."),
        _row("Capture card driver", sw.get("zedlink_driver")),
        _row("Jetson OS (L4T)", sw.get("l4t_version")),
        _row("JetPack", sw.get("jetpack_version")),
        _row("ROS distribution", sw.get("ros_distro")),
        _row("zed-ros-wrapper branch", sw.get("zed_ros_wrapper_branch")),
        _row("Host machine", sw.get("hostname")),
        _row("Kernel", sw.get("kernel")),
    ]))

    A(_section("How the camera was mounted", [
        _row("Bracket revision", mount.get("revision"),
             "Any change to the bracket moves the camera relative to the "
             "wheels, which changes every measurement the robot makes."),
        _row("Height above ground", "%s m" % mount.get("height_m")
             if mount.get("height_m") else None),
        _row("Pitch angle", "%s deg" % mount.get("pitch_deg")
             if mount.get("pitch_deg") else None,
             "A one-degree pitch error barely matters at arm's length, but "
             "displaces a point 20 m away by about 35 cm."),
        _row("Notes", mount.get("notes")),
    ]))

    A(_section("Where and what was driven", [
        _row("Description", envr.get("description")),
        _row("Route", envr.get("trajectory"),
             "A route that returns to its starting point can test loop "
             "closure — whether the robot recognises somewhere it has already "
             "been. An open-ended route cannot."),
        _row("Surface", envr.get("surface"),
             "Texture matters: stereo matching needs visible detail. Fresh "
             "asphalt, snow and standing water all give far fewer usable "
             "depth measurements than a textured pavement."),
        _row("Weather", envr.get("weather")),
        _row("Lighting", envr.get("lighting")),
        _row("Pedestrians", envr.get("pedestrians"),
             "Moving people violate the assumption that the world is still, "
             "which is what visual odometry relies on."),
    ]))

    A(_section("What the results will be compared against", [
        _row("Ground truth source", gt.get("source"),
             "Without an independent reference, an estimated path can only be "
             "compared to other estimates, never checked for correctness."),
        _row("Format", gt.get("format")),
        _row("File", gt.get("file")),
        _row("Notes", gt.get("notes")),
    ]))

    if man.get("known_anomalies"):
        A('<section class="card an"><h2>Known problems with this recording</h2>')
        A("<p>%s</p>" % html_escape(man["known_anomalies"]))
        A("<p>%s</p>" % html_escape(
            "This is recorded deliberately. A known fault noted at the time "
            "costs nothing; the same fault rediscovered six months later, "
            "while chasing an anomalous result, costs days."))
        A("</section>")

    files = rec.get("files") or []
    if files:
        rows = ["<tr><td><code>%s</code></td><td>%s GB</td><td>%s</td></tr>"
                % (html_escape(f.get("name")), html_escape(f.get("size_gb")),
                   html_escape(f.get("modified")))
                for f in files if isinstance(f, dict)]
        A('<section class="card"><h2>Files</h2><table class="tbl filelist">'
          '<thead><tr><th>File</th><th>Size</th><th>Written</th></tr></thead>'
          '<tbody>%s</tbody></table></section>' % "".join(rows))

    val = man.get("validation") or {}
    if any(val.values()):
        A(_section("Sensor validation", [
            _row("Verdict", val.get("verdict")),
            _row("Validator report", val.get("sensor_validator_report")),
            _row("Frame audit report", val.get("frame_audit_report")),
        ], "Whether the data stream itself was measured healthy — frame rate, "
           "dropped frames, timestamp consistency — before any conclusion was "
           "drawn from it."))

    A('<footer class="ft">Generated by <code>sidewalk_perception/make_manifest.py</code> '
      'on %s &middot; Mitacs Globalink 2026 &middot; Project 50124 &middot; THAW ZIN</footer>'
      % datetime.now().strftime("%Y-%m-%d %H:%M"))
    A("</main></body></html>")

    out = rec_dir / "DATASET.html"
    out.write_text("\n".join(P))
    return out


def build_index(data_dir=None):
    """Build one page listing every dataset — the front door to "the database"."""
    d = Path(data_dir) if data_dir else DATA_DIR
    d.mkdir(parents=True, exist_ok=True)
    rows = []
    for sub in sorted(p for p in d.iterdir() if p.is_dir()):
        man_path = sub / "manifest.yaml"
        if not man_path.exists():
            rows.append((sub.name, None, "no manifest", "FAIL", None))
            continue
        try:
            man = load_simple_yaml(man_path)
        except Exception:
            rows.append((sub.name, None, "manifest unreadable", "FAIL", None))
            continue
        verdict, _missing, _n = validate(man)
        rec = man.get("recording") or {}
        envr = man.get("environment") or {}
        rows.append((sub.name, rec.get("date"),
                     envr.get("description") or "", verdict, rec.get("duration_s")))

    css = load_css() + """
    .missing { color:#b03030; font-style:italic; }
    .v-PASS { color:#1a7f37; font-weight:600; }
    .v-WARN { color:#9a6700; font-weight:600; }
    .v-FAIL { color:#b03030; font-weight:600; }
    """
    P = ['<!doctype html><html lang="en"><head><meta charset="utf-8">',
         '<meta name="viewport" content="width=device-width,initial-scale=1">',
         "<title>Recorded datasets — Mitacs VSLAM</title>",
         "<style>%s</style></head><body>" % css,
         '<header class="hd"><div class="wrap">',
         '<a class="back" href="../../docs/index.html">&larr; All phases</a>',
         '<div class="crumb">Phase 2 &middot; <code>data/recordings</code></div>',
         "<h1>Recorded datasets</h1>",
         '<p class="lede">Every recording the robot has made, and how complete '
         'its provenance record is.</p>',
         "</div></header>", '<main class="wrap">',
         '<section class="card hi"><h2><span class="ic">◆</span> What this page is</h2>',
         "<p>%s</p>" % html_escape(
             "Each row is one drive the robot recorded. Click a name to see "
             "exactly how that recording was made — which camera, which "
             "settings, which software, what the weather was doing."),
         "<p>%s</p>" % html_escape(
             "The 'record' column says whether we captured enough detail about "
             "the conditions to trust the recording for a published result. "
             "A recording marked incomplete is still perfectly good video; it "
             "just cannot be fully accounted for."),
         "</section>"]

    if not rows:
        P.append('<section class="card"><h2>No recordings yet</h2>'
                 '<p>Nothing has been recorded. The camera is not wired up '
                 'yet. Once it is, <code>svo_manager.py record</code> will '
                 'create the first dataset here, and it will appear on this '
                 'page automatically.</p></section>')
    else:
        body = "".join(
            '<tr><td><a href="%s/DATASET.html"><code>%s</code></a></td>'
            '<td>%s</td><td>%s</td><td class="v-%s">%s</td><td>%s</td></tr>'
            % (html_escape(n), html_escape(n), html_escape(date or "—"),
               html_escape(desc or "—"), v, v,
               ("%.1f min" % (float(dur) / 60.0)) if dur else "—")
            for n, date, desc, v, dur in rows)
        P.append('<section class="card"><h2>%d dataset(s)</h2>'
                 '<table class="tbl"><thead><tr><th>Dataset</th><th>Recorded</th>'
                 '<th>Description</th><th>Record</th><th>Length</th></tr></thead>'
                 '<tbody>%s</tbody></table></section>' % (len(rows), body))

    P.append('<footer class="ft">Generated by '
             '<code>sidewalk_perception/make_manifest.py</code> on %s &middot; '
             'Mitacs Globalink 2026 &middot; Project 50124</footer>'
             % datetime.now().strftime("%Y-%m-%d %H:%M"))
    P.append("</main></body></html>")
    out = d / "index.html"
    out.write_text("\n".join(P))
    return out


# --------------------------------------------------------------------------- #

def build_parser():
    p = argparse.ArgumentParser(
        description="Create, edit, validate and render dataset manifests — the "
                    "provenance record for every recording.")
    sub = p.add_subparsers(dest="command")

    c = sub.add_parser("create", help="Create or update a manifest")
    c.add_argument("path", help="Recording directory")
    c.add_argument("--field", action="append", default=[], metavar="KEY=VALUE",
                   help="Set a field, e.g. --field environment/weather='light rain'. "
                        "Repeatable.")
    c.add_argument("--overwrite", action="store_true",
                   help="Discard any existing manifest instead of merging")

    e = sub.add_parser("edit", help="Update fields in an existing manifest")
    e.add_argument("path")
    e.add_argument("--field", action="append", default=[], metavar="KEY=VALUE")

    r = sub.add_parser("render", help="Regenerate DATASET.html from manifest.yaml")
    r.add_argument("path")

    v = sub.add_parser("validate", help="Report which required fields are missing")
    v.add_argument("path")

    i = sub.add_parser("index", help="Rebuild the dataset index page")
    i.add_argument("--data-dir", default="")

    sub.add_parser("schema", help="Print the manifest schema and field guidance")
    return p


def _parse_fields(pairs):
    out = {}
    for item in pairs:
        if "=" not in item:
            raise Unavailable(
                "Field `%s` is not in KEY=VALUE form." % item,
                "Write it as --field environment/weather='light rain'.")
        k, _, v = item.partition("=")
        out[k.strip()] = v.strip()
    return out


def main(argv=None):
    argv = [a for a in (sys.argv[1:] if argv is None else argv)
            if not a.startswith("__") and ":=" not in a]
    parser = build_parser()
    args = parser.parse_args(argv)
    if not args.command:
        parser.print_help()
        return EXIT_FAIL

    with RunLogger("sidewalk_perception",
                   run_name="manifest_%s" % args.command) as log:
        try:
            if args.command == "schema":
                log.section("Manifest schema")
                log.info(dump_simple_yaml(blank_manifest()))
                log.section("Required fields")
                for key, why in REQUIRED:
                    log.info("  %-38s %s" % (key, why))
                    if key in FIELD_HELP:
                        log.info("      %s" % FIELD_HELP[key])
                log.summary("Printed manifest schema", status="OK")
                return EXIT_PASS

            if args.command == "index":
                out = build_index(args.data_dir or None)
                log.info("Dataset index written: %s" % out)
                log.summary("Rebuilt dataset index", status="OK")
                return EXIT_PASS

            rec_dir = Path(args.path)
            if args.command in ("create", "edit"):
                fields = _parse_fields(args.field)
                path = create(rec_dir, fields, log=log,
                              overwrite=getattr(args, "overwrite", False))
                man = load(rec_dir)
                verdict, missing, n = validate(man)
                log.info("Manifest : %s" % path)
                log.info("JSON     : %s" % (rec_dir / "manifest.json"))
                log.info("HTML     : %s" % (rec_dir / "DATASET.html"))
                _report_validation(log, verdict, missing, n)
                log.summary("Manifest for %s is %s (%d/%d required fields)"
                            % (rec_dir.name, verdict, n, len(REQUIRED)),
                            status=verdict)
                return EXIT_PASS

            if args.command == "render":
                man = load(rec_dir)
                out = render(rec_dir, man)
                log.info("Rendered %s" % out)
                log.summary("Rendered dataset page for %s" % rec_dir.name, status="OK")
                return EXIT_PASS

            if args.command == "validate":
                man = load(rec_dir)
                verdict, missing, n = validate(man)
                _report_validation(log, verdict, missing, n)
                log.summary("Manifest for %s is %s (%d/%d required fields)"
                            % (rec_dir.name, verdict, n, len(REQUIRED)),
                            status=verdict)
                return EXIT_PASS if verdict != "FAIL" else EXIT_FAIL
        except Unavailable as exc:
            return die(log, exc)
    return EXIT_FAIL


def _report_validation(log, verdict, missing, n_present):
    log.section("Completeness")
    log.metric("required_fields_present", n_present)
    log.metric("required_fields_total", len(REQUIRED))
    if not missing:
        log.info("All %d required fields are recorded." % len(REQUIRED))
        return
    for key, why in missing:
        log.warn("MISSING %-34s (%s)" % (key, why))
        if key in FIELD_HELP:
            log.info("        %s" % FIELD_HELP[key])
    log.warn("Fill these in with: make_manifest.py edit <dir> --field key=value")


if __name__ == "__main__":
    sys.exit(main())
