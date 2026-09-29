#!/usr/bin/env python3
"""Copy the drive results packs into the website's Results page (public/results/).

Run on the Jetson whenever a results pack changes:
    python3 tools/live_site/results_website/build_results.py

Reads   results/series2_static/<run>/ and results/series2_slam/<run>/   (the results packs;
        $RESULTS_DIR overrides the results/ folder)
        an optional INDEX.md beside them (one headline line per pack; skipped if absent)
Writes  public/results/<run>/<picture or video>       (copies; small files only)
        public/results/results.json                   (what results.html draws)

Only pictures (.png) and videos (.mp4) under 30 MB are copied, plus each pack's RESULTS.md
text. Never raw recordings. The whole site is behind the view key (live_site_server.py), so these
files are not public.
"""
import json
import os
import re
import shlex
import shutil

HERE = os.path.dirname(os.path.abspath(__file__))
RESULTS = os.environ.get("RESULTS_DIR") or os.path.join(HERE, "..", "..", "..", "results")   # the repository's results/
RUNS = os.path.join(RESULTS, "series2_static")
RUN_ROOTS = [RUNS, os.path.join(RESULTS, "series2_slam")]   # a run is looked up in each, in order
INDEX = os.path.join(RESULTS, "INDEX.md")
OUT = os.path.join(HERE, "public", "results")
MAX_BYTES = 30 * 1000 * 1000
LINKS = {    # run -> [(page in public/results/<run>/, caption, preview picture)] - listed first in the 3D section
    "s2_static_10": [("pointcloud.html", "Interactive 3D point cloud of the whole floor (cleaned camera map; light copy 1 M points, 'full detail' 2.9 M) - auto-spin on", "d10_clean_topdown.png")],
}
EXTRAS = {   # run -> [(path inside the pack folder, caption)], copied at full size
    "s2_static_10": [
        ("flythrough/drive10_camera_view.mp4", "3D fly-through, robot camera view: the whole floor in the camera's 3D map, about x15 speed (full quality)"),
        ("flythrough/drive10_chase_view.mp4", "3D fly-through, from behind and above the robot (yellow box), path so far in cyan (full quality)"),
        ("flythrough/drive10_camera_still_1300.png", "3D fly-through still - camera view"),
        ("flythrough/drive10_chase_still_1300.png", "3D fly-through still - chase view"),
        ("map_3d/d10_3d.png", "3D map of the whole floor (camera, 4.5 M points, corrected positions)"),
        ("map_3d/d10_topdown.png", "The whole floor from above (camera 3D map, points below 2 m)"),
        ("map_3d_clean/d10_clean_topdown.png", "Cleaned whole-floor map from above: 3 m depth limit + plain-wall rule, 1 cm detail (walls thrown into the corridor ends removed, about 90 %)"),
        ("map_3d_clean/d10_clean_before_after.png", "Before / after cleaning, with the two corridor ends zoomed (red = wall points on open floor)"),
        ("map_3d_clean/d10_corner_diagnosis.png", "Why the corridor ends were wrong: the camera reads plain end walls too close beyond 3 m (a depth fault, not turning)"),
    ],
}

# the order and plain names shown on the page
PACKS = [
    ("s2_slam_01", "SLAM - continuing drive 10's whole-floor map", "27 Sept 2026, 13:25-13:52 - SLAM on top of drive 10's saved map: joined it at once (0.08 m from the start mark), 156 m driven, 238 links to the old map, start-to-end gap 0.044 m corrected (0.221 m tracking alone), 73 m2 of new corridor added, drive 10's map moved by a median 0.019 m; closed properly. Pass lines S1, S2 (0 wrong of 238 joins), S3 and S5 PASS; S4 not scorable (this drive\'s LiDAR estimate failed its own check, 0.52 m end gap). People blanking (the live run hit a label bug; the recording replayed off / on): old map pushed 0.36 -> 0.09 m"),
    ("s2_loc_04", "Localisation - the whole floor", "27 Sept 2026, 07:43-08:07 - on drive 10's whole-floor map, told it was 40 m outside the building: W1 FAIL (first fix after 212 s, line 120 s), W2 PASS (fixes at 3 of 3 walkway-end stops), W3 PASS (0 wrong places of 575 fixes), W4 FAIL (vs the LiDAR estimate median 0.64 m, line 0.50 m) - W4 is the map's size (drive 10's own map scores 0.69 m; size fit 1.068; size fitted, information only, 0.20 m), not the localiser losing its place"),
    ("s2_static_10", "Drive 10 - the whole 2nd floor", "27 Sept 2026, 02:02-02:31 - the whole 2nd floor (about 252 m by the camera map), fresh Jetson boot, tracker GEN_2, parked and closed properly: camera map, corrected, came home to 0.03 m with 199 loop closures (3 joining the start); LiDAR estimate (Force3DoF replay) 0.04 m, 76 closures, although the robot's own wheels + gyroscope ended 2.03 m / -15 deg off; camera map vs LiDAR estimate 0.69 / 1.15 m median / 95th percentile (drive 9: 0.42 / 0.65 m) - mostly a size difference (best-fit size factor 1.072, as the tape test predicts from the wheels; size fitted, for information, 0.14 / 0.29 m); no camera crash or hang; 1 freeze"),
    ("s2_static_09", "Drive 9 - the first longer drive", "26 Sept 2026, 20:52-21:21 - drive 4's route plus new corridors (about 247 m by the camera map), fresh Jetson boot, tracker GEN_2: camera map, corrected, came home to 0.01 m with 305 loop closures, none joining the start; LiDAR estimate (Force3DoF replay) 0.01 m, 379 closures; camera map vs LiDAR estimate 0.42 / 0.65 m median / 95th percentile (drive 4: 0.26 / 0.49 m) - kept drive 4's quality only partly; camera hung once (not a crash), back in 28.3 s; 6 freezes; no park - WiFi lost, Jetson power-cycled, so the corrected path is the graph re-optimised afterwards"),
    ("s2_static_08", "Drive 8 - drive 4's route repeated", "26 Sept 2026, 15:48-16:02 - the \"same result every time\" test, camera tracker GEN_2: camera map, corrected, came home to 0.10 m with 145 loop closures, 0 joining the start (drive 4: 0.05 m, 181, 3 joining the start), driven along drive 4's line; camera crashed once, back in 12.7 s; 15 freezes; LiDAR map failed its own check (not a yardstick)"),
    ("s2_static_07", "Drive 7", "26 Sept 2026, 13:28-13:41 - first mapping drive on the camera tracker GEN_1; no crash, 16 freezes, 30 loop closures; LiDAR made 0 closures (not a yardstick)"),
    ("s2_scale_01", "15 m distance test", "26 Sept 2026, 13:10-13:22 - taped 15.00 m: camera 14.90 m (-0.7 %), wheels 15.94 m (+6.2 %)"),
    ("s2_loc_03", "Localisation attempt 3", "26 Sept 2026, 10:41-11:06 - on drive 4's map, off-map start guess, LoopThr 0.08: 4 of 5 starts found, 3 in time, 0 wrong places shown"),
    ("s2_loc_02", "Localisation session 2", "26 Sept 2026, 06:16-06:34 - on drive 4's map, LoopThr 0.05: 1 of 5 starts in time, 4 wrong places shown"),
    ("s2_static_06", "Drive 6", "26 Sept 2026, 04:21-04:45 - new walkway; camera crashed and was back in 12.7 s; both maps drifted on the walkway"),
    ("s2_static_05", "Drive 5", "26 Sept 2026, 02:14-02:37 - blend as drive 4, confirmed saves off; camera program crashed at 02:25:26, camera map covers the first 11 min"),
    ("s2_static_04", "Drive 4", "26 Sept 2026, 00:44-00:53 - camera + wheels + gyroscope + camera gyroscope blended, 16-bit depth"),
    ("s2_static_03", "Drive 3", "25 Sept 2026, 04:30-04:58 - camera + wheels + gyroscope blended"),
    ("s2_fusion_T4b", "Rehearsal T4b", "25 Sept 2026, 03:58-04:14 - parked rehearsal before drive 3"),
    ("s2_static_02", "Drive 2", "24 Sept 2026 - camera only"),
    ("s2_static_01", "Drive 1", "24 Sept 2026 - camera only"),
]
# plain captions for the files the packs use; anything else keeps its file name
CAPTIONS = {
    "map.png": "Floor map from the camera, positions as tracked (tracking alone)",
    "map_corrected.png": "Floor map after the map's own corrections (loop closures)",
    "trajectory.png": "Path driven: robot's wheels + gyroscope solid, camera and blend dashed",
    "turns_difference.png": "Each turn: blend minus robot heading (degrees)",
    "timelapse_rebuilt.mp4": "Timelapse of the floor map growing - rebuilt after the drive from the saved map",
    "timelapse.mp4": "Timelapse of the floor map growing - recorded live during the drive",
    "heading_timeline.png": "Heading through the rehearsal: robot solid, blend dashed",
    "lidar_comparison.png": "Agreement with the LiDAR estimate (LiDAR solid, camera dashed) - not ground truth",
    "lidar_map.png": "LiDAR floor map: walls seen by the laser scanner, placed by the robot's wheels + gyroscope, corrected where the LiDAR recognised a place or lined up with nearby scans",
    "timelapse_25.png": "Timelapse frame at 25 %", "timelapse_50.png": "Timelapse frame at 50 %",
    "timelapse_75.png": "Timelapse frame at 75 %", "timelapse_100.png": "Timelapse frame at the end",
}

# blend's position, and its figures split tracking alone from corrected - rule 20). Drives 1-3 keep CAPTIONS.
PACK_CAPTIONS = {
    "s2_static_06": None, "s2_static_07": None, "s2_static_08": None, "s2_static_09": None,
    "s2_static_05": {
        "map.png": "Floor map, map nodes at their tracking-alone positions (the blend's, in a fused drive) - up to the camera crash at 02:25",
        "map_corrected.png": "Floor map after the map's own corrections (loop closures) - up to the camera crash at 02:25",
        "trajectory.png": "Path driven, two panels: tracking alone (robot's wheels + gyroscope solid; camera tracking, ending at the crash, and blend dashed) and corrected (LiDAR estimate solid, whole drive; camera map dashed, to the crash)",
        "turns_difference.png": "Each turn: camera minus robot and blend minus robot heading (degrees); no camera after the crash",
        "lidar_comparison.png": "Agreement with the LiDAR estimate, camera-alive part only - not ground truth (LiDAR solid, camera map dashed, map nodes = blend dotted; tracking alone and corrected on separate panels)",
        "lidar_carving_score.png": "LiDAR clearing test (staged person): N1 and N4 failed their registered lines",
        "timelapse.mp4": "Timelapse of the live map page (camera map left, LiDAR view right) - recorded live; the camera side says NO DATA from 02:25:29 (camera crash)",
        "timelapse_crash.png": "Last timelapse frame before the camera crash (02:25:23)",
    },
    "s2_static_04": {
        "map.png": "Floor map, map nodes at their tracking-alone positions (in a fused drive these are the blend's, not the camera's)",
        "trajectory.png": "Path driven, two panels: tracking alone (robot's wheels + gyroscope solid; camera tracking and blend dashed) and corrected (LiDAR estimate solid; camera map dashed)",
        "turns_difference.png": "Each turn: camera minus robot and blend minus robot heading (degrees)",
        "lidar_comparison.png": "Agreement with the LiDAR estimate - not ground truth (LiDAR solid, camera map dashed, map nodes = blend dotted; tracking alone and corrected on separate panels)",
        "timelapse.mp4": "Timelapse of the live map page (camera map left, LiDAR view right) - recorded live during the drive",
    },
}
for _r in ("s2_static_06", "s2_static_07", "s2_static_08"):
    PACK_CAPTIONS[_r] = dict(PACK_CAPTIONS["s2_static_04"])
PACK_CAPTIONS["s2_static_08"]["shared_route.png"] = ("Shared route: drive 8's and drive 7's camera maps (corrected, dashed) lined up on drive 4's; "
                                                     "the part within 0.5 m of drive 4's route drawn heavy, the rest black")
PACK_CAPTIONS["s2_static_09"] = dict(PACK_CAPTIONS["s2_static_04"])
PACK_CAPTIONS["s2_static_09"].update({
    "map_corrected.png": "Floor map after the map's own corrections (loop closures) - graph re-optimised afterwards (optimize_graph_se2.py), because the Jetson was power-cycled before the map was closed",
    "trajectory.png": "Path driven, two panels: tracking alone (robot's wheels + gyroscope solid, ending 21:15:59 when its data stopped reaching the Jetson; camera and blend dashed) and corrected (LiDAR estimate, Force3DoF replay, solid; camera map dashed)",
    "lidar_comparison.png": "Agreement with the LiDAR estimate (Force3DoF replay) - not ground truth (LiDAR solid, camera map dashed, map nodes = blend dotted)",
    "lidar_map.png": "LiDAR floor map, Force3DoF replay (held flat on the floor): walls seen by the laser scanner, placed by the robot's wheels + gyroscope, corrected by 379 LiDAR loop closures",
    "timelapse.mp4": "Timelapse of the live map page (camera map left, LiDAR view right) - frames saved live every 3 s; the video was built from them afterwards (no park)",
    "shared_route.png": "Shared route: drive 9's and drive 8's camera maps (corrected, dashed) lined up on drive 4's; the part within 0.5 m of drive 4's route drawn heavy, the rest black",
})
PACK_CAPTIONS["s2_slam_01"] = {
    "slam_map.png": "SLAM drive on drive 10's map: drive 10's places grey, today's corrected path dashed, new ground (more than 1 m from any drive-10 place) heavy red - areas A (north, top right), D (north of the start), B (west, bottom left), E (east, bottom right)",
    "blanking_comparison.png": "People blanking: the live drive and the SAME camera recording replayed twice (blanking off / on). Colour = how far each of drive 10's map places was pushed; blanking ON pushed the old map about 4x less (median 0.09 m vs 0.36 m, N = 1). Both replays use the camera alone and stop at the end of area D (red circle), so their 12-13 m end distances are not drift",
    "old_map_shift.png": "How far today's corrections moved each of drive 10's 860 map places - the old map stayed put (median 0.019 m)",
    "lidar_map.png": "This drive's LiDAR map (Force3DoF replay on the Jetson): it ends 0.52 m from its start and bends - it failed its own check, so camera vs LiDAR is not scored for this drive",
    "timelapse.mp4": "Timelapse of the live map page during the SLAM drive (camera map left, LiDAR view right) - recorded live every 3 s",
}
PACK_CAPTIONS["s2_loc_04"] = {
    "w4_lidar_comparison.png": "W4: where the localiser said it was (dashed) vs the LiDAR estimate (solid) - not ground truth; the distance peaks at the far corners, the pattern of the map being about 7 % too large",
    "floor_fixes.png": "Where on the floor the localiser recognised a place (fixes), on drive 10's map",
    "fixes_per_10m.png": "Recognitions per 10 m driven, along the route",
    "timeline.png": "Session timeline: recognitions, stops, speed",
    "start2_first_fix_diagnosis.png": "W1 diagnosis: why the first fix took 212 s - standing still does not raise the recognition score; driving does",
    "timelapse.mp4": "Timelapse of the live map page during the localisation session",
}
PACK_CAPTIONS["s2_static_10"] = dict(PACK_CAPTIONS["s2_static_04"])
PACK_CAPTIONS["s2_static_10"].update({
    "map_corrected.png": "Floor map after the map's own corrections (loop closures) - RTAB-Map's own saved corrected positions (map closed properly at park)",
    "trajectory.png": "Path driven, two panels: tracking alone (robot's wheels + gyroscope as received over WiFi solid; camera and blend dashed) and corrected (LiDAR estimate, Force3DoF replay, solid; camera map dashed)",
    "lidar_comparison.png": "Agreement with the LiDAR estimate (Force3DoF replay) - not ground truth (LiDAR solid, camera map dashed, map nodes = blend dotted); the distance peaks at the far corners, the pattern of a size difference",
    "lidar_map.png": "LiDAR floor map, Force3DoF replay (held flat on the floor): walls seen by the laser scanner, placed by the robot's wheels + gyroscope, corrected by 76 LiDAR loop closures",
    "d10_camera_vs_lidar_side_by_side.png": "Side-by-side picture: camera map (left) and LiDAR map (middle, Force3DoF replay) from above, same scale, paths LiDAR solid / camera dashed; right, the last live LiDAR frame with the default settings (tilt from the IMU) - doubled walls",
    "shared_route.png": "Shared route: drive 10's camera map (corrected, dashed) lined up on drives 4 and 9 (and drives 9, 8 on drive 4); the part within 0.5 m drawn heavy, the rest black",
    "bridge_freshness.png": "Robot messages reaching the blend over WiFi, and where on the route the blend had no fresh robot data",
    "timelapse.mp4": "Timelapse of the live map page (camera map left, LiDAR view with default settings right) - recorded live during the drive",
})
for _r in ("s2_static_06", "s2_static_07", "s2_static_08"):   # their LiDAR estimates failed their own check (agreement.json)
    PACK_CAPTIONS[_r]["lidar_comparison.png"] = ("Camera compared with the LiDAR estimate - this drive's LiDAR estimate FAILED its own check, "
                                                 "so this compares two estimates, not the camera against a yardstick (LiDAR solid, camera map dashed, map nodes = blend dotted)")

# which section of a drive each file belongs to: the camera's own results, the LiDAR's own
# results (the colleague's LiDAR mapping on the robot), and the two compared
SECTIONS = [("camera", "Camera (ZED X on the Jetson)"),
            ("lidar", "LiDAR (Helios laser scanner, mapped on the robot)"),
            ("compare", "Camera compared with the LiDAR estimate"),
            ("3d", "3D map and fly-through videos"),
            ("timelapse", "Timelapse")]


def section_of(f):
    if f.startswith("timelapse"):
        return "timelapse"
    if f.startswith("lidar_comparison") or f.startswith("agreement"):
        return "compare"
    if f.startswith("lidar"):
        return "lidar"
    return "camera"


def lidar_facts(src):
    """The LiDAR map's own numbers, straight from the files the robot wrote (names kept)."""
    out = {}
    for name, keys in (("lidar_map_facts.json", ("nodes", "duration_s", "path_m_nodes", "closed_loop_gap_m",
                                                  "cells_solid", "map_width_m", "map_height_m")),
                       ("lidar.tum.meta.json", ("loop_closures", "end_gap_corrected_m", "method"))):
        try:
            with open(os.path.join(src, name)) as fh:
                j = json.load(fh)
        except (OSError, ValueError):
            continue
        if not isinstance(j, dict):
            continue
        for k in keys:
            if k in j:
                out[k] = j[k]
        out.setdefault("files", []).append(name)
    return out


def headlines():
    """run -> the INDEX.md row's last column (its headline, with the file it comes from)."""
    out = {}
    try:
        for line in open(INDEX, encoding="utf-8"):
            if not line.startswith("| s2_"):
                continue
            cells = [c.strip() for c in line.strip().strip("|").split("|")]
            run = cells[0].split()[0]
            out[run] = {"kind": cells[1] if len(cells) > 1 else "",
                        "video": cells[3] if len(cells) > 3 else "",
                        "headline": cells[-1]}
    except OSError:
        pass
    return out


def main():
    heads = headlines()
    # NEVER deletes (ENGINEERING_NOTES.md rule 9): files are copied over in place; a file left here that
    # its pack no longer has is listed at the end as an rm line for the user to run
    os.makedirs(OUT, exist_ok=True)
    before = set()
    for root, _, fs in os.walk(OUT):
        for f in fs:
            before.add(os.path.relpath(os.path.join(root, f), OUT))
    written = {"results.json"}
    packs = []
    for run, name, when in PACKS:
        src = next((os.path.join(r, run) for r in RUN_ROOTS if os.path.isdir(os.path.join(r, run))), None)
        if src is None:
            continue
        dst = os.path.join(OUT, run)
        os.makedirs(dst, exist_ok=True)
        files = []
        for f in sorted(os.listdir(src)):
            p = os.path.join(src, f)
            if not os.path.isfile(p) or not f.endswith((".png", ".mp4")):
                continue
            if os.path.getsize(p) > MAX_BYTES:
                continue
            shutil.copy2(p, os.path.join(dst, f))
            written.add(os.path.join(run, f))
            files.append({"file": f, "kind": "video" if f.endswith(".mp4") else "picture",
                          "section": section_of(f),
                          "caption": PACK_CAPTIONS.get(run, {}).get(f) or CAPTIONS.get(f, f),
                          "bytes": os.path.getsize(p)})
        for rel, cap in EXTRAS.get(run, []):
            p = os.path.join(src, rel)
            if not os.path.isfile(p):
                continue
            f = os.path.basename(rel)
            shutil.copy2(p, os.path.join(dst, f))
            written.add(os.path.join(run, f))
            files.append({"file": f, "kind": "video" if f.endswith(".mp4") else "picture",
                          "section": "3d", "caption": cap, "bytes": os.path.getsize(p)})
        for page, cap, preview in LINKS.get(run, []):        # pages kept in the site folder (e.g. the 3D viewer)
            if os.path.isfile(os.path.join(dst, page)):
                written.add(os.path.join(run, page))
                files.insert(0, {"file": page, "kind": "link", "section": "3d", "caption": cap, "preview": preview,
                                 "bytes": os.path.getsize(os.path.join(dst, page))})
        # pictures of the main results first, timelapse stills last
        files.sort(key=lambda x: (x["kind"] != "video", x["file"].startswith("timelapse_"),
                                  list(CAPTIONS).index(x["file"]) if x["file"] in CAPTIONS else 99))
        text = ""
        rp = os.path.join(src, "RESULTS.md")
        if os.path.isfile(rp):
            text = open(rp, encoding="utf-8").read()
        h = heads.get(run, {})
        packs.append({"run": run, "name": name, "when": when, "kind": h.get("kind", ""),
                      "headline": h.get("headline", ""), "video_note": h.get("video", ""),
                      "files": files, "results_md": text, "lidar": lidar_facts(src)})
    with open(os.path.join(OUT, "results.json"), "w") as fh:
        json.dump({"packs": packs, "sections": SECTIONS,
                   "source": "results/series2_static + results/series2_slam"}, fh, indent=1)
    total = sum(f["bytes"] for p in packs for f in p["files"])
    print("results page: %d packs, %d files, %.1f MB" % (
        len(packs), sum(len(p["files"]) for p in packs), total / 1e6))
    stale = sorted(before - written)
    if stale:
        print("these files are no longer in any pack; to remove them from the site (Jetson):")
        for f in stale:
            print("  rm %s" % shlex.quote(os.path.join(OUT, f)))


if __name__ == "__main__":
    main()
