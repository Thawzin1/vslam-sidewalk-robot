#!/usr/bin/env python3
"""
map_manager.py — save, inspect, export and prepare RTAB-Map maps.

WHY THIS EXISTS
    RTAB-Map keeps everything in ONE file: a SQLite database (`.db`) holding the
    pose graph, the visual words, the raw depth and colour of every node, and
    the occupancy grid. That single-file design is a gift for reproducibility -
    the whole map is one artefact you can copy, archive and diff - and a trap
    for everyone who has not been told about it:

      * `--delete_db_on_start` is in most RTAB-Map example launch files. It does
        exactly what it says. Run a mapping launch twice and yesterday's map is
        gone.
      * The database keeps all raw sensor data by default, so an hour of
        1920x1200 stereo is measured in gigabytes, not megabytes.
      * Navigation does not want the database. move_base wants a
        `nav_msgs/OccupancyGrid` and the standard pgm+yaml pair on disk, which
        has to be exported.

    This tool wraps those operations so nobody has to remember the flags, and so
    every export leaves a record of where the map came from.

SUBCOMMANDS
    list             what maps exist, how big, when made
    info             interrogate a database: nodes, words, loop closures
    export           write a 3D point cloud (.ply) out of a database
    save-grid        save the live 2D occupancy grid as pgm + yaml for move_base
    prepare          copy a mapping database into a protected localisation copy
    timelapse        repeatedly save the live 2D grid while you drive, to watch
                     the map get built up afterwards
    timelapse-video  stitch a `timelapse` run's snapshots into an .avi

USAGE
    rosrun sidewalk_slam map_manager.py list
    rosrun sidewalk_slam map_manager.py info  --db <path>
    rosrun sidewalk_slam map_manager.py export --db <path> --voxel 0.03
    rosrun sidewalk_slam map_manager.py save-grid --name lab_floor2
    rosrun sidewalk_slam map_manager.py prepare --db <path> --name lab_floor2
    rosrun sidewalk_slam map_manager.py timelapse --name office_run1
    rosrun sidewalk_slam map_manager.py timelapse-video --name office_run1
"""
from __future__ import annotations

import argparse
import json
import shutil
import sys
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from slam_common import (SlamExit, die, make_logger, map_dir, probe_ros,  # noqa: E402
                         probe_rtabmap_core, run_cmd)


def parse_args(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("list", help="list databases in logs/sidewalk_slam/maps/")

    p_info = sub.add_parser("info", help="interrogate a database")
    p_info.add_argument("--db", required=True)

    p_exp = sub.add_parser("export", help="export a 3D cloud from a database")
    p_exp.add_argument("--db", required=True)
    p_exp.add_argument("--voxel", type=float, default=0.03,
                       help="voxel size in metres for downsampling. 0.03 keeps "
                            "a room readable at a manageable file size; 0.01 "
                            "quadruples the point count for detail you will "
                            "not use in navigation. (default: 0.03)")
    p_exp.add_argument("--max-range", type=float, default=8.0,
                       help="discard points further than this from the camera. "
                            "The ZED X reaches 35 m, but depth error grows as "
                            "the square of range - the datasheet bounds it at "
                            "under 0.4%% out to 2 m and under 7%% at 20 m - so "
                            "far points add noise, not map. (default: 8.0)")
    p_exp.add_argument("--output", default=None, help="output basename")

    p_grid = sub.add_parser("save-grid",
                            help="save the LIVE 2D occupancy grid to pgm+yaml")
    p_grid.add_argument("--name", required=True)
    p_grid.add_argument("--topic", default="/rtabmap/grid_map")
    p_grid.add_argument("--timeout", type=float, default=60.0)

    p_tl = sub.add_parser("timelapse",
                          help="repeatedly save the live 2D grid while driving")
    p_tl.add_argument("--name", required=True,
                      help="run name; snapshots go under "
                           "logs/sidewalk_slam/maps/timelapse/<name>/")
    p_tl.add_argument("--topic", default="/rtabmap/grid_map")
    p_tl.add_argument("--interval", type=float, default=1.0,
                      help="seconds between snapshots (default: 1.0)")
    p_tl.add_argument("--duration", type=float, default=None,
                      help="stop automatically after this many seconds "
                           "(default: run until Ctrl+C)")
    p_tl.add_argument("--timeout", type=float, default=10.0,
                      help="per-snapshot map_saver timeout (default: 10.0)")

    p_tlv = sub.add_parser("timelapse-video",
                           help="stitch a timelapse run's snapshots into an .avi")
    p_tlv.add_argument("--name", required=True,
                       help="the --name a `timelapse` run was recorded under")
    p_tlv.add_argument("--fps", type=int, default=5,
                       help="playback speed of the output video (default: 5)")
    p_tlv.add_argument("--output", default=None,
                       help="output .avi path (default: "
                            "logs/sidewalk_slam/maps/<name>_timelapse.avi)")

    p_prep = sub.add_parser("prepare",
                            help="make a protected localisation copy of a map")
    p_prep.add_argument("--db", required=True)
    p_prep.add_argument("--name", required=True)
    p_prep.add_argument("--force", action="store_true")

    return ap.parse_args(argv)


def human(nbytes):
    for unit in ("B", "KB", "MB", "GB", "TB"):
        if nbytes < 1024 or unit == "TB":
            return f"{nbytes:.1f} {unit}"
        nbytes /= 1024.0


def require_tool(log, name):
    core = probe_rtabmap_core()
    path = core["tools"].get(name)
    if not path:
        die(log, f"`{name}` is not on PATH.",
            "It is part of the standalone RTAB-Map install. On the Jetson the "
            "core library lives in /usr/local, so its tools should be in "
            "/usr/local/bin - check that /usr/local/bin is on your PATH, and "
            "that /usr/local/lib is on LD_LIBRARY_PATH (or in "
            "/etc/ld.so.conf.d/). This is a packaging problem, not a map "
            "problem.")
    return path


def map_search_dirs():
    """Everywhere a map might reasonably be.

    Two locations exist on purpose. The launch files default to ~/rtabmap_maps
    because a launch argument cannot contain a space and the repository path
    does ("vslam-sidewalk-robot"). slam_runner.py, which quotes its arguments,
    writes into the repository so that a comparison run keeps its map beside its
    trajectories. Both are searched so neither gets forgotten.
    """
    return [map_dir(), Path.home() / "rtabmap_maps", Path.home() / ".ros"]


def cmd_list(log, args):
    dbs = []
    for d in map_search_dirs():
        if d.is_dir():
            dbs.extend(d.glob("*.db"))
    dbs = sorted(set(dbs), key=lambda p: p.stat().st_mtime, reverse=True)
    log.section("RTAB-Map databases")
    for d in map_search_dirs():
        log.info(f"  searched: {d}{'' if d.is_dir() else '  (does not exist)'}")
    if not dbs:
        log.warn("no RTAB-Map databases found yet.")
        log.info("Databases appear after a mapping run. If you have been "
                 "mapping and this is empty, the launch wrote somewhere else - "
                 "pass database_path:= explicitly and note where it went.")
        log.summary("No maps found", status="WARN")
        return 0
    total = 0
    for db in dbs:
        st = db.stat()
        total += st.st_size
        when = datetime.fromtimestamp(st.st_mtime).strftime("%Y-%m-%d %H:%M")
        log.info(f"  {db.name:46s} {human(st.st_size):>10s}  {when}")
        log.info(f"      in: {db.parent}")
        side = db.with_suffix(".json")
        if side.exists():
            try:
                meta = json.loads(side.read_text())
                log.info(f"      from: {meta.get('source_database', '?')}")
            except (OSError, ValueError):
                pass
    log.metric("map_count", len(dbs))
    log.metric("map_total_size", round(total / (1024 ** 2), 1), "MB")
    log.summary(f"{len(dbs)} map database(s), {human(total)} total", status="OK")
    return 0


def cmd_info(log, args):
    db = Path(args.db).expanduser()
    if not db.exists():
        die(log, f"database does not exist: {db}",
            "Run `map_manager.py list` to see what is available.")
    tool = require_tool(log, "rtabmap-info")
    log.section(f"Database {db.name} ({human(db.stat().st_size)})")
    rc, out, err = run_cmd([tool, str(db)], timeout=180)
    if rc != 0:
        die(log, f"rtabmap-info failed (exit {rc}): {(err or out).strip()[:600]}",
            "If the message mentions a database version, the .db was written "
            "by a different RTAB-Map release than the one installed. That is "
            "exactly the mismatch build_rtabmap_ros.py exists to prevent - "
            "check for a second rtabmap installation.")
    for line in out.splitlines():
        if line.strip():
            log.info("  " + line.rstrip())

    # Pull out the numbers that decide whether a map is worth keeping.
    low = out.lower()
    for key, label in (("loop closures", "loop_closures"),
                       ("nodes", "nodes"),
                       ("words", "visual_words")):
        for line in out.splitlines():
            if key in line.lower() and ":" in line:
                val = line.split(":", 1)[1].strip().split()[0]
                try:
                    log.metric(label, float(val.replace(",", "")))
                except ValueError:
                    pass
                break
    if "loop closure" in low and " 0 " in low:
        log.warn("This map may contain zero loop closures. A map with no loop "
                 "closures has had no drift removed from it: it is visual "
                 "odometry stored in a database. Before blaming the algorithm, "
                 "check that the route actually revisited a place from a "
                 "similar viewpoint - RTAB-Map recognises places by appearance, "
                 "and the same corridor entered from the opposite end can look "
                 "like a different corridor.")
    log.summary(f"Inspected {db.name}", status="OK")
    return 0


def cmd_export(log, args):
    db = Path(args.db).expanduser()
    if not db.exists():
        die(log, f"database does not exist: {db}",
            "Run `map_manager.py list` to see what is available.")
    tool = require_tool(log, "rtabmap-export")

    out_dir = db.parent / f"{db.stem}_export"
    out_dir.mkdir(parents=True, exist_ok=True)
    base = args.output or db.stem

    cmd = [tool,
           "--voxel", str(args.voxel),
           "--max_range", str(args.max_range),
           "--output_dir", str(out_dir),
           "--output", base,
           str(db)]
    log.section("Exporting 3D cloud")
    log.info(" ".join(cmd))
    log.info("This re-assembles every node's point cloud into one map and can "
             "take several minutes on a long run.")
    rc, out, err = run_cmd(cmd, timeout=1800)
    if out.strip():
        log.info(out.strip()[-4000:])
    if rc != 0:
        die(log, f"rtabmap-export failed (exit {rc}): {(err or out).strip()[:600]}",
            "Run `rtabmap-export --help` and compare the option names: the CLI "
            "flags changed between RTAB-Map releases, and this command is "
            "written for the 0.21 line. Do not work around it by exporting "
            "from a different rtabmap binary - that is how two versions end up "
            "installed.")

    produced = sorted(out_dir.glob(f"{base}*"))
    if not produced:
        die(log, "rtabmap-export reported success but wrote no files.",
            "The database probably contains no depth data. If the mapping run "
            "used Mem/BinDataKept=false, the raw clouds were discarded and "
            "only the pose graph survives - the 3D map cannot be rebuilt.")
    total = 0
    for f in produced:
        total += f.stat().st_size
        log.info(f"  {f.name}  {human(f.stat().st_size)}")
    log.metric("exported_files", len(produced))
    log.metric("exported_size", round(total / (1024 ** 2), 2), "MB")
    log.summary(f"Exported {len(produced)} file(s) from {db.name} to "
                f"{out_dir.name}", status="OK")
    return 0


def cmd_save_grid(log, args):
    """Save the live occupancy grid using map_server's map_saver."""
    ros = probe_ros()
    if not ros["present"]:
        die(log, "save-grid needs a running ROS system.",
            f"{ros['detail']}. This reads a live topic, so RTAB-Map must be "
            f"running and publishing {args.topic} at the moment you run it.")
    if not shutil.which("rosrun"):
        die(log, "rosrun is not on PATH.",
            "Source /opt/ros/noetic/setup.bash.")

    out_base = map_dir(create=True) / args.name
    log.section("Saving the 2D occupancy grid")
    log.info(f"topic: {args.topic}")
    log.info(f"output: {out_base}.pgm + {out_base}.yaml")

    rc, out, err = run_cmd(
        ["bash", "-lc",
         f'rosrun map_server map_saver -f "{out_base}" map:={args.topic}'],
        timeout=args.timeout)
    if out.strip():
        log.info(out.strip()[-2000:])
    if rc != 0:
        die(log, f"map_saver failed (exit {rc}): {(err or out).strip()[:500]}",
            f"map_saver waits forever for one message on {args.topic}. If it "
            f"timed out, RTAB-Map is not publishing a grid - check "
            f"`rostopic hz {args.topic}`. An empty grid usually means the "
            f"Grid/ parameters rejected every point: with "
            f"Grid/MaxObstacleHeight or Grid/MaxGroundAngle set too tightly, "
            f"the whole floor gets classified as neither ground nor obstacle.")

    pgm, yml = Path(f"{out_base}.pgm"), Path(f"{out_base}.yaml")
    if not pgm.exists() or not yml.exists():
        die(log, "map_saver exited cleanly but the files are not there.",
            f"Check write permissions on {map_dir()}.")
    log.metric("grid_pgm_size", round(pgm.stat().st_size / 1024.0, 1), "KB")
    log.info(f"wrote {pgm.name} and {yml.name}")
    log.info("This pair is what move_base's map_server loads in Phase 5-6. The "
             ".yaml records the resolution and origin; if you move the .pgm, "
             "move the .yaml with it - the .yaml refers to the .pgm by relative "
             "path.")
    log.summary(f"Saved 2D occupancy grid '{args.name}'", status="OK")
    return 0


def cmd_timelapse(log, args):
    """Repeatedly save the live 2D grid so the mapping process can be replayed.

    `save-grid` above answers "what is the map right now" - this answers "how
    did the map get built". Same underlying mechanism (map_saver on
    Grid/Sensor's live topic), just called on a timer into its own numbered
    sequence instead of once under a name you choose. `timelapse-video` turns
    that sequence into an .avi afterwards.

    KEEP-ALIVE SUBSCRIBER (measured 2026-08-22, see SOLVED.md): map_saver
    connects, grabs one message, and disconnects within a few hundred ms -
    it is not a standing subscriber. Measured live: with only that
    connect/grab/disconnect cycle touching the topic, RTAB-Map's grid
    publisher went silent after its first (near-empty) publish and every
    snapshot for the rest of a multi-minute run came back byte-identical.
    Adding one persistent, do-nothing rospy.Subscriber for the whole
    recording (this function, not a separate process) was the only change
    between a run that froze at frame 0 and one that visibly grew - so this
    node keeps that subscription open the entire time, purely so RTAB-Map
    always sees at least one live listener on the topic.

    Run this in its own terminal while driving in another one - it saves in
    place until you stop it with Ctrl+C, or until --duration runs out.
    """
    import threading
    import time

    ros = probe_ros()
    if not ros["present"]:
        die(log, "timelapse needs a running ROS system.",
            f"{ros['detail']}. This reads a live topic, so RTAB-Map must be "
            f"running and publishing {args.topic} at the moment you run it.")
    if not shutil.which("rosrun"):
        die(log, "rosrun is not on PATH.", "Source /opt/ros/noetic/setup.bash.")

    import rospy
    from nav_msgs.msg import OccupancyGrid

    out_dir = map_dir(create=True) / "timelapse" / args.name
    out_dir.mkdir(parents=True, exist_ok=True)

    log.section("Recording a mapping timelapse")
    log.info(f"topic:     {args.topic}")
    log.info(f"output:    {out_dir}/")
    log.info(f"interval:  every {args.interval:.1f}s")
    log.info(f"duration:  {'until Ctrl+C' if args.duration is None else f'{args.duration:.0f}s'}")
    log.info("")

    keepalive_count = [0]

    def _keepalive_cb(_msg):
        keepalive_count[0] += 1

    rospy.init_node(f"map_timelapse_{args.name}", anonymous=True,
                     disable_signals=True)
    keepalive_sub = rospy.Subscriber(args.topic, OccupancyGrid, _keepalive_cb,
                                     queue_size=5)
    spin_thread = threading.Thread(target=rospy.spin, daemon=True)
    spin_thread.start()

    start = time.monotonic()
    saved = 0
    failed = 0
    try:
        while args.duration is None or (time.monotonic() - start) < args.duration:
            elapsed = time.monotonic() - start
            frame_base = out_dir / f"frame_{saved:06d}_{datetime.now().strftime('%H%M%S')}"
            rc, out, err = run_cmd(
                ["bash", "-lc",
                 f'rosrun map_server map_saver -f "{frame_base}" map:={args.topic}'],
                timeout=args.timeout)
            if rc == 0 and Path(f"{frame_base}.pgm").exists():
                saved += 1
                log.info(f"[{elapsed:6.1f}s] snapshot {saved} saved "
                         f"(topic has published {keepalive_count[0]}x)")
            else:
                failed += 1
                log.info(f"[{elapsed:6.1f}s] snapshot failed (exit {rc}): "
                         f"{(err or out).strip()[:200]}")
            time.sleep(args.interval)
    except KeyboardInterrupt:
        log.info("")
        log.info("stopped by Ctrl+C")
    finally:
        keepalive_sub.unregister()
        rospy.signal_shutdown("timelapse done")

    log.info("")
    if saved == 0:
        die(log, "no snapshots were saved.",
            f"map_saver never succeeded against {args.topic} - check "
            f"`rostopic hz {args.topic}` in another terminal to confirm RTAB-Map "
            f"is actually publishing the grid.")
    if failed:
        log.info(f"({failed} snapshot(s) failed and were skipped - usually a "
                 f"transient map_saver timeout, harmless if most succeeded)")
    if keepalive_count[0] <= 1:
        log.warn(f"the keep-alive subscriber only saw {keepalive_count[0]} "
                 f"publish(es) on {args.topic} across the whole recording. "
                 f"Every saved frame is almost certainly identical - RTAB-Map "
                 f"itself went silent on this topic, not this tool. Check "
                 f"`rostopic hz {args.topic}` live before trusting the video.")
    log.metric("grid_map_publishes_seen", keepalive_count[0])
    log.info(f"Next: rosrun sidewalk_slam map_manager.py timelapse-video "
             f"--name {args.name}")
    log.summary(f"Recorded {saved} snapshot(s) for timelapse '{args.name}' "
                f"({keepalive_count[0]} real grid publishes observed)",
                status="OK")
    return 0


def cmd_timelapse_video(log, args):
    """Stitch a `timelapse` run's pgm snapshots into an .avi.

    RTAB-Map's grid grows in pixel size as new area is explored, so later
    snapshots are literally larger images than early ones. Every frame is
    padded (white = free space, matching the pgm convention) onto a shared
    canvas the size of the largest snapshot before ffmpeg sees any of them,
    or they would not concatenate into one video at all.
    """
    try:
        from PIL import Image
        import numpy as np
    except ImportError as exc:
        die(log, f"missing a required Python package: {exc}",
            "This subcommand needs Pillow and numpy: "
            "pip3 install --user pillow numpy")
    if not shutil.which("ffmpeg"):
        die(log, "ffmpeg is not on PATH.",
            "It is used elsewhere in this project already; check it is "
            "installed (`sudo apt install ffmpeg`) and on PATH.")

    run_dir = map_dir() / "timelapse" / args.name
    pgm_files = sorted(run_dir.glob("*.pgm"))
    if not pgm_files:
        die(log, f"no .pgm snapshots found in {run_dir}",
            f"Run `map_manager.py timelapse --name {args.name}` first.")

    output = Path(args.output) if args.output else map_dir(create=True) / f"{args.name}_timelapse.avi"

    log.section("Building the timelapse video")
    log.info(f"input:  {run_dir}/ ({len(pgm_files)} frames)")
    log.info(f"output: {output}")
    log.info(f"fps:    {args.fps}")

    max_w = max_h = 0
    for f in pgm_files:
        with Image.open(f) as img:
            w, h = img.size
        max_w, max_h = max(max_w, w), max(max_h, h)
    log.info(f"canvas: {max_w}x{max_h} (largest snapshot; earlier, smaller "
             f"snapshots are centred and padded white to match)")

    import tempfile
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        list_file = tmp / "frames.txt"
        with list_file.open("w") as f:
            for i, pgm in enumerate(pgm_files):
                with Image.open(pgm) as img:
                    arr = np.array(img)
                padded = np.full((max_h, max_w), 255, dtype=np.uint8)
                h, w = arr.shape
                y0, x0 = (max_h - h) // 2, (max_w - w) // 2
                padded[y0:y0 + h, x0:x0 + w] = arr
                padded_path = tmp / f"padded_{i:06d}.pgm"
                Image.fromarray(padded).save(padded_path)
                f.write(f"file '{padded_path}'\nduration {1.0 / args.fps}\n")

        rc, out, err = run_cmd(
            ["ffmpeg", "-y", "-f", "concat", "-safe", "0", "-i", str(list_file),
             "-c:v", "mpeg4", "-r", str(args.fps), "-q:v", "2", str(output)],
            timeout=120)
        if rc != 0 or not output.exists():
            die(log, f"ffmpeg failed (exit {rc}): {(err or out).strip()[:500]}")

    log.metric("timelapse_video_size", round(output.stat().st_size / 1024.0, 1), "KB")
    log.summary(f"Built timelapse video '{output.name}'", status="OK")
    return 0


def cmd_prepare(log, args):
    """Copy a mapping database to a protected localisation copy.

    Localisation mode opens the database read-write even though it does not add
    nodes, and a mistaken launch argument can still wipe it. Working from a copy
    means the mapping session's result is never the thing at risk.
    """
    src = Path(args.db).expanduser()
    if not src.exists():
        die(log, f"database does not exist: {src}",
            "Run `map_manager.py list` to see what is available.")
    dst = map_dir(create=True) / f"{args.name}_localization.db"
    if dst.exists() and not args.force:
        die(log, f"{dst} already exists.",
            "Pass --force to overwrite it, or choose another --name. Refusing "
            "silently to overwrite a map is deliberate: maps take a long time "
            "to make.")

    log.section("Preparing a localisation map")
    log.info(f"source: {src} ({human(src.stat().st_size)})")
    shutil.copy2(src, dst)
    log.info(f"copy:   {dst}")

    meta = {
        "source_database": str(src),
        "prepared_at": datetime.now().isoformat(timespec="seconds"),
        "purpose": "localisation-only copy; safe to lose",
        "note": ("Launch with rtabmap_localization.launch, which sets "
                 "Mem/IncrementalMemory=false so no new nodes are added. "
                 "Never pass --delete_db_on_start with this file."),
    }
    dst.with_suffix(".json").write_text(json.dumps(meta, indent=2))

    if dst.stat().st_size != src.stat().st_size:
        die(log, "the copy is a different size from the original.",
            "The copy was interrupted or the disk is full. Check `df -h`.")

    log.metric("localization_map_size", round(dst.stat().st_size / (1024 ** 2), 1),
               "MB")
    log.info("")
    log.info("To localise against it:")
    log.info(f"  roslaunch sidewalk_slam rtabmap_localization.launch "
             f"database_path:={dst}")
    log.summary(f"Prepared localisation map '{args.name}'", status="OK")
    return 0


def main(argv=None):
    args = parse_args(argv)
    with make_logger(f"map_{args.cmd}") as log:
        return {
            "list": cmd_list,
            "info": cmd_info,
            "export": cmd_export,
            "save-grid": cmd_save_grid,
            "timelapse": cmd_timelapse,
            "timelapse-video": cmd_timelapse_video,
            "prepare": cmd_prepare,
        }[args.cmd](log, args)


if __name__ == "__main__":
    # SlamExit is how every tool in this package reports an already-diagnosed
    # refusal. The reason has been printed and logged by the time it reaches
    # here; all that is left is to hand the shell a non-zero status.
    try:
        sys.exit(main())
    except SlamExit as _exit:
        sys.exit(_exit.code)
