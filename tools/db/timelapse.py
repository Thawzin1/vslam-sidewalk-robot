#!/usr/bin/env python3
"""timelapse.py - a drive's floor map growing node by node, as video frames.

WHY
    ENGINEERING_NOTES.md rule 22: every drive gets a results pack, and a
    timelapse is part of it. From drive 4 on, the timelapse is
    recorded DURING the drive. Drives 1-3 had nothing recording the live
    picture, so for them this RECONSTRUCTS one after the drive from the saved
    map database. Every frame says so in its footer.

    *Plain terms: the map database keeps, for every snapshot the mapper took
    (a "node"), the time it was taken, where the robot was, and the small patch
    of floor it saw. Replaying those patches in time order redraws the map as
    it grew.*

TWO STEPS, SO THE HEAVY READ HAPPENS ONCE, WHERE THE DATABASE LIVES
    extract   reads the database (several GB - it streams, row by row) and
              writes a small .npz: per node its time, position, and the map
              cells it saw, already moved into the map frame.
    render    reads only that .npz and writes PNG frames (no database needed),
              so drives whose database is on another computer can be rendered on the
              Jetson, where ffmpeg is.

WHICH POSITIONS
    The same rule as render_map.py (it imports that tool's functions, so the
    last frame IS map_corrected.png's map): with --poses <camera_corrected.tum>
    every node is placed at the map's final corrected position (after all loop
    closures - a loop closure is the map recognising a place it has seen before
    and pulling itself straight). Nodes outside the saved graph take the
    correction of the nearest earlier graph node. Without --poses, the camera's
    tracking alone. The frames name which one they show.

    The live view did NOT look like this while driving: it drew each node where
    the map believed it was AT THAT MOMENT, and shifted as corrections arrived.

TIME
    Frames are spaced evenly in drive time, except that a stretch where the
    robot stood still for more than --idle-s seconds is squeezed to 2 s of
    drive time and captioned ("parked - 6 min skipped"). The on-screen clock
    always shows the real drive time, so the skip is visible.

  usage:
    timelapse.py extract <db> <out.npz> [--poses corrected.tum] [--run NAME]
                         [--progress FILE]
    timelapse.py render <cells.npz> <frames_dir> --title "Drive 3" [--seconds 40]
                        [--fps 25] [--events events.json] [--progress FILE]
                        [--stills DIR]

  events.json (optional): [[start_epoch_s, end_epoch_s, "caption", "rust"|"teal"], ...]
"""
from __future__ import print_function

import argparse
import json
import math
import os
import sqlite3
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from render_map import (apply_corrections, decode_cells, pose_matrix,  # noqa: E402
                        read_corrected, yaw_of)

TEAL = (31, 138, 140)
RUST = (192, 83, 43)
INK = (34, 37, 42)
MUTED = (110, 112, 116)
GREY_UNKNOWN = 184       # 0.72 * 255, the same grey render_map.py uses
SOLID = 20               # 0.08 * 255
FONT_DIR = "/usr/share/fonts/truetype/dejavu/"


def hamilton(epoch, fmt):
    os.environ["TZ"] = "America/Toronto"
    time.tzset()
    return time.strftime(fmt, time.localtime(epoch))


def write_progress(path, line):
    if not path:
        return
    tmp = path + ".tmp"
    with open(tmp, "w") as fh:
        fh.write(line + "\n")
    os.replace(tmp, path)


# ---------------------------------------------------------------- extract
def extract(args):
    t_start = time.time()
    con = sqlite3.connect("file:%s?immutable=1" % args.db, uri=True)
    poses, stamps = {}, {}
    for nid, stamp, blob in con.execute("SELECT id, stamp, pose FROM Node ORDER BY id"):
        if blob is None or len(bytes(blob)) < 48 or nid in poses:
            continue
        poses[nid] = pose_matrix(blob)
        stamps[nid] = stamp
    graph = set()
    kind = "odometry"
    if args.poses:
        corr = read_corrected(args.poses)
        graph = set(corr)
        poses, _ = apply_corrections(poses, corr)
        kind = "corrected"
    links = con.execute("SELECT from_id, to_id, type FROM Link WHERE type IN (1,2,3,4) "
                        "AND from_id > to_id").fetchall()

    ids = sorted(poses)
    index = {nid: k for k, nid in enumerate(ids)}
    cells = {"free": [[None] * len(ids), [None] * len(ids)],
             "occ": [[None] * len(ids), [None] * len(ids)]}
    cell = None
    done = 0
    last_write = 0.0
    total = len(ids)
    for nid, g, o, e, cs in con.execute(
            "SELECT id, ground_cells, obstacle_cells, empty_cells, cell_size "
            "FROM Data ORDER BY id"):
        if nid not in index:
            continue
        k = index[nid]
        if cells["free"][0][k] is not None:      # a damaged tree can revisit a row
            continue
        cell = cell or (cs if cs else 0.05)
        R, t = poses[nid]
        for name, blobs in (("free", (g, e)), ("occ", (o,))):
            parts = []
            for blob in blobs:
                pts = decode_cells(blob)
                if pts is None or len(pts) == 0:
                    continue
                w = pts @ R.T + t
                parts.append(np.floor(w[:, :2] / cell).astype(np.int32))
            if parts:
                ij = np.concatenate(parts)
                keys, counts = np.unique(ij, axis=0, return_counts=True)
            else:
                keys = np.zeros((0, 2), np.int32)
                counts = np.zeros(0, np.int64)
            cells[name][0][k] = keys.astype(np.int32)
            cells[name][1][k] = counts.astype(np.int32)
        done += 1
        now = time.time()
        if now - last_write > 10:
            write_progress(args.progress, "TIMELAPSE_EXTRACT %s %d/%d  %ds"
                           % (args.run, done, total, now - t_start))
            last_write = now
    con.close()

    out = {}
    for name in ("free", "occ"):
        keys, counts = cells[name]
        keys = [x if x is not None else np.zeros((0, 2), np.int32) for x in keys]
        counts = [x if x is not None else np.zeros(0, np.int32) for x in counts]
        off = np.zeros(len(ids) + 1, np.int64)
        off[1:] = np.cumsum([len(x) for x in counts])
        out[name + "_off"] = off
        out[name + "_ij"] = np.concatenate(keys) if keys else np.zeros((0, 2), np.int32)
        out[name + "_n"] = np.concatenate(counts) if counts else np.zeros(0, np.int32)
    xy = np.array([[poses[i][1][0], poses[i][1][1]] for i in ids], np.float64)
    yaw = np.array([yaw_of(poses[i][0]) for i in ids], np.float64)
    lk = np.array(links, np.int64).reshape(-1, 3)
    meta = {"database": os.path.abspath(args.db), "run": args.run, "positions": kind,
            "poses_file": os.path.abspath(args.poses) if args.poses else None,
            "nodes": len(ids), "nodes_with_cells": done, "graph_nodes": len(graph),
            "closures": int(len(lk)), "tool": "tools/db/timelapse.py extract"}
    np.savez_compressed(args.out, ids=np.array(ids, np.int64),
                        stamps=np.array([stamps[i] for i in ids], np.float64),
                        xy=xy, yaw=yaw, in_graph=np.array([i in graph for i in ids]),
                        links=lk, cell=float(cell or 0.05), meta=json.dumps(meta), **out)
    write_progress(args.progress, "TIMELAPSE_EXTRACT %s done %d/%d  %ds"
                   % (args.run, done, total, time.time() - t_start))
    print(json.dumps(meta, indent=2))
    return 0


# ----------------------------------------------------------------- render
def load_font(size, bold=False):
    from PIL import ImageFont
    name = "DejaVuSerif-Bold.ttf" if bold else "DejaVuSerif.ttf"
    try:
        return ImageFont.truetype(os.path.join(FONT_DIR, name), size)
    except IOError:
        return ImageFont.load_default()


def text_w(font, s):
    if hasattr(font, "getbbox"):
        b = font.getbbox(s)
        return b[2] - b[0]
    return font.getsize(s)[0]


def wrap(font, s, width):
    words, lines, cur = s.split(), [], ""
    for w in words:
        trial = (cur + " " + w).strip()
        if text_w(font, trial) <= width or not cur:
            cur = trial
        else:
            lines.append(cur)
            cur = w
    if cur:
        lines.append(cur)
    return lines


def virtual_timeline(stamps, xy, yaw, idle_s):
    """Map real node times to 'virtual' times with long still stretches squeezed.
    Returns (virtual time per node, list of skipped (start, end) real intervals)."""
    n = len(stamps)
    vt = np.zeros(n)
    skipped = []
    moving = np.ones(n, bool)
    if n > 1:
        d = np.linalg.norm(np.diff(xy, axis=0), axis=1)
        dy = np.abs(np.angle(np.exp(1j * np.diff(yaw))))
        moving[1:] = (d > 0.03) | (dy > math.radians(2.0))
    k = 1
    while k < n:
        if moving[k]:
            vt[k] = vt[k - 1] + (stamps[k] - stamps[k - 1])
            k += 1
            continue
        j = k
        while j < n and not moving[j]:
            j += 1
        # still from node k-1 to node j-1
        span = stamps[j - 1] - stamps[k - 1]
        scale = 1.0 if span <= idle_s else 2.0 / span
        if span > idle_s:
            skipped.append((stamps[k - 1], stamps[j - 1]))
        for m in range(k, j):
            vt[m] = vt[m - 1] + (stamps[m] - stamps[m - 1]) * scale
        k = j
    return vt, skipped


def render(args):
    from PIL import Image, ImageDraw, ImageFilter  # noqa: F401
    t_start = time.time()
    z = np.load(args.npz, allow_pickle=False)
    meta = json.loads(str(z["meta"]))
    ids, stamps, xy, yaw = z["ids"], z["stamps"], z["xy"], z["yaw"]
    cell = float(z["cell"])
    free_off, free_ij, free_n = z["free_off"], z["free_ij"], z["free_n"]
    occ_off, occ_ij, occ_n = z["occ_off"], z["occ_ij"], z["occ_n"]
    links = z["links"]
    n = len(ids)
    pos_of = {int(i): k for k, i in enumerate(ids)}

    allij = np.concatenate([free_ij, occ_ij])
    imin, jmin = allij.min(axis=0)
    imax, jmax = allij.max(axis=0)
    # keep the whole path inside the picture too
    pi = np.floor(xy / cell).astype(np.int64)
    imin, jmin = min(imin, pi[:, 0].min()) - 10, min(jmin, pi[:, 1].min()) - 10
    imax, jmax = max(imax, pi[:, 0].max()) + 10, max(jmax, pi[:, 1].max()) + 10
    W, H = int(imax - imin + 1), int(jmax - jmin + 1)
    occ = np.zeros((H, W), np.int32)
    free = np.zeros((H, W), np.int32)

    FW, FH = 1920, 1080
    box = (40, 40, 1240, 1030)                     # map panel
    bw, bh = box[2] - box[0], box[3] - box[1]
    s = min(bw / float(W), bh / float(H))
    mw, mh = max(1, int(round(W * s))), max(1, int(round(H * s)))
    ox = box[0] + (bw - mw) // 2
    oy = box[1] + (bh - mh) // 2

    def to_px(x, y):
        return (ox + (x / cell - imin) * s, oy + (jmax + 1 - y / cell) * s)

    path_px = [to_px(x, y) for x, y in xy]
    cum = np.zeros(n)
    if n > 1:
        cum[1:] = np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))

    vt, skipped = virtual_timeline(stamps, xy, yaw, args.idle_s)
    frames = int(args.seconds * args.fps)
    vtimes = np.linspace(0.0, vt[-1], frames)

    # closures: when each became known = stamp of its newer node
    cl_t, cl_kind, cl_age = [], [], []
    for f, t, ty in links:
        if int(f) in pos_of and int(t) in pos_of:
            cl_t.append(stamps[pos_of[int(f)]])
            cl_kind.append(int(ty))
            cl_age.append(stamps[pos_of[int(f)]] - stamps[pos_of[int(t)]])
    cl_t = np.array(cl_t)
    cl_kind = np.array(cl_kind)
    cl_age = np.array(cl_age)

    events = []
    if args.events:
        with open(args.events) as fh:
            events = json.load(fh)
    for a, b in skipped:
        events.append([a, b, "robot standing still - %d min %02d s skipped in this video"
                       % ((b - a) // 60, (b - a) % 60), "muted"])
    # each recognised-again closure gets a short caption
    for t, kind, age in zip(cl_t, cl_kind, cl_age):
        if kind == 1:
            events.append([t, t + 4.0, "loop closure: the map recognised a place it saw "
                           "%d min %02d s earlier and straightened itself" % (age // 60, age % 60),
                           "teal"])

    f_title = load_font(46, True)
    f_sub = load_font(25)
    f_lab = load_font(22)
    f_big = load_font(56, True)
    f_val = load_font(38, True)
    f_small = load_font(19)
    f_cap = load_font(24, True)
    t0, t1 = stamps[0], stamps[-1]
    date_line = "%s, %s-%s Hamilton time" % (
        hamilton(t0, "%d %b %Y").lstrip("0"), hamilton(t0, "%H:%M"), hamilton(t1, "%H:%M"))
    total_real = t1 - t0
    kind_text = ("positions: the map's FINAL corrected positions (after all loop closures)"
                 if meta.get("positions") == "corrected"
                 else "positions: the camera's tracking alone (never corrected)")
    footer = ("Reconstructed after the drive from the saved map database - NOT recorded live. "
              + kind_text + "; the live view shifted as corrections arrived. "
              "%d min of driving shown in %d s." % (round(total_real / 60.0), args.seconds))

    os.makedirs(args.frames, exist_ok=True)
    stills = {}
    if args.stills:
        os.makedirs(args.stills, exist_ok=True)
        for frac in (0.25, 0.5, 0.75):
            stills[int(frac * (frames - 1))] = "timelapse_%02d.png" % int(frac * 100)
        stills[frames - 1] = "timelapse_100.png"

    added = 0          # nodes whose cells are already in the grids
    last_write = 0.0
    for f in range(frames):
        v = vtimes[f]
        while added < n and vt[added] <= v + 1e-9:
            for name, grid, off, ij, cnt in (("free", free, free_off, free_ij, free_n),
                                             ("occ", occ, occ_off, occ_ij, occ_n)):
                a, b = off[added], off[added + 1]
                if b > a:
                    ii = ij[a:b, 0] - imin
                    jj = jmax - ij[a:b, 1]
                    grid[jj, ii] += cnt[a:b]
            added += 1
        cur = max(0, added - 1)
        t_real = stamps[cur] if added else t0

        img = np.full((H, W), GREY_UNKNOWN, np.uint8)
        seen = (occ + free) > 0
        img[seen] = 255
        img[occ > free] = SOLID
        m = Image.fromarray(img, "L")
        m = m.resize((mw, mh), Image.NEAREST if s >= 1.0 else Image.BOX)
        frame = Image.new("RGB", (FW, FH), (255, 255, 255))
        frame.paste(m.convert("RGB"), (ox, oy))
        d = ImageDraw.Draw(frame)

        # path so far, start mark, current position with heading
        if added > 1:
            d.line(path_px[:added], fill=TEAL, width=3)
        sx, sy = path_px[0]
        d.ellipse([sx - 9, sy - 9, sx + 9, sy + 9], fill=TEAL, outline=(255, 255, 255), width=2)
        d.text((sx + 12, sy - 30), "start", font=f_lab, fill=TEAL)
        if added:
            cx, cy = path_px[cur]
            hx = cx + 26 * math.cos(yaw[cur])
            hy = cy - 26 * math.sin(yaw[cur])
            d.line([(cx, cy), (hx, hy)], fill=INK, width=4)
            d.ellipse([cx - 10, cy - 10, cx + 10, cy + 10], fill=TEAL,
                      outline=(255, 255, 255), width=3)
        # 5 m scale bar
        bx, by = box[0] + 20, box[3] - 20
        d.line([(bx, by), (bx + 5.0 / cell * s, by)], fill=INK, width=5)
        d.text((bx, by - 32), "5 m", font=f_lab, fill=INK)

        # right-hand panel
        x0 = 1290
        d.text((x0, 50), args.title, font=f_title, fill=INK)
        yy = 112
        for line in wrap(f_sub, args.subtitle, 590):
            d.text((x0, yy), line, font=f_sub, fill=INK)
            yy += 32
        d.text((x0, yy + 4), date_line, font=f_lab, fill=MUTED)
        yy += 70
        el = t_real - t0
        d.text((x0, yy), "drive time", font=f_lab, fill=MUTED)
        d.text((x0, yy + 26), "%d:%02d" % (el // 60, el % 60), font=f_big, fill=INK)
        d.text((x0 + text_w(f_big, "%d:%02d" % (el // 60, el % 60)) + 14, yy + 52),
               "of %d:%02d" % (total_real // 60, total_real % 60), font=f_sub, fill=MUTED)
        yy += 110
        d.text((x0, yy), "path drawn so far", font=f_lab, fill=MUTED)
        d.text((x0, yy + 26), "%.1f m" % cum[cur], font=f_val, fill=INK)
        yy += 90
        known = cl_t <= t_real + 1e-6 if len(cl_t) else np.zeros(0, bool)
        n_glob = int(np.sum(known & (cl_kind == 1))) if len(cl_t) else 0
        n_near = int(np.sum(known & (cl_kind != 1))) if len(cl_t) else 0
        d.text((x0, yy), "loop closures so far (place recognised again)", font=f_lab, fill=MUTED)
        d.text((x0, yy + 26), "%d" % n_glob, font=f_val, fill=TEAL)
        d.text((x0 + 120, yy + 38), "+ %d nearby re-matches" % n_near, font=f_sub, fill=INK)
        yy += 90
        d.text((x0, yy), "map snapshots (nodes)", font=f_lab, fill=MUTED)
        d.text((x0, yy + 26), "%d of %d" % (added, n), font=f_val, fill=INK)
        yy += 100

        # captions active now
        caps = [e for e in events if e[0] - 1e-6 <= t_real <= e[1] + 1e-6]
        for e in caps[:2]:
            col = {"rust": RUST, "teal": TEAL}.get(e[3] if len(e) > 3 else "rust", MUTED)
            for line in wrap(f_cap, e[2], 590):
                d.text((x0, yy), line, font=f_cap, fill=col)
                yy += 30
            yy += 10

        # legend
        ly = 850
        for sw, label in (((255, 255, 255), "floor seen, free"),
                          ((SOLID,) * 3, "solid: walls, furniture"),
                          ((GREY_UNKNOWN,) * 3, "never seen")):
            d.rectangle([x0, ly, x0 + 26, ly + 22], fill=sw, outline=INK)
            d.text((x0 + 38, ly - 2), label, font=f_small, fill=INK)
            ly += 32
        d.line([(x0, ly + 11), (x0 + 26, ly + 11)], fill=TEAL, width=4)
        d.text((x0 + 38, ly - 2), "robot's path (camera map)", font=f_small, fill=INK)

        # progress bar through the video
        py = 1000
        d.rectangle([x0, py, x0 + 590, py + 10], fill=(232, 230, 226))
        d.rectangle([x0, py, x0 + int(590 * f / max(1, frames - 1)), py + 10], fill=TEAL)
        fy = 1044
        for line in wrap(f_small, footer, 1840)[:2]:
            d.text((40, fy), line, font=f_small, fill=MUTED)
            fy += 22

        name = os.path.join(args.frames, "f_%05d.png" % f)
        frame.save(name, compress_level=3)
        if f in stills:
            frame.save(os.path.join(args.stills, stills[f]))
        now = time.time()
        if now - last_write > 10 or f == frames - 1:
            write_progress(args.progress, "TIMELAPSE_RENDER %s %d/%d  %ds"
                           % (meta.get("run", ""), f + 1, frames, now - t_start))
            last_write = now
    info = {"frames": frames, "fps": args.fps, "seconds": args.seconds,
            "real_drive_s": round(total_real, 1), "skipped_still": [
                [round(a - t0, 1), round(b - t0, 1)] for a, b in skipped],
            "map_cells": [W, H], "px_per_cell": round(s, 3), "nodes": n,
            "positions": meta.get("positions"), "source": meta}
    with open(os.path.join(args.frames, "timelapse_info.json"), "w") as fh:
        json.dump(info, fh, indent=2)
    print(json.dumps({k: v for k, v in info.items() if k != "source"}))
    return 0


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd")
    e = sub.add_parser("extract")
    e.add_argument("db")
    e.add_argument("out")
    e.add_argument("--poses", default="")
    e.add_argument("--run", default="")
    e.add_argument("--progress", default="")
    r = sub.add_parser("render")
    r.add_argument("npz")
    r.add_argument("frames")
    r.add_argument("--title", default="")
    r.add_argument("--subtitle", default="camera map building up")
    r.add_argument("--seconds", type=int, default=40)
    r.add_argument("--fps", type=int, default=25)
    r.add_argument("--idle-s", type=float, default=30.0)
    r.add_argument("--events", default="")
    r.add_argument("--progress", default="")
    r.add_argument("--stills", default="")
    args = ap.parse_args()
    if args.cmd == "extract":
        return extract(args)
    if args.cmd == "render":
        return render(args)
    ap.print_help()
    return 2


if __name__ == "__main__":
    sys.exit(main())
