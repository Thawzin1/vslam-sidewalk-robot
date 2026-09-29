#!/usr/bin/env python3
"""localise_watch.py - the live "LOCALISED: yes/no" line for the localisation demo.

Runs on the Jetson beside RTAB-Map in localisation mode (start_drive.sh LOCALISE=1 starts it).

WHAT "LOCALISED" MEANS HERE (and what it does not)
  RTAB-Map in localisation mode starts by ASSUMING the robot is where the map last left it
  (drive 4 ended on the start mark: Admin.opt_last_localization, Rtabmap.cpp:1345-1362). So the
  robot's position on the map looks right at the start mark before anything has been recognised,
  and looks confidently wrong anywhere else. The position alone therefore proves nothing.
  "Localised" is counted from a recognition event on /rtabmap/info:
    - loopClosureId > 0        = "recognised": the current picture matched a saved picture by
                                  appearance anywhere in the map, and the 3D check passed
    - proximityDetectionId > 0 = "nearby re-match": matched a saved picture NEAR where the robot
                                  is currently believed to be (only trustworthy once the belief is right)
  *Plain terms: the robot has found itself only when it says "I have seen this before" and the
  geometry agrees - not merely because the dot on the map happens to sit in the right place.*

  ... AND, since session 1 (26 Sept 2026, see session1_diag/), from two lasting traces of an
  ACCEPTED fix, because the event itself is a single message that is easily missed:
    - "uncertainty": /rtabmap/localization_pose carries RTAB-Map's localisation uncertainty; it is
      9999 until a fix has been accepted (CoreWrapper.cpp:2102-2129, 0.21.13) and small after
    - "cache link": /rtabmap/info.odom_cache (the localiser's short list of its recent positions)
      holds a recognition or nearby re-match link to a node of the saved map
  Session 1 localised on its 2nd picture (05:03:24-25), before start 1 was written (05:03:26) and
  before this watcher was running; after that a parked robot never re-checks (movement under
  RGBD/LinearUpdate 0.05 m / AngularUpdate 0.05 rad -> recognition skipped, Rtabmap.cpp:1969), so
  no further event came and the old watcher said "not localised" for 7 min.
  START 1 is back-dated: evidence from any time counts for it (nothing comes before it). For a
  later start, evidence counts only if it ARRIVES more than RELOAD_GRACE_S after that start's
  line (the reload clears RTAB-Map's uncertainty and cache, so older messages belong to the
  previous start).

CHANGED AFTER SESSION 2 (26 Sept 2026, session2_diag/): A "CACHE LINK" IS NOT A FIX.
  Session 2 showed that a single recognition link in the odometry cache is a PENDING localisation:
  RTAB-Map holds it ("Localization was good, but waiting for another one", Rtabmap.cpp:3759) and does
  NOT move the robot. Its published uncertainty stays 9999 and map->odom keeps the reload's guess
  (the start mark). The old watcher counted that link as the fix and printed the guessed position, so
  it said "LOCALISED yes" at starts 2, 3 and 5 while the map showed the robot 4-11 m from where it was.
  Now:
    - a FIX = RTAB-Map ACCEPTED a localisation: /rtabmap/localization_pose uncertainty below 9999.
      A recognition or nearby re-match event counts only if it is accepted (checked 0.3 s later).
    - a cache link is logged as "pending" (recognised, not yet accepted) and shown on the page.
    - with --marks, each start's TAPED spot (its expected place on the map) is known. The page and the
      track show the distance from the map's position to the EXPECTED spot = the tape moved on by the
      robot's odometry since the start (so driving on from the spot is not "wrong"). An accepted fix
      more than 1.0 m from it is shown as "WRONG PLACE?" (the LiDAR check afterwards settles it).
  *Plain terms: the robot has found itself only when the map program itself says so, and the page now
  also says how far the dot is from the tape on the floor.*

WHAT IT WRITES
  ~/jobs/<run>_localise.progress   one line, rewritten every 2 s (rule 13; "start k/5" draws the bar)
  ~/.run_records/<run>/localise_events.csv    every recognition event (time, start, kind, pose, jump)
  ~/.run_records/<run>/localise_track.csv     1 Hz: where the map puts the robot, localised or not
  ~/.run_records/<run>/localise_summary.json  per start: first fix, time and distance to it, jumps
  Starts are read from ~/.run_records/<run>/starts.csv (written by start_drive.sh and next_start.sh).

JUMP
  Each recognition can move the robot on the map (it changes the map -> odom correction). The size
  of that move, measured at the robot's current position, is logged. The FIRST fix after a start is
  expected to move it (from the assumed start-mark position to the real one); any LATER move over
  1.0 m in the same start is flagged "suspect jump" - a possible false recognition, to be settled
  against the LiDAR estimate after the run (PASS_LINES.md P3).

  usage: localise_watch.py --run s2_loc_01 --map-db ~/slam_series2/localise/s2_static_04_map.db
"""
import argparse
import csv
import json
import math
import os
import sqlite3
import sys
import threading
import time
import zlib

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped
from rtabmap_msgs.msg import Info

JUMP_SUSPECT_M = 1.0
NOT_LOCALISED_COV = 9999.0     # CoreWrapper's "not yet localised" variance
RELOAD_GRACE_S = 1.0           # later starts: ignore evidence arriving this soon after the reload line
MAP_LINK_TYPES = {1, 2, 3, 4, 6, 8}   # global, nearby-space, nearby-time, user, merged, landmark


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def se2(x, y, th):
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s, x], [s, c, y], [0.0, 0.0, 1.0]])


def load_map_nodes(db):
    """Corrected node positions (Admin.opt_poses) of the saved map, opened READ-ONLY."""
    con = sqlite3.connect("file:%s?mode=ro" % db, uri=True)
    try:
        ids, poses = con.execute("select opt_ids, opt_poses from Admin").fetchone()
    finally:
        con.close()

    def dec(b):
        try:
            return zlib.decompress(b)
        except zlib.error:
            return b
    ids = np.frombuffer(dec(ids), dtype=np.int32)
    P = np.frombuffer(dec(poses), dtype=np.float32).reshape(-1, 3, 4)
    return ids, P[:, :2, 3].astype(float)


class Watch:
    def __init__(self, a):
        self.a = a
        self.rec = os.path.join(os.path.expanduser(os.environ.get("RECORDS_DIR", "~/.run_records")), a.run)
        os.makedirs(self.rec, exist_ok=True)
        self.prog = os.path.expanduser("~/jobs/%s_localise.progress" % a.run)
        os.makedirs(os.path.dirname(self.prog), exist_ok=True)
        self.node_ids, self.node_xy = load_map_nodes(a.map_db)
        self.t0 = time.time()
        self.buf = tf2_ros.Buffer(rospy.Duration(30.0))
        self.tfl = tf2_ros.TransformListener(self.buf)
        self.starts = []          # list of dicts from starts.csv
        self.st = {}              # per-start state, keyed by start number
        self.C_prev = None        # last map->odom correction (3x3) in this start
        self.cur = None           # current start number
        self.odom_prev = None
        self.odom_start = None
        self.lock = threading.RLock()
        self.pending = None       # evidence seen before starts.csv had a line (counts for start 1)
        self.last_info = 0.0
        self.last_stats = {}
        self.accepted = False     # latest localization_pose uncertainty below 9999 (RTAB-Map accepted a fix)
        self.pending_seen = set() # cache links already logged as pending, per start
        self.marks = {}
        if a.marks:
            with open(os.path.expanduser(a.marks)) as f:
                self.marks = {k: v for k, v in json.load(f).items() if not k.startswith("_")}
        ev = os.path.join(self.rec, "localise_events.csv")
        new = not os.path.exists(ev)
        self.evf = open(ev, "a", newline="")
        self.evw = csv.writer(self.evf)
        if new:
            self.evw.writerow(["t_epoch", "start", "mark", "kind", "loop_id", "prox_id", "ref_id",
                               "map_x", "map_y", "map_yaw_deg", "jump_m", "first_fix",
                               "suspect", "dist_to_map_path_m", "nearest_node", "inliers",
                               "odom_m_since_start", "s_since_start"])
        tr = os.path.join(self.rec, "localise_track.csv")
        new = not os.path.exists(tr)
        self.trf = open(tr, "a", newline="")
        self.trw = csv.writer(self.trf)
        if new:
            self.trw.writerow(["t_epoch", "start", "localised", "map_x", "map_y", "map_yaw_deg",
                               "odom_m_since_start", "dist_to_map_path_m", "accepted_now",
                               "dist_to_mark_m", "mark_dyaw_deg"])
        rospy.Subscriber("/rtabmap/info", Info, self.on_info, queue_size=5)
        rospy.Subscriber("/rtabmap/localization_pose", PoseWithCovarianceStamped, self.on_pose,
                         queue_size=5)

    # ---------- starts ----------
    def read_starts(self):
        p = os.path.join(self.rec, "starts.csv")
        try:
            with open(p) as f:
                rows = [r for r in csv.DictReader(f)]
        except OSError:
            return
        if len(rows) != len(self.starts):
            self.starts = rows
            k = int(rows[-1]["start"])
            if k != self.cur:
                self.cur = k
                self.C_prev = None
                self.st[k] = {"start": k, "mark": rows[-1]["mark"], "t_start": float(rows[-1]["t_epoch"]),
                              "first_fix_t": None, "first_fix_kind": None, "first_fix_odom_m": None,
                              "first_fix_s": None, "first_fix_jump_m": None, "first_fix_xy": None,
                              "fixes": 0, "global": 0, "proximity": 0, "suspect_jumps": 0,
                              "max_later_jump_m": 0.0, "odom_m": 0.0, "last_fix_t": None,
                              "pending": 0, "first_pending_s": None, "wrong_place_fixes": 0,
                              "first_fix_dist_to_mark_m": None}
                self.odom_prev = None
                self.odom_start = None    # odom pose at this start (set on the first tick after it)
                self.pending_seen = set()
                if k == 1 and self.pending is not None:
                    p, self.pending = self.pending, None
                    self.evidence(*p)

    # ---------- geometry ----------
    def lookup(self, parent, child, stamp=rospy.Time(0)):
        try:
            t = self.buf.lookup_transform(parent, child, stamp, rospy.Duration(0.3))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException):
            return None
        tr, q = t.transform.translation, t.transform.rotation
        return se2(tr.x, tr.y, yaw_of(q))

    def dist_to_path(self, x, y):
        d = np.hypot(self.node_xy[:, 0] - x, self.node_xy[:, 1] - y)
        i = int(np.argmin(d))
        return float(d[i]), int(self.node_ids[i])

    def dist_to_mark(self, x, y, yaw_deg):
        """distance (m) and heading difference (deg) from where the robot SHOULD be: the current start's taped
        spot moved on by the odometry since the start (tape (+) odom_start^-1 * odom_now). Comparing to the bare
        spot flagged start 1 as wrong once the robot drove on to A (review 26 Sept). (None, None) if unknown."""
        if self.cur is None or x is None:
            return None, None
        m = self.marks.get(self.st[self.cur]["mark"])
        if not m or self.odom_start is None or self.odom_prev is None:
            return None, None
        E = se2(m[0], m[1], math.radians(m[2])).dot(np.linalg.inv(self.odom_start)).dot(self.odom_prev)
        ey = math.degrees(math.atan2(E[1, 0], E[0, 0]))
        dy = (yaw_deg - ey + 180.0) % 360.0 - 180.0
        return math.hypot(x - E[0, 2], y - E[1, 2]), dy

    # ---------- callbacks ----------
    def on_info(self, m):
        self.last_info = time.time()
        self.last_stats = dict(zip(m.statsKeys, m.statsValues)) if m.statsKeys else {}
        if m.loopClosureId <= 0 and m.proximityDetectionId <= 0:
            try:
                cache = set(m.odom_cache.posesId)
                links = [l for l in m.odom_cache.links
                         if l.type in MAP_LINK_TYPES and l.toId > 0 and l.toId not in cache]
            except AttributeError:
                links = []
            if links:
                self.pending_link(links, m.refId)
            return
        self.read_starts()   # a start written in the last 2 s must own this event, not the previous one
        if self.cur is None:
            return
        s = self.st[self.cur]
        kind = "recognised" if m.loopClosureId > 0 else "nearby"
        # the correction this event produced: wait for rtabmap's tf (tf_delay 0.05 s) to carry it
        rospy.sleep(0.3)
        if not self.accepted:
            # held by the second-opinion rule (or rejected): the robot was NOT moved - not a fix
            self.pending_link([], m.refId, label="%s %d (not accepted)" % (
                kind, m.loopClosureId if m.loopClosureId > 0 else m.proximityDetectionId))
            return
        C = self.lookup("map", "odom")
        O = self.lookup("odom", "base_link")
        jump = None
        x = y = yaw = dist = None
        node = None
        if C is not None and O is not None:
            Mp = C.dot(O)
            x, y, yaw = Mp[0, 2], Mp[1, 2], math.degrees(math.atan2(Mp[1, 0], Mp[0, 0]))
            dist, node = self.dist_to_path(x, y)
            if self.C_prev is not None:
                old = self.C_prev.dot(O)
                jump = math.hypot(old[0, 2] - x, old[1, 2] - y)
            self.C_prev = C
        now = time.time()
        first = s["first_fix_t"] is None
        suspect = False
        dmark, _ = self.dist_to_mark(x, y, yaw)
        if dmark is not None and dmark > JUMP_SUSPECT_M and s["odom_m"] < 3.0 and now - s["t_start"] <= 120.0:
            s["wrong_place_fixes"] += 1
        if first:
            s.update(first_fix_t=now, first_fix_kind=kind, first_fix_odom_m=round(s["odom_m"], 3),
                     first_fix_s=round(now - s["t_start"], 1),
                     first_fix_jump_m=None if jump is None else round(jump, 3),
                     first_fix_xy=None if x is None else [round(x, 3), round(y, 3), round(yaw, 1)],
                     first_fix_dist_to_mark_m=None if dmark is None else round(dmark, 3))
        elif jump is not None and jump > JUMP_SUSPECT_M:
            suspect = True
            s["suspect_jumps"] += 1
        if not first and jump is not None:
            s["max_later_jump_m"] = round(max(s["max_later_jump_m"], jump), 3)
        s["fixes"] += 1
        s["global" if kind == "recognised" else "proximity"] += 1
        s["last_fix_t"] = now
        inl = self.last_stats.get("Loop/Visual_inliers/", "")
        self.evw.writerow(["%.3f" % now, self.cur, s["mark"], kind, m.loopClosureId,
                           m.proximityDetectionId, m.refId,
                           "" if x is None else "%.3f" % x, "" if y is None else "%.3f" % y,
                           "" if yaw is None else "%.1f" % yaw,
                           "" if jump is None else "%.3f" % jump, int(first), int(suspect),
                           "" if dist is None else "%.3f" % dist, "" if node is None else node,
                           inl, "%.3f" % s["odom_m"], "%.1f" % (now - s["t_start"])])
        self.evf.flush()

    def pending_link(self, links, ref, label=None):
        """a recognition RTAB-Map has NOT (yet) accepted: logged once per link, never counted as a fix"""
        with self.lock:
            self.read_starts()
            if self.cur is None:
                return
            s = self.st[self.cur]
            now = time.time()
            if self.cur > 1 and now - s["t_start"] < RELOAD_GRACE_S:
                return
            if label is None:
                new = [l for l in links if (l.fromId, l.toId) not in self.pending_seen]
                if not new:
                    return
                for l in new:
                    self.pending_seen.add((l.fromId, l.toId))
                label = "cache link %d->%d (pending)" % (new[0].fromId, new[0].toId)
            s["pending"] += 1
            if s["first_pending_s"] is None:
                s["first_pending_s"] = round(now - s["t_start"], 1)
            self.evw.writerow(["%.3f" % now, self.cur, s["mark"], "pending", "", label, ref,
                               "", "", "", "", 0, 0, "", "", "", "%.3f" % s["odom_m"],
                               "%.1f" % (now - s["t_start"])])
            self.evf.flush()

    def on_pose(self, m):
        # subscribing is also what makes rtabmap publish it (CoreWrapper.cpp:2102); fusion.bag keeps it
        c = m.pose.covariance
        self.accepted = 0.0 < c[0] < NOT_LOCALISED_COV and 0.0 < c[7] < NOT_LOCALISED_COV
        if self.accepted:
            p = m.pose.pose
            self.evidence("uncertainty", (p.position.x, p.position.y, math.degrees(yaw_of(p.orientation)),
                                          math.sqrt(max(c[0], c[7]))), "", 0)

    def evidence(self, kind, pose, link, ref, t=None):
        with self.lock:
            self._evidence(kind, pose, link, ref, t)

    def _evidence(self, kind, pose, link, ref, t):
        """A lasting trace of an ACCEPTED fix (see the header). Counts once per start as its first fix
        if no recognition event has been seen; otherwise only updates last_fix_t."""
        now = time.time() if t is None else t
        self.read_starts()
        if self.cur is None:
            if self.pending is None:
                self.pending = (kind, pose, link, ref, now)   # before starts.csv exists: kept for start 1
            return
        s = self.st[self.cur]
        if self.cur > 1 and now - s["t_start"] < RELOAD_GRACE_S:
            return
        s["last_fix_t"] = now
        s.setdefault("evidence", {})
        s["evidence"][kind] = s["evidence"].get(kind, 0) + 1
        if s["first_fix_t"] is not None:
            return
        if pose is None:
            C = self.lookup("map", "odom"); O = self.lookup("odom", "base_link")
            if C is not None and O is not None:
                Mp = C.dot(O)
                pose = (Mp[0, 2], Mp[1, 2], math.degrees(math.atan2(Mp[1, 0], Mp[0, 0])), None)
        x = y = yaw = dist = node = None
        if pose is not None:
            x, y, yaw = pose[0], pose[1], pose[2]
            dist, node = self.dist_to_path(x, y)
        dmark, _ = self.dist_to_mark(x, y, yaw)
        if dmark is not None and dmark > JUMP_SUSPECT_M and s["odom_m"] < 3.0 and now - s["t_start"] <= 120.0:
            s["wrong_place_fixes"] += 1
        s.update(first_fix_t=now, first_fix_kind=kind, first_fix_odom_m=round(s["odom_m"], 3),
                 first_fix_s=round(now - s["t_start"], 1), first_fix_jump_m=None,
                 first_fix_xy=None if x is None else [round(x, 3), round(y, 3), round(yaw, 1)],
                 first_fix_dist_to_mark_m=None if dmark is None else round(dmark, 3))
        s["fixes"] += 1
        self.C_prev = self.lookup("map", "odom")
        self.evw.writerow(["%.3f" % now, self.cur, s["mark"], kind, "", link, ref,
                           "" if x is None else "%.3f" % x, "" if y is None else "%.3f" % y,
                           "" if yaw is None else "%.1f" % yaw, "", 1, 0,
                           "" if dist is None else "%.3f" % dist, "" if node is None else node,
                           "", "%.3f" % s["odom_m"], "%.1f" % (now - s["t_start"])])
        self.evf.flush()

    # ---------- 1 Hz / 2 s loop ----------
    def tick(self):
        self.read_starts()
        now = time.time()
        el = now - self.t0
        if self.cur is None:
            self.write("LOCALISE waiting for start 1 (no starts.csv line yet)  %ds" % el)
            return
        s = self.st[self.cur]
        O = self.lookup("odom", "base_link")
        if O is not None:
            if self.odom_prev is not None:
                s["odom_m"] += math.hypot(O[0, 2] - self.odom_prev[0, 2], O[1, 2] - self.odom_prev[1, 2])
            self.odom_prev = O
            if self.odom_start is None:
                self.odom_start = O
        C = self.lookup("map", "odom")
        loc = s["first_fix_t"] is not None
        # until the first fix, remember the ASSUMED correction (the localiser's default guess, set on
        # its first frame after a (re)load - so only from 5 s after the start), so the first fix's
        # jump measures how far recognition moved the robot from the guess
        if not loc and C is not None and now - s["t_start"] > 5.0:
            self.C_prev = C
        x = y = yaw = dist = None
        if C is not None and O is not None:
            Mp = C.dot(O)
            x, y, yaw = Mp[0, 2], Mp[1, 2], math.degrees(math.atan2(Mp[1, 0], Mp[0, 0]))
            dist, _ = self.dist_to_path(x, y)
        dmark, dyaw = self.dist_to_mark(x, y, yaw)
        self.trw.writerow(["%.3f" % now, self.cur, int(loc), "" if x is None else "%.3f" % x,
                           "" if y is None else "%.3f" % y, "" if yaw is None else "%.1f" % yaw,
                           "%.3f" % s["odom_m"], "" if (dist is None or not loc) else "%.3f" % dist,
                           int(self.accepted), "" if dmark is None else "%.3f" % dmark,
                           "" if dyaw is None else "%.1f" % dyaw])
        self.trf.flush()
        n = len(self.starts)
        tot = max(self.a.planned_starts, n)
        info_age = now - self.last_info if self.last_info else None
        if info_age is None or info_age > 15:
            health = " | !! no /rtabmap/info for %s" % ("ever" if info_age is None else "%ds" % info_age)
        else:
            health = ""
        mk = "" if dmark is None else " | %.2f m from the expected spot (tape + odometry since the start)%s" % (
            dmark, " - WRONG PLACE?" if (loc and self.accepted and dmark > JUMP_SUSPECT_M) else "")
        if loc:
            ts = time.strftime("%H:%M:%S", time.localtime(s["first_fix_t"]))
            line = ("LOCALISED yes (accepted by RTAB-Map) since %s (first fix %s after %.0f s, %.1f m driven)"
                    " | start %d/%d %s%s | fixes %d (%d recognised) | last fix %ds ago | suspect jumps %d"
                    % (ts, s["first_fix_kind"], s["first_fix_s"], s["first_fix_odom_m"], self.cur, tot,
                       s["mark"], mk, s["fixes"], s["global"], now - s["last_fix_t"], s["suspect_jumps"]))
        else:
            line = ("LOCALISED no - searching %.0f s, %.1f m driven | start %d/%d %s | recognitions pending"
                    " (not accepted) %d | the dot on the map is only the guess, not a fix%s"
                    % (now - s["t_start"], s["odom_m"], self.cur, tot, s["mark"], s["pending"], mk))
        self.write("%s%s  %ds" % (line, health, el))
        with open(os.path.join(self.rec, "localise_summary.json.tmp"), "w") as f:
            json.dump({"run": self.a.run, "map_db": self.a.map_db, "starts": [self.st[k] for k in sorted(self.st)],
                       "written": now}, f, indent=1)
        os.replace(os.path.join(self.rec, "localise_summary.json.tmp"),
                   os.path.join(self.rec, "localise_summary.json"))

    def write(self, line):
        tmp = self.prog + ".tmp"
        with open(tmp, "w") as f:
            f.write(line + "\n")
        os.replace(tmp, self.prog)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", required=True)
    ap.add_argument("--map-db", required=True, help="the MASTER map, opened read-only, for node positions")
    ap.add_argument("--planned-starts", type=int, default=5)
    ap.add_argument("--marks", default="",
                    help="marks.json: each mark's expected place on the map, [x, y, yaw_deg] (optional)")
    ap.add_argument("--rtabmap-pid", type=int, default=0,
                    help="stop by itself 60 s after this process has gone (0 = never)")
    a = ap.parse_args(rospy.myargv(argv=sys.argv)[1:])
    rospy.init_node("localise_watch", anonymous=False, disable_signals=False)
    w = Watch(a)
    gone = None
    r = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        try:
            w.tick()
        except Exception as e:  # never die silently: say it on the jobs page
            w.write("LOCALISE watcher error: %s  %ds" % (e, time.time() - w.t0))
        if a.rtabmap_pid:
            alive = os.path.exists("/proc/%d" % a.rtabmap_pid)
            if alive:
                try:
                    alive = open("/proc/%d/comm" % a.rtabmap_pid).read().strip() == "rtabmap"
                except OSError:
                    alive = False
            if not alive:
                gone = gone or time.time()
                if time.time() - gone > 60:
                    w.write("LOCALISE ended: rtabmap (pid %d) stopped; summary in %s/localise_summary.json  %ds"
                            % (a.rtabmap_pid, w.rec, time.time() - w.t0))
                    break
        r.sleep()


if __name__ == "__main__":
    main()
