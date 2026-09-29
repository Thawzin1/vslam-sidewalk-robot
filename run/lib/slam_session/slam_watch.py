#!/usr/bin/env python3
"""slam_watch.py - the live "has the new session joined drive 10's map?" line (Jetson, ROS 1).

Used live in the SLAM session (27 Sept 2026, s2_slam_01). Started by start_drive_slam.sh (SLAM=1) as soon as
the map has loaded (step 3S).

WHAT "JOINED" MEANS
    RTAB-Map opens drive 10's map (node ids 1..OLD_MAX, map id 0) and starts a NEW map (map id 1) at the robot's
    first picture. The two are separate until RTAB-Map recognises a place of the old map and links a new picture
    to it. From then on they are one map (DESIGN.md section 1).
    *Plain terms: until it says JOINED, the robot is drawing on a blank sheet next to drive 10's floor plan; at
    JOINED it has recognised where on the floor plan it is and glued the two sheets together.*

SOURCES (both published by RTAB-Map itself)
    /rtabmap/mapGraph  the map's current working graph: which node ids are in it and every link between them.
                       STATE, not events: a join that happened before this watcher started is still seen here
                       (the first bench, 27 Sept, joined at the 3rd picture - before a watcher listening only to
                       events had started). Joined = a link between an id <= OLD_MAX and an id > OLD_MAX, and old
                       ids present in the working graph (before the join it holds today's nodes only).
    /rtabmap/info      per-picture events: new links (recognised again = loopClosureId, nearby re-match =
                       proximityDetectionId) with the recognition score, for the events file.
    /rtabmap/odom      distance driven (0.10 m dead band, so a parked camera's jitter does not count).
WRITES ~/jobs/<run>_slam.progress (jobs page, rule 13) every ~5 s;
       ~/.run_records/<run>/slam_events.csv (one row per link event); slam_summary.json at the end.
LIVE GUARD: "!! SUSPECT JOIN" when the first old place joined is further from the start mark than the robot can
    have driven + 3 m (RTAB-Map joins on ONE recognition in mapping mode - DESIGN.md 1.2 h).
  usage: python3 slam_watch.py --run s2_slam_01 --old-max 1558 --rtabmap-pid <pid> --old-facts <old_map_facts.json>
"""
from __future__ import print_function
import argparse, json, os, threading, time
import rospy
from rtabmap_msgs.msg import Info, MapGraph
from nav_msgs.msg import Odometry


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", required=True)
    ap.add_argument("--old-max", type=int, required=True, help="highest node id of the loaded old map")
    ap.add_argument("--rtabmap-pid", type=int, default=0)
    ap.add_argument("--old-facts", default="", help="old_map_facts.json (old nodes' saved x, y)")
    ap.add_argument("--start-xy", default="0,0", help="where the session starts on the old map (start mark)")
    a = ap.parse_args(rospy.myargv()[1:])
    rec = os.path.join(os.path.expanduser(os.environ.get("RECORDS_DIR", "~/.run_records")), a.run)
    prog = os.path.expanduser("~/jobs/%s_slam.progress" % a.run)
    rospy.init_node("slam_watch", anonymous=True, disable_signals=True)
    t0 = time.time()
    oldxy = json.load(open(a.old_facts)).get("old_graph_xy", {}) if a.old_facts else {}
    sx, sy = [float(v) for v in a.start_xy.split(",")]
    lock = threading.Lock()
    st = {"driven_m": 0.0, "anchor": None, "suspect": None, "join": None, "graph": None,
          "ev_old": {"global": 0, "nearby": 0}, "ev_new": {"global": 0, "nearby": 0},
          "hyp_max_before_join": 0.0, "last_info": None, "last_graph": None, "max_new_id": 0}
    ev = open(os.path.join(rec, "slam_events.csv"), "a")
    if ev.tell() == 0:
        ev.write("t_epoch,t_since_watch_start_s,kind,new_id,other_id,other_map,highest_hypothesis\n")
        ev.flush()

    def set_join(now, new_id, old_id, kind, hyp, source):
        xy = oldxy.get(str(old_id))
        d = round(((xy[0] - sx) ** 2 + (xy[1] - sy) ** 2) ** 0.5, 2) if xy else None
        st["join"] = {"seen_t_epoch": now, "seen_after_watch_start_s": round(now - t0, 1), "kind": kind,
                      "new_id": new_id, "old_id": old_id, "highest_hypothesis": hyp, "source": source,
                      "old_node_from_start_m": d, "driven_m": round(st["driven_m"], 2)}
        if d is not None and d > st["driven_m"] + 3.0:
            st["suspect"] = "old node %d is %.1f m from the start mark, robot driven %.1f m" % (old_id, d, st["driven_m"])

    def stat(m, key):
        for k, v in zip(m.statsKeys, m.statsValues):
            if k.startswith(key):
                return v
        return None

    def info_cb(m):
        now = time.time()
        with lock:
            st["last_info"] = now
            st["max_new_id"] = max(st["max_new_id"], m.refId)
            hyp = stat(m, "Loop/Highest_hypothesis_value") or 0.0
            if st["join"] is None:
                st["hyp_max_before_join"] = max(st["hyp_max_before_join"], hyp)
            for kind, other in (("global", m.loopClosureId), ("nearby", m.proximityDetectionId)):
                if other <= 0:
                    continue
                old = other <= a.old_max
                (st["ev_old"] if old else st["ev_new"])[kind] += 1
                ev.write("%.3f,%.1f,%s,%d,%d,%s,%.4f\n" % (now, now - t0, kind, m.refId, other,
                                                            "old" if old else "new", hyp))
                ev.flush()
                if old and st["join"] is None:
                    set_join(now, m.refId, other, kind, round(hyp, 4), "/rtabmap/info")

    def graph_cb(m):
        now = time.time()
        ids = list(m.posesId)
        n_old = sum(1 for i in ids if i <= a.old_max)
        inter = [(l.fromId, l.toId, l.type) for l in m.links
                 if l.fromId != l.toId and (l.fromId <= a.old_max) != (l.toId <= a.old_max)]
        with lock:
            st["last_graph"] = now
            st["graph"] = {"poses": len(ids), "old": n_old, "new": len(ids) - n_old,
                           "links_between_sessions": len(set(tuple(sorted(x[:2])) for x in inter))}
            if inter and st["join"] is None:
                f, t, ty = min(inter, key=lambda x: max(x[0], x[1]))      # the earliest new node
                new_id, old_id = (f, t) if f > a.old_max else (t, f)
                set_join(now, new_id, old_id, {1: "global", 2: "nearby"}.get(ty, "type %d" % ty), None,
                         "/rtabmap/mapGraph")

    def odom_cb(m):
        p = m.pose.pose.position
        with lock:
            if st["anchor"] is None:
                st["anchor"] = (p.x, p.y)
                return
            d = ((p.x - st["anchor"][0]) ** 2 + (p.y - st["anchor"][1]) ** 2) ** 0.5
            if d >= 0.10:                          # dead band: parked jitter never adds up
                if d < 2.0:                        # a jump (tracking restart) is not driving
                    st["driven_m"] += d
                st["anchor"] = (p.x, p.y)

    rospy.Subscriber("/rtabmap/info", Info, info_cb, queue_size=20)
    rospy.Subscriber("/rtabmap/mapGraph", MapGraph, graph_cb, queue_size=2, buff_size=2 ** 24)
    rospy.Subscriber("/rtabmap/odom", Odometry, odom_cb, queue_size=20)

    def alive():
        if not a.rtabmap_pid:
            return True
        try:
            return open("/proc/%d/comm" % a.rtabmap_pid).read().strip() == "rtabmap"
        except IOError:
            return False

    def line():
        with lock:
            j, g = st["join"], st["graph"]
            if j:
                js = "JOINED yes (seen %.0f s after watch start; new %d -> old %d, %s; old place %s m from the start mark)" % (
                    j["seen_after_watch_start_s"], j["new_id"], j["old_id"], j["kind"], j["old_node_from_start_m"])
            else:
                js = "JOINED no (best score so far %.3f)" % st["hyp_max_before_join"]
            if st["suspect"]:
                js = "!! SUSPECT JOIN (%s) - " % st["suspect"] + js
            gs = ("working graph %d old + %d new, %d links between sessions" % (g["old"], g["new"],
                  g["links_between_sessions"]) if g else "no working graph yet")
            now = time.time()
            silent = "" if st["last_info"] and now - st["last_info"] < 10 else "  !! no map messages for %s s" % (
                "ever" if not st["last_info"] else "%.0f" % (now - st["last_info"]))
            return "SLAM %s  %s  driven %.1f m  new links to old %d+%d, within new %d+%d%s  %ds" % (
                js, gs, st["driven_m"], st["ev_old"]["global"], st["ev_old"]["nearby"], st["ev_new"]["global"],
                st["ev_new"]["nearby"], silent, now - t0)

    while alive() and not rospy.is_shutdown():
        with open(prog + ".tmp", "w") as fh:
            fh.write(line() + "\n")
        os.rename(prog + ".tmp", prog)
        time.sleep(5)
    last = line()
    with lock:
        summ = {k: v for k, v in st.items() if k != "anchor"}
    summ.update(run=a.run, old_max=a.old_max, t_start=t0, t_end=time.time())
    json.dump(summ, open(os.path.join(rec, "slam_summary.json"), "w"), indent=1)
    with open(prog, "w") as fh:
        fh.write("SLAM complete (map closed) - %s\n" % last.replace("SLAM ", "", 1))


if __name__ == "__main__":
    main()
