#!/usr/bin/env python3
"""live_map_server.py — watch the map build, in a browser tab, over WiFi.

WHY THIS EXISTS

    Watching a mapping run needs a picture of the occupancy grid. The two
    obvious ways both fail over WiFi:

      X11 forwarding  RViz cannot use the Jetson's GPU for a window on your
                      viewer's computer, so it falls back to shipping whole bitmaps every
                      frame. Fine on the 0.5 ms cable, unusable on WiFi - it
                      stalled mid-drive on 2026-08-31.
      VNC             Better, but it is a protocol negotiation with a client
                      that has to agree about encryption. TigerVNC 1.15 refuses
                      x11vnc's unauthenticated offer and hangs up, which is a
                      thing to debug rather than a thing that works.

    This sends ONE SMALL PNG of the occupancy grid, over plain HTTP, to any
    browser. Nothing to install, nothing to negotiate, and a 5 cm grid of a lab
    is a few kilobytes. The same pattern as the storage dashboard on port 8092,
    which stayed readable even when the Jetson was too busy to accept an SSH
    login.

    Plain terms: instead of sending you a live video of a 3D program, it sends
    you a small map picture that refreshes. That is all you actually need while
    driving.

WHAT IT SHOWS
    occupied cells in red, free in pale grey, unknown in dark grey, the robot
    as a green dot, and the three things the driver must know to trust the
    picture:
      - is the camera still tracking?          (a red TRACKING LOST banner)
      - has the map corrected itself yet?      ("last map correction N s ago,
                                                 M so far")
      - where does the CORRECTED map put the robot?   (the green dot)

WHAT CHANGED ON 2026-09-24 (rank 2 of the occupancy research report,
REPORT.md (project records) section 4.2)

    1. THE DOT COMES FROM TF map -> base_link, NOT FROM /rtabmap/odom.
       (TF: ROS's live record of where each part of the robot is.)
       /rtabmap/odom is the camera's raw step-by-step tracking, in the "odom"
       frame. The picture is the map, in the "map" frame. The two agree only
       until the first loop closure (the map recognising a place it has seen
       before and pulling itself back into line). After that, RTAB-Map keeps
       the correction in the map -> odom transform, which the old page never
       applied. On drive 2 the dot sat 19 to 39 m from the robot for the last
       8 minutes. Asking TF for map -> base_link gives the map's own answer.

    2. ODOMETRY MESSAGES ARE NOT READ AT ALL. When tracking is lost, the
       odometry node publishes a message whose position is all zeros (it
       means "lost", not "at the start"). The old page drew that as the robot
       jumping to the start mark. The odometry node publishes NO TF while lost
       (rtabmap_odom OdometryROS.cpp, 0.21.13: the TF is sent only when the
       pose is valid), so the TF-based dot simply stays at the last good fix.

    3. A RED "TRACKING LOST" BANNER from /rtabmap/odom_info_lite, field
       "lost" (rtabmap_msgs/OdomInfo, verified with `rosmsg show` on the
       Jetson). It clears on the first "not lost" message; the page asks
       twice a second, so it clears within about half a second.
       WHY THE "_lite" TOPIC (changed after review, 2026-09-24): the odometry
       node (the program that tracks the camera frame to frame) publishes the
       same message type on two topics. /rtabmap/odom_info carries every
       matched feature and the whole local feature map; /rtabmap/odom_info_lite
       has those arrays emptied but keeps "lost" and "inliers" (rtabmap_odom
       OdometryROS.cpp 0.21.13, lines 106-107 and 946-968; both names are in
       the installed /opt/ros/noetic/lib/librtabmap_odom.so). The node builds a
       message only while something listens, and on a drive nothing else
       listens to either, so reading the full one made the odometry node copy
       and send every feature just for this banner. Measured on drive 2's
       pictures with a real rgbd_odometry and the drive's settings file
       (t2_lite_real.sh, two runs, 2026-09-24): WHILE TRACKING the full
       message's median was 55-57 KB and the light one's 30-32 KB - the light
       one still carries the local-bundle lists, about 0.5 KB per recent key
       frame, so it is not tiny; WHILE LOST, 24-25 KB against 0.7 KB. Python
       decoding of made-up messages sized like the real tracking ones: 1.25 ms
       light against 2.9 ms full, about 2 % against 4 % of one core at 15
       frames a second. On the replay's real mixed stream (about 45 % lost
       frames) this page used 2.5-2.7 ms less per message with the light one.

    4. "LAST MAP CORRECTION N s AGO, M SO FAR" from /rtabmap/info, fields
       loopClosureId and proximityDetectionId (rtabmap_msgs/Info, verified the
       same way). A non-zero value means a loop closure was accepted in that
       update, which is when the map corrects itself. Until the first one, the
       picture rests on step-by-step tracking, and the page says so.
       (Not quite "raw": RTAB-Map re-checks each step against the one before,
       RGBD/NeighborLinkRefining=true, and in test T2 that alone moved drive
       2's map up to 3.2 m from the raw tracking before the first closure.
       The dot follows that too, because it comes from TF. But nothing pulls
       the accumulated error back until a place is recognised again.)

    TESTED: test T2, 2026-09-24, drive 2 replayed on a spare ROS master -
    results_page_fix.md (project records)

    The page stays subscribed to /rtabmap/grid_map at all times: RTAB-Map
    builds the map picture only while something listens, and on drive 1
    nothing did, so no live map was ever built.

WHY A PERSISTENT SUBSCRIBER
    RTAB-Map's grid publisher goes quiet after one near-empty publish unless a
    subscriber stays connected (docs/SOLVED.md). A connect-grab-disconnect per
    request would sample that first empty grid forever.

WHY THE TF LOOKUP NEVER WAITS ON THE ROS CLOCK
    tf2's own "timeout" sleeps on ROS time (tf2_ros/buffer.py, can_transform).
    In a replay, ROS time is the recording's clock, which stands still between
    recorded frames and stops for good when the replay ends, so a tf2 timeout
    would freeze the thread. Instead a plain thread asks for the LATEST
    available transform ten times a second, with no waiting, and keeps the
    last good answer when one attempt fails.

SIDE BY SIDE WITH THE LiDAR (2026-09-24, the user's request for drive 3)
    _peer:=http://<robot>:8095 adds a second panel: the robot's live LiDAR view
    (tools/robot_side/lidar_live_map.py, running on the robot). THIS server
    fetches it and passes it on, so the one link works from anywhere the Jetson
    does - the robot's own address is reachable only on the McMaster network.
    If the robot's view is down, its panel says so; the camera panel is not
    affected.

FOR TESTS: /map.png carries its own map coordinates in response headers
    (X-Map-Left, X-Map-Top, X-Map-M-Per-Px: where the picture sits on the
    map), and the dot's position and TF time (X-Dot-*). A checker can then find
    the green dot in the picture and compare it with TF independently. The
    browser ignores these headers.

    usage, on the Jetson:
      rosrun sidewalk_slam live_map_server.py            # then browse to :8093
      rosrun sidewalk_slam live_map_server.py _port:=8095
      rosrun sidewalk_slam live_map_server.py _port:=8095 _peer:=http://<robot-wifi-address>:8095
    optional: _map_frame:=map _base_frame:=base_link
              _odom_info:=/rtabmap/odom_info_lite _info:=/rtabmap/info

IF THE ROBOT IS FAR OUTSIDE THE MAP (added after review, 2026-09-24)
    The picture is cropped to the mapped area and grows to include the robot.
    That growth is limited: beyond OUTSIDE_LIMIT_M (10 m) outside the mapped
    rectangle, the picture stays the size of the map, the dot is not drawn,
    a magenta arrow at the edge points towards it, and the "robot" line says
    how far out it is. Without a limit, a wild position jump would make every
    picture request allocate memory that grows with the square of the
    distance (about 10 GB for 1 km).
"""
import json
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer as HTTPServer
import urllib.request

import numpy as np
import rospy
import tf2_ros
from nav_msgs.msg import OccupancyGrid

try:
    from rtabmap_msgs.msg import Info, OdomInfo
except ImportError:                                          # pragma: no cover
    Info = OdomInfo = None

try:
    import cv2
except ImportError:                                          # pragma: no cover
    cv2 = None

OCC = 50

# How old the last tracking message may be before the page stops trusting it.
# The odometry node sends one per camera frame (15 a second), so 3 s of
# silence means it has stopped, not that it is slow.
TRACK_STALE_S = 3.0
# How old the dot's TF time may be before it is drawn hollow ("last known
# position, not live"). Live TF arrives 15 times a second.
DOT_STALE_S = 2.0
# How far OUTSIDE the rectangle of mapped cells the robot may be and still
# have the picture grow to include it. The camera maps up to Grid/RangeMax =
# 8 m ahead (config/rtabmap_zedx.yaml), so a real robot is always within a few
# metres of cells it has mapped itself; 10 m is that plus a margin. Beyond it
# the position is a jump (for example a wild odometry step), and growing the
# picture to include it would cost memory with the square of the distance:
# 60 picture pixels per metre each way at 5 cm squares scaled 3x, so 1 km
# would be about 10 GB per request.
OUTSIDE_LIMIT_M = 10.0

# Colours, BGR (OpenCV's order). The dot colour is used ONLY for the dot, so
# a checker can find the dot in the picture by colour alone.
C_UNKNOWN = (55, 55, 55)
C_FREE = (232, 232, 232)
C_OCC = (40, 40, 200)
C_DOT = (60, 200, 60)
C_HEADING = (20, 110, 20)
C_FAR = (255, 0, 255)          # magenta: the edge arrow when the robot is off the picture
C_SEEN = (255, 230, 0)         # cyan (BGR): what the camera sees NOW that stands up from the floor (27 Sept)
C_OUTLINE = (0, 0, 0)          # the robot's outline: black on the white free floor (27 Sept)
C_TURN = (90, 90, 90)          # the circle the outline sweeps when turning on the spot, plus the safety margin
LIVE = {"pts": None, "frame": None, "t": 0.0, "lock": threading.Lock()}
HALF_L, HALF_W, MARGIN = 0.495, 0.335, 0.10

STATE = {
    "grid": None, "grid_n": 0, "grid_rx": None,
    "cells": None,                 # (grid_n, cells int16 array, bounds, counts)
    "route": [],
    "pose": None,                  # last good TF map->base_link
    "pose_err": "no answer from TF yet",
    "track": {"n": 0, "lost": False, "rx": None, "inliers": 0,
              "lost_since": None, "losses": 0},
    "corr": {"info_n": 0, "info_rx": None, "n": 0, "loop": 0, "prox": 0,
             "stamp": None, "stamp_ns": None, "node": None, "with": None,
             "rx": None},
    "lock": threading.Lock(),
}
CFG = {"map_frame": "map", "base_frame": "base_link",
       "odom_info": "/rtabmap/odom_info_lite", "info": "/rtabmap/info",
       "msgs_ok": Info is not None}


def grid_cb(msg):
    with STATE["lock"]:
        STATE["grid"] = msg
        STATE["grid_n"] += 1
        STATE["grid_rx"] = time.time()


def odom_info_cb(msg):
    """Tracking state, one message per camera frame. Only 'lost' and the
    inlier count are used; the message's position fields are never read."""
    now = time.time()
    lost = bool(msg.lost)
    with STATE["lock"]:
        t = STATE["track"]
        if lost and not t["lost"]:
            t["losses"] += 1
            t["lost_since"] = now
        if not lost:
            t["lost_since"] = None
        t["lost"] = lost
        t["inliers"] = int(msg.inliers)
        t["rx"] = now
        t["n"] += 1


def info_cb(msg):
    """One message per map update. A non-zero loopClosureId (recognised by
    appearance) or proximityDetectionId (recognised by being nearby) means a
    loop closure was accepted in this update, i.e. the map corrected itself."""
    now = time.time()
    closed = msg.loopClosureId != 0 or msg.proximityDetectionId != 0
    with STATE["lock"]:
        c = STATE["corr"]
        c["info_n"] += 1
        c["info_rx"] = now
        if closed:
            c["n"] += 1
            c["loop"] += int(msg.loopClosureId != 0)
            c["prox"] += int(msg.proximityDetectionId != 0)
            c["stamp"] = msg.header.stamp.to_sec()
            c["stamp_ns"] = msg.header.stamp.to_nsec()
            c["node"] = int(msg.refId)
            c["with"] = int(msg.loopClosureId or msg.proximityDetectionId)
            c["rx"] = now


def pose_loop(buf):
    """Ask TF for the LATEST map -> base_link, ten times a second, never
    waiting on the ROS clock (see the docstring). A failed attempt keeps the
    last good answer and records why, so the page can say so."""
    while not rospy.is_shutdown():
        try:
            t = buf.lookup_transform(CFG["map_frame"], CFG["base_frame"],
                                     rospy.Time(0))
        except tf2_ros.TransformException as exc:
            with STATE["lock"]:
                STATE["pose_err"] = str(exc).splitlines()[0][:160]
        else:
            q = t.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y * q.y + q.z * q.z))
            p = {"x": t.transform.translation.x, "y": t.transform.translation.y,
                 "yaw": yaw, "stamp": t.header.stamp.to_sec(),
                 # exact, for tests: a float second loses the last digits,
                 # and asking TF for a time a fraction of a microsecond past
                 # its newest data is refused as "the future"
                 "stamp_ns": t.header.stamp.to_nsec()}
            with STATE["lock"]:
                STATE["pose"] = p
                STATE["pose_err"] = ""
        time.sleep(0.1)


def load_route(path):
    """Route points in MAP metres, from a csv of x,y.

    The route is a path the robot has already driven and recorded - not an
    invented line. Its coordinates are in the map's own frame, and a map's
    origin is wherever tracking began, NOT the floor mark. So two runs line up
    only as well as the parking does: follow the SHAPE, not the pixels.
    """
    if not path or not os.path.exists(path):
        return []
    out = []
    try:
        import csv as _csv
        for r in _csv.DictReader(open(path)):
            out.append((float(r["x"]), float(r["y"])))
    except Exception as exc:
        rospy.logwarn("route %s unreadable: %s", path, exc)
        return []
    return out


def draw_route(img, route, pose, to_px, res, w, h, rows):
    """Draw the route to follow, and say which part comes next."""
    if not route or cv2 is None:
        return
    pts = [to_px(x, y) for x, y in route]
    # Amber, distinct from the red obstacles and the green robot.
    for a, b in zip(pts, pts[1:]):
        cv2.line(img, a, b, (0, 170, 255), max(1, int(0.05 / res)))

    # Start of the route, so the direction to set off is unambiguous.
    if pts:
        cv2.circle(img, pts[0], max(3, int(0.22 / res)), (0, 170, 255), 2)

    # Arrows every so often, so the DIRECTION of travel is visible - a bare
    # line does not say which way round to drive it, and direction is the one
    # thing this project has proved matters.
    #
    # DRAW BETWEEN ADJACENT POINTS ONLY. The first version drew from point i to
    # point i+step, which on a closed loop is a CHORD STRAIGHT ACROSS THE
    # MIDDLE. The route was fine; the arrows made it look like a scribble.
    step = max(1, len(pts) // 10)
    for i in range(0, len(pts) - 1, step):
        a, b = pts[i], pts[i + 1]
        if abs(b[0] - a[0]) + abs(b[1] - a[1]) < 2:
            continue                    # too short to point anywhere
        cv2.arrowedLine(img, a, b, (0, 210, 255),
                        max(1, int(0.06 / res)), tipLength=0.8)

    if pose is None:
        return
    # "Where am I on the route" uses the CORRECTED position too (TF), for the
    # same reason as the dot.
    rx, ry = pose["x"], pose["y"]
    d = [math.hypot(x - rx, y - ry) for x, y in route]
    i = min(range(len(d)), key=lambda k: d[k])
    p = to_px(*route[i])
    if 0 <= p[0] < w and 0 <= p[1] < h:
        cv2.circle(img, p, max(3, int(0.25 / res)), (255, 255, 255), 2)
    rows["route"] = "point %d of %d, %.2f m off the line" % (
        i + 1, len(route), d[i])
    done = 0.0
    for k in range(i):
        done += math.hypot(route[k + 1][0] - route[k][0],
                           route[k + 1][1] - route[k][1])
    total = done
    for k in range(i, len(route) - 1):
        total += math.hypot(route[k + 1][0] - route[k][0],
                            route[k + 1][1] - route[k][1])
    rows["progress"] = "%.1f of %.1f m (%.0f %%)" % (
        done, total, 100.0 * done / max(total, 1e-6))


def grid_cells(g, n):
    """The grid as a numpy array, plus its mapped extent and counts - worked
    out ONCE per grid message, not once per browser request (converting a
    few million cells takes a noticeable fraction of a second)."""
    with STATE["lock"]:
        c = STATE["cells"]
    if c is not None and c[0] == n:
        return c
    W, H = g.info.width, g.info.height
    cells = np.array(g.data, dtype=np.int16).reshape(H, W)
    occ = cells > OCC
    free = (cells >= 0) & (cells <= OCC)
    nz = np.argwhere(occ | free)
    bounds = None
    if len(nz):
        (r0, c0), (r1, c1) = nz.min(0), nz.max(0)
        bounds = (int(r0), int(c0), int(r1) + 1, int(c1) + 1)
    c = (n, cells, bounds, (int(occ.sum()), int(free.sum())))
    with STATE["lock"]:
        STATE["cells"] = c
    return c


def snapshot():
    with STATE["lock"]:
        return {"grid": STATE["grid"], "grid_n": STATE["grid_n"],
                "grid_rx": STATE["grid_rx"], "route": STATE["route"],
                "pose": dict(STATE["pose"]) if STATE["pose"] else None,
                "pose_err": STATE["pose_err"],
                "track": dict(STATE["track"]), "corr": dict(STATE["corr"])}


def ros_now():
    try:
        return rospy.Time.now().to_sec()
    except Exception:                                        # pragma: no cover
        return 0.0


def finite(v):
    """A number the browser can read: JSON has no NaN or Infinity, and one
    such value makes the whole status unreadable to the page."""
    return v if isinstance(v, (int, float)) and math.isfinite(v) else None


def metres(v, places=2):
    """A distance for people: fixed decimals normally, but short for absurd
    values ("%.2f" of 1e300 prints three hundred digits)."""
    if not math.isfinite(v) or abs(v) >= 1e6:
        return "%.3g" % v
    return "%.*f" % (places, v)


def outside_mapped_m(pose, g, bounds):
    """How far the robot is OUTSIDE the rectangle of mapped cells, in metres
    (0 when inside it). None when TF gave a position that is not a number.
    This is exactly how much the picture would have to grow to include it."""
    if pose is None or any(finite(pose[k]) is None for k in ("x", "y", "yaw")):
        return None
    if g is None:
        return 0.0
    if bounds is None:                          # nothing mapped yet: the grid's own extent
        bounds = (0, 0, g.info.height, g.info.width)
    # In metres against the rectangle's edges, never via cell numbers: a huge
    # but finite position divided by 0.05 m overflows to infinity, and
    # math.floor(infinity) raises (checked in Python 3).
    res = g.info.resolution
    ox, oy = g.info.origin.position.x, g.info.origin.position.y
    r0, c0, r1, c1 = bounds                     # r1, c1 are one past the last
    x, y = pose["x"], pose["y"]
    ddx = max(ox + c0 * res - x, x - (ox + c1 * res), 0.0)
    ddy = max(oy + r0 * res - y, y - (oy + r1 * res), 0.0)
    return math.hypot(ddx, ddy)                 # may be inf for absurd input


def status(snap=None):
    """Everything the page prints, WITHOUT drawing the picture. Plain-text
    rows for people; the "_state" block is the same facts as numbers, for the
    banner code and for tests."""
    s = snap or snapshot()
    now = time.time()
    rows = {}
    st = {}

    # --- 1. is the camera still tracking? ---------------------------------
    t = s["track"]
    lost = False
    stale = False
    if not CFG["msgs_ok"]:
        rows["tracking"] = "unknown - rtabmap_msgs not importable on this machine"
        stale = True
    elif t["n"] == 0:
        rows["tracking"] = "no tracking messages yet on %s" % CFG["odom_info"]
        stale = True
    elif now - t["rx"] > TRACK_STALE_S:
        rows["tracking"] = ("NO DATA for %.0f s - the odometry node may have "
                            "stopped" % (now - t["rx"]))
        stale = True
    elif t["lost"]:
        lost = True
        rows["tracking"] = "LOST for %.0f s" % (now - (t["lost_since"] or now))
    else:
        rows["tracking"] = "ok (%d matched features)" % t["inliers"]
    rows["tracking losses"] = "%d so far" % t["losses"]
    st.update(lost=lost, tracking_stale=stale,
              lost_for_s=(now - t["lost_since"]) if lost and t["lost_since"] else 0.0,
              tracking_text=rows["tracking"], losses=t["losses"])

    # --- 2. has the map corrected itself? ---------------------------------
    c = s["corr"]
    if not CFG["msgs_ok"]:
        corr = "unknown - rtabmap_msgs not importable"
    elif c["info_n"] == 0:
        corr = "no map updates received yet on %s" % CFG["info"]
    elif c["n"] == 0:
        corr = ("NONE YET - no place recognised a second time, so the map so "
                "far rests on step-by-step tracking and its errors are still "
                "adding up (%d map updates)" % c["info_n"])
    else:
        corr = "%.0f s ago (node %d), %d so far" % (now - c["rx"], c["node"], c["n"])
    rows["last map correction"] = corr
    st.update(corrections=c["n"], corr_loop=c["loop"], corr_prox=c["prox"],
              corr_stamp=c["stamp"], corr_stamp_ns=c["stamp_ns"],
              corr_node=c["node"], corr_with=c["with"],
              corr_age_s=(now - c["rx"]) if c["rx"] else None,
              map_updates=c["info_n"], corr_text=corr)

    # --- 3. where does the corrected map put the robot? -------------------
    p = s["pose"]
    if p is None:
        rows["robot"] = "no position from TF %s -> %s: %s" % (
            CFG["map_frame"], CFG["base_frame"], s["pose_err"])
        st["dot"] = None
    else:
        age = max(0.0, ros_now() - p["stamp"])
        live = age <= DOT_STALE_S and not lost
        bounds = None
        if s["grid"] is not None:
            bounds = grid_cells(s["grid"], s["grid_n"])[2]   # cached per grid
        out = outside_mapped_m(p, s["grid"], bounds)
        drawn = out is not None and out <= OUTSIDE_LIMIT_M
        rows["robot"] = "%s, %s m on the corrected map%s" % (
            metres(p["x"]), metres(p["y"]), "" if live else
            " - LAST KNOWN position, %.0f s old" % age)
        if out is None:
            rows["robot"] += (" - NOT A NUMBER from TF, not drawn (the map's "
                              "position estimate has failed)")
        elif not drawn:
            rows["robot"] += (" - %s m OUTSIDE the mapped area, not drawn; the "
                              "magenta arrow at the edge points to it (a position "
                              "jump - do not trust the dot until it comes back)" % metres(out, 1))
        st["dot"] = {"x": finite(p["x"]), "y": finite(p["y"]), "yaw": finite(p["yaw"]),
                     "stamp": p["stamp"], "stamp_ns": p["stamp_ns"],
                     "age_s": age, "live": live,
                     "outside_m": finite(out), "drawn": drawn}

    # --- 4. the picture itself --------------------------------------------
    if s["grid"] is None:
        rows["map picture"] = "nothing received yet"
    else:
        _, _, _, (n_occ, n_free) = grid_cells(s["grid"], s["grid_n"])
        g = s["grid"]
        rows["occupied"] = n_occ
        rows["free"] = n_free
        rows["publishes"] = s["grid_n"]
        rows["map picture"] = "last received %.0f s ago" % (now - s["grid_rx"])
        st["grid_stamp"] = g.header.stamp.to_sec()
    st["grid_n"] = s["grid_n"]
    st["server_time"] = now

    # The guard rewrites this file every 2 s; older than 15 s = no guard running.
    try:
        cg_path = os.path.expanduser("~/.run_records/camera_guard_live.json")
        if now - os.path.getmtime(cg_path) <= 15:
            with open(cg_path) as fh:
                cg = json.load(fh)
            if cg.get("state") in ("down", "gave_up"):
                st["camera_down"] = "CAMERA DOWN since %s - %s" % (
                    cg.get("down_since_hms") or "?", cg.get("last_event", ""))
                rows["camera"] = st["camera_down"]
            elif cg.get("restarts"):
                rows["camera"] = "restarted %d time(s) this drive" % int(cg["restarts"])
    except Exception:       # a malformed file (a list, a non-number) must never break the page (review item 5)
        st.pop("camera_down", None)
        rows.pop("camera", None)
    rows["_state"] = st
    return rows


def obstacles_cb(m):
    """27 Sept: the camera's obstacle points (people, bins, anything standing up from the floor) as the navigation
    sees them NOW - /nav/obstacles_cloud from rtabmap_util/obstacles_detection. Drawn fresh every refresh and dropped
    after 1.5 s, so a person appears while in view and disappears once gone. Never enters the saved map."""
    try:
        from sensor_msgs import point_cloud2
        pts = np.array([p[:2] + (p[2],) for p in point_cloud2.read_points(m, field_names=("x", "y", "z"), skip_nans=True)],
                       np.float32)
    except Exception:
        return
    with LIVE["lock"]:
        LIVE["pts"], LIVE["frame"], LIVE["t"] = pts, m.header.frame_id, time.time()


def draw_live(img, pose, to_px, res, rows, tf_buf):
    """Camera obstacles now (cyan dots), then the robot's outline and its turn-on-the-spot circle."""
    with LIVE["lock"]:
        pts, frame, t = LIVE["pts"], LIVE["frame"], LIVE["t"]
    if pts is not None and len(pts) and time.time() - t < 1.5 and tf_buf is not None:
        try:
            tr = tf_buf.lookup_transform(CFG["map_frame"], frame, rospy.Time(0))
            q = tr.transform.rotation; v = tr.transform.translation
            R = np.array([[1 - 2 * (q.y * q.y + q.z * q.z), 2 * (q.x * q.y - q.z * q.w), 2 * (q.x * q.z + q.y * q.w)],
                          [2 * (q.x * q.y + q.z * q.w), 1 - 2 * (q.x * q.x + q.z * q.z), 2 * (q.y * q.z - q.x * q.w)]])
            xy = pts @ R.T + np.array([v.x, v.y])
            r = max(1, int(0.03 / res))
            for x, y in xy[:: max(1, len(xy) // 3000)]:
                px, py = to_px(x, y)
                cv2.circle(img, (px, py), r, C_SEEN, -1)
            rows["seen now"] = "%d points standing up from the floor, from the camera (cyan; gone once out of view)" % len(pts)
        except Exception as exc:
            rows["seen now"] = "camera points not drawn: %s" % exc
    elif pts is None:
        rows["seen now"] = "no camera obstacle points (they come from the navigation / obstacle step)"
    if pose is not None:
        c, s_ = math.cos(pose["yaw"]), math.sin(pose["yaw"])
        corners = [to_px(pose["x"] + c * u - s_ * w, pose["y"] + s_ * u + c * w)
                   for u, w in ((HALF_L, HALF_W), (HALF_L, -HALF_W), (-HALF_L, -HALF_W), (-HALF_L, HALF_W))]
        cv2.polylines(img, [np.array(corners, np.int32)], True, C_OUTLINE, 1)
        px, py = to_px(pose["x"], pose["y"])
        cv2.circle(img, (px, py), int((math.hypot(HALF_L, HALF_W) + MARGIN) / res), C_TURN, 1)


def render_png(scale):
    """Occupancy grid -> PNG bytes, cropped to what has been mapped PLUS the
    robot (the old version dropped the dot silently when the robot stood
    outside the mapped area) - but the robot is included only while it is
    within OUTSIDE_LIMIT_M of the mapped rectangle, so the picture's size
    stays bounded. Returns (png, rows, headers)."""
    s = snapshot()
    g = s["grid"]
    if g is None or cv2 is None:
        return None, status(s), {}
    n, cells, bounds, _ = grid_cells(g, s["grid_n"])
    W, H = g.info.width, g.info.height
    res = g.info.resolution
    ox, oy = g.info.origin.position.x, g.info.origin.position.y
    rows = status(s)
    dot = rows["_state"].get("dot")
    # the pose is DRAWN only when it is a number and not too far out (status()
    # decided, so the picture and the text below it always agree)
    far_pose = s["pose"] if (s["pose"] is not None and dot and not dot["drawn"]) else None
    pose = s["pose"] if (dot and dot["drawn"]) else None

    pad = 6
    box = None
    if bounds is not None:
        r0, c0, r1, c1 = bounds
        box = [r0 - pad, c0 - pad, r1 + pad, c1 + pad]
    if pose is not None:
        # floor, not int(): int() rounds towards zero, which is wrong for a
        # robot left of or below the grid's origin
        pr = int(math.floor((pose["y"] - oy) / res))
        pc = int(math.floor((pose["x"] - ox) / res))
        m = int(math.ceil(0.6 / res))
        rb = [pr - m, pc - m, pr + m + 1, pc + m + 1]
        box = rb if box is None else [min(box[0], rb[0]), min(box[1], rb[1]),
                                      max(box[2], rb[2]), max(box[3], rb[3])]
    if box is None:
        return None, rows, {}
    r0, c0, r1, c1 = box
    h, w = r1 - r0, c1 - c0

    img = np.full((h, w, 3), C_UNKNOWN[0], np.uint8)          # unknown
    gr0, gr1 = max(r0, 0), min(r1, H)
    gc0, gc1 = max(c0, 0), min(c1, W)
    if gr1 > gr0 and gc1 > gc0:
        sub = cells[gr0:gr1, gc0:gc1]
        view = img[gr0 - r0:gr1 - r0, gc0 - c0:gc1 - c0]
        view[(sub >= 0) & (sub <= OCC)] = C_FREE
        view[sub > OCC] = C_OCC

    def to_px(x, y):
        return (int(math.floor((x - ox) / res)) - c0,
                int(math.floor((y - oy) / res)) - r0)

    # Route FIRST, so the robot marker is drawn on top of it and never hidden.
    draw_route(img, s["route"], pose, to_px, res, w, h, rows)
    draw_live(img, pose, to_px, res, rows, CFG.get("tf_buf"))

    headers = {}
    if pose is not None:
        px, py = to_px(pose["x"], pose["y"])
        # heading line first, in its own darker colour, then the dot on top:
        # the dot colour then belongs to the dot alone
        cv2.line(img, (px, py),
                 (int(px + math.cos(pose["yaw"]) * 0.45 / res),
                  int(py + math.sin(pose["yaw"]) * 0.45 / res)), C_HEADING, 2)
        rad = max(2, int(0.18 / res))
        live = bool(dot and dot["live"])
        # filled = live position; hollow ring = last known position only
        cv2.circle(img, (px, py), rad, C_DOT, -1 if live else 2)
        headers.update({"X-Dot-X": "%.4f" % pose["x"],
                        "X-Dot-Y": "%.4f" % pose["y"],
                        "X-Dot-Yaw": "%.4f" % pose["yaw"],
                        "X-Dot-Stamp": "%.6f" % pose["stamp"],
                        "X-Dot-Stamp-Ns": str(pose["stamp_ns"]),
                        "X-Dot-Live": "1" if live else "0",
                        "X-Dot-Drawn": "1"})
    elif far_pose is not None:
        # Too far out (or not a number) to draw: a magenta arrow at the edge of
        # the picture, pointing from its centre towards the robot. Magenta,
        # never the dot's green, so nothing can mistake it for the robot.
        cx, cy = w / 2.0, h / 2.0
        fx, fy = finite(far_pose["x"]), finite(far_pose["y"])
        if fx is not None and fy is not None:
            # direction in metres from the picture's centre; halved first if
            # the plain difference would overflow (absurd positions only)
            mx, my = ox + (c0 + cx) * res, oy + (r0 + cy) * res
            dx, dy = fx - mx, fy - my
            if not (math.isfinite(dx) and math.isfinite(dy)):
                dx, dy = fx * 0.5 - mx * 0.5, fy * 0.5 - my * 0.5
            sc = max(abs(dx), abs(dy))
            if sc > 0:
                ux, uy = dx / sc, dy / sc
                d = math.hypot(ux, uy)
                ux, uy = ux / d, uy / d
                edge = 3                        # cells in from the border
                t = min((cx - edge) / abs(ux) if ux else float("inf"),
                        (cy - edge) / abs(uy) if uy else float("inf"))
                t = max(t, 0.0)
                tip = (int(cx + ux * t), int(cy + uy * t))
                ln = min(t, max(6.0, 0.8 / res))
                tail = (int(cx + ux * (t - ln)), int(cy + uy * (t - ln)))
                cv2.arrowedLine(img, tail, tip, C_FAR, max(1, int(0.06 / res)),
                                tipLength=0.4)
        out = dot.get("outside_m") if dot else None
        headers.update({"X-Dot-Drawn": "0",
                        "X-Dot-Outside-M": metres(out, 1) if out is not None else "nan"})

    # row 0 of an OccupancyGrid is the BOTTOM; flip so the picture is upright
    img = np.flipud(img)
    if scale > 1:
        img = cv2.resize(img, (w * scale, h * scale),
                         interpolation=cv2.INTER_NEAREST)
    ok, buf = cv2.imencode(".png", img)
    # Where the picture sits on the map: the left edge of pixel column 0, the
    # top edge of pixel row 0, and metres per picture pixel. A pixel (u, v)
    # centre is at x = left + (u + 0.5) * m, y = top - (v + 0.5) * m.
    headers.update({"X-Map-Left": "%.4f" % (ox + c0 * res),
                    "X-Map-Top": "%.4f" % (oy + r1 * res),
                    "X-Map-M-Per-Px": "%.6f" % (res / max(scale, 1)),
                    "X-Grid-N": str(n),
                    "X-Grid-Stamp": "%.6f" % g.header.stamp.to_sec()})
    rows["size_m"] = "%.1f x %.1f" % (w * res, h * res)
    return (buf.tobytes() if ok else None), rows, headers


# ----------------------------------------------------------------------------
# The browser pages. Both carry the same two status lines: the banner (red when
# tracking is lost, amber when there is no tracking data or the Jetson stops
# answering) and the correction line (amber until the first correction).
# ----------------------------------------------------------------------------
_STYLE = """
body{margin:0;background:#0e1116;color:#e8ecf2;
 font:14px/1.5 -apple-system,Segoe UI,Roboto,sans-serif;padding:14px}
h1{font-size:16px;margin:0 0 2px} h2{font-size:14.5px;margin:0 0 4px}
.s{color:#98a2b3;font-size:12.5px;margin-bottom:10px}
img{max-width:100%;image-rendering:pixelated;border:1px solid #2a3140;border-radius:8px;background:#000;min-height:60px}
.st{margin-top:6px;display:flex;gap:4px 14px;flex-wrap:wrap;font-size:12.5px;font-variant-numeric:tabular-nums}
.st b{color:#98a2b3;font-weight:600;margin-right:4px}
.off{color:#e0875f}
.k{display:flex;gap:14px;flex-wrap:wrap;margin-top:12px;color:#98a2b3;font-size:12.5px}
i{width:11px;height:11px;border-radius:2px;display:inline-block;vertical-align:-1px;margin-right:5px}
.ban{display:none;font-weight:700;font-size:17px;padding:10px 14px;border-radius:8px;margin:6px 0 10px;letter-spacing:.02em}
.ban.red{background:#b3261e;color:#fff}
.ban.amber{background:#8a5a00;color:#fff}
.corr{font-size:14px;margin:0 0 10px;padding:6px 10px;border-radius:6px;background:#1a2230}
.corr.warn{background:#3a2c0a;color:#ffd58a}
"""

_BANNER = """
<div id=lost class=ban data-state=none></div>
<div id=corr class=corr>last map correction: waiting for the Jetson</div>
"""

# Status every 0.5 s (so the banner appears and clears within about half a
# second), the picture every 1 s. Keys starting with "_" are data for this
# code, not rows to print.
_SCRIPT = """
let lastOk = Date.now();
function show(id, j){document.getElementById(id).innerHTML = Object.entries(j)
  .filter(([k,v]) => k[0] !== '_')
  .map(([k,v]) => '<span><b>'+k+'</b>'+v+'</span>').join('');}
function banner(state, text){
  const b = document.getElementById('lost');
  b.dataset.state = state;
  if(state === 'ok'){ b.style.display = 'none'; return; }
  b.className = 'ban ' + (state === 'lost' ? 'red' : 'amber');
  b.textContent = text; b.style.display = 'block';
}
async function status(){
  try{
    const j = await (await fetch('/stats.json', {cache:'no-store'})).json();
    lastOk = Date.now();
    const s = j._state || {};
    if(s.camera_down) banner('lost', s.camera_down);
    else if(s.lost) banner('lost', 'TRACKING LOST for ' + Math.round(s.lost_for_s) +
        ' s - the camera has lost its place. The dot is its last good position.');
    else if(s.tracking_stale) banner('stale', 'NO TRACKING DATA - ' + s.tracking_text);
    else banner('ok', '');
    const c = document.getElementById('corr');
    c.textContent = 'last map correction: ' + (s.corr_text || '?');
    c.className = 'corr' + (s.corrections ? '' : ' warn');
    show('st', j);
  }catch(e){
    const gone = Math.round((Date.now() - lastOk) / 1000);
    if(gone >= 2) banner('stale', 'NO ANSWER FROM THE JETSON for ' + gone +
        ' s - the picture below is frozen');
  }
}
setInterval(status, 500); status();
"""

PAGE = ("""<!doctype html><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>live map</title>
<style>""" + _STYLE + """</style>
<h1>Live occupancy map</h1>
<div class=s>picture refreshes every second, status twice a second &middot; 5 cm cells &middot;
sent as one small picture, so this works over WiFi where a forwarded window does not</div>
""" + _BANNER + """
<img id=m src="/map.png">
<div id=st class=st></div>
<div class=k>
 <span><i style="background:rgb(200,40,40)"></i>occupied</span>
 <span><i style="background:rgb(232,232,232)"></i>free (map says drivable)</span>
 <span><i style="background:rgb(55,55,55)"></i>unknown</span>
 <span><i style="background:rgb(60,200,60)"></i>robot on the corrected map (hollow = last known position)</span>
 <span><i style="background:rgb(255,0,255)"></i>arrow at the edge: robot far outside the map, not drawn (see the robot line)</span>
 <span><i style="background:rgb(255,170,0)"></i>route to drive</span>
</div>
<script>""" + _SCRIPT + """
function pic(){ document.getElementById('m').src = '/map.png?' + Date.now(); }
setInterval(pic, 1000);
</script>""").encode()


PAGE2 = ("""<!doctype html><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>live map - camera and LiDAR</title>
<style>""" + _STYLE + """
.g{display:grid;grid-template-columns:1fr 1fr;gap:14px}
@media (max-width:820px){.g{grid-template-columns:1fr}}
.g img{width:100%}
</style>
<h1>Live map - camera and LiDAR, side by side</h1>
<div class=s>pictures refresh every second, status twice a second &middot; both start at the start mark
facing right &middot; 5 cm cells, but each picture is scaled to fit its panel, so compare shapes, not sizes</div>
""" + _BANNER + """
<div class=g>
 <div><h2>Camera (ZED X) - the map being built</h2><img id=m src="/map.png"><div id=st class=st></div></div>
 <div><h2>LiDAR (Helios) - walls placed by the wheel+IMU estimate</h2><img id=p src="/peer/map.png"><div id=ps class=st></div>
  <div class=s style="margin-top:6px">A live view, not loop-corrected. The LiDAR reference used for the
  numbers is made after the drive, from the recording. The banner and correction line above are
  about the CAMERA map only.</div></div>
</div>
<div class=k>
 <span><i style="background:rgb(200,40,40)"></i>walls / occupied</span>
 <span><i style="background:rgb(232,232,232)"></i>floor seen / free</span>
 <span><i style="background:rgb(55,55,55)"></i>not seen</span>
 <span><i style="background:rgb(60,200,60)"></i>robot (camera panel: on the corrected map; hollow = last known position)</span>
 <span><i style="background:rgb(255,0,255)"></i>arrow at the edge: robot far outside the map, not drawn</span>
 <span><i style="background:rgb(255,170,0)"></i>path driven (LiDAR panel)</span>
</div>
<script>""" + _SCRIPT + """
async function pics(){
  const t = Date.now();
  document.getElementById('m').src = '/map.png?' + t;
  document.getElementById('p').src = '/peer/map.png?' + t;
  try{ const r = await fetch('/peer/stats.json',{cache:'no-store'});
       if(r.ok) show('ps', await r.json());
       else document.getElementById('ps').innerHTML='<span class=off>LiDAR view offline</span>'; }
  catch(e){ document.getElementById('ps').innerHTML='<span class=off>LiDAR view offline</span>'; }
}
setInterval(pics, 1000); pics();
</script>""").encode()


def fetch_peer(path):
    """The robot's view, passed through. Short timeout: a dead robot view must
    never stall the camera panel (the server is threaded, but be quick anyway)."""
    try:
        with urllib.request.urlopen(PEER["url"] + path, timeout=1.5) as r:
            return r.status, r.headers.get("Content-Type", ""), r.read()
    except Exception:
        return 503, "text/plain", b"LiDAR view offline"


PEER = {"url": ""}


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass                                    # do not spam the console

    def _send(self, code, ctype, body, extra=None):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Cache-Control", "no-store")
        for k, v in (extra or {}).items():
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        p = self.path.split("?")[0]
        if p in ("/", "/index.html"):
            self._send(200, "text/html; charset=utf-8",
                       PAGE2 if PEER["url"] else PAGE)
        elif p in ("/peer/map.png", "/peer/stats.json") and PEER["url"]:
            code, ctype, body = fetch_peer(p[len("/peer"):])
            self._send(code, ctype, body)
        elif p == "/map.png":
            png, _, headers = render_png(self.server.scale)
            if png is None:
                self._send(503, "text/plain", b"no map yet")
                return
            self._send(200, "image/png", png, headers)
        elif p == "/stats.json":
            # no picture drawn here: the status is cheap, the picture is not
            self._send(200, "application/json", json.dumps(status()).encode())
        else:
            self._send(404, "text/plain", b"not found")


def main():
    rospy.init_node("live_map_server", anonymous=True, disable_signals=True)
    port = int(rospy.get_param("~port", 8093))
    scale = int(rospy.get_param("~scale", 3))
    grid_topic = rospy.get_param("~grid", "/rtabmap/grid_map")
    route_csv = rospy.get_param("~route", "")
    PEER["url"] = rospy.get_param("~peer", "").rstrip("/")
    CFG["map_frame"] = rospy.get_param("~map_frame", "map")
    CFG["base_frame"] = rospy.get_param("~base_frame", "base_link")
    # the light topic: same message type and the same "lost" field, without
    # the large feature arrays (see "WHY THE _lite TOPIC" at the top)
    CFG["odom_info"] = rospy.get_param("~odom_info", "/rtabmap/odom_info_lite")
    CFG["info"] = rospy.get_param("~info", "/rtabmap/info")
    if rospy.has_param("~odom"):
        # Absent is not zero: say it is ignored rather than let a launch file
        # believe it still steers the dot.
        rospy.logwarn("~odom is no longer used: the robot is drawn from TF "
                      "%s -> %s (the corrected map), never from odometry.",
                      CFG["map_frame"], CFG["base_frame"])

    STATE["route"] = load_route(route_csv)
    if route_csv and not STATE["route"]:
        # Absent is not zero: say the route failed to load rather than drawing
        # nothing and letting it look like "no route was asked for".
        rospy.logwarn("ROUTE NOT LOADED from %r - the map will have no guide "
                      "on it. Check the path exists and has x,y columns.",
                      route_csv)
    elif STATE["route"]:
        rospy.loginfo("route guide: %d points from %s",
                      len(STATE["route"]), route_csv)

    if cv2 is None:
        rospy.logerr("cv2 is not available; cannot render the map")
        return

    # Persistent subscriptions - see the docstring. Never wait_for_message here.
    # Large receive buffers: a grid is several MB and an Info message carries
    # the whole memory state; a small buffer makes rospy fall behind.
    rospy.Subscriber(grid_topic, OccupancyGrid, grid_cb, queue_size=1,
                     buff_size=1 << 24)
    if Info is not None:
        rospy.Subscriber(CFG["odom_info"], OdomInfo, odom_info_cb,
                         queue_size=5, buff_size=1 << 20)
        # queue_size 20: every map update is counted, none may be dropped
        rospy.Subscriber(CFG["info"], Info, info_cb, queue_size=20,
                         buff_size=1 << 24)
    else:
        rospy.logerr("rtabmap_msgs is not importable: no tracking banner and "
                     "no correction count. The page will say so.")

    tf_buf = tf2_ros.Buffer(cache_time=rospy.Duration(60.0))
    tf2_ros.TransformListener(tf_buf)
    CFG["tf_buf"] = tf_buf
    from sensor_msgs.msg import PointCloud2
    rospy.Subscriber(rospy.get_param("~obstacles", "/nav/obstacles_cloud"), PointCloud2, obstacles_cb,
                     queue_size=1, buff_size=1 << 22)
    threading.Thread(target=pose_loop, args=(tf_buf,), daemon=True).start()

    srv = HTTPServer(("0.0.0.0", port), Handler)
    srv.scale = scale
    print("  live map on  http://<jetson-address>:%d/   (and any other address of"
          " this Jetson)" % port)
    print("  map picture from %s" % grid_topic)
    print("  robot from TF %s -> %s (the corrected map)" % (CFG["map_frame"],
                                                          CFG["base_frame"]))
    print("  tracking banner from %s, corrections from %s" % (CFG["odom_info"],
                                                             CFG["info"]))
    print("  route guide: %s" % (("%d points from %s" % (len(STATE["route"]), route_csv))
                                 if STATE["route"] else "none"))
    print("  LiDAR panel: %s" % (PEER["url"] or "none"))
    print("  Ctrl+C to stop")
    t = threading.Thread(target=srv.serve_forever, daemon=True)
    t.start()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    srv.shutdown()


if __name__ == "__main__":
    main()
