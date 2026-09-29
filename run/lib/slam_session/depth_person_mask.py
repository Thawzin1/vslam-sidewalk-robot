#!/usr/bin/env python3
"""depth_person_mask.py - blank out people in the depth picture before the map sees it (Jetson, ROS 1 node).

Used live in the SLAM session (27 Sept 2026; run/lib/slam_session/). Started by slam_run.launch
when mask:=true (start_slam.sh with MASK=1).

WHAT IT DOES
    in:   /zedx_front/zed_node/depth/depth_registered   (32-bit float metres, 1920x1200, ~15 per second)
          /zedx_front/zed_node/obj_det/objects          (the ZED camera's people boxes, zed_interfaces/ObjectsStamped)
    out:  /zedx_front/zed_node/depth/depth_registered_masked   the same picture, same time stamp, with the
          depth set to 0 inside every "Person" box (enlarged by a margin). RTAB-Map subscribes to this one.

WHY THIS REMOVES PEOPLE FROM ODOMETRY, THE VOCABULARY AND THE MAP (ENGINEERING_NOTES.md section 5, "Dynamic object masking")
    On this build Vis/DepthAsMask = true and Mem/DepthAsMask = true (`rtabmap --params`, 0.21.13): picture points
    with no depth are never used as features, neither by the camera tracker (odometry) nor by place recognition
    (the vocabulary). The map's 3D points and occupancy grid are also made from depth. So depth 0 inside a
    person's box = the person does not exist for RTAB-Map. The colour picture is left untouched.
    *Plain terms: where the camera sees a person, we tell the map "nothing measured here", so the person is
    neither used to judge movement nor drawn into the map.*

    This is the box-level form of the published idea (detect the moving class, drop its pixels before
    tracking and mapping: DynaSLAM, Bescos et al. RA-L 2018; DS-SLAM, Yu et al. IROS 2018; Detect-SLAM,
    Zhong et al. WACV 2018). The ZED ROS 1 wrapper here only gives boxes, not outlines: it hard-codes
    enable_segmentation = false (zed_wrapper_nodelet.cpp:2011), so the wall behind a person inside the box is
    blanked too - a cost in features, never a wrong point.

TIMING
    A box is used for a depth picture only if its message is at most --max-age (0.35 s) older or 0.1 s newer
    than the picture. Boxes come with the camera's own grab time stamp, so normally they match exactly.
    No fresh box message = the depth passes through unchanged and the status line says "DETECTOR STALE"
    (mapping never stops because the detector did).

OUTPUT FOR THE RECORD (every 10 s): ~/jobs/<run>_mask.progress (jobs page, rule 13) and
    ~/.run_records/<run>/mask_stats.csv (time, pictures in, pictures with people, % of pixels blanked, worst
    delay added, detector age). Per-person events go to mask_events.csv (time, id, box, confidence).

  usage (normally from slam_run.launch):  rosrun-free:  python3 depth_person_mask.py _run:=s2_slam_01
  test:  python3 depth_person_mask.py --selftest     (known-answer test, no ROS master needed)
"""
from __future__ import print_function
import os, sys, time, threading
import numpy as np

MARGIN_FRAC = 0.10   # box grown by 10 % of its width/height on each side ...
MARGIN_PX = 16       # ... plus 16 pixels (feet, hair, a swinging arm)


def is_person(o):
    return str(getattr(o, "label", "")).upper() == "PERSON" or str(getattr(o, "sublabel", "")).upper() == "PERSON"


def boxes_to_mask_rects(objs, w, h, scale_x, scale_y, min_conf, margin_frac=MARGIN_FRAC, margin_px=MARGIN_PX,
                        with_objects=False):
    """[(x0, y0, x1, y1)] pixel rectangles (inclusive-exclusive) to blank, from ZED objects.
    with_objects=True returns [(rect, object)] instead."""
    rects = []
    for o in objs:
        # the SDK sends label "PERSON" (sublabel "Person"); s2_slam_01 compared label to "Person" and masked nothing
        if not is_person(o) or o.confidence < min_conf:
            continue
        xs = [c.kp[0] for c in o.bounding_box_2d.corners]
        ys = [c.kp[1] for c in o.bounding_box_2d.corners]
        if not xs or max(xs) <= min(xs) or max(ys) <= min(ys):
            continue
        x0, x1 = min(xs) * scale_x, max(xs) * scale_x
        y0, y1 = min(ys) * scale_y, max(ys) * scale_y
        mx = (x1 - x0) * margin_frac + margin_px
        my = (y1 - y0) * margin_frac + margin_px
        r = (int(max(0, x0 - mx)), int(max(0, y0 - my)), int(min(w, x1 + mx + 1)), int(min(h, y1 + my + 1)))
        rects.append((r, o) if with_objects else r)
    return rects


def apply_rects(arr, rects):
    for x0, y0, x1, y1 in rects:
        arr[y0:y1, x0:x1] = 0
    return arr


def selftest():
    """Known answer: one Person box, one Chair box, one low-confidence Person, on a synthetic picture."""
    class KP(object):
        def __init__(s, x, y): s.kp = [x, y]

    class BB(object):
        def __init__(s, x0, y0, x1, y1): s.corners = [KP(x0, y0), KP(x1, y0), KP(x1, y1), KP(x0, y1)]

    class Obj(object):
        def __init__(s, label, conf, bb): s.label, s.confidence, s.bounding_box_2d = label, conf, bb
    w, h = 1920, 1200
    d = np.full((h, w), 2.5, np.float32)
    objs = [Obj("PERSON", 80, BB(900, 300, 1000, 900)), Obj("CHAIR", 90, BB(100, 100, 200, 200)),
            Obj("PERSON", 30, BB(1500, 100, 1600, 400))]
    rects = boxes_to_mask_rects(objs, w, h, 1.0, 1.0, 50)
    ok = len(rects) == 1
    x0, y0, x1, y1 = rects[0]
    # expected: 900 - (100*0.1+16)=874 ; 1000+26+1=1027 ; y: 300-(600*0.1+16)=224 ; 900+76+1=977
    ok = ok and (x0, y0, x1, y1) == (874, 224, 1027, 977)
    apply_rects(d, rects)
    blank = int((d == 0).sum())
    ok = ok and blank == (1027 - 874) * (977 - 224)
    ok = ok and d[150, 150] == 2.5 and d[200, 1550] == 2.5 and d[600, 950] == 0 and d[600, 873] == 2.5
    # half-resolution depth with full-resolution boxes: scale 0.5
    rects2 = boxes_to_mask_rects(objs[:1], 960, 600, 0.5, 0.5, 50)
    # box 450-500 x 150-450 at half resolution; margin 10 % + 16 px of THAT picture: x +-21, y +-46
    ok = ok and rects2 == [(429, 104, 522, 497)]
    print("SELFTEST %s: rect %s, %d pixels blanked (%.2f %% of the picture); chair and low-confidence person left alone;"
          " half-resolution rect %s" % ("PASS" if ok else "FAIL", rects[0], blank, 100.0 * blank / (w * h), rects2))
    return 0 if ok else 1


def main():
    import rospy
    from sensor_msgs.msg import Image
    from zed_interfaces.msg import ObjectsStamped
    rospy.init_node("depth_person_mask")
    run = rospy.get_param("~run", "unnamed")
    din = rospy.get_param("~depth_in", "/zedx_front/zed_node/depth/depth_registered")
    dout = rospy.get_param("~depth_out", "/zedx_front/zed_node/depth/depth_registered_masked")
    otopic = rospy.get_param("~objects", "/zedx_front/zed_node/obj_det/objects")
    min_conf = float(rospy.get_param("~min_confidence", 50.0))
    max_age = float(rospy.get_param("~max_age", 0.35))
    od_w = float(rospy.get_param("~od_width", 1920.0))    # the detector's box coordinates: camera resolution
    od_h = float(rospy.get_param("~od_height", 1200.0))
    rec = os.path.join(os.path.expanduser(os.environ.get("RECORDS_DIR", "~/.run_records")), run)
    if not os.path.isdir(rec):
        os.makedirs(rec)
    prog = os.path.expanduser("~/jobs/%s_mask.progress" % run)
    stats_f = open(os.path.join(rec, "mask_stats.csv"), "a")
    if stats_f.tell() == 0:
        stats_f.write("t_epoch,pictures,with_people,people_boxes,pct_pixels_blanked_mean,worst_added_ms,detector_age_s\n")
    ev_f = open(os.path.join(rec, "mask_events.csv"), "a")
    if ev_f.tell() == 0:
        ev_f.write("t_depth_stamp,instance_id,confidence,x0,y0,x1,y1,pct_pixels\n")

    lock = threading.Lock()
    hist = []          # recent (stamp_s, objects) - newest last
    st = {"pics": 0, "with": 0, "boxes": 0, "pct": [], "worst": 0.0, "last_obj_wall": 0.0,
          "tot_pics": 0, "tot_with": 0, "t0": time.time()}
    pub = rospy.Publisher(dout, Image, queue_size=2, tcp_nodelay=True)

    def ocb(m):
        with lock:
            hist.append((m.header.stamp.to_sec(), m.objects))
            del hist[:-30]
            st["last_obj_wall"] = time.time()

    def dcb(m):
        t_in = time.time()
        ts = m.header.stamp.to_sec()
        with lock:
            cand = [(abs(ts - s), o) for s, o in hist if -0.1 <= s - ts <= 0.1 or 0 <= ts - s <= max_age]
        objs = min(cand, key=lambda c: c[0])[1] if cand else []
        ro = boxes_to_mask_rects(objs, m.width, m.height, m.width / od_w, m.height / od_h, min_conf,
                                 with_objects=True)
        rects = [r for r, _ in ro]
        if rects and m.encoding == "32FC1":
            arr = np.frombuffer(m.data, dtype=np.float32).reshape(m.height, m.width).copy()
            apply_rects(arr, rects)
            pct = min(100.0, 100.0 * sum((x1 - x0) * (y1 - y0) for x0, y0, x1, y1 in rects) / arr.size)  # box area
            out = Image(header=m.header, height=m.height, width=m.width, encoding=m.encoding,
                        is_bigendian=m.is_bigendian, step=m.step, data=arr.tobytes())
            pub.publish(out)
            for (x0, y0, x1, y1), o in ro:
                ev_f.write("%.3f,%d,%.0f,%d,%d,%d,%d,%.2f\n" % (ts, o.instance_id, o.confidence, x0, y0, x1, y1,
                                                             100.0 * (x1 - x0) * (y1 - y0) / arr.size))
            with lock:
                st["with"] += 1; st["tot_with"] += 1; st["boxes"] += len(rects); st["pct"].append(pct)
        else:
            pub.publish(m)            # nothing to blank: the SAME message, no copy
        with lock:
            st["pics"] += 1; st["tot_pics"] += 1
            st["worst"] = max(st["worst"], (time.time() - t_in) * 1000.0)

    rospy.Subscriber(otopic, ObjectsStamped, ocb, queue_size=10)
    rospy.Subscriber(din, Image, dcb, queue_size=2, buff_size=2 ** 26, tcp_nodelay=True)
    rospy.loginfo("depth_person_mask: %s + %s -> %s (min confidence %.0f, max age %.2f s)", din, otopic, dout,
                  min_conf, max_age)
    pidfile = os.path.join(rec, "rtabmap.pid")     # written by the staged start_drive.sh once the map runs
    while not rospy.is_shutdown():
        time.sleep(10)
        try:                                        # stop with THIS session's map (whole-name check, rule 8)
            pid = int(open(pidfile).read().split()[0])
            if open("/proc/%d/comm" % pid).read().strip() != "rtabmap":
                raise OSError("not rtabmap")
        except (IOError, OSError, ValueError, IndexError) as e:
            if os.path.exists(pidfile):
                with open(prog, "w") as fh:
                    fh.write("MASK complete - the map (rtabmap) has exited; %d/%d pictures had people\n"
                             % (st["tot_with"], st["tot_pics"]))
                rospy.signal_shutdown("map exited")
                break
        with lock:
            age = time.time() - st["last_obj_wall"] if st["last_obj_wall"] else float("inf")
            pm = float(np.mean(st["pct"])) if st["pct"] else 0.0
            stats_f.write("%.1f,%d,%d,%d,%.2f,%.1f,%.1f\n" % (time.time(), st["pics"], st["with"], st["boxes"], pm,
                                                           st["worst"], age if age != float("inf") else -1))
            stats_f.flush(); ev_f.flush()
            state = "DETECTOR STALE (no boxes %s) - depth passed through unmasked" % (
                "ever" if age == float("inf") else "for %.0f s" % age) if age > 2.0 else "masking"
            line = "MASK %s  %d/%d pictures had people  %.1f pics/s  worst +%.0f ms  %ds" % (
                state, st["tot_with"], st["tot_pics"], st["pics"] / 10.0, st["worst"], time.time() - st["t0"])
            st["pics"] = 0; st["with"] = 0; st["boxes"] = 0; st["pct"] = []; st["worst"] = 0.0
        try:
            with open(prog + ".tmp", "w") as fh:
                fh.write(line + "\n")
            os.rename(prog + ".tmp", prog)
        except OSError:
            pass


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        sys.exit(selftest())
    main()
