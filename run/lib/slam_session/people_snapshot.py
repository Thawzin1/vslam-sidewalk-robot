#!/usr/bin/env python3
"""Live proof of the people mask during the SLAM session.
When the ZED detector reports a PERSON, save one picture pair at most every 5 s into <run folder>/people/:
  <time>_left.jpg   the camera picture with the person boxes drawn
  <time>_depth.png  the MASKED depth that RTAB-Map receives (black = removed)
and append a line to people_events.csv. Light: nothing is kept between events. Run with the run name."""
import os, sys, time, csv, rospy, numpy as np, cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
RUN = sys.argv[1]
OUT = os.path.expanduser("~/.run_records/%s/people" % RUN); os.makedirs(OUT, exist_ok=True)
NS = "/zedx_front/zed_node"
br = CvBridge(); last = {"left": None, "depth": None}; t_last = [0.0]
try:
    from zed_interfaces.msg import ObjectsStamped
except Exception:
    ObjectsStamped = None
def on_left(m): last["left"] = m
def on_depth(m): last["depth"] = m
def on_objects(m):
    persons = [o for o in m.objects if str(getattr(o, "label", "")).lower().startswith("person")]
    if not persons or time.time() - t_last[0] < 5 or last["left"] is None or last["depth"] is None:
        return
    t_last[0] = time.time(); ts = time.strftime("%H%M%S")
    img = br.imgmsg_to_cv2(last["left"], "bgr8").copy()
    h, w = img.shape[:2]
    for o in persons:                               # same corner handling as depth_person_mask.py person_rects()
        xs = [c.kp[0] for c in o.bounding_box_2d.corners]; ys = [c.kp[1] for c in o.bounding_box_2d.corners]
        if not xs or max(xs) <= min(xs):
            continue
        sx = w / 1920.0 if max(xs) > w else 1.0; sy = h / 1200.0 if max(ys) > h else 1.0
        cv2.rectangle(img, (int(min(xs) * sx), int(min(ys) * sy)), (int(max(xs) * sx), int(max(ys) * sy)), (0, 0, 255), 3)
    cv2.imwrite(os.path.join(OUT, ts + "_left.jpg"), cv2.resize(img, (960, 600)))
    d = br.imgmsg_to_cv2(last["depth"], "passthrough").astype(np.float32)
    d = np.nan_to_num(d, nan=0.0, posinf=0.0); vis = (np.clip(d / 6.0, 0, 1) * 255).astype(np.uint8)
    vis = cv2.applyColorMap(vis, cv2.COLORMAP_TURBO); vis[d <= 0] = 0
    cv2.imwrite(os.path.join(OUT, ts + "_depth.png"), cv2.resize(vis, (960, 600)))
    blank = float((d <= 0).mean())
    with open(os.path.join(OUT, "people_events.csv"), "a") as f:
        csv.writer(f).writerow([time.strftime("%F %T"), len(persons), "%.3f" % blank])
if __name__ == "__main__":
    rospy.init_node("people_snapshot", anonymous=True)
    rospy.Subscriber(NS + "/left/image_rect_color", Image, on_left, queue_size=1, buff_size=2 ** 24)
    rospy.Subscriber(NS + "/depth/depth_registered_masked", Image, on_depth, queue_size=1, buff_size=2 ** 24)
    if ObjectsStamped is None:
        sys.exit("zed_interfaces not importable - no snapshots")
    rospy.Subscriber(NS + "/obj_det/objects", ObjectsStamped, on_objects, queue_size=1)
    rospy.spin()
