#!/usr/bin/env python3
"""fake_person_test.py - LIVE known-answer test of the people mask, with no person present (Jetson, bench only).

Publishes a made-up "Person" box on the detector's topic, stamped exactly like each camera picture, for
--seconds, while the real detector is switched OFF (enable_object_detection false). Then takes the masked depth
pictures and checks: inside the made-up box (+ its margin) every pixel is 0; outside it the picture is untouched
(same count of valid depth values as the raw picture of the same time stamp, within the box area).
Also a NEGATIVE control first: with no box published, the masked picture must equal the raw one.

*Plain terms: pretend a person stands in the middle of the picture, and check the map is given a picture with
exactly that rectangle blanked - and nothing blanked when nobody is there.*

  usage (bench, mask node running):  python3 fake_person_test.py --seconds 20 --out result.json
"""
from __future__ import print_function
import argparse, json, threading, time
import numpy as np
import rospy
from sensor_msgs.msg import Image, CameraInfo
from zed_interfaces.msg import ObjectsStamped, Object

BOX = (800, 300, 1100, 1000)       # x0, y0, x1, y1 in camera pixels (1920x1200)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=20)
    ap.add_argument("--out", required=True)
    a = ap.parse_args(rospy.myargv()[1:])
    rospy.init_node("fake_person_test", anonymous=True, disable_signals=True)
    pub = rospy.Publisher("/zedx_front/zed_node/obj_det/objects", ObjectsStamped, queue_size=5)
    raw, masked = {}, {}
    lock = threading.Lock()
    state = {"publish": False, "sent": 0}

    def info_cb(m):
        if not state["publish"]:
            return
        o = Object(label="Person", label_id=0, instance_id=999, confidence=90.0)
        x0, y0, x1, y1 = BOX
        for c, (x, y) in zip(o.bounding_box_2d.corners, ((x0, y0), (x1, y0), (x1, y1), (x0, y1))):
            c.kp = [x, y]
        msg = ObjectsStamped(objects=[o])
        msg.header = m.header
        pub.publish(msg)
        state["sent"] += 1

    def keep(store, m):
        with lock:
            store[m.header.stamp.to_nsec()] = m
            if len(store) > 12:
                del store[min(store)]
    rospy.Subscriber("/zedx_front/zed_node/rgb/camera_info", CameraInfo, info_cb, queue_size=5)
    rospy.Subscriber("/zedx_front/zed_node/depth/depth_registered", Image, lambda m: keep(raw, m),
                     queue_size=2, buff_size=2 ** 26)
    rospy.Subscriber("/zedx_front/zed_node/depth/depth_registered_masked", Image, lambda m: keep(masked, m),
                     queue_size=2, buff_size=2 ** 26)

    def compare(expect_box):
        with lock:
            common = sorted(set(raw) & set(masked))[-5:]
            pairs = [(raw[k], masked[k]) for k in common]
        out = []
        for r, m in pairs:
            R = np.frombuffer(r.data, np.float32).reshape(r.height, r.width)
            M = np.frombuffer(m.data, np.float32).reshape(m.height, m.width)
            sel = np.zeros(R.shape, bool)
            if expect_box:
                x0, y0, x1, y1 = BOX
                mx, my = (x1 - x0) * 0.1 + 16, (y1 - y0) * 0.1 + 16
                sel[int(y0 - my):int(y1 + my + 1), int(x0 - mx):int(x1 + mx + 1)] = True
            inside_zero = bool((M[sel] == 0).all()) if expect_box else None
            same_outside = bool(np.array_equal(np.nan_to_num(R[~sel], nan=-1), np.nan_to_num(M[~sel], nan=-1)))
            out.append({"stamp": r.header.stamp.to_sec(), "inside_all_zero": inside_zero,
                        "outside_identical": same_outside, "box_pixels": int(sel.sum())})
        return out
    time.sleep(6)
    neg = compare(False)
    state["publish"] = True
    time.sleep(a.seconds)
    pos = compare(True)
    state["publish"] = False
    ok = bool(neg) and all(p["outside_identical"] for p in neg) and bool(pos) and \
        all(p["inside_all_zero"] and p["outside_identical"] for p in pos)
    res = {"PASS": ok, "negative_control_no_box": neg, "made_up_box": pos, "boxes_sent": state["sent"], "box": BOX}
    json.dump(res, open(a.out, "w"), indent=1)
    print("FAKE-PERSON TEST %s: negative control %d pictures identical=%s; with box %d pictures, inside all 0=%s, "
          "outside identical=%s; %d boxes sent" % ("PASS" if ok else "FAIL", len(neg),
          [p["outside_identical"] for p in neg], len(pos), [p["inside_all_zero"] for p in pos],
          [p["outside_identical"] for p in pos], state["sent"]))


if __name__ == "__main__":
    main()
