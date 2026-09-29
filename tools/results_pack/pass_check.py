#!/usr/bin/env python3
"""pass_check.py - the drive-4 rehearsal pass line for Jetson stalls. Round 2, registered 2026-09-25
before the rehearsal (round 1 replaced after review: its camera check could pass without a fix).

Usage (Jetson):  python3 extract_timing.py <run> /tmp/<run>_timing.npz
                 python3 pass_check.py /tmp/<run>_timing.npz ~/slam_series2/<run>.db

Plain terms: asks whether the camera's picture stream or the robot's wheel data still go quiet
during the drive (drive 3's fault), whatever the reason; checks that the camera node's own picture
loop never paused; and checks that the log could be read well enough for the answer to mean anything.

PASS needs all of:
  P1  camera tracker output (/rtabmap/odom header stamps): NO silence > 1.5 s, whatever the cause
                                                            (drive 3: 9)
  P2  wheel data (/robot/wheel_odom, received): no silence > 1.5 s with the "inbound held" signature
      (reply read >= 1 s late and more than twice the outbound delay)        (drive 3: 9)
  P3  control: camera gyroscope stream as received, longest gap < 0.1 s (drive 3: 0.031 s)
      - shows the recorder and the Jetson as a whole kept running; it is a separate thread of the
      camera process, so it does NOT show the camera's picture loop was running
  P4  'ZED Diagnostic' (sent only from inside the camera node's picture loop) never pauses > 2.0 s
                                                            (drive 3: 9 pauses of 2.89-4.19 s)
  P5  validity: map iterations parsed from mapping.log >= 95 % of the database's node count
      (drive 3: 1,088 of 1,128 = 96.5 %; 1,122 'Maps update' lines, 35 garbled by concurrent writers)
Reported, not gated: map saves >= 5.5 s (drive 3: 11), WiFi-type wheel silences.
If P3 fails the result is VOID; if P5 fails it is INVALID (fix the log, not the verdict).
"""
import sys, json, sqlite3, os
import numpy as np

z = np.load(sys.argv[1])
db = sys.argv[2] if len(sys.argv) > 2 else None


def gaps(t, thr, span=None):
    """Silences longer than thr between messages - and, if span=(start, end) of the recording is
    given, also before the first and after the last message (a stream that stops for good must
    not pass: stalls review round 2, defect 2)."""
    t = np.sort(t)
    out = [(t[k], t[k + 1]) for k in np.where(np.diff(t) > thr)[0]]
    if span is not None and len(t):
        if t[0] - span[0] > thr:
            out.insert(0, (span[0], t[0]))
        if span[1] - t[-1] > thr:
            out.append((t[-1], span[1]))
    elif span is not None:
        out.append(span)
    return out


# the recording's span, from the control stream (camera gyroscope as received, never gapped > 0.1 s)
_g = np.sort(z["zedx_front_zed_node_imu_data"][:, 1])
SPAN = (float(_g[0]), float(_g[-1]))
it = z["rtabmap_iter"]
saves = [(r[0] - r[5] - r[4], r[0] - r[5], r[4]) for r in it if r[4] >= 1.5]
odom = gaps(z["rtabmap_odom"][:, 0], 1.5, SPAN)
b = z["bridge_clock"]
t1, t2, t3, t4 = b.T
off = np.median(((t2 - t1) + (t3 - t4)) / 2)
up, dn = t2 - t1 - off, t4 - t3 + off
p2, wifi = [], []
for a, bb in gaps(z["robot_wheel_odom"][:, 1], 1.5):
    w = (t1 >= a - 1) & (t1 <= bb)
    u = up[w].max() if w.any() else 0.0
    d = dn[w].max() if w.any() else 0.0
    (p2 if (d >= 1.0 and d > 2 * u) else wifi).append((round(bb - a, 2), round(float(u), 2), round(float(d), 2)))
imu = float(np.diff(np.sort(z["zedx_front_zed_node_imu_data"][:, 1])).max())
zd = z["zed_diag_t"] if "zed_diag_t" in z.files else np.array([])
p4 = [round(float(bb - a), 2) for a, bb in gaps(zd, 2.0, SPAN)] if len(zd) > 1 else None
nodes = None
if db and os.path.exists(os.path.expanduser(db)):
    c = sqlite3.connect("file:%s?mode=ro&immutable=1" % os.path.expanduser(db), uri=True)
    nodes = c.execute("select count(*) from Node").fetchone()[0]
p5 = (len(it) >= 0.95 * nodes) if nodes else None
res = dict(P1_camera_silences_over_1p5s=[round(float(bb - a), 2) for a, bb in odom],
           P2_wheel_silences_inbound_held=p2, P3_camera_gyro_max_gap_s=round(imu, 3),
           P4_zed_diagnostic_pauses_over_2s=p4, P5_parsed_iterations=int(len(it)), P5_db_nodes=nodes,
           saves_over_5p5s=sum(1 for s in saves if s[2] >= 5.5), other_wheel_silences_over_1p5s=wifi)
if imu >= 0.1:
    v = "VOID (control failed: the recorder or the whole Jetson stalled)"
elif p4 is None:
    v = "INVALID (no ZED Diagnostic in the bag)"
elif not odom and not p2 and not p4:
    v = "PASS"
else:
    v = "FAIL"
# P5 is a WARNING only (round 2 review, defect 6): P1, P2 and P4 do not depend on the log; P5
# only protects the reported save counts
if p5 is not True:
    v += "  [warning: log coverage under 95 % or no database given - the save counts are unreliable]"
res["verdict"] = v
print(json.dumps(res, indent=1))
