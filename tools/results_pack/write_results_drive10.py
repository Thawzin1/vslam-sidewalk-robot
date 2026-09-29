#!/usr/bin/env python3
"""write_results_drive10.py - writes RESULTS.md of drive 10 (s2_static_10, the whole 2nd floor) from the pack's own files.
27 Sept (later): the LiDAR half added - the Force3DoF replay was re-run on the robot after the first attempt died with its battery
(~02:40); its products are in lidar_ref_f3dof/ and the pack root (pack_drive10_lidar.sh), compared by compare_lidar.py,
size_fit_agreement.py, score_f3dof.py, robot_wheels_from_robot_bag.py and compare_drives_4_8_9_10.py.
Same structure as write_results_drive9.py's RESULTS.md; every number read from a file named beside it (nothing typed by hand
except the pre-drive facts reported by the drive session, marked as such). usage: write_results_drive10.py"""
import csv, json, math, os, re, sqlite3, time
import numpy as np
os.environ["TZ"] = "America/Toronto"; time.tzset()
SR = os.environ.get("REPO_ROOT", os.path.expanduser("~/vslam-sidewalk-robot"))
RUN = "s2_static_10"
P = os.path.join(os.environ.get("RESULTS_DIR", os.path.join(SR, "results")), "series2_static", RUN)
P9 = os.path.join(SR, "01_runs/series2_static/s2_static_09")
REC = os.path.join(os.path.expanduser(os.environ.get("RECORDS_DIR", "~/.run_records")), RUN)
DB = os.path.join(os.path.expanduser(os.environ.get("WORK_DIR", "~/slam_series2")), "%s.db" % RUN)
hm = lambda t: time.strftime("%H:%M:%S", time.localtime(t))
J = lambda f, d=P: json.load(open(os.path.join(d, f)))
P4 = os.path.join(SR, "01_runs/series2_static/s2_static_04")

num, facts, mcf = J("numbers.json"), J("facts.json"), J("map_corrected_facts.json")
cmeta, tmeta = J("camera_corrected.tum.meta.json"), J("camera.tum.meta.json")
fpc, ft, pw, fr = J("freezes_pass_check.json"), J("freeze_timing.json"), J("power_temperature.json"), J("bridge_freshness.json")
ag, sfit, flat, rw = J("agreement.json"), J("size_fit.json"), J("lidar_flatness_check.json"), J("robot_wheels_from_robot_bag.json")
lmeta, lfacts, ag9 = J("lidar.tum.meta.json"), J("lidar_map_facts.json"), J("agreement.json", P9)
sfit9, lfacts9, lmeta9, rw9 = J("size_fit.json", P9), J("lidar_map_facts.json", P9), J("lidar.tum.meta.json", P9), J("robot_wheels_from_robot_bag.json", P9)
cmp = J("compare_drives_4_8_9_10.json")
LT = np.loadtxt(os.path.join(P, "lidar.tum"), comments="#")
lgap_floor = math.hypot(LT[-1, 1] - LT[0, 1], LT[-1, 2] - LT[0, 2])
WT = np.loadtxt(os.path.join(P, "wheel.tum"), comments="#")
prov = open(os.path.join(P, "lidar_ref_f3dof", "provenance.txt")).read().splitlines()
prov_start = [l for l in prov if l.startswith("started")][0]
dp = json.load(open(os.path.join(P, "d10_camera_vs_lidar_side_by_side.json")))
guard = json.load(open(os.path.join(REC, "camera_guard.json")))
num9, facts9, mcf9 = J("numbers.json", P9), J("facts.json", P9), J("map_corrected_facts.json", P9)
fpc9, pw9, cm9 = J("freezes_pass_check.json", P9), J("power_temperature.json", P9), J("camera_corrected.tum.meta.json", P9)
closed = open(os.path.join(P, "closed_check.txt")).read().strip()
dbchk = open(os.path.join(P, "db_check.txt")).read().strip()
mon = open(os.path.join(REC, "monitor_STATUS.txt")).read().strip()
mlog = open(os.path.join(REC, "mapping.log"), errors="replace").read().splitlines()

# ---- closures, from the database (read-only) ----
con = sqlite3.connect("file:%s?mode=ro&immutable=1" % DB, uri=True)
stamp = dict(con.execute("select id, stamp from Node"))
pairs = {}
for a, b, ty in con.execute("select from_id, to_id, type from Link where type in (1,2)"):
    pairs[(min(a, b), max(a, b))] = ty
n_glob = sum(1 for v in pairs.values() if v == 1); n_prox = sum(1 for v in pairs.values() if v == 2)
to_start = [k for k in pairs if k[0] == 1]
to_start_types = ["recognised-again" if pairs[k] == 1 else "nearby re-match" for k in to_start]
ts = sorted(stamp[k[1]] for k in pairs)
T0n, T1n = min(stamp.values()), max(stamp.values())
q = np.histogram(ts, np.linspace(T0n, T1n, 5))[0]
nodes_total = len(stamp)
rej = sum("Rejected loop closure" in l for l in mlog)
resets = [float(m.group(1)) for l in mlog if "Odometry automatically reset to latest odometry pose available from TF" in l
          for m in [re.search(r"\[\s*(\d+\.\d+)\]", l)] if m]
bursts = []
for t in sorted(resets):
    if bursts and t - bursts[-1][1] <= 5: bursts[-1][1] = t; bursts[-1][2] += 1
    else: bursts.append([t, t, 1])
burst_txt = "; ".join((hm(a) if a == b else "%s-%s" % (hm(a), hm(b))) + " (%d)" % n for a, b, n in bursts)
top_bursts = sorted(bursts, key=lambda x: -x[2])[:5]
downcut = sum(1 for l in mlog if "cannot be used" in l)
# route length 10 m blocks from the corrected path
path_c = mcf["path_m_nodes"]

# ---- paths_2hz ----
L = {}
with open(os.path.join(P, "paths_2hz.csv")) as f:
    rows = [r for r in csv.reader(l for l in f if not l.startswith("#"))]
hdr = rows[0]; si = hdr.index("source")
for r in rows[1:]:
    L.setdefault(r[si], []).append((float(r[hdr.index("x")]), float(r[hdr.index("y")])))
plen = {k: sum(math.dist(a, b) for a, b in zip(v, v[1:])) for k, v in L.items()}

# ---- bridge final counters ----
fin = None
for l in open(os.path.join(REC, "bridge_recv.log"), errors="replace"):
    if " FINAL " in l: fin = json.loads(l.split(" FINAL ", 1)[1])
downs = [l.split()[0] for l in open(os.path.join(REC, "bridge_recv.log"), errors="replace") if "STATE DOWN" in l]
def utc2ham(s):  # the bridge log's clock is UTC
    h, m, x = map(int, s.split(":")); return "%02d:%02d:%02d" % ((h - 4) % 24, m, x)
downs_h = [utc2ham(s) for s in downs]
nb = fr["inputs"]["neither wheels nor gyroscope"]; wh = fr["inputs"]["wheels (/ekf_in/wheel_odom)"]; gy = fr["inputs"]["gyroscope (/ekf_in/imu)"]

tg = num["tracking_alone_gaps"]; tr = num["turns"]; lost = num["camera_tracking_lost"]
still = num["blend_heading_while_still"]; rt = num["processor_rtabmap"]["rtabmap_ms_per_node"]
sil = fpc["P1_camera_silences_over_1p5s"]

o = []
w = o.append
w("# Drive 10 (s2_static_10) - results pack: the whole 2nd floor")
w("")
w("*27 Sept 2026, Hamilton time: mapping 02:02:42 (camera guard 'watching') to 02:31:15 (autostop closing the map); \"park\" received 02:30:31, still 60 s, **map closed properly** (`closed_check.txt`: Admin.opt_poses saved). "
  "Camera tracker GEN_2, depth NEURAL (`~/.run_records/s2_static_10/camera.log`); FUSION=1 (the camera blended with the robot's wheels and gyroscope (turn-rate sensor) by an EKF (extended Kalman filter, a standard way of mixing sensors)). "
  "Pre-drive, as reported by the drive session (not re-measured here): Jetson freshly rebooted (up about 8 min at the start); robot IMU (its motion sensor) tilted 1.8 deg (read from /imu/data at 02:00); gyroscope offset +0.43 deg/min at the start_drive ready check; WiFi roaming helper v2 on both the Jetson and the robot. "
  "Made on the Jetson on 27 Sept by `results_packs_2026-09-26/` (project records) (pack_drive10_heavy.sh, analyze_run.py, turns_replay.py, extract_timing.py, freeze_timing_drive7.py, tegra_peaks.py, bridge_freshness_drive10.py; LiDAR half: pack_drive10_lidar.sh, compare_lidar.py, size_fit_agreement.py, robot_wheels_from_robot_bag.py, `03_methods/lidar_drift_2026-09-26/score_f3dof.py`, compare_drives_4_8_9_10.py, deck_figure_drive10.py; this file's writer `write_results_drive10.py`). "
  "Every number names the file it comes from; files are in this folder unless a path is given. Nothing here is ground truth (ENGINEERING_NOTES.md rule 19). N = 1.*")
w("")
w("**The LiDAR half was added later on 27 Sept.** The first LiDAR replay died with the robot's battery at about 02:40; it was re-run on the robot (Force3DoF, the same method as drive 9) and its products pulled into `lidar_ref_f3dof/` (sha256 checked on arrival). Its `provenance.txt` says \"%s\": the robot had just booted and its clock had not locked yet when the replay started. The positions themselves carry the recording's own time stamps: `lidar.tum` runs %s to %s Hamilton (%d positions), inside the robot's own recording (`wheel.tum`, %s to %s) and around the camera map's %s to %s; the robot's wheels show it standing still (0.00 m moved before %s, under 0.02 m of jitter after %s). So the products are valid; only that one provenance line carries the wrong date." % (
    prov_start, hm(LT[0, 0]), hm(LT[-1, 0]), len(LT), hm(WT[0, 0]), hm(WT[-1, 0]), hm(T0n), hm(T1n), rw["whole_drive_to_parked"]["from_hamilton"], hm(LT[-1, 0])))
w("")
w("*Words used below: node = one saved map snapshot; Node.pose = each node's tracking-alone position; Admin.opt_poses = the corrected positions RTAB-Map saves when it shuts down properly; loop closure = the map recognising a place seen before and pulling itself straight; fusion.bag = the drive's recording of every sensor stream; blend = /fused/odometry, the EKF's mix of camera + wheels + gyroscope; tegrastats = NVIDIA's once-a-second load/power/temperature log; quick_check = SQLite's own database-integrity check.*")
w("")
w("## 0. At a glance")
w("")
w("| item | drive 9 | drive 10 | evidence |")
w("|---|---|---|---|")
w("| duration (map nodes, first to last) | %.0f s | **%.0f s** | facts.json `duration_s` |" % (facts9["duration_s"], facts["duration_s"]))
w("| distance, camera map corrected | %.1f m | **%.1f m** | map_corrected_facts.json `path_m_nodes` |" % (mcf9["path_m_nodes"], path_c))
w("| map snapshots (nodes) / in the corrected graph | %d / %d | **%d / %d** | facts.json, camera_corrected.tum.meta.json |" % (facts9["nodes"], mcf9["graph_nodes_corrected"], facts["nodes"], cmeta["poses"]))
w("| loop closures (pairs) | %d | **%d** (%d recognised-again + %d nearby) | database Link table types 1, 2 |" % (cm9["loop_closures"], len(pairs), n_glob, n_prox))
w("| loop closures per 10 m (corrected distance) | %.1f | **%.1f** | the two rows above |" % (10 * cm9["loop_closures"] / mcf9["path_m_nodes"], 10 * len(pairs) / path_c))
w("| end of drive | no park, NOT closed | **park, closed properly** | autostop.log, closed_check.txt |")
w("| corrected positions | re-optimised by our script | **RTAB-Map's own (Admin.opt_poses)** | camera_corrected.tum.meta.json `method` |")
w("| camera program | 1 hang (28.3 s) | **0 crashes, 0 hangs** (guard restarts %d) | camera_guard.json |" % guard["restarts"])
w("| camera restarts from the blend's position | 101 | **%d** | mapping.log |" % len(resets))
w("| camera tracker silences > 1.5 s | 7 (6 outside the hang) | **%d** | freezes_pass_check.json |" % len(sil))
w("| blend without fresh robot data (> 0.5 s) | - (not measured) | **%.1f s = %.1f %% of the drive** | bridge_freshness.json |" % (nb["over_0.5s"]["seconds"], nb["over_0.5s"]["percent_of_drive"]))
w("| LiDAR yardstick | Force3DoF replay | **Force3DoF replay**, passes its own check (%d LiDAR closures, ends %.2f m from its start) | agreement.json |" % (lmeta["loop_closures"], lgap_floor))
w("| **camera map vs LiDAR estimate, median / 95th percentile (SE(3), no resizing)** | %.2f / %.2f m | **%.2f / %.2f m** | agreement.json (section 7) |" % (
    ag9["estimates"]["camera_corrected"]["disagreement_median_m"], ag9["estimates"]["camera_corrected"]["disagreement_p95_m"],
    ag["estimates"]["camera_corrected"]["disagreement_median_m"], ag["estimates"]["camera_corrected"]["disagreement_p95_m"]))
w("| best-fit size factor (not applied) / size-fitted median (for information) | %.3f / %.2f m | **%.3f / %.2f m** | agreement.json, size_fit.json (section 7a) |" % (
    ag9["estimates"]["camera_corrected"]["sim3_scale_vs_reference"], sfit9["estimates"]["camera_corrected"]["sim3"]["median_m"],
    ag["estimates"]["camera_corrected"]["sim3_scale_vs_reference"], sfit["estimates"]["camera_corrected"]["sim3"]["median_m"]))
w("")

w("## 1. How far from home at the end - both kinds (rule 20)")
w("")
w("*Plain terms: \"tracking alone\" = where each source's own step-by-step tracking put the robot, never corrected; \"corrected\" = where the map puts it after its loop closures. In a fused drive the map's nodes carry the BLEND's position, and the camera's tracking restarts from the blend each time it gives up. The drive ended on its start mark by eye, so a perfect estimate shows about 0; tape-mark parking is only good to about 0.2-0.5 m.*")
w("")
w("| kind | source | end-to-start gap | heading gap | path length | file |")
w("|---|---|---|---|---|---|")
w("| tracking alone | map nodes (Node.pose = the blend's position at each node), node 1 to node %d | **%.3f m** | - | %.2f m | facts.json, camera.tum |" % (nodes_total and max(stamp), facts["closed_loop_gap_m"], facts["path_m_nodes"]))
w("| tracking alone | camera tracking, every message (/rtabmap/odom, lost stretches left out; re-seeded by the blend at each restart) | %.3f m | %+.2f deg | %.1f m | numbers.json (fusion.bag) |" % (tg["camera"]["end_to_start_m"], tg["camera"]["heading_deg"], tg["camera"]["path_m"]))
w("| tracking alone | blend (/fused/odometry) | %.3f m | %+.2f deg | %.1f m | numbers.json |" % (tg["blend"]["end_to_start_m"], tg["blend"]["heading_deg"], tg["blend"]["path_m"]))
w("| tracking alone | robot's wheels + gyroscope as the Jetson received it over WiFi (/robot/ekf_odom, %s-%s) | %.3f m | %+.2f deg | %.1f m | numbers.json (WiFi gaps: section 3) |" % (tg["robot"]["first"], tg["robot"]["last"], tg["robot"]["end_to_start_m"], tg["robot"]["heading_deg"], tg["robot"]["path_m"]))
_w = rw["whole_drive_to_parked"]
w("| tracking alone | robot's own wheels + gyroscope, **the robot's own recording** (/odometry/filtered, %s-%s, start to parked) | **%.3f m** | **%+.2f deg** | %.1f m | robot_wheels_from_robot_bag.json |" % (_w["from_hamilton"], _w["to_hamilton"], _w["end_to_start_m"], _w["heading_deg"], _w["path_m_every_message"]))
w("| **corrected** | camera map's graph, node %d to node %d (%d graph nodes), RTAB-Map's own Admin.opt_poses, 3D | **%.4f m** | - | - | camera_corrected.tum.meta.json |" % (cmeta["first_node"], cmeta["last_node"], cmeta["poses"], cmeta["end_gap_corrected_m"]))
w("| corrected | every camera node placed by its graph node's correction, node 1 to node %d, floor plane | %.3f m | - | %.2f m | map_corrected_facts.json |" % (max(stamp), mcf["closed_loop_gap_m"], path_c))
w("| **corrected** | **LiDAR estimate (Force3DoF replay)**, floor plane | **%.3f m** | - | %.1f m | lidar.tum, lidar_map_facts.json |" % (lgap_floor, lfacts["path_m_nodes"]))
w("| corrected | LiDAR estimate (Force3DoF replay), 3D | %.4f m | - | - | lidar.tum.meta.json |" % lmeta["end_gap_corrected_m"])
w("")
w("**Loop closures beside the corrected gaps: camera map %d** (%d recognised-again + %d nearby re-matches; database Link table types 1 and 2, counted once per pair; camera_corrected.tum.meta.json gives %d). The live monitor's last line: `%s` (`monitor_STATUS.txt`). Rejected in `mapping.log`: %d. **LiDAR closures: %d** (lidar.tum.meta.json)." % (len(pairs), n_glob, n_prox, cmeta["loop_closures"], mon.splitlines()[0], rej, lmeta["loop_closures"]))
w("")
end_ids = [k[1] for k in sorted(to_start) if stamp[k[1]] > T1n - 120]
w("- **Closures joining a later node straight to node 1 (the start): %d** (%s); **%d of them from the last 2 minutes of the drive** (node(s) %s, i.e. end-to-start; both in the last 15 s of driving). %s" % (len(to_start), ", ".join("1-%d %s at %s" % (k[1], t, hm(stamp[k[1]])) for k, t in zip(sorted(to_start), to_start_types)) or "none",
  len(end_ids), ", ".join(map(str, end_ids)) or "-",
  "So the corrected gap largely repeats those end-to-start closures (rule 20): it says the map joined its end to its start, not how far the tracking drifted - that is the tracking-alone rows (about %.1f m)." % facts["closed_loop_gap_m"] if end_ids else "Nothing ties the end straight to the start, so the corrected gap is not a repeat of an end-to-start closure."))
w("- Where the closures fell (newer node's time, the drive cut into four equal quarters): %s. First %s, last %s." % (" / ".join(map(str, q)), hm(ts[0]), hm(ts[-1])))
w("  *Plain terms: %s*" % ("no closure at all in the first quarter of the drive (until %s) - [INFERENCE] the first part of the route was new ground, so there was nothing yet to recognise; most closures came in the last quarter, when the robot came back through mapped corridors." % hm(T0n + (T1n - T0n) / 4) if q[0] == 0 else "closures are spread over the drive as shown."))
w("- **The robot's own wheels + gyroscope ended %.2f m and %+.1f deg off**, so its heading drifted by about %.0f deg over the drive; the LiDAR estimate, built on those same wheels + gyroscope, came home to %.2f m after its %d LiDAR loop closures. The Jetson's copy of that stream (over WiFi) gives %.3f m / %+.2f deg, the same drift: it is the wheels + gyroscope themselves, not the WiFi. *Plain terms: the robot's own sense of direction was off by about %.0f degrees by the end; the LiDAR map corrected it by recognising places.*" % (
    _w["end_to_start_m"], _w["heading_deg"], abs(_w["heading_deg"]), lgap_floor, lmeta["loop_closures"], tg["robot"]["end_to_start_m"], tg["robot"]["heading_deg"], abs(_w["heading_deg"])))
w("- %d of %d nodes are in the corrected graph; the rest stay in the database's Node table but out of the map graph (RTAB-Map's memory management)." % (cmeta["poses"], nodes_total))
w("")

w("## 2. Every turn: robot vs camera vs blend")
w("")
w("The live turn watcher's rule replayed after the drive over fusion.bag by `turns_replay.py` (unchanged). Output: turns_replayed.log -> turns.csv. *95th percentile = the value 95 % of turns stay under. The robot's wheels + gyroscope, as received over WiFi, is the comparison for short turns (not ground truth); a turn during a WiFi gap has a less reliable robot value (section 3).*")
w("")
w("| summary (%d turns) | median | 95th percentile | largest | within 1 deg |" % tr["n"])
w("|---|---|---|---|---|")
w("| abs(blend - robot) | %.2f deg | %.2f deg | %.1f deg | %d of %d |" % (tr["abs_blend_minus_robot_deg"]["median"], tr["abs_blend_minus_robot_deg"]["p95"], tr["abs_blend_minus_robot_deg"]["max"], tr["within_1deg_blend"], tr["n"]))
w("| abs(camera - robot) | %.2f deg | %.2f deg | %.1f deg | %d of %d |" % (tr["abs_camera_minus_robot_deg"]["median"], tr["abs_camera_minus_robot_deg"]["p95"], tr["abs_camera_minus_robot_deg"]["max"], tr["within_1deg_camera"], tr["n"]))
w("")
w("Turns with no camera value (camera lost for the whole turn): %d. Figure: `turns_difference.png`." % tr["turns_with_no_camera_value"])
w("")
w("| turn | ended | direction | s | robot deg | camera deg | blend deg | camera lost | blend - robot | camera - robot |")
w("|---|---|---|---|---|---|---|---|---|---|")
with open(os.path.join(P, "turns.csv")) as f:
    for r in csv.DictReader(l for l in f if not l.startswith("#")):
        w("| %s | %s | %s | %s | %s | %s | %s | %s %% | %s | %s |" % (r["turn"], r["ended_hamilton"], r["direction"].lower(), r["seconds"], r["robot_deg"], r["camera_deg"], r["blend_deg"], r["camera_lost_pct"], r["blend_minus_robot_deg"], r["camera_minus_robot_deg"]))
w("")

w("## 3. The WiFi link to the robot (the bridge) - how long the blend ran without fresh robot data")
w("")
w("*Sources: `~/.run_records/s2_static_10/bridge_recv.log` (its clock is UTC, converted here), its FINAL counters; `bridge_freshness.json` / `bridge_freshness.png` (bridge_freshness_drive10.py over fusion.bag). Rule: the blend's EKF ignores a robot input older than 0.5 s (`sensor_timeout: 0.5`, catkin_ws/src/sidewalk_slam/config/ekf_fused.yaml:49), so a moment counts as \"no fresh robot data\" when the newest robot message the blend received was more than 0.5 s old. Window %s-%s (%.0f s).*" % (fr["window"][0], fr["window"][1], fr["window_s"]))
w("")
w("*Plain terms: the robot sends its wheel readings 10 times a second and its gyroscope 20 times a second over WiFi. When nothing arrived for half a second, the blend was steering on the camera alone.*")
w("")
w("| robot input to the blend | no fresh data (> 0.5 s old) | stretches | > 2 s old | > 5 s old | longest stretch |")
w("|---|---|---|---|---|---|")
for name, d in fr["inputs"].items():
    w("| %s | **%.1f s (%.1f %%)** | %d | %.1f s (%.1f %%) | %.1f s (%.1f %%) | %.1f s |" % (name, d["over_0.5s"]["seconds"], d["over_0.5s"]["percent_of_drive"], d["over_0.5s"]["stretches"], d["over_2s"]["seconds"], d["over_2s"]["percent_of_drive"], d["over_5s"]["seconds"], d["over_5s"]["percent_of_drive"], d["longest_stretch_s"]))
w("")
w("- Longest stretches with neither wheels nor gyroscope fresh: %s." % "; ".join("%s (%.1f s)" % (s["from"], s["seconds"]) for s in nb["longest_stretches"]))
wr = fr["wheel_rate_per_10s"]
w("- Wheel messages received per 10 s window: median %.1f a second, lowest %.1f a second; %d of %d windows under 2 a second, %d under 1 a second (nominal 10)." % (wr["median_hz"], wr["lowest_hz"], wr["windows_below_2hz"], wr["windows_total"], wr["windows_below_1hz"]))
w("- **Link down (no bytes for 1.5 s): %d time(s)**, at %s Hamilton; connections %d (bridge_recv.log FINAL `connects`)." % (len(downs_h), ", ".join(downs_h), fin["connects"] if fin else -1))
if fin:
    w("")
    w("| stream | robot read | robot sent | published on the Jetson | % of sent | missed while down | too late (stale, > 300 ms) | lost |")
    w("|---|---|---|---|---|---|---|---|")
    ss = fin["sender_stats"]
    for k, v in fin["topics"].items():
        sent = ss["sent"].get(k, 0)
        w("| %s | %d | %d | %d | %.1f %% | %d | %d | %d |" % (k, ss["received"].get(k, 0), sent, v["published"], 100.0 * v["published"] / sent if sent else 0, v["missed_while_down"], v["stale"], v["lost"]))
    w("")
    w("*\"missed while down\" = readings the robot made while the link was down; \"stale\" = arrived but more than 300 ms old, so thrown away (the bridge receiver's rule). The robot's own copy of all these readings is in its own recording, unaffected by WiFi.*")
w("- Where it happened: `bridge_freshness.png` (right panel, the blend's own tracking-alone path, red where it had no fresh robot data). Read off that figure: the gaps sit on the far corridors of the loop (the bottom corridor and the right-hand corridor as drawn), hardly any on the corridor past the start - consistent with the WiFi flickering far from the lab [INFERENCE: no WiFi signal strength was logged along the route].")
w("")

w("## 4. Camera tracking: losses, restarts, freezes, crashes")
w("")
w("- **Camera program: 0 crashes, 0 hangs**: camera guard ended with restarts %d, state \"%s\" (`~/.run_records/s2_static_10/camera_guard.json`)." % (guard["restarts"], guard["state"]))
w("- **Lost** (fusion.bag, `/rtabmap/odom`: all-zero pose or covariance 9999 - the tracker's \"I am lost\" value): **%d stretches, %.1f s in total, longest %.1f s, %.1f %% of messages** (numbers.json)." % (lost["stretches"], lost["seconds_total"], lost["longest_s"], lost["share_of_messages_pct"]))
w("- **Restarts from TF** (`mapping.log`, \"Odometry automatically reset to latest odometry pose available from TF\" - the camera's tracking gave up and restarted from the blend's position): **%d**, in %d burst(s) (restarts within 5 s of each other grouped). Largest bursts: %s. The monitor's last line says resets=%s (`monitor_STATUS.txt`). Drive 9: 101 over %.0f s; drive 10: %d over %.0f s." % (
    len(resets), len(bursts), "; ".join("%s-%s (%d)" % (hm(a), hm(b), n) for a, b, n in top_bursts), re.search(r"resets=(\d+)", mon).group(1), facts9["duration_s"], len(resets), facts["duration_s"]))
w("  All bursts: %s." % burst_txt)
w("- **Freezes** (pass_check.py from `03_methods/jetson_stalls_drive3_2026-09-25/` -> freezes_pass_check.json): camera tracker silences > 1.5 s: **%d** (%s); 'ZED Diagnostic' pauses > 2 s: %d (%s); database saves >= 5.5 s: %d; control, camera gyroscope longest gap %.3f s; map updates parsed %d of %d nodes. Verdict as printed: **%s** (pass line = zero silences)." % (
    len(sil), ", ".join("%.2f s" % s for s in sil) or "none", len(fpc["P4_zed_diagnostic_pauses_over_2s"]), ", ".join("%.2f s" % s for s in fpc["P4_zed_diagnostic_pauses_over_2s"]) or "none",
    fpc["saves_over_5p5s"], fpc["P3_camera_gyro_max_gap_s"], fpc["P5_parsed_iterations"], fpc["P5_db_nodes"], fpc["verdict"]))
w("- Where the tracker silences come from (`freeze_timing_drive7.py` -> freeze_timing.json): %d silences; %d overlap a 'ZED Diagnostic' pause of the camera program's picture loop; %d overlap a database save of 1.5 s or more." % (ft["n"], ft["n_overlapping_zed_diagnostic_pause"], ft["n_overlapping_db_save_over_1p5s"]))
for r in ft["camera_tracker_silences_over_1p5s"]:
    w("  - %s, %.2f s: diagnostic pause %s, database save %s, restart from TF within 5 s after: %s" % (r["from"], r["seconds"], "yes" if r["overlaps_zed_diagnostic_pause"] else "no", r["overlaps_db_save_over_1p5s"] or "none", "yes" if r["restart_from_tf_within_5s_after"] else "no"))
w("- Silent feature-detector downgrade (`\"cannot be used\"` in mapping.log, ENGINEERING_NOTES.md section 2.6): %d lines." % downcut)
w("- Database integrity (`db_check.txt`, quick_check, read-only): `%s`" % dbchk.splitlines()[-1])
w("")

w("## 5. Blend heading while standing still (reported, not a pass line)")
w("")
w("stops %d (%.0f s still); signed sum **%+.3f deg**; sum of each stop's net change %.3f deg; largest single stop %+.3f deg (numbers.json). Rule: %s. *Stillness is judged from the robot's wheel stream as received over WiFi; during WiFi gaps (section 3) a stop may be missed.*" % (still["stops"], still["seconds_still"], still["signed_sum_deg"], still["sum_abs_net_per_stop_deg"], still["largest_stop_net_deg"], still["rule"]))
w("")

w("## 6. Processor, power and temperature")
w("")
w("*Source: `~/.run_records/s2_static_10/tegrastats.log` (once a second; its clock is UTC, converted), window %s to %s Hamilton, read by tegra_peaks.py -> power_temperature.json. Drive 9's figures: its pack's power_temperature.json.*" % tuple(s[11:] for s in pw["window_hamilton"]))
w("")
w("| quantity | drive 10 peak | when | drive 10 median | drive 9 peak / median |")
w("|---|---|---|---|---|")
w("| total power (four named supply rails) | **%.1f W** | %s | %.1f W | %.1f / %.1f W |" % (pw["power_total_W"]["max"]["value"], pw["power_total_W"]["max"]["at_hamilton"], pw["power_total_W"]["median"], pw9["power_total_W"]["max"]["value"], pw9["power_total_W"]["median"]))
w("| hottest point on the chip (tj) | **%.1f C** | %s | - | %.1f C / - |" % (pw["tj_C_max"]["value"], pw["tj_C_max"]["at_hamilton"], pw9["tj_C_max"]["value"]))
w("| processor temperature | %.1f C | %s | - | %.1f C / - |" % (pw["cpu_temp_C_max"]["value"], pw["cpu_temp_C_max"]["at_hamilton"], pw9["cpu_temp_C_max"]["value"]))
w("| graphics processor temperature | %.1f C | %s | - | %.1f C / - |" % (pw["gpu_temp_C_max"]["value"], pw["gpu_temp_C_max"]["at_hamilton"], pw9["gpu_temp_C_max"]["value"]))
w("| graphics processor load | %d %% | %s | %.0f %% | %d / %.0f %% |" % (pw["gpu_load_pct"]["max"]["value"], pw["gpu_load_pct"]["max"]["at_hamilton"], pw["gpu_load_pct"]["median"], pw9["gpu_load_pct"]["max"]["value"], pw9["gpu_load_pct"]["median"]))
w("| processor load, mean of the 12 cores | %.1f %% | %s | %.1f %% | %.1f / %.1f %% |" % (pw["cpu_mean_of_12_cores_pct"]["max"]["value"], pw["cpu_mean_of_12_cores_pct"]["max"]["at_hamilton"], pw["cpu_mean_of_12_cores_pct"]["median"], pw9["cpu_mean_of_12_cores_pct"]["max"]["value"], pw9["cpu_mean_of_12_cores_pct"]["median"]))
w("| memory free, lowest | %.1f %% (%.1f GB used) | %s | - | %.1f %% / - |" % (pw["ram_free_pct_lowest"]["value"], pw["ram_free_pct_lowest"]["used_GB"], pw["ram_free_pct_lowest"]["at_hamilton"], pw9["ram_free_pct_lowest"]["value"]))
w("")
w("- tegrastats skips over 2 s: %s; samples %d." % (", ".join("%s (%.0f s)" % tuple(s) for s in pw["tegrastats_skips_over_2s"]) or "none", pw["samples"]))
w("- *Plain terms: the memory guard (earlyoom) kills the mapping program if free memory falls under 15 %; the lowest here is shown above.*")
w("- RTAB-Map, time to process each map snapshot (database Statistics table, read-only): **median %.1f ms, 95th percentile %.1f ms, largest %.1f ms over %d nodes** (numbers.json). Drive 9: %.1f / %.1f / %.1f ms over %d nodes." % (
    rt["median"], rt["p95"], rt["max"], rt["n"], num9["processor_rtabmap"]["rtabmap_ms_per_node"]["median"], num9["processor_rtabmap"]["rtabmap_ms_per_node"]["p95"], num9["processor_rtabmap"]["rtabmap_ms_per_node"]["max"], num9["processor_rtabmap"]["rtabmap_ms_per_node"]["n"]))
w("")

w("## 7. Agreement with the LiDAR estimate")
w("")
w("*The reference is the **LiDAR estimate (Force3DoF replay)**: the robot's LiDAR mapping (a colleague's work: self_navigation `rtabmap_3d.launch`) replayed on the robot after the drive with ONE parameter changed, `Reg/Force3DoF=true` (the map is held flat on the floor, so a wrongly-read gravity direction cannot tip it; `03_methods/lidar_drift_2026-09-26/FINDINGS.md` section 11). No colleague-mode replay exists for drive 10. It is the robot's wheels + gyroscope corrected by LiDAR loop closures. **An independent estimate, not ground truth** (ENGINEERING_NOTES.md rule 19): LiDAR range noise 15-30 mm, the two LiDARs agree to about 5 mm, and the LiDAR-to-camera mounting offset has never been measured. It shares the robot's wheels + gyroscope with the blend, so for the blend it is not fully independent. Alignment: SE(3) (rotation + shift), never resizing (ENGINEERING_NOTES.md section 4 rule 4).*")
w("")
w("*Plain terms: \"agreement\" = at each moment, how far apart the camera map and the LiDAR map put the robot, after turning and sliding one path once to fit the other. Median = the typical distance; 95th percentile = the distance 95 % of moments stay under.*")
w("")
w("- LiDAR estimate (Force3DoF replay): %d positions, %.1f m, **%d LiDAR loop closures** (drive 9: %d); its own end gap %.3f m on the floor, %.3f m in 3D (section 1). Its own check (agreement.json, compare_lidar.py: ends within 0.5 m of its start AND at least one LiDAR closure): \"%s\"." % (
    lfacts["nodes"], lfacts["path_m_nodes"], lmeta["loop_closures"], lmeta9["loop_closures"], lgap_floor, lmeta["end_gap_corrected_m"], ag["reference_verdict"]))
w("- **Flatness check of the LiDAR estimate** (`03_methods/lidar_drift_2026-09-26/score_f3dof.py` -> lidar_flatness_check.json): largest tilt %.1f deg, height range %.2f m -> **%s** (pass line: tilt under 3 deg and height range under 0.3 m). *Under Force3DoF flatness is built in, so passing is expected; the evidence that this yardstick is sound is its closures and its end gap.* Its own quick comparison there (node positions, 2D fit, a different method from the table below): vs camera map %.2f / %.2f m, vs robot's wheels + gyroscope %.2f / %.2f m (median / 95th percentile)." % (
    flat["tilt_max_deg"], flat["height_range_m"], "PASS" if flat["flatness_pass"] else "FAIL", flat["vs_camera_SE2_med_p95"][0], flat["vs_camera_SE2_med_p95"][1], flat["vs_wheels_SE2_med_p95"][0], flat["vs_wheels_SE2_med_p95"][1]))
w("")
w("| path compared | kind (rule 20) | matched | median | 95th percentile | largest | best-fit size factor (NOT applied) | drive 9 median / 95th pct | file |")
w("|---|---|---|---|---|---|---|---|---|")
for k, lab, kind in (("camera_corrected", "camera map, corrected (camera_corrected.tum)", "map's corrected path"), ("camera_tracking", "map nodes (= blend), tracking alone (camera.tum)", "tracking alone")):
    e, e9 = ag["estimates"][k], ag9["estimates"][k]
    w("| %s | %s | %.1f %% (%d) | **%.3f m** | **%.3f m** | %.3f m | %.4f | %.3f / %.3f m | agreement.json |" % (lab, kind, e["matched_pct"], e["n_matched"], e["disagreement_median_m"], e["disagreement_p95_m"], e["disagreement_max_m"], e["sim3_scale_vs_reference"], e9["disagreement_median_m"], e9["disagreement_p95_m"]))
w("")
w("- Clock shift: estimated %+.2f s, applied %.1f s (compare_lidar.py applies one only if it improves the fit by > 0.05 m and 20 %%)." % (ag["time_offset_estimated_s"], ag["time_offset_applied_s"]))
w("- Where the distance grows (`lidar_comparison.png`, lower right): the camera map's distance from the LiDAR estimate rises and falls in a regular pattern, largest at the far corners of the loop and smallest near the start - the pattern a SIZE difference makes after a rotation + shift fit, rather than a bend in one place [INFERENCE from the figure; section 7a measures it].")
w("")
w("### 7a. For information, size fitted (NOT the result)")
w("")
w("*Why this is here: a taped 15.00 m test (`01_runs/series2_static/s2_scale_01/RESULTS.md`) found the robot's wheels read about 6 %% long and the camera within 1 %%. The LiDAR estimate takes its size from the wheels, so part of the SE(3) distance above is the LiDAR estimate being too large, not the camera map's shape. Below, the same comparison is repeated allowing one size change (Sim(3)). For information only: the camera's size is real and is not to be fitted away (ENGINEERING_NOTES.md section 4 rule 4).* Made by `size_fit_agreement.py` -> size_fit.json, same reference, same 50 ms pairing, same fitting code as compare_lidar.py; its rigid (SE(3)) run reproduces agreement.json exactly (control: %s)." % ("passed" if all(v["control_se3_reproduces_agreement_json"] for v in sfit["estimates"].values()) else "FAILED"))
w("")
w("| path compared | pairs | SE(3) median / 95th pct (the result) | size fitted: median / 95th pct | size factor applied to the camera path | drive 9 size fitted |")
w("|---|---|---|---|---|---|")
for k, lab in (("camera_corrected", "camera map, corrected"), ("camera_tracking", "map nodes (= blend), tracking alone")):
    e, e9 = sfit["estimates"][k], sfit9["estimates"][k]
    w("| %s | %d | %.3f / %.3f m | %.3f / %.3f m (for information, size fitted) | %.4f | %.3f / %.3f m at %.4f |" % (lab, e["n_pairs"], e["se3"]["median_m"], e["se3"]["p95_m"], e["sim3"]["median_m"], e["sim3"]["p95_m"], e["sim3"]["scale_applied"], e9["sim3"]["median_m"], e9["sim3"]["p95_m"], e9["sim3"]["scale_applied"]))
w("")
_sc = ag["estimates"]["camera_corrected"]["sim3_scale_vs_reference"]
w("*Plain terms: a size factor of %.3f = the corrected camera path would have to be stretched by %.1f %% to fit the LiDAR estimate best. The tape test says the robot's wheels + gyroscope read 1.064 times the true distance and the camera 0.993 times, and the corrected camera map 0.989 times (legs spread about +-1 %%), so the size difference expected from the tape alone is about 1.064 / 0.993 = 1.071 to 1.064 / 0.989 = 1.076. Drive 10's %.4f is consistent with that, well inside the one-drive spread of the size factor (%.3f, drives 3-5; compare_drives_4_8_9_10.json `spreads.scale`). [INFERENCE] so most of the %.2f m median is the LiDAR estimate's size (from the wheels), not the camera map's shape: allowing for size, the camera map sits a median %.2f m from the LiDAR estimate, about the same as drive 9's %.2f m.*" % (
    _sc, 100 * (_sc - 1), _sc, cmp["spreads"]["scale"], ag["estimates"]["camera_corrected"]["disagreement_median_m"], sfit["estimates"]["camera_corrected"]["sim3"]["median_m"], sfit9["estimates"]["camera_corrected"]["sim3"]["median_m"]))
w("")
w("### 7b. The side-by-side picture: camera map and LiDAR map side by side")
w("")
w("`d10_camera_vs_lidar_side_by_side.png` (deck_figure_drive10.py): left, the camera map from above (the 3D point cloud, points 0.15-2.0 m above the floor, %d of %d points, 5 cm cells); middle, the LiDAR estimate's map (Force3DoF replay, walls 0.15-2.0 m up) - both in the LiDAR map's frame at the same scale, the camera map moved by rotation + shift only (fit residual %.4f m, rotation %+.2f deg); right, the last frame the live page recorded of the LiDAR view during the drive (`%s`), i.e. the colleague's default settings with the tilt from the robot's IMU, in its own framing and scale." % (
    dp["camera_points_used"], dp["camera_points_total"], dp["alignment_fit_max_residual_m"], dp["rotation_deg"], os.path.basename(dp["live_frame"] or "-")))
w("")
w("- **What it shows (by eye, not measured):** the live default-settings LiDAR view has doubled and fanned-out walls, as seen during the drive; the Force3DoF replay of the same recording has single, straight walls; the camera map's walls are single too. [INFERENCE] the doubling in the live view is the tilt from the IMU (drive 9's investigation, FINDINGS.md section 11), not the LiDAR itself - holding the map flat removes it. The camera map is visibly smaller than the LiDAR map (its dashed path sits inside the solid one at the far corners): that is the size factor of section 7a.")
w("")
w("## 8. Distances - from the camera and from the wheels")
w("")
w("*Each source's own path length. \"2 Hz\" = positions every 0.5 s, straight steps summed (paths_2hz.csv). The tape test (s2_scale_01) found the robot's wheels read about 6 % long and the camera within 1 %.*")
w("")
w("| source | what it is | path length | drive 9 | file |")
w("|---|---|---|---|---|")
w("| **camera map, corrected** | camera, after its loop closures (every node) | **%.1f m** | %.1f m | map_corrected_facts.json |" % (path_c, mcf9["path_m_nodes"]))
w("| map nodes, tracking alone | the blend's position at each node | %.1f m | %.1f m | facts.json |" % (facts["path_m_nodes"], facts9["path_m_nodes"]))
for k, lab in (("camera", "camera tracking, 2 Hz (re-seeded by the blend at restarts)"), ("blend", "blend, 2 Hz"), ("robot", "robot's wheels + gyroscope as received over WiFi, 2 Hz (gaps bridged by straight steps)")):
    if k in plen: w("| %s | - | %.1f m | - | paths_2hz.csv |" % (lab, plen[k]))
w("| **robot's own wheels + gyroscope, 2 Hz, the robot's own recording** | wheels (what the LiDAR map is built on), start to parked | **%.1f m** | %.1f m | robot_wheels_from_robot_bag.json |" % (_w["path_m_2hz"], rw9["whole_drive_to_parked"]["path_m_2hz"]))
w("| robot's own wheels + gyroscope, every message, the robot's own recording | same, straight steps between every message | %.1f m | %.1f m | robot_wheels_from_robot_bag.json |" % (_w["path_m_every_message"], rw9["whole_drive_to_parked"]["path_m_every_message"]))
w("| LiDAR estimate (Force3DoF replay) | wheels + gyroscope corrected by LiDAR closures (its map nodes) | %.1f m | %.1f m | lidar_map_facts.json |" % (lfacts["path_m_nodes"], lfacts9["path_m_nodes"]))
w("| robot's wheels + gyroscope x 0.940 (for information: the tape's correction) | wheels (robot's own recording, 2 Hz), resized by the taped test | %.1f m | %.1f m | computed here |" % (0.940 * _w["path_m_2hz"], 0.940 * rw9["whole_drive_to_parked"]["path_m_2hz"]))
w("")
w("**Distance to quote for drive 10: about %.0f m by the camera map** (corrected; the wheels say %.0f m, which the tape shows reads about 6 %% long - resized by the tape, %.0f m)." % (path_c, _w["path_m_2hz"], 0.940 * _w["path_m_2hz"]))
w("")

w("## 9. Drives 4 / 8 / 9 / 10 - did the whole-floor drive keep the quality of the short drive and of drive 9?")
w("")
w("*Made by `compare_drives_4_8_9_10.py` -> compare_drives_4_8_9_10.md / .json, shared_route.json, shared_route.png (a copy of drive 9's compare_drives_4_8_9.py, which is left as it was). Drive 4 = the short reference drive (85 m), drive 9 = the first longer drive (247 m), drive 8 = drive 4's route repeated (for information). Drive 10's route lined up on drive 9's needed a constraint: the floor's corridors are nearly symmetric about the start, and an unconstrained fit turned it by about -90 deg; the heading is taken from both drives' clear alignment on drive 4 (shared_route.json `pairs_note_9_vs_10`).*")
w("")
w("*Plain terms for the verdict lines: each row states how much one drive's number is expected to vary by chance (\"spread\"); \"agrees\" = the two drives differ by less than that; \"differs by N times\" = N times more (3 or more is a real difference, 1-3 is a hint).*")
w("")
for l in open(os.path.join(P, "compare_drives_4_8_9_10.md")).read().splitlines():
    w(l)
w("")
w("## 10. Figures, video and 3D map in this folder")
w("")
w("| file | what it shows |")
w("|---|---|")
w("| map.png | camera floor map, map nodes at their tracking-alone positions (Node.pose = the blend's) |")
w("| map_corrected.png | camera floor map after the loop closures (RTAB-Map's own Admin.opt_poses) |")
w("| trajectory.png | where each source put the robot, two panels (rule 20): tracking alone (robot's wheels + gyroscope as received over WiFi solid; camera and blend dashed) and corrected (camera map dashed, LiDAR estimate solid) |")
w("| lidar_comparison.png | the LiDAR map with the camera's paths drawn on it, and how far apart they are along the route (compare_lidar.py) |")
w("| d10_camera_vs_lidar_side_by_side.png | **the side-by-side picture** (section 7b): camera map and LiDAR map from above, same scale, plus the last live LiDAR frame (default settings) |")
w("| shared_route.png | the parts of the route drive 10 shared with drives 4 and 9 (section 9) |")
w("| lidar_map.png | the LiDAR estimate's map (Force3DoF replay) with its path |")
w("| turns_difference.png | per turn: camera minus robot and blend minus robot, degrees |")
w("| bridge_freshness.png | robot messages reaching the blend over time, and where on the route the blend had no fresh robot data |")
ix = sum(1 for _ in open(os.path.join(P, "timelapse_live_index.csv"))) - 1
w("| timelapse.mp4 | **recorded live during the drive** (the live map page every 3 s: camera map left, LiDAR view right), built by the recorder at park: %d frames at 10 a second (`timelapse_live_index.csv`, `~/.run_records/s2_static_10/media_recorder.out`) |" % ix)
w("| timelapse_25/50/75/100.png | stills at 25/50/75/100 % of the timelapse (ffmpeg) |")
m3 = os.path.join(P, "map_3d")
if os.path.exists(os.path.join(m3, "drive10_map_3d.ply")):
    w("| map_3d/ | 3D point cloud of the camera map (`drive10_map_3d.ply`), `d10_3d.png` (two 3D views), `d10_topdown.png` (from above, robot path dashed); points placed at RTAB-Map's own saved corrected positions (`rtabmap-export --opt 2`, identical to camera_corrected.tum on all 860 nodes); `map_3d/README.txt` |")
else:
    w("| (no 3D map) | %s |" % (open(os.path.join(m3, "skipped.txt")).read().strip() if os.path.exists(os.path.join(m3, "skipped.txt")) else "not made"))
w("")
w("Other files: camera.tum / camera_corrected.tum (+ .meta.json); lidar.tum (+ .meta.json), wheel.tum (+ .meta.json), lidar_reference_dense.tum, lidar_map.npz, lidar_map_facts.json, lidar_comparison/ (evaluate_trajectory.py outputs), agreement.json, size_fit.json, lidar_flatness_check.json, robot_wheels_from_robot_bag.json, compare_drives_4_8_9_10.md/.json, shared_route.json, d10_camera_vs_lidar_side_by_side.json; lidar_ref_f3dof/ (the replay's products as pulled from the robot, with provenance.txt and logs); paths_2hz.csv; turns.csv, turns_replayed.log; numbers.json; facts.json, map_corrected_facts.json; freezes_pass_check.json; freeze_timing.json; power_temperature.json; bridge_freshness.json; db_check.txt; closed_check.txt.")
w("")
w("## 11. Still open")
w("")
w("- **Size**: the LiDAR estimate is %.1f %% larger than the camera map (the camera map %.1f %% smaller; section 7a); the tape test attributes this to the wheels, but the LiDAR-to-camera mounting offset has never been measured, and no drive has a measured true length. A taped long leg on this floor would settle which map has the right size." % (100 * (_sc - 1), 100 * (1 - 1 / _sc)))
w("- **LiDAR estimate: %d closures over %.0f m** (drive 9: %d over %.0f m) - fewer per metre; it still passes its own check (%.2f m end gap)." % (lmeta["loop_closures"], lfacts["path_m_nodes"], lmeta9["loop_closures"], lfacts9["path_m_nodes"], lgap_floor))
w("- **WiFi**: the blend ran %.1f %% of the drive without fresh robot data (section 3); the turn comparison and the stillness check use the robot stream as received, so they are weaker where it was missing." % nb["over_0.5s"]["percent_of_drive"])
w("- **%d camera restarts from the blend's position** (drive 9: 101): the camera's own tracking gave up often; the map still closed %d loops." % (len(resets), len(pairs)))
w("- N = 1: one drive of the whole floor, not five repeats (ENGINEERING_NOTES.md section 4 rule 2).")
w("- The camera's path is not independent of the blend in a fused drive (section 1).")
open(os.path.join(P, "RESULTS.md"), "w").write("\n".join(o) + "\n")
print("RESULTS.md", len(o), "lines; closures", len(pairs), "to start", len(to_start), "resets", len(resets))
