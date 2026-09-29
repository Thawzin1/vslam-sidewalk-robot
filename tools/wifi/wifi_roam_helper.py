#!/usr/bin/env python3
"""Jetson WiFi roaming helper - make the Jetson change access point like a phone does.

Why: the Jetson's Realtek driver (rtl88x2ce) cannot report signal changes, so wpa_supplicant's
background scan only runs on its fixed 5-minute timer. On drive 9 the Jetson sat on a fading
access point from 21:15:59 until the next timer (21:21:03) and lost the link.

What it does (runs as root, one process, no package installed):
  every 2 s   read the signal with `iw dev wlan0 link` and ping the gateway
  weak        signal below WEAK_DBM for 3 readings (6 s) -> scan, and if an access point of the
              same network is at least BETTER_DB stronger, `wpa_cli roam <it>`
  no link     gateway unanswered for 15 s -> `wpa_cli reassociate` (self-repair)
  cooldown    at most one action per COOLDOWN_S
Also keeps WiFi power saving off. Same file on the Jetson and on the robot.
Log: <jobs>/wifi_roam.log ; one-line status: <jobs>/wifi_roam.progress (argument 1 = jobs folder)
(the jobs page :8096 shows it). Times are Hamilton time.
"""
import os, re, subprocess, sys, time

IFACE = "wlan0"
WEAK_DBM = -67          # below this the helper starts looking for a better access point
BETTER_DB = 6           # a candidate must be this much stronger to be worth a switch
COOLDOWN_S = 10         # between two weak-signal actions
PERIOD_S = 2
NOLINK_S = 6            # v2 (26 Sept test): 15 s was too slow - the 00:18 outage lasted 70 s
SETTLE_S = 8            # after joining an access point, give it this long before judging the link
REPAIR_GAP_S = 8        # between two no-link repairs (doubles after each failed one, up to REPAIR_GAP_MAX_S)
REPAIR_GAP_MAX_S = 60
STRONG_NOLINK_S = 30    # v3 (27 Sept 11:32): on a STRONG link a ping gap is not a radio problem; reassociating at
                        # -56 dBm was rejected by the access point and turned an 8 s blip into a 6-minute outage
SELF = os.path.abspath(__file__)
JOBS = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser(os.environ.get("JOBS_DIR", "~/jobs"))   # robot: /home/administrator/jobs
LOG = os.path.join(JOBS, "wifi_roam.log")
PROG = os.path.join(JOBS, "wifi_roam.progress")
OWNER, GROUP = os.stat(JOBS).st_uid, os.stat(JOBS).st_gid
KNOWN_FREQS = set()     # channels where our network was seen: later scans look only there (shorter radio gap)
WPA = ["wpa_cli", "-p", "/run/wpa_supplicant", "-i", IFACE]
os.environ["TZ"] = "America/Toronto"; time.tzset()


def sh(cmd, timeout=8):
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout).stdout
    except Exception as e:
        return "ERR %s" % e


def log(msg):
    line = "%s %s\n" % (time.strftime("%F %T"), msg)
    with open(LOG, "a") as f:
        f.write(line)
    try:
        os.chown(LOG, OWNER, GROUP)
    except OSError:
        pass


def link():
    out = sh(["iw", "dev", IFACE, "link"])
    b = re.search(r"Connected to ([0-9a-f:]{17})", out)
    s = re.search(r"signal:\s*(-?\d+)", out)
    ssid = re.search(r"SSID:\s*(.+)", out)
    return (b.group(1) if b else None, int(s.group(1)) if s else None,
            ssid.group(1).strip() if ssid else None)


def gateway():
    out = sh(["ip", "-4", "route", "show", "default", "dev", IFACE])
    m = re.search(r"via (\S+)", out)
    return m.group(1) if m else None


def ping(gw):
    return gw is not None and subprocess.call(["ping", "-c1", "-W1", gw],
                                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) == 0


def best_candidate(ssid, cur_bssid, full=False, include_current=False):
    if KNOWN_FREQS and not full:
        sh(WPA + ["scan", "freq=" + ",".join(sorted(KNOWN_FREQS))])
        time.sleep(1.5)
    else:
        sh(WPA + ["scan"])
        time.sleep(3.5)
    # from an old scan and found it at -87 dBm
    sh(WPA + ["bss_flush", "3"])
    best = None
    for ln in sh(WPA + ["scan_results"]).splitlines()[1:]:
        p = ln.split("\t")
        if len(p) >= 5 and p[4] == ssid:
            KNOWN_FREQS.add(p[1])
        if len(p) >= 5 and p[4] == ssid and (include_current or p[0] != cur_bssid):
            sig = int(p[2])
            if best is None or sig > best[1]:
                best = (p[0], sig)
    return best


def go_to(bssid):
    """Join this exact access point. On the 26 Sept test `wpa_cli roam X` on the Jetson's Realtek driver landed on a
    different one (asked -50 dBm, got -70), so pin it for the connect, then un-pin so normal roaming stays possible."""
    sh(WPA + ["bssid", "0", bssid])
    sh(WPA + ["reassociate"])
    for _ in range(10):
        time.sleep(0.5)
        if link()[0] == bssid:
            break
    sh(WPA + ["bssid", "0", "00:00:00:00:00:00"])


def main():
    weak_n, last_ok, last_act, roams, repairs = 0, time.time(), 0.0, 0, 0
    prev_bssid, last_ssid, settle_until, last_repair = None, None, 0.0, 0.0
    last_ps = 0.0
    fails = 0
    mtime = os.stat(SELF).st_mtime
    log("START v3 weak<%d dBm, better by %d dB, no link %d s, settle %d s" % (WEAK_DBM, BETTER_DB, NOLINK_S, SETTLE_S))
    while True:
        now = time.time()
        if now - last_ps > 60:          # power saving adds delay; keep it off (a driver reset turns it back on)
            if "Power save: on" in sh(["iw", "dev", IFACE, "get", "power_save"]):
                sh(["iw", "dev", IFACE, "set", "power_save", "off"]); log("power saving was on -> off")
            last_ps = now
        try:                             # a new version of this file restarts it (no sudo needed)
            if os.stat(SELF).st_mtime != mtime:
                log("file changed -> restarting with the new version")
                os.execv(sys.executable, [sys.executable, SELF] + sys.argv[1:])
        except OSError:
            pass
        bssid, sig, ssid = link()
        if ssid:
            last_ssid = ssid
        if bssid != prev_bssid:
            log("ON %s signal %s" % (bssid, sig))
            prev_bssid = bssid
            if bssid:
                settle_until = now + SETTLE_S
        if ping(gateway()):
            last_ok = now; fails = 0
        weak_n = weak_n + 1 if (sig is not None and sig < WEAK_DBM) else 0
        can_act = now - last_act > COOLDOWN_S
        nolink_s = STRONG_NOLINK_S if (bssid and sig is not None and sig >= WEAK_DBM) else NOLINK_S
        gap = min(REPAIR_GAP_MAX_S, REPAIR_GAP_S * 2 ** fails)
        if now - last_ok > nolink_s and now > settle_until and now - last_repair > gap and last_ssid:
            cand = best_candidate(last_ssid, bssid, full=True, include_current=True)   # fresh full scan
            if cand and cand[0] != bssid:
                log("NO LINK %.0f s on %s (%s dBm) -> strongest %s (%d dBm)" % (now - last_ok, bssid, sig, cand[0], cand[1]))
                go_to(cand[0])
            else:
                log("NO LINK %.0f s on %s (%s dBm), it is the strongest -> reassociate" % (now - last_ok, bssid, sig))
                sh(WPA + ["reassociate"])
            repairs += 1; fails += 1; last_repair = last_act = time.time()
        elif can_act and weak_n >= 2 and ssid:
            cand = best_candidate(ssid, bssid)
            if cand and cand[1] >= sig + BETTER_DB:
                log("ROAM %s (%d dBm) -> %s (%d dBm)" % (bssid, sig, cand[0], cand[1]))
                go_to(cand[0]); roams += 1
            else:
                log("WEAK %s dBm, no better access point (%s)" % (sig, cand))
            last_act = now; weak_n = 0
        with open(PROG, "w") as f:
            os.fchown(f.fileno(), OWNER, GROUP)
            f.write("WIFI_ROAM signal %s dBm on %s  roams %d  repairs %d  link %s\n" % (
                sig, bssid, roams, repairs, "ok" if now - last_ok < 5 else "DOWN %.0fs" % (now - last_ok)))
        time.sleep(PERIOD_S)


if __name__ == "__main__":
    main()
