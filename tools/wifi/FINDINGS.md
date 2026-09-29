# WiFi roaming - why drive 9 lost the Jetson, and the fix (26 Sept 2026, Hamilton)

**Cause (Jetson journal, drive 9):** the Jetson's Realtek WiFi driver (rtl88x2ce) cannot report signal changes
("bgscan simple: Failed to enable signal strength monitoring"), so wpa_supplicant (the program that joins access
points) looked for a better access point only on its 5-minute timer: roams at exactly 20:55:59, 21:00:59, 21:10:59,
21:15:59. At 21:15:59 it joined an access point near the start (f4:bd:9e:a1:c1:82, 5320 MHz); the robot drove away,
internet was lost at 21:16:28 (NetworkManager CONNECTED_SITE), and the next scan was not due until 21:21:03 - the
moment the Jetson was power-cycled. The robot's Intel card (iwlwifi) roams on signal but had power saving ON.
*Plain terms: a phone checks its signal constantly and moves before it fades; the Jetson only checked every 5 minutes.*

**Fix (, both machines):**
`wifi_roam_helper.py` as root service `wifi-roam-helper` on the Jetson and the robot: every 2 s reads the signal
(`iw`) and pings the gateway; below -67 dBm for 6 s it scans only the channels this network uses and roams to an
access point at least 8 dB stronger (`wpa_cli roam`); 15 s without the gateway -> `wpa_cli reassociate`; keeps
power saving off. Installed 26 Sept ~23:45 (Jetson) and ~23:50 (robot); robot power saving on -> off confirmed.
Same night: slam_live_push idle interval 10 -> 2 s, robot agent health 5 -> 2 s.

**Not yet proven.** Pass line for the Sunday 10-minute corridor drive (no mapping): every access-point change under
5 s, and the live map never stops for more than 10 s. Logs: `~/jobs/wifi_roam.log` on each machine.

## Test 1 (27 Sept 00:12-00:25 Hamilton, helper v1) - FAILED the 10 s line
Good-signal areas: ~10 access-point changes at 1 s or less each. Far from the lab both machines fell to -70..-88 dBm:
robot<->Jetson gaps 13, 46, 6 and 208 s (the last until the robot's battery died); Jetson<->internet 69, 22, 26 s.
v1 made the weak areas worse: it switched every 20 s (each switch drops the link on the Realtek card), acted only
after 15 s of no link, reconnected from stale scan results (asked -58 dBm, got -87), and `wpa_cli roam X` on the
Realtek card joined a different access point (asked -50, got -70). Logs: `test1_0012/`.
**v2** (loaded 00:32): no-link action after 6 s with a fresh full scan; results not refreshed by that scan dropped
(`bss_flush 3`); joins the chosen access point by pinning it (`bssid 0 X` + reassociate, then un-pinned); 8 s settle
after each join; reloads itself when its file changes. Weak areas are far from the lab ; the robot's two stick
antennas sit low against the frame, one lying flat. Plan: drive blind in the far areas (all recordings on board),
require reconnection within 10 s of returning near the lab.

## Test 2 (27 Sept 01:06-01:15 Hamilton, helper v2 on both machines) - PASSED the revised line
Revised pass line (weak areas are far from the lab; all recordings are on board): outages allowed in the far
areas, but everything must reconnect within 10 s of returning near the lab. Result: robot drove into a far weak area
and back. Outages only in the far area: Jetson<->internet 16 s (01:08:17) and 22 s (01:12:35); robot<->Jetson 16 s and
20 s (robot's own log). Test 1 (v1) in the same kind of area: 69-75 s. Back near the lab from ~01:14:34: no outage.
v2 repairs logged ("NO LINK 8-9 s ... -> strongest"), lock-on worked (asked -44 dBm, got -44). Logs: `test2_0105/`.
