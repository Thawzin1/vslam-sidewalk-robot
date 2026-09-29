# Live pages served by the Jetson

- `slam_live_push.py` writes `~/jobs/live_snapshot.json` every 2 s (the drive's map and numbers).
- `live_site_server.py` (127.0.0.1:8097) serves `results_website/public/` and `/api/state` (the
  latest snapshot). Every page needs the view key, read from `$LIVE_SITE_KEY_FILE` (default
  `~/.config/live_site_key`, mode 600 - readable by its owner only; never print it). The first visit
  with `?k=<key>` sets a cookie; later visits use it.
- `keep_live_site.sh` keeps the server alive (cron every minute and at boot).
- The server is published to the internet from the Jetson; the link belongs to that Jetson. Students
  in the lab get it from Prof. Mehrtash.
- More detail: `results_website/README.md`.
