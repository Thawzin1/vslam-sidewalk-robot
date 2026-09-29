# The project website (served by the Jetson)

**What it is:** the drive's live map and status, and the Results pages, readable from a phone or
any computer. The Jetson in the lab serves it (`../live_site_server.py`, port 8097). The link is
tied to that Jetson; students in the lab get it from Prof. Mehrtash. Every page is behind a view
key (a short secret in the link, `?k=...`); after the first visit a cookie remembers it.

## How it works

```
Jetson
live_map_server.py  :8095          (the drive's own map page)
     | read on 127.0.0.1 only
slam_live_push.py  --every 2 s-->  ~/jobs/live_snapshot.json
                                        |
live_site_server.py :8097  -- /api/state -->  status.html / map.html, polled every 2 s
                           -- files ------->  results.html + public/results/<run>/
```

*Plain terms: the sender copies the drive's map and numbers into one small file every
2 seconds; the website hands the newest copy to whoever opens the link.*

- **The freshness strip comes first.** A page that stopped updating looks exactly like a quiet
  drive. The strip says LIVE / LATE / STOPPED SENDING, judged against the interval the sender
  itself declares (2 s while the drive's page answers, 10 s while it does not). "Never sent"
  and "stopped sending" are shown differently, and a clean stop (`kill -INT`) writes one last
  snapshot so the page says "stopped on purpose".
- **If the drive's page (8095) is down,** the sender keeps writing and the page shows an amber
  band saying so, rather than going quiet.
- **Bandwidth:** the page tells the server which pictures it already shows (a short fingerprint);
  unchanged pictures are left out of the answer. Parked robot: about 3 kB per poll instead of
  about 230 kB.

## Start and stop the sender (on the Jetson)

```bash
TZ=America/Toronto setsid nohup nice -n 19 python3 "$TOOLS_DIR/slam_live_push.py" >> ~/jobs/slam_live_push.log 2>&1 < /dev/null &
```

- *what it does:* starts the sender in its own session (it survives the terminal closing), at
  the lowest processor priority, logging to `~/jobs/slam_live_push.log`. `../keep_live_push.sh`
  (run by cron, the Linux scheduler) restarts it if it stops.
- *is it alive:* `cat ~/jobs/slam_live_push.progress` - one line, rewritten every 2 s; also
  shown on the jobs page (port 8096). The pid (process number) is in `~/jobs/slam_live_push.pid`.
- *stop it:* `kill -INT <pid>` using that number. Never `pkill -f` (it matches its own command
  line - `docs/ENGINEERING_NOTES.md` rule 8).

It only **reads** the drive's page and the `~/jobs/<run>*.progress` files of the newest run. It
never starts, stops or changes anything belonging to the drive.

## Refresh the Results pages

```bash
python3 build_results.py
```

Copies each run's pictures (.png), videos (.mp4, under 30 MB) and `RESULTS.md` into
`public/results/<run>/` and writes `public/results/results.json`, which `results.html` draws.
Never raw recordings. Run it again whenever a results pack changes.

## Files

| file | what |
|---|---|
| `public/index.html`, `status.html`, `map.html` | the live pages |
| `public/results.html` | the Results page (reads `public/results/results.json`) |
| `public/common.js`, `site.css` | shared page code and style |
| `build_results.py` | fills `public/results/` from the results packs |
| `../live_site_server.py` | the web server on the Jetson (port 8097) |
| `../slam_live_push.py` | the sender (Python 3.8, standard library only) |
| `../keep_live_site.sh`, `../keep_live_push.sh` | restart the server / the sender if they stop (cron) |
