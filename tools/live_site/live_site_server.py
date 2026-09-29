#!/usr/bin/env python3
"""Serve the project website (status, live map, results, replay and worksheet pages) from the Jetson.

What it serves (port 8097 on this machine, every page behind the site's view key):
  /, /status.html, /map.html, /common.js, /site.css ...  files of tools/live_site/results_website/public/
  /api/state?k=<key>[&m=<sha>&p=<sha>&only=map|status]  the latest snapshot of the Jetson's and robot's state
  /replay/api/...                                         forwarded to the replay backend (tools/replay_web/)
The snapshot is the file slam_live_push.py writes every 2 s (~/jobs/live_snapshot.json).
Key: $LIVE_SITE_KEY_FILE (default ~/.config/live_site_key), mode 600, never printed.
The first visit with ?k=<key> sets a cookie; later visits use the cookie.
To reach the pages from other computers, put this port behind whatever the lab network allows
(for example a reverse proxy); ask Prof. Mehrtash or the lab for the current address.
"""
import hmac, http.server, json, os, socketserver, sys, time, urllib.error, urllib.parse, urllib.request

PORT = int(os.environ.get("LIVE_SITE_PORT", "8097"))
ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results_website", "public")
SNAP = os.path.join(os.path.expanduser(os.environ.get("JOBS_DIR", "~/jobs")), "live_snapshot.json")
KEYF = os.path.expanduser(os.environ.get("LIVE_SITE_KEY_FILE", "~/.config/live_site_key"))
COOKIE = "slam_k"
# The Replay page's backend (tools/replay_web/replay_control_server.py). It listens on this
# computer only; this server checks the view key and then forwards /replay/api/... to it.
REPLAY_API = "http://127.0.0.1:%s" % os.environ.get("REPLAY_CONTROL_PORT", "8098")
REPLAY_MAX_BODY = 64 * 1024
LIVE_PAGES = {"/", "/index.html", "/status.html", "/map.html", "/common.js", "/site.css", "/favicon.ico", "/replay.html"}
DENIED = (b"<!doctype html><html lang=en><head><meta charset=utf-8><meta name=viewport "
          b"content='width=device-width,initial-scale=1'><title>Link needs its key</title></head>"
          b"<body style='font-family:sans-serif;padding:24px'><h2>This link needs its key</h2>"
          b"<p>Open the full link you were given (it ends in ?k=...).</p></body></html>")


def key():
    with open(KEYF) as f:
        return f.read().strip()


def same(a, b):
    return bool(a) and bool(b) and hmac.compare_digest(a.encode(), b.encode())


class H(http.server.SimpleHTTPRequestHandler):
    server_version = "slam-live/1"

    def __init__(self, *a, **kw):
        super().__init__(*a, directory=ROOT, **kw)

    def log_message(self, fmt, *args):      # quiet: one line per error only
        pass

    def given_key(self, q):
        k = (q.get("k") or [""])[0]
        if not k:
            for part in (self.headers.get("Cookie") or "").split(";"):
                n, _, v = part.strip().partition("=")
                if n == COOKIE:
                    k = urllib.parse.unquote(v)
        return k

    def send_json(self, code, obj):
        data = json.dumps(obj, separators=(",", ":")).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        u = urllib.parse.urlsplit(self.path)
        q = urllib.parse.parse_qs(u.query)
        k = self.given_key(q)
        try:
            with open(os.path.join(os.path.expanduser(os.environ.get("JOBS_DIR", "~/jobs")), "live_site_access.log"), "a") as f:   # no key value is ever written
                f.write("%s %s k_in_url=%s cookie=%s k_len=%d match=%s ua=%s\n" % (
                    time.strftime("%H:%M:%S"), u.path, "k" in q, "slam_k=" in (self.headers.get("Cookie") or ""),
                    len(k), same(k, key()), (self.headers.get("User-Agent") or "")[:40]))
        except OSError:
            pass
        if u.path == "/api/state":
            if not same(k, key()):
                return self.send_json(401, {"error": "this link needs its key"})
            try:
                with open(SNAP) as f:
                    stored = json.load(f)
            except (OSError, ValueError):
                return self.send_json(200, {"empty": True, "server_now": int(time.time() * 1000)})
            body = stored.get("body") or {}
            only = (q.get("only") or [""])[0]
            have_m, have_p = (q.get("m") or [""])[0], (q.get("p") or [""])[0]
            if only == "map":
                body.pop("jetson", None)
            for name, have in (("map", have_m), ("peer", have_p)):
                b = body.get(name)
                if isinstance(b, dict) and (only == "status" or (b.get("sha") and b.get("sha") == have)):
                    b["b64"] = None
                    b["same"] = True
            now = int(time.time() * 1000)
            return self.send_json(200, {"empty": False, "server_now": now,
                                        "age_s": (now - stored["received_at"]) / 1000,
                                        "received_at": stored["received_at"], "body": body})
        if u.path.startswith("/replay/api/"):          # Replay page: key first, then forward
            if not same(k, key()):
                return self.send_json(401, {"error": "this link needs its key"})
            return self.forward_to_replay(u, q)
        if u.path not in LIVE_PAGES and u.path != "/results.html" and not (u.path.startswith("/results/") and ".." not in u.path):
            self.send_response(404); self.send_header("Content-Length", "0"); self.end_headers(); return
        if not same(k, key()):
            self.send_response(401)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(DENIED)))
            self.end_headers()
            self.wfile.write(DENIED)
            return
        if "k" in q:                         # remember the key, then drop it from the address bar
            rest = {a: b for a, b in q.items() if a != "k"}
            loc = u.path + ("?" + urllib.parse.urlencode(rest, doseq=True) if rest else "")
            self.send_response(302)
            self.send_header("Set-Cookie", "%s=%s; Path=/; Max-Age=2592000; HttpOnly; Secure; SameSite=Lax"
                             % (COOKIE, urllib.parse.quote(k)))
            self.send_header("Location", loc)
            self.send_header("Content-Length", "0")
            self.end_headers()
            return
        rng = self.headers.get("Range", "")
        fpath = self.translate_path(u.path)
        if rng.startswith("bytes=") and os.path.isfile(fpath):     # videos: phones need partial downloads to play/seek
            size = os.path.getsize(fpath)
            a, _, b = rng[6:].split(",")[0].partition("-")
            try:
                start = int(a) if a else max(0, size - int(b))
                end = int(b) if (a and b) else size - 1
            except ValueError:
                start, end = 0, size - 1
            end = min(end, size - 1)
            if start > end:
                self.send_response(416); self.send_header("Content-Range", "bytes */%d" % size)
                self.send_header("Content-Length", "0"); self.end_headers(); return
            self.send_response(206)
            self.send_header("Content-Type", self.guess_type(fpath))
            self.send_header("Accept-Ranges", "bytes")
            self.send_header("Content-Range", "bytes %d-%d/%d" % (start, end, size))
            self.send_header("Content-Length", str(end - start + 1))
            self.end_headers()
            with open(fpath, "rb") as f:
                f.seek(start); left = end - start + 1
                while left > 0:
                    chunk = f.read(min(1 << 20, left))
                    if not chunk:
                        break
                    try:
                        self.wfile.write(chunk)
                    except (BrokenPipeError, ConnectionResetError):
                        return
                    left -= len(chunk)
            return
        return super().do_GET()

    def do_POST(self):
        # Only the Replay page's buttons send POST requests; every other address stays read-only.
        u = urllib.parse.urlsplit(self.path)
        q = urllib.parse.parse_qs(u.query)
        if not u.path.startswith("/replay/api/"):
            return self.send_json(405, {"error": "read-only address"})
        if not same(self.given_key(q), key()):
            return self.send_json(401, {"error": "this link needs its key"})
        return self.forward_to_replay(u, q)

    def forward_to_replay(self, u, q):
        """Pass one request on to the replay backend and its answer back, unchanged. The key is
        taken out of the address first; the backend never sees it."""
        body = None
        if self.command == "POST":
            length = int(self.headers.get("Content-Length") or 0)
            if length > REPLAY_MAX_BODY:
                return self.send_json(413, {"error": "request too large"})
            body = self.rfile.read(length)
        rest = urllib.parse.urlencode({a: b for a, b in q.items() if a != "k"}, doseq=True)
        req = urllib.request.Request(REPLAY_API + u.path + ("?" + rest if rest else ""), data=body,
                                     method=self.command)
        if body is not None:
            req.add_header("Content-Type", self.headers.get("Content-Type") or "")
        try:
            with urllib.request.urlopen(req, timeout=30) as r:
                code, ctype, data = r.status, r.headers.get("Content-Type", "application/json"), r.read()
        except urllib.error.HTTPError as e:
            code, ctype, data = e.code, e.headers.get("Content-Type", "application/json"), e.read()
        except OSError:
            return self.send_json(503, {"error": "the replay backend is not running on the Jetson"})
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def end_headers(self):
        self.send_header("Accept-Ranges", "bytes")
        self.send_header("X-Robots-Tag", "noindex, nofollow")
        super().end_headers()


class S(socketserver.ThreadingMixIn, http.server.HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


if __name__ == "__main__":
    key()                                   # fail at start if the key file is missing
    S(("127.0.0.1", PORT), H).serve_forever()
