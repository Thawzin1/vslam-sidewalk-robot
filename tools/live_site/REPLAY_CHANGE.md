# live_site_server.py: the change for the Replay page

After its existing view-key check, `live_site_server.py` forwards every `/replay/api/...` request
(GET and POST, with the key removed from the address) to the replay backend on
`127.0.0.1:$REPLAY_CONTROL_PORT` (default 8098, `tools/replay_web/replay_control_server.py`), and
serves `/replay.html` like the other live pages; every other POST is refused (405).
