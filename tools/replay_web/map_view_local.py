#!/usr/bin/env python3
"""map_view_local.py - the live map picture server, reachable only from this computer.

WHAT IT DOES
    Runs the project's live map server (catkin_ws/src/sidewalk_slam/scripts/live_map_server.py)
    unchanged, except that its web server listens on 127.0.0.1 instead of every network address.
    A replay's map picture is shown to people only through the live website, which checks the view
    key first, so the picture server itself must not be reachable from the network.

    How: live_map_server.py builds its web server with `HTTPServer((address, port), Handler)`.
    This file imports that script as a module and swaps in a HTTPServer that always binds to
    127.0.0.1, then calls its main(). Nothing in the original file is edited.

HOW TO RUN (with a ROS environment sourced, and ROS_MASTER_URI pointing at the replay's master)
    python3 map_view_local.py _port:=8195
    Every other ROS parameter of live_map_server.py works the same way (_grid:=..., _scale:=...).

INPUTS    the replay's ROS topics (/rtabmap/grid_map, /rtabmap/info, TF map -> base_link)
OUTPUTS   http://127.0.0.1:<port>/map.png (the map picture) and /stats.json (its status lines)

SETTINGS  REPO_ROOT (environment): the repository that holds live_map_server.py
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.expanduser(os.environ.get("REPO_ROOT", os.path.dirname(os.path.dirname(HERE))))
SCRIPTS = os.path.join(REPO_ROOT, "catkin_ws", "src", "sidewalk_slam", "scripts")
sys.path.insert(0, SCRIPTS)

import live_map_server  # noqa: E402  (imported after its folder is on the path)

ServerClass = live_map_server.HTTPServer


def local_only_server(address, handler):
    """Same server class, same port, but only this computer can connect."""
    _, port = address
    return ServerClass(("127.0.0.1", port), handler)


live_map_server.HTTPServer = local_only_server

if __name__ == "__main__":
    live_map_server.main()
