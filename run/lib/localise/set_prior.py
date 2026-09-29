#!/usr/bin/env python3
"""Write a deliberately WRONG 'last localisation' (x y yaw_deg) into a WORKING COPY of a map db.
RTAB-Map reads Admin.opt_last_localization on load (Memory::loadOptimizedPoses -> Rtabmap.cpp:394) and,
in localisation mode with RGBD/StartAtOrigin=false, moves the robot there (Rtabmap.cpp:1345-1370).
Refuses any file whose resolved path is the master."""
import sqlite3, sys, os, math, numpy as np
db, x, y, yaw = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), math.radians(float(sys.argv[4]))
master = os.path.realpath(os.path.expanduser(os.environ.get('MAP_DB', '~/slam_series2/localise/s2_static_04_map.db')))
if os.path.realpath(db) == master: sys.exit('REFUSED: that is the master map')
c, s = math.cos(yaw), math.sin(yaw)
m = np.array([[c, -s, 0, x], [s, c, 0, y], [0, 0, 1, 0]], dtype=np.float32)
con = sqlite3.connect(db)
n = con.execute("update Admin set opt_last_localization=?", (m.tobytes(),)).rowcount
con.commit()
back = np.frombuffer(con.execute("select opt_last_localization from Admin").fetchone()[0], dtype=np.float32).reshape(3, 4)
print("rows=%d  prior now x=%.2f y=%.2f yaw=%.1f deg" % (n, back[0, 3], back[1, 3], math.degrees(math.atan2(back[1, 0], back[0, 0]))))
