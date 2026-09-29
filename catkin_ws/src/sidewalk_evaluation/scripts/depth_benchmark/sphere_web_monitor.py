#!/usr/bin/env python3
"""sphere_web_monitor.py - watch a sphere-benchmark run in the browser instead of RViz.

WHY THIS EXISTS

    The user monitors runs from their PC over the network. RViz needs a forwarded
    X display (fragile, and the exact opposite SSH settings from the camera), and
    drawing the full camera cloud in RViz once stalled this Jetson for ~11 GB.
    This serves the same information as the sphere_test.rviz layout - the search
    box, the points inside it, the fitted spheres, plus the live statistics RViz
    never showed - as a single self-contained web page:

        http://<jetson>:8093/

    Over the direct ethernet cable the dense view is affordable; on lab WiFi the
    default (thinned) view is still smooth. Nothing here touches the recording
    pipeline - it only listens.

WHAT THE PAGE SHOWS

    - a rotatable 3D view (drag = rotate, wheel = zoom) of the points inside the
      search box, coloured by height
    - the search box as a wireframe, read live from /roi_viz/box
    - the fitted spheres from /sphere_centroid/markers when a frame is accepted
    - a stats strip computed from the run's CSV: samples so far, wobble per axis
      (standard deviation - how much the answer moves when nothing moves),
      RMS3D, triangle sides, points per sphere, acceptance pace and time left
    - a LIVE / STALE flag - STALE in red means no cloud has arrived for 5 s,
      so a silently dead camera is visible, not quiet

  usage, on the JETSON (camera + roi_viz already running):
    rosrun sidewalk_evaluation sphere_web_monitor.py \
        _csv_path:=/media/sidewalk/SIDEWALK128/ZEDX_vs_LiDAR/static_spheres/5m_zedx.csv

    _csv_path may be omitted: the monitor then picks the newest CSV under
    _csv_dir (default: the microSD ZEDX_vs_LiDAR tree, falling back to the USB).
"""
from __future__ import print_function

import glob
import json
import math
import os
import re
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray

_DT = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}

CSV_COLS = ('n', 'x', 'y', 'z', 'base', 'leg1', 'leg2',
            'n_bot1', 'n_bot2', 'n_apex', 'tx', 'ty', 'tz', 'stamp')


class State(object):
    """Everything the page needs, guarded by one lock."""

    def __init__(self):
        self.lock = threading.Lock()
        self.pts = np.zeros((0, 3), np.float32)   # in-box points, latest frame
        self.pts_time = 0.0
        self.box = None       # (cx,cy,cz,sx,sy,sz) from the roi_viz CUBE marker
        self.spheres = []     # [(x,y,z,r), ...] latest accepted fit
        self.spheres_time = 0.0
        self.csv_path = None
        self.csv_rows = 0
        self.stats = {}


S = State()


def cloud_cb(msg):
    names = [f.name for f in msg.fields]
    dt = np.dtype(dict(names=names,
                       formats=[_DT[f.datatype] for f in msg.fields],
                       offsets=[f.offset for f in msg.fields],
                       itemsize=msg.point_step))
    a = np.frombuffer(msg.data, dtype=dt)
    xyz = np.stack([a['x'], a['y'], a['z']], axis=1).astype(np.float32)
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    with S.lock:
        S.pts = xyz
        S.pts_time = time.time()


def box_cb(msg):
    for m in msg.markers:
        if m.type == 1:  # CUBE
            with S.lock:
                S.box = (m.pose.position.x, m.pose.position.y, m.pose.position.z,
                         m.scale.x, m.scale.y, m.scale.z)
            return


def spheres_cb(msg):
    sph = []
    for m in msg.markers:
        if m.type == 2:  # SPHERE
            sph.append((m.pose.position.x, m.pose.position.y,
                        m.pose.position.z, m.scale.x / 2.0))
    if sph:
        with S.lock:
            S.spheres = sph
            S.spheres_time = time.time()


def find_csv(csv_dir):
    """Newest CSV under the recording tree - so the monitor needs no per-run setup."""
    cands = []
    for d in csv_dir.split(':'):
        cands += glob.glob(os.path.join(d, '**', '*.csv'), recursive=True)
    return max(cands, key=os.path.getmtime) if cands else None


def sibling_parts(path):
    """4m_part2_zedx.csv -> every 4m_part*_zedx.csv beside it, oldest first.

    A run interrupted mid-way continues in a fresh partN file (the recorder
    opens its CSV with 'w' and would destroy the earlier samples otherwise).
    The first version of this page showed only the newest fragment, so after
    the 4 m run's network interruption the user watched the count restart at
    zero on a run that was actually 82/200 - the page must count the RUN."""
    m = re.match(r'(.+)_part\d+(_[A-Za-z0-9]+\.csv)$', os.path.basename(path))
    if not m:
        return [path]
    pat = os.path.join(os.path.dirname(path),
                       '%s_part*%s' % (m.group(1), m.group(2)))
    return sorted(glob.glob(pat)) or [path]


def csv_stats(paths, target):
    """Live figures from the run CSV (all parts of it). Reads whole files each
    time - a few hundred kilobytes at most, and re-reading beats a tail parser
    that can desynchronise if the recorder is restarted mid-run."""
    try:
        rows = []
        for path in paths:
            with open(path) as f:
                header = f.readline().strip().split(',')
                idx = {c: header.index(c) for c in CSV_COLS if c in header}
                for line in f:
                    p = line.strip().split(',')
                    if len(p) == len(header):
                        rows.append(p)
        if not rows:
            return {'n': 0}
        a = np.array([[float(r[idx[c]]) for c in
                       ('x', 'y', 'z', 'base', 'leg1', 'leg2',
                        'n_bot1', 'n_bot2', 'n_apex', 'stamp')] for r in rows])
        # SAMPLE standard deviation (divide by n-1), matching the camera depth study and
        # compare_sensors.py exactly - with plain .std() this page would read
        # 14.85 mm where the run's official figure is 14.92 mm, and a monitor
        # that disagrees with the record by even that much invites doubt.
        dd = 1 if len(rows) > 1 else 0
        sx, sy, sz = (float(a[:, i].std(ddof=dd)) for i in (0, 1, 2))
        # Pace: MEDIAN gap between recent samples, not (last-first)/count. With a
        # multi-part run the window can straddle the interruption, and one ten-
        # minute hole in an average would report a nonsense ETA; the median of
        # the individual gaps ignores it.
        t = np.sort(a[:, 9])
        gaps = np.diff(t[-21:]) if len(t) > 1 else np.array([0.0])
        pace = float(np.median(gaps))
        left = max(0, target - len(rows))
        return {
            'n': len(rows), 'target': target,
            'sx_mm': sx * 1000, 'sy_mm': sy * 1000, 'sz_mm': sz * 1000,
            'rms3d_mm': math.sqrt(sx * sx + sy * sy + sz * sz) * 1000,
            'mean_x': float(a[:, 0].mean()), 'mean_y': float(a[:, 1].mean()),
            'mean_z': float(a[:, 2].mean()),
            'sides': [float(a[:, i].mean()) for i in (3, 4, 5)],
            'pts_sphere': float(a[:, 6:9].mean()),
            'sec_per_sample': pace,
            'eta_min': left * pace / 60.0 if pace > 0 else None,
        }
    except (OSError, ValueError, KeyError):
        return {'n': 0}


PAGE = """<!DOCTYPE html><html><head><meta charset="utf-8">
<title>Sphere benchmark - live</title><style>
body{margin:0;background:#101418;color:#dde3e8;font:14px/1.4 system-ui,sans-serif}
#bar{display:flex;flex-wrap:wrap;gap:4px 18px;padding:8px 12px;background:#1a2027;align-items:baseline}
#bar b{color:#4fd8c8}.st{padding:2px 8px;border-radius:4px;font-weight:700}
.live{background:#1d5c46}.stale{background:#8a2b2b}
canvas{display:block;width:100vw;height:calc(100vh - 90px);cursor:grab}
#hint{position:fixed;bottom:6px;left:12px;color:#5a6672;font-size:12px}
#dense{position:fixed;bottom:6px;right:12px}</style></head><body>
<div id="bar">loading...</div><canvas id="c"></canvas>
<div id="hint">drag = rotate &middot; wheel = zoom &middot; teal box = search region &middot; rings = fitted spheres</div>
<label id="dense"><input type="checkbox" id="dchk"> dense view (ethernet only)</label>
<script>
const cv=document.getElementById('c'),cx2=cv.getContext('2d');
let yaw=2.6,pitch=0.35,dist=7,cen=[4,0,0],drag=null,data=null;
cv.onmousedown=e=>{drag=[e.clientX,e.clientY];cv.style.cursor='grabbing'};
window.onmouseup=()=>{drag=null;cv.style.cursor='grab'};
window.onmousemove=e=>{if(!drag)return;yaw-=(e.clientX-drag[0])*.008;
pitch=Math.max(-1.4,Math.min(1.4,pitch+(e.clientY-drag[1])*.008));drag=[e.clientX,e.clientY];draw()};
cv.onwheel=e=>{e.preventDefault();dist=Math.max(1.5,Math.min(30,dist*(e.deltaY>0?1.1:0.9)));draw()};
function proj(p){const cy=Math.cos(yaw),sy=Math.sin(yaw),cp=Math.cos(pitch),sp=Math.sin(pitch);
const x=p[0]-cen[0],y=p[1]-cen[1],z=p[2]-cen[2];
const x1=cy*x+sy*y,y1=-sy*x+cy*y,z1=cp*z-sp*x1,d=cp*x1+sp*z+dist;
if(d<=.1)return null;const s=Math.min(cv.width,cv.height)*.9/d;
return[cv.width/2+y1*s,cv.height/2-z1*s,s]}
function draw(){cv.width=cv.clientWidth;cv.height=cv.clientHeight;
cx2.fillStyle='#101418';cx2.fillRect(0,0,cv.width,cv.height);if(!data)return;
if(data.box){cen=[data.box[0],data.box[1],data.box[2]];
const[bx,by,bz,sx,sy,sz]=data.box,cs=[];
for(const dx of[-1,1])for(const dy of[-1,1])for(const dz of[-1,1])
cs.push(proj([bx+dx*sx/2,by+dy*sy/2,bz+dz*sz/2]));
cx2.strokeStyle='#2a8f83';cx2.lineWidth=1;
const E=[[0,1],[0,2],[1,3],[2,3],[4,5],[4,6],[5,7],[6,7],[0,4],[1,5],[2,6],[3,7]];
for(const[a,b]of E){if(cs[a]&&cs[b]){cx2.beginPath();cx2.moveTo(cs[a][0],cs[a][1]);
cx2.lineTo(cs[b][0],cs[b][1]);cx2.stroke()}}}
const P=data.pts;let zmin=1e9,zmax=-1e9;
for(let i=0;i<P.length;i+=3){if(P[i+2]<zmin)zmin=P[i+2];if(P[i+2]>zmax)zmax=P[i+2]}
const zr=Math.max(1e-6,zmax-zmin);
for(let i=0;i<P.length;i+=3){const q=proj([P[i],P[i+1],P[i+2]]);if(!q)continue;
const t=(P[i+2]-zmin)/zr;
cx2.fillStyle='hsl('+(220-180*t)+',75%,'+(38+30*t)+'%)';
cx2.fillRect(q[0],q[1],1.6,1.6)}
if(data.spheres){cx2.strokeStyle='#e06a4e';cx2.lineWidth=2;
for(const s of data.spheres){const q=proj([s[0],s[1],s[2]]);if(!q)continue;
cx2.beginPath();cx2.arc(q[0],q[1],Math.max(3,s[3]*q[2]),0,6.283);cx2.stroke()}}}
function fmt(v,d){return v==null?'-':v.toFixed(d)}
async function tick(){try{
const r=await fetch('/data.json'+(document.getElementById('dchk').checked?'?dense=1':''));
data=await r.json();const s=data.stats,live=data.age<45;
document.getElementById('bar').innerHTML=
'<span class="st '+(live?'live">LIVE':'stale">STALE '+data.age.toFixed(0)+'s')+'</span>'+
'<span>samples <b>'+s.n+(s.target?' / '+s.target:'')+'</b></span>'+
'<span>&sigma; x/y/z <b>'+fmt(s.sx_mm,2)+' / '+fmt(s.sy_mm,2)+' / '+fmt(s.sz_mm,2)+' mm</b></span>'+
'<span>RMS3D <b>'+fmt(s.rms3d_mm,2)+' mm</b></span>'+
'<span>sides <b>'+(s.sides?s.sides.map(v=>v.toFixed(3)).join(' / '):'-')+' m</b></span>'+
'<span>pts/sphere <b>'+fmt(s.pts_sphere,0)+'</b></span>'+
'<span>pace <b>'+fmt(s.sec_per_sample,1)+' s</b></span>'+
'<span>time left <b>'+(s.eta_min!=null?s.eta_min.toFixed(0)+' min':'-')+'</b></span>'+
'<span style="color:#5a6672">'+(data.csv||'no CSV yet')+'</span>';
draw()}catch(e){}setTimeout(tick,700)}
tick();window.onresize=draw;
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, body, ctype):
        self.send_response(200)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path.startswith('/data.json'):
            dense = 'dense=1' in self.path
            with S.lock:
                pts, box, sph = S.pts, S.box, list(S.spheres)
                age = time.time() - S.pts_time if S.pts_time else 1e9
                # spheres older than 4 s mean the last frames were rejected - hide them
                if time.time() - S.spheres_time > 4.0:
                    sph = []
                csv_path, stats = S.csv_path, dict(S.stats)
            cap = 60000 if dense else 6000
            if len(pts) > cap:
                pts = pts[np.random.default_rng(0).choice(len(pts), cap, replace=False)]
            body = json.dumps({
                'pts': np.round(pts, 3).ravel().tolist(),
                'box': box, 'spheres': sph, 'age': round(age, 1),
                'csv': csv_path,
                'stats': stats,
            }).encode()
            self._send(body, 'application/json')
        else:
            self._send(PAGE.encode(), 'text/html; charset=utf-8')


def stats_loop(csv_path, csv_dir, target):
    while not rospy.is_shutdown():
        path = csv_path or find_csv(csv_dir)
        parts = sibling_parts(path) if path else []
        st = csv_stats(parts, target) if parts else {'n': 0}
        if len(parts) > 1:
            label = '%s (%d parts)' % (os.path.basename(parts[-1]), len(parts))
        else:
            label = os.path.basename(path) if path else None
        with S.lock:
            S.csv_path = label
            S.stats = st
        time.sleep(2.0)


def main():
    rospy.init_node('sphere_web_monitor')
    g = rospy.get_param
    port = int(g('~port', 8093))
    topic_in = g('~inside_topic', '/roi_viz/inside')
    csv_path = g('~csv_path', '')
    csv_dir = g('~csv_dir',
                '/media/sidewalk/SIDEWALK128/ZEDX_vs_LiDAR'
                ':/media/sidewalk/USB/centroid/ThawZin_ZEDX')
    target = int(g('~target', 200))

    rospy.Subscriber(topic_in, PointCloud2, cloud_cb, queue_size=1,
                     buff_size=2 ** 24)
    rospy.Subscriber('/roi_viz/box', MarkerArray, box_cb, queue_size=1)
    rospy.Subscriber('/sphere_centroid/markers', MarkerArray, spheres_cb,
                     queue_size=1)

    threading.Thread(target=stats_loop, args=(csv_path or None, csv_dir, target),
                     daemon=True).start()
    srv = ThreadingHTTPServer(('0.0.0.0', port), Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    rospy.loginfo('sphere_web_monitor: http://0.0.0.0:%d/  watching %s',
                  port, topic_in)
    rospy.spin()
    srv.shutdown()


if __name__ == '__main__':
    main()
