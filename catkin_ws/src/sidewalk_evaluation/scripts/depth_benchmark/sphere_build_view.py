#!/usr/bin/env python3
"""sphere_build_view.py - watch the spheres build themselves out of real points.

WHAT THIS SHOWS THAT THE OTHER VIEWS DO NOT

    The box viewer shows every point in the search box. The algorithm dashboard
    shows one frame's accept/reject decision. Neither shows the thing the
    measurement is actually made of: THE POINTS THE DETECTOR ACCEPTED AS LYING
    ON A BALL, piling up over time until a ball's surface is drawn out in space.

    That is what this does. Every time the frozen detector accepts a frame it
    publishes the three sphere centres it fitted. This takes those centres, goes
    back to the points in the search box, and keeps the ones sitting within the
    detector's own 0.03 m inlier window of a ball's surface - the same window
    the detector used, not a lookalike. Those points accumulate, so the ball
    fills in as the run proceeds and you can see WHICH PART of each ball the
    camera has actually seen and which part it has never had a point on.

    It costs almost nothing. It does not re-run RANSAC - the detector has
    already done that, and repeating it would take processor time away from the
    run being watched. It only listens.

  the page: http://<jetson>:8096/

WHAT THE COVERAGE NUMBER MEANS

    A ball is only ever half visible - the camera cannot see its back. Coverage
    is the share of the FACING half whose surface has had at least one accepted
    point on it. Low coverage with a good centroid means the fit is being
    carried by a small patch, which is worth knowing: a sphere fitted to a small
    cap can slide along the viewing direction without the residuals objecting.

  usage, on the JETSON (camera, roi_viz and the detector already running):
    rosrun sidewalk_evaluation sphere_build_view.py _port:=8096
"""
from __future__ import print_function

import json
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

BALL_R = 0.20
INLIER = 0.03            # the detector's own window; do not change independently
KEEP = 9000              # accumulated points kept per ball
SEND = 2200              # points sent to the page per ball
NLAT, NLON = 12, 24      # coverage bins

_DT = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}

STATE = {'balls': {}, 'accepted': 0, 'last': 0.0, 'box_pts': 0, 'centroid': None,
         'started': time.time()}
LOCK = threading.Lock()


def xyz(msg):
    names = {f.name: f for f in msg.fields}
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
    out = []
    for k in ('x', 'y', 'z'):
        f = names[k]
        w = np.dtype(_DT[f.datatype]).itemsize
        col = raw[:, f.offset:f.offset + w]
        out.append(np.ascontiguousarray(col).view(_DT[f.datatype]).ravel().astype(np.float64))
    a = np.column_stack(out)
    return a[np.isfinite(a).all(axis=1)]


def label_of(centres):
    """Name the three balls stably: the highest is the apex, then left/right.

    They must keep the same names frame to frame or the accumulation smears two
    balls together.
    """
    order = np.argsort([c[2] for c in centres])
    low = sorted(order[:2], key=lambda i: -centres[i][1])   # +y is left
    return {int(low[0]): 'bot_left', int(low[1]): 'bot_right', int(order[2]): 'apex'}


class Ball(object):
    def __init__(self):
        self.pts = np.zeros((0, 3))
        self.centres = []
        self.bins = set()
        self.front = 0          # accepted points on the half the camera can see
        self.back = 0           # accepted points on the half it CANNOT

    def add(self, pts, centre):
        self.centres.append(centre)
        if len(pts):
            self.pts = np.vstack([self.pts, pts])
            if len(self.pts) > KEEP:
                sel = np.random.choice(len(self.pts), KEEP, replace=False)
                self.pts = self.pts[sel]
            d = pts - centre
            n = np.linalg.norm(d, axis=1)
            good = n > 1e-6
            u = d[good] / n[good][:, None]
            # SPLIT THE POINTS BY WHICH HALF OF THE BALL THEY SIT ON.
            #
            # A real point on a ball's surface must be on the half FACING the
            # camera - the ball is opaque, so the far side cannot be seen. Any
            # accepted point on the far side therefore is not ball surface at
            # all: it is something else that happens to lie within the
            # detector's 30 mm window of the sphere - the frame's own bar, the
            # cart, the floor. Counting those in the coverage is what made it
            # read 120-150 %, which is impossible and is why the number is now
            # split rather than patched.
            #
            # The far-side count is worth having on its own: it says how much of
            # what the fit is standing on is not the ball.
            v = centre / max(np.linalg.norm(centre), 1e-9)
            faces = u.dot(v) < 0.0
            self.front += int(faces.sum())
            self.back += int((~faces).sum())
            u = u[faces]
            if not len(u):
                return
            # EQUAL-AREA latitude bands: bin by the height on the unit sphere,
            # not by the angle. Binning by angle makes the bands near the poles
            # far smaller than the ones near the equator, so a facing hemisphere
            # covers MORE than half the bins and coverage reads over 100 % -
            # measured live at 122-150 % before this was fixed. Binning by u[:,2]
            # gives every band the same area (Archimedes), so a hemisphere is
            # exactly half the bins and the percentage means what it says.
            lat = np.clip(((np.clip(u[:, 2], -1, 1) + 1.0) / 2.0
                           * NLAT).astype(int), 0, NLAT - 1)
            lon = np.clip(((np.arctan2(u[:, 1], u[:, 0]) / (2 * np.pi) + 0.5)
                           * NLON).astype(int), 0, NLON - 1)
            for a, b in zip(lat.tolist(), lon.tolist()):
                self.bins.add((a, b))

    def stats(self):
        C = np.array(self.centres)
        s = C.std(axis=0, ddof=1) if len(C) > 1 else np.zeros(3)
        return {
            'n': len(C),
            'mean': C.mean(axis=0).round(4).tolist(),
            'sigma_mm': (s * 1000).round(2).tolist(),
            'rms3d_mm': round(float(np.sqrt((s ** 2).sum()) * 1000), 2),
            'dist_m': round(float(np.linalg.norm(C.mean(axis=0))), 4),
            'points': int(len(self.pts)),
            # only the facing half can ever be seen, so score against that
            'coverage_pct': round(min(100.0, 100.0 * len(self.bins)
                                      / (NLAT * NLON / 2.0)), 1),
            'off_ball_pct': round(100.0 * self.back / max(self.front + self.back, 1), 1),
        }


def main():
    import rospy
    from sensor_msgs.msg import PointCloud2
    from visualization_msgs.msg import MarkerArray

    rospy.init_node('sphere_build_view', anonymous=True)
    port = int(rospy.get_param('~port', 8096))
    inside_topic = rospy.get_param('~inside_topic', '/roi_viz_cam/inside')
    marker_topic = rospy.get_param('~marker_topic', '/sphere_centroid/markers')
    box = {'pts': None}

    def cloud_cb(m):
        try:
            box['pts'] = xyz(m)
            with LOCK:
                STATE['box_pts'] = int(len(box['pts']))
        except Exception:
            pass

    def marker_cb(ma):
        centres = [np.array([mk.pose.position.x, mk.pose.position.y,
                             mk.pose.position.z])
                   for mk in ma.markers if mk.ns == 'spheres']
        if len(centres) != 3 or box['pts'] is None or not len(box['pts']):
            return
        names = label_of(centres)
        P = box['pts']
        # distance from every point to every ball's surface, then keep the ones
        # inside the detector's own inlier window - the same 0.03 m it used.
        D = np.stack([np.abs(np.linalg.norm(P - c, axis=1) - BALL_R)
                      for c in centres], axis=1)
        who = np.argmin(D, axis=1)
        near = D[np.arange(len(P)), who] <= INLIER
        with LOCK:
            for i, c in enumerate(centres):
                nm = names[i]
                STATE['balls'].setdefault(nm, Ball()).add(P[near & (who == i)], c)
            STATE['accepted'] += 1
            STATE['last'] = time.time()
            STATE['centroid'] = np.mean(centres, axis=0).round(4).tolist()

    rospy.Subscriber(inside_topic, PointCloud2, cloud_cb, queue_size=1,
                     buff_size=2 ** 24)
    rospy.Subscriber(marker_topic, MarkerArray, marker_cb, queue_size=5)

    def payload():
        with LOCK:
            out = {'accepted': STATE['accepted'], 'box_pts': STATE['box_pts'],
                   'centroid': STATE['centroid'],
                   'age': round(time.time() - STATE['last'], 1) if STATE['last'] else None,
                   'elapsed_min': round((time.time() - STATE['started']) / 60.0, 1),
                   'inlier_mm': INLIER * 1000, 'radius': BALL_R, 'balls': {}}
            for nm, b in STATE['balls'].items():
                p = b.pts
                if len(p) > SEND:
                    p = p[np.random.choice(len(p), SEND, replace=False)]
                st = b.stats()
                st['pts'] = np.round(p, 4).tolist()
                out['balls'][nm] = st
        return out

    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            if self.path.startswith('/data'):
                body = json.dumps(payload()).encode()
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            else:
                body = PAGE.encode()
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)

    srv = ThreadingHTTPServer(('0.0.0.0', port), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print('sphere_build_view on http://0.0.0.0:%d/  (%s + %s)'
          % (port, inside_topic, marker_topic))
    sys.stdout.flush()
    rospy.spin()
    return 0


PAGE = r"""<!doctype html><html><head><meta charset="utf-8">
<title>Spheres building from real points</title>
<style>
 body{margin:0;background:#0f1412;color:#e7ebe8;
      font:14px/1.5 ui-monospace,SFMono-Regular,Menlo,monospace}
 header{padding:12px 18px;border-bottom:1px solid #2a3430;display:flex;
        gap:26px;align-items:baseline;flex-wrap:wrap}
 h1{font-size:15px;margin:0;letter-spacing:.06em;text-transform:uppercase}
 .k{color:#8b958f}
 #wrap{display:flex;flex-wrap:wrap;gap:14px;padding:14px 18px}
 canvas{background:#0b100e;border:1px solid #2a3430;border-radius:3px;
        touch-action:none;cursor:grab}
 #side{flex:1;min-width:290px;display:flex;flex-direction:column;gap:10px}
 .card{background:#18201d;border:1px solid #2a3430;border-radius:3px;padding:11px 13px}
 .card h2{margin:0 0 7px;font-size:13px;letter-spacing:.05em}
 table{border-collapse:collapse;width:100%;font-size:12.5px}
 td{padding:2px 0}td:last-child{text-align:right}
 .bar{height:7px;background:#0b100e;border-radius:4px;overflow:hidden;margin-top:5px}
 .bar>i{display:block;height:100%}
 .note{color:#8b958f;font-size:12px;padding:0 18px 18px;max-width:80ch;line-height:1.6}
 .stale{color:#e0714b}
</style></head><body>
<header>
 <h1>Spheres building from real points</h1>
 <span><span class="k">accepted frames</span> <b id="acc">0</b></span>
 <span><span class="k">points in box</span> <b id="bx">0</b></span>
 <span><span class="k">elapsed</span> <b id="el">0</b> min</span>
 <span id="age" class="k">waiting for the detector…</span>
</header>
<div id="wrap">
 <canvas id="c" width="640" height="560"></canvas>
 <div id="side"></div>
</div>
<p class="note">Every dot is a real camera point that the frozen detector accepted as lying on a
ball &mdash; inside its own 30 mm inlier window of the fitted surface. They accumulate, so each ball
draws itself out of the measurement as the run proceeds. The white wireframe is a true 0.20 m sphere drawn at the running mean
centre the detector fitted, over the top of the points, so you can see directly whether the fitted ball
sits where the measured surface actually is. Its white dot is the fitted centre. <b>Coverage</b> is the share of the ball&rsquo;s <i>facing</i> half that has had at
least one accepted point on it &mdash; the back is never visible. <b>Accepted but not on the ball</b>
counts points the detector took that sit on the far side: the ball is opaque, so those cannot be its
surface at all &mdash; they are the frame&rsquo;s bar, the cart or the floor happening to fall inside
the 30 mm window. It is a direct measure of what the fit is standing on besides the ball. Drag to rotate, wheel to zoom.</p>
<script>
const COL={bot_left:'#3fbbac',bot_right:'#e0a24c',apex:'#b98cff'};
let yaw=-0.6, pitch=0.25, zoom=1, data=null, drag=null;
const cv=document.getElementById('c'), cx=cv.getContext('2d');
cv.addEventListener('pointerdown',e=>{drag=[e.clientX,e.clientY];cv.setPointerCapture(e.pointerId)});
cv.addEventListener('pointerup',()=>drag=null);
cv.addEventListener('pointermove',e=>{if(!drag)return;
  yaw+=(e.clientX-drag[0])*0.008; pitch+=(e.clientY-drag[1])*0.008;
  pitch=Math.max(-1.5,Math.min(1.5,pitch)); drag=[e.clientX,e.clientY]; draw();});
cv.addEventListener('wheel',e=>{e.preventDefault();
  zoom*=Math.exp(-e.deltaY*0.0016); zoom=Math.max(0.25,Math.min(6,zoom)); draw();},{passive:false});

function centreOf(d){let s=[0,0,0],n=0;
  for(const k in d.balls){const m=d.balls[k].mean;s[0]+=m[0];s[1]+=m[1];s[2]+=m[2];n++;}
  return n?[s[0]/n,s[1]/n,s[2]/n]:[0,0,0];}
function proj(p,o,sc){
  const x=p[0]-o[0], y=p[1]-o[1], z=p[2]-o[2];
  const cy=Math.cos(yaw),sy=Math.sin(yaw);
  let X=x*cy - y*sy, Y=x*sy + y*cy;
  const cp=Math.cos(pitch),sp=Math.sin(pitch);
  let Z=z*cp - Y*sp; Y=z*sp + Y*cp;
  return [cv.width/2 + Y*sc, cv.height/2 - Z*sc, X];}
function draw(){
  cx.fillStyle='#0b100e'; cx.fillRect(0,0,cv.width,cv.height);
  if(!data||!Object.keys(data.balls).length){
    cx.fillStyle='#8b958f'; cx.font='13px monospace';
    cx.fillText('waiting for the detector to accept a frame…',24,30); return;}
  const o=centreOf(data), sc=190*zoom;
  // POINTS FIRST, then the fitted sphere drawn OVER them. The whole question
  // this view answers is whether the fitted ball sits where the real points
  // are, and a wireframe hidden behind a dense cloud cannot answer it.
  for(const k in data.balls){
    const b=data.balls[k];
    cx.fillStyle=COL[k]||'#ccc'; cx.globalAlpha=0.75;
    for(const p of b.pts){const q=proj(p,o,sc); cx.fillRect(q[0]-1,q[1]-1,2,2);}
    cx.globalAlpha=1;
  }
  for(const k in data.balls){
    const b=data.balls[k];
    cx.strokeStyle='#f2f5f3'; cx.globalAlpha=0.55; cx.lineWidth=1;
    for(let i=1;i<6;i++){const la=-Math.PI/2+i*Math.PI/6; cx.beginPath();
      for(let j=0;j<=48;j++){const lo=j/48*2*Math.PI;
        const p=[b.mean[0]+data.radius*Math.cos(la)*Math.cos(lo),
                 b.mean[1]+data.radius*Math.cos(la)*Math.sin(lo),
                 b.mean[2]+data.radius*Math.sin(la)];
        const q=proj(p,o,sc); j?cx.lineTo(q[0],q[1]):cx.moveTo(q[0],q[1]);}
      cx.stroke();}
    for(let j=0;j<12;j++){const lo=j/12*2*Math.PI; cx.beginPath();
      for(let i=0;i<=32;i++){const la=-Math.PI/2+i/32*Math.PI;
        const p=[b.mean[0]+data.radius*Math.cos(la)*Math.cos(lo),
                 b.mean[1]+data.radius*Math.cos(la)*Math.sin(lo),
                 b.mean[2]+data.radius*Math.sin(la)];
        const q=proj(p,o,sc); i?cx.lineTo(q[0],q[1]):cx.moveTo(q[0],q[1]);}
      cx.stroke();}
    cx.globalAlpha=1;
    const qc=proj(b.mean,o,sc);
    cx.fillStyle='#f2f5f3'; cx.beginPath(); cx.arc(qc[0],qc[1],2.5,0,7); cx.fill();
  }
  if(data.centroid){const q=proj(data.centroid,o,sc);
    cx.strokeStyle='#e0714b';cx.lineWidth=1.5;cx.beginPath();
    cx.arc(q[0],q[1],6,0,7);cx.stroke();
    cx.fillStyle='#e0714b';cx.font='11px monospace';
    cx.fillText('centroid',q[0]+10,q[1]+4);}
}
function side(){
  const s=document.getElementById('side'); s.innerHTML='';
  for(const k of ['apex','bot_left','bot_right']){
    const b=data.balls[k]; if(!b)continue;
    const d=document.createElement('div'); d.className='card';
    d.innerHTML='<h2 style="color:'+(COL[k]||'#ccc')+'">'+k+'</h2><table>'
     +'<tr><td class="k">distance</td><td>'+b.dist_m.toFixed(4)+' m</td></tr>'
     +'<tr><td class="k">wobble x / y / z</td><td>'+b.sigma_mm.map(v=>v.toFixed(1)).join(' / ')+' mm</td></tr>'
     +'<tr><td class="k">RMS in 3D</td><td>'+b.rms3d_mm.toFixed(2)+' mm</td></tr>'
     +'<tr><td class="k">fits</td><td>'+b.n+'</td></tr>'
     +'<tr><td class="k">accepted points held</td><td>'+b.points+'</td></tr>'
     +'<tr><td class="k">surface coverage</td><td>'+b.coverage_pct.toFixed(0)+' % of the facing half</td></tr>'
     +'<tr><td class="k">accepted but not on the ball</td><td>'+b.off_ball_pct.toFixed(1)+' %</td></tr>'
     +'</table><div class="bar"><i style="width:'+Math.min(100,b.coverage_pct)
     +'%;background:'+(COL[k]||'#888')+'"></i></div>';
    s.appendChild(d);}
}
async function tick(){
  try{const r=await fetch('/data',{cache:'no-store'}); data=await r.json();
    document.getElementById('acc').textContent=data.accepted;
    document.getElementById('bx').textContent=data.box_pts;
    document.getElementById('el').textContent=data.elapsed_min;
    const a=document.getElementById('age');
    if(data.age===null){a.textContent='waiting for the detector…';a.className='k';}
    else if(data.age>90){a.textContent='last acceptance '+data.age+' s ago';a.className='stale';}
    else {a.textContent='last acceptance '+data.age+' s ago';a.className='k';}
    side(); draw();
  }catch(e){}
  setTimeout(tick,2500);
}
tick();
</script></body></html>"""


if __name__ == '__main__':
    sys.exit(main())
