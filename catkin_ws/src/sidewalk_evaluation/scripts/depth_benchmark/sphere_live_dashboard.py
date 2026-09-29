#!/usr/bin/env python3
"""sphere_live_dashboard.py - watch the detector decide, live, and see WHY.

WHY THIS EXISTS

    The detector reports a centroid when it succeeds and says nothing at all
    when it fails. On 2026-09-07 it produced "no frame detected" 1948 times in
    a row on the Ouster and, separately, sat silent for ten minutes on the
    camera in QUALITY depth mode, with 15,000 points sitting in its search box.
    Neither told anyone which stage was throwing the frame away.

    Blind numbers cost three wrong diagnoses in one afternoon - a search box
    that was supposedly clipping, a calibration offset, a contamination tail -
    every one of which a picture of the accepted and rejected points would have
    settled in seconds.

    So: every frame is walked through the detector's own pipeline, the FIRST
    gate it fails is recorded, and the whole thing is served as a live page.

        http://<host>:8095/

    A frame lands in exactly one bucket, and the buckets are in pipeline order,
    so reading the table top to bottom walks a frame through the detector and
    shows where the population drains away:

      EMPTY_BOX       too few points in the search box to hold three balls
      NO_SPHERE       points there, but nothing ball-shaped came out of RANSAC
      FEW_CANDIDATES  balls found, but no three of them are the right distances
                      apart - the frame is looking at the wrong thing
      SHAPE_GATE      three balls at the right distances, but standing wrong -
                      bottom pair not level, or apex at the wrong height
      ACCEPTED        recorded

    It also plots the NEAR-MISS distance: the smallest side-length mismatch
    found over every triple considered, against the 0.06 m tolerance. A frame
    missing by 0.07 m is a hair outside; one missing by 0.9 m was never looking
    at the frame at all. That single number is what separates "our gate is too
    tight" from "the camera cannot see it".

    Nothing here touches the recording pipeline. It only listens, and it uses
    Nicolas's detector functions unmodified, so a frame it calls ACCEPTED is one
    the live node would also have accepted.

  usage, on the JETSON:
    rosrun sidewalk_evaluation sphere_live_dashboard.py _roi_x:=9.0 _sensor_h:=0.6972
"""
from __future__ import print_function

import json
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import itertools  # noqa: E402
from sphere_reject_diagnose import BUCKETS, DET  # noqa: E402
from sphere_centroid import (ransac_sphere, validate_triangle,  # noqa: E402
                             expected_sides)


def classify_verbose(pts, rng):
    """classify(), but it also hands back the geometry so it can be DRAWN.

    sphere_reject_diagnose.classify() throws away the candidate spheres and
    which points belonged to them - fine for counting, useless for looking. The
    logic below is the same walk through the same gates, calling the same
    unmodified functions from sphere_centroid.py, but it keeps:

        cands   the centre of every ball-shaped thing RANSAC found
        labels  for every point, which candidate claimed it (-1 = nothing did)
        chosen  the three candidates that passed, if any

    so the page can colour each point by the decision that was made about it.
    """
    n = len(pts)
    labels = np.full(n, -1, dtype=np.int16)
    if n < 3 * DET['min_inliers']:
        return 'EMPTY_BOX', [], labels, None, float('nan')

    idx = np.arange(n)
    remaining = pts
    rem_idx = idx
    cands = []
    for k in range(DET['max_candidates']):
        res = ransac_sphere(remaining, DET['radius'], DET['radius_tol'],
                            DET['thresh'], DET['iters'], DET['min_inliers'], rng)
        if res is None:
            break
        c, _, mask = res
        cands.append(c)
        labels[rem_idx[mask]] = k
        remaining = remaining[~mask]
        rem_idx = rem_idx[~mask]

    if len(cands) < 3:
        return 'NO_SPHERE', cands, labels, None, float('nan')

    exp = expected_sides(DET['base'], DET['rise'])
    best_err = float('inf')
    passed_sides = False
    chosen = None
    best_acc = None
    for combo in itertools.combinations(range(len(cands)), 3):
        P = np.array([cands[i] for i in combo])
        ok, d = validate_triangle(P, DET['base'], DET['rise'], DET['tol'])
        err = float(np.abs(np.sort(d) - exp).sum())
        best_err = min(best_err, err)
        if not ok:
            continue
        passed_sides = True
        apex = int(np.argmax(P[:, 2]))
        bottom = sorted([i for i in range(3) if i != apex], key=lambda i: P[i, 1])
        Po = P[[bottom[0], bottom[1], apex]]
        if abs(Po[0, 2] - Po[1, 2]) > DET['z_tol']:
            continue
        if abs((Po[2, 2] - 0.5 * (Po[0, 2] + Po[1, 2])) - DET['rise']) > DET['z_tol']:
            continue
        if best_acc is None or err < best_acc:
            best_acc = err
            chosen = list(combo)

    if chosen is not None:
        return 'ACCEPTED', cands, labels, chosen, best_err
    if passed_sides:
        return 'SHAPE_GATE', cands, labels, None, best_err
    return 'FEW_CANDIDATES', cands, labels, None, best_err

_DT = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}

S = {'counts': {b: 0 for b in BUCKETS}, 'total': 0, 'roi_pts': 0, 'cands': 0,
     'best_err': float('nan'), 'recent': [], 'errs': [], 'last': 0.0,
     'roi': [0, 0, 0, 0, 0, 0], 'mode': '?', 'hz': 0.0,
     'pts': [], 'lab': [], 'spheres': [], 'chosen': [], 'verdict': '-'}
LOCK = threading.Lock()


def decode(msg):
    names = {f.name: f for f in msg.fields}
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
    out = []
    for k in ('x', 'y', 'z'):
        f = names[k]
        col = raw[:, f.offset:f.offset + np.dtype(_DT[f.datatype]).itemsize]
        out.append(np.ascontiguousarray(col).view(_DT[f.datatype]).ravel())
    xyz = np.stack(out, axis=1).astype(np.float64)
    return xyz[np.isfinite(xyz).all(axis=1)]


PAGE = """<!doctype html><html><head><meta charset=utf-8>
<title>sphere detector - live decisions</title><style>
body{margin:0;background:#0f1316;color:#dfe5e9;font:14px/1.5 system-ui,sans-serif}
#top{padding:10px 16px;background:#182026;display:flex;gap:24px;flex-wrap:wrap;align-items:baseline}
#top b{color:#4fd8c8;font-family:ui-monospace,monospace}
.wrap{padding:16px;max-width:1000px}
h2{font-size:15px;font-weight:600;margin:20px 0 6px;color:#9fb0ba}
table{border-collapse:collapse;width:100%}
td,th{padding:6px 9px;text-align:left;border-bottom:1px solid #222c33}
th{font-size:11px;letter-spacing:.09em;text-transform:uppercase;color:#7b8b95;font-weight:500}
td.n{font-family:ui-monospace,monospace;text-align:right;white-space:nowrap}
.bar{height:9px;border-radius:5px;background:#222c33;overflow:hidden}
.bar i{display:block;height:100%}
.verdict{font-family:ui-monospace,monospace;font-weight:600}
.ACCEPTED{color:#5fd08a}.SHAPE_GATE{color:#e0b45c}.FEW_CANDIDATES{color:#e08a5c}
.NO_SPHERE{color:#e06a6a}.EMPTY_BOX{color:#9a6ae0}
.hint{color:#8fa0aa;font-size:13px;margin:4px 0 0}
#hist{display:flex;align-items:flex-end;gap:2px;height:100px;margin-top:8px}
#hist div{flex:1;background:#4fd8c8;min-height:1px}
#hist div.over{background:#e06a6a}
.stale{color:#e06a6a;font-weight:600}
#view{background:#0b0e10;border:1px solid #222c33;border-radius:8px;cursor:grab;display:block;width:100%}
#view:active{cursor:grabbing}
.key{display:flex;gap:18px;flex-wrap:wrap;margin-top:8px;font-size:13px;color:#9fb0ba}
.key span{display:inline-flex;align-items:center;gap:7px}
.sw{width:11px;height:11px;border-radius:99px;display:inline-block}
</style></head><body>
<div id=top></div>
<div class=wrap>

<h2>The points, coloured by what the detector decided about each one</h2>
<p class=hint>Drag to rotate, scroll to zoom. These are the actual points inside the
search box on the most recent frame. The camera is at the left, looking right.</p>
<canvas id=view height=430></canvas>
<div class=key>
  <span><i class=sw style="background:#5fd08a"></i>on a ball that was accepted</span>
  <span><i class=sw style="background:#e08a5c"></i>on a ball-shaped thing that was rejected</span>
  <span><i class=sw style="background:#4a565e"></i>not on any ball at all</span>
  <span><i class=sw style="background:#4fd8c8"></i>fitted ball outline</span>
  <span><i class=sw style="background:#2e3a42"></i>search box</span>
</div>

<h2>Where every frame stops</h2>
<p class=hint>A frame is put in the FIRST gate it fails, so this table is the
detector's pipeline read top to bottom.</p>
<table id=buckets></table>

<h2>How near the misses were</h2>
<p class=hint>The smallest side-length mismatch found over every triple the
detector considered, against its 0.06 m tolerance. Bars in red were never looking
at the right object; bars just short of the red are frames the tolerance threw away.</p>
<div id=hist></div>
<div id=histax class=hint></div>

<h2>Last 12 frames</h2>
<table id=recent></table>
</div>
<script>
var D=null, yaw=-0.55, pitch=0.30, zoom=1.0, drag=null;
var cv=document.getElementById('view');
function fit(){cv.width=cv.clientWidth;}
window.addEventListener('resize',function(){fit();render();});
cv.addEventListener('mousedown',function(e){drag=[e.clientX,e.clientY];});
window.addEventListener('mouseup',function(){drag=null;});
window.addEventListener('mousemove',function(e){
  if(!drag)return;
  yaw+=(e.clientX-drag[0])*0.008; pitch+=(e.clientY-drag[1])*0.008;
  pitch=Math.max(-1.4,Math.min(1.4,pitch)); drag=[e.clientX,e.clientY]; render();});
cv.addEventListener('wheel',function(e){e.preventDefault();
  zoom*=Math.exp(-e.deltaY*0.0012); zoom=Math.max(0.3,Math.min(6,zoom)); render();},{passive:false});

function proj(p,c,s){
  var x=p[0]-c[0], y=p[1]-c[1], z=p[2]-c[2];
  var cy=Math.cos(yaw), sy=Math.sin(yaw);
  var X=x*cy - y*sy, Y=x*sy + y*cy;
  var cp=Math.cos(pitch), sp=Math.sin(pitch);
  var Z=Y*sp + z*cp, Yp=Y*cp - z*sp;
  return [cv.width/2 + X*s, cv.height/2 - Z*s, Yp];
}
function render(){
  var g=cv.getContext('2d');
  g.clearRect(0,0,cv.width,cv.height);
  if(!D||!D.pts||!D.pts.length){
    g.fillStyle='#5b6b75';g.font='14px system-ui';
    g.fillText('waiting for a frame...',16,28);return;}
  var r=D.roi, c=[(r[0]+r[1])/2,(r[2]+r[3])/2,(r[4]+r[5])/2];
  var span=Math.max(r[1]-r[0], r[3]-r[2], r[5]-r[4]);
  var s=zoom*Math.min(cv.width,cv.height)/(span*1.5);
  // search box wireframe
  var V=[],i;
  for(i=0;i<8;i++)V.push([r[i&1?1:0], r[i&2?3:2], r[i&4?5:4]]);
  var E=[[0,1],[0,2],[0,4],[1,3],[1,5],[2,3],[2,6],[3,7],[4,5],[4,6],[5,7],[6,7]];
  g.strokeStyle='#2e3a42';g.lineWidth=1;
  E.forEach(function(e){var a=proj(V[e[0]],c,s),b=proj(V[e[1]],c,s);
    g.beginPath();g.moveTo(a[0],a[1]);g.lineTo(b[0],b[1]);g.stroke();});
  // points, far ones drawn first so near ones sit on top
  var ch=D.chosen||[], idx=D.pts.map(function(p,i){return i;});
  var pr=D.pts.map(function(p){return proj(p,c,s);});
  idx.sort(function(a,b){return pr[b][2]-pr[a][2];});
  idx.forEach(function(i){
    var l=D.lab[i], col;
    if(l<0) col='#4a565e';
    else if(ch.indexOf(l)>=0) col='#5fd08a';
    else col='#e08a5c';
    g.fillStyle=col;
    g.fillRect(pr[i][0]-1,pr[i][1]-1,2.4,2.4);
  });
  // fitted ball outlines
  (D.spheres||[]).forEach(function(q,k){
    var a=proj(q,c,s);
    g.strokeStyle = ch.indexOf(k)>=0 ? '#5fd08a' : '#4fd8c8';
    g.lineWidth = ch.indexOf(k)>=0 ? 2 : 1;
    g.beginPath();g.arc(a[0],a[1],D.radius*s,0,6.2832);g.stroke();
    g.fillStyle=g.strokeStyle;g.font='11px ui-monospace';
    g.fillText(String(k),a[0]+D.radius*s+3,a[1]);
  });
  g.fillStyle='#8fa0aa';g.font='12px ui-monospace';
  g.fillText('verdict: '+D.verdict+'   points shown: '+D.pts.length+
             '   balls found: '+(D.spheres||[]).length, 10, cv.height-10);
}
function draw(d){
  D=d;
  // The detector itself runs at a fraction of a hertz, so a few seconds
  // between frames is normal here. Judge staleness against ITS rate, not a
  // fixed clock, or the page cries wolf on every healthy frame.
  var age=(Date.now()/1000)-d.last;
  var gap=d.hz>0.02?(3/d.hz):15, stale=age>Math.max(8,gap);
  document.getElementById('top').innerHTML =
    '<span>'+(stale?'<span class=stale>STALE '+age.toFixed(0)+'s</span>':'<b>LIVE</b>')+'</span>'+
    '<span>frames <b>'+d.total+'</b></span>'+
    '<span>accepted <b>'+d.counts.ACCEPTED+'</b> ('+(d.total?(100*d.counts.ACCEPTED/d.total).toFixed(1):0)+'%)</span>'+
    '<span>points in box <b>'+d.roi_pts+'</b></span>'+
    '<span>balls found <b>'+d.cands+'</b> of 3 needed</span>'+
    '<span>rate <b>'+d.hz.toFixed(2)+'</b> Hz</span>';
  var order=['EMPTY_BOX','NO_SPHERE','FEW_CANDIDATES','SHAPE_GATE','ACCEPTED'];
  var why={EMPTY_BOX:'too few points in the box to hold three balls',
    NO_SPHERE:'points there, but nothing ball-shaped came out',
    FEW_CANDIDATES:'balls found, but not the right distances apart',
    SHAPE_GATE:'right distances, but standing wrong (tilt or height)',
    ACCEPTED:'recorded'};
  var h='<tr><th>stage</th><th>what it means</th><th>frames</th><th>share</th></tr>';
  order.forEach(function(b){
    var cc=d.counts[b]||0, p=d.total?100*cc/d.total:0;
    h+='<tr><td class="verdict '+b+'">'+b+'</td><td>'+why[b]+'</td><td class=n>'+cc+
       '</td><td style="width:30%"><div class=bar><i style="width:'+p.toFixed(1)+
       '%;background:'+(b==='ACCEPTED'?'#5fd08a':'#e08a5c')+'"></i></div>'+
       '<span class=n style="font-size:11px">'+p.toFixed(1)+'%</span></td></tr>';
  });
  document.getElementById('buckets').innerHTML=h;
  var hs=d.hist||[], mx=Math.max.apply(null,hs.concat([1]));
  document.getElementById('hist').innerHTML=hs.map(function(v,i){
    return '<div class="'+(i>=d.gate_bin?'over':'')+'" style="height:'+(100*v/mx)+'%"></div>';}).join('');
  document.getElementById('histax').textContent =
    '0 m  <- side-length mismatch ->  '+d.hist_max.toFixed(2)+' m     (tolerance '+d.tol+' m)';
  var rr='<tr><th>#</th><th>verdict</th><th>balls found</th><th>nearest miss</th></tr>';
  (d.recent||[]).slice().reverse().forEach(function(x){
    rr+='<tr><td class=n>'+x[0]+'</td><td class="verdict '+x[1]+'">'+x[1]+
       '</td><td class=n>'+x[2]+'</td><td class=n>'+
       (isFinite(x[3])?x[3].toFixed(3)+' m':'-')+'</td></tr>';});
  document.getElementById('recent').innerHTML=rr;
  render();
}
function tick(){fetch('/data').then(function(r){return r.json()}).then(draw).catch(function(){});}
fit();setInterval(tick,900);tick();
</script></body></html>
"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path.startswith('/data'):
            with LOCK:
                errs = [e for e in S['errs'] if np.isfinite(e)]
                hmax = max(0.30, float(np.percentile(errs, 95)) if errs else 0.30)
                hist, _ = np.histogram(np.clip(errs, 0, hmax), bins=30, range=(0, hmax))
                d = {'counts': S['counts'], 'total': S['total'],
                     'roi_pts': S['roi_pts'], 'cands': S['cands'],
                     'recent': S['recent'][-12:], 'last': S['last'],
                     'hist': hist.tolist(), 'hist_max': hmax,
                     'gate_bin': int(30 * min(DET['tol'] / hmax, 1.0)),
                     'tol': DET['tol'], 'hz': S['hz'],
                     'pts': S['pts'], 'lab': S['lab'], 'spheres': S['spheres'],
                     'chosen': S['chosen'], 'verdict': S['verdict'],
                     'roi': S['roi'], 'radius': DET['radius']}
            body = json.dumps(d).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        body = PAGE.encode()
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def main():
    rospy.init_node('sphere_live_dashboard')
    g = rospy.get_param
    port = int(g('~port', 8095))
    topic = g('~cloud_topic', '/zedx_front/zed_node/point_cloud/cloud_registered')
    roi_x = float(g('~roi_x', 9.0))
    roi_sx = float(g('~roi_sx', 1.2))
    sensor_h = float(g('~sensor_h', 0.6972))
    roi_z = float(g('~roi_z', 1.45))

    roi = dict(xmin=roi_x - roi_sx / 2, xmax=roi_x + roi_sx / 2,
               ymin=-0.95, ymax=0.95,
               zmin=roi_z - 0.9 - sensor_h, zmax=roi_z + 0.9 - sensor_h)
    # Nicolas's ransac_sphere calls rng.integers(), which belongs to the newer
    # Generator API - a legacy RandomState has no such method and dies on the
    # first frame. Fixed seed so the dashboard's verdicts are reproducible.
    rng = np.random.default_rng(12345)
    roi_ref = roi  # captured by the callback below
    stamps = []

    def cb(msg):
        t = time.time()
        xyz = decode(msg)
        m = ((xyz[:, 0] > roi['xmin']) & (xyz[:, 0] < roi['xmax']) &
             (xyz[:, 1] > roi['ymin']) & (xyz[:, 1] < roi['ymax']) &
             (xyz[:, 2] > roi['zmin']) & (xyz[:, 2] < roi['zmax']))
        pts = xyz[m]
        bucket, cands, labels, chosen, err = classify_verbose(pts, rng)
        ncand = len(cands)
        # Thin for the browser: neighbouring pixels are not independent anyway,
        # and 15,000 points per frame over the network at 1 Hz is pointless.
        step = max(1, len(pts) // 2500)
        draw_p = pts[::step]
        draw_l = labels[::step]
        with LOCK:
            S['total'] += 1
            S['counts'][bucket] = S['counts'].get(bucket, 0) + 1
            S['roi_pts'] = int(len(pts))
            S['cands'] = int(ncand)
            S['errs'].append(float(err))
            if len(S['errs']) > 4000:
                del S['errs'][:1000]
            S['pts'] = [[round(float(v), 3) for v in q] for q in draw_p]
            S['lab'] = [int(v) for v in draw_l]
            S['spheres'] = [[round(float(v), 3) for v in c] for c in cands]
            S['chosen'] = chosen or []
            S['verdict'] = bucket
            S['roi'] = [roi['xmin'], roi['xmax'], roi['ymin'],
                        roi['ymax'], roi['zmin'], roi['zmax']]
            S['recent'].append([S['total'], bucket, int(ncand), float(err)])
            if len(S['recent']) > 40:
                del S['recent'][:20]
            S['last'] = t
            stamps.append(t)
            if len(stamps) > 20:
                del stamps[:10]
            if len(stamps) > 2:
                S['hz'] = (len(stamps) - 1) / max(stamps[-1] - stamps[0], 1e-6)

    rospy.Subscriber(topic, PointCloud2, cb, queue_size=1, buff_size=2 ** 24)
    srv = ThreadingHTTPServer(('0.0.0.0', port), Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    rospy.loginfo('sphere_live_dashboard: http://0.0.0.0:%d/  ROI x[%.2f,%.2f]',
                  port, roi['xmin'], roi['xmax'])
    rospy.spin()
    srv.shutdown()


if __name__ == '__main__':
    main()
