#!/usr/bin/env python3
"""phone_teleop.py — drive the Husky from a phone browser over WiFi.

Runs ON THE ROBOT. Serves a touch-friendly page; the phone posts velocity
commands which are republished as geometry_msgs/Twist.

    ssh Robot_MARC269
    rosrun sidewalk_bringup phone_teleop.py      (or: python3 phone_teleop.py)
    # then on the phone, same WiFi:  http://<old-robot-address>:8090/

SAFETY - read before using, this moves a 50 kg robot
    * Publishes to kb_teleop/cmd_vel, which sits at priority 9 in the robot's
      twist_mux. The joystick (priority 10) still overrides it if it comes back,
      and the mux's own 0.5 s timeout stops the robot if commands stop arriving.
    * The page sends a command every 100 ms only WHILE a control is held, and
      sends an explicit zero on release, on touch-cancel, and when the page is
      hidden or loses focus (pocketed phone, incoming call, screen lock).
    * This node also enforces its own deadman: if no command arrives for
      DEADMAN_S it publishes zero continuously. Losing WiFi therefore stops the
      robot rather than leaving the last command latched.
    * Speeds are capped low on purpose (see MAX_LIN / MAX_ANG). Raise them only
      deliberately.
"""
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import rospy
from geometry_msgs.msg import Twist

PORT = 8090
TOPIC = "kb_teleop/cmd_vel"
MAX_LIN = 0.40          # m/s   - indoor mapping speed, deliberately gentle
MAX_ANG = 0.60          # rad/s - slow turns map far better than fast pivots
DEADMAN_S = 0.4         # stop if the phone goes quiet for this long
PUBLISH_HZ = 20.0

_lock = threading.Lock()
_cmd = {"lin": 0.0, "ang": 0.0, "t": 0.0}

PAGE = """<!doctype html>
<html><head>
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>Husky</title>
<style>
 *{box-sizing:border-box;-webkit-user-select:none;user-select:none;-webkit-touch-callout:none}
 body{margin:0;background:#14161a;color:#e8eaed;font:16px/1.4 system-ui,-apple-system,sans-serif;
      height:100vh;display:flex;flex-direction:column;overscroll-behavior:none;touch-action:none}
 header{padding:10px 14px;background:#1d2027;display:flex;align-items:center;gap:10px}
 #dot{width:10px;height:10px;border-radius:50%;background:#e0483d}
 #dot.ok{background:#2fbf5f}
 .grid{flex:1;display:grid;grid-template-columns:repeat(3,1fr);grid-template-rows:repeat(3,1fr);
       gap:10px;padding:12px}
 button{border:0;border-radius:14px;background:#2a2f3a;color:#e8eaed;font-size:26px;font-weight:600;
        touch-action:none}
 button:active,button.on{background:#3d6fd4}
 #stop{background:#8c2b24;font-size:20px}
 #stop:active{background:#c0392b}
 .sp{padding:10px 14px;background:#1d2027;display:flex;align-items:center;gap:12px}
 input[type=range]{flex:1}
</style></head><body>
<header><div id="dot"></div><div id="st">connecting…</div></header>
<div class="grid">
  <button data-l="1"  data-a="1">&#8598;</button>
  <button data-l="1"  data-a="0">&#8593;</button>
  <button data-l="1"  data-a="-1">&#8599;</button>
  <button data-l="0"  data-a="1">&#8634;</button>
  <button id="stop" data-l="0" data-a="0">STOP</button>
  <button data-l="0"  data-a="-1">&#8635;</button>
  <button data-l="-1" data-a="1">&#8601;</button>
  <button data-l="-1" data-a="0">&#8595;</button>
  <button data-l="-1" data-a="-1">&#8600;</button>
</div>
<div class="sp"><span>speed</span><input id="sp" type="range" min="20" max="100" value="50">
  <span id="spv">50%</span></div>
<script>
let cur={l:0,a:0}, timer=null, scale=0.5;
const dot=document.getElementById('dot'), st=document.getElementById('st');
const sp=document.getElementById('sp'), spv=document.getElementById('spv');
sp.addEventListener('input',()=>{scale=sp.value/100;spv.textContent=sp.value+'%';});

async function send(l,a){
  try{
    const r=await fetch('/cmd',{method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify({lin:l*scale,ang:a*scale})});
    const j=await r.json();
    dot.classList.add('ok'); st.textContent=j.msg;
  }catch(e){ dot.classList.remove('ok'); st.textContent='connection lost'; }
}
function start(l,a,el){
  cur={l:l,a:a}; if(el)el.classList.add('on');
  send(l,a);
  clearInterval(timer); timer=setInterval(()=>send(cur.l,cur.a),100);
}
function stop(){
  clearInterval(timer); timer=null; cur={l:0,a:0};
  document.querySelectorAll('button').forEach(b=>b.classList.remove('on'));
  send(0,0);
}
document.querySelectorAll('button').forEach(b=>{
  const l=+b.dataset.l, a=+b.dataset.a;
  const go=e=>{e.preventDefault(); if(b.id==='stop'){stop();return;} start(l,a,b);};
  b.addEventListener('touchstart',go,{passive:false});
  b.addEventListener('mousedown',go);
  ['touchend','touchcancel','mouseup','mouseleave'].forEach(ev=>
    b.addEventListener(ev,e=>{e.preventDefault(); stop();},{passive:false}));
});
// stop if the phone is pocketed, locked, or the tab loses focus
document.addEventListener('visibilitychange',()=>{if(document.hidden)stop();});
window.addEventListener('blur',stop);
window.addEventListener('pagehide',stop);
send(0,0);
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass                                   # keep the console readable

    def do_GET(self):
        body = PAGE.encode()
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        try:
            n = int(self.headers.get("Content-Length", 0))
            d = json.loads(self.rfile.read(n) or b"{}")
            lin = max(-1.0, min(1.0, float(d.get("lin", 0.0)))) * MAX_LIN
            ang = max(-1.0, min(1.0, float(d.get("ang", 0.0)))) * MAX_ANG
        except Exception:
            lin = ang = 0.0
        with _lock:
            _cmd["lin"], _cmd["ang"], _cmd["t"] = lin, ang, time.time()
        msg = "stopped" if (lin == 0 and ang == 0) else \
              "%.2f m/s  %.2f rad/s" % (lin, ang)
        body = json.dumps({"msg": msg}).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def serve():
    HTTPServer(("0.0.0.0", PORT), Handler).serve_forever()


if __name__ == "__main__":
    rospy.init_node("phone_teleop")
    pub = rospy.Publisher(TOPIC, Twist, queue_size=1)
    threading.Thread(target=serve, daemon=True).start()
    rospy.loginfo("phone_teleop: open http://<robot-ip>:%d/ on a phone on this "
                  "WiFi. Publishing to %s, max %.2f m/s / %.2f rad/s.",
                  PORT, TOPIC, MAX_LIN, MAX_ANG)

    rate = rospy.Rate(PUBLISH_HZ)
    while not rospy.is_shutdown():
        with _lock:
            fresh = (time.time() - _cmd["t"]) < DEADMAN_S
            lin, ang = (_cmd["lin"], _cmd["ang"]) if fresh else (0.0, 0.0)
        t = Twist()
        t.linear.x = lin
        t.angular.z = ang
        pub.publish(t)                          # keep publishing, incl. zeros
        rate.sleep()
