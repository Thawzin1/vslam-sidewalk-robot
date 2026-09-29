#!/usr/bin/env python3
"""jobs_dashboard.py - see ANY running test, live, from anywhere. Port 8096.

    WHY THIS EXISTS AND WHY IT IS GENERIC. Every earlier dashboard was written
    for one campaign and died with it, so each new test started invisible again.
    The standing rule is that EVERY test - this one and every future one - must
    be watchable live from a browser, so this page discovers jobs instead of
    being told about them.

    THE CONVENTION, and it is the whole interface:

        a job writes  <anything>.progress  next to its output, containing one
        short line of plain text. If that line contains "N/M" the page draws a
        bar; if it contains a number followed by "s" it works out the rate and
        the finish time. Nothing else is required of a job.

        A line that starts with "!!" , or contains STOPPED / ABORT / FAILED,
        is a FAILURE and is shown as one. A line containing DONE / COMPLETE /
        FINISHED is success. Those two vocabularies must never overlap - see
        the note on read_job().

    THREE THINGS THIS PAGE REFUSES TO DO, each because it went wrong:

    1. IT DOES NOT JUDGE LIVENESS BY LOOKING FOR A PROCESS. A check that asks
       whether a command line CONTAINS a name matches itself, and has reported
       dead jobs as alive four times in this project.

    2. IT DOES NOT JUDGE LIVENESS BY FILE FRESHNESS ALONE. Several jobs here
       publish their progress from a separate loop that never checks whether
       the work is alive, so the file keeps ticking over a dead pool. Where a
       line carries a counter, the COUNTER MOVING is the liveness signal, and a
       frozen counter is a stall no matter how fresh the file is.

    3. IT DOES NOT USE A FIXED TIME LIMIT FOR ANYTHING. A flat two-minute rule
       once called ten healthy workers dead because they wrote every five
       minutes. Every threshold here is learned from the job's own observed
       behaviour, and until enough has been observed the page says so rather
       than guessing.

    Reach it over the lab network:  the jobs page (JOBS_PAGE_URL)
"""
import http.server, json, os, re, socketserver, time, html, glob

# The Jetson's system clock is UTC. Everything a human reads here is Hamilton.
os.environ['TZ'] = 'America/Toronto'
time.tzset()

PORT = 8096
ROOTS = [os.path.expanduser('~/closeout_5m'),
         os.path.expanduser('~/jobs'),
         os.path.expanduser('~/zedx_vs_lidar_data'),
         '/tmp']
MAXDEPTH = 3
SLACK = 8               # multiples of a job's OWN worst observed gap
MIN_OBS = 4             # advances that must be seen before any judgement
OLD_S = 6 * 3600        # quiet this long AND finished -> archived, not an alarm

# path -> dict of what we have observed about this job, poll to poll.
_seen = {}

# ---------------------------------------------------------------- discovery
def find_progress():
    out = []
    for root in ROOTS:
        if not os.path.isdir(root):
            continue
        base = root.rstrip('/').count('/')
        for dirpath, dirnames, files in os.walk(root):
            if dirpath.count('/') - base >= MAXDEPTH:
                dirnames[:] = []
            dirnames[:] = [d for d in dirnames if not d.startswith('.')]
            for f in files:
                if f.endswith('.progress'):
                    out.append(os.path.join(dirpath, f))
    return sorted(out)

# The two vocabularies MUST stay disjoint. They did not: the watchdog writes
# "STOPPED" to mean ALARM, and "stopped" had been put in the success list, so a
# real alarm rendered as a finished job - sorted below the healthy ones.
FAIL_RE = re.compile(r'(^\s*!!)|\b(stopped|abort(ed)?|failed|failure|stalled|error)\b', re.I)
DONE_RE = re.compile(r'\b(done|complete[d]?|finished)\b', re.I)
BUSY_RE = re.compile(r'\b(starting|waiting|running|building|learning|measuring)\b', re.I)

def read_job(path):
    try:
        st = os.stat(path)
        with open(path) as fh:
            line = fh.read().strip().splitlines()[-1] if os.path.getsize(path) else ''
    except (OSError, IndexError):
        return None
    now = time.time()
    age = now - st.st_mtime

    cur = tot = None
    m = re.search(r'(\d+)\s*/\s*(\d+)', line)
    if m:
        cur, tot = int(m.group(1)), int(m.group(2))
    el = None
    m = re.search(r'(\d+(?:\.\d+)?)\s*s\b', line)
    if m:
        el = float(m.group(1))
    pct = (100.0 * cur / tot) if (cur is not None and tot) else None

    # ---- learn this job's own rhythm, from the COUNTER where there is one ----
    s = _seen.setdefault(path, dict(cur=None, mtime=0.0, last_move=now,
                                    worst=0.0, moves=0, rate=None, prev_move=None))
    moved = False
    if cur is not None:
        if s['cur'] is None or cur > s['cur']:
            moved = True
    elif st.st_mtime > s['mtime']:
        moved = True                      # no counter: fall back to the file
    if moved:
        gap = now - s['last_move']
        if s['cur'] is not None or s['moves'] > 0:
            if gap > s['worst']:
                s['worst'] = gap
            s['moves'] += 1
            # frames per second, smoothed, from the counter's own advance
            if cur is not None and s['cur'] is not None and gap > 0:
                r = (cur - s['cur']) / gap
                s['rate'] = r if s['rate'] is None else 0.7 * s['rate'] + 0.3 * r
        s['cur'] = cur if cur is not None else s['cur']
        s['last_move'] = now
    s['mtime'] = st.st_mtime
    quiet = now - s['last_move']

    # ETA from the rate we have MEASURED, not from the job's own arithmetic.
    eta = None
    if cur is not None and tot and s['rate'] and s['rate'] > 0 and cur < tot:
        eta = (tot - cur) / s['rate']

    # ---------------------------------- state, in strict order of authority --
    failed = bool(FAIL_RE.search(line))
    done_word = bool(DONE_RE.search(line))
    busy_word = bool(BUSY_RE.search(line))
    # "3/3 stages ... starting the campaign" is NOT finished. A counter reaching
    # its total only means done when the line does not also say it is busy.
    counter_done = (cur is not None and tot and cur >= tot and not busy_word)

    learned = None
    if failed:
        state = 'failed'
    elif done_word or counter_done:
        state = 'done'
    elif s['moves'] < MIN_OBS:
        state = 'starting'          # not enough seen to judge - say so, do not guess
    else:
        learned = max(s['worst'] * SLACK, 1.0)
        if quiet > learned:
            state = 'stalled'
        else:
            state = 'running'

    # Age alone must never turn a failure into a success, but a job quiet for
    # hours is also not "starting". Three different things get three names:
    #   archived  - it finished, and that was long ago
    #   abandoned - it never finished and stopped writing long ago. Not an
    #               active alarm (nobody can act on last week's run) but never
    #               shown as success either: it says plainly that it never
    #               reached its total.
    #   failed    - keeps its name no matter how old it is
    if age > OLD_S and state != 'failed':
        state = 'archived' if state == 'done' else 'abandoned'

    return dict(name=os.path.basename(path)[:-len('.progress')],
                where=os.path.dirname(path), line=line, age=age, quiet=quiet,
                pct=pct, cur=cur, tot=tot, eta=eta, state=state,
                worst=s['worst'], moves=s['moves'], learned=learned,
                rate=s['rate'])

# ------------------------------------------------------------ jetson health
_static = {}

def _read(p, default=''):
    try:
        with open(p) as fh:
            return fh.read()
    except OSError:
        return default

def _static_once():
    if _static:
        return _static
    zones = []
    for z in sorted(glob.glob('/sys/devices/virtual/thermal/thermal_zone*')):
        zones.append((z, _read(os.path.join(z, 'type'), '?').strip()))
    _static['zones'] = zones
    _static['ncpu'] = len([l for l in _read('/proc/stat').splitlines()
                           if re.match(r'cpu\d+ ', l)])
    mt = re.search(r'MemTotal:\s+(\d+)', _read('/proc/meminfo'))
    _static['memtotal_kb'] = int(mt.group(1)) if mt else 0
    # nvpmodel: which power mode the board is in
    mode = _read('/var/lib/nvpmodel/status', '').strip()
    m = re.search(r'pmode:?\s*(\d+)', mode)
    idx = m.group(1) if m else None
    name = None
    if idx is not None:
        for ln in _read('/etc/nvpmodel.conf').splitlines():
            mm = re.match(r'\s*<\s*POWER_MODEL\s+ID=(\d+)\s+NAME=(\S+)', ln)
            if mm and mm.group(1).lstrip('0') == idx.lstrip('0'):
                name = mm.group(2).rstrip('>')
    _static['power_mode'] = name or (('mode %s' % idx) if idx else 'unknown')
    # earlyoom's own thresholds, read from its command line
    _static['earlyoom'] = None
    for d in glob.glob('/proc/[0-9]*'):
        cl = _read(os.path.join(d, 'cmdline'), '').split('\0')
        if cl and os.path.basename(cl[0]) == 'earlyoom':
            mem = swp = 15.0
            for i, a in enumerate(cl):
                if a == '-m' and i + 1 < len(cl):
                    mem = float(re.split('[,%]', cl[i + 1])[0])
                if a == '-s' and i + 1 < len(cl):
                    swp = float(re.split('[,%]', cl[i + 1])[0])
            _static['earlyoom'] = dict(mem=mem, swap=swp)
            break
    gmin = _read('/sys/class/devfreq/17000000.ga10b/min_freq', '0').strip()
    _static['gpu_min'] = gmin
    return _static

_cpu_prev = {}
_gpu_prev = {}

def health():
    S = _static_once()
    h = {}

    # ---- memory. MemAvailable is the number earlyoom actually watches.
    mi = {}
    for ln in _read('/proc/meminfo').splitlines():
        k, _, v = ln.partition(':')
        mi[k] = int(v.split()[0]) if v.split() else 0
    tot = mi.get('MemTotal', 1)
    avail = mi.get('MemAvailable', 0)
    swtot = mi.get('SwapTotal', 0)
    swfree = mi.get('SwapFree', 0)
    h['mem'] = dict(total_mb=tot // 1024, avail_mb=avail // 1024,
                    used_mb=(tot - avail) // 1024,
                    avail_pct=100.0 * avail / tot if tot else 0,
                    swap_total_mb=swtot // 1024, swap_free_mb=swfree // 1024,
                    swap_free_pct=(100.0 * swfree / swtot) if swtot else 100.0)
    h['earlyoom'] = S['earlyoom']

    # ---- disk. statvfs's naive percentage disagrees with df by ~5 points
    # because ext4 reserves blocks for root; df excludes them from the total.
    h['disk'] = []
    for mp in ('/',):
        try:
            v = os.statvfs(mp)
        except OSError:
            continue
        total = v.f_blocks * v.f_frsize
        free_u = v.f_bavail * v.f_frsize          # what a normal user may use
        used = (v.f_blocks - v.f_bfree) * v.f_frsize
        denom = used + free_u                     # df's denominator
        h['disk'].append(dict(mount=mp, free_gb=free_u / 2**30,
                              total_gb=total / 2**30,
                              used_pct=(100.0 * used / denom) if denom else 0))

    # ---- per-core CPU, from counter deltas. No sleep, no aggregate line.
    cores = []
    for ln in _read('/proc/stat').splitlines():
        m = re.match(r'cpu(\d+)\s+(.*)', ln)
        if not m:
            continue
        i = int(m.group(1))
        f = [int(x) for x in m.group(2).split()]
        idle = f[3] + (f[4] if len(f) > 4 else 0)
        total = sum(f)
        p = _cpu_prev.get(i)
        _cpu_prev[i] = (total, idle)
        if p and total > p[0]:
            cores.append(100.0 * (1.0 - (idle - p[1]) / (total - p[0])))
        else:
            cores.append(None)
    h['cores'] = cores
    la = _read('/proc/loadavg').split()
    h['load'] = dict(one=float(la[0]) if la else 0.0,
                     ncpu=S['ncpu'],
                     runnable=la[3] if len(la) > 3 else '?')

    # ---- GPU. GR3D_FREQ / gpu.0/load read 0 most of the time because the rail
    # duty-cycles and a single instantaneous sample usually catches it OFF.
    # Time-in-state residency is the honest measure and was validated against an
    # independent duty-cycle sampler to within 0.5 percentage points.
    res = {}
    for ln in _read('/sys/class/devfreq/17000000.ga10b/trans_stat').splitlines()[2:]:
        mm = re.match(r'\s*\*?\s*(\d+):.*?(\d+)\s*$', ln)
        if mm:
            res[mm.group(1)] = int(mm.group(2))
    now = time.time()
    busy = powered = None
    if res and _gpu_prev.get('res') and _gpu_prev.get('t'):
        wall_ms = (now - _gpu_prev['t']) * 1000.0
        accrued = sum(max(0, res.get(k, 0) - _gpu_prev['res'].get(k, 0)) for k in res)
        above = sum(max(0, res.get(k, 0) - _gpu_prev['res'].get(k, 0))
                    for k in res if k != S['gpu_min'])
        # THE DENOMINATOR IS WALL TIME, NOT THE ACCRUED TOTAL.
        # This counter FREEZES while the GPU power rail is gated - measured, it
        # accrued 1,064 ms over a 5,001 ms window. Dividing by the accrued total
        # answers "while it was powered, was it above idle clock", which was
        # reading 100% while the true share of wall time was 21%. Dividing by
        # wall time answers the question a reader actually has.
        if wall_ms > 0:
            busy = min(100.0, 100.0 * above / wall_ms)
            powered = min(100.0, 100.0 * accrued / wall_ms)
    _gpu_prev['res'] = res
    _gpu_prev['t'] = now
    h['gpu'] = dict(busy_pct=busy, powered_pct=powered)

    # ---- thermals. Four of eleven zones are dead and report -256 C.
    temps = []
    for path, name in S['zones']:
        t = _read(os.path.join(path, 'temp'), '').strip()
        try:
            c = int(t) / 1000.0
        except ValueError:
            continue
        if c < -100:
            continue                     # dead sensor, not a cold one
        temps.append(dict(name=name, c=c))
    h['temps'] = sorted(temps, key=lambda d: -d['c'])[:6]

    # ---- fan. The thermal framework's cur_state is a REQUEST, not a speed.
    # rpm and pwm1 are NOT on the same hwmon node on this board - the tachometer
    # is on hwmon0 and the duty on hwmon2. Glob for each rather than assume.
    rpm = pwm = ''
    for f in sorted(glob.glob('/sys/class/hwmon/hwmon*/rpm')):
        rpm = _read(f, '').strip()
        if rpm:
            break
    for f in sorted(glob.glob('/sys/class/hwmon/hwmon*/pwm1')):
        pwm = _read(f, '').strip()
        if pwm:
            break
    h['fan'] = dict(rpm=int(rpm) if rpm.isdigit() else None,
                    duty=round(100 * int(pwm) / 255) if pwm.isdigit() else None)

    # ---- throttling
    thr = False
    for cd in glob.glob('/sys/class/thermal/cooling_device*/cur_state'):
        v = _read(cd, '0').strip()
        if v.isdigit() and int(v) > 0:
            thr = True
    h['throttling'] = thr
    h['power_mode'] = S['power_mode']

    up = _read('/proc/uptime', '0').split()
    h['uptime_s'] = float(up[0]) if up else 0
    return h

# ------------------------------------------------------------------ results
SUM_RE = re.compile(r'^(\S+)\s+n=(\d+)\s+sigma3D=\s*([\d.]+)\s*mm')

# The 5 m station's finished numbers, so 4 m can be read beside them.
# Source: 08_knowledge_base/05_WHAT_5M_SETTLED.md, section 2.
REF_5M = {
    'NEURAL':      dict(n=1444, acc=42.0, s3=18.34),
    'NEURAL_PLUS': dict(n=2990, acc=87.0, s3=18.87),
    'ULTRA':       dict(n=2647, acc=77.0, s3=23.53),
    'QUALITY':     dict(n=999,  acc=29.1, s3=25.71),
    'PERFORMANCE': dict(n=236,  acc=6.9,  s3=45.42),
}

def results():
    """Finished passes only. A SUMMARY line is written even when a pass FAILED,
    so the .complete marker beside the CSV is the only proof it is trustworthy."""
    out = []
    d = os.path.expanduser('~/zedx_vs_lidar_data/offline_4m')
    sumf = os.path.join(d, 'SUMMARY.txt')
    if not os.path.isfile(sumf):
        return out
    for ln in _read(sumf).splitlines():
        m = SUM_RE.match(ln.strip())
        if not m:
            continue
        label, n, s3 = m.group(1), int(m.group(2)), float(m.group(3))
        mode, _, tag = label.rpartition('_')
        csv = os.path.join(d, 'offline_w3_4.0m_%s.csv' % label)
        out.append(dict(label=label, mode=mode, tag=tag, n=n, s3=s3,
                        trusted=os.path.isfile(csv + '.complete'),
                        ref=REF_5M.get(mode)))
    return out

# -------------------------------------------------------------- the snapshot
def snapshot():
    jobs = [j for j in (read_job(p) for p in find_progress()) if j]
    order = {'failed': 0, 'stalled': 1, 'running': 2, 'starting': 3,
             'done': 4, 'abandoned': 5, 'archived': 6}
    jobs.sort(key=lambda j: (order.get(j['state'], 9), j['name']))
    return dict(jobs=jobs,
                now=time.strftime('%H:%M:%S %Z'),
                counts={k: sum(1 for j in jobs if j['state'] == k) for k in order},
                health=health(), results=results())

def dur(s):
    if s is None:
        return '—'
    s = int(s)
    if s < 90:
        return '%ds' % s
    if s < 5400:
        return '%dm' % (s // 60)
    return '%dh %02dm' % (s // 3600, (s % 3600) // 60)

PAGE = r"""<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Tests running now</title>
<style>
:root{--ink:#171A19;--body:#2A2E2C;--muted:#6E6A5C;--teal:#0F6B5D;--rust:#A8461E;
 --amber:#9E6612;--sand:#D9CFB8;--ground:#FBFAF6;--card:#fff;--rule:#DED8C8}
@media(prefers-color-scheme:dark){:root{--ink:#F2EFE6;--body:#D8D4C8;--muted:#9A9483;
 --teal:#4FB8A3;--rust:#E08B5E;--amber:#D9A247;--sand:#4A4433;--ground:#12140F;
 --card:#1A1D17;--rule:#332F26}}
*{box-sizing:border-box}
body{margin:0;background:var(--ground);color:var(--body);
 font:15px/1.5 -apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,sans-serif}
.wrap{max-width:820px;margin:0 auto;padding:20px 16px 60px}
h1{font-size:20px;color:var(--ink);margin:0 0 3px}
h2{font-size:14px;color:var(--ink);margin:26px 0 9px;text-transform:uppercase;
 letter-spacing:.09em}
.sub{color:var(--muted);font-size:13px;margin:0 0 16px}
.tally{display:flex;gap:7px;flex-wrap:wrap;margin-bottom:16px}
.pill{padding:5px 12px;border-radius:20px;font-size:13px;font-weight:600}
.pill.f{background:var(--rust);color:#fff}
.pill.s{background:var(--amber);color:#fff}
.pill.r{background:var(--teal);color:#fff}
.pill.d{background:var(--sand);color:var(--ink)}
.job{background:var(--card);border:1px solid var(--rule);border-radius:8px;
 padding:13px 15px;margin-bottom:9px}
.job.failed{border-color:var(--rust);border-width:2px}
.job.stalled{border-color:var(--amber);border-width:2px}
.job.archived,.job.done,.job.abandoned{opacity:.6}
.job.abandoned{border-style:dashed}
.top{display:flex;justify-content:space-between;align-items:baseline;gap:10px}
.nm{font-weight:650;color:var(--ink);font-size:15px;word-break:break-all}
.st{font-size:11.5px;text-transform:uppercase;letter-spacing:.08em;font-weight:700;
 white-space:nowrap}
.st.running{color:var(--teal)}.st.stalled{color:var(--amber)}
.st.failed{color:var(--rust)}
.st.done,.st.archived,.st.starting,.st.abandoned{color:var(--muted)}
.bar{height:8px;background:var(--sand);border-radius:5px;overflow:hidden;margin:9px 0 6px}
.bar i{display:block;height:100%;background:var(--teal);transition:width .4s}
.job.done .bar i,.job.archived .bar i{background:var(--muted)}
.job.failed .bar i{background:var(--rust)}
.line{font-family:ui-monospace,Menlo,Consolas,monospace;font-size:12.5px;
 color:var(--muted);word-break:break-all}
.meta{font-size:12.5px;color:var(--muted);margin-top:4px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(160px,1fr));gap:8px}
.tile{background:var(--card);border:1px solid var(--rule);border-radius:8px;padding:11px 13px}
.tile.warn{border-color:var(--amber)}
.tile.bad{border-color:var(--rust);border-width:2px}
.tk{font-size:11.5px;color:var(--muted);text-transform:uppercase;letter-spacing:.07em}
.tv{font-size:19px;color:var(--ink);font-weight:650;font-variant-numeric:tabular-nums;
 margin-top:2px}
.tn{font-size:12px;color:var(--muted);margin-top:2px}
.cores{display:flex;gap:2px;align-items:flex-end;height:30px;margin-top:6px}
.cores i{flex:1;background:var(--teal);border-radius:1px;min-height:1px;display:block}
table{width:100%;border-collapse:collapse;font-size:13px}
th,td{text-align:right;padding:5px 7px;border-bottom:1px solid var(--rule);
 font-variant-numeric:tabular-nums}
th:first-child,td:first-child{text-align:left}
th{color:var(--muted);font-weight:600;font-size:11.5px;text-transform:uppercase;
 letter-spacing:.06em}
.warnrow{color:var(--rust);font-weight:600}
.none{color:var(--muted);text-align:center;padding:30px 0}
.foot{color:var(--muted);font-size:12px;margin-top:22px;line-height:1.6}
code{font-family:ui-monospace,Menlo,Consolas,monospace}
</style></head><body><div class="wrap">
<h1>Tests running now</h1>
<p class="sub">Every job that writes a <code>.progress</code> file appears here by itself.
Updates every 5 seconds &middot; <span id="t">—</span> &middot; Hamilton time</p>
<div class="tally" id="tally"></div>
<div id="jobs"></div>

<h2>Results so far</h2>
<div id="results"></div>

<h2>Jetson</h2>
<div class="grid" id="health"></div>
<div id="cores"></div>

<p class="foot" id="foot"></p>
</div>
<script>
function dur(s){if(s==null)return'—';s=Math.round(s);
 if(s<90)return s+'s';if(s<5400)return Math.round(s/60)+'m';
 return Math.floor(s/3600)+'h '+String(Math.floor((s%3600)/60)).padStart(2,'0')+'m';}
function esc(x){return String(x).replace(/[&<>]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;'}[c]));}

async function tick(){
 let d;try{d=await(await fetch('/data?'+Date.now())).json()}catch(e){return}
 document.getElementById('t').textContent=d.now;

 const c=d.counts;
 document.getElementById('tally').innerHTML=
   (c.failed?'<span class="pill f">'+c.failed+' failed</span>':'')+
   (c.stalled?'<span class="pill s">'+c.stalled+' stalled</span>':'')+
   (c.running?'<span class="pill r">'+c.running+' running</span>':'')+
   (c.starting?'<span class="pill d">'+c.starting+' starting</span>':'')+
   (c.done?'<span class="pill d">'+c.done+' done</span>':'')+
   (c.abandoned?'<span class="pill d">'+c.abandoned+' abandoned</span>':'')+
   (c.archived?'<span class="pill d">'+c.archived+' archived</span>':'');

 document.getElementById('jobs').innerHTML = d.jobs.length? d.jobs.map(j=>{
   let m=[];
   if(j.pct!=null) m.push(j.pct.toFixed(1)+'%');
   if(j.eta!=null) m.push('about '+dur(j.eta)+' left');
   if(j.rate) m.push((j.rate>=1? j.rate.toFixed(2)+'/s' : (1/j.rate).toFixed(0)+'s each'));
   if(j.state==='abandoned') m.push('stopped writing '+dur(j.age)+' ago and never reached its total');
   else if(j.state==='starting') m.push('learning its pace ('+j.moves+' advances seen)');
   else if(j.learned) m.push('quiet '+dur(j.quiet)+', its own worst gap '+dur(j.worst)+
                             ', flagged at '+dur(j.learned));
   m.push('file written '+dur(j.age)+' ago');
   return '<div class="job '+j.state+'"><div class="top"><span class="nm">'+esc(j.name)+
     '</span><span class="st '+j.state+'">'+j.state+'</span></div>'+
     (j.pct!=null?'<div class="bar"><i style="width:'+Math.min(100,j.pct)+'%"></i></div>':'')+
     '<div class="line">'+esc(j.line)+'</div><div class="meta">'+m.join(' &middot; ')+'</div></div>';
 }).join('') : '<p class="none">No test is writing a progress file right now.</p>';

 // ---- results
 const R=d.results;
 document.getElementById('results').innerHTML = R.length? (
   '<table><tr><th>pass</th><th>n</th><th>&sigma;3D mm</th><th>5 m &sigma;3D</th><th>trusted</th></tr>'+
   R.map(r=>'<tr><td>'+esc(r.label)+'</td><td>'+r.n+'</td><td>'+r.s3.toFixed(2)+'</td><td>'+
     (r.ref? r.ref.s3.toFixed(2) : '—')+'</td><td>'+
     (r.trusted?'yes':'<span class="warnrow">NO marker</span>')+'</td></tr>').join('')+
   '</table><p class="foot">A SUMMARY line is written even when a pass FAILED. The '+
   '<code>.complete</code> marker beside the CSV is the only proof a pass replayed every '+
   'frame it was given — a pass without one must not enter a mode comparison.</p>'
 ) : '<p class="none">No pass has finished yet. There is no partial result to show: the '+
     'replay holds every accepted centroid in memory and writes its CSV only when the pass ends.</p>';

 // ---- jetson
 const h=d.health, t=[];
 const mem=h.mem, eo=h.earlyoom;
 const memCls = eo && mem.avail_pct<=eo.mem && mem.swap_free_pct<=eo.swap ? 'bad'
              : (eo && mem.avail_pct<=eo.mem ? 'warn':'');
 t.push('<div class="tile '+memCls+'"><div class="tk">memory available</div><div class="tv">'+
   mem.avail_pct.toFixed(1)+'%</div><div class="tn">'+(mem.avail_mb/1024).toFixed(1)+
   ' of '+(mem.total_mb/1024).toFixed(1)+' GiB'+
   (eo?'<br>earlyoom kills only when memory &le;'+eo.mem+'% <b>and</b> swap &le;'+eo.swap+'%':'')+
   '</div></div>');
 t.push('<div class="tile"><div class="tk">swap free (zram)</div><div class="tv">'+
   mem.swap_free_pct.toFixed(1)+'%</div><div class="tn">'+
   ((mem.swap_total_mb-mem.swap_free_mb)/1024).toFixed(2)+' GiB used — zram is compressed RAM, not disk</div></div>');
 h.disk.forEach(k=>{
   const cls = k.used_pct>=95?'bad':(k.used_pct>=90?'warn':'');
   t.push('<div class="tile '+cls+'"><div class="tk">disk '+k.mount+'</div><div class="tv">'+
     k.free_gb.toFixed(1)+' GiB free</div><div class="tn">'+k.used_pct.toFixed(0)+'% used of '+
     k.total_gb.toFixed(0)+' GiB</div></div>');
 });
 t.push('<div class="tile"><div class="tk">load</div><div class="tv">'+h.load.one.toFixed(1)+
   '</div><div class="tn">'+(h.load.one/h.load.ncpu).toFixed(2)+' per core across '+
   h.load.ncpu+' · runnable '+h.load.runnable+'</div></div>');
 t.push('<div class="tile"><div class="tk">gpu busy</div><div class="tv">'+
   (h.gpu.busy_pct==null?'measuring':h.gpu.busy_pct.toFixed(0)+'%')+
   '</div><div class="tn">share of wall time above idle clock'+
   (h.gpu.powered_pct!=null?'; rail powered '+h.gpu.powered_pct.toFixed(0)+'% of the time':'')+
   '. A single read of GR3D_FREQ shows 0% here because the rail duty-cycles and one '+
   'instantaneous sample usually catches it off.</div></div>');
 const hot=h.temps[0];
 t.push('<div class="tile'+(hot&&hot.c>=90?' warn':'')+'"><div class="tk">temperature</div><div class="tv">'+
   (hot?hot.c.toFixed(0)+'°C':'—')+'</div><div class="tn">'+
   h.temps.map(x=>esc(x.name)+' '+x.c.toFixed(0)).join(' · ')+
   '<br>no GPU sensor on this board</div></div>');
 t.push('<div class="tile'+(h.throttling?' warn':'')+'"><div class="tk">throttling</div><div class="tv">'+
   (h.throttling?'YES':'no')+'</div><div class="tn">fan '+
   (h.fan.rpm==null?'—':h.fan.rpm+' rpm'+(h.fan.duty!=null?' at '+h.fan.duty+'%':''))+
   ' · '+esc(h.power_mode)+'</div></div>');
 t.push('<div class="tile"><div class="tk">uptime</div><div class="tv">'+dur(h.uptime_s)+
   '</div><div class="tn">a reboot ends a detached run silently</div></div>');
 document.getElementById('health').innerHTML=t.join('');

 document.getElementById('cores').innerHTML='<div class="tile" style="margin-top:8px">'+
   '<div class="tk">'+h.cores.length+' cores</div><div class="cores">'+
   h.cores.map(v=>'<i style="height:'+(v==null?1:Math.max(1,v))+'%"></i>').join('')+
   '</div><div class="tn">'+h.cores.map(v=>v==null?'—':v.toFixed(0)).join(' ')+'</div></div>';

 document.getElementById('foot').innerHTML=
   'A job is <b>running</b> while its counter is still advancing, <b>stalled</b> only after '+
   'several of <em>its own</em> observed gaps have passed with the counter frozen, and '+
   '<b>starting</b> until enough advances have been seen to know its rhythm — never after '+
   'a fixed clock, because a flat two-minute rule once called ten healthy workers dead. '+
   '<b>Liveness is the counter moving, not the file being fresh</b>: several jobs here publish '+
   'progress from a separate loop that never checks whether the work is alive.';
}
tick();setInterval(tick,5000);
</script></body></html>"""


class H(http.server.BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path.startswith('/data'):
            body = json.dumps(snapshot()).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Cache-Control', 'no-store')
        else:
            body = PAGE.encode()
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)


class S(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


if __name__ == '__main__':
    print('jobs dashboard on http://0.0.0.0:%d/  (%s)'
          % (PORT, time.strftime('%Y-%m-%d %H:%M:%S %Z')))
    S(('0.0.0.0', PORT), H).serve_forever()
