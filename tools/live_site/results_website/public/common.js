'use strict'
// Shared by every page: the key, Hamilton time, the freshness strip and the poll loop.
// A page defines onBody(body) (draw one snapshot) and may define stateQuery() (extra ?... for /api/state).
const K = new URLSearchParams(location.search).get('k') || ''
const POLL_MS = 2000
const TZ = 'America/Toronto'
const fmtT = new Intl.DateTimeFormat('en-CA', {timeZone: TZ, hour: '2-digit', minute: '2-digit', second: '2-digit', hour12: false})
const hhmmss = ms => fmtT.format(new Date(ms))

let anchor = null        // {age_s, at}: how old the last push was when we last heard, and when that was
let body = null          // the last pushed snapshot
let empty = false        // the store has never received a push
let phoneOkAt = 0        // last time this phone got any answer at all
let phoneErr = ''
let drawErr = ''         // a fault in the page's own drawing code, shown in its own red band        // why the last request failed, if it did
let fatal = null          // a refusal from the website itself (wrong key, store down): shown until it clears

function esc(v){ return String(v).replace(/[&<>"]/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c])) }
function ago(s){
  if (s == null || !isFinite(s)) return '?'
  s = Math.max(0, s)
  if (s < 90) return Math.round(s) + ' s'
  if (s < 5400) return Math.round(s / 60) + ' min'
  return (s / 3600).toFixed(1) + ' h'
}
function setFresh(cls, main, sub){
  const f = document.getElementById('fresh')
  f.className = cls
  f.innerHTML = esc(main) + '<span class="sub">' + esc(sub || '') + '</span>'
}
function band(id, cls, text){
  const b = document.getElementById(id)
  if (!cls){ b.className = 'band'; b.textContent = ''; return }
  b.className = 'band ' + cls; b.textContent = text
}

// ---- 1. the freshness strip, re-judged every second between polls ----------------
function freshness(){
  const now = Date.now()
  if (fatal){ setFresh('bad', fatal[0], fatal[1]); return }
  if (phoneErr && (now - phoneOkAt) > 5000){
    const since = phoneOkAt ? ago((now - phoneOkAt) / 1000) : 'the start'
    setFresh('bad', 'THIS PHONE CANNOT REACH THE WEBSITE', 'No answer for ' + since + ' (' + phoneErr + '). Everything below is frozen.')
    return
  }
  if (empty){
    setFresh('grey', 'NEVER SENT - the Jetson has never sent anything to this page',
      'The sender program (slam_live_push.py) was never started on the Jetson. This is not the same as "stopped".')
    return
  }
  if (!body || !anchor){ return }
  const age = anchor.age_s + (now - anchor.at) / 1000
  const iv = Number(body.interval_s) || 2
  const lastAt = now - age * 1000
  const when = hhmmss(lastAt)
  if (body.stopping){
    setFresh('grey', 'SENDER STOPPED ON PURPOSE at ' + when,
      'Someone stopped slam_live_push.py (' + (body.stop_reason || 'signal') + '). The numbers below are the last ones sent, ' + ago(age) + ' ago.')
    return
  }
  const late = 3 * iv + 3, dead = Math.max(20, 10 * iv)
  if (age <= late && drawErr){
    // updates arrive, but the last one could not be drawn: never green over an old picture.
    // Only here, so the network and data-age states (red / late / stopped) always win.
    setFresh('late', 'UPDATES ARRIVE BUT THIS PAGE CANNOT DRAW THEM', drawErr + ' - what is shown may be old.')
  } else if (age <= late){
    setFresh('ok', 'LIVE - last update ' + ago(age) + ' ago',
      'The Jetson sends every ' + iv + ' s. Last update ' + when + ' Hamilton time.')
  } else if (age <= dead){
    setFresh('late', 'LATE - last update ' + ago(age) + ' ago',
      'The Jetson normally sends every ' + iv + ' s. A WiFi hiccup looks like this; if the number keeps growing, the sender has stopped.')
  } else {
    setFresh('bad', 'STOPPED SENDING - last update ' + ago(age) + ' ago (' + when + ')',
      'Everything below is frozen at that moment. The Jetson, its WiFi, or the sender program has stopped - it was sending every ' + iv + ' s.')
  }
}

// ---- 3. the poll loop ------------------------------------------------------------
async function poll(){
  const url = '/api/state?k=' + encodeURIComponent(K) + (window.stateQuery ? window.stateQuery() : '')
  try {
    const r = await fetch(url, {cache: 'no-store'})
    let j = {}
    try { j = await r.json() } catch (e) {}
    phoneOkAt = Date.now(); phoneErr = ''
    if (r.status === 401){
      fatal = ['THIS LINK\'S KEY IS MISSING OR WRONG', 'Open the full link (ending in ?k=...) once; the key may also have been changed. Ask for the current link.']
      freshness(); return
    }
    if (r.status === 410 && j.moved){          // 27 Sept: live pages moved to the Jetson
      location.replace(j.moved + location.pathname + (K ? '?k=' + encodeURIComponent(K) : ''))
      return
    }
    if (!r.ok){
      fatal = ['THE WEBSITE IS UP BUT ITS DATA STORE IS NOT ANSWERING',
        (j.error || ('HTTP ' + r.status)) + '. This is a website-side fault, not the Jetson.']
      freshness(); return
    }
    fatal = null
    if (j.empty){ empty = true; body = null; freshness(); return }
    empty = false
    anchor = {age_s: j.age_s, at: Date.now()}
    body = j.body
    // a fault while DRAWING is not a network fault: say so on the page instead of letting the
    // strip keep saying LIVE over a frozen picture (status-site review, defect 1)
    try { window.onBody(body); drawErr = '' }
    catch (e){ drawErr = String((e && e.message) || e) }
    const de = document.getElementById('drawerr')
    if (de){ de.className = drawErr ? 'band red' : 'band'; de.textContent = drawErr ? 'THIS PAGE HIT A FAULT DRAWING THE LATEST UPDATE: ' + drawErr + ' - what is shown may be old.' : '' }
    freshness()
  } catch (e){
    phoneErr = 'network error'
    freshness()
  }
}
// ---- the robot's battery voltage (added 2026-09-26) ----------------------------------
// A READING, NOT A CHARGE LEVEL: never coloured, never judged (the user's rule - ask before a drive).
// Returns {volts: '19.84 V' or null, text: plain words}. A value is shown as current ONLY while it is
// under BATT_FRESH_S old, counting every step: robot -> Jetson -> website -> this phone.
const BATT_FRESH_S = 20
const fmtHM = new Intl.DateTimeFormat('en-CA', {timeZone: TZ, hour: '2-digit', minute: '2-digit', hour12: false})
const hhmm = s => (typeof s === 'number' && isFinite(s)) ? fmtHM.format(new Date(s * 1000)) : '?'
function robotBattery(J){
  const R = (J && J.robot) || {}
  const B = R.battery, L = R.battery_last
  const n = v => typeof v === 'number' && isFinite(v)
  const snapAge = anchor ? Math.max(0, anchor.age_s + (Date.now() - anchor.at) / 1000) : 0
  const askAge = (n(R.asked_at) && J && n(J.at)) ? Math.max(0, J.at - R.asked_at) : 0
  const last = (L && n(L.volts) && n(L.at)) ? ' (last reading ' + L.volts.toFixed(2) + ' V at ' + hhmm(L.at) + ')' : ''
  if (R.online === true && B && n(B.volts) && n(B.age_s)){
    const age = B.age_s + askAge + snapAge
    if (age <= BATT_FRESH_S) return {volts: B.volts.toFixed(2) + ' V', text: 'read ' + ago(age) + ' ago'}
    return {volts: null, text: 'no current reading - the newest one is ' + ago(age) + ' old' + last}
  }
  if (R.online !== true){
    return {volts: null, text: n(R.last_seen) ? 'robot offline - last seen ' + hhmm(R.last_seen) + last
                                              : 'robot offline - not seen since the Jetson\'s sender last started' + last}
  }
  const why = (B && typeof B.state === 'string') ? B.state
    : (R.host ? 'the robot\'s reporter is an older version with no battery reading (needs version 4)' : (R.why || 'no report'))
  return {volts: null, text: 'no current reading - ' + why + last}
}
function rowsHtml(pairs){
  return pairs.filter(p => p && p[1] !== undefined && p[1] !== null && p[1] !== '')
    .map(([k, v]) => '<b>' + esc(k) + '</b><span>' + esc(v) + '</span>').join('')
}
// the navigation keeps the key, so moving between pages never asks for it again
function nav(here){
  const q = K ? '?k=' + encodeURIComponent(K) : ''
  const pages = [['index.html', 'Home'], ['status.html', 'Computer status'], ['map.html', 'Live map'], ['results.html', 'Results'], ['replay.html', 'Replay']]
  document.getElementById('nav').innerHTML = pages.map(([f, t]) =>
    '<a href="/' + f + q + '"' + (f === here ? ' class="on"' : '') + '>' + t + '</a>').join('')
}
function startPolling(){
  setInterval(freshness, 1000)
  setInterval(poll, POLL_MS)
  poll()
}
