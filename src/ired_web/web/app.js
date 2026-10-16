/* iRED Control web app (phase 1): Status, Launch, Drive, Camera, E-STOP. */
'use strict';

const $ = (sel) => document.querySelector(sel);
const host = location.hostname;

const S = {
  pin: (() => { try { return localStorage.getItem('ired_pin') || ''; } catch (e) { return ''; } })(),
  cfg: null,
  status: null,
  ros: null,
  rosOk: false,
  tab: 'status',
  estop: false,
  cmdPub: null,
  markerSub: null,
  openLogs: new Set(),
};

/* ---------- manager API (needs PIN) ---------- */
async function api(path, opts = {}) {
  const res = await fetch('/api/' + path, {
    method: opts.method || 'GET',
    headers: { 'X-Pin': S.pin, 'Content-Type': 'application/json' },
    body: opts.body ? JSON.stringify(opts.body) : undefined,
  });
  if (res.status === 401) {
    const j = await res.json().catch(() => ({}));
    askPin(j.error === 'locked' ? 'Too many wrong PINs, wait 30 s.' : (S.pin ? 'Wrong PIN.' : ''));
    throw new Error('pin');
  }
  if (!res.ok) throw new Error('http ' + res.status);
  return res.json();
}

function askPin(msg) {
  const d = $('#pin-dialog');
  $('#pin-error').textContent = msg || '';
  if (!d.open) d.showModal();
  $('#pin-input').focus();
}

$('#pin-form').addEventListener('submit', async (e) => {
  e.preventDefault();
  S.pin = $('#pin-input').value.trim();
  try { localStorage.setItem('ired_pin', S.pin); } catch (err) { /* private mode */ }
  try {
    await api('status');
    $('#pin-dialog').close();
    $('#pin-input').value = '';
    start();
  } catch (err) { /* askPin already shown */ }
});

/* ---------- ROS bridge ---------- */
function connectRos() {
  if (!S.cfg) return;
  const ros = new ROSLIB.Ros({ url: `ws://${host}:${S.cfg.rosbridge_port}` });
  S.ros = ros;
  ros.on('connection', () => {
    S.rosOk = true;
    S.cmdPub = new ROSLIB.Topic({ ros, name: '/cmd_vel', messageType: 'geometry_msgs/msg/TwistStamped' });
    if (S.tab === 'camera') subscribeMarkers(true);
    renderChips();
  });
  const lost = () => {
    if (S.ros !== ros) return;
    S.rosOk = false;
    S.cmdPub = null;
    S.markerSub = null;
    renderChips();
    setTimeout(connectRos, 3000);
  };
  ros.on('close', lost);
  ros.on('error', () => {});
}

function publishCmd(vx, vy, wz) {
  if (!S.cmdPub) return;
  const now = Date.now();
  S.cmdPub.publish({
    header: { stamp: { sec: Math.floor(now / 1000), nanosec: (now % 1000) * 1e6 }, frame_id: 'base_link' },
    twist: { linear: { x: vx, y: vy, z: 0 }, angular: { x: 0, y: 0, z: wz } },
  });
}

/* ---------- status polling ---------- */
let appOk = false;
async function poll() {
  try {
    S.status = await api('status');
    appOk = true;
    S.estop = S.status.estop;
    render();
  } catch (e) {
    appOk = e.message === 'pin';
  }
  renderChips();
  setTimeout(poll, 1000);
}

function renderChips() {
  $('#chip-app').className = 'chip ' + (appOk ? 'ok' : 'bad');
  $('#chip-ros').className = 'chip ' + (S.rosOk ? 'ok' : 'bad');
}

function fmtUptime(s) {
  const h = Math.floor(s / 3600), m = Math.floor((s % 3600) / 60);
  return h ? `${h} h ${m} min` : `${m} min ${s % 60} s`;
}

function card(k, v, s, level) {
  return `<div class="card ${level || ''}"><div class="k">${k}</div><div class="v">${v}</div><div class="s">${s || ''}</div></div>`;
}

const EXPECTED = {   // rough healthy rates, Hz
  '/scan_filtered': [5, 'LiDAR'], '/odometry/filtered': [15, 'EKF odometry'], '/ired/imu/data': [8, 'IMU'],
  '/ired/motor/data': [8, 'Wheels'], '/aruco/detections': [5, 'ArUco'], '/cmd_vel': [0, 'Velocity command'],
};

function render() {
  const st = S.status;
  if (!st) return;
  const sy = st.system;

  // E-STOP
  $('#estop').textContent = S.estop ? 'RELEASE' : 'STOP';
  $('#estop').classList.toggle('active', S.estop);
  $('#estop-banner').hidden = !S.estop;

  // status cards
  const fanBad = sy.fan_pwm_percent > 0 && sy.fan_rpm === 0;
  const thr = sy.throttled && sy.throttled !== '0x0';
  $('#sys-cards').innerHTML = [
    card('CPU', `${Math.round(sy.cpu_percent)}%`, `load ${sy.load.join(' ')}`, sy.cpu_percent > 85 ? 'warn' : ''),
    card('Temperature', `${sy.temp_c}°C`, sy.temp_c >= 80 ? 'throttling soon' : '', sy.temp_c >= 80 ? 'bad' : sy.temp_c >= 70 ? 'warn' : ''),
    card('Fan', sy.fan_rpm === null ? '–' : `${sy.fan_rpm} rpm`, sy.fan_rpm === null ? 'no fan header' : fanBad ? `set ${sy.fan_pwm_percent}% but not spinning` : `${sy.fan_pwm_percent}% power`, fanBad ? 'bad' : ''),
    card('Memory', `${Math.round(sy.ram_percent)}%`, `disk ${Math.round(sy.disk_percent)}%`, sy.ram_percent > 85 ? 'warn' : ''),
    card('Wi-Fi', sy.wifi ? `${sy.wifi.signal_dbm} dBm` : '–', sy.wifi ? (sy.wifi.signal_dbm < -70 ? 'weak' : 'ok') : 'not on Wi-Fi', sy.wifi && sy.wifi.signal_dbm < -70 ? 'warn' : ''),
    card('Power', thr ? 'Throttled' : 'OK', thr ? `flags ${sy.throttled}` : 'no under-voltage', thr ? 'bad' : ''),
    card('Uptime', fmtUptime(sy.uptime_s), sy.ips.join(' · ')),
  ].join('');

  // topics
  $('#topic-table tbody').innerHTML = Object.entries(st.topics).map(([t, hz]) => {
    const [want, label] = EXPECTED[t] || [1, t];
    const lvl = hz <= 0 ? (want ? 'bad' : '') : hz < want * 0.6 ? 'warn' : 'ok';
    return `<tr><td><span class="dot ${lvl}"></span>${label} <span class="muted small">${t}</span></td><td>${hz ? hz.toFixed(1) + ' Hz' : '—'}</td></tr>`;
  }).join('');

  // nodes
  const nodes = st.nodes.filter((n) => !n.startsWith('_') && !n.startsWith('launch_ros'));
  $('#node-count').textContent = `(${nodes.length})`;
  $('#node-list').innerHTML = nodes.map((n) => `<span>${n}</span>`).join('');

  renderProfiles();
  renderDriveWarn();
  renderCamera();
}

/* ---------- launch page ---------- */
function renderProfiles() {
  const box = $('#profile-cards');
  const visible = S.status.profiles.filter((p) => !p.hidden);
  if (box.children.length !== visible.length) {
    box.innerHTML = visible.map((p) => `
      <div class="profile" data-name="${p.name}">
        <div class="head"><span class="title">${p.title}</span><span class="badge"></span></div>
        <div class="muted small">${p.description}${p.requires.length ? ` · needs ${p.requires.join(', ')}` : ''}</div>
        <div class="muted small uptime"></div>
        <div class="actions">
          <button class="btn primary start">Start</button>
          <button class="btn danger stop">Stop</button>
          <button class="btn log-btn">Log</button>
        </div>
        <pre class="log" hidden></pre>
      </div>`).join('');
    box.querySelectorAll('.profile').forEach((el) => {
      const name = el.dataset.name;
      el.querySelector('.start').onclick = () => api(`profiles/${name}/start`, { method: 'POST' }).then(poll1).catch(() => {});
      el.querySelector('.stop').onclick = () => api(`profiles/${name}/stop`, { method: 'POST' }).then(poll1).catch(() => {});
      el.querySelector('.log-btn').onclick = () => {
        if (S.openLogs.has(name)) S.openLogs.delete(name); else S.openLogs.add(name);
        el.querySelector('.log').hidden = !S.openLogs.has(name);
        refreshLog(name);
      };
    });
  }
  for (const p of visible) {
    const el = box.querySelector(`[data-name="${p.name}"]`);
    const b = el.querySelector('.badge');
    b.textContent = p.state + (p.state === 'failed' && p.exit_code !== null ? ` (${p.exit_code})` : '');
    b.className = 'badge ' + p.state;
    el.querySelector('.uptime').textContent = p.uptime ? `running ${fmtUptime(p.uptime)}` : '';
    const busy = p.state === 'starting' || p.state === 'stopping';
    el.querySelector('.start').disabled = busy || p.state === 'running';
    el.querySelector('.stop').disabled = busy || p.state === 'stopped' || p.state === 'failed';
  }
}

function poll1() { return api('status').then((s) => { S.status = s; render(); }); }

async function refreshLog(name) {
  if (!S.openLogs.has(name)) return;
  try {
    const { log } = await api(`profiles/${name}/log?lines=120`);
    const pre = document.querySelector(`[data-name="${name}"] .log`);
    const atBottom = pre.scrollTop + pre.clientHeight >= pre.scrollHeight - 10;
    pre.textContent = log || '(empty)';
    if (atBottom) pre.scrollTop = pre.scrollHeight;
  } catch (e) { /* ignore */ }
  setTimeout(() => refreshLog(name), 2000);
}

$('#stop-all').onclick = () => {
  if (confirm('Stop every running mode?')) api('stop_all', { method: 'POST' }).catch(() => {});
};

/* ---------- E-STOP ---------- */
$('#estop').onclick = async () => {
  if (!S.estop) {
    stopDriving();
    publishCmd(0, 0, 0);
    S.estop = true;
    render();
    api('estop', { method: 'POST', body: { active: true } }).catch(() => {});
  } else if (confirm('Release E-STOP? The robot can move again.')) {
    api('estop', { method: 'POST', body: { active: false } }).then(poll1).catch(() => {});
  }
};

/* ---------- drive ---------- */
const drive = { move: { x: 0, y: 0, on: false }, turn: { x: 0, y: 0, on: false }, keys: new Set(), timer: null, zeros: 0 };

function makePad(el, state, xOnly) {
  const knob = el.querySelector('.knob');
  let id = null;
  const set = (e) => {
    const r = el.getBoundingClientRect();
    const rx = r.width / 2, ry = r.height / 2;
    let x = (e.clientX - r.left - rx) / Math.max(rx - 32, 1);
    let y = xOnly ? 0 : (e.clientY - r.top - ry) / Math.max(ry - 32, 1);
    const m = Math.hypot(x, y);
    if (m > 1) { x /= m; y /= m; }
    state.x = x; state.y = y;
    knob.style.transform = `translate(${x * (rx - 32)}px, ${y * (ry - 32)}px)`;
  };
  el.addEventListener('pointerdown', (e) => {
    if (S.estop) return;
    id = e.pointerId; el.setPointerCapture(id); state.on = true; el.classList.add('active'); set(e); startDriving();
  });
  el.addEventListener('pointermove', (e) => { if (e.pointerId === id) set(e); });
  const end = (e) => {
    if (e.pointerId !== id) return;
    id = null; state.on = false; state.x = 0; state.y = 0; el.classList.remove('active'); knob.style.transform = '';
  };
  el.addEventListener('pointerup', end);
  el.addEventListener('pointercancel', end);
}
makePad($('#pad-move'), drive.move, false);
makePad($('#pad-turn'), drive.turn, true);

const linMax = () => parseFloat($('#lin-max').value);
const angMax = () => parseFloat($('#ang-max').value);
function showLimits() {
  $('#lin-val').textContent = `${linMax().toFixed(2)} m/s`;
  $('#ang-val').textContent = `${angMax().toFixed(1)} rad/s`;
}
$('#lin-max').oninput = showLimits;
$('#ang-max').oninput = showLimits;
showLimits();

function currentCmd() {
  const k = drive.keys;
  let vx = -drive.move.y, vy = -drive.move.x, wz = -drive.turn.x;
  if (k.has('w')) vx = 1; if (k.has('s')) vx = -1;
  if (k.has('a')) vy = 1; if (k.has('d')) vy = -1;
  if (k.has('q')) wz = 1; if (k.has('e')) wz = -1;
  return [vx * linMax(), vy * linMax(), wz * angMax()];
}

function startDriving() {
  if (drive.timer) return;
  drive.zeros = 0;
  drive.timer = setInterval(() => {
    const active = drive.move.on || drive.turn.on || drive.keys.size > 0;
    if (S.estop || !S.rosOk || document.hidden) { stopDriving(); return; }
    const [vx, vy, wz] = active ? currentCmd() : [0, 0, 0];
    publishCmd(vx, vy, wz);
    $('#cmd-readout').textContent = `vx ${vx.toFixed(2)} · vy ${vy.toFixed(2)} · wz ${wz.toFixed(2)}`;
    if (!active && ++drive.zeros >= 3) stopDriving();   // a few zeros after release, then go quiet
    if (active) drive.zeros = 0;
  }, 100);
}

function stopDriving() {
  if (drive.timer) { clearInterval(drive.timer); drive.timer = null; }
  drive.keys.clear();
  publishCmd(0, 0, 0);
  $('#cmd-readout').textContent = 'vx 0.00 · vy 0.00 · wz 0.00';
}

document.addEventListener('keydown', (e) => {
  if (e.target.tagName === 'INPUT') return;
  if (e.code === 'Space') { e.preventDefault(); if (!S.estop) $('#estop').click(); return; }
  if (S.tab !== 'drive' || S.estop) return;
  const key = e.key.toLowerCase();
  if ('wasdqe'.includes(key) && key.length === 1) { drive.keys.add(key); startDriving(); }
});
document.addEventListener('keyup', (e) => drive.keys.delete(e.key.toLowerCase()));
document.addEventListener('visibilitychange', () => { if (document.hidden) stopDriving(); });
window.addEventListener('blur', () => drive.keys.clear());

function profileState(name) {
  const p = S.status && S.status.profiles.find((x) => x.name === name);
  return p ? p.state : 'stopped';
}

function renderDriveWarn() {
  const robot = profileState('robot');
  let msg = '';
  if (S.estop) msg = 'E-STOP is active.';
  else if (!S.rosOk) msg = 'Not connected to ROS.';
  else if (robot !== 'running') msg = robot === 'starting' ? 'Robot is starting…' : 'Robot mode is not running: start it on the Launch page.';
  $('#drive-warn').textContent = msg;
}

/* ---------- camera ---------- */
let camOn = false;
function renderCamera() {
  const running = profileState('camera') === 'running' || profileState('camera') === 'starting';
  const want = S.tab === 'camera' && running && S.cfg;
  const img = $('#camera-img');
  if (want && !camOn) {
    img.src = `http://${host}:${S.cfg.video_port}/stream?topic=/aruco/image_raw&type=mjpeg&quality=60&width=640&height=360`;
    camOn = true;
  } else if (!want && camOn) {
    img.removeAttribute('src');   // stop streaming when not looking at it, saves Pi CPU
    camOn = false;
  }
  $('#camera-empty').hidden = running;
}
$('#camera-start').onclick = () => api('profiles/camera/start', { method: 'POST' }).then(poll1).catch(() => {});

function subscribeMarkers(on) {
  if (on && S.ros && S.rosOk && !S.markerSub) {
    S.markerSub = new ROSLIB.Topic({ ros: S.ros, name: '/aruco/detections', messageType: 'std_msgs/msg/String', throttle_rate: 200 });
    S.markerSub.subscribe((m) => {
      let d;
      try { d = JSON.parse(m.data); } catch (e) { return; }
      const rows = (d.markers || []).map((k) => {
        const pos = k.position_base_link_m ? k.position_base_link_m.map((v) => v.toFixed(2)).join(', ') : '—';
        const dist = k.distance_m !== undefined && k.distance_m !== null ? `${k.distance_m.toFixed(2)} m` : '—';
        return `<tr><td>ID <b>${k.id}</b> <span class="muted small">pos [${pos}]</span></td><td>${dist}</td></tr>`;
      });
      $('#marker-table tbody').innerHTML = rows.join('') || '<tr><td class="muted">No markers in view</td><td></td></tr>';
    });
  } else if (!on && S.markerSub) {
    S.markerSub.unsubscribe();
    S.markerSub = null;
  }
}

/* ---------- tabs ---------- */
document.querySelectorAll('.tabs button').forEach((b) => {
  b.onclick = () => {
    S.tab = b.dataset.tab;
    document.querySelectorAll('.tabs button').forEach((x) => x.classList.toggle('active', x === b));
    document.querySelectorAll('.page').forEach((p) => { p.hidden = p.id !== 'page-' + S.tab; });
    if (S.tab !== 'drive') stopDriving();
    subscribeMarkers(S.tab === 'camera');
    if (S.status) render();
  };
});

/* ---------- boot ---------- */
let started = false;
async function start() {
  if (started) return;
  try {
    S.cfg = await api('config');
  } catch (e) { return; }
  started = true;
  connectRos();
  poll();
}

$('#host').textContent = host;
renderChips();
if (S.pin) start(); else askPin('');
