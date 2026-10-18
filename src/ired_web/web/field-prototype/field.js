/* iRED field view prototype.
 * One `model` (robot pose, LiDAR points, stations, goal) drawn by:
 *  - the top view: plain Canvas 2D, works on every phone and costs almost nothing
 *  - the 3D view: three.js, loaded only when picked, needs WebGL
 * Demo mode simulates the robot; the real app fills the same model from ROS. */

const SIZE = 5;                                  // 5 x 5 zones of 1 m
const START = { x0: 2, x1: 5, y0: 0, y1: 1 };    // C_Z31..C_Z51
const RED_ZONE = 'C_Z32';

const css = (n) => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
const zoneName = (x, y) => (x >= 0 && x < SIZE && y >= 0 && y < SIZE) ? `C_Z${Math.floor(x) + 1}${Math.floor(y) + 1}` : null;
const zoneCenter = (z) => [Number(z[3]) - 0.5, Number(z[4]) - 0.5];

let colors;
function readColors() {
  colors = { floor: css('--floor'), line: css('--line'), text: css('--text'), muted: css('--muted'),
    cyan: css('--cyan'), magenta: css('--magenta'), wood: css('--wood'), amber: css('--amber') };
}
readColors();

const model = {
  size: SIZE,
  robot: { x: 4.5, y: 0.5, yaw: Math.PI },
  scan: [],                                      // [[x, y], ...] in the map frame
  stations: [],                                  // [{name, zone, x, y, rotation}]
  goal: null, selected: null,
  layers: { scan: true, path: true, boxes: true, model: true },
  floorCanvas: document.createElement('canvas'),
  floorVersion: 0,
  meshUrl: 'ired_body_lite.stl',
};

/* ------------------------------------------------ floor drawing (shared) */
model.floorCanvas.width = model.floorCanvas.height = 1000;       // 200 px per metre
function drawFloor() {
  const g = model.floorCanvas.getContext('2d');
  const px = (x) => x * 200, py = (y) => (SIZE - y) * 200;
  g.fillStyle = colors.floor;
  g.fillRect(0, 0, 1000, 1000);
  for (let i = 0; i < SIZE; i++) for (let j = 0; j < SIZE; j++) {
    if ((i + j) % 2) { g.fillStyle = 'rgba(128,150,165,.08)'; g.fillRect(px(i), py(j + 1), 200, 200); }
  }
  g.fillStyle = colors.cyan + '24';
  g.fillRect(px(START.x0), py(START.y1), px(START.x1 - START.x0), 200);
  if (model.selected) {
    const [cx, cy] = zoneCenter(model.selected);
    g.fillStyle = colors.magenta + '3a';
    g.fillRect(px(cx - 0.5), py(cy + 0.5), 200, 200);
  }
  g.strokeStyle = colors.line; g.lineWidth = 3;
  for (let k = 0; k <= SIZE; k++) {
    g.beginPath(); g.moveTo(px(k), 0); g.lineTo(px(k), 1000); g.stroke();
    g.beginPath(); g.moveTo(0, py(k)); g.lineTo(1000, py(k)); g.stroke();
  }
  const [rx, ry] = zoneCenter(RED_ZONE);
  g.strokeStyle = '#e5383b'; g.lineWidth = 8; g.setLineDash([22, 14]);
  g.strokeRect(px(rx - 0.5) + 8, py(ry + 0.5) + 8, 184, 184);
  g.setLineDash([]);
  g.font = '600 26px system-ui, sans-serif';
  g.fillStyle = colors.muted;
  for (let i = 0; i < SIZE; i++) for (let j = 0; j < SIZE; j++) g.fillText(`C_Z${i + 1}${j + 1}`, px(i) + 12, py(j) - 14);
  g.font = '700 30px system-ui, sans-serif';
  g.fillStyle = colors.cyan;
  g.fillText('START', px(START.x1) - 120, py(START.y1) + 40);
  model.floorVersion++;
}

/* ------------------------------------------------ top view (Canvas 2D) */
const host = document.getElementById('view');
const top = document.createElement('canvas');
host.appendChild(top);
const tv = { scale: 1, ox: 0, oy: 0 };          // metres -> CSS pixels

function fitTop() {
  const w = host.clientWidth, h = host.clientHeight, dpr = Math.min(devicePixelRatio, 2);
  top.width = w * dpr; top.height = h * dpr;
  top.style.width = w + 'px'; top.style.height = h + 'px';
  const pad = 18;
  tv.scale = Math.min((w - 2 * pad) / SIZE, (h - 2 * pad) / SIZE);
  tv.ox = (w - SIZE * tv.scale) / 2;
  tv.oy = (h - SIZE * tv.scale) / 2;
  tv.dpr = dpr;
}
const sx = (x) => tv.ox + x * tv.scale, sy = (y) => tv.oy + (SIZE - y) * tv.scale;

function drawTop() {
  const g = top.getContext('2d');
  g.setTransform(tv.dpr, 0, 0, tv.dpr, 0, 0);
  g.clearRect(0, 0, top.width, top.height);
  g.drawImage(model.floorCanvas, sx(0), sy(SIZE), SIZE * tv.scale, SIZE * tv.scale);
  g.strokeStyle = 'rgba(138,160,176,.7)'; g.lineWidth = 3;          // walls
  g.strokeRect(sx(0) - 2, sy(SIZE) - 2, SIZE * tv.scale + 4, SIZE * tv.scale + 4);

  if (model.layers.boxes) for (const st of model.stations) {
    g.save();
    g.translate(sx(st.x), sy(st.y));
    g.rotate(-st.rotation * Math.PI / 180);                           // screen y is flipped
    const b = 0.2 * tv.scale;
    g.fillStyle = colors.wood; g.fillRect(-b / 2, -b / 2, b, b);
    g.fillStyle = '#fff'; g.fillRect(b / 2 - 3, -b * 0.18, 3, b * 0.36);  // input-side tag
    g.fillStyle = colors.cyan;                                          // arrow: input side
    g.beginPath(); g.moveTo(b / 2 + 4, -5); g.lineTo(b / 2 + 14, 0); g.lineTo(b / 2 + 4, 5); g.fill();
    g.restore();
    g.fillStyle = colors.text; g.font = '600 12px system-ui, sans-serif';
    g.fillText(st.name, sx(st.x) - b / 2, sy(st.y) - b / 2 - 5);
  }

  if (model.goal && model.layers.path) {
    g.strokeStyle = colors.cyan; g.lineWidth = 2.5; g.setLineDash([8, 6]);
    g.beginPath(); g.moveTo(sx(model.robot.x), sy(model.robot.y)); g.lineTo(sx(model.goal[0]), sy(model.goal[1])); g.stroke();
    g.setLineDash([]);
  }
  if (model.goal) {
    g.strokeStyle = colors.magenta; g.lineWidth = 4;
    g.beginPath(); g.arc(sx(model.goal[0]), sy(model.goal[1]), 0.2 * tv.scale, 0, Math.PI * 2); g.stroke();
  }

  if (model.layers.scan) {
    g.fillStyle = colors.amber;
    for (const [x, y] of model.scan) g.fillRect(sx(x) - 1.5, sy(y) - 1.5, 3, 3);
  }

  const r = model.robot, R = 0.215 * tv.scale;                       // robot: real 43 cm base
  g.save();
  g.translate(sx(r.x), sy(r.y));
  g.rotate(-r.yaw);
  g.fillStyle = colors.cyan + '40'; g.strokeStyle = colors.cyan; g.lineWidth = 2.5;
  g.beginPath(); g.arc(0, 0, R, 0, Math.PI * 2); g.fill(); g.stroke();
  g.fillStyle = colors.cyan;
  g.beginPath(); g.moveTo(R * 1.35, 0); g.lineTo(R * 0.5, R * 0.42); g.lineTo(R * 0.5, -R * 0.42); g.fill();
  g.restore();
}

/* ------------------------------------------------ 3D view (lazy) */
let view3d = null;
let mode = '2d';
async function setMode(m) {
  if (m === '3d' && !view3d) {
    try {
      const { createView3D } = await import('./view3d.js');
      view3d = createView3D(host, model, colors);
      bindTap(view3d.canvas, (x, y) => view3d.pick(x, y));
    } catch (e) {
      say('3D needs WebGL, which this browser does not have. Showing the top view.');
      m = '2d';
    }
  }
  mode = m;
  top.hidden = m !== '2d';
  if (view3d) view3d.canvas.hidden = m !== '3d';
  document.querySelectorAll('[data-view]').forEach((b) => b.setAttribute('aria-checked', String(b.dataset.view === m)));
}

/* ------------------------------------------------ HUD, taps, sheet */
const $ = (id) => document.getElementById(id);
let notice = null, noticeUntil = 0;
function say(text) { notice = text; noticeUntil = performance.now() + 4000; }

const demo = { inZoneSince: null, stopped: false, counted: [] };
function updateHud(now) {
  const r = model.robot, z = zoneName(r.x, r.y);
  $('zone-now').textContent = z || 'Off the field';
  $('zone-now').classList.toggle('off', !z);
  const deg = ((r.yaw * 180 / Math.PI) % 360 + 360) % 360;
  $('pose').textContent = `x ${r.x.toFixed(2)} m, y ${r.y.toFixed(2)} m, heading ${deg.toFixed(0)}°`;
  const t = $('task');
  if (notice && now < noticeUntil) t.textContent = notice;
  else if (demo.stopped) t.textContent = 'Stopped. Tap STOP again to release.';
  else if (!model.goal) t.textContent = demo.counted.length ? `Counted ${demo.counted.join(', ')}. Tap a zone to send the robot.` : 'Idle. Tap a zone to send the robot.';
  else if (demo.inZoneSince !== null) t.innerHTML = `In <b>${model.selected}</b>, staying ${Math.min(5, (now - demo.inZoneSince) / 1000).toFixed(1)} of 5 s`;
  else t.innerHTML = `Driving to <b>${model.selected}</b>, ${Math.hypot(model.goal[0] - r.x, model.goal[1] - r.y).toFixed(1)} m left`;
}

let tapMode = 'goal', pending = null;
function bindTap(el, toMap) {
  let down = null;
  el.addEventListener('pointerdown', (e) => { down = { x: e.clientX, y: e.clientY, t: performance.now() }; });
  el.addEventListener('pointerup', (e) => {
    if (!down || Math.hypot(e.clientX - down.x, e.clientY - down.y) > 8 || performance.now() - down.t > 600) return;
    const p = toMap(e.clientX, e.clientY);
    const zone = p && zoneName(p.x, p.y);
    if (!zone) return;
    if (tapMode === 'pose') {
      pending = { kind: 'pose', x: p.x, y: p.y };
      $('sheet-text').textContent = `Set the robot's start position to x ${p.x.toFixed(2)}, y ${p.y.toFixed(2)} in ${zone}?`;
      $('sheet-ok').textContent = 'Set start';
    } else {
      const st = model.stations.find((s) => s.zone === zone);
      pending = { kind: 'goal', zone };
      $('sheet-text').textContent = st ? `${zone} has station ${st.name}. Send the robot next to it?` : `Send the robot to ${zone}?`;
      $('sheet-ok').textContent = 'Send';
      model.selected = zone;
      drawFloor();
    }
    $('sheet').hidden = false;
  });
}
bindTap(top, (cx, cy) => {
  const b = top.getBoundingClientRect();
  return { x: (cx - b.left - tv.ox) / tv.scale, y: SIZE - (cy - b.top - tv.oy) / tv.scale };
});

$('sheet-cancel').onclick = () => {
  $('sheet').hidden = true;
  pending = null;
  if (!model.goal) model.selected = null;
  else model.selected = zoneName(...model.goal);
  drawFloor();
};
$('sheet-ok').onclick = () => {
  $('sheet').hidden = true;
  if (!pending) return;
  if (pending.kind === 'pose') {
    Object.assign(model.robot, { x: pending.x, y: pending.y });
    say('Start position set.');
  } else {
    const [cx, cy] = zoneCenter(pending.zone);
    const st = model.stations.find((s) => s.zone === pending.zone);
    model.goal = st ? [cx + 0.4 * Math.sign(model.robot.x - cx || 1), cy] : [cx, cy];
    demo.inZoneSince = null;
  }
  pending = null;
};

document.querySelectorAll('[data-view]').forEach((b) => b.onclick = () => setMode(b.dataset.view));
document.querySelectorAll('[data-mode]').forEach((b) => b.onclick = () => {
  tapMode = b.dataset.mode;
  document.querySelectorAll('[data-mode]').forEach((x) => x.setAttribute('aria-checked', String(x === b)));
});
document.querySelectorAll('[data-layer]').forEach((b) => b.onclick = () => {
  const k = b.dataset.layer;
  model.layers[k] = !model.layers[k];
  b.setAttribute('aria-pressed', String(model.layers[k]));
});
$('estop').onclick = () => {
  demo.stopped = !demo.stopped;
  $('estop').textContent = demo.stopped ? 'RELEASE' : 'STOP';
};
matchMedia('(prefers-color-scheme: light)').addEventListener('change', () => { readColors(); drawFloor(); });
new ResizeObserver(fitTop).observe(host);

/* ------------------------------------------------ demo robot */
function simScan() {
  // the LiDAR plane is above the 30 cm stations, so only the walls answer (as on the real robot)
  const { x: ox, y: oy } = model.robot, pts = [];
  for (let i = 0; i < 360; i++) {
    const a = i * Math.PI / 180, dx = Math.cos(a), dy = Math.sin(a);
    let t = Infinity;
    if (dx > 1e-9) t = Math.min(t, (SIZE - ox) / dx); else if (dx < -1e-9) t = Math.min(t, -ox / dx);
    if (dy > 1e-9) t = Math.min(t, (SIZE - oy) / dy); else if (dy < -1e-9) t = Math.min(t, -oy / dy);
    t += (Math.random() - 0.5) * 0.02;
    pts.push([ox + t * dx, oy + t * dy]);
  }
  model.scan = pts;
}

let last = performance.now(), lastScan = 0;
function frame(now) {
  const dt = Math.min(0.1, (now - last) / 1000);
  last = now;
  const r = model.robot;
  if (model.goal && !demo.stopped) {
    const dx = model.goal[0] - r.x, dy = model.goal[1] - r.y, d = Math.hypot(dx, dy);
    if (d > 0.02) {
      const v = Math.min(0.25, d * 1.5) * dt;                       // omni: slide straight, keep heading
      r.x += dx / d * v; r.y += dy / d * v;
      demo.inZoneSince = null;
    } else if (demo.inZoneSince === null) demo.inZoneSince = now;
    else if (now - demo.inZoneSince > 5000) {
      demo.counted.push(model.selected);
      model.goal = null; model.selected = null; drawFloor();
    }
  }
  if (now - lastScan > 130) { simScan(); updateHud(now); lastScan = now; }   // ~7.5 Hz like the LiDAR
  if (mode === '3d' && view3d) view3d.draw(); else drawTop();
  requestAnimationFrame(frame);
}

for (const [name, zone, rotation] of [['C-BS', 'C_Z24', 90], ['C-CS1', 'C_Z43', 0], ['C-RS1', 'C_Z15', 135]]) {
  const [x, y] = zoneCenter(zone);
  model.stations.push({ name, zone, x, y, rotation });
}
drawFloor();
fitTop();
requestAnimationFrame(frame);
