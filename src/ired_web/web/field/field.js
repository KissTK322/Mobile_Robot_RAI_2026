/* Field view for the Map tab.
 *
 * One `model` (robot pose, LiDAR points, map, path, stations, goal) in FIELD
 * coordinates (origin = outer corner of C_Z11, x along columns, y along rows),
 * drawn by the top view (Canvas 2D, any phone) or the 3D view (three.js,
 * loaded on demand, needs WebGL).
 *
 * Data comes in through `view.ros.*` as the JSON objects rosbridge delivers, so
 * the same code runs on live topics and on test messages. Commands go out
 * through opts.onGoal / opts.onInitialPose, in MAP coordinates; they return an
 * error text when the command could not be sent. */

const SIZE = 5;
const START = { x0: 2, x1: 5 };                 // C_Z31..C_Z51
const RED_ZONE = 'C_Z32';

const zoneName = (x, y) => (x >= 0 && x < SIZE && y >= 0 && y < SIZE) ? `C_Z${Math.floor(x) + 1}${Math.floor(y) + 1}` : null;
const zoneCenter = (z) => [Number(z[3]) - 0.5, Number(z[4]) - 0.5];
const yawOf = (q) => Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));

/* pose algebra in 2D: a ⊕ b and a⁻¹ ⊕ b */
function compose(a, b) {
  const c = Math.cos(a.yaw), s = Math.sin(a.yaw);
  return { x: a.x + c * b.x - s * b.y, y: a.y + s * b.x + c * b.y, yaw: wrap(a.yaw + b.yaw) };
}
function between(a, b) {
  const c = Math.cos(-a.yaw), s = Math.sin(-a.yaw), dx = b.x - a.x, dy = b.y - a.y;
  return { x: c * dx - s * dy, y: s * dx + c * dy, yaw: wrap(b.yaw - a.yaw) };
}

const MARKUP = `
  <div class="fv-view"></div>
  <section class="fv-hud" aria-live="polite">
    <div class="fv-zone">Off the field</div>
    <div class="fv-pose"></div>
    <div class="fv-task"></div>
    <div class="fv-source"></div>
  </section>
  <div class="fv-toolbar">
    <div class="fv-seg" role="radiogroup" aria-label="View">
      <button role="radio" aria-checked="true" data-view="2d">Top</button>
      <button role="radio" aria-checked="false" data-view="3d">3D</button>
    </div>
    <div class="fv-seg" aria-label="Show">
      <button aria-pressed="true" data-layer="scan"><i class="fv-sw scan"></i>LiDAR</button>
      <button aria-pressed="true" data-layer="map"><i class="fv-sw map"></i>Map</button>
      <button aria-pressed="true" data-layer="path"><i class="fv-sw path"></i>Path</button>
      <button aria-pressed="true" data-layer="boxes"><i class="fv-sw box"></i>Stations</button>
      <button aria-pressed="true" data-layer="model"><i class="fv-sw bot"></i>Robot</button>
    </div>
    <div class="fv-seg" role="radiogroup" aria-label="Tap does">
      <button role="radio" aria-checked="true" data-mode="goal">Tap: send</button>
      <button role="radio" aria-checked="false" data-mode="pose">Tap: set start</button>
    </div>
  </div>
  <div class="fv-sheet" hidden>
    <p></p>
    <div class="acts"><button class="go"></button><button class="no">Cancel</button></div>
  </div>`;

export function createFieldView(root, opts = {}) {
  root.classList.add('fv');
  root.innerHTML = MARKUP;
  const q = (s) => root.querySelector(s);
  const origin = opts.origin || [0, 0];            // map coordinates of the field corner
  const toField = (x, y) => [x - origin[0], y - origin[1]];

  let colors;
  const css = (n) => getComputedStyle(root).getPropertyValue(n).trim();
  function readColors() {
    colors = { floor: css('--fv-floor'), line: css('--line'), text: css('--text'), muted: css('--muted'),
      cyan: css('--fv-cyan'), magenta: css('--fv-magenta'), wood: css('--fv-wood'), amber: css('--fv-amber') };
  }
  readColors();

  const model = {
    size: SIZE,
    robot: { x: 4.5, y: 0.5, yaw: Math.PI }, robotKnown: false, frame: 'none',
    scan: [], path: [], stations: [], goal: null, selected: null,
    layers: { scan: true, map: true, path: true, boxes: true, model: true },
    floorCanvas: document.createElement('canvas'), floorVersion: 0,
    meshUrl: opts.meshUrl || 'field/ired_body_lite.stl',
    challenge: null, live: false,
  };
  const loc = { amcl: null, odomAtAmcl: null, odom: null, scanMsg: null, lastMsg: 0 };
  const mapLayer = { canvas: document.createElement('canvas'), info: null };

  /* ------------------------------------------------ floor (shared by both views) */
  model.floorCanvas.width = model.floorCanvas.height = 1000;     // 200 px per metre
  function drawFloor() {
    const g = model.floorCanvas.getContext('2d');
    const px = (x) => x * 200, py = (y) => (SIZE - y) * 200;
    g.fillStyle = colors.floor;
    g.fillRect(0, 0, 1000, 1000);
    for (let i = 0; i < SIZE; i++) for (let j = 0; j < SIZE; j++) {
      if ((i + j) % 2) { g.fillStyle = 'rgba(128,150,165,.08)'; g.fillRect(px(i), py(j + 1), 200, 200); }
    }
    if (model.layers.map && mapLayer.info) {                        // occupancy map under the grid
      const { res, fx, fy } = mapLayer.info;
      g.save();
      g.translate(px(fx), py(fy));
      g.scale(res * 200, -res * 200);
      g.imageSmoothingEnabled = false;
      g.drawImage(mapLayer.canvas, 0, 0);
      g.restore();
    }
    g.fillStyle = colors.cyan + '24';
    g.fillRect(px(START.x0), py(1), px(START.x1 - START.x0), 200);
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
    g.fillText('START', px(START.x1) - 120, py(1) + 40);
    model.floorVersion++;
  }

  /* ------------------------------------------------ top view */
  const host = q('.fv-view');
  const top = document.createElement('canvas');
  host.appendChild(top);
  const tv = { scale: 1, ox: 0, oy: 0, dpr: 1 };
  function fitTop() {
    const w = host.clientWidth, h = host.clientHeight, dpr = Math.min(devicePixelRatio, 2);
    if (!w || !h) return;
    top.width = w * dpr; top.height = h * dpr;
    top.style.width = w + 'px'; top.style.height = h + 'px';
    const padTop = 16, padBottom = root.querySelector('.fv-toolbar').offsetHeight + 18;   // clear of the toolbar
    tv.scale = Math.min((w - 32) / SIZE, (h - padTop - padBottom) / SIZE);
    tv.ox = (w - SIZE * tv.scale) / 2;
    tv.oy = padTop + (h - padTop - padBottom - SIZE * tv.scale) / 2;
    const hud = root.querySelector('.fv-hud'), hudW = hud.offsetWidth + 24, hudH = hud.offsetHeight + 20;
    if (w - SIZE * tv.scale - 16 >= hudW) tv.ox = Math.max(tv.ox, hudW);          // wide: field right of the HUD
    else if (tv.oy < hudH) {                                                         // tall: field below the HUD
      tv.scale = Math.min(tv.scale, (h - hudH - padBottom) / SIZE);
      tv.ox = (w - SIZE * tv.scale) / 2;
      tv.oy = hudH;
    }
    tv.dpr = dpr;
  }
  const sx = (x) => tv.ox + x * tv.scale, sy = (y) => tv.oy + (SIZE - y) * tv.scale;

  function drawTop() {
    const g = top.getContext('2d');
    g.setTransform(tv.dpr, 0, 0, tv.dpr, 0, 0);
    g.clearRect(0, 0, top.width, top.height);
    g.drawImage(model.floorCanvas, sx(0), sy(SIZE), SIZE * tv.scale, SIZE * tv.scale);
    g.strokeStyle = 'rgba(138,160,176,.7)'; g.lineWidth = 3;
    g.strokeRect(sx(0) - 2, sy(SIZE) - 2, SIZE * tv.scale + 4, SIZE * tv.scale + 4);

    if (model.layers.boxes) for (const st of model.stations) {
      g.save();
      g.translate(sx(st.x), sy(st.y));
      g.rotate(-st.rotation * Math.PI / 180);
      const b = 0.2 * tv.scale;
      g.fillStyle = colors.wood; g.fillRect(-b / 2, -b / 2, b, b);
      g.fillStyle = '#fff'; g.fillRect(b / 2 - 3, -b * 0.18, 3, b * 0.36);
      g.fillStyle = colors.cyan;
      g.beginPath(); g.moveTo(b / 2 + 4, -5); g.lineTo(b / 2 + 14, 0); g.lineTo(b / 2 + 4, 5); g.fill();
      g.restore();
      g.fillStyle = colors.text; g.font = '600 12px system-ui, sans-serif';
      g.fillText(st.name, sx(st.x) - b / 2, sy(st.y) - b / 2 - 5);
    }

    const route = model.path.length ? model.path : (model.goal ? [[model.robot.x, model.robot.y], model.goal] : []);
    if (route.length && model.layers.path) {
      g.strokeStyle = colors.cyan; g.lineWidth = 2.5; g.setLineDash([8, 6]);
      g.beginPath();
      route.forEach(([x, y], i) => (i ? g.lineTo(sx(x), sy(y)) : g.moveTo(sx(x), sy(y))));
      g.stroke(); g.setLineDash([]);
    }
    if (model.goal) {
      g.strokeStyle = colors.magenta; g.lineWidth = 4;
      g.beginPath(); g.arc(sx(model.goal[0]), sy(model.goal[1]), 0.2 * tv.scale, 0, Math.PI * 2); g.stroke();
    }
    if (model.layers.scan) {
      g.fillStyle = colors.amber;
      for (const [x, y] of model.scan) g.fillRect(sx(x) - 1.5, sy(y) - 1.5, 3, 3);
    }
    if (model.robotKnown) {
      const r = model.robot, R = 0.215 * tv.scale;
      g.save();
      g.translate(sx(r.x), sy(r.y));
      g.rotate(-r.yaw);
      g.fillStyle = colors.cyan + '40'; g.strokeStyle = colors.cyan; g.lineWidth = 2.5;
      g.beginPath(); g.arc(0, 0, R, 0, Math.PI * 2); g.fill(); g.stroke();
      g.fillStyle = colors.cyan;
      g.beginPath(); g.moveTo(R * 1.35, 0); g.lineTo(R * 0.5, R * 0.42); g.lineTo(R * 0.5, -R * 0.42); g.fill();
      g.restore();
    }
  }

  /* ------------------------------------------------ 3D view (lazy) */
  let view3d = null, mode = '2d';
  async function setMode(m) {
    if (m === '3d' && !view3d) {
      try {
        const { createView3D } = await import('./view3d.js');
        view3d = createView3D(host, model, colors);
        bindTap(view3d.canvas, (x, y) => view3d.pick(x, y));
      } catch (e) {
        notice('3D needs WebGL, which this browser does not have. Showing the top view.');
        m = '2d';
      }
    }
    mode = m;
    top.hidden = m !== '2d';
    if (view3d) view3d.canvas.hidden = m !== '3d';
    root.querySelectorAll('[data-view]').forEach((b) => b.setAttribute('aria-checked', String(b.dataset.view === m)));
  }

  /* ------------------------------------------------ HUD */
  let noticeText = null, noticeUntil = 0;
  function notice(t) { noticeText = t; noticeUntil = performance.now() + 4000; }
  let arrivedSince = null;
  function updateHud(now) {
    const r = model.robot, z = model.robotKnown ? zoneName(r.x, r.y) : null;
    q('.fv-zone').textContent = !model.robotKnown ? 'Robot position unknown' : (z || 'Off the field');
    q('.fv-zone').classList.toggle('off', !z);
    const deg = ((r.yaw * 180 / Math.PI) % 360 + 360) % 360;
    q('.fv-pose').textContent = model.robotKnown
      ? `x ${r.x.toFixed(2)} m, y ${r.y.toFixed(2)} m, heading ${deg.toFixed(0)}°${model.frame === 'odom' ? ' (odometry only, no map yet)' : ''}`
      : 'Start Robot mode, then Navigation or Mapping.';
    const t = q('.fv-task'), c = model.challenge;
    if (noticeText && now < noticeUntil) t.textContent = noticeText;
    else if (c && c.state && c.state !== 'WAIT_GAME') {
      t.innerHTML = c.state === 'DONE' ? `Challenge done: ${c.visited.length} zones counted`
        : `Challenge ${c.target ? `driving to <b>${c.target}</b>` : c.state.toLowerCase()}, ${c.visited.length} counted, ${c.plan.length} left`;
    } else if (model.goal) {
      t.innerHTML = arrivedSince ? `At <b>${model.selected}</b>` : `Driving to <b>${model.selected}</b>, ${Math.hypot(model.goal[0] - r.x, model.goal[1] - r.y).toFixed(1)} m left`;
    } else t.textContent = 'Tap a zone to send the robot.';
    const src = q('.fv-source');
    const fresh = model.live && now - loc.lastMsg < 2000;
    src.textContent = model.live ? (fresh ? 'Live from the robot' : 'Waiting for data from the robot') : 'Demo data, not the robot';
    src.classList.toggle('live', fresh);
  }

  /* ------------------------------------------------ taps and the confirm sheet */
  let tapMode = 'goal', pending = null;
  const sheet = q('.fv-sheet');
  function bindTap(el, toField) {
    let down = null;
    el.addEventListener('pointerdown', (e) => { down = { x: e.clientX, y: e.clientY, t: performance.now() }; });
    el.addEventListener('pointerup', (e) => {
      if (!down || Math.hypot(e.clientX - down.x, e.clientY - down.y) > 8 || performance.now() - down.t > 600) return;
      const p = toField(e.clientX, e.clientY);
      const zone = p && zoneName(p.x, p.y);
      if (!zone) return;
      if (tapMode === 'pose') {
        pending = { kind: 'pose', x: p.x, y: p.y };
        sheet.querySelector('p').textContent = `Set the robot's start position to x ${p.x.toFixed(2)}, y ${p.y.toFixed(2)} in ${zone}, facing ${(((model.robot.yaw * 180 / Math.PI) % 360 + 360) % 360).toFixed(0)}°?`;
        sheet.querySelector('.go').textContent = 'Set start';
      } else {
        const st = model.stations.find((s) => s.zone === zone);
        pending = { kind: 'goal', zone };
        sheet.querySelector('p').textContent = st ? `${zone} has station ${st.name}. Send the robot next to it?` : `Send the robot to ${zone}?`;
        sheet.querySelector('.go').textContent = 'Send';
        model.selected = zone;
        drawFloor();
      }
      sheet.hidden = false;
    });
  }
  bindTap(top, (cx, cy) => {
    const b = top.getBoundingClientRect();
    return { x: (cx - b.left - tv.ox) / tv.scale, y: SIZE - (cy - b.top - tv.oy) / tv.scale };
  });
  sheet.querySelector('.no').onclick = () => {
    sheet.hidden = true; pending = null;
    model.selected = model.goal ? zoneName(...model.goal) : null;
    drawFloor();
  };
  sheet.querySelector('.go').onclick = () => {
    sheet.hidden = true;
    if (!pending) return;
    if (pending.kind === 'pose') {
      const yaw = model.robot.yaw;
      const err = opts.onInitialPose?.({ x: pending.x + origin[0], y: pending.y + origin[1], yaw });
      if (!model.live) Object.assign(model.robot, { x: pending.x, y: pending.y });
      notice(err || 'Start position sent.');
    } else {
      const [cx, cy] = zoneCenter(pending.zone);
      const st = model.stations.find((s) => s.zone === pending.zone);
      model.goal = st ? [cx + 0.4 * Math.sign(model.robot.x - cx || 1), cy] : [cx, cy];
      model.path = [];
      arrivedSince = null;
      const err = opts.onGoal?.({ x: model.goal[0] + origin[0], y: model.goal[1] + origin[1], yaw: model.robot.yaw, zone: pending.zone });
      if (err) {                                    // nothing was sent: do not pretend the robot is going
        notice(err);
        model.goal = null; model.selected = null; drawFloor();
      }
    }
    pending = null;
  };

  root.querySelectorAll('[data-view]').forEach((b) => b.onclick = () => setMode(b.dataset.view));
  root.querySelectorAll('[data-mode]').forEach((b) => b.onclick = () => {
    tapMode = b.dataset.mode;
    root.querySelectorAll('[data-mode]').forEach((x) => x.setAttribute('aria-checked', String(x === b)));
  });
  root.querySelectorAll('[data-layer]').forEach((b) => b.onclick = () => {
    const k = b.dataset.layer;
    model.layers[k] = !model.layers[k];
    b.setAttribute('aria-pressed', String(model.layers[k]));
    if (k === 'map') drawFloor();
  });
  const mq = matchMedia('(prefers-color-scheme: dark)');
  mq.addEventListener('change', () => { readColors(); drawFloor(); });
  const ro = new ResizeObserver(fitTop);
  ro.observe(host);

  /* ------------------------------------------------ ROS message handlers (rosbridge JSON) */
  function updateRobot() {
    let p = null;
    if (loc.amcl && loc.odomAtAmcl && loc.odom) { p = compose(loc.amcl, between(loc.odomAtAmcl, loc.odom)); model.frame = 'map'; }
    else if (loc.amcl) { p = loc.amcl; model.frame = 'map'; }
    else if (loc.odom) { p = loc.odom; model.frame = 'odom'; }
    if (!p) return;
    const [fx, fy] = model.frame === 'map' ? toField(p.x, p.y) : [p.x, p.y];
    Object.assign(model.robot, { x: fx, y: fy, yaw: p.yaw });
    model.robotKnown = true;
    updateScanPoints();
  }
  function updateScanPoints() {
    const m = loc.scanMsg;
    if (!m || !model.robotKnown) return;
    const r = model.robot, pts = [], n = m.ranges.length;
    for (let i = 0; i < n; i++) {
      const d = m.ranges[i];
      if (!(d > m.range_min && d < m.range_max)) continue;      // also drops NaN / Inf
      const a = r.yaw + m.angle_min + i * m.angle_increment;   // base_scan sits on the robot centre
      pts.push([r.x + d * Math.cos(a), r.y + d * Math.sin(a)]);
    }
    model.scan = pts;
  }
  const seen = () => { loc.lastMsg = performance.now(); };
  const ros = {
    amcl(msg) {          // geometry_msgs/PoseWithCovarianceStamped on /amcl_pose
      const p = msg.pose.pose;
      loc.amcl = { x: p.position.x, y: p.position.y, yaw: yawOf(p.orientation) };
      loc.odomAtAmcl = loc.odom;
      seen(); updateRobot();
    },
    odom(msg) {          // nav_msgs/Odometry on /odometry/filtered
      const p = msg.pose.pose;
      loc.odom = { x: p.position.x, y: p.position.y, yaw: yawOf(p.orientation) };
      seen(); updateRobot();
    },
    scan(msg) { loc.scanMsg = msg; seen(); updateScanPoints(); },   // sensor_msgs/LaserScan
    map(msg) {           // nav_msgs/OccupancyGrid on /map
      const { width: w, height: h, resolution: res } = msg.info;
      const c = mapLayer.canvas;
      c.width = w; c.height = h;
      const g = c.getContext('2d'), img = g.createImageData(w, h);
      const [tr, tg, tb] = colors.text.match(/\w\w/g)?.map((v) => parseInt(v, 16)) || [20, 30, 40];
      for (let i = 0; i < w * h; i++) {
        const v = msg.data[i], o = i * 4;
        if (v > 50) { img.data[o] = tr; img.data[o + 1] = tg; img.data[o + 2] = tb; img.data[o + 3] = 200; }
        else if (v < 0) { img.data[o] = 128; img.data[o + 1] = 140; img.data[o + 2] = 150; img.data[o + 3] = 40; }
      }
      g.putImageData(img, 0, 0);
      const [fx, fy] = toField(msg.info.origin.position.x, msg.info.origin.position.y);
      mapLayer.info = { res, fx, fy };
      seen(); drawFloor();
    },
    path(msg) {          // nav_msgs/Path on /plan
      model.path = (msg.poses || []).map((s) => toField(s.pose.position.x, s.pose.position.y));
      seen();
    },
    machines(msg) {      // refbox_msgs/MachineInfoData on /refbox/machine_info
      model.stations = (msg.machines || []).filter((m) => /^C_Z[1-5][1-5]$/.test(m.zone)).map((m) => {
        const [x, y] = zoneCenter(m.zone);
        return { name: m.name, zone: m.zone, x, y, rotation: m.rotation };
      });
      seen();
    },
    challenge(msg) {     // std_msgs/String JSON on /challenge/navigation/state
      try { model.challenge = JSON.parse(msg.data); } catch (e) { /* ignore */ }
      seen();
    },
  };

  /* ------------------------------------------------ demo robot (no ROS) */
  let demoOn = false;
  function startDemo() {
    demoOn = true;
    model.live = false;
    model.robotKnown = true;
    model.frame = 'map';
    for (const [name, zone, rotation] of [['C-BS', 'C_Z24', 90], ['C-CS1', 'C_Z43', 0], ['C-RS1', 'C_Z15', 135]]) {
      const [x, y] = zoneCenter(zone);
      model.stations.push({ name, zone, x, y, rotation });
    }
  }
  function demoStep(dt) {
    const r = model.robot;
    if (model.goal) {
      const dx = model.goal[0] - r.x, dy = model.goal[1] - r.y, d = Math.hypot(dx, dy);
      if (d > 0.02) { const v = Math.min(0.25, d * 1.5) * dt; r.x += dx / d * v; r.y += dy / d * v; }
    }
    const pts = [];
    for (let i = 0; i < 360; i++) {                 // walls only: the LiDAR plane is above the stations
      const a = i * Math.PI / 180, cx = Math.cos(a), cy = Math.sin(a);
      let t = Infinity;
      if (cx > 1e-9) t = Math.min(t, (SIZE - r.x) / cx); else if (cx < -1e-9) t = Math.min(t, -r.x / cx);
      if (cy > 1e-9) t = Math.min(t, (SIZE - r.y) / cy); else if (cy < -1e-9) t = Math.min(t, -r.y / cy);
      pts.push([r.x + t * cx, r.y + t * cy]);
    }
    model.scan = pts;
  }

  /* ------------------------------------------------ loop (only while visible) */
  let running = false, last = 0, lastHud = 0;
  function frame(now) {
    if (!running) return;
    const dt = Math.min(0.1, (now - last) / 1000);
    last = now;
    if (demoOn) demoStep(dt);
    if (model.goal) {                               // goal reached: clear it after 3 s inside 12 cm
      const near = Math.hypot(model.goal[0] - model.robot.x, model.goal[1] - model.robot.y) < 0.12;
      if (near && !arrivedSince) arrivedSince = now;
      if (!near) arrivedSince = null;
      if (arrivedSince && now - arrivedSince > 3000) { model.goal = null; model.path = []; model.selected = null; arrivedSince = null; drawFloor(); }
    }
    if (now - lastHud > 150) { updateHud(now); lastHud = now; }
    if (mode === '3d' && view3d) view3d.draw(); else drawTop();
    requestAnimationFrame(frame);
  }

  drawFloor();
  fitTop();
  return {
    ros, model, startDemo,
    setLive(on) { model.live = on; },
    show() { if (!running) { running = true; fitTop(); last = performance.now(); requestAnimationFrame(frame); } },
    hide() { running = false; },
  };
}
