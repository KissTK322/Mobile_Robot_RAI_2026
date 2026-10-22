/* Phone/tablet practice refbox. No ROS velocity, goal or initial-pose publishing here. */
'use strict';
const PRACTICE = { info: null, layout: null, dirty: false, pending: false, historyKey: '', initialized: false };
const rb = (id) => document.getElementById('refbox-' + id);
const rbError = (e) => { rb('message').textContent = String(e.message || e).replace(/^http \d+: /, ''); };
function rbSettings() {
  const out = { n: Number(rb('boxes').value), limit_s: Number(rb('duration').value), countdown_s: Number(rb('countdown').value) };
  if (rb('seed').value !== '') out.seed = Number(rb('seed').value);
  return out;
}
async function rbAction(action, body) {
  if (PRACTICE.pending) return;
  PRACTICE.pending = true; renderRefbox(); rb('message').textContent = 'Working…';
  try {
    PRACTICE.info = await api('practice/' + action, { method: 'POST', body });
    if (action === 'source') rb('confirm').checked = false;
    rb('message').textContent = action === 'source' ? 'Switching connection…' : PRACTICE.info.message;
    renderRefbox(); renderGame();
  } catch (e) { rbError(e); }
  finally { PRACTICE.pending = false; renderRefbox(); }
}
function rbDirty() {
  PRACTICE.dirty = true; rb('confirm').checked = false; renderRefbox();
}
function rbOptions(select, values, value) {
  select.replaceChildren(...values.map(v => { const o=document.createElement('option'); o.value=String(v); o.textContent=String(v); return o; }));
  if (values.map(String).includes(String(value))) select.value=String(value);
}
function rbMachineRows(config) {
  const rules=PRACTICE.info.placement_rules || {};
  rb('layout').replaceChildren(...config.machines.map(m => {
    const row=document.createElement('div'); row.className='refbox-machine'; row.dataset.machine=m.name;
    const name=document.createElement('strong'); name.textContent=m.name;
    const zone=document.createElement('select'); zone.setAttribute('aria-label',m.name+' zone');
    const angle=document.createElement('select'); angle.setAttribute('aria-label',m.name+' rotation');
    rbOptions(zone, Object.keys(rules[m.name] || {}), m.zone);
    rbOptions(angle, (rules[m.name] || {})[zone.value] || [], m.rotation);
    zone.onchange=()=>{ rbOptions(angle,(rules[m.name] || {})[zone.value] || [],angle.value); rbDirty(); };
    angle.onchange=rbDirty;
    row.append(name,zone,angle); return row;
  }));
}
function renderRefbox() {
  const d=PRACTICE.info;
  if (!d) return;
  if (!PRACTICE.initialized) { rb('source').value=d.source; PRACTICE.initialized=true; }
  const active=['countdown','running'].includes(d.state), busy=PRACTICE.pending || d.busy;
  rb('connection').textContent = (d.fake ? 'PC demo · no robot motion · ' : '') + `${d.source === 'practice' ? 'Practice' : 'Field refbox'} · ${d.busy ? 'connecting…' : d.source === 'practice' ? d.broker_connected ? 'broker connected' : 'broker offline' : 'external connection via Launch'}` + (d.error ? ' · '+d.error : '');
  if (d.layout_id !== PRACTICE.layout) {
    PRACTICE.layout=d.layout_id; PRACTICE.dirty=false; rb('confirm').checked=false;
    if (d.config) {
      rb('boxes').value=String(d.config.n); rb('duration').value=String(d.config.limit_s);
      rb('countdown').value=String(d.config.countdown_s); rb('seed').value=String(d.config.seed);
      rbMachineRows(d.config);
    }
  }
  rb('route').textContent=d.config ? 'Targets: '+d.config.route.join(' · ') : 'Generate a field to see box positions and targets.';
  rb('readiness').textContent=d.source !== 'practice' ? 'Select Practice and connect to run a practice game. For the field refbox, start Challenge: Navigation on Launch.'
    : PRACTICE.dirty ? 'Field settings changed. Save the field, or generate a new field after changing the box count.'
    : d.ready_reasons.length ? d.ready_reasons.join(' · ')
    : d.state === 'prepared' ? 'Ready. Check the physical box positions and start pose, then press Start.' : d.message;
  rb('save').hidden=!PRACTICE.dirty;
  ['connect','generate','load','save'].forEach(k=>{rb(k).disabled=busy || active;});
  ['source','boxes','duration','countdown','seed','saved'].forEach(k=>{rb(k).disabled=busy || active;});
  rb('layout').querySelectorAll('select').forEach(el=>{el.disabled=busy || active;});
  rb('confirm').disabled=active || busy;
  rb('start').textContent=d.state==='countdown' ? `Starting in ${d.countdown_left}…` : d.state==='running' ? 'Game running' : 'Start game — robot will move';
  rb('start').disabled=busy || active || PRACTICE.dirty || d.source!=='practice' || d.state!=='prepared' || d.ready_reasons.length>0 || !rb('confirm').checked;
  // Stop stays usable while a start request is in flight.
  rb('stop').disabled=d.source!=='practice' || (!active && !PRACTICE.pending);
  const savedKey=JSON.stringify(d.layouts.map(l=>l.id));
  if (rb('saved').dataset.key!==savedKey) {
    const previous=rb('saved').value;
    rb('saved').replaceChildren(new Option('Saved layouts…',''),...d.layouts.map(l=>new Option(`${l.n} boxes · seed ${l.seed} · ${l.id.slice(0,6)}`,l.id)));
    rb('saved').value=previous; rb('saved').dataset.key=savedKey;
  }
  const historyKey=JSON.stringify(d.history);
  if (historyKey!==PRACTICE.historyKey) {
    PRACTICE.historyKey=historyKey;
    rb('history').replaceChildren(...d.history.map(run=>{
      const p=document.createElement('p'); p.className='small';
      p.textContent=`${new Date(run.saved_at*1000).toLocaleString()} · ${run.config.n} boxes · ${run.state} · ${run.counted}/12 · ${run.elapsed_s}s · ${run.reason}`;return p;
    }));
    if (!d.history.length) rb('history').textContent='No practice games yet.';
  }
}
rb('connect').onclick=()=>rbAction('source',{source:rb('source').value});
rb('generate').onclick=()=>rbAction('prepare',rbSettings());
rb('load').onclick=()=>{if (!rb('saved').value) {rb('message').textContent='Select a saved layout first.';return;} const body=rbSettings();delete body.n;delete body.seed;rbAction('prepare',{...body,layout_id:rb('saved').value});};
rb('save').onclick=()=>{
  const config=PRACTICE.info && PRACTICE.info.config;
  if (!config) return;
  const machines=[...rb('layout').children].map(row=>({name:row.dataset.machine,zone:row.querySelectorAll('select')[0].value,rotation:Number(row.querySelectorAll('select')[1].value)}));
  rbAction('prepare',{...rbSettings(),seed:config.seed,machines,route:config.route});
};
['boxes','duration','countdown','seed'].forEach(k=>{rb(k).onchange=rbDirty;});
rb('confirm').onchange=renderRefbox;
rb('start').onclick=()=>rbAction('start',{layout_id:PRACTICE.layout,confirmed:rb('confirm').checked});
rb('stop').onclick=async()=>{
  // A separate stop request may overtake a slow Start response. The backend serializes them.
  try { PRACTICE.info=await api('practice/stop',{method:'POST',body:{}});rb('message').textContent=PRACTICE.info.message;renderRefbox();renderGame(); }
  catch(e){rbError(e);}
};
async function pollPractice() {
  try {
    if (S.tab==='game' && !PRACTICE.pending) {
      PRACTICE.info=await api('practice/status');renderRefbox();renderGame();
    }
  } catch(e) {if(S.tab==='game') rbError(e);}
  finally {setTimeout(pollPractice,1000);}
}
pollPractice();
function renderPracticeGame() {
  const d=PRACTICE.info;
  if (!d || d.source!=='practice' || !d.config) return false;
  const remaining=d.remaining || [],visited=d.config.route.filter(z=>!remaining.includes(z));
  $('#game-count').textContent=`${d.counted}/12`;
  const left=Math.max(0,Math.ceil(d.time_left_s));
  $('#game-time').textContent=`${Math.floor(left/60)}:${String(left%60).padStart(2,'0')}`;
  $('#game-duration-label').textContent=`left of ${Math.floor(d.config.limit_s/60)}:00`;
  $('#game-retries').textContent=GAME.d && Date.now()-GAME.at<3000 ? String(GAME.d.retries || 0) : '–';
  $('#game-state').textContent=`· ${d.state}`+(d.state==='countdown' ? ` ${d.countdown_left}` : '');
  $('#game-msg').textContent=d.message;
  drawGame({...(GAME.d || {}),pose:d.pose_age_s!==null && d.pose_age_s<2 ? d.pose : null,
    mps_boxes:d.config.machines.map(m=>[Number(m.zone[3])-.5,Number(m.zone[4])-.5,m.rotation])},remaining,visited);
  return true;
}
