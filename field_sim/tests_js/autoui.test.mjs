import test from 'node:test';
import assert from 'node:assert/strict';
import { createAutoPlanner } from '../web/js/autoui.js';
import { newAuto, addNode, toJSON } from '../web/js/autos.js';

// ---- tiny fake DOM ----
class FEl {
  constructor(tag, doc) { this.tagName = tag.toUpperCase(); this.ownerDocument = doc; this.children = []; this.parentNode = null; this.attrs = {}; this.style = {}; this.listeners = {}; this._text = ''; this.value = ''; this.className = ''; }
  get firstChild() { return this.children[0] || null; }
  appendChild(c) { if (c.parentNode) c.parentNode.removeChild(c); c.parentNode = this; this.children.push(c); return c; }
  removeChild(c) { const i = this.children.indexOf(c); if (i >= 0) this.children.splice(i, 1); c.parentNode = null; return c; }
  setAttribute(k, v) { this.attrs[k] = String(v); } getAttribute(k) { return this.attrs[k] ?? null; } removeAttribute(k) { delete this.attrs[k]; }
  addEventListener(t, f) { (this.listeners[t] ||= []).push(f); }
  removeEventListener(t, f) { this.listeners[t] = (this.listeners[t] || []).filter(x => x !== f); }
  fire(t, ev = {}) { const e = { type: t, target: this, preventDefault() {}, stopPropagation() {}, ...ev }; for (const f of [...(this.listeners[t] || [])]) f(e); }
  set textContent(v) { this.children = []; this._text = String(v); }
  get textContent() { return this._text + this.children.map(c => c.textContent).join(''); }
  getBoundingClientRect() { return { left: 0, top: 0, width: 100, height: 20 }; }
  click() { this.fire('click'); }
  all(pred, out = []) { if (pred(this)) out.push(this); for (const c of this.children) c.all(pred, out); return out; }
}
function makeDoc() {
  const doc = new FEl('#doc', null); doc.ownerDocument = doc;
  doc.createElement = t => new FEl(t, doc); doc.head = new FEl('head', doc); doc.body = new FEl('body', doc);
  doc.getElementById = () => null;
  return doc;
}
const q = (root, attr, val) => root.all(e => e.attrs[attr] !== undefined && (val === undefined || e.attrs[attr] === val));
function setup(over = {}) {
  const doc = makeDoc(), leftEl = doc.createElement('div'), rightEl = doc.createElement('div'), timelineEl = doc.createElement('div');
  const toasts = [], changes = [];
  const paths = over.paths ?? { p1: { name: 'P1', segments: [{ type: 'line', points: [{ x: 24, y: 24 }, { x: 24, y: 84 }] }] } };
  const p = createAutoPlanner({ leftEl, rightEl, timelineEl, getPaths: () => paths, getHost: () => over.host || null, toast: m => toasts.push(m), onChange: a => changes.push(a), exportFn: (a, s) => `// ${s} ${a.name}`, ...over.opts });
  return { doc, leftEl, rightEl, timelineEl, p, toasts, changes, paths };
}
const chip = (ctx, kind) => q(ctx.leftEl, 'data-palette', kind)[0];
const rows = ctx => ctx.rightEl.all(e => e.className === 'ap-node');

test('renders palette chips for every command group', () => {
  const c = setup();
  for (const k of ['follow', 'intakeOn', 'shoot', 'shootAll', 'aimOn', 'flowerDeploy', 'gateOpen', 'wait', 'waitUntil', 'sequential', 'parallel', 'race', 'deadline', 'park']) assert.ok(chip(c, k), k);
  assert.equal(rows(c).length, 1, 'only the root row initially');
  assert.equal(q(c.leftEl, 'data-path').length, 1);
});
test('palette click inserts into root; tree row count grows; onChange fires', () => {
  const c = setup(); chip(c, 'intakeOn').fire('click'); chip(c, 'wait').fire('click');
  assert.equal(c.p.auto.root.children.length, 2); assert.equal(rows(c).length, 3);
  assert.equal(c.p.auto.root.children[0].name, 'intakeOn'); assert.ok(c.changes.length >= 2);
});
test('insert into selected group, and after a selected leaf', () => {
  const c = setup(); chip(c, 'parallel').fire('click'); const g = c.p.auto.root.children[0];
  assert.equal(c.p.selected, g.id); chip(c, 'intakeOn').fire('click');
  assert.equal(g.children.length, 1);
  chip(c, 'wait').fire('click'); assert.equal(g.children.length, 2, 'inserted after selected leaf inside the same group');
});
test('drop from palette onto group appends, drop node moves it', () => {
  const c = setup(); c.p.insertFromPalette('sequential'); const g = c.p.auto.root.children[0];
  c.p.select(null); c.p.insertFromPalette('wait', c.p.auto.root.id);
  const w = c.p.auto.root.children[1];
  assert.ok(c.p.dropAt('palette:intakeOff', g.id, 'into')); assert.equal(g.children.length, 1);
  assert.ok(c.p.dropAt('node:' + w.id, g.id, 'into')); assert.equal(g.children.length, 2); assert.equal(c.p.auto.root.children.length, 1);
  // DOM drop event path
  const row = q(c.rightEl, 'data-node', g.id)[0];
  row.fire('drop', { dataTransfer: { getData: () => 'palette:park' }, clientY: 10 });
  assert.equal(g.children.length, 3);
});
test('inspector edits propagate (wait seconds, command arg, condition, timeout)', () => {
  const c = setup(); c.p.insertFromPalette('wait'); const w = c.p.auto.root.children[0];
  const inp = q(c.rightEl, 'data-field', 'seconds')[0]; inp.value = '2.5'; inp.fire('change');
  assert.equal(w.seconds, 2.5);
  c.p.insertFromPalette('shoot'); const s = c.p.auto.root.children[1];
  const cnt = q(c.rightEl, 'data-field', 'arg.count')[0]; cnt.value = '3'; cnt.fire('change'); assert.equal(s.args.count, 3);
  c.p.insertFromPalette('waitUntil'); const u = c.p.auto.root.children[2];
  const k = q(c.rightEl, 'data-field', 'cond-kind')[0]; k.value = 'hopperFull'; k.fire('change'); assert.equal(u.condition, 'hopperFull');
  const kk = q(c.rightEl, 'data-field', 'cond-kind')[0]; kk.value = 'ballCount'; kk.fire('change');
  const n = q(c.rightEl, 'data-field', 'cond-n')[0]; n.value = '2'; n.fire('change');
  const op = q(c.rightEl, 'data-field', 'cond-op')[0]; op.value = '<='; op.fire('change'); assert.equal(u.condition, 'ballCount<=2');
  const to = q(c.rightEl, 'data-field', 'timeout')[0]; to.value = '4'; to.fire('change'); assert.equal(u.timeout, 4);
});
test('follow inspector: path picker, markers add/edit/delete', () => {
  const c = setup(); c.p.insertFromPalette('follow'); const f = c.p.auto.root.children[0];
  assert.equal(f.path, 'p1'); assert.ok(c.p.auto.paths.p1, 'host path imported');
  q(c.rightEl, 'data-act', 'marker-add')[0].fire('click'); assert.equal(c.p.auto.paths.p1.markers.length, 1);
  const t = q(c.rightEl, 'data-field', 'marker-t')[0]; t.value = '0.25'; t.fire('change'); assert.equal(c.p.auto.paths.p1.markers[0].t, 0.25);
  const mc = q(c.rightEl, 'data-field', 'marker-cmd')[0]; mc.value = 'park'; mc.fire('change'); assert.equal(c.p.auto.paths.p1.markers[0].command.name, 'park');
  q(c.rightEl, 'data-act', 'marker-delete')[0].fire('click'); assert.equal(c.p.auto.paths.p1.markers.length, 0);
});
test('undo / redo and keyboard shortcuts', () => {
  const c = setup(); c.p.insertFromPalette('wait'); c.p.insertFromPalette('park');
  assert.equal(c.p.auto.root.children.length, 2);
  c.p.undo(); assert.equal(c.p.auto.root.children.length, 1);
  c.p.redo(); assert.equal(c.p.auto.root.children.length, 2);
  c.p.focus(); c.doc.fire('keydown', { key: 'z', ctrlKey: true }); assert.equal(c.p.auto.root.children.length, 1);
  c.doc.fire('keydown', { key: 'y', ctrlKey: true }); assert.equal(c.p.auto.root.children.length, 2);
  c.p.select(c.p.auto.root.children[1].id); c.doc.fire('keydown', { key: 'Delete' }); assert.equal(c.p.auto.root.children.length, 1);
  c.p.undo(); assert.equal(c.p.auto.root.children.length, 2);
});
test('Delete ignored while typing in an input', () => {
  const c = setup(); c.p.insertFromPalette('wait'); c.p.focus();
  c.doc.fire('keydown', { key: 'Delete', target: { tagName: 'INPUT' } }); assert.equal(c.p.auto.root.children.length, 1);
});
test('tree buttons: up/down/indent/delete/wrap', () => {
  const c = setup(); c.p.insertFromPalette('sequential'); c.p.select(null); c.p.insertFromPalette('wait', c.p.auto.root.id); c.p.insertFromPalette('park', c.p.auto.root.id);
  const [g, w, pk] = c.p.auto.root.children;
  q(c.rightEl, 'data-node', pk.id)[0].all(e => e.attrs['data-act'] === 'up')[0].fire('click');
  assert.equal(c.p.auto.root.children[1].id, pk.id);
  q(c.rightEl, 'data-node', pk.id)[0].all(e => e.attrs['data-act'] === 'up')[0].fire('click');
  assert.equal(c.p.auto.root.children[0].id, pk.id);
  q(c.rightEl, 'data-node', w.id)[0].all(e => e.attrs['data-act'] === 'down')[0].fire('click'); // w is last: disabled-by-index no-op
  const order = c.p.auto.root.children.map(x => x.id); assert.deepEqual(order, [pk.id, g.id, w.id]);
  c.p.select(w.id); q(c.rightEl, 'data-act', 'indent')[0].fire('click');          // indent / wrap live in the inspector
  assert.equal(g.children[0].id, w.id);
  c.p.select(pk.id); const wrap = q(c.rightEl, 'data-act', 'wrap')[0]; wrap.value = 'race'; wrap.fire('change');
  assert.equal(c.p.auto.root.children[0].type, 'race'); assert.equal(c.p.auto.root.children[0].children[0].id, pk.id);
  q(c.rightEl, 'data-node', w.id)[0].all(e => e.attrs['data-act'] === 'delete')[0].fire('click'); assert.equal(g.children.length, 0);
});
test('collapse hides child rows', () => {
  const c = setup(); c.p.insertFromPalette('sequential'); const g = c.p.auto.root.children[0]; c.p.insertFromPalette('wait', g.id);
  assert.equal(rows(c).length, 3);
  q(c.rightEl, 'data-node', g.id)[0].all(e => e.attrs['data-act'] === 'toggle')[0].fire('click'); assert.equal(rows(c).length, 2);
});
test('validation list shows issues and clicking selects the node', () => {
  const c = setup(); c.p.insertFromPalette('sequential'); const g = c.p.auto.root.children[0];
  const iss = q(c.rightEl, 'data-act', 'issue'); assert.ok(iss.length >= 1);
  assert.ok(iss.some(i => /Empty/.test(i.textContent)));
  c.p.select(null); q(c.rightEl, 'data-act', 'issue').find(i => i.attrs['data-node'] === g.id).fire('click');
  assert.equal(c.p.selected, g.id);
});
test('no crash when getPaths returns empty; follow gets empty path and an error issue', () => {
  const c = setup({ paths: {} });
  assert.equal(q(c.leftEl, 'data-role', 'no-paths').length, 1);
  c.p.insertFromPalette('follow'); assert.equal(c.p.auto.root.children[0].path, '');
  assert.ok(q(c.rightEl, 'data-act', 'issue').some(i => i.attrs['data-level'] === 'error'));
  const bad = setup({ opts: { getPaths: () => { throw new Error('x'); } } }); assert.ok(bad.p.auto);
  const nul = setup({ opts: { getPaths: () => null } }); assert.ok(nul.p.auto);
});
test('alliance, mirror, load/save round trip', () => {
  const c = setup(); c.p.setAlliance('red'); assert.equal(c.p.auto.alliance, 'red');
  c.p.auto.start.x = 20; c.p.mirror(); assert.equal(c.p.auto.alliance, 'blue'); assert.equal(c.p.auto.start.x, 124);
  const a = newAuto({ name: 'Zed' }); addNode(a, null, { type: 'wait', seconds: 2 });
  c.p.load(a); assert.equal(c.p.auto.name, 'Zed'); assert.equal(rows(c).length, 2);
  const json = c.p.save(); assert.equal(JSON.parse(json).name, 'Zed');
  c.p.newAuto(); assert.equal(c.p.auto.root.children.length, 0);
  assert.ok(c.p.loadJSON(json)); assert.equal(c.p.auto.root.children.length, 1);
  assert.equal(c.p.loadJSON('{nope'), false); assert.ok(c.toasts.some(t => /Load failed/.test(t)));
});
test('timeline shows bars, total, 30s warning in red', () => {
  const c = setup(); c.p.insertFromPalette('wait');
  c.p.auto.root.children[0].seconds = 40; c.p.render();
  assert.equal(q(c.timelineEl, 'data-bar').length >= 1, true);
  const tot = q(c.timelineEl, 'data-role', 'total')[0]; assert.equal(tot.attrs['data-over'], '1'); assert.match(tot.textContent, /40\.\d s \/ 30 s/);
  assert.equal(q(c.timelineEl, 'data-role', 'warn').length, 1);
  c.p.auto.root.children[0].seconds = 3; c.p.render(); assert.equal(q(c.timelineEl, 'data-role', 'total')[0].attrs['data-over'], '0');
});
test('export calls exportFn and is blocked on errors', () => {
  const c = setup(); c.p.insertFromPalette('wait');
  assert.equal(c.p.exportJava('ivy'), '// ivy MyAuto');
  c.p.insertFromPalette('waitUntil'); c.p.auto.root.children[1].condition = 'bogus';
  assert.equal(c.p.exportJava('statemachine'), null);
});
test('run in sim with a fake host, tick, pause, scrub via timeline', async () => {
  const host = { pose: {}, vel: [0, 0, 0], setPose(p) { this.pose = { ...p }; }, setVelocity() {}, ballCount: () => 0, park() {} };
  const c = setup({ host });
  c.p.insertFromPalette('follow'); c.p.insertFromPalette('park');
  c.p.auto.start = { x: 24, y: 24, h: 0 };
  assert.equal(await c.p.run(), true); assert.equal(c.p.runner.state, 'running');
  for (let i = 0; i < 30; i++) c.p.tick(1 / 60);
  assert.ok(host.pose.y > 24); assert.ok(q(c.timelineEl, 'data-role', 'time')[0].textContent.includes('0.50'));
  assert.ok(q(c.rightEl, 'data-live', '1').length >= 1, 'live node highlighted');
  q(c.timelineEl, 'data-act', 'pause')[0].fire('click'); assert.equal(c.p.runner.state, 'paused');
  await c.p.seek(1.0); assert.ok(Math.abs(c.p.runner.time - 1.0) < 1e-6);
  q(c.timelineEl, 'data-act', 'stop')[0].fire('click'); assert.equal(c.p.runner.state, 'idle');
  const none = setup(); assert.equal(await none.p.run(), false); assert.ok(none.toasts.some(t => /not ready/.test(t)));
});

function memStore() { const m = new Map(); return { getItem: k => (m.has(k) ? m.get(k) : null), setItem: (k, v) => m.set(k, String(v)) }; }
test('auto library card: save, list, load, copy and delete via buttons', () => {
  const st = memStore(), c = setup({ opts: { storage: st } });
  c.p.auto.name = 'Left9'; c.p.insertFromPalette('wait');
  q(c.leftEl, 'data-act', 'lib-save')[0].fire('click');
  assert.equal(q(c.leftEl, 'data-lib').length, 1);
  q(c.leftEl, 'data-act', 'lib-dup')[0].fire('click'); assert.equal(q(c.leftEl, 'data-lib').length, 2);
  c.p.newAuto(); assert.equal(c.p.auto.root.children.length, 0);
  q(c.leftEl, 'data-lib', 'Left9')[0].all(e => e.attrs['data-act'] === 'lib-load')[0].fire('click');
  assert.equal(c.p.auto.root.children.length, 1);
  q(c.leftEl, 'data-act', 'lib-del')[0].fire('click'); assert.equal(q(c.leftEl, 'data-lib').length, 1);
  assert.equal(c.p.importLibrary(JSON.stringify({ library: 1, autos: [JSON.parse(c.p.save())] })), 1);
});
test('toolbar buttons carry tooltips and icons; preview starts paused', async () => {
  const c = setup({ host: { getPose: () => ({ x: 0, y: 0, h: 0 }), setPose() {}, setVelocity() {} } });
  for (const act of ['run', 'preview', 'pause', 'stop', 'tl-back', 'tl-fwd', 'undo', 'redo', 'export-statemachine']) {
    const b = (['undo', 'redo', 'export-statemachine'].includes(act) ? q(c.rightEl, 'data-act', act) : q(c.timelineEl, 'data-act', act))[0]; assert.ok(b, act); assert.ok(b.attrs.title, 'tooltip for ' + act); assert.ok(b.attrs['data-ico'], 'icon for ' + act);
  }
  c.p.insertFromPalette('wait'); await c.p.run({ paused: true }); assert.equal(c.p.runner.state, 'paused');
});
