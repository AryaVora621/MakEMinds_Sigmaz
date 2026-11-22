import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import {
  ACTIONS, ACTION_LABELS, PRESETS, KeyBindings, keyboardCommand, aimInputFromBindings, normalizeKey,
  createControlsPanel, STORAGE_KEY,
} from '../web/js/controls.js';
import { mapKeyboard } from '../web/js/driver.js';

const cmd = (keys, preset = 'classic', opts) => keyboardCommand(new Set(keys), new KeyBindings(preset), opts);

// ---- tiny fake DOM
class FakeEl {
  constructor(tag) { this.tag = tag; this.children = []; this.listeners = {}; this.className = ''; this.textContent = ''; this.value = ''; this.id = ''; }
  get firstChild() { return this.children[0] ?? null; }
  appendChild(c) { this.children.push(c); return c; }
  removeChild(c) { this.children = this.children.filter(x => x !== c); return c; }
  addEventListener(t, f) { (this.listeners[t] ||= []).push(f); }
  removeEventListener(t, f) { this.listeners[t] = (this.listeners[t] || []).filter(x => x !== f); }
  fire(t, ev = {}) { for (const f of [...(this.listeners[t] || [])]) f(ev); }
  find(pred, out = []) { if (pred(this)) out.push(this); this.children.forEach(c => c.find(pred, out)); return out; }
}
class FakeDoc extends FakeEl {
  constructor() { super('#doc'); this.head = new FakeEl('head'); this.body = new FakeEl('body'); }
  createElement(t) { return new FakeEl(t); }
  getElementById(id) { return this.head.find(e => e.id === id)[0] ?? null; }
}
const fakeStorage = () => { const m = new Map(); return { getItem: k => (m.has(k) ? m.get(k) : null), setItem: (k, v) => m.set(k, String(v)), m }; };
const ev = key => ({ key, preventDefault() {}, stopPropagation() {} });

test('classic: W forward, S backward', () => {
  assert.equal(cmd(['w']).fwd, 1);
  assert.equal(cmd(['s']).fwd, -1);
});
test('classic: A turns left (+1, ccw), D turns right (-1)', () => {
  assert.equal(cmd(['a']).turn, 1);
  assert.equal(cmd(['d']).turn, -1);
});
test('classic: Q strafes left (+1), E strafes right (-1)', () => {
  assert.equal(cmd(['q']).strafe, 1);
  assert.equal(cmd(['e']).strafe, -1);
});
test('classic preset matches driver.js mapKeyboard', () => {
  for (const keys of [['w'], ['s'], ['a'], ['d'], ['q'], ['e'], ['w', 'a'], ['s', 'd', 'q'], [' '], ['f'], ['shift']]) {
    const a = cmd(keys, 'classic'), b = mapKeyboard(new Set(keys));
    assert.equal(a.fwd, b.fwd); assert.equal(a.strafe, b.strafe); assert.equal(a.turn, b.turn);
    assert.equal(a.shoot, b.shoot); assert.equal(a.fieldCentricToggle, b.fieldCentricToggle); assert.equal(a.aux.lb, b.aux.lb);
  }
});
test('wasd_strafe signs: A strafe +1, D strafe -1, Q turn +1, E turn -1', () => {
  assert.equal(cmd(['a'], 'wasd_strafe').strafe, 1);
  assert.equal(cmd(['d'], 'wasd_strafe').strafe, -1);
  assert.equal(cmd(['q'], 'wasd_strafe').turn, 1);
  assert.equal(cmd(['e'], 'wasd_strafe').turn, -1);
});
test('literal: Q/E strafe, W/S turn, arrows drive', () => {
  assert.equal(cmd(['q'], 'literal').strafe, 1);
  assert.equal(cmd(['e'], 'literal').strafe, -1);
  assert.equal(cmd(['w'], 'literal').turn, 1);
  assert.equal(cmd(['s'], 'literal').turn, -1);
  assert.equal(cmd(['arrowup'], 'literal').fwd, 1);
  assert.equal(cmd(['arrowdown'], 'literal').fwd, -1);
  assert.equal(cmd(['w'], 'literal').fwd, 0);
});
test('literal moves aim keys off the arrows (J/K turret, U/N hood)', () => {
  const b = new KeyBindings('literal');
  assert.equal(aimInputFromBindings(new Set(['arrowup']), b).hoodAxis, 0);
  assert.equal(aimInputFromBindings(new Set(['j']), b).turretAxis, 1);
  assert.equal(aimInputFromBindings(new Set(['k']), b).turretAxis, -1);
  assert.equal(aimInputFromBindings(new Set(['u']), b).hoodAxis, 1);
  assert.equal(aimInputFromBindings(new Set(['n']), b).hoodAxis, -1);
  assert.deepEqual(b.conflicts(), []);
});
test('classic aim keys behave like aimInputFromKeys', () => {
  const b = new KeyBindings();
  const r = aimInputFromBindings(new Set(['arrowleft', ']', 'z', 'shift']), b);
  assert.equal(r.turretAxis, 1); assert.equal(r.rpmAxis, 1); assert.equal(r.rpmPreset, 'stop'); assert.equal(r.fine, true);
});
test('diagonals are normalised to magnitude 1', () => {
  const c = cmd(['w', 'q']);
  assert.ok(Math.abs(Math.hypot(c.fwd, c.strafe) - 1) < 1e-12);
  assert.ok(c.fwd > 0 && c.strafe > 0);
});
test('opposite keys cancel', () => {
  const c = cmd(['w', 's', 'q', 'e', 'a', 'd']);
  assert.equal(c.fwd, 0); assert.equal(c.strafe, 0); assert.equal(c.turn, 0);
});
test('output shape matches driver.js and carries shoot/field/fast/intake', () => {
  const c = cmd([' ', 'f', 'shift', 'i', 'r']);
  assert.equal(c.connected, true); assert.equal(c.shoot, true); assert.equal(c.fieldCentricToggle, true);
  assert.equal(c.aux.lb, true); assert.equal(c.intakeToggle, true); assert.equal(c.resetRobot, true);
  assert.deepEqual(Object.keys(c.aux), Object.keys(mapKeyboard(new Set()).aux));
});
test('fast and slow scales apply to translation', () => {
  const b = new KeyBindings(); b.rebind('slow', 'v');
  assert.equal(keyboardCommand(new Set(['w', 'v']), b, { slowScale: 0.5 }).fwd, 0.5);
  assert.equal(keyboardCommand(new Set(['w', 'shift']), b, { fastScale: 2 }).fwd, 2);
});
test('rebind steals the key from the old action', () => {
  const b = new KeyBindings();
  const r = b.rebind('shoot', 'w');
  assert.equal(r.ok, true); assert.deepEqual(r.stolenFrom, ['forward']);
  assert.deepEqual(b.bindingsFor('forward'), []);
  assert.deepEqual(b.bindingsFor('shoot'), ['w']);
  assert.equal(b.actionForKey('W'), 'shoot');
});
test('reserved keys are rejected', () => {
  const b = new KeyBindings();
  const r = b.rebind('forward', 'Escape');
  assert.equal(r.ok, false);
  assert.deepEqual(b.bindingsFor('forward'), ['w']);
  assert.equal(b.rebind('forward', 'Tab').ok, false);
});
test('conflicts: duplicates and host keys; shift sharing is allowed', () => {
  const b = new KeyBindings();
  assert.deepEqual(b.conflicts(), []);
  b.map.shoot.push('w');
  const c = b.conflicts();
  assert.ok(c.some(x => x.key === 'w' && x.reason === 'duplicate' && x.actions.includes('forward') && x.actions.includes('shoot')));
  const b2 = new KeyBindings(); b2.rebind('intakeToggle', 'm');
  assert.ok(b2.conflicts().some(x => x.key === 'm' && x.reason === 'host'));
});
test('JSON round trip keeps preset and overrides', () => {
  const b = new KeyBindings('literal'); b.rebind('shoot', 'g');
  const j = JSON.parse(JSON.stringify(b));
  assert.equal(j.preset, 'literal'); assert.deepEqual(Object.keys(j.overrides), ['shoot']);
  const b2 = KeyBindings.fromJSON(j);
  for (const a of ACTIONS) assert.deepEqual(b2.bindingsFor(a), b.bindingsFor(a));
});
test('fromJSON rejects bad data', () => {
  assert.throws(() => KeyBindings.fromJSON(null));
  assert.throws(() => KeyBindings.fromJSON('{not json'));
  assert.throws(() => KeyBindings.fromJSON({ version: 1, preset: 'nope' }));
  assert.throws(() => KeyBindings.fromJSON({ version: 2, preset: 'classic' }));
  assert.throws(() => KeyBindings.fromJSON({ version: 1, preset: 'classic', overrides: { bogus: ['a'] } }));
  assert.throws(() => KeyBindings.fromJSON({ version: 1, preset: 'classic', overrides: { shoot: [5] } }));
  assert.throws(() => KeyBindings.fromJSON({ version: 1, preset: 'classic', overrides: { shoot: ['escape'] } }));
});
test('storage save/load with fake storage, bad stored data ignored', () => {
  const st = fakeStorage();
  const b = new KeyBindings('wasd_strafe'); b.rebind('shoot', 'g');
  assert.equal(b.save(st), true);
  assert.ok(st.m.has(STORAGE_KEY));
  const b2 = new KeyBindings(); assert.equal(b2.load(st), true);
  assert.equal(b2.presetId, 'wasd_strafe'); assert.deepEqual(b2.bindingsFor('shoot'), ['g']);
  const bad = fakeStorage(); bad.setItem(STORAGE_KEY, '{oops');
  const b3 = new KeyBindings(); assert.equal(b3.load(bad), false); assert.equal(b3.presetId, 'classic');
  assert.equal(new KeyBindings().load(fakeStorage()), false);
});
test('key name normalisation', () => {
  assert.equal(normalizeKey('W'), 'w');
  assert.equal(normalizeKey('Arrow Up'), 'arrowup');
  assert.equal(normalizeKey('ArrowUp'), 'arrowup');
  assert.equal(normalizeKey('up'), 'arrowup');
  assert.equal(normalizeKey('Space'), ' ');
  assert.equal(normalizeKey(' '), ' ');
  assert.equal(normalizeKey('Esc'), 'escape');
  assert.equal(new KeyBindings('literal').actionForKey('Arrow Down'), 'backward');
  assert.equal(cmd(['W', 'Q']).fwd > 0, true);
});
test('unknown preset throws; presets have labels and every action', () => {
  assert.throws(() => new KeyBindings('nope'));
  assert.throws(() => new KeyBindings().setPreset('nope'));
  for (const id of Object.keys(PRESETS)) {
    assert.ok(PRESETS[id].label && PRESETS[id].description);
    for (const a of ACTIONS) assert.ok(Array.isArray(PRESETS[id].bindings[a]), id + ' ' + a);
  }
  for (const a of ACTIONS) assert.ok(ACTION_LABELS[a]);
});
test('panel builds, injects style once, preset change and rebind work', () => {
  const doc = new FakeDoc(); const b = new KeyBindings(); let changes = 0;
  const p = createControlsPanel({ bindings: b, doc, onChange: () => changes++ });
  createControlsPanel({ bindings: b, doc });
  assert.equal(doc.head.find(e => e.tag === 'style').length, 1);
  assert.equal(p.el.find(e => e.tag === 'tr').length, ACTIONS.length);
  const sel = p.el.find(e => e.tag === 'select')[0];
  sel.value = 'literal'; sel.fire('change');
  assert.equal(b.presetId, 'literal'); assert.equal(changes, 1);
  const btns = () => p.el.find(e => e.tag === 'button' && e.textContent !== 'Reset');
  btns()[ACTIONS.indexOf('shoot')].fire('click');
  doc.fire('keydown', ev('Escape'));
  assert.deepEqual(b.bindingsFor('shoot'), [' ']);
  btns()[ACTIONS.indexOf('shoot')].fire('click');
  doc.fire('keydown', ev('G'));
  assert.deepEqual(b.bindingsFor('shoot'), ['g']); assert.equal(changes, 2);
  assert.equal((doc.listeners.keydown || []).length, 0);
  const reset = p.el.find(e => e.tag === 'button' && e.textContent === 'Reset')[0];
  reset.fire('click');
  assert.deepEqual(b.bindingsFor('shoot'), [' ']);
});
test('panel shows conflicts warning', () => {
  const doc = new FakeDoc(); const b = new KeyBindings();
  const p = createControlsPanel({ bindings: b, doc });
  assert.equal(p.el.find(e => e.tag === 'li').length, 0);
  b.rebind('shoot', 't'); p.refresh();
  assert.equal(p.el.find(e => e.tag === 'li').length, 1);
});
test('panel requires bindings and doc', () => {
  assert.throws(() => createControlsPanel({}));
});

// ---- source-text checks for the planner flip, kind toggle, hive HUD and 3D hotkey gate
const WEB = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..', 'web');
const appSrc = fs.readFileSync(path.join(WEB, 'js', 'app.js'), 'utf8');
const indexHtml = fs.readFileSync(path.join(WEB, 'index.html'), 'utf8');

test('app.js wires the planner flip (planFlip + plan-flip id)', () => {
  assert.match(appSrc, /function planFlip\(/);
  assert.match(appSrc, /\$\('plan-flip'\)/);
  assert.match(appSrc, /'plan-flip'|"plan-flip"|#plan-flip/);
});

test('index.html has the kind toggle and hive HUD ids', () => {
  for (const id of ['kind-pollen', 'kind-nectar', 'hud-hive-red', 'hud-hive-blue']) {
    assert.match(indexHtml, new RegExp('id="' + id + '"'), id);
  }
});

test('app.js does not hard-code ballsToTip to 4', () => {
  assert.doesNotMatch(appSrc, /ballsToTip: 4/);
  assert.doesNotMatch(appSrc, /ballsToTip = 4/);
});

test('hotkey handler still gates on the 3D view', () => {
  assert.ok(appSrc.includes("if (S.view !== '3d' || e.ctrlKey || e.metaKey || e.altKey) return;"));
});

test('field-to-screen mx/my go through the flip-aware view transform', () => {
  // mx/my read viewXf(), and viewXf() reads planFlip() for the rotate180 flag.
  assert.match(appSrc, /function mx\(x\) \{[^}]*viewXf\(\)/);
  assert.match(appSrc, /function my\(y\) \{[^}]*viewXf\(\)/);
  assert.match(appSrc, /function viewXf\(\) \{[^}]*rotate180: planFlip\(\)/);
});
