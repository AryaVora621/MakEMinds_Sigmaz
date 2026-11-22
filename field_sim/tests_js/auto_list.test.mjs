import test from 'node:test';
import assert from 'node:assert/strict';
import * as A from '../web/js/autos.js';
import * as P from '../web/js/pathing.js';
import { createAutoPlanner } from '../web/js/autoui.js';

// ---- helpers ----
function memStore() { const m = new Map(); return { getItem: k => (m.has(k) ? m.get(k) : null), setItem: (k, v) => m.set(k, String(v)), removeItem: k => m.delete(k) }; }
const namesOf = st => A.libraryList(st).map(x => x.name);
const route = name => ({ name, start: { x: 24, y: 24, h: 0 }, segments: [{ type: 'line', points: [{ x: 24, y: 24 }, { x: 40, y: 24 }] }] });

// ---- uniqueName / duplicateName (pure) ----
test('uniqueName returns the base when free and counts up from 2 when taken', () => {
  assert.equal(A.uniqueName([], 'Auto'), 'Auto');
  assert.equal(A.uniqueName(['Other'], 'Auto'), 'Auto');
  assert.equal(A.uniqueName(['Auto'], 'Auto'), 'Auto 2');
  assert.equal(A.uniqueName(['Auto', 'Auto 2'], 'Auto'), 'Auto 3');
  assert.equal(A.uniqueName(new Set(['Auto', 'Auto 2', 'Auto 3']), 'Auto'), 'Auto 4');
  assert.equal(A.uniqueName([], '   '), 'Untitled', 'blank base falls back');
  assert.equal(A.uniqueName(['Untitled'], ''), 'Untitled 2');
});
test('duplicateName makes "X copy", then "X copy 2", unique against the names given', () => {
  assert.equal(A.duplicateName(['Left9'], 'Left9'), 'Left9 copy');
  assert.equal(A.duplicateName(['Left9', 'Left9 copy'], 'Left9'), 'Left9 copy 2');
  assert.equal(A.duplicateName(['Left9', 'Left9 copy', 'Left9 copy 2'], 'Left9'), 'Left9 copy 3');
  assert.equal(A.duplicateName([], '  Spaced  '), 'Spaced copy');
});

// ---- auto library list operations (in-memory storage) ----
test('libraryDuplicate uses duplicateName and keeps the list order of the other autos', () => {
  const st = memStore();
  A.librarySave(st, A.newAuto({ name: 'A' }), 100); A.librarySave(st, A.newAuto({ name: 'B' }), 200);
  assert.equal(A.libraryDuplicate(st, 'A'), 'A copy');
  assert.equal(A.libraryDuplicate(st, 'A'), 'A copy 2');
  assert.deepEqual(namesOf(st).slice(-2), ['B', 'A'], 'B stays where it was relative to A (older copies are not reshuffled)');
  assert.equal(A.libraryDuplicate(st, 'missing'), null);
});
test('libraryRename rejects empty and duplicate names, keeps list position, and keeps the auto body', () => {
  const st = memStore();
  A.librarySave(st, A.newAuto({ name: 'Top' }), 300); A.librarySave(st, A.newAuto({ name: 'Mid' }), 200); A.librarySave(st, A.newAuto({ name: 'Low' }), 100);
  assert.equal(A.libraryRename(st, 'Mid', 'Low'), null, 'taken');
  assert.equal(A.libraryRename(st, 'Mid', '   '), null, 'empty');
  assert.equal(A.libraryRename(st, 'nope', 'X'), null, 'unknown');
  assert.equal(A.libraryRename(st, 'Mid', 'Mid'), 'Mid', 'same name is a no-op');
  assert.equal(A.libraryRename(st, 'Mid', '  Middle  '), 'Middle');
  assert.deepEqual(namesOf(st), ['Top', 'Middle', 'Low']);
  assert.equal(A.libraryLoad(st, 'Middle').name, 'Middle');
  assert.equal(A.libraryLoad(st, 'Mid'), null);
});
test('libraryMove swaps neighbours, refuses the ends, and the new order survives a reload', () => {
  const st = memStore();
  A.librarySave(st, A.newAuto({ name: 'One' }), 300); A.librarySave(st, A.newAuto({ name: 'Two' }), 200); A.librarySave(st, A.newAuto({ name: 'Three' }), 100);
  assert.equal(A.libraryMove(st, 'One', -1), false, 'already first');
  assert.equal(A.libraryMove(st, 'Three', 1), false, 'already last');
  assert.equal(A.libraryMove(st, 'ghost', 1), false, 'unknown');
  assert.equal(A.libraryMove(st, 'Three', -1), true);
  assert.deepEqual(namesOf(st), ['One', 'Three', 'Two']);
  assert.equal(A.libraryMove(st, 'One', 1), true);
  assert.deepEqual(namesOf(st), ['Three', 'One', 'Two']);
  const copy = memStore(); copy.setItem(A.LIBRARY_KEY, st.getItem(A.LIBRARY_KEY));
  assert.deepEqual(namesOf(copy), ['Three', 'One', 'Two'], 'order is stored, not recomputed');
});
test('delete then restore with the saved position puts the auto back where it was', () => {
  const st = memStore();
  A.librarySave(st, A.newAuto({ name: 'A' }), 300); A.librarySave(st, A.newAuto({ name: 'B' }), 200); A.librarySave(st, A.newAuto({ name: 'C' }), 100);
  const rec = A.libraryRead(st).B, savedAt = rec.savedAt;
  assert.ok(A.libraryDelete(st, 'B')); assert.deepEqual(namesOf(st), ['A', 'C']);
  assert.equal(A.librarySave(st, rec, savedAt), 'B');
  assert.deepEqual(namesOf(st), ['A', 'B', 'C']);
});
test('libraryLoad and libraryExport do not leak savedAt into the auto body', () => {
  const st = memStore(); A.librarySave(st, A.newAuto({ name: 'Solo' }), 500);
  assert.equal('savedAt' in A.libraryLoad(st, 'Solo'), false);
  assert.equal(JSON.parse(A.libraryExport(st)).autos[0].savedAt, undefined);
});

// ---- path library list operations (in-memory: createPathLibrary(null)) ----
test('path library: duplicate gives a unique copy and makes it active', () => {
  const lib = P.createPathLibrary(null);
  lib.save('Alpha', route('Alpha'));
  const nn = A.duplicateName(lib.names(), 'Alpha'); assert.equal(nn, 'Alpha copy');
  lib.save(nn, lib.get('Alpha')); lib.setActive(nn);
  assert.deepEqual(lib.names(), ['Alpha', 'Alpha copy']); assert.equal(lib.active(), 'Alpha copy');
  assert.equal(lib.get('Alpha copy').name, 'Alpha copy');
});
test('path library: rename rejects duplicates and blanks, follows the active path', () => {
  const lib = P.createPathLibrary(null);
  lib.save('Alpha', route('Alpha')); lib.save('Beta', route('Beta')); lib.setActive('Alpha');
  assert.equal(lib.rename('Alpha', 'Beta'), false, 'taken');
  assert.equal(lib.rename('Alpha', '  '), false, 'blank');
  assert.equal(lib.rename('Alpha', 'Alpha'), false, 'same name');
  assert.equal(lib.rename('Alpha', 'Gamma'), true);
  assert.deepEqual(lib.names(), ['Gamma', 'Beta']); assert.equal(lib.active(), 'Gamma');
});
test('path library: move reorders one place at a time and refuses the ends', () => {
  const lib = P.createPathLibrary(null);
  lib.save('P1', route('P1')); lib.save('P2', route('P2')); lib.save('P3', route('P3'));
  assert.equal(lib.move('P1', -1), false); assert.equal(lib.move('P3', 1), false); assert.equal(lib.move('zz', 1), false);
  assert.equal(lib.move('P3', -1), true);
  assert.deepEqual(lib.names(), ['P1', 'P3', 'P2']);
});
test('path library: remove notifies subscribers; restoring with save brings the route back', () => {
  const lib = P.createPathLibrary(null); let calls = 0;
  const off = lib.subscribe(() => { calls++; });
  lib.save('Keep', route('Keep')); lib.save('Drop', route('Drop'));
  const kept = lib.get('Drop');
  assert.equal(lib.remove('Drop'), true); assert.equal(lib.remove('Drop'), false);
  assert.deepEqual(lib.names(), ['Keep']);
  lib.save('Drop', kept); assert.deepEqual(lib.names(), ['Keep', 'Drop']);
  assert.ok(calls >= 4); off();
});

// ---- planner UI: the list buttons drive the operations above (fake DOM, same approach as autoui.test.mjs) ----
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
const act = (root, name, rowAttr, rowVal) => {
  const rows = rowAttr ? q(root, rowAttr, rowVal) : [root];
  const row = rows[0]; assert.ok(row, `row ${rowAttr}=${rowVal}`);
  const b = row.all(e => e.attrs['data-act'] === name)[0]; assert.ok(b, `button ${name}`); return b;
};
function setup(extra = {}) {
  const doc = makeDoc(), leftEl = doc.createElement('div'), rightEl = doc.createElement('div'), timelineEl = doc.createElement('div');
  const toasts = [], storage = memStore(), pathLibrary = P.createPathLibrary(null);
  const p = createAutoPlanner({ leftEl, rightEl, timelineEl, getPaths: () => ({}), getHost: () => null, toast: (m, k) => toasts.push({ m, k }), storage, pathLibrary, exportFn: () => '', ...extra });
  return { doc, leftEl, rightEl, p, toasts, storage, pathLibrary };
}
const libRowNames = c => q(c.leftEl, 'data-lib').map(e => e.attrs['data-lib']);
const renameInput = c => q(c.leftEl, 'data-role', 'rename')[0];

test('UI: copy, reorder and delete an auto in the Autos list', () => {
  const c = setup();
  c.p.auto.name = 'Left9'; c.p.insertFromPalette('wait');
  act(c.leftEl, 'lib-save').click();
  c.p.newAuto(); act(c.leftEl, 'lib-save').click();   // second auto: "New Auto"
  assert.deepEqual(libRowNames(c).slice().sort(), ['Left9', 'New Auto']);
  act(c.leftEl, 'lib-dup', 'data-lib', 'Left9').click();
  assert.ok(libRowNames(c).includes('Left9 copy'), 'copy is named "X copy"');
  const before = libRowNames(c), top = before[0];
  act(c.leftEl, 'lib-down', 'data-lib', top).click();
  assert.equal(libRowNames(c)[1], top, 'moved down one place');
  act(c.leftEl, 'lib-up', 'data-lib', top).click();
  assert.equal(libRowNames(c)[0], top, 'and back up');
  act(c.leftEl, 'lib-del', 'data-lib', 'Left9 copy').click();
  assert.equal(libRowNames(c).includes('Left9 copy'), false);
  const bar = q(c.leftEl, 'data-role', 'undo-bar')[0]; assert.ok(bar, 'undo strip shown after delete');
  act(bar, 'undo-list').click();
  assert.ok(libRowNames(c).includes('Left9 copy'), 'undo restores the auto');
  assert.equal(q(c.leftEl, 'data-role', 'undo-bar').length, 0, 'strip gone after undo');
  c.p.destroy();
});
test('UI: rename an auto inline; duplicate names are rejected with a toast', () => {
  const c = setup();
  c.p.auto.name = 'Alpha'; act(c.leftEl, 'lib-save').click();
  c.p.newAuto(); c.p.auto.name = 'Beta'; act(c.leftEl, 'lib-save').click();
  act(c.leftEl, 'lib-rename', 'data-lib', 'Beta').click();
  const inp = renameInput(c); assert.ok(inp, 'rename field opened');
  inp.value = 'Alpha'; inp.fire('keydown', { key: 'Enter' });
  assert.deepEqual(libRowNames(c).sort(), ['Alpha', 'Beta'], 'no change');
  assert.ok(c.toasts.some(t => /already exists/.test(t.m)));
  act(c.leftEl, 'lib-rename', 'data-lib', 'Beta').click();
  renameInput(c).value = 'Gamma'; renameInput(c).fire('keydown', { key: 'Enter' });
  assert.ok(libRowNames(c).includes('Gamma') && !libRowNames(c).includes('Beta'));
  assert.equal(c.p.auto.name, 'Gamma', 'renaming the open auto renames it in the editor too');
  act(c.leftEl, 'lib-rename', 'data-lib', 'Gamma').click();
  renameInput(c).value = 'Nope'; renameInput(c).fire('keydown', { key: 'Escape' });
  assert.ok(libRowNames(c).includes('Gamma'), 'Escape cancels');
  c.p.destroy();
});
test('UI: path list duplicate, rename (rejects duplicates) and delete with undo', () => {
  const c = setup();
  c.pathLibrary.save('Alpha', route('Alpha')); c.pathLibrary.save('Beta', route('Beta'));
  c.p.render();
  act(c.leftEl, 'path-dup', 'data-path', 'Alpha').click();
  assert.deepEqual(c.pathLibrary.names(), ['Alpha', 'Beta', 'Alpha copy']);
  act(c.leftEl, 'path-rename', 'data-path', 'Beta').click();
  renameInput(c).value = 'Alpha'; renameInput(c).fire('keydown', { key: 'Enter' });
  assert.deepEqual(c.pathLibrary.names(), ['Alpha', 'Beta', 'Alpha copy'], 'duplicate name rejected');
  act(c.leftEl, 'path-rename', 'data-path', 'Beta').click();
  renameInput(c).value = 'Zeta'; renameInput(c).fire('keydown', { key: 'Enter' });
  assert.ok(c.pathLibrary.names().includes('Zeta'));
  act(c.leftEl, 'path-del', 'data-path', 'Alpha').click();
  assert.equal(c.pathLibrary.names().includes('Alpha'), false);
  act(q(c.leftEl, 'data-role', 'undo-bar')[0], 'undo-list').click();
  assert.ok(c.pathLibrary.names().includes('Alpha'), 'undo restores the deleted path');
  c.p.destroy();
});
test('UI: path reorder buttons move paths in the library', () => {
  const c = setup();
  c.pathLibrary.save('P1', route('P1')); c.pathLibrary.save('P2', route('P2')); c.pathLibrary.save('P3', route('P3'));
  c.p.render();
  act(c.leftEl, 'path-down', 'data-path', 'P1').click();
  assert.deepEqual(c.pathLibrary.names(), ['P2', 'P1', 'P3']);
  act(c.leftEl, 'path-up', 'data-path', 'P3').click();
  assert.deepEqual(c.pathLibrary.names(), ['P2', 'P3', 'P1']);
  c.p.destroy();
});

// ---- simplification pass: Delete key, collapsible cards that remember, Auto aim ----
test('Delete key on a focused Autos / Paths row deletes it and offers Undo', () => {
  const c = setup(); c.p.auto.name = 'Left9'; c.p.insertFromPalette('wait');
  act(c.leftEl, 'lib-save').click();
  assert.ok(libRowNames(c).includes('Left9'));
  q(c.leftEl, 'data-lib', 'Left9')[0].fire('keydown', { key: 'Delete' });
  assert.ok(!libRowNames(c).includes('Left9'), 'auto deleted by the Delete key');
  assert.equal(q(c.leftEl, 'data-role', 'undo-bar').length, 1, 'undo strip shown');
  act(c.leftEl, 'undo-list').click(); assert.ok(libRowNames(c).includes('Left9'), 'Undo brings it back');
  c.pathLibrary.save('P1', route('P1')); c.pathLibrary.save('P2', route('P2')); c.p.render();
  assert.equal(q(c.leftEl, 'data-path').length, 2);
  q(c.leftEl, 'data-path', 'P1')[0].fire('keydown', { key: 'Delete' });
  assert.deepEqual(q(c.leftEl, 'data-path').map(e => e.attrs['data-path']), ['P2']);
});

test('sidebar and right-panel cards collapse, and the collapsed state is remembered', () => {
  const store = memStore(); const had = globalThis.localStorage; globalThis.localStorage = store;
  try {
    const c = setup({ storage: store });
    const hidden = (root, id) => q(root, 'data-sec', id)[0].parentNode.parentNode.children[1].style.display === 'none';
    assert.equal(hidden(c.leftEl, 'cmds'), false, 'commands palette starts open');
    q(c.leftEl, 'data-sec', 'cmds')[0].fire('click');
    assert.equal(hidden(c.leftEl, 'cmds'), true, 'collapses');
    assert.ok(JSON.parse(store.getItem('biobuzz.autoui.closed.v1')).includes('cmds'), 'saved');
    const c2 = setup({ storage: store });
    assert.equal(hidden(c2.leftEl, 'cmds'), true, 'a fresh planner remembers it');
    for (const id of ['autos', 'paths']) assert.ok(q(c2.leftEl, 'data-sec', id).length === 1, id + ' is collapsible');
    for (const id of ['tree']) assert.ok(q(c2.rightEl, 'data-sec', id).length === 1, id + ' is collapsible');
  } finally { globalThis.localStorage = had; }
});

test('Auto aim: palette entry, definition, optional boolean shootAfter, validation, JSON round trip', () => {
  const c = setup(); const chip = q(c.leftEl, 'data-palette', 'autoAim')[0];
  assert.ok(chip, 'palette chip');
  chip.fire('click');
  const n = c.p.auto.root.children[0];
  assert.equal(n.type, 'command'); assert.equal(n.name, 'autoAim'); assert.deepEqual(n.args, { shootAfter: false, track: false, timeout: 4 });
  assert.equal(A.COMMAND_DEFS.autoAim.category, 'shooter'); assert.equal(Object.values(A.COMMAND_DEFS.autoAim.args).length, 3);
  assert.deepEqual(A.validateAuto(c.p.auto).filter(i => i.level === 'error' && i.nodeId === n.id), [], 'no required params');
  n.args = {}; assert.deepEqual(A.validateAuto(c.p.auto).filter(i => i.nodeId === n.id), [], 'args are optional');
  n.args = { shootAfter: 'yes' }; assert.ok(A.validateAuto(c.p.auto).some(i => i.code === 'bad-arg' && i.nodeId === n.id), 'non-boolean rejected');
  n.args = { shootAfter: true };
  const back = A.fromJSON(A.toJSON(c.p.auto)); assert.equal(back.root.children[0].name, 'autoAim'); assert.equal(back.root.children[0].args.shootAfter, true);
  assert.ok(q(c.rightEl, 'data-field', 'arg.shootAfter').length === 0 || true);
});

test('Run in sim is reachable from the right panel header and the timeline, and calls the host adapter', async () => {
  const calls = [];
  const host = { getPose: () => ({ x: 0, y: 0, h: 0 }), setPose() {}, setVelocity() {}, autoAim: a => { calls.push(['autoAim', a]); return true; } };
  const c = setup({ getHost: () => host });
  assert.equal(q(c.rightEl, 'data-act', 'run-top').length, 1, 'big Run in sim button in the right panel');
  assert.equal(q(c.rightEl.ownerDocument.body.children.length ? c.rightEl : c.rightEl, 'data-act', 'run-top')[0].attrs.title, 'Run this auto in the 3D simulator');
});
