import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import * as P from '../web/js/pathing.js';
import { createPathEditor } from '../web/js/pathui.js';
import { drawPlannerField, drawFieldOverlay, FIELD_COLORS } from '../web/js/plannerfield.js';

const here = path.dirname(fileURLToPath(import.meta.url));
const scene = JSON.parse(fs.readFileSync(path.join(here, '../web/data/scene.json'), 'utf8'));
const appSrc = fs.readFileSync(path.join(here, '../web/js/app.js'), 'utf8');
const mk = () => createPathEditor({ mx: x => x * 5, my: y => y * 5, getK: () => 5, fetchImpl: async () => { throw new Error('offline'); }, debounceMs: 0 });
function stub() {
  const log = [], store = {};
  const ctx = new Proxy(store, {
    get(t, k) { if (k === 'measureText') return s => ({ width: String(s).length * 6 }); if (!(k in t)) t[k] = (...a) => { log.push([k, ...a]); }; return t[k]; },
    set(t, k, v) { t[k] = v; if (k === 'fillStyle' || k === 'strokeStyle') log.push(['set:' + k, v]); return true; },
  });
  return { ctx, log };
}

test('pathing view quarter turn: toScreen/fromScreen round trip and the alliance half FILLS a landscape box', () => {
  for (const [alliance, q] of [['blue', 1], ['red', -1]]) {
    const view = { bounds: P.viewBounds(alliance), rot90: q };
    const W = 1200, H = 600, k = P.viewScale(view, W, H);
    assert.ok(Math.abs(k - Math.min(W / 144, H / 72)) < 1e-9, 'half field turned: 144 wide x 72 tall');
    for (const [x, y] of [[80, 10], [100, 140], [130, 72]].map(([a, b]) => (alliance === 'red' ? [144 - a, b] : [a, b]))) {
      const s = P.toScreen(view, x, y, W, H), back = P.fromScreen(view, s.x, s.y, W, H);
      assert.ok(Math.abs(back.x - x) < 1e-6 && Math.abs(back.y - y) < 1e-6, `${alliance} ${x},${y}`);
    }
    // own wall at the bottom of the screen: the wall corner maps near the bottom edge
    const wall = P.toScreen(view, alliance === 'blue' ? 144 : 0, 72, W, H);
    assert.ok(Math.abs(wall.y - H) < 1e-6, alliance + ' wall is the bottom edge');
    // the half spans the full box width
    const a = P.toScreen(view, 100, 0, W, H), b = P.toScreen(view, 100, 144, W, H);
    assert.ok(Math.abs(Math.abs(a.x - b.x) - W) < 1e-6, 'fills the width, no side bars');
  }
  assert.equal(P.toScreen({ bounds: P.viewBounds('blue'), rotate180: true }, 144, 0, 100, 200).x, P.toScreen({ bounds: P.viewBounds('blue'), rotate180: true, rot90: 0 }, 144, 0, 100, 200).x, 'rot90 0 keeps the old rotate180 mapping');
});

test('plannerfield draws the real scene as solid shapes: no translucent fills, border, no text', () => {
  const { ctx, log } = stub();
  const k = 6, env = { W: 1000, H: 700, k, mx: x => 20 + x * k, my: y => 20 + (144 - y) * k, bounds: { x0: 0, x1: 144, y0: 0, y1: 144, full: true }, scene, hive: { red: 'down', blue: 'down' }, mouse: null };
  drawPlannerField(ctx, env);
  const styles = log.filter(l => l[0] === 'set:fillStyle' || l[0] === 'set:strokeStyle').map(l => String(l[1]));
  assert.ok(!styles.some(s => /rgba\(/.test(s) && !/,\s*1\)$/.test(s)), 'no translucent colours: ' + styles.filter(s => /rgba/.test(s)).join(','));
  assert.ok(styles.includes(FIELD_COLORS.bg) && styles.includes(FIELD_COLORS.border), 'flat background and white border');
  assert.equal(log.filter(l => l[0] === 'fillText' || l[0] === 'strokeText').length, 0, 'the field itself has no captions (nothing to clip or overlap)');
  assert.ok(log.filter(l => l[0] === 'stroke').length > 10, 'tapes, hive and flowers drawn as outlines');
});

test('field overlay: gizmo and readout stay inside the free area in every quarter turn', () => {
  for (const q of [0, 1, -1]) {
    const { ctx, log } = stub();
    const A = { L: 280, T: 108, R: 1236 + 280, B: 700 };
    const px = (x, y) => { const cx = (A.L + A.R) / 2, cy = (A.T + A.B) / 2, a = q * Math.PI / 2, dx = (20 + x * 6) - cx, dy = (20 + (144 - y) * 6) - cy; return [cx + dx * Math.cos(a) - dy * Math.sin(a), cy + dx * Math.sin(a) + dy * Math.cos(a)]; };
    drawFieldOverlay(ctx, { bounds: { x0: 0, x1: 144, y0: 0, y1: 144 }, px, avail: A, mx: x => x, my: y => y, mouse: [600, 400], fromScreen: () => [10, 20] });
    const texts = log.filter(l => l[0] === 'fillText');
    assert.ok(texts.length >= 3, 'x, y labels and the readout');
    for (const [, , x, y] of texts) assert.ok(x >= A.L && x <= A.R && y >= A.T && y <= A.B, `q=${q} label at ${x},${y} inside the free area`);
  }
});

test('path editor panel is PathPlanner cards: waypoints, markers, rotation targets, states, constraints, zones (no placeholder cards)', () => {
  const ed = mk(); ed.active = true; ed.route.start = { x: 120, y: 24, h: Math.PI / 2 }; P.addWaypoint(ed.route, 100, 60); P.addWaypoint(ed.route, 90, 100);
  const html = ed.panelHtml();
  const titles = ['Waypoints', 'Event Markers', 'Rotation Targets', 'Starting State', 'Final State', 'Global Constraints', 'Constraint Zones'];
  let at = -1; for (const t of titles) { const i = html.indexOf(`<span>${t}</span>`); assert.ok(i > at, `${t} present and in order`); at = i; }
  assert.ok(!/Point Towards|Path Optimizer/.test(html), 'placeholder cards removed');
  assert.match(html, /class="pp-time"[^>]*>~\d+\.\d\ds</, 'green time chip like ~3.12s');
  assert.match(html, /class="pp-count">3</, 'waypoint count');
  assert.match(html, /&deg; starting/); assert.match(html, /&deg; ending/);
  ed.sel = 0; ed.act('addrot'); ed.act('addzone'); ed.act('addev');
  const h2 = ed.panelHtml();
  assert.equal((h2.match(/class="pp-count">1</g) || []).length, 3, 'rotation target, zone and marker counts each 1');
  assert.ok(ed.route.segments[0].rotations.length === 1 && ed.route.segments[0].zones.length === 1 && ed.route.segments[0].callbacks.length === 1);
  ed.open.add('rotations'); ed.open.add('zones');
  const h3 = ed.panelHtml(); assert.match(h3, /data-rotdel="0" data-rseg="0"/); assert.match(h3, /data-zdel="0" data-zseg="0"/);
});

test('floating playback bar is a separate element: play/pause, reset, slider; the panel then omits its copy', () => {
  const ed = mk(); ed.active = true; P.addWaypoint(ed.route, 100, 60);
  const fake = (() => { const e = { innerHTML: '', querySelector() { return null; }, querySelectorAll() { return []; } }; return e; })();
  assert.match(ed.panelHtml(), /data-f="scrub"/, 'embedded when no floating bar is bound');
  ed.bindPlayback(fake);
  assert.match(fake.innerHTML, /data-act="toggle"/); assert.match(fake.innerHTML, /data-act="stop"/); assert.match(fake.innerHTML, /data-f="scrub"/);
  assert.doesNotMatch(ed.panelHtml(), /data-f="scrub"/, 'panel no longer carries the slider');
  ed.bindPlayback(null); assert.match(ed.panelHtml(), /data-f="scrub"/);
});

test('ed.rename (editor top bar) renames the open path with one undo entry', () => {
  const ed = mk(); ed.active = true; P.addWaypoint(ed.route, 100, 60);
  ed.rename('Score Path'); assert.equal(ed.route.name, 'Score Path');
  assert.equal(ed.undo(), true); assert.notEqual(ed.route.name, 'Score Path');
});

test('dotted trajectory: no solid polyline, no robot fill; labels counter-rotate under a view quarter turn', () => {
  const ed = mk(); ed.active = true; ed.grid = 0; ed.route.start = { x: 20, y: 20, h: 0 }; P.addWaypoint(ed.route, 100, 20);
  const { ctx, log } = stub(); ed.draw(ctx);
  const dots = log.filter(l => l[0] === 'arc').length;
  assert.ok(dots > 20, 'many small dots along the path: ' + dots);
  assert.equal(log.filter(l => l[0] === 'fillRect' && l[3] > 20).length, 0, 'no big filled boxes');
  P.PLAN_VIEW.viewQuarter = 1; try { const s2 = stub(); P.addEvent(ed.route, 0, 0.5, 'shoot'); ed.draw(s2.ctx); assert.ok(s2.log.some(l => l[0] === 'rotate'), 'tag text rotated back'); } finally { P.PLAN_VIEW.viewQuarter = 0; }
});

test('app.js: the field area is measured from the planner panels and the half is turned to fill it', () => {
  assert.match(appSrc, /function autoAvail\(\)/);
  assert.match(appSrc, /vis\('pp-topbar'\)/); assert.match(appSrc, /vis\('pp-float'\)/); assert.match(appSrc, /vis\('timeline'\)/);
  assert.match(appSrc, /function planRot90\(\)/);
  assert.match(appSrc, /viewScale\(\{ bounds: b, rotate180: false \}, A\.availH, A\.availW\)/);
  assert.match(appSrc, /if \(S\.view === 'auto'\) \{ drawPlannerView\(\); return; \}/);
  assert.match(appSrc, /pathEditor\.drawField\(mctx, env\)/);
  assert.match(appSrc, /if \(q\) \[sx, sy\] = rotPt\(sx, sy, -q \* Math\.PI \/ 2\)/, 'pointer input is un-rotated before the field transform');
});
