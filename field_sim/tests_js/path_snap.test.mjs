import test from 'node:test';
import assert from 'node:assert/strict';
import { createPathEditor, gridLines } from '../web/js/pathui.js';
import * as P from '../web/js/pathing.js';

const mk = () => createPathEditor({ mx: x => x * 5, my: y => y * 5, getK: () => 5, fetchImpl: async () => { throw new Error('offline'); }, debounceMs: 0 });
const near = (a, b, eps = 1e-6) => assert.ok(Math.abs(a - b) <= eps, `expected ${a} to be ${b}`);

// Recording canvas stub (same idea as path_draw.test.mjs): every method logs its name.
function stubCtx() {
  const log = [], store = {};
  const ctx = new Proxy(store, {
    get(t, k) {
      if (k === 'measureText') return s => ({ width: String(s).length * 6 });
      if (!(k in t)) t[k] = (...a) => { log.push(k); return undefined; };
      return t[k];
    },
    set(t, k, v) { t[k] = v; return true; },
  });
  return { ctx, log };
}

// ---------- gridLines (pure) ----------
test('gridLines: step 24 over 144 gives 7 lines per axis (0..144 inclusive)', () => {
  const g = gridLines({ x0: 0, x1: 144, y0: 0, y1: 144 }, 24);
  assert.deepEqual(g.vertical, [0, 24, 48, 72, 96, 120, 144]);
  assert.deepEqual(g.horizontal, [0, 24, 48, 72, 96, 120, 144]);
});

test('gridLines: half-field bounds keep only lines inside x 72..144', () => {
  const g = gridLines({ x0: 72, x1: 144, y0: 0, y1: 144 }, 24);
  assert.deepEqual(g.vertical, [72, 96, 120, 144]);
  assert.equal(g.horizontal.length, 7);
});

test('gridLines: step that does not divide the bounds stops at the last line inside', () => {
  const g = gridLines({ x0: 0, x1: 10, y0: 1, y1: 9 }, 4);
  assert.deepEqual(g.vertical, [0, 4, 8]);
  assert.deepEqual(g.horizontal, [4, 8]);
});

test('gridLines: fractional step (0.5) is exact, no float drift', () => {
  const g = gridLines({ x0: 0, x1: 2, y0: 0, y1: 0 }, 0.5);
  assert.deepEqual(g.vertical, [0, 0.5, 1, 1.5, 2]);
  assert.deepEqual(g.horizontal, [0]);
});

test('gridLines: step 0 (grid off), negative, or missing bounds give no lines', () => {
  assert.deepEqual(gridLines({ x0: 0, x1: 144, y0: 0, y1: 144 }, 0), { vertical: [], horizontal: [] });
  assert.deepEqual(gridLines({ x0: 0, x1: 144, y0: 0, y1: 144 }, -3), { vertical: [], horizontal: [] });
  assert.deepEqual(gridLines(null, 24), { vertical: [], horizontal: [] });
});

// ---------- snapPoint / snapAngle (read-only from pathing.js) ----------
test('snapPoint: grid snap rounds to the step and clamps to the field', () => {
  const route = P.newRoute({ name: 't', start: { x: 0, y: 0, h: 0 } });
  const r = P.snapPoint(route, 25.2, 37.9, { grid: 1, magnet: false });
  assert.deepEqual([r.x, r.y, r.snapped], [25, 38, 'grid']);
  const c = P.snapPoint(route, 150, -4, { grid: 24, magnet: false });
  assert.deepEqual([c.x, c.y], [144, 0]);
});

test('snapPoint: magnet beats the grid, and an excluded key is not a magnet target', () => {
  const route = P.newRoute({ name: 't', start: { x: 24, y: 24, h: 0 } });
  const magnet = P.snapPoint(route, 25, 25.5, { grid: 24, magnet: true, radius: 2 });
  assert.equal(magnet.snapped, 'point');
  assert.deepEqual([magnet.x, magnet.y], [24, 24]);
  const skipped = P.snapPoint(route, 25, 25.5, { grid: 24, magnet: true, radius: 2, exclude: new Set(['0:0']) });
  assert.equal(skipped.snapped, 'grid');
});

test('snapAngle: rounds to the step, step 0 leaves the angle free', () => {
  near(P.snapAngle(20 * Math.PI / 180, 15), 15 * Math.PI / 180);
  near(P.snapAngle(20 * Math.PI / 180, 0), 20 * Math.PI / 180);
});

// ---------- editor: drags use the grid ----------
test('waypoint drag snaps the end anchor to the grid', () => {
  const ed = mk(); ed.active = true; ed.tool = 'select'; ed.grid = 1; ed.magnet = false;
  P.addLine(ed.route, 50, 50);
  ed.pointer('down', 50, 50, { button: 0 });
  ed.pointer('move', 51.4, 49.6, {});
  ed.pointer('up', 51.4, 49.6, {});
  const p = ed.route.segments[0].points.at(-1);
  assert.deepEqual([p.x, p.y], [51, 50]);
});

test('control point drag snaps to the grid (curve segment handle)', () => {
  const ed = mk(); ed.active = true; ed.tool = 'select'; ed.grid = 2; ed.magnet = false;
  P.addWaypoint(ed.route, 60, 60);
  const pts = ed.route.segments[0].points;
  const ctl = pts[1];                                            // first control handle (j = 1)
  ed.pointer('down', ctl.x, ctl.y, { button: 0 });
  const tx = ctl.x + 0.9, ty = ctl.y + 0.2;
  ed.pointer('move', tx, ty, {});
  ed.pointer('up', tx, ty, {});
  const after = ed.route.segments[0].points[1];
  assert.deepEqual([after.x, after.y], [Math.round(tx / 2) * 2, Math.round(ty / 2) * 2]);
});

test('start marker drag snaps to the grid', () => {
  const ed = mk(); ed.active = true; ed.tool = 'select'; ed.grid = 1; ed.magnet = false;
  ed.route.start.x = 24; ed.route.start.y = 24;
  ed.pointer('down', 24, 24, { button: 0 });
  ed.pointer('move', 25.6, 22.4, {});
  ed.pointer('up', 25.6, 22.4, {});
  assert.deepEqual([ed.route.start.x, ed.route.start.y], [26, 22]);
});

test('Alt disables grid snapping while dragging', () => {
  const ed = mk(); ed.active = true; ed.tool = 'select'; ed.grid = 1; ed.magnet = false;
  P.addLine(ed.route, 50, 50);
  ed.pointer('down', 50, 50, { button: 0 });
  ed.pointer('move', 51.4, 49.6, { altKey: true });
  ed.pointer('up', 51.4, 49.6, {});
  const p = ed.route.segments[0].points.at(-1);
  near(p.x, 51.4, 1e-9); near(p.y, 49.6, 1e-9);
});

// ---------- heading snap ----------
test('heading drag: hdgSnap 0 leaves it free, Shift forces 15 deg steps', () => {
  const ed = mk(); ed.active = true; ed.tool = 'select'; ed.magnet = false; ed.hdgSnap = 0;
  const a = 20 * Math.PI / 180, hx = 24 + Math.cos(0) * 24 / 5, hy = 24;
  ed.route.start.x = 24; ed.route.start.y = 24; ed.route.start.h = 0;
  ed.pointer('down', hx, hy, { button: 0 });
  ed.pointer('move', 24 + 10 * Math.cos(a), 24 + 10 * Math.sin(a), {});
  near(ed.route.start.h, a, 1e-6);
  ed.pointer('move', 24 + 10 * Math.cos(a), 24 + 10 * Math.sin(a), { shiftKey: true });
  near(ed.route.start.h, 15 * Math.PI / 180, 1e-6);
  ed.pointer('up', 0, 0, {});
});

test('heading drag: hdgSnap 15 snaps even without Shift; Alt frees it', () => {
  const ed = mk(); ed.active = true; ed.tool = 'select'; ed.magnet = false; ed.hdgSnap = 15;
  const a = 20 * Math.PI / 180;
  ed.route.start.x = 24; ed.route.start.y = 24; ed.route.start.h = 0;
  ed.pointer('down', 24 + 24 / 5, 24, { button: 0 });
  ed.pointer('move', 24 + 10 * Math.cos(a), 24 + 10 * Math.sin(a), {});
  near(ed.route.start.h, 15 * Math.PI / 180, 1e-6);
  ed.pointer('move', 24 + 10 * Math.cos(a), 24 + 10 * Math.sin(a), { altKey: true });
  near(ed.route.start.h, a, 1e-6);
  ed.pointer('up', 0, 0, {});
});

// ---------- grid overlay ----------
test('grid overlay: showGrid adds 14 grid lines (7 per axis at step 24) to the draw', () => {
  const ed = mk(); ed.active = true; ed.sel = 0; ed.grid = 24;
  P.addLine(ed.route, 50, 50);
  const count = show => {
    ed.showGrid = show;
    const { ctx, log } = stubCtx();
    ed.draw(ctx);
    return log.filter(n => n === 'moveTo').length;
  };
  assert.equal(count(true) - count(false), 14);
});

test('grid overlay: grid off (0) or showGrid false draws no grid lines', () => {
  const ed = mk(); ed.active = true; ed.sel = 0;
  P.addLine(ed.route, 50, 50);
  const moves = () => { const { ctx, log } = stubCtx(); ed.draw(ctx); return log.filter(n => n === 'moveTo').length; };
  ed.grid = 0; ed.showGrid = true; const off = moves();
  ed.grid = 24; ed.showGrid = false; const hidden = moves();
  assert.equal(off, hidden);
});
