// Property-style robustness test for the path planner model (web/js/pathing.js).
// KNOWN-BUG repro (round-trip): addCurve with auto control can place a control point off-field.
//   const r = P.newRoute({ start: { x: 10, y: 10, h: 0 } }); P.addCurve(r, 10, 140);
//   P.fromJSON(P.toJSON(r)); // throws "segments[0].points[1].x: -22.5 is outside the field (0..144)"
import test from 'node:test';
import assert from 'node:assert/strict';
import * as P from '../web/js/pathing.js';

// Seeded LCG (no Math.random): deterministic across runs.
function lcg(seed) { let s = seed >>> 0; return () => { s = (s * 1664525 + 1013904223) % 4294967296; return s / 4294967296; }; }
const same = (a, b) => { try { assert.deepStrictEqual(a, b); return true; } catch (_) { return false; } };

function fuzz(seed, N = 500) {
  const rnd = lcg(seed), ri = n => Math.floor(rnd() * n), pt = () => ({ x: rnd() * 144, y: rnd() * 144 });
  const route = P.newRoute({ name: 'Fuzz' }), h = P.createHistory(route);
  const bad = { threw: [], nan: [], minPts: [], continuity: [], roundtrip: [] };
  const check = tag => {
    const all = [route.start, ...route.segments.flatMap(s => s.points)];
    if (all.some(p => !Number.isFinite(p.x) || !Number.isFinite(p.y))) bad.nan.push(tag);
    if (route.segments.some(s => s.points.length < 2)) bad.minPts.push(tag);
    let prev = route.start;
    for (const s of route.segments) {
      if (Math.hypot(prev.x - s.points[0].x, prev.y - s.points[0].y) > 1e-6) { bad.continuity.push(tag); break; }
      prev = s.points.at(-1);
    }
    try { const text = P.toJSON(route); if (!same(JSON.parse(P.toJSON(P.fromJSON(text))), JSON.parse(text))) bad.roundtrip.push(tag + ' mismatch'); }
    catch (e) { bad.roundtrip.push(`${tag} ${e.message}`); }
  };
  for (let k = 0; k < N; k++) {
    const roll = rnd(); let op;
    try {
      if (roll < 0.25) { op = 'addLine'; const p = pt(); P.addLine(route, p.x, p.y); }
      else if (roll < 0.45) { op = 'addCurve'; const p = pt(); if (rnd() < 0.5) P.addCurve(route, p.x, p.y); else { const c = pt(); P.addCurve(route, p.x, p.y, c.x, c.y); } }
      else if (roll < 0.6) { op = 'deletePoint'; const s = ri(route.segments.length || 1); P.deletePoint(route, s, ri(route.segments[s]?.points.length || 1)); }
      else if (roll < 0.7) { op = 'deleteSegment'; P.deleteSegment(route, ri(route.segments.length || 1)); }
      else if (roll < 0.85) { op = 'moveSegment'; P.moveSegment(route, ri(route.segments.length || 1), rnd() < 0.5 ? -1 : 1); }
      else { op = 'undo/redo'; if (rnd() < 0.5) h.undo(); else h.redo(); }
      if (op !== 'undo/redo' && rnd() < 0.7) h.commit();
    } catch (e) { bad.threw.push(`${k} ${op}: ${e.message}`); }
    check(`${k} ${op}`);
  }
  return bad;
}

const run = fuzz(20260711);

test('500 random ops: no exceptions thrown', () => assert.deepEqual(run.threw, []));
test('500 random ops: no NaN coordinates', () => assert.deepEqual(run.nan, []));
test('500 random ops: every segment has >= 2 points', () => assert.deepEqual(run.minPts, []));
test('500 random ops: consecutive segments are continuous', () => assert.deepEqual(run.continuity, []));
test('KNOWN-BUG: toJSON/fromJSON round-trip after random ops', { todo: true }, () => assert.deepEqual(run.roundtrip, []));
