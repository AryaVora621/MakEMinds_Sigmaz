import test from 'node:test';
import assert from 'node:assert/strict';
import * as P from '../web/js/pathing.js';

// 100 in straight line along +x. Continuous trapezoid with vmax 60, aMax 90: 2*(60/90) + (100 - 2*20)/60 = 2.333 s.
const mkLine = () => { const r = P.newRoute({ start: { x: 10, y: 10, h: 0 } }); P.addLine(r, 110, 10); return r; };
const near = (a, b, eps) => assert.ok(Math.abs(a - b) < eps, `expected ${a} to be within ${eps} of ${b}`);

test('estimateTime is unchanged when a route has no zones (trapezoid reference)', () => {
  const r = mkLine();
  // pinned to the discretised follower (dt 0.005, 0.5 in samples): 2.295 s; the continuous trapezoid is 2.333 s
  near(P.estimateTime(r, 60, 90), 2.295, 0.01);
  const withEmpty = mkLine(); withEmpty.segments[0].zones = [];
  assert.equal(P.estimateTime(withEmpty, 60, 90), P.estimateTime(r, 60, 90), 'an empty zones list changes nothing');
  assert.equal(P.constraintsAt(r, 0, 0.5, { vmax: 60, aMax: 90 }).vmax, 60, 'no zones -> globals');
});

test('a slow zone increases estimateTime, and a zone on one half only slows that half', () => {
  const base = P.estimateTime(mkLine(), 60, 90);
  const r = mkLine();
  assert.equal(P.addZone(r, 0, 0, 1, 20, 90), 0);
  const slow = P.estimateTime(r, 60, 90);
  assert.ok(slow > base + 2.5, `slow zone should add time (base ${base}, slow ${slow})`);
  near(slow, 5.22, 0.06); // cruise at 20 in/s over the whole line, with the same accel ramps
  const half = mkLine(); P.addZone(half, 0, 0.5, 1, 20, 90);
  const ht = P.estimateTime(half, 60, 90);
  assert.ok(ht > base && ht < slow, 'half-length zone sits between no zone and full zone');
});

test('overlapping zones take the minimum vmax and aMax; outside zones fall back to globals', () => {
  const r = mkLine();
  P.addZone(r, 0, 0, 1, 30, 90);
  P.addZone(r, 0, 0.25, 0.75, 15, 40);
  assert.deepEqual(P.constraintsAt(r, 0, 0.5, { vmax: 60, aMax: 90 }), { vmax: 15, aMax: 40 }, 'overlap -> min of both');
  assert.deepEqual(P.constraintsAt(r, 0, 0.1, { vmax: 60, aMax: 90 }), { vmax: 30, aMax: 90 }, 'only the wide zone');
  assert.deepEqual(P.constraintsAt(r, 0, 0.5, { vmax: 60, aMax: 90 }), { vmax: 15, aMax: 40 });
  const r2 = mkLine(); P.addZone(r2, 0, 0.5, 0.6, 10, 20);
  assert.deepEqual(P.constraintsAt(r2, 0, 0.1, { vmax: 60, aMax: 90 }), { vmax: 60, aMax: 90 }, 'outside -> globals');
  assert.deepEqual(P.constraintsAt(r2, 5, 0.5, { vmax: 60, aMax: 90 }), { vmax: 60, aMax: 90 }, 'unknown segment -> globals');
});

test('addZone clamps t, orders t0 <= t1, rejects non-positive or non-finite speeds', () => {
  const r = mkLine();
  assert.equal(P.addZone(r, 0, 1.5, -0.2, 20, 50), 0);
  assert.deepEqual([r.segments[0].zones[0].t0, r.segments[0].zones[0].t1], [0, 1]);
  assert.equal(P.addZone(r, 0, 0.4, 0.2, 20, 50), 1, 'reversed range is ordered; sorted after the t0=0 zone');
  assert.deepEqual([r.segments[0].zones[1].t0, r.segments[0].zones[1].t1], [0.2, 0.4]);
  const before = JSON.stringify(r.segments[0].zones);
  assert.equal(P.addZone(r, 0, 0, 1, 0, 50), -1, 'vmax 0 rejected');
  assert.equal(P.addZone(r, 0, 0, 1, -5, 50), -1, 'negative vmax rejected');
  assert.equal(P.addZone(r, 0, 0, 1, 20, 0), -1, 'aMax 0 rejected');
  assert.equal(P.addZone(r, 0, 0, 1, NaN, 50), -1, 'NaN rejected');
  assert.equal(P.addZone(r, 9, 0, 1, 20, 50), -1, 'unknown segment rejected');
  assert.equal(JSON.stringify(r.segments[0].zones), before, 'rejected calls leave zones untouched');
});

test('removeZone removes by index and drops the field when empty', () => {
  const r = mkLine();
  P.addZone(r, 0, 0, 0.3, 20, 50); P.addZone(r, 0, 0.6, 1, 25, 50);
  assert.equal(P.removeZone(r, 0, 5), false);
  assert.equal(P.removeZone(r, 0, 1), true);
  assert.equal(r.segments[0].zones.length, 1);
  assert.equal(P.removeZone(r, 0, 0), true);
  assert.equal('zones' in r.segments[0], false);
});

test('zones survive toJSON/fromJSON and cloneRoute; absent zones stay absent (default [])', () => {
  const r = mkLine(); P.addZone(r, 0, 0.2, 0.7, 18, 45);
  const back = P.fromJSON(P.toJSON(r));
  assert.deepEqual(back.segments[0].zones, [{ t0: 0.2, t1: 0.7, vmax: 18, aMax: 45 }]);
  assert.deepEqual(P.cloneRoute(r).segments[0].zones, r.segments[0].zones);
  assert.deepEqual(P.toJSON(back), P.toJSON(r), 'JSON is stable across a round trip');
  const plain = mkLine();
  const plainBack = P.fromJSON(P.toJSON(plain));
  assert.equal(P.toJSON(plainBack).includes('zones'), false, 'no zones written for zone-free routes');
  assert.deepEqual(plainBack.segments[0].zones || [], []);
});

test('fromJSON rejects malformed zones', () => {
  const r = mkLine(); P.addZone(r, 0, 0, 1, 20, 50);
  const bad = o => { const j = JSON.parse(P.toJSON(r)); Object.assign(j.segments[0].zones[0], o); return JSON.stringify(j); };
  assert.throws(() => P.fromJSON(bad({ vmax: 0 })), /vmax and aMax must be > 0/);
  assert.throws(() => P.fromJSON(bad({ aMax: -1 })), /vmax and aMax must be > 0/);
  assert.throws(() => P.fromJSON(bad({ t1: 2 })), /within 0\.\.1/);
  const notArr = JSON.parse(P.toJSON(r)); notArr.segments[0].zones = 'x';
  assert.throws(() => P.fromJSON(JSON.stringify(notArr)), /must be an array/);
});

test('splitSegment divides zones between the two halves without sharing arrays', () => {
  const r = mkLine(); P.addZone(r, 0, 0, 0.4, 20, 50); P.addZone(r, 0, 0.6, 1, 25, 60);
  assert.equal(P.splitSegment(r, 0, 0.5), true);
  const [a, b] = r.segments;
  assert.equal(a.zones.length, 1); assert.equal(b.zones.length, 1);
  near(a.zones[0].t1, 0.8, 1e-9); assert.equal(a.zones[0].vmax, 20);
  near(b.zones[0].t0, 0.2, 1e-9); near(b.zones[0].t1, 1, 1e-9); assert.equal(b.zones[0].vmax, 25);
  assert.notEqual(a.zones, b.zones);
});
