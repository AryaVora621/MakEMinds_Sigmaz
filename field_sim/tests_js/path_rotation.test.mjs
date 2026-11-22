import test from 'node:test';
import assert from 'node:assert/strict';
import * as P from '../web/js/pathing.js';

const near = (a, b, eps = 1e-9) => assert.ok(Math.abs(a - b) < eps, `expected ${a} to be ${b}`);
const mkRoute = () => {
  const r = P.newRoute({ start: { x: 10, y: 10, h: 0 } });
  P.addLine(r, 50, 10);
  r.segments[0].heading = { mode: 'linear', start: 0, end: Math.PI / 2, reversed: false };
  return r;
};

test('addRotation adds two targets kept sorted by t, headings wrapped', () => {
  const r = mkRoute();
  const i1 = P.addRotation(r, 0, 0.8, Math.PI / 2);
  const i0 = P.addRotation(r, 0, 0.25, 3 * Math.PI);
  assert.equal(i1, 0, 'first target on an empty list is index 0');
  assert.equal(i0, 0, 'the earlier target sorts first');
  assert.equal(r.segments[0].rotations.length, 2);
  assert.deepEqual(r.segments[0].rotations.map(c => c.t), [0.25, 0.8]);
  near(r.segments[0].rotations[0].h, Math.PI, 1e-9); // 3pi wraps to pi
  assert.equal(P.addRotation(r, 0, 1.5, 0), -1, 't outside 0..1 is refused');
  assert.equal(P.addRotation(r, 5, 0.5, 0), -1, 'unknown segment is refused');
});

test('moveRotation and removeRotation edit targets', () => {
  const r = mkRoute();
  P.addRotation(r, 0, 0.25, 0.1); P.addRotation(r, 0, 0.8, 0.2);
  assert.equal(P.moveRotation(r, 0, 0, 0.9, 0.3), 1, 'moved past the other target, re-sorted');
  assert.deepEqual(r.segments[0].rotations.map(c => c.t), [0.8, 0.9]);
  assert.equal(P.moveRotation(r, 0, 9, 0.5, 0), -1);
  assert.equal(P.removeRotation(r, 0, 0), true);
  assert.equal(r.segments[0].rotations.length, 1);
  assert.equal(P.removeRotation(r, 0, 4), false);
});

test('rotationAt interpolates linearly between anchors and targets, shortest way round', () => {
  const r = mkRoute();
  // anchors: start 0 at t=0, end pi/2 at t=1; one target at t=0.5 with heading pi/4
  P.addRotation(r, 0, 0.5, Math.PI / 4);
  near(P.rotationAt(r, 0, 0), 0);
  near(P.rotationAt(r, 0, 0.25), Math.PI / 8, 1e-9);
  near(P.rotationAt(r, 0, 0.5), Math.PI / 4, 1e-9);
  near(P.rotationAt(r, 0, 0.75), 3 * Math.PI / 8, 1e-9);
  near(P.rotationAt(r, 0, 1), Math.PI / 2, 1e-9);
  assert.equal(P.rotationAt(r, 9, 0.5), null);

  // shortest way: from 170 deg to -170 deg passes through 180, not 0
  const w = P.newRoute({ start: { x: 10, y: 10 } });
  P.addLine(w, 50, 10);
  w.segments[0].heading = { mode: 'linear', start: 170 * Math.PI / 180, end: -170 * Math.PI / 180, reversed: false };
  near(Math.abs(P.rotationAt(w, 0, 0.5)), Math.PI, 1e-9);
});

test('rotation targets survive toJSON/fromJSON and cloneRoute', () => {
  const r = mkRoute();
  P.addRotation(r, 0, 0.3, 0.7); P.addRotation(r, 0, 0.6, -0.4);
  const back = P.fromJSON(P.toJSON(r));
  assert.deepEqual(back.segments[0].rotations, r.segments[0].rotations);
  assert.deepEqual(P.cloneRoute(r).segments[0].rotations, r.segments[0].rotations);
  const text = JSON.parse(P.toJSON(r));
  assert.equal(text.segments[0].rotations.length, 2);
});

test('old JSON without rotations loads with an empty list', () => {
  const old = JSON.stringify({ schema: 1, name: 'Old', alliance: 'red', start: { x: 10, y: 10, h: 0 },
    segments: [{ type: 'line', points: [{ x: 10, y: 10 }, { x: 40, y: 10 }], heading: { mode: 'linear', reversed: false } }] });
  const r = P.fromJSON(old);
  assert.deepEqual(r.segments[0].rotations || [], [], 'no rotations field needed; reads as an empty list');
  assert.equal(P.toJSON(r).includes('rotations'), false, 'old routes re-save without the new field');
  assert.equal(P.rotationAt(r, 0, 0.5), 0);
});

test('fromJSON rejects malformed rotation targets', () => {
  const base = (rot) => JSON.stringify({ schema: 1, start: { x: 10, y: 10, h: 0 },
    segments: [{ type: 'line', points: [{ x: 10, y: 10 }, { x: 40, y: 10 }], heading: { mode: 'linear' }, rotations: rot }] });
  assert.throws(() => P.fromJSON(base('x')), /rotations: must be an array/);
  assert.throws(() => P.fromJSON(base([{ t: 1.5, h: 0 }])), /within 0\.\.1/);
  assert.throws(() => P.fromJSON(base([{ t: 0.5, h: null }])), /not a finite/);
});

test('mirrorRoute mirrors rotation headings and splitSegment divides them', () => {
  const r = mkRoute();
  P.addRotation(r, 0, 0.25, 0.5); P.addRotation(r, 0, 0.75, 1.0);
  const m = P.mirrorRoute(r);
  near(m.segments[0].rotations[0].h, Math.PI - 0.5, 1e-9);
  assert.equal(P.splitSegment(r, 0, 0.5), true);
  assert.deepEqual(r.segments[0].rotations.map(c => Math.round(c.t * 1000) / 1000), [0.5]);
  assert.deepEqual(r.segments[1].rotations.map(c => Math.round(c.t * 1000) / 1000), [0.5]);
});
