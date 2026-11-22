import test from 'node:test';
import assert from 'node:assert/strict';
import * as P from '../web/js/pathing.js';

const near = (a, b, e = 1e-9) => assert.ok(Math.abs(a - b) <= e, `${a} vs ${b}`);

// Deterministic PRNG so the 100-point checks are repeatable.
function rng(seed) { let s = seed >>> 0; return () => { s = (s * 1664525 + 1013904223) >>> 0; return s / 4294967296; }; }

test('flipPoint mirror and rotate', () => {
  const m = P.flipPoint(30, 50, 'mirror');
  near(m.x, 114); near(m.y, 50);
  const r = P.flipPoint(30, 50, 'rotate');
  near(r.x, 114); near(r.y, 94);
});

test('flipPoint twice is identity', () => {
  const R = rng(1);
  for (let i = 0; i < 100; i++) {
    const x = R() * P.FIELD, y = R() * P.FIELD;
    for (const mode of ['mirror', 'rotate']) {
      const q = P.flipPoint(P.flipPoint(x, y, mode).x, P.flipPoint(x, y, mode).y, mode);
      near(q.x, x); near(q.y, y);
    }
  }
});

test('flipHeading mirror and rotate and twice is identity', () => {
  near(P.flipHeading(0.3, 'mirror'), P.wrapAngle(Math.PI - 0.3));
  near(P.flipHeading(0.3, 'rotate'), P.wrapAngle(0.3 + Math.PI));
  const R = rng(2);
  for (let i = 0; i < 100; i++) {
    const h = P.wrapAngle((R() - 0.5) * 20);
    for (const mode of ['mirror', 'rotate']) {
      near(P.wrapAngle(P.flipHeading(P.flipHeading(h, mode), mode)), P.wrapAngle(h));
    }
  }
});

test('allianceViewTransform: blue rotated, red not, no flipX', () => {
  assert.deepEqual(P.allianceViewTransform('blue'), { rotate180: true, flipX: false });
  assert.deepEqual(P.allianceViewTransform('red'), { rotate180: false, flipX: false });
});

test('toScreen and fromScreen round trip for both alliances (100 random points)', () => {
  const R = rng(3);
  for (const alliance of ['red', 'blue']) {
    const view = { bounds: P.viewBounds(alliance), rotate180: P.allianceViewTransform(alliance).rotate180 };
    for (let i = 0; i < 100; i++) {
      const x = R() * P.FIELD, y = R() * P.FIELD;
      const s = P.toScreen(view, x, y, 640, 480);
      const back = P.fromScreen(view, s.x, s.y, 640, 480);
      near(back.x, x); near(back.y, y);
    }
  }
});

test('toScreen maps cropped bounds into the canvas and rotate180 swaps corners', () => {
  const w = 200, h = 200;
  const red = { bounds: P.viewBounds('red'), rotate180: false };
  // red view is x 0..72, y 0..144: scale = min(200/72, 200/144) = 200/144, centred horizontally.
  const k = 200 / 144, ox = (200 - 72 * k) / 2;
  const a = P.toScreen(red, 0, 144, w, h);
  near(a.x, ox); near(a.y, 0);
  const b = P.toScreen(red, 72, 0, w, h);
  near(b.x, ox + 72 * k); near(b.y, 200);
  // Rotated blue view: field (144,0) (bottom-right corner of the blue half) lands at top-left after the 180 turn,
  // and field (72,144) (top-left corner of the blue half) lands at bottom-right.
  const blue = { bounds: P.viewBounds('blue'), rotate180: true };
  const c = P.toScreen(blue, 144, 0, w, h);
  near(c.x, ox); near(c.y, 0);
  const d = P.toScreen(blue, 72, 144, w, h);
  near(d.x, ox + 72 * k); near(d.y, 200);
});
