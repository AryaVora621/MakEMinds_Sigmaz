import test from 'node:test';
import assert from 'node:assert/strict';
import * as P from '../web/js/pathing.js';

const near = (a, b, e = 1e-9) => assert.ok(Math.abs(a - b) <= e, `${a} vs ${b}`);

test('default starts are inside their alliance view', () => {
  for (const a of ['red', 'blue']) {
    const s = P.defaultStart(a);
    assert.ok(P.inView(P.viewBounds(a), s.x, s.y), `${a} default ${JSON.stringify(s)} outside view`);
    const r = P.newRoute({ alliance: a, start: s });
    assert.equal(P.startInView(r), true);
  }
});

test('red and blue defaults are mirror images', () => {
  const r = P.defaultStart('red'), b = P.defaultStart('blue');
  near(b.x, P.FIELD - r.x); near(b.y, r.y);
  near(P.wrapAngle(Math.PI - r.h), b.h);
});

test('startInView is false for a start in the other half', () => {
  const r = P.newRoute({ alliance: 'red', start: { x: 120, y: 24, h: 0 } });
  assert.equal(P.startInView(r), false);
});

test('retargetStart mirrors an out-of-view start and sets alliance', () => {
  const r = P.newRoute({ alliance: 'blue', start: { x: 24, y: 24, h: Math.PI / 2 } });
  P.retargetStart(r, 'red');
  assert.equal(r.alliance, 'red');
  assert.equal(P.startInView(r), true);
  near(r.start.x, 24); // 24 is already in red's half, so it should stay
});

test('retargetStart mirrors x and heading when needed', () => {
  const r = P.newRoute({ alliance: 'red', start: { x: 24, y: 24, h: 0.3 } });
  P.retargetStart(r, 'blue');
  near(r.start.x, P.FIELD - 24);
  near(r.start.h, P.wrapAngle(Math.PI - 0.3));
  assert.equal(r.alliance, 'blue');
});

test('retargetStart is idempotent', () => {
  const r = P.newRoute({ alliance: 'red', start: { x: 30, y: 40, h: 1.1 } });
  P.retargetStart(r, 'blue');
  const once = JSON.stringify(r);
  P.retargetStart(r, 'blue');
  assert.equal(JSON.stringify(r), once);
  P.retargetStart(r, 'red');
  const back = JSON.stringify(r);
  P.retargetStart(r, 'red');
  assert.equal(JSON.stringify(r), back);
});

test('retargetStart leaves segments untouched', () => {
  const r = P.newRoute({ alliance: 'red', start: { x: 24, y: 24, h: 0 } });
  P.addLine(r, 30, 60);
  const before = JSON.stringify(r.segments);
  P.retargetStart(r, 'blue');
  assert.equal(JSON.stringify(r.segments), before);
});
