// Pedro 2.1.0 geometry / heading interpolators / bridge-request grouping. Units: in, rad (bridge bodies use degrees).
import test from 'node:test';
import assert from 'node:assert/strict';
import { BezierCurve, BezierLine, Path, PathChain, HI, groupsFromBridgeRequest } from '../../web/js/physics/pedro-fidelity/pedropath.js';

const near = (a, b, tol = 1e-9, msg = '') => assert.ok(Math.abs(a - b) <= tol, `${msg} ${a} vs ${b}`);
const dist = (a, b) => Math.hypot(a.x - b.x, a.y - b.y);

test('Newton closest point matches brute force (distance within 1e-3 in)', () => {
  const c = new BezierCurve([{ x: 0, y: 0 }, { x: 20, y: 60 }, { x: 70, y: 60 }, { x: 90, y: 0 }]);
  const queries = [{ x: 10, y: 10 }, { x: 45, y: 50 }, { x: 45, y: 20 }, { x: 80, y: 10 }, { x: 100, y: -5 }, { x: -10, y: 0 }, { x: 45, y: 100 }];
  for (const q of queries) {
    let best = Infinity;
    for (let i = 0; i <= 20000; i++) best = Math.min(best, dist(c.pose(i / 20000), q));
    const t = c.closestT(q, 20, 0.5);
    const d = dist(c.pose(t), q);
    assert.ok(d - best < 1e-3, `q=(${q.x},${q.y}) newton ${d} brute ${best}`);
  }
});

test('curvature is positive for a left (CCW) turn, negative for a right turn, zero for a line', () => {
  const left = new BezierCurve([{ x: 0, y: 0 }, { x: 40, y: 0 }, { x: 40, y: 40 }]);
  const right = new BezierCurve([{ x: 0, y: 0 }, { x: 40, y: 0 }, { x: 40, y: -40 }]);
  assert.ok(left.curvature(0.5) > 0, `left ${left.curvature(0.5)}`);
  assert.ok(right.curvature(0.5) < 0, `right ${right.curvature(0.5)}`);
  near(new BezierLine({ x: 0, y: 0 }, { x: 10, y: 0 }).curvature(0.5), 0);
});

test('curve length approximates the analytic value (quadratic: straight control points)', () => {
  near(new BezierCurve([{ x: 0, y: 0 }, { x: 5, y: 0 }, { x: 10, y: 0 }]).length, 10, 1e-6);
  near(new BezierLine({ x: 0, y: 0 }, { x: 3, y: 4 }).length, 5, 1e-9);
});

test('linear heading interpolates in PARAMETRIC t with endT', () => {
  const f = HI.linear(0, Math.PI / 2, 0.5);
  near(f({ tValue: 0.25 }), Math.PI / 4);
  near(f({ tValue: 0.5 }), Math.PI / 2);
  near(f({ tValue: 0.9 }), Math.PI / 2);
  // through the Path API
  const p = new Path(new BezierLine({ x: 0, y: 0 }, { x: 10, y: 0 }));
  p.setLinearHeading(0, Math.PI / 2, 1.0);
  near(p.headingGoalAt(0.5), Math.PI / 4, 1e-9);
});

test('linear heading takes the short way; reversedLinear the long way', () => {
  const short = HI.linear(0.1, 2 * Math.PI - 0.1, 1);
  near(short({ tValue: 1 }), 2 * Math.PI - 0.1);
  near(short({ tValue: 0.5 }), 0);   // passes through 0 going CW
  const long = HI.reversedLinear(0, Math.PI / 2, 1);
  near(long({ tValue: 1 }), Math.PI / 2);
  near(long({ tValue: 0.5 }), (0 + (-1.5 * Math.PI) * 0.5 + 4 * Math.PI) % (2 * Math.PI), 1e-9);   // -3PI/4 normalised
});

const body = () => ({
  start: { x: 10, y: 10, h: 0 },
  segments: [
    { type: 'line', points: [{ x: 10, y: 10 }, { x: 99, y: 99 }, { x: 40, y: 10 }], heading: { mode: 'constant', start: 0 }, waitAfter: 0.5 },
    { type: 'curve', points: [{ x: 60, y: 30 }], heading: { mode: 'tangent' }, callbacks: [{ t: 0.5, name: 'mid' }] },
    { type: 'line', points: [{ x: 60, y: 30 }, { x: 80, y: 30 }], heading: { mode: 'linear', start: 0, end: 90 }, callbacks: [{ ms: 200, name: 'tmr' }] },
  ],
});

test('groupsFromBridgeRequest: waitAfter splits groups; line uses first/last points; 1-point segment prepends previous end', () => {
  const g = groupsFromBridgeRequest(body());
  assert.equal(g.length, 2);
  assert.equal(g[0].chain.size(), 1); assert.equal(g[0].waitAfter, 0.5); assert.equal(g[0].firstSeg, 0);
  assert.equal(g[1].chain.size(), 2); assert.equal(g[1].firstSeg, 1); assert.equal(g[1].waitAfter, 0);
  // 'line' ignores the middle point (99,99): length is |(40,10)-(10,10)| = 30
  near(g[0].chain.getPath(0).length, 30, 1e-9);
  // 1-point curve segment starts at the previous end (40,10)
  const p1 = g[1].chain.getPath(0);
  near(p1.curve.firstPoint.x, 40); near(p1.curve.firstPoint.y, 10);
  near(p1.lastControlPoint().x, 60); near(p1.lastControlPoint().y, 30);
});

test('groupsFromBridgeRequest: callbacks carry the in-group pathIndex and the global seg', () => {
  const g = groupsFromBridgeRequest(body());
  assert.deepEqual(g[1].callbacks.map(c => [c.name, c.pathIndex, c.seg]), [['mid', 0, 1], ['tmr', 1, 2]]);
  assert.equal(g[1].callbacks[0].t, 0.5); assert.equal(g[1].callbacks[1].ms, 200);
});

test('heading propagation: a tangent segment keeps the previous heading for the next one', () => {
  const b = { start: { x: 0, y: 0, h: 30 }, segments: [
    { type: 'line', points: [{ x: 0, y: 0 }, { x: 10, y: 0 }], heading: { mode: 'tangent' } },
    { type: 'line', points: [{ x: 10, y: 0 }, { x: 20, y: 0 }], heading: { mode: 'linear', end: 60 } },   // start omitted -> previous (30)
  ] };
  const g = groupsFromBridgeRequest(b)[0];
  near(g.chain.getPath(1).headingGoalAt(0), Math.PI / 6, 1e-9);
  near(g.chain.getPath(1).headingGoalAt(1), Math.PI / 3, 1e-9);
});

test('PathChain defaults to LAST_PATH deceleration', () => {
  const c = new PathChain([new Path(new BezierLine({ x: 0, y: 0 }, { x: 10, y: 0 }))]);
  assert.equal(c.decelerationType, 'LAST_PATH');
});
