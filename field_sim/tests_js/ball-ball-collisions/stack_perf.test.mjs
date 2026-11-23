import test from 'node:test';
import assert from 'node:assert/strict';
import { BB_PARAMS } from '../../web/js/physics/ball-ball-collisions/params.js';
import { resetUidCounter } from '../../web/js/physics/ball-ball-collisions/bodies.js';
import { BallBallSystem } from '../../web/js/physics/ball-ball-collisions/system.js';
import { BALL_KINDS } from '../../web/js/balls.js';
import { mkBall, runWorld } from './harness.mjs';

test('BB_PARAMS.massKg matches balls.js BALL_KINDS', () => {
  for (const k of Object.keys(BB_PARAMS.massKg)) assert.equal(BB_PARAMS.massKg[k], BALL_KINDS[k].massKg, k);
});

test('tetrahedron: no sinking, no energy gain, top ball sleeps', () => {
  resetUidCounter();
  const rN = 1.7995, rP = 1.391, d = 2 * rN;
  const base = [
    mkBall('nectar', 0, 0, rN, 0, 0, 0, { pinned: true }),
    mkBall('nectar', d, 0, rN, 0, 0, 0, { pinned: true }),
    mkBall('nectar', d / 2, d * Math.sqrt(3) / 2, rN, 0, 0, 0, { pinned: true }),
  ];
  const hz = Math.sqrt((rN + rP) ** 2 - (d / Math.sqrt(3)) ** 2);
  const top = mkBall('pollen', d / 2, d * Math.sqrt(3) / 6, rN + hz);
  const balls = [...base, top];
  const res = runWorld(balls, { system: new BallBallSystem(), seconds: 3, floor: false });
  const ke0 = 0; // starts at rest
  assert.ok(Math.max(...res.penTrace.slice(30)) < 3 * BB_PARAMS.linearSlop, `pen ${Math.max(...res.penTrace)}`);
  for (let i = 30; i < res.keTrace.length; i++) assert.ok(res.keTrace[i] <= ke0 + 1e-9, `KE ${res.keTrace[i]} at ${i}`);
  assert.ok(top.sleeping, 'top ball should sleep');
});

test('dropped ball never bounces above 20*eMax^2 + 0.1 in', () => {
  resetUidCounter();
  const base = mkBall('pollen', 0, 0, 0, 0, 0, 0, { pinned: true });
  const ball = mkBall('pollen', 0, 0, 2 * 1.391 + 20);
  let impacted = false, maxH = 0;
  runWorld([base, ball], {
    system: new BallBallSystem(), seconds: 3, floor: false,
    onStep() { const h = ball.z - 2 * 1.391; if (h < 0.5) impacted = true; else if (impacted && h > maxH) maxH = h; },
  });
  assert.ok(impacted, 'never landed');
  assert.ok(maxH <= 20 * BB_PARAMS.eMax ** 2 + 0.1, `bounce ${maxH}`);
});

function makePile() {
  resetUidCounter();
  const balls = [];
  for (let k = 0; balls.length < 150; k++) {
    for (let i = 0; i < 7 && balls.length < 150; i++) for (let j = 0; j < 7 && balls.length < 150; j++) {
      const n = balls.length;
      balls.push(mkBall(n % 3 === 0 ? 'nectar' : 'pollen', 3.5 + 3.9 * i + 0.1 * ((n * 7) % 5), 3.5 + 3.9 * j + 0.1 * ((n * 3) % 5), 3 + 4 * k));
    }
  }
  return balls;
}
const runPile = () => runWorld(makePile(), { system: new BallBallSystem(), seconds: 3, box: { minX: 0, maxX: 30, minY: 0, maxY: 30 } });

test('150-ball pile in a 30x30 in box: penetration, floor, sleeping, wall time', () => {
  const res = runPile();
  const worstPen = Math.max(...res.penTrace);
  assert.ok(worstPen < 0.15, `max penetration ${worstPen}`);
  for (const b of res.balls) assert.ok(b.z >= b.r - 0.05, `ball below floor ${b.z}`);
  const asleep = res.balls.filter((b) => b.sleeping).length;
  assert.ok(asleep >= 0.8 * res.balls.length, `only ${asleep}/150 asleep`);
  assert.ok(res.wallMs < 4000, `wall ${res.wallMs} ms`);
});

test('determinism: two identical runs give bitwise-equal positions', () => {
  const a = runPile().balls, b = runPile().balls;
  for (let i = 0; i < a.length; i++) {
    assert.ok(Object.is(a[i].x, b[i].x) && Object.is(a[i].y, b[i].y) && Object.is(a[i].z, b[i].z), `ball ${i}`);
  }
});

test('floor plane solved in-solver holds a 3-ball column with no external clamp', () => {
  resetUidCounter();
  const R = 1.391; // in, pollen radius
  const sys = new BallBallSystem(BB_PARAMS, { planes: [{ nx: 0, ny: 0, nz: 1, d: 0 }] });
  const col = [0, 1, 2].map((k) => mkBall('pollen', 0, 0, R + 2 * R * k + 0.5 * k));
  // solverPlanes:false keeps the planes given to the constructor; floor:false removes the harness clamp
  const res = runWorld(col, { system: sys, seconds: 2, floor: false, solverPlanes: false });
  for (const b of col) assert.ok(b.z >= b.r - 3 * BB_PARAMS.linearSlop, `ball sank into plane z=${b.z}`);
  assert.ok(Math.max(...res.penTrace.slice(60)) < 3 * BB_PARAMS.linearSlop, `pen ${Math.max(...res.penTrace.slice(60))}`);
});

test('pile perf: ms per 240 Hz substep with 150 balls awake (solver only)', () => {
  const balls = makePile();
  const sys = new BallBallSystem();
  // first 0.5 s: everything is falling or colliding, nothing can sleep yet (timeToSleep 0.5 s)
  let ms = 0, k = 0;
  const orig = sys.substep.bind(sys);
  sys.substep = (l, h) => { const r = orig(l, h); ms += sys.lastSubstepMs; k++; return r; };
  runWorld(balls, { system: sys, seconds: 0.5, box: { minX: 0, maxX: 30, minY: 0, maxY: 30 } });
  const per = ms / k;
  // budget: 2 ms per substep = 8 ms per 60 Hz frame at 4 substeps (half a 16.7 ms frame). ASSUMED budget.
  assert.ok(per < 2, `${per.toFixed(3)} ms per substep`);
  console.log(`# pile perf ${per.toFixed(3)} ms/substep over ${k} substeps`);
});
