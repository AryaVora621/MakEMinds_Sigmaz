import test, { beforeEach } from 'node:test';
import assert from 'node:assert/strict';
import { resetBodyIds, World, softCoefficients } from '../../web/js/physics/rigid-body-solver/index.js';
import { runFor, makeWall, makeRobot, penetrationOf, ROBOT_HALF_M as H } from './helpers.mjs';

beforeEach(() => resetBodyIds());

test('softCoefficients(30, 10, 0.004) matches the spec derived example', () => {
  const s = softCoefficients(30, 10, 0.004);
  assert.ok(Math.abs(s.biasRate - 9.08) < 1e-2);
  assert.ok(Math.abs(s.massScale - 0.94) < 1e-2);
  assert.ok(Math.abs(s.impulseScale - 0.06) < 1e-2);
});

test('softCoefficients with hertz 0 is all zero', () => {
  assert.deepEqual(softCoefficients(0, 10, 0.004), { biasRate: 0, massScale: 0, impulseScale: 0 });
});

// Wall face at x = 0.5 (centre 1, half 0.5); robot starts near it moving +x.
function bounce(speed, e) {
  const w = new World();
  makeWall(w, 1, 0, 0.5, 5, { restitution: e });
  const r = makeRobot(w, { x: 0.2, y: 0, drive: false, vx: speed, restitution: e });
  runFor(w, 0.4);
  return r.linearVelocity.x;
}

test('restitution 0.5 at 2 m/s rebounds at 0.75 to 1.15 m/s', () => {
  const vx = bounce(2, 0.5);
  assert.ok(vx < 0, 'moving away from the wall');
  assert.ok(-vx > 0.75 && -vx < 1.15, 'rebound speed ' + -vx);
});

test('impact below the restitution threshold (0.5 m/s) does not bounce', () => {
  const vx = bounce(0.5, 0.5);
  assert.ok(Math.abs(vx) < 0.1, 'speed after ' + vx);
});

for (const mode of ['soft', 'baumgarte', 'ngs']) {
  test(`3 m/s into a 0.02 m thick wall never tunnels (${mode})`, () => {
    const w = new World({ solver: { positionMode: mode } });
    makeWall(w, 1, 0, 0.01, 2);
    const r = makeRobot(w, { x: 0, y: 0, drive: false, vx: 3 });
    let maxX = -Infinity;
    runFor(w, 1, 1 / 60, () => { maxX = Math.max(maxX, r.position.x); });
    assert.ok(maxX < 1, 'centre crossed the wall centre, max x = ' + maxX);
  });
}

test('hitting an offset static box corner spins the body', () => {
  const w = new World();
  // Box spans y 0.15..0.55, so only the top 0.08 m of the robot face overlaps: off-centre impact.
  makeWall(w, 1, 0.35, 0.2, 0.2);
  const r = makeRobot(w, { x: 0, y: 0, drive: false, vx: 1.5 });
  let peak = 0;
  runFor(w, 0.5, 1 / 60, () => { peak = Math.max(peak, Math.abs(r.angularVelocity)); });
  assert.ok(peak > 0.1, 'peak |w| = ' + peak);
});

function slideAlongWall(friction) {
  const w = new World();
  makeWall(w, 1, 0, 0.5, 5, { friction });
  const r = makeRobot(w, { x: 0.2, y: 0, drive: false, vx: 1, vy: 2, friction });
  runFor(w, 0.5);
  return r.linearVelocity.y;
}

test('friction slows tangential sliding by at least 0.05 m/s versus friction 0', () => {
  const withFriction = (() => {
    const w = new World();
    makeWall(w, 1, 0, 0.5, 5); // default obstacle material
    const r = makeRobot(w, { x: 0.2, y: 0, drive: false, vx: 1, vy: 2 }); // default robot material
    runFor(w, 0.5);
    return r.linearVelocity.y;
  })();
  const frictionless = slideAlongWall(0);
  assert.ok(frictionless - withFriction >= 0.05, `frictionless ${frictionless} vs default ${withFriction}`);
});

function cornerScenario(mode, warmStarting = true) {
  const w = new World({ solver: { positionMode: mode, warmStarting } });
  makeWall(w, 1, 0, 0.5, 5); // face at x = 0.5
  makeWall(w, 0, 1, 5, 0.5); // face at y = 0.5
  const r = makeRobot(w, { x: 0.1, y: 0.1, target: { vx: 1, vy: 1, om: 0 } });
  const xs = [], ys = [];
  let maxPen = 0;
  runFor(w, 3, 1 / 60, (i, t) => {
    if (t > 2) { xs.push(r.position.x); ys.push(r.position.y); }
    if (t > 1) maxPen = Math.max(maxPen, penetrationOf(w, r));
  });
  const range = (a) => Math.max(...a) - Math.min(...a);
  return { jitter: Math.max(range(xs), range(ys)), maxPen };
}

for (const mode of ['soft', 'baumgarte', 'ngs']) {
  test(`robot driven into a corner rests without jitter and with small penetration (${mode})`, () => {
    const { jitter, maxPen } = cornerScenario(mode);
    assert.ok(jitter < 0.002, 'jitter ' + jitter + ' m');
    assert.ok(maxPen < 0.012, 'penetration ' + maxPen + ' m');
  });
}

test('two identical worlds produce bit-identical poses', () => {
  const run = () => {
    resetBodyIds();
    const w = new World();
    makeWall(w, 1, 0, 0.5, 5);
    const a = makeRobot(w, { x: 0.1, y: 0.1, angle: 0.3, target: { vx: 1, vy: 0.5, om: 0.4 } });
    const b = makeRobot(w, { x: 0.1, y: 0.8, angle: -0.2, target: { vx: 0.8, vy: -0.3, om: 0 } });
    runFor(w, 2);
    return JSON.stringify([a.position, a.angle, a.linearVelocity, b.position, b.angle, b.linearVelocity]);
  };
  assert.equal(run(), run());
});
