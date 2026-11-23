// Mecanum calculateDrive (Pedro ftc Mecanum.java) and the drivetrain power cache. Wheel order LF, LR, RF, RR.
import test from 'node:test';
import assert from 'node:assert/strict';
import { calculateDrive, mecanumWheelVectors, MecanumDrivetrain } from '../../web/js/physics/pedro-fidelity/mecanumdrive.js';
import { MecanumPlant } from '../../web/js/physics/pedro-fidelity/plant.js';
import { resolveParams } from '../../web/js/physics/pedro-fidelity/params.js';
import { vPolar } from '../../web/js/physics/pedro-fidelity/pedromath.js';

const near = (a, b, tol = 1e-9, msg = '') => assert.ok(Math.abs(a - b) <= tol, `${msg} ${a} vs ${b}`);
const P = resolveParams();
const vectors = mecanumWheelVectors(P.mecanum.xVelocity, P.mecanum.yVelocity);
const Z = { x: 0, y: 0 };
const opts = { maxPowerScaling: 1, vectors };
const efforts = (p) => { const pl = new MecanumPlant(P); pl.setPowers(p); return pl.efforts(); };

test('wheel vectors: LF = RR, LR = RF, unit length', () => {
  assert.equal(vectors.length, 4);
  for (const v of vectors) near(Math.hypot(v.x, v.y), 1, 1e-12);
  near(vectors[0].x, vectors[3].x); near(vectors[0].y, vectors[3].y);
  near(vectors[1].x, vectors[2].x); near(vectors[1].y, vectors[2].y);
});

test('pathing (1,0) at heading 0: all wheel powers = 1 and cx = 1', () => {
  const p = calculateDrive(Z, Z, { x: 1, y: 0 }, 0, opts);
  p.forEach((x, i) => near(x, 1, 1e-9, 'wheel ' + i));
  const e = efforts(p); near(e.cx, 1, 1e-9); near(e.cy, 0, 1e-9); near(e.cw, 0, 1e-9);
});

test('pathing (0,1) at heading 0: strafes left, cy = +1', () => {
  const p = calculateDrive(Z, Z, { x: 0, y: 1 }, 0, opts);
  const e = efforts(p);
  assert.ok(e.cy > 0.5, `cy ${e.cy}`); near(e.cx, 0, 1e-9);
  assert.ok(Math.max(...p.map(Math.abs)) <= 1 + 1e-12);
});

test('positive heading vector gives positive (CCW) turning effort', () => {
  for (const h of [0, Math.PI / 3, 2.5]) {
    const p = calculateDrive(Z, vPolar(0.5, h), Z, h, opts);
    const e = efforts(p);
    assert.ok(e.cw > 0.1, `heading ${h} cw ${e.cw}`);
    near(e.cx, 0, 1e-9); near(e.cy, 0, 1e-9);
  }
});

test('output is scaled so no wheel exceeds maxPowerScaling', () => {
  const p = calculateDrive({ x: 0.9, y: 0 }, { x: 0.9, y: 0 }, { x: 1, y: 0 }, 0, { ...opts, maxPowerScaling: 0.5 });
  assert.ok(Math.max(...p.map(Math.abs)) <= 0.5 + 1e-12);
});

test('MecanumDrivetrain caches writes below motorCachingThreshold, but always writes an exact zero', () => {
  // Pedro quirk: calculateDrive gain is ~1.28 per unit path vector (clamped by the max-power scaling), so compare against the first write.
  const d = new MecanumDrivetrain(P);
  d.runDrive(Z, Z, { x: 0.5, y: 0 }, 0);
  const first = d.powers[0]; assert.ok(first > 0.4 && first < 0.8, `first ${first}`);
  d.runDrive(Z, Z, { x: 0.505, y: 0 }, 0);                       // |delta| ~0.006 < 0.01: not rewritten
  d.powers.forEach(x => near(x, first, 1e-12));
  d.runDrive(Z, Z, { x: 0.52, y: 0 }, 0);                        // |delta| ~0.026 > 0.01: written
  d.powers.forEach(x => assert.ok(x > first + 0.01));
  d.runDrive(Z, Z, Z, 0);                                        // exact zero while last != 0
  d.powers.forEach(x => assert.ok(x === 0, "power " + x));   // -0 === 0
  d.runDrive(Z, Z, { x: 0.3, y: 0 }, 0); d.breakFollowing();
  d.powers.forEach(x => assert.ok(x === 0, "power " + x));   // -0 === 0
});

test('setMaxPowerScaling clamps the drivetrain output', () => {
  const d = new MecanumDrivetrain(P); d.setMaxPowerScaling(0.4);
  assert.equal(d.getMaxPowerScaling(), 0.4);
  d.runDrive(Z, Z, { x: 1, y: 0 }, 0);
  d.powers.forEach(x => near(x, 0.4, 1e-9));   // 1.28 gain saturates, so the scale-down lands exactly on the cap
});
