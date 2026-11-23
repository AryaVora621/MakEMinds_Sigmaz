// Mecanum plant (spec section 3). Replays the Pedro tuner procedures (Tuning.java) on the plant.
import test from 'node:test';
import assert from 'node:assert/strict';
import { MecanumPlant } from '../../web/js/physics/pedro-fidelity/plant.js';
import { resolveParams } from '../../web/js/physics/pedro-fidelity/params.js';

const P = resolveParams();
const DT = P.loop.dt;   // s

function run(plant, powers, seconds) { plant.setPowers(powers); for (let i = 0; i < Math.round(seconds / DT); i++) plant.step(DT); }

test('tuner replay: forward velocity (mean |u| over the last 10 loops at full power) within 0.5% of xVelocity', () => {
  const pl = new MecanumPlant(P); pl.setPowers([1, 1, 1, 1]);
  const us = [];
  for (let i = 0; i < 400; i++) { pl.step(DT); if (i >= 390) us.push(Math.abs(pl.robotVelocity.u)); }
  const mean = us.reduce((a, b) => a + b, 0) / us.length;
  assert.ok(Math.abs(mean - P.mecanum.xVelocity) / P.mecanum.xVelocity < 0.005, `${mean} vs ${P.mecanum.xVelocity}`);
});

test('tuner replay: lateral velocity within 0.5% of yVelocity', () => {
  const pl = new MecanumPlant(P); pl.setPowers([-1, 1, 1, -1]);   // cy = +1, cx = cw = 0
  const e = pl.efforts(); assert.ok(Math.abs(e.cy - 1) < 1e-9 && Math.abs(e.cx) < 1e-9 && Math.abs(e.cw) < 1e-9, JSON.stringify(e));
  const vs = [];
  for (let i = 0; i < 400; i++) { pl.step(DT); if (i >= 390) vs.push(Math.abs(pl.robotVelocity.s)); }
  const mean = vs.reduce((a, b) => a + b, 0) / vs.length;
  assert.ok(Math.abs(mean - P.mecanum.yVelocity) / P.mecanum.yVelocity < 0.005, `${mean} vs ${P.mecanum.yVelocity}`);
});

// Pedro ForwardZeroPowerAccelerationTuner (Tuning.java:345-): coast from 30 in/s down to velocityConstraint, mean of per-loop dv/dt.
function zpaTuner(powersOn, axis) {
  const pl = new MecanumPlant(P); pl.setPowers(powersOn);
  for (let i = 0; i < 300; i++) pl.step(DT);
  pl.setPowers([0, 0, 0, 0]);
  const vOf = () => Math.abs(pl.robotVelocity[axis]);
  while (vOf() > 30) pl.step(DT);
  const acc = []; let prev = pl.robotVelocity[axis]; const v0 = Math.abs(prev);
  while (vOf() > P.constraints.velocityConstraint) {
    pl.step(DT); const v = pl.robotVelocity[axis]; acc.push((v - prev) / DT); prev = v;
    if (acc.length > 5000) break;
  }
  return { mean: acc.reduce((a, b) => a + b, 0) / acc.length, acc, v0, vEnd: Math.abs(prev) };
}

// Threshold note (round 2). The old check was |tuner mean - ZPA| < 1% of ZPA. That bound is not a property of the plant:
// the plant's FLOAT decel is exactly F (every non-final loop sample equals -F to ~1e-12), but the tuner's LAST sample is
// the loop in which v reaches 0 part-way (or the loop stops early at vEnd <= velocityConstraint), so
// mean = (v0 - vEnd)/(N*DT) with N = ceil((v0 - velocityConstraint)/(F*DT)) or so, and the tuner under-reads F by
// a fraction in [0, 1/N). With the real 2.1.0 defaults (defaults(): lateral -59.7819) v0 = 29.5611 in/s,
// F*DT = 0.597819 in/s, v0/(F*DT) = 49.45 -> N = 50, mean = 29.5611/0.50 = 59.1222 in/s^2, i.e. 1.10% low, while the
// analytic bound is 1/N = 2.0%. (Old field-initializer value -78.15554 happened to land at 0.74%; forward -41.278 at
// 0.000%.) The test now checks the stronger exact statements: every non-final sample is -F, and the mean equals the
// analytic discrete-tuner value (v0 - vEnd)/(N*DT), plus the |error| < F/N bound. The real Pedro tuner has the same bias.
function checkZpa(powers, axis, want) {
  const { mean, acc, v0, vEnd } = zpaTuner(powers, axis);
  const F = Math.abs(want), N = acc.length;
  for (let i = 0; i < N - 1; i++) assert.ok(Math.abs(Math.abs(acc[i]) - F) <= 1e-9 * F, `sample ${i}: ${acc[i]} vs -${F}`);
  const analytic = (v0 - vEnd) / (N * DT);   // telescoping sum of the per-loop dv/dt samples
  assert.ok(Math.abs(Math.abs(mean) - analytic) <= 1e-9 * F, `mean ${mean} vs analytic ${analytic}`);
  assert.ok(Math.abs(Math.abs(mean) - F) < F / N, `${mean} vs ${want} (bound F/N = ${F / N})`);
  assert.ok(mean < 0, 'coast decel is signed negative');
}

test('Pedro forward ZPA tuner on plant coast samples reproduces forwardZeroPowerAcceleration (exact samples, discrete-tuner bound)', () => {
  checkZpa([1, 1, 1, 1], 'u', P.pedro.forwardZeroPowerAcceleration);
});

test('Pedro lateral ZPA tuner reproduces |lateralZeroPowerAcceleration| (exact samples, discrete-tuner bound)', () => {
  checkZpa([-1, 1, 1, -1], 's', P.pedro.lateralZeroPowerAcceleration);
});

test('FLOAT coast never reverses: velocity lands on exactly 0', () => {
  const pl = new MecanumPlant(P); run(pl, [1, 1, 1, 1], 1); run(pl, [0, 0, 0, 0], 4);
  assert.equal(pl.velocity.vx, 0); assert.equal(pl.velocity.vy, 0); assert.equal(pl.velocity.w, 0);
});

test('traction clamp: full power from rest slips; a gentle command does not', () => {
  const a = new MecanumPlant(P); a.setPowers([1, 1, 1, 1]); a.step(DT);
  assert.equal(a.slipping, true);
  const b = new MecanumPlant(P); b.setPowers([0.2, 0.2, 0.2, 0.2]); b.step(DT);
  assert.equal(b.slipping, false);
  // acceleration never exceeds aTractionMax (in/s^2) by more than numerical noise
  const c = new MecanumPlant(P); c.setPowers([1, 1, 1, 1]); c.step(DT);
  assert.ok(Math.hypot(c.velocity.vx, c.velocity.vy) / DT <= P.plant.aTractionMax * 1.001);
});

test('injected voltageSource scales the drive (kV = V / plantNominalV)', () => {
  const half = new MecanumPlant(P, { x: 0, y: 0, h: 0 }, { voltageSource: () => P.plant.plantNominalV / 2 });
  run(half, [0.5, 0.5, 0.5, 0.5], 4);
  const want = 0.5 * 0.5 * P.plant.vFreeX - P.plant.coulombX * P.plant.tauX;   // kV*c*vFree - F*tau
  assert.ok(Math.abs(half.robotVelocity.u - want) / want < 0.005, `${half.robotVelocity.u} vs ${want}`);
});

test('syncTo overwrites pose and velocity; heading integrates for a turn command', () => {
  const pl = new MecanumPlant(P); pl.syncTo({ x: 5, y: 6, h: 1 }, { vx: 1, vy: 2, w: 0 });
  assert.deepEqual(pl.pose, { x: 5, y: 6, h: 1 }); assert.deepEqual(pl.velocity, { vx: 1, vy: 2, w: 0 });
  run(pl, [-0.5, -0.5, 0.5, 0.5], 1);   // cw > 0
  assert.ok(pl.velocity.w > 0);
});
