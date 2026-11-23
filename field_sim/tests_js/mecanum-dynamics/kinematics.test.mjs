// Kinematics + params tests for mecanum-dynamics (node:test). Frame: x fwd, y left, omega ccw+.
import test from 'node:test';
import assert from 'node:assert/strict';
import {
  inverseKinematics, forwardKinematics, desaturate, commandToWheelPowers, maxSpeedAtHeading, twistIsFeasible,
} from '../../web/js/physics/mecanum-dynamics/kinematics.js';
import { resolveParams, DEFAULTS } from '../../web/js/physics/mecanum-dynamics/params.js';

const P = resolveParams();
const W = P.wheels;
const near = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg ?? ''} expected ${b}, got ${a}`);
const nearArr = (a, b, tol = 1e-9) => { assert.equal(a.length, b.length); a.forEach((v, i) => near(v, b[i], tol, `[${i}]`)); };

test('IK: unit vx -> all wheels +1', () => nearArr(inverseKinematics(W, { vx: 1, vy: 0, omega: 0 }), [1, 1, 1, 1]));
test('IK: unit vy (left) -> [-1,1,1,-1]', () => nearArr(inverseKinematics(W, { vx: 0, vy: 1, omega: 0 }), [-1, 1, 1, -1]));
test('IK: unit omega -> k_i with |k| = 0.2893 m', () => {
  const s = inverseKinematics(W, { vx: 0, vy: 0, omega: 1 });
  s.forEach((v, i) => { near(Math.abs(v), 0.2893, 1e-4, 'abs'); near(v, W[i].k, 1e-12, 'k'); });
  assert.ok(s[0] < 0 && s[1] > 0 && s[2] < 0 && s[3] > 0);
});
test('IK: diagonal vx=vy=1 -> [0,2,2,0]', () => nearArr(inverseKinematics(W, { vx: 1, vy: 1, omega: 0 }), [0, 2, 2, 0]));
test('IK: optional out array is reused', () => {
  const out = new Array(4);
  assert.equal(inverseKinematics(W, { vx: 1, vy: 0, omega: 0 }, out), out);
});

test('FK(IK(t)) == t', () => {
  const t = { vx: 0.7, vy: -0.4, omega: 1.3 };
  const r = forwardKinematics(W, inverseKinematics(W, t));
  near(r.vx, t.vx, 1e-9); near(r.vy, t.vy, 1e-9); near(r.omega, t.omega, 1e-9);
});

test('FK least squares for inconsistent speeds equals the pseudo-inverse (closed form, symmetric layout)', () => {
  const s = [1, 0.2, -0.3, 0.9];
  const r = forwardKinematics(W, s);
  const k2 = W.reduce((a, w) => a + w.k * w.k, 0);
  near(r.vx, s.reduce((a, v) => a + v, 0) / 4, 1e-9, 'vx');
  near(r.vy, s.reduce((a, v, i) => a + W[i].sigma * v, 0) / 4, 1e-9, 'vy');
  near(r.omega, s.reduce((a, v, i) => a + W[i].k * v, 0) / k2, 1e-9, 'omega');
  // normal-equation optimality: A^T (A q - s) = 0
  const res = inverseKinematics(W, r).map((v, i) => v - s[i]);
  const g = [0, 0, 0];
  W.forEach((w, i) => { g[0] += res[i]; g[1] += w.sigma * res[i]; g[2] += w.k * res[i]; });
  g.forEach(v => near(v, 0, 1e-9, 'gradient'));
});

test('desaturate (max): keeps ratios, max == 1, scale reported', () => {
  const r = desaturate([2, 1, -0.5, 0], 1, 'max');
  nearArr(r.values, [1, 0.5, -0.25, 0]);
  near(r.scale, 0.5, 1e-12);
  const ok = desaturate([0.5, -0.2, 0, 0.1], 1, 'max');
  nearArr(ok.values, [0.5, -0.2, 0, 0.1]);
  near(ok.scale, 1, 1e-12);
});
test('desaturate (gm0): divides by max(1, max_i(sum |terms|))', () => {
  const terms = [[1, 1, 1], [1, 0.5, 0], [0.2, 0.2, 0.2], [0, 0, 0]];
  const r = desaturate([3, 1.5, 0.6, 0], 1, 'gm0', terms);
  near(r.scale, 1 / 3, 1e-12);
  nearArr(r.values, [1, 0.5, 0.2, 0]);
  near(desaturate([0.1, 0, 0, 0], 1, 'gm0', [[0.1, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]).scale, 1, 1e-12);
});

test('commandToWheelPowers: turn=1 alone gives |p|=1 on every wheel', () => {
  const r = commandToWheelPowers(W, { fwd: 0, strafe: 0, turn: 1 }, { kMax: P.kMax });
  r.powers.forEach(p => near(Math.abs(p), 1, 1e-12));
  near(r.scale, 1, 1e-12);
});
test('commandToWheelPowers: full fwd+strafe+turn desaturates to max 1, ratios kept', () => {
  const raw = [1 - 1 - 1, 1 + 1 + 1, 1 + 1 - 1, 1 - 1 + 1];   // [-1, 3, 1, 1]
  const r = commandToWheelPowers(W, { fwd: 1, strafe: 1, turn: 1 }, { kMax: P.kMax });
  near(Math.max(...r.powers.map(Math.abs)), 1, 1e-12);
  r.powers.forEach((p, i) => near(p, raw[i] / 3, 1e-12));
  near(r.scale, 1 / 3, 1e-12);
});
test('commandToWheelPowers: lateralMultiplier scales strafe only', () => {
  const r = commandToWheelPowers(W, { fwd: 0, strafe: 0.5, turn: 0 }, { lateralMultiplier: 1.1, kMax: P.kMax });
  nearArr(r.powers, [-0.55, 0.55, 0.55, -0.55], 1e-12);
});

test('maxSpeedAtHeading: S at 0 and 90 deg, S/sqrt2 at 45 deg, decreases with |omega|', () => {
  const S = 1.7;
  near(maxSpeedAtHeading(W, 0, 0, S), S, 1e-9);
  near(maxSpeedAtHeading(W, Math.PI / 2, 0, S), S, 1e-9);
  near(maxSpeedAtHeading(W, Math.PI / 4, 0, S), S / Math.SQRT2, 1e-9);
  const a = maxSpeedAtHeading(W, 0, 1, S), b = maxSpeedAtHeading(W, 0, 3, S);
  assert.ok(a < S && b < a, `${S} > ${a} > ${b}`);
  assert.equal(maxSpeedAtHeading(W, 0, 100, S), 0);   // clamped at 0
});
test('twistIsFeasible', () => {
  assert.equal(twistIsFeasible(W, { vx: 1, vy: 0, omega: 0 }, 1), true);
  assert.equal(twistIsFeasible(W, { vx: 1, vy: 1, omega: 0 }, 1), false);   // rim speed 2
  assert.equal(twistIsFeasible(W, { vx: 0.5, vy: 0.5, omega: 0 }, 1), true);
});

// ------------------------------------------------------------------ params
test('params: default mass, sigma signs, yaw inertia, kMax', () => {
  near(P.massKg, 31.4 * 0.45359237, 1e-9);
  nearArr(W.map(w => w.sigma), [-1, 1, 1, -1], 1e-12);   // cot(45 deg) is 1 only to ~2e-16
  assert.deepEqual(W.map(w => w.name), ['FL', 'FR', 'BL', 'BR']);
  near(P.yawInertiaKgM2, 0.496, 1e-2);
  near(P.kMax, 0.2893, 1e-4);
  near(P.gripScale, 1, 0);
  near(P.normalForceN.reduce((a, b) => a + b, 0), P.massKg * 9.80665, 1e-9);
  assert.ok(Object.isFrozen(P) && Object.isFrozen(P.motor) && Object.isFrozen(P.wheels));
});
test('params: derived motor constants', () => {
  near(P.motor.resistanceOhm, 12 / 9.2, 1e-9);
  near(P.motor.kt, P.motor.stallTorqueNm / 9.2, 1e-12);
  near(P.motor.ke, 12 / P.motor.freeSpeedRadS, 1e-12);
  near(P.motor.stallTorqueNm, 0.14899, 1e-4);
  near(P.motor.freeSpeedRadS, 626.2, 0.1);
});
test('params: throws on bad enums and non-physical values', () => {
  assert.throws(() => resolveParams(null, { zeroPowerBehavior: 'coast' }), /zeroPowerBehavior/);
  assert.throws(() => resolveParams(null, { desaturateMode: 'nope' }));
  assert.throws(() => resolveParams(null, { massLb: -1 }));
  assert.throws(() => resolveParams(null, { mu: NaN }));
  assert.throws(() => resolveParams(null, { normalSplit: [0.4, 0.4, 0.4, 0.4] }));
  assert.throws(() => resolveParams(null, { wheelsIn: [{ name: 'A', x: 0, y: 1 }, ...DEFAULTS.wheelsIn.slice(1)] }));
});
test('params: precedence DEFAULTS < spec < spec.drive.dynamics < overrides', () => {
  const spec = { chassis: { mass_lb: 20 }, drive: { dynamics: { massLb: 25, mu: 0.5 } } };
  near(resolveParams(spec).massKg, 25 * 0.45359237, 1e-9);                     // dynamics beats spec
  near(resolveParams({ chassis: { mass_lb: 20 } }).massKg, 20 * 0.45359237, 1e-9);   // spec beats defaults
  const o = resolveParams(spec, { massLb: 30 });
  near(o.massKg, 30 * 0.45359237, 1e-9);                                       // overrides beat dynamics
  assert.equal(o.mu, 0.5);                                                       // untouched dynamics key survives
});
