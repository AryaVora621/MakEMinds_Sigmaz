// Rigid-body dynamics tests (node:test). Windows come from the mecanum-dynamics SPEC section 6.
import test from 'node:test';
import assert from 'node:assert/strict';
import { resolveParams } from '../../web/js/physics/mecanum-dynamics/params.js';
import {
  buildMassMatrix, solve3, stepTwist, derivative, applyImpulse, impulseAtPoint, integratePose,
} from '../../web/js/physics/mecanum-dynamics/dynamics.js';
import { MecanumDynamics } from '../../web/js/physics/mecanum-dynamics/index.js';

const IN = 0.0254;
const near = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg ?? ''} expected ${b} +-${tol}, got ${a}`);
const inRange = (v, lo, hi, msg) => assert.ok(v >= lo && v <= hi, `${msg ?? ''} ${v} not in [${lo}, ${hi}]`);

const P = resolveParams();
const M = buildMassMatrix(P);
const sim = new MecanumDynamics();
const top = (cmd) => sim.steadyState(cmd, 4, 0.01);

test('mass matrix: diagonal, m_eff = 14.24 + 2.33 kg, yaw term', () => {
  near(M[0], 16.57, 0.05); near(M[4], M[0], 1e-9);
  const Jr2 = P.wheelSpinInertiaKgM2 / (P.wheelRadiusM ** 2);
  near(M[0], P.massKg + 4 * Jr2, 1e-9);
  near(M[8], P.yawInertiaKgM2 + 4 * Jr2 * (0.2893 ** 2), 1e-3);
  for (const i of [1, 2, 3, 5, 6, 7]) near(M[i], 0, 1e-9, 'offdiag ' + i);
});
test('solve3 solves and throws on singular', () => {
  const x = solve3(new Float64Array([2, 0, 0, 0, 3, 0, 0, 0, 4]), [2, 3, 4]);
  near(x[0], 1, 1e-12); near(x[1], 1, 1e-12); near(x[2], 1, 1e-12);
  const A = new Float64Array([0, 2, 1, 1, 0, 3, 4, 5, 0]);
  const b = solve3(A, [3, 4, 9]);   // pivoting needed
  near(A[0] * b[0] + A[1] * b[1] + A[2] * b[2], 3, 1e-9);
  assert.throws(() => solve3(new Float64Array([1, 2, 3, 2, 4, 6, 0, 0, 1]), [1, 1, 1]));
});

test('forward top speed 55-66 in/s', () => inRange(top({ fwd: 1, strafe: 0, turn: 0 }).vfInS, 55, 66));
test('strafe/forward ratio 0.80-0.97', () => {
  const r = top({ fwd: 0, strafe: 1, turn: 0 }).vlInS / top({ fwd: 1, strafe: 0, turn: 0 }).vfInS;
  inRange(r, 0.80, 0.97);
});
test('diagonal (45 deg, magnitude 1)/forward 0.60-0.80', () => {
  const k = Math.SQRT1_2, d = top({ fwd: k, strafe: k, turn: 0 });
  inRange(Math.hypot(d.vfInS, d.vlInS) / top({ fwd: 1, strafe: 0, turn: 0 }).vfInS, 0.60, 0.80);
});
test('pure spin top rate 3.5-6.5 rad/s', () => inRange(top({ fwd: 0, strafe: 0, turn: 1 }).omRadS, 3.5, 6.5));

function launch(overrides = {}, cmd = { fwd: 1, strafe: 0, turn: 0 }, seconds = 0.02) {
  const d = new MecanumDynamics({ overrides });
  const st = { x: 0, y: 0, h: 0, bodyV: [0, 0, 0] };
  const n = Math.round(seconds / 0.002);
  for (let i = 0; i < n; i++) d.step(st, cmd, 0.002);
  return { st, d, t: n * 0.002 };
}
test('initial forward accel 140-190 in/s^2 and <= mu*g/sqrt2*m/m_eff*1.02', () => {
  const { st, t } = launch();
  const a = st.bodyV[0] / t;                              // in/s^2
  inRange(a, 140, 190);
  const cap = P.mu * 9.80665 / Math.SQRT2 * (P.massKg / M[0]) / IN;
  assert.ok(a <= cap * 1.02, `accel ${a} exceeds traction cap ${cap}`);
});
test('slip (gripScale 0.3) lowers initial accel', () => {
  const full = launch().st.bodyV[0];
  const d = new MecanumDynamics(); d.setGripScale(0.3);
  const st = { h: 0, bodyV: [0, 0, 0] };
  for (let i = 0; i < 10; i++) d.step(st, { fwd: 1, strafe: 0, turn: 0 }, 0.002);
  assert.ok(st.bodyV[0] < full * 0.6, `${st.bodyV[0]} vs ${full}`);
});

function coast(overrides, axis, seconds = 0.05) {
  const d = new MecanumDynamics({ overrides });
  const base = new MecanumDynamics();
  const v0 = axis === 0 ? base.steadyState({ fwd: 1, strafe: 0, turn: 0 }, 4).vfInS
                        : base.steadyState({ fwd: 0, strafe: 1, turn: 0 }, 4).vlInS;
  const st = { h: 0, bodyV: [0, 0, 0] }; st.bodyV[axis] = v0;
  const n = Math.round(seconds / 0.005);
  for (let i = 0; i < n; i++) d.step(st, { fwd: 0, strafe: 0, turn: 0 }, 0.005);
  return (v0 - st.bodyV[axis]) / seconds;
}
test('float coast decel: forward 15-45 in/s^2, lateral > 1.5x forward', () => {
  const f = coast({ zeroPowerBehavior: 'float' }, 0), l = coast({ zeroPowerBehavior: 'float' }, 1);
  inRange(f, 15, 45, 'forward float'); assert.ok(l / f > 1.5, `lateral ${l} / forward ${f}`);
});
test('brake decel > 3x float decel', () => {
  assert.ok(coast({ zeroPowerBehavior: 'brake' }, 0) > 3 * coast({ zeroPowerBehavior: 'float' }, 0));
});

test('zero-loss float spin+translate keeps |v| within 1% over 1 s', () => {
  const lossless = { kS: 0, crrBase: 0, crrRoller: 0, rollerViscousNsPerM: 0, zeroPowerBehavior: 'float' };
  const p = resolveParams(null, lossless), m = buildMassMatrix(p);
  let tw = { vx: 0.5, vy: 0.3, omega: 2 };
  const v0 = Math.hypot(tw.vx, tw.vy);
  for (let i = 0; i < 10; i++) tw = stepTwist(p, m, tw, [0, 0, 0, 0], 0.1).twist;
  near(Math.hypot(tw.vx, tw.vy) / v0, 1, 0.01);
  near(tw.omega, 2, 1e-9);
});
test('kinetic energy never increases in float with zero power', () => {
  const p = resolveParams(null, { zeroPowerBehavior: 'float' }), m = buildMassMatrix(p);
  const E = t => 0.5 * (m[0] * t.vx * t.vx + m[4] * t.vy * t.vy + m[8] * t.omega * t.omega);
  let tw = { vx: 0.8, vy: 0.3, omega: 1.5 }, e = E(tw);
  for (let i = 0; i < 300; i++) {
    tw = stepTwist(p, m, tw, [0, 0, 0, 0], 0.005).twist;
    const e2 = E(tw); assert.ok(e2 <= e + 1e-9, `step ${i}: ${e2} > ${e}`); e = e2;
  }
});
test('one stepTwist(0.1) is within 2% of ten steps of 0.01', () => {
  const pw = [0.8, 0.8, 0.8, 0.8], t0 = { vx: 0.2, vy: 0, omega: 0 };
  const a = stepTwist(P, M, t0, pw, 0.1).twist;
  let b = t0; for (let i = 0; i < 10; i++) b = stepTwist(P, M, b, pw, 0.01).twist;
  near(a.vx, b.vx, Math.abs(b.vx) * 0.02);
});
test('stepTwist: dt<=0 / NaN returns a copy with telemetry; inputs not mutated', () => {
  const t0 = { vx: 0.3, vy: 0, omega: 0 }, pw = [1, 1, 1, 1];
  for (const dt of [0, -1, NaN]) {
    const r = stepTwist(P, M, t0, pw, dt);
    assert.deepEqual(r.twist, t0); assert.notEqual(r.twist, t0); assert.equal(r.telemetry.length, 4);
  }
  assert.deepEqual(pw, [1, 1, 1, 1]); assert.deepEqual(t0, { vx: 0.3, vy: 0, omega: 0 });
});
test('derivative returns 4 telemetry rows with the documented keys', () => {
  const r = derivative(P, M, { vx: 0.5, vy: 0, omega: 0 }, [1, 1, 1, 1]);
  assert.equal(r.telemetry.length, 4);
  for (const k of ['power', 'rimSpeedMps', 'torqueNm', 'currentA', 'driveN', 'rollerN', 'rollingN', 'rollerSpeedMps', 'tractionLimited'])
    assert.ok(k in r.telemetry[0], k);
  assert.ok(r.ax > 0); near(r.ay, 0, 1e-6); near(r.alpha, 0, 1e-6);
});

test('applyImpulse gives dv = J/M[0] and dOmega = Jz/M[8]', () => {
  const r = applyImpulse(P, M, { vx: 0, vy: 0, omega: 0 }, { jx: 3, jy: -2, jz: 0.5 });
  near(r.vx, 3 / M[0], 1e-9); near(r.vy, -2 / M[4], 1e-9); near(r.omega, 0.5 / M[8], 1e-9);
});
test('impulseAtPoint: jz = px*jy - py*jx', () => {
  assert.deepEqual(impulseAtPoint(0.2, 0.1, 5, 7), { jx: 5, jy: 7, jz: 0.2 * 7 - 0.1 * 5 });
});
test('applyWorldImpulse rotates into body frame', () => {
  const d = new MecanumDynamics(), st = { h: Math.PI / 2, bodyV: [0, 0, 0] };
  d.applyWorldImpulse(st, 0, 10);   // world +y at h=90deg is body forward
  near(st.bodyV[0], 10 / d.M[0] / IN, 1e-6); near(st.bodyV[1], 0, 1e-6);
});

test('integratePose: pure forward twist at h=90 deg moves +y', () => {
  const r = integratePose({ x: 1, y: 2, h: Math.PI / 2 }, { vx: 3, vy: 0, omega: 0 }, 0.5);
  near(r.x, 1, 1e-12); near(r.y, 3.5, 1e-12); near(r.h, Math.PI / 2, 1e-12);
});
test('integratePose: constant omega traces a circle of radius v/omega', () => {
  const v = 2, om = 1, R = v / om;
  const r = integratePose({ x: 0, y: 0, h: 0 }, { vx: v, vy: 0, omega: om }, Math.PI / 2);
  near(r.x, R, 1e-9); near(r.y, R, 1e-9); near(r.h, Math.PI / 2, 1e-9);
  const full = integratePose({ x: 0, y: 0, h: 0 }, { vx: v, vy: 0, omega: om }, 2 * Math.PI);
  near(full.x, 0, 1e-9); near(full.y, 0, 1e-9);
  near(integratePose({ x: 0, y: 0, h: 3 }, { vx: 0, vy: 0, omega: 1 }, 1).h, 4 - 2 * Math.PI, 1e-9);   // wrapped
});
test('integratePose: tiny omega*dt uses straight line (no NaN)', () => {
  const r = integratePose({ x: 0, y: 0, h: 0 }, { vx: 1, vy: 0, omega: 1e-12 }, 1);
  near(r.x, 1, 1e-9); near(r.y, 0, 1e-9);
});
