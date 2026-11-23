// Motor and per-wheel force tests (node:test).
import test from 'node:test';
import assert from 'node:assert/strict';
import { resolveParams } from '../../web/js/physics/mecanum-dynamics/params.js';
import { wheelTorque, motorCurrentTorque, smoothSign } from '../../web/js/physics/mecanum-dynamics/motor.js';
import { wheelForces, wheelLoads } from '../../web/js/physics/mecanum-dynamics/forces.js';

const P = resolveParams();
const near = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg ?? ''} expected ${b}, got ${a}`);

test('smoothSign', () => {
  near(smoothSign(0, 0.02), 0, 0);
  assert.ok(smoothSign(1, 0.02) > 0.999);
  assert.equal(smoothSign(-3, 0), -1);
});

test('motorCurrentTorque: stall and current limit', () => {
  const m = P.motor;
  const r = motorCurrentTorque(m, 12, 0);
  near(r.currentA, 9.2, 1e-9); near(r.torqueNm, m.stallTorqueNm, 1e-9);
  const lim = motorCurrentTorque(m, 12, 0, 5);
  near(lim.currentA, 5, 1e-12); near(lim.torqueNm, m.kt * 5, 1e-12);
  near(motorCurrentTorque(m, 12, m.freeSpeedRadS).currentA, 0, 1e-9);
});

test('wheelTorque: stall torque at the wheel = kt*9.2*19.2*0.85 (no friction at rest)', () => {
  const t = wheelTorque(P, 1, 0);
  near(t.torqueNm, P.motor.kt * 9.2 * 19.2 * 0.85, 1e-9);
  near(t.currentA, 9.2, 1e-9);
  near(t.volts, 12, 1e-12);
});
test('wheelTorque: power is clipped to [-1,1]', () => {
  near(wheelTorque(P, 5, 0).torqueNm, wheelTorque(P, 1, 0).torqueNm, 1e-12);
  near(wheelTorque(P, -5, 0).torqueNm, -wheelTorque(P, 1, 0).torqueNm, 1e-9);
});
test('wheelTorque: near free speed the motor torque vanishes, leaving only -Coulomb', () => {
  const rim = P.motor.nominalV / P.motor.ke / P.gearRatio * P.wheelRadiusM;
  const t = wheelTorque(P, 1, rim);
  near(t.currentA, 0, 1e-9);
  near(t.torqueNm, -P.wheelCoulombTorqueNm, 1e-9);
});
test('wheelTorque: float at power 0 -> zero current, only friction', () => {
  const f = resolveParams(null, { zeroPowerBehavior: 'float' });
  const t = wheelTorque(f, 0, 1.0);
  assert.equal(t.currentA, 0);
  assert.ok(t.torqueNm < 0 && t.torqueNm >= -f.wheelCoulombTorqueNm - 1e-12);
});
test('wheelTorque: brake at power 0 opposes motion with back-EMF current', () => {
  const fwd = wheelTorque(P, 0, 1.0), rev = wheelTorque(P, 0, -1.0);
  assert.ok(fwd.torqueNm < 0 && fwd.currentA < 0);
  assert.ok(rev.torqueNm > 0 && rev.currentA > 0);
  const fl = wheelTorque(resolveParams(null, { zeroPowerBehavior: 'float' }), 0, 1.0);
  assert.ok(fwd.torqueNm < fl.torqueNm);
});

test('wheelLoads sum to m*g', () => {
  near(wheelLoads(P).reduce((a, b) => a + b, 0), P.massKg * 9.80665, 1e-9);
});

test('wheelForces: traction clamp magnitude <= mu*N and flagged', () => {
  const f = wheelForces(P, { vx: 0, vy: 0, omega: 0 }, [100, 100, 100, 100]);
  f.perWheel.forEach((w, i) => {
    near(Math.hypot(...w.driveN), P.mu * P.normalForceN[i], 1e-9);
    assert.equal(w.tractionLimited, true);
  });
  const g = wheelForces(P, { vx: 0, vy: 0, omega: 0 }, [0.1, 0.1, 0.1, 0.1]);
  g.perWheel.forEach(w => assert.equal(w.tractionLimited, false));
});
test('wheelForces: gripScale lowers the clamp', () => {
  const low = { ...P, gripScale: 0.3 };
  const f = wheelForces(low, { vx: 0, vy: 0, omega: 0 }, [100, 100, 100, 100]);
  near(Math.hypot(...f.perWheel[0].driveN), 0.3 * P.mu * P.normalForceN[0], 1e-9);
});
test('wheelForces: drive force of 4 equal torques is pure forward, no yaw', () => {
  const tau = 0.05;
  const f = wheelForces(P, { vx: 0, vy: 0, omega: 0 }, [tau, tau, tau, tau]);
  near(f.fx, 4 * tau / P.wheelRadiusM, 1e-9);
  near(f.fy, 0, 1e-9); near(f.mz, 0, 1e-9);
});
// Roller surface speed is hub-relative: u = (c - s*xhat).d with s = cx + sigma*cy, i.e. u = sqrt(1+sigma^2)*cy.
// Round-1 tests asserted |u| = vx/sqrt2 in pure forward (u = c.d), which contradicted the spec ("zero roller
// force in pure forward") and made forward and strafe equally lossy (strafe/forward ratio exactly 1.0).
test('wheelForces: pure forward twist -> rollers do not spin, zero roller force (spec)', () => {
  const f = wheelForces(P, { vx: 1, vy: 0, omega: 0 }, [0, 0, 0, 0]);
  f.perWheel.forEach((w, i) => {
    near(w.rollerSpeedMps, 0, 1e-12, 'u' + i);
    near(w.rollerN[0], 0, 1e-12, 'rx' + i); near(w.rollerN[1], 0, 1e-12, 'ry' + i);
  });
});
test('wheelForces: pure strafe -> u = sqrt2*vy, roller force purely lateral and opposes vy', () => {
  const f = wheelForces(P, { vx: 0, vy: 1, omega: 0 }, [0, 0, 0, 0]);
  let rx = 0, ry = 0;
  f.perWheel.forEach((w, i) => {
    rx += w.rollerN[0]; ry += w.rollerN[1];
    near(w.rollerSpeedMps, Math.SQRT2, 1e-12);
    // per-wheel lateral = sqrt2 * (crrRoller*N*tanh(u/eps) + b*u)
    const u = Math.SQRT2;
    near(w.rollerN[1], -Math.SQRT2 * (P.crrRoller * P.normalForceN[i] * Math.tanh(u / P.coulombEpsMps) + P.rollerViscousNsPerM * u), 1e-9);
  });
  assert.ok(ry < 0); near(rx, 0, 1e-12);
  assert.ok(f.fy < 0);
});
test('wheelForces: roller loss power rl*u is never positive (any twist)', () => {
  for (const tw of [{ vx: 0.6, vy: -0.3, omega: 1.1 }, { vx: -0.2, vy: 0.9, omega: -2 }, { vx: 0, vy: 0, omega: 3 }]) {
    const f = wheelForces(P, tw, [0, 0, 0, 0]);
    f.perWheel.forEach((w, i) => {
      const c = { cx: tw.vx - tw.omega * P.wheels[i].y, cy: tw.vy + tw.omega * P.wheels[i].x };
      assert.ok(w.rollerN[0] * c.cx + w.rollerN[1] * c.cy <= 1e-12);
    });
  }
});
test('wheelForces: mz consistent with sum(x*Fy - y*Fx) over per-wheel forces', () => {
  const tw = { vx: 0.6, vy: -0.3, omega: 1.1 };
  const f = wheelForces(P, tw, [0.03, -0.02, 0.05, 0.01]);
  let fx = 0, fy = 0, mz = 0;
  f.perWheel.forEach((w, i) => {
    const wx = w.driveN[0] + w.rollerN[0] + w.rollingN[0], wy = w.driveN[1] + w.rollerN[1] + w.rollingN[1];
    fx += wx; fy += wy; mz += P.wheels[i].x * wy - P.wheels[i].y * wx;
  });
  near(f.fx, fx, 1e-9); near(f.fy, fy, 1e-9); near(f.mz, mz, 1e-9);
});
test('wheelForces: drive mz_i = (tau/r)*k_i (virtual work)', () => {
  const f = wheelForces(P, { vx: 0, vy: 0, omega: 0 }, [0.04, 0, 0, 0]);
  near(f.mz, (0.04 / P.wheelRadiusM) * P.wheels[0].k, 1e-9);
});
test('wheelForces: rolling resistance is zero at rest', () => {
  const f = wheelForces(P, { vx: 0, vy: 0, omega: 0 }, [0, 0, 0, 0]);
  f.perWheel.forEach(w => { assert.deepEqual(w.rollingN, [0, 0]); });
});
