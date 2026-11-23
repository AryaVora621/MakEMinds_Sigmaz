// Tests for timing.js and adapter.js (flywheel-dynamics). The fake ShooterPlant is local; aimcontrol.js is not imported.
import test from 'node:test';
import assert from 'node:assert/strict';
import {
  spinUpTimeS, recoveryTimeS, coastDownTimeS, burstSim, reachableRpm,
} from '../../web/js/physics/flywheel-dynamics/timing.js';
import { wrapShooterPlant, FlywheelShooterAdapter, M_TO_IN } from '../../web/js/physics/flywheel-dynamics/adapter.js';

const near = (a, b, rel, msg) => assert.ok(Math.abs(a - b) <= rel * Math.abs(b), `${msg || ''} got ${a} want ${b} (rel ${rel})`);

test('spinUpTimeS is finite and monotone in the target', () => {
  const t2 = spinUpTimeS({}, 0, 2000);
  const t3 = spinUpTimeS({}, 0, 3000);
  const t4 = spinUpTimeS({}, 0, 4000);
  for (const t of [t2, t3, t4]) assert.ok(Number.isFinite(t) && t > 0);
  assert.ok(t2 < t3 && t3 < t4, `${t2} ${t3} ${t4}`);
  assert.ok(t4 >= 0.3 && t4 <= 1.5, `t4 ${t4}`);
});

test('spinUpTimeS is 0-ish when already at target and Infinity beyond the physical limit', () => {
  const t = spinUpTimeS({}, 4000, 4000);
  assert.ok(Number.isFinite(t) && t < 0.2);
  assert.equal(spinUpTimeS({}, 0, 7000), Infinity);
});

test('reachableRpm at 12 V is in [5900, 6000]', () => {
  const r = reachableRpm({});
  assert.ok(r >= 5900 && r <= 6000, `reachable ${r}`);
  assert.ok(reachableRpm({ battery: { internalResOhm: 0, voltageV: 10 } }) < r);
});

test('recoveryTimeS reports a pollen shot at 4000 rpm', () => {
  const r = recoveryTimeS({}, 4000, 'pollen');
  assert.ok(r.dipFrac > 0.04 && r.dipFrac < 0.10, `dip ${r.dipFrac}`);
  assert.ok(r.recoveryS > 0 && r.recoveryS < 0.3, `recovery ${r.recoveryS}`);
  assert.ok(r.minRpm < 4000 && r.minRpm > 3500);
  assert.ok(r.shot.exitSpeedMps > 0);
});

test('coastDownTimeS: float is slow, brake is fast', () => {
  assert.ok(coastDownTimeS({}, 4000, 2000) > 2);
  assert.ok(coastDownTimeS({ losses: { zeroPowerBehavior: 'brake' } }, 4000, 2000) < 1);
});

test('burstSim with 3 pollen balls at 0.15 s keeps rpmAtFire >= 90% of target', () => {
  const b = burstSim({}, 4000, ['pollen', 'pollen', 'pollen'], 0.15);
  assert.equal(b.shots.length, 3);
  for (const s of b.shots) {
    assert.ok(s.rpmAtFire >= 3600, `rpmAtFire ${s.rpmAtFire}`);
    assert.ok(s.exitSpeedMps > 0 && s.dipFrac > 0);
  }
  near(b.shots[1].tS - b.shots[0].tS, 0.15, 0.02);
  assert.ok(b.trace.length > 10);
  assert.ok(b.shots[2].rpmAtFire <= b.shots[0].rpmAtFire + 1); // can only have lost speed
});

// ---- adapter ----
class FakeShooterPlant {
  constructor() { this.state = { rpm: 0, turretDeg: 0, hoodDeg: 20 }; this.kicks = 0; }
  step(dt, cmd) {
    const c = cmd || {};
    if (Number.isFinite(c.turret)) this.state.turretDeg += c.turret * dt;
    if (Number.isFinite(c.hood)) this.state.hoodDeg += (c.hood - this.state.hoodDeg) * Math.min(1, 5 * dt);
    if (Number.isFinite(c.rpm)) this.state.rpm += (c.rpm - this.state.rpm) * Math.min(1, dt / 0.3); // legacy hack
  }
  kick() { this.kicks++; this.state.rpm *= 0.94; return null; }
}

test('wrapShooterPlant leaves turret and hood identical to an unwrapped plant', () => {
  const a = new FakeShooterPlant();
  const b = wrapShooterPlant(new FakeShooterPlant(), {});
  assert.ok(b.flywheel);
  for (let i = 0; i < 500; i++) {
    const cmd = { turret: 10 * Math.sin(i * 0.01), hood: 30 + 5 * Math.cos(i * 0.02), rpm: 4000 };
    a.step(0.004, { ...cmd });
    b.step(0.004, { ...cmd });
    assert.equal(b.state.turretDeg, a.state.turretDeg);
    assert.equal(b.state.hoodDeg, a.state.hoodDeg);
  }
});

test('wrapped plant: rpm follows the flywheel model, kick lowers rpm, null command holds target', () => {
  const p = wrapShooterPlant(new FakeShooterPlant(), {});
  for (let i = 0; i < 500; i++) p.step(0.004, { rpm: 4000 });
  assert.ok(p.state.rpm > 3500 && p.state.rpm < 4100, `rpm ${p.state.rpm}`);
  assert.equal(p.state.rpm, p.flywheel.state.rpm);
  assert.equal(p.flywheel.state.targetRpm, 4000);
  for (let i = 0; i < 250; i++) p.step(0.004, { rpm: null }); // hold target
  assert.equal(p.flywheel.state.targetRpm, 4000);
  assert.ok(p.state.rpm > 3900);
  const before = p.state.rpm;
  const shot = p.kick('pollen');
  // the ball load is spread over the contact time, so the dip appears over the next few steps
  let minRpm = p.state.rpm;
  for (let i = 0; i < 10; i++) { p.step(0.004, { rpm: null }); minRpm = Math.min(minRpm, p.state.rpm); }
  assert.ok(minRpm < before - 50, `min ${minRpm} before ${before}`);
  assert.ok(shot.exitSpeedMps > 0);
  assert.equal(p.lastShot, shot);
  assert.equal(p.state.rpm, p.flywheel.state.rpm);
  assert.doesNotThrow(() => p.step(0.004, undefined));
});

test('FlywheelShooterAdapter standalone', () => {
  const a = new FlywheelShooterAdapter();
  let rpm = 0;
  for (let i = 0; i < 500; i++) rpm = a.step(0.004, 3000);
  assert.equal(rpm, a.rpm);
  assert.ok(rpm > 2800 && rpm < 3100, `rpm ${rpm}`);
  const shot = a.kick('nectar');
  let minRpm = a.rpm;
  for (let i = 0; i < 10; i++) { a.step(0.004, null); minRpm = Math.min(minRpm, a.rpm); }
  assert.ok(minRpm < rpm - 50, `min ${minRpm} before ${rpm}`);
  near(a.exitSpeedInS(shot), shot.exitSpeedMps * M_TO_IN, 1e-9);
  near(M_TO_IN, 1 / 0.0254, 1e-12);
  a.step(0.004, null);
  assert.equal(a.flywheel.state.targetRpm, 3000);
});
