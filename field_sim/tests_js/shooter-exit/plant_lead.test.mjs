// plant_lead.test.mjs - SlewServo, ShooterPlantV2, lead.js (spec sections 4 and 5)
import test from 'node:test';
import assert from 'node:assert/strict';
import { SlewServo, ShooterPlantV2, turretServoSpec, hoodServoSpec } from '../../web/js/physics/shooter-exit/servo_plant.js';
import { releaseLeadTime, predictPose, turretFeedforward } from '../../web/js/physics/shooter-exit/lead.js';
import { transferFactor } from '../../web/js/physics/shooter-exit/exit_model.js';
import { PARAMS } from '../../web/js/physics/shooter-exit/params.js';

const base = { minDeg: -180, maxDeg: 180, maxRateDegS: 450, maxAccelDegS2: 3000 };
const DT = 0.001;

function run(servo, cmd, T, dt = DT) {
  const out = [];
  for (let t = 0; t < T; t += dt) { const r = servo.step(dt, typeof cmd === 'function' ? cmd(t) : cmd); out.push({ t: t + dt, ...r }); }
  return out;
}

test('latency delays the response by latencyS (+/- dt)', () => {
  const lat = 0.1;
  const s = new SlewServo({ ...base, latencyS: lat });
  const tr = run(s, 30, 0.3);
  const first = tr.find((r) => Math.abs(r.pos) > 1e-9);
  assert.ok(first, 'moves eventually');
  assert.ok(Math.abs(first.t - lat) <= 2 * DT + 1e-9, `first move at ${first.t}`);
});

test('no overshoot on a step', () => {
  const s = new SlewServo({ ...base, backlashDeg: 0 });
  const tr = run(s, 90, 1);
  const peak = Math.max(...tr.map((r) => r.pos));
  assert.ok(peak <= 90 + 1e-9, `peak ${peak}`);
  assert.ok(Math.abs(tr[tr.length - 1].pos - 90) < 1e-9);
});

test('peak rate <= maxRate and accel bound holds', () => {
  const s = new SlewServo({ ...base });
  const tr = run(s, (t) => (t < 0.5 ? 120 : -60), 1.2);
  let prev = 0, peakV = 0;
  for (const r of tr) {
    peakV = Math.max(peakV, Math.abs(r.vel));
    // the spec's snap-to-target (vel = 0 on crossing) is exempt; it only drops a residual <= 2*a*dt
    if (r.vel === 0 && prev !== 0) assert.ok(Math.abs(prev) <= 2 * 3000 * DT + 1e-9, `snap residual ${prev}`);
    else assert.ok(Math.abs(r.vel - prev) / DT <= 3000 + 1e-6, `accel ${(r.vel - prev) / DT}`);
    prev = r.vel;
  }
  assert.ok(peakV <= 450 + 1e-9, `peak rate ${peakV}`);
  assert.ok(peakV > 300, 'actually cruised fast');
});

test('deadband ignores a command change < db/2, accepts larger', () => {
  const s = new SlewServo({ ...base, deadbandDeg: 2, backlashDeg: 0 });
  run(s, 10, 0.5);
  assert.ok(Math.abs(s.pos - 10) < 1e-9);
  run(s, 10.9, 0.3);
  assert.ok(Math.abs(s.pos - 10) < 1e-9, 'moved by <db/2');
  run(s, 11.5, 0.3);
  assert.ok(Math.abs(s.pos - 11.5) < 1e-9, 'moved by >db/2');
});

test('backlash: output lags by b after a direction reversal', () => {
  const b = 1;
  const s = new SlewServo({ ...base, backlashDeg: b });
  run(s, 20, 0.5);
  const upOffset = s._inner - s.pos;   // out trails inner by b/2 going up
  assert.ok(Math.abs(upOffset - b / 2) < 1e-9);
  run(s, 10, 0.5);
  const downOffset = s._inner - s.pos; // out leads inner by b/2 going down
  assert.ok(Math.abs(downOffset + b / 2) < 1e-9);
  assert.ok(Math.abs(upOffset - downOffset - b) < 1e-9);
  // right after reversing the output must not move until the gap is taken up
  const s2 = new SlewServo({ ...base, backlashDeg: b });
  run(s2, 20, 0.5);
  const p0 = s2.pos;
  const r = s2.step(DT, 0);
  assert.equal(r.pos, p0);
});

test('90 deg step settles near the analytic trapezoid time', () => {
  const s = new SlewServo({ ...base, deadbandDeg: 0.5, backlashDeg: 0.5, latencyS: 0.02 });
  const tr = run(s, 90, 1);
  const hit = tr.find((r) => Math.abs(r.pos - 90) <= 0.5);
  const analytic = 90 / 450 + 450 / 3000 + 0.02;
  assert.ok(Math.abs(hit.t - analytic) <= 0.05, `reach ${hit.t} vs ${analytic}`);
  const settledAt = tr.find((r) => r.settled);
  assert.ok(settledAt && settledAt.t < analytic + 0.15, `settled ${settledAt && settledAt.t}`);
});

test('quantization, clamp and atLimit', () => {
  const s = new SlewServo({ ...base, quantDeg: 0.15, minDeg: -10, maxDeg: 10, backlashDeg: 0 });
  let r;
  for (let i = 0; i < 1000; i++) r = s.step(DT, 50);
  assert.equal(r.atLimit, 'max');
  assert.ok(Math.abs(r.pos - 10) < 1e-9);
  const s2 = new SlewServo({ ...base, quantDeg: 0.5, backlashDeg: 0 });
  run(s2, 1.1, 0.5);
  assert.ok(Math.abs(s2.pos - 1.0) < 1e-9);
});

test('servo specs follow PARAMS', () => {
  const h = hoodServoSpec({ hoodMin: 25, hoodMax: 65 });
  assert.ok(Math.abs(h.maxRateDegS - 300 * 0.7 / PARAMS.hood.LINK_RATIO) < 1e-12);
  assert.ok(Math.abs(h.deadbandDeg - 0.6 / PARAMS.hood.LINK_RATIO) < 1e-12);
  assert.ok(Math.abs(h.quantDeg - 0.15 / PARAMS.hood.LINK_RATIO) < 1e-12);
  const t = turretServoSpec({ turretMinDeg: -127.5, turretMaxDeg: 127.5 });
  assert.equal(t.maxRateDegS, PARAMS.turretServo.MAX_RATE_DEG_S);
  assert.equal(t.minDeg, -127.5);
});

test('ShooterPlantV2 API shape, negative/NaN dt ignored, atSetpoint', () => {
  const p = new ShooterPlantV2({});
  for (const k of ['turretDeg', 'hoodDeg', 'rpm', 'turretRateDegS', 'hoodRateDegS']) assert.ok(k in p.state);
  const before = { ...p.state };
  p.step(-1, { turretDeg: 50, rpm: 3000 });
  p.step(NaN, { turretDeg: 50, rpm: 3000 });
  assert.deepEqual(p.state, before);
  let r;
  for (let i = 0; i < 2000; i++) r = p.step(0.002, { turretDeg: 40, hoodDeg: 45, rpm: 3000 });
  assert.ok('atTurretLimit' in r && 'atHoodLimit' in r);
  const sp = p.atSetpoint({ turretDeg: 40, hoodDeg: 45, rpm: 3000 });
  assert.deepEqual(sp, { turret: true, hood: true, rpm: true });
  assert.ok(p.atSetpoint({ turretDeg: 80 }).turret === false);
});

test('ShooterPlantV2 physical kick lowers rpm by the factor s; legacy by rpmDipFrac', () => {
  const p = new ShooterPlantV2({ exitModel: 'physical', kind: 'pollen' });
  p.state.rpm = 4000;
  const s = transferFactor('pollen', 4000, { model: 'physical' });
  p.kick('pollen');
  assert.ok(Math.abs(p.state.rpm - 4000 * s) < 1e-6 * 4000, `${p.state.rpm} vs ${4000 * s}`);
  assert.ok(p.state.rpm < 4000);
  const q = new ShooterPlantV2({});
  q.state.rpm = 4000;
  q.kick();
  assert.ok(Math.abs(q.state.rpm - 4000 * (1 - PARAMS.flywheel.RPM_DIP_FRAC_LEGACY)) < 1e-9);
});

test('releaseLeadTime sums components', () => {
  assert.ok(Math.abs(releaseLeadTime() - (PARAMS.timing.CMD_LATENCY_S + PARAMS.timing.FEED_DELAY_S)) < 1e-15);
  assert.ok(Math.abs(releaseLeadTime({ cmdLatencyS: 0.1, feedDelayS: 0.2, contactTimeS: 0.01 }) - 0.31) < 1e-15);
});

test('predictPose: om = 0 is linear (and accel)', () => {
  const r = predictPose({ x: 10, y: 20, h: 0.3 }, { vx: 30, vy: -10, om: 0 }, 0.5);
  assert.deepEqual([r.x, r.y, r.h, r.vx, r.vy], [25, 15, 0.3, 30, -10]);
  const a = predictPose({ x: 0, y: 0, h: 0 }, { vx: 10, vy: 0, om: 0 }, 2, { ax: 4, ay: -2 });
  assert.ok(Math.abs(a.x - (20 + 8)) < 1e-12 && Math.abs(a.y + 4) < 1e-12);
  assert.ok(Math.abs(a.vx - 18) < 1e-12 && Math.abs(a.vy + 4) < 1e-12);
});

test('predictPose: arc matches 10000-step Euler integration within 1e-4 in', () => {
  const pose = { x: 5, y: -3, h: 0.2 }, vel = { vx: 40, vy: 15, om: 2.5 }, T = 0.8;
  let x = pose.x, y = pose.y, vx = vel.vx, vy = vel.vy;
  const n = 10000, dt = T / n;
  for (let i = 0; i < n; i++) {
    const th = vel.om * dt;
    // midpoint rotation of the velocity keeps Euler error O(dt^2)
    const c = Math.cos(vel.om * dt / 2), s = Math.sin(vel.om * dt / 2);
    x += (vx * c - vy * s) * dt; y += (vx * s + vy * c) * dt;
    const c2 = Math.cos(th), s2 = Math.sin(th);
    [vx, vy] = [vx * c2 - vy * s2, vx * s2 + vy * c2];
  }
  const r = predictPose(pose, vel, T);
  assert.ok(Math.abs(r.x - x) < 1e-4 && Math.abs(r.y - y) < 1e-4, `${r.x - x}, ${r.y - y}`);
  assert.ok(Math.abs(r.h - (pose.h + vel.om * T)) < 1e-12);
  assert.ok(Math.abs(Math.hypot(r.vx, r.vy) - Math.hypot(vel.vx, vel.vy)) < 1e-9);
  assert.ok(Math.abs(r.vx - vx) < 1e-6 && Math.abs(r.vy - vy) < 1e-6);
});

test('turretFeedforward holds field aim', () => {
  const f = turretFeedforward(37, Math.PI / 2);
  assert.equal(f.turretDeg, 37);
  assert.ok(Math.abs(f.turretRateDegS + 90) < 1e-12);
  assert.equal(turretFeedforward(5, 0).turretRateDegS, 0);
});
