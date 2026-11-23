import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { WeightTransferSim, G, makeParams } from '../../web/js/physics/weight-transfer/index.js';

const spec = JSON.parse(fs.readFileSync(new URL('../../web/data/robot_spec.json', import.meta.url), 'utf8'));
const near = (a, b, e = 1e-6) => assert.ok(Math.abs(a - b) < e, `${a} !~ ${b} (tol ${e})`);

test('re-exports G and makeParams', () => {
  near(G, 386.0886, 1e-3);
  assert.equal(typeof makeParams, 'function');
});

test('60 in/s head-on wall: no tip, peak load above static, pitch kicked', () => {
  const sim = new WeightTransferSim(spec);
  const r = sim.onCollision({ normals: [[-1, 0]], vBefore: { vx: 60, vy: 0 }, h: 0 });
  near(r.dvBody.x, 60, 1e-9); near(r.dvBody.y, 0, 1e-9);
  // 60 in/s is below vCrit (~100 in/s): a small rock may start but must not fall over
  const staticMax = Math.max(...sim.snapshot.loads.lbf);
  assert.ok(r.impactPeakLbf > staticMax, `${r.impactPeakLbf} <= ${staticMax}`);
  assert.notEqual(sim.compliance.state.pitchRate, 0);
  for (let i = 0; i < 180; i++) {
    sim.step(1 / 60, [0, 0, 0]);
    assert.notEqual(sim.snapshot.tip.mode, 'fallen');
  }
  assert.equal(sim.snapshot.tip.mode, 'grounded');
  assert.equal(sim.snapshot.disabled, false);
});

test('collision heading rotates the impact into the body frame', () => {
  const sim = new WeightTransferSim(spec);
  // heading 90 deg: body +x is world +y; wall normal points -y (into robot), robot moving +y
  const r = sim.onCollision({ normals: [[0, -1]], vBefore: { vx: 0, vy: 40 }, h: Math.PI / 2 });
  near(r.dvBody.x, 40, 1e-9); near(r.dvBody.y, 0, 1e-9);
});

test('limitDrive never raises accel above the input and converts to world', () => {
  const sim = new WeightTransferSim(spec);
  const dt = 1 / 60;
  const prev = [0, 0, 0];
  for (const want of [[-60, 0, 0], [60, 0, 0], [0, 60, 0], [30, -30, 5], [1, 0, 0]]) {
    const r = sim.limitDrive(prev, want, dt, 0.7);
    for (let i = 0; i < 3; i++) {
      assert.ok(Math.abs(r.bodyV[i] - prev[i]) <= Math.abs(want[i] - prev[i]) + 1e-9);
    }
    assert.ok(r.scale >= 0 && r.scale <= 1);
    near(Math.hypot(r.vx, r.vy), Math.hypot(r.bodyV[0], r.bodyV[1]), 1e-9);
    near(r.vx, Math.cos(0.7) * r.bodyV[0] - Math.sin(0.7) * r.bodyV[1], 1e-9);
    near(r.vy, Math.sin(0.7) * r.bodyV[0] + Math.cos(0.7) * r.bodyV[1], 1e-9);
  }
  const hard = sim.limitDrive([0, 0, 0], [-60, 0, 0], dt, 0);
  assert.ok(hard.scale < 1 && hard.slip[2]);
});

test('visualMatrix at rest: rotation ~ identity, z translation = -zSink', () => {
  const sim = new WeightTransferSim(spec);
  const m = sim.visualMatrix();
  assert.equal(m.length, 16);
  const I = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0];
  for (let i = 0; i < 12; i++) near(m[i], I[i], 1e-9);
  const zSink = (sim.cg.massLb * G) / (4 * sim.params.compliance.kWheelLbfPerIn * G);
  near(m[12], 0, 1e-9); near(m[13], 0, 1e-9); near(m[14], -zSink, 1e-9); near(m[15], 1, 1e-12);
});

test('braking pitches the visual matrix nose down (x axis z-component < 0)', () => {
  const sim = new WeightTransferSim(spec);
  let v = 100;
  for (let i = 0; i < 60; i++) { v = Math.max(0, v - 140 / 60); sim.step(1 / 60, [v, 0, 0]); }
  const m = sim.visualMatrix();
  assert.ok(m[2] < 0, `m[2]=${m[2]}`);
});

test('setMechState changes cg and mass', () => {
  const sim = new WeightTransferSim(spec);
  const m0 = sim.cg.massLb;
  sim.setMechState({ ballsHeld: 4 });
  near(sim.cg.massLb - m0, 4 * sim.params.mass.ballMassLb, 1e-9);
  const x0 = sim.cg.x;
  sim.setMechState({ intakeDeployed: 1 });
  assert.ok(sim.cg.x > x0);
});

test('pathLimits returns load-aware limits', () => {
  const sim = new WeightTransferSim(spec);
  const L = sim.pathLimits();
  assert.ok(L.maxBrake < L.maxAccelFwd);
});

test('braking step reports snapshot fields and load transfer', () => {
  const sim = new WeightTransferSim(spec);
  sim.step(1 / 60, [0, 0, 0]);
  const s = sim.step(1 / 60, [-4, 0, 0]);
  for (const k of ['cg', 'accel', 'loads', 'scale', 'slip', 'pitch', 'roll', 'heave', 'tip', 'disabled', 'impactPeakLbf']) assert.ok(k in s, k);
  assert.ok(s.accel.ax < 0);
  assert.ok(s.loads.n[0] > s.loads.n[2]);
});

function scenario() {
  const sim = new WeightTransferSim(spec);
  const out = [];
  let prev = [0, 0, 0];
  for (let i = 0; i < 240; i++) {
    const want = [i < 120 ? 80 : -80, i % 50 < 25 ? 20 : -20, 0.5];
    const r = sim.limitDrive(prev, want, 1 / 60, 0.1 * i);
    if (i === 100) sim.onCollision({ normals: [[-1, 0]], vBefore: { vx: 50, vy: 5 }, h: 0.1 * i });
    const s = sim.step(1 / 60, r.bodyV);
    prev = r.bodyV;
    out.push([s.pitch, s.roll, s.heave, s.accel.ax, s.loads.n.slice(), s.scale, s.tip.mode]);
  }
  return JSON.stringify([out, Array.from(sim.visualMatrix())]);
}

test('deterministic across two identical runs', () => {
  assert.equal(scenario(), scenario());
});

test('reset returns sim to rest', () => {
  const sim = new WeightTransferSim(spec);
  sim.onCollision({ normals: [[-1, 0]], vBefore: { vx: 60, vy: 0 }, h: 0 });
  for (let i = 0; i < 30; i++) sim.step(1 / 60, [20, 0, 0]);
  sim.reset();
  assert.equal(sim.snapshot.pitch, 0);
  assert.equal(sim.snapshot.impactPeakLbf, 0);
});
