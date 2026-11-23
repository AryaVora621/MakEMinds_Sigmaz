// Tests for flywheel_plant.js (flywheel-dynamics). Oracles are spec section 2g numbers and analytic formulas.
import test from 'node:test';
import assert from 'node:assert/strict';
import { FlywheelPlant } from '../../web/js/physics/flywheel-dynamics/flywheel_plant.js';
import { coastDownOmega } from '../../web/js/physics/flywheel-dynamics/losses.js';

const DT = 0.001;
const near = (a, b, rel, msg) => assert.ok(Math.abs(a - b) <= rel * Math.abs(b), `${msg || ''} got ${a} want ${b} (rel ${rel})`);

function run(p, seconds, cb) {
  const n = Math.round(seconds / DT);
  for (let i = 0; i < n; i++) { p.step(DT); if (cb) cb(p, i); }
}

/** Time for a fresh plant (overrides) to first enter and then stay within frac of target, from rest. */
function spinUpTime(overrides, target = 4000, frac = 0.03) {
  const p = new FlywheelPlant(overrides);
  p.reset(0);
  p.setTargetRpm(target);
  let t = null;
  for (let i = 0; i < 10000; i++) {
    p.step(DT);
    const inside = Math.abs(p.state.rpm - target) <= frac * target;
    if (inside && t === null) t = p.state.timeS;
    if (!inside) t = null;
  }
  return t;
}

test('spin-up 0 to 4000 rpm settles to 3% in [0.3, 1.5] s', () => {
  const t = spinUpTime();
  assert.ok(t !== null && t >= 0.3 && t <= 1.5, `spin-up ${t}`);
});

test('plant reaches the target and holds it', () => {
  const p = new FlywheelPlant();
  p.reset(0);
  p.setTargetRpm(4000);
  run(p, 2.5);
  near(p.state.rpm, 4000, 0.01);
  assert.equal(p.state.float, false);
});

test('energy invariant 0.5*J*(w1^2-w0^2) = drive - loss - load within 0.5% over spin-up plus 3 shots', () => {
  const p = new FlywheelPlant();
  p.reset(0);
  p.setTargetRpm(4000);
  const E0 = p.kineticEnergyJ();
  run(p, 1.5);
  for (let k = 0; k < 3; k++) { p.fire('pollen'); run(p, 0.4); }
  const dE = p.kineticEnergyJ() - E0;
  const e = p.state.energy;
  const bal = e.driveJ - e.lossJ - e.loadJ;
  const scale = Math.max(Math.abs(dE), p.kineticEnergyJ());
  assert.ok(Math.abs(dE - bal) <= 0.005 * scale, `dE ${dE} vs ${bal}`);
  assert.ok(e.driveJ > 0 && e.lossJ > 0 && e.loadJ > 0);
});

test('plant dip for pollen at 4000 rpm is in [0.04, 0.10] and no larger than the prediction', () => {
  const p = new FlywheelPlant();
  p.reset(0);
  p.setTargetRpm(4000);
  run(p, 2.0);
  const pre = p.state.rpm;
  const shot = p.fire('pollen');
  let minRpm = pre;
  run(p, 0.15, () => { minRpm = Math.min(minRpm, p.state.rpm); });
  const dip = 1 - minRpm / pre;
  assert.ok(dip >= 0.04 && dip <= 0.10, `dip ${dip}`);
  assert.ok(dip <= shot.dipFrac + 1e-3, `plant ${dip} vs predicted ${shot.dipFrac}`);
  assert.ok(shot.exitSpeedMps > 0);
  assert.equal(typeof shot.shotId, 'number');
});

test('recovery to 2% after a pollen shot is under 0.3 s and faster than spin-up', () => {
  const p = new FlywheelPlant();
  p.reset(0);
  p.setTargetRpm(4000);
  run(p, 2.0);
  p.fire('pollen');
  const tFire = p.state.timeS;
  let sawDip = false;
  let rec = null;
  for (let i = 0; i < 1000 && rec === null; i++) {
    p.step(DT);
    const err = Math.abs(p.state.rpm - 4000);
    if (err > 0.02 * 4000) sawDip = true;
    else if (sawDip) rec = p.state.timeS - tFire;
  }
  assert.ok(sawDip, 'shot produced a dip beyond 2%');
  assert.ok(rec !== null && rec < 0.3, `recovery ${rec}`);
  assert.ok(rec < spinUpTime(), 'recovery faster than spin-up');
});

test('float coast 4000 to 2000 rpm takes more than 2 s; brake coast less than 1 s', () => {
  const coast = (ov) => {
    const p = new FlywheelPlant(ov);
    p.reset(4000);
    p.setTargetRpm(0);
    for (let i = 0; i < 30000; i++) {
      p.step(DT);
      if (p.state.rpm <= 2000) return p.state.timeS;
    }
    return Infinity;
  };
  assert.ok(coast() > 2, 'float');
  assert.ok(coast({ losses: { zeroPowerBehavior: 'brake' } }) < 1, 'brake');
});

test('coast with windage off and viscous 0 matches analytic linear coast-down within 1%', () => {
  const p = new FlywheelPlant({ losses: { windage: { enabled: false }, viscousNmsPerRad: 0 } });
  p.reset(4000);
  p.setTargetRpm(0);
  const w0 = p.state.omegaRadS;
  run(p, 2.0);
  const want = coastDownOmega(p.jKgM2, p.coulombNm, 0, w0, p.state.timeS);
  near(p.state.omegaRadS, want, 0.01);
});

test('stays at 0 with target 0 and never goes negative', () => {
  const p = new FlywheelPlant();
  p.reset(0);
  p.setTargetRpm(0);
  run(p, 1.0, () => assert.ok(p.state.omegaRadS === 0));
  const q = new FlywheelPlant();
  q.reset(100);
  q.setTargetRpm(0);
  run(q, 5.0, () => assert.ok(q.state.omegaRadS >= 0));
  assert.equal(q.state.omegaRadS, 0);
});

test('step with NaN, negative or zero dt is a no-op', () => {
  const p = new FlywheelPlant();
  p.reset(0);
  p.setTargetRpm(3000);
  run(p, 0.2);
  const before = JSON.stringify(p.state);
  for (const dt of [NaN, -0.01, 0, Infinity, undefined]) {
    assert.doesNotThrow(() => p.step(dt));
    assert.equal(JSON.stringify(p.state), before, `dt=${dt}`);
  }
});

test('setTargetRpm ignores non-finite and clamps to [0, maxRpm]', () => {
  const p = new FlywheelPlant();
  p.setTargetRpm(3000);
  p.setTargetRpm(NaN);
  assert.equal(p.state.targetRpm, 3000);
  p.setTargetRpm(99999);
  assert.equal(p.state.targetRpm, 6000);
  p.setTargetRpm(-5);
  assert.equal(p.state.targetRpm, 0);
});

test('doubling J roughly doubles the spin-up time', () => {
  const base = new FlywheelPlant().jKgM2;
  const t1 = spinUpTime();
  const t2 = spinUpTime({ inertia: { extraKgM2: base } });
  const ratio = t2 / t1;
  assert.ok(ratio >= 1.7 && ratio <= 2.3, `ratio ${ratio}`);
});

test('constructor throws on invalid params; reset clears state', () => {
  assert.throws(() => new FlywheelPlant({ motor: { count: 0 } }), Error);
  const p = new FlywheelPlant();
  p.reset(0);
  p.setTargetRpm(4000);
  run(p, 1.0);
  p.fire('pollen');
  p.reset(500);
  assert.equal(p.state.timeS, 0);
  near(p.state.rpm, 500, 1e-9);
  assert.equal(p.state.activeLoads, 0);
  assert.equal(p.state.energy.driveJ, 0);
});

test('no NaN anywhere, even with junk fire options and many shots', () => {
  const p = new FlywheelPlant();
  p.reset(0);
  p.setTargetRpm(5000);
  const check = () => {
    for (const [k, v] of Object.entries(p.state)) {
      if (typeof v === 'number') assert.ok(Number.isFinite(v), k);
    }
    for (const v of Object.values(p.state.energy)) assert.ok(Number.isFinite(v));
  };
  run(p, 1.0, check);
  assert.doesNotThrow(() => p.fire('nectar', { massScale: NaN, etaScale: undefined }));
  assert.doesNotThrow(() => p.fire({ massKg: 0.03 }, { massScale: 1.05, etaScale: 0.97 }));
  assert.doesNotThrow(() => p.fire('nope'));
  run(p, 0.5, check);
  p.setTargetRpm(0);
  run(p, 5.0, check);
});

test('fire at rest is harmless', () => {
  const p = new FlywheelPlant();
  p.reset(0);
  const s = p.fire('pollen');
  assert.equal(s.exitSpeedMps, 0);
  run(p, 0.1);
  assert.equal(p.state.omegaRadS, 0);
});
