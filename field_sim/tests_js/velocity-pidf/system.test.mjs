import test from 'node:test';
import assert from 'node:assert/strict';
import { FlywheelSystem, createShooterPlantAdapter } from '../../web/js/physics/velocity-pidf/flywheel_system.js';
import { ShooterPlant } from '../../web/js/aimcontrol.js';

const TARGET_TICKS = 3000 * 28 / 60;   // 1400 ticks/s
const FRAME = 0.005;

function run(fw, seconds, cmd, onFrame) {
  const n = Math.round(seconds / FRAME);
  for (let i = 0; i < n; i++) { fw.step(FRAME, cmd); if (onFrame) onFrame(fw.state, i); }
}

test('sdk mode 3000 rpm: reaches tolerance before 3 s, overshoot below 10%, small steady error', () => {
  const fw = new FlywheelSystem();
  let tReady = null, maxRpm = 0;
  const tail = [];
  run(fw, 6, { rpm: 3000 }, (s) => {
    if (tReady === null && Math.abs(s.measuredTicksPerS - TARGET_TICKS) <= 50) tReady = s.time;
    maxRpm = Math.max(maxRpm, s.rpm);
    if (s.time > 5) tail.push(s.measuredTicksPerS - TARGET_TICKS);
  });
  assert.ok(tReady !== null && tReady < 3.0, `tReady=${tReady}`);
  assert.ok(maxRpm < 3000 * 1.10, `max rpm ${maxRpm}`);
  const mean = tail.reduce((a, b) => a + b, 0) / tail.length;
  assert.ok(Math.abs(mean) < 30, `steady error ${mean} ticks/s`);   // derived about -4 ticks/s plus 20 ticks/s quantisation
  assert.equal(fw.state.targetTicksPerS, TARGET_TICKS);
  assert.ok(fw.state.ready);
});

test('measured ticks/s is always an integer multiple of 20', () => {
  const fw = new FlywheelSystem();
  run(fw, 3, { rpm: 3000 }, (s) => {
    assert.ok(Number.isInteger(s.measuredTicksPerS));
    assert.equal(s.measuredTicksPerS % 20, 0, `v=${s.measuredTicksPerS}`);
  });
});

test('power stays 0 until bulkReadS + cmdLatencyS after the first loop (serial per port)', () => {
  const fw = new FlywheelSystem();
  const H = 0.0005;
  let sawZero = true;
  while (fw.time < 0.004 - 1e-9) { fw.step(H, { rpm: 3000 }); if (fw.state.appliedVolts[0] !== 0 || fw.state.appliedVolts[1] !== 0) sawZero = false; }
  assert.ok(sawZero);
  assert.equal(fw.state.loopCount >= 1, true);
  while (fw.time < 0.0055 - 1e-9) fw.step(H, { rpm: 3000 });
  assert.ok(fw.state.appliedVolts[0] > 0, 'port 0 applied at 4.5 ms');
  assert.equal(fw.state.appliedVolts[1], 0, 'port 1 not yet (6 ms)');
  while (fw.time < 0.0075 - 1e-9) fw.step(H, { rpm: 3000 });
  assert.ok(fw.state.appliedVolts[1] > 0, 'port 1 applied at 6 ms');
});

test('same seed gives an identical trajectory, different seed differs', () => {
  const trace = (over) => {
    const fw = new FlywheelSystem(over);
    const out = [];
    run(fw, 2, { rpm: 3000 }, (s) => out.push([s.rpm, s.measuredTicksPerS, s.powerCmd, s.loopCount]));
    return out;
  };
  assert.deepEqual(trace(), trace());
  assert.notDeepEqual(trace(), trace({ timing: { seed: 777 } }));
});

test('kick at 3000 rpm: 1-8% true drop, recovery within 1% by 1.5 s, measured drop lags the next loop', () => {
  const fw = new FlywheelSystem();
  run(fw, 6, { rpm: 3000 });
  const pre = fw.state.rpm;
  assert.ok(Math.abs(pre - 3000) < 60, `pre ${pre}`);
  const preMeasured = fw.state.measuredTicksPerS;
  const preLoops = fw.state.loopCount;
  const r = fw.kick('pollen');
  assert.ok(r.energyJ > 1.5 && r.energyJ < 1.8, `E=${r.energyJ}`);
  assert.ok(r.expectedDropRpm > 0);
  // sub-loop stepping: until the next loop runs, the measured value cannot have changed
  let minRpm = pre;
  for (let i = 0; i < 400 && fw.state.loopCount === preLoops; i++) {
    fw.step(0.0005, { rpm: 3000 });
    minRpm = Math.min(minRpm, fw.state.rpm);
    if (fw.state.loopCount === preLoops) assert.equal(fw.state.measuredTicksPerS, preMeasured);
  }
  const tKick = fw.time;
  run(fw, 0.5, { rpm: 3000 }, (s) => { minRpm = Math.min(minRpm, s.rpm); });
  const drop = (pre - minRpm) / pre;
  assert.ok(drop > 0.01 && drop < 0.08, `drop ${drop}`);
  while (fw.time < tKick + 1.5) fw.step(FRAME, { rpm: 3000 });
  assert.ok(Math.abs(fw.state.rpm - pre) / pre < 0.01, `recovered rpm ${fw.state.rpm}`);
});

test('state exposes the documented readouts', () => {
  const fw = new FlywheelSystem();
  run(fw, 1, { rpm: 3000 });
  const s = fw.state;
  for (const k of ['rpm', 'measuredRpm', 'measuredTicksPerS', 'targetRpm', 'targetTicksPerS', 'powerCmd', 'currentA', 'batteryV', 'loopDtS', 'loopCount', 'time']) {
    assert.ok(Number.isFinite(s[k]), k);
  }
  assert.equal(s.appliedVolts.length, 2);
  assert.equal(typeof s.ready, 'boolean');
  assert.ok(Math.abs(s.targetRpm - 3000) < 1e-6);
  assert.ok(s.batteryV <= 12.8 && s.batteryV >= 6);
  assert.equal(fw.ready(), s.ready);
});

test('target below min rpm is clamped up, target 0 gives zero power', () => {
  const fw = new FlywheelSystem();
  run(fw, 0.5, { rpm: 1000 });
  assert.ok(Math.abs(fw.state.targetRpm - 2500) < 1e-6);
  run(fw, 0.5, { rpm: 0 });
  assert.equal(fw.state.targetTicksPerS, 0);
  assert.equal(fw.state.powerCmd, 0);
});

test('step guards NaN and negative frame dt, clamps huge frame dt', () => {
  const fw = new FlywheelSystem();
  fw.step(NaN, { rpm: 3000 });
  assert.equal(fw.time, 0);
  fw.step(-1, { rpm: 3000 });
  assert.equal(fw.time, 0);
  fw.step(100, { rpm: 3000 });
  assert.ok(fw.time <= 0.25 + 1e-9);
});

test('hub mode runs, stays finite and quantised, and setMode/reset work', () => {
  const fw = new FlywheelSystem({ mode: 'hub' });
  run(fw, 4, { rpm: 3000 }, (s) => {
    assert.ok(Number.isFinite(s.rpm) && s.rpm >= 0 && s.rpm <= 6600);
    assert.equal(s.measuredTicksPerS % 20, 0);
  });
  assert.ok(fw.state.rpm > 500);
  fw.reset();
  assert.equal(fw.time, 0);
  assert.equal(fw.state.rpm, 0);
  fw.setMode('sdk');
  run(fw, 3, { rpm: 3000 });
  assert.ok(Math.abs(fw.state.rpm - 3000) < 150);
});

test('adapter forwards turret/hood to ShooterPlant and takes rpm from the flywheel system', () => {
  const base = new ShooterPlant();
  const ref = new ShooterPlant();
  const fw = new FlywheelSystem();
  const ad = createShooterPlantAdapter(base, fw);
  assert.equal(ad.flywheel, fw);
  assert.equal(ad.spec, base.spec);
  const cmd = { turretDeg: 90, hoodDeg: 40, rpm: 3000 };
  let res;
  for (let i = 0; i < 100; i++) {
    res = ad.step(0.01, cmd);
    ref.step(0.01, { turretDeg: 90, hoodDeg: 40 });
  }
  assert.equal(base.state.turretDeg, ref.state.turretDeg);
  assert.equal(base.state.hoodDeg, ref.state.hoodDeg);
  assert.ok(base.state.turretDeg > 10);
  assert.ok('atTurretLimit' in res && 'atHoodLimit' in res);
  assert.equal(ad.state.rpm, fw.state.rpm);
  assert.ok(ad.state.rpm > 100);
  assert.ok(Number.isFinite(ad.state.measuredRpm));
  assert.equal(ad.atLimit, base.atLimit);
  const sp = ad.atSetpoint({ turretDeg: 90, hoodDeg: 40, rpm: 3000 });
  assert.equal(typeof sp.turret, 'boolean');
  assert.equal(typeof sp.hood, 'boolean');
  assert.equal(sp.rpm, fw.ready());
  const before = fw.state.rpm;
  const k = ad.kick('pollen');
  assert.ok(k && k.energyJ > 0);
  ad.step(0.05, cmd);
  assert.ok(fw.state.rpm < before);
});
