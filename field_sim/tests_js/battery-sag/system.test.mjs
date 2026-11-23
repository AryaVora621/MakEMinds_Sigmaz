import test from 'node:test';
import assert from 'node:assert/strict';
import { ElectricalSystem } from '../../web/js/physics/battery-sag/system.js';

const NOM = { maxSpeed: 60, maxAccel: 120, maxTurn: 5 };
const STALL = { driveCmd: { fwd: 1, strafe: 0, turn: 0 }, driveBodyV: [0, 0, 0], driveNominal: NOM };

function run(opts, inputs, n, dt = 0.002) {
  const sys = new ElectricalSystem(opts);
  let minV = Infinity, snap = null;
  for (let i = 0; i < n; i++) {
    snap = sys.step(dt, inputs);
    minV = Math.min(minV, snap.vBus);
  }
  return { sys, snap, minV };
}

const NOFUSE = { preset: 'fresh', fuseEnabled: false };

test('idle fresh battery sits near 14 V with small current', () => {
  const { snap } = run(NOFUSE, { driveNominal: NOM }, 10);
  assert.ok(snap.vBus > 13.5 && snap.vBus <= 14.0, `vBus=${snap.vBus}`);
  assert.ok(snap.iTotal > 0 && snap.iTotal < 3, `iTotal=${snap.iTotal}`);
  assert.equal(snap.hub.mode, 'ok');
  assert.equal(snap.fuseBlown, false);
});

test('drive stall plus flywheel spin-up sags more than either alone', () => {
  const fly = { flywheelRpm: 5500, driveNominal: NOM };
  const both = run(NOFUSE, { ...STALL, flywheelRpm: 5500 }, 20).minV;
  const driveOnly = run(NOFUSE, STALL, 20).minV;
  const flyOnly = run(NOFUSE, fly, 20).minV;
  assert.ok(both < driveOnly, `both=${both} drive=${driveOnly}`);
  assert.ok(both < flyOnly, `both=${both} fly=${flyOnly}`);
});

test('tired battery sags more than fresh at the same load', () => {
  const fresh = run({ ...NOFUSE, preset: 'fresh' }, STALL, 50, 0.01).snap.vBus;
  const tired = run({ ...NOFUSE, preset: 'tired' }, STALL, 50, 0.01).snap.vBus;
  assert.ok(tired < fresh - 0.3, `tired=${tired} fresh=${fresh}`);
});

test('4-motor stall on a fresh battery: about 10.6 V, about 32 A, fuse trips in 0.4-50 s', () => {
  const sys = new ElectricalSystem({ preset: 'fresh' });
  const first = sys.step(0.001, STALL);
  assert.ok(first.vBus > 10.3 && first.vBus < 10.9, `vBus=${first.vBus}`);
  assert.ok(first.iTotal > 28 && first.iTotal < 36, `iTotal=${first.iTotal}`);
  let tTrip = null;
  const dt = 0.005;
  let t = 0.001;
  for (let i = 0; i < 20000; i++) {
    const s = sys.step(dt, STALL);
    t += dt;
    if (s.fuseBlown) { tTrip = t; break; }
  }
  assert.ok(tTrip !== null, 'fuse never blew');
  // Spec acceptance says 0.75-50 s, but the spec's own I2t cap (520/I^2 = about 0.5 s at 32 A) trips earlier.
  assert.ok(tTrip > 0.4 && tTrip < 50, `tTrip=${tTrip}`);
  const after = sys.step(dt, STALL);
  assert.equal(after.fuseBlown, true);
  assert.equal(after.vBus, 0);
  assert.equal(after.iTotal, 0);
});

test('SoC decreases monotonically under load and ahUsed grows', () => {
  const sys = new ElectricalSystem(NOFUSE);
  let prev = 2, prevAh = -1;
  for (let i = 0; i < 300; i++) {
    const s = sys.step(0.01, { ...STALL, flywheelRpm: 4000 });
    assert.ok(s.soc <= prev + 1e-15);
    assert.ok(s.ahUsed >= prevAh);
    prev = s.soc; prevAh = s.ahUsed;
  }
  assert.ok(prev < 1);
});

test('the same inputs give identical results (deterministic)', () => {
  const a = run(NOFUSE, { ...STALL, flywheelRpm: 5000 }, 100).snap;
  const b = run(NOFUSE, { ...STALL, flywheelRpm: 5000 }, 100).snap;
  assert.equal(a.vBus, b.vBus);
  assert.equal(a.soc, b.soc);
});

test('a 5500 rpm flywheel saturates when the battery is sagged by a drive stall', () => {
  const sys = new ElectricalSystem({ ...NOFUSE, preset: 'tired' });
  let s;
  for (let i = 0; i < 1500; i++) s = sys.step(0.002, { ...STALL, flywheelRpm: 5500 });
  assert.ok(s.flywheelRpm < 5500 - 100, `rpm=${s.flywheelRpm}`);
  assert.equal(s.flywheelAtSpeed, false);
});

test('driveLimits shrink with sag; disabled hub outputs zero them', () => {
  const { snap } = run(NOFUSE, STALL, 50, 0.01);
  assert.ok(snap.driveLimits.maxSpeed < 60);
  assert.ok(snap.driveLimits.speedScale < 1);
  const hubOff = new ElectricalSystem({ ...NOFUSE, preset: 'fresh' });
  hubOff.hub.state.outputsEnabled = false;
  hubOff.hub.state.mode = 'reset';
  const s = hubOff.step(0.01, { ...STALL, driveNominal: NOM });
  assert.equal(s.driveLimits.maxSpeed, 0);
});

test('snapshot has every documented key', () => {
  const { snap } = run(NOFUSE, { ...STALL, servosMoving: 1, servosIdle: 2, intakeDuty: 0.5 }, 5);
  const keys = ['t', 'vBus', 'vSensor', 'vOc', 'iTotal', 'iDrive', 'iFlywheel', 'iIntake', 'iElectronics',
    'soc', 'ahUsed', 'fuseBlown', 'fuseDamage', 'hub', 'portOvercurrent', 'driveLimits',
    'flywheelRpm', 'flywheelAtSpeed', 'events'];
  for (const k of keys) assert.ok(k in snap, 'missing ' + k);
  for (const k of ['mode', 'outputsEnabled', 'lowBattery', 'ledFault', 'servoRailOk', 'resetCount']) assert.ok(k in snap.hub, 'hub.' + k);
  for (const k of ['maxSpeed', 'maxAccel', 'maxDecel', 'maxTurn', 'speedScale']) assert.ok(k in snap.driveLimits, 'driveLimits.' + k);
  assert.equal(snap.iDrive.length, 4);
  assert.equal(snap.portOvercurrent.length, 4);
  assert.ok(Array.isArray(snap.events));
  assert.ok(snap.t > 0);
});

test('kickFlywheel dips the flywheel rpm; setPreset and reset work', () => {
  const sys = new ElectricalSystem(NOFUSE);
  for (let i = 0; i < 400; i++) sys.step(0.002, { flywheelRpm: 3000, driveNominal: NOM });
  const before = sys.flywheel.state.rpm;
  sys.kickFlywheel();
  assert.ok(sys.flywheel.state.rpm < before);
  sys.setPreset('tired');
  assert.ok(sys.battery.state.soc < 0.5);
  sys.reset('fresh');
  assert.equal(sys.battery.state.soc, 1);
  assert.equal(sys.t, 0);
});
