import test from 'node:test';
import assert from 'node:assert/strict';
import { solveBus } from '../../web/js/physics/battery-sag/bus.js';
import { Fuse, fuseOpenTime } from '../../web/js/physics/battery-sag/fuse.js';
import { FUSE } from '../../web/js/physics/battery-sag/params.js';

const near = (a, b, e = 1e-6) => assert.ok(Math.abs(a - b) <= e, `${a} vs ${b} (tol ${e})`);

test('Vth 13, Rs 0.1, linear g=1 gives 11.818 V', () => {
  const r = solveBus(13, 0.1, [{ kind: 'linear', g: 1, a: 0 }]);
  near(r.v, 11.818, 1e-3);
  near(r.iTotal, 11.818, 1e-3);
  near(r.currents[0], 11.818, 1e-3);
});

test('constant-current load: V = Vth - Rs*I', () => {
  const r = solveBus(13, 0.1, [{ kind: 'current', amps: 10 }]);
  near(r.v, 12, 1e-9);
  near(r.iTotal, 10, 1e-9);
});

test('constant-power load converges close to the exact root', () => {
  const r = solveBus(13, 0.1, [{ kind: 'power', watts: 50 }]);
  const exact = (13 + Math.sqrt(169 - 20)) / 2; // V^2 - 13V + 5 = 0
  near(r.v, exact, 0.01);
  near(r.iTotal, 50 / r.v, 0.05);
});

test('linear load with offset: I = g*V - a', () => {
  const r = solveBus(13, 0.1, [{ kind: 'linear', g: 2, a: 5 }]);
  near(r.currents[0], 2 * r.v - 5, 1e-9);
  near(r.v, 13 - 0.1 * r.iTotal, 1e-9);
});

test('Rs = 0 gives V = Vth', () => {
  const r = solveBus(13, 0, [{ kind: 'linear', g: 3, a: 1 }, { kind: 'power', watts: 20 }]);
  near(r.v, 13, 1e-12);
});

test('mixed loads: iTotal is the sum of currents and V sags', () => {
  const loads = [{ kind: 'linear', g: 1, a: 0 }, { kind: 'current', amps: 3 }, { kind: 'power', watts: 10 }];
  const r = solveBus(13, 0.1, loads);
  near(r.iTotal, r.currents.reduce((a, b) => a + b, 0), 1e-9);
  assert.ok(r.v < 13);
  near(r.v, 13 - 0.1 * r.iTotal, 0.02);
});

test('vFloor clamps a negative solution to 0', () => {
  const r = solveBus(13, 1, [{ kind: 'current', amps: 100 }]);
  assert.equal(r.v, 0);
});

function tripTime(amps, dt = 0.001, maxT = 600) {
  const f = new Fuse();
  for (let t = 0; t < maxT; t += dt) if (f.step(amps, dt)) return t + dt;
  return Infinity;
}

test('fuseOpenTime is Infinity at or below 1.10x rating', () => {
  assert.equal(fuseOpenTime(21), Infinity);
  assert.equal(fuseOpenTime(22), Infinity);
});

// NOTE: spec quotes 0.866 s for 40 A from the curve, but the I2t cap 520/40^2 = 0.325 s is
// smaller, so the min() in the spec gives 0.325 s. Both lie inside the datasheet window 0.15-5 s.
test('40 A trips inside the datasheet window 0.15-5 s', () => {
  const t = tripTime(40);
  assert.ok(t > 0.15 && t < 5, `t=${t}`);
  const open = fuseOpenTime(40);
  assert.ok(open > 0.15 && open <= 0.866 + 1e-6, `open=${open}`);
});

test('70 A trips inside 0.08-0.5 s (about 0.106 s)', () => {
  const t = tripTime(70, 0.0005, 5);
  assert.ok(t > 0.08 && t < 0.5, `t=${t}`);
  near(t, 0.106, 0.02);
});

test('21 A never trips in 600 s', () => {
  const f = new Fuse();
  for (let i = 0; i < 6000; i++) assert.equal(f.step(21, 0.1), false);
  assert.equal(f.state.blown, false);
});

test('damage accumulates above threshold and decays below 22 A', () => {
  const f = new Fuse();
  f.step(30, 0.5);
  const d1 = f.state.damage;
  assert.ok(d1 > 0 && d1 < 1);
  f.step(15, 5);
  assert.ok(f.state.damage < d1 * 0.7, `damage ${f.state.damage} vs ${d1}`);
});

test('a blown fuse latches until reset()', () => {
  const f = new Fuse();
  for (let i = 0; i < 5000 && !f.state.blown; i++) f.step(70, 0.001);
  assert.equal(f.state.blown, true);
  assert.equal(f.step(0, 1), true);
  assert.equal(f.state.blown, true);
  f.reset();
  assert.equal(f.state.blown, false);
  assert.equal(f.state.damage, 0);
});

test('a fuse with enabled=false never blows', () => {
  const f = new Fuse({ ...FUSE, enabled: false });
  let blown = false;
  for (let i = 0; i < 1000; i++) blown = f.step(100, 0.01) || blown;
  assert.equal(blown, false);
});
