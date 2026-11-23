// Tests for fit.js: synthetic round-trips with analytic data, plus dipFromLog on a burstSim trace.
import test from 'node:test';
import assert from 'node:assert/strict';
import { FLYWHEEL_PARAMS, RPM_TO_RADS } from '../../web/js/physics/flywheel-dynamics/params.js';
import { makeDcMotor } from '../../web/js/physics/flywheel-dynamics/motor.js';
import { coastDownOmega } from '../../web/js/physics/flywheel-dynamics/losses.js';
import { burstSim } from '../../web/js/physics/flywheel-dynamics/timing.js';
import {
  fitCoastDown, fitStepResponse, inertiaFromKa, jFromCoastDown, dipFromLog,
} from '../../web/js/physics/flywheel-dynamics/fit.js';

const near = (a, b, rel, msg) => assert.ok(Math.abs(a - b) <= rel * Math.abs(b), `${msg || ''} got ${a} want ${b} (rel ${rel})`);
const J = 2.515e-4;
const Tc = 8.4286e-3;
const b = 2e-4; // test-only viscous term, large enough to be identifiable

test('fitCoastDown recovers a = Tc/J and k = b/J from analytic coast-down data', () => {
  const t = [], rpm = [];
  for (let i = 0; i <= 1200; i++) {
    const ti = i * 0.005;
    t.push(ti);
    rpm.push(coastDownOmega(J, Tc, b, 4000 * RPM_TO_RADS, ti) / RPM_TO_RADS);
  }
  const f = fitCoastDown(t, rpm);
  near(f.aRadS2, Tc / J, 0.02, 'a');
  near(f.kPerS, b / J, 0.02, 'k');
  assert.ok(f.n > 100);
  assert.ok(f.rmsRadS2 < 0.5);
  near(jFromCoastDown(f.aRadS2, Tc), J, 0.02);
});

test('fitCoastDown tolerates measurement noise', () => {
  let seed = 12345;
  const rnd = () => { seed = (seed * 1664525 + 1013904223) % 4294967296; return seed / 4294967296 - 0.5; };
  const t = [], rpm = [];
  for (let i = 0; i <= 600; i++) {
    const ti = i * 0.005;
    t.push(ti);
    rpm.push(coastDownOmega(J, Tc, b, 4000 * RPM_TO_RADS, ti) / RPM_TO_RADS + 2 * rnd());
  }
  near(fitCoastDown(t, rpm).aRadS2, Tc / J, 0.15);
});

test('fit functions return NaN or empty on bad input and never throw', () => {
  assert.ok(Number.isNaN(fitCoastDown([], []).aRadS2));
  assert.ok(Number.isNaN(fitCoastDown([0, 1], [1]).aRadS2));
  assert.ok(Number.isNaN(fitStepResponse([0, 1], [0, 1], 12).tauS));
  assert.ok(Number.isNaN(inertiaFromKa(NaN, { R: 1, Kt: 1, count: 1 }, 1)));
  assert.ok(Number.isNaN(jFromCoastDown(0, 1)));
  assert.deepEqual(dipFromLog(null, null, null), []);
});

test('fitStepResponse recovers tau, kV, kA, and J via inertiaFromKa', () => {
  const m = makeDcMotor(FLYWHEEL_PARAMS.motor);
  const G = 1, V = 12;
  const wss = V / (m.Ke * G);
  const tau = J * m.R / (m.count * m.Kt * G * G * m.Ke); // open-loop tau, spec 2g
  near(tau, 0.5614, 2e-3, 'spec tau');
  const t = [], rpm = [];
  for (let i = 0; i <= 800; i++) {
    const ti = i * 0.01;
    t.push(ti);
    rpm.push(wss * (1 - Math.exp(-ti / tau)) / RPM_TO_RADS);
  }
  const f = fitStepResponse(t, rpm, V);
  near(f.tauS, tau, 0.02, 'tau');
  near(f.omegaSsRadS, wss, 0.01, 'wss');
  near(f.kVVoltsPerRadS, m.Ke * G, 0.01, 'kV');
  near(f.kAVoltsPerRadS2, m.Ke * G * tau, 0.03, 'kA');
  near(inertiaFromKa(f.kAVoltsPerRadS2, m, G), J, 0.03, 'J');
  // kS is subtracted from kV only when supplied
  const g = fitStepResponse(t, rpm, V, { kSVolts: 0.35 });
  assert.ok(g.kVVoltsPerRadS < f.kVVoltsPerRadS);
});

test('inertiaFromKa is exact on the model identity', () => {
  const m = makeDcMotor(FLYWHEEL_PARAMS.motor);
  const kA = J * m.R / (m.count * 1.5 * m.Kt);
  near(inertiaFromKa(kA, m, 1.5), J, 1e-12);
});

test('dipFromLog on a synthetic exponential-recovery trace', () => {
  const t = [], rpm = [];
  for (let i = 0; i <= 2000; i++) {
    const ti = i * 0.001;
    t.push(ti);
    rpm.push(ti < 1.0 ? 4000 : 4000 - 300 * Math.exp(-(ti - 1.0) / 0.05));
  }
  const [r] = dipFromLog(t, rpm, [1.0]);
  near(r.preRpm, 4000, 1e-9);
  near(r.minRpm, 3700, 1e-6);
  near(r.dipFrac, 0.075, 1e-6);
  near(r.recoveryS, 0.05 * Math.log(300 / 80), 0.05); // back inside 2%
  const never = dipFromLog(t.slice(0, 1050), rpm.slice(0, 1050), [1.0])[0];
  assert.equal(never.recoveryS, Infinity); // log ends before recovery completes
});

test('dipFromLog on a burstSim trace agrees with the shot predictions', () => {
  const sim = burstSim({}, 4000, ['pollen', 'pollen', 'pollen'], 0.3, { traceEveryS: 0.002 });
  const res = dipFromLog(sim.trace.map((p) => p.tS), sim.trace.map((p) => p.rpm), sim.shots.map((s) => s.tS));
  assert.equal(res.length, 3);
  // burstSim time starts at the first fire, so shot 0 has no pre-fire samples and must not produce a bogus number
  assert.ok(Number.isNaN(res[0].preRpm) || res[0].preRpm > 3000);
  for (let i = 1; i < 3; i++) {
    assert.ok(res[i].dipFrac > 0.03 && res[i].dipFrac < 0.12, `dip ${res[i].dipFrac}`);
    assert.ok(Math.abs(res[i].dipFrac - sim.shots[i].dipFrac) < 0.03, 'matches predicted dip');
    assert.ok(res[i].recoveryS > 0);
    assert.ok(res[i].minRpm < res[i].preRpm);
  }
  assert.ok(Number.isFinite(res[1].recoveryS));
});
