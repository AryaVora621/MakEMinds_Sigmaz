import test from 'node:test';
import assert from 'node:assert/strict';
import { buildDrivetrainConfig } from '../../web/js/physics/wheel-friction/geometry.js';
import { makeParams, CHASSIS_DEFAULTS } from '../../web/js/physics/wheel-friction/params.js';
import { WheelFrictionDrivetrain } from '../../web/js/physics/wheel-friction/drivetrain.js';
import { mixWheelPowers } from '../../web/js/physics/wheel-friction/kinematics.js';

const IN = 0.0254; // m per inch
const inRange = (v, lo, hi, label) => assert.ok(v >= lo && v <= hi, `${label}: ${v} not in [${lo}, ${hi}]`);
const FOOT = [[-7.5, -7.38], [7.5, -7.38], [7.5, 7.38], [-7.5, 7.38]];

function make(spec = null, overrides = null) {
  const cfg = buildDrivetrainConfig(spec, overrides ? makeParams(overrides) : makeParams());
  return { cfg, dt: new WheelFrictionDrivetrain(cfg) };
}

// Traces in in/s and rad/s (body frame); x, y = distance travelled in inches along body axes.
function simulate(drivetrain, powers, seconds, dt, opts = {}) {
  const n = Math.round(seconds / dt);
  const tr = { dt, t: [], vx: [], vy: [], om: [], x: 0, y: 0, tel: [] };
  for (let i = 0; i < n; i++) {
    const r = drivetrain.step(powers, dt, opts);
    tr.x += r.vx * dt / IN; tr.y += r.vy * dt / IN;
    tr.t.push((i + 1) * dt); tr.vx.push(r.vx / IN); tr.vy.push(r.vy / IN); tr.om.push(r.om);
    tr.tel.push(drivetrain.getTelemetry());
  }
  return tr;
}
const idx = (tr, t) => Math.min(tr.t.length - 1, Math.round(t / tr.dt) - 1);

test('forward launch is traction limited and saturated', () => {
  const { cfg, dt } = make();
  const tr = simulate(dt, mixWheelPowers(1, 0, 0, cfg), 0.25, 1 / 100);
  inRange(tr.vx[idx(tr, 0.25)] / 0.25, 100, 175, 'launch accel in/s^2');
  for (const w of tr.tel[idx(tr, 0.05)].wheels) assert.equal(w.saturated, true);
});

test('top speeds: forward, strafe, diagonal, turn', () => {
  const { cfg, dt } = make();
  let tr = simulate(dt, mixWheelPowers(1, 0, 0, cfg), 2, 1 / 100);
  const vf = tr.vx.at(-1);
  inRange(vf, 58, 68, 'forward top in/s');
  dt.reset();
  tr = simulate(dt, mixWheelPowers(0, 1, 0, cfg), 2, 1 / 100);
  assert.ok(Math.abs(tr.vy.at(-1) - vf) / vf < 0.10, `strafe ${tr.vy.at(-1)} vs fwd ${vf}`);
  dt.reset();
  tr = simulate(dt, mixWheelPowers(1, 1, 0, cfg), 2, 1 / 100);
  inRange(Math.hypot(tr.vx.at(-1), tr.vy.at(-1)), 41, 50, 'diagonal top in/s');
  dt.reset();
  tr = simulate(dt, mixWheelPowers(0, 0, 1, cfg), 2, 1 / 100);
  inRange(tr.om.at(-1), 4.5, 6.0, 'turn rate rad/s');
});

// Diagonal / forward launch acceleration (round 2). The old [0.6, 0.8] band assumed the ideal
// 1/sqrt(2) = 0.707, which ignores roller drag. Analytic value, both launches traction limited:
//  forward : 4 wheels saturated, each mu_k*N along its 45 deg roller-constrained axis
//            -> a_f = 4 * mu_k*N * cos45 / M = mu_k*g/sqrt(2)         (sim: 139.4 in/s^2, 1.0014x analytic)
//  diagonal: powers [0,1,1,0]; FR and BL push mu_k*N exactly along the motion, FL and BR are
//            unpowered and roll on their rollers, i.e. the motion is along their FREE axis, so they
//            drag crrFree*N each  -> a_d = (2*mu_k*N - 2*crrFree*N) / M = (mu_k - crrFree)*g/2
//  ratio = (mu_k - crrFree) / (sqrt(2)*mu_k) = (0.51 - 0.084)/(1.4142*0.51) = 0.5906 with the SOURCED
//  crrFree (arXiv 1211.2323). Sim sweep of crrFree (ratio sim / analytic): 0 -> 0.7071/0.7071,
//  0.04 -> 0.6517/0.6516, 0.084 -> 0.5908/0.5906, 0.15 -> 0.4994/0.4991. So 0.59 is the correct physics
//  for these parameters, not a defect; the test now checks the analytic value (tighter than the old
//  0.2-wide band) and, with crrFree = 0, the ideal 1/sqrt(2) inside the original band.
test('diagonal / forward launch acceleration matches (mu_k - crrFree)/(sqrt2*mu_k)', () => {
  const ratio = (overrides) => {
    const { cfg, dt } = make(null, overrides);
    const f = simulate(dt, mixWheelPowers(1, 0, 0, cfg), 0.25, 1 / 100);
    dt.reset();
    const d = simulate(dt, mixWheelPowers(1, 1, 0, cfg), 0.25, 1 / 100);
    const i = idx(f, 0.25);
    const w = cfg.wheels[0];
    return { r: Math.hypot(d.vx[i], d.vy[i]) / f.vx[i], want: (w.muKinetic - w.crrFree) / (Math.SQRT2 * w.muKinetic) };
  };
  const a = ratio(null);
  assert.ok(Math.abs(a.r - a.want) < 0.01, `diag/fwd ${a.r} vs analytic ${a.want}`);
  const b = ratio({ WHEEL_TYPES: { mecanum: { crrFree: 0 } } });
  assert.ok(Math.abs(b.r - Math.SQRT1_2) < 0.01, `diag/fwd without roller drag ${b.r} vs 0.7071`);
  inRange(b.r, 0.6, 0.8, 'diag/fwd without roller drag (original band)');
});

test('coast down: brake stops fast, float slower and farther', () => {
  const run = (zpb) => {
    const { dt } = make(null, { MOTOR: { zeroPowerBehavior: zpb } });
    dt.setBodyVelocity(60 * IN, 0, 0, true);
    const tr = simulate(dt, [0, 0, 0, 0], 5, 1 / 100);
    let tStop = Infinity;
    for (let i = 0; i < tr.t.length; i++) if (Math.abs(tr.vx[i]) < 0.5) { tStop = tr.t[i]; break; }
    return { tStop, dist: tr.x };
  };
  const b = run('brake'), f = run('float');
  assert.ok(b.tStop <= 0.6, `brake stop ${b.tStop}`);
  inRange(f.tStop, 0.8, 4, 'float stop time s');
  assert.ok(f.dist > b.dist);
});

test('rest with zero power is stable', () => {
  const { dt } = make();
  const tr = simulate(dt, [0, 0, 0, 0], 2, 1 / 60);
  assert.ok(Math.hypot(tr.vx.at(-1), tr.vy.at(-1)) < 1e-6);
});

test('pushing: 40 N opposing still goes forward, 75 N goes backward', () => {
  const run = (F) => {
    const { cfg, dt } = make();
    return simulate(dt, mixWheelPowers(1, 0, 0, cfg), 1.5, 1 / 100, { extForceBodyN: [-F, 0] }).vx.at(-1);
  };
  assert.ok(run(40) > 0, `40 N v=${run(40)}`);
  assert.ok(run(75) < 0, `75 N v=${run(75)}`);
});

// Double mass (round 2). The premise is right: the launch is traction limited, so a ~ mu*g is
// mass independent. Per wheel at 62.8 lb the stall force along the roller axis is
// Ts/(c*R) = 2.264/(0.7071*0.052) = 61.6 N, above the grip mu_s*N ~ 0.6*70 = 42 N, so the robot is NOT
// motor limited at launch (halving would need motor limiting from t = 0); it only becomes motor
// limited once ground speed passes ~0.3-0.4 of free speed, late in the 0.25 s window -> a modest drop.
// The 20% drop seen in round 1 was a time-splitting error, proved by a substep study with the old code
// (a2 in in/s^2 at 500/2000/10000/50000 Hz: 111.1 / 121.3 / 123.9 / 124.4): the motor step was implicit
// on the bare wheel inertia J, so for a rolling (stuck) wheel back-EMF was evaluated at the free-spin
// predictor speed and ~18% of torque was lost. Fixed by solving motor + contact jointly implicit
// (wheel response inertia J + h*k, motor.js). New code: a1 139.4, a2 128.3 (7.9% drop) at every substep.
test('double mass keeps launch accel within 15%', () => {
  const a = (mass) => {
    const spec = { chassis: { mass_lb: mass, footprint: FOOT }, wheels: CHASSIS_DEFAULTS.wheels };
    const { cfg, dt } = make(spec);
    const tr = simulate(dt, mixWheelPowers(1, 0, 0, cfg), 0.25, 1 / 100);
    return tr.vx[idx(tr, 0.25)] / 0.25;
  };
  const a1 = a(31.4), a2 = a(62.8);
  assert.ok(Math.abs(a2 - a1) / a1 < 0.15, `${a1} vs ${a2}`);
});

test('motor-limited launch is converged at the default 500 Hz substep (joint implicit motor+contact)', () => {
  const a = (sub) => {
    const spec = { chassis: { mass_lb: 62.8, footprint: FOOT }, wheels: CHASSIS_DEFAULTS.wheels };
    const { cfg, dt } = make(spec, { SOLVER: { maxSubstepS: sub, maxSubsteps: 100000 } });
    const tr = simulate(dt, mixWheelPowers(1, 0, 0, cfg), 0.25, 1 / 100);
    return tr.vx[idx(tr, 0.25)] / 0.25;
  };
  const coarse = a(1 / 500), fine = a(1 / 20000);
  // old split scheme: 111.1 vs 124.x (11% error); joint implicit: 128.25 vs 128.29
  assert.ok(Math.abs(coarse - fine) / fine < 0.01, `500 Hz ${coarse} vs 20 kHz ${fine}`);
});

test('slip knob = 1 cuts launch accel to 40-65%', () => {
  const a = (scale) => {
    const { cfg, dt } = make();
    dt.setMuScale(scale);
    const tr = simulate(dt, mixWheelPowers(1, 0, 0, cfg), 0.25, 1 / 100);
    return tr.vx[idx(tr, 0.25)] / 0.25;
  };
  inRange(a(0.5) / a(1), 0.4, 0.65, 'slip=1 / slip=0 (muScale 0.5 / 1.0)');
});

test('traction robot does not strafe but drives forward', () => {
  const spec = { chassis: { mass_lb: 31.4, footprint: FOOT }, wheels: CHASSIS_DEFAULTS.wheels.map((w) => ({ ...w, type: 'traction' })) };
  const { cfg, dt } = make(spec);
  let tr = simulate(dt, mixWheelPowers(0, 1, 0, cfg), 1.5, 1 / 100);
  assert.ok(Math.abs(tr.vy.at(-1)) < 2, `lateral ${tr.vy.at(-1)}`);
  dt.reset();
  tr = simulate(dt, mixWheelPowers(1, 0, 0, cfg), 1.5, 1 / 100);
  assert.ok(tr.vx.at(-1) > 30, `forward ${tr.vx.at(-1)}`);
});

test('dt invariance: 1 s launch position within 3%', () => {
  const pos = (step) => {
    const { cfg, dt } = make();
    return simulate(dt, mixWheelPowers(1, 0, 0, cfg), 1, step).x;
  };
  const a = pos(1 / 30), b = pos(1 / 120);
  assert.ok(Math.abs(a - b) / b < 0.03, `${a} vs ${b}`);
});
