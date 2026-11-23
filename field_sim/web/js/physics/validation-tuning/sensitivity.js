// One-at-a-time sensitivity of simulator outputs to BALL and MecanumDrive parameters (central differences).
// elasticity = (y(p(1+h)) - y(p(1-h))) / (2*h*y0). For p0 = 0 the step is the schema step, applied one-sided
// (a negative restitution/drag is non-physical): elasticity = (y(step) - y0) / (step*y0).
// If y0 = 0 the denominator uses 1 (so the number is an unnormalised derivative). BALL is restored in finally.
import { BALL } from '../../balls.js';
import { MecanumDrive } from '../../driver.js';
import { SURFACES } from '../ball-bounce-rolling/params.js';
import { SENS, PARAM_SCHEMA, schemaEntry } from './params.js';
import { makeScenario } from './scenarios.js';

function runBall(name, maxSteps, stopFn) {
  const sc = makeScenario(name);
  for (let i = 0; i < maxSteps; i++) {
    const prevVz = sc.ball.vz, prevX = sc.ball.x;
    sc.step(i);
    if (stopFn(sc.ball, prevVz, prevX, i)) break;
  }
  return sc;
}

export const OUTPUTS = {
  // peak centre height after the first bounce minus r (in), drop40 from z = 40 in
  bounceRise(_d) {
    const sc = makeScenario('drop40'), b = sc.ball;
    let bounced = false, peak = 0;
    for (let i = 0; i < sc.steps; i++) {
      const vzPrev = b.vz; sc.step(i);
      if (!bounced && vzPrev < 0 && b.vz > 0) bounced = true;
      if (bounced) { peak = Math.max(peak, b.z); if (b.vz < 0) break; }
    }
    return peak - b.r;
  },
  // distance (in) rolled from x = 20 over 240 steps at 40 in/s initial
  rollDistance(_d) { const sc = runBall('roll', 240, () => false); return sc.ball.x - 20; },
  // horizontal speed lost (in/s) in 15 steps from 500 in/s (drag only; gravity acts on vz)
  airSpeedLoss(_d) { const sc = runBall('airDrag', 15, () => false); return 500 - sc.ball.vx; },
  // x (in) of the first floor bounce of shotArc (stepped up to 240 steps, longer than the 90-step golden)
  shotRange(_d) {
    const sc = runBall('shotArc', 240, (b, vzPrev) => vzPrev < 0 && b.vz > 0);
    return sc.ball.x;
  },
  // time (s) to reach 0.9*maxSpeed with fwd = 1, linearly interpolated between steps
  driveT90(driveOpts) {
    const sc = makeScenario('driveStep', { driveOpts }), target = 0.9 * sc.drive.maxSpeed;
    let prev = 0;
    for (let i = 0; i < 1200; i++) {
      sc.drive.step({ fwd: 1 }, sc.dt, sc.state);
      const bv = sc.state.bodyV[0];
      if (bv >= target) { const f = (target - prev) / (bv - prev); return (i + f) * sc.dt; }
      prev = bv;
    }
    return NaN;
  },
};

function nominalOf(key) {
  const e = schemaEntry(key);
  return e.group === 'ball' ? BALL[e.field] : new MecanumDrive()[e.field];
}
function driveDefaults() {
  const d = new MecanumDrive(), o = {};
  for (const e of PARAM_SCHEMA) if (e.group === 'drive') o[e.field] = d[e.field];
  return o;   // every field pinned so perturbing maxAccel does not silently move the maxDecel default
}

// balls.js reads restitution/friction from SURFACES (physics/ball-bounce-rolling/params.js), NOT from BALL.restitution*,
// BALL.rollFriction or BALL.slideFriction, which are only initial copies. To make the sensitivity real, the perturbation
// is mirrored into SURFACES for the run and restored in finally. (Finding for the integrator: the BALL fields are dead.)
const SURFACE_MIRROR = {
  restitutionFloor: ['floor', 'eRef'], restitutionWall: ['wall', 'eRef'], restitutionBall: ['ball', 'eRef'],
  restitutionRobot: ['robot', 'eRef'], restitutionHive: ['hive', 'eRef'],
  rollFriction: ['floor', 'crr'], slideFriction: ['floor', 'muSlide'],
};

function evalAt(key, value, outName) {
  const e = schemaEntry(key);
  const saved = {};
  let savedSurf = null;
  for (const e2 of PARAM_SCHEMA) if (e2.group === 'ball') saved[e2.field] = BALL[e2.field];
  const dOpts = driveDefaults();
  try {
    if (e.group === 'ball') {
      BALL[e.field] = value;
      const m = SURFACE_MIRROR[e.field];
      if (m && SURFACES[m[0]]) { savedSurf = { s: SURFACES[m[0]], f: m[1], v: SURFACES[m[0]][m[1]] }; SURFACES[m[0]][m[1]] = value; }
    } else dOpts[e.field] = value;
    return OUTPUTS[outName](dOpts);
  } finally {
    for (const k of Object.keys(saved)) BALL[k] = saved[k];
    if (savedSurf) savedSurf.s[savedSurf.f] = savedSurf.v;
  }
}

export function sensitivity(opts = {}) {
  const h = opts.relStep ?? SENS.relStep;
  const params = opts.params || PARAM_SCHEMA.map(e => e.key);
  const outputs = opts.outputs || Object.keys(OUTPUTS);
  const rows = [], score = new Map();
  for (const key of params) {
    const e = schemaEntry(key);
    if (!e) throw new Error('unknown parameter: ' + key);
    const p0 = nominalOf(key);
    for (const out of outputs) {
      if (!OUTPUTS[out]) throw new Error('unknown output: ' + out);
      const y0 = evalAt(key, p0, out);
      const den = y0 !== 0 ? y0 : 1;
      let yPlus, yMinus, elasticity;
      if (p0 === 0) {
        yPlus = evalAt(key, e.step, out); yMinus = y0;
        elasticity = (yPlus - y0) / (e.step * den);
      } else {
        yPlus = evalAt(key, p0 * (1 + h), out); yMinus = evalAt(key, p0 * (1 - h), out);
        elasticity = (yPlus - yMinus) / (2 * h * den);
      }
      rows.push({ param: key, nominal: p0, output: out, y0, yPlus, yMinus, elasticity });
      if (Number.isFinite(elasticity)) score.set(key, Math.max(score.get(key) ?? 0, Math.abs(elasticity)));
    }
  }
  const ranking = [...score].map(([param, s]) => ({ param, score: s })).sort((a, b) => b.score - a.score);
  return { rows, ranking };
}
