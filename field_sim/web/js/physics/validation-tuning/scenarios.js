// Deterministic validation scenarios. Units: PEDRO inches, s, rad. All run at dt = 1/60 (BENCH.dt, ASSUMED).
// A scenario owns its world/drive; step(i) advances one frame, sample() returns the row for `fields`.
import { BallWorld, BALL } from '../../balls.js';
import { MecanumDrive, integrate } from '../../driver.js';
import { BENCH } from './params.js';
import { mulberry32 } from './rng.js';
import { FIELD_BOUNDS } from './node_env.js';

const DT = BENCH.dt;   // 1/60 s
const BALL_FIELDS = ['t', 'x', 'y', 'z', 'vx', 'vy', 'vz'];
const DRIVE_FIELDS = ['t', 'x', 'y', 'h', 'bvx', 'bvy', 'bw'];

export const SCENARIO_NAMES = ['drop40', 'wallBounce', 'roll', 'airDrag', 'shotArc', 'driveStep', 'driveTurn', 'pile30', 'pushThrough'];
// Non-chaotic scenes that have golden files.
export const GOLDEN_SCENES = ['drop40', 'wallBounce', 'roll', 'airDrag', 'shotArc', 'driveStep', 'driveTurn'];

function defaultEnv() { return { bounds: { ...FIELD_BOUNDS } }; }

// Single-ball closed scene. start = [x,y,z] (z may be the string 'r' meaning resting on the floor), vel = [vx,vy,vz].
function singleBall(name, steps, start, vel, env, extra = {}) {
  const world = new BallWorld(env || defaultEnv(), null);
  const z = start[2] === 'r' ? BALL.r : start[2];
  const ball = world.addBall('pollen', [start[0], start[1], z], vel, 'pollen');
  const sc = {
    name, dt: DT, steps, fields: BALL_FIELDS, closed: true, chaotic: false, world, ball, drive: null, state: { t: 0 },
    step(i) { world.step(DT); sc.state.t = (i + 1) * DT; },
    sample() { return [sc.state.t, ball.x, ball.y, ball.z, ball.vx, ball.vy, ball.vz]; },
    ...extra,
  };
  return sc;
}

function driveScene(name, steps, cmdFn, driveOpts) {
  const drive = new MecanumDrive(driveOpts || {});
  const state = { x: 72, y: 72, h: 0, bodyV: [0, 0, 0], t: 0 };
  const sc = {
    name, dt: DT, steps, fields: DRIVE_FIELDS, closed: true, chaotic: false, world: null, drive, state,
    step(i) {
      const r = drive.step(cmdFn(i), DT, state);
      integrate(state, r.vx, r.vy, r.om, DT);
      state.t = (i + 1) * DT;
    },
    sample() { return [state.t, state.x, state.y, state.h, state.bodyV[0], state.bodyV[1], state.bodyV[2]]; },
  };
  return sc;
}

/** Build a scenario by name. opts.env overrides the world env; opts.seed seeds chaotic scenes (default 1);
 *  opts.driveOpts overrides MecanumDrive options for the drive scenes. */
export function makeScenario(name, opts = {}) {
  const env = opts.env || null;
  const seed = opts.seed ?? 1;
  switch (name) {
    case 'drop40': return singleBall(name, 180, [70, 70, 40], [0, 0, 0], env);
    case 'wallBounce': return singleBall(name, 120, [30, 72, 'r'], [-200, 0, 0], env);
    case 'roll': return singleBall(name, 240, [20, 72, 'r'], [40, 0, 0], env);   // analytic stop distance v^2/(2*rollFriction*g) = 51.8 in
    case 'airDrag': return singleBall(name, 15, [8, 72, 40], [500, 0, 0], env);
    case 'shotArc': return singleBall(name, 90, [20, 72, 12], [300, 0, 300], env);
    case 'driveStep': return driveScene(name, 120, i => ({ fwd: i < 60 ? 1 : 0 }), opts.driveOpts);
    case 'driveTurn': return driveScene(name, 120, i => ({ turn: i < 60 ? 1 : 0 }), opts.driveOpts);
    case 'pile30': {
      const rng = mulberry32(seed);
      const world = new BallWorld(env || defaultEnv(), rng);
      for (let i = 0; i < 30; i++) {
        // 30 x 30 in region centred on the field, z from 6 to 60 in (as in sim_physics buildWorld)
        world.addBall('pollen', [57 + rng() * 30, 57 + rng() * 30, 6 + rng() * 54], [0, 0, 0], 'pollen');
      }
      const sc = {
        name, dt: DT, steps: 600, fields: ['t', 'meanZ', 'meanSpeed', 'maxZ'], closed: true, chaotic: true, world, drive: null, state: { t: 0 },
        step(i) { world.step(DT); sc.state.t = (i + 1) * DT; },
        sample() {
          let sz = 0, sv = 0, mz = 0;
          for (const b of world.balls) { sz += b.z; sv += Math.hypot(b.vx, b.vy, b.vz); mz = Math.max(mz, b.z); }
          const n = world.balls.length || 1;
          return [sc.state.t, sz / n, sv / n, mz];
        },
      };
      return sc;
    }
    case 'pushThrough': {
      const rng = mulberry32(seed);
      const world = new BallWorld(env || defaultEnv(), rng);
      // 60 balls on a jittered 10 x 6 grid (4 in pitch, +-0.4 in jitter) resting on the floor in the robot's path (y = 72)
      for (let i = 0; i < 60; i++) {
        const gx = i % 10, gy = (i / 10) | 0;
        world.addBall('pollen', [40 + gx * 4 + (rng() - 0.5) * 0.8, 72 - 10 + gy * 4 + (rng() - 0.5) * 0.8, BALL.r], [0, 0, 0], 'pollen');
      }
      const drive = new MecanumDrive(opts.driveOpts || {});
      const state = { x: 14, y: 72, h: 0, bodyV: [0, 0, 0], t: 0 };
      const dims = { halfX: 9, halfY: 9, height: 14 };
      const sc = {
        name, dt: DT, steps: 240, fields: ['t', 'rx', 'ry', 'meanBallX', 'maxBallX', 'meanBallSpeed'], closed: false, chaotic: true, world, drive, state, dims,
        step(i) {
          // drive at maxSpeed (60 in/s) along y=72; stop the command before the far wall (x 110, ASSUMED)
          const r = drive.step({ fwd: state.x < 110 ? 1 : 0 }, DT, state);
          integrate(state, r.vx, r.vy, r.om, DT);
          world.setRobot({ x: state.x, y: state.y, h: state.h, vx: r.vx, vy: r.vy, om: r.om }, dims);
          world.step(DT);
          state.t = (i + 1) * DT;
        },
        sample() {
          let sx = 0, mx = 0, sv = 0;
          for (const b of world.balls) { sx += b.x; mx = Math.max(mx, b.x); sv += Math.hypot(b.vx, b.vy, b.vz); }
          const n = world.balls.length || 1;
          return [state.t, state.x, state.y, sx / n, mx, sv / n];
        },
      };
      return sc;
    }
    default: throw new Error('unknown scenario: ' + name);
  }
}

/** Run a scenario to completion; onStep(sc, i) is called after each step (before sample). Returns {fields, data}. */
export function runScenario(sc, opts = {}) {
  const data = [];
  for (let i = 0; i < sc.steps; i++) {
    sc.step(i);
    if (opts.onStep) opts.onStep(sc, i);
    data.push(sc.sample());
  }
  return { fields: sc.fields.slice(), data };
}
