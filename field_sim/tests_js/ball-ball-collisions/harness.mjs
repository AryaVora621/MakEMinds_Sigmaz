// Test harness for ball-ball-collisions: a mini integrator mirroring balls.js (units: in, s, kg, rad/s).
import { BB_PARAMS, G_IN } from '../../web/js/physics/ball-ball-collisions/params.js';
import { massOf, inertiaOf, syncRollingOnSurface, dampSpin } from '../../web/js/physics/ball-ball-collisions/bodies.js';

const RADIUS = { pollen: 1.391, nectar: 1.7995 }; // in, from balls.js BALL_KINDS (sourced there)
let idCounter = 0;

export function mkBall(kind, x, y, z, vx = 0, vy = 0, vz = 0, extra = {}) {
  return {
    id: `t${idCounter++}`, kind, x, y, z, vx, vy, vz, r: RADIUS[kind] ?? RADIUS.pollen,
    state: 'free', sleeping: false, sleepT: 0, _c: false, ...extra,
  };
}

export function kineticEnergy(balls, params = BB_PARAMS) {
  let ke = 0;
  for (const b of balls) {
    if (b.pinned === true) continue;
    const m = massOf(b.kind, params), I = inertiaOf(b.kind, b.r, params);
    ke += 0.5 * m * (b.vx * b.vx + b.vy * b.vy + b.vz * b.vz);
    ke += 0.5 * I * ((b.wx || 0) ** 2 + (b.wy || 0) ** 2 + (b.wz || 0) ** 2);
  }
  return ke;
}

export function maxPenetration(balls) {
  let worst = 0;
  for (let i = 0; i < balls.length; i++) {
    const a = balls[i];
    for (let j = i + 1; j < balls.length; j++) {
      const b = balls[j];
      const rr = a.r + b.r;
      const dx = a.x - b.x; if (dx > rr || dx < -rr) continue;
      const dy = a.y - b.y; if (dy > rr || dy < -rr) continue;
      const dz = a.z - b.z; if (dz > rr || dz < -rr) continue;
      const pen = rr - Math.sqrt(dx * dx + dy * dy + dz * dz);
      if (pen > worst) worst = pen;
    }
  }
  return worst;
}

/** opts: {system, params, seconds, dt, substeps, gravity, floor, box, onStep(stepIndex, balls)} */
export function runWorld(balls, opts) {
  const { system, seconds, dt = 1 / 60, substeps = 4, gravity = G_IN, floor = true, box = null, onStep = null } = opts;
  const params = opts.params || BB_PARAMS;
  const steps = Math.round(seconds / dt), h = dt / substeps;
  const keTrace = [], penTrace = [];
  // Static floor/box half-spaces are solved inside the ball solver (system.js INTEGRATION RECIPE,
  // setPlanes). The after-solve clamps below stay as a safety net, as balls.js would keep them.
  // opts.solverPlanes === false reproduces the old clamp-only coupling for comparison.
  if (opts.solverPlanes !== false && typeof system.setPlanes === 'function') {
    const planes = [];
    if (floor) planes.push({ nx: 0, ny: 0, nz: 1, d: 0 });
    if (box) {
      planes.push({ nx: 1, ny: 0, nz: 0, d: box.minX }, { nx: -1, ny: 0, nz: 0, d: -box.maxX });
      planes.push({ nx: 0, ny: 1, nz: 0, d: box.minY }, { nx: 0, ny: -1, nz: 0, d: -box.maxY });
    }
    system.setPlanes(planes);
  }
  const t0 = performance.now();
  for (let s = 0; s < steps; s++) {
    system.beginStep(balls, dt);
    for (let k = 0; k < substeps; k++) {
      for (const b of balls) {
        b._c = false;
        if (b.state !== 'free' || b.pinned === true || b.sleeping === true) continue;
        b.x += b.vx * h; b.y += b.vy * h;
        b.z += b.vz * h - 0.5 * gravity * h * h;
        b.vz -= gravity * h;
      }
      system.substep(balls, h);
      for (const b of balls) {
        if (b.state !== 'free' || b.pinned === true || b.sleeping === true) continue;
        if (box) {
          if (b.x < box.minX + b.r) { b.x = box.minX + b.r; if (b.vx < 0) b.vx = 0; b._c = true; }
          if (b.x > box.maxX - b.r) { b.x = box.maxX - b.r; if (b.vx > 0) b.vx = 0; b._c = true; }
          if (b.y < box.minY + b.r) { b.y = box.minY + b.r; if (b.vy < 0) b.vy = 0; b._c = true; }
          if (b.y > box.maxY - b.r) { b.y = box.maxY - b.r; if (b.vy > 0) b.vy = 0; b._c = true; }
        }
        if (floor) {
          if (b.z < b.r) { b.z = b.r; if (b.vz < 0) b.vz = 0; }
          if (b.z <= b.r + 0.01) {
            b._c = true;
            const sp = Math.hypot(b.vx, b.vy);
            if (sp > 0) { const f = Math.max(0, sp - params.muRoll * gravity * h) / sp; b.vx *= f; b.vy *= f; }
            syncRollingOnSurface(b, 0, 0, 1, h, params);
            dampSpin(b, h, params);
          }
        }
      }
      system.endSubstep(balls, h);
    }
    keTrace.push(kineticEnergy(balls, params));
    penTrace.push(maxPenetration(balls));
    if (onStep) onStep(s, balls);
  }
  return { system, balls, keTrace, penTrace, wallMs: performance.now() - t0 };
}
