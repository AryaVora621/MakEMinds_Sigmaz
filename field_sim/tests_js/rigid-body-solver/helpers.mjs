// Test helpers for the rigid-body-solver suites. All units SI (m, kg, s, rad) unless noted.
import { createBody, makeBox, createDrive, attachDrive, setDriveTarget } from '../../web/js/physics/rigid-body-solver/index.js';

export const ROBOT_HALF_M = 0.2286; // 9 in half size, from collision.js ROBOT (sourced in the spec, section 1)

// Step the world for `seconds` of sim time in frames of dt seconds.
// onFrame(frameIndex, timeSeconds) runs after each step.
export function runFor(world, seconds, dt = 1 / 60, onFrame) {
  const n = Math.round(seconds / dt);
  for (let i = 0; i < n; i++) {
    world.step(dt);
    if (onFrame) onFrame(i, (i + 1) * dt);
  }
}

// Static axis-aligned box centred at (x, y) with half extents hx, hy (m).
export function makeWall(world, x, y, hx, hy, opts = {}) {
  return world.addBody(createBody({ type: 'static', shape: makeBox(hx, hy), x, y, ...opts }, world.params));
}

// Box robot. opts: x, y, angle, mass (kg, default 15), drive (true | false | partial drive opts),
// target {vx, vy, om} body frame, type, half {hx, hy} (m), plus any createBody option (vx, vy, restitution, friction).
export function makeRobot(world, opts = {}) {
  const { x = 0, y = 0, angle = 0, mass = 15, drive = true, target, half, type = 'dynamic', ...rest } = opts;
  const hx = half ? half.hx : ROBOT_HALF_M;
  const hy = half ? half.hy : ROBOT_HALF_M;
  const body = createBody({ type, shape: makeBox(hx, hy), x, y, angle, mass, ...rest }, world.params);
  if (drive) {
    const d = createDrive(typeof drive === 'object' ? drive : {}, world.params);
    attachDrive(body, d);
    if (target) setDriveTarget(d, target.vx || 0, target.vy || 0, target.om || 0);
  }
  return world.addBody(body);
}

// Deepest penetration (m, positive) over the arbiter points involving this body. 0 if none or only speculative.
export function penetrationOf(world, body) {
  let worst = 0;
  for (const arb of world.arbiters.values()) {
    if (arb.bodyA !== body && arb.bodyB !== body) continue;
    for (let i = 0; i < arb.pointCount; i++) {
      worst = Math.max(worst, -arb.points[i].separation);
    }
  }
  return worst;
}
