// Robot-vs-field collision facade. Pure ES module (browser + node). Inches, Pedro frame (x right, y far, z up).
// Legacy API is re-exported unchanged from physics/wall-collisions/legacy.js; the new contact solver is added below.
//
// Integration note for the app.js owner (NOT applied here):
//   import { createWallContact, stepRobotContact, resetContactCache } from './collision.js';
//   After app.js:112 add:   const wallCtx = createWallContact(obstacles);
//   In driveStep, replace lines 401-410 with:
//     const r = stepRobotContact(S, dt, wallCtx); if (r.hit) S.bump = (S.bump || 0) + 1;
//   placeRobot keeps resolveRobot and also calls resetContactCache(wallCtx).
import { makeParams, WALL_PARAMS } from './physics/wall-collisions/params.js';
import { robotPolygon, bodyMass } from './physics/wall-collisions/geometry.js';
import { prepareObstacles } from './physics/wall-collisions/manifold.js';
import { createContactCache, robotBodyFromState, writeBodyToState, contactStep } from './physics/wall-collisions/step.js';

export { ROBOT, buildObstacles, robotCorners, resolveRobot, slideVelocity, isFree } from './physics/wall-collisions/legacy.js';

// The legacy kinematic fallback (legacy.js resolveRobot/isFree) reads box centre/half-extent fields
// (cx, cy, hx, hy) that only buildObstacles() fills in. A raw {minX,minY,maxX,maxY} box gives
// undefined - undefined = NaN there, which used to leak NaN poses out of the fallback. Fill them here
// (no z filtering: the same boxes feed prepareObstacles, so both paths see identical geometry).
function legacyObstacles(obstacles) {
  const o = obstacles || {};
  const boxes = (o.boxes || []).map((b) => (
    Number.isFinite(b.cx) && Number.isFinite(b.cy) && Number.isFinite(b.hx) && Number.isFinite(b.hy)
      ? b
      : { ...b, cx: (b.minX + b.maxX) / 2, cy: (b.minY + b.maxY) / 2, hx: (b.maxX - b.minX) / 2, hy: (b.maxY - b.minY) / 2 }));
  // No play-area bounds given: unbounded (legacy resolveBounds would otherwise throw on undefined).
  const bounds = o.bounds || { minX: -Infinity, minY: -Infinity, maxX: Infinity, maxY: Infinity };
  return { ...o, bounds, boxes, circles: o.circles || [] };
}

export function createWallContact(obstacles, overrides) {
  const P = makeParams(overrides || {});
  const obs = legacyObstacles(obstacles);
  return { P, poly: robotPolygon(P.robot), mass: bodyMass(P), prepared: prepareObstacles(obs, P), cache: createContactCache(), obstacles: obs };
}

export function stepRobotContact(S, dt, ctx) {
  const body = robotBodyFromState(S, ctx.mass);
  const report = contactStep(body, ctx, dt);
  writeBodyToState(body, S);
  return report;
}

export function resetContactCache(ctx) { ctx.cache.clear(); }

export function setRobotGeometry(ctx, geom) {
  Object.assign(ctx.P.robot, geom || {});
  ctx.poly = robotPolygon(ctx.P.robot);
  ctx.mass = bodyMass(ctx.P);
  ctx.cache.clear();
}
