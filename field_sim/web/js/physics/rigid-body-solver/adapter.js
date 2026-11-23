// Inch-world bridge between app.js (Pedro frame, inches, in/s) and the SI solver.
// IN_TO_M = 0.0254 m/in: exact by definition of the inch.
import { World } from './world.js';
import { createBody, boxInertia, setPose } from './body.js';
import { createDrive, attachDrive, setDriveTarget } from './drive.js';
import { makeBox, makeCircle, makePolygon } from './shapes.js';
import { wrapAngle } from './vec2.js';

export const IN_TO_M = 0.0254;

export function createFieldWorld(colliders, opts = {}) {
  const world = new World(opts.params || {});
  const P = world.params;
  const zcap = opts.zCapIn ?? colliders.z_cap ?? P.field.zCapIn; // in
  const statics = [];
  const keep = (o) => !(o.zMin >= zcap || o.zMax <= 0);
  const add = (shape, x, y, kind, mat) => {
    const body = createBody({
      type: 'static', shape, x, y,
      friction: mat.friction, restitution: mat.restitution,
      categoryBits: P.filter.CAT_STATIC, maskBits: P.filter.MASK_ALL,
      userData: { kind },
    }, P);
    world.addBody(body);
    statics.push(body);
    return body;
  };
  for (const b of colliders.boxes || []) {
    if (!keep(b)) continue;
    add(makeBox((b.maxX - b.minX) / 2 * IN_TO_M, (b.maxY - b.minY) / 2 * IN_TO_M),
      (b.minX + b.maxX) / 2 * IN_TO_M, (b.minY + b.maxY) / 2 * IN_TO_M, b.kind, P.materials.obstacle);
  }
  for (const c of colliders.circles || []) {
    if (!keep(c)) continue;
    add(makeCircle(c.r * IN_TO_M), c.x * IN_TO_M, c.y * IN_TO_M, c.kind, P.materials.obstacle);
  }
  for (const pg of colliders.polygons || []) {
    if (!keep(pg)) continue;
    const pts = pg.points.map((q) => ({ x: q[0] * IN_TO_M, y: q[1] * IN_TO_M }));
    const c = makePolygon(pts).centroid;
    const rel = pts.map((q) => ({ x: q.x - c.x, y: q.y - c.y }));
    add(makePolygon(rel), c.x, c.y, pg.kind, P.materials.obstacle);
  }
  const bd = colliders.bounds;
  if (bd) {
    const t = P.field.wallThickness; // m
    const x0 = bd.minX * IN_TO_M, x1 = bd.maxX * IN_TO_M;
    const y0 = bd.minY * IN_TO_M, y1 = bd.maxY * IN_TO_M;
    const cx = (x0 + x1) / 2, cy = (y0 + y1) / 2;
    const hw = (x1 - x0) / 2 + t, hh = (y1 - y0) / 2 + t;
    add(makeBox(t / 2, hh), x0 - t / 2, cy, 'wall', P.materials.wall);
    add(makeBox(t / 2, hh), x1 + t / 2, cy, 'wall', P.materials.wall);
    add(makeBox(hw, t / 2), cx, y0 - t / 2, 'wall', P.materials.wall);
    add(makeBox(hw, t / 2), cx, y1 + t / 2, 'wall', P.materials.wall);
  }
  return { world, statics };
}

export function addRobot(world, opts = {}) {
  const P = world.params;
  const hx = (opts.halfXIn ?? P.robot.halfXIn) * IN_TO_M;
  const hy = (opts.halfYIn ?? P.robot.halfYIn) * IN_TO_M;
  const mass = opts.massKg ?? P.robot.massKg;
  const inertia = opts.inertia ?? P.robot.inertiaScale * boxInertia(mass, hx, hy);
  const body = createBody({
    type: opts.type || 'dynamic', shape: makeBox(hx, hy),
    x: (opts.x || 0) * IN_TO_M, y: (opts.y || 0) * IN_TO_M, angle: opts.h || 0,
    mass, inertia,
    friction: opts.friction, restitution: opts.restitution,
    categoryBits: P.filter.CAT_ROBOT, maskBits: P.filter.MASK_ALL,
    userData: { role: opts.role || 'self', kind: 'robot' },
    name: opts.name || '',
  }, P);
  if (opts.drive !== false) {
    const dopts = (opts.drive && typeof opts.drive === 'object') ? opts.drive : {};
    attachDrive(body, createDrive(dopts, P));
  }
  world.addBody(body);
  return body;
}

// cmd in body frame: in/s, +y left, rad/s
export function setRobotCommandBody(body, cmd) {
  if (!body.drive) return;
  setDriveTarget(body.drive, (cmd.vx || 0) * IN_TO_M, (cmd.vy || 0) * IN_TO_M, cmd.om || 0);
}

// cmd in field frame: in/s, rad/s (as S.vx, S.vy, S.om in app.js)
export function setRobotCommand(body, cmd) {
  if (!body.drive) return;
  const c = body.q.c, s = body.q.s;
  const vx = cmd.vx || 0, vy = cmd.vy || 0;
  setDriveTarget(body.drive, (c * vx + s * vy) * IN_TO_M, (-s * vx + c * vy) * IN_TO_M, cmd.om || 0);
}

export function readRobot(world, body) {
  const normals = [];
  for (const hit of world.contactsOf(body)) {
    const n = hit.normal;
    if (!normals.some((m) => m[0] * n.x + m[1] * n.y > 0.9999)) normals.push([n.x, n.y]);
  }
  const f = world.contactForceOf(body);
  const d = body.drive;
  return {
    x: body.position.x / IN_TO_M,
    y: body.position.y / IN_TO_M,
    h: wrapAngle(body.angle),
    vx: body.linearVelocity.x / IN_TO_M,
    vy: body.linearVelocity.y / IN_TO_M,
    om: body.angularVelocity,
    hit: normals.length > 0,
    normals,
    contactForceN: f.magnitude,
    contactForce: { x: f.x, y: f.y },
    driveUsage: d ? d.usage : 0,
    slipping: d ? d.slipping : false,
    driveForce: d ? { f: d.force.f, s: d.force.s, w: d.force.w } : { f: 0, s: 0, w: 0 },
  };
}

export function teleportRobot(body, pose, resetVelocity = true) {
  setPose(body, pose.x * IN_TO_M, pose.y * IN_TO_M, pose.h ?? body.angle, resetVelocity);
  if (resetVelocity && body.drive) body.drive.impulse = { f: 0, s: 0, w: 0 };
}

export function stepWorld(world, dt) {
  world.step(Math.min(dt, world.params.step.maxFrameDt));
}
