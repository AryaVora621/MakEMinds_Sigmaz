// body.js - rigid body state, mass/inertia, transforms, integration (spec sections 0, 7.4, 7.8).
// SI units. A body's origin is its centre of mass. No DOM.
import { PARAMS } from './params.js';
import { shapeUnitInertia } from './shapes.js';

let nextId = 1;
export function resetBodyIds() { nextId = 1; }

// Uniform plate about the centre: m*((2hx)^2+(2hy)^2)/12 (Choreo "Estimating MOI").
export function boxInertia(mass, hx, hy) {
  const w = 2 * hx, h = 2 * hy;
  return mass * (w * w + h * h) / 12;
}

export function createBody(opts, params = PARAMS) {
  const type = opts.type || 'dynamic';
  if (type !== 'dynamic' && type !== 'static' && type !== 'kinematic') {
    throw new Error('unknown body type ' + type);
  }
  const shape = opts.shape;
  let mass = 0, invMass = 0, inertia = 0, invInertia = 0;
  if (type === 'dynamic') {
    if (!(opts.mass > 0)) throw new Error('dynamic body needs mass');
    mass = opts.mass;
    inertia = opts.inertia !== undefined
      ? opts.inertia
      : mass * shapeUnitInertia(shape) * params.robot.inertiaScale;
    invMass = 1 / mass;
    invInertia = inertia > 0 ? 1 / inertia : 0;
  } else if (type === 'kinematic') {
    mass = opts.mass || 0;
    inertia = opts.inertia || 0;
  }
  const mat = type === 'static' ? params.materials.obstacle : params.materials.robot;
  const angle = opts.angle || 0;
  return {
    id: nextId++,
    name: opts.name || '',
    type,
    shape,
    position: { x: opts.x || 0, y: opts.y || 0 },
    angle,
    q: { c: Math.cos(angle), s: Math.sin(angle) },
    linearVelocity: { x: opts.vx || 0, y: opts.vy || 0 },
    angularVelocity: opts.w || 0,
    force: { x: 0, y: 0 },
    torque: 0,
    mass, invMass, inertia, invInertia,
    friction: opts.friction !== undefined ? opts.friction : mat.friction,
    restitution: opts.restitution !== undefined ? opts.restitution : mat.restitution,
    linearDamping: opts.linearDamping !== undefined ? opts.linearDamping : params.motion.linearDamping,
    angularDamping: opts.angularDamping !== undefined ? opts.angularDamping : params.motion.angularDamping,
    categoryBits: opts.categoryBits !== undefined ? opts.categoryBits : 1,
    maskBits: opts.maskBits !== undefined ? opts.maskBits : 0xffff,
    drive: null,
    userData: opts.userData !== undefined ? opts.userData : null,
    aabb: null,
    contactImpulse: { x: 0, y: 0 },
    contactHits: [],
  };
}

export function syncTransform(body) {
  body.q.c = Math.cos(body.angle);
  body.q.s = Math.sin(body.angle);
}

export function getTransform(body) {
  return { p: { x: body.position.x, y: body.position.y }, q: { c: body.q.c, s: body.q.s } };
}

export function setPose(body, x, y, angle, resetVelocity = false) {
  body.position.x = x;
  body.position.y = y;
  body.angle = angle;
  syncTransform(body);
  if (resetVelocity) {
    body.linearVelocity.x = 0;
    body.linearVelocity.y = 0;
    body.angularVelocity = 0;
  }
}

// Force in N, optional world-space application point (adds torque about the centre).
export function applyForce(body, f, worldPoint) {
  body.force.x += f.x;
  body.force.y += f.y;
  if (worldPoint) {
    body.torque += (worldPoint.x - body.position.x) * f.y - (worldPoint.y - body.position.y) * f.x;
  }
}

// Impulse P in N*s, optional world-space point.
export function applyLinearImpulse(body, P, worldPoint) {
  body.linearVelocity.x += body.invMass * P.x;
  body.linearVelocity.y += body.invMass * P.y;
  if (worldPoint) {
    const rx = worldPoint.x - body.position.x, ry = worldPoint.y - body.position.y;
    body.angularVelocity += body.invInertia * (rx * P.y - ry * P.x);
  }
}

// Dynamic only. Semi-implicit Euler with implicit damping v *= 1/(1+h*c) (Box2D).
export function integrateVelocities(body, h, params) {
  if (body.type !== 'dynamic') return;
  const v = body.linearVelocity;
  v.x += h * body.invMass * body.force.x;
  v.y += h * body.invMass * body.force.y;
  body.angularVelocity += h * body.invInertia * body.torque;
  const kl = 1 / (1 + h * body.linearDamping);
  const ka = 1 / (1 + h * body.angularDamping);
  v.x *= kl;
  v.y *= kl;
  body.angularVelocity *= ka;
}

// Dynamic and kinematic. Clamps per-substep motion (planck maxTranslation / maxRotation).
export function integratePositions(body, h, params) {
  if (body.type === 'static') return;
  const v = body.linearVelocity;
  const tx = h * v.x, ty = h * v.y;
  const t2 = tx * tx + ty * ty;
  const maxT = params.motion.maxTranslation;
  if (t2 > maxT * maxT) {
    const k = maxT / Math.sqrt(t2);
    v.x *= k;
    v.y *= k;
  }
  const rot = h * body.angularVelocity;
  const maxR = params.motion.maxRotation;
  if (rot * rot > maxR * maxR) {
    body.angularVelocity *= maxR / Math.abs(rot);
  }
  body.position.x += h * v.x;
  body.position.y += h * v.y;
  body.angle += h * body.angularVelocity;
  syncTransform(body);
}

export function clearForces(body) {
  body.force.x = 0;
  body.force.y = 0;
  body.torque = 0;
}
