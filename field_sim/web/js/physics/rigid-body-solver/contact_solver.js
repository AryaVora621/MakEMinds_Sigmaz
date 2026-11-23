// contact_solver.js - sequential-impulse contact solver.
// Sources:
//  - Catto 2005 "Fast and Simple Physics using Sequential Impulses" (accumulated impulse clamp, warm starting).
//  - box2d-lite Arbiter.cpp PreStep/ApplyImpulse (kNormal, Coulomb clamp, Baumgarte bias):
//    https://github.com/erincatto/box2d-lite
//  - Box2D v3 solver (b2MakeSoft, soft contacts, speculative bias s/h, relax, restitution):
//    https://github.com/erincatto/box2d (src/solver.c, contact_solver.c, constants.h)
//  - planck.js / Box2D v2.4 nonlinear Gauss-Seidel b2ContactSolver::SolvePositionConstraints (solvePositions).
// Units: SI. Impulse P = lambda*n: vA -= invMassA*P, wA -= invIA*cross(rA,P), vB += invMassB*P, wB += invIB*cross(rB,P).
// Tangent t = rightPerp(n) = (n.y, -n.x). No DOM.

import { syncTransform } from './body.js';

function cross(a, b) { return a.x * b.y - a.y * b.x; }

export function softCoefficients(hertz, zeta, h) {
  if (hertz === 0) return { biasRate: 0, massScale: 0, impulseScale: 0 };
  const omega = 2 * Math.PI * hertz;
  const a1 = 2 * zeta + h * omega;
  const a2 = h * omega * a1;
  const a3 = 1 / (1 + a2);
  return { biasRate: omega / a1, massScale: a2 * a3, impulseScale: a3 };
}

// Apply impulse (px,py) at the point with arms rA, rB (sign rules above).
function applyImpulse(a, b, rA, rB, px, py) {
  const va = a.linearVelocity, vb = b.linearVelocity;
  va.x -= a.invMass * px; va.y -= a.invMass * py;
  a.angularVelocity -= a.invInertia * (rA.x * py - rA.y * px);
  vb.x += b.invMass * px; vb.y += b.invMass * py;
  b.angularVelocity += b.invInertia * (rB.x * py - rB.y * px);
}

// dv = vB + cross(wB,rB) - vA - cross(wA,rA)
function relVel(a, b, rA, rB) {
  return {
    x: b.linearVelocity.x - b.angularVelocity * rB.y - a.linearVelocity.x + a.angularVelocity * rA.y,
    y: b.linearVelocity.y + b.angularVelocity * rB.x - a.linearVelocity.y - a.angularVelocity * rA.x,
  };
}

export function prepareContacts(arbiters, h, params) {
  const soft = softCoefficients(params.contact.contactHertz, params.contact.contactDampingRatio, h);
  for (const arb of arbiters) {
    const a = arb.bodyA, b = arb.bodyB;
    const n = arb.normal;
    const t = { x: n.y, y: -n.x };
    arb.soft = { biasRate: soft.biasRate, massScale: soft.massScale, impulseScale: soft.impulseScale };
    for (let i = 0; i < arb.pointCount; i++) {
      const cp = arb.points[i];
      const p = cp.point;
      cp.rA = { x: p.x - a.position.x, y: p.y - a.position.y };
      cp.rB = { x: p.x - b.position.x, y: p.y - b.position.y };
      const rnA = cross(cp.rA, n), rnB = cross(cp.rB, n);
      const kN = a.invMass + b.invMass + a.invInertia * rnA * rnA + b.invInertia * rnB * rnB;
      cp.normalMass = kN > 0 ? 1 / kN : 0;
      const rtA = cross(cp.rA, t), rtB = cross(cp.rB, t);
      const kT = a.invMass + b.invMass + a.invInertia * rtA * rtA + b.invInertia * rtB * rtB;
      cp.tangentMass = kT > 0 ? 1 / kT : 0;
      const dv = relVel(a, b, cp.rA, cp.rB);
      cp.relativeVelocity = dv.x * n.x + dv.y * n.y;
      cp.maxNormalImpulse = 0;
      // NGS anchors (planck Solver: local points and normal in each body frame at step start)
      const dxA = p.x - a.position.x, dyA = p.y - a.position.y;
      const dxB = p.x - b.position.x, dyB = p.y - b.position.y;
      cp.localA = { x: a.q.c * dxA + a.q.s * dyA, y: -a.q.s * dxA + a.q.c * dyA };
      cp.localB = { x: b.q.c * dxB + b.q.s * dyB, y: -b.q.s * dxB + b.q.c * dyB };
      cp.localNormal = { x: a.q.c * n.x + a.q.s * n.y, y: -a.q.s * n.x + a.q.c * n.y };
      cp.baseSeparation = cp.separation;
    }
  }
}

export function warmStartContacts(arbiters) {
  for (const arb of arbiters) {
    const n = arb.normal;
    const tx = n.y, ty = -n.x;
    for (let i = 0; i < arb.pointCount; i++) {
      const cp = arb.points[i];
      const px = cp.normalImpulse * n.x + cp.tangentImpulse * tx;
      const py = cp.normalImpulse * n.y + cp.tangentImpulse * ty;
      applyImpulse(arb.bodyA, arb.bodyB, cp.rA, cp.rB, px, py);
    }
  }
}

export function solveContacts(arbiters, h, params, useBias) {
  const c = params.contact;
  const mode = params.solver.positionMode;
  for (const arb of arbiters) {
    const a = arb.bodyA, b = arb.bodyB;
    const n = arb.normal;
    const t = { x: n.y, y: -n.x };
    for (let i = 0; i < arb.pointCount; i++) {
      const cp = arb.points[i];
      // friction first (Box2D v2.4 order); clamp against current normal impulse
      {
        const dv = relVel(a, b, cp.rA, cp.rB);
        const vt = dv.x * t.x + dv.y * t.y;
        const lambda = -cp.tangentMass * vt;
        const maxF = arb.friction * cp.normalImpulse;
        const newPt = Math.max(-maxF, Math.min(maxF, cp.tangentImpulse + lambda));
        const d = newPt - cp.tangentImpulse;
        cp.tangentImpulse = newPt;
        applyImpulse(a, b, cp.rA, cp.rB, d * t.x, d * t.y);
      }
      // normal
      {
        const dv = relVel(a, b, cp.rA, cp.rB);
        const vn = dv.x * n.x + dv.y * n.y;
        const s = cp.separation;
        let bias = 0, mScale = 1, iScale = 0;
        if (s > 0) {
          bias = s / h; // speculative (Box2D v3)
        } else if (useBias && mode === 'soft') {
          bias = Math.max(arb.soft.biasRate * s, -c.maxContactPushSpeed);
          mScale = arb.soft.massScale; iScale = arb.soft.impulseScale;
        } else if (useBias && mode === 'baumgarte') {
          bias = (c.baumgarte / h) * Math.min(0, s + c.allowedPenetration);
        }
        const impulse = -cp.normalMass * mScale * (vn + bias) - iScale * cp.normalImpulse;
        const newPn = Math.max(cp.normalImpulse + impulse, 0);
        const d = newPn - cp.normalImpulse;
        cp.normalImpulse = newPn;
        cp.maxNormalImpulse = Math.max(cp.maxNormalImpulse, newPn);
        applyImpulse(a, b, cp.rA, cp.rB, d * n.x, d * n.y);
      }
    }
  }
}

// Box2D v3 b2ApplyRestitution, iterated.
// Target per point: post-step normal velocity vn = -e * relativeVelocity, where relativeVelocity is the
// PRE-solve approach speed stored by prepareContacts (as Box2D v3). Accumulated impulse stays >= 0.
// Box2D v3 does ONE sequential pass. With a 2-point face contact that pass is order dependent: the
// second point's impulse un-does part of the first point's rebound through the angular coupling, so a
// square body hitting a wall square-on came back at 0.68 m/s (target 1.0 m/s) with a spurious spin of
// 1.28 rad/s. Repeating the pass is projected Gauss-Seidel on the same restitution constraints and
// converges to the simultaneous (symmetric) solution. For the 18 in, 15 kg square robot the coupling
// ratio per pass is |1 - m*r^2/I| / (1 + m*r^2/I) = 0.5/2.5 = 0.2, so 4 passes leave < 0.2^4 = 0.16 %
// error. Stops early once every point's change is below restitutionTolerance.
export function applyRestitution(arbiters, params) {
  const thr = params.contact.restitutionThreshold;
  const iters = params.solver.restitutionIterations;
  const tol = params.solver.restitutionTolerance;
  for (const arb of arbiters) {
    if (arb.restitution === 0) continue;
    const a = arb.bodyA, b = arb.bodyB;
    const n = arb.normal;
    for (let k = 0; k < iters; k++) {
      let maxDv = 0; // m/s, largest normal velocity error fixed in this pass
      for (let i = 0; i < arb.pointCount; i++) {
        const cp = arb.points[i];
        if (cp.relativeVelocity > -thr || cp.maxNormalImpulse === 0) continue;
        const dv = relVel(a, b, cp.rA, cp.rB);
        const vn = dv.x * n.x + dv.y * n.y;
        const err = vn + arb.restitution * cp.relativeVelocity;
        const impulse = -cp.normalMass * err;
        const newPn = Math.max(cp.normalImpulse + impulse, 0);
        const d = newPn - cp.normalImpulse;
        if (d !== 0) maxDv = Math.max(maxDv, Math.abs(err));
        cp.normalImpulse = newPn;
        applyImpulse(a, b, cp.rA, cp.rB, d * n.x, d * n.y);
      }
      if (maxDv < tol) break;
    }
  }
}

// planck / Box2D v2.4 NGS position correction. Returns min separation (Infinity if no points).
export function solvePositions(arbiters, params) {
  const c = params.contact;
  let minSep = Infinity;
  for (const arb of arbiters) {
    const a = arb.bodyA, b = arb.bodyB;
    for (let i = 0; i < arb.pointCount; i++) {
      const cp = arb.points[i];
      const qa = a.q, qb = b.q;
      const pA = { x: a.position.x + qa.c * cp.localA.x - qa.s * cp.localA.y,
                   y: a.position.y + qa.s * cp.localA.x + qa.c * cp.localA.y };
      const pB = { x: b.position.x + qb.c * cp.localB.x - qb.s * cp.localB.y,
                   y: b.position.y + qb.s * cp.localB.x + qb.c * cp.localB.y };
      const n = { x: qa.c * cp.localNormal.x - qa.s * cp.localNormal.y,
                  y: qa.s * cp.localNormal.x + qa.c * cp.localNormal.y };
      const s = cp.baseSeparation + (pB.x - pA.x) * n.x + (pB.y - pA.y) * n.y;
      const m = { x: 0.5 * (pA.x + pB.x), y: 0.5 * (pA.y + pB.y) };
      const rA = { x: m.x - a.position.x, y: m.y - a.position.y };
      const rB = { x: m.x - b.position.x, y: m.y - b.position.y };
      if (s < minSep) minSep = s;
      const C = Math.max(-c.maxLinearCorrection, Math.min(0, c.baumgarte * (s + c.linearSlop)));
      const rnA = cross(rA, n), rnB = cross(rB, n);
      const K = a.invMass + b.invMass + a.invInertia * rnA * rnA + b.invInertia * rnB * rnB;
      const lambda = K > 0 ? -C / K : 0;
      const px = lambda * n.x, py = lambda * n.y;
      a.position.x -= a.invMass * px; a.position.y -= a.invMass * py;
      a.angle -= a.invInertia * (rA.x * py - rA.y * px);
      b.position.x += b.invMass * px; b.position.y += b.invMass * py;
      b.angle += b.invInertia * (rB.x * py - rB.y * px);
      syncTransform(a); syncTransform(b);
    }
  }
  return minSep;
}
