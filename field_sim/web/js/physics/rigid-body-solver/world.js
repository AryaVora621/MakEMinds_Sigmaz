// World: fixed-substep sequential-impulse pipeline (spec section 7).
// Sources: Box2D v3 soft step ordering (prepare, warm start, solve with bias,
// integrate, relax, restitution), planck.js/Box2D v2.4 NGS position stage.
// All units SI (m, kg, s, rad, N*s).
import { makeParams } from './params.js';
import { clamp } from './vec2.js';
import { getTransform, syncTransform, integrateVelocities, integratePositions, clearForces } from './body.js';
import { computeAABB, aabbOverlap } from './shapes.js';
import { collide } from './collide.js';
import { pairKey, createArbiter, updateArbiter } from './arbiter.js';
import { prepareContacts, warmStartContacts, solveContacts, applyRestitution, solvePositions } from './contact_solver.js';
import { prepareDrive, warmStartDrive, solveDrive, finishDrive } from './drive.js';

export class World {
  constructor(paramOverrides = {}) {
    this.params = makeParams(paramOverrides);
    this.bodies = [];
    this.arbiters = new Map();
    this.time = 0;          // s
    this.lastDt = 0;        // s, last step() dt (after clamp)
    this.stats = { substeps: 0, h: 0, arbiterCount: 0, pointCount: 0 };
  }

  addBody(body) {
    this.bodies.push(body);
    return body;
  }

  removeBody(bodyOrId) {
    const id = typeof bodyOrId === 'object' ? bodyOrId.id : bodyOrId;
    const i = this.bodies.findIndex((b) => b.id === id);
    if (i >= 0) this.bodies.splice(i, 1);
    for (const [key, arb] of this.arbiters) {
      if (arb.bodyA.id === id || arb.bodyB.id === id) this.arbiters.delete(key);
    }
  }

  getBody(id) {
    return this.bodies.find((b) => b.id === id);
  }

  step(dt) {
    const P = this.params;
    dt = Math.min(dt, P.step.maxFrameDt);
    if (!(dt > 0)) return;
    const n = clamp(Math.ceil(dt / P.step.maxSubstepDt), 1, P.step.maxSubsteps);
    const h = dt / n;
    const bodies = this.bodies;
    this.lastDt = dt;
    for (const b of bodies) {
      b.contactImpulse = { x: 0, y: 0 };
      b.contactHits = [];
    }
    const margin = P.contact.speculativeDistance;
    const copts = {
      speculativeDistance: margin,
      faceRelTol: P.collide.faceRelTol,
      faceAbsTolFrac: P.collide.faceAbsTolFrac,
    };

    for (let sub = 0; sub < n; sub++) {
      // 1 transforms + AABBs
      for (const b of bodies) {
        syncTransform(b);
        b.aabb = computeAABB(b.shape, getTransform(b), margin);
      }
      // 2-3 broadphase + narrowphase
      const seen = new Set();
      for (let i = 0; i < bodies.length; i++) {
        const bi = bodies[i];
        for (let j = i + 1; j < bodies.length; j++) {
          const bj = bodies[j];
          if (bi.type !== 'dynamic' && bj.type !== 'dynamic') continue;
          if (!aabbOverlap(bi.aabb, bj.aabb)) continue;
          if (!((bi.categoryBits & bj.maskBits) && (bj.categoryBits & bi.maskBits))) continue;
          const a = bi.id < bj.id ? bi : bj;
          const b = a === bi ? bj : bi;
          const m = collide(a.shape, getTransform(a), b.shape, getTransform(b), copts);
          if (m.pointCount === 0) continue;
          const key = pairKey(a.id, b.id);
          let arb = this.arbiters.get(key);
          if (!arb) {
            arb = createArbiter(a, b);
            this.arbiters.set(key, arb);
          }
          updateArbiter(arb, m, P);
          seen.add(key);
        }
      }
      for (const key of Array.from(this.arbiters.keys())) {
        if (!seen.has(key)) this.arbiters.delete(key);
      }
      const arbs = Array.from(this.arbiters.values());

      // 4 kinematic drive velocities, then integrate velocities
      for (const b of bodies) {
        if (b.type === 'kinematic' && b.drive && b.drive.enabled) {
          const t = b.drive.target;
          b.linearVelocity = {
            x: b.q.c * t.vx - b.q.s * t.vy,
            y: b.q.s * t.vx + b.q.c * t.vy,
          };
          b.angularVelocity = t.om;
        }
        integrateVelocities(b, h, P);
      }
      const driven = bodies.filter((b) => b.type === 'dynamic' && b.drive && b.drive.enabled);

      // 5 prepare
      prepareContacts(arbs, h, P);
      for (const b of driven) prepareDrive(b, h, P);
      // 6 warm start (arbiter points were zeroed by updateArbiter when warm starting is off)
      warmStartContacts(arbs);
      for (const b of driven) warmStartDrive(b, P);
      // 7 velocity iterations
      for (let k = 0; k < P.solver.velocityIterations; k++) {
        for (const b of driven) solveDrive(b, h);
        solveContacts(arbs, h, P, true);
      }
      // 8 integrate positions
      for (const b of bodies) {
        if (b.type !== 'static') integratePositions(b, h, P);
      }
      // 9 soft relax
      if (P.solver.positionMode === 'soft') {
        for (let k = 0; k < P.solver.relaxIterations; k++) {
          for (const b of driven) solveDrive(b, h);
          solveContacts(arbs, h, P, false);
        }
      }
      // 10 restitution
      applyRestitution(arbs, P);
      // 11 NGS
      if (P.solver.positionMode === 'ngs') {
        for (let k = 0; k < P.solver.positionIterations; k++) {
          if (solvePositions(arbs, P) >= -P.contact.ngsAcceptFactor * P.contact.linearSlop) break;
        }
      }
      // 12 finish drive + feedback (hits: last substep wins)
      for (const b of driven) finishDrive(b, h);
      for (const b of bodies) b.contactHits = [];
      let pts = 0;
      for (const arb of arbs) {
        const nx = arb.normal.x, ny = arb.normal.y;
        const tx = ny, ty = -nx;
        const A = arb.bodyA, B = arb.bodyB;
        for (let k = 0; k < arb.pointCount; k++) {
          const p = arb.points[k];
          pts++;
          const Px = p.normalImpulse * nx + p.tangentImpulse * tx;
          const Py = p.normalImpulse * ny + p.tangentImpulse * ty;
          B.contactImpulse.x += Px; B.contactImpulse.y += Py;
          A.contactImpulse.x -= Px; A.contactImpulse.y -= Py;
          if (p.normalImpulse > 0) {
            B.contactHits.push({ other: A, normal: { x: nx, y: ny }, point: p.point, normalImpulse: p.normalImpulse });
            A.contactHits.push({ other: B, normal: { x: -nx, y: -ny }, point: p.point, normalImpulse: p.normalImpulse });
          }
        }
      }
      this.stats.arbiterCount = arbs.length;
      this.stats.pointCount = pts;
    }
    for (const b of bodies) clearForces(b);
    this.time += dt;
    this.stats.substeps = n;
    this.stats.h = h;
  }

  contactsOf(bodyOrId) {
    const b = typeof bodyOrId === 'object' ? bodyOrId : this.getBody(bodyOrId);
    return b ? b.contactHits : [];
  }

  contactForceOf(bodyOrId) {
    const b = typeof bodyOrId === 'object' ? bodyOrId : this.getBody(bodyOrId);
    if (!b || !(this.lastDt > 0)) return { x: 0, y: 0, magnitude: 0 };
    const x = b.contactImpulse.x / this.lastDt;
    const y = b.contactImpulse.y / this.lastDt;
    return { x, y, magnitude: Math.hypot(x, y) };
  }
}
