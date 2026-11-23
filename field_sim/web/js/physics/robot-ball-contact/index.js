// index.js - robot-ball-contact facade. One step() per balls.js substep h, replacing _intakeStep + the
// _robotCollide loop. Units: length in, time s, velocity in/s, mass kg, impulse kg*in/s, sim force kg*in/s^2.
// All tunables live in params.js (each tagged SOURCED/COMPUTED/ASSUMED in RBC_SOURCES).
import { makeParams } from './params.js';
import { robotInertia } from './mixing.js';
import { buildRobotShape, robotContacts } from './robot_geometry.js';
import { solveRobotBallContact } from './solver.js';
import { RobotReactionAccumulator } from './reaction.js';
import { detectPinch, resolvePinch } from './pinch.js';
import { IntakeController } from './roller.js';
import { HoldQueue } from './hold_queue.js';

export * from './params.js';
export * from './mixing.js';
export { buildRobotShape, robotContacts, pocketTest } from './robot_geometry.js';
export { solveRobotBallContact } from './solver.js';
export { RobotReactionAccumulator } from './reaction.js';
export { detectPinch, resolvePinch, underFrame } from './pinch.js';
export { IntakeController, rollerContact } from './roller.js';
export { HoldQueue } from './hold_queue.js';

/**
 * @param {{params?:object, rng?:()=>number}} opts params = overrides for makeParams; rng returns [0,1)
 */
export function createRobotBallContact(opts = {}) {
  const params = makeParams(opts.params || {});
  const hold = new HoldQueue(params.hold.limit);
  const intake = new IntakeController(params, opts.rng || null);
  const reaction = new RobotReactionAccumulator(params);
  let massKg = params.robot.massKg;
  let shape = null, shapeKey = '';

  const api = {
    params, hold, intake, reaction,
    setRpm(side, rpm) { intake.setRpm(side, rpm); },
    /** Robot mass, kg. Inertia is recomputed from it on the next step. */
    setRobotMass(m) { massKg = m; },
    reset() { hold.clear(); intake.reset(); reaction.reset(); },

    /**
     * @param {object[]} balls balls with {state,x,y,z,vx,vy,vz,r,kind,...}
     * @param {{x,y,h,vx,vy,om}} robot mutated only by the reaction (vx,vy,om)
     * @param {{halfX,halfY,height}} dims in
     * @param {number} h substep, s
     * @param {{wake?:(b)=>void, staticContacts?:(b)=>Array<{nx,ny,nz,pen}>}} ctx
     */
    step(balls, robot, dims, h, ctx = {}) {
      reaction.reset();
      const key = dims.halfX + '|' + dims.halfY + '|' + dims.height;
      if (!shape || key !== shapeKey) { shape = buildRobotShape(dims, params); shapeKey = key; }
      const rs = {
        x: robot.x, y: robot.y, h: robot.h, vx: robot.vx || 0, vy: robot.vy || 0, om: robot.om || 0,
        massKg, inertia: robotInertia(massKg, dims.halfX, dims.halfY), halfX: dims.halfX, halfY: dims.halfY,
      };

      // 1. intake pass
      const cand = balls.filter((b) => b.state === 'free' && !b.shot && !b._ghost);
      const ir = intake.step(cand, rs, dims, shape, hold, reaction, h);
      const taken = new Set(ir.captured.map((c) => c[0]));

      // 2./3. body pass + pinch pass
      for (const b of cand) {
        if (taken.has(b) || b.state !== 'free') continue;
        const cs = robotContacts(b, rs, shape, params.contact.maxContactsPerBall);
        if (!cs.length) continue;
        if (ctx.wake) ctx.wake(b);
        const deepest = cs[0];
        for (const c of cs) {
          const J = solveRobotBallContact(b, rs, c, params);
          if (J.jn > 0 || J.jt !== 0) reaction.addImpulse(-J.jx, -J.jy, J.rx, J.ry, J.jn > 0, J.jn);
        }
        if (ctx.staticContacts) {
          const sc = ctx.staticContacts(b);
          if (sc && sc.length) {
            const p = detectPinch(b, rs, deepest, sc, h, params);
            if (p) { const blk = resolvePinch(b, rs, p, params); if (blk) reaction.addBlock(blk); }
          }
        }
      }

      // 4. reaction
      const res = reaction.applyTo(rs, h);
      if (params.robot.reactionEnabled && !params.robot.legacyInfiniteMass) {
        robot.vx = rs.vx; robot.vy = rs.vy; robot.om = rs.om;
      }
      return {
        captured: ir.captured, jammed: ir.jammed, plowForceN: res.plowForceN,
        stalled: intake.stalled, blocks: res.blocks,
        reaction: { dvx: res.dvx, dvy: res.dvy, dom: res.dom },
      };
    },
  };
  return api;
}
