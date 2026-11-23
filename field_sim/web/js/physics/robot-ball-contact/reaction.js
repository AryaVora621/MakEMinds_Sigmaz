// reaction.js - robot reaction accumulator for robot-ball contact.
// Units: length in, time s, mass kg, impulse kg*in/s, sim force kg*in/s^2 (1 N = 39.3700787 sim force).
// All clamps come from params.js (RBC_PARAMS.robot.*, tagged ASSUMED there).
import { RBC_PARAMS } from './params.js';

/**
 * Sums impulses applied TO THE ROBOT during one substep, then converts them to a velocity
 * change (dv in in/s, dom in rad/s) and a plow force. Caller passes minus the ball impulse.
 */
export class RobotReactionAccumulator {
  constructor(params = RBC_PARAMS) {
    this.params = params;
    this.reset();
  }

  /** Clear all sums and block constraints. */
  reset() {
    this.sx = 0;          // sum impulse x on robot [kg*in/s]
    this.sy = 0;          // sum impulse y on robot [kg*in/s]
    this.tau = 0;         // sum angular impulse about robot centre [kg*in^2/s]
    this.plowImpulse = 0; // sum of robot-contact normal impulses |jn| [kg*in/s]
    this.blocks = [];
  }

  /**
   * @param {number} jx impulse on robot x [kg*in/s]
   * @param {number} jy impulse on robot y [kg*in/s]
   * @param {number} rx world lever from robot centre [in]
   * @param {number} ry world lever from robot centre [in]
   * @param {boolean} isNormalPush count jn toward plowForce
   * @param {number} jn normal impulse magnitude [kg*in/s]
   */
  addImpulse(jx, jy, rx, ry, isNormalPush = false, jn = 0) {
    this.sx += jx;
    this.sy += jy;
    this.tau += rx * jy - ry * jx;
    if (isNormalPush) this.plowImpulse += Math.abs(jn);
  }

  /** @param {{nx:number, ny:number, depth?:number, force?:number, maxClosingSpeed:number}} block */
  addBlock(block) {
    if (block) this.blocks.push(block);
  }

  /**
   * Compute (without mutating the robot) the reaction for substep h [s].
   * @param {{massKg:number, inertia:number}} robot
   * @returns {{dvx:number,dvy:number,dom:number,fx:number,fy:number,tau:number,plowForce:number,plowForceN:number,blocks:object[]}}
   */
  result(robot, h) {
    const p = this.params;
    const legacy = p.robot.legacyInfiniteMass;
    const m = robot && robot.massKg, I = robot && robot.inertia;
    const mOk = !legacy && Number.isFinite(m) && m > 0;
    const iOk = !legacy && Number.isFinite(I) && I > 0;
    const clamp = (v, lim) => Math.max(-lim, Math.min(lim, v));
    const dvx = mOk ? clamp(this.sx / m, p.robot.maxReactionDvPerStep) : 0;
    const dvy = mOk ? clamp(this.sy / m, p.robot.maxReactionDvPerStep) : 0;
    const dom = iOk ? clamp(this.tau / I, p.robot.maxReactionDomPerStep) : 0;
    const hh = h > 0 ? h : 1;
    const plowForce = this.plowImpulse / hh; // sim force
    return {
      dvx, dvy, dom,
      fx: this.sx / hh, fy: this.sy / hh, tau: this.tau / hh,
      plowForce,
      plowForceN: plowForce / p.units.N_TO_SIM,
      blocks: this.blocks.slice(),
    };
  }

  /**
   * Apply reaction and block clipping to robot.vx, vy, om (only if reactionEnabled and
   * not legacyInfiniteMass), then reset. Returns the result() object.
   */
  applyTo(robot, h) {
    const p = this.params;
    const res = this.result(robot, h);
    if (p.robot.reactionEnabled && !p.robot.legacyInfiniteMass) {
      robot.vx += res.dvx;
      robot.vy += res.dvy;
      robot.om += res.dom;
      for (const b of this.blocks) {
        const vc = robot.vx * b.nx + robot.vy * b.ny; // closing speed toward ball [in/s]
        const lim = Math.max(0, b.maxClosingSpeed || 0);
        if (vc > lim) {
          robot.vx -= (vc - lim) * b.nx;
          robot.vy -= (vc - lim) * b.ny;
        }
      }
    }
    this.reset();
    return res;
  }
}
