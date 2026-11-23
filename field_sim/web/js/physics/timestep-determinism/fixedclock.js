// fixedclock.js - Fiedler accumulator with integer tick counter (timestep-determinism, spec sections 3, 4).
// Units: seconds, ticks. simTime = tick*h is computed from the integer tick, never accumulated.
// Sources: Fiedler "Fix Your Timestep!" https://gafferongames.com/post/fix_your_timestep/ ;
// Bullet btDiscreteDynamicsWorld clamp-and-drop. Constants come from params.js (TS).
import { TS } from './params.js';

export class FixedClock {
  constructor({
    hz = TS.PHYSICS_HZ,
    controlDivider = TS.CONTROL_DIVIDER,
    maxFrameDt = TS.MAX_FRAME_DT,
    maxStepsPerFrame = TS.MAX_STEPS_PER_FRAME,
    backlog = TS.BACKLOG_POLICY,
  } = {}) {
    this.hz = hz;
    this.h = 1 / hz; // [s]
    this.controlDivider = controlDivider;
    this.maxFrameDt = maxFrameDt; // [s]
    this.maxStepsPerFrame = maxStepsPerFrame; // [ticks]
    this.backlog = backlog; // 'drop' | 'carry'
    this.tick = 0;
    this.acc = 0; // [s] unconsumed time
  }

  get simTime() {
    return this.tick * this.h;
  }

  get alpha() {
    const a = this.acc / this.h;
    return a < 0 ? 0 : a > 1 ? 1 : a;
  }

  // Adds a wall-clock frame time and reports how many ticks to run. Does NOT increment tick.
  advance(frameDt) {
    let f = Number.isFinite(frameDt) && frameDt > 0 ? frameDt : 0;
    if (f > this.maxFrameDt) f = this.maxFrameDt;
    this.acc += f;
    const h = this.h;
    let n = Math.floor(this.acc / h + 1e-9);
    let dropped = 0;
    if (n > this.maxStepsPerFrame) {
      const over = n - this.maxStepsPerFrame;
      dropped = over * h;
      if (this.backlog === 'drop') {
        // consume the whole backlog, keep only the fractional remainder
        this.acc -= n * h;
      } else {
        this.acc -= this.maxStepsPerFrame * h;
        dropped = 0;
      }
      n = this.maxStepsPerFrame;
    } else {
      this.acc -= n * h;
    }
    if (this.acc < 0) this.acc = 0;
    return { steps: n, alpha: this.alpha, droppedSeconds: dropped };
  }

  onStep() {
    this.tick++;
  }

  isControlTick(tick = this.tick) {
    return tick % this.controlDivider === 0;
  }

  reset(tick = 0) {
    this.tick = tick;
    this.acc = 0;
  }
}
