// hold_queue.js - FIFO of balls held by the robot, with a hard limit (FTC G407 = 4, params.hold.limit).
// Pure ES module, no DOM. No physical units here (count only). Mirrors maple-sim IntakeSimulation: refuse when full.
import { RBC_PARAMS } from './params.js';

export class HoldQueue {
  /** @param {number} limit max balls held (G407 = 4; SOURCED via params.hold.limit) */
  constructor(limit = RBC_PARAMS.hold.limit) {
    this.limit = limit;
    this._q = [];
  }

  get count() { return this._q.length; }
  get full() { return this._q.length >= this.limit; }
  canAccept() { return this._q.length < this.limit; }

  /** Append a ball record. Returns the stored entry {id,type,kind}, or null if the queue is full. */
  push(ball) {
    if (this.full) return null;
    const entry = { id: ball.id, type: ball.type, kind: ball.kind ?? ball.type };
    this._q.push(entry);
    return entry;
  }

  /** Remove and return the oldest entry, or null if empty. */
  shift() { return this._q.length ? this._q.shift() : null; }

  /** Oldest entry without removing it, or null. */
  peek() { return this._q.length ? this._q[0] : null; }

  clear() { this._q.length = 0; }

  /** Copy of the entries, oldest first. */
  snapshot() { return this._q.map((e) => ({ id: e.id, type: e.type, kind: e.kind })); }
}
