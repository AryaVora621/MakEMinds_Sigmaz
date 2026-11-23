// rotary.js - single-DOF rotating mechanism driven by a MotorUnit (flywheel, DC turret).
// Units: rad, rad/s, N*m, kg*m^2, s. See spec section 5.
// Semi-implicit update with the motor torque linearised about omega (stable for back-EMF damping),
// then a Karnopp-style Coulomb/stiction impulse clamp, then hard stops.
import { SIM } from './params.js';

export class RotaryMechanism {
  constructor({ motor, J, viscous = 0, loadTorque = null, minTheta = -Infinity, maxTheta = Infinity, power = null, name = 'rotary' }) {
    this.motor = motor;
    this.J = J; // kg*m^2 mechanism inertia at the output (excludes reflected rotor)
    this.viscous = viscous; // N*m*s/rad
    this.loadTorque = loadTorque;
    this.minTheta = minTheta;
    this.maxTheta = maxTheta;
    this.power = power;
    this.name = name;
    this.theta = 0;
    this.omega = 0;
  }
  get Jtot() { return this.J + this.motor.gm.rotorJReflected; }

  substep(h, duty, busV, zeroPower) {
    const c = this.motor.compute(this.omega, busV, duty, zeroPower);
    const Jt = this.Jtot;
    const Tl = this.loadTorque ? this.loadTorque(this.theta, this.omega) : 0;
    let w1 = (Jt * this.omega + h * (c.torqueOut - c.dTdOmega * this.omega - Tl)) /
             (Jt - h * c.dTdOmega + h * this.viscous);
    const imp = c.frictionOut * h / Jt;
    if (Math.abs(w1) <= imp) w1 = 0;
    else w1 -= Math.sign(w1) * imp;
    this.theta += w1 * h;
    this.omega = w1;
    if (this.theta < this.minTheta) { this.theta = this.minTheta; this.omega = 0; }
    else if (this.theta > this.maxTheta) { this.theta = this.maxTheta; this.omega = 0; }
    this.motor.commit(c, h);
    if (this.power) this.power.record(this.name, c.statorA, c.supplyA, this.motor.gm.n, h);
    return c;
  }

  step(dtS, duty, busV, zeroPower) {
    const dt = Math.min(dtS, SIM.maxFrameDtS);
    if (!(dt > 0)) return;
    const n = Math.ceil(dt / SIM.substepS - 1e-9);
    const h = dt / n;
    for (let i = 0; i < n; i++) this.substep(h, duty, busV, zeroPower);
  }

  /** Remove kinetic energy (J), e.g. a shot: w' = sign(w)*sqrt(max(0, w^2 - 2E/Jtot)). */
  removeEnergy(joules) {
    const w = this.omega;
    this.omega = Math.sign(w) * Math.sqrt(Math.max(0, w * w - 2 * joules / this.Jtot));
  }
  applyAngularImpulse(nms) { this.omega += nms / this.Jtot; }
  reset(theta = 0, omega = 0) { this.theta = theta; this.omega = omega; this.motor.reset(); }
}
