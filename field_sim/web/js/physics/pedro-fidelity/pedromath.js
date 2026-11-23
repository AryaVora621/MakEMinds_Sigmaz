// pedromath.js - Pedro Pathing 2.1.0 math, vector helpers, controllers and clock, ported line for line to JS.
// Pure ES module: no DOM, no imports. Units: radians, seconds/nanoseconds as noted, inches where a length appears.
//
// SOURCE OF RECORD (sourced, not assumed): the cached Pedro 2.1.0 sources jar
//   com.pedropathing:core:2.1.0 (%USERPROFILE%/.gradle/caches/modules-2/files-2.1/com.pedropathing/core/2.1.0/.../core-2.1.0-sources.jar)
// "Pedro:File.java:line" in the JSDoc below refers to files in that jar (com/pedropathing/...). Line numbers are of the
// comment-stripped file layout as shipped; the method name is the stable anchor.
//
// DEVIATIONS FROM PEDRO (all deliberate, all listed here):
//  D1. PIDF / FilteredPIDF: dt comes from an injected SimClock (not System.nanoTime). If dt <= 0 the derivative is set to 0
//      and the integral is skipped (Java would produce Inf/NaN).
//  D2. Vectors are plain {x,y} objects, not Vector(magnitude, theta). Pedro stores polar + cartesian; the cartesian part is
//      authoritative for every use in the follower, and theta is recomputed on demand by vTheta (= normalizeAngle(atan2(y,x)),
//      which is 0 for the zero vector, matching Pose.cartesianToPolar).
//  D3. Controllers hold the coefficient object by reference (as Java does), so mutating it live retunes the controller.
//  Everything else (including the FilteredPIDF raw-derivative quirk) is reproduced as is.

export const TAU = 2 * Math.PI; // rad, full turn

// ---------------------------------------------------------------------------------------------------------------------
// MathFunctions (Pedro:math/MathFunctions.java)
// ---------------------------------------------------------------------------------------------------------------------

/** Pedro:math/MathFunctions.java clamp(). NaN passes through unchanged, as in Java. */
export function clamp(x, lo, hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

/** Pedro:math/MathFunctions.java normalizeAngle(): result in [0, 2PI). Java % keeps the dividend's sign, as JS % does. */
export function normalizeAngle(a) {
  const angle = a % TAU;
  if (angle < 0) return angle + TAU;
  return angle;
}

/** Pedro:math/MathFunctions.java normalizeAngleSigned(): result in [-PI, PI). */
export function normalizeAngleSigned(a) {
  const angle = normalizeAngle(a);
  if (angle >= Math.PI) return angle - TAU;
  return angle;
}

/** Pedro:math/MathFunctions.java getSmallestAngleDifference(): unsigned, in [0, PI]. */
export function smallestAngleDiff(a, b) {
  return Math.min(normalizeAngle(a - b), normalizeAngle(b - a));
}

/** Pedro:math/MathFunctions.java getTurnDirection(): +1 CCW if (end-start) mod 2PI is in [0, PI], else -1 (CW). */
export function turnDirection(start, end) {
  const d = normalizeAngle(end - start);
  if (d >= 0 && d <= Math.PI) return 1;
  return -1;
}

// ---------------------------------------------------------------------------------------------------------------------
// Vector helpers on plain {x, y} (Pedro:math/Vector.java, Pedro:geometry/Pose.java polarToCartesian/cartesianToPolar)
// ---------------------------------------------------------------------------------------------------------------------

/** Pedro:math/Vector.java setComponents(magnitude, theta). A negative magnitude flips the direction (cos/sin of the raw args). */
export function vPolar(mag, theta) {
  return { x: mag * Math.cos(theta), y: mag * Math.sin(theta) };
}
/** Pedro:math/Vector.java getMagnitude() (via Pose.cartesianToPolar). */
export function vMag(v) { return Math.sqrt(v.x * v.x + v.y * v.y); }
/** Pedro:geometry/Pose.java cartesianToPolar(): theta = normalizeAngle(atan2(y, x)); the zero vector gives 0. */
export function vTheta(v) { return normalizeAngle(Math.atan2(v.y, v.x)); }
/** Pedro:math/Vector.java plus(). */
export function vAdd(a, b) { return { x: a.x + b.x, y: a.y + b.y }; }
/** Pedro:math/Vector.java minus(). */
export function vSub(a, b) { return { x: a.x - b.x, y: a.y - b.y }; }
/** Pedro:math/Vector.java times(scalar) = Vector(mag*scalar, theta); a negative scalar flips direction. */
export function vScale(v, k) { return { x: v.x * k, y: v.y * k }; }
/** Pedro:math/Vector.java dot(). */
export function vDot(a, b) { return a.x * b.x + a.y * b.y; }
/** Pedro:math/Vector.java cross() = ax*by - ay*bx (z component; positive when b is CCW of a). */
export function vCross(a, b) { return a.x * b.y - a.y * b.x; }
/** Pedro:math/Vector.java setMagnitude(m) = setComponents(m, theta). On a zero vector theta is 0, so the result points +x
 *  (Pedro quirk that matters for the translational F feedforward). */
export function vSetMag(v, m) { return vPolar(m, vTheta(v)); }
/** Pedro:math/Vector.java normalize(): zero vector stays zero, else unit vector at theta. */
export function vNormalize(v) {
  if (vMag(v) === 0) return { x: 0, y: 0 };
  return vPolar(1, vTheta(v));
}
/** Pedro:math/Vector.java rotateVector(theta2) = setTheta(theta + theta2). */
export function vRotate(v, theta) { return vPolar(vMag(v), vTheta(v) + theta); }
/** Pedro:math/Vector.java projectOnto(other): zero vector if |other| == 0, else other * (this.other / other.other). */
export function vProjectOnto(a, onto) {
  if (vMag(onto) === 0) return { x: 0, y: 0 };
  const s = vDot(a, onto) / vDot(onto, onto);
  return { x: onto.x * s, y: onto.y * s };
}

/**
 * Pedro:math/MathFunctions.java findNormalizingScaling(staticVector, variableVector, maxPowerScaling).
 * Root of |static + k*variable| = maxPower:  a = |v|^2, b = s.v, c = |s|^2 - m^2,  k = (-b + sqrt(b^2 - a c)) / a.
 */
export function findNormalizingScaling(staticV, variableV, maxPower) {
  const a = variableV.x * variableV.x + variableV.y * variableV.y;
  const b = staticV.x * variableV.x + staticV.y * variableV.y;
  const c = staticV.x * staticV.x + staticV.y * staticV.y - maxPower * maxPower;
  return (-b + Math.sqrt(b * b - a * c)) / a;
}

// ---------------------------------------------------------------------------------------------------------------------
// Kinematics (Pedro:math/Kinematics.java). Units: velocity in/s, decel in/s^2 (sign as Pedro stores it), distance in.
// ---------------------------------------------------------------------------------------------------------------------
export const Kinematics = {
  /** Pedro:math/Kinematics.java getVelocityToStopWithDeceleration: sign(d)*sqrt(|-2*a*d|). */
  velToStopWithDecel(d, decel) { return Math.sign(d) * Math.sqrt(Math.abs(-2 * decel * d)); },
  /** Pedro:math/Kinematics.java getFinalVelocityAtDistance: sign(d)*sqrt(|v^2 + 2*a*|d||). */
  finalVelAtDistance(v, decel, d) { return Math.sign(d) * Math.sqrt(Math.abs(v * v + 2 * decel * Math.abs(d))); },
  /** Pedro:math/Kinematics.java predictNextLoopVelocity: linear extrapolation 2*cur - prev. */
  predictNextLoopVelocity(cur, prev) { return 2 * cur - prev; },
  /** Pedro:math/Kinematics.java getDistanceToVelocity: |(vt^2 - v^2) / (2a)|. */
  distanceToVelocity(v, decel, vt) { return Math.abs((vt * vt - v * v) / (2 * decel)); },
  /** Pedro:math/Kinematics.java getStoppingDistance: distanceToVelocity(v, a, 0). */
  stoppingDistance(v, decel) { return Kinematics.distanceToVelocity(v, decel, 0); },
};

// ---------------------------------------------------------------------------------------------------------------------
// SimClock: deterministic replacement for System.nanoTime() / Timer. Time is only advanced by the sim.
// ---------------------------------------------------------------------------------------------------------------------
export class SimClock {
  /** @param {number} t0Ns start time, ns */
  constructor(t0Ns = 0) { this._ns = t0Ns; }
  /** Current time, ns (replaces System.nanoTime). */
  nowNs() { return this._ns; }
  /** Current time, ms (replaces Pedro Timer.getElapsedTime based ms timers). */
  nowMs() { return this._ns / 1e6; }
  /** Advance by dtS seconds. Rounded to whole ns so that repeated advances are exactly reproducible. */
  advance(dtS) { this._ns += Math.round(dtS * 1e9); }
}

// ---------------------------------------------------------------------------------------------------------------------
// Controllers
// ---------------------------------------------------------------------------------------------------------------------

/**
 * Pedro:control/PIDFController.java. run() = P*e + D*de/dt + I*integral + F*ff.
 * updateError(): previousError = error; error = e; dt = (now - prevT)/1e9; integral += e*dt; deriv = (e - prev)/dt.
 * Deviation D1: dt <= 0 -> deriv = 0, integral skipped.
 */
export class PIDF {
  /** @param {{P:number,I:number,D:number,F:number}} coeffs  @param {SimClock} clock */
  constructor(coeffs, clock) {
    this.c = coeffs;
    this.clock = clock;
    this.reset();
  }
  /** Pedro:control/PIDFController.java setCoefficients (held by reference, D3). */
  setCoefficients(c) { this.c = c; }
  /** Pedro:control/PIDFController.java updateError(double). */
  updateError(e) {
    this._prevError = this._error;
    this._error = e;
    const now = this.clock.nowNs();
    const dt = (now - this._prevT) / 1e9;
    this._prevT = now;
    if (dt > 0) {
      this._integral += e * dt;
      this._deriv = (e - this._prevError) / dt;
    } else {
      this._deriv = 0;
    }
  }
  /** Pedro:control/PIDFController.java updateFeedForwardInput. */
  updateFeedForwardInput(x) { this._ff = x; }
  /** Pedro:control/PIDFController.java run(). */
  run() {
    return this._error * this.c.P + this._deriv * this.c.D + this._integral * this.c.I + this._ff * this.c.F;
  }
  /** Pedro:control/PIDFController.java reset(): zero all state, prevT = now. */
  reset() {
    this._prevError = 0; this._error = 0; this._integral = 0; this._deriv = 0; this._ff = 0;
    this._prevT = this.clock.nowNs();
  }
  get error() { return this._error; }
  get errorDerivative() { return this._deriv; }
  get errorIntegral() { return this._integral; }
}

/**
 * Pedro:control/FilteredPIDFController.java. Identical to PIDF except the derivative filter, which has a Pedro quirk
 * that is reproduced on purpose: previousDerivative = errorDerivative (the RAW previous derivative, not the previous
 * filtered value), then filtered = T*previousDerivative + (1 - T)*raw. run() uses the filtered derivative.
 */
export class FilteredPIDF {
  /** @param {{P:number,I:number,D:number,T:number,F:number}} coeffs  @param {SimClock} clock */
  constructor(coeffs, clock) {
    this.c = coeffs;
    this.clock = clock;
    this.reset();
  }
  /** Pedro:control/FilteredPIDFController.java setCoefficients (by reference, D3). */
  setCoefficients(c) { this.c = c; }
  /** Pedro:control/FilteredPIDFController.java updateError(double). */
  updateError(e) {
    this._prevError = this._error;
    this._error = e;
    const now = this.clock.nowNs();
    const dt = (now - this._prevT) / 1e9;
    this._prevT = now;
    if (dt > 0) this._integral += e * dt;
    this._prevDeriv = this._deriv; // quirk: RAW previous derivative
    this._deriv = dt > 0 ? (e - this._prevError) / dt : 0;
    this._filtered = this.c.T * this._prevDeriv + (1 - this.c.T) * this._deriv;
  }
  /** Pedro:control/FilteredPIDFController.java updateFeedForwardInput. */
  updateFeedForwardInput(x) { this._ff = x; }
  /** Pedro:control/FilteredPIDFController.java run(). */
  run() {
    return this._error * this.c.P + this._filtered * this.c.D + this._integral * this.c.I + this.c.F * this._ff;
  }
  /** Pedro:control/FilteredPIDFController.java reset(). */
  reset() {
    this._prevError = 0; this._error = 0; this._integral = 0; this._deriv = 0;
    this._prevDeriv = 0; this._filtered = 0; this._ff = 0;
    this._prevT = this.clock.nowNs();
  }
  get error() { return this._error; }
  get errorDerivative() { return this._deriv; }
  get filteredDerivative() { return this._filtered; }
}

/**
 * Pedro:control/KalmanFilter.java (1-D). update(data, projection):
 *   state = prevState + data; variance = prevVariance + Q; K = variance/(variance + R);
 *   state += K*(projection - state); variance *= (1 - K); previous := current.
 * reset() = reset(0, 1, 1).
 */
export class KalmanFilter {
  /** @param {{modelCovariance:number, dataCovariance:number}} params (Q, R) */
  constructor(params) {
    this.params = params;
    this.reset();
  }
  /** Pedro:control/KalmanFilter.java reset(startState, startVariance, startGain). */
  reset(state = 0, variance = 1, gain = 1) {
    this._state = state; this._prevState = state;
    this._variance = variance; this._prevVariance = variance;
    this._gain = gain;
  }
  /** Pedro:control/KalmanFilter.java update(updateData, updateProjection). */
  update(data, projection) {
    this._state = this._prevState + data;
    this._variance = this._prevVariance + this.params.modelCovariance;
    this._gain = this._variance / (this._variance + this.params.dataCovariance);
    this._state += this._gain * (projection - this._state);
    this._variance *= (1.0 - this._gain);
    this._prevState = this._state;
    this._prevVariance = this._variance;
  }
  get state() { return this._state; }
  get variance() { return this._variance; }
  get gain() { return this._gain; }
}

/**
 * Pedro:control/PredictiveBrakingController.java computeOutput(error, velocity)
 *   = P * (error - (sign(v)*v^2*kQuadraticFriction + v*kLinearBraking)).
 */
export class PredictiveBraking {
  /** @param {{P:number,kLinearBraking:number,kQuadraticFriction:number,maximumBrakingPower:number}} coeffs */
  constructor(coeffs) { this.c = coeffs; }
  setCoefficients(c) { this.c = c; }
  /** Pedro:control/PredictiveBrakingController.java computeBrakingDisplacement. */
  brakingDisplacement(velocity, dir) {
    return dir * velocity * velocity * this.c.kQuadraticFriction + velocity * this.c.kLinearBraking;
  }
  /** Pedro:control/PredictiveBrakingController.java computeOutput. */
  computeOutput(error, velocity) {
    return this.c.P * (error - this.brakingDisplacement(velocity, Math.sign(velocity)));
  }
}
