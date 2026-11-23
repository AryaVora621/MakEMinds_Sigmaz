// pedropath.js - Pedro Pathing 2.1.0 geometry (Bezier curve/line/point), Path, PathChain, HeadingInterpolator and the
// bridge-request -> path-group builder, ported to a pure ES module. No DOM. Imports only ./pedromath.js.
// Units: inches, radians (headings in [0, 2PI) unless noted), t is the Bezier parameter in [0, 1].
//
// SOURCE OF RECORD (sourced, not assumed): cached Pedro 2.1.0 sources jar com.pedropathing:core:2.1.0
// ("Pedro:File.java" = com/pedropathing/<File>). groupsFromBridgeRequest mirrors tools_java/PedroFollowSim.java buildGroups.
//
// DEVIATIONS FROM PEDRO:
//  D1. A fresh Path has its closest-point state initialised from t = 0 (Pedro leaves closestPose/tangent null until the first
//      updateClosestPose and would NullPointerException). Values are what updateClosestPose would return for t = 0.
//  D2. BezierCurve evaluates the polynomial in Bernstein form instead of Pedro's characteristic-matrix product. Same
//      polynomial, equal to ~1e-12 in.
//  D3. makeCurve('curve', 2 points) returns a BezierLine (Pedro's BezierCurve constructor throws "Too few control points").
//  D4. BezierLine.closestT of a zero-length line returns 1 (Pedro would produce NaN from 0/0).
//  D5. Tangent vectors returned by Path/BezierCurve are RAW derivatives (not normalised), exactly as Pedro's PathPoint
//      carries them; callers normalise (see Path.endTangent for the one normalised accessor Pedro provides).
//  D6. PathChain keeps each Path's own constraints (Pedro's PathChain(paths) constructor re-applies
//      PathConstraints.defaultConstraints, but FollowerBuilder.pathConstraints() sets that default to the robot's
//      constraints - FollowerBuilder.java:88 - so the two are identical on the robot).

import {
  clamp, normalizeAngle, smallestAngleDiff, turnDirection,
  vPolar, vMag, vTheta, vDot, vCross, vNormalize, vRotate,
} from './pedromath.js';

/** Pedro:paths/PathConstraints.java defaultConstraints = new PathConstraints(0.995, 0.1, 0.1, 0.007, 100, 1, 10, 1).
 *  velocityConstraint in/s, translationalConstraint in, headingConstraint rad, timeoutConstraint ms. Sourced. */
export const DEFAULT_CONSTRAINTS = Object.freeze({
  tValueConstraint: 0.995, velocityConstraint: 0.1, translationalConstraint: 0.1, headingConstraint: 0.007,
  timeoutConstraint: 100, brakingStrength: 1, brakingStart: 1, BEZIER_CURVE_SEARCH_LIMIT: 10,
});

const ZERO = () => ({ x: 0, y: 0 });
const cp = (p) => ({ x: p.x, y: p.y });
const dist = (a, b) => Math.hypot(a.x - b.x, a.y - b.y);

function binom(n, k) {
  let r = 1;
  for (let i = 1; i <= k; i++) r = (r * (n - k + i)) / i;
  return r;
}

/** Pedro:math/AbstractBijectiveMap.java NumericBijectiveMap.interpolate: linear between neighbouring keys, clamped to the
 *  end values outside the key range (note the map holds keys i/1000, i = 1..1000, with NO entry at key 0). */
function interpolateSorted(keys, vals, x) {
  const n = keys.length;
  if (n === 0) return NaN;
  if (x <= keys[0]) return vals[0];
  if (x >= keys[n - 1]) return vals[n - 1];
  let lo = 0, hi = n - 1;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (keys[mid] <= x) lo = mid; else hi = mid;
  }
  if (keys[lo] === x) return vals[lo];
  return vals[lo] + (vals[hi] - vals[lo]) * ((x - keys[lo]) / (keys[hi] - keys[lo]));
}

// ---------------------------------------------------------------------------------------------------------------------
// Curves
// ---------------------------------------------------------------------------------------------------------------------

/**
 * Pedro:geometry/BezierCurve.java (>= 3 control points). Also the base class of BezierLine / BezierPoint, which override
 * the same methods as their Pedro counterparts do.
 */
export class BezierCurve {
  /** @param {{x:number,y:number}[]} points control points, in  @param {object} constraints PathConstraints-shaped */
  constructor(points, constraints = DEFAULT_CONSTRAINTS) {
    this.constraints = { ...DEFAULT_CONSTRAINTS, ...constraints };
    this.points = points.map(cp);
    this.type = 'curve';
    this._init();
  }

  /** Pedro:geometry/BezierCurve.java initialize(): length = approximateLength() (1000 chords), completion map, endTangent. */
  _init() {
    if (this.points.length < 3) throw new Error('Too few control points'); // Pedro:BezierCurve.java ctor
    const N = 1000; // Pedro:BezierCurve.java APPROXIMATION_STEPS
    this._ckeys = new Float64Array(N);
    this._cvals = new Float64Array(N);
    let prev = this.pose(0), len = 0;
    for (let i = 1; i <= N; i++) {
      const t = i / N;
      const cur = this.pose(t);
      len += dist(prev, cur);
      prev = cur;
      this._ckeys[i - 1] = t;
      this._cvals[i - 1] = len;
    }
    this._length = len;
    const n = this.points.length;
    this._endTangent = vNormalize({ x: this.points[n - 1].x - this.points[n - 2].x, y: this.points[n - 1].y - this.points[n - 2].y });
  }

  get length() { return this._length; }
  get firstPoint() { return this.points[0]; }
  get lastPoint() { return this.points[this.points.length - 1]; }
  get secondToLastPoint() { return this.points[this.points.length - 2]; }
  get controlPoints() { return this.points; }

  /** Pedro:geometry/BezierCurve.java getPose(t): t clamped to [0,1]. */
  pose(t) {
    t = clamp(t, 0, 1);
    const P = this.points, n = P.length - 1, u = 1 - t;
    let x = 0, y = 0;
    for (let i = 0; i <= n; i++) {
      const b = binom(n, i) * Math.pow(u, n - i) * Math.pow(t, i);
      x += b * P[i].x; y += b * P[i].y;
    }
    return { x, y };
  }

  /** Pedro:geometry/BezierCurve.java getDerivative(t) (raw dB/dt, in/unit-t). */
  deriv(t) {
    t = clamp(t, 0, 1);
    const P = this.points, n = P.length - 1, u = 1 - t;
    let x = 0, y = 0;
    for (let i = 0; i < n; i++) {
      const b = n * binom(n - 1, i) * Math.pow(u, n - 1 - i) * Math.pow(t, i);
      x += b * (P[i + 1].x - P[i].x); y += b * (P[i + 1].y - P[i].y);
    }
    return { x, y };
  }

  /** Pedro:geometry/BezierCurve.java getSecondDerivative(t). */
  deriv2(t) {
    t = clamp(t, 0, 1);
    const P = this.points, n = P.length - 1, u = 1 - t;
    let x = 0, y = 0;
    if (n >= 2) {
      for (let i = 0; i < n - 1; i++) {
        const b = n * (n - 1) * binom(n - 2, i) * Math.pow(u, n - 2 - i) * Math.pow(t, i);
        x += b * (P[i + 2].x - 2 * P[i + 1].x + P[i].x); y += b * (P[i + 2].y - 2 * P[i + 1].y + P[i].y);
      }
    }
    return { x, y };
  }

  /** Pedro:geometry/Curve.java getCurvature (also BezierCurve.java): cross(d, dd)/|d|^3, 0 if |d| = 0. Signed, + = left/CCW. */
  curvature(t) {
    t = clamp(t, 0, 1);
    const d = this.deriv(t), dd = this.deriv2(t);
    const m = vMag(d);
    if (m === 0) return 0;
    return vCross(d, dd) / Math.pow(m, 3);
  }

  /** Pedro:geometry/BezierCurve.java getNormalVector(t) = Vector(1, theta(d(t+0.0001)) - theta(d(t))). */
  normalVector(t) {
    const current = vTheta(this.deriv(t));
    const next = vTheta(this.deriv(t + 0.0001));
    return vPolar(1, next - current);
  }

  /** Pedro:geometry/Curve.java leftGradient(t): derivative rotated by -PI/2 (not normalised). */
  leftGradient(t) { return vRotate(this.deriv(t), -Math.PI / 2); }

  /** Pedro:geometry/BezierCurve.java getEndTangent(): normalised (last - secondToLast control point). */
  endTangent() { return { x: this._endTangent.x, y: this._endTangent.y }; }

  /**
   * Pedro:geometry/BezierCurve.java getClosestPoint(pose, searchLimit, initialTValueGuess).
   * Seed = best of t in {0, .1, ..., 1, guess} by squared distance (strict <, first wins), then up to searchLimit Newton
   * steps on f(t) = |B(t) - q|^2:  t -= f'/(f'' + 1e-9), clamped to [0,1]; stop when the point moved < 0.1 in.
   */
  closestT(pose, searchLimit = this.constraints.BEZIER_CURVE_SEARCH_LIMIT, guess = 0) {
    const seeds = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, guess];
    let closestDist = 1e9, best = 0;
    for (const t of seeds) {
      const p = this.pose(t);
      const d2 = (p.x - pose.x) * (p.x - pose.x) + (p.y - pose.y) * (p.y - pose.y);
      if (d2 < closestDist) { closestDist = d2; best = t; }
    }
    let t = best;
    for (let i = 0; i < searchLimit; i++) {
      const last = this.pose(t), d1 = this.deriv(t), d2 = this.deriv2(t);
      const diff = { x: last.x - pose.x, y: last.y - pose.y };
      const f1 = 2 * vDot(diff, d1);
      const f2 = 2 * (vDot(diff, d2) + d1.x * d1.x + d1.y * d1.y);
      t = clamp(t - f1 / (f2 + 1e-9), 0, 1);
      if (dist(this.pose(t), last) < 0.1) break;
    }
    return t;
  }

  /** Pedro:geometry/BezierCurve.java atParametricEnd(t): t >= tValueConstraint. */
  atParametricEnd(t) { return t >= this.constraints.tValueConstraint; }

  /** Pedro:geometry/BezierCurve.java getPathCompletion(t) = completionMap.interpolateKey(t)/length (0 if length 0).
   *  Quirk kept: the map has no key at t = 0, so t below 0.001 returns d(0.001)/length. */
  pathCompletion(t) {
    if (this._length === 0) return 0;
    return interpolateSorted(this._ckeys, this._cvals, t) / this._length;
  }

  /** Pedro:geometry/BezierCurve.java getT(pathCompletion) = completionMap.interpolateValue(completion). */
  tAtCompletion(c) {
    // interpolateValue(value): reverse map keyed by cumulative length. Cumulative lengths are non-decreasing; for a
    // strictly increasing run this is a plain inverse interpolation.
    return interpolateSorted(this._cvals, this._ckeys, c);
  }

  setConstraints(c) { this.constraints = { ...c }; }
}

/** Pedro:geometry/BezierLine.java. Straight line; derivative is constant, closest point is an orthogonal projection. */
export class BezierLine extends BezierCurve {
  constructor(p0, p1, constraints = DEFAULT_CONSTRAINTS) { super([p0, p1], constraints); this.type = 'line'; }
  /** Pedro:geometry/BezierLine.java initialize(): length = chord, endTangent = normalised derivative. */
  _init() {
    const [a, b] = this.points;
    this._length = dist(a, b); // Pedro:BezierLine.java approximateLength
    this._endTangent = vNormalize(this.deriv(1));
  }
  pose(t) {
    t = clamp(t, 0, 1);
    const [a, b] = this.points;
    return { x: (b.x - a.x) * t + a.x, y: (b.y - a.y) * t + a.y }; // Pedro:BezierLine.java getPose
  }
  deriv(_t) { const [a, b] = this.points; return { x: b.x - a.x, y: b.y - a.y }; } // Pedro:BezierLine.java getDerivative
  deriv2(_t) { return ZERO(); }
  curvature(_t) { return 0; }
  normalVector(_t) { return ZERO(); }
  /** Pedro:geometry/BezierLine.java getClosestPoint: clamp(BA.PA / |BA|^2, 0, 1). */
  closestT(pose) {
    const [a, b] = this.points;
    const BA = { x: b.x - a.x, y: b.y - a.y }, PA = { x: pose.x - a.x, y: pose.y - a.y };
    const m2 = Math.pow(vMag(BA), 2);
    if (m2 === 0) return 1; // D4
    return clamp(vDot(BA, PA) / m2, 0, 1);
  }
  /** Pedro:geometry/BezierLine.java getPathCompletion: clamp(t, 0, 1). */
  pathCompletion(t) { return clamp(t, 0, 1); }
  /** Pedro:geometry/BezierLine.java getT: clamp(c, 0, 1). */
  tAtCompletion(c) { return clamp(c, 0, 1); }
}

/** Pedro:geometry/BezierPoint.java. A single point: length 0, closest t = 1, derivative/end tangent = zero vector. */
export class BezierPoint extends BezierCurve {
  constructor(p, constraints = DEFAULT_CONSTRAINTS) { super([p], constraints); this.type = 'point'; }
  _init() { this._length = 0; this._endTangent = ZERO(); }
  pose(_t) { return cp(this.points[0]); }
  deriv(_t) { return ZERO(); }       // Pedro:BezierPoint.java getDerivative = endTangent (zero vector)
  deriv2(_t) { return ZERO(); }
  curvature(_t) { return 0; }
  normalVector(_t) { return ZERO(); }
  closestT(_pose) { return 1; }      // Pedro:BezierPoint.java getClosestPoint
  pathCompletion(_t) { return 1; }   // Pedro:BezierPoint.java getPathCompletion
  tAtCompletion(_c) { return 1; }    // Pedro:BezierPoint.java getT
}

/**
 * Factory. type 'line' -> BezierLine (first and last point only, as PedroFollowSim.buildGroups does); 'point' or a single
 * point -> BezierPoint; anything else -> BezierCurve (2 points degrade to a line, D3).
 */
export function makeCurve(type, points, constraints = DEFAULT_CONSTRAINTS) {
  if (!points || points.length === 0) throw new Error('makeCurve: no points');
  if (type === 'point' || points.length === 1) return new BezierPoint(points[0], constraints);
  if (type === 'line' || points.length === 2) return new BezierLine(points[0], points[points.length - 1], constraints);
  return new BezierCurve(points, constraints);
}

// ---------------------------------------------------------------------------------------------------------------------
// Heading interpolators (Pedro:paths/HeadingInterpolator.java). Each is fn(pathPoint) -> rad in [0, 2PI), where
// pathPoint = { tValue, pose:{x,y}, tangent:{x,y} } (Pedro:paths/PathPoint.java).
// ---------------------------------------------------------------------------------------------------------------------
export const HI = {
  /** HeadingInterpolator.tangent: theta of the (raw) tangent vector. */
  tangent: (pp) => vTheta(pp.tangent),

  /** HeadingInterpolator.constant(headingRad): normalizeAngle(heading). */
  constant(h) { return () => normalizeAngle(h); },

  /** HeadingInterpolator.linear(start, end, endT): works in PARAMETRIC t.
   *  tt = min(t / clamp(endT, 1e-4, 1), 1); out = norm(h0 + turnDir*smallestDiff * tt). */
  linear(h0, h1, endT = 1) {
    const s = normalizeAngle(h0), e = normalizeAngle(h1);
    return (pp) => {
      const cEnd = clamp(endT, 0.0001, 1);
      const tt = Math.min(pp.tValue / cEnd, 1.0);
      const delta = turnDirection(s, e) * smallestAngleDiff(e, s);
      return normalizeAngle(s + delta * tt);
    };
  },

  /** HeadingInterpolator.reversedLinear(start, end, endT): the long way round.
   *  delta = -turnDir * max(norm(e - s), norm(s - e)). */
  reversedLinear(h0, h1, endT = 1) {
    const s = normalizeAngle(h0), e = normalizeAngle(h1);
    return (pp) => {
      const cEnd = clamp(endT, 0.0001, 1);
      const tt = Math.min(pp.tValue / cEnd, 1.0);
      const delta = -turnDirection(s, e) * Math.max(normalizeAngle(e - s), normalizeAngle(s - e));
      return normalizeAngle(s + delta * tt);
    };
  },

  /** HeadingInterpolator.facingPoint(x, y): norm(atan2(y - pose.y, x - pose.x)). */
  facingPoint(x, y) { return (pp) => normalizeAngle(Math.atan2(y - pp.pose.y, x - pp.pose.x)); },

  /** HeadingInterpolator.piecewise(nodes): first node with t0 <= t <= t1 wins; its fn sees t rescaled to [0,1]
   *  (MathFunctions.scale). No node matches -> normalizeAngle(tangent). nodes = [{t0, t1, fn}]. */
  piecewise(nodes) {
    return (pp) => {
      for (const nd of nodes) {
        if (pp.tValue >= nd.t0 && pp.tValue <= nd.t1) {
          if (nd.t1 - nd.t0 === 0) throw new Error('Input range cannot be zero.'); // Pedro:MathFunctions.scale
          const scaled = (pp.tValue - nd.t0) * (1 - 0) / (nd.t1 - nd.t0) + 0;
          return nd.fn({ tValue: scaled, pose: pp.pose, tangent: pp.tangent });
        }
      }
      return normalizeAngle(HI.tangent(pp));
    };
  },

  /** HeadingInterpolator.reverse(): norm(inner + PI). */
  reverse(fn) { return (pp) => normalizeAngle(fn(pp) + Math.PI); },

  /** HeadingInterpolator.offset(rad): inner + norm(offset) (NOT re-normalised, as in Pedro). */
  offset(fn, rad) { return (pp) => fn(pp) + normalizeAngle(rad); },
};

// ---------------------------------------------------------------------------------------------------------------------
// Path (Pedro:paths/Path.java)
// ---------------------------------------------------------------------------------------------------------------------
export class Path {
  /** @param {BezierCurve} curve  @param {object} constraints copied, and pushed into the curve (Path.setConstraints) */
  constructor(curve, constraints = DEFAULT_CONSTRAINTS) {
    this._curve = curve;
    this.constraints = { ...DEFAULT_CONSTRAINTS, ...constraints };
    curve.setConstraints(this.constraints);
    this._interp = HI.tangent; // Pedro:Path.java default headingInterpolator = HeadingInterpolator.tangent
    this._t = 0;
    // D1: initial closest-point state = information at t = 0.
    this._pose = null; this._tangent = null; this._curv = 0; this._normal = ZERO();
    this._setClosest(0);
  }

  get curve() { return this._curve; }

  /** Pedro:Path.java setConstantHeadingInterpolation(rad). */
  setConstantHeading(h) { this._interp = HI.constant(h); }
  /** Pedro:Path.java setLinearHeadingInterpolation(start, end, endTime, reversed): linear, then reverse() if reversed. */
  setLinearHeading(h0, h1, endT = 1, reversed = false) {
    this._interp = HI.linear(h0, h1, endT);
    if (reversed) this.reverseHeading();
  }
  /** Pedro:Path.java setTangentHeadingInterpolation(). */
  setTangentHeading() { this._interp = HI.tangent; }
  /** Pedro:Path.java reverseHeadingInterpolation(): wraps the current interpolator in reverse(). */
  reverseHeading() { this._interp = HI.reverse(this._interp); }
  /** Pedro:Path.java setHeadingInterpolation(interpolator). */
  setHeadingInterpolator(fn) { this._interp = fn; }
  get headingInterpolator() { return this._interp; }

  /** Pedro:Path.java getPoseInformation(t) -> PathPoint(t, getPose(t), getTangentVector(t)); pose carries the heading goal. */
  poseInformation(t) {
    const p = this._curve.pose(t);
    return { tValue: t, pose: { x: p.x, y: p.y, h: this.headingGoalAt(t) }, tangent: this._curve.deriv(t) };
  }

  _setClosest(t) {
    const pp = this.poseInformation(t);
    this._t = pp.tValue;
    this._pose = pp.pose;
    this._tangent = pp.tangent;
    this._normal = this._curve.normalVector(t);
    this._curv = this._curve.curvature(t);
    return pp;
  }

  /** Pedro:Path.java updateClosestPose(currentPose, searchLimit): closest t (seeded by the previous t), then stores
   *  tValue, pose, tangent, normal and curvature. Returns the PathPoint {tValue, pose:{x,y,h}, tangent}. */
  updateClosestPose(pose, searchLimit = this.constraints.BEZIER_CURVE_SEARCH_LIMIT) {
    const t = this._curve.closestT(pose, searchLimit, this._t);
    return this._setClosest(t);
  }

  get closestT() { return this._t; }                       // Pedro:Path.java getClosestPointTValue
  get closestPose() { return this._pose; }                 // Pedro:Path.java getClosestPose (pose incl. heading goal)
  get tangent() { return this._tangent; }                  // Pedro:Path.java getClosestPointTangentVector (raw derivative)
  get curvature() { return this._curv; }                   // Pedro:Path.java getClosestPointCurvature
  get normal() { return this._normal; }                    // Pedro:Path.java getClosestPointNormalVector

  /** Pedro:Path.java getClosestPointHeadingGoal() = closestPose.getHeading(). */
  headingGoal() { return this._pose.h; }
  /** Pedro:Path.java getHeadingGoal(t): interpolator at PathPoint(t, curve.getPose(t), tangent(t)). */
  headingGoalAt(t) {
    return this._interp({ tValue: t, pose: this._curve.pose(t), tangent: this._curve.deriv(t) });
  }
  /** Pedro:Path.java getPathCompletion(t). */
  pathCompletion(t = this._t) { return this._curve.pathCompletion(t); }
  /** Pedro:Path.java getDistanceTraveled(t) = completion(t) * length. */
  distanceTraveled(t = this._t) { return this.pathCompletion(t) * this._curve.length; }
  /** Pedro:Path.java getDistanceRemaining(t) = length - distanceTraveled(t); no argument = at the closest point. */
  distanceRemaining(t = this._t) { return this._curve.length - this.distanceTraveled(t); }
  get length() { return this._curve.length; }              // Pedro:Path.java length()

  /** Pedro:Path.java isAtParametricEnd(): curve.atParametricEnd(closestT). */
  isAtParametricEnd() { return this._curve.atParametricEnd(this._t); }
  /** Pedro:Path.java isAtParametricStart(): closestT <= 1 - tValueConstraint. */
  isAtParametricStart() { return this._t <= 1 - this.constraints.tValueConstraint; }
  /** Pedro:Path.java getEndTangent(): normalised end tangent. */
  endTangent() { return this._curve.endTangent(); }
  /** Pedro:Path.java getLastControlPoint(). */
  lastControlPoint() { return cp(this._curve.lastPoint); }
  /** Pedro:Path.java getClosestLeftGradientVector(): normalised left gradient at the closest t. */
  leftGradient() { return vNormalize(this._curve.leftGradient(this._t)); }
  /** Pedro:Path.java endPose(): last control point with the heading goal at an end PathPoint(1, last, endTangent). */
  endPose() {
    const l = this._curve.lastPoint;
    return { x: l.x, y: l.y, h: this._interp({ tValue: 1, pose: cp(l), tangent: this._curve.endTangent() }) };
  }

  // Pedro:Path.java constraint getters.
  brakingStrength() { return this.constraints.brakingStrength; }
  brakingStartMultiplier() { return this.constraints.brakingStart; }
  endVelocityConstraint() { return this.constraints.velocityConstraint; }
  endTranslationalConstraint() { return this.constraints.translationalConstraint; }
  endHeadingConstraint() { return this.constraints.headingConstraint; }
  endTimeoutConstraint() { return this.constraints.timeoutConstraint; }
  endTValueConstraint() { return this.constraints.tValueConstraint; }
}

// ---------------------------------------------------------------------------------------------------------------------
// PathChain (Pedro:paths/PathChain.java)
// ---------------------------------------------------------------------------------------------------------------------
export class PathChain {
  /**
   * @param {Path[]} paths
   * @param {{decelerationType?:string, callbacks?:object[], headingInterpolator?:Function}} opts
   *   decelerationType 'NONE'|'GLOBAL'|'LAST_PATH' (Pedro default LAST_PATH). callbacks [{pathIndex, t?|ms?, name, seg}]
   *   are sorted by pathIndex then by their own trigger (Pedro:PathChain.java setCallbacks comparator).
   */
  constructor(paths, { decelerationType = 'LAST_PATH', callbacks = [], headingInterpolator = null } = {}) {
    this._paths = paths.slice();
    this.decelerationType = decelerationType;
    this._headingInterp = headingInterpolator;
    let len = 0;
    for (const p of this._paths) len += p.length;
    this._length = len;
    this._callbacks = callbacks.map((c) => ({ ...c })).sort((a, b) => {
      if (a.pathIndex !== b.pathIndex) return a.pathIndex - b.pathIndex;
      const ta = a.ms !== undefined, tb = b.ms !== undefined;
      if (!ta && !tb) return (a.t ?? 1) - (b.t ?? 1);
      if (ta && tb) return a.ms - b.ms;
      return 0;
    });
  }

  size() { return this._paths.length; }                    // Pedro:PathChain.java size()
  getPath(i) { return this._paths[i]; }                    // Pedro:PathChain.java getPath
  get paths() { return this._paths; }
  get callbacks() { return this._callbacks; }
  get length() { return this._length; }                    // Pedro:PathChain.java length()
  firstPath() { return this._paths[0] ?? null; }
  lastPath() { return this._paths[this._paths.length - 1]; }

  /** Pedro:PathChain.java getDistanceRemaining(int chainIndex): sum of the lengths of all later paths plus the current
   *  path's own remaining distance at its closest point. */
  distanceRemaining(chainIndex) {
    let rem = 0;
    if (chainIndex < this.size()) {
      for (let i = chainIndex + 1; i < this.size(); i++) rem += this._paths[i].length;
    }
    return rem + this._paths[chainIndex].distanceRemaining();
  }

  /** Pedro:PathChain.java getHeadingGoal(PathT(pathIndex, t)). With a chain-level interpolator the PathPoint's tValue is
   *  the chain completion (sum of earlier lengths + distance travelled on this path) / chain length. */
  headingGoal(pathIndex, t) {
    const path = this._paths[pathIndex];
    if (this._headingInterp) {
      let sum = 0;
      for (let i = 0; i < pathIndex; i++) sum += this._paths[i].length;
      const chainT = sum / this._length + path.distanceTraveled(t) / this._length;
      return this._headingInterp({ tValue: chainT, pose: path.curve.pose(t), tangent: path.curve.deriv(t) });
    }
    return path.headingGoalAt(t);
  }

  /** Pedro:PathChain.java getFinalHeadingGoal(). */
  finalHeadingGoal() {
    const last = this.lastPath();
    if (!this._headingInterp) return last.headingGoalAt(1);
    const e = last.endPose();
    return this._headingInterp({ tValue: 1, pose: { x: e.x, y: e.y }, tangent: last.endTangent() });
  }

  /** Pedro:PathChain.java endPose(). */
  endPose() { return this.lastPath().endPose(); }
}

// ---------------------------------------------------------------------------------------------------------------------
// Bridge request -> path groups. Mirrors tools_java/PedroFollowSim.java buildGroups (lines 196-239) exactly.
// ---------------------------------------------------------------------------------------------------------------------
const num = (o, k, d) => {
  const v = o ? o[k] : undefined;
  if (v === undefined || v === null) return d;
  const n = Number(v);
  return Number.isFinite(n) ? n : d;   // PedroSampler.num
};
const rad = (deg) => (deg * Math.PI) / 180; // Math.toRadians

/**
 * @param {object} body {start:{x,y,h(deg)}, segments:[{type,points:[{x,y}],heading:{mode,reversed,start?,end?}(deg),
 *                       callbacks?:[{t|ms,name}], waitAfter?(s)}]}
 * @param {object} constraints PathConstraints-shaped (params.constraints); missing keys use Pedro defaults.
 * @returns {{chain:PathChain, firstSeg:number, waitAfter:number, callbacks:object[]}[]}
 */
export function groupsFromBridgeRequest(body, constraints = DEFAULT_CONSTRAINTS) {
  const segs = body.segments || [];
  if (!segs.length) throw new Error('route has no segments');
  const st = body.start || {};
  let px = num(st, 'x', 0), py = num(st, 'y', 0), ph = num(st, 'h', 0); // ph in degrees
  const groups = [];
  let cur = { paths: [], callbacks: [], firstSeg: 0, waitAfter: 0 };
  for (let si = 0; si < segs.length; si++) {
    const sg = segs[si];
    const type = sg.type == null ? 'line' : String(sg.type);
    const pts = (sg.points || []).map((p) => ({ x: num(p, 'x', 0), y: num(p, 'y', 0) }));
    if (pts.length === 1) pts.unshift({ x: px, y: py });
    // PedroFollowSim: type 'line' -> BezierLine(first, last); else BezierCurve(pts) (>= 3 pts; 2 degrade to a line, D3).
    const curve = type === 'line'
      ? new BezierLine(pts[0], pts[pts.length - 1], constraints)
      : makeCurve('curve', pts, constraints);
    const path = new Path(curve, constraints);
    const hd = sg.heading || {};
    const mode = hd.mode == null ? 'constant' : String(hd.mode);
    const rev = hd.reversed === true;
    const hs = num(hd, 'start', ph);
    let he = num(hd, 'end', hs);
    if (mode === 'tangent') {
      path.setTangentHeading();
      if (rev) path.reverseHeading();
    } else if (mode === 'linear') {
      path.setLinearHeading(rad(hs), rad(he), 1.0, rev);
    } else {
      path.setConstantHeading(rad(hs));
      he = hs;
    }
    const last = pts[pts.length - 1];
    px = last.x; py = last.y;
    ph = mode === 'tangent' ? ph : he;
    const local = cur.paths.length;
    cur.paths.push(path);
    for (const cb of sg.callbacks || []) {
      const ms = num(cb, 'ms', -1);
      const name = cb.name == null ? 'callback' : String(cb.name);
      const entry = { pathIndex: local, name, seg: cur.firstSeg + local };
      if (ms >= 0) entry.ms = ms; else entry.t = num(cb, 't', 1.0);
      cur.callbacks.push(entry);
    }
    const wa = num(sg, 'waitAfter', 0);
    if (wa > 0 && si < segs.length - 1) {
      cur.waitAfter = wa;
      groups.push(cur);
      cur = { paths: [], callbacks: [], firstSeg: si + 1, waitAfter: 0 };
    } else if (si === segs.length - 1) {
      cur.waitAfter = Math.max(0, wa);
    }
  }
  groups.push(cur);
  return groups.map((g) => ({
    chain: new PathChain(g.paths, { callbacks: g.callbacks }),
    firstSeg: g.firstSeg,
    waitAfter: g.waitAfter,
    callbacks: g.callbacks.map((c) => ({ ...c })),
  }));
}
