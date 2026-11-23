/**
 * follower.js - Pedro Pathing 2.1.0 ErrorCalculator, VectorCalculator and Follower state machine in JS.
 *
 * Source of record (cached 2.1.0 core source jar):
 *   Pedro:ErrorCalculator.java
 *   Pedro:VectorCalculator.java
 *   Pedro:follower/Follower.java  (update 486-596, holdPoint 212-251, followPath 260-325,
 *                                  breakFollowing 608-618, turn/turnTo 724-770, atParametricEnd 646,
 *                                  isRobotStuck 712, getClosestPointHeadingGoal 1041, getPathCompletion 1117,
 *                                  getTotalDistanceRemaining 1188)
 *   Pedro:paths/PathChain.java  (getNextPathCallbacks 440), paths/callbacks/*
 *
 * Units: pose x,y in inches, h in radians ([0, 2PI) from the localizer); velocity in/s; powers in [-1,1];
 * time only from the injected SimClock (ms for stuck / end timers, ns inside the PIDFs).
 *
 * Deviations from Pedro 2.1.0 (every one listed here):
 *   1. Time comes from SimClock, not System.currentTimeMillis / Timer / nanoTime.
 *   2. PIDF dt <= 0 guard lives in pedromath.js (derivative 0, no integral); Java would give Inf/NaN.
 *   3. Zero-vector theta is 0 (Pedro quirk, kept) via pedromath vTheta/vSetMag.
 *   4. Callbacks: by default INTENDED semantics (fire once; parametric when its path is passed or its
 *      t is reached, temporal ms after that path started; none are lost). With
 *      params.sim.emulate210CallbackBug the 2.1.0 getNextPathCallbacks queue and the TemporalCallback
 *      unit bug are reproduced (path-0 callbacks never fire, temporal callbacks fire immediately).
 *      Callbacks fire through onCallback listeners instead of Runnables; they are wrapped fire-once
 *      like PathBuilder's FiniteRunAction.
 *   5. Unit tangent / end-tangent are normalised locally so a P1 path may return either form.
 *   6. Path.init() is called only if the P1 Path has one.
 *   7. Static switch flags in Java's VectorCalculator (static fields) are per instance here.
 *   8. Teleop / manual drive, pausePathFollowing and the PoseTracker history are not ported.
 *   9. turnTo() reproduces the 2.1.0 expression getHeading()+smallestAngleDiff(...) exactly. Because
 *      smallestAngleDiff is unsigned, a clockwise target is reached by turning counter-clockwise by the
 *      unsigned difference (Follower.java:745-750). Kept as is.
 * Teleop/manual drive is out of scope.
 */

import {
  smallestAngleDiff, turnDirection, normalizeAngleSigned, clamp,
  vPolar, vMag, vTheta, vAdd, vSub, vScale, vDot, vSetMag,
  findNormalizingScaling, Kinematics, PIDF, FilteredPIDF, KalmanFilter, PredictiveBraking,
} from './pedromath.js';
import { Path, makeCurve } from './pedropath.js';

const ZERO = () => ({ x: 0, y: 0 });

/** Local unit vector; zero stays zero. Pedro:math/Vector.java normalize(). */
function unit(v) {
  const m = vMag(v);
  return m === 0 ? ZERO() : { x: v.x / m, y: v.y / m };
}

/** Pedro:math/Vector.java projectOnto(): zero vector if |other| == 0. */
function project(a, onto) {
  const d = vDot(onto, onto);
  if (d === 0) return ZERO();
  const s = vDot(a, onto) / d;
  return { x: onto.x * s, y: onto.y * s };
}

// =====================================================================================================
// ErrorCalculator  (Pedro:ErrorCalculator.java)
// =====================================================================================================
export class ErrorCalculator {
  /** @param {object} params resolved params (reads params.pedro.*) */
  constructor(params) {
    this._c = params.pedro;
    // ErrorCalculator.java:44-52
    this._kalman = new KalmanFilter({
      modelCovariance: this._c.driveKalmanFilterModelCovariance,
      dataCovariance: this._c.driveKalmanFilterDataCovariance,
    });
    this._ctx = null;
    this._driveError = null; // Double driveError (null = not yet computed this loop)
    this._rawDriveError = 0;
    this._prevRawDriveError = 0;
    this._headingError = 0;
    this._driveErrors = [0, 0];
    this.breakFollowing();
  }

  /**
   * ErrorCalculator.java:61-80. ctx = {pose, path, chain, followingChain, closestPose{x,y}, velocity{x,y},
   * chainIndex, xVelocity, yVelocity, headingGoal, usePredictiveBraking}.
   */
  update(ctx) {
    this._ctx = ctx;
    this._driveError = null;
  }

  /** ErrorCalculator.java:82-95 getTranslationalError(): closest - current, field frame, in. */
  translationalError() {
    const c = this._ctx;
    if (!c) return { x: 0, y: 0 };
    return { x: c.closestPose.x - c.pose.x, y: c.closestPose.y - c.pose.y };
  }

  /** ErrorCalculator.java:97-117 getHeadingError(): signed, rad. 0 if no path. */
  headingError() {
    const c = this._ctx;
    if (!c || !c.path) return 0;
    this._headingError = turnDirection(c.pose.h, c.headingGoal) * smallestAngleDiff(c.pose.h, c.headingGoal);
    return this._headingError;
  }

  /**
   * ErrorCalculator.java:120-174 getDriveVelocityError(): Kalman-filtered velocity error along the tangent, in/s.
   * @param {number} distanceToGoal in (signed along the tangent)
   */
  _driveVelocityError(distanceToGoal) {
    const c = this._ctx;
    if (!c.path) return 0;
    const k = this._c;
    const tangent = unit(c.path.tangent);
    const distVec = vScale(tangent, distanceToGoal);
    const velocity = project(c.velocity, tangent);

    const fwd = vPolar(1.0, c.pose.h);
    const fwdVel = vDot(fwd, velocity);
    const fwdDist = vDot(fwd, distVec);
    const fwdGoal = Kinematics.velToStopWithDecel(fwdDist, k.forwardZeroPowerAcceleration);
    const fwdDecay = fwdVel - Kinematics.finalVelAtDistance(fwdVel, k.forwardZeroPowerAcceleration, fwdDist);

    const lat = vPolar(1.0, c.pose.h - Math.PI / 2);
    const latVel = vDot(lat, velocity);
    const latDist = vDot(lat, distVec);
    const latGoal = Kinematics.velToStopWithDecel(latDist, k.lateralZeroPowerAcceleration);
    const latDecay = latVel - Kinematics.finalVelAtDistance(latVel, k.lateralZeroPowerAcceleration, latDist);

    // Vector(magnitude, theta): signed magnitude along each heading axis.
    const fErr = vScale(fwd, fwdGoal - fwdDecay - fwdVel);
    const lErr = vScale(lat, latGoal - latDecay - latVel);
    const errVec = vAdd(fErr, lErr);

    this._prevRawDriveError = this._rawDriveError;
    this._rawDriveError = vMag(errVec) * Math.sign(vDot(errVec, tangent));

    const projection = Kinematics.predictNextLoopVelocity(this._driveErrors[1], this._driveErrors[0]);
    this._kalman.update(this._rawDriveError - this._prevRawDriveError, projection);
    this._driveErrors[0] = this._driveErrors[1];
    this._driveErrors[1] = this._kalman.state;
    return this._kalman.state;
  }

  /**
   * ErrorCalculator.java:176-228 getDriveError(): -1 means "full power along the tangent" (non-final path of
   * a LAST_PATH/NONE chain, or GLOBAL outside braking distance). Cached until the next update().
   */
  driveError() {
    const c = this._ctx;
    if (c.usePredictiveBraking) return 0;
    if (this._driveError !== null) return this._driveError;
    if (!c.path) return 0;

    let distanceToGoal;
    if (!c.path.isAtParametricEnd()) {
      if (c.followingChain) {
        const type = c.chain.decelerationType;
        if (type === 'GLOBAL') {
          distanceToGoal = c.chain.distanceRemaining(c.chainIndex);
          const tangent = unit(c.path.tangent);
          const fwdTheo = vPolar(1.0, c.headingGoal);
          const stopDist = Kinematics.stoppingDistance(
            c.yVelocity + (c.xVelocity - c.yVelocity) * Math.abs(vDot(fwdTheo, tangent)),
            this._c.forwardZeroPowerAcceleration,
          );
          if (distanceToGoal >= stopDist * c.path.brakingStartMultiplier()) return -1;
        } else if ((type === 'LAST_PATH' && c.chainIndex < c.chain.size() - 1) || type === 'NONE') {
          return -1;
        } else {
          distanceToGoal = c.path.distanceRemaining();
        }
      } else {
        distanceToGoal = c.path.distanceRemaining();
      }
    } else {
      const offset = vSub(c.path.lastControlPoint(), c.pose);
      distanceToGoal = vDot(unit(c.path.endTangent()), offset);
    }
    this._driveError = this._driveVelocityError(distanceToGoal);
    return this._driveError;
  }

  /** ErrorCalculator.java:230 */
  rawDriveError() { return this._rawDriveError; }

  /** ErrorCalculator.java:238-247: zero everything, driveError = 0.0 (cached zero until next update()). */
  breakFollowing() {
    this._driveError = 0.0;
    this._headingError = 0;
    this._rawDriveError = 0;
    this._prevRawDriveError = 0;
    this._driveErrors = [0, 0];
    this._kalman.reset();
  }
}

// =====================================================================================================
// VectorCalculator  (Pedro:VectorCalculator.java)
// =====================================================================================================
export class VectorCalculator {
  /**
   * @param {object} params resolved params (params.pedro.*)
   * @param {import('./pedromath.js').SimClock} clock
   */
  constructor(params, clock) {
    const k = params.pedro;
    this._k = k;
    // VectorCalculator.java:76-85
    this._drivePIDF = new FilteredPIDF(k.coefficientsDrivePIDF, clock);
    this._secondaryDrivePIDF = new FilteredPIDF(k.coefficientsSecondaryDrivePIDF, clock);
    this._headingPIDF = new PIDF(k.coefficientsHeadingPIDF, clock);
    this._secondaryHeadingPIDF = new PIDF(k.coefficientsSecondaryHeadingPIDF, clock);
    this._translationalPIDF = new PIDF(k.coefficientsTranslationalPIDF, clock);
    this._secondaryTranslationalPIDF = new PIDF(k.coefficientsSecondaryTranslationalPIDF, clock);
    this._translationalIntegral = new PIDF(k.integralTranslational, clock);
    this._secondaryTranslationalIntegral = new PIDF(k.integralSecondaryTranslational, clock);
    this._predictive = new PredictiveBraking(k.predictiveBrakingCoefficients);
    this._ctx = null;
    this._last = {
      drive: ZERO(), heading: ZERO(), translational: ZERO(), centripetal: ZERO(), corrective: ZERO(),
    };
    this.breakFollowing();
  }

  /**
   * VectorCalculator.java:130-169. ctx = {useDrive,useHeading,useTranslational,useCentripetal,chainIndex,
   * maxPowerScaling,followingChain,centripetalScaling,pose,closestPose,velocity,path,chain,driveError,
   * translationalError,headingError,headingGoal,distanceRemaining,usePredictiveBraking}.
   */
  update(ctx) {
    const k = this._k;
    // updateConstants() each loop (VectorCalculator.java:104-119)
    this._drivePIDF.setCoefficients(k.coefficientsDrivePIDF);
    this._secondaryDrivePIDF.setCoefficients(k.coefficientsSecondaryDrivePIDF);
    this._headingPIDF.setCoefficients(k.coefficientsHeadingPIDF);
    this._secondaryHeadingPIDF.setCoefficients(k.coefficientsSecondaryHeadingPIDF);
    this._translationalPIDF.setCoefficients(k.coefficientsTranslationalPIDF);
    this._secondaryTranslationalPIDF.setCoefficients(k.coefficientsSecondaryTranslationalPIDF);
    this._translationalIntegral.setCoefficients(k.integralTranslational);
    this._secondaryTranslationalIntegral.setCoefficients(k.integralSecondaryTranslational);
    this._predictive.setCoefficients(k.predictiveBrakingCoefficients);
    this._ctx = ctx;
  }

  /** VectorCalculator.java:171-205 breakFollowing(). */
  breakFollowing() {
    this._secondaryDrivePIDF.reset();
    this._drivePIDF.reset();
    this._secondaryHeadingPIDF.reset();
    this._headingPIDF.reset();
    this._secondaryTranslationalPIDF.reset();
    this._secondaryTranslationalIntegral.reset();
    this._translationalPIDF.reset();
    this._translationalIntegral.reset();
    this._secondaryTranslationalIntegralVector = ZERO();
    this._translationalIntegralVector = ZERO();
    this._last = {
      drive: ZERO(), heading: ZERO(), translational: ZERO(), centripetal: ZERO(), corrective: ZERO(),
    };
    this._prevSecondaryTranslationalIntegral = 0;
    this._prevTranslationalIntegral = 0;
  }

  /** VectorCalculator.java:215-251 getDriveVector(). Power units, along the path tangent. */
  driveVector() {
    const c = this._ctx;
    if (!c.useDrive) return ZERO();
    const maxP = c.maxPowerScaling;
    const tanTheta = vTheta(c.path.tangent);

    if (c.followingChain && ((c.chainIndex < c.chain.size() - 1 && c.chain.decelerationType === 'LAST_PATH')
        || c.chain.decelerationType === 'NONE')) {
      return vPolar(maxP, tanTheta);
    }
    if (c.usePredictiveBraking) {
      if (c.distanceRemaining === -1) return vPolar(maxP, tanTheta);
      return vPolar(
        this._predictive.computeOutput(c.distanceRemaining, vDot(c.velocity, unit(c.path.tangent))),
        tanTheta,
      );
    }
    if (c.driveError === -1) return vPolar(maxP, tanTheta);

    const theta = vTheta(unit(c.path.tangent));
    const k = this._k;
    if (Math.abs(c.driveError) < k.drivePIDFSwitch && k.useSecondaryDrivePIDF) {
      this._secondaryDrivePIDF.updateFeedForwardInput(Math.sign(c.driveError));
      this._secondaryDrivePIDF.updateError(c.driveError);
      this._last.drive = vPolar(clamp(this._secondaryDrivePIDF.run(), -maxP, maxP), theta);
      return { ...this._last.drive };
    }
    this._drivePIDF.updateFeedForwardInput(Math.sign(c.driveError));
    this._drivePIDF.updateError(c.driveError);
    this._last.drive = vPolar(clamp(this._drivePIDF.run(), -maxP, maxP), theta);
    return { ...this._last.drive };
  }

  /** VectorCalculator.java:257-273 getHeadingVector(): signed power along the robot heading direction. */
  headingVector() {
    const c = this._ctx;
    if (!c.useHeading) return ZERO();
    const k = this._k;
    const maxP = c.maxPowerScaling;
    const ff = turnDirection(c.pose.h, c.headingGoal);
    if (Math.abs(c.headingError) < k.headingPIDFSwitch && k.useSecondaryHeadingPIDF) {
      this._secondaryHeadingPIDF.updateFeedForwardInput(ff);
      this._secondaryHeadingPIDF.updateError(c.headingError);
      this._last.heading = vPolar(clamp(this._secondaryHeadingPIDF.run(), -maxP, maxP), c.pose.h);
      return { ...this._last.heading };
    }
    this._headingPIDF.updateFeedForwardInput(ff);
    this._headingPIDF.updateError(c.headingError);
    this._last.heading = vPolar(clamp(this._headingPIDF.run(), -maxP, maxP), c.pose.h);
    return { ...this._last.heading };
  }

  /** VectorCalculator.java:275-288 getCorrectiveVector() = centripetal + translational, normalised to maxPower. */
  correctiveVector() {
    const maxP = this._ctx.maxPowerScaling;
    const centripetal = this.centripetalCorrection();
    const translational = this.translationalCorrection();
    const corrective = vAdd(centripetal, translational);
    if (vMag(corrective) > maxP) {
      return vAdd(centripetal, vScale(translational, findNormalizingScaling(centripetal, translational, maxP)));
    }
    this._last.corrective = { ...corrective };
    return corrective;
  }

  /**
   * VectorCalculator.java:294-352 getTranslationalCorrection(). Order of state updates matches Java exactly
   * (tangent component removed from error and both integral vectors, then primary or secondary PIDF).
   */
  translationalCorrection() {
    const c = this._ctx;
    if (!c.useTranslational) return ZERO();
    const k = this._k;
    const maxP = c.maxPowerScaling;
    let tv = { ...c.translationalError };

    if (c.usePredictiveBraking) {
      if (c.path.isAtParametricEnd()) {
        return {
          x: this._predictive.computeOutput(c.translationalError.x, c.velocity.x),
          y: this._predictive.computeOutput(c.translationalError.y, c.velocity.y),
        };
      }
      const normal = c.path.leftGradient();
      return vScale(normal, this._predictive.computeOutput(vDot(c.translationalError, normal), vDot(c.velocity, normal)));
    }

    if (!(c.path.isAtParametricEnd() || c.path.isAtParametricStart())) {
      const tan = unit(c.path.tangent);
      const tanTheta = vTheta(c.path.tangent);
      tv = vSub(tv, vPolar(vDot(tv, tan), tanTheta));
      this._secondaryTranslationalIntegralVector = vSub(
        this._secondaryTranslationalIntegralVector,
        vPolar(vDot(this._secondaryTranslationalIntegralVector, tan), tanTheta));
      this._translationalIntegralVector = vSub(
        this._translationalIntegralVector,
        vPolar(vDot(this._translationalIntegralVector, tan), tanTheta));
    }

    const dist = Math.hypot(c.pose.x - c.closestPose.x, c.pose.y - c.closestPose.y);
    const tvMag = vMag(tv);
    const tvTheta = vTheta(tv);
    if (dist < k.translationalPIDFSwitch && k.useSecondaryTranslationalPIDF) {
      this._secondaryTranslationalIntegral.updateError(tvMag);
      this._secondaryTranslationalIntegralVector = vAdd(
        this._secondaryTranslationalIntegralVector,
        vPolar(this._secondaryTranslationalIntegral.run() - this._prevSecondaryTranslationalIntegral, tvTheta));
      this._prevSecondaryTranslationalIntegral = this._secondaryTranslationalIntegral.run();

      this._secondaryTranslationalPIDF.updateFeedForwardInput(1);
      this._secondaryTranslationalPIDF.updateError(tvMag);
      tv = vSetMag(tv, this._secondaryTranslationalPIDF.run());
      tv = vAdd(tv, this._secondaryTranslationalIntegralVector);
    } else {
      this._translationalIntegral.updateError(tvMag);
      this._translationalIntegralVector = vAdd(
        this._translationalIntegralVector,
        vPolar(this._translationalIntegral.run() - this._prevTranslationalIntegral, tvTheta));
      this._prevTranslationalIntegral = this._translationalIntegral.run();

      this._translationalPIDF.updateFeedForwardInput(1);
      this._translationalPIDF.updateError(tvMag);
      tv = vSetMag(tv, this._translationalPIDF.run());
      tv = vAdd(tv, this._translationalIntegralVector);
    }

    tv = vSetMag(tv, clamp(vMag(tv), 0, maxP));
    this._last.translational = { ...tv };
    return tv;
  }

  /**
   * VectorCalculator.java:354-366 getCentripetalForceCorrection().
   * magnitude = clamp(centripetalScaling * mass * (v.T)^2 * curvature, +-maxPower),
   * direction = theta(T) + PI/2 * sign(theta(normal)); the normal theta is in [0, 2PI) so sign >= 0.
   */
  centripetalCorrection() {
    const c = this._ctx;
    if (!c.useCentripetal) return ZERO();
    const curvature = c.path.curvature;
    if (Number.isNaN(curvature)) return ZERO();
    const maxP = c.maxPowerScaling;
    const vt = vDot(c.velocity, unit(c.path.tangent));
    const mag = clamp(c.centripetalScaling * this._k.mass * vt * vt * curvature, -maxP, maxP);
    this._last.centripetal = vPolar(
      mag, vTheta(c.path.tangent) + (Math.PI / 2) * Math.sign(vTheta(c.path.normal)));
    return { ...this._last.centripetal };
  }

  get last() { return this._last; }
}

// =====================================================================================================
// PedroFollower  (Pedro:follower/Follower.java)
// =====================================================================================================
export class PedroFollower {
  /**
   * @param {object} o
   * @param {object} o.params resolved params
   * @param {import('./pedromath.js').SimClock} o.clock
   * @param {{update():void,pose():{x,y,h},velocity():{x,y}}} o.localizer field-frame localizer
   * @param {import('./mecanumdrive.js').MecanumDrivetrain} o.drivetrain
   */
  constructor({ params, clock, localizer, drivetrain }) {
    this._params = params;
    this._k = params.pedro;
    this._cons = params.constraints;
    this._emulateBug = !!(params.sim && params.sim.emulate210CallbackBug);
    this._clock = clock;
    this._loc = localizer;
    this._dt = drivetrain;
    this._errors = new ErrorCalculator(params);
    this._vectors = new VectorCalculator(params, clock);

    this._path = null;
    this._chain = null;
    this._chainIndex = 0;
    this._followingChain = false;
    this._isBusy = false;
    this._isTurning = false;
    this._holding = false;
    this._useHoldScaling = true;
    this._holdEnd = false;
    this._reachedEnd = false;
    this._reachedEndMs = 0;
    this._stuckStartMs = null;
    this._globalMaxPower = 1.0;
    this._closest = { tValue: 0, pose: ZERO(), tangent: { x: 1, y: 0 } };
    this._pose = { x: 0, y: 0, h: 0 };
    this._vel = ZERO();
    this._listeners = new Set();
    this._cb = [];            // per-chain-follow callback records
    this._pathStartMs = [];   // ms the chain's path i was started (intended semantics)
    this._cbQueue = [];       // 2.1.0 emulation: currentCallbacks queue
    this._cbIndex = 0;        // 2.1.0 emulation: PathChain.currentCallbackIndex

    // constructor-time reset of the controllers (Follower ctor -> breakFollowing)
    this._errors.breakFollowing();
    this._vectors.breakFollowing();
    this._dt.breakFollowing();
  }

  // ---- derived "constants refreshed each update" (Follower.java:98-106) --------------------------------
  /** usePredictiveBraking = !manualDrive && constants.usePredictiveBraking. Teleop is not ported. */
  get _usePredictiveBraking() { return !!this._k.usePredictiveBraking; }

  /** Follower.java:200-202 */
  setStartingPose(pose) {
    if (typeof this._loc.setPose === 'function') this._loc.setPose(pose);
    this._pose = { x: pose.x, y: pose.y, h: pose.h };
  }

  /** Follower.setMaxPower (cap applied by followPath(Path)). */
  setMaxPower(m) { this._globalMaxPower = m; this._dt.setMaxPowerScaling(m); }

  /** Follower.java:226-238 setPath: this.currentPath = path; path.init(). */
  _setPath(path) {
    this._path = path;
    if (typeof path.init === 'function') path.init();
  }

  _updateClosest(limit) {
    this._closest = this._path.updateClosestPose(this._pose, limit);
    return this._closest;
  }

  /**
   * Follower.java:212-251 holdPoint(BezierPoint, heading, useHoldScaling) + the Pose overloads.
   * Holds a point with a constant heading until another command arrives. isBusy becomes false.
   * @param {{x:number,y:number,h:number}} pose in, in, rad
   * @param {boolean} [useHoldScaling=true] scale translational by holdPointTranslationalScaling (0.45), heading by 0.35
   */
  holdPoint(pose, useHoldScaling = true) {
    this.breakFollowing();
    this._holding = true;
    this._useHoldScaling = useHoldScaling;
    this._isBusy = false;
    this._followingChain = false;
    const pt = { x: pose.x, y: pose.y };
    const path = new Path(makeCurve('point', [pt], this._cons), this._cons);
    this._setPath(path);
    path.setConstantHeading(pose.h);
    this._closest = path.updateClosestPose(this._pose, 1);
  }

  /** Follower.java:260-275 followPath(Path, holdEnd). */
  followPath(path, holdEnd = this._k.automaticHoldEnd) {
    this._dt.setMaxPowerScaling(this._globalMaxPower);
    this.breakFollowing();
    this._holdEnd = holdEnd;
    this._isBusy = true;
    this._followingChain = false;
    this._setPath(path);
    this._updateClosest(this._k.BEZIER_CURVE_SEARCH_LIMIT);
  }

  /**
   * Follower.java:305-325 followPath(PathChain, maxPower, holdEnd).
   * @param {import('./pedropath.js').PathChain} chain
   * @param {{maxPower?:number, holdEnd?:boolean}} [o]
   */
  followChain(chain, { maxPower = 1, holdEnd = this._k.automaticHoldEnd } = {}) {
    this._dt.setMaxPowerScaling(maxPower);
    this.breakFollowing();
    this._holdEnd = holdEnd;
    this._isBusy = true;
    this._followingChain = true;
    this._chainIndex = 0;
    this._chain = chain;
    this._setPath(chain.getPath(0));
    this._updateClosest(this._k.BEZIER_CURVE_SEARCH_LIMIT);
    // resetCallbacks(); currentCallbacks = getNextPathCallbacks(0); initialize()
    this._cb = (chain.callbacks || []).map((c) => ({ ...c, fired: false }));
    this._pathStartMs = [];
    this._pathStartMs[0] = this._clock.nowMs();
    this._cbIndex = 0;
    this._cbQueue = [];
    if (this._emulateBug) this._queueNextCallbacks(0);
  }

  /** Follower.java:724-742 turn(radians, counterClockwise): relative turn, hold without scaling. */
  turn(rad, counterClockwise = true) {
    this.holdPoint({ x: this._pose.x, y: this._pose.y, h: this._pose.h + (counterClockwise ? rad : -rad) }, false);
    this._isTurning = true;
    this._isBusy = true;
  }

  /** Follower.java:745-750 turnTo(radians). See deviation 9. */
  turnTo(rad) {
    const h = this._pose.h;
    const heading = normalizeAngleSigned(h + smallestAngleDiff(h, rad));
    this.holdPoint({ x: this._pose.x, y: this._pose.y, h: heading }, false);
    this._isTurning = true;
    this._isBusy = true;
  }

  /**
   * Follower.java:608-618 breakFollowing(): reset PIDFs, zero the drive (FLOAT), clear hold/busy/turn flags,
   * the parametric-end latch and the stuck timer (the ONLY place the stuck timer resets).
   */
  breakFollowing() {
    this._errors.breakFollowing();
    this._vectors.breakFollowing();
    this._dt.breakFollowing();
    this._holding = false;
    this._isBusy = false;
    this._isTurning = false;
    this._reachedEnd = false;
    this._stuckStartMs = null;
  }

  // ---- helpers mirroring Follower getters -------------------------------------------------------------

  /** Follower.java:1041 getClosestPointHeadingGoal(). */
  _headingGoal() {
    if (!this._path) return 0;
    if (this._followingChain && this._chain) return this._chain.headingGoal(this._chainIndex, this._closest.tValue);
    return this._path.headingGoalAt(this._closest.tValue);
  }

  /** Follower.java:1188 getTotalDistanceRemaining(). -1 for NONE chains. */
  _totalDistanceRemaining() {
    if (!this._path) return 0;
    if (!this._followingChain) return this._path.distanceRemaining();
    if (this._chain.decelerationType === 'NONE') return -1;
    return this._chain.distanceRemaining(this._chainIndex);
  }

  /** Follower.java:446-456 updateErrors(). */
  _updateErrors() {
    this._errors.update({
      pose: this._pose, path: this._path, chain: this._chain, followingChain: this._followingChain,
      closestPose: this._closest.pose, velocity: this._vel, chainIndex: this._chainIndex,
      xVelocity: this._dt.xVelocity(), yVelocity: this._dt.yVelocity(),
      headingGoal: this._headingGoal(), usePredictiveBraking: this._usePredictiveBraking,
    });
  }

  /** Follower.java:459-477 updateVectors(). Argument evaluation order matches Java. */
  _updateVectors() {
    const driveError = !this._holding ? this._errors.driveError() : -1; // useDrive is always true
    const translationalError = this._errors.translationalError();
    const headingError = this._errors.headingError();
    this._vectors.update({
      useDrive: true, useHeading: true, useTranslational: true, useCentripetal: true,
      chainIndex: this._chainIndex, maxPowerScaling: this._dt.getMaxPowerScaling(),
      followingChain: this._followingChain, centripetalScaling: this._k.centripetalScaling,
      pose: this._pose, closestPose: this._closest.pose, velocity: this._vel,
      path: this._path, chain: this._chain, driveError, translationalError, headingError,
      headingGoal: this._headingGoal(), distanceRemaining: this._totalDistanceRemaining(),
      usePredictiveBraking: this._usePredictiveBraking,
    });
  }

  _updateErrorAndVectors() { this._updateErrors(); this._updateVectors(); }

  // ---- callbacks ----------------------------------------------------------------------------------------

  _fire(c) {
    if (c.fired) return;
    c.fired = true;
    for (const fn of this._listeners) fn(c.name, c.seg, c.pathIndex);
  }

  /** PathChain.java:440-460 getNextPathCallbacks(chainIndex) + callback.initialize() (2.1.0 emulation only). */
  _queueNextCallbacks(chainIndex) {
    const cbs = this._cb;
    const q = [];
    if (cbs.length === 0 || this._cbIndex >= cbs.length) { this._cbQueue = q; return; }
    const lastIndex = this._cbIndex === 0 ? 0 : cbs[this._cbIndex].pathIndex - 1;
    if (chainIndex > lastIndex + 1) {
      while (this._cbIndex < cbs.length && cbs[this._cbIndex].pathIndex < chainIndex) this._cbIndex++;
    } else if (chainIndex < lastIndex + 1) {
      while (this._cbIndex > 0 && cbs[this._cbIndex - 1].pathIndex >= chainIndex) this._cbIndex--;
    }
    while (this._cbIndex < cbs.length && cbs[this._cbIndex].pathIndex === lastIndex + 1) {
      cbs[this._cbIndex].startMs = this._clock.nowMs(); // TemporalCallback.initialize()
      q.push(cbs[this._cbIndex]);
      this._cbIndex++;
    }
    this._cbQueue = q;
  }

  /** Follower.java:598-604 updateCallbacks(); intended semantics unless the 2.1.0 bug flag is set. */
  _updateCallbacks() {
    if (this._emulateBug) {
      for (const c of this._cbQueue) {
        if (c.fired) continue;
        let ready;
        if (c.t !== undefined && c.t !== null) {
          // ParametricCallback.isReady: follower.atParametricEnd() || getPathCompletion() >= t
          ready = this.atParametricEnd() || this.getPathCompletion() >= c.t;
        } else {
          // TemporalCallback.isReady with the NANOSECONDS.convert(x, MILLISECONDS) unit bug:
          // both times are scaled by 1e6, so (elapsedMs * 1e6) >= ms
          ready = (this._clock.nowMs() - c.startMs) * 1e6 >= c.ms;
        }
        if (ready) this._fire(c);
      }
      return;
    }
    const now = this._clock.nowMs();
    for (const c of this._cb) {
      if (c.fired) continue;
      let ready = false;
      if (c.pathIndex < this._chainIndex && (c.t !== undefined && c.t !== null)) {
        ready = true;
      } else if (c.pathIndex <= this._chainIndex) {
        if (c.t !== undefined && c.t !== null) {
          const own = this._chain.getPath(c.pathIndex);
          ready = this._chainIndex > c.pathIndex || own.isAtParametricEnd() || this._closest.tValue >= c.t;
        } else {
          const start = this._pathStartMs[c.pathIndex];
          ready = start !== undefined && now - start >= c.ms;
        }
      }
      if (ready) this._fire(c);
    }
  }

  /** Intended semantics: any callback not yet fired fires when the group ends. */
  _fireRemaining() {
    if (this._emulateBug) return;
    for (const c of this._cb) this._fire(c);
  }

  /**
   * Subscribe to callback events.
   * @param {(name:string, seg:number, pathIndex:number)=>void} fn
   * @returns {()=>void} unsubscribe
   */
  onCallback(fn) {
    this._listeners.add(fn);
    return () => this._listeners.delete(fn);
  }

  // ---- main loop -----------------------------------------------------------------------------------------

  /**
   * Follower.java:486-596 update(). One control loop; call once per loop.dt after advancing the clock.
   * Order: localizer -> (no path: return) -> HOLD branch -> BUSY branch -> stuck timer -> advance/end logic.
   */
  update() {
    // updatePose (Follower.java:421-425)
    this._loc.update();
    const p = this._loc.pose();
    this._pose = { x: p.x, y: p.y, h: p.h };
    const v = this._loc.velocity();
    this._vel = { x: v.x, y: v.y };

    if (!this._path) return;
    const nowMs = this._clock.nowMs();
    const kk = this._k;
    const dt = this._dt;

    if (this._holding) {
      this._updateClosest(1);
      this._updateErrorAndVectors();
      const trans = this._vectors.translationalCorrection();
      const head = this._vectors.headingVector();
      if (this._useHoldScaling) {
        dt.runDrive(vScale(trans, kk.holdPointTranslationalScaling), vScale(head, kk.holdPointHeadingScaling),
          ZERO(), this._pose.h);
      } else {
        dt.runDrive(trans, head, ZERO(), this._pose.h);
      }
      if (Math.abs(this._errors.headingError()) < kk.turnHeadingErrorThreshold && this._isTurning) {
        this._isTurning = false;
        this._isBusy = false;
      }
      return;
    }

    if (this._isBusy) {
      this._updateClosest(kk.BEZIER_CURVE_SEARCH_LIMIT);
      this._updateErrorAndVectors();
      if (this._followingChain) this._updateCallbacks();
      const corr = this._vectors.correctiveVector();
      const head = this._vectors.headingVector();
      const drive = this._vectors.driveVector();
      dt.runDrive(corr, head, drive, this._pose.h);
    }

    // stuck detection (Follower.java:528-530; credited to FTC team 21229)
    if (vMag(this._vel) < kk.stuckVelocity && this._path.closestT > kk.stuckTValue
        && this._stuckStartMs === null && this._isBusy) {
      this._stuckStartMs = nowMs;
    }

    const stuckExpired = this._stuckStartMs !== null && (nowMs - this._stuckStartMs) > kk.stuckTimeout;
    if (!(this._path.isAtParametricEnd() || stuckExpired)) return;

    if (this._followingChain && this._chainIndex < this._chain.size() - 1) {
      this.breakFollowing();
      this._isBusy = true;
      this._followingChain = true;
      this._chainIndex++;
      this._setPath(this._chain.getPath(this._chainIndex));
      this._pathStartMs[this._chainIndex] = nowMs;
      this._updateClosest(kk.BEZIER_CURVE_SEARCH_LIMIT);
      this._updateErrorAndVectors();
      if (this._emulateBug) this._queueNextCallbacks(this._chainIndex);
      return;
    }

    if (!this._reachedEnd) {
      this._reachedEnd = true;
      this._reachedEndMs = nowMs;
    }

    this._updateErrorAndVectors();
    const path = this._path;
    const posErr = Math.hypot(this._pose.x - this._closest.pose.x, this._pose.y - this._closest.pose.y);
    const timedOut = (nowMs - this._reachedEndMs) > path.endTimeoutConstraint();
    const settled = vMag(this._vel) < path.endVelocityConstraint()
      && posErr < path.endTranslationalConstraint()
      && smallestAngleDiff(this._pose.h, this._headingGoal()) < path.endHeadingConstraint();
    if (!(timedOut || settled)) return; // Java precedence: A || (B && C && D)

    this._fireRemaining();
    if (this._holdEnd) {
      this._holdEnd = false;
      const lcp = path.lastControlPoint();
      const goal = this._followingChain
        ? this._chain.headingGoal(this._chain.size() - 1, 1)
        : path.headingGoalAt(1);
      this.holdPoint({ x: lcp.x, y: lcp.y, h: goal });
    } else {
      this.breakFollowing();
    }
  }

  // ---- queries ----------------------------------------------------------------------------------------------
  isBusy() { return this._isBusy; }
  isTurning() { return this._isTurning; }
  isHolding() { return this._holding; }
  /** Follower.java:712 isRobotStuck(): the stuck timer has been started. */
  isRobotStuck() { return this._stuckStartMs !== null; }

  /** Follower.java:646-660 atParametricEnd(): last path of a chain only. */
  atParametricEnd() {
    if (!this._path) return true;
    if (this._followingChain) {
      if (this._chainIndex === this._chain.size() - 1) return this._path.isAtParametricEnd();
      return false;
    }
    return this._path.isAtParametricEnd();
  }

  /** Follower.java:1117 getPathCompletion(): closest t of the current path, 0..1. */
  getPathCompletion() { return this._path ? this._path.closestT : 0; }
  getChainIndex() { return this._chainIndex; }
  getFollowingPathChain() { return this._followingChain; }
  getCurrentPath() { return this._path; }
  getClosestPose() { return { x: this._closest.pose.x, y: this._closest.pose.y }; }
  getPose() { return { ...this._pose }; }
  getVelocity() { return { ...this._vel }; }
  headingError() { return this._path ? this._errors.headingError() : 0; }
  driveError() { return this._errors.driveError(); }
  translationalError() { return this._errors._ctx ? this._errors.translationalError() : ZERO(); }
  /** Last powers written to the drivetrain, [LF, LR, RF, RR]. */
  powers() { return this._dt.powers; }

  /** Snapshot for the debug overlay. */
  telemetry() {
    let mode = 'idle';
    if (this._holding) mode = this._isTurning ? 'turn' : 'hold';
    else if (this._isBusy) mode = 'follow';
    return {
      mode,
      chainIndex: this._chainIndex,
      tValue: this.getPathCompletion(),
      stuck: this.isRobotStuck(),
      driveError: this._errors._driveError ?? 0,
      headingError: this._path && this._errors._ctx ? this._errors._headingError : 0,
      transError: this._errors._ctx ? this._errors.translationalError() : ZERO(),
      vectors: this._vectors.last,
      powers: this.powers(),
      reachedEnd: this._reachedEnd,
    };
  }
}
