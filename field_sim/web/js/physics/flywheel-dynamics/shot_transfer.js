// shot_transfer.js - ball/flywheel momentum transfer for a single-wheel shooter with a fixed hood.
// Pure ES module, no DOM, no dependencies. SI units (rad/s, kg, m, m/s, kg*m^2, N*s, N*m*s, J, s).
//
/**
 * DERIVATION (label: DERIVED; first-principles algebra, not measured. No FTC/FRC dip data was found.)
 *
 * Setup: one flywheel of radius R [m], inertia J [kg*m^2], speed w [rad/s]. The ball (mass m [kg], radius r_b [m],
 * moment of inertia I_b = k*m*r_b^2, k = shellK) is squeezed between the wheel surface (speed w*R) and a fixed hood.
 * Ball exit speed (empirical efficiency, ASSUMED/legacy exitEta from app.js):
 *     v = etaEff * w_ref * R,   etaEff = exitEta * ball.etaScale
 * The ball leaves rolling on the hood, so its spin is  w_b = s * v / r_b  (s = shot.spinRatio, DERIVED s = 1).
 *
 * Tangential impulses on the ball: P_w from the wheel, P_h from the hood (same sign convention, along the exit direction).
 *   Linear momentum:            m * v              = P_w + P_h
 *   Angular momentum about the
 *   ball centre:                k*m*r_b^2 * w_b    = r_b * (P_w - P_h)
 *   With w_b = s*v/r_b:         k*m*r_b*s*v        = r_b*(P_w - P_h)   =>  P_w - P_h = k*s*m*v
 *   Add and halve:              P_w = m * v * (1 + k*s) / 2
 *
 * Wheel angular momentum: the wheel feels the reaction impulse P_w at radius R, so
 *     J * (w0 - w1) = R * P_w
 * Define c = m*etaEff*R^2*(1 + k*s)/2 [kg*m^2] and beta = c/J.
 *   'mid'   (default) w_ref = (w0+w1)/2 :  J(w0-w1) = c*(w0+w1)/2  =>  w1 = w0*(1 - beta/2)/(1 + beta/2)
 *   'end'             w_ref = w1        :  J(w0-w1) = c*w1        =>  w1 = w0/(1 + beta)
 *   'start' (legacy)  w_ref = w0        :  J(w0-w1) = c*w0        =>  w1 = max(0, w0*(1 - beta))
 * Energy: E_wheel = 0.5*J*(w0^2 - w1^2); E_ball = 0.5*m*v^2*(1 + k*s^2) (translation + spin);
 * slipLoss = E_wheel - E_ball (>= 0; the rest is lost to slip at the contacts).
 *
 * Contact time (ASSUMPTION): ball travels contactPathM at average speed v/2, so t_c = 2*path/v, clamped.
 * Inverse (exact, linear in w0 since beta does not depend on w):
 *   'mid': w0 = v*(1+beta/2)/(etaEff*R);  'end': w0 = v*(1+beta)/(etaEff*R);  'start': w0 = v/(etaEff*R).
 * Note for 'start' with beta > 1 the clamp w1 = 0 breaks the impulse identity; P_w is then reported as J*(w0-w1)/R.
 */

const DEFAULT_SHELL_K = 2 / 3; // thin hollow shell (ASSUMPTION; solid sphere would be 0.4)

function fin(x) { return typeof x === 'number' && Number.isFinite(x); }

/**
 * Resolve a ball descriptor to a plain ball object.
 * @param {string|object|null|undefined} desc ball kind name, partial ball object, or nothing (default ball)
 * @param {object} shot params.shot ({balls, defaultBall, ...})
 * @returns {{massKg:number, radiusM:number, shellK:number, etaScale:number}} fresh copy (kg, m, -, -)
 */
export function makeBall(desc, shot) {
  const balls = (shot && shot.balls) || {};
  const def = balls[shot && shot.defaultBall] || balls.pollen || { massKg: 0.025, radiusM: 1.391 * 0.0254 };
  let base = def;
  let over = null;
  if (typeof desc === 'string') {
    if (Object.prototype.hasOwnProperty.call(balls, desc)) base = balls[desc];
  } else if (desc && typeof desc === 'object') {
    over = desc;
  }
  const b = over ? { ...base, ...over } : { ...base };
  return {
    massKg: b.massKg,
    radiusM: b.radiusM,
    shellK: fin(b.shellK) ? b.shellK : DEFAULT_SHELL_K,
    etaScale: fin(b.etaScale) ? b.etaScale : 1,
  };
}

function etaEff(ball, shot) { return shot.exitEta * ball.etaScale; }
function spinTerm(ball, shot) { return 1 + ball.shellK * shot.spinRatio; }

/**
 * Impulse coefficient c = m*etaEff*R^2*(1 + k*s)/2 (DERIVED).
 * @param {object} ball resolved ball
 * @param {object} shot params.shot
 * @param {number} wheelRadiusM wheel radius R [m]
 * @returns {number} c [kg*m^2]
 */
export function impulseCoefficientKgM2(ball, shot, wheelRadiusM) {
  return ball.massKg * etaEff(ball, shot) * wheelRadiusM * wheelRadiusM * spinTerm(ball, shot) / 2;
}

/**
 * Ball-wheel contact time t_c = clamp(2*contactPathM/max(v, 0.1), minContactS, maxContactS) (ASSUMPTION path).
 * @param {number} exitSpeedMps ball exit speed [m/s]
 * @param {object} shot params.shot
 * @returns {number} seconds
 */
export function contactTimeS(exitSpeedMps, shot) {
  const v = fin(exitSpeedMps) ? exitSpeedMps : 0;
  const t = 2 * shot.contactPathM / Math.max(v, 0.1);
  return Math.min(shot.maxContactS, Math.max(shot.minContactS, t));
}

/**
 * Predict the shot outcome (closed form, no integration).
 * @param {number} omega0RadS wheel speed before contact [rad/s]
 * @param {number} jKgM2 total wheel-shaft inertia [kg*m^2]
 * @param {object} ball resolved ball (see makeBall)
 * @param {object} shot params.shot
 * @param {number} wheelRadiusM wheel radius [m]
 * @returns {{omega0RadS:number, omega1RadS:number, dipFrac:number, beta:number, exitSpeedMps:number,
 *   wheelImpulseNs:number, wheelAngularImpulseNms:number, energyFromWheelJ:number, ballKineticJ:number,
 *   slipLossJ:number, energyRatio:number, contactTimeS:number}}
 */
export function predictShot(omega0RadS, jKgM2, ball, shot, wheelRadiusM) {
  const valid = fin(omega0RadS) && omega0RadS > 0 && fin(jKgM2) && jKgM2 > 0 && fin(wheelRadiusM) && wheelRadiusM > 0;
  const c = impulseCoefficientKgM2(ball, shot, wheelRadiusM);
  const beta = fin(c) && fin(jKgM2) && jKgM2 > 0 ? c / jKgM2 : 0;
  if (!valid) {
    return {
      omega0RadS: fin(omega0RadS) ? omega0RadS : 0, omega1RadS: 0, dipFrac: 0, beta,
      exitSpeedMps: 0, wheelImpulseNs: 0, wheelAngularImpulseNms: 0,
      energyFromWheelJ: 0, ballKineticJ: 0, slipLossJ: 0, energyRatio: Infinity,
      contactTimeS: shot.maxContactS,
    };
  }
  const eta = etaEff(ball, shot);
  const R = wheelRadiusM;
  const w0 = omega0RadS;
  let w1;
  let v;
  const ref = shot.exitSpeedRef;
  if (ref === 'end') {
    w1 = w0 / (1 + beta);
    v = eta * R * w1;
  } else if (ref === 'start') {
    w1 = Math.max(0, w0 * (1 - beta));
    v = eta * R * w0;
  } else {
    w1 = w0 * (1 - beta / 2) / (1 + beta / 2);
    v = eta * R * (w0 + w1) / 2;
  }
  const angImpulse = jKgM2 * (w0 - w1);
  const Pw = angImpulse / R;
  const eWheel = 0.5 * jKgM2 * (w0 * w0 - w1 * w1);
  const eBall = 0.5 * ball.massKg * v * v * (1 + ball.shellK * shot.spinRatio * shot.spinRatio);
  return {
    omega0RadS: w0,
    omega1RadS: w1,
    dipFrac: 1 - w1 / w0,
    beta,
    exitSpeedMps: v,
    wheelImpulseNs: Pw,
    wheelAngularImpulseNms: angImpulse,
    energyFromWheelJ: eWheel,
    ballKineticJ: eBall,
    slipLossJ: eWheel - eBall,
    energyRatio: eBall > 0 ? 0.5 * jKgM2 * w0 * w0 / eBall : Infinity,
    contactTimeS: contactTimeS(v, shot),
  };
}

/**
 * Wheel speed needed (before contact) to launch the ball at a given exit speed; exact inverse of predictShot.
 * @param {number} exitSpeedMps desired exit speed [m/s]
 * @param {number} jKgM2 wheel inertia [kg*m^2]
 * @param {object} ball resolved ball
 * @param {object} shot params.shot
 * @param {number} wheelRadiusM wheel radius [m]
 * @returns {number} omega0 [rad/s] (0 for non-positive or invalid input)
 */
export function omegaForExitSpeed(exitSpeedMps, jKgM2, ball, shot, wheelRadiusM) {
  if (!fin(exitSpeedMps) || exitSpeedMps <= 0 || !fin(jKgM2) || jKgM2 <= 0 || !fin(wheelRadiusM) || wheelRadiusM <= 0) return 0;
  const beta = impulseCoefficientKgM2(ball, shot, wheelRadiusM) / jKgM2;
  const base = exitSpeedMps / (etaEff(ball, shot) * wheelRadiusM);
  if (shot.exitSpeedRef === 'end') return base * (1 + beta);
  if (shot.exitSpeedRef === 'start') return base;
  return base * (1 + beta / 2);
}
