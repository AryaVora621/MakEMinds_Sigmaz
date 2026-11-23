// forces.js - per-wheel forces on the chassis (SI: N, N*m, m/s). Pure functions.
// Frame convention: body x = forward, y = left, omega = CCW+; wheels in spec order FL, FR, BL, BR.
// Forces: drive (along roller axle (1,sigma), traction-limited), roller loss (net body-lateral, see below),
// floor rolling resistance (opposes contact-point velocity). Moment about body origin: mz = sum(x*Fy - y*Fx).
import { contactVelocity } from './kinematics.js';
import { smoothSign } from './motor.js';

/** Static wheel loads (N), spec order. */
export function wheelLoads(p) {
  return p.normalForceN.slice();
}

/** Summed wrench plus per-wheel detail. torques = wheel-shaft torque (N*m) per wheel. */
export function wheelForces(p, twist, torques) {
  const eps = p.coulombEpsMps;
  const r = p.wheelRadiusM;
  let fx = 0;
  let fy = 0;
  let mz = 0;
  const perWheel = new Array(p.wheels.length);
  for (let i = 0; i < p.wheels.length; i++) {
    const w = p.wheels[i];
    const N = p.normalForceN[i];

    // Drive force along (1, sigma), clamped to the friction circle.
    const fm = torques[i] / r;
    let dx = fm;
    let dy = fm * w.sigma;
    const maxF = p.mu * p.gripScale * N;
    const mag = Math.hypot(dx, dy);
    let tractionLimited = false;
    if (mag > maxF) {
      tractionLimited = true;
      const s = mag > 0 ? maxF / mag : 0;
      dx *= s;
      dy *= s;
    }

    const c = contactVelocity(w, twist);

    // Roller loss. The roller surface slips relative to the HUB, not relative to the chassis:
    // no-slip along the roller axle a = (1,sigma)/n (n = sqrt(1+sigma^2)) fixes the rim speed
    // s = cx + sigma*cy, and the hub-relative contact velocity is c - s*xhat = (-sigma*cy, cy).
    // Its component along the free direction d = (-sigma,1)/n is the roller surface speed
    //   u = (c - s*xhat) . d = n*cy            (pure forward: u = 0; pure strafe: u = sqrt2*vy).
    // (Round-1 used u = c.d, which spins the rollers at vx/sqrt2 in pure forward - kinematically wrong.)
    // The loss force rl acts along d on the wheel; its hub-x component is reacted by the wheel shaft
    // (the total ground x-force is fixed at tau/r by the shaft torque balance), so the net extra force on
    // the chassis is rl*(d + sigma*a) = rl*(0, n): purely body-lateral. Power check: F.c = rl*n*cy = rl*u <= 0.
    const n = Math.sqrt(1 + w.sigma * w.sigma);
    const u = n * c.cy;
    const rl = -(p.crrRoller * N * smoothSign(u, eps) + p.rollerViscousNsPerM * u);
    const rx = 0;
    const ry = rl * n;

    // Floor rolling resistance opposing contact-point velocity.
    const cm = Math.hypot(c.cx, c.cy);
    let gx = 0;
    let gy = 0;
    if (cm >= 1e-12) {
      const k = -p.crrBase * N * (eps > 0 ? Math.tanh(cm / eps) : 1) / cm;
      gx = k * c.cx;
      gy = k * c.cy;
    }

    const wx = dx + rx + gx;
    const wy = dy + ry + gy;
    fx += wx;
    fy += wy;
    mz += w.x * wy - w.y * wx;
    perWheel[i] = {
      driveN: [dx, dy],
      rollerN: [rx, ry],
      rollingN: [gx, gy],
      rollerSpeedMps: u,
      tractionLimited,
    };
  }
  return { fx, fy, mz, perWheel };
}
