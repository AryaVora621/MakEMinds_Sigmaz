// Sequential-impulse wheel contact solver (spec section 7). Pure SI (m, s, kg, N, rad); no DOM.
// Body frame: +x forward, +y left, omega > 0 CCW. Wheel order FL, FR, BL, BR.
// Impulses are in N*s (force * substep h). Accumulators are reset on every call.

// Scratch accumulators (solver is synchronous, so module-level reuse is safe).
const ACC = new Float64Array(8);   // [wheel*2 + axis] constrained-axis impulse, N*s
const ACC_P = new Float64Array(4); // free-axis (roller) impulse, N*s

/** Telemetry container: slip (m/s), slipRatio (-), gripUsed (-), saturated (0/1), forceN (N, body frame). */
export function makeContactOut() {
  return {
    slip: new Float64Array(4),
    slipRatio: new Float64Array(4),
    gripUsed: new Float64Array(4),
    saturated: new Uint8Array(4),
    forceN: [[0, 0], [0, 0], [0, 0], [0, 0]],
  };
}

/**
 * Solve the four wheel contacts for one substep h (s). Mutates body {vx,vy,om} and wheelOmega (rad/s).
 * loads: normal loads N; muEff: effective friction coefficients (already Stribeck-selected and scaled).
 * wheelInertia (optional, kg*m^2 per wheel): response inertia of each wheel for this substep. The
 * drivetrain passes J + h*k from the implicit motor step (k = back-EMF + viscous damping) so that the
 * motor and the contact are solved jointly implicit; omitted -> bare w.inertiaKgM2.
 */
export function solveWheelContacts(body, wheelOmega, loads, muEff, cfg, h, out = makeContactOut(), wheelInertia = null) {
  const M = cfg.massKg, I = cfg.inertiaKgM2;
  const wheels = cfg.wheels;
  const iters = cfg.solver.iterations;
  ACC.fill(0);
  ACC_P.fill(0);

  let vx = body.vx, vy = body.vy, om = body.om;

  for (let it = 0; it < iters; it++) {
    for (let i = 0; i < 4; i++) {
      const N = loads[i];
      if (!(N > 0)) continue;
      const L = muEff[i] * N * h; // grip impulse limit, N*s
      if (!(L > 0)) continue;
      const w = wheels[i];
      const rx = w.pos[0], ry = w.pos[1], R = w.radiusM, J = wheelInertia ? wheelInertia[i] : w.inertiaKgM2;
      const axes = w.axes;
      const na = axes.length;
      const om_i = wheelOmega[i];

      // contact-point velocity of the chassis (m/s)
      const cvx = vx - om * ry, cvy = vy + om * rx;

      // tentative accumulated impulses for each constrained axis (from the same pre-update state)
      let t0 = 0, t1 = 0;
      for (let a = 0; a < na; a++) {
        const ax = axes[a];
        const dx = ax.dir[0], dy = ax.dir[1], c = ax.coupling;
        const s = cvx * dx + cvy * dy - c * R * om_i;
        const rxd = rx * dy - ry * dx;
        const k = 1 / M + (rxd * rxd) / I + (c * R) * (c * R) / J;
        const t = ACC[i * 2 + a] + (-s / k);
        if (a === 0) t0 = t; else t1 = t;
      }

      // clamp
      if (na === 1) {
        const lim = L * axes[0].muFactor;
        if (t0 > lim) t0 = lim; else if (t0 < -lim) t0 = -lim;
      } else {
        const la = L * axes[0].muFactor, lb = L * axes[1].muFactor;
        const q = (t0 / la) * (t0 / la) + (t1 / lb) * (t1 / lb);
        if (q > 1) { const f = 1 / Math.sqrt(q); t0 *= f; t1 *= f; }
      }

      // apply deltas
      for (let a = 0; a < na; a++) {
        const ax = axes[a];
        const dx = ax.dir[0], dy = ax.dir[1], c = ax.coupling;
        const delta = (a === 0 ? t0 : t1) - ACC[i * 2 + a];
        ACC[i * 2 + a] = a === 0 ? t0 : t1;
        if (delta === 0) continue;
        vx += delta * dx / M;
        vy += delta * dy / M;
        om += (rx * dy - ry * dx) * delta / I;
        wheelOmega[i] -= c * R * delta / J;
      }

      // free (roller) axis: chassis only, bounded by rolling resistance and the remaining friction circle
      const fd = w.freeDir;
      if (fd) {
        const pvx = vx - om * ry, pvy = vy + om * rx;
        const sp = pvx * fd[0] + pvy * fd[1];
        const rxp = rx * fd[1] - ry * fd[0];
        const kp = 1 / M + (rxp * rxp) / I;
        let tp = ACC_P[i] + (-sp / kp);
        const la = ACC[i * 2];
        const circ = Math.sqrt(Math.max(0, L * L - la * la));
        const lim = Math.min(w.crrFree * N * h, circ);
        if (tp > lim) tp = lim; else if (tp < -lim) tp = -lim;
        const delta = tp - ACC_P[i];
        ACC_P[i] = tp;
        vx += delta * fd[0] / M;
        vy += delta * fd[1] / M;
        om += rxp * delta / I;
      }
    }
  }

  body.vx = vx; body.vy = vy; body.om = om;

  // telemetry from the post-solve state
  const vMin = cfg.friction.slipRatioVMinMS;
  for (let i = 0; i < 4; i++) {
    const w = wheels[i];
    const ax0 = w.axes[0];
    const R = w.radiusM;
    const cvx = vx - om * w.pos[1], cvy = vy + om * w.pos[0];
    const vd = cvx * ax0.dir[0] + cvy * ax0.dir[1];
    const surf = ax0.coupling * R * wheelOmega[i];
    out.slip[i] = vd - surf;
    const den = Math.max(Math.abs(vd), Math.abs(surf), vMin);
    out.slipRatio[i] = (surf - vd) / den;

    const N = loads[i];
    const L = muEff[i] * N * h;
    let fx = 0, fy = 0, sq = 0;
    for (let a = 0; a < w.axes.length; a++) {
      const lam = ACC[i * 2 + a];
      fx += lam * w.axes[a].dir[0];
      fy += lam * w.axes[a].dir[1];
      sq += lam * lam;
    }
    if (w.freeDir) {
      const lp = ACC_P[i];
      fx += lp * w.freeDir[0];
      fy += lp * w.freeDir[1];
      sq += lp * lp;
    }
    const used = L > 0 ? Math.sqrt(sq) / L : 0;
    out.gripUsed[i] = used;
    out.saturated[i] = used >= 0.999 ? 1 : 0;
    out.forceN[i][0] = fx / h;
    out.forceN[i][1] = fy / h;
  }
  return out;
}
