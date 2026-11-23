/* Drag-free FTC shooter ball-trajectory solver (JS port of core.py).
 * Units: inches, seconds, radians. Pedro frame. See core.py for the derivations.
 * p = {x,y,psi,vx,vy,om,rx,ry,z0,X,Y,Z, allow_rising?}
 * modes: 'gamma' (rad), 'theta' (rad), 'speed' (in/s, branch 'low'|'high'), 'minspeed' */
(function (root) {
  'use strict';
  var G = 386.0886, IN_TO_M = 0.0254, PI = Math.PI;

  function wrap(a) { var m = (a + PI) % (2 * PI); if (m < 0) m += 2 * PI; return m - PI; }

  function kinematics(p) {
    var c = Math.cos(p.psi), s = Math.sin(p.psi);
    var rfx = c * p.rx - s * p.ry, rfy = s * p.rx + c * p.ry;
    var P = [p.x + rfx, p.y + rfy];
    var vs = [p.vx - p.om * rfy, p.vy + p.om * rfx];
    var D = [p.X - P[0], p.Y - P[1]];
    return { r_field: [rfx, rfy], P: P, vs: vs, Delta: D, d: Math.hypot(D[0], D[1]), h: p.Z - p.z0 };
  }

  function forward(P, z0, vs, uh, uz, X, Y, Z, rising) {
    var h = Z - z0, disc = uz * uz - 2 * G * h;
    if (disc < 0) return null;
    var t = rising ? (uz - Math.sqrt(disc)) / G : (uz + Math.sqrt(disc)) / G;
    var vgx = vs[0] + uh[0], vgy = vs[1] + uh[1];
    var x = P[0] + vgx * t, y = P[1] + vgy * t, vz = uz - G * t;
    return { t: t, x: x, y: y, z: z0 + uz * t - 0.5 * G * t * t, vz: vz,
             entry: Math.atan2(-vz, Math.hypot(vgx, vgy)), err: Math.hypot(x - X, y - Y) };
  }

  function trajectoryPoint(P, z0, vs, uh, uz, t) {
    return [P[0] + (vs[0] + uh[0]) * t, P[1] + (vs[1] + uh[1]) * t, z0 + uz * t - 0.5 * G * t * t];
  }

  function cbrt(x) { return Math.cbrt(x); }

  function cubicRealRoots(p, q) {
    if (Math.abs(p) < 1e-300) return [cbrt(-q)];
    var disc = Math.pow(q / 2, 2) + Math.pow(p / 3, 3);
    if (disc > 0) { var sq = Math.sqrt(disc); return [cbrt(-q / 2 + sq) + cbrt(-q / 2 - sq)]; }
    var r = 2 * Math.sqrt(-p / 3);
    var arg = Math.max(-1, Math.min(1, 3 * q / (2 * p) * Math.sqrt(-3 / p)));
    var phi = Math.acos(arg) / 3, out = [];
    for (var k = 0; k < 3; k++) out.push(r * Math.cos(phi - 2 * PI * k / 3));
    return out;
  }

  /* positive real roots of a T^4 + b T^2 + c T + e, a>0, ascending */
  function quarticPosRoots(a, b, c, e) {
    function f(T) { return (((a * T * T) + b) * T + c) * T + e; }
    function scaleAt(T) { return a * Math.pow(T, 4) + Math.abs(b) * T * T + Math.abs(c) * T + Math.abs(e); }
    var bound = 1 + Math.max(Math.abs(b), Math.abs(c), Math.abs(e)) / a;
    var crit = cubicRealRoots(b / (2 * a), c / (4 * a)).filter(function (t) { return t > 0 && t < bound; })
      .sort(function (u, v) { return u - v; });
    var pts = [0].concat(crit, [bound]), roots = [], i, it;
    for (i = 0; i < pts.length - 1; i++) {
      var lo = pts[i], hi = pts[i + 1], flo = f(lo), fhi = f(hi);
      if (flo === 0 && lo > 0) roots.push(lo);
      else if (flo * fhi < 0) {
        for (it = 0; it < 200; it++) {
          var mid = 0.5 * (lo + hi);
          if (mid <= lo || mid >= hi) break;
          var fm = f(mid);
          if (fm === 0) { lo = hi = mid; break; }
          if ((fm < 0) === (flo < 0)) { lo = mid; flo = fm; } else hi = mid;
        }
        roots.push(0.5 * (lo + hi));
      }
    }
    crit.forEach(function (t) {
      if (Math.abs(f(t)) <= 1e-12 * scaleAt(t) &&
          roots.every(function (r) { return Math.abs(t - r) > 1e-6 * Math.max(1, t); })) roots.push(t);
    });
    return roots.sort(function (u, v) { return u - v; });
  }

  function minSpeedT(D, vs, h) {
    var dv = D[0] * vs[0] + D[1] * vs[1], Dd = D[0] * D[0] + D[1] * D[1] + h * h;
    function q(T) { return 0.5 * G * G * Math.pow(T, 4) + 2 * dv * T - 2 * Dd; }
    var lo = 0, hi = 1, it;
    while (q(hi) <= 0) hi *= 2;
    for (it = 0; it < 200; it++) {
      var mid = 0.5 * (lo + hi);
      if (mid <= lo || mid >= hi) break;
      if (q(mid) < 0) lo = mid; else hi = mid;
    }
    return 0.5 * (lo + hi);
  }

  function stationaryAngles(v, d, h) {
    var disc = Math.pow(v, 4) - G * (G * d * d + 2 * h * v * v);
    if (disc < 0 || d <= 0) return null;
    var sq = Math.sqrt(disc);
    return [Math.atan2(v * v - sq, G * d), Math.atan2(v * v + sq, G * d)];
  }

  function stationaryTSpeed(s, d, h) {
    var A = s * s - G * h, disc = A * A - G * G * (d * d + h * h);
    if (disc < 0) return null;
    var sq = Math.sqrt(disc), w1 = 2 * (A - sq) / (G * G), w2 = 2 * (A + sq) / (G * G);
    if (w1 <= 0) return null;
    return [Math.sqrt(w1), Math.sqrt(w2)];
  }

  function finish(p, k, T, out) {
    var d = k.d, h = k.h, D = k.Delta, vs = k.vs;
    var uh = [D[0] / T - vs[0], D[1] / T - vs[1]];
    var uz = (h + 0.5 * G * T * T) / T;
    var uhm = Math.hypot(uh[0], uh[1]), speed = Math.hypot(uhm, uz), az = Math.atan2(uh[1], uh[0]);
    out.T = T; out.uh = uh; out.uz = uz; out.uh_mag = uhm; out.speed = speed; out.speed_ms = speed * IN_TO_M;
    out.elev = Math.atan2(uz, uhm); out.az = az; out.turret = wrap(az - p.psi);
    out.gamma = Math.atan2(0.5 * G * T * T - h, d);
    out.apex_h = p.z0 + (uz > 0 ? uz * uz / (2 * G) : 0);
    out.apex_t = Math.max(uz, 0) / G;
    out.Gv = [p.X - vs[0] * T, p.Y - vs[1] * T];
    out.rising = (uz - G * T) > 0;
    out.fwd = forward(k.P, p.z0, vs, uh, uz, p.X, p.Y, p.Z, out.rising);
    out.ok = true;
    return out;
  }

  function solve(p, mode, param, branch) {
    branch = branch || 'low';
    var k = kinematics(p), out = {}, key;
    for (key in k) out[key] = k[key];
    out.mode = mode; out.ok = false; out.reason = ''; out.roots = [];
    var d = k.d, h = k.h, D = k.Delta, vs = k.vs;
    var vs2 = vs[0] * vs[0] + vs[1] * vs[1], Dv = D[0] * vs[0] + D[1] * vs[1], d2 = d * d, cand = [];
    if (mode === 'gamma') {
      if (!(param > 0 && param < PI / 2)) { out.reason = 'entry angle must be in (0, 90) deg'; return out; }
      if (d < 1e-9) { out.reason = 'unreachable: target is directly above/below the exit point (d = 0), entry angle cannot be chosen'; return out; } // Opus fix
      var w = 2 * (h + d * Math.tan(param)) / G;
      if (w <= 0) { out.reason = 'unreachable: h + d*tan(gamma) <= 0 (target too far below for this entry angle)'; return out; }
      cand = [Math.sqrt(w)];
    } else if (mode === 'theta') {
      if (!(param > 0 && param < PI / 2)) { out.reason = 'hood angle must be in (0, 90) deg'; return out; }
      var t2 = Math.pow(Math.tan(param), 2);
      var rts = quarticPosRoots(G * G / 4, G * h - t2 * vs2, 2 * t2 * Dv, h * h - t2 * d2);
      rts.forEach(function (T) { if (h + 0.5 * G * T * T > 0) cand.push(T); });
      if (!cand.length) {
        out.reason = 'unreachable: no flight time for this fixed hood angle (d*tan(theta) <= h when stationary, or shooter-velocity too large)';
        return out;
      }
    } else if (mode === 'speed') {
      cand = quarticPosRoots(G * G / 4, vs2 + G * h - param * param, -2 * Dv, d2 + h * h);
      if (!cand.length) {
        var Tm = minSpeedT(D, vs, h);
        var uhm = [D[0] / Tm - vs[0], D[1] / Tm - vs[1]];
        var smin = Math.hypot(Math.hypot(uhm[0], uhm[1]), (h + 0.5 * G * Tm * Tm) / Tm);
        out.min_speed = smin;
        out.reason = 'unreachable: launch speed ' + param.toFixed(2) + ' in/s is below the minimum ' + smin.toFixed(2) + ' in/s';
        return out;
      }
    } else if (mode === 'minspeed') {
      cand = [minSpeedT(D, vs, h)];
    } else throw new Error('bad mode ' + mode);
    var nBefore = cand.length;
    if (!p.allow_rising) cand = cand.filter(function (T) { return 0.5 * G * T * T > h; });
    out.roots = cand;
    if (!cand.length) {
      out.reason = 'no descending-arrival solution (all solutions hit the target height while still rising; enable "allow rising arrival")';
      return out;
    }
    // Opus fix: tell the user when the requested arc does not exist and the other one is shown
    out.branch_note = '';
    if ((mode === 'speed' || mode === 'theta') && cand.length === 1) {
      if (nBefore > 1) out.branch_note = 'only one arc arrives descending (the other root hits the target height while still rising); showing it';
      else if (mode === 'speed') out.branch_note = 'only one arc (double root: speed equals the minimum speed)';
    }
    var T = (branch === 'low' || cand.length === 1) ? cand[0] : cand[cand.length - 1];
    return finish(p, k, T, out);
  }

  function rpmEstimate(speed, Rw, eta) {
    Rw = Rw || 2; eta = eta || 0.45;
    return 60 * speed / (2 * PI * Rw * eta);
  }

  /* exact inverse of rpmEstimate: exit speed (in/s) of a ball for a CURRENT flywheel rpm (rpm/scale -> wheel surface speed * eta) */
  function speedFromRpm(rpm, Rw, eta, scale) {
    Rw = Rw || 2; eta = eta || 0.45;
    return Math.max(0, rpm || 0) / (scale || 1) * 2 * PI * Rw * eta / 60;
  }

  /* Ball exit speed for a ball of a given mass: the wheel-surface speed (speedFromRpm) times massScale. massScale is the
   * kind factor from balls.js launchScaleFor (1 = reference pollen). ASSUMPTION: equal kinetic energy per ball, so 1/sqrt(mass). */
  function launchSpeed(rpm, Rw, eta, scale, massScale) {
    return speedFromRpm(rpm, Rw, eta, scale) * (massScale == null ? 1 : massScale);
  }
  /* Inverse of launchSpeed: the flywheel rpm that gives a ball of massScale the exit speed sp (exact inverse of launchSpeed). */
  function rpmForSpeed(sp, Rw, eta, scale, massScale) {
    return rpmEstimate(sp / (massScale == null ? 1 : massScale), Rw, eta) * (scale || 1);
  }

  /* Launch state from the ACTUAL shooter state: no solver, no readiness check. azRad = absolute azimuth (heading + turret),
   * elevRad = hood elevation. Robot velocity (vs = [vx,vy] at the pivot) is added. Low rpm just gives a slow, short ball.
   * massScale (optional, default 1) scales the exit speed for the ball kind (see launchSpeed). */
  function launchVelocity(rpm, azRad, elevRad, Rw, eta, scale, vs, massScale) {
    var sp = launchSpeed(rpm, Rw, eta, scale, massScale), ce = Math.cos(elevRad);
    return { speed: sp, vel: [(vs ? vs[0] : 0) + Math.cos(azRad) * ce * sp, (vs ? vs[1] : 0) + Math.sin(azRad) * ce * sp, Math.sin(elevRad) * sp] };
  }

  /* ---- shooter-exit additions (opt-in; existing functions above are untouched) ---- */

  /* Ball velocity from speed/azimuth/elevation plus an optional base velocity vBase [3] (robot + rotation terms). */
  function launchVelocityFrom(speed, azRad, elevRad, vBase) {
    var ce = Math.cos(elevRad);
    return [(vBase ? vBase[0] : 0) + Math.cos(azRad) * ce * speed,
            (vBase ? vBase[1] : 0) + Math.sin(azRad) * ce * speed,
            (vBase ? vBase[2] : 0) + Math.sin(elevRad) * speed];
  }

  /* Solve with a non-zero exit point. exitFn(elevRad, azAbsRad) -> {dx,dy,dz} = exit minus pivot, field-frame inches.
   * The offset is folded into p.rx/p.ry (robot frame, via R(-psi)) and p.z0, so kinematics() adds om x r correctly.
   * Fixed-point iteration: the offset depends on the solved (elev, az). Stops when the offset moves < 1e-4 in. */
  function solveExit(p, mode, param, branch, exitFn, iters) {
    iters = iters == null ? 4 : iters;
    var c = Math.cos(p.psi), s = Math.sin(p.psi);
    var px = p.x + c * p.rx - s * p.ry, py = p.y + s * p.rx + c * p.ry;
    var off = exitFn(PI / 4, Math.atan2(p.Y - py, p.X - px));
    var r = null, pe = p, used = off, i, n = Math.max(1, iters);
    for (i = 1; i <= n; i++) {
      pe = {};
      for (var k in p) pe[k] = p[k];
      pe.rx = p.rx + (c * off.dx + s * off.dy);
      pe.ry = p.ry + (-s * off.dx + c * off.dy);
      pe.z0 = p.z0 + off.dz; used = off;
      r = solve(pe, mode, param, branch);
      if (!r.ok) break;
      var nxt = exitFn(r.elev, r.az);
      var delta = Math.hypot(nxt.dx - off.dx, nxt.dy - off.dy, nxt.dz - off.dz);
      if (delta < 1e-4) break;
      off = nxt;
    }
    r.exitOffset = { dx: used.dx, dy: used.dy, dz: used.dz };
    r.iters = Math.min(i, n);
    r.p = pe;
    return r;
  }

  /* First front-to-back crossing of the ballistic path P0 + v t - 0.5 G t^2 zhat with plane {x,y,z,normal}.
   * Solves -0.5 G nz t^2 + (n.v) t + n.(P0 - X) = 0 for the smallest t > 1e-9 with ds/dt = n.v - G nz t < 0. */
  function planeCrossing(P0, vel, plane) {
    var n = plane.normal, A = -0.5 * G * n[2];
    var B = n[0] * vel[0] + n[1] * vel[1] + n[2] * vel[2];
    var C = n[0] * (P0[0] - plane.x) + n[1] * (P0[1] - plane.y) + n[2] * (P0[2] - plane.z);
    var roots = [];
    if (Math.abs(A) < 1e-12) { if (Math.abs(B) > 1e-300) roots.push(-C / B); }
    else {
      var disc = B * B - 4 * A * C;
      if (disc < 0) return null;
      var sq = Math.sqrt(disc), q = -0.5 * (B + (B >= 0 ? sq : -sq));
      if (q !== 0) roots.push(q / A, C / q); else roots.push(0);
      roots.sort(function (u, v) { return u - v; });
    }
    for (var i = 0; i < roots.length; i++) {
      var t = roots[i];
      if (!(t > 1e-9)) continue;
      if (B + 2 * A * t < 0) {
        return { t: t, p: [P0[0] + vel[0] * t, P0[1] + vel[1] * t, P0[2] + vel[2] * t - 0.5 * G * t * t],
                 v: [vel[0], vel[1], vel[2] - G * t] };
      }
    }
    return null;
  }

  var Core = { G: G, speedFromRpm: speedFromRpm, launchVelocity: launchVelocity, launchSpeed: launchSpeed, rpmForSpeed: rpmForSpeed,
    IN_TO_M: IN_TO_M, wrap: wrap, kinematics: kinematics, forward: forward,
    trajectoryPoint: trajectoryPoint, quarticPosRoots: quarticPosRoots, minSpeedT: minSpeedT,
    stationaryAngles: stationaryAngles, stationaryTSpeed: stationaryTSpeed, solve: solve, rpmEstimate: rpmEstimate,
    solveExit: solveExit, planeCrossing: planeCrossing, launchVelocityFrom: launchVelocityFrom };
  if (typeof module !== 'undefined' && module.exports) module.exports = Core; else root.Core = Core;
})(typeof window !== 'undefined' ? window : this);
