// Robot spec loading, validation and derivation. Pure (no DOM, no three.js).
// Footprint note: collision.js treats the robot as a rectangle (halfX/halfY). A non-rectangular footprint
// is approximated by its bounding box for collisions (deriveConstants returns that box); the true polygon is
// still available as FOOTPRINT for drawing and for footprintCorners().

const TAGS = ['cad', 'repo', 'vault', 'placeholder'];   // vault = team design notes (MODEL, no CAD yet)
const FTC_LIMIT = 18;           // in, FTC sizing rule: 18 x 18 x 18

export async function loadRobotSpec(url = 'data/robot_spec.json') {
  const r = await fetch(url);
  if (!r.ok) throw new Error(`robot spec ${url}: HTTP ${r.status}`);
  return r.json();
}

const fin = v => typeof v === 'number' && Number.isFinite(v);

/** Normalise a polygon: drop a duplicated closing vertex. */
function openRing(poly) {
  const p = poly.map(q => [q[0], q[1]]);
  if (p.length > 1) {
    const a = p[0], b = p[p.length - 1];
    if (a[0] === b[0] && a[1] === b[1]) p.pop();
  }
  return p;
}

function segCross(a, b, c, d) {
  const o = (p, q, r) => (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0]);
  const d1 = o(a, b, c), d2 = o(a, b, d), d3 = o(c, d, a), d4 = o(c, d, b);
  return d1 * d2 < 0 && d3 * d4 < 0;
}

function polyArea(p) {
  let s = 0;
  for (let i = 0; i < p.length; i++) { const a = p[i], b = p[(i + 1) % p.length]; s += a[0] * b[1] - b[0] * a[1]; }
  return s / 2;
}

function bbox(p) {
  let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
  for (const q of p) { x0 = Math.min(x0, q[0]); x1 = Math.max(x1, q[0]); y0 = Math.min(y0, q[1]); y1 = Math.max(y1, q[1]); }
  return { x0, x1, y0, y1 };
}

export function validateSpec(spec) {
  const errors = [], warnings = [];
  const err = m => errors.push(m), warn = m => warnings.push(m);
  if (!spec || typeof spec !== 'object') return { ok: false, errors: ['spec is not an object'], warnings };
  if (spec.schema_version !== 1) err(`schema_version must be 1 (got ${spec.schema_version})`);
  if (spec.units !== 'in') err(`units must be 'in' (got ${spec.units})`);
  if (typeof spec.frame !== 'string' || !spec.frame) warn('spec.frame (coordinate-frame description) is missing');

  // every number must be finite
  (function walk(o, path) {
    if (typeof o === 'number') { if (!Number.isFinite(o)) err(`${path} is not a finite number`); return; }
    if (Array.isArray(o)) { o.forEach((v, i) => walk(v, `${path}[${i}]`)); return; }
    if (o && typeof o === 'object') for (const k of Object.keys(o)) walk(o[k], path ? `${path}.${k}` : k);
  })(spec, '');

  const need = (obj, key, path) => {
    if (!obj || !fin(obj[key])) { err(`${path}.${key} missing or not a finite number`); return false; }
    return true;
  };
  const pos = (obj, key, path) => { if (need(obj, key, path) && !(obj[key] > 0)) err(`${path}.${key} must be > 0 (got ${obj[key]})`); };

  // chassis
  const ch = spec.chassis;
  if (!ch) err('chassis missing');
  else {
    const fp = ch.footprint;
    if (!Array.isArray(fp) || fp.some(q => !Array.isArray(q) || q.length !== 2 || !fin(q[0]) || !fin(q[1]))) err('chassis.footprint must be an array of finite [x,y] points');
    else {
      const p = openRing(fp);
      if (p.length < 3) err('chassis.footprint is open/degenerate: needs at least 3 distinct vertices');
      else {
        let dup = false;
        for (let i = 0; i < p.length; i++) { const a = p[i], b = p[(i + 1) % p.length]; if (a[0] === b[0] && a[1] === b[1]) dup = true; }
        if (dup) err('chassis.footprint has duplicate consecutive vertices');
        let cross = false;
        for (let i = 0; i < p.length && !cross; i++) for (let j = i + 2; j < p.length; j++) {
          if (i === 0 && j === p.length - 1) continue;
          if (segCross(p[i], p[(i + 1) % p.length], p[j], p[(j + 1) % p.length])) { cross = true; break; }
        }
        if (cross) err('chassis.footprint is self-intersecting (not a simple polygon)');
        if (Math.abs(polyArea(p)) < 1e-6) err('chassis.footprint has zero area');
        const b = bbox(p);
        if (b.x1 - b.x0 > FTC_LIMIT + 1e-6 || b.y1 - b.y0 > FTC_LIMIT + 1e-6) warn(`footprint ${(b.x1 - b.x0).toFixed(2)} x ${(b.y1 - b.y0).toFixed(2)} in exceeds the FTC ${FTC_LIMIT} x ${FTC_LIMIT} in sizing rule`);
      }
    }
    pos(ch, 'height', 'chassis');
    if (fin(ch.height) && ch.height > FTC_LIMIT + 1e-6) warn(`chassis.height ${ch.height} in exceeds the FTC ${FTC_LIMIT} in sizing rule`);
    pos(ch, 'mass_lb', 'chassis');
  }

  // wheels
  if (!Array.isArray(spec.wheels) || spec.wheels.length !== 4) err('wheels must list exactly 4 wheels');
  else spec.wheels.forEach((w, i) => {
    const pth = `wheels[${i}]`;
    need(w, 'x', pth); need(w, 'y', pth); pos(w, 'radius', pth); pos(w, 'width', pth);
    if (fin(w.radius) && (w.radius < 0.5 || w.radius > 4)) err(`${pth}.radius ${w.radius} in is not a sane drive-wheel radius (0.5 to 4)`);
    if (!['mecanum', 'omni', 'traction'].includes(w.type)) err(`${pth}.type must be mecanum|omni|traction`);
  });

  // shooter
  const sh = spec.shooter;
  if (!sh) err('shooter missing');
  else {
    for (const a of ['x', 'y', 'z']) { need(sh.pivot, a, 'shooter.pivot'); need(sh.exitPoint, a, 'shooter.exitPoint'); }
    pos(sh, 'exitHeightAboveFloor', 'shooter');
    pos(sh, 'flywheelRadius', 'shooter');
    if (fin(sh.flywheelRadius) && (sh.flywheelRadius < 0.5 || sh.flywheelRadius > 4)) err(`shooter.flywheelRadius ${sh.flywheelRadius} in is not sane (0.5 to 4)`);
    pos(sh, 'maxRpm', 'shooter');
    if (need(sh, 'efficiency', 'shooter') && !(sh.efficiency > 0 && sh.efficiency <= 1)) err('shooter.efficiency must be in (0, 1]');
    pos(sh, 'rpmScale', 'shooter');
    if (sh.hood) {
      if (need(sh.hood, 'minDeg', 'shooter.hood') & need(sh.hood, 'maxDeg', 'shooter.hood') && !(sh.hood.minDeg < sh.hood.maxDeg)) err(`shooter.hood.minDeg (${sh.hood.minDeg}) must be < maxDeg (${sh.hood.maxDeg})`);
    } else err('shooter.hood missing');
    if (sh.turret) {
      if (need(sh.turret, 'minDeg', 'shooter.turret') & need(sh.turret, 'maxDeg', 'shooter.turret') && !(sh.turret.minDeg < sh.turret.maxDeg)) err('shooter.turret.minDeg must be < maxDeg');
    } else err('shooter.turret missing');
    if (sh.pivot && sh.exitPoint && fin(sh.pivot.z) && fin(sh.exitPoint.z) && fin(sh.exitHeightAboveFloor)
        && Math.abs(sh.pivot.z + sh.exitPoint.z - sh.exitHeightAboveFloor) > 0.5)
      warn(`shooter.exitHeightAboveFloor (${sh.exitHeightAboveFloor}) differs from pivot.z + exitPoint.z (${sh.pivot.z + sh.exitPoint.z})`);
  }

  // camera, drive
  if (!spec.camera) err('camera missing');
  else {
    for (const k of ['x', 'y', 'z', 'pitchDeg']) need(spec.camera, k, 'camera');
    for (const k of ['hfovDeg', 'vfovDeg', 'rangeIn']) pos(spec.camera, k, 'camera');
  }
  if (!spec.drive) err('drive missing');
  else for (const k of ['maxSpeedInS', 'maxAccel', 'maxTurnRadS']) pos(spec.drive, k, 'drive');

  if (!Array.isArray(spec.sensors)) err('sensors must be an array');
  else for (const n of ['intakeColor', 'shooterColor']) if (!spec.sensors.some(s => s.name === n)) warn(`sensor ${n} not listed`);
  if (!Array.isArray(spec.cad_todo)) warn('cad_todo list missing');
  return { ok: errors.length === 0, errors, warnings };
}

/** Constants in the exact shapes app.js uses today. Camera x/y/z are in the robot frame (pivot + relative offset). */
export function deriveConstants(spec) {
  const sh = spec.shooter, cam = spec.camera, dr = spec.drive;
  const fp = openRing(spec.chassis.footprint);
  const b = bbox(fp);
  const SHOOTER = {
    exitHeight: sh.exitHeightAboveFloor, wheelR: sh.flywheelRadius, eff: sh.efficiency, rpmScale: sh.rpmScale, maxRpm: sh.maxRpm,
    entryDeg: sh.entryDeg, fixedElevDeg: sh.fixedElevDeg, hoodMin: sh.hood.minDeg, hoodMax: sh.hood.maxDeg,
    pivotX: sh.pivot.x, pivotY: sh.pivot.y, turretMinDeg: sh.turret.minDeg, turretMaxDeg: sh.turret.maxDeg,
  };
  const PLANT = { turretRateDegS: sh.turret.rateDegS, turretKp: sh.turret.kp, flywheelTauS: sh.flywheelTauS, rpmDipFrac: sh.rpmDipFrac };
  const CAMERA = {
    x: sh.pivot.x + cam.x, y: sh.pivot.y + cam.y, z: sh.pivot.z + cam.z,
    pitchDeg: cam.pitchDeg, hfovDeg: cam.hfovDeg, vfovDeg: cam.vfovDeg, rangeIn: cam.rangeIn,
  };
  const DRIVE = { maxSpeed: dr.maxSpeedInS, maxAccel: dr.maxAccel, maxTurn: dr.maxTurnRadS };
  // collision.js is centred on the robot origin, so use the largest extent on each side
  return {
    SHOOTER, PLANT, CAMERA, DRIVE, FOOTPRINT: fp,
    halfX: Math.max(Math.abs(b.x0), Math.abs(b.x1)), halfY: Math.max(Math.abs(b.y0), Math.abs(b.y1)),
  };
}

/** World-frame corners of the footprint polygon for pose (x, y, heading h). */
export function footprintCorners(polygon, x, y, h) {
  const c = Math.cos(h), s = Math.sin(h);
  return openRing(polygon).map(q => [x + c * q[0] - s * q[1], y + s * q[0] + c * q[1]]);
}

/** Same output (order and values) as collision.js robotCorners(x,y,h,halfX,halfY): the bounding-box approximation. */
export function footprintBoxCorners(x, y, h, halfX, halfY) {
  return footprintCorners([[halfX, halfY], [halfX, -halfY], [-halfX, -halfY], [-halfX, halfY]], x, y, h);
}

/** How much of the robot is real: counts every numeric leaf by provenance tag. A footprint counts as one item. */
export function provenanceReport(spec) {
  const rep = { cad: 0, repo: 0, vault: 0, placeholder: 0, items: [] };
  const SKIP = new Set(['schema_version', 'prov', 'provenance_legend', 'cad_todo']);
  const add = (path, value, tag) => {
    const t = TAGS.includes(tag) ? tag : 'placeholder';
    rep[t]++; rep.items.push({ path, value, provenance: t });
  };
  // recursive worker with parent prov info
  function visit(v, path, parentProv, star, key) {
    const tag = parentProv?.[key] ?? star;
    if (typeof v === 'number') { add(path, v, tag); return; }
    if (Array.isArray(v)) {
      if (v.length && v.every(q => Array.isArray(q))) { add(path, v, tag); return; }   // polygon = one item
      v.forEach((e, i) => visit(e, `${path}[${i}]`, parentProv, star, key));
      return;
    }
    if (v && typeof v === 'object') {
      const prov = v.prov || {};
      const s = prov['*'] ?? parentProv?.[key] ?? star;
      for (const k of Object.keys(v)) { if (!SKIP.has(k)) visit(v[k], path ? `${path}.${k}` : k, prov, s, k); }
    }
  }
  visit(spec, '', null, 'placeholder', '');
  return rep;
}
