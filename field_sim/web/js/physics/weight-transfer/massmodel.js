// massmodel.js - centre of gravity and inertia of the robot (spec section 3).
// Units: in, lbm, lbm*in^2. Robot frame: +x forward, +y left, z up.

export function itemPositions(params, mech = {}) {
  const m = params.mass;
  const turretRad = mech.turretRad || 0;
  const dep = Math.min(1, Math.max(0, mech.intakeDeployed || 0));
  const c = Math.cos(turretRad), s = Math.sin(turretRad);
  const out = [];
  for (const it of m.items) {
    let x = it.x, y = it.y, z = it.z;
    if (it.frame === 'turret') {
      x = m.turretPivot.x + c * it.x - s * it.y;
      y = m.turretPivot.y + s * it.x + c * it.y;
      z = m.turretPivot.z + it.z;
    } else if (it.frame === 'intake') {
      x += dep * m.intakeTravelIn;
    }
    out.push({ name: it.name, massLb: it.massLb, x, y, z, box: { sx: it.box.sx, sy: it.box.sy, sz: it.box.sz } });
  }
  const nb = Math.max(0, Math.min(m.ballSlots.length, Math.round(mech.ballsHeld || 0)));
  for (let i = 0; i < nb; i++) {
    const sl = m.ballSlots[i];
    out.push({ name: 'ball' + i, massLb: m.ballMassLb, x: sl.x, y: sl.y, z: sl.z, box: { sx: 0, sy: 0, sz: 0 } });
  }
  return out;
}

export function computeCG(params, mech = {}) {
  const items = itemPositions(params, mech);
  let M = 0, sx = 0, sy = 0, sz = 0;
  for (const i of items) { M += i.massLb; sx += i.massLb * i.x; sy += i.massLb * i.y; sz += i.massLb * i.z; }
  const x = sx / M, y = sy / M, z = sz / M;
  let Ixx = 0, Iyy = 0, Izz = 0;
  for (const i of items) {
    const dx = i.x - x, dy = i.y - y, dz = i.z - z, b = i.box, m = i.massLb;
    Ixx += m * (dy * dy + dz * dz) + (m * (b.sy * b.sy + b.sz * b.sz)) / 12;
    Iyy += m * (dx * dx + dz * dz) + (m * (b.sx * b.sx + b.sz * b.sz)) / 12;
    Izz += m * (dx * dx + dy * dy) + (m * (b.sx * b.sx + b.sy * b.sy)) / 12;
  }
  return { massLb: M, x, y, z, Ixx, Iyy, Izz };
}
