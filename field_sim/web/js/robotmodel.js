// Builds the whole robot from a robot spec. three.js is passed in by the host (so node tests can use the vendored
// module): buildRobot(THREE, spec, opts?). Robot frame: +x forward, +y left, z up, origin on the floor at the robot centre.
// Hood convention (same as app.js): hoodPivot.rotation.y = -rad(hoodDeg), +x = launch direction.
// opts.allianceColor ('red' | 'blue' | 0xRRGGBB) or spec.alliance adds a back plate in that colour (default: none).
// opts.flowerArmCfg is the parsed data/flower_arm.json (default: the geometry mirrored in flowerarm3d.js).
//
// This is the PROCEDURAL robot, modelled on the team's MaxBot design notes: two drive cassettes (4 x 104 mm mecanum wheels
// with rollers, belt-driven), a spine with the ball lane on the centreline, full-width FRONT and REAR intakes (they fold to
// 38 mm at the start and open to 106 mm in play, each with flex rollers, mouth gate and hook fingers), the transfer cartridge,
// a deck at z 224 mm with a turret ring (105 mm bore, pinion drive), a head with ONE 96 mm flywheel above the ball and an
// adjustable hood, a drop-down flower arm, odometry pods and electronics. Every dimension comes from robot_spec.json.
// For the real CAD see robotglb.js.
//
// Animation: setPose({...}) sets absolute values, animate(dt, state) integrates spinning parts (wheels from robot-frame
// velocities, intake rollers, flywheel from rpm, intake fold, flower arm angle). See the bottom of this file.

import { buildFlowerArm, DEFAULT_ARM_CFG } from './flowerarm3d.js';

const rad = d => d * Math.PI / 180;
const PI = Math.PI;

/** Pure numbers for the shooter: exit point in the robot frame, launch direction, exit height. */
export function shooterGeometry(spec, turretDeg = 0, hoodDeg = null) {
  const sh = spec.shooter;
  const hood = hoodDeg ?? (sh.hood.minDeg + sh.hood.maxDeg) / 2;
  const t = rad(turretDeg), e = rad(hood), c = Math.cos(t), s = Math.sin(t);
  const ex = sh.exitPoint, above = sh.flywheelAbove ?? 0;
  const exitWorld = [sh.pivot.x + c * ex.x - s * ex.y, sh.pivot.y + s * ex.x + c * ex.y, sh.pivot.z + ex.z];
  return {
    pivot: [sh.pivot.x, sh.pivot.y, sh.pivot.z],
    exitLocal: [ex.x, ex.y, ex.z],
    exitPointRobot: exitWorld,                      // robot frame, for the given turret angle (ball release point)
    exitHeight: exitWorld[2],
    launchDir: [c * Math.cos(e), s * Math.cos(e), Math.sin(e)],
    hoodDeg: hood, turretDeg,
    flywheelRadius: sh.flywheelRadius,
    flywheelCenterRobot: [exitWorld[0], exitWorld[1], exitWorld[2] + above],   // the wheel sits above the ball
  };
}

// Collects transformed geometries and merges them into one BufferGeometry (one draw call per material).
function makeBatch(THREE) {
  const pos = [], nor = [];
  const m = new THREE.Matrix4(), q = new THREE.Quaternion(), e = new THREE.Euler(), p = new THREE.Vector3(), one = new THREE.Vector3(1, 1, 1);
  const nm = new THREE.Matrix3();
  return {
    add(geo, x = 0, y = 0, z = 0, rx = 0, ry = 0, rz = 0) {
      const g = geo.index ? geo.toNonIndexed() : geo.clone();
      m.compose(p.set(x, y, z), q.setFromEuler(e.set(rx, ry, rz, 'XYZ')), one);
      nm.getNormalMatrix(m);
      const P = g.attributes.position, N = g.attributes.normal, v = new THREE.Vector3();
      for (let i = 0; i < P.count; i++) {
        v.fromBufferAttribute(P, i).applyMatrix4(m); pos.push(v.x, v.y, v.z);
        v.fromBufferAttribute(N, i).applyMatrix3(nm).normalize(); nor.push(v.x, v.y, v.z);
      }
      g.dispose(); geo.dispose();
    },
    box(sx, sy, sz, x, y, z, rx, ry, rz) { this.add(new THREE.BoxGeometry(sx, sy, sz), x, y, z, rx, ry, rz); },
    /** cylinder along the y axis (default), 'x' or 'z' */
    cyl(r, len, x, y, z, axis = 'y', seg = 16) {
      const rot = axis === 'x' ? [0, 0, PI / 2] : axis === 'z' ? [PI / 2, 0, 0] : [0, 0, 0];
      this.add(new THREE.CylinderGeometry(r, r, len, seg), x, y, z, ...rot);
    },
    done() {
      const g = new THREE.BufferGeometry();
      g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
      g.setAttribute('normal', new THREE.Float32BufferAttribute(nor, 3));
      return g;
    },
    get empty() { return pos.length === 0; },
  };
}

function resolveAlliance(v) {
  if (v == null || v === 'none') return null;
  if (typeof v === 'number') return v;
  const s = String(v).toLowerCase();
  if (s === 'red') return 0xd62828;
  if (s === 'blue') return 0x1d4ed8;
  if (/^#?[0-9a-f]{6}$/.test(s)) return parseInt(s.replace('#', ''), 16);
  return null;
}

export function buildRobot(THREE, spec, opts = {}) {
  const std = (c, r = 0.5, mt = 0.3, extra = {}) => new THREE.MeshStandardMaterial({ color: c, roughness: r, metalness: mt, ...extra });
  // ---- material palette: raw + black-anodised aluminium, MaxBot teal/orange accents, urethane yellow intake, purple gate ----
  const M = {
    alu: std(0xc9ced8, 0.32, 0.9), anod: std(0x2f3540, 0.42, 0.75), black: std(0x15171c, 0.55, 0.4),
    steel: std(0x9aa0aa, 0.3, 0.9), rubber: std(0x202228, 0.85, 0.05), tread: std(0x3a3d45, 0.9, 0.05),
    orange: std(0xff7a1a, 0.4, 0.3), yellow: std(0xffc926, 0.55, 0.1), motor: std(0xaeb3bc, 0.35, 0.8),
    gearbox: std(0x2b2e35, 0.5, 0.5), teal: std(0x1d8f8a, 0.55, 0.2), gate: std(0xb06af0, 0.5, 0.2),
    wire: std(0x0c0c0e, 0.8, 0.0), wireRed: std(0xc2231f, 0.8, 0.0), batt: std(0x2a2d35, 0.6, 0.2),
    battLabel: std(0xe0b030, 0.5, 0.1), controlHub: std(0x1f8a4c, 0.5, 0.2), expHub: std(0x2457c5, 0.5, 0.2),
    belt: std(0x111216, 0.9, 0.0), pcb: std(0x0e5a3a, 0.5, 0.3),
    lens: std(0x1b2a5a, 0.1, 0.8), led: new THREE.MeshStandardMaterial({ color: 0xffffff, emissive: 0xdde8ff, emissiveIntensity: 0.9 }),
    glass: std(0x9fd8ff, 0.1, 0.0, { transparent: true, opacity: 0.22, side: THREE.DoubleSide, depthWrite: false }),
  };
  M.hoodShell = std(0xff7a1a, 0.4, 0.3, { side: THREE.DoubleSide });

  const mesh = (geo, mat, parent, x = 0, y = 0, z = 0) => {
    const o = new THREE.Mesh(geo, mat); o.position.set(x, y, z); o.castShadow = true; o.receiveShadow = true;
    if (parent) parent.add(o); return o;
  };
  /** per-parent set of merged batches, one per material */
  const kit = parent => {
    const mp = new Map();
    return {
      b(mat) { if (!mp.has(mat)) mp.set(mat, makeBatch(THREE)); return mp.get(mat); },
      flush() { for (const [mat, b] of mp) if (!b.empty) mesh(b.done(), mat, parent); mp.clear(); },
    };
  };

  // ---- shape helpers ----
  const circ = (x, y, r) => { const p = new THREE.Path(); p.absarc(x, y, r, 0, PI * 2, false); return p; };
  const rrect = (x0, y0, x1, y1, r) => {
    const s = new THREE.Shape();
    s.moveTo(x0 + r, y0); s.lineTo(x1 - r, y0); s.absarc(x1 - r, y0 + r, r, -PI / 2, 0, false);
    s.lineTo(x1, y1 - r); s.absarc(x1 - r, y1 - r, r, 0, PI / 2, false);
    s.lineTo(x0 + r, y1); s.absarc(x0 + r, y1 - r, r, PI / 2, PI, false);
    s.lineTo(x0, y0 + r); s.absarc(x0 + r, y0 + r, r, PI, PI * 1.5, false);
    return s;
  };
  const annulus = (ro, ri) => { const s = new THREE.Shape(); s.absarc(0, 0, ro, 0, PI * 2, false); s.holes.push(circ(0, 0, ri)); return s; };
  /** grid of lightening holes, keeping only those accepted by keep(x, y) */
  const holeGrid = (x0, y0, x1, y1, pitch, r, keep = null) => {
    const hs = [], nx = Math.max(0, Math.floor((x1 - x0) / pitch) + 1), ny = Math.max(0, Math.floor((y1 - y0) / pitch) + 1);
    const cx = (x0 + x1) / 2 - (nx - 1) * pitch / 2, cy = (y0 + y1) / 2 - (ny - 1) * pitch / 2;
    for (let i = 0; i < nx; i++) for (let j = 0; j < ny; j++) {
      const x = cx + i * pitch, y = cy + j * pitch;
      if (!keep || keep(x, y)) hs.push(circ(x, y, r));
    }
    return hs;
  };
  /** flat plate in the xy plane centred on z=0, thickness t */
  const plate = (shape, holes, t) => {
    shape.holes.push(...holes);
    return new THREE.ExtrudeGeometry(shape, { depth: t, bevelEnabled: false, curveSegments: 8 }).translate(0, 0, -t / 2);
  };
  /** the same plate stood up in the xz plane (shape x -> x, shape y -> z), thickness along y */
  const plateXZ = (shape, holes, t) => plate(shape, holes, t).rotateX(PI / 2);
  /** annular sector for the hood cheeks, angle measured from +z toward +x, in the shape's (x, z) plane */
  const sector = (rin, rout, th0, th1, n = 18) => {
    const s = new THREE.Shape(), pt = (r, th) => [r * Math.sin(th), r * Math.cos(th)];
    let [x, y] = pt(rout, th0); s.moveTo(x, y);
    for (let i = 1; i <= n; i++) { [x, y] = pt(rout, th0 + (th1 - th0) * i / n); s.lineTo(x, y); }
    for (let i = n; i >= 0; i--) { [x, y] = pt(rin, th0 + (th1 - th0) * i / n); s.lineTo(x, y); }
    return s;
  };
  /** a toothed ring or pinion approximated by boxes around a cylinder */
  const gear = (k, rPitch, thick, z, cx, cy, teeth, mat, bore = 0) => {
    const b = k.b(mat);
    if (bore > 0) b.add(plate(annulus(rPitch - 0.06, bore), [], thick), cx, cy, z); else b.cyl(rPitch - 0.06, thick, cx, cy, z, 'z', 20);
    for (let i = 0; i < teeth; i++) { const a = i / teeth * PI * 2; b.box(0.13, 0.1, thick, cx + rPitch * Math.cos(a), cy + rPitch * Math.sin(a), z, 0, 0, a); }
  };

  const group = new THREE.Group(); group.name = 'robot';
  const parts = {
    chassis: null, wheels: [], intake: null, intakeRear: null, intakes: [], intakeRollers: [], transfer: null, transferRollers: [],
    sensors: [], camera: null, flower: null, flowerArm: null, gate: null, gateRear: null, battery: null, bumper: null,
    electronics: null, wiring: null, motors: null, odometry: null, pods: [],
  };

  // ---------------- dimensions from the spec (inches) ----------------
  const ch = spec.chassis;
  const z0 = ch.plateZ ?? 0.55, T = ch.plateThickness ?? 0.25, deckZ = ch.deckZ ?? 8.82, deckT = ch.deckThickness ?? 0.25;
  const cassH = ch.cassetteHeight ?? 8.57, laneZ = ch.laneCenterZ ?? 5.2, laneHalf = (ch.laneWidth ?? 3.54) / 2, ballD = ch.ballDia ?? 2.8;
  let fx0 = Infinity, fx1 = -Infinity, fy = 0;
  for (const p of ch.footprint) { fx0 = Math.min(fx0, p[0]); fx1 = Math.max(fx1, p[0]); fy = Math.max(fy, Math.abs(p[1])); }
  const wheels = spec.wheels || [];
  const wy = wheels.length ? Math.min(...wheels.map(w => Math.abs(w.y) - w.width / 2)) : 5.6;
  const platesY = wy - 0.15;                                   // cassette plate sits just inboard of the wheels
  const sh = spec.shooter, tu = sh.turret, RR = tu.ringRadius, boreR = tu.boreRadius ?? 2.07;
  const deckHalfX = Math.min(5.0, fx1 - 2.5), deckHalfY = platesY + 0.1;

  // ================= chassis: lane floor, spine walls, cassette plates, ties, deck with the turret bore =================
  {
    const g = new THREE.Group(); g.name = 'chassis'; group.add(g); parts.chassis = g;
    const k = kit(g), pods = (spec.odometry && spec.odometry.pods) || [];
    // lane floor, with slots where the odometry pod wheels poke through
    const floorHalf = laneHalf + 0.35, floorHoles = pods.map(p => circ(p.x, p.y, 0.8));
    k.b(M.black).add(plate(rrect(fx0 + 0.9, -floorHalf, fx1 - 0.9, floorHalf, 0.5), floorHoles, T), 0, 0, z0 + T / 2);
    // spine walls (the ball lane is the gap between them) from floor to deck
    for (const s of [-1, 1]) {
      const x0 = fx0 + 1.2, x1 = fx1 - 1.2, zt = deckZ - deckT;
      k.b(M.alu).add(plateXZ(rrect(x0, z0, x1, zt, 0.4), holeGrid(x0 + 1, z0 + 0.8, x1 - 1, zt - 0.8, 1.5, 0.42, (x, y) => y > z0 + 1.2), 0.18), 0, s * (laneHalf + 0.1), 0);
    }
    // drive cassettes: one plate per side (same part, turned 180 deg), axle bearing caps outside the wheels
    for (const s of [-1, 1]) {
      const x0 = fx0 + 0.7, x1 = fx1 - 0.7;
      const holes = [];
      for (const w of wheels) if (Math.sign(w.y) === s) holes.push(circ(w.x, w.radius, 0.45));
      holes.push(...holeGrid(x0 + 1, w0(wheels) + 0.5, x1 - 1, cassH - 0.7, 1.45, 0.42));
      k.b(M.alu).add(plateXZ(rrect(x0, 0.3, x1, cassH, 0.6), holes, 0.16), 0, s * platesY, 0);
      k.b(M.anod).box(x1 - x0, 0.35, 0.3, (x0 + x1) / 2, s * (platesY - 0.2), cassH - 0.15);        // top rail of the cassette
      for (const w of wheels) if (Math.sign(w.y) === s) {
        k.b(M.anod).cyl(0.95, 0.18, w.x, s * (Math.abs(w.y) + w.width / 2 + 0.12), w.radius, 'y', 18);   // bearing cap
        k.b(M.steel).cyl(0.22, 0.1, w.x, s * (Math.abs(w.y) + w.width / 2 + 0.24), w.radius, 'y', 8);
      }
    }
    // ties and tension plates between spine and cassettes
    for (const s of [-1, 1]) for (const x of [-(fx1 - 1.3), -1.0, 1.0, fx1 - 1.3])
      k.b(M.anod).box(0.28, platesY - floorHalf, 0.25, x, s * (floorHalf + (platesY - floorHalf) / 2), z0 + 0.12);
    // deck (z 224 mm) with the turret bore and lightening holes
    const dz = deckZ - deckT / 2;
    const keep = (x, y) => Math.hypot(x, y) > boreR + 1.3 && Math.abs(x) < deckHalfX - 0.7 && Math.abs(y) < deckHalfY - 0.7;
    k.b(M.alu).add(plate(rrect(-deckHalfX, -deckHalfY, deckHalfX, deckHalfY, 0.7), [circ(0, 0, boreR), ...holeGrid(-deckHalfX, -deckHalfY, deckHalfX, deckHalfY, 1.25, 0.38, keep)], deckT), 0, 0, dz);
    // odometry: Pinpoint board under the lane floor centre, two spring pods with 32 mm wheels
    const og = new THREE.Group(); og.name = 'odometry'; group.add(og); parts.odometry = og;
    const ok = kit(og);
    ok.b(M.black).box(1.3, 1.0, 0.3, 0, 0, z0 + T + 0.2);
    ok.b(M.led).box(0.2, 0.2, 0.05, 0.4, 0.3, z0 + T + 0.37);
    for (const p of pods) {
      const alongX = p.axis === 'x';                         // axis of the wheel; 'y' pod rolls along x
      const pg = new THREE.Group(); pg.name = p.name; pg.position.set(p.x, p.y, 0.63); og.add(pg); parts.pods.push(pg);
      const pk = kit(pg);
      pk.b(M.rubber).cyl(0.63, 0.5, 0, 0, 0, alongX ? 'x' : 'y', 18);
      pk.b(M.alu).cyl(0.3, 0.55, 0, 0, 0, alongX ? 'x' : 'y', 10);
      pk.b(M.anod).box(alongX ? 0.4 : 1.4, alongX ? 1.4 : 0.4, 0.2, 0, 0, 0.8);        // swing-arm
      pk.flush();
    }
    ok.flush();
    k.flush();
  }

  // ================= back plate (alliance colour), standing on the rear of the deck =================
  const ally = resolveAlliance(opts.allianceColor ?? spec.alliance);
  if (ally != null) {
    const g = new THREE.Group(); g.name = 'bumper';
    const bh = ch.bumperHeight || 3;
    mesh(new THREE.BoxGeometry(0.3, deckHalfY * 2 - 1.2, bh), std(ally, 0.6, 0.05), g, -deckHalfX + 0.2, 0, deckZ + bh / 2);
    const k = kit(g);
    for (const s of [-1, 1]) k.b(M.black).box(0.7, 0.4, 0.5, -deckHalfX + 0.55, s * (deckHalfY - 1.0), deckZ + 0.25);
    k.flush(); group.add(g); parts.bumper = g;
  }

  // ================= wheels: 104 mm mecanum, side plates + 9 angled rollers, belt drive, motors =================
  const motorsG = new THREE.Group(); motorsG.name = 'motors'; group.add(motorsG); parts.motors = motorsG;
  const mk = kit(motorsG);
  for (const w of wheels) {
    const wg = new THREE.Group(); wg.name = 'wheel_' + w.name; wg.position.set(w.x, w.y, w.radius);
    const k = kit(wg), R = w.radius, sgn = w.x * w.y > 0 ? 1 : -1;
    if (w.type === 'traction') {
      k.b(M.rubber).cyl(R, w.width * 0.9, 0, 0, 0, 'y', 28);
      k.b(M.alu).cyl(R * 0.6, w.width, 0, 0, 0, 'y', 20);
    } else {
      const rr = Math.min(0.38, R * 0.19), len = Math.min(w.width * 0.7, 1.3), ring = R - rr - 0.1;
      for (const s of [-1, 1]) {
        const sh0 = new THREE.Shape(); sh0.absarc(0, 0, ring - 0.1, 0, PI * 2, false);   // side plate with 5 lightening holes
        const hs = []; for (let i = 0; i < 5; i++) { const a = i / 5 * PI * 2; hs.push(circ(Math.cos(a) * ring * 0.62, Math.sin(a) * ring * 0.62, ring * 0.2)); }
        k.b(M.anod).add(plate(sh0, hs, 0.12).rotateX(PI / 2), 0, s * (w.width / 2 - 0.1), 0);
        k.b(M.orange).cyl(0.42, 0.06, 0, s * (w.width / 2 + 0.01), 0, 'y', 14);
      }
      k.b(M.black).cyl(0.62, w.width, 0, 0, 0, 'y', 16);
      k.b(M.steel).cyl(0.2, w.width + 0.04, 0, 0, 0, 'y', 8);
      const rollGeo = new THREE.CylinderGeometry(rr, rr, len, 8);
      const A = w.type === 'mecanum' ? PI / 4 * sgn : PI / 2;            // roller axis: 45 deg to the axle (mecanum) or along the tangent (omni)
      const up = new THREE.Vector3(0, 1, 0), dir = new THREE.Vector3();
      for (let i = 0; i < 9; i++) {
        const th = i / 9 * PI * 2;
        dir.set(-Math.sin(th) * Math.sin(A), Math.cos(A), Math.cos(th) * Math.sin(A)).normalize();
        const r = mesh(rollGeo, M.tread, wg, ring * Math.cos(th), 0, ring * Math.sin(th));
        r.quaternion.setFromUnitVectors(up, dir); r.name = 'roller';
        const pin = mesh(new THREE.CylinderGeometry(0.1, 0.1, len + 0.3, 6), M.steel, r); pin.name = 'rollerPin'; pin.castShadow = false;
      }
    }
    k.flush(); group.add(wg); parts.wheels.push(wg);

    // inboard of the cassette plate: axle, big pulley, belt to a 5203 motor (24:48), motor with encoder cap
    const s = Math.sign(w.y) || 1, mx = w.x - Math.sign(w.x || 1) * 2.0, mz = w.radius + 0.25, by = s * (platesY - 0.3);
    mk.b(M.steel).cyl(0.22, wy - platesY + 0.9, w.x, s * (platesY + (wy - platesY) / 2 - 0.1), w.radius, 'y', 8);
    mk.b(M.anod).cyl(0.85, 0.3, w.x, by, w.radius, 'y', 20);
    mk.b(M.anod).cyl(0.42, 0.3, mx, by, mz, 'y', 14);
    const dx = mx - w.x, dz = mz - w.radius, len = Math.hypot(dx, dz);
    mk.b(M.belt).box(len, 0.28, 0.12, (mx + w.x) / 2, by, (mz + w.radius) / 2, 0, -Math.atan2(dz, dx), 0);
    mk.b(M.motor).cyl(0.71, 2.7, mx, s * (platesY - 1.9), mz, 'y', 16);
    mk.b(M.gearbox).cyl(0.74, 0.9, mx, s * (platesY - 0.75), mz, 'y', 16);
    mk.b(M.black).cyl(0.55, 0.35, mx, s * (platesY - 3.4), mz, 'y', 12);
  }
  mk.flush();

  // ================= intakes: one build for the front, mirrored for the rear =================
  const ik = spec.intake && spec.intake.present ? spec.intake : null;
  const gt = spec.gate && spec.gate.present ? spec.gate : null;
  const protOut = ik ? ik.protrusionDeployedIn ?? 4.2 : 4.2, protIn = ik ? ik.protrusionFoldedIn ?? 1.5 : 1.5;
  const foldAt = d => protIn + (protOut - protIn) * d - protOut;           // local x offset of the arm for fold fraction d
  const makeRoller = (r, len, parent, x, z, name) => {
    const rg = new THREE.Group(); rg.name = name; rg.position.set(x, 0, z); parent.add(rg);
    const k = kit(rg);
    k.b(M.steel).cyl(r * 0.35, len + 0.3, 0, 0, 0, 'y', 10);
    const n = 8, pitch = len / n;
    for (let i = 0; i < n; i++) {                                           // soft flex wheels along the shaft
      k.b(M.yellow).cyl(r, pitch * 0.62, 0, -len / 2 + pitch * (i + 0.5), 0, 'y', 16);
      k.b(M.orange).cyl(r * 0.6, pitch * 0.7, 0, -len / 2 + pitch * (i + 0.5), 0, 'y', 10);
    }
    for (let i = 0; i < 4; i++) { const a = i * PI / 2; k.b(M.black).box(0.1, len, r * 0.45, (r + 0.02) * Math.cos(a), 0, (r + 0.02) * Math.sin(a), 0, -a, 0); }   // fins make the spin visible
    k.flush(); return rg;
  };
  const buildIntake = (sign, name, key) => {
    const g = new THREE.Group(); g.name = name; g.position.set(sign * fx1, 0, 0); if (sign < 0) g.rotation.z = PI;   // local +x points outward
    const arm = new THREE.Group(); arm.name = name + 'Arm'; arm.position.x = foldAt(0); g.add(arm);
    const W = ik.width, rr = ik.rollerRadius, lowZ = ik.lowerRollerZ ?? 3.3, upZ = ik.upperRollerZ ?? 7.1;
    const lowX = protOut - rr - 0.1, upX = lowX - 0.4;
    const k = kit(arm);
    for (const s of [-1, 1]) {                                              // side plates carry both rollers
      const x0 = -2.4, zb = lowZ - 1.4, zt = upZ + 1.3;
      k.b(M.alu).add(plateXZ(rrect(x0, zb, protOut, zt, 0.5), [circ(lowX, lowZ, 0.3), circ(upX, upZ, 0.3), ...holeGrid(x0 + 0.8, zb + 0.6, protOut - 2.2, zt - 0.6, 1.4, 0.36, (x, y) => Math.hypot(x - lowX, y - lowZ) > 1.2 && Math.hypot(x - upX, y - upZ) > 1.2)], 0.14), 0, s * (W / 2 + 0.05), 0);
      k.b(M.anod).cyl(0.4, 0.12, lowX, s * (W / 2 + 0.17), lowZ, 'y', 12);
      k.b(M.anod).cyl(0.4, 0.12, upX, s * (W / 2 + 0.17), upZ, 'y', 12);
    }
    // lane funnel guides at ball height, so the full-width mouth narrows to the centreline lane
    for (const s of [-1, 1]) {
      const x0 = lowX - 0.2, y0 = s * (W / 2 - 0.9), x1 = -1.2, y1 = s * (laneHalf + 0.1), len = Math.hypot(x1 - x0, y1 - y0);
      k.b(M.black).box(len, 0.12, 2.3, (x0 + x1) / 2, (y0 + y1) / 2, laneZ, 0, 0, Math.atan2(y1 - y0, x1 - x0));
    }
    // two sprung hook fingers past the equator of a seated pollen, plus the ring B pocket lip on the lower bar
    for (const s of [-1, 1]) {
      const c = new THREE.CatmullRomCurve3([new THREE.Vector3(lowX - 0.2, s * 2.6, laneZ + 0.9), new THREE.Vector3(lowX + 0.5, s * 2.6, laneZ + 0.5), new THREE.Vector3(lowX + 0.55, s * 2.6, laneZ - 0.5)]);
      k.b(M.orange).add(new THREE.TubeGeometry(c, 8, 0.07, 6, false));
    }
    k.b(M.teal).box(0.3, W - 1.2, 0.2, lowX - rr - 0.1, 0, lowZ + 0.1);
    k.flush();
    const lo = makeRoller(rr, W - 0.6, arm, lowX, lowZ, 'intakeRoller'), up = makeRoller(rr, W - 0.6, arm, upX, upZ, 'intakeRoller');
    parts.intakeRollers.push(lo, up);
    group.add(g); parts[key] = g; parts.intakes.push(g);
    g.userData.arm = arm;
    // mouth gate lever on the frame (78 mm refuses nectar, 97 mm passes both)
    if (gt) {
      const gg = new THREE.Group(); gg.name = name === 'intake' ? 'gate' : 'gateRear';
      gg.position.set(sign * gt.x, gt.y, gt.z); if (sign < 0) gg.rotation.z = PI;
      mesh(new THREE.BoxGeometry(gt.thickness, gt.width, gt.height), M.gate, gg, 0, 0, gt.height / 2);
      group.add(gg); parts[name === 'intake' ? 'gate' : 'gateRear'] = gg;
    }
  };
  if (ik) { buildIntake(1, 'intake', 'intake'); if (ik.dual !== false) buildIntake(-1, 'intakeRear', 'intakeRear'); }

  // ================= transfer cartridge on T-rails =================
  const tr = spec.transfer;
  if (tr && tr.present) {
    const g = new THREE.Group(); g.name = 'transfer'; g.position.set(tr.x, tr.y, tr.z); group.add(g); parts.transfer = g;
    const k = kit(g), hw = tr.width / 2;
    for (const s of [-1, 1]) {
      k.b(M.teal).add(plateXZ(rrect(-tr.length / 2, -tr.height / 2, tr.length / 2, tr.height / 2, 0.4), holeGrid(-tr.length / 2 + 0.8, -tr.height / 2 + 0.7, tr.length / 2 - 0.8, tr.height / 2 - 0.7, 1.2, 0.3), 0.12), 0, s * hw, 0);
      k.b(M.anod).box(tr.length + 0.6, 0.18, 0.18, 0, s * (hw + 0.14), -tr.height / 2 + 0.1);          // T-rail
    }
    k.flush();
    for (const dx of [-tr.length * 0.33, 0, tr.length * 0.33]) parts.transferRollers.push(makeRoller(0.45, tr.width - 0.3, g, dx, -tr.height / 2 + 0.55, 'transferRoller'));
  }

  // ================= battery, Control Hub, Expansion Hub, wiring =================
  const eg = new THREE.Group(); eg.name = 'electronics'; group.add(eg); parts.electronics = eg;
  if (spec.battery) {
    const bt = spec.battery, g = new THREE.Group(); g.name = 'battery'; g.position.set(bt.x, bt.y, bt.z);
    const k = kit(g);
    k.b(M.batt).box(bt.sx, bt.sy, bt.sz, 0, 0, 0);
    k.b(M.battLabel).box(bt.sx * 0.6, 0.04, bt.sz * 0.6, 0, bt.sy / 2 + 0.02, 0);
    k.b(M.black).box(0.5, bt.sy + 0.06, bt.sz + 0.06, bt.sx * 0.28, 0, 0);       // velcro straps
    k.b(M.black).box(0.5, bt.sy + 0.06, bt.sz + 0.06, -bt.sx * 0.28, 0, 0);
    k.b(M.orange).box(0.5, 0.7, 0.9, bt.sx / 2 + 0.25, 0, bt.sz / 2 - 0.7);      // connector
    k.flush(); eg.add(g); parts.battery = g;
  }
  for (const [name, mat, sy, x] of [['controlHub', M.controlHub, 1, 2.5], ['expansionHub', M.expHub, -1, 2.5]]) {
    const g = new THREE.Group(); g.name = name; g.position.set(x, sy * 4.5, 6.9);
    const k = kit(g);
    k.b(mat).box(3.9, 0.9, 3.4, 0, 0, 0);                                         // stood on edge under the deck
    k.b(M.black).box(0.5, 0.5, 3.0, 1.8, 0, 0.1);                                  // port strips
    k.b(M.black).box(0.5, 0.5, 3.0, -1.8, 0, 0.1);
    k.b(M.led).box(0.25, 0.05, 0.25, -0.8, sy * 0.47, 1.2);
    k.flush(); eg.add(g);
  }
  {
    const g = new THREE.Group(); g.name = 'powerSwitch'; g.position.set(-3.4, -4.5, 6.9); eg.add(g);
    mesh(new THREE.BoxGeometry(1.4, 0.9, 1.2), M.black, g); mesh(new THREE.BoxGeometry(0.4, 0.3, 0.5), M.wireRed, g, 0, 0.4, 0.5);
  }
  {
    const wg = new THREE.Group(); wg.name = 'wiring'; group.add(wg); parts.wiring = wg;
    const k = kit(wg), V = (x, y, z) => new THREE.Vector3(x, y, z);
    for (const w of wheels) {
      const s = Math.sign(w.y) || 1, mx = w.x - Math.sign(w.x || 1) * 2.0;
      const c = new THREE.CatmullRomCurve3([V(2.5 + Math.sign(w.x) * 1.9, s * 4.5, 6.6), V(w.x * 0.8, s * 4.4, 5.0), V(mx, s * (platesY - 3.5), w.radius + 0.9)]);
      k.b(M.wire).add(new THREE.TubeGeometry(c, 10, 0.06, 5, false));
    }
    if (spec.battery) {
      const bt = spec.battery;
      const c = new THREE.CatmullRomCurve3([V(bt.x + bt.sx / 2 + 0.5, bt.y, bt.z), V(-1.0, 3.6, 7.5), V(0.6, 4.2, 7.0), V(2.5 - 1.9, 4.5, 7.0)]);
      k.b(M.wireRed).add(new THREE.TubeGeometry(c, 10, 0.09, 5, false));
    }
    k.flush();
  }

  // ================= flower mechanism: drop-down arm (stowed inside the frame, folds down over a flower) =================
  const fl = spec.flower;
  if (fl && fl.present) {
    const A = buildFlowerArm(THREE, opts.flowerArmCfg || DEFAULT_ARM_CFG);
    group.add(A.group); parts.flower = A.group; parts.flowerArm = A;
  }

  // ================= turret: fixed ring + pinion drive on the deck, rotating ring + gear, head =================
  const pv = sh.pivot, ex = sh.exitPoint, hp = sh.hood.pivot, above = sh.flywheelAbove ?? 0, comp = Math.max(0, sh.compression || 0);
  {
    const k = kit(group);
    const ringZ0 = deckZ, ringT = Math.max(0.2, pv.z - deckZ - 0.38);
    k.b(M.black).add(plate(annulus(RR, boreR), [], ringT), pv.x, pv.y, ringZ0 + ringT / 2);                   // fixed bearing ring on the deck
    for (let i = 0; i < 12; i++) { const a = i / 12 * PI * 2; k.b(M.steel).cyl(0.1, 0.1, pv.x + (RR - 0.3) * Math.cos(a), pv.y + (RR - 0.3) * Math.sin(a), ringZ0 + ringT + 0.02, 'z', 6); }
    // turret motor under the deck, pinion (64T) on its shaft meshing the 176T ring, limit switch beside the ring
    const pa = rad(tu.pinionAngleDeg ?? -135), pd = tu.pinionDistance ?? 3.78, pr = tu.pinionRadius ?? 1.0;
    const px = pv.x + pd * Math.cos(pa), py = pv.y + pd * Math.sin(pa);
    k.b(M.motor).cyl(0.71, 4.2, px, py, deckZ - deckT - 2.1, 'z', 16);
    k.b(M.gearbox).cyl(0.74, 0.9, px, py, deckZ - deckT - 0.5, 'z', 16);
    k.b(M.steel).cyl(0.2, 1.5, px, py, deckZ + 0.55, 'z', 8);
    gear(k, pr, 0.3, pv.z - 0.25, px, py, 16, M.anod);
    k.b(M.black).box(0.7, 0.4, 0.4, pv.x + (RR + 0.5) * Math.cos(rad(60)), pv.y + (RR + 0.5) * Math.sin(rad(60)), ringZ0 + 0.2);
    k.flush();
  }
  const turretGroup = new THREE.Group(); turretGroup.name = 'turret'; turretGroup.position.set(pv.x, pv.y, pv.z);
  group.add(turretGroup);
  const fcT = { x: ex.x, y: ex.y, z: ex.z + above };                          // flywheel centre relative to the turret pivot
  const fc = { x: fcT.x - hp.x, y: fcT.y - hp.y, z: fcT.z - hp.z };            // ... and relative to the hood pivot
  const fr = sh.flywheelRadius, fw = sh.flywheelWidth, hoodW = sh.hood.width;
  {
    const k = kit(turretGroup), ringT = Math.max(0.2, pv.z - deckZ - 0.38);
    k.b(M.anod).add(plate(annulus(RR, boreR), [], 0.38), 0, 0, -0.19);                                      // rotating ring on the bearing
    gear(k, 2.77, 0.3, 0.15, 0, 0, 44, M.steel, boreR);                                                      // 176T ring gear (every 4th tooth drawn)
    void ringT;
    // head base plate with the bore, then the two head plates either side of the ball column
    const bx0 = -3.4, bx1 = fcT.x + 1.2, bh = RR;
    k.b(M.alu).add(plate(rrect(bx0, -bh, bx1, bh, 0.7), [circ(0, 0, boreR), ...holeGrid(bx0 + 0.8, -bh + 0.8, bx1 - 0.8, bh - 0.8, 1.3, 0.36, (x, y) => Math.hypot(x, y) > boreR + 0.8)], 0.22), 0, 0, 0.4);
    const uy = hoodW / 2 + 0.3, ux0 = -2.2, ux1 = fcT.x + fr + 0.6, uz1 = fcT.z + fr + 0.5;
    for (const s of [-1, 1]) {
      const holes = [circ(fcT.x, fcT.z, 0.45)];
      for (const hx of [-1.4, 0.1, 1.4]) for (const hz of [1.9, 4.2, 6.5, 8.8]) holes.push(circ(hx, hz, 0.45));
      k.b(M.alu).add(plateXZ(rrect(ux0, 0.5, ux1, uz1, 0.6), holes, 0.18), 0, s * uy, 0);
    }
    // flywheel motor coaxial behind the +y plate, bearing block on the other side
    k.b(M.motor).cyl(0.71, 2.6, fcT.x, uy + 1.5, fcT.z, 'y', 16);
    k.b(M.gearbox).cyl(0.74, 0.9, fcT.x, uy + 0.45, fcT.z, 'y', 16);
    k.b(M.black).cyl(0.55, 0.3, fcT.x, uy + 2.9, fcT.z, 'y', 12);
    k.b(M.anod).cyl(0.6, 0.3, fcT.x, -uy - 0.2, fcT.z, 'y', 14);
    k.flush();
    // translucent ball column rising through the 105 mm bore to the flywheel (the lane continues up the centreline)
    const colTop = ex.z - 0.2, colBot = -1.2;
    mesh(new THREE.CylinderGeometry(1.55, 1.55, colTop - colBot, 24, 1, true), M.glass, turretGroup, 0, 0, (colTop + colBot) / 2).rotation.x = PI / 2;
    const rk = kit(turretGroup);
    for (const z of [colBot + 0.3, 2.5, 5.0, colTop - 0.2]) rk.b(M.alu).add(plate(annulus(1.7, 1.5), [], 0.12), 0, 0, z);
    rk.flush();
  }

  const hoodPivot = new THREE.Group(); hoodPivot.name = 'hoodPivot';
  hoodPivot.position.set(hp.x, hp.y, hp.z); turretGroup.add(hoodPivot);
  const flywheelMesh = new THREE.Group(); flywheelMesh.name = 'flywheel';
  flywheelMesh.position.set(fc.x, fc.y, fc.z); hoodPivot.add(flywheelMesh);
  {
    // ONE 96 mm compliant flywheel on a shaft, with two 208 g weights; this group is what the host rotates
    const k = kit(flywheelMesh), ww = fw - 0.2;
    k.b(M.rubber).cyl(fr, ww, 0, 0, 0, 'y', 32);
    k.b(M.orange).cyl(fr * 0.55, ww + 0.06, 0, 0, 0, 'y', 24);
    for (const s of [-1, 1]) {
      k.b(M.steel).cyl(fr * 0.78, 0.14, 0, s * (ww / 2 + 0.1), 0, 'y', 24);                                  // weight discs
      k.b(M.steel).cyl(fr * 0.2, 0.2, 0, s * (ww / 2 + 0.18), 0, 'y', 10);
      for (let i = 0; i < 6; i++) { const a = i / 6 * PI * 2; k.b(M.black).cyl(0.14, 0.16, Math.cos(a) * fr * 0.42, s * (ww / 2 + 0.12), Math.sin(a) * fr * 0.42, 'y', 6); }
    }
    k.b(M.steel).cyl(0.18, fw + 1.4, 0, 0, 0, 'y', 8);                       // shaft
    k.flush();
    mesh(new THREE.BoxGeometry(0.15, fw * 0.8, 0.5), M.yellow, flywheelMesh, fr * 0.8, 0, 0).name = 'flywheelMark';
  }
  {
    // hood: curved floor under the wheel (ball squeezed between wheel and hood), side cheeks, pivot bolt
    const k = kit(hoodPivot), aR = fr + ballD - comp + 0.05, th0 = PI / 2, th1 = PI / 2 + 2.1;
    k.b(M.steel).cyl(0.22, hoodW + 0.5, 0, 0, 0, 'y', 8);
    const shell = new THREE.CylinderGeometry(aR, aR, hoodW - 0.2, 28, 1, true, th0, th1 - th0);
    mesh(shell, M.hoodShell, hoodPivot, fc.x, fc.y, fc.z).name = 'hoodShell';
    for (const s of [-1, 1]) k.b(M.alu).add(plateXZ(sector(aR - 0.1, aR + 0.45, th0, th1), [], 0.12), fc.x, fc.y + s * hoodW / 2, fc.z);
    k.flush();
  }
  hoodPivot.rotation.y = -rad((sh.hood.minDeg + sh.hood.maxDeg) / 2);

  // ================= camera (Limelight) on the turret =================
  const cam = spec.camera;
  if (cam) {
    const g = new THREE.Group(); g.name = 'camera'; g.position.set(cam.x, cam.y, cam.z);
    const k = kit(g);
    k.b(M.black).box(1.6, 3.6, 1.6, 0, 0, 0);
    k.b(M.alu).box(0.2, 3.7, 1.7, -0.7, 0, 0);                                  // heat-sink back plate
    k.b(M.black).cyl(0.55, 0.2, 0.85, 0, 0, 'x', 16);                           // lens barrel
    k.b(M.lens).cyl(0.4, 0.22, 0.9, 0, 0, 'x', 16);                             // lens glass
    for (const s of [-1, 1]) k.b(M.led).box(0.06, 0.3, 0.3, 0.81, s * 1.1, 0);  // illumination LEDs
    k.b(M.alu).box(1.2, 1.6, 0.15, 0, 0, -0.85);                                // bracket
    k.flush();
    g.rotation.y = -rad(cam.pitchDeg);
    (cam.mount === 'turret' ? turretGroup : group).add(g); parts.camera = g;
  }

  // ================= colour sensors: PCB with a lit sensor =================
  const sensorCol = { intakeColor: 0xff3df0, shooterColor: 0x3dffb0 };
  for (const s of spec.sensors || []) {
    const g = new THREE.Group(); g.name = s.name; g.position.set(s.x, s.y, s.z);
    const c = sensorCol[s.name] ?? 0xffffff;
    mesh(new THREE.BoxGeometry(0.9, 1.3, 0.2), M.pcb, g, 0, 0, -0.3);
    mesh(new THREE.BoxGeometry(0.5, 0.5, 0.3), new THREE.MeshStandardMaterial({ color: c, emissive: c, emissiveIntensity: 0.5 }), g, 0, 0, 0);
    (s.frame === 'turret' ? turretGroup : group).add(g); parts.sensors.push(g);
  }

  // ================= animation =================
  const anim = { wheelRad: wheels.map(() => 0), rollerRad: 0, flywheelRad: 0, deploy: 0 };
  const FLY_VISUAL = 0.05;                     // real rpm is far too fast to see at 60 fps, so it is slowed down for display
  const ROLLER_RAD_S = 28;

  /** Absolute pose. Degrees: turretDeg, hoodDeg, flowerArmDeg. Radians: flywheelRad, wheelRad[4], intakeRad, flowerRad (= arm angle, back compat).
   *  intakeDeploy 0 (folded, match start) .. 1 (open, in play). gateOpen 0..1. */
  function setPose(p = {}) {
    if (p.turretDeg != null) turretGroup.rotation.z = rad(p.turretDeg);
    if (p.hoodDeg != null) hoodPivot.rotation.y = -rad(p.hoodDeg);
    if (p.flywheelRad != null) flywheelMesh.rotation.y = p.flywheelRad;
    if (p.wheelRad) parts.wheels.forEach((w, i) => { w.rotation.y = p.wheelRad[i] ?? p.wheelRad[0]; });
    if (p.intakeRad != null) for (const r of parts.intakeRollers) r.rotation.y = p.intakeRad;
    if (p.transferRad != null) for (const r of parts.transferRollers) r.rotation.y = p.transferRad;
    if (p.intakeDeploy != null) { anim.deploy = Math.min(1, Math.max(0, p.intakeDeploy)); for (const g of parts.intakes) g.userData.arm.position.x = foldAt(anim.deploy); }
    if (p.flowerArmDeg != null && parts.flowerArm) parts.flowerArm.setAngle(p.flowerArmDeg);
    if (p.flowerRad != null && parts.flowerArm) parts.flowerArm.setAngle(p.flowerRad * 180 / PI);
    if (p.gateOpen != null) {
      if (parts.gate) parts.gate.rotation.y = -p.gateOpen * PI / 2;
      if (parts.gateRear) parts.gateRear.rotation.y = -p.gateOpen * PI / 2;
    }
  }

  /** Per-frame animation. st (all optional):
   *  vx, vy, om           robot-frame velocity (in/s, in/s, rad/s; +y left, +om counter-clockwise) -> wheels spin (or wheelOmega[4] in rad/s)
   *  intakeOn, intakeDir  rollers spin (dir -1 = outtake) and the intakes unfold; off = they fold back to 38 mm
   *  rpm                  flywheel speed (slowed for display)
   *  turretDeg, hoodDeg, flowerArmDeg  as in setPose
   *  intakeDeploy         force the fold fraction instead of following intakeOn */
  function animate(dt, st = {}) {
    dt = Math.min(Math.max(dt || 0, 0), 0.1);
    let om = st.wheelOmega;
    if (!om && (st.vx != null || st.vy != null || st.om != null)) {
      const vx = st.vx || 0, vy = st.vy || 0, w = st.om || 0;
      om = wheels.map(wh => {
        const sx = Math.sign(wh.x) || 1, sy = Math.sign(wh.y) || 1, kk = Math.abs(wh.x) + Math.abs(wh.y);
        return (vx - sx * sy * vy - sy * kk * w) / wh.radius;
      });
    }
    if (om) { anim.wheelRad = anim.wheelRad.map((a, i) => (a + (om[i] || 0) * dt) % (PI * 2)); setPose({ wheelRad: anim.wheelRad }); }
    const target = st.intakeDeploy != null ? st.intakeDeploy : (st.intakeOn ? 1 : 0), rate = dt / ((ik && ik.foldTimeS) || 0.4);
    const d = anim.deploy + Math.sign(target - anim.deploy) * Math.min(Math.abs(target - anim.deploy), rate);
    if (d !== anim.deploy) setPose({ intakeDeploy: d });
    if (st.intakeOn) {
      anim.rollerRad = (anim.rollerRad + (st.intakeDir ?? 1) * ROLLER_RAD_S * dt) % (PI * 2);
      setPose({ intakeRad: anim.rollerRad, transferRad: anim.rollerRad });
    }
    if (st.rpm != null) { anim.flywheelRad = (anim.flywheelRad + st.rpm / 60 * PI * 2 * FLY_VISUAL * dt) % (PI * 2); setPose({ flywheelRad: anim.flywheelRad }); }
    setPose({ turretDeg: st.turretDeg, hoodDeg: st.hoodDeg, flowerArmDeg: st.flowerArmDeg });
  }

  /** Frees GPU buffers of everything under `root` (default: the whole robot). Pass `except` (an Object3D, e.g. the flower arm) to keep it. */
  function dispose(root = group, except = null) {
    const keep = new Set(); if (except) except.traverse(o => keep.add(o));
    root.traverse(o => {
      if (keep.has(o) || (!o.isMesh && !o.isLine && !o.isPoints)) return;
      if (o.geometry) o.geometry.dispose();
      for (const m of Array.isArray(o.material) ? o.material : [o.material]) if (m) m.dispose();
    });
  }

  return { group, turretGroup, hoodPivot, flywheelMesh, parts, setPose, animate, dispose, state: anim };
}

// small helper used while building the cassette plate: the z of the top of the wheels (so lightening holes clear them)
function w0(wheels) { return wheels.length ? Math.max(...wheels.map(w => w.radius * 2)) : 4; }
