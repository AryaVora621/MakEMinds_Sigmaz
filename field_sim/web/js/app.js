// BIOBUZZ field simulator (WebGL, three.js). Frame = PEDRO inches: x right, y toward the far wall, z up.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';
import { CSS2DRenderer, CSS2DObject } from 'three/addons/renderers/CSS2DRenderer.js';

import { buildObstacles, resolveRobot, slideVelocity, ROBOT } from './collision.js';
import { BallWorld, BALL } from './balls.js';
import { parseHiveCollider } from './hivemesh.js';
import { FlowerArm, findFlowerContact, feedFromFlower } from './flowerarm.js';
import { HiveDynamics, HIVE_PARAMS } from './hivecycle.js';
import { readGamepad, mapKeyboard, MecanumDrive, MatchClock, driverCamera, PracticeStats } from './driver.js';
import { MatchScorer, RULES } from './matchrules.js';
import { createPathEditor } from './pathui.js';
import { sampleRoute, PLAN_VIEW, viewBounds, resolveHeadings, toScreen, fromScreen, allianceViewTransform, viewScale } from './pathing.js';
import { evaluateShot, forwardShot, feasibleRegion, setCore, REASON } from './shotrules.js';
import { LegalZone } from './legalzone.js';
import { ShooterPlant, ManualAim, aimInputFromKeys, aimInputFromGamepad, modeCommand, describeMode } from './aimcontrol.js';
import { loadRobotSpec, deriveConstants, provenanceReport, validateSpec } from './robotspec.js';
import { buildRobot } from './robotmodel.js';
import { tryLoadRobotGLB } from './robotglb.js';
import { initDiag } from './diag.js';
import { createAutoPlanner } from './autoui.js';
import { exportAutoJava } from './autoexport.js';
import { samplePedro } from './pedrosample.js';
const diag = initDiag();

const Core = window.Core; setCore(Core);                       // verified ballistic solver (same maths as ShotSolver.java)

// ================= constants (mirror TurretAiming.java; edit here to match your robot) =================
const SHOOTER = {
  exitHeight: 12, wheelR: 1.89, eff: 0.45, rpmScale: 1.0, maxRpm: 6000,
  entryDeg: 50, fixedElevDeg: 45, hoodMin: 25, hoodMax: 65,
  pivotX: 0, pivotY: 0,
  turretMinDeg: -180, turretMaxDeg: 180,
};
const PLANT = { turretRateDegS: 540, turretKp: 14, flywheelTauS: 0.35, rpmDipFrac: 0.06 };  // placeholder dynamics
const CAMERA = { x: 4.5, z: 14, pitchDeg: 30, hfovDeg: 54.5, vfovDeg: 42, rangeIn: 150 };    // Limelight 3A on the turret
const DRIVE = { maxSpeed: 60, maxAccel: 140, maxTurn: 3.2 };
const START = { x: 72, y: 24, h: Math.PI / 2 };
const BALL_R = 1.4, G = 386.0886;
// The robot is data-driven: data/robot_spec.json (CAD numbers go there). Its derived constants override the defaults above.
const robotSpec = await loadRobotSpec('data/robot_spec.json');
{
  const v = validateSpec(robotSpec), D = deriveConstants(robotSpec);
  Object.assign(SHOOTER, D.SHOOTER); Object.assign(PLANT, D.PLANT); Object.assign(CAMERA, D.CAMERA); Object.assign(DRIVE, D.DRIVE);
  v.errors.forEach(e => diag.toast('robot_spec.json: ' + e, 'bad', { sticky: true }));
  v.warnings.forEach(w => diag.log('robot_spec warning: ' + w));
  const pr = provenanceReport(robotSpec);
  diag.setSimNote('robot', 'robot: ' + pr.cad + ' CAD / ' + pr.repo + ' code / ' + pr.placeholder + ' placeholder values');
}

// ================= renderer, scene, lights =================
const canvas = document.getElementById('view');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, powerPreference: 'high-performance' });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.05;
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0d1220);
scene.fog = new THREE.Fog(0x0d1220, 500, 1200);
const pm = new THREE.PMREMGenerator(renderer);
scene.environment = pm.fromScene(new RoomEnvironment(), 0.04).texture;
scene.environmentIntensity = 0.55;
scene.add(new THREE.HemisphereLight(0xdfe8ff, 0x20242e, 0.55));
const sun = new THREE.DirectionalLight(0xffffff, 2.2);
sun.position.set(20, -60, 190);
sun.target.position.set(72, 72, 0);
sun.castShadow = true;
sun.shadow.mapSize.set(4096, 4096);
const sc = sun.shadow.camera; sc.left = -120; sc.right = 120; sc.top = 120; sc.bottom = -120; sc.near = 10; sc.far = 500;
sun.shadow.bias = -0.0004; sun.shadow.normalBias = 0.35;
scene.add(sun, sun.target);

const camera = new THREE.PerspectiveCamera(40, 1, 1, 2000);
camera.up.set(0, 0, 1);
camera.position.set(-70, -90, 120);
const controls = new OrbitControls(camera, canvas);
controls.target.set(72, 72, 25);
controls.enableDamping = true; controls.dampingFactor = 0.08; controls.maxPolarAngle = Math.PI * 0.499;
const labelRenderer = new CSS2DRenderer({ element: document.getElementById('labels') });

const ground = new THREE.Mesh(new THREE.PlaneGeometry(1600, 1600), new THREE.MeshStandardMaterial({ color: 0x1a2130, roughness: 1 }));
ground.position.set(72, 72, -0.4); ground.receiveShadow = true; scene.add(ground);

function resize() {
  const w = window.innerWidth, h = window.innerHeight;
  renderer.setSize(w, h, false); labelRenderer.setSize(w, h);
  camera.aspect = w / h; camera.updateProjectionMatrix();
}
window.addEventListener('resize', resize); resize();

// ================= load the exported scene =================
const $ = id => document.getElementById(id);
const [sceneJson, buf, colliders] = await Promise.all([
  fetch('data/scene.json').then(r => r.json()),
  fetch('data/scene.bin').then(r => r.arrayBuffer()),
  fetch('data/colliders.json').then(r => r.json()),
]);
const RAIL_MAX_Z = 3;                                 // floor rails lower than this are driven over, not treated as walls
function trimFlower(b, bnd) {                           // a flower is a wall fixture: keep only a thin back plate so the ball stacks in front of it are reachable
  const t = 1.0, o = { ...b };
  if (b.minX <= bnd.minX + 0.3) o.maxX = Math.min(b.maxX, b.minX + t);
  else if (b.maxX >= bnd.maxX - 0.3) o.minX = Math.max(b.minX, b.maxX - t);
  else if (b.minY <= bnd.minY + 0.3) o.maxY = Math.min(b.maxY, b.minY + t);
  else if (b.maxY >= bnd.maxY - 0.3) o.minY = Math.max(b.minY, b.maxY - t);
  return o;
}
const fieldData = { ...colliders, boxes: colliders.boxes.filter(b => b.zMax > RAIL_MAX_Z).map(b => (b.kind === 'flower' ? trimFlower(b, colliders.bounds) : b)) };
const obstacles = buildObstacles(fieldData);
const LUT = new Float32Array(256).map((_, i) => { const c = i / 255; return c <= 0.04045 ? c / 12.92 : Math.pow((c + 0.055) / 1.055, 2.4); });
const geoCache = {};
function geometry(pk, label) {
  const k = pk + label; if (geoCache[k]) return geoCache[k];
  const e = sceneJson.parts[pk][label]; if (!e) return null;
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(buf, e.pos, e.count * 3), 3));
  g.setAttribute('normal', new THREE.BufferAttribute(new Int8Array(buf, e.nrm, e.count * 3), 3, true));
  const c8 = new Uint8Array(buf, e.col, e.count * 4), cf = new Float32Array(e.count * 4);
  for (let i = 0; i < c8.length; i += 4) { cf[i] = LUT[c8[i]]; cf[i + 1] = LUT[c8[i + 1]]; cf[i + 2] = LUT[c8[i + 2]]; cf[i + 3] = c8[i + 3] / 255; }
  g.setAttribute('color', new THREE.BufferAttribute(cf, 4));
  g.computeBoundingSphere();
  return (geoCache[k] = g);
}
const matOpaque = new THREE.MeshStandardMaterial({ vertexColors: true, roughness: 0.55, metalness: 0.08, side: THREE.DoubleSide });
const matTrans = new THREE.MeshStandardMaterial({ vertexColors: true, roughness: 0.12, metalness: 0.0, side: THREE.DoubleSide, transparent: true, depthWrite: false });

// layers: key -> [objects]
const layers = {};
const addLayer = (key, obj) => (layers[key] = layers[key] || []).push(obj);

const fieldGroup = new THREE.Group(); scene.add(fieldGroup);
const hive = {};
const cadState = {};
for (const a of ['red', 'blue']) {
  const g = sceneJson.groups['hive_' + a];
  const grp = new THREE.Group();
  grp.position.set(...g.pivot);
  scene.add(grp);
  hive[a] = { group: grp, pivot: new THREE.Vector3(...g.pivot), state: g.cad_pose_state, shown: g.stable_angles_deg[g.cad_pose_state], target: g.stable_angles_deg[g.cad_pose_state], angles: g.stable_angles_deg, cad: g.cad_pose_state };
  cadState[a] = g.cad_pose_state;
}

{
  const buckets = {};
  for (const inst of sceneJson.instances) {
    const parent = inst.group === 'hive_red' ? 'red' : inst.group === 'hive_blue' ? 'blue' : 'field';
    const k = [inst.part, parent, inst.category].join('|');
    (buckets[k] = buckets[k] || { part: inst.part, parent, category: inst.category, list: [] }).list.push(inst);
  }
  const m = new THREE.Matrix4(), t = new THREE.Matrix4();
  for (const b of Object.values(buckets)) {
    const holder = b.parent === 'field' ? fieldGroup : hive[b.parent].group;
    for (const label of ['opaque', 'trans']) {
      const g = geometry(b.part, label); if (!g) continue;
      const im = new THREE.InstancedMesh(g, label === 'opaque' ? matOpaque : matTrans, b.list.length);
      b.list.forEach((inst, i) => {
        m.set(...inst.m);
        if (b.parent !== 'field') { t.makeTranslation(-hive[b.parent].pivot.x, -hive[b.parent].pivot.y, -hive[b.parent].pivot.z); m.premultiply(t); }
        im.setMatrixAt(i, m);
      });
      im.instanceMatrix.needsUpdate = true;
      im.frustumCulled = false;
      im.castShadow = label === 'opaque' && b.category !== 'tape';
      im.receiveShadow = label === 'opaque';
      holder.add(im);
      const key = b.category.startsWith('scoring_') ? 'balls' : b.category;
      if (key === 'balls') { im.visible = false; continue; }     // static CAD balls are replaced by the physics balls below
      addLayer(key, im);
    }
  }
}

// ================= AprilTags: real 36h11 textures on each tag of the cluster =================
const tagInfo = {};          // id -> {mesh, center(world, current), normal(world), alliance}
const texLoader = new THREE.TextureLoader();
const aprilTags = sceneJson.constants.apriltags;
for (const id of Object.keys(aprilTags)) {
  const t = aprilTags[id];
  const a = t.alliance;
  const st = t.states[cadState[a]];
  const pv = hive[a].pivot;
  const c = new THREE.Vector3(...st.center);
  const n = new THREE.Vector3(...st.normal);
  const corners = st.corners.map(p => new THREE.Vector3(...p));     // TL, TR, BR, BL facing the tag
  const scale = 10 / 8;                                              // include the 1-module quiet zone
  const q = corners.map(p => p.clone().sub(c).multiplyScalar(scale).add(c).addScaledVector(n, 0.04).sub(pv));
  const geo = new THREE.BufferGeometry();
  const [TL, TR, BR, BL] = q;
  const pos = new Float32Array([...TL.toArray(), ...BL.toArray(), ...BR.toArray(), ...TL.toArray(), ...BR.toArray(), ...TR.toArray()]);
  const uv = new Float32Array([0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1, 1]);
  geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  geo.setAttribute('uv', new THREE.BufferAttribute(uv, 2));
  geo.computeVertexNormals();
  const tex = texLoader.load(`textures/tag36h11_${id}.png`);
  tex.colorSpace = THREE.SRGBColorSpace; tex.magFilter = THREE.NearestFilter; tex.minFilter = THREE.LinearMipmapLinearFilter;
  tex.anisotropy = renderer.capabilities.getMaxAnisotropy();
  const mesh = new THREE.Mesh(geo, new THREE.MeshBasicMaterial({ map: tex, side: THREE.DoubleSide }));
  hive[a].group.add(mesh);
  addLayer('tags', mesh);
  tagInfo[id] = { mesh, alliance: a, side: t.side, label: null };
  const div = document.createElement('div');
  div.className = 'lbl ' + a; div.textContent = 'ID ' + id;
  const lab = new CSS2DObject(div); lab.position.copy(c).sub(pv).addScaledVector(n, 2.0);
  hive[a].group.add(lab); addLayer('labels', lab); tagInfo[id].label = lab;
}

// ================= overlays: aim points, opening outlines, zones, axes =================
const overlay = new THREE.Group(); scene.add(overlay);
const aimMarks = {};
for (const a of ['red', 'blue']) {
  const s = new THREE.Mesh(new THREE.SphereGeometry(1.1, 20, 14), new THREE.MeshBasicMaterial({ color: a === 'red' ? 0xff4d4d : 0x4d8dff }));
  const outline = new THREE.LineLoop(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: a === 'red' ? 0xff8a8a : 0x8abaff }));
  overlay.add(s, outline); aimMarks[a] = { s, outline };
  addLayer('aim', s); addLayer('aim', outline);
}
function updateAimMarks() {
  for (const a of ['red', 'blue']) {
    const st = hive[a].state, cells = sceneJson.constants.hives[a].states[st].cells;
    const up = Object.values(cells).find(c => c.up);
    aimMarks[a].s.position.set(...up.aim_point);
    aimMarks[a].outline.geometry.setFromPoints((up.opening_polygon || up.opening_corners).map(p => new THREE.Vector3(...p)));
  }
}
updateAimMarks();

const zoneColors = { red: 0xff5a5a, blue: 0x5a8dff, null: 0xcfd8e8 };
for (const z of sceneJson.tapes.zones) {
  if (!['loading_zone', 'garden', 'alliance_area'].includes(z.kind)) continue;
  const pts = z.polygon.map(p => new THREE.Vector3(p[0], p[1], 0.25));
  const line = new THREE.LineLoop(new THREE.BufferGeometry().setFromPoints(pts), new THREE.LineBasicMaterial({ color: zoneColors[z.alliance] ?? zoneColors.null, transparent: true, opacity: 0.9 }));
  overlay.add(line); addLayer('zones', line);
  const cx = pts.reduce((s, p) => s + p.x, 0) / pts.length, cy = pts.reduce((s, p) => s + p.y, 0) / pts.length;
  const div = document.createElement('div'); div.className = 'lbl zone ' + (z.alliance || ''); div.textContent = z.name;
  const lab = new CSS2DObject(div); lab.position.set(cx, cy, 1.5); overlay.add(lab); addLayer('labels', lab);
}
{
  const ax = new THREE.AxesHelper(20); ax.position.set(0, 0, 0.3); overlay.add(ax); addLayer('axes', ax);
  const edge = new THREE.LineLoop(new THREE.BufferGeometry().setFromPoints([[0, 0], [144, 0], [144, 144], [0, 144]].map(p => new THREE.Vector3(p[0], p[1], 0.2))), new THREE.LineBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.35 }));
  overlay.add(edge); addLayer('axes', edge);
}

// ================= robot model (built from data/robot_spec.json) =================
const flowerArmCfg = await fetch('data/flower_arm.json').then(r => r.json()).catch(() => null);
const flowerArm = new FlowerArm(flowerArmCfg || undefined);
const built = buildRobot(THREE, robotSpec, flowerArmCfg ? { flowerArmCfg } : {});
const robot = built.group; scene.add(robot);
{
  const r = await tryLoadRobotGLB(THREE, 'data/robot.glb', robotSpec, {}).catch(e => ({ model: null, reason: String(e) }));
  if (r && r.model) {
    robot.children.forEach(c => { c.visible = false; });
    robot.add(r.model.group);
    if (built.parts.flowerArm) built.parts.flowerArm.group.visible = true;   // the CAD export has no moving arm: keep the procedural one (built.animate drives it via flowerArmDeg)
    (r.model.warnings || []).forEach(w => diag.toast('robot.glb: ' + w, 'warn'));
    diag.setSimNote('robotmodel', 'robot model: data/robot.glb (CAD export)');
  } else diag.setSimNote('robotmodel', 'robot model: procedural placeholder (drop data/robot.glb to use your CAD)');
}
const turretG = built.turretGroup, hoodPivot = built.hoodPivot, flywheelMesh = built.flywheelMesh;
const frustumLines = new THREE.LineSegments(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0x66ffcc, transparent: true, opacity: 0.55 }));
scene.add(frustumLines); addLayer('frustum', frustumLines);
const trajLine = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0xffa21a }));
scene.add(trajLine); addLayer('traj', trajLine);
const landMark = new THREE.Mesh(new THREE.RingGeometry(1.6, 2.2, 28), new THREE.MeshBasicMaterial({ color: 0xffa21a, side: THREE.DoubleSide }));
scene.add(landMark); addLayer('traj', landMark);
addLayer('robot', robot);

// ================= state =================
const S = {
  x: START.x, y: START.y, h: START.h, vx: 0, vy: 0, om: 0, bodyV: [0, 0, 0],
  turretDeg: 0, hoodDeg: 45, rpm: 0, spin: 0,
  alliance: 'blue', autoAim: true, noise: false,
  ballKind: 'pollen',   // kind of the next shot: 'pollen' | 'nectar' (not persisted)
  linked: false, shots: 0, scored: 0, cam: 'iso', extTs: 0,
  sol: null,
};
const keys = new Set();

// ---- physics world: balls, robot-vs-field collisions, self-tipping hives ----
const worldEnv = { bounds: fieldData.bounds, boxes: fieldData.boxes, circles: fieldData.circles, hives: {} };
const hiveDyn = {};
for (const a of ['red', 'blue']) {
  const g = sceneJson.groups['hive_' + a];
  worldEnv.hives[a] = { pivot: g.pivot, cadAngle: g.cad_pose_angle_deg, cells: sceneJson.constants.hives[a].states[g.cad_pose_state].cells };
  hiveDyn[a] = new HiveDynamics(g, sceneJson.constants.hives[a].states, { ...HIVE_PARAMS });   // official rule: balls stay in the cell that becomes the down cell (manual 10.5); tip count comes only from HIVE_PARAMS (ballsToTip = 8 pollen, user fact; no UI control sets it)
}
// real hive triangle mesh (from the CAD) so balls bounce off the hive body and score only through the openings
try {
  const hiveMeshes = parseHiveCollider(await (await fetch('data/hive_collider.bin')).arrayBuffer());
  for (const a of ['red', 'blue']) if (hiveMeshes[a]) worldEnv.hives[a].mesh = hiveMeshes[a];
} catch (e) { diag.toast('hive collider not loaded, using the idealised mesh: ' + e.message, 'warn'); }
const world = new BallWorld(worldEnv);
world.reset(sceneJson.balls.balls);
// a ball physically released/recycled out of a hive cell must also leave the hive model, or cycleStats()/HUD keep counting it (hive-physics patch)
world.onRelease = world.onRecycle = (b, alliance, cell) => { try { hiveDyn[alliance]?.forgetBall(cell, 1); } catch (_) { /* model has no record */ } };
const S_PRACTICE = { ammo: 0, ammoCap: RULES.holdLimit, ammoLimit: false, intake: true, fieldCentric: false, headingOffset: 0, look: { yawDeg: 0, pitchDeg: 0 }, prevToggle: false };
const scorer = new MatchScorer(RULES), stats = new PracticeStats(), matchClock = new MatchClock(), drive = new MecanumDrive({ maxSpeed: DRIVE.maxSpeed, maxAccel: DRIVE.maxAccel, maxTurn: DRIVE.maxTurn });
const ballGroup = new THREE.Group(); scene.add(ballGroup); addLayer('balls', ballGroup);
const ballMeshes = new Map();
const ballGeoPool = { r: new THREE.SphereGeometry(BALL.r, 18, 12), n: new THREE.SphereGeometry(BALL.nectarR, 18, 12) };
const ballColor = { pollen: 0xffd23a, nectar: 0xff9a3d, nectar_red: 0xff5a5a, nectar_blue: 0x5a8dff };   // 'nectar' = a shot nectar ball (no alliance)
const ballMats = {}; for (const [k, c] of Object.entries(ballColor)) ballMats[k] = new THREE.MeshStandardMaterial({ color: c, roughness: 0.5 });
function syncBallMeshes() {
  const seen = new Set();
  for (const b of world.balls) {
    seen.add(b.id);
    let m = ballMeshes.get(b.id);
    if (!m) { m = new THREE.Mesh(/nectar/.test(b.type) ? ballGeoPool.n : ballGeoPool.r, ballMats[b.type] || ballMats.pollen); m.castShadow = true; m.receiveShadow = true; ballGroup.add(m); ballMeshes.set(b.id, m); }
    m.visible = b.state !== 'held';
    m.position.set(b.x, b.y, b.z);
  }
  for (const [id, m] of ballMeshes) if (!seen.has(id)) { ballGroup.remove(m); ballMeshes.delete(id); }
}

function toast(text, cls = '') { diag.toast(text, cls || 'info'); }
const deg = r => r * 180 / Math.PI, rad = d => d * Math.PI / 180;
const wrap = a => Core.wrap(a);

// ================= physics layer: setpoints from the ballistic solver (same maths as ShotSolver.java) =================
function aimTarget() {
  const a = S.alliance, st = hive[a].state;
  const cells = sceneJson.constants.hives[a].states[st].cells;
  const up = Object.values(cells).find(c => c.up);
  return { x: up.aim_point[0], y: up.aim_point[1], z: up.aim_point[2], normal: up.opening_normal, name: st };
}
function solveShot() {
  const tg = aimTarget();
  const p = { x: S.x, y: S.y, psi: S.h, vx: S.vx, vy: S.vy, om: S.om, rx: SHOOTER.pivotX, ry: SHOOTER.pivotY, z0: SHOOTER.exitHeight, X: tg.x, Y: tg.y, Z: tg.z };
  let r = Core.solve(p, 'gamma', rad(SHOOTER.entryDeg), 'low');
  let clamped = false;
  if (r.ok) {
    const e = deg(r.elev), c = Math.min(SHOOTER.hoodMax, Math.max(SHOOTER.hoodMin, e));
    if (c !== e) { clamped = true; const r2 = Core.solve(p, 'theta', rad(c), 'low'); if (r2.ok) r = r2; }
  }
  r.p = p; r.clamped = clamped; r.target = tg;
  r.rpm = r.ok ? Core.rpmEstimate(r.speed, SHOOTER.wheelR, SHOOTER.eff) * SHOOTER.rpmScale : 0;
  r.entry = r.ok && r.fwd ? deg(r.fwd.entry) : NaN;
  r.turretDeg = r.ok ? deg(r.turret) : 0;
  if (r.ok) { const n = tg.normal, vg = [r.vs[0] + r.uh[0], r.vs[1] + r.uh[1], r.uz - G * r.T]; r.fromBehind = n[0] * vg[0] + n[1] * vg[1] + n[2] * vg[2] >= 0; }
  return r;
}

// ================= shooter mechanism + aim modes (auto / assist / manual / free) =================
const plantSpec = { turretRateDegS: PLANT.turretRateDegS, turretKp: PLANT.turretKp, turretMinDeg: SHOOTER.turretMinDeg, turretMaxDeg: SHOOTER.turretMaxDeg, hoodMinDeg: SHOOTER.hoodMin, hoodMaxDeg: SHOOTER.hoodMax, flywheelTauS: PLANT.flywheelTauS, maxRpm: SHOOTER.maxRpm, rpmDipFrac: PLANT.rpmDipFrac };
const plant = new ShooterPlant(plantSpec); plant.state.hoodDeg = S.hoodDeg;
const manualAim = new ManualAim({ turretMinDeg: plantSpec.turretMinDeg, turretMaxDeg: plantSpec.turretMaxDeg, hoodMinDeg: plantSpec.hoodMinDeg, hoodMaxDeg: plantSpec.hoodMaxDeg, maxRpm: plantSpec.maxRpm });
const S_AIM = { mode: 'auto', enforceZone: true, zone: new LegalZone({ preset: 'open' }), lastCmd: null, label: 'auto', showRegion: false, regionKey: '', region: null };
const hiveTall = fieldData.boxes.filter(b => b.kind === 'hive_frame').map(b => ({ ...b, zMax: 80 }));   // hive legs are solid posts up to the hive body
function aimSol(sol) { return sol && sol.ok ? { ok: true, turretDeg: sol.turretDeg, elevDeg: deg(sol.elev), rpm: sol.rpm } : { ok: false }; }
function setAimMode(m) {
  S_AIM.mode = m; S_AIM.lastCmd = null;
  Object.assign(manualAim.state, { turretDeg: plant.state.turretDeg, hoodDeg: plant.state.hoodDeg, rpm: plant.state.rpm });
  const d = describeMode(m); $('aim-mode-desc').textContent = d.summary; toast('Aim mode: ' + d.label, 'ok');
}
function plantStep(dt, sol) {
  const pads = navigator.getGamepads ? navigator.getGamepads() : [];
  let input = aimInputFromKeys(keys);
  const gp = readGamepad(pads);
  if (gp.connected && gp.aux.rb) {
    const gi = aimInputFromGamepad(pads[0]);
    input = { ...input, turretAxis: input.turretAxis || gi.turretAxis, hoodAxis: input.hoodAxis || gi.hoodAxis, rpmAxis: input.rpmAxis || gi.rpmAxis, fine: input.fine || gi.fine, snapToAuto: input.snapToAuto || gi.snapToAuto, rpmPreset: input.rpmPreset || gi.rpmPreset };
    S_PRACTICE.aimPad = true;
  } else S_PRACTICE.aimPad = false;
  const solA = aimSol(sol);
  if (S_AIM.mode !== 'auto') manualAim.update(dt, { ...input, sol: solA });
  const cmd = modeCommand(S_AIM.mode, solA, manualAim, plantSpec, S_AIM.lastCmd);
  S_AIM.lastCmd = cmd; S_AIM.label = cmd.label;
  plant.step(dt, cmd);
  S.turretDeg = plant.state.turretDeg; S.hoodDeg = plant.state.hoodDeg; S.rpm = plant.state.rpm;
  if (sol && sol.ok) sol.turretCmd = cmd.turretDeg ?? sol.turretDeg;
}
function nextShotKind() { const h = world.held && world.held[0]; const k = h ? h.kind : S.ballKind; return k === "nectar" ? "nectar" : "pollen"; }   // oldest held ball fires first (FIFO); the selector applies when nothing is held
function evalCtx() {
  const tg = aimTarget();
  return { kind: nextShotKind(), mode: S_AIM.mode, pose: { x: S.x, y: S.y, h: S.h, vx: S.vx, vy: S.vy, om: S.om }, turretDeg: S.turretDeg, hoodDeg: S.hoodDeg, rpm: S.rpm, sol: S.sol,
    target: { x: tg.x, y: tg.y, z: tg.z, normal: tg.normal }, shooter: SHOOTER, ammo: { limited: S_PRACTICE.ammoLimit, count: S_PRACTICE.ammo },
    obstacles: fieldData, hiveBoxes: hiveTall, legalZone: S_AIM.zone, enforceZone: S_AIM.enforceZone, alliance: S.alliance, core: Core };
}

// ================= drive: mecanum model + real collisions (stand-in for Pedro; replaced when linked) =================
function currentCmd() {
  const gp = readGamepad(navigator.getGamepads ? navigator.getGamepads() : []);
  const kb = mapKeyboard(keys);
  let cmd = kb;
  if (gp.connected && (Math.abs(gp.fwd) + Math.abs(gp.strafe) + Math.abs(gp.turn) > 0 || gp.shoot || gp.aux.a)) cmd = gp;
  else if (gp.connected) cmd = { ...kb, fwd: kb.fwd, strafe: kb.strafe, turn: kb.turn };
  if (!(cmd === gp)) { const f = cmd.aux.lb ? 1 : 0.6; cmd = { ...cmd, fwd: cmd.fwd * f, strafe: cmd.strafe * f, turn: cmd.turn * f }; }
  const tog = !!(gp.connected && gp.fieldCentricToggle) || keys.has('f');
  if (tog && !S_PRACTICE.prevToggle) { S_PRACTICE.fieldCentric = !S_PRACTICE.fieldCentric; if (S_PRACTICE.fieldCentric) S_PRACTICE.headingOffset = 0; toast('Field-centric drive: ' + (S_PRACTICE.fieldCentric ? 'ON' : 'OFF')); }
  S_PRACTICE.prevToggle = tog;
  cmd.gpShoot = gp.connected && gp.shoot;
  return cmd;
}
function driveStep(dt) {
  const cmd = currentCmd();
  if (matchClock.phase === 'over') { cmd.fwd = cmd.strafe = cmd.turn = 0; }
  const prev = { x: S.x, y: S.y, h: S.h };
  const v = drive.step(cmd, dt, S, { fieldCentric: S_PRACTICE.fieldCentric, headingOffset: S_PRACTICE.headingOffset, alliance: S.alliance });
  S.vx = v.vx; S.vy = v.vy; S.om = v.om;
  const want = { x: S.x + S.vx * dt, y: S.y + S.vy * dt, h: wrap(S.h + S.om * dt) };
  const r = resolveRobot(want, prev, obstacles);
  S.x = r.x; S.y = r.y; S.h = r.h;
  if (r.hit) {
    const sv = slideVelocity(S.vx, S.vy, r.normals);
    S.vx = sv.vx; S.vy = sv.vy;
    const c = Math.cos(S.h), s = Math.sin(S.h);                         // back into the body frame so the robot does not "remember" pushing into the wall
    S.bodyV[0] = c * S.vx + s * S.vy; S.bodyV[1] = -s * S.vx + c * S.vy;
    S.bump = (S.bump || 0) + 1;
  }
  if (cmd.shoot && !S_PRACTICE.shootHeld) shoot();
  S_PRACTICE.shootHeld = cmd.shoot;
}
function placeRobot(x, y) {
  const r = resolveRobot({ x, y, h: S.h }, { x: S.x, y: S.y, h: S.h }, obstacles);
  S.x = r.x; S.y = r.y; S.bodyV = [0, 0, 0]; S.vx = S.vy = 0;
}

// ================= shooting: a real physics ball leaves the shooter; the hive opening captures it =================
function shoot() {
  const ev = S.eval || evaluateShot(evalCtx());
  if (S_PRACTICE.ammoLimit && S_PRACTICE.ammo <= 0) { diag.toast('Hopper empty: drive over balls with the intake on (I)', 'info', { id: 'hopper-empty' }); return; }
  // shots ALWAYS fire: the ball leaves from the real turret/hood/flywheel state and the physics decides whether it scores
  const f = forwardShot(evalCtx());               // the ball leaves from the REAL turret / hood / rpm, not from the solver
  const n = S.noise ? 1 + (Math.random() - 0.5) * 0.03 : 1;
  const vel = [f.vel[0] * n, f.vel[1] * n, f.vel[2] * n];
  const kind = f.kind;   // oldest held ball's kind, else the selector; balls.js addBall(type, pos, vel, kind) keeps it
  const ball = world.fireHeld(f.exit, vel) || world.addBall(kind, f.exit, vel, kind);   // a real hold slot is freed so intake can refill
  ball.shot = true; ball.shotAge = 0;
  if (S_PRACTICE.ammoLimit) S_PRACTICE.ammo--;
  plant.kick(); S.rpm = plant.state.rpm;
  stats.record('shot'); S.shots++;
}
world.onIntake = (ball, side) => { S_PRACTICE.ammo++; diag.toast('Intake (' + side + '): ' + S_PRACTICE.ammo + '/' + S_PRACTICE.ammoCap, 'ok', { id: 'intake' }); };
world.onCapture = (ball, alliance, cell) => {
  hiveDyn[alliance].addScoredBall(cell, [ball.x, ball.y, ball.z], /nectar/.test(ball.type) ? 'nectar' : 'pollen');
  if (ball.shot) {
    ball.shot = false; stats.record('scored'); S.scored++;
    toast(alliance === S.alliance ? 'SCORED' : 'SCORED in the ' + alliance.toUpperCase() + ' hive', alliance === S.alliance ? 'ok' : 'warn');
  }
};
for (const a of ['red', 'blue']) {
  hiveDyn[a].onTip = (state, count) => {
    hive[a].state = state; updateAimMarks(); refreshTipButtons();
        scorer.onTip(a, matchPhase());
    toast(a.toUpperCase() + ' hive tipped (' + count + ' balls scored)', 'ok');
  };
}
function updateBalls(dt) {
  // intake: flared funnel + roller pull at the front and rear mouths (balls.js); capacity = free hopper slots
  world.setIntake({ enabled: S_PRACTICE.intake, space: S_PRACTICE.ammoLimit ? Math.max(0, S_PRACTICE.ammoCap - S_PRACTICE.ammo) : Infinity });
  world.setRobot({ x: S.x, y: S.y, h: S.h, vx: S.vx, vy: S.vy, om: S.om }, { halfX: ROBOT.halfX, halfY: ROBOT.halfY, height: 18 });
  world.setHiveAngles({ red: hive.red.shown, blue: hive.blue.shown });
  world.step(dt);
  for (const b of world.balls) if (b.shot) {
    b.shotAge += dt;
    if (b.state === 'free' && (b.sleeping || b.shotAge > 8)) { b.shot = false; stats.record('miss'); toast('MISS: the ball did not enter a hive', 'bad'); }
  }
  syncBallMeshes();
}


// ================= camera on the turret: frustum, visible tags, tx =================
function cameraPose() {
  const yaw = S.h + rad(S.turretDeg), pitch = rad(CAMERA.pitchDeg), c = Math.cos(S.h), s = Math.sin(S.h);
  const lx = SHOOTER.pivotX + CAMERA.x * Math.cos(rad(S.turretDeg)), ly = SHOOTER.pivotY + CAMERA.x * Math.sin(rad(S.turretDeg));
  const pos = new THREE.Vector3(S.x + c * lx - s * ly, S.y + s * lx + c * ly, CAMERA.z);
  const f = new THREE.Vector3(Math.cos(yaw) * Math.cos(pitch), Math.sin(yaw) * Math.cos(pitch), Math.sin(pitch));
  const l = new THREE.Vector3(-Math.sin(yaw), Math.cos(yaw), 0);
  const u = new THREE.Vector3().crossVectors(f, l);
  return { pos, f, l, u };
}
function updateCameraAndTags() {
  const cp = cameraPose();
  // frustum drawing
  const hx = Math.tan(rad(CAMERA.hfovDeg / 2)), vy = Math.tan(rad(CAMERA.vfovDeg / 2)), R = 36;
  const corner = (a, b) => cp.pos.clone().addScaledVector(cp.f, R).addScaledVector(cp.l, a * hx * R).addScaledVector(cp.u, b * vy * R);
  const cs = [corner(1, 1), corner(-1, 1), corner(-1, -1), corner(1, -1)];
  const pts = []; cs.forEach((p, i) => { pts.push(cp.pos, p, p, cs[(i + 1) % 4]); });
  frustumLines.geometry.setFromPoints(pts);
  // which tags does it see? tag world centres follow the hive tilt
  const vis = [], tx = [];
  for (const id of Object.keys(tagInfo)) {
    const ti = tagInfo[id], st = hive[ti.alliance];
    const a = rad(st.shown - st.angles[cadState[ti.alliance]]), cs_ = Math.cos(a), sn = Math.sin(a);
    const base = aprilTags[id].states[cadState[ti.alliance]];
    const rel = new THREE.Vector3(...base.center).sub(st.pivot), nrm = new THREE.Vector3(...base.normal);
    const rot = v => new THREE.Vector3(v.x, v.y * cs_ - v.z * sn, v.y * sn + v.z * cs_);
    const cw = rot(rel).add(st.pivot), nw = rot(nrm);
    ti.centerWorld = cw; ti.normalWorld = nw;
    const d = cw.clone().sub(cp.pos), x = d.dot(cp.f);
    if (x <= 3 || d.length() > CAMERA.rangeIn) { ti.visible = false; continue; }
    const az = Math.atan2(d.dot(cp.l), x), el = Math.atan2(d.dot(cp.u), x);
    const facing = nw.dot(d.clone().negate().normalize()) > 0.15;
    ti.visible = Math.abs(az) <= rad(CAMERA.hfovDeg / 2) && Math.abs(el) <= rad(CAMERA.vfovDeg / 2) && facing;
    if (ti.visible) { vis.push(id); if (ti.alliance === S.alliance) tx.push(-deg(az)); }
  }
  S.visibleTags = vis;
  S.tx = tx.length ? tx.reduce((a, b) => a + b, 0) / tx.length : NaN;
}

// ================= UI wiring =================
function buildUi() {
  const defs = [['tile', 'Tiles'], ['perimeter_wall', 'Walls'], ['perimeter_pin', 'Wall pins'], ['hive_frame', 'Hive frame'], ['hive_red', 'Red hive'], ['hive_blue', 'Blue hive'],
    ['apriltag_sticker', 'Tag stickers'], ['tags', 'AprilTags (36h11)'], ['flower', 'Flowers'], ['tape', 'Tape'], ['balls', 'Pollen / nectar'], ['artifact_tray', 'Trays'],
    ['hardware', 'Hardware'], ['decoration', 'Decoration'], ['zones', 'Zones'], ['aim', 'Aim points'], ['labels', 'Labels'], ['traj', 'Trajectory'], ['frustum', 'Camera frustum'], ['robot', 'Robot'], ['axes', 'Axes / outline']];
  const box = $('layers');
  for (const [key, name] of defs) {
    if (!layers[key]) continue;
    const l = document.createElement('label'); l.className = 't';
    const c = document.createElement('input'); c.type = 'checkbox'; c.checked = !['hardware', 'axes', 'perimeter_pin', 'labels'].includes(key); c.id = 'layer-' + key;
    c.addEventListener('change', () => setLayer(key, c.checked));
    l.append(c, name); box.append(l); setLayer(key, c.checked);
  }
}
function setLayer(key, on) { (layers[key] || []).forEach(o => (o.visible = on)); }
const stateName = s => (s === 'far_up' ? 'Far cell up' : 'Audience cell up');
function tip(a) {
  if (hiveDyn[a].moving) return;                         // a tip in progress (natural or manual) is not interrupted, so its onTip/score is not lost
  const h = hive[a], next = h.state === 'far_up' ? 'audience_up' : 'far_up';
  h.state = next; h.target = h.angles[next]; hiveDyn[a].setState(next, true); updateAimMarks(); refreshTipButtons(); toast(a.toUpperCase() + ' hive: ' + stateName(next));
}
function refreshTipButtons() { $('tip-red').textContent = stateName(hive.red.state); $('tip-blue').textContent = stateName(hive.blue.state); }
$('tip-red').onclick = () => tip('red'); $('tip-blue').onclick = () => tip('blue');
$('alliance').onchange = e => { S.alliance = e.target.value; };
$('opt-shadow').onchange = e => { renderer.shadowMap.enabled = e.target.checked; scene.traverse(o => { if (o.material) o.material.needsUpdate = true; }); sun.castShadow = e.target.checked; };
$('opt-noise').onchange = e => (S.noise = e.target.checked);
$('aim-mode').onchange = e => setAimMode(e.target.value);
// Ball kind for the next shot (pollen 2.8 in / nectar 3.6 in). Not persisted: every page load starts on pollen.
// Held balls keep the kind they were loaded with (balls.js fireHeld); this selector only sets new shots.
function setBallKind(k, quiet) {
  S.ballKind = k === 'nectar' ? 'nectar' : 'pollen';
  $('ball-kind').value = S.ballKind;
  $('kind-pollen').classList.toggle('on', S.ballKind === 'pollen');
  $('kind-nectar').classList.toggle('on', S.ballKind === 'nectar');
  if (!quiet) toast('Next shot: ' + (S.ballKind === 'nectar' ? 'nectar' : 'pollen'), 'ok');
}
$('kind-pollen').onclick = () => setBallKind('pollen');
$('kind-nectar').onclick = () => setBallKind('nectar');
$('ball-kind').onchange = e => setBallKind(e.target.value);
setBallKind('pollen', true);
for (const p of LegalZone.presets()) { if (p.id === 'custom') continue; const o = document.createElement('option'); o.value = p.id; o.textContent = p.label; o.title = p.description; $('zone-preset').append(o); }
$('zone-preset').onchange = e => { S_AIM.zone = new LegalZone({ preset: e.target.value }); $('zone-desc').textContent = S_AIM.zone.describe(); };
$('opt-zone').onchange = e => { S_AIM.enforceZone = e.target.checked; };
$('opt-region').onchange = e => { S_AIM.showRegion = e.target.checked; };
$('zone-desc').textContent = S_AIM.zone.describe(); $('aim-mode-desc').textContent = describeMode('auto').summary;
$('btn-reset').onclick = () => resetAllState(); $('btn-clear').onclick = () => { world.reset([]); flowerArm.reset(); S_PRACTICE.ammo = 0; for (const a of ['red', 'blue']) hiveDyn[a].reset(); syncBallMeshes(); toast('Field cleared: every ball removed', 'ok'); };
$('btn-refill').onclick = () => { resetAllState(); toast('Field reset to the starting balls', 'ok'); };
// Full field reset: starting balls, hives back to their CAD pose, flower arm stowed, hopper empty, queued auto shots dropped.
// Balls are passed explicitly so a Clear (world.reset([])) cannot leave the field without its starting balls.
function resetField() {
  if (typeof world.resetField === 'function') world.resetField({ hiveDyn, balls: sceneJson.balls.balls }); else { world.reset(sceneJson.balls.balls); for (const a of ['red', 'blue']) hiveDyn[a].reset(); }
  flowerArm.reset(); S_PRACTICE.ammo = 0; S_AUTO.shots = 0; S_AUTO.cool = 0;
  for (const a of ['red', 'blue']) { const st = hiveDyn[a].state, h = hive[a]; h.state = st; h.target = h.angles[st]; h.shown = h.angles[st]; }
  updateAimMarks(); refreshTipButtons(); syncBallMeshes(); refreshHiveHud();
}
function resetRobot() { Object.assign(S, { x: START.x, y: START.y, h: START.h, vx: 0, vy: 0, om: 0, bodyV: [0, 0, 0], turretDeg: 0, rpm: 0 }); Object.assign(plant.state, { turretDeg: 0, rpm: 0 }); S_PRACTICE.headingOffset = 0; MAP.trail.length = 0; }
// Shared reset for Stop, Restart, the match Reset, Reset field and Reset robot. Order matters: stop the auto first (its reset
// restores the field through autoHost), then the field, hives, next-shot kind and robot pose.
function stopAutoRun() { if (autoActive()) autoPlanner.stop(); }
function resetAllState() {
  stopAutoRun();
  resetField();
  resetHiveDynamics();
  resetBallKind();
  resetRobot();
}
// Every HiveDynamics back to its CAD start state; the hive meshes, aim marks, tip buttons and HUD follow it.
function resetHiveDynamics() {
  for (const a of ['red', 'blue']) {
    const d = hiveDyn[a], h = hive[a]; if (!d) continue;
    if (typeof d.reset === 'function') d.reset();
    if (h && d.state) { h.state = d.state; h.target = h.angles[d.state]; h.shown = h.angles[d.state]; }
  }
  updateAimMarks(); refreshTipButtons(); syncBallMeshes(); refreshHiveHud();
}
function refreshHiveHud() { if (typeof updateHiveHud === 'function') updateHiveHud(performance.now()); }
// Next-shot kind goes back to pollen on every reset (held balls are cleared by the field reset and keep their own kind).
function resetBallKind() { setBallKind('pollen', true); }
document.querySelectorAll('[data-cam]').forEach(b => (b.onclick = () => setCam(b.dataset.cam)));
const FIRST_PERSON = new Set(['pov', 'driver', 'driver_wide', 'chase', 'coach']);
function setCam(name) {
  S.cam = name; controls.enabled = !FIRST_PERSON.has(name); if (name !== 'pov') { S_PRACTICE.look = { yawDeg: 0, pitchDeg: 0 }; }
  const P = (x, y, z, tx, ty, tz) => { camera.position.set(x, y, z); controls.target.set(tx, ty, tz); camera.fov = 40; camera.updateProjectionMatrix(); };
  if (name === 'iso') P(-60, -85, 115, 72, 72, 25);
  if (name === 'top') P(72, 71.9, 280, 72, 72, 0);
  if (name === 'audience') P(72, -120, 70, 72, 72, 35);
  if (name === 'side') P(260, 72, 60, 72, 72, 38);
}
const isTyping = t => { const tag = String(t?.tagName || '').toUpperCase(); return tag === 'TEXTAREA' || tag === 'SELECT' || !!t?.isContentEditable || (tag === 'INPUT' && !['checkbox', 'range', 'button', 'radio'].includes(String(t.type))); };
window.addEventListener('keydown', e => {
  const k = String(e.key || "").toLowerCase(); if (!k || isTyping(e.target)) return;
  if ((S.view === 'path' || pathEditor.editing) && pathEditor.key(e)) { e.preventDefault(); return; }
  if (S.view !== '3d' || e.ctrlKey || e.metaKey || e.altKey) return;       // the planner / Pedro tabs own the keyboard; 3D hotkeys must not act behind them
  keys.add(k);
  const once = !e.repeat, free = !autoActive();   // toggles fire once per press; a running or paused auto owns the robot, so field-changing hotkeys wait
  if (k === ' ') { e.preventDefault(); if (once && free) shoot(); }
  if (k.startsWith('arrow') || k === '[' || k === ']') e.preventDefault();
  if (once && free) { if (k === 't') tip('red'); if (k === 'y') tip('blue'); if (k === 'r') resetRobot(); if (k === 'g') flowerArm.toggle(); }
  if (k === 'l' && once) { const on = !(layers.labels?.[0]?.visible); setLayer('labels', on); const cb = $('layer-labels'); if (cb) cb.checked = on; }
  const cams = { 1: 'iso', 2: 'top', 3: 'audience', 4: 'side', 5: 'follow', 6: 'pov', 7: 'driver', 8: 'driver_wide', 9: 'chase', 0: 'coach' }; if (cams[k] && once) setCam(cams[k]);
  if (k === 'i' && once && free) { S_PRACTICE.intake = !S_PRACTICE.intake; $('opt-intake').checked = S_PRACTICE.intake; toast('Intake ' + (S_PRACTICE.intake ? 'ON' : 'OFF')); }
});
window.addEventListener('keyup', e => keys.delete(String(e.key || "").toLowerCase()));
window.addEventListener('blur', () => keys.clear());
window.addEventListener('pointermove', e => {
  if (!(e.buttons & 1) || !['driver', 'driver_wide', 'coach', 'chase'].includes(S.cam) || e.target !== canvas) return;
  const L = S_PRACTICE.look; L.yawDeg = Math.max(-70, Math.min(70, L.yawDeg - e.movementX * 0.15)); L.pitchDeg = Math.max(-60, Math.min(40, L.pitchDeg - e.movementY * 0.15));
});
// click the floor to place the robot
{
  const ray = new THREE.Raycaster(), plane = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0); let down = null;
  canvas.addEventListener('pointerdown', e => (down = [e.clientX, e.clientY, e.button]));
  canvas.addEventListener('pointerup', e => {
    if (!down || down[2] !== 0 || Math.hypot(e.clientX - down[0], e.clientY - down[1]) > 4 || S.linked || autoActive()) return;
    ray.setFromCamera(new THREE.Vector2(e.clientX / innerWidth * 2 - 1, -(e.clientY / innerHeight) * 2 + 1), camera);
    const hit = new THREE.Vector3(); if (ray.ray.intersectPlane(plane, hit)) { placeRobot(hit.x, hit.y); }
  });
}
buildUi(); refreshTipButtons();
$('m-start').onclick = () => { scorer.reset(); S_PRACTICE.finalized = false; stats.reset(); matchClock.reset(); matchClock.start(); S_PRACTICE.ammo = Math.min(RULES.preloadPollen, S_PRACTICE.ammoCap); toast('Match started: preloaded ' + S_PRACTICE.ammo + ' pollen', 'ok'); };
$('m-pause').onclick = () => matchClock.pause();
$('m-reset').onclick = () => { matchClock.reset(); scorer.reset(); S_PRACTICE.finalized = false; stats.reset(); S.shots = 0; S.scored = 0; resetAllState(); };
$('opt-intake').onchange = e => (S_PRACTICE.intake = e.target.checked);
{
  const sp = $('opt-speed'), ac = $('opt-accel');
  sp.value = DRIVE.maxSpeed; ac.value = DRIVE.maxAccel; $('v-speedset').textContent = DRIVE.maxSpeed + ' in/s'; $('v-accelset').textContent = DRIVE.maxAccel + ' in/s\u00b2';
  sp.oninput = () => { drive.maxSpeed = +sp.value; $('v-speedset').textContent = sp.value + ' in/s'; };
  ac.oninput = () => { drive.maxAccel = +ac.value; drive.maxDecel = +ac.value * 1.6; drive.turnAccel = drive.maxTurn * 6; $('v-accelset').textContent = ac.value + ' in/s\u00b2'; };
}
$('opt-ammo').onchange = e => (S_PRACTICE.ammoLimit = e.target.checked);
$('opt-fc').onchange = e => { S_PRACTICE.fieldCentric = e.target.checked; S_PRACTICE.headingOffset = 0; };

// ================= link to a real robot / Pedro (see README: POST /api/state) =================
async function poll() {
  try {
    const j = await (await fetch('/api/state', { cache: 'no-store' })).json();
    const fresh = j.t && Date.now() / 1000 - j.t < 1.0;
    if (j.alliance && j.alliance !== S.alliance) { S.alliance = j.alliance; $('alliance').value = j.alliance; }
    if (j.tip) for (const a of ['red', 'blue']) if (j.tip[a] && j.tip[a] !== hive[a].state) { hive[a].state = j.tip[a]; hive[a].target = hive[a].angles[j.tip[a]]; hiveDyn[a].setState(j.tip[a], true); updateAimMarks(); refreshTipButtons(); }
    S.pollOk = true; S.linked = !!(fresh && j.pose);
    if (S.linked) S.ext = j; S.extTs = j.t;
    if (j.path && JSON.stringify(j.path) !== S.pathKey) { S.pathKey = JSON.stringify(j.path); S.path = j.path; drawPath(j.path); }
  } catch (e) { S.linked = false; S.pollOk = false; }
}
(function pollLoop() { poll().finally(() => setTimeout(pollLoop, S.pollOk ? 80 : 1000)); })();
let pathLine = null;
function drawPath(pts) {
  if (pathLine) { scene.remove(pathLine); pathLine = null; }
  if (!pts.length) return;
  pathLine = new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts.map(p => new THREE.Vector3(p[0], p[1], 0.5))), new THREE.LineBasicMaterial({ color: 0x40ffa0 }));
  scene.add(pathLine);
}

const pathEditor = createPathEditor({ mx, my, getK: () => MAP.k, toast, getObstacles: () => fieldData.boxes.filter(b => b.zMax > 3) });
$('pathpanel').innerHTML = pathEditor.panelHtml(); pathEditor.bindPanel($('pathpanel'));
// ================= auto planner (PathPlanner-style command editor) and the host that lets it drive the sim robot =================
const S_AUTO = { shots: 0, cool: 0, time: 0, savedLimit: null, transfer: false };
const autoHost = {
  // start() / seek() / Stop go through snapshot + restore: every run begins on a freshly reset field with the 4-pollen preload.
  snapshot: () => { prepareAutoRun(); return { auto: true }; },
  resetHives: () => resetHiveDynamics(),   // AutoRunner.reset() hook: Stop / Restart put every HiveDynamics back to its start state
  restore: () => { prepareAutoRun(); const st = autoPlanner.auto && autoPlanner.auto.start; if (st) { S.x = st.x; S.y = st.y; S.h = st.h || 0; } },
  getPose: () => ({ x: S.x, y: S.y, h: S.h }),
  setPose: pz => { S.x = pz.x; S.y = pz.y; S.h = pz.h; },
  setVelocity: (vx, vy, om) => { S.vx = vx; S.vy = vy; S.om = om; S.bodyV = [0, 0, 0]; },
  drive: () => ({ vmax: drive.maxSpeed, aMax: drive.maxAccel }),
  time: () => S_AUTO.time,
  onLog: m => diag.log('auto: ' + m),
  intake: on => { S_PRACTICE.intake = !!on; $('opt-intake').checked = !!on; return true; },
  // queues shots; autoAim{shootAfter} calls shoot({all:true}) and autoShots() fires while the shot is allowed
  shoot: args => { S_AUTO.shots = args && args.all ? Infinity : Math.max(1, (args && args.count) || 1); return true; },
  // turns the transfer on/off; the sim has no transfer gate yet, so the state is recorded and logged only
  transfer: on => { S_AUTO.transfer = !!on; diag.log('auto: transfer ' + (on ? 'on' : 'off') + ' (recorded, no sim gate)'); return true; },
  // setAim only forces auto aim mode (the frame loop drives the plant toward S_AIM.lastCmd)
  setAim: on => { if (on && S_AIM.mode !== 'auto') { $('aim-mode').value = 'auto'; setAimMode('auto'); } return true; },
  setRpm: r => { manualAim.state.rpm = r; return true; },
  flower: deploy => { if (deploy) flowerArm.deploy(); else flowerArm.stow(); return true; },
  gate: open => { diag.log('auto: gate ' + (open ? 'open' : 'close') + ' (not simulated)'); return true; },
  park: () => { diag.log('auto: park (not simulated)'); return true; },
  ballCount: () => S_PRACTICE.ammo,
  hopperFull: () => S_PRACTICE.ammo >= S_PRACTICE.ammoCap,
  hopperEmpty: () => S_PRACTICE.ammo <= 0,
  flywheelReady: () => { const c = S_AIM.lastCmd; return !!c && c.rpm > 300 && Math.abs(S.rpm - c.rpm) / Math.max(c.rpm, 1) < 0.04; },
  // aimed: turret within 2 deg and hood within 1.5 deg of the current auto command (hood checked only when the command has one)
  aimed: () => { const c = S_AIM.lastCmd; return !!c && c.turretDeg != null && Math.abs(S.turretDeg - c.turretDeg) < 2 && (c.hoodDeg == null || Math.abs(S.hoodDeg - c.hoodDeg) < 1.5); },
  // autoAim target: the solved shot right now (rpm, turret, hood) or null when there is no solution
  autoAimTarget: () => { const c = S_AIM.lastCmd; return c && c.rpm > 300 && c.turretDeg != null && c.hoodDeg != null ? { rpm: c.rpm, turretDeg: c.turretDeg, hoodDeg: c.hoodDeg, feasible: true } : null; },
  // autoAim current: plant state of flywheel, turret and hood
  autoAimCurrent: () => ({ rpm: S.rpm, turretDeg: S.turretDeg, hoodDeg: S.hoodDeg }),
  // autoAim apply: forces auto aim mode; the frame loop already drives the plant toward S_AIM.lastCmd (argument unused)
  autoAimApply: () => { if (S_AIM.mode !== 'auto') { $('aim-mode').value = 'auto'; setAimMode('auto'); } return true; },
};
const autoPlanner = createAutoPlanner({
  leftEl: $('autopanel-left'), rightEl: $('autopanel-right'), timelineEl: $('timeline'),
  getPaths: () => ({ route: pathEditor.route }),
  getHost: () => autoHost,
  toast, onChange: () => {}, onPreviewPath: () => { setView('auto'); },
  exportFn: (auto, style) => exportAutoJava(auto, { style, className: (auto.name || 'MyAuto').replace(/[^A-Za-z0-9_]/g, '') || 'MyAuto' }),
  pathSampler: async route => { const r = await samplePedro(route, { step: 1 }); if (!r.ok) throw new Error(r.error || 'sampling failed'); return r.samples; },
});
const autoRunning = () => !!(autoPlanner.runner && autoPlanner.runner.state === 'running');
const autoActive = () => !!(autoPlanner.runner && (autoPlanner.runner.state === 'running' || autoPlanner.runner.state === 'paused'));
function prepareAutoRun() {
  resetField(); stats.reset(); S.shots = 0; S.scored = 0;
  Object.assign(S, { vx: 0, vy: 0, om: 0, bodyV: [0, 0, 0], turretDeg: 0, rpm: 0 }); Object.assign(plant.state, { turretDeg: 0, rpm: 0 });
  if (S_AUTO.savedLimit === null) S_AUTO.savedLimit = S_PRACTICE.ammoLimit;
  S_PRACTICE.ammoLimit = true; $('opt-ammo').checked = true;                   // real hopper accounting so shootAll ends when the hopper is empty
  S_AUTO.transfer = false;                                                      // every run starts with the transfer off
  S_PRACTICE.ammo = Math.min(RULES.preloadPollen, S_PRACTICE.ammoCap); MAP.trail.length = 0;
}
// 3D run panel: switches to the 3D view when a run starts, shows progress, Stop / Restart / Pause
const AR = { prev: 'idle', show: false };
function autoRunTick() {
  const r = autoPlanner.runner, st = r ? r.state : 'idle';
  if (st === 'running' && AR.prev !== 'running') {
    if (AR.prev !== 'paused') AR.show = true;            // Resume after the user hid the panel keeps it hidden (the #ar-open button brings it back)
    if (S.view !== '3d') setView('3d');
  }
  if (S_AUTO.savedLimit !== null && (st === 'idle' || st === 'done' || st === 'error')) { S_PRACTICE.ammoLimit = S_AUTO.savedLimit; $('opt-ammo').checked = S_AUTO.savedLimit; S_AUTO.savedLimit = null; }
  AR.prev = st;
}
function updateAutoPanel() {
  const r = autoPlanner.runner, p = $('autorun'), st = r ? r.state : 'idle', show = AR.show && S.view === '3d' && !!r;
  const op = $('ar-open'); op.hidden = !(S.view === '3d' && !show); op.textContent = r && st !== 'idle' ? 'Run panel' : 'Run auto';
  p.hidden = !show; if (!show) return;
  const total = (autoPlanner.bars && autoPlanner.bars.total) || (autoPlanner.auto && autoPlanner.auto.settings && autoPlanner.auto.settings.matchSeconds) || 30;
  const label = { running: 'Running', paused: 'Paused', done: 'Finished', error: 'Failed', idle: 'Stopped' }[st] || st;
  p.dataset.state = st; $('ar-name').textContent = (autoPlanner.auto && autoPlanner.auto.name) || 'Auto'; $('ar-state').textContent = label;
  $('ar-fill').style.width = Math.min(100, st === 'done' ? 100 : (r.time / Math.max(total, 0.1)) * 100).toFixed(1) + '%';
  $('ar-time').textContent = r.time.toFixed(1) + ' s / ' + total.toFixed(1) + ' s';
  $('ar-stats').textContent = 'shots ' + S.shots + ' / scored ' + S.scored + ' / balls ' + S_PRACTICE.ammo + (S_AUTO.transfer ? ' / transfer on' : '');
  $('ar-src').textContent = st === 'error' ? 'Error: ' + (r.errors.slice(-1)[0] || 'unknown') : 'Motion: ' + (Object.keys(r.trajs || {}).length ? 'real Pedro follower' : 'built-in model (start run_sim.py for the real Pedro follower)');
  $('ar-pause').textContent = st === 'running' ? 'Pause' : st === 'paused' ? 'Resume' : 'Run'; $('ar-stop').disabled = st === 'idle';
}
$('ar-pause').onclick = () => autoPlanner.pause();
// Stop: runner.reset() restores the auto start pose, but resetAllState() ends with resetRobot() (START pose), so the auto start is re-applied last.
function putRobotAtAutoStart() {
  const st = autoPlanner.auto && autoPlanner.auto.start; if (!st) return;
  S.x = st.x; S.y = st.y; S.h = st.h || 0; S.vx = S.vy = S.om = 0; S.bodyV = [0, 0, 0];
}
$('ar-stop').onclick = () => { autoPlanner.stop(); resetAllState(); };   // Stop also resets hives, kind, robot
$('ar-stop').addEventListener('click', () => autoHost.transfer(false));   // Stop also turns the transfer off
$('ar-stop').addEventListener('click', putRobotAtAutoStart);   // runs after the onclick above: resetAllState() put the robot at START, so the auto start pose is re-applied last
$('ar-restart').onclick = () => { resetAllState(); autoPlanner.run(); };   // reset the field, then run the same auto again
// #ar-open: 3D-view entry point. Shows the run panel again; with no run yet it starts the current auto (Run in sim without opening the Auto tab).
$('ar-open').onclick = () => { const r = autoPlanner.runner; if (!r || r.state === 'idle') { resetAllState(); autoPlanner.run(); } AR.show = true; };
$('ar-edit').onclick = () => setView('auto'); $('ar-close').onclick = () => { AR.show = false; $('autorun').hidden = true; };
// queued shots from the auto (shoot / shootAll): fire whenever the shot is allowed, one every 0.25 s
function autoShots(dt) {
  S_AUTO.time += dt;
  if (!(S_AUTO.shots > 0)) return;
  S_AUTO.cool -= dt;
  if (S_PRACTICE.ammo <= 0) { S_AUTO.shots = 0; return; }
  if (S_AUTO.cool <= 0 && S.eval && S.eval.allowed) { shoot(); S_AUTO.shots--; S_AUTO.cool = 0.25; }
}

// shootable area (feasibleRegion): cached, shown in the 3D view as tinted floor cells and in the planner's 2D map
function getRegion() {
  const key = [S.alliance, hive.red.state, hive.blue.state, S_AIM.zone.preset, S_AIM.enforceZone].join('|');
  if (key !== S_AIM.regionKey) { S_AIM.regionKey = key; S_AIM.region = feasibleRegion({ shooter: SHOOTER, target: aimTarget(), obstacles: fieldData, hiveBoxes: hiveTall, legalZone: S_AIM.zone, enforceZone: S_AIM.enforceZone, alliance: S.alliance, step: 3, core: Core }); }
  return S_AIM.region;
}
const regionMesh = new THREE.InstancedMesh(new THREE.PlaneGeometry(1, 1), new THREE.MeshBasicMaterial({ transparent: true, opacity: 0.34, depthWrite: false }), 6000);
regionMesh.visible = false; regionMesh.frustumCulled = false; regionMesh.count = 0; regionMesh.renderOrder = 2; scene.add(regionMesh);
function updateRegion3d() {
  const on = S_AIM.showRegion && S.view === '3d'; regionMesh.visible = on; if (!on) return;
  const R = getRegion(); if (regionMesh.userData.built === R) return; regionMesh.userData.built = R;
  const m = new THREE.Matrix4(), col = new THREE.Color(), n = Math.min(R.cells.length, 6000);
  for (let i = 0; i < n; i++) { const c = R.cells[i]; m.makeScale(R.step * 0.94, R.step * 0.94, 1).setPosition(c.x, c.y, 0.35); regionMesh.setMatrixAt(i, m); regionMesh.setColorAt(i, col.set(c.ok ? 0x5fe08a : 0xff5a5a)); }
  regionMesh.count = n; regionMesh.instanceMatrix.needsUpdate = true; if (regionMesh.instanceColor) regionMesh.instanceColor.needsUpdate = true;
}

// ================= 2D bird's-eye map (Pedro frame: origin bottom-left, x right, y up the screen) =================
const mapCv = $('map2d'), mctx = mapCv.getContext('2d');
const MAP = { k: 1, mouse: null, trail: [] };
let PLAN_FLIP = true;   // "View from alliance wall" checkbox (#plan-flip); only affects the Auto planner on blue
const isOn = key => !!(layers[key] && layers[key][0] && layers[key][0].visible);
function mapResize() {
  const dpr = window.devicePixelRatio || 1;
  mapCv.width = Math.round(innerWidth * dpr); mapCv.height = Math.round(innerHeight * dpr);
  mctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  if (S.view === 'auto') { MAP.avail = autoAvail(); fitMap(); return; }
  const left = S.view === 'path' ? 20 : 248, right = S.view === 'path' ? 360 : 320, pad = 24;
  const top = 40 + 24, bottom = 36 + 24;
  const availW = Math.max(200, innerWidth - left - right - pad * 2), availH = Math.max(200, innerHeight - top - bottom);
  MAP.avail = { left, pad, top, availW, availH };
  fitMap();
}
// Auto planner: the field gets ALL the room the planner panels leave (measured from the DOM every frame, so the timeline resize handle, the editor top bar and
// the floating playback bar are all respected). Margin 12 px, no other padding.
function autoAvail() {
  const vis = id => { const e = $(id); return e && e.offsetWidth > 0 && e.offsetHeight > 0 ? e.getBoundingClientRect() : null; };
  const Lr = vis('autopanel-left'), Rr = vis('autopanel-right'), Tl = vis('timeline'), Tb = vis('pp-topbar'), Fl = vis('pp-float'), pad = 12;
  const left = Lr ? Lr.right : 0, right = Rr ? Math.max(0, innerWidth - Rr.left) : 0, top = Tb ? Tb.bottom : 40;
  const bottom = Fl ? innerHeight - Fl.top + 4 : Tl ? innerHeight - Tl.top : 36;
  return { left, pad, top: top + pad, availW: Math.max(100, innerWidth - left - right - pad * 2), availH: Math.max(100, innerHeight - top - bottom - pad * 2) };
}
// Fit the map to the whole field, or (Auto planner) to the chosen alliance's half only, like PathPlanner's cropped view.
function fitMap() {
  if (S.view === 'auto') MAP.avail = autoAvail();
  const A = MAP.avail; if (!A) return;
  const b = S.view === 'auto' ? viewBounds(PLAN_VIEW.alliance, !!PLAN_VIEW.fullField) : { x0: 0, x1: 144, y0: 0, y1: 144 };
  const w = b.x1 - b.x0, h = b.y1 - b.y0, key = [b.x0, b.x1, b.y0, b.y1, A.availW, A.availH, planFlip() ? 'flip' : 'plain', planRot90()].join();
  if (MAP.fitKey === key) return; MAP.fitKey = key; MAP.bounds = b;
  // a quarter-turned half is drawn through a canvas rotation, so its unrotated box is the available box with width and height swapped
  MAP.k = S.view === 'auto' ? (planRot90() ? viewScale({ bounds: b, rotate180: false }, A.availH, A.availW) : viewScale({ bounds: b, rotate180: planFlip() }, A.availW, A.availH)) : Math.min(A.availW / w, A.availH / h);   // origin is no longer stored: mx/my/screenToField call pathing.js toScreen/fromScreen on the fitted box
}
window.addEventListener('resize', mapResize); mapResize();
// View transform. The Auto planner on the BLUE alliance is drawn rotated 180 degrees about the crop centre (view from the blue wall).
// Only positions are mirrored; text is always drawn plain. Every other view is the unflipped field frame.
// Function declarations (not const) so the path editor closure at the top of this file can reference them.
// The flip decision comes from pathing.js allianceViewTransform (blue -> rotate180); the mapping itself is pathing.js toScreen/fromScreen.
function planFlip() { return S.view === 'auto' && allianceViewTransform(PLAN_VIEW.alliance).rotate180 && PLAN_FLIP; }
// Alliance half shown (not full field) and "View from alliance wall" on: the half is turned a quarter (canvas rotation about the field box centre) so its long side runs across the screen and the field FILLS the area. Blue +1 (clockwise, own wall at the bottom), red -1.
function planRot90() { return S.view === 'auto' && PLAN_FLIP && !PLAN_VIEW.fullField && (PLAN_VIEW.alliance === 'blue' || PLAN_VIEW.alliance === 'red') ? (PLAN_VIEW.alliance === 'blue' ? 1 : -1) : 0; }
function viewBox() { return MAP.bounds || { x0: 0, x1: 144, y0: 0, y1: 144 }; }
// View object for pathing.js: crop bounds, rotated 180 about field centre when flipped. Canvas box = the fitted area (MAP.avail), so the origin matches fitMap.
function viewXf() { return { bounds: viewBox(), rotate180: planFlip() && !planRot90() }; }
function viewBoxPx() {
  const A = MAP.avail || { left: 0, pad: 0, top: 0, availW: 1, availH: 1 };
  if (planRot90()) { const c = rotCenter(); return { left: c.x - A.availH / 2, top: c.y - A.availW / 2, w: A.availH, h: A.availW }; }   // unrotated box of a quarter-turned view
  return { left: A.left + A.pad, top: A.top, w: A.availW, h: A.availH };
}
function rotCenter() { const A = MAP.avail || { left: 0, pad: 0, top: 0, availW: 1, availH: 1 }; return { x: A.left + A.pad + A.availW / 2, y: A.top + A.availH / 2 }; }
const rotPt = (px, py, a) => { const c = rotCenter(), dx = px - c.x, dy = py - c.y, co = Math.cos(a), si = Math.sin(a); return [c.x + dx * co - dy * si, c.y + dx * si + dy * co]; };
// Field inches -> final screen pixels (includes the quarter turn); mx/my are the pre-rotation positions the canvas drawing code works in.
function fieldToPx(x, y) { const q = planRot90(); return q ? rotPt(mx(x), my(y), q * Math.PI / 2) : [mx(x), my(y)]; }
// Field inches (Pedro frame) -> screen pixels. toScreen is separable, so the x-only and y-only calls each give one axis exactly.
function mx(x) { const s = viewBoxPx(); return s.left + toScreen(viewXf(), x, 0, s.w, s.h).x; }
function my(y) { const s = viewBoxPx(); return s.top + toScreen(viewXf(), 0, y, s.w, s.h).y; }
// Inverse of mx/my: screen pixels -> field inches (Pedro frame). Used by every mouse handler and the cursor readout.
function screenToField(sx, sy) {
  const q = planRot90(); if (q) [sx, sy] = rotPt(sx, sy, -q * Math.PI / 2);
  const s = viewBoxPx(), p = fromScreen(viewXf(), sx - s.left, sy - s.top, s.w, s.h);
  return [p.x, p.y];
}
// Screen rect [left, top, w, h] of the full 0..144 field square (min/abs so a mirrored view still gets a positive rect).
function floorRect() { return [Math.min(mx(0), mx(144)), Math.min(my(0), my(144)), Math.abs(mx(144) - mx(0)), Math.abs(my(144) - my(0))]; }
function hull(pts) {
  const p = [...pts].sort((a, b) => a[0] - b[0] || a[1] - b[1]); const cr = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const lo = []; for (const q of p) { while (lo.length >= 2 && cr(lo[lo.length - 2], lo[lo.length - 1], q) <= 0) lo.pop(); lo.push(q); }
  const up = []; for (const q of p.reverse()) { while (up.length >= 2 && cr(up[up.length - 2], up[up.length - 1], q) <= 0) up.pop(); up.push(q); }
  return lo.slice(0, -1).concat(up.slice(0, -1));
}
function poly(pts, fill, stroke, lw = 1.5, dash = null) {
  mctx.beginPath(); pts.forEach((p, i) => (i ? mctx.lineTo(mx(p[0]), my(p[1])) : mctx.moveTo(mx(p[0]), my(p[1])))); mctx.closePath();
  if (fill) { mctx.fillStyle = fill; mctx.fill(); }
  if (stroke) { mctx.setLineDash(dash || []); mctx.lineWidth = lw; mctx.strokeStyle = stroke; mctx.stroke(); mctx.setLineDash([]); }
}
// Auto planner view: PathPlanner-style flat field (plannerfield.js) with the path editor's drawing on top. Nothing of the live robot, balls or the 3D layers.
function drawPlannerView() {
  const W = innerWidth, H = innerHeight, was = pathEditor.active;
  const q = planRot90(), c = rotCenter();
  PLAN_VIEW.viewQuarter = q;   // labels counter-rotate by this
  mctx.clearRect(0, 0, W, H);
  mctx.fillStyle = '#1b1b1f'; mctx.fillRect(0, 0, W, H);
  mctx.save();
  if (q) { mctx.translate(c.x, c.y); mctx.rotate(q * Math.PI / 2); mctx.translate(-c.x, -c.y); }
  const A = MAP.avail, env = { W, H, k: MAP.k, mx, my, px: fieldToPx, avail: { L: A.left + A.pad, T: A.top, R: A.left + A.pad + A.availW, B: A.top + A.availH }, bounds: viewBox(), scene: sceneJson, hive: { red: hive.red.state, blue: hive.blue.state }, mouse: MAP.mouse, fromScreen: screenToField };
  pathEditor.drawField(mctx, env);
  pathEditor.active = true; pathEditor.draw(mctx); pathEditor.active = was;
  mctx.restore();
  pathEditor.drawOverlay(mctx, env);
}
function drawMap() {
  fitMap();
  if (S.view === 'auto') { drawPlannerView(); return; }
  const b = MAP.bounds || { x0: 0, x1: 144, y0: 0, y1: 144 }, crop = S.view === 'auto' && (b.x1 - b.x0 < 144 || b.y1 - b.y0 < 144);
  mctx.save();
  if (crop) {
    // clip = the mapped bounds' screen box (min/max of the corners, so it works flipped) plus a margin: footprints that overhang the field edge, the start marker and the captions are not cut
    const m = Math.max(34, 10 * MAP.k), L = Math.min(mx(b.x0), mx(b.x1)), R = Math.max(mx(b.x0), mx(b.x1)), Tp = Math.min(my(b.y0), my(b.y1)), Bt = Math.max(my(b.y0), my(b.y1));
    mctx.beginPath(); mctx.rect(L - m, Tp - m, R - L + 2 * m, Bt - Tp + m + 50); mctx.clip();
  }
  drawMapInner();
  mctx.restore();
}
// Field values (multiples of 24 in) that fall inside [lo, hi]: ticks and grid lines only for the visible bounds.
function ticksIn(lo, hi) { const out = []; for (let i = 0; i <= 144; i += 24) if (i >= lo - 1e-9 && i <= hi + 1e-9) out.push(i); return out; }
// Robot footprint at a pose (18 in square, the same size as the live robot). Heading h in Pedro radians. The flipped view is a 180 degree turn, so the outline turns with it.
function drawFootprint(x, y, h, fill) {
  const k = MAP.k, ang = planFlip() ? Math.PI - h : -h;
  mctx.save(); mctx.translate(mx(x), my(y)); mctx.rotate(ang);
  mctx.fillStyle = fill; mctx.fillRect(-9 * k, -9 * k, 18 * k, 18 * k);
  mctx.strokeStyle = '#e6ecf5'; mctx.lineWidth = 2; mctx.strokeRect(-9 * k, -9 * k, 18 * k, 18 * k);
  mctx.fillStyle = '#ffd23a'; mctx.fillRect(7 * k, -9 * k, 2 * k, 18 * k);                    // front edge, as on the live robot
  mctx.restore();
}
// Centre dot plus a heading arrow. Positions go through mx/my, so the arrow is correct in both views.
function drawPoseMarker(x, y, h, color) {
  mctx.beginPath(); mctx.moveTo(mx(x), my(y)); mctx.lineTo(mx(x + Math.cos(h) * 12), my(y + Math.sin(h) * 12));
  mctx.strokeStyle = color; mctx.lineWidth = 2; mctx.stroke();
  mctx.beginPath(); mctx.arc(mx(x), my(y), 4.5, 0, 6.283); mctx.fillStyle = color; mctx.fill();
}
// Auto planner: the start pose, and the end pose of every selected path, as footprints with heading arrows.
function drawPlannerPoses() {
  const st = PLAN_VIEW.start;
  if (st) { drawFootprint(st.x, st.y, st.h || 0, 'rgba(64,255,160,0.22)'); drawPoseMarker(st.x, st.y, st.h || 0, '#40ffa0'); }
  for (const it of PLAN_VIEW.items || []) {
    if (!it.selected || !it.route) continue;
    const r = it.route, segs = r.segments || [], last = segs.length ? segs[segs.length - 1].points.at(-1) : r.start;
    if (!last) continue;
    let h = r.start ? (r.start.h || 0) : 0;
    try { const res = resolveHeadings(r); if (res.length) h = res[res.length - 1].h1 ?? h; } catch (_) { /* keep the start heading */ }
    drawFootprint(last.x, last.y, h, 'rgba(255,152,0,0.2)'); drawPoseMarker(last.x, last.y, h, '#ff9800');
  }
}
function drawMapInner() {
  const W = innerWidth, H = innerHeight, k = MAP.k, vb = viewBox(), flip = planFlip();
  const cropped = S.view === 'auto' && (vb.x1 - vb.x0 < 144 || vb.y1 - vb.y0 < 144);
  const xs = ticksIn(vb.x0, vb.x1), ys = ticksIn(vb.y0, vb.y1);
  const sL = Math.min(mx(vb.x0), mx(vb.x1)), sR = Math.max(mx(vb.x0), mx(vb.x1)), sT = Math.min(my(vb.y0), my(vb.y1)), sB = Math.max(my(vb.y0), my(vb.y1));
  mctx.clearRect(0, 0, W, H); mctx.fillStyle = '#0d1220'; mctx.fillRect(0, 0, W, H);
  // field floor, 24 in tiles, border, ticks
  mctx.fillStyle = '#262d40'; mctx.fillRect(...floorRect());
  mctx.lineWidth = 1; mctx.strokeStyle = 'rgba(255,255,255,0.13)';
  for (const i of xs) { mctx.beginPath(); mctx.moveTo(mx(i), sT); mctx.lineTo(mx(i), sB); mctx.stroke(); }
  for (const i of ys) { mctx.beginPath(); mctx.moveTo(sL, my(i)); mctx.lineTo(sR, my(i)); mctx.stroke(); }
  if (cropped) { mctx.save(); mctx.setLineDash([6, 4]); mctx.strokeStyle = 'rgba(200,210,230,0.6)'; mctx.lineWidth = 1.5; mctx.beginPath(); mctx.moveTo(mx(72), sT); mctx.lineTo(mx(72), sB); mctx.stroke(); mctx.restore(); }   // half-field boundary x = 72
  mctx.strokeStyle = 'rgba(255,255,255,0.7)'; mctx.lineWidth = 2; mctx.strokeRect(...floorRect());
  mctx.fillStyle = '#8d9ab5'; mctx.font = '11px Segoe UI, sans-serif'; mctx.textAlign = 'center';
  for (const i of xs) mctx.fillText(i, mx(i), sB + 14);
  mctx.textAlign = 'right'; for (const i of ys) mctx.fillText(i, sL - 6, my(i) + 4); mctx.textAlign = 'center';
  // captions: the far wall (y = 144) sits on the top edge unless the view is flipped, then it swaps to the bottom
  mctx.fillText(flip ? '←  x (in)' : 'x (in)  →', (sL + sR) / 2, sB + 28);
  mctx.fillText('far wall (y = 144)', (sL + sR) / 2, flip ? sB + 42 : sT - 8); mctx.fillText('audience (y = 0)', (sL + sR) / 2, flip ? sT - 8 : sB + 42);
  // zones
  if (isOn('zones')) for (const z of sceneJson.tapes.zones) {
    if (!['loading_zone', 'garden', 'alliance_area', 'start_line'].includes(z.kind)) continue;
    const c = z.alliance === 'red' ? '255,90,90' : z.alliance === 'blue' ? '90,141,255' : '207,216,232';
    poly(z.polygon, `rgba(${c},0.16)`, `rgba(${c},0.9)`, z.kind === 'alliance_area' ? 1 : 1.5);
    if (isOn('labels')) { const cx = z.polygon.reduce((s, p) => s + p[0], 0) / z.polygon.length, cy = z.polygon.reduce((s, p) => s + p[1], 0) / z.polygon.length; mctx.fillStyle = `rgb(${c})`; mctx.textAlign = 'center'; mctx.fillText(z.name, mx(cx), my(cy) + 4); }
  }
  // hive frame footprint (49.46 x 38.95, approximate) and the hive cells for the current tip states
  poly([[72 - 24.73, 72 - 19.475], [72 + 24.73, 72 - 19.475], [72 + 24.73, 72 + 19.475], [72 - 24.73, 72 + 19.475]], null, 'rgba(200,210,230,0.35)', 1.2, [5, 4]);
  for (const a of ['red', 'blue']) {
    const col = a === 'red' ? '255,90,90' : '90,141,255', cells = sceneJson.constants.hives[a].states[hive[a].state].cells;
    for (const c of Object.values(cells)) {
      const pts = [...(c.opening_corners || []), ...(c.floor_corners || []), ...(c.back_wall_corners || [])].map(p => [p[0], p[1]]);
      if (pts.length >= 3) poly(hull(pts), `rgba(${col},${c.up ? 0.34 : 0.14})`, `rgba(${col},${c.up ? 1 : 0.5})`, c.up ? 2 : 1);
    }
  }
  // flowers and the static ball starts
  for (const f of sceneJson.constants.flowers) { const c = f.opening_center; mctx.beginPath(); mctx.arc(mx(c[0]), my(c[1]), Math.max(3, 2.2 * k), 0, 6.283); mctx.fillStyle = 'rgba(255,200,60,0.85)'; mctx.fill(); }
  
  // AprilTags: positions follow the hive tilt; green when the turret camera sees them
  if (isOn('tags')) for (const id of Object.keys(tagInfo)) {
    const ti = tagInfo[id]; if (!ti.centerWorld) continue; const c = ti.centerWorld, n = ti.normalWorld;
    mctx.fillStyle = ti.visible ? '#5fe08a' : (ti.alliance === 'red' ? '#ff9c9c' : '#9cc4ff'); mctx.fillRect(mx(c.x) - 3, my(c.y) - 3, 6, 6);
    mctx.strokeStyle = mctx.fillStyle; mctx.lineWidth = 1; mctx.beginPath(); mctx.moveTo(mx(c.x), my(c.y)); mctx.lineTo(mx(c.x + n.x * 5), my(c.y + n.y * 5)); mctx.stroke();
    if (isOn('labels')) { mctx.font = '10px Segoe UI, sans-serif'; mctx.textAlign = 'left'; mctx.fillText(id, mx(c.x) + 5, my(c.y) - 4); }
  }
  // aim points
  if (isOn('aim')) for (const a of ['red', 'blue']) { const m = aimMarks[a].s.position; mctx.beginPath(); mctx.arc(mx(m.x), my(m.y), 5, 0, 6.283); mctx.strokeStyle = a === 'red' ? '#ff6a6a' : '#6aa0ff'; mctx.lineWidth = 2; mctx.stroke(); mctx.beginPath(); mctx.moveTo(mx(m.x) - 8, my(m.y)); mctx.lineTo(mx(m.x) + 8, my(m.y)); mctx.moveTo(mx(m.x), my(m.y) - 8); mctx.lineTo(mx(m.x), my(m.y) + 8); mctx.stroke(); }
  // Pedro path from the robot (same samples the follower uses), with start/end and heading ticks
  if (S.view !== 'auto' && S.path && S.path.length > 1) {
    mctx.beginPath(); S.path.forEach((p, i) => (i ? mctx.lineTo(mx(p[0]), my(p[1])) : mctx.moveTo(mx(p[0]), my(p[1])))); mctx.strokeStyle = '#40ffa0'; mctx.lineWidth = 2.5; mctx.stroke();
    if (S.path[0].length > 2) for (let i = 0; i < S.path.length; i += 8) { const p = S.path[i]; mctx.beginPath(); mctx.moveTo(mx(p[0]), my(p[1])); mctx.lineTo(mx(p[0] + Math.cos(p[2]) * 6), my(p[1] + Math.sin(p[2]) * 6)); mctx.strokeStyle = 'rgba(64,255,160,0.7)'; mctx.lineWidth = 1.5; mctx.stroke(); }
    const e = S.path[S.path.length - 1], s0 = S.path[0];
    mctx.fillStyle = '#40ffa0'; mctx.beginPath(); mctx.arc(mx(s0[0]), my(s0[1]), 4, 0, 6.283); mctx.fill(); mctx.fillRect(mx(e[0]) - 4, my(e[1]) - 4, 8, 8);
  }
  // robot trail
  if (S.view !== 'auto') MAP.trail.push([S.x, S.y]); if (MAP.trail.length > 700) MAP.trail.shift();
  if (S.view !== 'auto' && MAP.trail.length > 1) { mctx.beginPath(); MAP.trail.forEach((p, i) => (i ? mctx.lineTo(mx(p[0]), my(p[1])) : mctx.moveTo(mx(p[0]), my(p[1])))); mctx.strokeStyle = 'rgba(255,255,255,0.35)'; mctx.lineWidth = 1.5; mctx.stroke(); }
  if (S_AIM.showRegion) {
    const R = getRegion(), hh = R.step * k;
    for (const c of R.cells) { mctx.fillStyle = c.ok ? 'rgba(95,224,138,0.28)' : 'rgba(255,90,90,0.16)'; mctx.fillRect(mx(c.x) - hh / 2, my(c.y) - hh / 2, hh, hh); }
    for (const pg of S_AIM.zone.polygonsFor(S.alliance)) { if (Array.isArray(pg)) poly(pg, null, 'rgba(95,224,138,0.95)', 2, [6, 4]); else { poly(pg.outer, null, 'rgba(95,224,138,0.95)', 2, [6, 4]); poly(pg.inner, null, 'rgba(95,224,138,0.95)', 2, [6, 4]); } }
  }
  if (S.view === 'path') pathEditor.draw(mctx);
  else if (S.view === 'auto') { const was = pathEditor.active; pathEditor.active = true; pathEditor.draw(mctx); pathEditor.active = was; drawPlannerPoses(); }
  // camera wedge, shot ground track, robot footprint
  const cp = cameraPose();
  if (isOn('frustum') && S.view !== 'auto') {
    const yaw = S.h + rad(S.turretDeg), hf = rad(CAMERA.hfovDeg / 2), R = 60;
    mctx.beginPath(); mctx.moveTo(mx(cp.pos.x), my(cp.pos.y)); mctx.lineTo(mx(cp.pos.x + Math.cos(yaw + hf) * R), my(cp.pos.y + Math.sin(yaw + hf) * R)); mctx.lineTo(mx(cp.pos.x + Math.cos(yaw - hf) * R), my(cp.pos.y + Math.sin(yaw - hf) * R)); mctx.closePath(); mctx.fillStyle = 'rgba(102,255,204,0.12)'; mctx.fill(); mctx.strokeStyle = 'rgba(102,255,204,0.5)'; mctx.lineWidth = 1; mctx.stroke();
  }
  const sol = S.sol;
  if (isOn('traj') && sol && sol.ok && S.rpm > 300) {
    mctx.beginPath(); for (let i = 0; i <= 30; i++) { const q = Core.trajectoryPoint(sol.P, sol.p.z0, sol.vs, sol.uh, sol.uz, sol.T * i / 30); i ? mctx.lineTo(mx(q[0]), my(q[1])) : mctx.moveTo(mx(q[0]), my(q[1])); }
    mctx.strokeStyle = sol.fromBehind ? '#ff5a5a' : '#ffa21a'; mctx.lineWidth = 2; mctx.setLineDash([6, 4]); mctx.stroke(); mctx.setLineDash([]);
  }
  if (isOn('robot') && S.view !== 'auto') {
    mctx.save(); mctx.translate(mx(S.x), my(S.y)); mctx.rotate(-S.h);
    mctx.fillStyle = 'rgba(58,70,96,0.95)'; mctx.fillRect(-9 * k, -9 * k, 18 * k, 18 * k); mctx.strokeStyle = '#e6ecf5'; mctx.lineWidth = 2; mctx.strokeRect(-9 * k, -9 * k, 18 * k, 18 * k);
    mctx.fillStyle = '#ffd23a'; mctx.fillRect(7 * k, -9 * k, 2 * k, 18 * k);                    // front edge
    mctx.rotate(-rad(S.turretDeg)); mctx.beginPath(); mctx.arc(0, 0, 4.2 * k, 0, 6.283); mctx.fillStyle = '#8a96b0'; mctx.fill();
    mctx.beginPath(); mctx.moveTo(0, 0); mctx.lineTo(9 * k, 0); mctx.strokeStyle = '#ff7a1a'; mctx.lineWidth = 3; mctx.stroke();   // turret / shooter direction
    mctx.restore();
  }
  // balls in flight
  if (isOn('balls')) for (const b of world.balls) { if (b.state === 'held') continue; mctx.beginPath(); mctx.arc(mx(b.x), my(b.y), Math.max(2, b.r * k), 0, 6.283); mctx.fillStyle = b.state === 'scored' ? '#8d9ab5' : b.type === 'pollen' ? '#ffd23a' : b.type === 'nectar' ? '#ff9a3d' : b.type === 'nectar_red' ? '#ff5a5a' : '#5a8dff'; mctx.fill(); }
  // cursor readout
  if (MAP.mouse) {
    const [x, y] = screenToField(MAP.mouse[0], MAP.mouse[1]);
    mctx.fillStyle = 'rgba(14,20,34,0.9)'; mctx.fillRect(MAP.mouse[0] + 12, MAP.mouse[1] + 8, 218, 22);
    mctx.fillStyle = '#e6ecf5'; mctx.font = '12px Consolas, monospace'; mctx.textAlign = 'left'; mctx.fillText(`Pose(${x.toFixed(1)}, ${y.toFixed(1)}, h)  ctrl-click copies`, MAP.mouse[0] + 17, MAP.mouse[1] + 23);
  }
}
mapCv.addEventListener('pointermove', e => (MAP.mouse = [e.clientX, e.clientY]));
mapCv.addEventListener('pointerleave', () => (MAP.mouse = null));
mapCv.addEventListener('contextmenu', e => e.preventDefault());
const fieldXY = e => screenToField(e.clientX, e.clientY);
mapCv.addEventListener('pointermove', e => { if ((S.view === 'path' || pathEditor.editing)) { const [fx, fy] = fieldXY(e); pathEditor.pointer('move', fx, fy, e); } });
window.addEventListener('pointerup', e => { if ((S.view === 'path' || pathEditor.editing)) { const [fx, fy] = fieldXY(e); pathEditor.pointer('up', fx, fy, e); } });
mapCv.addEventListener('pointerdown', e => {
  const [x, y] = fieldXY(e);
  if ((S.view === 'path' || pathEditor.editing) && pathEditor.pointer('down', x, y, e)) return;
  if (e.button === 2) { S.h = Math.atan2(y - S.y, x - S.x); return; }              // right click: face that point
  if (e.ctrlKey) { navigator.clipboard?.writeText(`new Pose(${x.toFixed(1)}, ${y.toFixed(1)}, Math.toRadians(${deg(S.h).toFixed(0)}))`); toast('Copied Pose to clipboard'); return; }
  if (S.linked || autoActive()) return; placeRobot(x, y); MAP.trail.length = 0;
});
const vizFrame = document.createElement('iframe');
vizFrame.id = 'vizframe'; vizFrame.title = 'Official Pedro Pathing Visualizer';
Object.assign(vizFrame.style, { position: 'fixed', left: '0', right: '0', top: '40px', bottom: '36px', width: '100%', height: 'calc(100% - 76px)', border: '0', display: 'none', zIndex: '3', background: '#0f0f14' });
document.body.append(vizFrame);
function setView(v) {
  S.view = v; keys.clear(); const is2d = v === '2d' || v === 'path' || v === 'auto';
  pathEditor.active = v === 'path';
  $('pathpanel').style.display = v === 'path' ? 'block' : 'none';
  $('opts').style.display = (v === 'path' || v === 'auto' || v === 'viz') ? 'none' : '';
  { const lp = $('leftpanel'); if (lp) lp.style.display = (v === '3d' || v === '2d') ? '' : 'none'; }
  for (const [id, name] of [['tab-3d', '3d'], ['tab-2d', '2d'], ['tab-path', 'path'], ['tab-auto', 'auto'], ['tab-viz', 'viz']]) { const el = $(id); if (el) el.classList.toggle('on', v === name); }
  vizFrame.style.display = v === 'viz' ? 'block' : 'none';
  { const ss = document.querySelector('.dg-shot'); if (ss) ss.style.visibility = (v === '3d' || v === '2d') ? '' : 'hidden'; }
  if (v === 'viz' && !vizFrame.getAttribute('src')) vizFrame.setAttribute('src', 'visualizer/index.html');
  mapCv.style.display = is2d ? 'block' : 'none'; canvas.style.visibility = (is2d || v === 'viz') ? 'hidden' : 'visible'; $('labels').style.display = (is2d || v === 'viz') ? 'none' : 'block';
  $('cam-box').style.display = is2d ? 'none' : 'block';
  mapResize();
  $('help').innerHTML = is2d ? '<b>Drive</b> W/S, A/D, Q/E (+Shift) &nbsp;|&nbsp; <b>Space</b> shoot &nbsp;|&nbsp; <b>Click</b> place robot &nbsp;|&nbsp; <b>Right-click</b> face that point &nbsp;|&nbsp; <b>Ctrl-click</b> copy <code>new Pose(x, y, h)</code><br><b>T</b>/<b>Y</b> tip hives &nbsp; <b>L</b> labels &nbsp; <b>R</b> reset' : HELP3D;
}
const HELP3D = $('help').innerHTML;
$('tab-3d').onclick = () => setView('3d'); $('tab-auto').onclick = () => setView('auto'); $('tab-viz').onclick = () => setView('viz');
$('plan-flip').checked = PLAN_FLIP;
$('plan-flip').onchange = e => { PLAN_FLIP = !!e.target.checked; MAP.fitKey = null; drawMap(); };

// ================= main loop =================
const clock = new THREE.Clock();
let hudT = 0, last = { x: S.x, y: S.y, h: S.h };
$('loading').style.display = 'none'; diag.bootDone();
setCam('iso');
function matchPhase() { const ph = matchClock.phase; return ph === 'auto' || ph === 'transition' ? 'auto' : 'teleop'; }
function simUpdate(dt) {
  matchClock.update(dt);
  if (matchClock.phase === 'over' && !S_PRACTICE.finalized) {
    S_PRACTICE.finalized = true;
    scorer.finalize(null, { red: hiveDyn.red.cycleStats().upCellCount, blue: hiveDyn.blue.cycleStats().upCellCount });
    const f = scorer.summary(); toast('Match over: red ' + f.red.total + ' / blue ' + f.blue.total + (f.winner === 'tie' ? ' (tie)' : '  winner ' + f.winner), 'ok');
  }
  if (pathEditor.playing) { pathEditor.step(dt); const pp = pathEditor.playPose; if (pp) { S.x = pp.x; S.y = pp.y; S.h = pp.h; S.vx = S.vy = S.om = 0; S.bodyV = [0, 0, 0]; } }
  const linkedLive = S.linked && S.ext && !autoActive();           // a running auto owns the robot, the robot link does not
  if (linkedLive) {
    const e = S.ext, p = e.pose;
    const nvx = (p.x - last.x) / Math.max(dt, 1e-3), nvy = (p.y - last.y) / Math.max(dt, 1e-3), nom = wrap(p.h - last.h) / Math.max(dt, 1e-3);
    S.vx += (nvx - S.vx) * 0.3; S.vy += (nvy - S.vy) * 0.3; S.om += (nom - S.om) * 0.3;
    S.x = p.x; S.y = p.y; S.h = p.h;
  } else if (!pathEditor.playing && !autoActive()) driveStep(dt);
  last = { x: S.x, y: S.y, h: S.h };

  S.sol = solveShot();
  if (linkedLive && S.ext.turretDeg !== undefined) {
    S.turretDeg = S.ext.turretDeg; S.hoodDeg = S.ext.hoodDeg ?? S.hoodDeg; S.rpm = S.ext.rpm ?? S.rpm; Object.assign(plant.state, { turretDeg: S.turretDeg, hoodDeg: S.hoodDeg, rpm: S.rpm });
    if (S.sol.ok) S.sol.turretCmd = S.sol.turretDeg;
  } else plantStep(dt, S.sol);
  S.eval = evaluateShot(evalCtx());
  updateRegion3d(); autoRunTick(); autoPlanner.tick(dt); autoShots(dt);

  // hive tip animation
  for (const a of ['red', 'blue']) {
    const h = hive[a]; hiveDyn[a].step(dt); h.shown = hiveDyn[a].angleDeg;
    h.group.rotation.x = rad(h.shown - h.angles[h.cad]);
  }
  updateBalls(dt);

  robot.position.set(S.x, S.y, 0); robot.rotation.z = S.h;
  turretG.rotation.z = rad(S.turretDeg);
  hoodPivot.rotation.y = -rad(S.hoodDeg);
  flowerArm.update(dt);
  { const fp = { x: S.x, y: S.y, h: S.h }; feedFromFlower(flowerArm, world, findFlowerContact(flowerArm, fp, sceneJson.constants.flowers), dt, { count: S_PRACTICE.ammo, cap: S_PRACTICE.ammoLimit ? S_PRACTICE.ammoCap : 99 }, fp); }
  const cr = Math.cos(S.h), sr = Math.sin(S.h);
  built.animate(dt, { vx: S.vx * cr + S.vy * sr, vy: -S.vx * sr + S.vy * cr, om: S.om, intakeOn: S_PRACTICE.intake, rpm: S.rpm, turretDeg: S.turretDeg, hoodDeg: S.hoodDeg, flowerArmDeg: flowerArm.angleDeg });
  updateCameraAndTags();
}
function frame() { requestAnimationFrame(frame); try { frameBody(); } catch (e) { diag.error(e, 'frame loop'); } }
function frameBody() {
  const dt = Math.min(clock.getDelta(), 0.05);
  simUpdate(dt);

  // trajectory preview
  const sol = S.sol;
  if (sol.ok && S.rpm > 300) {
    const pts = [], T = sol.T;
    for (let i = 0; i <= 40; i++) { const q = Core.trajectoryPoint(sol.P, sol.p.z0, sol.vs, sol.uh, sol.uz, T * i / 40); pts.push(new THREE.Vector3(q[0], q[1], q[2])); }
    trajLine.geometry.setFromPoints(pts); trajLine.visible = layers.traj[0].visible;
    landMark.position.set(sol.target.x, sol.target.y, sol.target.z); landMark.lookAt(landMark.position.clone().add(new THREE.Vector3(...sol.target.normal)));
  } else { trajLine.visible = false; }

  // camera modes
  if (S.cam === 'follow') {
    const d = new THREE.Vector3(-Math.cos(S.h), -Math.sin(S.h), 0).multiplyScalar(55);
    camera.position.lerp(new THREE.Vector3(S.x + d.x, S.y + d.y, 42), 0.12); controls.target.lerp(new THREE.Vector3(S.x, S.y, 10), 0.2);
  } else if (S.cam === 'pov') {
    const cp = cameraPose(); camera.fov = CAMERA.vfovDeg; camera.updateProjectionMatrix();
    camera.position.copy(cp.pos); camera.up.set(0, 0, 1); camera.lookAt(cp.pos.clone().add(cp.f));
  } else if (S.cam === 'driver' || S.cam === 'driver_wide' || S.cam === 'chase' || S.cam === 'coach') {
    const dc = driverCamera(S.alliance, S.cam === 'chase' ? 'robot_chase' : S.cam === 'coach' ? 'coach_box' : S.cam, S, S_PRACTICE.look);
    if (camera.fov !== dc.fov) { camera.fov = dc.fov; camera.updateProjectionMatrix(); }
    camera.up.set(0, 0, 1); camera.position.set(...dc.pos); camera.lookAt(...dc.target);
  } else if (camera.fov !== 40) { camera.fov = 40; camera.updateProjectionMatrix(); }
  if (!FIRST_PERSON.has(S.cam)) controls.update();

  S.fps = S.fps ? S.fps * 0.9 + (1 / Math.max(dt, 1e-3)) * 0.1 : 60;
  hudT += dt; if (hudT > 0.08) { hudT = 0; updateHud(); }
  if (S.view === '2d' || S.view === 'path' || S.view === 'auto') { drawMap(); return; }
  if (S.view === 'viz') return;
  const _t0 = performance.now(); renderer.render(scene, camera); const _t1 = performance.now(); if (window.__sync) renderer.getContext().finish(); const _t2 = performance.now(); labelRenderer.render(scene, camera); const _t3 = performance.now();
  S.perf = { cpuRender: _t1 - _t0, gpuWait: _t2 - _t1, labels: _t3 - _t2, frame: _t3 - _t0 };
}
// ---- hive tip HUD (left panel): pollen/nectar counts, progress bar, state per alliance ----
// hivetip_model.js is optional: a missing or broken module must not break the app.
let hiveModel = null;
try { hiveModel = await import('./hivetip_model.js'); } catch (e) { hiveModel = null; }
const HIVE_HUD_POLLEN_THRESHOLD = 8; // user-confirmed fact (manual is silent); used only if the model is absent
let hiveHudT = -Infinity;
function hiveHudCounts(d) {
  // 1) explicit count objects or numbers on the dynamics object
  if (d.counts && typeof d.counts === 'object') return { pollen: d.counts.pollen, nectar: d.counts.nectar };
  if (typeof d.pollen === 'number' || typeof d.nectar === 'number') return { pollen: d.pollen, nectar: d.nectar };
  // 2) per-ball data: scored balls flagged as counted in the up cell (starting nectar is not counted)
  if (Array.isArray(d.scored)) {
    let pollen = 0, nectar = 0;
    for (const b of d.scored) if (b && b.counts) { if (b.type === 'nectar') nectar++; else pollen++; }
    return { pollen, nectar };
  }
  return null;
}
function hiveHudState(d, progress) {
  if (d.moving) return 'tipping';
  if (progress >= 1) return 'tipped';
  return 'filling';
}
function updateHiveHud(now) {
  if (now - hiveHudT < 100) return;
  hiveHudT = now;
  for (const a of ['red', 'blue']) {
    const d = hiveDyn[a], counts = d && hiveHudCounts(d);
    const elCounts = $('hud-hive-' + a + '-counts'), elFill = $('hud-hive-' + a + '-fill'), elState = $('hud-hive-' + a + '-state');
    if (!elCounts || !elFill || !elState) continue;
    const box = $('hud-hive-' + a);
    if (!counts) {
      elCounts.textContent = 'no hive data'; elFill.style.width = '0%'; elState.textContent = 'unknown';
      if (box) box.dataset.state = 'unknown';
      continue;
    }
    const pollenKnown = Number.isFinite(Number(counts.pollen)), nectarKnown = Number.isFinite(Number(counts.nectar));
    const pollen = pollenKnown ? Math.max(0, Math.floor(Number(counts.pollen))) : null;
    const nectar = nectarKnown ? Math.max(0, Math.floor(Number(counts.nectar))) : null;
    // threshold and progress: model first, then the dynamics object, then the user-confirmed pollen count
    let progress = null, threshold = HIVE_HUD_POLLEN_THRESHOLD;
    if (hiveModel && hiveModel.HIVE_TIP && Number.isFinite(hiveModel.HIVE_TIP.threshold)) threshold = hiveModel.HIVE_TIP.threshold;
    else if (Number.isFinite(d.weightThreshold)) threshold = d.weightThreshold;
    else if (Number.isFinite(d.p && d.p.ballsToTip)) threshold = d.p.ballsToTip;
    if (pollen !== null && nectar !== null && hiveModel && typeof hiveModel.tipProgress === 'function') {
      progress = hiveModel.tipProgress({ pollen, nectar }, hiveModel.HIVE_TIP);
    } else if (typeof d.tipProgress === 'number' && Number.isFinite(d.tipProgress)) {
      progress = d.tipProgress;
    } else if (pollen !== null && threshold > 0) {
      // no model: pollen only (nectar has no threshold without the model, so it is not counted here)
      progress = pollen / threshold;
    }
    if (progress === null || !Number.isFinite(progress)) progress = 0;
    progress = Math.min(1, Math.max(0, progress));
    const nectarText = nectarKnown ? String(nectar) : '-';
    const pollenText = pollenKnown ? String(pollen) : '-';
    elCounts.textContent = `Pollen ${pollenText} / ${threshold}  Nectar ${nectarText}`;
    const nectarToTip = hiveModel && typeof hiveModel.nectarToTip === 'function' ? hiveModel.nectarToTip(hiveModel.HIVE_TIP) : null;
    elCounts.title = Number.isFinite(nectarToTip) ? `nectar alone tips at ${nectarToTip} (ASSUMED weight, see hivetip_model.js)` : 'nectar threshold: unknown';
    elFill.style.width = (progress * 100).toFixed(1) + '%';
    const state = hiveHudState(d, progress);
    elState.textContent = state;
    if (box) box.dataset.state = state;
  }
}
function updateHud() {
  const s = S.sol, f = (v, d = 1) => (Number.isFinite(v) ? v.toFixed(d) : '-');
  $('v-pose').textContent = `${f(S.x)}, ${f(S.y)}, ${f(deg(S.h), 0)}°`;
  $('v-speed').textContent = `${f(Math.hypot(S.vx, S.vy))} in/s`;
  $('hud-link').innerHTML = '<span class="dim">link</span><b class="' + (S.linked ? 'ok' : '') + '">' + (S.linked ? 'LINKED to robot' : 'local sim') + '</b><span class="dim">' + Math.round(S.fps || 0) + ' fps</span>';
  $('v-target').textContent = `${S.alliance} hive, ${stateName(hive[S.alliance].state).toLowerCase()}`;
  if (s.ok) {
    $('v-dist').textContent = `${f(s.d)} in / ${f(s.T, 2)} s`;
    $('v-turret').innerHTML = `set ${f(s.turretCmd ?? s.turretDeg)}° · now ${f(S.turretDeg)}°`;
    $('v-hood').textContent = `set ${f(deg(s.elev))}° · now ${f(S.hoodDeg)}°${s.clamped ? ' (clamped)' : ''}`;
    $('v-rpm').textContent = `set ${f(s.rpm, 0)} · now ${f(S.rpm, 0)}`;
    $('v-entry').textContent = `${f(s.entry)}°`;
    const ready = Math.abs(S.rpm - s.rpm) / s.rpm < 0.04 && Math.abs((s.turretCmd ?? s.turretDeg) - S.turretDeg) < 2;
  } else {
    ['v-dist', 'v-turret', 'v-hood', 'v-rpm', 'v-entry'].forEach(i => ($(i).textContent = '-'));
  }
  updateAutoPanel();
  updateHiveHud(performance.now());
  $('v-tags').innerHTML = (S.visibleTags || []).map(i => `<span class="tag vis">${i}</span>`).join('') || '<span class="dim">none</span>';
  $('v-tx').textContent = Number.isFinite(S.tx) ? S.tx.toFixed(1) + '°' : '-';
  $('v-score').textContent = `${S.shots} / ${S.scored}`;
  { const m = $('match'), sm = stats.summary(), ss = scorer.summary(), tot = { red: ss.red.total, blue: ss.blue.total }; m.style.display = matchClock.phase === 'idle' ? 'none' : 'block';
    m.innerHTML = `<b>${matchClock.phase.toUpperCase()}</b> &nbsp; <span style="font-size:20px">${matchClock.label}</span><br><span class="dim">score</span> red <b>${tot.red ?? 0}</b> · blue <b>${tot.blue ?? 0}</b> &nbsp; <span class="dim">balls</span> <b>${S_PRACTICE.ammo}/${S_PRACTICE.ammoCap}</b>`;
    $('pstats').textContent = `shots ${sm.shots ?? 0} · scored ${sm.scored ?? 0} · missed ${sm.misses} · balls ${S_PRACTICE.ammo}/${S_PRACTICE.ammoCap}` + (S_PRACTICE.fieldCentric ? ' · field-centric' : ''); }
  for (const id of Object.keys(tagInfo)) tagInfo[id].label.element.style.outline = tagInfo[id].visible ? '1px solid #5fe08a' : 'none';
}
window.__sim = { flowerArm, built, step: simUpdate, renderer, sun, S, world, hiveDyn, obstacles, matchClock, scorer, stats, autoPlanner, autoHost, pathEditor, S_PRACTICE, hive, camera, controls, setCam, tip, shoot: () => shoot(), S_AIM, get ballKind() { return S.ballKind; }, plant, manualAim, diag, evalCtx, evaluateShot, scene, layers };   // handy for debugging in the console
{ const h = (location.hash || '').slice(1); setView(['3d', 'auto', 'viz'].includes(h) ? h : '3d'); }   // #path, #auto ... deep-link a tab
requestAnimationFrame(frame);
