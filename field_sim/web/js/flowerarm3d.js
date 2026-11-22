// Drop-down flower arm model (VEX Push Back loader style). Stowed, the arm stands upright at the front arch (inside the
// 18 in envelope, scoop paddle hanging against it); deployed it drops forward-down over the bumper and the scoop paddle hangs
// over a flower. cfg.side 'rear' mirrors it to the back. group is added to the robot group at the hinge.
// hinge rotates about the robot lateral axis: hinge.rotation.y = -rad(angleDeg); angle 0 = forward, 90 = up, negative = lowered.
// A wrist joint (a 4-bar linkage in real life) keeps the paddle hanging straight down at every angle.

/** Mirror of data/flower_arm.json geometry, used when the host does not pass the loaded json. */
export const DEFAULT_ARM_CFG = {
  side: 'front',
  hinge: { x: 8.0, y: 0, z: 9.2 },
  arm: { length: 8.2, width: 9, linkSpacing: 10.8, linkThickness: 0.5, linkDepth: 1.2 },
  angles: { stowedDeg: 90, deployedDeg: -35 },
  timing: { deployS: 0.45, stowS: 0.45 },
  paddle: { width: 8, height: 3.2, thickness: 0.35, rubberThickness: 0.25 },
};

/** Wrist angle (deg, same sense as the arm angle): always the opposite of the arm so the paddle hangs straight down. */
export function wristDegFor(cfg, angleDeg) { return -angleDeg; }

export function buildFlowerArm(THREE, cfg = DEFAULT_ARM_CFG) {
  const PI = Math.PI, std = (c, r, m) => new THREE.MeshStandardMaterial({ color: c, roughness: r, metalness: m });
  const M = { alu: std(0xc4cad4, 0.35, 0.85), anod: std(0x3a404c, 0.45, 0.7), orange: std(0xff7a1a, 0.4, 0.3), black: std(0x1a1d24, 0.55, 0.4), rubber: std(0x24262c, 0.85, 0.05), steel: std(0x9aa0aa, 0.3, 0.9) };
  const H = cfg.hinge, L = cfg.arm.length, sp = cfg.arm.linkSpacing, lt = cfg.arm.linkThickness, ld = cfg.arm.linkDepth;
  const P = cfg.paddle, rear = cfg.side === 'rear';
  const group = new THREE.Group(); group.name = 'flowerArm';
  group.position.set((rear ? -1 : 1) * H.x, H.y, H.z);
  if (rear) group.rotation.z = PI;
  const hinge = new THREE.Object3D(); hinge.name = 'flowerArmHinge'; group.add(hinge);
  const wrist = new THREE.Object3D(); wrist.name = 'flowerArmWrist'; wrist.position.set(L, 0, 0); hinge.add(wrist);
  const parts = { links: [], bracket: [], servos: [], horns: [], wrist };
  const box = (sx, sy, sz, mat, x, y, z, parent, name) => {
    const m = new THREE.Mesh(new THREE.BoxGeometry(sx, sy, sz), mat); m.position.set(x, y, z); m.name = name;
    m.castShadow = true; m.receiveShadow = true; parent.add(m); return m;
  };
  const cyl = (r, len, mat, x, y, z, parent, name, axis = 'y') => {
    const m = new THREE.Mesh(new THREE.CylinderGeometry(r, r, len, 16), mat); m.position.set(x, y, z); m.name = name;
    if (axis === 'x') m.rotation.z = PI / 2; if (axis === 'z') m.rotation.x = PI / 2;
    m.castShadow = true; parent.add(m); return m;
  };
  for (const s of [-1, 1]) {
    const y = s * sp / 2;
    // C-channel link: web + two flanges
    parts.links.push(box(L, lt, ld, M.alu, L / 2, y, 0, hinge, 'link'));
    box(L, lt * 2.2, lt * 0.5, M.alu, L / 2, y + s * lt * 0.7, ld / 2 - lt * 0.25, hinge, 'flange');
    box(L, lt * 2.2, lt * 0.5, M.alu, L / 2, y + s * lt * 0.7, -ld / 2 + lt * 0.25, hinge, 'flange');
    cyl(1.0, lt * 1.4, M.black, 0, y, 0, hinge, 'boss');
    cyl(0.5, lt * 1.6, M.steel, L, y, 0, hinge, 'wristBoss');
    // servo horn (turns with the arm), servo body and bracket stay fixed to the robot
    parts.horns.push(cyl(0.8, 0.12, M.orange, 0, s * (sp / 2 + lt * 0.9), 0, hinge, 'horn'));
    parts.bracket.push(box(3.8, 0.3, 1.8, M.anod, -1.6, s * (sp / 2 + lt + 0.5), 0.27, group, 'bracket'));   // arch plates standing on the cassette rails
    parts.servos.push(box(1.5, 0.8, 1.5, M.black, 0, s * (sp / 2 + lt + 1.0), 0, group, 'axon'));
  }
  cyl(0.22, sp + lt * 3, M.steel, 0, 0, 0, hinge, 'pivotAxle');
  parts.crossbar = box(0.8, sp + lt, 0.8, M.alu, L * 0.55, 0, 0, hinge, 'crossbar');
  parts.linkage = box(L * 0.45, 0.3, 0.3, M.alu, L * 0.8, sp / 2 - 1, 1.1, hinge, 'linkageRod');
  box(0.4, sp + 2.4, 0.4, M.anod, -3.4, 0, -0.3, group, 'baseplate');
  // paddle (scoop) on the wrist: hangs along local -z, back plate + rubber lip + side cheeks forming a pocket
  parts.paddle = box(P.thickness, P.width, P.height, M.orange, 0, 0, -P.height / 2 + 0.4, wrist, 'paddle');
  parts.rubber = box(P.thickness + 0.1, P.width, P.rubberThickness, M.rubber, 0, 0, -P.height + 0.4 - P.rubberThickness / 2, wrist, 'rubber');
  for (const s of [-1, 1]) box(1.1, 0.25, P.height * 0.8, M.orange, 0.55, s * (P.width / 2 - 0.12), -P.height * 0.45 + 0.4, wrist, 'cheek');
  box(0.6, P.width, 0.3, M.orange, 0.3, 0, 0.4, wrist, 'paddleTop');

  const setAngle = deg => {
    hinge.rotation.y = -deg * PI / 180;
    wrist.rotation.y = -wristDegFor(cfg, deg) * PI / 180;
  };
  /** 0 = stowed .. 1 = deployed */
  const setFraction = f => { const a = cfg.angles; setAngle(a.stowedDeg + (a.deployedDeg - a.stowedDeg) * Math.min(1, Math.max(0, f))); };
  /** Follow a FlowerArm logic object (flowerarm.js). */
  const sync = arm => setAngle(arm.angleDeg);
  setAngle(cfg.angles.stowedDeg);
  return { group, hinge, wrist, setAngle, setFraction, sync, parts, cfg };
}
