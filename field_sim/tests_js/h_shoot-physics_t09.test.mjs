// t09 (shoot-physics): deterministic fuzz of solver-velocity pollen shots at the RED hive through BallWorld at 1/60 s.
// Checks: finite state, field bounds, every capture reported via onCapture with a valid hive id, capture fraction in a band.
// Seeded LCG, no randomness outside it. Model: tests_js/shotaudit.test.mjs and tests_js/shotrules.test.mjs.
import test from 'node:test';
import assert from 'node:assert/strict';
import vm from 'node:vm';
import { readFileSync } from 'node:fs';
import { BallWorld } from '../web/js/balls.js';
import { parseHiveCollider } from '../web/js/hivemesh.js';
import * as SR from '../web/js/shotrules.js';

const root = {};
vm.runInNewContext(readFileSync(new URL('../web/js/shot_core.js', import.meta.url), 'utf8'), { window: root });
SR.setCore(root.Core);
const scene = JSON.parse(readFileSync(new URL('../web/data/scene.json', import.meta.url), 'utf8'));
const bin = readFileSync(new URL('../web/data/hive_collider.bin', import.meta.url));
const meshes = parseHiveCollider(bin.buffer.slice(bin.byteOffset, bin.byteOffset + bin.byteLength));
const bounds = { minX: 1.3, minY: 1.3, maxX: 142.7, maxY: 142.7 };
const g = scene.groups.hive_red, hc = scene.constants.hives.red;
const cells = hc.states[g.cad_pose_state].cells;
const env = { bounds, hives: { red: { pivot: g.pivot, cadAngle: g.cad_pose_angle_deg, cells, mesh: meshes.red } } };
const cell = Object.values(cells).find(c => c.up) || Object.values(cells)[0];
const target = { x: cell.aim_point[0], y: cell.aim_point[1], z: cell.aim_point[2], normal: cell.opening_normal };
const SH = { exitHeight: 12, wheelR: 1.89, eff: 0.45, rpmScale: 1, maxRpm: 6000, entryDeg: 50, hoodMin: 25, hoodMax: 65,
  pivotX: 0, pivotY: 0, turretMinDeg: -180, turretMaxDeg: 180 };
const rng = seed => () => (seed = (seed * 1664525 + 1013904223) >>> 0) / 4294967296;
const N = 300, SEED = 9, BAND = [0.007, 0.207];   // observed 0.1033 (31/300 still scored, 32 reported, seed 9); band = observed +-10 pp
const VALID_HIVES = new Set(['red', 'blue']);

function fire(R) {
  const shots = [];
  while (shots.length < N) {
    const pose = { x: 10 + R() * 124, y: 10 + R() * 124, h: R() * Math.PI * 2, vx: 0, vy: 0, om: 0 };
    if (Math.hypot(pose.x - g.pivot[0], pose.y - g.pivot[1]) < 30) continue;
    const sol = SR.solveAutoShot(pose, SH, target, 'pollen', root.Core);
    if (!sol.ok) continue;
    const ball = SR.launchBall({ pose, shooter: SH, rpm: sol.rpm, turretDeg: sol.turretDeg, hoodDeg: sol.elev * 180 / Math.PI, target, kind: 'pollen', robotVel: pose });
    shots.push(ball);
  }
  return shots;
}

test('t09: 300 solver-velocity pollen shots at red hive: finite, in field, captures reported, fraction in band', () => {
  const w = new BallWorld(env), R = rng(SEED), caps = [], shots = fire(R);
  w.onCapture = (b, hive, cell) => caps.push({ b, hive, cell });
  const made = shots.map(s => { const b = w.addBall('pollen', s.pos, s.vel); b.shot = true; return b; });
  for (let k = 0; k < 60 * 6; k++) w.step(1 / 60);
  for (const b of made) {
    assert.ok(Number.isFinite(b.x + b.y + b.z + b.vx + b.vy + b.vz), 'non-finite ball state');
    assert.ok(b.x > -2 && b.x < 146 && b.y > -2 && b.y < 146 && b.z > -2 && b.z < 200, `ball out of field ${b.x},${b.y},${b.z}`);
  }
  for (const c of caps) assert.ok(VALID_HIVES.has(c.hive), `invalid hive id ${c.hive}`);
  const scored = made.filter(b => b.state === 'scored').length;
  // a full cell recycles its oldest ball (removed from the world), so reports can exceed balls still scored; never fewer
  assert.ok(caps.length >= scored, 'every scored ball must be reported via onCapture');
  const reported = new Set(caps.map(c => c.b));
  for (const b of made) if (b.state === 'scored') assert.ok(reported.has(b), 'scored ball not reported');
  const frac = scored / made.length;
  console.log(`t09 observed: fired=${made.length} scored=${scored} fraction=${frac.toFixed(4)}`);
  assert.ok(frac >= BAND[0] && frac <= BAND[1], `capture fraction ${frac} outside band ${BAND}`);
});
