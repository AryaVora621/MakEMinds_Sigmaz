// a10: balls launched into the UP cell, then the hive tips. Physics runs 5 s. Checks: no NaN, no ball below the floor,
// no ball outside the field. Reports where each ball ends (down cell, or released out of the opening).
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { BallWorld, BALL } from '../web/js/balls.js';
import { parseHiveCollider, cellSensors, insideCell } from '../web/js/hivemesh.js';
import { HiveDynamics, HIVE_PARAMS } from '../web/js/hivecycle.js';

const scene = JSON.parse(fs.readFileSync(new URL('../web/data/scene.json', import.meta.url), 'utf8'));
const bin = fs.readFileSync(new URL('../web/data/hive_collider.bin', import.meta.url));
const meshes = parseHiveCollider(bin.buffer.slice(bin.byteOffset, bin.byteOffset + bin.byteLength));
const bounds = { minX: 1.3, minY: 1.3, maxX: 142.7, maxY: 142.7 };
const g = scene.groups.hive_red, hc = scene.constants.hives.red, cells = hc.states[g.cad_pose_state].cells;
const env = { bounds, hives: { red: { pivot: g.pivot, cadAngle: g.cad_pose_angle_deg, cells, mesh: meshes.red } } };
const sensors = cellSensors({ pivot: g.pivot, cells });
const RAD = Math.PI / 180, G = BALL.g;
const upName = cells.audience.opening_normal[2] > 0 ? 'audience' : 'far';   // raised cell at CAD pose (as hivecycle test)
const cell = cells[upName];

// Aimed approach from outside the raised opening (same construction as the hive entry tests in hivecycle/balls tests).
function approach(S, T, ox) {
  const ap = cell.aim_point, n = cell.opening_normal, uy = [0, n[2], -n[1]];
  const tgt = [ap[0] + ox, ap[1], ap[2]];
  const from = [tgt[0] + n[0] * S, tgt[1] + n[1] * S, tgt[2] + n[2] * S];
  return { from, v: [(tgt[0] - from[0]) / T, (tgt[1] - from[1]) / T, (tgt[2] - from[2]) / T + 0.5 * G * T] };
}

test('a10: balls in the up cell stay in the down cell after a tip (or release out the opening); no NaN, floor or field escape', (t) => {
  const w = new BallWorld(env), dyn = new HiveDynamics(g, hc.states, { ...HIVE_PARAMS });
  w.onCapture = (b, al, c, info) => dyn.addScoredBall(c, [b.x, b.y, b.z], info.kind);
  const dt = 1 / 120;
  const tick = () => { dyn.step(dt); w.setHiveAngles({ red: dyn.angleDeg }); w.step(dt); };
  // 8 pollen, staggered so each lands before the next; the 8th starts the tip (8 pollen tip a hive, user fact).
  const offs = [0, 3, -3, 5, -5, 2, -2, 4];
  offs.forEach((ox) => { const { from, v } = approach(70, 0.45, ox); w.addBall('pollen', from, v); for (let i = 0; i < 240; i++) tick(); });
  assert.equal(dyn.cycleStats().tips, 1, 'the hive tipped once');
  for (let i = 0; i < 5 / dt; i++) tick();   // 5 simulated seconds after the tip starts
  assert.equal(dyn.state, 'far_up');

  const down = dyn.upCell === 'far' ? 'audience' : 'far';
  const dl = (dyn.angleDeg - g.cad_pose_angle_deg) * RAD, c = Math.cos(dl), s = Math.sin(dl), piv = g.pivot;
  const where = { down: 0, up: 0, released: 0 };
  for (const b of w.balls) {
    assert.ok([b.x, b.y, b.z].every(Number.isFinite), 'NaN position');
    assert.ok(b.z > -b.r, `below floor z=${b.z}`);
    assert.ok(b.x >= 0 && b.x <= 144 && b.y >= 0 && b.y <= 144, `outside field (${b.x}, ${b.y})`);
    const dy = b.y - piv[1], dz = b.z - piv[2];
    const q = [b.x - piv[0], dy * c + dz * s, -dy * s + dz * c];   // hive-local, same frame as balls.js _loc
    if (insideCell(q, sensors[down])) where.down++;
    else if (insideCell(q, sensors[dyn.upCell])) where.up++;
    else where.released++;
  }
  t.diagnostic(`balls=${w.balls.length} down(${down})=${where.down} up=${where.up} released=${where.released}`);
  assert.ok(w.balls.length > 0, 'no balls left in the world');
});
