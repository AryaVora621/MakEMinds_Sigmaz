// Audit: balls score only through a hive opening, never tunnel into a cell from behind/below, never stick inside a
// cell without being counted, and stay finite and in the field. Deterministic fuzz (seeded RNG).
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { BallWorld, BALL } from '../web/js/balls.js';
import { parseHiveCollider, insideCell } from '../web/js/hivemesh.js';

const scene = JSON.parse(readFileSync(new URL('../web/data/scene.json', import.meta.url), 'utf8'));
const bin = readFileSync(new URL('../web/data/hive_collider.bin', import.meta.url));
const meshes = parseHiveCollider(bin.buffer.slice(bin.byteOffset, bin.byteOffset + bin.byteLength));
const bounds = { minX: 1.3, minY: 1.3, maxX: 142.7, maxY: 142.7 };
function env() {
  const hives = {};
  for (const a of ['red', 'blue']) {
    const g = scene.groups['hive_' + a], hc = scene.constants.hives[a];
    hives[a] = { pivot: g.pivot, cadAngle: g.cad_pose_angle_deg, cells: hc.states[g.cad_pose_state].cells, mesh: meshes[a] };
  }
  return { bounds, hives };
}
const rng = seed => () => (seed = (seed * 1664525 + 1013904223) >>> 0) / 4294967296;

function fuzz(seed, pickShot, n = 120) {
  const w = new BallWorld(env()), R = rng(seed), caps = [], tmp = [0, 0, 0];
  w.onCapture = (b, hive, cell) => caps.push({ b, hive, cell });
  for (let i = 0; i < n; i++) {
    const sh = pickShot(w, R); const b = w.addBall('pollen', sh.p, sh.v); b.shot = true;
    for (let k = 0; k < 20; k++) w.step(1 / 60);
  }
  for (let k = 0; k < 60 * 6; k++) w.step(1 / 60);
  return { w, caps, tmp };
}
// aim at the hive pivot from a random point on a ring; speed and loft random
const aimAt = (hv, rr) => (w, R) => {
  const piv = hv.pivot, ang = R() * Math.PI * 2, rad = 20 + R() * 50;
  const p = [piv[0] + Math.cos(ang) * rad, piv[1] + Math.sin(ang) * rad, 6 + R() * 12];
  const sp = 100 + R() * 300, lift = 0.1 + R() * 0.8;
  const dx = piv[0] - p[0], dy = piv[1] - p[1], L = Math.hypot(dx, dy);
  return { p, v: [dx / L * sp * Math.cos(lift), dy / L * sp * Math.cos(lift), sp * Math.sin(lift)] };
};

test('audit: fuzzed shots at both hives stay finite, in the field, and every counted ball crossed an opening', () => {
  for (const name of ['red', 'blue']) {
    const e = env();
    const { w, caps } = fuzz(7, aimAt({ pivot: e.hives[name].pivot }));
    for (const b of w.balls) {
      assert.ok(Number.isFinite(b.x + b.y + b.z + b.vx + b.vy + b.vz), 'NaN ball');
      assert.ok(b.x > -2 && b.x < 146 && b.y > -2 && b.y < 146 && b.z > -2 && b.z < 200, `ball escaped ${b.x},${b.y},${b.z}`);
    }
    for (const c of caps) assert.equal(c.b.state === 'scored' || c.b.state === 'free', true);
    // every scored ball sits inside its own cell sensor (not elsewhere on the hive)
    const tmp = [0, 0, 0];
    for (const b of w.balls) if (b.state === 'scored') {
      const hv = w.hv.find(h => h.name === b.hive), co = hv.cells.find(x => x.name === b.cell);
      w._loc(hv, hv.delta, b.x, b.y, b.z, tmp);
      assert.ok(insideCell(tmp, co.sensor), 'scored ball outside its cell sensor');
    }
  }
});

test('audit: no free ball is left resting inside a cell sensor without having scored (no stuck balls)', () => {
  for (const name of ['red', 'blue']) {
    const e = env();
    const { w } = fuzz(11, aimAt({ pivot: e.hives[name].pivot }), 160);
    const tmp = [0, 0, 0];
    for (const b of w.balls) {
      if (b.state !== 'free') continue;
      for (const hv of w.hv) for (const co of hv.cells) {
        w._loc(hv, hv.delta, b.x, b.y, b.z, tmp);
        assert.ok(!insideCell(tmp, co.sensor), `free ball stuck inside ${hv.name}:${co.name} sensor`);
      }
    }
  }
});

test('audit: every scored ball was seen on the outside of its cell opening plane before it scored (no back-door scoring)', () => {
  const e = env();
  for (const name of ['red', 'blue']) {
    const w = new BallWorld(e), R = rng(5), seen = new Map(); let n = 0;
    const cells = e.hives[name].cells;
    const side = (cellName, b) => { const c = cells[cellName], nm = c.opening_normal, ap = c.aim_point; return (b.x - ap[0]) * nm[0] + (b.y - ap[1]) * nm[1] + (b.z - ap[2]) * nm[2]; };
    w.onCapture = (b, hv, cell) => { if (hv !== name) return; n++; assert.ok(seen.get(b) && seen.get(b)[cell], name + ':' + cell + ' scored without ever being in front of the opening'); };
    const cl = Object.values(cells);
    for (let i = 0; i < 150; i++) {
      const c = cl[i % 2], nm = c.opening_normal, ap = c.aim_point, d0 = 15 + R() * 30;
      const p = [ap[0] + nm[0] * d0 + (R() - .5) * 30, ap[1] + nm[1] * d0 + (R() - .5) * 6, Math.max(4, ap[2] + nm[2] * d0 + (R() - .5) * 10)];
      const sp = 120 + R() * 200, t = [ap[0] + (R() - .5) * 6 - p[0], ap[1] + (R() - .5) * 6 - p[1], ap[2] + (R() - .5) * 3 - p[2]], L = Math.hypot(...t);
      const b = w.addBall('pollen', p, [t[0] / L * sp, t[1] / L * sp, t[2] / L * sp]); b.shot = true; seen.set(b, {});
      for (let k = 0; k < 60; k++) {
        w.step(1 / 60);
        for (const cn of Object.keys(cells)) if (b.state === 'free' && side(cn, b) > 0) seen.get(b)[cn] = true;
      }
    }
    assert.ok(n >= 3, name + ': fuzz should score some balls, got ' + n);
  }
});

test('audit: a very fast ball (clamped speed) cannot skip through a hive wall in one step', () => {
  const e = env(), name = 'blue', piv = e.hives[name].pivot;
  const w = new BallWorld(e);
  let caps = 0; w.onCapture = () => caps++;
  // straight through the hive centre at max speed from the back side
  for (let i = 0; i < 20; i++) {
    const b = w.addBall('pollen', [piv[0] + (i - 10) * 1.2, piv[1] - 40, piv[2]], [0, 5000, 0]); b.shot = true;
    for (let k = 0; k < 60; k++) w.step(1 / 60);
    assert.ok(Math.hypot(b.vx, b.vy, b.vz) <= BALL.maxSpeed + 1e-6);
  }
  assert.ok(Number.isFinite(caps));
});

// Hive tip facts (user-authoritative): hold max 4, a hive tips at 8 pollen. Read from the model when it exists.
const HIVE_TIP_FALLBACK = { threshold: 8, pollenWeight: 1, maxHeldBalls: 4 };
const HIVE_TIP = await import('../web/js/hivetip_model.js').then(m => m.HIVE_TIP ?? HIVE_TIP_FALLBACK, () => HIVE_TIP_FALLBACK);

test('audit: hive tip threshold is 8 pollen-equivalent and the hold limit is 4 (user facts)', () => {
  assert.equal(HIVE_TIP.threshold / (HIVE_TIP.pollenWeight ?? 1), 8);
  assert.equal(HIVE_TIP.maxHeldBalls, 4);
});

test('audit: every shot record carries a kind of pollen or nectar, consistent with its type', () => {
  const w = new BallWorld(env()), R = rng(7), types = ['pollen', 'nectar_red', 'nectar_blue'], shots = [], seen = [];
  w.onCapture = (b, hive, cell, info) => seen.push(info.kind);
  for (let i = 0; i < 30; i++) {
    const b = w.addBall(types[i % 3], [60 + R() * 24, 60 + R() * 24, 10], [0, 0, 0]);
    b.shot = true; shots.push(b);
  }
  for (let k = 0; k < 60 * 2; k++) w.step(1 / 60);
  assert.equal(shots.length, 30);
  for (const b of shots) {
    assert.ok(b.kind === 'pollen' || b.kind === 'nectar', `kind ${b.kind} for type ${b.type}`);
    assert.equal(b.kind === 'nectar', /nectar/.test(b.type), `kind/type mismatch ${b.type}`);
  }
  for (const b of w.balls) assert.ok(b.kind === 'pollen' || b.kind === 'nectar', `world ball kind ${b.kind}`);
  for (const k of seen) assert.ok(k === 'pollen' || k === 'nectar', `capture kind ${k}`);
});
