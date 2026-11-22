// Ball kinds in the shooting path: pollen and nectar launch, held FIFO with a mixed hold, and the tip-count helper.
// Helpers are local copies (test files do not import each other). Units: inches, seconds.
import test from 'node:test';
import assert from 'node:assert/strict';
import vm from 'node:vm';
import { readFileSync } from 'node:fs';
import { BallWorld, BALL, BALL_KINDS, launchScaleFor, MAX_HELD, tipReady } from '../web/js/balls.js';
import * as SR from '../web/js/shotrules.js';

const rd = (p) => readFileSync(new URL(p, import.meta.url), 'utf8');
const field = JSON.parse(rd('../web/data/colliders.json'));
const root = {}; vm.runInNewContext(rd('../web/js/shot_core.js'), { window: root });
SR.setCore(root.Core);
const SH = { exitHeight: 18.4, wheelR: 1.89, eff: 0.45, rpmScale: 1, maxRpm: 6000, entryDeg: 50, hoodMin: 25, hoodMax: 65, pivotX: 0, pivotY: 0, turretMinDeg: -180, turretMaxDeg: 180 };
const DIMS = { halfX: 9, halfY: 9, height: 18 }, DT = 1 / 60;
const TG = { x: 130, y: 72, z: 50, normal: [-1, 0, 0] };

test('kinds: radii come from the kind table (pollen 2.78 in, nectar 3.6 in) and pollen is the launch reference', () => {
  assert.ok(Math.abs(BALL_KINDS.pollen.radiusIn - 1.391) < 1e-9);
  assert.ok(Math.abs(BALL_KINDS.nectar.radiusIn - 1.7995) < 1e-9);
  assert.equal(launchScaleFor('pollen'), 1);
  const s = launchScaleFor('nectar');
  assert.ok(s > 0.77 && s < 0.79, 'nectar launch scale ' + s);   // sqrt(0.025 / 0.041), ASSUMED mass scaling
  assert.equal(new BallWorld({ bounds: field.bounds })._mk('nectar', [0, 0, 0], [0, 0, 0], undefined, undefined, 'nectar').r, BALL_KINDS.nectar.radiusIn);
});

test('kinds: the same rpm launches a nectar ball slower than a pollen ball by the mass-scale factor', () => {
  const pose = { x: 60, y: 72, h: 0, vx: 0, vy: 0, om: 0 };
  const args = { pose, shooter: SH, target: TG, turretDeg: 0, hoodDeg: 35, rpm: 2400 };
  const p = SR.forwardShot({ ...args, kind: 'pollen' }), n = SR.forwardShot({ ...args, kind: 'nectar' });
  assert.equal(n.kind, 'nectar'); assert.equal(p.kind, 'pollen');
  assert.ok(Math.abs(n.speed / p.speed - launchScaleFor('nectar')) < 1e-9, 'speed ratio ' + n.speed / p.speed);
  assert.ok(Math.abs(n.ballR - BALL_KINDS.nectar.radiusIn) < 1e-9, 'nectar rim clearance uses the nectar radius');
});

test('kinds: a nectar shot solved for nectar and fired with ctx.kind nectar lands at the solver speed', () => {
  // The solver raises rpm for nectar so the exit speed matches pollen's. Firing it as a pollen (what app.js evalCtx does
  // today: no kind) gives 1/launchScale times the speed, which is the miss seen in the sim.
  const pose = { x: 60, y: 72, h: 0, vx: 0, vy: 0, om: 0 };
  const sol = SR.solveAutoShot(pose, SH, TG, 'nectar', root.Core);
  assert.equal(sol.ok, true, 'no nectar solution');
  const ok = SR.forwardShot({ pose, shooter: SH, target: TG, turretDeg: sol.turretDeg, hoodDeg: sol.elev * 180 / Math.PI, rpm: sol.rpm, kind: 'nectar' });
  const wrong = SR.forwardShot({ pose, shooter: SH, target: TG, turretDeg: sol.turretDeg, hoodDeg: sol.elev * 180 / Math.PI, rpm: sol.rpm, kind: 'pollen' });
  assert.ok(Math.abs(ok.speed - sol.speed) < 1e-6, 'nectar fire speed ' + ok.speed + ' vs solver ' + sol.speed);
  assert.ok(Math.abs(wrong.speed / ok.speed - 1 / launchScaleFor('nectar')) < 1e-9, 'kind mismatch factor');
});

test('kinds: fireHeld keeps the kind and type of the oldest held ball and does not re-intake it', () => {
  const w = new BallWorld({ bounds: field.bounds, boxes: field.boxes, circles: field.circles });
  w.reset([]); w.setIntake({ enabled: true, space: Infinity });
  w.held.push({ id: 'h1', type: 'nectar', kind: 'nectar' }, { id: 'h2', type: 'pollen', kind: 'pollen' });
  const pose = { x: 60, y: 72, h: 0, vx: 0, vy: 0, om: 0 };
  const b = w.fireHeld([70, 72, 18.4], [200, 0, 0]);
  assert.equal(b.kind, 'nectar'); assert.equal(b.type, 'nectar');
  assert.equal(b.r, BALL_KINDS.nectar.radiusIn);
  assert.equal(b.shot, true);
  assert.deepEqual(w.heldKindCounts(), { pollen: 1, nectar: 0 });
  for (let i = 0; i < 30; i++) { w.setRobot(pose, DIMS); w.step(DT); }
  assert.notEqual(b.state, 'intaken');
  assert.equal(w.held.length, 1);
});

test('kinds: mixed pollen + nectar hold is capped at MAX_HELD (4) combined and the cap is 4', () => {
  assert.equal(MAX_HELD, 4);
  const w = new BallWorld({ bounds: field.bounds, boxes: field.boxes, circles: field.circles });
  w.reset([]); w.setIntake({ enabled: true, space: Infinity });
  const pose = { x: 60, y: 72, h: 0, vx: 0, vy: 0, om: 0 };
  const kinds = ['pollen', 'nectar', 'pollen', 'nectar', 'nectar', 'pollen'];
  kinds.forEach((k, i) => w.balls.push(w._mk(k, [71 + (i % 2) * 0.5, 69 + i * 2.4, k === 'nectar' ? BALL_KINDS.nectar.radiusIn : BALL.r], [0, 0, 0], undefined, undefined, k)));
  for (let i = 0; i < 900 && w.held.length < MAX_HELD; i++) { w.setRobot(pose, DIMS); w.step(DT); }
  for (let i = 0; i < 300; i++) { w.setRobot(pose, DIMS); w.step(DT); }
  assert.equal(w.held.length, MAX_HELD, 'held ' + w.held.length);
  const c = w.heldKindCounts();
  assert.equal(c.pollen + c.nectar, MAX_HELD);
  assert.equal(w.balls.filter(b => b.state === 'free').length, 2, 'the 2 balls over the hold limit must stay on the field');
});

test('kinds: tipReady (8 pollen, or pollen + 1.65 per nectar >= 8) for mixed combinations', () => {
  assert.equal(tipReady({ pollen: 8, nectar: 0 }), true);
  assert.equal(tipReady({ pollen: 7, nectar: 0 }), false);
  assert.equal(tipReady({ pollen: 4, nectar: 2 }), false);   // 4 + 2 * 1.654 = 7.31
  assert.equal(tipReady({ pollen: 4, nectar: 3 }), true);    // 4 + 3 * 1.654 = 8.96
  assert.equal(tipReady({ pollen: 0, nectar: 5 }), true);    // 5 * 1.654 = 8.27
  assert.equal(tipReady({ pollen: 0, nectar: 4 }), false);   // 6.62
});
