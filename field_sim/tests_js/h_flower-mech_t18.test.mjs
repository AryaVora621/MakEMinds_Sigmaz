// h_flower-mech_t18: full physics flower-to-intake integration on the REAL BallWorld (no app.js).
// Intake capture runs inside BallWorld.step (_intakeStep), so no landing-position fallback is needed.
// Failing-by-design findings: none declared. A failure is a real defect; the assert message names the invariant.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { BallWorld, MAX_HELD } from '../web/js/balls.js';
import { FlowerArm, findFlowerContact, feedFromFlower } from '../web/js/flowerarm.js';

const rd = p => JSON.parse(readFileSync(new URL(p, import.meta.url), 'utf8'));
const cfg = rd('../web/data/flower_arm.json'), scene = rd('../web/data/scene.json'), flowers = scene.constants.flowers;
const bounds = { minX: 0, minY: 0, maxX: 144, maxY: 144 }, DIMS = { halfX: 9, halfY: 9, height: 18 };
const DT = 1 / 120, STEPS = 360;   // 3 simulated seconds
const DIR = { far: [0, 1], audience: [0, -1], red_side: [-1, 0], blue_side: [1, 0] };
const inField = b => b.x >= bounds.minX && b.x <= bounds.maxX && b.y >= bounds.minY && b.y <= bounds.maxY;
// Ball centre inside the flower scoring volume (flowerarm.js volumeMarginIn margin).
const inVolume = (b, fl) => {
  const bb = fl.scoring_volume_bbox, zr = fl.scoring_volume_z, m = cfg.feed.volumeMarginIn;
  return b.x >= bb[0][0] - m && b.x <= bb[1][0] + m && b.y >= bb[0][1] - m && b.y <= bb[1][1] + m && b.z <= zr[1] + b.r;
};

test('flower-to-intake: balls leave the flower, are held (<=4), none lost, none tunnel the boundary', () => {
  const fl = flowers[0];
  // Pose: tip of the DEPLOYED arm over the opening, robot facing it (construction from flowerarm.test.mjs).
  const ref = new FlowerArm(cfg); ref.deploy(); for (let i = 0; i < 40; i++) ref.update(0.02);
  const tx = ref.tipRobot()[0], dir = DIR[fl.wall], h = Math.atan2(dir[1], dir[0]);
  const pose = { x: fl.opening_center[0] - dir[0] * tx, y: fl.opening_center[1] - dir[1] * tx, h };

  const arm = new FlowerArm(cfg), w = new BallWorld({ bounds });
  w.reset(scene.balls.balls);
  w.setIntake({ enabled: true, space: Infinity });
  arm.deploy();   // real deploy runs inside simulated time, from stowed
  const total0 = w.balls.length, vol0 = w.balls.filter(b => inVolume(b, fl)).length;
  const startOut = new Set(w.balls.filter(b => !inField(b)).map(b => b.id));   // start balls already outside the field are skipped
  let released = 0, maxHeld = 0; const badTotal = [], escaped = [];

  for (let i = 0; i < STEPS; i++) {
    arm.update(DT);
    const contact = findFlowerContact(arm, pose, flowers);
    released += feedFromFlower(arm, w, contact, DT, { count: w.held.length, cap: 99 }, pose).released;
    w.setRobot(pose, DIMS);
    w.step(DT);
    maxHeld = Math.max(maxHeld, w.held.length);
    if (w.held.length > MAX_HELD) assert.fail(`held ${w.held.length} > MAX_HELD ${MAX_HELD} at step ${i}`);
    if (w.balls.length + w.held.length !== total0) badTotal.push(`step ${i}: ${w.balls.length}+${w.held.length}`);
    for (const b of w.balls) {
      if (!startOut.has(b.id) && (!inField(b) || b.z < -0.01)) escaped.push(`${b.id} (${b.x.toFixed(2)},${b.y.toFixed(2)},${b.z.toFixed(2)}) step ${i}`);
    }
  }

  const vol1 = w.balls.filter(b => inVolume(b, fl)).length;
  console.log(`flower ${fl.id}: released=${released} inVolume ${vol0}->${vol1} held=${w.held.length} maxHeld=${maxHeld} taken=${w.intake.taken}`);
  assert.ok(released >= 1, 'feedFromFlower released no ball');
  assert.ok(vol1 < vol0, `no ball left the flower volume (${vol0} -> ${vol1})`);
  assert.ok(maxHeld >= 1, 'no ball was ever held by the robot');
  assert.ok(maxHeld <= MAX_HELD, `held count ${maxHeld} exceeded MAX_HELD ${MAX_HELD}`);
  assert.deepEqual(badTotal.slice(0, 3), [], 'ball count not conserved (balls + held != start count)');
  assert.deepEqual(escaped.slice(0, 3), [], 'ball escaped the field boundary');
});
