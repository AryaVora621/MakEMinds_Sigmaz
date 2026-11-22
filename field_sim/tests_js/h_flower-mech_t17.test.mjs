// feedFromFlower (web/js/flowerarm.js): order, cadence, hopper cap, pending, skip rules.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { FlowerArm, feedFromFlower } from '../web/js/flowerarm.js';

const flower = { opening_center: [0, 0], scoring_volume_bbox: [[-5, -5], [5, 5]], scoring_volume_z: [0, 10] };
const contact = { flower, distance: 0, alignmentDeg: 0 }, pose = { x: 0, y: 0, h: 0 };
const ball = (id, z, o = {}) => ({ id, x: 0, y: 0, z, r: 0.5, type: 'pollen', state: 'held', ...o });
const world = balls => ({ balls, released: [], release(b) { this.released.push(b.id); b.state = 'free'; b.x = 999; } }); // fake release moves ball out of volume
const hopper = (count = 0) => ({ count, cap: 4 });
const deployed = cfg => { const a = new FlowerArm(cfg); a.deploy(); a.update(1); return a; };
const IV = new FlowerArm().cfg.feed.feedIntervalS;

test('top ball is released first', () => {
  const w = world([ball('low', 2), ball('high', 8), ball('mid', 5)]);
  assert.equal(feedFromFlower(deployed(), w, contact, IV, hopper(), pose).released, 1);
  assert.deepEqual(w.released, ['high']);
});
test('cadence equals feedIntervalS; a 2*IV frame releases two', () => {
  const arm = deployed(), w = world([ball('a', 9), ball('b', 7), ball('c', 5)]);
  const step = dt => feedFromFlower(arm, w, contact, dt, hopper(), pose).released;
  assert.deepEqual([step(IV - 0.05), step(0.05), step(IV - 0.01), step(0.01)], [0, 1, 0, 1]);
  const w2 = world([ball('x', 9), ball('y', 7)]);
  assert.equal(feedFromFlower(deployed(), w2, contact, 2 * IV, hopper(), pose).released, 2);
});
test('hopper cap 4: count 3 releases at most 1 (pending counted); count 4 releases 0', () => {
  const w = world([ball('a', 9), ball('b', 7)]);
  assert.equal(feedFromFlower(deployed(), w, contact, 3 * IV, hopper(3), pose).released, 1);
  const w4 = world([ball('a', 9)]);
  assert.equal(feedFromFlower(deployed(), w4, contact, 3 * IV, hopper(4), pose).released, 0);
  assert.deepEqual(w4.released, []);
});
test('maxBallsPerDeploy caps each deploy and resets on re-deploy', () => {
  const arm = deployed({ feed: { maxBallsPerDeploy: 2 } });
  const w = world([1, 2, 3, 4, 5, 6].map(z => ball('z' + z, z)));
  assert.equal(feedFromFlower(arm, w, contact, 6 * IV, hopper(), pose).released, 2);
  assert.equal(feedFromFlower(arm, w, contact, IV, hopper(), pose).released, 0);
  arm.stow(); arm.update(1); arm.deploy(); arm.update(1);
  assert.equal(feedFromFlower(arm, w, contact, 2 * IV, hopper(), pose).released, 2);
});
test('nectar, scored and out-of-volume balls are skipped', () => {
  const w = world([ball('nec', 9, { type: 'nectar' }), ball('scored', 8, { state: 'scored' }), ball('outside', 7, { x: 50 }), ball('ok', 2)]);
  feedFromFlower(deployed(), w, contact, IV, hopper(), pose);
  assert.deepEqual(w.released, ['ok']);
});
test('no release with null contact or while stowing', () => {
  const w = world([ball('a', 9)]);
  assert.equal(feedFromFlower(deployed(), w, null, 2 * IV, hopper(), pose).released, 0);
  const arm = deployed(); arm.stow();
  assert.equal(feedFromFlower(arm, w, contact, 2 * IV, hopper(), pose).released, 0);
  assert.deepEqual(w.released, []);
});

// BUG (skipped): top-ball pick ignores arm._pending, so a free ball still in the volume is re-released on the next tick.
test('same ball is not released twice while pending', () => {
  const w = world([ball('a', 9, { state: 'free' })]), arm = deployed();
  assert.equal(feedFromFlower(arm, w, contact, IV, hopper(), pose).released, 1);
  assert.equal(feedFromFlower(arm, w, contact, IV, hopper(), pose).released, 0);
});
