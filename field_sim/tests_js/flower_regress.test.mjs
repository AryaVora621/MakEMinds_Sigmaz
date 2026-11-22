import { test } from 'node:test';
import assert from 'node:assert/strict';
import { FlowerArm } from '../web/js/flowerarm.js';
test('negative dt does not move the arm', () => {
  const a = new FlowerArm(); a.deploy(); a.update(0.2); const p = a.progress; a.update(-1); assert.equal(a.progress, p);
});
test('nested cfg objects are not shared between arms', () => {
  const a = new FlowerArm(); a.cfg.feed.intakeMouth.x = 99; assert.notEqual(new FlowerArm().cfg.feed.intakeMouth.x, 99);
});
