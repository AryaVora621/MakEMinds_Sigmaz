// t15: FlowerArm state machine and reversals. Pure, no timers; drives update(dt) directly.
// No KNOWN-BUG cases: every behaviour below passed against current web/js/flowerarm.js.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { FlowerArm } from '../web/js/flowerarm.js';

const EPS = 1e-9;

test('deploy reaches deployed in about deployS seconds', () => {
  const arm = new FlowerArm();
  const S = arm.cfg.timing.deployS;
  arm.deploy();
  assert.equal(arm.state, 'deploying');
  arm.update(S * 0.5);
  assert.equal(arm.state, 'deploying');
  arm.update(S * 0.5 + 1e-6);
  assert.equal(arm.state, 'deployed');
  assert.equal(arm.progress, 1);
});

test('stow reaches stowed in about stowS seconds', () => {
  const arm = new FlowerArm();
  const S = arm.cfg.timing.stowS;
  arm.deploy(); arm.update(10);
  arm.stow();
  assert.equal(arm.state, 'stowing');
  arm.update(S * 0.5);
  assert.equal(arm.state, 'stowing');
  arm.update(S * 0.5 + 1e-6);
  assert.equal(arm.state, 'stowed');
  assert.equal(arm.progress, 0);
});

test('toggle alternates deploy and stow, including mid-motion', () => {
  const arm = new FlowerArm();
  arm.toggle(); assert.equal(arm.state, 'deploying');
  arm.toggle(); assert.equal(arm.state, 'stowing');
  arm.toggle(); assert.equal(arm.state, 'deploying');
  arm.update(10); assert.equal(arm.state, 'deployed');
  arm.toggle(); assert.equal(arm.state, 'stowing');
});

test('mid-motion reversal keeps progress continuous', () => {
  const arm = new FlowerArm();
  arm.deploy(); arm.update(0.2);
  const p0 = arm.progress, a0 = arm.angleDeg;
  arm.stow();
  assert.equal(arm.progress, p0, 'reversal itself must not jump progress');
  assert.equal(arm.angleDeg, a0, 'reversal itself must not jump angle');
  arm.update(0.01);
  assert.ok(Math.abs(arm.progress - (p0 - 0.01 / arm.cfg.timing.stowS)) < EPS);
  assert.ok(arm.progress < p0 && arm.progress > 0);
  // and back again
  const p1 = arm.progress;
  arm.deploy();
  assert.equal(arm.progress, p1, 'second reversal must not jump progress');
  arm.update(0.01);
  assert.ok(arm.progress > p1);
});

test('onState callbacks fire once per transition with (new, old)', () => {
  const arm = new FlowerArm();
  const seen = [];
  arm.onState((s, o) => seen.push([s, o]));
  arm.deploy();
  arm.deploy();            // no-op while deploying
  arm.update(0.1);         // no transition
  arm.update(1);           // -> deployed
  arm.update(1);           // no transition
  arm.stow();
  arm.stow();              // no-op while stowing
  arm.update(1);           // -> stowed
  assert.deepEqual(seen, [
    ['deploying', 'stowed'],
    ['deployed', 'deploying'],
    ['stowing', 'deployed'],
    ['stowed', 'stowing'],
  ]);
});

test('angleDeg endpoints equal configured stowed and deployed angles', () => {
  const arm = new FlowerArm();
  const { stowedDeg, deployedDeg } = arm.cfg.angles;
  assert.equal(arm.angleDeg, stowedDeg);
  arm.deploy(); arm.update(10);
  assert.equal(arm.angleDeg, deployedDeg);
  arm.stow(); arm.update(10);
  assert.equal(arm.angleDeg, stowedDeg);
});

test('deploy() resets fedCount and feedT', () => {
  const arm = new FlowerArm();
  arm.deploy(); arm.update(10);
  arm.fedCount = 3; arm.feedT = 0.2;
  arm.stow(); arm.update(10);
  arm.deploy();
  assert.equal(arm.fedCount, 0);
  assert.equal(arm.feedT, 0);
});

test('reset() returns to stowed with zeroed progress and counters', () => {
  const arm = new FlowerArm();
  arm.deploy(); arm.update(0.2); arm.fedCount = 2;
  arm.reset();
  assert.equal(arm.state, 'stowed');
  assert.equal(arm.progress, 0);
  assert.equal(arm.fedCount, 0);
  assert.equal(arm.angleDeg, arm.cfg.angles.stowedDeg);
});
