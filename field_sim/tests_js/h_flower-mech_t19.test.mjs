// t19: 3D flower arm (flowerarm3d.js buildFlowerArm) vs logic (flowerarm.js tipRobot). Tip = rubber bottom edge in parent frame.
// Known mismatches (failing-by-design): none when written; a future failure means the 3D paddle and logic tip drifted.
import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from '../web/vendor/three.module.js';
import { buildFlowerArm, wristDegFor } from '../web/js/flowerarm3d.js';
import { FlowerArm } from '../web/js/flowerarm.js';

const ANGLES = [90, 45, 0, -35];
const SIDES = ['front', 'rear'];
const TOL_IN = 0.3;
const RAD = Math.PI / 180;

// Build the rig under a parent group that stands in for the robot frame; return the rubber bottom edge centre in that frame.
function rig(side) {
  const arm = new FlowerArm({ side });
  const parent = new THREE.Group();
  const r = buildFlowerArm(THREE, arm.cfg);
  parent.add(r.group);
  return { arm, parent, r };
}
const tipInParent = (r, parent) => {
  parent.updateMatrixWorld(true);
  const rubber = r.parts.rubber;
  const d = rubber.geometry.parameters.depth; // rubberThickness, box is centred on the bottom edge
  return new THREE.Vector3(0, 0, -d / 2).applyMatrix4(rubber.matrixWorld);
};

for (const side of SIDES) {
  for (const a of ANGLES) {
    test(`${side} ${a} deg: tip matches tipRobot within 0.3 in and paddle stays vertical`, () => {
      const { arm, parent, r } = rig(side);
      r.setAngle(a); arm.angleDeg = a;
      const p = tipInParent(r, parent), want = arm.tipRobot();
      assert.ok(Math.abs(p.x - want[0]) <= TOL_IN, `x ${p.x} vs ${want[0]}`);
      assert.ok(Math.abs(p.y - want[1]) <= TOL_IN, `y ${p.y} vs ${want[1]}`);
      assert.ok(Math.abs(p.z - want[2]) <= TOL_IN, `z ${p.z} vs ${want[2]}`);
      const d = new THREE.Vector3(0, 0, -1).transformDirection(r.parts.paddle.matrixWorld);
      assert.ok(Math.abs(d.x) < 1e-6 && Math.abs(d.y) < 1e-6 && Math.abs(d.z + 1) < 1e-6, `tilted: ${d.toArray()}`);
    });
  }
}

test('wristDegFor(cfg, a) == -a for all test angles', () => {
  const cfg = new FlowerArm({}).cfg;
  for (const a of ANGLES) assert.equal(wristDegFor(cfg, a), -a);
});

test('setFraction: 0 maps to stowedDeg, 1 maps to deployedDeg (hinge.rotation.y = -rad)', () => {
  for (const side of SIDES) {
    const { arm, r } = rig(side);
    const { stowedDeg, deployedDeg } = arm.cfg.angles;
    r.setFraction(0);
    assert.ok(Math.abs(r.hinge.rotation.y - (-stowedDeg * RAD)) < 1e-9, `stowed ${side}`);
    r.setFraction(1);
    assert.ok(Math.abs(r.hinge.rotation.y - (-deployedDeg * RAD)) < 1e-9, `deployed ${side}`);
  }
});
