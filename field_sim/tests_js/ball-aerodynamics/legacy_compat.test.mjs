// legacy_compat.test.mjs - airStep in legacy mode must equal balls.js _substep lines 526-532 bit for bit.
import test from 'node:test';
import assert from 'node:assert/strict';
import { AERO } from '../../web/js/physics/ball-aerodynamics/params.js';
import { buildAeroModel } from '../../web/js/physics/ball-aerodynamics/air.js';
import { airStep } from '../../web/js/physics/ball-aerodynamics/ballstep.js';

function mulberry32(a) {
  return function () {
    a |= 0; a = (a + 0x6D2B79F5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

const G = AERO.g;               // in/s^2
const dragK = AERO.legacyDragK; // 1/in
const MAXSPEED = AERO.maxSpeed; // in/s

// Reference: copy of balls.js lines 526-532 (airborne test, drag, integrate, clamp).
function reference(b, h) {
  if (b.z > b.r + 0.05) {
    const sp = Math.sqrt(b.vx * b.vx + b.vy * b.vy + b.vz * b.vz), f = 1 / (1 + dragK * sp * h);
    b.vx *= f; b.vy *= f; b.vz *= f;
  }
  b.x += b.vx * h; b.y += b.vy * h; b.z += b.vz * h - 0.5 * G * h * h; b.vz -= G * h;
  const sp2 = b.vx * b.vx + b.vy * b.vy + b.vz * b.vz;
  if (sp2 > MAXSPEED * MAXSPEED) { const k = MAXSPEED / Math.sqrt(sp2); b.vx *= k; b.vy *= k; b.vz *= k; }
  return b;
}

const legacyModel = buildAeroModel(AERO, AERO.kinds.pollen, { model: 'legacy' });
const FIELDS = ['x', 'y', 'z', 'vx', 'vy', 'vz'];

test('1000 random states: airStep == inline legacy code (strict ===)', () => {
  const rnd = mulberry32(12345);
  let air = 0, grounded = 0;
  for (let i = 0; i < 1000; i++) {
    const r = 1.391; // in
    const isGrounded = rnd() < 0.4;
    const b = {
      x: rnd() * 144 - 72, y: rnd() * 144 - 72,
      z: isGrounded ? r + rnd() * 0.05 : r + 0.06 + rnd() * 60,
      vx: (rnd() - 0.5) * 1600, vy: (rnd() - 0.5) * 1600, vz: (rnd() - 0.5) * 1600, // some exceed maxSpeed
      r, type: 'pollen',
    };
    const h = [0.001, 0.002, 0.004][i % 3]; // s
    const a = { ...b }, c = { ...b };
    airStep(a, h, legacyModel);
    reference(c, h);
    for (const f of FIELDS) assert.ok(a[f] === c[f], `state ${i} field ${f}: ${a[f]} vs ${c[f]}`);
    if (isGrounded) grounded++; else air++;
  }
  assert.ok(air > 100 && grounded > 100);
});

test('grounded balls get no drag', () => {
  const b = { x: 0, y: 0, z: 1.391, vx: 300, vy: -200, vz: 0, r: 1.391, type: 'pollen' };
  airStep(b, 0.002, legacyModel);
  assert.equal(b.vx, 300);
  assert.equal(b.vy, -200);
});
