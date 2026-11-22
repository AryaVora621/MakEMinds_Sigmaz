// RESULT (node --test tests_js/h_robot-model_a15.test.mjs): tests 5, suites 0, pass 5, fail 0, cancelled 0, skipped 0, todo 0
// Covers tryLoadRobotGLB fallback (web/js/robotglb.js:304-307, loadRobotGLB 281-300): 404, fetch reject, garbage, zero-mesh GLB, draco.
import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from '../web/vendor/three.module.js';
import { tryLoadRobotGLB } from '../web/js/robotglb.js';

const SPEC = { chassis: { footprint: [[-9, -9], [9, -9], [9, 9], [-9, 9]] } };

// Minimal GLB container: 12-byte header + one JSON chunk, space-padded to 4 bytes.
function glb(json) {
  const body = new TextEncoder().encode(JSON.stringify(json));
  const padded = (body.length + 3) & ~3;
  const u8 = new Uint8Array(20 + padded);
  const dv = new DataView(u8.buffer);
  dv.setUint32(0, 0x46546C67, true); dv.setUint32(4, 2, true); dv.setUint32(8, u8.length, true);
  dv.setUint32(12, padded, true); dv.setUint32(16, 0x4E4F534A, true);
  u8.fill(0x20, 20);
  u8.set(body, 20);
  return u8.buffer;
}

// Fallback contract: resolves (never throws), model is null, reason is a non-empty string explaining why.
function assertFallback(res, reasonRe) {
  assert.equal(res.model, null);
  assert.equal(typeof res.reason, 'string');
  assert.ok(res.reason.length > 0, 'reason must not be empty');
  assert.match(res.reason, reasonRe);
}

test('(1) fetch returning 404 falls back with an HTTP reason', async () => {
  const res = await tryLoadRobotGLB(THREE, 'data/robot.glb', SPEC, {
    fetchImpl: async () => ({ ok: false, status: 404 }),
  });
  assertFallback(res, /HTTP 404/);
});

test('(2) fetch rejecting falls back with a could-not-fetch reason', async () => {
  const res = await tryLoadRobotGLB(THREE, 'data/robot.glb', SPEC, {
    fetchImpl: async () => { throw new TypeError('network down'); },
  });
  assertFallback(res, /could not fetch .*network down/);
});

test('(3) garbage bytes fall back with a parse reason', async () => {
  const garbage = new Uint8Array(64).map((_, i) => (i * 37 + 11) & 0xff);
  const res = await tryLoadRobotGLB(THREE, garbage.buffer, SPEC);
  assertFallback(res, /not a glTF\/GLB|GLB:/);
});

test('(4) valid GLB header with zero meshes falls back as an empty scene', async () => {
  const res = await tryLoadRobotGLB(THREE, glb({ asset: { version: '2.0' }, meshes: [], nodes: [] }), SPEC);
  assertFallback(res, /scene is empty/);
});

test('(5) draco extensionsRequired falls back with a draco reason', async () => {
  const json = { asset: { version: '2.0' }, extensionsRequired: ['KHR_draco_mesh_compression'], meshes: [], nodes: [] };
  const res = await tryLoadRobotGLB(THREE, glb(json), SPEC);
  assertFallback(res, /draco/);
});
