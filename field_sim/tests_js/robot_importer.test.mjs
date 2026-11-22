// End to end: synthetic STL parts (mm, Z-up) -> tools/import_robot_cad.py -> GLB -> loadRobotGLB. Skipped if the venv python is missing.
import test from 'node:test';
import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import { existsSync, mkdtempSync, writeFileSync, readFileSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import * as THREE from '../web/vendor/three.module.js';
import { loadRobotGLB, animateCADParts } from '../web/js/robotglb.js';

const ROOT = new URL('..', import.meta.url).pathname.replace(/^\/([A-Za-z]:)/, '$1');
const PY = join(ROOT, '.venv', 'Scripts', 'python.exe');
const TOOL = join(ROOT, 'tools', 'import_robot_cad.py');
const SPEC = JSON.parse(readFileSync(join(ROOT, 'web', 'data', 'robot_spec.json'), 'utf8'));

function asciiBox(name, sx, sy, sz, cx, cy, cz) {
  const v = []; for (let i = 0; i < 8; i++) v.push([cx + (i & 1 ? 1 : -1) * sx / 2, cy + (i & 2 ? 1 : -1) * sy / 2, cz + (i & 4 ? 1 : -1) * sz / 2]);
  const I = [0, 1, 3, 0, 3, 2, 4, 7, 5, 4, 6, 7, 0, 5, 1, 0, 4, 5, 2, 3, 7, 2, 7, 6, 0, 2, 6, 0, 6, 4, 1, 5, 7, 1, 7, 3];
  let s = `solid ${name}\n`;
  for (let t = 0; t < I.length; t += 3) s += ` facet normal 0 0 0\n  outer loop\n` + [0, 1, 2].map(k => `   vertex ${v[I[t + k]].join(' ')}\n`).join('') + `  endloop\n endfacet\n`;
  return s + `endsolid ${name}\n`;
}

test('STL parts (mm, Z-up) convert to a GLB the loader fits to the 381 x 375 mm frame', { skip: !existsSync(PY) }, async () => {
  const dir = mkdtempSync(join(tmpdir(), 'robotcad-'));
  const zipPy = join(dir, 'mk.py');
  writeFileSync(join(dir, 'chassis.stl'), asciiBox('chassis', 381, 375, 200, 0, 0, 100));
  writeFileSync(join(dir, 'turret_assembly.stl'), asciiBox('turret', 100, 100, 50, 0, 0, 225));
  writeFileSync(zipPy, `import zipfile,sys,os\nd=sys.argv[1]\nz=zipfile.ZipFile(os.path.join(d,'parts.zip'),'w')\nfor n in ('chassis.stl','turret_assembly.stl'): z.write(os.path.join(d,n),n)\nz.close()\n`);
  assert.equal(spawnSync(PY, ['-I', zipPy, dir]).status, 0);
  const out = join(dir, 'robot.glb');
  const r = spawnSync(PY, ['-I', TOOL, join(dir, 'parts.zip'), '-o', out, '--units', 'mm', '--pivot-from-name'], { encoding: 'utf8' });
  assert.equal(r.status, 0, r.stderr + r.stdout);
  const m = await loadRobotGLB(THREE, new Uint8Array(readFileSync(out)), SPEC, { orientation: { upAxis: 'Z', forwardAxis: '+X' } });
  assert.ok(m, loadRobotGLB.lastReason);
  assert.equal(m.units, 'm');
  const b = new THREE.Box3().setFromObject(m.group);
  assert.ok(Math.abs(b.max.x - b.min.x - 15) < 0.01 && Math.abs(b.max.y - b.min.y - 14.76) < 0.01, 'fits 381 x 375 mm');
  assert.ok(Math.abs(b.min.z) < 1e-6, 'floor at z=0');
  assert.ok(m.parts.turret, 'turret node found');
  assert.equal(animateCADParts(THREE, m.parts, { turretDeg: 45 }), 1);
  // pivot re-based on the part's bbox centre: yawing the turret must not move its centre in xy
  const c0 = new THREE.Box3().setFromObject(m.parts.turret).getCenter(new THREE.Vector3());
  animateCADParts(THREE, m.parts, { turretDeg: 0 });
  const c1 = new THREE.Box3().setFromObject(m.parts.turret).getCenter(new THREE.Vector3());
  assert.ok(Math.hypot(c0.x - c1.x, c0.y - c1.y) < 1e-3);
  // re-importing the produced GLB is a clean pass-through
  const out2 = join(dir, 'robot2.glb');
  assert.equal(spawnSync(PY, ['-I', TOOL, out, '-o', out2], { encoding: 'utf8' }).status, 0);
});

test('importer rejects missing input and compressed glTF with a non-zero exit', { skip: !existsSync(PY) }, () => {
  assert.notEqual(spawnSync(PY, ['-I', TOOL, join(tmpdir(), 'nope.stl')]).status, 0);
  const dir = mkdtempSync(join(tmpdir(), 'robotcad-'));
  const f = join(dir, 'c.gltf');
  writeFileSync(f, JSON.stringify({ asset: { version: '2.0' }, extensionsRequired: ['KHR_draco_mesh_compression'] }));
  const r = spawnSync(PY, ['-I', TOOL, f, '-o', join(dir, 'o.glb')], { encoding: 'utf8' });
  assert.equal(r.status, 1); assert.match(r.stderr, /compress/i);
});
