// JS <-> Java parity. (1) PARAMS-SYNC comments in PedroFollowSim.java equal resolveParams(). (2) Optional end-to-end comparison
// that spawns java when tools_java/out/follow and a java binary exist (otherwise skipped).
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync, existsSync, readdirSync } from 'node:fs';
import { spawnSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import path from 'node:path';
import os from 'node:os';
import { resolveParams, toJavaRequest } from '../../web/js/physics/pedro-fidelity/params.js';
import { simulateBridgeRequest } from '../../web/js/physics/pedro-fidelity/closedloop.js';

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..', '..');
const JAVA_SRC = path.join(ROOT, 'tools_java', 'PedroFollowSim.java');
const FOLLOW_OUT = path.join(ROOT, 'tools_java', 'out', 'follow');
const P = resolveParams();

// PARAMS-SYNC key -> value in the resolved JS params (units in the Java comments)
const JS = {
  'loop.dt': P.loop.dt, dt: P.loop.dt,
  tauX: P.plant.tauX, tauY: P.plant.tauY, tauW: P.plant.tauW,
  trackWidthIn: P.plant.trackWidthIn, wheelBaseIn: P.plant.wheelBaseIn, muTraction: P.plant.muTraction,
  plantNominalV: P.plant.plantNominalV, batteryVoc: P.plant.batteryVoc, physicsDt: P.plant.physicsDt,
  xVelocity: P.mecanum.xVelocity, yVelocity: P.mecanum.yVelocity,
  tValue: P.constraints.tValueConstraint, timeoutMs: P.constraints.timeoutConstraint,
  braking: P.constraints.brakingStrength, brakingStart: P.constraints.brakingStart,
};

function syncPairs() {
  const src = readFileSync(JAVA_SRC, 'utf8'), pairs = [];
  for (const line of src.split(/\r?\n/)) {
    const i = line.indexOf('PARAMS-SYNC');
    if (i < 0) continue;
    for (const m of line.slice(i + 'PARAMS-SYNC'.length).matchAll(/([A-Za-z_][\w.]*)=(-?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)/g)) pairs.push([m[1], +m[2]]);
  }
  return pairs;
}

test('PARAMS-SYNC comments exist for the key plant/constraint defaults', () => {
  const keys = new Set(syncPairs().map(([k]) => k));
  for (const k of ['tauX', 'tauY', 'tauW', 'trackWidthIn', 'wheelBaseIn', 'muTraction', 'plantNominalV', 'batteryVoc', 'xVelocity', 'yVelocity', 'tValue', 'timeoutMs']) {
    assert.ok(keys.has(k), `PedroFollowSim.java is missing a PARAMS-SYNC comment for ${k}`);
  }
});

test('every PARAMS-SYNC default in PedroFollowSim.java equals resolveParams()', () => {
  for (const [k, v] of syncPairs()) {
    assert.ok(k in JS, `PARAMS-SYNC key "${k}" has no mapping in this test (add it to the JS table)`);
    assert.ok(Math.abs(JS[k] - v) <= 1e-12 * Math.max(1, Math.abs(v)), `${k}: Java ${v} vs JS ${JS[k]}`);
  }
});

test('toJavaRequest emits every robot key the Java side reads', () => {
  const req = toJavaRequest(P), java = readFileSync(JAVA_SRC, 'utf8');
  for (const k of Object.keys(req.robot)) assert.ok(java.includes(`"${k}"`), `Java never reads robot.${k}`);
  for (const k of Object.keys(req.pathConstraints)) assert.ok(java.includes(`"${k}"`), `Java never reads pathConstraints.${k}`);
});

// ---------------- optional end-to-end comparison ----------------
function findCoreJar() {
  const base = path.join(os.homedir(), '.gradle', 'caches', 'modules-2', 'files-2.1', 'com.pedropathing', 'core', '2.1.0');
  if (!existsSync(base)) return null;
  for (const d of readdirSync(base)) { const f = path.join(base, d, 'core-2.1.0.jar'); if (existsSync(f)) return f; }
  return null;
}
function javaAvailable() { const r = spawnSync('java', ['-version'], { encoding: 'utf8' }); return !r.error && r.status === 0; }
function runJava(req) {
  const jar = findCoreJar();
  const r = spawnSync('java', ['-cp', [FOLLOW_OUT, jar].join(path.delimiter), 'PedroFollowSim'], { input: JSON.stringify(req), encoding: 'utf8', timeout: 60000, maxBuffer: 64 * 1024 * 1024 });
  if (r.error) throw r.error;
  return JSON.parse(r.stdout);
}
const skipReason = () => !existsSync(path.join(FOLLOW_OUT, 'PedroFollowSim.class')) ? 'tools_java/out/follow not built'
  : !findCoreJar() ? 'Pedro core 2.1.0 jar not in the gradle cache' : !javaAvailable() ? 'java not on PATH' : null;

const body = () => ({ start: { x: 12, y: 12, h: 0 }, segments: [
  { type: 'line', points: [{ x: 12, y: 12 }, { x: 60, y: 12 }], heading: { mode: 'constant', start: 0 } },
  { type: 'line', points: [{ x: 60, y: 12 }, { x: 60, y: 48 }], heading: { mode: 'constant', start: 0 } },
] });

test('Java and JS closed loops agree on the same bridge body (end pose < 1 in, duration within 10%)', (t) => {
  const why = skipReason(); if (why) return t.skip(why);
  const b = body();
  const js = simulateBridgeRequest(b);
  const jv = runJava({ ...toJavaRequest(P), ...b });
  assert.ok(!jv.error, jv.error);
  const jl = jv.ticks.at(-1), sl = js.at(-1);
  assert.ok(Math.hypot(jl.x - sl.x, jl.y - sl.y) < 1, `end pose java (${jl.x},${jl.y}) js (${sl.x},${sl.y})`);
  assert.ok(Math.abs(jv.duration - js.duration) / js.duration < 0.10, `duration java ${jv.duration} js ${js.duration}`);
  assert.ok('stuck' in jl && 'mode' in jl, 'Java ticks carry stuck and mode');
});

test('Java library defaults (FollowerConstants.defaults()) equal the JS PEDRO constants', (t) => {
  const why = skipReason(); if (why) return t.skip(why);
  const jv = runJava(body());   // no constants block: Java uses the jar defaults
  assert.ok(!jv.error, jv.error);
  const c = jv.info.constants;
  for (const k of ['mass', 'forwardZeroPowerAcceleration', 'lateralZeroPowerAcceleration', 'centripetalScaling']) {
    assert.ok(Math.abs(c[k] - P.pedro[k]) <= 1e-6 * Math.max(1, Math.abs(c[k])), `${k}: Pedro 2.1.0 jar ${c[k]} vs params.js ${P.pedro[k]}`);
  }
});

// Every field of a fresh `new FollowerConstants()` / `new PathConstraints(0.99,100,1,1)`, read by reflection from the
// jar on the classpath (info.libraryDefaults), must equal params.js exactly. The FollowerConstants ctor calls defaults(),
// which overwrites the field initializers in the sources jar, so reading the source is not enough.
function compareDeep(jar, js, where, out) {
  for (const k of Object.keys(jar)) {
    const a = jar[k], b = js?.[k];
    if (a !== null && typeof a === 'object') { compareDeep(a, b, `${where}.${k}`, out); continue; }
    if (b === undefined) { out.push(`${where}.${k}: missing in params.js (jar ${a})`); continue; }
    if (typeof a === 'boolean' ? a !== b : Math.abs(a - b) > 1e-12 * Math.max(1, Math.abs(a))) out.push(`${where}.${k}: jar ${a} vs params.js ${b}`);
  }
}
test('every FollowerConstants / PathConstraints field of the real jar equals params.js', (t) => {
  const why = skipReason(); if (why) return t.skip(why);
  const jv = runJava(body());
  assert.ok(!jv.error, jv.error);
  const lib = jv.info.libraryDefaults;
  assert.ok(lib && lib.followerConstants && lib.pathConstraints, 'PedroFollowSim info.libraryDefaults missing (rebuild tools_java/out/follow)');
  const bad = [];
  compareDeep(lib.followerConstants, P.pedro, 'pedro', bad);
  compareDeep(lib.pathConstraints, P.constraints, 'constraints', bad);
  assert.deepEqual(bad, []);
});
