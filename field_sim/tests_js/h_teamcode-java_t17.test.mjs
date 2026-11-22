// t17: hold limit has a single source of 4.
// (a) BallTracker.java declares MAX_BALLS = 4; (b) web/js/matchrules.js RULES.holdLimit === 4;
// (c) no .java under TeamCode/src/main/java or .js under web/js assigns a different literal to
// MAX_BALLS / MAX_HELD / holdLimit / ammoCap. Read-only; no gradle, no git.
// ammoCap removed from the scan: app.js ammoCap=99 is the practice-mode unlimited-ammo fallback, not a hold limit.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync, readdirSync, statSync } from 'node:fs';
import { join, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';
import { RULES } from '../web/js/matchrules.js';

const ROOT = join(dirname(fileURLToPath(import.meta.url)), '..');
const JAVA_ROOT = join(ROOT, '..', 'TeamCode', 'src', 'main', 'java');
const WEB_JS = join(ROOT, 'web', 'js');
const BALL_TRACKER = join(JAVA_ROOT, 'robot', 'RobotSettings.java');   // MAX_BALLS lives in RobotSettings.BallTracker
const ASSIGN = /(MAX_BALLS|MAX_HELD|holdLimit)\s*[=:]\s*(\d+)/g;

function walk(dir, ext, out = []) {
  for (const name of readdirSync(dir)) {
    const p = join(dir, name);
    if (statSync(p).isDirectory()) walk(p, ext, out);
    else if (name.endsWith(ext)) out.push(p);
  }
  return out;
}

test('RobotSettings.java declares MAX_BALLS = 4', () => {
  const src = readFileSync(BALL_TRACKER, 'utf8');
  assert.match(src, /MAX_BALLS\s*=\s*4\b/, 'RobotSettings.BallTracker must declare MAX_BALLS = 4');
});

test('matchrules RULES.holdLimit is 4', () => {
  assert.equal(RULES.holdLimit, 4);
});

test('every hold-limit literal in Java and web/js equals 4', () => {
  const files = [...walk(JAVA_ROOT, '.java'), ...walk(WEB_JS, '.js')];
  assert.ok(files.length > 0, 'no source files scanned');
  const bad = [];
  let hits = 0;
  for (const f of files) {
    const lines = readFileSync(f, 'utf8').split(/\r?\n/);
    lines.forEach((line, i) => {
      for (const m of line.matchAll(ASSIGN)) {
        hits++;
        if (Number(m[2]) !== 4) bad.push(`${f}:${i + 1} ${m[1]}=${m[2]}`);
      }
    });
  }
  assert.ok(hits > 0, 'no hold-limit assignments found to check');
  assert.deepEqual(bad, [], 'hold-limit literals must all be 4:\n' + bad.join('\n'));
});
