// h_teamcode-java_t19: no blocking sleeps or unbounded while loops in subsystem code (expected PASS).
// Scope: TeamCode/src/main/java/commands/subsystems/**/*.java and robot/Robot.java.
// commands/autonomous is exempt; its loops are printed via t.diagnostic.
// Caveat: the file-level "bounded counter" exemption is coarse; the one current hit is allow-listed.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const javaRoot = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..', '..', 'TeamCode', 'src', 'main', 'java');
const rel = (p) => path.relative(javaRoot, p).split(path.sep).join('/');
const walk = (dir) => fs.readdirSync(dir, { withFileTypes: true }).flatMap((e) => {
  const p = path.join(dir, e.name);
  return e.isDirectory() ? walk(p) : e.name.endsWith('.java') ? [p] : [];
});
// Blank out block and line comments, keeping line numbers stable.
const codeLines = (f) => fs.readFileSync(f, 'utf8').replace(/\/\*[\s\S]*?\*\//g, (m) => m.replace(/[^\n]/g, ' '))
  .split('\n').map((l) => l.replace(/\/\/.*$/, ''));

// Allow-list: ShotSolver.java:206 bracket doubling. qMin(t) = c*t^4 + 2*dv*t - 2*dd (c > 0) is
// negative at t=0 and quartic-dominated, so hi doubles to a positive value and the loop exits.
const ALLOW = [{ file: 'commands/subsystems/grouped_subsystems/shooter/ShotSolver.java',
  match: 'while (qMin(hi, dv, dd) <= 0)' }];
const SLEEP = /Thread\.sleep|\bsleep\s*\(/;
const WHILE = /\bwhile\s*\(/;
const BOUNDED = /for\s*\(\s*int\s+\w+\s*=\s*0\s*;\s*\w+\s*<\s*\d+/;
const subs = [...walk(path.join(javaRoot, 'commands', 'subsystems')), path.join(javaRoot, 'robot', 'Robot.java')];

test('subsystem and Robot.java have no Thread.sleep or sleep( calls', () => {
  const hits = subs.flatMap((f) => codeLines(f).flatMap((l, i) =>
    SLEEP.test(l) ? [`${rel(f)}:${i + 1}: ${l.trim()}`] : []));
  assert.deepEqual(hits, []);
});

test('subsystem and Robot.java while loops are bounded or allow-listed', () => {
  const bad = [];
  for (const f of subs) {
    const lines = codeLines(f);
    const bounded = lines.some((l) => BOUNDED.test(l));
    lines.forEach((l, i) => {
      const allowed = ALLOW.some((a) => a.file === rel(f) && l.includes(a.match));
      if (WHILE.test(l) && !bounded && !allowed) bad.push(`${rel(f)}:${i + 1}: ${l.trim()}`);
    });
  }
  assert.deepEqual(bad, []);
});

test('autonomous blocking loops and sleeps (exempt, reported only)', (t) => {
  let n = 0;
  for (const f of walk(path.join(javaRoot, 'commands', 'autonomous'))) {
    codeLines(f).forEach((l, i) => {
      if (WHILE.test(l) || SLEEP.test(l)) { n++; t.diagnostic(`${rel(f)}:${i + 1}: ${l.trim()}`); }
    });
  }
  t.diagnostic(`autonomous blocking-loop/sleep hits: ${n}`);
});
