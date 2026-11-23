// Golden-file record / compare / load / save for NON-chaotic scenes. Check is numpy assert_allclose via metrics.allclose.
// Tolerances come from params.js GOLDEN (ASSUMED). Units: inches, in/s, rad, s.
import { GOLDEN } from './params.js';
import { allclose } from './metrics.js';
import { makeScenario, runScenario } from './scenarios.js';

const VERSION = 1;

/** 'pos' (x y z t rx ry ...), 'vel' (vx vy vz bv*), 'ang' (h bw). */
function fieldClass(f) {
  if (f === 'h' || f === 'bw') return 'ang';
  if (f === 'vx' || f === 'vy' || f === 'vz' || f.startsWith('bv')) return 'vel';
  return 'pos';
}
const TOL_KEYS = { pos: ['atolPos', 'rtolPos'], vel: ['atolVel', 'rtolVel'], ang: ['atolAng', 'rtolAng'] };

function tolFor(fields) {
  const tol = {};
  for (const f of fields) for (const k of TOL_KEYS[fieldClass(f)]) tol[k] = GOLDEN[k];
  return tol;
}

/** Run scene `name` against the current sim and return a golden file object. */
export function recordGolden(name, opts = {}) {
  const sc = makeScenario(name, { env: opts.env });
  if (sc.chaotic) throw new Error('golden files exist only for non-chaotic scenes: ' + name);
  const run = runScenario(sc);
  return { name, version: VERSION, dt: sc.dt, steps: sc.steps, fields: run.fields, data: run.data, tol: tolFor(run.fields) };
}

/** Compare a run {fields,data} to a golden. tol defaults to golden.tol. Returns {pass, worst, rmseByField}. */
export function compareGolden(golden, run, tol) {
  const T = tol || golden.tol || tolFor(golden.fields);
  if (run.data.length !== golden.data.length) throw new Error(`step count differs: ${run.data.length} vs ${golden.data.length}`);
  let pass = true, worst = null, worstRatio = -Infinity;
  const rmseByField = {};
  golden.fields.forEach((f, c) => {
    const [ak, rk] = TOL_KEYS[fieldClass(f)];
    const exp = golden.data.map(r => r[c]);
    const act = run.data.map(r => r[run.fields.indexOf(f)]);
    const r = allclose(act, exp, { atol: T[ak], rtol: T[rk] });
    let ss = 0;
    for (let i = 0; i < exp.length; i++) ss += (act[i] - exp[i]) ** 2;
    rmseByField[f] = Math.sqrt(ss / exp.length);
    if (!r.pass) pass = false;
    const ratio = r.allowedAtWorst > 0 ? r.maxErr / r.allowedAtWorst : (r.maxErr > 0 ? Infinity : 0);
    if (ratio > worstRatio) {
      worstRatio = ratio;
      const i = r.worstIndex;
      worst = { field: f, step: i, actual: act[i], expected: exp[i], err: r.maxErr, allowed: r.allowedAtWorst };
    }
  });
  return { pass, worst, rmseByField };
}

export function goldenUrl(name) { return new URL('./golden/' + name + '.json', import.meta.url); }

const isNode = () => typeof process !== 'undefined' && !!(process.versions && process.versions.node);

export async function loadGolden(name) {
  const url = goldenUrl(name);
  if (isNode() && url.protocol === 'file:') {
    const { readFile } = await import('node:fs/promises');
    return JSON.parse(await readFile(url, 'utf8'));
  }
  const res = await fetch(url);
  if (!res.ok) throw new Error('golden fetch failed: ' + url + ' ' + res.status);
  return res.json();
}

/** Node only. Writes the golden JSON (one step row per line for readable diffs). */
export async function saveGolden(name, g) {
  const { writeFile, mkdir } = await import('node:fs/promises');
  const { fileURLToPath } = await import('node:url');
  const url = goldenUrl(name);
  await mkdir(fileURLToPath(new URL('./', url)), { recursive: true });
  const head = JSON.stringify({ name: g.name, version: g.version, dt: g.dt, steps: g.steps, fields: g.fields, tol: g.tol });
  const rows = g.data.map(r => JSON.stringify(r)).join(',\n');
  const text = head.slice(0, -1) + ',"data":[\n' + rows + '\n]}\n';
  await writeFile(url, text, 'utf8');
}
