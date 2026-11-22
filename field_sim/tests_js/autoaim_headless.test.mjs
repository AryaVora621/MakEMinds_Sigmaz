// Headless browser check: [follow path A, autoAim{shootAfter:true}, follow path B] run in the real page sim.
// Gated: runs only with AUTOAIM_HEADLESS=1 (needs puppeteer-core + Chrome). Otherwise every case is skipped.
// Env: AUTOAIM_HEADLESS=1, PUPPETEER_CORE (puppeteer-core ESM entry), CHROME_PATH (chrome.exe).
// Starts its own static server on the first free port in 8840..8860 (serves web/) and stops only that server.
// Prints one JSON object per scenario (console.log) so the numbers can be read verbatim.

import test from 'node:test';
import assert from 'node:assert/strict';
import http from 'node:http';
import { readFile } from 'node:fs/promises';
import path from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';

const HERE = path.dirname(fileURLToPath(import.meta.url));
const WEB = path.resolve(HERE, '..', 'web');
const RUN = process.env.AUTOAIM_HEADLESS === '1';
const SKIP = RUN ? false : 'set AUTOAIM_HEADLESS=1 to run the headless autoAim scenario';
const PUPPETEER_CORE = process.env.PUPPETEER_CORE
  || 'C:/Users/Jindal/AppData/Local/Temp/pp/node_modules/puppeteer-core/lib/esm/puppeteer/puppeteer-core.js';
const CHROME = process.env.CHROME_PATH || 'C:/Program Files/Google/Chrome/Application/chrome.exe';
const MAX_SIM_S = 15;     // hard limit on auto (runner) time
const DT = 1 / 60;
const TAIL_PATH_B = { x: 60, y: 100 };   // end of path B; the robot should be here when the auto is done

const MIME = {
  '.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8', '.mjs': 'text/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8', '.json': 'application/json; charset=utf-8', '.bin': 'application/octet-stream',
  '.png': 'image/png', '.jpg': 'image/jpeg', '.svg': 'image/svg+xml', '.glb': 'model/gltf-binary', '.wasm': 'application/wasm',
};

function startServer() {
  const srv = http.createServer(async (req, res) => {
    try {
      let p = decodeURIComponent(new URL(req.url, 'http://x').pathname);
      if (p === '/') p = '/index.html';
      const file = path.join(WEB, p);
      if (!file.startsWith(WEB)) { res.writeHead(403).end(); return; }
      const body = await readFile(file);
      res.writeHead(200, { 'Content-Type': MIME[path.extname(file)] || 'application/octet-stream', 'Cache-Control': 'no-store' }).end(body);
    } catch (_) { res.writeHead(404).end('not found'); }
  });
  return new Promise((resolve, reject) => {
    let port = 8840;
    const listen = () => {
      srv.once('error', e => { if (e.code === 'EADDRINUSE' && port < 8860) { port++; listen(); } else reject(e); });
      srv.listen(port, '127.0.0.1', () => resolve({ srv, port }));
    };
    listen();
  });
}

// ---- runs inside the page ----

// Builds [follow A, autoAim{shootAfter:true}, follow B] from two saved path-library routes and loads it into the planner.
async function setupAutoAimAuto() {
  const sim = window.__sim;
  const P = await import('/js/pathing.js');
  const lib = P.sharedPathLibrary;
  const base = JSON.parse(JSON.stringify(sim.pathEditor.route));
  const save = (name, pts) => {
    const r = JSON.parse(JSON.stringify(base));
    r.name = name; r.start = { x: pts[0].x, y: pts[0].y, h: 0 }; r.segments.length = 0;
    for (let i = 0; i < pts.length - 1; i++) {
      r.segments.push({ type: 'line', points: [pts[i], pts[i + 1]], heading: { mode: 'linear', reversed: false }, callbacks: [], label: '' });
    }
    lib.save(name, r);
  };
  save('AimPathA', [{ x: 24, y: 24 }, { x: 24, y: 100 }]);
  save('AimPathB', [{ x: 24, y: 100 }, { x: 60, y: 100 }]);
  const auto = {
    schema_version: 1, name: 'AutoAimVerify', alliance: 'blue', start: { x: 24, y: 24, h: 0 }, paths: {},
    root: { id: 'n1', type: 'sequential', children: [
      { id: 'n2', type: 'follow', path: 'AimPathA' },
      { id: 'n3', type: 'command', name: 'autoAim', args: { shootAfter: true } },
      { id: 'n4', type: 'follow', path: 'AimPathB' },
    ] },
    settings: { vmax: 60, aMax: 90, matchSeconds: 30 },
  };
  sim.autoPlanner.load(auto);
  return { root: sim.autoPlanner.auto.root.children.map(c => c.type + (c.name ? ':' + c.name : '')) };
}

// Snapshot of everything the assertions read. Plain numbers and strings only.
function snapshotState() {
  const sim = window.__sim, r = sim.autoPlanner.runner, S = sim.S;
  const stats = {};
  for (const [k, v] of Object.entries(sim.stats || {})) if (typeof v === 'number') stats[k] = v;
  return {
    state: r ? r.state : 'none', runnerTime: r ? +r.time.toFixed(3) : null,
    shots: S.shots, scored: S.scored, rpm: +S.rpm.toFixed(1), turretDeg: +S.turretDeg.toFixed(2), hoodDeg: +S.hoodDeg.toFixed(2),
    ammo: sim.S_PRACTICE.ammo, aimLabel: sim.S_AIM.label,
    pose: { x: +S.x.toFixed(2), y: +S.y.toFixed(2), h: +S.h.toFixed(3) },
    warnings: r ? r.warnings.slice() : [], errors: r ? r.errors.slice() : [], stats,
  };
}

// Steps the planner's runner through window.__sim.step until it leaves 'running' or runner time reaches maxS.
// Runs inside the page so there is one round trip per scenario, not one per frame.
async function runAutoToEnd(maxS, dt) {
  const sim = window.__sim;
  // run() is async (it awaits path sampling before start), so wait until the runner has left 'idle'.
  for (let w = 0; w < 500; w++) {
    const q = sim.autoPlanner.runner;
    if (q && q.state !== 'idle') break;
    await new Promise(res => setTimeout(res, 20));
  }
  const r = sim.autoPlanner.runner;
  if (!r || r.state === 'idle') return { ok: false, why: 'run() did not start: ' + (r ? r.state : 'no runner') };
  let guard = Math.ceil(maxS / dt) + 600, samples = [], last = -1;
  while (r.state === 'running' && r.time < maxS && guard-- > 0) {
    sim.step(dt);
    const bucket = Math.floor(r.time);
    if (bucket !== last) { last = bucket; samples.push({ t: +r.time.toFixed(2), state: r.state, shots: sim.S.shots, rpm: +sim.S.rpm.toFixed(0), turret: +sim.S.turretDeg.toFixed(1), hood: +sim.S.hoodDeg.toFixed(1), aim: sim.S_AIM.label }); }
  }
  return { ok: true, timedOut: r.state === 'running', samples };
}

async function runScenario(browser, base, { forceFailure }) {
  const page = await browser.newPage();
  const errs = [];
  page.on('pageerror', e => errs.push(String(e && e.message || e)));
  try {
    await page.setViewport({ width: 1280, height: 800 });
    await page.goto(`${base}/index.html`, { waitUntil: 'load', timeout: 60000 });
    await page.waitForFunction(() => !!window.__sim && !!window.__sim.autoPlanner, { timeout: 60000 });
    const built = await page.evaluate(setupAutoAimAuto);
    if (forceFailure) {
      // Failure variant: aim apply is a no-op and the target is an unreachable hood (feasible:false, hood 80 deg).
      await page.evaluate(() => {
        const h = window.__sim.autoHost;
        h.autoAimApply = () => false;
        h.autoAimTarget = () => ({ rpm: 3000, turretDeg: 0, hoodDeg: 80, feasible: false });
      });
    }
    const before = await page.evaluate(snapshotState);
    await page.evaluate(() => { window.__sim.autoPlanner.run(); });
    const run = await page.evaluate(runAutoToEnd, MAX_SIM_S, DT);
    const after = await page.evaluate(snapshotState);
    return { built, before, after, run, pageErrors: errs.slice(0, 5) };
  } finally {
    await page.close().catch(() => {});
  }
}

test('autoAim: [follow A, autoAim shootAfter, follow B] fires, reaches done within 15 s', { skip: SKIP, timeout: 300000 }, async () => {
  const { default: puppeteer } = await import(pathToFileURL(PUPPETEER_CORE).href);
  const { srv, port } = await startServer();
  const base = `http://127.0.0.1:${port}`;
  let browser = null;
  try {
    browser = await puppeteer.launch({
      executablePath: CHROME, headless: 'new',
      args: ['--use-gl=swiftshader', '--enable-unsafe-swiftshader', '--no-sandbox', '--window-size=1280,800'],
      defaultViewport: { width: 1280, height: 800 },
    });
    const res = await runScenario(browser, base, { forceFailure: false });
    const shotsUp = res.after.shots > res.before.shots;
    const out = {
      scenario: 'happy', built: res.built, state: res.after.state, runnerTime: res.after.runnerTime,
      before: res.before, after: res.after, scoredDelta: res.after.scored - res.before.scored,
      shotsDelta: res.after.shots - res.before.shots, samples: res.run.samples, pageErrors: res.pageErrors,
    };
    console.log(JSON.stringify(out, null, 2));
    assert.equal(res.run.ok, true, res.run.why);
    assert.equal(res.run.timedOut, false, 'auto did not reach done within 15 s');
    assert.equal(res.after.state, 'done', 'auto state after run');
    assert.ok(shotsUp, 'autoAim{shootAfter} should fire (shots must increase)');
    assert.deepEqual(res.after.errors, [], 'no runner errors');
    assert.deepEqual(res.pageErrors, [], 'no page errors');
  } finally {
    if (browser) await browser.close().catch(() => {});
    srv.close();
  }
});

test('autoAim forced failure: unreachable hood and no-op apply still complete with warnings and reach follow B', { skip: SKIP, timeout: 300000 }, async () => {
  const { default: puppeteer } = await import(pathToFileURL(PUPPETEER_CORE).href);
  const { srv, port } = await startServer();
  const base = `http://127.0.0.1:${port}`;
  let browser = null;
  try {
    browser = await puppeteer.launch({
      executablePath: CHROME, headless: 'new',
      args: ['--use-gl=swiftshader', '--enable-unsafe-swiftshader', '--no-sandbox', '--window-size=1280,800'],
      defaultViewport: { width: 1280, height: 800 },
    });
    const res = await runScenario(browser, base, { forceFailure: true });
    const distB = Math.hypot(res.after.pose.x - TAIL_PATH_B.x, res.after.pose.y - TAIL_PATH_B.y);
    const out = {
      scenario: 'forced-failure', built: res.built, state: res.after.state, runnerTime: res.after.runnerTime,
      before: res.before, after: res.after, shotsDelta: res.after.shots - res.before.shots,
      distanceToPathBEnd_in: +distB.toFixed(2), samples: res.run.samples, pageErrors: res.pageErrors,
    };
    console.log(JSON.stringify(out, null, 2));
    assert.equal(res.run.timedOut, false, 'auto did not reach done within 15 s');
    assert.equal(res.after.state, 'done', 'auto state after run');
    assert.ok(res.after.warnings.length > 0, 'forced failure should record warnings');
    assert.ok(distB < 3, `follow B should still run to (60,100); distance ${distB.toFixed(2)} in`);
    assert.deepEqual(res.pageErrors, [], 'no page errors');
  } finally {
    if (browser) await browser.close().catch(() => {});
    srv.close();
  }
});
