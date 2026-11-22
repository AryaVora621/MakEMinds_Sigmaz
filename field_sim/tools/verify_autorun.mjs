// Headless repro: "Run in sim" drives the robot in the 3D view, then Stop and Restart.
// Usage: node tools/verify_autorun.mjs
// Env (optional): PUPPETEER_CORE (puppeteer-core ESM entry), CHROME_PATH (chrome.exe), SHOT_DIR (screenshot folder).
// Starts its own static server on the first free port in 8841..8860 (serves web/) and stops only that server.
// Exit code 0 when every assertion passes, 1 otherwise. Prints one JSON object.

import http from 'node:http';
import { readFile } from 'node:fs/promises';
import path from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';

const HERE = path.dirname(fileURLToPath(import.meta.url));
const WEB = path.resolve(HERE, '..', 'web');
const PUPPETEER_CORE = process.env.PUPPETEER_CORE
  || 'C:/Users/Jindal/AppData/Local/Temp/pp/node_modules/puppeteer-core/lib/esm/puppeteer/puppeteer-core.js';
const CHROME = process.env.CHROME_PATH || 'C:/Program Files/Google/Chrome/Application/chrome.exe';
const SHOT_DIR = process.env.SHOT_DIR || 'C:/Users/Jindal/AppData/Local/Temp/claude/1bf884c3629a4194/cab1052d-b484-449d-8151-8eddbe3da6ad/scratchpad';
const SHOT_PATH = path.join(SHOT_DIR, 'verify_autorun_3d.png');
const START = { x: 24, y: 24, h: 0 };          // auto start pose (the path starts here)
const MOVE_MIN_IN = 10;                         // "moved" = first and last pose more than 10 in apart
const RUN_MS = 8000, RESTART_MS = 15000;
const sleep = ms => new Promise(r => setTimeout(r, ms));
const dist = (a, b) => Math.hypot(a.x - b.x, a.y - b.y);

const MIME = {
  '.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8', '.mjs': 'text/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8', '.json': 'application/json; charset=utf-8', '.bin': 'application/octet-stream',
  '.png': 'image/png', '.jpg': 'image/jpeg', '.svg': 'image/svg+xml', '.glb': 'model/gltf-binary', '.wasm': 'application/wasm',
};

// Static file server for web/ (path-traversal guarded). Unknown paths get 404, the page falls back where it can.
function makeHandler() {
  return async (req, res) => {
    try {
      const url = new URL(req.url, 'http://127.0.0.1');
      let rel = decodeURIComponent(url.pathname);
      if (rel === '/' || rel === '') rel = '/index.html';
      const file = path.resolve(WEB, '.' + rel);
      if (file !== WEB && !file.startsWith(WEB + path.sep)) { res.writeHead(403); res.end(); return; }
      const body = await readFile(file);
      res.writeHead(200, { 'Content-Type': MIME[path.extname(file).toLowerCase()] || 'application/octet-stream', 'Cache-Control': 'no-store' });
      res.end(body);
    } catch (e) {
      res.writeHead(404, { 'Content-Type': 'text/plain' }); res.end('not found');
    }
  };
}

// First free port from 8841 upward. Never touches a server it did not start.
async function startServer() {
  for (let port = 8841; port <= 8860; port++) {
    const srv = http.createServer(makeHandler());
    const ok = await new Promise(resolve => {
      srv.once('error', () => resolve(false));
      srv.listen(port, '127.0.0.1', () => resolve(true));
    });
    if (ok) return { srv, port };
  }
  throw new Error('no free port in 8841..8860');
}

// Runs inside the page. Builds the auto: one path (start -> (24,100) -> (60,100)), a follow of it, then a shoot.
// The path goes into the shared path library (autoui hostPaths reads pathLib.all()), so the follow node resolves.
async function buildAuto(start) {
  const sim = window.__sim;
  const P = await import('/js/pathing.js');
  const lib = P.sharedPathLibrary;
  const name = 'VerifyPath';
  const route = sim.pathEditor.route;
  route.name = name;
  route.start = { x: start.x, y: start.y, h: start.h };
  const p0 = { x: start.x, y: start.y }, p1 = { x: start.x, y: 100 }, p2 = { x: 60, y: 100 };
  const seg = (a, b) => ({ type: 'line', points: [{ x: a.x, y: a.y }, { x: b.x, y: b.y }], heading: { mode: 'linear', reversed: false }, callbacks: [], label: '' });
  route.segments.length = 0;
  route.segments.push(seg(p0, p1), seg(p1, p2));
  lib.save(name, route);
  lib.setActive(name);
  const auto = {
    schema_version: 1, name: 'VerifyAutorun', alliance: 'blue', start: { x: start.x, y: start.y, h: start.h }, paths: {},
    root: { id: 'n1', type: 'sequential', children: [
      { id: 'n2', type: 'follow', path: name },
      { id: 'n3', type: 'command', name: 'shoot', args: { count: 1 } },
    ] },
    settings: { vmax: 60, aMax: 90, matchSeconds: 30 },
  };
  sim.autoPlanner.load(auto);
  return { name: auto.name, path: name, segments: route.segments.length, start: route.start, root: sim.autoPlanner.auto.root.children.map(c => c.type) };
}

// Runs inside the page. A requestAnimationFrame logger records pose/view/state about every 250 ms, so the log does not
// depend on round trips from Node (the 3D render loop makes each page.evaluate slow under SwiftShader).
function startLogger() {
  window.__arStop = false;
  window.__arLog = [];
  window.__arFrames = 0;
  let last = -1e9;
  const t0 = performance.now();
  const frame = () => {
    if (window.__arStop) return;
    window.__arFrames++;
    const now = performance.now();
    if (now - last >= 250) {
      last = now;
      const sim = window.__sim, r = sim.autoPlanner.runner;
      window.__arLog.push({ t: Math.round(now - t0), x: sim.S.x, y: sim.S.y, h: sim.S.h, view: sim.S.view,
        state: r ? r.state : 'none', time: r ? r.time : 0, shots: sim.S.shots, scored: sim.S.scored, errors: r ? r.errors.slice(-2) : [] });
    }
    requestAnimationFrame(frame);
  };
  requestAnimationFrame(frame);
}
function takeLog() { return window.__arLog ? window.__arLog.splice(0) : []; }
function stopLogger() { window.__arStop = true; }

function collapseStates(samples) {
  const seq = [];
  for (const s of samples) if (!seq.length || seq[seq.length - 1].state !== s.state) seq.push({ state: s.state, t_ms: s.t });
  return seq;
}

async function main() {
  const out = {
    url: null, moved: false, view: null, states: [], shots: null, samples: 0,
    assertions: {}, stop: null, restart: null, screenshot: SHOT_PATH,
    console_errors: [], page_errors: [], http_errors: [], toasts: [], build: null, error: null, exit: 0,
  };
  const { default: puppeteer } = await import(pathToFileURL(PUPPETEER_CORE).href);
  const { srv, port } = await startServer();
  out.url = `http://127.0.0.1:${port}/index.html`;
  let browser = null;
  try {
    browser = await puppeteer.launch({
      executablePath: CHROME, headless: 'new',
      args: ['--use-gl=swiftshader', '--enable-unsafe-swiftshader', '--no-sandbox', '--window-size=1280,800'],
      defaultViewport: { width: 1280, height: 800 },
    });
    const page = await browser.newPage();
    page.on('console', msg => { if (msg.type() === 'error') out.console_errors.push(msg.text()); });
    page.on('pageerror', e => out.page_errors.push(String(e && e.message || e)));
    page.on('response', r => { if (r.status() >= 400) out.http_errors.push(r.status() + ' ' + r.url().replace(out.url.replace(/\/index\.html$/, ''), '')); });
    await page.goto(out.url, { waitUntil: 'load', timeout: 60000 });
    await page.waitForFunction(() => !!window.__sim, { timeout: 60000 });

    // Run in sim lives in the Auto planner tab (#autopanel), so open that tab first with a real click.
    await page.click('#tab-auto');
    await sleep(300);
    out.build = await page.evaluate(buildAuto, START);

    // Real mouse click on the visible "Run in sim" button. Coordinates are read right before the click.
    const runBox = await page.evaluate(() => {
      const b = [...document.querySelectorAll('button')]
        .find(x => x.offsetParent !== null && /run in sim/i.test((x.textContent || '') + ' ' + (x.title || '')));
      if (!b) return null;
      b.scrollIntoView({ block: 'center' });
      const r = b.getBoundingClientRect();
      return { x: r.x + r.width / 2, y: r.y + r.height / 2, w: r.width, h: r.height };
    });
    out.assertions.run_button_found = !!runBox && runBox.w > 0;
    if (!runBox || !(runBox.w > 0)) throw new Error('visible "Run in sim" button not found');

    await page.evaluate(startLogger);
    await page.mouse.click(runBox.x, runBox.y);
    await sleep(500);
    out.frames_after_click = await page.evaluate(() => window.__arFrames || 0);
    const runLog = await collect(page, RUN_MS, { shotAt: 3000 });
    out.samples = runLog.length;
    out.states = collapseStates(runLog);
    const first = runLog[0], last = runLog[runLog.length - 1];
    out.moved = !!first && !!last && dist(first, last) > MOVE_MIN_IN;
    out.view = last ? last.view : null;
    out.shots = {
      start: first ? first.shots : null, end: last ? last.shots : null, scored_end: last ? last.scored : null,
      increments: runLog.reduce((n, s, i) => n + (i > 0 && s.shots > runLog[i - 1].shots ? 1 : 0), 0),
    };
    out.run_errors = last ? last.errors : [];
    out.assertions.moved = out.moved;
    out.assertions.view_3d = runLog.some(s => s.view === '3d');

    // Stop: the run goes idle and the robot is back at the auto start pose.
    await page.click('#ar-stop');
    await sleep(600);
    const afterStop = await page.evaluate(() => { const sim = window.__sim, r = sim.autoPlanner.runner; return { state: r ? r.state : 'none', x: sim.S.x, y: sim.S.y, h: sim.S.h, view: sim.S.view }; });
    out.stop = afterStop;
    out.assertions.stop_idle = afterStop.state === 'idle';
    out.assertions.stop_pose_at_auto_start = Math.abs(afterStop.x - START.x) < 0.5 && Math.abs(afterStop.y - START.y) < 0.5;

    // Restart: runs the same auto again from the start, and the robot moves.
    await page.evaluate(() => { if (window.__arLog) window.__arLog.splice(0); });
    await page.click('#ar-restart');
    // SwiftShader runs ~0.5 fps and the sim clamps dt to 0.05 s per frame, so wall time cannot advance the sim: step it deterministically.
    await page.waitForFunction(() => { const r = window.__sim.autoPlanner.runner; return r && r.state === 'running'; }, { timeout: 30000 });
    await page.evaluate(() => { const sim = window.__sim, r0 = sim.autoPlanner.runner; window.__arLog.push({ t: 0, x: sim.S.x, y: sim.S.y, h: sim.S.h, view: sim.S.view, state: r0.state, time: r0.time, shots: 0, scored: 0, errors: [] }); for (let i = 0; i < 100; i++) sim.step(0.05); const r = sim.autoPlanner.runner; window.__arLog.push({ t: 0, x: sim.S.x, y: sim.S.y, h: sim.S.h, view: sim.S.view, state: r.state, time: r.time, shots: sim.S.shots, scored: sim.S.scored, errors: r.errors.slice(-2) }); });
    const restartLog = await collect(page, 1500, {});
    const r0 = restartLog[0], rN = restartLog[restartLog.length - 1];
    const restartRunning = restartLog.some(s => s.state === 'running');
    const restartMoved = restartRunning && !!r0 && !!rN && dist(r0, rN) > MOVE_MIN_IN;
    out.restart = { moved: restartMoved, running_seen: restartRunning, states: collapseStates(restartLog), first: r0 ? { x: r0.x, y: r0.y } : null, last: rN ? { x: rN.x, y: rN.y } : null, view: rN ? rN.view : null, samples: restartLog.length };
    out.assertions.restart_moves = restartMoved;

    out.frames_total = await page.evaluate(() => window.__arFrames || 0);
    out.toasts = await page.evaluate(() => [...document.querySelectorAll('[class*="toast"]')].map(e => (e.textContent || '').trim()).filter(Boolean).slice(-6));
    await page.evaluate(stopLogger);
  } catch (e) {
    out.error = String(e && e.message || e);
    out.exit = 1;
  } finally {
    try { if (browser) await browser.close(); } catch (_) { /* ignore */ }
    await new Promise(r => srv.close(() => r()));
  }
  const failed = Object.entries(out.assertions).filter(([, v]) => v !== true).map(([k]) => k);
  if (failed.length || out.error) out.exit = 1;
  out.failed_assertions = failed;
  console.log(JSON.stringify(out, null, 2));
  process.exitCode = out.exit;
}

// Collects the page-side log for `ms` milliseconds, optionally taking one screenshot at `shotAt`.
async function collect(page, ms, { shotAt = null } = {}) {
  const got = [];
  const t0 = Date.now();
  let shot = false;
  while (Date.now() - t0 < ms) {
    await sleep(Math.min(1000, Math.max(0, ms - (Date.now() - t0))));
    got.push(...await page.evaluate(takeLog));
    if (shotAt != null && !shot && Date.now() - t0 >= shotAt) {
      shot = true;
      try { await page.screenshot({ path: SHOT_PATH }); } catch (_) { /* ignore */ }
    }
  }
  return got;
}

main().catch(e => { console.error(e); process.exitCode = 1; });
