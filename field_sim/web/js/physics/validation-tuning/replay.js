// Replay logged driver commands through MecanumDrive and compare to the logged pose track.
// Log = {source, meta, series:{[name]:{t:number[] (s), v:any[], type}}} (P2 contract). Units: inches, rad, s.
// Command series names default to fwd, strafe, turn; pose series to x, y, heading (alias h). Sample-and-hold at the
// sim dt = median log dt of the command series. The sim starts at the first logged pose.
import { MecanumDrive, integrate } from '../../driver.js';
import { median } from './stats.js';
import { compareTracks } from './metrics.js';

function holdAt(series, t, idx) {   // last sample with series.t <= t; idx is a moving cursor {i}
  if (!series) return 0;
  while (idx.i + 1 < series.t.length && series.t[idx.i + 1] <= t) idx.i++;
  return series.t[idx.i] <= t ? Number(series.v[idx.i]) || 0 : 0;
}

export function replayDriveLog(log, opts = {}) {
  const S = log.series || {};
  const cmdNames = { fwd: 'fwd', strafe: 'strafe', turn: 'turn', ...(opts.cmd || {}) };
  const poseNames = { x: 'x', y: 'y', h: S.heading ? 'heading' : 'h' };   // opts.pose = {x,y,h} overrides the START pose (numbers)
  const px = S[poseNames.x], py = S[poseNames.y], ph = S[poseNames.h];
  if (!px || !py) throw new Error('replayDriveLog: log needs x and y pose series');
  const cmds = { fwd: S[cmdNames.fwd], strafe: S[cmdNames.strafe], turn: S[cmdNames.turn] };
  const ref = cmds.fwd || cmds.strafe || cmds.turn;
  if (!ref) throw new Error('replayDriveLog: log needs a command series (fwd/strafe/turn)');
  const dts = [];
  for (let i = 1; i < ref.t.length; i++) { const d = ref.t[i] - ref.t[i - 1]; if (d > 0) dts.push(d); }
  const dt = dts.length ? median(dts) : 1 / 60;

  const t0 = px.t[0], tEnd = px.t[px.t.length - 1];
  const real = { t: px.t.slice(), x: px.v.map(Number), y: py.v.map(Number) };
  if (ph) real.h = ph.v.map(Number);
  const start = {
    x: opts.pose?.x ?? real.x[0],
    y: opts.pose?.y ?? real.y[0],
    h: opts.pose?.h ?? (real.h ? real.h[0] : 0),
  };
  const drive = new MecanumDrive(opts.driveOpts || {});
  const state = { x: start.x, y: start.y, h: start.h, bodyV: [0, 0, 0] };
  const cur = { fwd: { i: 0 }, strafe: { i: 0 }, turn: { i: 0 } };
  const sim = { t: [t0], x: [state.x], y: [state.y], h: [state.h] };
  const n = Math.round((tEnd - t0) / dt);
  for (let k = 1; k <= n; k++) {
    const t = t0 + (k - 1) * dt;   // command holds over the interval starting at t
    const cmd = {
      fwd: holdAt(cmds.fwd, t, cur.fwd), strafe: holdAt(cmds.strafe, t, cur.strafe), turn: holdAt(cmds.turn, t, cur.turn),
    };
    const r = drive.step(cmd, dt, state, { fieldCentric: !!opts.fieldCentric });
    integrate(state, r.vx, r.vy, r.om, dt);
    sim.t.push(t0 + k * dt); sim.x.push(state.x); sim.y.push(state.y); sim.h.push(state.h);
  }
  if (!ph) { delete sim.h; }
  return { sim, real, metrics: compareTracks(sim, real) };
}
