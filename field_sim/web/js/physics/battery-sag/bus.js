// bus.js - DC bus solve for the battery + loads network. Pure functions, SI units (V, A, ohm, W).
import { BUS } from './params.js';

// Thevenin source (vth [V], rs [ohm]) feeding parallel loads.
// Loads: {kind:'linear', g [S], a [A]} -> I = g*V - a ; {kind:'current', amps [A]} ; {kind:'power', watts [W]} -> I = W/max(V, powerVMin).
// Linear + current parts are solved in closed form; power loads by fixed-point iteration (opts.iters).
export function solveBus(vth, rs, loads, opts = BUS) {
  let G = 0, A = 0, Ic = 0;
  for (const l of loads) {
    if (l.kind === 'linear') { G += l.g; A += l.a; }
    else if (l.kind === 'current') Ic += l.amps;
  }
  const powerLoads = loads.filter(l => l.kind === 'power');
  let v = vth;
  if (rs > 0) {
    for (let k = 0; k < opts.iters; k++) {
      let Ip = 0;
      for (const l of powerLoads) Ip += l.watts / Math.max(v, opts.powerVMin);
      v = (vth + rs * (A - Ic - Ip)) / (1 + rs * G);
    }
  }
  v = Math.max(opts.vFloor, v);
  const currents = loads.map(l => {
    if (l.kind === 'linear') return l.g * v - l.a;
    if (l.kind === 'current') return l.amps;
    return l.watts / Math.max(v, opts.powerVMin);
  });
  let iTotal = 0;
  for (const c of currents) iTotal += c;
  return { v, iTotal, currents };
}
