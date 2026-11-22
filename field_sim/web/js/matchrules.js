// BIOBUZZ match rules and scoring (pure, no DOM; unit-testable in node).
// Source: official FTC Competition Manual TU08 (section numbers in `ref`). Items flagged official:false are
// assumptions NOT stated in the manual and must be confirmed.
//
// NOTE: the manual gives NO ball count/mass for a tip; that is physical (see hivecycle.js, an assumption).

export const RULES = {
  source: 'BIOBUZZ manual TU08',
  holdLimit: 4,           // G407: max scoring elements controlled (5 momentary tolerated; 6+ strategic)
  preloadPollen: 4,       // 10.3.1: 4 pollen preload per robot
  timing: {
    auto: 30,             // 10.1
    transition: 8,        // 10.1 / G403 (no powered movement)
    teleop: 120,          // 10.1
    endgame: 30,          // sim convention only (standard FTC endgame; no manual meaning)
    endgameFlowerUnlockRemaining: 60,   // G410: no flower scoring before the last 60 s (major foul per nectar)
  },
  points: {
    tipAuto: 20,          // Table 10-2: each tip completed in AUTO (tips before TELEOP count as AUTO)
    tipTeleop: 20,        // Table 10-2
    ballInUpCellAtEnd: 2, // Table 10-2 (TELEOP, balls remaining in the UP cell)
    leave: 3,             // Table 10-2 / 10.5.4 (AUTO)
    parkAuto: 5,          // Table 10-2 (at least partially in own LOADING ZONE)
    parkTeleop: 5,        // Table 10-2
    bottomNectarBonus: 5, // Table 10-2 (per flower, TELEOP)
    ballInOwnedFlower: 2, // Table 10-2 / 10.5.2
    ballInGarden: 1,      // Table 10-2 / 10.5.3
    win: '3 RP',          // Table 10-2
    tie: '1 RP',          // Table 10-2
  },
  nectar: {
    perAlliance: 5,             // 10.3.1: 5 nectar in each alliance area at start
    humanEntersOnOwnTip: 1,     // G426: one nectar per own-hive tip
    remainingAtSecondsLeft: 60, // G426: all remaining at 60 s left
  },
  robotLimits: {
    start: [18, 18, 18],        // R102 (in)
    expanded: [18, 24, 29],     // R105 (in)
  },
  // Provenance per dotted path (prefix match applies to children). official:true = manual states it.
  provenance: {
    'source': { official: true, ref: 'Competition Manual TU08' },
    'holdLimit': { official: true, ref: 'G407' },
    'preloadPollen': { official: true, ref: '10.3.1' },
    'timing.auto': { official: true, ref: '10.1' },
    'timing.transition': { official: true, ref: '10.1, G403' },
    'timing.teleop': { official: true, ref: '10.1' },
    'timing.endgame': { official: false, ref: 'assumption: sim-only last-30s label' },
    'timing.endgameFlowerUnlockRemaining': { official: true, ref: 'G410 (Table 9-1 text garbled; 1:00 chime matches)' },
    'points.win': { official: true, ref: 'Table 10-2' },
    'points.tie': { official: true, ref: 'Table 10-2' },
    'points': { official: true, ref: 'Table 10-2' },
    'nectar': { official: true, ref: '10.3.1, G426' },
    'robotLimits': { official: true, ref: 'R102, R105' },
  },
};

/** True when the dotted rule path is flagged official (longest matching provenance prefix wins). */
export function isOfficial(path, rules = RULES) {
  const parts = path.split('.');
  for (let n = parts.length; n >= 1; n--) {
    const p = rules.provenance[parts.slice(0, n).join('.')];
    if (p) return p.official === true;
  }
  return false;
}

const ALLIANCES = ['red', 'blue'];
const chk = a => { if (!ALLIANCES.includes(a)) throw new Error('unknown alliance ' + a); };

function blank() {
  return {
    tipsAuto: 0, tipsTeleop: 0, flowerBalls: 0, bottomNectar: 0, gardenBalls: 0,
    parkAuto: false, parkTeleop: false, leave: false, upCell: 0,
  };
}

export class MatchScorer {
  constructor(rules = RULES) { this.rules = rules; this.reset(); }
  reset() {
    this.s = { red: blank(), blue: blank() };
    this.history = [];
    this.finalized = false;
    this.hiveStates = null;
  }
  _log(type, alliance, extra = {}) { this.history.push({ type, alliance, ...extra }); }

  /** A completed tip of the alliance's hive. 'auto' and 'transition' count as AUTO (10.5). */
  onTip(alliance, phase) {
    chk(alliance);
    const auto = phase === 'auto' || phase === 'transition';
    if (auto) this.s[alliance].tipsAuto++; else this.s[alliance].tipsTeleop++;
    this._log('tip', alliance, { phase, points: auto ? this.rules.points.tipAuto : this.rules.points.tipTeleop });
    return true;
  }
  /** Ball in a flower. Not scored before the last 60 s (G410). Scores only if the alliance owns the flower. */
  onBallInFlower(alliance, ownedFlag, secondsLeft) {
    chk(alliance);
    if (secondsLeft > this.rules.timing.endgameFlowerUnlockRemaining) {
      this._log('flower_rejected', alliance, { secondsLeft });
      return false;
    }
    if (!ownedFlag) { this._log('flower_unowned', alliance, { secondsLeft }); return false; }
    this.s[alliance].flowerBalls++;
    this._log('flower', alliance, { points: this.rules.points.ballInOwnedFlower });
    return true;
  }
  /** Bottom-nectar bonus (per flower, TELEOP). Also subject to the flower window. */
  onBottomNectar(alliance, secondsLeft) {
    chk(alliance);
    if (secondsLeft > this.rules.timing.endgameFlowerUnlockRemaining) return false;
    this.s[alliance].bottomNectar++;
    this._log('bottom_nectar', alliance, { points: this.rules.points.bottomNectarBonus });
    return true;
  }
  onBallInGarden(alliance) {
    chk(alliance);
    this.s[alliance].gardenBalls++;
    this._log('garden', alliance, { points: this.rules.points.ballInGarden });
    return true;
  }
  /** Park scores once per phase per alliance. */
  onPark(alliance, phase) {
    chk(alliance);
    const key = phase === 'auto' ? 'parkAuto' : 'parkTeleop';
    if (this.s[alliance][key]) return false;
    this.s[alliance][key] = true;
    this._log('park', alliance, { phase });
    return true;
  }
  /** Leave (AUTO) scores once. */
  onLeave(alliance) {
    chk(alliance);
    if (this.s[alliance].leave) return false;
    this.s[alliance].leave = true;
    this._log('leave', alliance);
    return true;
  }

  _breakdown(a) {
    const s = this.s[a], p = this.rules.points;
    const tips = s.tipsAuto + s.tipsTeleop;
    const tipPoints = s.tipsAuto * p.tipAuto + s.tipsTeleop * p.tipTeleop;
    const upCellBalls = s.upCell;
    const parkPoints = (s.parkAuto ? p.parkAuto : 0) + (s.parkTeleop ? p.parkTeleop : 0);
    const leavePoints = s.leave ? p.leave : 0;
    const flowerPoints = s.flowerBalls * p.ballInOwnedFlower + s.bottomNectar * p.bottomNectarBonus;
    const gardenPoints = s.gardenBalls * p.ballInGarden;
    const upCellPoints = upCellBalls * p.ballInUpCellAtEnd;
    return {
      tips, tipPoints, upCellBalls, upCellPoints, parkPoints, leavePoints, flowerPoints, gardenPoints,
      total: tipPoints + upCellPoints + parkPoints + leavePoints + flowerPoints + gardenPoints,
    };
  }
  /** upCellCounts = {red, blue}: balls remaining in each alliance's hive UP cell at the end. hiveStates kept for display. */
  finalize(hiveStates = null, upCellCounts = {}) {
    for (const a of ALLIANCES) this.s[a].upCell = Math.max(0, upCellCounts[a] | 0);
    this.hiveStates = hiveStates;
    this.finalized = true;
    return { red: this._breakdown('red'), blue: this._breakdown('blue') };
  }
  summary() {
    const red = this._breakdown('red'), blue = this._breakdown('blue');
    const winner = red.total > blue.total ? 'red' : blue.total > red.total ? 'blue' : 'tie';
    const rp = winner === 'tie' ? { red: 1, blue: 1 } : { red: winner === 'red' ? 3 : 0, blue: winner === 'blue' ? 3 : 0 };
    return { red, blue, winner, rp, finalized: this.finalized };
  }
}

/** Human nectar entry (G426): one nectar per own-hive tip, all remaining at 60 s left. Stateful. */
export class NectarRelease {
  constructor(rules = RULES) { this.rules = rules; this.reset(); }
  reset() { this.released = 0; }
  /** @returns {{release:number}} nectar the human may enter now (also counted as released). */
  update(secondsLeft, tipsSoFar) {
    const n = this.rules.nectar;
    const pool = n.perAlliance - this.released;
    if (pool <= 0) return { release: 0 };
    let r;
    if (secondsLeft <= n.remainingAtSecondsLeft) r = pool;
    else r = Math.min(pool, Math.max(0, tipsSoFar * n.humanEntersOnOwnTip - this.released));
    this.released += r;
    return { release: r };
  }
}
