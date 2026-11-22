// hivetip_model.js - pure tip model for a BIOBUZZ hive: how many POLLEN and NECTAR in the
// up cell make the hive tip. ES module, no imports, no DOM, no state.
//
// Units: "pollen-equivalent weight". One pollen = pollenWeight, one nectar = nectarWeight.
// The tip threshold is expressed in the same units, so threshold = 8 * pollenWeight = 8.
//
// SOURCE LEDGER (every number below is tagged SOURCED or ASSUMED)
//
//  threshold = 8 pollen (pollen-equivalent weight 8):
//    SOURCED: user-authoritative (the project owner, from the team's own observation of the
//    hive). NOT in the Competition Manual: the manual says a hive tips when "enough" SCORING
//    ELEMENTS are launched into the up cell (manual Sec 8, Sec 9.6, 10.5.1) and gives no count.
//    Official trigger wording, from the game postcard (search snippet only, not fetched):
//    "when enough POLLEN or NECTAR is launched into the CELL the HIVE will TIP"
//    https://www.firstinspires.org/hubfs/web/program/ftc/biobuzz-gamepostcard.pdf
//
//  pollenWeight = 1:
//    ASSUMED: the unit of the scale. Only the ratio nectarWeight / pollenWeight matters.
//
//  nectarWeight = 1.75 (INFERRED, range 1.75..2.0, from a team test table; see below).
//    Team 14343 build thread (Sept 14 2026 "Tipping Point" post) lists tip mixes 0n8p,1n7p,2n6p,3n3p,4n1p,5n0p
//    https://www.chiefdelphi.com/t/ftc-14343-escape-velocity-biobuzz-build-thread/524080
//    3n3p and 4n1p tipping needs nectar>=1.75; 4 nectar alone not tipping needs <2. 5 nectar alone tips.
//    The post is unclear on whether it is a minimum list, and a sloppy 2n5p test also tipped, so treat as approximate.
//  Older note: AndyMark mass ratio 0.091/0.055 = ~1.6545:
//    SOURCED (third-party, mass ratio, not a tip count): AndyMark's BIOBUZZ game-set listing
//    gives nectar 0.091 lb and pollen 0.055 lb.
//    https://andymark.com/products/ftc-2026-27-biobuzz-game-sets
//    Plex also lists pollen 0.055 lb. Those figures came from search results in the research
//    digest and were not fetched directly for this file.
//    CAVEAT (ASSUMED link): using ball MASS as the tip weight is an assumption. The
//    third-party video summary (https://youtubesummary.com/summary/sUH3z5a5S9I) only says
//    nectar is "larger/heavier than pollen and can tip the hive faster", with no figure.
//    No source gives a measured nectar tip count. Nectar weight is therefore ASSUMED
//    proportional to mass.
//    Units check: 1.6545 * 4 = 6.6 pollen, so nectarToTip() = ceil(8 / 1.6545) = 5 nectar.
//    That is a derived value, not a measured one.
//
//  Mixed nectar + pollen: a linear sum, tip when pollen*pollenWeight + nectar*nectarWeight >= 8.
//    ASSUMED: additivity (every ball contributes its weight to one shared total). No source
//    gives a mixed-piece tip rule. Example: 4 pollen + 1 nectar = 4 + 1.6545 = 5.65, not tipped;
//    4 pollen + 2 nectar = 7.31, not tipped; 4 pollen + 3 nectar = 8.96, tipped.
//
//  maxHeldBalls = 4 (robot hold limit):
//    SOURCED: user-authoritative and the official hold limit (repo: docs/OVERNIGHT_RULES.md
//    "Official ball hold limit is 4"). This is the robot's carry capacity, not a tip count.
//    It does NOT limit how many balls can be in the up cell over the match.
//
//  startingNectar = 3:
//    SOURCED: manual 10.3.1.B, per the repo's hivecycle.js header.
//    Not used by this model. Listed for the caller's reference only.
//    NOTE (not resolved here): docs/overnight/research_manual.md records 5 nectar at start.
//    The conflict is unresolved.
//
// These are model choices, not measured robot or hive facts. Any value marked ASSUMED
// should be replaced after measuring the real hive.

const SOURCE = {
  pollenThreshold: 'user-authoritative: 8 pollen tip a hive; NOT in manual',
  nectarWeight: 'SOURCED https://andymark.com/products/ftc-2026-27-biobuzz-game-sets (mass ratio 0.091 lb / 0.055 lb); ASSUMED that tip weight is proportional to mass',
  notes: [
    'threshold is in pollen-equivalent weight units; 8 * pollenWeight(1) = 8',
    'nectarToTip = ceil(8 / 1.6545) = 5 nectar alone (derived, not measured)',
    'mixed nectar + pollen is a linear weight sum (ASSUMED, no source)',
    'maxHeldBalls 4 is the robot hold limit (SOURCED: user + OVERNIGHT_RULES), not a tip count',
    'startingNectar 3 per manual 10.3.1.B; manual copy in research_manual.md says 5 (CONFLICT, unresolved)',
  ],
};

export const HIVE_TIP = {
  pollenWeight: 1,            // ASSUMED unit scale
  nectarWeight: 1.75,          // INFERRED from Chief Delphi 14343 table (3n3p and 4n1p tip, 4n alone does not => 1.75..2); AndyMark mass ratio 1.6545 is too low for that
  threshold: 8,                // 8 * pollenWeight; SOURCED user fact, not in manual
  maxHeldBalls: 4,             // SOURCED: robot hold limit
  startingNectar: 3,           // SOURCED: manual 10.3.1.B (conflict noted above)
  source: SOURCE,
};

const EPS = 1e-9;

/** Non-negative integer count: negatives and NaN become 0, fractions round down. */
function count(n) {
  const v = Number(n);
  if (!Number.isFinite(v)) return 0;
  return Math.max(0, Math.floor(v));
}

/** Total tip weight of a set of counts, in pollen-equivalent units. */
export function tipWeight({ pollen = 0, nectar = 0 } = {}, params = HIVE_TIP) {
  return count(pollen) * params.pollenWeight + count(nectar) * params.nectarWeight;
}

/** True when the counts are at or above the tip threshold. */
export function shouldTip(counts = {}, params = HIVE_TIP) {
  return tipWeight(counts, params) >= params.threshold - EPS;
}

/** Progress toward a tip, in [0, 1]. Monotone in each count. */
export function tipProgress(counts = {}, params = HIVE_TIP) {
  if (!(params.threshold > 0)) return 1;
  const p = tipWeight(counts, params) / params.threshold;
  return Math.min(1, Math.max(0, p));
}

/** Nectar alone that tips a hive. */
export function nectarToTip(params = HIVE_TIP) {
  if (!(params.nectarWeight > 0)) return Infinity;
  return Math.ceil(params.threshold / params.nectarWeight - EPS);
}

/**
 * How many more balls of `ballType` ('pollen' | 'nectar') are needed to tip, given the
 * current counts. 0 when already tipped.
 */
export function ballsNeeded(counts = {}, ballType = 'pollen', params = HIVE_TIP) {
  const remaining = params.threshold - tipWeight(counts, params);
  if (remaining <= EPS) return 0;
  const w = ballType === 'nectar' ? params.nectarWeight : params.pollenWeight;
  if (!(w > 0)) return Infinity;
  return Math.ceil(remaining / w - EPS);
}

/**
 * Settings for HiveDynamics.setRule(). Mode 'weight' with the threshold, plus an equivalent
 * pollen-only ballsToTip for callers that only read counts.
 */
export function hivePolicyFromParams(params = HIVE_TIP) {
  return {
    ballsToTip: Math.ceil(params.threshold / params.pollenWeight - EPS),
    mode: 'weight',
    pollenWeight: params.pollenWeight,
    nectarWeight: params.nectarWeight,
    tipWeightThreshold: params.threshold,
  };
}
