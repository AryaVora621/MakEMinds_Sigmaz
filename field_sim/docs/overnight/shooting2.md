# shooting2: shooting and tipping with pollen and nectar (haiku edit task)

Scope: owned files only (balls.js, collision.js, shotrules.js, shot_core.js, aimcontrol.js, shootend2end.test.mjs,
physics3d/shotaudit tests, new ballkinds_shootfix.test.mjs, this report). No git commands run.

## Root causes found (verified headless)
1. Nectar shots from the app fly at pollen speed. `web/js/app.js:371-376` (`evalCtx`) builds the shot context with no
   `kind`, so `shotrules.js:36` (`kindName`) and `shotrules.js:145` (`forwardShot`) use pollen. `app.js:425`
   (`world.addBall(kind, ...)`) then spawns a nectar ball at pollen speed. Measured: nectar solved for nectar at 3018 rpm,
   fired as pollen, leaves at 268.8 in/s instead of 209.9 and misses (closest approach 24.9 in). Fired with `kind:'nectar'`
   it scores (blue and red).
2. Held balls fired without the shot flag were taken back in. `balls.js:312` (`fireHeld`) launched the oldest held ball.
   Fixed earlier: `balls.js:316` now sets `shot = true`. Verified: with that line reverted, the test
   "a ball fired from a held slot is not taken straight back in" fails ("fired ball was re-intaken").
3. Three shooting tests assumed the wrong tip rule, which made them fail (baseline 629 tests: 624 pass, 4 fail, 1 skipped).
   `hivecycle.js:156` (`_shouldTip`) reads `ballsToTip` only in 'count' mode; the default mode is 'weight' (threshold 8
   pollen, `hivecycle.js:54,57`). Tests passing `ballsToTip: 1000 / 100000` tipped at 8 pollen and the aim moved, so later
   shots missed. The solver test expected a tip after 1 shot, which contradicts the user's 8-pollen fact.

## Changes
- `tests_js/shootend2end.test.mjs`:
  - `NEVER_TIP = {mode:'count', ballsToTip:1000}` for the continuous-fire and unlimited-recycle tests.
  - Solver test now fires 8 pollen (60 ticks apart), asserts no tip after 7, exactly 1 tip after 8 (both alliances).
    Fails on the old version: "red did not tip" (1 shot).
- `tests_js/ballkinds_shootfix.test.mjs` (new, 6 tests, all pass with current code):
  kind radii and launch scale; same rpm gives nectar speed x0.781; nectar solve + nectar fire lands at solver speed and
  kind mismatch gives 1/0.781; fireHeld keeps kind/type and does not re-intake; mixed hold capped at 4 combined (2 pollen
  + 2 nectar + 2 extra stay on the field); tipReady mixed combinations.
- No edits to balls.js, shotrules.js, shot_core.js, aimcontrol.js, collision.js this pass (the code paths above already
  work when called with the right kind).

## Ball kinds (balls.js:31-41; `BALL_KINDS`)
| kind | radiusIn | diameter | mass | source | status |
|---|---|---|---|---|---|
| pollen | 1.391 | 2.8 in | 0.025 kg (0.055 lb) | FTC BIOBUZZ manual TU06 Sec 9.8 via https://ftc.uprobotics.tech/questions/78 ; mass from Plex/AndyMark snippets | diameter SOURCED, mass medium confidence |
| nectar | 1.7995 | 3.6 in | 0.041 kg (0.091 lb) | same manual section for diameter; mass pairing from an ambiguous AndyMark snippet | mass ASSUMED |
Launch scale: exit speed = wheel speed x sqrt(m_pollen / m_kind) (`balls.js:43`). Nectar 0.781, ASSUMED, not measured.

## Hold limit and hive tip
- Hold limit: 4 total, pollen + nectar (`balls.js:76` MAX_HELD = 4, matches OVERNIGHT_RULES and matchrules). Enforced in
  the intake (`balls.js:547`) and in the FIFO `held` list. NOT enforced by app.js shooting (see remaining issues).
- Tip: 8 POLLEN. User-authoritative fact, NOT in the manual (manual Sec 8/9.6/10.5.1 says "enough" scoring elements).
  Implemented in `hivetip_model.js` (HIVE_TIP.threshold = 8) and used by `hivecycle.js` weight mode.
- Nectar tip threshold: UNKNOWN in any source found. Current model value nectarWeight = 0.091/0.055 = 1.6545 is an
  ASSUMPTION (mass ratio used as tip weight). Derived: 5 nectar alone tip (ceil(8/1.6545)). Sources checked:
  manual research (docs/overnight/research_manual.md), postcard wording in hivetip_model.js header (no count), a video
  summary (no figure). Needs a measured or official count before it is treated as fact.

## Mixed combinations (linear weight, ASSUMED)
- Tip when pollen x 1 + nectar x 1.6545 >= 8. Examples (tipReady, tested): 4P+2N = 7.31 no; 4P+3N = 8.96 yes;
  0P+5N = 8.27 yes; 0P+4N = 6.62 no. No source gives a mixed-piece rule; additivity is an assumption.

## Tests
- Full run after changes: `node --test tests_js/*.test.mjs` = 650 tests, 649 pass, 0 fail, 1 skipped
  (pedrofollow.test.mjs, JDK/Pedro jar not available). The count includes other owners' concurrent test additions.
- Shooting suites: shootend2end 14/14 with ballkinds_shootfix (run together).

## Remaining issues in non-owned files (not edited)
1. `web/js/app.js:371-376` evalCtx: add `kind: S.ballKind` so nectar shots use the nectar launch scale. Without it nectar
   shots from the app miss (see root cause 1). Also feed the same kind to the auto solver (`S.sol`).
2. `web/js/app.js:421-431` shoot(): does not call `world.fireHeld` and does not consume the held FIFO; `S_PRACTICE.ammo`
   only counts when `ammoLimit` is on, and `setIntake` space is Infinity by default (`app.js:448`). The 4-ball hold is
   therefore not enforced in default practice. Route shoot() through fireHeld (kind taken from the held ball).
3. `web/js/app.js:616` preloads `RULES.preloadPollen` (4) pollen; nectar preload is not modelled in practice mode.
4. `web/js/hivecycle.js:156`: `ballsToTip` is ignored in weight mode. Any caller that sets ballsToTip expecting a count
   change must also pass mode:'count' or tipWeightThreshold.
5. `web/js/balls.js:378` `_cellCapacity` uses a hard 3.62 in diameter; `balls.js:364` slot spacing uses rS 1.81. Both are
   fine for nectar, but they should use BALL_KINDS if sizes change.
6. Visual check not done (no headless screenshot in this pass).
