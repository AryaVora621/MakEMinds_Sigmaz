# hive-physics report (Sonnet lead, step 3 integrate)

## Decision (ASSUMED, manual silent)
BIOBUZZ manual (t01, sections 8/9/10) never says pieces leave a CELL on a tip; it only scores pieces in the upward CELL at match end
(2 pts each). So drop-out is a design decision, not a sourced rule. Basis: CAD geometry (t03, scene.json): at a stable tilt the
DOWN cell floor falls 30 deg toward its opening with no lip, so a ball there rolls out; the UP cell floor falls inward and holds.
Hold limit 4 and tip threshold 8 pollen (nectar 1.75, INFERRED) are unchanged.

## Changed files (all owned)
- web/js/balls.js: new `_releaseDrained()` (called each step). A stored ball whose cell floor now slopes toward its opening
  (world floorU_z > DROP_SLOPE = 0.17, ~10 deg) becomes a free ball at its current position with the hive's tangential velocity;
  gravity + hive collider roll it out. No teleport. Options: `env.dropOnTip` (default true), hooks `onRelease(b,hive,cell)` and
  `onRecycle(b,hive,cell)` (cell-full recycle), counter `world.released`.
- web/js/hivecycle.js: `releaseOnTip` default false -> true (model mirrors the physics); header comments updated;
  new `forgetBall(cell, n)` removes oldest records of a cell (for the hooks above).
- tests_js/hive_dropout.test.mjs (new, 5 tests): up cell holds at rest; 8 pollen tip then old up cell empties physically and balls
  end outside the hive footprint; dropOnTip=false legacy; model has no records in down cell; forgetBall.
- tests_js/hivecycle.test.mjs (2 tests) and hivetip_mixed.test.mjs (1 test) updated to the new default (down cell empties after a
  tip, onTip count is post-release; legacy keep path tested with releaseOnTip:false).
- Haiku tests h_hive-physics_t17/t18/t19 kept as is (pass, still correct).

## Evidence (commands I ran)
- `node --test tests_js/hive*.test.mjs tests_js/sim_physics.test.mjs tests_js/h_hive-physics_*.test.mjs tests_js/hivetip*.mjs`: 97 pass, 0 fail.
- `node --test tests_js/*.test.mjs` (final run): tests 973, pass 954, fail 0. Earlier mid-run showed failures in other owners' files
  (h_shoot-physics_t09/t13, hold-limit literal, etc.); they were red/green depending on concurrent edits by others.
- Sign of floorU confirmed empirically: test shows up cell keeps balls (released==0) and old up cell drains after a tip.

## Haiku findings reviewed
- t01 (manual silent on fall-out): accepted, basis of the ASSUMED label. t03 (down cell drains by geometry): accepted, verified by test.
- t04/t10 (down-cell balls stuck forever, releaseOnTip silent drop and ghost): fixed via drop-out + model default + hooks.
- t04 overflow recycle left a model record: hook `onRecycle` added, but app.js wiring is not done (see patches).
- t08 stale 1.6545 comments in hivetip_model.js header: real but cosmetic, NOT edited (no test impact).
- Not acted on: starting-nectar double counting (t06 F1, "likely, untested") and HIVE_BALL_COUNTS dead state; unverified, left alone.

## Unverified
- No visual check: I did not take a screenshot of balls falling in the 3D view.
- DROP_SLOPE 10 deg is a chosen value, not measured. Real friction/lip/mesh of the physical hive is unknown (no lip in CAD corners).
- Wall-clock cost of releasing many balls at once not profiled. Manual section 11 G-rules not read.
- New up cell is not "physically empty" when balls were in it before the tip; with drop-out they fall when it was down, so this mostly
  resolves, but a ball resting in an up cell that never tipped stays uncounted for the tip (counts=false after a tip) - unchanged.

## Patches for other owners (app.js)
1. In app.js after creating `world`: `world.onRelease = world.onRecycle = (b, alliance, cell) => hiveDyn[alliance].forgetBall(cell, 1);`
   Without it, mid-tip drops are reconciled only at tip settle by releaseOnTip, and cell-full recycles leave a ghost model record.
2. Optional: toast/stat for `world.released`. Tests of other owners that count `state==='scored'` after shooting a static hive
   (h_shoot-physics_t09/t13) can lose balls that landed in the down cell; pass `dropOnTip:false` in their env if that appears.
