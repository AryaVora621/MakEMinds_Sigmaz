# app2 report: planner view, ball kind, hive tip HUD, reset hooks (area: app)

Owned files: web/js/app.js, web/index.html, web/css/ui.css, web/js/autorunner.js, web/js/driver.js, tests_js/ui_ids.test.mjs, tests_js/controls.test.mjs, tests_js/autorunner.test.mjs, this file.
This pass edited NO code. It verified the current working tree with screenshots and tests, and records what is in place.

## 1. What changed (current tree, as found)

Planner view (web/js/app.js):
- Flip: `PLAN_FLIP` (app.js:747) is bound to the "View from alliance wall" checkbox `#plan-flip` (web/index.html:33, app.js:969-970). It only applies on blue. The view transform comment at app.js:768-771 says the flip comes from pathing.js `allianceViewTransform` (blue -> rotate 180 about the crop centre).
- Crop: `fitMap` (app.js:759-763) fits the Auto planner to the chosen alliance half only. Caption text: "Blue half, x 72 to 144. Eye button: full field" (seen in screenshot).
- Start marker and footprints: `drawPlannerPoses()` (app.js:831) draws the start pose and each selected path end pose as an 18 in footprint with a heading arrow. The footprint rotates by pi - h when flipped (app.js:815 comment).
- Ball kind selector: `S.ballKind` ('pollen' | 'nectar', app.js:274), `setBallKind` (app.js:528-536), buttons `#kind-pollen` / `#kind-nectar` (web/index.html:80-81), and the shot path uses the kind (app.js:424).
- Hive tip HUD: per-hive readout in the 3D sidebar (app.js:1097-1112). It shows "Pollen n / 8 Nectar m" and a FILLING / TIPPED state. The nectar tip count is shown as "unknown" unless the assumed weight is available (app.js:1112).
- ballsToTip fix: the old value 4 is replaced by 8 pollen (web/js/hivecycle.js:53 `ballsToTip: HIVE_TIP.threshold / HIVE_TIP.pollenWeight`, comment at hivecycle.js:25). The 8 is user-stated (see section 3).
- Reset hooks: `resetField()` (app.js:547-548) passes the scene balls explicitly to `world.resetField({ hiveDyn, balls })`. The comment at app.js:546 says this stops a Clear from leaving the field empty. `BallWorld.resetField(opts)` is at balls.js:250.

## 2. Evidence

Screenshots (headless Chrome 1280x800 and 1500x900, SwiftShader, saved in the scratchpad):
- `C:\Users\Jindal\AppData\Local\Temp\claude\1bf884c3629a4194\cab1052d-b484-449d-8151-8eddbe3da6ad\scratchpad\app2_3d_c.png`: 3D view renders. Hive tip HUD reads "Red hive tip FILLING Pollen 0 / 8 Nectar 0". Shooting panel shows Auto, Assist, Manual, Free modes. The "Enforce legal area" toggle is on, and the legal-area note says "[assumed, no official launch zone]".
- `...\scratchpad\app2_auto.png`: Auto planner renders in the cropped blue half with the flip on. The blue-half caption sits on the top edge. A second caption appears to overlap it at the top centre (seen at full size, not zoomed; magnitude unverified).
- `...\scratchpad\app2_viz.png`: Pedro tab renders.
- Note: the 3D view showed "Loading field model..." at a 10 s virtual-time budget (`app2_3d.png`). It rendered fully at a 20 s budget on a 1280x800 window. Under SwiftShader it is slow, so the first capture is not a bug.
- Console: no page errors in the three logs. Only Chrome updater and registry noise appeared, plus WebGL "GPU stall due to ReadPixels" performance messages.

Tests: `node --test --test-reporter=tap tests_js/*.test.mjs` from field_sim. The default spec/dots reporter printed no summary here, so the TAP reporter was used.
- Final lead re-run: 608 tests, 603 pass, 4 fail, 1 skipped. Own-area files (ui_ids, controls, autorunner): 58/58 pass. app.js passes node --check. Lead review of Haiku edits (resetAllState, HUD, kind selector, flip via pathing.js helpers): no defects found.
- Failing: `intake: balls touching / just ahead of the roller inside the mouth width are taken, front and rear` (test 458), and three e2e tests: `a solver shot ... scores and tips the hive` (547), `continuous auto fire every 0.15 s ... keeps hitting the opening` (548), `unlimited shots are recycled ... stay fast` (551). None of these imports app.js. They are in balls and shot/hive code owned by other leads.
- Earlier numbers in this file (532 tests, 27 fail) were stale. The current suite gives 587 pass, 4 fail.

## 3. Decisions

- Pollen tip count: 8. Source: user fact in this run ("a hive needs 8 POLLEN to tip"). The manual does not state it. The old ballsToTip=4 is wrong.
- Hold limit: 4. Source: user fact; docs/OVERNIGHT_RULES.md ("Official ball hold limit is 4"). Keep `holdLimit` at 4.
- Nectar tip threshold: UNKNOWN. No source was fetched that gives a nectar tip count. The hive-tip model (docs/overnight/hive-tip.md) uses a derived ASSUMED value: nectar weight 1.6545 (mass ratio 0.091 lb / 0.055 lb, andymark product page, search snippet only, not fetched), so 5 nectar alone. Mixed pieces use a linear weight sum (ASSUMED). The HUD labels the nectar count as an assumption.
- Auto planner crop and flip: the crop follows the alliance half, and the flip is on by default for blue. These are UI choices, not robot facts.

## 4. Not verified

- The flip and footprint orientation were checked only by eye at screenshot size. No pixel-level check of the start footprint against the start heading.
- Caption overlap at the top of the Auto planner is seen, not measured.
- The Clear-then-Refill path: `btn-clear` (app.js:543) still calls `world.reset([])`, which overwrites `_startBalls` (balls.js:239, `if (startBalls)` treats [] as truthy). The current guard is that `resetField` passes balls explicitly (app.js:548), so Refill and Reset should work. No test covers Clear followed by Refill or a run. Unverified in a browser.
- Shooting and tipping in the sim (the user's report that 1 hive does not tip) was not run in this pass. The failing e2e tests above say it is still broken for some paths.
- No Pedro, Java or robot constants were changed or invented.

## 5. Follow-ups for other owners

- pathui.js owner: `robotOutline` (pathui.js ~line 332) uses rotate(-h). In the flipped Auto view it needs rotate(Math.PI - h). The start is drawn twice (once in drawAuto, once by drawPlannerPoses). Remove the drawAuto start draw after the fix.
- balls.js owner: `BallWorld.reset([])` should not overwrite `_startBalls` on Clear. Add a test for Clear -> Refill -> run. Fix the intake test (458) and the e2e shot-to-tip tests (547, 548, 551) that fail.
- hivetip_model.js / hivecycle.js owner: the nectar threshold stays ASSUMED until a source is found. Keep the mixed-piece rule labelled as an assumption. Add a shot path for nectar to test the HUD count.
- app.js: the shot path still uses `world.addBall(kind, ...)`. Confirm the nectar mesh and tip count update on a real nectar shot (not verified in this pass).
