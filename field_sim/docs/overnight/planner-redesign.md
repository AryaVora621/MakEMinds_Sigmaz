# planner-redesign report

Goal: make the Auto planner look and behave like FRC PathPlanner (refs: Screenshot 2026-11-07 091143 / 091202 / 091057 in C:\Users\Jindal\Pictures\Screenshots).

## What changed
- web/js/plannerfield.js (new): flat field view, background #1b1b1f sampled from the reference screenshot, white border, tapes as solid lines, hive/flowers/balls as solid outlines or fills, no translucent boxes, no captions. Axes gizmo and cursor readout are a separate overlay in the free area (cannot clip or overlap).
- web/js/app.js (map section only): `autoAvail()` measures the free area from the DOM every frame (left sidebar, 340 px right panel, timeline, editor top bar, floating playback bar); the field fills it. Alliance half with "View from alliance wall" on is turned a quarter (canvas rotation, own wall at the bottom) so it fills a landscape area; full field stays square and centred. `drawPlannerView()`, `screenToField` un-rotates pointer input. `viewScale/rot90` helpers in pathing.js.
- web/js/pathui.js: PathPlanner path editor. Dotted trajectory (yellow slow to green fast, from the planned trajectory), white anchors, green first / red last, stroke-only start (green) and goal (red) robot outlines, orange rings for rotation targets, preview robot with corner dashes. Right panel: green `~3.9s` chip, undo/redo/mirror, collapsible cards Waypoints, Event Markers (+), Rotation Targets (+), Point Towards Zones (+, shown as not supported), Starting State, Final State, Global Constraints, Constraint Zones (+), Path Optimizer (shown as not available), Editor Settings, Export. Floating playback bar (play/pause, reset, slider, time, speed) via `ed.bindPlayback`. Advanced stuff (per-path list, path settings) sits in closed `<details>`. Open cards are remembered (localStorage).
- web/js/autoui.js: path editor is full screen (top bar: back, name, delete path, full-field eye; `#autopanel.editing` hides sidebar and timeline). Main screen: sidebar sections and right cards all collapsible and remembered; Inspector only when something is selected; Validation only when there are issues; Start state and Export closed by default; list hover actions cut to rename/copy/delete; big "Run in sim" button at the top of the right panel; timeline is one compact transport row (chevron expands lanes, remembered). Delete: trash on rows, Delete/Backspace key on a focused Autos or Paths row, undo strip for 8 s; deleting a path now really removes it (also the copy kept for followed paths and the ghost editor route).
- web/js/autos.js: `autoAim` command (shooter category, optional boolean `shootAfter`, validated, serialises, adapter call `autoAim(args)`); palette entry "Auto aim"; inspector checkbox.
- CSS: planner.css tokens and layout (cards #23232a, panel #292a33, rows #313f90, chip #c8e6c9, slider #bac3ff, all sampled from the reference), ui.css layout (340 px right panel, editing mode).
- Robot is never draggable on the Auto tab (unchanged gate, verified below).

## Evidence (screenshots, scratchpad C:\Users\Jindal\AppData\Local\Temp\claude\1bf884c3629a4194\cab1052d-b484-449d-8151-8eddbe3da6ad\scratchpad)
Best: final_edit_blue_1600.png (editor, blue half fills the area), final_edit_full_1600.png (full field), final_list_blue_1600.png (autos screen), final_list_red_1600.png, final_list_full_1600.png; 1280x720 variants final_*_1280.png; final2_*.png (delete, collapsed state, 3D run).
- Real mouse (puppeteer): 5 clicks append 5 waypoints at the intended coordinates in the rotated view, anchor drag works, right-click and clicks on the overview are swallowed, S.x/S.y/S.h unchanged (72, 24, 1.571) through all of it.
- Run in sim: real click on the big button switches to the 3D view, runner state `running`, no errors. Stepping the sim 700 frames: robot goes to the path start, follows it to (90, 107), runner `done` at t=2.53 s with Auto aim and Shoot in the sequence (headless SwiftShader renders ~1 fps so wall-clock motion is very slow; I stepped `__sim.step`). Auto aim does nothing at runtime yet (host adapter has no `autoAim`; `a[m]?.()` no-ops).
- Tests: `node --test tests_js/*.test.mjs`: 693 pass, 1 fail (ball-pushing physics test, not planner, another worker's area). New: tests_js/planner_redesign.test.mjs, additions in auto_list.test.mjs. Updated DOM-coupled tests (pathing, pathui, path_draw, pathplanner_edit, ui_ids).

## Does not match / not verified
- Hive and flower art is simplified, not a field image; the grey hive box is heavier than PathPlanner's stage.
- Alliance half is quarter-turned (own wall at the bottom), PathPlanner keeps wall left; chosen so the half fills the screen.
- Point Towards Zones and Path Optimizer are placeholders (no model, no export).
- Waypoints card icon looks like an eye at 18 px.
- Obstacle warnings ("passes through obstacle 7") appear in the editor for paths crossing the hive.
- Not exercised: touch input, Pedro sampler server (served by simple http.server, so JS fallback sampler), 1280x720 delete/collapse run.

## Follow-ups for other owners
- autorunner.js/app.js host adapter: implement `autoAim({shootAfter})` (RPM, turret, hood, transfer on). autoexport.js KNOWN_COMMANDS and ppformat need `autoAim` (export currently treats it as unknown).
- app.js: `drawMapInner` still has dead auto-view branches; diag `.dg-notes` is hidden on the Auto tab by css.
