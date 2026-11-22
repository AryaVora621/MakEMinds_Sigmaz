# planner-ui report (BRIEF 1)

Full suite: `node --test tests_js/*.test.mjs` = 478 pass, 0 fail (includes autoexport/ppformat tests). New tests: `tests_js/pathplanner_edit.test.mjs` (9 tests).

## Earlier partial run
All files were untracked so git could not show a diff; I read pathui.js/autoui.js/pathing.js/planner.css whole before editing. They were coherent (no half-written code); I edited in place.

## What changed
- `web/js/pathing.js`: `addWaypoint` (connected cubic, handles at 1/3 and 2/3), `nearestOnRoute`, `insertWaypointAt`, `moveAnchor` (anchor carries its Bezier handles), `viewBounds/clampToView/inView` (alliance half), `PLAN_VIEW` gained `alliance, fullField, openPath, bounds`. Java export (`toJava`) no longer emits `addParametricCallback`: markers are polled in the follow state (`private boolean evN_K;` + `follower.getCurrentTValue() >= t`), same pattern as autoexport.js (coordinator request).
- `web/js/pathui.js`: PathPlanner editing + painting, see below. `web/js/autoui.js`: publishes alliance and `openPath`, "Full field" eye button in the auto header. `web/css/planner.css`: PathPlanner M3 dark tokens (literal, no longer follow ui.css).
- `tests_js/pathing.test.mjs`: default tool now appends a cubic (Shift = straight); Java export test now expects polling.

## Editing model (done)
1. Robot not draggable: `ed.editing` is now an accessor that reads true on the whole Auto tab, so app.js's existing gate (`pathEditor.editing`) routes pointer/key events to the editor there; every click is swallowed (empty click, right-click), so `placeRobot` and "right click faces point" never run. Ctrl-click still copies a Pose. Verified with real mouse events: robot S.x/S.y unchanged after append/insert/drag/delete and overview clicks.
2. Waypoints: click empty field appends a connected waypoint (cubic with two handles, Shift = straight, tools: Select/Curve/Line/Delete); click on a drawn path inserts a waypoint (de Casteljau split); drag anchor/handle (anchor carries its handles); drag the rotation dot (15 deg snap, Alt free); right-click or Delete removes (hovered point, else selected point, else selected path). Pan tool removed (it did nothing and let clicks fall through to robot placement).
3. Auto tab with no path open: display only; clicking a drawn path opens it for editing (`PLAN_VIEW.openPath`).
4. Robot footprint drawn only at start (green, alpha 125), goal (red, alpha 125), selected anchor (orange) and the preview pose. The host's own live-robot drawing is separate (see follow-up 1).
5. Data model unchanged (route {start, segments[{type, points, heading, callbacks, label,...}]}); autoexport/ppformat tests green.

## Alliance-limited view
- Geometry: red half x 0..72, blue half x 72..144, full y. Source: docs/FIELD_CONSTANTS.md (red at low x, hives x 59.25/84.75, "alliance areas" red x<0, blue x>144 outside the walls, loading zones red x 1.9-12.9, blue x 131.1-142.1). docs/overnight/research_manual.md confirms only "start on own alliance side (G304)"; the exact split line at x = 72 is a repo assumption, not a quoted manual number; start coordinates (Fig 11-1) unverified.
- Blue/Red toggle sets `PLAN_VIEW.alliance`; the other half is dimmed (alpha 0.9) with a dashed cut line and label; edits clamp into the half and clicks outside toast "Outside the X half"; Eye button (Full field) lifts it. Mirror unchanged (x -> 144 - x).
- NOT done: a true crop/zoom, and flipping the orientation (blue half rotated 180 so own wall is always left). Those need MAP changes in app.js (follow-up 2). I read "flips" as mirror, which already works.

## PathPlanner colours (source: raw.githubusercontent.com/mjansen4857/pathplanner/main/lib/...)
- path_painter.dart (https://github.com/mjansen4857/pathplanner/blob/main/lib/widgets/editor/path_painter.dart): path = colorScheme.secondary, hovered/selected = Colors.orange; anchors: first Colors.green, last Colors.red, others colorScheme.secondary, selected Colors.orange, hovered Colors.deepPurpleAccent, ring colorScheme.surfaceContainer, width 2; control lines Colors.grey[700]; start/goal robot outline green/red withAlpha(125); break warning Colors.yellow[800]; grid colorScheme.secondary withAlpha(50), 0.5 m; anchor radius 25 and control 20 "ui points" of a 3240 px image (`uiPointSizeToPixels`, path_painter_util.dart).
- lib/main.dart: `ThemeData(useMaterial3: true, colorSchemeSeed: teamColor, brightness: dark)`; lib/util/prefs.dart `Defaults.teamColor = 0xFF3F51B5`.
- Hex used: green #4caf50, red #f44336, orange #ff9800, deepPurpleAccent #7c4dff, grey700 #616161, yellow800 #f9a825 (Flutter Material constants, recalled, not read in the repo). Derived from seed #3F51B5 with @material/material-color-utilities 0.3.0 (SchemeTonalSpot, dark): primary #bac3ff, onPrimary #222c61, secondary #c3c5dd, surface #121318, surfaceContainerLow #1b1b21, surfaceContainer #1f1f25, surfaceContainerHigh #29292f, surfaceContainerHighest #34343a, outlineVariant #46464f, outline #90909a, onSurface #e4e1e9, onSurfaceVariant #c7c5d0. Unverified: that Flutter's current `colorSchemeSeed` matches this library version exactly.

## Evidence
- Screenshots (headless Chrome, SwiftShader, scratchpad): `auto_edit_blue_1600.png`, `auto_overview_1600.png`, `auto_red_1600.png`, and 1280x720 variants `auto_edit_blue_1280.png`, `auto_overview_1280.png`, `auto_red_1280.png` (scratchpad dir ...\scratchpad). I viewed 1600 edit/overview and 1280 red. Panels fit at 1280x720.
- Puppeteer real mouse events (page.mouse) on the Auto tab: 3 clicks -> 3 segments; click on path -> 4; right-click anchor -> 3; click + Delete -> back to 3; bounds blue {72..144}, red {0..72}, full {0..144}; robot pose unchanged throughout.

## Not verified
- Run in sim / onPreviewPath hooks untouched but not exercised end to end here. Touch behaviour. The side panel text colours were not re-checked beyond the screenshots. The JS-fallback badge text overlaps nothing now but the half label can sit under host-drawn balls.

## Follow-ups for other owners (app.js, integrator)
1. In `drawMap` skip the host robot (`isOn('robot')` block), trail, and S.path overlay when `S.view === 'auto'` (the editor draws footprints); otherwise the old robot box stays visible at S.x/S.y.
2. True crop: in the fit code (`MAP.k = Math.min(availW, availH) / 144; MAP.ox/oy ...`) when `S.view==='auto'` use `const b = PLAN_VIEW.bounds` (import from pathing.js, refreshed by `pathEditor.bounds()` each draw): `MAP.k = Math.min(availW/(b.x1-b.x0), availH/(b.y1-b.y0)); MAP.ox = left + pad + (availW-(b.x1-b.x0)*MAP.k)/2 - b.x0*MAP.k; MAP.oy = top + (availH-(b.y1-b.y0)*MAP.k)/2 + b.y1*MAP.k`. The field background/tile loops in drawMap should clip to the bounds. Optional flip: rotate by 180 for blue.
3. app.js still has `S.view === 'path'` branches and `$('pathpanel')`; harmless (tab is gone), `pathEditor.active` is never true now.
4. `getFootprint` is not passed to createPathEditor; footprint defaults to 18x18 in. Pass the MaxBot frame (robotspec) when known.
