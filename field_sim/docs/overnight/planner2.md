# Planner2: planner summary, decisions, helpers and gaps

Scope: web/js/pathui.js, web/js/autoui.js, web/js/pathing.js, web/js/autos.js, web/css/planner.css, tests_js/path*.test.mjs, tests_js/auto*.test.mjs. No git add/commit/stash/checkout/reset was run.

## 1. Research (R1): verified colours and behaviours

Source: team research digest, PathPlanner GitHub raw files (`https://raw.githubusercontent.com/mjansen4857/pathplanner/main/lib/...` returned 404 for the `pathplanner/lib/` prefix; the working paths have no `pathplanner/` prefix, so the file is `lib/widgets/editor/path_painter.dart`). RE-VERIFIED in the integration pass by fetching https://raw.githubusercontent.com/mjansen4857/pathplanner/main/lib/widgets/editor/path_painter.dart (colour roles match; unselected handle lines grey[700], grid secondary alpha 50, break warning yellow[800]). Line numbers not re-checked.

- Path, waypoint and control-point default: `colorScheme.secondary` (path_painter.dart lines 115, 151, 754, 785, 827). Grid uses it at 50% alpha.
- Hovered path: `Colors.orange` (114, 150). Optimized path: `Colors.deepPurpleAccent` (160).
- Choreo start anchor green (153), end anchor red (155). First and last waypoint anchors green and red (750, 752).
- Selected waypoint, anchor and control points: `Colors.orange` (722, 758, 781, 802). Hovered: `Colors.deepPurpleAccent` (724, 760, 783, 804).
- Selected waypoint radius circle: `surfaceContainerHighest` (703).
- Rotation target: default `Colors.grey[700]` (591), selected orange (594), hovered deep purple (597). Start and end rotation outlines green and red at alpha 125 (620, 632).
- Event marker: default grey 700 (520), selected orange (523), hovered deep purple (525). Marker stroke `surfaceContainer` (521).
- Constraint zone and point zone: orange when selected (358, 552, 463), deep purple when hovered (383, 570, 491).
- Break warning: `Colors.yellow[800]` (641).

Status: THEME in web/js/pathui.js (l.35-41) matches these roles. planner.css not audited. Default constraint-zone colour (grey[700]) is an assumption.

## 2. Decisions and assumptions (recorded, not checked against the manual)

- Field: 144 x 144 in, Pedro frame. Origin corner is disputed: pathing.js:1 says bottom-left, docs/ARCHITECTURE.md:80 says audience-left corner. Left as-is; unverified.
- Red owns low x (x < 72), blue owns high x (x > 72). Source: docs/FIELD_CONSTANTS.md:5 and docs/ARCHITECTURE.md:83. Hive pivots at x 59.267 (red) and 84.767 (blue), midline 72.0 (FIELD_CONSTANTS.md:25). Manual only says "start on own alliance side" (G304, docs/overnight/research_manual.md:22).
- Default start per alliance (ASSUMPTION, pathing.js:648, unverified against manual Fig 11-1):
  - red: `{x:24, y:24, h:PI/2}`
  - blue: `{x:FIELD-24, y:24, h:PI/2}` = `{x:120, y:24, h:PI/2}`
  - Blue is the mirror of red through x = 72. Heading PI/2 maps to PI/2, so it is unchanged.
- Robot is 18 in square (R102). Start at 24 in from the wall clears the garden (y 1.9-3.9) and the loading zone (y 95.9+) for red. Blue is the mirror.
- View bounds: `viewBounds(alliance)` returns `{x0,x1,y0,y1,full,alliance}`. Red is x 0..72, blue is x 72..144, full field when `full` is true or the alliance is neither red nor blue.
- Flip modes (pathing.js:677-683): `'mirror'` is x -> 144-x, heading -> wrapAngle(PI-h). `'rotate'` is x -> 144-x and y -> 144-y, heading -> wrapAngle(h+PI). Any other mode returns a copy.
- `mirrorRoute` (pathing.js:167) is the alliance flip used by the Mirror button. It mirrors segments and start.
- `allianceViewTransform(alliance)` returns `{rotate180: alliance==='blue', flipX: false}`. ASSUMPTION (research 3, no verified source): blue is drawn rotated 180 so its own wall is on the left.
- Zone and rotation data shapes: zones and rotation targets live on segments in the route JSON (as before this pass). This pass did not change their shape. Verify against web/js/pathing.js before relying on it.
- `PLAN_VIEW.alliance` (pathing.js:614) starts as "blue", stays equal to `route.alliance` after setAlliance, mirror, undo and redo, and is overwritten on Auto-tab publish.
- Existing test pins: `tests_js/pathplanner_edit.test.mjs:33` pins the red/blue split at x=72.

## 3. What each earlier edit task changed

Haiku edit task A (alliance and per-alliance starts):
- pathui.js: initial route uses `P.defaultStart(P.PLAN_VIEW.alliance)` instead of the literal `{x:24,y:24,h:PI/2}`.
- pathui.js: `ed.setAlliance(a)` calls `retargetStart` then `commit()` (one undo entry; no-op for same or invalid alliance).
- pathui.js: `ed.mirror` calls `mirrorRoute`, then `retargetStart` on the result, then replaceRoute (one undo entry).
- pathui.js: `ed.clear` retargets the new route so the start stays in view.
- pathui.js: `syncAlliance()` keeps `PLAN_VIEW.alliance` equal to `route.alliance`.
- autoui.js: `setAlliance(a)` (Auto tab Red/Blue) calls `retargetStart(auto, a)`; `retargetStart` added to the pathing import.
- tests: `tests_js/path_alliance_ui.test.mjs` (new). `tests_js/pathui.test.mjs` and `tests_js/pathing.test.mjs` had start pins changed from (24,24,PI/2) to the new behaviour.

Haiku edit task B (keyboard and undo):
- pathui.js: exported `handleKey(ed, {key, ctrlKey, metaKey, shiftKey, altKey, targetTag})`. `ed.key(e)` delegates to it. INPUT, SELECT and TEXTAREA targets are ignored.
- Delete/Backspace order: rotation target, event marker, selected waypoint (hovered point only when none selected), then selected path. Start point is refused with a toast.
- Rotation targets are selectable by clicking (`ed.rotSel`, orange). Escape clears path, waypoint, event and rotation selections.
- `switchRoute` no longer pushes an undo entry; history restarts at the new path.
- tests: `tests_js/path_keys.test.mjs` (new, 9 tests).

Haiku edit task C (snapping and grid):
- pathui.js: `export function gridLines(bounds, step)` returns `{vertical, horizontal}`. Step 24 over 0..144 gives 7 lines per axis.
- pathui.js: grid overlay under paths when `ed.grid > 0` and `ed.showGrid` (new, default true). Panel checkbox `data-f="showgrid"`. Skipped above 300 lines.
- pathui.js: heading drag snaps at `hdgSnap` step, or 15 deg when Shift is held. Alt is free. With Snap "free", angle is free unless Shift.
- Waypoint, control and start drags already use `snapXY` (grid, magnet, Alt). Confirmed by tests.
- tests: `tests_js/path_snap.test.mjs` (new, 16 tests).

## 4. Helpers app.js owner should call

From pathing.js (pure, exported):
- `defaultStart(alliance)`: start pose for a new route of that alliance.
- `retargetStart(route, alliance)`: sets alliance, mirrors start into that half if outside it. Idempotent. Mutates and returns route.
- `flipPoint(x, y, mode)` and `flipHeading(h, mode)`: mode `'mirror'` or `'rotate'`.
- `allianceViewTransform(alliance)`: `{rotate180, flipX}`. app.js already uses it (app.js:771-772).
- `toScreen(view, x, y, w, h)` and `fromScreen(view, px, py, w, h)`: exact inverses for the same view and canvas size. app.js uses them at app.js:781-785.
- `viewBounds`, `clampToView`, `inView`, `startInView`: view crop helpers.

From pathui.js:
- `handleKey(ed, e)`: keyboard handler for the planner. app.js should call `ed.key(e)` or `handleKey` directly; it returns true when it consumed the key.
- `gridLines(bounds, step)`: grid geometry.

From autoui.js: `setAlliance(a)`, `mirror()`, `newAuto()`, `handleKey(e)`, `focus()`, `destroy()`.

## 5. Evidence (test counts)

- Full suite: `node --test tests_js/*.test.mjs` (shell glob expansion, from field_sim). Final integration run: tests 650, pass 649, fail 0, 1 skipped (earlier run had shootend2end hive-tip failures from other owners, since gone).
- The 3 failures are in tests_js/shootend2end.test.mjs ("a solver shot from the real turret/hood/rpm scores and tips the hive", "continuous auto fire every 0.15 s ... keeps hitting the opening", "unlimited shots are recycled ... stay fast"). These are ball/shot tests outside the planner area. This pass did not touch them. Not baseline-checked (git stash is forbidden).
- Earlier passes reported 629/625 and 525/499 totals; those numbers were at different points in the work and are superseded by the 644/640/3 run above.

## 6. Remaining gaps and follow-ups

- DONE (integration pass): `newRoute` and `newAuto` now default start x to 24 (red) or 120 (blue), h stays 0.
- Default start poses (24 in and 120 in, heading PI/2) are assumptions. Check against manual Fig 11-1 (research_manual.md:24, unverified).
- Visual check (headless Chrome screenshot) of the grid, handles and alliance flip not done in this pass.
- Blue view rotate180 is an assumption (no source).
- Origin corner (bottom-left vs audience-left) is unresolved between pathing.js:1 and ARCHITECTURE.md:80.
- E3/E5-E8 task IDs were not defined anywhere in this doc set, so the keyboard coverage claim is limited to the panel actions callable headlessly.
