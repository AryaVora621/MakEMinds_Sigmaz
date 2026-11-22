# planner-gaps: step 3 integration (Sonnet lead)

## Changed files (all owned)
- web/js/pathui.js: removed the placeholder cards "Point Towards Zones" and "Path Optimizer" (honest removal: no export support, no optimizer; t04/t05 verdicts, confirmed in code). Waypoints icon is now a new `waypoints` polyline icon (was the teardrop `pin`). Event-marker delete from the events list and from right-click/Delete tool now respect path lock (t18 bug 5, verified in source).
- web/js/pathing.js: addCurve clamps its auto control point to 0..144 (fixes fromJSON throwing "outside the field", t19 repro). validateRoute samples obstacles every 0.25 in (was 1 in; corner clips under 1 in were missed, t03). Warning text now names the collider kind when no name exists ("obstacle hive frame 3").
- web/js/autos.js: COMMAND_DEFS.autoAim declares `track` (bool, false) and `timeout` (number, default 4, 0.5..10). Compile already forwarded all args (t09 test confirms).
- tests_js: h_planner-gaps_t09 todo flipped to a hard assert; h_planner-gaps_t17 DECORATIVE list gets `waypoints`; planner_redesign.test.mjs and auto_list.test.mjs updated for removed cards and the 3 autoAim args (+ assertion that placeholder cards are gone).

## Evidence (ran)
- `node --test tests_js/*.test.mjs` (from field_sim): tests 978, pass 960, fail 0, todo 15.
- Before edits the Haiku t19 file had 1 failing round-trip test marked KNOWN-BUG todo; still todo (see below).
- Not run in this step: headless Chrome screenshots, pointer/touch synthetic events at 1280x720 (Haiku t11/t12/t13 did; their findings were read, not re-run by me).

## Haiku findings judged
- Accepted and applied: t03, t04, t05, t06 (waypoints icon), t09, t18 bug 5, t19 curve clamp.
- Rejected or deferred: t01 footprint check (needs collision.js export `isFree`, not my file); t18 bugs 1-4 (autoui.js double-delete, deleteAuto resurrecting via autosave, undo losing markers): plausible but I did not reproduce them, left unchanged; t07 trajectory colour unification was handled earlier, not re-verified here.

## Unverified
- t19 round trip still todo: movePoint/moveSegment can place points off-field in random ops (x=-3.28). The UI snaps/clamps, but the model does not. I did not change movePoint (could alter drag behaviour).
- autoAim timeout default 4 (sim, AUTOAIM_DEFAULTS) vs 3.0 in AutoAimTunables.java: unresolved, sim value used, Java export truth ASSUMED to need a decision.
- 0.25 in sampling cost on long routes not benchmarked (500-op tests stayed fast).

## Patches for other owners
- collision.js / app.js:651: export `isFree` and pass `buildObstacles(fieldData)` to the editor so validateRoute can test the 18x18 robot footprint, not the centre point (t01 B/C). Add `name: kind + index` to boxes.
- app.js:588-590: skip pathEditor.key while the auto planner owns keys (t18 bug 1).
- autoexport.js: `conditionExpr` lacks timeElapsed/hopperFull/hopperEmpty; parallel follow nodes share one follower; single-point segments export a zero-length BezierLine; rotation targets are not exported (t10, t04). Unverified by me.
- plannerfield.js:104 overlay halo uses translucent colour contradicting its header (t15): not applied.
- app.js: no change needed for my edits.
