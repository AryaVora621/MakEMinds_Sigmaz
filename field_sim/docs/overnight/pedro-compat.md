# pedro-compat report (lead, Sonnet)

## Answer
- Pedro Pathing: compatible. Exported Java uses only Pedro 2.1.0 calls that exist (PathBuilder.addPath, BezierLine/Curve, set*HeadingInterpolation, setReversed, Follower.getChainIndex/getCurrentTValue). No chain callbacks (markers are polled).
- Command groups: TeamCode has NO command library (grep of gradle + java for nextftc/ftclib/ivy/CommandGroup: no hits; autos are LinearOpMode + hand-written `while` loops over `robot.auto.followPath/isBusyOrTimeout`; source: h/pedro-compat/t01.md, read only).
  So the default export style ("statemachine") emits a per-tick state machine for sequential / parallel / race / deadline plus wait, waitUntil, timed commands and path markers. It needs no dependency.
  Style "ivy" (Pedro's own command lib, https://pedropathing.com/docs/ivy , /command-compositions , /pedro-commands) is optional: group names and follow/instant confirmed by Haiku t04 from those pages, waitSeconds/waitUntil/Scheduler names UNVERIFIED, Ivy is not a project dependency.

## Changed (web/js/autoexport.js, tests_js/pedrofollow_compat.test.mjs)
- Marker fix: segmentWeights counts a 'line' segment by endpoints only (it is emitted as BezierLine(first,last)); before, a bent line mis-placed markers.
- Marker fix: t=1 with zero total weight mapped to the START of the last segment; now the end. Zero-segment path: marker dropped (before it fired on the first poll, seg -1).
- locateMarker/segmentWeights now exported (were untested).
- Stub collisions: custom command / condition names that sanitize to the same Java name (a-b, a_b) emitted a duplicate method (compile error); stub key now uses the sanitized name.
- New exportWarnings(auto), also written into the Java header as an EXPORT WARNINGS block: concurrent-follow (two follows live in one parallel/race/deadline: ONE follower, last followPath wins, cancel holds pose), no-timeout waitUntil, unknown-condition stub, bad-deadline, timed path marker (exported as TODO).
- shoot/shootAll default durations (1.0 / 3.0 s) are now commented ASSUMED.
- Haiku tests kept: h_pedro-compat_t12..t18 (all pass; t17 has 4 todo documenting .pp format gaps: no groups/commands in .pp round trip).

## Evidence (ran)
- node --test tests_js/pedrofollow_compat.test.mjs tests_js/autoexport*.test.mjs tests_js/h_pedro-compat_*.test.mjs: 86 tests, 82 pass, 0 fail, 4 todo.
- Full node --test tests_js/*.test.mjs: 968 tests, 949 pass, 2 fail, rest skip/todo. Failures are not mine: h_shoot-physics_t12 (file-level) and h_teamcode-java_t16 "timeout parity" (JS AUTOAIM_DEFAULTS.timeoutSec=4 vs Java TIMEOUT_S=3; owner of autoaim_cmd.js / AutoAimTunables). Earlier run also showed a hold-limit literal test and hood/tunnel failures from other owners; they varied between runs (files in flux).
- javac -Xlint:all (Android Studio jbr javac 25) of a generated sample (sequential/parallel/race with markers, bent line, curve, waitUntil, a-b/a_b commands) against real core-2.1.0.jar plus signature stubs (scratchpad/pc): exit 0, no warnings.

## Unverified
- Stubs are signature-only: a mismatch with real TeamCode Robot is not caught by javac here (t02 matched every symbol by reading source, not compiling). gradlew not run (rule).
- Generated state machine has never run on a robot or in the sim; Ivy names; Pedro getChainIndex/getCurrentTValue semantics at runtime; .pp format round trip of groups (gap).
- Sim vs robot gaps from t09 (not edited, pedro_bridge/PedroFollowSim untouched): robot uses ProfiledVectorCalculator/SCurve, sim uses stock; sim fires 'callback' events that never fire on the robot; 'js-fallback' trajectories are approximate and should be flagged in the Pedro tab.

## Patches for other owners
- web/js/autoui.js:889 downloads `<cls>.java` using a different sanitizer than autoexport.sanitizeClassName (name "1st Auto" -> file 1stAuto.java, class Auto_1stAuto: public class/file mismatch = compile error). Use `sanitizeClassName(name)` for the filename.
- web/js/autos.js buildCommandTree: deadline index out of range is not clamped (Java clamps to 0, validator errors); autoAim reads args.timeout in sim vs args.seconds in Java.
- Sim runs marker node types race/deadline/waitUntil that Java emits as TODO; validator should block them in markers.
- Pedro tab / planner validator: surface exportWarnings() (concurrent-follow especially) before export.
