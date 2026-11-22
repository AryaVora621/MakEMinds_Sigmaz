# export2: autoAim export, ppformat and scheduler

## What changed (per file)
- `web/js/autoexport.js`: `normAutoAim()` maps `{type:'autoAim', shootAfter, seconds}` to a command node with args. `walk()` has an `autoAim` branch before the `TIMED` branch that emits a timed state (`runN`/`cancelN`, `nN_logic`, `nN_feedT0` when shootAfter). Imports `AutoAimLogic` and `AutoAimTunables` only when the tree has an autoAim node and style is not ivy. Marker autoAim gives a TODO, like shoot. Constants `AUTOAIM_PACKAGE`, `AUTOAIM_LOGIC_CLASS`, `AUTOAIM_TUNABLES_CLASS` at line ~121.
- `web/js/ppformat.js`: `sanitizeAutoAimArgs()` (line 17) keeps only `shootAfter`, as a strict boolean. Other callbacks clone args as before.
- `web/js/scheduler.js`: `AUTOAIM_ASSUMPTIONS` (line 71), `estimateAutoAimSeconds()` (line 85), `autoAimCmd()` (line 103). A generic leaf command. It calls optional hooks `ctx.setAutoAim` and `ctx.feed`.
- `web/js/pedrofollow.js`, `pedro_bridge.py`, `tools_java/*`: no change. None dispatches on command names; callback names pass through as strings (`pedrofollow.js:29-32`, `PedroFollowSim.java:248/264/278`).
- Tests present and passing: `tests_js/autoexport_autoaim.test.mjs`, `ppformat_autoaim.test.mjs`, `scheduler_autoaim.test.mjs`.
- Note: this pass re-ran the tests and checked constants; it did not re-read the full JS diff.

## AutoAim name, package, verification
- Command name `autoAim`. Package `commands.autonomous`.
- Verified present in TeamCode: `TeamCode/src/main/java/commands/autonomous/AutoAim.java`, `AutoAimLogic.java`, `AutoAimTunables.java`. Generated code uses `AutoAimLogic` (per tick) and `AutoAimTunables`. The javac check below had zero errors in these files, using the Pedro jar and minimal stubs.
- `AutoAimCommand` (named in the brief): unverified. No such TeamCode class exists. The generated marker path defines a private inner class of that name (`autoexport.js:188`).
- Deviations from the brief: (1) `AutoAim.run()` blocks, so the export uses the non-blocking `AutoAimLogic.update(nowS, shooterAvailable, aimReady)` per loop. (2) Default timeout is `AutoAimTunables.TIMEOUT_S` (3.0). (3) Shoot-after feed length is `SHOOT_AFTER_MAX_S` (5.0), not the brief's 1.0 s. (4) Transfer follows `AutoAimLogic`'s `transferOn` output, not a separate call.

## Assumptions (all unverified against hardware)
- `scheduler.js` `AUTOAIM_ASSUMPTIONS`: `turretRateDegS` 180, `flywheelTauS` 0.35, `hoodRateDegS` 120, `settleFrac` 0.03. Each is used only when the caller's spec lacks the key.
- Conflict: `web/data/robot_spec.json:63` gives turret rate 487 deg/s (vault source). The 180 fallback is inconsistent with it. Pass the spec value.
- Estimate clamped to [0.15, 3.0] s.
- `scheduler.js` `autoAimCmd` `shootSeconds` default 1.0, vs TeamCode `SHOOT_AFTER_MAX_S` 5.0. Pick one (see follow-ups).
- `AutoAimTunables`: `TIMEOUT_S` 3.0, `TRANSFER_ON_START` true, `SHOOT_AFTER_MAX_S` 5.0. All marked PLACEHOLDER / TODO(tune on robot).

## Evidence
Command (run this pass, from `field_sim`):
`node --test tests_js/autoexport.test.mjs tests_js/autoexport_autoaim.test.mjs tests_js/ppformat.test.mjs tests_js/ppformat_autoaim.test.mjs tests_js/scheduler.test.mjs tests_js/scheduler_autoaim.test.mjs tests_js/pedrofollow.test.mjs`
- Combined: tests 95, pass 95, fail 0, skipped 0.
- Per file: autoexport 32/32; autoexport_autoaim 11/11; ppformat 6/6; ppformat_autoaim 8/8; scheduler 20/20; scheduler_autoaim 10/10; pedrofollow 8/8.
- Full suite `node --test tests_js/*.test.mjs`: tests 741, pass 740, fail 0, skipped 1.

## Not verified
- Full Java compile with the real FTC SDK (see javac section). The FTC SDK binary jars are not on this machine.
- Byte-identical shoot output against a pre-change snapshot. There is no snapshot (no git stash or checkout allowed).
- `AutoAimLogic` timeout and `done` semantics: read from source, not run on a robot.
- Sim-side estimate against the real robot. Turret rate and flywheel tau are placeholders or assumptions.

## Needs from other owners
- planner-redesign / `autos.js` (`COMMAND_DEFS`, `commandDuration`, `buildCommandTree`): should call `scheduler.js` `autoAimCmd` and `estimateAutoAimSeconds`. Per research 6, `autos.js:442` already dispatches `autoAim`.
- `pathing.js` / `autoui.js`: store `shootAfter` as a boolean. `pathing.js` `EVENT_PRESETS` (line ~369) has no `autoAim` entry, so a path callback named autoAim is ignored by export (research 5).
- Decide the shoot-after feed length: 1.0 s (scheduler) vs 5.0 s (`SHOOT_AFTER_MAX_S`).
- TeamCode owner: `AutoAimLogic` has no cancel. The generated `cancelN()` only stops the intake, so the flywheel keeps spinning if autoAim is cancelled mid-aim. Confirm this is intended.
- TeamCode owner: `scheduler.js` fallback turret rate (180) should match `robot_spec.json` (487), or the spec should always be passed.
- Re-run the javac check with the real FTC SDK jars on `-cp`.

## javac check (sample export vs TeamCode)
Scratch: `C:\Users\Jindal\AppData\Local\Temp\claude\1bf884c3629a4194\cab1052d-b484-449d-8151-8eddbe3da6ad\scratchpad\export2` (nothing written to TeamCode). Generator: `gen-sample.mjs`. Output: `gen/commands/autonomous/AutoAimSampleAuto.java`. `checkJavaStructure` reports ok.
- Compiler: `C:\Program Files\Java\jdk-17\bin\javac.exe` 17.0.12. Classpath: Pedro `core-2.1.0.jar` from the gradle cache. FTC SDK binary jars are absent (only `-sources.jar` cached).
- Run 1 (no stubs): exit 1, 235 errors. Generated file: 14 errors, all missing SDK (`com.qualcomm.robotcore.eventloop.opmode`, `Autonomous`, `LinearOpMode`, `ElapsedTime`, `hardwareMap`, `telemetry`). No errors in `robot.*` members or in `AutoAim*`. Full text: `javac-stderr.txt`.
- Run 2 (minimal signature stubs in `scratchpad/export2/stubs`, no robot values): 158 errors. Zero in `AutoAimSampleAuto.java` and zero in `AutoAim*.java`. Remaining errors are SDK-dependent TeamCode files. Some `Telemetry.addData` errors come from the stub signature. Full text: `javac-stderr-withstubs.txt`.
- Verdict: the generated sample compiles against its own code. Full compile needs the real FTC SDK 11.0.0 jars.

## Final full-suite run
- Command: `node --test tests_js/*.test.mjs` (from `field_sim`, Node 24). Result: tests 741, pass 740, fail 0, skipped 1, cancelled 0.
- Failures in other owners' files: none. Nothing was edited in this pass, so no owned-file fixes were needed.
- The one skipped test is a pre-existing skip; it is not a failure.

### Failures in other owners files
- None.

## Lead review pass (integration)
- Re-read the autoAim code in autoexport.js (marker stub, timed state), scheduler.js (estimate, autoAimCmd) and ppformat.js (sanitize). No defects found; no code edits this pass.
- Recheck of TeamCode: `AutoAimCommand` still does not exist; real classes are AutoAim/AutoAimLogic/AutoAimTunables in `commands.autonomous`. Generated code uses those. The only `AutoAimCommand` is a generated inner class.
- Full suite at this time: 743 tests, 740 pass, 2 fail. Failures are in other owners' areas (sim e2e "8 solver shots ... tip the hive once"; "recycling a full cell wakes free balls"), not autoAim/export. Likely concurrent physics edits.
