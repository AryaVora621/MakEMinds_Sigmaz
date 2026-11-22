# auto-run: Run-in-sim in 3D + autoAim runtime (Sonnet lead review)

## Changes (files)
- NEW web/js/autoaim_cmd.js: pure command (start/update/done/stop/status). Ready = flywheel within 4 %, turret within 2 deg, hood within 1.5 deg, held 0.1 s; timeout 4 s default; never throws; optional track, shootAfter (fires held balls, ends on empty hopper or 6 s), fireOnTimeout.
- web/js/autorunner.js: `autoAim` adapter command (+ `track`, `keepAim`), soft failures via `runner.warnings` (a failed/timed-out aim warns and the auto continues), transfer off on stop/end/reset. Fixed multi-path autos: paths are now sampled one at a time (the shared Pedro sampler aborted all but the last path, so any 2+ path auto never started).
- web/js/app.js (auto-host section only): host `transfer`, `autoAimTarget`, `autoAimCurrent`, `autoAimApply`; `aimed` also checks hood; `prepareAutoRun` resets transfer; Stop re-applies the auto start pose (was landing at the default 72,24); run panel stats now show "transfer on".
- tools/verify_autorun.mjs: Restart check now steps the sim deterministically (`__sim.step`) instead of waiting wall time.
- Tests: tests_js/autoaim_cmd.test.mjs (14), autoaim_runner.test.mjs (6), autoaim_headless.test.mjs (gated, AUTOAIM_HEADLESS=1).

## Root cause of "cannot run autos in 3D" (reviewed)
1. Multi-path autos never started (sampler abort) -> fixed in autorunner.js.
2. Stop moved the robot to the default pose -> fixed in app.js.
3. UI: the 3D run panel (`#autorun`) has no Run button and can be closed with no way back; Run buttons exist only in the Auto tab (planner-owned, see follow-ups).
4. The earlier "restart_moves" failure was the verify harness (SwiftShader ~0.5 fps, dt clamp 0.05 s), not a sim bug.

## Evidence
- `node --test tests_js/*.test.mjs`: 752 tests, 749 pass, 0 fail, 3 skipped (gated headless).
- `node tools/verify_autorun.mjs` (own server, 8841+, headless Chrome SwiftShader): exit 0, failed_assertions [] : real click on Run in sim switches to 3D, robot moves, Stop returns to (24,24,0) idle, Restart runs and moves again. Screenshot in scratchpad verify_autorun_3d.png.
- `AUTOAIM_HEADLESS=1 node --test tests_js/autoaim_headless.test.mjs`: 2/2 pass. Auto [follow A, autoAim{shootAfter}, follow B]: done at 4.07 s runner time, shots 0->4, scored 0->4, ammo 4->0, fired at rpm ~2120, turret -33.8, hood 65, final pose (60,100.5). Forced infeasible aim: warning "autoAim: timeout after 4s (infeasible shot)", no shot, follow B still completes.

## Unverified / known gaps
- Transfer is recorded and logged only. Shots are gated by `S.eval.allowed`, not by transfer (gating would break existing shoot-only autos).
- `autoAimApply` only forces auto-aim mode; the existing frame loop drives flywheel/turret/hood toward `S_AIM.lastCmd` (physics files untouched). So tracking while driving is effectively always on in auto mode.
- Tolerances are sim-only: robot Turret 1.5 deg, Flywheel 50 ticks/s, Hood 1.5 (placeholder) differ from sim 2 deg / 4 %. No decision made with robot owners.
- A non-track autoAim leaves transfer on until the auto ends (no transfer-off node in the palette).
- Run-in-sim from the 3D view with the Auto tab never opened, and Pause/Resume, were not exercised.

## Follow-ups for planner-redesign (autos.js, index.html; not edited here)
- autos.js:442 currently `act(call('autoAim', args), ...)`; optional patch to pass timeout and track:
  `act(call('autoAim', { shootAfter: !!args.shootAfter, timeout: args.timeout ?? args.seconds, track: !!args.track }), nm, 0, args)`. The palette (autos.js:41) has no timeout/track args, so the 4 s default applies.
- Add a Run button to `#autorun` in index.html wired to `autoPlanner.run()` (same entry as Restart, app.js `ar-restart`), and a way to reopen the panel in 3D after closing it (AR.show is only set true when a run starts).
- Offer a "transfer off" command if desired.
