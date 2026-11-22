# physics3: simulator physics research, audit, fixes (2026-11-07 overnight, Sonnet lead integration)

Owned files touched: web/js/balls.js, tests_js/sim_physics.test.mjs (new), tests_js/hive_tip_dynamics.test.mjs (new). No git commands run. Full prior detail (constants table, per-audit notes) was condensed from 219 lines to this file; no values changed.

## Changes
- balls.js: quadratic air drag on airborne free balls (`BALL.dragK = 0.0004`, ASSUMED), applied before integration, deterministic.
- balls.js: `BallWorld.stepFixed(frameDt, hz=240)` fixed-step accumulator, max 8 steps per call, backlog dropped (no spiral of death); `step(dt)` unchanged; returns render alpha.
- balls.js: `_wakeAbove/_wakeNear` wake sleeping balls when the ball under them is removed (recycling a full cell).
- balls.js: every BALL constant tagged SOURCED or ASSUMED in comments; values unchanged. holdLimit stays 4; intake constants (docs/overnight/intake-physics.md) and the 8-pollen tip threshold (hivetip_model.js) untouched.
- shotrules/shot_core: no change needed. Shots already add pivot velocity (v + omega x r) and `solve()` compensates in closed form; test added with optional `robotVel` override; moving shots score 20/20 at 0, 30, 60 in/s on both hives (per worker e6).
- Tests: sim_physics.test.mjs (determinism, hive-cell stacking, no tunnelling at max speed x4, robot push x2, drag, restitution rebound, moving shots, stepFixed, intake guards x5, wake-on-removal); hive_tip_dynamics.test.mjs (7 tests: 7 pollen no tip, 8th tips once, monotone angle, no overshoot, reset after tip).

## Research (verified = URL given; others unverified)
- Fixed timestep + accumulator, clamp frame time: https://gafferongames.com/post/fix_your_timestep/
- Rapier CCD off per body by default, sweeps vs fixed colliders only: https://rapier.rs/docs/user_guides/javascript/rigid_bodies
- maple-sim (FRC, dyn4j) game pieces as rigid bodies; intake = contact-capture with capacity: https://shenzhen-robotics-alliance.github.io/maple-sim/simulation-details
- WPILib physics sim covers mechanisms only, no projectile model: https://docs.wpilib.org/en/2024/docs/software/wpilib-tools/robot-simulation/physics-sim.html
- Hollow hard plastic ball on concrete COR about 0.69 (snippet): https://hypertextbook.com/facts/2006/restitution.shtml
- Ball sizes 2.8 in / 3.6 in (manual TU06 9.8): https://ftc.uprobotics.tech/questions/78
- Not found: sourced foam COR/friction, nectar tip count, maniaSim, Cheesy Arena or FTCSim ball physics.
- Drag estimate 0.5 rho Cd A / m gives about 0.0011 per in; shipped 0.0004 is 3x lower, chosen so shots stay accurate (0.5-6% loss over 0.25 s).

## Constants (all in balls.js BALL)
SOURCED: g 386.0886 in/s^2 (unit conversion), r 1.391 / nectarR 1.7995 (manual diameters), restitutionWall 0.55 (weak, plastic on concrete).
ASSUMED, no source (TODO measure): restitutionFloor 0.45, restitutionBall 0.7, restitutionRobot 0.35, restitutionHive 0.3, dragK 0.0004, rollFriction 0.04, slideFriction 0.5, restSpeed 5, sleepSpeed 1.5, sleepTime 0.25 s, maxSubstep 0.004 s, captureRest 0.15 s, captureSlow 30, maxStepTravel 0.8, maxSpeed 1800.

## Audit results
- Determinism: no Math.random/Date.now in owned physics; two runs at fixed dt are bit-identical in Node 24 (cross-engine unverified). step() at differing dt diverges; stepFixed addresses it.
- Stacking in a hive cell, no tunnelling at max ball speed, robot push: tests pass on existing code, no fix needed. Robot push exits at (1+e)v (1.35x at 60 in/s), by design with ASSUMED e.
- Hive tip: fires once at 8, resets, no overshoot. No defect.
- Test run: full suite `node --test tests_js/*.test.mjs` = 749 pass, 0 fail (my re-run). An earlier mid-run invocation showed one assertion failure that did not reproduce (likely another agent's file mid-edit); area tests (142) green on 3 repeat runs.

## Open items needing a decision (not fixed)
1. A1/A2: sleeping start preloads resting in a hive cell never score (rest timer chatters across the sensor edge, never reaches captureRest 0.15 s). A pause-timer fix scored them but broke shootend2end "8 solver shots tip hive once" (tips after 7), so it was reverted. Needs a scoring-rule decision for preloads.
2. A3: robot sweep gate `mv < 40` in balls.js: a robot moving at or over 40 in per step skips ball interpolation (only at about 15 fps or lower). Suggested: remove the gate or sub-step the robot; add regression test.
3. A5: robot pinching a ball against a wall lets the ball centre reach about 8.4 in inside the robot box for about 9 frames.

## Unverified
Visual jitter in the browser; all ASSUMED constants vs real balls; cross-engine determinism.

## Follow-ups for other owners
- app.js:452 calls `world.step(dt)` directly; switch to `const alpha = world.stepFixed(dt)` for frame-rate independence and to remove A3 exposure (not my file).
- app.js:441-443 takes hive tip phase at settle time (after the 0.6 s animation), not launch time.
- Measure a dropped pollen ball and a rolling ball on the field mat to replace ASSUMED values.
