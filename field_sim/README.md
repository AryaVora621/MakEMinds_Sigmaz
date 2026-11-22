# BIOBUZZ Field Simulator (FTC 2026-27)

A 3D, hardware-accelerated (WebGL) model of the full BIOBUZZ field, exported once from the official Field CAD so
Onshape is never needed again. It shows every part (tiles, walls, hive frame, red and blue hives with working tip
animation, flowers, tape and zones, pollen and nectar, trays), the 16 AprilTags (IDs 30-45) with their real
36h11 patterns, and a robot with a turret, hood and flywheel that aims using the SAME ballistic physics as the
robot code (`ShotSolver.java`). It can also be driven live by the real robot (Pedro pose, turret, hood, RPM).

## Run it

```
cd field_sim
.venv\Scripts\python.exe run_sim.py          # starts http://127.0.0.1:8780 and opens your browser
```
(First time only: `python -m venv .venv` then `.venv\Scripts\python.exe -m pip install -r requirements.txt`.
The viewer itself only needs Python's standard library; three.js is vendored under `web/vendor`.)
If the data changes, rebuild it with `run_sim.py --export` (reads `data/field_model`, writes `web/data`).

## Controls

| Input | Action |
|---|---|
| W/S, A/D, Q/E (+Shift) | drive forward-back, strafe, rotate |
| Space | shoot (blocked if the flywheel or turret is not ready, or the shot would arrive from behind the opening) |
| click the floor | place the robot there |
| T / Y | tip the red / blue hive between its two stable states |
| 1-6 | cameras: iso, top, audience, side, follow, turret camera |
| L | labels (tag IDs, zone names) |
| R | reset the robot; mouse: drag = orbit, right-drag = pan, wheel = zoom |

The left panel shows the **physics setpoint** (distance, flight time, turret angle, hood angle, RPM, entry angle),
which AprilTags the turret camera sees and the `tx` to your alliance's tags, and a shots / scored counter. The
right panel toggles every layer (including AprilTags, aim points, trajectory, camera frustum).

**Two tabs** (top of the window, or press M): **3D view**, and **2D map (Pedro)**, a bird's-eye view in Pedro
coordinates (origin bottom-left, x right, y toward the far wall, 24 in tiles, zones, hive cells for the current tip
states, tags, flowers, ball starts, robot footprint, turret direction, camera wedge, shot ground track, pose trail,
and the robot's Pedro path). In the 2D tab: click places the robot, right-click turns it to face a point, and
ctrl-click copies `new Pose(x, y, Math.toRadians(h))` for pasting into your path code. Both tabs share the same state.

Important field fact the sim exposes: each raised cell's opening faces one side of the hive, 30 deg above
horizontal. A ball only scores if it travels against that normal, so you must shoot from the open side (the far
side when the far cell is up, the audience side otherwise). The robot code (`TurretAiming.fromBehind`) blocks
shots from behind it too.

## Practice mode, physics and path designer

- **Collisions.** The robot is an 18 x 18 in rotated box. It cannot leave the glass walls (play area 1.33 to 142.67 in, from the CAD) and cannot drive through the hive legs, hive rails or flower bases. Colliders are generated from the CAD by `tools_js/extract_colliders.mjs` into `web/data/colliders.json`. A robot can drive under the raised hive along y between the legs (about 37 in gap); the 2 in floor rails block entry from the sides (team decision pending).
- **Balls.** All 56 balls are real physics bodies (`web/js/balls.js`): gravity, bounce, rolling friction, ball-to-ball collisions, they get pushed by the robot, and they bounce inside the hive cells. The intake (key I) loads free balls in front of the robot (3 balls max, uncheck "limited balls" for unlimited). A shot ball is captured only when it crosses a hive opening against its normal.
- **Hive cycles.** Scored balls load the hive and it tips by itself (`web/js/hivecycle.js`): 3 balls tip it with the default settings, and the lowered cell is cleared on a tip. Then you must score from the other side. T / Y still force a tip.
- **Driver practice.** Cameras 7 Driver view (standing driver, eye height 64 in), 8 Driver wide, 9 Chase, 0 Coach box. Drag with the mouse to look around. Gamepad: left stick drive, right stick turn, RT shoot, Back toggles field-centric (also F). Start match runs a 30 s auto, 8 s transition, 2:00 teleop clock and keeps score.
- **Path designer** (third tab). Click adds a line, shift-click adds a curve, drag moves points, right-click deletes, Alt turns off the 1 in snap. Heading modes constant, linear, tangent. Copy or download Java for Pedro Pathing 2.1.0 (`BezierLine`, `BezierCurve`, `PathChain`), save and load JSON, mirror red/blue, play the route back.
- **Shooting modes** (Shooting panel): *Auto-aim* (turret, hood and flywheel follow the solver; a shot is refused unless everything is in range and ready), *Assisted* (you aim with the arrow keys, [ ] for rpm, X snaps to auto, Z stops the flywheel, Shift = fine; the physics rules still apply and the status panel tells you where the ball would land), *Manual* (the ball flies from your real turret, hood and rpm; only hard impossibilities block), *Free sandbox* (nothing blocks). Gamepad: hold RB, then right stick aims, D-pad up/down changes rpm.
- **Why a shot is refused.** The pinned READY / BLOCKED panel lists every reason (too close or too far for the shooter, hood or rpm limit, turret limit, ball would arrive from behind the opening, path blocked by the hive legs, flywheel not ready, no balls, illegal area) and a hint. The game rules have no launch zone, so "legal area" is a configurable assumption (presets in the panel, default Open). Tick "show shootable area" on the 2D map to see where auto-aim can shoot from.
- **Errors.** JavaScript errors, a failed startup, and blocked-shot reasons are shown on screen (red banner with Copy report; backtick key opens the log).
- **Robot model.** The robot is built from `web/data/robot_spec.json`. Every number is tagged cad / repo / placeholder; today nothing is from the CAD yet (see `cad_todo` in the file). Put the measured values there and the sim uses them.
- **Official rules.** Scoring, timing, the 4-ball hold limit, 4 preloaded pollen and the driver positions follow the BIOBUZZ manual (see `web/js/matchrules.js`; each rule is flagged official or assumed). The manual does NOT publish how many balls tip a hive: `HIVE_PARAMS.ballsToTip` (default 4) is an editable assumption.
- **Path planner** (Path planner tab): looks like the Pedro Visualizer and draws paths sampled by the REAL Pedro 2.1.0 core library through `/api/pedro/sample` (badge shows "Pedro 2.1.0 core" or "JS fallback"). Needs a JDK: see `tools_java/README.md`. Imports/exports Visualizer `.pp` files.
- **Pedro tool** tab: the official Pedro Pathing Visualizer (Apache-2.0), vendored and built locally (`third_party/README.md`). Its own Java export targets a newer Pedro API than 2.1.0; use the sim's exporters instead.
- **Auto planner** (Auto planner tab): PathPlanner-style command editor. Build an auto from follow-path, intake, shoot, aim, wait, wait-until and sequential/parallel/race/deadline groups, with a timeline. "Run in sim" makes the sim robot perform it; "Export Java" writes a state-machine OpMode that compiles against Pedro 2.1.0 (flower, gate, park and setRpm export as TODO stubs).
- **Robot model.** Procedural placeholder. To use your CAD, export a GLB from Onshape and save it as `web/data/robot.glb` (`web/data/MODEL_README.md`).
- **Tests.** `node --test tests_js/*.test.mjs` (360+ tests, Node 20+, no install).

**Assumptions that are NOT from the rules (check against the game manual):** match timing, point values (ball 1, audience cell 3, far cell 5), ball mass and the hive latch strength (so 3 balls tip it), that a tip clears the lowered cell, ball bounce and friction values, driver station position.

## Sync with the real robot / Pedro Pathing

The viewer polls `GET /api/state`; anything that `POST`s JSON to `/api/state` drives the robot (stale after 1 s).

```
.venv\Scripts\python.exe run_sim.py --host 0.0.0.0      # allow the robot to reach the PC
```
NOTE: the robot-side `SimBridge` class was removed from TeamCode, so the robot no longer posts to this sim
(it can still be driven by any other client that POSTs to `/api/state`). Fields: `pose{x,y,h}` (Pedro inches, radians), `turretDeg`, `hoodDeg`, `rpm`,
`alliance`, `tip{red|blue: "audience_up"|"far_up"}`, `path [[x,y,heading],...]` (drawn green in both tabs).
`POST /api/replay` takes a
list of timestamped states and plays it back in real time.

## Data (all in inches, Pedro frame: origin at the audience-left corner, x right, y toward the far wall, z up)

`data/field_model/` is the complete export (3.7 MB): `model.json` (1,011 instances), `meshes/*.npz` (115 parts),
`field_constants.json` (hive pivots, aim points per tip state, all 16 tags with centre/normal/corners per tip
state, flowers, zones), `tape_zones.json`, `ball_start_positions.json`, `SCHEMA.md`, `docs/FIELD_CONSTANTS.md`
(comparison against the manual). `data/raw/` is the untouched Onshape dump (310 MB, git-ignored): the full
assembly JSON and every part tessellation. `web/textures/` holds the verified tag36h11 images.

## Files

- `run_sim.py` web server + live-link API; `web/index.html`, `web/js/app.js` the viewer; `web/js/shot_core.js` the solver.
- `tools/export_web.py` converts `data/field_model` to `web/data` (meshes with smoothed normals).
- `sim/`, `tests/`, `run_sim_software.py` the earlier numpy Field API (`sim/field.py`) and software renderer, kept
  for headless use and tests (`pytest tests`). The WebGL viewer is the main one.

## Known limits

- The 2 in tape and the hardware (screws, nuts) are reconstructed stand-ins (their Onshape documents are private).
- The turret/flywheel are placeholder dynamics; the drive is a mecanum model with acceleration limits. Real behaviour comes from the live link.
- CAD perimeter interior is about 141.35 in, not the manual's 144 in; tile pitch is 23.44 in.
- Which face of each tag sticker carries the print is inferred (no ink data in the CAD).
