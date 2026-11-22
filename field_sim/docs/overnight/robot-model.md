# robot-model (Sonnet lead): robot CAD import prep

## Changed files (all mine)
- web/js/robotglb.js: sparse accessors decoded; draco/meshopt anywhere (used, required, bufferView, primitive) and unknown
  required extensions now rejected with a reason; emissive_strength applied, specGloss approximated, transmission/volume/etc warn;
  negative/NaN `opts.scale` rejected, unknown `units` warn; fit/centre ignore `intake`/`flower` subtrees (chassis-only box);
  `findMovingParts` also finds optional `intake`/`flower` (never listed in `missing`); new export `animateCADParts()`.
- web/js/robotmodel.js: `buildRobot().dispose(root, except)` frees geometry/material GPU buffers.
- web/js/robotspec.js: missing `spec.frame` is now a warning.
- web/data/robot_spec.json: intake.capacity 3 -> 4 (BallTracker.java MAX_BALLS=4, user-confirmed hold limit; tag stays "repo").
- tools/import_robot_cad.py (new): glb/gltf pass-through (embeds buffers, strips textures, rejects Draco), STL (binary+ascii),
  OBJ, zip of parts -> one GLB with a node per part; `--units`, `--bake-axes`, `--pivot-from-name`; optional `--onshape-url`.
- docs/CAD_IMPORT.md (new): exact user steps (UI export, API-key option), axes/units/fit, naming, pivots, limits.
- tests_js/robotglb_extra.test.mjs, tests_js/robot_importer.test.mjs (new). Edited Haiku tests: a16 (frame check now expects a
  warning), a14 (found now includes intake/flower), a17 (deleted "hood raises muzzle height": wrong, release point is fixed in
  the turret frame by design; the hood sets launch pitch, which a17 already tests).

## Evidence (ran)
- `node --test tests_js/robot*.test.mjs tests_js/h_robot-model*.test.mjs`: 78 pass, 0 fail (before the importer test was added).
- `node --test tests_js/robot_importer.test.mjs`: 2 pass. It runs the .venv python on a synthetic STL zip (381x375x200 mm chassis
  + turret), loads the GLB with loadRobotGLB, checks 15.00 x 14.76 in, floor z=0, turret found, pivot re-base keeps the turret centre
  fixed under yaw, GLB re-import pass-through, and Draco / missing-input exit codes.
- `node --test tests_js/*.test.mjs`: tests 975, pass 956, fail 0, skipped 3, todo 16.
- Not run: any browser/headless Chrome render with a CAD; the real Onshape file.

## Haiku findings: accepted / rejected
- Accepted: a01 (sparse, extension gating), a03 (chassis-only fit, units warn), a04 (intake/flower patterns), a05 (documented pivot
  contract), a09 (capacity 4), a16/a19 (frame warn, dispose), a18 (doc outline), a20 (the 2 failing tests).
- Rejected/not acted: a17 muzzle-height test (wrong, see above). KHR_texture_transform etc. allowed in "required" list but textures
  are ignored anyway. a09 mm-in-inch-spec notes for gate.openMm and the cad_todo gaps: left as is (labelled in the spec, no code reads them wrongly); not re-verified.
- Spot-checked by me against code: a01 sparse/extension claims, a04 matching behaviour, a09 capacity vs BallTracker (Facts say hold
  limit 4). Others (a02 matrix maths, a08 API endpoints) read but not independently re-derived.

## Unverified
- Onshape glTF default up-axis and units (docs only hint). The loader auto-detects units; up axis must be chosen by the user.
- `--onshape-url` flow (endpoint, HMAC signing, result download) was never run; written from public docs, may need tweaks.
- Whether an Onshape export puts moving parts at identity nodes with the pivot at the assembly origin (likely; documented workaround).
- Khronos glTF spec was not fetched; sparse-accessor decode follows memory of the spec and is tested only on a synthetic file.

## Patches for other owners
1. web/js/app.js (~line 254, CAD branch): drive the CAD each frame and free the placeholder:
   ```js
   // after robot.add(r.model.group):
   const cadParts = r.model.parts;           // keep in scope
   built.dispose(robot, built.parts.flowerArm && built.parts.flowerArm.group);   // optional: free placeholder GPU buffers
   // in the frame loop next to built.animate(...):
   if (cadParts) animateCADParts(THREE, cadParts, { turretDeg: S.turretDeg, hoodDeg: S.hoodDeg, hoodZeroDeg: <hood min or mid>, flywheelRad: flyRad });
   ```
   import `animateCADParts` from './robotglb.js'. Pass `{ orientation: { upAxis: 'Z', forwardAxis: '+X' } }` as the 4th arg of
   `tryLoadRobotGLB` for an Onshape Z-up export. Also `console.warn(r.reason)` when `r.model` is null, so a bad file is visible.
   Note `built.dispose` visits hidden placeholder meshes only if they are still in `robot`; call it before `robot.add(CAD group)`.
2. web/data/MODEL_README.md still says sparse accessors are unsupported and lacks the intake/flower names: point it at docs/CAD_IMPORT.md.
