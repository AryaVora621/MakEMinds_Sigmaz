# Using your own Onshape robot CAD in the sim

The sim draws a procedural placeholder robot until `field_sim/web/data/robot.glb` exists. When it does, `web/js/robotglb.js`
loads it, scales and orients it, fits it to the 381 x 375 mm frame, and falls back to the placeholder if anything is wrong.

The Onshape document is private, so nobody but you can export it. Do these steps yourself.

## Option A: export from the Onshape UI (simplest)

1. Open the robot Part Studio (or the Assembly). Hide fasteners, wires and anything you do not want drawn.
2. Right-click the Part Studio / Assembly tab at the bottom > **Export**.
3. Format **GLB** (or **GLTF**). Resolution **Medium** or **Fine**. Leave **Compress** OFF (Draco/meshopt is not supported).
   Source: https://cad.onshape.com/help/Content/File/exporting_files.htm
4. Save the file, then either copy it to `field_sim/web/data/robot.glb`, or run the importer (it validates the file, embeds
   external `.bin` files and strips textures):

   ```
   cd field_sim
   .venv\Scripts\python.exe tools\import_robot_cad.py path\to\MaxBot.glb
   ```
5. Reload the sim (`http://localhost:<port>/`). A toast lists warnings (units guessed, size over 18 in, ignored textures).

STL / OBJ also work (one file per part, or a zip of them; each file becomes a named node):

```
.venv\Scripts\python.exe tools\import_robot_cad.py parts.zip --units mm --pivot-from-name
```

STL/OBJ exports have no units: pass `--units mm|cm|m|in` (Onshape STL dialog has a Units dropdown; default here is mm).

## Option B: Onshape API key (no manual export)

1. Onshape > My Account > Developer > **API keys** > Create (https://cad.onshape.com/help/Content/Plans/developer-myaccount.htm).
2. Set the keys as environment variables (never commit them):
   `set ONSHAPE_ACCESS_KEY=...` and `set ONSHAPE_SECRET_KEY=...`
3. `.venv\Scripts\python.exe tools\import_robot_cad.py --onshape-url "<your document URL>"`

UNVERIFIED: this path was written from the public API docs (https://onshape-public.github.io/docs/api-adv/translation/,
https://onshape-public.github.io/docs/auth/apikeys/) and has NOT been run against a live account. It posts a Part Studio
glTF translation (for an Assembly tab change `partstudios` to `assemblies` in `onshape_download_gltf`), polls it, and
downloads the result. If it fails, use Option A.

## Axes, units, fit

Onshape is Z-up, so the sim default needs telling. `web/js/app.js` currently calls
`tryLoadRobotGLB(THREE, 'data/robot.glb', robotSpec, {})` (default Y-up). For a Z-up export pass
`{ orientation: { upAxis: 'Z', forwardAxis: '+X' } }` (change `forwardAxis` if the robot drives sideways; `yawDeg` adds a final turn).
UNVERIFIED which up axis Onshape's glTF uses by default: its dialog has "Export models oriented Y axis up", so check by looking.

The loader picks m / cm / mm / in so the chassis is closest to 15 in, and warns if it is not within 40 %. Force with
`units: 'mm'` or `scale: <inches per file unit>`. The fit ignores nodes whose name contains `intake` or `flower` (they stick
out of the frame), centres the chassis on x=y=0 and puts the lowest point on the floor.

## Naming parts so they move

Name nodes/instances (any case, substring) `turret`, `hood`, `flywheel`; optional `intake`, `flower`. The topmost match wins
("Turret Assembly" beats "Turret Plate"). Custom patterns: `names: { turret: ['yaw', /rotat/i] }`.

- **Pivot**: a node rotates about its own origin. In Onshape every instance uses the assembly origin, so a turret exported
  without mate origins will swing about the robot centre. Fix by giving the sub-assembly its own origin on the rotation axis
  (turret: vertical axis, hood: lateral axis, flywheel: its axle), or use `--pivot-from-name` as an approximation.
- Turret yaws about node-local +Z, hood pitches about local Y (positive hood angle tilts up), flywheel spins about local Y.
  `animateCADParts(THREE, model.parts, { turretDeg, hoodDeg, flywheelRad })` drives them; rest rotation is the 0 pose.
- GAP (not in my files): `web/js/app.js` does not yet call `animateCADParts`, so with a CAD the turret/hood/flywheel are static.
  See the patch in `docs/overnight/robot-model.md`.

## Not supported

Draco/meshopt, textures (ignored), animations, skins, morph targets. Sparse accessors ARE supported. Files over about
100k triangles will load but slow the page: re-export at Coarse/Medium.

## Checking it worked

`node --test tests_js/robotglb_extra.test.mjs tests_js/robot_importer.test.mjs` exercises the loader and importer on synthetic
files. It does not prove your real file loads; open the sim and read the toast and the "robot model" note.
