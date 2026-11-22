# Robot model: procedural fallback and your own CAD

The simulator draws a procedural robot (`web/js/robotmodel.js`) built from `robot_spec.json`. To use your real
MaxBot CAD instead, export it from Onshape as glTF/GLB and drop it next to this file.

## Export from Onshape

1. Open the MaxBot Part Studio or Assembly. For an assembly, hide anything you do not want drawn (fasteners, wires).
2. Right-click the Part Studio / Assembly tab at the bottom, choose **Export...**.
3. Format: **GLTF** (a `.glb` binary is best; if you get `.gltf` plus a `.bin`, keep them together).
4. Options: leave **Units** as is (metres or millimetres both work, the loader detects it), turn **Draco compression OFF**
   (not supported), and prefer one file for the whole assembly.
5. Save the file as `field_sim/web/data/robot.glb`.

## Place it

Put the file at `field_sim/web/data/robot.glb`. The host calls
`loadRobotGLB(THREE, 'data/robot.glb', spec, options)`; if the file is missing or unreadable it returns `null`
(reason in `loadRobotGLB.lastReason`) and the procedural robot is used.

## Orientation and size

glTF is Y-up; Onshape files are often Z-up. Tell the loader which file axis is up and which points to the robot front:

```js
loadRobotGLB(THREE, 'data/robot.glb', spec, {
  orientation: { upAxis: 'Z', forwardAxis: '+X' },   // or upAxis 'Y'; forwardAxis '+X' '-X' '+Y' '-Y' '+Z' '-Z'
  // units: 'mm',        // force units (m, cm, mm, in) if auto-detect warns
  // scale: 0.03937,     // or an explicit file-unit -> inch factor
  // offset: [0, 0],     // nudge the chassis centre (inches)
});
```

Defaults: `upAxis 'Y'`, `forwardAxis '+X'`. If the robot appears sideways, change `forwardAxis`; if it lies on its side, change `upAxis`.
Optional `yawDeg` adds a final rotation about up.

The loader scales to inches (auto-picks m / cm / mm / in so the largest footprint dimension is within 40 % of the spec
footprint, 18 in by default; otherwise it still loads and returns a `warnings` entry), centres the bounding box on the
robot origin and puts the lowest point on the floor (z = 0). Result: `{ group, scale, units, bounds, warnings, parts }`.

## Moving parts

Name the turret, hood and flywheel groups/parts in Onshape (instances in an assembly, or parts) with the words
`turret`, `hood`, `flywheel` anywhere in the name (case-insensitive). `findMovingParts(group)` returns
`{ turret, hood, flywheel, found, missing }` (also under `model.parts`) so the host can rotate them. Custom patterns:
`loadRobotGLB(..., { names: { turret: ['yaw', /rotat/i] } })`. For a sensible pivot, the node origin should be the
rotation axis; Onshape uses the assembly origin for every node, so for animation you may need to set mate origins
or export the moving sub-assemblies as separate files.

## Parser limits

three's `GLTFLoader` is not vendored (offline), so `web/js/robotglb.js` has its own parser: meshes, node transforms,
simple PBR colours, `.glb` and `.gltf` (embedded or relative buffers). Not supported: textures (ignored), animations,
skins, morph targets, Draco/meshopt compression, sparse accessors.
