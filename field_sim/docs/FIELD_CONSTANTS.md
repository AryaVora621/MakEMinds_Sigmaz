# BIOBUZZ field constants from the Onshape field CAD (S3)

Machine-readable: `field_sim/data/field_model/field_constants.json` (schema in `SCHEMA.md`). Java: `docs/FieldConstants_from_CAD.java.txt`.
Reproduce: `python -I field_sim/tools/compute_constants.py` then `python -I field_sim/tools/gen_java.py` (numpy only; shared helpers in `tools/cadlib.py`).
Frame: PEDRO inches (x_p = x_cad_in + 72, y_p = y_cad_in + 72, z up). Red at low x, audience at low y.
Tags: OFFICIAL (TU07 manual, page/figure cited), DERIVED-FROM-CAD (computed from tessellation + occurrence transforms), UNVERIFIED.
Tolerance: tessellation is exact to ~0.01 in; the opening aperture is rasterised at 0.04 in (so +/-0.04 in on its edges); the manual itself is +/-1 in (9.1 p62).
Note: no Haiku workers were spawned. I did the work directly with one shared codepath (`cadlib.py`) so every number comes from the same transform chain.

## Method
1. `cadlib.py` resolves each of the 1094 occurrences through rootAssembly.instances and subAssemblies to a manifest part (83 occurrences are assembly nodes with no geometry) and builds 4x4 part-local to pedro-inch transforms. Tessellation metres to inches.
2. The hive frame H is the transform of the `Goal Pivot Assembly` occurrence. Its origin lies on the pivot axis (y = 72.000, z = 43.9497), the axis is world X, and its rotation is exactly Rx(-30 deg) for the red hive (audience cell up) and Rx(+30 deg) for the blue hive (far cell up).
3. Cell opening: the outer rib (p026 red / p056 blue, larger |y_H|) has a thin lip on its outer face plane (y_H = +/-21.457). I rasterise that face and flood fill the hole; this is the clear aperture. Roof slope fitted on the right edge.
4. Tip states: the other stable state is the hive rotated 60 deg about the pivot axis (red +60, blue -60). Applied as `rot_about(delta, pivot, X) @ T_cad`. The red audience_up and blue audience_up apertures agree to 0.001 in (independent geometry and an independent rotation), which is the main self-check.
5. Stickers (p029 red audience, p083 red far, p057 blue audience, p079 blue far): part-local x = 17 in, **local z = 5 in, thickness is along local y (-0.01..0), not z** (the brief said z). Printed face = the exposed face (z_H = -1.498, skin outer surface at -1.488), which faces away from the cell. The tessellation has no appearance data and no holes (172 triangles, rounded rectangle), so ink side is inferred, not read.
6. Bottom edge of the sticker = the edge nearer the field centre; that edge is at |y_H| = 11.519 and the cell front is at 21.457, so front-of-cell to hole line = 9.938 in, which equals Fig 9-15 exactly. This pins the tag row to 2.75 in from that edge (tag centre at |y_H| = 14.269). Up vector = away from the bottom edge; right = up x normal as seen facing the tag.
7. Flowers from p113 "Flower Scoring Volume" (bbox) and p104 Layer C (top ring). Tape zones from the tape occurrence transforms (tape parts p000-p019 have no tessellation, Onshape 403; tape origin taken at its centre, validated by sizes below).

## Headline results

### Hives (CAD, both hives identical in hive-local geometry)
| Item | Value | Tag |
|---|---|---|
| Tilt of the arm in either stable state | 30.000 deg about X | DERIVED-FROM-CAD (OFFICIAL callout Fig 9-10) |
| Pivot axis | along x, y = 72.000, z = 43.950; x = 59.267 red / 84.767 blue (centre of the opening) | DERIVED-FROM-CAD; z OFFICIAL 9.6.1; x vs Fig 9-10 59.25/84.75: +0.02 |
| CAD pose | red = audience_up (angle -30), blue = far_up (angle +30). Delta to other state: red +60, blue -60 | DERIVED-FROM-CAD |
| Cell clear opening | pentagon: width 20.04, height 14.00 in plane, area 218 in^2; vertical walls to z_H 6.40, roof slope 32.6 deg, peak z_H 12.6 | DERIVED-FROM-CAD |
| Cell depth (opening plane to back skin) | 12.028 in | DERIVED-FROM-CAD |
| Opening normal | outward along the arm: audience cell up (0, -0.866, +0.5); far cell up (0, +0.866, +0.5) i.e. 30 deg ABOVE horizontal, not vertical | DERIVED-FROM-CAD |
| Up-cell opening z range | bottom 53.466, top 65.590 | DERIVED-FROM-CAD |
| Down-cell opening z range | 32.009 to 44.133 | DERIVED-FROM-CAD |
| Lowest point of hive (CAD pose) | 30.65 red, 30.87 blue | DERIVED-FROM-CAD |
| Arm: pivot to opening plane / back wall | 21.457 / 9.429 in | DERIVED-FROM-CAD |
| Cell floor | plane of aperture bottom, slopes 30 deg: front edge z 53.47 (y 52.72), back edge z 47.45 (y 63.14) for red audience_up | DERIVED-FROM-CAD |

### Aim points (UP cell opening centroid, pedro x, y, z) and normals
| Hive | State (up cell) | aim_point | bbox-centre alt. | normal |
|---|---|---|---|---|
| red | audience_up (audience cell) | 59.267, 55.517, 58.313 | 59.267, 56.218, 59.528 | 0, -0.866, 0.5 |
| red | far_up (far cell) | 59.267, 88.483, 58.313 | 59.267, 87.782, 59.528 | 0, 0.866, 0.5 |
| blue | audience_up | 84.767, 55.517, 58.313 | 84.767, 56.218, 59.528 | 0, -0.866, 0.5 |
| blue | far_up | 84.767, 88.483, 58.313 | 84.767, 87.782, 59.528 | 0, 0.866, 0.5 |

The aim y is 16.48 in from the field centre (the opening plane sits at the end of the 21.46 in arm), not the 8.16 in or 9.42 in used in 12_master. Down-cell centroids (state audience_up: far cell at y 92.68, z 36.86; state far_up: audience cell at y 51.32, z 36.86) are in the JSON. Full corners, polygon, floor and back-wall corners per cell/state are in the JSON.

### AprilTags (clusters centred at the hive x: red 59.258 +/- offsets, blue 84.758)
Offsets from cluster centre along printed-face "right": -6.5, -2.75, +2.75, +6.5 (OFFICIAL Fig 9-15). IDs ascend left to right on the printed face. Resulting world x of tag centres:
| IDs | Cell | x_p of tags (ascending ID) | y_p, z in audience_up | y_p, z in far_up |
|---|---|---|---|---|
| 30-33 | red far | 65.76, 62.01, 56.51, 52.76 | 83.61, 35.52 | 85.11, 49.79 |
| 34-37 | red audience | 52.76, 56.51, 62.01, 65.76 | 58.89, 49.79 | 60.39, 35.52 |
| 38-41 | blue audience | 78.26, 82.01, 87.51, 91.26 | 58.89, 49.79 | 60.39, 35.52 |
| 42-45 | blue far | 91.26, 87.51, 82.01, 78.26 | 83.61, 35.52 | 85.11, 49.79 |

Normal (toward the viewer, away from the cell skin) depends only on the hive state, the same for all 16 tags: audience_up (0, -0.5, -0.866); far_up (0, +0.5, -0.866). Both are 60 deg below horizontal, so every tag faces the floor and leans toward the audience wall in audience_up and toward the far wall in far_up. Tag up vector: audience cells (0, -0.866, +0.5) audience_up / (0, -0.866, -0.5) far_up; far cells (0, +0.866, -0.5) / (0, +0.866, +0.5). Right vector: +x for audience cells (34-41), -x for far cells (30-33, 42-45). The JSON holds center/normal/up/right/corners (TL,TR,BR,BL facing the tag) per tag per state.
Example (id 36, audience_up): corners TL 60.383,57.486,50.599; TR 63.633,57.486,50.599; BR 63.633,60.301,48.974; BL 60.383,60.301,48.974.

### Flowers (no fixed owner: ownership is gameplay-dependent, manual 10.5.2 p86)
| id | opening centre (x_p, y_p, z) | wall | scoring volume z | note |
|---|---|---|---|---|
| 1 | 48.607, 140.071, 21.404 | far | 4.254-21.054 | |
| 2 | 95.393, 3.929, 21.404 | audience | same | |
| 3 | 3.929, 48.607, 21.404 | red side wall | same | |
| 4 | 140.071, 95.393, 21.404 | blue side wall | same | |
Opening radius 1.98 (diameter 3.96), top ring top 21.404, backstop top 22.654.

### Zones, walls, tiles (CAD)
| Item | Value |
|---|---|
| Red LOADING ZONE (outer tape edges) | x 1.90-12.90, y 95.91-118.60 (11.0 x 22.69) |
| Blue LOADING ZONE | x 131.10-142.10, y 25.40-48.09 |
| Red GARDEN | x 1.90-24.59, y 1.90-3.90 (22.69 x 2.0), audience wall, red corner |
| Blue GARDEN | x 119.41-142.10, y 140.10-142.10, far wall, blue corner |
| ALLIANCE AREAS | red x -53.65 to 0.35, blue x 143.65 to 197.65; both y 23.59-120.41 (54 x 96.8) |
| Perimeter glass inner faces | x 1.326 and 142.674, y 1.326 and 142.674 (interior 141.35); rail inner face 1.41/142.59; glass z 0.09-10.96; rail top 11.64 |
| Tiles | 36 tiles, array 1.42-142.58 (141.16), top z 0, thickness 0.59 |
| Balls | pollen 2.80 dia, nectar 3.62 dia |

## Comparison with TU07
| Quantity | Manual (TU07) | CAD | Verdict |
|---|---|---|---|
| Field interior | ~144 x 144 (9.2 p62) | 141.35 between glass faces; tile array 141.16 | DISAGREE by 2.7 in (CAD +/-1 in per 9.1). Pedro [0,144] is nominal; real wall contact is at 1.33 / 142.67. |
| Tile 24 x 24 x 0.59 | 9.2 | 36 tiles, 0.59 thick, interlocked | agree |
| Pivot height | 43.95 (9.6.1 p67) | 43.950 | agree |
| Hive centre-to-centre | 25.5 (Fig 9-10 p69) | 25.500 (84.767-59.267) | agree |
| Cell depth | 12.04 (Fig 9-9 p69); "12 deep" 9.6.2 | 12.028 | agree |
| Hive span | 42.91 (Fig 9-9) | 42.914 (2 x 21.457) | agree |
| "18.84" | "CELLS approximately 18.8 apart" (9.6.2); Fig 9-9 | 18.86 = gap between the inner faces (2 x 9.43), NOT centre to centre | clarification: centre to centre is 30.9 |
| Opening 20 w x 14 t | 9.6.2 p68 | 20.04 x 14.00 (in the opening plane) | agree |
| Opening z 53.5 / 65.6 | Fig 9-10 p69 | 53.47 / 65.59 | agree; the "14 vs 12.1" conflict is solved: 14 in is measured in the tilted plane, 12.1 is its vertical projection (14 x cos30 = 12.1) |
| Lowest point 30.6 | Fig 9-10 | 30.65 red, 30.87 blue | agree (blue +0.27) |
| Tilt 30 deg | Fig 9-10 | 30.000 | agree |
| Frame footprint 49.46 x 38.95 | Fig 9-8 p68 | not extracted (frame parts not checked for width; hive-local axis along x, so the 49.46 side presumably runs along X) | UNVERIFIED |
| Tag size 3.25, offsets +/-2.75, +/-6.5 | Fig 9-15 p73 | sticker 17 x 5 in, tag row 2.75 above hole line, hole line 9.938 from front of cell | agree (9.938 exact) |
| Tag ID order | Fig 9-17 p75 | red far 30 at +x ... 33 at -x; red audience 34 at -x ... 37 at +x; blue audience 38-41 and blue far 42 at +x ... 45 at -x | agree (mirror worry in 12a is resolved: it was the sticker seen from below vs a top view) |
| Flower opening height 21.5 | 9.7 p70 | 21.40 | agree (-0.1) |
| Flower opening dia ~4 | 9.7 | 3.96 | agree |
| Flower XY | Fig 9-2 reading (48,144), (144,96), (96,0), (0,48) | (48.6,140.1), (140.1,95.4), (95.4,3.9), (3.9,48.6) | agree in x/y order; the y/x 144 and 0 were wall positions, the opening is 3.9 in inside the wall |
| Loading zone 23 x 11 | 9.3 p63 | 22.69 x 11.0 | agree |
| Garden 23 x 2 | 9.3 p64 | 22.69 x 2.0 | agree |
| Alliance area 97 x 54 | 9.3 p63 | 96.8 x 54.0 | agree |
| Red hive on audience's left | 9.5 / Fig 9-17 | red at low x | agree |

## Answers to the open questions
1. **Corners and floor of each cell**: in the JSON per hive, state, cell (`opening_corners`, `opening_polygon`, `floor_corners`, `back_wall_corners`). The opening is a pentagon (rectangle 20.04 x 8.0 plus a roof triangle), not a 20 x 14 rectangle. The floor slopes 30 deg when the cell is up (the ball rolls toward the back wall).
2. **Aim point and normal**: table above. The normal tilts 30 deg above horizontal, pointing outward along the arm. The old 12_master aim y (63.84 / 80.16) is wrong by 8.3 in and z (59.5) is right only for the bbox centre.
3. **Does the printed face point down or outward?** It is the face of the 0.01 in sticker that is exposed, normal = hive-local -z (away from the cell skin): 60 deg below horizontal in either state. In the CAD tess no appearance/ink data exists, so which face carries ink is inferred from "only the outer face is visible" and from Fig 9-16 (face downward). Confidence high.
4. **Which tag ID at which end**: from CAD geometry (bottom edge toward field centre, up = away from it, ascending ID left to right on the printed face) the resulting world-x ordering matches Fig 9-17 for all four clusters. Two independent sources agree, so confidence is high. Residual risk: the audience stickers' internal ID order is not printed in the manual figure that is supplied (Fig 9-15 shows 30-33 only).
5. **Hive pivot y**: 72.000 exactly (axis along x).
6. **Both hives, both states**: done (red delta +60, blue -60).

## Unresolved / caveats
- Perimeter width mismatch (141.35 vs 144) should be decided by the team (which one the localizer uses).
- Frame footprint (49.46 x 38.95) and the scoring-volume of the hive were not re-derived.
- Tape part origin assumed at the centre; corroborated by sizes matching the manual within 0.4 in and by tape edges sitting 0.5 in from the wall.
- Blue lowest point 0.22 in above red: unexplained asymmetry in the blue cell parts (not investigated).
- Tag position tolerance: positions follow from the sticker transform (exact in CAD); real stickers may be shifted by hole tolerances (+/-0.25 in not measured).
