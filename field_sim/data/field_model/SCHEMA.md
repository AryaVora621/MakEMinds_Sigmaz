# BIOBUZZ field model: file formats (schema 1)

Producer: orchestrator A (EXTRACT & MODEL). Consumer: orchestrator B (SIM) and robot code.
All files live in `field_sim/data/field_model/` unless noted. JSON is UTF-8, numbers are plain floats.

## Frames and units
- Units: **inches** everywhere (angles in degrees).
- Output frame **pedro**: origin at the audience-left field corner, +x to the audience's right,
  +y toward the far wall (audience is at y = 0), +z up. Field interior is x,y in [0,144].
- CAD frame (Onshape) has origin at field centre, same axis directions, metres.
  Conversion: `x_p = x_cad_in + 72`, `y_p = y_cad_in + 72`, `z_p = z_cad_in`, `in = m / 0.0254`.
- Red alliance is at LOW x (red hive x = 59.25), blue at HIGH x (blue hive x = 84.75).

## Part keys
A part key is `p` + the 3-digit manifest index `n` from `data/raw/part_manifest.json`
(e.g. `p026` = "am-5866-red: Red Goal Rib"). Keys are stable; the Onshape key string is kept in
`parts.json` as `onshape_key`.

## meshes/<partkey>.npz  (written by S1)
`numpy.load(path)` gives:
- `vertices` float32 (N,3), inches, **part-local frame** (the frame the instance transform maps from).
- `faces` int32 (M,3), indices into vertices, counter-clockwise = outward where known.
- `face_colors` uint8 (M,4) RGBA, optional (present when the source had appearances).
Vertices are de-duplicated. Large parts may be decimated (see `parts.json`.decimated).

## parts.json  (written by S1)
```
{ "schema":1, "units":"in",
  "parts": { "p026": { "name": "...", "onshape_key": "...", "source": "onshape|step|synthetic",
                        "mesh": "meshes/p026.npz", "triangles": 12345, "triangles_original": 190000,
                        "decimated": true, "bbox_min": [x,y,z], "bbox_max": [x,y,z],
                        "default_color": [r,g,b,a] (0-255), "notes": "..." } } }
```
`source`: onshape = Onshape tessellation; step = local solid geometry from the official STEP;
synthetic = reconstructed box/cylinder (documented in notes).

## model.json  (written by S2)
```
{ "schema":1, "units":"in", "frame":"pedro",
  "parts": { "<partkey>": {"name":..., "mesh":"meshes/<partkey>.npz"} },
  "instances": [ { "id": "<occurrence path ids joined by '/'>",
                   "name": "<leaf instance name>", "path": ["<names root->leaf>"],
                   "category": "tile|perimeter_wall|perimeter_pin|hive_frame|hive_red|hive_blue|apriltag_sticker|flower|tape|scoring_pollen|scoring_nectar_red|scoring_nectar_blue|artifact_tray|decoration|hardware|other",
                   "subcategory": "<e.g. cell_audience, cell_far, rib, skin, tag_audience, tag_far>|null",
                   "group": "hive_red|hive_blue|hive_frame|flower_<id>|field|...",
                   "alliance": "red|blue|null",
                   "part": "<partkey>",
                   "transform": [16 floats, row-major 4x4, part-local inches -> pedro inches] } ],
  "dropped": [ {"id":..., "name":..., "reason":"assembly node (no geometry)|..."} ],
  "groups": { "hive_red":  {"pivot":[x,y,z], "axis":[1,0,0], "cad_pose_angle_deg": <tilt of the CAD pose>,
                             "cad_pose_state": "audience_up|far_up",
                             "stable_angles_deg": {"audience_up": a1, "far_up": a2} },
              "hive_blue": {...} },
  "stats": { "occurrences_total": 1094, "instances": n, "dropped": n, "by_category": {...} } }
```
Transforms map a homogeneous column vector: `p_world = T @ [x,y,z,1]`, T row-major.
**Tipping a hive**: to put hive group G into state S, left-multiply every instance transform whose
`group == G` by the rotation of `stable_angles_deg[S] - stable_angles_deg[cad_pose_state]` degrees
about `axis` through `pivot` (right-hand rule). The AprilTag stickers are in the hive group.

## ball_start_positions.json  (S2)
```
{ "schema":1, "units":"in", "frame":"pedro",
  "balls": [ {"id": "<instance id>", "type": "pollen|nectar_red|nectar_blue",
              "position": [x,y,z] (ball centre), "diameter": d,
              "location": "tray|preloaded|field|other", "where": "<free text e.g. red tray 1 slot 3>"} ],
  "counts": {"pollen": n, "nectar_red": n, "nectar_blue": n} }
```

## tape_zones.json  (S2)
```
{ "schema":1, "units":"in", "frame":"pedro",
  "tapes": [ {"id":..., "name":..., "color":"red|blue|white|...", "width": w, "length": L,
              "p0":[x,y], "p1":[x,y] (centre-line endpoints), "z": top_z } ],
  "zones": [ {"name":..., "kind":"loading_zone|garden|alliance_area|start_line|tile_seam|other",
              "alliance":"red|blue|null", "polygon":[[x,y],...], "source":"tape|tiles|manual"} ] }
```

## field_constants.json  (S3)
```
{ "schema":1, "units":"in", "frame":"pedro",
  "field": {"size":[144,144], "walls": {...}, "tile": {...}},
  "hives": { "red": { "pivot":[x,y,z], "axis":[1,0,0], "tilt_deg":30,
                      "states": { "audience_up": { "angle_deg_from_cad": a,
                                    "cells": { "audience": {"up": true, "opening_corners":[[x,y,z]x4],
                                                             "aim_point":[x,y,z], "opening_normal":[x,y,z],
                                                             "floor_corners":[[x,y,z]x4], "depth": d},
                                               "far": {...} } },
                                  "far_up": {...} } },
             "blue": {...} },
  "apriltags": { "30": { "alliance":"red", "side":"far", "size":3.25,
                         "states": { "audience_up": {"center":[..], "normal":[..], "up":[..], "right":[..],
                                                       "corners":[[..]x4] (TL,TR,BR,BL as seen facing the tag)},
                                     "far_up": {...} } }, ... "45": {...} },
  "flowers": [ {"id":..., "alliance":..., "opening_center":[x,y,z], "opening_height": z, "radius": r} ],
  "zones": {...}, "balls": {...},
  "provenance": { "<dotted.key>": "OFFICIAL|DERIVED-FROM-CAD|UNVERIFIED" } }
```
State names are always `audience_up` and `far_up` (which cell of that hive faces up/opens).

## Notes found during export (read these)
- Hive cell openings are PENTAGONS: `opening_corners` is their in-plane bounding rectangle,
  `opening_polygon` the true outline, `aim_point` the area centroid (`aim_point_bbox_center` also given).
- Tag sticker parts (p029/p057/p079/p083) are 17 x 5 in with thickness along local **y** (-0.01..0).
- The CAD perimeter interior is ~141.35 in (glass inner faces at 1.33 / 142.67), not 144; tiles span ~1.42-142.58.
- Alliance-area tape and artifact-tray/stock balls lie outside the walls (x < 0 or x > 144) on purpose.
- model.json hive instances carry an extra `cell` field (audience|far). Tape meshes p010-p019 (2 in) and
  hardware are approximations (see parts.json `source`/`notes`).

## EXPORT_DONE.txt
Written last by orchestrator A after `field_sim/tools/check_model.py` passes. Until it exists the
data set is incomplete.
