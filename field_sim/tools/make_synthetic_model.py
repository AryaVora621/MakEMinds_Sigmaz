"""Generate a SYNTHETIC mini field model that follows the field_model data contract.

Purpose: lets the viewer, the Field API and the tests run before the real CAD export
(field_sim/data/field_model/, written by the extract pipeline) exists.  Geometry is
deliberately crude (boxes, spheres, cylinders); only the layout logic is meaningful.

Output (never touches data/field_model/):
    field_sim/data/synthetic_model/
        model.json, parts.json, field_constants.json, tape_zones.json,
        ball_start_positions.json, meshes/<partkey>.npz, SCHEMA_SYNTHETIC.txt

Frame: PEDRO inches. Origin audience-left corner, +x right, +y toward far wall, +z up.
Red alliance at low x, blue at high x.

Hive convention used here (and by sim.hive):
    group pivot = point on the tilt axis, axis = unit vector (here +x).
    Angle theta (deg) is a right-hand rotation about `axis` through `pivot`
    applied to the hive in its LEVEL (theta = 0) pose.  With axis = +x a
    positive theta raises the far (+y) side.  cad_pose_angle_deg is the angle
    the exported geometry is already in.  stable_angles_deg = [-30, +30].
    Red CAD pose: audience cell up  -> -30.  Blue CAD pose: far cell up -> +30.

Usage:  python tools/make_synthetic_model.py [--out DIR]
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUT = ROOT / "data" / "synthetic_model"

PIVOT_Z = 43.95
PIVOT_Y = 72.0
HIVE_X = {"red": 59.25, "blue": 84.75}
CELL_OFFSET = 9.42           # cell centre offset from pivot along y (level pose)
BODY = (20.0, 38.0, 6.0)     # hive body size x, y, z (level pose)
TILT = 30.0
CAD_ANGLE = {"red": -TILT, "blue": +TILT}
BALL_D = 5.0

COLORS = {
    "tile": (90, 90, 90, 255),
    "perimeter_wall": (200, 200, 210, 120),
    "hive_frame": (60, 60, 60, 255),
    "hive_red": (200, 40, 40, 255),
    "hive_blue": (40, 70, 200, 255),
    "apriltag_sticker": (245, 245, 245, 255),
    "flower": (240, 200, 40, 255),
    "scoring_pollen": (250, 230, 120, 255),
    "scoring_nectar_red": (220, 30, 30, 255),
    "scoring_nectar_blue": (30, 60, 220, 255),
}


# ---------------------------------------------------------------- mesh helpers
def box_mesh(sx, sy, sz):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    v = np.array([[x, y, z] for z in (-hz, hz) for y in (-hy, hy) for x in (-hx, hx)], np.float32)
    # vertex index = zi*4 + yi*2 + xi
    quads = [(0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5)]
    f = []
    for a, b, c, d in quads:
        f += [(a, b, c), (a, c, d)]
    return v, np.array(f, np.int32)


def sphere_mesh(d, n_lat=10, n_lon=16):
    r = d / 2
    verts = [(0, 0, r)]
    for i in range(1, n_lat):
        th = math.pi * i / n_lat
        for j in range(n_lon):
            ph = 2 * math.pi * j / n_lon
            verts.append((r * math.sin(th) * math.cos(ph), r * math.sin(th) * math.sin(ph), r * math.cos(th)))
    verts.append((0, 0, -r))
    faces = []
    for j in range(n_lon):
        faces.append((0, 1 + j, 1 + (j + 1) % n_lon))
    for i in range(n_lat - 2):
        a0 = 1 + i * n_lon
        b0 = a0 + n_lon
        for j in range(n_lon):
            j1 = (j + 1) % n_lon
            faces += [(a0 + j, b0 + j, b0 + j1), (a0 + j, b0 + j1, a0 + j1)]
    last = len(verts) - 1
    base = 1 + (n_lat - 2) * n_lon
    for j in range(n_lon):
        faces.append((base + j, last, base + (j + 1) % n_lon))
    return np.array(verts, np.float32), np.array(faces, np.int32)


def cylinder_mesh(d, h, n=16):
    r = d / 2
    ring = [(r * math.cos(2 * math.pi * k / n), r * math.sin(2 * math.pi * k / n)) for k in range(n)]
    v = [(x, y, 0.0) for x, y in ring] + [(x, y, h) for x, y in ring] + [(0, 0, 0), (0, 0, h)]
    f = []
    for k in range(n):
        k1 = (k + 1) % n
        f += [(k, k1, n + k1), (k, n + k1, n + k)]
        f.append((2 * n, k1, k))
        f.append((2 * n + 1, n + k, n + k1))
    return np.array(v, np.float32), np.array(f, np.int32)


# ------------------------------------------------------------ transform helpers
def translate(x, y, z):
    m = np.eye(4)
    m[:3, 3] = (x, y, z)
    return m


def rot_x(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    m = np.eye(4)
    m[1:3, 1:3] = [[c, -s], [s, c]]
    return m


def rot_z(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    m = np.eye(4)
    m[0:2, 0:2] = [[c, -s], [s, c]]
    return m


def hive_pose(alliance):
    """Level-pose -> CAD-pose transform for a hive (rotation about the pivot axis)."""
    p = translate(HIVE_X[alliance], PIVOT_Y, PIVOT_Z)
    return p @ rot_x(CAD_ANGLE[alliance]) @ np.linalg.inv(p)


def apply(m, p):
    return (m @ np.append(np.asarray(p, float), 1.0))[:3].tolist()


def apply_dir(m, d):
    return (m[:3, :3] @ np.asarray(d, float)).tolist()


# ---------------------------------------------------------------------- build
def build(out: Path):
    (out / "meshes").mkdir(parents=True, exist_ok=True)
    parts, instances = {}, []

    def add_part(key, v, f, category):
        col = np.tile(np.array(COLORS.get(category, (150, 150, 150, 255)), np.uint8), (len(f), 1))
        np.savez_compressed(out / "meshes" / f"{key}.npz", vertices=v, faces=f, face_colors=col)
        lo, hi = v.min(0), v.max(0)
        parts[key] = {"key": key, "name": key, "mesh": f"meshes/{key}.npz",
                      "n_vertices": int(len(v)), "n_faces": int(len(f)),
                      "bbox_min": lo.tolist(), "bbox_max": hi.tolist(),
                      "color": list(COLORS.get(category, (150, 150, 150, 255)))}

    def add_inst(name, category, part, m, group=None, alliance=None):
        instances.append({"id": f"syn{len(instances):04d}", "name": name, "path": f"Synthetic/{name}",
                          "category": category, "group": group, "alliance": alliance, "part": part,
                          "transform": [float(x) for x in np.asarray(m).reshape(-1)]})

    # tiles 6x6
    add_part("tile_24", *box_mesh(24, 24, 0.5), "tile")
    for i in range(6):
        for j in range(6):
            add_inst(f"Tile_{i}{j}", "tile", "tile_24", translate(12 + 24 * i, 12 + 24 * j, -0.25))
    # walls
    add_part("wall_long", *box_mesh(144, 1, 12), "perimeter_wall")
    add_inst("Wall_audience", "perimeter_wall", "wall_long", translate(72, -0.5, 6))
    add_inst("Wall_far", "perimeter_wall", "wall_long", translate(72, 144.5, 6))
    add_inst("Wall_left", "perimeter_wall", "wall_long", translate(-0.5, 72, 6) @ rot_z(90))
    add_inst("Wall_right", "perimeter_wall", "wall_long", translate(144.5, 72, 6) @ rot_z(90))

    # hive stand (static) + rotating bodies, cells, stickers
    add_part("hive_stand", *box_mesh(4, 30, PIVOT_Z - 3), "hive_frame")
    add_part("hive_body_red", *box_mesh(*BODY), "hive_red")
    add_part("hive_body_blue", *box_mesh(*BODY), "hive_blue")
    add_part("hive_cell_rim", *box_mesh(8, 8, 1.0), "hive_frame")
    add_part("tag_sticker", *box_mesh(17, 0.05, 5), "apriltag_sticker")
    constants = {"schema": 1, "synthetic": True, "field_size_in": 144.0, "tile_in": 24.0,
                 "ball_diameter_in": BALL_D, "tilt_deg": TILT, "hives": {}, "tags": {}}
    groups = {}
    tag_ids = {("red", "far"): 30, ("red", "audience"): 34, ("blue", "audience"): 38, ("blue", "far"): 42}
    for al in ("red", "blue"):
        g = f"hive_{al}"
        hx = HIVE_X[al]
        groups[g] = {"pivot": [hx, PIVOT_Y, PIVOT_Z], "axis": [1.0, 0.0, 0.0],
                     "cad_pose_angle_deg": CAD_ANGLE[al], "stable_angles_deg": [-TILT, TILT]}
        add_inst(f"HiveStand_{al}", "hive_frame", "hive_stand",
                 translate(hx, PIVOT_Y, (PIVOT_Z - 3) / 2), alliance=al)
        H = hive_pose(al)
        add_inst(f"HiveBody_{al}", g, f"hive_body_{al}", H @ translate(hx, PIVOT_Y, PIVOT_Z), group=g, alliance=al)
        cells = {}
        for side, sgn in (("audience", -1), ("far", +1)):
            cy = PIVOT_Y + sgn * CELL_OFFSET
            top = PIVOT_Z + BODY[2] / 2
            add_inst(f"HiveCell_{al}_{side}", g, "hive_cell_rim", H @ translate(hx, cy, top + 0.5),
                     group=g, alliance=al)
            cells[side] = {"opening_center": apply(H, (hx, cy, top + 1.0)),
                           "opening_normal": apply_dir(H, (0, 0, 1)),
                           "opening_size_in": [8.0, 8.0]}
            # sticker on the end face, normal pointing out along +-y
            ey = PIVOT_Y + sgn * (BODY[1] / 2 + 0.05)
            add_inst(f"TagSticker_{al}_{side}", "apriltag_sticker", "tag_sticker",
                     H @ translate(hx, ey, PIVOT_Z), group=g, alliance=al)
            first = tag_ids[(al, side)]
            for k in range(4):
                tx = hx - 6.375 + 4.25 * k
                constants["tags"][str(first + k)] = {
                    "id": first + k, "group": g, "alliance": al, "side": side, "size_in": 3.25,
                    "center": apply(H, (tx, ey + sgn * 0.03, PIVOT_Z)),
                    "normal": apply_dir(H, (0, sgn, 0)),
                    "up": apply_dir(H, (0, 0, 1))}
        constants["hives"][al] = {"group": g, "pivot": groups[g]["pivot"], "axis": [1.0, 0.0, 0.0],
                                  "pose": "cad", "cells": cells}

    # flowers (4)
    add_part("flower", *cylinder_mesh(6, 10), "flower")
    for k, (fx, fy) in enumerate([(24, 24), (120, 24), (24, 120), (120, 120)]):
        add_inst(f"Flower_{k}", "flower", "flower", translate(fx, fy, 0), alliance="red" if fx < 72 else "blue")

    # balls: 40 pollen, 8 red nectar, 8 blue nectar
    add_part("pollen", *sphere_mesh(BALL_D), "scoring_pollen")
    add_part("nectar_red", *sphere_mesh(BALL_D), "scoring_nectar_red")
    add_part("nectar_blue", *sphere_mesh(BALL_D), "scoring_nectar_blue")
    balls = []
    for k in range(40):
        x = 40 + 6 * (k % 10) + 5
        y = 6 + 6 * (k // 10)
        balls.append(("pollen", "scoring_pollen", None, (x, y, BALL_D / 2)))
    for k in range(8):
        balls.append(("nectar_red", "scoring_nectar_red", "red", (6 + 6 * (k % 2), 60 + 6 * (k // 2), BALL_D / 2)))
        balls.append(("nectar_blue", "scoring_nectar_blue", "blue", (132 + 6 * (k % 2), 60 + 6 * (k // 2), BALL_D / 2)))
    ball_json = []
    for n, (part, cat, al, c) in enumerate(balls):
        add_inst(f"Ball_{part}_{n}", cat, part, translate(*c), alliance=al)
        ball_json.append({"kind": part, "category": cat, "alliance": al, "center": list(c), "diameter_in": BALL_D})

    zones = {"schema": 1, "zones": [
        {"name": "red_base", "alliance": "red", "color": [220, 30, 30], "polygon": [[0, 0], [24, 0], [24, 24], [0, 24]]},
        {"name": "blue_base", "alliance": "blue", "color": [30, 60, 220], "polygon": [[120, 0], [144, 0], [144, 24], [120, 24]]},
        {"name": "center_zone", "alliance": None, "color": [240, 240, 240], "polygon": [[48, 48], [96, 48], [96, 96], [48, 96]]},
    ]}

    model = {"schema": 1, "units": "in", "frame": "pedro", "source": "synthetic",
             "parts": parts, "instances": instances, "groups": groups}
    (out / "model.json").write_text(json.dumps(model, indent=1))
    (out / "parts.json").write_text(json.dumps(parts, indent=1))
    (out / "field_constants.json").write_text(json.dumps(constants, indent=1))
    (out / "tape_zones.json").write_text(json.dumps(zones, indent=1))
    (out / "ball_start_positions.json").write_text(json.dumps({"schema": 1, "balls": ball_json}, indent=1))
    (out / "SCHEMA_SYNTHETIC.txt").write_text(__doc__)
    print(f"wrote {len(instances)} instances, {len(parts)} parts, {len(constants['tags'])} tags -> {out}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=DEFAULT_OUT)
    build(ap.parse_args().out)
