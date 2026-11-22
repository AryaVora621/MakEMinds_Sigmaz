"""Export the field model into the compact files the WebGL viewer loads.

    .venv\\Scripts\\python.exe tools\\export_web.py

Reads  data/field_model/{model,parts,field_constants,tape_zones,ball_start_positions}.json + meshes/*.npz
Writes web/data/scene.json  (parts, instances, groups, constants)
       web/data/scene.bin   (per part: float32 positions, int8 normals, uint8 RGBA colours; non-indexed)

Each part is split into an OPAQUE and a TRANSPARENT (alpha < 250) mesh. Normals are creased: a corner's normal is the
area-weighted average of the faces that meet at that point AND lie within CREASE_DEG of the corner's own face, so
curved CAD surfaces look smooth while hard edges stay sharp.
"""
import json
import math
import pathlib
import sys

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
DATA = ROOT / "data" / "field_model"
OUT = ROOT / "web" / "data"
CREASE_DEG = 38.0
MAX_GROUP = 24          # faces considered per vertex position
QUANT = 1e-3            # inches, position welding


def creased_normals(tri):
    """tri: (F,3,3) float32 triangle corners. Returns (F,3,3) float32 unit normals per corner."""
    e1 = tri[:, 1] - tri[:, 0]
    e2 = tri[:, 2] - tri[:, 0]
    fn = np.cross(e1, e2)
    area2 = np.linalg.norm(fn, axis=1)
    safe = np.where(area2 > 1e-12, area2, 1.0)
    fu = fn / safe[:, None]                      # unit face normals
    F = len(tri)
    if F == 0:
        return np.zeros((0, 3, 3), np.float32)
    corners = tri.reshape(-1, 3)
    keys = np.round(corners / QUANT).astype(np.int64)
    _, gid = np.unique(keys, axis=0, return_inverse=True)
    gid = gid.reshape(-1)
    order = np.argsort(gid, kind="stable")
    sg = gid[order]
    starts = np.r_[0, np.flatnonzero(np.diff(sg)) + 1]
    counts = np.diff(np.r_[starts, len(sg)])
    rank = np.arange(len(sg)) - np.repeat(starts, counts)
    keep = rank < MAX_GROUP
    ngroups = len(starts)
    table = -np.ones((ngroups, MAX_GROUP), np.int64)
    table[sg[keep], rank[keep]] = order[keep] // 3      # face index of that corner
    cos_c = math.cos(math.radians(CREASE_DEG))
    out = np.zeros((F * 3, 3), np.float32)
    CH = 60000
    for s in range(0, F * 3, CH):
        idx = np.arange(s, min(F * 3, s + CH))
        fidx = table[gid[idx]]                              # (n,K)
        valid = fidx >= 0
        fsafe = np.where(valid, fidx, 0)
        nn = fu[fsafe]                                      # (n,K,3)
        own = fu[idx // 3][:, None, :]
        dot = (nn * own).sum(-1)
        w = np.where(valid & (dot >= cos_c), area2[fsafe], 0.0)
        acc = (nn * w[..., None]).sum(1)
        ln = np.linalg.norm(acc, axis=1)
        res = np.where(ln[:, None] > 1e-12, acc / np.where(ln > 1e-12, ln, 1.0)[:, None], fu[idx // 3])
        out[idx] = res
    return out.reshape(F, 3, 3).astype(np.float32)


def main():
    model = json.loads((DATA / "model.json").read_text(encoding="utf-8"))
    parts_meta = json.loads((DATA / "parts.json").read_text(encoding="utf-8"))["parts"]
    consts = json.loads((DATA / "field_constants.json").read_text(encoding="utf-8"))
    tapes = json.loads((DATA / "tape_zones.json").read_text(encoding="utf-8"))
    balls = json.loads((DATA / "ball_start_positions.json").read_text(encoding="utf-8"))

    used = sorted({i["part"] for i in model["instances"]})
    OUT.mkdir(parents=True, exist_ok=True)
    binf = open(OUT / "scene.bin", "wb")
    pos = 0
    parts_out = {}

    def write(arr):
        nonlocal pos
        b = np.ascontiguousarray(arr).tobytes()
        binf.write(b)
        off = pos
        pos += len(b)
        pad = (-pos) % 4
        if pad:
            binf.write(b"\0" * pad)
            pos += pad
        return off

    tri_total = 0
    for pk in used:
        npz = np.load(DATA / "meshes" / (pk + ".npz"))
        v = npz["vertices"].astype(np.float32)
        f = npz["faces"].astype(np.int64)
        if len(f) == 0:
            continue
        meta = parts_meta.get(pk, {})
        dc = meta.get("default_color") or [180, 180, 180, 255]
        fc = npz["face_colors"] if "face_colors" in npz.files else np.tile(np.array(dc, np.uint8), (len(f), 1))
        if fc.shape[1] == 3:
            fc = np.concatenate([fc, np.full((len(f), 1), 255, np.uint8)], axis=1)
        tri = v[f]                                         # (F,3,3)
        nrm = creased_normals(tri)
        entry = {"name": meta.get("name", pk), "source": meta.get("source", "")}
        for label, mask in (("opaque", fc[:, 3] >= 250), ("trans", fc[:, 3] < 250)):
            if not mask.any():
                continue
            t = tri[mask]
            n = nrm[mask]
            c = np.repeat(fc[mask][:, None, :], 3, axis=1).astype(np.uint8)
            entry[label] = {
                "count": int(len(t) * 3),
                "pos": write(t.reshape(-1, 3).astype(np.float32)),
                "nrm": write(np.round(n.reshape(-1, 3) * 127).astype(np.int8)
                             if False else (n.reshape(-1, 3) * 127).round().astype(np.int8)),
                "col": write(c.reshape(-1, 4)),
            }
            tri_total += int(mask.sum())
        parts_out[pk] = entry
    binf.close()

    instances = []
    for i in model["instances"]:
        instances.append({
            "id": i["id"], "name": i["name"], "part": i["part"], "category": i["category"],
            "sub": i.get("subcategory"), "group": i.get("group"), "alliance": i.get("alliance"),
            "cell": i.get("cell"), "m": [round(x, 5) for x in i["transform"]],
        })

    scene = {
        "schema": 1, "units": "in", "frame": "pedro",
        "parts": parts_out, "instances": instances, "groups": model["groups"],
        "constants": {k: consts[k] for k in ("field", "hives", "apriltags", "flowers", "zones") if k in consts},
        "tapes": tapes, "balls": balls,
    }
    (OUT / "scene.json").write_text(json.dumps(scene, separators=(",", ":")), encoding="utf-8")
    print("parts", len(parts_out), "instances", len(instances), "triangles", tri_total,
          "bin MB %.1f" % (pos / 1e6))


if __name__ == "__main__":
    sys.exit(main())
