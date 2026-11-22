"""Final integrity check for the BIOBUZZ field model export (see data/field_model/SCHEMA.md).

Usage: python -I field_sim/tools/check_model.py [field_model_dir]
Exit code 0 = all hard checks pass. Prints one line per check (PASS/FAIL/WARN).
"""
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
FM = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "..", "data", "field_model")
FM = os.path.abspath(FM)

results = []


def rec(ok, name, detail="", hard=True):
    tag = "PASS" if ok else ("FAIL" if hard else "WARN")
    results.append((tag, name, detail))
    print(f"{tag:4} {name}" + (f" :: {detail}" if detail else ""))
    return ok


def load(name):
    with open(os.path.join(FM, name), encoding="utf-8") as f:
        return json.load(f)


def rot_about(axis, pivot, deg):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    t = math.radians(deg)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    R = np.eye(3) + math.sin(t) * K + (1 - math.cos(t)) * K @ K
    M = np.eye(4)
    M[:3, :3] = R
    p = np.asarray(pivot, float)
    M[:3, 3] = p - R @ p
    return M


# ---------------------------------------------------------------- parts + meshes
parts = load("parts.json")["parts"]
rec(len(parts) == 115, "parts.json has 115 parts", f"{len(parts)}")
mesh_bytes = 0
bad_mesh = []
tris = 0
for k, p in parts.items():
    path = os.path.join(FM, p["mesh"])
    try:
        d = np.load(path)
        v, f = d["vertices"], d["faces"]
        ok = (v.dtype == np.float32 and v.ndim == 2 and v.shape[1] == 3 and f.dtype == np.int32
              and f.ndim == 2 and f.shape[1] == 3 and len(f) > 0 and f.min() >= 0 and f.max() < len(v)
              and np.isfinite(v).all())
        if "face_colors" in d.files:
            fc = d["face_colors"]
            ok = ok and fc.dtype == np.uint8 and fc.shape == (len(f), 4)
        if ok:
            bmin, bmax = v.min(0), v.max(0)
            ok = np.allclose(bmin, p["bbox_min"], atol=0.02) and np.allclose(bmax, p["bbox_max"], atol=0.02)
        tris += len(f)
        mesh_bytes += os.path.getsize(path)
        if not ok:
            bad_mesh.append(k)
    except Exception as e:  # noqa: BLE001
        bad_mesh.append(f"{k}({e.__class__.__name__})")
rec(not bad_mesh, "every part mesh loads, valid dtypes/indices, bbox matches parts.json", ",".join(bad_mesh[:10]))
rec(mesh_bytes < 130e6, "meshes total size", f"{mesh_bytes/1e6:.1f} MB, {tris} triangles")
src = {}
for p in parts.values():
    src[p["source"]] = src.get(p["source"], 0) + 1
print(f"     part sources: {src}")

# ---------------------------------------------------------------- model.json
model = load("model.json")
rec(model.get("units") == "in" and model.get("frame") == "pedro", "model.json units/frame")
inst = model["instances"]
dropped = model.get("dropped", [])
total = model.get("stats", {}).get("occurrences_total")
rec(len(inst) + len(dropped) == 1094 and total == 1094, "instances + dropped == 1094 occurrences",
    f"{len(inst)} + {len(dropped)} (stats {total})")
ids = [i["id"] for i in inst] + [d["id"] for d in dropped]
rec(len(set(ids)) == len(ids), "every occurrence appears exactly once")
missing_part = [i["id"] for i in inst if i.get("part") not in parts]
rec(not missing_part, "every instance resolves to a mesh", f"{len(missing_part)} unresolved")
bad_T = []
oob = []
for i in inst:
    T = np.asarray(i["transform"], float)
    if T.shape != (16,):
        bad_T.append(i["id"])
        continue
    T = T.reshape(4, 4)
    R = T[:3, :3]
    if not (np.allclose(R @ R.T, np.eye(3), atol=1e-4) and abs(np.linalg.det(R) - 1) < 1e-4
            and np.allclose(T[3], [0, 0, 0, 1])):
        bad_T.append(i["id"])
    t = T[:3, 3]
    # alliance-area tape lies ~54 in outside the side walls, so allow x in [-80, 224]
    if not (-80 <= t[0] <= 224 and -30 <= t[1] <= 174 and -30 <= t[2] <= 120):
        oob.append(i["id"])
rec(not bad_T, "all transforms are rigid 4x4 row-major", f"{len(bad_T)} bad")
rec(not oob, "all instance origins near the field (pedro inches)", f"{len(oob)} out of range")

CATS = {"tile", "perimeter_wall", "perimeter_pin", "hive_frame", "hive_red", "hive_blue", "apriltag_sticker",
        "flower", "tape", "scoring_pollen", "scoring_nectar_red", "scoring_nectar_blue", "artifact_tray",
        "decoration", "hardware", "other"}
cat = {}
for i in inst:
    cat[i["category"]] = cat.get(i["category"], 0) + 1
rec(set(cat) <= CATS, "categories within the allowed set", str(sorted(set(cat) - CATS)))
print(f"     by category: {dict(sorted(cat.items()))}")
n_pol = cat.get("scoring_pollen", 0)
n_nec = cat.get("scoring_nectar_red", 0) + cat.get("scoring_nectar_blue", 0)
rec(n_pol == 40, "40 pollen instances", str(n_pol))
rec(n_nec == 16, "16 nectar instances in total", f"red {cat.get('scoring_nectar_red', 0)}, blue {cat.get('scoring_nectar_blue', 0)}")
rec(cat.get("apriltag_sticker", 0) == 4, "4 AprilTag sticker instances", str(cat.get("apriltag_sticker", 0)))
rec(cat.get("other", 0) == 0, "no instance left as 'other'", str(cat.get("other", 0)), hard=False)

groups = model.get("groups", {})
for g in ("hive_red", "hive_blue"):
    G = groups.get(g, {})
    ok = all(k in G for k in ("pivot", "axis", "cad_pose_state", "stable_angles_deg")) and \
        set(G.get("stable_angles_deg", {})) == {"audience_up", "far_up"}
    rec(ok, f"groups.{g} complete", json.dumps({k: G.get(k) for k in ("pivot", "cad_pose_state", "stable_angles_deg")}))
    if ok:
        sa = G["stable_angles_deg"]
        rec(abs(abs(sa["audience_up"] - sa["far_up"]) - 60) < 0.1, f"{g} stable states 60 deg apart",
            f"{sa}")
        rec(sum(1 for i in inst if i.get("group") == g) > 10, f"{g} has member instances",
            str(sum(1 for i in inst if i.get("group") == g)))

# ---------------------------------------------------------------- balls / tape
balls = load("ball_start_positions.json")
bc = {}
for b in balls["balls"]:
    bc[b["type"]] = bc.get(b["type"], 0) + 1
rec(bc.get("pollen", 0) == n_pol and bc.get("nectar_red", 0) + bc.get("nectar_blue", 0) == n_nec,
    "ball_start_positions consistent with model", str(bc))
inb = all(-2 <= b["position"][0] <= 146 and -2 <= b["position"][1] <= 146 for b in balls["balls"])
rec(inb, "all ball start positions inside the field footprint", hard=False)
tz = load("tape_zones.json")
rec(len(tz.get("tapes", [])) == cat.get("tape", -1), "tape_zones tapes == tape instances",
    f"{len(tz.get('tapes', []))} vs {cat.get('tape')}")
rec(len(tz.get("zones", [])) > 0, "tape_zones has zones", str(len(tz.get("zones", []))))

# ---------------------------------------------------------------- field constants
fc = load("field_constants.json")
tags = fc.get("apriltags", {})
want = {str(i) for i in range(30, 46)}
rec(set(tags) == want, "AprilTag IDs 30-45 each present exactly once", str(sorted(set(tags) ^ want)))
bad_tag = []
for tid, t in tags.items():
    st = t.get("states", {})
    if set(st) != {"audience_up", "far_up"}:
        bad_tag.append(f"{tid}:states")
        continue
    for s, q in st.items():
        n = np.asarray(q["normal"], float)
        c = np.asarray(q["center"], float)
        C = np.asarray(q["corners"], float)
        if abs(np.linalg.norm(n) - 1) > 1e-3 or C.shape != (4, 3):
            bad_tag.append(f"{tid}/{s}:shape")
            continue
        edges = [np.linalg.norm(C[k] - C[(k + 1) % 4]) for k in range(4)]
        if max(abs(e - 3.25) for e in edges) > 0.05 or np.linalg.norm(C.mean(0) - c) > 0.05:
            bad_tag.append(f"{tid}/{s}:size/centre")
rec(not bad_tag, "16 tags x 2 tip states: unit normals, 3.25 in square, centred", ",".join(bad_tag[:8]))

# Cross-check S3 tag centres against S2 sticker instances + S1 sticker mesh bbox, per tip state.
stick = [i for i in inst if i["category"] == "apriltag_sticker"]
xfail = []
for tid, t in tags.items():
    al = t.get("alliance")
    g = f"hive_{al}"
    G = groups.get(g)
    if not G:
        xfail.append(f"{tid}:nogroup")
        continue
    for s, q in t["states"].items():
        M = rot_about(G["axis"], G["pivot"], G["stable_angles_deg"][s] - G["stable_angles_deg"][G["cad_pose_state"]])
        best = 1e9
        for i in stick:
            if i.get("group") != g:
                continue
            T = M @ np.asarray(i["transform"], float).reshape(4, 4)
            loc = np.linalg.inv(T) @ np.append(np.asarray(q["center"], float), 1)
            bb0, bb1 = np.asarray(parts[i["part"]]["bbox_min"]), np.asarray(parts[i["part"]]["bbox_max"])
            out = np.maximum(0, np.maximum(bb0 - loc[:3], loc[:3] - bb1))
            best = min(best, float(np.linalg.norm(out)))
        if best > 0.1:
            xfail.append(f"{tid}/{s}:{best:.2f}in")
rec(not xfail, "every tag centre lies on a sticker of its alliance hive in that tip state (<0.1 in)",
    ",".join(xfail[:8]))

hv = fc.get("hives", {})
bad_cell = []
for al in ("red", "blue"):
    for s in ("audience_up", "far_up"):
        cells = hv.get(al, {}).get("states", {}).get(s, {}).get("cells", {})
        if set(cells) != {"audience", "far"}:
            bad_cell.append(f"{al}/{s}:cells")
            continue
        ups = [c for c, v in cells.items() if v.get("up")]
        if ups != [s.split("_")[0]]:
            bad_cell.append(f"{al}/{s}:up={ups}")
        for c, v in cells.items():
            # aim point = area centroid of the (pentagonal) aperture; opening_corners = its bounding
            # rectangle. Require the aim point to lie in the opening plane and inside the rectangle.
            C = np.asarray(v["opening_corners"], float)
            a = np.asarray(v["aim_point"], float)
            if C.shape != (4, 3):
                bad_cell.append(f"{al}/{s}/{c}:corners")
                continue
            nrm = np.cross(C[1] - C[0], C[3] - C[0])
            nrm /= np.linalg.norm(nrm)
            inplane = abs(np.dot(a - C[0], nrm)) < 0.05
            inside = np.all(a >= C.min(0) - 0.05) and np.all(a <= C.max(0) + 0.05)
            parallel = abs(abs(np.dot(nrm, np.asarray(v["opening_normal"], float))) - 1) < 1e-3
            if not (inplane and inside and parallel):
                bad_cell.append(f"{al}/{s}/{c}:aim")
rec(not bad_cell, "hive cells: 2 hives x 2 states x 2 cells, correct up cell, aim = opening centre",
    ",".join(bad_cell[:8]))
fl = fc.get("flowers", [])
rec(len(fl) == 4 and all("opening_center" in f for f in fl), "4 flowers with opening centres", str(len(fl)))
prov = fc.get("provenance", {})
bad_prov = [k for k, v in prov.items()
            if not str(v).startswith(("OFFICIAL", "DERIVED-FROM-CAD", "UNVERIFIED"))]
rec(len(prov) > 0 and not bad_prov, "provenance labels valid", f"{len(prov)} entries")

nf = sum(1 for r in results if r[0] == "FAIL")
nw = sum(1 for r in results if r[0] == "WARN")
print(f"\nSUMMARY: {len(results)} checks, {nf} FAIL, {nw} WARN")
sys.exit(1 if nf else 0)
