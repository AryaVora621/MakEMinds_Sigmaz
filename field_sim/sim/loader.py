"""Model loader for the FTC field simulator.

Reads a model directory with this layout (see tools/make_synthetic_model.py):

    model.json                 schema, units, frame, source, parts, instances, groups
    parts.json                 optional, per-part metadata (mesh path, bbox, colour)
    field_constants.json       optional, hive/tag constants
    tape_zones.json            optional, named polygons
    ball_start_positions.json  optional, ball start list
    meshes/<key>.npz           arrays: vertices (N,3), faces (M,3), face_colors (M,4) uint8

Frame: PEDRO inches (origin audience-left corner, +x right, +y far wall, +z up).
Instance transforms are 4x4 float64, part-local -> Pedro world, column-vector convention.

load_model() never loads meshes. FieldModel.get_mesh(key) loads and memoises one
mesh on first use.
"""
from __future__ import annotations

import hashlib
import json
import os
import pickle
import sys
import warnings
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DATA_DIR = ROOT / "data"
CACHE_DIR = DATA_DIR / "cache"
ENV_MODEL_DIR = "FIELD_SIM_MODEL_DIR"
CACHE_VERSION = 2

# Ordered list of the 16 categories the viewer knows about.  The first 10 are
# the categories the synthetic generator emits; the rest are plausible
# categories for the real export.  Unknown categories map to "other".
CATEGORIES: tuple[str, ...] = (
    "tile",
    "perimeter_wall",
    "trim",
    "tape",
    "structure",
    "hive_frame",
    "hive_red",
    "hive_blue",
    "apriltag_sticker",
    "flower",
    "scoring_pollen",
    "scoring_nectar_red",
    "scoring_nectar_blue",
    "hardware",
    "decoration",
    "other",
)
_CATEGORY_SET = frozenset(CATEGORIES)
_FALLBACK_CATEGORY = "other"

CATEGORY_PALETTE: dict[str, tuple[int, int, int]] = {
    "tile": (90, 90, 90),
    "perimeter_wall": (200, 200, 210),
    "trim": (120, 120, 130),
    "tape": (240, 240, 240),
    "structure": (110, 110, 120),
    "hive_frame": (60, 60, 60),
    "hive_red": (200, 40, 40),
    "hive_blue": (40, 70, 200),
    "apriltag_sticker": (245, 245, 245),
    "flower": (240, 200, 40),
    "scoring_pollen": (250, 230, 120),
    "scoring_nectar_red": (220, 30, 30),
    "scoring_nectar_blue": (30, 60, 220),
    "hardware": (150, 150, 150),
    "decoration": (170, 140, 100),
    "other": (128, 128, 128),
}

_VERTEX_KEYS = ("vertices", "verts", "v")
_FACE_KEYS = ("faces", "f", "triangles")
_COLOR_KEYS = ("face_colors", "colors")
# Files whose stat() feeds the cache key.
_CACHE_KEY_FILES = (
    "model.json",
    "parts.json",
    "field_constants.json",
    "tape_zones.json",
    "ball_start_positions.json",
)


# ---------------------------------------------------------------- data types
@dataclass(eq=False)
class Instance:
    id: str
    name: str
    path: str
    category: str
    group: str | None
    alliance: str | None
    part: str | None
    transform: np.ndarray  # (4, 4) float64


@dataclass(eq=False)
class Mesh:
    vertices: np.ndarray            # (N, 3) float32
    faces: np.ndarray               # (M, 3) int32
    face_colors: np.ndarray | None  # (M, 4) uint8 or None


@dataclass(eq=False)
class FieldModel:
    root: Path
    meta: dict
    parts: dict
    instances: list[Instance]
    groups: dict
    meshes: dict[str, Mesh]
    constants: dict
    tape_zones: dict
    ball_starts: dict

    def mesh_path(self, key: str) -> Path:
        """Path of the npz for a part/mesh key (parts[key]['mesh'] if given)."""
        part = self.parts.get(key)
        rel = part.get("mesh") if isinstance(part, dict) else None
        if rel:
            return self.root / rel
        return self.root / "meshes" / f"{key}.npz"

    def get_mesh(self, key: str) -> Mesh:
        """Return the mesh for `key`, loading and memoising it on first use."""
        mesh = self.meshes.get(key)
        if mesh is None:
            mesh = _read_mesh_npz(self.mesh_path(key), key)
            self.meshes[key] = mesh
        return mesh


# ------------------------------------------------------------ mesh reading
def _pick_array(npz: Any, names: tuple[str, ...], what: str, path: Path) -> np.ndarray:
    for name in names:
        if name in npz.files:
            return npz[name]
    raise KeyError(f"{path}: no {what} array (tried {names}, file has {npz.files})")


def _normalise_colors(colors: Any, n_faces: int, path: Path) -> np.ndarray | None:
    if colors is None:
        return None
    c = np.asarray(colors)
    if c.ndim != 2 or c.shape[0] != n_faces or c.shape[1] not in (3, 4):
        warnings.warn(f"{path}: face colours shape {c.shape} does not match {n_faces} faces; ignoring")
        return None
    if c.dtype != np.uint8:
        if np.issubdtype(c.dtype, np.floating) and c.size and float(c.max()) <= 1.0:
            c = c * 255.0
        c = np.clip(c, 0, 255).astype(np.uint8)
    if c.shape[1] == 3:
        alpha = np.full((n_faces, 1), 255, dtype=np.uint8)
        c = np.hstack([c, alpha])
    return np.ascontiguousarray(c)


def _read_mesh_npz(path: Path, key: str) -> Mesh:
    if not path.is_file():
        raise FileNotFoundError(f"no mesh file for key {key!r}: {path}")
    with np.load(path, allow_pickle=False) as npz:
        verts = np.asarray(_pick_array(npz, _VERTEX_KEYS, "vertices", path), dtype=np.float32)
        faces = np.asarray(_pick_array(npz, _FACE_KEYS, "faces", path), dtype=np.int32)
        colors = None
        for name in _COLOR_KEYS:
            if name in npz.files:
                colors = npz[name]
                break
    if verts.ndim != 2 or verts.shape[1] != 3:
        raise ValueError(f"{path}: vertices must be (N,3), got {verts.shape}")
    if faces.ndim != 2 or faces.shape[1] != 3:
        raise ValueError(f"{path}: faces must be (M,3), got {faces.shape}")
    if faces.size and (int(faces.min()) < 0 or int(faces.max()) >= len(verts)):
        raise ValueError(f"{path}: face indices out of range for {len(verts)} vertices")
    return Mesh(
        vertices=np.ascontiguousarray(verts),
        faces=np.ascontiguousarray(faces),
        face_colors=_normalise_colors(colors, len(faces), path),
    )


# ------------------------------------------------------------ json helpers
def _read_json(path: Path, required: bool) -> Any:
    if not path.is_file():
        if required:
            raise FileNotFoundError(f"required model file missing: {path}")
        return None
    with path.open("r", encoding="utf-8") as fh:
        return json.load(fh)


def _optional_dict(path: Path) -> dict:
    data = _read_json(path, required=False)
    if data is None:
        return {}
    if isinstance(data, dict):
        return data
    return {"items": data}


def _keyed(obj: Any, key_field: str = "key") -> dict:
    """Accept either a dict or a list of dicts carrying `key_field`."""
    if isinstance(obj, dict):
        return obj
    if isinstance(obj, list):
        return {str(e[key_field]): e for e in obj if isinstance(e, dict) and key_field in e}
    return {}


def _opt_str(value: Any) -> str | None:
    return None if value is None else str(value)


def _parse_instance(raw: dict, index: int) -> Instance:
    try:
        transform = np.asarray(raw["transform"], dtype=np.float64)
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"instance {index}: missing or non-numeric transform") from exc
    if transform.size != 16:
        raise ValueError(f"instance {index}: transform has {transform.size} values, expected 16")
    transform = np.array(transform.reshape(4, 4), dtype=np.float64)  # row-major, owned copy

    category = raw.get("category")
    if category not in _CATEGORY_SET:
        category = _FALLBACK_CATEGORY
    raw_id = raw.get("id")
    return Instance(
        id=str(index) if raw_id is None else str(raw_id),
        name=str(raw.get("name") or ""),
        path=("/".join(str(p) for p in raw["path"]) if isinstance(raw.get("path"), list)
              else str(raw.get("path") or "")),
        category=category,
        group=_opt_str(raw.get("group")),
        alliance=_opt_str(raw.get("alliance")),
        part=_opt_str(raw.get("part")),
        transform=transform,
    )


# ------------------------------------------------------------- model loading
def default_model_dir() -> Path:
    """FIELD_SIM_MODEL_DIR if set; else the real export when EXPORT_DONE.txt exists; else synthetic."""
    env = os.environ.get(ENV_MODEL_DIR, "").strip()
    if env:
        return Path(env).expanduser()
    real = DATA_DIR / "field_model"
    if (real / "EXPORT_DONE.txt").is_file():
        return real
    return DATA_DIR / "synthetic_model"


def _load_fresh(model_dir: Path) -> dict:
    """Parse the model directory from its json files.  Returns FieldModel kwargs (minus root/meshes)."""
    model = _read_json(model_dir / "model.json", required=True)
    if not isinstance(model, dict):
        raise ValueError(f"{model_dir / 'model.json'}: top level must be an object")

    parts = _keyed(model.get("parts") or {})
    if not parts:
        parts = _keyed(_read_json(model_dir / "parts.json", required=False) or {})

    raw_instances = model.get("instances") or []
    instances = [_parse_instance(r, i) for i, r in enumerate(raw_instances)]

    meta = {k: v for k, v in model.items() if k not in ("parts", "instances", "groups")}
    groups = {k: _normalise_group(v) for k, v in (model.get("groups") or {}).items()}
    return {
        "meta": meta,
        "parts": parts,
        "instances": instances,
        "groups": groups,
        "constants": _normalise_constants(_optional_dict(model_dir / "field_constants.json")),
        "tape_zones": _optional_dict(model_dir / "tape_zones.json"),
        "ball_starts": _optional_dict(model_dir / "ball_start_positions.json"),
    }


# ----------------------------------------------- real-export schema adapter
# The real export (data/field_model/SCHEMA.md) uses state names "audience_up"/"far_up",
# stable_angles_deg as a dict, and per-state tag/cell poses under "apriltags" and
# hives[al]["states"].  These helpers convert it to the internal form used by sim.hive
# and sim.field (stable_angles_deg list; constants["tags"] / hives[al]["cells"] at CAD pose)
# while keeping the exact per-state data in "states" so Field can use it directly.
_STATE_ALIAS = {"audience_up": "audience", "far_up": "far", "audience": "audience", "far": "far"}


def _normalise_group(g: Any) -> Any:
    if not isinstance(g, dict):
        return g
    g = dict(g)
    sa = g.get("stable_angles_deg")
    if isinstance(sa, dict):
        g["stable_angles_by_state"] = {_STATE_ALIAS.get(k, k): float(v) for k, v in sa.items()}
        g["stable_angles_deg"] = [float(v) for v in sa.values()]
    if "cad_pose_state" in g:
        g["cad_pose_state"] = _STATE_ALIAS.get(g["cad_pose_state"], g["cad_pose_state"])
    return g


def _normalise_constants(c: dict) -> dict:
    if not isinstance(c, dict) or ("apriltags" not in c and not any(
            isinstance(h, dict) and "states" in h for h in (c.get("hives") or {}).values())):
        return c
    c = dict(c)
    hives = {}
    for al, h in (c.get("hives") or {}).items():
        h = dict(h)
        states = {_STATE_ALIAS.get(k, k): v for k, v in (h.get("states") or {}).items()}
        cad = _STATE_ALIAS.get(h.get("cad_pose_state", ""), None)
        cells = {}
        for side in ("audience", "far"):
            # the cell named S is the up cell in state S; record it in that state
            cell = (states.get(side, {}).get("cells") or {}).get(side)
            if cell:
                cells[side] = dict(cell, opening_center=cell.get("aim_point"), state=side)
        h["states"] = states
        h["cells"] = cells
        h["group"] = h.get("group", f"hive_{al}")
        h["pose"] = "per_state"
        h["cad_state"] = cad
        hives[al] = h
    c["hives"] = hives
    tags = {}
    for tid, t in (c.get("apriltags") or {}).items():
        st = {_STATE_ALIAS.get(k, k): v for k, v in (t.get("states") or {}).items()}
        al = t.get("alliance")
        cad = (hives.get(al) or {}).get("cad_state") or next(iter(st), None)
        base = st.get(cad, {})
        tags[str(tid)] = dict(t, id=int(tid), group=t.get("group", f"hive_{al}" if al else None),
                              states=st, center=base.get("center"), normal=base.get("normal"),
                              up=base.get("up"), right=base.get("right"))
    c["tags"] = tags
    return c


def _cache_path(model_dir: Path) -> Path:
    digest = hashlib.sha1(str(model_dir).encode("utf-8")).hexdigest()[:12]
    return CACHE_DIR / f"model_{digest}.pkl"


def _cache_key(model_dir: Path) -> str | None:
    """Key from the resolved dir plus mtime/size of each model file.  None if it cannot be computed."""
    try:
        pieces = [str(model_dir.resolve())]
        for name in _CACHE_KEY_FILES:
            try:
                st = (model_dir / name).stat()
                pieces.append(f"{name}:{st.st_mtime_ns}:{st.st_size}")
            except OSError:
                pieces.append(f"{name}:-")
        return "|".join(pieces)
    except Exception:
        return None


def _try_load_cache(model_dir: Path, key: str) -> dict | None:
    try:
        with _cache_path(model_dir).open("rb") as fh:
            payload = pickle.load(fh)
    except Exception:  # missing, unreadable, corrupt, or from an incompatible code version
        return None
    if not isinstance(payload, dict):
        return None
    if payload.get("version") != CACHE_VERSION or payload.get("key") != key:
        return None
    fields = payload.get("fields")
    return fields if isinstance(fields, dict) else None


def _save_cache(model_dir: Path, key: str, fields: dict) -> None:
    tmp: Path | None = None
    try:
        CACHE_DIR.mkdir(parents=True, exist_ok=True)
        path = _cache_path(model_dir)
        tmp = path.with_name(f"{path.name}.{os.getpid()}.tmp")
        payload = {"version": CACHE_VERSION, "key": key, "fields": fields}
        with tmp.open("wb") as fh:
            pickle.dump(payload, fh, protocol=pickle.HIGHEST_PROTOCOL)
        os.replace(tmp, path)
    except Exception:  # unwritable cache dir is not an error
        if tmp is not None:
            try:
                tmp.unlink()
            except OSError:
                pass


def load_model(model_dir: Path | None = None, use_cache: bool = True) -> FieldModel:
    """Load a field model.  Meshes are NOT loaded here; see FieldModel.get_mesh."""
    model_dir = Path(model_dir).expanduser() if model_dir is not None else default_model_dir()
    model_dir = model_dir.resolve()

    key = _cache_key(model_dir) if use_cache else None
    if key is not None:
        try:
            fields = _try_load_cache(model_dir, key)
            if fields is not None:
                return FieldModel(root=model_dir, meshes={}, **fields)
        except Exception:  # stale or malformed cache entry: fall back to a fresh parse
            pass

    fields = _load_fresh(model_dir)
    if key is not None:
        _save_cache(model_dir, key, fields)
    return FieldModel(root=model_dir, meshes={}, **fields)


def iter_meshes_by_category(model: FieldModel) -> dict[tuple[str, str | None], list[tuple[str, list[Instance]]]]:
    """Group instances as {(category, group): [(mesh_key, [Instance, ...]), ...]}.

    Instances with no part (no mesh) are skipped.
    """
    grouped: dict[tuple[str, str | None], dict[str, list[Instance]]] = {}
    for inst in model.instances:
        if inst.part is None:
            continue
        by_part = grouped.setdefault((inst.category, inst.group), {})
        by_part.setdefault(inst.part, []).append(inst)
    return {k: list(v.items()) for k, v in grouped.items()}


# --------------------------------------------------------------- self-check
def _self_check() -> int:
    model_dir = default_model_dir()
    print(f"model dir: {model_dir}")

    model = load_model(model_dir)
    fresh = load_model(model_dir, use_cache=False)
    cache_file = _cache_path(model_dir.resolve())
    print(f"cache file: {cache_file} (exists: {cache_file.is_file()})")

    assert len(model.instances) == len(fresh.instances), "cached and fresh instance counts differ"
    assert model.parts.keys() == fresh.parts.keys(), "cached and fresh part keys differ"

    print(f"parts: {len(model.parts)}  instances: {len(model.instances)}  groups: {sorted(model.groups)}")
    print(f"meta keys: {sorted(model.meta)}")
    print(f"constants: {len(model.constants)} keys  tape_zones: {len(model.tape_zones)} keys  "
          f"ball_starts: {len(model.ball_starts)} keys")
    assert set(CATEGORY_PALETTE) == set(CATEGORIES), "palette and CATEGORIES disagree"
    assert len(CATEGORIES) == 16, "expected 16 categories"

    for inst in model.instances:
        assert inst.transform.shape == (4, 4), f"{inst.id}: transform shape {inst.transform.shape}"
        assert inst.transform.dtype == np.float64, f"{inst.id}: transform dtype {inst.transform.dtype}"

    print("category counts:")
    for cat, n in sorted(Counter(i.category for i in model.instances).items()):
        print(f"  {cat:22s} {n}")

    print("iter_meshes_by_category:")
    for (cat, grp), items in sorted(iter_meshes_by_category(model).items(), key=lambda kv: str(kv[0])):
        n_inst = sum(len(insts) for _, insts in items)
        print(f"  {cat:22s} group={str(grp):10s} mesh_keys={len(items)} instances={n_inst}")

    n_tri = 0
    for inst in model.instances:
        if inst.part is None:
            continue
        mesh = model.get_mesh(inst.part)
        assert model.get_mesh(inst.part) is mesh, "get_mesh is not memoised"
        n_tri += len(mesh.faces)
    print(f"all instance meshes loaded; mesh cache holds {len(model.meshes)} meshes, {n_tri} triangles total")
    print("self-check OK")
    return 0


if __name__ == "__main__":
    # Import through the package so cache pickles reference sim.loader, not __main__.
    sys.path.insert(0, str(ROOT))
    from sim import loader as _loader

    raise SystemExit(_loader._self_check())
