"""Integrity checks for the field model (synthetic or real).

Counts are FTC game rules and are asserted exactly.  Geometry checks cover
finite values, part/mesh references, rigid transforms and the field envelope.
"""
from __future__ import annotations

from collections import Counter

import numpy as np
import pytest

TAG_IDS = list(range(30, 46))  # 16 AprilTags, IDs 30..45
FIELD_XY = (-12.0, 156.0)      # world x/y envelope (inches, Pedro frame)
FIELD_Z = (-2.0, 100.0)        # world z envelope (inches)
OUTSIDE_XY = (-60.0, 204.0)    # alliance areas (54 in deep) outside the walls
ORTHO_TOL = 1e-4

EXPECTED_COUNTS = [
    ("scoring_pollen", 40),
    ("scoring_nectar_red", 8),
    ("scoring_nectar_blue", 8),
    ("flower", 4),
]


# ----------------------------------------------------------------- helpers
def _get_mesh(model, key):
    if hasattr(model, "get_mesh"):
        return model.get_mesh(key)
    return model.meshes[key]


def _referenced_parts(model):
    return sorted({inst.part for inst in model.instances})


@pytest.fixture(scope="session")
def part_bboxes(model):
    """Map part key -> (min xyz, max xyz) computed from mesh vertices."""
    boxes = {}
    for key in _referenced_parts(model):
        verts = np.asarray(_get_mesh(model, key).vertices, dtype=float)
        boxes[key] = (verts.min(axis=0), verts.max(axis=0))
    return boxes


# ------------------------------------------------------------------- tags
def test_tag_ids_are_exactly_30_to_45(model):
    tags = model.constants["tags"]
    assert len(tags) == 16
    assert sorted(int(k) for k in tags) == TAG_IDS


def test_tag_entries_are_self_consistent(model):
    tags = model.constants["tags"]
    group_names = set(model.groups)
    for key, tag in tags.items():
        assert int(tag["id"]) == int(key), f"tag key {key} has id {tag['id']}"
        assert tag["group"] in group_names, f"tag {key} references unknown group {tag['group']}"


def test_field_tag_ids_match_30_to_45(field):
    assert sorted(field.tag_ids()) == TAG_IDS


@pytest.mark.real_model
def test_real_model_tag_axes_are_orthonormal(model):
    for key, tag in model.constants["tags"].items():
        up = np.asarray(tag["up"], float)
        normal = np.asarray(tag["normal"], float)
        assert abs(np.linalg.norm(up) - 1.0) < ORTHO_TOL, f"tag {key} up not unit"
        assert abs(np.linalg.norm(normal) - 1.0) < ORTHO_TOL, f"tag {key} normal not unit"
        assert abs(np.dot(up, normal)) < ORTHO_TOL, f"tag {key} up/normal not perpendicular"


# ----------------------------------------------------------------- counts
@pytest.mark.parametrize("category,expected", EXPECTED_COUNTS)
def test_category_counts_match_game_rules(model, category, expected):
    counts = Counter(inst.category for inst in model.instances)
    if category == "flower":
        # real export: each flower is an assembly of many parts in group flower_<id>
        groups = {inst.group for inst in model.instances
                  if inst.category == "flower" and (inst.group or "").startswith("flower_")}
        if groups:
            counts["flower"] = len(groups)
    assert counts[category] == expected, f"{category}: expected {expected}, got {counts[category]}"


def test_instance_categories_are_known(model):
    from sim.loader import CATEGORIES

    unknown = {inst.category for inst in model.instances} - set(CATEGORIES)
    assert not unknown, f"instance categories not in loader.CATEGORIES: {sorted(unknown)}"


# ------------------------------------------------------- finite values
def test_transforms_are_finite(model):
    bad = [inst.name for inst in model.instances if not np.isfinite(inst.transform).all()]
    assert not bad, f"non-finite transforms: {bad[:10]}"


def test_mesh_vertices_are_finite(model):
    bad = []
    for key in _referenced_parts(model):
        if not np.isfinite(np.asarray(_get_mesh(model, key).vertices, dtype=float)).all():
            bad.append(key)
    assert not bad, f"non-finite mesh vertices in parts: {bad}"


# ------------------------------------------------------ parts and meshes
def test_instance_parts_exist_in_model_parts(model):
    missing = sorted({inst.part for inst in model.instances} - set(model.parts))
    assert not missing, f"instance parts missing from model.parts: {missing}"


@pytest.mark.slow
def test_instance_meshes_load_with_valid_faces(model):
    problems = []
    for key in _referenced_parts(model):
        try:
            mesh = _get_mesh(model, key)
        except Exception as exc:  # report every failing part, not just the first
            problems.append(f"{key}: load failed ({exc!r})")
            continue
        verts = np.asarray(mesh.vertices)
        faces = np.asarray(mesh.faces)
        if verts.ndim != 2 or verts.shape[1] != 3 or len(verts) == 0:
            problems.append(f"{key}: bad vertex shape {verts.shape}")
            continue
        if faces.ndim != 2 or faces.shape[1] != 3 or len(faces) == 0:
            problems.append(f"{key}: bad face shape {faces.shape}")
            continue
        if faces.min() < 0 or faces.max() >= len(verts):
            problems.append(f"{key}: face index out of range [{faces.min()}, {faces.max()}] for {len(verts)} verts")
    assert not problems, "\n".join(problems)


# ------------------------------------------------------------ transforms
def test_transforms_are_rigid_rotations(model):
    problems = []
    for inst in model.instances:
        T = np.asarray(inst.transform, dtype=float)
        if T.shape != (4, 4):
            problems.append(f"{inst.name}: shape {T.shape}")
            continue
        R = T[:3, :3]
        if not np.allclose(R.T @ R, np.eye(3), atol=ORTHO_TOL):
            problems.append(f"{inst.name}: 3x3 block not orthonormal")
        if abs(np.linalg.det(R) - 1.0) > ORTHO_TOL:
            problems.append(f"{inst.name}: det={np.linalg.det(R):.6f} (expected +1)")
        if not np.allclose(T[3], [0.0, 0.0, 0.0, 1.0], atol=1e-9):
            problems.append(f"{inst.name}: last row {T[3].tolist()}")
    assert not problems, "\n".join(problems[:20])


# -------------------------------------------------------- field envelope
def test_world_bounds_within_field_envelope(model, part_bboxes):
    problems = []
    for inst in model.instances:
        lo, hi = part_bboxes[inst.part]
        corners = np.array(
            [[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])],
            dtype=float,
        )
        world = (np.c_[corners, np.ones(len(corners))] @ np.asarray(inst.transform, float).T)[:, :3]
        wlo, whi = world.min(axis=0), world.max(axis=0)
        # Alliance-area tape and artifact trays sit outside the walls by design (SCHEMA.md).
        xy = OUTSIDE_XY if inst.category in ("tape", "artifact_tray") else FIELD_XY
        if wlo[0] < xy[0] or whi[0] > xy[1] or wlo[1] < xy[0] or whi[1] > xy[1]:
            problems.append(f"{inst.name}: xy bbox {wlo[:2].round(2)}..{whi[:2].round(2)}")
        if wlo[2] < FIELD_Z[0] or whi[2] > FIELD_Z[1]:
            problems.append(f"{inst.name}: z bbox {wlo[2]:.2f}..{whi[2]:.2f}")
    assert not problems, "instances outside field envelope:\n" + "\n".join(problems[:20])
