"""Tests for the hive tilt mechanism: stable angles, pivot, CAD pose, rigid
tipping, toggle round-trip, and AprilTag pose behaviour under tipping.

Code is written against the field_sim API contract (sim.hive, sim.field.Field).
Fixtures `model` (FieldModel) and `field` (Field(model)) come from tests/conftest.py.
"""
from __future__ import annotations

import itertools
import math

import numpy as np
import pytest

from sim import hive

ALLIANCES = ("red", "blue")
HIVE_GROUPS = {"red": "hive_red", "blue": "hive_blue"}
STATES = ("audience", "far")
EXPECTED_TILT_DEG = 30.0
PIVOT_Y = 72.0
PIVOT_Z = 43.95
PIVOT_TOL = 0.05
GEOM_TOL = 1e-3
N_TAGS = 16


# ----------------------------------------------------------------- helpers
def _group_def(model, alliance):
    return model.groups[HIVE_GROUPS[alliance]]


def _mesh_vertices(model, key):
    get = getattr(model, "get_mesh", None)
    mesh = get(key) if callable(get) else model.meshes[key]
    return np.asarray(mesh.vertices, dtype=np.float64)


def _hive_instances(model, alliance):
    g = HIVE_GROUPS[alliance]
    return [i for i in model.instances if i.group == g]


def _hive_group_points(model, field, alliance, stride=1):
    """World-space vertices of every instance in the alliance's hive group."""
    pts = []
    for inst in _hive_instances(model, alliance):
        v = _mesh_vertices(model, inst.part)[::stride]
        M = field.instance_transform(inst)
        pts.append((M @ np.hstack([v, np.ones((len(v), 1))]).T).T[:, :3])
    return np.vstack(pts)


def _pairwise(P, max_points=1500):
    # Deterministic subsample so the real model (~90k vertices per hive) stays small.
    if len(P) > max_points:
        P = P[np.linspace(0, len(P) - 1, max_points).astype(int)]
    return np.linalg.norm(P[:, None, :] - P[None, :, :], axis=-1)


def _rotation_angle_deg(R):
    c = (np.trace(R) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def _rotation_axis(R):
    """Unit axis of a rotation matrix (only meaningful when angle is not 0/180)."""
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    n = np.linalg.norm(v)
    return v / n if n > 1e-12 else v


def _is_rotation(R, tol=1e-9):
    return np.allclose(R.T @ R, np.eye(3), atol=tol) and math.isclose(
        np.linalg.det(R), 1.0, abs_tol=tol)


# ------------------------------------------------- stable angles and tilt
@pytest.mark.parametrize("alliance", ALLIANCES)
def test_stable_angles_have_30_deg_tilt(model, alliance):
    g = _group_def(model, alliance)
    angles = g["stable_angles_deg"]
    assert len(angles) == 2
    for a in angles:
        assert abs(abs(a) - EXPECTED_TILT_DEG) < 1e-9, f"{alliance}: stable angle {a}"


@pytest.mark.parametrize("alliance", ALLIANCES)
def test_tip_angle_for_both_states_is_30_deg(model, alliance):
    g = _group_def(model, alliance)
    for state in STATES:
        assert abs(abs(hive.tip_angle(g, state)) - EXPECTED_TILT_DEG) < 1e-9


@pytest.mark.parametrize("alliance", ALLIANCES)
def test_far_state_raises_plus_y_side(model, alliance):
    """Axis is +x for both hives, so 'far' (+y raised) must be the +30 deg stable angle
    and 'audience' the -30 deg one. Verified geometrically with a point on the +y side."""
    g = _group_def(model, alliance)
    assert hive.tip_angle(g, "far") == pytest.approx(+EXPECTED_TILT_DEG, abs=1e-9)
    assert hive.tip_angle(g, "audience") == pytest.approx(-EXPECTED_TILT_DEG, abs=1e-9)

    pivot = np.asarray(g["pivot"], float)
    axis = np.asarray(g["axis"], float)
    probe = pivot + np.array([0.0, 10.0, 0.0])  # on the +y side, level pose
    R_far = hive.rotation_about_axis(pivot, axis, hive.tip_angle(g, "far"))
    p_far = (R_far @ np.append(probe, 1.0))[:3]
    assert p_far[2] > pivot[2], "far tip must raise the +y side"


# ------------------------------------------------------------- pivot
@pytest.mark.parametrize("alliance", ALLIANCES)
def test_pivot_location(model, alliance):
    pivot = np.asarray(_group_def(model, alliance)["pivot"], float)
    assert abs(pivot[1] - PIVOT_Y) <= PIVOT_TOL, f"pivot y={pivot[1]}"
    assert abs(pivot[2] - PIVOT_Z) <= PIVOT_TOL, f"pivot z={pivot[2]}"


# ------------------------------------------------------------- CAD pose
def test_initial_hive_state_is_cad_pose(field):
    assert field.hive_state == {"red": "audience", "blue": "far"}


@pytest.mark.parametrize("alliance,up_state", [("red", "audience"), ("blue", "far")])
def test_cad_pose_matches_group_angle(model, field, alliance, up_state):
    g = _group_def(model, alliance)
    assert field.hive_state[alliance] == up_state
    assert hive.tip_angle(g, field.hive_state[alliance]) == pytest.approx(
        g["cad_pose_angle_deg"], abs=1e-9)


@pytest.mark.parametrize("alliance,up_state", [("red", "audience"), ("blue", "far")])
def test_cad_up_cell_opening_is_higher(model, alliance, up_state):
    hd = model.constants["hives"][alliance]
    other = "far" if up_state == "audience" else "audience"
    if "states" in hd:  # real export: both cells given per state; compare inside the CAD state
        cells = hd["states"][up_state]["cells"]
        z_up, z_other = cells[up_state]["aim_point"][2], cells[other]["aim_point"][2]
    else:               # synthetic: both cells given at the CAD pose
        cells = hd["cells"]
        z_up = cells[up_state]["opening_center"][2]
        z_other = cells[other]["opening_center"][2]
    assert z_up > z_other, f"{alliance}: up cell z={z_up} not above {z_other}"


@pytest.mark.parametrize("alliance,up_state", [("red", "audience"), ("blue", "far")])
def test_aim_point_is_up_cell_opening(model, field, alliance, up_state):
    up_center = np.asarray(model.constants["hives"][alliance]["cells"][up_state]["opening_center"])
    aim = np.asarray(field.aim_point(alliance), float)
    assert np.allclose(aim, up_center, atol=GEOM_TOL)


# ------------------------------------------------------- rigid tipping
@pytest.mark.parametrize("alliance", ALLIANCES)
def test_tip_preserves_rigid_geometry(model, field, alliance):
    # Subsample: every 2nd vertex keeps the pairwise matrix small.
    before = _hive_group_points(model, field, alliance, stride=2)
    d_before = _pairwise(before)

    field.toggle_tip(alliance)
    after = _hive_group_points(model, field, alliance, stride=2)
    d_after = _pairwise(after)

    assert before.shape == after.shape
    assert np.max(np.abs(d_before - d_after)) < GEOM_TOL

    # The tip must actually have moved geometry, otherwise the check is vacuous.
    assert np.max(np.abs(before - after)) > 1.0


@pytest.mark.parametrize("alliance", ALLIANCES)
@pytest.mark.parametrize("state", STATES)
def test_delta_transform_is_proper_rotation(model, alliance, state):
    g = _group_def(model, alliance)
    M = np.asarray(hive.delta_transform(g, state), float)
    assert M.shape == (4, 4)
    R = M[:3, :3]
    assert math.isclose(np.linalg.det(R), 1.0, abs_tol=1e-9)
    assert _is_rotation(R)


@pytest.mark.parametrize("alliance", ALLIANCES)
def test_delta_transform_by_angle_is_proper_rotation(model, alliance):
    g = _group_def(model, alliance)
    for angle in (-30.0, 0.0, 17.5, 30.0):
        R = np.asarray(hive.delta_transform(g, angle), float)[:3, :3]
        assert math.isclose(np.linalg.det(R), 1.0, abs_tol=1e-9)


# ------------------------------------------------------- toggle round-trip
def test_double_toggle_returns_to_start(model, field):
    start_state = dict(field.hive_state)
    start_T = {id(i): np.array(field.instance_transform(i), float)
               for i in model.instances}

    for alliance in ALLIANCES:
        field.toggle_tip(alliance)
        field.toggle_tip(alliance)

    assert field.hive_state == start_state
    for inst in model.instances:
        assert np.allclose(field.instance_transform(inst), start_T[id(inst)], atol=1e-9), inst.name


@pytest.mark.parametrize("alliance", ALLIANCES)
def test_single_toggle_changes_state(field, alliance):
    before = field.hive_state[alliance]
    field.toggle_tip(alliance)
    after = field.hive_state[alliance]
    assert after != before
    assert after in STATES


# ----------------------------------------------------- tag poses with tip
def _hive_tag_ids(model):
    return sorted(int(k) for k, v in model.constants["tags"].items()
                  if v.get("group") in HIVE_GROUPS.values())


def _non_hive_tag_ids(model, field):
    hive_ids = set(_hive_tag_ids(model))
    return [t for t in field.tag_ids() if t not in hive_ids]


def test_tag_count(field):
    assert len(field.tag_ids()) == N_TAGS


def test_tag_ids_are_all_on_hives_in_synthetic_model(model, field):
    # Layout check: every tag in field_constants belongs to a hive group.
    tag_groups = {int(k): v.get("group") for k, v in model.constants["tags"].items()}
    assert set(tag_groups) == set(field.tag_ids())
    assert all(g in HIVE_GROUPS.values() for g in tag_groups.values())


def test_hive_tags_rotate_60_deg_between_tip_states(model, field):
    checked = 0
    for tag_id in field.tag_ids():
        if tag_id not in _hive_tag_ids(model):
            continue
        checked += 1
        _check_hive_tag_rotation(model, field, tag_id)
    assert checked == len(_hive_tag_ids(model)) > 0


def _check_hive_tag_rotation(model, field, tag_id):
    T1 = np.asarray(field.tag_pose(tag_id, "audience"), float)
    T2 = np.asarray(field.tag_pose(tag_id, "far"), float)
    rel = T1[:3, :3].T @ T2[:3, :3]
    assert _is_rotation(rel, tol=1e-9)
    # real export stores tag vectors rounded to 1e-3 -> allow 0.01 deg
    assert _rotation_angle_deg(rel) == pytest.approx(2 * EXPECTED_TILT_DEG, abs=1e-2)

    # The relative rotation must be about the hive tilt axis (+x for both hives).
    tag_group = model.constants["tags"][str(tag_id)]["group"]
    axis = np.asarray(model.groups[tag_group]["axis"], float)
    rot_axis = _rotation_axis(rel)
    assert abs(abs(float(np.dot(rot_axis, axis))) - 1.0) < 1e-6


def test_all_tags_normals_and_frames_valid_in_both_states(field):
    ids = field.tag_ids()
    assert len(ids) == N_TAGS
    for tag_id in ids:
        for state in STATES:
            T = np.asarray(field.tag_pose(tag_id, state), float)
            assert T.shape == (4, 4)
            R = T[:3, :3]
            assert abs(np.linalg.norm(R[:, 2]) - 1.0) < 1e-9, f"tag {tag_id} {state}: normal not unit"
            assert _is_rotation(R), f"tag {tag_id} {state}: frame not orthonormal with det +1"


def test_non_hive_tags_do_not_move_with_tip(model, field):
    non_hive = _non_hive_tag_ids(model, field)
    if not non_hive:
        pytest.skip("no non-hive tags in this model (all tags are on hive groups)")
    for tag_id in non_hive:
        T_a = np.asarray(field.tag_pose(tag_id, "audience"), float)
        T_f = np.asarray(field.tag_pose(tag_id, "far"), float)
        assert np.allclose(T_a, T_f, atol=1e-9), f"non-hive tag {tag_id} moved with tip"


def test_tag_pose_default_uses_current_hive_state(model, field):
    for tag_id in _hive_tag_ids(model):
        group_alliance = model.constants["tags"][str(tag_id)]["alliance"]
        explicit = np.asarray(field.tag_pose(tag_id, field.hive_state[group_alliance]), float)
        default = np.asarray(field.tag_pose(tag_id), float)
        assert np.allclose(default, explicit, atol=1e-9)
