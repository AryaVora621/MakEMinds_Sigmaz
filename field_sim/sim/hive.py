"""Hive tilt kinematics for the FTC field simulator (pure numpy, headless).

Each hive group is described by a dict (as stored in model.json "groups"):
    pivot               (3,)  point on the tilt axis, Pedro inches
    axis                (3,)  tilt axis direction (normalised internally)
    cad_pose_angle_deg  angle the exported geometry is already at
    stable_angles_deg   the two angles (deg) where the hive rests

Conventions
-----------
* Angle theta is a right-hand rotation about `axis` through `pivot`, applied to
  the hive in its LEVEL pose (theta = 0).
* Tip states:
    "far"      the +y cell is raised
    "audience" the -y cell is raised
  A stable angle is "far" if rotating about it lifts a point at
  pivot + 10*[0,1,0] (z increases); the other stable angle is "audience".
  For axis +x, a positive angle raises +y (so +30 is "far"); for axis -x the
  mapping flips.
* delta_transform(group, state_or_angle) returns the 4x4 that moves geometry
  from the CAD pose to the requested tip angle:
      R_pivot(axis, target - cad_pose_angle_deg)
* Matrices use the column-vector convention: p_world = M @ [p, 1].

Self-check: ``python -B sim/hive.py``.
"""
from __future__ import annotations

import math

import numpy as np

STATES = ("audience", "far")


def _check_state(state: str) -> None:
    if state not in STATES:
        raise ValueError(f"unknown tip state {state!r}; expected one of {STATES}")


def _deg_dist(a: float, b: float) -> float:
    """Absolute angular distance in degrees, wrapped to [0, 180]."""
    return abs(((a - b + 180.0) % 360.0) - 180.0)


def rotation_about_axis(pivot, axis, angle_deg) -> np.ndarray:
    """4x4 right-hand rotation by angle_deg about `axis` passing through `pivot`.

    Uses Rodrigues' formula R = I + sin(t) K + (1 - cos(t)) K^2 with K the cross-
    product matrix of the unit axis, then conjugates by a translation to the pivot.

    Args:
        pivot: (3,) point on the rotation axis.
        axis: (3,) non-zero direction (normalised internally).
        angle_deg: rotation angle in degrees (right-hand rule about axis).

    Returns:
        (4, 4) float64 homogeneous transform.

    Raises:
        ValueError: if axis is the zero vector.
    """
    p = np.asarray(pivot, dtype=float).reshape(3)
    a = np.asarray(axis, dtype=float).reshape(3)
    n = np.linalg.norm(a)
    if n == 0.0:
        raise ValueError("rotation axis must be non-zero")
    a = a / n
    th = math.radians(float(angle_deg))
    K = np.array([[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]])
    R = np.eye(3) + math.sin(th) * K + (1.0 - math.cos(th)) * (K @ K)
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = p - R @ p
    return M


def _lift(group_def: dict, angle_deg: float) -> float:
    """Change in z of a point at pivot + 10*[0,1,0] after rotating by angle_deg."""
    pivot = np.asarray(group_def["pivot"], dtype=float)
    probe = np.append(pivot + np.array([0.0, 10.0, 0.0]), 1.0)
    M = rotation_about_axis(pivot, group_def["axis"], angle_deg)
    return float((M @ probe)[2] - pivot[2])


def _stable_pair(group_def: dict) -> tuple[float, float]:
    """Return (far_angle, audience_angle) from group_def['stable_angles_deg'].

    Raises:
        ValueError: if the two stable angles do not lift opposite sides.
    """
    a, b = (float(v) for v in group_def["stable_angles_deg"])
    la, lb = _lift(group_def, a), _lift(group_def, b)
    if not la * lb < 0.0:
        raise ValueError(
            f"stable angles {a}, {b} must raise opposite sides (lifts {la:.3g}, {lb:.3g})"
        )
    return (a, b) if la > 0.0 else (b, a)


def tip_angle(group_def: dict, state: str) -> float:
    """Stable tilt angle (deg) for a tip state.

    Args:
        group_def: hive group dict (pivot, axis, cad_pose_angle_deg, stable_angles_deg).
        state: "audience" or "far".

    Returns:
        The stable angle in degrees whose rotation raises the requested side.
    """
    _check_state(state)
    far, aud = _stable_pair(group_def)
    return far if state == "far" else aud


def state_of_angle(group_def: dict, angle: float) -> str:
    """Tip state whose stable angle is nearest to `angle` (degrees, circular distance)."""
    far, aud = _stable_pair(group_def)
    angle = float(angle)
    return "far" if _deg_dist(angle, far) <= _deg_dist(angle, aud) else "audience"


def cad_state(group_def: dict) -> str:
    """Tip state of the CAD pose (the angle the exported geometry is already at)."""
    return state_of_angle(group_def, group_def["cad_pose_angle_deg"])


def other_state(state: str) -> str:
    """The opposite tip state: "audience" <-> "far"."""
    _check_state(state)
    return "far" if state == "audience" else "audience"


def delta_transform(group_def: dict, state_or_angle) -> np.ndarray:
    """4x4 transform that moves CAD-pose geometry to the requested tilt.

    Args:
        group_def: hive group dict.
        state_or_angle: a tip state string ("audience"/"far") or a target angle in degrees.

    Returns:
        (4, 4) transform R_pivot(axis, target - cad_pose_angle_deg). Identity when
        the target is the CAD pose.
    """
    if isinstance(state_or_angle, str):
        target = tip_angle(group_def, state_or_angle)
    else:
        target = float(state_or_angle)
    delta = target - float(group_def["cad_pose_angle_deg"])
    return rotation_about_axis(group_def["pivot"], group_def["axis"], delta)


if __name__ == "__main__":
    import json
    from pathlib import Path

    model_path = Path(__file__).resolve().parents[1] / "data" / "synthetic_model" / "model.json"
    groups = json.loads(model_path.read_text(encoding="utf-8"))["groups"]
    red, blue = groups["hive_red"], groups["hive_blue"]
    I4 = np.eye(4)

    # 1. State mapping from the synthetic model.
    assert cad_state(red) == "audience", cad_state(red)
    assert cad_state(blue) == "far", cad_state(blue)
    assert tip_angle(red, "far") == 30.0 and tip_angle(red, "audience") == -30.0
    assert tip_angle(blue, "far") == 30.0 and tip_angle(blue, "audience") == -30.0
    assert other_state("audience") == "far" and other_state("far") == "audience"
    assert state_of_angle(red, -25.0) == "audience" and state_of_angle(blue, 10.0) == "far"

    # 2. Flipped axis: a positive angle now lowers +y, so "far" is -30.
    flipped = dict(red, axis=[-1.0, 0.0, 0.0])
    assert tip_angle(flipped, "far") == -30.0 and tip_angle(flipped, "audience") == 30.0

    # 3. delta_transform to the CAD state is the identity.
    assert np.allclose(delta_transform(red, cad_state(red)), I4, atol=1e-12)
    assert np.allclose(delta_transform(blue, cad_state(blue)), I4, atol=1e-12)

    # 4. delta_transform to the other state rotates by 60 deg about the pivot axis.
    for g in (red, blue):
        d = delta_transform(g, other_state(cad_state(g)))
        R = d[:3, :3]
        assert np.allclose(R @ R.T, np.eye(3), atol=1e-12)
        assert math.isclose(np.linalg.det(R), 1.0, abs_tol=1e-12)
        assert math.isclose(np.trace(R), 1.0 + 2.0 * math.cos(math.radians(60.0)), abs_tol=1e-9)
        assert np.allclose(R[:, 0], [1.0, 0.0, 0.0], atol=1e-12)  # still about +x
    # Numeric angle and state string agree.
    assert np.allclose(delta_transform(red, 30.0), delta_transform(red, "far"))

    # 5. A point on the pivot stays fixed under every delta, and rotations are proper.
    for g in (red, blue):
        pv = np.append(np.asarray(g["pivot"], float), 1.0)
        for st in STATES:
            assert np.allclose(delta_transform(g, st) @ pv, pv, atol=1e-9)
    rng = np.random.default_rng(1)
    for _ in range(50):
        piv = rng.uniform(-50, 50, 3)
        ax = rng.normal(size=3)
        M = rotation_about_axis(piv, ax, rng.uniform(-180, 180))
        assert np.allclose(M @ np.append(piv, 1.0), np.append(piv, 1.0), atol=1e-9)
        assert np.allclose(M[:3, :3] @ M[:3, :3].T, np.eye(3), atol=1e-9)

    # 6. Right-hand rule: +90 deg about +x maps +y to +z.
    v = rotation_about_axis([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], 90.0) @ [0.0, 1.0, 0.0, 1.0]
    assert np.allclose(v[:3], [0.0, 0.0, 1.0], atol=1e-12), v

    print("hive.py self-check OK")
    print(f"  red  cad={cad_state(red)} far={tip_angle(red, 'far')} audience={tip_angle(red, 'audience')}")
    print(f"  blue cad={cad_state(blue)} far={tip_angle(blue, 'far')} audience={tip_angle(blue, 'audience')}")
    print(f"  delta_transform(red, 'far')[:3,:3] trace = {np.trace(delta_transform(red, 'far')[:3, :3]):.6f} (60 deg)")
