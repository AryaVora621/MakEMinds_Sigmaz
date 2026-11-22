"""Planar frame conversions for the FTC field simulator (pure numpy, headless).

Frames
------
FTC field frame (x_f, y_f, h_f): positions in inches, heading h_f in radians,
    measured from +x_f toward +y_f.
Pedro frame (x_p, y_p, h_p): inches. Origin at the audience-left corner,
    +x right, +y toward the far wall, +z up. Field is 144 x 144 in.
    Heading h_p in radians, measured from +x_p toward +y_p.
    Red alliance at low x, blue at high x.

Mapping (FTC -> Pedro)
----------------------
    x_p = y_f + 72
    y_p = 72 - x_f
    h_p = h_f - pi/2          (wrapped to (-pi, pi])

The xy part is a proper rotation (det +1) plus a translation; z is unchanged.
Pedro -> FTC is the exact inverse:
    x_f = 72 - y_p,  y_f = x_p - 72,  h_f = h_p + pi/2 (wrapped).

Conventions
-----------
* Matrices use the column-vector convention: p_world = M @ [p, 1].
* Functions accept scalars or numpy-broadcastable arrays. Scalar inputs return
  Python floats; array inputs return arrays.
* Headings are always returned wrapped to (-pi, pi].

Self-check: ``python -B sim/frames.py``.
"""
from __future__ import annotations

import math

import numpy as np

FIELD_HALF_IN = 72.0  # half of the 144 in field side

FTC_TO_PEDRO_4x4 = np.array(
    [
        [0.0, 1.0, 0.0, FIELD_HALF_IN],
        [-1.0, 0.0, 0.0, FIELD_HALF_IN],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ],
    dtype=float,
)
PEDRO_TO_FTC_4x4 = np.array(
    [
        [0.0, -1.0, 0.0, FIELD_HALF_IN],
        [1.0, 0.0, 0.0, -FIELD_HALF_IN],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ],
    dtype=float,
)


def _out(x: np.ndarray):
    """Return a Python float for 0-d results, otherwise the array unchanged."""
    return float(x) if x.ndim == 0 else x


def wrap_angle(a):
    """Wrap angle(s) in radians into the half-open interval (-pi, pi].

    Args:
        a: scalar or array of angles in radians.

    Returns:
        Wrapped angle(s): float for scalar input, ndarray otherwise.
    """
    arr = np.asarray(a, dtype=float)
    r = np.mod(arr, 2.0 * math.pi)  # [0, 2pi)
    w = np.where(r > math.pi, r - 2.0 * math.pi, r)  # (-pi, pi]
    return _out(w)


def ftc_to_pedro(x_f, y_f, h_f):
    """Convert FTC field pose(s) to Pedro pose(s).

    Args:
        x_f, y_f: FTC position in inches (scalar or array).
        h_f: FTC heading in radians (scalar or array).

    Returns:
        Tuple (x_p, y_p, h_p). Pedro position in inches, heading wrapped to (-pi, pi].
    """
    x_f = np.asarray(x_f, dtype=float)
    y_f = np.asarray(y_f, dtype=float)
    h_f = np.asarray(h_f, dtype=float)
    x_p = y_f + FIELD_HALF_IN
    y_p = FIELD_HALF_IN - x_f
    h_p = wrap_angle(h_f - math.pi / 2.0)
    return _out(np.asarray(x_p)), _out(np.asarray(y_p)), _out(np.asarray(h_p))


def pedro_to_ftc(x_p, y_p, h_p):
    """Convert Pedro pose(s) to FTC field pose(s). Exact inverse of ftc_to_pedro.

    Args:
        x_p, y_p: Pedro position in inches (scalar or array).
        h_p: Pedro heading in radians (scalar or array).

    Returns:
        Tuple (x_f, y_f, h_f). FTC position in inches, heading wrapped to (-pi, pi].
    """
    x_p = np.asarray(x_p, dtype=float)
    y_p = np.asarray(y_p, dtype=float)
    h_p = np.asarray(h_p, dtype=float)
    x_f = FIELD_HALF_IN - y_p
    y_f = x_p - FIELD_HALF_IN
    h_f = wrap_angle(h_p + math.pi / 2.0)
    return _out(np.asarray(x_f)), _out(np.asarray(y_f)), _out(np.asarray(h_f))


def ftc_point_to_pedro(p):
    """Convert FTC xy point(s) to Pedro xy.

    Args:
        p: array-like of shape (..., 2) with (x_f, y_f) in inches.

    Returns:
        ndarray of shape (..., 2) with (x_p, y_p).
    """
    p = np.asarray(p, dtype=float)
    if p.shape[-1] != 2:
        raise ValueError(f"expected last axis of size 2, got shape {p.shape}")
    out = np.empty_like(p)
    out[..., 0] = p[..., 1] + FIELD_HALF_IN
    out[..., 1] = FIELD_HALF_IN - p[..., 0]
    return out


def pedro_point_to_ftc(p):
    """Convert Pedro xy point(s) to FTC xy. Inverse of ftc_point_to_pedro.

    Args:
        p: array-like of shape (..., 2) with (x_p, y_p) in inches.

    Returns:
        ndarray of shape (..., 2) with (x_f, y_f).
    """
    p = np.asarray(p, dtype=float)
    if p.shape[-1] != 2:
        raise ValueError(f"expected last axis of size 2, got shape {p.shape}")
    out = np.empty_like(p)
    out[..., 0] = FIELD_HALF_IN - p[..., 1]
    out[..., 1] = p[..., 0] - FIELD_HALF_IN
    return out


def pose_to_matrix(x, y, heading_rad, z=0.0):
    """Build a 4x4 pose matrix: planar rotation about +z by heading, then translation.

    The heading is measured from +x toward +y in the frame the pose is expressed in.

    Args:
        x, y: position (inches).
        heading_rad: heading in radians.
        z: height (inches), default 0.

    Returns:
        (4, 4) float64 homogeneous transform.
    """
    c, s = math.cos(heading_rad), math.sin(heading_rad)
    return np.array(
        [
            [c, -s, 0.0, float(x)],
            [s, c, 0.0, float(y)],
            [0.0, 0.0, 1.0, float(z)],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


if __name__ == "__main__":
    pi = math.pi

    # 1. Documented example: FTC (0,0,0) -> Pedro (72,72,-pi/2), and back.
    xp, yp, hp = ftc_to_pedro(0.0, 0.0, 0.0)
    assert (xp, yp) == (72.0, 72.0), (xp, yp)
    assert math.isclose(hp, -pi / 2), hp
    assert isinstance(xp, float) and isinstance(hp, float), type(xp)
    xf, yf, hf = pedro_to_ftc(72.0, 72.0, -pi / 2)
    assert abs(xf) < 1e-12 and abs(yf) < 1e-12 and abs(hf) < 1e-12, (xf, yf, hf)

    # 2. Vectorised round trips FTC -> Pedro -> FTC and Pedro -> FTC -> Pedro.
    rng = np.random.default_rng(0)
    x = rng.uniform(-100, 100, 2000)
    y = rng.uniform(-100, 100, 2000)
    h = rng.uniform(-10 * pi, 10 * pi, 2000)
    xp, yp, hp = ftc_to_pedro(x, y, h)
    xb, yb, hb = pedro_to_ftc(xp, yp, hp)
    assert np.allclose(xb, x) and np.allclose(yb, y)
    assert np.allclose(np.cos(hb), np.cos(h)) and np.allclose(np.sin(hb), np.sin(h))
    assert np.all(hp > -pi) and np.all(hp <= pi) and np.all(hb > -pi) and np.all(hb <= pi)
    assert np.allclose(np.cos(hp), np.cos(h - pi / 2)) and np.allclose(np.sin(hp), np.sin(h - pi / 2))
    xp2, yp2, hp2 = ftc_to_pedro(*pedro_to_ftc(x, y, h))  # Pedro-side round trip
    xq, yq, hq = pedro_to_ftc(*ftc_to_pedro(x, y, h))
    assert np.allclose(xq, x) and np.allclose(yq, y)

    # 3. Wrapping.
    assert wrap_angle(pi) == pi and wrap_angle(-pi) == pi
    assert math.isclose(wrap_angle(0.5), 0.5)
    assert math.isclose(wrap_angle(2 * pi + 0.1), 0.1, abs_tol=1e-12)
    assert math.isclose(wrap_angle(-3 * pi / 2), pi / 2, abs_tol=1e-12)
    assert math.isclose(abs(wrap_angle(3 * pi)), pi, abs_tol=1e-12)

    # 4. Matrices.
    p = np.array([3.0, 5.0, 2.0, 1.0])  # FTC point x=3, y=5, z=2
    assert np.allclose(FTC_TO_PEDRO_4x4 @ p, [5.0 + 72.0, 72.0 - 3.0, 2.0, 1.0])
    assert np.allclose(FTC_TO_PEDRO_4x4 @ PEDRO_TO_FTC_4x4, np.eye(4))
    assert np.allclose(PEDRO_TO_FTC_4x4, np.linalg.inv(FTC_TO_PEDRO_4x4))
    # Heading consistency: an FTC heading h maps to the Pedro direction of angle h - pi/2.
    for h0 in (0.0, 0.7, 2.5, -2.0):
        d = FTC_TO_PEDRO_4x4[:3, :3] @ np.array([math.cos(h0), math.sin(h0), 0.0])
        assert math.isclose(math.atan2(d[1], d[0]), float(wrap_angle(h0 - pi / 2)), abs_tol=1e-12)

    # 5. Point helpers.
    pts = np.array([[0.0, 0.0], [10.0, -20.0]])
    pp = ftc_point_to_pedro(pts)
    assert np.allclose(pp, [[72.0, 72.0], [52.0, 62.0]]), pp
    assert np.allclose(pedro_point_to_ftc(pp), pts)
    assert np.allclose(ftc_point_to_pedro([3.0, 5.0]), [77.0, 69.0])

    # 6. pose_to_matrix: rotating +x by 90 deg gives +y; translation and z applied.
    M = pose_to_matrix(10.0, 20.0, pi / 2, z=3.0)
    assert np.allclose(M @ [1.0, 0.0, 0.0, 1.0], [10.0, 21.0, 3.0, 1.0])

    print("frames.py self-check OK")
    print(f"  ftc_to_pedro(0,0,0)      = {ftc_to_pedro(0.0, 0.0, 0.0)}")
    print(f"  pedro_to_ftc(72,72,-pi/2)= {pedro_to_ftc(72.0, 72.0, -pi / 2)}")
    print(f"  random round trips       : {x.size} samples, max |dx| = {np.max(np.abs(xb - x)):.2e}")
