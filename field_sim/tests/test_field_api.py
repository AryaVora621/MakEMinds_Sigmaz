"""Tests for the headless Field API and the FTC <-> Pedro frame helpers.

Code is written against the contract in the sim API notes:
  sim.frames.ftc_to_pedro / pedro_to_ftc, sim.field.Field.
Fixtures `model` (FieldModel) and `field` (Field) come from tests/conftest.py.
"""
from __future__ import annotations

import math
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from sim.frames import ftc_to_pedro, pedro_to_ftc

ROOT = Path(__file__).resolve().parents[1]
VENV_PY = ROOT / ".venv" / "Scripts" / "python.exe"

PI = math.pi
ALLIANCES = ("red", "blue")
STATES = ("audience", "far")
PIVOT_Z_EXPECTED = 43.95  # level pivot height of the hive tilt axis (inches)


def _wrap_ok(h: float) -> bool:
    """Heading must lie in the half-open interval (-pi, pi]."""
    return -PI < h <= PI


def _angle_diff(a: float, b: float) -> float:
    """Smallest signed difference between two angles, in radians."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


# ---------------------------------------------------------------- frames

def test_ftc_origin_maps_to_pedro_centre_with_rotated_heading():
    x_p, y_p, h_p = ftc_to_pedro(0.0, 0.0, 0.0)
    assert x_p == pytest.approx(72.0, abs=1e-9)
    assert y_p == pytest.approx(72.0, abs=1e-9)
    assert h_p == pytest.approx(-PI / 2, abs=1e-9)


@pytest.mark.parametrize(
    "x_f, y_f, h_f, x_p, y_p, h_p",
    [
        (10.0, 0.0, 0.0, 72.0, 62.0, -PI / 2),      # x_f moves Pedro y
        (0.0, 10.0, 0.0, 82.0, 72.0, -PI / 2),      # y_f moves Pedro x
        (0.0, 0.0, PI / 2, 72.0, 72.0, 0.0),        # heading shift by +pi/2
    ],
)
def test_ftc_to_pedro_explicit_formulas(x_f, y_f, h_f, x_p, y_p, h_p):
    got = ftc_to_pedro(x_f, y_f, h_f)
    assert len(got) == 3
    assert got[0] == pytest.approx(x_p, abs=1e-9)
    assert got[1] == pytest.approx(y_p, abs=1e-9)
    assert got[2] == pytest.approx(h_p, abs=1e-9)
    assert x_p == pytest.approx(y_f + 72.0, abs=1e-9)
    assert y_p == pytest.approx(72.0 - x_f, abs=1e-9)


def test_round_trip_scalar_random_poses_including_wrap():
    rng = np.random.default_rng(1234)
    for _ in range(200):
        x_f, y_f = rng.uniform(-72.0, 72.0, size=2)
        h_f = rng.uniform(-4 * PI, 4 * PI)
        x_p, y_p, h_p = ftc_to_pedro(float(x_f), float(y_f), float(h_f))
        assert _wrap_ok(h_p), f"pedro heading {h_p} outside (-pi, pi]"
        bx, by, bh = pedro_to_ftc(x_p, y_p, h_p)
        assert bx == pytest.approx(x_f, abs=1e-9)
        assert by == pytest.approx(y_f, abs=1e-9)
        assert _wrap_ok(bh)
        assert abs(_angle_diff(bh, h_f)) < 1e-9


def test_round_trip_pedro_to_ftc_to_pedro_random():
    rng = np.random.default_rng(99)
    for _ in range(200):
        x_p, y_p = rng.uniform(0.0, 144.0, size=2)
        h_p = rng.uniform(-PI, PI)
        x_f, y_f, h_f = pedro_to_ftc(float(x_p), float(y_p), float(h_p))
        assert _wrap_ok(h_f)
        bx, by, bh = ftc_to_pedro(x_f, y_f, h_f)
        assert bx == pytest.approx(x_p, abs=1e-9)
        assert by == pytest.approx(y_p, abs=1e-9)
        assert abs(_angle_diff(bh, h_p)) < 1e-9


def test_vectorised_round_trip_matches_scalar():
    rng = np.random.default_rng(7)
    n = 500
    x_f = rng.uniform(-72.0, 72.0, n)
    y_f = rng.uniform(-72.0, 72.0, n)
    h_f = rng.uniform(-4 * PI, 4 * PI, n)

    x_p, y_p, h_p = ftc_to_pedro(x_f, y_f, h_f)
    x_p, y_p, h_p = map(np.asarray, (x_p, y_p, h_p))
    assert x_p.shape == (n,) and y_p.shape == (n,) and h_p.shape == (n,)

    np.testing.assert_allclose(x_p, y_f + 72.0, atol=1e-9)
    np.testing.assert_allclose(y_p, 72.0 - x_f, atol=1e-9)
    assert np.all(h_p > -PI) and np.all(h_p <= PI)

    bx, by, bh = map(np.asarray, pedro_to_ftc(x_p, y_p, h_p))
    np.testing.assert_allclose(bx, x_f, atol=1e-9)
    np.testing.assert_allclose(by, y_f, atol=1e-9)
    assert np.all(bh > -PI) and np.all(bh <= PI)
    # headings are equal modulo 2*pi
    np.testing.assert_allclose(np.cos(bh), np.cos(h_f), atol=1e-9)
    np.testing.assert_allclose(np.sin(bh), np.sin(h_f), atol=1e-9)


@pytest.mark.parametrize("h_f", [3 * PI, -3 * PI, 10.0, -10.0, PI, -PI])
def test_heading_wrap_stays_in_range(h_f):
    _, _, h_p = ftc_to_pedro(0.0, 0.0, h_f)
    assert _wrap_ok(h_p)
    assert abs(math.cos(h_p) - math.cos(h_f - PI / 2)) < 1e-9
    assert abs(math.sin(h_p) - math.sin(h_f - PI / 2)) < 1e-9


# ---------------------------------------------------------- Field: hive aim

@pytest.mark.parametrize("alliance", ALLIANCES)
@pytest.mark.parametrize("state", STATES)
def test_aim_point_shape_and_above_pivot(field, model, alliance, state):
    aim = np.asarray(field.aim_point(alliance, state), dtype=float)
    assert aim.shape == (3,)
    pivot = np.asarray(model.groups[f"hive_{alliance}"]["pivot"], dtype=float)
    assert pivot[2] == pytest.approx(PIVOT_Z_EXPECTED, abs=1e-6)
    assert aim[2] > pivot[2], "aim point must be above the hive pivot"
    assert aim[0] == pytest.approx(pivot[0], abs=3.0)


@pytest.mark.parametrize("alliance", ALLIANCES)
def test_aim_point_audience_differs_from_far(field, alliance):
    a = np.asarray(field.aim_point(alliance, "audience"), dtype=float)
    f = np.asarray(field.aim_point(alliance, "far"), dtype=float)
    assert np.linalg.norm(a - f) > 1.0


@pytest.mark.parametrize("alliance", ALLIANCES)
def test_aim_point_y_side_matches_tip_state(field, model, alliance):
    """'audience' raises the -y side of the hive, 'far' raises the +y side."""
    pivot_y = model.groups[f"hive_{alliance}"]["pivot"][1]
    aud = np.asarray(field.aim_point(alliance, "audience"), dtype=float)
    far = np.asarray(field.aim_point(alliance, "far"), dtype=float)
    assert aud[1] < pivot_y
    assert far[1] > pivot_y


@pytest.mark.parametrize("alliance", ALLIANCES)
def test_aim_point_matches_cad_up_cell_opening(field, model, alliance):
    """At the CAD pose, red is audience-up and blue is far-up."""
    cad_state = "audience" if alliance == "red" else "far"
    cells = model.constants["hives"][alliance]["cells"]
    expected = np.asarray(cells[cad_state]["opening_center"], dtype=float)
    aim = np.asarray(field.aim_point(alliance, cad_state), dtype=float)
    assert np.linalg.norm(aim - expected) <= 6.0


# -------------------------------------------------------------- Field: zones

def test_zone_names_non_empty_and_cover_model(field, model):
    names = list(field.zone_names())
    assert names, "zone_names() returned nothing"
    for z in model.tape_zones.get("zones", []):
        assert z["name"] in names


def test_every_zone_is_a_valid_polygon_inside_field(field):
    names = list(field.zone_names())
    for name in names:
        if name.startswith("tile seam"):
            continue  # seams are 2-point centre lines, not polygons
        poly = np.asarray(field.zone(name))
        assert poly.ndim == 2 and poly.shape[1] == 2, f"{name}: shape {poly.shape}"
        assert np.issubdtype(poly.dtype, np.floating), f"{name}: dtype {poly.dtype}"
        distinct = np.unique(poly, axis=0)
        assert len(distinct) >= 3, f"{name}: fewer than 3 distinct vertices"
        # closed (first == last) or open are both accepted
        if "alliance area" in name or "alliance_area" in name:
            # alliance areas lie OUTSIDE the walls by design (54 in deep), see SCHEMA.md
            assert np.all(poly >= -60.0) and np.all(poly <= 204.0), f"{name}: outside [-60,204]"
        else:
            assert np.all(poly >= -1.0) and np.all(poly <= 145.0), f"{name}: outside [-1,145]"


# -------------------------------------------------------------- Field: balls

def test_ball_starts_is_list_of_dicts(field, model):
    balls = field.ball_starts()
    assert isinstance(balls, list)
    assert len(balls) > 0
    assert all(isinstance(b, dict) for b in balls)
    expected = model.ball_starts.get("balls", None)
    if expected is not None:
        assert len(balls) == len(expected)


# ------------------------------------------------------- headless importability

def test_field_import_has_no_viewer_modules():
    code = (
        "import sys\n"
        "import sim.field\n"
        "bad = [m for m in ('pyvista', 'vtk') if m in sys.modules]\n"
        "assert not bad, bad\n"
        "print('ok')\n"
    )
    proc = subprocess.run(
        [str(VENV_PY), "-c", code],
        cwd=str(ROOT),
        capture_output=True,
        text=True,
        timeout=120,
    )
    assert proc.returncode == 0, proc.stderr
    assert "ok" in proc.stdout
