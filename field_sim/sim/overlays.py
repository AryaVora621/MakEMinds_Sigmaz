"""Viewer-agnostic overlay geometry for the FTC field simulator.

Pure numpy, no viewer imports. Each function takes a Field-like object (duck typed:
``.model.constants``, ``.tag_ids()``, ``.tag_pose(id)``, ``.aim_point(alliance)``,
``.zone_names()``, ``.zone(name)``) and returns plain dataclasses describing polylines,
arrows, labels and point markers in PEDRO inches. A viewer (sim/scene.py) converts these
into actors.

Frame: Pedro inches, +x right, +y toward far wall, +z up, field 144 x 144.
Overlay z values default to small positive offsets to avoid z-fighting with the tiles.
Everything reflects the Field's current hive_state because tag_pose/aim_point do.

Usage (self-check):  python sim/overlays.py
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

import numpy as np

FIELD_SIZE = 144.0
RED = (220, 30, 30)
BLUE = (30, 60, 220)
WHITE = (255, 255, 255)
GREY = (150, 150, 150)
AXIS_X = (230, 40, 40)
AXIS_Y = (40, 200, 40)
AXIS_Z = (60, 90, 255)
ROBOT = (255, 140, 0)
HEADING = (255, 255, 0)
TAG_NEUTRAL = (240, 240, 240)
ZONE_PALETTE = ((255, 200, 0), (0, 200, 160), (200, 120, 255),
                (255, 120, 60), (120, 220, 80), (255, 255, 255))
FTC_ORIGIN = (72.0, 72.0, 0.0)


# ------------------------------------------------------------------ data types
@dataclass
class Polyline:
    points: np.ndarray            # (N,3) float, not repeating the first point when closed
    color: tuple = WHITE          # (r,g,b) 0-255
    name: str = ""
    closed: bool = False
    width: float = 2.0

    def __post_init__(self):
        self.points = np.asarray(self.points, dtype=float).reshape(-1, 3)


@dataclass
class Label:
    text: str
    position: np.ndarray          # (3,)
    color: tuple = WHITE

    def __post_init__(self):
        self.position = np.asarray(self.position, dtype=float).reshape(3)


@dataclass
class Arrow:
    origin: np.ndarray            # (3,)
    vector: np.ndarray            # (3,) full arrow vector (tip = origin + vector)
    color: tuple = WHITE
    name: str = ""

    def __post_init__(self):
        self.origin = np.asarray(self.origin, dtype=float).reshape(3)
        self.vector = np.asarray(self.vector, dtype=float).reshape(3)


@dataclass
class PointMarker:
    position: np.ndarray          # (3,)
    color: tuple = WHITE
    name: str = ""
    radius: float = 2.0

    def __post_init__(self):
        self.position = np.asarray(self.position, dtype=float).reshape(3)


# ------------------------------------------------------------------- helpers
def _zone_colors(names: Sequence[str]) -> dict:
    """Alliance colour for names containing red/blue, palette order otherwise."""
    out, k = {}, 0
    for n in names:
        low = n.lower()
        if "red" in low:
            out[n] = RED
        elif "blue" in low:
            out[n] = BLUE
        else:
            out[n] = ZONE_PALETTE[k % len(ZONE_PALETTE)]
            k += 1
    return out


def _centroid(poly: np.ndarray) -> np.ndarray:
    """Area centroid of a simple 2D polygon (falls back to vertex mean if degenerate)."""
    x, y = poly[:, 0], poly[:, 1]
    x1, y1 = np.roll(x, -1), np.roll(y, -1)
    cross = x * y1 - x1 * y
    area2 = cross.sum()
    if abs(area2) < 1e-9:
        return poly.mean(axis=0)
    cx = ((x + x1) * cross).sum() / (3.0 * area2)
    cy = ((y + y1) * cross).sum() / (3.0 * area2)
    return np.array([cx, cy])


def _frame_basis(frame: str):
    """Return (origin, 3x3 rows = frame x,y,z axes expressed in Pedro)."""
    if frame == "pedro":
        return np.zeros(3), np.eye(3)
    if frame == "ftc":
        # FTC +x = Pedro -y, FTC +y = Pedro +x, z up; origin at field centre.
        return np.array(FTC_ORIGIN), np.array([[0.0, -1.0, 0.0],
                                               [1.0, 0.0, 0.0],
                                               [0.0, 0.0, 1.0]])
    raise ValueError(f"frame must be 'pedro' or 'ftc', got {frame!r}")


def _tag_info(field, tid) -> dict:
    consts = getattr(getattr(field, "model", None), "constants", None) or {}
    return (consts.get("tags") or {}).get(str(tid), {})


def _tag_color(info: dict) -> tuple:
    al = str(info.get("alliance") or "").lower()
    return RED if al == "red" else BLUE if al == "blue" else TAG_NEUTRAL


# ----------------------------------------------------------------- tape zones
def _drawable_zone_names(field) -> list:
    """Zone names to draw. Tile seams (2-point lines in the real export) are skipped:
    they are not tape and the 24-in grid overlay already shows them."""
    return [n for n in field.zone_names() if not str(n).lower().startswith("tile seam")]


def tape_zone_polylines(field, z: float = 0.15) -> list[Polyline]:
    """One closed polyline per tape zone polygon."""
    names = _drawable_zone_names(field)
    colors = _zone_colors(names)
    out = []
    for n in names:
        poly = np.asarray(field.zone(n), dtype=float).reshape(-1, 2)
        if len(poly) > 1 and np.allclose(poly[0], poly[-1]):
            poly = poly[:-1]
        pts = np.column_stack([poly, np.full(len(poly), z)])
        out.append(Polyline(pts, colors[n], n, closed=True, width=3.0))
    return out


def zone_labels(field, z: float = 0.2) -> list[Label]:
    """Zone name labels at polygon area centroids."""
    names = _drawable_zone_names(field)
    colors = _zone_colors(names)
    out = []
    for n in names:
        poly = np.asarray(field.zone(n), dtype=float).reshape(-1, 2)
        c = _centroid(poly)
        out.append(Label(n, [c[0], c[1], z], colors[n]))
    return out


# ------------------------------------------------------------ field and grid
def field_outline_polyline(size: float = FIELD_SIZE, z: float = 0.1) -> Polyline:
    s = float(size)
    pts = [(0, 0, z), (s, 0, z), (s, s, z), (0, s, z)]
    return Polyline(pts, WHITE, "field_outline", closed=True, width=3.0)


def grid_polylines(step: float = 24.0, size: float = FIELD_SIZE, z: float = 0.05) -> list[Polyline]:
    """Lines at every multiple of `step` across the field, both axes (inclusive of both edges)."""
    n = int(round(size / step))
    if abs(n * step - size) > 1e-9:
        raise ValueError(f"step {step} does not divide size {size}")
    lines = []
    for k in range(n + 1):
        c = k * step
        lines.append(Polyline([(c, 0, z), (c, size, z)], GREY, f"grid_x_{c:g}", width=1.0))
        lines.append(Polyline([(0, c, z), (size, c, z)], GREY, f"grid_y_{c:g}", width=1.0))
    return lines


# ------------------------------------------------------------------- axes
def axes_arrows(frame: str = "pedro", length: float = 24.0) -> list[Arrow]:
    """x (red), y (green), z (blue) arrows from the frame origin."""
    origin, basis = _frame_basis(frame)
    items = ((AXIS_X, "x"), (AXIS_Y, "y"), (AXIS_Z, "z"))
    return [Arrow(origin, basis[i] * length, col, f"{frame}_{nm}")
            for i, (col, nm) in enumerate(items)]


def axes_labels(frame: str = "pedro", length: float = 24.0) -> list[Label]:
    """Labels just beyond each arrow tip."""
    origin, basis = _frame_basis(frame)
    texts = ("+x", "+y", "+z") if frame == "pedro" else ("FTC +x", "FTC +y", "FTC +z")
    cols = (AXIS_X, AXIS_Y, AXIS_Z)
    return [Label(texts[i], origin + basis[i] * length * 1.1, cols[i]) for i in range(3)]


# ------------------------------------------------------------------ robot
def robot_corners(x: float, y: float, heading_deg: float,
                  size: float = 18.0, height: float = 18.0) -> np.ndarray:
    """(8,3) corners: 0-3 bottom ring (z=0), 4-7 top ring (z=height), CCW in plan.

    Footprint is a square of side `size` centred on (x, y), rotated CCW by heading_deg from +x.
    """
    h = math.radians(heading_deg)
    c, s = math.cos(h), math.sin(h)
    half = size / 2.0
    local = ((-half, -half), (half, -half), (half, half), (-half, half))
    out = []
    for zz in (0.0, float(height)):
        for u, v in local:
            out.append((c * u - s * v + x, s * u + c * v + y, zz))
    return np.array(out, dtype=float)


def robot_box_polylines(x: float, y: float, heading_deg: float, size: float = 18.0,
                        height: float = 18.0, color: tuple = ROBOT) -> list[Polyline]:
    """12 edges as 6 polylines: bottom loop, top loop, 4 verticals."""
    P = robot_corners(x, y, heading_deg, size, height)
    out = [Polyline(P[0:4], color, "robot_bottom", closed=True),
           Polyline(P[4:8], color, "robot_top", closed=True)]
    for i in range(4):
        out.append(Polyline(P[[i, i + 4]], color, f"robot_edge_{i}"))
    return out


def robot_heading_arrow(x: float, y: float, heading_deg: float, size: float = 18.0,
                        height: float = 18.0, color: tuple = HEADING) -> Arrow:
    """Arrow on the top face from the box centre, pointing along heading, reaching past the front edge."""
    h = math.radians(heading_deg)
    vec = (size * math.cos(h), size * math.sin(h), 0.0)
    return Arrow((x, y, height), vec, color, "robot_heading")


# -------------------------------------------------------------------- tags
def tag_normal_arrows(field, length: float = 6.0) -> list[Arrow]:
    """Arrow from each tag centre along its outward normal (tag_pose column z)."""
    out = []
    for tid in field.tag_ids():
        T = np.asarray(field.tag_pose(tid), dtype=float)
        out.append(Arrow(T[:3, 3], T[:3, 2] * length, _tag_color(_tag_info(field, tid)),
                         f"tag {tid} normal"))
    return out


def tag_square_polylines(field, size: float = 6.5) -> list[Polyline]:
    """Closed square of side `size` centred on each tag, lying in the tag plane (columns x, y)."""
    out = []
    for tid in field.tag_ids():
        T = np.asarray(field.tag_pose(tid), dtype=float)
        c = T[:3, 3]
        ax = T[:3, 0] * (size / 2.0)
        ay = T[:3, 1] * (size / 2.0)
        corners = [c - ax - ay, c + ax - ay, c + ax + ay, c - ax + ay]
        out.append(Polyline(np.array(corners), _tag_color(_tag_info(field, tid)),
                            f"tag {tid} square", closed=True, width=2.0))
    return out


def tag_labels(field, offset: float = 1.5) -> list[Label]:
    """'ID n' labels offset from each tag centre along its outward normal."""
    out = []
    for tid in field.tag_ids():
        T = np.asarray(field.tag_pose(tid), dtype=float)
        pos = T[:3, 3] + T[:3, 2] * offset
        out.append(Label(f"ID {tid}", pos, _tag_color(_tag_info(field, tid))))
    return out


# ------------------------------------------------------------- aim points
def aim_point_markers(field, radius: float = 2.0) -> list[PointMarker]:
    out = []
    for al, col in (("red", RED), ("blue", BLUE)):
        out.append(PointMarker(field.aim_point(al), col, f"aim_{al}", radius))
    return out


def aim_point_labels(field, offset: float = 3.0) -> list[Label]:
    out = []
    for al, col in (("red", RED), ("blue", BLUE)):
        p = np.asarray(field.aim_point(al), dtype=float).reshape(3)
        out.append(Label(f"AIM {al.upper()}", p + np.array([0.0, 0.0, offset]), col))
    return out


# -------------------------------------------------------------- self-check
class _JsonStub:
    """Stand-in Field built straight from the synthetic JSON files (CAD pose)."""

    def __init__(self):
        import json
        from pathlib import Path
        d = Path(__file__).resolve().parents[1] / "data" / "synthetic_model"
        self._zones = json.loads((d / "tape_zones.json").read_text())["zones"]
        self._consts = json.loads((d / "field_constants.json").read_text())
        self.model = type("M", (), {"constants": self._consts})()

    def zone_names(self):
        return [z["name"] for z in self._zones]

    def zone(self, name):
        for z in self._zones:
            if z["name"] == name:
                return np.array(z["polygon"], dtype=float)
        raise KeyError(name)

    def tag_ids(self):
        return sorted(int(k) for k in self._consts["tags"])

    def tag_pose(self, tid, tip_state=None):
        t = self._consts["tags"][str(tid)]
        y = np.array(t["up"], float)
        z = np.array(t["normal"], float)
        x = np.cross(y, z)
        M = np.eye(4)
        M[:3, 0], M[:3, 1], M[:3, 2], M[:3, 3] = x, y, z, t["center"]
        return M

    def aim_point(self, alliance, tip_state=None):
        # CAD pose: red has the audience cell up, blue has the far cell up.
        side = "audience" if alliance == "red" else "far"
        return np.array(self._consts["hives"][alliance]["cells"][side]["opening_center"], float)


def _self_check(field, source: str) -> None:
    print(f"field source: {source}")
    zones = tape_zone_polylines(field)
    zlabels = zone_labels(field)
    outline = field_outline_polyline()
    grid = grid_polylines()
    axes = axes_arrows("pedro")
    ftc_axes = axes_arrows("ftc")
    ax_labels = axes_labels("ftc")
    robot = robot_box_polylines(72.0, 72.0, 30.0)
    heading = robot_heading_arrow(72.0, 72.0, 30.0)
    tnorm = tag_normal_arrows(field)
    tsq = tag_square_polylines(field)
    tlab = tag_labels(field)
    aim = aim_point_markers(field)
    alab = aim_point_labels(field)

    print(f"tape zones: {len(zones)} polylines, {len(zlabels)} labels")
    print(f"outline: {outline.name}, grid lines: {len(grid)}")
    print(f"axes: pedro {len(axes)} arrows, ftc {len(ftc_axes)} arrows, {len(ax_labels)} labels")
    print(f"robot box: {len(robot)} polylines, heading arrow: {heading.name}")
    print(f"tags: {len(tnorm)} normal arrows, {len(tsq)} squares, {len(tlab)} labels")
    print(f"aim: {len(aim)} markers, {len(alab)} labels")

    # tape zones
    assert len(zones) == len(_drawable_zone_names(field))
    assert all(z.closed and len(z.points) >= 3 for z in zones)

    # robot box: 8 distinct corners, 18x18 footprint, correct rotation
    for hd in (0.0, 30.0, 90.0):
        P = robot_corners(10.0, 20.0, hd)
        assert len({tuple(np.round(p, 6)) for p in P}) == 8, "robot corners not distinct"
        assert np.allclose(P[:4, 2], 0.0) and np.allclose(P[4:, 2], 18.0)
        B = P[:4]
        edges = [np.linalg.norm(B[(i + 1) % 4] - B[i]) for i in range(4)]
        assert np.allclose(edges, 18.0), edges
        assert np.allclose(B[:, :2].mean(axis=0), (10.0, 20.0))
        d0 = B[0, :2] - (10.0, 20.0)
        ang = math.degrees(math.atan2(d0[1], d0[0])) % 360.0
        expect = (hd + 225.0) % 360.0
        assert min(abs(ang - expect), 360.0 - abs(ang - expect)) < 1e-6, (hd, ang, expect)
    P90 = robot_corners(0.0, 0.0, 90.0)
    assert np.isclose(np.ptp(P90[:, 0]), 18.0) and np.isclose(np.ptp(P90[:, 1]), 18.0)
    P0 = robot_corners(0.0, 0.0, 0.0)
    assert np.isclose(np.ptp(P0[:, 0]), 18.0) and np.isclose(np.ptp(P0[:, 1]), 18.0)

    # grids: 7 distinct x positions and 7 distinct y positions spanning 0..144
    xs = {round(float(p.points[0, 0]), 6) for p in grid if np.allclose(p.points[:, 0], p.points[0, 0])}
    ys = {round(float(p.points[0, 1]), 6) for p in grid if np.allclose(p.points[:, 1], p.points[0, 1])}
    assert len(grid) == 14, len(grid)
    assert sorted(xs) == [0, 24, 48, 72, 96, 120, 144] and sorted(ys) == sorted(xs), (xs, ys)
    assert len(xs) == 7 and len(ys) == 7

    # ftc axes: origin at field centre, x arrow points to Pedro -y, y arrow to Pedro +x
    assert np.allclose(ftc_axes[0].origin, (72.0, 72.0, 0.0))
    assert np.allclose(ftc_axes[0].vector, (0.0, -24.0, 0.0))
    assert np.allclose(ftc_axes[1].vector, (24.0, 0.0, 0.0))

    # tags and aim points
    assert len(tnorm) == len(field.tag_ids()) == len(tsq) == len(tlab)
    for tid, p in zip(field.tag_ids(), tsq):  # squares lie in the tag plane (normal = column z)
        T = np.asarray(field.tag_pose(tid), dtype=float)
        assert np.allclose((p.points - T[:3, 3]) @ T[:3, 2], 0.0, atol=1e-9)
        assert np.allclose(np.linalg.norm(p.points - T[:3, 3], axis=1), np.hypot(3.25, 3.25))
    assert len(aim) == 2 and len(alab) == 2

    print("self-check: all assertions passed")


if __name__ == "__main__":
    import sys
    from pathlib import Path

    root = Path(__file__).resolve().parents[1]
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))
    try:
        from sim.field import Field
        field = Field()
        source = "sim.field.Field"
    except Exception as exc:  # Field not available yet; use the JSON stub
        print(f"sim.field.Field unavailable ({type(exc).__name__}: {exc}); using JSON stub")
        field = _JsonStub()
        source = "JSON stub (synthetic_model)"
    _self_check(field, source)
