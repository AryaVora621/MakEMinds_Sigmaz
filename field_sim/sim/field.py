"""Headless field model: hive tip state, tag / aim poses, zones and ball starts.

Pure numpy.  NO viewer imports, so a physics sim can import this directly.

Conventions (see field_sim/data contract):
  * PEDRO inches, origin audience-left corner, +x right, +y far wall, +z up.
  * Transforms are 4x4, column-vector convention (p_world = M @ [p, 1]).
  * Hive groups 'hive_red' / 'hive_blue' carry a pivot/axis.  Exported geometry
    (instances and constants) sits at cad_pose_angle_deg.  delta_transform(group,
    state) maps CAD-pose geometry to the requested tip state.
  * Tip states: 'audience' (-y side raised) and 'far' (+y side raised).
    The cell named S is the UP cell when the hive is in state S.
  * Tag frame: columns x = tag right, y = tag up, z = outward normal,
    translation = tag centre (right-handed: x = y cross z).

Run the self-check with:  python -m sim.field [--model-dir DIR]
"""
from __future__ import annotations

import math

import numpy as np

from .frames import (  # re-exported helpers
    FTC_TO_PEDRO_4x4,
    ftc_to_pedro,
    pedro_to_ftc,
    pose_to_matrix,
    wrap_angle,
)
from .hive import (
    cad_state,
    delta_transform,
    other_state,
    rotation_about_axis,
    tip_angle,
)
from .loader import load_model

__all__ = [
    "Field",
    "STATES",
    "ALLIANCES",
    "FTC_TO_PEDRO_4x4",
    "ftc_to_pedro",
    "pedro_to_ftc",
    "pose_to_matrix",
    "wrap_angle",
]

STATES = ("audience", "far")
ALLIANCES = ("red", "blue")
_HIVE_PREFIX = "hive_"


# ----------------------------------------------------------------- helpers
def _get_ci(d, *names):
    """Case-insensitive dict lookup; returns None when absent."""
    if not isinstance(d, dict):
        return None
    lower = {str(k).lower(): v for k, v in d.items()}
    for n in names:
        if n.lower() in lower:
            return lower[n.lower()]
    return None


def _frame_from_center_normal_up(center, normal, up) -> np.ndarray:
    """Build an orthonormal right-handed frame: z = normal, y ~ up, x = y x z."""
    z = np.asarray(normal, float).reshape(3)
    z = z / np.linalg.norm(z)
    y = np.asarray(up, float).reshape(3)
    y = y - np.dot(y, z) * z
    y = y / np.linalg.norm(y)
    x = np.cross(y, z)
    m = np.eye(4)
    m[:3, 0] = x
    m[:3, 1] = y
    m[:3, 2] = z
    m[:3, 3] = np.asarray(center, float).reshape(3)
    return m


def _apply(m: np.ndarray, p) -> np.ndarray:
    return (m @ np.append(np.asarray(p, float).reshape(3), 1.0))[:3]


class Field:
    """Headless field state.  hive_state holds the current tip of each alliance."""

    def __init__(self, model=None):
        self.model = load_model() if model is None else model
        self._constants = self.model.constants or {}
        groups = self.model.groups

        # alliance <-> hive group mapping
        self._hive_group = {}
        for al in ALLIANCES:
            g = f"{_HIVE_PREFIX}{al}"
            if g not in groups:
                raise KeyError(f"model.groups has no '{g}'")
            self._hive_group[al] = g
        self._group_alliance = {g: al for al, g in self._hive_group.items()}

        # current tip state, initialised to CAD pose
        self.hive_state = {al: cad_state(groups[g]) for al, g in self._hive_group.items()}

        # tags: normalised id -> record
        self._tag_cad = {}      # id -> 4x4 at CAD pose (before hive delta)
        self._tag_alliance = {}  # id -> alliance for hive tags, None for static
        self._tags = {}          # id -> raw tag dict
        for key, tag in self._iter_tags():
            tid = self._norm_id(key, tag)
            self._tags[tid] = tag
            group = _get_ci(tag, "group")
            al = self._group_alliance.get(group)
            self._tag_alliance[tid] = al
            self._tag_cad[tid] = self._tag_cad_matrix(tag, al)

    # ------------------------------------------------------------ parsing
    def _iter_tags(self):
        tags = _get_ci(self._constants, "tags")
        if tags is None:
            return []
        if isinstance(tags, dict):
            return list(tags.items())
        return [(None, t) for t in tags]

    @staticmethod
    def _norm_id(key, tag):
        raw = _get_ci(tag, "id")
        if raw is None:
            raw = key
        try:
            return int(raw)
        except (TypeError, ValueError):
            return str(raw)

    def _tag_cad_matrix(self, tag, al) -> np.ndarray:
        pose = _get_ci(tag, "pose")
        if pose is not None and np.size(pose) == 16:
            m = np.asarray(pose, float).reshape(4, 4)
        else:
            center = _get_ci(tag, "center", "centre", "position", "pos")
            normal = _get_ci(tag, "normal")
            up = _get_ci(tag, "up")
            if center is None or normal is None or up is None:
                raise ValueError(f"tag {tag!r} lacks center/normal/up or a 16-float pose")
            m = _frame_from_center_normal_up(center, normal, up)
        if al is not None and self._hive_is_level(al):
            m = self._level_to_cad(al) @ m
        return m

    def _hive_dict(self, al):
        return (_get_ci(self._constants, "hives") or {}).get(al, {})

    def _hive_is_level(self, al) -> bool:
        return str(_get_ci(self._hive_dict(al), "pose") or "cad").lower() == "level"

    def _level_to_cad(self, al) -> np.ndarray:
        g = self.model.groups[self._hive_group[al]]
        return rotation_about_axis(g["pivot"], g["axis"], g["cad_pose_angle_deg"])

    # ------------------------------------------------------------ validation
    @staticmethod
    def _check_alliance(al):
        if al not in ALLIANCES:
            raise ValueError(f"alliance must be one of {ALLIANCES}, got {al!r}")

    @staticmethod
    def _check_state(state):
        if state not in STATES:
            raise ValueError(f"tip state must be one of {STATES}, got {state!r}")

    def _resolve_state(self, al, state):
        if state is None:
            return self.hive_state[al]
        self._check_state(state)
        return state

    # ------------------------------------------------------------ hive tip
    def set_tip(self, alliance, state):
        self._check_alliance(alliance)
        self._check_state(state)
        self.hive_state[alliance] = state

    def toggle_tip(self, alliance):
        self._check_alliance(alliance)
        self.hive_state[alliance] = other_state(self.hive_state[alliance])
        return self.hive_state[alliance]

    def hive_delta(self, alliance, state=None) -> np.ndarray:
        """4x4 mapping CAD-pose hive geometry to the given (default: current) tip state."""
        self._check_alliance(alliance)
        st = self._resolve_state(alliance, state)
        return np.asarray(delta_transform(self.model.groups[self._hive_group[alliance]], st), float)

    def hive_angle(self, alliance, state=None) -> float:
        """Tip angle in degrees for the given (default: current) state."""
        self._check_alliance(alliance)
        st = self._resolve_state(alliance, state)
        return float(tip_angle(self.model.groups[self._hive_group[alliance]], st))

    # ------------------------------------------------------------ instances
    def instance_transform(self, inst) -> np.ndarray:
        """World 4x4 of an instance, including the current tip for hive instances."""
        t = np.asarray(inst.transform, float).reshape(4, 4)
        al = self._group_alliance.get(inst.group)
        if al is not None:
            return self.hive_delta(al) @ t
        return t.copy()

    # ------------------------------------------------------------ tags
    def tag_ids(self):
        return sorted(self._tags.keys(), key=lambda v: (isinstance(v, str), v))

    def tag_pose(self, tag_id, tip_state=None) -> np.ndarray:
        tid = self._lookup_tag(tag_id)
        m = self._tag_cad[tid]
        al = self._tag_alliance[tid]
        if al is None:
            return m.copy()
        st = self._resolve_state(al, tip_state)
        per_state = (_get_ci(self._tags[tid], "states") or {}).get(st)
        if per_state and all(k in per_state for k in ("center", "normal", "up")):
            # exact pose exported for this state (real model)
            return _frame_from_center_normal_up(per_state["center"], per_state["normal"], per_state["up"])
        return self.hive_delta(al, st) @ m

    def _lookup_tag(self, tag_id):
        if tag_id in self._tags:
            return tag_id
        try:
            ti = int(tag_id)
        except (TypeError, ValueError):
            ti = None
        if ti is not None and ti in self._tags:
            return ti
        raise KeyError(f"unknown tag id {tag_id!r}")

    # ------------------------------------------------------------ aim / cells
    def aim_point(self, alliance, tip_state=None) -> np.ndarray:
        """Centre of the opening of the cell named by the (given/current) tip state.

        The cell named S is the one that is up when the hive is in state S, so
        its CAD opening centre is moved with hive_delta(alliance, S).
        """
        self._check_alliance(alliance)
        st = self._resolve_state(alliance, tip_state)
        hive = self._hive_dict(alliance)
        cell = hive["cells"][st]
        p = np.asarray(cell["opening_center"], float)
        if str(_get_ci(hive, "pose") or "").lower() == "per_state":
            # real model: aim_point is already exported in the state where this cell is up
            return p.copy()
        if self._hive_is_level(alliance):
            p = _apply(self._level_to_cad(alliance), p)
        return _apply(self.hive_delta(alliance, st), p)

    # ------------------------------------------------------------ zones / balls
    def zone_names(self):
        return [z["name"] for z in self._zone_list()]

    def zone(self, name) -> np.ndarray:
        for z in self._zone_list():
            if z["name"] == name:
                return np.asarray(z["polygon"], float).reshape(-1, 2)
        raise KeyError(f"unknown zone {name!r}; have {self.zone_names()}")

    def _zone_list(self):
        tz = self.model.tape_zones or {}
        zones = tz.get("zones", tz) if isinstance(tz, dict) else tz
        if isinstance(zones, dict):
            return [dict(name=k, **v) for k, v in zones.items()]
        return list(zones)

    def ball_starts(self):
        bs = self.model.ball_starts
        if isinstance(bs, dict):
            bs = bs.get("balls", [])
        return [dict(b) for b in bs]


# ----------------------------------------------------------------- self-check
def _angle_deg(a, b) -> float:
    c = float(np.clip(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b)), -1.0, 1.0))
    return math.degrees(math.acos(c))


def _self_check(model_dir=None):
    model = load_model(model_dir)
    f = Field(model)
    print(f"model: {model.root}")
    print(f"hive_state at init: {f.hive_state}")
    hive_tags = [t for t in f.tag_ids() if f._tag_alliance[t] is not None]
    static_tags = [t for t in f.tag_ids() if f._tag_alliance[t] is None]
    print(f"tags: {len(f.tag_ids())} total, {len(hive_tags)} hive, {len(static_tags)} static")

    # 1) hive tags: normals differ by exactly 60 deg between the two stable states
    worst = 0.0
    for t in hive_tags:
        na = f.tag_pose(t, "audience")[:3, 2]
        nb = f.tag_pose(t, "far")[:3, 2]
        ang = _angle_deg(na, nb)
        worst = max(worst, abs(ang - 60.0))
        # real export rounds to 1e-3, so allow 0.01 deg
        assert abs(ang - 60.0) < 1e-2, f"tag {t}: normal angle {ang}"
    print(f"hive tag normal angle audience->far: 60.000 deg (max err {worst:.2e})")

    # 2) static tags unaffected by tip
    for t in static_tags:
        assert np.allclose(f.tag_pose(t, "audience"), f.tag_pose(t, "far"))

    # 3) aim point z changes between states, and the cell named S is the highest one
    for al in ALLIANCES:
        pa = f.aim_point(al, "audience")
        pf = f.aim_point(al, "far")
        za, zf = pa[2], pf[2]
        # synthetic hive is symmetric about its pivot, so the up cell has the same z in
        # both states; the aim point moves in y (and z stays equal) -- assert the move.
        assert np.linalg.norm(pa - pf) > 1e-3, f"{al}: aim point unchanged {pa} {pf}"
        print(f"{al}: aim audience={np.round(pa, 3).tolist()} far={np.round(pf, 3).tolist()}")
        for st in STATES:
            hd = f._hive_dict(al)
            if str(hd.get("pose", "")).lower() == "per_state":
                cells = hd["states"][st]["cells"]
                zs = {s: float(cells[s]["aim_point"][2]) for s in STATES}
            else:
                d = f.hive_delta(al, st)
                zs = {s: _apply(d, hd["cells"][s]["opening_center"])[2] for s in STATES}
            assert max(zs, key=zs.get) == st, f"{al}: cell named {st} is not the up cell ({zs})"
        print(f"{al}: aim z audience={za:.3f} far={zf:.3f} (named cell is up in each state)")

    # 4) aim at CAD state equals raw opening_center of that cell
    for al in ALLIANCES:
        cad = cad_state(model.groups[f"hive_{al}"])
        raw = np.asarray(model.constants["hives"][al]["cells"][cad]["opening_center"], float)
        aim = f.aim_point(al, cad)
        assert np.allclose(aim, raw, atol=1e-9), f"{al}: aim {aim} != raw {raw}"
        assert np.allclose(f.hive_delta(al, cad), np.eye(4), atol=1e-9)
        print(f"{al}: CAD state '{cad}' aim == raw opening_center {np.round(raw, 4).tolist()}")

    # 5) tag frames are unit and orthonormal (both states, all tags)
    for t in f.tag_ids():
        for st in STATES:
            R = f.tag_pose(t, st)[:3, :3]
            assert np.allclose(R.T @ R, np.eye(3), atol=1e-9), f"tag {t} not orthonormal"
            assert np.allclose(np.linalg.norm(R, axis=0), 1.0, atol=1e-9)
            assert abs(np.linalg.det(R) - 1.0) < 1e-9, f"tag {t} not right-handed"
    print(f"tag frames: unit, orthonormal, right-handed for {len(f.tag_ids())} tags x 2 states")

    # 6) state API and instance transforms
    f.set_tip("red", "far")
    assert f.hive_state["red"] == "far"
    assert f.toggle_tip("red") == "audience"
    f.set_tip("red", "audience")
    try:
        f.set_tip("red", "middle")
        raise AssertionError("bad state accepted")
    except ValueError:
        pass
    for inst in model.instances:
        if inst.group in ("hive_red", "hive_blue"):
            assert np.allclose(f.instance_transform(inst), inst.transform, atol=1e-9)
    print(f"hive angles: red={f.hive_angle('red', 'audience'):+.1f} "
          f"red far={f.hive_angle('red', 'far'):+.1f} blue far={f.hive_angle('blue', 'far'):+.1f}")
    print(f"zones: {f.zone_names()}  ball starts: {len(f.ball_starts())}")
    print("SELF-CHECK PASSED")


if __name__ == "__main__":
    import argparse
    from pathlib import Path

    ap = argparse.ArgumentParser()
    ap.add_argument("--model-dir", type=Path, default=None)
    _self_check(ap.parse_args().model_dir)
