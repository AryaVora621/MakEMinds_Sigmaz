"""Scene builder for the field simulator (viewer specific; software renderer sim.render).

build_scene(plotter, field, opts) -> SceneHandles

* Meshes are merged per (category, group) into ONE actor (a few dozen actors even for
  hundreds of parts / millions of triangles).  Hive groups are separate actors whose
  ``user_matrix`` carries the tip rotation, so tipping costs nothing.
* Static overlays (tape zones, outline, grid, axes, robot box) are drawn once.
  Dynamic overlays (AprilTag squares/normals/labels, aim points) depend on the hive
  tip state and are rebuilt by ``refresh_dynamic()``.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field as dc_field
from pathlib import Path

import numpy as np

from . import hive, overlays
from .frames import pedro_to_ftc
from .loader import CATEGORIES, CATEGORY_PALETTE
from .render import Actor

LOW_LOD_HIDDEN = ("hardware", "decoration", "other")
CAMERAS = ("iso", "top", "side-x", "side-y", "audience", "red", "blue")
_C = np.array([72.0, 72.0, 15.0])


@dataclass
class SceneOpts:
    lod: str = "full"                 # full | low
    hide: tuple = ()
    only: tuple = ()                  # if non-empty, only these categories
    alliance_axes: bool = False
    labels: bool = False
    frame: str = "pedro"
    show_aim_points: bool = False
    show_tags: bool = False
    robot_box: bool = False
    pose: tuple | None = None         # (x, y, heading_deg) Pedro
    grid: bool = False
    zones: bool = True
    background: str = "#1e2230"


def camera_preset(name: str):
    """Return (position, focal, viewup) for a named camera (Pedro inches)."""
    c = _C
    presets = {
        "iso": ((c[0] - 120, c[1] - 190, 190.0), c, (0, 0, 1)),
        "top": ((72, 72, 330), (72, 72, 0), (0, 1, 0)),
        "side-x": ((72 + 300, 72, 40), (72, 72, 25), (0, 0, 1)),   # from +x side, looks along -x
        "side-y": ((72, -230, 40), (72, 72, 25), (0, 0, 1)),       # from audience side
        "audience": ((72, -170, 55), (72, 72, 20), (0, 0, 1)),
        "red": ((-190, 72, 60), (72, 72, 20), (0, 0, 1)),
        "blue": ((334, 72, 60), (72, 72, 20), (0, 0, 1)),
    }
    if name not in presets:
        raise ValueError(f"unknown camera {name!r}; choose from {CAMERAS}")
    p, f, u = presets[name]
    return tuple(map(float, p)), tuple(map(float, f)), tuple(map(float, u))


def apply_camera(plotter, name: str):
    pos, foc, up = camera_preset(name)
    plotter.camera.set(pos, foc, up)


# ----------------------------------------------------------------------- drawing helpers
def draw_polylines(plotter, polylines, name, width=2.0, opacity=1.0):
    plotter.set_overlay(name, "lines", polylines)


def draw_arrows(plotter, arrows, name):
    plotter.set_overlay(name, "arrows", arrows)


def draw_markers(plotter, markers, name):
    plotter.set_overlay(name, "markers", markers)


def draw_labels(plotter, labels, name, font_size=12):
    plotter.set_overlay(name, "labels", labels)


# ----------------------------------------------------------------------- merged geometry
def _merge_instances(model, items):
    """items: list of (mesh_key, [Instance]) -> vertices, faces, per-face rgba or None-list, counts."""
    verts, faces, cols, counts = [], [], [], []
    off = 0
    for key, insts in items:
        mesh = model.get_mesh(key)
        v = np.asarray(mesh.vertices, np.float64)
        f = np.asarray(mesh.faces, np.int64)
        fc = mesh.face_colors
        for inst in insts:
            M = inst.transform
            verts.append((v @ M[:3, :3].T + M[:3, 3]).astype(np.float32))
            faces.append(f + off)
            off += len(v)
            cols.append(fc)
            counts.append(len(f))
    return np.vstack(verts), np.vstack(faces), cols, counts


class SceneHandles:
    def __init__(self, plotter, field, opts):
        self.plotter = plotter
        self.field = field
        self.opts = opts
        self.actors = {}            # (cat, group) -> actor
        self.cat_actors = {}        # cat -> [actor]
        self.cat_visible = {}
        self.hive_actors = {"red": [], "blue": []}
        self.angles = {"red": 0.0, "blue": 0.0}
        self.categories_present = []
        self.picked = None

    # -- categories
    def set_category_visible(self, cat, visible: bool):
        self.cat_visible[cat] = bool(visible)
        for a in self.cat_actors.get(cat, []):
            a.visible = bool(visible)

    def toggle_category(self, cat):
        self.set_category_visible(cat, not self.cat_visible.get(cat, True))
        return self.cat_visible[cat]

    # -- hive
    def _group_def(self, alliance):
        return self.field.model.groups["hive_" + alliance]

    def set_hive_angle(self, alliance, deg: float):
        gd = self._group_def(alliance)
        M = hive.rotation_about_axis(gd["pivot"], gd["axis"], float(deg) - float(gd["cad_pose_angle_deg"]))
        for a in self.hive_actors[alliance]:
            a.matrix = M
        self.angles[alliance] = float(deg)

    def apply_state_instant(self, alliance, state):
        self.field.set_tip(alliance, state)
        self.set_hive_angle(alliance, hive.tip_angle(self._group_def(alliance), state))
        self.refresh_dynamic()

    def animate_tip(self, targets: dict, duration=1.0, steps=24):
        """targets: {alliance: state}.  Animates both (or one) hive(s) then refreshes overlays."""
        start = {al: self.angles[al] for al in targets}
        end = {al: hive.tip_angle(self._group_def(al), st) for al, st in targets.items()}
        for al, st in targets.items():
            self.field.set_tip(al, st)
        self._set_dynamic_visible(False)
        for i in range(1, steps + 1):
            u = i / steps
            u = u * u * (3 - 2 * u)
            for al in targets:
                self.set_hive_angle(al, start[al] + (end[al] - start[al]) * u)
            self.plotter.render_hook()
            time.sleep(duration / steps)
        for al in targets:
            self.set_hive_angle(al, end[al])
        self.refresh_dynamic()
        self._set_dynamic_visible(True)
        self.plotter.render_hook()

    def toggle_tip(self, alliances=("red", "blue"), animate=True):
        tg = {al: hive.other_state(self.field.hive_state[al]) for al in alliances}
        if animate:
            self.animate_tip(tg)
        else:
            for al, st in tg.items():
                self.apply_state_instant(al, st)
        return tg

    def reset_tips(self):
        tg = {al: hive.cad_state(self._group_def(al)) for al in ("red", "blue")}
        self.animate_tip(tg, duration=0.5, steps=10)

    # -- overlays
    _DYN = ("tag_squares", "tag_normals", "tag_labels", "aim_markers", "aim_labels")

    def _set_dynamic_visible(self, vis):
        for n in self._DYN:
            self.plotter.set_overlay_visible(n, vis)

    def refresh_dynamic(self):
        p, f, o = self.plotter, self.field, self.opts
        if o.show_tags or o.labels:
            if o.show_tags:
                draw_polylines(p, overlays.tag_square_polylines(f), "tag_squares", width=2.0)
                draw_arrows(p, overlays.tag_normal_arrows(f), "tag_normals")
            if o.labels:
                draw_labels(p, overlays.tag_labels(f), "tag_labels", font_size=11)
        if o.show_aim_points:
            draw_markers(p, overlays.aim_point_markers(f), "aim_markers")
            if o.labels:
                draw_labels(p, overlays.aim_point_labels(f), "aim_labels", font_size=11)

    def screenshot(self, path):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        self.plotter.screenshot(str(path), ssaa=2)
        return path

    def on_pick(self, point):
        pt = np.asarray(point, float)
        xf, yf, _ = pedro_to_ftc(pt[0], pt[1], 0.0)
        self.picked = pt
        print(f"PICK Pedro (x={pt[0]:.2f}, y={pt[1]:.2f}, z={pt[2]:.2f}) in | "
              f"FTC (x={xf:.2f}, y={yf:.2f}, z={pt[2]:.2f}) in", flush=True)


def build_scene(plotter, field, opts: SceneOpts | None = None) -> SceneHandles:
    opts = opts or SceneOpts()
    model = field.model
    h = SceneHandles(plotter, field, opts)
    from .loader import iter_meshes_by_category
    groups = iter_meshes_by_category(model)

    hidden = set(opts.hide)
    only = set(opts.only)
    if opts.lod == "low":
        hidden |= set(LOW_LOD_HIDDEN)

    present = []
    for (cat, grp), items in groups.items():
        if not items:
            continue
        V, F, cols, nf = _merge_instances(model, items)
        pal = CATEGORY_PALETTE.get(cat, (180, 180, 180))
        rgba = np.empty((len(F), 4), np.uint8)
        pos = 0
        for fcol, n in zip(cols, nf):
            if fcol is not None and len(fcol) == n:
                rgba[pos:pos + n] = fcol
            else:
                rgba[pos:pos + n] = (*pal, 255)
            pos += n
        alpha = float(rgba[:, 3].mean()) / 255.0
        opacity = alpha if alpha < 0.97 else 1.0
        actor = plotter.add_actor(Actor(f"{cat}:{grp}", V, F, rgba[:, :3].copy(), opacity=opacity))
        h.actors[(cat, grp)] = actor
        h.cat_actors.setdefault(cat, []).append(actor)
        if cat not in present:
            present.append(cat)
        if grp and grp.startswith("hive_") and grp[5:] in h.hive_actors:
            h.hive_actors[grp[5:]].append(actor)
    order = {c: i for i, c in enumerate(CATEGORIES)}
    h.categories_present = sorted(present, key=lambda c: order.get(c, 99))
    for cat in h.categories_present:
        vis = cat not in hidden and (not only or cat in only)
        h.set_category_visible(cat, vis)

    # initial hive tip state
    for al, st in (("red", field.hive_state["red"]), ("blue", field.hive_state["blue"])):
        if "hive_" + al in model.groups:
            h.set_hive_angle(al, hive.tip_angle(model.groups["hive_" + al], st))

    # static overlays
    if opts.zones:
        draw_polylines(plotter, overlays.tape_zone_polylines(field), "zones", width=3.0)
    draw_polylines(plotter, [overlays.field_outline_polyline()], "outline", width=3.0)
    if opts.grid:
        draw_polylines(plotter, overlays.grid_polylines(), "grid", width=1.0, opacity=0.5)
    if opts.labels and opts.zones:
        draw_labels(plotter, overlays.zone_labels(field), "zone_labels", font_size=11)
    if opts.alliance_axes:
        draw_arrows(plotter, overlays.axes_arrows(opts.frame), "axes")
        draw_labels(plotter, overlays.axes_labels(opts.frame), "axes_labels", font_size=13)
    if opts.robot_box and opts.pose is not None:
        x, y, hd = opts.pose
        draw_polylines(plotter, overlays.robot_box_polylines(x, y, hd), "robot_box", width=3.0)
        draw_arrows(plotter, [overlays.robot_heading_arrow(x, y, hd)], "robot_heading")
    h.refresh_dynamic()
    return h
