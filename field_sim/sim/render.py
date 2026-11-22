"""Pure numpy + Pillow software renderer (z-buffer) used by the field viewer.

Why this exists: on the target machine Windows App Control blocks the compiled VTK and
vispy extension modules, so GPU/VTK stacks cannot even be imported.  numpy, Pillow and
matplotlib/Tk load fine, so the viewer rasterises on the CPU.

Design
* ``Actor``  : merged triangle soup (one per category/group) + 4x4 user matrix + colours.
* ``SoftPlotter`` : camera, actors, named overlay layers (lines, arrows, markers, labels).
* ``SoftPlotter.render(size, ssaa, max_tris)`` -> RGB uint8 image; keeps the z-buffer so
  ``unproject(px, py)`` can return the world point under a pixel (used for picking).
* Sub-pixel triangles are splatted in a vectorised pass, bigger ones go through a
  per-triangle barycentric loop.  A cap (``max_tris``, keeps the largest on screen) keeps
  million-triangle models usable; this is a documented approximation.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field as dc_field
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont


@dataclass
class Camera:
    position: np.ndarray = dc_field(default_factory=lambda: np.array([0.0, -200.0, 100.0]))
    focal: np.ndarray = dc_field(default_factory=lambda: np.array([72.0, 72.0, 15.0]))
    up: np.ndarray = dc_field(default_factory=lambda: np.array([0.0, 0.0, 1.0]))
    fov_deg: float = 30.0          # vertical field of view
    near: float = 1.0

    def set(self, pos, focal, up):
        self.position = np.array(pos, float)
        self.focal = np.array(focal, float)
        self.up = np.array(up, float)

    def basis(self):
        fwd = self.focal - self.position
        fwd = fwd / np.linalg.norm(fwd)
        right = np.cross(fwd, self.up)
        n = np.linalg.norm(right)
        right = right / n if n > 1e-9 else np.array([1.0, 0, 0])
        upv = np.cross(right, fwd)
        return right, upv, fwd

    def orbit(self, d_az_deg, d_el_deg):
        """Rotate the camera around the focal point (z-up world)."""
        v = self.position - self.focal
        r = np.linalg.norm(v)
        az = math.atan2(v[1], v[0]) + math.radians(d_az_deg)
        el = math.asin(np.clip(v[2] / r, -1, 1)) + math.radians(d_el_deg)
        el = float(np.clip(el, math.radians(-89), math.radians(89)))
        self.position = self.focal + r * np.array([math.cos(el) * math.cos(az),
                                                   math.cos(el) * math.sin(az), math.sin(el)])
        self.up = np.array([0.0, 0.0, 1.0])

    def zoom(self, factor):
        v = self.position - self.focal
        self.position = self.focal + v * factor

    def pan(self, dx_in, dy_in):
        right, upv, _ = self.basis()
        d = right * dx_in + upv * dy_in
        self.position = self.position + d
        self.focal = self.focal + d


@dataclass
class Actor:
    name: str
    vertices: np.ndarray            # (N,3) float32, geometry in its own (CAD) frame
    faces: np.ndarray               # (M,3) int
    colors: np.ndarray              # (M,3) uint8
    opacity: float = 1.0
    matrix: np.ndarray = dc_field(default_factory=lambda: np.eye(4))
    visible: bool = True
    normals: np.ndarray | None = None

    def __post_init__(self):
        if self.normals is None:
            v = self.vertices.astype(np.float64)
            f = self.faces
            n = np.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]])
            ln = np.linalg.norm(n, axis=1, keepdims=True)
            self.normals = (n / np.where(ln > 1e-12, ln, 1.0)).astype(np.float32)

    @property
    def n_faces(self):
        return len(self.faces)


class SoftPlotter:
    def __init__(self, window_size=(1280, 800), background=(30, 34, 48)):
        self.window_size = tuple(window_size)
        self.background = tuple(background)
        self.camera = Camera()
        self.actors: list[Actor] = []
        self.overlays: dict[str, dict] = {}     # name -> {"kind","items","visible"}
        self.on_render = None
        self.max_tris = 400000
        self._last = None

    # -- scene
    def add_actor(self, actor: Actor) -> Actor:
        self.actors.append(actor)
        return actor

    def set_overlay(self, name, kind, items, **style):
        items = list(items)
        if not items:
            self.overlays.pop(name, None)
            return
        self.overlays[name] = dict(kind=kind, items=items, visible=True, **style)

    def set_overlay_visible(self, name, vis):
        if name in self.overlays:
            self.overlays[name]["visible"] = vis

    def render_hook(self):
        if self.on_render:
            self.on_render()

    # -- rendering
    def render(self, size=None, ssaa=2, max_tris=None) -> np.ndarray:
        size = size or self.window_size
        W, H = int(size[0]), int(size[1])
        s = max(1, int(ssaa))
        img = self._render_raw(W * s, H * s, s, max_tris or self.max_tris)
        im = Image.fromarray(img)
        if s > 1:
            im = im.resize((W, H), Image.LANCZOS)
        draw = ImageDraw.Draw(im)
        self._draw_labels(draw, W, H)
        out = np.asarray(im).copy()
        return out

    def screenshot(self, path, size=None, ssaa=2):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        Image.fromarray(self.render(size, ssaa)).save(path)
        return path

    # camera helpers
    def _project_setup(self, W, H):
        cam = self.camera
        right, upv, fwd = cam.basis()
        f = (H / 2.0) / math.tan(math.radians(cam.fov_deg) / 2.0)
        return right, upv, fwd, f, W / 2.0, H / 2.0

    def _to_screen(self, P, W, H):
        """P (...,3) world -> sx, sy, z (camera forward distance)."""
        right, upv, fwd, f, cx, cy = self._project_setup(W, H)
        d = P - self.camera.position
        x = d @ right
        y = d @ upv
        z = d @ fwd
        zs = np.where(z > 1e-6, z, 1e-6)
        return cx + f * x / zs, cy - f * y / zs, z

    def _render_raw(self, W, H, s, max_tris):
        cam = self.camera
        bg = np.array(self.background, np.uint8)
        img = np.empty((H, W, 3), np.uint8)
        img[:] = bg
        zb = np.zeros((H, W), np.float32)      # stores 1/z (larger = nearer)
        right, upv, fwd, f, cx, cy = self._project_setup(W, H)
        L1 = -fwd
        L2 = np.array([0.4, -0.5, 0.8])
        L2 = L2 / np.linalg.norm(L2)

        opaque = ([], [], [], [], [], [])       # sx0,sy0,iz0 ... collected as arrays
        P0, P1, P2, COL, ALP = [], [], [], [], []
        for a in self.actors:
            if not a.visible or a.n_faces == 0:
                continue
            R = a.matrix[:3, :3]
            t = a.matrix[:3, 3]
            Vw = a.vertices.astype(np.float64) @ R.T + t
            d = Vw - cam.position
            x = d @ right
            y = d @ upv
            z = d @ fwd
            F = a.faces
            zf = z[F]
            ok = (zf > cam.near).all(axis=1)
            zs = np.where(z > cam.near, z, cam.near)
            sx = cx + f * x / zs
            sy = cy - f * y / zs
            fx = sx[F]
            fy = sy[F]
            ok &= (fx.max(1) >= 0) & (fx.min(1) <= W) & (fy.max(1) >= 0) & (fy.min(1) <= H)
            idx = np.nonzero(ok)[0]
            if len(idx) == 0:
                continue
            Nw = a.normals[idx].astype(np.float64) @ R.T
            lam = 0.55 * np.abs(Nw @ L1) + 0.45 * np.abs(Nw @ L2)
            shade = (0.28 + 0.72 * lam)[:, None]
            col = np.clip(a.colors[idx].astype(np.float64) * shade, 0, 255).astype(np.uint8)
            iz = 1.0 / zf[idx]
            P0.append(np.stack([fx[idx, 0], fy[idx, 0], iz[:, 0]], 1))
            P1.append(np.stack([fx[idx, 1], fy[idx, 1], iz[:, 1]], 1))
            P2.append(np.stack([fx[idx, 2], fy[idx, 2], iz[:, 2]], 1))
            COL.append(col)
            ALP.append(np.full(len(idx), a.opacity, np.float32))
        if P0:
            P0 = np.vstack(P0)
            P1 = np.vstack(P1)
            P2 = np.vstack(P2)
            COL = np.vstack(COL)
            ALP = np.concatenate(ALP)
            self._raster_all(img, zb, P0, P1, P2, COL, ALP, max_tris * s * s // max(1, s * s))

        self._draw_overlays(img, zb, W, H, s)
        self._last = dict(zb=zb, W=W, H=H, s=s, right=right, upv=upv, fwd=fwd, f=f, cx=cx, cy=cy,
                          pos=cam.position.copy())
        return img

    # -- rasterisation
    def _raster_all(self, img, zb, P0, P1, P2, COL, ALP, max_tris):
        x0, y0, x1, y1, x2, y2 = (P0[:, 0], P0[:, 1], P1[:, 0], P1[:, 1], P2[:, 0], P2[:, 1])
        bw = np.maximum(np.maximum(x0, x1), x2) - np.minimum(np.minimum(x0, x1), x2)
        bh = np.maximum(np.maximum(y0, y1), y2) - np.minimum(np.minimum(y0, y1), y2)
        area = np.abs((x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0))
        keep = area > 1e-4
        if keep.sum() > max_tris:
            thr = np.partition(area[keep], -max_tris)[-max_tris]
            keep &= area >= thr
        opaque = keep & (ALP >= 0.97)
        trans = keep & (ALP < 0.97)
        H, W = zb.shape
        # --- splat tiny opaque tris (centroid pixel), vectorised
        small = opaque & (bw < 1.6) & (bh < 1.6)
        si = np.nonzero(small)[0]
        if len(si):
            cxp = ((x0[si] + x1[si] + x2[si]) / 3).astype(np.int64)
            cyp = ((y0[si] + y1[si] + y2[si]) / 3).astype(np.int64)
            izs = (P0[si, 2] + P1[si, 2] + P2[si, 2]) / 3
            inb = (cxp >= 0) & (cxp < W) & (cyp >= 0) & (cyp < H)
            si, cxp, cyp, izs = si[inb], cxp[inb], cyp[inb], izs[inb]
            order = np.argsort(izs)               # ascending: nearest written last
            si, cxp, cyp, izs = si[order], cxp[order], cyp[order], izs[order]
            flat = cyp * W + cxp
            zf = zb.reshape(-1)
            imf = img.reshape(-1, 3)
            # keep only the nearest per pixel
            _, last = np.unique(flat[::-1], return_index=True)
            sel = len(flat) - 1 - last
            zf[flat[sel]] = izs[sel]
            imf[flat[sel]] = COL[si[sel]]
        big = np.nonzero(opaque & ~small)[0]
        self._raster_loop(img, zb, P0, P1, P2, COL, big, None)
        tr = np.nonzero(trans)[0]
        if len(tr):
            tr = tr[np.argsort(P0[tr, 2] + P1[tr, 2] + P2[tr, 2])]   # far (small 1/z) first
            self._raster_loop(img, zb, P0, P1, P2, COL, tr, ALP)

    @staticmethod
    def _raster_loop(img, zb, P0, P1, P2, COL, idx, alpha):
        H, W = zb.shape
        a0 = P0[idx].tolist()
        a1 = P1[idx].tolist()
        a2 = P2[idx].tolist()
        for k, i in enumerate(idx.tolist()):
            ax, ay, az = a0[k]
            bx, by, bz = a1[k]
            cx_, cy_, cz = a2[k]
            minx = max(int(math.floor(min(ax, bx, cx_))), 0)
            maxx = min(int(math.ceil(max(ax, bx, cx_))), W)
            miny = max(int(math.floor(min(ay, by, cy_))), 0)
            maxy = min(int(math.ceil(max(ay, by, cy_))), H)
            if minx >= maxx or miny >= maxy:
                continue
            den = (bx - ax) * (cy_ - ay) - (cx_ - ax) * (by - ay)
            if abs(den) < 1e-12:
                continue
            X = np.arange(minx, maxx, dtype=np.float32) + 0.5
            Y = (np.arange(miny, maxy, dtype=np.float32) + 0.5)[:, None]
            l1 = ((X - ax) * (cy_ - ay) - (Y - ay) * (cx_ - ax)) / den
            l2 = ((bx - ax) * (Y - ay) - (by - ay) * (X - ax)) / den
            l0 = 1.0 - l1 - l2
            e = -1e-4
            m = (l0 >= e) & (l1 >= e) & (l2 >= e)
            iz = l0 * az + l1 * bz + l2 * cz
            zsl = zb[miny:maxy, minx:maxx]
            m &= iz > zsl
            if not m.any():
                continue
            isl = img[miny:maxy, minx:maxx]
            if alpha is None:
                zsl[m] = iz[m]
                isl[m] = COL[i]
            else:
                al = float(alpha[i])
                isl[m] = (isl[m].astype(np.float32) * (1 - al) + COL[i].astype(np.float32) * al).astype(np.uint8)

    # -- overlays
    def _draw_overlays(self, img, zb, W, H, s):
        for name, ov in self.overlays.items():
            if not ov["visible"]:
                continue
            kind = ov["kind"]
            if kind == "lines":
                for pl in ov["items"]:
                    pts = np.asarray(pl.points, float).reshape(-1, 3)
                    if getattr(pl, "closed", False) and len(pts) > 2:
                        pts = np.vstack([pts, pts[:1]])
                    self._draw_poly(img, zb, pts, pl.color, getattr(pl, "width", 2.0) * s * ov.get("wscale", 0.6))
            elif kind == "arrows":
                for ar in ov["items"]:
                    self._draw_arrow(img, zb, ar, s * ov.get("wscale", 0.7))
            elif kind == "markers":
                for mk in ov["items"]:
                    self._draw_marker(img, zb, mk, s)

    def _draw_poly(self, img, zb, pts, color, width):
        H, W = zb.shape
        sx, sy, z = self._to_screen(pts, W, H)
        w = max(1, int(round(width)))
        col = np.array(color[:3], np.uint8)
        for i in range(len(pts) - 1):
            if z[i] <= self.camera.near or z[i + 1] <= self.camera.near:
                continue
            L = max(abs(sx[i + 1] - sx[i]), abs(sy[i + 1] - sy[i]))
            n = int(min(max(L * 1.5, 2), 4000))
            t = np.linspace(0, 1, n)
            x = sx[i] + (sx[i + 1] - sx[i]) * t
            y = sy[i] + (sy[i + 1] - sy[i]) * t
            iz = (1 / z[i]) + (1 / z[i + 1] - 1 / z[i]) * t
            self._plot_pixels(img, zb, x, y, iz, col, w)

    @staticmethod
    def _plot_pixels(img, zb, x, y, iz, col, w):
        H, W = zb.shape
        r0 = -(w // 2)
        for dy in range(r0, r0 + w):
            for dx in range(r0, r0 + w):
                xi = (x + dx).astype(np.int64)
                yi = (y + dy).astype(np.int64)
                ok = (xi >= 0) & (xi < W) & (yi >= 0) & (yi < H)
                xi, yi, izo = xi[ok], yi[ok], iz[ok]
                vis = izo * 1.012 >= zb[yi, xi]
                img[yi[vis], xi[vis]] = col

    def _draw_arrow(self, img, zb, ar, wscale):
        o = np.asarray(ar.origin, float)
        v = np.asarray(ar.vector, float)
        L = np.linalg.norm(v)
        if L < 1e-9:
            return
        d = v / L
        tip = o + v
        ref = np.array([0, 0, 1.0]) if abs(d[2]) < 0.9 else np.array([1.0, 0, 0])
        p1 = np.cross(d, ref)
        p1 /= np.linalg.norm(p1)
        p2 = np.cross(d, p1)
        base = tip - d * 0.22 * L
        segs = [np.array([o, tip])]
        for p in (p1, -p1, p2, -p2):
            segs.append(np.array([tip, base + p * 0.09 * L]))
        for sgm in segs:
            self._draw_poly(img, zb, sgm, ar.color, max(1.0, wscale * 2.4))

    def _draw_marker(self, img, zb, mk, s):
        H, W = zb.shape
        p = np.asarray(mk.position, float)[None, :]
        sx, sy, z = self._to_screen(p, W, H)
        if z[0] <= self.camera.near:
            return
        _, _, _, f, _, _ = self._project_setup(W, H)
        r = max(2.0, mk.radius * f / z[0])
        x0, x1 = int(max(sx[0] - r, 0)), int(min(sx[0] + r + 1, W))
        y0, y1 = int(max(sy[0] - r, 0)), int(min(sy[0] + r + 1, H))
        if x0 >= x1 or y0 >= y1:
            return
        X, Y = np.meshgrid(np.arange(x0, x1), np.arange(y0, y1))
        m = (X - sx[0]) ** 2 + (Y - sy[0]) ** 2 <= r * r
        shade = 1.0 - 0.45 * np.clip(((X - sx[0] + 0.3 * r) ** 2 + (Y - sy[0] + 0.3 * r) ** 2) / (r * r), 0, 1)
        col = np.array(mk.color[:3], float)
        sl = img[y0:y1, x0:x1]
        sl[m] = np.clip(col[None, :] * shade[m][:, None] + 40 * (1 - shade[m])[:, None] * 0.0, 0, 255).astype(np.uint8)

    def _draw_labels(self, draw, W, H):
        try:
            font = ImageFont.load_default(size=max(11, H // 55))
        except TypeError:
            font = ImageFont.load_default()
        for name, ov in self.overlays.items():
            if ov["kind"] != "labels" or not ov["visible"]:
                continue
            for lb in ov["items"]:
                p = np.asarray(lb.position, float)[None, :]
                sx, sy, z = self._to_screen(p, W, H)
                if z[0] <= self.camera.near or not (0 <= sx[0] < W and 0 <= sy[0] < H):
                    continue
                draw.text((float(sx[0]), float(sy[0])), lb.text, fill=tuple(int(c) for c in lb.color[:3]),
                          font=font, anchor="mm", stroke_width=2, stroke_fill=(0, 0, 0))

    # -- picking
    def unproject(self, px, py):
        """px,py in the coordinates of the LAST rendered frame's final size. Returns (3,) or None."""
        st = self._last
        if st is None:
            return None
        s = st["s"]
        X = int(np.clip(px * s, 0, st["W"] - 1))
        Y = int(np.clip(py * s, 0, st["H"] - 1))
        izv = float(st["zb"][Y, X])
        xs, ys = X + 0.5, Y + 0.5
        dirc = (st["right"] * (xs - st["cx"]) / st["f"] - st["upv"] * (ys - st["cy"]) / st["f"] + st["fwd"])
        if izv > 0:
            z = 1.0 / izv
            return st["pos"] + dirc * z
        if abs(dirc[2]) > 1e-9:                       # fall back to the z=0 plane
            t = -st["pos"][2] / dirc[2]
            if t > 0:
                return st["pos"] + dirc * t
        return None
