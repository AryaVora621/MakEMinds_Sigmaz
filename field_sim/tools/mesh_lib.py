"""Shared mesh decoder for Onshape BTExportTessellatedFacesResponse JSON (S1)."""
import json, base64
import numpy as np
M2IN = 1.0 / 0.0254

def decode_tess(path):
    """Return vertices float64 (N,3) inches, faces int32 (M,3), face_colors uint8 (M,4), n_bodies.
    Colour = base64 RGB in bodies[].appearance.color (+ opacity); faces' own appearance used if non-null."""
    d = json.load(open(path, encoding='utf-8'))
    tri = []; cols = []
    for b in d.get('bodies', []):
        ap = b.get('appearance') or {}
        def rgba(a):
            try:
                rgb = base64.b64decode(a['color'])
                return [rgb[0], rgb[1], rgb[2], int(a.get('opacity', 255))]
            except Exception:
                return [200, 200, 200, 255]
        bc = rgba(ap) if ap else [200, 200, 200, 255]
        for f in b.get('faces', []):
            fc = rgba(f['appearance']) if f.get('appearance') else bc
            for ft in f.get('facets', []):
                v = ft['vertices']
                if len(v) < 3: continue
                for k in range(1, len(v) - 1):  # fan (normally 3 verts)
                    tri.append([(v[0]['x'], v[0]['y'], v[0]['z']), (v[k]['x'], v[k]['y'], v[k]['z']),
                                (v[k+1]['x'], v[k+1]['y'], v[k+1]['z'])])
                    cols.append(fc)
    if not tri:
        return np.zeros((0, 3)), np.zeros((0, 3), np.int32), np.zeros((0, 4), np.uint8)
    t = np.asarray(tri, dtype=np.float64).reshape(-1, 3) * M2IN
    return dedupe(t, np.asarray(cols, np.uint8))

def dedupe(tri_pts, cols=None):
    """tri_pts (3M,3) -> unique verts (tol 1e-6 in), faces; drops degenerate faces."""
    q = np.round(tri_pts / 1e-5).astype(np.int64)
    _, idx, inv = np.unique(q, axis=0, return_index=True, return_inverse=True)
    v = tri_pts[idx]; f = inv.reshape(-1, 3).astype(np.int32)
    ok = (f[:, 0] != f[:, 1]) & (f[:, 1] != f[:, 2]) & (f[:, 0] != f[:, 2])
    return v, f[ok], (cols[ok] if cols is not None else None)

def cluster_decimate(v, f, cols, target):
    """Vertex-clustering decimation to ~target triangles (pure numpy). Cell size found by bisection.
    Cluster representative = mean of members. Returns v,f,cols, cell size."""
    lo_b, hi_b = v.min(0), v.max(0); ext = (hi_b - lo_b).max()
    def run(h):
        k = np.floor((v - lo_b) / h).astype(np.int64)
        _, first, inv = np.unique(k, axis=0, return_index=True, return_inverse=True)
        inv = inv.ravel()
        n = inv.max() + 1
        s = np.zeros((n, 3)); np.add.at(s, inv, v); c = np.bincount(inv, minlength=n)[:, None]
        nv = s / c
        nf = inv[f]
        ok = (nf[:, 0] != nf[:, 1]) & (nf[:, 1] != nf[:, 2]) & (nf[:, 0] != nf[:, 2])
        return nv, nf[ok].astype(np.int32), ok
    a, b = ext / 4000.0, ext / 2.0
    best = None
    for _ in range(18):
        h = (a * b) ** 0.5
        nv, nf, ok = run(h)
        if len(nf) > target: a = h
        else: b = h; best = (nv, nf, ok, h)
    if best is None: best = (*run(b), b)
    nv, nf, ok, h = best
    # drop duplicate faces
    s = np.sort(nf, axis=1); _, ui = np.unique(s, axis=0, return_index=True); ui.sort()
    nf = nf[ui]; c2 = cols[ok][ui] if cols is not None else None
    used = np.unique(nf); remap = -np.ones(len(nv), np.int64); remap[used] = np.arange(len(used))
    return nv[used], remap[nf].astype(np.int32), c2, h

def make_box(lo, hi):
    """Axis-aligned box lo..hi (inches) -> verts float32 (8,3), faces int32 (12,3) outward CCW."""
    x0, y0, z0 = lo; x1, y1, z1 = hi
    v = np.array([[x0,y0,z0],[x1,y0,z0],[x1,y1,z0],[x0,y1,z0],[x0,y0,z1],[x1,y0,z1],[x1,y1,z1],[x0,y1,z1]], np.float32)
    f = np.array([[0,3,2],[0,2,1],[4,5,6],[4,6,7],[0,1,5],[0,5,4],[1,2,6],[1,6,5],[2,3,7],[2,7,6],[3,0,4],[3,4,7]], np.int32)
    return v, f

def make_cylinder(center_xy, z0, z1, radius, n=32):
    """Z-axis cylinder (inches) with caps -> verts, faces (outward CCW)."""
    cx, cy = center_xy
    a = np.linspace(0, 2*np.pi, n, endpoint=False)
    ring = np.stack([cx + radius*np.cos(a), cy + radius*np.sin(a)], 1)
    v = np.vstack([np.c_[ring, np.full(n, z0)], np.c_[ring, np.full(n, z1)], [[cx, cy, z0]], [[cx, cy, z1]]]).astype(np.float32)
    f = []
    for i in range(n):
        j = (i + 1) % n
        f += [[i, j, n+j], [i, n+j, n+i], [2*n, j, i], [2*n+1, n+i, n+j]]
    return v, np.array(f, np.int32)

def save_npz(path, v, f, color=None):
    """color: RGBA tuple applied to all faces."""
    kw = {}
    if color is not None: kw['face_colors'] = np.tile(np.array(color, np.uint8), (len(f), 1))
    np.savez_compressed(path, vertices=np.asarray(v, np.float32), faces=np.asarray(f, np.int32), **kw)
