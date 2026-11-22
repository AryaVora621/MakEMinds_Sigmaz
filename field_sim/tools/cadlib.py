"""Shared CAD helpers (numpy only). Onshape tessellation + occurrences -> pedro inches.
Run with: python -I (script dir is tools/; cadlib imported via sys.path insert)."""
import json, os, functools
import numpy as np
RAW = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'data', 'raw')
M2IN = 1/0.0254

@functools.lru_cache(None)
def manifest():
    return json.load(open(os.path.join(RAW, 'part_manifest.json')))

@functools.lru_cache(None)
def assembly():
    return json.load(open(os.path.join(RAW, 'assembly_raw.json')))

def load_part(n):
    """-> (T,3,3) float64 triangles, inches, part-local."""
    f = os.path.join(RAW, 'tess_%03d.json' % n)
    if not os.path.exists(f): return None
    d = json.load(open(f))
    if 'bodies' not in d: return None   # parts 0-19 (tape) were 403 in Onshape: STEP only
    tris = []
    for b in d['bodies']:
        for f in b['faces']:
            for ft in f['facets']:
                tris.append([[v['x'], v['y'], v['z']] for v in ft['vertices']])
    return np.array(tris, float).reshape(-1, 3, 3) * M2IN

def load_part_faces(n):
    """-> list of (face_id, color, tris) per face (for appearance)."""
    d = json.load(open(os.path.join(RAW, 'tess_%03d.json' % n)))
    out = []
    for b in d['bodies']:
        for f in b['faces']:
            t = np.array([[[v['x'], v['y'], v['z']] for v in ft['vertices']] for ft in f['facets']], float).reshape(-1, 3, 3) * M2IN
            out.append((f.get('id'), f.get('color') or f.get('appearance'), t))
    return out

def _cfg(i): return i.get('fullConfiguration') or i.get('configuration')

@functools.lru_cache(None)
def _subs():
    a = assembly()
    return [(s['documentId'], s['elementId'], s.get('fullConfiguration'), s) for s in a['subAssemblies']]

def _find_sub(inst):
    for d, e, c, s in _subs():
        if d == inst['documentId'] and e == inst['elementId'] and c == _cfg(inst):
            return s
    for d, e, c, s in _subs():
        if d == inst['documentId'] and e == inst['elementId']:
            return s
    return None

def resolve(occ):
    """-> (names list, leaf instance dict)"""
    a = assembly(); cur = a['rootAssembly']; names = []; inst = None
    for iid in occ['path']:
        inst = next((i for i in cur['instances'] if i['id'] == iid), None)
        if inst is None: return names, None
        names.append(inst['name'])
        if inst['type'] == 'Assembly':
            cur = _find_sub(inst)
            if cur is None: return names, inst
    return names, inst

@functools.lru_cache(None)
def occurrences():
    """list of dict(names, part n or None, T 4x4 pedro inches, inst)"""
    a = assembly(); man = manifest(); out = []
    for o in a['rootAssembly']['occurrences']:
        names, inst = resolve(o)
        n = None
        if inst is not None and inst['type'] == 'Part':
            for m in man:
                if m['documentId'] == inst['documentId'] and m['elementId'] == inst['elementId'] and m['partId'] == inst['partId'] and m['configuration'] == _cfg(inst):
                    n = m['n']; break
            else:
                for m in man:
                    if m['documentId'] == inst['documentId'] and m['elementId'] == inst['elementId'] and m['partId'] == inst['partId'] and m['name'] == inst['name']:
                        n = m['n']; break
        T = np.array(o['transform'], float).reshape(4, 4).copy()
        T[:3, 3] *= M2IN
        T = to_pedro() @ T
        out.append(dict(names=names, n=n, T=T, inst=inst, path=o['path']))
    return out

def to_pedro():
    P = np.eye(4); P[0, 3] = 72; P[1, 3] = 72
    return P

def occ_of(n):
    return [o for o in occurrences() if o['n'] == n]

def xform(T, pts):
    pts = np.asarray(pts, float); sh = pts.shape
    p = pts.reshape(-1, 3)
    return (p @ T[:3, :3].T + T[:3, 3]).reshape(sh)

def rot_about(angle_deg, pivot, axis=(1, 0, 0)):
    """4x4 rotation (right-hand) by angle about axis through pivot."""
    ax = np.array(axis, float); ax /= np.linalg.norm(ax)
    a = np.radians(angle_deg); K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    R = np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * K @ K
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = np.array(pivot) - R @ np.array(pivot)
    return T

def bbox(t):
    t = t.reshape(-1, 3); return t.min(0), t.max(0)
