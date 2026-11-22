"""Validate field_model/parts.json and every meshes/*.npz. Exit 1 on failure."""
import sys, os, json
import numpy as np
FS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
fm = os.path.join(FS, 'data', 'field_model')
errs = []
pj = json.load(open(os.path.join(fm, 'parts.json')))
parts = pj['parts']
man = json.load(open(os.path.join(FS, 'data', 'raw', 'part_manifest.json')))
for m in man:
    if 'p%03d' % m['n'] not in parts: errs.append('missing part p%03d' % m['n'])
tri = tri0 = 0; size = 0; nsyn = ndec = 0; inv = []
for k, p in sorted(parts.items()):
    path = os.path.join(fm, p['mesh'])
    if not os.path.exists(path): errs.append(k + ': mesh file missing'); continue
    size += os.path.getsize(path)
    z = np.load(path); v = z['vertices']; f = z['faces']
    if v.dtype != np.float32 or v.ndim != 2 or v.shape[1] != 3: errs.append(k + ': bad vertices'); continue
    if f.dtype != np.int32 or f.ndim != 2 or f.shape[1] != 3: errs.append(k + ': bad faces'); continue
    if not np.isfinite(v).all(): errs.append(k + ': NaN/inf')
    if len(f) == 0 or f.min() < 0 or f.max() >= len(v): errs.append(k + ': index range')
    if 'face_colors' in z.files:
        c = z['face_colors']
        if c.dtype != np.uint8 or c.shape != (len(f), 4): errs.append(k + ': bad face_colors')
    if p['triangles'] != len(f): errs.append(k + ': triangles mismatch %d vs %d' % (p['triangles'], len(f)))
    if np.abs(v.min(0) - p['bbox_min']).max() > 1e-3 or np.abs(v.max(0) - p['bbox_max']).max() > 1e-3: errs.append(k + ': bbox mismatch')
    if len(np.unique(v, axis=0)) != len(v): errs.append(k + ': duplicate vertices')
    # signed volume
    a = v[f[:, 0]].astype(np.float64); b = v[f[:, 1]]; c2 = v[f[:, 2]]
    vol = np.einsum('ij,ij->i', a, np.cross(b, c2)).sum() / 6
    if vol < 0: inv.append(k)
    tri += len(f); tri0 += p['triangles_original']; nsyn += p['source'] == 'synthetic'; ndec += bool(p['decimated'])
    for key in ('name', 'onshape_key', 'source', 'default_color', 'notes'):
        if key not in p: errs.append(k + ': missing ' + key)
print('parts %d  triangles %d (original %d)  meshes %.2f MB  decimated %d  synthetic %d  negative-volume %s' % (
    len(parts), tri, tri0, size / 1e6, ndec, nsyn, inv))
print('sources:', {s: sum(1 for p in parts.values() if p['source'] == s) for s in ('onshape', 'step', 'synthetic')})
if errs:
    print('FAIL'); [print(' ', e) for e in errs]; sys.exit(1)
print('OK')
