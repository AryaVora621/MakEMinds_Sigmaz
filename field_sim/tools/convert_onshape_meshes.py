"""Convert data/raw/tess_NNN.json -> data/field_model/meshes/pNNN.npz + partial parts index (parts_onshape.json)."""
import sys, os, json, glob
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import mesh_lib
FS = os.path.dirname(HERE); raw = os.path.join(FS, 'data', 'raw'); fm = os.path.join(FS, 'data', 'field_model')
man = {m['n']: m for m in json.load(open(os.path.join(raw, 'part_manifest.json')))}
os.makedirs(os.path.join(fm, 'meshes'), exist_ok=True)
res = {}
for p in sorted(glob.glob(os.path.join(raw, 'tess_*.json'))):
    n = int(os.path.basename(p)[5:8])
    if os.path.getsize(p) < 1000: continue
    v, f, c = mesh_lib.decode_tess(p)
    if len(f) == 0: print('EMPTY', n); continue
    np.savez_compressed(os.path.join(fm, 'meshes', 'p%03d.npz' % n), vertices=v.astype(np.float32), faces=f, face_colors=c)
    u, cnt = np.unique(c, axis=0, return_counts=True)
    res[n] = dict(triangles=int(len(f)), color=[int(x) for x in u[cnt.argmax()]], ncolors=int(len(u)))
    print(n, len(f), flush=True)
json.dump(res, open(os.path.join(fm, '_onshape_index.json'), 'w'))
