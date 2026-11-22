"""Assemble data/field_model/parts.json from meshes + _onshape_index.json + STEP/synthetic reports (A_S1 scratch)."""
import sys, os, json, glob
import numpy as np
FS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
fm = os.path.join(FS, 'data', 'field_model')
SCR = sys.argv[1]
man = {m['n']: m for m in json.load(open(os.path.join(FS, 'data', 'raw', 'part_manifest.json')))}
oi = {int(k): v for k, v in json.load(open(os.path.join(fm, '_onshape_index.json'))).items()}
rep = {}
for r in ('tape_report.json', 'hardware_report.json'):
    pth = os.path.join(SCR, r)
    if os.path.exists(pth): rep.update(json.load(open(pth)))
parts = {}
for n in range(115):
    k = 'p%03d' % n; m = man[n]
    z = np.load(os.path.join(fm, 'meshes', k + '.npz')); v = z['vertices']; nt = len(z['faces'])
    if n in oi:
        src = 'onshape'; col = oi[n]['color']
        notes = 'Onshape tessellation, metres->inches, vertices deduped (1e-5 in), not decimated. Colour from bodies[].appearance (base64 RGB); %d distinct colour(s).' % oi[n]['ncolors']
    else:
        r = rep.get(k, {}); src = r.get('source', 'step'); col = r.get('color', [180, 180, 185, 255]); notes = r.get('notes', '')
        notes = 'Onshape tessellation unavailable (403/absent); ' + notes
    parts[k] = dict(name=m['name'], onshape_key=m['key'], source=src, mesh='meshes/%s.npz' % k, triangles=nt,
                    triangles_original=nt, decimated=False, bbox_min=[round(float(x), 5) for x in v.min(0)],
                    bbox_max=[round(float(x), 5) for x in v.max(0)], default_color=col, notes=notes)
json.dump(dict(schema=1, units='in', parts=parts), open(os.path.join(fm, 'parts.json'), 'w'), indent=1)
print(len(parts), 'parts written')
