#!/usr/bin/env python
"""Convert an Onshape robot export (glTF/GLB/STL/OBJ, or a zip of them) into web/data/robot.glb.

  .venv\\Scripts\\python.exe tools\\import_robot_cad.py MaxBot.glb
  .venv\\Scripts\\python.exe tools\\import_robot_cad.py MaxBot.stl --units mm --up Z --forward +X
  .venv\\Scripts\\python.exe tools\\import_robot_cad.py parts.zip            # every .stl/.obj inside becomes a named node
  .venv\\Scripts\\python.exe tools\\import_robot_cad.py --onshape-url "<document url>"   # needs ONSHAPE_ACCESS_KEY / ONSHAPE_SECRET_KEY

What it does (numpy + stdlib only):
  * .glb / .gltf are validated and re-packed as one .glb (external .bin and data URIs are embedded). The geometry and node
    names are NOT changed: the browser loader (web/js/robotglb.js) does the unit / up-axis / fit normalisation.
  * .stl / .obj (and zips of them) are triangulated and converted to metres (--units, default mm). Axes stay in the SOURCE
    frame (Onshape: Z-up) unless --bake-axes is given; otherwise tell the loader: orientation {upAxis:'Z', forwardAxis:'+X'}.
  * One node per part file/object, named after it, so 'turret', 'hood', 'flywheel', 'intake', 'flower' in a name become
    animatable nodes. Vertices keep CAD coordinates (assembly origin); --pivot-from-name re-bases turret/hood/flywheel node
    origins on their bbox centre (a guess: set mate origins in Onshape for exact axes).

Exit code 0 on success. Prints size, vertex/triangle counts and a bbox in metres.
"""
import argparse, base64, hmac, hashlib, io, json, os, struct, sys, time, uuid, zipfile
from datetime import datetime, timezone
from email.utils import formatdate
from urllib import request, parse
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_OUT = os.path.join(ROOT, 'web', 'data', 'robot.glb')
UNIT_TO_M = {'m': 1.0, 'cm': 0.01, 'mm': 0.001, 'in': 0.0254}
PIVOT_WORDS = ('turret', 'hood', 'flywheel')
GLB_MAGIC, CH_JSON, CH_BIN = 0x46546C67, 0x4E4F534A, 0x004E4942


# ---------------------------------------------------------------- readers -> list of (name, (N,3,3) float64 triangles)
def read_stl(data: bytes):
    if len(data) >= 84:
        n = struct.unpack_from('<I', data, 80)[0]
        if 84 + 50 * n == len(data):                          # binary STL
            rec = np.frombuffer(data, dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]), count=n, offset=84)
            return rec['v'].astype(np.float64)
    txt = data.decode('ascii', 'ignore').split()
    pts = []
    i = 0
    while i < len(txt) - 3:
        if txt[i] == 'vertex':
            pts.append((float(txt[i + 1]), float(txt[i + 2]), float(txt[i + 3]))); i += 4
        else:
            i += 1
    if len(pts) % 3 or not pts:
        raise ValueError('not a valid STL (binary size mismatch and no ascii vertices)')
    return np.array(pts, dtype=np.float64).reshape(-1, 3, 3)


def read_obj(data: bytes):
    """Returns {object name: (N,3,3)}; supports 'o'/'g' groups, negative indices, quads and n-gons (fan)."""
    verts, groups, cur = [], {}, 'obj'
    for line in data.decode('utf8', 'ignore').splitlines():
        t = line.split()
        if not t:
            continue
        if t[0] == 'v':
            verts.append([float(t[1]), float(t[2]), float(t[3])])
        elif t[0] in ('o', 'g') and len(t) > 1:
            cur = ' '.join(t[1:])
        elif t[0] == 'f':
            ix = [int(p.split('/')[0]) for p in t[1:]]
            ix = [i - 1 if i > 0 else len(verts) + i for i in ix]
            for k in range(1, len(ix) - 1):
                groups.setdefault(cur, []).append([verts[ix[0]], verts[ix[k]], verts[ix[k + 1]]])
    return {k: np.array(v, dtype=np.float64) for k, v in groups.items() if v}


def parts_from_file(name, data: bytes):
    ext = os.path.splitext(name)[1].lower()
    base = os.path.splitext(os.path.basename(name))[0]
    if ext == '.stl':
        return [(base, read_stl(data))]
    if ext == '.obj':
        g = read_obj(data)
        return [(k if len(g) > 1 else base, v) for k, v in g.items()]
    raise ValueError('unsupported mesh file ' + name)


# ---------------------------------------------------------------- writer
def weld(tris):
    """(N,3,3) -> (indexed verts float32 (M,3), uint32 indices); drops degenerate triangles."""
    flat = tris.reshape(-1, 3)
    q = np.round(flat / max(1e-9, float(np.abs(flat).max()) * 1e-7)).astype(np.int64)
    _, first, inv = np.unique(q, axis=0, return_index=True, return_inverse=True)
    v = flat[first]
    idx = inv.reshape(-1).reshape(-1, 3)
    keep = (idx[:, 0] != idx[:, 1]) & (idx[:, 1] != idx[:, 2]) & (idx[:, 0] != idx[:, 2])
    return v.astype(np.float32), idx[keep].astype(np.uint32).reshape(-1)


def axis_matrix(up, forward):
    ax = {'+X': (1, 0, 0), '-X': (-1, 0, 0), '+Y': (0, 1, 0), '-Y': (0, -1, 0), '+Z': (0, 0, 1), '-Z': (0, 0, -1)}
    norm = lambda s: s.upper() if s[0] in '+-' else '+' + s.upper()
    u, f = np.array(ax[norm(up)], float), np.array(ax[norm(forward)], float)
    if abs(u @ f) > 0.5:
        raise ValueError('--forward must be perpendicular to --up')
    # target glTF frame: +Y up, +X forward, +Z right (right-handed): rows map file coords -> (fwd, up, right)
    right = np.cross(f, u)
    return np.stack([f, u, right])


def build_glb(parts, pivot_from_name=False, bake=None):
    """parts: list of (name, tris in metres). Returns (glb bytes, stats)."""
    chunks, views, accs, meshes, nodes = [], [], [], [], []
    off = 0
    allv = []

    def add(arr, target=None):
        nonlocal off
        b = arr.tobytes()
        pad = (-len(b)) % 4
        views.append({'buffer': 0, 'byteOffset': off, 'byteLength': len(b), **({'target': target} if target else {})})
        chunks.append(b + b'\0' * pad); off += len(b) + pad
        return len(views) - 1

    for name, tris in parts:
        if bake is not None:
            tris = tris @ bake.T
        v, ix = weld(tris)
        if len(ix) == 0:
            print('skipping empty part', name, file=sys.stderr); continue
        translation = None
        if pivot_from_name and any(w in name.lower() for w in PIVOT_WORDS):
            c = ((v.min(0) + v.max(0)) / 2).astype(np.float32)
            v = v - c; translation = [float(x) for x in c]
        allv.append(v if translation is None else v + np.array(translation, np.float32))
        pv = add(v, 34962)
        accs.append({'bufferView': pv, 'componentType': 5126, 'count': int(len(v)), 'type': 'VEC3', 'min': [float(x) for x in v.min(0)], 'max': [float(x) for x in v.max(0)]})
        iv = add(ix, 34963)
        accs.append({'bufferView': iv, 'componentType': 5125, 'count': int(len(ix)), 'type': 'SCALAR'})
        meshes.append({'name': name, 'primitives': [{'attributes': {'POSITION': len(accs) - 2}, 'indices': len(accs) - 1, 'material': 0}]})
        node = {'name': name, 'mesh': len(meshes) - 1}
        if translation:
            node['translation'] = translation
        nodes.append(node)
    if not nodes:
        raise ValueError('no triangles found')
    root = {'name': 'Robot', 'children': list(range(1, len(nodes) + 1))}
    all_nodes = [root] + nodes
    bin_ = b''.join(chunks)
    js = {'asset': {'version': '2.0', 'generator': 'tools/import_robot_cad.py'}, 'scene': 0, 'scenes': [{'nodes': [0]}], 'nodes': all_nodes,
          'meshes': meshes, 'materials': [{'name': 'cad', 'pbrMetallicRoughness': {'baseColorFactor': [0.72, 0.74, 0.78, 1], 'metallicFactor': 0.4, 'roughnessFactor': 0.55}, 'doubleSided': True}],
          'accessors': accs, 'bufferViews': views, 'buffers': [{'byteLength': len(bin_)}]}
    cat = np.concatenate(allv)
    return pack_glb(js, bin_), {'parts': len(nodes), 'verts': int(len(cat)), 'tris': int(sum(a['count'] for a in accs if a['type'] == 'SCALAR') // 3),
                               'bbox_min': cat.min(0).tolist(), 'bbox_max': cat.max(0).tolist()}


def pack_glb(js, bin_):
    j = json.dumps(js, separators=(',', ':')).encode()
    j += b' ' * ((-len(j)) % 4)
    bin_ += b'\0' * ((-len(bin_)) % 4)
    total = 12 + 8 + len(j) + 8 + len(bin_)
    return struct.pack('<III', GLB_MAGIC, 2, total) + struct.pack('<II', len(j), CH_JSON) + j + struct.pack('<II', len(bin_), CH_BIN) + bin_


# ---------------------------------------------------------------- glTF / GLB pass-through
def read_glb_container(data: bytes):
    magic, ver, length = struct.unpack_from('<III', data, 0)
    if magic != GLB_MAGIC or ver != 2 or length > len(data):
        raise ValueError('bad GLB header')
    off, js, bin_ = 12, None, b''
    while off + 8 <= length:
        n, t = struct.unpack_from('<II', data, off)
        body = data[off + 8: off + 8 + n]
        if t == CH_JSON: js = json.loads(body.decode('utf8'))
        elif t == CH_BIN: bin_ = bytes(body)
        off += 8 + n
    if js is None: raise ValueError('GLB has no JSON chunk')
    return js, bin_


def gltf_to_glb(path, data: bytes, zf=None):
    if data[:4] == b'glTF':
        js, bin_ = read_glb_container(data)
    else:
        js, bin_ = json.loads(data.decode('utf8')), b''
    bad = [e for e in (js.get('extensionsRequired') or []) + (js.get('extensionsUsed') or []) if 'draco' in e.lower() or 'meshopt' in e.lower()]
    if bad:
        raise ValueError('compressed glTF (%s): re-export from Onshape with Compress OFF' % ', '.join(bad))
    pieces, newbufs = [], []
    for b in js.get('buffers', []):
        uri = b.get('uri')
        if uri is None: raw = bin_
        elif uri.startswith('data:'): raw = base64.b64decode(uri.split(',', 1)[1])
        else:
            if zf is not None: raw = zf.read(uri)
            else: raw = open(os.path.join(os.path.dirname(path), parse.unquote(uri)), 'rb').read()
        pieces.append(raw)
    # concatenate all buffers into one, shifting bufferViews
    offs, cur, out = [], 0, b''
    for raw in pieces:
        pad = (-len(raw)) % 4
        offs.append(cur); out += raw + b'\0' * pad; cur += len(raw) + pad
    for bv in js.get('bufferViews', []):
        bv['byteOffset'] = bv.get('byteOffset', 0) + offs[bv['buffer']]; bv['buffer'] = 0
    js['buffers'] = [{'byteLength': len(out)}]
    for k in ('images', 'textures', 'samplers'):                # URI images cannot be embedded cheaply; the loader ignores textures anyway
        js.pop(k, None)
    for m in js.get('materials', []):
        for sub in (m.get('pbrMetallicRoughness') or {},):
            sub.pop('baseColorTexture', None); sub.pop('metallicRoughnessTexture', None)
        m.pop('normalTexture', None); m.pop('occlusionTexture', None); m.pop('emissiveTexture', None)
    return pack_glb(js, out), summarize_gltf(js, out)


def summarize_gltf(js, bin_):
    n_nodes = len(js.get('nodes', []))
    mn, mx = [1e30] * 3, [-1e30] * 3
    for me in js.get('meshes', []):
        for pr in me.get('primitives', []):
            a = js['accessors'][pr['attributes']['POSITION']]
            if 'min' in a and 'max' in a:
                mn = [min(x, y) for x, y in zip(mn, a['min'])]; mx = [max(x, y) for x, y in zip(mx, a['max'])]
    return {'nodes': n_nodes, 'meshes': len(js.get('meshes', [])), 'accessor_bbox_min': mn, 'accessor_bbox_max': mx,
            'names': [n.get('name') for n in js.get('nodes', []) if n.get('name')][:40]}


# ---------------------------------------------------------------- Onshape REST (optional; NOT exercised offline)
def onshape_get_or_post(method, path, query='', body=None, ak=None, sk=None, base='https://cad.onshape.com'):
    """HMAC-signed request per https://onshape-public.github.io/docs/auth/apikeys/ (UNVERIFIED against a live account)."""
    nonce = uuid.uuid4().hex[:25]
    date = formatdate(timeval=None, localtime=False, usegmt=True)
    ctype = 'application/json'
    s = ('\n'.join([method, nonce, date, ctype, path, query, '']) + '').lower()
    sig = base64.b64encode(hmac.new(sk.encode(), s.encode(), hashlib.sha256).digest()).decode()
    headers = {'Date': date, 'On-Nonce': nonce, 'Content-Type': ctype, 'Accept': 'application/json',
               'Authorization': 'On %s:HmacSHA256:%s' % (ak, sig)}
    req = request.Request(base + path + ('?' + query if query else ''), data=json.dumps(body).encode() if body is not None else None, headers=headers, method=method)
    return request.urlopen(req, timeout=60)


def onshape_download_gltf(url, out_path):
    ak, sk = os.environ.get('ONSHAPE_ACCESS_KEY'), os.environ.get('ONSHAPE_SECRET_KEY')
    if not ak or not sk:
        raise SystemExit('set ONSHAPE_ACCESS_KEY and ONSHAPE_SECRET_KEY (Onshape: My Account > Developer > API keys). Never commit them.')
    u = parse.urlparse(url)
    parts = u.path.strip('/').split('/')                      # documents/{did}/(w|v)/{wvid}/e/{eid}
    try:
        did, wv, wvid, eid = parts[1], parts[2], parts[3], parts[5]
    except IndexError:
        raise SystemExit('cannot parse the document URL; expected .../documents/<did>/w/<wid>/e/<eid>')
    body = {'format': 'GLTF', 'angularTolerance': 0.001, 'distanceTolerance': 0.001, 'maximumChordLength': 0.01, 'resolution': 'FINE', 'unit': 'METER', 'storeInDocument': False}
    # Part Studio endpoint; for an Assembly tab replace 'partstudios' with 'assemblies'. Endpoint shape from the public docs, UNVERIFIED here.
    path = '/api/partstudios/d/%s/%s/%s/e/%s/export/gltf' % (did, wv, wvid, eid)
    r = json.loads(onshape_get_or_post('POST', path, ak=ak, sk=sk, body=body).read())
    tid = r.get('id')
    for _ in range(120):
        t = json.loads(onshape_get_or_post('GET', '/api/translations/' + tid, ak=ak, sk=sk).read())
        if t.get('requestState') == 'DONE': break
        if t.get('requestState') == 'FAILED': raise SystemExit('Onshape translation failed: ' + str(t.get('failureReason')))
        time.sleep(2)
    else:
        raise SystemExit('Onshape translation timed out')
    ids = t.get('resultExternalDataIds') or []
    if not ids: raise SystemExit('no external data id in translation result (storeInDocument false); see docs/CAD_IMPORT.md')
    data = onshape_get_or_post('GET', '/api/documents/d/%s/externaldata/%s' % (did, ids[0]), ak=ak, sk=sk).read()
    open(out_path, 'wb').write(data)
    return out_path


# ---------------------------------------------------------------- main
def convert(path_or_bytes, name, args):
    data = path_or_bytes if isinstance(path_or_bytes, bytes) else open(path_or_bytes, 'rb').read()
    ext = os.path.splitext(name)[1].lower()
    if ext in ('.glb', '.gltf'):
        glb, st = gltf_to_glb(name, data)
        return glb, st
    parts = []
    scale = UNIT_TO_M[args.units]
    if ext == '.zip':
        with zipfile.ZipFile(io.BytesIO(data)) as zf:
            names = [n for n in zf.namelist() if n.lower().endswith(('.stl', '.obj'))]
            gl = [n for n in zf.namelist() if n.lower().endswith(('.glb', '.gltf'))]
            if not names and gl:
                return gltf_to_glb(gl[0], zf.read(gl[0]), zf)
            for n in sorted(names):
                parts += parts_from_file(n, zf.read(n))
    else:
        parts = parts_from_file(name, data)
    parts = [(n, t * scale) for n, t in parts]
    bake = axis_matrix(args.up, args.forward) if args.bake_axes else None
    return build_glb(parts, args.pivot_from_name, bake)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('input', nargs='?', help='.glb .gltf .stl .obj or .zip')
    ap.add_argument('-o', '--out', default=DEFAULT_OUT)
    ap.add_argument('--units', default='mm', choices=sorted(UNIT_TO_M), help='units of STL/OBJ input (default mm); ignored for glTF (always metres)')
    ap.add_argument('--up', default='Z'); ap.add_argument('--forward', default='+X')
    ap.add_argument('--bake-axes', action='store_true', help='rotate STL/OBJ geometry into glTF Y-up / +X forward (otherwise pass orientation to the loader)')
    ap.add_argument('--pivot-from-name', action='store_true', help='re-base turret/hood/flywheel node origins on their bbox centre')
    ap.add_argument('--onshape-url', help='download a glTF via the REST API (env ONSHAPE_ACCESS_KEY / ONSHAPE_SECRET_KEY)')
    a = ap.parse_args(argv)
    src = a.input
    if a.onshape_url:
        src = onshape_download_gltf(a.onshape_url, os.path.join(os.path.dirname(a.out), '_onshape_download.glb'))
    if not src:
        ap.error('give an input file or --onshape-url')
    if not os.path.exists(src):
        print('input not found: ' + src, file=sys.stderr); return 2
    try:
        glb, st = convert(src, os.path.basename(src), a)
    except (ValueError, KeyError, zipfile.BadZipFile, struct.error) as e:
        print('conversion failed: %s' % e, file=sys.stderr); return 1
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    with open(a.out, 'wb') as f:
        f.write(glb)
    print('wrote %s (%.1f kB)' % (a.out, len(glb) / 1024)); print(json.dumps(st, indent=1))
    return 0


if __name__ == '__main__':
    sys.exit(main())
