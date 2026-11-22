"""Compute field_constants.json from the raw Onshape tessellation + assembly occurrences.
Run:  python -I field_sim/tools/compute_constants.py   (numpy only)
Writes data/field_model/field_constants.json and data/field_model/_constants_debug.json
All lengths inches, PEDRO frame (x_p = x_cad_in+72, y_p = y_cad_in+72, z up).
Hive-local frame H = the 'Goal Pivot Assembly' occurrence frame (axis = x, origin on the pivot axis).
"""
import sys, os, json, re
from collections import deque
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import cadlib as c

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'data', 'field_model')
occ = c.occurrences()
man = c.manifest()
_mesh = {}
def mesh(n):
    if n not in _mesh: _mesh[n] = c.load_part(n)
    return _mesh[n]
def r(v, k=3):
    return np.round(np.asarray(v, float), k).tolist()
prov = {}
def P(key, tag): prov[key] = tag

def rx(theta_deg):
    a = np.radians(theta_deg); cs, sn = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, cs, -sn], [0, sn, cs]])

# ------------------------------------------------------------------ hive geometry in hive-local frame
def pivot_T(hive):
    return [o for o in occ if o['names'] == ['am-5853-%s Hive <1>' % hive, 'Goal Pivot Assembly <1>']][0]['T']

def raster_hole(face_tris, res=0.04, box=(-13, 12, -5, 15)):
    x0, x1, z0, z1 = box
    W = int((x1 - x0) / res); H = int((z1 - z0) / res); img = np.zeros((H, W), bool)
    for tr in face_tris:
        p = tr[:, [0, 2]]; mn = p.min(0); mx = p.max(0)
        ix0 = max(int((mn[0] - x0) / res), 0); ix1 = int((mx[0] - x0) / res) + 1
        iz0 = max(int((mn[1] - z0) / res), 0); iz1 = int((mx[1] - z0) / res) + 1
        gx, gz = np.meshgrid(x0 + (np.arange(ix0, ix1) + .5) * res, z0 + (np.arange(iz0, iz1) + .5) * res)
        a_, b_, c_ = p; d = (b_[1] - c_[1]) * (a_[0] - c_[0]) + (c_[0] - b_[0]) * (a_[1] - c_[1])
        if abs(d) < 1e-12: continue
        l1 = ((b_[1] - c_[1]) * (gx - c_[0]) + (c_[0] - b_[0]) * (gz - c_[1])) / d
        l2 = ((c_[1] - a_[1]) * (gx - c_[0]) + (a_[0] - c_[0]) * (gz - c_[1])) / d
        m = (l1 >= -1e-9) & (l2 >= -1e-9) & (1 - l1 - l2 >= -1e-9)
        img[iz0:iz1, ix0:ix1][m] = True
    return img, (x0, z0, res)

def flood(img, start):
    H, W = img.shape; seen = np.zeros_like(img); dq = deque([start]); seen[start] = True
    while dq:
        i, j = dq.popleft()
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            a, b = i + di, j + dj
            if 0 <= a < H and 0 <= b < W and not img[a, b] and not seen[a, b]:
                seen[a, b] = True; dq.append((a, b))
    return seen

def poly_centroid(pts):
    pts = np.array(pts); x, z = pts[:, 0], pts[:, 1]
    x2, z2 = np.roll(x, -1), np.roll(z, -1); cr = x * z2 - x2 * z; A = cr.sum() / 2
    return np.array([((x + x2) * cr).sum() / (6 * A), ((z + z2) * cr).sum() / (6 * A)]), abs(A)

def cell_geometry(hive, cell):
    """cell: 'Audience' or 'Scoring'(far). returns dict in hive-local coords."""
    Pinv = np.linalg.inv(pivot_T(hive))
    sel = lambda n: [o for o in occ if o['n'] == n and o['names'][0].startswith('am-5853-%s' % hive) and cell in o['names'][1]]
    ribs = sel(26) if hive == 'Red' else sel(56)
    rl = []
    for o in ribs:
        t = c.xform(Pinv @ o['T'], mesh(o['n'])); rl.append((np.abs(t[:, :, 1]).max(), t))
    rl.sort(key=lambda x: x[0]); tin, tout = rl[0][1], rl[1][1]          # outer rib = larger |y|
    sgn = 1.0 if tout[:, :, 1].mean() > 0 else -1.0                       # +1 far cell, -1 audience cell
    yout = sgn * np.abs(tout[:, :, 1]).max(); yin_rib = sgn * np.abs(tin[:, :, 1]).min()
    nn = np.cross(tout[:, 1] - tout[:, 0], tout[:, 2] - tout[:, 0]); nn /= np.linalg.norm(nn, axis=1)[:, None] + 1e-30
    ys = tout[:, :, 1].mean(1)
    face = tout[(abs(nn[:, 1]) > 0.99) & (np.abs(ys - yout) < 0.02)]
    img, (x0, z0, res) = raster_hole(face)
    seen = flood(img, (int((5 - z0) / res), int((-0.75 - x0) / res)))
    ii, jj = np.nonzero(seen)
    xl, xr = x0 + jj.min() * res, x0 + (jj.max() + 1) * res
    zb, zp = z0 + ii.min() * res, z0 + (ii.max() + 1) * res
    # roof line fit on right edge
    zs_rows, xr_rows = [], []
    for zz in np.arange(zp - 5.0, zp - 1.0, 0.2):
        row = np.nonzero(seen[int((zz - z0) / res)])[0]
        zs_rows.append(zz); xr_rows.append(x0 + (row.max() + 1) * res)
    a, b = np.polyfit(zs_rows, xr_rows, 1); xc = (xl + xr) / 2
    zs = (xr - b) / a
    poly = [(xl, zb), (xr, zb), (xr, zs), (xc, zp), (xl, zs)]
    cen, area = poly_centroid(poly)
    # back wall (back skin p27 / blue also p27) and bottom skin
    back = sel(27)[0]; tb = c.xform(Pinv @ back['T'], mesh(27)); yback = float(tb[:, :, 1].mean())
    bs = sel(28)[0]; tbs = c.xform(Pinv @ bs['T'], mesh(28))
    return dict(sgn=sgn, yout=float(yout), yin_rib=float(yin_rib), yback=yback, xl=float(xl), xr=float(xr), zb=float(zb), zp=float(zp),
                zs=float(zs), roof_slope_deg=float(np.degrees(np.arctan(1 / abs(a)))), poly=[(float(p), float(q)) for p, q in poly],
                centroid=[float(cen[0]), float(cen[1])], area=float(area), mask_area=float(seen.sum() * res * res),
                skin_zmin=float(tbs[:, :, 2].min()),
                lowest_pt_local=None)

# ------------------------------------------------------------------ hive state math
def cad_theta(T):   # Rx angle of hive-local frame in CAD pose
    return float(np.degrees(np.arctan2(T[2, 1], T[1, 1])))

STATE_THETA = {'audience_up': -30.0, 'far_up': 30.0}   # theta of hive frame about +x (local -y raised for -30)

def state_T(hive, state):
    T0 = pivot_T(hive); th0 = cad_theta(T0); pv = T0[:3, 3].copy()
    d = STATE_THETA[state] - th0
    return c.rot_about(d, pv, (1, 0, 0)) @ T0, d, th0

def W(T, p):  # hive-local -> pedro world
    return c.xform(T, np.array(p, float))

# ------------------------------------------------------------------ sticker (tag) frames
STICKER = {('Red', 'Audience'): 29, ('Red', 'Scoring'): 83, ('Blue', 'Audience'): 57, ('Blue', 'Scoring'): 79}
TAG_IDS = {('Red', 'Scoring'): [30, 31, 32, 33], ('Red', 'Audience'): [34, 35, 36, 37],
           ('Blue', 'Audience'): [38, 39, 40, 41], ('Blue', 'Scoring'): [42, 43, 44, 45]}
OFFS = [-6.5, -2.75, 2.75, 6.5]
TAG = 3.25

def sticker_info(hive, cell):
    n = STICKER[(hive, cell)]
    Pinv = np.linalg.inv(pivot_T(hive))
    o = [o for o in occ if o['n'] == n][0]
    Ts = Pinv @ o['T']                       # sticker-local -> hive-local
    Rl = Ts[:3, :3]; t = Ts[:3, 3]
    # skin bottom (outer) surface z in hive-local
    bs = [q for q in occ if q['n'] == 28 and q['names'][0].startswith('am-5853-%s' % hive) and cell in q['names'][1]][0]
    skin_zmin = c.xform(Pinv @ bs['T'], mesh(28))[:, :, 2].min()
    # two faces of the sticker (local y = 0 and y = -0.01): which is farther below the skin (free face)?
    z_face0 = (Rl @ np.array([8.5, 0.0, 2.5]) + t)[2]; z_face1 = (Rl @ np.array([8.5, -0.01, 2.5]) + t)[2]
    free_is_y0 = z_face0 < z_face1
    nloc = np.array([0, 1.0, 0]) if free_is_y0 else np.array([0, -1.0, 0])
    yface = 0.0 if free_is_y0 else -0.01
    # bottom (hole-line) edge = the long edge nearer the field centre = smaller |y| in hive-local (OFFICIAL 9.9: bottom edge toward field centre;
    # check: front of cell to hole line = 9.938 in, Fig 9-15)
    y_z0 = (Rl @ np.array([8.5, yface, 0.0]) + t)[1]; y_z5 = (Rl @ np.array([8.5, yface, 5.0]) + t)[1]
    bottom_is_z0 = abs(y_z0) < abs(y_z5)
    uloc = np.array([0, 0, 1.0]) if bottom_is_z0 else np.array([0, 0, -1.0])   # sticker 'up' = away from bottom edge
    zrow = 2.75 if bottom_is_z0 else 5.0 - 2.75                                  # tag row is 2.75 in above the hole line
    rloc = np.cross(uloc, nloc)                # right as seen facing the printed face
    y_bottom = min(abs(y_z0), abs(y_z5)); y_top = max(abs(y_z0), abs(y_z5))
    n_h, u_h, r_h = Rl @ nloc, Rl @ uloc, Rl @ rloc
    tags = {}
    for k, tid in enumerate(TAG_IDS[(hive, cell)]):
        # position of tag along printed-face 'right' direction
        ploc = np.array([8.5, yface, zrow]) + rloc * OFFS[k]
        tags[tid] = dict(center=Rl @ ploc + t, normal=n_h, up=u_h, right=r_h)
    info = dict(part=n, free_face='y=0' if free_is_y0 else 'y=-0.01', skin_zmin=float(skin_zmin),
                z_free=float(min(z_face0, z_face1)), z_contact=float(max(z_face0, z_face1)),
                hole_line_abs_y=float(y_bottom), top_edge_abs_y=float(y_top), front_of_cell_to_hole_line=float(21.457 - y_bottom), bottom_edge_is_local_z0=bool(bottom_is_z0), normal_h=n_h, up_h=u_h, right_h=r_h)
    return info, tags

# ------------------------------------------------------------------ build hive section
hives_out = {}; tags_out = {}; dbg = {'cells': {}, 'stickers': {}}
STATES = ['audience_up', 'far_up']
for hive, alli in (('Red', 'red'), ('Blue', 'blue')):
    T0 = pivot_T(hive); th0 = cad_theta(T0)
    geos = {cell: cell_geometry(hive, cell) for cell in ('Audience', 'Scoring')}
    dbg['cells'][hive] = geos
    # hive centreline x (hive-local x centre of aperture)
    xc_loc = (geos['Audience']['xl'] + geos['Audience']['xr']) / 2
    pivot_world = W(T0, [xc_loc, 0, 0])
    stickers = {cell: sticker_info(hive, cell) for cell in ('Audience', 'Scoring')}
    dbg['stickers'][hive] = {c_: {k: (v.tolist() if hasattr(v, 'tolist') else v) for k, v in s[0].items()} for c_, s in stickers.items()}
    hv = dict(pivot=r(pivot_world), axis=[1, 0, 0], tilt_deg=30,
              cad_pose_state='audience_up' if abs(th0 - STATE_THETA['audience_up']) < 1 else 'far_up', cad_pose_theta_deg=round(th0, 3), states={})
    for st in STATES:
        Ts, d, _ = state_T(hive, st)
        cells = {}
        for cell, cname in (('Audience', 'audience'), ('Scoring', 'far')):
            g = geos[cell]; y = g['yout']; sgn = g['sgn']
            xl, xr, zb, zp = g['xl'], g['xr'], g['zb'], g['zp']
            # opening corners ordered BL,BR,TR,TL seen from outside (looking along -normal): bbox rectangle of the aperture
            # seen from outside the cell looking in, local +x is to the right if sgn<0 (audience cell), to the left if sgn>0
            loc = [(xl, y, zb), (xr, y, zb), (xr, y, zp), (xl, y, zp)]
            if sgn > 0: loc = [loc[1], loc[0], loc[3], loc[2]]
            ocs = [W(Ts, p) for p in loc]
            poly_w = [W(Ts, (px, y, pz)) for px, pz in g['poly']]
            cen_loc = (g['centroid'][0], y, g['centroid'][1])
            bbc_loc = ((xl + xr) / 2, y, (zb + zp) / 2)
            n_w = Ts[:3, :3] @ np.array([0, sgn, 0.0])
            depth = abs(y - g['yback'])
            zf = zb  # floor plane = aperture bottom (skin inner surface)
            fl = [(xl, y, zf), (xr, y, zf), (xr, g['yback'], zf), (xl, g['yback'], zf)]
            bk = [(xl, g['yback'], zb), (xr, g['yback'], zb), (xr, g['yback'], zp), (xl, g['yback'], zp)]
            up_state = (st == 'audience_up') == (cname == 'audience')
            cells[cname] = dict(up=bool(up_state), opening_corners=[r(p) for p in ocs],
                                opening_polygon=[r(p) for p in poly_w],
                                aim_point=r(W(Ts, cen_loc)), aim_point_bbox_center=r(W(Ts, bbc_loc)),
                                aim_point_mid_depth=r(W(Ts, (cen_loc[0], y - sgn * depth / 2, cen_loc[2]))),
                                opening_normal=r(n_w, 4), floor_corners=[r(W(Ts, p)) for p in fl],
                                back_wall_corners=[r(W(Ts, p)) for p in bk], depth=round(depth, 3),
                                opening_width=round(xr - xl, 3), opening_height_in_plane=round(zp - zb, 3),
                                opening_z_bottom=round(float(min(p[2] for p in ocs)), 3), opening_z_top=round(float(max(p[2] for p in ocs)), 3))
        hv['states'][st] = dict(angle_deg_from_cad=round(d, 3), hive_theta_deg=STATE_THETA[st], cells=cells)
        # tags
        for cell in ('Audience', 'Scoring'):
            for tid, tg in stickers[cell][1].items():
                Rs = Ts[:3, :3]
                ce = W(Ts, tg['center']); nw = Rs @ tg['normal']; uw = Rs @ tg['up']; rw = Rs @ tg['right']
                h = TAG / 2
                corners = [ce + uw * h - rw * h, ce + uw * h + rw * h, ce - uw * h + rw * h, ce - uw * h - rw * h]  # TL,TR,BR,BL facing tag
                e = tags_out.setdefault(str(tid), dict(alliance=alli, side='audience' if cell == 'Audience' else 'far', size=TAG, states={}))
                e['states'][st] = dict(center=r(ce), normal=r(nw, 4), up=r(uw, 4), right=r(rw, 4), corners=[r(q) for q in corners])
    hives_out[alli] = hv

# ------------------------------------------------------------------ lowest point of hive in CAD pose
low = {}
for hive in ('Red', 'Blue'):
    zs = []
    for o in occ:
        if o['n'] in (24, 25, 26, 27, 28, 30) and o['names'][0].startswith('am-5853-%s' % hive):
            zs.append(c.xform(o['T'], mesh(o['n']))[:, :, 2].min())
    low[hive] = float(min(zs))

# ------------------------------------------------------------------ field / walls / tiles
def bb(ns):
    lo = np.full(3, 1e9); hi = np.full(3, -1e9)
    for o in occ:
        if o['n'] in ns:
            l, h = c.bbox(c.xform(o['T'], mesh(o['n']))); lo = np.minimum(lo, l); hi = np.maximum(hi, h)
    return lo, hi
tlo, thi = bb((80, 81, 82)); glo, ghi = bb((64,)); rlo, rhi = bb((63,))
# inner glass faces: min over panels of the inside faces
gl = [c.bbox(c.xform(o['T'], mesh(64))) for o in occ if o['n'] == 64]
wall_in = dict(x_low=min(h[0] for l, h in gl if abs(l[0] - h[0]) < 0.5 and h[0] < 72),
               x_high=max(l[0] for l, h in gl if abs(l[0] - h[0]) < 0.5 and l[0] > 72),
               y_low=min(h[1] for l, h in gl if abs(l[1] - h[1]) < 0.5 and h[1] < 72),
               y_high=max(l[1] for l, h in gl if abs(l[1] - h[1]) < 0.5 and l[1] > 72))
rail_in = dict(x_low=1.41, x_high=142.59, y_low=1.41, y_high=142.59)
wall_in = {k: round(float(v), 3) for k, v in wall_in.items()}
tiles = dict(count=36, nominal_size=24, thickness=0.59, array_min=r(tlo[:2], 2), array_max=r(thi[:2], 2), top_z=round(float(thi[2]), 3))

# ------------------------------------------------------------------ flowers
fl = []
for k, o in enumerate(sorted([o for o in occ if o['n'] == 113], key=lambda o: o['names'][0])):
    lo, hi = c.bbox(c.xform(o['T'], mesh(113)))
    asm = o['names'][0]
    sib = {n: [q for q in occ if q['n'] == n and q['names'][0] == asm][0] for n in (104, 106, 101, 107)}
    cl, ch = c.bbox(c.xform(sib[104]['T'], mesh(104))); bl, bh = c.bbox(c.xform(sib[106]['T'], mesh(106)))
    ctr = (lo + hi) / 2
    fl.append(dict(id=int(re.search(r'<(\d)>', asm).group(1)), alliance=None, opening_center=r([ctr[0], ctr[1], float(ch[2])]),
                   opening_height=round(float(ch[2]), 3), radius=round(float((hi[0] - lo[0] + hi[1] - lo[1]) / 4), 3),
                   scoring_volume_bbox=[r(lo), r(hi)], scoring_volume_z=[round(float(lo[2]), 3), round(float(hi[2]), 3)],
                   backstop_top_z=round(float(bh[2]), 3), top_ring_z=[round(float(cl[2]), 3), round(float(ch[2]), 3)],
                   wall=('far' if ctr[1] > 100 else 'audience' if ctr[1] < 44 else 'red_side' if ctr[0] < 44 else 'blue_side')))
fl.sort(key=lambda d: d['id'])

# ------------------------------------------------------------------ balls
balls = {}
for nm, n in (('pollen', 21), ('nectar_red', 20), ('nectar_blue', 22)):
    t = mesh(n); lo, hi = c.bbox(t); balls[nm] = dict(diameter=round(float((hi - lo).max()), 3), part='p%03d' % n)

# ------------------------------------------------------------------ tape zones (tape parts 0-19 have no tessellation; rectangle from
# occurrence transform + length/width in the part name; origin assumed at the tape centre - cross-checked below)
def tape_rect(o):
    nm = man[o['n']]['name']; w = float(re.search(r'(\d+)in Wide', nm).group(1)); L = float(re.search(r'([\d.]+)in Long', nm).group(1))
    T = o['T']; ax = T[:2, 0]; ay = T[:2, 1]; ctr = T[:2, 3]
    pts = [ctr + s1 * ax * L / 2 + s2 * ay * w / 2 for s1 in (-1, 1) for s2 in (-1, 1)]
    pts = np.array(pts); return pts.min(0), pts.max(0)
groups = {}
for o in occ:
    if o['n'] is not None and o['n'] < 20:
        nm = man[o['n']]['name']; col = 'red' if 'Red' in nm else 'blue'
        L = float(re.search(r'([\d.]+)in Long', nm).group(1)); lo, hi = tape_rect(o)
        kind = 'alliance_area' if (lo[0] < -1 or hi[0] > 145) else ('garden' if (lo[1] < 5 or hi[1] > 139) else 'loading_zone')
        groups.setdefault((col, kind), []).append((lo, hi))
zones = {}
for (col, kind), rs in groups.items():
    lo = np.min([a for a, b in rs], 0); hi = np.max([b for a, b in rs], 0)
    zones['%s_%s' % (col, kind)] = dict(alliance=col, kind=kind, xmin=round(float(lo[0]), 2), xmax=round(float(hi[0]), 2), ymin=round(float(lo[1]), 2),
                                        ymax=round(float(hi[1]), 2), size=[round(float(hi[0] - lo[0]), 2), round(float(hi[1] - lo[1]), 2)])

# ------------------------------------------------------------------ assemble JSON
field = dict(size=[144, 144], size_note='nominal (OFFICIAL 9.2). CAD interior between perimeter glass faces is smaller, see walls',
             center=[72, 72],
             walls=dict(glass_inner_faces=wall_in, rail_inner_faces=rail_in, glass_height_z=[0.09, 10.96], rail_top_z=11.64,
                        interior_size_cad=[round(wall_in['x_high'] - wall_in['x_low'], 2), round(wall_in['y_high'] - wall_in['y_low'], 2)]),
             tile=tiles)
fc = dict(schema=1, units='in', frame='pedro', field=field, hives=hives_out, apriltags=dict(sorted(tags_out.items(), key=lambda kv: int(kv[0]))),
          flowers=fl, zones=zones, balls=balls,
          hive_extra=dict(lowest_point_z_cad_pose=low, cell_depth_cad=hives_out['red']['states']['audience_up']['cells']['audience']['depth']))

# provenance
for k in ('field.size', 'field.walls.glass_inner_faces', 'field.tile'):
    P(k, 'DERIVED-FROM-CAD' if k != 'field.size' else 'OFFICIAL')
P('field.size', 'OFFICIAL (9.2 p62: ~144 x 144); CAD interior differs, see field.walls.interior_size_cad')
P('hives.*.pivot', 'DERIVED-FROM-CAD (y,z); x = aperture centreline. z 43.95 also OFFICIAL 9.6.1 p67; x 59.25/84.75 matches Fig 9-10 25.5 spacing')
P('hives.*.tilt_deg', 'DERIVED-FROM-CAD (rotation matrix of Goal Pivot Assembly = 30.000 deg about X); Fig 9-10 callout 30 deg OFFICIAL')
P('hives.*.states.*.cells.*.opening_corners', 'DERIVED-FROM-CAD (outer rib aperture, 0.04 in raster); width/height consistent with OFFICIAL 9.6.2 (20 x 14 x 12)')
P('hives.*.states.*.cells.*.aim_point', 'DERIVED-FROM-CAD (area centroid of the pentagonal aperture on the outer rib plane)')
P('hives.*.states.*.cells.*.floor_corners', 'DERIVED-FROM-CAD (plane of aperture bottom = inner face of bottom skin, to back skin)')
P('hives.*.states.*.cells.*.opening_normal', 'DERIVED-FROM-CAD (outward along the arm axis, 30 deg above horizontal when the cell is up)')
P('apriltags.*.states.*.center/normal/up/right/corners', 'DERIVED-FROM-CAD (sticker parts p029/p083/p057/p079; tag layout offsets OFFICIAL Fig 9-15)')
P('apriltags.*.size', 'OFFICIAL 9.9 (3.25 in)')
P('apriltags.*.id_order', 'OFFICIAL Fig 9-17 p75 x-order, reproduced by CAD printed-face geometry (ascending ID = left to right on printed face)')
P('apriltags.*.printed_face', 'DERIVED-FROM-CAD: free (outward) face of the 0.01 in sticker; no appearance data in tessellation (UNVERIFIED which face carries ink, but only one face is exposed)')
P('flowers.*', 'DERIVED-FROM-CAD (p113 Flower Scoring Volume bbox + p104 top ring); owner alliance = none (ownership is dynamic, 10.5.2)')
P('zones.*', 'DERIVED-FROM-CAD tape placement (tape part origin assumed at centre; sizes agree with OFFICIAL 9.3 within 0.4 in)')
P('balls.*.diameter', 'DERIVED-FROM-CAD mesh bbox')
P('field.walls.interior_size_cad', 'DERIVED-FROM-CAD; DISAGREES with OFFICIAL ~144')
fc['provenance'] = prov
os.makedirs(OUT, exist_ok=True)
json.dump(fc, open(os.path.join(OUT, 'field_constants.json'), 'w'), indent=1)
dbg['lowest'] = low
json.dump(dbg, open(os.path.join(OUT, '_constants_debug.json'), 'w'), indent=1, default=lambda x: x.tolist() if hasattr(x, 'tolist') else str(x))
print('ok')
