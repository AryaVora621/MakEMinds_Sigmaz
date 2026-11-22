// Loads the real robot CAD (glTF 2.0 / GLB, e.g. exported from Onshape) into three.js.
// three's GLTFLoader is NOT vendored in web/vendor/addons (no network), so this file contains its own small,
// dependency-free parser: meshes (triangles), simple PBR materials (base colour, metallic, roughness, emissive,
// alpha, double sided), node hierarchy + transforms (matrix or TRS), .glb and .gltf (embedded/data-URI/relative buffers).
// Not supported: animations, skins, morph targets, draco/meshopt, textures (ignored, warning), sparse accessors.
//
// Output frame: robot frame (+x forward, +y left, z up, inches), floor at z=0, chassis bbox centred on x=y=0.
//
//   const m = await loadRobotGLB(THREE, 'data/robot.glb', spec, { orientation: { upAxis: 'Z', forwardAxis: '+X' } });
//   if (m) { m.group ... findMovingParts(m.group) }       // null => use the procedural robot (loadRobotGLB.lastReason says why)

const GLB_MAGIC = 0x46546C67, CH_JSON = 0x4E4F534A, CH_BIN = 0x004E4942;
const COMP = { 5120: [Int8Array, 1], 5121: [Uint8Array, 1], 5122: [Int16Array, 2], 5123: [Uint16Array, 2], 5125: [Uint32Array, 4], 5126: [Float32Array, 4] };
const NCOMP = { SCALAR: 1, VEC2: 2, VEC3: 3, VEC4: 4, MAT2: 4, MAT3: 9, MAT4: 16 };

const toU8 = b => (b instanceof Uint8Array ? b : b instanceof ArrayBuffer ? new Uint8Array(b) : ArrayBuffer.isView(b) ? new Uint8Array(b.buffer, b.byteOffset, b.byteLength) : null);

function b64(str) {
  if (typeof atob === 'function') { const s = atob(str), u = new Uint8Array(s.length); for (let i = 0; i < s.length; i++) u[i] = s.charCodeAt(i); return u; }
  return new Uint8Array(Buffer.from(str, 'base64'));
}

/** Splits a .glb into {json, bin}. Throws on anything malformed. */
export function readGLBContainer(data) {
  const u8 = toU8(data);
  if (!u8 || u8.byteLength < 20) throw new Error('GLB: file too small');
  const dv = new DataView(u8.buffer, u8.byteOffset, u8.byteLength);
  if (dv.getUint32(0, true) !== GLB_MAGIC) throw new Error('GLB: bad magic (not a glTF binary)');
  if (dv.getUint32(4, true) !== 2) throw new Error('GLB: only glTF version 2 is supported');
  const total = dv.getUint32(8, true);
  if (total > u8.byteLength) throw new Error('GLB: truncated file (header says ' + total + ' bytes, got ' + u8.byteLength + ')');
  let off = 12, json = null, bin = null;
  while (off + 8 <= total) {
    const len = dv.getUint32(off, true), type = dv.getUint32(off + 4, true), start = off + 8;
    if (start + len > total) throw new Error('GLB: chunk overruns the file');
    if (type === CH_JSON && json === null) {
      try { json = JSON.parse(new TextDecoder().decode(u8.subarray(start, start + len))); } catch (e) { throw new Error('GLB: JSON chunk is not valid JSON'); }
    } else if (type === CH_BIN && bin === null) bin = u8.subarray(start, start + len);
    off = start + ((len + 3) & ~3);
  }
  if (!json) throw new Error('GLB: no JSON chunk');
  return { json, bin };
}

async function resolveBuffers(json, bin, baseUrl, fetchImpl) {
  const out = [];
  for (let i = 0; i < (json.buffers || []).length; i++) {
    const b = json.buffers[i];
    if (!b.uri) { if (!bin) throw new Error('glTF: buffer ' + i + ' has no data'); out.push(bin); continue; }
    if (b.uri.startsWith('data:')) { out.push(b64(b.uri.slice(b.uri.indexOf(',') + 1))); continue; }
    if (!fetchImpl) throw new Error('glTF: external buffer ' + b.uri + ' needs fetch (export as .glb instead)');
    const r = await fetchImpl(/^[a-z]+:|^\//i.test(b.uri) ? b.uri : (baseUrl || '') + b.uri);
    if (!r.ok) throw new Error('glTF: could not fetch buffer ' + b.uri);
    out.push(new Uint8Array(await r.arrayBuffer()));
  }
  return out;
}

function readAccessor(json, buffers, idx) {
  const a = (json.accessors || [])[idx];
  if (!a) throw new Error('glTF: missing accessor ' + idx);
  const ct = COMP[a.componentType], n = NCOMP[a.type];
  if (!ct || !n) throw new Error('glTF: bad accessor type');
  const [Arr, size] = ct, count = a.count | 0, out = new Float32Array(count * n);
  const finish = () => { if (a.sparse) applySparse(json, buffers, a, out, n); return { data: out, n, count, ints: a.componentType !== 5126 }; };
  if (a.bufferView == null) return finish();
  const bv = (json.bufferViews || [])[a.bufferView];
  if (!bv) throw new Error('glTF: missing bufferView');
  const buf = buffers[bv.buffer];
  if (!buf) throw new Error('glTF: missing buffer');
  const stride = bv.byteStride || size * n, base = (bv.byteOffset || 0) + (a.byteOffset || 0);
  if (base + (count ? (count - 1) * stride + size * n : 0) > buf.byteLength) throw new Error('glTF: accessor reads past the end of its buffer');
  const dv = new DataView(buf.buffer, buf.byteOffset, buf.byteLength);
  const get = Arr === Float32Array ? 'getFloat32' : Arr === Int8Array ? 'getInt8' : Arr === Uint8Array ? 'getUint8' : Arr === Int16Array ? 'getInt16' : Arr === Uint16Array ? 'getUint16' : 'getUint32';
  const norm = a.normalized, maxv = { 5120: 127, 5121: 255, 5122: 32767, 5123: 65535 }[a.componentType];
  for (let i = 0; i < count; i++) for (let j = 0; j < n; j++) {
    let v = dv[get](base + i * stride + j * size, true);
    if (norm && maxv) v = a.componentType === 5120 || a.componentType === 5122 ? Math.max(v / maxv, -1) : v / maxv;
    out[i * n + j] = v;
  }
  return finish();
}

// glTF sparse accessor: overwrite `sparse.count` elements of the dense array (zeros if the accessor has no bufferView).
function applySparse(json, buffers, a, out, n) {
  const sp = a.sparse, cnt = sp.count | 0;
  const rd = (view, off, Arr, k) => {
    const bv = (json.bufferViews || [])[view.bufferView], buf = bv && buffers[bv.buffer];
    if (!buf) throw new Error('glTF: sparse accessor has a missing bufferView');
    const size = Arr.BYTES_PER_ELEMENT, base = (bv.byteOffset || 0) + (view.byteOffset || 0);
    if (base + cnt * k * size > buf.byteLength) throw new Error('glTF: sparse accessor reads past the end of its buffer');
    const dv = new DataView(buf.buffer, buf.byteOffset, buf.byteLength), fn = 'get' + Arr.name.replace('Array', '');
    const o = new Array(cnt * k);
    for (let i = 0; i < cnt * k; i++) o[i] = dv[fn](base + i * size, true);
    return o;
  };
  const IA = { 5121: Uint8Array, 5123: Uint16Array, 5125: Uint32Array }[sp.indices && sp.indices.componentType];
  const VA = (COMP[a.componentType] || [])[0];
  if (!IA || !VA) throw new Error('glTF: bad sparse accessor');
  const idx = rd(sp.indices, 0, IA, 1), val = rd(sp.values, 0, VA, n);
  const maxv = { 5120: 127, 5121: 255, 5122: 32767, 5123: 65535 }[a.componentType];
  for (let i = 0; i < cnt; i++) {
    if (!(idx[i] >= 0 && idx[i] < (a.count | 0))) throw new Error('glTF: sparse index out of range');
    for (let j = 0; j < n; j++) { let v = val[i * n + j]; if (a.normalized && maxv) v = Math.max(v / maxv, -1); out[idx[i] * n + j] = v; }
  }
}

/**
 * Parses glTF/GLB bytes (ArrayBuffer/Uint8Array) or a .gltf JSON object into a THREE.Group in FILE coordinates
 * (no orientation, scale or recentring). Returns {scene, warnings}. Throws a clean Error on malformed data.
 */
export async function parseGLB(THREE, data, { baseUrl = '', fetchImpl = null } = {}) {
  const warnings = [];
  let json, bin = null;
  if (data && typeof data === 'object' && !ArrayBuffer.isView(data) && !(data instanceof ArrayBuffer)) json = data;
  else {
    const u8 = toU8(data);
    if (u8 && u8.byteLength >= 4 && new DataView(u8.buffer, u8.byteOffset, 4).getUint32(0, true) === GLB_MAGIC) ({ json, bin } = readGLBContainer(u8));
    else { try { json = JSON.parse(new TextDecoder().decode(u8)); } catch (e) { throw new Error('not a glTF/GLB file'); } }
  }
  if (!json.asset || !/^2/.test(String(json.asset.version))) throw new Error('glTF: asset.version 2.x required');
  const buffers = await resolveBuffers(json, bin, baseUrl, fetchImpl);
  if ((json.images || []).length || (json.textures || []).length) warnings.push('textures are ignored (colours only)');
  if ((json.animations || []).length) warnings.push('animations are ignored');
  const SUPPORTED_EXT = ['KHR_materials_emissive_strength', 'KHR_materials_unlit', 'KHR_texture_transform', 'KHR_mesh_quantization'];
  const ext = [...(json.extensionsRequired || [])], used = json.extensionsUsed || [];
  if (ext.some(e => /draco|meshopt/i.test(e)) || used.some(e => /draco|meshopt/i.test(e)) || (json.bufferViews || []).some(b => b.extensions && b.extensions.EXT_meshopt_compression)
      || (json.meshes || []).some(me => (me.primitives || []).some(pr => pr.extensions && pr.extensions.KHR_draco_mesh_compression)))
    throw new Error('glTF: draco/meshopt compression is not supported; re-export without compression');
  const badExt = ext.filter(e => !SUPPORTED_EXT.includes(e));
  if (badExt.length) throw new Error('glTF: required extension(s) not supported: ' + badExt.join(', '));

  const mats = (json.materials || []).map(m => {
    const mx = m.extensions || {}, sg = mx.KHR_materials_pbrSpecularGlossiness;
    const p = m.pbrMetallicRoughness || (sg ? { baseColorFactor: sg.diffuseFactor, metallicFactor: 0, roughnessFactor: 1 - (sg.glossinessFactor ?? 1) } : {});
    const c = p.baseColorFactor || [1, 1, 1, 1], es = (mx.KHR_materials_emissive_strength || {}).emissiveStrength ?? 1;
    const e = (m.emissiveFactor || [0, 0, 0]).map(v => v * es);
    if (sg) warnings.push('material ' + (m.name || '') + ': pbrSpecularGlossiness approximated as diffuse colour');
    for (const k of Object.keys(mx)) if (/transmission|volume|clearcoat|sheen|iridescence/.test(k)) warnings.push('material ' + (m.name || '') + ': ' + k + ' ignored');
    const mat = new THREE.MeshStandardMaterial({
      color: new THREE.Color(c[0], c[1], c[2]), metalness: p.metallicFactor ?? 1, roughness: p.roughnessFactor ?? 1,
      emissive: new THREE.Color(e[0], e[1], e[2]), side: m.doubleSided ? THREE.DoubleSide : THREE.FrontSide,
    });
    if (m.alphaMode === 'BLEND' || (c[3] ?? 1) < 1) { mat.transparent = true; mat.opacity = c[3] ?? 1; }
    mat.name = m.name || '';
    return mat;
  });
  const defMat = new THREE.MeshStandardMaterial({ color: 0xaaaaaa, roughness: 0.6, metalness: 0.3 });

  const meshes = (json.meshes || []).map(me => (me.primitives || []).map(pr => {
    if ((pr.mode ?? 4) !== 4) { warnings.push('non-triangle primitive skipped'); return null; }
    const at = pr.attributes || {};
    if (at.POSITION == null) throw new Error('glTF: primitive without POSITION');
    const P = readAccessor(json, buffers, at.POSITION);
    if (P.n !== 3) throw new Error('glTF: POSITION must be VEC3');
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(P.data, 3));
    if (at.NORMAL != null) { const N = readAccessor(json, buffers, at.NORMAL); if (N.count === P.count) g.setAttribute('normal', new THREE.BufferAttribute(N.data, 3)); }
    if (pr.indices != null) {
      const I = readAccessor(json, buffers, pr.indices);
      const ix = new Uint32Array(I.count);
      for (let i = 0; i < I.count; i++) {
        const v = I.data[i];
        if (!(v >= 0 && v < P.count)) throw new Error('glTF: index out of range');
        ix[i] = v;
      }
      if (ix.length % 3) throw new Error('glTF: index count is not a multiple of 3');
      g.setIndex(new THREE.BufferAttribute(ix, 1));
    } else if (P.count % 3) throw new Error('glTF: vertex count is not a multiple of 3');
    if (!g.attributes.normal) g.computeVertexNormals();
    return { geo: g, mat: pr.material != null ? mats[pr.material] || defMat : defMat };
  }).filter(Boolean));

  const nodes = json.nodes || [], objs = new Array(nodes.length);
  const build = (i, depth) => {
    if (depth > 256) throw new Error('glTF: node hierarchy too deep');
    if (objs[i]) return objs[i];
    const n = nodes[i];
    if (!n) throw new Error('glTF: missing node ' + i);
    const o = new THREE.Group(); o.name = n.name || ('node' + i);
    if (n.matrix) {
      if (n.matrix.length !== 16) throw new Error('glTF: bad node matrix');
      const m = new THREE.Matrix4().fromArray(n.matrix); m.decompose(o.position, o.quaternion, o.scale);
    } else {
      if (n.translation) o.position.fromArray(n.translation);
      if (n.rotation) o.quaternion.fromArray(n.rotation).normalize();
      if (n.scale) o.scale.fromArray(n.scale);
    }
    if (n.mesh != null) {
      const prims = meshes[n.mesh];
      if (!prims) throw new Error('glTF: missing mesh ' + n.mesh);
      prims.forEach((p, k) => {
        const m = new THREE.Mesh(p.geo, p.mat); m.name = o.name + '#' + k; m.castShadow = true; m.receiveShadow = true; o.add(m);
      });
    }
    objs[i] = o;
    for (const c of n.children || []) o.add(build(c, depth + 1));
    return o;
  };
  const scene = new THREE.Group(); scene.name = 'cad';
  const sc = (json.scenes || [])[json.scene ?? 0];
  const roots = sc && sc.nodes ? sc.nodes : (() => {                       // no scene: roots are nodes nobody lists as a child
    const kids = new Set(); nodes.forEach(n => (n.children || []).forEach(c => kids.add(c)));
    return nodes.map((_, i) => i).filter(i => !kids.has(i));
  })();
  for (const r of roots) scene.add(build(r, 0));
  if (!scene.children.length) throw new Error('glTF: scene is empty');
  return { scene, warnings };
}

const AXES = { '+X': [1, 0, 0], '-X': [-1, 0, 0], '+Y': [0, 1, 0], '-Y': [0, -1, 0], '+Z': [0, 0, 1], '-Z': [0, 0, -1] };
const UNIT_TO_IN = { m: 1 / 0.0254, cm: 1 / 2.54, mm: 1 / 25.4, in: 1 };

/** Matrix (file coords -> robot coords) from {upAxis:'Y'|'Z'|'+Y'.., forwardAxis:'+X'..}. Throws if they are parallel. */
export function orientationMatrix(THREE, orientation = {}) {
  const up = AXES[/^[+-]/.test(orientation.upAxis || '') ? orientation.upAxis.toUpperCase() : '+' + String(orientation.upAxis || 'Y').toUpperCase()];
  const fw = AXES[String(orientation.forwardAxis || '+X').toUpperCase().replace(/^([XYZ])$/, '+$1')];
  if (!up || !fw) throw new Error('bad orientation: ' + JSON.stringify(orientation));
  const u = new THREE.Vector3(...up), f = new THREE.Vector3(...fw);
  if (Math.abs(u.dot(f)) > 0.5) throw new Error('orientation: forwardAxis must be perpendicular to upAxis');
  const l = new THREE.Vector3().crossVectors(u, f);                          // left = up x forward
  const m = new THREE.Matrix4().set(f.x, f.y, f.z, 0, l.x, l.y, l.z, 0, u.x, u.y, u.z, 0, 0, 0, 0, 1);
  if (orientation.yawDeg) m.premultiply(new THREE.Matrix4().makeRotationZ(orientation.yawDeg * Math.PI / 180));
  return m;
}

function footprintTarget(spec) {
  const fp = spec && spec.chassis && spec.chassis.footprint;
  if (!fp || !fp.length) return 18;
  let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
  for (const p of fp) { x0 = Math.min(x0, p[0]); x1 = Math.max(x1, p[0]); y0 = Math.min(y0, p[1]); y1 = Math.max(y1, p[1]); }
  return Math.max(x1 - x0, y1 - y0);
}

const DEFAULT_NAMES = {
  turret: ['turret'],
  hood: ['hood'],
  flywheel: ['flywheel', 'fly wheel', 'shooter wheel'],
};
const OPTIONAL_NAMES = { intake: ['intake'], flower: ['flower'] };   // found if present, never reported as missing
const isOptional = k => k in OPTIONAL_NAMES;

/**
 * Finds the nodes of the CAD that look like turret / hood / flywheel by case-insensitive name patterns.
 * `names` = { turret: ['turret', /yaw/i], hood: [...], flywheel: [...] } (strings are substrings). Returns
 * { turret, hood, flywheel, found: {key: nodeName}, missing: [keys] } with Object3D|null values. The topmost match
 * wins (a "Turret Assembly" node beats "Turret Plate" inside it); earlier patterns beat later ones.
 */
export function findMovingParts(group, names = {}) {
  const pats = {};
  for (const k of [...Object.keys(DEFAULT_NAMES), ...Object.keys(OPTIONAL_NAMES)]) pats[k] = (names[k] || DEFAULT_NAMES[k] || OPTIONAL_NAMES[k]).map(p => (p instanceof RegExp ? p : String(p).toLowerCase()));
  const test = (p, nm) => (p instanceof RegExp ? p.test(nm) : nm.toLowerCase().includes(p));
  const res = { turret: null, hood: null, flywheel: null, intake: null, flower: null, found: {}, missing: [] };
  for (const k of Object.keys(pats)) {
    for (const p of pats[k]) {
      let best = null, bestDepth = Infinity;
      const walk = (o, d) => {
        if (o.name && test(p, o.name) && !o.isMesh && d < bestDepth) { best = o; bestDepth = d; }
        else if (o.name && test(p, o.name) && o.isMesh && !best) { best = o.parent && o.parent.name && test(p, o.parent.name) ? o.parent : o; bestDepth = d; }
        for (const c of o.children) walk(c, d + 1);
      };
      walk(group, 0);
      if (best) { res[k] = best; res.found[k] = best.name; break; }
    }
    if (!res[k] && !isOptional(k)) res.missing.push(k);
  }
  return res;
}

/**
 * Turns parsed CAD (file coords) into a robot-frame group: orient, scale to inches, centre, floor at z=0.
 * Returns {group, scale, bounds, warnings, units, parts}. Pure (no fetch) so it is testable.
 */
export function placeCAD(THREE, scene, spec, opts = {}, warnings = []) {
  const outer = new THREE.Group(); outer.name = 'robotGLB';
  const offsetG = new THREE.Group(), scaleG = new THREE.Group(), orientG = new THREE.Group();
  orientG.matrixAutoUpdate = false; orientG.matrix.copy(orientationMatrix(THREE, opts.orientation));
  orientG.add(scene); scaleG.add(orientG); offsetG.add(scaleG); outer.add(offsetG);
  outer.updateMatrixWorld(true);
  // fit to the chassis: ignore intake/flower subtrees (they stick out of the frame) unless that would leave nothing
  const fitBox = () => {
    const skip = [OPTIONAL_NAMES.intake[0], OPTIONAL_NAMES.flower[0]], b = new THREE.Box3(), tmp = new THREE.Box3();
    (function walk(o) {
      if (o.name && !o.isMesh && skip.some(k => o.name.toLowerCase().includes(k))) return;
      if (o.isMesh && o.geometry) { tmp.setFromObject(o); if (!tmp.isEmpty()) b.union(tmp); }
      o.children.forEach(walk);
    })(outer);
    return b.isEmpty() ? new THREE.Box3().setFromObject(outer) : b;
  };
  const box = fitBox();
  if (box.isEmpty()) throw new Error('CAD has no geometry');
  const size = box.getSize(new THREE.Vector3());
  const rawW = Math.max(size.x, size.y), target = footprintTarget(spec);

  let unit = opts.units || null, scale;
  if (opts.scale != null) {
    if (!(Number.isFinite(opts.scale) && opts.scale > 0)) throw new Error('opts.scale must be a positive number');
    scale = opts.scale; unit = unit || 'custom';
  }
  else if (unit && UNIT_TO_IN[unit]) scale = UNIT_TO_IN[unit];
  else {
    if (unit) warnings.push(`unknown units '${unit}', auto-detecting`);
    let bestK = null, bestErr = Infinity;
    for (const [k, s] of Object.entries(UNIT_TO_IN)) { const err = Math.abs(Math.log(rawW * s / target)); if (err < bestErr) { bestErr = err; bestK = k; } }
    unit = bestK; scale = UNIT_TO_IN[bestK];
    const ratio = rawW * scale / target;
    if (ratio < 0.6 || ratio > 1.4) warnings.push(`CAD width ${(rawW * scale).toFixed(1)} in (guessed units: ${unit}) is not within 40% of the ${target.toFixed(1)} in footprint; pass opts.units ('m'|'mm'|'in') or opts.scale to confirm`);
  }
  scaleG.scale.setScalar(scale);
  outer.updateMatrixWorld(true);
  const b2 = fitBox();
  const cx = (b2.min.x + b2.max.x) / 2, cy = (b2.min.y + b2.max.y) / 2;
  offsetG.position.set(-cx - (opts.offset ? opts.offset[0] : 0), -cy - (opts.offset ? opts.offset[1] : 0), -b2.min.z);
  outer.updateMatrixWorld(true);
  const b3 = new THREE.Box3().setFromObject(outer), sz = b3.getSize(new THREE.Vector3());
  outer.traverse(o => { if (o.isMesh) { o.castShadow = true; o.receiveShadow = true; } });
  if (sz.x > 18.5 || sz.y > 18.5 || sz.z > 18.5) warnings.push(`CAD is ${sz.x.toFixed(1)} x ${sz.y.toFixed(1)} x ${sz.z.toFixed(1)} in: over the FTC 18 in limit (check units / orientation)`);
  return {
    group: outer, scale, units: unit, warnings,
    bounds: { min: b3.min.toArray(), max: b3.max.toArray(), size: sz.toArray() },
    parts: findMovingParts(outer, opts.names),
  };
}

/**
 * Loads the CAD. `urlOrData` is a URL (fetched) or an ArrayBuffer/Uint8Array. Returns
 * {group, scale, bounds, warnings, units, parts} or null (missing file, bad file): the reason is in
 * loadRobotGLB.lastReason, so the host can fall back to the procedural robot. Never throws.
 * opts: { orientation:{upAxis:'Y'|'Z', forwardAxis:'+X'}, units:'m'|'cm'|'mm'|'in', scale, names, offset:[x,y], fetchImpl }
 */
export async function loadRobotGLB(THREE, urlOrData, spec, opts = {}) {
  const fail = why => { loadRobotGLB.lastReason = why; return null; };
  loadRobotGLB.lastReason = '';
  try {
    const f = opts.fetchImpl || (typeof fetch === 'function' ? fetch : null);
    let data = urlOrData, base = '';
    if (typeof urlOrData === 'string') {
      if (!f) return fail('fetch is not available');
      let r;
      try { r = await f(urlOrData); } catch (e) { return fail('could not fetch ' + urlOrData + ': ' + (e && e.message || e)); }
      if (!r.ok) return fail(`${urlOrData}: HTTP ${r.status} (no CAD file, using the procedural robot)`);
      data = await r.arrayBuffer(); base = urlOrData.slice(0, urlOrData.lastIndexOf('/') + 1);
    }
    if (!data) return fail('no data');
    const { scene, warnings } = await parseGLB(THREE, data, { baseUrl: base, fetchImpl: f });
    return placeCAD(THREE, scene, spec, opts, warnings);
  } catch (e) {
    return fail(String(e && e.message || e));
  }
}
loadRobotGLB.lastReason = '';

/** Same as loadRobotGLB but returns {model, reason} so the reason travels with the result. */
export async function tryLoadRobotGLB(THREE, urlOrData, spec, opts) {
  const model = await loadRobotGLB(THREE, urlOrData, spec, opts);
  return { model, reason: model ? '' : loadRobotGLB.lastReason };
}

/**
 * Drives the CAD's moving nodes from the same state the procedural robot uses. `parts` = model.parts.
 * Turret: yaw about the node's local z. Hood: pitch about local y (negative = up, like robotmodel.js). Flywheel: spin about local y.
 * Each node's rest rotation is captured on first call, so the CAD's authored pose is the 0 deg pose. Node origins must sit on the axes
 * (docs/CAD_IMPORT.md). Missing parts are skipped. Returns the number of nodes driven.
 */
export function animateCADParts(THREE, parts, { turretDeg = 0, hoodDeg = 0, hoodZeroDeg = 0, flywheelRad = 0 } = {}) {
  if (!parts) return 0;
  let n = 0;
  const drive = (o, axis, ang) => {
    if (!o) return;
    if (!o.userData._rest) o.userData._rest = o.quaternion.clone();
    const q = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(axis === 'z' ? 0 : axis === 'x' ? 1 : 0, axis === 'y' ? 1 : 0, axis === 'z' ? 1 : 0), ang);
    o.quaternion.copy(o.userData._rest).multiply(q); n++;
  };
  const rad = d => d * Math.PI / 180;
  drive(parts.turret, 'z', rad(turretDeg));
  drive(parts.hood, 'y', -rad(hoodDeg - hoodZeroDeg));
  drive(parts.flywheel, 'y', flywheelRad);
  return n;
}
