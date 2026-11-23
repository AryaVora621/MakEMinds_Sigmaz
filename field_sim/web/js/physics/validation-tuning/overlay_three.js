// overlay_three.js - turns the renderer-agnostic geometry from overlay.js into one THREE.LineSegments.
// THREE is injected (never imported). Buffers are preallocated and doubled on overflow; setDrawRange limits
// drawing to the live segments. Never touches the sim.

const INITIAL_SEGMENTS = 4096; // segments, ASSUMED initial capacity (grows by doubling)

export function createThreeOverlay(THREE, parent, opts = {}) {
  const depthTest = opts.depthTest != null ? opts.depthTest : false;
  const renderOrder = opts.renderOrder != null ? opts.renderOrder : 999;
  let cap = INITIAL_SEGMENTS;
  let pos = new Float32Array(cap * 6), col = new Float32Array(cap * 6);
  const geometry = new THREE.BufferGeometry();
  let posAttr = new THREE.BufferAttribute(pos, 3);
  let colAttr = new THREE.BufferAttribute(col, 3);
  geometry.setAttribute('position', posAttr);
  geometry.setAttribute('color', colAttr);
  geometry.setDrawRange(0, 0);
  const material = new THREE.LineBasicMaterial({ vertexColors: true, depthTest, transparent: true });
  const object3d = new THREE.LineSegments(geometry, material);
  object3d.renderOrder = renderOrder;
  object3d.frustumCulled = false;
  object3d.visible = false;
  if (parent && parent.add) parent.add(object3d);
  let visible = false;

  function grow(need) {
    while (cap < need) cap *= 2;
    pos = new Float32Array(cap * 6); col = new Float32Array(cap * 6);
    posAttr = new THREE.BufferAttribute(pos, 3); colAttr = new THREE.BufferAttribute(col, 3);
    geometry.setAttribute('position', posAttr);
    geometry.setAttribute('color', colAttr);
  }

  return {
    update(geom) {
      const n = geom.segCount | 0;
      if (n > cap) grow(n);
      pos.set(geom.positions.subarray(0, n * 6));
      col.set(geom.colors.subarray(0, n * 6));
      posAttr.needsUpdate = true; colAttr.needsUpdate = true;
      geometry.setDrawRange(0, n * 2);
    },
    setVisible(v) { visible = !!v; object3d.visible = visible; },
    get visible() { return visible; },
    object3d,
    dispose() {
      if (parent && parent.remove) parent.remove(object3d);
      if (geometry.dispose) geometry.dispose();
      if (material.dispose) material.dispose();
    },
  };
}
