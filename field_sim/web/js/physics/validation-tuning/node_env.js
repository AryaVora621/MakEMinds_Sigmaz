// Node-only loader for the field environment (bounds + hive colliders). Mirrors tests_js/sim_physics.test.mjs lines 57-66.
// node:fs is imported dynamically inside the function, so merely importing this module in a browser is harmless.
// Units: PEDRO inches.

// SOURCED from tests_js/sim_physics.test.mjs (field bounds used by the existing sim tests).
export const FIELD_BOUNDS = Object.freeze({ minX: 1.3, minY: 1.3, maxX: 142.7, maxY: 142.7 });

/** Load {bounds, hives?} for BallWorld. hives[a] = {pivot, cadAngle, cells, mesh} for a in red/blue. */
export async function loadFieldEnvNode(opts = {}) {
  const withHives = opts.withHives !== false;
  const env = { bounds: { ...FIELD_BOUNDS } };
  if (!withHives) return env;
  const { readFileSync } = await import('node:fs');
  const { parseHiveCollider } = await import('../../hivemesh.js');
  const scene = JSON.parse(readFileSync(new URL('../../../data/scene.json', import.meta.url), 'utf8'));
  const bin = readFileSync(new URL('../../../data/hive_collider.bin', import.meta.url));
  const meshes = parseHiveCollider(bin.buffer.slice(bin.byteOffset, bin.byteOffset + bin.byteLength));
  const hives = {};
  for (const a of ['red', 'blue']) {
    const g = scene.groups['hive_' + a], hc = scene.constants.hives[a];
    hives[a] = { pivot: g.pivot, cadAngle: g.cad_pose_angle_deg, cells: hc.states[g.cad_pose_state].cells, mesh: meshes[a] };
  }
  env.hives = hives;
  return env;
}
