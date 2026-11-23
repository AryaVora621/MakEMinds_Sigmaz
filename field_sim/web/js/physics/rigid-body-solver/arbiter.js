// arbiter.js - contact cache keyed by body pair, with feature-id warm starting.
// Sources: Catto 2005 "Fast and Simple Physics using Sequential Impulses" / "Contact caching" (Sec 8.1-8.2,
// https://box2d.org/files/ErinCatto_ContactCaching_GDC2005.pdf); box2d-lite Arbiter::Update
// (https://github.com/erincatto/box2d-lite/blob/master/src/Arbiter.cpp); material mixing per Randy Gaul ImpulseEngine
// Manifold::Initialize (friction = sqrt(fA*fB), restitution = min(eA,eB)).
// Units: SI (m, kg, s). No DOM.

export function pairKey(idA, idB) {
  return idA < idB ? `${idA}:${idB}` : `${idB}:${idA}`;
}

export function mixFriction(a, b) { return Math.sqrt(a * b); }
export function mixRestitution(a, b) { return Math.min(a, b); }

export function createArbiter(bodyA, bodyB) {
  if (!(bodyA.id < bodyB.id)) throw new Error('arbiter requires bodyA.id < bodyB.id');
  return {
    key: pairKey(bodyA.id, bodyB.id),
    bodyA, bodyB,
    friction: mixFriction(bodyA.friction, bodyB.friction),
    restitution: mixRestitution(bodyA.restitution, bodyB.restitution),
    normal: { x: 0, y: 0 },
    pointCount: 0,
    points: [],
    soft: { biasRate: 0, massScale: 0, impulseScale: 0 },
  };
}

function makePoint(mp) {
  return {
    id: mp.id,
    point: { x: mp.point.x, y: mp.point.y },
    separation: mp.separation,
    normalImpulse: 0, tangentImpulse: 0, maxNormalImpulse: 0,
    rA: { x: 0, y: 0 }, rB: { x: 0, y: 0 },
    normalMass: 0, tangentMass: 0, relativeVelocity: 0,
    localA: { x: 0, y: 0 }, localB: { x: 0, y: 0 }, localNormal: { x: 0, y: 0 },
    baseSeparation: mp.separation,
  };
}

// Replace the point list from a fresh manifold, carrying accumulated impulses across matching feature ids.
// params.solver.warmStarting / warmStartFactor decide whether (and how much) is carried.
export function updateArbiter(arb, manifold, params) {
  const ws = params.solver.warmStarting;
  const k = params.solver.warmStartFactor;
  const old = arb.points;
  const pts = [];
  for (let i = 0; i < manifold.pointCount; i++) {
    const np = makePoint(manifold.points[i]);
    if (ws) {
      for (let j = 0; j < old.length; j++) {
        if (old[j].id === np.id) {
          np.normalImpulse = old[j].normalImpulse * k;
          np.tangentImpulse = old[j].tangentImpulse * k;
          break;
        }
      }
    }
    pts.push(np);
  }
  arb.points = pts;
  arb.pointCount = pts.length;
  arb.normal = { x: manifold.normal.x, y: manifold.normal.y };
  arb.friction = mixFriction(arb.bodyA.friction, arb.bodyB.friction);
  arb.restitution = mixRestitution(arb.bodyA.restitution, arb.bodyB.restitution);
}
