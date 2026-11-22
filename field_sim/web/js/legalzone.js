// Configurable launch-legality zone. ALL presets are ASSUMPTIONS: the game rules
// define no official launch zone (see TeamCode FieldConstants.java ~line 100).
// Pure ES module, no DOM. Units inches, Pedro frame (x right, y far, field 0..144).

export const FIELD = 144;
export const MID = 72;
// Hive pivots from web/data/scene.json groups.hive_red / hive_blue.
export const HIVES = { red: { x: 59.25, y: 72 }, blue: { x: 84.75, y: 72 } };
// Assumed hive frame footprint (opening is ~20.04 wide in x; depth spans y ~50..94).
export const HIVE_HALF_W = 10.02;
export const HIVE_HALF_D = 22;

const ID_LIST = ['open', 'own_half', 'audience_side', 'far_side', 'near_hive_ring', 'outside_hive_footprint', 'custom'];
const ASSUME = ' ASSUMPTION: not from the game rules (no official launch zone exists).';

const PRESETS = [
  { id: 'open', label: 'Open field', description: 'Anywhere inside the field (0..144).' + ASSUME },
  { id: 'own_half', label: 'Own half', description: 'Robot alliance half, split at x = 72 (midline between the hive axes at x 59.25 and 84.75; red x < 72, blue x > 72).' + ASSUME },
  { id: 'audience_side', label: 'Audience side', description: 'Audience half of the field, y < 72.' + ASSUME },
  { id: 'far_side', label: 'Far side', description: 'Far half of the field, y > 72.' + ASSUME },
  { id: 'near_hive_ring', label: 'Ring around own hive', description: 'Annulus around the alliance hive pivot, minR..maxR (default 30..90 in).' + ASSUME },
  { id: 'outside_hive_footprint', label: 'Outside hive footprint', description: 'Anywhere except the hive frame footprint rectangle (both hives).' + ASSUME },
  { id: 'custom', label: 'Custom polygons', description: 'User-defined polygons; legal if inside any.' + ASSUME },
].map((p) => ({ ...p, assumption: true }));

const EPS = 1e-9;

function onSegment(x, y, a, b) {
  const cross = (b[0] - a[0]) * (y - a[1]) - (b[1] - a[1]) * (x - a[0]);
  const scale = Math.max(1, Math.abs(b[0] - a[0]) + Math.abs(b[1] - a[1]));
  if (Math.abs(cross) > 1e-9 * scale) return false;
  return x >= Math.min(a[0], b[0]) - EPS && x <= Math.max(a[0], b[0]) + EPS &&
         y >= Math.min(a[1], b[1]) - EPS && y <= Math.max(a[1], b[1]) + EPS;
}

/** Ray-cast point in polygon; boundary points count as inside. */
export function pointInPolygon(x, y, poly) {
  if (!Array.isArray(poly) || poly.length < 3) return false;
  let inside = false;
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
    const a = poly[j], b = poly[i];
    if (onSegment(x, y, a, b)) return true;
    if ((b[1] > y) !== (a[1] > y) && x < ((a[0] - b[0]) * (y - b[1])) / (a[1] - b[1]) + b[0]) inside = !inside;
  }
  return inside;
}

/** Distance from (x,y) to the alliance hive pivot. hiveState optional {x,y} override. */
export function distanceToHive(x, y, alliance, hiveState) {
  const h = hiveState || HIVES[alliance === 'blue' ? 'blue' : 'red'];
  return Math.hypot(x - h.x, y - h.y);
}

const rect = (x0, y0, x1, y1) => [[x0, y0], [x1, y0], [x1, y1], [x0, y1]];
const circle = (cx, cy, r, n = 48) =>
  Array.from({ length: n }, (_, k) => [cx + r * Math.cos((2 * Math.PI * k) / n), cy + r * Math.sin((2 * Math.PI * k) / n)]);
const isPt = (p) => Array.isArray(p) && p.length === 2 && p.every(Number.isFinite);
const hiveRect = (a) => { const h = HIVES[a]; return rect(h.x - HIVE_HALF_W, h.y - HIVE_HALF_D, h.x + HIVE_HALF_W, h.y + HIVE_HALF_D); };
const norm = (a) => (a === 'blue' ? 'blue' : 'red');

export class LegalZone {
  constructor({ preset = 'open', polygons = [], alliance = null, enabled = true, minR = 30, maxR = 90 } = {}) {
    if (!ID_LIST.includes(preset)) throw new Error(`LegalZone: unknown preset '${preset}'`);
    if (alliance !== null && alliance !== 'red' && alliance !== 'blue') throw new Error('LegalZone: alliance must be red, blue or null');
    if (!Array.isArray(polygons)) throw new Error('LegalZone: polygons must be an array');
    for (const poly of polygons) {
      if (!Array.isArray(poly) || poly.length < 3 || !poly.every(isPt)) throw new Error('LegalZone: each polygon needs >=3 finite [x,y] points');
    }
    if (!Number.isFinite(minR) || !Number.isFinite(maxR) || minR < 0 || maxR <= minR) throw new Error('LegalZone: need 0 <= minR < maxR');
    if (preset === 'custom' && polygons.length === 0) throw new Error('LegalZone: custom preset needs at least one polygon');
    this.preset = preset;
    this.polygons = polygons.map((p) => p.map((q) => [q[0], q[1]]));
    this.alliance = alliance; // default alliance used when contains() gets none
    this.enabled = !!enabled;
    this.minR = minR;
    this.maxR = maxR;
  }

  static presets() { return PRESETS.map((p) => ({ ...p })); }

  /** True if the robot centre may shoot from (x,y). Always true when disabled. */
  contains(x, y, alliance) {
    if (!this.enabled) return true;
    if (!Number.isFinite(x) || !Number.isFinite(y)) return false;
    const al = norm(alliance || this.alliance);
    const inField = x >= -EPS && x <= FIELD + EPS && y >= -EPS && y <= FIELD + EPS;
    switch (this.preset) {
      case 'open': return inField;
      case 'own_half': return inField && (al === 'red' ? x <= MID + EPS : x >= MID - EPS);
      case 'audience_side': return inField && y <= MID + EPS;
      case 'far_side': return inField && y >= MID - EPS;
      case 'near_hive_ring': {
        const d = distanceToHive(x, y, al);
        return inField && d >= this.minR - EPS && d <= this.maxR + EPS;
      }
      case 'outside_hive_footprint': {
        if (!inField) return false;
        // footprint boundary counts as outside the footprint (legal)
        for (const a of ['red', 'blue']) {
          const h = HIVES[a];
          if (Math.abs(x - h.x) < HIVE_HALF_W - EPS && Math.abs(y - h.y) < HIVE_HALF_D - EPS) return false;
        }
        return true;
      }
      case 'custom': return this.polygons.some((p) => pointInPolygon(x, y, p));
      default: return false;
    }
  }

  describe() {
    if (!this.enabled) return 'Launch zone check disabled: shots allowed anywhere [assumed]';
    const al = this.alliance ? ` (${this.alliance})` : '';
    switch (this.preset) {
      case 'open': return 'Legal: anywhere in the field [assumed, no official launch zone]';
      case 'own_half': return `Legal: own alliance half${al}, red x < 72 / blue x > 72 [assumed]`;
      case 'audience_side': return 'Legal: audience half (y < 72) [assumed]';
      case 'far_side': return 'Legal: far half (y > 72) [assumed]';
      case 'near_hive_ring': return `Legal: ${this.minR}-${this.maxR} in from own hive [assumed]`;
      case 'outside_hive_footprint': return 'Legal: anywhere outside the hive frame footprint [assumed]';
      default: return `Legal: inside ${this.polygons.length} custom polygon(s) [assumed]`;
    }
  }

  /**
   * Outlines for drawing. Returns an array; each item is either
   *   [[x,y],...]                              a plain polygon (legal region outline), or
   *   {outer:[[x,y],...], inner:[[x,y],...]}   region between outer and inner (inner is the
   *       excluded hole). near_hive_ring: two 48-gon circles. outside_hive_footprint: one
   *       item per hive, outer = field rectangle, inner = hive footprint rectangle.
   */
  polygonsFor(alliance) {
    const al = norm(alliance || this.alliance);
    const field = rect(0, 0, FIELD, FIELD);
    switch (this.preset) {
      case 'open': return [field];
      case 'own_half': return [al === 'red' ? rect(0, 0, MID, FIELD) : rect(MID, 0, FIELD, FIELD)];
      case 'audience_side': return [rect(0, 0, FIELD, MID)];
      case 'far_side': return [rect(0, MID, FIELD, FIELD)];
      case 'near_hive_ring': {
        const h = HIVES[al];
        return [{ outer: circle(h.x, h.y, this.maxR), inner: circle(h.x, h.y, this.minR) }];
      }
      case 'outside_hive_footprint': return [{ outer: field, inner: hiveRect('red') }, { outer: field, inner: hiveRect('blue') }];
      default: return this.polygons.map((p) => p.map((q) => [q[0], q[1]]));
    }
  }

  toJSON() {
    return { preset: this.preset, polygons: this.polygons.map((p) => p.map((q) => [q[0], q[1]])), alliance: this.alliance, enabled: this.enabled, minR: this.minR, maxR: this.maxR };
  }

  static fromJSON(obj) {
    if (obj === null || typeof obj !== 'object' || Array.isArray(obj)) throw new Error('LegalZone.fromJSON: object required');
    return new LegalZone({
      preset: obj.preset ?? 'open', polygons: obj.polygons ?? [], alliance: obj.alliance ?? null,
      enabled: obj.enabled ?? true, minR: obj.minR ?? 30, maxR: obj.maxR ?? 90,
    });
  }
}
