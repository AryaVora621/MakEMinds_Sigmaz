import test from 'node:test';
import assert from 'node:assert/strict';
import { createPathEditor, THEME } from '../web/js/pathui.js';
import * as P from '../web/js/pathing.js';

const mk = () => createPathEditor({ mx: x => x * 5, my: y => y * 5, getK: () => 5, fetchImpl: async () => { throw new Error('offline'); }, debounceMs: 0 });

// Recording canvas stub: every method is a no-op that logs its name; measureText gives a width.
function stubCtx() {
  const log = [], store = {};
  const ctx = new Proxy(store, {
    get(t, k) {
      if (k === 'measureText') return s => ({ width: String(s).length * 6 });
      if (!(k in t)) t[k] = (...a) => { log.push(k); return undefined; };
      return t[k];
    },
    set(t, k, v) { t[k] = v; return true; },
  });
  return { ctx, log };
}

function routeWithTargets(ed) {
  const r = ed.route;
  r.start = { x: 20, y: 20, h: 0 };
  P.addLine(r, 100, 20);
  r.segments[0].heading = { mode: 'linear', start: 0, end: Math.PI / 2, reversed: false };
  P.addRotation(r, 0, 0.5, Math.PI / 4);
  P.addZone(r, 0, 0.25, 0.75, 20, 45);
  P.addEvent(r, 0, 0.6, 'shoot');
  return r;
}

test('draw renders rotation targets, zone stretches and labelled event markers without throwing', () => {
  const ed = mk(); ed.active = true; ed.sel = 0;
  routeWithTargets(ed);
  const { ctx, log } = stubCtx();
  ed.draw(ctx);
  const texts = log.filter(n => n === 'fillText').length;
  assert.ok(texts >= 3, 'tags are drawn for the rotation target, the zone and the event');
  assert.ok(log.includes('arc'), 'rotation target dot / anchors drawn');
  assert.ok(log.filter(n => n === 'stroke').length > 5, 'strokes drawn');
});

test('draw without rotations or zones matches the old output path (no tag for the target or zone)', () => {
  const ed = mk(); ed.active = true; ed.sel = 0;
  const r = ed.route; r.start = { x: 20, y: 20, h: 0 }; P.addLine(r, 100, 20);
  const plain = stubCtx(); ed.draw(plain.ctx);
  const plainTags = plain.log.filter(n => n === 'fillText').length;
  routeWithTargets(ed);
  const full = stubCtx(); ed.draw(full.ctx);
  assert.ok(full.log.filter(n => n === 'fillText').length > plainTags, 'the targets add labels');
});

test('side panel has add/remove rows for rotation targets and constraint zones on the selected path', () => {
  const ed = mk(); ed.active = true; ed.sel = 0;
  routeWithTargets(ed);
  ed.open.add('rotations'); ed.open.add('zones');
  const html = ed.panelHtml();
  assert.match(html, /data-act="addrot"/);
  assert.match(html, /data-act="addzone"/);
  assert.match(html, /data-rotdel="0"/);
  assert.match(html, /data-zdel="0"/);
  assert.match(html, /t 0\.50, 45 deg/);
  assert.match(html, /t 0\.25 to 0\.75, 20 in\/s, 45 in\/s2/);
});

test('the add-zone and add-rotation defaults are t 0.25..0.75 and t 0.5 in the model helpers', () => {
  const r = P.newRoute({ start: { x: 10, y: 10, h: 0 } }); P.addLine(r, 110, 10);
  assert.equal(P.addRotation(r, 0, 0.5, P.rotationAt(r, 0, 0.5)), 0);
  assert.equal(P.addZone(r, 0, 0.25, 0.75, 30, 45), 0);
  assert.deepEqual([r.segments[0].zones[0].t0, r.segments[0].zones[0].t1], [0.25, 0.75]);
  assert.equal(r.segments[0].rotations[0].t, 0.5);
});

test('THEME exposes the rotation and zone colour tokens', () => {
  assert.equal(THEME.rot, '#616161');
  assert.equal(THEME.rotSel, '#ff9800');
  assert.equal(THEME.zoneSel, '#ff9800');
  assert.equal(THEME.zoneHot, '#7c4dff');
});
