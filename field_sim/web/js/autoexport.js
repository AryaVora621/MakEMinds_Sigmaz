// autoexport.js - turns a planned auto (plain JSON, schema_version 1) into COMPILABLE Java for the real robot.
// Pure ES module, no imports. Input contract (defined here, accepted as plain JSON):
//   { name, alliance, start:{x,y,h(rad)},
//     paths:{ id:{ segments:[{type:'line'|'curve', points:[{x,y}...], heading:{mode:'constant'|'linear'|'tangent', start?, end?, reversed?}}],
//                  markers:[{t, command:node}] } },
//     root: node }
//   node = {id,type:'follow',path} | {type:'wait',seconds} | {type:'waitUntil',condition,timeout}
//        | {type:'command',name,args} | {type:'sequential'|'parallel'|'race'|'deadline',children:[...],deadline?:index}
// Pedro 2.1.0 calls used (verified against core-2.1.0-sources.jar): PathBuilder.addPath(Curve), setLinear/Constant/
// TangentHeadingInterpolation, setReversed (markers are polled via Follower.getChainIndex/getCurrentTValue; chain callbacks never fire in 2.1.0), BezierLine(Pose,Pose),
// BezierCurve(FuturePose...), Pose(x,y[,h]). Robot calls verified against TeamCode sources (Robot, AutoDrivebase,
// IntakeTransfer, ShooterBase, Gate, FlowerMech, BallTracker).

import { callbackMarkers } from './autos.js';

/** Copy of the auto whose paths also carry the path-planner event markers (segment callbacks) as auto markers. */
function withEventMarkers(auto) {
  if (!auto || !auto.paths || typeof auto.paths !== 'object') return auto;
  const out = { ...auto, paths: {} };
  for (const [id, p] of Object.entries(auto.paths)) out.paths[id] = p && Array.isArray(p.segments) ? { ...p, markers: [...(p.markers || []), ...callbackMarkers(p)] } : p;
  return out;
}

const KEYWORDS = new Set(['abstract', 'assert', 'boolean', 'break', 'byte', 'case', 'catch', 'char', 'class', 'const', 'continue', 'default', 'do', 'double', 'else', 'enum', 'extends', 'final', 'finally', 'float', 'for', 'goto', 'if', 'implements', 'import', 'instanceof', 'int', 'interface', 'long', 'native', 'new', 'package', 'private', 'protected', 'public', 'return', 'short', 'static', 'strictfp', 'super', 'switch', 'synchronized', 'this', 'throw', 'throws', 'transient', 'try', 'void', 'volatile', 'while', 'true', 'false', 'null', '_']);
// names that would clash with imports used by the generated file
const RESERVED_CLASS = new Set(['Robot', 'Pose', 'Follower', 'PathChain', 'BezierLine', 'BezierCurve', 'Alliance', 'Autonomous', 'LinearOpMode', 'ElapsedTime', 'String', 'Object', 'Math', 'System']);
export const KNOWN_COMMANDS = ['shoot', 'shootAll', 'intakeOn', 'intakeOff', 'aimOn', 'aimOff', 'autoAim', 'setRpm', 'flowerDeploy', 'flowerStow', 'gateOpen', 'gateClose', 'park'];
export const IVY_GRADLE_LINE = "implementation 'com.pedropathing.ivy:pedro:1.1.1'";

// ---------- formatting ----------
function num(v, digits = 2) {
  v = Number(v);
  if (!Number.isFinite(v)) return '0.0';
  const f = 10 ** digits;
  let s = (Math.round(v * f) / f).toFixed(digits).replace(/0+$/, '');
  if (s.endsWith('.')) s += '0';
  return s === '-0.0' ? '0.0' : s;
}
export function fmtAngleRad(rad) {
  let deg = Number(rad);
  deg = Number.isFinite(deg) ? deg * 180 / Math.PI : 0;
  const r5 = Math.round(deg / 5) * 5;
  deg = Math.abs(deg - r5) < 1e-6 ? r5 : Math.round(deg * 100) / 100;
  if (Object.is(deg, -0) || deg === 0) deg = 0;
  return `Math.toRadians(${deg})`;
}
export function sanitizeClassName(name, fallback = 'GeneratedAuto') {
  let id = String(name == null ? '' : name).replace(/[^A-Za-z0-9_]/g, '_').replace(/_+/g, '_');
  id = id.replace(/^_+|_+$/g, '');
  if (!id) id = fallback;
  if (!/^[A-Za-z]/.test(id)) id = 'Auto_' + id;
  if (KEYWORDS.has(id) || RESERVED_CLASS.has(id)) id += 'Auto';
  return id;
}
const ident = s => {
  let id = String(s == null ? '' : s).replace(/[^A-Za-z0-9_]/g, '_');
  if (!id) id = 'x';
  if (/^[0-9]/.test(id)) id = '_' + id;
  return id;
};
const cmt = s => String(s == null ? '' : s).replace(/[^\w .,:()+-]/g, '').replace(/\*\//g, '');
const jstr = s => cmt(s).replace(/"/g, '');
const isNum = v => typeof v === 'number' && Number.isFinite(v);
const secs = v => { const n = Number(v); return Number.isFinite(n) && n > 0 ? n : 0; };

// ---------- path generation ----------
export function segmentWeights(segs) {
  const w = segs.map(s => {
    let p = s.points || []; let L = 0;
    // a 'line' segment is emitted as BezierLine(first, last) (see buildPathCode), so only its endpoints count
    if (s.type === 'line' && p.length > 2) p = [p[0], p[p.length - 1]];
    for (let i = 1; i < p.length; i++) L += Math.hypot((p[i].x || 0) - (p[i - 1].x || 0), (p[i].y || 0) - (p[i - 1].y || 0));
    return L;
  });
  return w;
}
export function locateMarker(t, weights) {
  const total = weights.reduce((a, b) => a + b, 0);
  const n = weights.length;
  if (n === 0) return null;   // no segments: nothing to attach a marker to
  t = Math.min(1, Math.max(0, Number.isFinite(Number(t)) ? Number(t) : 0));
  if (n === 1 || total < 1e-9) {
    if (t >= 1) return { seg: n - 1, t: 1 };   // end of path = end of last segment (not the start of it)
    return { seg: Math.min(n - 1, Math.floor(t * n)), t: n === 1 ? t : (t * n) % 1 };
  }
  const g = t * total; let cum = 0;
  for (let i = 0; i < n; i++) {
    if (g <= cum + weights[i] + 1e-9 || i === n - 1) return { seg: i, t: weights[i] > 0 ? Math.min(1, Math.max(0, (g - cum) / weights[i])) : 0 };
    cum += weights[i];
  }
  return { seg: n - 1, t: 1 };
}
// statements for one instant command; also records stubs
function instantStatements(cmd, args, ctx) {
  args = args && typeof args === 'object' ? args : {};
  const name = String(cmd == null ? '' : cmd);
  switch (name) {
    case 'intakeOn': return ['robot.intake.forward();'];
    case 'intakeOff': return ['robot.intake.stop();'];
    case 'aimOn': return ['if (robot.shooter != null) robot.shooter.setSpinning(true);   // shooter aims itself every robot.update()'];
    case 'aimOff': return ['if (robot.shooter != null) robot.shooter.setSpinning(false);'];
    case 'flowerDeploy': return ['if (robot.flower != null) robot.flower.deploy();'];
    case 'flowerStow': return ['if (robot.flower != null) robot.flower.stow();'];
    case 'gateOpen': return ['if (robot.gate != null) robot.gate.open();'];
    case 'gateClose': return ['if (robot.gate != null) robot.gate.close();'];
    case 'park': ctx.stubs.set('park', 'private void cmdPark() {\n        // TODO: no park routine exists on the robot yet; implement it here (e.g. follow a park path).\n    }'); return ['cmdPark();'];
    case 'setRpm': {
      ctx.stubs.set('setRpm', 'private void cmdSetRpm(double rpm) {\n        // TODO: ShooterBase has no setRpm (RPM is solved from the shot); implement an override here.\n    }');
      return [`cmdSetRpm(${num(args.rpm, 1)});`];
    }
    default: {
      const raw = name.startsWith('custom:') ? name.slice(7) : name;
      const m = 'custom_' + ident(raw);
      ctx.stubs.set('custom:' + ident(raw), `private void ${m}() {\n        // TODO: implement custom command "${jstr(raw)}".\n    }`);
      return [`${m}();`];
    }
  }
}
const TIMED = new Set(['shoot', 'shootAll']);

// ---------- autoAim ----------
// The robot side is TeamCode/src/main/java/commands/autonomous/AutoAim.java (blocking static run()) and
// AutoAimLogic.java (the same loop as a per-tick state machine). Generated code uses AutoAimLogic per tick so it
// works inside sequential/parallel/race/deadline states; AutoAim.run() blocks and would stall sibling branches.
// Change the package or class names here only.
// TODO re-check against TeamCode AutoAimCommand once it lands (no AutoAimCommand class exists yet; AutoAimLogic/AutoAimTunables were verified present).
export const AUTOAIM_PACKAGE = 'commands.autonomous';
export const AUTOAIM_LOGIC_CLASS = 'AutoAimLogic';
export const AUTOAIM_TUNABLES_CLASS = 'RobotSettings.AutoAim';   // moved: all values live in robot/RobotSettings.java
const AUTOAIM_IMPORTS = [`${AUTOAIM_PACKAGE}.${AUTOAIM_LOGIC_CLASS}`];
// planner shape node.type==='autoAim' (+ node.shootAfter / node.seconds) is the same as
// {type:'command', name:'autoAim', args:{shootAfter, seconds}}
function normAutoAim(node) {
  if (!node || node.type !== 'autoAim') return node;
  return { ...node, type: 'command', name: 'autoAim', args: { shootAfter: node.shootAfter, seconds: node.seconds } };
}
function treeHasAutoAim(node) {
  if (!node || typeof node !== 'object') return false;
  if (node.type === 'autoAim' || (node.type === 'command' && node.name === 'autoAim')) return true;
  return Array.isArray(node.children) && node.children.some(treeHasAutoAim);
}

function buildPathCode(auto, builderExpr, withMarkers, ctx) {
  const fields = [], pathFields = [], body = [];
  const sp = auto.start || { x: 0, y: 0, h: 0 };
  fields.push(`private final Pose START_POSE = new Pose(${num(sp.x)}, ${num(sp.y)}, ${fmtAngleRad(sp.h || 0)});`);
  const used = new Set();
  const names = ctx.pathNames;
  for (const id of Object.keys(auto.paths || {}).sort()) {
    let f = 'path_' + ident(id), k = 2;
    while (used.has(f)) f = 'path_' + ident(id) + '_' + k++;
    used.add(f); names.set(id, f);
  }
  for (const id of Object.keys(auto.paths || {}).sort()) {
    const path = auto.paths[id], f = names.get(id), segs = path.segments || [];
    pathFields.push(f);
    const weights = segmentWeights(segs);
    const lines = [`        ${f} = ${builderExpr}`];
    segs.forEach((s, i) => {
      const pts = s.points || [];
      const pn = pts.map((p, j) => {
        const nm = `pose_${f.slice(5)}_${i}_${j}`;
        fields.push(`private final Pose ${nm} = new Pose(${num(p.x)}, ${num(p.y)});`);
        return nm;
      });
      let geom;
      if (pn.length < 2) geom = `new BezierLine(${pn[0] || 'START_POSE'}, ${pn[0] || 'START_POSE'})`;
      else if (pn.length === 2 || s.type === 'line') geom = `new BezierLine(${pn[0]}, ${pn[pn.length - 1]})`;
      else geom = `new BezierCurve(${pn.join(', ')})`;
      lines.push(`                .addPath(${geom})`);
      const h = s.heading || { mode: 'tangent' };
      if (h.mode === 'tangent') lines.push('                .setTangentHeadingInterpolation()');
      else if (h.mode === 'constant') lines.push(`                .setConstantHeadingInterpolation(${fmtAngleRad(h.start || 0)})`);
      else lines.push(`                .setLinearHeadingInterpolation(${fmtAngleRad(h.start || 0)}, ${fmtAngleRad(h.end == null ? (h.start || 0) : h.end)})`);
      if (h.reversed) lines.push('                .setReversed()');
    });
    // Pedro 2.1.0 chain callbacks never fire, so markers are NOT put in the builder (no addParametricCallback).
    // They are polled in the follow state (chain index + t): see emitStateMachine.
    if (withMarkers) {
      if (!ctx.pathMarkers) ctx.pathMarkers = new Map();
      const list = (path.markers || []).map(m => {
        const loc = locateMarker(m.t, weights);
        if (!loc) return null;
        return { seg: loc.seg, t: Number(num(loc.t, 3)), stmts: markerStatements(m.command, ctx) };
      }).filter(Boolean).sort((a, b) => a.seg - b.seg || a.t - b.t);
      ctx.pathMarkers.set(id, list);
    }
    lines.push('                .build();');
    body.push(`        // path "${cmt(id)}"\n` + lines.join('\n'));
  }
  return { fields, pathFields, body };
}
// autoAim as a path marker: non-blocking. startAutoAim(shootAfter) creates the AutoAimCommand and the generated
// loop steps it once per robot tick (ctx.usesAutoAim adds the `if (autoAimCmd != null) autoAimCmd.update();` line).
const AUTOAIM_MARKER_STUB = `private AutoAimCommand autoAimCmd;
    // autoAim marker: starts aiming without blocking the path. autoAimCmd.update() runs every loop tick.
    private void startAutoAim(boolean shootAfter) {
        if (robot.shooter != null) {
            robot.shooter.setSpinning(true);
        }
        autoAimCmd = new AutoAimCommand(shootAfter);
    }

    private final class AutoAimCommand {
        private final AutoAimLogic logic;
        private final boolean shootAfter;
        private boolean feeding, done;
        private double feedT0;

        AutoAimCommand(boolean shootAfter) {
            this.shootAfter = shootAfter;
            this.logic = new AutoAimLogic(RobotSettings.AutoAim.TIMEOUT_S, shootAfter, RobotSettings.AutoAim.TRANSFER_ON_START);
        }

        void update() {
            if (done) {
                return;
            }
            if (robot.shooter == null) {
                done = true;   // no shooter: nothing to aim (same as AutoAim.run)
                return;
            }
            if (!feeding) {
                AutoAimLogic.Output out = logic.update(clock.seconds(), true, robot.shooter.ready());
                if (out.transferOn) {
                    robot.intake.feed();
                } else {
                    robot.intake.stop();
                }
                if (!out.done) {
                    return;
                }
                if (!shootAfter) {
                    robot.intake.stop();
                    done = true;
                    return;
                }
                feeding = true;
                feedT0 = clock.seconds();
            }
            if (clock.seconds() - feedT0 >= RobotSettings.AutoAim.SHOOT_AFTER_MAX_S) {
                robot.intake.stop();
                done = true;
                return;
            }
            if (!robot.feedIfReady(true)) {
                robot.intake.stop();
            }
        }
    }`;
function markerStatements(node, ctx) {
  if (!node) return ['// empty marker'];
  node = normAutoAim(node);
  if (node.type === 'command' && node.name === 'autoAim') {
    const a = node.args && typeof node.args === 'object' ? node.args : {};
    ctx.usesAutoAim = true;
    ctx.stubs.set('autoAim', AUTOAIM_MARKER_STUB);
    return [`startAutoAim(${a.shootAfter === true ? 'true' : 'false'});`];
  }
  if (node.type === 'command') {
    if (TIMED.has(node.name)) {
      ctx.stubs.set('marker:' + node.name, null);
      return [`// TODO: marker command "${jstr(node.name)}" is timed; it cannot run inside a path callback. Use a parallel group instead.`];
    }
    return instantStatements(node.name, node.args, ctx);
  }
  if (node.type === 'sequential' || node.type === 'parallel') return (node.children || []).flatMap(c => markerStatements(c, ctx));
  ctx.stubs.set('marker:' + node.type, null);
  return [`// TODO: marker node type "${jstr(node.type)}" is not supported inside a callback.`];
}

// ---------- condition mapping ----------
function conditionExpr(cond, ctx) {
  let c = cond;
  if (c && typeof c === 'object') c = c.name || c.type || '';
  c = String(c == null ? '' : c).trim();
  const m = /^ballCount\s*(>=|>|<=|<|==)\s*(\d+)$/.exec(c);
  if (m) return `robot.tracker != null && robot.tracker.getCount() ${m[1]} ${m[2]}`;
  switch (c) {
    case 'flywheelReady': case 'shooterReady': case 'aimed': return 'robot.shooter != null && robot.shooter.ready()';
    case 'readyToFire': return 'robot.readyToFire()';
    case 'ballsFull': case 'full': case 'isFull': return 'robot.tracker != null && robot.tracker.isFull()';
    case 'gateOpen': return 'robot.gate != null && robot.gate.isOpen()';
    case 'gateAtTarget': return 'robot.gate != null && robot.gate.atTarget()';
    case 'flowerDeployed': return 'robot.flower != null && robot.flower.isDeployed()';
    case 'flowerIdle': return 'robot.flower != null && !robot.flower.isBusy()';
    default: {
      const m2 = 'condition_' + ident(c || 'unnamed');
      ctx.stubs.set('condition:' + ident(c || 'unnamed'), `private boolean ${m2}() {\n        // TODO: implement the wait condition "${jstr(c)}"; return true when it is satisfied.\n        return false;\n    }`);
      return `${m2}()`;
    }
  }
}

// ---------- state-machine walk ----------
function emitStateMachine(auto, ctx) {
  const decl = [], methods = [];
  let counter = 0;
  const kids = n => Array.isArray(n.children) ? n.children : [];
  function walk(node) {
    node = normAutoAim(node);
    const n = counter++; const p = `n${n}`;
    const T = node && node.type;
    const label = cmt(`${T}${node.id ? ' ' + node.id : ''}`);
    const out = { n };
    if (T === 'follow') {
      const f = ctx.pathNames.get(node.path) || ('path_' + ident(node.path));
      const marks = (ctx.pathMarkers && ctx.pathMarkers.get(node.path)) || [];
      decl.push(`private boolean ${p}_started, ${p}_done;`);
      if (marks.length) decl.push(`private boolean ${marks.map((_, k) => `${p}_m${k}`).join(', ')};`);
      const fire = (k, ind) => `${ind}${p}_m${k} = true;\n` + marks[k].stmts.map(st => ind + st).join('\n');
      const poll = marks.length ? '        int chainIdx = robot.auto.getFollower().getChainIndex();\n        double tNow = robot.auto.getFollower().getCurrentTValue();\n'
        + marks.map((m, k) => `        if (!${p}_m${k} && (chainIdx > ${m.seg} || (chainIdx == ${m.seg} && tNow >= ${num(m.t, 3)}))) {\n${fire(k, '            ')}\n        }\n`).join('') : '';
      const flush = marks.map((m, k) => `        if (!${p}_m${k}) {\n${fire(k, '            ')}\n        }\n`).join('');
      methods.push(`    // state ${n}: follow ${cmt(node.path)}${marks.length ? ' (' + marks.length + ' marker(s) polled by chain index and t)' : ''}\n    private boolean run${n}() {\n        if (!${p}_started) {\n            ${p}_started = true;\n            robot.auto.followPath(${f}, true, FOLLOW_TIMEOUT_S);\n            return false;\n        }\n${poll}        if (robot.auto.isBusyOrTimeout()) {\n            return false;\n        }\n${flush}        if (robot.auto.timedOut()) {\n            robot.auto.cancelMove();   // stuck: hold here and move on\n        }\n        ${p}_done = true;\n        return true;\n    }\n\n    private void cancel${n}() {\n        if (${p}_started && !${p}_done) {\n            robot.auto.cancelMove();\n            ${p}_done = true;\n        }\n    }`);
    } else if (T === 'wait') {
      decl.push(`private boolean ${p}_started;`, `private double ${p}_t0;`);
      methods.push(`    // state ${n}: wait ${num(secs(node.seconds), 3)} s\n    private boolean run${n}() {\n        if (!${p}_started) {\n            ${p}_started = true;\n            ${p}_t0 = clock.seconds();\n        }\n        return clock.seconds() - ${p}_t0 >= ${num(secs(node.seconds), 3)};\n    }\n\n    private void cancel${n}() {\n    }`);
    } else if (T === 'waitUntil') {
      const expr = conditionExpr(node.condition, ctx), to = secs(node.timeout);
      decl.push(`private boolean ${p}_started;`, `private double ${p}_t0;`);
      methods.push(`    // state ${n}: waitUntil ${cmt(typeof node.condition === 'string' ? node.condition : (node.condition && (node.condition.name || node.condition.type)) || '')}\n    private boolean run${n}() {\n        if (!${p}_started) {\n            ${p}_started = true;\n            ${p}_t0 = clock.seconds();\n        }\n        if (${expr}) {\n            return true;\n        }\n` + (to > 0 ? `        return clock.seconds() - ${p}_t0 >= ${num(to, 3)};   // timeout\n` : '        return false;   // no timeout set\n') + `    }\n\n    private void cancel${n}() {\n    }`);
    } else if (T === 'command') {
      const name = String(node.name == null ? '' : node.name);
      if (name === 'autoAim') {
        // Non-blocking: AutoAimLogic is stepped once per robot loop (same logic as AutoAim.run's loop).
        const a = node.args && typeof node.args === 'object' ? node.args : {};
        const timeout = secs(a.seconds) ? num(secs(a.seconds), 3) : `${AUTOAIM_TUNABLES_CLASS}.TIMEOUT_S`;
        const shootAfterOn = a.shootAfter === true;
        decl.push(`private boolean ${p}_started, ${p}_done, ${p}_feeding;`, `private ${AUTOAIM_LOGIC_CLASS} ${p}_logic;`);
        if (shootAfterOn) decl.push(`private double ${p}_feedT0;`);
        const feedTail = shootAfterOn
          ? `        if (clock.seconds() - ${p}_feedT0 >= ${AUTOAIM_TUNABLES_CLASS}.SHOOT_AFTER_MAX_S) {\n            robot.intake.stop();\n            ${p}_done = true;\n            return true;\n        }\n        if (!robot.feedIfReady(true)) {\n            robot.intake.stop();\n        }\n        return false;\n`
          : `        robot.intake.stop();\n        ${p}_done = true;\n        return true;\n`;
        const aimBlock = shootAfterOn
          ? `        if (!${p}_feeding) {\n            ${AUTOAIM_LOGIC_CLASS}.Output out = ${p}_logic.update(clock.seconds(), true, robot.shooter.ready());\n            if (out.transferOn) {\n                robot.intake.feed();\n            } else {\n                robot.intake.stop();\n            }\n            if (!out.done) {\n                return false;\n            }\n            ${p}_feeding = true;\n            ${p}_feedT0 = clock.seconds();\n        }\n`
          :`        ${AUTOAIM_LOGIC_CLASS}.Output out = ${p}_logic.update(clock.seconds(), true, robot.shooter.ready());\n        if (out.transferOn) {\n            robot.intake.feed();\n        } else {\n            robot.intake.stop();\n        }\n        if (!out.done) {\n            return false;\n        }\n`;
        // AutoAimLogic.update returns out.transferOn while aligning; when aim ends, shootAfter feeds, otherwise the state ends.
        methods.push(`    // state ${n}: autoAim (timeout ${timeout} s; AutoAimLogic drives flywheel, turret and hood; transfer on while aligning per ${AUTOAIM_TUNABLES_CLASS}.TRANSFER_ON_START)${shootAfterOn ? ' then feeds while ready' : ''}\n    private boolean run${n}() {\n        if (!${p}_started) {\n            ${p}_started = true;\n            ${p}_logic = new ${AUTOAIM_LOGIC_CLASS}(${timeout}, ${shootAfterOn ? 'true' : 'false'}, ${AUTOAIM_TUNABLES_CLASS}.TRANSFER_ON_START);\n            if (robot.shooter != null) {\n                robot.shooter.setSpinning(true);\n            }\n        }\n        if (robot.shooter == null) {\n            ${p}_done = true;   // no shooter: nothing to aim (same as AutoAim.run)\n            return true;\n        }\n${aimBlock}${feedTail}    }\n\n    private void cancel${n}() {\n        if (${p}_started && !${p}_done) {\n            robot.intake.stop();   // stop the transfer that aiming turned on\n            ${p}_done = true;\n        }\n    }`);
      } else if (TIMED.has(name)) {
        const a = node.args && typeof node.args === 'object' ? node.args : {};
        const dur = secs(a.seconds) || (name === 'shoot' ? 1.0 : 3.0);   // ASSUMED default durations (not measured); set args.seconds
        decl.push(`private boolean ${p}_started, ${p}_done;`, `private double ${p}_t0;`);
        methods.push(`    // state ${n}: ${name} (feeds while the shot is ready, for ${num(dur, 3)} s)\n    private boolean run${n}() {\n        if (!${p}_started) {\n            ${p}_started = true;\n            ${p}_t0 = clock.seconds();\n        }\n        if (clock.seconds() - ${p}_t0 >= ${num(dur, 3)}) {\n            robot.intake.stop();\n            ${p}_done = true;\n            return true;\n        }\n        robot.feedIfReady(true);\n        return false;\n    }\n\n    private void cancel${n}() {\n        if (${p}_started && !${p}_done) {\n            robot.intake.stop();\n            ${p}_done = true;\n        }\n    }`);
      } else {
        const st = instantStatements(name, node.args, ctx);
        methods.push(`    // state ${n}: ${cmt(name)}\n    private boolean run${n}() {\n${st.map(s => '        ' + s).join('\n')}\n        return true;\n    }\n\n    private void cancel${n}() {\n    }`);
      }
    } else if (T === 'sequential' || T === 'parallel' || T === 'race' || T === 'deadline') {
      const ks = kids(node), ids = ks.map(walk), K = ks.length;
      if (T === 'sequential') {
        decl.push(`private int ${p}_pc;`);
        const cases = ids.map((c, i) => `            case ${i}:\n                if (!run${c.n}()) {\n                    return false;\n                }\n                break;`).join('\n');
        const ccases = ids.map((c, i) => `            case ${i}:\n                cancel${c.n}();\n                break;`).join('\n');
        methods.push(`    // state ${n}: sequential (${K} steps)\n    private boolean run${n}() {\n        while (${p}_pc < ${K}) {\n            switch (${p}_pc) {\n${cases}${cases ? '\n' : ''}            default:\n                break;\n            }\n            ${p}_pc++;\n        }\n        return true;\n    }\n\n    private void cancel${n}() {\n        switch (${p}_pc) {\n${ccases}${ccases ? '\n' : ''}        default:\n            break;\n        }\n    }`);
      } else if (T === 'parallel') {
        decl.push(`private final boolean[] ${p}_flags = new boolean[${K}];`);
        const runs = ids.map((c, i) => `        if (!${p}_flags[${i}]) {\n            if (run${c.n}()) {\n                ${p}_flags[${i}] = true;\n            } else {\n                all = false;\n            }\n        }`).join('\n');
        const cans = ids.map((c, i) => `        if (!${p}_flags[${i}]) {\n            cancel${c.n}();\n        }`).join('\n');
        methods.push(`    // state ${n}: parallel (${K} branches run together, ends when all end)\n    private boolean run${n}() {\n        boolean all = true;\n${runs}${runs ? '\n' : ''}        return all;\n    }\n\n    private void cancel${n}() {\n${cans}${cans ? '\n' : ''}    }`);
      } else if (T === 'race') {
        decl.push(`private final boolean[] ${p}_flags = new boolean[${K}];`);
        const runs = ids.map((c, i) => `        if (winner < 0 && run${c.n}()) {\n            winner = ${i};\n        }`).join('\n');
        const cans = ids.map((c, i) => `            if (winner != ${i}) {\n                cancel${c.n}();\n            }`).join('\n');
        const cans2 = ids.map(c => `        cancel${c.n}();`).join('\n');
        methods.push(`    // state ${n}: race (${K} branches, ends when the first ends; the rest are cancelled)\n    private boolean run${n}() {\n        int winner = -1;\n${runs}${runs ? '\n' : ''}        if (winner >= 0) {\n${cans}${cans ? '\n' : ''}            return true;\n        }\n        return ${K === 0 ? 'true' : 'false'};\n    }\n\n    private void cancel${n}() {\n${cans2}${cans2 ? '\n' : ''}    }`);
      } else {
        let d = Number.isInteger(node.deadline) ? node.deadline : 0;
        if (d < 0 || d >= K) d = 0;
        decl.push(`private final boolean[] ${p}_flags = new boolean[${K}];`);
        if (K === 0) {
          methods.push(`    // state ${n}: deadline (empty)\n    private boolean run${n}() {\n        return true;\n    }\n\n    private void cancel${n}() {\n    }`);
        } else {
          const others = ids.map((c, i) => ({ c, i })).filter(x => x.i !== d);
          const cancelOthers = others.map(x => `            if (!${p}_flags[${x.i}]) {\n                cancel${x.c.n}();\n            }`).join('\n');
          const runOthers = others.map(x => `        if (!${p}_flags[${x.i}] && run${x.c.n}()) {\n            ${p}_flags[${x.i}] = true;\n        }`).join('\n');
          const cans = ids.map((c, i) => `        if (!${p}_flags[${i}]) {\n            cancel${c.n}();\n        }`).join('\n');
          methods.push(`    // state ${n}: deadline (branch ${d} decides; the rest are cancelled when it ends)\n    private boolean run${n}() {\n        if (run${ids[d].n}()) {\n            ${p}_flags[${d}] = true;\n${cancelOthers}${cancelOthers ? '\n' : ''}            return true;\n        }\n${runOthers}${runOthers ? '\n' : ''}        return false;\n    }\n\n    private void cancel${n}() {\n${cans}\n    }`);
        }
      }
    } else {
      methods.push(`    // state ${n}: unknown node type "${cmt(T)}" (skipped)\n    private boolean run${n}() {\n        return true;\n    }\n\n    private void cancel${n}() {\n    }`);
    }
    void label;
    return out;
  }
  const root = auto.root ? walk(auto.root) : null;
  // methods were pushed children-first; keep a stable order by state number
  const ordered = methods.map(m => ({ n: Number(/state (\d+):/.exec(m)[1]), m })).sort((a, b) => a.n - b.n).map(x => x.m);
  return { decl, methods: ordered, rootRun: root ? `run${root.n}()` : 'true', count: counter };
}

// ---------- ivy style (OPTIONAL; Ivy is NOT a dependency of this project) ----------
function ivyExpr(node, ctx, indent) {
  node = normAutoAim(node);
  const pad = '    '.repeat(indent), pad1 = '    '.repeat(indent + 1);
  const T = node && node.type;
  const list = cs => cs.map(c => pad1 + ivyExpr(c, ctx, indent + 1)).join(',\n');
  if (T === 'follow') return `follow(robot.auto.getFollower(), ${ctx.pathNames.get(node.path) || 'path_' + ident(node.path)})`;
  if (T === 'wait') return `waitSeconds(${num(secs(node.seconds), 3)})   /* UNVERIFIED Ivy name */`;
  if (T === 'waitUntil') return `waitUntil(() -> ${conditionExpr(node.condition, ctx)})   /* UNVERIFIED Ivy name */`;
  if (T === 'command') {
    if (node.name === 'autoAim') return 'instant(() -> { })   /* TODO: "autoAim" is timed (AutoAimLogic per tick); wrap it in a duration */';
    if (TIMED.has(node.name)) return `instant(() -> robot.feedIfReady(true))   /* TODO: "${jstr(node.name)}" is timed; wrap in a duration */`;
    const st = instantStatements(node.name, node.args, ctx);
    return `instant(() -> { ${st.join(' ')} })`;
  }
  const ks = Array.isArray(node.children) ? node.children : [];
  if (T === 'deadline') {
    let d = Number.isInteger(node.deadline) ? node.deadline : 0; if (d < 0 || d >= ks.length) d = 0;
    const ordered = ks.length ? [ks[d], ...ks.filter((_, i) => i !== d)] : [];
    return `deadline(\n${list(ordered)}\n${pad})`;
  }
  const fn = T === 'parallel' ? 'parallel' : T === 'race' ? 'race' : 'sequential';
  return ks.length ? `${fn}(\n${list(ks)}\n${pad})` : `${fn}()`;
}

// ---------- export warnings ----------
/** How many follow nodes can be live in the same tick (parallel/race/deadline add up, sequential takes the max). */
function liveFollows(node) {
  node = normAutoAim(node);
  if (!node || typeof node !== 'object') return 0;
  if (node.type === 'follow') return 1;
  const ks = Array.isArray(node.children) ? node.children : [];
  if (node.type === 'sequential') return ks.reduce((m, c) => Math.max(m, liveFollows(c)), 0);
  if (node.type === 'parallel' || node.type === 'race' || node.type === 'deadline') return ks.reduce((a, c) => a + liveFollows(c), 0);
  return 0;
}
/** Behavioural problems the generated Java would have on the real robot (it compiles either way). Returns [{code, message}]. */
export function exportWarnings(auto) {
  const out = [];
  const seen = new Set();
  const add = (code, message) => { if (!seen.has(code + message)) { seen.add(code + message); out.push({ code, message }); } };
  const root = auto && auto.root;
  (function walk(node) {
    node = normAutoAim(node);
    if (!node || typeof node !== 'object') return;
    const ks = Array.isArray(node.children) ? node.children : [];
    if (['parallel', 'race', 'deadline'].includes(node.type) && liveFollows(node) > 1) add('concurrent-follow', node.type + ' group runs more than one follow at once. The robot has ONE Pedro follower, the last followPath wins and a cancelled branch holds the pose of the winner too. Keep at most one follow per group.');
    if (node.type === 'deadline' && !(Number.isInteger(node.deadline) && node.deadline >= 0 && node.deadline < ks.length)) add('bad-deadline', 'deadline index out of range. Exported Java uses branch 0.');
    if (node.type === 'waitUntil' && !(secs(node.timeout) > 0)) add('no-timeout', 'waitUntil has no timeout and can hang the auto forever. Set a timeout.');
    if (node.type === 'waitUntil') { const c = node.condition && typeof node.condition === 'object' ? (node.condition.name || node.condition.type) : node.condition; const tmp = { stubs: new Map() }; conditionExpr(c, tmp); if (tmp.stubs.size) add('unknown-condition', 'waitUntil condition "' + jstr(c) + '" is unknown. A stub that returns false was emitted (waits for the timeout).'); }
    ks.forEach(walk);
  })(root);
  for (const [id, p] of Object.entries((auto && auto.paths) || {})) {
    for (const m of (p && p.markers) || []) {
      const c = normAutoAim(m && m.command);
      if (c && c.type === 'command' && TIMED.has(c.name)) add('timed-marker', 'path "' + jstr(id) + '" marker "' + jstr(c.name) + '" is timed. Markers cannot run timed commands (exported as TODO). Use a parallel group.');
    }
  }
  return out;
}

// ---------- public API ----------
function header(auto, cls, style, opts) {
  const alliance = String(auto.alliance || '').toLowerCase();
  const lines = [];
  lines.push(`/** ${cmt(auto.name) || cls} (${cmt(auto.alliance) || 'no'} alliance). Generated by the BIOBUZZ field sim auto planner; edit the TODO stubs, not the state methods. */`);
  void alliance; void opts; void style;
  const warns = exportWarnings(auto);
  if (warns.length) { lines.push('/*', ' * EXPORT WARNINGS (review before running on the robot):'); for (const w of warns) lines.push(' *  - [' + w.code + '] ' + cmt(w.message)); lines.push(' */'); }
  return lines.join('\n');
}
function allianceStmt(auto) {
  const a = String(auto.alliance || '').toLowerCase();
  if (a === 'red') return '        RobotSettings.Robot.ALLIANCE = Alliance.RED;';
  if (a === 'blue') return '        RobotSettings.Robot.ALLIANCE = Alliance.BLUE;';
  return '        // alliance "' + cmt(auto.alliance) + '" not set here: RobotSettings.Robot.ALLIANCE is left as configured';
}
function stubTexts(ctx) {
  return [...ctx.stubs.entries()].sort((a, b) => a[0] < b[0] ? -1 : 1).filter(e => e[1]).map(e => '    ' + e[1]);
}

export function exportAutoJava(auto, opts = {}) {
  auto = withEventMarkers(auto || {});
  const style = opts.style === 'ivy' ? 'ivy' : 'statemachine';
  const cls = sanitizeClassName(opts.className || auto.name || 'GeneratedAuto');
  const pkg = opts.packageName === undefined ? 'commands.autonomous' : opts.packageName;
  const group = jstr(opts.group || 'Auto') || 'Auto';
  const ctx = { stubs: new Map(), pathNames: new Map() };
  const code = buildPathCode(auto, 'robot.auto.pathBuilder()', true, ctx);
  if (style === 'ivy') ctx.stubs.delete('autoAim');   // ivy never emits path markers, so the marker helper is not needed
  const L = [];
  if (pkg) L.push(`package ${pkg};`, '');
  const imports = ['com.pedropathing.geometry.BezierCurve', 'com.pedropathing.geometry.BezierLine', 'com.pedropathing.geometry.Pose', 'com.pedropathing.paths.PathChain', 'com.qualcomm.robotcore.eventloop.opmode.Autonomous', 'com.qualcomm.robotcore.eventloop.opmode.LinearOpMode', 'com.qualcomm.robotcore.util.ElapsedTime', 'commands.field.Alliance', 'robot.Robot', 'robot.RobotSettings'];
  // autoAim: the state machine uses AutoAimLogic/AutoAimTunables; import them only when such a node exists (not for ivy)
  if (style !== 'ivy' && (treeHasAutoAim(auto.root) || ctx.usesAutoAim)) imports.push(...AUTOAIM_IMPORTS);
  if (style === 'ivy') imports.push('static com.pedropathing.ivy.pedro.PedroCommands.*');
  const std = imports.filter(i => !i.startsWith('static ')).sort().map(i => `import ${i};`);
  const stat = imports.filter(i => i.startsWith('static ')).map(i => `import ${i};`);
  L.push(...std, ...stat, '');
  if (style === 'ivy') {
    L.push('/*', ' * IVY STYLE REQUESTED. Ivy is NOT a dependency of this project (not in build.dependencies.gradle / TeamCode/build.gradle).',
      ` * To use this file add to the dependencies block of TeamCode/build.dependencies.gradle (or TeamCode/build.gradle):`,
      ` *     ${IVY_GRADLE_LINE}`,
      ' * The Ivy names below were NOT verified against an Ivy jar (none is in the Gradle cache). Documented: sequential, parallel, race, deadline,',
      ' * follow(follower, path), instant. Marked UNVERIFIED: waitSeconds, waitUntil, and the scheduler calls. follow() may need a Path, not a PathChain.',
      ' * If you do not want Ivy, regenerate with style "statemachine" (no extra dependency).', ' */');
  }
  L.push(header(auto, cls, style, opts));
  L.push(`@Autonomous(name = "${jstr(auto.name) || cls}", group = "${group}")`);
  L.push(`public class ${cls} extends LinearOpMode {`);
  L.push('    private Robot robot;', '    private final ElapsedTime clock = new ElapsedTime();', '    /** Safety bound per follow (s): a stuck robot gives up and moves on instead of hanging. Raise it for very long paths. */', '    public static double FOLLOW_TIMEOUT_S = 15;', '');
  for (const f of code.fields) L.push('    ' + f);
  L.push('');
  if (code.pathFields.length) L.push(`    private PathChain ${code.pathFields.join(', ')};`, '');
  L.push('    private void buildPaths() {', code.body.length ? code.body.join('\n\n') : '        // no paths defined', '    }', '');
  let rootRun;
  if (style === 'ivy') {
    const expr = auto.root ? ivyExpr(auto.root, ctx, 2) : 'sequential()';
    L.push('    private boolean finished;', '', '    private void startRoutine() {', `        // UNVERIFIED: the Ivy scheduler entry point name (check the Ivy Scheduler class).`, `        com.pedropathing.ivy.Scheduler.schedule(${expr});`, '    }', '');
    rootRun = 'finished';
  } else {
    const sm = emitStateMachine(auto, ctx);
    for (const d of sm.decl) L.push('    ' + d);
    if (sm.decl.length) L.push('');
    L.push(...sm.methods.map(m => m + '\n'));
    rootRun = sm.rootRun;
  }
  const stubs = stubTexts(ctx);
  if (stubs.length) L.push('    // ---- TODO stubs: need implementation ----', ...stubs.map(s => s + '\n'));
  L.push('    @Override', '    public void runOpMode() {');
  L.push(allianceStmt(auto));
  L.push('        robot = new Robot(hardwareMap, START_POSE);', '        buildPaths();', '        telemetry.addLine("' + (jstr(auto.name) || cls) + ' ready. Press start.");', '        telemetry.update();', '        waitForStart();', '        clock.reset();');
  if (style === 'ivy') L.push('        startRoutine();', '');
  else L.push('        boolean finished = false;', '');
  const loop = ['        while (opModeIsActive()) {', '            robot.update();'];
  if (style === 'ivy') loop.push('            com.pedropathing.ivy.Scheduler.execute();   // UNVERIFIED name');
  else loop.push(`            if (!finished) {
                finished = ${rootRun};
            }`);
  if (style !== 'ivy' && ctx.usesAutoAim) loop.push('            if (autoAimCmd != null) autoAimCmd.update();   // non-blocking autoAim marker: one step per tick');
  loop.push('            telemetry.addData("finished", finished);', '            robot.addTelemetry(telemetry);', '            telemetry.update();', '        }');
  L.push('        try {', ...loop.join('\n').split('\n').map(l => '    ' + l), '        } finally {', '            robot.stop();   // always: saves pose and turret angle, stops the follower and motors', '        }', '    }', '}', '');
  return L.join('\n');
}

export function exportPathsOnly(auto) {
  auto = auto || {};
  const ctx = { stubs: new Map(), pathNames: new Map() };
  const code = buildPathCode(auto, 'follower.pathBuilder()', false, ctx);
  const L = ['// Pose constants and PathChain builders (markers omitted). Needs: BezierLine, BezierCurve, Pose, PathChain, Follower imports.'];
  for (const f of code.fields) L.push(f);
  L.push('');
  if (code.pathFields.length) L.push(`private PathChain ${code.pathFields.join(', ')};`, '');
  L.push('public void buildPaths(Follower follower) {', code.body.length ? code.body.join('\n\n') : '    // no paths defined', '}', '');
  return L.join('\n');
}

export function listCommandStubs(auto) {
  auto = auto || {};
  const ctx = { stubs: new Map(), pathNames: new Map() };
  buildPathCode(auto, 'x', true, ctx);
  emitStateMachine(auto, ctx);
  return [...ctx.stubs.keys()].sort();
}

// ---------- structure checker ----------
function stripCode(src) {
  // remove comments and string/char literals, reporting unterminated ones
  let out = '', i = 0, problems = [];
  while (i < src.length) {
    const c = src[i], d = src[i + 1];
    if (c === '/' && d === '/') { while (i < src.length && src[i] !== '\n') i++; continue; }
    if (c === '/' && d === '*') {
      const e = src.indexOf('*/', i + 2);
      if (e < 0) { problems.push('unterminated block comment'); break; }
      i = e + 2; out += ' '; continue;
    }
    if (c === '"' || c === "'") {
      let j = i + 1;
      while (j < src.length && src[j] !== c && src[j] !== '\n') { if (src[j] === '\\') j++; j++; }
      if (src[j] !== c) { problems.push(`unterminated ${c === '"' ? 'string' : 'char'} literal`); i = j; continue; }
      out += c + c; i = j + 1; continue;
    }
    out += c; i++;
  }
  return { code: out, problems };
}
export function checkJavaStructure(javaText) {
  const problems = [];
  const text = String(javaText == null ? '' : javaText);
  if (!text.trim()) return { ok: false, problems: ['empty output'] };
  const { code, problems: sp } = stripCode(text);
  problems.push(...sp);
  const stack = [], pairs = { ')': '(', ']': '[', '}': '{' };
  for (const ch of code) {
    if ('([{'.includes(ch)) stack.push(ch);
    else if (')]}'.includes(ch)) { if (stack.pop() !== pairs[ch]) { problems.push(`unbalanced "${ch}"`); break; } }
  }
  if (stack.length) problems.push(`unclosed "${stack[stack.length - 1]}"`);
  if (/\bNaN\b|\bInfinity\b|\bundefined\b|\[object Object\]|\bnull\s*\)\s*;?\s*\/\/\s*bad/.test(code)) problems.push('contains NaN/Infinity/undefined');
  const cm = /\bclass\s+([A-Za-z_$][\w$]*)/.exec(code);
  if (!cm) problems.push('no class declaration');
  else if (KEYWORDS.has(cm[1])) problems.push(`class name "${cm[1]}" is a Java keyword`);
  if (/\bclass\s+[^A-Za-z_$\s]/.test(code)) problems.push('invalid class name');
  // path fields: every referenced path_X must be declared in a "PathChain a, b;" line
  const declared = new Set();
  for (const m of code.matchAll(/\bPathChain\s+([\w\s,]+);/g)) m[1].split(',').forEach(s => declared.add(s.trim()));
  for (const m of code.matchAll(/\bpath_\w+/g)) if (!declared.has(m[0])) { problems.push(`path field "${m[0]}" is used but not declared`); declared.add(m[0]); }
  // state machine: every runN/cancelN called must be defined; every state falls through correctly
  const defRun = new Set([...code.matchAll(/\bprivate boolean run(\d+)\(\)\s*\{/g)].map(m => m[1]));
  const defCancel = new Set([...code.matchAll(/\bprivate void cancel(\d+)\(\)\s*\{/g)].map(m => m[1]));
  for (const m of code.matchAll(/\b(run|cancel)(\d+)\(\)/g)) {
    const set = m[1] === 'run' ? defRun : defCancel;
    if (!set.has(m[2]) && !new RegExp(`private (boolean|void) ${m[1]}${m[2]}\\(`).test(code)) { problems.push(`${m[1]}${m[2]}() is called but not defined`); set.add(m[2]); }
  }
  for (const n of defRun) if (!defCancel.has(n)) problems.push(`state ${n} has no cancel${n}()`);
  // each switch case body must end in break/return
  for (const m of code.matchAll(/\bcase\s+\d+\s*:/g)) {
    let i = m.index + m[0].length, depth = 0;
    const start = i;
    while (i < code.length) {
      const ch = code[i];
      if (ch === '{') depth++;
      else if (ch === '}') { if (depth === 0) break; depth--; }
      else if (depth === 0 && /^(case\s+\d+\s*:|default\s*:)/.test(code.slice(i, i + 12)) && /\W/.test(code[i - 1] || ' ')) break;
      i++;
    }
    const body = code.slice(start, i).trim();
    if (!/(\bbreak|\breturn\b[^;]*);$/.test(body)) { problems.push(`switch case falls through without break/return: ${body.slice(0, 40)}`); break; }
  }
  // each run method must end by returning
  for (const m of code.matchAll(/\bprivate boolean run\d+\(\)\s*\{/g)) {
    let i = m.index + m[0].length, depth = 1;
    while (i < code.length && depth) { if (code[i] === '{') depth++; else if (code[i] === '}') depth--; i++; }
    if (!/return\b[^;]*;\s*\}$/.test(code.slice(m.index, i).trim())) { problems.push('a run method does not end with return'); break; }
  }
  return { ok: problems.length === 0, problems };
}
