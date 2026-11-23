// physics_glue.js - integrator glue between app.js and the physics area modules in web/js/physics/<area>/.
// Owns the runtime switches (one per model) and the per-step calls that tie the models into the sim loop.
// No DOM, no three.js: node-testable (tests_js/integration/physics_glue.test.mjs, round2.test.mjs).
//
// Units: inches, seconds, radians, rpm, volts, amps (the app's units). The area modules convert internally.
// Every default below is tagged [assumed] (an integrator choice) or [sourced] (with where it comes from).
//
// Wired (area verifier passed):
//   motor-model        MotorMecanumDrive    driveModel 'motor'      (DC motor + gearbox + traction cap per wheel)
//   mecanum-dynamics   MecanumDynamics      driveModel 'dynamics'   (rigid-body mecanum, roller losses, Coulomb/rolling)
//   wheel-friction     FrictionMecanumDrive driveModel 'friction'   (per-wheel Stribeck friction + load transfer)
//   battery-sag        ElectricalSystem     batterySag              (bus voltage, sag, fuse, hub brownout); ticks on EVERY
//                                                                   motion source and feeds every drive and flywheel plant
//   weight-transfer    WeightTransferSim    weightTransfer          (load-aware traction limit, pitch/roll, tip-over)
//   flywheel-dynamics  FlywheelPlant        flywheelModel 'dynamics'(motor-driven flywheel, physical shot dip)
//   velocity-pidf      FlywheelSystem       flywheelModel 'pidf'    (repo SDK PIDF loop, encoder, loop timing)
//   pinpoint-odometry  OdometrySim          odometry preset         (pods + IMU + Pinpoint firmware + host read)
//   imu-heading        GyroModel            imuModel 'gyro'         (replaces the Pinpoint's simple IMU yaw model)
//   wall-collisions    collision.js         robotCollision 'contact'(speculative contact solver, legacy fallback)
//   rigid-body-solver  World/addRobot       robotCollision 'rigid'  (Box2D-style rigid world, robot as dynamic body)
//   timestep-determinism FixedClock, createRng  fixedStep / seed / renderInterp (alpha)
//   shooter-exit       (via shotrules.js)   exitModel 'physical'    (set on SHOOTER.exitModel by app.js)
//   hive-physics       (via hivecycle.js)   hiveTipModel/hiveMotion (HiveDynamics.setRule)
//   pedro-fidelity     (via autorunner.js)  autoFollower / pedroPlant (AutoRunner.followerModel / pedroParams)
//   ball-aerodynamics  (via balls.js)       ballAero                (BALL_MODELS.aero)
//   ball-ball-collisions (via balls.js)     ballBall                (BALL_MODELS.ballBall)
// Left out: robot-ball-contact (balls.js owner: not a strict improvement), shooter pre-existing-test area (hivemesh defect).
import { MotorMecanumDrive } from './physics/motor-model/mecanum.js';
import { GEARBOXES, MOTORS, DRIVE as MOTOR_DRIVE } from './physics/motor-model/params.js';
import { MecanumDynamics } from './physics/mecanum-dynamics/index.js';
import { FrictionMecanumDrive } from './physics/wheel-friction/index.js';
import { ElectricalSystem } from './physics/battery-sag/system.js';
import { WeightTransferSim } from './physics/weight-transfer/index.js';
import { FlywheelPlant } from './physics/flywheel-dynamics/flywheel_plant.js';
import { paramsFromRobotSpec } from './physics/flywheel-dynamics/params.js';
import { FlywheelSystem } from './physics/velocity-pidf/flywheel_system.js';
import { OdometrySim } from './physics/pinpoint-odometry/odometry.js';
import { GyroModel, resolveImuConfig, createRng as imuRng } from './physics/imu-heading/index.js';
import { FixedClock } from './physics/timestep-determinism/fixedclock.js';
import { createRng } from './physics/timestep-determinism/prng.js';
import { lerpPose } from './physics/timestep-determinism/interp.js';
import { TS } from './physics/timestep-determinism/params.js';
import { buildObstacles, resolveRobot, slideVelocity, createWallContact, stepRobotContact, resetContactCache } from './collision.js';
import { createFieldWorld, addRobot, teleportRobot, readRobot, stepWorld, IN_TO_M as RB_IN_TO_M } from './physics/rigid-body-solver/adapter.js';
import { driverForwardOffset } from './driver.js';
import { BALL_MODELS } from './balls.js';

const IN_TO_M = 0.0254;            // [m/in] exact (SI definition)
const LB_TO_KG = 0.45359237;       // [kg/lb] exact (SI definition)
const TELEPORT_IN = 24;            // [in] [assumed] one-step jump larger than this = teleport (placeRobot, reset), not motion
const VAR_DT_MAX_S = 0.05;         // [s] [sourced] the app's existing variable-step clamp (app.js frameBody, before this glue)
const NOMINAL_V = 12;              // [V] [sourced] goBILDA 5203 rated voltage (motor-model MOTORS.gobilda5203.nominalV)
const MIN_PLANT_V = 1e-3;          // [V] [assumed] numerical floor so a dead bus never divides by zero in a controller
const RIGID_HIT_DV_INS = 1;        // [in/s] [assumed] see moveRobot 'rigid'

/** Allowed values of every switch (the panel builds its controls from this). */
export const PHYS_OPTIONS = Object.freeze({
  driveModel: ['kinematic', 'motor', 'dynamics', 'friction'],
  robotCollision: ['legacy', 'contact', 'rigid'],
  flywheelModel: ['legacy', 'dynamics', 'pidf'],
  exitModel: ['legacy', 'physical'],
  odometry: ['off', 'ideal', 'nominal', 'harsh'],
  imuModel: ['basic', 'gyro'],
  batteryPreset: ['fresh', 'nominal', 'tired', 'dead'],
  fixedHz: [60, 120, 240],
  hiveTipModel: ['rule', 'torque'],
  hiveMotion: ['dynamic', 'ease'],
  autoFollower: ['auto', 'replay', 'trapezoid', 'pedro'],
  pedroPlant: ['library', 'spec'],
  ballAero: ['legacy', 'physical'],
  ballBall: ['legacy', 'solver'],
});

/** Defaults. robot_spec.json "physics" overrides these; the panel (and localStorage) override that. */
export const PHYS_DEFAULTS = Object.freeze({
  fixedStep: true,          // [assumed] run simUpdate at a fixed step (FixedClock) instead of the frame dt
  fixedHz: 120,             // [Hz] [assumed] 2 steps per 60 Hz display frame; 60 Hz matches autorunner.js FIXED_DT = 1/60
  renderInterp: true,       // [assumed] draw the robot/balls at lerp(prev, curr, FixedClock.alpha) (visual only)
  driveModel: 'motor',      // [assumed] motor-model drive; 'kinematic' = the old slew-rate MecanumDrive
  robotCollision: 'legacy', // [assumed] see tests_js/integration/round2.test.mjs for why 'contact' is not the default
  batterySag: true,         // [assumed]
  batteryPreset: 'fresh',   // [assumed] battery-sag PRESETS key
  weightTransfer: true,     // [assumed]
  flywheelModel: 'dynamics',// [assumed] flywheel-dynamics FlywheelPlant
  exitModel: 'physical',    // [assumed] shooter-exit physical exit model (shotrules.js reads SHOOTER.exitModel)
  odometry: 'nominal',      // [assumed] pinpoint-odometry preset; 'off' = the aim uses the true pose
  imuModel: 'basic',        // [assumed] 'gyro' = imu-heading GyroModel inside the Pinpoint (opt-in, see round2 test)
  aimFromOdometry: true,    // [assumed] the shot solver aims from the believed (Pinpoint) pose, like the robot does
  hiveTipModel: 'rule',     // [sourced] hivecycle.js HIVE_PARAMS.tipModel default (owner 8-pollen rule)
  hiveMotion: 'dynamic',    // [sourced] hivecycle.js HIVE_PARAMS.motionModel default
  autoFollower: 'auto',     // [sourced] autorunner.js AutoRunner default followerModel (auto.settings still override it)
  pedroPlant: 'spec',       // [assumed] Pedro plant geometry/traction from robot_spec.json instead of the ESTIMATE defaults
  ballAero: 'legacy',       // [sourced] balls.js BALL_MODELS.aero default (shot solver tuned to the legacy drag)
  ballBall: 'legacy',       // [sourced] balls.js BALL_MODELS.ballBall default (solver loses restitution, see balls_models test)
  validationTools: false,   // [assumed] validation-tuning overlay + tuning panel (F8/F9), off: it costs a world scan per frame
  seed: TS.DEFAULT_SEED,    // [sourced] timestep-determinism params.js DEFAULT_SEED (itself assumed)
});

const BOOL_KEYS = ['fixedStep', 'renderInterp', 'batterySag', 'weightTransfer', 'aimFromOdometry', 'validationTools'];

/** Validate and merge a settings object onto a base. Unknown keys and bad values are dropped (reported in .rejected). */
export function mergeSettings(base, over) {
  const out = { ...base }, rejected = [];
  for (const [k, v] of Object.entries(over || {})) {
    if (k === 'seed') { if (Number.isInteger(v) && v >= 0) out.seed = v >>> 0; else rejected.push(k); continue; }
    if (BOOL_KEYS.includes(k)) { if (typeof v === 'boolean') out[k] = v; else rejected.push(k); continue; }
    if (PHYS_OPTIONS[k]) { if (PHYS_OPTIONS[k].includes(v)) out[k] = v; else rejected.push(k); continue; }
    rejected.push(k);
  }
  Object.defineProperty(out, 'rejected', { value: rejected, enumerable: false });
  return out;
}

const fin = (v) => typeof v === 'number' && Number.isFinite(v);

/** Drive gearing from robot_spec.json drive.gearing (motor id, gearbox id, belt ratio), or null (each model keeps its own
 *  assumed default). Returns {motor, gearbox, beltRatio, ratio (motor rev per wheel rev), wheelFreeRpm, freeSpeedInS}. */
export function driveGearingFromSpec(spec) {
  const g = spec && spec.drive && spec.drive.gearing;
  if (!g || !MOTORS[g.motor] || !GEARBOXES[g.gearbox] || !(fin(g.beltRatio) && g.beltRatio > 0)) return null;
  const ratio = GEARBOXES[g.gearbox].ratio * g.beltRatio;
  const w = Array.isArray(spec.wheels) && spec.wheels.length === 4 && spec.wheels.every((q) => fin(q.radius)) ? spec.wheels.reduce((s, q) => s + q.radius, 0) / 4 : null;
  const wheelFreeRpm = MOTORS[g.motor].freeSpeedRpm / ratio;
  return { motor: g.motor, gearbox: g.gearbox, beltRatio: g.beltRatio, ratio, wheelFreeRpm, freeSpeedInS: w ? wheelFreeRpm * 2 * Math.PI * w / 60 : NaN };
}

/** Motor-drive overrides from robot_spec.json (mass, wheel radius, wheel half-spacings, gearing). Missing fields keep the
 *  motor-model DRIVE defaults (physics/motor-model/params.js). */
export function motorDriveParamsFromSpec(spec) {
  const o = {};
  if (spec?.chassis && fin(spec.chassis.mass_lb) && spec.chassis.mass_lb > 0) o.massKg = spec.chassis.mass_lb * LB_TO_KG;
  const w = Array.isArray(spec?.wheels) ? spec.wheels : [];
  if (w.length === 4 && w.every((q) => fin(q.x) && fin(q.y) && fin(q.radius))) {
    o.wheelRadiusM = w.reduce((s, q) => s + q.radius, 0) / 4 * IN_TO_M;
    o.lxM = w.reduce((s, q) => s + Math.abs(q.x), 0) / 4 * IN_TO_M;   // half wheelbase
    o.lyM = w.reduce((s, q) => s + Math.abs(q.y), 0) / 4 * IN_TO_M;   // half track
  }
  const g = driveGearingFromSpec(spec);
  if (g) { o.motor = g.motor; o.gearbox = g.gearbox; o.extraRatio = g.beltRatio; }
  return o;
}

/** wheel-friction MOTOR overrides (values at the WHEEL shaft) from the spec gearing: free speed = motor free rpm / ratio,
 *  stall torque = bare-motor stall torque x ratio (wheel-friction applies its own drivetrainEfficiency). {} = its default. */
export function frictionDriveParamsFromSpec(spec) {
  const g = driveGearingFromSpec(spec);
  if (!g) return {};
  const m = MOTORS[g.motor];
  return { MOTOR: { freeSpeedRpm: g.wheelFreeRpm, stallTorqueNm: m.stallTorqueNm * g.ratio, stallCurrentA: m.stallCurrentA } };
}

/** mecanum-dynamics overrides from the spec gearing (motor shaft : wheel ratio). {} = its default. */
export function dynamicsDriveParamsFromSpec(spec) {
  const g = driveGearingFromSpec(spec);
  return g ? { gearRatio: g.ratio } : {};
}

/** pedro-fidelity resolveParams overrides derived from the Pedro constants the robot runs (library defaults, kept) plus the
 *  robot geometry in robot_spec.json. Only plant fields marked ESTIMATE in pedro-fidelity/params.js are replaced:
 *  trackWidthIn/wheelBaseIn from chassis.track/wheelbase [vault], muTraction from the drive models' shared mu
 *  (motor-model DRIVE.mu 0.7 [assumed]), stallCurrentA from the drive motor [sourced goBILDA 5203]. Pedro FollowerConstants /
 *  MecanumConstants stay the 2.1.0 library values, because TeamCode pedroPathing/Constants.java sets none of them. */
export function pedroParamsFromSpec(spec) {
  const plant = {};
  const ch = spec && spec.chassis;
  if (ch && fin(ch.track) && ch.track > 0) plant.trackWidthIn = ch.track;
  if (ch && fin(ch.wheelbase) && ch.wheelbase > 0) plant.wheelBaseIn = ch.wheelbase;
  plant.muTraction = MOTOR_DRIVE.mu;
  const g = driveGearingFromSpec(spec);
  if (g) plant.stallCurrentA = MOTORS[g.motor].stallCurrentA;
  return { plant };
}

/** imu-heading GyroModel behind the pinpoint-odometry Imu interface (reset(h), advance(om, dt), headingRad, rateRadS), so
 *  the Pinpoint firmware model reads the detailed gyro (bias GM/RW, scale asymmetry, temperature, g-sensitivity, LSB,
 *  latency, shocks) instead of its own white-noise + bias yaw. */
export class GyroImuAdapter {
  constructor(preset = 'pinpoint', seed = 1, overrides = {}) {
    this.cfg = resolveImuConfig(preset, overrides);
    this._g = new GyroModel(this.cfg, imuRng(seed >>> 0).fork('gyro'));
    this.flywheelOn = false;
    this.linAccelG = 0;
    this.reset(0);
  }
  reset(headingRad = 0) {
    const g = this._g;
    g.reset(0);
    // robot still before the match: finish the startup calibration window at zero true rate (as imu-heading system.js)
    for (let i = 0; i < 1000 && g.status === 'CALIBRATING'; i++) g.advance(0.05, 0, 0, { linAccelG: 0, flywheelOn: false });
    g.setYaw(headingRad);
  }
  advance(om, dt) { if (dt > 0) this._g.advance(dt, om, om, { linAccelG: this.linAccelG, flywheelOn: this.flywheelOn }); }
  shock(impactSpeedInS) { this._g.shock({ impactSpeedInS }); }
  /** Unwrapped (not LSB-quantised) yaw total at the sensor clock, latency applied: the Pinpoint integrates its deltas. */
  get headingRad() { return this._g.read(this._g.time).total; }
  get rateRadS() { return this._g.read(this._g.time).rate; }
  get gyro() { return this._g; }
}

/** IMU preset per odometry preset: ideal -> 'ideal' (bit-exact), nominal -> 'pinpoint', harsh -> 'pinpoint' with 3x noise
 *  and bias [assumed, mirrors the pinpoint-odometry 'harsh' intent]. */
const IMU_FOR_ODO = {
  ideal: ['ideal', {}],
  nominal: ['pinpoint', {}],
  harsh: ['pinpoint', { noiseDensityDpsRtHz: 0.015, biasInitSigmaDps: 0.012, gmSigmaDph: 9 }],
};

function makeOdometry(preset, imuModel, seed) {
  if (preset === 'off') return null;
  const o = new OdometrySim({ preset });
  if (imuModel === 'gyro') {
    // OdometrySim has no injection hook; the IMU instance is shared by the facade and the device (both private fields).
    const [p, ov] = IMU_FOR_ODO[preset] || IMU_FOR_ODO.nominal;
    const imu = new GyroImuAdapter(p, seed, ov);
    o._imu = imu; o._dev._imu = imu;
  }
  return o;
}
function wrapPi(a) { return Math.atan2(Math.sin(a), Math.cos(a)); }
const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

/** Driver stick -> body-frame fractions [F forward, L left, T ccw] (same mapping as driver.js MecanumDrive). */
export function stickToBody(cmd, h, { fieldCentric = false, headingOffset = 0, alliance = null } = {}) {
  const fwd = clamp(cmd.fwd || 0, -1, 1), str = clamp(cmd.strafe || 0, -1, 1), trn = clamp(cmd.turn || 0, -1, 1);
  if (!fieldCentric) return [fwd, str, trn];
  const off = (alliance ? driverForwardOffset(alliance) : 0) + headingOffset;
  const c = Math.cos(h), s = Math.sin(h), dx = -str, dy = fwd, co = Math.cos(off), so = Math.sin(off);
  const wx = dx * co - dy * so, wy = dx * so + dy * co;
  return [wx * c + wy * s, -wx * s + wy * c, trn];
}

export class PhysicsRuntime {
  /**
   * @param {object} o
   * @param {object|null} o.spec         parsed robot_spec.json (or null)
   * @param {object} o.settings          overrides of PHYS_DEFAULTS (already merged with spec.physics by the caller, or not)
   * @param {object} o.kinematicDrive    driver.js MecanumDrive (the legacy drive, kept for 'kinematic')
   * @param {object} o.plant             aimcontrol.js ShooterPlant (turret + hood stay with it in every flywheel mode)
   * @param {{x,y,h}} o.startPose        in, rad
   * @param {object|null} o.colliders    raw field colliders (app fieldData: bounds, boxes, circles) for moveRobot(), or null
   * @param {object|null} o.obstacles    collision.js buildObstacles(colliders) result (built here when omitted)
   */
  constructor({ spec = null, settings = {}, kinematicDrive, plant, startPose = { x: 0, y: 0, h: 0 }, colliders = null, obstacles = null } = {}) {
    this.spec = spec;
    this.cfg = mergeSettings(mergeSettings(PHYS_DEFAULTS, spec && spec.physics ? spec.physics.defaults : null), settings);
    this.kin = kinematicDrive;
    this.plant = plant;
    this.gearing = driveGearingFromSpec(spec);
    this.motorDriveParams = motorDriveParamsFromSpec(spec);
    this.motorDrive = new MotorMecanumDrive({ params: this.motorDriveParams });
    this.dynDrive = new MecanumDynamics({ spec, overrides: dynamicsDriveParamsFromSpec(spec) });
    this.fricDrive = new FrictionMecanumDrive({ spec, params: frictionDriveParamsFromSpec(spec) });
    // battery-sag -> motor drive: the drive reads the battery-sag bus voltage instead of its own ideal 12 V battery
    const baseBusV = this.motorDrive.power.busVoltage.bind(this.motorDrive.power);
    this.motorDrive.power.busVoltage = () => (this.cfg.batterySag && this.elec.last ? this.elec.last.vBus : baseBusV());
    this.elec = new ElectricalSystem({ preset: this.cfg.batteryPreset });
    this._elecFromSpec();
    this.wt = new WeightTransferSim(spec);
    this._mechKey = '';
    // flywheel plants: mutable battery copies so the battery-sag bus voltage can be fed in each step
    this.fwParams = paramsFromRobotSpec(spec && spec.shooter);   // partial overrides; FlywheelPlant builds the full set
    this.fwDyn = new FlywheelPlant(this.fwParams);
    this._fwBattery0 = { ...this.fwDyn.params.battery };          // its own default battery (12 V, 0 ohm, assumed there)
    this.fwPidf = new FlywheelSystem();
    this._pidfBattery0 = { ...this.fwPidf.battery.p };
    this.odo = makeOdometry(this.cfg.odometry, this.cfg.imuModel, this.cfg.seed);
    this._buildClock();
    this.rng = createRng(this.cfg.seed).stream('shotNoise');
    this._fwTarget = 0;
    this._prevPose = { ...startPose };
    this._renderPrev = null;
    this._events = [];
    this.tel = { steps: 0, h: 0 };
    // robot-vs-field collision (moveRobot): legacy resolveRobot, wall-collisions contact solver, or rigid-body world
    this.obstacles = obstacles || (colliders ? buildObstacles(colliders) : null);
    this.colliders = colliders;
    this.wallCtx = this.obstacles ? createWallContact(colliders || this.obstacles) : null;
    this._rigid = null;            // built lazily (only when 'rigid' is selected)
    this._applyBallModels();
    this.reset(startPose);
  }

  /** battery-sag drive electrical model: replace its [A] placeholders with the spec geometry/gearing. */
  _elecFromSpec() {
    const d = this.elec.driveParams, m = this.motorDriveParams;
    if (this.gearing) d.gearRatio = this.gearing.ratio;
    if (fin(m.wheelRadiusM)) d.wheelRadiusIn = m.wheelRadiusM / IN_TO_M;
    if (fin(m.massKg)) d.massKg = m.massKg;
    if (fin(m.lxM)) d.halfBaseIn = m.lxM / IN_TO_M;
    if (fin(m.lyM)) d.halfTrackIn = m.lyM / IN_TO_M;
  }

  _applyBallModels() { BALL_MODELS.aero = this.cfg.ballAero; BALL_MODELS.ballBall = this.cfg.ballBall; }

  _buildClock() {
    const hz = this.cfg.fixedHz;
    // maxFrameDt = the app's old 0.05 s clamp (not TS.MAX_FRAME_DT 0.25 s): a hitch never runs more than 0.05 s of
    // sim in one frame (at most 12 steps at 240 Hz), which keeps the expensive per-step shot evaluation bounded.
    this.clock = new FixedClock({ hz, controlDivider: Math.max(1, Math.round(hz / TS.CONTROL_HZ)), maxFrameDt: VAR_DT_MAX_S });
  }

  _rigidWorld() {
    if (!this._rigid && this.colliders) {
      const { world } = createFieldWorld(this.colliders);
      const m = this.motorDriveParams;
      const body = addRobot(world, { x: this._prevPose.x, y: this._prevPose.y, h: this._prevPose.h, drive: false, massKg: fin(m.massKg) ? m.massKg : undefined });
      this._rigid = { world, body };
    }
    return this._rigid;
  }

  /** Change one switch at runtime; returns the (possibly unchanged) value. Side effects keep every model consistent. */
  set(key, value) {
    const m = mergeSettings(this.cfg, { [key]: value });
    if (m.rejected.length) return this.cfg[key];
    const old = this.cfg[key];
    this.cfg = m;
    if (old === value) return value;
    switch (key) {
      case 'fixedHz': this._buildClock(); break;
      case 'fixedStep': this.clock.reset(this.clock.tick); break;
      case 'batteryPreset': this.elec.reset(value); break;
      case 'batterySag': this.elec.reset(this.cfg.batteryPreset); this._feedFlywheelBus(); break;
      case 'weightTransfer': this.wt.reset(); break;
      case 'odometry': case 'imuModel': this.odo = makeOdometry(this.cfg.odometry, this.cfg.imuModel, this.cfg.seed); if (this.odo) this.odo.reset(this._prevPose); break;
      case 'flywheelModel': {
        const rpm = this.plant ? this.plant.state.rpm : 0;
        this.fwDyn.reset(rpm);           // flywheel-dynamics resumes at the current rpm
        this.fwPidf.reset();             // velocity-pidf has no rpm seed: it spins up from rest
        break;
      }
      case 'seed': this.rng = createRng(this.cfg.seed).stream('shotNoise'); break;
      // every drive keeps its state in S.bodyV (wheel-friction re-syncs on an external edit), so the switch is seamless
      case 'driveModel': this.motorDrive.reset(); this.fricDrive.reset(); break;
      case 'robotCollision': if (this.wallCtx) resetContactCache(this.wallCtx); if (this._rigid) teleportRobot(this._rigid.body, this._prevPose, true); break;
      case 'ballAero': case 'ballBall': this._applyBallModels(); break;
      default: break;
    }
    return value;
  }

  /** Reset every model (robot reset / match reset). */
  reset(pose) {
    this.elec.reset(this.cfg.batteryPreset);
    this.wt.reset();
    this.motorDrive.reset();
    this.fricDrive.reset();
    this.fwDyn.reset(0);
    this.fwPidf.reset();
    if (pose) this._prevPose = { x: pose.x, y: pose.y, h: pose.h };
    if (this.odo) this.odo.reset(this._prevPose);
    this.clock.reset(0);
    this.rng = createRng(this.cfg.seed).stream('shotNoise');
    this._events = [];
    this._renderPrev = null;
    this._collReset();
  }

  /** Robot-only reset (the R key): chassis, drive motors, flywheels and odometry; the battery keeps its charge. */
  resetRobot(pose) {
    this.wt.reset();
    this.motorDrive.reset();
    this.fricDrive.reset();
    this.fwDyn.reset(0);
    this.fwPidf.reset();
    this.relocalize(pose);
  }

  _collReset() {
    if (this.wallCtx) resetContactCache(this.wallCtx);
    if (this._rigid) teleportRobot(this._rigid.body, this._prevPose, true);
  }

  /** Steps for one display frame: {steps, h}. Fixed: FixedClock decides (0..maxStepsPerFrame); variable: 1 step of
   *  min(frameDt, 0.05 s). */
  frameSteps(frameDt) {
    if (!this.cfg.fixedStep) { this.tel.steps = 1; this.tel.h = Math.min(Math.max(frameDt || 0, 0), VAR_DT_MAX_S); return { steps: 1, h: this.tel.h }; }
    const r = this.clock.advance(frameDt);
    this.tel.steps = r.steps; this.tel.h = this.clock.h; this.tel.droppedS = r.droppedSeconds;
    return { steps: r.steps, h: this.clock.h };
  }
  /** Call once per executed fixed step (after simUpdate). */
  onStep() { if (this.cfg.fixedStep) this.clock.onStep(); }

  /** Render interpolation (timestep-determinism, Fiedler): call before each simUpdate with the pose about to be stepped. */
  beginStep(S) { this._renderPrev = { x: S.x, y: S.y, h: S.h }; }
  /** Interpolation factor for drawing: FixedClock.alpha in fixed mode with renderInterp on, else 1 (draw the current state). */
  renderAlpha() { return this.cfg.fixedStep && this.cfg.renderInterp ? this.clock.alpha : 1; }
  /** Pose to draw: lerp(previous step, current step, alpha). A teleport (> TELEPORT_IN) draws the current pose. */
  renderPose(S) {
    const cur = { x: S.x, y: S.y, h: S.h }, p = this._renderPrev, a = this.renderAlpha();
    if (!p || a >= 1 || Math.hypot(cur.x - p.x, cur.y - p.y) > TELEPORT_IN) return cur;
    return lerpPose(p, cur, a);
  }

  /** battery-sag tick (any motion source). driveCmd = stick fractions, bodyV = body velocity [in/s, in/s, rad/s]. */
  _elecStep(dt, driveCmd, bodyV, intakeOn) {
    if (!this.cfg.batterySag) return null;
    const k = this.kin;
    const snap = this.elec.step(dt, {
      driveCmd: { fwd: driveCmd.fwd || 0, strafe: driveCmd.strafe || 0, turn: driveCmd.turn || 0 },
      driveBodyV: bodyV, driveNominal: { maxSpeed: k.maxSpeed, maxAccel: k.maxAccel, maxDecel: k.maxDecel, maxTurn: k.maxTurn },
      flywheelRpm: this._fwTarget, intakeDuty: intakeOn ? 1 : 0,   // [assumed] intake roller runs at full power when on
    });
    for (const e of snap.events) this._events.push(e);
    this._feedFlywheelBus();
    return snap;
  }

  /** Electrical tick when the robot is moved by something other than the driver (auto, robot link, path playback).
   *  The drive duty is estimated from the actual body velocity: duty = wheel surface speed / free surface speed, i.e. the
   *  steady-state duty that sustains that speed [assumed: acceleration current is not modelled in this mode]. */
  passiveStep(dt, S, { intakeOn = false } = {}) {
    if (!this.cfg.batterySag || !(dt > 0)) return null;
    const c = Math.cos(S.h), s = Math.sin(S.h), vx = S.vx || 0, vy = S.vy || 0, om = S.om || 0;
    const bv = [c * vx + s * vy, -s * vx + c * vy, om];
    const d = this.elec.driveParams;
    const vFree = (MOTORS.gobilda5203.freeSpeedRpm / d.gearRatio) * 2 * Math.PI * d.wheelRadiusIn / 60;   // in/s
    const kTurn = d.halfTrackIn + d.halfBaseIn;                                                           // in
    const cmd = vFree > 0 ? { fwd: clamp(bv[0] / vFree, -1, 1), strafe: clamp(bv[1] / vFree, -1, 1), turn: clamp(om * kTurn / vFree, -1, 1) } : {};
    return this._elecStep(dt, cmd, bv, intakeOn);
  }

  /** Battery-sag tick + drive step. Mutates S.bodyV. Returns world velocity {vx, vy, om} (in/s, rad/s).
   *  intakeOn: boolean, opts: MecanumDrive.step options. */
  driveStep(cmd, dt, S, opts, { intakeOn = false } = {}) {
    const k = this.kin;
    const base = { maxSpeed: k.maxSpeed, maxAccel: k.maxAccel, maxDecel: k.maxDecel, maxTurn: k.maxTurn };
    const prevBody = S.bodyV ? S.bodyV.slice() : [0, 0, 0];
    let c = cmd;
    const snap = this._elecStep(dt, cmd, prevBody, intakeOn);
    if (snap && !snap.hub.outputsEnabled) c = { ...cmd, fwd: 0, strafe: 0, turn: 0 };
    const lim = snap ? snap.driveLimits : null;
    const vBus = snap ? snap.vBus : NOMINAL_V;
    let v;
    const model = this.cfg.driveModel;
    if (model === 'motor') {
      v = this.motorDrive.step(c, dt, S, opts);
    } else if (model === 'dynamics') {
      const [F, L, T] = stickToBody(c, S.h, opts);
      this.dynDrive.params.batteryV = Math.max(vBus, MIN_PLANT_V);   // mecanum-dynamics motor.js reads p.batteryV per substep
      const r = this.dynDrive.step(S, { fwd: F, strafe: L, turn: T }, dt);
      v = { vx: r.vx, vy: r.vy, om: r.om };
    } else if (model === 'friction') {
      this.fricDrive.cfg.motor.voltageScale = vBus / NOMINAL_V;        // wheel-friction motor.js: torque x voltageScale
      v = this.fricDrive.step(c, dt, S, opts);
    } else {
      if (lim) { k.maxSpeed = lim.maxSpeed; k.maxAccel = lim.maxAccel; k.maxDecel = lim.maxDecel; k.maxTurn = lim.maxTurn; }
      try { v = k.step(c, dt, S, opts); } finally { Object.assign(k, base); }
    }
    if (this.cfg.weightTransfer) {
      if (this.wt.snapshot.disabled) {   // tipped over: the wheels are off the floor
        S.bodyV[0] = S.bodyV[1] = S.bodyV[2] = 0;
        return { vx: 0, vy: 0, om: 0 };
      }
      const r = this.wt.limitDrive(prevBody, S.bodyV, dt, S.h);
      S.bodyV[0] = r.bodyV[0]; S.bodyV[1] = r.bodyV[1]; S.bodyV[2] = r.bodyV[2];
      v = { vx: r.vx, vy: r.vy, om: r.om };
    }
    return v;
  }

  /** Move the robot by S.vx/vy/om over dt against the field (selected robotCollision model). Writes S.x/y/h/vx/vy/om and
   *  S.bodyV. Returns the collision record for afterMotion ({hit, normals, vBefore}) or null. No colliders: free motion. */
  moveRobot(S, dt) {
    const v0 = { vx: S.vx, vy: S.vy };
    // collision.vBefore = the velocity the contact REMOVED (v before - v after). weight-transfer onCollision and the gyro
    // shock read its into-wall component as the impact speed. For legacy (slide: after-contact normal speed 0) this equals
    // the pre-contact velocity, as in round 1; for the solvers it counts each impulse once (a speculative contact that is
    // resolved over two steps does not count the full approach speed twice).
    const removed = () => ({ vx: v0.vx - S.vx, vy: v0.vy - S.vy });
    const mode = this.cfg.robotCollision;
    if (!this.obstacles || !(dt > 0)) {
      S.x += S.vx * dt; S.y += S.vy * dt; S.h = wrapPi(S.h + S.om * dt);
      return null;
    }
    if (mode === 'contact' && this.wallCtx) {
      const r = stepRobotContact(S, dt, this.wallCtx);
      S.h = wrapPi(S.h);
      return r.hit ? { hit: true, normals: r.normals, vBefore: removed(), fallback: r.fallback, penetration: r.maxPenetration } : null;
    }
    if (mode === 'rigid' && this._rigidWorld()) {
      const { world, body } = this._rigid;
      teleportRobot(body, { x: S.x, y: S.y, h: S.h }, false);
      body.linearVelocity.x = S.vx * RB_IN_TO_M; body.linearVelocity.y = S.vy * RB_IN_TO_M; body.angularVelocity = S.om;
      stepWorld(world, dt);
      const r = readRobot(world, body);
      if (![r.x, r.y, r.h, r.vx, r.vy, r.om].every(Number.isFinite)) {   // never let a solver NaN into the app state
        teleportRobot(body, { x: S.x, y: S.y, h: S.h }, true);
        S.vx = S.vy = S.om = 0; S.bodyV = [0, 0, 0];
        return { hit: true, normals: [], vBefore: v0 };
      }
      S.x = r.x; S.y = r.y; S.h = r.h; S.vx = r.vx; S.vy = r.vy; S.om = r.om;
      const c = Math.cos(S.h), s = Math.sin(S.h);
      S.bodyV = [c * S.vx + s * S.vy, -s * S.vx + c * S.vy, S.om];
      if (r.hit) return { hit: true, normals: r.normals, vBefore: removed() };
      // A contact that is solved and released inside one world step (bounce) is gone from contactsOf() afterwards; detect
      // it from the velocity the step removed. RIGID_HIT_DV_INS: [assumed] 1 in/s, 10x the solver's own per-step
      // linear damping loss (0.1 1/s x 124 in/s x 1/120 s = 0.1 in/s), so damping alone never reads as a hit.
      const rv = removed(), m = Math.hypot(rv.vx, rv.vy);
      return m > RIGID_HIT_DV_INS ? { hit: true, normals: [[-rv.vx / m, -rv.vy / m]], vBefore: rv } : null;
    }
    // legacy: kinematic pose correction + velocity slide (the pre-round-2 app.js driveStep)
    const prev = { x: S.x, y: S.y, h: S.h };
    const want = { x: S.x + S.vx * dt, y: S.y + S.vy * dt, h: wrapPi(S.h + S.om * dt) };
    const r = resolveRobot(want, prev, this.obstacles);
    S.x = r.x; S.y = r.y; S.h = r.h;
    if (!r.hit) return null;
    const sv = slideVelocity(S.vx, S.vy, r.normals);
    S.vx = sv.vx; S.vy = sv.vy;
    const c = Math.cos(S.h), s = Math.sin(S.h);                         // back into the body frame so the robot does not "remember" pushing into the wall
    if (!S.bodyV) S.bodyV = [0, 0, 0];
    S.bodyV[0] = c * S.vx + s * S.vy; S.bodyV[1] = -s * S.vx + c * S.vy;
    return { hit: true, normals: r.normals, vBefore: v0 };
  }

  /** Mechanism state for the weight-transfer CG (turret angle deg, balls held, intake on). Cheap when unchanged. */
  setMech(turretDeg, ballsHeld, intakeOn) {
    const key = Math.round(turretDeg) + '|' + ballsHeld + '|' + (intakeOn ? 1 : 0);
    if (key === this._mechKey) return;
    this._mechKey = key;
    this.wt.setMechState({ turretRad: turretDeg * Math.PI / 180, ballsHeld, intakeDeployed: intakeOn ? 1 : 0 });
  }

  /** After the robot moved this step (any source: driver, auto, link, path playback). S has the new pose and
   *  velocity; collision = {hit, normals, vBefore:{vx,vy}} from moveRobot, or null. */
  afterMotion(S, dt, collision = null) {
    const p0 = this._prevPose, p1 = { x: S.x, y: S.y, h: S.h };
    const jump = Math.hypot(p1.x - p0.x, p1.y - p0.y) > TELEPORT_IN;
    this._prevPose = p1;
    if (!(dt > 0)) return;
    if (this.cfg.weightTransfer) {
      if (collision && collision.hit) {
        const r = this.wt.onCollision({ normals: collision.normals || [], vBefore: collision.vBefore, h: S.h });
        if (r.tipStarted) this._events.push('tipStarted');
      }
      this.wt.step(dt, S.bodyV || [0, 0, 0]);
    }
    if (this.odo) {
      if (jump) { this.odo.reset(p1); return; }                 // a teleport is not wheel motion: relocalise
      const imu = this.odo.imu;
      if (imu instanceof GyroImuAdapter) {
        imu.flywheelOn = this._fwTarget > 0;
        if (collision && collision.hit && collision.vBefore) {
          let vn = 0;
          for (const n of collision.normals || []) vn = Math.max(vn, -(collision.vBefore.vx * n[0] + collision.vBefore.vy * n[1]));
          if (vn > 0) imu.shock(vn);
        }
      }
      // truth twist = actual displacement over the step (Pinpoint pods measure ground motion, pushes included)
      const vx = (p1.x - p0.x) / dt, vy = (p1.y - p0.y) / dt, om = wrapPi(p1.h - p0.h) / dt;
      this.odo.stepPhysics(dt, { x: p0.x, y: p0.y, h: p0.h, vx, vy, om }, { collision: collision && collision.hit ? 1 : 0 });
    }
  }

  /** Relocalise the believed pose to the truth (sim tools: place robot, reset). */
  relocalize(pose) {
    this._prevPose = { x: pose.x, y: pose.y, h: pose.h };
    if (this.odo) this.odo.reset(this._prevPose);
    this._renderPrev = null;
    this._collReset();
  }

  /** Pose + field velocity the robot code believes (for the aim). Falls back to the truth. */
  aimPose(S) {
    if (!this.odo || !this.cfg.aimFromOdometry) return { x: S.x, y: S.y, h: S.h, vx: S.vx, vy: S.vy, om: S.om };
    const p = this.odo.getPose(), v = this.odo.getVelocity();
    return { x: p.x, y: p.y, h: p.h, vx: v.x, vy: v.y, om: v.h };
  }

  /** battery-sag bus voltage -> both physical flywheel plants (the bus solve already includes the flywheel load, so the
   *  plants see it as an ideal source: internal resistance 0). Sag off: each plant's own battery defaults come back. */
  _feedFlywheelBus() {
    const e = this.cfg.batterySag ? this.elec.last : null;
    const b = this.fwDyn.params.battery, p0 = this._pidfBattery0;
    if (e) {
      const V = e.hub.outputsEnabled ? Math.max(e.vBus, MIN_PLANT_V) : MIN_PLANT_V;   // brownout: hub outputs off
      b.voltageV = V; b.internalResOhm = 0;
      this.fwPidf.battery.p = { ...p0, openCircuitV: V, internalOhm: 0, minV: Math.min(p0.minV, V) };
    } else {
      b.voltageV = this._fwBattery0.voltageV; b.internalResOhm = this._fwBattery0.internalResOhm;
      this.fwPidf.battery.p = p0;
    }
  }

  /** Shooter plant step (turret/hood always the aimcontrol plant; rpm from the selected flywheel model). */
  plantStep(dt, cmd) {
    const pl = this.plant, mode = this.cfg.flywheelModel;
    if (cmd && Number.isFinite(cmd.rpm)) this._fwTarget = Math.min(Math.max(cmd.rpm, 0), pl.spec.maxRpm);
    if (mode === 'legacy') return pl.step(dt, cmd);
    const { rpm, ...rest } = cmd || {};
    const ret = pl.step(dt, rest);
    if (mode === 'dynamics') {
      if (Number.isFinite(rpm)) this.fwDyn.setTargetRpm(this._fwTarget);
      this.fwDyn.step(dt);
      pl.state.rpm = this.fwDyn.state.rpm;
    } else {
      this.fwPidf.step(dt, Number.isFinite(rpm) ? { rpm: this._fwTarget } : {});
      pl.state.rpm = this.fwPidf.state.rpm;
    }
    return ret;
  }

  /** A ball left the flywheel: rpm dip from the selected model. kind 'pollen'|'nectar'. */
  kick(kind) {
    const pl = this.plant, mode = this.cfg.flywheelModel;
    if (this.cfg.batterySag) this.elec.kickFlywheel();
    if (mode === 'legacy') { pl.kick(); return null; }
    if (mode === 'dynamics') { const s = this.fwDyn.fire(kind); pl.state.rpm = this.fwDyn.state.rpm; return s; }
    const s = this.fwPidf.kick(kind); return s;
  }

  /** External rpm write (robot link): keep the physical flywheel in step with it. */
  syncRpm(rpm) { if (Number.isFinite(rpm)) { this.fwDyn.reset(rpm); } }

  /** Shot-speed noise factor (seeded, deterministic): 1 +- 1.5 %. [sourced] same +-1.5 % band as the old Math.random noise in app.js. */
  shotNoise() { return 1 + (this.rng.next() - 0.5) * 0.03; }

  /** pedro-fidelity params for an AutoRunner: {} = library/ESTIMATE defaults, else pedroParamsFromSpec(spec). */
  pedroParams() { return this.cfg.pedroPlant === 'spec' ? pedroParamsFromSpec(this.spec) : {}; }

  /** Apply autoFollower / pedroPlant to an AutoRunner (autoui.js builds it without options). The Pedro service reads
   *  pedroParams only when it is created (first follow of a run), so assigning them at any time is safe. */
  configureRunner(runner) {
    if (!runner) return;
    runner.followerModel = this.cfg.autoFollower;
    if (!runner._svc) runner.pedroParams = this.pedroParams();
  }

  /** Weight-transfer visual transform (Float64Array(16) column-major, robot frame) or null. */
  visualMatrix() { return this.cfg.weightTransfer ? this.wt.visualMatrix() : null; }

  /** Events since the last call ('hubReset', 'fuseBlown', 'lowBattery', 'tipStarted', ...). */
  consumeEvents() { const e = this._events; this._events = []; return e; }

  /** Telemetry for the HUD. */
  telemetry(S) {
    const t = { steps: this.tel.steps, h: this.tel.h, fixed: this.cfg.fixedStep };
    if (this.cfg.batterySag && this.elec.last) {
      const e = this.elec.last;
      t.battery = { vBus: e.vBus, soc: e.soc, iTotal: e.iTotal, hub: e.hub.mode, fuseBlown: e.fuseBlown, speedScale: e.driveLimits.speedScale };
    }
    const dm = this.cfg.driveModel;
    if (dm === 'motor') {
      const m = this.motorDrive.telemetry();
      t.drive = { model: 'motor', freeSpeedInS: this.motorDrive.freeSpeedInS(), statorA: m.wheelStatorA, tractionLimited: m.tractionLimited.some(Boolean), windingC: Math.max(...m.wheelWindingC) };
    } else if (dm === 'dynamics') {
      t.drive = { model: 'dynamics', freeSpeedInS: this.gearing ? this.gearing.freeSpeedInS : NaN, batteryV: this.dynDrive.params.batteryV };
    } else if (dm === 'friction') {
      const f = this.fricDrive.telemetry() || {};
      t.drive = { model: 'friction', freeSpeedInS: this.gearing ? this.gearing.freeSpeedInS : NaN, tractionLimited: Array.isArray(f.wheels) && f.wheels.some((w) => w.saturated) };
    } else t.drive = { model: 'kinematic' };
    t.collision = this.cfg.robotCollision;
    if (this.cfg.weightTransfer) {
      const w = this.wt.snapshot;
      t.weight = { scale: w.scale, pitchDeg: w.pitch * 180 / Math.PI, rollDeg: w.roll * 180 / Math.PI, tip: w.tip.mode, loadsLbf: w.loads.lbf };
    }
    if (this.odo && S) {
      const e = this.odo.getTruthError({ x: S.x, y: S.y, h: S.h });
      t.odometry = { preset: this.cfg.odometry, imu: this.cfg.imuModel, errIn: e.dist, errDeg: e.dh * 180 / Math.PI, aim: this.cfg.aimFromOdometry };
    }
    const mode = this.cfg.flywheelModel;
    if (mode === 'dynamics') t.flywheel = { model: mode, rpm: this.fwDyn.state.rpm, targetRpm: this.fwDyn.state.targetRpm, currentA: this.fwDyn.state.currentA, busV: this.fwDyn.state.busV };
    else if (mode === 'pidf') { const s = this.fwPidf.state; t.flywheel = { model: mode, rpm: s.rpm, targetRpm: s.targetRpm, currentA: s.currentA, ready: s.ready, busV: this.fwPidf.battery.voltage }; }
    else t.flywheel = { model: mode, rpm: this.plant ? this.plant.state.rpm : 0 };
    t.balls = { aero: BALL_MODELS.aero, ballBall: BALL_MODELS.ballBall };
    return t;
  }
}

/** One-line HUD text of telemetry(). */
export function formatPhysicsTelemetry(t) {
  const f = (v, d = 1) => (Number.isFinite(v) ? v.toFixed(d) : '-');
  const out = [];
  out.push(t.fixed ? `step ${f(t.h * 1000, 2)} ms x${t.steps}` : `variable step ${f(t.h * 1000, 1)} ms`);
  if (t.battery) out.push(`battery ${f(t.battery.vBus, 2)} V, ${f(t.battery.iTotal, 0)} A, SoC ${f(t.battery.soc * 100, 0)}%${t.battery.hub !== 'ok' ? ', hub ' + t.battery.hub : ''}${t.battery.fuseBlown ? ', FUSE BLOWN' : ''}`);
  if (t.drive) out.push(t.drive.model === 'kinematic' ? 'kinematic drive' : `${t.drive.model} drive, free ${f(t.drive.freeSpeedInS, 0)} in/s${t.drive.tractionLimited ? ', wheel slip' : ''}`);
  if (t.collision) out.push(`robot collision: ${t.collision}`);
  if (t.weight) out.push(`traction x${f(t.weight.scale, 2)}, pitch ${f(t.weight.pitchDeg, 2)} deg, roll ${f(t.weight.rollDeg, 2)} deg${t.weight.tip !== 'grounded' ? ', ' + t.weight.tip.toUpperCase() : ''}`);
  if (t.odometry) out.push(`Pinpoint (${t.odometry.preset}, ${t.odometry.imu} IMU) error ${f(t.odometry.errIn, 2)} in, ${f(t.odometry.errDeg, 2)} deg${t.odometry.aim ? ', aim uses it' : ''}`);
  if (t.flywheel) out.push(`flywheel (${t.flywheel.model}) ${f(t.flywheel.rpm, 0)} rpm${Number.isFinite(t.flywheel.currentA) ? ', ' + f(t.flywheel.currentA, 1) + ' A' : ''}${Number.isFinite(t.flywheel.busV) ? ', ' + f(t.flywheel.busV, 2) + ' V' : ''}`);
  if (t.balls) out.push(`balls: ${t.balls.aero} aero, ${t.balls.ballBall} ball-ball`);
  return out;
}
