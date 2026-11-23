// flywheel_system.js - closed-loop flywheel system (spec sections 6 and 7).
// Units: seconds, rad, rpm, ticks/s (motor-shaft encoder counts per second), volts, amps, joules, kg, m.
// Modes: 'sdk' = OpMode-side FlywheelPIDF loop (matches the repo Flywheel.java, RUN_WITHOUT_ENCODER),
//        'hub' = ASSUMED hub-side RUN_USING_ENCODER PIDF ([UNVERIFIED], see params.js PARAM_SOURCES).
import { BALL_KINDS } from '../../balls.js';
import { mergeParams, rpmToTicksPerS, ticksPerSToRpm } from './params.js';
import { FlywheelPlant, Battery } from './dcmotor.js';
import { QuadratureEncoder } from './encoder.js';
import { mulberry32, LoopClock, CommandQueue, quantizePower } from './timing.js';
import { SdkFlywheelPIDF, HubVelocityPIDF, ReadyGate, clampTargetTicks } from './controllers.js';

const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));

export class FlywheelSystem {
  constructor(overrides = {}) {
    this.params = mergeParams(overrides);
    this._build();
  }

  _build() {
    const p = this.params;
    this.plant = new FlywheelPlant(p);
    this.battery = new Battery(p.battery);
    const eo = { cpr: p.motor.cpr, velWindowS: p.encoder.velWindowS,
      historyS: p.encoder.historyS, int16Wrap: p.encoder.int16Wrap, direction: p.encoder.direction };
    this.encA = new QuadratureEncoder(eo);
    this.encB = new QuadratureEncoder(eo);
    this.rng = mulberry32(p.timing.seed);
    this.clock = new LoopClock(p.timing, this.rng);
    this.queue = new CommandQueue();
    this.ctrl = new SdkFlywheelPIDF(p.sdkPidf);
    this.hub = new HubVelocityPIDF(p.hubPidf);
    this.gate = new ReadyGate(p.ready);
    this.mode = p.mode === 'hub' ? 'hub' : 'sdk';
    this.time = 0;
    this.pendingRpm = 0;
    this.nextLoopT = 0;
    this.lastLoopT = 0;
    this.nextHubT = 0;
    this.motorPower = [0, 0];
    this._s = { measuredTicksPerS: 0, targetTicksPerS: 0, powerCmd: 0, loopDtS: 0, loopCount: 0,
      appliedVolts: [0, 0], currentA: 0 };
    this.encA.reset(0, 0);
    this.encB.reset(0, 0);
  }

  reset() {
    this._build();
  }

  setMode(mode) {
    if (mode !== 'sdk' && mode !== 'hub') return;
    if (mode === this.mode) return;
    this.mode = mode;
    this.params.mode = mode;
    this.queue.clear();
    this.ctrl.reset();
    this.hub.reset();
    this.motorPower = [0, 0];
    this.nextHubT = this.time;
  }

  ready() { return this.gate.ready; }

  step(frameDtS, cmd = {}) {
    const p = this.params, tpr = p.settings.ticksPerRev, T = p.timing;
    let dt = Number.isFinite(frameDtS) ? frameDtS : 0;
    dt = clamp(dt, 0, T.maxFrameDtS);
    if (cmd && Number.isFinite(cmd.rpm)) this.pendingRpm = cmd.rpm;
    const tEnd = this.time + dt;
    const s = this._s;
    while (this.time < tEnd - 1e-12) {
      const h = Math.min(T.physicsDtS, tEnd - this.time);
      const t = this.time;
      // (1) due commands
      for (const c of this.queue.popDue(t)) {
        if (c.kind === 'power') this.motorPower[c.port] = c.value;
        else if (c.kind === 'vel') this.hub.setTarget(c.value);
      }
      // (2) hub-side PIDF tick
      if (this.mode === 'hub' && t >= this.nextHubT) {
        const pw = this.hub.tick(this.encA.velocityAt(t), p.hubPidf.periodS);
        this.motorPower[0] = pw; this.motorPower[1] = pw;
        this.nextHubT += p.hubPidf.periodS;
      }
      // (3) OpMode loop
      if (t >= this.nextLoopT) {
        const dtCtrl = Math.max(t - this.lastLoopT, p.settings.minDtS);
        this.lastLoopT = t;
        const velA = this.encA.velocityAt(t), velB = this.encB.velocityAt(t);
        const batteryV = this.battery.voltage;
        const vel = p.settings.velocityFromBoth ? 0.5 * (velA + velB) : velA;
        const targetT = clampTargetTicks(rpmToTicksPerS(this.pendingRpm, tpr),
          p.settings.minRpm, p.settings.maxRpm, tpr);
        let pw = 0;
        if (this.mode === 'sdk') {
          if (targetT === 0) { this.ctrl.reset(); pw = 0; }
          else pw = this.ctrl.calculate(targetT, vel, batteryV, dtCtrl);
          this.queue.push(t + T.bulkReadS + T.cmdLatencyS, { port: 0, kind: 'power', value: pw });
          this.queue.push(t + T.bulkReadS + 2 * T.cmdLatencyS, { port: 1, kind: 'power', value: pw });
        } else {
          this.queue.push(t + T.bulkReadS + T.cmdLatencyS, { kind: 'vel', value: targetT });
        }
        this.gate.update(targetT, vel);
        s.measuredTicksPerS = vel; s.targetTicksPerS = targetT; s.powerCmd = pw;
        s.loopDtS = dtCtrl; s.loopCount++;
        this.nextLoopT = this.clock.next(t);
      }
      // (4) drive volts
      const drives = [0, 1].map((i) => ({
        volts: quantizePower(this.motorPower[i], T.powerLevels) * this.battery.voltage,
        coast: this.motorPower[i] === 0 && p.settings.zeroPowerFloat,
      }));
      // (5) plant, (6) battery
      const itot = this.plant.step(h, drives);
      this.battery.update(itot);
      s.appliedVolts = [drives[0].volts, drives[1].volts];
      s.currentA = itot;
      // (7) advance time, encoders
      this.time += h;
      this.encA.update(this.time, this.plant.state.thetaMotorRad);
      this.encB.update(this.time, this.plant.state.thetaMotorRad);
    }
    return this.state;
  }

  kick(kind = 'pollen', exitSpeedMps = null) {
    const f = this.params.flywheel;
    const m = BALL_KINDS[kind]?.massKg ?? f.fallbackBallMassKg;
    const w = this.plant.state.omegaWheelRadS;
    const v = Number.isFinite(exitSpeedMps) ? exitSpeedMps : w * f.wheelRadiusM * f.exitSpeedFrac;
    const E = 0.5 * m * v * v * (1 + f.ballSpinFactor) / f.energyTransferEff;
    this.plant.addBallLoad(E, f.ballContactS);
    const wAfter = Math.sqrt(Math.max(0, w * w - 2 * E / f.inertiaKgM2));
    return { energyJ: E, expectedDropRpm: this.plant.rpm - (60 / (2 * Math.PI)) * wAfter };
  }

  get state() {
    const s = this._s, tpr = this.params.settings.ticksPerRev;
    return {
      rpm: this.plant.rpm,
      measuredRpm: ticksPerSToRpm(s.measuredTicksPerS, tpr),
      measuredTicksPerS: s.measuredTicksPerS,
      targetRpm: ticksPerSToRpm(s.targetTicksPerS, tpr),
      targetTicksPerS: s.targetTicksPerS,
      powerCmd: s.powerCmd,
      appliedVolts: [s.appliedVolts[0], s.appliedVolts[1]],
      currentA: s.currentA,
      batteryV: this.battery.voltage,
      ready: this.gate.ready,
      loopDtS: s.loopDtS,
      loopCount: s.loopCount,
      time: this.time,
    };
  }
}

// Wraps an aimcontrol ShooterPlant: turret/hood stay with the base plant, the flywheel is replaced.
export function createShooterPlantAdapter(basePlant, fw) {
  return {
    flywheel: fw,
    step(dt, cmd = {}) {
      const res = basePlant.step(dt, { ...cmd, rpm: undefined });
      fw.step(dt, cmd);
      basePlant.state.rpm = fw.state.rpm;
      return res;
    },
    kick(kind, exitSpeedMps) { return fw.kick(kind, exitSpeedMps); },
    atSetpoint(cmd, tol) {
      const r = basePlant.atSetpoint(cmd, tol);
      return { turret: r.turret, hood: r.hood, rpm: fw.ready() };
    },
    get state() { return { ...basePlant.state, measuredRpm: fw.state.measuredRpm }; },
    get spec() { return basePlant.spec; },
    get atLimit() { return basePlant.atLimit; },
  };
}
