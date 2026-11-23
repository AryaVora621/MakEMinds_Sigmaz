// system.js - ElectricalSystem: battery + bus + fuse + hub + motors, one step per sim tick (spec section 10 order).
import { FUSE, DRIVE_ELEC, FLYWHEEL, INTAKE, MOTOR_5203 } from './params.js';
import { Battery } from './battery.js';
import { FlywheelSim } from './flywheel.js';
import { DcMotorSim, motorBusLoad, deriveMotor } from './motor.js';
import { solveBus } from './bus.js';
import { Fuse } from './fuse.js';
import { HubBrownout, hubLoads, portOvercurrent } from './hub.js';
import { VoltageSensor } from './sensor.js';
import { mecanumDuties, driveLimits, wheelMotorOmegas } from './drive_adapter.js';

const DEFAULT_NOMINAL = { maxSpeed: 60, maxAccel: 60, maxTurn: 6 }; // [A] in/s, in/s^2, rad/s fallback

export class ElectricalSystem {
  constructor(opts = {}) {
    const { preset = 'fresh', fuseEnabled = FUSE.enabled,
      driveCompensation = DRIVE_ELEC.compensation, flywheelCompensation = FLYWHEEL.compensation } = opts;
    this.driveParams = { ...DRIVE_ELEC, compensation: driveCompensation };
    this.motor = deriveMotor(MOTOR_5203);
    this.battery = new Battery();
    this.fuse = new Fuse({ ...FUSE, enabled: fuseEnabled });
    this.hub = new HubBrownout();
    this.sensor = new VoltageSensor();
    this.flywheel = new FlywheelSim({ params: { ...FLYWHEEL, compensation: flywheelCompensation } });
    this.intake = new DcMotorSim();
    this.t = 0;
    this.last = null;
    this.setPreset(preset);
  }

  setPreset(name) { this.battery.applyPreset(name); }

  reset(preset = 'fresh') {
    this.setPreset(preset);
    this.fuse.reset(); this.hub.reset(); this.sensor.reset(); this.flywheel.reset();
    this.intake = new DcMotorSim();
    this.t = 0; this.last = null;
  }

  kickFlywheel(frac) { this.flywheel.kick(frac); }

  step(dt, inputs = {}) {
    const {
      driveCmd = {}, driveBodyV = [0, 0, 0], driveNominal = DEFAULT_NOMINAL,
      flywheelRpm = 0, intakeDuty = 0, servosMoving = 0, servosIdle = 0,
    } = inputs;
    // 1. outputs enabled from the previous tick
    const en = this.hub.state.outputsEnabled;
    const duties = en ? mecanumDuties(driveCmd) : [0, 0, 0, 0];
    const omegas = wheelMotorOmegas(driveBodyV, this.driveParams);
    // 2. loads: 4 drive, flywheel, intake, hub electronics, hub servos
    const m = this.motor;
    const loads = [];
    for (let i = 0; i < 4; i++) loads.push(motorBusLoad(duties[i], omegas[i], m, 1));
    loads.push(this.flywheel.busLoad()); // duty/omega from its last substep (documented one-tick lag)
    loads.push(motorBusLoad(en ? intakeDuty : 0, this.intake.state.omega, this.intake.m, INTAKE.nMotors));
    loads.push(...hubLoads({ servosMoving, servosIdle }));
    // 3. bus solve (blown fuse = open circuit)
    let V, cur, iTotal;
    if (this.fuse.state.blown) {
      V = 0; cur = loads.map(() => 0); iTotal = 0;
    } else {
      const sol = solveBus(this.battery.vth(), this.battery.rs(), loads);
      V = sol.v; cur = sol.currents; iTotal = sol.iTotal;
    }
    // 4. battery + fuse
    const wasBlown = this.fuse.state.blown;
    this.battery.step(iTotal, dt);
    this.fuse.step(iTotal, dt);
    // 5. hub
    this.hub.step(V, dt);
    const events = this.hub.consumeEvents().slice();
    if (!wasBlown && this.fuse.state.blown) events.push('fuseBlown');
    // 6. flywheel + intake
    this.flywheel.setTargetRpm(flywheelRpm);
    this.flywheel.step(dt, V, { enabled: en });
    this.intake.step(en ? intakeDuty : 0, V, dt);
    // 7. drive limits
    const hs = this.hub.state;
    const lim = driveLimits(V, driveBodyV, driveNominal, hs.outputsEnabled, this.driveParams);
    // 8. sensor
    const vSensor = this.sensor.sample(V, dt);
    this.t += dt;
    const iDrive = cur.slice(0, 4);
    this.last = {
      t: this.t, vBus: V, vSensor, vOc: this.battery.ocv(), iTotal,
      iDrive, iFlywheel: cur[4], iIntake: cur[5], iElectronics: cur[6] + cur[7],
      soc: this.battery.state.soc, ahUsed: this.battery.state.ahUsed,
      fuseBlown: this.fuse.state.blown, fuseDamage: this.fuse.state.damage,
      hub: { mode: hs.mode, outputsEnabled: hs.outputsEnabled, lowBattery: hs.lowBattery,
        ledFault: hs.ledFault, servoRailOk: hs.servoRailOk, resetCount: hs.resetCount },
      portOvercurrent: portOvercurrent(iDrive),
      driveLimits: { maxSpeed: lim.maxSpeed, maxAccel: lim.maxAccel, maxDecel: lim.maxDecel,
        maxTurn: lim.maxTurn, speedScale: lim.speedScale },
      flywheelRpm: this.flywheel.state.rpm, flywheelAtSpeed: this.flywheel.atSpeed(),
      events,
    };
    return this.last;
  }
}
