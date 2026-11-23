// drive_adapter.js - mecanum duty mixing, wheel speeds and voltage-scaled drive limits.
// Sim boundary units are inches (in, in/s, in/s^2); electrical math is SI (1 in = 0.0254 m).
import { DRIVE_ELEC, MOTOR_5203 } from './params.js';
import { deriveMotor } from './motor.js';

const IN_TO_M = 0.0254; // m per in (exact)

// Command: MecanumDrive convention (fwd, strafe = +left, turn = +ccw). Returns duties [fl,fr,bl,br].
export function mecanumDuties(cmd) {
  const f = cmd?.fwd ?? 0, s = cmd?.strafe ?? 0, t = cmd?.turn ?? 0;
  const d = [f - s - t, f + s + t, f + s - t, f - s + t];
  const m = Math.max(1, ...d.map(Math.abs));
  return d.map(x => x / m);
}

// bodyV = [vf, vl, w] in in/s, in/s, rad/s. Returns wheel linear speeds [fl,fr,bl,br] in in/s.
export function wheelLinearSpeeds(bodyV, p = DRIVE_ELEC) {
  const [vf, vl, w] = bodyV;
  const k = p.halfTrackIn + p.halfBaseIn; // in
  return [vf - vl - k * w, vf + vl + k * w, vf + vl - k * w, vf - vl + k * w];
}

// Motor shaft speeds (rad/s) for each wheel.
export function wheelMotorOmegas(bodyV, p = DRIVE_ELEC) {
  return wheelLinearSpeeds(bodyV, p).map(l => l / p.wheelRadiusIn * p.gearRatio);
}

export function driveLimits(vBus, bodyV, nominal, outputsEnabled = true, p = DRIVE_ELEC, spec = MOTOR_5203) {
  const m = deriveMotor(spec);
  const ratio = vBus / p.nominalV;
  const speedScale = p.compensation === 'nominal12' ? Math.min(1, ratio) : ratio;
  let maxSpeed = nominal.maxSpeed * speedScale;
  let maxTurn = nominal.maxTurn * speedScale;

  const vf = Math.abs(bodyV?.[0] ?? 0); // in/s
  const wm = vf / p.wheelRadiusIn * p.gearRatio;               // rad/s motor
  const iA = Math.max(0, (vBus - m.kE * wm) / m.rOhm);         // A per motor
  const forceN = 4 * m.kT * iA * p.gearRatio * p.gearEff / (p.wheelRadiusIn * IN_TO_M); // N
  const aIn = forceN / p.massKg / IN_TO_M;                      // in/s^2
  const maxAccel = Math.min(nominal.maxAccel, aIn);
  const maxDecel = nominal.maxDecel ?? nominal.maxAccel * 1.6;

  if (!outputsEnabled) { maxSpeed = 0; maxTurn = 0; }
  return { maxSpeed, maxAccel, maxDecel, maxTurn, speedScale };
}
