// pinpoint.js - goBILDA Pinpoint firmware emulation (spec section 3).
// Units inside the device: millimetres, radians, seconds, mm/s (as in GoBildaPinpointDriver).
// No DOM; imports only from this folder.
//
// Provenance: the firmware cycle (read two pod counters + IMU yaw, dead-reckon with pose exponential,
// differentiate over a short window for velocity) follows the spec; the velocity window length, history length
// and reset duration come from params.firmware and are [ASSUMPTION] values defined in params.js.
import { pinpointBodyDelta, poseExp, wrapAngle } from './kinematics.js';

const MM_PER_IN = 25.4; // [SRC definition of the inch]

export class PinpointDevice {
  constructor(params, { fwdPod, strPod, imu }) {
    this._p = params;
    this._fwd = fwdPod;
    this._str = strPod;
    this._imu = imu;

    // Config (mm). xOffsetMm = forward-pod sideways position; yOffsetMm = strafe-pod forward position.
    this._xOffMm = params.config.forwardPodYIn * MM_PER_IN;
    this._yOffMm = params.config.strafePodXIn * MM_PER_IN;
    this._ticksPerMm = params.config.ticksPerMm;
    this._dirX = params.config.forwardDir;
    this._dirY = params.config.strafeDir;
    this._yawScalar = params.config.yawScalar;

    this._quantize = !!(params.counter && params.counter.quantize);
    this._velN = Math.max(1, params.firmware.velWindowSamples | 0);
    this._histCap = Math.max(1, params.firmware.historyLen | 0);
    this._resetDurS = params.firmware.resetDurationS;

    this._pose = { x: 0, y: 0, h: 0 }; // mm, mm, rad wrapped
    this._hUnw = 0; // unwrapped heading (rad)
    this._ring = []; // {t,x,y,h}
    this._hist = [];
    this._status = 'READY';
    this._readyAtS = 0;
    this._lastTickS = null;
    this._vx = 0; this._vy = 0; this._w = 0;
    this._loopUs = 0; this._freq = 0;
    this._tS = 0;

    this._rebase();
    this._last = this._build();
  }

  _rebase() {
    this._lastX = this._fwd.count;
    this._lastY = this._str.count;
    this._lastImu = this._imu.headingRad;
  }

  _delta(cur, last) {
    return this._quantize ? ((cur - last) | 0) : (cur - last);
  }

  _build() {
    return {
      tS: this._tS,
      xMm: this._pose.x,
      yMm: this._pose.y,
      hRad: this._pose.h,
      hUnwrappedRad: this._hUnw,
      vxMm: this._vx,
      vyMm: this._vy,
      wRadS: this._w,
      encX: this._fwd.count,
      encY: this._str.count,
      loopTimeUs: this._loopUs,
      frequencyHz: this._freq,
      status: this._status,
    };
  }

  _store() {
    this._last = this._build();
    this._hist.push(this._last);
    if (this._hist.length > this._histCap) this._hist.shift();
  }

  setOffsets(xOffsetMm, yOffsetMm) { this._xOffMm = xOffsetMm; this._yOffMm = yOffsetMm; }
  setEncoderResolution(ticksPerMm) { this._ticksPerMm = ticksPerMm; }
  setEncoderDirections(fwdSign, strSign) { this._dirX = fwdSign; this._dirY = strSign; }
  setYawScalar(s) { this._yawScalar = s; }

  setPosition({ xMm, yMm, hRad }) {
    this._pose = { x: xMm, y: yMm, h: wrapAngle(hRad) };
    this._hUnw = hRad;
    this._ring.length = 0;
    this._vx = 0; this._vy = 0; this._w = 0;
    this._rebase();
    this._last = this._build();
  }

  resetPosAndIMU(nowS) {
    this._pose = { x: 0, y: 0, h: 0 };
    this._hUnw = 0;
    this._ring.length = 0;
    this._vx = 0; this._vy = 0; this._w = 0;
    if (typeof this._imu.reset === 'function') this._imu.reset(0);
    this._rebase();
    this._status = 'NOT_READY';
    this._readyAtS = nowS + this._resetDurS;
    this._tS = nowS;
    this._last = this._build();
  }

  tick(nowS) {
    const dtTick = this._lastTickS === null ? 0 : nowS - this._lastTickS;
    this._tS = nowS;
    if (this._status === 'NOT_READY') {
      if (nowS < this._readyAtS) {
        this._rebase(); // motion during reset is discarded
        this._lastTickS = nowS;
        this._store();
        return;
      }
      this._status = 'READY';
    }

    const cX = this._fwd.count, cY = this._str.count;
    const dX = this._dirX * this._delta(cX, this._lastX);
    const dY = this._dirY * this._delta(cY, this._lastY);
    this._lastX = cX; this._lastY = cY;
    const dF = dX / this._ticksPerMm;
    const dS = dY / this._ticksPerMm;

    const hImu = this._imu.headingRad;
    const dth = (hImu - this._lastImu) * this._yawScalar;
    this._lastImu = hImu;

    const { dxB, dyB } = pinpointBodyDelta(dF, dS, dth, this._xOffMm, this._yOffMm);
    this._pose = poseExp(this._pose, dxB, dyB, dth);
    this._hUnw += dth;

    const ring = this._ring;
    ring.push({ t: nowS, x: this._pose.x, y: this._pose.y, h: this._hUnw });
    while (ring.length > this._velN + 1) ring.shift();
    if (ring.length >= 2) {
      const a = ring[0], b = ring[ring.length - 1];
      const dt = b.t - a.t;
      if (dt > 0) {
        this._vx = (b.x - a.x) / dt;
        this._vy = (b.y - a.y) / dt;
        this._w = (b.h - a.h) / dt;
      } else { this._vx = 0; this._vy = 0; this._w = 0; }
    } else { this._vx = 0; this._vy = 0; this._w = 0; }

    this._loopUs = dtTick * 1e6;
    this._freq = dtTick > 0 ? 1 / dtTick : 0;
    this._lastTickS = nowS;
    this._store();
  }

  snapshot() { return this._last; }
  history() { return this._hist.slice(); }
  clearHistory() { this._hist.length = 0; }
  get status() { return this._status; }
}
