package commands.subsystems.turret;

import commands.field.Alliance;
import commands.field.FieldConstants;
import commands.subsystems.grouped_subsystems.shooter.ShotSolver;
import commands.subsystems.grouped_subsystems.shooter.ShotTable;
import commands.subsystems.flywheel.Flywheel;
import robot.RobotSettings;

/**
 * Aiming math for the whole shooter (no hardware). This is the PHYSICS layer: it turns the robot's pose and
 * motion into SETPOINTS (turret angle, flywheel RPM) for a FIXED launch elevation (the robot has no hood).
 * The control layer (Flywheel's PIDF and the turret's positional servos) then makes the hardware follow
 * those setpoints.
 *
 * Units: inches, seconds, heading in radians (Pedro frame: x right, y up, heading CCW-positive, 0 along +x).
 * Turret angle in degrees, 0 = straight ahead, CCW positive.
 *
 * Pipeline (RobotSettings.TurretAiming.SHOT_SOURCE = PHYSICS):
 *  1. Latency compensation: pose += velocity * L, heading += omega * L.
 *  2. {@link ShotSolver}: exact ballistic solution to the raised cell's opening, including the robot's own
 *     velocity and spin at the turret pivot, so shooting while driving needs no iterative lead guess.
 *  3. Launch speed -> flywheel RPM (then calibrate with RobotSettings.TurretAiming.RPM_SCALE on the robot).
 *  4. Optional camera tx trim on the turret.
 *  5. Pick the equivalent turret angle (phi + k*360) inside the turret limits (TurretLimits, the servo
 *     window), closest to the current angle; outside the window the turret parks at the nearest limit.
 *
 * RobotSettings.TurretAiming.SHOT_SOURCE = TABLE skips the physics and uses the measured distance table in {@link ShotTable} instead
 * (use it to compare, or when you have measured shots you trust more than the model).
 */
public class TurretAiming {
    public enum ShotSource { PHYSICS, TABLE }

    /** Result of one aiming solve. */
    public static class Solution {
        public boolean ok;              // false if the physics found no shot (see reason)
        public String reason = "";
        public double turretAngleDeg;   // command for the turret (0 if there is no turret)
        public double aimHeadingRad;    // field direction of the shot; a turretless robot must face this
        public double distanceIn;       // pivot-to-target horizontal distance
        public double flightTimeS;
        public double launchSpeedInS;
        public double rpm;              // flywheel RPM for this shot
        public double launchElevDeg;    // launch elevation (fixed: RobotSettings.TurretAiming.FIXED_LAUNCH_ELEVATION_DEG)
        public double entryAngleDeg;    // descent angle at the target
        public boolean reachable;       // false if the aim is outside turret limits or RPM is above the flywheel maximum
        public boolean compensating;    // robot was moving, so the shot includes the motion
        public boolean fromBehind;      // the ball would arrive from behind the cell opening, so it cannot score

        /** Appends a note to reason (separated by "; "), never replacing an earlier one. */
        void addReason(String note) {
            reason = (reason == null || reason.isEmpty()) ? note : reason + "; " + note;
        }

        /** One short hardware-free status line: ok/reachable/fromBehind plus the reason. */
        public String statusText() {
            StringBuilder b = new StringBuilder();
            b.append(ok ? "ok" : "NO SHOT");
            b.append(reachable ? " reachable" : " UNREACHABLE");
            if (fromBehind) b.append(" from-behind");
            if (reason != null && !reason.isEmpty()) b.append(": ").append(reason);
            return b.toString();
        }
    }


    /**
     * @param x,y             robot pose (inches, radians)
     * @param vx,vy           robot field-frame velocity (in/s)
     * @param omega           robot yaw rate (rad/s, CCW positive)
     * @param alliance        which hive to aim at
     * @param farCellUp       true if the far-side cell of our hive is the raised one
     * @param currentAngleDeg current turret angle (ignored without a turret)
     * @param hasTurret       true if the robot has a turret
     * @param visionValid     true if the camera sees the target this loop
     * @param txDeg           camera horizontal offset in degrees (ignored unless visionValid)
     */
    public static Solution solve(double x, double y, double heading,
                                 double vx, double vy, double omega,
                                 Alliance alliance, boolean farCellUp, double currentAngleDeg,
                                 boolean hasTurret,
                                 boolean visionValid, double txDeg) {
        Solution s = new Solution();
        double targetX = FieldConstants.aimX(alliance);
        double targetY = FieldConstants.aimY(farCellUp);

        if (RobotSettings.TurretAiming.USE_LATENCY_COMPENSATION) {
            x += vx * RobotSettings.TurretAiming.LATENCY_S;
            y += vy * RobotSettings.TurretAiming.LATENCY_S;
            heading += omega * RobotSettings.TurretAiming.LATENCY_S;
        }

        double phiDeg;
        if (RobotSettings.TurretAiming.SHOT_SOURCE == ShotSource.TABLE) {
            double dx = targetX - x;
            double dy = targetY - y;
            s.ok = true;
            s.distanceIn = Math.hypot(dx, dy);
            s.aimHeadingRad = Math.atan2(dy, dx);
            s.rpm = ShotTable.rpm(s.distanceIn);
            s.launchElevDeg = RobotSettings.TurretAiming.FIXED_LAUNCH_ELEVATION_DEG;
            phiDeg = Math.toDegrees(wrapRadians(s.aimHeadingRad - heading));
        } else {
            ShotSolver.Input in = new ShotSolver.Input();
            boolean moving = RobotSettings.TurretAiming.USE_SHOOT_ON_THE_MOVE && Math.hypot(vx, vy) >= RobotSettings.TurretAiming.MIN_MOVE_SPEED_IN_S;
            in.x = x;
            in.y = y;
            in.heading = heading;
            in.vx = moving ? vx : 0;
            in.vy = moving ? vy : 0;
            in.omega = moving ? omega : 0;
            in.pivotOffsetX = hasTurret ? RobotSettings.TurretAiming.PIVOT_OFFSET_X_IN : 0;
            in.pivotOffsetY = hasTurret ? RobotSettings.TurretAiming.PIVOT_OFFSET_Y_IN : 0;
            in.exitHeight = RobotSettings.TurretAiming.EXIT_HEIGHT_IN;
            in.targetX = targetX;
            in.targetY = targetY;
            in.targetZ = FieldConstants.UP_OPENING_CENTER_Z_IN;
            in.allowRising = RobotSettings.TurretAiming.ALLOW_RISING_ARRIVAL;
            s.compensating = moving;

            // Fixed launch elevation (no hood): ShotSolver's HOOD_ANGLE mode solves for the launch speed.
            ShotSolver.Result r = ShotSolver.solve(in, ShotSolver.Mode.HOOD_ANGLE,
                    Math.toRadians(RobotSettings.TurretAiming.FIXED_LAUNCH_ELEVATION_DEG), ShotSolver.Arc.LOW);

            if (!r.ok) {
                s.ok = false;
                s.addReason(r.reason == null || r.reason.isEmpty() ? "no ballistic solution" : r.reason);
                s.distanceIn = r.distanceIn;
                s.reachable = false;
                return s;
            }
            s.ok = true;
            s.distanceIn = r.distanceIn;
            s.flightTimeS = r.flightTimeS;
            s.launchSpeedInS = r.launchSpeedInS;
            s.launchElevDeg = Math.toDegrees(r.elevationRad);
            s.entryAngleDeg = Math.toDegrees(r.entryAngleRad);
            s.aimHeadingRad = r.azimuthRad;
            s.rpm = speedToRpm(r.launchSpeedInS);
            phiDeg = Math.toDegrees(r.turretRad);

            // The ball must travel AGAINST the opening's outward normal to get in. Check the arrival velocity.
            double vgx = r.pivotVx + r.launchSpeedInS * Math.cos(r.elevationRad) * Math.cos(r.azimuthRad);
            double vgy = r.pivotVy + r.launchSpeedInS * Math.cos(r.elevationRad) * Math.sin(r.azimuthRad);
            double vz = r.launchSpeedInS * Math.sin(r.elevationRad) - ShotSolver.G_IN_S2 * r.flightTimeS;
            double[] n = FieldConstants.openingNormal(farCellUp);
            s.fromBehind = n[0] * vgx + n[1] * vgy + n[2] * vz >= 0;
            if (s.fromBehind) s.addReason("ball would arrive from behind the cell opening");
        }

        if (hasTurret) {
            if (RobotSettings.TurretAiming.USE_VISION_TRIM && visionValid) {
                double trim = RobotSettings.TurretAiming.VISION_TX_SIGN * RobotSettings.TurretAiming.VISION_KP * txDeg;
                phiDeg += Math.max(-RobotSettings.TurretAiming.MAX_VISION_TRIM_DEG, Math.min(RobotSettings.TurretAiming.MAX_VISION_TRIM_DEG, trim));
            }
            double lo = TurretLimits.min();
            double hi = TurretLimits.max();
            // The servo window is narrower than 360 deg, so there is no seam: at most one equivalent angle fits.
            double best = Double.NaN;
            for (int k = -2; k <= 2; k++) {
                double c = phiDeg + 360.0 * k;
                if (c >= lo && c <= hi
                        && (Double.isNaN(best) || Math.abs(c - currentAngleDeg) < Math.abs(best - currentAngleDeg))) {
                    best = c;
                }
            }
            if (Double.isNaN(best)) {
                // Target in the dead zone (e.g. behind the robot): park at the nearest limit.
                double dLo = Math.abs(shortestDifference(phiDeg, lo));
                double dHi = Math.abs(shortestDifference(phiDeg, hi));
                best = dLo < dHi ? lo : hi;
                s.reachable = false;
                s.addReason("aim outside turret limits");
            } else {
                s.reachable = true;
            }
            s.turretAngleDeg = best;
        } else {
            s.turretAngleDeg = 0;
            s.reachable = true;
        }
        double maxRpm = Flywheel.maxRpm();
        boolean rpmTooHigh = s.rpm > maxRpm;
        if (rpmTooHigh) {
            s.addReason("rpm " + String.format("%.0f", s.rpm) + " above MAX_RPM " + String.format("%.0f", maxRpm));
        }
        if (!(s.rpm > 0) || Double.isInfinite(s.rpm)) {   // NaN / 0 / negative / infinite RPM can never be a shot
            s.addReason("rpm " + s.rpm + " is not a usable flywheel speed");
            s.reachable = false;
        }
        if (s.rpm > 0 && s.rpm < RobotSettings.Flywheel.MIN_RPM) {
            s.addReason("rpm " + String.format("%.0f", s.rpm) + " below Flywheel.MIN_RPM " + String.format("%.0f", RobotSettings.Flywheel.MIN_RPM)
                    + " (flywheel would overshoot the shot)");
            s.reachable = false;
        }
        if (rpmTooHigh || s.fromBehind) {
            s.reachable = false;
        }
        return s;
    }

    /** Ball launch speed (in/s) to flywheel RPM: 60 * v / (2 pi R * efficiency), times RobotSettings.TurretAiming.RPM_SCALE. */
    public static double speedToRpm(double speedInS) {
        return RobotSettings.TurretAiming.RPM_SCALE * 60.0 * speedInS / (2.0 * Math.PI * RobotSettings.TurretAiming.WHEEL_RADIUS_IN * RobotSettings.TurretAiming.TRANSFER_EFFICIENCY);
    }

    /** Wraps an angle in radians to (-pi, pi]. */
    public static double wrapRadians(double a) {
        return ShotSolver.wrap(a);
    }

    /** Smallest signed difference a - b in degrees, in (-180, 180]. */
    private static double shortestDifference(double a, double b) {
        double d = (a - b) % 360.0;
        if (d > 180) d -= 360;
        if (d <= -180) d += 360;
        return d;
    }
}
