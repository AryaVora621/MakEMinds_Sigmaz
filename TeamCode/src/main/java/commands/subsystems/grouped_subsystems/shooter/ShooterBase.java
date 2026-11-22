package commands.subsystems.grouped_subsystems.shooter;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import commands.field.Alliance;
import commands.subsystems.flywheel.Flywheel;
import commands.subsystems.turret.Turret;
import commands.subsystems.turret.TurretAiming;
import robot.RobotSettings;

/**
 * Shared logic for every shooter configuration. A configuration is just whether a turret exists (the launch
 * angle is fixed: there is no hood); the public class FlywheelTurret (the only one kept) chooses that. Each
 * loop: solve the shot with {@link TurretAiming} (ballistic physics, or {@link ShotTable} when
 * RobotSettings.TurretAiming.SHOT_SOURCE = TABLE), command the turret / flywheel, report readiness.
 *
 * Robots without a turret must point the chassis at the target: use {@link #getAimHeadingRad()}.
 */
public abstract class ShooterBase {

    private final Flywheel flywheel;
    private final Turret turret;       // null if the configuration has no turret

    private TurretAiming.Solution solution = new TurretAiming.Solution();
    private boolean spinning;
    private double heading;
    /** RPM of the last reachable solution (0 if none yet); held while the current solution is unreachable. */
    private double lastReachableRpm;

    protected ShooterBase(HardwareMap hardwareMap, boolean hasTurret) {
        flywheel = new Flywheel(hardwareMap);
        turret = hasTurret ? new Turret(hardwareMap) : null;
    }

    /** Human-readable configuration name. */
    public abstract String configurationName();

    public boolean hasTurret() { return turret != null; }

    /** Spin the flywheel up to the solved RPM (true) or let it stop (false). */
    public void setSpinning(boolean on) {
        spinning = on;
    }

    /**
     * Call once per loop.
     *
     * @param pose        robot pose from the follower
     * @param velocity    robot field-frame velocity (in/s), for shooting while driving
     * @param omega       robot yaw rate (rad/s, CCW positive), for shooting while turning
     * @param farCellUp   true if the far-side cell of our hive is the raised one
     * @param visionValid true if the camera sees the target
     * @param txDeg       camera horizontal offset (degrees)
     */
    public void update(Pose pose, Vector velocity, double omega, Alliance alliance, boolean farCellUp,
                       boolean visionValid, double txDeg) {
        heading = pose.getHeading();
        solution = TurretAiming.solve(pose.getX(), pose.getY(), pose.getHeading(),
                velocity.getXComponent(), velocity.getYComponent(), omega,
                alliance, farCellUp, turret == null ? 0 : turret.getAngle(),
                turret != null, visionValid, txDeg);

        if (solution.ok && turret != null) {
            turret.setTargetAngle(solution.turretAngleDeg);
        }
        // Only a reachable solution may change the RPM target; otherwise keep the last reachable one.
        if (solution.ok && solution.reachable && !Double.isNaN(solution.rpm) && !Double.isInfinite(solution.rpm)) {
            lastReachableRpm = Math.max(0, Math.min(solution.rpm, Flywheel.maxRpm()));
        }
        flywheel.setTargetRpm(spinning ? lastReachableRpm : 0);
        if (turret != null) {
            turret.update(Math.toDegrees(omega));
        }
        flywheel.update();
    }

    /** True when it is safe to feed a ball: flywheel at speed, aim within the turret limits, and aimed. */
    public boolean ready() {
        if (!spinning || !solution.ok || !solution.reachable || !flywheel.atTarget()) {
            return false;
        }
        if (turret != null) {
            return turret.atTarget();
        }
        double err = Math.toDegrees(TurretAiming.wrapRadians(solution.aimHeadingRad - heading));
        return Math.abs(err) <= RobotSettings.Shooter.HEADING_TOLERANCE_DEG;
    }

    /** Field direction the shot must go (rad). A robot without a turret must face this. */
    public double getAimHeadingRad() {
        return solution.aimHeadingRad;
    }

    /** Sends the turret to 0 deg (servo centre); the servos are absolute, so nothing is re-zeroed. Does nothing without a turret. */
    public void rezeroTurret() {
        if (turret != null) {
            turret.rezeroHere();
        }
    }

    /** Where the turret zero came from (for telemetry); "no turret" if there is none. */
    public String turretZeroSource() {
        return turret == null ? "no turret" : turret.zeroSource();
    }

    /** Turret angle (degrees): Axon feedback if calibrated, otherwise Turret's time-based estimate; 0 if there is no turret. */
    public double getTurretAngleDeg() {
        return turret == null ? 0 : turret.getAngle();
    }

    public TurretAiming.Solution getSolution() {
        return solution;
    }

    public Flywheel getFlywheel() {
        return flywheel;
    }

    public void stop() {
        spinning = false;
        lastReachableRpm = 0;
        flywheel.stop();
        if (turret != null) {
            turret.stop();
        }
    }
}
