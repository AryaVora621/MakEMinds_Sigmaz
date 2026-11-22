package commands.autonomous;

import java.util.function.DoubleSupplier;

import robot.Robot;
import robot.RobotSettings;

/**
 * Non-blocking, framework-neutral command form of {@link AutoAim}. The codebase has no command framework
 * (no NextFTC/FTCLib/Ivy dependency), so this uses the common lifecycle shape: {@link #initialize()},
 * {@link #execute()} every loop, {@link #isFinished()}, {@link #end(boolean)}. Adapt it to whichever
 * scheduler is added later by forwarding those four calls.
 *
 * <p>Differences from the blocking {@code AutoAim.run}:
 * <ul>
 *   <li>It never calls {@code robot.update()}. The owner (scheduler or OpMode loop) must call it exactly
 *       once per loop. That is what makes it safe in a parallel group (no double update).</li>
 *   <li>It never loops. {@code execute()} does one tick and returns.</li>
 *   <li>{@code end(interrupted)} always stops the intake/transfer (a latched feed would otherwise run on).
 *       The flywheel is deliberately LEFT SPINNING on both normal end and cancel, so the next path keeps
 *       the shooter at speed. Call {@code robot.shooter.setSpinning(false)} (or {@code robot.stop()}) when
 *       a long non-shooting stretch follows.</li>
 *   <li>Requirements: it drives the intake/transfer and the shooter. Do not run it in parallel with another
 *       command that writes the intake (last writer wins) or sets shooter targets.</li>
 * </ul>
 * Pedro 2.1.0 chain callbacks are not used.
 */
public final class AutoAimStep {

    /** Hardware seam so the step is unit-testable without a Robot. */
    public interface Port {
        boolean shooterPresent();
        void setSpinning(boolean on);
        boolean aimReady();
        /** Intake and transfer forward (latched feed). */
        void feed();
        void stopIntake();
        /** Feed only when the robot may fire; returns true if feeding. */
        boolean feedIfReady();
    }

    private final Port port;
    private final double timeoutS;
    private final boolean shootAfter;
    private final DoubleSupplier clockS;

    private AutoAimLogic logic;
    private boolean initialized;
    private boolean finished;
    private boolean aimed;
    private boolean timedOut;
    private boolean inShootPhase;
    private double shootEndS;

    public AutoAimStep(Port port, double timeoutS, boolean shootAfter, DoubleSupplier clockS) {
        this.port = port;
        this.timeoutS = !(timeoutS > 0) ? RobotSettings.AutoAim.TIMEOUT_S : timeoutS;
        this.shootAfter = shootAfter;
        this.clockS = clockS;
    }

    /** Builds a step bound to a real robot, using {@link RobotSettings.AutoAim} and the system clock. */
    public static AutoAimStep forRobot(Robot robot, boolean shootAfter) {
        return forRobot(robot, RobotSettings.AutoAim.TIMEOUT_S, shootAfter);
    }

    public static AutoAimStep forRobot(final Robot robot, double timeoutS, boolean shootAfter) {
        Port p = new Port() {
            @Override public boolean shooterPresent() { return robot != null && robot.shooter != null; }
            @Override public void setSpinning(boolean on) { robot.shooter.setSpinning(on); }
            @Override public boolean aimReady() { return robot.shooter.ready(); }
            @Override public void feed() { robot.intake.feed(); }
            @Override public void stopIntake() { robot.intake.stop(); }
            @Override public boolean feedIfReady() { return robot.feedIfReady(true); }
        };
        return new AutoAimStep(p, timeoutS, shootAfter, () -> System.nanoTime() / 1e9);
    }

    /** Starts (or restarts) the command. Safe to call again to re-run. */
    public void initialize() {
        logic = new AutoAimLogic(timeoutS, shootAfter, RobotSettings.AutoAim.TRANSFER_ON_START);
        finished = false;
        aimed = false;
        timedOut = false;
        inShootPhase = false;
        initialized = true;
        if (port.shooterPresent()) {
            port.setSpinning(true);
        }
    }

    /** One tick. Call once per loop AFTER the owner's single robot.update(). No-op once finished. */
    public void execute() {
        if (!initialized) {
            initialize();
        }
        if (finished) {
            return;
        }
        double now = clockS.getAsDouble();
        if (inShootPhase) {
            if (!(now < shootEndS)) {
                end(false);
                return;
            }
            if (!port.feedIfReady()) {
                port.stopIntake();
            }
            return;
        }
        boolean present = port.shooterPresent();
        AutoAimLogic.Output out = logic.update(now, present, present && port.aimReady());
        if (out.transferOn) {
            port.feed();
        } else {
            port.stopIntake();
        }
        if (out.done) {
            aimed = out.aimed;
            timedOut = out.timedOut;
            if (shootAfter && present) {
                inShootPhase = true;
                shootEndS = now + RobotSettings.AutoAim.SHOOT_AFTER_MAX_S;
            } else {
                end(false);
            }
        }
    }

    public boolean isFinished() {
        return finished;
    }

    /**
     * Ends the command. Idempotent. Stops intake/transfer in every case; leaves the flywheel spinning.
     *
     * @param interrupted true if cancelled by a group (race/deadline loser, OpMode stop)
     */
    public void end(boolean interrupted) {
        finished = true;
        inShootPhase = false;
        if (initialized) {
            port.stopIntake();
        }
    }

    /** True if aim was reached before the timeout. Valid once aiming has finished. */
    public boolean aimed() {
        return aimed;
    }

    public boolean timedOut() {
        return timedOut;
    }
}
