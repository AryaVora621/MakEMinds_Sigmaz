package commands.autonomous;

import java.util.function.BooleanSupplier;

import robot.Robot;
import robot.RobotSettings;

/**
 * Blocking autoAim command for LinearOpMode autonomous routines.
 *
 * <p>It is the Robot-side counterpart of the simulator's {@code autoAim} palette command (command type
 * {@code 'autoAim'} with boolean {@code shootAfter}); see {@code field_sim/web/js/autoaim_cmd.js}.
 *
 * <p>Usage example:
 * <pre>{@code
 * public void runOpMode() {
 *     Robot robot = new Robot(hardwareMap, startPose);
 *     waitForStart();
 *     try {
 *         robot.auto.followPath(pathToShootingSpot, true, 4.0);   // drive to the shooting spot
 *         while (opModeIsActive() && robot.auto.isBusyOrTimeout()) {
 *             robot.update();
 *         }
 *         AutoAim.run(robot, this::opModeIsActive, RobotSettings.AutoAim.TIMEOUT_S, true, null); // aim, feed, shoot
 *     } finally {
 *         robot.stop();
 *     }
 * }
 * }</pre>
 *
 * <p>Semantics:
 * <ul>
 *   <li>Aim targets come from the existing solver, {@code TurretAiming} / {@code ShotSolver}, which
 *       {@code ShooterBase.update} already calls each loop via {@code robot.update()}. AutoAim does not
 *       duplicate the solver. It only sets the flywheel spinning.</li>
 *   <li>Aligns the flywheel RPM and the turret, both through {@code ShooterBase.update} (no hood on this robot).</li>
 *   <li>Turns the transfer on by calling {@code robot.intake.feed()} (intake and transfer forward) each
 *       loop while waiting, when {@link RobotSettings.AutoAim#TRANSFER_ON_START} is true. Otherwise it stops
 *       the intake until the logic turns transfer on.</li>
 *   <li>Finishes when {@code ShooterBase.ready()} is true. That covers the spinning flag, a valid and
 *       reachable solution, the flywheel at target, and the turret at target when a turret is present.
 *       It does not repeat those checks.</li>
 *   <li>The timeout guarantees auto never hangs. After {@code timeoutS} seconds the loop ends with
 *       {@code aimed=false} even if the shooter never becomes ready. A value {@code <= 0} uses
 *       {@link RobotSettings.AutoAim#TIMEOUT_S}. The loop also ends if the OpMode stops.</li>
 *   <li>If {@code shootAfter} is false, the intake is stopped on exit. The flywheel keeps spinning and
 *       aimed so the next path can keep driving. {@code robot.stop()} is NOT called.</li>
 *   <li>If {@code shootAfter} is true, a second loop runs for at most
 *       {@link RobotSettings.AutoAim#SHOOT_AFTER_MAX_S} seconds. Each loop it calls
 *       {@code robot.feedIfReady(true)}. That feeds only when {@code Robot.readyToFire()} allows, and
 *       otherwise stops the intake. The shooter keeps tracking during this phase. The intake
 *       is stopped before returning. This loop is bounded too, so it cannot hang.</li>
 * </ul>
 *
 * <p>Return value: true only if aim was reached before the timeout. A false return with an active
 * OpMode means the timeout elapsed. A false return with no shooter means nothing was done.
 */
public final class AutoAim {

    private AutoAim() {
        // static helpers only
    }

    /**
     * Runs autoAim using {@link RobotSettings.AutoAim#TIMEOUT_S} as the timeout.
     *
     * @param robot         the robot (its update(), shooter and intake are used)
     * @param opModeActive  supplies opModeIsActive(); the loop ends as soon as it returns false
     * @param shootAfter    if true, keep feeding for up to {@link RobotSettings.AutoAim#SHOOT_AFTER_MAX_S} seconds
     * @return true if aimed before the timeout
     */
    public static boolean run(Robot robot, BooleanSupplier opModeActive, boolean shootAfter) {
        return run(robot, opModeActive, RobotSettings.AutoAim.TIMEOUT_S, shootAfter, null);
    }

    /**
     * Runs autoAim.
     *
     * @param robot         the robot (its update(), shooter and intake are used)
     * @param opModeActive  supplies opModeIsActive(); the loop ends as soon as it returns false
     * @param timeoutS      seconds to wait for aim; a value {@code <= 0} uses {@link RobotSettings.AutoAim#TIMEOUT_S}
     * @param shootAfter    if true, keep feeding for up to {@link RobotSettings.AutoAim#SHOOT_AFTER_MAX_S} seconds
     * @param perLoop       optional callback run once per loop for telemetry; may be null
     * @return true if aimed before the timeout; false on timeout, no shooter, or the OpMode stopping
     */
    public static boolean run(Robot robot, BooleanSupplier opModeActive, double timeoutS, boolean shootAfter,
                              Runnable perLoop) {
        if (robot == null || robot.shooter == null) {
            return false;
        }
        robot.shooter.setSpinning(true);

        double effectiveTimeoutS = !(timeoutS > 0) ? RobotSettings.AutoAim.TIMEOUT_S : timeoutS;
        AutoAimLogic logic = new AutoAimLogic(effectiveTimeoutS, shootAfter, RobotSettings.AutoAim.TRANSFER_ON_START);

        boolean aimed = false;
        try {
            while (opModeActive.getAsBoolean()) {
                robot.update();
                boolean aimReady = robot.shooter.ready();
                AutoAimLogic.Output out = logic.update(System.nanoTime() / 1e9, true, aimReady);
                if (out.transferOn) {
                    robot.intake.feed();
                } else {
                    robot.intake.stop();
                }
                if (perLoop != null) {
                    perLoop.run();
                }
                if (out.done) {
                    aimed = out.aimed;
                    break;
                }
            }

            if (!shootAfter) {
                robot.intake.stop();
                return aimed;
            }

            // Shoot-after phase: bounded, so it cannot hang even if the robot never becomes ready.
            double endS = System.nanoTime() / 1e9 + RobotSettings.AutoAim.SHOOT_AFTER_MAX_S;
            while (opModeActive.getAsBoolean() && System.nanoTime() / 1e9 < endS) {
                robot.update();
                if (!robot.feedIfReady(true)) {
                    robot.intake.stop();
                }
                if (perLoop != null) {
                    perLoop.run();
                }
            }
            robot.intake.stop();
        } finally {
            robot.intake.stop(); // never leave a latched feed running on exception/cancel; flywheel stays spinning
        }
        return aimed;
    }

    /**
     * Human-readable status for telemetry.
     *
     * @param o the output from {@link AutoAimLogic#update}; may be null
     * @return "AutoAim: aimed", "AutoAim: TIMEOUT (not aimed)", or "AutoAim: aligning"
     */
    public static String statusText(AutoAimLogic.Output o) {
        if (o == null || !o.done) {
            return "AutoAim: aligning";
        }
        if (o.aimed) {
            return "AutoAim: aimed";
        }
        if (o.timedOut) {
            return "AutoAim: TIMEOUT (not aimed)";
        }
        return "AutoAim: no shooter (not aimed)";
    }
}
