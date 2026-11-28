package commands.teleop.demo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import commands.subsystems.flywheel.Flywheel;
import robot.RobotSettings;

/**
 * Spins both flywheel motors to a chosen RPM and shows what each motor really does. KEEP HANDS, LOOSE ITEMS AND
 * BALLS AWAY from the wheel. The shooter must be clear and the robot held or on blocks.
 *
 *   A = spin up to the target, B = STOP (also stops if the OpMode ends), release nothing else needed
 *   D-pad up / down = target +/- 100 rpm, D-pad right / left = +/- 500 rpm (default 2000, max Flywheel.HARD_MAX_RPM)
 *
 * The request ramps up at 1000 rpm/s so the PIDF does not slam. Flywheel raises any non-zero target to
 * RobotSettings.Flywheel.MIN_RPM, so a 2000 target actually runs at "effective target" (shown). If a motor's rpm is
 * negative while the other is positive, flip MOTOR_B_REVERSED (or MOTOR_REVERSED). If one reads 0 while the wheel
 * spins, that motor's encoder is not plugged in. If the controller sees no motion at full power for 2 s the
 * flywheel cuts its own power (shown as RUN-AWAY GUARD).
 */
@TeleOp(name = "Demo: Flywheel Spin", group = "Demo")
public class FlywheelSpinDemo extends LinearOpMode {
    private static final double RAMP_RPM_PER_S = 1000;

    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(50);
        Flywheel fw = null;
        String err = "";
        try {
            fw = new Flywheel(hardwareMap);
        } catch (RuntimeException e) {
            err = "flywheel motor A \"" + RobotSettings.Flywheel.MOTOR_NAME + "\" not found in the robot config (" + e.getMessage() + ")";
        }
        VoltageSensor vs = hardwareMap.voltageSensor.iterator().hasNext() ? hardwareMap.voltageSensor.iterator().next() : null;
        double max = Flywheel.HARD_MAX_RPM;
        double target = 2000;
        while (!isStarted() && !isStopRequested()) {
            if (fw == null) telemetry.addData("Flywheel", "NOT FOUND: " + err);
            else telemetry.addData("Flywheel", "motor A found, motor B \"%s\" %s", RobotSettings.Flywheel.MOTOR_B_NAME, fw.hasMotorB() ? "found" : "NOT FOUND (running on A only)");
            telemetry.addLine("Nothing spins until PLAY and A. Keep clear of the wheel.");
            telemetry.update();
        }
        if (isStopRequested()) return;

        boolean spin = false;
        double cmd = 0;
        boolean upWas = false, downWas = false, rightWas = false, leftWas = false;
        ElapsedTime loop = new ElapsedTime();
        try {
            while (opModeIsActive()) {
                double dt = Math.min(loop.seconds(), 0.1);
                loop.reset();
                if (gamepad1.dpad_up && !upWas) target = DemoMath.adjust(target, true, false, 100, 0, max);
                if (gamepad1.dpad_down && !downWas) target = DemoMath.adjust(target, false, true, 100, 0, max);
                if (gamepad1.dpad_right && !rightWas) target = DemoMath.adjust(target, true, false, 500, 0, max);
                if (gamepad1.dpad_left && !leftWas) target = DemoMath.adjust(target, false, true, 500, 0, max);
                upWas = gamepad1.dpad_up; downWas = gamepad1.dpad_down; rightWas = gamepad1.dpad_right; leftWas = gamepad1.dpad_left;
                if (gamepad1.b) spin = false;
                else if (gamepad1.a) spin = true;

                if (fw != null) {
                    if (spin && target > 0) {
                        if (cmd < 1) cmd = Math.min(RobotSettings.Flywheel.MIN_RPM, target);   // Flywheel raises small targets to MIN_RPM anyway
                        cmd = DemoMath.stepToward(cmd, target, RAMP_RPM_PER_S * dt);
                        fw.setTargetRpm(cmd);
                    } else {
                        cmd = 0;
                        fw.setTargetRpm(0);
                    }
                    fw.update();
                }

                telemetry.addData("state", fw == null ? "NO FLYWHEEL" : fw.isFaulted() ? "RUN-AWAY GUARD: power cut (no wheel motion at full power). Press B, check encoder and wiring" : spin ? "SPINNING (B = stop)" : "STOPPED (A = spin)");
                if (fw == null) {
                    telemetry.addData("Flywheel", "NOT FOUND: " + err);
                } else {
                    telemetry.addData("target (dpad)", "%.0f rpm   (max %.0f)", target, max);
                    telemetry.addData("effective target", "%.0f rpm%s", fw.getTargetRpm(),
                            spin && target > 0 && target < RobotSettings.Flywheel.MIN_RPM ? "  (raised to Flywheel.MIN_RPM)" : "");
                    double a = fw.getMotorARpm(), b = fw.getMotorBRpm();
                    telemetry.addData("motor A rpm", "%.0f", a);
                    telemetry.addData("motor B rpm", fw.hasMotorB() ? String.format("%.0f", b) : "NOT FOUND (\"" + RobotSettings.Flywheel.MOTOR_B_NAME + "\")");
                    if (fw.hasMotorB()) {
                        telemetry.addData("A - B difference (rpm)", "%.0f%s", a - b,
                                (a > 300 && b < -300) || (b > 300 && a < -300) ? "  OPPOSITE SIGNS: flip a MOTOR_REVERSED setting" : "");
                    }
                    telemetry.addData("controller rpm (used for control)", "%.0f  (%s)", fw.getRpm(), RobotSettings.Flywheel.VELOCITY_FROM_BOTH ? "avg of A and B" : "motor A only");
                    telemetry.addData("at target", "%b", fw.atTarget());
                }
                telemetry.addData("battery (V)", vs == null ? "no sensor" : String.format("%.1f", vs.getVoltage()));
                telemetry.addLine("A spin | B stop | dpad up/down +/-100, left/right +/-500");
                telemetry.update();
            }
        } finally {
            if (fw != null) {
                try { fw.stop(); } catch (RuntimeException ignored) { }
            }
        }
    }
}
