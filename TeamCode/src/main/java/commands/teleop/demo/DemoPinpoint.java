package commands.teleop.demo;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import robot.RobotSettings;

/**
 * Opens the Pinpoint for the Demo OpModes straight from the driver, without Pedro, using the same
 * RobotSettings.Odometry values Pedro's localizer is built from (pedroPathing.Constants). Returns null when the
 * device is not in the robot configuration so each demo can show "not found" instead of crashing.
 *
 * Frame: x forward, y left, heading CCW positive, all relative to where the robot was when the pose was last reset.
 */
final class DemoPinpoint {
    private DemoPinpoint() {}

    /** The configured Pinpoint, or null if missing. Does not move anything. */
    static GoBildaPinpointDriver open(HardwareMap hw) {
        GoBildaPinpointDriver p;
        try {
            p = hw.tryGet(GoBildaPinpointDriver.class, RobotSettings.Odometry.HARDWARE_NAME);
        } catch (RuntimeException e) {
            return null;
        }
        if (p == null) {
            return null;
        }
        // Same call order and argument meaning as Pedro's PinpointLocalizer: (parallel pod Y, perpendicular pod X).
        p.setOffsets(RobotSettings.Odometry.PARALLEL_POD_OFFSET_IN, RobotSettings.Odometry.PERPENDICULAR_POD_OFFSET_IN, DistanceUnit.INCH);
        p.setEncoderResolution(RobotSettings.Odometry.POD_TYPE);
        p.setEncoderDirections(RobotSettings.Odometry.PARALLEL_POD_DIRECTION, RobotSettings.Odometry.PERPENDICULAR_POD_DIRECTION);
        p.resetPosAndIMU();
        return p;
    }

    /** Sets the pose to (0, 0, 0) without recalibrating the IMU (robot may be anywhere). */
    static void zeroPose(GoBildaPinpointDriver p) {
        p.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0));
    }

    static double x(GoBildaPinpointDriver p) { return p.getPosX(DistanceUnit.INCH); }

    static double y(GoBildaPinpointDriver p) { return p.getPosY(DistanceUnit.INCH); }

    static double headingRad(GoBildaPinpointDriver p) { return p.getHeading(AngleUnit.RADIANS); }

    static boolean finite(double... v) {
        for (double d : v) {
            if (Double.isNaN(d) || Double.isInfinite(d)) return false;
        }
        return true;
    }
}
