package pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import robot.RobotSettings;

/**
 * Pedro Pathing setup. Motor names/directions come from RobotSettings.Drivetrain, the localizer from
 * RobotSettings.Odometry and the path constraints from RobotSettings.PathFollowing (one source of truth).
 * Only followerConstants (Pedro library defaults, filled in by Pedro tuning) is built here.
 *
 * The follower, braking and PIDF values are still LIBRARY DEFAULTS: run the Tuning OpMode
 * (forward/lateral velocity, zero-power acceleration, then PIDFs) before trusting the path following.
 * The motion profiles read the measured forward velocity (driveConstants.xVelocity) from here.
 */
public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants();

    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName(RobotSettings.Drivetrain.FRONT_LEFT_NAME)
            .leftRearMotorName(RobotSettings.Drivetrain.BACK_LEFT_NAME)
            .rightFrontMotorName(RobotSettings.Drivetrain.FRONT_RIGHT_NAME)
            .rightRearMotorName(RobotSettings.Drivetrain.BACK_RIGHT_NAME)
            .leftFrontMotorDirection(RobotSettings.Drivetrain.FRONT_LEFT_DIRECTION)
            .leftRearMotorDirection(RobotSettings.Drivetrain.BACK_LEFT_DIRECTION)
            .rightFrontMotorDirection(RobotSettings.Drivetrain.FRONT_RIGHT_DIRECTION)
            .rightRearMotorDirection(RobotSettings.Drivetrain.BACK_RIGHT_DIRECTION);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .hardwareMapName(RobotSettings.Odometry.HARDWARE_NAME)
            .distanceUnit(DistanceUnit.INCH)
            .forwardPodY(RobotSettings.Odometry.PARALLEL_POD_OFFSET_IN)
            .strafePodX(RobotSettings.Odometry.PERPENDICULAR_POD_OFFSET_IN)
            .encoderResolution(RobotSettings.Odometry.POD_TYPE)
            .forwardEncoderDirection(RobotSettings.Odometry.PARALLEL_POD_DIRECTION)
            .strafeEncoderDirection(RobotSettings.Odometry.PERPENDICULAR_POD_DIRECTION);

    public static PathConstraints pathConstraints = new PathConstraints(RobotSettings.PathFollowing.TVALUE_CONSTRAINT,
            RobotSettings.PathFollowing.TIMEOUT_CONSTRAINT_MS,
            RobotSettings.PathFollowing.BRAKING_STRENGTH,
            RobotSettings.PathFollowing.BRAKING_START);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
