package robot;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;

import commands.subsystems.ball_sensing.BallSensor;


/**
 * Pre-flight check: lists every robot-configuration device the enabled subsystems need, so a missing or
 * misspelled name is reported ALL AT ONCE (with the subsystem it belongs to) instead of crashing on the first
 * hardwareMap.get(). Call {@link #missing} before building the Robot and show the result in telemetry.
 *
 * Names come from the same constants the subsystems use, so this can never disagree with them.
 * Optional devices (the Axon analog feedback inputs) are not checked: a missing one just means no feedback.
 */
public final class HardwareCheck {
    private HardwareCheck() {}

    /** One required device: what it is for, its config name and its type. */
    public static final class Need {
        public final String purpose;
        public final String name;
        public final Class<?> type;

        Need(String purpose, String name, Class<?> type) {
            this.purpose = purpose;
            this.name = name;
            this.type = type;
        }
    }

    /** Every device the currently enabled subsystems require (hardware-free, unit tested). */
    public static List<Need> required() {
        List<Need> n = new ArrayList<>();
        n.add(new Need("drive front left", RobotSettings.Drivetrain.FRONT_LEFT_NAME, DcMotorEx.class));
        n.add(new Need("drive front right", RobotSettings.Drivetrain.FRONT_RIGHT_NAME, DcMotorEx.class));
        n.add(new Need("drive back left", RobotSettings.Drivetrain.BACK_LEFT_NAME, DcMotorEx.class));
        n.add(new Need("drive back right", RobotSettings.Drivetrain.BACK_RIGHT_NAME, DcMotorEx.class));
        // DRIVE-MOTORS-ONLY MODE: odometry is not built (Robot.auto = null), so the Pinpoint is not required. Restore with Robot.java.
        // n.add(new Need("odometry (Pinpoint)", RobotSettings.Odometry.HARDWARE_NAME, GoBildaPinpointDriver.class));
        // DRIVE-ONLY MODE: only the four drive motors are required while every other subsystem is commented out
        // in Robot.java. To restore a subsystem, uncomment its lines here too (they mirror the blocks in Robot.java).
        // n.add(new Need("intake motor", RobotSettings.Intake.MOTOR_NAME, DcMotorEx.class));
        // n.add(new Need("transfer motor", RobotSettings.Transfer.MOTOR_NAME, DcMotorEx.class));
        // if (RobotSettings.Robot.USE_SHOOTER) {
        //     n.add(new Need("flywheel motor A", RobotSettings.Flywheel.MOTOR_NAME, DcMotorEx.class));
        //     n.add(new Need("flywheel motor B", RobotSettings.Flywheel.MOTOR_B_NAME, DcMotorEx.class));
        //     n.add(new Need("turret servo A", RobotSettings.Turret.SERVO_A_NAME, Servo.class));
        //     n.add(new Need("turret servo B", RobotSettings.Turret.SERVO_B_NAME, Servo.class));
        // }
        // if (RobotSettings.Robot.USE_LIMELIGHT) {
        //     n.add(new Need("limelight", RobotSettings.Limelight.HARDWARE_NAME, Limelight3A.class));
        // }
        // // FlowerMech / Gate build no servo at all when their own ENABLED switch is off.
        // if (RobotSettings.Robot.USE_FLOWER_MECH && RobotSettings.FlowerMech.ENABLED) {
        //     n.add(new Need("flower mech servo A", RobotSettings.FlowerMech.SERVO_A_NAME, Servo.class));
        //     if (RobotSettings.FlowerMech.SERVO_COUNT >= 2) {
        //         n.add(new Need("flower mech servo B", RobotSettings.FlowerMech.SERVO_B_NAME, Servo.class));
        //     }
        // }
        // if ((RobotSettings.Robot.USE_GATE || RobotSettings.Robot.USE_COLOR_SORT) && RobotSettings.Gate.ENABLED) {
        //     n.add(new Need("lane gate servo", RobotSettings.Gate.SERVO_NAME, Servo.class));
        // }
        // if (RobotSettings.Robot.USE_COLOR_SORT) {
        //     // BallSensor looks a DISTANCE_ONLY sensor up as a DistanceSensor, every other type as a colour sensor.
        //     Class<?> sensorType = RobotSettings.BallSensor.SENSOR_TYPE == BallSensor.SensorType.DISTANCE_ONLY
        //             ? DistanceSensor.class : NormalizedColorSensor.class;
        //     n.add(new Need("lane colour sensor", RobotSettings.LaneSorter.SENSOR_NAME, sensorType));
        // }
        return n;
    }

    /** One line per required device that is NOT in the robot configuration; empty = all present. */
    public static List<String> missing(HardwareMap hardwareMap) {
        List<String> out = new ArrayList<>();
        for (Need need : required()) {
            Object found = null;
            try {
                found = hardwareMap.tryGet(need.type, need.name);
            } catch (RuntimeException ignored) {
                // treated as missing
            }
            if (found == null) {
                out.add(need.purpose + ": \"" + need.name + "\" (" + need.type.getSimpleName() + ")");
            }
        }
        return out;
    }

    /** Every required config name, for printing a configuration checklist. */
    public static List<String> checklist() {
        List<String> out = new ArrayList<>();
        for (Need need : required()) {
            out.add(need.purpose + ": " + need.name + " (" + need.type.getSimpleName() + ")");
        }
        return out;
    }
}
