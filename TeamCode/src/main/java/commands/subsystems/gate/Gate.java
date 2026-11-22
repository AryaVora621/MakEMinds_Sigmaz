package commands.subsystems.gate;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import commands.subsystems.servo.AxonServo;
import robot.RobotSettings;

/** Gate on an Axon servo. Starts closed. Call update() every loop. */
public class Gate {
    private final AxonServo servo;
    private final ElapsedTime dtTimer = new ElapsedTime();
    private double lastDtS = 0;
    private boolean open;

    public Gate(HardwareMap hw) {
        if (!RobotSettings.Gate.ENABLED) {
            servo = null;
            return;
        }
        servo = new AxonServo(hw, RobotSettings.Gate.SERVO_NAME, RobotSettings.Gate.FEEDBACK_NAME);
        servo.setReversed(RobotSettings.Gate.SERVO_REVERSED);
        close();
    }

    public void open() {
        open = true;
        if (servo != null) servo.setTargetDeg(RobotSettings.Gate.OPEN_DEG);
    }

    public void close() {
        open = false;
        if (servo != null) servo.setTargetDeg(RobotSettings.Gate.CLOSED_DEG);
    }

    public void toggle() {
        if (open) close(); else open();
    }

    public void update() {
        lastDtS = GateLogic.clampDt(dtTimer.seconds(), RobotSettings.Gate.MAX_DT_S);
        dtTimer.reset();
        if (servo != null) servo.update();
    }

    /** Capped time step of the last update (AxonServo keeps its own timer and cap). */
    public double getLastDtS() {
        return lastDtS;
    }

    /** Commanded state, not measured. */
    public boolean isOpen() {
        return open;
    }

    public boolean atTarget() {
        return servo == null || servo.atTarget(RobotSettings.Gate.AT_TARGET_TOL_DEG);
    }

    public boolean isStalled() {
        return servo != null && servo.isStalled();
    }
}
