package commands.subsystems.grouped_subsystems.intake_transfer;

import com.qualcomm.robotcore.hardware.HardwareMap;

import commands.subsystems.intake.Intake;
import commands.subsystems.transfer.Transfer;

/** Intake and transfer kept in sync: the transfer always runs in the same direction as the intake. No sensors. */
public class IntakeTransfer {
    enum Mode { STOPPED, FORWARD, BACKWARD, FEED }

    private final Intake intake;
    private final Transfer transfer;
    private Mode mode = Mode.STOPPED;

    public IntakeTransfer(HardwareMap hardwareMap) {
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
    }

    public void forward() {
        mode = Mode.FORWARD;
        intake.forward();
        transfer.forward();
    }

    public void backward() {
        mode = Mode.BACKWARD;
        intake.backward();
        transfer.backward();
    }

    public void stop() {
        mode = Mode.STOPPED;
        intake.stop();
        transfer.stop();
    }

    /**
     * Runs intake+transfer forward to feed the shooter (the gate decides whether balls reach it). Safe to call every
     * loop. The feed lasts until the caller calls stop()/forward()/backward().
     */
    public void feed() {
        mode = Mode.FEED;
        intake.forward();
        transfer.forward();
    }

    /** True while the intake is being driven backward (outtake). */
    public boolean isReversing() { return isReversingMode(mode); }

    public boolean isFeeding() { return mode == Mode.FEED; }

    static boolean isReversingMode(Mode mode) { return mode == Mode.BACKWARD; }
}
