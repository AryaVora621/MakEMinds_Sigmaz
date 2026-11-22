package commands.subsystems.flower_mech;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import commands.subsystems.servo.AxonServo;
import robot.RobotSettings;

/**
 * Arm that comes down to empty the flowers, on one Axon servo (RobotSettings.FlowerMech.SERVO_COUNT = 1, the current robot) or
 * two in sync (B mirrored). deploy() -> DEPLOYING -> DWELL -> STOWING -> STOWED automatically. Call update() every loop.
 * All servos are commanded with the same progress fraction (0 = stow, 1 = deploy).
 * Decisions live in FlowerMechLogic (hardware-free, unit tested). Feedback faults only run while
 * moving with trusted feedback on every servo; otherwise it runs open-loop on the slew and dwell timers
 * (the physical servo is assumed to keep up with RobotSettings.FlowerMech.SLEW_DEG_PER_S). The A/B sync check needs two servos.
 */
public class FlowerMech {
    private final AxonServo a, b;
    private final FlowerMechLogic logic = new FlowerMechLogic();
    private final ElapsedTime dtTimer = new ElapsedTime();

    public FlowerMech(HardwareMap hw) {
        if (!RobotSettings.FlowerMech.ENABLED) {
            a = null;
            b = null;
            return;
        }
        a = new AxonServo(hw, RobotSettings.FlowerMech.SERVO_A_NAME, RobotSettings.FlowerMech.FEEDBACK_A_NAME);
        if (RobotSettings.FlowerMech.SERVO_COUNT >= 2) {
            b = new AxonServo(hw, RobotSettings.FlowerMech.SERVO_B_NAME, RobotSettings.FlowerMech.FEEDBACK_B_NAME);
            b.setReversed(RobotSettings.FlowerMech.SERVO_B_REVERSED);
        } else {
            b = null;
        }
        command(0);
    }

    public void deploy() {
        if (active()) logic.deploy();
    }

    /** Stows, also cancelling a deploy or dwell in progress. */
    public void stow() {
        if (active()) logic.stow();
    }

    public void toggle() {
        if (logic.getState() == FlowerMechLogic.State.STOWED) deploy(); else stow();
    }

    /** Clears a fault and heads back to stow; no re-fault during RobotSettings.FlowerMech.FAULT_GRACE_S. */
    public void clearFault() {
        if (!active()) return;
        applyTunables();
        logic.clearFault();
    }

    public void update() {
        if (!active()) return;
        double dt = Math.min(dtTimer.seconds(), RobotSettings.FlowerMech.MAX_DT_S);
        dtTimer.reset();
        a.update();
        if (b != null) b.update();
        applyTunables();

        double p = logic.update(dt, trusted(), atTarget(), stalled(), syncDiff());
        // NaN = faulted: servos keep their own last commanded target (no measured-angle hold)
        if (!Double.isNaN(p)) command(p);
    }

    public boolean isDeployed() {
        return active() && logic.isDeployed(trusted(), atTarget());
    }

    /** True while moving, dwelling or stowing. */
    public boolean isBusy() {
        return active() && logic.getState() != FlowerMechLogic.State.STOWED
                && logic.getState() != FlowerMechLogic.State.FAULTED;
    }

    public boolean isFaulted() {
        return active() && logic.getState() == FlowerMechLogic.State.FAULTED;
    }

    public String getState() {
        return active() ? logic.getState().name() : "DISABLED";
    }

    /** RobotSettings.FlowerMech.ENABLED can be changed at runtime (code or tests); the servos only exist if it was true at construction. */
    private boolean active() {
        return RobotSettings.FlowerMech.ENABLED && a != null;
    }

    private void applyTunables() {
        logic.slewDegPerS = RobotSettings.FlowerMech.SLEW_DEG_PER_S;
        logic.spanDeg = RobotSettings.FlowerMech.A_DEPLOY_DEG - RobotSettings.FlowerMech.A_STOW_DEG;
        logic.dwellMs = RobotSettings.FlowerMech.DWELL_MS;
        logic.syncTolDeg = RobotSettings.FlowerMech.SYNC_TOL_DEG;
        logic.syncFaultMs = RobotSettings.FlowerMech.SYNC_FAULT_MS;
        logic.faultGraceS = RobotSettings.FlowerMech.FAULT_GRACE_S;
        logic.maxDtS = RobotSettings.FlowerMech.MAX_DT_S;
        logic.arriveTimeoutMs = RobotSettings.FlowerMech.ARRIVE_TIMEOUT_MS;
    }

    private void command(double p) {
        a.setTargetDeg(RobotSettings.FlowerMech.A_STOW_DEG + p * (RobotSettings.FlowerMech.A_DEPLOY_DEG - RobotSettings.FlowerMech.A_STOW_DEG));
        if (b != null) b.setTargetDeg(RobotSettings.FlowerMech.B_STOW_DEG + p * (RobotSettings.FlowerMech.B_DEPLOY_DEG - RobotSettings.FlowerMech.B_STOW_DEG));
    }

    private boolean trusted() {
        return a.feedbackTrusted() && (b == null || b.feedbackTrusted());
    }

    private boolean atTarget() {
        return a.atTarget(RobotSettings.FlowerMech.AT_TARGET_TOL_DEG) && (b == null || b.atTarget(RobotSettings.FlowerMech.AT_TARGET_TOL_DEG));
    }

    private boolean stalled() {
        return a.isStalled() || (b != null && b.isStalled());
    }

    /** A/B disagreement in A degrees, NaN when not measurable. */
    private double syncDiff() {
        if (b == null || !trusted()) return Double.NaN;
        if (RobotSettings.FlowerMech.A_DEPLOY_DEG == RobotSettings.FlowerMech.A_STOW_DEG || RobotSettings.FlowerMech.B_DEPLOY_DEG == RobotSettings.FlowerMech.B_STOW_DEG) return Double.NaN; // no span: not measurable
        double fa = (a.getPositionDeg() - RobotSettings.FlowerMech.A_STOW_DEG) / (RobotSettings.FlowerMech.A_DEPLOY_DEG - RobotSettings.FlowerMech.A_STOW_DEG);
        double fb = (b.getPositionDeg() - RobotSettings.FlowerMech.B_STOW_DEG) / (RobotSettings.FlowerMech.B_DEPLOY_DEG - RobotSettings.FlowerMech.B_STOW_DEG);
        return Math.abs(fa - fb) * Math.abs(RobotSettings.FlowerMech.A_DEPLOY_DEG - RobotSettings.FlowerMech.A_STOW_DEG);
    }
}
