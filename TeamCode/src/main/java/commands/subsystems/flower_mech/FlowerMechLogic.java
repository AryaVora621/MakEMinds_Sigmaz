package commands.subsystems.flower_mech;

import robot.RobotSettings;

/**
 * Hardware-free state machine for FlowerMech: deploy, dwell, stow, cancel, fault and grace.
 * Time comes in as dt (seconds), so it is fully unit testable. Progress is 0 = stow, 1 = deploy.
 */
public class FlowerMechLogic {
    public enum State { STOWED, DEPLOYING, DWELL, STOWING, FAULTED }

    // ---- Tunables: FlowerMech copies these from RobotSettings.FlowerMech every update; the
    //      initial values below are the same settings (edit them in RobotSettings, not here). ----
    public double slewDegPerS = RobotSettings.FlowerMech.SLEW_DEG_PER_S;
    public double spanDeg = RobotSettings.FlowerMech.A_DEPLOY_DEG - RobotSettings.FlowerMech.A_STOW_DEG;
    public double dwellMs = RobotSettings.FlowerMech.DWELL_MS;
    public double syncTolDeg = RobotSettings.FlowerMech.SYNC_TOL_DEG;
    public double syncFaultMs = RobotSettings.FlowerMech.SYNC_FAULT_MS;
    public double faultGraceS = RobotSettings.FlowerMech.FAULT_GRACE_S;
    public double maxDtS = RobotSettings.FlowerMech.MAX_DT_S;
    /**
     * With trusted feedback, once the command has arrived the servos must report atTarget within this
     * time; after it the move counts as arrived anyway (open-loop fallback). Without it an error between
     * the at-target tolerance and the servo stall tolerance would leave the state machine stuck. <= 0 = wait forever.
     */
    public double arriveTimeoutMs = RobotSettings.FlowerMech.ARRIVE_TIMEOUT_MS;

    private State state = State.STOWED;
    private double goal = 0;
    private double progress = 0;
    private double dwellS = 0;
    private double syncBadS = 0;
    private boolean syncBad = false;
    private double graceLeftS = 0;
    /** Time the command has been at the goal while waiting for atTarget (trusted feedback only). */
    private double arrivedWaitS = 0;
    /** True after a state change until the next update: stall/sync inputs may be stale then, so they are ignored once. */
    private boolean settle = false;

    public State getState() { return state; }
    public double getProgress() { return progress; }
    public double getGoal() { return goal; }
    public boolean inGrace() { return graceLeftS > 0; }

    public void deploy() {
        if (state != State.STOWED) return;
        goal = 1;
        settle = true;
        arrivedWaitS = 0;
        state = State.DEPLOYING;
    }

    /** Also cancels a deploy or dwell in progress. */
    public void stow() {
        if (state == State.FAULTED || state == State.STOWED) return;
        goal = 0;
        settle = true;
        arrivedWaitS = 0;
        state = State.STOWING;
    }

    public void toggle() {
        if (state == State.STOWED) deploy(); else stow();
    }

    /** Clears a fault, heads to stow, and blocks re-latching for faultGraceS. */
    public void clearFault() {
        if (state != State.FAULTED) return;
        syncBad = false;
        syncBadS = 0;
        graceLeftS = faultGraceS;
        goal = 0;
        settle = true;
        arrivedWaitS = 0;
        state = State.STOWING;
    }

    public boolean isMoving() {
        return state == State.DEPLOYING || state == State.STOWING;
    }

    public double clampDt(double dt) {
        if (!(dt > 0)) return 0;
        return Math.min(dt, maxDtS);
    }

    /**
     * @param rawDt        seconds since last update (capped here)
     * @param trusted      both servos have trusted feedback
     * @param atTarget     both servos are at their commanded target (ignored if not trusted)
     * @param stalled      either servo reports a stall (ignored if not trusted, and for the first update after a state change,
     *                     since the flag may be left over from the previous state)
     * @param syncDiffDeg  A/B disagreement in A degrees, NaN if unavailable
     * @return progress to command, or NaN if the servos must keep their last command (faulted)
     */
    public double update(double rawDt, boolean trusted, boolean atTarget, boolean stalled, double syncDiffDeg) {
        double dt = clampDt(rawDt);
        if (graceLeftS > 0) graceLeftS = Math.max(0, graceLeftS - dt);
        if (state == State.FAULTED) return Double.NaN;

        boolean settling = settle;
        settle = false;
        if (isMoving() && trusted && graceLeftS <= 0 && !settling) {
            boolean fault = stalled;
            if (!fault && !Double.isNaN(syncDiffDeg) && syncDiffDeg > syncTolDeg) {
                if (!syncBad) {
                    syncBad = true;
                    syncBadS = 0;
                }
                syncBadS += dt;
                fault = syncBadS * 1000.0 > syncFaultMs;
            } else if (!fault) {
                syncBad = false;
            }
            if (fault) {
                state = State.FAULTED;
                return Double.NaN;
            }
        } else {
            syncBad = false;
        }

        if (slewDegPerS <= 0) {
            progress = goal;
        } else {
            double step = slewDegPerS * dt / Math.max(1e-6, Math.abs(spanDeg));
            double diff = goal - progress;
            progress = Math.abs(diff) <= step ? goal : progress + Math.signum(diff) * step;
        }

        boolean commandArrived = progress == goal;
        if (isMoving() && commandArrived && trusted && !atTarget) {
            arrivedWaitS += dt;
        } else {
            arrivedWaitS = 0;
        }
        boolean arriveTimedOut = arriveTimeoutMs > 0 && arrivedWaitS * 1000.0 >= arriveTimeoutMs;
        boolean reached = commandArrived && (!trusted || atTarget || arriveTimedOut);
        switch (state) {
            case DEPLOYING:
                if (reached) {
                    state = State.DWELL;
                    dwellS = 0;
                    arrivedWaitS = 0;
                }
                break;
            case DWELL:
                dwellS += dt;
                if (dwellS * 1000.0 >= dwellMs) {
                    goal = 0;
                    settle = true;   // the stall flag still describes the deployed hold: ignore it once
                    state = State.STOWING;
                }
                break;
            case STOWING:
                if (reached) {
                    state = State.STOWED;
                    arrivedWaitS = 0;
                }
                break;
            default:
                break;
        }
        return progress;
    }

    /** Fully deployed: dwelling, or deploying and arrived. */
    public boolean isDeployed(boolean trusted, boolean atTarget) {
        return state == State.DWELL
                || (state == State.DEPLOYING && progress == goal && (!trusted || atTarget));
    }
}
