package commands.autonomous;

/**
 * Hardware-free state machine for the autoAim autonomous command. It aligns flywheel and turret
 * and turns the transfer on while aligning. Call update() every loop with the current time, whether the
 * shooter is available, and whether the aim is ready. Time is passed in, so this is unit-testable.
 *
 * <p>Rules:
 * <ul>
 *   <li>The first update records the start time.</li>
 *   <li>If the shooter is unavailable, the command finishes immediately with transfer off, aimed=false and
 *       timedOut=false. Auto is never left hanging when there is no shooter.</li>
 *   <li>If aimReady is true, the command finishes with aimed=true.</li>
 *   <li>If the elapsed time reaches timeoutS, the command finishes with timedOut=true and aimed=false.
 *       A timeoutS &lt;= 0 (or NaN) is treated as 0 seconds, never as infinite.</li>
 *   <li>While aligning, transfer is on when transferOnStart is true. Otherwise it is on only once aimReady
 *       has been seen.</li>
 *   <li>After finishing with shootAfter=true, done stays true and the transfer state is left unchanged.
 *       After finishing with shootAfter=false, transfer turns off on the update after done was reported.</li>
 * </ul>
 */
public class AutoAimLogic {

    /** Result of one update call. */
    public static final class Output {
        /** True when the transfer should be on this loop. */
        public final boolean transferOn;
        /** True once the command has finished. */
        public final boolean done;
        /** True when the command finished because the timeout elapsed before aim was ready. */
        public final boolean timedOut;
        /** True when the command finished because aim was ready. */
        public final boolean aimed;

        /**
         * @param transferOn whether the transfer should be on
         * @param done       whether the command has finished
         * @param timedOut   whether the command finished by timeout
         * @param aimed      whether the command finished with aim ready
         */
        public Output(boolean transferOn, boolean done, boolean timedOut, boolean aimed) {
            this.transferOn = transferOn;
            this.done = done;
            this.timedOut = timedOut;
            this.aimed = aimed;
        }
    }

    private final double timeoutS;
    private final boolean shootAfter;
    private final boolean transferOnStart;

    private boolean started;
    private double startS;
    private boolean aimReadySeen;
    private boolean done;
    private boolean timedOut;
    private boolean aimed;
    private boolean doneReported;
    private boolean transferOn;

    /**
     * @param timeoutS       seconds to wait for aim before timing out; a value &lt;= 0 means time out immediately
     * @param shootAfter     if true, the transfer state is kept after done; if false, it turns off after done
     * @param transferOnStart if true, the transfer is on from the start of alignment
     */
    public AutoAimLogic(double timeoutS, boolean shootAfter, boolean transferOnStart) {
        this.timeoutS = timeoutS;
        this.shootAfter = shootAfter;
        this.transferOnStart = transferOnStart;
    }

    /**
     * Advances the state machine one loop.
     *
     * @param nowS            monotonic time in seconds
     * @param shooterAvailable whether the shooter subsystem is present and usable
     * @param aimReady        whether flywheel and turret are both aligned
     * @return the state for this loop
     */
    public Output update(double nowS, boolean shooterAvailable, boolean aimReady) {
        if (!started) {
            started = true;
            startS = nowS;
        }
        if (aimReady) {
            aimReadySeen = true;
        }

        if (done) {
            if (doneReported && !shootAfter) {
                transferOn = false;
            }
            doneReported = true;
            return snapshot();
        }

        if (!shooterAvailable) {
            done = true;
            aimed = false;
            timedOut = false;
            transferOn = false;
            doneReported = true;
            return snapshot();
        }

        if (aimReady) {
            done = true;
            aimed = true;
            timedOut = false;
            transferOn = true;
            doneReported = true;
            return snapshot();
        }

        double effectiveTimeoutS = timeoutS > 0 ? timeoutS : 0.0;
        // Negated "<" so a NaN clock fails safe (times out) instead of never finishing.
        if (!(nowS - startS < effectiveTimeoutS)) {
            done = true;
            aimed = false;
            timedOut = true;
            doneReported = true;
            return snapshot();
        }

        transferOn = transferOnStart || aimReadySeen;
        return snapshot();
    }

    /**
     * Clears all state so the command can be run again from the start.
     */
    public void reset() {
        started = false;
        startS = 0.0;
        aimReadySeen = false;
        done = false;
        timedOut = false;
        aimed = false;
        doneReported = false;
        transferOn = false;
    }

    private Output snapshot() {
        return new Output(transferOn, done, timedOut, aimed);
    }
}
