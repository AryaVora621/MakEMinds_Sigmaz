package commands.subsystems.ball_sensing;

/**
 * Time-based debouncer: the output only changes after the raw input has held the new value for
 * riseMs (false -> true) or fallMs (true -> false). Starts false.
 */
public class Debouncer {
    private long riseMs;
    private long fallMs;
    private boolean state = false;
    /** True while the raw input differs from the output and the hold timer is running. */
    private boolean pending = false;
    private long pendingSinceMs = 0L;

    public Debouncer(long riseMs, long fallMs) {
        this.riseMs = riseMs;
        this.fallMs = fallMs;
    }

    public void setTimes(long riseMs, long fallMs) {
        this.riseMs = riseMs;
        this.fallMs = fallMs;
    }

    public boolean update(boolean raw, long nowMs) {
        if (raw == state) {
            pending = false;   // input agrees with output again: cancel any pending change
            return state;
        }
        // Start the hold timer, or restart it if the clock went backwards.
        if (!pending || nowMs < pendingSinceMs) {
            pending = true;
            pendingSinceMs = nowMs;
        }
        long needed = raw ? riseMs : fallMs;
        if (nowMs - pendingSinceMs >= needed) {
            state = raw;
            pending = false;
        }
        return state;
    }

    public boolean get() {
        return state;
    }

    public void reset() {
        state = false;
        pending = false;
    }
}
