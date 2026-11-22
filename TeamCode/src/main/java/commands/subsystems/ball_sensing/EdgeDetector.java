package commands.subsystems.ball_sensing;

/** Latches rising and falling edges of a boolean until they are consumed. Starts false. */
public class EdgeDetector {
    private boolean previous = false;
    private boolean risingLatched = false;
    private boolean fallingLatched = false;

    public boolean update(boolean current) {
        if (current && !previous) {
            risingLatched = true;
        }
        if (!current && previous) {
            fallingLatched = true;
        }
        previous = current;
        return current;
    }

    /** Returns and clears the latched rising edge. */
    public boolean consumeRising() {
        boolean r = risingLatched;
        risingLatched = false;
        return r;
    }

    /** Returns and clears the latched falling edge. */
    public boolean consumeFalling() {
        boolean f = fallingLatched;
        fallingLatched = false;
        return f;
    }

    public boolean getCurrent() {
        return previous;
    }

    public void clearEvents() {
        risingLatched = false;
        fallingLatched = false;
    }

    /** Adopts the current value without producing an edge (e.g. when a sensor comes back online). */
    public void sync(boolean current) {
        previous = current;
        clearEvents();
    }

    public void reset() {
        previous = false;
        clearEvents();
    }
}
