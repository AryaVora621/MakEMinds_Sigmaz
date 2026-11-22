package commands.subsystems.ball_sensing;

/** Ball count clamped to [0, max]. A negative max is treated as 0. */
public class BallCounter {
    private int max;
    private int count = 0;

    public BallCounter(int max) {
        this.max = Math.max(0, max);
    }

    public int getCount() {
        return count;
    }

    public int getMax() {
        return max;
    }

    /** Changes the capacity and re-clamps the current count into it. */
    public void setMax(int max) {
        this.max = Math.max(0, max);
        count = clamp(count);
    }

    public void increment() {
        set(count + 1);
    }

    public void decrement() {
        set(count - 1);
    }

    public void set(int value) {
        count = clamp(value);
    }

    public void reset() {
        count = 0;
    }

    public boolean isFull() {
        return count >= max;
    }

    private int clamp(int v) {
        return Math.max(0, Math.min(max, v));
    }
}
