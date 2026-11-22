package commands.subsystems.sorting;

import commands.subsystems.ball_sensing.BallColor;

/**
 * Hardware-free decision logic for the lane sorter (one colour sensor on the left lane wall plus the lane gate).
 *
 * Gate OPEN = the ball at the sensor is let through to the shooter. Gate CLOSED = it is held back.
 * A ball is let through when its colour is the colour the pattern wants next (or it is UNKNOWN and
 * passUnknown is set, so a misread never jams the lane). When the ball has been gone for clearMs the gate
 * returns to CLOSED once, and, if the ball was let through, the pattern advances to the next colour.
 *
 * The pattern is a string of 'P' (pollen) and 'N' (nectar); it repeats. Anything else is ignored; a pattern
 * with no usable letters lets every ball through.
 */
public final class SortLogic {
    public enum Decision { NONE, OPEN, CLOSE }
    /** Where the shooter's rack carriage must sit for a ball: rear for pollen, forward for nectar. */
    public enum Carriage { NONE, REAR, FORWARD }

    public static Carriage carriageFor(BallColor c) {
        if (c == BallColor.POLLEN) return Carriage.REAR;
        if (c == BallColor.NECTAR) return Carriage.FORWARD;
        return Carriage.NONE;
    }

    private boolean seen = false;
    private boolean passing = false;
    private boolean idleClosed = true;
    private long lastPresentMs = 0;
    private int index = 0;
    private Decision prevDecision = Decision.NONE;
    private boolean changed = false;
    private boolean cleared = false;

    /** Colour wanted at position idx of the pattern, or UNKNOWN when the pattern has no usable letters. */
    public static BallColor wantedAt(String pattern, int idx) {
        if (pattern == null) return BallColor.UNKNOWN;
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < pattern.length(); i++) {
            char c = Character.toUpperCase(pattern.charAt(i));
            if (c == 'P' || c == 'N') sb.append(c);
        }
        if (sb.length() == 0) return BallColor.UNKNOWN;
        char c = sb.charAt(Math.floorMod(idx, sb.length()));
        return c == 'P' ? BallColor.POLLEN : BallColor.NECTAR;
    }

    /**
     * @param valid       the sensor is working; when false the sorter lets go (returns NONE) so manual control works
     * @param present     a ball is at the sensor (already debounced)
     * @param color       classified colour of that ball (POLLEN, NECTAR, UNKNOWN or NONE)
     * @param pattern     wanted colour order
     * @param passUnknown true = let an UNKNOWN ball through, false = hold it
     * @param clearMs     how long the ball must be gone before the gate resets
     */
    public Decision update(boolean valid, boolean present, BallColor color, String pattern,
                           boolean passUnknown, long nowMs, long clearMs) {
        cleared = false;
        Decision d = decide(valid, present, color, pattern, passUnknown, nowMs, clearMs);
        changed = d != prevDecision || cleared;
        prevDecision = d;
        return d;
    }

    /**
     * True when the last update's decision should be sent to the gate: it differs from the one before, or a
     * ball just cleared. Repeats are not re-sent, so a manual gate toggle is not overridden every loop.
     */
    public boolean isNewCommand() { return changed && prevDecision != Decision.NONE; }

    /** Forget the last sent command (e.g. the sorter was switched off) so the next decision is sent again. */
    public void forgetCommand() {
        prevDecision = Decision.NONE;
        changed = false;
    }

    private Decision decide(boolean valid, boolean present, BallColor color, String pattern,
                            boolean passUnknown, long nowMs, long clearMs) {
        if (!valid) {
            // A ball already let through goes on through the open gate: count it so the pattern does not drift.
            if (seen && passing) index++;
            seen = false;
            passing = false;
            idleClosed = false;   // re-assert CLOSE when the sensor comes back
            return Decision.NONE;
        }
        if (present) {
            BallColor wanted = wantedAt(pattern, index);
            boolean pass = wanted == BallColor.UNKNOWN   // no usable pattern: do not sort, never jam
                    || color == wanted
                    || (color == BallColor.UNKNOWN && passUnknown);
            // Once a ball is let through it stays let through: a mid-pass colour flicker must not slam the
            // gate on it or lose its pattern step. A held ball may still be re-read as the wanted colour.
            passing = (seen && passing) || pass;
            seen = true;
            idleClosed = false;
            lastPresentMs = nowMs;
            return passing ? Decision.OPEN : Decision.CLOSE;
        }
        if (seen) {
            if (nowMs - lastPresentMs >= clearMs) {
                if (passing) index++;
                seen = false;
                passing = false;
                idleClosed = true;
                cleared = true;
                return Decision.CLOSE;
            }
            return passing ? Decision.OPEN : Decision.CLOSE;
        }
        if (!idleClosed) {
            idleClosed = true;
            return Decision.CLOSE;
        }
        return Decision.NONE;
    }

    /** Position in the pattern (number of balls let through so far). */
    public int getIndex() { return index; }

    /** Starts the pattern again from its first colour. */
    public void reset() {
        index = 0;
        seen = false;
        passing = false;
        idleClosed = false;
        forgetCommand();
    }
}
