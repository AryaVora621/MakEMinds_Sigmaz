package commands.subsystems.servo;

import static org.junit.Assert.assertEquals;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class AxonServoMathTest {
    private static final double EPS = 1e-9;

    @Test
    public void wrap360() {
        assertEquals(0, AxonServoMath.wrap360(0), EPS);
        assertEquals(0, AxonServoMath.wrap360(360), EPS);
        assertEquals(10, AxonServoMath.wrap360(370), EPS);
        assertEquals(350, AxonServoMath.wrap360(-10), EPS);
        assertEquals(180, AxonServoMath.wrap360(-180), EPS);
        assertEquals(0, AxonServoMath.wrap360(-720), EPS);
        assertEquals(359.5, AxonServoMath.wrap360(359.5), EPS);
    }

    @Test
    public void wrap180() {
        assertEquals(0, AxonServoMath.wrap180(0), EPS);
        assertEquals(180, AxonServoMath.wrap180(180), EPS);
        assertEquals(180, AxonServoMath.wrap180(-180), EPS);
        assertEquals(-170, AxonServoMath.wrap180(190), EPS);
        assertEquals(170, AxonServoMath.wrap180(-190), EPS);
        assertEquals(0, AxonServoMath.wrap180(360), EPS);
        assertEquals(-10, AxonServoMath.wrap180(350), EPS);
        assertEquals(-90, AxonServoMath.wrap180(-90), EPS);
    }

    @Test
    public void angleErrorAcrossSeam() {
        assertEquals(20, AxonServoMath.angleError(10, 350), EPS);
        assertEquals(-20, AxonServoMath.angleError(350, 10), EPS);
        assertEquals(5, AxonServoMath.angleError(50, 45), EPS);
        assertEquals(180, AxonServoMath.angleError(180, 0), EPS);
    }

    @Test
    public void voltageToDeg() {
        assertEquals(0, AxonServoMath.voltageToDeg(0, 0, 3.3, 360), EPS);
        assertEquals(360, AxonServoMath.voltageToDeg(3.3, 0, 3.3, 360), EPS);
        assertEquals(180, AxonServoMath.voltageToDeg(1.65, 0, 3.3, 360), EPS);
        assertEquals(0, AxonServoMath.voltageToDeg(-1, 0, 3.3, 360), EPS);
        assertEquals(360, AxonServoMath.voltageToDeg(5, 0, 3.3, 360), EPS);
    }

    @Test
    public void slewStep() {
        assertEquals(5, AxonServoMath.slewStep(0, 100, 5), EPS);
        assertEquals(95, AxonServoMath.slewStep(100, 0, 5), EPS);
        assertEquals(100, AxonServoMath.slewStep(98, 100, 5), EPS);
        assertEquals(0, AxonServoMath.slewStep(3, 0, 5), EPS);
        assertEquals(0, AxonServoMath.slewStep(0, 100, 0), EPS);
        assertEquals(0, AxonServoMath.slewStep(0, 100, -1), EPS);
        assertEquals(100, AxonServoMath.slewStep(Double.NaN, 100, 5), EPS);
    }

    @Test
    public void recenter() {
        // travel 255: window is [-52.5, 307.5]
        assertEquals(-0.2, AxonServoMath.recenter(359.8, 0, 255), 1e-6);
        assertEquals(100, AxonServoMath.recenter(100, 0, 255), EPS);
        assertEquals(90, AxonServoMath.recenter(100, 10, 255), EPS);
        assertEquals(-5, AxonServoMath.recenter(5, 10, 255), EPS);
        assertEquals(260, AxonServoMath.recenter(260, 0, 255), EPS);
        assertEquals(-5, AxonServoMath.recenter(355, 0, 255), EPS);
    }

    @Test
    public void capDt() {
        assertEquals(0.02, AxonServoMath.capDt(0.02, 0.1), EPS);
        assertEquals(0.1, AxonServoMath.capDt(5.0, 0.1), EPS);
        assertEquals(0, AxonServoMath.capDt(-1, 0.1), EPS);
        assertEquals(0, AxonServoMath.capDt(Double.NaN, 0.1), EPS);
        assertEquals(5.0, AxonServoMath.capDt(5.0, 0), EPS); // cap disabled
        // slew after a long pause moves at most rate * cap
        assertEquals(10, AxonServoMath.slewStep(0, 255, 100 * AxonServoMath.capDt(30, 0.1)), EPS);
    }

    @Test
    public void untrustedFeedbackFallsBackToCommand() {
        assertEquals(40, AxonServoMath.effectivePosition(false, 200, 40), EPS);
        assertEquals(40, AxonServoMath.effectivePosition(true, Double.NaN, 40), EPS);
        assertEquals(200, AxonServoMath.effectivePosition(true, 200, 40), EPS);
        assertFalse(AxonServoMath.outOfTolerance(false, 50, 50, 200, 10));
        assertTrue(AxonServoMath.outOfTolerance(true, 50, 50, 200, 10));
        assertFalse(AxonServoMath.outOfTolerance(true, 50, 50, 55, 10));
        assertFalse(AxonServoMath.outOfTolerance(true, 40, 50, 200, 10)); // still slewing
        assertFalse(AxonServoMath.outOfTolerance(true, 50, 50, Double.NaN, 10));
    }

    @Test
    public void feedbackStillTreatsNaNAsMoving() {
        assertTrue(AxonServoMath.feedbackStill(100.0, 100.5, 1.0));
        assertFalse(AxonServoMath.feedbackStill(100.0, 101.0, 1.0));
        assertTrue(AxonServoMath.feedbackStill(359.8, 0.1, 1.0));     // across the seam
        assertFalse(AxonServoMath.feedbackStill(Double.NaN, 100, 1.0)); // a NaN read must not let calibration record NaN
        assertFalse(AxonServoMath.feedbackStill(100, Double.NaN, 1.0));
    }

    @Test
    public void nanInputsDoNotPoisonMath() {
        assertTrue(Double.isNaN(AxonServoMath.voltageToDeg(Double.NaN, 0, 3.3, 360)));
        assertTrue(Double.isNaN(AxonServoMath.degToPosition(Double.NaN, 255))); // AxonServo skips writing NaN
        assertEquals(0, AxonServoMath.voltageToDeg(1.0, 3.3, 3.3, 360), EPS);   // degenerate range
        assertEquals(0, AxonServoMath.degToPosition(100, 0), EPS);
    }

    @Test
    public void degToPosition() {
        assertEquals(0.5, AxonServoMath.degToPosition(127.5, 255), EPS);
        assertEquals(0, AxonServoMath.degToPosition(-10, 255), EPS);
        assertEquals(1, AxonServoMath.degToPosition(300, 255), EPS);
        assertEquals(90, AxonServoMath.positionToDeg(AxonServoMath.degToPosition(90, 255), 255), EPS);
    }
}
