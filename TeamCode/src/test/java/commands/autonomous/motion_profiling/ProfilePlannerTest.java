package commands.autonomous.motion_profiling;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class ProfilePlannerTest {
    private static final double EPS = 1e-6;

    private static final double[] LONG_DISTANCES = {50, 100, 200};
    private static final double VMAX = 60, AMAX = 90, JMAX = 900;

    private static ProfilePlanner.Plan[] allPlans(double d) {
        return new ProfilePlanner.Plan[]{
                ProfilePlanner.trapezoid(d, VMAX, AMAX),
                ProfilePlanner.sCurve(d, VMAX, AMAX, JMAX)};
    }

    @Test
    public void endPositionEqualsDistance() {
        double[] out = new double[4];
        for (double d : new double[]{0.5, 3, 20, 50, 100, 200}) {
            for (ProfilePlanner.Plan plan : allPlans(d)) {
                assertEquals(d, plan.distance, 0);
                assertTrue(plan.totalTime > 0);
                // Just before the end the integrated position must already be at the distance.
                plan.sample(plan.totalTime - 1e-9, out);
                assertEquals("d=" + d, d, out[0], 1e-4);
                plan.sample(plan.totalTime, out);
                assertEquals(d, out[0], 1e-12);
                assertEquals(0.0, out[1], 1e-12);
            }
        }
    }

    @Test
    public void peakVelocityNeverExceedsVmaxAndSamplesAgree() {
        double[] out = new double[4];
        for (double d : new double[]{3, 20, 50, 100, 200}) {
            for (ProfilePlanner.Plan plan : allPlans(d)) {
                assertTrue(plan.peakVelocity <= VMAX + EPS);
                double sampledPeak = 0;
                for (int i = 0; i <= 2000; i++) {
                    plan.sample(plan.totalTime * i / 2000.0, out);
                    sampledPeak = Math.max(sampledPeak, out[1]);
                    assertTrue(out[1] >= -EPS);
                }
                assertTrue(sampledPeak <= VMAX + EPS);
                assertEquals(plan.peakVelocity, sampledPeak, 0.05 * plan.peakVelocity + EPS);
            }
        }
    }

    @Test
    public void longMoveReachesVmax() {
        for (double d : LONG_DISTANCES) {
            assertEquals(VMAX, ProfilePlanner.trapezoid(d, VMAX, AMAX).peakVelocity, EPS);
            assertEquals(VMAX, ProfilePlanner.sCurve(d, VMAX, AMAX, JMAX).peakVelocity, EPS);
        }
    }

    @Test
    public void shortMoveIsATriangleBelowVmax() {
        double d = 5;
        ProfilePlanner.Plan trap = ProfilePlanner.trapezoid(d, VMAX, AMAX);
        assertTrue(trap.peakVelocity < VMAX);
        assertEquals(Math.sqrt(AMAX * d), trap.peakVelocity, EPS);
        // Triangle: acceleration for half the time, so the peak is at the midpoint.
        double[] out = new double[4];
        trap.sample(trap.totalTime / 2 - 1e-9, out);
        assertEquals(trap.peakVelocity, out[1], 1e-3);
        ProfilePlanner.Plan s = ProfilePlanner.sCurve(d, VMAX, AMAX, JMAX);
        assertTrue(s.peakVelocity < VMAX);
        assertTrue(s.peakVelocity < trap.peakVelocity + EPS);
    }

    @Test
    public void sampleAtStartAndEnd() {
        double[] out = new double[4];
        for (ProfilePlanner.Plan plan : allPlans(100)) {
            plan.sample(0, out);
            assertEquals(0.0, out[0], 1e-12);
            assertEquals(0.0, out[1], 1e-12);
            plan.sample(-1, out);
            assertEquals(0.0, out[0], 1e-12);
            plan.sample(plan.totalTime, out);
            assertEquals(100.0, out[0], 1e-12);
            assertEquals(0.0, out[1], 1e-12);
            assertEquals(0.0, out[2], 1e-12);
            plan.sample(plan.totalTime + 5, out);
            assertEquals(100.0, out[0], 1e-12);
        }
    }

    @Test
    public void positionIsMonotone() {
        double[] out = new double[4];
        for (double d : new double[]{3, 20, 100}) {
            for (ProfilePlanner.Plan plan : allPlans(d)) {
                double prev = -1;
                for (int i = 0; i <= 3000; i++) {
                    plan.sample(plan.totalTime * i / 3000.0, out);
                    assertTrue("d=" + d + " i=" + i, out[0] >= prev - 1e-9);
                    assertTrue(out[0] <= d + 1e-6);
                    prev = out[0];
                }
            }
        }
    }

    @Test
    public void trapezoidAccelerationIsBounded() {
        double[] out = new double[4];
        ProfilePlanner.Plan plan = ProfilePlanner.trapezoid(100, VMAX, AMAX);
        for (int i = 0; i <= 1000; i++) {
            plan.sample(plan.totalTime * i / 1000.0, out);
            assertTrue(Math.abs(out[2]) <= AMAX + EPS);
        }
    }

    @Test
    public void sCurveJerkAndAccelerationAreBounded() {
        double[] out = new double[4];
        for (double d : new double[]{3, 20, 100}) {
            ProfilePlanner.Plan plan = ProfilePlanner.sCurve(d, VMAX, AMAX, JMAX);
            assertTrue(plan.peakAcceleration <= AMAX + EPS);
            for (int i = 0; i <= 3000; i++) {
                plan.sample(plan.totalTime * i / 3000.0, out);
                assertTrue("jerk " + out[3], Math.abs(out[3]) <= JMAX + EPS);
                assertTrue("accel " + out[2], Math.abs(out[2]) <= AMAX + 1e-3);
            }
        }
    }

    @Test
    public void sCurveAccelerationIsContinuous() {
        // No acceleration steps: neighbouring samples differ by at most jerk * dt (plus slack).
        double[] a = new double[4], b = new double[4];
        ProfilePlanner.Plan plan = ProfilePlanner.sCurve(100, VMAX, AMAX, JMAX);
        double dt = plan.totalTime / 5000.0;
        for (int i = 0; i < 5000; i++) {
            plan.sample(i * dt, a);
            plan.sample((i + 1) * dt, b);
            assertTrue(Math.abs(b[2] - a[2]) <= JMAX * dt * 1.01 + 1e-6);
        }
    }

    @Test
    public void sCurveIsSlowerThanTrapezoid() {
        assertTrue(ProfilePlanner.sCurve(100, VMAX, AMAX, JMAX).totalTime
                > ProfilePlanner.trapezoid(100, VMAX, AMAX).totalTime);
    }

    @Test
    public void degenerateInputsGiveEmptyPlan() {
        double[] out = new double[4];
        ProfilePlanner.Plan[] plans = {
                ProfilePlanner.trapezoid(0, VMAX, AMAX),
                ProfilePlanner.trapezoid(10, 0, AMAX),
                ProfilePlanner.sCurve(10, VMAX, AMAX, 0),
                // NaN / infinite inputs must also give an empty (finite) plan, not a NaN one.
                ProfilePlanner.trapezoid(Double.NaN, VMAX, AMAX),
                ProfilePlanner.trapezoid(10, Double.NaN, AMAX),
                ProfilePlanner.trapezoid(10, VMAX, Double.POSITIVE_INFINITY),
                ProfilePlanner.sCurve(Double.NaN, VMAX, AMAX, JMAX),
                ProfilePlanner.sCurve(10, Double.POSITIVE_INFINITY, AMAX, JMAX),
                ProfilePlanner.sCurve(10, VMAX, AMAX, Double.NaN)};
        for (ProfilePlanner.Plan p : plans) {
            assertEquals(0.0, p.totalTime, 0);
            p.sample(1, out);
            assertTrue(Double.isFinite(out[0]));
        }
    }
}
