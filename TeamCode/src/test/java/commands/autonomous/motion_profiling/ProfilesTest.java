package commands.autonomous.motion_profiling;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import pedroPathing.Constants;

/** Basic apply() behaviour of every {@link MotionProfile}. */
public class ProfilesTest {
    private double savedFreeSpeed, savedKv;

    @Before
    public void setUp() {
        savedFreeSpeed = PlannedProfile.FREE_SPEED_IN_S;
        savedKv = PlannedProfile.K_V;
        // PlannedProfile would otherwise read pedroPathing.Constants (Pedro/FTC classes).
        PlannedProfile.FREE_SPEED_IN_S = 60;
    }

    @After
    public void tearDown() {
        PlannedProfile.FREE_SPEED_IN_S = savedFreeSpeed;
        PlannedProfile.K_V = savedKv;
    }

    private static MotionProfile[] all() {
        return new MotionProfile[]{new NoProfile(), new PowerRampProfile(), new SlewLimitProfile(),
                new TrapezoidProfile(), new SCurveProfile()};
    }

    private static void assertValid(String msg, double p) {
        assertTrue(msg + " finite: " + p, Double.isFinite(p));
        assertTrue(msg + " range: " + p, p >= -1 && p <= 1);
    }

    @Test
    public void namesAreNonEmpty() {
        for (MotionProfile m : all()) {
            assertFalse(m.name().isEmpty());
        }
    }

    @Test
    public void noProfilePassesStockPowerThrough() {
        NoProfile p = new NoProfile();
        p.start(100);
        assertEquals(1.0, p.apply(1.0, 0, 0, 0.02), 0);
        assertEquals(0.4, p.apply(0.4, 10, 5, 0.02), 0);
        assertEquals(-0.7, p.apply(-0.7, 10, 5, 0.02), 0);
    }

    @Test
    public void outputsAreFiniteAndInRangeThroughAWholeMove() {
        for (MotionProfile m : all()) {
            m.start(80);
            double traveled = 0;
            for (int i = 0; i < 400; i++) {
                double speed = Math.min(60, i * 1.5);
                double out = m.apply(1.0, speed, traveled, 0.02);
                assertValid(m.name(), out);
                traveled += speed * 0.02;
            }
            m.stop();
        }
    }

    @Test
    public void neverRaisesPowerAboveStock() {
        for (MotionProfile m : all()) {
            m.start(80);
            for (int i = 0; i < 200; i++) {
                double stock = 0.3;
                assertTrue(m.name(), m.apply(stock, 10, 5, 0.02) <= stock + 1e-12);
            }
        }
    }

    @Test
    public void brakingIsPassedThroughUntouched() {
        for (MotionProfile m : all()) {
            m.start(80);
            assertEquals(m.name(), -0.5, m.apply(-0.5, 20, 40, 0.02), 0);
            assertEquals(m.name(), -1.0, m.apply(-1.0, 20, 40, 0.02), 0);
        }
    }

    @Test
    public void powerRampStartsAtStartPowerAndRises() {
        PowerRampProfile p = new PowerRampProfile();
        p.start(100);
        double first = p.apply(1.0, 0, 0, 0.02);
        assertEquals(PowerRampProfile.START_POWER + PowerRampProfile.RAMP_PER_S * 0.02, first, 1e-12);
        double later = first;
        for (int i = 0; i < 100; i++) {
            later = p.apply(1.0, 0, 0, 0.02);
        }
        assertTrue(later > first);
        assertEquals(1.0, later, 1e-12);       // capped at 1
        p.start(100);                           // restart resets the ramp
        assertEquals(first, p.apply(1.0, 0, 0, 0.02), 1e-12);
    }

    @Test
    public void slewLimitRisesAtMostRisePerSecond() {
        SlewLimitProfile p = new SlewLimitProfile();
        p.start(100);
        double prev = SlewLimitProfile.START_POWER;
        for (int i = 0; i < 100; i++) {
            double out = p.apply(1.0, 0, 0, 0.02);
            assertTrue(out - prev <= SlewLimitProfile.RISE_PER_S * 0.02 + 1e-12);
            assertTrue(out >= prev - 1e-12);
            prev = out;
        }
        assertEquals(1.0, prev, 1e-12);
        // A drop in the request is followed immediately.
        assertEquals(0.1, p.apply(0.1, 0, 0, 0.02), 1e-12);
    }

    @Test
    public void plannedProfilesFollowTheReferenceAndHandOverAtTheEnd() {
        PlannedProfile[] ps = {new TrapezoidProfile(), new SCurveProfile()};
        for (PlannedProfile p : ps) {
            assertTrue(Double.isNaN(p.plannedTime()));
            p.start(60);
            assertTrue(p.plannedTime() > 0);
            // After the planned time has elapsed, the stock power is returned.
            double out = 0;
            for (int i = 0; i < 1000; i++) {
                out = p.apply(0.8, 30, 60, 0.02);
            }
            assertEquals(p.name(), 0.8, out, 0);
            p.stop();
            assertTrue(Double.isNaN(p.plannedTime()));
            assertEquals(0.6, p.apply(0.6, 0, 0, 0.02), 0);   // no plan: stock power
        }
    }

    @Test
    public void plannedProfileReferenceStartsAtZero() {
        TrapezoidProfile p = new TrapezoidProfile();
        p.start(60);
        double out = p.apply(1.0, 0, 0, 0.02);
        assertValid("first", out);
        assertTrue(p.referencePosition() >= 0 && p.referencePosition() < 1);
        assertTrue(p.referenceVelocity() > 0);
    }

    @Test
    public void shortMoveStillProducesValidOutput() {
        for (MotionProfile m : new MotionProfile[]{new TrapezoidProfile(), new SCurveProfile()}) {
            m.start(2);
            for (int i = 0; i < 100; i++) {
                assertValid(m.name(), m.apply(1.0, 5, 0, 0.02));
            }
        }
    }

    @Test
    public void zeroOrNegativeDtNeverGivesNaN() {
        for (MotionProfile m : all()) {
            m.start(80);
            assertValid(m.name(), m.apply(1.0, 10, 5, 0));
            assertValid(m.name(), m.apply(1.0, 10, 5, -0.02));
        }
    }

    /** NaN / infinite / negative dt must not poison the ramp or slew state: later good loops behave normally. */
    @Test
    public void badDtDoesNotPoisonRampOrSlewState() {
        for (double bad : new double[]{Double.NaN, Double.POSITIVE_INFINITY, -0.5}) {
            PowerRampProfile r = new PowerRampProfile();
            r.start(100);
            assertValid("ramp dt=" + bad, r.apply(1.0, 0, 0, bad));
            assertEquals("ramp dt=" + bad, PowerRampProfile.START_POWER + PowerRampProfile.RAMP_PER_S * 0.02,
                    r.apply(1.0, 0, 0, 0.02), 1e-12);

            SlewLimitProfile s = new SlewLimitProfile();
            s.start(100);
            double held = s.apply(1.0, 0, 0, bad);
            assertValid("slew dt=" + bad, held);
            assertTrue("slew dt=" + bad + " gave " + held, held >= 0);
            assertEquals("slew dt=" + bad, SlewLimitProfile.START_POWER, held, 1e-12);
            assertEquals("slew dt=" + bad, SlewLimitProfile.START_POWER + SlewLimitProfile.RISE_PER_S * 0.02,
                    s.apply(1.0, 0, 0, 0.02), 1e-12);
        }
    }

    /**
     * Guard: when no free speed is available (FREE_SPEED_IN_S = 0, K_V = 0 and Pedro's measured xVelocity is
     * zero, negative, NaN or infinite) the profile is effectively off and the stock power passes through unchanged.
     */
    @Test
    public void invalidFreeSpeedPassesStockPowerThroughExactly() {
        PlannedProfile.FREE_SPEED_IN_S = 0;
        PlannedProfile.K_V = 0;
        double saved = Constants.driveConstants.getXVelocity();
        try {
            for (double bad : new double[]{0, -5, Double.NaN, Double.POSITIVE_INFINITY}) {
                Constants.driveConstants.xVelocity(bad);
                for (PlannedProfile p : new PlannedProfile[]{new TrapezoidProfile(), new SCurveProfile()}) {
                    p.start(80);
                    for (double stock : new double[]{1.0, 0.3}) {
                        for (int i = 0; i < 50; i++) {
                            assertEquals(p.name() + " xVelocity=" + bad + " stock=" + stock,
                                    stock, p.apply(stock, 10, 5, 0.02), 0);
                        }
                    }
                }
            }
            // Control: with a valid measured speed the same profile does shape the output (so the test above can fail).
            Constants.driveConstants.xVelocity(60);
            TrapezoidProfile t = new TrapezoidProfile();
            t.start(80);
            assertTrue(t.apply(1.0, 0, 0, 0.02) < 1.0);
        } finally {
            Constants.driveConstants.xVelocity(saved);
        }
    }

    /** A zero, negative, NaN or infinite time step switches a planned profile off for that loop: stock power exactly. */
    @Test
    public void badTimeStepPassesStockPowerThroughForPlannedProfiles() {
        for (PlannedProfile p : new PlannedProfile[]{new TrapezoidProfile(), new SCurveProfile()}) {
            p.start(80);
            for (double dt : new double[]{0, -0.02, Double.NaN, Double.POSITIVE_INFINITY}) {
                assertEquals(p.name() + " dt=" + dt, 0.7, p.apply(0.7, 10, 5, dt), 0);
            }
        }
    }
}
