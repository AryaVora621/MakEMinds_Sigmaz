package commands.subsystems.grouped_subsystems.shooter;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;

import org.junit.Test;

/**
 * Golden-value regression test: {@link ShotSolver} against the JavaScript reference solver.
 *
 * HOW THE NUMBERS WERE DERIVED: field_sim/web/js/shot_core.js (the simulator's solver, which sets
 * module.exports under Node) was loaded in Node v24 with require(). Each case below was fed to
 * Core.solve(p, mode, param, branch) with the same inputs as used here (mapping: x,y; psi = heading;
 * vx,vy; om = omega; rx,ry = pivot offset; z0 = exit height 14.0; X,Y,Z = target; allow_rising;
 * JS modes gamma/theta/speed/minspeed = ENTRY_ANGLE/HOOD_ANGLE/SPEED/MIN_SPEED; branch low/high = Arc).
 * The JS result fields T, speed, elev, az, turret, gamma were printed with full double precision
 * (String(number)) and pasted below. The Java solver was run on the same inputs separately and agreed
 * with the JS values to about 1e-13 (the tolerance here is 1e-9). Reason strings are not compared.
 *
 * Aim points: RED_FAR = (59.25, 88.483, 58.313) and BLUE_AUDIENCE = (84.75, 55.517, 58.313), taken from
 * FieldConstants (hive x = 72 -+ 12.75, y = 72 +- 16.483, z = 58.313). Exit height 14.0 in is an assumed
 * value, not a FieldConstants entry. C4/C5 use allowRising = true because a 300 in/s SPEED shot at this
 * distance has a rising low arc.
 */
public class ShotSolverGoldenTest {
    private static final double TOL = 1e-9;
    private static final double RED_X = 59.25, RED_FAR_Y = 88.483, BLUE_X = 84.75, AUD_Y = 55.517, Z = 58.313;

    private static ShotSolver.Result solve(double x, double y, double heading, double vx, double vy, double omega,
                                           double rx, double ry, boolean redFar, boolean allowRising,
                                           ShotSolver.Mode mode, double param, ShotSolver.Arc arc) {
        ShotSolver.Input in = new ShotSolver.Input();
        in.x = x;
        in.y = y;
        in.heading = heading;
        in.vx = vx;
        in.vy = vy;
        in.omega = omega;
        in.pivotOffsetX = rx;
        in.pivotOffsetY = ry;
        in.exitHeight = 14.0;
        in.targetX = redFar ? RED_X : BLUE_X;
        in.targetY = redFar ? RED_FAR_Y : AUD_Y;
        in.targetZ = Z;
        in.allowRising = allowRising;
        return ShotSolver.solve(in, mode, param, arc);
    }

    private static void check(ShotSolver.Result r, double flightTime, double speed, double elevation,
                              double azimuth, double turret, double entryAngle) {
        assertEquals("ok (" + r.reason + ")", true, r.ok);
        assertEquals("flightTimeS", flightTime, r.flightTimeS, TOL);
        assertEquals("launchSpeedInS", speed, r.launchSpeedInS, TOL);
        assertEquals("elevationRad", elevation, r.elevationRad, TOL);
        assertEquals("azimuthRad", azimuth, r.azimuthRad, TOL);
        assertEquals("turretRad", turret, r.turretRad, TOL);
        assertEquals("entryAngleRad", entryAngle, r.entryAngleRad, TOL);
    }

    @Test
    public void c1_stationaryMinSpeed() {
        check(solve(40, 60, 0.6, 0, 0, 0, 0, 0, true, false, ShotSolver.Mode.MIN_SPEED, 0, ShotSolver.Arc.LOW),
                0.5390058011621892, 196.88164092447832, 1.2408919830203233, 0.9764686254664231,
                0.3764686254664231, 0.329904343774573);
    }

    @Test
    public void c2_stationaryEntryAngle() {
        check(solve(30, 70, 0.2, 0, 0, 0, 0, 0, true, false, ShotSolver.Mode.ENTRY_ANGLE, 0.9, ShotSolver.Arc.LOW),
                0.6748430357910472, 202.53588308710104, 1.3148627578756795, 0.5635439007531086,
                0.3635439007531085, 0.8999999999999999);
    }

    @Test
    public void c3_stationaryHoodAngleHighArc() {
        check(solve(35, 75, -0.3, 0, 0, 0, 0, 0, true, false, ShotSolver.Mode.HOOD_ANGLE, 1.3, ShotSolver.Arc.HIGH),
                0.5368253913569518, 193.21871602808326, 1.3, 0.5074380629531406,
                0.8074380629531408, 0.3873322634067203);
    }

    @Test
    public void c4_stationarySpeedLowArc() {
        check(solve(50, 40, 1.0, 0, 0, 0, 0, 0, true, true, ShotSolver.Mode.SPEED, 300, ShotSolver.Arc.LOW),
                0.2496971427376125, 300.0, 0.8514430268290686, 1.382273440548991,
                0.3822734405489907, -0.5791413686859603);
    }

    @Test
    public void c5_stationarySpeedHighArc() {
        check(solve(50, 40, 1.0, 0, 0, 0, 0, 0, true, true, ShotSolver.Mode.SPEED, 300, ShotSolver.Arc.HIGH),
                1.3760879640218526, 299.9999999999999, 1.4509496672286777, 1.382273440548991,
                0.3822734405489907, 1.418341328218007);
    }

    @Test
    public void c6_movingEntryAngle() {
        check(solve(60, 50, 0.9, 20, -35, 0, 0, 0, false, false, ShotSolver.Mode.ENTRY_ANGLE, 0.85, ShotSolver.Arc.LOW),
                0.6156903999734613, 196.86544483508126, 1.3225063667808166, 1.1400920119058415,
                0.2400920119058414, 0.8499999999999998);
    }

    @Test
    public void c7_rotatingWithPivotOffsetHoodAngle() {
        check(solve(45, 65, -0.5, -10, 15, 0.8, 6, -4, true, false, ShotSolver.Mode.HOOD_ANGLE, 1.3, ShotSolver.Arc.LOW),
                0.4878818835681234, 192.00722009032168, 1.3, 1.0118562635352073,
                1.5118562635352077, 0.05143977646332624);
    }

    @Test
    public void c8_unreachableSpeed() {
        ShotSolver.Result r = solve(30, 70, 0.0, 0, 0, 0, 0, 0, true, false, ShotSolver.Mode.SPEED, 60, ShotSolver.Arc.LOW);
        assertFalse(r.ok);
        assertEquals(197.01552443642535, r.minSpeedInS, TOL);
    }
}
