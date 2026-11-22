package robot;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;

import com.pedropathing.geometry.Pose;

import org.junit.After;
import org.junit.Test;

/** Hardware-free helpers of {@link Robot}: acceleration filter and the auto -> teleop saved pose. */
public class RobotHelpersTest {
    @After
    public void clear() {
        Robot.takeSavedPose();
    }

    @Test
    public void accelFilterSmoothsFiniteSamples() {
        assertEquals(2.5, Robot.filterAccel(0, 3, 4, 0.5), 1e-12);   // |(3,4)| = 5, half way from 0
        assertEquals(5, Robot.filterAccel(5, 3, 4, 0.25), 1e-12);
    }

    @Test
    public void nonFiniteSampleDoesNotLatchTheFilter() {
        // A single NaN / infinite follower acceleration used to make the filter NaN forever (readyToFire never true).
        assertEquals(7, Robot.filterAccel(7, Double.NaN, 0, 0.25), 0);
        assertEquals(7, Robot.filterAccel(7, Double.POSITIVE_INFINITY, 0, 0.25), 0);
        double f = Robot.filterAccel(Double.NaN, 3, 4, 0.25);
        assertEquals(5, f, 0);                                          // recovers from a poisoned previous value
        assertEquals(0, Robot.filterAccel(Double.NaN, Double.NaN, 1, 0.25), 0);
    }

    @Test
    public void savedPoseIsUsedOnceAndRejectsNonFinite() {
        Pose p = new Pose(10, 20, 1);
        Robot.setSavedPoseForTest(p);
        assertSame(p, Robot.takeSavedPose());
        assertNull("saved pose must be consumed", Robot.takeSavedPose());

        Robot.setSavedPoseForTest(new Pose(Double.NaN, 20, 1));
        assertNull(Robot.takeSavedPose());
        Robot.setSavedPoseForTest(new Pose(10, 20, Double.POSITIVE_INFINITY));
        assertNull(Robot.takeSavedPose());
        Robot.setSavedPoseForTest(null);
        assertNull(Robot.takeSavedPose());
    }
}
