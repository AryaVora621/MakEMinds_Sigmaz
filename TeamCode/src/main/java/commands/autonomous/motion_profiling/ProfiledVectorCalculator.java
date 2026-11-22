package commands.autonomous.motion_profiling;

import com.pedropathing.VectorCalculator;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.math.Vector;

/**
 * Pedro's VectorCalculator with one change: the forward drive vector passes through a
 * {@link MotionProfile}. Everything else (translational, heading, centripetal, predictive braking)
 * is stock Pedro, so the follower still corrects error exactly as before.
 *
 * AutoDrivebase calls {@link #beginLoop} once per loop before follower.update(); the profile is
 * evaluated at most once per loop even if getDriveVector() is called several times.
 */
public class ProfiledVectorCalculator extends VectorCalculator {
    private MotionProfile profile = new NoProfile();

    private int loopId;
    private int cachedLoopId = -1;
    private double cachedMagnitude;

    private double speedAlongPath;
    private double traveled;
    private double dt = 0.02;
    private double tangentTheta;
    private boolean moving;

    public ProfiledVectorCalculator(FollowerConstants constants) {
        super(constants);
    }

    public void setProfile(MotionProfile profile) {
        this.profile = profile;
    }

    /**
     * @param moving            true while a path is being followed
     * @param speedAlongPath    robot speed along the path tangent (in/s)
     * @param traveledInMove    distance travelled along the whole move (in)
     * @param tangentTheta      direction of the path tangent (rad)
     * @param dtSeconds         seconds since the previous loop
     */
    public void beginLoop(boolean moving, double speedAlongPath, double traveledInMove,
                          double tangentTheta, double dtSeconds) {
        this.moving = moving;
        this.speedAlongPath = speedAlongPath;
        this.traveled = traveledInMove;
        this.tangentTheta = tangentTheta;
        this.dt = dtSeconds;
        loopId++;
    }

    @Override
    public Vector getDriveVector() {
        Vector stock = super.getDriveVector();
        if (!moving || stock.getMagnitude() == 0) {
            return stock;
        }
        // Vector magnitudes are never negative; a reversed vector means Pedro is braking.
        boolean forward = stock.getXComponent() * Math.cos(tangentTheta)
                + stock.getYComponent() * Math.sin(tangentTheta) >= 0;
        if (!forward) {
            return stock;
        }
        if (cachedLoopId != loopId) {
            cachedMagnitude = profile.apply(stock.getMagnitude(), speedAlongPath, traveled, dt);
            cachedLoopId = loopId;
        }
        return new Vector(Math.max(0, cachedMagnitude), stock.getTheta());
    }
}
