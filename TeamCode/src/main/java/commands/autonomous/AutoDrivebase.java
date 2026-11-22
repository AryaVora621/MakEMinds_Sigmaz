package commands.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

// All five choices are imported (some unused) so swapping the profile is a one-line edit.
import commands.autonomous.motion_profiling.MotionProfile;
import commands.autonomous.motion_profiling.NoProfile;
import commands.autonomous.motion_profiling.PowerRampProfile;
import commands.autonomous.motion_profiling.ProfiledVectorCalculator;
import commands.autonomous.motion_profiling.SCurveProfile;
import commands.autonomous.motion_profiling.SlewLimitProfile;
import commands.autonomous.motion_profiling.TrapezoidProfile;
import pedroPathing.Constants;
import robot.RobotSettings;

/**
 * THE main drivebase file for autonomous. Owns the Pedro follower and the motion profile.
 *
 * To change how the robot accelerates, change the one line marked "CHOOSE THE PROFILE" below.
 * Nothing else needs to change. The choices (all in commands.autonomous.motion_profiling):
 *   new NoProfile()         - stock Pedro, no limit (base version)
 *   new PowerRampProfile()  - ceiling on power that rises with time
 *   new SlewLimitProfile()  - limits how fast the power command may rise
 *   new TrapezoidProfile()  - planned trapezoid speed curve
 *   new SCurveProfile()     - planned seven-phase S-curve
 *
 * It is also the pose source for the whole robot: call update() every loop in autonomous and
 * teleop so the shooter knows where the robot is and how fast it is moving.
 */
public class AutoDrivebase {
    // ===== CHOOSE THE PROFILE: change the class on this line =====
    private final MotionProfile profile = new NoProfile();   // simplest: stock Pedro, no motion profile
    // =============================================================

    private final Follower follower;
    private final ProfiledVectorCalculator calculator;
    private final ElapsedTime timer = new ElapsedTime();

    private double[] chainStartDistance = new double[0];
    private boolean wasBusy;
    private double lastTraveled;
    private long moveStartNs = System.nanoTime();
    private double moveTimeoutS = 0;      // <= 0 means no deadline for the current move
    private boolean timedOut;

    public AutoDrivebase(HardwareMap hardwareMap, Pose startPose) {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        calculator = new ProfiledVectorCalculator(Constants.followerConstants);
        calculator.setProfile(profile);
        follower.vectorCalculator = calculator;
        calculator.breakFollowing();
        timer.reset();
    }

    /** Builds paths with this follower. */
    public PathBuilder pathBuilder() {
        return follower.pathBuilder();
    }

    /** Follows a path chain through the motion profile. holdEnd keeps the robot at the end pose. */
    public void followPath(PathChain chain, boolean holdEnd) {
        beginMove(chain.length());
        chainStartDistance = new double[chain.size()];
        double sum = 0;
        for (int i = 0; i < chain.size(); i++) {
            chainStartDistance[i] = sum;
            sum += chain.getPath(i).length();
        }
        follower.followPath(chain, holdEnd);
    }

    /** Like {@link #followPath(PathChain, boolean)} but with a deadline: see {@link #isBusyOrTimeout()}. timeoutS <= 0 = none. */
    public void followPath(PathChain chain, boolean holdEnd, double timeoutS) {
        followPath(chain, holdEnd);
        moveTimeoutS = timeoutS;
    }

    public void followPath(PathChain chain) {
        followPath(chain, true);
    }

    /** Follows a single path through the motion profile. */
    public void followPath(Path path, boolean holdEnd) {
        beginMove(path.length());
        chainStartDistance = new double[]{0};
        follower.followPath(path, holdEnd);
    }

    /** Like {@link #followPath(Path, boolean)} but with a deadline: see {@link #isBusyOrTimeout()}. timeoutS <= 0 = none. */
    public void followPath(Path path, boolean holdEnd, double timeoutS) {
        followPath(path, holdEnd);
        moveTimeoutS = timeoutS;
    }

    /** Holds a pose (no profile: this is position hold, not travel). */
    public void holdPoint(Pose pose) {
        profile.stop();
        wasBusy = false;
        moveTimeoutS = 0;
        follower.holdPoint(pose);
    }

    /** Call once per loop, in autonomous and in teleop. */
    public void update() {
        double dt = Math.min(Math.max(timer.seconds(), 1e-3), RobotSettings.PathFollowing.MAX_DT_S);
        timer.reset();

        boolean moving = follower.isBusy() && follower.getCurrentPath() != null && !follower.isTeleopDrive();
        double speed = 0;
        double traveled = lastTraveled;
        double tangentTheta = 0;
        if (moving) {
            Vector tangent = follower.getClosestPointTangentVector();
            tangentTheta = tangent.getTheta();
            speed = follower.getTangentialVelocity();
            int index = (int) follower.getCurrentPathNumber();
            double base = index < chainStartDistance.length ? chainStartDistance[index] : 0;
            traveled = base + follower.getDistanceTraveledOnPath();
            lastTraveled = traveled;
        }
        calculator.beginLoop(moving, speed, traveled, tangentTheta, dt);

        follower.update();

        if (wasBusy && !follower.isBusy()) {
            profile.stop();
        }
        wasBusy = follower.isBusy();
    }

    /** True while a path is being followed. */
    public boolean isBusy() {
        return follower.isBusy();
    }

    /** Seconds since the current move started. */
    public double moveElapsedS() {
        return (System.nanoTime() - moveStartNs) * 1e-9;
    }

    /** True while a path is being followed and timeoutS has not passed since it started (timeoutS <= 0 = no limit). */
    public boolean isBusy(double timeoutS) {
        if (!follower.isBusy()) {
            return false;
        }
        if (timeoutS > 0 && moveElapsedS() > timeoutS) {
            timedOut = true;
            return false;
        }
        return true;
    }

    /**
     * Use as the waiting-loop condition: true while busy and the deadline given to followPath(..., timeoutS)
     * has not passed. When it returns false because of the deadline, {@link #timedOut()} is true; the robot
     * keeps following the path until you call {@link #cancelMove()}.
     */
    public boolean isBusyOrTimeout() {
        return isBusy(moveTimeoutS);
    }

    /** True if the last move ended its waiting loop because of a timeout (cleared when a new move starts). */
    public boolean timedOut() {
        return timedOut;
    }

    /** Abandons the current path and holds the current pose (use after a timeout). */
    public void cancelMove() {
        holdPoint(getPose());
    }

    public Follower getFollower() {
        return follower;
    }

    public Pose getPose() {
        return follower.getPose();
    }

    /** Field-frame velocity in inches per second. */
    public Vector getVelocity() {
        return follower.getVelocity();
    }

    /** Yaw rate in radians per second, counter-clockwise positive. */
    public double getAngularVelocity() {
        return follower.getAngularVelocity();
    }

    /** Field-frame acceleration in inches per second squared. */
    public Vector getAcceleration() {
        return follower.getAcceleration();
    }

    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    public MotionProfile getProfile() {
        return profile;
    }

    private void beginMove(double lengthIn) {
        lastTraveled = 0;
        moveStartNs = System.nanoTime();
        moveTimeoutS = 0;
        timedOut = false;
        profile.start(lengthIn);
        wasBusy = true;
    }
}
