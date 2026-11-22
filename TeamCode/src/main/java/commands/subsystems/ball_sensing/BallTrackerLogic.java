package commands.subsystems.ball_sensing;

/**
 * Hardware-free counting rules for BallTracker.
 *
 * Intake sensor: a rising edge (ball arrives) counts +1 while intaking; while the intake is reversing a
 * rising edge is ignored and a falling edge (ball leaves outward) counts -1, so a ball pushed back out
 * past the sensor is uncounted exactly once.
 * Shooter sensor: a falling edge (ball leaves toward the flywheel) is a shot, -1, unless reversing.
 * A sensor that just came back online is re-synced without producing an edge. The very first time the
 * shooter sensor is live, a ball already sitting there means the count is at least 1.
 * Pending events are capped at max + 2 so an unconsumed queue cannot grow without bound.
 */
class BallTrackerLogic {
    private final BallCounter counter;
    private final EdgeDetector intakeEdge = new EdgeDetector();
    private final EdgeDetector shooterEdge = new EdgeDetector();
    private boolean useIntake = true;
    private boolean useShooter = true;
    private boolean intakePresent = false;
    private boolean shooterPresent = false;
    /** The sensor has been live at least once. */
    private boolean intakeSeen = false;
    private boolean shooterSeen = false;
    /** The sensor was live on the previous update. */
    private boolean intakeWasLive = false;
    private boolean shooterWasLive = false;
    private int intakeEvents = 0;
    private int shotEvents = 0;

    BallTrackerLogic(int maxBalls) {
        counter = new BallCounter(maxBalls);
    }

    void setUse(boolean useIntake, boolean useShooter) {
        this.useIntake = useIntake;
        this.useShooter = useShooter;
    }

    void setMax(int max) {
        counter.setMax(max);
    }

    /** Both sensors valid, not reversing (test convenience). */
    void update(boolean intakeIn, boolean shooterIn) {
        update(intakeIn, true, shooterIn, true, false);
    }

    void update(boolean intakeIn, boolean intakeValid, boolean shooterIn, boolean shooterValid, boolean intakeReversing) {
        boolean intakeLive = useIntake && intakeValid;
        boolean shooterLive = useShooter && shooterValid;
        intakePresent = intakeLive && intakeIn;
        shooterPresent = shooterLive && shooterIn;
        int eventCap = counter.getMax() + 2;

        if (intakeLive) {
            if (intakeSeen && !intakeWasLive) {
                intakeEdge.sync(intakePresent);   // back online: no edge
            } else {
                // Also on the very first live update: a ball already at the intake counts.
                intakeEdge.update(intakePresent);
            }
            intakeSeen = true;
            boolean rising = intakeEdge.consumeRising();
            boolean falling = intakeEdge.consumeFalling();
            if (rising && !intakeReversing) {
                counter.increment();
                intakeEvents = Math.min(intakeEvents + 1, eventCap);
            } else if (falling && intakeReversing) {
                counter.decrement();
            }
        }
        intakeWasLive = intakeLive;

        boolean shot = false;
        if (shooterLive) {
            if (!shooterSeen) {
                shooterEdge.sync(shooterPresent);
                if (shooterPresent && counter.getCount() < 1) {
                    counter.set(1);   // a ball is visibly waiting at the shooter
                }
            } else if (!shooterWasLive) {
                shooterEdge.sync(shooterPresent);   // back online: no edge
            } else {
                shooterEdge.update(shooterPresent);
            }
            shooterSeen = true;
            shot = shooterEdge.consumeFalling() && !intakeReversing;
            shooterEdge.consumeRising();   // arrivals at the shooter are not counted (the intake already did)
        }
        shooterWasLive = shooterLive;

        if (shot) {
            counter.decrement();
            shotEvents = Math.min(shotEvents + 1, eventCap);
        }
    }

    boolean consumeIntakeEvent() {
        if (intakeEvents <= 0) {
            return false;
        }
        intakeEvents--;
        return true;
    }

    boolean consumeShotEvent() {
        if (shotEvents <= 0) {
            return false;
        }
        shotEvents--;
        return true;
    }

    boolean ballAtIntake() {
        return intakePresent;
    }

    boolean ballAtShooter() {
        return shooterPresent;
    }

    int getCount() {
        return counter.getCount();
    }

    boolean isFull() {
        return counter.isFull();
    }

    /** Sets the count; never below 1 while a ball is at the shooter sensor. */
    void setCount(int n) {
        counter.set(n);
        if (n < 1 && shooterPresent) {
            counter.set(1);
        }
    }

    void clearEvents() {
        intakeEvents = 0;
        shotEvents = 0;
    }

    /** Count back to 0 (1 if a ball is at the shooter sensor) and drop pending events. */
    void reset() {
        counter.reset();
        if (shooterPresent) {
            counter.set(1);
        }
        clearEvents();
    }
}
