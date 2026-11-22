package commands.subsystems.ball_sensing;

import java.util.Random;
import org.junit.Assert;
import org.junit.Test;

public class BallTrackerLogicTest {
   public BallTrackerLogicTest() {
   }

   @Test
   public void reversingOutDecrementsAndIsNotAShot() {
      BallTrackerLogic l = new BallTrackerLogic(4);
      l.setCount(2);
      l.update(false, true, false, true, true);
      l.update(true, true, false, true, true);
      l.update(false, true, false, true, true);
      Assert.assertEquals(1L, (long)l.getCount());
      Assert.assertFalse(l.consumeShotEvent());
      l.update(false, true, true, true, true);
      l.update(false, true, false, true, true);
      Assert.assertFalse(l.consumeShotEvent());
      Assert.assertEquals(1L, (long)l.getCount());
   }

   @Test
   public void intakeEventOncePerPassAndCountUp() {
      BallTrackerLogic l = new BallTrackerLogic(4);
      l.update(false, false);
      Assert.assertFalse(l.consumeIntakeEvent());
      l.update(true, false);
      l.update(true, false);
      Assert.assertEquals(1L, (long)l.getCount());
      Assert.assertTrue(l.consumeIntakeEvent());
      Assert.assertFalse(l.consumeIntakeEvent());
      l.update(false, false);
      Assert.assertFalse(l.consumeIntakeEvent());
      Assert.assertEquals(1L, (long)l.getCount());
   }

   @Test
   public void shotEventExactlyOncePerPassAndCountDown() {
      BallTrackerLogic l = new BallTrackerLogic(4);
      l.setCount(2);
      l.update(false, true);
      l.update(false, true);
      Assert.assertFalse(l.consumeShotEvent());
      Assert.assertTrue(l.ballAtShooter());
      l.update(false, false);
      Assert.assertTrue(l.consumeShotEvent());
      Assert.assertFalse(l.consumeShotEvent());
      l.update(false, false);
      Assert.assertFalse(l.consumeShotEvent());
      Assert.assertEquals(1L, (long)l.getCount());
   }

   @Test
   public void countClamped() {
      BallTrackerLogic l = new BallTrackerLogic(2);

      for(int i = 0; i < 4; ++i) {
         l.update(true, false);
         l.update(false, false);
      }

      Assert.assertEquals(2L, (long)l.getCount());
      Assert.assertTrue(l.isFull());

      for(int i = 0; i < 4; ++i) {
         l.update(false, true);
         l.update(false, false);
      }

      Assert.assertEquals(0L, (long)l.getCount());
   }

   @Test
   public void unusedSensorIgnored() {
      BallTrackerLogic l = new BallTrackerLogic(4);
      l.setUse(false, true);
      l.update(true, false);
      l.update(false, false);
      Assert.assertEquals(0L, (long)l.getCount());
      Assert.assertFalse(l.consumeIntakeEvent());
      Assert.assertFalse(l.ballAtIntake());
      l.setUse(true, false);
      l.update(false, true);
      l.update(false, false);
      Assert.assertFalse(l.consumeShotEvent());
      Assert.assertEquals(0L, (long)l.getCount());
   }

   @Test
   public void shooterFaultWithBallPresentIsNotAShot() {
      BallTrackerLogic l = new BallTrackerLogic(4);
      l.setCount(2);
      l.update(false, true, true, true, false);
      l.update(false, true, false, false, false);
      l.update(false, true, false, false, false);
      Assert.assertFalse(l.consumeShotEvent());
      Assert.assertEquals(2L, (long)l.getCount());
      l.update(false, true, true, true, false);
      Assert.assertFalse(l.consumeShotEvent());
      Assert.assertEquals(2L, (long)l.getCount());
      l.update(false, true, false, true, false);
      Assert.assertTrue(l.consumeShotEvent());
      Assert.assertEquals(1L, (long)l.getCount());
   }

   @Test
   public void intakeRecoveryWithBallDoesNotDoubleCount() {
      BallTrackerLogic l = new BallTrackerLogic(4);
      l.update(false, true, false, true, false);
      l.update(true, true, false, true, false);
      Assert.assertEquals(1L, (long)l.getCount());
      Assert.assertTrue(l.consumeIntakeEvent());
      l.update(false, false, false, true, false);
      l.update(true, true, false, true, false);
      Assert.assertEquals(1L, (long)l.getCount());
      Assert.assertFalse(l.consumeIntakeEvent());
      l.update(false, true, false, true, false);
      l.update(true, true, false, true, false);
      Assert.assertEquals(2L, (long)l.getCount());
      Assert.assertTrue(l.consumeIntakeEvent());
   }

   @Test
   public void shooterBallAtStartSeedsCountAndNoEvent() {
      BallTrackerLogic l = new BallTrackerLogic(4);
      l.update(false, true, true, true, false);
      Assert.assertEquals(1L, (long)l.getCount());
      Assert.assertFalse(l.consumeShotEvent());
      l.update(false, true, false, true, false);
      Assert.assertTrue(l.consumeShotEvent());
      Assert.assertEquals(0L, (long)l.getCount());
   }

   @Test
   public void shooterFirstValidReadingAfterInvalidSeeds() {
      BallTrackerLogic l = new BallTrackerLogic(4);
      l.update(false, true, true, false, false);
      Assert.assertEquals(0L, (long)l.getCount());
      l.update(false, true, true, true, false);
      Assert.assertEquals(1L, (long)l.getCount());
      Assert.assertFalse(l.consumeShotEvent());
   }

   @Test
   public void intakeBallAtStartCountsOnce() {
      BallTrackerLogic l = new BallTrackerLogic(4);
      l.update(true, true, false, true, false);
      l.update(true, true, false, true, false);
      Assert.assertEquals(1L, (long)l.getCount());
      Assert.assertTrue(l.consumeIntakeEvent());
      Assert.assertFalse(l.consumeIntakeEvent());
   }

   @Test
   public void reversingIntakeIgnoresRisingEdges() {
      BallTrackerLogic l = new BallTrackerLogic(4);
      l.update(false, true, false, true, true);
      l.update(true, true, false, true, true);
      l.update(true, true, false, true, false);
      Assert.assertEquals(0L, (long)l.getCount());
      Assert.assertFalse(l.consumeIntakeEvent());
      l.update(false, true, false, true, false);
      l.update(true, true, false, true, false);
      Assert.assertEquals(1L, (long)l.getCount());
      Assert.assertTrue(l.consumeIntakeEvent());
   }

   @Test
   public void twoArgUpdateStillWorks() {
      BallTrackerLogic l = new BallTrackerLogic(4);
      l.update(false, false);
      l.update(true, false);
      Assert.assertEquals(1L, (long)l.getCount());
   }

   @Test
   public void resetClearsCountAndEvents() {
      BallTrackerLogic l = new BallTrackerLogic(4);
      l.update(true, false);
      l.reset();
      Assert.assertEquals(0L, (long)l.getCount());
      Assert.assertFalse(l.consumeIntakeEvent());
   }

   @Test
   public void resetWithBallAtShooterReseedsCountToOne() {
      BallTrackerLogic l = new BallTrackerLogic(4);
      l.update(false, true);
      l.reset();
      Assert.assertEquals(1L, (long)l.getCount());
      l.setCount(0);
      Assert.assertEquals(1L, (long)l.getCount());
      l.update(false, false);
      Assert.assertEquals(0L, (long)l.getCount());
      Assert.assertTrue(l.consumeShotEvent());
   }

   @Test
   public void resetWithNoBallGivesZero() {
      BallTrackerLogic l = new BallTrackerLogic(4);
      l.update(true, false);
      l.reset();
      Assert.assertEquals(0L, (long)l.getCount());
      l.setCount(0);
      Assert.assertEquals(0L, (long)l.getCount());
   }

   @Test
   public void fuzzKeepsCountInRange() {
      Random r = new Random(7L);
      BallTrackerLogic l = new BallTrackerLogic(4);

      for(int i = 0; i < 5000; ++i) {
         int k = r.nextInt(10);
         if (k == 0) {
            l.reset();
         } else if (k == 1) {
            l.setCount(r.nextInt(8) - 2);
         } else {
            l.update(r.nextBoolean(), r.nextBoolean());
         }

         Assert.assertTrue(l.getCount() >= 0 && l.getCount() <= 4);
         if (l.ballAtShooter()) {
            Assert.assertTrue(l.getCount() >= 1 || k >= 2);
         }
      }

   }
}
