package commands.subsystems.ball_sensing;

import org.junit.Assert;
import org.junit.Test;
import robot.RobotSettings;

public class BallCounterTest {
   public BallCounterTest() {
   }

   @Test
   public void neverNegative() {
      BallCounter c = new BallCounter(4);
      c.decrement();
      c.decrement();
      Assert.assertEquals(0L, (long)c.getCount());
   }

   @Test
   public void neverAboveMax() {
      BallCounter c = new BallCounter(4);

      for(int i = 0; i < 10; ++i) {
         c.increment();
      }

      Assert.assertEquals(4L, (long)c.getCount());
      Assert.assertTrue(c.isFull());
   }

   @Test
   public void setClamps() {
      BallCounter c = new BallCounter(4);
      c.set(99);
      Assert.assertEquals(4L, (long)c.getCount());
      c.set(-5);
      Assert.assertEquals(0L, (long)c.getCount());
      Assert.assertFalse(c.isFull());
   }

   @Test
   public void maxChangeReclamps() {
      BallCounter c = new BallCounter(4);
      c.set(4);
      c.setMax(2);
      Assert.assertEquals(2L, (long)c.getCount());
      c.setMax(5);
      Assert.assertEquals(2L, (long)c.getCount());
      c.reset();
      Assert.assertEquals(0L, (long)c.getCount());
   }

   @Test
   public void maxBallsIsFour() {
      Assert.assertEquals(4L, (long)RobotSettings.BallTracker.MAX_BALLS);
      Assert.assertEquals(4L, (long)RobotSettings.BallTracker.MAX_BALLS);
      BallCounter c = new BallCounter(RobotSettings.BallTracker.MAX_BALLS);

      for(int i = 0; i < 4; ++i) {
         c.increment();
      }

      Assert.assertEquals(4L, (long)c.getCount());
      Assert.assertTrue(c.isFull());
      c.increment();
      Assert.assertEquals(4L, (long)c.getCount());
      BallTrackerLogic l = new BallTrackerLogic(RobotSettings.BallTracker.MAX_BALLS);

      for(int i = 0; i < 5; ++i) {
         l.update(true, false);
         l.update(false, false);
      }

      Assert.assertEquals(4L, (long)l.getCount());
      Assert.assertTrue(l.isFull());
   }
}
