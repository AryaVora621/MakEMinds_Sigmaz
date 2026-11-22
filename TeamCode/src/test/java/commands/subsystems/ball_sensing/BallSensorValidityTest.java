package commands.subsystems.ball_sensing;

import org.junit.Assert;
import org.junit.Test;

public class BallSensorValidityTest {
   public BallSensorValidityTest() {
   }

   @Test
   public void limitOffAlwaysReads() {
      Assert.assertTrue(BallSensor.isReadDue(true, 100L, 100L, (double)0.0F));
      Assert.assertTrue(BallSensor.isReadDue(true, 100L, 101L, (double)-5.0F));
   }

   @Test
   public void firstReadAlwaysDue() {
      Assert.assertTrue(BallSensor.isReadDue(false, 0L, 0L, (double)50.0F));
   }

   @Test
   public void holdsUntilIntervalElapsed() {
      Assert.assertFalse(BallSensor.isReadDue(true, 100L, 109L, (double)10.0F));
      Assert.assertTrue(BallSensor.isReadDue(true, 100L, 110L, (double)10.0F));
      Assert.assertTrue(BallSensor.isReadDue(true, 100L, 150L, (double)10.0F));
   }

   @Test
   public void clockGoingBackwardsReads() {
      Assert.assertTrue(BallSensor.isReadDue(true, 100L, 50L, (double)10.0F));
   }

   @Test
   public void nanIntervalReadsEveryLoop() {
      Assert.assertTrue(BallSensor.isReadDue(true, 100L, 100L, Double.NaN));
   }
}
