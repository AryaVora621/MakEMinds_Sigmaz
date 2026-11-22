package commands.subsystems.ball_sensing;

import org.junit.Assert;
import org.junit.Test;

public class HueClassifierTest {

    @Test
    public void primaryHues() {
        Assert.assertEquals(0.0, HueClassifier.rgbToHue(1.0, 0.0, 0.0), 1.0E-9);
        Assert.assertEquals(120.0, HueClassifier.rgbToHue(0.0, 1.0, 0.0), 1.0E-9);
        Assert.assertEquals(240.0, HueClassifier.rgbToHue(0.0, 0.0, 1.0), 1.0E-9);
        Assert.assertEquals(60.0, HueClassifier.rgbToHue(1.0, 1.0, 0.0), 1.0E-9);
        Assert.assertEquals(300.0, HueClassifier.rgbToHue(1.0, 0.0, 1.0), 1.0E-9);
    }

    @Test
    public void greyAndInvalidAreNaN() {
        Assert.assertTrue(Double.isNaN(HueClassifier.rgbToHue(0.5, 0.5, 0.5)));
        Assert.assertTrue(Double.isNaN(HueClassifier.rgbToHue(Double.NaN, 0.0, 0.0)));
    }

    @Test
    public void classifyUsesRangesAndPresence() {
        Assert.assertEquals(BallColor.NONE, HueClassifier.classify(false, 1.0, 1.0, 0.0));
        Assert.assertEquals(BallColor.POLLEN, HueClassifier.classify(true, 1.0, 1.0, 0.0));
        Assert.assertEquals(BallColor.UNKNOWN, HueClassifier.classify(true, 0.0, 1.0, 0.0));
        Assert.assertEquals(BallColor.UNKNOWN, HueClassifier.classify(true, 0.5, 0.5, 0.5));
    }

    /** Field CAD appearance colours: pollen 255,239,63; red nectar 255,0,0; blue nectar 0,0,255. */
    @Test
    public void fieldBallColoursClassify() {
        Assert.assertEquals(BallColor.POLLEN, HueClassifier.classify(true, 255 / 255.0, 239 / 255.0, 63 / 255.0));
        Assert.assertEquals(BallColor.NECTAR, HueClassifier.classify(true, 1.0, 0.0, 0.0));
        Assert.assertEquals(BallColor.NECTAR, HueClassifier.classify(true, 0.0, 0.0, 1.0));
        // a typical V3 reading of a blue ball leans green (hue ~210)
        Assert.assertEquals(BallColor.NECTAR, HueClassifier.classify(true, 0.05, 0.25, 0.40));
        // a red ball reading slightly orange or slightly magenta
        Assert.assertEquals(BallColor.NECTAR, HueClassifier.classify(true, 0.50, 0.10, 0.05));
        Assert.assertEquals(BallColor.NECTAR, HueClassifier.classify(true, 0.50, 0.05, 0.12));
    }

    @Test
    public void wrappingRange() {
        Assert.assertTrue(HueClassifier.inRange(350, 330, 25));
        Assert.assertTrue(HueClassifier.inRange(10, 330, 25));
        Assert.assertFalse(HueClassifier.inRange(100, 330, 25));
        Assert.assertTrue(HueClassifier.inRange(50, 40, 75));
        Assert.assertFalse(HueClassifier.inRange(80, 40, 75));
    }
}
