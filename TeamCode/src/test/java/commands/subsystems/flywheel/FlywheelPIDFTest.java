package commands.subsystems.flywheel;

import robot.RobotSettings;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

public class FlywheelPIDFTest {
    private double savedKS, savedKV, savedKA, savedKP, savedKI, savedKD, savedIMax;

    @Before
    public void saveTunables() {
        savedKS = RobotSettings.FlywheelPIDF.kS;
        savedKV = RobotSettings.FlywheelPIDF.kV;
        savedKA = RobotSettings.FlywheelPIDF.kA;
        savedKP = RobotSettings.FlywheelPIDF.kP;
        savedKI = RobotSettings.FlywheelPIDF.kI;
        savedKD = RobotSettings.FlywheelPIDF.kD;
        savedIMax = RobotSettings.FlywheelPIDF.I_MAX_VOLTS;
    }

    @After
    public void restoreTunables() {
        RobotSettings.FlywheelPIDF.kS = savedKS;
        RobotSettings.FlywheelPIDF.kV = savedKV;
        RobotSettings.FlywheelPIDF.kA = savedKA;
        RobotSettings.FlywheelPIDF.kP = savedKP;
        RobotSettings.FlywheelPIDF.kI = savedKI;
        RobotSettings.FlywheelPIDF.kD = savedKD;
        RobotSettings.FlywheelPIDF.I_MAX_VOLTS = savedIMax;
    }

    /** With only kI active and a negative gain, the integral must still stay within +/- I_MAX_VOLTS / |kI|. */
    @Test
    public void negativeKiIntegralClampIsSymmetric() {
        RobotSettings.FlywheelPIDF.kS = 0;
        RobotSettings.FlywheelPIDF.kV = 0;
        RobotSettings.FlywheelPIDF.kA = 0;
        RobotSettings.FlywheelPIDF.kP = 0;
        RobotSettings.FlywheelPIDF.kD = 0;
        RobotSettings.FlywheelPIDF.kI = -0.1;
        RobotSettings.FlywheelPIDF.I_MAX_VOLTS = 2.0;

        FlywheelPIDF pidf = new FlywheelPIDF();
        // Large positive error builds the integral up; clamp should hold it at +20 (2 V / 0.1).
        for (int i = 0; i < 1000; i++) {
            pidf.calculate(1000, 0, 12.0, 0.01);
        }
        // Output = kI * integral / 12 = -0.1 * 20 / 12 is the clamped contribution; must be bounded by 2 V.
        double out = pidf.calculate(1000, 0, 12.0, 0.01);
        assertEquals(-2.0 / 12.0, out, 1e-9);

        // Now large negative error must pull the integral back to -20 (not pin it at +20).
        pidf.reset();
        for (int i = 0; i < 1000; i++) {
            pidf.calculate(0.0001, 1000, 12.0, 0.01);
        }
        double low = pidf.calculate(0.0001, 1000, 12.0, 0.01);
        assertEquals(2.0 / 12.0, low, 1e-9);
    }

    /** A 0 or NaN battery reading must not give Infinity/NaN power; nominal voltage is used instead. */
    @Test
    public void badBatteryVoltageGivesFinitePower() {
        RobotSettings.FlywheelPIDF.kS = 0;
        RobotSettings.FlywheelPIDF.kV = 0.001;
        RobotSettings.FlywheelPIDF.kA = 0;
        RobotSettings.FlywheelPIDF.kP = 0;
        RobotSettings.FlywheelPIDF.kI = 0;
        RobotSettings.FlywheelPIDF.kD = 0;
        double expect = 0.001 * 1000 / RobotSettings.FlywheelPIDF.NOMINAL_VOLTAGE;
        assertEquals(expect, new FlywheelPIDF().calculate(1000, 0, 0.0, 0.02), 1e-12);
        assertEquals(expect, new FlywheelPIDF().calculate(1000, 0, Double.NaN, 0.02), 1e-12);
        assertEquals(0.001 * 1000 / 12.5, new FlywheelPIDF().calculate(1000, 0, 12.5, 0.02), 1e-12);
    }

    /** Positive kI behaves as before: integral bounded to +/- I_MAX_VOLTS / kI. */
    @Test
    public void positiveKiIntegralClampUnchanged() {
        RobotSettings.FlywheelPIDF.kS = 0;
        RobotSettings.FlywheelPIDF.kV = 0;
        RobotSettings.FlywheelPIDF.kA = 0;
        RobotSettings.FlywheelPIDF.kP = 0;
        RobotSettings.FlywheelPIDF.kD = 0;
        RobotSettings.FlywheelPIDF.kI = 0.1;
        RobotSettings.FlywheelPIDF.I_MAX_VOLTS = 2.0;

        FlywheelPIDF pidf = new FlywheelPIDF();
        for (int i = 0; i < 1000; i++) {
            pidf.calculate(1000, 0, 12.0, 0.01);
        }
        double out = pidf.calculate(1000, 0, 12.0, 0.01);
        assertEquals(2.0 / 12.0, out, 1e-9);
        assertTrue(out <= 1.0);
    }
}
