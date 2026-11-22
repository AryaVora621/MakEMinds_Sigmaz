package commands.subsystems.grouped_subsystems.shooter;

import robot.RobotSettings;

/**
 * Distance to the hive -> flywheel RPM, by straight-line interpolation between measured rows. No physics:
 * you measure a few shots on the field and type them in. The launch angle is fixed (no hood), see
 * RobotSettings.TurretAiming.FIXED_LAUNCH_ELEVATION_DEG.
 *
 * HOW TO TUNE: stand at a few distances, find the RPM that makes 10 of 10 shots, and write each into a row
 * of RobotSettings.ShotTable.DISTANCE_IN / RPM (rows must go from nearest to farthest). Two rows is a straight
 * line; add rows (the same number in both arrays) to bend it. Outside the first and last row the values stay flat.
 *
 * THE NUMBERS IN RobotSettings.ShotTable ARE PLACEHOLDERS, NOT MEASUREMENTS.
 */
public final class ShotTable {
    private ShotTable() {
    }

    public static double rpm(double distanceIn) {
        return interpolate(distanceIn, RobotSettings.ShotTable.RPM);
    }

    private static double interpolate(double d, double[] values) {
        int n = Math.min(RobotSettings.ShotTable.DISTANCE_IN.length, values.length);
        if (n == 0) {
            return 0;
        }
        if (d <= RobotSettings.ShotTable.DISTANCE_IN[0] || n == 1) {
            return values[0];
        }
        if (d >= RobotSettings.ShotTable.DISTANCE_IN[n - 1]) {
            return values[n - 1];
        }
        for (int i = 1; i < n; i++) {
            if (d <= RobotSettings.ShotTable.DISTANCE_IN[i]) {
                double span = RobotSettings.ShotTable.DISTANCE_IN[i] - RobotSettings.ShotTable.DISTANCE_IN[i - 1];
                double t = span > 0 ? (d - RobotSettings.ShotTable.DISTANCE_IN[i - 1]) / span : 0;
                return values[i - 1] + t * (values[i] - values[i - 1]);
            }
        }
        return values[n - 1];
    }
}
