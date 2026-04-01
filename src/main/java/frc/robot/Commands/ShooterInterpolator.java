package frc.robot.Commands;

import java.io.File;
import java.io.PrintWriter;

public class ShooterInterpolator {
    private static final double ANGLE_EPSILON = 1e-9;
    private static final double HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES = 76.9;

    // --- 1. EMPIRICAL DATA ---
    private static final double[] ANGLES = {
        HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES - 0.0,
        HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES - 5.0,
        HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES - 10.0,
        HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES - 15.0,
        HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES - 20.0,
        HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES - 25.0,
        HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES - 30.0,
        HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES - 35.0,
        HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES - 40.0,
        HOOD_ANGLE_AT_MECHANISM_ZERO_DEGREES - 45.0
    };
    private static final double[] SPEEDS = {200, 240, 280, 320, 360, 400, 440, 480, 520, 560, 600, 640, 680, 720}; 
    private static final double[] SHOT_EXIT_SPEEDS = buildShotExitSpeeds();
    
    private static final double[][] DISTANCE_GRID = {
        // 76.9 71.9 66.9 61.9 56.9 51.9 46.9 41.9 36.9 31.9
        {  8, 18, 29, 40, 46, 55, 61, 63, 68, 70}, //200
        { 21, 35, 51, 66, 75, 85, 97,100,102,103}, //240
        { 47, 64, 90,108,122,134,150,153,157,159}, //280
        { 65, 89,122,150,168,189,202,211,219,220}, //320
        { 74,106,145,176,206,232,247,266,275,274}, //360
        {  0,  0,  0,  0,235,258,284,309,315,323}, //25
        {  0,  0,  0,  0,  0,290,314,334,353,359}, //30
        {  0,  0,  0,  0,  0,  0,343,364,377,390}, //35
        {  0,  0,  0,  0,  0,  0,363,385,406,414}, //40
        {  0,  0,  0,  0,  0,  0,  0,417,429,432}, //45
        {  0,  0,  0,  0,  0,  0,  0,  0,450,459}, //50
        {  0,  0,  0,  0,  0,  0,  0,  0,  0,490}, //55
        {  0,  0,  0,  0,  0,  0,  0,  0,  0,527}, //60
        {  0,  0,  0,  0,  0,  0,  0,  0,  0,558} //65
    };

    // --- 2. PRE-COMPUTED MEMORY CACHE ---
    private static double[][] generatedShotVelocityMap;
    private static double[] minValidAngleByDistance;
    private static double[] maxValidAngleByDistance;
    private static int minDistance = Integer.MAX_VALUE;
    private static int maxDistance = Integer.MIN_VALUE;

    public record ShotSolution(double angleDegrees, double shotVelocityIps) {
        public static ShotSolution invalid() {
            return new ShotSolution(Double.NaN, Double.NaN);
        }

        public boolean isValid() {
            return Double.isFinite(angleDegrees) && Double.isFinite(shotVelocityIps);
        }
    }

    static {
        for (double[] row : DISTANCE_GRID) {
            if (row.length != ANGLES.length) {
                throw new IllegalStateException(
                        "DISTANCE_GRID column count must match ANGLES length");
            }
        }

        // Find bounds, ignoring 0s
        for (double[] row : DISTANCE_GRID) {
            for (double dist : row) {
                if (dist > 0) {
                    if (dist < minDistance) minDistance = (int) Math.floor(dist);
                    if (dist > maxDistance) maxDistance = (int) Math.ceil(dist);
                }
            }
        }

        generatedShotVelocityMap = new double[maxDistance + 1][ANGLES.length];
        minValidAngleByDistance = new double[maxDistance + 1];
        maxValidAngleByDistance = new double[maxDistance + 1];

        for (int targetDist = minDistance; targetDist <= maxDistance; targetDist++) {
            double minValidAngle = Double.POSITIVE_INFINITY;
            double maxValidAngle = Double.NEGATIVE_INFINITY;

            for (int col = 0; col < ANGLES.length; col++) {
                double calculatedShotVelocity = 0.0;
                
                for (int row1 = 0; row1 < SPEEDS.length - 1; row1++) {
                    double d1 = DISTANCE_GRID[row1][col];
                    if (d1 <= 0) continue; 
                    
                    int row2 = row1 + 1;
                    while (row2 < SPEEDS.length && DISTANCE_GRID[row2][col] <= 0) {
                        row2++;
                    }
                    if (row2 >= SPEEDS.length) break; 
                    
                    double d2 = DISTANCE_GRID[row2][col];
                    
                    if ((targetDist >= d1 && targetDist <= d2) || (targetDist <= d1 && targetDist >= d2)) {
                        double v1 = SHOT_EXIT_SPEEDS[row1];
                        double v2 = SHOT_EXIT_SPEEDS[row2];
                        
                        if (d1 == d2) calculatedShotVelocity = v1; 
                        else calculatedShotVelocity = v1 + (v2 - v1) * ((targetDist - d1) / (d2 - d1));
                        
                        break; 
                    }
                }
                generatedShotVelocityMap[targetDist][col] = calculatedShotVelocity;
                if (calculatedShotVelocity > 0.0) {
                    minValidAngle = Math.min(minValidAngle, ANGLES[col]);
                    maxValidAngle = Math.max(maxValidAngle, ANGLES[col]);
                }
            }

            if (minValidAngle == Double.POSITIVE_INFINITY) {
                minValidAngleByDistance[targetDist] = Double.NaN;
                maxValidAngleByDistance[targetDist] = Double.NaN;
            } else {
                minValidAngleByDistance[targetDist] = minValidAngle;
                maxValidAngleByDistance[targetDist] = maxValidAngle;
            }
        }
    }

    // --- 3. RUNTIME QUERY METHOD ---
    public static ShotSolution getShotAtAngle(int targetDistance, double targetAngle) {
        if (!isDistanceInRange(targetDistance)) return ShotSolution.invalid();

        double resolvedAngle = clampAngleToBounds(targetAngle);
        double shotVelocityIps = getShotVelocityAtAngle(targetDistance, resolvedAngle);
        if (!Double.isFinite(shotVelocityIps) || shotVelocityIps <= 0.0) {
            return ShotSolution.invalid();
        }

        return new ShotSolution(resolvedAngle, shotVelocityIps);
    }

    public static ShotSolution getMinValidShot(int targetDistance) {
        if (!isDistanceInRange(targetDistance)) return ShotSolution.invalid();
        double minValidAngle = minValidAngleByDistance[targetDistance];
        if (!Double.isFinite(minValidAngle)) return ShotSolution.invalid();
        return getShotAtAngle(targetDistance, minValidAngle);
    }

    public static ShotSolution getMaxValidShot(int targetDistance) {
        if (!isDistanceInRange(targetDistance)) return ShotSolution.invalid();
        double maxValidAngle = maxValidAngleByDistance[targetDistance];
        if (!Double.isFinite(maxValidAngle)) return ShotSolution.invalid();
        return getShotAtAngle(targetDistance, maxValidAngle);
    }

    public static double getShotDistance(double shotVelocityIps, double targetAngle) {
        if (!Double.isFinite(shotVelocityIps) || shotVelocityIps <= 0.0) return Double.NaN;

        double resolvedAngle = clampAngleToBounds(targetAngle);

        for (int i = 0; i < ANGLES.length; i++) {
            if (Math.abs(resolvedAngle - ANGLES[i]) <= ANGLE_EPSILON) {
                return getDistanceAtAngleColumn(shotVelocityIps, i);
            }
        }

        for (int i = 0; i < ANGLES.length - 1; i++) {
            double a1 = ANGLES[i];
            double a2 = ANGLES[i + 1];
            if (isBetween(resolvedAngle, a1, a2)) {
                double d1 = getDistanceAtAngleColumn(shotVelocityIps, i);
                double d2 = getDistanceAtAngleColumn(shotVelocityIps, i + 1);
                if (!Double.isFinite(d1) || !Double.isFinite(d2)) return Double.NaN;
                return d1 + (d2 - d1) * ((resolvedAngle - a1) / (a2 - a1));
            }
        }

        return Double.NaN;
    }

    public static double getMinValidAngle(int targetDistance) {
        if (!isDistanceInRange(targetDistance)) return Double.NaN;
        return minValidAngleByDistance[targetDistance];
    }

    public static double getMaxValidAngle(int targetDistance) {
        if (!isDistanceInRange(targetDistance)) return Double.NaN;
        return maxValidAngleByDistance[targetDistance];
    }

    public static double getShotVelocityAtAngle(int targetDistance, double targetAngle) {
        if (!isDistanceInRange(targetDistance)) return Double.NaN;

        double[] shotVelocitiesForDistance = generatedShotVelocityMap[targetDistance];

        for (int i = 0; i < ANGLES.length; i++) {
            if (Math.abs(targetAngle - ANGLES[i]) <= ANGLE_EPSILON) {
                return shotVelocitiesForDistance[i] > 0.0 ? shotVelocitiesForDistance[i] : Double.NaN;
            }
        }

        boolean ascendingAngles = ANGLES[0] <= ANGLES[ANGLES.length - 1];
        if (ascendingAngles) {
            if (targetAngle <= ANGLES[0]) {
                return shotVelocitiesForDistance[0] > 0.0 ? shotVelocitiesForDistance[0] : Double.NaN;
            }
            if (targetAngle >= ANGLES[ANGLES.length - 1]) {
                return shotVelocitiesForDistance[ANGLES.length - 1] > 0.0
                        ? shotVelocitiesForDistance[ANGLES.length - 1]
                        : Double.NaN;
            }
        } else {
            if (targetAngle >= ANGLES[0]) {
                return shotVelocitiesForDistance[0] > 0.0 ? shotVelocitiesForDistance[0] : Double.NaN;
            }
            if (targetAngle <= ANGLES[ANGLES.length - 1]) {
                return shotVelocitiesForDistance[ANGLES.length - 1] > 0.0
                        ? shotVelocitiesForDistance[ANGLES.length - 1]
                        : Double.NaN;
            }
        }

        for (int i = 0; i < ANGLES.length - 1; i++) {
            double a1 = ANGLES[i];
            double a2 = ANGLES[i + 1];

            if (isBetween(targetAngle, a1, a2)) {
                double v1 = shotVelocitiesForDistance[i];
                double v2 = shotVelocitiesForDistance[i + 1];

                if (v1 <= 0.0 || v2 <= 0.0) return Double.NaN;

                return v1 + (v2 - v1) * ((targetAngle - a1) / (a2 - a1));
            }
        }
        return Double.NaN;
    }

    private static boolean isDistanceInRange(int targetDistance) {
        return targetDistance >= minDistance && targetDistance <= maxDistance;
    }

    private static double clampAngleToBounds(double targetAngle) {
        boolean ascendingAngles = ANGLES[0] <= ANGLES[ANGLES.length - 1];
        if (ascendingAngles) {
            return Math.max(ANGLES[0], Math.min(ANGLES[ANGLES.length - 1], targetAngle));
        }
        return Math.min(ANGLES[0], Math.max(ANGLES[ANGLES.length - 1], targetAngle));
    }

    private static double getDistanceAtAngleColumn(double shotVelocityIps, int angleColumn) {
        double previousDistance = Double.NaN;
        double previousShotVelocity = Double.NaN;

        for (int row = 0; row < DISTANCE_GRID.length; row++) {
            double distance = DISTANCE_GRID[row][angleColumn];
            if (distance <= 0.0) continue;

            double currentShotVelocity = SHOT_EXIT_SPEEDS[row];
            if (Math.abs(shotVelocityIps - currentShotVelocity) <= ANGLE_EPSILON) {
                return distance;
            }

            if (Double.isFinite(previousDistance) && isBetween(shotVelocityIps, previousShotVelocity, currentShotVelocity)) {
                if (Math.abs(currentShotVelocity - previousShotVelocity) <= ANGLE_EPSILON) {
                    return previousDistance;
                }

                return previousDistance
                        + (distance - previousDistance)
                                * ((shotVelocityIps - previousShotVelocity)
                                        / (currentShotVelocity - previousShotVelocity));
            }

            previousDistance = distance;
            previousShotVelocity = currentShotVelocity;
        }

        return Double.NaN;
    }

    private static double[] buildShotExitSpeeds() {
        double[] shotExitSpeeds = new double[SPEEDS.length];
        for (int i = 0; i < SPEEDS.length; i++) {
            double shotExitSpeed = FlywheelBallExitInterpolator.getBallExitIpsForSetIps(SPEEDS[i]);
            if (!Double.isFinite(shotExitSpeed)) {
                throw new IllegalStateException("Unable to convert flywheel set IPS " + SPEEDS[i]
                        + " to ball exit IPS.");
            }
            shotExitSpeeds[i] = shotExitSpeed;
        }
        return shotExitSpeeds;
    }

    private static boolean isBetween(double value, double bound1, double bound2) {
        return value >= Math.min(bound1, bound2) && value <= Math.max(bound1, bound2);
    }

    // --- 4. CSV GENERATOR / TEST SUITE ---
    public static void main(String[] args) {
        System.out.println("Shooter map initialized in memory.");
        
        String filename = "ShooterInterpolationTest.csv";
        try (PrintWriter writer = new PrintWriter(new File(filename))) {
            writer.println("Target Distance (in),Hood Angle (deg),Shot Exit Velocity (ips),Flywheel Command Speed (ips)");
            
            for (int dist = minDistance; dist <= maxDistance; dist++) {
                for (double angle : ANGLES) {
                    ShotSolution shot = getShotAtAngle(dist, angle);
                    double flywheelCommandSpeed = FlywheelBallExitInterpolator.getSetIpsForBallExitIps(
                            shot.shotVelocityIps());
                    writer.printf(
                            "%d,%.1f,%.2f,%.2f%n",
                            dist,
                            angle,
                            shot.shotVelocityIps(),
                            flywheelCommandSpeed);
                }
            }
            
            System.out.println("Test Complete. CSV Exported to: " + new File(filename).getAbsolutePath());
            
        } catch (Exception e) {
            System.err.println("Failed to write CSV: " + e.getMessage());
        }
    }
}
