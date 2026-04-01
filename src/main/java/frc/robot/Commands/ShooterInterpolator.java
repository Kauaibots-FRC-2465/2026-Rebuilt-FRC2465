package frc.robot.Commands;

import java.io.File;
import java.io.PrintWriter;

public class ShooterInterpolator {
    private static final double ANGLE_EPSILON = 1e-9;

    // --- 1. EMPIRICAL DATA ---
    private static final double[] ANGLES = ShooterConstants.ACTUAL_ANGLES_DEGREES;
    private static final double[] SPEEDS = ShooterConstants.COMMAND_SPEEDS_IPS;
    private static final double[] SHOT_EXIT_SPEEDS = ShooterConstants.COMMAND_BALL_EXIT_IPS;
    private static final double[][] DISTANCE_GRID = ShooterConstants.DISTANCE_GRID_INCHES;

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
