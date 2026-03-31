package frc.robot.Commands;

import java.io.File;
import java.io.PrintWriter;

public class ShooterInterpolator {
    private static final double ANGLE_EPSILON = 1e-9;

    // --- 1. EMPIRICAL DATA ---
                                            //0  //5  //10 //15 //20 //25 //30 //35 //40 //45 //50
    //private static final double[] ANGLES = {78.7,73.7,68.7,63.7,58.7,53.7,48.7,43.7,38.7,33.7}; 
    private static final double[] ANGLES = {0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50}; 
    private static final double[] SPEEDS = {200, 240, 280, 320, 360, 400, 440, 480, 520, 560, 600, 640, 680, 720}; 
    
    private static final double[][] DISTANCE_GRID = {
        {8,18,29,40,46,55,61,63,68,70,68},
        {21,35,51,66,75,85,97,100,102,103,100},
        {47,64,90,108,122,134,150,153,157,159,157},
        {65,89,122,150,168,189,202,211,219,220,219},
        {74,106,145,176,206,232,247,266,275,274,275},
        {0,0,0,0,235,258,284,309,315,323,321},
        {0,0,0,0,0,290,314,334,353,359,361},
        {0,0,0,0,0,0,343,364,377,390,383},
        {0,0,0,0,0,0,363,385,406,414,395},
        {0,0,0,0,0,0,0,417,429,432,429},
        {0,0,0,0,0,0,0,0,450,459,450},
        {0,0,0,0,0,0,0,0,0,490,479},
        {0,0,0,0,0,0,0,0,0,527,522},
        {0,0,0,0,0,0,0,0,0,558,552}
    };

    // --- 2. PRE-COMPUTED MEMORY CACHE ---
    private static double[][] generatedSpeedMap;
    private static double[] minValidAngleByDistance;
    private static double[] maxValidAngleByDistance;
    private static int minDistance = Integer.MAX_VALUE;
    private static int maxDistance = Integer.MIN_VALUE;

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

        generatedSpeedMap = new double[maxDistance + 1][ANGLES.length];
        minValidAngleByDistance = new double[maxDistance + 1];
        maxValidAngleByDistance = new double[maxDistance + 1];

        for (int targetDist = minDistance; targetDist <= maxDistance; targetDist++) {
            double minValidAngle = Double.POSITIVE_INFINITY;
            double maxValidAngle = Double.NEGATIVE_INFINITY;

            for (int col = 0; col < ANGLES.length; col++) {
                double calculatedSpeed = 0.0;
                
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
                        double v1 = SPEEDS[row1];
                        double v2 = SPEEDS[row2];
                        
                        if (d1 == d2) calculatedSpeed = v1; 
                        else calculatedSpeed = v1 + (v2 - v1) * ((targetDist - d1) / (d2 - d1));
                        
                        break; 
                    }
                }
                generatedSpeedMap[targetDist][col] = calculatedSpeed;
                if (calculatedSpeed > 0.0) {
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
    public static double getSpeedAtAngle(int targetDistance, double targetAngle) {
        if (!isDistanceInRange(targetDistance)) return 0.0;

        double[] speedsForDist = generatedSpeedMap[targetDistance];

        for (int i = 0; i < ANGLES.length; i++) {
            if (Math.abs(targetAngle - ANGLES[i]) <= ANGLE_EPSILON) {
                return speedsForDist[i];
            }
        }

        boolean ascendingAngles = ANGLES[0] <= ANGLES[ANGLES.length - 1];
        if (ascendingAngles) {
            if (targetAngle <= ANGLES[0]) return speedsForDist[0];
            if (targetAngle >= ANGLES[ANGLES.length - 1]) return speedsForDist[ANGLES.length - 1];
        } else {
            if (targetAngle >= ANGLES[0]) return speedsForDist[0];
            if (targetAngle <= ANGLES[ANGLES.length - 1]) return speedsForDist[ANGLES.length - 1];
        }

        for (int i = 0; i < ANGLES.length - 1; i++) {
            double a1 = ANGLES[i];
            double a2 = ANGLES[i + 1];

            if (isBetween(targetAngle, a1, a2)) {
                double v1 = speedsForDist[i];
                double v2 = speedsForDist[i + 1];

                if (v1 <= 0.0 || v2 <= 0.0) return 0.0;

                return v1 + (v2 - v1) * ((targetAngle - a1) / (a2 - a1));
            }
        }
        return 0.0;
    }

    public static double getMinValidAngle(int targetDistance) {
        if (!isDistanceInRange(targetDistance)) return Double.NaN;
        return minValidAngleByDistance[targetDistance];
    }

    public static double getMaxValidAngle(int targetDistance) {
        if (!isDistanceInRange(targetDistance)) return Double.NaN;
        return maxValidAngleByDistance[targetDistance];
    }

    private static boolean isDistanceInRange(int targetDistance) {
        return targetDistance >= minDistance && targetDistance <= maxDistance;
    }

    private static boolean isBetween(double value, double bound1, double bound2) {
        return value >= Math.min(bound1, bound2) && value <= Math.max(bound1, bound2);
    }

    // --- 4. CSV GENERATOR / TEST SUITE ---
    public static void main(String[] args) {
        System.out.println("Shooter map initialized in memory.");
        
        String filename = "ShooterInterpolationTest.csv";
        try (PrintWriter writer = new PrintWriter(new File(filename))) {
            writer.println("Target Distance (in),Hood Angle (deg),Flywheel Speed (ips)");
            
            for (int dist = minDistance; dist <= maxDistance; dist++) {
                for (double angle : ANGLES) {
                    double speed = getSpeedAtAngle(dist, angle);
                    writer.printf("%d,%.1f,%.2f%n", dist, angle, speed);
                }
            }
            
            System.out.println("Test Complete. CSV Exported to: " + new File(filename).getAbsolutePath());
            
        } catch (Exception e) {
            System.err.println("Failed to write CSV: " + e.getMessage());
        }
    }
}
