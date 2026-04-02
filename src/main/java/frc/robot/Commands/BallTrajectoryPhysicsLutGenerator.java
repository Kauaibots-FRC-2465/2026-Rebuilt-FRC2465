package frc.robot.Commands;

import java.io.BufferedOutputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;

public final class BallTrajectoryPhysicsLutGenerator {
    private static final int LUT_FILE_MAGIC = ShooterConstants.FITTED_BALL_TRAJECTORY_LUT_MAGIC;
    private static final Path OUTPUT_PATH = Path.of(
            "src",
            "main",
            "deploy",
            ShooterConstants.FITTED_BALL_TRAJECTORY_LUT_FILENAME);

    private BallTrajectoryPhysicsLutGenerator() {
    }

    public static void main(String[] args) throws IOException {
        Files.createDirectories(OUTPUT_PATH.getParent());

        int lutLength = BallTrajectoryLookup.LUT_ELEVATION_BIN_COUNT
                * BallTrajectoryLookup.LUT_HOOD_ANGLE_BIN_COUNT
                * BallTrajectoryLookup.LUT_DISTANCE_BIN_COUNT;
        float[] requiredExitVelocityLut = new float[lutLength];
        float[] maximumBallZElevationLut = new float[lutLength];

        for (int angleIndex = 0; angleIndex < BallTrajectoryLookup.LUT_HOOD_ANGLE_BIN_COUNT; angleIndex++) {
            double hoodAngleDegrees = BallTrajectoryLookup.LUT_MIN_HOOD_ANGLE_DEGREES
                    + angleIndex * BallTrajectoryLookup.LUT_HOOD_ANGLE_STEP_DEGREES;
            System.out.printf(
                    "Generating hood angle bin %d/%d -> %.1f deg%n",
                    angleIndex + 1,
                    BallTrajectoryLookup.LUT_HOOD_ANGLE_BIN_COUNT,
                    hoodAngleDegrees);

            for (int elevationIndex = 0;
                    elevationIndex < BallTrajectoryLookup.LUT_ELEVATION_BIN_COUNT;
                    elevationIndex++) {
                double targetElevationInches =
                        elevationIndex * BallTrajectoryLookup.LUT_ELEVATION_STEP_INCHES;
                for (int distanceIndex = 0;
                        distanceIndex < BallTrajectoryLookup.LUT_DISTANCE_BIN_COUNT;
                        distanceIndex++) {
                    int lutIndex = BallTrajectoryLookup.toLutIndex(angleIndex, distanceIndex, elevationIndex);
                    float requiredExitVelocityIps = (float) BallTrajectoryLookup.solveRequiredExitVelocityIpsExact(
                            hoodAngleDegrees,
                            distanceIndex,
                            targetElevationInches);
                    requiredExitVelocityLut[lutIndex] = requiredExitVelocityIps;
                    maximumBallZElevationLut[lutIndex] = (float) BallTrajectoryLookup
                            .getMaximumBallZElevationForSolvedShotInches(
                                    hoodAngleDegrees,
                                    requiredExitVelocityIps,
                                    distanceIndex);
                }
            }
        }

        try (DataOutputStream output = new DataOutputStream(
                new BufferedOutputStream(Files.newOutputStream(OUTPUT_PATH)))) {
            output.writeInt(LUT_FILE_MAGIC);
            output.writeInt(ShooterConstants.FITTED_BALL_TRAJECTORY_LUT_VERSION);
            output.writeInt(BallTrajectoryLookup.LUT_DISTANCE_BIN_COUNT);
            output.writeInt(BallTrajectoryLookup.LUT_HOOD_ANGLE_BIN_COUNT);
            output.writeInt(BallTrajectoryLookup.LUT_ELEVATION_BIN_COUNT);

            for (float value : requiredExitVelocityLut) {
                output.writeFloat(value);
            }
            for (float value : maximumBallZElevationLut) {
                output.writeFloat(value);
            }
        }

        System.out.println("Wrote LUT to: " + OUTPUT_PATH.toAbsolutePath());
    }
}
