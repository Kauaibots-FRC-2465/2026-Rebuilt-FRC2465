package frc.robot.Commands;

import java.io.BufferedOutputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;

public final class BallTrajectoryPhysicsLutGenerator {
    private static final int LUT_FILE_MAGIC = 0x42544C54; // "BTLT"
    private static final Path OUTPUT_PATH = Path.of(
            "src",
            "main",
            "deploy",
            ShooterConstants.BALL_TRAJECTORY_LUT_FILENAME);

    private BallTrajectoryPhysicsLutGenerator() {
    }

    public static void main(String[] args) throws IOException {
        Files.createDirectories(OUTPUT_PATH.getParent());

        float[] lut = new float[BallTrajectoryPhysics.LUT_ELEVATION_BIN_COUNT
                * BallTrajectoryPhysics.LUT_HOOD_ANGLE_BIN_COUNT
                * BallTrajectoryPhysics.LUT_DISTANCE_BIN_COUNT];

        for (int angleIndex = 0; angleIndex < BallTrajectoryPhysics.LUT_HOOD_ANGLE_BIN_COUNT; angleIndex++) {
            double hoodAngleDegrees = BallTrajectoryPhysics.LUT_MIN_HOOD_ANGLE_DEGREES
                    + angleIndex * BallTrajectoryPhysics.LUT_HOOD_ANGLE_STEP_DEGREES;
            System.out.printf(
                    "Generating hood angle bin %d/%d -> %.1f deg%n",
                    angleIndex + 1,
                    BallTrajectoryPhysics.LUT_HOOD_ANGLE_BIN_COUNT,
                    hoodAngleDegrees);

            for (int elevationIndex = 0;
                    elevationIndex < BallTrajectoryPhysics.LUT_ELEVATION_BIN_COUNT;
                    elevationIndex++) {
                double targetElevationInches =
                        elevationIndex * BallTrajectoryPhysics.LUT_ELEVATION_STEP_INCHES;
                for (int distanceIndex = 0;
                        distanceIndex < BallTrajectoryPhysics.LUT_DISTANCE_BIN_COUNT;
                        distanceIndex++) {
                    float requiredExitVelocityIps = (float) BallTrajectoryPhysics
                            .solveRequiredExitVelocityIpsExact(
                                    hoodAngleDegrees,
                                    distanceIndex,
                                    targetElevationInches);
                    lut[BallTrajectoryPhysics.toLutIndex(angleIndex, distanceIndex, elevationIndex)] =
                            requiredExitVelocityIps;
                }
            }
        }

        try (DataOutputStream output = new DataOutputStream(
                new BufferedOutputStream(Files.newOutputStream(OUTPUT_PATH)))) {
            output.writeInt(LUT_FILE_MAGIC);
            output.writeInt(ShooterConstants.BALL_TRAJECTORY_LUT_VERSION);
            output.writeInt(BallTrajectoryPhysics.LUT_DISTANCE_BIN_COUNT);
            output.writeInt(BallTrajectoryPhysics.LUT_HOOD_ANGLE_BIN_COUNT);
            output.writeInt(BallTrajectoryPhysics.LUT_ELEVATION_BIN_COUNT);

            for (float value : lut) {
                output.writeFloat(value);
            }
        }

        System.out.println("Wrote LUT to: " + OUTPUT_PATH.toAbsolutePath());
    }
}
