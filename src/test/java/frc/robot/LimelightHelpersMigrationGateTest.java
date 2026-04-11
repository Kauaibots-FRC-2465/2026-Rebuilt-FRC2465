package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.junit.jupiter.api.Test;

class LimelightHelpersMigrationGateTest {
    private static final String TEST_LIMELIGHT_NAME = "limelightMigrationGate";

    @Test
    void poseEstimateUsesPublishedTimestampMinusLatencySeconds() {
        NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
        try (DoubleArrayPublisher posePublisher = ntInstance
                .getTable(TEST_LIMELIGHT_NAME)
                .getDoubleArrayTopic("botpose_wpiblue")
                .publish()) {
            long publishedTimestampNtLocalMicros = 3_000_000L;
            double latencyMilliseconds = 11.0;
            double[] poseArray = new double[] {
                1.5,
                -2.25,
                0.0,
                0.0,
                0.0,
                90.0,
                latencyMilliseconds,
                0.0,
                0.0,
                0.0,
                0.0
            };

            posePublisher.set(poseArray, publishedTimestampNtLocalMicros);
            ntInstance.flush();

            LimelightHelpers.PoseEstimate poseEstimate =
                    LimelightHelpers.getBotPoseEstimate_wpiBlue(TEST_LIMELIGHT_NAME);

            assertNotNull(poseEstimate);
            assertEquals(1.5, poseEstimate.pose.getX(), 1e-9);
            assertEquals(-2.25, poseEstimate.pose.getY(), 1e-9);
            assertEquals(Math.PI / 2.0, poseEstimate.pose.getRotation().getRadians(), 1e-9);
            assertEquals(
                    (publishedTimestampNtLocalMicros / 1_000_000.0) - (latencyMilliseconds / 1_000.0),
                    poseEstimate.timestampNtLocalSeconds,
                    1e-9);
        }
    }
}
