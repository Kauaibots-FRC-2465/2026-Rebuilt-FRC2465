package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import frc.robot.subsystems.LimelightSubsystem;

class LimelightSubsystemMigrationGateTest {
    @Test
    void poseEstimateTimestampConvertsDirectlyToMicrosWithoutReprojection() {
        double poseEstimateTimestampSeconds = 1.234567;

        long poseEstimateTimestampMicros =
                LimelightSubsystem.poseEstimateTimestampSecondsToMicros(poseEstimateTimestampSeconds);

        assertEquals(1_234_567L, poseEstimateTimestampMicros);
    }
}
