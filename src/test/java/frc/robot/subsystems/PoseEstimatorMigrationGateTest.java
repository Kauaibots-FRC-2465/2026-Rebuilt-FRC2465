package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotController;
import org.junit.jupiter.api.Test;

class PoseEstimatorMigrationGateTest {
    private static final double EPSILON = 1e-9;

    @Test
    void freshEstimatorHasNoPredictionAndNoFusedPose() {
        MutableOdometrySource odometrySource = new MutableOdometrySource();
        odometrySource.valid = false;
        PoseEstimatorSubsystem poseEstimator = createPoseEstimator(odometrySource);
        PoseEstimatorSubsystem.PredictedFusedState predictedState =
                new PoseEstimatorSubsystem.PredictedFusedState();

        assertFalse(poseEstimator.getPredictedFusedState(0.0, predictedState));
        assertFalse(predictedState.isValid());
        assertNull(poseEstimator.getFusedPoseSupplier().get());
    }

    @Test
    void resetPoseProvidesImmediatePredictionAtResetPose() {
        MutableOdometrySource odometrySource = new MutableOdometrySource();
        PoseEstimatorSubsystem poseEstimator = createPoseEstimator(odometrySource);
        Pose2d resetPoseFldMetersRadians = new Pose2d(1.25, -0.75, Rotation2d.fromDegrees(32.0));
        PoseEstimatorSubsystem.PredictedFusedState predictedState =
                new PoseEstimatorSubsystem.PredictedFusedState();

        poseEstimator.resetPose(resetPoseFldMetersRadians);

        assertTrue(poseEstimator.getPredictedFusedState(0.0, predictedState));
        assertTrue(predictedState.isValid());
        assertEquals(resetPoseFldMetersRadians.getX(), predictedState.xMeters, EPSILON);
        assertEquals(resetPoseFldMetersRadians.getY(), predictedState.yMeters, EPSILON);
        assertEquals(
                resetPoseFldMetersRadians.getRotation().getRadians(),
                predictedState.headingRadians,
                EPSILON);
        assertEquals(0.0, predictedState.vxMetersPerSecond, EPSILON);
        assertEquals(0.0, predictedState.vyMetersPerSecond, EPSILON);
        assertEquals(0.0, predictedState.omegaRadiansPerSecond, EPSILON);

        Pose2d fusedPoseFldMetersRadians = poseEstimator.getFusedPoseSupplier().get();
        assertNotNull(fusedPoseFldMetersRadians);
        assertEquals(resetPoseFldMetersRadians.getX(), fusedPoseFldMetersRadians.getX(), EPSILON);
        assertEquals(resetPoseFldMetersRadians.getY(), fusedPoseFldMetersRadians.getY(), EPSILON);
        assertEquals(
                resetPoseFldMetersRadians.getRotation().getRadians(),
                fusedPoseFldMetersRadians.getRotation().getRadians(),
                EPSILON);
    }

    @Test
    void stationaryResetPoseRemainsStationaryForPositiveLookahead() {
        MutableOdometrySource odometrySource = new MutableOdometrySource();
        PoseEstimatorSubsystem poseEstimator = createPoseEstimator(odometrySource);
        Pose2d resetPoseFldMetersRadians = new Pose2d(-0.5, 2.0, Rotation2d.fromDegrees(-90.0));
        PoseEstimatorSubsystem.PredictedFusedState predictedState =
                new PoseEstimatorSubsystem.PredictedFusedState();

        poseEstimator.resetPose(resetPoseFldMetersRadians);

        assertTrue(poseEstimator.getPredictedFusedState(0.25, predictedState));
        assertEquals(resetPoseFldMetersRadians.getX(), predictedState.xMeters, EPSILON);
        assertEquals(resetPoseFldMetersRadians.getY(), predictedState.yMeters, EPSILON);
        assertEquals(
                resetPoseFldMetersRadians.getRotation().getRadians(),
                predictedState.headingRadians,
                EPSILON);
        assertEquals(0.0, predictedState.vxMetersPerSecond, EPSILON);
        assertEquals(0.0, predictedState.vyMetersPerSecond, EPSILON);
        assertEquals(0.0, predictedState.omegaRadiansPerSecond, EPSILON);
        assertTrue(
                predictedState.timestampFpgaMicros >= RobotController.getFPGATime() + 200_000L,
                "Expected positive-lookahead prediction timestamps to stay materially in the future");
    }

    @Test
    void predictedVelocitiesRemainFieldRelativeAcrossRobotHeading() {
        MutableOdometrySource odometrySource = new MutableOdometrySource();
        odometrySource.valid = true;
        PoseEstimatorSubsystem poseEstimator = createPoseEstimator(odometrySource);
        PoseEstimatorSubsystem.PredictedFusedState predictedState =
                new PoseEstimatorSubsystem.PredictedFusedState();

        long latestTimestampFpgaMicros = RobotController.getFPGATime() - 5_000L;
        long priorTimestampFpgaMicros = latestTimestampFpgaMicros - 20_000L;

        odometrySource.poseFldMetersRadians = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(90.0));
        odometrySource.timestampFpga = Microseconds.of(priorTimestampFpgaMicros);
        poseEstimator.periodic();

        odometrySource.poseFldMetersRadians = new Pose2d(0.0, 0.02, Rotation2d.fromDegrees(90.0));
        odometrySource.timestampFpga = Microseconds.of(latestTimestampFpgaMicros);
        poseEstimator.periodic();

        assertTrue(poseEstimator.getPredictedFusedState(0.0, predictedState));
        assertEquals(0.0, predictedState.vxMetersPerSecond, 1e-6);
        assertEquals(1.0, predictedState.vyMetersPerSecond, 1e-6);
        assertEquals(0.0, predictedState.omegaRadiansPerSecond, 1e-6);
        assertEquals(Math.PI / 2.0, predictedState.headingRadians, 1e-6);
    }

    private static PoseEstimatorSubsystem createPoseEstimator(MutableOdometrySource odometrySource) {
        PoseEstimatorSubsystem.Configuration configuration = new PoseEstimatorSubsystem.Configuration();
        configuration.odometryPose = odometrySource.poseSupplier;
        configuration.odometryTimestampFpga = odometrySource.timestampSupplier;
        configuration.odometryValid = odometrySource.validSupplier;
        configuration.odoLongitudinalDeviationPerDistance = 0.01;
        configuration.odoLateralDeviationPerDistance = 0.01;
        configuration.odoHeadingDeviationPerDistance = 0.01;
        configuration.odoLongitudinalDeviationPerRadian = 0.01;
        configuration.odoLateralDeviationPerRadian = 0.01;
        configuration.odoHeadingDeviationPerRadian = 0.01;
        configuration.visionPose = () -> null;
        configuration.visionTimestampNtLocal = Seconds::zero;
        configuration.visionIsValid = () -> false;
        configuration.visionXDeviation = finiteZeroSupplier();
        configuration.visionYDeviation = finiteZeroSupplier();
        configuration.visionThetaDeviation = finiteZeroSupplier();
        configuration.initialXDeviation = 1.0;
        configuration.initialYDeviation = 1.0;
        configuration.initialThetaDeviation = Math.toRadians(10.0);
        return new PoseEstimatorSubsystem(configuration);
    }

    private static DoubleSupplier finiteZeroSupplier() {
        return () -> 0.0;
    }

    private static final class MutableOdometrySource {
        private Pose2d poseFldMetersRadians = new Pose2d();
        private Time timestampFpga = Seconds.zero();
        private boolean valid = false;

        private final Supplier<Pose2d> poseSupplier = () -> poseFldMetersRadians;
        private final Supplier<Time> timestampSupplier = () -> timestampFpga;
        private final BooleanSupplier validSupplier = () -> valid;
    }
}
