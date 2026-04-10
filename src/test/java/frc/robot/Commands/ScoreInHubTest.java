package frc.robot.Commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

class ScoreInHubTest {
    private static final double MINIMUM_ALLOWED_RADIAL_SPEED_METERS_PER_SECOND = 0.5;
    private static final double MIN_TURRET_ANGLE_DEGREES = -180.0;
    private static final double MAX_TURRET_ANGLE_DEGREES = 180.0;
    private static final int[] CANDIDATE_LATCH_DISTANCE_ROW_INDEXES = {7, 6, 5, 4, 3, 2, 1, 0};
    private static final int LATCH_SHOT_COLUMN_INDEX = 2;
    private static final double STRICT_LATCH_DROP_TOLERANCE_METERS_PER_SECOND = 0.01;

    @Test
    void driverVelocityToFieldVelocityIsIdentityForBluePerspective() {
        Translation2d velocityDrvMetersPerSecond = new Translation2d(1.25, -0.75);
        Translation2d velocityFldMetersPerSecond = ScoreInHub.driverVelocityToFieldVelocity(
                velocityDrvMetersPerSecond,
                Rotation2d.kZero);

        assertEquals(velocityDrvMetersPerSecond.getX(), velocityFldMetersPerSecond.getX(), 1e-9);
        assertEquals(velocityDrvMetersPerSecond.getY(), velocityFldMetersPerSecond.getY(), 1e-9);
    }

    @Test
    void driverVelocityToFieldVelocityRotatesRedPerspectiveByOneEightyDegrees() {
        Translation2d velocityDrvMetersPerSecond = new Translation2d(1.0, 0.25);
        Translation2d velocityFldMetersPerSecond = ScoreInHub.driverVelocityToFieldVelocity(
                velocityDrvMetersPerSecond,
                Rotation2d.k180deg);

        assertEquals(-1.0, velocityFldMetersPerSecond.getX(), 1e-9);
        assertEquals(-0.25, velocityFldMetersPerSecond.getY(), 1e-9);
    }

    @Test
    void fieldAndDriverVelocityConversionsRoundTripForArbitraryPerspective() {
        Rotation2d driverPerspectiveForward = Rotation2d.fromDegrees(37.0);
        Translation2d originalVelocityDrvMetersPerSecond = new Translation2d(0.9, -1.7);
        Translation2d velocityFldMetersPerSecond = ScoreInHub.driverVelocityToFieldVelocity(
                originalVelocityDrvMetersPerSecond,
                driverPerspectiveForward);
        Translation2d roundTrippedVelocityDrvMetersPerSecond = ScoreInHub.fieldVelocityToDriverVelocity(
                velocityFldMetersPerSecond,
                driverPerspectiveForward);

        assertEquals(
                originalVelocityDrvMetersPerSecond.getX(),
                roundTrippedVelocityDrvMetersPerSecond.getX(),
                1e-9);
        assertEquals(
                originalVelocityDrvMetersPerSecond.getY(),
                roundTrippedVelocityDrvMetersPerSecond.getY(),
                1e-9);
    }

    @Test
    void fieldHeadingToDriverHeadingSubtractsDriverPerspectiveForward() {
        Rotation2d fieldHeadingFldRadians = Rotation2d.fromDegrees(180.0);
        Rotation2d driverHeadingDrvRadians = ScoreInHub.fieldHeadingToDriverHeading(
                fieldHeadingFldRadians,
                Rotation2d.k180deg);

        assertEquals(0.0, driverHeadingDrvRadians.getRadians(), 1e-9);
    }

    @Test
    void hasResolvedAimTreatsOnlyNoSolutionAsUnresolved() {
        assertTrue(ScoreInHub.hasResolvedAim(BallTrajectoryLookup.FixedFlywheelShotStatus.VALID));
        assertTrue(ScoreInHub.hasResolvedAim(BallTrajectoryLookup.FixedFlywheelShotStatus.TOO_SLOW));
        assertTrue(ScoreInHub.hasResolvedAim(BallTrajectoryLookup.FixedFlywheelShotStatus.TOO_FAST));
        assertTrue(!ScoreInHub.hasResolvedAim(BallTrajectoryLookup.FixedFlywheelShotStatus.NO_SOLUTION));
    }

    @Test
    void empiricalLookupDistanceShorteningDoesNotExpandDefaultApplicability() {
        double rawTargetDistanceInches =
                ShooterConstants.DATA_COLLECTION_SHORT_RANGE_EMPIRICAL_MAX_DISTANCE_INCHES
                        + ShooterConstants.COMMANDED_EMPIRICAL_MOVING_SHOT_LOOKUP_DISTANCE_SHORTENING_INCHES
                        - 1.0;

        assertTrue(
                !MovingShotMath.shouldUseEmpiricalHubMovingShotModel(
                        rawTargetDistanceInches,
                        ShooterConstants.COMMANDED_SCORE_IN_HUB_TARGET_ELEVATION_INCHES));
        assertTrue(
                !MovingShotMath.shouldUseEmpiricalHubMovingShotModel(
                        rawTargetDistanceInches,
                        ShooterConstants.COMMANDED_SCORE_IN_HUB_TARGET_ELEVATION_INCHES,
                        0.0));
        assertTrue(
                MovingShotMath.shouldUseEmpiricalHubMovingShotModel(
                        rawTargetDistanceInches,
                        ShooterConstants.COMMANDED_SCORE_IN_HUB_TARGET_ELEVATION_INCHES,
                        ShooterConstants.COMMANDED_EMPIRICAL_MOVING_SHOT_LOOKUP_DISTANCE_SHORTENING_INCHES));
    }

    @Test
    void empiricalLookupDistanceShorteningDoesNotMoveStationaryAimOffTarget() {
        int distanceRowIndex = 5;
        int shotColumnIndex = 2;
        double rawTargetDistanceInches = ShooterConstants.DATA_COLLECTION_SHORT_RANGE_DISTANCES_INCHES[distanceRowIndex];
        double preferredHoodAngleDegrees =
                ShooterConstants.DATA_COLLECTION_SHORT_RANGE_HOOD_ANGLES_DEGREES[distanceRowIndex][shotColumnIndex];
        double flywheelCommandIps =
                ShooterConstants.DATA_COLLECTION_SHORT_RANGE_COMMAND_SPEEDS_IPS[distanceRowIndex][shotColumnIndex];
        double targetAzimuthDegrees = 27.0;
        double targetAzimuthRadians = Math.toRadians(targetAzimuthDegrees);
        double targetXMeters = Inches.of(rawTargetDistanceInches * Math.cos(targetAzimuthRadians)).in(Meters);
        double targetYMeters = Inches.of(rawTargetDistanceInches * Math.sin(targetAzimuthRadians)).in(Meters);
        MovingShotMath.EmpiricalMovingShotDebugInfo defaultDebugInfo =
                new MovingShotMath.EmpiricalMovingShotDebugInfo();
        MovingShotMath.EmpiricalMovingShotDebugInfo legacyDebugInfo =
                new MovingShotMath.EmpiricalMovingShotDebugInfo();
        BallTrajectoryLookup.MovingShotSolution defaultSolution =
                new BallTrajectoryLookup.MovingShotSolution();
        BallTrajectoryLookup.MovingShotSolution legacySolution =
                new BallTrajectoryLookup.MovingShotSolution();

        boolean defaultSolved = MovingShotMath.solveIdealMovingShotWithUpperHoodFallback(
                ShooterConstants.COMMANDED_MINIMUM_ALLOWED_HOOD_ANGLE_DEGREES,
                ShooterConstants.COMMANDED_MAXIMUM_ALLOWED_HOOD_ANGLE_DEGREES,
                preferredHoodAngleDegrees,
                ShooterConstants.COMMANDED_MOVING_SHOT_HOOD_SEARCH_STEP_DEGREES,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                targetXMeters,
                targetYMeters,
                ShooterConstants.COMMANDED_SCORE_IN_HUB_TARGET_ELEVATION_INCHES,
                ShooterConstants.COMMANDED_MAXIMUM_SHOOTING_HEIGHT_INCHES,
                Rotation2d.kZero.getRadians(),
                MIN_TURRET_ANGLE_DEGREES,
                MAX_TURRET_ANGLE_DEGREES,
                flywheelCommandIps,
                flywheelCommandIps,
                defaultSolution,
                defaultDebugInfo);
        boolean legacySolved = MovingShotMath.solveIdealMovingShotWithUpperHoodFallback(
                ShooterConstants.COMMANDED_MINIMUM_ALLOWED_HOOD_ANGLE_DEGREES,
                ShooterConstants.COMMANDED_MAXIMUM_ALLOWED_HOOD_ANGLE_DEGREES,
                preferredHoodAngleDegrees,
                ShooterConstants.COMMANDED_MOVING_SHOT_HOOD_SEARCH_STEP_DEGREES,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                targetXMeters,
                targetYMeters,
                ShooterConstants.COMMANDED_SCORE_IN_HUB_TARGET_ELEVATION_INCHES,
                ShooterConstants.COMMANDED_MAXIMUM_SHOOTING_HEIGHT_INCHES,
                Rotation2d.kZero.getRadians(),
                MIN_TURRET_ANGLE_DEGREES,
                MAX_TURRET_ANGLE_DEGREES,
                flywheelCommandIps,
                flywheelCommandIps,
                legacySolution,
                legacyDebugInfo,
                0.0);

        assertTrue(defaultSolved);
        assertTrue(legacySolved);
        assertEquals(
                rawTargetDistanceInches
                        - ShooterConstants.COMMANDED_EMPIRICAL_MOVING_SHOT_LOOKUP_DISTANCE_SHORTENING_INCHES,
                defaultDebugInfo.getLookupTargetDistanceInches(),
                1e-9);
        assertEquals(rawTargetDistanceInches, legacyDebugInfo.getLookupTargetDistanceInches(), 1e-9);
        assertEquals(rawTargetDistanceInches, defaultSolution.getTargetRadialDistanceInches(), 1e-9);
        assertEquals(rawTargetDistanceInches, legacySolution.getTargetRadialDistanceInches(), 1e-9);
        assertEquals(
                defaultSolution.getTurretDeltaDegrees(),
                legacySolution.getTurretDeltaDegrees(),
                1e-9);
        assertEquals(
                defaultSolution.getShotAzimuthDegrees(),
                legacySolution.getShotAzimuthDegrees(),
                1e-9);
    }

    private static final class LatchSequenceResult {
        private final int[] distanceRowIndexes;
        private final double[] updatedMaximumTravelSpeedsMetersPerSecond;
        private final boolean[] viableShots;

        LatchSequenceResult(
                int[] distanceRowIndexes,
                double[] updatedMaximumTravelSpeedsMetersPerSecond,
                boolean[] viableShots) {
            this.distanceRowIndexes = distanceRowIndexes;
            this.updatedMaximumTravelSpeedsMetersPerSecond = updatedMaximumTravelSpeedsMetersPerSecond;
            this.viableShots = viableShots;
        }

        int[] getDistanceRowIndexes() {
            return distanceRowIndexes;
        }

        double[] getUpdatedMaximumTravelSpeedsMetersPerSecond() {
            return updatedMaximumTravelSpeedsMetersPerSecond;
        }

        boolean[] getViableShots() {
            return viableShots;
        }
    }

    private static final class LatchSequenceScenario {
        private final int[] distanceRowIndexes;
        private final double requestedTravelSpeedMetersPerSecond;

        LatchSequenceScenario(int[] distanceRowIndexes, double requestedTravelSpeedMetersPerSecond) {
            this.distanceRowIndexes = distanceRowIndexes;
            this.requestedTravelSpeedMetersPerSecond = requestedTravelSpeedMetersPerSecond;
        }

        int[] getDistanceRowIndexes() {
            return distanceRowIndexes;
        }

        double getRequestedTravelSpeedMetersPerSecond() {
            return requestedTravelSpeedMetersPerSecond;
        }
    }

    @Test
    void travelSpeedLimiterDropsAngledInwardSpeedWhenRequestedSpeedExceedsEmpiricalEnvelope() {
        int distanceRowIndex = 5;
        int shotColumnIndex = 2;
        double targetDistanceInches = ShooterConstants.DATA_COLLECTION_SHORT_RANGE_DISTANCES_INCHES[distanceRowIndex];
        double preferredHoodAngleDegrees =
                ShooterConstants.DATA_COLLECTION_SHORT_RANGE_HOOD_ANGLES_DEGREES[distanceRowIndex][shotColumnIndex];
        double currentFlywheelSpeedIps =
                ShooterConstants.DATA_COLLECTION_SHORT_RANGE_COMMAND_SPEEDS_IPS[distanceRowIndex][shotColumnIndex];
        Translation2d futureRobotPosition = new Translation2d();
        Translation2d target = new Translation2d(Inches.of(targetDistanceInches).in(Meters), 0.0);
        Translation2d inwardLeftTravelUnitVector = new Translation2d(1.0, 1.0).times(1.0 / Math.sqrt(2.0));
        ScoreInHub.HubShotViabilityEvaluator empiricalViabilityEvaluator =
                (robotFieldVxMetersPerSecond, robotFieldVyMetersPerSecond) -> {
                    BallTrajectoryLookup.MovingShotSolution solution =
                            new BallTrajectoryLookup.MovingShotSolution();
                    return MovingShotMath.solveIdealMovingShotWithUpperHoodFallback(
                            ShooterConstants.COMMANDED_MINIMUM_ALLOWED_HOOD_ANGLE_DEGREES,
                            ShooterConstants.COMMANDED_MAXIMUM_ALLOWED_HOOD_ANGLE_DEGREES,
                            preferredHoodAngleDegrees,
                            ShooterConstants.COMMANDED_MOVING_SHOT_HOOD_SEARCH_STEP_DEGREES,
                            futureRobotPosition.getX(),
                            futureRobotPosition.getY(),
                            0.0,
                            robotFieldVxMetersPerSecond,
                            robotFieldVyMetersPerSecond,
                            target.getX(),
                            target.getY(),
                            ShooterConstants.COMMANDED_SCORE_IN_HUB_TARGET_ELEVATION_INCHES,
                            ShooterConstants.COMMANDED_MAXIMUM_SHOOTING_HEIGHT_INCHES,
                            Rotation2d.kZero.getRadians(),
                            MIN_TURRET_ANGLE_DEGREES,
                            MAX_TURRET_ANGLE_DEGREES,
                            currentFlywheelSpeedIps,
                            currentFlywheelSpeedIps,
                            solution);
                };
        double requestedTravelSpeedMetersPerSecond = findFirstUnviableTravelSpeedMetersPerSecond(
                empiricalViabilityEvaluator,
                inwardLeftTravelUnitVector,
                MINIMUM_ALLOWED_RADIAL_SPEED_METERS_PER_SECOND);
        Translation2d requestedVelocityMetersPerSecond =
                inwardLeftTravelUnitVector.times(requestedTravelSpeedMetersPerSecond);

        assertTrue(
                empiricalViabilityEvaluator.hasViableShot(
                        inwardLeftTravelUnitVector.getX() * MINIMUM_ALLOWED_RADIAL_SPEED_METERS_PER_SECOND,
                        inwardLeftTravelUnitVector.getY() * MINIMUM_ALLOWED_RADIAL_SPEED_METERS_PER_SECOND),
                "Expected the empirical hub solve to remain viable at the minimum allowed travel speed");
        assertTrue(
                !empiricalViabilityEvaluator.hasViableShot(
                        requestedVelocityMetersPerSecond.getX(),
                        requestedVelocityMetersPerSecond.getY()),
                "Expected the requested angled travel speed to exceed the empirical hub envelope");

        ScoreInHub.TravelVelocityLimitResult limitResult = ScoreInHub.limitTowardHubTravelVelocityForViableShot(
                requestedVelocityMetersPerSecond,
                futureRobotPosition,
                target,
                MINIMUM_ALLOWED_RADIAL_SPEED_METERS_PER_SECOND,
                Double.POSITIVE_INFINITY,
                empiricalViabilityEvaluator);

        assertTrue(
                limitResult.getUpdatedMaximumTowardHubTravelSpeedMetersPerSecond()
                        < requestedTravelSpeedMetersPerSecond,
                "Expected the latched maximum toward-hub travel speed to drop below the requested speed");
        assertTrue(
                limitResult.getUpdatedMaximumTowardHubTravelSpeedMetersPerSecond()
                        >= MINIMUM_ALLOWED_RADIAL_SPEED_METERS_PER_SECOND - 1e-9,
                "Expected the latched maximum toward-hub travel speed to stay above the minimum allowed speed");
        assertTrue(
                limitResult.getLimitedVelocityFldMetersPerSecond().getNorm() < requestedTravelSpeedMetersPerSecond,
                "Expected the limiter to reduce the requested angled travel speed");
        assertEquals(
                requestedVelocityMetersPerSecond.getAngle().getRadians(),
                limitResult.getLimitedVelocityFldMetersPerSecond().getAngle().getRadians(),
                1e-9,
                "Expected the limiter to preserve travel direction while reducing speed");
        assertTrue(
                empiricalViabilityEvaluator.hasViableShot(
                        limitResult.getLimitedVelocityFldMetersPerSecond().getX(),
                        limitResult.getLimitedVelocityFldMetersPerSecond().getY()),
                "Expected the limited angled travel speed to be empirically viable");
    }

    @Test
    void travelSpeedLimiterResetsLatchWhenDrivingAwayFromHub() {
        Translation2d futureRobotPosition = new Translation2d();
        Translation2d target = new Translation2d(1.0, 0.0);
        Translation2d requestedAwayVelocityMetersPerSecond = new Translation2d(-1.0, 0.25);

        ScoreInHub.TravelVelocityLimitResult limitResult = ScoreInHub.limitTowardHubTravelVelocityForViableShot(
                requestedAwayVelocityMetersPerSecond,
                futureRobotPosition,
                target,
                MINIMUM_ALLOWED_RADIAL_SPEED_METERS_PER_SECOND,
                0.6,
                (robotFieldVxMetersPerSecond, robotFieldVyMetersPerSecond) -> false);

        assertEquals(
                requestedAwayVelocityMetersPerSecond.getX(),
                limitResult.getLimitedVelocityFldMetersPerSecond().getX(),
                1e-9);
        assertEquals(
                requestedAwayVelocityMetersPerSecond.getY(),
                limitResult.getLimitedVelocityFldMetersPerSecond().getY(),
                1e-9);
        assertTrue(
                Double.isInfinite(limitResult.getUpdatedMaximumTowardHubTravelSpeedMetersPerSecond()),
                "Expected an away-from-hub command to reset the latched travel-speed limit");
    }

    @Test
    void travelSpeedLatchDropsAcrossDistancesResetsAndRepeatsForPureRadialApproach() {
        assertLatchSequenceRepeatsAfterAwayReset(new Translation2d(1.0, 0.0));
    }

    @Test
    void travelSpeedLatchDropsAcrossDistancesResetsAndRepeatsForAngledApproach() {
        assertLatchSequenceRepeatsAfterAwayReset(new Translation2d(1.0, 1.0).times(1.0 / Math.sqrt(2.0)));
    }

    @Test
    void straightTravelSpeedEnvelopeDoesNotCollapseAtFormerlySuspiciousInterpolatedDistances() {
        Translation2d straightTravelUnitVector = new Translation2d(1.0, 0.0);
        double maximumTravelSpeedAt230Inches = ScoreInHubSpeedEnvelopeReportTest.findMaximumTravelSpeedMetersPerSecond(
                230.0,
                straightTravelUnitVector);
        double maximumTravelSpeedAt221Inches = ScoreInHubSpeedEnvelopeReportTest.findMaximumTravelSpeedMetersPerSecond(
                221.0,
                straightTravelUnitVector);
        double maximumTravelSpeedAt212Inches = ScoreInHubSpeedEnvelopeReportTest.findMaximumTravelSpeedMetersPerSecond(
                212.0,
                straightTravelUnitVector);
        double maximumTravelSpeedAt167Inches = ScoreInHubSpeedEnvelopeReportTest.findMaximumTravelSpeedMetersPerSecond(
                167.0,
                straightTravelUnitVector);
        double maximumTravelSpeedAt158Inches = ScoreInHubSpeedEnvelopeReportTest.findMaximumTravelSpeedMetersPerSecond(
                158.0,
                straightTravelUnitVector);
        double maximumTravelSpeedAt149Inches = ScoreInHubSpeedEnvelopeReportTest.findMaximumTravelSpeedMetersPerSecond(
                149.0,
                straightTravelUnitVector);

        assertTrue(
                maximumTravelSpeedAt221Inches
                        >= 0.70 * Math.min(maximumTravelSpeedAt230Inches, maximumTravelSpeedAt212Inches),
                () -> String.format(
                        "Expected 221 in straight max speed %.6f to stay reasonably close to neighbors 230 %.6f / 212 %.6f",
                        maximumTravelSpeedAt221Inches,
                        maximumTravelSpeedAt230Inches,
                        maximumTravelSpeedAt212Inches));
        assertTrue(
                maximumTravelSpeedAt158Inches
                        >= 0.70 * Math.min(maximumTravelSpeedAt167Inches, maximumTravelSpeedAt149Inches),
                () -> String.format(
                        "Expected 158 in straight max speed %.6f to stay reasonably close to neighbors 167 %.6f / 149 %.6f",
                        maximumTravelSpeedAt158Inches,
                        maximumTravelSpeedAt167Inches,
                        maximumTravelSpeedAt149Inches));
        assertTrue(
                maximumTravelSpeedAt221Inches > MINIMUM_ALLOWED_RADIAL_SPEED_METERS_PER_SECOND + 0.5,
                () -> String.format(
                        "Expected 221 in straight max speed %.6f to stay materially above the floor",
                        maximumTravelSpeedAt221Inches));
        assertTrue(
                maximumTravelSpeedAt158Inches > MINIMUM_ALLOWED_RADIAL_SPEED_METERS_PER_SECOND + 0.5,
                () -> String.format(
                        "Expected 158 in straight max speed %.6f to stay materially above the floor",
                        maximumTravelSpeedAt158Inches));
    }

    private static double findFirstUnviableTravelSpeedMetersPerSecond(
            ScoreInHub.HubShotViabilityEvaluator empiricalViabilityEvaluator,
            Translation2d travelUnitVector,
            double minimumAllowedRadialSpeedMetersPerSecond) {
        for (double candidateSpeedMetersPerSecond = minimumAllowedRadialSpeedMetersPerSecond + 0.25;
                candidateSpeedMetersPerSecond <= 40.0;
                candidateSpeedMetersPerSecond += 0.25) {
            Translation2d candidateVelocityMetersPerSecond = travelUnitVector.times(candidateSpeedMetersPerSecond);
            if (!empiricalViabilityEvaluator.hasViableShot(
                    candidateVelocityMetersPerSecond.getX(),
                    candidateVelocityMetersPerSecond.getY())) {
                return candidateSpeedMetersPerSecond;
            }
        }
        throw new AssertionError("Expected to find an angled travel speed that exceeds the empirical envelope");
    }

    private static void assertLatchSequenceRepeatsAfterAwayReset(Translation2d travelUnitVector) {
        LatchSequenceScenario scenario = findLatchSequenceScenario(travelUnitVector);
        double requestedTravelSpeedMetersPerSecond = scenario.getRequestedTravelSpeedMetersPerSecond();
        LatchSequenceResult firstForwardSequence = runLatchSequence(
                scenario.getDistanceRowIndexes(),
                travelUnitVector,
                requestedTravelSpeedMetersPerSecond,
                Double.POSITIVE_INFINITY);

        assertAllLatchStepsViable(firstForwardSequence);
        assertLatchStrictlyDrops(
                firstForwardSequence.getUpdatedMaximumTravelSpeedsMetersPerSecond(),
                requestedTravelSpeedMetersPerSecond);

        double closestTargetDistanceInches =
                ShooterConstants.DATA_COLLECTION_SHORT_RANGE_DISTANCES_INCHES[
                        scenario.getDistanceRowIndexes()[scenario.getDistanceRowIndexes().length - 1]];
        Translation2d requestedAwayVelocityMetersPerSecond = travelUnitVector.times(-requestedTravelSpeedMetersPerSecond);
        ScoreInHub.TravelVelocityLimitResult awayResetResult = ScoreInHub.limitTowardHubTravelVelocityForViableShot(
                requestedAwayVelocityMetersPerSecond,
                new Translation2d(),
                new Translation2d(Inches.of(closestTargetDistanceInches).in(Meters), 0.0),
                MINIMUM_ALLOWED_RADIAL_SPEED_METERS_PER_SECOND,
                firstForwardSequence.getUpdatedMaximumTravelSpeedsMetersPerSecond()
                        [firstForwardSequence.getUpdatedMaximumTravelSpeedsMetersPerSecond().length - 1],
                (robotFieldVxMetersPerSecond, robotFieldVyMetersPerSecond) -> false);

        assertEquals(
                requestedAwayVelocityMetersPerSecond.getX(),
                awayResetResult.getLimitedVelocityFldMetersPerSecond().getX(),
                1e-9);
        assertEquals(
                requestedAwayVelocityMetersPerSecond.getY(),
                awayResetResult.getLimitedVelocityFldMetersPerSecond().getY(),
                1e-9);
        assertTrue(
                Double.isInfinite(awayResetResult.getUpdatedMaximumTowardHubTravelSpeedMetersPerSecond()),
                "Expected reversing away from the hub to reset the latched travel-speed limit");

        LatchSequenceResult secondForwardSequence = runLatchSequence(
                scenario.getDistanceRowIndexes(),
                travelUnitVector,
                requestedTravelSpeedMetersPerSecond,
                awayResetResult.getUpdatedMaximumTowardHubTravelSpeedMetersPerSecond());

        assertAllLatchStepsViable(secondForwardSequence);
        for (int i = 0; i < firstForwardSequence.getUpdatedMaximumTravelSpeedsMetersPerSecond().length; i++) {
            assertEquals(
                    firstForwardSequence.getUpdatedMaximumTravelSpeedsMetersPerSecond()[i],
                    secondForwardSequence.getUpdatedMaximumTravelSpeedsMetersPerSecond()[i],
                    1e-9,
                    "Expected the same latched-speed pattern after resetting by backing away");
        }
    }

    private static LatchSequenceScenario findLatchSequenceScenario(
            Translation2d travelUnitVector) {
        List<int[]> candidateSequences = buildCandidateDistanceRowSequences();
        for (double candidateTravelSpeedMetersPerSecond = MINIMUM_ALLOWED_RADIAL_SPEED_METERS_PER_SECOND + 0.25;
                candidateTravelSpeedMetersPerSecond <= 40.0;
                candidateTravelSpeedMetersPerSecond += 0.25) {
            for (int[] candidateDistanceRowIndexes : candidateSequences) {
                LatchSequenceResult sequenceResult = runLatchSequence(
                        candidateDistanceRowIndexes,
                        travelUnitVector,
                        candidateTravelSpeedMetersPerSecond,
                        Double.POSITIVE_INFINITY);
                if (doesLatchStrictlyDrop(
                        sequenceResult.getUpdatedMaximumTravelSpeedsMetersPerSecond(),
                        candidateTravelSpeedMetersPerSecond)
                        && areAllLatchStepsViable(sequenceResult)) {
                    return new LatchSequenceScenario(
                            candidateDistanceRowIndexes,
                            candidateTravelSpeedMetersPerSecond);
                }
            }
        }
        throw new AssertionError(
                "Expected to find a fixed travel speed and measured-distance sequence whose latch drops repeatedly");
    }

    private static LatchSequenceResult runLatchSequence(
            int[] distanceRowIndexes,
            Translation2d travelUnitVector,
            double requestedTravelSpeedMetersPerSecond,
            double startingMaximumTravelSpeedMetersPerSecond) {
        Translation2d requestedVelocityMetersPerSecond = travelUnitVector.times(requestedTravelSpeedMetersPerSecond);
        double currentMaximumTravelSpeedMetersPerSecond = startingMaximumTravelSpeedMetersPerSecond;
        double[] updatedMaximumTravelSpeedsMetersPerSecond = new double[distanceRowIndexes.length];
        boolean[] viableShots = new boolean[distanceRowIndexes.length];

        for (int i = 0; i < distanceRowIndexes.length; i++) {
            int distanceRowIndex = distanceRowIndexes[i];
            double targetDistanceInches = ShooterConstants.DATA_COLLECTION_SHORT_RANGE_DISTANCES_INCHES[distanceRowIndex];
            Translation2d target = new Translation2d(Inches.of(targetDistanceInches).in(Meters), 0.0);
            ScoreInHub.HubShotViabilityEvaluator empiricalViabilityEvaluator =
                    createEmpiricalViabilityEvaluator(distanceRowIndex, LATCH_SHOT_COLUMN_INDEX, target);
            ScoreInHub.TravelVelocityLimitResult limitResult = ScoreInHub.limitTowardHubTravelVelocityForViableShot(
                    requestedVelocityMetersPerSecond,
                    new Translation2d(),
                    target,
                    MINIMUM_ALLOWED_RADIAL_SPEED_METERS_PER_SECOND,
                    currentMaximumTravelSpeedMetersPerSecond,
                    empiricalViabilityEvaluator);

            updatedMaximumTravelSpeedsMetersPerSecond[i] =
                    limitResult.getUpdatedMaximumTowardHubTravelSpeedMetersPerSecond();
            currentMaximumTravelSpeedMetersPerSecond =
                    limitResult.getUpdatedMaximumTowardHubTravelSpeedMetersPerSecond();
            viableShots[i] = empiricalViabilityEvaluator.hasViableShot(
                    limitResult.getLimitedVelocityFldMetersPerSecond().getX(),
                    limitResult.getLimitedVelocityFldMetersPerSecond().getY());

            assertTrue(
                    limitResult.getLimitedVelocityFldMetersPerSecond().getNorm()
                            <= requestedTravelSpeedMetersPerSecond + 1e-9,
                    "Expected the limited speed to stay at or below the requested travel speed");
            if (limitResult.getLimitedVelocityFldMetersPerSecond().getNorm() > 1e-9) {
                assertEquals(
                        requestedVelocityMetersPerSecond.getAngle().getRadians(),
                        limitResult.getLimitedVelocityFldMetersPerSecond().getAngle().getRadians(),
                        1e-9,
                        "Expected the limiter to preserve travel direction across the latched sequence");
            }
        }

        return new LatchSequenceResult(distanceRowIndexes, updatedMaximumTravelSpeedsMetersPerSecond, viableShots);
    }

    private static List<int[]> buildCandidateDistanceRowSequences() {
        ArrayList<int[]> sequences = new ArrayList<>();
        for (int i = 0; i < CANDIDATE_LATCH_DISTANCE_ROW_INDEXES.length; i++) {
            for (int j = i + 1; j < CANDIDATE_LATCH_DISTANCE_ROW_INDEXES.length; j++) {
                for (int k = j + 1; k < CANDIDATE_LATCH_DISTANCE_ROW_INDEXES.length; k++) {
                    sequences.add(new int[] {
                        CANDIDATE_LATCH_DISTANCE_ROW_INDEXES[i],
                        CANDIDATE_LATCH_DISTANCE_ROW_INDEXES[j],
                        CANDIDATE_LATCH_DISTANCE_ROW_INDEXES[k]
                    });
                }
            }
        }
        for (int i = 0; i < CANDIDATE_LATCH_DISTANCE_ROW_INDEXES.length; i++) {
            for (int j = i + 1; j < CANDIDATE_LATCH_DISTANCE_ROW_INDEXES.length; j++) {
                sequences.add(new int[] {
                    CANDIDATE_LATCH_DISTANCE_ROW_INDEXES[i],
                    CANDIDATE_LATCH_DISTANCE_ROW_INDEXES[j]
                });
            }
        }
        return sequences;
    }

    private static ScoreInHub.HubShotViabilityEvaluator createEmpiricalViabilityEvaluator(
            int distanceRowIndex,
            int shotColumnIndex,
            Translation2d target) {
        double preferredHoodAngleDegrees =
                ShooterConstants.DATA_COLLECTION_SHORT_RANGE_HOOD_ANGLES_DEGREES[distanceRowIndex][shotColumnIndex];
        double currentFlywheelSpeedIps =
                ShooterConstants.DATA_COLLECTION_SHORT_RANGE_COMMAND_SPEEDS_IPS[distanceRowIndex][shotColumnIndex];
        Translation2d futureRobotPosition = new Translation2d();
        return (robotFieldVxMetersPerSecond, robotFieldVyMetersPerSecond) -> {
            BallTrajectoryLookup.MovingShotSolution solution =
                    new BallTrajectoryLookup.MovingShotSolution();
            return MovingShotMath.solveIdealMovingShotWithUpperHoodFallback(
                    ShooterConstants.COMMANDED_MINIMUM_ALLOWED_HOOD_ANGLE_DEGREES,
                    ShooterConstants.COMMANDED_MAXIMUM_ALLOWED_HOOD_ANGLE_DEGREES,
                    preferredHoodAngleDegrees,
                    ShooterConstants.COMMANDED_MOVING_SHOT_HOOD_SEARCH_STEP_DEGREES,
                    futureRobotPosition.getX(),
                    futureRobotPosition.getY(),
                    0.0,
                    robotFieldVxMetersPerSecond,
                    robotFieldVyMetersPerSecond,
                    target.getX(),
                    target.getY(),
                    ShooterConstants.COMMANDED_SCORE_IN_HUB_TARGET_ELEVATION_INCHES,
                    ShooterConstants.COMMANDED_MAXIMUM_SHOOTING_HEIGHT_INCHES,
                    Rotation2d.kZero.getRadians(),
                    MIN_TURRET_ANGLE_DEGREES,
                    MAX_TURRET_ANGLE_DEGREES,
                    currentFlywheelSpeedIps,
                    currentFlywheelSpeedIps,
                    solution);
        };
    }

    private static boolean doesLatchStrictlyDrop(
            double[] updatedMaximumTravelSpeedsMetersPerSecond,
            double requestedTravelSpeedMetersPerSecond) {
        if (updatedMaximumTravelSpeedsMetersPerSecond.length == 0
                || !(updatedMaximumTravelSpeedsMetersPerSecond[0]
                        < requestedTravelSpeedMetersPerSecond - STRICT_LATCH_DROP_TOLERANCE_METERS_PER_SECOND)) {
            return false;
        }

        for (int i = 1; i < updatedMaximumTravelSpeedsMetersPerSecond.length; i++) {
            if (!(updatedMaximumTravelSpeedsMetersPerSecond[i]
                    < updatedMaximumTravelSpeedsMetersPerSecond[i - 1]
                            - STRICT_LATCH_DROP_TOLERANCE_METERS_PER_SECOND)) {
                return false;
            }
        }
        return true;
    }

    private static void assertLatchStrictlyDrops(
            double[] updatedMaximumTravelSpeedsMetersPerSecond,
            double requestedTravelSpeedMetersPerSecond) {
        assertTrue(
                doesLatchStrictlyDrop(
                updatedMaximumTravelSpeedsMetersPerSecond,
                requestedTravelSpeedMetersPerSecond),
                "Expected the latched maximum travel speed to drop at each closer empirical distance");
    }

    private static boolean areAllLatchStepsViable(LatchSequenceResult sequenceResult) {
        for (boolean viableShot : sequenceResult.getViableShots()) {
            if (!viableShot) {
                return false;
            }
        }
        return true;
    }

    private static void assertAllLatchStepsViable(LatchSequenceResult sequenceResult) {
        assertTrue(
                areAllLatchStepsViable(sequenceResult),
                "Expected each step in the latched-speed sequence to remain empirically viable");
    }
}
