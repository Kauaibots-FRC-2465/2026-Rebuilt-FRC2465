package frc.robot.Commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.Random;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

class MovingShotMathTest {
    private static final long RANDOM_SHOOT_ON_THE_FLY_SEED = 2465L;
    private static final int RANDOM_SHOOT_ON_THE_FLY_SAMPLE_COUNT = 200;
    private static final int RANDOM_SHOOT_ON_THE_FLY_MAX_ATTEMPTS = 5000;
    private static final double MAX_REASONABLE_SHOT_AZIMUTH_ERROR_DEGREES = 60.0;
    private static final double SEARCH_STEP_DEGREES =
            ShooterConstants.COMMANDED_MOVING_SHOT_FIXED_FLYWHEEL_HOOD_SEARCH_STEP_DEGREES;
    private static final double TARGET_ELEVATION_INCHES =
            ShooterConstants.COMMANDED_SCORE_IN_HUB_TARGET_ELEVATION_INCHES;
    private static final double MAX_HEIGHT_INCHES =
            ShooterConstants.COMMANDED_TEST_SHOOTING_MAXIMUM_HEIGHT_INCHES;
    private static final double MAX_ROBOT_SPEED_METERS_PER_SECOND = 2.0;
    private static final double MIN_RANDOM_VALID_DISTANCE_INCHES =
            ShooterConstants.DATA_COLLECTION_SHORT_RANGE_DISTANCES_INCHES[2];
    private static final double MIN_TURRET_ANGLE_DEGREES = -180.0;
    private static final double MAX_TURRET_ANGLE_DEGREES = 180.0;
    private static final double INTERIOR_HOOD_EDGE_MARGIN_DEGREES = 0.001;
    private static final double MANIFOLD_SPEED_TOLERANCE_IPS = 1e-3;
    private static final double EMPIRICAL_PHYSICS_LANDING_TOLERANCE_INCHES = 30.0;
    private static final double PHYSICS_SIMULATION_DT_SECONDS = 0.002;
    private static final double PHYSICS_MAX_SIMULATION_TIME_SECONDS = 5.0;
    private static final double GRAVITY_IPS2 = 386.08858267716535;
    private static final double ANGLED_PHYSICS_TANGENTIAL_TO_RADIAL_RATIO = 0.2;
    private static final double LOCAL_CONTINUITY_DISTANCE_STEP_INCHES = 1.0;
    private static final double LOCAL_CONTINUITY_MAX_HOOD_STEP_DEGREES = 8.0;
    private static final double LOCAL_CONTINUITY_MAX_FLYWHEEL_STEP_IPS = 40.0;
    private static final double LOCAL_CONTINUITY_MAX_AZIMUTH_STEP_DEGREES = 8.0;
    private static final double LOCAL_CONTINUITY_MAX_SPEED_LIMIT_STEP_METERS_PER_SECOND = 0.60;
    private static final double MINIMUM_ALLOWED_TRAVEL_SPEED_METERS_PER_SECOND = 0.5;
    private static final double VIABLE_SPEED_MARGIN = 0.90;
    private static final double GLOBAL_MIN_EMPIRICAL_FLYWHEEL_IPS =
            computeGlobalEmpiricalFlywheelBound(true);
    private static final double GLOBAL_MAX_EMPIRICAL_FLYWHEEL_IPS =
            computeGlobalEmpiricalFlywheelBound(false);

    @Test
    void closestHoodFallbackClampsToInterpolatedEmpiricalEnvelopeBoundary() {
        double targetDistanceInches = 0.5 * (
                ShooterConstants.DATA_COLLECTION_SHORT_RANGE_DISTANCES_INCHES[0]
                        + ShooterConstants.DATA_COLLECTION_SHORT_RANGE_DISTANCES_INCHES[1]);
        double expectedMinimumHoodAngleDegrees =
                ShortRangeHubFlywheelLookup.getMinimumHoodAngleDegrees(targetDistanceInches);
        double preferredHoodAngleDegrees = expectedMinimumHoodAngleDegrees - 5.0;
        BallTrajectoryLookup.MovingShotSolution solution = new BallTrajectoryLookup.MovingShotSolution();

        boolean solved = MovingShotMath.solveMovingShotAtClosestHoodAngle(
                preferredHoodAngleDegrees,
                ShooterConstants.COMMANDED_MINIMUM_ALLOWED_HOOD_ANGLE_DEGREES,
                ShooterConstants.COMMANDED_MAXIMUM_ALLOWED_HOOD_ANGLE_DEGREES,
                SEARCH_STEP_DEGREES,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                Inches.of(targetDistanceInches).in(Meters),
                0.0,
                TARGET_ELEVATION_INCHES,
                MAX_HEIGHT_INCHES,
                0.0,
                MIN_TURRET_ANGLE_DEGREES,
                MAX_TURRET_ANGLE_DEGREES,
                solution);

        assertTrue(solved, "Expected an empirical solution after clamping to the interpolated hood envelope");
        assertEquals(
                expectedMinimumHoodAngleDegrees,
                solution.getHoodAngleDegrees(),
                1e-6,
                "Empirical fallback should clamp to the interpolated minimum hood angle");
    }

    @Test
    void empiricalMovingShotUsesModeledLaunchAngleAndExitSpeed() {
        double targetDistanceInches = ShooterConstants.DATA_COLLECTION_SHORT_RANGE_DISTANCES_INCHES[2];
        double hoodAngleDegrees = ShooterConstants.DATA_COLLECTION_SHORT_RANGE_HOOD_ANGLES_DEGREES[2][1];
        double flywheelCommandIps = ShooterConstants.DATA_COLLECTION_SHORT_RANGE_COMMAND_SPEEDS_IPS[2][1];
        BallTrajectoryLookup.MovingShotSolution solution = new BallTrajectoryLookup.MovingShotSolution();
        double expectedExitVelocityIps =
                BallTrajectoryLookup.getEstimatedExitVelocityIpsForCommandedShot(
                        hoodAngleDegrees,
                        flywheelCommandIps);

        boolean solved = MovingShotMath.populateEmpiricalMovingShotSolution(
                hoodAngleDegrees,
                flywheelCommandIps,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                Inches.of(targetDistanceInches).in(Meters),
                0.0,
                TARGET_ELEVATION_INCHES,
                MAX_HEIGHT_INCHES,
                0.0,
                MIN_TURRET_ANGLE_DEGREES,
                MAX_TURRET_ANGLE_DEGREES,
                solution);

        assertTrue(solved, "Expected a valid stationary empirical moving-shot solution");
        assertEquals(flywheelCommandIps, solution.getFlywheelCommandIps(), 1e-9);
        assertEquals(expectedExitVelocityIps, solution.getLauncherRelativeExitVelocityIps(), 1e-9,
                "Empirical interpolation path should use the modeled launch-speed estimate");
        assertEquals(expectedExitVelocityIps, solution.getFieldRelativeExitVelocityIps(), 1e-9,
                "Stationary empirical path should keep field-relative speed equal to modeled exit speed");
    }

    @Test
    void empiricalMovingShotClampsModeledFlywheelSpeedIntoEstimatedRange() {
        double targetDistanceInches = ShooterConstants.DATA_COLLECTION_SHORT_RANGE_DISTANCES_INCHES[2];
        double hoodAngleDegrees = ShooterConstants.DATA_COLLECTION_SHORT_RANGE_HOOD_ANGLES_DEGREES[2][1];
        double flywheelCommandIps = ShooterConstants.DATA_COLLECTION_SHORT_RANGE_COMMAND_SPEEDS_IPS[2][1];
        double minimumModeledFlywheelIps = ShooterConstants.COMMANDED_FLYWHEEL_SET_IPS[0];
        BallTrajectoryLookup.MovingShotSolution solution = new BallTrajectoryLookup.MovingShotSolution();
        double expectedExitVelocityIps =
                BallTrajectoryLookup.getEstimatedExitVelocityIpsForCommandedShot(
                        hoodAngleDegrees,
                        minimumModeledFlywheelIps);

        boolean solved = MovingShotMath.populateEmpiricalMovingShotSolution(
                hoodAngleDegrees,
                flywheelCommandIps,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                Inches.of(targetDistanceInches).in(Meters),
                0.0,
                TARGET_ELEVATION_INCHES,
                MAX_HEIGHT_INCHES,
                0.0,
                MIN_TURRET_ANGLE_DEGREES,
                MAX_TURRET_ANGLE_DEGREES,
                solution);

        assertTrue(solved, "Expected the empirical moving-shot estimate to remain valid below fitted flywheel minimum");
        assertEquals(flywheelCommandIps, solution.getFlywheelCommandIps(), 1e-9);
        assertEquals(expectedExitVelocityIps, solution.getLauncherRelativeExitVelocityIps(), 1e-9);
    }

    @Test
    void empiricalMovingShotClampsOnlyOutwardRadialRobotVelocity() {
        Translation2d effectiveVelocityIps = MovingShotMath.clampEmpiricalMovingShotRobotFieldVelocityIps(
                100.0,
                0.0,
                -30.0,
                12.0);

        assertEquals(0.0, effectiveVelocityIps.getX(), 1e-9);
        assertEquals(12.0, effectiveVelocityIps.getY(), 1e-9);
    }

    @Test
    void empiricalMovingShotRejectsNearHubBackwardLauncherSolutions() {
        double targetDistanceInches = 60.0;
        double preferredHoodAngleDegrees = ShortRangeHubFlywheelLookup.getIdealHoodAngleDegrees(targetDistanceInches);
        double fallbackFlywheelCommandIps =
                ShortRangeHubFlywheelLookup.getFallbackManifoldFlywheelCommandIps(targetDistanceInches);
        BallTrajectoryLookup.MovingShotSolution solution = new BallTrajectoryLookup.MovingShotSolution();

        boolean solved = MovingShotMath.solveIdealMovingShotWithUpperHoodFallback(
                ShooterConstants.COMMANDED_MINIMUM_ALLOWED_HOOD_ANGLE_DEGREES,
                ShooterConstants.COMMANDED_MAXIMUM_ALLOWED_HOOD_ANGLE_DEGREES,
                preferredHoodAngleDegrees,
                ShooterConstants.COMMANDED_MOVING_SHOT_HOOD_SEARCH_STEP_DEGREES,
                0.0,
                0.0,
                0.0,
                8.0,
                0.0,
                Inches.of(targetDistanceInches).in(Meters),
                0.0,
                TARGET_ELEVATION_INCHES,
                MAX_HEIGHT_INCHES,
                0.0,
                MIN_TURRET_ANGLE_DEGREES,
                MAX_TURRET_ANGLE_DEGREES,
                fallbackFlywheelCommandIps,
                fallbackFlywheelCommandIps,
                solution);

        assertTrue(
                !solved,
                "Empirical moving-shot solve should reject near-hub inward-speed cases that only work by launching backward");
    }

    @Test
    void empiricalMovingShotShortensAcrossMultipleMeasuredRowsAsTowardHubSpeedIncreases() {
        int[] distanceRowIndexes = {5, 6, 7};
        int shotColumnIndex = 2;

        for (int distanceRowIndex : distanceRowIndexes) {
            double targetDistanceInches =
                    ShooterConstants.DATA_COLLECTION_SHORT_RANGE_DISTANCES_INCHES[distanceRowIndex];
            double tableHoodAngleDegrees =
                    ShooterConstants.DATA_COLLECTION_SHORT_RANGE_HOOD_ANGLES_DEGREES[distanceRowIndex][shotColumnIndex];
            double tableFlywheelCommandIps =
                    ShooterConstants.DATA_COLLECTION_SHORT_RANGE_COMMAND_SPEEDS_IPS[distanceRowIndex][shotColumnIndex];
            double expectedStationaryCommandIps =
                    ShortRangeHubFlywheelLookup.getFallbackManifoldFlywheelCommandIps(targetDistanceInches);
            BallTrajectoryLookup.MovingShotSolution stationarySolution = solveEmpiricalMovingShotForTowardHubSpeed(
                    targetDistanceInches,
                    tableHoodAngleDegrees,
                    tableFlywheelCommandIps,
                    0.0);
            BallTrajectoryLookup.MovingShotSolution halfMeterPerSecondSolution = solveEmpiricalMovingShotForTowardHubSpeed(
                    targetDistanceInches,
                    tableHoodAngleDegrees,
                    tableFlywheelCommandIps,
                    0.5);
            BallTrajectoryLookup.MovingShotSolution oneMeterPerSecondSolution = solveEmpiricalMovingShotForTowardHubSpeed(
                    targetDistanceInches,
                    tableHoodAngleDegrees,
                    tableFlywheelCommandIps,
                    1.0);

            assertTrue(
                    stationarySolution.isValid(),
                    () -> String.format("Expected a valid stationary empirical shot at %.1f in", targetDistanceInches));
            assertEquals(targetDistanceInches, stationarySolution.getTargetRadialDistanceInches(), 1e-6);
            assertTrue(
                    stationarySolution.getHoodAngleDegrees()
                            >= ShortRangeHubFlywheelLookup.getMinimumHoodAngleDegrees(targetDistanceInches) - 1e-9
                            && stationarySolution.getHoodAngleDegrees()
                                    <= ShortRangeHubFlywheelLookup.getMaximumHoodAngleDegrees(targetDistanceInches) + 1e-9,
                    String.format(
                            "Stationary empirical solve should stay inside the interpolation-table hood envelope at %.1f in",
                            targetDistanceInches));
            assertTrue(
                    Math.abs(stationarySolution.getFlywheelCommandIps() - expectedStationaryCommandIps) <= 20.0,
                    String.format(
                            "Stationary empirical solve should stay reasonably close to the manifold-center speed at %.1f in",
                            targetDistanceInches));
            assertTrue(
                    stationarySolution.getFlywheelCommandIps()
                            >= ShortRangeHubFlywheelLookup.getMinimumFlywheelCommandIps(targetDistanceInches) - 1e-9
                            && stationarySolution.getFlywheelCommandIps()
                                    <= ShortRangeHubFlywheelLookup.getMaximumFlywheelCommandIps(targetDistanceInches) + 1e-9,
                    String.format(
                            "Stationary empirical command should remain inside the interpolation-table manifold at %.1f in",
                            targetDistanceInches));

            assertTrue(
                    halfMeterPerSecondSolution.isValid(),
                    () -> String.format(
                            "Expected a valid empirical moving shot at %.1f in and 0.5 m/s toward the hub",
                            targetDistanceInches));
            assertSolutionShortens(
                    stationarySolution,
                    halfMeterPerSecondSolution,
                    String.format("0.5 m/s toward-hub shot should shorten relative to stationary at %.1f in", targetDistanceInches));

            assertTrue(
                    oneMeterPerSecondSolution.isValid(),
                    () -> String.format(
                            "Expected a valid empirical moving shot at %.1f in and 1.0 m/s toward the hub",
                            targetDistanceInches));
            assertSolutionShortens(
                    halfMeterPerSecondSolution,
                    oneMeterPerSecondSolution,
                    String.format("1.0 m/s toward-hub shot should shorten relative to 0.5 m/s at %.1f in", targetDistanceInches));
        }
    }

    @Test
    void empiricalMovingShotStaysOnLookupManifoldAtFormerlyBadStraightPoints() {
        assertFormerlyBadStraightPointSolvesOnLookupManifold(221.0, 0.5);
        assertFormerlyBadStraightPointSolvesOnLookupManifold(158.0, 1.0);
    }

    @Test
    void stationaryEmpiricalShotsLandNearTargetInPhysicsModel() {
        int[] distanceRowIndexes = {0, 1, 2, 3, 4, 5, 6, 7};

        for (int distanceRowIndex : distanceRowIndexes) {
            double targetDistanceInches =
                    ShooterConstants.DATA_COLLECTION_SHORT_RANGE_DISTANCES_INCHES[distanceRowIndex];
            double preferredHoodAngleDegrees = ShortRangeHubFlywheelLookup.getIdealHoodAngleDegrees(targetDistanceInches);
            double currentFlywheelSpeedIps =
                    ShortRangeHubFlywheelLookup.getFallbackManifoldFlywheelCommandIps(targetDistanceInches);
            BallTrajectoryLookup.MovingShotSolution solution = solveEmpiricalMovingShotForVelocity(
                    targetDistanceInches,
                    preferredHoodAngleDegrees,
                    currentFlywheelSpeedIps,
                    0.0,
                    0.0);

            assertTrue(
                    solution.isValid(),
                    () -> String.format(
                            "Expected a valid stationary empirical solution at %.1f in",
                            targetDistanceInches));
            assertEmpiricalShotLandsNearTargetInPhysicsModel(
                    targetDistanceInches,
                    solution,
                    String.format("stationary %.1f in", targetDistanceInches));
        }
    }

    @Test
    void movingEmpiricalShotsLandNearTargetInPhysicsModel() {
        int[] distanceRowIndexes = {4, 5, 6, 7};
        double[] towardHubSpeedsMetersPerSecond = {0.25, 0.5, 1.0};

        for (int distanceRowIndex : distanceRowIndexes) {
            double targetDistanceInches =
                    ShooterConstants.DATA_COLLECTION_SHORT_RANGE_DISTANCES_INCHES[distanceRowIndex];
            double preferredHoodAngleDegrees = ShortRangeHubFlywheelLookup.getIdealHoodAngleDegrees(targetDistanceInches);
            double currentFlywheelSpeedIps =
                    ShortRangeHubFlywheelLookup.getFallbackManifoldFlywheelCommandIps(targetDistanceInches);

            for (double towardHubSpeedMetersPerSecond : towardHubSpeedsMetersPerSecond) {
                BallTrajectoryLookup.MovingShotSolution solution = solveEmpiricalMovingShotForVelocity(
                        targetDistanceInches,
                        preferredHoodAngleDegrees,
                        currentFlywheelSpeedIps,
                        towardHubSpeedMetersPerSecond,
                        0.0);

                assertTrue(
                        solution.isValid(),
                        () -> String.format(
                                "Expected a valid moving empirical solution at %.1f in and %.1f m/s toward the hub",
                                targetDistanceInches,
                                towardHubSpeedMetersPerSecond));
                assertEmpiricalShotLandsNearTargetInPhysicsModel(
                        targetDistanceInches,
                        solution,
                        String.format(
                                "moving %.1f in at %.1f m/s toward hub",
                                targetDistanceInches,
                                towardHubSpeedMetersPerSecond));
            }
        }
    }

    @Test
    void angledEmpiricalShotsLandNearTargetInPhysicsModel() {
        int[] distanceRowIndexes = {4, 5, 6, 7};
        double[] towardHubSpeedsMetersPerSecond = {0.5, 1.0};
        double[] tangentialSigns = {-1.0, 1.0};

        for (int distanceRowIndex : distanceRowIndexes) {
            double targetDistanceInches =
                    ShooterConstants.DATA_COLLECTION_SHORT_RANGE_DISTANCES_INCHES[distanceRowIndex];
            double preferredHoodAngleDegrees = ShortRangeHubFlywheelLookup.getIdealHoodAngleDegrees(targetDistanceInches);
            double currentFlywheelSpeedIps =
                    ShortRangeHubFlywheelLookup.getFallbackManifoldFlywheelCommandIps(targetDistanceInches);

            for (double towardHubSpeedMetersPerSecond : towardHubSpeedsMetersPerSecond) {
                double tangentialMagnitudeMetersPerSecond =
                        towardHubSpeedMetersPerSecond * ANGLED_PHYSICS_TANGENTIAL_TO_RADIAL_RATIO;
                for (double tangentialSign : tangentialSigns) {
                    double tangentialSpeedMetersPerSecond = tangentialSign * tangentialMagnitudeMetersPerSecond;
                    BallTrajectoryLookup.MovingShotSolution solution = solveEmpiricalMovingShotForVelocity(
                            targetDistanceInches,
                            preferredHoodAngleDegrees,
                            currentFlywheelSpeedIps,
                            towardHubSpeedMetersPerSecond,
                            tangentialSpeedMetersPerSecond);

                    assertTrue(
                            solution.isValid(),
                            () -> String.format(
                                    "Expected a valid angled empirical solution at %.1f in and (%.1f, %.1f) m/s",
                                    targetDistanceInches,
                                    towardHubSpeedMetersPerSecond,
                                    tangentialSpeedMetersPerSecond));
                    assertEmpiricalShotLandsNearTargetInPhysicsModel(
                            targetDistanceInches,
                            solution,
                            String.format(
                                    "angled %.1f in at (%.1f, %.1f) m/s",
                                    targetDistanceInches,
                                    towardHubSpeedMetersPerSecond,
                                    tangentialSpeedMetersPerSecond));
                }
            }
        }
    }

    @Test
    void empiricalMovingShotRemainsLocallyContinuousForStraightApproachWhenFlywheelStateIsCarriedForward() {
        assertEmpiricalMovingShotLocalContinuity(new Translation2d(0.5, 0.0), "straight");
    }

    @Test
    void empiricalMovingShotRemainsLocallyContinuousForAngledApproachWhenFlywheelStateIsCarriedForward() {
        assertEmpiricalMovingShotLocalContinuity(
                new Translation2d(0.5, 0.5 * ANGLED_PHYSICS_TANGENTIAL_TO_RADIAL_RATIO),
                "angled");
    }

    @Test
    void stationaryEmpiricalShotsRemainValidAcrossDistanceInterpolationAndManifoldEdges() {
        BallTrajectoryLookup.MovingShotSolution solution = new BallTrajectoryLookup.MovingShotSolution();
        ShortRangeHubFlywheelLookup.EmpiricalShotCandidate candidate =
                new ShortRangeHubFlywheelLookup.EmpiricalShotCandidate();

        for (double targetDistanceInches : buildEmpiricalDistanceEdgeCases()) {
            assertTrue(
                    MovingShotMath.shouldUseEmpiricalHubMovingShotModel(
                            targetDistanceInches,
                            TARGET_ELEVATION_INCHES),
                    () -> String.format(
                            "Expected %.3f in to stay on the empirical hub path",
                            targetDistanceInches));

            double minimumHoodAngleDegrees =
                    ShortRangeHubFlywheelLookup.getMinimumHoodAngleDegrees(targetDistanceInches);
            double idealHoodAngleDegrees =
                    ShortRangeHubFlywheelLookup.getIdealHoodAngleDegrees(targetDistanceInches);
            double maximumHoodAngleDegrees =
                    ShortRangeHubFlywheelLookup.getMaximumHoodAngleDegrees(targetDistanceInches);
            double minimumFlywheelCommandIps =
                    ShortRangeHubFlywheelLookup.getMinimumFlywheelCommandIps(targetDistanceInches);
            double maximumFlywheelCommandIps =
                    ShortRangeHubFlywheelLookup.getMaximumFlywheelCommandIps(targetDistanceInches);
            double fallbackFlywheelCommandIps =
                    ShortRangeHubFlywheelLookup.getFallbackManifoldFlywheelCommandIps(targetDistanceInches);
            ShortRangeHubFlywheelLookup.EmpiricalShotCandidate representativeCandidate =
                    new ShortRangeHubFlywheelLookup.EmpiricalShotCandidate();
            boolean foundRepresentativeCandidate = ShortRangeHubFlywheelLookup.findPreferredLegalShot(
                    targetDistanceInches,
                    fallbackFlywheelCommandIps,
                    Double.isFinite(idealHoodAngleDegrees)
                            ? idealHoodAngleDegrees
                            : 0.5 * (minimumHoodAngleDegrees + maximumHoodAngleDegrees),
                    representativeCandidate);

            assertFinite(minimumHoodAngleDegrees, "minimum hood angle");
            assertFinite(maximumHoodAngleDegrees, "maximum hood angle");
            assertFinite(minimumFlywheelCommandIps, "minimum flywheel command");
            assertFinite(maximumFlywheelCommandIps, "maximum flywheel command");
            assertFinite(fallbackFlywheelCommandIps, "fallback flywheel command");
            assertTrue(
                    foundRepresentativeCandidate,
                    () -> String.format(
                            "Expected a representative empirical candidate at %.3f in",
                            targetDistanceInches));
            assertTrue(
                    Double.isFinite(representativeCandidate.getHoodAngleDegrees())
                            && representativeCandidate.getHoodAngleDegrees()
                                    >= ShooterConstants.COMMANDED_MINIMUM_ALLOWED_HOOD_ANGLE_DEGREES - 1e-9
                            && representativeCandidate.getHoodAngleDegrees()
                                    <= ShooterConstants.COMMANDED_MAXIMUM_ALLOWED_HOOD_ANGLE_DEGREES + 1e-9,
                    () -> String.format(
                            "Expected representative candidate hood at %.3f in to stay inside physical hood limits. hood %.6f min %.6f max %.6f",
                            targetDistanceInches,
                            representativeCandidate.getHoodAngleDegrees(),
                            ShooterConstants.COMMANDED_MINIMUM_ALLOWED_HOOD_ANGLE_DEGREES,
                            ShooterConstants.COMMANDED_MAXIMUM_ALLOWED_HOOD_ANGLE_DEGREES));
            assertTrue(
                    minimumFlywheelCommandIps <= maximumFlywheelCommandIps + 1e-9,
                    () -> String.format(
                            "Expected flywheel manifold ordering at %.3f in. min %.6f max %.6f",
                            targetDistanceInches,
                            minimumFlywheelCommandIps,
                            maximumFlywheelCommandIps));

            assertStationaryEmpiricalCandidateShotValid(
                    targetDistanceInches,
                    fallbackFlywheelCommandIps,
                    representativeCandidate.getHoodAngleDegrees(),
                    "representative manifold candidate",
                    candidate,
                    solution);
        }
    }

    @Test
    void stationaryEmpiricalShotsRemainValidJustInsideInterpolatedHoodBoundsAcrossDistanceEdges() {
        BallTrajectoryLookup.MovingShotSolution solution = new BallTrajectoryLookup.MovingShotSolution();

        for (double targetDistanceInches : buildEmpiricalDistanceEdgeCases()) {
            double minimumHoodAngleDegrees = ShortRangeHubFlywheelLookup.getMinimumHoodAngleDegrees(targetDistanceInches);
            double maximumHoodAngleDegrees = ShortRangeHubFlywheelLookup.getMaximumHoodAngleDegrees(targetDistanceInches);
            double validMinimumHoodAngleDegrees = Math.max(
                    ShooterConstants.COMMANDED_MINIMUM_ALLOWED_HOOD_ANGLE_DEGREES,
                    minimumHoodAngleDegrees);
            double validMaximumHoodAngleDegrees = Math.min(
                    ShooterConstants.COMMANDED_MAXIMUM_ALLOWED_HOOD_ANGLE_DEGREES,
                    maximumHoodAngleDegrees);

            assertFinite(validMinimumHoodAngleDegrees, "valid minimum hood angle");
            assertFinite(validMaximumHoodAngleDegrees, "valid maximum hood angle");
            assertTrue(
                    validMinimumHoodAngleDegrees <= validMaximumHoodAngleDegrees + 1e-9,
                    () -> String.format(
                            "Expected ordered physical hood bounds at %.3f in. min %.6f max %.6f",
                            targetDistanceInches,
                            validMinimumHoodAngleDegrees,
                            validMaximumHoodAngleDegrees));

            double hoodSpanDegrees = validMaximumHoodAngleDegrees - validMinimumHoodAngleDegrees;
            double interiorMarginDegrees = Math.min(
                    INTERIOR_HOOD_EDGE_MARGIN_DEGREES,
                    Math.max(0.0, 0.25 * hoodSpanDegrees));
            double justInsideMinimumHoodAngleDegrees = validMinimumHoodAngleDegrees + interiorMarginDegrees;
            double justInsideMaximumHoodAngleDegrees = validMaximumHoodAngleDegrees - interiorMarginDegrees;

            assertStationaryEmpiricalFixedHoodShotValid(
                    targetDistanceInches,
                    justInsideMinimumHoodAngleDegrees,
                    "just inside minimum manifold hood",
                    solution);
            assertStationaryEmpiricalFixedHoodShotValid(
                    targetDistanceInches,
                    justInsideMaximumHoodAngleDegrees,
                    "just inside maximum manifold hood",
                    solution);
        }
    }

    @Test
    void empiricalMovingShotLeadsOppositeAngledApproachAndShootsFartherThanPureRadialApproach() {
        int[] distanceRowIndexes = {5, 6};
        int shotColumnIndex = 2;
        double inwardApproachSpeedMetersPerSecond = 0.5;
        double tangentialSpeedMetersPerSecond = 0.5;

        for (int distanceRowIndex : distanceRowIndexes) {
            double targetDistanceInches =
                    ShooterConstants.DATA_COLLECTION_SHORT_RANGE_DISTANCES_INCHES[distanceRowIndex];
            double tableHoodAngleDegrees =
                    ShooterConstants.DATA_COLLECTION_SHORT_RANGE_HOOD_ANGLES_DEGREES[distanceRowIndex][shotColumnIndex];
            double tableFlywheelCommandIps =
                    ShooterConstants.DATA_COLLECTION_SHORT_RANGE_COMMAND_SPEEDS_IPS[distanceRowIndex][shotColumnIndex];
            BallTrajectoryLookup.MovingShotSolution inwardOnlySolution = solveEmpiricalMovingShotForVelocity(
                    targetDistanceInches,
                    tableHoodAngleDegrees,
                    tableFlywheelCommandIps,
                    inwardApproachSpeedMetersPerSecond,
                    0.0);
            BallTrajectoryLookup.MovingShotSolution leftLateralSolution = solveEmpiricalMovingShotForVelocity(
                    targetDistanceInches,
                    tableHoodAngleDegrees,
                    tableFlywheelCommandIps,
                    inwardApproachSpeedMetersPerSecond,
                    tangentialSpeedMetersPerSecond);
            BallTrajectoryLookup.MovingShotSolution rightLateralSolution = solveEmpiricalMovingShotForVelocity(
                    targetDistanceInches,
                    tableHoodAngleDegrees,
                    tableFlywheelCommandIps,
                    inwardApproachSpeedMetersPerSecond,
                    -tangentialSpeedMetersPerSecond);

            assertTrue(
                    inwardOnlySolution.isValid(),
                    () -> String.format(
                            "Expected a valid empirical shot at %.1f in while driving straight inward at %.1f m/s",
                            targetDistanceInches,
                            inwardApproachSpeedMetersPerSecond));
            assertTrue(
                    leftLateralSolution.isValid(),
                    () -> String.format(
                            "Expected a valid empirical shot at %.1f in while driving inward-left at %.1f/%.1f m/s",
                            targetDistanceInches,
                            inwardApproachSpeedMetersPerSecond,
                            tangentialSpeedMetersPerSecond));
            assertTrue(
                    rightLateralSolution.isValid(),
                    () -> String.format(
                            "Expected a valid empirical shot at %.1f in while driving inward-right at %.1f/%.1f m/s",
                            targetDistanceInches,
                            inwardApproachSpeedMetersPerSecond,
                            tangentialSpeedMetersPerSecond));

            assertTrue(
                    leftLateralSolution.getShotAzimuthDegrees() < inwardOnlySolution.getShotAzimuthDegrees() - 1e-9,
                    () -> String.format(
                            "Driving inward-left should shift shot azimuth to the right at %.1f in. Inward az %.3f, left az %.3f",
                            targetDistanceInches,
                            inwardOnlySolution.getShotAzimuthDegrees(),
                            leftLateralSolution.getShotAzimuthDegrees()));
            assertTrue(
                    rightLateralSolution.getShotAzimuthDegrees() > inwardOnlySolution.getShotAzimuthDegrees() + 1e-9,
                    () -> String.format(
                            "Driving inward-right should shift shot azimuth to the left at %.1f in. Inward az %.3f, right az %.3f",
                            targetDistanceInches,
                            inwardOnlySolution.getShotAzimuthDegrees(),
                            rightLateralSolution.getShotAzimuthDegrees()));
            double inwardOnlyLauncherHorizontalExitVelocityIps =
                    getLauncherRelativeHorizontalExitVelocityIps(inwardOnlySolution);
            double leftLauncherHorizontalExitVelocityIps =
                    getLauncherRelativeHorizontalExitVelocityIps(leftLateralSolution);
            double rightLauncherHorizontalExitVelocityIps =
                    getLauncherRelativeHorizontalExitVelocityIps(rightLateralSolution);
            assertTrue(
                    leftLauncherHorizontalExitVelocityIps
                            > inwardOnlyLauncherHorizontalExitVelocityIps + 1e-9,
                    () -> String.format(
                            "Driving inward-left should increase launcher-relative horizontal speed over pure inward approach at %.1f in. "
                                    + "Inward horizontal %.3f, left horizontal %.3f",
                            targetDistanceInches,
                            inwardOnlyLauncherHorizontalExitVelocityIps,
                            leftLauncherHorizontalExitVelocityIps));
            assertTrue(
                    rightLauncherHorizontalExitVelocityIps
                            > inwardOnlyLauncherHorizontalExitVelocityIps + 1e-9,
                    () -> String.format(
                            "Driving inward-right should increase launcher-relative horizontal speed over pure inward approach at %.1f in. "
                                    + "Inward horizontal %.3f, right horizontal %.3f",
                            targetDistanceInches,
                            inwardOnlyLauncherHorizontalExitVelocityIps,
                            rightLauncherHorizontalExitVelocityIps));
        }
    }

    @Test
    void empiricalLookupDistanceBiasesDistanceShorterByRadialClosure() {
        Translation2d effectiveVelocityIps = new Translation2d(20.0, 0.0);

        assertEquals(
                120.0,
                MovingShotMath.getEmpiricalMovingShotLookupDistanceInches(
                        120.0,
                        100.0,
                        0.0,
                        effectiveVelocityIps),
                1e-9);
    }

    @Test
    void empiricalLookupDistanceNeverGoesNegativeAndStaysInsideEmpiricalMinimum() {
        Translation2d effectiveVelocityIps = new Translation2d(0.0, 0.0);

        assertEquals(
                ShooterConstants.DATA_COLLECTION_SHORT_RANGE_MIN_DISTANCE_INCHES,
                MovingShotMath.getEmpiricalMovingShotLookupDistanceInches(
                        10.0,
                        100.0,
                        0.0,
                        effectiveVelocityIps),
                1e-9);
    }

    @Test
    void headingTowardTargetPointsDirectlyAtHubAndFallsBackWhenCoincident() {
        Rotation2d targetHeading = MovingShotMath.getHeadingTowardTarget(
                3.0,
                4.0,
                Rotation2d.kZero);
        Rotation2d fallbackHeading = Rotation2d.fromDegrees(37.0);

        assertEquals(Math.toDegrees(Math.atan2(4.0, 3.0)), targetHeading.getDegrees(), 1e-9);
        assertEquals(
                fallbackHeading.getDegrees(),
                MovingShotMath.getHeadingTowardTarget(0.0, 0.0, fallbackHeading).getDegrees(),
                1e-9);
    }

    @Test
    void preferredTravelHeadingReflectsBackingAwayFromTarget() {
        Rotation2d reflectedHeading = MovingShotMath.getPreferredHeadingForTravelDirection(
                new Translation2d(-1.0, 0.0),
                new Translation2d(0.0, 0.0),
                new Translation2d(10.0, 0.0),
                Rotation2d.fromDegrees(37.0));
        Rotation2d forwardHeading = MovingShotMath.getPreferredHeadingForTravelDirection(
                new Translation2d(1.0, 0.0),
                new Translation2d(0.0, 0.0),
                new Translation2d(10.0, 0.0),
                Rotation2d.fromDegrees(37.0));

        assertEquals(0.0, reflectedHeading.getDegrees(), 1e-9);
        assertEquals(0.0, forwardHeading.getDegrees(), 1e-9);
    }

    @Test
    void preferredTravelHeadingStaysConstantWhenOnlyDirectionReverses() {
        Translation2d robotPosition = new Translation2d(1.0, -2.0);
        Translation2d target = new Translation2d(11.0, -2.0);
        Translation2d forwardTravelDirection = new Translation2d(2.0, 1.0);
        Translation2d reversedTravelDirection = forwardTravelDirection.times(-1.0);
        Rotation2d fallbackHeading = Rotation2d.fromDegrees(91.0);

        Rotation2d forwardHeading = MovingShotMath.getPreferredHeadingForTravelDirection(
                forwardTravelDirection,
                robotPosition,
                target,
                fallbackHeading);
        Rotation2d reversedHeading = MovingShotMath.getPreferredHeadingForTravelDirection(
                reversedTravelDirection,
                robotPosition,
                target,
                fallbackHeading);

        assertEquals(
                forwardHeading.getDegrees(),
                reversedHeading.getDegrees(),
                1e-9,
                "Preferred heading should not change when the travel direction is exactly reversed");
    }

    @Test
    void commandTrackingErrorReportsOvershootAndUndershootIndependentOfDirection() {
        assertEquals(
                -1.0,
                MovingShotMath.computeCommandTrackingError(70.0, 72.0, 71.0),
                1e-9);
        assertEquals(
                -1.0,
                MovingShotMath.computeCommandTrackingError(72.0, 70.0, 71.0),
                1e-9);
        assertEquals(
                1.0,
                MovingShotMath.computeCommandTrackingError(70.0, 72.0, 73.0),
                1e-9);
        assertEquals(
                1.0,
                MovingShotMath.computeCommandTrackingError(72.0, 70.0, 69.0),
                1e-9);
    }

    @Test
    void flywheelPredictionFromPreviousSetpointUsesPreviousCommandAndFallsBackToCurrentSpeed() {
        double currentFlywheelSpeedIps = 100.0;
        double previousCommandedFlywheelSetpointIps = 400.0;
        double expectedPredictedSpeedIps = Math.min(
                previousCommandedFlywheelSetpointIps,
                currentFlywheelSpeedIps
                        + ShooterConstants.COMMANDED_MOVING_SHOT_FLYWHEEL_SPIN_UP_RATE_IPS_PER_SECOND
                                * ShooterConstants.COMMANDED_MOVING_SHOT_FLYWHEEL_PREDICTION_SECONDS);

        assertEquals(
                expectedPredictedSpeedIps,
                MovingShotMath.predictFlywheelSpeedFromPreviousSetpointIps(
                        currentFlywheelSpeedIps,
                        previousCommandedFlywheelSetpointIps),
                1e-9);
        assertEquals(
                currentFlywheelSpeedIps,
                MovingShotMath.predictFlywheelSpeedFromPreviousSetpointIps(
                        currentFlywheelSpeedIps,
                        Double.NaN),
                1e-9);
    }

    @Test
    void randomEmpiricalShootOnTheFlyStatesReturnReasonableSolutions() {
        Random random = new Random(RANDOM_SHOOT_ON_THE_FLY_SEED);
        BallTrajectoryLookup.MovingShotSolution solution = new BallTrajectoryLookup.MovingShotSolution();
        ShortRangeHubFlywheelLookup.EmpiricalShotCandidate candidate =
                new ShortRangeHubFlywheelLookup.EmpiricalShotCandidate();
        int acceptedSamples = 0;

        for (int attempt = 0;
                attempt < RANDOM_SHOOT_ON_THE_FLY_MAX_ATTEMPTS
                        && acceptedSamples < RANDOM_SHOOT_ON_THE_FLY_SAMPLE_COUNT;
                attempt++) {
            double targetDistanceInches = interpolate(
                    MIN_RANDOM_VALID_DISTANCE_INCHES,
                    ShooterConstants.DATA_COLLECTION_SHORT_RANGE_EMPIRICAL_MAX_DISTANCE_INCHES,
                    random.nextDouble());
            double robotPositionAngleRadians = interpolate(-Math.PI, Math.PI, random.nextDouble());
            double robotXMeters = Inches.of(targetDistanceInches).in(Meters) * Math.cos(robotPositionAngleRadians);
            double robotYMeters = Inches.of(targetDistanceInches).in(Meters) * Math.sin(robotPositionAngleRadians);
            double targetDxMeters = -robotXMeters;
            double targetDyMeters = -robotYMeters;
            double hubBearingRadians = Math.atan2(targetDyMeters, targetDxMeters);
            double robotSpeedMetersPerSecond = MAX_ROBOT_SPEED_METERS_PER_SECOND * Math.sqrt(random.nextDouble());
            double robotVelocityAngleRadians = interpolate(-Math.PI, Math.PI, random.nextDouble());
            double robotFieldVxMetersPerSecond = robotSpeedMetersPerSecond * Math.cos(robotVelocityAngleRadians);
            double robotFieldVyMetersPerSecond = robotSpeedMetersPerSecond * Math.sin(robotVelocityAngleRadians);
            Translation2d effectiveRobotFieldVelocityIps = MovingShotMath.clampEmpiricalMovingShotRobotFieldVelocityIps(
                    Inches.convertFrom(targetDxMeters, Meters),
                    Inches.convertFrom(targetDyMeters, Meters),
                    Inches.convertFrom(robotFieldVxMetersPerSecond, Meters),
                    Inches.convertFrom(robotFieldVyMetersPerSecond, Meters));
            boolean foundCandidate = ShortRangeHubFlywheelLookup.findPreferredLegalShot(
                    targetDistanceInches,
                    0.0,
                    ShortRangeHubFlywheelLookup.getIdealHoodAngleDegrees(targetDistanceInches),
                    candidate);

            assertTrue(
                    foundCandidate,
                    () -> String.format(
                            "Expected a legal empirical shot candidate at distance %.3f in",
                            targetDistanceInches));
            assertTrue(
                    candidate.getFlywheelCommandIps()
                            >= ShortRangeHubFlywheelLookup.getMinimumFlywheelCommandIps(targetDistanceInches) - 1e-9,
                    "Selected flywheel command should stay inside the legal manifold minimum");
            assertTrue(
                    candidate.getFlywheelCommandIps()
                            <= ShortRangeHubFlywheelLookup.getMaximumFlywheelCommandIps(targetDistanceInches) + 1e-9,
                    "Selected flywheel command should stay inside the legal manifold maximum");
            if (!isExpectedToStayWithinShotBearingLimit(
                    candidate,
                    targetDxMeters,
                    targetDyMeters,
                    effectiveRobotFieldVelocityIps,
                    MAX_REASONABLE_SHOT_AZIMUTH_ERROR_DEGREES)) {
                continue;
            }

            boolean solved = MovingShotMath.populateEmpiricalMovingShotSolution(
                    candidate.getHoodAngleDegrees(),
                    candidate.getFlywheelCommandIps(),
                    robotXMeters,
                    robotYMeters,
                    hubBearingRadians,
                    Meters.convertFrom(effectiveRobotFieldVelocityIps.getX(), Inches),
                    Meters.convertFrom(effectiveRobotFieldVelocityIps.getY(), Inches),
                    0.0,
                    0.0,
                    TARGET_ELEVATION_INCHES,
                    MAX_HEIGHT_INCHES,
                    hubBearingRadians,
                    MIN_TURRET_ANGLE_DEGREES,
                    MAX_TURRET_ANGLE_DEGREES,
                    solution);

            assertTrue(
                    solved,
                    () -> String.format(
                            "Expected a reasonable shoot-on-the-fly solution at distance %.3f in and velocity (%.3f, %.3f) m/s",
                            targetDistanceInches,
                            robotFieldVxMetersPerSecond,
                            robotFieldVyMetersPerSecond));
            assertTrue(solution.isValid(), "Solver should leave a valid moving-shot solution");
            assertFinite(solution.getHoodAngleDegrees(), "hood angle");
            assertFinite(solution.getTargetRadialDistanceInches(), "target distance");
            assertFinite(solution.getTargetElevationInches(), "target elevation");
            assertFinite(solution.getMaximumBallZElevationInches(), "maximum ball height");
            assertFinite(solution.getFieldRelativeExitVelocityIps(), "field-relative exit velocity");
            assertFinite(solution.getLauncherRelativeExitVelocityIps(), "launcher-relative exit velocity");
            assertFinite(solution.getFlywheelCommandIps(), "flywheel command");
            assertFinite(solution.getShotAzimuthDegrees(), "shot azimuth");
            assertFinite(solution.getTurretDeltaDegrees(), "turret delta");
            assertFinite(solution.getRobotHeadingDegrees(), "robot heading");
            assertTrue(
                    solution.getFlywheelCommandIps() >= GLOBAL_MIN_EMPIRICAL_FLYWHEEL_IPS - 1e-9,
                    "Flywheel command should stay above the empirical minimum");
            assertTrue(
                    solution.getFlywheelCommandIps() <= GLOBAL_MAX_EMPIRICAL_FLYWHEEL_IPS + 1e-9,
                    "Flywheel command should stay below the empirical maximum");
            assertTrue(
                    solution.getLauncherRelativeExitVelocityIps() > 0.0,
                    "Launcher-relative exit velocity should remain positive");
            assertTrue(
                    solution.getFieldRelativeExitVelocityIps() > 0.0,
                    "Field-relative exit velocity should remain positive");
            assertTrue(
                    solution.getTurretDeltaDegrees() >= MIN_TURRET_ANGLE_DEGREES - 1e-9
                            && solution.getTurretDeltaDegrees() <= MAX_TURRET_ANGLE_DEGREES + 1e-9,
                    "Turret delta should stay within limits");
            assertEquals(targetDistanceInches, solution.getTargetRadialDistanceInches(), 1e-6);

            double shotBearingErrorDegrees = Math.abs(MathUtil.inputModulus(
                    solution.getShotAzimuthDegrees() - Math.toDegrees(hubBearingRadians),
                    -180.0,
                    180.0));
            assertTrue(
                    shotBearingErrorDegrees <= 60.0,
                    () -> String.format(
                            "Shot azimuth %.3f deg should stay within 60 deg of hub bearing %.3f deg",
                            solution.getShotAzimuthDegrees(),
                            Math.toDegrees(hubBearingRadians)));

            double azimuthClosureErrorDegrees = Math.abs(MathUtil.inputModulus(
                    solution.getShotAzimuthDegrees()
                            - (solution.getRobotHeadingDegrees() + solution.getTurretDeltaDegrees()),
                    -180.0,
                    180.0));
            assertTrue(
                    azimuthClosureErrorDegrees <= 1e-6,
                    "Robot heading plus turret delta should reconstruct the shot azimuth");
            acceptedSamples++;
        }

        assertEquals(
                RANDOM_SHOOT_ON_THE_FLY_SAMPLE_COUNT,
                acceptedSamples,
                "Expected enough random hub states with a plausible <=60 degree shot lead to validate");
    }

    private static double computeGlobalEmpiricalFlywheelBound(boolean minimumBound) {
        double bound = minimumBound ? Double.POSITIVE_INFINITY : Double.NEGATIVE_INFINITY;
        for (double targetDistanceInches : ShooterConstants.DATA_COLLECTION_SHORT_RANGE_DISTANCES_INCHES) {
            double candidateBound = minimumBound
                    ? ShortRangeHubFlywheelLookup.getMinimumFlywheelCommandIps(targetDistanceInches)
                    : ShortRangeHubFlywheelLookup.getMaximumFlywheelCommandIps(targetDistanceInches);
            bound = minimumBound ? Math.min(bound, candidateBound) : Math.max(bound, candidateBound);
        }
        double hardLimitBound = minimumBound
                ? ShortRangeHubFlywheelLookup.getMinimumFlywheelCommandIps(
                        ShooterConstants.DATA_COLLECTION_SHORT_RANGE_EMPIRICAL_MAX_DISTANCE_INCHES)
                : ShortRangeHubFlywheelLookup.getMaximumFlywheelCommandIps(
                        ShooterConstants.DATA_COLLECTION_SHORT_RANGE_EMPIRICAL_MAX_DISTANCE_INCHES);
        return minimumBound ? Math.min(bound, hardLimitBound) : Math.max(bound, hardLimitBound);
    }

    private static boolean isExpectedToStayWithinShotBearingLimit(
            ShortRangeHubFlywheelLookup.EmpiricalShotCandidate candidate,
            double targetDxMeters,
            double targetDyMeters,
            Translation2d effectiveRobotFieldVelocityIps,
            double maximumShotBearingErrorDegrees) {
        double launcherRelativeExitVelocityIps = BallTrajectoryLookup.getEstimatedExitVelocityIpsForCommandedShot(
                candidate.getHoodAngleDegrees(),
                candidate.getFlywheelCommandIps());
        double trueHoodAngleDegrees = ShooterConstants.getTrueAngleDegreesForCommandedAngle(
                candidate.getHoodAngleDegrees());
        if (!Double.isFinite(launcherRelativeExitVelocityIps) || !Double.isFinite(trueHoodAngleDegrees)) {
            return false;
        }

        double launcherRelativeHorizontalExitVelocityIps =
                launcherRelativeExitVelocityIps * Math.cos(Math.toRadians(trueHoodAngleDegrees));
        double targetDistanceInches = Math.hypot(
                Inches.convertFrom(targetDxMeters, Meters),
                Inches.convertFrom(targetDyMeters, Meters));
        if (!(targetDistanceInches > 0.0) || !(launcherRelativeHorizontalExitVelocityIps > 0.0)) {
            return false;
        }

        double targetUnitX = Inches.convertFrom(targetDxMeters, Meters) / targetDistanceInches;
        double targetUnitY = Inches.convertFrom(targetDyMeters, Meters) / targetDistanceInches;
        double tangentialVelocityIps = Math.abs(
                -targetUnitY * effectiveRobotFieldVelocityIps.getX()
                        + targetUnitX * effectiveRobotFieldVelocityIps.getY());
        double maximumTangentialVelocityIps =
                launcherRelativeHorizontalExitVelocityIps * Math.sin(Math.toRadians(maximumShotBearingErrorDegrees));
        return tangentialVelocityIps <= maximumTangentialVelocityIps + 1e-9;
    }

    private static double[] buildEmpiricalDistanceEdgeCases() {
        ArrayList<Double> distancesInches = new ArrayList<>();
        double[] measuredDistancesInches = ShooterConstants.DATA_COLLECTION_SHORT_RANGE_DISTANCES_INCHES;
        double edgeOffsetInches = 0.01;

        distancesInches.add(ShooterConstants.DATA_COLLECTION_SHORT_RANGE_MIN_DISTANCE_INCHES);
        for (int i = 0; i < measuredDistancesInches.length; i++) {
            double distanceInches = measuredDistancesInches[i];
            distancesInches.add(distanceInches);
            if (i > 0) {
                distancesInches.add(distanceInches - edgeOffsetInches);
            }
            if (i < measuredDistancesInches.length - 1) {
                distancesInches.add(distanceInches + edgeOffsetInches);
                distancesInches.add(0.5 * (distanceInches + measuredDistancesInches[i + 1]));
            }
        }
        distancesInches.add(ShooterConstants.DATA_COLLECTION_SHORT_RANGE_EMPIRICAL_MAX_DISTANCE_INCHES);

        distancesInches.sort(Double::compare);
        ArrayList<Double> uniqueDistancesInches = new ArrayList<>();
        for (double distanceInches : distancesInches) {
            if (uniqueDistancesInches.isEmpty()
                    || Math.abs(distanceInches - uniqueDistancesInches.get(uniqueDistancesInches.size() - 1)) > 1e-9) {
                uniqueDistancesInches.add(distanceInches);
            }
        }

        double[] result = new double[uniqueDistancesInches.size()];
        for (int i = 0; i < uniqueDistancesInches.size(); i++) {
            result[i] = uniqueDistancesInches.get(i);
        }
        return result;
    }

    private static BallTrajectoryLookup.MovingShotSolution solveEmpiricalMovingShotForTowardHubSpeed(
            double targetDistanceInches,
            double preferredHoodAngleDegrees,
            double currentFlywheelSpeedIps,
            double towardHubSpeedMetersPerSecond) {
        return solveEmpiricalMovingShotForVelocity(
                targetDistanceInches,
                preferredHoodAngleDegrees,
                currentFlywheelSpeedIps,
                towardHubSpeedMetersPerSecond,
                0.0);
    }

    private static BallTrajectoryLookup.MovingShotSolution solveEmpiricalMovingShotForVelocity(
            double targetDistanceInches,
            double preferredHoodAngleDegrees,
            double currentFlywheelSpeedIps,
            double robotFieldVxMetersPerSecond,
            double robotFieldVyMetersPerSecond) {
        return solveEmpiricalMovingShotForVelocity(
                targetDistanceInches,
                preferredHoodAngleDegrees,
                currentFlywheelSpeedIps,
                currentFlywheelSpeedIps,
                robotFieldVxMetersPerSecond,
                robotFieldVyMetersPerSecond);
    }

    private static BallTrajectoryLookup.MovingShotSolution solveEmpiricalMovingShotForVelocity(
            double targetDistanceInches,
            double preferredHoodAngleDegrees,
            double currentFlywheelSpeedIps,
            double previousCommandedFlywheelSetpointIps,
            double robotFieldVxMetersPerSecond,
            double robotFieldVyMetersPerSecond) {
        BallTrajectoryLookup.MovingShotSolution solution = new BallTrajectoryLookup.MovingShotSolution();
        boolean solved = MovingShotMath.solveIdealMovingShotWithUpperHoodFallback(
                ShooterConstants.COMMANDED_MINIMUM_ALLOWED_HOOD_ANGLE_DEGREES,
                ShooterConstants.COMMANDED_MAXIMUM_ALLOWED_HOOD_ANGLE_DEGREES,
                preferredHoodAngleDegrees,
                ShooterConstants.COMMANDED_MOVING_SHOT_HOOD_SEARCH_STEP_DEGREES,
                0.0,
                0.0,
                0.0,
                robotFieldVxMetersPerSecond,
                robotFieldVyMetersPerSecond,
                Inches.of(targetDistanceInches).in(Meters),
                0.0,
                TARGET_ELEVATION_INCHES,
                MAX_HEIGHT_INCHES,
                0.0,
                MIN_TURRET_ANGLE_DEGREES,
                MAX_TURRET_ANGLE_DEGREES,
                currentFlywheelSpeedIps,
                previousCommandedFlywheelSetpointIps,
                solution);
        if (!solved) {
            solution.invalidate();
        }
        return solution;
    }

    private static void assertFormerlyBadStraightPointSolvesOnLookupManifold(
            double targetDistanceInches,
            double towardHubSpeedMetersPerSecond) {
        BallTrajectoryLookup.MovingShotSolution solution = new BallTrajectoryLookup.MovingShotSolution();
        MovingShotMath.EmpiricalMovingShotDebugInfo debugInfo =
                new MovingShotMath.EmpiricalMovingShotDebugInfo();
        double preferredHoodAngleDegrees = ShortRangeHubFlywheelLookup.getIdealHoodAngleDegrees(targetDistanceInches);
        double currentFlywheelSpeedIps =
                ShortRangeHubFlywheelLookup.getFallbackManifoldFlywheelCommandIps(targetDistanceInches);

        boolean solved = MovingShotMath.solveIdealMovingShotWithUpperHoodFallback(
                ShooterConstants.COMMANDED_MINIMUM_ALLOWED_HOOD_ANGLE_DEGREES,
                ShooterConstants.COMMANDED_MAXIMUM_ALLOWED_HOOD_ANGLE_DEGREES,
                preferredHoodAngleDegrees,
                ShooterConstants.COMMANDED_MOVING_SHOT_HOOD_SEARCH_STEP_DEGREES,
                0.0,
                0.0,
                0.0,
                towardHubSpeedMetersPerSecond,
                0.0,
                Inches.of(targetDistanceInches).in(Meters),
                0.0,
                TARGET_ELEVATION_INCHES,
                MAX_HEIGHT_INCHES,
                0.0,
                MIN_TURRET_ANGLE_DEGREES,
                MAX_TURRET_ANGLE_DEGREES,
                currentFlywheelSpeedIps,
                currentFlywheelSpeedIps,
                solution,
                debugInfo);

        assertTrue(
                solved,
                () -> String.format(
                        "Expected a valid empirical straight-in moving shot at %.1f in and %.2f m/s",
                        targetDistanceInches,
                        towardHubSpeedMetersPerSecond));
        assertTrue(debugInfo.isValid(), "Expected empirical debug info to remain valid");

        double lookupTargetDistanceInches = debugInfo.getLookupTargetDistanceInches();
        double selectedHoodAngleDegrees = debugInfo.getSelectedHoodAngleDegrees();
        double lookupMinimumHoodAngleDegrees =
                ShortRangeHubFlywheelLookup.getMinimumHoodAngleDegrees(lookupTargetDistanceInches);
        double lookupMaximumHoodAngleDegrees =
                ShortRangeHubFlywheelLookup.getMaximumHoodAngleDegrees(lookupTargetDistanceInches);
        double lookupTimeOfFlightSeconds =
                BallTrajectoryLookup.getEstimatedTimeOfFlightSecondsForCommandedShot(
                        selectedHoodAngleDegrees,
                        debugInfo.getModeledFlywheelCommandIps(),
                        lookupTargetDistanceInches);

        assertTrue(
                selectedHoodAngleDegrees >= lookupMinimumHoodAngleDegrees - 1e-9
                        && selectedHoodAngleDegrees <= lookupMaximumHoodAngleDegrees + 1e-9,
                () -> String.format(
                        "Expected selected hood to stay inside the lookup manifold at %.1f in / %.2f m/s. lookup %.3f hood %.3f min %.3f max %.3f",
                        targetDistanceInches,
                        towardHubSpeedMetersPerSecond,
                        lookupTargetDistanceInches,
                        selectedHoodAngleDegrees,
                        lookupMinimumHoodAngleDegrees,
                        lookupMaximumHoodAngleDegrees));
        assertTrue(
                Double.isFinite(lookupTimeOfFlightSeconds) && lookupTimeOfFlightSeconds > 0.0,
                () -> String.format(
                        "Expected a finite descending TOF at %.1f in / %.2f m/s. lookup %.3f hood %.3f modeled %.3f",
                        targetDistanceInches,
                        towardHubSpeedMetersPerSecond,
                        lookupTargetDistanceInches,
                        selectedHoodAngleDegrees,
                        debugInfo.getModeledFlywheelCommandIps()));
    }

    private static void assertStationaryEmpiricalCandidateShotValid(
            double targetDistanceInches,
            double preferredFlywheelCommandIps,
            double preferredHoodAngleDegrees,
            String label,
            ShortRangeHubFlywheelLookup.EmpiricalShotCandidate candidate,
            BallTrajectoryLookup.MovingShotSolution solution) {
        candidate.invalidate();
        boolean found = ShortRangeHubFlywheelLookup.findPreferredLegalShot(
                targetDistanceInches,
                preferredFlywheelCommandIps,
                preferredHoodAngleDegrees,
                candidate);

        double minimumHoodAngleDegrees = ShortRangeHubFlywheelLookup.getMinimumHoodAngleDegrees(targetDistanceInches);
        double maximumHoodAngleDegrees = ShortRangeHubFlywheelLookup.getMaximumHoodAngleDegrees(targetDistanceInches);
        double minimumFlywheelCommandIps =
                ShortRangeHubFlywheelLookup.getMinimumFlywheelCommandIps(targetDistanceInches);
        double maximumFlywheelCommandIps =
                ShortRangeHubFlywheelLookup.getMaximumFlywheelCommandIps(targetDistanceInches);

        assertTrue(
                found,
                () -> String.format(
                        "Expected a legal empirical candidate at %.3f in for %s",
                        targetDistanceInches,
                        label));
        assertTrue(candidate.isValid(), "Expected a valid empirical shot candidate");
        assertTrue(
                Double.isFinite(candidate.getHoodAngleDegrees())
                        && candidate.getHoodAngleDegrees()
                                >= ShooterConstants.COMMANDED_MINIMUM_ALLOWED_HOOD_ANGLE_DEGREES - 1e-9
                        && candidate.getHoodAngleDegrees()
                                <= ShooterConstants.COMMANDED_MAXIMUM_ALLOWED_HOOD_ANGLE_DEGREES + 1e-9,
                () -> String.format(
                        "Expected %s candidate at %.3f in to stay inside the physical hood limits. hood %.6f min %.6f max %.6f",
                        label,
                        targetDistanceInches,
                        candidate.getHoodAngleDegrees(),
                        ShooterConstants.COMMANDED_MINIMUM_ALLOWED_HOOD_ANGLE_DEGREES,
                        ShooterConstants.COMMANDED_MAXIMUM_ALLOWED_HOOD_ANGLE_DEGREES));
        assertTrue(
                candidate.getFlywheelCommandIps() >= minimumFlywheelCommandIps - MANIFOLD_SPEED_TOLERANCE_IPS
                        && candidate.getFlywheelCommandIps() <= maximumFlywheelCommandIps + MANIFOLD_SPEED_TOLERANCE_IPS,
                () -> String.format(
                        "Expected %s candidate at %.3f in to stay inside the flywheel manifold. cmd %.6f min %.6f max %.6f",
                        label,
                        targetDistanceInches,
                        candidate.getFlywheelCommandIps(),
                        minimumFlywheelCommandIps,
                        maximumFlywheelCommandIps));

        boolean solved = MovingShotMath.populateEmpiricalMovingShotSolution(
                candidate.getHoodAngleDegrees(),
                candidate.getFlywheelCommandIps(),
                candidate.getModeledFlywheelCommandIps(),
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                Inches.of(targetDistanceInches).in(Meters),
                0.0,
                TARGET_ELEVATION_INCHES,
                MAX_HEIGHT_INCHES,
                0.0,
                MIN_TURRET_ANGLE_DEGREES,
                MAX_TURRET_ANGLE_DEGREES,
                solution);

        assertTrue(
                solved,
                () -> String.format(
                        "Expected a valid stationary empirical shot at %.3f in for %s",
                        targetDistanceInches,
                        label));
        assertTrue(solution.isValid(), "Expected a valid stationary empirical moving-shot solution");
        assertEquals(
                targetDistanceInches,
                solution.getTargetRadialDistanceInches(),
                1e-6,
                String.format(
                        "Expected stationary empirical target distance to remain %.3f in",
                        targetDistanceInches));
        assertTrue(
                Double.isFinite(solution.getHoodAngleDegrees())
                        && solution.getHoodAngleDegrees()
                                >= ShooterConstants.COMMANDED_MINIMUM_ALLOWED_HOOD_ANGLE_DEGREES - 1e-9
                        && solution.getHoodAngleDegrees()
                                <= ShooterConstants.COMMANDED_MAXIMUM_ALLOWED_HOOD_ANGLE_DEGREES + 1e-9,
                () -> String.format(
                        "Expected %s shot at %.3f in to stay inside the physical hood limits. hood %.6f min %.6f max %.6f",
                        label,
                        targetDistanceInches,
                        solution.getHoodAngleDegrees(),
                        ShooterConstants.COMMANDED_MINIMUM_ALLOWED_HOOD_ANGLE_DEGREES,
                        ShooterConstants.COMMANDED_MAXIMUM_ALLOWED_HOOD_ANGLE_DEGREES));
        assertTrue(
                solution.getFlywheelCommandIps() >= minimumFlywheelCommandIps - MANIFOLD_SPEED_TOLERANCE_IPS
                        && solution.getFlywheelCommandIps() <= maximumFlywheelCommandIps + MANIFOLD_SPEED_TOLERANCE_IPS,
                () -> String.format(
                        "Expected %s shot at %.3f in to stay inside the flywheel manifold. cmd %.6f min %.6f max %.6f",
                        label,
                        targetDistanceInches,
                        solution.getFlywheelCommandIps(),
                        minimumFlywheelCommandIps,
                        maximumFlywheelCommandIps));
        assertEquals(
                0.0,
                solution.getShotAzimuthDegrees(),
                1e-6,
                String.format(
                        "Expected stationary empirical shot at %.3f in to aim directly at the hub",
                        targetDistanceInches));
        assertEquals(
                0.0,
                solution.getTurretDeltaDegrees(),
                1e-6,
                String.format(
                        "Expected stationary empirical shot at %.3f in to require zero turret delta",
                        targetDistanceInches));
        assertEquals(
                0.0,
                solution.getRobotHeadingDegrees(),
                1e-6,
                String.format(
                        "Expected stationary empirical shot at %.3f in to preserve zero robot heading",
                        targetDistanceInches));
    }

    private static void assertStationaryEmpiricalFixedHoodShotValid(
            double targetDistanceInches,
            double preferredHoodAngleDegrees,
            String label,
            BallTrajectoryLookup.MovingShotSolution solution) {
        boolean solved = MovingShotMath.solveMovingShotAtClosestHoodAngle(
                preferredHoodAngleDegrees,
                ShooterConstants.COMMANDED_MINIMUM_ALLOWED_HOOD_ANGLE_DEGREES,
                ShooterConstants.COMMANDED_MAXIMUM_ALLOWED_HOOD_ANGLE_DEGREES,
                SEARCH_STEP_DEGREES,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                Inches.of(targetDistanceInches).in(Meters),
                0.0,
                TARGET_ELEVATION_INCHES,
                MAX_HEIGHT_INCHES,
                0.0,
                MIN_TURRET_ANGLE_DEGREES,
                MAX_TURRET_ANGLE_DEGREES,
                solution);

        assertTrue(
                solved,
                () -> String.format(
                        "Expected a valid stationary empirical shot at %.3f in for %s",
                        targetDistanceInches,
                        label));
        assertTrue(solution.isValid(), "Expected a valid stationary empirical moving-shot solution");
        assertEquals(
                targetDistanceInches,
                solution.getTargetRadialDistanceInches(),
                1e-6,
                String.format(
                        "Expected stationary empirical target distance to remain %.3f in",
                        targetDistanceInches));
        assertEquals(
                preferredHoodAngleDegrees,
                solution.getHoodAngleDegrees(),
                1e-6,
                String.format(
                        "Expected %s shot at %.3f in to keep the requested hood %.6f",
                        label,
                        targetDistanceInches,
                        preferredHoodAngleDegrees));
        assertTrue(
                Double.isFinite(solution.getFlywheelCommandIps())
                        && solution.getFlywheelCommandIps() >= ShooterConstants.COMMANDED_FLYWHEEL_SET_IPS[0] - MANIFOLD_SPEED_TOLERANCE_IPS
                        && solution.getFlywheelCommandIps()
                                <= ShooterConstants.COMMANDED_FLYWHEEL_SET_IPS[
                                        ShooterConstants.COMMANDED_FLYWHEEL_SET_IPS.length - 1]
                                                + MANIFOLD_SPEED_TOLERANCE_IPS,
                () -> String.format(
                        "Expected %s shot at %.3f in to stay inside the physical flywheel command limits. cmd %.6f min %.6f max %.6f",
                        label,
                        targetDistanceInches,
                        solution.getFlywheelCommandIps(),
                        ShooterConstants.COMMANDED_FLYWHEEL_SET_IPS[0],
                        ShooterConstants.COMMANDED_FLYWHEEL_SET_IPS[
                                ShooterConstants.COMMANDED_FLYWHEEL_SET_IPS.length - 1]));
        assertEquals(
                0.0,
                solution.getShotAzimuthDegrees(),
                1e-6,
                String.format(
                        "Expected stationary empirical shot at %.3f in to aim directly at the hub",
                        targetDistanceInches));
        assertEquals(
                0.0,
                solution.getTurretDeltaDegrees(),
                1e-6,
                String.format(
                        "Expected stationary empirical shot at %.3f in to require zero turret delta",
                        targetDistanceInches));
        assertEquals(
                0.0,
                solution.getRobotHeadingDegrees(),
                1e-6,
                String.format(
                        "Expected stationary empirical shot at %.3f in to preserve zero robot heading",
                        targetDistanceInches));
    }

    private static void assertSolutionShortens(
            BallTrajectoryLookup.MovingShotSolution longerShotSolution,
            BallTrajectoryLookup.MovingShotSolution shorterShotSolution,
            String message) {
        assertTrue(
                shorterShotSolution.getHoodAngleDegrees() > longerShotSolution.getHoodAngleDegrees() + 1e-9
                        || shorterShotSolution.getFlywheelCommandIps()
                                < longerShotSolution.getFlywheelCommandIps() - 1e-9,
                () -> String.format(
                        "%s. Longer hood %.3f / cmd %.3f, shorter hood %.3f / cmd %.3f",
                        message,
                        longerShotSolution.getHoodAngleDegrees(),
                        longerShotSolution.getFlywheelCommandIps(),
                        shorterShotSolution.getHoodAngleDegrees(),
                        shorterShotSolution.getFlywheelCommandIps()));
    }

    private static void assertEmpiricalMovingShotLocalContinuity(
            Translation2d requestedVelocityMetersPerSecond,
            String label) {
        double travelSpeedMetersPerSecond = requestedVelocityMetersPerSecond.getNorm();
        Translation2d travelUnitVector = requestedVelocityMetersPerSecond.div(travelSpeedMetersPerSecond);
        double currentFlywheelSpeedIps =
                ShortRangeHubFlywheelLookup.getFallbackManifoldFlywheelCommandIps(
                        ShooterConstants.DATA_COLLECTION_SHORT_RANGE_EMPIRICAL_MAX_DISTANCE_INCHES);
        double previousCommandedFlywheelSetpointIps = currentFlywheelSpeedIps;
        double currentPreferredHoodAngleDegrees =
                ShortRangeHubFlywheelLookup.getIdealHoodAngleDegrees(
                        ShooterConstants.DATA_COLLECTION_SHORT_RANGE_EMPIRICAL_MAX_DISTANCE_INCHES);
        BallTrajectoryLookup.MovingShotSolution previousSolution = null;
        double previousMaximumTravelSpeedMetersPerSecond = Double.NaN;

        for (double targetDistanceInches = ShooterConstants.DATA_COLLECTION_SHORT_RANGE_EMPIRICAL_MAX_DISTANCE_INCHES;
                targetDistanceInches >= ShooterConstants.DATA_COLLECTION_SHORT_RANGE_MIN_DISTANCE_INCHES - 1e-9;
                targetDistanceInches -= LOCAL_CONTINUITY_DISTANCE_STEP_INCHES) {
            double maximumTravelSpeedMetersPerSecond = findMaximumStrictEmpiricalTravelSpeedMetersPerSecond(
                    targetDistanceInches,
                    travelUnitVector,
                    currentPreferredHoodAngleDegrees,
                    currentFlywheelSpeedIps,
                    previousCommandedFlywheelSetpointIps);
            double limitedTravelSpeedMetersPerSecond =
                    Math.min(travelSpeedMetersPerSecond, maximumTravelSpeedMetersPerSecond);
            Translation2d limitedVelocityMetersPerSecond =
                    travelUnitVector.times(limitedTravelSpeedMetersPerSecond);
            BallTrajectoryLookup.MovingShotSolution solution = solveEmpiricalMovingShotForVelocity(
                    targetDistanceInches,
                    currentPreferredHoodAngleDegrees,
                    currentFlywheelSpeedIps,
                    previousCommandedFlywheelSetpointIps,
                    limitedVelocityMetersPerSecond.getX(),
                    limitedVelocityMetersPerSecond.getY());
            String continuitySolveDescription = describeEmpiricalContinuitySolve(
                    targetDistanceInches,
                    limitedVelocityMetersPerSecond,
                    currentPreferredHoodAngleDegrees,
                    currentFlywheelSpeedIps,
                    previousCommandedFlywheelSetpointIps);

            assertTrue(
                    solution.isValid(),
                    String.format(
                            "Expected a valid %s continuity solution at %.1f in with current %.3f and previous %.3f IPS. %s",
                            label,
                            targetDistanceInches,
                            currentFlywheelSpeedIps,
                            previousCommandedFlywheelSetpointIps,
                            continuitySolveDescription));
            assertTrue(
                    Double.isFinite(maximumTravelSpeedMetersPerSecond),
                    String.format(
                            "Expected a finite %s max speed at %.1f in with current %.3f and previous %.3f IPS",
                            label,
                            targetDistanceInches,
                            currentFlywheelSpeedIps,
                            previousCommandedFlywheelSetpointIps));

            if (previousSolution != null) {
                double hoodDeltaDegrees =
                        Math.abs(solution.getHoodAngleDegrees() - previousSolution.getHoodAngleDegrees());
                double flywheelDeltaIps =
                        Math.abs(solution.getFlywheelCommandIps() - previousSolution.getFlywheelCommandIps());
                double azimuthDeltaDegrees = Math.abs(MathUtil.inputModulus(
                        solution.getShotAzimuthDegrees() - previousSolution.getShotAzimuthDegrees(),
                        -180.0,
                        180.0));
                double maximumTravelSpeedDeltaMetersPerSecond =
                        Math.abs(maximumTravelSpeedMetersPerSecond - previousMaximumTravelSpeedMetersPerSecond);

                assertTrue(
                        hoodDeltaDegrees <= LOCAL_CONTINUITY_MAX_HOOD_STEP_DEGREES,
                        String.format(
                                "Expected %s hood continuity across %.1f in -> %.1f in, but delta was %.3f deg (%.3f -> %.3f)",
                                label,
                                previousSolution.getTargetRadialDistanceInches(),
                                targetDistanceInches,
                                hoodDeltaDegrees,
                                previousSolution.getHoodAngleDegrees(),
                                solution.getHoodAngleDegrees()));
                assertTrue(
                        flywheelDeltaIps <= LOCAL_CONTINUITY_MAX_FLYWHEEL_STEP_IPS,
                        String.format(
                                "Expected %s flywheel continuity across %.1f in -> %.1f in, but delta was %.3f ips (%.3f -> %.3f)",
                                label,
                                previousSolution.getTargetRadialDistanceInches(),
                                targetDistanceInches,
                                flywheelDeltaIps,
                                previousSolution.getFlywheelCommandIps(),
                                solution.getFlywheelCommandIps()));
                assertTrue(
                        azimuthDeltaDegrees <= LOCAL_CONTINUITY_MAX_AZIMUTH_STEP_DEGREES,
                        String.format(
                                "Expected %s azimuth continuity across %.1f in -> %.1f in, but delta was %.3f deg (%.3f -> %.3f)",
                                label,
                                previousSolution.getTargetRadialDistanceInches(),
                                targetDistanceInches,
                                azimuthDeltaDegrees,
                                previousSolution.getShotAzimuthDegrees(),
                                solution.getShotAzimuthDegrees()));
                assertTrue(
                        maximumTravelSpeedDeltaMetersPerSecond <= LOCAL_CONTINUITY_MAX_SPEED_LIMIT_STEP_METERS_PER_SECOND,
                        String.format(
                                "Expected %s max-speed continuity across %.1f in -> %.1f in, but delta was %.3f m/s (%.3f -> %.3f)",
                                label,
                                previousSolution.getTargetRadialDistanceInches(),
                                targetDistanceInches,
                                maximumTravelSpeedDeltaMetersPerSecond,
                                previousMaximumTravelSpeedMetersPerSecond,
                                maximumTravelSpeedMetersPerSecond));
            }

            BallTrajectoryLookup.MovingShotSolution solutionCopy = new BallTrajectoryLookup.MovingShotSolution();
            solutionCopy.copyFrom(solution);
            previousSolution = solutionCopy;
            previousMaximumTravelSpeedMetersPerSecond = maximumTravelSpeedMetersPerSecond;
            currentPreferredHoodAngleDegrees = solution.getHoodAngleDegrees();
            previousCommandedFlywheelSetpointIps = solution.getFlywheelCommandIps();
            currentFlywheelSpeedIps = MovingShotMath.predictFlywheelSpeedFromPreviousSetpointIps(
                    currentFlywheelSpeedIps,
                    previousCommandedFlywheelSetpointIps);
        }
    }

    private static double findMaximumStrictEmpiricalTravelSpeedMetersPerSecond(
            double targetDistanceInches,
            Translation2d travelUnitVector,
            double preferredHoodAngleDegrees,
            double currentFlywheelSpeedIps,
            double previousCommandedFlywheelSetpointIps) {
        double lowMetersPerSecond = MINIMUM_ALLOWED_TRAVEL_SPEED_METERS_PER_SECOND;
        double highMetersPerSecond = Math.max(lowMetersPerSecond, 10.0);
        if (!hasStrictViableEmpiricalHubShot(
                targetDistanceInches,
                travelUnitVector.times(lowMetersPerSecond),
                preferredHoodAngleDegrees,
                currentFlywheelSpeedIps,
                previousCommandedFlywheelSetpointIps)) {
            return lowMetersPerSecond;
        }
        while (hasStrictViableEmpiricalHubShot(
                targetDistanceInches,
                travelUnitVector.times(highMetersPerSecond),
                preferredHoodAngleDegrees,
                currentFlywheelSpeedIps,
                previousCommandedFlywheelSetpointIps)) {
            highMetersPerSecond *= 2.0;
            if (highMetersPerSecond > 40.0) {
                return highMetersPerSecond;
            }
        }

        double bestViableMetersPerSecond = lowMetersPerSecond;
        for (int iteration = 0; iteration < 25; iteration++) {
            double midMetersPerSecond = 0.5 * (lowMetersPerSecond + highMetersPerSecond);
            if (hasStrictViableEmpiricalHubShot(
                    targetDistanceInches,
                    travelUnitVector.times(midMetersPerSecond),
                    preferredHoodAngleDegrees,
                    currentFlywheelSpeedIps,
                    previousCommandedFlywheelSetpointIps)) {
                bestViableMetersPerSecond = midMetersPerSecond;
                lowMetersPerSecond = midMetersPerSecond;
            } else {
                highMetersPerSecond = midMetersPerSecond;
            }
        }
        return Math.max(
                MINIMUM_ALLOWED_TRAVEL_SPEED_METERS_PER_SECOND,
                VIABLE_SPEED_MARGIN * bestViableMetersPerSecond);
    }

    private static boolean hasStrictViableEmpiricalHubShot(
            double targetDistanceInches,
            Translation2d robotFieldVelocityMetersPerSecond,
            double preferredHoodAngleDegrees,
            double currentFlywheelSpeedIps,
            double previousCommandedFlywheelSetpointIps) {
        BallTrajectoryLookup.MovingShotSolution solution = new BallTrajectoryLookup.MovingShotSolution();
        return MovingShotMath.solveIdealMovingShotWithUpperHoodFallback(
                ShooterConstants.COMMANDED_MINIMUM_ALLOWED_HOOD_ANGLE_DEGREES,
                ShooterConstants.COMMANDED_MAXIMUM_ALLOWED_HOOD_ANGLE_DEGREES,
                preferredHoodAngleDegrees,
                ShooterConstants.COMMANDED_MOVING_SHOT_HOOD_SEARCH_STEP_DEGREES,
                0.0,
                0.0,
                0.0,
                robotFieldVelocityMetersPerSecond.getX(),
                robotFieldVelocityMetersPerSecond.getY(),
                Inches.of(targetDistanceInches).in(Meters),
                0.0,
                TARGET_ELEVATION_INCHES,
                MAX_HEIGHT_INCHES,
                0.0,
                MIN_TURRET_ANGLE_DEGREES,
                MAX_TURRET_ANGLE_DEGREES,
                currentFlywheelSpeedIps,
                previousCommandedFlywheelSetpointIps,
                true,
                solution,
                null);
    }

    private static String describeEmpiricalContinuitySolve(
            double targetDistanceInches,
            Translation2d robotFieldVelocityMetersPerSecond,
            double preferredHoodAngleDegrees,
            double currentFlywheelSpeedIps,
            double previousCommandedFlywheelSetpointIps) {
        BallTrajectoryLookup.MovingShotSolution permissiveSolution = new BallTrajectoryLookup.MovingShotSolution();
        MovingShotMath.EmpiricalMovingShotDebugInfo permissiveDebugInfo =
                new MovingShotMath.EmpiricalMovingShotDebugInfo();
        boolean hasPermissiveSolution = MovingShotMath.solveIdealMovingShotWithUpperHoodFallback(
                ShooterConstants.COMMANDED_MINIMUM_ALLOWED_HOOD_ANGLE_DEGREES,
                ShooterConstants.COMMANDED_MAXIMUM_ALLOWED_HOOD_ANGLE_DEGREES,
                preferredHoodAngleDegrees,
                ShooterConstants.COMMANDED_MOVING_SHOT_HOOD_SEARCH_STEP_DEGREES,
                0.0,
                0.0,
                0.0,
                robotFieldVelocityMetersPerSecond.getX(),
                robotFieldVelocityMetersPerSecond.getY(),
                Inches.of(targetDistanceInches).in(Meters),
                0.0,
                TARGET_ELEVATION_INCHES,
                MAX_HEIGHT_INCHES,
                0.0,
                MIN_TURRET_ANGLE_DEGREES,
                MAX_TURRET_ANGLE_DEGREES,
                currentFlywheelSpeedIps,
                previousCommandedFlywheelSetpointIps,
                permissiveSolution,
                permissiveDebugInfo);

        BallTrajectoryLookup.MovingShotSolution strictSolution = new BallTrajectoryLookup.MovingShotSolution();
        MovingShotMath.EmpiricalMovingShotDebugInfo strictDebugInfo =
                new MovingShotMath.EmpiricalMovingShotDebugInfo();
        boolean hasStrictSolution = MovingShotMath.solveIdealMovingShotWithUpperHoodFallback(
                ShooterConstants.COMMANDED_MINIMUM_ALLOWED_HOOD_ANGLE_DEGREES,
                ShooterConstants.COMMANDED_MAXIMUM_ALLOWED_HOOD_ANGLE_DEGREES,
                preferredHoodAngleDegrees,
                ShooterConstants.COMMANDED_MOVING_SHOT_HOOD_SEARCH_STEP_DEGREES,
                0.0,
                0.0,
                0.0,
                robotFieldVelocityMetersPerSecond.getX(),
                robotFieldVelocityMetersPerSecond.getY(),
                Inches.of(targetDistanceInches).in(Meters),
                0.0,
                TARGET_ELEVATION_INCHES,
                MAX_HEIGHT_INCHES,
                0.0,
                MIN_TURRET_ANGLE_DEGREES,
                MAX_TURRET_ANGLE_DEGREES,
                currentFlywheelSpeedIps,
                previousCommandedFlywheelSetpointIps,
                true,
                strictSolution,
                strictDebugInfo);

        return String.format(
                "vel=(%.3f, %.3f) m/s prefHood=%.3f permissive=%s lookup=%.3f hood=%.3f modeled=%.3f cmd=%.3f strict=%s strictLookup=%.3f strictHood=%.3f strictModeled=%.3f strictCmd=%.3f",
                robotFieldVelocityMetersPerSecond.getX(),
                robotFieldVelocityMetersPerSecond.getY(),
                preferredHoodAngleDegrees,
                hasPermissiveSolution,
                permissiveDebugInfo.getLookupTargetDistanceInches(),
                permissiveDebugInfo.getSelectedHoodAngleDegrees(),
                permissiveDebugInfo.getModeledFlywheelCommandIps(),
                permissiveDebugInfo.getCommandedFlywheelCommandIps(),
                hasStrictSolution,
                strictDebugInfo.getLookupTargetDistanceInches(),
                strictDebugInfo.getSelectedHoodAngleDegrees(),
                strictDebugInfo.getModeledFlywheelCommandIps(),
                strictDebugInfo.getCommandedFlywheelCommandIps());
    }

    private static void assertEmpiricalShotLandsNearTargetInPhysicsModel(
            double targetDistanceInches,
            BallTrajectoryLookup.MovingShotSolution solution,
            String label) {
        double missDistanceInches = simulateDescendingPlanarMissDistanceAtTargetHeightInches(
                solution,
                targetDistanceInches,
                TARGET_ELEVATION_INCHES);

        assertTrue(
                Double.isFinite(missDistanceInches),
                () -> String.format(
                        "Expected a finite descending miss distance for %s, but got %.6f",
                        label,
                        missDistanceInches));
        assertTrue(
                missDistanceInches <= EMPIRICAL_PHYSICS_LANDING_TOLERANCE_INCHES,
                () -> String.format(
                        "Expected empirical shot for %s to land within %.1f in of the target in the physics model, but miss was %.3f in",
                        label,
                        EMPIRICAL_PHYSICS_LANDING_TOLERANCE_INCHES,
                        missDistanceInches));
    }

    private static double getLauncherRelativeHorizontalExitVelocityIps(
            BallTrajectoryLookup.MovingShotSolution solution) {
        double trueHoodAngleDegrees =
                ShooterConstants.getTrueAngleDegreesForCommandedAngle(solution.getHoodAngleDegrees());
        return solution.getLauncherRelativeExitVelocityIps()
                * Math.cos(Math.toRadians(trueHoodAngleDegrees));
    }

    private static double simulateDescendingPlanarMissDistanceAtTargetHeightInches(
            BallTrajectoryLookup.MovingShotSolution solution,
            double targetDistanceInches,
            double targetHeightInches) {
        double trueHoodAngleDegrees =
                ShooterConstants.getTrueAngleDegreesForCommandedAngle(solution.getHoodAngleDegrees());
        if (!Double.isFinite(trueHoodAngleDegrees)
                || !Double.isFinite(solution.getLauncherRelativeExitVelocityIps())
                || !Double.isFinite(solution.getFieldRelativeExitVelocityIps())
                || !Double.isFinite(solution.getShotAzimuthDegrees())
                || !Double.isFinite(targetDistanceInches)
                || !Double.isFinite(targetHeightInches)) {
            return Double.NaN;
        }

        double hoodAngleRadians = Math.toRadians(trueHoodAngleDegrees);
        double shotAzimuthRadians = Math.toRadians(solution.getShotAzimuthDegrees());
        double verticalExitVelocityIps =
                solution.getLauncherRelativeExitVelocityIps() * Math.sin(hoodAngleRadians);
        double horizontalExitVelocityIpsSquared =
                solution.getFieldRelativeExitVelocityIps() * solution.getFieldRelativeExitVelocityIps()
                        - verticalExitVelocityIps * verticalExitVelocityIps;
        double fieldHorizontalExitVelocityIps = Math.sqrt(Math.max(0.0, horizontalExitVelocityIpsSquared));
        double vxIps = fieldHorizontalExitVelocityIps * Math.cos(shotAzimuthRadians);
        double vyIps = fieldHorizontalExitVelocityIps * Math.sin(shotAzimuthRadians);
        double vzIps = verticalExitVelocityIps;
        double launchXInches = BallTrajectoryLookup.getLaunchXInches(trueHoodAngleDegrees);
        double xInches = launchXInches * Math.cos(shotAzimuthRadians);
        double yInches = launchXInches * Math.sin(shotAzimuthRadians);
        double zInches = BallTrajectoryLookup.getLaunchZInches(trueHoodAngleDegrees);
        double spinProxyIps = getSpinProxyIpsForCorrectedExitVelocity(
                trueHoodAngleDegrees,
                solution.getLauncherRelativeExitVelocityIps());
        boolean hasStartedDescending = vzIps <= 0.0;

        for (double timeSeconds = 0.0;
                timeSeconds < PHYSICS_MAX_SIMULATION_TIME_SECONDS;
                timeSeconds += PHYSICS_SIMULATION_DT_SECONDS) {
            double previousXInches = xInches;
            double previousYInches = yInches;
            double previousZInches = zInches;
            double previousVzIps = vzIps;
            double horizontalSpeedIps = Math.hypot(vxIps, vyIps);
            double speedIps = Math.hypot(horizontalSpeedIps, vzIps);
            double dragCoefficientPerInch =
                    ShooterConstants.FITTED_TRAJECTORY_DRAG_COEFFICIENT_BASE_PER_INCH
                            + ShooterConstants.FITTED_TRAJECTORY_DRAG_COEFFICIENT_LOG_SLOPE_PER_INCH
                                    * Math.log(Math.max(1.0, speedIps)
                                            / ShooterConstants.FITTED_TRAJECTORY_DRAG_LOG_REFERENCE_SPEED_IPS);
            dragCoefficientPerInch = Math.max(0.0, dragCoefficientPerInch);
            double dragGainPerSecond = dragCoefficientPerInch * speedIps;
            double magnusGainPerSecond =
                    ShooterConstants.FITTED_TRAJECTORY_MAGNUS_PER_SPIN_INCH * spinProxyIps * speedIps;
            double horizontalAccelerationIps2 =
                    -dragGainPerSecond * horizontalSpeedIps - magnusGainPerSecond * vzIps;
            double horizontalUnitX = horizontalSpeedIps > 1e-9 ? vxIps / horizontalSpeedIps : 0.0;
            double horizontalUnitY = horizontalSpeedIps > 1e-9 ? vyIps / horizontalSpeedIps : 0.0;
            double axIps2 = horizontalAccelerationIps2 * horizontalUnitX;
            double ayIps2 = horizontalAccelerationIps2 * horizontalUnitY;
            double azIps2 = -GRAVITY_IPS2 - dragGainPerSecond * vzIps + magnusGainPerSecond * horizontalSpeedIps;

            double nextVxIps = vxIps + axIps2 * PHYSICS_SIMULATION_DT_SECONDS;
            double nextVyIps = vyIps + ayIps2 * PHYSICS_SIMULATION_DT_SECONDS;
            double nextVzIps = vzIps + azIps2 * PHYSICS_SIMULATION_DT_SECONDS;
            double nextXInches = xInches + 0.5 * (vxIps + nextVxIps) * PHYSICS_SIMULATION_DT_SECONDS;
            double nextYInches = yInches + 0.5 * (vyIps + nextVyIps) * PHYSICS_SIMULATION_DT_SECONDS;
            double nextZInches = zInches + 0.5 * (vzIps + nextVzIps) * PHYSICS_SIMULATION_DT_SECONDS;

            if (!hasStartedDescending && nextVzIps <= 0.0) {
                hasStartedDescending = true;
            }

            if (hasStartedDescending
                    && previousZInches >= targetHeightInches
                    && nextZInches <= targetHeightInches
                    && (previousVzIps <= 0.0 || nextVzIps <= 0.0)) {
                double ratio = (targetHeightInches - previousZInches) / (nextZInches - previousZInches);
                double hitXInches = interpolate(previousXInches, nextXInches, ratio);
                double hitYInches = interpolate(previousYInches, nextYInches, ratio);
                return Math.hypot(hitXInches - targetDistanceInches, hitYInches);
            }

            xInches = nextXInches;
            yInches = nextYInches;
            zInches = nextZInches;
            vxIps = nextVxIps;
            vyIps = nextVyIps;
            vzIps = nextVzIps;

            if (zInches <= 0.0 && targetHeightInches > 0.0) {
                return Double.NaN;
            }
        }

        return Double.NaN;
    }

    private static double getSpinProxyIpsForCorrectedExitVelocity(
            double trueHoodAngleDegrees,
            double correctedExitVelocityIps) {
        double angleExitScale = interpolateDescending(
                trueHoodAngleDegrees,
                ShooterConstants.TRUE_HOOD_ANGLES_DEGREES,
                ShooterConstants.FITTED_COMMAND_ANGLE_EXIT_SCALES);
        if (!(angleExitScale > 0.0)) {
            return 0.0;
        }
        double baseExitVelocityIps = correctedExitVelocityIps / angleExitScale;
        double estimatedFlywheelCommandIps = FlywheelBallExitInterpolator.getSetIpsForBallExitIps(baseExitVelocityIps);
        if (!Double.isFinite(estimatedFlywheelCommandIps)) {
            return 0.0;
        }
        return Math.max(0.0, estimatedFlywheelCommandIps - ShooterConstants.MEASURED_BACKSPIN_CANCEL_LIMIT_COMMAND_IPS)
                * angleExitScale;
    }

    private static double interpolate(double value0, double value1, double ratio) {
        return value0 + (value1 - value0) * ratio;
    }

    private static double interpolateDescending(
            double value,
            double[] sampleXs,
            double[] sampleYs) {
        if (!Double.isFinite(value)
                || sampleXs == null
                || sampleYs == null
                || sampleXs.length != sampleYs.length
                || sampleXs.length == 0) {
            return Double.NaN;
        }
        if (value >= sampleXs[0]) {
            return sampleYs[0];
        }
        if (value <= sampleXs[sampleXs.length - 1]) {
            return sampleYs[sampleYs.length - 1];
        }
        for (int i = 1; i < sampleXs.length; i++) {
            double upperX = sampleXs[i - 1];
            double lowerX = sampleXs[i];
            if (value <= upperX + 1e-9 && value >= lowerX - 1e-9) {
                double ratio = (value - upperX) / (lowerX - upperX);
                return interpolate(sampleYs[i - 1], sampleYs[i], ratio);
            }
        }
        return Double.NaN;
    }

    private static void assertFinite(double value, String label) {
        assertTrue(Double.isFinite(value), () -> label + " should be finite");
    }
}
