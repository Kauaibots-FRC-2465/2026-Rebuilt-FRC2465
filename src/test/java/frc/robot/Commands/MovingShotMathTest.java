package frc.robot.Commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

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
    void empiricalLookupDistanceBiasesDistanceShorterByRadialClosure() {
        Translation2d effectiveVelocityIps = new Translation2d(20.0, 0.0);

        assertEquals(
                ShooterConstants.DATA_COLLECTION_SHORT_RANGE_MIN_DISTANCE_INCHES,
                MovingShotMath.getEmpiricalMovingShotLookupDistanceInches(
                        120.0,
                        100.0,
                        0.0,
                        effectiveVelocityIps),
                1e-9);
    }

    @Test
    void empiricalLookupDistanceNeverGoesNegativeAndStaysInsideEmpiricalMinimum() {
        Translation2d effectiveVelocityIps = new Translation2d(1000.0, 0.0);

        assertEquals(
                ShooterConstants.DATA_COLLECTION_SHORT_RANGE_MIN_DISTANCE_INCHES,
                MovingShotMath.getEmpiricalMovingShotLookupDistanceInches(
                        120.0,
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

    private static double interpolate(double value0, double value1, double ratio) {
        return value0 + (value1 - value0) * ratio;
    }

    private static void assertFinite(double value, String label) {
        assertTrue(Double.isFinite(value), () -> label + " should be finite");
    }
}
