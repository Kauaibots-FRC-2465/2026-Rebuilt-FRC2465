package frc.robot.Commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

class ScoreInHubSpeedEnvelopeReportTest {
    private static final class EnvelopePointResult {
        private final double maximumTravelSpeedMetersPerSecond;
        private final double selectedHoodAngleDegrees;
        private final double commandedFlywheelCommandIps;

        private EnvelopePointResult(
                double maximumTravelSpeedMetersPerSecond,
                double selectedHoodAngleDegrees,
                double commandedFlywheelCommandIps) {
            this.maximumTravelSpeedMetersPerSecond = maximumTravelSpeedMetersPerSecond;
            this.selectedHoodAngleDegrees = selectedHoodAngleDegrees;
            this.commandedFlywheelCommandIps = commandedFlywheelCommandIps;
        }
    }

    private static final double MINIMUM_ALLOWED_TRAVEL_SPEED_METERS_PER_SECOND = 0.5;
    private static final double VIABLE_SPEED_MARGIN = 0.90;
    private static final double ANGLED_TANGENTIAL_TO_RADIAL_RATIO = 0.2;
    private static final double MIN_TURRET_ANGLE_DEGREES = -180.0;
    private static final double MAX_TURRET_ANGLE_DEGREES = 180.0;
    private static final int REPORT_POINT_COUNT = 80;

    @Test
    void printStraightAndAngledMaximumTravelSpeedEnvelope() {
        printEnvelope("straight", new Translation2d(1.0, 0.0));
        printEnvelope(
                "angled",
                new Translation2d(1.0, ANGLED_TANGENTIAL_TO_RADIAL_RATIO)
                        .times(1.0 / Math.hypot(1.0, ANGLED_TANGENTIAL_TO_RADIAL_RATIO)));
    }

    @Test
    void printSuspiciousStraightPointDetails() {
        printSuspiciousStraightPoint(221.0);
        printSuspiciousStraightPoint(158.0);
    }

    @Test
    void straightEnvelopePointsRemainFiniteAndRespectMinimumFloor() {
        assertEnvelopePointsRemainFiniteAndRespectMinimumFloor(new Translation2d(1.0, 0.0));
    }

    @Test
    void angledEnvelopePointsRemainFiniteAndRespectMinimumFloor() {
        assertEnvelopePointsRemainFiniteAndRespectMinimumFloor(
                new Translation2d(1.0, ANGLED_TANGENTIAL_TO_RADIAL_RATIO)
                        .times(1.0 / Math.hypot(1.0, ANGLED_TANGENTIAL_TO_RADIAL_RATIO)));
    }

    private static void printEnvelope(String label, Translation2d travelUnitVector) {
        double maxDistanceInches = ShooterConstants.DATA_COLLECTION_SHORT_RANGE_EMPIRICAL_MAX_DISTANCE_INCHES;
        double minDistanceInches = ShooterConstants.DATA_COLLECTION_SHORT_RANGE_MIN_DISTANCE_INCHES;
        double distanceStepInches = (maxDistanceInches - minDistanceInches) / (REPORT_POINT_COUNT - 1);
        System.out.printf("%n%s envelope%n", label);
        for (int i = 0; i < REPORT_POINT_COUNT; i++) {
            double targetDistanceInches = maxDistanceInches - i * distanceStepInches;
            EnvelopePointResult pointResult =
                    computeEnvelopePointResult(targetDistanceInches, travelUnitVector);
            System.out.printf(
                    "dist=%.3f maxSpeed=%.6f hood=%.3f cmd=%.3f%n",
                    targetDistanceInches,
                    pointResult.maximumTravelSpeedMetersPerSecond,
                    pointResult.selectedHoodAngleDegrees,
                    pointResult.commandedFlywheelCommandIps);
        }
    }

    private static void printSuspiciousStraightPoint(double targetDistanceInches) {
        double centerFlywheelCommandIps =
                ShortRangeHubFlywheelLookup.getFallbackManifoldFlywheelCommandIps(targetDistanceInches);
        double minimumHoodAngleDegrees = ShortRangeHubFlywheelLookup.getMinimumHoodAngleDegrees(targetDistanceInches);
        double maximumHoodAngleDegrees = ShortRangeHubFlywheelLookup.getMaximumHoodAngleDegrees(targetDistanceInches);
        System.out.printf(
                "%nSuspicious straight point %.3f in centerFlywheel=%.3f minHood=%.3f maxHood=%.3f%n",
                targetDistanceInches,
                centerFlywheelCommandIps,
                minimumHoodAngleDegrees,
                maximumHoodAngleDegrees);

        for (double speedMetersPerSecond : new double[] {0.5, 0.75, 1.0, 1.25, 1.5}) {
            printPreferredHoodScan(targetDistanceInches, speedMetersPerSecond, centerFlywheelCommandIps);
        }
    }

    private static void printPreferredHoodScan(
            double targetDistanceInches,
            double travelSpeedMetersPerSecond,
            double currentFlywheelSpeedIps) {
        Translation2d target = new Translation2d(Inches.of(targetDistanceInches).in(Meters), 0.0);
        double minimumHoodAngleDegrees = ShortRangeHubFlywheelLookup.getMinimumHoodAngleDegrees(targetDistanceInches);
        double maximumHoodAngleDegrees = ShortRangeHubFlywheelLookup.getMaximumHoodAngleDegrees(targetDistanceInches);
        System.out.printf("  speed=%.3f%n", travelSpeedMetersPerSecond);
        for (int i = 0; i <= 8; i++) {
            double interpolation = i / 8.0;
            double preferredHoodAngleDegrees =
                    minimumHoodAngleDegrees
                            + interpolation * (maximumHoodAngleDegrees - minimumHoodAngleDegrees);
            BallTrajectoryLookup.MovingShotSolution solution = new BallTrajectoryLookup.MovingShotSolution();
            MovingShotMath.EmpiricalMovingShotDebugInfo debugInfo =
                    new MovingShotMath.EmpiricalMovingShotDebugInfo();
            boolean solved = MovingShotMath.solveIdealMovingShotWithUpperHoodFallback(
                    ShooterConstants.COMMANDED_MINIMUM_ALLOWED_HOOD_ANGLE_DEGREES,
                    ShooterConstants.COMMANDED_MAXIMUM_ALLOWED_HOOD_ANGLE_DEGREES,
                    preferredHoodAngleDegrees,
                    ShooterConstants.COMMANDED_MOVING_SHOT_HOOD_SEARCH_STEP_DEGREES,
                    0.0,
                    0.0,
                    0.0,
                    travelSpeedMetersPerSecond,
                    0.0,
                    target.getX(),
                    target.getY(),
                    ShooterConstants.COMMANDED_SCORE_IN_HUB_TARGET_ELEVATION_INCHES,
                    ShooterConstants.COMMANDED_MAXIMUM_SHOOTING_HEIGHT_INCHES,
                    Rotation2d.kZero.getRadians(),
                    MIN_TURRET_ANGLE_DEGREES,
                    MAX_TURRET_ANGLE_DEGREES,
                    currentFlywheelSpeedIps,
                    currentFlywheelSpeedIps,
                    solution,
                    debugInfo);
            System.out.printf(
                    "    preferredHood=%.3f solved=%s lookup=%.3f selectedHood=%.3f modeled=%.3f commanded=%.3f tof=%.3f exit=%.3f%n",
                    preferredHoodAngleDegrees,
                    Boolean.toString(solved),
                    debugInfo.getLookupTargetDistanceInches(),
                    debugInfo.getSelectedHoodAngleDegrees(),
                    debugInfo.getModeledFlywheelCommandIps(),
                    debugInfo.getCommandedFlywheelCommandIps(),
                    BallTrajectoryLookup.getEstimatedTimeOfFlightSecondsForCommandedShot(
                            debugInfo.getSelectedHoodAngleDegrees(),
                            debugInfo.getModeledFlywheelCommandIps(),
                            debugInfo.getLookupTargetDistanceInches()),
                    BallTrajectoryLookup.getEstimatedExitVelocityIpsForCommandedShot(
                            debugInfo.getSelectedHoodAngleDegrees(),
                            debugInfo.getModeledFlywheelCommandIps()));
        }
    }

    static double findMaximumTravelSpeedMetersPerSecond(
            double targetDistanceInches,
            Translation2d travelUnitVector) {
        return computeEnvelopePointResult(targetDistanceInches, travelUnitVector).maximumTravelSpeedMetersPerSecond;
    }

    private static EnvelopePointResult computeEnvelopePointResult(
            double targetDistanceInches,
            Translation2d travelUnitVector) {
        double lowMetersPerSecond = MINIMUM_ALLOWED_TRAVEL_SPEED_METERS_PER_SECOND;
        double highMetersPerSecond = Math.max(lowMetersPerSecond, 10.0);
        if (!hasViableEmpiricalHubShot(targetDistanceInches, travelUnitVector.times(lowMetersPerSecond))) {
            return buildEnvelopePointResult(
                    targetDistanceInches,
                    travelUnitVector.times(lowMetersPerSecond),
                    lowMetersPerSecond);
        }
        while (hasViableEmpiricalHubShot(targetDistanceInches, travelUnitVector.times(highMetersPerSecond))) {
            highMetersPerSecond *= 2.0;
            if (highMetersPerSecond > 40.0) {
                return buildEnvelopePointResult(
                        targetDistanceInches,
                        travelUnitVector.times(highMetersPerSecond),
                        highMetersPerSecond);
            }
        }

        double bestViableMetersPerSecond = lowMetersPerSecond;
        for (int iteration = 0; iteration < 25; iteration++) {
            double midMetersPerSecond = 0.5 * (lowMetersPerSecond + highMetersPerSecond);
            if (hasViableEmpiricalHubShot(targetDistanceInches, travelUnitVector.times(midMetersPerSecond))) {
                bestViableMetersPerSecond = midMetersPerSecond;
                lowMetersPerSecond = midMetersPerSecond;
            } else {
                highMetersPerSecond = midMetersPerSecond;
            }
        }
        double limitedMetersPerSecond = Math.max(
                MINIMUM_ALLOWED_TRAVEL_SPEED_METERS_PER_SECOND,
                VIABLE_SPEED_MARGIN * bestViableMetersPerSecond);
        return buildEnvelopePointResult(
                targetDistanceInches,
                travelUnitVector.times(limitedMetersPerSecond),
                limitedMetersPerSecond);
    }

    private static EnvelopePointResult buildEnvelopePointResult(
            double targetDistanceInches,
            Translation2d robotFieldVelocityMetersPerSecond,
            double maximumTravelSpeedMetersPerSecond) {
        BallTrajectoryLookup.MovingShotSolution solution = new BallTrajectoryLookup.MovingShotSolution();
        MovingShotMath.EmpiricalMovingShotDebugInfo debugInfo =
                new MovingShotMath.EmpiricalMovingShotDebugInfo();
        double preferredHoodAngleDegrees =
                ShortRangeHubFlywheelLookup.getIdealHoodAngleDegrees(targetDistanceInches);
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
                robotFieldVelocityMetersPerSecond.getX(),
                robotFieldVelocityMetersPerSecond.getY(),
                Inches.of(targetDistanceInches).in(Meters),
                0.0,
                ShooterConstants.COMMANDED_SCORE_IN_HUB_TARGET_ELEVATION_INCHES,
                ShooterConstants.COMMANDED_MAXIMUM_SHOOTING_HEIGHT_INCHES,
                Rotation2d.kZero.getRadians(),
                MIN_TURRET_ANGLE_DEGREES,
                MAX_TURRET_ANGLE_DEGREES,
                currentFlywheelSpeedIps,
                currentFlywheelSpeedIps,
                solution,
                debugInfo);
        return new EnvelopePointResult(
                maximumTravelSpeedMetersPerSecond,
                solved && debugInfo.isValid() ? debugInfo.getSelectedHoodAngleDegrees() : Double.NaN,
                solved && debugInfo.isValid() ? debugInfo.getCommandedFlywheelCommandIps() : Double.NaN);
    }

    private static boolean hasViableEmpiricalHubShot(
            double targetDistanceInches,
            Translation2d robotFieldVelocityMetersPerSecond) {
        Translation2d target = new Translation2d(Inches.of(targetDistanceInches).in(Meters), 0.0);
        BallTrajectoryLookup.MovingShotSolution solution = new BallTrajectoryLookup.MovingShotSolution();
        double preferredHoodAngleDegrees =
                ShortRangeHubFlywheelLookup.getIdealHoodAngleDegrees(targetDistanceInches);
        double currentFlywheelSpeedIps =
                ShortRangeHubFlywheelLookup.getFallbackManifoldFlywheelCommandIps(targetDistanceInches);
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
                target.getX(),
                target.getY(),
                ShooterConstants.COMMANDED_SCORE_IN_HUB_TARGET_ELEVATION_INCHES,
                ShooterConstants.COMMANDED_MAXIMUM_SHOOTING_HEIGHT_INCHES,
                Rotation2d.kZero.getRadians(),
                MIN_TURRET_ANGLE_DEGREES,
                MAX_TURRET_ANGLE_DEGREES,
                currentFlywheelSpeedIps,
                currentFlywheelSpeedIps,
                true,
                solution,
                null);
    }

    private static void assertEnvelopePointsRemainFiniteAndRespectMinimumFloor(
            Translation2d travelUnitVector) {
        double maxDistanceInches = ShooterConstants.DATA_COLLECTION_SHORT_RANGE_EMPIRICAL_MAX_DISTANCE_INCHES;
        double minDistanceInches = ShooterConstants.DATA_COLLECTION_SHORT_RANGE_MIN_DISTANCE_INCHES;
        double distanceStepInches = (maxDistanceInches - minDistanceInches) / (REPORT_POINT_COUNT - 1);

        for (int i = 0; i < REPORT_POINT_COUNT; i++) {
            double targetDistanceInches = maxDistanceInches - i * distanceStepInches;
            EnvelopePointResult pointResult = computeEnvelopePointResult(targetDistanceInches, travelUnitVector);
            org.junit.jupiter.api.Assertions.assertTrue(
                    pointResult.maximumTravelSpeedMetersPerSecond
                            >= MINIMUM_ALLOWED_TRAVEL_SPEED_METERS_PER_SECOND - 1e-9,
                    () -> String.format(
                            "Expected envelope speed at %.3f in to respect the %.3f m/s floor, but got %.6f",
                            targetDistanceInches,
                            MINIMUM_ALLOWED_TRAVEL_SPEED_METERS_PER_SECOND,
                            pointResult.maximumTravelSpeedMetersPerSecond));
            org.junit.jupiter.api.Assertions.assertTrue(
                    Double.isFinite(pointResult.selectedHoodAngleDegrees),
                    () -> String.format(
                            "Expected finite hood at %.3f in, but got %.6f",
                            targetDistanceInches,
                            pointResult.selectedHoodAngleDegrees));
            org.junit.jupiter.api.Assertions.assertTrue(
                    Double.isFinite(pointResult.commandedFlywheelCommandIps),
                    () -> String.format(
                            "Expected finite commanded IPS at %.3f in, but got %.6f",
                            targetDistanceInches,
                            pointResult.commandedFlywheelCommandIps));
        }
    }
}
