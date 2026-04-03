package frc.robot.Commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

import org.junit.jupiter.api.Test;

class BallTrajectoryLookupTest {
    private static final double FRAME_TO_CENTER_DISTANCE_INCHES =
            ShooterConstants.MEASURED_FRAME_TO_CENTER_DISTANCE_INCHES;
    private static final double GRAVITY_IPS2 = 386.08858267716535;
    private static final double MIN_COMMANDED_FLYWHEEL_SET_IPS = ShooterConstants.COMMANDED_FLYWHEEL_SET_IPS[0];
    private static final double MAX_COMMANDED_FLYWHEEL_SET_IPS =
            ShooterConstants.COMMANDED_FLYWHEEL_SET_IPS[ShooterConstants.COMMANDED_FLYWHEEL_SET_IPS.length - 1];
    private static final double MAX_FITTED_BALL_EXIT_IPS =
            ShooterConstants.FITTED_BALL_EXIT_IPS[ShooterConstants.FITTED_BALL_EXIT_IPS.length - 1];
    private static final double[] TABLE_COMMAND_SPEEDS_IPS =
            Arrays.copyOf(
                    ShooterConstants.COMMANDED_FLYWHEEL_SET_IPS,
                    ShooterConstants.MEASURED_DISTANCE_GRID_INCHES.length);
    private static final double[] TABLE_BALL_EXIT_IPS =
            Arrays.copyOf(
                    ShooterConstants.FITTED_BALL_EXIT_IPS,
                    ShooterConstants.MEASURED_DISTANCE_GRID_INCHES.length);
    private static final double LANDING_DISTANCE_TOLERANCE_INCHES = 0.01;
    private static final int SHORT_RANGE_THRESHOLD_INCHES = 150;
    private static final int MAX_REPORTED_ERRORS = 10;

    private record SampleError(
            double angleDegrees,
            double commandSpeedIps,
            double effectiveExitVelocityIps,
            double measuredDistanceInches,
            double predictedDistanceInches,
            double errorInches) {
    }

    private record HeightDiscrepancy(
            double angleDegrees,
            double targetDistanceInches,
            double targetElevationInches,
            double lookupMaximumHeightInches,
            double algebraicMaximumHeightInches,
            double absoluteDifferenceInches,
            double percentDifference) {
    }

    private record PhysicalLimitSample(
            double angleDegrees,
            double targetDistanceInches,
            double targetElevationInches,
            double requiredExitVelocityIps,
            double estimatedCommandIps,
            double maximumHeightInches) {
    }

    @Test
    void empiricalDistancesMatchBallTrajectoryModelWithinExpectedError() {
        List<SampleError> sampleErrors = new ArrayList<>();
        SampleError worstOverall = null;
        SampleError worstBelow150 = null;

        for (int row = 0; row < ShooterConstants.MEASURED_DISTANCE_GRID_INCHES.length; row++) {
            for (int col = 0; col < ShooterConstants.MEASURED_DISTANCE_GRID_INCHES[row].length; col++) {
                double measuredDistanceInches = ShooterConstants.MEASURED_DISTANCE_GRID_INCHES[row][col];
                if (measuredDistanceInches <= 0.0) {
                    continue;
                }

                double angleDegrees = ShooterConstants.MEASURED_ACTUAL_ANGLES_DEGREES[col];
                double effectiveExitVelocityIps =
                        TABLE_BALL_EXIT_IPS[row]
                                * ShooterConstants.FITTED_COMMAND_ANGLE_EXIT_SCALES[col];
                double predictedDistanceInches =
                        findLandingDistanceInches(angleDegrees, effectiveExitVelocityIps);
                double errorInches = predictedDistanceInches - measuredDistanceInches;

                SampleError sampleError = new SampleError(
                        angleDegrees,
                        TABLE_COMMAND_SPEEDS_IPS[row],
                        effectiveExitVelocityIps,
                        measuredDistanceInches,
                        predictedDistanceInches,
                        errorInches);
                sampleErrors.add(sampleError);

                if (worstOverall == null
                        || Math.abs(sampleError.errorInches()) > Math.abs(worstOverall.errorInches())) {
                    worstOverall = sampleError;
                }

                if (measuredDistanceInches < SHORT_RANGE_THRESHOLD_INCHES
                        && (worstBelow150 == null
                                || Math.abs(sampleError.errorInches())
                                        > Math.abs(worstBelow150.errorInches()))) {
                    worstBelow150 = sampleError;
                }
            }
        }

        sampleErrors.sort(Comparator.comparingDouble(sampleError -> -Math.abs(sampleError.errorInches())));

        System.out.println("BallTrajectoryLookup empirical comparison:");
        System.out.printf(
                "Largest error below %d in: %.3f in at range %.3f in, hood angle %.1f deg (command %.0f ips, predicted %.3f in)%n",
                SHORT_RANGE_THRESHOLD_INCHES,
                Math.abs(worstBelow150.errorInches()),
                worstBelow150.measuredDistanceInches(),
                worstBelow150.angleDegrees(),
                worstBelow150.commandSpeedIps(),
                worstBelow150.predictedDistanceInches());
        System.out.printf(
                "Largest error overall: %.3f in at range %.3f in, hood angle %.1f deg (command %.0f ips, predicted %.3f in)%n",
                Math.abs(worstOverall.errorInches()),
                worstOverall.measuredDistanceInches(),
                worstOverall.angleDegrees(),
                worstOverall.commandSpeedIps(),
                worstOverall.predictedDistanceInches());

        System.out.println("Top sample errors:");
        for (int i = 0; i < Math.min(MAX_REPORTED_ERRORS, sampleErrors.size()); i++) {
            SampleError sampleError = sampleErrors.get(i);
            System.out.printf(
                    "  hood angle %.1f deg | range %.3f in | command %.0f ips | exit %.3f ips | predicted %.3f in | error %+,.3f in%n",
                    sampleError.angleDegrees(),
                    sampleError.measuredDistanceInches(),
                    sampleError.commandSpeedIps(),
                    sampleError.effectiveExitVelocityIps(),
                    sampleError.predictedDistanceInches(),
                    sampleError.errorInches());
        }

        assertTrue(Math.abs(worstBelow150.errorInches()) < 5.1,
                "Expected short-range worst error to stay near the baked ~5.0 in");
        assertTrue(Math.abs(worstOverall.errorInches()) < 8.6,
                "Expected overall worst error to stay near the current baked ~8.5 in");
    }

    @Test
    void lookupMaximumHeightsLookReasonableComparedToNoDragPeak() {
        List<HeightDiscrepancy> worstDiscrepanciesByAngle = new ArrayList<>();
        HeightDiscrepancy worstOverall = null;

        for (double angleDegrees : ShooterConstants.MEASURED_ACTUAL_ANGLES_DEGREES) {
            HeightDiscrepancy worstForAngle = null;
            for (int elevationIndex = 0;
                    elevationIndex <= ShooterConstants.FITTED_BALL_TRAJECTORY_LUT_MAX_TARGET_ELEVATION_FEET;
                    elevationIndex++) {
                double targetElevationInches =
                        elevationIndex * ShooterConstants.FITTED_BALL_TRAJECTORY_LUT_ELEVATION_STEP_INCHES;
                for (int targetDistanceInches = 1;
                        targetDistanceInches <= ShooterConstants.FITTED_BALL_TRAJECTORY_LUT_MAX_TARGET_DISTANCE_INCHES;
                        targetDistanceInches++) {
                    double requiredExitVelocityIps = BallTrajectoryLookup.getRequiredExitVelocityIps(
                            angleDegrees,
                            targetDistanceInches,
                            targetElevationInches);
                    if (!Double.isFinite(requiredExitVelocityIps)) {
                        continue;
                    }

                    double lookupMaximumHeightInches = BallTrajectoryLookup.getMaximumBallZElevationInches(
                            angleDegrees,
                            targetDistanceInches,
                            targetElevationInches);
                    double algebraicMaximumHeightInches =
                            getAlgebraicNoDragMaximumHeightInches(angleDegrees, requiredExitVelocityIps);
                    if (!Double.isFinite(lookupMaximumHeightInches)
                            || !Double.isFinite(algebraicMaximumHeightInches)) {
                        continue;
                    }

                    double absoluteDifferenceInches =
                            Math.abs(lookupMaximumHeightInches - algebraicMaximumHeightInches);
                    double percentDifference = 100.0 * absoluteDifferenceInches
                            / Math.max(1e-9, Math.abs(algebraicMaximumHeightInches));

                    HeightDiscrepancy discrepancy = new HeightDiscrepancy(
                            angleDegrees,
                            targetDistanceInches,
                            targetElevationInches,
                            lookupMaximumHeightInches,
                            algebraicMaximumHeightInches,
                            absoluteDifferenceInches,
                            percentDifference);

                    if (worstForAngle == null
                            || discrepancy.absoluteDifferenceInches()
                                    > worstForAngle.absoluteDifferenceInches()) {
                        worstForAngle = discrepancy;
                    }
                    if (worstOverall == null
                            || discrepancy.absoluteDifferenceInches()
                                    > worstOverall.absoluteDifferenceInches()) {
                        worstOverall = discrepancy;
                    }
                }
            }

            assertNotNull(worstForAngle, "Expected at least one valid trajectory for hood angle " + angleDegrees);
            worstDiscrepanciesByAngle.add(worstForAngle);
        }

        System.out.println("BallTrajectoryLookup max-height sanity check vs no-drag algebraic apex:");
        for (HeightDiscrepancy discrepancy : worstDiscrepanciesByAngle) {
            System.out.printf(
                    "  hood angle %.1f deg | worst at range %.0f in, target z %.0f in | LUT max %.3f in | algebraic max %.3f in | abs diff %.3f in | diff %.2f%%%n",
                    discrepancy.angleDegrees(),
                    discrepancy.targetDistanceInches(),
                    discrepancy.targetElevationInches(),
                    discrepancy.lookupMaximumHeightInches(),
                    discrepancy.algebraicMaximumHeightInches(),
                    discrepancy.absoluteDifferenceInches(),
                    discrepancy.percentDifference());
        }

        assertNotNull(worstOverall, "Expected at least one valid trajectory for the max-height sanity sweep");
        System.out.printf(
                "Largest max-height discrepancy overall: hood angle %.1f deg | range %.0f in | target z %.0f in | LUT max %.3f in | algebraic max %.3f in | abs diff %.3f in | diff %.2f%%%n",
                worstOverall.angleDegrees(),
                worstOverall.targetDistanceInches(),
                worstOverall.targetElevationInches(),
                worstOverall.lookupMaximumHeightInches(),
                worstOverall.algebraicMaximumHeightInches(),
                worstOverall.absoluteDifferenceInches(),
                worstOverall.percentDifference());

        assertTrue(Double.isFinite(worstOverall.absoluteDifferenceInches()),
                "Expected finite max-height discrepancy results");
    }

    @Test
    void lookupOnlyReportsPhysicallyAchievableShots() {
        PhysicalLimitSample highestApexSample = null;
        PhysicalLimitSample highestCommandSample = null;

        for (double angleDegrees = ShooterConstants.FITTED_BALL_TRAJECTORY_LUT_MIN_HOOD_ANGLE_DEGREES;
                angleDegrees <= ShooterConstants.FITTED_BALL_TRAJECTORY_LUT_MAX_HOOD_ANGLE_DEGREES + 1e-9;
                angleDegrees += ShooterConstants.FITTED_BALL_TRAJECTORY_LUT_HOOD_ANGLE_STEP_DEGREES) {
            for (int elevationIndex = 0;
                    elevationIndex <= ShooterConstants.FITTED_BALL_TRAJECTORY_LUT_MAX_TARGET_ELEVATION_FEET;
                    elevationIndex++) {
                double targetElevationInches =
                        elevationIndex * ShooterConstants.FITTED_BALL_TRAJECTORY_LUT_ELEVATION_STEP_INCHES;
                for (int targetDistanceInches = 1;
                        targetDistanceInches <= ShooterConstants.FITTED_BALL_TRAJECTORY_LUT_MAX_TARGET_DISTANCE_INCHES;
                        targetDistanceInches++) {
                    double requiredExitVelocityIps = BallTrajectoryLookup.getRequiredExitVelocityIps(
                            angleDegrees,
                            targetDistanceInches,
                            targetElevationInches);
                    if (!Double.isFinite(requiredExitVelocityIps)) {
                        continue;
                    }

                    double angleExitScale = getAngleExitScale(angleDegrees);
                    double maximumPhysicalExitVelocityIps = MAX_FITTED_BALL_EXIT_IPS * angleExitScale;
                    assertTrue(requiredExitVelocityIps <= maximumPhysicalExitVelocityIps + 1e-3,
                            String.format(
                                    "Lookup exceeded physical exit ceiling at %.1f deg, range %.0f in, target z %.0f in: required %.3f ips > max %.3f ips",
                                    angleDegrees,
                                    (double) targetDistanceInches,
                                    targetElevationInches,
                                    requiredExitVelocityIps,
                                    maximumPhysicalExitVelocityIps));

                    double baseExitVelocityIps = requiredExitVelocityIps / angleExitScale;
                    double estimatedCommandIps =
                            FlywheelBallExitInterpolator.getSetIpsForBallExitIps(baseExitVelocityIps);
                    assertTrue(Double.isFinite(estimatedCommandIps),
                            String.format(
                                    "Expected finite command estimate at %.1f deg, range %.0f in, target z %.0f in: base exit %.3f ips",
                                    angleDegrees,
                                    (double) targetDistanceInches,
                                    targetElevationInches,
                                    baseExitVelocityIps));
                    assertTrue(estimatedCommandIps >= MIN_COMMANDED_FLYWHEEL_SET_IPS - 1e-3,
                            String.format(
                                    "Lookup dipped below flywheel floor at %.1f deg, range %.0f in, target z %.0f in: required %.3f ips command < min %.3f ips",
                                    angleDegrees,
                                    (double) targetDistanceInches,
                                    targetElevationInches,
                                    estimatedCommandIps,
                                    MIN_COMMANDED_FLYWHEEL_SET_IPS));
                    assertTrue(estimatedCommandIps <= MAX_COMMANDED_FLYWHEEL_SET_IPS + 1e-3,
                            String.format(
                                    "Lookup exceeded flywheel ceiling at %.1f deg, range %.0f in, target z %.0f in: required %.3f ips command > max %.3f ips",
                                    angleDegrees,
                                    (double) targetDistanceInches,
                                    targetElevationInches,
                                    estimatedCommandIps,
                                    MAX_COMMANDED_FLYWHEEL_SET_IPS));

                    double maximumHeightInches = BallTrajectoryLookup.getMaximumBallZElevationInches(
                            angleDegrees,
                            targetDistanceInches,
                            targetElevationInches);
                    PhysicalLimitSample sample = new PhysicalLimitSample(
                            angleDegrees,
                            targetDistanceInches,
                            targetElevationInches,
                            requiredExitVelocityIps,
                            estimatedCommandIps,
                            maximumHeightInches);

                    if (highestApexSample == null
                            || sample.maximumHeightInches() > highestApexSample.maximumHeightInches()) {
                        highestApexSample = sample;
                    }
                    if (highestCommandSample == null
                            || sample.estimatedCommandIps() > highestCommandSample.estimatedCommandIps()) {
                        highestCommandSample = sample;
                    }
                }
            }
        }

        assertNotNull(highestApexSample, "Expected at least one physically valid LUT shot");
        assertNotNull(highestCommandSample, "Expected at least one physically valid LUT shot");

        System.out.printf(
                "Highest LUT apex within device limits: hood angle %.1f deg | range %.0f in | target z %.0f in | max height %.3f in | required exit %.3f ips | estimated command %.3f ips%n",
                highestApexSample.angleDegrees(),
                highestApexSample.targetDistanceInches(),
                highestApexSample.targetElevationInches(),
                highestApexSample.maximumHeightInches(),
                highestApexSample.requiredExitVelocityIps(),
                highestApexSample.estimatedCommandIps());
        System.out.printf(
                "Highest LUT command within device limits: hood angle %.1f deg | range %.0f in | target z %.0f in | required exit %.3f ips | estimated command %.3f ips | max height %.3f in%n",
                highestCommandSample.angleDegrees(),
                highestCommandSample.targetDistanceInches(),
                highestCommandSample.targetElevationInches(),
                highestCommandSample.requiredExitVelocityIps(),
                highestCommandSample.estimatedCommandIps(),
                highestCommandSample.maximumHeightInches());
    }

    @Test
    void movingShotTurretDeltaUsesRobotLeftPositiveConvention() {
        BallTrajectoryLookup.MovingShotSolution solution = new BallTrajectoryLookup.MovingShotSolution();

        boolean solved = BallTrajectoryLookup.solveMovingShot(
                ShooterConstants.FITTED_BALL_TRAJECTORY_LUT_MIN_HOOD_ANGLE_DEGREES,
                ShooterConstants.FITTED_BALL_TRAJECTORY_LUT_MAX_HOOD_ANGLE_DEGREES,
                ShooterConstants.COMMANDED_MOVING_SHOT_HOOD_SEARCH_STEP_DEGREES,
                true,
                0.0,
                0.0,
                Math.toRadians(10.0),
                0.0,
                0.0,
                Inches.of(120.0).in(Meters),
                0.0,
                ShooterConstants.COMMANDED_SCORE_IN_HUB_TARGET_ELEVATION_INCHES,
                ShooterConstants.COMMANDED_MAXIMUM_SHOOTING_HEIGHT_INCHES,
                Math.toRadians(10.0),
                -18.0,
                18.0,
                solution);

        assertTrue(solved, "Expected a valid stationary moving-shot solution for sign-convention test");
        assertEquals(-10.0, solution.getTurretDeltaDegrees(), 0.25,
                "Positive turret delta should mean robot-left, matching CCW-positive horizontal aim");
        assertEquals(10.0, solution.getRobotHeadingDegrees(), 0.25,
                "Preferred heading should be preserved when the turret can absorb the full offset");
        assertEquals(0.0, solution.getShotAzimuthDegrees(), 0.25,
                "Target placed directly downfield should yield zero shot azimuth");
    }

    private static double findLandingDistanceInches(double hoodAngleDegrees, double exitVelocityIps) {
        double lowCenterDistanceInches =
                BallTrajectoryLookup.getLaunchXInches(hoodAngleDegrees)
                        + FRAME_TO_CENTER_DISTANCE_INCHES;
        double highCenterDistanceInches = Math.max(lowCenterDistanceInches + 1.0, 100.0);

        while (Double.isFinite(BallTrajectoryLookup.getHeightAtDistanceInches(
                hoodAngleDegrees,
                exitVelocityIps,
                highCenterDistanceInches))) {
            highCenterDistanceInches *= 1.5;
            if (highCenterDistanceInches > 2000.0) {
                throw new IllegalStateException("Unable to bracket landing distance for shot.");
            }
        }

        for (int iteration = 0; iteration < 80; iteration++) {
            double midCenterDistanceInches =
                    0.5 * (lowCenterDistanceInches + highCenterDistanceInches);
            double heightAtMid = BallTrajectoryLookup.getHeightAtDistanceInches(
                    hoodAngleDegrees,
                    exitVelocityIps,
                    midCenterDistanceInches);

            if (Double.isFinite(heightAtMid)) {
                lowCenterDistanceInches = midCenterDistanceInches;
            } else {
                highCenterDistanceInches = midCenterDistanceInches;
            }

            if (highCenterDistanceInches - lowCenterDistanceInches
                    <= LANDING_DISTANCE_TOLERANCE_INCHES) {
                return lowCenterDistanceInches - FRAME_TO_CENTER_DISTANCE_INCHES;
            }
        }

        return lowCenterDistanceInches - FRAME_TO_CENTER_DISTANCE_INCHES;
    }

    private static double getAlgebraicNoDragMaximumHeightInches(
            double hoodAngleDegrees,
            double exitVelocityIps) {
        double launchHeightInches = BallTrajectoryLookup.getLaunchZInches(hoodAngleDegrees);
        double verticalExitVelocityIps = exitVelocityIps * Math.sin(Math.toRadians(hoodAngleDegrees));
        return launchHeightInches + (verticalExitVelocityIps * verticalExitVelocityIps) / (2.0 * GRAVITY_IPS2);
    }

    private static double getAngleExitScale(double hoodAngleDegrees) {
        double[] sampleAnglesDegrees = ShooterConstants.MEASURED_ACTUAL_ANGLES_DEGREES;
        double[] angleExitScales = ShooterConstants.FITTED_COMMAND_ANGLE_EXIT_SCALES;

        for (int i = 0; i < sampleAnglesDegrees.length; i++) {
            if (Math.abs(hoodAngleDegrees - sampleAnglesDegrees[i]) <= 1e-9) {
                return angleExitScales[i];
            }
        }

        boolean ascendingAngles = sampleAnglesDegrees[0] <= sampleAnglesDegrees[sampleAnglesDegrees.length - 1];
        if (ascendingAngles) {
            if (hoodAngleDegrees <= sampleAnglesDegrees[0]) {
                return angleExitScales[0];
            }
            if (hoodAngleDegrees >= sampleAnglesDegrees[sampleAnglesDegrees.length - 1]) {
                return angleExitScales[angleExitScales.length - 1];
            }
        } else {
            if (hoodAngleDegrees >= sampleAnglesDegrees[0]) {
                return angleExitScales[0];
            }
            if (hoodAngleDegrees <= sampleAnglesDegrees[sampleAnglesDegrees.length - 1]) {
                return angleExitScales[angleExitScales.length - 1];
            }
        }

        for (int i = 0; i < sampleAnglesDegrees.length - 1; i++) {
            double angle1 = sampleAnglesDegrees[i];
            double angle2 = sampleAnglesDegrees[i + 1];
            if (isBetween(hoodAngleDegrees, angle1, angle2)) {
                double scale1 = angleExitScales[i];
                double scale2 = angleExitScales[i + 1];
                return scale1 + (scale2 - scale1) * ((hoodAngleDegrees - angle1) / (angle2 - angle1));
            }
        }

        throw new IllegalStateException("Unable to interpolate angle exit scale for " + hoodAngleDegrees);
    }

    private static boolean isBetween(double value, double bound1, double bound2) {
        return value >= Math.min(bound1, bound2) && value <= Math.max(bound1, bound2);
    }
}
