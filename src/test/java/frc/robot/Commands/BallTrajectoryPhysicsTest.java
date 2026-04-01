package frc.robot.Commands;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import org.junit.jupiter.api.Test;

class BallTrajectoryPhysicsTest {
    private static final double[] ROW_EXIT_CORRECTIONS_IPS = {
        -9.453, -9.766, -8.281, -1.484, 5.156, 13.047, 17.031,
        22.266, 26.797, 31.094, 35.547, 40.938, 47.813, 54.375
    };
    private static final double FRAME_TO_CENTER_DISTANCE_INCHES =
            ShooterConstants.FRAME_TO_CENTER_DISTANCE_INCHES;
    private static final double LANDING_DISTANCE_TOLERANCE_INCHES = 0.01;
    private static final int SHORT_RANGE_THRESHOLD_INCHES = 150;
    private static final int MAX_REPORTED_ERRORS = 10;
    private static final double VELOCITY_STEP_IPS = 36.0;
    private static final double MAX_TEST_VELOCITY_IPS = 180.0;
    private static final double MONOTONIC_EPSILON_IPS = 0.25;

    private record SampleError(
            double angleDegrees,
            double commandSpeedIps,
            double effectiveExitVelocityIps,
            double measuredDistanceInches,
            double predictedDistanceInches,
            double errorInches) {
    }

    private record MonotonicityViolation(
            double targetDistanceInches,
            double targetElevationInches,
            double robotVxIps,
            double robotVyIps,
            double previousAngleDegrees,
            double currentAngleDegrees,
            double previousRequiredExitIps,
            double currentRequiredExitIps,
            int expectedDirectionSign,
            int observedDirectionSign) {
    }

    @Test
    void empiricalDistancesMatchBallTrajectoryModelWithinExpectedError() {
        List<SampleError> sampleErrors = new ArrayList<>();
        SampleError worstOverall = null;
        SampleError worstBelow150 = null;

        for (int row = 0; row < ShooterConstants.DISTANCE_GRID_INCHES.length; row++) {
            for (int col = 0; col < ShooterConstants.DISTANCE_GRID_INCHES[row].length; col++) {
                double measuredDistanceInches = ShooterConstants.DISTANCE_GRID_INCHES[row][col];
                if (measuredDistanceInches <= 0.0) {
                    continue;
                }

                double angleDegrees = ShooterConstants.ACTUAL_ANGLES_DEGREES[col];
                double effectiveExitVelocityIps =
                        ShooterConstants.COMMAND_BALL_EXIT_IPS[row]
                                * ShooterConstants.COMMAND_ANGLE_EXIT_SCALES[col]
                                + ROW_EXIT_CORRECTIONS_IPS[row];
                double predictedDistanceInches =
                        findLandingDistanceInches(angleDegrees, effectiveExitVelocityIps);
                double errorInches = predictedDistanceInches - measuredDistanceInches;

                SampleError sampleError = new SampleError(
                        angleDegrees,
                        ShooterConstants.COMMAND_SPEEDS_IPS[row],
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

        System.out.println("BallTrajectoryPhysics empirical comparison:");
        System.out.printf(
                "Largest error below %d in: %.3f in at %.3f in (angle %.1f deg, command %.0f ips, predicted %.3f in)%n",
                SHORT_RANGE_THRESHOLD_INCHES,
                Math.abs(worstBelow150.errorInches()),
                worstBelow150.measuredDistanceInches(),
                worstBelow150.angleDegrees(),
                worstBelow150.commandSpeedIps(),
                worstBelow150.predictedDistanceInches());
        System.out.printf(
                "Largest error overall: %.3f in at %.3f in (angle %.1f deg, command %.0f ips, predicted %.3f in)%n",
                Math.abs(worstOverall.errorInches()),
                worstOverall.measuredDistanceInches(),
                worstOverall.angleDegrees(),
                worstOverall.commandSpeedIps(),
                worstOverall.predictedDistanceInches());

        System.out.println("Top sample errors:");
        for (int i = 0; i < Math.min(MAX_REPORTED_ERRORS, sampleErrors.size()); i++) {
            SampleError sampleError = sampleErrors.get(i);
            System.out.printf(
                    "  angle %.1f deg | command %.0f ips | exit %.3f ips | measured %.3f in | predicted %.3f in | error %+,.3f in%n",
                    sampleError.angleDegrees(),
                    sampleError.commandSpeedIps(),
                    sampleError.effectiveExitVelocityIps(),
                    sampleError.measuredDistanceInches(),
                    sampleError.predictedDistanceInches(),
                    sampleError.errorInches());
        }

        assertTrue(Math.abs(worstBelow150.errorInches()) < 6.1,
                "Expected short-range worst error to stay near the fitted ~5.9 in");
        assertTrue(Math.abs(worstOverall.errorInches()) < 6.1,
                "Expected overall worst error to stay near the fitted ~5.9 in");
    }

    @Test
    void compensatedRequiredExitSpeedIsMonotonicAcrossHoodAngle() {
        List<MonotonicityViolation> violations = new ArrayList<>();

        for (int targetDistanceInches = 0;
                targetDistanceInches <= BallTrajectoryPhysics.LUT_MAX_TARGET_DISTANCE_INCHES;
                targetDistanceInches += 50) {
            for (int elevationFeet = 0;
                    elevationFeet <= BallTrajectoryPhysics.LUT_MAX_TARGET_ELEVATION_FEET;
                    elevationFeet++) {
                double targetElevationInches = elevationFeet * BallTrajectoryPhysics.LUT_ELEVATION_STEP_INCHES;

                for (double robotVxIps = -MAX_TEST_VELOCITY_IPS;
                        robotVxIps <= MAX_TEST_VELOCITY_IPS + 1e-9;
                        robotVxIps += VELOCITY_STEP_IPS) {
                    for (double robotVyIps = -MAX_TEST_VELOCITY_IPS;
                            robotVyIps <= MAX_TEST_VELOCITY_IPS + 1e-9;
                            robotVyIps += VELOCITY_STEP_IPS) {
                        double previousRequiredExitIps = Double.NaN;
                        double previousAngleDegrees = Double.NaN;
                        int monotonicDirectionSign = 0;

                        for (int angleIndex = 0;
                                angleIndex < BallTrajectoryPhysics.LUT_HOOD_ANGLE_BIN_COUNT;
                                angleIndex++) {
                            double angleDegrees = BallTrajectoryPhysics.LUT_MIN_HOOD_ANGLE_DEGREES
                                    + angleIndex * BallTrajectoryPhysics.LUT_HOOD_ANGLE_STEP_DEGREES;
                            double requiredFieldExitIps = BallTrajectoryPhysics.getRequiredExitVelocityIps(
                                    angleDegrees,
                                    targetDistanceInches,
                                    targetElevationInches);
                            if (!Double.isFinite(requiredFieldExitIps)) {
                                continue;
                            }

                            double angleRadians = Math.toRadians(angleDegrees);
                            double requiredHorizontalExitIps = requiredFieldExitIps * Math.cos(angleRadians);
                            double requiredVerticalExitIps = requiredFieldExitIps * Math.sin(angleRadians);
                            double launcherRelativeVxIps = requiredHorizontalExitIps - robotVxIps;
                            double launcherRelativeVyIps = -robotVyIps;
                            double requiredLauncherExitIps = Math.hypot(
                                    requiredVerticalExitIps,
                                    Math.hypot(launcherRelativeVxIps, launcherRelativeVyIps));

                            if (Double.isFinite(previousRequiredExitIps)) {
                                double deltaIps = requiredLauncherExitIps - previousRequiredExitIps;
                                int observedDirectionSign = getDirectionSign(deltaIps);

                                if (observedDirectionSign != 0 && monotonicDirectionSign == 0) {
                                    monotonicDirectionSign = observedDirectionSign;
                                } else if (observedDirectionSign != 0
                                        && monotonicDirectionSign != 0
                                        && observedDirectionSign != monotonicDirectionSign) {
                                    violations.add(new MonotonicityViolation(
                                            targetDistanceInches,
                                            targetElevationInches,
                                            robotVxIps,
                                            robotVyIps,
                                            previousAngleDegrees,
                                            angleDegrees,
                                            previousRequiredExitIps,
                                            requiredLauncherExitIps,
                                            monotonicDirectionSign,
                                            observedDirectionSign));
                                }
                            }

                            previousRequiredExitIps = requiredLauncherExitIps;
                            previousAngleDegrees = angleDegrees;
                        }
                    }
                }
            }
        }

        System.out.printf(
                "Monotonicity sweep completed. Violations found: %d%n",
                violations.size());
        for (int i = 0; i < Math.min(MAX_REPORTED_ERRORS, violations.size()); i++) {
            MonotonicityViolation violation = violations.get(i);
            System.out.printf(
                    "  target %.0f in @ %.0f in | robotV=(%+.0f,%+.0f) ips | angle %.1f -> %.1f deg | exit %.3f -> %.3f ips%n",
                    violation.targetDistanceInches(),
                    violation.targetElevationInches(),
                    violation.robotVxIps(),
                    violation.robotVyIps(),
                    violation.previousAngleDegrees(),
                    violation.currentAngleDegrees(),
                    violation.previousRequiredExitIps(),
                    violation.currentRequiredExitIps());
        }

        assertTrue(violations.isEmpty(),
                "Expected compensated required exit speed to remain monotonic over hood angle.");
    }

    private static int getDirectionSign(double deltaIps) {
        if (deltaIps > MONOTONIC_EPSILON_IPS) {
            return 1;
        }
        if (deltaIps < -MONOTONIC_EPSILON_IPS) {
            return -1;
        }
        return 0;
    }

    private static double findLandingDistanceInches(double hoodAngleDegrees, double exitVelocityIps) {
        double lowCenterDistanceInches =
                BallTrajectoryPhysics.getLaunchXInches(hoodAngleDegrees)
                        + FRAME_TO_CENTER_DISTANCE_INCHES;
        double highCenterDistanceInches = Math.max(lowCenterDistanceInches + 1.0, 100.0);

        while (Double.isFinite(BallTrajectoryPhysics.getHeightAtDistanceInches(
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
            double heightAtMid = BallTrajectoryPhysics.getHeightAtDistanceInches(
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
}
