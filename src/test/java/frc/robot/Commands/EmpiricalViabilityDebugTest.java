package frc.robot.Commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import org.junit.jupiter.api.Test;

class EmpiricalViabilityDebugTest {
    @Test
    void printSuspiciousEmpiricalCases() {
        printCase(221.0, 0.5, 0.0);
        printCase(221.0, 1.0, 0.0);
        printCase(86.0, 5.796, 0.0);
        printCase(77.0, 0.5, 0.0);
        printCase(77.0, 1.0, 0.0);
    }

    private static void printCase(
            double targetDistanceInches,
            double robotFieldVxMetersPerSecond,
            double robotFieldVyMetersPerSecond) {
        BallTrajectoryLookup.MovingShotSolution solution = new BallTrajectoryLookup.MovingShotSolution();
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
                robotFieldVxMetersPerSecond,
                robotFieldVyMetersPerSecond,
                Inches.of(targetDistanceInches).in(Meters),
                0.0,
                ShooterConstants.COMMANDED_SCORE_IN_HUB_TARGET_ELEVATION_INCHES,
                ShooterConstants.COMMANDED_MAXIMUM_SHOOTING_HEIGHT_INCHES,
                0.0,
                -180.0,
                180.0,
                currentFlywheelSpeedIps,
                currentFlywheelSpeedIps,
                solution);

        double tofSeconds = solved
                ? BallTrajectoryLookup.getEstimatedTimeOfFlightSecondsForCommandedShot(
                        solution.getHoodAngleDegrees(),
                        solution.getFlywheelCommandIps(),
                        targetDistanceInches)
                : Double.NaN;

        System.out.printf(
                "dist=%.1f vx=%.3f vy=%.3f solved=%s hood=%.3f cmd=%.3f tof=%.3f%n",
                targetDistanceInches,
                robotFieldVxMetersPerSecond,
                robotFieldVyMetersPerSecond,
                Boolean.toString(solved),
                solution.getHoodAngleDegrees(),
                solution.getFlywheelCommandIps(),
                tofSeconds);
    }
}
