package frc.robot.Commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class MovingShotMathTest {
    private static final double SEARCH_STEP_DEGREES =
            ShooterConstants.COMMANDED_MOVING_SHOT_FIXED_FLYWHEEL_HOOD_SEARCH_STEP_DEGREES;
    private static final double TARGET_ELEVATION_INCHES =
            ShooterConstants.COMMANDED_SCORE_IN_HUB_TARGET_ELEVATION_INCHES;
    private static final double MAX_HEIGHT_INCHES =
            ShooterConstants.COMMANDED_TEST_SHOOTING_MAXIMUM_HEIGHT_INCHES;
    private static final double MIN_TURRET_ANGLE_DEGREES = -180.0;
    private static final double MAX_TURRET_ANGLE_DEGREES = 180.0;

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
                BallTrajectoryLookup.LUT_MIN_HOOD_ANGLE_DEGREES,
                BallTrajectoryLookup.LUT_MAX_HOOD_ANGLE_DEGREES,
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
    void empiricalMovingShotUsesCommandedSpeedDirectly() {
        double targetDistanceInches = ShooterConstants.DATA_COLLECTION_SHORT_RANGE_DISTANCES_INCHES[2];
        double hoodAngleDegrees = ShooterConstants.DATA_COLLECTION_SHORT_RANGE_HOOD_ANGLES_DEGREES[2][1];
        double flywheelCommandIps = ShooterConstants.DATA_COLLECTION_SHORT_RANGE_COMMAND_SPEEDS_IPS[2][1];
        BallTrajectoryLookup.MovingShotSolution solution = new BallTrajectoryLookup.MovingShotSolution();

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
        assertEquals(flywheelCommandIps, solution.getLauncherRelativeExitVelocityIps(), 1e-9,
                "Empirical interpolation path should use commanded flywheel speed directly");
        assertEquals(flywheelCommandIps, solution.getFieldRelativeExitVelocityIps(), 1e-9,
                "Stationary empirical path should keep field-relative speed equal to commanded speed");
    }
}
