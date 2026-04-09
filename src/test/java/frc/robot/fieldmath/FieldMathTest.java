package frc.robot.fieldmath;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

class FieldMathTest {
    private static final double FIELD_LENGTH_METERS = Meters.convertFrom(651.2, Inches);
    private static final double FIELD_WIDTH_METERS = Meters.convertFrom(317.7, Inches);
    private static final double ENGINEER_TARGET_SIDE_INSET_METERS = Meters.convertFrom(3.0, Feet);
    private static final double ENGINEER_TARGET_FORWARD_LIMIT_METERS = Meters.convertFrom(170.0, Inches);
    private static final double ENGINEER_TARGET_FORWARD_HALF_RANGE_METERS =
            ENGINEER_TARGET_FORWARD_LIMIT_METERS * 0.5;
    private static final double ENGINEER_TARGET_LATERAL_HALF_RANGE_METERS =
            (FIELD_WIDTH_METERS - 2.0 * ENGINEER_TARGET_SIDE_INSET_METERS) * 0.5;
    private static final double ENGINEER_TARGET_JOYSTICK_LATERAL_LIMIT =
            ENGINEER_TARGET_LATERAL_HALF_RANGE_METERS
                    / Math.hypot(
                            ENGINEER_TARGET_LATERAL_HALF_RANGE_METERS,
                            ENGINEER_TARGET_FORWARD_HALF_RANGE_METERS);
    private static final double ENGINEER_TARGET_JOYSTICK_FORWARD_LIMIT =
            ENGINEER_TARGET_FORWARD_HALF_RANGE_METERS
                    / Math.hypot(
                            ENGINEER_TARGET_LATERAL_HALF_RANGE_METERS,
                            ENGINEER_TARGET_FORWARD_HALF_RANGE_METERS);
    private static final double HUB_TARGET_ENGINEER_ADJUSTMENT_LIMIT_METERS = Meters.convertFrom(3.0, Feet);

    @Test
    void zeroStickTargetsAllianceZoneCenterForBlueAlliance() {
        Translation2d target = FieldMath.getEngineerTargetInAllianceZone(Alliance.Blue, 0.0, 0.0);

        assertEquals(ENGINEER_TARGET_FORWARD_LIMIT_METERS * 0.5, target.getX(), 1e-9);
        assertEquals(FIELD_WIDTH_METERS * 0.5, target.getY(), 1e-9);
    }

    @Test
    void fullForwardClipsToForwardEdgeCenter() {
        Translation2d target = FieldMath.getEngineerTargetInAllianceZone(Alliance.Blue, 0.0, -1.0);

        assertEquals(ENGINEER_TARGET_FORWARD_LIMIT_METERS, target.getX(), 1e-9);
        assertEquals(FIELD_WIDTH_METERS * 0.5, target.getY(), 1e-9);
    }

    @Test
    void inscribedCircleCornerReachesForwardRightCornerForBlueAlliance() {
        Translation2d target = FieldMath.getEngineerTargetInAllianceZone(
                Alliance.Blue,
                ENGINEER_TARGET_JOYSTICK_LATERAL_LIMIT,
                -ENGINEER_TARGET_JOYSTICK_FORWARD_LIMIT);

        assertEquals(ENGINEER_TARGET_FORWARD_LIMIT_METERS, target.getX(), 1e-9);
        assertEquals(ENGINEER_TARGET_SIDE_INSET_METERS, target.getY(), 1e-9);
    }

    @Test
    void inscribedCircleCornerMirrorsCorrectlyForRedAlliance() {
        Translation2d target = FieldMath.getEngineerTargetInAllianceZone(
                Alliance.Red,
                ENGINEER_TARGET_JOYSTICK_LATERAL_LIMIT,
                -ENGINEER_TARGET_JOYSTICK_FORWARD_LIMIT);

        assertEquals(FIELD_LENGTH_METERS - ENGINEER_TARGET_FORWARD_LIMIT_METERS, target.getX(), 1e-9);
        assertEquals(FIELD_WIDTH_METERS - ENGINEER_TARGET_SIDE_INSET_METERS, target.getY(), 1e-9);
    }

    @Test
    void zeroStickKeepsHubTargetCentered() {
        Translation2d hubTarget = FieldMath.getHubTarget(Alliance.Blue);
        Translation2d adjustedTarget = FieldMath.getAdjustedHubTarget(
                Alliance.Blue,
                new Translation2d(0.0, hubTarget.getY()),
                0.0,
                0.0);

        assertEquals(hubTarget.getX(), adjustedTarget.getX(), 1e-9);
        assertEquals(hubTarget.getY(), adjustedTarget.getY(), 1e-9);
    }

    @Test
    void forwardStickMovesHubTargetFartherAlongShotLine() {
        Translation2d hubTarget = FieldMath.getHubTarget(Alliance.Blue);
        Translation2d adjustedTarget = FieldMath.getAdjustedHubTarget(
                Alliance.Blue,
                new Translation2d(0.0, hubTarget.getY()),
                0.0,
                -1.0);

        assertEquals(hubTarget.getX() + HUB_TARGET_ENGINEER_ADJUSTMENT_LIMIT_METERS, adjustedTarget.getX(), 1e-9);
        assertEquals(hubTarget.getY(), adjustedTarget.getY(), 1e-9);
    }

    @Test
    void rightStickMovesHubTargetToShotRight() {
        Translation2d hubTarget = FieldMath.getHubTarget(Alliance.Blue);
        Translation2d adjustedTarget = FieldMath.getAdjustedHubTarget(
                Alliance.Blue,
                new Translation2d(0.0, hubTarget.getY()),
                1.0,
                0.0);

        assertEquals(hubTarget.getX(), adjustedTarget.getX(), 1e-9);
        assertEquals(hubTarget.getY() - HUB_TARGET_ENGINEER_ADJUSTMENT_LIMIT_METERS, adjustedTarget.getY(), 1e-9);
    }

    @Test
    void diagonalHubAdjustmentClampsToThreeFeet() {
        Translation2d hubTarget = FieldMath.getHubTarget(Alliance.Blue);
        Translation2d adjustedTarget = FieldMath.getAdjustedHubTarget(
                Alliance.Blue,
                new Translation2d(0.0, hubTarget.getY()),
                1.0,
                -1.0);

        assertEquals(HUB_TARGET_ENGINEER_ADJUSTMENT_LIMIT_METERS, adjustedTarget.getDistance(hubTarget), 1e-9);
    }
}
