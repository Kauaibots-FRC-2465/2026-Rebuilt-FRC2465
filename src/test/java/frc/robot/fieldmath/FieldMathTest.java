package frc.robot.fieldmath;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
    void hubTargetMirrorsAcrossFieldForRedAlliance() {
        Translation2d blueHubTarget = FieldMath.getHubTarget(Alliance.Blue);
        Translation2d redHubTarget = FieldMath.getHubTarget(Alliance.Red);

        assertEquals(FIELD_LENGTH_METERS - blueHubTarget.getX(), redHubTarget.getX(), 1e-9);
        assertEquals(FIELD_WIDTH_METERS - blueHubTarget.getY(), redHubTarget.getY(), 1e-9);
    }

    @Test
    void snowblowTargetMirrorsForMirroredRobotPoseAndTravelDirection() {
        Pose2d blueRobotPose = new Pose2d(4.0, 2.0, Rotation2d.kZero);
        Translation2d blueTravelDirection = new Translation2d(-1.0, 0.25);
        Translation2d blueTarget = FieldMath.getSnowblowTarget(
                blueRobotPose,
                blueTravelDirection,
                Alliance.Blue);

        Pose2d redRobotPose = new Pose2d(
                FIELD_LENGTH_METERS - blueRobotPose.getX(),
                FIELD_WIDTH_METERS - blueRobotPose.getY(),
                Rotation2d.k180deg);
        Translation2d redTravelDirection = blueTravelDirection.times(-1.0);
        Translation2d redTarget = FieldMath.getSnowblowTarget(
                redRobotPose,
                redTravelDirection,
                Alliance.Red);

        assertEquals(FIELD_LENGTH_METERS - blueTarget.getX(), redTarget.getX(), 1e-9);
        assertEquals(FIELD_WIDTH_METERS - blueTarget.getY(), redTarget.getY(), 1e-9);
    }
}
