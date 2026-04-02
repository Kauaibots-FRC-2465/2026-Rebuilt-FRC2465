package frc.robot.fieldmath;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Commands.ShooterConstants;

/**
 * Field-geometry helpers for alliance-relative targeting logic.
 */
public final class FieldMath {
    private static final double FIELD_LENGTH_METERS = Meters.convertFrom(651.2, Inches);
    private static final double FIELD_WIDTH_METERS = Meters.convertFrom(317.7, Inches);
    private static final double TARGET_LINE_OFFSET_METERS = Meters.convertFrom(3.0, Feet);
    private static final double TARGET_SIDE_INSET_METERS = Meters.convertFrom(2.0, Feet);
    private static final double DIRECTION_SPEED_THRESHOLD_METERS_PER_SECOND = 0.05;
    private static final double DIRECTION_DISTANCE_THRESHOLD_METERS = 0.001;
    private static final double EPSILON = 1e-9;

    private FieldMath() {
    }

    /**
     * Returns the latest meaningful field-relative drive direction.
     *
     * <p>If the requested field-relative velocity is faster than the threshold,
     * that direction is returned. Otherwise the previous direction is preserved.
     */
    public static Translation2d updateLastDriveDirection(
            Translation2d fieldRelativeVelocity,
            Translation2d previousDirection) {
        double speed = fieldRelativeVelocity.getNorm();
        if (speed > DIRECTION_SPEED_THRESHOLD_METERS_PER_SECOND) {
            return fieldRelativeVelocity.div(speed);
        }

        return previousDirection;
    }

    /**
     * Returns the latest meaningful field-relative drive direction from
     * consecutive field positions.
     */
    public static Translation2d updateLastDriveDirection(
            Translation2d currentFieldTranslation,
            Translation2d previousFieldTranslation,
            Translation2d previousDirection) {
        Translation2d fieldRelativeDisplacement = currentFieldTranslation.minus(previousFieldTranslation);
        double distance = fieldRelativeDisplacement.getNorm();
        if (distance > DIRECTION_DISTANCE_THRESHOLD_METERS) {
            return fieldRelativeDisplacement.div(distance);
        }

        return previousDirection;
    }

    /**
     * Returns the latest meaningful field-relative drive direction from
     * robot-relative chassis speeds.
     */
    public static Translation2d updateLastDriveDirection(
            Pose2d robotPose,
            ChassisSpeeds robotRelativeSpeeds,
            Translation2d previousDirection) {
        Translation2d fieldRelativeVelocity = new Translation2d(
                robotRelativeSpeeds.vxMetersPerSecond,
                robotRelativeSpeeds.vyMetersPerSecond).rotateBy(robotPose.getRotation());
        return updateLastDriveDirection(fieldRelativeVelocity, previousDirection);
    }

    /**
     * Returns the alliance-relative snowblow target point on the field.
     */
    public static Translation2d getSnowblowTarget(
            Pose2d robotPose,
            Translation2d lastFieldRelativeDriveDirection,
            Alliance alliance) {
        Translation2d driveDirection =
                normalizeOrFallback(lastFieldRelativeDriveDirection, getDefaultAllianceDirection(alliance));

        double targetLineX = alliance == Alliance.Red
                ? FIELD_LENGTH_METERS - TARGET_LINE_OFFSET_METERS
                : TARGET_LINE_OFFSET_METERS;

        double targetY;
        if (Math.abs(driveDirection.getX()) > EPSILON) {
            double t = (targetLineX - robotPose.getX()) / driveDirection.getX();
            targetY = robotPose.getY() + driveDirection.getY() * t;
            targetY = clamp(targetY, TARGET_SIDE_INSET_METERS, FIELD_WIDTH_METERS - TARGET_SIDE_INSET_METERS);
        } else {
            targetY = getParallelTargetY(driveDirection);
        }

        Translation2d target = new Translation2d(targetLineX, targetY);
        if (isDrivingAwayFromAllianceWall(driveDirection, alliance)) {
            target = mirrorTargetAcrossFieldCenterline(target);
        }
        return target;
    }

    /**
     * Returns a fixed hub target expressed by alliance-relative field distances.
     */
    public static Translation2d getHubTarget(Alliance alliance) {
        return getAllianceRelativeFieldPoint(
                Meters.convertFrom(
                        ShooterConstants.COMMANDED_SCORE_IN_HUB_ALLIANCE_WALL_TO_HUB_CENTER_INCHES,
                        Inches),
                Meters.convertFrom(
                        ShooterConstants.COMMANDED_SCORE_IN_HUB_RIGHT_FIELD_WALL_TO_HUB_CENTER_INCHES,
                        Inches),
                alliance);
    }

    /**
     * Converts alliance-relative wall distances into the WPILib blue-origin field coordinate system.
     */
    public static Translation2d getAllianceRelativeFieldPoint(
            double allianceWallDistanceMeters,
            double rightFieldWallDistanceMeters,
            Alliance alliance) {
        double xMeters = alliance == Alliance.Red
                ? FIELD_LENGTH_METERS - allianceWallDistanceMeters
                : allianceWallDistanceMeters;
        double yMeters = alliance == Alliance.Red
                ? FIELD_WIDTH_METERS - rightFieldWallDistanceMeters
                : rightFieldWallDistanceMeters;
        return new Translation2d(xMeters, yMeters);
    }

    private static boolean isDrivingAwayFromAllianceWall(Translation2d driveDirection, Alliance alliance) {
        return alliance == Alliance.Red
                ? driveDirection.getX() < -EPSILON
                : driveDirection.getX() > EPSILON;
    }

    private static Translation2d mirrorTargetAcrossFieldCenterline(Translation2d target) {
        return new Translation2d(target.getX(), FIELD_WIDTH_METERS - target.getY());
    }

    private static double getParallelTargetY(Translation2d driveDirection) {
        if (driveDirection.getY() > EPSILON) {
            return FIELD_WIDTH_METERS - TARGET_SIDE_INSET_METERS;
        }
        if (driveDirection.getY() < -EPSILON) {
            return TARGET_SIDE_INSET_METERS;
        }

        return FIELD_WIDTH_METERS / 2.0;
    }

    private static Translation2d getDefaultAllianceDirection(Alliance alliance) {
        return alliance == Alliance.Red
                ? new Translation2d(1.0, 0.0)
                : new Translation2d(-1.0, 0.0);
    }

    private static Translation2d normalizeOrFallback(Translation2d vector, Translation2d fallback) {
        double norm = vector.getNorm();
        if (norm <= EPSILON) {
            return fallback;
        }
        return vector.div(norm);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
