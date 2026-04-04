package frc.robot.Commands;

import java.util.Objects;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.SparkAnglePositionSubsystem;

final class MovingShotMath {
    private MovingShotMath() {
    }

    static boolean predictFutureStateFromCommand(
            PoseEstimatorSubsystem poseEstimator,
            Rotation2d driverPerspectiveForward,
            double commandedVelocityXMetersPerSecond,
            double commandedVelocityYMetersPerSecond,
            double lookaheadSeconds,
            PoseEstimatorSubsystem.PredictedFusedState out) {
        Objects.requireNonNull(poseEstimator, "poseEstimator must not be null");
        Objects.requireNonNull(driverPerspectiveForward, "driverPerspectiveForward must not be null");
        Objects.requireNonNull(out, "out must not be null");

        if (!Double.isFinite(lookaheadSeconds) || lookaheadSeconds < 0.0) {
            out.setInvalid();
            return false;
        }

        Pose2d currentFusedPose = poseEstimator.getFusedPoseSupplier().get();
        if (currentFusedPose == null) {
            out.setInvalid();
            return false;
        }

        Translation2d fieldRelativeVelocity = new Translation2d(
                commandedVelocityXMetersPerSecond,
                commandedVelocityYMetersPerSecond).rotateBy(driverPerspectiveForward);
        // These aim commands replace driver yaw-rate control with a heading target,
        // so predict translation from the commanded field velocity but keep the
        // current fused heading instead of integrating the raw stick rotational rate.
        double omegaRadiansPerSecond = 0.0;
        Rotation2d futureHeading = currentFusedPose.getRotation();

        out.set(
                currentFusedPose.getX() + fieldRelativeVelocity.getX() * lookaheadSeconds,
                currentFusedPose.getY() + fieldRelativeVelocity.getY() * lookaheadSeconds,
                futureHeading.getRadians(),
                fieldRelativeVelocity.getX(),
                fieldRelativeVelocity.getY(),
                omegaRadiansPerSecond,
                RobotController.getFPGATime() + Math.round(lookaheadSeconds * 1_000_000.0));
        return true;
    }

    static double predictFlywheelSpeedIps(double currentFlywheelSpeedIps, double targetFlywheelSpeedIps) {
        return predictFlywheelSpeedIps(
                currentFlywheelSpeedIps,
                targetFlywheelSpeedIps,
                ShooterConstants.COMMANDED_MOVING_SHOT_FLYWHEEL_PREDICTION_SECONDS,
                ShooterConstants.COMMANDED_MOVING_SHOT_FLYWHEEL_SPIN_UP_RATE_IPS_PER_SECOND,
                ShooterConstants.COMMANDED_MOVING_SHOT_FLYWHEEL_SPIN_DOWN_RATE_IPS_PER_SECOND);
    }

    static double predictFlywheelSpeedIps(
            double currentFlywheelSpeedIps,
            double targetFlywheelSpeedIps,
            double dtSeconds,
            double spinUpRateIpsPerSecond,
            double spinDownRateIpsPerSecond) {
        if (!Double.isFinite(currentFlywheelSpeedIps)
                || !Double.isFinite(targetFlywheelSpeedIps)
                || !Double.isFinite(dtSeconds)
                || dtSeconds < 0.0) {
            return Double.NaN;
        }

        double deltaIps = targetFlywheelSpeedIps - currentFlywheelSpeedIps;
        if (Math.abs(deltaIps) <= 1e-9) {
            return targetFlywheelSpeedIps;
        }

        double rateLimitIpsPerSecond = deltaIps > 0.0
                ? spinUpRateIpsPerSecond
                : spinDownRateIpsPerSecond;
        if (!Double.isFinite(rateLimitIpsPerSecond) || rateLimitIpsPerSecond < 0.0) {
            return Double.NaN;
        }

        double maxDeltaIps = rateLimitIpsPerSecond * dtSeconds;
        return currentFlywheelSpeedIps
                + MathUtil.clamp(deltaIps, -maxDeltaIps, maxDeltaIps);
    }

    static double getIdealMaximumHoodAngleDegrees(SparkAnglePositionSubsystem verticalAim) {
        Objects.requireNonNull(verticalAim, "verticalAim must not be null");
        double minimumHoodAngleDegrees = verticalAim.getMinimumAngle().in(edu.wpi.first.units.Units.Degrees);
        double maximumHoodAngleDegrees = verticalAim.getMaximumAngle().in(edu.wpi.first.units.Units.Degrees);
        return Math.max(
                minimumHoodAngleDegrees,
                maximumHoodAngleDegrees - ShooterConstants.COMMANDED_MOVING_SHOT_HOOD_LIMIT_HEADROOM_DEGREES);
    }

    static boolean solveIdealMovingShotWithUpperHoodFallback(
            SparkAnglePositionSubsystem verticalAim,
            double hoodAngleStepDegrees,
            double futureRobotXMeters,
            double futureRobotYMeters,
            double futureRobotHeadingRadians,
            double robotFieldVxMetersPerSecond,
            double robotFieldVyMetersPerSecond,
            double targetXMeters,
            double targetYMeters,
            double targetElevationInches,
            double maximumBallZElevationInches,
            double preferredRobotHeadingRadians,
            double minTurretAngleDegrees,
            double maxTurretAngleDegrees,
            BallTrajectoryLookup.MovingShotSolution out) {
        Objects.requireNonNull(verticalAim, "verticalAim must not be null");
        Objects.requireNonNull(out, "out must not be null");

        double minimumHoodAngleDegrees = verticalAim.getMinimumAngle().in(edu.wpi.first.units.Units.Degrees);
        double maximumHoodAngleDegrees = verticalAim.getMaximumAngle().in(edu.wpi.first.units.Units.Degrees);
        double idealMaximumHoodAngleDegrees = getIdealMaximumHoodAngleDegrees(verticalAim);

        boolean hasPreferredSolution = BallTrajectoryLookup.solveMovingShot(
                minimumHoodAngleDegrees,
                idealMaximumHoodAngleDegrees,
                hoodAngleStepDegrees,
                true,
                futureRobotXMeters,
                futureRobotYMeters,
                futureRobotHeadingRadians,
                robotFieldVxMetersPerSecond,
                robotFieldVyMetersPerSecond,
                targetXMeters,
                targetYMeters,
                targetElevationInches,
                maximumBallZElevationInches,
                preferredRobotHeadingRadians,
                minTurretAngleDegrees,
                maxTurretAngleDegrees,
                out);
        if (hasPreferredSolution || idealMaximumHoodAngleDegrees >= maximumHoodAngleDegrees - 1e-9) {
            return hasPreferredSolution;
        }

        return BallTrajectoryLookup.solveMovingShot(
                idealMaximumHoodAngleDegrees,
                maximumHoodAngleDegrees,
                hoodAngleStepDegrees,
                false,
                futureRobotXMeters,
                futureRobotYMeters,
                futureRobotHeadingRadians,
                robotFieldVxMetersPerSecond,
                robotFieldVyMetersPerSecond,
                targetXMeters,
                targetYMeters,
                targetElevationInches,
                maximumBallZElevationInches,
                preferredRobotHeadingRadians,
                minTurretAngleDegrees,
                maxTurretAngleDegrees,
                out);
    }
}
