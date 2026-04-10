package frc.robot.Commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.Objects;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.fieldmath.FieldMath;
import frc.robot.fieldmath.TravelWindowDirectionTracker;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SparkAnglePositionSubsystem;
import com.pathplanner.lib.util.DriveFeedforwards;

/**
 * Autonomous shot-assist state that keeps shot geometry in the absolute WPILib field frame
 * while accepting PathPlanner robot-relative chassis speeds at the drive boundary.
 */
public class PathPlannerAutoAssist {
    public enum ShotMode {
        OFF,
        SCORE_IN_HUB,
        SNOWBLOW_TO_ALLIANCE
    }

    public static final double AUTO_INTAKE_DEPLOY_ANGLE_DEGREES = 109.0;
    public static final double AUTO_INTAKE_DRIVE_SPEED_IPS = 300.0;

    private final PoseEstimatorSubsystem poseEstimator;
    private final SparkAnglePositionSubsystem horizontalAim;
    private final SparkAnglePositionSubsystem verticalAim;
    private final ShooterSubsystem shooter;
    private final Supplier<Translation2d> activePointTowardsTargetFldSupplier;
    private final PoseEstimatorSubsystem.PredictedFusedState futureStateFld =
            new PoseEstimatorSubsystem.PredictedFusedState();
    private final BallTrajectoryLookup.MovingShotSolution idealMovingShotSolution =
            new BallTrajectoryLookup.MovingShotSolution();
    private final BallTrajectoryLookup.MovingShotSolution movingShotSolution =
            new BallTrajectoryLookup.MovingShotSolution();
    private final SwerveRequest.ApplyRobotSpeeds pathFollowingRequest =
            new SwerveRequest.ApplyRobotSpeeds()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.Position);
    private final SwerveRequest.RobotCentricFacingAngle facingAngleDrive =
            new SwerveRequest.RobotCentricFacingAngle()
                    .withHeadingPID(5.0, 0.0, 0.0)
                    .withDriveRequestType(DriveRequestType.Velocity)
                    .withSteerRequestType(SteerRequestType.Position)
                    .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance);

    private ShotMode shotMode = ShotMode.OFF;
    private boolean intakeEnabled = false;
    private boolean shotOutputsActive = false;
    private double commandedTurretDeltaDegrees = 0.0;
    private double commandedHoodAngleDegrees =
            ShooterConstants.COMMANDED_MAXIMUM_ALLOWED_HOOD_ANGLE_DEGREES;
    private double commandedFlywheelIps = 0.0;
    private final TravelWindowDirectionTracker preferredHeadingTracker =
            new TravelWindowDirectionTracker(
                    Meters.convertFrom(
                            ShooterConstants.COMMANDED_PREFERRED_HEADING_TRAVEL_WINDOW_INCHES,
                            Inches),
                    Meters.convertFrom(
                            ShooterConstants.COMMANDED_PREFERRED_HEADING_MIN_SAMPLE_SPACING_INCHES,
                            Inches));
    private Translation2d lastDriveDirectionFld = new Translation2d();

    public PathPlannerAutoAssist(
            PoseEstimatorSubsystem poseEstimator,
            SparkAnglePositionSubsystem horizontalAim,
            SparkAnglePositionSubsystem verticalAim,
            ShooterSubsystem shooter,
            Supplier<Translation2d> activePointTowardsTargetFldSupplier) {
        this.poseEstimator = Objects.requireNonNull(poseEstimator, "poseEstimator must not be null");
        this.horizontalAim = Objects.requireNonNull(horizontalAim, "horizontalAim must not be null");
        this.verticalAim = Objects.requireNonNull(verticalAim, "verticalAim must not be null");
        this.shooter = Objects.requireNonNull(shooter, "shooter must not be null");
        this.activePointTowardsTargetFldSupplier =
                Objects.requireNonNull(
                        activePointTowardsTargetFldSupplier,
                        "activePointTowardsTargetFldSupplier must not be null");
        facingAngleDrive.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void reset() {
        shotMode = ShotMode.OFF;
        intakeEnabled = false;
        clearShotOutputs();
        preferredHeadingTracker.reset();
        lastDriveDirectionFld = new Translation2d();
    }

    public void enableScoreInHub() {
        shotMode = ShotMode.SCORE_IN_HUB;
    }

    public void enableSnowblowToAlliance() {
        shotMode = ShotMode.SNOWBLOW_TO_ALLIANCE;
    }

    public void disableShotAssist() {
        shotMode = ShotMode.OFF;
        clearShotOutputs();
    }

    public void activateIntake() {
        intakeEnabled = true;
    }

    public void deactivateIntake() {
        intakeEnabled = false;
    }

    public boolean isIntakeEnabled() {
        return intakeEnabled;
    }

    public boolean hasShotOutputsActive() {
        return shotOutputsActive;
    }

    public double getCommandedTurretDeltaDegrees() {
        return commandedTurretDeltaDegrees;
    }

    public double getCommandedHoodAngleDegrees() {
        return commandedHoodAngleDegrees;
    }

    public double getCommandedFlywheelIps() {
        return commandedFlywheelIps;
    }

    public SwerveRequest buildDriveRequest(
            CommandSwerveDrivetrain drivetrain,
            ChassisSpeeds robotRelativeSpeedsRbt,
            DriveFeedforwards feedforwards) {
        Objects.requireNonNull(drivetrain, "drivetrain must not be null");
        Objects.requireNonNull(robotRelativeSpeedsRbt, "robotRelativeSpeedsRbt must not be null");
        Objects.requireNonNull(feedforwards, "feedforwards must not be null");

        updateLastDriveDirectionFld();
        if (shotMode == ShotMode.OFF) {
            clearShotOutputs();
            return buildPathFollowingRequest(robotRelativeSpeedsRbt, feedforwards);
        }
        if (!poseEstimator.getPredictedFusedState(
                ShooterConstants.COMMANDED_SHOOTER_LOOKAHEAD_SECONDS,
                futureStateFld)) {
            applyNoSolutionShotOutputs(poseEstimator.getFusedPoseSupplier().get(), null);
            return buildPathFollowingRequest(robotRelativeSpeedsRbt, feedforwards);
        }

        Pose2d futurePoseFld = new Pose2d(
                futureStateFld.xMeters,
                futureStateFld.yMeters,
                Rotation2d.fromRadians(futureStateFld.headingRadians));
        Translation2d targetFld = getTargetFld(futurePoseFld);
        Rotation2d preferredRobotHeadingFld = getPreferredRobotHeadingFld(
                futurePoseFld,
                targetFld,
                futurePoseFld.getRotation());

        if (!updateShotSolution(targetFld, preferredRobotHeadingFld)) {
            applyNoSolutionShotOutputs(futurePoseFld, targetFld);
            return buildPathFollowingRequest(robotRelativeSpeedsRbt, feedforwards);
        }

        return facingAngleDrive
                .withVelocityX(robotRelativeSpeedsRbt.vxMetersPerSecond)
                .withVelocityY(robotRelativeSpeedsRbt.vyMetersPerSecond)
                .withTargetDirection(Rotation2d.fromDegrees(movingShotSolution.getRobotHeadingDegrees()))
                .withTargetRateFeedforward(0.0)
                .withDeadband(0.0)
                .withRotationalDeadband(0.0);
    }

    private void updateLastDriveDirectionFld() {
        Pose2d currentPoseFld = poseEstimator.getFusedPoseSupplier().get();
        if (currentPoseFld == null) {
            return;
        }

        lastDriveDirectionFld = preferredHeadingTracker.update(currentPoseFld.getTranslation());
    }

    private SwerveRequest buildPathFollowingRequest(
            ChassisSpeeds robotRelativeSpeedsRbt,
            DriveFeedforwards feedforwards) {
        // PathPlanner's documented integration only requires driving the robot-relative
        // chassis speeds. Dropping module force feedforwards here keeps the CTRE bridge
        // aligned with the official "driveRobotRelative(speeds)" pattern.
        return pathFollowingRequest
                .withSpeeds(robotRelativeSpeedsRbt)
                .withCenterOfRotation(new Translation2d())
                .withDesaturateWheelSpeeds(true);
    }

    private Translation2d getTargetFld(Pose2d futurePoseFld) {
        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        return switch (shotMode) {
            case SCORE_IN_HUB -> FieldMath.getHubTarget(alliance);
            case SNOWBLOW_TO_ALLIANCE -> getSnowblowTargetFld(futurePoseFld, alliance);
            case OFF -> futurePoseFld.getTranslation();
        };
    }

    private Translation2d getSnowblowTargetFld(Pose2d futurePoseFld, DriverStation.Alliance alliance) {
        Translation2d activePointTowardsTargetFld = activePointTowardsTargetFldSupplier.get();
        if (activePointTowardsTargetFld != null) {
            return activePointTowardsTargetFld;
        }

        return FieldMath.getSnowblowTarget(futurePoseFld, lastDriveDirectionFld, alliance);
    }

    private Rotation2d getPreferredRobotHeadingFld(
            Pose2d futurePoseFld,
            Translation2d targetFld,
            Rotation2d fallbackHeadingFld) {
        if (shotMode == ShotMode.SCORE_IN_HUB) {
            return MovingShotMath.getHeadingTowardTarget(
                    targetFld.getX() - futurePoseFld.getX(),
                    targetFld.getY() - futurePoseFld.getY(),
                    fallbackHeadingFld);
        }
        if (lastDriveDirectionFld.getNorm() > 1e-9) {
            return lastDriveDirectionFld.getAngle();
        }
        return fallbackHeadingFld;
    }

    private boolean updateShotSolution(
            Translation2d targetFld,
            Rotation2d preferredRobotHeadingFld) {
        double minimumHoodAngleDegrees = verticalAim.getMinimumAngle().in(Degrees);
        double maximumHoodAngleDegrees = verticalAim.getMaximumAngle().in(Degrees);
        BallTrajectoryLookup.FixedFlywheelShotStatus fixedFlywheelStatus =
                MovingShotMath.solveCommandedMovingShot(
                        verticalAim,
                        ShooterConstants.COMMANDED_MOVING_SHOT_HOOD_SEARCH_STEP_DEGREES,
                        ShooterConstants.COMMANDED_MOVING_SHOT_FIXED_FLYWHEEL_HOOD_SEARCH_STEP_DEGREES,
                        futureStateFld,
                        targetFld,
                        getTargetElevationInches(),
                        ShooterConstants.COMMANDED_MAXIMUM_SHOOTING_HEIGHT_INCHES,
                        preferredRobotHeadingFld.getRadians(),
                        horizontalAim.getMinimumAngle().in(Degrees),
                        horizontalAim.getMaximumAngle().in(Degrees),
                        shooter.getMainFlywheelSpeedIPS(),
                        idealMovingShotSolution,
                        movingShotSolution);
        if (fixedFlywheelStatus == BallTrajectoryLookup.FixedFlywheelShotStatus.NO_SOLUTION) {
            return false;
        }

        double idealFlywheelCommandIps = idealMovingShotSolution.getFlywheelCommandIps();
        commandedHoodAngleDegrees = MovingShotMath.getCommandedHoodAngleDegrees(
                fixedFlywheelStatus,
                minimumHoodAngleDegrees,
                maximumHoodAngleDegrees,
                movingShotSolution);
        commandedTurretDeltaDegrees = MovingShotMath.clampTurretDeltaDegrees(
                movingShotSolution.getTurretDeltaDegrees(),
                horizontalAim.getMinimumAngle().in(Degrees),
                horizontalAim.getMaximumAngle().in(Degrees));
        commandedFlywheelIps = idealFlywheelCommandIps;
        shotOutputsActive = true;
        return true;
    }

    private double getTargetElevationInches() {
        return shotMode == ShotMode.SCORE_IN_HUB
                ? ShooterConstants.COMMANDED_SCORE_IN_HUB_TARGET_ELEVATION_INCHES
                : ShooterConstants.COMMANDED_SNOWBLOW_TARGET_ELEVATION_INCHES;
    }

    private void applyNoSolutionShotOutputs(Pose2d aimingPoseFld, Translation2d targetFld) {
        if (shotMode != ShotMode.SCORE_IN_HUB) {
            clearShotOutputs();
            return;
        }

        Translation2d fallbackTargetFld = targetFld;
        if (fallbackTargetFld == null) {
            Pose2d targetPoseFld = aimingPoseFld != null ? aimingPoseFld : new Pose2d();
            fallbackTargetFld = getTargetFld(targetPoseFld);
        }

        double fallbackFlywheelIps =
                ShortRangeHubFlywheelLookup.getFallbackManifoldFlywheelCommandIps(
                        getFallbackHubDistanceInches(aimingPoseFld, fallbackTargetFld));
        commandedTurretDeltaDegrees = 0.0;
        commandedHoodAngleDegrees = ShooterConstants.COMMANDED_MAXIMUM_ALLOWED_HOOD_ANGLE_DEGREES;
        commandedFlywheelIps = Double.isFinite(fallbackFlywheelIps) ? fallbackFlywheelIps : 0.0;
        shotOutputsActive = commandedFlywheelIps > 0.0;
    }

    private double getFallbackHubDistanceInches(Pose2d aimingPoseFld, Translation2d targetFld) {
        if (aimingPoseFld != null && targetFld != null) {
            return Inches.convertFrom(targetFld.getDistance(aimingPoseFld.getTranslation()), Meters);
        }
        return 0.5 * (
                ShooterConstants.DATA_COLLECTION_SHORT_RANGE_MIN_DISTANCE_INCHES
                        + ShooterConstants.DATA_COLLECTION_SHORT_RANGE_EMPIRICAL_MAX_DISTANCE_INCHES);
    }

    private void clearShotOutputs() {
        shotOutputsActive = false;
        commandedTurretDeltaDegrees = 0.0;
        commandedHoodAngleDegrees = ShooterConstants.COMMANDED_MAXIMUM_ALLOWED_HOOD_ANGLE_DEGREES;
        commandedFlywheelIps = 0.0;
    }
}
