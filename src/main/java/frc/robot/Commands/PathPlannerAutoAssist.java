package frc.robot.Commands;

import static edu.wpi.first.units.Units.Degrees;

import java.util.Objects;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.fieldmath.FieldMath;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SparkAnglePositionSubsystem;
import com.pathplanner.lib.util.DriveFeedforwards;

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
    private final PoseEstimatorSubsystem.PredictedFusedState futureState =
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
    private Translation2d lastFieldRelativeDriveDirection = new Translation2d();

    public PathPlannerAutoAssist(
            PoseEstimatorSubsystem poseEstimator,
            SparkAnglePositionSubsystem horizontalAim,
            SparkAnglePositionSubsystem verticalAim,
            ShooterSubsystem shooter) {
        this.poseEstimator = Objects.requireNonNull(poseEstimator, "poseEstimator must not be null");
        this.horizontalAim = Objects.requireNonNull(horizontalAim, "horizontalAim must not be null");
        this.verticalAim = Objects.requireNonNull(verticalAim, "verticalAim must not be null");
        this.shooter = Objects.requireNonNull(shooter, "shooter must not be null");
        facingAngleDrive.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void reset() {
        shotMode = ShotMode.OFF;
        intakeEnabled = false;
        clearShotOutputs();
        lastFieldRelativeDriveDirection = new Translation2d();
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
            ChassisSpeeds robotRelativeSpeeds,
            DriveFeedforwards feedforwards,
            double azimuthTrimDegrees) {
        Objects.requireNonNull(drivetrain, "drivetrain must not be null");
        Objects.requireNonNull(robotRelativeSpeeds, "robotRelativeSpeeds must not be null");
        Objects.requireNonNull(feedforwards, "feedforwards must not be null");

        updateLastDriveDirection(robotRelativeSpeeds);
        if (shotMode == ShotMode.OFF) {
            clearShotOutputs();
            return buildPathFollowingRequest(robotRelativeSpeeds, feedforwards);
        }
        if (!poseEstimator.getPredictedFusedState(
                ShooterConstants.COMMANDED_SHOOTER_LOOKAHEAD_SECONDS,
                futureState)) {
            clearShotOutputs();
            return buildPathFollowingRequest(robotRelativeSpeeds, feedforwards);
        }

        Pose2d futurePose = new Pose2d(
                futureState.xMeters,
                futureState.yMeters,
                Rotation2d.fromRadians(futureState.headingRadians));
        Translation2d target = getTarget(futurePose);
        Rotation2d preferredRobotHeading = getPreferredRobotHeading(futurePose.getRotation());

        if (!updateShotSolution(target, preferredRobotHeading, azimuthTrimDegrees)) {
            clearShotOutputs();
            return buildPathFollowingRequest(robotRelativeSpeeds, feedforwards);
        }

        return facingAngleDrive
                .withVelocityX(robotRelativeSpeeds.vxMetersPerSecond)
                .withVelocityY(robotRelativeSpeeds.vyMetersPerSecond)
                .withTargetDirection(Rotation2d.fromDegrees(movingShotSolution.getRobotHeadingDegrees()))
                .withTargetRateFeedforward(0.0)
                .withDeadband(0.0)
                .withRotationalDeadband(0.0);
    }

    private void updateLastDriveDirection(ChassisSpeeds robotRelativeSpeeds) {
        Pose2d currentPose = poseEstimator.getFusedPoseSupplier().get();
        if (currentPose == null) {
            return;
        }

        lastFieldRelativeDriveDirection = FieldMath.updateLastDriveDirection(
                currentPose,
                robotRelativeSpeeds,
                lastFieldRelativeDriveDirection);
    }

    private SwerveRequest buildPathFollowingRequest(
            ChassisSpeeds robotRelativeSpeeds,
            DriveFeedforwards feedforwards) {
        // PathPlanner's documented integration only requires driving the robot-relative
        // chassis speeds. Dropping module force feedforwards here keeps the CTRE bridge
        // aligned with the official "driveRobotRelative(speeds)" pattern.
        return pathFollowingRequest
                .withSpeeds(robotRelativeSpeeds)
                .withCenterOfRotation(new Translation2d())
                .withDesaturateWheelSpeeds(true);
    }

    private Translation2d getTarget(Pose2d futurePose) {
        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        return switch (shotMode) {
            case SCORE_IN_HUB -> FieldMath.getHubTarget(alliance);
            case SNOWBLOW_TO_ALLIANCE ->
                    FieldMath.getSnowblowTarget(futurePose, lastFieldRelativeDriveDirection, alliance);
            case OFF -> futurePose.getTranslation();
        };
    }

    private Rotation2d getPreferredRobotHeading(Rotation2d fallbackHeading) {
        if (lastFieldRelativeDriveDirection.getNorm() > 1e-9) {
            return lastFieldRelativeDriveDirection.getAngle();
        }
        return fallbackHeading;
    }

    private boolean updateShotSolution(
            Translation2d target,
            Rotation2d preferredRobotHeading,
            double azimuthTrimDegrees) {
        double minimumHoodAngleDegrees = verticalAim.getMinimumAngle().in(Degrees);
        double maximumHoodAngleDegrees = verticalAim.getMaximumAngle().in(Degrees);
        boolean hasIdealSolution = MovingShotMath.solveIdealMovingShotWithUpperHoodFallback(
                verticalAim,
                ShooterConstants.COMMANDED_MOVING_SHOT_HOOD_SEARCH_STEP_DEGREES,
                futureState.xMeters,
                futureState.yMeters,
                futureState.headingRadians,
                futureState.vxMetersPerSecond,
                futureState.vyMetersPerSecond,
                target.getX(),
                target.getY(),
                getTargetElevationInches(),
                ShooterConstants.COMMANDED_MAXIMUM_SHOOTING_HEIGHT_INCHES,
                preferredRobotHeading.getRadians(),
                horizontalAim.getMinimumAngle().in(Degrees),
                horizontalAim.getMaximumAngle().in(Degrees),
                idealMovingShotSolution);
        if (!hasIdealSolution) {
            return false;
        }

        double idealFlywheelCommandIps = idealMovingShotSolution.getFlywheelCommandIps();
        double predictedFlywheelCommandIps = MovingShotMath.predictFlywheelSpeedIps(
                shooter.getMainFlywheelSpeedIPS(),
                idealFlywheelCommandIps);
        BallTrajectoryLookup.FixedFlywheelShotStatus fixedFlywheelStatus =
                BallTrajectoryLookup.solveMovingShotForFlywheelCommand(
                        minimumHoodAngleDegrees,
                        maximumHoodAngleDegrees,
                        ShooterConstants.COMMANDED_MOVING_SHOT_FIXED_FLYWHEEL_HOOD_SEARCH_STEP_DEGREES,
                        true,
                        futureState.xMeters,
                        futureState.yMeters,
                        futureState.headingRadians,
                        futureState.vxMetersPerSecond,
                        futureState.vyMetersPerSecond,
                        target.getX(),
                        target.getY(),
                        getTargetElevationInches(),
                        ShooterConstants.COMMANDED_MAXIMUM_SHOOTING_HEIGHT_INCHES,
                        preferredRobotHeading.getRadians(),
                        horizontalAim.getMinimumAngle().in(Degrees),
                        horizontalAim.getMaximumAngle().in(Degrees),
                        predictedFlywheelCommandIps,
                        movingShotSolution);
        if (fixedFlywheelStatus == BallTrajectoryLookup.FixedFlywheelShotStatus.NO_SOLUTION) {
            return false;
        }

        commandedHoodAngleDegrees = switch (fixedFlywheelStatus) {
            case TOO_SLOW -> minimumHoodAngleDegrees;
            case TOO_FAST -> maximumHoodAngleDegrees;
            case VALID -> movingShotSolution.getHoodAngleDegrees();
            case NO_SOLUTION -> ShooterConstants.COMMANDED_MAXIMUM_ALLOWED_HOOD_ANGLE_DEGREES;
        };
        commandedTurretDeltaDegrees = clampTurretDeltaDegrees(
                movingShotSolution.getTurretDeltaDegrees() + azimuthTrimDegrees);
        commandedFlywheelIps = idealFlywheelCommandIps;
        shotOutputsActive = true;
        return true;
    }

    private double getTargetElevationInches() {
        return shotMode == ShotMode.SCORE_IN_HUB
                ? ShooterConstants.COMMANDED_SCORE_IN_HUB_TARGET_ELEVATION_INCHES
                : ShooterConstants.COMMANDED_SNOWBLOW_TARGET_ELEVATION_INCHES;
    }

    private double clampTurretDeltaDegrees(double turretDeltaDegrees) {
        return Math.max(
                horizontalAim.getMinimumAngle().in(Degrees),
                Math.min(horizontalAim.getMaximumAngle().in(Degrees), turretDeltaDegrees));
    }

    private void clearShotOutputs() {
        shotOutputsActive = false;
        commandedTurretDeltaDegrees = 0.0;
        commandedHoodAngleDegrees = ShooterConstants.COMMANDED_MAXIMUM_ALLOWED_HOOD_ANGLE_DEGREES;
        commandedFlywheelIps = 0.0;
    }
}
