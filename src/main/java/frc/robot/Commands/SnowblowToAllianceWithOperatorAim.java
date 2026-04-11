package frc.robot.Commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.Objects;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.fieldmath.TravelWindowDirectionTracker;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SparkAnglePositionSubsystem;

/**
 * Snowblow variant that uses an operator-selected field target inside the alliance zone.
 *
 * <p>Frame suffixes used in this command:
 * Fld = absolute WPILib field frame and the default for internal geometry/solver math.
 * Drv = driver-perspective field-centric request frame at the Phoenix request boundary.
 * Rbt = robot/body frame.
 *
 * <p>Unit conventions:
 * Translation2d and Pose2d values use WPILib meters/radians defaults.
 * Primitive suffixes spell units explicitly: Degrees, Radians, Inches, Meters, Ips.
 */
public class SnowblowToAllianceWithOperatorAim extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final PoseEstimatorSubsystem poseEstimator;
    private final SparkAnglePositionSubsystem horizontalAim;
    private final SparkAnglePositionSubsystem verticalAim;
    private final ShooterSubsystem shooter;
    private final Supplier<SwerveRequest> driveRequestSupplier;
    private final Supplier<Translation2d> targetTranslationFldMetersSupplier;
    private final SwerveRequest.FieldCentricFacingAngle facingAngleDrive =
            new SwerveRequest.FieldCentricFacingAngle();
    private final PoseEstimatorSubsystem.PredictedFusedState futureStateFldMetersRadians =
            new PoseEstimatorSubsystem.PredictedFusedState();
    private final BallTrajectoryLookup.MovingShotSolution idealMovingShotSolution =
            new BallTrajectoryLookup.MovingShotSolution();
    private final BallTrajectoryLookup.MovingShotSolution movingShotSolution =
            new BallTrajectoryLookup.MovingShotSolution();
    private Rotation2d lastValidRobotHeadingTargetFldRadians = new Rotation2d();
    private double lastValidTurretDeltaDegrees = 0.0;
    private double lastValidHoodAngleDegrees = ShooterConstants.COMMANDED_MAXIMUM_ALLOWED_HOOD_ANGLE_DEGREES;
    private double lastValidFlywheelCommandIps = 0.0;
    private double lastCommandedFlywheelSetpointIps = 0.0;
    private boolean hasLatchedShotCommand = false;
    private final TravelWindowDirectionTracker preferredHeadingTracker =
            new TravelWindowDirectionTracker(
                    Meters.convertFrom(
                            ShooterConstants.COMMANDED_PREFERRED_HEADING_TRAVEL_WINDOW_INCHES,
                            Inches),
                    Meters.convertFrom(
                            ShooterConstants.COMMANDED_PREFERRED_HEADING_MIN_SAMPLE_SPACING_INCHES,
                            Inches));
    private Translation2d lastTravelVectorFldMeters = new Translation2d();

    public SnowblowToAllianceWithOperatorAim(
            CommandSwerveDrivetrain drivetrain,
            PoseEstimatorSubsystem poseEstimator,
            SparkAnglePositionSubsystem horizontalAim,
            SparkAnglePositionSubsystem verticalAim,
            ShooterSubsystem shooter,
            Supplier<SwerveRequest> driveRequestSupplier,
            Supplier<Translation2d> targetTranslationFldMetersSupplier) {
        this.drivetrain = Objects.requireNonNull(drivetrain, "drivetrain must not be null");
        this.poseEstimator = Objects.requireNonNull(poseEstimator, "poseEstimator must not be null");
        this.horizontalAim = Objects.requireNonNull(horizontalAim, "horizontalAim must not be null");
        this.verticalAim = Objects.requireNonNull(verticalAim, "verticalAim must not be null");
        this.shooter = Objects.requireNonNull(shooter, "shooter must not be null");
        this.driveRequestSupplier =
                Objects.requireNonNull(driveRequestSupplier, "driveRequestSupplier must not be null");
        this.targetTranslationFldMetersSupplier =
                Objects.requireNonNull(targetTranslationFldMetersSupplier, "targetSupplier must not be null");
        facingAngleDrive.withHeadingPID(5.0, 0.0, 0.0);
        facingAngleDrive.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(this.drivetrain, horizontalAim, verticalAim, shooter);
    }

    @Override
    public void initialize() {
        preferredHeadingTracker.reset();
        lastTravelVectorFldMeters = new Translation2d();
        hasLatchedShotCommand = false;
        lastValidRobotHeadingTargetFldRadians = new Rotation2d();
        lastValidTurretDeltaDegrees = 0.0;
        lastValidHoodAngleDegrees = verticalAim.getAngle().in(Degrees);
        lastValidFlywheelCommandIps = 0.0;
        lastCommandedFlywheelSetpointIps = shooter.getMainFlywheelSpeedIPS();
    }

    @Override
    public void execute() {
        SwerveRequest requestedDrive = driveRequestSupplier.get();
        if (!(requestedDrive instanceof SwerveRequest.FieldCentric fieldCentricRequestDrv)) {
            drivetrain.setControl(requestedDrive);
            return;
        }

        updateLastTravelVectorFldMeters();
        if (!poseEstimator.getPredictedFusedState(
                ShooterConstants.COMMANDED_SHOOTER_LOOKAHEAD_SECONDS,
                futureStateFldMetersRadians)) {
            if (hasLatchedShotCommand) {
                applyHeldShotCommand();
            }
            drivetrain.setControl(requestedDrive);
            return;
        }

        Pose2d futureRobotPoseFldMetersRadians = new Pose2d(
                futureStateFldMetersRadians.xMeters,
                futureStateFldMetersRadians.yMeters,
                Rotation2d.fromRadians(futureStateFldMetersRadians.headingRadians));
        Translation2d targetTranslationFldMeters = targetTranslationFldMetersSupplier.get();

        Rotation2d preferredRobotHeadingFldRadians = getPreferredRobotHeadingFldRadians(
                futureRobotPoseFldMetersRadians.getTranslation(),
                targetTranslationFldMeters,
                futureRobotPoseFldMetersRadians.getRotation());
        Rotation2d robotHeadingTargetFldRadians = preferredRobotHeadingFldRadians;
        BallTrajectoryLookup.FixedFlywheelShotStatus fixedFlywheelStatus =
                updateShooterSolution(targetTranslationFldMeters, preferredRobotHeadingFldRadians);
        if (fixedFlywheelStatus != BallTrajectoryLookup.FixedFlywheelShotStatus.NO_SOLUTION) {
            robotHeadingTargetFldRadians = Rotation2d.fromDegrees(movingShotSolution.getRobotHeadingDegrees());
        } else {
            if (hasLatchedShotCommand) {
                applyHeldShotCommand();
                robotHeadingTargetFldRadians = lastValidRobotHeadingTargetFldRadians;
            }
        }
        Rotation2d robotHeadingTargetDrvRadians =
                robotHeadingTargetFldRadians.minus(drivetrain.getDriverPerspectiveForward());

        drivetrain.setControl(
                facingAngleDrive
                        .withVelocityX(fieldCentricRequestDrv.VelocityX)
                        .withVelocityY(fieldCentricRequestDrv.VelocityY)
                        .withTargetDirection(robotHeadingTargetDrvRadians)
                        .withDeadband(fieldCentricRequestDrv.Deadband)
                        .withRotationalDeadband(fieldCentricRequestDrv.RotationalDeadband)
                        .withCenterOfRotation(fieldCentricRequestDrv.CenterOfRotation)
                        .withDriveRequestType(fieldCentricRequestDrv.DriveRequestType)
                        .withSteerRequestType(fieldCentricRequestDrv.SteerRequestType)
                        .withDesaturateWheelSpeeds(fieldCentricRequestDrv.DesaturateWheelSpeeds)
                        .withForwardPerspective(fieldCentricRequestDrv.ForwardPerspective));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}

    private BallTrajectoryLookup.FixedFlywheelShotStatus updateShooterSolution(
            Translation2d targetTranslationFldMeters,
            Rotation2d preferredRobotHeadingFldRadians) {
        double minimumHoodAngleDegrees = verticalAim.getMinimumAngle().in(Degrees);
        double maximumHoodAngleDegrees = verticalAim.getMaximumAngle().in(Degrees);
        BallTrajectoryLookup.FixedFlywheelShotStatus fixedFlywheelStatus =
                MovingShotMath.solveCommandedMovingShot(
                        verticalAim,
                        ShooterConstants.COMMANDED_MOVING_SHOT_HOOD_SEARCH_STEP_DEGREES,
                        ShooterConstants.COMMANDED_MOVING_SHOT_FIXED_FLYWHEEL_HOOD_SEARCH_STEP_DEGREES,
                        futureStateFldMetersRadians,
                        targetTranslationFldMeters,
                        ShooterConstants.COMMANDED_SNOWBLOW_TARGET_ELEVATION_INCHES,
                        ShooterConstants.COMMANDED_MAXIMUM_SHOOTING_HEIGHT_INCHES,
                        preferredRobotHeadingFldRadians.getRadians(),
                        horizontalAim.getMinimumAngle().in(Degrees),
                        horizontalAim.getMaximumAngle().in(Degrees),
                        shooter.getMainFlywheelSpeedIPS(),
                        lastCommandedFlywheelSetpointIps,
                        idealMovingShotSolution,
                        movingShotSolution);
        if (fixedFlywheelStatus == BallTrajectoryLookup.FixedFlywheelShotStatus.NO_SOLUTION) {
            return fixedFlywheelStatus;
        }

        double idealFlywheelCommandIps = idealMovingShotSolution.getFlywheelCommandIps();

        double commandedHoodAngleDegrees = MovingShotMath.getCommandedHoodAngleDegrees(
                fixedFlywheelStatus,
                minimumHoodAngleDegrees,
                maximumHoodAngleDegrees,
                movingShotSolution);
        double clampedTurretDeltaDegrees = MovingShotMath.clampTurretDeltaDegrees(
                movingShotSolution.getTurretDeltaDegrees(),
                horizontalAim.getMinimumAngle().in(Degrees),
                horizontalAim.getMaximumAngle().in(Degrees));
        verticalAim.setAngle(Degrees.of(commandedHoodAngleDegrees));
        horizontalAim.setAngle(Degrees.of(clampedTurretDeltaDegrees));
        shooter.setCoupledIPS(idealFlywheelCommandIps);
        lastCommandedFlywheelSetpointIps = idealFlywheelCommandIps;
        lastValidRobotHeadingTargetFldRadians =
                Rotation2d.fromDegrees(movingShotSolution.getRobotHeadingDegrees());
        lastValidTurretDeltaDegrees = clampedTurretDeltaDegrees;
        lastValidHoodAngleDegrees = commandedHoodAngleDegrees;
        lastValidFlywheelCommandIps = idealFlywheelCommandIps;
        hasLatchedShotCommand = true;
        return fixedFlywheelStatus;
    }

    private void updateLastTravelVectorFldMeters() {
        Pose2d currentRobotPoseFldMetersRadians = poseEstimator.getFusedPoseSupplier().get();
        if (currentRobotPoseFldMetersRadians == null) {
            return;
        }
        lastTravelVectorFldMeters =
                preferredHeadingTracker.update(currentRobotPoseFldMetersRadians.getTranslation());
    }

    private Rotation2d getPreferredRobotHeadingFldRadians(
            Translation2d robotTranslationFldMeters,
            Translation2d targetTranslationFldMeters,
            Rotation2d fallbackHeadingFldRadians) {
        return MovingShotMath.getPreferredHeadingForTravelDirection(
                lastTravelVectorFldMeters,
                robotTranslationFldMeters,
                targetTranslationFldMeters,
                fallbackHeadingFldRadians);
    }

    private void applyHeldShotCommand() {
        verticalAim.setAngle(Degrees.of(lastValidHoodAngleDegrees));
        horizontalAim.setAngle(Degrees.of(lastValidTurretDeltaDegrees));
        shooter.setCoupledIPS(lastValidFlywheelCommandIps);
        lastCommandedFlywheelSetpointIps = lastValidFlywheelCommandIps;
    }

}
