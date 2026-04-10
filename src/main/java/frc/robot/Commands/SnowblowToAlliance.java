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
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.fieldmath.FieldMath;
import frc.robot.fieldmath.TravelWindowDirectionTracker;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SparkAnglePositionSubsystem;
import frc.robot.utility.TuningDashboard;

/**
 * Skeleton command for future alliance-directed snowblowing behavior.
 *
 * <p>For now this command only preserves normal driving while taking ownership
 * of the drivetrain, horizontal aim, vertical aim, and shooter subsystems.
 *
 * <p>Frame suffixes used in this command:
 * Fld = field frame and the default for internal geometry/solver math.
 * Drv = driver-perspective field-centric request frame at the Phoenix request boundary.
 */
public class SnowblowToAlliance extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final PoseEstimatorSubsystem poseEstimator;
    private final SparkAnglePositionSubsystem horizontalAim;
    private final SparkAnglePositionSubsystem verticalAim;
    private final ShooterSubsystem shooter;
    private final Supplier<SwerveRequest> driveRequestSupplier;
    private final SwerveRequest.FieldCentricFacingAngle facingAngleDrive =
            new SwerveRequest.FieldCentricFacingAngle();
    private final DoublePublisher targetDistanceInchesPublisher;
    private final DoublePublisher targetElevationInchesPublisher;
    private final DoublePublisher hoodAngleDegreesPublisher;
    private final DoublePublisher shotExitVelocityIpsPublisher;
    private final DoublePublisher fieldRelativeExitVelocityIpsPublisher;
    private final DoublePublisher flywheelCommandIpsPublisher;
    private final DoublePublisher shotAzimuthDegreesPublisher;
    private final DoublePublisher turretDeltaDegreesPublisher;
    private final DoublePublisher robotHeadingDegreesPublisher;
    private final edu.wpi.first.networktables.DoubleArrayPublisher futurePoseFldPublisher;
    private final edu.wpi.first.networktables.DoubleArrayPublisher futureVelocityFldPublisher;
    private final BooleanPublisher validSolutionPublisher;
    private final StringPublisher fieldTypePublisher;
    private final PoseEstimatorSubsystem.PredictedFusedState futureStateFld =
            new PoseEstimatorSubsystem.PredictedFusedState();
    private final BallTrajectoryLookup.MovingShotSolution idealMovingShotSolution =
            new BallTrajectoryLookup.MovingShotSolution();
    private final BallTrajectoryLookup.MovingShotSolution movingShotSolution =
            new BallTrajectoryLookup.MovingShotSolution();
    private final double[] futurePoseFldArray = new double[3];
    private final double[] futureVelocityFldArray = new double[3];
    private final TravelWindowDirectionTracker preferredHeadingTracker =
            new TravelWindowDirectionTracker(
                    Meters.convertFrom(
                            ShooterConstants.COMMANDED_PREFERRED_HEADING_TRAVEL_WINDOW_INCHES,
                            Inches),
                    Meters.convertFrom(
                            ShooterConstants.COMMANDED_PREFERRED_HEADING_MIN_SAMPLE_SPACING_INCHES,
                            Inches));
    private Translation2d lastDriveDirectionFld = new Translation2d();

    public SnowblowToAlliance(
            CommandSwerveDrivetrain drivetrain,
            PoseEstimatorSubsystem poseEstimator,
            SparkAnglePositionSubsystem horizontalAim,
            SparkAnglePositionSubsystem verticalAim,
            ShooterSubsystem shooter,
            Supplier<SwerveRequest> driveRequestSupplier) {
        this.drivetrain = Objects.requireNonNull(drivetrain, "drivetrain must not be null");
        this.poseEstimator = Objects.requireNonNull(poseEstimator, "poseEstimator must not be null");
        this.horizontalAim = Objects.requireNonNull(horizontalAim, "horizontalAim must not be null");
        this.verticalAim = Objects.requireNonNull(verticalAim, "verticalAim must not be null");
        this.shooter = Objects.requireNonNull(shooter, "shooter must not be null");
        this.driveRequestSupplier =
                Objects.requireNonNull(driveRequestSupplier, "driveRequestSupplier must not be null");
        NetworkTable snowblowTable = NetworkTableInstance.getDefault().getTable("SnowblowToAlliance");
        targetDistanceInchesPublisher = snowblowTable.getDoubleTopic("targetDistanceInches").publish();
        targetElevationInchesPublisher = snowblowTable.getDoubleTopic("targetElevationInches").publish();
        hoodAngleDegreesPublisher = snowblowTable.getDoubleTopic("hoodAngleDegrees").publish();
        shotExitVelocityIpsPublisher = snowblowTable.getDoubleTopic("shotExitVelocityIps").publish();
        fieldRelativeExitVelocityIpsPublisher =
                snowblowTable.getDoubleTopic("fieldRelativeExitVelocityIps").publish();
        flywheelCommandIpsPublisher = snowblowTable.getDoubleTopic("flywheelCommandIps").publish();
        shotAzimuthDegreesPublisher = snowblowTable.getDoubleTopic("shotAzimuthDegrees").publish();
        turretDeltaDegreesPublisher = snowblowTable.getDoubleTopic("turretDeltaDegrees").publish();
        robotHeadingDegreesPublisher = snowblowTable.getDoubleTopic("robotHeadingDegrees").publish();
        futurePoseFldPublisher = snowblowTable.getDoubleArrayTopic("futurePose").publish();
        futureVelocityFldPublisher = snowblowTable.getDoubleArrayTopic("futureVelocity").publish();
        validSolutionPublisher = snowblowTable.getBooleanTopic("validSolution").publish();
        fieldTypePublisher = snowblowTable.getStringTopic(".type").publish();
        // Debug dashboard telemetry disabled to reduce NetworkTables traffic.
        // fieldTypePublisher.set("Field2d");

        facingAngleDrive.withHeadingPID(5.0, 0.0, 0.0);
        facingAngleDrive.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(this.drivetrain, horizontalAim, verticalAim, shooter);
    }

    @Override
    public void initialize() {
        preferredHeadingTracker.reset();
        lastDriveDirectionFld = new Translation2d();
    }

    @Override
    public void execute() {
        SwerveRequest requestedDrive = driveRequestSupplier.get();
        if (!(requestedDrive instanceof SwerveRequest.FieldCentric fieldCentricRequestDrv)) {
            clearSolutionTelemetry();
            drivetrain.setControl(requestedDrive);
            return;
        }

        updateLastDriveDirectionFld();
        if (!poseEstimator.getPredictedFusedState(
                ShooterConstants.COMMANDED_SHOOTER_LOOKAHEAD_SECONDS,
                futureStateFld)) {
            clearSolutionTelemetry();
            shooter.setCoupledIPS(0.0);
            horizontalAim.setAngle(Degrees.of(0.0));
            drivetrain.setControl(requestedDrive);
            return;
        }
        publishFutureFieldState();

        Pose2d futurePoseFld = new Pose2d(
                futureStateFld.xMeters,
                futureStateFld.yMeters,
                Rotation2d.fromRadians(futureStateFld.headingRadians));
        Translation2d targetFld = getTargetFld(futurePoseFld);
        publishTargetFld(targetFld);

        Rotation2d preferredRobotHeadingFld = getPreferredRobotHeadingFld(futurePoseFld.getRotation());
        Rotation2d robotHeadingTargetFld = preferredRobotHeadingFld;
        if (updateShooterSolution(targetFld, preferredRobotHeadingFld)) {
            robotHeadingTargetFld = Rotation2d.fromDegrees(movingShotSolution.getRobotHeadingDegrees());
        }
        Rotation2d robotHeadingTargetDrv =
                robotHeadingTargetFld.minus(drivetrain.getDriverPerspectiveForward());

        drivetrain.setControl(
                facingAngleDrive
                        .withVelocityX(fieldCentricRequestDrv.VelocityX)
                        .withVelocityY(fieldCentricRequestDrv.VelocityY)
                        .withTargetDirection(robotHeadingTargetDrv)
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
    public void end(boolean interrupted) {
    }

    /**
     * Returns the current alliance-relative snowblow target point on the field.
     */
    private Translation2d getTargetFld(Pose2d robotPoseFld) {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        return FieldMath.getSnowblowTarget(robotPoseFld, lastDriveDirectionFld, alliance);
    }

    private boolean updateShooterSolution(
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
                        ShooterConstants.COMMANDED_SNOWBLOW_TARGET_ELEVATION_INCHES,
                        ShooterConstants.COMMANDED_MAXIMUM_SHOOTING_HEIGHT_INCHES,
                        preferredRobotHeadingFld.getRadians(),
                        horizontalAim.getMinimumAngle().in(Degrees),
                        horizontalAim.getMaximumAngle().in(Degrees),
                        shooter.getMainFlywheelSpeedIPS(),
                        idealMovingShotSolution,
                        movingShotSolution);
        if (fixedFlywheelStatus == BallTrajectoryLookup.FixedFlywheelShotStatus.NO_SOLUTION) {
            shooter.setCoupledIPS(0.0);
            horizontalAim.setAngle(Degrees.of(0.0));
            return false;
        }

        double idealFlywheelCommandIps = idealMovingShotSolution.getFlywheelCommandIps();
        // Debug dashboard telemetry disabled to reduce NetworkTables traffic.
        // targetElevationInchesPublisher.set(ShooterConstants.COMMANDED_SNOWBLOW_TARGET_ELEVATION_INCHES);
        // validSolutionPublisher.set(fixedFlywheelStatus == BallTrajectoryLookup.FixedFlywheelShotStatus.VALID);

        double commandedHoodAngleDegrees = MovingShotMath.getCommandedHoodAngleDegrees(
                fixedFlywheelStatus,
                minimumHoodAngleDegrees,
                maximumHoodAngleDegrees,
                movingShotSolution);
        double clampedTurretDeltaDegrees = MovingShotMath.clampTurretDeltaDegrees(
                movingShotSolution.getTurretDeltaDegrees(),
                horizontalAim.getMinimumAngle().in(Degrees),
                horizontalAim.getMaximumAngle().in(Degrees));
        // hoodAngleDegreesPublisher.set(movingShotSolution.getHoodAngleDegrees());
        // shotExitVelocityIpsPublisher.set(movingShotSolution.getLauncherRelativeExitVelocityIps());
        // fieldRelativeExitVelocityIpsPublisher.set(movingShotSolution.getFieldRelativeExitVelocityIps());
        // flywheelCommandIpsPublisher.set(movingShotSolution.getFlywheelCommandIps());
        // shotAzimuthDegreesPublisher.set(movingShotSolution.getShotAzimuthDegrees());
        // turretDeltaDegreesPublisher.set(clampedTurretDeltaDegrees);
        // robotHeadingDegreesPublisher.set(movingShotSolution.getRobotHeadingDegrees());
        verticalAim.setAngle(Degrees.of(commandedHoodAngleDegrees));
        horizontalAim.setAngle(Degrees.of(clampedTurretDeltaDegrees));
        shooter.setCoupledIPS(idealFlywheelCommandIps);
        return true;
    }

    private void updateLastDriveDirectionFld() {
        Pose2d currentPoseFld = poseEstimator.getFusedPoseSupplier().get();
        if (currentPoseFld == null) {
            return;
        }
        lastDriveDirectionFld = preferredHeadingTracker.update(currentPoseFld.getTranslation());
    }

    private Rotation2d getPreferredRobotHeadingFld(Rotation2d fallbackHeadingFld) {
        if (lastDriveDirectionFld.getNorm() > 1e-9) {
            return lastDriveDirectionFld.getAngle();
        }
        return fallbackHeadingFld;
    }

    private void publishTargetFld(Translation2d targetFld) {
        TuningDashboard.publishShootingTarget(targetFld);
    }

    private void publishFutureFieldState() {
        futurePoseFldArray[0] = futureStateFld.xMeters;
        futurePoseFldArray[1] = futureStateFld.yMeters;
        futurePoseFldArray[2] = Math.toDegrees(futureStateFld.headingRadians);
        futureVelocityFldArray[0] = futureStateFld.vxMetersPerSecond;
        futureVelocityFldArray[1] = futureStateFld.vyMetersPerSecond;
        futureVelocityFldArray[2] = Math.toDegrees(futureStateFld.omegaRadiansPerSecond);
        // Debug dashboard telemetry disabled to reduce NetworkTables traffic.
        // futurePoseFldPublisher.set(futurePoseFldArray);
        // futureVelocityFldPublisher.set(futureVelocityFldArray);
    }

    private void clearSolutionTelemetry() {
        // Debug dashboard telemetry disabled to reduce NetworkTables traffic.
        // validSolutionPublisher.set(false);
        // targetDistanceInchesPublisher.set(Double.NaN);
        // targetElevationInchesPublisher.set(Double.NaN);
        // hoodAngleDegreesPublisher.set(Double.NaN);
        // shotExitVelocityIpsPublisher.set(0.0);
        // fieldRelativeExitVelocityIpsPublisher.set(0.0);
        // flywheelCommandIpsPublisher.set(0.0);
        // shotAzimuthDegreesPublisher.set(Double.NaN);
        // turretDeltaDegreesPublisher.set(Double.NaN);
        // robotHeadingDegreesPublisher.set(Double.NaN);
    }
}
