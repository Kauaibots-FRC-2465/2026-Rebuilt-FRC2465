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
 * Fld = absolute WPILib field frame and the default for internal geometry/solver math.
 * Drv = driver-perspective field-centric request frame at the Phoenix request boundary.
 * Rbt = robot/body frame.
 *
 * <p>Unit conventions:
 * Translation2d and Pose2d values use WPILib meters/radians defaults.
 * Primitive suffixes spell units explicitly: Degrees, Radians, Inches, Meters, Ips.
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
    private final PoseEstimatorSubsystem.PredictedFusedState futureStateFldMetersRadians =
            new PoseEstimatorSubsystem.PredictedFusedState();
    private final BallTrajectoryLookup.MovingShotSolution idealMovingShotSolution =
            new BallTrajectoryLookup.MovingShotSolution();
    private final BallTrajectoryLookup.MovingShotSolution movingShotSolution =
            new BallTrajectoryLookup.MovingShotSolution();
    private final double[] futurePoseFldMetersDegreesArray = new double[3];
    private final double[] futureVelocityFldMetersPerSecondDegreesPerSecondArray = new double[3];
    private final TravelWindowDirectionTracker preferredHeadingTracker =
            new TravelWindowDirectionTracker(
                    Meters.convertFrom(
                            ShooterConstants.COMMANDED_PREFERRED_HEADING_TRAVEL_WINDOW_INCHES,
                            Inches),
                    Meters.convertFrom(
                            ShooterConstants.COMMANDED_PREFERRED_HEADING_MIN_SAMPLE_SPACING_INCHES,
                            Inches));
    private Translation2d lastTravelVectorFldMeters = new Translation2d();
    private double lastCommandedFlywheelSetpointIps = 0.0;

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
        lastTravelVectorFldMeters = new Translation2d();
        lastCommandedFlywheelSetpointIps = shooter.getMainFlywheelSpeedIPS();
    }

    @Override
    public void execute() {
        SwerveRequest requestedDrive = driveRequestSupplier.get();
        if (!(requestedDrive instanceof SwerveRequest.FieldCentric fieldCentricRequestDrv)) {
            clearSolutionTelemetry();
            drivetrain.setControl(requestedDrive);
            return;
        }

        updateLastTravelVectorFldMeters();
        if (!poseEstimator.getPredictedFusedState(
                ShooterConstants.COMMANDED_SHOOTER_LOOKAHEAD_SECONDS,
                futureStateFldMetersRadians)) {
            clearSolutionTelemetry();
            shooter.setCoupledIPS(0.0);
            lastCommandedFlywheelSetpointIps = 0.0;
            horizontalAim.setAngle(Degrees.of(0.0));
            drivetrain.setControl(requestedDrive);
            return;
        }
        publishFutureFieldState();

        Pose2d futureRobotPoseFldMetersRadians = new Pose2d(
                futureStateFldMetersRadians.xMeters,
                futureStateFldMetersRadians.yMeters,
                Rotation2d.fromRadians(futureStateFldMetersRadians.headingRadians));
        Translation2d targetTranslationFldMeters = getTargetTranslationFldMeters(futureRobotPoseFldMetersRadians);
        publishTargetTranslationFldMeters(targetTranslationFldMeters);

        Rotation2d preferredRobotHeadingFldRadians = getPreferredRobotHeadingFldRadians(
                futureRobotPoseFldMetersRadians.getTranslation(),
                targetTranslationFldMeters,
                futureRobotPoseFldMetersRadians.getRotation());
        Rotation2d robotHeadingTargetFldRadians = preferredRobotHeadingFldRadians;
        if (updateShooterSolution(targetTranslationFldMeters, preferredRobotHeadingFldRadians)) {
            robotHeadingTargetFldRadians = Rotation2d.fromDegrees(movingShotSolution.getRobotHeadingDegrees());
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
    public void end(boolean interrupted) {
    }

    /**
     * Returns the current alliance-relative snowblow target point on the field.
     */
    private Translation2d getTargetTranslationFldMeters(Pose2d robotPoseFldMetersRadians) {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        return FieldMath.getSnowblowTarget(robotPoseFldMetersRadians, lastTravelVectorFldMeters, alliance);
    }

    private boolean updateShooterSolution(
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
            shooter.setCoupledIPS(0.0);
            lastCommandedFlywheelSetpointIps = 0.0;
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
        lastCommandedFlywheelSetpointIps = idealFlywheelCommandIps;
        return true;
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

    private void publishTargetTranslationFldMeters(Translation2d targetTranslationFldMeters) {
        TuningDashboard.publishShootingTarget(targetTranslationFldMeters);
    }

    private void publishFutureFieldState() {
        futurePoseFldMetersDegreesArray[0] = futureStateFldMetersRadians.xMeters;
        futurePoseFldMetersDegreesArray[1] = futureStateFldMetersRadians.yMeters;
        futurePoseFldMetersDegreesArray[2] = Math.toDegrees(futureStateFldMetersRadians.headingRadians);
        futureVelocityFldMetersPerSecondDegreesPerSecondArray[0] =
                futureStateFldMetersRadians.vxMetersPerSecond;
        futureVelocityFldMetersPerSecondDegreesPerSecondArray[1] =
                futureStateFldMetersRadians.vyMetersPerSecond;
        futureVelocityFldMetersPerSecondDegreesPerSecondArray[2] =
                Math.toDegrees(futureStateFldMetersRadians.omegaRadiansPerSecond);
        // Debug dashboard telemetry disabled to reduce NetworkTables traffic.
        // futurePoseFldPublisher.set(futurePoseFldMetersDegreesArray);
        // futureVelocityFldPublisher.set(futureVelocityFldMetersPerSecondDegreesPerSecondArray);
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
