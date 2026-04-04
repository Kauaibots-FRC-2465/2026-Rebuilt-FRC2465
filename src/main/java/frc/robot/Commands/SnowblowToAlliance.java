package frc.robot.Commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.Objects;
import java.util.function.DoubleSupplier;
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
 */
public class SnowblowToAlliance extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final PoseEstimatorSubsystem poseEstimator;
    private final SparkAnglePositionSubsystem horizontalAim;
    private final SparkAnglePositionSubsystem verticalAim;
    private final ShooterSubsystem shooter;
    private final Supplier<SwerveRequest> driveRequestSupplier;
    private final DoubleSupplier azimuthTrimDegreesSupplier;
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
    private final edu.wpi.first.networktables.DoubleArrayPublisher futurePosePublisher;
    private final edu.wpi.first.networktables.DoubleArrayPublisher futureVelocityPublisher;
    private final BooleanPublisher validSolutionPublisher;
    private final StringPublisher fieldTypePublisher;
    private final PoseEstimatorSubsystem.PredictedFusedState futureState =
            new PoseEstimatorSubsystem.PredictedFusedState();
    private final BallTrajectoryLookup.MovingShotSolution idealMovingShotSolution =
            new BallTrajectoryLookup.MovingShotSolution();
    private final BallTrajectoryLookup.MovingShotSolution movingShotSolution =
            new BallTrajectoryLookup.MovingShotSolution();
    private final double[] futureFieldPose = new double[3];
    private final double[] futureFieldVelocity = new double[3];
    private Translation2d lastFieldRelativeDriveDirection = new Translation2d();

    public SnowblowToAlliance(
            CommandSwerveDrivetrain drivetrain,
            PoseEstimatorSubsystem poseEstimator,
            SparkAnglePositionSubsystem horizontalAim,
            SparkAnglePositionSubsystem verticalAim,
            ShooterSubsystem shooter,
            Supplier<SwerveRequest> driveRequestSupplier,
            DoubleSupplier azimuthTrimDegreesSupplier) {
        this.drivetrain = Objects.requireNonNull(drivetrain, "drivetrain must not be null");
        this.poseEstimator = Objects.requireNonNull(poseEstimator, "poseEstimator must not be null");
        this.horizontalAim = Objects.requireNonNull(horizontalAim, "horizontalAim must not be null");
        this.verticalAim = Objects.requireNonNull(verticalAim, "verticalAim must not be null");
        this.shooter = Objects.requireNonNull(shooter, "shooter must not be null");
        this.driveRequestSupplier =
                Objects.requireNonNull(driveRequestSupplier, "driveRequestSupplier must not be null");
        this.azimuthTrimDegreesSupplier =
                Objects.requireNonNull(azimuthTrimDegreesSupplier, "azimuthTrimDegreesSupplier must not be null");
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
        futurePosePublisher = snowblowTable.getDoubleArrayTopic("futurePose").publish();
        futureVelocityPublisher = snowblowTable.getDoubleArrayTopic("futureVelocity").publish();
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
    }

    @Override
    public void execute() {
        SwerveRequest requestedDrive = driveRequestSupplier.get();
        if (!(requestedDrive instanceof SwerveRequest.FieldCentric fieldCentricRequest)) {
            clearSolutionTelemetry();
            drivetrain.setControl(requestedDrive);
            return;
        }

        updateLastDriveDirection(fieldCentricRequest);
        if (!MovingShotMath.predictFutureStateFromCommand(
                poseEstimator,
                drivetrain.getDriverPerspectiveForward(),
                fieldCentricRequest.VelocityX,
                fieldCentricRequest.VelocityY,
                ShooterConstants.COMMANDED_SHOOTER_LOOKAHEAD_SECONDS,
                futureState)) {
            clearSolutionTelemetry();
            shooter.setCoupledIPS(0.0);
            horizontalAim.setAngle(Degrees.of(0.0));
            drivetrain.setControl(requestedDrive);
            return;
        }
        publishFutureState();

        Pose2d futurePose = new Pose2d(
                futureState.xMeters,
                futureState.yMeters,
                Rotation2d.fromRadians(futureState.headingRadians));
        Translation2d target = getTarget(futurePose);
        publishTarget(target);

        Rotation2d preferredRobotHeading = getPreferredRobotHeading(futurePose.getRotation());
        Rotation2d robotHeadingTarget = preferredRobotHeading;
        if (updateShooterSolution(target, preferredRobotHeading)) {
            robotHeadingTarget = Rotation2d.fromDegrees(movingShotSolution.getRobotHeadingDegrees());
        }
        Rotation2d operatorPerspectiveHeadingTarget =
                robotHeadingTarget.minus(drivetrain.getDriverPerspectiveForward());

        drivetrain.setControl(
                facingAngleDrive
                        .withVelocityX(fieldCentricRequest.VelocityX)
                        .withVelocityY(fieldCentricRequest.VelocityY)
                        .withTargetDirection(operatorPerspectiveHeadingTarget)
                        .withDeadband(fieldCentricRequest.Deadband)
                        .withRotationalDeadband(fieldCentricRequest.RotationalDeadband)
                        .withCenterOfRotation(fieldCentricRequest.CenterOfRotation)
                        .withDriveRequestType(fieldCentricRequest.DriveRequestType)
                        .withSteerRequestType(fieldCentricRequest.SteerRequestType)
                        .withDesaturateWheelSpeeds(fieldCentricRequest.DesaturateWheelSpeeds)
                        .withForwardPerspective(fieldCentricRequest.ForwardPerspective));
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
    private Translation2d getTarget(Pose2d robotPose) {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        return FieldMath.getSnowblowTarget(robotPose, lastFieldRelativeDriveDirection, alliance);
    }

    private boolean updateShooterSolution(
            Translation2d target,
            Rotation2d preferredRobotHeading) {
        double minimumHoodAngleDegrees = verticalAim.getMinimumAngle().in(Degrees);
        double maximumHoodAngleDegrees = verticalAim.getMaximumAngle().in(Degrees);
        double targetDistanceInches = Inches.convertFrom(
                target.getDistance(new Translation2d(futureState.xMeters, futureState.yMeters)),
                Meters);
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
                ShooterConstants.COMMANDED_SNOWBLOW_TARGET_ELEVATION_INCHES,
                ShooterConstants.COMMANDED_MAXIMUM_SHOOTING_HEIGHT_INCHES,
                preferredRobotHeading.getRadians(),
                horizontalAim.getMinimumAngle().in(Degrees),
                horizontalAim.getMaximumAngle().in(Degrees),
                idealMovingShotSolution);
        if (!hasIdealSolution) {
            shooter.setCoupledIPS(0.0);
            horizontalAim.setAngle(Degrees.of(0.0));
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
                        ShooterConstants.COMMANDED_SNOWBLOW_TARGET_ELEVATION_INCHES,
                        ShooterConstants.COMMANDED_MAXIMUM_SHOOTING_HEIGHT_INCHES,
                        preferredRobotHeading.getRadians(),
                        horizontalAim.getMinimumAngle().in(Degrees),
                        horizontalAim.getMaximumAngle().in(Degrees),
                        predictedFlywheelCommandIps,
                        movingShotSolution);
        // Debug dashboard telemetry disabled to reduce NetworkTables traffic.
        // targetDistanceInchesPublisher.set(targetDistanceInches);
        // targetElevationInchesPublisher.set(ShooterConstants.COMMANDED_SNOWBLOW_TARGET_ELEVATION_INCHES);
        // validSolutionPublisher.set(fixedFlywheelStatus == BallTrajectoryLookup.FixedFlywheelShotStatus.VALID);

        if (fixedFlywheelStatus == BallTrajectoryLookup.FixedFlywheelShotStatus.NO_SOLUTION) {
            // hoodAngleDegreesPublisher.set(Double.NaN);
            // shotExitVelocityIpsPublisher.set(0.0);
            // fieldRelativeExitVelocityIpsPublisher.set(0.0);
            // flywheelCommandIpsPublisher.set(0.0);
            // shotAzimuthDegreesPublisher.set(Double.NaN);
            // turretDeltaDegreesPublisher.set(Double.NaN);
            // robotHeadingDegreesPublisher.set(preferredRobotHeading.getDegrees());
            shooter.setCoupledIPS(0.0);
            horizontalAim.setAngle(Degrees.of(0.0));
            return false;
        }

        double commandedHoodAngleDegrees = switch (fixedFlywheelStatus) {
            case TOO_SLOW -> minimumHoodAngleDegrees;
            case TOO_FAST -> maximumHoodAngleDegrees;
            case VALID -> movingShotSolution.getHoodAngleDegrees();
            case NO_SOLUTION -> Double.NaN;
        };
        double clampedTurretDeltaDegrees = clampTurretDeltaDegrees(
                movingShotSolution.getTurretDeltaDegrees() + azimuthTrimDegreesSupplier.getAsDouble());
        double commandedRobotHeadingDegrees = movingShotSolution.getRobotHeadingDegrees();
        // hoodAngleDegreesPublisher.set(movingShotSolution.getHoodAngleDegrees());
        // shotExitVelocityIpsPublisher.set(movingShotSolution.getLauncherRelativeExitVelocityIps());
        // fieldRelativeExitVelocityIpsPublisher.set(movingShotSolution.getFieldRelativeExitVelocityIps());
        // flywheelCommandIpsPublisher.set(movingShotSolution.getFlywheelCommandIps());
        // shotAzimuthDegreesPublisher.set(movingShotSolution.getShotAzimuthDegrees());
        // turretDeltaDegreesPublisher.set(clampedTurretDeltaDegrees);
        // robotHeadingDegreesPublisher.set(commandedRobotHeadingDegrees);
        verticalAim.setAngle(Degrees.of(commandedHoodAngleDegrees));
        horizontalAim.setAngle(Degrees.of(clampedTurretDeltaDegrees));
        shooter.setCoupledIPS(idealFlywheelCommandIps);
        return true;
    }

    private void updateLastDriveDirection(SwerveRequest.FieldCentric fieldCentricRequest) {
        Translation2d fieldRelativeVelocity = new Translation2d(
                fieldCentricRequest.VelocityX,
                fieldCentricRequest.VelocityY).rotateBy(drivetrain.getDriverPerspectiveForward());
        lastFieldRelativeDriveDirection = FieldMath.updateLastDriveDirection(
                fieldRelativeVelocity,
                lastFieldRelativeDriveDirection);
    }

    private Rotation2d getPreferredRobotHeading(Rotation2d fallbackHeading) {
        if (lastFieldRelativeDriveDirection.getNorm() > 1e-9) {
            return lastFieldRelativeDriveDirection.getAngle();
        }
        return fallbackHeading;
    }

    private double clampTurretDeltaDegrees(double turretDeltaDegrees) {
        return Math.max(
                horizontalAim.getMinimumAngle().in(Degrees),
                Math.min(horizontalAim.getMaximumAngle().in(Degrees), turretDeltaDegrees));
    }

    private void publishTarget(Translation2d target) {
        TuningDashboard.publishShootingTarget(target);
    }

    private void publishFutureState() {
        futureFieldPose[0] = futureState.xMeters;
        futureFieldPose[1] = futureState.yMeters;
        futureFieldPose[2] = Math.toDegrees(futureState.headingRadians);
        futureFieldVelocity[0] = futureState.vxMetersPerSecond;
        futureFieldVelocity[1] = futureState.vyMetersPerSecond;
        futureFieldVelocity[2] = Math.toDegrees(futureState.omegaRadiansPerSecond);
        // Debug dashboard telemetry disabled to reduce NetworkTables traffic.
        // futurePosePublisher.set(futureFieldPose);
        // futureVelocityPublisher.set(futureFieldVelocity);
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
