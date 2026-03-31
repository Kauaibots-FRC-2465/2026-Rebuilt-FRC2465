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
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.fieldmath.FieldMath;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SparkAnglePositionSubsystem;

/**
 * Skeleton command for future alliance-directed snowblowing behavior.
 *
 * <p>For now this command only preserves normal driving while taking ownership
 * of the drivetrain, horizontal aim, vertical aim, and shooter subsystems.
 */
public class SnowblowToAlliance extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final SparkAnglePositionSubsystem verticalAim;
    private final ShooterSubsystem shooter;
    private final Supplier<SwerveRequest> driveRequestSupplier;
    private final SwerveRequest.FieldCentricFacingAngle facingAngleDrive =
            new SwerveRequest.FieldCentricFacingAngle();
    private final DoubleArrayPublisher targetPublisher;
    private final DoublePublisher targetDistanceInchesPublisher;
    private final DoublePublisher hoodAngleDegreesPublisher;
    private final DoublePublisher flywheelSpeedIpsPublisher;
    private final StringPublisher fieldTypePublisher;
    private Translation2d lastFieldRelativeDriveDirection = new Translation2d();

    public SnowblowToAlliance(
            CommandSwerveDrivetrain drivetrain,
            SparkAnglePositionSubsystem verticalAim,
            ShooterSubsystem shooter,
            Supplier<SwerveRequest> driveRequestSupplier) {
        this.drivetrain = Objects.requireNonNull(drivetrain, "drivetrain must not be null");
        this.verticalAim = Objects.requireNonNull(verticalAim, "verticalAim must not be null");
        this.shooter = Objects.requireNonNull(shooter, "shooter must not be null");
        this.driveRequestSupplier =
                Objects.requireNonNull(driveRequestSupplier, "driveRequestSupplier must not be null");
        NetworkTable snowblowTable = NetworkTableInstance.getDefault().getTable("SnowblowToAlliance");
        targetPublisher = snowblowTable.getDoubleArrayTopic("target").publish();
        targetDistanceInchesPublisher = snowblowTable.getDoubleTopic("targetDistanceInches").publish();
        hoodAngleDegreesPublisher = snowblowTable.getDoubleTopic("hoodAngleDegrees").publish();
        flywheelSpeedIpsPublisher = snowblowTable.getDoubleTopic("flywheelSpeedIps").publish();
        fieldTypePublisher = snowblowTable.getStringTopic(".type").publish();
        fieldTypePublisher.set("Field2d");

        facingAngleDrive.withHeadingPID(5.0, 0.0, 0.0);
        facingAngleDrive.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(this.drivetrain, verticalAim, shooter);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        SwerveRequest requestedDrive = driveRequestSupplier.get();
        if (!(requestedDrive instanceof SwerveRequest.FieldCentric fieldCentricRequest)) {
            Translation2d target = getTarget();
            targetPublisher.set(new double[] {target.getX(), target.getY(), 0.0});
            updateShooterSolution(target);
            drivetrain.setControl(requestedDrive);
            return;
        }

        updateLastDriveDirection(fieldCentricRequest);
        Translation2d target = getTarget();
        targetPublisher.set(new double[] {target.getX(), target.getY(), 0.0});
        updateShooterSolution(target);
        Rotation2d targetDirection = target.minus(drivetrain.getState().Pose.getTranslation()).getAngle();

        drivetrain.setControl(
                facingAngleDrive
                        .withVelocityX(fieldCentricRequest.VelocityX)
                        .withVelocityY(fieldCentricRequest.VelocityY)
                        .withTargetDirection(targetDirection)
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
    private Translation2d getTarget() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        Pose2d robotPose = drivetrain.getState().Pose;
        return FieldMath.getSnowblowTarget(robotPose, lastFieldRelativeDriveDirection, alliance);
    }

    private void updateShooterSolution(Translation2d target) {
        Translation2d robotTranslation = drivetrain.getState().Pose.getTranslation();
        double targetDistanceMeters = robotTranslation.getDistance(target);
        int targetDistanceInches = (int) Math.round(Inches.convertFrom(targetDistanceMeters, Meters));
        double hoodAngleDegrees = ShooterInterpolator.getMinValidAngle(targetDistanceInches);
        targetDistanceInchesPublisher.set(targetDistanceInches);

        if (!Double.isFinite(hoodAngleDegrees)) {
            hoodAngleDegreesPublisher.set(Double.NaN);
            flywheelSpeedIpsPublisher.set(0.0);
            shooter.setCoupledIPS(0.0);
            return;
        }

        double flywheelSpeedIps = ShooterInterpolator.getSpeedAtAngle(targetDistanceInches, hoodAngleDegrees);
        hoodAngleDegreesPublisher.set(hoodAngleDegrees);
        flywheelSpeedIpsPublisher.set(flywheelSpeedIps);
        verticalAim.setAngle(Degrees.of(hoodAngleDegrees));
        shooter.setCoupledIPS(flywheelSpeedIps);
    }

    private void updateLastDriveDirection(SwerveRequest.FieldCentric fieldCentricRequest) {
        lastFieldRelativeDriveDirection = FieldMath.updateLastDriveDirection(
                new Translation2d(fieldCentricRequest.VelocityX, fieldCentricRequest.VelocityY),
                lastFieldRelativeDriveDirection);
    }
}
