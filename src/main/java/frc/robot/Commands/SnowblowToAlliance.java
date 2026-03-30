package frc.robot.Commands;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Objects;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.fieldmath.FieldMath;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakePositionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SparkAnglePositionSubsystem;

/**
 * Skeleton command for future alliance-directed snowblowing behavior.
 *
 * <p>For now this command only preserves normal driving while taking ownership
 * of the drivetrain, horizontal aim, vertical aim, and shooter subsystems.
 */
public class SnowblowToAlliance extends Command {
    private static final long DRIVE_DIRECTION_LOOKBACK_MICROSECONDS = 250_000;

    private static class PoseSample {
        final Translation2d translation;
        final long timestampMicroseconds;

        PoseSample(Translation2d translation, long timestampMicroseconds) {
            this.translation = translation;
            this.timestampMicroseconds = timestampMicroseconds;
        }
    }

    private final CommandSwerveDrivetrain drivetrain;
    private final Supplier<SwerveRequest> driveRequestSupplier;
    private final SwerveRequest.FieldCentricFacingAngle facingAngleDrive =
            new SwerveRequest.FieldCentricFacingAngle();
    private final DoubleArrayPublisher targetPublisher;
    private final StringPublisher fieldTypePublisher;
    private final Deque<PoseSample> recentPoseSamples = new ArrayDeque<>();
    private Translation2d lastFieldRelativeDriveDirection = new Translation2d();

    public SnowblowToAlliance(
            CommandSwerveDrivetrain drivetrain,
            IntakePositionSubsystem horizontalAim,
            SparkAnglePositionSubsystem verticalAim,
            ShooterSubsystem shooter,
            Supplier<SwerveRequest> driveRequestSupplier) {
        this.drivetrain = Objects.requireNonNull(drivetrain, "drivetrain must not be null");
        Objects.requireNonNull(horizontalAim, "horizontalAim must not be null");
        Objects.requireNonNull(verticalAim, "verticalAim must not be null");
        Objects.requireNonNull(shooter, "shooter must not be null");
        this.driveRequestSupplier =
                Objects.requireNonNull(driveRequestSupplier, "driveRequestSupplier must not be null");
        NetworkTable snowblowTable = NetworkTableInstance.getDefault().getTable("SnowblowToAlliance");
        targetPublisher = snowblowTable.getDoubleArrayTopic("target").publish();
        fieldTypePublisher = snowblowTable.getStringTopic(".type").publish();
        fieldTypePublisher.set("Field2d");

        facingAngleDrive.withHeadingPID(4.0, 0.0, 0.0);
        facingAngleDrive.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(this.drivetrain, horizontalAim, verticalAim, shooter);
    }

    @Override
    public void initialize() {
        recentPoseSamples.clear();
        recordCurrentPoseSample();
    }

    @Override
    public void execute() {
        SwerveRequest requestedDrive = driveRequestSupplier.get();
        updateLastDriveDirection();
        if (!(requestedDrive instanceof SwerveRequest.FieldCentric fieldCentricRequest)) {
            drivetrain.setControl(requestedDrive);
            return;
        }

        Translation2d target = getTarget();
        targetPublisher.set(new double[] {target.getX(), target.getY(), 0.0});
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
        recentPoseSamples.clear();
    }

    /**
     * Returns the current alliance-relative snowblow target point on the field.
     */
    private Translation2d getTarget() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        Pose2d robotPose = drivetrain.getState().Pose;
        return FieldMath.getSnowblowTarget(robotPose, lastFieldRelativeDriveDirection, alliance);
    }

    private void updateLastDriveDirection() {
        recordCurrentPoseSample();
        PoseSample currentSample = recentPoseSamples.peekLast();
        PoseSample referenceSample = getReferencePoseSample(currentSample.timestampMicroseconds);
        if (currentSample != null && referenceSample != null) {
            lastFieldRelativeDriveDirection = FieldMath.updateLastDriveDirection(
                    currentSample.translation,
                    referenceSample.translation,
                    lastFieldRelativeDriveDirection);
        }
    }

    private void recordCurrentPoseSample() {
        recentPoseSamples.addLast(new PoseSample(
                drivetrain.getState().Pose.getTranslation(),
                RobotController.getFPGATime()));
    }

    private PoseSample getReferencePoseSample(long currentTimestampMicroseconds) {
        long cutoffTimestampMicroseconds = currentTimestampMicroseconds - DRIVE_DIRECTION_LOOKBACK_MICROSECONDS;
        while (recentPoseSamples.size() > 1) {
            java.util.Iterator<PoseSample> poseIterator = recentPoseSamples.iterator();
            poseIterator.next();
            PoseSample nextOldestSample = poseIterator.next();
            if (nextOldestSample.timestampMicroseconds <= cutoffTimestampMicroseconds) {
                recentPoseSamples.removeFirst();
                continue;
            }
            break;
        }

        PoseSample oldestSample = recentPoseSamples.peekFirst();
        if (oldestSample != null && oldestSample.timestampMicroseconds <= cutoffTimestampMicroseconds) {
            return oldestSample;
        }
        return null;
    }
}
