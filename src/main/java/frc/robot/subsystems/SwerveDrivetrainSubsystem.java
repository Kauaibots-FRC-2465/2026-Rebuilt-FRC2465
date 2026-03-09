package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.utility.PPrint;

public class SwerveDrivetrainSubsystem implements Subsystem {
    CommandSwerveDrivetrain encapsulatedDrivetrain;
    Supplier<Pose2d> poseSupplier;
    Supplier<Long> timestampSupplier;

    public SwerveDrivetrainSubsystem(CommandSwerveDrivetrain encapsulate, PoseEstimatorSubsystem poseEstimator) {
        encapsulatedDrivetrain = encapsulate;
        this.poseSupplier = poseEstimator.getFusedPoseSupplier();
        timestampSupplier = poseEstimator.getTimestampSupplier();
        CommandScheduler.getInstance().registerSubsystem(this);
    }

    @Override
    public void periodic() {
        Pose2d retrievedPose = poseSupplier.get();
        if (retrievedPose != null) PPrint.every(25, retrievedPose.toString());
        if (retrievedPose != null) encapsulatedDrivetrain.addVisionMeasurement(retrievedPose, timestampSupplier.get());
    }
    
    public void applyRequest(Supplier<SwerveRequest> request) {
        encapsulatedDrivetrain.applyRequest(request);
    }

    public CommandSwerveDrivetrain getEncapsulate() {
        return encapsulatedDrivetrain;
    }
}