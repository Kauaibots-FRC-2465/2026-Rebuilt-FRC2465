package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Microseconds;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.utility.SlowCallMonitor;

public class SwerveDrivetrainSubsystem implements Subsystem {
    private static final double SLOW_PERIODIC_THRESHOLD_MS = 4.0;
    private static final double SLOW_POSE_SUPPLIER_THRESHOLD_MS = 3.0;
    private static final double SLOW_ADD_VISION_THRESHOLD_MS = 2.0;

    CommandSwerveDrivetrain encapsulatedDrivetrain;
    private final PoseEstimatorSubsystem poseEstimator;
    private final PoseEstimatorSubsystem.PredictedFusedState predictedState =
            new PoseEstimatorSubsystem.PredictedFusedState();

    public SwerveDrivetrainSubsystem(CommandSwerveDrivetrain encapsulate, PoseEstimatorSubsystem poseEstimator) {
        encapsulatedDrivetrain = encapsulate;
        this.poseEstimator = poseEstimator;
        CommandScheduler.getInstance().registerSubsystem(this);
    }

    @Override
    public void periodic() {
        long periodicStartMicros = SlowCallMonitor.nowMicros();
        long poseSupplierMicros;
        long addVisionMicros = 0L;

        long poseSupplierStartMicros = SlowCallMonitor.nowMicros();
        boolean hasPredictedState = poseEstimator.getPredictedFusedState(0.0, predictedState);
        poseSupplierMicros = SlowCallMonitor.nowMicros() - poseSupplierStartMicros;
        if (hasPredictedState) {
            Pose2d retrievedPose = new Pose2d(
                    predictedState.xMeters,
                    predictedState.yMeters,
                    Rotation2d.fromRadians(predictedState.headingRadians));
            long addVisionStartMicros = SlowCallMonitor.nowMicros();
            encapsulatedDrivetrain.addVisionMeasurement(
                    retrievedPose,
                    Microseconds.of(predictedState.timestampMicros));
            addVisionMicros = SlowCallMonitor.nowMicros() - addVisionStartMicros;
        }

        long totalMicros = SlowCallMonitor.nowMicros() - periodicStartMicros;
        if (SlowCallMonitor.isSlow(totalMicros, SLOW_PERIODIC_THRESHOLD_MS)
                || SlowCallMonitor.isSlow(poseSupplierMicros, SLOW_POSE_SUPPLIER_THRESHOLD_MS)
                || SlowCallMonitor.isSlow(addVisionMicros, SLOW_ADD_VISION_THRESHOLD_MS)) {
            SlowCallMonitor.print(
                    "SwerveDrivetrainSubsystem.periodic",
                    totalMicros,
                    String.format(
                            "poseSupplier=%.3f ms addVisionMeasurement=%.3f ms poseNull=%s",
                            SlowCallMonitor.toMillis(poseSupplierMicros),
                            SlowCallMonitor.toMillis(addVisionMicros),
                            Boolean.toString(!hasPredictedState)));
        }
    }
    
    public void applyRequest(Supplier<SwerveRequest> request) {
        encapsulatedDrivetrain.applyRequest(request);
    }

    public CommandSwerveDrivetrain getEncapsulate() {
        return encapsulatedDrivetrain;
    }
}
