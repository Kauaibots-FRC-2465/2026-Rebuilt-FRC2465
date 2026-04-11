package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Microseconds;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.utility.SlowCallMonitor;

public class SwerveDrivetrainSubsystem implements Subsystem {
    private static final double SLOW_PERIODIC_THRESHOLD_MS = 4.0;
    private static final double SLOW_VISION_SUPPLIER_THRESHOLD_MS = 3.0;
    private static final double SLOW_ADD_VISION_THRESHOLD_MS = 2.0;

    CommandSwerveDrivetrain encapsulatedDrivetrain;
    private final LimelightSubsystem limelightSubsystem;
    private final LimelightSubsystem.VisionMeasurement latestVisionMeasurement =
            new LimelightSubsystem.VisionMeasurement();
    private long lastAppliedVisionMeasurementSequenceNumber = -1L;

    public SwerveDrivetrainSubsystem(
            CommandSwerveDrivetrain encapsulate,
            LimelightSubsystem limelightSubsystem) {
        encapsulatedDrivetrain = encapsulate;
        this.limelightSubsystem = limelightSubsystem;
        CommandScheduler.getInstance().registerSubsystem(this);
    }

    @Override
    public void periodic() {
        long periodicStartMicros = SlowCallMonitor.nowMicros();
        long visionSupplierMicros;
        long addVisionMicros = 0L;

        long visionSupplierStartMicros = SlowCallMonitor.nowMicros();
        boolean hasVisionMeasurement = limelightSubsystem.getLatestVisionMeasurement(latestVisionMeasurement);
        visionSupplierMicros = SlowCallMonitor.nowMicros() - visionSupplierStartMicros;
        boolean hasNewVisionMeasurement = hasVisionMeasurement
                && latestVisionMeasurement.sequenceNumber != lastAppliedVisionMeasurementSequenceNumber;
        if (hasNewVisionMeasurement) {
            long addVisionStartMicros = SlowCallMonitor.nowMicros();
            encapsulatedDrivetrain.addVisionMeasurement(
                    latestVisionMeasurement.robotPoseFldMeters,
                    Microseconds.of(latestVisionMeasurement.captureTimestampPoseEstimateMicros),
                    VecBuilder.fill(
                            latestVisionMeasurement.xDeviationFldMeters,
                            latestVisionMeasurement.yDeviationFldMeters,
                            latestVisionMeasurement.thetaDeviationFldRadians));
            addVisionMicros = SlowCallMonitor.nowMicros() - addVisionStartMicros;
            lastAppliedVisionMeasurementSequenceNumber = latestVisionMeasurement.sequenceNumber;
        }

        long totalMicros = SlowCallMonitor.nowMicros() - periodicStartMicros;
        if (SlowCallMonitor.isSlow(totalMicros, SLOW_PERIODIC_THRESHOLD_MS)
                || SlowCallMonitor.isSlow(visionSupplierMicros, SLOW_VISION_SUPPLIER_THRESHOLD_MS)
                || SlowCallMonitor.isSlow(addVisionMicros, SLOW_ADD_VISION_THRESHOLD_MS)) {
            SlowCallMonitor.print(
                    "SwerveDrivetrainSubsystem.periodic",
                    totalMicros,
                    String.format(
                            "visionSupplier=%.3f ms addVisionMeasurement=%.3f ms hasVision=%s hasNewVision=%s",
                            SlowCallMonitor.toMillis(visionSupplierMicros),
                            SlowCallMonitor.toMillis(addVisionMicros),
                            Boolean.toString(hasVisionMeasurement),
                            Boolean.toString(hasNewVisionMeasurement)));
        }
    }
    
    public void applyRequest(Supplier<SwerveRequest> request) {
        encapsulatedDrivetrain.applyRequest(request);
    }

    public CommandSwerveDrivetrain getEncapsulate() {
        return encapsulatedDrivetrain;
    }
}
