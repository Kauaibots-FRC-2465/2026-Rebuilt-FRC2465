package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.utility.SlowCallMonitor;

public class LimelightSubsystem extends SubsystemBase {
    private static final String LIMELIGHT_NAME = "limelight";
    private static final double SLOW_PERIODIC_THRESHOLD_MS = 4.0;
    private static final double SLOW_STDDEVS_THRESHOLD_MS = 2.0;
    private static final double SLOW_ORIENTATION_THRESHOLD_MS = 1.0;
    private static final double SLOW_POSE_FETCH_THRESHOLD_MS = 2.0;

    DoubleSupplier externalHeadingDegreesSupplier;
    PoseEstimate currentPoseEstimate;
    private double lastHeartbeat = Double.NaN;
    private double lastProcessedPoseTimestampSeconds = Double.NaN;
    
    public LimelightSubsystem(DoubleSupplier externalHeadingDegreesSupplier, boolean useMT2, int[] validIDs) {
        this.externalHeadingDegreesSupplier=externalHeadingDegreesSupplier;
        LimelightHelpers.setFiducialIDFiltersOverride(LIMELIGHT_NAME, validIDs);
        LimelightHelpers.SetIMUMode(LIMELIGHT_NAME, 0);
        this.useMT2 = useMT2;
    }

    public Supplier<Pose2d> getPose2dSupplier() {
        return ()-> {
            return currentPoseEstimate == null ? null : currentPoseEstimate.pose;
        };
    }

    public Supplier<Time> getPose2dTimestampSupplier() {
        return ()-> {
            return currentPoseEstimate == null
                    ? Seconds.zero()
                    : Seconds.of(currentPoseEstimate.timestampSeconds);
        };
    }
    
    public BooleanSupplier getIsValidSupplier() {
        return ()-> {
            if(currentPoseEstimate == null) return false;
            if(currentPoseEstimate.tagCount==0) return false;
            if(currentPoseEstimate.tagCount>1) return true;
            if (currentPoseEstimate.rawFiducials == null || currentPoseEstimate.rawFiducials.length == 0) return false;
            if(currentPoseEstimate.rawFiducials[0].ambiguity<0.2) return true;
            return false;
        }; 
    }

    public DoubleSupplier getXDeviationSupplier() {
        return ()-> xDeviation;
    }

    public DoubleSupplier getYDeviationSupplier() {
        return ()-> yDeviation;
    }

    public DoubleSupplier getThetaDeviationSupplier() {
        return ()-> yawDeviation;
    }
    
    private double xDeviation, yDeviation, yawDeviation;
    private boolean useMT2;

    public void useMT1() {
        useMT2 = false;
    }

    public void useMT2() {
        useMT2 = true;
    }

    @Override
    public void periodic() {
        long periodicStartMicros = SlowCallMonitor.nowMicros();
        long stddevsMicros = 0L;
        long orientationMicros = 0L;
        long poseFetchMicros = 0L;
        boolean frameChanged = false;
        boolean poseChanged = false;

        double heartbeat = LimelightHelpers.getLimelightNTDouble(LIMELIGHT_NAME, "hb");
        if (Double.compare(heartbeat, lastHeartbeat) != 0) {
            frameChanged = true;
            lastHeartbeat = heartbeat;

            if (useMT2) {
                long orientationStartMicros = SlowCallMonitor.nowMicros();
                LimelightHelpers.setRobotOrientation(
                        LIMELIGHT_NAME,
                        externalHeadingDegreesSupplier.getAsDouble(),
                        0,
                        0,
                        0,
                        0,
                        0);
                orientationMicros = SlowCallMonitor.nowMicros() - orientationStartMicros;
            }

            long poseFetchStartMicros = SlowCallMonitor.nowMicros();
            currentPoseEstimate = useMT2
                    ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_NAME)
                    : LimelightHelpers.getBotPoseEstimate_wpiBlue(LIMELIGHT_NAME);
            poseFetchMicros = SlowCallMonitor.nowMicros() - poseFetchStartMicros;

            if (currentPoseEstimate != null
                    && Double.compare(currentPoseEstimate.timestampSeconds, lastProcessedPoseTimestampSeconds) != 0) {
                poseChanged = true;
                lastProcessedPoseTimestampSeconds = currentPoseEstimate.timestampSeconds;

                // MegaTag Standard Deviations [0=MT1x, 1=MT1y, 2=MT1z, 3=MT1roll, 4=MT1pitch, 5=MT1Yaw, 6=MT2x, 7=MT2y, 8=MT2z, 9=MT2roll, 10=MT2pitch, 11=MT2yaw]
                long stddevsStartMicros = SlowCallMonitor.nowMicros();
                double[] stddevs = LimelightHelpers.getLimelightNTDoubleArray(LIMELIGHT_NAME, "stddevs");
                stddevsMicros = SlowCallMonitor.nowMicros() - stddevsStartMicros;

                if (!useMT2 && stddevs.length >= 6) {
                    xDeviation = stddevs[0];
                    yDeviation = stddevs[1];
                    yawDeviation = Math.toRadians(stddevs[5]);
                } else if (useMT2 && stddevs.length >= 12) {
                    xDeviation = stddevs[6];
                    yDeviation = stddevs[7];
                    yawDeviation = Math.toRadians(stddevs[11]);
                }
            }
        }

        long totalMicros = SlowCallMonitor.nowMicros() - periodicStartMicros;
        if (SlowCallMonitor.isSlow(totalMicros, SLOW_PERIODIC_THRESHOLD_MS)
                || SlowCallMonitor.isSlow(stddevsMicros, SLOW_STDDEVS_THRESHOLD_MS)
                || SlowCallMonitor.isSlow(orientationMicros, SLOW_ORIENTATION_THRESHOLD_MS)
                || SlowCallMonitor.isSlow(poseFetchMicros, SLOW_POSE_FETCH_THRESHOLD_MS)) {
            SlowCallMonitor.print(
                    "LimelightSubsystem.periodic",
                    totalMicros,
                    String.format(
                            "stddevs=%.3f ms setOrientation=%.3f ms poseFetch=%.3f ms useMT2=%s frameChanged=%s poseChanged=%s poseNull=%s",
                            SlowCallMonitor.toMillis(stddevsMicros),
                            SlowCallMonitor.toMillis(orientationMicros),
                            SlowCallMonitor.toMillis(poseFetchMicros),
                            Boolean.toString(useMT2),
                            Boolean.toString(frameChanged),
                            Boolean.toString(poseChanged),
                            Boolean.toString(currentPoseEstimate == null)));
        }
    }
}
