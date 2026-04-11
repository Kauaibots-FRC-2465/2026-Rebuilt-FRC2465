package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
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
    private static final int MT1_BOOTSTRAP_REQUIRED_CONSISTENT_FRAMES = 3;
    private static final double MT1_BOOTSTRAP_MAX_TRANSLATION_DELTA_METERS = 0.35;
    private static final double MT1_BOOTSTRAP_MAX_HEADING_DELTA_DEGREES = 10.0;
    private static final double MT1_BOOTSTRAP_MAX_SINGLE_TAG_AMBIGUITY = 0.1;

    public static final class VisionMeasurement {
        public Pose2d robotPoseFldMeters = new Pose2d();
        public long captureTimestampPoseEstimateMicros = -1L;
        public double xDeviationFldMeters = Double.NaN;
        public double yDeviationFldMeters = Double.NaN;
        public double thetaDeviationFldRadians = Double.NaN;
        public long sequenceNumber = -1L;
        private boolean valid = false;

        public boolean isValid() {
            return valid;
        }

        public void setInvalid() {
            robotPoseFldMeters = new Pose2d();
            captureTimestampPoseEstimateMicros = -1L;
            xDeviationFldMeters = Double.NaN;
            yDeviationFldMeters = Double.NaN;
            thetaDeviationFldRadians = Double.NaN;
            sequenceNumber = -1L;
            valid = false;
        }

        void set(
                Pose2d robotPoseFldMeters,
                long captureTimestampPoseEstimateMicros,
                double xDeviationFldMeters,
                double yDeviationFldMeters,
                double thetaDeviationFldRadians,
                long sequenceNumber) {
            this.robotPoseFldMeters = robotPoseFldMeters;
            this.captureTimestampPoseEstimateMicros = captureTimestampPoseEstimateMicros;
            this.xDeviationFldMeters = xDeviationFldMeters;
            this.yDeviationFldMeters = yDeviationFldMeters;
            this.thetaDeviationFldRadians = thetaDeviationFldRadians;
            this.sequenceNumber = sequenceNumber;
            valid = true;
        }

        public void copyFrom(VisionMeasurement other) {
            if (other == null || !other.valid) {
                setInvalid();
                return;
            }
            set(
                    other.robotPoseFldMeters,
                    other.captureTimestampPoseEstimateMicros,
                    other.xDeviationFldMeters,
                    other.yDeviationFldMeters,
                    other.thetaDeviationFldRadians,
                    other.sequenceNumber);
        }
    }

    private final DoubleSupplier externalHeadingDegreesSupplier;
    private final BooleanSupplier externalHeadingReliableSupplier;
    private final Consumer<Pose2d> mt1SeedPoseConsumer;
    PoseEstimate currentPoseEstimate;
    private double lastHeartbeat = Double.NaN;
    private double lastProcessedPoseEstimateTimestampSeconds = Double.NaN;
    private boolean hasReceivedValidMt2Pose = false;
    private boolean hasSeededHeadingFromMt1 = false;
    private PoseEstimate lastMt1BootstrapCandidate = null;
    private int consecutiveConsistentMt1BootstrapFrames = 0;
    private final VisionMeasurement latestVisionMeasurement = new VisionMeasurement();
    private long nextVisionMeasurementSequenceNumber = 0L;
    
    public LimelightSubsystem(
            DoubleSupplier externalHeadingDegreesSupplier,
            BooleanSupplier externalHeadingReliableSupplier,
            Consumer<Pose2d> mt1SeedPoseConsumer,
            boolean useMT2,
            int[] validIDs) {
        this.externalHeadingDegreesSupplier =
                Objects.requireNonNull(externalHeadingDegreesSupplier, "externalHeadingDegreesSupplier");
        this.externalHeadingReliableSupplier =
                Objects.requireNonNull(externalHeadingReliableSupplier, "externalHeadingReliableSupplier");
        this.mt1SeedPoseConsumer =
                Objects.requireNonNull(mt1SeedPoseConsumer, "mt1SeedPoseConsumer");
        LimelightHelpers.setFiducialIDFiltersOverride(LIMELIGHT_NAME, validIDs);
        LimelightHelpers.SetIMUMode(LIMELIGHT_NAME, 0);
        this.useMT2 = useMT2;
    }

    public Supplier<Pose2d> getPose2dSupplier() {
        return ()-> {
            return currentPoseEstimate == null ? null : currentPoseEstimate.pose;
        };
    }

    public Supplier<Time> getPoseTimestampNtLocalSupplier() {
        return ()-> {
            return currentPoseEstimate == null
                    ? Seconds.zero()
                    : Seconds.of(currentPoseEstimate.timestampNtLocalSeconds);
        };
    }
    
    public BooleanSupplier getIsValidSupplier() {
        return () -> isPoseEstimateValid(currentPoseEstimate);
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

    public boolean getLatestVisionMeasurement(VisionMeasurement out) {
        if (out == null) {
            throw new IllegalArgumentException("VisionMeasurement output must not be null.");
        }
        out.copyFrom(latestVisionMeasurement);
        return out.isValid();
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
                double headingDegrees = externalHeadingDegreesSupplier.getAsDouble();
                LimelightHelpers.setRobotOrientation(
                        LIMELIGHT_NAME,
                        headingDegrees,
                        0,
                        0,
                        0,
                        0,
                        0);
                orientationMicros = SlowCallMonitor.nowMicros() - orientationStartMicros;
            }

            long poseFetchStartMicros = SlowCallMonitor.nowMicros();
            PoseEstimate fetchedPoseEstimate;
            boolean mt1SeededThisFrame = false;
            if (useMT2) {
                PoseEstimate mt2PoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_NAME);
                if (!hasReliableHeading()) {
                    PoseEstimate mt1PoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(LIMELIGHT_NAME);
                    mt1SeededThisFrame = considerMt1Bootstrap(mt1PoseEstimate);
                    fetchedPoseEstimate = null;
                } else {
                    clearMt1BootstrapTracking();
                    fetchedPoseEstimate = mt2PoseEstimate;
                }
                if (!mt1SeededThisFrame && hasReliableHeading() && isPoseEstimateValid(mt2PoseEstimate)) {
                    hasReceivedValidMt2Pose = true;
                    fetchedPoseEstimate = mt2PoseEstimate;
                } else if (!hasReliableHeading()) {
                    fetchedPoseEstimate = null;
                }
            } else {
                clearMt1BootstrapTracking();
                fetchedPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(LIMELIGHT_NAME);
            }
            currentPoseEstimate = fetchedPoseEstimate;
            poseFetchMicros = SlowCallMonitor.nowMicros() - poseFetchStartMicros;

            if (currentPoseEstimate != null
                    && Double.compare(
                                    currentPoseEstimate.timestampNtLocalSeconds,
                                    lastProcessedPoseEstimateTimestampSeconds)
                            != 0) {
                poseChanged = true;
                lastProcessedPoseEstimateTimestampSeconds =
                        currentPoseEstimate.timestampNtLocalSeconds;

                // MegaTag Standard Deviations [0=MT1x, 1=MT1y, 2=MT1z, 3=MT1roll, 4=MT1pitch, 5=MT1Yaw, 6=MT2x, 7=MT2y, 8=MT2z, 9=MT2roll, 10=MT2pitch, 11=MT2yaw]
                long stddevsStartMicros = SlowCallMonitor.nowMicros();
                double[] stddevs = LimelightHelpers.getLimelightNTDoubleArray(LIMELIGHT_NAME, "stddevs");
                stddevsMicros = SlowCallMonitor.nowMicros() - stddevsStartMicros;
                xDeviation = Double.NaN;
                yDeviation = Double.NaN;
                yawDeviation = Double.NaN;

                if (!useMT2 && stddevs.length >= 6) {
                    xDeviation = stddevs[0];
                    yDeviation = stddevs[1];
                    yawDeviation = Math.toRadians(stddevs[5]);
                } else if (useMT2 && stddevs.length >= 12) {
                    xDeviation = stddevs[6];
                    yDeviation = stddevs[7];
                    yawDeviation = Math.toRadians(stddevs[11]);
                }

                if (Double.isFinite(xDeviation)
                        && Double.isFinite(yDeviation)
                        && Double.isFinite(yawDeviation)) {
                    long captureTimestampPoseEstimateMicros =
                            poseEstimateTimestampSecondsToMicros(currentPoseEstimate.timestampNtLocalSeconds);
                    latestVisionMeasurement.set(
                            currentPoseEstimate.pose,
                            captureTimestampPoseEstimateMicros,
                            xDeviation,
                            yDeviation,
                            yawDeviation,
                            nextVisionMeasurementSequenceNumber++);
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

    private boolean hasReliableHeading() {
        return externalHeadingReliableSupplier.getAsBoolean()
                || hasReceivedValidMt2Pose
                || hasSeededHeadingFromMt1;
    }

    public static long poseEstimateTimestampSecondsToMicros(double timestampSeconds) {
        return Math.round(timestampSeconds * 1_000_000.0);
    }

    private static boolean isPoseEstimateValid(PoseEstimate poseEstimate) {
        return isPoseEstimateValid(poseEstimate, 0.2);
    }

    private static boolean isPoseEstimateValid(PoseEstimate poseEstimate, double singleTagAmbiguityThreshold) {
        if (poseEstimate == null || poseEstimate.tagCount == 0) {
            return false;
        }
        if (poseEstimate.tagCount > 1) {
            return true;
        }
        if (poseEstimate.rawFiducials == null || poseEstimate.rawFiducials.length == 0) {
            return false;
        }
        return poseEstimate.rawFiducials[0].ambiguity < singleTagAmbiguityThreshold;
    }

    private boolean considerMt1Bootstrap(PoseEstimate mt1PoseEstimate) {
        if (!isPoseEstimateValid(mt1PoseEstimate, MT1_BOOTSTRAP_MAX_SINGLE_TAG_AMBIGUITY)) {
            clearMt1BootstrapTracking();
            return false;
        }

        if (lastMt1BootstrapCandidate == null
                || !arePoseEstimatesConsistent(lastMt1BootstrapCandidate, mt1PoseEstimate)) {
            consecutiveConsistentMt1BootstrapFrames = 1;
        } else {
            consecutiveConsistentMt1BootstrapFrames++;
        }
        lastMt1BootstrapCandidate = mt1PoseEstimate;

        if (consecutiveConsistentMt1BootstrapFrames < MT1_BOOTSTRAP_REQUIRED_CONSISTENT_FRAMES) {
            return false;
        }

        mt1SeedPoseConsumer.accept(mt1PoseEstimate.pose);
        hasSeededHeadingFromMt1 = true;
        clearMt1BootstrapTracking();
        return true;
    }

    private void clearMt1BootstrapTracking() {
        lastMt1BootstrapCandidate = null;
        consecutiveConsistentMt1BootstrapFrames = 0;
    }

    private static boolean arePoseEstimatesConsistent(PoseEstimate prior, PoseEstimate current) {
        if (prior == null || current == null) {
            return false;
        }
        double translationDeltaMeters =
                prior.pose.getTranslation().getDistance(current.pose.getTranslation());
        double headingDeltaDegrees =
                Math.abs(prior.pose.getRotation().minus(current.pose.getRotation()).getDegrees());
        return translationDeltaMeters <= MT1_BOOTSTRAP_MAX_TRANSLATION_DELTA_METERS
                && headingDeltaDegrees <= MT1_BOOTSTRAP_MAX_HEADING_DELTA_DEGREES;
    }
}
