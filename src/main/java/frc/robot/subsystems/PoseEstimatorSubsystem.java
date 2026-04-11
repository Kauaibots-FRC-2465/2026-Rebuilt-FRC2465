package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Microseconds;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.RingBuffer;
import frc.robot.utility.SlowCallMonitor;

/**
 * A high-fidelity, latency-compensated pose estimator designed to serve as the
 * authoritative "Source of Truth" for the robot's field position.
 *
 * <p>
 * <b>Architecture:</b>
 * This subsystem implements a custom sensor fusion algorithm that combines
 * high-frequency
 * odometry (e.g., from a Deadwheel/Pinpoint system) with lower-frequency,
 * high-latency
 * vision updates (e.g., AprilTags). Unlike standard Extended Kalman Filters
 * (EKF),
 * this class maintains a separate "Odometry Origin" and "Fused Origin" to apply
 * vision corrections as a differential offset to the rigid odometry path.
 *
 * <p>
 * <b>Key Features:</b>
 * <ul>
 * <li><b>Latency Compensation:</b> Uses a {@link RingBuffer} to store
 * historical odometry
 * snapshots. Incoming vision measurements are matched against the exact
 * odometry pose
 * at the time of capture (interpolated to the microsecond) to calculate
 * accurate
 * corrections without "smearing" due to robot motion during processing
 * time.</li>
 *
 * <li><b>Geometric Error Modeling:</b> Instead of a static coDeviation matrix,
 * this class
 * calculates dynamic error bounds ({@code xVarianceBound},
 * {@code thetaVarianceBound}) based
 * on physical phenomena like Versine Loss (drift due to heading error) and
 * Cross-Contamination
 * (lateral slip bleeding into longitudinal error).</li>
 *
 * <li><b>Downstream Integration:</b> This class is intended to feed a
 * "Federated" filter
 * architecture. It exports a {@link Supplier} that provides a timestamped,
 * extrapolated
 * pose with Deviations to a downstream drivetrain controller (e.g., CTRE
 * Swerve). It does not
 * directly control motors but acts as the master reference signal that the
 * drivetrain loops
 * synchronize to.</li>
 * </ul>
 *
 * <p>
 * <b>Usage Warning:</b>
 * The provided odometry source is expected to provide a new field-centric
 * odometry solution
 * each robot cycle. If the source is unable to provide a fresh solution, it
 * must return null.
 * Occasional drops are tolerated, but will cause a reset of the internal
 * odometry buffer.
 * While the odometry buffer is insufficiently full, vision updates are dropped.
 * Fused
 * poses will be unavailable unless there are at least two successful updates in
 * a row.
 * Odometry and vision updates must occur in their update(), and calls to their
 * supplier methods
 * must return the latest sample which cannot change as this code runs. (That
 * is, do not
 * update odometry in a supplier method.)
 */
public class PoseEstimatorSubsystem extends SubsystemBase {
    private static final double SLOW_PERIODIC_THRESHOLD_MS = 4.0;
    private static final double SLOW_ODOMETRY_THRESHOLD_MS = 2.0;
    private static final double SLOW_VISION_THRESHOLD_MS = 2.0;
    private static final double SLOW_PREDICTION_THRESHOLD_MS = 3.0;
    private static final int MAX_CONSECUTIVE_INVALID_ODOMETRY_CYCLES_BEFORE_CLEAR = 3;
    private static final long MAX_PREDICTION_ODOMETRY_GAP_MICROS = 200_000L;

    private static class PoseDashboardPublisher {
        private final StructPublisher<Pose2d> posePublisher;
        private final DoubleArrayPublisher fieldPosePublisher;
        private final StringPublisher fieldTypePublisher;
        private final double[] poseArray = new double[3];

        PoseDashboardPublisher(NetworkTable parentTable, String name) {
            posePublisher = parentTable.getStructTopic(name, Pose2d.struct).publish();
            NetworkTable fieldTable = parentTable.getSubTable(name);
            fieldPosePublisher = fieldTable.getDoubleArrayTopic("robotPose").publish();
            fieldTypePublisher = fieldTable.getStringTopic(".type").publish();
        }

        void publish(Pose2d pose, String signalLogKey) {
            // Debug dashboard telemetry disabled to reduce NetworkTables traffic.
            // posePublisher.set(pose);
            // fieldTypePublisher.set("Field2d");
            // poseArray[0] = pose.getX();
            // poseArray[1] = pose.getY();
            // poseArray[2] = pose.getRotation().getDegrees();
            // fieldPosePublisher.set(poseArray);
            // SignalLogger.writeStruct(signalLogKey, Pose2d.struct, pose);
        }
    }

    private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private final NetworkTable odometryTable = ntInstance.getTable("PoseEstimator");
    private final NetworkTable debugTable = odometryTable.getSubTable("Debug");
    private final PoseDashboardPublisher newOdometryPosePublisher =
            new PoseDashboardPublisher(debugTable, "NewOdometryPose");
    private final PoseDashboardPublisher visionPosePublisher =
            new PoseDashboardPublisher(debugTable, "visionPose");
    private final PoseDashboardPublisher odometryAnchorPublisher =
            new PoseDashboardPublisher(debugTable, "odometryAnchor");
    private final PoseDashboardPublisher fusedAnchorPublisher =
            new PoseDashboardPublisher(debugTable, "fusedAnchor");
    private final PoseDashboardPublisher latestOdometryPosePublisher =
            new PoseDashboardPublisher(debugTable, "latestOdometryPose");
    private final PoseDashboardPublisher priorOdometryPosePublisher =
            new PoseDashboardPublisher(debugTable, "priorOdometryPose");
    private final PoseDashboardPublisher odometryNowPublisher =
            new PoseDashboardPublisher(debugTable, "odometryNow");
    private final PoseDashboardPublisher fusedPosePublisher =
            new PoseDashboardPublisher(debugTable, "fusedPose");
    private final IntegerPublisher visionTimestampNtLocalPublisher =
            debugTable.getIntegerTopic("visionTimestamp").publish();
    private final IntegerPublisher odometryHistorySizePublisher =
            debugTable.getIntegerTopic("odometryHistory size").publish();
    private final IntegerPublisher dtPublisher =
            debugTable.getIntegerTopic("dt from before to after odometry record").publish();
    private final DoublePublisher xKalmanGainPublisher =
            debugTable.getDoubleTopic("xKalmanGain").publish();
    private final DoublePublisher yKalmanGainPublisher =
            debugTable.getDoubleTopic("yKalmanGain").publish();
    private final DoublePublisher thetaKalmanGainPublisher =
            debugTable.getDoubleTopic("thetaKalmanGain").publish();
    private final IntegerPublisher newOdometryPoseTimestampPublisher =
            debugTable.getIntegerTopic("NewOdometryPoseTimestamp").publish();
    private final DoublePublisher xVarianceOdometryPublisher =
            debugTable.getDoubleTopic("variance x, odometry").publish();
    private final DoublePublisher yVarianceOdometryPublisher =
            debugTable.getDoubleTopic("variance y, odometry").publish();
    private final DoublePublisher hVarianceOdometryPublisher =
            debugTable.getDoubleTopic("variance h, odometry").publish();
    private final IntegerPublisher latestOdometryTimestampPublisher =
            debugTable.getIntegerTopic("latestOdometryTimestamp").publish();
    private final IntegerPublisher priorOdometryTimestampPublisher =
            debugTable.getIntegerTopic("priorOdometryTimestamp").publish();
    private final IntegerPublisher timeSinceLatestOdometryPublisher =
            debugTable.getIntegerTopic("timeSinceLatestOdometry").publish();
    private final IntegerPublisher timeBetweenLatestAndPriorOdometryPublisher =
            debugTable.getIntegerTopic("timeBetweenLatestAndPriorOdometry").publish();
    private final DoublePublisher twistFractionToGetToNowPublisher =
            debugTable.getDoubleTopic("twistFractionToGetToNow").publish();
    private final DoublePublisher xVarianceVisionPublisher =
            debugTable.getDoubleTopic("variance x, vision").publish();
    private final DoublePublisher yVarianceVisionPublisher =
            debugTable.getDoubleTopic("variance y, vision").publish();
    private final DoublePublisher thetaVarianceVisionPublisher =
            debugTable.getDoubleTopic("variance h, vision").publish();

    private class OdometryData {
        Pose2d pose;
        Long timestampFpgaMicros;
    }

    // Fusion typically happens between indexes 4 and 8 
    RingBuffer<OdometryData> odometryHistory = new RingBuffer<>(20, () -> new OdometryData());

    public static class Configuration {
        public Supplier<Pose2d> odometryPose;
        public Supplier<Time> odometryTimestampFpga;
        public BooleanSupplier odometryValid;
        public double odoLongitudinalDeviationPerDistance;
        public double odoLateralDeviationPerDistance;
        public double odoHeadingDeviationPerDistance;
        public double odoLongitudinalDeviationPerRadian;
        public double odoLateralDeviationPerRadian;
        public double odoHeadingDeviationPerRadian;
        public Supplier<Pose2d> visionPose;
        public Supplier<Time> visionTimestampNtLocal;
        public BooleanSupplier visionIsValid;
        public DoubleSupplier visionXDeviation;
        public DoubleSupplier visionYDeviation;
        public DoubleSupplier visionThetaDeviation;
        public double initialXDeviation;
        public double initialYDeviation;
        public double initialThetaDeviation;
    }

    public static class PredictedFusedState {
        public double xMeters;
        public double yMeters;
        public double headingRadians;
        public double vxMetersPerSecond;
        public double vyMetersPerSecond;
        public double omegaRadiansPerSecond;
        public long timestampFpgaMicros;
        private boolean valid;

        public boolean isValid() {
            return valid;
        }

        public void setInvalid() {
            xMeters = Double.NaN;
            yMeters = Double.NaN;
            headingRadians = Double.NaN;
            vxMetersPerSecond = Double.NaN;
            vyMetersPerSecond = Double.NaN;
            omegaRadiansPerSecond = Double.NaN;
            timestampFpgaMicros = -1L;
            valid = false;
        }

        public void set(
                double xMeters,
                double yMeters,
                double headingRadians,
                double vxMetersPerSecond,
                double vyMetersPerSecond,
                double omegaRadiansPerSecond,
                long timestampFpgaMicros) {
            this.xMeters = xMeters;
            this.yMeters = yMeters;
            this.headingRadians = headingRadians;
            this.vxMetersPerSecond = vxMetersPerSecond;
            this.vyMetersPerSecond = vyMetersPerSecond;
            this.omegaRadiansPerSecond = omegaRadiansPerSecond;
            this.timestampFpgaMicros = timestampFpgaMicros;
            valid = true;
        }

        void copyFrom(PredictedFusedState other) {
            if (other == null || !other.valid) {
                setInvalid();
                return;
            }
            set(
                    other.xMeters,
                    other.yMeters,
                    other.headingRadians,
                    other.vxMetersPerSecond,
                    other.vyMetersPerSecond,
                    other.omegaRadiansPerSecond,
                    other.timestampFpgaMicros);
        }
    }

    public PoseEstimatorSubsystem(CommandSwerveDrivetrain ctreDrivetrain, Configuration config) {
        this.ctreDrivetrain = ctreDrivetrain;
        useCtrePoseFacade = ctreDrivetrain != null;
        odometryPoseSupplier = useCtrePoseFacade ? () -> ctreDrivetrain.getState().Pose : config.odometryPose;
        odometryTimestampFpgaSupplier = useCtrePoseFacade
                ? () -> Microseconds.of(currentTimeSecondsToFpgaMicros(ctreDrivetrain.getState().Timestamp))
                : config.odometryTimestampFpga;
        odometryIsValidSupplier = useCtrePoseFacade
                ? () -> isCtreStateValid(ctreDrivetrain.getStateCopy())
                : config.odometryValid;
        odoLongitudinalDeviationPerDistance = config.odoLongitudinalDeviationPerDistance;
        odoLateralDeviationPerDistance = config.odoLateralDeviationPerDistance;
        odoHeadingDeviationPerDistance = config.odoHeadingDeviationPerDistance;
        odoLongitudinalDeviationPerRadian = config.odoLongitudinalDeviationPerRadian;
        odoLateralDeviationPerRadian = config.odoLateralDeviationPerRadian;
        odoHeadingDeviationPerRadian = config.odoHeadingDeviationPerRadian;
        visionPoseSupplier = config.visionPose;
        visionTimestampNtLocalSupplier = config.visionTimestampNtLocal;
        visionIsValidSupplier = config.visionIsValid;
        visionXDeviationSupplier = config.visionXDeviation;
        visionYDeviationSupplier = config.visionYDeviation;
        visionThetaDeviationSupplier = config.visionThetaDeviation;
        initialXVarianceOdometry = config.initialXDeviation * config.initialXDeviation;
        initialYVarianceOdometry = config.initialYDeviation * config.initialYDeviation;
        initialThetaVarianceOdometry = config.initialThetaDeviation * config.initialThetaDeviation;
        xVarianceOdometry = initialXVarianceOdometry;
        yVarianceOdometry = initialYVarianceOdometry;
        thetaVarianceOdometry = initialThetaVarianceOdometry;
        fusedAnchor = odometryAnchor = new Pose2d();
    }

    public PoseEstimatorSubsystem(Configuration config) {
        this(null, config);
    }

    Pose2d odometryAnchor;
    Pose2d fusedAnchor;

    private final Supplier<Pose2d> odometryPoseSupplier;
    private final Supplier<Time> odometryTimestampFpgaSupplier;
    private final BooleanSupplier odometryIsValidSupplier;
    private final double odoLongitudinalDeviationPerDistance;
    private final double odoLateralDeviationPerDistance;
    private final double odoHeadingDeviationPerDistance;
    private final double odoLongitudinalDeviationPerRadian;
    private final double odoLateralDeviationPerRadian;
    private final double odoHeadingDeviationPerRadian;
    private final DoubleSupplier visionXDeviationSupplier;
    private final DoubleSupplier visionYDeviationSupplier;
    private final DoubleSupplier visionThetaDeviationSupplier;
    private final double initialXVarianceOdometry;
    private final double initialYVarianceOdometry;
    private final double initialThetaVarianceOdometry;
    private Supplier<Pose2d> visionPoseSupplier;
    private Supplier<Time> visionTimestampNtLocalSupplier;
    private BooleanSupplier visionIsValidSupplier;
    private Long lastFusedVisionTimestampNtLocalMicros;
    private long predictionCacheEpoch = 0L;
    private long cachedPredictionEpoch = -1L;
    private double cachedPredictionLookaheadSeconds = Double.NaN;
    private final PredictedFusedState cachedPredictedState = new PredictedFusedState();
    private String lastPredictionOutcome = "neverRun";
    private int consecutiveInvalidOdometryCycles = 0;
    private boolean lastOdometryValid = false;
    private int lastPredictionHistorySize = 0;
    private long lastPredictionLatestOdometryTimestampMicros = -1L;
    private long lastPredictionPriorOdometryTimestampMicros = -1L;
    private long lastPredictionTimeSinceLatestOdometryMicros = -1L;
    private long lastPredictionTimeBetweenLatestAndPriorOdometryMicros = -1L;
    private String lastOdometryHistoryResetReason = "never";
    private final CommandSwerveDrivetrain ctreDrivetrain;
    private final boolean useCtrePoseFacade;
    private Pose2d pendingResetPoseFldMetersRadians;
    private long pendingResetTimestampFpgaMicros = -1L;

    private final Supplier<Pose2d> fusedPoseSupplier = new Supplier<Pose2d>() {
        private final PredictedFusedState predictedState = new PredictedFusedState();

        @Override
        public Pose2d get() {
            if (!getPredictedFusedState(0.0, predictedState)) {
                return null;
            }
            return new Pose2d(
                    predictedState.xMeters,
                    predictedState.yMeters,
                    Rotation2d.fromRadians(predictedState.headingRadians));
        }
    };

    double xVarianceOdometry;
    double yVarianceOdometry;
    double thetaVarianceOdometry;

    Pose2d priorPose;
    Rotation2d priorHeading;

    @Override
    public void periodic() {
        if (useCtrePoseFacade) {
            periodicCtrePoseFacade();
            return;
        }
        invalidatePredictionCache();
        long periodicStartMicros = SlowCallMonitor.nowMicros();
        long odometryMicros = 0L;
        long visionMicros = 0L;
        boolean odometryValid = odometryIsValidSupplier.getAsBoolean();
        boolean visionValid = visionIsValidSupplier.getAsBoolean();
        lastOdometryValid = odometryValid;

        if (odometryValid) {
            consecutiveInvalidOdometryCycles = 0;
            long odometryStartMicros = SlowCallMonitor.nowMicros();
            updateOdometry();
            odometryMicros = SlowCallMonitor.nowMicros() - odometryStartMicros;
        } else {
            consecutiveInvalidOdometryCycles++;
            if (consecutiveInvalidOdometryCycles >= MAX_CONSECUTIVE_INVALID_ODOMETRY_CYCLES_BEFORE_CLEAR) {
                odometryHistory.clear();
                priorPose = null;
                priorHeading = null;
                lastOdometryHistoryResetReason = "consecutiveInvalidOdometry";
            }
        }

        if (visionValid) {
            long visionStartMicros = SlowCallMonitor.nowMicros();
            fuseVision();
            visionMicros = SlowCallMonitor.nowMicros() - visionStartMicros;
        }

        long totalMicros = SlowCallMonitor.nowMicros() - periodicStartMicros;
        if (SlowCallMonitor.isSlow(totalMicros, SLOW_PERIODIC_THRESHOLD_MS)
                || SlowCallMonitor.isSlow(odometryMicros, SLOW_ODOMETRY_THRESHOLD_MS)
                || SlowCallMonitor.isSlow(visionMicros, SLOW_VISION_THRESHOLD_MS)) {
            SlowCallMonitor.print(
                    "PoseEstimatorSubsystem.periodic",
                    totalMicros,
                    String.format(
                            "updateOdometry=%.3f ms fuseVision=%.3f ms odometryValid=%s visionValid=%s historySize=%d",
                            SlowCallMonitor.toMillis(odometryMicros),
                            SlowCallMonitor.toMillis(visionMicros),
                            Boolean.toString(odometryValid),
                            Boolean.toString(visionValid),
                            odometryHistory.size()));
        }
    }

    /**
     * Integrates a new vision measurement to correct the field-relative origin of
     * the robot's odometry.
     * *
     * <p>
     * This method implements a <b>Latent Differential Update</b> strategy:
     * <ol>
     * <li><b>Latency Compensation:</b> It searches the {@code odometryHistory}
     * buffer to find
     * the exact robot pose at the moment the vision image was captured
     * ({@code visionTimestampNtLocal}).
     * It linearly interpolates between history points to handle sub-tick timing
     * differences.</li>
     * *
     * <li><b>Correction Calculation:</b> It compares this historical odometry pose
     * against the
     * supplied {@code visionPose}. The difference defines a new "anchor point"
     * ({@code fusedAnchor})
     * that realigns the odometry coordinate system to match the vision data.</li>
     * *
     * <li><b>Deviation Update:</b> It applies a simplified Kalman update to the
     * error bounds
     * ({@code xVarianceBound}, {@code yVarianceBound}, {@code thetaVarianceBound}).
     * If the vision measurement is trusted (low Deviation), the system's global
     * error bound shrinks,
     * shifting the fused origin significantly towards the vision pose. If the
     * vision is noisy,
     * the correction is weighted less.</li>
     * </ol>
     * *
     * <p>
     * <b>Prerequisites:</b>
     * <ul>
     * <li>{@code odometryHistory} must contain at least two snapshots.</li>
     * <li>{@code visionTimestampNtLocal} must fall within the time range covered by the
     * history buffer.
     * Measurements older than the buffer or newer than the latest update are
     * discarded.</li>
     * </ul>
     */
    private void fuseVision() {
        OdometryData newerOdometryData = null;
        OdometryData olderOdometryData = null;
        OdometryData exactOdometryData = null;
        Long visionTimestampNtLocalMicros = toMicroseconds(visionTimestampNtLocalSupplier.get());
        if (visionTimestampNtLocalMicros.equals(lastFusedVisionTimestampNtLocalMicros)) {
            return;
        }
        // Debug dashboard telemetry disabled to reduce NetworkTables traffic.
        // visionTimestampNtLocalPublisher.set(visionTimestampNtLocalMicros);
        // SignalLogger.writeInteger("PoseEstimator/Debug/visionTimestamp", visionTimestampNtLocalMicros);
        // odometryHistorySizePublisher.set(odometryHistory.size());
        // SignalLogger.writeInteger("PoseEstimator/Debug/odometryHistory size", odometryHistory.size());

        if (odometryHistory.size() < 2)
            return;
        for (int index = 0; index < odometryHistory.size(); ++index) {
            Long odometryTimestampFpgaMicros = odometryHistory.get(index).timestampFpgaMicros;
            if (odometryTimestampFpgaMicros.equals(visionTimestampNtLocalMicros)) {
                exactOdometryData = odometryHistory.get(index);
                break;
            }
            if (odometryTimestampFpgaMicros >= visionTimestampNtLocalMicros) {
                newerOdometryData = odometryHistory.get(index);
            }
            if (odometryTimestampFpgaMicros <= visionTimestampNtLocalMicros) {
                olderOdometryData = odometryHistory.get(index);
                break;
            }
        }
        if (exactOdometryData != null) {
            newerOdometryData = exactOdometryData;
            olderOdometryData = exactOdometryData;
        }
        // Odometry is much faster than vision, which should never be newer
        // The apriltag reading must exist inside our odometry history
        if (newerOdometryData == null || olderOdometryData == null) {
            return;
        }
        long dtFpgaMicros =
                newerOdometryData.timestampFpgaMicros - olderOdometryData.timestampFpgaMicros;
        // dtPublisher.set(dt);
        // SignalLogger.writeInteger("PoseEstimator/Debug/dt from before to after odometry record", dt);
        if (dtFpgaMicros < 0) {
            throw new IllegalStateException(
                    "CRITICAL: Odometry timestamps are non-monotonic or duplicate. dt=" +
                            dtFpgaMicros +
                            ". Fix upstream sensor/loop timing.");
        }
        Pose2d visionPose = visionPoseSupplier.get();
        visionPosePublisher.publish(visionPose, "PoseEstimator/Debug/visionPose");
        Pose2d previousOdometryAnchor = odometryAnchor;
        Pose2d visionTimeOdometryAnchor;
        if (dtFpgaMicros == 0) {
            visionTimeOdometryAnchor = olderOdometryData.pose;
        } else {
            double alpha =
                    (double)
                                    (visionTimestampNtLocalMicros
                                            - olderOdometryData.timestampFpgaMicros)
                            / dtFpgaMicros;
            visionTimeOdometryAnchor = olderOdometryData.pose.interpolate(newerOdometryData.pose, alpha);
        }
        odometryAnchorPublisher.publish(visionTimeOdometryAnchor, "PoseEstimator/Debug/odometryAnchor");
        double xDeviationSupplier = Math.max(0.005, visionXDeviationSupplier.getAsDouble()); // No better than .5 cm
        double xVarianceVision = xDeviationSupplier * xDeviationSupplier;
        // xVarianceVisionPublisher.set(xVarianceVision);
        double yDeviationVision = Math.max(0.005, visionYDeviationSupplier.getAsDouble()); // No better than .5 cm
        double yVarianceVision = yDeviationVision * yDeviationVision;
        // yVarianceVisionPublisher.set(yVarianceVision);
        double thetaDeviationVision = Math.max(0.00174533 /* radians */, visionThetaDeviationSupplier.getAsDouble());  // No better than .1 deg
        double thetaVarianceVision = thetaDeviationVision * thetaDeviationVision;
        if(Double.isNaN(xVarianceVision)
        ||Double.isNaN(yVarianceVision)
        ||Double.isNaN(thetaVarianceVision)) return;

        // thetaVarianceVisionPublisher.set(thetaVarianceVision);
        double xKalmanGain = xVarianceOdometry / (xVarianceOdometry + xVarianceVision);
        double yKalmanGain = yVarianceOdometry / (yVarianceOdometry + yVarianceVision);
        double thetaKalmanGain = thetaVarianceOdometry / (thetaVarianceOdometry + thetaVarianceVision);
        // xKalmanGainPublisher.set(xKalmanGain);
        // yKalmanGainPublisher.set(yKalmanGain);
        // thetaKalmanGainPublisher.set(thetaKalmanGain);
        // SignalLogger.writeDouble("PoseEstimator/Debug/xKalmanGain", xKalmanGain);
        // SignalLogger.writeDouble("PoseEstimator/Debug/yKalmanGain", yKalmanGain);
        // SignalLogger.writeDouble("PoseEstimator/Debug/thetaKalmanGain", thetaKalmanGain);
        Pose2d predictedFusedPoseAtVision = fusedAnchor.plus(visionTimeOdometryAnchor.minus(previousOdometryAnchor));
        Rotation2d predictedRotation = predictedFusedPoseAtVision.getRotation();
        double ThetaDifference = visionPose.getRotation().minus(predictedRotation).getRadians();
        fusedAnchor = new Pose2d(
                predictedFusedPoseAtVision.getX() * (1 - xKalmanGain) + visionPose.getX() * (xKalmanGain),
                predictedFusedPoseAtVision.getY() * (1 - yKalmanGain) + visionPose.getY() * (yKalmanGain),
                predictedRotation.plus(new Rotation2d(ThetaDifference * thetaKalmanGain)));
        fusedAnchorPublisher.publish(fusedAnchor, "PoseEstimator/Debug/fusedAnchor");
        odometryAnchor = visionTimeOdometryAnchor;
        xVarianceOdometry = (1.0 - xKalmanGain) * xVarianceOdometry;
        xVarianceOdometry = Math.max(xVarianceOdometry, .005*.005); // No better than 1/2 cm
        yVarianceOdometry = (1.0 - yKalmanGain) * yVarianceOdometry;
        yVarianceOdometry = Math.max(yVarianceOdometry, .005*.005);
        thetaVarianceOdometry = (1.0 - thetaKalmanGain) * thetaVarianceOdometry;
        thetaVarianceOdometry = Math.max(thetaVarianceOdometry, 0.01*0.01); // No better than .5 deegrees
 
        lastFusedVisionTimestampNtLocalMicros = visionTimestampNtLocalMicros;
    }

    /**
     * Updates the odometry history buffer and propagates the worst-case error
     * bounds
     * for the robot's pose based on movement deltas and current heading
     * uncertainty.
     * *
     * <p>
     * This method employs a geometric conservative model to expand the
     * {@code xVarianceBound} and {@code yVarianceBound} intervals. The error
     * accumulation
     * logic accounts for three primary sources of drift:
     * *
     * <ul>
     * <li><b>Projection Uncertainty (Versine Loss):</b>
     * Calculates the potential loss of distance along the primary axis due to
     * heading error ({@code 1 - cos(Theta_err)}). This ensures the bound covers
     * cases where the robot moved diagonally rather than straight.
     * </li>
     * <li><b>Cross-Contamination:</b>
     * Projects movement from the orthogonal axis into the current axis based
     * on the sine of the heading error (e.g., how much Y-movement bleeds into
     * X-error given the potential heading deviation).
     * </li>
     * <li><b>Sensor Noise:</b>
     * Adds linear error accumulation proportional to distance traveled and
     * angle rotated, scaled by the respective Deviation constants.
     * </li>
     * </ul>
     * *
     * <p>
     * <b>Heading Safety Cap:</b> The {@code thetaVarianceBound} is capped at
     * {@code PI/2} (90 degrees). This prevents cosine projection terms from
     * becoming
     * negative, which would invalidate the bounding box logic. If uncertainty
     * hits 90 degrees, the error model effectively assumes 100% of motion could
     * be in any direction.
     */
    private void updateOdometry() {
        Pose2d newPose = odometryPoseSupplier.get();
        long newTimestampFpgaMicros = toMicroseconds(odometryTimestampFpgaSupplier.get());

        if (odometryHistory.size() > 0) {
            long latestTimestampFpgaMicros = odometryHistory.get(0).timestampFpgaMicros;
            if (newTimestampFpgaMicros == latestTimestampFpgaMicros) {
                return;
            }
            if (newTimestampFpgaMicros < latestTimestampFpgaMicros) {
                odometryHistory.clear();
                priorPose = null;
                priorHeading = null;
                lastOdometryHistoryResetReason = "timestampReversed";
            }
        }

        newOdometryPosePublisher.publish(newPose, "PoseEstimator/Debug/NewOdometryPose");
        Rotation2d newHeading = newPose.getRotation();

        odometryHistory.getScratchpad().pose = newPose;
        odometryHistory.getScratchpad().timestampFpgaMicros = newTimestampFpgaMicros;
        // Debug dashboard telemetry disabled to reduce NetworkTables traffic.
        // newOdometryPoseTimestampPublisher.set(odometryHistory.getScratchpad().timestampFpgaMicros);
        // SignalLogger.writeInteger(
        //         "PoseEstimator/Debug/NewOdometryPoseTimestamp",
        //         odometryHistory.getScratchpad().timestampFpgaMicros);
        odometryHistory.add();
        if (odometryHistory.size() == 1 || priorPose == null || priorHeading == null) {
            // If this is the first pose after startup or after clearing stale history,
            // seed the previous sample state without attempting a delta calculation.
            priorPose = newPose;
            priorHeading = priorPose.getRotation();
            return;
        }

        // Calculate how far the robot has moved and turned.
        double delta_x = Math.abs(newPose.getX() - priorPose.getX());
        double delta_y = Math.abs(newPose.getY() - priorPose.getY());
        double delta_d = Math.sqrt(delta_x * delta_x + delta_y * delta_y);
        double delta_h = Math.abs(newPose.getRotation().minus(priorHeading).getRadians());
        // Variances, which is the key value used by kalman filters, can be added, but they are in squared units.
        // When we calcualte the changes in the error, we need to do it in non-squared units, which is deviation.
        // Calculate the deviation from the current variance.
        
        // Calculate the potential current heading deviation (+/- so many degrees.)
        double ThetaDeviation = Math.sqrt(thetaVarianceOdometry);

        // We get some theta deviation when we turn, some when we drive.  Multiply how much we did these things by their respective constants.
        double additionalDeviation = odoHeadingDeviationPerDistance * delta_d + odoHeadingDeviationPerRadian * delta_h;
        // Convert to variance and add the variance to the prior variance
        thetaVarianceOdometry = (ThetaDeviation+additionalDeviation) * (ThetaDeviation+additionalDeviation);
        // Bound it so we can't ever be more than +/- 90 confused
        thetaVarianceOdometry = Math.min(Math.PI / 2 * Math.PI / 2, thetaVarianceOdometry);

        // For convenience, get the sin and cos values of the heading deviation.
        // We're actually using the prior ThetaDeviation instead of the new ThetaDeviation to do this, but
        // since the time slices are so small it's not a serious source of error.
        double ThetaDeviationBoundCos = Math.cos(ThetaDeviation);
        double ThetaDeviationBoundSin = Math.sin(ThetaDeviation);

        // Calcuale how much x deviation we get from our change in x given our theta deviation
        // See C-D distance of Geogebra/sigma x calculation.ggb x_1-(x_1-σ_x_m) cos(σ_θ) 
        double xVersineError = (delta_x - ThetaDeviationBoundCos * (delta_x - odoLongitudinalDeviationPerDistance * delta_x));

        // Calcuale how much x deviation we get from our change in y given our theta deviation
        double xCrossContaminationError = delta_y * ThetaDeviationBoundSin
                + odoLateralDeviationPerDistance * delta_y * ThetaDeviationBoundCos;
        // Calcuale how much x deviation we get from our change in y given our heading
        double xErrorFromHeading = (Math.abs(priorHeading.getCos()) * odoLongitudinalDeviationPerRadian
                + Math.abs(priorHeading.getSin()) * odoLateralDeviationPerRadian) * delta_h;
        // Add them all together
        additionalDeviation = xVersineError + xCrossContaminationError + xErrorFromHeading;
        // Convert it to deviation and add it in.

        double xDeviationOdometry = Math.sqrt(xVarianceOdometry);
        xVarianceOdometry = (xDeviationOdometry+additionalDeviation) * (xDeviationOdometry+additionalDeviation); // Add variance

        // Repeat the calculation for y
        double yVersineError = (delta_y
                - ThetaDeviationBoundCos * (delta_y - odoLongitudinalDeviationPerDistance * delta_y));
        double yCrossContaminationError = delta_x * ThetaDeviationBoundSin
                + odoLateralDeviationPerDistance * delta_x * ThetaDeviationBoundCos;
        double yErrorFromHeading = (Math.abs(priorHeading.getSin()) * odoLongitudinalDeviationPerRadian
                + Math.abs(priorHeading.getCos()) * odoLateralDeviationPerRadian) * delta_h;
        additionalDeviation = yVersineError + yCrossContaminationError + yErrorFromHeading;
        double yDeviationOdometry = Math.sqrt(yVarianceOdometry);
        yVarianceOdometry = (yDeviationOdometry+additionalDeviation) * (yDeviationOdometry+additionalDeviation); // Add variance

        // Bound the deviation
        xVarianceOdometry = Math.min(xVarianceOdometry, 3 * 3); // No worse than 3 meters
                                                                // We should never be this lost;
                                                                // clamp here to prevent NaNs
        yVarianceOdometry = Math.min(yVarianceOdometry, 3 * 3); // and extreme values leading to
                                                                // overconfidence after a vision update
        // xVarianceOdometryPublisher.set(xVarianceOdometry);
        // yVarianceOdometryPublisher.set(yVarianceOdometry);
        // hVarianceOdometryPublisher.set(thetaVarianceOdometry);
        // SignalLogger.writeDouble("PoseEstimator/Debug/xVarianceBound", xVarianceOdometry);
        // SignalLogger.writeDouble("PoseEstimator/Debug/yVarianceBound", yVarianceOdometry);
        // SignalLogger.writeDouble("PoseEstimator/Debug/hVarianceBound", thetaVarianceOdometry);
        priorPose = newPose;
        priorHeading = newHeading;
    }

    /**
     * Returns a supplier that calculates the robot's estimated pose at the current
     * timestamp,
     * extrapolated from historical odometry data, then re-based relative to the
     * lastest
     * vision-fused pose, or null if no reliable pose estimation can be made.
     *
     * <p>
     * This method compensates for data latency by performing a linear
     * extrapolation.
     * It calculates the velocity (represented as a {@link Twist2d}) between the two
     * most
     * recent odometry snapshots and projects that movement forward to the current
     * {@link RobotController#getFPGATime()}.
     *
     * @return A {@link Supplier} that provides the extrapolated {@link Pose2d} of
     *         the
     *         robot at the exact moment of invocation.
     * 
     *         Contracts:
     *         odometryHistory must constitute a coninuous, sequential, record wiht
     *         unique timestmaps
     *         executed in a single-threaded environment (FRC WPILib)
     * 
     *         Single threadded FRC code
     */
    public Supplier<Pose2d> getFusedPoseSupplier() {
        return fusedPoseSupplier;
    }

    public void resetPose(Pose2d pose) {
        if (pose == null) {
            throw new IllegalArgumentException("Reset pose must not be null.");
        }
        if (useCtrePoseFacade) {
            pendingResetPoseFldMetersRadians = pose;
            pendingResetTimestampFpgaMicros = RobotController.getFPGATime();
            ctreDrivetrain.resetPose(pose);
            xVarianceOdometry = initialXVarianceOdometry;
            yVarianceOdometry = initialYVarianceOdometry;
            thetaVarianceOdometry = initialThetaVarianceOdometry;
            consecutiveInvalidOdometryCycles = 0;
            lastOdometryHistoryResetReason = "resetPose";
            invalidatePredictionCache();
            return;
        }

        long latestTimestampMicros = RobotController.getFPGATime();
        long priorTimestampMicros = latestTimestampMicros - 20_000L;

        odometryHistory.clear();
        odometryHistory.getScratchpad().pose = pose;
        odometryHistory.getScratchpad().timestampFpgaMicros = priorTimestampMicros;
        odometryHistory.add();
        odometryHistory.getScratchpad().pose = pose;
        odometryHistory.getScratchpad().timestampFpgaMicros = latestTimestampMicros;
        odometryHistory.add();

        odometryAnchor = pose;
        fusedAnchor = pose;
        priorPose = pose;
        priorHeading = pose.getRotation();
        xVarianceOdometry = initialXVarianceOdometry;
        yVarianceOdometry = initialYVarianceOdometry;
        thetaVarianceOdometry = initialThetaVarianceOdometry;
        lastFusedVisionTimestampNtLocalMicros = null;
        consecutiveInvalidOdometryCycles = 0;
        lastOdometryHistoryResetReason = "resetPose";
        invalidatePredictionCache();
    }

    public boolean getPredictedFusedState(double lookaheadSeconds, PredictedFusedState out) {
        long predictionStartMicros = SlowCallMonitor.nowMicros();
        String outcome = "ok";
        long latestOdometryTimestamp = -1L;
        long priorOdometryTimestamp = -1L;
        long timeSinceLatestOdometry = -1L;
        long timeBetweenLatestAndPriorOdometry = -1L;

        if (out == null) {
            outcome = "nullOutput";
            throw new IllegalArgumentException("PredictedFusedState output must not be null.");
        }
        if (cachedPredictionEpoch == predictionCacheEpoch
                && Double.compare(cachedPredictionLookaheadSeconds, lookaheadSeconds) == 0) {
            out.copyFrom(cachedPredictedState);
            return out.isValid();
        }
        if (!Double.isFinite(lookaheadSeconds) || lookaheadSeconds < 0.0) {
            outcome = "invalidLookahead";
            out.setInvalid();
            boolean result = printSlowPredictionAndReturn(
                    predictionStartMicros,
                    lookaheadSeconds,
                    outcome,
                    latestOdometryTimestamp,
                    priorOdometryTimestamp,
                    timeSinceLatestOdometry,
                    timeBetweenLatestAndPriorOdometry,
                    false);
            cachePrediction(lookaheadSeconds, out);
            return result;
        }
        if (useCtrePoseFacade) {
            boolean result = getPredictedFusedStateFromCtre(lookaheadSeconds, out, predictionStartMicros);
            cachePrediction(lookaheadSeconds, out);
            return result;
        }
        if (odometryHistory.size() < 2) {
            outcome = "historyTooShort";
            out.setInvalid();
            boolean result = printSlowPredictionAndReturn(
                    predictionStartMicros,
                    lookaheadSeconds,
                    outcome,
                    latestOdometryTimestamp,
                    priorOdometryTimestamp,
                    timeSinceLatestOdometry,
                    timeBetweenLatestAndPriorOdometry,
                    false);
            cachePrediction(lookaheadSeconds, out);
            return result;
        }

        Pose2d latestOdometryPose = odometryHistory.get(0).pose;
        latestOdometryTimestamp = odometryHistory.get(0).timestampFpgaMicros;
        Pose2d priorOdometryPose = odometryHistory.get(1).pose;
        priorOdometryTimestamp = odometryHistory.get(1).timestampFpgaMicros;
        timeSinceLatestOdometry = RobotController.getFPGATime() - latestOdometryTimestamp;
        timeBetweenLatestAndPriorOdometry = latestOdometryTimestamp - priorOdometryTimestamp;
        latestOdometryPosePublisher.publish(latestOdometryPose, "PoseEstimator/Debug/latestOdometryPose");
        // Debug dashboard telemetry disabled to reduce NetworkTables traffic.
        // latestOdometryTimestampPublisher.set(latestOdometryTimestamp);
        priorOdometryPosePublisher.publish(priorOdometryPose, "PoseEstimator/Debug/priorOdometryPose");
        // priorOdometryTimestampPublisher.set(priorOdometryTimestamp);
        // timeSinceLatestOdometryPublisher.set(timeSinceLatestOdometry);
        // timeBetweenLatestAndPriorOdometryPublisher.set(timeBetweenLatestAndPriorOdometry);
        // SignalLogger.writeInteger("PoseEstimator/Debug/latestOdometryTimestamp", latestOdometryTimestamp);
        // SignalLogger.writeInteger("PoseEstimator/Debug/priorOdometryTimestamp", priorOdometryTimestamp);
        // SignalLogger.writeInteger("PoseEstimator/Debug/timeSinceLatestOdometry", timeSinceLatestOdometry);
        // SignalLogger.writeInteger(
        //         "PoseEstimator/Debug/timeBetweenLatestAndPriorOdometry",
        //         timeBetweenLatestAndPriorOdometry);

        if (timeBetweenLatestAndPriorOdometry < 0) {
            outcome = "nonMonotonicOdometry";
            lastPredictionOutcome = outcome;
            updatePredictionDiagnostics(
                    latestOdometryTimestamp,
                    priorOdometryTimestamp,
                    timeSinceLatestOdometry,
                    timeBetweenLatestAndPriorOdometry);
            throw new IllegalStateException(
                    "CRITICAL: Odometry timestamps are non-monotonic or duplicate. dt=" +
                            timeBetweenLatestAndPriorOdometry +
                            ". Fix upstream sensor/loop timing.");
        }
        if (timeBetweenLatestAndPriorOdometry == 0) {
            outcome = "duplicateTimestamp";
            out.setInvalid();
            boolean result = printSlowPredictionAndReturn(
                    predictionStartMicros,
                    lookaheadSeconds,
                    outcome,
                    latestOdometryTimestamp,
                    priorOdometryTimestamp,
                    timeSinceLatestOdometry,
                    timeBetweenLatestAndPriorOdometry,
                    false);
            cachePrediction(lookaheadSeconds, out);
            return result;
        }
        if (timeBetweenLatestAndPriorOdometry > MAX_PREDICTION_ODOMETRY_GAP_MICROS) {
            outcome = "odometryGap";
            out.setInvalid();
            boolean result = printSlowPredictionAndReturn(
                    predictionStartMicros,
                    lookaheadSeconds,
                    outcome,
                    latestOdometryTimestamp,
                    priorOdometryTimestamp,
                    timeSinceLatestOdometry,
                    timeBetweenLatestAndPriorOdometry,
                    false);
            cachePrediction(lookaheadSeconds, out);
            return result;
        }

        double currentFraction = (double) timeSinceLatestOdometry / timeBetweenLatestAndPriorOdometry;
        // twistFractionToGetToNowPublisher.set(currentFraction);
        // SignalLogger.writeDouble("PoseEstimator/Debug/twistFractionToGetToNow", currentFraction);

        Twist2d extrapolatedTwist = priorOdometryPose.log(latestOdometryPose);
        double predictionFraction =
                (timeSinceLatestOdometry + lookaheadSeconds * 1_000_000.0) / timeBetweenLatestAndPriorOdometry;
        extrapolatedTwist.dtheta *= predictionFraction;
        extrapolatedTwist.dx *= predictionFraction;
        extrapolatedTwist.dy *= predictionFraction;
        Pose2d odometryPrediction = latestOdometryPose.exp(extrapolatedTwist);
        odometryNowPublisher.publish(odometryPrediction, "PoseEstimator/Debug/odometryNow");

        Pose2d fusedPosePrediction = fusedAnchor.plus(odometryPrediction.minus(odometryAnchor));
        fusedPosePublisher.publish(fusedPosePrediction, "PoseEstimator/Debug/fusedPose");

        double dtSeconds = timeBetweenLatestAndPriorOdometry / 1_000_000.0;
        double vxMetersPerSecond =
                (latestOdometryPose.getX() - priorOdometryPose.getX()) / dtSeconds;
        double vyMetersPerSecond =
                (latestOdometryPose.getY() - priorOdometryPose.getY()) / dtSeconds;
        double omegaRadiansPerSecond =
                latestOdometryPose.getRotation().minus(priorOdometryPose.getRotation()).getRadians() / dtSeconds;
        long predictionTimestampFpgaMicros = latestOdometryTimestamp
                + timeSinceLatestOdometry
                + Math.round(lookaheadSeconds * 1_000_000.0);

        out.set(
                fusedPosePrediction.getX(),
                fusedPosePrediction.getY(),
                fusedPosePrediction.getRotation().getRadians(),
                vxMetersPerSecond,
                vyMetersPerSecond,
                omegaRadiansPerSecond,
                predictionTimestampFpgaMicros);
        boolean result = printSlowPredictionAndReturn(
                predictionStartMicros,
                lookaheadSeconds,
                outcome,
                latestOdometryTimestamp,
                priorOdometryTimestamp,
                timeSinceLatestOdometry,
                timeBetweenLatestAndPriorOdometry,
                true);
        cachePrediction(lookaheadSeconds, out);
        return result;
    }

    public DoubleSupplier getFusedXDeviation() {
        if (useCtrePoseFacade) {
            return () -> getCtreBackedDeviation(visionXDeviationSupplier, initialXVarianceOdometry);
        }
        return () -> Math.sqrt(xVarianceOdometry);
    }

    public DoubleSupplier getFusedYDeviation() {
        if (useCtrePoseFacade) {
            return () -> getCtreBackedDeviation(visionYDeviationSupplier, initialYVarianceOdometry);
        }
        return () -> Math.sqrt(yVarianceOdometry);
    }

    public DoubleSupplier getFusedThetaDeviation() {
        if (useCtrePoseFacade) {
            return () -> getCtreBackedDeviation(visionThetaDeviationSupplier, initialThetaVarianceOdometry);
        }
        return () -> Math.sqrt(thetaVarianceOdometry);
    }
    
    public Supplier<Time> getOdometryTimestampFpgaSupplier() {
        return odometryTimestampFpgaSupplier;
    }

    public String getLastPredictionOutcome() {
        return lastPredictionOutcome;
    }

    public boolean getLastOdometryValid() {
        return lastOdometryValid;
    }

    public int getConsecutiveInvalidOdometryCycles() {
        return consecutiveInvalidOdometryCycles;
    }

    public int getLastPredictionHistorySize() {
        return lastPredictionHistorySize;
    }

    public long getLastPredictionLatestOdometryTimestampMicros() {
        return lastPredictionLatestOdometryTimestampMicros;
    }

    public long getLastPredictionPriorOdometryTimestampMicros() {
        return lastPredictionPriorOdometryTimestampMicros;
    }

    public long getLastPredictionTimeSinceLatestOdometryMicros() {
        return lastPredictionTimeSinceLatestOdometryMicros;
    }

    public long getLastPredictionTimeBetweenLatestAndPriorOdometryMicros() {
        return lastPredictionTimeBetweenLatestAndPriorOdometryMicros;
    }

    public String getLastOdometryHistoryResetReason() {
        return lastOdometryHistoryResetReason;
    }

    private void invalidatePredictionCache() {
        predictionCacheEpoch++;
        cachedPredictionEpoch = -1L;
        cachedPredictionLookaheadSeconds = Double.NaN;
        cachedPredictedState.setInvalid();
    }

    private void cachePrediction(double lookaheadSeconds, PredictedFusedState state) {
        cachedPredictionEpoch = predictionCacheEpoch;
        cachedPredictionLookaheadSeconds = lookaheadSeconds;
        cachedPredictedState.copyFrom(state);
    }

    private static long toMicroseconds(Time timestamp) {
        return Math.round(timestamp.in(Microseconds));
    }

    private void updatePredictionDiagnostics(
            long latestOdometryTimestamp,
            long priorOdometryTimestamp,
            long timeSinceLatestOdometry,
            long timeBetweenLatestAndPriorOdometry) {
        lastPredictionHistorySize = useCtrePoseFacade
                ? (lastOdometryValid ? 2 : 0)
                : odometryHistory.size();
        lastPredictionLatestOdometryTimestampMicros = latestOdometryTimestamp;
        lastPredictionPriorOdometryTimestampMicros = priorOdometryTimestamp;
        lastPredictionTimeSinceLatestOdometryMicros = timeSinceLatestOdometry;
        lastPredictionTimeBetweenLatestAndPriorOdometryMicros =
                timeBetweenLatestAndPriorOdometry;
    }

    private boolean printSlowPredictionAndReturn(
            long predictionStartMicros,
            double lookaheadSeconds,
            String outcome,
            long latestOdometryTimestamp,
            long priorOdometryTimestamp,
            long timeSinceLatestOdometry,
            long timeBetweenLatestAndPriorOdometry,
            boolean returnValue) {
        lastPredictionOutcome = outcome;
        updatePredictionDiagnostics(
                latestOdometryTimestamp,
                priorOdometryTimestamp,
                timeSinceLatestOdometry,
                timeBetweenLatestAndPriorOdometry);
        long totalMicros = SlowCallMonitor.nowMicros() - predictionStartMicros;
        if (SlowCallMonitor.isSlow(totalMicros, SLOW_PREDICTION_THRESHOLD_MS)) {
            SlowCallMonitor.print(
                    "PoseEstimatorSubsystem.getPredictedFusedState",
                    totalMicros,
                String.format(
                        "outcome=%s lookahead=%.3f s historySize=%d latestTs=%d priorTs=%d sinceLatest=%d us dt=%d us",
                        outcome,
                        lookaheadSeconds,
                        lastPredictionHistorySize,
                        latestOdometryTimestamp,
                        priorOdometryTimestamp,
                        timeSinceLatestOdometry,
                        timeBetweenLatestAndPriorOdometry));
        }
        return returnValue;
    }

    private void periodicCtrePoseFacade() {
        invalidatePredictionCache();
        long periodicStartMicros = SlowCallMonitor.nowMicros();
        SwerveDriveState currentState = ctreDrivetrain.getStateCopy();
        boolean odometryValid = isCtreStateValid(currentState);
        lastOdometryValid = odometryValid;
        if (odometryValid) {
            consecutiveInvalidOdometryCycles = 0;
            long stateTimestampFpgaMicros = currentTimeSecondsToFpgaMicros(currentState.Timestamp);
            if (pendingResetPoseFldMetersRadians != null
                    && stateTimestampFpgaMicros >= pendingResetTimestampFpgaMicros) {
                pendingResetPoseFldMetersRadians = null;
                pendingResetTimestampFpgaMicros = -1L;
            }
        } else {
            consecutiveInvalidOdometryCycles++;
            if (consecutiveInvalidOdometryCycles >= MAX_CONSECUTIVE_INVALID_ODOMETRY_CYCLES_BEFORE_CLEAR) {
                lastOdometryHistoryResetReason = "ctreStateInvalid";
            }
        }
        long totalMicros = SlowCallMonitor.nowMicros() - periodicStartMicros;
        if (SlowCallMonitor.isSlow(totalMicros, SLOW_PERIODIC_THRESHOLD_MS)) {
            SlowCallMonitor.print(
                    "PoseEstimatorSubsystem.periodic",
                    totalMicros,
                    String.format(
                            "ctrePoseFacade=true odometryValid=%s pendingReset=%s",
                            Boolean.toString(odometryValid),
                            Boolean.toString(pendingResetPoseFldMetersRadians != null)));
        }
    }

    private boolean getPredictedFusedStateFromCtre(
            double lookaheadSeconds,
            PredictedFusedState out,
            long predictionStartMicros) {
        String outcome = "ok";
        SwerveDriveState currentState = ctreDrivetrain.getStateCopy();
        long latestOdometryTimestamp = -1L;
        long priorOdometryTimestamp = -1L;
        long timeSinceLatestOdometry = -1L;
        long timeBetweenLatestAndPriorOdometry = -1L;

        if (!isCtreStateValid(currentState)) {
            lastOdometryValid = false;
            out.setInvalid();
            return printSlowPredictionAndReturn(
                    predictionStartMicros,
                    lookaheadSeconds,
                    "ctreStateInvalid",
                    latestOdometryTimestamp,
                    priorOdometryTimestamp,
                    timeSinceLatestOdometry,
                    timeBetweenLatestAndPriorOdometry,
                    false);
        }
        lastOdometryValid = true;

        latestOdometryTimestamp = currentTimeSecondsToFpgaMicros(currentState.Timestamp);
        timeBetweenLatestAndPriorOdometry = Math.max(
                1L,
                Math.round(Math.max(currentState.OdometryPeriod, 1e-6) * 1_000_000.0));
        priorOdometryTimestamp = latestOdometryTimestamp - timeBetweenLatestAndPriorOdometry;
        timeSinceLatestOdometry = Math.max(0L, RobotController.getFPGATime() - latestOdometryTimestamp);

        if (pendingResetPoseFldMetersRadians != null
                && latestOdometryTimestamp < pendingResetTimestampFpgaMicros) {
            long predictionTimestampFpgaMicros =
                    RobotController.getFPGATime() + Math.round(lookaheadSeconds * 1_000_000.0);
            out.set(
                    pendingResetPoseFldMetersRadians.getX(),
                    pendingResetPoseFldMetersRadians.getY(),
                    pendingResetPoseFldMetersRadians.getRotation().getRadians(),
                    0.0,
                    0.0,
                    0.0,
                    predictionTimestampFpgaMicros);
            return printSlowPredictionAndReturn(
                    predictionStartMicros,
                    lookaheadSeconds,
                    "ctreResetPending",
                    latestOdometryTimestamp,
                    priorOdometryTimestamp,
                    timeSinceLatestOdometry,
                    timeBetweenLatestAndPriorOdometry,
                    true);
        }

        Pose2d currentPoseFldMetersRadians = currentState.Pose;
        Rotation2d currentHeadingFldRadians = currentPoseFldMetersRadians.getRotation();
        double totalPredictionSeconds = (timeSinceLatestOdometry / 1_000_000.0) + lookaheadSeconds;
        Twist2d predictedTwistRbtMetersRadians = new Twist2d(
                currentState.Speeds.vxMetersPerSecond * totalPredictionSeconds,
                currentState.Speeds.vyMetersPerSecond * totalPredictionSeconds,
                currentState.Speeds.omegaRadiansPerSecond * totalPredictionSeconds);
        Pose2d predictedPoseFldMetersRadians = currentPoseFldMetersRadians.exp(predictedTwistRbtMetersRadians);
        Translation2d velocityFldMetersPerSecond = new Translation2d(
                currentState.Speeds.vxMetersPerSecond,
                currentState.Speeds.vyMetersPerSecond).rotateBy(currentHeadingFldRadians);
        long predictionTimestampFpgaMicros = latestOdometryTimestamp
                + timeSinceLatestOdometry
                + Math.round(lookaheadSeconds * 1_000_000.0);

        out.set(
                predictedPoseFldMetersRadians.getX(),
                predictedPoseFldMetersRadians.getY(),
                predictedPoseFldMetersRadians.getRotation().getRadians(),
                velocityFldMetersPerSecond.getX(),
                velocityFldMetersPerSecond.getY(),
                currentState.Speeds.omegaRadiansPerSecond,
                predictionTimestampFpgaMicros);
        latestOdometryPosePublisher.publish(currentPoseFldMetersRadians, "PoseEstimator/Debug/latestOdometryPose");
        fusedPosePublisher.publish(predictedPoseFldMetersRadians, "PoseEstimator/Debug/fusedPose");
        return printSlowPredictionAndReturn(
                predictionStartMicros,
                lookaheadSeconds,
                outcome,
                latestOdometryTimestamp,
                priorOdometryTimestamp,
                timeSinceLatestOdometry,
                timeBetweenLatestAndPriorOdometry,
                true);
    }

    private static long currentTimeSecondsToFpgaMicros(double currentTimeSeconds) {
        return Math.round(Utils.currentTimeToFPGATime(currentTimeSeconds) * 1_000_000.0);
    }

    private static boolean isCtreStateValid(SwerveDriveState currentState) {
        return currentState != null
                && currentState.SuccessfulDaqs > 0
                && Double.isFinite(currentState.Pose.getX())
                && Double.isFinite(currentState.Pose.getY())
                && Double.isFinite(currentState.Pose.getRotation().getRadians())
                && Double.isFinite(currentState.Speeds.vxMetersPerSecond)
                && Double.isFinite(currentState.Speeds.vyMetersPerSecond)
                && Double.isFinite(currentState.Speeds.omegaRadiansPerSecond)
                && Double.isFinite(currentState.Timestamp);
    }

    private static double getCtreBackedDeviation(
            DoubleSupplier visionDeviationSupplier,
            double initialVariance) {
        double configuredVisionDeviation = visionDeviationSupplier.getAsDouble();
        if (Double.isFinite(configuredVisionDeviation) && configuredVisionDeviation > 0.0) {
            return configuredVisionDeviation;
        }
        return Math.sqrt(initialVariance);
    }
}
