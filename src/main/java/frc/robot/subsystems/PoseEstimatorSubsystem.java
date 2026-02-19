package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.RingBuffer;

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

    private class OdometryData {
        Pose2d pose;
        Long timestamp;
    }

    // Fusion typically happens between indexes 4 and 8 
    RingBuffer<OdometryData> odometryHistory = new RingBuffer<>(20, () -> new OdometryData());

    public static class Configuration {
        public Supplier<Pose2d> odometryPose;
        public Supplier<Long> odometryTimestamp;
        public BooleanSupplier odometryValid;
        public double odoLongitudinalDeviationPerDistance;
        public double odoLateralDeviationPerDistance;
        public double odoHeadingDeviationPerDistance;
        public double odoLongitudinalDeviationPerRadian;
        public double odoLateralDeviationPerRadian;
        public double odoHeadingDeviationPerRadian;
        public Supplier<Pose2d> visionPose;
        public Supplier<Long> visionTimestamp;
        public BooleanSupplier visionIsValid;
        public DoubleSupplier visionXDeviation;
        public DoubleSupplier visionYDeviation;
        public DoubleSupplier visionThetaDeviation;
        public double initialXDeviation;
        public double initialYDeviation;
        public double initialThetaDeviation;
    }

    public PoseEstimatorSubsystem(Configuration config) {
        odometryPose = config.odometryPose;
        odometryTimestamp = config.odometryTimestamp;
        odometryIsValid = config.odometryValid;
        odoLongitudinalDeviationPerDistance = config.odoLongitudinalDeviationPerDistance;
        odoLateralDeviationPerDistance = config.odoLateralDeviationPerDistance;
        odoHeadingDeviationPerDistance = config.odoHeadingDeviationPerDistance;
        odoLongitudinalDeviationPerRadian = config.odoLongitudinalDeviationPerRadian;
        odoLateralDeviationPerRadian = config.odoLateralDeviationPerRadian;
        odoHeadingDeviationPerRadian = config.odoHeadingDeviationPerRadian;
        visionPose = config.visionPose;
        visionTimestamp = config.visionTimestamp;
        visionIsValid = config.visionIsValid;
        visionXDeviation = config.visionXDeviation;
        visionYDeviation = config.visionYDeviation;
        visionThetaDeviation = config.visionThetaDeviation;
        xVarianceBound = config.initialXDeviation * config.initialXDeviation;
        yVarianceBound = config.initialYDeviation * config.initialYDeviation;
        ThetaVarianceBound = config.initialThetaDeviation * config.initialThetaDeviation;
        fusedAnchor = odometryAnchor = new Pose2d();
    }

    Pose2d odometryAnchor;
    Pose2d fusedAnchor;

    private final Supplier<Pose2d> odometryPose;
    private final Supplier<Long> odometryTimestamp;
    private final BooleanSupplier odometryIsValid;
    private final double odoLongitudinalDeviationPerDistance;
    private final double odoLateralDeviationPerDistance;
    private final double odoHeadingDeviationPerDistance;
    private final double odoLongitudinalDeviationPerRadian;
    private final double odoLateralDeviationPerRadian;
    private final double odoHeadingDeviationPerRadian;
    private final DoubleSupplier visionXDeviation;
    private final DoubleSupplier visionYDeviation;
    private final DoubleSupplier visionThetaDeviation;
    private Supplier<Pose2d> visionPose;
    private Supplier<Long> visionTimestamp;
    private BooleanSupplier visionIsValid;

    double xVarianceBound = 0;
    double yVarianceBound = 0;
    double ThetaVarianceBound = 0;

    Pose2d priorPose;
    Rotation2d priorHeading;

    @Override
    public void periodic() {
        if (odometryIsValid.getAsBoolean()) {
            updateOdometry();
        } else {
            odometryHistory.clear();
        }

        if (visionIsValid.getAsBoolean()) {
            fuseVision();
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
     * ({@code visionTimestamp}).
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
     * <li>{@code visionTimestamp} must fall within the time range covered by the
     * history buffer.
     * Measurements older than the buffer or newer than the latest update are
     * discarded.</li>
     * </ul>
     */
    private void fuseVision() {
        OdometryData newerOdometryData = null;
        OdometryData olderOdometryData = null;
        Long visionTime = visionTimestamp.get();
        if (odometryHistory.size() < 2)
            return;
        for (int index = 0; index < odometryHistory.size(); ++index) {
            Long odometryTimestamp = odometryHistory.get(index).timestamp;
            if (odometryTimestamp >= visionTime) {
                newerOdometryData = odometryHistory.get(index);
            }
            if (odometryTimestamp <= visionTime) {
                olderOdometryData = odometryHistory.get(index);
                break;
            }
        }
        // Odometry is much faster than vision, which should never be newer
        // The apriltag reading must exist inside our odometry history
        if (newerOdometryData == null || olderOdometryData == null) {
            return;
        }
        long dt = newerOdometryData.timestamp - olderOdometryData.timestamp;
        double alpha = (double) (visionTime - olderOdometryData.timestamp) / dt;
        Pose2d matchingVisionPose = visionPose.get();
        odometryAnchor = olderOdometryData.pose.interpolate(newerOdometryData.pose, alpha);
        double vXVariance = visionXDeviation.getAsDouble();
        vXVariance = vXVariance * vXVariance;
        double vYVariance = visionYDeviation.getAsDouble();
        vYVariance = vYVariance * vYVariance;
        double vThetaVariance = visionThetaDeviation.getAsDouble();
        vThetaVariance = vThetaVariance * vThetaVariance;
        double xKalmanGain = xVarianceBound / (xVarianceBound + vXVariance);
        double yKalmanGain = yVarianceBound / (yVarianceBound + vYVariance);
        double ThetaKalmanGain = ThetaVarianceBound / (ThetaVarianceBound + vThetaVariance);
        Rotation2d odometryRotation = odometryAnchor.getRotation();
        double ThetaDifference = matchingVisionPose.getRotation().minus(odometryRotation).getRadians();
        fusedAnchor = new Pose2d(
                odometryAnchor.getX() * (1 - xKalmanGain) + matchingVisionPose.getX() * (xKalmanGain),
                odometryAnchor.getY() * (1 - yKalmanGain) + matchingVisionPose.getY() * (yKalmanGain),
                odometryRotation.plus(new Rotation2d(ThetaDifference * ThetaKalmanGain)));
        xVarianceBound = (1.0 - xKalmanGain) * xVarianceBound;
        yVarianceBound = (1.0 - yKalmanGain) * yVarianceBound;
        ThetaVarianceBound = (1.0 - ThetaKalmanGain) * ThetaVarianceBound;
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
        Pose2d newPose = odometryPose.get();
        Rotation2d newHeading = newPose.getRotation();
        odometryHistory.getScratchpad().pose = newPose;
        odometryHistory.getScratchpad().timestamp = odometryTimestamp.get();
        System.out.println("Timestamp: " + odometryTimestamp.get());
        odometryHistory.add();
        if (odometryHistory.size() == 1) {
            priorPose = newPose;
            priorHeading = priorPose.getRotation();
            return;
        }
        double delta_x = Math.abs(newPose.getX() - priorPose.getX());
        double delta_y = Math.abs(newPose.getY() - priorPose.getY());
        double delta_d = Math.sqrt(delta_x * delta_x + delta_y * delta_y);
        double delta_h = Math.abs(newPose.getRotation().minus(priorHeading).getRadians());
        double ThetaDeviation = Math.sqrt(ThetaVarianceBound);
        double ThetaDeviationBoundCos = Math.cos(ThetaDeviation);
        double ThetaDeviationBoundSin = Math.sin(ThetaDeviation);
        double newDeviation = odoHeadingDeviationPerDistance * delta_d + odoHeadingDeviationPerRadian * delta_h;
        ThetaVarianceBound += newDeviation * newDeviation;
        ThetaVarianceBound = Math.min(Math.PI / 2 * Math.PI / 2, ThetaVarianceBound);
        double xVersineError = (delta_x
                - ThetaDeviationBoundCos * (delta_x - odoLongitudinalDeviationPerDistance * delta_x));
        double xCrossContaminationError = delta_y * ThetaDeviationBoundSin
                + odoLateralDeviationPerDistance * delta_y * ThetaDeviationBoundCos;
        double xErrorFromHeading = (Math.abs(priorHeading.getCos()) * odoLongitudinalDeviationPerRadian
                + Math.abs(priorHeading.getSin()) * odoLateralDeviationPerRadian) * delta_h;
        newDeviation = xVersineError + xCrossContaminationError + xErrorFromHeading;
        xVarianceBound += newDeviation * newDeviation; // Add variance
        double yVersineError = (delta_y
                - ThetaDeviationBoundCos * (delta_y - odoLongitudinalDeviationPerDistance * delta_y));
        double yCrossContaminationError = delta_x * ThetaDeviationBoundSin
                + odoLateralDeviationPerDistance * delta_x * ThetaDeviationBoundCos;
        double yErrorFromHeading = (Math.abs(priorHeading.getSin()) * odoLongitudinalDeviationPerRadian
                + Math.abs(priorHeading.getCos()) * odoLateralDeviationPerRadian) * delta_h;
        newDeviation = yVersineError + yCrossContaminationError + yErrorFromHeading;
        yVarianceBound += newDeviation * newDeviation; // Add variance
        xVarianceBound = Math.min(xVarianceBound, 3 * 3); // We should never be this lost; clamp here to prevent NaNs
                                                          // and also
        yVarianceBound = Math.min(yVarianceBound, 3 * 3); // extreme values leading to overconfidence after a vision
                                                          // update
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
        return () -> {
            if (odometryHistory.size() < 2)
                return null; // This is a federated filter, tell the downstream drive not to rely on us
            Pose2d latestOdometryPose       = odometryHistory.get(0).pose;
            Long   latestOdometryTimestamp  = odometryHistory.get(0).timestamp;
            Pose2d priorOdometryPose        = odometryHistory.get(1).pose;
            Long   priorOdometryTimestamp   = odometryHistory.get(1).timestamp;
            Long   timeSinceLatestOdometry  = (RobotController.getFPGATime() - latestOdometryTimestamp);
            Long   timeBetweenLatestAndPriorOdometry = (latestOdometryTimestamp - priorOdometryTimestamp);
            System.out.println("Latest time: " + latestOdometryTimestamp);
            System.out.println("Prior time:  " + priorOdometryTimestamp);
            System.out.println("Delta time:  " + timeBetweenLatestAndPriorOdometry);
            System.out.println(odometryHistory.size());
            if (timeBetweenLatestAndPriorOdometry <= 0) {
                throw new IllegalStateException(
                        "CRITICAL: Odometry timestamps are non-monotonic or duplicate. dt=" +
                                timeBetweenLatestAndPriorOdometry +
                                ". Fix upstream sensor/loop timing.");
            }
            if (timeBetweenLatestAndPriorOdometry > 50000)
                return null; // if more than 50 ms between poses, tell the downstream drive not to rely on us
            double twistFractionToGetToNow = (double) timeSinceLatestOdometry / timeBetweenLatestAndPriorOdometry;
            if (twistFractionToGetToNow > 1.5)
                return null; // If we have to extrapolate more than 1.5 cycles, tell the downstream drive not
                             // to rely on us
            Twist2d extrapolatedTwist = priorOdometryPose.log(latestOdometryPose);
            extrapolatedTwist.dtheta *= twistFractionToGetToNow;
            extrapolatedTwist.dx     *= twistFractionToGetToNow;
            extrapolatedTwist.dy     *= twistFractionToGetToNow;
            Pose2d odometryNow = latestOdometryPose.exp(extrapolatedTwist);
            /*
             * * Geometric Logic Confirmation:
             * 1. odometryNow.minus(odometryAnchor) calculates the relative Transform2d
             * (dx, dy, dtheta) in the *local frame* of the odometryAnchor.
             * (e.g., "Robot moved 1.0m Forward relative to where it started").
             * * 2. fusedAnchor.plus(...) applies that local movement to the fusedAnchor.
             * Since .plus() rotates the translation by the pose's heading,
             * this correctly "replays" the robot's specific physical motion
             * onto the new, corrected field heading provided by vision.
             * * DO NOT CHANGE TO GLOBAL SUBTRACTION.
             * The following is mathematically equivalent to
             * fusedAnchor.exp(odometryAnchor.log(odometryNow))
             * for a static rebase.
             */
            return fusedAnchor.plus(odometryNow.minus(odometryAnchor));
        };
    }

    public DoubleSupplier getFusedXDeviation() {
        return () -> Math.sqrt(xVarianceBound);
    }

    public DoubleSupplier getFusedYDeviation() {
        return () -> Math.sqrt(yVarianceBound);
    }

    public DoubleSupplier getFusedThetaDeviation() {
        return () -> Math.sqrt(ThetaVarianceBound);
    }
    
    public Supplier<Long> getTimestampSupplier() {
        return odometryTimestamp; 
    }
}
