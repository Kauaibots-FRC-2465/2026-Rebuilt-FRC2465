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
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.fieldmath.FieldMath;
import frc.robot.fieldmath.TravelWindowDirectionTracker;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SparkAnglePositionSubsystem;
import frc.robot.utility.TuningDashboard;

/**
 * Snowblow variant that uses an operator-selected field target inside the alliance zone.
 *
 * <p>Frame suffixes used in this command:
 * Fld = absolute WPILib field frame and the default for internal geometry/solver math.
 * Drv = driver-perspective field-centric request frame at the Phoenix request boundary.
 * Rbt = robot/body frame.
 *
 * <p>Unit conventions:
 * Translation2d and Pose2d values use WPILib meters/radians defaults.
 * Primitive suffixes spell units explicitly: Degrees, Radians, Inches, Meters, Ips.
 */
public class SnowblowToAllianceWithOperatorAim extends Command {
    private static final String SOLVE_STATUS_NOT_RUN = "NOT_RUN";
    private static final String COMMAND_STATE_INITIALIZED = "INITIALIZED";
    private static final String COMMAND_STATE_ACTIVE = "ACTIVE";
    private static final String COMMAND_STATE_HOLDING_LAST_RESULT = "HOLDING_LAST_RESULT";
    private static final String COMMAND_STATE_NON_FIELD_CENTRIC = "NON_FIELD_CENTRIC";
    private static final String COMMAND_STATE_PREDICTION_INVALID = "PREDICTION_INVALID";
    private static final String COMMAND_STATE_NO_SOLUTION = "NO_SOLUTION";
    private static final String COMMAND_STATE_ENDED = "ENDED";
    private static final String STOP_REASON_NONE = "NONE";
    private static final String STOP_REASON_NON_FIELD_CENTRIC = "NON_FIELD_CENTRIC";
    private static final String STOP_REASON_PREDICTION_INVALID = "PREDICTION_INVALID";
    private static final String STOP_REASON_NO_SOLUTION = "NO_SOLUTION";
    private static final String STOP_REASON_INTERRUPTED = "INTERRUPTED";
    private static final String STOP_REASON_FINISHED = "FINISHED";

    private final CommandSwerveDrivetrain drivetrain;
    private final PoseEstimatorSubsystem poseEstimator;
    private final SparkAnglePositionSubsystem horizontalAim;
    private final SparkAnglePositionSubsystem verticalAim;
    private final ShooterSubsystem shooter;
    private final Supplier<SwerveRequest> driveRequestSupplier;
    private final Supplier<Translation2d> targetTranslationFldMetersSupplier;
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
    private final edu.wpi.first.networktables.DoubleArrayPublisher futurePoseFldPublisher;
    private final edu.wpi.first.networktables.DoubleArrayPublisher futureVelocityFldPublisher;
    private final BooleanPublisher validSolutionPublisher;
    private final BooleanPublisher tuningActivePublisher;
    private final BooleanPublisher tuningPredictedStateValidPublisher;
    private final StringPublisher tuningCommandStatePublisher;
    private final StringPublisher tuningLastStopReasonPublisher;
    private final StringPublisher tuningPredictionOutcomePublisher;
    private final StringPublisher tuningSolveStatusPublisher;
    private final StringPublisher tuningSolveDetailPublisher;
    private final DoublePublisher tuningAppliedFlywheelCommandIpsPublisher;
    private final DoublePublisher tuningMeasuredFlywheelSpeedIpsPublisher;
    private final DoublePublisher tuningTargetDistanceInchesPublisher;
    private final DoublePublisher tuningTargetElevationInchesPublisher;
    private final IntegerPublisher tuningPredictionInvalidCountPublisher;
    private final IntegerPublisher tuningNoSolutionCountPublisher;
    private final IntegerPublisher tuningNonFieldCentricCountPublisher;
    private final BooleanPublisher tuningOdometryValidPublisher;
    private final IntegerPublisher tuningPredictionHistorySizePublisher;
    private final IntegerPublisher tuningConsecutiveInvalidOdometryCyclesPublisher;
    private final IntegerPublisher tuningLatestOdometryTimestampMicrosPublisher;
    private final IntegerPublisher tuningPriorOdometryTimestampMicrosPublisher;
    private final IntegerPublisher tuningTimeSinceLatestOdometryMicrosPublisher;
    private final IntegerPublisher tuningTimeBetweenLatestAndPriorOdometryMicrosPublisher;
    private final StringPublisher tuningOdometryHistoryResetReasonPublisher;
    private final DoublePublisher tuningPredictedVelocityXFldMetersPerSecondPublisher;
    private final DoublePublisher tuningPredictedVelocityYFldMetersPerSecondPublisher;
    private final DoublePublisher tuningPredictedOmegaFldRadiansPerSecondPublisher;
    private final PoseEstimatorSubsystem.PredictedFusedState futureStateFldMetersRadians =
            new PoseEstimatorSubsystem.PredictedFusedState();
    private final BallTrajectoryLookup.MovingShotSolution idealMovingShotSolution =
            new BallTrajectoryLookup.MovingShotSolution();
    private final BallTrajectoryLookup.MovingShotSolution movingShotSolution =
            new BallTrajectoryLookup.MovingShotSolution();
    private final double[] futurePoseFldMetersDegreesArray = new double[3];
    private final double[] futureVelocityFldMetersPerSecondDegreesPerSecondArray = new double[3];
    private Rotation2d lastValidRobotHeadingTargetFldRadians = new Rotation2d();
    private double lastValidTurretDeltaDegrees = 0.0;
    private double lastValidHoodAngleDegrees = ShooterConstants.COMMANDED_MAXIMUM_ALLOWED_HOOD_ANGLE_DEGREES;
    private double lastValidFlywheelCommandIps = 0.0;
    private double lastCommandedFlywheelSetpointIps = 0.0;
    private boolean hasLatchedShotCommand = false;
    private final TravelWindowDirectionTracker preferredHeadingTracker =
            new TravelWindowDirectionTracker(
                    Meters.convertFrom(
                            ShooterConstants.COMMANDED_PREFERRED_HEADING_TRAVEL_WINDOW_INCHES,
                            Inches),
                    Meters.convertFrom(
                            ShooterConstants.COMMANDED_PREFERRED_HEADING_MIN_SAMPLE_SPACING_INCHES,
                            Inches));
    private Translation2d lastTravelVectorFldMeters = new Translation2d();
    private int predictionInvalidCount;
    private int noSolutionCount;
    private int nonFieldCentricCount;

    public SnowblowToAllianceWithOperatorAim(
            CommandSwerveDrivetrain drivetrain,
            PoseEstimatorSubsystem poseEstimator,
            SparkAnglePositionSubsystem horizontalAim,
            SparkAnglePositionSubsystem verticalAim,
            ShooterSubsystem shooter,
            Supplier<SwerveRequest> driveRequestSupplier,
            Supplier<Translation2d> targetTranslationFldMetersSupplier) {
        this.drivetrain = Objects.requireNonNull(drivetrain, "drivetrain must not be null");
        this.poseEstimator = Objects.requireNonNull(poseEstimator, "poseEstimator must not be null");
        this.horizontalAim = Objects.requireNonNull(horizontalAim, "horizontalAim must not be null");
        this.verticalAim = Objects.requireNonNull(verticalAim, "verticalAim must not be null");
        this.shooter = Objects.requireNonNull(shooter, "shooter must not be null");
        this.driveRequestSupplier =
                Objects.requireNonNull(driveRequestSupplier, "driveRequestSupplier must not be null");
        this.targetTranslationFldMetersSupplier =
                Objects.requireNonNull(targetTranslationFldMetersSupplier, "targetSupplier must not be null");

        NetworkTable snowblowTable = NetworkTableInstance.getDefault()
                .getTable("SnowblowToAllianceWithOperatorAim");
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
        futurePoseFldPublisher = snowblowTable.getDoubleArrayTopic("futurePose").publish();
        futureVelocityFldPublisher = snowblowTable.getDoubleArrayTopic("futureVelocity").publish();
        validSolutionPublisher = snowblowTable.getBooleanTopic("validSolution").publish();
        NetworkTable tuningSnowblowTable =
                NetworkTableInstance.getDefault().getTable("Tuning").getSubTable("SnowblowOperatorAim");
        tuningActivePublisher = tuningSnowblowTable.getBooleanTopic("active").publish();
        tuningPredictedStateValidPublisher = tuningSnowblowTable.getBooleanTopic("predictedStateValid").publish();
        tuningCommandStatePublisher = tuningSnowblowTable.getStringTopic("commandState").publish();
        tuningLastStopReasonPublisher = tuningSnowblowTable.getStringTopic("lastStopReason").publish();
        tuningPredictionOutcomePublisher = tuningSnowblowTable.getStringTopic("predictionOutcome").publish();
        tuningSolveStatusPublisher = tuningSnowblowTable.getStringTopic("solveStatus").publish();
        tuningSolveDetailPublisher = tuningSnowblowTable.getStringTopic("solveDetail").publish();
        tuningAppliedFlywheelCommandIpsPublisher =
                tuningSnowblowTable.getDoubleTopic("appliedFlywheelCommandIps").publish();
        tuningMeasuredFlywheelSpeedIpsPublisher =
                tuningSnowblowTable.getDoubleTopic("measuredFlywheelSpeedIps").publish();
        tuningTargetDistanceInchesPublisher =
                tuningSnowblowTable.getDoubleTopic("targetDistanceInches").publish();
        tuningTargetElevationInchesPublisher =
                tuningSnowblowTable.getDoubleTopic("targetElevationInches").publish();
        tuningPredictionInvalidCountPublisher =
                tuningSnowblowTable.getIntegerTopic("predictionInvalidCount").publish();
        tuningNoSolutionCountPublisher = tuningSnowblowTable.getIntegerTopic("noSolutionCount").publish();
        tuningNonFieldCentricCountPublisher =
                tuningSnowblowTable.getIntegerTopic("nonFieldCentricCount").publish();
        tuningOdometryValidPublisher = tuningSnowblowTable.getBooleanTopic("odometryValid").publish();
        tuningPredictionHistorySizePublisher =
                tuningSnowblowTable.getIntegerTopic("predictionHistorySize").publish();
        tuningConsecutiveInvalidOdometryCyclesPublisher =
                tuningSnowblowTable.getIntegerTopic("consecutiveInvalidOdometryCycles").publish();
        tuningLatestOdometryTimestampMicrosPublisher =
                tuningSnowblowTable.getIntegerTopic("latestOdometryTimestampMicros").publish();
        tuningPriorOdometryTimestampMicrosPublisher =
                tuningSnowblowTable.getIntegerTopic("priorOdometryTimestampMicros").publish();
        tuningTimeSinceLatestOdometryMicrosPublisher =
                tuningSnowblowTable.getIntegerTopic("timeSinceLatestOdometryMicros").publish();
        tuningTimeBetweenLatestAndPriorOdometryMicrosPublisher =
                tuningSnowblowTable.getIntegerTopic("timeBetweenLatestAndPriorOdometryMicros").publish();
        tuningOdometryHistoryResetReasonPublisher =
                tuningSnowblowTable.getStringTopic("odometryHistoryResetReason").publish();
        tuningPredictedVelocityXFldMetersPerSecondPublisher =
                tuningSnowblowTable.getDoubleTopic("predictedVelocityXMetersPerSecond").publish();
        tuningPredictedVelocityYFldMetersPerSecondPublisher =
                tuningSnowblowTable.getDoubleTopic("predictedVelocityYMetersPerSecond").publish();
        tuningPredictedOmegaFldRadiansPerSecondPublisher =
                tuningSnowblowTable.getDoubleTopic("predictedOmegaRadiansPerSecond").publish();
        publishPredictedVelocityFldTelemetry(Double.NaN, Double.NaN, Double.NaN);
        facingAngleDrive.withHeadingPID(5.0, 0.0, 0.0);
        facingAngleDrive.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(this.drivetrain, horizontalAim, verticalAim, shooter);
    }

    @Override
    public void initialize() {
        preferredHeadingTracker.reset();
        lastTravelVectorFldMeters = new Translation2d();
        predictionInvalidCount = 0;
        noSolutionCount = 0;
        nonFieldCentricCount = 0;
        tuningActivePublisher.set(true);
        tuningPredictedStateValidPublisher.set(false);
        tuningCommandStatePublisher.set(COMMAND_STATE_INITIALIZED);
        tuningLastStopReasonPublisher.set(STOP_REASON_NONE);
        tuningPredictionOutcomePublisher.set("NOT_RUN");
        tuningSolveStatusPublisher.set(SOLVE_STATUS_NOT_RUN);
        tuningSolveDetailPublisher.set("INITIALIZED_NOT_RUN");
        tuningAppliedFlywheelCommandIpsPublisher.set(0.0);
        tuningMeasuredFlywheelSpeedIpsPublisher.set(shooter.getMainFlywheelSpeedIPS());
        tuningTargetDistanceInchesPublisher.set(Double.NaN);
        tuningTargetElevationInchesPublisher.set(ShooterConstants.COMMANDED_SNOWBLOW_TARGET_ELEVATION_INCHES);
        publishPredictedVelocityFldTelemetry(Double.NaN, Double.NaN, Double.NaN);
        hasLatchedShotCommand = false;
        lastValidRobotHeadingTargetFldRadians = new Rotation2d();
        lastValidTurretDeltaDegrees = 0.0;
        lastValidHoodAngleDegrees = verticalAim.getAngle().in(Degrees);
        lastValidFlywheelCommandIps = 0.0;
        lastCommandedFlywheelSetpointIps = shooter.getMainFlywheelSpeedIPS();
        publishFailureCounters();
        publishPredictionDiagnostics();
    }

    @Override
    public void execute() {
        SwerveRequest requestedDrive = driveRequestSupplier.get();
        if (!(requestedDrive instanceof SwerveRequest.FieldCentric fieldCentricRequestDrv)) {
            clearSolutionTelemetry();
            nonFieldCentricCount++;
            tuningActivePublisher.set(true);
            tuningPredictedStateValidPublisher.set(false);
            tuningCommandStatePublisher.set(COMMAND_STATE_NON_FIELD_CENTRIC);
            tuningLastStopReasonPublisher.set(STOP_REASON_NON_FIELD_CENTRIC);
            tuningPredictionOutcomePublisher.set("NOT_REQUESTED");
            tuningSolveStatusPublisher.set(SOLVE_STATUS_NOT_RUN);
            tuningSolveDetailPublisher.set("SKIPPED_NON_FIELD_CENTRIC_REQUEST");
            tuningAppliedFlywheelCommandIpsPublisher.set(0.0);
            tuningMeasuredFlywheelSpeedIpsPublisher.set(shooter.getMainFlywheelSpeedIPS());
            publishPredictedVelocityFldTelemetry(Double.NaN, Double.NaN, Double.NaN);
            publishFailureCounters();
            publishPredictionDiagnostics();
            drivetrain.setControl(requestedDrive);
            return;
        }

        updateLastTravelVectorFldMeters();
        if (!poseEstimator.getPredictedFusedState(
                ShooterConstants.COMMANDED_SHOOTER_LOOKAHEAD_SECONDS,
                futureStateFldMetersRadians)) {
            clearSolutionTelemetry();
            predictionInvalidCount++;
            tuningActivePublisher.set(true);
            tuningPredictedStateValidPublisher.set(false);
            tuningCommandStatePublisher.set(
                    hasLatchedShotCommand ? COMMAND_STATE_HOLDING_LAST_RESULT : COMMAND_STATE_PREDICTION_INVALID);
            tuningLastStopReasonPublisher.set(STOP_REASON_PREDICTION_INVALID);
            String predictionOutcome = poseEstimator.getLastPredictionOutcome();
            tuningPredictionOutcomePublisher.set(predictionOutcome);
            tuningSolveStatusPublisher.set(SOLVE_STATUS_NOT_RUN);
            tuningSolveDetailPublisher.set(
                    hasLatchedShotCommand
                            ? "SKIPPED_PREDICTION_INVALID_HOLDING_LAST_RESULT_" + predictionOutcome
                            : "SKIPPED_PREDICTION_INVALID_" + predictionOutcome);
            tuningAppliedFlywheelCommandIpsPublisher.set(
                    hasLatchedShotCommand ? lastValidFlywheelCommandIps : 0.0);
            tuningMeasuredFlywheelSpeedIpsPublisher.set(shooter.getMainFlywheelSpeedIPS());
            publishPredictedVelocityFldTelemetry(Double.NaN, Double.NaN, Double.NaN);
            publishFailureCounters();
            publishPredictionDiagnostics();
            if (hasLatchedShotCommand) {
                applyHeldShotCommand();
            }
            drivetrain.setControl(requestedDrive);
            return;
        }
        tuningPredictedStateValidPublisher.set(true);
        tuningPredictionOutcomePublisher.set(poseEstimator.getLastPredictionOutcome());
        publishPredictionDiagnostics();
        publishPredictedVelocityFldTelemetry(
                futureStateFldMetersRadians.vxMetersPerSecond,
                futureStateFldMetersRadians.vyMetersPerSecond,
                futureStateFldMetersRadians.omegaRadiansPerSecond);
        publishFutureFieldState();

        Pose2d futureRobotPoseFldMetersRadians = new Pose2d(
                futureStateFldMetersRadians.xMeters,
                futureStateFldMetersRadians.yMeters,
                Rotation2d.fromRadians(futureStateFldMetersRadians.headingRadians));
        Translation2d targetTranslationFldMeters = targetTranslationFldMetersSupplier.get();
        publishTargetTranslationFldMeters(targetTranslationFldMeters);
        tuningTargetDistanceInchesPublisher.set(Inches.convertFrom(
                targetTranslationFldMeters.getDistance(futureRobotPoseFldMetersRadians.getTranslation()),
                Meters));
        tuningTargetElevationInchesPublisher.set(ShooterConstants.COMMANDED_SNOWBLOW_TARGET_ELEVATION_INCHES);

        Rotation2d preferredRobotHeadingFldRadians = getPreferredRobotHeadingFldRadians(
                futureRobotPoseFldMetersRadians.getTranslation(),
                targetTranslationFldMeters,
                futureRobotPoseFldMetersRadians.getRotation());
        Rotation2d robotHeadingTargetFldRadians = preferredRobotHeadingFldRadians;
        BallTrajectoryLookup.FixedFlywheelShotStatus fixedFlywheelStatus =
                updateShooterSolution(targetTranslationFldMeters, preferredRobotHeadingFldRadians);
        tuningSolveStatusPublisher.set(fixedFlywheelStatus.name());
        tuningMeasuredFlywheelSpeedIpsPublisher.set(shooter.getMainFlywheelSpeedIPS());
        if (fixedFlywheelStatus != BallTrajectoryLookup.FixedFlywheelShotStatus.NO_SOLUTION) {
            tuningActivePublisher.set(true);
            tuningCommandStatePublisher.set(COMMAND_STATE_ACTIVE);
            tuningAppliedFlywheelCommandIpsPublisher.set(lastValidFlywheelCommandIps);
            robotHeadingTargetFldRadians = Rotation2d.fromDegrees(movingShotSolution.getRobotHeadingDegrees());
        } else {
            noSolutionCount++;
            tuningActivePublisher.set(true);
            tuningCommandStatePublisher.set(
                    hasLatchedShotCommand ? COMMAND_STATE_HOLDING_LAST_RESULT : COMMAND_STATE_NO_SOLUTION);
            tuningLastStopReasonPublisher.set(STOP_REASON_NO_SOLUTION);
            tuningSolveDetailPublisher.set(
                    hasLatchedShotCommand
                            ? "NO_SOLUTION_HOLDING_LAST_RESULT"
                            : "NO_SOLUTION_NO_LATCHED_RESULT");
            tuningAppliedFlywheelCommandIpsPublisher.set(
                    hasLatchedShotCommand ? lastValidFlywheelCommandIps : 0.0);
            if (hasLatchedShotCommand) {
                applyHeldShotCommand();
                robotHeadingTargetFldRadians = lastValidRobotHeadingTargetFldRadians;
            }
            publishFailureCounters();
        }
        Rotation2d robotHeadingTargetDrvRadians =
                robotHeadingTargetFldRadians.minus(drivetrain.getDriverPerspectiveForward());

        drivetrain.setControl(
                facingAngleDrive
                        .withVelocityX(fieldCentricRequestDrv.VelocityX)
                        .withVelocityY(fieldCentricRequestDrv.VelocityY)
                        .withTargetDirection(robotHeadingTargetDrvRadians)
                        .withDeadband(fieldCentricRequestDrv.Deadband)
                        .withRotationalDeadband(fieldCentricRequestDrv.RotationalDeadband)
                        .withCenterOfRotation(fieldCentricRequestDrv.CenterOfRotation)
                        .withDriveRequestType(fieldCentricRequestDrv.DriveRequestType)
                        .withSteerRequestType(fieldCentricRequestDrv.SteerRequestType)
                        .withDesaturateWheelSpeeds(fieldCentricRequestDrv.DesaturateWheelSpeeds)
                        .withForwardPerspective(fieldCentricRequestDrv.ForwardPerspective));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        tuningActivePublisher.set(false);
        tuningPredictedStateValidPublisher.set(false);
        tuningCommandStatePublisher.set(COMMAND_STATE_ENDED);
        tuningLastStopReasonPublisher.set(interrupted ? STOP_REASON_INTERRUPTED : STOP_REASON_FINISHED);
        tuningPredictionOutcomePublisher.set("NOT_RUN");
        tuningSolveStatusPublisher.set(SOLVE_STATUS_NOT_RUN);
        tuningSolveDetailPublisher.set(interrupted ? "ENDED_INTERRUPTED" : "ENDED_FINISHED");
        tuningAppliedFlywheelCommandIpsPublisher.set(0.0);
        tuningMeasuredFlywheelSpeedIpsPublisher.set(shooter.getMainFlywheelSpeedIPS());
        publishPredictedVelocityFldTelemetry(Double.NaN, Double.NaN, Double.NaN);
        publishPredictionDiagnostics();
    }

    private BallTrajectoryLookup.FixedFlywheelShotStatus updateShooterSolution(
            Translation2d targetTranslationFldMeters,
            Rotation2d preferredRobotHeadingFldRadians) {
        double minimumHoodAngleDegrees = verticalAim.getMinimumAngle().in(Degrees);
        double maximumHoodAngleDegrees = verticalAim.getMaximumAngle().in(Degrees);
        BallTrajectoryLookup.FixedFlywheelShotStatus fixedFlywheelStatus =
                MovingShotMath.solveCommandedMovingShot(
                        verticalAim,
                        ShooterConstants.COMMANDED_MOVING_SHOT_HOOD_SEARCH_STEP_DEGREES,
                        ShooterConstants.COMMANDED_MOVING_SHOT_FIXED_FLYWHEEL_HOOD_SEARCH_STEP_DEGREES,
                        futureStateFldMetersRadians,
                        targetTranslationFldMeters,
                        ShooterConstants.COMMANDED_SNOWBLOW_TARGET_ELEVATION_INCHES,
                        ShooterConstants.COMMANDED_MAXIMUM_SHOOTING_HEIGHT_INCHES,
                        preferredRobotHeadingFldRadians.getRadians(),
                        horizontalAim.getMinimumAngle().in(Degrees),
                        horizontalAim.getMaximumAngle().in(Degrees),
                        shooter.getMainFlywheelSpeedIPS(),
                        lastCommandedFlywheelSetpointIps,
                        idealMovingShotSolution,
                        movingShotSolution);
        if (fixedFlywheelStatus == BallTrajectoryLookup.FixedFlywheelShotStatus.NO_SOLUTION) {
            return fixedFlywheelStatus;
        }

        double idealFlywheelCommandIps = idealMovingShotSolution.getFlywheelCommandIps();
        // Debug dashboard telemetry disabled to reduce NetworkTables traffic.
        // targetElevationInchesPublisher.set(ShooterConstants.COMMANDED_SNOWBLOW_TARGET_ELEVATION_INCHES);
        // validSolutionPublisher.set(fixedFlywheelStatus == BallTrajectoryLookup.FixedFlywheelShotStatus.VALID);

        double commandedHoodAngleDegrees = MovingShotMath.getCommandedHoodAngleDegrees(
                fixedFlywheelStatus,
                minimumHoodAngleDegrees,
                maximumHoodAngleDegrees,
                movingShotSolution);
        double clampedTurretDeltaDegrees = MovingShotMath.clampTurretDeltaDegrees(
                movingShotSolution.getTurretDeltaDegrees(),
                horizontalAim.getMinimumAngle().in(Degrees),
                horizontalAim.getMaximumAngle().in(Degrees));
        // hoodAngleDegreesPublisher.set(movingShotSolution.getHoodAngleDegrees());
        // shotExitVelocityIpsPublisher.set(movingShotSolution.getLauncherRelativeExitVelocityIps());
        // fieldRelativeExitVelocityIpsPublisher.set(movingShotSolution.getFieldRelativeExitVelocityIps());
        // flywheelCommandIpsPublisher.set(movingShotSolution.getFlywheelCommandIps());
        // shotAzimuthDegreesPublisher.set(movingShotSolution.getShotAzimuthDegrees());
        // turretDeltaDegreesPublisher.set(clampedTurretDeltaDegrees);
        // robotHeadingDegreesPublisher.set(movingShotSolution.getRobotHeadingDegrees());
        verticalAim.setAngle(Degrees.of(commandedHoodAngleDegrees));
        horizontalAim.setAngle(Degrees.of(clampedTurretDeltaDegrees));
        shooter.setCoupledIPS(idealFlywheelCommandIps);
        lastCommandedFlywheelSetpointIps = idealFlywheelCommandIps;
        lastValidRobotHeadingTargetFldRadians =
                Rotation2d.fromDegrees(movingShotSolution.getRobotHeadingDegrees());
        lastValidTurretDeltaDegrees = clampedTurretDeltaDegrees;
        lastValidHoodAngleDegrees = commandedHoodAngleDegrees;
        lastValidFlywheelCommandIps = idealFlywheelCommandIps;
        hasLatchedShotCommand = true;
        return fixedFlywheelStatus;
    }

    private void updateLastTravelVectorFldMeters() {
        Pose2d currentRobotPoseFldMetersRadians = poseEstimator.getFusedPoseSupplier().get();
        if (currentRobotPoseFldMetersRadians == null) {
            return;
        }
        lastTravelVectorFldMeters =
                preferredHeadingTracker.update(currentRobotPoseFldMetersRadians.getTranslation());
    }

    private Rotation2d getPreferredRobotHeadingFldRadians(
            Translation2d robotTranslationFldMeters,
            Translation2d targetTranslationFldMeters,
            Rotation2d fallbackHeadingFldRadians) {
        return MovingShotMath.getPreferredHeadingForTravelDirection(
                lastTravelVectorFldMeters,
                robotTranslationFldMeters,
                targetTranslationFldMeters,
                fallbackHeadingFldRadians);
    }

    private void publishTargetTranslationFldMeters(Translation2d targetTranslationFldMeters) {
        TuningDashboard.publishShootingTarget(targetTranslationFldMeters);
    }

    private void publishFutureFieldState() {
        futurePoseFldMetersDegreesArray[0] = futureStateFldMetersRadians.xMeters;
        futurePoseFldMetersDegreesArray[1] = futureStateFldMetersRadians.yMeters;
        futurePoseFldMetersDegreesArray[2] = Math.toDegrees(futureStateFldMetersRadians.headingRadians);
        futureVelocityFldMetersPerSecondDegreesPerSecondArray[0] =
                futureStateFldMetersRadians.vxMetersPerSecond;
        futureVelocityFldMetersPerSecondDegreesPerSecondArray[1] =
                futureStateFldMetersRadians.vyMetersPerSecond;
        futureVelocityFldMetersPerSecondDegreesPerSecondArray[2] =
                Math.toDegrees(futureStateFldMetersRadians.omegaRadiansPerSecond);
        // Debug dashboard telemetry disabled to reduce NetworkTables traffic.
        // futurePoseFldPublisher.set(futurePoseFldMetersDegreesArray);
        // futureVelocityFldPublisher.set(futureVelocityFldMetersPerSecondDegreesPerSecondArray);
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

    private void applyHeldShotCommand() {
        verticalAim.setAngle(Degrees.of(lastValidHoodAngleDegrees));
        horizontalAim.setAngle(Degrees.of(lastValidTurretDeltaDegrees));
        shooter.setCoupledIPS(lastValidFlywheelCommandIps);
        lastCommandedFlywheelSetpointIps = lastValidFlywheelCommandIps;
    }

    private void publishFailureCounters() {
        tuningPredictionInvalidCountPublisher.set(predictionInvalidCount);
        tuningNoSolutionCountPublisher.set(noSolutionCount);
        tuningNonFieldCentricCountPublisher.set(nonFieldCentricCount);
    }

    private void publishPredictionDiagnostics() {
        tuningOdometryValidPublisher.set(poseEstimator.getLastOdometryValid());
        tuningPredictionHistorySizePublisher.set(poseEstimator.getLastPredictionHistorySize());
        tuningConsecutiveInvalidOdometryCyclesPublisher.set(poseEstimator.getConsecutiveInvalidOdometryCycles());
        tuningLatestOdometryTimestampMicrosPublisher.set(
                poseEstimator.getLastPredictionLatestOdometryTimestampMicros());
        tuningPriorOdometryTimestampMicrosPublisher.set(
                poseEstimator.getLastPredictionPriorOdometryTimestampMicros());
        tuningTimeSinceLatestOdometryMicrosPublisher.set(
                poseEstimator.getLastPredictionTimeSinceLatestOdometryMicros());
        tuningTimeBetweenLatestAndPriorOdometryMicrosPublisher.set(
                poseEstimator.getLastPredictionTimeBetweenLatestAndPriorOdometryMicros());
        tuningOdometryHistoryResetReasonPublisher.set(
                poseEstimator.getLastOdometryHistoryResetReason());
    }

    private void publishPredictedVelocityFldTelemetry(
            double vxFldMetersPerSecond,
            double vyFldMetersPerSecond,
            double omegaFldRadiansPerSecond) {
        tuningPredictedVelocityXFldMetersPerSecondPublisher.set(vxFldMetersPerSecond);
        tuningPredictedVelocityYFldMetersPerSecondPublisher.set(vyFldMetersPerSecond);
        tuningPredictedOmegaFldRadiansPerSecondPublisher.set(omegaFldRadiansPerSecond);
    }

}
