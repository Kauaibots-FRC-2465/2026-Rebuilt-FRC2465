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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.fieldmath.FieldMath;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PinpointSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SparkAnglePositionSubsystem;
import frc.robot.utility.SlowCallMonitor;
import frc.robot.utility.TuningDashboard;

/**
 * Drives while solving a moving shot into the fixed hub target.
 */
public class ScoreInHub extends Command {
    private static final double SLOW_EXECUTE_THRESHOLD_MS = 8.0;
    private static final double SLOW_PREDICTION_THRESHOLD_MS = 4.0;
    private static final double SLOW_SOLVER_THRESHOLD_MS = 5.0;
    private static final double SLOW_SET_CONTROL_THRESHOLD_MS = 2.0;
    private static final double TRANSIENT_SOLUTION_HOLD_SECONDS = 0.25;
    private static final boolean ENABLE_SPEED_UP_ACCELERATION_LIMIT = false;
    private static final double SPEED_UP_ACCELERATION_LIMIT_METERS_PER_SECOND_SQUARED = 0.1524;
    private static final double MAX_ACTIVE_TRANSLATIONAL_SPEED_METERS_PER_SECOND = 0.5;
    private static final double RADIAL_SPEED_LIMIT_VIABLE_MARGIN = 0.90;
    private static final double RADIAL_SPEED_LIMIT_SEARCH_TOLERANCE_METERS_PER_SECOND = 0.02;
    private static final int RADIAL_SPEED_LIMIT_SEARCH_MAX_ITERATIONS = 10;

    @FunctionalInterface
    interface HubShotViabilityEvaluator {
        boolean hasViableShot(double robotFieldVxMetersPerSecond, double robotFieldVyMetersPerSecond);
    }

    static final class TravelVelocityLimitResult {
        private final Translation2d limitedVelocityMetersPerSecond;
        private final double updatedMaximumTowardHubTravelSpeedMetersPerSecond;

        TravelVelocityLimitResult(
                Translation2d limitedVelocityMetersPerSecond,
                double updatedMaximumTowardHubTravelSpeedMetersPerSecond) {
            this.limitedVelocityMetersPerSecond = limitedVelocityMetersPerSecond;
            this.updatedMaximumTowardHubTravelSpeedMetersPerSecond =
                    updatedMaximumTowardHubTravelSpeedMetersPerSecond;
        }

        Translation2d getLimitedVelocityMetersPerSecond() {
            return limitedVelocityMetersPerSecond;
        }

        double getUpdatedMaximumTowardHubTravelSpeedMetersPerSecond() {
            return updatedMaximumTowardHubTravelSpeedMetersPerSecond;
        }
    }

    private final CommandSwerveDrivetrain drivetrain;
    private final PinpointSubsystem pinpointSubsystem;
    private final PoseEstimatorSubsystem poseEstimator;
    private final SparkAnglePositionSubsystem horizontalAim;
    private final SparkAnglePositionSubsystem verticalAim;
    private final ShooterSubsystem shooter;
    private final Supplier<SwerveRequest> driveRequestSupplier;
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
    private final DoublePublisher hoodTrackingErrorDegreesPublisher;
    private final DoublePublisher flywheelPredictionErrorIpsPublisher;
    private final DoublePublisher tuningPredictedVelocityXMetersPerSecondPublisher;
    private final DoublePublisher tuningPredictedVelocityYMetersPerSecondPublisher;
    private final DoublePublisher tuningPredictedOmegaRadiansPerSecondPublisher;
    private final DoublePublisher tuningPinpointVelocityXMetersPerSecondPublisher;
    private final DoublePublisher tuningPinpointVelocityYMetersPerSecondPublisher;
    private final DoublePublisher tuningPinpointOmegaRadiansPerSecondPublisher;
    private final DoublePublisher tuningLookupTargetDistanceInchesPublisher;
    private final DoublePublisher tuningSelectedHoodAngleDegreesPublisher;
    private final DoublePublisher tuningModeledFlywheelCommandIpsPublisher;
    private final DoublePublisher tuningCommandedFlywheelCommandIpsPublisher;
    private final edu.wpi.first.networktables.DoubleArrayPublisher futurePosePublisher;
    private final edu.wpi.first.networktables.DoubleArrayPublisher futureVelocityPublisher;
    private final BooleanPublisher validSolutionPublisher;
    private final StringPublisher fieldTypePublisher;
    private final PoseEstimatorSubsystem.PredictedFusedState futureState =
            new PoseEstimatorSubsystem.PredictedFusedState();
    private final BallTrajectoryLookup.MovingShotSolution idealMovingShotSolution =
            new BallTrajectoryLookup.MovingShotSolution();
    private final BallTrajectoryLookup.MovingShotSolution movingShotSolution =
            new BallTrajectoryLookup.MovingShotSolution();
    private final BallTrajectoryLookup.MovingShotSolution radialSpeedLimitSearchSolution =
            new BallTrajectoryLookup.MovingShotSolution();
    private final MovingShotMath.EmpiricalMovingShotDebugInfo empiricalDebugInfo =
            new MovingShotMath.EmpiricalMovingShotDebugInfo();
    private final double[] futureFieldPose = new double[3];
    private final double[] futureFieldVelocity = new double[3];
    private Rotation2d lastValidRobotHeadingTarget = new Rotation2d();
    private double lastValidTurretDeltaDegrees = 0.0;
    private double lastValidFlywheelCommandIps = 0.0;
    private double lastValidSolutionTimestampSeconds = Double.NEGATIVE_INFINITY;
    private boolean hasLatchedValidSolution = false;
    private double lastVelocityLimitTimestampSeconds = Double.NaN;
    private double latchedMaximumTowardHubTravelSpeedMetersPerSecond = Double.POSITIVE_INFINITY;
    private double lastCommandedFlywheelSetpointIps = 0.0;
    private double previousCycleMeasuredHoodAngleDegrees = Double.NaN;
    private double previousCycleCommandedHoodAngleDegrees = Double.NaN;
    private double previousCycleMeasuredFlywheelSpeedIps = Double.NaN;
    private double previousCyclePredictedFlywheelSpeedIps = Double.NaN;

    public ScoreInHub(
            CommandSwerveDrivetrain drivetrain,
            PinpointSubsystem pinpointSubsystem,
            PoseEstimatorSubsystem poseEstimator,
            SparkAnglePositionSubsystem horizontalAim,
            SparkAnglePositionSubsystem verticalAim,
            ShooterSubsystem shooter,
            Supplier<SwerveRequest> driveRequestSupplier) {
        this.drivetrain = Objects.requireNonNull(drivetrain, "drivetrain must not be null");
        this.pinpointSubsystem = Objects.requireNonNull(pinpointSubsystem, "pinpointSubsystem must not be null");
        this.poseEstimator = Objects.requireNonNull(poseEstimator, "poseEstimator must not be null");
        this.horizontalAim = Objects.requireNonNull(horizontalAim, "horizontalAim must not be null");
        this.verticalAim = Objects.requireNonNull(verticalAim, "verticalAim must not be null");
        this.shooter = Objects.requireNonNull(shooter, "shooter must not be null");
        this.driveRequestSupplier =
                Objects.requireNonNull(driveRequestSupplier, "driveRequestSupplier must not be null");
        NetworkTable scoreTable = NetworkTableInstance.getDefault().getTable("ScoreInHub");
        targetDistanceInchesPublisher = scoreTable.getDoubleTopic("targetDistanceInches").publish();
        targetElevationInchesPublisher = scoreTable.getDoubleTopic("targetElevationInches").publish();
        hoodAngleDegreesPublisher = scoreTable.getDoubleTopic("hoodAngleDegrees").publish();
        shotExitVelocityIpsPublisher = scoreTable.getDoubleTopic("shotExitVelocityIps").publish();
        fieldRelativeExitVelocityIpsPublisher =
                scoreTable.getDoubleTopic("fieldRelativeExitVelocityIps").publish();
        flywheelCommandIpsPublisher = scoreTable.getDoubleTopic("flywheelCommandIps").publish();
        shotAzimuthDegreesPublisher = scoreTable.getDoubleTopic("shotAzimuthDegrees").publish();
        turretDeltaDegreesPublisher = scoreTable.getDoubleTopic("turretDeltaDegrees").publish();
        robotHeadingDegreesPublisher = scoreTable.getDoubleTopic("robotHeadingDegrees").publish();
        hoodTrackingErrorDegreesPublisher = scoreTable.getDoubleTopic("hoodTrackingErrorDegrees").publish();
        flywheelPredictionErrorIpsPublisher = scoreTable.getDoubleTopic("flywheelPredictionErrorIps").publish();
        futurePosePublisher = scoreTable.getDoubleArrayTopic("futurePose").publish();
        futureVelocityPublisher = scoreTable.getDoubleArrayTopic("futureVelocity").publish();
        validSolutionPublisher = scoreTable.getBooleanTopic("validSolution").publish();
        fieldTypePublisher = scoreTable.getStringTopic(".type").publish();
        NetworkTable tuningScoreTable =
                NetworkTableInstance.getDefault().getTable("Tuning").getSubTable("ScoreInHub");
        tuningPredictedVelocityXMetersPerSecondPublisher =
                tuningScoreTable.getDoubleTopic("predictedVelocityXMetersPerSecond").publish();
        tuningPredictedVelocityYMetersPerSecondPublisher =
                tuningScoreTable.getDoubleTopic("predictedVelocityYMetersPerSecond").publish();
        tuningPredictedOmegaRadiansPerSecondPublisher =
                tuningScoreTable.getDoubleTopic("predictedOmegaRadiansPerSecond").publish();
        tuningPinpointVelocityXMetersPerSecondPublisher =
                tuningScoreTable.getDoubleTopic("pinpointVelocityXMetersPerSecond").publish();
        tuningPinpointVelocityYMetersPerSecondPublisher =
                tuningScoreTable.getDoubleTopic("pinpointVelocityYMetersPerSecond").publish();
        tuningPinpointOmegaRadiansPerSecondPublisher =
                tuningScoreTable.getDoubleTopic("pinpointOmegaRadiansPerSecond").publish();
        tuningLookupTargetDistanceInchesPublisher =
                tuningScoreTable.getDoubleTopic("lookupTargetDistanceInches").publish();
        tuningSelectedHoodAngleDegreesPublisher =
                tuningScoreTable.getDoubleTopic("selectedHoodAngleDegrees").publish();
        tuningModeledFlywheelCommandIpsPublisher =
                tuningScoreTable.getDoubleTopic("modeledFlywheelCommandIps").publish();
        tuningCommandedFlywheelCommandIpsPublisher =
                tuningScoreTable.getDoubleTopic("commandedFlywheelCommandIps").publish();
        publishPredictedVelocityTelemetry(Double.NaN, Double.NaN, Double.NaN);
        publishPinpointVelocityTelemetry(Double.NaN, Double.NaN, Double.NaN);
        publishEmpiricalSolverTelemetry();
        // Debug dashboard telemetry disabled to reduce NetworkTables traffic.
        // fieldTypePublisher.set("Field2d");

        facingAngleDrive.withHeadingPID(5.0, 0.0, 0.0);
        facingAngleDrive.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(this.drivetrain, horizontalAim, verticalAim, shooter);
    }

    @Override
    public void initialize() {
        lastVelocityLimitTimestampSeconds = Double.NaN;
        latchedMaximumTowardHubTravelSpeedMetersPerSecond = Double.POSITIVE_INFINITY;
        lastCommandedFlywheelSetpointIps = shooter.getMainFlywheelSpeedIPS();
        previousCycleMeasuredHoodAngleDegrees = Double.NaN;
        previousCycleCommandedHoodAngleDegrees = Double.NaN;
        previousCycleMeasuredFlywheelSpeedIps = Double.NaN;
        previousCyclePredictedFlywheelSpeedIps = Double.NaN;
        hoodTrackingErrorDegreesPublisher.set(Double.NaN);
        flywheelPredictionErrorIpsPublisher.set(Double.NaN);
        publishPredictedVelocityTelemetry(Double.NaN, Double.NaN, Double.NaN);
        publishPinpointVelocityTelemetry(Double.NaN, Double.NaN, Double.NaN);
        empiricalDebugInfo.invalidate();
        publishEmpiricalSolverTelemetry();
    }

    @Override
    public void execute() {
        long executeStartMicros = SlowCallMonitor.nowMicros();
        long predictionMicros = 0L;
        long solutionMicros = 0L;
        long setControlMicros = 0L;
        Translation2d target = getTarget();

        SwerveRequest requestedDrive = driveRequestSupplier.get();
        if (!(requestedDrive instanceof SwerveRequest.FieldCentric fieldCentricRequest)) {
            clearSolutionTelemetry();
            publishPredictedVelocityTelemetry(Double.NaN, Double.NaN, Double.NaN);
            publishCurrentPinpointVelocityTelemetry();
            empiricalDebugInfo.invalidate();
            publishEmpiricalSolverTelemetry();
            drivetrain.setControl(requestedDrive);
            long totalMicros = SlowCallMonitor.nowMicros() - executeStartMicros;
            if (SlowCallMonitor.isSlow(totalMicros, SLOW_EXECUTE_THRESHOLD_MS)) {
                SlowCallMonitor.print(
                        "ScoreInHub.execute",
                        totalMicros,
                        "fieldCentric=false");
            }
            return;
        }

        publishTrackingDiagnostics();
        publishCurrentPinpointVelocityTelemetry();

        Translation2d requestedOperatorPerspectiveVelocityMetersPerSecond =
                limitTranslationalAcceleration(fieldCentricRequest);
        Translation2d requestedFieldRelativeVelocityMetersPerSecond =
                operatorPerspectiveToFieldRelativeVelocity(requestedOperatorPerspectiveVelocityMetersPerSecond);
        double limitedVelocityXMetersPerSecond;
        double limitedVelocityYMetersPerSecond;

        long predictionStartMicros = SlowCallMonitor.nowMicros();
        if (!poseEstimator.getPredictedFusedState(
                ShooterConstants.COMMANDED_SHOOTER_LOOKAHEAD_SECONDS,
                futureState)) {
            predictionMicros = SlowCallMonitor.nowMicros() - predictionStartMicros;
            clearSolutionTelemetry();
            publishPredictedVelocityTelemetry(Double.NaN, Double.NaN, Double.NaN);
            empiricalDebugInfo.invalidate();
            publishEmpiricalSolverTelemetry();
            Translation2d currentRobotPosition = getCurrentRobotPosition();
            if (isDrivingAwayFromHub(
                    requestedFieldRelativeVelocityMetersPerSecond,
                    currentRobotPosition,
                    target)) {
                latchedMaximumTowardHubTravelSpeedMetersPerSecond = Double.POSITIVE_INFINITY;
            }
            Translation2d fallbackFieldVelocityMetersPerSecond = clampTowardHubTravelSpeed(
                    requestedFieldRelativeVelocityMetersPerSecond,
                    currentRobotPosition,
                    target,
                    MAX_ACTIVE_TRANSLATIONAL_SPEED_METERS_PER_SECOND);
            Translation2d fallbackOperatorPerspectiveVelocityMetersPerSecond =
                    fieldRelativeToOperatorPerspectiveVelocity(fallbackFieldVelocityMetersPerSecond);
            limitedVelocityXMetersPerSecond = fallbackOperatorPerspectiveVelocityMetersPerSecond.getX();
            limitedVelocityYMetersPerSecond = fallbackOperatorPerspectiveVelocityMetersPerSecond.getY();
            boolean holdingLastSolution = shouldHoldLastSolution();
            if (holdingLastSolution) {
                applyHeldShotCommand(
                        fieldCentricRequest,
                        limitedVelocityXMetersPerSecond,
                        limitedVelocityYMetersPerSecond);
            } else {
                applyNoSolutionShotCommand(getCurrentRobotPosition(), target);
                drivetrain.setControl(
                        fieldCentricRequest
                                .withVelocityX(limitedVelocityXMetersPerSecond)
                                .withVelocityY(limitedVelocityYMetersPerSecond));
            }
            long totalMicros = SlowCallMonitor.nowMicros() - executeStartMicros;
            if (SlowCallMonitor.isSlow(totalMicros, SLOW_EXECUTE_THRESHOLD_MS)
                    || SlowCallMonitor.isSlow(predictionMicros, SLOW_PREDICTION_THRESHOLD_MS)) {
                SlowCallMonitor.print(
                        "ScoreInHub.execute",
                        totalMicros,
                        String.format(
                                "futureState=false holding=%s holdAge=%.3f s predict=%.3f ms",
                                Boolean.toString(holdingLastSolution),
                                getLatchedSolutionAgeSeconds(),
                                SlowCallMonitor.toMillis(predictionMicros)));
            }
            return;
        }
        predictionMicros = SlowCallMonitor.nowMicros() - predictionStartMicros;
        publishPredictedVelocityTelemetry(
                futureState.vxMetersPerSecond,
                futureState.vyMetersPerSecond,
                futureState.omegaRadiansPerSecond);
        publishFutureState();

        Pose2d futurePose = new Pose2d(
                futureState.xMeters,
                futureState.yMeters,
                Rotation2d.fromRadians(futureState.headingRadians));
        publishTarget(target);

        Rotation2d preferredRobotHeading = getPreferredRobotHeading(
                futurePose.getTranslation(),
                target,
                futurePose.getRotation());
        Translation2d limitedFieldRelativeVelocityMetersPerSecond = limitTowardHubTravelVelocityForViableShot(
                requestedFieldRelativeVelocityMetersPerSecond,
                futurePose.getTranslation(),
                target,
                preferredRobotHeading);
        Translation2d limitedOperatorPerspectiveVelocityMetersPerSecond =
                fieldRelativeToOperatorPerspectiveVelocity(limitedFieldRelativeVelocityMetersPerSecond);
        limitedVelocityXMetersPerSecond = limitedOperatorPerspectiveVelocityMetersPerSecond.getX();
        limitedVelocityYMetersPerSecond = limitedOperatorPerspectiveVelocityMetersPerSecond.getY();
        Rotation2d robotHeadingTarget = preferredRobotHeading;
        long solutionStartMicros = SlowCallMonitor.nowMicros();
        if (updateShooterSolution(target, preferredRobotHeading)) {
            robotHeadingTarget = Rotation2d.fromDegrees(movingShotSolution.getRobotHeadingDegrees());
        } else if (shouldHoldLastSolution()) {
            applyHeldShotCommand(
                    fieldCentricRequest,
                    limitedVelocityXMetersPerSecond,
                    limitedVelocityYMetersPerSecond);
            long totalMicros = SlowCallMonitor.nowMicros() - executeStartMicros;
            solutionMicros = SlowCallMonitor.nowMicros() - solutionStartMicros;
            if (SlowCallMonitor.isSlow(totalMicros, SLOW_EXECUTE_THRESHOLD_MS)
                    || SlowCallMonitor.isSlow(solutionMicros, SLOW_SOLVER_THRESHOLD_MS)) {
                SlowCallMonitor.print(
                        "ScoreInHub.execute",
                        totalMicros,
                        String.format(
                                "holdingLastSolution=true holdAge=%.3f s predict=%.3f ms solve=%.3f ms setControl=0.000 ms",
                                getLatchedSolutionAgeSeconds(),
                                SlowCallMonitor.toMillis(predictionMicros),
                                SlowCallMonitor.toMillis(solutionMicros)));
            }
            return;
        } else {
            applyNoSolutionShotCommand(futurePose.getTranslation(), target);
        }
        solutionMicros = SlowCallMonitor.nowMicros() - solutionStartMicros;
        Rotation2d operatorPerspectiveHeadingTarget =
                robotHeadingTarget.minus(drivetrain.getDriverPerspectiveForward());

        long setControlStartMicros = SlowCallMonitor.nowMicros();
        drivetrain.setControl(
                facingAngleDrive
                        .withVelocityX(limitedVelocityXMetersPerSecond)
                        .withVelocityY(limitedVelocityYMetersPerSecond)
                        .withTargetDirection(operatorPerspectiveHeadingTarget)
                        .withDeadband(fieldCentricRequest.Deadband)
                        .withRotationalDeadband(fieldCentricRequest.RotationalDeadband)
                        .withCenterOfRotation(fieldCentricRequest.CenterOfRotation)
                        .withDriveRequestType(fieldCentricRequest.DriveRequestType)
                        .withSteerRequestType(fieldCentricRequest.SteerRequestType)
                        .withDesaturateWheelSpeeds(fieldCentricRequest.DesaturateWheelSpeeds)
                        .withForwardPerspective(fieldCentricRequest.ForwardPerspective));
        setControlMicros = SlowCallMonitor.nowMicros() - setControlStartMicros;

        long totalMicros = SlowCallMonitor.nowMicros() - executeStartMicros;
        if (SlowCallMonitor.isSlow(totalMicros, SLOW_EXECUTE_THRESHOLD_MS)
                || SlowCallMonitor.isSlow(predictionMicros, SLOW_PREDICTION_THRESHOLD_MS)
                || SlowCallMonitor.isSlow(solutionMicros, SLOW_SOLVER_THRESHOLD_MS)
                || SlowCallMonitor.isSlow(setControlMicros, SLOW_SET_CONTROL_THRESHOLD_MS)) {
            SlowCallMonitor.print(
                    "ScoreInHub.execute",
                    totalMicros,
                    String.format(
                            "predict=%.3f ms solve=%.3f ms setControl=%.3f ms",
                            SlowCallMonitor.toMillis(predictionMicros),
                            SlowCallMonitor.toMillis(solutionMicros),
                            SlowCallMonitor.toMillis(setControlMicros)));
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        lastVelocityLimitTimestampSeconds = Double.NaN;
        latchedMaximumTowardHubTravelSpeedMetersPerSecond = Double.POSITIVE_INFINITY;
        publishPredictedVelocityTelemetry(Double.NaN, Double.NaN, Double.NaN);
        publishPinpointVelocityTelemetry(Double.NaN, Double.NaN, Double.NaN);
        empiricalDebugInfo.invalidate();
        publishEmpiricalSolverTelemetry();
    }

    private Translation2d getTarget() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        return FieldMath.getHubTarget(alliance);
    }

    private boolean updateShooterSolution(
            Translation2d target,
            Rotation2d preferredRobotHeading) {
        long functionStartMicros = SlowCallMonitor.nowMicros();
        double minimumHoodAngleDegrees = verticalAim.getMinimumAngle().in(Degrees);
        double maximumHoodAngleDegrees = verticalAim.getMaximumAngle().in(Degrees);
        double targetDistanceInches = Inches.convertFrom(
                target.getDistance(new Translation2d(futureState.xMeters, futureState.yMeters)),
                Meters);
        long solverStartMicros = SlowCallMonitor.nowMicros();
        BallTrajectoryLookup.FixedFlywheelShotStatus fixedFlywheelStatus =
                MovingShotMath.solveCommandedMovingShot(
                        verticalAim,
                        ShooterConstants.COMMANDED_MOVING_SHOT_HOOD_SEARCH_STEP_DEGREES,
                        ShooterConstants.COMMANDED_MOVING_SHOT_FIXED_FLYWHEEL_HOOD_SEARCH_STEP_DEGREES,
                        futureState,
                        target,
                        ShooterConstants.COMMANDED_SCORE_IN_HUB_TARGET_ELEVATION_INCHES,
                        ShooterConstants.COMMANDED_MAXIMUM_SHOOTING_HEIGHT_INCHES,
                        preferredRobotHeading.getRadians(),
                        horizontalAim.getMinimumAngle().in(Degrees),
                        horizontalAim.getMaximumAngle().in(Degrees),
                        shooter.getMainFlywheelSpeedIPS(),
                        lastCommandedFlywheelSetpointIps,
                        idealMovingShotSolution,
                        movingShotSolution,
                        empiricalDebugInfo);
        publishEmpiricalSolverTelemetry();
        if (fixedFlywheelStatus == BallTrajectoryLookup.FixedFlywheelShotStatus.NO_SOLUTION) {
            long solverMicros = SlowCallMonitor.nowMicros() - solverStartMicros;
            long totalMicros = SlowCallMonitor.nowMicros() - functionStartMicros;
            if (SlowCallMonitor.isSlow(totalMicros, SLOW_SOLVER_THRESHOLD_MS)
                    || SlowCallMonitor.isSlow(solverMicros, SLOW_SOLVER_THRESHOLD_MS)) {
                SlowCallMonitor.print(
                        "ScoreInHub.updateShooterSolution",
                        totalMicros,
                        String.format(
                                "hasFinalSolution=false distance=%.1f in solveMovingShot=%.3f ms",
                                targetDistanceInches,
                                SlowCallMonitor.toMillis(solverMicros)));
            }
            return false;
        }

        double idealFlywheelCommandIps = idealMovingShotSolution.getFlywheelCommandIps();
        long solverMicros = SlowCallMonitor.nowMicros() - solverStartMicros;
        // Debug dashboard telemetry disabled to reduce NetworkTables traffic.
        // targetDistanceInchesPublisher.set(targetDistanceInches);
        // targetElevationInchesPublisher.set(ShooterConstants.COMMANDED_SCORE_IN_HUB_TARGET_ELEVATION_INCHES);
        // validSolutionPublisher.set(hasSolution);

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
        recordPreviousCycleTargets(commandedHoodAngleDegrees, idealFlywheelCommandIps);
        if (fixedFlywheelStatus == BallTrajectoryLookup.FixedFlywheelShotStatus.VALID) {
            lastValidRobotHeadingTarget = Rotation2d.fromDegrees(movingShotSolution.getRobotHeadingDegrees());
            lastValidTurretDeltaDegrees = clampedTurretDeltaDegrees;
            lastValidFlywheelCommandIps = idealFlywheelCommandIps;
            lastValidSolutionTimestampSeconds = Timer.getFPGATimestamp();
            hasLatchedValidSolution = true;
        } else {
            hasLatchedValidSolution = false;
        }
        long totalMicros = SlowCallMonitor.nowMicros() - functionStartMicros;
        if (SlowCallMonitor.isSlow(totalMicros, SLOW_SOLVER_THRESHOLD_MS)
                || SlowCallMonitor.isSlow(solverMicros, SLOW_SOLVER_THRESHOLD_MS)) {
            SlowCallMonitor.print(
                    "ScoreInHub.updateShooterSolution",
                    totalMicros,
                    String.format(
                            "status=%s distance=%.1f in solveMovingShot=%.3f ms",
                            fixedFlywheelStatus,
                            targetDistanceInches,
                            SlowCallMonitor.toMillis(solverMicros)));
        }
        return true;
    }

    private Rotation2d getPreferredRobotHeading(
            Translation2d futureRobotPosition,
            Translation2d target,
            Rotation2d fallbackHeading) {
        return MovingShotMath.getHeadingTowardTarget(
                target.getX() - futureRobotPosition.getX(),
                target.getY() - futureRobotPosition.getY(),
                fallbackHeading);
    }

    private void publishTarget(Translation2d target) {
        TuningDashboard.publishShootingTarget(target);
    }

    private void publishFutureState() {
        futureFieldPose[0] = futureState.xMeters;
        futureFieldPose[1] = futureState.yMeters;
        futureFieldPose[2] = Math.toDegrees(futureState.headingRadians);
        futureFieldVelocity[0] = futureState.vxMetersPerSecond;
        futureFieldVelocity[1] = futureState.vyMetersPerSecond;
        futureFieldVelocity[2] = Math.toDegrees(futureState.omegaRadiansPerSecond);
        // Debug dashboard telemetry disabled to reduce NetworkTables traffic.
        // futurePosePublisher.set(futureFieldPose);
        // futureVelocityPublisher.set(futureFieldVelocity);
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

    private boolean shouldHoldLastSolution() {
        return hasLatchedValidSolution
                && getLatchedSolutionAgeSeconds() <= TRANSIENT_SOLUTION_HOLD_SECONDS;
    }

    private double getLatchedSolutionAgeSeconds() {
        if (!hasLatchedValidSolution) {
            return Double.POSITIVE_INFINITY;
        }
        return Timer.getFPGATimestamp() - lastValidSolutionTimestampSeconds;
    }

    private Translation2d getCurrentRobotPosition() {
        Pose2d currentPose = poseEstimator.getFusedPoseSupplier().get();
        return currentPose != null ? currentPose.getTranslation() : null;
    }

    private void applyNoSolutionShotCommand(
            Translation2d robotPosition,
            Translation2d target) {
        double fallbackFlywheelCommandIps = getNoSolutionHubFlywheelCommandIps(robotPosition, target);
        shooter.setCoupledIPS(fallbackFlywheelCommandIps);
        recordPreviousCycleTargets(verticalAim.getAngle().in(Degrees), fallbackFlywheelCommandIps);
        horizontalAim.setAngle(Degrees.of(0.0));
        hasLatchedValidSolution = false;
    }

    private double getNoSolutionHubFlywheelCommandIps(
            Translation2d robotPosition,
            Translation2d target) {
        double fallbackDistanceInches = getFallbackHubDistanceInches(robotPosition, target);
        double fallbackFlywheelCommandIps =
                ShortRangeHubFlywheelLookup.getFallbackManifoldFlywheelCommandIps(fallbackDistanceInches);
        return Double.isFinite(fallbackFlywheelCommandIps) ? fallbackFlywheelCommandIps : 0.0;
    }

    private double getFallbackHubDistanceInches(
            Translation2d robotPosition,
            Translation2d target) {
        if (robotPosition != null && target != null) {
            return Inches.convertFrom(target.getDistance(robotPosition), Meters);
        }
        return 0.5 * (
                ShooterConstants.DATA_COLLECTION_SHORT_RANGE_MIN_DISTANCE_INCHES
                        + ShooterConstants.DATA_COLLECTION_SHORT_RANGE_EMPIRICAL_MAX_DISTANCE_INCHES);
    }

    private void clearShotCommands() {
        shooter.setCoupledIPS(0.0);
        recordPreviousCycleTargets(verticalAim.getAngle().in(Degrees), 0.0);
        horizontalAim.setAngle(Degrees.of(0.0));
        hasLatchedValidSolution = false;
    }

    private void applyHeldShotCommand(
            SwerveRequest.FieldCentric fieldCentricRequest,
            double velocityXMetersPerSecond,
            double velocityYMetersPerSecond) {
        horizontalAim.setAngle(Degrees.of(lastValidTurretDeltaDegrees));
        shooter.setCoupledIPS(lastValidFlywheelCommandIps);
        recordPreviousCycleTargets(verticalAim.getAngle().in(Degrees), lastValidFlywheelCommandIps);
        Rotation2d operatorPerspectiveHeadingTarget =
                lastValidRobotHeadingTarget.minus(drivetrain.getDriverPerspectiveForward());
        drivetrain.setControl(
                facingAngleDrive
                        .withVelocityX(velocityXMetersPerSecond)
                        .withVelocityY(velocityYMetersPerSecond)
                        .withTargetDirection(operatorPerspectiveHeadingTarget)
                        .withDeadband(fieldCentricRequest.Deadband)
                        .withRotationalDeadband(fieldCentricRequest.RotationalDeadband)
                        .withCenterOfRotation(fieldCentricRequest.CenterOfRotation)
                        .withDriveRequestType(fieldCentricRequest.DriveRequestType)
                        .withSteerRequestType(fieldCentricRequest.SteerRequestType)
                        .withDesaturateWheelSpeeds(fieldCentricRequest.DesaturateWheelSpeeds)
                        .withForwardPerspective(fieldCentricRequest.ForwardPerspective));
    }

    private void publishTrackingDiagnostics() {
        double currentHoodAngleDegrees = verticalAim.getAngle().in(Degrees);
        double currentFlywheelSpeedIps = shooter.getMainFlywheelSpeedIPS();
        hoodTrackingErrorDegreesPublisher.set(MovingShotMath.computeCommandTrackingError(
                previousCycleMeasuredHoodAngleDegrees,
                previousCycleCommandedHoodAngleDegrees,
                currentHoodAngleDegrees));
        flywheelPredictionErrorIpsPublisher.set(MovingShotMath.computeCommandTrackingError(
                previousCycleMeasuredFlywheelSpeedIps,
                previousCyclePredictedFlywheelSpeedIps,
                currentFlywheelSpeedIps));
    }

    private void recordPreviousCycleTargets(
            double commandedHoodAngleDegrees,
            double commandedFlywheelIps) {
        previousCycleMeasuredHoodAngleDegrees = verticalAim.getAngle().in(Degrees);
        previousCycleCommandedHoodAngleDegrees = commandedHoodAngleDegrees;
        previousCycleMeasuredFlywheelSpeedIps = shooter.getMainFlywheelSpeedIPS();
        previousCyclePredictedFlywheelSpeedIps = MovingShotMath.predictFlywheelSpeedIps(
                previousCycleMeasuredFlywheelSpeedIps,
                commandedFlywheelIps);
        lastCommandedFlywheelSetpointIps = commandedFlywheelIps;
    }

    private void publishPredictedVelocityTelemetry(
            double vxMetersPerSecond,
            double vyMetersPerSecond,
            double omegaRadiansPerSecond) {
        tuningPredictedVelocityXMetersPerSecondPublisher.set(vxMetersPerSecond);
        tuningPredictedVelocityYMetersPerSecondPublisher.set(vyMetersPerSecond);
        tuningPredictedOmegaRadiansPerSecondPublisher.set(omegaRadiansPerSecond);
    }

    private void publishCurrentPinpointVelocityTelemetry() {
        publishPinpointVelocityTelemetry(
                pinpointSubsystem.getVelocityXMetersPerSecond(),
                pinpointSubsystem.getVelocityYMetersPerSecond(),
                pinpointSubsystem.getHeadingVelocityRadiansPerSecond());
    }

    private void publishPinpointVelocityTelemetry(
            double vxMetersPerSecond,
            double vyMetersPerSecond,
            double omegaRadiansPerSecond) {
        tuningPinpointVelocityXMetersPerSecondPublisher.set(vxMetersPerSecond);
        tuningPinpointVelocityYMetersPerSecondPublisher.set(vyMetersPerSecond);
        tuningPinpointOmegaRadiansPerSecondPublisher.set(omegaRadiansPerSecond);
    }

    private void publishEmpiricalSolverTelemetry() {
        if (!empiricalDebugInfo.isValid()) {
            tuningLookupTargetDistanceInchesPublisher.set(Double.NaN);
            tuningSelectedHoodAngleDegreesPublisher.set(Double.NaN);
            tuningModeledFlywheelCommandIpsPublisher.set(Double.NaN);
            tuningCommandedFlywheelCommandIpsPublisher.set(Double.NaN);
            return;
        }

        tuningLookupTargetDistanceInchesPublisher.set(empiricalDebugInfo.getLookupTargetDistanceInches());
        tuningSelectedHoodAngleDegreesPublisher.set(empiricalDebugInfo.getSelectedHoodAngleDegrees());
        tuningModeledFlywheelCommandIpsPublisher.set(empiricalDebugInfo.getModeledFlywheelCommandIps());
        tuningCommandedFlywheelCommandIpsPublisher.set(empiricalDebugInfo.getCommandedFlywheelCommandIps());
    }

    private Translation2d limitTowardHubTravelVelocityForViableShot(
            Translation2d requestedVelocityMetersPerSecond,
            Translation2d futureRobotPosition,
            Translation2d target,
            Rotation2d preferredRobotHeading) {
        TravelVelocityLimitResult limitResult = limitTowardHubTravelVelocityForViableShot(
                requestedVelocityMetersPerSecond,
                futureRobotPosition,
                target,
                MAX_ACTIVE_TRANSLATIONAL_SPEED_METERS_PER_SECOND,
                latchedMaximumTowardHubTravelSpeedMetersPerSecond,
                (robotFieldVxMetersPerSecond, robotFieldVyMetersPerSecond) -> hasViableHubShotForVelocity(
                        robotFieldVxMetersPerSecond,
                        robotFieldVyMetersPerSecond,
                        target,
                        preferredRobotHeading));
        latchedMaximumTowardHubTravelSpeedMetersPerSecond =
                limitResult.getUpdatedMaximumTowardHubTravelSpeedMetersPerSecond();
        return limitResult.getLimitedVelocityMetersPerSecond();
    }

    static TravelVelocityLimitResult limitTowardHubTravelVelocityForViableShot(
            Translation2d requestedVelocityMetersPerSecond,
            Translation2d futureRobotPosition,
            Translation2d target,
            double minimumAllowedTravelSpeedMetersPerSecond,
            double currentMaximumTowardHubTravelSpeedMetersPerSecond,
            HubShotViabilityEvaluator shotViabilityEvaluator) {
        if (requestedVelocityMetersPerSecond == null
                || futureRobotPosition == null
                || target == null
                || shotViabilityEvaluator == null
                || !Double.isFinite(minimumAllowedTravelSpeedMetersPerSecond)
                || minimumAllowedTravelSpeedMetersPerSecond < 0.0) {
            return new TravelVelocityLimitResult(
                    requestedVelocityMetersPerSecond,
                    currentMaximumTowardHubTravelSpeedMetersPerSecond);
        }

        double targetDxMeters = target.getX() - futureRobotPosition.getX();
        double targetDyMeters = target.getY() - futureRobotPosition.getY();
        double targetDistanceMeters = Math.hypot(targetDxMeters, targetDyMeters);
        if (!(targetDistanceMeters > 1e-9)) {
            return new TravelVelocityLimitResult(
                    requestedVelocityMetersPerSecond,
                    currentMaximumTowardHubTravelSpeedMetersPerSecond);
        }

        double radialUnitXMeters = targetDxMeters / targetDistanceMeters;
        double radialUnitYMeters = targetDyMeters / targetDistanceMeters;
        double requestedRadialSpeedMetersPerSecond =
                requestedVelocityMetersPerSecond.getX() * radialUnitXMeters
                        + requestedVelocityMetersPerSecond.getY() * radialUnitYMeters;
        if (!Double.isFinite(requestedRadialSpeedMetersPerSecond)) {
            return new TravelVelocityLimitResult(
                    requestedVelocityMetersPerSecond,
                    currentMaximumTowardHubTravelSpeedMetersPerSecond);
        }
        if (requestedRadialSpeedMetersPerSecond < -1e-9) {
            return new TravelVelocityLimitResult(requestedVelocityMetersPerSecond, Double.POSITIVE_INFINITY);
        }

        double requestedTravelSpeedMetersPerSecond = requestedVelocityMetersPerSecond.getNorm();
        if (!(requestedTravelSpeedMetersPerSecond > 1e-9)) {
            return new TravelVelocityLimitResult(
                    requestedVelocityMetersPerSecond,
                    currentMaximumTowardHubTravelSpeedMetersPerSecond);
        }
        if (requestedRadialSpeedMetersPerSecond <= 1e-9
                || requestedTravelSpeedMetersPerSecond <= minimumAllowedTravelSpeedMetersPerSecond + 1e-9) {
            return new TravelVelocityLimitResult(
                    requestedVelocityMetersPerSecond,
                    currentMaximumTowardHubTravelSpeedMetersPerSecond);
        }

        Translation2d travelUnitVector = requestedVelocityMetersPerSecond.div(requestedTravelSpeedMetersPerSecond);
        double currentMaximumAllowedTravelSpeedMetersPerSecond = Math.max(
                minimumAllowedTravelSpeedMetersPerSecond,
                currentMaximumTowardHubTravelSpeedMetersPerSecond);
        double candidateTravelSpeedMetersPerSecond = Math.min(
                requestedTravelSpeedMetersPerSecond,
                currentMaximumAllowedTravelSpeedMetersPerSecond);
        if (shotViabilityEvaluator.hasViableShot(
                travelUnitVector.getX() * candidateTravelSpeedMetersPerSecond,
                travelUnitVector.getY() * candidateTravelSpeedMetersPerSecond)) {
            if (requestedTravelSpeedMetersPerSecond <= currentMaximumAllowedTravelSpeedMetersPerSecond + 1e-9) {
                return new TravelVelocityLimitResult(
                        requestedVelocityMetersPerSecond,
                        currentMaximumTowardHubTravelSpeedMetersPerSecond);
            }
            return new TravelVelocityLimitResult(
                    travelUnitVector.times(candidateTravelSpeedMetersPerSecond),
                    currentMaximumTowardHubTravelSpeedMetersPerSecond);
        }

        if (!shotViabilityEvaluator.hasViableShot(
                travelUnitVector.getX() * minimumAllowedTravelSpeedMetersPerSecond,
                travelUnitVector.getY() * minimumAllowedTravelSpeedMetersPerSecond)) {
            return new TravelVelocityLimitResult(
                    travelUnitVector.times(minimumAllowedTravelSpeedMetersPerSecond),
                    minimumAllowedTravelSpeedMetersPerSecond);
        }

        double lowTravelSpeedMetersPerSecond = minimumAllowedTravelSpeedMetersPerSecond;
        double highTravelSpeedMetersPerSecond = candidateTravelSpeedMetersPerSecond;
        double bestViableTravelSpeedMetersPerSecond = lowTravelSpeedMetersPerSecond;
        for (int iteration = 0; iteration < RADIAL_SPEED_LIMIT_SEARCH_MAX_ITERATIONS; iteration++) {
            if (highTravelSpeedMetersPerSecond - lowTravelSpeedMetersPerSecond
                    <= RADIAL_SPEED_LIMIT_SEARCH_TOLERANCE_METERS_PER_SECOND) {
                break;
            }

            double midTravelSpeedMetersPerSecond =
                    0.5 * (lowTravelSpeedMetersPerSecond + highTravelSpeedMetersPerSecond);
            boolean hasViableShot = shotViabilityEvaluator.hasViableShot(
                    travelUnitVector.getX() * midTravelSpeedMetersPerSecond,
                    travelUnitVector.getY() * midTravelSpeedMetersPerSecond);
            if (hasViableShot) {
                bestViableTravelSpeedMetersPerSecond = midTravelSpeedMetersPerSecond;
                lowTravelSpeedMetersPerSecond = midTravelSpeedMetersPerSecond;
            } else {
                highTravelSpeedMetersPerSecond = midTravelSpeedMetersPerSecond;
            }
        }

        double updatedMaximumTowardHubTravelSpeedMetersPerSecond = Math.min(
                currentMaximumAllowedTravelSpeedMetersPerSecond,
                Math.max(
                        minimumAllowedTravelSpeedMetersPerSecond,
                        RADIAL_SPEED_LIMIT_VIABLE_MARGIN * bestViableTravelSpeedMetersPerSecond));
        double limitedTravelSpeedMetersPerSecond = Math.min(
                requestedTravelSpeedMetersPerSecond,
                updatedMaximumTowardHubTravelSpeedMetersPerSecond);
        return new TravelVelocityLimitResult(
                travelUnitVector.times(limitedTravelSpeedMetersPerSecond),
                updatedMaximumTowardHubTravelSpeedMetersPerSecond);
    }

    private boolean hasViableHubShotForVelocity(
            double robotFieldVxMetersPerSecond,
            double robotFieldVyMetersPerSecond,
            Translation2d target,
            Rotation2d preferredRobotHeading) {
        double targetDistanceInches = Inches.convertFrom(
                target.getDistance(new Translation2d(futureState.xMeters, futureState.yMeters)),
                Meters);
        if (!MovingShotMath.shouldUseEmpiricalHubMovingShotModel(
                targetDistanceInches,
                ShooterConstants.COMMANDED_SCORE_IN_HUB_TARGET_ELEVATION_INCHES)) {
            return true;
        }

        return MovingShotMath.solveIdealMovingShotWithUpperHoodFallback(
                verticalAim.getMinimumAngle().in(Degrees),
                verticalAim.getMaximumAngle().in(Degrees),
                verticalAim.getAngle().in(Degrees),
                ShooterConstants.COMMANDED_MOVING_SHOT_HOOD_SEARCH_STEP_DEGREES,
                futureState.xMeters,
                futureState.yMeters,
                futureState.headingRadians,
                robotFieldVxMetersPerSecond,
                robotFieldVyMetersPerSecond,
                target.getX(),
                target.getY(),
                ShooterConstants.COMMANDED_SCORE_IN_HUB_TARGET_ELEVATION_INCHES,
                ShooterConstants.COMMANDED_MAXIMUM_SHOOTING_HEIGHT_INCHES,
                preferredRobotHeading.getRadians(),
                horizontalAim.getMinimumAngle().in(Degrees),
                horizontalAim.getMaximumAngle().in(Degrees),
                shooter.getMainFlywheelSpeedIPS(),
                lastCommandedFlywheelSetpointIps,
                true,
                radialSpeedLimitSearchSolution,
                null);
    }

    private Translation2d operatorPerspectiveToFieldRelativeVelocity(Translation2d operatorPerspectiveVelocity) {
        if (operatorPerspectiveVelocity == null) {
            return null;
        }
        return operatorPerspectiveVelocity.rotateBy(drivetrain.getDriverPerspectiveForward());
    }

    private Translation2d fieldRelativeToOperatorPerspectiveVelocity(Translation2d fieldRelativeVelocity) {
        if (fieldRelativeVelocity == null) {
            return null;
        }
        return fieldRelativeVelocity.rotateBy(drivetrain.getDriverPerspectiveForward().unaryMinus());
    }

    private static Translation2d clampTowardHubTravelSpeed(
            Translation2d requestedVelocity,
            Translation2d robotPosition,
            Translation2d target,
            double maximumTowardHubTravelSpeedMetersPerSecond) {
        if (requestedVelocity == null
                || robotPosition == null
                || target == null
                || !Double.isFinite(maximumTowardHubTravelSpeedMetersPerSecond)
                || maximumTowardHubTravelSpeedMetersPerSecond < 0.0) {
            return requestedVelocity;
        }

        double targetDxMeters = target.getX() - robotPosition.getX();
        double targetDyMeters = target.getY() - robotPosition.getY();
        double targetDistanceMeters = Math.hypot(targetDxMeters, targetDyMeters);
        if (!(targetDistanceMeters > 1e-9)) {
            return requestedVelocity;
        }

        double radialUnitXMeters = targetDxMeters / targetDistanceMeters;
        double radialUnitYMeters = targetDyMeters / targetDistanceMeters;
        double requestedRadialSpeedMetersPerSecond =
                requestedVelocity.getX() * radialUnitXMeters
                        + requestedVelocity.getY() * radialUnitYMeters;
        if (!(requestedRadialSpeedMetersPerSecond > 1e-9)) {
            return requestedVelocity;
        }
        double requestedTravelSpeedMetersPerSecond = requestedVelocity.getNorm();
        if (!(requestedTravelSpeedMetersPerSecond > maximumTowardHubTravelSpeedMetersPerSecond)) {
            return requestedVelocity;
        }
        return requestedVelocity.times(
                maximumTowardHubTravelSpeedMetersPerSecond / requestedTravelSpeedMetersPerSecond);
    }

    private static boolean isDrivingAwayFromHub(
            Translation2d requestedVelocity,
            Translation2d robotPosition,
            Translation2d target) {
        if (requestedVelocity == null || robotPosition == null || target == null) {
            return false;
        }

        double targetDxMeters = target.getX() - robotPosition.getX();
        double targetDyMeters = target.getY() - robotPosition.getY();
        double targetDistanceMeters = Math.hypot(targetDxMeters, targetDyMeters);
        if (!(targetDistanceMeters > 1e-9)) {
            return false;
        }

        double requestedRadialSpeedMetersPerSecond =
                (requestedVelocity.getX() * targetDxMeters + requestedVelocity.getY() * targetDyMeters)
                        / targetDistanceMeters;
        return requestedRadialSpeedMetersPerSecond < -1e-9;
    }

    private Translation2d limitTranslationalAcceleration(SwerveRequest.FieldCentric fieldCentricRequest) {
        double requestedVelocityXMetersPerSecond = fieldCentricRequest.VelocityX;
        double requestedVelocityYMetersPerSecond = fieldCentricRequest.VelocityY;
        Translation2d requestedVelocity = new Translation2d(
                requestedVelocityXMetersPerSecond,
                requestedVelocityYMetersPerSecond);
        if (!ENABLE_SPEED_UP_ACCELERATION_LIMIT) {
            lastVelocityLimitTimestampSeconds = Double.NaN;
            return requestedVelocity;
        }

        double currentTimestampSeconds = Timer.getFPGATimestamp();

        if (!Double.isFinite(lastVelocityLimitTimestampSeconds)) {
            lastVelocityLimitTimestampSeconds = currentTimestampSeconds;
            return requestedVelocity;
        }

        double dtSeconds = Math.max(0.0, currentTimestampSeconds - lastVelocityLimitTimestampSeconds);
        Translation2d limitedVelocity = requestedVelocity;
        double currentRobotSpeedMetersPerSecond = Math.hypot(
                drivetrain.getState().Speeds.vxMetersPerSecond,
                drivetrain.getState().Speeds.vyMetersPerSecond);

        double requestedSpeedMetersPerSecond = requestedVelocity.getNorm();
        if (requestedSpeedMetersPerSecond > currentRobotSpeedMetersPerSecond && dtSeconds > 0.0) {
            double maximumSpeedIncreaseMetersPerSecond =
                    SPEED_UP_ACCELERATION_LIMIT_METERS_PER_SECOND_SQUARED * dtSeconds;
            double limitedSpeedMetersPerSecond = Math.min(
                    requestedSpeedMetersPerSecond,
                    currentRobotSpeedMetersPerSecond + maximumSpeedIncreaseMetersPerSecond);
            if (requestedSpeedMetersPerSecond > 1e-9) {
                limitedVelocity = new Translation2d(
                        limitedSpeedMetersPerSecond,
                        requestedVelocity.getAngle());
            }
        }

        lastVelocityLimitTimestampSeconds = currentTimestampSeconds;
        return limitedVelocity;
    }
}
