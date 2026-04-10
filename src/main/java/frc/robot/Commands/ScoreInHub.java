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
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SparkAnglePositionSubsystem;
import frc.robot.utility.SlowCallMonitor;
import frc.robot.utility.TuningDashboard;

/**
 * Drives while solving a moving shot into the fixed hub target.
 *
 * Frame suffixes used in this command:
 * Fld = field frame and the default for internal geometry/solver math.
 * Drv = driver-perspective field-centric request frame at the Phoenix request boundary.
 * Rbt = robot/body frame.
 */
public class ScoreInHub extends Command {
    private static final double SLOW_EXECUTE_THRESHOLD_MS = 8.0;
    private static final double SLOW_PREDICTION_THRESHOLD_MS = 4.0;
    private static final double SLOW_SOLVER_THRESHOLD_MS = 5.0;
    private static final double SLOW_SET_CONTROL_THRESHOLD_MS = 2.0;
    private static final double TRANSIENT_SOLUTION_HOLD_SECONDS = 0.25;
    private static final double MAX_TRANSLATIONAL_SPEED_METERS_PER_SECOND = 0.75;
    private static final boolean ENABLE_SPEED_UP_ACCELERATION_LIMIT = false;
    private static final double SPEED_UP_ACCELERATION_LIMIT_METERS_PER_SECOND_SQUARED = 0.1524;

    private final CommandSwerveDrivetrain drivetrain;
    private final PoseEstimatorSubsystem poseEstimator;
    private final SparkAnglePositionSubsystem horizontalAim;
    private final SparkAnglePositionSubsystem verticalAim;
    private final ShooterSubsystem shooter;
    private final Supplier<SwerveRequest> driveRequestSupplier;
    private final Supplier<Translation2d> targetSupplier;
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
    private final edu.wpi.first.networktables.DoubleArrayPublisher futurePoseFldPublisher;
    private final edu.wpi.first.networktables.DoubleArrayPublisher futureVelocityFldPublisher;
    private final BooleanPublisher validSolutionPublisher;
    private final StringPublisher fieldTypePublisher;
    private final PoseEstimatorSubsystem.PredictedFusedState futureStateFld =
            new PoseEstimatorSubsystem.PredictedFusedState();
    private final BallTrajectoryLookup.MovingShotSolution idealMovingShotSolution =
            new BallTrajectoryLookup.MovingShotSolution();
    private final BallTrajectoryLookup.MovingShotSolution movingShotSolution =
            new BallTrajectoryLookup.MovingShotSolution();
    private final double[] futurePoseFldArray = new double[3];
    private final double[] futureVelocityFldArray = new double[3];
    private Rotation2d lastValidRobotHeadingTargetFld = new Rotation2d();
    private double lastValidTurretDeltaDegrees = 0.0;
    private double lastValidHoodAngleDegrees = Double.NaN;
    private double lastValidFlywheelCommandIps = 0.0;
    private double lastValidSolutionTimestampSeconds = Double.NEGATIVE_INFINITY;
    private boolean hasLatchedValidSolution = false;
    private double lastVelocityLimitTimestampSeconds = Double.NaN;
    private double previousCycleMeasuredHoodAngleDegrees = Double.NaN;
    private double previousCycleCommandedHoodAngleDegrees = Double.NaN;
    private double previousCycleMeasuredFlywheelSpeedIps = Double.NaN;
    private double previousCyclePredictedFlywheelSpeedIps = Double.NaN;

    public ScoreInHub(
            CommandSwerveDrivetrain drivetrain,
            PoseEstimatorSubsystem poseEstimator,
            SparkAnglePositionSubsystem horizontalAim,
            SparkAnglePositionSubsystem verticalAim,
            ShooterSubsystem shooter,
            Supplier<SwerveRequest> driveRequestSupplier,
            Supplier<Translation2d> targetSupplier) {
        this.drivetrain = Objects.requireNonNull(drivetrain, "drivetrain must not be null");
        this.poseEstimator = Objects.requireNonNull(poseEstimator, "poseEstimator must not be null");
        this.horizontalAim = Objects.requireNonNull(horizontalAim, "horizontalAim must not be null");
        this.verticalAim = Objects.requireNonNull(verticalAim, "verticalAim must not be null");
        this.shooter = Objects.requireNonNull(shooter, "shooter must not be null");
        this.driveRequestSupplier =
                Objects.requireNonNull(driveRequestSupplier, "driveRequestSupplier must not be null");
        this.targetSupplier = Objects.requireNonNull(targetSupplier, "targetSupplier must not be null");
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
        futurePoseFldPublisher = scoreTable.getDoubleArrayTopic("futurePose").publish();
        futureVelocityFldPublisher = scoreTable.getDoubleArrayTopic("futureVelocity").publish();
        validSolutionPublisher = scoreTable.getBooleanTopic("validSolution").publish();
        fieldTypePublisher = scoreTable.getStringTopic(".type").publish();
        // Debug dashboard telemetry disabled to reduce NetworkTables traffic.
        // fieldTypePublisher.set("Field2d");

        facingAngleDrive.withHeadingPID(5.0, 0.0, 0.0);
        facingAngleDrive.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(this.drivetrain, horizontalAim, verticalAim, shooter);
    }

    @Override
    public void initialize() {
        lastVelocityLimitTimestampSeconds = Double.NaN;
        lastValidHoodAngleDegrees = verticalAim.getAngle().in(Degrees);
        previousCycleMeasuredHoodAngleDegrees = Double.NaN;
        previousCycleCommandedHoodAngleDegrees = Double.NaN;
        previousCycleMeasuredFlywheelSpeedIps = Double.NaN;
        previousCyclePredictedFlywheelSpeedIps = Double.NaN;
        hoodTrackingErrorDegreesPublisher.set(Double.NaN);
        flywheelPredictionErrorIpsPublisher.set(Double.NaN);
    }

    @Override
    public void execute() {
        long executeStartMicros = SlowCallMonitor.nowMicros();
        long predictionMicros = 0L;
        long solutionMicros = 0L;
        long setControlMicros = 0L;
        Translation2d targetFld = getTargetFld();

        SwerveRequest requestedDrive = driveRequestSupplier.get();
        if (!(requestedDrive instanceof SwerveRequest.FieldCentric fieldCentricRequestDrv)) {
            clearSolutionTelemetry();
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

        double limitedVelocityXDrvMetersPerSecond;
        double limitedVelocityYDrvMetersPerSecond;
        {
            Translation2d limitedVelocityDrvMps = limitDriverFrameTranslationalAcceleration(fieldCentricRequestDrv);
            limitedVelocityXDrvMetersPerSecond = limitedVelocityDrvMps.getX();
            limitedVelocityYDrvMetersPerSecond = limitedVelocityDrvMps.getY();
        }

        long predictionStartMicros = SlowCallMonitor.nowMicros();
        if (!poseEstimator.getPredictedFusedState(
                ShooterConstants.COMMANDED_SHOOTER_LOOKAHEAD_SECONDS,
                futureStateFld)) {
            predictionMicros = SlowCallMonitor.nowMicros() - predictionStartMicros;
            clearSolutionTelemetry();
            boolean holdingLastSolution = shouldHoldLastSolution();
            if (holdingLastSolution) {
                applyHeldShotCommand(
                        fieldCentricRequestDrv,
                        limitedVelocityXDrvMetersPerSecond,
                        limitedVelocityYDrvMetersPerSecond);
            } else {
                applyNoSolutionShotCommand(getCurrentRobotTranslationFld(), targetFld);
                drivetrain.setControl(fieldCentricRequestDrv
                        .withVelocityX(limitedVelocityXDrvMetersPerSecond)
                        .withVelocityY(limitedVelocityYDrvMetersPerSecond));
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
        publishFutureFieldState();

        Pose2d futurePoseFld = new Pose2d(
                futureStateFld.xMeters,
                futureStateFld.yMeters,
                Rotation2d.fromRadians(futureStateFld.headingRadians));
        publishTargetFld(targetFld);

        Rotation2d preferredRobotHeadingFld = getPreferredRobotHeadingFld(
                futurePoseFld.getTranslation(),
                targetFld,
                futurePoseFld.getRotation());
        Rotation2d robotHeadingTargetFld = preferredRobotHeadingFld;
        long solutionStartMicros = SlowCallMonitor.nowMicros();
        if (updateShooterSolution(targetFld, preferredRobotHeadingFld)) {
            robotHeadingTargetFld = Rotation2d.fromDegrees(movingShotSolution.getRobotHeadingDegrees());
        } else if (shouldHoldLastSolution()) {
            applyHeldShotCommand(
                    fieldCentricRequestDrv,
                    limitedVelocityXDrvMetersPerSecond,
                    limitedVelocityYDrvMetersPerSecond);
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
            applyNoSolutionShotCommand(futurePoseFld.getTranslation(), targetFld);
        }
        solutionMicros = SlowCallMonitor.nowMicros() - solutionStartMicros;
        Rotation2d robotHeadingTargetDrv =
                robotHeadingTargetFld.minus(drivetrain.getDriverPerspectiveForward());

        long setControlStartMicros = SlowCallMonitor.nowMicros();
        drivetrain.setControl(
                facingAngleDrive
                        .withVelocityX(limitedVelocityXDrvMetersPerSecond)
                        .withVelocityY(limitedVelocityYDrvMetersPerSecond)
                        .withTargetDirection(robotHeadingTargetDrv)
                        .withDeadband(fieldCentricRequestDrv.Deadband)
                        .withRotationalDeadband(fieldCentricRequestDrv.RotationalDeadband)
                        .withCenterOfRotation(fieldCentricRequestDrv.CenterOfRotation)
                        .withDriveRequestType(fieldCentricRequestDrv.DriveRequestType)
                        .withSteerRequestType(fieldCentricRequestDrv.SteerRequestType)
                        .withDesaturateWheelSpeeds(fieldCentricRequestDrv.DesaturateWheelSpeeds)
                        .withForwardPerspective(fieldCentricRequestDrv.ForwardPerspective));
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
    }

    private Translation2d getTargetFld() {
        Translation2d suppliedTargetFld = targetSupplier.get();
        if (suppliedTargetFld != null) {
            return suppliedTargetFld;
        }
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        return FieldMath.getHubTarget(alliance);
    }

    private boolean updateShooterSolution(
            Translation2d targetFld,
            Rotation2d preferredRobotHeadingFld) {
        long functionStartMicros = SlowCallMonitor.nowMicros();
        double minimumHoodAngleDegrees = verticalAim.getMinimumAngle().in(Degrees);
        double maximumHoodAngleDegrees = verticalAim.getMaximumAngle().in(Degrees);
        double targetDistanceInches = Inches.convertFrom(
                targetFld.getDistance(new Translation2d(futureStateFld.xMeters, futureStateFld.yMeters)),
                Meters);
        long solverStartMicros = SlowCallMonitor.nowMicros();
        BallTrajectoryLookup.FixedFlywheelShotStatus fixedFlywheelStatus =
                MovingShotMath.solveCommandedMovingShot(
                        verticalAim,
                        ShooterConstants.COMMANDED_MOVING_SHOT_HOOD_SEARCH_STEP_DEGREES,
                        ShooterConstants.COMMANDED_MOVING_SHOT_FIXED_FLYWHEEL_HOOD_SEARCH_STEP_DEGREES,
                        futureStateFld,
                        targetFld,
                        ShooterConstants.COMMANDED_SCORE_IN_HUB_TARGET_ELEVATION_INCHES,
                        ShooterConstants.COMMANDED_MAXIMUM_SHOOTING_HEIGHT_INCHES,
                        preferredRobotHeadingFld.getRadians(),
                        horizontalAim.getMinimumAngle().in(Degrees),
                        horizontalAim.getMaximumAngle().in(Degrees),
                        shooter.getMainFlywheelSpeedIPS(),
                        idealMovingShotSolution,
                        movingShotSolution);
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

        BallTrajectoryLookup.MovingShotSolution commandedShotSolution =
                fixedFlywheelStatus == BallTrajectoryLookup.FixedFlywheelShotStatus.VALID
                        ? movingShotSolution
                        : idealMovingShotSolution;
        double commandedHoodAngleDegrees = getCommandedTrackingHoodAngleDegrees(
                fixedFlywheelStatus,
                minimumHoodAngleDegrees,
                maximumHoodAngleDegrees,
                commandedShotSolution);
        double clampedTurretDeltaDegrees = MovingShotMath.clampTurretDeltaDegrees(
                commandedShotSolution.getTurretDeltaDegrees(),
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
            lastValidRobotHeadingTargetFld = Rotation2d.fromDegrees(commandedShotSolution.getRobotHeadingDegrees());
            lastValidTurretDeltaDegrees = clampedTurretDeltaDegrees;
            lastValidHoodAngleDegrees = commandedHoodAngleDegrees;
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

    private Rotation2d getPreferredRobotHeadingFld(
            Translation2d futureRobotTranslationFld,
            Translation2d targetFld,
            Rotation2d fallbackHeadingFld) {
        return MovingShotMath.getHeadingTowardTarget(
                targetFld.getX() - futureRobotTranslationFld.getX(),
                targetFld.getY() - futureRobotTranslationFld.getY(),
                fallbackHeadingFld);
    }

    private void publishTargetFld(Translation2d targetFld) {
        TuningDashboard.publishShootingTarget(targetFld);
    }

    private void publishFutureFieldState() {
        futurePoseFldArray[0] = futureStateFld.xMeters;
        futurePoseFldArray[1] = futureStateFld.yMeters;
        futurePoseFldArray[2] = Math.toDegrees(futureStateFld.headingRadians);
        futureVelocityFldArray[0] = futureStateFld.vxMetersPerSecond;
        futureVelocityFldArray[1] = futureStateFld.vyMetersPerSecond;
        futureVelocityFldArray[2] = Math.toDegrees(futureStateFld.omegaRadiansPerSecond);
        // Debug dashboard telemetry disabled to reduce NetworkTables traffic.
        // futurePoseFldPublisher.set(futurePoseFldArray);
        // futureVelocityFldPublisher.set(futureVelocityFldArray);
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

    private Translation2d getCurrentRobotTranslationFld() {
        Pose2d currentPoseFld = poseEstimator.getFusedPoseSupplier().get();
        return currentPoseFld != null ? currentPoseFld.getTranslation() : null;
    }

    private void applyNoSolutionShotCommand(
            Translation2d robotTranslationFld,
            Translation2d targetFld) {
        double fallbackDistanceInches = getFallbackHubDistanceInches(robotTranslationFld, targetFld);
        double fallbackFlywheelCommandIps = getNoSolutionHubFlywheelCommandIps(robotTranslationFld, targetFld);
        double fallbackHoodAngleDegrees = ShortRangeHubFlywheelLookup.getIdealHoodAngleDegrees(fallbackDistanceInches);
        if (!Double.isFinite(fallbackHoodAngleDegrees)) {
            fallbackHoodAngleDegrees = verticalAim.getMaximumAngle().in(Degrees);
        }
        fallbackHoodAngleDegrees = Math.max(
                verticalAim.getMinimumAngle().in(Degrees),
                Math.min(verticalAim.getMaximumAngle().in(Degrees), fallbackHoodAngleDegrees));
        verticalAim.setAngle(Degrees.of(fallbackHoodAngleDegrees));
        shooter.setCoupledIPS(fallbackFlywheelCommandIps);
        recordPreviousCycleTargets(verticalAim.getAngle().in(Degrees), fallbackFlywheelCommandIps);
        horizontalAim.setAngle(Degrees.of(0.0));
        hasLatchedValidSolution = false;
    }

    private double getNoSolutionHubFlywheelCommandIps(
            Translation2d robotTranslationFld,
            Translation2d targetFld) {
        double fallbackDistanceInches = getFallbackHubDistanceInches(robotTranslationFld, targetFld);
        double fallbackFlywheelCommandIps =
                ShortRangeHubFlywheelLookup.getFallbackManifoldFlywheelCommandIps(fallbackDistanceInches);
        return Double.isFinite(fallbackFlywheelCommandIps) ? fallbackFlywheelCommandIps : 0.0;
    }

    private double getFallbackHubDistanceInches(
            Translation2d robotTranslationFld,
            Translation2d targetFld) {
        if (robotTranslationFld != null && targetFld != null) {
            return Inches.convertFrom(targetFld.getDistance(robotTranslationFld), Meters);
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
            SwerveRequest.FieldCentric fieldCentricRequestDrv,
            double velocityXDrvMetersPerSecond,
            double velocityYDrvMetersPerSecond) {
        if (Double.isFinite(lastValidHoodAngleDegrees)) {
            verticalAim.setAngle(Degrees.of(lastValidHoodAngleDegrees));
        }
        horizontalAim.setAngle(Degrees.of(lastValidTurretDeltaDegrees));
        shooter.setCoupledIPS(lastValidFlywheelCommandIps);
        recordPreviousCycleTargets(verticalAim.getAngle().in(Degrees), lastValidFlywheelCommandIps);
        Rotation2d robotHeadingTargetDrv =
                lastValidRobotHeadingTargetFld.minus(drivetrain.getDriverPerspectiveForward());
        drivetrain.setControl(
                facingAngleDrive
                        .withVelocityX(velocityXDrvMetersPerSecond)
                        .withVelocityY(velocityYDrvMetersPerSecond)
                        .withTargetDirection(robotHeadingTargetDrv)
                        .withDeadband(fieldCentricRequestDrv.Deadband)
                        .withRotationalDeadband(fieldCentricRequestDrv.RotationalDeadband)
                        .withCenterOfRotation(fieldCentricRequestDrv.CenterOfRotation)
                        .withDriveRequestType(fieldCentricRequestDrv.DriveRequestType)
                        .withSteerRequestType(fieldCentricRequestDrv.SteerRequestType)
                        .withDesaturateWheelSpeeds(fieldCentricRequestDrv.DesaturateWheelSpeeds)
                        .withForwardPerspective(fieldCentricRequestDrv.ForwardPerspective));
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
    }

    private Translation2d limitDriverFrameTranslationalAcceleration(
            SwerveRequest.FieldCentric fieldCentricRequestDrv) {
        Translation2d requestedVelocityDrvMps = clampVelocityMagnitude(new Translation2d(
                fieldCentricRequestDrv.VelocityX,
                fieldCentricRequestDrv.VelocityY),
                MAX_TRANSLATIONAL_SPEED_METERS_PER_SECOND);
        if (!ENABLE_SPEED_UP_ACCELERATION_LIMIT) {
            lastVelocityLimitTimestampSeconds = Double.NaN;
            return requestedVelocityDrvMps;
        }

        double currentTimestampSeconds = Timer.getFPGATimestamp();

        if (!Double.isFinite(lastVelocityLimitTimestampSeconds)) {
            lastVelocityLimitTimestampSeconds = currentTimestampSeconds;
            return requestedVelocityDrvMps;
        }

        double dtSeconds = Math.max(0.0, currentTimestampSeconds - lastVelocityLimitTimestampSeconds);
        Translation2d limitedVelocityDrvMps = requestedVelocityDrvMps;
        double currentRobotSpeedMetersPerSecond = Math.hypot(
                drivetrain.getState().Speeds.vxMetersPerSecond,
                drivetrain.getState().Speeds.vyMetersPerSecond);

        double requestedSpeedMetersPerSecond = requestedVelocityDrvMps.getNorm();
        if (requestedSpeedMetersPerSecond > currentRobotSpeedMetersPerSecond && dtSeconds > 0.0) {
            double maximumSpeedIncreaseMetersPerSecond =
                    SPEED_UP_ACCELERATION_LIMIT_METERS_PER_SECOND_SQUARED * dtSeconds;
            double limitedSpeedMetersPerSecond = Math.min(
                    requestedSpeedMetersPerSecond,
                    currentRobotSpeedMetersPerSecond + maximumSpeedIncreaseMetersPerSecond);
            if (requestedSpeedMetersPerSecond > 1e-9) {
                limitedVelocityDrvMps = new Translation2d(
                        limitedSpeedMetersPerSecond,
                        requestedVelocityDrvMps.getAngle());
            }
        }

        lastVelocityLimitTimestampSeconds = currentTimestampSeconds;
        return limitedVelocityDrvMps;
    }

    private static Translation2d clampVelocityMagnitude(Translation2d velocity, double maximumSpeedMetersPerSecond) {
        double speedMetersPerSecond = velocity.getNorm();
        if (speedMetersPerSecond <= maximumSpeedMetersPerSecond || speedMetersPerSecond <= 1e-9) {
            return velocity;
        }
        return velocity.times(maximumSpeedMetersPerSecond / speedMetersPerSecond);
    }

    private static double getCommandedTrackingHoodAngleDegrees(
            BallTrajectoryLookup.FixedFlywheelShotStatus fixedFlywheelStatus,
            double minimumHoodAngleDegrees,
            double maximumHoodAngleDegrees,
            BallTrajectoryLookup.MovingShotSolution commandedShotSolution) {
        double solvedHoodAngleDegrees = commandedShotSolution.getHoodAngleDegrees();
        if (Double.isFinite(solvedHoodAngleDegrees)) {
            return Math.max(
                    minimumHoodAngleDegrees,
                    Math.min(maximumHoodAngleDegrees, solvedHoodAngleDegrees));
        }
        return MovingShotMath.getCommandedHoodAngleDegrees(
                fixedFlywheelStatus,
                minimumHoodAngleDegrees,
                maximumHoodAngleDegrees,
                commandedShotSolution);
    }
}
