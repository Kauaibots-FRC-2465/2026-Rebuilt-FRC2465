package frc.robot.Commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.Objects;
import java.util.function.DoubleSupplier;
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
 */
public class ScoreInHub extends Command {
    private static final double SLOW_EXECUTE_THRESHOLD_MS = 8.0;
    private static final double SLOW_PREDICTION_THRESHOLD_MS = 4.0;
    private static final double SLOW_SOLVER_THRESHOLD_MS = 5.0;
    private static final double SLOW_SET_CONTROL_THRESHOLD_MS = 2.0;
    private static final double TRANSIENT_SOLUTION_HOLD_SECONDS = 0.25;
    private static final boolean ENABLE_SPEED_UP_ACCELERATION_LIMIT = false;
    private static final double SPEED_UP_ACCELERATION_LIMIT_METERS_PER_SECOND_SQUARED = 0.1524;

    private final CommandSwerveDrivetrain drivetrain;
    private final PoseEstimatorSubsystem poseEstimator;
    private final SparkAnglePositionSubsystem horizontalAim;
    private final SparkAnglePositionSubsystem verticalAim;
    private final ShooterSubsystem shooter;
    private final Supplier<SwerveRequest> driveRequestSupplier;
    private final DoubleSupplier azimuthTrimDegreesSupplier;
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
    private final double[] futureFieldPose = new double[3];
    private final double[] futureFieldVelocity = new double[3];
    private Translation2d lastFieldRelativeDriveDirection = new Translation2d();
    private Rotation2d lastValidRobotHeadingTarget = new Rotation2d();
    private double lastValidTurretDeltaDegrees = 0.0;
    private double lastValidFlywheelCommandIps = 0.0;
    private double lastValidSolutionTimestampSeconds = Double.NEGATIVE_INFINITY;
    private boolean hasLatchedValidSolution = false;
    private double lastVelocityLimitTimestampSeconds = Double.NaN;

    public ScoreInHub(
            CommandSwerveDrivetrain drivetrain,
            PoseEstimatorSubsystem poseEstimator,
            SparkAnglePositionSubsystem horizontalAim,
            SparkAnglePositionSubsystem verticalAim,
            ShooterSubsystem shooter,
            Supplier<SwerveRequest> driveRequestSupplier,
            DoubleSupplier azimuthTrimDegreesSupplier) {
        this.drivetrain = Objects.requireNonNull(drivetrain, "drivetrain must not be null");
        this.poseEstimator = Objects.requireNonNull(poseEstimator, "poseEstimator must not be null");
        this.horizontalAim = Objects.requireNonNull(horizontalAim, "horizontalAim must not be null");
        this.verticalAim = Objects.requireNonNull(verticalAim, "verticalAim must not be null");
        this.shooter = Objects.requireNonNull(shooter, "shooter must not be null");
        this.driveRequestSupplier =
                Objects.requireNonNull(driveRequestSupplier, "driveRequestSupplier must not be null");
        this.azimuthTrimDegreesSupplier =
                Objects.requireNonNull(azimuthTrimDegreesSupplier, "azimuthTrimDegreesSupplier must not be null");
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
        futurePosePublisher = scoreTable.getDoubleArrayTopic("futurePose").publish();
        futureVelocityPublisher = scoreTable.getDoubleArrayTopic("futureVelocity").publish();
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
    }

    @Override
    public void execute() {
        long executeStartMicros = SlowCallMonitor.nowMicros();
        long predictionMicros = 0L;
        long solutionMicros = 0L;
        long setControlMicros = 0L;

        SwerveRequest requestedDrive = driveRequestSupplier.get();
        if (!(requestedDrive instanceof SwerveRequest.FieldCentric fieldCentricRequest)) {
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

        double limitedVelocityXMetersPerSecond;
        double limitedVelocityYMetersPerSecond;
        {
            Translation2d limitedVelocity = limitTranslationalAcceleration(fieldCentricRequest);
            limitedVelocityXMetersPerSecond = limitedVelocity.getX();
            limitedVelocityYMetersPerSecond = limitedVelocity.getY();
        }

        updateLastDriveDirection(limitedVelocityXMetersPerSecond, limitedVelocityYMetersPerSecond);
        long predictionStartMicros = SlowCallMonitor.nowMicros();
        if (!poseEstimator.getPredictedFusedState(
                ShooterConstants.COMMANDED_SHOOTER_LOOKAHEAD_SECONDS,
                futureState)) {
            predictionMicros = SlowCallMonitor.nowMicros() - predictionStartMicros;
            clearSolutionTelemetry();
            boolean holdingLastSolution = shouldHoldLastSolution();
            if (holdingLastSolution) {
                applyHeldShotCommand(
                        fieldCentricRequest,
                        limitedVelocityXMetersPerSecond,
                        limitedVelocityYMetersPerSecond);
            } else {
                clearShotCommands();
                drivetrain.setControl(requestedDrive);
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
        publishFutureState();

        Pose2d futurePose = new Pose2d(
                futureState.xMeters,
                futureState.yMeters,
                Rotation2d.fromRadians(futureState.headingRadians));
        Translation2d target = getTarget();
        publishTarget(target);

        Rotation2d preferredRobotHeading = getPreferredRobotHeading(futurePose.getRotation());
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
            clearShotCommands();
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
        boolean hasIdealSolution = MovingShotMath.solveIdealMovingShotWithUpperHoodFallback(
                verticalAim,
                ShooterConstants.COMMANDED_MOVING_SHOT_HOOD_SEARCH_STEP_DEGREES,
                futureState.xMeters,
                futureState.yMeters,
                futureState.headingRadians,
                futureState.vxMetersPerSecond,
                futureState.vyMetersPerSecond,
                target.getX(),
                target.getY(),
                ShooterConstants.COMMANDED_SCORE_IN_HUB_TARGET_ELEVATION_INCHES,
                ShooterConstants.COMMANDED_MAXIMUM_SHOOTING_HEIGHT_INCHES,
                preferredRobotHeading.getRadians(),
                horizontalAim.getMinimumAngle().in(Degrees),
                horizontalAim.getMaximumAngle().in(Degrees),
                idealMovingShotSolution);
        if (!hasIdealSolution) {
            long solverMicros = SlowCallMonitor.nowMicros() - solverStartMicros;
            long totalMicros = SlowCallMonitor.nowMicros() - functionStartMicros;
            if (SlowCallMonitor.isSlow(totalMicros, SLOW_SOLVER_THRESHOLD_MS)
                    || SlowCallMonitor.isSlow(solverMicros, SLOW_SOLVER_THRESHOLD_MS)) {
                SlowCallMonitor.print(
                        "ScoreInHub.updateShooterSolution",
                        totalMicros,
                        String.format(
                                "hasIdealSolution=false distance=%.1f in solveMovingShot=%.3f ms",
                                targetDistanceInches,
                                SlowCallMonitor.toMillis(solverMicros)));
            }
            return false;
        }

        double idealFlywheelCommandIps = idealMovingShotSolution.getFlywheelCommandIps();
        boolean useEmpiricalMovingShotModel = MovingShotMath.shouldUseEmpiricalHubMovingShotModel(
                targetDistanceInches,
                ShooterConstants.COMMANDED_SCORE_IN_HUB_TARGET_ELEVATION_INCHES);
        BallTrajectoryLookup.FixedFlywheelShotStatus fixedFlywheelStatus;
        if (useEmpiricalMovingShotModel) {
            movingShotSolution.copyFrom(idealMovingShotSolution);
            fixedFlywheelStatus = BallTrajectoryLookup.FixedFlywheelShotStatus.VALID;
        } else {
            double predictedFlywheelCommandIps = MovingShotMath.predictFlywheelSpeedIps(
                    shooter.getMainFlywheelSpeedIPS(),
                    idealFlywheelCommandIps);
            fixedFlywheelStatus =
                    BallTrajectoryLookup.solveMovingShotForFlywheelCommand(
                            minimumHoodAngleDegrees,
                            maximumHoodAngleDegrees,
                            ShooterConstants.COMMANDED_MOVING_SHOT_FIXED_FLYWHEEL_HOOD_SEARCH_STEP_DEGREES,
                            true,
                            futureState.xMeters,
                            futureState.yMeters,
                            futureState.headingRadians,
                            futureState.vxMetersPerSecond,
                            futureState.vyMetersPerSecond,
                            target.getX(),
                            target.getY(),
                            ShooterConstants.COMMANDED_SCORE_IN_HUB_TARGET_ELEVATION_INCHES,
                            ShooterConstants.COMMANDED_MAXIMUM_SHOOTING_HEIGHT_INCHES,
                            preferredRobotHeading.getRadians(),
                            horizontalAim.getMinimumAngle().in(Degrees),
                            horizontalAim.getMaximumAngle().in(Degrees),
                            predictedFlywheelCommandIps,
                            movingShotSolution);
        }
        long solverMicros = SlowCallMonitor.nowMicros() - solverStartMicros;
        // Debug dashboard telemetry disabled to reduce NetworkTables traffic.
        // targetDistanceInchesPublisher.set(targetDistanceInches);
        // targetElevationInchesPublisher.set(ShooterConstants.COMMANDED_SCORE_IN_HUB_TARGET_ELEVATION_INCHES);
        // validSolutionPublisher.set(hasSolution);

        if (fixedFlywheelStatus == BallTrajectoryLookup.FixedFlywheelShotStatus.NO_SOLUTION) {
            // hoodAngleDegreesPublisher.set(Double.NaN);
            // shotExitVelocityIpsPublisher.set(0.0);
            // fieldRelativeExitVelocityIpsPublisher.set(0.0);
            // flywheelCommandIpsPublisher.set(0.0);
            // shotAzimuthDegreesPublisher.set(Double.NaN);
            // turretDeltaDegreesPublisher.set(Double.NaN);
            // robotHeadingDegreesPublisher.set(preferredRobotHeading.getDegrees());
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

        double commandedHoodAngleDegrees = switch (fixedFlywheelStatus) {
            case TOO_SLOW -> minimumHoodAngleDegrees;
            case TOO_FAST -> maximumHoodAngleDegrees;
            case VALID -> movingShotSolution.getHoodAngleDegrees();
            case NO_SOLUTION -> Double.NaN;
        };
        double clampedTurretDeltaDegrees = clampTurretDeltaDegrees(
                movingShotSolution.getTurretDeltaDegrees() + azimuthTrimDegreesSupplier.getAsDouble());
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

    private void updateLastDriveDirection(
            double velocityXMetersPerSecond,
            double velocityYMetersPerSecond) {
        Translation2d fieldRelativeVelocity = new Translation2d(
                velocityXMetersPerSecond,
                velocityYMetersPerSecond).rotateBy(drivetrain.getDriverPerspectiveForward());
        lastFieldRelativeDriveDirection = FieldMath.updateLastDriveDirection(
                fieldRelativeVelocity,
                lastFieldRelativeDriveDirection);
    }

    private Rotation2d getPreferredRobotHeading(Rotation2d fallbackHeading) {
        if (lastFieldRelativeDriveDirection.getNorm() > 1e-9) {
            return lastFieldRelativeDriveDirection.getAngle();
        }
        return fallbackHeading;
    }

    private double clampTurretDeltaDegrees(double turretDeltaDegrees) {
        return Math.max(
                horizontalAim.getMinimumAngle().in(Degrees),
                Math.min(horizontalAim.getMaximumAngle().in(Degrees), turretDeltaDegrees));
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

    private void clearShotCommands() {
        shooter.setCoupledIPS(0.0);
        horizontalAim.setAngle(Degrees.of(0.0));
        hasLatchedValidSolution = false;
    }

    private void applyHeldShotCommand(
            SwerveRequest.FieldCentric fieldCentricRequest,
            double velocityXMetersPerSecond,
            double velocityYMetersPerSecond) {
        horizontalAim.setAngle(Degrees.of(lastValidTurretDeltaDegrees));
        shooter.setCoupledIPS(lastValidFlywheelCommandIps);
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
