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

/**
 * Drives while solving a moving shot into the fixed hub target.
 *
 * <p>Frame suffixes used in this command:
 * Fld = absolute WPILib field frame and the default for internal geometry/solver math.
 * Drv = driver-perspective field-centric request frame at the Phoenix request boundary.
 * Rbt = robot/body frame.
 *
 * <p>Unit conventions:
 * Translation2d and Pose2d values use WPILib meters/radians defaults.
 * Primitive suffixes spell units explicitly: Degrees, Radians, Inches, Meters,
 * MetersPerSecond, RadiansPerSecond, Ips, Seconds, Micros.
 */
public class ScoreInHub extends Command {
    private static final double SLOW_EXECUTE_THRESHOLD_MS = 8.0;
    private static final double SLOW_PREDICTION_THRESHOLD_MS = 4.0;
    private static final double SLOW_SOLVER_THRESHOLD_MS = 5.0;
    private static final double SLOW_SET_CONTROL_THRESHOLD_MS = 2.0;
    private static final double TRANSIENT_SOLUTION_HOLD_SECONDS = 0.25;
    private static final double MAX_ACTIVE_TRANSLATIONAL_SPEED_METERS_PER_SECOND = 0.5;
    private static final double RADIAL_SPEED_LIMIT_VIABLE_MARGIN = 0.90;
    private static final double RADIAL_SPEED_LIMIT_SEARCH_TOLERANCE_METERS_PER_SECOND = 0.02;
    private static final int RADIAL_SPEED_LIMIT_SEARCH_MAX_ITERATIONS = 10;

    @FunctionalInterface
    interface HubShotViabilityEvaluator {
        boolean hasViableShot(double robotVxFldMetersPerSecond, double robotVyFldMetersPerSecond);
    }

    static final class TravelVelocityLimitResult {
        private final Translation2d limitedVelocityFldMetersPerSecond;
        private final double updatedMaximumTowardHubTravelSpeedMetersPerSecond;

        TravelVelocityLimitResult(
                Translation2d limitedVelocityFldMetersPerSecond,
                double updatedMaximumTowardHubTravelSpeedMetersPerSecond) {
            this.limitedVelocityFldMetersPerSecond = limitedVelocityFldMetersPerSecond;
            this.updatedMaximumTowardHubTravelSpeedMetersPerSecond =
                    updatedMaximumTowardHubTravelSpeedMetersPerSecond;
        }

        Translation2d getLimitedVelocityFldMetersPerSecond() {
            return limitedVelocityFldMetersPerSecond;
        }

        double getUpdatedMaximumTowardHubTravelSpeedMetersPerSecond() {
            return updatedMaximumTowardHubTravelSpeedMetersPerSecond;
        }
    }

    private final CommandSwerveDrivetrain drivetrain;
    private final PoseEstimatorSubsystem poseEstimator;
    private final SparkAnglePositionSubsystem horizontalAim;
    private final SparkAnglePositionSubsystem verticalAim;
    private final ShooterSubsystem shooter;
    private final Supplier<SwerveRequest> driveRequestSupplier;
    private final SwerveRequest.FieldCentricFacingAngle facingAngleDrive =
            new SwerveRequest.FieldCentricFacingAngle();
    private final PoseEstimatorSubsystem.PredictedFusedState futureStateFldMetersRadians =
            new PoseEstimatorSubsystem.PredictedFusedState();
    private final BallTrajectoryLookup.MovingShotSolution idealMovingShotSolution =
            new BallTrajectoryLookup.MovingShotSolution();
    private final BallTrajectoryLookup.MovingShotSolution movingShotSolution =
            new BallTrajectoryLookup.MovingShotSolution();
    private final BallTrajectoryLookup.MovingShotSolution radialSpeedLimitSearchSolution =
            new BallTrajectoryLookup.MovingShotSolution();
    private Rotation2d lastResolvedRobotHeadingTargetFldRadians = new Rotation2d();
    private double lastResolvedHoodAngleDegrees = 0.0;
    private double lastResolvedTurretDeltaDegrees = 0.0;
    private double lastResolvedFlywheelCommandIps = 0.0;
    private double lastResolvedSolutionTimestampSeconds = Double.NEGATIVE_INFINITY;
    private boolean hasLatchedResolvedSolution = false;
    private double latchedMaximumTowardHubTravelSpeedMetersPerSecond = Double.POSITIVE_INFINITY;
    private double lastCommandedFlywheelSetpointIps = 0.0;

    public ScoreInHub(
            CommandSwerveDrivetrain drivetrain,
            PoseEstimatorSubsystem poseEstimator,
            SparkAnglePositionSubsystem horizontalAim,
            SparkAnglePositionSubsystem verticalAim,
            ShooterSubsystem shooter,
            Supplier<SwerveRequest> driveRequestSupplier) {
        this.drivetrain = Objects.requireNonNull(drivetrain, "drivetrain must not be null");
        this.poseEstimator = Objects.requireNonNull(poseEstimator, "poseEstimator must not be null");
        this.horizontalAim = Objects.requireNonNull(horizontalAim, "horizontalAim must not be null");
        this.verticalAim = Objects.requireNonNull(verticalAim, "verticalAim must not be null");
        this.shooter = Objects.requireNonNull(shooter, "shooter must not be null");
        this.driveRequestSupplier =
                Objects.requireNonNull(driveRequestSupplier, "driveRequestSupplier must not be null");

        facingAngleDrive.withHeadingPID(5.0, 0.0, 0.0);
        facingAngleDrive.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(this.drivetrain, horizontalAim, verticalAim, shooter);
    }

    @Override
    public void initialize() {
        latchedMaximumTowardHubTravelSpeedMetersPerSecond = Double.POSITIVE_INFINITY;
        hasLatchedResolvedSolution = false;
        lastResolvedSolutionTimestampSeconds = Double.NEGATIVE_INFINITY;
        lastCommandedFlywheelSetpointIps = shooter.getMainFlywheelSpeedIPS();
    }

    @Override
    public void execute() {
        long executeStartMicros = SlowCallMonitor.nowMicros();
        long predictionMicros = 0L;
        long solutionMicros = 0L;
        long setControlMicros = 0L;
        Translation2d targetTranslationFldMeters = getTargetTranslationFldMeters();

        SwerveRequest requestedDrive = driveRequestSupplier.get();
        if (!(requestedDrive instanceof SwerveRequest.FieldCentric fieldCentricRequestDrv)) {
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

        Translation2d requestedVelocityDrvMetersPerSecond = new Translation2d(
                fieldCentricRequestDrv.VelocityX,
                fieldCentricRequestDrv.VelocityY);
        Translation2d requestedVelocityFldMetersPerSecond =
                driverVelocityToFieldVelocity(requestedVelocityDrvMetersPerSecond);
        double limitedVelocityXDrvMetersPerSecond;
        double limitedVelocityYDrvMetersPerSecond;

        long predictionStartMicros = SlowCallMonitor.nowMicros();
        if (!poseEstimator.getPredictedFusedState(
                ShooterConstants.COMMANDED_SHOOTER_LOOKAHEAD_SECONDS,
                futureStateFldMetersRadians)) {
            predictionMicros = SlowCallMonitor.nowMicros() - predictionStartMicros;
            Translation2d currentRobotTranslationFldMeters = getCurrentRobotTranslationFldMeters();
            Translation2d fallbackVelocityFldMetersPerSecond = clampTowardHubTravelSpeed(
                    requestedVelocityFldMetersPerSecond,
                    currentRobotTranslationFldMeters,
                    targetTranslationFldMeters,
                    MAX_ACTIVE_TRANSLATIONAL_SPEED_METERS_PER_SECOND);
            Translation2d fallbackVelocityDrvMetersPerSecond =
                    fieldVelocityToDriverVelocity(fallbackVelocityFldMetersPerSecond);
            limitedVelocityXDrvMetersPerSecond = fallbackVelocityDrvMetersPerSecond.getX();
            limitedVelocityYDrvMetersPerSecond = fallbackVelocityDrvMetersPerSecond.getY();
            boolean holdingLastSolution = shouldHoldLastResolvedAim();
            if (holdingLastSolution) {
                applyHeldShotCommand(
                        fieldCentricRequestDrv,
                        limitedVelocityXDrvMetersPerSecond,
                        limitedVelocityYDrvMetersPerSecond);
            } else {
                applyNoSolutionShotCommand(
                        getCurrentRobotTranslationFldMeters(),
                        targetTranslationFldMeters);
                drivetrain.setControl(
                        fieldCentricRequestDrv
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
                                getLatchedResolvedSolutionAgeSeconds(),
                                SlowCallMonitor.toMillis(predictionMicros)));
            }
            return;
        }
        predictionMicros = SlowCallMonitor.nowMicros() - predictionStartMicros;

        Pose2d futureRobotPoseFldMetersRadians = new Pose2d(
                futureStateFldMetersRadians.xMeters,
                futureStateFldMetersRadians.yMeters,
                Rotation2d.fromRadians(futureStateFldMetersRadians.headingRadians));

        Rotation2d preferredRobotHeadingFldRadians = getPreferredRobotHeadingFldRadians(
                futureRobotPoseFldMetersRadians.getTranslation(),
                targetTranslationFldMeters,
                futureRobotPoseFldMetersRadians.getRotation());
        Translation2d limitedVelocityFldMetersPerSecond = limitTowardHubTravelVelocityForViableShot(
                requestedVelocityFldMetersPerSecond,
                futureRobotPoseFldMetersRadians.getTranslation(),
                targetTranslationFldMeters,
                preferredRobotHeadingFldRadians);
        Translation2d limitedVelocityDrvMetersPerSecond =
                fieldVelocityToDriverVelocity(limitedVelocityFldMetersPerSecond);
        limitedVelocityXDrvMetersPerSecond = limitedVelocityDrvMetersPerSecond.getX();
        limitedVelocityYDrvMetersPerSecond = limitedVelocityDrvMetersPerSecond.getY();
        Rotation2d robotHeadingTargetFldRadians = preferredRobotHeadingFldRadians;
        long solutionStartMicros = SlowCallMonitor.nowMicros();
        if (updateShooterSolution(targetTranslationFldMeters, preferredRobotHeadingFldRadians)) {
            robotHeadingTargetFldRadians = Rotation2d.fromDegrees(movingShotSolution.getRobotHeadingDegrees());
        } else if (shouldHoldLastResolvedAim()) {
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
                                getLatchedResolvedSolutionAgeSeconds(),
                                SlowCallMonitor.toMillis(predictionMicros),
                                SlowCallMonitor.toMillis(solutionMicros)));
            }
            return;
        } else {
            applyNoSolutionShotCommand(
                    futureRobotPoseFldMetersRadians.getTranslation(),
                    targetTranslationFldMeters);
        }
        solutionMicros = SlowCallMonitor.nowMicros() - solutionStartMicros;
        Rotation2d robotHeadingTargetDrvRadians = fieldHeadingToDriverHeading(
                robotHeadingTargetFldRadians,
                drivetrain.getDriverPerspectiveForward());

        long setControlStartMicros = SlowCallMonitor.nowMicros();
        drivetrain.setControl(
                facingAngleDrive
                        .withVelocityX(limitedVelocityXDrvMetersPerSecond)
                        .withVelocityY(limitedVelocityYDrvMetersPerSecond)
                        .withTargetDirection(robotHeadingTargetDrvRadians)
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
        latchedMaximumTowardHubTravelSpeedMetersPerSecond = Double.POSITIVE_INFINITY;
    }

    private Translation2d getTargetTranslationFldMeters() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        return FieldMath.getHubTarget(alliance);
    }

    private boolean updateShooterSolution(
            Translation2d targetTranslationFldMeters,
            Rotation2d preferredRobotHeadingFldRadians) {
        long functionStartMicros = SlowCallMonitor.nowMicros();
        double minimumHoodAngleDegrees = verticalAim.getMinimumAngle().in(Degrees);
        double maximumHoodAngleDegrees = verticalAim.getMaximumAngle().in(Degrees);
        double targetDistanceInches = Inches.convertFrom(
                targetTranslationFldMeters.getDistance(
                        new Translation2d(
                                futureStateFldMetersRadians.xMeters,
                                futureStateFldMetersRadians.yMeters)),
                Meters);
        long solverStartMicros = SlowCallMonitor.nowMicros();
        BallTrajectoryLookup.FixedFlywheelShotStatus fixedFlywheelStatus =
                MovingShotMath.solveCommandedMovingShot(
                        verticalAim,
                        ShooterConstants.COMMANDED_MOVING_SHOT_HOOD_SEARCH_STEP_DEGREES,
                        ShooterConstants.COMMANDED_MOVING_SHOT_FIXED_FLYWHEEL_HOOD_SEARCH_STEP_DEGREES,
                        futureStateFldMetersRadians,
                        targetTranslationFldMeters,
                        ShooterConstants.COMMANDED_SCORE_IN_HUB_TARGET_ELEVATION_INCHES,
                        ShooterConstants.COMMANDED_MAXIMUM_SHOOTING_HEIGHT_INCHES,
                        preferredRobotHeadingFldRadians.getRadians(),
                        horizontalAim.getMinimumAngle().in(Degrees),
                        horizontalAim.getMaximumAngle().in(Degrees),
                        shooter.getMainFlywheelSpeedIPS(),
                        lastCommandedFlywheelSetpointIps,
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
        double commandedHoodAngleDegrees = MovingShotMath.getCommandedHoodAngleDegrees(
                fixedFlywheelStatus,
                minimumHoodAngleDegrees,
                maximumHoodAngleDegrees,
                movingShotSolution);
        double clampedTurretDeltaDegrees = MovingShotMath.clampTurretDeltaDegrees(
                movingShotSolution.getTurretDeltaDegrees(),
                horizontalAim.getMinimumAngle().in(Degrees),
                horizontalAim.getMaximumAngle().in(Degrees));
        verticalAim.setAngle(Degrees.of(commandedHoodAngleDegrees));
        horizontalAim.setAngle(Degrees.of(clampedTurretDeltaDegrees));
        shooter.setCoupledIPS(idealFlywheelCommandIps);
        recordPreviousCycleTargets(commandedHoodAngleDegrees, idealFlywheelCommandIps);
        if (hasResolvedAim(fixedFlywheelStatus)) {
            latchResolvedAim(
                    Rotation2d.fromDegrees(movingShotSolution.getRobotHeadingDegrees()),
                    commandedHoodAngleDegrees,
                    clampedTurretDeltaDegrees,
                    idealFlywheelCommandIps);
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

    private Rotation2d getPreferredRobotHeadingFldRadians(
            Translation2d futureRobotTranslationFldMeters,
            Translation2d targetTranslationFldMeters,
            Rotation2d fallbackHeadingFldRadians) {
        return MovingShotMath.getHeadingTowardTarget(
                targetTranslationFldMeters.getX() - futureRobotTranslationFldMeters.getX(),
                targetTranslationFldMeters.getY() - futureRobotTranslationFldMeters.getY(),
                fallbackHeadingFldRadians);
    }

    private boolean shouldHoldLastResolvedAim() {
        return hasLatchedResolvedSolution
                && getLatchedResolvedSolutionAgeSeconds() <= TRANSIENT_SOLUTION_HOLD_SECONDS;
    }

    private double getLatchedResolvedSolutionAgeSeconds() {
        if (!hasLatchedResolvedSolution) {
            return Double.POSITIVE_INFINITY;
        }
        return Timer.getFPGATimestamp() - lastResolvedSolutionTimestampSeconds;
    }

    private void latchResolvedAim(
            Rotation2d robotHeadingTargetFldRadians,
            double hoodAngleDegrees,
            double turretDeltaDegrees,
            double flywheelCommandIps) {
        lastResolvedRobotHeadingTargetFldRadians = robotHeadingTargetFldRadians;
        lastResolvedHoodAngleDegrees = hoodAngleDegrees;
        lastResolvedTurretDeltaDegrees = turretDeltaDegrees;
        lastResolvedFlywheelCommandIps = flywheelCommandIps;
        lastResolvedSolutionTimestampSeconds = Timer.getFPGATimestamp();
        hasLatchedResolvedSolution = true;
    }

    private Translation2d getCurrentRobotTranslationFldMeters() {
        Pose2d currentRobotPoseFldMetersRadians = poseEstimator.getFusedPoseSupplier().get();
        return currentRobotPoseFldMetersRadians != null
                ? currentRobotPoseFldMetersRadians.getTranslation()
                : null;
    }

    private void applyNoSolutionShotCommand(
            Translation2d robotTranslationFldMeters,
            Translation2d targetTranslationFldMeters) {
        double fallbackFlywheelCommandIps = getNoSolutionHubFlywheelCommandIps(
                robotTranslationFldMeters,
                targetTranslationFldMeters);
        shooter.setCoupledIPS(fallbackFlywheelCommandIps);
        recordPreviousCycleTargets(verticalAim.getAngle().in(Degrees), fallbackFlywheelCommandIps);
        horizontalAim.setAngle(Degrees.of(0.0));
        hasLatchedResolvedSolution = false;
    }

    private double getNoSolutionHubFlywheelCommandIps(
            Translation2d robotTranslationFldMeters,
            Translation2d targetTranslationFldMeters) {
        double fallbackDistanceInches = getFallbackHubDistanceInches(
                robotTranslationFldMeters,
                targetTranslationFldMeters);
        double fallbackFlywheelCommandIps =
                ShortRangeHubFlywheelLookup.getFallbackManifoldFlywheelCommandIps(fallbackDistanceInches);
        return Double.isFinite(fallbackFlywheelCommandIps) ? fallbackFlywheelCommandIps : 0.0;
    }

    private double getFallbackHubDistanceInches(
            Translation2d robotTranslationFldMeters,
            Translation2d targetTranslationFldMeters) {
        if (robotTranslationFldMeters != null && targetTranslationFldMeters != null) {
            return Inches.convertFrom(
                    targetTranslationFldMeters.getDistance(robotTranslationFldMeters),
                    Meters);
        }
        return 0.5 * (
                ShooterConstants.DATA_COLLECTION_SHORT_RANGE_MIN_DISTANCE_INCHES
                        + ShooterConstants.DATA_COLLECTION_SHORT_RANGE_EMPIRICAL_MAX_DISTANCE_INCHES);
    }

    private void clearShotCommands() {
        shooter.setCoupledIPS(0.0);
        recordPreviousCycleTargets(verticalAim.getAngle().in(Degrees), 0.0);
        horizontalAim.setAngle(Degrees.of(0.0));
        hasLatchedResolvedSolution = false;
    }

    private void applyHeldShotCommand(
            SwerveRequest.FieldCentric fieldCentricRequestDrv,
            double velocityXDrvMetersPerSecond,
            double velocityYDrvMetersPerSecond) {
        verticalAim.setAngle(Degrees.of(lastResolvedHoodAngleDegrees));
        horizontalAim.setAngle(Degrees.of(lastResolvedTurretDeltaDegrees));
        shooter.setCoupledIPS(lastResolvedFlywheelCommandIps);
        recordPreviousCycleTargets(lastResolvedHoodAngleDegrees, lastResolvedFlywheelCommandIps);
        Rotation2d robotHeadingTargetDrvRadians = fieldHeadingToDriverHeading(
                lastResolvedRobotHeadingTargetFldRadians,
                drivetrain.getDriverPerspectiveForward());
        drivetrain.setControl(
                facingAngleDrive
                        .withVelocityX(velocityXDrvMetersPerSecond)
                        .withVelocityY(velocityYDrvMetersPerSecond)
                        .withTargetDirection(robotHeadingTargetDrvRadians)
                        .withDeadband(fieldCentricRequestDrv.Deadband)
                        .withRotationalDeadband(fieldCentricRequestDrv.RotationalDeadband)
                        .withCenterOfRotation(fieldCentricRequestDrv.CenterOfRotation)
                        .withDriveRequestType(fieldCentricRequestDrv.DriveRequestType)
                        .withSteerRequestType(fieldCentricRequestDrv.SteerRequestType)
                        .withDesaturateWheelSpeeds(fieldCentricRequestDrv.DesaturateWheelSpeeds)
                        .withForwardPerspective(fieldCentricRequestDrv.ForwardPerspective));
    }

    private void recordPreviousCycleTargets(
            double commandedHoodAngleDegrees,
            double commandedFlywheelIps) {
        lastCommandedFlywheelSetpointIps = commandedFlywheelIps;
    }

    private Translation2d limitTowardHubTravelVelocityForViableShot(
            Translation2d requestedVelocityFldMetersPerSecond,
            Translation2d futureRobotTranslationFldMeters,
            Translation2d targetTranslationFldMeters,
            Rotation2d preferredRobotHeadingFldRadians) {
        TravelVelocityLimitResult limitResult = limitTowardHubTravelVelocityForViableShot(
                requestedVelocityFldMetersPerSecond,
                futureRobotTranslationFldMeters,
                targetTranslationFldMeters,
                MAX_ACTIVE_TRANSLATIONAL_SPEED_METERS_PER_SECOND,
                latchedMaximumTowardHubTravelSpeedMetersPerSecond,
                (robotVxFldMetersPerSecond, robotVyFldMetersPerSecond) -> hasViableHubShotForVelocity(
                        robotVxFldMetersPerSecond,
                        robotVyFldMetersPerSecond,
                        targetTranslationFldMeters,
                        preferredRobotHeadingFldRadians));
        latchedMaximumTowardHubTravelSpeedMetersPerSecond =
                limitResult.getUpdatedMaximumTowardHubTravelSpeedMetersPerSecond();
        return limitResult.getLimitedVelocityFldMetersPerSecond();
    }

    static TravelVelocityLimitResult limitTowardHubTravelVelocityForViableShot(
            Translation2d requestedVelocityFldMetersPerSecond,
            Translation2d futureRobotTranslationFldMeters,
            Translation2d targetTranslationFldMeters,
            double minimumAllowedTravelSpeedMetersPerSecond,
            double currentMaximumTowardHubTravelSpeedMetersPerSecond,
            HubShotViabilityEvaluator shotViabilityEvaluator) {
        if (requestedVelocityFldMetersPerSecond == null
                || futureRobotTranslationFldMeters == null
                || targetTranslationFldMeters == null
                || shotViabilityEvaluator == null
                || !Double.isFinite(minimumAllowedTravelSpeedMetersPerSecond)
                || minimumAllowedTravelSpeedMetersPerSecond < 0.0) {
            return new TravelVelocityLimitResult(
                    requestedVelocityFldMetersPerSecond,
                    currentMaximumTowardHubTravelSpeedMetersPerSecond);
        }

        double targetDxFldMeters =
                targetTranslationFldMeters.getX() - futureRobotTranslationFldMeters.getX();
        double targetDyFldMeters =
                targetTranslationFldMeters.getY() - futureRobotTranslationFldMeters.getY();
        double targetDistanceMeters = Math.hypot(targetDxFldMeters, targetDyFldMeters);
        if (!(targetDistanceMeters > 1e-9)) {
            return new TravelVelocityLimitResult(
                    requestedVelocityFldMetersPerSecond,
                    currentMaximumTowardHubTravelSpeedMetersPerSecond);
        }

        double radialUnitXFld = targetDxFldMeters / targetDistanceMeters;
        double radialUnitYFld = targetDyFldMeters / targetDistanceMeters;
        double requestedRadialSpeedMetersPerSecond =
                requestedVelocityFldMetersPerSecond.getX() * radialUnitXFld
                        + requestedVelocityFldMetersPerSecond.getY() * radialUnitYFld;
        if (!Double.isFinite(requestedRadialSpeedMetersPerSecond)) {
            return new TravelVelocityLimitResult(
                    requestedVelocityFldMetersPerSecond,
                    currentMaximumTowardHubTravelSpeedMetersPerSecond);
        }
        if (requestedRadialSpeedMetersPerSecond < -1e-9) {
            return new TravelVelocityLimitResult(
                    requestedVelocityFldMetersPerSecond,
                    currentMaximumTowardHubTravelSpeedMetersPerSecond);
        }

        double requestedTravelSpeedMetersPerSecond = requestedVelocityFldMetersPerSecond.getNorm();
        if (!(requestedTravelSpeedMetersPerSecond > 1e-9)) {
            return new TravelVelocityLimitResult(
                    requestedVelocityFldMetersPerSecond,
                    currentMaximumTowardHubTravelSpeedMetersPerSecond);
        }
        if (requestedRadialSpeedMetersPerSecond <= 1e-9
                || requestedTravelSpeedMetersPerSecond <= minimumAllowedTravelSpeedMetersPerSecond + 1e-9) {
            return new TravelVelocityLimitResult(
                    requestedVelocityFldMetersPerSecond,
                    currentMaximumTowardHubTravelSpeedMetersPerSecond);
        }

        Translation2d travelUnitVectorFld = requestedVelocityFldMetersPerSecond.div(requestedTravelSpeedMetersPerSecond);
        double currentMaximumAllowedTravelSpeedMetersPerSecond = Math.max(
                minimumAllowedTravelSpeedMetersPerSecond,
                currentMaximumTowardHubTravelSpeedMetersPerSecond);
        double candidateTravelSpeedMetersPerSecond = Math.min(
                requestedTravelSpeedMetersPerSecond,
                currentMaximumAllowedTravelSpeedMetersPerSecond);
        if (shotViabilityEvaluator.hasViableShot(
                travelUnitVectorFld.getX() * candidateTravelSpeedMetersPerSecond,
                travelUnitVectorFld.getY() * candidateTravelSpeedMetersPerSecond)) {
            if (requestedTravelSpeedMetersPerSecond <= currentMaximumAllowedTravelSpeedMetersPerSecond + 1e-9) {
                return new TravelVelocityLimitResult(
                        requestedVelocityFldMetersPerSecond,
                        currentMaximumTowardHubTravelSpeedMetersPerSecond);
            }
            return new TravelVelocityLimitResult(
                    travelUnitVectorFld.times(candidateTravelSpeedMetersPerSecond),
                    currentMaximumTowardHubTravelSpeedMetersPerSecond);
        }

        if (!shotViabilityEvaluator.hasViableShot(
                travelUnitVectorFld.getX() * minimumAllowedTravelSpeedMetersPerSecond,
                travelUnitVectorFld.getY() * minimumAllowedTravelSpeedMetersPerSecond)) {
            return new TravelVelocityLimitResult(
                    travelUnitVectorFld.times(minimumAllowedTravelSpeedMetersPerSecond),
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
                    travelUnitVectorFld.getX() * midTravelSpeedMetersPerSecond,
                    travelUnitVectorFld.getY() * midTravelSpeedMetersPerSecond);
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
                travelUnitVectorFld.times(limitedTravelSpeedMetersPerSecond),
                updatedMaximumTowardHubTravelSpeedMetersPerSecond);
    }

    private boolean hasViableHubShotForVelocity(
            double robotVxFldMetersPerSecond,
            double robotVyFldMetersPerSecond,
            Translation2d targetTranslationFldMeters,
            Rotation2d preferredRobotHeadingFldRadians) {
        double targetDistanceInches = Inches.convertFrom(
                targetTranslationFldMeters.getDistance(new Translation2d(
                        futureStateFldMetersRadians.xMeters,
                        futureStateFldMetersRadians.yMeters)),
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
                futureStateFldMetersRadians.xMeters,
                futureStateFldMetersRadians.yMeters,
                futureStateFldMetersRadians.headingRadians,
                robotVxFldMetersPerSecond,
                robotVyFldMetersPerSecond,
                targetTranslationFldMeters.getX(),
                targetTranslationFldMeters.getY(),
                ShooterConstants.COMMANDED_SCORE_IN_HUB_TARGET_ELEVATION_INCHES,
                ShooterConstants.COMMANDED_MAXIMUM_SHOOTING_HEIGHT_INCHES,
                preferredRobotHeadingFldRadians.getRadians(),
                horizontalAim.getMinimumAngle().in(Degrees),
                horizontalAim.getMaximumAngle().in(Degrees),
                shooter.getMainFlywheelSpeedIPS(),
                lastCommandedFlywheelSetpointIps,
                true,
                radialSpeedLimitSearchSolution,
                null);
    }

    private Translation2d driverVelocityToFieldVelocity(Translation2d velocityDrvMetersPerSecond) {
        return driverVelocityToFieldVelocity(
                velocityDrvMetersPerSecond,
                drivetrain.getDriverPerspectiveForward());
    }

    private Translation2d fieldVelocityToDriverVelocity(Translation2d velocityFldMetersPerSecond) {
        return fieldVelocityToDriverVelocity(
                velocityFldMetersPerSecond,
                drivetrain.getDriverPerspectiveForward());
    }

    static Translation2d driverVelocityToFieldVelocity(
            Translation2d velocityDrvMetersPerSecond,
            Rotation2d driverPerspectiveForward) {
        if (velocityDrvMetersPerSecond == null || driverPerspectiveForward == null) {
            return null;
        }
        return velocityDrvMetersPerSecond.rotateBy(driverPerspectiveForward);
    }

    static Translation2d fieldVelocityToDriverVelocity(
            Translation2d velocityFldMetersPerSecond,
            Rotation2d driverPerspectiveForward) {
        if (velocityFldMetersPerSecond == null || driverPerspectiveForward == null) {
            return null;
        }
        return velocityFldMetersPerSecond.rotateBy(driverPerspectiveForward.unaryMinus());
    }

    static Rotation2d fieldHeadingToDriverHeading(
            Rotation2d headingFldRadians,
            Rotation2d driverPerspectiveForward) {
        if (headingFldRadians == null || driverPerspectiveForward == null) {
            return null;
        }
        return headingFldRadians.minus(driverPerspectiveForward);
    }

    static boolean hasResolvedAim(BallTrajectoryLookup.FixedFlywheelShotStatus fixedFlywheelStatus) {
        return fixedFlywheelStatus != BallTrajectoryLookup.FixedFlywheelShotStatus.NO_SOLUTION;
    }

    private static Translation2d clampTowardHubTravelSpeed(
            Translation2d requestedVelocityFldMetersPerSecond,
            Translation2d robotTranslationFldMeters,
            Translation2d targetTranslationFldMeters,
            double maximumTowardHubTravelSpeedMetersPerSecond) {
        if (requestedVelocityFldMetersPerSecond == null
                || robotTranslationFldMeters == null
                || targetTranslationFldMeters == null
                || !Double.isFinite(maximumTowardHubTravelSpeedMetersPerSecond)
                || maximumTowardHubTravelSpeedMetersPerSecond < 0.0) {
            return requestedVelocityFldMetersPerSecond;
        }

        double targetDxFldMeters =
                targetTranslationFldMeters.getX() - robotTranslationFldMeters.getX();
        double targetDyFldMeters =
                targetTranslationFldMeters.getY() - robotTranslationFldMeters.getY();
        double targetDistanceMeters = Math.hypot(targetDxFldMeters, targetDyFldMeters);
        if (!(targetDistanceMeters > 1e-9)) {
            return requestedVelocityFldMetersPerSecond;
        }

        double radialUnitXFld = targetDxFldMeters / targetDistanceMeters;
        double radialUnitYFld = targetDyFldMeters / targetDistanceMeters;
        double requestedRadialSpeedMetersPerSecond =
                requestedVelocityFldMetersPerSecond.getX() * radialUnitXFld
                        + requestedVelocityFldMetersPerSecond.getY() * radialUnitYFld;
        if (!(requestedRadialSpeedMetersPerSecond > 1e-9)) {
            return requestedVelocityFldMetersPerSecond;
        }
        double requestedTravelSpeedMetersPerSecond = requestedVelocityFldMetersPerSecond.getNorm();
        if (!(requestedTravelSpeedMetersPerSecond > maximumTowardHubTravelSpeedMetersPerSecond)) {
            return requestedVelocityFldMetersPerSecond;
        }
        return requestedVelocityFldMetersPerSecond.times(
                maximumTowardHubTravelSpeedMetersPerSecond / requestedTravelSpeedMetersPerSecond);
    }

}
