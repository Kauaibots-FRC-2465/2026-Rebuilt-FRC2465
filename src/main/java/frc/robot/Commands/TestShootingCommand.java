package frc.robot.Commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.Objects;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.fieldmath.FieldMath;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakePositionSubsystem;
import frc.robot.subsystems.KrakenFlywheelSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SparkAnglePositionSubsystem;
import frc.robot.utility.TuningDashboard;

/**
 * Toggleable hub-aiming test mode that follows the same solve path as
 * ScoreInHub for aiming, while allowing manual hood selection.
 *
 * Frame suffixes used in this command:
 * Fld = field frame and the default for internal geometry/solver math.
 * Drv = driver-perspective field-centric request frame at the Phoenix request boundary.
 * Rbt = robot/body frame.
 */
public class TestShootingCommand extends Command {
    private static final double HOOD_TRIM_STEP_DEGREES = 1.0;
    private static final double INTAKE_HOLD_ANGLE_DEGREES = 90.0;

    private final CommandSwerveDrivetrain drivetrain;
    private final PoseEstimatorSubsystem poseEstimator;
    private final SparkAnglePositionSubsystem horizontalAim;
    private final SparkAnglePositionSubsystem verticalAim;
    private final IntakePositionSubsystem intakePosition;
    private final ShooterSubsystem shooter;
    private final KrakenFlywheelSubsystem intakeDrive;
    private final Supplier<SwerveRequest> driveRequestSupplier;
    private final SwerveRequest.FieldCentricFacingAngle facingAngleDrive =
            new SwerveRequest.FieldCentricFacingAngle();
    private final PoseEstimatorSubsystem.PredictedFusedState predictedStateFld =
            new PoseEstimatorSubsystem.PredictedFusedState();
    private final BallTrajectoryLookup.MovingShotSolution fixedAngleSolution =
            new BallTrajectoryLookup.MovingShotSolution();
    private final DoublePublisher targetDistanceInchesPublisher;
    private final DoublePublisher hoodAngleDegreesPublisher;
    private final DoublePublisher flywheelCommandIpsPublisher;
    private final DoublePublisher launcherRelativeExitVelocityIpsPublisher;

    private double requestedHoodAngleDegrees;
    private Translation2d lastDriveDirectionFld = new Translation2d();
    private boolean hasLatchedState;
    private double latchedRobotXFldMeters;
    private double latchedRobotYFldMeters;
    private double latchedRobotHeadingFldRadians;
    private double latchedRobotVxFldMetersPerSecond;
    private double latchedRobotVyFldMetersPerSecond;

    public TestShootingCommand(
            CommandSwerveDrivetrain drivetrain,
            PoseEstimatorSubsystem poseEstimator,
            SparkAnglePositionSubsystem horizontalAim,
            SparkAnglePositionSubsystem verticalAim,
            IntakePositionSubsystem intakePosition,
            ShooterSubsystem shooter,
            KrakenFlywheelSubsystem intakeDrive,
            Supplier<SwerveRequest> driveRequestSupplier) {
        this.drivetrain = Objects.requireNonNull(drivetrain, "drivetrain must not be null");
        this.poseEstimator = Objects.requireNonNull(poseEstimator, "poseEstimator must not be null");
        this.horizontalAim = Objects.requireNonNull(horizontalAim, "horizontalAim must not be null");
        this.verticalAim = Objects.requireNonNull(verticalAim, "verticalAim must not be null");
        this.intakePosition = Objects.requireNonNull(intakePosition, "intakePosition must not be null");
        this.shooter = Objects.requireNonNull(shooter, "shooter must not be null");
        this.intakeDrive = Objects.requireNonNull(intakeDrive, "intakeDrive must not be null");
        this.driveRequestSupplier = Objects.requireNonNull(driveRequestSupplier, "driveRequestSupplier must not be null");

        facingAngleDrive.withHeadingPID(5.0, 0.0, 0.0);
        facingAngleDrive.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        NetworkTable testShootingTable = NetworkTableInstance.getDefault().getTable("testShootingCommand");
        targetDistanceInchesPublisher = testShootingTable.getDoubleTopic("targetDistanceInches").publish();
        hoodAngleDegreesPublisher = testShootingTable.getDoubleTopic("hoodAngleDegrees").publish();
        flywheelCommandIpsPublisher = testShootingTable.getDoubleTopic("flywheelCommandIps").publish();
        launcherRelativeExitVelocityIpsPublisher =
                testShootingTable.getDoubleTopic("launcherRelativeExitVelocityIps").publish();

        addRequirements(
                this.drivetrain,
                this.horizontalAim,
                this.verticalAim,
                this.intakePosition,
                this.shooter,
                this.intakeDrive);
    }

    @Override
    public void initialize() {
        requestedHoodAngleDegrees = Double.NaN;
        lastDriveDirectionFld = new Translation2d();
        hasLatchedState = false;
        targetDistanceInchesPublisher.set(Double.NaN);
        hoodAngleDegreesPublisher.set(verticalAim.getAngle().in(Degrees));
        flywheelCommandIpsPublisher.set(0.0);
        launcherRelativeExitVelocityIpsPublisher.set(0.0);
    }

    @Override
    public void execute() {
        SwerveRequest requestedDrive = driveRequestSupplier.get();
        intakePosition.setAngle(Degrees.of(INTAKE_HOLD_ANGLE_DEGREES));
        intakeDrive.setIPS(0.0);

        if (!refreshLatchedState()) {
            clearShotOutputs();
            publishCurrentHoodAngle(verticalAim.getAngle().in(Degrees));
            drivetrain.setControl(requestedDrive);
            return;
        }

        Translation2d hubTargetFld = getHubTargetFld();
        TuningDashboard.publishShootingTarget(hubTargetFld);
        double targetDistanceInches = Inches.convertFrom(
                hubTargetFld.getDistance(new Translation2d(latchedRobotXFldMeters, latchedRobotYFldMeters)),
                Meters);
        targetDistanceInchesPublisher.set(targetDistanceInches);
        updateLastDriveDirectionFld(requestedDrive);
        Rotation2d preferredRobotHeadingFld =
                getPreferredRobotHeadingFld(Rotation2d.fromRadians(latchedRobotHeadingFldRadians));

        boolean useEmpiricalMovingShotModel = MovingShotMath.shouldUseEmpiricalHubMovingShotModel(
                targetDistanceInches,
                ShooterConstants.COMMANDED_SCORE_IN_HUB_TARGET_ELEVATION_INCHES);
        double commandedHoodAngleDegrees = resolveCommandedHoodAngleDegrees(
                targetDistanceInches,
                useEmpiricalMovingShotModel);
        if (!Double.isFinite(commandedHoodAngleDegrees)) {
            clearShotOutputs();
            publishCurrentHoodAngle(verticalAim.getAngle().in(Degrees));
            drivetrain.setControl(requestedDrive);
            return;
        }

        boolean hasShotSolution = MovingShotMath.solveMovingShotAtClosestHoodAngle(
                commandedHoodAngleDegrees,
                verticalAim.getMinimumAngle().in(Degrees),
                verticalAim.getMaximumAngle().in(Degrees),
                ShooterConstants.COMMANDED_MOVING_SHOT_FIXED_FLYWHEEL_HOOD_SEARCH_STEP_DEGREES,
                latchedRobotXFldMeters,
                latchedRobotYFldMeters,
                latchedRobotHeadingFldRadians,
                latchedRobotVxFldMetersPerSecond,
                latchedRobotVyFldMetersPerSecond,
                hubTargetFld.getX(),
                hubTargetFld.getY(),
                ShooterConstants.COMMANDED_SCORE_IN_HUB_TARGET_ELEVATION_INCHES,
                ShooterConstants.COMMANDED_TEST_SHOOTING_MAXIMUM_HEIGHT_INCHES,
                preferredRobotHeadingFld.getRadians(),
                horizontalAim.getMinimumAngle().in(Degrees),
                horizontalAim.getMaximumAngle().in(Degrees),
                fixedAngleSolution);
        if (!hasShotSolution) {
            clearShotOutputs();
            drivetrain.setControl(requestedDrive);
            return;
        }

        commandedHoodAngleDegrees = fixedAngleSolution.getHoodAngleDegrees();
        requestedHoodAngleDegrees = commandedHoodAngleDegrees;
        verticalAim.setAngle(Degrees.of(commandedHoodAngleDegrees));
        publishCurrentHoodAngle(commandedHoodAngleDegrees);

        double commandedFlywheelIps = fixedAngleSolution.getFlywheelCommandIps();
        if (!Double.isFinite(commandedFlywheelIps)) {
            clearShotOutputs();
            drivetrain.setControl(requestedDrive);
            return;
        }

        double turretDeltaDegrees = fixedAngleSolution.getTurretDeltaDegrees();
        double launcherRelativeExitVelocityIps = fixedAngleSolution.getLauncherRelativeExitVelocityIps();
        double robotHeadingFldDegrees = fixedAngleSolution.getRobotHeadingDegrees();
        horizontalAim.setAngle(Degrees.of(clampTurretDeltaDegrees(turretDeltaDegrees)));
        shooter.setCoupledIPS(commandedFlywheelIps);
        flywheelCommandIpsPublisher.set(commandedFlywheelIps);
        launcherRelativeExitVelocityIpsPublisher.set(launcherRelativeExitVelocityIps);
        applyDriveHeadingTarget(requestedDrive, robotHeadingFldDegrees);
    }

    @Override
    public void end(boolean interrupted) {
        intakeDrive.setIPS(0.0);
        clearOutputs();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public void trimHoodUp() {
        if (!Double.isFinite(requestedHoodAngleDegrees)) {
            requestedHoodAngleDegrees = verticalAim.getAngle().in(Degrees);
        }
        requestedHoodAngleDegrees += HOOD_TRIM_STEP_DEGREES;
    }

    public void trimHoodDown() {
        if (!Double.isFinite(requestedHoodAngleDegrees)) {
            requestedHoodAngleDegrees = verticalAim.getAngle().in(Degrees);
        }
        requestedHoodAngleDegrees -= HOOD_TRIM_STEP_DEGREES;
    }

    private Translation2d getHubTargetFld() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        return FieldMath.getHubTarget(alliance);
    }

    private double clampMechanismHoodAngle(double hoodAngleDegrees) {
        return Math.max(
                verticalAim.getMinimumAngle().in(Degrees),
                Math.min(verticalAim.getMaximumAngle().in(Degrees), hoodAngleDegrees));
    }

    private double resolveCommandedHoodAngleDegrees(double targetDistanceInches, boolean useEmpiricalMovingShotModel) {
        if (!Double.isFinite(requestedHoodAngleDegrees)) {
            requestedHoodAngleDegrees = verticalAim.getAngle().in(Degrees);
        }

        if (useEmpiricalMovingShotModel) {
            requestedHoodAngleDegrees = ShortRangeHubFlywheelLookup.clampHoodAngle(
                    targetDistanceInches,
                    requestedHoodAngleDegrees);
        } else {
            requestedHoodAngleDegrees = clampMechanismHoodAngle(requestedHoodAngleDegrees);
        }
        return requestedHoodAngleDegrees;
    }

    private double clampTurretDeltaDegrees(double turretDeltaDegrees) {
        return Math.max(
                horizontalAim.getMinimumAngle().in(Degrees),
                Math.min(horizontalAim.getMaximumAngle().in(Degrees), turretDeltaDegrees));
    }

    private void updateLastDriveDirectionFld(SwerveRequest requestedDrive) {
        if (!(requestedDrive instanceof SwerveRequest.FieldCentric fieldCentricRequestDrv)) {
            return;
        }

        Translation2d requestedVelocityFldMps = new Translation2d(
                fieldCentricRequestDrv.VelocityX,
                fieldCentricRequestDrv.VelocityY).rotateBy(drivetrain.getDriverPerspectiveForward());
        lastDriveDirectionFld = FieldMath.updateLastDriveDirection(
                requestedVelocityFldMps,
                lastDriveDirectionFld);
    }

    private Rotation2d getPreferredRobotHeadingFld(Rotation2d fallbackHeadingFld) {
        if (lastDriveDirectionFld.getNorm() > 1e-9) {
            return lastDriveDirectionFld.getAngle();
        }
        return fallbackHeadingFld;
    }

    private boolean refreshLatchedState() {
        if (poseEstimator.getPredictedFusedState(0.0, predictedStateFld)) {
            latchPredictedStateFld();
            return true;
        }

        if (hasLatchedState) {
            return true;
        }

        Pose2d fusedPoseFld = poseEstimator.getFusedPoseSupplier().get();
        if (fusedPoseFld == null) {
            return false;
        }

        latchedRobotXFldMeters = fusedPoseFld.getX();
        latchedRobotYFldMeters = fusedPoseFld.getY();
        latchedRobotHeadingFldRadians = fusedPoseFld.getRotation().getRadians();
        latchedRobotVxFldMetersPerSecond = 0.0;
        latchedRobotVyFldMetersPerSecond = 0.0;
        hasLatchedState = true;
        return true;
    }

    private void latchPredictedStateFld() {
        latchedRobotXFldMeters = predictedStateFld.xMeters;
        latchedRobotYFldMeters = predictedStateFld.yMeters;
        latchedRobotHeadingFldRadians = predictedStateFld.headingRadians;
        latchedRobotVxFldMetersPerSecond = predictedStateFld.vxMetersPerSecond;
        latchedRobotVyFldMetersPerSecond = predictedStateFld.vyMetersPerSecond;
        hasLatchedState = true;
    }

    private void publishCurrentHoodAngle(double hoodAngleDegrees) {
        hoodAngleDegreesPublisher.set(hoodAngleDegrees);
    }

    private void clearShotOutputs() {
        shooter.setCoupledIPS(0.0);
        horizontalAim.setAngle(Degrees.of(0.0));
        flywheelCommandIpsPublisher.set(0.0);
        launcherRelativeExitVelocityIpsPublisher.set(0.0);
    }

    private void clearOutputs() {
        shooter.setCoupledIPS(0.0);
        horizontalAim.setAngle(Degrees.of(0.0));
        targetDistanceInchesPublisher.set(Double.NaN);
        hoodAngleDegreesPublisher.set(Double.NaN);
        flywheelCommandIpsPublisher.set(0.0);
        launcherRelativeExitVelocityIpsPublisher.set(0.0);
    }

    private void applyDriveHeadingTarget(SwerveRequest requestedDrive, double robotHeadingFldDegrees) {
        if (!(requestedDrive instanceof SwerveRequest.FieldCentric fieldCentricRequestDrv)) {
            drivetrain.setControl(requestedDrive);
            return;
        }

        Rotation2d robotHeadingTargetDrv =
                Rotation2d.fromDegrees(robotHeadingFldDegrees).minus(drivetrain.getDriverPerspectiveForward());
        drivetrain.setControl(
                facingAngleDrive
                        .withVelocityX(fieldCentricRequestDrv.VelocityX)
                        .withVelocityY(fieldCentricRequestDrv.VelocityY)
                        .withTargetDirection(robotHeadingTargetDrv)
                        .withDeadband(fieldCentricRequestDrv.Deadband)
                        .withRotationalDeadband(fieldCentricRequestDrv.RotationalDeadband)
                        .withCenterOfRotation(fieldCentricRequestDrv.CenterOfRotation)
                        .withDriveRequestType(fieldCentricRequestDrv.DriveRequestType)
                        .withSteerRequestType(fieldCentricRequestDrv.SteerRequestType)
                        .withDesaturateWheelSpeeds(fieldCentricRequestDrv.DesaturateWheelSpeeds)
                        .withForwardPerspective(fieldCentricRequestDrv.ForwardPerspective));
    }
}
