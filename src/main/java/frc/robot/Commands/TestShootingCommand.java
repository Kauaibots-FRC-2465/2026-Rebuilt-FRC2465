package frc.robot.Commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.Objects;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.fieldmath.FieldMath;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SparkAnglePositionSubsystem;
import frc.robot.utility.TuningDashboard;

/**
 * Aims at the hub using the live pose estimate, then allows the operator to
 * trim the hood target in actual-angle degrees while recomputing the required
 * flywheel speed for that fixed hood angle.
 */
public class TestShootingCommand extends Command {
    private static final double HOOD_TRIM_STEP_DEGREES = 1.0;

    private final PoseEstimatorSubsystem poseEstimator;
    private final SparkAnglePositionSubsystem horizontalAim;
    private final SparkAnglePositionSubsystem verticalAim;
    private final ShooterSubsystem shooter;
    private final DoubleSupplier azimuthTrimDegreesSupplier;
    private final PoseEstimatorSubsystem.PredictedFusedState predictedState =
            new PoseEstimatorSubsystem.PredictedFusedState();
    private final BallTrajectoryLookup.MovingShotSolution baseSolution =
            new BallTrajectoryLookup.MovingShotSolution();
    private final BallTrajectoryLookup.MovingShotSolution fixedAngleSolution =
            new BallTrajectoryLookup.MovingShotSolution();
    private final DoublePublisher targetDistanceInchesPublisher;
    private final DoublePublisher hoodAngleDegreesPublisher;
    private final DoublePublisher flywheelCommandIpsPublisher;
    private final DoublePublisher launcherRelativeExitVelocityIpsPublisher;

    private double desiredHoodAngleDegrees;
    private boolean hasLatchedState;
    private double latchedXMeters;
    private double latchedYMeters;
    private double latchedHeadingRadians;
    private double latchedVxMetersPerSecond;
    private double latchedVyMetersPerSecond;

    public TestShootingCommand(
            PoseEstimatorSubsystem poseEstimator,
            SparkAnglePositionSubsystem horizontalAim,
            SparkAnglePositionSubsystem verticalAim,
            ShooterSubsystem shooter,
            DoubleSupplier azimuthTrimDegreesSupplier) {
        this.poseEstimator = Objects.requireNonNull(poseEstimator, "poseEstimator must not be null");
        this.horizontalAim = Objects.requireNonNull(horizontalAim, "horizontalAim must not be null");
        this.verticalAim = Objects.requireNonNull(verticalAim, "verticalAim must not be null");
        this.shooter = Objects.requireNonNull(shooter, "shooter must not be null");
        this.azimuthTrimDegreesSupplier =
                Objects.requireNonNull(azimuthTrimDegreesSupplier, "azimuthTrimDegreesSupplier must not be null");

        NetworkTable testShootingTable = NetworkTableInstance.getDefault().getTable("testShootingCommand");
        targetDistanceInchesPublisher = testShootingTable.getDoubleTopic("targetDistanceInches").publish();
        hoodAngleDegreesPublisher = testShootingTable.getDoubleTopic("hoodAngleDegrees").publish();
        flywheelCommandIpsPublisher = testShootingTable.getDoubleTopic("flywheelCommandIps").publish();
        launcherRelativeExitVelocityIpsPublisher =
                testShootingTable.getDoubleTopic("launcherRelativeExitVelocityIps").publish();

        addRequirements(this.horizontalAim, this.verticalAim, this.shooter);
    }

    @Override
    public void initialize() {
        desiredHoodAngleDegrees = Double.NaN;
        hasLatchedState = false;
        publishCurrentHoodAngle();
        flywheelCommandIpsPublisher.set(0.0);
        launcherRelativeExitVelocityIpsPublisher.set(0.0);
    }

    @Override
    public void execute() {
        if (!refreshLatchedState()) {
            publishCurrentHoodAngle();
            shooter.setCoupledIPS(0.0);
            flywheelCommandIpsPublisher.set(0.0);
            launcherRelativeExitVelocityIpsPublisher.set(0.0);
            return;
        }

        Translation2d target = getTarget();
        TuningDashboard.publishShootingTarget(target);
        double targetDistanceInches = Inches.convertFrom(
                target.getDistance(new Translation2d(latchedXMeters, latchedYMeters)),
                Meters);
        targetDistanceInchesPublisher.set(targetDistanceInches);

        seedDesiredHoodAngleIfNeeded(target);
        desiredHoodAngleDegrees = clampHoodAngle(desiredHoodAngleDegrees);
        verticalAim.setAngle(Degrees.of(desiredHoodAngleDegrees));
        hoodAngleDegreesPublisher.set(desiredHoodAngleDegrees);

        boolean hasFixedAngleSolution = BallTrajectoryLookup.solveMovingShot(
                desiredHoodAngleDegrees,
                desiredHoodAngleDegrees,
                HOOD_TRIM_STEP_DEGREES,
                true,
                latchedXMeters,
                latchedYMeters,
                latchedHeadingRadians,
                latchedVxMetersPerSecond,
                latchedVyMetersPerSecond,
                target.getX(),
                target.getY(),
                ShooterConstants.COMMANDED_SCORE_IN_HUB_TARGET_ELEVATION_INCHES,
                ShooterConstants.COMMANDED_TEST_SHOOTING_MAXIMUM_HEIGHT_INCHES,
                latchedHeadingRadians,
                horizontalAim.getMinimumAngle().in(Degrees),
                horizontalAim.getMaximumAngle().in(Degrees),
                fixedAngleSolution);

        if (hasFixedAngleSolution) {
            horizontalAim.setAngle(Degrees.of(clampTurretDeltaDegrees(
                    fixedAngleSolution.getTurretDeltaDegrees() + azimuthTrimDegreesSupplier.getAsDouble())));
            shooter.setCoupledIPS(fixedAngleSolution.getFlywheelCommandIps());
            flywheelCommandIpsPublisher.set(fixedAngleSolution.getFlywheelCommandIps());
            launcherRelativeExitVelocityIpsPublisher.set(fixedAngleSolution.getLauncherRelativeExitVelocityIps());
            return;
        }

        shooter.setCoupledIPS(0.0);
        flywheelCommandIpsPublisher.set(0.0);
        launcherRelativeExitVelocityIpsPublisher.set(0.0);
    }

    @Override
    public void end(boolean interrupted) {
        clearOutputs();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public void trimHoodUp() {
        if (!Double.isFinite(desiredHoodAngleDegrees)) {
            desiredHoodAngleDegrees = verticalAim.getAngle().in(Degrees);
        }
        desiredHoodAngleDegrees = clampHoodAngle(desiredHoodAngleDegrees - HOOD_TRIM_STEP_DEGREES);
    }

    public void trimHoodDown() {
        if (!Double.isFinite(desiredHoodAngleDegrees)) {
            desiredHoodAngleDegrees = verticalAim.getAngle().in(Degrees);
        }
        desiredHoodAngleDegrees = clampHoodAngle(desiredHoodAngleDegrees + HOOD_TRIM_STEP_DEGREES);
    }

    private Translation2d getTarget() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        return FieldMath.getHubTarget(alliance);
    }

    private double clampHoodAngle(double hoodAngleDegrees) {
        return Math.max(
                verticalAim.getMinimumAngle().in(Degrees),
                Math.min(verticalAim.getMaximumAngle().in(Degrees), hoodAngleDegrees));
    }

    private double clampTurretDeltaDegrees(double turretDeltaDegrees) {
        return Math.max(
                horizontalAim.getMinimumAngle().in(Degrees),
                Math.min(horizontalAim.getMaximumAngle().in(Degrees), turretDeltaDegrees));
    }

    private boolean refreshLatchedState() {
        if (poseEstimator.getPredictedFusedState(
                ShooterConstants.COMMANDED_SHOOTER_LOOKAHEAD_SECONDS,
                predictedState)) {
            latchedXMeters = predictedState.xMeters;
            latchedYMeters = predictedState.yMeters;
            latchedHeadingRadians = predictedState.headingRadians;
            latchedVxMetersPerSecond = predictedState.vxMetersPerSecond;
            latchedVyMetersPerSecond = predictedState.vyMetersPerSecond;
            hasLatchedState = true;
            return true;
        }

        if (hasLatchedState) {
            return true;
        }

        Pose2d fusedPose = poseEstimator.getFusedPoseSupplier().get();
        if (fusedPose == null) {
            return false;
        }

        latchedXMeters = fusedPose.getX();
        latchedYMeters = fusedPose.getY();
        latchedHeadingRadians = fusedPose.getRotation().getRadians();
        latchedVxMetersPerSecond = 0.0;
        latchedVyMetersPerSecond = 0.0;
        hasLatchedState = true;
        return true;
    }

    private void seedDesiredHoodAngleIfNeeded(Translation2d target) {
        if (Double.isFinite(desiredHoodAngleDegrees)) {
            return;
        }

        boolean hasBaseSolution = BallTrajectoryLookup.solveMovingShot(
                verticalAim.getMinimumAngle().in(Degrees),
                verticalAim.getMaximumAngle().in(Degrees),
                ShooterConstants.COMMANDED_MOVING_SHOT_HOOD_SEARCH_STEP_DEGREES,
                true,
                latchedXMeters,
                latchedYMeters,
                latchedHeadingRadians,
                latchedVxMetersPerSecond,
                latchedVyMetersPerSecond,
                target.getX(),
                target.getY(),
                ShooterConstants.COMMANDED_SCORE_IN_HUB_TARGET_ELEVATION_INCHES,
                ShooterConstants.COMMANDED_TEST_SHOOTING_MAXIMUM_HEIGHT_INCHES,
                latchedHeadingRadians,
                horizontalAim.getMinimumAngle().in(Degrees),
                horizontalAim.getMaximumAngle().in(Degrees),
                baseSolution);
        desiredHoodAngleDegrees = hasBaseSolution
                ? baseSolution.getHoodAngleDegrees()
                : verticalAim.getAngle().in(Degrees);
    }

    private void publishCurrentHoodAngle() {
        double hoodAngleDegrees = Double.isFinite(desiredHoodAngleDegrees)
                ? desiredHoodAngleDegrees
                : verticalAim.getAngle().in(Degrees);
        hoodAngleDegreesPublisher.set(hoodAngleDegrees);
    }

    private void clearOutputs() {
        shooter.setCoupledIPS(0.0);
        horizontalAim.setAngle(Degrees.of(0.0));
        targetDistanceInchesPublisher.set(Double.NaN);
        hoodAngleDegreesPublisher.set(Double.NaN);
        flywheelCommandIpsPublisher.set(0.0);
        launcherRelativeExitVelocityIpsPublisher.set(0.0);
    }
}
