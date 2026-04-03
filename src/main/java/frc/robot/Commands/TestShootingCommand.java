package frc.robot.Commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.Objects;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.fieldmath.FieldMath;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SparkAnglePositionSubsystem;

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
    private final PoseEstimatorSubsystem.PredictedFusedState futureState =
            new PoseEstimatorSubsystem.PredictedFusedState();
    private final BallTrajectoryLookup.MovingShotSolution baseSolution =
            new BallTrajectoryLookup.MovingShotSolution();
    private final BallTrajectoryLookup.MovingShotSolution fixedAngleSolution =
            new BallTrajectoryLookup.MovingShotSolution();
    private final DoublePublisher targetDistanceInchesPublisher;
    private final DoublePublisher hoodAngleDegreesPublisher;

    private double hoodTrimDegrees;

    public TestShootingCommand(
            PoseEstimatorSubsystem poseEstimator,
            SparkAnglePositionSubsystem horizontalAim,
            SparkAnglePositionSubsystem verticalAim,
            ShooterSubsystem shooter) {
        this.poseEstimator = Objects.requireNonNull(poseEstimator, "poseEstimator must not be null");
        this.horizontalAim = Objects.requireNonNull(horizontalAim, "horizontalAim must not be null");
        this.verticalAim = Objects.requireNonNull(verticalAim, "verticalAim must not be null");
        this.shooter = Objects.requireNonNull(shooter, "shooter must not be null");

        NetworkTable testShootingTable = NetworkTableInstance.getDefault().getTable("testShootingCommand");
        targetDistanceInchesPublisher = testShootingTable.getDoubleTopic("targetDistanceInches").publish();
        hoodAngleDegreesPublisher = testShootingTable.getDoubleTopic("hoodAngleDegrees").publish();

        addRequirements(this.horizontalAim, this.verticalAim, this.shooter);
    }

    @Override
    public void initialize() {
        hoodTrimDegrees = 0.0;
    }

    @Override
    public void execute() {
        if (!poseEstimator.getPredictedFusedState(
                ShooterConstants.COMMANDED_SHOOTER_LOOKAHEAD_SECONDS,
                futureState)) {
            clearOutputs();
            return;
        }

        Translation2d target = getTarget();
        double targetDistanceInches = Inches.convertFrom(
                target.getDistance(new Translation2d(futureState.xMeters, futureState.yMeters)),
                Meters);
        targetDistanceInchesPublisher.set(targetDistanceInches);

        boolean hasBaseSolution = BallTrajectoryLookup.solveMovingShot(
                verticalAim.getMinimumAngle().in(Degrees),
                verticalAim.getMaximumAngle().in(Degrees),
                ShooterConstants.COMMANDED_MOVING_SHOT_HOOD_SEARCH_STEP_DEGREES,
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
                futureState.headingRadians,
                horizontalAim.getMinimumAngle().in(Degrees),
                horizontalAim.getMaximumAngle().in(Degrees),
                baseSolution);
        if (!hasBaseSolution) {
            clearOutputs();
            return;
        }

        double desiredHoodAngleDegrees = clampHoodAngle(baseSolution.getHoodAngleDegrees() + hoodTrimDegrees);
        hoodTrimDegrees = desiredHoodAngleDegrees - baseSolution.getHoodAngleDegrees();
        verticalAim.setAngle(Degrees.of(desiredHoodAngleDegrees));
        hoodAngleDegreesPublisher.set(desiredHoodAngleDegrees);

        boolean hasFixedAngleSolution = BallTrajectoryLookup.solveMovingShot(
                desiredHoodAngleDegrees,
                desiredHoodAngleDegrees,
                HOOD_TRIM_STEP_DEGREES,
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
                futureState.headingRadians,
                horizontalAim.getMinimumAngle().in(Degrees),
                horizontalAim.getMaximumAngle().in(Degrees),
                fixedAngleSolution);

        if (hasFixedAngleSolution) {
            horizontalAim.setAngle(Degrees.of(clampTurretDeltaDegrees(fixedAngleSolution.getTurretDeltaDegrees())));
            shooter.setCoupledIPS(fixedAngleSolution.getFlywheelCommandIps());
            return;
        }

        horizontalAim.setAngle(Degrees.of(clampTurretDeltaDegrees(baseSolution.getTurretDeltaDegrees())));
        shooter.setCoupledIPS(0.0);
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
        hoodTrimDegrees -= HOOD_TRIM_STEP_DEGREES;
    }

    public void trimHoodDown() {
        hoodTrimDegrees += HOOD_TRIM_STEP_DEGREES;
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

    private void clearOutputs() {
        shooter.setCoupledIPS(0.0);
        horizontalAim.setAngle(Degrees.of(0.0));
        targetDistanceInchesPublisher.set(Double.NaN);
        hoodAngleDegreesPublisher.set(Double.NaN);
    }
}
