package frc.robot.Commands;

import static edu.wpi.first.units.Units.Degrees;

import java.util.Objects;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.KrakenFlywheelSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SparkAnglePositionSubsystem;

/**
 * Manual component pre-check for hood, turret, shooter, and kicker without driving.
 */
public class PreCheck extends Command {
    private static final double AXIS_DEADBAND = 0.10;
    private static final double PRECHECK_INTAKE_DRIVE_IPS = 300.0;
    private static final double MAX_SHOOTER_COMMAND_IPS =
            ShooterConstants.COMMANDED_FLYWHEEL_SET_IPS[ShooterConstants.COMMANDED_FLYWHEEL_SET_IPS.length - 1];
    private static final double MAX_KICKER_COMMAND_IPS = MAX_SHOOTER_COMMAND_IPS;

    private final SparkAnglePositionSubsystem horizontalAim;
    private final SparkAnglePositionSubsystem verticalAim;
    private final ShooterSubsystem shooter;
    private final KrakenFlywheelSubsystem intakeDrive;
    private final CommandXboxController controller;

    public PreCheck(
            SparkAnglePositionSubsystem horizontalAim,
            SparkAnglePositionSubsystem verticalAim,
            ShooterSubsystem shooter,
            KrakenFlywheelSubsystem intakeDrive,
            CommandXboxController controller) {
        this.horizontalAim = Objects.requireNonNull(horizontalAim, "horizontalAim must not be null");
        this.verticalAim = Objects.requireNonNull(verticalAim, "verticalAim must not be null");
        this.shooter = Objects.requireNonNull(shooter, "shooter must not be null");
        this.intakeDrive = Objects.requireNonNull(intakeDrive, "intakeDrive must not be null");
        this.controller = Objects.requireNonNull(controller, "controller must not be null");

        addRequirements(this.horizontalAim, this.verticalAim, this.shooter, this.intakeDrive);
    }

    @Override
    public void execute() {
        double desiredHoodAngleDegrees = mapAxisToRange(
                controller.getRightY(),
                verticalAim.getMinimumAngle().in(Degrees),
                verticalAim.getMaximumAngle().in(Degrees));
        double desiredHorizontalAimDegrees = mapAxisToRange(
                controller.getRightX(),
                horizontalAim.getMinimumAngle().in(Degrees),
                horizontalAim.getMaximumAngle().in(Degrees));

        verticalAim.setAngle(Degrees.of(desiredHoodAngleDegrees));
        horizontalAim.setAngle(Degrees.of(desiredHorizontalAimDegrees));

        double shooterCommandIps = MAX_SHOOTER_COMMAND_IPS
                * MathUtil.applyDeadband(controller.getLeftTriggerAxis(), AXIS_DEADBAND);
        double kickerTriggerCommand = MathUtil.applyDeadband(controller.getRightTriggerAxis(), AXIS_DEADBAND);
        double kickerCommandIps = kickerTriggerCommand > 0.0
                ? MAX_KICKER_COMMAND_IPS * kickerTriggerCommand
                : -ShooterSubsystem.KICKER_IDLE_REVERSE_MAGNITUDE_IPS;
        shooter.setIPS(shooterCommandIps, kickerCommandIps, shooterCommandIps);
        intakeDrive.setIPS(controller.a().getAsBoolean() ? PRECHECK_INTAKE_DRIVE_IPS : 0.0);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setIPS(0.0, 0.0, 0.0);
        intakeDrive.setIPS(0.0);
    }

    private static double mapAxisToRange(double axisValue, double minimumValue, double maximumValue) {
        double normalizedAxisValue = MathUtil.applyDeadband(axisValue, AXIS_DEADBAND);
        double positionFraction = 0.5 * (normalizedAxisValue + 1.0);
        return MathUtil.clamp(
                minimumValue + positionFraction * (maximumValue - minimumValue),
                minimumValue,
                maximumValue);
    }
}
