package frc.robot.Commands;

import static edu.wpi.first.units.Units.Degrees;

import java.util.Objects;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SparkAnglePositionSubsystem;

/**
 * Manual component pre-check for hood, turret, shooter, and kicker without driving.
 */
public class PreCheck extends Command {
    private static final double AXIS_DEADBAND = 0.10;
    private static final double HOOD_RATE_DEGREES_PER_SECOND = 20.0;
    private static final double HORIZONTAL_AIM_RATE_DEGREES_PER_SECOND = 30.0;
    private static final double MAX_SHOOTER_COMMAND_IPS =
            ShooterConstants.COMMANDED_FLYWHEEL_SET_IPS[ShooterConstants.COMMANDED_FLYWHEEL_SET_IPS.length - 1];
    private static final double MAX_KICKER_COMMAND_IPS = MAX_SHOOTER_COMMAND_IPS;

    private final SparkAnglePositionSubsystem horizontalAim;
    private final SparkAnglePositionSubsystem verticalAim;
    private final ShooterSubsystem shooter;
    private final CommandXboxController controller;
    private double desiredHorizontalAimDegrees;
    private double desiredHoodAngleDegrees;
    private double previousTimestampSeconds;

    public PreCheck(
            SparkAnglePositionSubsystem horizontalAim,
            SparkAnglePositionSubsystem verticalAim,
            ShooterSubsystem shooter,
            CommandXboxController controller) {
        this.horizontalAim = Objects.requireNonNull(horizontalAim, "horizontalAim must not be null");
        this.verticalAim = Objects.requireNonNull(verticalAim, "verticalAim must not be null");
        this.shooter = Objects.requireNonNull(shooter, "shooter must not be null");
        this.controller = Objects.requireNonNull(controller, "controller must not be null");

        addRequirements(this.horizontalAim, this.verticalAim, this.shooter);
    }

    @Override
    public void initialize() {
        desiredHorizontalAimDegrees = horizontalAim.getAngle().in(Degrees);
        desiredHoodAngleDegrees = verticalAim.getAngle().in(Degrees);
        previousTimestampSeconds = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        double nowSeconds = Timer.getFPGATimestamp();
        double dtSeconds = MathUtil.clamp(nowSeconds - previousTimestampSeconds, 0.0, 0.1);
        previousTimestampSeconds = nowSeconds;

        double hoodRateCommand = MathUtil.applyDeadband(controller.getRightY(), AXIS_DEADBAND);
        double horizontalAimRateCommand = MathUtil.applyDeadband(-controller.getRightX(), AXIS_DEADBAND);
        desiredHoodAngleDegrees = MathUtil.clamp(
                desiredHoodAngleDegrees + hoodRateCommand * HOOD_RATE_DEGREES_PER_SECOND * dtSeconds,
                verticalAim.getMinimumAngle().in(Degrees),
                verticalAim.getMaximumAngle().in(Degrees));
        desiredHorizontalAimDegrees = MathUtil.clamp(
                desiredHorizontalAimDegrees + horizontalAimRateCommand * HORIZONTAL_AIM_RATE_DEGREES_PER_SECOND * dtSeconds,
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
    }
}
