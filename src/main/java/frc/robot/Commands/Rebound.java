package frc.robot.Commands;

import static edu.wpi.first.units.Units.Degrees;

import java.util.Objects;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePositionSubsystem;
import frc.robot.subsystems.KrakenFlywheelSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SparkAnglePositionSubsystem;

/**
 * Holds the robot in a fixed rebound configuration.
 */
public class Rebound extends Command {
    private final SparkAnglePositionSubsystem horizontalAim;
    private final SparkAnglePositionSubsystem verticalAim;
    private final ShooterSubsystem shooter;
    private final IntakePositionSubsystem intakePosition;
    private final KrakenFlywheelSubsystem intakeDrive;

    public Rebound(
            SparkAnglePositionSubsystem horizontalAim,
            SparkAnglePositionSubsystem verticalAim,
            ShooterSubsystem shooter,
            IntakePositionSubsystem intakePosition,
            KrakenFlywheelSubsystem intakeDrive) {
        this.horizontalAim = Objects.requireNonNull(horizontalAim, "horizontalAim must not be null");
        this.verticalAim = Objects.requireNonNull(verticalAim, "verticalAim must not be null");
        this.shooter = Objects.requireNonNull(shooter, "shooter must not be null");
        this.intakePosition = Objects.requireNonNull(intakePosition, "intakePosition must not be null");
        this.intakeDrive = Objects.requireNonNull(intakeDrive, "intakeDrive must not be null");

        addRequirements(
                this.horizontalAim,
                this.verticalAim,
                this.shooter,
                this.intakePosition,
                this.intakeDrive);
    }

    @Override
    public void execute() {
        horizontalAim.setAngle(Degrees.of(ShooterConstants.COMMANDED_REBOUND_HORIZONTAL_AIM_DEGREES));
        verticalAim.setAngle(Degrees.of(ShooterConstants.COMMANDED_REBOUND_HOOD_ANGLE_DEGREES));
        shooter.setCoupledIPS(ShooterConstants.COMMANDED_REBOUND_SHOOTER_SPEED_IPS);
        intakePosition.setAngle(Degrees.of(ShooterConstants.COMMANDED_REBOUND_INTAKE_ANGLE_DEGREES));
        intakeDrive.setIPS(ShooterConstants.COMMANDED_REBOUND_INTAKE_DRIVE_SPEED_IPS);
    }
}
