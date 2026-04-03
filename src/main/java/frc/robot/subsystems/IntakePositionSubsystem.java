package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import java.util.Objects;
import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OverrideCommand;

/**
 * Aggregate subsystem for the paired intake angle actuators.
 *
 * <p>This subsystem owns the two underlying TalonFX angle subsystems so intake
 * aim can be scheduled as one coordinated mechanism.
 */
public class IntakePositionSubsystem extends SubsystemBase {
    private final KrakenAnglePositionSubsystem leftIntakePosition;
    private final KrakenAnglePositionSubsystem rightIntakePosition;

    /**
     * Creates a coordinated intake-aim subsystem.
     *
     * <p>The shared motion parameters are applied to both sides while each side
     * retains its own CAN identity, offset, and inversion.
     */
    public IntakePositionSubsystem(
            int leftCanId,
            String leftMotorName,
            double leftFeedbackRotorOffset,
            boolean leftMotorReversed,
            int rightCanId,
            String rightMotorName,
            double rightFeedbackRotorOffset,
            boolean rightMotorReversed,
            String canBusName,
            double mechanismGearRatio,
            double kP,
            double peakCurrent,
            double maxPositionVoltage,
            Angle minimumAngle,
            Angle maximumAngle) {
        leftIntakePosition = new KrakenAnglePositionSubsystem(
                leftCanId,
                canBusName,
                leftMotorName,
                mechanismGearRatio,
                leftFeedbackRotorOffset,
                kP,
                peakCurrent,
                maxPositionVoltage,
                minimumAngle,
                maximumAngle,
                leftMotorReversed);

        rightIntakePosition = new KrakenAnglePositionSubsystem(
                rightCanId,
                canBusName,
                rightMotorName,
                mechanismGearRatio,
                rightFeedbackRotorOffset,
                kP,
                peakCurrent,
                maxPositionVoltage,
                minimumAngle,
                maximumAngle,
                rightMotorReversed);
    }

    /**
     * Commands both intake angle actuators to the same target angle.
     */
    public void setAngle(Angle angle) {
        leftIntakePosition.setAngle(angle);
        rightIntakePosition.setAngle(angle);
    }

    /**
     * Returns a command that continuously drives both intake angle actuators to
     * the supplied target angle.
     */
    public Command cmdSetAngle(Supplier<Angle> angleSupplier) {
        Objects.requireNonNull(angleSupplier, "angleSupplier must not be null");

        return new OverrideCommand(this) {
            @Override
            public void execute() {
                setAngle(angleSupplier.get());
            }
        };
    }

    /**
     * Returns the average of the two current mechanism angles.
     */
    public Angle getAngle() {
        return Rotations.of(
                (leftIntakePosition.getAngle().in(Rotations)
                        + rightIntakePosition.getAngle().in(Rotations))
                        / 2.0);
    }

    public void recoverIfResetOccurred() {
        leftIntakePosition.recoverIfResetOccurred();
        rightIntakePosition.recoverIfResetOccurred();
    }
}
