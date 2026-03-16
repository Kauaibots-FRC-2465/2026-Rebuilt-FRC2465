package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.helpers.KrakenFlywheelSubsystem;



public class FlywheelSubsystem extends SubsystemBase {
    private final KrakenFlywheelSubsystem mainFlywheel;
    private final KrakenFlywheelSubsystem backspinFlywheel;
    private final KrakenFlywheelSubsystem kicker;
    private double threshold;


    public FlywheelSubsystem(KrakenFlywheelSubsystem mainFlywheel, KrakenFlywheelSubsystem backspinFlywheel, KrakenFlywheelSubsystem kicker) {
        this.mainFlywheel = mainFlywheel;
        this.backspinFlywheel = backspinFlywheel;
        this.kicker = kicker;
    }

    public Command requestSpeed(double speed) {
        return Commands.parallel(mainFlywheel.cmdSetIPS(()->speed), backspinFlywheel.cmdSetIPS(()->speed), run(
            () -> {
                
            }

        ));
    }

    public Command requestSpeedFrom(DoubleSupplier speed) {
        return Commands.parallel(mainFlywheel.cmdSetIPS(speed), backspinFlywheel.cmdSetIPS(speed));
    }
}
