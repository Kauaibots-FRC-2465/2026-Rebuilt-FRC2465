package frc.robot.utility;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class OverrideCommand extends Command {

  public OverrideCommand(Subsystem... requirements) {
    addRequirements(requirements);
  }
}