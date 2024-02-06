package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimbCommand extends Command {
    public ClimberSubsystem climberSubsystem;

    public ClimbCommand(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
    }

    @Override
    public void execute() {
        climberSubsystem.LClimber.set(1);
        climberSubsystem.RClimber.set(1);
    }
}
