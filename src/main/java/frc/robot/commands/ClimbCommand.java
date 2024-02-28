package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimbCommand extends Command {
    public ClimberSubsystem climberSubsystem;
    public int direction;

    public ClimbCommand(ClimberSubsystem climberSubsystem, int direction) {
        this.climberSubsystem = climberSubsystem;
        this.direction = direction;
    } 

    @Override
    public void execute() {
        climberSubsystem.LClimber.set(direction);
        climberSubsystem.RClimber.set(direction);
    }

    @Override
    public void end(boolean isFinished) {
        climberSubsystem.LClimber.set(0);
        climberSubsystem.RClimber.set(0);
    }
}
