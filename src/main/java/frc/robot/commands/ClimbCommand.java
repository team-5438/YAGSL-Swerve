package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimbCommand extends Command {
    public ClimberSubsystem climberSubsystem;

    public ClimbCommand(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
    } 

    public Command climb(int foo, ClimberSubsystem climberSubsystem) {
        switch (foo) {
            case 0:
                climberSubsystem.RClimber.set(-1);
                break;
            case 90:
                climberSubsystem.RClimber.set(1);
                break;
            case 180:
                climberSubsystem.LClimber.set(-1);
                break;
            case 270:
                climberSubsystem.LClimber.set(1);
                break;
        }
        return null;
    }
    @Override
    public void execute() {
    }
}
