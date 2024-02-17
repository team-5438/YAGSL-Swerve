package frc.robot.commands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimbCommand extends Command {
    public ClimberSubsystem climberSubsystem;
    public PS4Controller operator;

    public ClimbCommand(ClimberSubsystem climberSubsystem, PS4Controller operator) {
        this.climberSubsystem = climberSubsystem;
        this.operator = operator;
    } 
    
    @Override
    public void execute() {
        switch (operator.getPOV()) {
            case -1:
                climberSubsystem.RClimber.set(0);
                climberSubsystem.LClimber.set(0);
                break;
            case 0:
                climberSubsystem.RClimber.set(-1);
                climberSubsystem.LClimber.set(0);
                break;
            case 90:
                climberSubsystem.RClimber.set(1);
                climberSubsystem.LClimber.set(0);
                break;
            case 180:
                climberSubsystem.LClimber.set(-1);
                climberSubsystem.RClimber.set(0);
                break;
            case 270:
                climberSubsystem.LClimber.set(1);
                climberSubsystem.RClimber.set(0);
                break;
        }
    }
}