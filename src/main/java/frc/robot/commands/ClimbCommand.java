package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimbCommand extends Command {
    public ClimberSubsystem climberSubsystem;
    private int foo;

    public ClimbCommand(ClimberSubsystem climberSubsystem, int foo) {

        this.climberSubsystem = climberSubsystem;
        this.foo = foo;

    } 

    public static void climb(int foo, ClimberSubsystem climberSubsystem) {
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
    }
    @Override
    public void execute() {
        climb(foo, climberSubsystem);
    }
}
