package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeCommandReverse extends Command{
    public ShooterSubsystem shooterSubsystem;
    public IntakeSubsystem intakeSubsystem;
    public int[][] green = {{0, 255, 0, 100}, {0, 0, 0, 100},{0, 255, 0, 100},{0, 0, 0, 100},{0, 255, 0, 100}, {0, 0, 0, 0}};

    public IntakeCommandReverse(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void execute() {
        shooterSubsystem.feedMotor.set(-0.1);
        intakeSubsystem.intakeMotor.set(-1);
    }
}