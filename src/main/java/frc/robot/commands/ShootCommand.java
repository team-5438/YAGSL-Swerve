
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command{
    public ShooterSubsystem shooterSubsystem;
    public IntakeSubsystem intakeSubsystem;

    public ShootCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void execute() {
        shooterSubsystem.feedMotor.set(0.5);
        intakeSubsystem.intakeMotor.set(0.90);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.feedMotor.set(0);
        intakeSubsystem.intakeMotor.set(0);
    }
}

