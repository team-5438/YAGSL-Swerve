
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command{
    public ShooterSubsystem shooterSubsystem;

    public ShootCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void execute() {
        shooterSubsystem.feedMotor.set(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.feedMotor.set(0);
    }
}

