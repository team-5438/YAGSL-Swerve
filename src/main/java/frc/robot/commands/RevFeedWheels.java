package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class RevFeedWheels extends Command {
    private ShooterSubsystem shooterSubsystem;

    public RevFeedWheels(ShooterSubsystem _shooterSubsystem) {
        shooterSubsystem = _shooterSubsystem;
    }

    @Override
    public void execute() {
        shooterSubsystem.shooterFeed.set(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.shooterFeed.set(0);
    }
}