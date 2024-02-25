package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class RevShooterWheels extends Command {
    private ShooterSubsystem shooterSubsystem;

    public RevShooterWheels(ShooterSubsystem _shooterSubsystem) {
        shooterSubsystem = _shooterSubsystem;
    }

    @Override
    public void execute() {
        shooterSubsystem.speakerMotorTop.set(Constants.Shooter.shooterRevSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.speakerMotorTop.set(0);
    }
}
