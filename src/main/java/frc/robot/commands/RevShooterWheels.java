package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class RevShooterWheels extends Command {
    private ShooterSubsystem shooterSubsystem;

    public RevShooterWheels(ShooterSubsystem _shooterSubsystem) {
        shooterSubsystem = _shooterSubsystem;
    }

    public void execute() {
        shooterSubsystem.speakerMotorTop.set(Constants.Shooter.shooterRevSpeed);
        shooterSubsystem.speakerMotorBottom.set(Constants.Shooter.shooterRevSpeed);
    }
}
