package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class RevShooterWheels extends Command {
    private ShooterSubsystem shooterSubsystem;
    public double speed;

    public RevShooterWheels(ShooterSubsystem _shooterSubsystem, double speed) {
        shooterSubsystem = _shooterSubsystem;
        this.speed = speed;
    }

    @Override
    public void execute() {
        if (DriverStation.isAutonomous()) {
            shooterSubsystem.speakerMotorBottom.set(-1.0);
            shooterSubsystem.speakerMotorTop.set(1.0);
        } else {
            shooterSubsystem.speakerMotorBottom.set(-speed);
            shooterSubsystem.speakerMotorTop.set(speed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.speakerMotorTop.set(0);
        shooterSubsystem.speakerMotorBottom.set(0);
    }
}
