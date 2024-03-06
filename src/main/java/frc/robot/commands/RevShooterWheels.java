package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class RevShooterWheels extends Command {
    private ShooterSubsystem shooterSubsystem;
    public double speed;

    public RevShooterWheels(ShooterSubsystem shooterSubsystem, double speed) {
        this.shooterSubsystem = shooterSubsystem;
        this.speed = speed;
    }

    @Override
    public void execute() {
        if (DriverStation.isAutonomous()) {
            shooterSubsystem.bottomRevMotor.set(-1.0);
            shooterSubsystem.topRevMotor.set(1.0);
        } else {
            shooterSubsystem.bottomRevMotor.set(-speed);
            shooterSubsystem.topRevMotor.set(speed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.topRevMotor.set(0);
        shooterSubsystem.bottomRevMotor.set(0);
    }
}
