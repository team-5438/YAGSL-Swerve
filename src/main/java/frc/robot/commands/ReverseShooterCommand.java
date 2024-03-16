package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ReverseShooterCommand extends Command {
    ShooterSubsystem shooterSubsystem;

    public ReverseShooterCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        shooterSubsystem.topRevMotor.set(-0.2);
        shooterSubsystem.bottomRevMotor.set(0.2);
        shooterSubsystem.feedMotor.set(-0.2);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.topRevMotor.set(0);
        shooterSubsystem.bottomRevMotor.set(0);
        shooterSubsystem.feedMotor.set(0);
    }
}
