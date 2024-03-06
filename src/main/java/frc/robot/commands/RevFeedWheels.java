package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RevFeedWheels extends Command {
    private ShooterSubsystem shooterSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private double previousVoltage;
    private boolean tillVoltage;

    public RevFeedWheels(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, boolean tillVoltage) {
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.tillVoltage = tillVoltage;
    }

    @Override
    public void execute() {
        shooterSubsystem.feedMotor.set(0.5);
        shooterSubsystem.feedVoltage = intakeSubsystem.getVoltage(shooterSubsystem.feedMotor);
        System.out.println("Voltage: " + shooterSubsystem.feedVoltage);
    }

    @Override
    public boolean isFinished() {
        if(tillVoltage) {
            if(shooterSubsystem.feedVoltage != 0.0) {
                if(shooterSubsystem.feedVoltage < previousVoltage - 0.2) {
                    return true;
                } else {
                    previousVoltage = shooterSubsystem.feedVoltage;
                    return false;
                }
            } else {
                return false;
            }
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.feedMotor.set(0);
    }
}