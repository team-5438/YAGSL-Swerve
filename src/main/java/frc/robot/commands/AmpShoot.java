package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AmpShoot extends Command {
    private AmpSubsystem ampSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private double speed;

    public AmpShoot(AmpSubsystem ampSubsystem, ShooterSubsystem shooterSubsystem, double speed) {
        this.ampSubsystem = ampSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.speed = speed;
    }

    @Override
    public void execute() {
        System.out.println("AMP SHOOTING");
        ampSubsystem.ampShootMotor.set(speed);
        if (speed > 0) {
            shooterSubsystem.speakerMotorTop.set(0.2);
            shooterSubsystem.speakerMotorBottom.set(-0.2);
        }
    }

    @Override
    public void end(boolean interrupted) {
        ampSubsystem.ampShootMotor.set(0);
        shooterSubsystem.speakerMotorTop.set(0);
        shooterSubsystem.speakerMotorBottom.set(0);
    }
}