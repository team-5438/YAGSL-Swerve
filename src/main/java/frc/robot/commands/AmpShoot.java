package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AmpShoot extends Command {
    private AmpSubsystem ampSubsystem;
    private ShooterSubsystem shooterSubsystem;

    public AmpShoot(AmpSubsystem ampSubsystem, ShooterSubsystem shooterSubsystem) {
        this.ampSubsystem = ampSubsystem;
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void execute() {
        ampSubsystem.ampShootMotor.set(1);
        shooterSubsystem.speakerMotorTop.set(0.2);
        shooterSubsystem.speakerMotorBottom.set(-0.2);
    }

    @Override
    public void end(boolean interrupted) {
        ampSubsystem.ampShootMotor.set(0);
        shooterSubsystem.speakerMotorTop.set(0);
        shooterSubsystem.speakerMotorBottom.set(0);
    }
}