package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AmpPreset extends Command {
    AmpSubsystem ampSubsystem;
    ShooterSubsystem shooterSubsystem;
    double encoderSetPoint;
    boolean endAtAngle;
    double shooterAngle = 0.118;

    public AmpPreset(double encoderSetPoint, AmpSubsystem ampSubsystem, ShooterSubsystem shooterSubsystem, boolean endAtAngle) {
        // this.addRequirements(ampSubsystem);
        this.addRequirements(shooterSubsystem);
        this.ampSubsystem = ampSubsystem;
        this.encoderSetPoint = encoderSetPoint;
        this.shooterSubsystem = shooterSubsystem;
        this.endAtAngle = endAtAngle;
    }

    @Override
    public void execute() {
        double shooterSpeed = shooterSubsystem.pivotPIDControllerAuto.calculate(shooterSubsystem.fixEncoderAngle(shooterSubsystem.pivotEncoder.getDistance()), shooterAngle);
        // double ampSpeed = -ampSubsystem.ampPivotPIDController.calculate(ampSubsystem.ampPivotEncoder.getPosition(), encoderSetPoint);
        // sp += ampSubsystem.ampFeedforward.calculate(ampSubsystem.ampPivotEncoder.getPosition(), sp);
        // ampSubsystem.ampPivotMotor.set(ampSpeed);
        shooterSubsystem.speakerMotorPivot.set(shooterSpeed);
    }

    public boolean isFinished() {
        if(endAtAngle) {
            return Math.abs(shooterAngle - -shooterSubsystem.pivotEncoder.getDistance()) < 0.01;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.speakerMotorPivot.set(0);
    }
}
